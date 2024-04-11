#include <chrono>
// #include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdarg.h>
#include "protocol_nmea.h"
#include "device_runtime_tests.h"
#include "ISFileManager.h"


#define LOG_DIRECTORY   "realtime_logs"

DeviceRuntimeTests::DeviceRuntimeTests()
{
    m_hist.gps1Pos.leapS = C_GPS_LEAP_SECONDS;

    ISFileManager::CreateDirectory(LOG_DIRECTORY);
    m_filename = CreateLogFilename(LOG_DIRECTORY);
}

std::string DeviceRuntimeTests::CreateLogFilename(const std::string path)
{
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // Convert to local time
    std::tm bt = *localtime(&in_time_t);

    // Create a stringstream and format time
    std::stringstream ss;
    ss << std::put_time(&bt, "%Y%m%d_%H%M%S"); // YYYYMMDD_HHMMSS format

    // Construct filename
    std::string filename = path + "/" + ss.str() + ".txt"; // or any other extension

    return filename;
}

std::string charArrayToHex(uint8_t* arr, int arrSize) 
{
    std::stringstream ss;
    ss << "0x" << std::hex << std::setfill('0'); // Set to output hex values, padded with 0

    for (int i = 0; i < arrSize; ++i) 
    {
        // Static cast to unsigned int to handle negative char values correctly
        ss << std::setw(2) << static_cast<unsigned int>(arr[i]);
    }

    return ss.str();
}

void DeviceRuntimeTests::ProcessParseError(is_comm_instance_t &comm)
{
    std::string parser;
    switch (comm.rxBuf.head[0])
    {
    case PSC_ISB_PREAMBLE_BYTE1:    parser = "ISB";     break;
    case PSC_NMEA_START_BYTE:       parser = "NMEA";    break;
    case UBLOX_START_BYTE1:         parser = "UBX";     break;
    case RTCM3_START_BYTE:          parser = "RTCM3";   break;
    case SPARTN_START_BYTE:         parser = "SPARTN";  break;
    case SONY_START_BYTE:           parser = "SONY";    break;
    default:                        parser = charArrayToHex(comm.rxBuf.head, 4);    break;
    }
    
    m_errorCount.parse = comm.rxErrorCount;
    LogEvent("Parse error #%d: %s size %d\n", comm.rxErrorCount, parser.c_str(), comm.rxBuf.scanPrior - comm.rxBuf.head);
}

void DeviceRuntimeTests::ProcessISB(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf)
{
    if (!m_enable)
    {
        return;
    }
    
    // printf("ISB: ID %d  Size %d\n", dataHdr.id, dataHdr.size);

    switch(dataHdr.id)
    {
    case DID_GPS1_POS:      copyDataPToStructP2(&m_hist.gps1Pos, &dataHdr, dataBuf, sizeof(gps_pos_t));       break;
    }
}

void DeviceRuntimeTests::ProcessNMEA(const uint8_t* msg, int msgSize)
{
    if (!m_enable)
    {
        return;
    }

    // printf("NMEA (%d): %.*s", msgSize, msgSize, msg);
    
    int id = getNmeaMsgId(msg, msgSize);
    switch(id)
    {
    case NMEA_MSG_ID_GxGGA:     TestNmeaGga(msg, msgSize);      break;
    case NMEA_MSG_ID_GxZDA:     TestNmeaZda(msg, msgSize);      break;
    }
}

void DeviceRuntimeTests::TestNmeaGga(const uint8_t* msg, int msgSize)
{
    msg_history_t &hist = m_hist.nmea.gga;
    gps_pos_t gpsPos = {};

    utc_time_t t;
    int utcWeekday = gpsTowMsToUtcWeekday(gpsPos.timeOfWeekMs, m_hist.gps1Pos.leapS);
    nmea_parse_gga((const char *)msg, msgSize, gpsPos, t, utcWeekday);

    // printf("NMEA GGA (%d ms, %d wkday): %.*s", gpsPos.timeOfWeekMs, utcWeekday, msgSize, msg);

    if (CheckGpsDuplicate("NEA GGA Error", m_errorCount.nmeaGgaTime, gpsPos.timeOfWeekMs, gpsPos.week, hist))
    {
        LogEvent("  1: %.*s  2: %.*s", hist.msgSize, hist.msg, msgSize, msg);
    }
    if (CheckGpsTimeReverse("NEA GGA Error", m_errorCount.nmeaGgaTime, gpsPos.timeOfWeekMs, gpsPos.week, hist))
    {
        LogEvent("  1: %.*s  2: %.*s", hist.msgSize, hist.msg, msgSize, msg);
    }

    // Update history
    hist.update(gpsPos.timeOfWeekMs, gpsPos.week, (uint8_t*)msg, msgSize);
}

void DeviceRuntimeTests::TestNmeaZda(const uint8_t* msg, int msgSize)
{
    msg_history_t &hist = m_hist.nmea.zda;
    uint32_t gpsTowMs;
    uint32_t gpsWeek;
    utc_date_t utcDate; 
    utc_time_t utcTime;
    int leapS;
    nmea_parse_zda((char*)msg, msgSize, gpsTowMs, gpsWeek, utcDate, utcTime, C_GPS_LEAP_SECONDS);

    // printf("NMEA ZDA (%d ms): %.*s", gpsTowMs, msgSize, msg);

    if (CheckGpsDuplicate("NMEA ZDA Error", m_errorCount.nmeaZdaTime, gpsTowMs, gpsWeek, hist))
    {
        LogEvent("  1: %.*s  2: %.*s", hist.msgSize, hist.msg, msgSize, msg);
    }
    if (CheckGpsTimeReverse("NMEA ZDA Error", m_errorCount.nmeaZdaTime, gpsTowMs, gpsWeek, hist))
    {
        LogEvent("  1: %.*s  2: %.*s", hist.msgSize, hist.msg, msgSize, msg);
    }

    // Update history
    hist.update(gpsTowMs, gpsWeek, (uint8_t*)msg, msgSize);
}

bool DeviceRuntimeTests::CheckGpsDuplicate(const char* description, int &count, uint32_t towMs, uint32_t gpsWeek, msg_history_t &hist)
{
    int toyMs = towMs + gpsWeek * C_MILLISECONDS_PER_WEEK;
    int histToyMs = hist.gpsTowMs + hist.gpsWeek * C_MILLISECONDS_PER_WEEK;

    if (toyMs == histToyMs)
    {   // Duplicate time
        LogEvent("%s: Duplicate time (#%d): %d ms %d week >> %d ms %d week\n", description, ++count, hist.gpsTowMs, hist.gpsWeek, towMs, gpsWeek);
        return true;
    }

    return false;
}

bool DeviceRuntimeTests::CheckGpsTimeReverse(const char* description, int &count, uint32_t towMs, uint32_t gpsWeek, msg_history_t &hist)
{
    int toyMs = towMs + gpsWeek * C_MILLISECONDS_PER_WEEK;
    int histToyMs = hist.gpsTowMs + hist.gpsWeek * C_MILLISECONDS_PER_WEEK;

    if (toyMs < histToyMs)
    {   // Reversed time
        LogEvent("%s: Reversed time (#%d): %d ms %d week >> %d ms %d week\n", description, ++count, hist.gpsTowMs, hist.gpsWeek, towMs, gpsWeek);
        return true;
    }

    return false;
}

void DeviceRuntimeTests::LogEvent(const char *format, ...)
{
#if 0
    va_list args1;
    va_start(args1, format);
    vprintf(format, args1);
    va_end(args1);              // Clean up the variable arguments list
#endif

    FILE *file = fopen(m_filename.c_str(), "a");
    if (file != NULL)
    {
        va_list args2;
        va_start(args2, format);
        fprintf(file, "NMEA Error: ");
        vfprintf(file, format, args2);
        fclose(file);
        va_end(args2);          // Clean up the variable arguments list
    }
}

