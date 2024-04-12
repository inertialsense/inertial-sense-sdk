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

    LogEvent("Realtime tests started...\n");
}

std::string DeviceRuntimeTests::CreateLogFilename(const std::string path, int serialNumber)
{
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // Convert to local time
    std::tm bt = *localtime(&in_time_t);

    // Create a stringstream filename
    std::stringstream ss;
    ss << "SN" << serialNumber << "_" << std::put_time(&bt, "%Y%m%d_%H%M%S") << ".txt"; // YYYYMMDD_HHMMSS format

    // Construct filename
    std::string filename = path + "/" + ss.str();

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
    // default:                        parser = charArrayToHex(comm.rxBuf.head, 4);    break;
    default:                        parser = "Unknown"; break;
    }
    
    m_errorCount.parse = comm.rxErrorCount;
    LogEvent("Parse error #%d: %s, size %d", comm.rxErrorCount, parser.c_str(), comm.rxBuf.scanPrior - comm.rxBuf.head);
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
    case DID_DEV_INFO: {
        int serialNumber = m_devInfo.serialNumber;     
        copyDataPToStructP2(&m_devInfo, &dataHdr, dataBuf, sizeof(dev_info_t));
        if (serialNumber != m_devInfo.serialNumber)
        {   // Serial number changed.  Update log filename.
            m_filename = CreateLogFilename(LOG_DIRECTORY, m_devInfo.serialNumber);
        }
    }
        break;
    case DID_GPS1_POS:      copyDataPToStructP2(&m_hist.gps1Pos, &dataHdr, dataBuf, sizeof(gps_pos_t));     break;
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

    CheckGpsDuplicate("NEA GGA Error",   m_errorCount.nmeaGgaTime, gpsPos.timeOfWeekMs, gpsPos.week, msg, msgSize, hist);
    CheckGpsTimeReverse("NEA GGA Error", m_errorCount.nmeaGgaTime, gpsPos.timeOfWeekMs, gpsPos.week, msg, msgSize, hist);

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

    printf("NMEA ZDA (%d ms): %.*s", gpsTowMs, msgSize, msg);

    CheckGpsDuplicate("NMEA ZDA Error",   m_errorCount.nmeaZdaTime, gpsTowMs, gpsWeek, msg, msgSize, hist);
    CheckGpsTimeReverse("NMEA ZDA Error", m_errorCount.nmeaZdaTime, gpsTowMs, gpsWeek, msg, msgSize, hist);

    // Update history
    hist.update(gpsTowMs, gpsWeek, (uint8_t*)msg, msgSize);
}

std::string printfToString(const char* format, ...) 
{
    va_list args;
    va_start(args, format);

    // Start with a size that might be large enough
    std::string str(100, '\0');

    // Attempt to write to the string's buffer
    int needed = std::vsnprintf(&str[0], str.size(), format, args);

    // If the string was not big enough, resize and try again
    if (needed >= str.size()) 
    {
        str.resize(needed + 1);
        std::vsnprintf(&str[0], str.size(), format, args);
    } 
    else 
    {
        str.resize(needed); // Resize to actual needed size
    }

    va_end(args);

    return str;
}

bool DeviceRuntimeTests::CheckGpsDuplicate(const char* description, int &count, uint32_t towMs, uint32_t gpsWeek, const uint8_t* msg, int msgSize, msg_history_t &hist)
{
    int toyMs = towMs + gpsWeek * C_MILLISECONDS_PER_WEEK;
    int histToyMs = hist.gpsTowMs + hist.gpsWeek * C_MILLISECONDS_PER_WEEK;

    if (toyMs == histToyMs)
    {   // Duplicate time
        LogEvent("NMEA Error: %s: Duplicate time (#%d): %d ms %d week >> %d ms %d week", description, ++count, hist.gpsTowMs, hist.gpsWeek, towMs, gpsWeek);
        LogEvent("  1: %.*s", hist.msgSize-2, (char*)hist.msg);
        LogEvent("  2: %.*s", msgSize-2, (char*)msg);
        return true;
    }

    return false;
}

bool DeviceRuntimeTests::CheckGpsTimeReverse(const char* description, int &count, uint32_t towMs, uint32_t gpsWeek, const uint8_t* msg, int msgSize, msg_history_t &hist)
{
    int toyMs = towMs + gpsWeek * C_MILLISECONDS_PER_WEEK;
    int histToyMs = hist.gpsTowMs + hist.gpsWeek * C_MILLISECONDS_PER_WEEK;

    if (toyMs < histToyMs)
    {   // Reversed time
        LogEvent("NMEA Error: %s: Reversed time (#%d): %d ms %d week >> %d ms %d week", description, ++count, hist.gpsTowMs, hist.gpsWeek, towMs, gpsWeek);
        LogEvent("  1: %.*s", hist.msgSize-2, (char*)hist.msg);
        LogEvent("  2: %.*s", msgSize-2, (char*)msg);
        return true;
    }

    return false;
}

void DeviceRuntimeTests::LogEvent(std::string str)
{   
    // Add serial number if non-zero
    if (m_devInfo.serialNumber)
    {
        str = "[SN" + std::to_string(m_devInfo.serialNumber) + "] " + str;
    }
    str += "\n";

#define MAX_LOG_SIZE    500000
    // Prevent logging too much data
    if (m_log.size() + str.size() > MAX_LOG_SIZE) 
    {
        // If appending would exceed maxSize, trim the existing content first
        m_log = m_log.substr(0, MAX_LOG_SIZE - str.size());
    }
    m_log += str;

#if 0   // Print to display
    std::cout << str;
#endif

    // Log to file 
    FILE *file = fopen(m_filename.c_str(), "a");
    if (file != NULL)
    {
        fprintf(file, "%.*s", (int)str.size(), str.c_str());
        fclose(file);
    }
}

std::string formatString(const char* format, va_list args) 
{
    // Starting with a guess for the required length
    size_t size = MAX_MSG_LENGTH_NMEA;
    std::vector<char> buffer(size);

    while (true) {
        va_list args_copy;
        va_copy(args_copy, args); // Make a copy of args to use

        // Attempt to format the string
        int needed = std::vsnprintf(buffer.data(), size, format, args_copy);

        // Clean up the copied va_list
        va_end(args_copy);

        // Check if the buffer was large enough
        if (needed < 0) 
        {   // Formatting error
            return "";
        }

        if (needed < size) 
        {   // Buffer was big enough
            return std::string(buffer.data());
        }

        // Increase buffer size and retry
        size = needed + 10;
        buffer.resize(size);
    }
}

void DeviceRuntimeTests::LogEvent(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    LogEvent(formatString(format, args));
    va_end(args);
}
