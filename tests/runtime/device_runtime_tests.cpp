#include <chrono>
// #include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdarg.h>
#include "protocol_nmea.h"
#include "device_runtime_tests.h"
#include "ISFileManager.h"
#include "ISDataMappings.h"


#define LOG_DIRECTORY   "realtime_logs"

/**
 * @brief This class provides realtime evaluation conditions or errors in data streaming from the Inertial Sense products.
 * 
 */
DeviceRuntimeTests::DeviceRuntimeTests()
{
    m_gps1Pos.leapS = C_GPS_LEAP_SECONDS;

    ISFileManager::CreateDirectory(LOG_DIRECTORY);
    m_filename = CreateLogFilename(LOG_DIRECTORY);

    LogEvent("Realtime tests started...");
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
    int size = comm.rxBuf.scanPrior - comm.rxBuf.head;

    std::string parser;
    switch (comm.rxBuf.head[0])
    {
    case PSC_ISB_PREAMBLE_BYTE1:    
        parser = std::string("ISB id ") + std::to_string(comm.rxPkt.dataHdr.id) + " " + std::string(cISDataMappings::GetDataSetName(comm.rxPkt.dataHdr.id));
        parser += ", size " + std::to_string(comm.rxPkt.dataHdr.size); 
        break;
    case PSC_NMEA_START_BYTE:       parser = std::string("NMEA ") + std::string((char*)comm.rxBuf.head, _MIN(size, MAX_MSG_LENGTH_NMEA));    break;
    case UBLOX_START_BYTE1:         parser = std::string("UBX");        break;
    case RTCM3_START_BYTE:          parser = std::string("RTCM3");      break;
    case SPARTN_START_BYTE:         parser = std::string("SPARTN");     break;
    case SONY_START_BYTE:           parser = std::string("SONY");       break;
    // default:                        parser = charArrayToHex(comm.rxBuf.head, 4);    break;
    default:                        parser = std::string("Unknown");    break;
    }
    
    m_errorCount.parse = comm.rxErrorCount;
    LogEvent("Parse Error #%d, size %d: %s", comm.rxErrorCount, size, parser.c_str());
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
        uint32_t serialNumber = m_devInfo.serialNumber;
        copyDataPToStructP2(&m_devInfo, &dataHdr, dataBuf, sizeof(dev_info_t));
        if (serialNumber != m_devInfo.serialNumber)
        {   // Serial number changed.  Update log filename.
            m_filename = CreateLogFilename(LOG_DIRECTORY, m_devInfo.serialNumber);
        }
    }
        break;

    case DID_GPS1_POS:      TestIsbGps(dataHdr, dataBuf);       break;
    }
}

void DeviceRuntimeTests::TestIsbGps(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf)
{
    msg_history_t &hist = m_hist.isb.gps1Pos;

    copyDataPToStructP2(&m_gps1Pos, &dataHdr, dataBuf, sizeof(gps_pos_t));

    // printf("ISB GpsPos1 (%d ms, %d wkday): %.*s", m_gps1Pos.timeOfWeekMs, utcWeekday, msgSize, msg);

    CheckGpsDuplicate  ("ISB GpsPos1 Error", m_errorCount.nmeaGgaTime, m_gps1Pos.timeOfWeekMs, m_gps1Pos.week, NULL, 0, hist);
    CheckGpsTimeReverse("ISB GpsPos1 Error", m_errorCount.nmeaGgaTime, m_gps1Pos.timeOfWeekMs, m_gps1Pos.week, NULL, 0, hist);

    // Update history
    hist.update(m_gps1Pos.timeOfWeekMs, m_gps1Pos.week, NULL, 0);
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
    int utcWeekday = gpsTowMsToUtcWeekday(gpsPos.timeOfWeekMs, m_gps1Pos.leapS);
    nmea_parse_gga((const char *)msg, msgSize, gpsPos, t, utcWeekday);

    // printf("NMEA GGA (%d ms, %d wkday): %.*s", gpsPos.timeOfWeekMs, utcWeekday, msgSize, msg);

    CheckGpsDuplicate  ("NMEA GGA Error", m_errorCount.nmeaGgaTime, gpsPos.timeOfWeekMs, gpsPos.week, msg, msgSize, hist);
    CheckGpsTimeReverse("NMEA GGA Error", m_errorCount.nmeaGgaTime, gpsPos.timeOfWeekMs, gpsPos.week, msg, msgSize, hist);

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
    nmea_parse_zda((char*)msg, msgSize, gpsTowMs, gpsWeek, utcDate, utcTime, C_GPS_LEAP_SECONDS);

    printf("NMEA ZDA (%d ms): %.*s", gpsTowMs, msgSize, msg);

    CheckGpsDuplicate  ("NMEA ZDA Error", m_errorCount.nmeaZdaTime, gpsTowMs, gpsWeek, msg, msgSize, hist);
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
    unsigned int needed = std::vsnprintf(&str[0], str.size(), format, args);

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
        if (msg)
        {
            LogEvent("  1: %.*s", hist.msgSize-2, (char*)hist.msg);
            LogEvent("  2: %.*s", msgSize-2, (char*)msg);
        }
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
        if (msg)
        {
            LogEvent("  1: %.*s", hist.msgSize-2, (char*)hist.msg);
            LogEvent("  2: %.*s", msgSize-2, (char*)msg);
        }
        return true;
    }

    return false;
}

std::string DeviceRuntimeTests::Timestamp() 
{
    // Get current time as a high-resolution time_point
    auto now = std::chrono::system_clock::now();
    // Convert time_point to time_t for easier formatting of date and time
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // To add milliseconds, subtract time_t from time_point, then cast to milliseconds
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    // Use put_time to format the date and time part
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d %H:%M:%S");
    // Manually add the formatted milliseconds
    ss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count();

    std::string timestamp = "[" + ss.str();
    if (m_portName.size())
    {
        timestamp += " " + m_portName;
    }
    timestamp += "] ";
    return timestamp;
}

void DeviceRuntimeTests::LogEvent(std::string str)
{   
    // Add serial number if non-zero
    if (m_devInfo.serialNumber)
    {
        str = "[SN" + std::to_string(m_devInfo.serialNumber) + "] " + str;
    }
    str += "\n";

    // Prepend timestamp
    str = Timestamp() + str;

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

    while (true) 
    {
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

        if (needed < (int)size)
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
