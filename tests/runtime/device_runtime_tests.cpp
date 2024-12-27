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
    ISFileManager::CreateDirectory(LOG_DIRECTORY);
    m_filename = CreateLogFilename(LOG_DIRECTORY);
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
    if (!m_enable)
    {
        return;
    }
    
    int size = comm.rxBuf.scanPrior - comm.rxBuf.head;

    std::string parser;
    switch (comm.rxBuf.head[0])
    {
    case PSC_ISB_PREAMBLE_BYTE1:    
        parser = std::string("ISB id ") + std::to_string(comm.rxPkt.dataHdr.id) + " " + std::string(cISDataMappings::DataName(comm.rxPkt.dataHdr.id));
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
    
//    printf("ISB: ID %d  Size %d\n", dataHdr.id, dataHdr.size);

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

std::deque<DeviceRuntimeTests::msg_history_t>& DeviceRuntimeTests::AddMsgHistory(std::deque<DeviceRuntimeTests::msg_history_t> &hist, DeviceRuntimeTests::msg_history_t msgHist)
{
    hist.push_front(msgHist);
    if (hist.size() > 3) { hist.pop_back(); }    // Keep 2, newest at front
    return hist;
}

void DeviceRuntimeTests::TestIsbGps(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf)
{
    std::deque<msg_history_t> &hist = AddMsgHistory(m_hist.isb.gps1Pos, msg_history_t((gps_pos_t*)dataBuf));

//        printf("ISB GpsPos1 (%d towMs, %d week)", hist[0].gpsTowMs, hist[0].gpsWeek);

    CheckGpsDuplicate       ("ISB Gps1Pos Error", m_errorCount.isbGpsTime, hist);
    CheckGpsTimeReverse     ("ISB Gps1Pos Error", m_errorCount.isbGpsTime, hist);
    CheckGpsIrregularPeriod ("ISB Gps1Pos Error", m_errorCount.isbGpsTime, hist);
}

void DeviceRuntimeTests::ProcessNMEA(const uint8_t* msg, int msgSize)
{
    if (!m_enable)
    {
        return;
    }

//    printf("NMEA (%d): %.*s", msgSize, msgSize, msg);
    
    int id = getNmeaMsgId(msg, msgSize);
    switch(id)
    {
    case NMEA_MSG_ID_GNGGA:     TestNmeaGga(msg, msgSize);      break;
    case NMEA_MSG_ID_GNZDA:     TestNmeaZda(msg, msgSize);      break;
    }
}

void DeviceRuntimeTests::TestNmeaGga(const uint8_t* msg, int msgSize)
{
    int utcWeekday = gpsTowMsToUtcWeekday(m_hist.isb.gps1Pos[0].gpsTowMs, C_GPS_LEAP_SECONDS);
    gps_pos_t gpsPos = {};
    utc_time_t t;
    nmea_parse_gga((const char *)msg, msgSize, gpsPos, t, utcWeekday);
    std::deque<msg_history_t> &hist = AddMsgHistory(m_hist.nmea.gga, msg_history_t(gpsPos.timeOfWeekMs, gpsPos.week, (uint8_t*)msg, msgSize));

//    printf("NMEA GGA (%d ms, %d wkday): %.*s", gpsPos.timeOfWeekMs, utcWeekday, msgSize, msg);

    CheckGpsDuplicate       ("NMEA GGA Error", m_errorCount.nmeaGgaTime, hist);
    CheckGpsTimeReverse     ("NMEA GGA Error", m_errorCount.nmeaGgaTime, hist);
    CheckGpsIrregularPeriod ("NMEA GGA Error", m_errorCount.nmeaGgaTime, hist);
}

void DeviceRuntimeTests::TestNmeaZda(const uint8_t* msg, int msgSize)
{
    uint32_t gpsTowMs;
    uint32_t gpsWeek;
    utc_date_t utcDate; 
    utc_time_t utcTime;
    nmea_parse_zda((char*)msg, msgSize, gpsTowMs, gpsWeek, utcDate, utcTime, C_GPS_LEAP_SECONDS);
    std::deque<msg_history_t> &hist = AddMsgHistory(m_hist.nmea.zda, msg_history_t(gpsTowMs, gpsWeek, (uint8_t*)msg, msgSize));

//    printf("NMEA ZDA (%d ms): %.*s", gpsTowMs, msgSize, msg);

    CheckGpsDuplicate       ("NMEA ZDA Error", m_errorCount.nmeaZdaTime, hist);
    CheckGpsTimeReverse     ("NMEA ZDA Error", m_errorCount.nmeaZdaTime, hist);
    CheckGpsIrregularPeriod ("NMEA ZDA Error", m_errorCount.nmeaZdaTime, hist);
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

bool DeviceRuntimeTests::CheckGpsDuplicate(const char* description, int &count, std::deque<msg_history_t> &hist)
{
    if (hist.size()<2) return false;
    
    int64_t toyMs[2];
    for (int i=0; i<2; i++)
        toyMs[i] = hist[i].gpsTowMs + hist[i].gpsWeek * C_MILLISECONDS_PER_WEEK;     // newest at front

    if (toyMs[0] == toyMs[1])
    {   // Duplicate time
        LogEvent("Error: %s: Duplicate time (#%d): %d ms %d week >> %d ms %d week", description, ++count, hist[1].gpsTowMs, hist[1].gpsWeek, hist[0].gpsTowMs, hist[0].gpsWeek);
        if (hist[0].msgSize)
        {
            for (int i=1; i>=0; i--)
                LogEvent("  %d: %.*s", i+1, hist[i].msgSize-2, (char*)hist[i].msg);
        }
        return true;
    }

    return false;
}

bool DeviceRuntimeTests::CheckGpsTimeReverse(const char* description, int &count, std::deque<msg_history_t> &hist)
{
    if (hist.size()<2) return false;

    int64_t toyMs[2];
    for (int i=0; i<2; i++)
        toyMs[i] = hist[i].gpsTowMs + hist[i].gpsWeek * C_MILLISECONDS_PER_WEEK;     // newest at front

    if (toyMs[0] < toyMs[1])
    {   // Reversed time
        LogEvent("Error: %s: Reversed time (#%d): %d ms %d week >> %d ms %d week", description, ++count, hist[1].gpsTowMs, hist[1].gpsWeek, hist[0].gpsTowMs, hist[0].gpsWeek);
        if (hist[0].msgSize)
        {
            for (int i=1; i>=0; i--)
                LogEvent("  %d: %.*s", i+1, hist[i].msgSize-2, (char*)hist[i].msg);
        }
        return true;
    }

    return false;
}

bool DeviceRuntimeTests::CheckGpsIrregularPeriod(const char* description, int &count, std::deque<msg_history_t> &hist)
{
    if (hist.size()<3) return false;

    int64_t toyMs[3];
    for (int i=0; i<3; i++)
        toyMs[i] = hist[i].gpsTowMs + hist[i].gpsWeek * C_MILLISECONDS_PER_WEEK;     // newest at front

    int64_t dtMs[2];
    for (int i=0; i<2; i++)
        dtMs[i] = toyMs[i] - toyMs[i+1];

    if (dtMs[0] != dtMs[1])
    {   // Irregular period
        LogEvent("Error: %s: Irregular period (#%d): %d ms %d week >> %d ms %d week", description, ++count, hist[1].gpsTowMs, hist[1].gpsWeek, hist[0].gpsTowMs, hist[0].gpsWeek);
        if (hist[0].msgSize)
        {
            for (int i=2; i>=0; i--)
                LogEvent("  %d: %.*s", i+1, hist[i].msgSize-2, (char*)hist[i].msg);
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
