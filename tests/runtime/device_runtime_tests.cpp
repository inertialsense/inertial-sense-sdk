#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdarg.h>
#include "protocol_nmea.h"
#include "device_runtime_tests.h"
#include "ISFileManager.h"
#include "ISDataMappings.h"


#define LOG_DIRECTORY       "realtime_logs"

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
    auto now = SYS_TIME_NOW;
    auto in_time = std::chrono::system_clock::to_time_t(now);

    // Convert to local time
    std::tm bt = *localtime(&in_time);

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
    LogEvent(SYS_TIME_NOW, "Parse Error #%d, size %d: %s", comm.rxErrorCount, size, parser.c_str());
}

void DeviceRuntimeTests::ProcessISB(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf)
{
    if (!m_enable)
    {
        return;
    }

    // WriteStatus("ISB: ID %d   Size %d (%s)\n", dataHdr.id, dataHdr.size, cISDataMappings::DataName(dataHdr.id));

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

/**
 * @brief Test messages for duplicates, reversed order, or irregular timestamps.
 */
void DeviceRuntimeTests::TestIsbGps(const p_data_hdr_t &dataHdr, const uint8_t *dataBuf)
{
    std::deque<msg_history_t> &hist = AddMsgHistory(m_hist.isb.gps1Pos, msg_history_t((gps_pos_t*)dataBuf));

    WriteStatus("ISB GpsPos1 (%d towMs, %d week)\n", hist[0].gpsTowMs, hist[0].gpsWeek);

    CheckGpsTime("ISB Gps1Pos Error", m_errorCount.isbGpsTime, hist);
}

void DeviceRuntimeTests::ProcessNMEA(const uint8_t* msg, int msgSize)
{
    if (!m_enable)
    {
        return;
    }

    WriteStatus("NMEA (%d): %.*s", msgSize, msgSize, msg);

    int id = getNmeaMsgId(msg, msgSize);
    switch(id)
    {
    case NMEA_MSG_ID_GNGGA:     TestNmeaGga(msg, msgSize);      break;
    case NMEA_MSG_ID_GNZDA:     TestNmeaZda(msg, msgSize);      break;
    case NMEA_MSG_ID_INTEL:     TestNmeaIntel(msg, msgSize);    break;
    }
}

/**
 * @brief Test messages for duplicates, reversed order, or irregular timestamps.
 */
void DeviceRuntimeTests::TestNmeaGga(const uint8_t* msg, int msgSize)
{
    int utcWeekday = m_hist.nmea.gga.size() ? gpsTowMsToUtcWeekday(m_hist.nmea.gga[0].gpsTowMs, C_GPS_LEAP_SECONDS) : 0;
    gps_pos_t gpsPos = {};
    utc_time_t t;
    nmea_parse_gga((const char *)msg, msgSize, gpsPos, t, utcWeekday);
    std::deque<msg_history_t> &hist = AddMsgHistory(m_hist.nmea.gga, msg_history_t(gpsPos.timeOfWeekMs, gpsPos.week, (uint8_t*)msg, msgSize));

    // WriteStatus("NMEA GGA (%d ms, %d wkday): %.*s", gpsPos.timeOfWeekMs, utcWeekday, msgSize, msg);

    CheckGpsTime("NMEA GGA Error", m_errorCount.nmeaGgaTime, hist);
}

/**
 * @brief Test messages for duplicates, reversed order, or irregular timestamps.
 */
void DeviceRuntimeTests::TestNmeaZda(const uint8_t* msg, int msgSize)
{
    uint32_t gpsTowMs;
    uint32_t gpsWeek;
    utc_date_t utcDate;
    utc_time_t utcTime;
    nmea_parse_zda((char*)msg, msgSize, gpsTowMs, gpsWeek, utcDate, utcTime, C_GPS_LEAP_SECONDS);
    std::deque<msg_history_t> &hist = AddMsgHistory(m_hist.nmea.zda, msg_history_t(gpsTowMs, gpsWeek, (uint8_t*)msg, msgSize));

    // WriteStatus("NMEA ZDA (%d ms): %.*s", gpsTowMs, msgSize, msg);

    CheckGpsTime("NMEA ZDA Error", m_errorCount.nmeaZdaTime, hist);
}

/**
 * @brief Test messages for duplicates, reversed order, or irregular timestamps.
 */
void DeviceRuntimeTests::TestNmeaIntel(const uint8_t* msg, int msgSize)
{
    dev_info_t info;
    gps_pos_t pos;
    gps_vel_t vel;
    float ppsPhase[2];
    uint32_t ppsNoiseNs[1];
    nmea_parse_intel((char*)msg, msgSize, info, pos, vel, ppsPhase, ppsNoiseNs);
    std::deque<msg_history_t> &hist = AddMsgHistory(m_hist.nmea.intel, msg_history_t(&pos, (uint8_t*)msg, msgSize));

    // WriteStatus("NMEA INTEL (%d ms): %.*s", gpsTowMs, msgSize, msg);

    CheckGpsTime("NMEA INTEL Error", m_errorCount.nmeaZdaTime, hist);
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

bool DeviceRuntimeTests::CheckGpsTime(const char* description, int &count, std::deque<msg_history_t> &hist)
{
    return CheckGpsTimeIrregular (description, count, hist) ||
           CheckGpsTimeDuplicate (description, count, hist) ||
           CheckGpsTimeReversed  (description, count, hist);
}

bool DeviceRuntimeTests::CheckGpsTimeDuplicate(const char* description, int &count, std::deque<msg_history_t> &hist)
{
    if (hist.size()<2) return false;

    int64_t toyMs[2];
    for (int i=0; i<2; i++)
        toyMs[i] = hist[i].gpsTowMs + hist[i].gpsWeek * C_MILLISECONDS_PER_WEEK;     // newest at front

    if (toyMs[0] == toyMs[1])
    {   // Duplicate time
        LogEvent(SYS_TIME_NOW, "Error: %s: Duplicate time (#%d): %d ms %d week >> %d ms %d week", description, ++count, hist[1].gpsTowMs, hist[1].gpsWeek, hist[0].gpsTowMs, hist[0].gpsWeek);
        if (hist[0].msgSize)
        {
            for (int i=1; i>=0; i--)
                LogEvent(hist[i].localTime, "  %d: %.*s", i+1, hist[i].msgSize-2, (char*)hist[i].msg);
        }
        return true;
    }

    return false;
}

bool DeviceRuntimeTests::CheckGpsTimeReversed(const char* description, int &count, std::deque<msg_history_t> &hist)
{
    if (hist.size()<2) return false;

    int64_t toyMs[2];
    for (int i=0; i<2; i++)
        toyMs[i] = hist[i].gpsTowMs + hist[i].gpsWeek * C_MILLISECONDS_PER_WEEK;     // newest at front

    if (toyMs[0] < toyMs[1])
    {   // Reversed time
        LogEvent(SYS_TIME_NOW, "Error: %s: Reversed time (#%d): %d ms %d week >> %d ms %d week", description, ++count, hist[1].gpsTowMs, hist[1].gpsWeek, hist[0].gpsTowMs, hist[0].gpsWeek);
        if (hist[0].msgSize)
        {
            for (int i=1; i>=0; i--)
                LogEvent(hist[i].localTime, "  %d: %.*s", i+1, hist[i].msgSize-2, (char*)hist[i].msg);
        }
        return true;
    }

    return false;
}

/**
 * @brief Detects irregular timestamps in a message stream and reports cause as either dropped message(s) or irregular timestamps.  
 */
bool DeviceRuntimeTests::CheckGpsTimeIrregular(const char* description, int &count, std::deque<msg_history_t> &hist)
{
    if (hist.size()<3) return false;

    if (hist[1].timeIrregular) return false;  // Prevent displaying irregular period twice

    int64_t toyMs[3];
    for (int i=0; i<3; i++)
        toyMs[i] = hist[i].gpsTowMs + hist[i].gpsWeek * C_MILLISECONDS_PER_WEEK;     // newest at front

    int64_t rxDtMs[2];
    int64_t dtMs[2];
    for (int i=0; i<2; i++)
    {
        dtMs[i] = toyMs[i] - toyMs[i+1];
        auto delta = hist[i].localTime - hist[i+1].localTime;
        rxDtMs[i] = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();
    }

    double dtRatio = (double)rxDtMs[0] / (double)rxDtMs[1];

    if (dtMs[0] != dtMs[1])
    {   // Irregular timestamps
        hist[0].timeIrregular = true;

        // Check if the interval between received messages changed by more than 50%
        std::string causeStr;
        if (dtRatio < 0.75 || dtRatio > 1.5)
        {   // Dropped message
            causeStr = "Dropped message";
        }
        else
        {   // Irregular timestamp
            causeStr = "Irregular time";
        }

        LogEvent(SYS_TIME_NOW, "Error: %s: %s (#%d): %d ms %d week >> %d ms %d week", description, causeStr.c_str(), ++count, hist[1].gpsTowMs, hist[1].gpsWeek, hist[0].gpsTowMs, hist[0].gpsWeek);
        if (hist[0].msgSize)
        {
            for (int i=2; i>=0; i--)
                LogEvent(hist[i].localTime, "  %d: %.*s", 3-i, hist[i].msgSize-2, (char*)hist[i].msg);
        }
        return true;
    }

    return false;
}

std::string DeviceRuntimeTests::Timestamp(system_time_t time)
{
    // Convert time_point to time_t for easier formatting of date and time
    auto in_time = std::chrono::system_clock::to_time_t(time);

    // To add milliseconds, subtract time_t from time_point, then cast to milliseconds
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()) % 1000;

    std::stringstream ss;
    // Use put_time to format the date and time part
    ss << std::put_time(std::localtime(&in_time), "%Y%m%d %H:%M:%S");
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

void DeviceRuntimeTests::LogEvent(system_time_t time, std::string str)
{
    // Add serial number if non-zero
    if (m_devInfo.serialNumber)
    {
        str = "[SN" + std::to_string(m_devInfo.serialNumber) + "] " + str;
    }
    str += "\n";

    // Prepend timestamp
    str = Timestamp(time) + str;

    // Prevent logging too much data
    if (m_log.size() + str.size() > RUNTIME_TEST_MAX_LOG_SIZE)
    {
        // If appending would exceed maxSize, trim the existing content first
        m_log = m_log.substr(0, RUNTIME_TEST_MAX_LOG_SIZE - str.size());
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

void DeviceRuntimeTests::WriteStatus(std::string str)
{
    // Prevent logging too much data
    if (m_status.size() + str.size() > RUNTIME_TEST_MAX_LOG_SIZE)
    {
        // If appending would exceed maxSize, trim the existing content first
        m_status = m_status.substr(0, RUNTIME_TEST_MAX_LOG_SIZE - str.size());
    }
    m_status += str;

#if 0   // Print to display
    std::cout << str;
#endif
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

void DeviceRuntimeTests::LogEvent(system_time_t time, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    LogEvent(time, formatString(format, args));
    va_end(args);
}

void DeviceRuntimeTests::WriteStatus(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    WriteStatus(formatString(format, args));
    va_end(args);
}
