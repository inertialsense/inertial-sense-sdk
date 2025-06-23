/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_LOGGER_H
#define IS_LOGGER_H

#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <memory>

#include "DeviceLogSerial.h"
#include "DeviceLogRaw.h"
 
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
#include "DeviceLogCSV.h"
#include "DeviceLogJSON.h"
#include "DeviceLogKML.h"
#endif

#if PLATFORM_IS_EVB_2
#include "d_time.h"
#include "ISLogFileFatFs.h"
#else
#include "ISLogFile.h"
#endif

#include "ISConstants.h"
#include "ISLogStats.h"


// default logging path if none specified
#define DEFAULT_LOGS_DIRECTORY          "IS_logs"
#define DEFAULT_LOGS_MAX_FILE_SIZE      (1024 * 1024 * 5)	// 5 MB

class cISLogger
{
public:
    enum eLogType
    {
        LOGTYPE_DAT = 0,    // serial
        LOGTYPE_RAW,        // packetized serial.  Supports multiple packet types
        LOGTYPE_SDAT,       // sorted -- unsupported/deprecated, DO NOT USE!
        LOGTYPE_CSV,
        LOGTYPE_KML,
        LOGTYPE_JSON,
        LOGTYPE_COUNT
    };

    // Static array of strings for log type names
    static const char* logTypeStrings[LOGTYPE_COUNT];

    struct sSaveOptions
    {
        eLogType logType;                           // File format to use for logger
        float driveUsageLimitPercent;               // Limit of drive usage in percent of total drive size.
        float driveUsageLimitMb;                    // Limit of drive usage in megabytes.
        uint32_t maxFileSize;                       // Largest size of each individual log file.
        bool useSubFolderTimestamp;                 // Cause each log instance to be written to separate timestamped folder.
        std::string timeStamp;                      // Used to name each log instance directory.  System date and time used if left empty.
        std::string subDirectory;                   // Write logs into sub-directory of this name inside log instance directory. 

        sSaveOptions(                               // Default Options:
            eLogType type = LOGTYPE_RAW,            // Raw packetized serial.  
            float limitPercent = 0.5,               // 50% of total drive
            float limitMb = 0,                      // Drive usage limit in MB.  0 disables this limit
            uint32_t fileSize = 5 * 1024 * 1024,    // 5 MB size of each individual file in log
            bool useTimestamp = true,               // A timestamped subdirectory is created for new log
            std::string subDir = ""                 // Logs will be written into a subdirector of this string name within the timestamped subdirectory this string is not empty
        ) : logType(type),
            driveUsageLimitPercent(limitPercent),
            driveUsageLimitMb(limitMb),
            maxFileSize(fileSize),
            useSubFolderTimestamp(useTimestamp),
            subDirectory(subDir)
        {}
    };

    static const std::string g_emptyString;

    cISLogger();
    virtual ~cISLogger();

    // Setup logger to read from file
    bool LoadFromDirectory(const std::string& directory, eLogType logType = LOGTYPE_RAW, std::vector<std::string> serials = {});

    // Setup logger for writing to file
    bool InitSave(const std::string& directory = g_emptyString, const sSaveOptions& options = cISLogger::sSaveOptions());
    [[deprecated("Not recommended for future development.  Use InitSave() with sSaveOptions instead.")]]
    bool InitSave(eLogType logType = LOGTYPE_RAW, const std::string& directory = g_emptyString, float driveUsageLimitPercent = 0.5f, uint32_t maxFileSize = 5 * 1024 * 1024, bool useSubFolderTimestamp = true);
    [[deprecated("Not recommended for future development.  Use InitSave() with sSaveOptions instead.")]]
    bool InitSaveTimestamp(const std::string& timeStamp, const std::string& directory = g_emptyString, const std::string& subDirectory = g_emptyString, eLogType logType = LOGTYPE_DAT, float driveUsageLimitPercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, bool useSubFolderTimestamp = true);

    // Establish link between devices and this logger
    std::shared_ptr<cDeviceLog> registerDevice(ISDevice& device);
    std::shared_ptr<cDeviceLog> registerDevice(uint16_t hdwId, uint32_t serialNo);
    std::shared_ptr<cDeviceLog> registerDevice(dev_info_t& devInfo) { return registerDevice(ENCODE_DEV_INFO_TO_HDW_ID(devInfo), devInfo.serialNumber); }

    // Update internal state, handle timeouts, remove old files for file culling, etc.
    void Update();
    bool LogData(std::shared_ptr<cDeviceLog> devLogger, p_data_hdr_t* dataHdr, const uint8_t* dataBuf);
    bool LogData(std::shared_ptr<cDeviceLog> devLogger, int dataSize, const uint8_t* dataBuf);
    bool LogDataBySN(uint32_t serialNo, p_data_hdr_t* dataHdr, const uint8_t* dataBuf) { return LogData(DeviceLogBySerialNumber(serialNo), dataHdr, dataBuf); }
    bool LogDataBySN(uint32_t serialNo, int dataSize, const uint8_t* dataBuf) {  return LogData(DeviceLogBySerialNumber(serialNo), dataSize, dataBuf); }
    p_data_buf_t* ReadData(std::shared_ptr<cDeviceLog> devLogger = nullptr);
    p_data_buf_t* ReadData(size_t devIndex);
    p_data_buf_t* ReadNextData(size_t& devIndex);
    void EnableLogging(bool enabled) { m_enabled = enabled; }
    bool Enabled() { return m_enabled; }
    void CloseAllFiles();
    void FlushToFile();
    void OpenWithSystemApp();
    void ShowParseErrors(bool show);
    std::string TimeStamp() { return m_timeStamp; }
    std::string LogDirectory() { return m_directory; }
    std::string RootDirectory() { return m_rootDirectory; }
    uint64_t LogSizeAll();
    uint64_t LogSize(uint32_t devSerialNo);
    float LogSizeAllMB();
    float LogSizeMB(uint32_t devSerialNo);
    float FileSizeMB(uint32_t devSerialNo);
    uint32_t FileCount(uint32_t devSerialNo);
    float MaxDiskSpaceMB() { return ((float)m_maxDiskSpace) / (1024*1024); }
    float UsedDiskSpaceMB() { return ((float)m_usedDiskSpace) / (1024*1024); }
    std::string GetNewFileName(uint32_t devSerialNo, uint32_t fileCount, const char* suffix);
    std::vector<std::shared_ptr<cDeviceLog>> DeviceLogs();
    uint32_t DeviceCount() { return (uint32_t)m_devices.size(); }
    std::shared_ptr<cDeviceLog> DeviceLogBySerialNumber(uint32_t serialNo) {
        return (m_devices.count(serialNo) ? m_devices[serialNo] : nullptr);
    }
    // bool SetDeviceInfo(const dev_info_t *info, unsigned int device = 0);
    // const dev_info_t* DeviceInfo(unsigned int device = 0);

    /**
     * @brief Copy (convert) one log to another log type.
     * 
     * @param log Input log
     * @param timestamp Time to use on the output log
     * @param outputDir Directory to write output log
     * @param logType Type of the output log
     * @param maxFileSize Max size of the individual output files
     * @param driveUsageLimitPercent Drive usage limit in percent of total drive space
     * @param driveUsageLimitMb Drive usage limit in MB
     * @param useSubFolderTimestamp Output logs should be written into a timestamped subdirectory 
     * @param enableCsvIns2ToIns1Conversion  Convert Ins2 to Ins1 within the log 
     * @return true if log copy (conversion) output was successful, false if not.
     */
    bool CopyLog(
        cISLogger& log,
        const std::string& timestamp = g_emptyString,
        const std::string& outputDir = g_emptyString,
        eLogType logType = LOGTYPE_DAT,
        uint32_t maxFileSize = DEFAULT_LOGS_MAX_FILE_SIZE,
        float driveUsageLimitPercent = 0.5f, 
        bool useSubFolderTimestamp = true,
        bool enableCsvIns2ToIns1Conversion = true);
    unsigned int Count() { return m_logStats.Count(); }
    unsigned int Errors() { return m_logStats.Errors(); }
    eLogType Type() { return m_logType; }

    /**
    * Get the timeout flush parameter in seconds
    * @return the timeout flush parameter in seconds
    */
    time_t TimeoutFlushSeconds() { return m_timeoutFlushSeconds; }

    /**
    * Set the timeout flush logger parameter in seconds
    * @param timeoutFlushSeconds the timeout flush logger parameter in seconds
    */
    void SetTimeoutFlushSeconds(time_t timeoutFlushSeconds) { m_timeoutFlushSeconds = timeoutFlushSeconds; }

    // check if a data header is corrupt
    static bool isHeaderCorrupt(const p_data_hdr_t* hdr);

    // create a timestamp
    static std::string CreateCurrentTimestamp();

    // check if a data packet is corrupt, NULL data is OK
    bool isDataCorrupt(const p_data_buf_t* data);

    // read all log data into memory - if the log is over 1.5 GB this will fail on 32 bit processes
    // the map contains device id (serial number) key and a vector containing log data for each data id, which will be an empty vector if no log data for that id
    // static bool ReadAllLogDataIntoMemory(const std::string& directory, std::map<uint32_t, std::vector<std::vector<uint8_t>>>& data);

    void SetKmlConfig(bool gpsData = true, bool showPath = true, bool showSample = false, bool showTimeStamp = true, double updatePeriodSec = 1.0, bool altClampToGround = true)
    {
        m_gpsData = gpsData;
        m_showPath = showPath;
        m_showSample = showSample;
        m_showTimeStamp = showTimeStamp;
        m_iconUpdatePeriodSec = updatePeriodSec;
        m_altClampToGround = altClampToGround;

        for (auto d : DeviceLogs())
        {
            d->SetKmlConfig(m_showPath, m_showSample, m_showTimeStamp, m_iconUpdatePeriodSec, m_altClampToGround);
        }
    }

    static eLogType ParseLogType(const std::string& logTypeString)
    {
        if (logTypeString == "csv")
        {
            return cISLogger::eLogType::LOGTYPE_CSV;
        }
        else if (logTypeString == "kml")
        {
            return cISLogger::eLogType::LOGTYPE_KML;
        }
        else if (logTypeString == "sdat")
        {
            return cISLogger::eLogType::LOGTYPE_SDAT;
        }
        else if (logTypeString == "json")
        {
            return cISLogger::eLogType::LOGTYPE_JSON;
        }
        else if (logTypeString == "raw")
        {
            return cISLogger::eLogType::LOGTYPE_RAW;
        }
        return cISLogger::eLogType::LOGTYPE_DAT;
    }

    static bool ParseFilename(std::string filename, int &serialNum, std::string &date, std::string &time, int &index);
    void PrintStatistics();
    void PrintIsCommStatus();
    void PrintLogDiskUsage();

private:
#if CPP11_IS_ENABLED
    cISLogger(const cISLogger& copy) = delete;
#else
    cISLogger(const cISLogger& copy); // Disable copy constructors
#endif

    bool InitDevicesForWriting(std::vector<ISDevice>& devices);
    void Cleanup();
    void PrintProgress();

    static time_t GetTime()
    {
#if PLATFORM_IS_EVB_2
        return static_cast<time_t>(time_msec() / 1000);
#else
        return time(NULLPTR);
#endif
    }

    eLogType				m_logType = LOGTYPE_DAT;
    bool					m_useChunkHeader = true;
    bool					m_enabled = false;
    std::string				m_rootDirectory;
    std::string				m_directory;
    std::string				m_timeStamp;
    std::map<uint32_t, std::shared_ptr<cDeviceLog>> m_devices = { };

    uint64_t				m_maxDiskSpace = 0;		// Limit for logging.  Zero to disable file culling drive management.
    uint64_t				m_usedDiskSpace = 0;	// Size of all logs
    uint32_t				m_maxFileSize = 0;
    cLogStats				m_logStats;
#if PLATFORM_IS_EVB_2
    cISLogFileFatFs         m_errorFile;
#else
    cISLogFile				m_errorFile;
#endif

    bool					m_altClampToGround = false;
    bool					m_gpsData = false;
    bool					m_showSample = false;
    bool					m_showPath = false;
    bool					m_showTimeStamp = false;
    double					m_iconUpdatePeriodSec = false;
    time_t                  m_logStartTime = 0;
    time_t					m_lastCommTime = 0;
    time_t					m_timeoutFlushSeconds = 0;
    time_t					m_timeoutFileCullingSeconds = 10;
    time_t					m_lastFileCullingTime = 0;
    int						m_progress = 0;
    bool					m_showParseErrors = true;
};


#endif // IS_LOGGER_H
