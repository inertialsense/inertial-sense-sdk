/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ctime>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <regex>
#include <set>
#include <sstream>
#include <mutex>

#include "ISFileManager.h"
#include "ISLogger.h"
#include "ISDataMappings.h"
#include "ISDisplay.h"
#include "ISLogFileFactory.h"
#include "ISUtilities.h"

#include "convert_ins.h"

#if PLATFORM_IS_EVB_2
#include "globals.h"
#include "rtc.h"
#endif

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE
#include <sys/statvfs.h>
#endif

using namespace std;

// #define DONT_CHECK_LOG_DATA_SET_SIZE		// uncomment to allow reading in of new data logs into older code sets
#define LOG_DEBUG_PRINT_READ		0
#define STATS_ALL_FILENAME          "/stats_all.txt"

const string cISLogger::g_emptyString;

#if !PLATFORM_IS_EMBEDDED
class SimpleMutex {
private:
    std::atomic_flag flag = ATOMIC_FLAG_INIT; // Atomic flag for lock status

public:
    void lock() {
        // Spin-wait loop until the lock is acquired
        while (flag.test_and_set(std::memory_order_acquire)) {
            // Busy-wait until the lock is acquired
            SLEEP_MS(1);
        }
    }

    void unlock() {
        // Release the lock
        flag.clear(std::memory_order_release);
    }
};

SimpleMutex myMutex;

#define LOCK_MUTEX()        myMutex.lock()
#define UNLOCK_MUTEX()      myMutex.unlock()

#else

#define LOCK_MUTEX()        
#define UNLOCK_MUTEX()      

#endif

const char* cISLogger::logTypeStrings[] = {
    "dat",  // LOGTYPE_DAT
    "raw",  // LOGTYPE_RAW
    "sdat", // LOGTYPE_SDAT
    "csv",  // LOGTYPE_CSV
    "kml",  // LOGTYPE_KML
    "json"  // LOGTYPE_JSON
};

bool cISLogger::isHeaderCorrupt(const p_data_hdr_t *hdr)
{
    bool isCorrupt = true;

    if (hdr != NULL)
    {   // if any case is true this is corrupt
        isCorrupt = (hdr->size == 0 ||
            hdr->offset + hdr->size > MAX_DATASET_SIZE ||
            hdr->id == 0 ||
            hdr->offset % 4 != 0);
    }

    return isCorrupt;
}

bool cISLogger::isDataCorrupt(const p_data_buf_t *data)
{
    return m_useChunkHeader && data != NULL && isHeaderCorrupt(&data->hdr);
}

cISLogger::cISLogger()
{
    m_logStats.Clear();
}

cISLogger::~cISLogger()
{
    Cleanup();
    m_errorFile.close();
}

void cISLogger::Cleanup()
{
    LOCK_MUTEX();
    m_devices.clear();
    m_logStats.Clear();
    UNLOCK_MUTEX();
}

void cISLogger::Update()
{
    time_t timeSec = GetTime();

    if (m_lastCommTime == 0)
    {
        m_lastCommTime = timeSec;
    }
    else if (m_timeoutFlushSeconds > 0 && timeSec - m_lastCommTime >= m_timeoutFlushSeconds)
    {
        for (auto it : m_devices)
        {
            it.second->Flush();
        }
    }

    if (m_enabled && m_maxDiskSpace!=0 && 
        m_timeoutFileCullingSeconds > 0 && 
        timeSec - m_lastFileCullingTime >= m_timeoutFileCullingSeconds)
    {   // File culling
        m_lastFileCullingTime = timeSec;

        // Update amount of drive space for parent log directory
        m_usedDiskSpace = ISFileManager::GetDirectorySpaceUsed(m_rootDirectory);

        if (m_usedDiskSpace > m_maxDiskSpace)
        {   // Need to remove files
           ISFileManager::RemoveOldestFiles(m_rootDirectory, m_maxDiskSpace);
           ISFileManager::RemoveEmptyDirectories(m_rootDirectory);
        }
    }

    ISFileManager::TouchFile(m_directory + STATS_ALL_FILENAME);
}

bool cISLogger::InitSave(const string &directory, const sSaveOptions &options) 
{
    static const int minFileCount = 50;
    static const int maxFileCount = 10000;

    // Close any open files
    CloseAllFiles();

    m_logType = options.logType;
    m_timeStamp = (options.timeStamp.empty() ? CreateCurrentTimestamp() : options.timeStamp);
    m_rootDirectory = m_directory = (directory.empty() ? DEFAULT_LOGS_DIRECTORY : directory);
    m_logStartTime = GetTime();

    // Drive usage limit
    m_maxDiskSpace = 0;                 // disable log file culling
    if (options.driveUsageLimitPercent > 0.0f)
    {   // Percent limit enabled
        // options.driveUsageLimitPercent = _CLAMP(options.driveUsageLimitPercent, 0.01f, 0.99f);
        uint64_t totalDiskSize = ISFileManager::GetDirectoryDriveTotalSize(m_rootDirectory);
        m_maxDiskSpace = (uint64_t)(totalDiskSize * options.driveUsageLimitPercent);
    }
    if (options.driveUsageLimitMb > 0.0f)
    {   // Size limit enabled
        m_maxDiskSpace = _MIN(m_maxDiskSpace, (uint64_t)(options.driveUsageLimitMb*1024*1024));
    }

    // Limit to available size
    uint64_t availableSpace = ISFileManager::GetDirectorySpaceAvailable(m_rootDirectory);
    m_maxDiskSpace = _MIN(m_maxDiskSpace, availableSpace); 

    // Amount of drive space used by parent log directory (i.e. "IS_log")
    m_usedDiskSpace = ISFileManager::GetDirectorySpaceUsed(m_rootDirectory);

    // ensure there are between min and max file count
    if (options.maxFileSize > availableSpace / minFileCount)
    {
        m_maxFileSize = (uint32_t)(availableSpace / minFileCount);
    }
    else if (options.maxFileSize < availableSpace / maxFileCount)
    {
        m_maxFileSize = (uint32_t)(availableSpace / maxFileCount);
    }
    else
    {
        m_maxFileSize = options.maxFileSize;
    }

    m_maxFileSize = _MIN(m_maxFileSize, options.maxFileSize);

    // create root dir
    _MKDIR(m_rootDirectory.c_str());

    if (options.useSubFolderTimestamp || options.subDirectory.size()>0)
    {
        // create time stamp dir
        m_directory = m_rootDirectory + "/" + m_timeStamp;
        _MKDIR(m_directory.c_str());

        if (!options.subDirectory.empty())
        {
            // create sub dir
            m_directory += "/" + options.subDirectory;
            _MKDIR(m_directory.c_str());
        }
    }

    // create empty stats file to track timestamps
    string str = m_directory + (options.subDirectory.empty() ? "" : "/" + options.subDirectory) + STATS_ALL_FILENAME;
    cISLogFileBase *statsFile = CreateISLogFile(str, "w");
    CloseISLogFile(statsFile);

    // Initialize devices
    // return InitDevicesForWriting(numDevices); // Lazy Initialize of devices (when they are explicitly added)
    return ISFileManager::PathIsDir(m_directory);
}

[[deprecated("Not recommended for future development.")]]
bool cISLogger::InitSave(eLogType logType, const string &directory, float driveUsageLimitPercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
    sSaveOptions options;
    options.logType                 = logType;
    options.driveUsageLimitPercent  = driveUsageLimitPercent;
    options.maxFileSize             = maxFileSize;
    options.useSubFolderTimestamp   = useSubFolderTimestamp;
    return InitSave(directory, options); 
}

[[deprecated("Not recommended for future development.")]]
bool cISLogger::InitSaveTimestamp(const string &timeStamp, const string &directory, const string &subDirectory, eLogType logType, float driveUsageLimitPercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
    sSaveOptions options;
    options.logType                 = logType;
    options.driveUsageLimitPercent  = driveUsageLimitPercent;
    options.maxFileSize             = maxFileSize;
    options.useSubFolderTimestamp   = useSubFolderTimestamp;
    options.timeStamp               = timeStamp;
    options.subDirectory            = subDirectory;
    return InitSave(directory, options); 
}

string cISLogger::CreateCurrentTimestamp()
{
    char buf[80];

#if PLATFORM_IS_EVB_2
    date_time_t rtc;
#if USE_RTC_DATE_TIME   // use RTC
    rtc_get_date(RTC, &rtc.year, &rtc.month, &rtc.day, &rtc.week);
    rtc_get_time(RTC, &rtc.hour, &rtc.minute, &rtc.second);
#else   // use uINS GPS time
    rtc = g_gps_date_time;
#endif
    SNPRINTF(buf, _ARRAY_BYTE_COUNT(buf), "%.4d%.2d%.2d_%.2d%.2d%.2d",
        (int)rtc.year, (int)rtc.month, (int)rtc.day,
        (int)rtc.hour, (int)rtc.minute, (int)rtc.second);
#else
    // Create timestamp
    time_t rawtime = GetTime();
    struct tm *timeinfo = localtime(&rawtime);
    strftime(buf, 80, "%Y%m%d_%H%M%S", timeinfo);
#endif

    return string(buf);
}

std::shared_ptr<cDeviceLog> cISLogger::registerDevice(ISDevice& device) {
    switch (m_logType)
    {
        default:
        case LOGTYPE_DAT:   device.devLogger = make_shared<cDeviceLogSerial>(&device);  break;
        case LOGTYPE_RAW:   device.devLogger = make_shared<cDeviceLogRaw>(&device);     break;
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
        case LOGTYPE_CSV:   device.devLogger = make_shared<cDeviceLogCSV>(&device);     break;
        case LOGTYPE_JSON:  device.devLogger = make_shared<cDeviceLogJSON>(&device);    break;
        case LOGTYPE_KML:   device.devLogger = make_shared<cDeviceLogKML>(&device);     break;
#endif
    }
    device.devLogger->InitDeviceForWriting(m_timeStamp, m_directory, m_maxDiskSpace, m_maxFileSize);
    m_devices[device.devInfo.serialNumber] = device.devLogger;

    return device.devLogger;
}

std::shared_ptr<cDeviceLog> cISLogger::registerDevice(uint16_t hdwId, uint32_t serialNo) {
    std::shared_ptr<cDeviceLog> deviceLog;
    switch (m_logType)
    {
        default:
        case LOGTYPE_DAT:   deviceLog = make_shared<cDeviceLogSerial>(hdwId, serialNo);  break;
        case LOGTYPE_RAW:   deviceLog = make_shared<cDeviceLogRaw>(hdwId, serialNo);     break;
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
        case LOGTYPE_CSV:   deviceLog = make_shared<cDeviceLogCSV>(hdwId, serialNo);     break;
        case LOGTYPE_JSON:  deviceLog = make_shared<cDeviceLogJSON>(hdwId, serialNo);    break;
        case LOGTYPE_KML:   deviceLog = make_shared<cDeviceLogKML>(hdwId, serialNo);     break;
#endif
    }
    deviceLog->InitDeviceForWriting(m_timeStamp, m_directory, m_maxDiskSpace, m_maxFileSize);
    m_devices[serialNo] = deviceLog;

    return deviceLog;
}

bool cISLogger::InitDevicesForWriting(std::vector<ISDevice>& devices)
{
    // Remove all devices
    Cleanup();

    // Add new devices
    {
        LOCK_MUTEX();
        // for (int i = 0; i < numDevices; i++)
        for (auto& d : devices) {
            registerDevice(d);
        }
        UNLOCK_MUTEX();
    }

    //m_errorFile = CreateISLogFile((m_directory + "/errors.txt"), "w");
    //m_errorFile.open((m_directory + "/errors.txt"), "w");

    return ISFileManager::PathIsDir(m_directory);
}

bool nextStreamDigit(stringstream &ss, string &str)
{
    if (!getline(ss, str, '_'))
    {
        return false;	// No data 
    }

    if (str.size() == 0 || !isdigit(str[0]))
    {
        return false;	// No numerical data 
    }

    return true;
}

// Return true for valid filenames, only if they contain 1.) serial number, date, time, and index number, or 2.) only an index number.  
bool cISLogger::ParseFilename(string filename, int &serialNum, string &date, string &time, int &index)
{
    serialNum = 0;
    date.clear();
    time.clear();
    index = -1;

    // Remove file extension
    size_t n = filename.rfind('.');
    if (n == string::npos)
    {	// No file extension
        return false;
    }
    string content = filename.substr(0, n);

    n = content.find(IS_LOG_FILE_PREFIX);
    string str;
    if (n != string::npos)
    {	// Has prefix - get serial number
        content = content.substr(n + sizeof(IS_LOG_FILE_PREFIX) - 1);
        stringstream ss(content);

        // Read serial number, date, time, index
        if (!nextStreamDigit(ss, str) || str.size()==0) { return false; } 	
        serialNum = stoi(str);
        if (!nextStreamDigit(ss, str) || str.size()==0) { return false; } 	
        date = str;
        if (!nextStreamDigit(ss, str) || str.size()==0) { return false; } 	
        time = str;
        if (!nextStreamDigit(ss, str) || str.size()==0) { return false; } 	
        index = stoi(str);
    }
    else
    {	// No prefix - only index number
        stringstream ss(content);
        if (!nextStreamDigit(ss, str) && str.size()) { return false; } 	
        index = stoi(str);
    }

    return true;
}


bool cISLogger::LoadFromDirectory(const string &directory, eLogType logType, vector<string> serials)
{
    // Delete and clear prior devices
    Cleanup();
    m_logType = logType;
    m_useChunkHeader = logType != cISLogger::LOGTYPE_RAW;
    string fileExtensionRegex;
    set<string> serialNumbers;

    switch (logType)
    {
    default:
    case cISLogger::LOGTYPE_DAT: fileExtensionRegex = "\\.dat$"; break;
    case cISLogger::LOGTYPE_RAW: fileExtensionRegex = "\\.raw$"; break;
    case cISLogger::LOGTYPE_SDAT: fileExtensionRegex = "\\.sdat$"; break;
    case cISLogger::LOGTYPE_CSV: fileExtensionRegex = "\\.csv$"; break;
    case cISLogger::LOGTYPE_JSON: fileExtensionRegex = "\\.json$"; break;
    case cISLogger::LOGTYPE_KML: return false; // fileExtensionRegex = "\\.kml$"; break; // kml read not supported
    }

    // get all files, sorted by name
    vector<ISFileManager::file_info_t> files;
    ISFileManager::GetDirectorySpaceUsed(directory, fileExtensionRegex, files, false, false);
    if (files.size() == 0)
    {
        return false;
    }

    int serialNum, index;
    string date, time;

    for (size_t i = 0; i < files.size(); i++)
    {
        string name = ISFileManager::GetFileName(files[i].name);
        if (!ParseFilename(name, serialNum, date, time, index))
        {   // Skip invalid filename
            continue;
        }

        if (serialNum >= 0)
        {
            string serialNumber = to_string(serialNum);

            // if we don't have a timestamp yet, see if we can parse it from the filename, i.e. IS_LOG_FILE_PREFIX 30013_20170103_151023_001
            if (m_timeStamp.length() == 0)
            {
                m_timeStamp = date;
                m_timeStamp += (m_timeStamp.size() ? "_" : "") + time;
            }

            // check for unique serial numbers
            if (serialNumbers.find(serialNumber) == serialNumbers.end())
            {
                bool emptySerialsList = serials.size() == 0;
                bool inSerialsList = find(serials.begin(), serials.end(), serialNumber) != serials.end();
                bool useAll = find(serials.begin(), serials.end(), "ALL") != serials.end();

                if (emptySerialsList || inSerialsList || useAll) // and that it is a serial number we want to use
                {
                    serialNumbers.insert(serialNumber);

                    // Add devices
                    {
                        LOCK_MUTEX();
                        std::shared_ptr<cDeviceLog> deviceLog;
                        switch (logType)
                        {
                        default:
                        case cISLogger::LOGTYPE_DAT:    deviceLog = make_shared<cDeviceLogSerial>(0, serialNum); break;
                        case cISLogger::LOGTYPE_RAW:    deviceLog = make_shared<cDeviceLogRaw>(0, serialNum); break;
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
                        case cISLogger::LOGTYPE_CSV:    deviceLog = make_shared<cDeviceLogCSV>(0, serialNum); break;
                        case cISLogger::LOGTYPE_JSON:   deviceLog = make_shared<cDeviceLogJSON>(0, serialNum); break;
#endif
                        }
                        deviceLog->SetupReadInfo(directory, serialNumber, m_timeStamp);
                        m_devices[serialNum] = deviceLog;
                        UNLOCK_MUTEX();
                    }

#if (LOG_DEBUG_GEN == 2)
                    PrintProgress();
#elif LOG_DEBUG_GEN
                    printf("cISLogger::LoadFromDirectory SN%s %s (file %d of %d)\n", serialNumber.c_str(), m_timeStamp.c_str(), (int)i + 1, (int)files.size());
#endif
                }
            }
        }
    }

    for (auto &d : this->DeviceLogs()) {
        d->InitDeviceForReading();
    }
    return (m_devices.size() != 0);
}

bool cISLogger::LogData(std::shared_ptr<cDeviceLog> deviceLog, p_data_hdr_t *dataHdr, const uint8_t *dataBuf)
{
    // This method is NOT for LOGTYPE_RAW (but all others)
    if (!m_enabled || (deviceLog == nullptr) || (m_logType == LOGTYPE_RAW)) {
        return false;
    }

    if (deviceLog == NULL || dataHdr == NULL || dataBuf == NULL)
    {
        m_errorFile.lprintf("Invalid device handle or NULL data\r\n");
        return false;
    }

    m_lastCommTime = GetTime();
    if (isHeaderCorrupt(dataHdr))
    {
        m_errorFile.lprintf("Corrupt log header, id: %lu, offset: %lu, size: %lu\r\n", (unsigned long)dataHdr->id, (unsigned long)dataHdr->offset, (unsigned long)dataHdr->size);
        m_logStats.LogError(dataHdr);
    }
    else if (!deviceLog->SaveData(dataHdr, dataBuf))
    {
        m_errorFile.lprintf("Underlying log implementation failed to save\r\n");
        m_logStats.LogError(dataHdr);
    }
#if 1
    else
    {	// Success
        m_logStats.LogData(_PTYPE_INERTIAL_SENSE_DATA, dataHdr->id, ISB_HDR_TO_PACKET_SIZE(*dataHdr));

        if (dataHdr->id == DID_DIAGNOSTIC_MESSAGE)
        {
            cISLogFileBase *outfile = CreateISLogFile(m_directory + "/diagnostic_" + std::to_string(deviceLog->DeviceInfo()->serialNumber) + ".txt", "a");
            std::string msg = (((diag_msg_t *)dataBuf)->message);
            outfile->write(msg.c_str(), msg.length());
            if (msg.length() > 0 && *msg.end() != '\n')
            {
                outfile->putch('\n');
            }
            CloseISLogFile(outfile);
        }
    }
#endif
    return true;
}

bool cISLogger::LogData(std::shared_ptr<cDeviceLog> deviceLog, int dataSize, const uint8_t *dataBuf)
{
    // This method is ONLY for LOGTYPE_RAW
    if (!m_enabled || (deviceLog == nullptr) || (m_logType != LOGTYPE_RAW)) {
        return false;
    }

    if (deviceLog == NULL || dataSize <= 0 || dataBuf == NULL)
    {
        m_errorFile.lprintf("Invalid device handle or NULL data\r\n");
        return false;
    }

    m_lastCommTime = GetTime();
    if (!deviceLog->SaveData(dataSize, dataBuf, m_logStats))
    {	// Save Error
        m_errorFile.lprintf("Underlying log implementation failed to save\r\n");
        m_logStats.LogError(NULL);
    }
    else
    {	// Success

    }
    return true;
}

p_data_buf_t *cISLogger::ReadData(std::shared_ptr<cDeviceLog> deviceLog)
{
    if (deviceLog == nullptr) {
        return NULL;
    }

    p_data_buf_t *data = NULL;
    while (isDataCorrupt(data = deviceLog->ReadData()))
    {
        m_errorFile.lprintf("Corrupt log header, id: %lu, offset: %lu, size: %lu\r\n", (unsigned long)data->hdr.id, (unsigned long)data->hdr.offset, (unsigned long)data->hdr.size);
        m_logStats.LogError(&data->hdr);
        data = NULL;
    }
    if (data != NULL)
    {
        m_logStats.LogData(_PTYPE_INERTIAL_SENSE_DATA, data->hdr.id, cISDataMappings::Timestamp(&data->hdr, data->buf), ISB_HDR_TO_PACKET_SIZE(data->hdr));
    }
    return data;
}

p_data_buf_t *cISLogger::ReadData(size_t devIndex) 
{
    if (devIndex >= m_devices.size())
        return nullptr;

    return ReadData(m_devices[devIndex]);
}

p_data_buf_t *cISLogger::ReadNextData(size_t& devIndex)
{
    while (devIndex < m_devices.size())
    {
        p_data_buf_t *data = ReadData(devIndex);
        if (data == NULL)
        {
            ++devIndex;
        }
        else
        {
            return data;
        }
    }
    return NULL;
}

void cISLogger::CloseAllFiles()
{
    // PrintStatistics();
    PrintIsCommStatus();

    for (auto it : m_devices)
    {
        if (it.second != nullptr)
            it.second->CloseAllFiles();
    }

    m_logStats.WriteToFile(m_directory + STATS_ALL_FILENAME);
    m_errorFile.close();
}

void cISLogger::FlushToFile()
{
    for (auto it : m_devices)
    {
        it.second->FlushToFile();
    }
}

void cISLogger::OpenWithSystemApp()
{
    for (auto it : m_devices)
    {
        it.second->OpenWithSystemApp();
    }
}

void cISLogger::ShowParseErrors(bool show)
{
    for (auto it : m_devices)
    {
        it.second->ShowParseErrors(show);
    }
    m_showParseErrors = show;
}

uint64_t cISLogger::LogSize(uint32_t devSerialNo)
{
    auto device = DeviceLogBySerialNumber(devSerialNo);
    return (device ? device->LogSize() : 0);
}

uint64_t cISLogger::LogSizeAll()
{
    uint64_t size = 0;
    for (auto it : m_devices)
    {
        size += it.second->LogSize();
    }
    return size;
}

float cISLogger::LogSizeAllMB()
{
    return LogSizeAll() * 0.000001f;
}

float cISLogger::LogSizeMB(uint32_t devSerialNo)
{
    auto device = DeviceLogBySerialNumber(devSerialNo);
    return (device ? device->LogSize() * 0.000001f : 0);
}

float cISLogger::FileSizeMB(uint32_t devSerialNo)
{
    auto device = DeviceLogBySerialNumber(devSerialNo);
    return (device ? device->FileSize() * 0.000001f : 0);
}

uint32_t cISLogger::FileCount(uint32_t devSerialNo)
{
    auto device = DeviceLogBySerialNumber(devSerialNo);
    return (device ? device->FileCount() : 0);
}

std::string cISLogger::GetNewFileName(uint32_t devSerialNo, uint32_t fileCount, const char *suffix)
{
    auto device = DeviceLogBySerialNumber(devSerialNo);
    return (device ? device->GetNewFileName(devSerialNo, fileCount, suffix) : "");
}

int g_copyReadCount;
int g_copyReadDid;

bool cISLogger::CopyLog(cISLogger &log, const string &timestamp, const string &outputDir, eLogType logType, uint32_t maxFileSize, float driveUsageLimitPercent, bool useSubFolderTimestamp, bool enableCsvIns2ToIns1Conversion)
{
    m_logStats.Clear();

    sSaveOptions options;
    options.logType                 = logType;
    options.driveUsageLimitPercent  = driveUsageLimitPercent;
    options.maxFileSize             = maxFileSize;
    options.useSubFolderTimestamp   = useSubFolderTimestamp;
    options.timeStamp               = timestamp;
    if (!InitSave(outputDir, options))
    {
        return false;
    }

    is_comm_instance_t comm;
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf));

    EnableLogging(true);
    p_data_buf_t *data = NULL;
    for ( auto& srcDev : log.DeviceLogs() )
    {
        auto dstDev = ( srcDev->Device() != nullptr ? registerDevice(*(srcDev->Device())) : registerDevice(0, srcDev->SerialNumber()) );

#if LOG_DEBUG_GEN == 2
        // Don't print status here
#elif LOG_DEBUG_GEN || DEBUG_PRINT
        printf("cISLogger::CopyLog SN%d type %d, (%d of %d)\n", devInfo->serialNumber, logType, dev + 1, log.DeviceCount());
#endif

        // Set KML configuration
        dstDev->SetKmlConfig(m_gpsData, m_showPath, m_showSample, m_showTimeStamp, m_iconUpdatePeriodSec, m_altClampToGround);

        // Copy data
        for (g_copyReadCount = 0; (data = log.ReadData(srcDev)); g_copyReadCount++)
        {

#if LOG_DEBUG_PRINT_READ
            double timestamp = cISDataMappings::Timestamp(&(data->hdr), data->buf);
            printf("read: %d DID: %3d time: %.4lf\n", g_copyReadCount, data->hdr.id, timestamp);
            g_copyReadDid = data->hdr.id;
#endif

#if LOG_DEBUG_GEN == 2
            PrintProgress();
#endif
            // CSV special cases 
            if (logType == eLogType::LOGTYPE_CSV && enableCsvIns2ToIns1Conversion)
            {
                static bool hasIns1 = false;
                if (data->hdr.id == DID_INS_1)
                {   // Indicate log contains DID_INS_1 so we don't need to convert from DID_INS_2
                    hasIns1 = true;
                }
                if (data->hdr.id == DID_INS_2 && !hasIns1)
                {	// Convert INS2 to INS1 when creating .csv logs
                    ins_1_t ins1;
                    ins_2_t ins2;

                    copyDataBufPToStructP(&ins2, data, sizeof(ins_2_t));
                    convertIns2ToIns1(&ins2, &ins1);

                    p_data_hdr_t hdr;
                    hdr.id = DID_INS_1;
                    hdr.size = sizeof(ins_1_t);
                    hdr.offset = 0;
                    LogData(dstDev, &hdr, (uint8_t *)&ins1);
                }
            }

            // Save data
            if (logType == LOGTYPE_RAW)
            {	// Encode data into to ISB packet
                int pktSize = is_comm_data_to_buf(comm.rxBuf.start, comm.rxBuf.size, &comm, data->hdr.id, data->hdr.size, data->hdr.offset, data->buf);
                LogData(dstDev, pktSize, comm.rxBuf.start);
            }
            else
            {
                LogData(dstDev, &data->hdr, data->buf);
            }
        }
    }
    CloseAllFiles();
    return true;
}

void cISLogger::PrintProgress()
{
#if (LOG_DEBUG_GEN == 2)
#if 0
    advance_cursor();
#else
    if (++m_progress > 50000)
    {
        m_progress = 0;
        printf(".");
        fflush(stdout);
    }
#endif
#endif
}

void cISLogger::PrintStatistics()
{
    return;
    
    for (auto it : m_devices)
    {   // Print message statistics 
        std::shared_ptr<cDeviceLog> dev = it.second;
        if (dev==NULL)
            continue;
        cout << endl << "SN" << std::setw(6) << dev->SerialNumber() << " " << dev->LogStatsString();
    }

    PrintIsCommStatus();
}

void cISLogger::PrintIsCommStatus()
{
    for (auto it : m_devices)
    {   // Print errors
        std::shared_ptr<cDeviceLog> dev = it.second;
        if (dev==NULL)
            continue;
        // cout << endl << "SN" << std::setw(6) << dev->SerialNumber() << " " << cInertialSenseDisplay::PrintIsCommStatus(dev->IsCommInstance());
    }
}

void cISLogger::PrintLogDiskUsage()
{
    float logSize = LogSizeAll();

    // Compute elapsed time since logging started
    time_t elapsed = GetTime() - m_logStartTime;
    int hours = elapsed / 3600;
    int minutes = (elapsed % 3600) / 60;
    int seconds = elapsed % 60;

    if (logSize < 0.5e6f)
        printf("\nLogging %d:%02d:%02ds %5.1f KB to: %s", hours, minutes, seconds, logSize * 1.0e-3f, LogDirectory().c_str());
    else
        printf("\nLogging %d:%02d:%02ds %5.2f MB to: %s", hours, minutes, seconds, logSize * 1.0e-6f, LogDirectory().c_str());

    // Disk usage
    if (MaxDiskSpaceMB() > 0.0f)
    {   // Limit enabled
        float percentUsed = 100.0f * UsedDiskSpaceMB() / MaxDiskSpaceMB();
        printf("      %s disk usage/limit: %.0f/%.0f MB (%.0f%%) ", RootDirectory().c_str(), UsedDiskSpaceMB(), MaxDiskSpaceMB(), percentUsed);
        if (percentUsed > 98)
        {
            printf("...deleting old logs ");
        }
    }
    else
    {   // Limit disabled
        printf(",    %s disk usage: %.0f MB ", RootDirectory().c_str(), UsedDiskSpaceMB());
    }
    printf("\n");
}

std::vector<std::shared_ptr<cDeviceLog>> cISLogger::DeviceLogs() 
{
    std::vector<std::shared_ptr<cDeviceLog>> out;
    for (auto it : m_devices) {
        out.push_back(it.second);
    }
    return out;
}
