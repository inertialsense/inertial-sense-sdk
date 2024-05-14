/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

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


const string cISLogger::g_emptyString;

#if !PLATFORM_IS_EMBEDDED
static std::mutex g_devices_mutex;
#endif

bool cISLogger::isHeaderCorrupt(const p_data_hdr_t *hdr)
{
    bool isCorrupt = true;

    if (hdr != NULL)
    {
        // if any case is true this is corrupt
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
#if !PLATFORM_IS_EMBEDDED
    const std::lock_guard<std::mutex> lock(g_devices_mutex);
#endif
    m_devices.clear();
    m_logStats.Clear();
}


void cISLogger::Update()
{
    // if we have a timeout param and the time has elapsed, shutdown
    if (m_lastCommTime == 0)
    {
        m_lastCommTime = GetTime();
    }
    else if (m_timeoutFlushSeconds > 0 && GetTime() - m_lastCommTime >= m_timeoutFlushSeconds)
    {
        for (auto it : m_devices)
        {
            it.second->Flush();
        }
    }
    ISFileManager::TouchFile(m_directory + "/stats.txt");
}


bool cISLogger::InitSaveCommon(eLogType logType, const string &directory, const string &subDirectory, float maxDiskSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
    static const int minFileCount = 50;
    static const int maxFileCount = 10000;

    // Close any open files
    CloseAllFiles();

    m_logType = logType;
    m_directory = (directory.empty() ? DEFAULT_LOGS_DIRECTORY : directory);
    maxDiskSpacePercent = _CLAMP(maxDiskSpacePercent, 0.01f, 0.99f);
    uint64_t availableSpace = ISFileManager::GetDirectorySpaceAvailable(m_directory);
    m_maxDiskSpace = (maxDiskSpacePercent <= 0.0f ? availableSpace : (uint64_t)(availableSpace * maxDiskSpacePercent));

    // ensure there are between min and max file count
    if (maxFileSize > m_maxDiskSpace / minFileCount)
    {
        m_maxFileSize = (uint32_t)(m_maxDiskSpace / minFileCount);
    }
    else if (maxFileSize < m_maxDiskSpace / maxFileCount)
    {
        m_maxFileSize = (uint32_t)(m_maxDiskSpace / maxFileCount);
    }
    else
    {
        m_maxFileSize = maxFileSize;
    }

    m_maxFileSize = _MIN(m_maxFileSize, maxFileSize);

    // create root dir
    _MKDIR(m_directory.c_str());

    if (useSubFolderTimestamp)
    {
        // create time stamp dir
        m_directory = directory + "/" + m_timeStamp;
        _MKDIR(m_directory.c_str());

        if (!subDirectory.empty())
        {
            // create sub dir
            m_directory += "/" + subDirectory;
            _MKDIR(m_directory.c_str());
        }
    }

    // create empty stats file to track timestamps
    string str = m_directory + (subDirectory.empty() ? "" : "/" + subDirectory) + "/stats.txt";
    cISLogFileBase *statsFile = CreateISLogFile(str, "w");
    CloseISLogFile(statsFile);

    // Initialize devices
    // return InitDevicesForWriting(numDevices); // Lazy Initialize of devices (when they are explicitly added)
    return ISFileManager::PathIsDir(m_directory);
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


bool cISLogger::InitSave(eLogType logType, const string &directory, float maxDiskSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
    m_timeStamp = CreateCurrentTimestamp();
    return InitSaveCommon(logType, directory, g_emptyString, maxDiskSpacePercent, maxFileSize, useSubFolderTimestamp);
}


bool cISLogger::InitSaveTimestamp(const string &timeStamp, const string &directory, const string &subDirectory, eLogType logType, float maxDiskSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp)
{
    if (timeStamp.length() == 0)
    {
        m_timeStamp = CreateCurrentTimestamp();
    }
    else
    {
        // Only use first 15 characters for the timestamp
        // m_timeStamp = timeStamp.substr(0, IS_LOG_TIMESTAMP_LENGTH);
        m_timeStamp = timeStamp;
    }

    return InitSaveCommon(logType, directory, subDirectory, maxDiskSpacePercent, maxFileSize, useSubFolderTimestamp);
}

std::shared_ptr<cDeviceLog> cISLogger::registerDevice(ISDevice& device) {
    switch (m_logType)
    {
        default:
        case LOGTYPE_DAT:   device.devLogger = make_shared<cDeviceLogSerial>(&device);  break;
        case LOGTYPE_RAW:   device.devLogger = make_shared<cDeviceLogRaw>(&device);     break;
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
        case LOGTYPE_SDAT:  device.devLogger = make_shared<cDeviceLogSorted>(&device);  break;
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
        case LOGTYPE_SDAT:  deviceLog = make_shared<cDeviceLogSorted>(hdwId, serialNo);  break;
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
#if !PLATFORM_IS_EMBEDDED
        const std::lock_guard<std::mutex> lock(g_devices_mutex);
#endif
        // for (int i = 0; i < numDevices; i++)
        for (auto& d : devices) {
            registerDevice(d);
        }
    }

    //m_errorFile = CreateISLogFile((m_directory + "/errors.txt"), "w");
    //m_errorFile.open((m_directory + "/errors.txt"), "w");

    return ISFileManager::PathIsDir(m_directory);
}


bool nextStreamDigit(stringstream &ss, string &str)
{
    char c;
    if (!(ss >> c))	// Read delimiter
    {
        return false;
    }

    if (isdigit(c))
    {	// If not delimeter, put first char/digit back
        ss.unget();
    }

    if (!getline(ss, str, '_'))
    {
        return false;	// No more data 
    }

    return true;
}


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
        if (!nextStreamDigit(ss, str) && str.size()) { return false; } 	serialNum = stoi(str);
        if (!nextStreamDigit(ss, str) && str.size()) { return false; } 	date = str;
        if (!nextStreamDigit(ss, str) && str.size()) { return false; } 	time = str;
        if (!nextStreamDigit(ss, str) && str.size()) { return false; } 	index = stoi(str);
    }
    else
    {	// No prefix - only index number
        stringstream ss(content);
        if (!nextStreamDigit(ss, str) && str.size()) { return false; } 	index = stoi(str);
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
        ParseFilename(name, serialNum, date, time, index);

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
#if !PLATFORM_IS_EMBEDDED
                        const std::lock_guard<std::mutex> lock(g_devices_mutex);
#endif
                        std::shared_ptr<cDeviceLog> deviceLog;
                        switch (logType)
                        {
                        default:
                        case cISLogger::LOGTYPE_DAT:    deviceLog = make_shared<cDeviceLogSerial>(0, serialNum); break;
                        case cISLogger::LOGTYPE_RAW:    deviceLog = make_shared<cDeviceLogRaw>(0, serialNum); break;
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
                        case cISLogger::LOGTYPE_SDAT:   deviceLog = make_shared<cDeviceLogSorted>(0, serialNum); break;
                        case cISLogger::LOGTYPE_CSV:    deviceLog = make_shared<cDeviceLogCSV>(0, serialNum); break;
                        case cISLogger::LOGTYPE_JSON:   deviceLog = make_shared<cDeviceLogJSON>(0, serialNum); break;
#endif
                        }
                        deviceLog->SetupReadInfo(directory, serialNumber, m_timeStamp);
                        m_devices[serialNum] = deviceLog;
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
        double timestamp = cISDataMappings::GetTimestamp(dataHdr, dataBuf);
        m_logStats.LogDataAndTimestamp(dataHdr->id, timestamp);

        if (dataHdr->id == DID_DIAGNOSTIC_MESSAGE)
        {
            // write to diagnostic text file
#if 0        
            std::ostringstream outFilePath;
            outFilePath << m_directory << "/diagnostic_" << m_devices[device]->DeviceInfo()->serialNumber << ".txt";
            cISLogFileBase *outfile = CreateISLogFile(outFilePath.str(), "a");
#else
            cISLogFileBase *outfile = CreateISLogFile(m_directory + "/diagnostic_" + std::to_string(deviceLog->DeviceInfo()->serialNumber) + ".txt", "a");
#endif            
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
        double timestamp = cISDataMappings::GetTimestamp(&data->hdr, data->buf);
        m_logStats.LogDataAndTimestamp(data->hdr.id, timestamp);
    }
    return data;
}

p_data_buf_t *cISLogger::ReadData(size_t devIndex) {
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
    for (auto it : m_devices)
    {
        if (it.second != nullptr)
            it.second->CloseAllFiles();
    }

    m_logStats.WriteToFile(m_directory + "/stats.txt");
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
    return m_devices.contains(devSerialNo) ? m_devices[devSerialNo]->LogSize() : -1;
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
    return m_devices.contains(devSerialNo) ? m_devices[devSerialNo]->LogSize() * 0.000001f : 0;
}


float cISLogger::FileSizeMB(uint32_t devSerialNo)
{
    return m_devices.contains(devSerialNo) ? m_devices[devSerialNo]->FileSize() * 0.000001f : 0;
}


uint32_t cISLogger::FileCount(uint32_t devSerialNo)
{
    return m_devices.contains(devSerialNo) ? m_devices[devSerialNo]->FileCount() : 0;
}

std::string cISLogger::GetNewFileName(uint32_t devSerialNo, uint32_t fileCount, const char *suffix)
{
    return m_devices.contains(devSerialNo) ? m_devices[devSerialNo]->GetNewFileName(devSerialNo, fileCount, suffix) : std::string("");
}

/*
bool cISLogger::SetDevice(const ISDevice *info)
{
    if (device >= m_devices.size() || info == NULL)
    {
        return false;
    }

    m_devices[device]->SetDeviceInfo(info);
    return true;
}

bool cISLogger::SetDeviceInfo(const dev_info_t *info, unsigned int device)
{
    if (device >= m_devices.size() || info == NULL)
    {
        return false;
    }

    m_devices[device]->SetDeviceInfo(info);
    return true;
}

const dev_info_t *cISLogger::DeviceInfo(unsigned int device)
{
    if (device >= m_devices.size())
    {
        return NULL;
    }

    return m_devices[device]->DeviceInfo();
}
*/

int g_copyReadCount;
int g_copyReadDid;

/**
 * FIXME:  Not sure what to do here... this seems to be making a copy of a cISLogger instance, but I'm not sure why...
 * @param log
 * @param timestamp
 * @param outputDir
 * @param logType
 * @param maxLogSpacePercent
 * @param maxFileSize
 * @param useSubFolderTimestamp
 * @param enableCsvIns2ToIns1Conversion
 * @return
 */
bool cISLogger::CopyLog(cISLogger &log, const string &timestamp, const string &outputDir, eLogType logType, float maxLogSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp, bool enableCsvIns2ToIns1Conversion)
{
    m_logStats.Clear();
    if (!InitSaveTimestamp(timestamp, outputDir, g_emptyString, logType, maxLogSpacePercent, maxFileSize, useSubFolderTimestamp))
    {
        return false;
    }

    is_comm_instance_t comm;
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf));

    EnableLogging(true);
    p_data_buf_t *data = NULL;
    // for (unsigned int dev = 0; dev < log.DeviceCount(); dev++)
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
            double timestamp = cISDataMappings::GetTimestamp(&(data->hdr), data->buf);
            printf("read: %d DID: %3d time: %.4lf\n", g_copyReadCount, data->hdr.id, timestamp);
            g_copyReadDid = data->hdr.id;
#endif

#if LOG_DEBUG_GEN == 2
            PrintProgress();
#endif

            // CSV special cases 
            if (logType == eLogType::LOGTYPE_CSV && enableCsvIns2ToIns1Conversion)
            {
                if (data->hdr.id == DID_INS_2)
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

/*
bool cISLogger::ReadAllLogDataIntoMemory(const string &directory, map<uint32_t, vector<vector<uint8_t>>> &data)
{
    cISLogger logger;
    if (!logger.LoadFromDirectory(directory))
    {
        return false;
    }
    unsigned int deviceId = 0;
    unsigned int lastDeviceId = 0xFFFFFEFE;
    p_data_buf_t *p;
    vector<vector<uint8_t>> *currentDeviceData = NULL;
    const dev_info_t *info;
    uint8_t *ptr, *ptrEnd;
    data.clear();

    // read every piece of data out of every device log
    while ((p = logger.ReadNextData(deviceId)) != NULL)
    {
        // if this is a new device, we need to add the device to the map using the serial number
        if (deviceId != lastDeviceId)
        {
            lastDeviceId = deviceId;
            // info = logger.DeviceInfo(deviceId);
            data[info->serialNumber] = vector<vector<uint8_t>>();
            currentDeviceData = &data[info->serialNumber];
        }

        assert(currentDeviceData != NULL);

        // add slots until we have slots at least equal to the data id
        while (currentDeviceData->size() < p->hdr.id)
        {
            currentDeviceData->push_back(vector<uint8_t>());
        }

        // append each byte of the packet to the slot
        ptrEnd = p->buf + p->hdr.size;
        vector<uint8_t> &stream = (*currentDeviceData)[p->hdr.id];
        for (ptr = p->buf; ptr != ptrEnd; ptr++)
        {
            stream.push_back(*ptr);
        }
    }
    return true;
}
 */

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


std::vector<std::shared_ptr<cDeviceLog>> cISLogger::DeviceLogs() {
    std::vector<std::shared_ptr<cDeviceLog>> out;
    for (auto it : m_devices) {
        out.push_back(it.second);
    }
    return out;
}
