/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

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
#include "DeviceLogSorted.h"
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
#define DEFAULT_LOGS_DIRECTORY "IS_logs"


class cISLogger
{
public:
	enum eLogType
	{
		LOGTYPE_DAT = 0,	// serial
		LOGTYPE_RAW,		// packetized serial
		LOGTYPE_SDAT,		// sorted
		LOGTYPE_CSV,
		LOGTYPE_KML,
		LOGTYPE_JSON
	};

	static const std::string g_emptyString;

	cISLogger();
	virtual ~cISLogger();

	// Setup logger to read from file.
	bool LoadFromDirectory(const std::string& directory, eLogType logType = LOGTYPE_DAT, std::vector<std::string> serials = {});

	// Setup logger for writing to file.
	bool InitSave(eLogType logType = LOGTYPE_DAT, const std::string& directory = g_emptyString, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, bool useSubFolderTimestamp = true);
	bool InitSaveTimestamp(const std::string& timeStamp, const std::string& directory = g_emptyString, const std::string& subDirectory = g_emptyString, eLogType logType = LOGTYPE_DAT, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, bool useSubFolderTimestamp = true);

    // Establish link between devices and this logger
    std::shared_ptr<cDeviceLog> registerDevice(ISDevice& device);
    std::shared_ptr<cDeviceLog> registerDevice(uint16_t hdwId, uint32_t serialNo);
    std::shared_ptr<cDeviceLog> registerDevice(dev_info_t& devInfo) { return registerDevice(ENCODE_DEV_INFO_TO_HDW_ID(devInfo), devInfo.serialNumber); }


	// update internal state, handle timeouts, etc.
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
	uint64_t LogSizeAll();
	uint64_t LogSize(uint32_t devSerialNo);
	float LogSizeAllMB();
	float LogSizeMB(uint32_t devSerialNo);
	float FileSizeMB(uint32_t devSerialNo);
	uint32_t FileCount(uint32_t devSerialNo);
	std::string GetNewFileName(uint32_t devSerialNo, uint32_t fileCount, const char* suffix);
    std::vector<std::shared_ptr<cDeviceLog>> DeviceLogs();
	uint32_t DeviceCount() { return (uint32_t)m_devices.size(); }
    std::shared_ptr<cDeviceLog> DeviceLogBySerialNumber(uint32_t serialNo) {
        return (m_devices.count(serialNo) ? m_devices[serialNo] : nullptr);
    }
	// bool SetDeviceInfo(const dev_info_t *info, unsigned int device = 0);
	// const dev_info_t* DeviceInfo(unsigned int device = 0);

	bool CopyLog(
		cISLogger& log,
		const std::string& timestamp = g_emptyString,
		const std::string& outputDir = g_emptyString,
		eLogType logType = LOGTYPE_DAT,
		float maxDiskSpacePercent = 0.5f,
		uint32_t maxFileSize = 1024 * 1024 * 5,
		bool useSubFolderTimestamp = true,
		bool enableCsvIns2ToIns1Conversion = true);
	const cLogStats& GetStats() { return m_logStats; }
	eLogType GetType() { return m_logType; }

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

private:
#if CPP11_IS_ENABLED
    cISLogger(const cISLogger& copy) = delete;
#else
	cISLogger(const cISLogger& copy); // Disable copy constructors
#endif

	bool InitSaveCommon(eLogType logType, const std::string& directory, const std::string& subDirectory, float maxDiskSpacePercent, uint32_t maxFileSize, bool useSubFolderTimestamp);
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
	std::string				m_directory;
	std::string				m_timeStamp;
	std::map<uint32_t, std::shared_ptr<cDeviceLog>> m_devices = { };

	uint64_t				m_maxDiskSpace = 0;
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
	time_t					m_lastCommTime = 0;
	time_t					m_timeoutFlushSeconds = 0;
	int						m_progress = 0;
	bool					m_showParseErrors = true;
};


#endif // IS_LOGGER_H
