/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

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

#include "DeviceLogSerial.h"
#include "DeviceLogSorted.h"
#include "DeviceLogCSV.h"
#include "DeviceLogKML.h"
#include "ISConstants.h"
#include "ISDisplay.h"

#if defined(ENABLE_IS_PYTHON_WRAPPER)

#include "../pySDK/pySDK.h"

#endif

using namespace std;

// default logging path if none specified
#define DEFAULT_LOGS_DIRECTORY "IS_logs"

#if PLATFORM_IS_WINDOWS

#include <direct.h>
#define _MKDIR(dir) _mkdir(dir)
#define _RMDIR(dir) _rmdir(dir)
#define _GETCWD(buf, len) _getcwd(buf, len)
#define FLOAT2DOUBLE // Used to prevent warning when compiling with -Wdouble-promotion in Linux

#else

#include <unistd.h>
#include <dirent.h>
#include <errno.h>
//#define _MKDIR(dir) mkdir(dir, S_IRWXU) // 777 owner access only 
#define _MKDIR(dir) mkdir(dir, ACCESSPERMS) // 0777 access for all
#define _RMDIR(dir) rmdir(dir)
#define _GETCWD(buf, len) getcwd(buf, len)
#define FLOAT2DOUBLE (double) // Used to prevent warning when compiling with -Wdouble-promotion in Linux

#endif

typedef struct
{
	string name;
	uint64_t size;
	time_t lastModificationDate;
} file_info_t;

class cLogStats;

class cISLogger
{
public:
	enum eLogType
	{
		LOGTYPE_DAT = 0,
		LOGTYPE_SDAT,
		LOGTYPE_CSV,
		LOGTYPE_KML, 
	};

	static const string g_emptyString;

	cISLogger();
	virtual ~cISLogger();

	// Setup logger to read from file.
	bool LoadFromDirectory(const string& directory, eLogType logType = LOGTYPE_DAT);

	// Setup logger for writing to file.
	bool InitSave(eLogType logType = LOGTYPE_DAT, const string& directory = g_emptyString, int numDevices = 1, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, uint32_t maxChunkSize = 131072, bool useSubFolderTimestamp = true);
	bool InitSaveTimestamp(const string& timeStamp, const string& directory = g_emptyString, const string& subDirectory = g_emptyString, int numDevices = 1, eLogType logType = LOGTYPE_DAT, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, uint32_t maxChunkSize = 131072, bool useSubFolderTimestamp = true);

    bool LogData(unsigned int device, p_data_hdr_t* dataHdr, const uint8_t* dataBuf);
	p_data_t* ReadData(unsigned int device = 0);
	p_data_t* ReadNextData(unsigned int& device);
	void EnableLogging(bool enabled) { m_enabled = enabled; }
	bool Enabled() { return m_enabled; }
	void CloseAllFiles();
	void OpenWithSystemApp();
	string TimeStamp() { return m_timeStamp; }
	string LogDirectory() { return m_directory; }
    uint64_t LogSizeAll();
	uint64_t LogSize(unsigned int device = 0);
    float LogSizeAllMB();
	float LogSizeMB(unsigned int device = 0);
	float FileSizeMB(unsigned int device = 0);
	uint32_t FileCount(unsigned int device = 0);
	uint32_t GetDeviceCount() { return (uint32_t)m_devices.size(); }
	bool SetDeviceInfo(const dev_info_t *info, unsigned int device = 0);
	const dev_info_t* GetDeviceInfo(unsigned int device = 0);
	bool CopyLog(cISLogger& log, const string& timestamp = g_emptyString, const string& outputDir = g_emptyString, eLogType logType = LOGTYPE_DAT, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, uint32_t maxChunkSize = 131072, bool useSubFolderTimestamp = true);
	const cLogStats& GetStats() { return *m_logStats; }

	// get all files in a folder - files parameter is not cleared in this function
	// files contains the full path to the file
	// return false if no files found, true otherwise
	static bool GetAllFilesInDirectory(const string& directory, bool recursive, vector<string>& files);
	static bool GetAllFilesInDirectory(const string& directory, bool recursive, const string& regexPattern, vector<string>& files);

	// delete a directory, and optionally all files and sub-directories
	static void DeleteDirectory(const string& directory, bool recursive = true);

	// get space used in a directory recursively - files is filled with all files in the directory, sorted by modification date, files is NOT cleared beforehand. sortByDate of false sorts by file name
	static uint64_t GetDirectorySpaceUsed(const string& directory, bool recursive = true);
	static uint64_t GetDirectorySpaceUsed(const string& directory, vector<file_info_t>& files, bool sortByDate = true, bool recursive = true);
	static uint64_t GetDirectorySpaceUsed(const string& directory, string regexPattern, vector<file_info_t>& files, bool sortByDate = true, bool recursive = true);

	// get free space for the disk that the specified directory exists on
	static uint64_t GetDirectorySpaceAvailable(const string& directory);

	// get just the file name from a path
	static string GetFileName(const string& path);

    // check if a data header is corrupt
    static bool LogHeaderIsCorrupt(const p_data_hdr_t* hdr);

	// create a timestamp
	static string CreateCurrentTimestamp();

    // check if a data packet is corrupt, NULL data is OK
    static bool LogDataIsCorrupt(const p_data_t* data);

    // read all log data into memory - if the log is over 1.5 GB this will fail on 32 bit processess
    // the map contains device id (serial number) key and a vector containing log data for each data id, which will be an empty vector if no log data for that id
    static bool ReadAllLogDataIntoMemory(const string& directory, map<uint32_t, vector<vector<uint8_t>>>& data);

	void SetKmlConfig(bool showPath = true, bool showTimeStamp = true, double updatePeriodSec = 1.0, bool altClampToGround = true)
	{
		m_showPath = showPath;
		m_showTimeStamp = showTimeStamp;
		m_iconUpdatePeriodSec = updatePeriodSec;
		m_altClampToGround = altClampToGround;
	}

private:
	bool InitSaveCommon(eLogType logType, const string& directory, const string& subDirectory, int numDevices, float maxDiskSpacePercent, uint32_t maxFileSize, uint32_t chunkSize, bool useSubFolderTimestamp);
	bool InitDevicesForWriting(int numDevices = 1);
	void Cleanup();

	eLogType				m_logType;
	bool					m_enabled;
	string					m_directory;
	string					m_timeStamp;
	vector<cDeviceLog*>		m_devices;
	uint64_t				m_maxDiskSpace;
	uint32_t				m_maxFileSize;
	uint32_t				m_maxChunkSize;
	cLogStats*				m_logStats;
	FILE*					m_errorFile;

	bool					m_altClampToGround;
	bool					m_showPath;
	bool					m_showTimeStamp;
	double					m_iconUpdatePeriodSec;

#if defined(ENABLE_IS_PYTHON_WRAPPER)

    cInertialSenseDisplay	m_pyDisplay;

public:
	/*!
	* Set the Display variable for the pyCallback
	* @param python callback function
	*/
	void SetPyDisplay(cInertialSenseDisplay display);

	pybind11::dict PyReadData(unsigned int device = 0);

#endif

};

class cLogStatDataId
{
public:
	uint64_t count; // count for this data id
	uint64_t errorCount; // error count for this data id
	double averageTimeDelta; // average time delta for the data id
	double totalTimeDelta; // sum of all time deltas
	double lastTimestamp;
	double lastTimestampDelta;
	double maxTimestampDelta;
	uint64_t timestampDeltaCount;
	uint64_t timestampDropCount; // count of delta timestamps > 50% different from previous delta timestamp

	cLogStatDataId();
	void LogTimestamp(double timestamp);
	void Printf();
};

class cLogStats
{
public:
	map<uint32_t, cLogStatDataId> dataIdStats;
	uint64_t count; // count of all data ids
	uint64_t errorCount; // total error count

	cLogStats();
	void Clear();
	void LogError(const p_data_hdr_t* hdr);
	void LogData(uint32_t dataId);
	void LogDataAndTimestamp(uint32_t dataId, double timestamp);
	void Printf();
    void WriteToFile(const string& fileName);
};

#endif // IS_LOGGER_H
