/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

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
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <regex>
#include <set>

#include "ISLogger.h"
#include "ISDataMappings.h"
#include "InertialSense.h"

#if PLATFORM_IS_LINUX

#include <sys/statvfs.h>

#endif

const string cISLogger::g_emptyString;

bool cISLogger::LogHeaderIsCorrupt(const p_data_hdr_t* hdr)
{
    bool corrupt = (hdr != NULL &&
    (
        hdr->size == 0 ||
        hdr->offset + hdr->size > MAX_DATASET_SIZE ||
        hdr->id == 0 ||
        hdr->id >= DID_COUNT ||
        hdr->offset % 4 != 0 ||
        hdr->size % 4 != 0 ||
        (cISDataMappings::GetSize(hdr->id) > 0 && hdr->offset + hdr->size > cISDataMappings::GetSize(hdr->id))
    ));
	return corrupt;
}

bool cISLogger::LogDataIsCorrupt(const p_data_t* data)
{
    return (data != NULL && LogHeaderIsCorrupt(&data->hdr));
}

cISLogger::cISLogger()
{
	m_enabled = false;
	m_logStats = new cLogStats;
	m_logStats->Clear();
	m_errorFile = NULL;
}


cISLogger::~cISLogger()
{
	Cleanup();
	delete m_logStats;
}


void cISLogger::Cleanup()
{
	// Delete old devices
	for (unsigned int i = 0; i < m_devices.size(); i++)
	{
		if (m_devices[i] != NULL)
		{
			delete m_devices[i];
		}
	}
	m_devices.clear();
	m_logStats->Clear();
	if (m_errorFile != NULL)
	{
		fclose(m_errorFile);
		m_errorFile = NULL;
	}
}


bool cISLogger::InitSaveCommon(eLogType logType, const string& directory, const string& subDirectory, int numDevices, float maxDiskSpacePercent, uint32_t maxFileSize, uint32_t maxChunkSize, bool useSubFolderTimestamp)
{
	static const int minFileCount = 50;
	static const int maxFileCount = 10000;

	// Close any open files
	CloseAllFiles();

	m_logType = logType;
	m_directory = directory;
    maxDiskSpacePercent = _CLAMP(maxDiskSpacePercent, 0.01f, 0.99f);
	uint64_t availableSpace = cISLogger::GetDirectorySpaceAvailable(m_directory);
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

	// make sure chunk size is valid
	if (maxChunkSize > m_maxFileSize)
	{
		m_maxChunkSize = _MIN(m_maxFileSize, MAX_CHUNK_SIZE);
	}
	else
	{
		m_maxChunkSize = _MIN(maxChunkSize, MAX_CHUNK_SIZE);
	}

	if (m_directory.empty())
	{
		// Create local default directory
		m_directory = DEFAULT_LOGS_DIRECTORY;
	}

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

	// Initialize devices
	return InitDevicesForWriting(numDevices);
}


string cISLogger::CreateCurrentTimestamp()
{
	// Create timestamp
	char buf[80];
	time_t rawtime;
	time(&rawtime);
	struct tm * timeinfo = localtime(&rawtime);
	strftime(buf, 80, "%Y%m%d_%H%M%S", timeinfo);
	return string(buf);
}


bool cISLogger::InitSave(eLogType logType, const string& directory, int numDevices, float maxDiskSpacePercent, uint32_t maxFileSize, uint32_t maxChunkSize, bool useSubFolderTimestamp)
{
	m_timeStamp = CreateCurrentTimestamp();
	return InitSaveCommon(logType, directory, g_emptyString, numDevices, maxDiskSpacePercent, maxFileSize, maxChunkSize, useSubFolderTimestamp);
}


bool cISLogger::InitSaveTimestamp(const string& timeStamp, const string& directory, const string& subDirectory, int numDevices, eLogType logType, float maxDiskSpacePercent, uint32_t maxFileSize, uint32_t maxChunkSize, bool useSubFolderTimestamp)
{
	if (timeStamp.length() == 0)
	{
		m_timeStamp = CreateCurrentTimestamp();
	}
	else
	{
		// Only use first 15 characters for the timestamp
		m_timeStamp = timeStamp.substr(0, IS_LOG_TIMESTAMP_LENGTH);
	}

	return InitSaveCommon(logType, directory, subDirectory, numDevices, maxDiskSpacePercent, maxFileSize, maxChunkSize, useSubFolderTimestamp);
}


bool cISLogger::InitDevicesForWriting(int numDevices)
{
	Cleanup();

	// Add devices
	m_devices.resize(numDevices);

	// Create and init new devices
	for (int i = 0; i < numDevices; i++)
	{
		switch (m_logType)
		{
		default:
		case LOGTYPE_DAT:	m_devices[i] = new cDeviceLogSerial();	break;
		case LOGTYPE_SDAT:	m_devices[i] = new cDeviceLogSorted();	break;
		case LOGTYPE_CSV:	m_devices[i] = new cDeviceLogCSV();		break;
		case LOGTYPE_KML:	m_devices[i] = new cDeviceLogKML();		break;
		}

		m_devices[i]->InitDeviceForWriting(i, m_timeStamp, m_directory, m_maxDiskSpace, m_maxFileSize, m_maxChunkSize);
	}

#ifdef _MSC_VER

	fopen_s(&m_errorFile, (m_directory + "/errors.txt").c_str(), "wb");

#else

	m_errorFile = fopen((m_directory + "/errors.txt").c_str(), "wb");

#endif

	struct stat sb;
	return (stat(m_directory.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode));
}


bool cISLogger::GetAllFilesInDirectory(const string& directory, bool recursive, vector<string>& files)
{
	return GetAllFilesInDirectory(directory, recursive, g_emptyString, files);
}

bool cISLogger::GetAllFilesInDirectory(const string& directory, bool recursive, const string& regexPattern, vector<string>& files)
{
	size_t startSize = files.size();
	regex* rePtr = NULL;
	regex re;
	if (regexPattern.length() != 0)
	{
		re = regex(regexPattern, regex::icase);
		rePtr = &re;
	}

#if PLATFORM_IS_WINDOWS

	HANDLE dir;
	WIN32_FIND_DATAA file_data;
	if ((dir = FindFirstFileA((directory + "/*").c_str(), &file_data)) == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	do
	{
		string file_name = file_data.cFileName;
		string full_file_name = directory + "/" + file_name;
		if (file_name[0] == '.' || (rePtr != NULL && !regex_search(full_file_name, re)))
		{
			continue;
		}
		else if ((file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
		{
			if (recursive)
			{
				GetAllFilesInDirectory(full_file_name, true, files);
			}
			continue;
		}

		files.push_back(full_file_name);
	} while (FindNextFileA(dir, &file_data));
	FindClose(dir);

#else

	class dirent* ent;
	class stat st;
	DIR* dir = opendir(directory.c_str());

	if (dir == NULL)
	{
		return false;
	}

	while ((ent = readdir(dir)) != NULL)
	{
		const string file_name = ent->d_name;
		const string full_file_name = directory + "/" + file_name;

		// if file is current path or does not exist (-1) then continue
		if (file_name[0] == '.' || stat(full_file_name.c_str(), &st) == -1 || (rePtr != NULL && !regex_search(full_file_name, re)))
		{
			continue;
		}
		else if ((st.st_mode & S_IFDIR) != 0)
		{
			if (recursive)
			{
				GetAllFilesInDirectory(full_file_name, true, files);
			}
			continue;
		}

		files.push_back(full_file_name);
	}
	closedir(dir);

#endif

	return (files.size() != startSize);
}


void cISLogger::DeleteDirectory(const string& directory, bool recursive)
{

#if PLATFORM_IS_WINDOWS

	HANDLE dir;
	WIN32_FIND_DATAA file_data;
	if ((dir = FindFirstFileA((directory + "/*").c_str(), &file_data)) == INVALID_HANDLE_VALUE)
	{
		return;
	}
	do
	{
		const string file_name = file_data.cFileName;
		const string full_file_name = directory + "/" + file_name;
		const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
		if (file_name[0] == '.')
		{
			continue;
		}
		else if (is_directory)
		{
			if (recursive)
			{
				DeleteDirectory(full_file_name);
			}
			continue;
		}
		remove(full_file_name.c_str());
	} while (FindNextFileA(dir, &file_data));
	FindClose(dir);

#else

	class dirent* ent;
	class stat st;
 	
        DIR* dir;  
	if ((dir = opendir(directory.c_str())) == NULL) 
	{
		return;
	}

        while ((ent = readdir(dir)) != NULL)
	{
		const string file_name = ent->d_name;
		const string full_file_name = directory + "/" + file_name;

		// if file is current path or does not exist (-1) then continue
		if (file_name[0] == '.' || stat(full_file_name.c_str(), &st) == -1)
		{
			continue;
		}
		else if ((st.st_mode & S_IFDIR) != 0)
		{
			if (recursive)
			{
				DeleteDirectory(full_file_name.c_str());
			}
			continue;
		}
		remove(full_file_name.c_str());
	}
	closedir(dir);

#endif

	_RMDIR(directory.c_str());
}


uint64_t cISLogger::GetDirectorySpaceUsed(const string& directory, bool recursive)
{
	vector<file_info_t> files;
	return GetDirectorySpaceUsed(directory, "", files, true, recursive);
}


uint64_t cISLogger::GetDirectorySpaceUsed(const string& directory, vector<file_info_t>& files, bool sortByDate, bool recursive)
{
	return GetDirectorySpaceUsed(directory, "", files, sortByDate, recursive);
}


uint64_t cISLogger::GetDirectorySpaceUsed(const string& directory, string regexPattern, vector<file_info_t>& files, bool sortByDate, bool recursive)
{
	vector<string> fileNames;
	GetAllFilesInDirectory(directory, recursive, regexPattern, fileNames);
	uint64_t spaceUsed = 0;
	for (unsigned int i = 0; i < fileNames.size(); i++)
	{
		file_info_t info;
		info.name = fileNames[i];
		struct stat st;
		stat(info.name.c_str(), &st);
		info.size = st.st_size;
		info.lastModificationDate = st.st_mtime;
		files.push_back(info);
		spaceUsed += info.size;
	}

	if (sortByDate)
	{
		struct
		{
			bool operator()(const file_info_t& a, const file_info_t& b)
			{
				int compare = (a.lastModificationDate == b.lastModificationDate ? 0 : (a.lastModificationDate < b.lastModificationDate ? -1 : 1));
				if (compare == 0)
				{
					compare = a.name.compare(b.name);
				}
				return compare < 0;
			}
		} customSortDate;
		sort(files.begin(), files.end(), customSortDate);
	}
	else
	{
		struct
		{
			bool operator()(const file_info_t& a, const file_info_t& b)
			{
				return a.name < b.name;
			}
		} customSortName;
		sort(files.begin(), files.end(), customSortName);
	}

	return spaceUsed;
}


uint64_t cISLogger::GetDirectorySpaceAvailable(const string& directory)
{

#if PLATFORM_IS_WINDOWS

	ULARGE_INTEGER space;
	memset(&space, 0, sizeof(space));
	char fullPath[MAX_PATH];
	GetFullPathNameA(directory.c_str(), MAX_PATH, fullPath, NULL);
	bool created = (_MKDIR(fullPath) == 0);
	GetDiskFreeSpaceExA(fullPath, &space, NULL, NULL);
	if (created)
	{
		_RMDIR(fullPath);
	}
	return (uint64_t)space.QuadPart;

#else

	struct statvfs stat;
	memset(&stat, 0, sizeof(stat));
	char fullPath[PATH_MAX];
	if (realpath(directory.c_str(), fullPath) == NULL)
	{
		return 0;
	}
	bool created = (_MKDIR(fullPath) == 0);
	statvfs(fullPath, &stat);
	if (created)
	{
		_RMDIR(fullPath);
	}
	return (uint64_t)stat.f_bsize * (uint64_t)stat.f_bavail;

#endif

}

string cISLogger::GetFileName(const string& path)
{
	size_t lastSepIndex = path.find_last_of("\\/");
	if (lastSepIndex != string::npos)
	{
		return path.substr(lastSepIndex + 1);
	}
	return path;
}

bool cISLogger::LoadFromDirectory(const string& directory, eLogType logType)
{
	// Delete and clear prior devices
	Cleanup();
	m_logType = logType;
	string fileExtensionRegex;
	set<string> serialNumbers;

	switch (logType)
	{
	default:
	case cISLogger::LOGTYPE_DAT: fileExtensionRegex = "\\.dat$"; break;
	case cISLogger::LOGTYPE_SDAT: fileExtensionRegex = "\\.sdat$"; break;
	case cISLogger::LOGTYPE_CSV: fileExtensionRegex = "\\.csv$"; break;
	case cISLogger::LOGTYPE_KML: return false; // fileExtensionRegex = "\\.kml$"; break; // kml read not supported
	}

	// get all files, sorted by name
	vector<file_info_t> files;
	GetDirectorySpaceUsed(directory, fileExtensionRegex, files, false, false);
	if (files.size() == 0)
	{
		return false;
	}

	for (size_t i = 0; i < files.size(); i++)
	{
		string name = GetFileName(files[i].name);

		// check for log file prefix
		size_t endOfLogPrefixIndex = name.find(IS_LOG_FILE_PREFIX);
		if (endOfLogPrefixIndex != string::npos)
		{
			endOfLogPrefixIndex += IS_LOG_FILE_PREFIX_LENGTH;
			size_t serialNumberEndIndex = name.find('_', endOfLogPrefixIndex);
			if (serialNumberEndIndex != string::npos)
			{
				string serialNumber = name.substr(endOfLogPrefixIndex, serialNumberEndIndex - endOfLogPrefixIndex);

				// if we don't have a timestamp yet, see if we can parse it from the file, i.e. IS_LOG_FILE_PREFIX 30013_20170103_151023_001
				if (m_timeStamp.length() == 0)
				{
					// find _ at end of timestamp
					size_t timestampIndex = name.find_last_of('_');
					if (timestampIndex == string::npos)
					{
						timestampIndex = name.find_last_of('.');
					}
					if (timestampIndex != string::npos && timestampIndex - ++serialNumberEndIndex == IS_LOG_TIMESTAMP_LENGTH)
					{
						m_timeStamp = name.substr(timestampIndex - IS_LOG_TIMESTAMP_LENGTH, IS_LOG_TIMESTAMP_LENGTH);
					}
				}

				// check for unique serial numbers
				if (serialNumbers.find(serialNumber) == serialNumbers.end())
				{
					serialNumbers.insert(serialNumber);

					// Add devices
					switch (logType)
					{
					default:
					case cISLogger::LOGTYPE_DAT: m_devices.push_back(new cDeviceLogSerial()); break;
					case cISLogger::LOGTYPE_SDAT: m_devices.push_back(new cDeviceLogSorted()); break;
					case cISLogger::LOGTYPE_CSV: m_devices.push_back(new cDeviceLogCSV()); break;
					}
					m_devices.back()->SetupReadInfo(directory, serialNumber, m_timeStamp);
				}
			}
		}
	}

	for (uint32_t i = 0; i < m_devices.size(); i++)
	{
		m_devices[i]->InitDeviceForReading();
	}
	return (m_devices.size() != 0);
}


bool cISLogger::LogData(unsigned int device, p_data_hdr_t* dataHdr, const uint8_t* dataBuf)
{
	if (!m_enabled)
	{
		return false;
	}
	else if (device >= m_devices.size() || dataHdr == NULL || dataBuf == NULL)
	{
		if (m_errorFile != NULL)
		{
			fprintf(m_errorFile, "Error writing to log, log is not enabled, invalid device handle or NULL data");
		}
		return false;
	}
	else if (LogHeaderIsCorrupt(dataHdr))
	{
		if (m_errorFile != NULL)
		{
			fprintf(m_errorFile, "Corrupt log header, id: %lu, offset: %lu, size: %lu\r\n", (unsigned long)dataHdr->id, (unsigned long)dataHdr->offset, (unsigned long)dataHdr->size);
		}
		m_logStats->LogError(dataHdr);
	}
	else
	{
        if (!m_devices[device]->SaveData(dataHdr, dataBuf))
		{
			if (m_errorFile != NULL)
			{
				fprintf(m_errorFile, "Error logging data, underlying log implementation failed to save\r\n");
			}
			m_logStats->LogError(dataHdr);
		}
		else
		{
            double timestamp = cISDataMappings::GetTimestamp(dataHdr, dataBuf);
			m_logStats->LogDataAndTimestamp(dataHdr->id, timestamp);
		}
	}
	return true;
}


p_data_t* cISLogger::ReadData(unsigned int device)
{
	if (device >= m_devices.size())
	{
		return NULL;
	}

	p_data_t* data = NULL;
	while (LogDataIsCorrupt(data = m_devices[device]->ReadData()))
	{
		if (m_errorFile != NULL)
		{
			fprintf(m_errorFile, "Corrupt log header, id: %lu, offset: %lu, size: %lu\r\n", (unsigned long)data->hdr.id, (unsigned long)data->hdr.offset, (unsigned long)data->hdr.size);
		}
		m_logStats->LogError(&data->hdr);
		data = NULL;
	}
	if (data != NULL)
	{
        double timestamp = cISDataMappings::GetTimestamp(&data->hdr, data->buf);
		m_logStats->LogDataAndTimestamp(data->hdr.id, timestamp);
	}
	return data;
}


p_data_t* cISLogger::ReadNextData(unsigned int& device)
{
	while (device < m_devices.size())
	{
		p_data_t* data = ReadData(device);
		if (data == NULL)
		{
			++device;
		}
	}
	return NULL;
}


void cISLogger::CloseAllFiles()
{
	for( unsigned int i=0; i < m_devices.size(); i++ )
	{
		m_devices[i]->CloseAllFiles();
	}

    m_logStats->WriteToFile(m_directory + "/stats.txt");
    if (m_errorFile != NULL)
    {
        fclose(m_errorFile);
        m_errorFile = NULL;
    }
}


void cISLogger::OpenWithSystemApp()
{
	for (unsigned int i = 0; i < m_devices.size(); i++)
	{
		m_devices[i]->OpenWithSystemApp();
	}
}


uint64_t cISLogger::LogSizeAll()
{
    uint64_t size = 0;
    for (size_t i = 0; i < m_devices.size(); i++)
    {
        size += m_devices[i]->LogSize();
    }
    return size;
}


uint64_t cISLogger::LogSize( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return 0;
	}

	return m_devices[device]->LogSize();
}


float cISLogger::LogSizeAllMB()
{
    return LogSizeAll() * 0.000001f;
}


float cISLogger::LogSizeMB( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return 0;
	}

	return m_devices[device]->LogSize() * 0.000001f;
}


float cISLogger::FileSizeMB( unsigned int device )
{
    if (device >= m_devices.size())
    {
        return 0;
    }

    return m_devices[device]->FileSize() * 0.000001f;
}


uint32_t cISLogger::FileCount( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return 0;
	}

	return m_devices[device]->FileCount();
}

bool cISLogger::SetDeviceInfo(const dev_info_t *info, unsigned int device )
{
	if (device >= m_devices.size() || info == NULL)
	{
		return false;
	}

	m_devices[device]->SetDeviceInfo( info );
	return true;
}

const dev_info_t* cISLogger::GetDeviceInfo( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return NULL;
	}

	return m_devices[device]->GetDeviceInfo();
}

bool cISLogger::CopyLog(cISLogger& log, const string& timestamp, const string &outputDir, eLogType logType, float maxLogSpacePercent, uint32_t maxFileSize, uint32_t maxChunkSize, bool useSubFolderTimestamp)
{
	m_logStats->Clear();
	if (!InitSaveTimestamp(timestamp, outputDir, g_emptyString, log.GetDeviceCount(), logType, maxLogSpacePercent, maxFileSize, maxChunkSize, useSubFolderTimestamp))
	{
		return false;
	}
	EnableLogging(true);
	p_data_t* data = NULL;
	for (unsigned int dev = 0; dev < log.GetDeviceCount(); dev++)
	{
		// Copy device info
		const dev_info_t* devInfo = log.GetDeviceInfo(dev);
		SetDeviceInfo(devInfo, dev);

		// Set KML configuration
		m_devices[dev]->SetKmlConfig(m_showPath, m_showTimeStamp, m_iconUpdatePeriodSec, m_altClampToGround);

		// Copy data
		while ((data = log.ReadData(dev)))
		{
            LogData(dev, &data->hdr, data->buf);
		}
	}
	CloseAllFiles();
	return true;
}

bool cISLogger::ReadAllLogDataIntoMemory(const string& directory, map<uint32_t, vector<vector<uint8_t>>>& data)
{
    cISLogger logger;
    if (!logger.LoadFromDirectory(directory))
    {
        return false;
    }
    unsigned int deviceId = 0;
    unsigned int lastDeviceId = 0xFFFFFEFE;
    p_data_t* p;
	vector<vector<uint8_t>>* currentDeviceData = NULL;
    const dev_info_t* info;
    uint8_t* ptr, *ptrEnd;
    data.clear();

    // read every piece of data out of every device log
    while ((p = logger.ReadNextData(deviceId)) != NULL)
    {
        // if this is a new device, we need to add the device to the map using the serial number
        if (deviceId != lastDeviceId)
        {
            lastDeviceId = deviceId;
            info = logger.GetDeviceInfo(deviceId);
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
		vector<uint8_t>& stream = (*currentDeviceData)[p->hdr.id];
        for (ptr = p->buf; ptr != ptrEnd; ptr++)
        {
			stream.push_back(*ptr);
        }
    }
    return true;
}

cLogStatDataId::cLogStatDataId()
{
	count = 0;
	errorCount = 0;
	averageTimeDelta = 0.0;
	totalTimeDelta = 0.0;
	lastTimestamp = 0.0;
	lastTimestampDelta = 0.0;
	maxTimestampDelta = 0.0;
    timestampDeltaCount = 0;
	timestampDropCount = 0;
}

void cLogStatDataId::LogTimestamp(double timestamp)
{
    // check for corrupt data
    if (_ISNAN(timestamp) || timestamp < 0.0 || timestamp > 999999999999.0)
    {
        return;
    }
    else if (lastTimestamp > 0.0)
    {
        double delta = fabs(timestamp - lastTimestamp);
        maxTimestampDelta = _MAX(delta, maxTimestampDelta);
        totalTimeDelta += delta;
        averageTimeDelta = (totalTimeDelta / (double)++timestampDeltaCount);
        if (lastTimestampDelta != 0.0 && (fabs(delta - lastTimestampDelta) > (lastTimestampDelta * 0.5)))
        {
            timestampDropCount++;
        }
        lastTimestampDelta = delta;
	}
    lastTimestamp = timestamp;
    count++;
}

void cLogStatDataId::Printf()
{
	printf(" Count: %llu\r\n", (unsigned long long)count);
	printf(" Errors: %llu\r\n", (unsigned long long)errorCount);
	printf(" Time delta avg: %f\r\n", averageTimeDelta);
	printf(" Max time delta: %f\r\n", maxTimestampDelta);
	printf(" Time delta drop: %llu\r\n", (unsigned long long)timestampDropCount);
}

cLogStats::cLogStats()
{
	Clear();
}

void cLogStats::Clear()
{
	dataIdStats.clear();
	errorCount = 0;
	count = 0;
}

void cLogStats::LogError(const p_data_hdr_t* hdr)
{
	errorCount++;
	if (hdr != NULL)
	{
		cLogStatDataId& d = dataIdStats[hdr->id];
		d.errorCount++;
	}
}

void cLogStats::LogData(uint32_t dataId)
{
	cLogStatDataId& d = dataIdStats[dataId];
	d.count++;
	count++;
}

void cLogStats::LogDataAndTimestamp(uint32_t dataId, double timestamp)
{
	cLogStatDataId& d = dataIdStats[dataId];
	d.count++;
	count++;
	if (timestamp != 0.0)
	{
		d.LogTimestamp(timestamp);
	}
}

void cLogStats::Printf()
{
	printf("LOG STATS\r\n");
	printf("----------");
	printf("Count: %llu\r\n", (unsigned long long)count);
	printf("Errors: %llu\r\n", (unsigned long long)errorCount);
	for (map<uint32_t, cLogStatDataId>::iterator i = dataIdStats.begin(); i != dataIdStats.end(); i++)
	{
		printf(" DID: %d\r\n", i->first);
		i->second.Printf();
		printf("\r\n");
	}
}

void cLogStats::WriteToFile(const string &fileName)
{
    if (count != 0)
    {
        // flush log stats to disk
        ofstream stats;
        string newline = "\r\n";
        stats << fixed << setprecision(6);
        stats.open(fileName, ios::out | ios::binary);
        stats << "Count: " << count << newline;
        stats << "Errors: " << errorCount << newline << newline;
        for (map<uint32_t, cLogStatDataId>::iterator i = dataIdStats.begin(); i != dataIdStats.end(); i++)
        {
            stats << "Data Id: " << i->first << " - " << cISDataMappings::GetDataSetName(i->first) << newline;
            stats << "Count: " << i->second.count << newline;
            stats << "Errors: " << i->second.errorCount << newline;
            stats << "Average Timestamp Delta: " << i->second.averageTimeDelta << newline;
            stats << "Max Timestamp Delta: " << i->second.maxTimestampDelta << newline;
            stats << "Timestamp Drops: " << i->second.timestampDropCount << newline;
            stats << newline;
        }
        stats.close();
    }
}

#if defined(ENABLE_IS_PYTHON_WRAPPER)

void cISLogger::SetPyDisplay(cInertialSenseDisplay display)
{
	m_pyDisplay = display;
}

namespace py = pybind11;

py::dict cISLogger::PyReadData(unsigned int device)
{
	py::dict dOut;
	if (device >= m_devices.size())
	{
		return dOut;
	}

	p_data_t* data = NULL;
	while (LogDataIsCorrupt(data = m_devices[device]->ReadData()))
	{
		perror("Corrupt data read from log file");
		m_logStats->LogError(&data->hdr);
		data = NULL;
	}
	if (data != NULL)
	{
		double timestamp = cISDataMappings::GetTimestamp(&data->hdr, data->buf);
		m_logStats->LogDataAndTimestamp(data->hdr.id, timestamp);
		m_pyDisplay.ProcessData(data, true, 1.0);
		dOut = py_dataHandling(data);
	} 
	
	return dOut;
}

test_initializer logger([](py::module &m) {
	py::module m2 = m.def_submodule("logger");

	py::class_<cISLogger> log(m2, "cISLogger");
	log.def(py::init<>());
	log.def("LoadFromDirectory", &cISLogger::LoadFromDirectory);
	log.def("ReadData", &cISLogger::ReadData);
	log.def("PyReadData", &cISLogger::PyReadData);

	py::enum_<cISLogger::eLogType>(log, "eLogType")
		.value("LOGTYPE_DAT", cISLogger::eLogType::LOGTYPE_DAT)
		.value("LOGTYPE_SDAT",cISLogger::eLogType::LOGTYPE_SDAT)
		.value("LOGTYPE_CSV", cISLogger::eLogType::LOGTYPE_CSV)
		.value("LOGTYPE_KML", cISLogger::eLogType::LOGTYPE_KML)
		.export_values();

});

#endif
