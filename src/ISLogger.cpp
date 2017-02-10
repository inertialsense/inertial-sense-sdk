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

#if !defined(_WIN32)

#include <sys/statvfs.h>

#endif

const string cISLogger::g_emptyString;


cISLogger::cISLogger()
{
	m_enabled = false;
}


cISLogger::~cISLogger()
{
	Cleanup();
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
}


bool cISLogger::InitSaveCommon(eLogType logType, const string& directory, const string& subDirectory, int numDevices, float maxDiskSpaceMB, uint32_t maxFileSize, uint32_t maxChunkSize, bool useSubFolderTimestamp)
{
	// Close any open files
	CloseAllFiles();

	m_logType = logType;
	m_directory = directory;
	m_maxDiskSpace = (uint64_t)(maxDiskSpaceMB * 1024.0f * 1024.0f);
	uint64_t availableSpace = cISLogger::GetDirectorySpaceAvailable(m_directory);
	m_maxDiskSpace = (m_maxDiskSpace == 0 || m_maxDiskSpace > availableSpace / MAX_PERCENT_OF_FREE_SPACE_TO_USE_FOR_IS_LOGS ? m_maxDiskSpace / MAX_PERCENT_OF_FREE_SPACE_TO_USE_FOR_IS_LOGS : m_maxDiskSpace);

	// ensure there are between 50 and 1000 files
	if (maxFileSize > m_maxDiskSpace / 50)
	{
		m_maxFileSize = (uint32_t)(m_maxDiskSpace / 50);
	}
	else if (maxFileSize < m_maxDiskSpace / 1000)
	{
		m_maxFileSize = (uint32_t)(m_maxDiskSpace / 1000);
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


bool cISLogger::InitSave(eLogType logType, const string& directory, int numDevices, float maxDiskSpaceMB, uint32_t maxFileSize, uint32_t maxChunkSize, bool useSubFolderTimestamp)
{
	m_timeStamp = CreateCurrentTimestamp();
	return InitSaveCommon(logType, directory, g_emptyString, numDevices, maxDiskSpaceMB, maxFileSize, maxChunkSize, useSubFolderTimestamp);
}


bool cISLogger::InitSaveTimestamp(const string& timeStamp, const string& directory, const string& subDirectory, int numDevices, eLogType logType, float maxDiskSpaceMB, uint32_t maxFileSize, uint32_t maxChunkSize, bool useSubFolderTimestamp)
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

	return InitSaveCommon(logType, directory, subDirectory, numDevices, maxDiskSpaceMB, maxFileSize, maxChunkSize, useSubFolderTimestamp);
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

	return true;
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

#if defined(_WIN32)

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
		if (file_name[0] == '.' || stat(full_file_name.c_str(), &st) == -1)
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


void cISLogger::DeleteDirectory(const string& directory)
{

#if defined(_WIN32)

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
			DeleteDirectory(full_file_name);
			continue;
		}
		remove(full_file_name.c_str());
	} while (FindNextFileA(dir, &file_data));
	FindClose(dir);

#else

	class dirent* ent;
	class stat st;
	DIR* dir = opendir(directory.c_str());
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
			DeleteDirectory(full_file_name.c_str());
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
	return GetDirectorySpaceUsed(directory, "", files, recursive);
}


uint64_t cISLogger::GetDirectorySpaceUsed(const string& directory, vector<file_info_t>& files, bool recursive)
{
	return GetDirectorySpaceUsed(directory, "", files, recursive);
}


uint64_t cISLogger::GetDirectorySpaceUsed(const string& directory, string regexPattern, vector<file_info_t>& files, bool recursive)
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
	} customSort;

	sort(files.begin(), files.end(), customSort);
	return spaceUsed;
}


uint64_t cISLogger::GetDirectorySpaceAvailable(const string& directory)
{

#if defined(_WIN32)

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
	realpath(directory.c_str(), fullPath);
	bool created = (_MKDIR(fullPath) == 0);
	statvfs(fullPath, &stat);
	if (created)
	{
		_RMDIR(fullPath);
	}
	return (uint64_t)stat.f_bsize * (uint64_t)stat.f_bavail;

#endif

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

	// get all files
	vector<string> fileNames;
	if (!GetAllFilesInDirectory(directory, false, fileExtensionRegex, fileNames))
	{
		return false;
	}

	for (size_t i = 0; i < fileNames.size(); i++)
	{
		string& name = fileNames[i];

		// remove sub directory info
		for (int32_t j = (int32_t)name.length() - 1; j >= 0; j--)
		{
			if (name[j] == '/' || name[j] == '\\')
			{
				name = name.substr(++j);
				break;
			}
		}
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


bool cISLogger::LogData(unsigned int device, p_data_hdr_t *dataHdr, uint8_t *dataBuf)
{
	if (!m_enabled || device >= m_devices.size())
	{
		return false;
	}

	// Save data
	if (!m_devices[device]->SaveData(dataHdr, dataBuf))
	{
		perror("Error logging data");
		return false;
	}
	return true;
}


p_data_t* cISLogger::ReadData( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return NULL;
	}

	return m_devices[device]->ReadData();
}


void cISLogger::CloseAllFiles()
{
	for( unsigned int i=0; i < m_devices.size(); i++ )
	{
		m_devices[i]->CloseAllFiles();
	}
}


void cISLogger::OpenWithSystemApp()
{
	for (unsigned int i = 0; i < m_devices.size(); i++)
	{
		m_devices[i]->OpenWithSystemApp();
	}
}


uint64_t cISLogger::LogSize( unsigned int device )
{
	if (device >= m_devices.size())
	{
		return 0;
	}

	return m_devices[device]->LogSize();
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

bool cISLogger::CopyLog(cISLogger& log, const string& timestamp, const string &outputDir, eLogType logType, float maxDiskSpaceMB, uint32_t maxFileSize, uint32_t maxChunkSize, bool useSubFolderTimestamp)
{
	if (!InitSaveTimestamp(timestamp, outputDir, g_emptyString, log.GetDeviceCount(), logType, maxDiskSpaceMB, maxFileSize, maxChunkSize, useSubFolderTimestamp))
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

		// Copy data
		while ((data = log.ReadData(dev)))
		{
			if (data->hdr.size == 0)
			{
				continue;
			}
			else if (!LogData(dev, &data->hdr, data->buf))
			{
				return false;
			}
		}

		// Close file
		m_devices[dev]->CloseAllFiles();
	}
	return true;
}