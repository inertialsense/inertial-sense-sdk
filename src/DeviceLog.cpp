/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ctime>
#include <string>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#include "ISDevice.h"
#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISConstants.h"
#include "ISDataMappings.h"
#include "ISLogFileFactory.h"
#include "util/util.h"

using namespace std;

cDeviceLog::cDeviceLog() {
    m_logStats.Clear();
}

cDeviceLog::cDeviceLog(const ISDevice* dev) : device(dev)  {
    if (dev == nullptr)
        throw std::invalid_argument("cDeviceLog() must be passed a valid ISDevice instance.");
    m_devHdwId = ENCODE_DEV_INFO_TO_HDW_ID(dev->devInfo);
    m_devSerialNo = dev->devInfo.serialNumber;
    m_logStats.Clear();
}

cDeviceLog::cDeviceLog(uint16_t hdwId, uint32_t serial) : m_devHdwId(hdwId), m_devSerialNo(serial) {
    m_logStats.Clear();
}

cDeviceLog::~cDeviceLog()
{
    // Close open files
    CloseISLogFile(m_pFile);
    CloseAllFiles();
}

void cDeviceLog::InitDeviceForWriting(std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
	m_timeStamp = timestamp;
	m_directory = directory;
	m_fileCount = 0;
	m_maxDiskSpace = maxDiskSpace;
	m_maxFileSize = maxFileSize;
	m_logSize = 0;
	m_writeMode = true;
	m_logStats.Clear();
}


void cDeviceLog::InitDeviceForReading()
{
	m_fileSize = 0;
	m_logSize = 0;
	m_fileCount = 0;
	m_writeMode = false;
	m_logStats.Clear();
}

bool cDeviceLog::CloseAllFiles()
{
    if (m_writeMode) {
        string str = m_directory + "/stats_SN" + to_string(m_devSerialNo) + ".txt";
        m_logStats.WriteToFile(str);
    }
    return true;
}

bool cDeviceLog::OpenWithSystemApp()
{

#if PLATFORM_IS_WINDOWS

	std::wstring stemp = std::wstring(m_fileName.begin(), m_fileName.end());
	LPCWSTR filename = stemp.c_str();
	ShellExecuteW(0, 0, filename, 0, 0, SW_SHOW);

#endif

	return true;
}

bool cDeviceLog::SaveData(p_data_hdr_t *dataHdr, const uint8_t* dataBuf, protocol_type_t ptype)
{
	// Update log statistics
	if (dataHdr != NULL)
    {
		double timestamp = (ptype == _PTYPE_INERTIAL_SENSE_DATA ? cISDataMappings::TimestampOrCurrentTime(dataHdr, dataBuf) : current_timeSecD());
        m_logStats.LogData(ptype, dataHdr->id, timestamp);
	}

    return true;
}

bool cDeviceLog::SaveData(int dataSize, const uint8_t* dataBuf, cLogStats &globalLogStats)
{
	// Update log statistics done in cDeviceLogRaw::SaveData()
	return true;
}

bool cDeviceLog::SetupReadInfo(const string& directory, const string& serialNum, const string& timeStamp)
{
	m_directory = directory;
	m_fileCount = 0;
	m_timeStamp = timeStamp;
	m_fileNames.clear();
	vector<ISFileManager::file_info_t> fileInfos;
	SetSerialNumber((uint32_t)strtoul(serialNum.c_str(), NULL, 10));

	string regExp;
	if (serialNum == "0" && timeStamp == "")
	{	// Simple filename regular expression: [\/\\][0-9]+\.dat
		regExp = string("[\\/\\\\][0-9]+\\") + LogFileExtention();
	}
	else
	{	// Default filename regular expression: [\/\\]LOG_SN60339_.*\.dat
		regExp = string("[\\/\\\\]" IS_LOG_FILE_PREFIX) + serialNum + "_.*\\" + LogFileExtention();
	}
	// Search is case insensitive, finds both upper and lower case file extensions.

	ISFileManager::GetDirectorySpaceUsed(directory, regExp, fileInfos, false, false);

	if (fileInfos.size() != 0)
	{
		m_fileName = fileInfos[0].name;
		for (size_t i = 0; i < fileInfos.size(); i++)
		{
			m_fileNames.push_back(fileInfos[i].name);
		}
	}
	return true;
}


bool cDeviceLog::OpenNewSaveFile()
{
	// Close existing file
	CloseISLogFile(m_pFile);

	// Ensure directory exists
	if (m_directory.empty())
	{
		return false;
	}

	// create directory
	_MKDIR(m_directory.c_str());

	// Open new file
	m_fileCount++;
	uint32_t serNum = (device != nullptr ? device->devInfo.serialNumber : SerialNumber());
	if (!serNum)
		return false;

	string fileName = GetNewFileName(serNum, m_fileCount, NULL);
	m_pFile = CreateISLogFile(fileName, "wb");
	m_fileSize = 0;

	if (m_pFile && m_pFile->isOpened())
	{
#if LOG_DEBUG_FILE_WRITE
		printf("cDeviceLog::OpenNewSaveFile %s\n", fileName.c_str());
#endif
		return true;
	}
	else
	{
#if LOG_DEBUG_FILE_WRITE
		printf("cDeviceLog::OpenNewSaveFile FAILED %s\n", fileName.c_str());
#endif
		return false;
	}
}


bool cDeviceLog::OpenNextReadFile()
{
	// Close file if open
	CloseISLogFile(m_pFile);

	if (m_fileCount == m_fileNames.size())
	{
		return false;
	}
	
	m_fileName = m_fileNames[m_fileCount++];
	m_pFile = CreateISLogFile(m_fileName, "rb");

	if (m_pFile)
	{

#if LOG_DEBUG_FILE_READ
		printf("cDeviceLog::OpenNextReadFile %s\n", m_fileName.c_str());
#endif
		return true;
	}
	else
	{
#if LOG_DEBUG_FILE_READ
		printf("cDeviceLog::OpenNextReadFile FAILED %s\n", m_fileName.c_str());
#endif
		return false;
	}
}

string cDeviceLog::GetNewFileName(uint32_t serialNumber, uint32_t fileCount, const char* suffix)
{
    return utils::string_format("%s/%s%d_%s_%04d%s%s",
        m_directory.c_str(),
        IS_LOG_FILE_PREFIX, 
        (int)serialNumber, 
        m_timeStamp.c_str(), 
        (int)(fileCount % 10000), 
        (suffix == NULL || *suffix == 0 ? "" : (string("_") + suffix).c_str()), 
        LogFileExtention().c_str()
    );
}

void cDeviceLog::UpdateStatsFromFile(p_data_buf_t *data)
{ 
	double timestamp = cISDataMappings::Timestamp(&data->hdr, data->buf);
	m_logStats.LogData(_PTYPE_INERTIAL_SENSE_DATA, data->hdr.id, timestamp);  
}

void cDeviceLog::UpdateStatsFromFile(protocol_type_t ptype, int id, double timestamp)
{ 
	m_logStats.LogData(ptype, id, timestamp);  
}

ISDevice* cDeviceLog::Device() {
    return (ISDevice*)device;
}
const dev_info_t* cDeviceLog::DeviceInfo() {
    return (dev_info_t*)&(device->devInfo);
}

