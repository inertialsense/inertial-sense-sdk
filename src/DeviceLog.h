/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_H
#define DEVICE_LOG_H

#include <stdio.h>
#include <string.h>
#include <vector>

extern "C"
{
	#include "com_manager.h"
}

using namespace std;

// never change these!
#define IS_LOG_FILE_PREFIX "LOG_SN"
#define IS_LOG_FILE_PREFIX_LENGTH 6
#define IS_LOG_TIMESTAMP_LENGTH 15

class cDeviceLog
{
public:
	cDeviceLog()
	{
		m_pFile = NULL;
		m_pHandle = 0;
		m_fileSize = 0;
		m_logSize = 0;
		m_fileCount = 0;
		memset(&m_devInfo, 0, sizeof(dev_info_t));
	}

	virtual ~cDeviceLog()
	{	
		// Close open files
		if (m_pFile)
		{
			fclose(m_pFile);
			m_pFile = NULL;
		}
		CloseAllFiles();
	}

	virtual void InitDeviceForWriting(int pHandle, string timestamp, string directory, uint64_t maxDiskSpace, uint32_t maxFileSize, uint32_t chunkSize);
	virtual void InitDeviceForReading();
	virtual bool CloseAllFiles() { return false; }
	virtual bool OpenWithSystemApp();
	virtual bool SaveData(p_data_hdr_t *dataHdr, uint8_t *dataBuf) = 0;
	bool SetupReadInfo(const string& directory, const string& deviceName, const string& timeStamp);
	virtual p_data_t* ReadData() = 0;
	void SetDeviceInfo(const dev_info_t *info);
	const dev_info_t* GetDeviceInfo() { return &m_devInfo; }
	virtual void SetSerialNumber(uint32_t serialNumber) = 0;
	virtual string LogFileExtention() = 0;
	uint64_t FileSize() { return m_fileSize; }
	uint64_t LogSize() { return m_logSize; }
	uint32_t FileCount() { return m_fileCount; }

protected:
	bool OpenNewSaveFile();
	bool OpenNextReadFile();
	string GetNewFileName(uint32_t serialNumber, uint32_t fileCount, const char* suffix);

	vector<string>			m_fileNames;
	FILE*					m_pFile;
	string					m_directory;
	string					m_timeStamp;
	string					m_fileName;
	dev_info_t				m_devInfo;
	int						m_pHandle;
	uint64_t				m_fileSize;
	uint64_t				m_logSize;
	uint32_t				m_fileCount;
	uint64_t				m_maxDiskSpace;
	uint32_t				m_maxFileSize;
	uint32_t				m_maxChunkSize;
};

#endif // DEVICE_LOG_H
