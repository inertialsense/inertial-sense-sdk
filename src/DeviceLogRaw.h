/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_RAW_H
#define DEVICE_LOG_RAW_H

#include <stdio.h>
#include <string>
#include <vector>

#include "DataChunk.h"
#include "DeviceLog.h"
#include "ISDevice.h"
#include "com_manager.h"
#include "ISLogStats.h"


class cDeviceLogRaw : public cDeviceLog
{
public:
    cDeviceLogRaw();
    cDeviceLogRaw(const ISDevice* dev);
    cDeviceLogRaw(uint16_t hdwId, uint32_t serialNo);


    void InitDeviceForWriting(std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFilesize) OVERRIDE;
	bool CloseAllFiles() OVERRIDE;
	bool FlushToFile() OVERRIDE;
	bool SaveData(int dataSize, const uint8_t* dataBuf, cLogStats &globalLogStats) OVERRIDE;
	p_data_buf_t* ReadData() OVERRIDE;
	void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
    std::string LogFileExtention() OVERRIDE { return std::string(".raw"); }
	void Flush() OVERRIDE;
	is_comm_instance_t* IsCommInstance() { return &m_comm; }

	cDataChunk m_chunk;

private:
	p_data_buf_t* ReadDataFromChunk();
	bool ReadChunkFromFile();
	bool WriteChunkToFile();

	uint8_t m_commBuf[PKT_BUF_SIZE];
	p_data_buf_t m_pData;
	is_comm_instance_t m_comm;
};

#endif // DEVICE_LOG_RAW_H
