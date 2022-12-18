/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_SORTED_H
#define DEVICE_LOG_SORTED_H

#include <stdio.h>
#include <string>
#include <vector>
#include <list>

#include "DeviceLog.h"
#include "DataChunkSorted.h"


class cDeviceLogSorted : public cDeviceLog
{
public:
    cDeviceLogSorted();

	void InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize) OVERRIDE;
	void InitDeviceForReading() OVERRIDE;
	bool OpenAllReadFiles();
	bool CloseAllFiles() OVERRIDE;
	bool FlushToFile() OVERRIDE;
    bool SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf) OVERRIDE;
	p_data_t* ReadData() OVERRIDE;
	void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
	std::string LogFileExtention() OVERRIDE { return std::string(".sdat"); }

    cSortedDataChunk *m_chunks[DID_COUNT];
	bool m_chunksAvailable[DID_COUNT];

	p_data_t* SerializeDataFromChunks();
	bool ReadNextChunkFromFiles(uint32_t id);
	bool ReadChunkFromFiles(cSortedDataChunk *chunk, uint32_t id);
	bool WriteChunkToFile(uint32_t id);

	uint32_t m_dataSerNum;
	uint32_t m_lastSerNum;
	p_data_t m_data;
	cSortedDataChunk m_readChunk;

	std::vector<cISLogFileBase*> m_pFiles;

};

#endif // DEVICE_LOG_SORTED_H
