/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

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
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <limits.h>

#include "DeviceLogSorted.h"
#include "ISLogger.h"


void cDeviceLogSorted::InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize, uint32_t chunkSize)
{
	m_chunks.resize(0);
	m_dataSerNum = 0;
	m_lastSerNum = 0xFFFFFFFF;
	cDeviceLog::InitDeviceForWriting(pHandle, timestamp, directory, maxDiskSpace, maxFileSize, chunkSize);
}


void cDeviceLogSorted::InitDeviceForReading()
{
	m_chunks.resize(0);
	m_dataSerNum = 0;
	m_lastSerNum = 0xFFFFFFFF;
	cDeviceLog::InitDeviceForReading();
}


bool cDeviceLogSorted::CloseAllFiles()
{
	// Write to file and clear any non-empty chunks to file
	for (uint32_t id = 0; id < m_chunks.size(); id++)
	{
		cSortedDataChunk& chunk = m_chunks[id];
		if (chunk.GetDataSize() != 0)
		{
			// Create new file if needed
			if (m_pFile == NULL && !OpenNewSaveFile())
			{
				return false;
			}

			// Write to file and clear chunk
			if (chunk.WriteToFile(m_pFile, 1) <= 0)
			{
				return false;
			}
		}
	}

	// Close existing file
	if (m_pFile)
	{
		fclose(m_pFile);
		m_pFile = NULL;
	}

	return true;
}


bool cDeviceLogSorted::SaveData(p_data_hdr_t* dataHdr, uint8_t* dataBuf)
{
	uint32_t id = dataHdr->id;

	if (id >= 256)
	{
		return false;
	}

	if (id >= m_chunks.size())
	{
		m_chunks.resize(id + 1);
	}

	cSortedDataChunk& chunk = m_chunks[id];

	// First time saving to this chunk
	if (chunk.m_subHdr.dHdr.id == 0)
	{
		chunk.m_subHdr.dHdr = *dataHdr;
		chunk.m_hdr.pHandle = m_pHandle;
	}

	// Add serial number if available
	if (id == DID_DEV_INFO && !copyDataPToStructP2(&m_devInfo, dataHdr, dataBuf, sizeof(dev_info_t)))
	{
		int start = dataHdr->offset;
		int end = dataHdr->offset + dataHdr->size;
		int snOffset = offsetof(dev_info_t, serialNumber);

		// Did we get the serial number portion of devInfo?
		if (start <= snOffset && (int)(snOffset + sizeof(uint32_t)) <= end)
		{
			chunk.m_hdr.devSerialNum = m_devInfo.serialNumber;
		}
	}

	// Ensure new data size matches size defined in chunk
	if (chunk.m_subHdr.dHdr.size != dataHdr->size)
	{
		return false;
	}

	// Ensure data will fit in chunk, if not, create new chunk.
	uint32_t dataBytes = sizeof(int32_t) + dataHdr->size;
	if (dataBytes > chunk.GetByteCountAvailableToWrite())
	{
		// Create new file if needed
		if (m_pFile == NULL && !OpenNewSaveFile())
		{
			return false;
		}

		// Byte size
		int nBytes = chunk.GetDataSize();

		// Write to file and clear chunk
		if ((nBytes = chunk.WriteToFile(m_pFile, 1)) <= 0)
		{
			return false;
		}

		m_fileSize += nBytes;
		m_logSize += nBytes;

		// File is large enough to be closed
		if (m_fileSize >= m_maxFileSize)
		{
			// Write and clear remaining chunks to file and close file
			if (!CloseAllFiles())
			{
				return false;
			}
		}
	}

	// Add data header and data buffer to chunk
	if (!chunk.PushBack((unsigned char*)&(m_dataSerNum), sizeof(int32_t), (unsigned char*)dataBuf, dataHdr->size))
	{
		return false;
	}
	chunk.m_subHdr.dCount++;
	m_dataSerNum++;

	return true;
}


// Re-serialize data to original order
// Consider using a queue where data from chunks are added on one end and the serial order # is checked on the other end.
// This function needs to scan across all chunk queues and find the next lowest data to pop off.

// Read serialized data
p_data_t* cDeviceLogSorted::ReadData()
{
	p_data_t *data;

	while (1)
	{
		data = SerializeDataFromChunks();

		if (data)
		{
			// Found data
			return data;
		}

		// No more data
		m_chunks.clear();
		while (!ReadAllChunksFromFile())
		{
			if (!OpenNextReadFile())
			{
				// No more files or data
				return NULL;
			}
		}
	}
}


p_data_t* cDeviceLogSorted::SerializeDataFromChunks()
{

tryAgain:

	uint32_t foundId = UINT_MAX;
	uint32_t minSerialNum = UINT_MAX;

	// while there is data in any chunk, find the chunk with the next id
	for (uint32_t id = 0; id < m_chunks.size(); id++)
	{
		cSortedDataChunk& chunk = m_chunks[id];
		if (chunk.GetByteCountAvailableToRead() == 0)
		{
			continue;
		}
		p_cnk_data_t* cnkData = (p_cnk_data_t*)chunk.GetDataPtr();
		if (cnkData->dataSerNum < minSerialNum)
		{
			foundId = id;
			minSerialNum = cnkData->dataSerNum;
		}
	}

	if (foundId == UINT_MAX)
	{
		return NULL;
	}

	cSortedDataChunk& chunk = m_chunks[foundId];
	p_cnk_data_t* cnkData = (p_cnk_data_t*)chunk.GetDataPtr();

#if 0	// Error check for gap data serial number.  Verify we didn't drop any data.

	if (m_lastSerNum == UINT_MAX)
	{
		m_lastSerNum = m_dataSerNum;
	}
	if (m_dataSerNum - m_lastSerNum > 1)
	{
		cout << endl << "Gap in data: " << foundId << "\t" << m_dataSerNum;
	}
	m_lastSerNum = m_dataSerNum;

#endif

	// Increment data serial number to one larger than current
	m_dataSerNum = cnkData->dataSerNum + 1;

	// Data fits in temp buf
	if (chunk.m_subHdr.dHdr.size < MAX_P_DATA_BODY_SIZE)
	{
		// Copy data header
		m_data.hdr = chunk.m_subHdr.dHdr;

		// Copy data buffer
		memcpy(m_data.buf, cnkData->buf, m_data.hdr.size);

		int pSize = chunk.m_subHdr.dHdr.size + sizeof(uint32_t);
		chunk.PopFront(pSize);
		return &m_data;
	}

	goto tryAgain;
}


bool cDeviceLogSorted::ReadAllChunksFromFile()
{
	unsigned int id;

	// Reset data serial number and file data size
	m_fileSize = 0;

	// Read chunk and append it to existing chunks of same type
	while (ReadChunkFromFile())
	{
		id = m_currentReadChunk.m_subHdr.dHdr.id;

		// Expand chunk vector if necessary
		if (id >= m_chunks.size())
		{
			m_chunks.resize(id + 1);
		}

		cSortedDataChunk& chunk = m_chunks[id];

		if (chunk.GetDataSize() == 0)
		{
			// Empty source chunk, copy Header
			chunk.m_subHdr = m_currentReadChunk.m_subHdr;

			// Find lowest serial number
			p_cnk_data_t* cdat = (p_cnk_data_t*)m_currentReadChunk.GetDataPtr();
			m_dataSerNum = _MIN(m_dataSerNum, cdat->dataSerNum);
		}
		else
		{	
			// Appending read chunk to chunk w/ existing data
			// Ensure new data matches ID, size, offset
			if (chunk.m_subHdr.dHdr.id != m_currentReadChunk.m_subHdr.dHdr.id ||
				chunk.m_subHdr.dHdr.size != m_currentReadChunk.m_subHdr.dHdr.size ||
				chunk.m_subHdr.dHdr.offset != m_currentReadChunk.m_subHdr.dHdr.offset)
			{
				cout << endl << "MISMATCHED SORTED CHUNKS!";
				return false;
			}
			chunk.m_subHdr.dCount += m_currentReadChunk.m_subHdr.dCount;
		}

		// Make sure chunk max size is large enough
		chunk.SetMaxSize(chunk.GetMaxSize() + m_currentReadChunk.GetDataSize());

		// Append data
		chunk.PushBack(m_currentReadChunk.GetDataPtr(), m_currentReadChunk.GetDataSize());
	}

	return m_fileSize != 0;
}


bool cDeviceLogSorted::ReadChunkFromFile()
{
	if (m_pFile == NULL)
	{
		return false;
	}

	// Read chunk from file
	int nBytes = m_currentReadChunk.ReadFromFile(m_pFile);

	// Validate chunk: non-zero size and is sorted type
	int hdrSize = sizeof(p_data_hdr_t);
	if (nBytes < hdrSize || m_currentReadChunk.m_hdr.grpNum != 1)
	{
		return false;
	}

	// Update stats
	m_fileSize += nBytes;
	m_logSize += nBytes;

	return true;
}


void cDeviceLogSorted::SetSerialNumber(uint32_t serialNumber)
{
	m_devInfo.serialNumber = serialNumber;
	if (DID_DEV_INFO > m_chunks.size())
	{
		m_chunks.resize(DID_DEV_INFO + 1);
	}
	m_chunks[DID_DEV_INFO].m_hdr.devSerialNum = serialNumber;
}







