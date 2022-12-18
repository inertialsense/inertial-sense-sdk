/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

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

#include "ISLogger.h"
#include "ISLogFileFactory.h"
#include "ISDataMappings.h"
#include "DeviceLogSorted.h"

#define LOG_DEBUG_PRINT_CHUNK_SAVE		0
#define LOG_DEBUG_PRINT_CHUNK_READ		0
#define LOG_DEBUG_PRINT_DID_SAVE		0
#define LOG_DEBUG_PRINT_DID_READ		0


cDeviceLogSorted::cDeviceLogSorted()
{
    for (uint32_t i = 0; i < DID_COUNT; i++)
    {
        m_chunks[i] = NULLPTR;
    }
}


void cDeviceLogSorted::InitDeviceForWriting(int pHandle, std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
	for (uint32_t i = 0; i < DID_COUNT; i++)
    {
        if (m_chunks[i])
        {
            delete m_chunks[i];
            m_chunks[i] = NULLPTR;
        }
    }
	m_dataSerNum = 0;
	m_lastSerNum = 0xFFFFFFFF;
	cDeviceLog::InitDeviceForWriting(pHandle, timestamp, directory, maxDiskSpace, maxFileSize);
}


void cDeviceLogSorted::InitDeviceForReading()
{
	// Clear chunk array
	for (uint32_t id = 0; id < DID_COUNT; id++)
	{
		if (m_chunks[id] != NULLPTR)
		{
			delete m_chunks[id];
			m_chunks[id] = NULLPTR;
		}
	}
    m_dataSerNum = 0;
	m_lastSerNum = 0xFFFFFFFF;
	cDeviceLog::InitDeviceForReading();

	// Open all files
	OpenAllReadFiles();
}


bool cDeviceLogSorted::OpenAllReadFiles()
{
	// Close files if open
	for (unsigned int i = 0; i < m_pFiles.size(); i++)
	{	
		CloseISLogFile(m_pFiles[i]);
	}
	m_pFiles.clear();

	for (m_fileCount = 0; m_fileCount < m_fileNames.size(); m_fileCount++)
	{
		m_fileName = m_fileNames[m_fileCount];
		m_pFiles.push_back(CreateISLogFile(m_fileName, "rb"));

		if (m_pFiles.back())
		{	// Success
#if LOG_DEBUG_FILE_READ
			printf("cDeviceLog::OpenNextReadFile %s\n", m_fileName.c_str());
#endif
		}
		else
		{	// Failure
#if LOG_DEBUG_FILE_READ
			printf("cDeviceLog::OpenNextReadFile FAILED %s\n", m_fileName.c_str());
#endif
			return false;
		}
	}

	for (uint32_t id = 0; id < DID_COUNT; id++)
	{
		m_chunksAvailable[id] = true;
	}

	return true;
}


bool cDeviceLogSorted::CloseAllFiles()
{
	// Write remaining data to file
	FlushToFile();

	// Close file pointer used for writing
	CloseISLogFile(m_pFile);

	// Close file pointer array used for reading
	for (unsigned int i = 0; i < m_pFiles.size(); i++)
	{
		CloseISLogFile(m_pFiles[i]);
	}
	m_pFiles.clear();

	cDeviceLog::CloseAllFiles();

	return true;
}


bool cDeviceLogSorted::FlushToFile()
{
	if (m_writeMode)
	{	// Write to file and clear any non-empty chunks to file
		for (uint32_t id = 1; id < DID_COUNT; id++)
		{
			// Write to file and clear chunk
			WriteChunkToFile(id);
		}

		return true;
	}

	return false;
}


bool cDeviceLogSorted::SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf)
{
    cDeviceLog::SaveData(dataHdr, dataBuf);

	uint32_t id = dataHdr->id;

	if (id >= DID_COUNT)
	{
		return false;
	}

    if (m_chunks[id] == NULLPTR)
    {	
        m_chunks[id] = new cSortedDataChunk();
        // First time saving to this chunk
        m_chunks[id]->m_subHdr.dHdr = *dataHdr;
        m_chunks[id]->m_hdr.pHandle = m_pHandle;
        m_chunks[id]->m_hdr.devSerialNum = m_devInfo.serialNumber;
    }
    cSortedDataChunk* chunk = m_chunks[id];

	// Add serial number if available
	if (id == DID_DEV_INFO)
	{
		if (!copyDataPToStructP2(&m_devInfo, dataHdr, dataBuf, sizeof(dev_info_t)))
		{
			int start = dataHdr->offset;
			int end = dataHdr->offset + dataHdr->size;
			int snOffset = offsetof(dev_info_t, serialNumber);

			// Did we get the serial number portion of devInfo?
			if (start <= snOffset && (int)(snOffset + sizeof(uint32_t)) <= end)
			{
				chunk->m_hdr.devSerialNum = m_devInfo.serialNumber;
			}
		}
	}

	int32_t dataBytes = sizeof(int32_t) + dataHdr->size;

	// depending on devices and sources, some logs may have different size for the same id
    // if the size changes, flush the chunk and start a new one with the new size
    if ((chunk->m_subHdr.dHdr.size != dataHdr->size) ||			// DID chunk size or offset changed
		(chunk->m_subHdr.dHdr.offset != dataHdr->offset) ||
		(dataBytes > chunk->GetBuffFree()))						// chunk is too full
	{
        // write chunk to file and clear chunk.
		if (!WriteChunkToFile(id))
		{
			return false;
		}
		chunk = m_chunks[id];

		// Reset data header in case it changed
		chunk->m_subHdr.dHdr = *dataHdr;
	}

	// Add data header and data buffer to chunk
	int nBytes = chunk->PushBack((unsigned char*)&(m_dataSerNum), sizeof(uint32_t), (unsigned char*)dataBuf, dataHdr->size);
	if (nBytes <= 0)
	{
		return false;
	}

#if LOG_DEBUG_PRINT_DID_SAVE
	double timestamp = cISDataMappings::GetTimestamp(dataHdr, dataBuf);
	printf("sorted did save: %d  DID: %2d  size: %3d  time: %.4lf\n", m_dataSerNum, dataHdr->id, nBytes-(int)sizeof(uint32_t), timestamp);
#endif

	// Prep for next save
	chunk->m_subHdr.dCount++;
	m_dataSerNum++;

	return true;
}


bool cDeviceLogSorted::WriteChunkToFile(uint32_t id)
{
	if (m_chunks[id] == NULLPTR)
	{
		return false;
	}

	// Byte size
	int nBytes = m_chunks[id]->GetDataSize();
	if (nBytes <= 0)
	{	// No data
		return true;
	}

	// Create new file if needed
	if (m_pFile == NULLPTR && !OpenNewSaveFile())
	{
		return false;
	}

#if LOG_DEBUG_PRINT_CHUNK_SAVE
	printf("sorted chunk save:   DID:%3d  dCount: %5d  nBytes: %6d   dataSN: %2d\n", m_chunks[id]->m_subHdr.dHdr.id, m_chunks[id]->m_subHdr.dCount, nBytes, m_chunks[id]->GetDataSerNum());
#endif

	if (nBytes != 0)
	{
		// Write chunk to file and clear chunk
		if ((nBytes = m_chunks[id]->WriteToFile(m_pFile, 1)) <= 0)
		{
			return false;
		}
	}

	m_fileSize += nBytes;
	m_logSize += nBytes;

	// Remove data
	m_chunks[id]->Clear();

	// File is large enough to be closed
	if (m_fileSize >= m_maxFileSize)
	{
		CloseISLogFile(m_pFile);
	}

	return true;
}


// Re-serialize data to original order
// Consider using a queue where data from chunks are added on one end and the serial order # is checked on the other end.
// This function needs to scan across all chunk queues and find the next lowest data to pop off.

// Read serialized data
p_data_t* cDeviceLogSorted::ReadData()
{
	p_data_t* data;

	while (1)
	{
		data = SerializeDataFromChunks();

		if (data)
		{	// Found data
            cDeviceLog::OnReadData(data);       // Record statistics

#if LOG_DEBUG_PRINT_DID_READ
			double timestamp = cISDataMappings::GetTimestamp(&(data->hdr), data->buf);
			printf("sorted did read: %d  DID: %2d  size: %3d  time: %.4lf\n", m_dataSerNum-1, data->hdr.id, data->hdr.size, timestamp);
#endif
		}

		return data;
	}
}


p_data_t* cDeviceLogSorted::SerializeDataFromChunks()
{

tryAgain:

	uint32_t foundId = UINT_MAX;
	uint32_t minSerialNum = UINT_MAX;
    cSortedDataChunk *chunk;

	// while there is data in any chunk, find the chunk with the next id
	for (uint32_t id = 1; id < DID_COUNT; id++)
	{
		if (!m_chunksAvailable[id])
		{
			continue;
		}

        chunk = m_chunks[id];
        if (chunk == NULLPTR || chunk->GetDataSize() == 0)
		{
			// Chunk is empty.  Search all files for new chuck.
			if (ReadNextChunkFromFiles(id))
			{
				chunk = m_chunks[id];
			}
			if (chunk == NULLPTR || chunk->GetDataSize() == 0)
			{
				m_chunksAvailable[id] = false;
				continue;
			}
		}

		uint32_t dataSerNum = chunk->GetDataSerNum();
        if (dataSerNum == UINT_MAX)
        {
            continue;
        }
		if (dataSerNum == m_dataSerNum)
		{	// Found exact match.
			foundId = id;
			break;
		}
		if (dataSerNum < minSerialNum)
		{	// Search for lowest serial number
			foundId = id;
			minSerialNum = dataSerNum;
		}
	}

	if (foundId == UINT_MAX)
	{   // No more data left
		return NULL;
	}

    chunk = m_chunks[foundId];
	p_cnk_data_t* cnkData = (p_cnk_data_t*)(chunk->GetDataPtr());

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
	m_dataSerNum = chunk->GetDataSerNum() + 1;
	
	if (chunk->m_subHdr.dHdr.size <= MAX_DATASET_SIZE)
	{   
		// Data fits in temp buf

        // Copy data header
		m_data.hdr = chunk->m_subHdr.dHdr;

		// Copy data buffer, ensure not to overrun chunk memory in case of corrupt data
		memcpy(m_data.buf, cnkData->buf, _MIN(chunk->GetDataSize(), (int32_t)(m_data.hdr.size)));

        // Size = serial number plus data size
		int pSize = chunk->m_subHdr.dHdr.size + sizeof(uint32_t);

		chunk->PopFront(pSize);
		return &m_data;
	}
    else
    {
        perror("Data is larger than max data set size");
    }

	goto tryAgain;
}


// This function reads the next sorted chunk matching the specified DID from all of the files.  The file pointers are advanced 
// if the current chunk dataSerNum is older than the current m_dataSerNum.  Files are closed if the last chuch dataSerNum 
// is older than the current m_dataSerNum.
bool cDeviceLogSorted::ReadNextChunkFromFiles(uint32_t id)
{
	// Error check ID
	if (id >= DID_COUNT || id == DID_NULL)
	{
		return false;
	}

	// Reset data serial number and file data size
	m_fileSize = 0;

	// Read chunk and append it to existing chunks of same type
	if (!ReadChunkFromFiles(&m_readChunk, id))
	{
		return false;
	}

	id = m_readChunk.m_subHdr.dHdr.id;

	if (m_chunks[id] == NULLPTR)
	{
		m_chunks[id] = new cSortedDataChunk();
	}

	cSortedDataChunk* chunk = m_chunks[id];

	if (chunk->GetDataSize() == 0)
	{
		// Reset buffer
		chunk->Clear();

		// Copy chunk sub header
		chunk->m_subHdr = m_readChunk.m_subHdr;

		// Find lowest serial number (used for sorting data)
		m_dataSerNum = _MIN(m_dataSerNum, m_readChunk.GetDataSerNum());
	}

	// add data count
	chunk->m_subHdr.dCount += m_readChunk.m_subHdr.dCount;

	// Resize chunk if necessary
//         if (m_readChunk.GetDataSize() > chunk->GetBuffFree())
//         {
//             chunk->Resize(chunk->GetDataSize() + m_readChunk.GetDataSize());
//         }

	// Append data
	chunk->PushBack(m_readChunk.GetDataPtr(), m_readChunk.GetDataSize());

	return m_fileSize != 0;
}


// This function reads the next sorted chunk matching the specified DID from all of the files.  The file pointers are advanced 
// if the current chunk dataSerNum is older than the current m_dataSerNum.  Files are closed if the last chuch dataSerNum 
// is older than the current m_dataSerNum.
bool cDeviceLogSorted::ReadChunkFromFiles(cSortedDataChunk *chunk, uint32_t id)
{
	if (m_pFiles.size() == 0 || m_pFiles.front() == NULLPTR || chunk == NULLPTR)
	{
		return false;
	}

	// Read chunk from file
	int nBytes = chunk->ReadFromFiles(m_pFiles, id, m_dataSerNum);
	if (nBytes < 0)
	{
		return false;
	}

#if LOG_DEBUG_PRINT_CHUNK_READ
	printf("sorted chunk read:   DID:%3d  dCount: %5d  nBytes: %6d   dataSN: %2d\n", chunk->m_subHdr.dHdr.id, chunk->m_subHdr.dCount, (int)(nBytes-sizeof(sChunkHeader)-sizeof(sChunkSubHeader)), chunk->GetDataSerNum());
#endif

	// Validate chunk: non-zero size and is sorted type
	int hdrSize = sizeof(p_data_hdr_t);
	if (nBytes < hdrSize || chunk->m_hdr.grpNum != 1)
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
    if (m_chunks[DID_DEV_INFO] == NULLPTR)
	{
        m_chunks[DID_DEV_INFO] = new cSortedDataChunk();
		m_chunks[DID_DEV_INFO]->m_subHdr.dHdr.id = DID_DEV_INFO;
		m_chunks[DID_DEV_INFO]->m_hdr.pHandle = m_pHandle;
	}
    m_chunks[DID_DEV_INFO]->m_hdr.devSerialNum = serialNumber;
}







