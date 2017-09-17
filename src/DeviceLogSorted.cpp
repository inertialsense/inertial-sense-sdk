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
    cDeviceLog::CloseAllFiles();

	// Write to file and clear any non-empty chunks to file
	for (uint32_t id = 0; id < m_chunks.size(); id++)
	{
        list<cSortedDataChunk>& chunkList = m_chunks[id];
        if (chunkList.size() != 0)
		{
            cSortedDataChunk& chunk = chunkList.front();

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
	}

	// Close existing file
	if (m_pFile)
	{
		fclose(m_pFile);
		m_pFile = NULL;
	}

	return true;
}


bool cDeviceLogSorted::SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf)
{
    cDeviceLog::SaveData(dataHdr, dataBuf);

	uint32_t id = dataHdr->id;

	if (id >= DID_MAX_COUNT)
	{
		return false;
	}
	else if (id >= m_chunks.size())
	{
		m_chunks.resize(id + 1);
	}

    cSortedDataChunk* currentChunk;
    list<cSortedDataChunk>& chunkList = m_chunks[id];

	// First time saving to this chunk
    if (chunkList.size() == 0)
	{
        cSortedDataChunk newChunk = { 0 };
        chunkList.push_back(newChunk);
        currentChunk = &chunkList.front();
        currentChunk->m_subHdr.dHdr = *dataHdr;
        currentChunk->m_hdr.pHandle = m_pHandle;
	}
	else
	{
		currentChunk = &chunkList.front();
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
			currentChunk->m_hdr.devSerialNum = m_devInfo.serialNumber;
		}
	}

    // depending on devices and sources, some logs may have different size for the same id
    // if the size changes, flush the chunk and start a new one with the new size
    uint32_t dataBytes;
    if (currentChunk->m_subHdr.dHdr.size != dataHdr->size || currentChunk->m_subHdr.dHdr.offset != dataHdr->offset)
    {
        // force a flush of the chunk and start a new chunk with the new size
        dataBytes = UINT_MAX;
    }
    else
    {
        dataBytes = sizeof(int32_t) + dataHdr->size;
    }
	if (dataBytes > currentChunk->GetByteCountAvailableToWrite())
	{
		// Create new file if needed
		if (m_pFile == NULL && !OpenNewSaveFile())
		{
			return false;
		}

		// Byte size
		int nBytes = currentChunk->GetDataSize();

        if (nBytes != 0)
        {
            // Write to file and clear chunk
            if ((nBytes = currentChunk->WriteToFile(m_pFile, 1)) <= 0)
            {
                return false;
            }
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

	// reset data header if it changed
	if (dataBytes == UINT_MAX)
	{
		currentChunk->m_subHdr.dHdr = *dataHdr;
	}

	// Add data header and data buffer to chunk
	if (!currentChunk->PushBack((unsigned char*)&(m_dataSerNum), sizeof(int32_t), (unsigned char*)dataBuf, dataHdr->size))
	{
		return false;
	}
	currentChunk->m_subHdr.dCount++;
	m_dataSerNum++;

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
		{
			// Found data
            cDeviceLog::OnReadData(data);
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
tryChunkListAgain:
        list<cSortedDataChunk>& chunksForId = m_chunks[id];
		if (chunksForId.size() == 0)
		{
			continue;
		}
        cSortedDataChunk& chunk = chunksForId.front();
		if (chunk.GetByteCountAvailableToRead() == 0)
		{
			chunksForId.pop_front();
            goto tryChunkListAgain;
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

	cSortedDataChunk& chunk = m_chunks[foundId].front();
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
	if (chunk.m_subHdr.dHdr.size <= MAX_DATASET_SIZE)
	{
		// Copy data header
		m_data.hdr = chunk.m_subHdr.dHdr;

		// Copy data buffer
		memcpy(m_data.buf, cnkData->buf, m_data.hdr.size);

		int pSize = chunk.m_subHdr.dHdr.size + sizeof(uint32_t);
		chunk.PopFront(pSize);
		return &m_data;
	}
    else
    {
        perror("Data is larger than max data set size");
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

        list<cSortedDataChunk>& chunkList = m_chunks[id];
		cSortedDataChunk* currentChunk;

        if (chunkList.size() == 0)
		{
			// Empty source chunk, copy Header
            cSortedDataChunk chunk = { 0 };
			chunk.m_subHdr = m_currentReadChunk.m_subHdr;

			// Find lowest serial number
			p_cnk_data_t* cdat = (p_cnk_data_t*)m_currentReadChunk.GetDataPtr();
			m_dataSerNum = _MIN(m_dataSerNum, cdat->dataSerNum);

            chunkList.push_back(chunk);
			currentChunk = &chunkList.front();
		}
        else
		{
			currentChunk = &chunkList.back();
			if (currentChunk->m_subHdr.dHdr.size != m_currentReadChunk.m_subHdr.dHdr.size ||
				currentChunk->m_subHdr.dHdr.offset != m_currentReadChunk.m_subHdr.dHdr.offset)
			{
				// chunk data size or offset change, append a new chunk
				cSortedDataChunk chunk = { 0 };
				chunk.m_subHdr = m_currentReadChunk.m_subHdr;
				chunkList.push_back(chunk);
				currentChunk = &chunkList.back();
			}
        }

		// add data count
		currentChunk->m_subHdr.dCount += m_currentReadChunk.m_subHdr.dCount;

		// Make sure chunk max size is large enough
		currentChunk->SetMaxSize(currentChunk->GetMaxSize() + m_currentReadChunk.GetDataSize());

		// Append data
		currentChunk->PushBack(m_currentReadChunk.GetDataPtr(), m_currentReadChunk.GetDataSize());
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
    list<cSortedDataChunk>& chunkList = m_chunks[DID_DEV_INFO];
	for (list<cSortedDataChunk>::iterator i = chunkList.begin(); i != chunkList.end(); i++)
	{
		i->m_hdr.devSerialNum = serialNumber;
	}
}







