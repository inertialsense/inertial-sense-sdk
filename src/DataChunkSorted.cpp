/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include "DataChunkSorted.h"


cSortedDataChunk::cSortedDataChunk(const char* name) : cDataChunk()
{
	memset(&m_subHdr, 0, sizeof(sChunkSubHeader));
	SetName(name);
}


void cSortedDataChunk::Clear()
{
	cDataChunk::Clear();
	m_subHdr.dCount = 0;
}


// This function reads the next sorted chunk matching the specified DID from all of the files.  The file pointers are advanced 
// if the current chunk dataSerNum is older than the current m_dataSerNum.  Files are closed if the last chuch dataSerNum 
// is older than the current m_dataSerNum.
// Returns number of bytes read, or -1 for error
int32_t cSortedDataChunk::ReadFromFiles(vector<cISLogFileBase*>& pFiles, uint32_t id, uint32_t dataSerNum)
{
	if (pFiles.size() == 0 || pFiles.front() == NULLPTR)
	{
		return -1;
	}






	// reset state, prepare to read from file
	Clear();

	// Read chunk header
	int32_t nBytes = static_cast<int32_t>(pFile->read(&m_hdr, sizeof(sChunkHeader)));

	// Error checking
	if (m_hdr.dataSize != ~(m_hdr.invDataSize) || nBytes <= 0)
	{
		return -1;
	}

	// Read additional chunk header
	nBytes += ReadAdditionalChunkHeader(pFile);

	//     // Error check data size
	//     if (m_hdr.dataSize > MAX_DATASET_SIZE)
	//     {
	//         return -1;
	//     }

		// Read chunk data
	m_dataTail += static_cast<int32_t>(pFile->read(m_buffHead, m_hdr.dataSize));
	nBytes += GetDataSize();

#if LOG_DEBUG_CHUNK_READ
	static int totalBytes = 0;
	totalBytes += nBytes;
	printf("cDataChunk::ReadFromFile %d : %d  -  %x %d  ", totalBytes, nBytes, m_hdr.marker, m_hdr.dataSize);
	if ((nBytes > 0) && (m_hdr.marker != DATA_CHUNK_MARKER))
	{
		printf("MARKER MISMATCH!");
	}
	printf("\n");
#endif

	if (m_hdr.marker == DATA_CHUNK_MARKER && nBytes == static_cast<int>(GetHeaderSize() + m_hdr.dataSize))
	{

#if LOG_CHUNK_STATS
		static bool start = true;
		int totalBytes = 0;
		for (int id = 0; id < DID_COUNT; id++)
			if (m_stats[id].total)
			{
				if (start)
				{
					start = false;
					logStats("------------------------------------------------\n");
					logStats("Chunk Data Summary\n");
					logStats("   ID:  Count:  Sizeof:  Total:\n");
				}
				logStats(" %4d%8d    %5d %7d\n", id, m_stats[id].count, m_stats[id].size, m_stats[id].total);
				totalBytes += m_stats[id].total;
			}
		start = true;
		if (totalBytes)
		{
			logStats("------------------------------------------------\n");
			logStats("                      %8d Total bytes\n", totalBytes);
		}
		logStats("\n");
		logStats("================================================\n");
		logStats("            *.dat Data Log File\n");
		logStats("================================================\n");
		m_Hdr.print();
#if LOG_CHUNK_STATS==2
		logStats("------------------------------------------------\n");
		logStats("Chunk Data\n");
#endif
		memset(m_stats, 0, sizeof(m_stats));
#endif

		return nBytes;
	}
	else
	{
		Clear();

		// Error reading from file
		return -1;
	}
}


int32_t cSortedDataChunk::WriteAdditionalChunkHeader(cISLogFileBase* pFile)
{
	// Write sub header to file
	return static_cast<int32_t>(pFile->write(&m_subHdr, sizeof(m_subHdr)));
}


int32_t cSortedDataChunk::ReadAdditionalChunkHeader(cISLogFileBase* pFile)
{
	// Read chunk header
	return static_cast<int32_t>(pFile->read(&m_subHdr, sizeof(m_subHdr)));
}


int32_t cSortedDataChunk::GetHeaderSize()
{
    return int32_t((sizeof(sChunkHeader) + sizeof(sChunkSubHeader)));
}
