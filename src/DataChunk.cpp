/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "DataChunk.h"

cDataChunk::cDataChunk(uint32_t maxSize, const char* name)
{
	Clear();
	m_maxSize = maxSize;
	SetName(name);
	m_hdr.marker = DATA_CHUNK_MARKER;
	m_hdr.version = 1;
	m_hdr.classification = ' ' << 8 | 'U';
	m_hdr.grpNum = 0; //!< Chunk group number
	m_hdr.devSerialNum = 0; //!< Serial number
	m_hdr.reserved = 0; //!< Reserved 
}


void cDataChunk::SetName(const char *name)
{
	if (name == NULL)
	{
		return;
	}

	strncpy(m_hdr.name, name, 4);
	m_hdr.invName[0] = ~m_hdr.name[0];
	m_hdr.invName[1] = ~m_hdr.name[1];
	m_hdr.invName[2] = ~m_hdr.name[2];
	m_hdr.invName[3] = ~m_hdr.name[3];
}


bool cDataChunk::PushBack(uint8_t* d1, uint32_t d1Size, uint8_t* d2, uint32_t d2Size)
{
	// Ensure data will fit
	uint32_t count = d1Size + d2Size;
	if (count > GetByteCountAvailableToWrite())
	{
		return false;
	}
	if (d1Size > 0)
	{
		m_data.insert(m_data.end(), d1, d1 + d1Size);
		m_hdr.dataSize += d1Size;
	}
	if (d2Size > 0)
	{
		m_data.insert(m_data.end(), d2, d2 + d2Size);
		m_hdr.dataSize += d2Size;
	}
	m_hdr.invDataSize = ~m_hdr.dataSize;
	m_bytesRemaining += d1Size + d2Size;
	return true;
}


uint8_t* cDataChunk::GetDataPtr()
{
	if (m_bytesRemaining == 0)
	{
		return NULL;
	}
	return &m_data[m_data.size() - m_bytesRemaining];
}


bool cDataChunk::PopFront(uint32_t size)
{
	if (size <= m_bytesRemaining)
	{
		m_bytesRemaining -= size;
		return true;
	}
	else
	{
		m_bytesRemaining = 0;
		m_data.clear();
		return false;
	}
}


void cDataChunk::Clear()
{
	m_hdr.dataSize = 0; //!< Byte size of data in this chunk
	m_hdr.invDataSize = ~0; //!< Bitwise inverse of m_Size
	m_bytesRemaining = 0;
	m_data.clear();

#if LOG_CHUNK_STATS
	memset(m_stats, 0, sizeof(m_stats));
#endif

}

// Returns number of bytes written, or -1 for error
int cDataChunk::WriteToFile(FILE* pFile, int groupNumber)
{
	if (pFile == NULL)
	{
		return -1;
	}

	m_hdr.grpNum = groupNumber;

	// Write chunk header to file
	int32_t nBytes = (int32_t)fwrite(&m_hdr, 1, sizeof(sChunkHeader), pFile);

	// Write any additional chunk header
	nBytes += WriteAdditionalChunkHeader(pFile);

	// Write chunk data to file
	nBytes += (int32_t)fwrite(&m_data[0], 1, m_hdr.dataSize, pFile);

#if LOG_DEBUG_WRITE
	static int totalBytes = 0;
	totalBytes += nBytes;
	printf("%d : %d  -  %x %d", totalBytes, nBytes, m_Hdr.marker, m_Hdr.dataSize);
	if (nBytes != (headerSize + (int)m_Hdr.dataSize))
		printf("ERROR WRITING!");
	printf("\n");
#endif

	// Error writing to file
	if (nBytes != GetHeaderSize() + (int)m_hdr.dataSize)
	{
		return -1;
	}

	// Clear chunk data
	Clear();

	return nBytes;
}


// Returns number of bytes read, or -1 for error
int cDataChunk::ReadFromFile(FILE* pFile)
{
	if (pFile == NULL)
	{
		return -1;
	}

	// Read chunk header
	int32_t nBytes = (int32_t)fread(&m_hdr, 1, sizeof(sChunkHeader), pFile);

	// Error checking
	if (m_hdr.dataSize != ~(m_hdr.invDataSize) || nBytes <= 0)
	{
		return -1;
	}

	// Read additional chunk header
	nBytes += ReadAdditionalChunkHeader(pFile);

	// Read chunk data
	m_maxSize = m_hdr.dataSize;
	m_data.resize(m_hdr.dataSize);
	nBytes += (int32_t)fread(&m_data[0], 1, m_hdr.dataSize, pFile);
	m_bytesRemaining = m_hdr.dataSize;

#if LOG_DEBUG_READ
	static int totalBytes = 0;
	totalBytes += nBytes;
	printf("%d : %d  -  %x %d  ", totalBytes, nBytes, m_Hdr.marker, m_Hdr.dataSize);
	if ((nBytes > 0) && (m_Hdr.marker != DATA_CHUNK_MARKER))
	{
		printf("MARKER MISMATCH!");
}
	printf("\n");
#endif

	if (m_hdr.marker == DATA_CHUNK_MARKER && nBytes == (int)(GetHeaderSize() + m_hdr.dataSize))
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
		m_bytesRemaining = 0;
		m_hdr.dataSize = 0;

		// Error reading from file
		return -1;
	}
}


int32_t cDataChunk::WriteAdditionalChunkHeader(FILE* pFile)
{
    (void)pFile;
	return 0;
}


int32_t cDataChunk::ReadAdditionalChunkHeader(FILE* pFile)
{
    (void)pFile;
	return 0;
}


int32_t cDataChunk::GetHeaderSize()
{
	return (int32_t)sizeof(sChunkHeader);
}


// LOG_CHUNK_STATS
void logStats( const char *format, ... )
{
	static FILE* pFile = NULL;
	va_list args;
	va_start( args, format );

#if 0
	vprintf( format, args );			// print to terminal
#endif
#if 1
	if( pFile == NULL )
		pFile = fopen( "STATS_.txt", "w" );
	vfprintf( pFile, format, args );	// print to file
#endif
	va_end( args );
}
