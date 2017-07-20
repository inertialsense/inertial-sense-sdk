/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DATA_CHUNK_H
#define DATA_CHUNK_H

#define MAX_CHUNK_SIZE 100000
#define DATA_CHUNK_MARKER 0xFC05EA32

#include <vector>
#include <stdint.h>

#include "com_manager.h"

#define LOG_DEBUG_WRITE		0		// Enable debug printout
#define LOG_DEBUG_READ		0
#define LOG_CHUNK_STATS		0		// 0 = disabled, 1 = summary, 2 = detailed

using namespace std;

void logStats(const char *format, ...);

//!< Chunk Header
PUSH_PACK_1

struct sChunkHeader 
{
	uint32_t	marker;				//!< Chunk marker (0xFC05EA32)
	uint16_t	version;			//!< Chunk Version
	uint16_t	classification;		//!< Chunk classification
	char		name[4];			//!< Chunk name
	char		invName[4];			//!< Bitwise inverse of chunk name
	uint32_t	dataSize;			//!< Chunk data length in bytes
	uint32_t	invDataSize;		//!< Bitwise inverse of chunk data length
	uint32_t	grpNum;				//!< Chunk Group Number: 0 = serial data, 1 = sorted data...
	uint32_t	devSerialNum;		//!< Device serial number
	uint32_t	pHandle;			//!< Device port handle
	uint32_t	reserved;			//!< Unused

#if LOG_CHUNK_STATS
	void print()
	{
		logStats( "Chunk Header\n" );
		logStats( "         marker:  %u (0x%x)\n", marker, marker );
		logStats( "        version:  %d\n", version );
		logStats( " classification:  %d\n", classification );
		logStats( "           name:  %c%c%c%c\n", name[0], name[1], name[2], name[3] );
		logStats( "        invName:  %c%c%c%c\n", invName[0], invName[1], invName[2], invName[3] );
		logStats( "       dataSize:  %d\n", dataSize );
		logStats( "    invDataSize:  %d\n", invDataSize );
		logStats( "         grpNum:  %d\n", grpNum );
		logStats( "   devSerialNum:  %d\n", devSerialNum );
		logStats( "        pHandle:  %d\n", pHandle );
		logStats( "       reserved:  %d\n", reserved );
	}
#endif
};

POP_PACK

class cDataChunk
{
public:
	cDataChunk(uint32_t maxSize = MAX_CHUNK_SIZE, const char* name = "PDAT");
	uint32_t GetMaxSize() { return m_maxSize; }
	void SetMaxSize(uint32_t maxSize) { m_maxSize = maxSize; }
	void SetName(const char *name);
	uint8_t* GetDataPtr();
	bool PopFront(uint32_t size);
	uint32_t GetByteCountAvailableToRead() { return m_bytesRemaining; }
	uint32_t GetByteCountAvailableToWrite() { return m_maxSize - (uint32_t)m_data.size(); }
	uint32_t GetDataSize() { return (uint32_t)m_data.size(); }
    int32_t WriteToFile(FILE* pFile, int groupNumber = 0); // Returns number of bytes written to file and clears the chunk
	int32_t ReadFromFile(FILE* pFile);
	bool PushBack(uint8_t* d1, uint32_t d1Size, uint8_t* d2 = NULL, uint32_t d2Size = 0);

	virtual void Clear();

	sChunkHeader m_hdr;

#if LOG_CHUNK_STATS
	struct
	{
		uint32_t count;		// Number of occurrences
		uint32_t size;		// Size of each data structure
		uint32_t total;		// Total bytes read
	} m_stats[DID_COUNT];
#endif

protected:
	virtual int32_t WriteAdditionalChunkHeader(FILE* pFile);
	virtual int32_t ReadAdditionalChunkHeader(FILE* pFile);
	virtual int32_t GetHeaderSize();

private:
	std::vector<uint8_t> m_data;
	uint32_t m_maxSize;
	uint32_t m_bytesRemaining;
};


#endif // DATA_CHUNK_H
