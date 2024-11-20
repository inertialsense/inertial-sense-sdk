/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DATA_CHUNK_H
#define DATA_CHUNK_H

#include "ISConstants.h"

#if PLATFORM_IS_EVB_2
#define DEFAULT_CHUNK_DATA_SIZE     16384           // 16 KB (EVB)
#else
#define DEFAULT_CHUNK_DATA_SIZE     (128 * 1024)    // 128 KB
#endif

#define DATA_CHUNK_MARKER           0xFC05EA32

#include <stdint.h>

#include "com_manager.h"
#include "ISLogFileBase.h"

void logStats(const char *format, ...);

// #define CHUNK_VER_1

//!< Chunk Header
PUSH_PACK_1

struct sChunkHeader
{
    union {
        struct {
            uint32_t marker;                             //!< Chunk marker (0xFC05EA32)
            uint8_t version;                             //!< Chunk Version
            uint8_t dataOffset;                          //!< offset from this position until the start of the chunk data (34 = sum of all hdr bytes following this byte, upto the chnk data, or sizeof(sChunkHeader) - 6).
            char protocolVersion[2];                     //!< major/minor version of the underlying line/protocol
            char name[4];                                //!< Chunk name
            char invName[4];                             //!< Bitwise inverse of chunk name
            uint32_t dataSize;                           //!< Chunk data length in bytes
            uint32_t invDataSize;                        //!< Bitwise inverse of chunk data length
            uint32_t grpNum;                             //!< Chunk Group Number: 0 = serial data, 1 = sorted data...
            uint32_t devSerialNum;                       //!< Device serial number
            uint16_t portId;                             //!< Device port id
            uint16_t portType;                           //!< Device port type
            char fwVersion[4];                           //!< Device firmware version
        };
        struct {
            uint32_t marker;                             //!< Chunk marker (0xFC05EA32)
            uint16_t version;                            //!< Chunk Version
            uint16_t classification;                     //!< Chunk classification
            char name[4];                                //!< Chunk name
            char invName[4];                             //!< Bitwise inverse of chunk name
            uint32_t dataSize;                           //!< Chunk data length in bytes
            uint32_t invDataSize;                        //!< Bitwise inverse of chunk data length
            uint32_t grpNum;                             //!< Chunk Group Number: 0 = serial data, 1 = sorted data...
            uint32_t devSerialNum;                       //!< Device serial number
            uint32_t pHandle;                            //!< Device port handle
            uint32_t reserved;                           //!< Unused
        } v1;
    };
    sChunkHeader() {
        marker = DATA_CHUNK_MARKER;
        version = 2;
        dataOffset = 34;
        protocolVersion[0] = PROTOCOL_VERSION_CHAR0;
        protocolVersion[1] = PROTOCOL_VERSION_CHAR1;
        dataSize = 0;
        invDataSize = 0xFFFFFFFF;
        grpNum = 0;
        devSerialNum = 0;
        portId = 0xFFFF;
        portType = PORT_TYPE__UNKNOWN;
        fwVersion[0] = 0;
        fwVersion[1] = 0;
        fwVersion[2] = 0;
        fwVersion[3] = 0;
    }
#if LOG_CHUNK_STATS
    void print()
    {
    #ifdef CHUNK_VER_1
        logStats("Chunk Header\n");
        logStats("         marker:  %u (0x%x)\n", marker, marker);
        logStats("        version:  %d\n", version);
        logStats(" classification:  %d\n", classification);
        logStats("           name:  %c%c%c%c\n", name[0], name[1], name[2], name[3]);
        logStats("        invName:  %c%c%c%c\n", invName[0], invName[1], invName[2], invName[3]);
        logStats("       dataSize:  %d\n", dataSize);
        logStats("    invDataSize:  %d\n", invDataSize);
        logStats("         grpNum:  %d\n", grpNum);
        logStats("   devSerialNum:  %d\n", devSerialNum);
        logStats("        port:  %d\n", port);
        logStats("       reserved:  %d\n", reserved);
    #else
        logStats("Chunk Header\n");
        logStats("         marker:  %u (0x%x)\n", marker, marker);
        logStats("        version:  %d\n", version);
        logStats("     dataOffset:  %d\n", dataOffset);
        logStats("protocolVersion:  %c.%c", protocolVersion[0], protocolVersion[1]);
        logStats("           name:  %c%c%c%c\n", name[0], name[1], name[2], name[3]);
        logStats("        invName:  %c%c%c%c\n", invName[0], invName[1], invName[2], invName[3]);
        logStats("       dataSize:  %d\n", dataSize);
        logStats("    invDataSize:  %d\n", invDataSize);
        logStats("         grpNum:  %d\n", grpNum);
        logStats("   devSerialNum:  %d\n", devSerialNum);
        #ifndef port_handle_t
        logStats("        pHandle:  %d\n", pHandle);
        #else
        logStats("         portId:  %d\n", portId);
        logStats("       portType:  %04X\n", portType);
        #endif
        logStats("firmwareVersion:  %c.%c.%c.%c\n", fwVersion[0], fwVersion[1], fwVersion[2], fwVersion[3]);
    #endif
    }
#endif
};

POP_PACK

class cDataChunk {
public:
    cDataChunk();

    virtual ~cDataChunk();

    int32_t GetBuffSize() { return (int32_t) (m_buffTail - m_buffHead); }

    int32_t GetBuffFree() { return (int32_t) (m_buffTail - m_dataTail); }

    int32_t GetDataSize() { return (int32_t) (m_dataTail - m_dataHead); }

    void SetName(const char name[4]);

    void SetDevInfo(const dev_info_t& devInfo);

    uint8_t *GetDataPtr();

    bool PopFront(int32_t size);

    int32_t WriteToFile(cISLogFileBase *pFile, int groupNumber = 0, bool writeHeader = true); // Returns number of bytes written to file and clears the chunk

    int32_t ReadFromFile(cISLogFileBase *pFile, bool readHeader = true);

    int32_t PushBack(uint8_t *d1, int32_t d1Size, uint8_t *d2 = NULL, int32_t d2Size = 0);

    virtual void Clear();

    sChunkHeader m_hdr = { };

#if LOG_CHUNK_STATS
    struct
    {
        uint32_t count;     // Number of occurrences
        uint32_t size;      // Size of each data structure
        uint32_t total;     // Total bytes read
    } m_stats[DID_COUNT];
#endif

protected:
    virtual int32_t WriteAdditionalChunkHeader(cISLogFileBase *pFile);

    virtual int32_t ReadAdditionalChunkHeader(cISLogFileBase *pFile);

    virtual int32_t GetHeaderSize();

private:
    uint8_t m_buffHead[DEFAULT_CHUNK_DATA_SIZE];    // Start of buffer
    uint8_t *m_buffTail;    // End of buffer
    uint8_t *m_dataHead;    // Front of data in buffer.  This moves as data is read.
    uint8_t *m_dataTail;    // End of data in buffer.  This moves as data is written.
};


#endif // DATA_CHUNK_H
