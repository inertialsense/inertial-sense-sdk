/**
 * @file DataChunk.h
 * @brief Chunk framing layer: `sChunkHeader` on-disk header and `cDataChunk` in-memory staging
 *        buffer used by the raw/serial device log formats (DeviceLogRaw, DeviceLogSerial).
 *
 * A "chunk" is a marker-delimited, header-prefixed block of data written as a unit to a
 * `.raw`/`.dat` log file. `cDataChunk` owns a single fixed-size, non-wrapping buffer: bytes are
 * appended at the tail (PushBack()) and consumed from the head (PopFront()/WriteToFile()); it does
 * not recycle space until explicitly Clear()'d, so a chunk is filled once, flushed, and reset.
 * `sChunkHeader` mirrors the current on-disk layout (`version` 2) alongside the legacy v1 layout
 * (`sChunkHeader::v1`) kept only so old `.dat` files remain readable.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
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

/**
 * @brief Debug-only sink for chunk read/write statistics (see LOG_CHUNK_STATS above).
 *
 * Writes formatted text either to the terminal (EVB-2, via `vprintf`) or to a lazily-created
 * `STATS_.txt` log file (host platforms, via a static `cISLogFileBase` opened on first call).
 * @param format printf-style format string, followed by its matching arguments.
 */
void logStats(const char *format, ...);

// #define CHUNK_VER_1

PUSH_PACK_1

/**
 * @brief On-disk chunk header written immediately before a chunk's data payload.
 *
 * The anonymous struct is the current (v2, `version == 2`) layout; `v1` is the legacy layout kept
 * only so `.dat` files written by older SDKs can still be parsed by ReadFromFile(). Both layouts
 * share the same leading `marker` field so a reader can identify which layout follows by first
 * inspecting `marker`/`version` before committing to one struct or the other.
 */
struct sChunkHeader
{
    union {
        struct {
            uint32_t marker;                             //!< chunk marker, always DATA_CHUNK_MARKER (0xFC05EA32)
            uint8_t version;                             //!< chunk header version; 2 for this layout
            uint8_t dataOffset;                          //!< byte offset from this field to the start of chunk data (34 = sizeof(sChunkHeader) - 6)
            char protocolVersion[2];                      //!< major/minor version of the underlying comm protocol
            char name[4];                                //!< 4-character chunk type name (e.g. "PDAT")
            char invName[4];                              //!< bitwise inverse of name, for framing validation on read
            uint32_t dataSize;                           //!< chunk data length in bytes, following this header
            uint32_t invDataSize;                        //!< bitwise inverse of dataSize, for framing validation on read
            uint32_t grpNum;                             //!< chunk group number: 0 = serial data, 1 = sorted data, ...
            uint32_t devSerialNum;                       //!< serial number of the device that produced this chunk
            uint16_t portId;                              //!< port id the data was received on
            uint16_t portType;                            //!< port type (see PORT_TYPE__* constants) the data was received on
            char fwVersion[4];                            //!< firmware version of the source device (defaults to the SDK version until overwritten by devInfo)
        };
        /** @brief Legacy v1 chunk header layout, kept for reading old `.dat` files. */
        struct {
            uint32_t marker;                             //!< chunk marker, always DATA_CHUNK_MARKER (0xFC05EA32)
            uint16_t version;                            //!< chunk header version; 1 for this layout
            uint16_t classification;                     //!< chunk classification
            char name[4];                                //!< 4-character chunk type name
            char invName[4];                              //!< bitwise inverse of name, for framing validation on read
            uint32_t dataSize;                           //!< chunk data length in bytes, following this header
            uint32_t invDataSize;                        //!< bitwise inverse of dataSize, for framing validation on read
            uint32_t grpNum;                             //!< chunk group number: 0 = serial data, 1 = sorted data, ...
            uint32_t devSerialNum;                       //!< serial number of the device that produced this chunk
            uint32_t pHandle;                             //!< legacy device port handle
            uint32_t reserved;                            //!< unused
        } v1;
    };

    /** @brief Initialize a v2 header with the marker, version, and current SDK firmware version. */
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
    /** @brief Debug-only: dump this header's fields via logStats(). Compiled in only when LOG_CHUNK_STATS is non-zero. */
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

/**
 * @brief Fixed-size, non-wrapping staging buffer for one chunk's worth of raw/serial log data.
 *
 * Data is appended at the tail via PushBack() and consumed from the head via PopFront() or
 * WriteToFile(); the buffer does not recycle freed head space until Clear() rewinds both pointers
 * back to the start, so callers must drain (or discard) a chunk before it fills. The backing
 * storage is a fixed `DEFAULT_CHUNK_DATA_SIZE`-byte array owned by this instance, not the heap.
 */
class cDataChunk {
public:
    cDataChunk();

    virtual ~cDataChunk();

    /** @brief Total capacity of the backing buffer in bytes (constant for the lifetime of the instance). */
    int32_t GetBuffSize() { return (int32_t) (m_buffTail - m_buffHead); }

    /** @brief Unused capacity remaining at the tail of the buffer, available to PushBack(). */
    int32_t GetBuffFree() { return (int32_t) (m_buffTail - m_dataTail); }

    /** @brief Number of bytes of unconsumed data currently buffered (tail minus head). */
    int32_t GetDataSize() { return (int32_t) (m_dataTail - m_dataHead); }

    /**
     * @brief Set the chunk's 4-character type name and its bitwise-inverse validation field.
     * @param name 4-character chunk type name (e.g. "PDAT"); ignored (no-op) if null.
     */
    void SetName(const char name[4]);

    /** @brief Copy the source device's serial number and firmware version into the chunk header. */
    void SetDevInfo(const dev_info_t& devInfo);

    /**
     * @brief Get a pointer to the start of the currently buffered, unconsumed data.
     * @return pointer to the data, or null if GetDataSize() is 0.
     */
    uint8_t *GetDataPtr();

    /**
     * @brief Discard @p size bytes from the front of the buffered data.
     *
     * If @p size exceeds GetDataSize(), the entire chunk is Clear()'d (not just partially
     * consumed) and false is returned — callers can't rely on partial consumption on failure.
     *
     * @param size number of bytes to discard from the front of the buffer.
     * @return true if exactly @p size bytes were discarded; false if @p size exceeded the
     *         buffered data size, in which case the chunk was cleared entirely instead.
     */
    bool PopFront(int32_t size);

    /**
     * @brief Write this chunk's header (optional) and buffered data to @p pFile, then Clear() it.
     * @param pFile destination file; must not be null.
     * @param groupNumber value written into the header's `grpNum` field before writing.
     * @param writeHeader if true, write the chunk header (and any subclass-provided additional
     *        header via WriteAdditionalChunkHeader()) before the data.
     * @return the number of bytes written, or -1 if @p pFile is null or the byte count written
     *         didn't match the expected header + data size.
     */
    int32_t WriteToFile(cISLogFileBase *pFile, int groupNumber = 0, bool writeHeader = true);

    /**
     * @brief Clear() this chunk, then read a header (optional) and its data from @p pFile.
     * @param pFile source file; must not be null.
     * @param readHeader if true, read and validate a chunk header (and any subclass-provided
     *        additional header via ReadAdditionalChunkHeader()) before reading data; if false,
     *        treat all bytes read as raw, unframed data with no header validation.
     * @return the number of bytes read (header + data) on success; -1 if @p pFile is null, the
     *         header's `dataSize`/`invDataSize` checksum didn't match, the marker didn't match
     *         DATA_CHUNK_MARKER, or fewer bytes were read than the header promised.
     */
    int32_t ReadFromFile(cISLogFileBase *pFile, bool readHeader = true);

    /**
     * @brief Append up to two buffers to the tail of the chunk's data.
     * @param d1 first buffer to append; must not be null.
     * @param d1Size number of bytes to append from @p d1.
     * @param d2 optional second buffer to append immediately after @p d1; may be null.
     * @param d2Size number of bytes to append from @p d2 (ignored if @p d2 is null).
     * @return the total number of bytes appended (`d1Size + d2Size`), or 0 if the combined size
     *         exceeds GetBuffFree() and nothing was appended.
     */
    int32_t PushBack(uint8_t *d1, int32_t d1Size, uint8_t *d2 = NULL, int32_t d2Size = 0);

    /** @brief Reset the chunk to empty: zeroes the header's data-size fields and rewinds the head/tail pointers to the start of the buffer. Does not clear the buffer's contents, only its bookkeeping. */
    virtual void Clear();

    sChunkHeader m_hdr = { };   //!< current chunk header; rewritten by SetName()/SetDevInfo() and on every WriteToFile()/ReadFromFile()

#if LOG_CHUNK_STATS
    /** @brief Debug-only per-DID read counters, indexed by data ID. Compiled in only when LOG_CHUNK_STATS is non-zero. */
    struct
    {
        uint32_t count;     //!< number of occurrences read for this DID
        uint32_t size;      //!< size in bytes of each occurrence's data structure
        uint32_t total;     //!< total bytes read across all occurrences of this DID
    } m_stats[DID_COUNT];
#endif

protected:
    /**
     * @brief Extension point for subclasses to write additional header data after the base chunk header.
     * @param pFile destination file, already positioned after the base sChunkHeader.
     * @return number of additional bytes written. Base implementation writes nothing and returns 0.
     */
    virtual int32_t WriteAdditionalChunkHeader(cISLogFileBase *pFile);

    /**
     * @brief Extension point for subclasses to read additional header data after the base chunk header.
     * @param pFile source file, already positioned after the base sChunkHeader.
     * @return number of additional bytes read. Base implementation reads nothing and returns 0.
     */
    virtual int32_t ReadAdditionalChunkHeader(cISLogFileBase *pFile);

    /**
     * @brief Total on-disk header size, including any subclass additional header.
     * @return `sizeof(sChunkHeader)` in the base implementation; subclasses that override
     *         WriteAdditionalChunkHeader()/ReadAdditionalChunkHeader() should override this too.
     */
    virtual int32_t GetHeaderSize();

private:
    uint8_t m_buffHead[DEFAULT_CHUNK_DATA_SIZE];    //!< start of the fixed backing buffer
    uint8_t *m_buffTail;    //!< end of the backing buffer (m_buffHead + DEFAULT_CHUNK_DATA_SIZE)
    uint8_t *m_dataHead;    //!< front of unconsumed data in the buffer; advances as data is popped
    uint8_t *m_dataTail;    //!< end of buffered data; advances as data is pushed
};


#endif // DATA_CHUNK_H
