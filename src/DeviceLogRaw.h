/**
 * @file DeviceLogRaw.h
 * @brief `LOGTYPE_RAW` log format (`.raw`): the undecoded, multi-protocol byte stream from the
 *        device, framed through a `cDataChunk` staging buffer with no per-chunk header.
 *
 * Unlike DeviceLogSerial (which stores already-parsed `p_data_hdr_t`/payload pairs), this format's
 * SaveData() takes raw bytes straight off the wire and feeds them byte-by-byte through an
 * `is_comm_instance_t` parser (`m_comm`) that recognizes multiple protocols (IS binary, NMEA,
 * RTCM3, u-blox) — so a `.raw` log can capture correction streams and third-party protocols
 * alongside IS data, not just `p_data_hdr_t`-shaped records. Its `cDataChunk` writes/reads with
 * `writeHeader = false` (see WriteChunkToFile()/ReadChunkFromFile()), so the on-disk file is a
 * flat, unframed byte stream rather than a sequence of `sChunkHeader`-prefixed chunks.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__DEVICE_LOG_RAW_H
#define IS_SDK__DEVICE_LOG_RAW_H

#include <stdio.h>
#include <string>
#include <vector>

#include "DataChunk.h"
#include "DeviceLog.h"
#include "ISDevice.h"
#include "com_manager.h"
#include "ISLogStats.h"


/** @brief `cDeviceLog` implementation for the raw, multi-protocol byte-stream format; see the file-level documentation above. */
class cDeviceLogRaw : public cDeviceLog
{
public:
    cDeviceLogRaw();
    cDeviceLogRaw(device_handle_t dev);
    cDeviceLogRaw(uint16_t hdwId, uint32_t serialNo);

    /** @brief Clear the staging chunk and record the device's serial number/port info in its header. */
    void InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFilesize) OVERRIDE;

    /** @brief Clear the staging chunk. */
    void InitDeviceForReading() OVERRIDE;

    bool CloseAllFiles() OVERRIDE;
    bool FlushToFile() OVERRIDE;

    /**
     * @brief Parse @p dataBuf byte-by-byte for statistics/DID_DEV_INFO capture, then buffer it (unparsed) into the current chunk.
     *
     * Every recognized packet is indexed via `addIndexRecord()` as it's parsed. When the chunk
     * fills, it's flushed to disk via WriteChunkToFile() first.
     *
     * @param dataSize number of bytes in @p dataBuf.
     * @param dataBuf raw bytes as received from the device; may span partial or multiple packets/protocols.
     * @param globalLogStats the owning `cISLogger`'s aggregate statistics, updated alongside this device's own.
     * @return true if the bytes were buffered (and any triggered flush succeeded); false if a flush failed or the chunk couldn't hold the data.
     */
    bool SaveData(int dataSize, const uint8_t* dataBuf, cLogStats &globalLogStats) OVERRIDE;

    /** @brief Read the next `_PTYPE_INERTIAL_SENSE_DATA`/`_PTYPE_INERTIAL_SENSE_CMD` packet, skipping over packets of other recognized protocols. @return the parsed data set, or null if the log is exhausted. */
    p_data_buf_t* ReadData() OVERRIDE;

    /** @brief Read the next packet of any recognized protocol. @param[out] ptype protocol type of the returned packet. @return the next packet, or null if the log is exhausted. */
    packet_t* ReadPacket(protocol_type_t& ptype) OVERRIDE;

    void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
    std::string LogFileExtention() OVERRIDE { return std::string(".raw"); }
    void Flush() OVERRIDE;

    /** @brief Get the IS-comm parser instance used to recognize packets in this log's byte stream. */
    is_comm_instance_t* IsCommInstance() OVERRIDE { return &m_comm; }

    cDataChunk m_chunk;     //!< staging buffer for not-yet-flushed raw bytes; written/read with no chunk header (see the file-level documentation)

private:
    /** @brief Initialize `m_comm` and enable the recognized protocols (IS binary data, NMEA, RTCM3, u-blox). */
    void initCommInstance();

    /**
     * @brief Parse and consume the next recognized packet from `m_chunk`, one byte at a time.
     * @param[out] ptype protocol type of the returned packet; `_PTYPE_NONE` if the chunk was exhausted with no complete packet found.
     * @return the parsed packet, or null if no complete packet was found in the currently buffered data.
     */
    packet_t* ReadPacketFromChunk(protocol_type_t& ptype);

    /** @brief Read the next unframed chunk of raw bytes from the file into `m_chunk`, opening the next file if the current one is exhausted. @return true if a chunk was read; false if no more data/files are available. */
    bool ReadChunkFromFile();

    /** @brief Write `m_chunk`'s buffered bytes to the current file with no chunk header (opening a new file first if needed), then clear the chunk. @return true on success; false if there's nothing to write or the file couldn't be opened/written. */
    bool WriteChunkToFile();

    uint8_t m_commBuf[PKT_BUF_SIZE];       //!< working buffer owned by `m_comm` for in-progress packet assembly
    p_data_buf_t m_pData;                   //!< scratch buffer for the most recently read data set
    is_comm_instance_t m_comm;              //!< multi-protocol packet parser (IS binary, NMEA, RTCM3, u-blox)
    protocol_type_t m_protocolType;         //!< unused
};

#endif // IS_SDK__DEVICE_LOG_RAW_H
