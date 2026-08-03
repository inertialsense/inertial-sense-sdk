/**
 * @file DeviceLogSerial.h
 * @brief `LOGTYPE_DAT` log format (`.dat`): already-parsed data sets (`p_data_hdr_t` + payload
 *        pairs), framed through a `cDataChunk` staging buffer with a full chunk header.
 *
 * Unlike DeviceLogRaw (which stores the undecoded, multi-protocol byte stream), this format's
 * SaveData() takes an already-parsed data set — it pushes the `p_data_hdr_t` header and payload
 * as-is into the staging chunk, with no IS-comm protocol parsing. Its `cDataChunk` writes/reads
 * with the default `writeHeader = true` (see WriteChunkToFile()/ReadChunkFromFile()), so the
 * on-disk file is a sequence of `sChunkHeader`-prefixed chunks, each containing a run of
 * `p_data_hdr_t`/payload pairs.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__DEVICE_LOG_SERIAL_H
#define IS_SDK__DEVICE_LOG_SERIAL_H

#include <stdio.h>
#include <string>
#include <vector>

#include "DataChunk.h"
#include "DeviceLog.h"
#include "com_manager.h"


/** @brief `cDeviceLog` implementation for the already-parsed, chunk-header-framed `.dat` format; see the file-level documentation above. */
class cDeviceLogSerial : public cDeviceLog
{
public:
    cDeviceLogSerial();
    cDeviceLogSerial(device_handle_t dev);
    cDeviceLogSerial(uint16_t hdwId, uint32_t serialNo);

    /** @note Declared but not defined anywhere in the SDK; calling it will fail to link. */
    cDeviceLogSerial(port_handle_t port);

    /** @brief Clear the staging chunk and record the device's serial number/port info in its header. */
    void InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFilesize) OVERRIDE;

    /** @brief Clear the staging chunk. */
    void InitDeviceForReading() OVERRIDE;

    bool CloseAllFiles() OVERRIDE;
    bool FlushToFile() OVERRIDE;

    /**
     * @brief Buffer one already-parsed data set (header + payload) into the current chunk, rotating files/chunks as needed.
     * @param dataHdr header describing @p dataBuf's data ID, size, and offset.
     * @param dataBuf the data payload described by @p dataHdr.
     * @param ptype protocol type the data arrived as; forwarded to the base class for statistics.
     * @return true if the data set was buffered (and any triggered chunk flush succeeded); false if a flush failed or the chunk couldn't hold the data.
     */
    bool SaveData(p_data_hdr_t *dataHdr, const uint8_t *dataBuf, protocol_type_t ptype = _PTYPE_INERTIAL_SENSE_DATA) OVERRIDE;

    /** @brief Read the next data set, reading the next chunk/file as needed. @return the next data set, or null if the log is exhausted. */
    p_data_buf_t *ReadData() OVERRIDE;

    void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
    std::string LogFileExtention() OVERRIDE { return std::string(".dat"); }
    void Flush() OVERRIDE;

    cDataChunk m_chunk;     //!< staging buffer for not-yet-flushed header/payload pairs; written/read with a full chunk header (see the file-level documentation)

private:
    /**
     * @brief Pop the next header/payload pair off the front of `m_chunk`.
     * @return a pointer into the chunk's buffer, valid until the next chunk mutation; null if the chunk is empty.
     */
    p_data_buf_t *ReadDataFromChunk();

    /** @brief Read the next chunk-header-framed chunk from the file into `m_chunk`, opening the next file if the current one is exhausted. @return true if a chunk was read; false if no more data/files are available. */
    bool ReadChunkFromFile();

    /** @brief Write `m_chunk`'s buffered data to the current file with a full chunk header (opening a new file first if needed), then clear the chunk. @return true on success; false if there's nothing to write or the file couldn't be opened/written. */
    bool WriteChunkToFile();
};

#endif // IS_SDK__DEVICE_LOG_SERIAL_H
