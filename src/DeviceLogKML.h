/**
 * @file DeviceLogKML.h
 * @brief KML log format: write-only flight-path visualization, one KML file per logical channel
 *        (INS, reference INS, GNSS variants, RTK — see `cDataKML::MyEnum`) rather than per-DID.
 *
 * Unlike the other formats, KML reading is not implemented — ReadData() and its helpers are stubs
 * (`cISLogger::LoadFromDirectory()` already refuses `LOGTYPE_KML` for exactly this reason). Data
 * sets are buffered in memory per channel (`sKmlLog::data`) and only rendered to XML and written
 * out when a channel's buffer crosses `m_maxFileSize` (CloseWriteFile()) or the log is closed.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__DEVICE_LOG_KML_H
#define IS_SDK__DEVICE_LOG_KML_H

#include <stdio.h>
#include <string>
#include <vector>

#include "DataKML.h"
#include "DeviceLog.h"
#include "com_manager.h"

#if PLATFORM_IS_EVB_2
#include "ff.h"
#endif


/** @brief In-memory buffer of not-yet-written points/track for one KML channel (see `cDataKML::MyEnum`). */
struct sKmlLog
{
    std::vector<sKmlLogData> data;   //!< buffered points for this channel, cleared once written out by CloseWriteFile()

    std::string     fileName;        //!< name of the file this channel was (or will be) written to
    uint32_t        fileCount;       //!< number of files written so far for this channel
    uint32_t        fileDataSize;    //!< unused
    uint32_t        fileSize;        //!< estimated buffered size in bytes (`data.size() * cDataKML::BYTES_PER_KID()`); compared against `m_maxFileSize` to trigger a flush
};


/** @brief `cDeviceLog` implementation that renders buffered points to write-only KML files, one per logical channel. */
class cDeviceLogKML : public cDeviceLog
{
public:
    cDeviceLogKML() : cDeviceLog() {};
    cDeviceLogKML(device_handle_t dev) : cDeviceLog(dev) {};
    cDeviceLogKML(uint16_t hdwId, uint32_t serialNo) : cDeviceLog(hdwId, serialNo) {};

    /** @brief Clear every channel's buffered points/counters and reset the reference-INS detection flag. */
    void InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFileSize) OVERRIDE;

    /** @brief Flush and write out every channel's buffered points via CloseWriteFile(). */
    bool CloseAllFiles() OVERRIDE;

    /**
     * @brief Render @p log's buffered points to a KML document and write it to a new file, then clear the buffer.
     *
     * @p kid is remapped from `KID_INS` to `KID_REF` if this device was detected as a reference INS
     * (serial number 99999 or 10101; see SaveData()). GNSS-family channels are skipped entirely if
     * GNSS logging is disabled (SetKmlConfig()).
     *
     * @param kid logical channel to write; see `cDataKML::MyEnum`.
     * @param log the channel's buffered data.
     * @return true on success; false if @p log has no buffered data, `m_directory` is empty, the
     *         channel is a disabled GNSS variant, no serial number is resolvable, or the XML write failed.
     */
    bool CloseWriteFile(int kid, sKmlLog& log);

    /** @brief Open every channel's current KML file with the OS's default associated application (Windows only; a no-op elsewhere). */
    bool OpenWithSystemApp(void) OVERRIDE;

    /**
     * @brief Buffer one data set into its channel via WriteDateToFile(), and detect reference-INS devices.
     *
     * If @p dataHdr is `DID_DEV_INFO` and the reported serial number is 99999 or 10101, this
     * device's `KID_INS` channel is treated as the reference INS (`KID_REF`) for the remainder of
     * the log.
     */
    bool SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf, protocol_type_t ptype=_PTYPE_INERTIAL_SENSE_DATA) OVERRIDE;

    /**
     * @brief Not implemented for KML — always fails.
     * @return null; see the file-level documentation for why KML reading isn't supported.
     */
    p_data_buf_t* ReadData() OVERRIDE;

    void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
    std::string LogFileExtention() OVERRIDE { return std::string(".kml"); }

private:
    /** @brief Unused stub kept for interface symmetry with the other formats; always succeeds without doing anything. */
    bool OpenNewSaveFile(int kid, sKmlLog &log) { (void)kid; (void)log; return true; }

    /** @brief Not implemented — always returns null. See ReadData(). */
    p_data_buf_t* ReadDataFromChunk();

    /** @brief Not implemented — always returns true without reading anything. @note Because this never returns false, ReadData()'s retry loop around it would spin forever if ever called; KML reading is not a supported path (see ReadData()). */
    bool ReadChunkFromFile();

    /**
     * @brief Route one data set to its channel's buffer (via `cDataKML::DID_TO_KID()`), and flush if the channel's estimated size crosses `m_maxFileSize`.
     * @param dataHdr header identifying the data set; data sets that don't map to a known channel, or map to a disabled GNSS channel, are silently ignored.
     * @param dataBuf the data payload described by @p dataHdr.
     * @return true if the data was buffered (or ignored) successfully; false if a size-triggered flush via CloseWriteFile() failed.
     */
    bool WriteDateToFile(const p_data_hdr_t *dataHdr, const uint8_t *dataBuf);

    cDataKML                m_kml;                            //!< KML point-buffering helper shared across all channels
    sKmlLog                 m_Log[cDataKML::MAX_NUM_KID];      //!< per-channel buffered state, indexed by `cDataKML::MyEnum`
    bool                    m_isRefIns;                        //!< true once a companion device with a reference-INS serial number (99999 or 10101) has been detected; remaps this device's KID_INS output to KID_REF
};

#endif // IS_SDK__DEVICE_LOG_KML_H
