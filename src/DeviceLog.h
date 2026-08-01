/**
 * @file DeviceLog.h
 * @brief Per-device log base class: file rotation, `.idx` sidecar indexing, and the abstract
 *        SaveData()/ReadData() contract each on-disk format implements.
 *
 * `cDeviceLog` owns exactly one device's log: it decides when to roll over to a new file
 * (OpenNewSaveFile()/OpenNextReadFile()), names files consistently (GetNewFileName()), and
 * maintains the per-segment `.idx` sidecar (addIndexRecord()/writeIndexChunk()/finalizeIndex(),
 * see ISLogIndex.h for the sidecar format itself). It does not know how to serialize a data set to
 * bytes — SaveData()/ReadData() are format-specific and implemented by the DeviceLog{CSV,JSON,KML,
 * Raw,Serial} subclasses.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef DEVICE_LOG_H
#define DEVICE_LOG_H

#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include "ISDevice.h"
#include "ISLogFileBase.h"
#include "ISLogIndex.h"
#include "ISLogStats.h"

#include "com_manager.h"

// never change these!
#define IS_LOG_FILE_PREFIX "LOG_SN"
#define IS_LOG_TIMESTAMP_LENGTH 15

#if !defined(ISDevice)
    class ISDevice;
    typedef std::shared_ptr<ISDevice> device_handle_t;
#endif


/**
 * @brief Abstract per-device log; see the file-level documentation above for the overview.
 *
 * Subclasses (DeviceLog{CSV,JSON,KML,Raw,Serial}) implement ReadData(), SetSerialNumber(), and
 * LogFileExtention() at minimum, and typically override InitDeviceForWriting()/CloseAllFiles() to
 * layer their own per-format file state on top of this class's rotation/indexing bookkeeping.
 */
class cDeviceLog {
public:
    /**
     * @brief Legacy v1 `.idx` record layout (host-uptime ms + record counter, 16 bytes).
     *
     * Kept for back-compat readers per D-01 (SN-7879). Writers emit v2 exclusively (see
     * `index_record_t`); this layout exists only so legacy-aware reader code can deserialize old
     * `.idx` files produced by SDK <= 2.x.
     */
    typedef struct index_record_v1_s {
        uint32_t time;      //!< host-uptime milliseconds at the time this record was written
        uint32_t offset;    //!< byte offset into the corresponding `.raw`/`.dat` segment
        uint32_t msg_id;    //!< data ID of the record
        uint32_t reserved;  //!< unused
    } index_record_v1_t;

    /**
     * @brief Current (v2) `.idx` record type. Both spellings alias `is_log_idx_record_v2_t`.
     *
     * New code uses the v2 fields: `timestamp` is payload-derived milliseconds (falling back to
     * host-uptime delta when the payload carries none), `did` is the data ID (0 for the
     * streaming-only path), `flags` bit 0 signals a real ToW, and `offset` is a 64-bit byte offset.
     */
    using index_record_s = inertial_sense::idx::is_log_idx_record_v2_t;
    using index_record_t = inertial_sense::idx::is_log_idx_record_v2_t;    //!< preferred spelling for new code

    /** @brief Construct with no bound device; HardwareId()/SerialNumber() read as unset until SetSerialNumber() or SetupReadInfo() is called. */
    cDeviceLog();

    /**
     * @brief Construct bound to a live device, deriving hardware ID/serial number/device ID from it.
     * @param dev device to bind to; must not be null.
     * @throws std::invalid_argument if @p dev is null.
     */
    cDeviceLog(device_handle_t dev);

    /** @brief Construct for a device identified only by hardware ID + serial number, with no live `ISDevice` (e.g. reading a log from disk). */
    cDeviceLog(uint16_t hdwId, uint32_t serial);

    /** @brief Closes any open file (see OpenNewSaveFile()/OpenNextReadFile()) and calls CloseAllFiles(). */
    virtual ~cDeviceLog();

    /**
     * @brief Prepare this log for writing: reset rotation/index state and record where/how to write files.
     * @param timestamp timestamp string used to name new log files (see GetNewFileName()).
     * @param directory directory new log files are created in.
     * @param maxDiskSpace drive-usage limit forwarded from the owning `cISLogger`; not enforced by this class directly.
     * @param maxFileSize largest size, in bytes, a single log file should reach before rolling over.
     */
    virtual void InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFileSize);

    /** @brief Prepare this log for reading: resets size/count bookkeeping and log statistics. Call SetupReadInfo() first to discover the files to read. */
    virtual void InitDeviceForReading();

    /** @brief Close all files, then re-initialize for writing or reading (whichever mode was last active), effectively restarting the log from scratch. */
    virtual void ResetToStart();

    /**
     * @brief Close the current file. In write mode, flushes any buffered `.idx` records and
     *        finalizes the `.idx` header (see writeIndexChunk()/finalizeIndex()), then writes a
     *        per-device `summary_SN<serial>.txt` statistics file.
     * @return true (base implementation always succeeds).
     */
    virtual bool CloseAllFiles();

    /** @brief Flush buffered writes to disk without closing the file. Base implementation is a no-op that always returns true; formats with buffered I/O should override. */
    virtual bool FlushToFile() { return true; };

    /** @brief Open the current log file with the OS's default associated application (Windows only; a no-op elsewhere). @return true (base implementation always succeeds). */
    virtual bool OpenWithSystemApp();

    /**
     * @brief Record one already-parsed data set into this device's log statistics and `.idx` index.
     *
     * The base implementation only updates statistics and calls addIndexRecord() — it does not
     * write the data set's bytes anywhere; format subclasses that store parsed data (CSV/JSON/KML)
     * override this to actually serialize @p dataBuf.
     *
     * @param dataHdr header describing @p dataBuf's data ID, size, and offset; if null, statistics/indexing are skipped.
     * @param dataBuf the data payload described by @p dataHdr.
     * @param ptype protocol type the data arrived as; affects how its timestamp is derived.
     * @return true (base implementation always succeeds).
     */
    virtual bool SaveData(p_data_hdr_t *dataHdr, const uint8_t *dataBuf, protocol_type_t ptype = _PTYPE_INERTIAL_SENSE_DATA);

    /**
     * @brief Streaming raw-bytes save path (`LOGTYPE_RAW`/`LOGTYPE_DAT` only).
     *
     * The base implementation is a no-op: it has no parsed header to index or account for.
     * cDeviceLogRaw overrides this to parse individual packets out of the byte stream and emit its
     * own per-packet `.idx` records as it goes, since a coarse chunk-level record here (with no
     * DID) would be strictly less useful.
     *
     * @param dataSize number of bytes in @p dataBuf.
     * @param dataBuf the raw bytes to save.
     * @param globalLogStats the owning `cISLogger`'s aggregate statistics, for overrides that update it directly.
     * @return true (base implementation always succeeds).
     */
    virtual bool SaveData(int dataSize, const uint8_t *dataBuf, cLogStats &globalLogStats);

    /**
     * @brief Read the next data set from the log.
     * @return the next data set, or null if no more data is available. Pure virtual; every format subclass must implement this.
     */
    virtual p_data_buf_t *ReadData() = 0;

    /**
     * @brief Read the next raw packet from the log (raw/serial formats only).
     * @param[out] ptype protocol type of the returned packet.
     * @return the next packet. Base implementation always returns null; only cDeviceLogRaw overrides this.
     */
    virtual packet_t* ReadPacket(protocol_type_t& ptype) { (void)ptype; return NULL; };

    /**
     * @brief Set the device's serial number when there is no live `ISDevice` to derive it from (e.g. reading a log from disk).
     * @param serialNumber serial number to record. Pure virtual; every format subclass must implement this.
     */
    virtual void SetSerialNumber(uint32_t serialNumber) = 0;

    /** @brief Get the bound `ISDevice`, if any. @return the device, or null if this log was constructed without one. */
    device_handle_t getDevice() { return device; }

    /**
     * @brief Get this format's log file extension, including the leading dot (e.g. ".csv").
     * @return the file extension. Pure virtual; every format subclass must implement this.
     */
    virtual std::string LogFileExtention() = 0;

    /** @brief Force any pending buffered data out (distinct from FlushToFile(), which targets the OS file buffer). Base implementation is a no-op. */
    virtual void Flush() {}

    /**
     * @brief Discover the log file(s) for a device, for later reading via OpenNextReadFile().
     * @param directory directory to search.
     * @param deviceName serial number as a string; pass "0" together with an empty @p timeStamp to match any serial number's file by index only.
     * @param timeStamp timestamp string identifying the log instance directory/filename set to search for.
     * @return true (base implementation always succeeds, even if no matching files were found — check FileCount() after calling).
     */
    bool SetupReadInfo(const std::string &directory, const std::string &deviceName, const std::string &timeStamp);

    /** @brief Get the bound `ISDevice`. @return the device; behavior is undefined if this log was constructed without one (see getDevice() for a null-safe accessor). */
    device_handle_t Device();

    /** @brief Get the bound `ISDevice`'s device info. @return the device info; behavior is undefined if this log was constructed without a device. */
    dev_info_t DeviceInfo();

    /** @brief Get the device's encoded hardware ID. */
    uint16_t HardwareId() { return m_devHdwId; }

    /** @brief Get the device's serial number. */
    uint32_t SerialNumber() { return m_devSerialNo; }

    /** @brief Get the device's string identifier (hardware ID + serial number). */
    std::string& getDeviceId() { return m_deviceId; }

    /** @brief Total size, in bytes, of the currently open log file. */
    uint64_t FileSize() { return m_fileSize; }

    /** @brief Total size, in bytes, of this device's entire log (all files). */
    uint64_t LogSize() { return m_logSize; }

    /** @brief Number of log files written or discovered so far for this device. */
    uint32_t FileCount() { return m_fileCount; }

    /**
     * @brief Build a new log file's base name (directory + prefix + serial + timestamp + index), without the format's file extension.
     * @param serialNumber device serial number to embed in the filename.
     * @param fileCount file index to embed in the filename (wrapped to 4 digits, modulo 10000).
     * @param suffix optional suffix appended after the index (e.g. for a companion file); ignored if null or empty.
     * @return the base file name/path, with no extension.
     */
    std::string GetNewBaseFileName(uint32_t serialNumber, uint32_t fileCount, const char* suffix);

    /** @brief GetNewBaseFileName() with this format's LogFileExtention() appended. @return the full file name/path. */
    std::string GetNewFileName(uint32_t serialNumber, uint32_t fileCount, const char *suffix);

    /**
     * @brief Configure KML output options.
     * @param gnssData include GNSS position data in the KML output.
     * @param showTracks draw the flight/travel path as a line.
     * @param showPoints draw individual sample points along the path.
     * @param showPointTimestamps label sample points with their timestamp.
     * @param pointUpdatePeriodSec minimum time between consecutive drawn sample points, in seconds.
     * @param altClampToGround clamp altitude to ground level in the KML viewer instead of using the logged altitude.
     */
    void SetKmlConfig(bool gnssData = true, bool showTracks = true, bool showPoints = true, bool showPointTimestamps = true, double pointUpdatePeriodSec = 1.0, bool altClampToGround = true) {
        m_enableGnssLogging = gnssData;
        m_showTracks = showTracks;
        m_showPoints = showPoints;
        m_showPointTimestamps = showPointTimestamps;
        m_pointUpdatePeriodSec = pointUpdatePeriodSec;
        m_altClampToGround = altClampToGround;
    }

    /** @brief Enable or disable printing of parse errors encountered while reading this log. */
    void ShowParseErrors(bool show) { m_showParseErrors = show; }

    /** @brief Record one already-read data set into this device's log statistics, using its payload-derived timestamp. */
    void UpdateStatsFromFile(p_data_buf_t *data);

    /** @brief Record one already-read item into this device's log statistics using an explicit timestamp, when no full data set/header is available. */
    void UpdateStatsFromFile(protocol_type_t ptype, int id, double timestamp);

    /** @brief Get this device's log statistics, formatted as a human-readable string. */
    std::string LogStatsString() { return m_logStats.Stats(); }

    /** @brief Get the IS-comm parser instance backing this log, if any. @return the parser instance, or null. Only cDeviceLogRaw overrides this to return a real instance. */
    virtual is_comm_instance_t* IsCommInstance() { return NULL; }

    /**
     * @brief Buffer one `.idx` v2 record for the current segment, to be flushed by writeIndexChunk().
     *
     * When @p dataHdr is provided, the record's `did`/`timestamp`/ToW flag are derived from the
     * parsed header and payload (falling back to a host-uptime-delta timestamp with no ToW flag if
     * the payload carries no timestamp of its own). When called with no arguments — the streaming
     * path used by cDeviceLogRaw for callers with no parsed header — the record gets `did = 0` and
     * a host-uptime-delta timestamp.
     *
     * @param dataHdr header describing the data set this record indexes, or null for the streaming path.
     * @param dataBuf the data payload described by @p dataHdr; ignored if @p dataHdr is null.
     */
    void addIndexRecord(const p_data_hdr_t* dataHdr = nullptr,
                        const uint8_t* dataBuf = nullptr);

    /**
     * @brief Flush buffered `.idx` records (from addIndexRecord()) to the current segment's `.idx` sidecar file.
     *
     * Writes the `.idx` header on the first call for a segment (see OpenNewSaveFile(), which
     * resets the per-segment header/counter state). No-ops successfully if there's nothing
     * buffered and the header was already written. Fails without writing anything if no segment
     * file name has been set yet (avoids creating an orphan `.idx` with no corresponding data file).
     *
     * @return true on success (including the no-op case); false if no file name is set yet, the
     *         `.idx` file couldn't be opened, or a record/header write failed partway through.
     */
    bool writeIndexChunk();

    /**
     * @brief Rewrite the `.idx` header at offset 0 with final record totals/timestamps and set the FINALIZED flag.
     *
     * Idempotent: a no-op success if no header was ever written for this segment (i.e. no records
     * were ever indexed). Called by CloseAllFiles().
     *
     * @return true on success (including the no-op case); false if the `.idx` file couldn't be reopened, seeked to offset 0, or rewritten.
     */
    bool finalizeIndex();

protected:
    /**
     * @brief Close any currently open save file and open a new one, rolling the file index forward.
     *
     * Also resets the per-segment `.idx` bookkeeping (offset, header-written flag, record totals)
     * so the new segment gets its own `.idx` header rather than inheriting the prior segment's.
     *
     * @return true if the new file was created and opened successfully; false if `m_directory` is
     *         empty, the device has no resolvable serial number, or the file failed to open.
     */
    bool OpenNewSaveFile();

    /**
     * @brief Close any currently open read file and open the next one discovered by SetupReadInfo().
     * @return true if a next file was opened; false if every discovered file has already been consumed, or the open failed.
     */
    bool OpenNextReadFile();

    /** @brief Hook called after ReadPacket() successfully reads a packet, to record it in log statistics. */
    void OnReadPacket(packet_t* pkt, protocol_type_t ptype);

    /** @brief Hook called after ReadData() successfully reads a data set, to record it in log statistics. */
    void OnReadData(p_data_buf_t *data);

    device_handle_t device;                                     //!< ISDevice reference to source of data

    uint16_t m_devHdwId = 0;                                    //!< used when reading a file and no ISDevice is available
    uint32_t m_devSerialNo = static_cast<uint32_t>(-1);        //!< used when reading a file, and no ISDevice is available
    std::string m_deviceId;                                     //!< a string representation of a unique device id (hdwid+sn)

    std::vector<std::string> m_fileNames;      //!< files discovered by SetupReadInfo(), consumed in order by OpenNextReadFile()
    cISLogFileBase *m_pFile = NULL;             //!< currently open data file (write or read mode); owned by this instance
    cISLogFileBase *m_indexFile = NULL;         //!< currently unused: writeIndexChunk()/finalizeIndex() open their own local `.idx` file handles instead of this member
    std::string m_directory;                    //!< directory new files are created in (write mode) or were discovered in (read mode)
    std::string m_timeStamp;                    //!< timestamp string used to name new log files; see GetNewFileName()
    std::string m_fileName;                     //!< name of the currently open file, without its format extension
    bool m_writeMode = false;                                   //!< Logger initialized for writing
    bool m_showParseErrors = false;             //!< whether parse errors are printed while reading; see ShowParseErrors()
    uint64_t m_fileSize = 0;                    //!< size, in bytes, of the currently open file
    uint64_t m_logSize = 0;                     //!< total size, in bytes, of this device's entire log (all files)
    uint32_t m_fileCount = 0;                   //!< number of files written (write mode) or consumed so far (read mode)
    uint64_t m_maxDiskSpace;                    //!< drive-usage limit forwarded from the owning cISLogger; not enforced directly by this class
    uint32_t m_maxFileSize;                     //!< largest size, in bytes, a single log file should reach before rolling over
    bool m_altClampToGround = true;             //!< KML config: clamp altitude to ground level; see SetKmlConfig()
    bool m_enableGnssLogging = true;            //!< KML config: include GNSS position data; see SetKmlConfig()
    bool m_showTracks = true;                   //!< KML config: draw the path as a line; see SetKmlConfig()
    bool m_showPoints;                          //!< KML config: draw individual sample points; see SetKmlConfig()
    bool m_showPointTimestamps = true;          //!< KML config: label sample points with their timestamp; see SetKmlConfig()
    double m_pointUpdatePeriodSec = 1.0f;       //!< KML config: minimum time between drawn sample points, in seconds; see SetKmlConfig()
    cLogStats m_logStats;                       //!< this device's aggregate data/error counters
    std::vector<index_record_t> m_indexChunks;      //!< v2 records buffered in memory, waiting to be flushed via writeIndexChunk()
    uint64_t m_lastIndexOffset = 0;                 //!< running byte offset into the .raw segment; v2 widened from u32 to u64 since segments can exceed 4GB
    uint32_t m_logStartUpTime = 0;                  //!< the system uptime (in millis) at the moment this index was created
    uint32_t m_lastIndexTime = 0;                   //!< system uptime (in ms) of the last index record; v2 still tracks this for the host-uptime fallback path
    uint64_t m_captureEpochMs = 0;                  //!< SN-8340: absolute host wall-clock (ms since Unix epoch) captured at log-open; written to the v2.1 .idx header so a no-GPS-ever log still has a wall-clock anchor

    // D-01 / SN-7879 v2 .idx bookkeeping. The header at offset 0 of
    // the .idx file carries these aggregates; finalizeIndex() seeks(0)
    // and rewrites the header on close.
    bool     m_idxHeaderWritten = false;            //!< true once the v2 header has been emitted (first writeIndexChunk call)
    uint64_t m_idxTotalRecords  = 0;                //!< running count of records written to the .idx file
    uint64_t m_idxFirstTimestampMs = 0;             //!< timestamp of the first record (set on first record)
    uint64_t m_idxLastTimestampMs  = 0;             //!< timestamp of the most recent record

};

#endif // DEVICE_LOG_H
