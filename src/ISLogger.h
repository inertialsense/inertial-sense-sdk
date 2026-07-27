/**
 * @file ISLogger.h
 * @brief Top-level logging orchestrator: fans out data/packets to one `cDeviceLog` per device,
 *        and owns the on-disk log directory (drive-usage limiting, file culling, statistics).
 *
 * `cISLogger` is the entry point applications use to write or read device logs; it does not
 * serialize any data itself — each registered device gets a format-specific `cDeviceLog`
 * (cDeviceLogSerial/Raw/CSV/JSON/KML, chosen by `eLogType`) that owns the actual file I/O. The
 * logger's job is device bookkeeping (registerDevice()/DeviceLogs()), routing LogData()/ReadData()
 * calls to the right device's log, and directory-level housekeeping in Update() (idle-timeout
 * flushing and drive-space-limited file culling via ISFileManager).
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_LOGGER_H
#define IS_LOGGER_H

#define _FILE_OFFSET_BITS 64

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <memory>

#include "DeviceLogSerial.h"
#include "DeviceLogRaw.h"
 
#if !defined(PLATFORM_IS_EVB_2) || !PLATFORM_IS_EVB_2
#include "DeviceLogCSV.h"
#include "DeviceLogJSON.h"
#include "DeviceLogKML.h"
#endif

#if PLATFORM_IS_EVB_2
#include "d_time.h"
#include "ISLogFileFatFs.h"
#else
#include "ISLogFile.h"
#endif

#include "ISConstants.h"
#include "ISLogStats.h"


// default logging path if none specified
#define DEFAULT_LOGS_DIRECTORY          "IS_logs"
#define DEFAULT_LOGS_MAX_FILE_SIZE      (1024 * 1024 * 5)    // 5 MB

/**
 * @brief Top-level device-log orchestrator; see the file-level documentation above for the overview.
 */
class cISLogger
{
public:
    /** @brief Selects which `cDeviceLog` subclass (and on-disk format) new devices are logged with. */
    enum eLogType
    {
        LOGTYPE_DAT = 0,    //!< raw serial byte stream, unparsed (cDeviceLogSerial, `.dat`)
        LOGTYPE_RAW,        //!< packetized serial stream; supports multiple packet types (cDeviceLogRaw, `.raw`)
        LOGTYPE_CSV,        //!< one CSV file per data set (cDeviceLogCSV, `.csv`)
        LOGTYPE_KML,        //!< KML flight-path visualization, write-only (cDeviceLogKML, `.kml`)
        LOGTYPE_JSON,       //!< one JSON-lines file per data set (cDeviceLogJSON, `.json`)
        LOGTYPE_COUNT       //!< number of log types; not a valid log type itself
    };

    /** @brief Display/file-extension name for each `eLogType`, indexed by the enum value. */
    static const char* logTypeStrings[LOGTYPE_COUNT];

    /** @brief Options controlling how InitSave() lays out and limits a new log directory. */
    struct sSaveOptions
    {
        eLogType logType;                           //!< file format to use for the logger
        float driveUsageLimitPercent;               //!< limit of drive usage, as a fraction (0.0-1.0) of the total drive size; 0 disables this limit
        float driveUsageLimitMb;                    //!< limit of drive usage in megabytes; 0 disables this limit
        uint32_t maxFileSize;                       //!< largest size of each individual log file, in bytes
        bool useSubFolderTimestamp;                 //!< write each log instance to its own timestamped subfolder of `directory`
        std::string timeStamp;                      //!< name used for the log instance directory; current system date/time is used if left empty
        std::string subDirectory;                   //!< write logs into a subdirectory of this name inside the log instance directory, if non-empty

        /**
         * @brief Construct with default options: raw packetized logging, 50% drive-usage limit, 5 MB files, timestamped subfolder.
         * @param type file format to use for the logger.
         * @param limitPercent drive-usage limit as a fraction (0.0-1.0) of total drive size; 0 disables this limit.
         * @param limitMb drive-usage limit in megabytes; 0 disables this limit.
         * @param fileSize size, in bytes, of each individual file in the log.
         * @param useTimestamp if true, a timestamped subdirectory is created for the new log.
         * @param subDir if non-empty, logs are written into a subdirectory of this name within the log instance directory.
         */
        sSaveOptions(
            eLogType type = LOGTYPE_RAW,
            float limitPercent = 0.5,
            float limitMb = 0,
            uint32_t fileSize = 5 * 1024 * 1024,
            bool useTimestamp = true,
            std::string subDir = ""
      ) : logType(type),
            driveUsageLimitPercent(limitPercent),
            driveUsageLimitMb(limitMb),
            maxFileSize(fileSize),
            useSubFolderTimestamp(useTimestamp),
            subDirectory(subDir)
        {}
    };

    static const std::string g_emptyString;    //!< shared empty-string default for optional `const std::string&` parameters

    cISLogger();
    virtual ~cISLogger();

    /**
     * @brief Set up the logger to read an existing log from disk.
     *
     * Scans @p directory for files matching @p logType's extension, groups them by the serial
     * number embedded in their filename (see ParseFilename()), and creates one reading-mode
     * `cDeviceLog` per unique serial number found (filtered to @p serials, if non-empty).
     * `LOGTYPE_KML` is not supported for reading and always fails.
     *
     * @param directory directory to scan for log files.
     * @param logType log format to look for; determines the file extension matched.
     * @param serials if non-empty, only devices whose serial number (as a string) appears in this
     *        list are loaded; the literal value "ALL" also matches every serial number found.
     * @return true if at least one device log was found and set up for reading, false otherwise.
     */
    bool LoadFromDirectory(const std::string& directory, eLogType logType = LOGTYPE_RAW, std::vector<std::string> serials = {});

    /**
     * @brief Set up the logger to write a new log to disk.
     *
     * Closes any currently-open files, then creates the log directory tree under @p directory
     * (optionally a timestamped subfolder and/or `options.subDirectory`), and computes the
     * effective drive-usage limit from `options.driveUsageLimitPercent`/`driveUsageLimitMb`
     * clamped to actually-available disk space. Devices registered afterward via registerDevice()
     * use this directory and limit.
     *
     * @param directory root directory to write logs into; DEFAULT_LOGS_DIRECTORY ("IS_logs") is used if empty.
     * @param options save options; see sSaveOptions.
     * @return true if the log directory was created successfully, false otherwise.
     */
    bool InitSave(const std::string& directory = g_emptyString, const sSaveOptions& options = cISLogger::sSaveOptions());

    /** @deprecated Use InitSave(const std::string&, const sSaveOptions&) instead. */
    [[deprecated("Not recommended for future development.  Use InitSave() with sSaveOptions instead.")]]
    bool InitSave(eLogType logType = LOGTYPE_RAW, const std::string& directory = g_emptyString, float driveUsageLimitPercent = 0.5f, uint32_t maxFileSize = 5 * 1024 * 1024, bool useSubFolderTimestamp = true);

    /** @deprecated Use InitSave(const std::string&, const sSaveOptions&) instead. */
    [[deprecated("Not recommended for future development.  Use InitSave() with sSaveOptions instead.")]]
    bool InitSaveTimestamp(const std::string& timeStamp, const std::string& directory = g_emptyString, const std::string& subDirectory = g_emptyString, eLogType logType = LOGTYPE_DAT, float driveUsageLimitPercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, bool useSubFolderTimestamp = true);

    /**
     * @brief Establish a link between @p device and this logger, creating its `cDeviceLog` if needed.
     *
     * The concrete `cDeviceLog` subclass created is determined by `m_logType` (set by InitSave()).
     * If @p device already has a `devLogger`, it is reused. Also wires the port so incoming data on
     * @p device is automatically routed here via logPortData().
     *
     * @param device device to register; a no-op returning null if null.
     * @return the device's `cDeviceLog`, or null if @p device was null.
     */
    std::shared_ptr<cDeviceLog> registerDevice(device_handle_t device);

    /**
     * @brief Establish a link between a device identified only by hardware ID + serial number and this logger.
     *
     * Use this overload when there is no live `ISDevice` (e.g. setting up devices ahead of connection).
     * The concrete `cDeviceLog` subclass created is determined by `m_logType` (set by InitSave()).
     *
     * @param hdwId encoded hardware ID of the device.
     * @param serialNo serial number of the device.
     * @return the newly created `cDeviceLog`.
     */
    std::shared_ptr<cDeviceLog> registerDevice(uint16_t hdwId, uint32_t serialNo);

    /** @brief Convenience overload of registerDevice() that derives the hardware ID from a `dev_info_t`. */
    std::shared_ptr<cDeviceLog> registerDevice(dev_info_t& devInfo) { return registerDevice(ENCODE_DEV_INFO_TO_HDW_ID(devInfo), devInfo.serialNumber); }

    /**
     * @brief Periodic housekeeping: idle-timeout flushing and drive-space-limited file culling.
     *
     * Call regularly (e.g. once per main loop iteration). If no data has been logged for
     * `TimeoutFlushSeconds()` seconds, flushes every registered device's log. Independently, at
     * most once every 10 seconds, if logging is enabled and a disk-space limit is set, checks
     * actual disk usage and — if over the limit — removes the oldest log files/directories via
     * ISFileManager until back under the limit.
     */
    void Update();

    /**
     * @brief Log one already-parsed data set to @p devLogger. Not valid when the logger's type is `LOGTYPE_RAW`.
     * @param devLogger destination device log; logging is skipped (returns false) if null.
     * @param dataHdr header describing @p dataBuf's data ID, size, and offset.
     * @param dataBuf the data payload described by @p dataHdr.
     * @return true if the call was handled (including recorded failures such as a corrupt header
     *         or a save failure, which are logged to the error file and counted in the log
     *         statistics); false if logging is disabled, @p devLogger is null, or the logger's
     *         type is `LOGTYPE_RAW`.
     */
    bool LogData(std::shared_ptr<cDeviceLog> devLogger, p_data_hdr_t* dataHdr, const uint8_t* dataBuf);

    /**
     * @brief Log one raw byte buffer to @p devLogger. Only valid when the logger's type is `LOGTYPE_RAW`.
     * @param devLogger destination device log; if null and exactly one device is registered, that
     *        device is used instead.
     * @param dataSize number of bytes in @p dataBuf.
     * @param dataBuf the raw bytes to log.
     * @return true if the call was handled (including a recorded save failure); false if logging
     *         is disabled, the logger's type isn't `LOGTYPE_RAW`, no device could be resolved, or
     *         @p dataSize/@p dataBuf are invalid.
     */
    bool LogData(std::shared_ptr<cDeviceLog> devLogger, int dataSize, const uint8_t* dataBuf);

    /** @brief LogData() overload that looks up the destination device by serial number via DeviceLogBySerialNumber(). */
    bool LogDataBySN(uint32_t serialNo, p_data_hdr_t* dataHdr, const uint8_t* dataBuf) { return LogData(DeviceLogBySerialNumber(serialNo), dataHdr, dataBuf); }

    /** @brief LogData() overload that looks up the destination device by serial number via DeviceLogBySerialNumber(). */
    bool LogDataBySN(uint32_t serialNo, int dataSize, const uint8_t* dataBuf) {  return LogData(DeviceLogBySerialNumber(serialNo), dataSize, dataBuf); }

    /**
     * @brief Read the next data set from @p devLogger, skipping over (and logging as errors) any corrupt records.
     * @param devLogger device log to read from; defaults to null, which returns null.
     * @return the next data set, or null if @p devLogger is null or no more data is available.
     */
    p_data_buf_t* ReadData(std::shared_ptr<cDeviceLog> devLogger = nullptr);

    /** @brief ReadData() overload that looks up the device by its index in DeviceLogs(). @return the next data set, or null if @p devIndex is out of range or no more data is available. */
    p_data_buf_t* ReadData(size_t devIndex);

    /**
     * @brief Read the next data set across all registered devices, round-robin starting at @p devIndex.
     * @param[in,out] devIndex index of the device to try first; advanced to the next device on a
     *        miss, and reset to 0 once every device has been tried with no data available.
     * @return the next available data set from some device, or null if no device had data available.
     */
    p_data_buf_t* ReadNextData(size_t& devIndex);

    /**
     * @brief Read the next packet from @p devLogger (raw/serial log formats only).
     * @param[out] ptype protocol type of the returned packet; set to `_PTYPE_PARSE_ERROR` if a corrupt packet was encountered (and logged as an error).
     * @param devLogger device log to read from; defaults to null, which returns null.
     * @return the next packet, or null if @p devLogger is null or no more packets are available.
     */
    packet_t* ReadPacket(protocol_type_t& ptype, std::shared_ptr<cDeviceLog> devLogger = nullptr);

    /** @brief ReadPacket() overload that looks up the device by its index in DeviceLogs(). @return the next packet, or null if @p devIndex is out of range or no more packets are available. */
    packet_t* ReadPacket(protocol_type_t& ptype, size_t devIndex);

    /**
     * @brief Read the next packet across all registered devices, round-robin starting at @p devIndex.
     * @param[out] ptype protocol type of the returned packet.
     * @param[in,out] devIndex index of the device to try first; advanced to the next device on a miss.
     * @return the next available packet from some device, or null if no device had one available.
     */
    packet_t* ReadNextPacket(protocol_type_t& ptype, size_t& devIndex);

    /** @brief Enable or disable logging. When disabled, LogData() is a no-op. */
    void EnableLogging(bool enabled) { m_enabled = enabled; }

    /** @brief Check whether logging is currently enabled. */
    bool Enabled() { return m_enabled; }

    /** @brief Close every registered device's open log files and write final statistics to `stats_all.txt`. */
    void CloseAllFiles();

    /** @brief Flush every registered device's log to disk without closing it. */
    void FlushToFile();

    /** @brief Open every registered device's current log file with the OS's default associated application. */
    void OpenWithSystemApp();

    /** @brief Enable or disable printing of parse errors, for this logger and every currently-registered device. */
    void ShowParseErrors(bool show);

    /** @brief Get the timestamp string used to name the current log instance directory. */
    std::string TimeStamp() { return m_timeStamp; }

    /** @brief Get the directory logs are currently being written to (includes any timestamp/sub-directory components). */
    std::string LogDirectory() { return m_directory; }

    /** @brief Get the root log directory passed to InitSave() (excludes any timestamp/sub-directory components). */
    std::string RootDirectory() { return m_rootDirectory; }

    /** @brief Total size, in bytes, of all registered devices' logs. */
    uint64_t LogSizeAll();

    /** @brief Size, in bytes, of the log for the device with the given serial number. @return 0 if no such device is registered. */
    uint64_t LogSize(uint32_t devSerialNo);

    /** @brief LogSizeAll(), in megabytes. */
    float LogSizeAllMB();

    /** @brief LogSize(), in megabytes. @return 0 if no such device is registered. */
    float LogSizeMB(uint32_t devSerialNo);

    /** @brief Size, in megabytes, of the current log file (not the whole log) for the device with the given serial number. @return 0 if no such device is registered. */
    float FileSizeMB(uint32_t devSerialNo);

    /** @brief Number of log files written so far for the device with the given serial number. @return 0 if no such device is registered. */
    uint32_t FileCount(uint32_t devSerialNo);

    /** @brief Configured drive-usage limit, in megabytes; 0 if file culling is disabled. */
    float MaxDiskSpaceMB() { return ((float)m_maxDiskSpace) / (1024*1024); }

    /** @brief Drive usage of the root log directory, in megabytes, as of the last Update() culling check. */
    float UsedDiskSpaceMB() { return ((float)m_usedDiskSpace) / (1024*1024); }

    /** @brief Get the next log file name for the device with the given serial number. @return the file name, or an empty string if no such device is registered. */
    std::string GetNewFileName(uint32_t devSerialNo, uint32_t fileCount, const char* suffix);

    /** @brief Get every currently-registered device's `cDeviceLog`. */
    std::vector<std::shared_ptr<cDeviceLog>> DeviceLogs();

    /** @brief Number of currently-registered devices. */
    uint32_t DeviceCount() { return (uint32_t)m_devices.size(); }

    /** @brief Look up a registered device's `cDeviceLog` by serial number. @return the device's log, or null if no device with that serial number is registered. */
    std::shared_ptr<cDeviceLog> DeviceLogBySerialNumber(uint32_t serialNo) {
        return (m_devices.count(serialNo) ? m_devices[serialNo] : nullptr);
    }

    /** @brief Unregister and release every device, detaching each from its port and clearing log statistics. */
    void Cleanup();

    // bool SetDeviceInfo(const dev_info_t *info, unsigned int device = 0);
    // const dev_info_t* DeviceInfo(unsigned int device = 0);

    /**
     * @brief Copy (convert) one log to another log type.
     * @param log input log to read and convert.
     * @param timestamp timestamp to use for the output log's instance directory; current date/time if empty.
     * @param outputDir directory to write the output log to.
     * @param logType format of the output log.
     * @param maxFileSize max size, in bytes, of each individual output file.
     * @param driveUsageLimitPercent drive usage limit, as a fraction (0.0-1.0) of total drive space.
     * @param useSubFolderTimestamp output logs should be written into a timestamped subdirectory.
     * @param enableCsvIns2ToIns1Conversion when @p logType is `LOGTYPE_CSV`, convert `DID_INS_2` records to `DID_INS_1` in the output (skipped if the source log already contains `DID_INS_1`).
     * @return true if the log copy (conversion) completed successfully, false if the output log's InitSave() failed.
     */
    bool CopyLog(
        cISLogger& log,
        const std::string& timestamp = g_emptyString,
        const std::string& outputDir = g_emptyString,
        eLogType logType = LOGTYPE_DAT,
        uint32_t maxFileSize = DEFAULT_LOGS_MAX_FILE_SIZE,
        float driveUsageLimitPercent = 0.5f,
        bool useSubFolderTimestamp = true,
        bool enableCsvIns2ToIns1Conversion = true);

    /** @brief Total number of data sets successfully logged across all registered devices. */
    unsigned int Count() { return m_logStats.Count(); }

    /** @brief Total number of logging errors (corrupt headers, failed saves) across all registered devices. */
    unsigned int Errors() { return m_logStats.Errors(); }

    /** @brief The log format this logger was configured with, via InitSave() or LoadFromDirectory(). */
    eLogType Type() { return m_logType; }

    /**
     * @brief Get the timeout flush parameter in seconds.
     * @return the timeout flush parameter in seconds.
     */
    time_t TimeoutFlushSeconds() { return m_timeoutFlushSeconds; }

    /**
     * @brief Set the idle-timeout flush parameter used by Update().
     * @param timeoutFlushSeconds seconds of inactivity after which Update() flushes every device's
     *        log; 0 disables idle-timeout flushing.
     */
    void SetTimeoutFlushSeconds(time_t timeoutFlushSeconds) { m_timeoutFlushSeconds = timeoutFlushSeconds; }

    /**
     * @brief Check whether a data header's fields are self-consistent.
     * @param hdr header to validate; a null header is always considered corrupt.
     * @return true if @p hdr is null, has a zero size, a zero id, an offset that isn't a multiple
     *         of 4, or an offset+size that exceeds MAX_DATASET_SIZE; false otherwise.
     */
    static bool isHeaderCorrupt(const p_data_hdr_t* hdr);

    /** @brief Build a timestamp string (`YYYYMMDD_HHMMSS`) from the current date/time, used to name new log instance directories. */
    static std::string CreateCurrentTimestamp();

    /**
     * @brief Check whether a data packet's header is corrupt, per isHeaderCorrupt().
     * @param data packet to check; null is never considered corrupt (returns false).
     * @return false if chunk-header validation is disabled (`LOGTYPE_RAW`, which has no chunk
     *         header) or @p data is null; otherwise the result of isHeaderCorrupt() on its header.
     */
    bool isDataCorrupt(const p_data_buf_t* data);

    // read all log data into memory - if the log is over 1.5 GB this will fail on 32 bit processes
    // the map contains device id (serial number) key and a vector containing log data for each data id, which will be an empty vector if no log data for that id
    // static bool ReadAllLogDataIntoMemory(const std::string& directory, std::map<uint32_t, std::vector<std::vector<uint8_t>>>& data);

    /**
     * @brief Configure KML output options and apply them to every currently-registered device.
     * @param gpsData include GNSS position data in the KML output.
     * @param showPath draw the flight/travel path as a line.
     * @param showSample draw individual sample points along the path.
     * @param showTimeStamp label sample points with their timestamp.
     * @param updatePeriodSec minimum time between consecutive drawn sample points, in seconds.
     * @param altClampToGround clamp altitude to ground level in the KML viewer instead of using the logged altitude.
     */
    void SetKmlConfig(bool gpsData = true, bool showPath = true, bool showSample = false, bool showTimeStamp = true, double updatePeriodSec = 1.0, bool altClampToGround = true)
    {
        m_gpsData = gpsData;
        m_showPath = showPath;
        m_showSample = showSample;
        m_showTimeStamp = showTimeStamp;
        m_iconUpdatePeriodSec = updatePeriodSec;
        m_altClampToGround = altClampToGround;

        for (auto d : DeviceLogs())
        {
            d->SetKmlConfig(m_gpsData, m_showPath, m_showSample, m_showTimeStamp, m_iconUpdatePeriodSec, m_altClampToGround);
        }
    }

    /** @brief Map a log-type name ("csv", "kml", "json", "raw") to its `eLogType`. @return the matching `eLogType`, or `LOGTYPE_DAT` if @p logTypeString matches none of the recognized names. */
    static eLogType ParseLogType(const std::string& logTypeString)
    {
        if (logTypeString == "csv")
        {
            return cISLogger::eLogType::LOGTYPE_CSV;
        }
        else if (logTypeString == "kml")
        {
            return cISLogger::eLogType::LOGTYPE_KML;
        }
        else if (logTypeString == "json")
        {
            return cISLogger::eLogType::LOGTYPE_JSON;
        }
        else if (logTypeString == "raw")
        {
            return cISLogger::eLogType::LOGTYPE_RAW;
        }
        return cISLogger::eLogType::LOGTYPE_DAT;
    }

    /**
     * @brief Parse a log filename into its serial number, date, time, and file-index components.
     *
     * Recognizes two forms: `IS_LOG_FILE_PREFIX` followed by `<serial>_<date>_<time>_<index>`
     * (e.g. "LOG_SN30013_20170103_151023_001.raw"), or a bare `<index>` with no prefix. Any other
     * form fails to parse.
     *
     * @param filename filename to parse (extension is stripped internally; path should already be removed).
     * @param[out] serialNum parsed serial number; 0 if the bare-index form was used.
     * @param[out] date parsed date string; empty if the bare-index form was used.
     * @param[out] time parsed time string; empty if the bare-index form was used.
     * @param[out] index parsed file index; -1 if parsing failed before reaching it.
     * @return true if @p filename matched one of the two recognized forms, false otherwise.
     */
    static bool ParseFilename(std::string filename, int &serialNum, std::string &date, std::string &time, int &index);

    /** @brief Print per-device message statistics. @note currently a no-op; retained as a hook for future diagnostics. */
    void PrintStatistics();

    /** @brief Print IS-comm parser status for every registered device. @note currently a no-op; retained as a hook for future diagnostics. */
    void PrintIsCommStatus();

    /** @brief Print elapsed logging time, current log size, and disk usage/limit to stdout. */
    void PrintLogDiskUsage();

    /**
     * @brief Look up the registered device (if any) whose `ISDevice` is bound to @p port.
     *
     * Used by logPortData() to route port callbacks to the correct `cDeviceLog`, since the
     * callback only receives the low-level `port_handle_t`, not a device or log reference.
     *
     * @param port port handle to search for.
     * @return the matching device's `cDeviceLog`, or null if no registered device uses @p port.
     */
    std::shared_ptr<cDeviceLog> getDeviceLogByPort(port_handle_t port);

    /**
     * @brief Port data callback wired up by registerDevice() via `portSetLogger()`.
     *
     * Resolves @p port to a registered device via getDeviceLogByPort() and logs @p buf to it as
     * raw data. As a logger, only outbound/inbound byte data is of interest here regardless of
     * @p op's direction, so @p op is intentionally ignored.
     *
     * @param port port the data was sent/received on.
     * @param op unused; direction of the data (send vs. receive) doesn't change how it's logged.
     * @param buf the data bytes to log.
     * @param len number of bytes in @p buf.
     * @param userData the `cISLogger*` this callback was registered with.
     * @return 1 if the data was logged successfully, -1 if @p userData is null, no device matches
     *         @p port, or the underlying LogData() call failed.
     */
    static inline int logPortData(port_handle_t port, uint8_t op, const uint8_t* buf, unsigned int len, void* userData) {
        (void)op; // Suppress unused parameter warning
        // remember, that as a logger, we GENERALLY are only interested in WRITING data, regardless of whether that data is sent or received.
        if (!userData)
            return -1;

        cISLogger* logInstance = (cISLogger*)userData;
        auto devLog = logInstance->getDeviceLogByPort(port);
        return (devLog && logInstance->LogData(devLog, len, buf)) ? 1 : -1;
    }

private:
#if CPP11_IS_ENABLED
    cISLogger(const cISLogger& copy) = delete;
#else
    cISLogger(const cISLogger& copy); // Disable copy constructors
#endif

    /** @brief Re-register every device in @p devices from scratch (Cleanup() first, then registerDevice() each). @return true if the resulting log directory exists. */
    bool InitDevicesForWriting(std::vector<device_handle_t>& devices);

    /** @brief Debug progress-dot printer; currently disabled (body compiled out). */
    void PrintProgress();

    /** @brief Current time in seconds: device RTC/system uptime on EVB-2, wall-clock `time()` elsewhere. */
    static time_t GetTime()
    {
#if PLATFORM_IS_EVB_2
        return static_cast<time_t>(time_msec() / 1000);
#else
        return time(NULLPTR);
#endif
    }

    eLogType        m_logType = LOGTYPE_DAT;                              //!< log format devices are registered with; set by InitSave()/LoadFromDirectory()
    bool            m_useChunkHeader = true;                              //!< true unless reading a `LOGTYPE_RAW` log, which has no per-record chunk header to validate
    bool            m_enabled = false;                                    //!< logging on/off switch; see EnableLogging()
    std::string     m_rootDirectory;                                      //!< root log directory passed to InitSave(), before any timestamp/sub-directory is appended
    std::string     m_directory;                                          //!< directory actually being written to (root + timestamp + sub-directory)
    std::string     m_timeStamp;                                          //!< timestamp string naming the current log instance directory
    std::map<uint32_t, std::shared_ptr<cDeviceLog>> m_devices = { };      //!< registered devices, keyed by serial number

    uint64_t        m_maxDiskSpace = 0;        //!< drive-usage limit for file culling, in bytes; 0 disables file culling
    uint64_t        m_usedDiskSpace = 0;       //!< drive space used by m_rootDirectory, as of the last Update() culling check
    uint32_t        m_maxFileSize = 0;         //!< largest size, in bytes, of each individual log file
    cLogStats       m_logStats;                //!< aggregate data/error counters across all registered devices
#if PLATFORM_IS_EVB_2
    cISLogFileFatFs m_errorFile;   //!< sink for LogData()/ReadData() error messages
#else
    cISLogFile      m_errorFile;   //!< sink for LogData()/ReadData() error messages
#endif

    bool            m_altClampToGround = false;    //!< KML config: clamp altitude to ground level; see SetKmlConfig()
    bool            m_gpsData = false;              //!< KML config: include GNSS position data; see SetKmlConfig()
    bool            m_showSample = false;            //!< KML config: draw individual sample points; see SetKmlConfig()
    bool            m_showPath = false;              //!< KML config: draw the path as a line; see SetKmlConfig()
    bool            m_showTimeStamp = false;         //!< KML config: label sample points with their timestamp; see SetKmlConfig()
    double          m_iconUpdatePeriodSec = false;   //!< KML config: minimum time between drawn sample points, in seconds; see SetKmlConfig()
    time_t          m_logStartTime = 0;              //!< time InitSave() was called; used by PrintLogDiskUsage() to compute elapsed time
    time_t          m_lastCommTime = 0;              //!< time of the last successful LogData() call; used by Update() for idle-timeout flushing
    time_t          m_timeoutFlushSeconds = 0;       //!< idle-timeout flush parameter; see SetTimeoutFlushSeconds()
    time_t          m_timeoutFileCullingSeconds = 10; //!< minimum seconds between Update()'s disk-usage/file-culling checks
    time_t          m_lastFileCullingTime = 0;       //!< time of the last file-culling check performed by Update()
    int             m_progress = 0;                  //!< progress counter for PrintProgress(), currently unused (PrintProgress() body is compiled out)
    bool            m_showParseErrors = true;        //!< whether parse errors are printed; see ShowParseErrors()
};

#endif // IS_LOGGER_H
