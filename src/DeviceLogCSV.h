/**
 * @file DeviceLogCSV.h
 * @brief CSV log format: one CSV file per data set (DID), reassembled into a single time-ordered
 *        stream on read via an incrementing per-row order id.
 *
 * Unlike the raw/serial formats, a CSV log has no single file to read sequentially — each data set
 * gets its own file (see cCsvLog), so ReadData() does a k-way merge across every currently-open
 * per-DID file, picking the row with the lowest cCsvLog::orderId (the value written into column 0
 * at write time) to reconstruct the original write order across data sets.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef DEVICE_LOG_CSV_H
#define DEVICE_LOG_CSV_H

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "DataCSV.h"
#include "DeviceLog.h"
#include "com_manager.h"

/** @brief Per-data-set (per-DID) CSV file state, both for writing and for reading. */
class cCsvLog
{
public:
    cCsvLog() : pFile(NULL), fileCount(0), fileSize(0), dataId(0), orderId(0) { }

    FILE* pFile;                                //!< currently open file for this data set; null if not yet opened
    uint32_t fileCount;                         //!< number of files written/opened so far for this data set
    uint64_t fileSize;                           //!< size, in bytes, of the currently open file
    uint32_t dataId;                             //!< data ID (DID) this log holds rows for
    uint32_t dataSize;                           //!< size, in bytes, of the @ref dataId struct (read mode only)
    uint64_t orderId;                            //!< order id parsed from column 0 of @ref nextLine; used to merge multiple data sets' files back into write order
    std::string nextLine;                        //!< next unconsumed CSV row read from @ref pFile (read mode only)
    std::vector<data_info_t> columnHeaders;      //!< column layout parsed from the file's header row (read mode only)
};


/** @brief `cDeviceLog` implementation that writes/reads one CSV file per data set. */
class cDeviceLogCSV : public cDeviceLog
{
public:
    cDeviceLogCSV() : cDeviceLog() {};
    cDeviceLogCSV(device_handle_t dev) : cDeviceLog(dev) {};
    cDeviceLogCSV(uint16_t hdwId, uint32_t serialNo) : cDeviceLog(hdwId, serialNo) {};

    void InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFileSize) OVERRIDE;

    /** @brief Discover every existing per-DID CSV file for this device and open the first file (with data) for each. */
    void InitDeviceForReading() OVERRIDE;

    bool CloseAllFiles() OVERRIDE;

    /** @brief Write one data set as a new row to its data set's CSV file, opening/rotating files as needed (see OpenNewFile()). */
    bool SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf, protocol_type_t ptype=_PTYPE_INERTIAL_SENSE_DATA) OVERRIDE;

    /** @brief Read the next data set in original write order, merged across every open per-data-set file by cCsvLog::orderId. @return the next data set, or null if every data set's files are exhausted. */
    p_data_buf_t* ReadData() OVERRIDE;

    void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
    std::string LogFileExtention() OVERRIDE { return std::string(".csv"); }

private:
    /**
     * @brief Open the next file for @p log, either for writing (a new file) or reading (the next discovered file).
     * @param log data set to open a file for.
     * @param readOnly if true, advance to and open the next file in `m_currentFiles[log.dataId]`
     *        (skipping files whose header fails to parse); if false, create and open a new write file.
     * @return true if a file was opened successfully; false if the data set is unknown, `m_directory`
     *         is empty, no serial number is resolvable, no files remain to read, or the open failed
     *         (in the failure/exhausted case, @p log's entry is removed from `m_logs`).
     */
    bool OpenNewFile(cCsvLog& log, bool readOnly);

    /**
     * @brief Read the next row from @p log's open file into `log.nextLine`/`log.orderId` (read mode only).
     * @param log data set to read the next line for.
     * @return true if a row was read and its order id (column 0) parsed; false on EOF, no open file, or a row with no comma-separated order id.
     */
    bool GetNextLineForFile(cCsvLog& log);

    /**
     * @brief Parse @p log's currently buffered row (from GetNextLineForFile()) into a data set, then advance to the next row.
     * @param log data set to read from.
     * @return the parsed data set, or null if the row failed to parse as @p log's data ID.
     */
    p_data_buf_t* ReadDataFromFile(cCsvLog& log);

    std::map<uint32_t, cCsvLog> m_logs;                                   //!< open per-data-set state, keyed by data ID
    cDataCSV m_csv;                                                        //!< CSV (de)serialization helper shared across all data sets
    std::map<uint32_t, std::vector<std::string> > m_currentFiles;         //!< all discovered files for each data set (read mode)
    std::map<uint32_t, uint32_t> m_currentFileIndex;                      //!< current index into `m_currentFiles[id]` for each data set (read mode)
    p_data_buf_t m_data;                                                   //!< scratch buffer for the most recently read data set
    uint64_t m_nextId;                                                     //!< next order id to write into column 0; lets ReadData() reconstruct write order across data sets
};

#endif // DEVICE_LOG_CSV_H
