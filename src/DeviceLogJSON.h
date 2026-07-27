/**
 * @file DeviceLogJSON.h
 * @brief JSON log format: every data set for a device is written as one JSON object into a single
 *        top-level JSON array per log file (unlike DeviceLogCSV, which splits by data set).
 *
 * A file is a `[` ... comma-separated `{...}` objects ... `]` JSON array. Reading scans for
 * balanced-brace top-level objects (GetNextItemForFile()) rather than parsing the whole file as
 * JSON, so a file can be read incrementally without holding it entirely in memory.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef DEVICE_LOG_JSON_H
#define DEVICE_LOG_JSON_H

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "DataJSON.h"
#include "DeviceLog.h"
#include "com_manager.h"

/** @brief `cDeviceLog` implementation that writes/reads a single JSON-array log file. */
class cDeviceLogJSON : public cDeviceLog
{
public:
    cDeviceLogJSON() : cDeviceLog() {};
    cDeviceLogJSON(const device_handle_t dev) : cDeviceLog(dev) {};
    cDeviceLogJSON(uint16_t hdwId, uint32_t serialNo) : cDeviceLog(hdwId, serialNo) {};

    /** @brief Close the current file, writing the closing `]` first if one is open. */
    bool CloseAllFiles() OVERRIDE;

    /** @brief Write one data set as a new JSON object to the current file, opening (with a leading `[`) or rotating (closing with `]`) files as needed. */
    bool SaveData(p_data_hdr_t* dataHdr, const uint8_t* dataBuf, protocol_type_t ptype=_PTYPE_INERTIAL_SENSE_DATA) OVERRIDE;

    /** @brief Read the next JSON object from the current (or next) file. @return the parsed data set, or null if no more objects/files are available or the current object failed to parse. */
    p_data_buf_t* ReadData() OVERRIDE;

    void SetSerialNumber(uint32_t serialNumber) OVERRIDE;
    std::string LogFileExtention() OVERRIDE { return std::string(".json"); }

private:
    /**
     * @brief Scan the current file for the next balanced-brace top-level `{...}` JSON object into `m_jsonString`.
     * @return true if a complete object was found; false on EOF or if no file is open.
     */
    bool GetNextItemForFile();

    /**
     * @brief Parse the object most recently scanned by GetNextItemForFile() (advancing to the next file/object first) into a data set.
     * @return the parsed data set, or null if no object could be scanned or it failed to parse.
     */
    p_data_buf_t* ReadDataFromFile();

    std::string m_jsonString;      //!< most recently scanned top-level JSON object, from GetNextItemForFile()
    cDataJSON m_json;              //!< JSON (de)serialization helper
    p_data_buf_t m_data;           //!< scratch buffer for the most recently read data set
};

#endif // DEVICE_LOG_CSV_H
