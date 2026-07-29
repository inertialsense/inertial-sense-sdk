/**
 * @file DataCSV.h
 * @brief CSV (de)serialization for a single data set (DID), used by DeviceLogCSV.
 *
 * Each row is `<orderId>,<field1>,<field2>,...` where the order id (see cCsvLog::orderId) lets
 * DeviceLogCSV::ReadData() merge multiple per-DID files back into their original write order. The
 * header row is `_ID_,<name1>,<name2>,...`, with array fields expanded into `<name>[<i>]` columns.
 * Field names and layout come from `cISDataMappings`, keyed by DID.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef DATA_CVS_H
#define DATA_CVS_H

#include <string>
#include <map>
#include <regex>

#include "com_manager.h"
#include "ISDataMappings.h"

/** @brief Stateless CSV (de)serialization helpers for one data set (DID) at a time. */
class cDataCSV
{
public:
    /**
     * @brief Write the CSV header row (`_ID_,<field names>`) for @p id's data set.
     * @param pFile file to write to.
     * @param id data ID whose field layout (from `cISDataMappings`) to write column names for.
     * @return the number of bytes written, or 0 if @p pFile is null, @p id is out of range, or @p id has no known field mapping.
     */
    int WriteHeaderToFile(FILE* pFile, uint32_t id);

    /**
     * @brief Read and parse a CSV header row into column descriptors.
     * @param pFile file to read the header row from.
     * @param id data ID whose field mapping (from `cISDataMappings`) resolves each column name; unresolved columns are recorded as `DATA_TYPE_BINARY` ("UNKNOWN" if the column name itself was empty).
     * @param[out] columnHeaders per-column field descriptors, in file order; cleared and repopulated by this call.
     * @return the number of bytes read, or 0 on a read error or end of file (always 0 on embedded platforms, which don't support header parsing).
     */
    int ReadHeaderFromFile(FILE* pFile, uint32_t id, std::vector<data_info_t>& columnHeaders);

    /**
     * @brief Write one data set as a new CSV row (`<orderId>,<fields>`).
     * @param orderId value to write into column 0, used later to restore write order across data sets; see cCsvLog::orderId.
     * @param pFile file to write to.
     * @param dataHdr header describing @p dataBuf's data ID, size, and offset.
     * @param dataBuf the data payload described by @p dataHdr.
     * @return the number of bytes written, or 0 if @p pFile is null, @p dataHdr.id has no known field mapping, or the data set has zero size.
     */
    int WriteDataToFile(uint64_t orderId, FILE* pFile, const p_data_hdr_t& dataHdr, const uint8_t* dataBuf);

    /**
     * @brief Parse one CSV row (with no leading order id) into a data set, using a previously-parsed header's column layout.
     * @param s the CSV row to parse (mutated: trailing line terminators are stripped and a trailing comma is appended internally).
     * @param hdr on input, `id` must already be set to the target data ID; on success, `offset` is set to 0 and `size` to the DID's full struct size.
     * @param buf destination buffer to fill; zeroed before parsing, and must be at least @p hdr.size (the DID's full struct size) bytes.
     * @param bufSize size of @p buf, in bytes.
     * @param columnHeaders column layout from ReadHeaderFromFile(), matching @p s's columns positionally.
     * @return true if every field parsed successfully; false if @p hdr.id has no known field mapping, @p hdr.offset/size/bufSize are inconsistent, or any field failed to parse.
     */
    bool StringCSVToData(std::string& s, p_data_hdr_t& hdr, uint8_t* buf, uint32_t bufSize, const std::vector<data_info_t>& columnHeaders);

    /**
     * @brief Convert one data set to a CSV row string (comma-prefixed fields, no order id, no trailing newline stripped).
     * @param hdr header describing @p buf's data ID, size, and offset; a partial buffer (offset/size narrower than the DID's full struct) is zero-padded around its actual bytes before conversion.
     * @param buf the data payload described by @p hdr.
     * @param[out] csv the resulting CSV row, including a trailing newline; cleared and repopulated by this call.
     * @return true on success; false if @p hdr.id has no known field mapping.
     */
    bool DataToStringCSV(const p_data_hdr_t& hdr, const uint8_t* buf, std::string& csv);
};

#endif // DATA_CVS_H
