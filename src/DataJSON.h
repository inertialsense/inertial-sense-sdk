/**
 * @file DataJSON.h
 * @brief JSON (de)serialization for a single data set (DID), used by DeviceLogJSON.
 *
 * Each data set is one JSON object: `{"id":<did>,"<field1>":<value1>,...}`. Field names and
 * layout come from `cISDataMappings`, keyed by DID. Parsing (StringJSONToData()) uses a simple
 * bracket/quote scanner rather than a general JSON parser — it expects an already-isolated,
 * single top-level object (as produced by DeviceLogJSON's balanced-brace file scanner), not
 * arbitrary JSON.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef DATA_JSON_H
#define DATA_JSON_H

#include <string>
#include <map>
#include <regex>

#include "com_manager.h"
#include "ISLogFileBase.h"

/** @brief Check whether @p c is a character that must be escaped inside a JSON string value. */
static inline bool IS_JSON_ESCAPE_CHAR(char c)
{
    switch (c)
    {
    case '"':
    case '\\':
    case '/':
    case '\b':
    case '\f':
    case '\n':
    case '\r':
    case '\t':
        return true;
    }
    return false;
}

/** @brief Stateless JSON (de)serialization helpers for one data set (DID) at a time. */
class cDataJSON
{
public:
    /**
     * @brief Write one data set as a JSON object (`{"id":...,"field":value,...}`) to @p pFile.
     * @param pFile file to write to.
     * @param dataHdr header describing @p dataBuf's data ID, size, and offset.
     * @param dataBuf the data payload described by @p dataHdr.
     * @param prefix if non-null, written immediately before the JSON object (e.g. `",\n"` to separate array elements); not included in the returned byte count.
     * @return the number of bytes of the JSON object written (excluding @p prefix), or 0 if @p pFile is null or @p dataHdr.id has no known field mapping.
     */
    int WriteDataToFile(cISLogFileBase* pFile, const p_data_hdr_t& dataHdr, const uint8_t* dataBuf, const char* prefix);

    /**
     * @brief Parse a single JSON object string into a data set.
     *
     * Expects @p s to be exactly one top-level `{...}` object with an `"id"` field appearing
     * first; fields that don't map to a known field for that DID are silently skipped rather than
     * treated as an error.
     *
     * @param s the JSON object to parse.
     * @param[out] hdr `id` is set from the object's `"id"` field; on success, `size` is set to the DID's full struct size.
     * @param buf destination buffer to fill; must be at least the DID's full struct size.
     * @param bufSize size of @p buf, in bytes (currently unused — the caller is responsible for @p buf being large enough).
     * @return true if an `"id"` field was found, it mapped to a known DID, and every recognized field parsed successfully; false otherwise.
     */
    bool StringJSONToData(std::string& s, p_data_hdr_t& hdr, uint8_t* buf, uint32_t bufSize);

    /**
     * @brief Convert one data set to a JSON object string (`{"id":...,"field":value,...}`, no trailing newline).
     * @param hdr header describing @p buf's data ID, size, and offset; a partial buffer (offset/size narrower than the DID's full struct) is zero-padded around its actual bytes before conversion.
     * @param buf the data payload described by @p hdr.
     * @param[out] json the resulting JSON object string; cleared and repopulated by this call.
     * @return true on success; false if @p hdr.id has no known field mapping.
     */
    bool DataToStringJSON(const p_data_hdr_t& hdr, const uint8_t* buf, std::string& json);
};

#endif // DATA_JSON_H
