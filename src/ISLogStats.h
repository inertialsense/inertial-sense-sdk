/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file ISLogStats.h
 * @brief Per-DID receive statistics tracking (count, error count, inter-arrival timing, bitrate)
 *        and a diagnostic-DID cache used to render a device-health summary alongside a log's
 *        stats file.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_LOG_STATS_H
#define IS_LOG_STATS_H

#include <string>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "data_sets.h"
#include "ISComm.h"


typedef void (*FuncLogDataAndTimestamp)(uint32_t dataId, double timeMs);   //!< callback signature for logging a data ID with an associated timestamp (ms)

/** Receive statistics tracked for a single message/data ID: counts, error counts, inter-arrival timing, and bitrate. */
class cLogStatMsgId
{
public:
    unsigned int count = 0;         //!< count for this data id
    unsigned int errors = 0;        //!< error count for this data id
    unsigned int meanDtMs = 0;      //!< average time delta for the data id
    unsigned int accumDtMs = 0;     //!< sum of all time deltas
    unsigned int lastTimeMs = 0;    //!< timestamp (ms) of the most recent sample, used to compute the next dt
    unsigned int lastDtMs = 0;      //!< the most recently computed time delta (ms)
    unsigned int minDtMs = 0;       //!< the smallest time delta (ms) observed
    unsigned int maxDtMs = 0;       //!< the largest time delta (ms) observed
    unsigned int dtMsCount = 0;     //!< number of time deltas accumulated into meanDtMs/accumDtMs (periodically halved to bound the running average's window)
    unsigned int timeIrregCount = 0; //!< count of irregularities in delta timestamps (> 50% different from previous delta timestamp)

    unsigned int bpsBytes = 0;         //!< bytes accumulated in the current bitrate measurement window
    unsigned int bpsStartTimeMs = 0;   //!< timestamp (ms) at the start of the current bitrate measurement window
    unsigned int bytesPerSec = 0;      //!< most recently computed bytes-per-second rate for this data id

    /** Constructs a stat with counters zeroed (minDtMs seeded high so the first sample always sets it). */
    cLogStatMsgId();

    /**
     * Records a new sample timestamp, updating dt/min/max/mean and timeIrregCount.
     * @param timeMs the timestamp (ms) of the new sample; a value of 0 is ignored
     */
    void LogTimestamp(unsigned int timeMs);

    /**
     * Records bytes received for this data id, updating bytesPerSec once the current 1-second window has elapsed.
     * @param timeMs the timestamp (ms) of the new sample
     * @param bytes the number of bytes received in this sample
     */
    void LogByteSize(unsigned int timeMs, int bytes);
};

/** Aggregate receive statistics for all data ids belonging to a single protocol type (see protocol_type_t). */
struct sLogStatPType
{
    std::map<int, cLogStatMsgId> stats;     //!< per-data-id statistics, keyed by data id
    unsigned int count;                     //!< count of all message ids
    unsigned int errors;                    //!< total error count
};

/** Cached copy of the most recently observed data for one diagnostic DID, used to build DiagnosticSummary(). */
struct DiagnosticDIDCache {
    std::vector<uint8_t> data;      //!< buffer holding the most recently observed (possibly partially merged) copy of the data set
    double timestamp = 0.0;         //!< timestamp of the most recent update to data
    bool observed = false;          //!< true once at least one update has been received
};

/** Aggregates per-protocol/per-DID receive statistics and a diagnostic-DID cache, and renders both to a stats summary. */
class cLogStats
{
public:
    std::map<protocol_type_t, sLogStatPType> msgs;     //!< per-protocol-type statistics, keyed by protocol type
    cISLogFileBase* statsFile;                          //!< log file the stats summary is written to by WriteToFile()

    /** Constructs the stats tracker with all counters cleared. */
    cLogStats();

    /** Resets all per-protocol/per-DID statistics to their initial state. */
    void Clear();

    /**
     * Records a parse/receive error for a given protocol type and (if known) data id.
     * @param hdr the packet header identifying the data id in error, or nullptr if unknown
     * @param ptype the protocol type the error occurred on
     */
    void LogError(const p_data_hdr_t* hdr, protocol_type_t ptype=_PTYPE_INERTIAL_SENSE_DATA);

    /**
     * Records a successfully received message, updating its protocol-type and per-id statistics.
     * @param ptype the protocol type of the received message
     * @param id the data id of the received message
     * @param bytes the size, in bytes, of the received message
     * @param timeMs the timestamp, in seconds, of the received message (converted to ms internally); 0 disables timing/bitrate updates for this sample
     */
    void LogData(protocol_type_t ptype, int id, int bytes, double timeMs=0.0);

    /**
     * Merges a (possibly partial) update into the cached copy of a diagnostic DID's data, for later inclusion in DiagnosticSummary().
     * Data ids not in the diagnostic set (see s_diagnosticDIDs) are ignored.
     * @param did the data id being cached
     * @param data pointer to the received bytes to merge in
     * @param size the number of bytes pointed to by data
     * @param timestamp the timestamp of this update
     * @param offset byte offset within the full data set where data should be merged (0 for a full update)
     */
    void CacheDiagnosticData(uint32_t did, const uint8_t* data, uint32_t size, double timestamp, uint32_t offset=0);

    /** @return the total count of all messages received across all protocol types */
    unsigned int Count();

    /** @return the total count of all errors recorded across all protocol types */
    unsigned int Errors();

    /**
     * @param ptype the protocol type msg belongs to, used to select id-to-name formatting
     * @param msg the per-protocol-type statistics to format
     * @param showDeltaTime if true, includes the per-id inter-arrival timing/bitrate columns
     * @param showErrors if true, includes the per-id error-count column
     * @return a formatted, human-readable table of msg's per-id statistics
     */
    std::string MessageStats(protocol_type_t ptype, sLogStatPType &msg, bool showDeltaTime=true, bool showErrors=false);

    /** @return a formatted, human-readable summary of statistics for all tracked protocol types */
    std::string Stats();

    /** @return a formatted, human-readable device-health summary derived from the cached diagnostic DID data (device info, non-default flash config, BIT/status fields) */
    std::string DiagnosticSummary();

    /**
     * Writes the diagnostic summary (if any) and full stats summary to the given file.
     * @param fileName path of the file to write
     */
    void WriteToFile(const std::string& fileName);

private:
    std::map<uint32_t, DiagnosticDIDCache> m_diagCache;
    static const std::set<uint32_t> s_diagnosticDIDs;

    std::string FormatDevInfoSection(uint32_t did, const char* label);
    std::string FormatFlashConfigDiffSection(uint32_t did, const char* label);
    std::string FormatBitSection(uint32_t did, const char* label, const std::string& fields);
    std::string FormatCuratedSection(uint32_t did, const char* label, const std::string& fields);
    std::string FormatTimestamp(double timestamp);
};



#endif // IS_LOG_STATS_H
