/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <chrono>
#include <ctime>
#include <string>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#include "ISDevice.h"
#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISConstants.h"
#include "ISDataMappings.h"
#include "ISLogFileFactory.h"
#include "util/util.h"

using namespace std;

cDeviceLog::cDeviceLog() {
    m_logStats.Clear();
}

cDeviceLog::cDeviceLog(device_handle_t dev) : device(dev)  {
    if (dev == nullptr)
        throw std::invalid_argument("cDeviceLog() must be passed a valid ISDevice instance.");
    m_devHdwId = ENCODE_DEV_INFO_TO_HDW_ID(dev->devInfo);
    m_devSerialNo = dev->devInfo.serialNumber;
    m_deviceId = dev->getIdAsString();
    m_logStats.Clear();
}

cDeviceLog::cDeviceLog(uint16_t hdwId, uint32_t serial) : m_devHdwId(hdwId), m_devSerialNo(serial) {
    m_logStats.Clear();
}

cDeviceLog::~cDeviceLog()
{
    // Close open files
    CloseISLogFile(m_pFile);
    CloseAllFiles();
}

void cDeviceLog::InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
    m_timeStamp = timestamp;
    m_directory = directory;
    m_fileCount = 0;
    m_maxDiskSpace = maxDiskSpace;
    m_maxFileSize = maxFileSize;
    m_logSize = 0;
    m_writeMode = true;
    m_logStats.Clear();
    m_indexChunks.clear();
    m_logStartUpTime = current_uptimeMs();
    // SN-8340: capture the absolute host wall-clock once, at log-open, so the
    // v2.1 .idx header carries a durable epoch anchor even for logs that never
    // acquire GPS. Per-record times stay relative (m_logStartUpTime delta).
    m_captureEpochMs = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}


void cDeviceLog::InitDeviceForReading()
{
    m_fileSize = 0;
    m_logSize = 0;
    m_fileCount = 0;
    m_writeMode = false;
    m_logStats.Clear();
}

bool cDeviceLog::CloseAllFiles()
{
    if (m_writeMode) {
        // Flush any buffered index records and finalize the .idx
        // header (rewrite at offset 0 with final totals + FINALIZED
        // flag). Order matters: chunk-write first so finalizeIndex
        // sees the right total count.
        if (!m_indexChunks.empty()) {
            writeIndexChunk();
        }
        finalizeIndex();

        string str = m_directory + "/summary_SN" + to_string(m_devSerialNo) + ".txt";
        m_logStats.WriteToFile(str);
    }
    return true;
}

/**
* Resets the log to the beginning
*/
void cDeviceLog::ResetToStart()
{
    CloseAllFiles();

    if (m_writeMode)
    {
        InitDeviceForWriting(m_timeStamp, m_directory, m_maxDiskSpace, m_maxFileSize);
    }
    else
    {
        InitDeviceForReading();
    }
    return;
}

bool cDeviceLog::OpenWithSystemApp()
{

#if PLATFORM_IS_WINDOWS

    std::wstring stemp = std::wstring(m_fileName.begin(), m_fileName.end());
    LPCWSTR filename = stemp.c_str();
    ShellExecuteW(0, 0, filename, 0, 0, SW_SHOW);

#endif

    return true;
}

bool cDeviceLog::SaveData(p_data_hdr_t *dataHdr, const uint8_t* dataBuf, protocol_type_t ptype)
{
    // Update log statistics
    if (dataHdr != NULL)
    {
        double timestamp = (ptype == _PTYPE_INERTIAL_SENSE_DATA ? cISDataMappings::TimestampOrCurrentTime(dataHdr, dataBuf) : current_timeSecD());
        m_logStats.LogData(ptype, dataHdr->id, dataHdr->size, timestamp);

        // Cache diagnostic DIDs for summary generation
        if (ptype == _PTYPE_INERTIAL_SENSE_DATA)
        {
            m_logStats.CacheDiagnosticData(dataHdr->id, dataBuf, dataHdr->size, timestamp);
        }

        // D-01 / SN-7879: pass the parsed header + buffer through so
        // the v2 index record carries the actual DID + payload-derived
        // timestamp + ToW flag.
        addIndexRecord(dataHdr, dataBuf);
        m_lastIndexOffset += dataHdr->size;
    }

    return true;
}

bool cDeviceLog::SaveData(int dataSize, const uint8_t* dataBuf, cLogStats &globalLogStats)
{
    // Streaming raw-bytes path: no parsed header is available *here*,
    // but the caller (cDeviceLogRaw::SaveData) parses individual
    // packets and emits per-packet v2 index records inside its parser
    // loop. We deliberately do NOT call addIndexRecord(no-args) here
    // — it would clobber the per-packet emission with a coarser
    // chunk-level record carrying did=0. The offset increment is also
    // deferred to the caller so per-packet records capture the
    // chunk-start offset (D-01 / SN-7879).
    (void)dataSize;
    (void)dataBuf;
    (void)globalLogStats;
    return true;
}

bool cDeviceLog::SetupReadInfo(const string& directory, const string& serialNum, const string& timeStamp)
{
    m_directory = directory;
    m_fileCount = 0;
    m_timeStamp = timeStamp;
    m_fileNames.clear();
    vector<ISFileManager::file_info_t> fileInfos;
    SetSerialNumber((uint32_t)strtoul(serialNum.c_str(), NULL, 10));

    string regExp;
    if (serialNum == "0" && timeStamp == "")
    {   // Simple filename regular expression: [\/\\][0-9]+\.dat
        regExp = string("[\\/\\\\][0-9]+\\") + LogFileExtention();
    }
    else
    {   // Default filename regular expression: [\/\\]LOG_SN60339_.*\.dat
        regExp = string("[\\/\\\\]" IS_LOG_FILE_PREFIX) + serialNum + "_.*\\" + LogFileExtention();
    }
    // Search is case insensitive, finds both upper and lower case file extensions.

    ISFileManager::GetDirectorySpaceUsed(directory, regExp, fileInfos, false, false);

    if (fileInfos.size() != 0)
    {
        m_fileName = fileInfos[0].name;
        for (size_t i = 0; i < fileInfos.size(); i++)
        {
            m_fileNames.push_back(fileInfos[i].name);
        }
    }
    return true;
}


bool cDeviceLog::OpenNewSaveFile()
{
    // Close existing file
    CloseISLogFile(m_pFile);

    // Ensure directory exists
    if (m_directory.empty())
    {
        return false;
    }

    // create directory
    _MKDIR(m_directory.c_str());

    // Open new file
    // D-01 / SN-7879: every .raw segment gets its own .idx sidecar with
    // its own header + per-segment record counts and timestamps. Reset
    // the per-segment state here so segment N+1 doesn't inherit segment
    // N's flags and counters (which would skip the new segment's header
    // write and overstate its total_records).
    m_lastIndexOffset       = 0;
    m_idxHeaderWritten      = false;
    m_idxTotalRecords       = 0;
    m_idxFirstTimestampMs   = 0;
    m_idxLastTimestampMs    = 0;
    m_fileCount++;
    uint32_t serNum = (device != nullptr ? device->devInfo.serialNumber : SerialNumber());
    if (!serNum)
        return false;

    m_fileName = GetNewBaseFileName(serNum, m_fileCount, NULL);
    m_pFile = CreateISLogFile(m_fileName + LogFileExtention(), "wb");
    m_fileSize = 0;

    if (m_pFile && m_pFile->isOpened())
    {
#if LOG_DEBUG_FILE_WRITE
        printf("cDeviceLog::OpenNewSaveFile %s\n", fileName.c_str());
#endif
        return true;
    }
    else
    {
#if LOG_DEBUG_FILE_WRITE
        printf("cDeviceLog::OpenNewSaveFile FAILED %s\n", fileName.c_str());
#endif
        return false;
    }
}


bool cDeviceLog::OpenNextReadFile()
{
    // Close file if open
    CloseISLogFile(m_pFile);

    if (m_fileCount == m_fileNames.size())
    {
        return false;
    }

    m_fileName = m_fileNames[m_fileCount++];
    m_pFile = CreateISLogFile(m_fileName, "rb");

    if (m_pFile)
    {

#if LOG_DEBUG_FILE_READ
        printf("cDeviceLog::OpenNextReadFile %s\n", m_fileName.c_str());
#endif
        return true;
    }
    else
    {
#if LOG_DEBUG_FILE_READ
        printf("cDeviceLog::OpenNextReadFile FAILED %s\n", m_fileName.c_str());
#endif
        return false;
    }
}

std::string cDeviceLog::GetNewBaseFileName(uint32_t serialNumber, uint32_t fileCount, const char* suffix)
{
    return utils::string_format("%s/%s%d_%s_%04d%s",
                                m_directory.c_str(),
                                IS_LOG_FILE_PREFIX,
                                (int)serialNumber,
                                m_timeStamp.c_str(),
                                (int)(fileCount % 10000),
                                (suffix == NULL || *suffix == 0 ? "" : (string("_") + suffix).c_str())
);
}

std::string cDeviceLog::GetNewFileName(uint32_t serialNumber, uint32_t fileCount, const char* suffix)
{
    return utils::string_format("%s%s",
        GetNewBaseFileName(serialNumber, fileCount, suffix).c_str(),
        LogFileExtention().c_str()
);
}

void cDeviceLog::UpdateStatsFromFile(p_data_buf_t *data)
{
    double timestamp = cISDataMappings::Timestamp(&data->hdr, data->buf);
    m_logStats.LogData(_PTYPE_INERTIAL_SENSE_DATA, data->hdr.id, data->hdr.size, timestamp);
}

void cDeviceLog::UpdateStatsFromFile(protocol_type_t ptype, int id, double timestamp)
{
    m_logStats.LogData(ptype, id, 0, timestamp);    // FIXME!! is bytes really 0? Maybe its the size of the ID struct?
}

device_handle_t cDeviceLog::Device() {
    return device;
}

dev_info_t cDeviceLog::DeviceInfo() {
    return device->devInfo;
}

void cDeviceLog::OnReadPacket(packet_t* pkt, protocol_type_t ptype) {
    if (pkt != NULL)
    {
        double timestamp = cISDataMappings::Timestamp(&pkt->dataHdr, pkt->data.ptr);
        m_logStats.LogData(ptype, pkt->dataHdr.id, pkt->dataHdr.size, timestamp);
    }
}

void cDeviceLog::OnReadData(p_data_buf_t* data)
{
    if (data != NULL)
    {
        double timestamp = cISDataMappings::Timestamp(&data->hdr, data->buf);
        m_logStats.LogData(_PTYPE_INERTIAL_SENSE_DATA, data->hdr.id, data->hdr.size, timestamp);
    }
}

// Pack the SDK's PROTOCOL_VERSION_CHAR0..3 quadruple into a single u32
// for the .idx header's `producer_version` field. Layout chosen so a
// hex dump reads "02 01 00 00" → SDK 2.1.0.0 left-to-right.
static inline uint32_t encode_sdk_producer_version() noexcept {
    return  (static_cast<uint32_t>(PROTOCOL_VERSION_CHAR0) << 24)
          | (static_cast<uint32_t>(PROTOCOL_VERSION_CHAR1) << 16)
          | (static_cast<uint32_t>(PROTOCOL_VERSION_CHAR2) <<  8)
          | (static_cast<uint32_t>(PROTOCOL_VERSION_CHAR3));
}

void cDeviceLog::addIndexRecord(const p_data_hdr_t* dataHdr, const uint8_t* dataBuf) {
    using namespace inertial_sense::idx;

    is_log_idx_record_v2_t rec{};
    rec.offset = m_lastIndexOffset;
    rec.reserved = 0;

    // SN-8383 (v2.1): stamp the host-uptime-since-log-start for EVERY record —
    // the per-record local clock v1 had and the v2 transition dropped. This is
    // independent of `timestamp` below (which still prefers the payload ToW),
    // so a downstream resolver can bracket a timeless record between the local
    // deltas of its timed neighbours instead of guessing by arrival index.
    const uint32_t localDelta = static_cast<uint32_t>(current_uptimeMs() - m_logStartUpTime);
    rec.local_uptime_ms = localDelta;
    rec.reserved2       = 0;

    if (dataHdr != nullptr) {
        rec.did = dataHdr->id;
        // cISDataMappings::Timestamp returns 0.0 if the DID doesn't
        // carry one — that's the signal to fall back to the host
        // uptime delta and clear the ToW flag.
        const double tsSeconds = cISDataMappings::Timestamp(dataHdr, dataBuf);
        if (tsSeconds > 0.0) {
            rec.timestamp = static_cast<uint64_t>(tsSeconds * 1000.0);
            rec.flags = IS_LOG_IDX_REC_FLAG_HAS_TOW;
        } else {
            rec.timestamp = localDelta;
            rec.flags = 0;
        }
    } else {
        // Streaming-only path: no DID, no payload timestamp. Fall back
        // to host uptime delta so the record still anchors a position
        // in the .raw segment by approximate time.
        rec.did = 0;
        rec.timestamp = localDelta;
        rec.flags = 0;
    }

    // Track first/last for the header rewrite at finalize time.
    if (m_idxTotalRecords == 0 && m_indexChunks.empty()) {
        m_idxFirstTimestampMs = rec.timestamp;
    }
    m_idxLastTimestampMs = rec.timestamp;

    m_indexChunks.push_back(rec);
    m_lastIndexTime = current_uptimeMs();
}

bool cDeviceLog::writeIndexChunk() {
    using namespace inertial_sense::idx;

    if (m_indexChunks.empty() && m_idxHeaderWritten) {
        return true; // nothing to do
    }

    // SN-8328: never emit an index for a segment that has no file name yet.
    // Writing "m_fileName + .idx" with an empty m_fileName would create an
    // orphan "./.idx" in the working directory (unassociated with any .raw).
    // Callers must open the segment file (OpenNewSaveFile) before indexing.
    if (m_fileName.empty()) {
        return false;
    }

    const std::string fileName = m_fileName + ".idx";
    cISLogFile indexFile(fileName, "ab");
    if (!indexFile.isOpened()) {
        return false;
    }

    // First chunk write: emit the v2 header. total_records /
    // first/last timestamps stay zero here — finalizeIndex() seeks(0)
    // and rewrites the header on close. ts_units = HostUptimeMs is
    // the conservative default; per-record `flags` bit 0 still tells
    // readers when a specific record actually carried a real ToW.
    if (!m_idxHeaderWritten) {
        is_log_idx_header_t hdr = makeDefaultHeader(
            encode_sdk_producer_version(),
            TimestampUnits::HostUptimeMs,
            HeaderTimeSource::Mixed,
            m_captureEpochMs);
        hdr.flags |= IS_LOG_IDX_HDR_FLAG_HAS_LOCAL_DELTA;   // SN-8383: every record carries local_uptime_ms
        auto r = writeHeader(indexFile, hdr);
        if (!r) {
            return false;
        }
        m_idxHeaderWritten = true;
    }

    for (const auto& rec : m_indexChunks) {
        auto r = writeRecord(indexFile, rec);
        if (!r) {
            // writing error; whole file should be considered bad.
            return false;
        }
        ++m_idxTotalRecords;
    }
    m_indexChunks.clear();
    indexFile.close();  // unnecessary since this is destroyed on exit
    return true;
}

bool cDeviceLog::finalizeIndex() {
    using namespace inertial_sense::idx;

    // Idempotent: if no header was ever written (no records emitted)
    // there's nothing to finalize.
    if (!m_idxHeaderWritten) {
        return true;
    }

    const std::string fileName = m_fileName + ".idx";
    // Open r+b so we can seek to offset 0 and rewrite the header.
    cISLogFile indexFile(fileName, "rb+");
    if (!indexFile.isOpened()) {
        return false;
    }

    if (indexFile.seek(0, SEEK_SET) != 0) {
        return false;
    }

    is_log_idx_header_t hdr = makeDefaultHeader(
        encode_sdk_producer_version(),
        TimestampUnits::HostUptimeMs,
        HeaderTimeSource::Mixed,
        m_captureEpochMs);
    hdr.total_records       = m_idxTotalRecords;
    hdr.first_timestamp_ms  = m_idxFirstTimestampMs;
    hdr.last_timestamp_ms   = m_idxLastTimestampMs;
    // Preserve the v2.1 flags across the finalize header rewrite (the plain
    // "= FINALIZED" would otherwise drop HAS_LOCAL_DELTA / HAS_CAPTURE_EPOCH).
    hdr.flags               = static_cast<uint8_t>(
                                  IS_LOG_IDX_HDR_FLAG_FINALIZED
                                | IS_LOG_IDX_HDR_FLAG_HAS_LOCAL_DELTA
                                | (m_captureEpochMs ? IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH : 0));

    auto r = writeHeader(indexFile, hdr);
    if (!r) {
        return false;
    }
    return true;
}
