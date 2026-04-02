/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>

#include "ISDataMappings.h"
#include "ISLogFile.h"
#include "ISLogFileFactory.h"
#include "ISLogStats.h"
#include "ISUtilities.h"
#include "protocol_nmea.h"
#include "util/util.h"

using namespace std;


cLogStatMsgId::cLogStatMsgId()
{
    count = 0;
    errors = 0;
    meanDtMs = 0;
    accumDtMs = 0;
    lastTimeMs = 0;
    lastDtMs = 0;
    maxDtMs = 0;
    minDtMs = 10000000;
    dtMsCount = 0;
    timeIrregCount = 0;
}

void cLogStatMsgId::LogTimestamp(unsigned int timeMs)
{
    if (timeMs == 0)
        return;

    if (lastTimeMs)
    {
        unsigned int dtMs = timeMs - lastTimeMs;
        minDtMs = _MIN(dtMs, minDtMs);
        maxDtMs = _MAX(dtMs, maxDtMs);
        accumDtMs += dtMs;
        meanDtMs = accumDtMs / (++dtMsCount);
        if (dtMsCount > 20)
        {
            dtMsCount /= 2;
            accumDtMs /= 2;
        }
        if (lastDtMs != 0.0) 
        {
            unsigned int ddtMs = dtMs - lastDtMs;
            if (ddtMs > (lastDtMs/2))
            {
                timeIrregCount++;
            }
        }
        lastDtMs = dtMs;
    }
    lastTimeMs = timeMs;
}

void cLogStatMsgId::LogByteSize(unsigned int timeMs, int bytes)
{
    if (bpsStartTimeMs == 0)
    {   // Initialize
        bpsBytes = bytes;
        bpsStartTimeMs = timeMs;
    }
    else
    {
        bpsBytes += bytes;
        unsigned int dtMs = timeMs - bpsStartTimeMs;
        if (dtMs >= 1000)
        {
            bytesPerSec = 1000 * bpsBytes / dtMs;
            bpsBytes = 0;
            bpsStartTimeMs = timeMs;
        }
    }
}

cLogStats::cLogStats()
{
    Clear();
}

void cLogStats::Clear()
{
    for (auto& [ptype, msg] : msgs)
    {
        msg.count = 0;
        msg.errors = 0;
        for (uint32_t id=0; id < DID_COUNT; id++)
        {
            msg.stats[id] = {};
            msg.stats[id].minDtMs = 1.0E6;
        }
    }
}

void cLogStats::LogError(const p_data_hdr_t* hdr, protocol_type_t ptype)
{
    sLogStatPType &msg = msgs[ptype];

    msg.errors++;
    if (hdr != NULL && hdr->id < DID_COUNT)
    {
        cLogStatMsgId& d = msg.stats[hdr->id];
        d.errors++;
    }
}

void cLogStats::LogData(protocol_type_t ptype, int id, int bytes, double timestamp)
{
    sLogStatPType &msg = msgs[ptype];
    cLogStatMsgId &d = msg.stats[id];
    msg.count++;
    d.count++;

    unsigned int timeMs = (unsigned int)(timestamp*1000.0);
    if (timeMs != 0.0)
    {   // Use system time
        d.LogTimestamp(timeMs);
        d.LogByteSize(timeMs, bytes);
    }
}

string cLogStats::MessageStats(protocol_type_t ptype, sLogStatPType &msg, bool showDeltaTime, bool showErrors)
{
    string msgName;
    int colWidName = 24;
    switch (ptype)
    {
    default:                                msgName = "Ptype " + std::to_string(ptype);  break;
    case _PTYPE_PARSE_ERROR:                msgName = "Parse Error";    break;
    case _PTYPE_INERTIAL_SENSE_CMD:         msgName = "ISB-CMD";        break;
    case _PTYPE_INERTIAL_SENSE_DATA:        msgName = "ISB";            break;
    case _PTYPE_NMEA:                       msgName = "NMEA";           colWidName = 8;  break;
    case _PTYPE_UBLOX:                      msgName = "UBLOX";          colWidName = 8;  break;
    case _PTYPE_RTCM3:                      msgName = "RTCM3";          break;
    case _PTYPE_SPARTN:                     msgName = "SPARTN";         break;
    case _PTYPE_SONY:                       msgName = "SONY";           break;
    }

    std::stringstream ss;
    ss << msgName << ": count " << msg.count << ", errors " << msg.errors << " _____________________" << endl;

    ss << " ID " << std::setw(colWidName) << std::left << "Name" << std::right << " Count";
    if (showErrors)     { ss << " Errors"; }
    if (showDeltaTime)  { ss << "  dtMs(avg  min  max)   Bps Irreg"; }
    ss << endl;

    for (std::map<int, cLogStatMsgId>::iterator it = msg.stats.begin(); it != msg.stats.end(); ++it)
    {
        int id = it->first;
        cLogStatMsgId& stat = it->second;

        if (stat.count == 0 && stat.errors == 0)
        {   // Exclude zero count stats
            continue;
        }

        // Don't print using stringstream as it causes the system to hang on the EVB.
        switch (ptype)
        {
        case _PTYPE_INERTIAL_SENSE_CMD:
        case _PTYPE_INERTIAL_SENSE_DATA:
            ss << std::setw(3) << std::right << id << " " << std::setw(colWidName) << std::left << cISDataMappings::DataName(id) << std::right;
            break;

        case _PTYPE_NMEA: {
                char talker[10] = {0};
                nmeaMsgIdToTalker(id, talker, sizeof(talker));
                ss << std::setw(3) << ::right << id << " " << std::setw(colWidName) << std::left << talker << std::right;
            }
            break;

        case _PTYPE_UBLOX: {
                uint8_t msgClass = 0xFF & (id>>0);
                uint8_t msgID    = 0xFF & (id>>8);
                ss << "Class ID (0x" << std::hex << msgClass << "0x" << std::hex << msgID << ")" << std::hex << endl;
            }
            break;

        default:
            ss << std::setw(3) << id;
            break;
        }

        ss << std::setw(6) << std::setfill(' ') << stat.count;
        if (showErrors)     { ss << std::setw(7) << stat.errors; }
        if (showDeltaTime)
        {
#define DT_COL_WIDTH    5
            if (stat.dtMsCount)
            {
                ss << "  (" 
                   << std::setw(DT_COL_WIDTH) << stat.meanDtMs << " " 
                   << std::setw(DT_COL_WIDTH) << stat.minDtMs << " " 
                   << std::setw(DT_COL_WIDTH) << stat.maxDtMs << ") " 
                   << std::setw(DT_COL_WIDTH) << stat.bytesPerSec << " " 
                   << std::setw(5) << stat.timeIrregCount;
            }
        }
        ss << endl;
    }

    return ss.str();
}

unsigned int cLogStats::Count()
{
    unsigned int count = 0;
    for (auto [ptype, stats] : msgs)
    {
        count += stats.count;
    }
    return count;
}

unsigned int cLogStats::Errors()
{
    unsigned int errors = 0;
    for (auto [ptype, stats] : msgs)
    {
        errors += stats.errors;
    }    
    return errors;
}

string cLogStats::Stats()
{
    std::stringstream ss;
    unsigned int count = Count();
    ss << "Total: count " << count << endl;
    for (auto [ptype, stats] : msgs)
    {
        switch (ptype)
        {   // Skip these
        case _PTYPE_PARSE_ERROR:
        case _PTYPE_INERTIAL_SENSE_CMD:
        case _PTYPE_INERTIAL_SENSE_ACK: continue;
        default: break;
        }

        ss << MessageStats(ptype, stats);
    }
    return ss.str();
}

void cLogStats::WriteToFile(const string& file_name)
{
    unsigned int count = Count();
    if (count != 0)
    {   // Write log summary + stats to disk
        statsFile = CreateISLogFile(file_name, "wb");
        string diagSummary = DiagnosticSummary();
        if (!diagSummary.empty())
        {
            statsFile->lprintf("%s\n", diagSummary.c_str());
        }
        statsFile->lprintf("%s", Stats().c_str());
        CloseISLogFile(statsFile);
    }
}


// ============================================================================
// Diagnostic DID caching and summary generation
// ============================================================================

const std::set<uint32_t> cLogStats::s_diagnosticDIDs = {
    DID_DEV_INFO,           // 1
    DID_SYS_PARAMS,         // 10
    DID_FLASH_CONFIG,       // 12
    DID_BIT,                // 64
    DID_GPX_DEV_INFO,       // 120
    DID_GPX_FLASH_CFG,      // 121
    DID_GPX_STATUS,         // 123
    DID_GPX_BIT,            // 125
};

void cLogStats::CacheDiagnosticData(uint32_t did, const uint8_t* data, uint32_t size, double timestamp, uint32_t offset)
{
    if (s_diagnosticDIDs.find(did) == s_diagnosticDIDs.end())
        return;

    DiagnosticDIDCache& cache = m_diagCache[did];

    // Get the full struct size from data mappings
    uint32_t fullSize = cISDataMappings::DataSize(did);
    if (fullSize == 0)
        fullSize = offset + size;

    // Initialize buffer to zero on first use or if it's too small
    if (cache.data.size() < fullSize)
        cache.data.resize(fullSize, 0);

    // Merge partial update at the correct offset
    uint32_t copySize = _MIN(size, fullSize - offset);
    memcpy(cache.data.data() + offset, data, copySize);

    cache.timestamp = timestamp;
    cache.observed = true;
}

string cLogStats::FormatTimestamp(double timestamp)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << timestamp << "s";
    return ss.str();
}

string cLogStats::FormatDevInfoSection(uint32_t did, const char* label)
{
    auto it = m_diagCache.find(did);
    if (it == m_diagCache.end() || !it->second.observed)
        return "";

    const DiagnosticDIDCache& cache = it->second;
    if (cache.data.size() < sizeof(dev_info_t))
        return "";

    const dev_info_t* devInfo = reinterpret_cast<const dev_info_t*>(cache.data.data());

    std::stringstream ss;
    ss << "=== Device Info (" << label << ") ===" << endl;
    ss << utils::devInfoToString(*devInfo) << endl;
    ss << "(last received: " << FormatTimestamp(cache.timestamp) << ")" << endl;
    ss << endl;
    return ss.str();
}

static void flashCfgDefaultsIMX(nvm_flash_cfg_t* fc)
{
    memset(fc, 0, sizeof(nvm_flash_cfg_t));
    fc->size                    = sizeof(nvm_flash_cfg_t);
    fc->key                     = 36;
    fc->startupImuDtMs          = 1;
    fc->startupGPSDtMs          = 200;
    fc->startupNavDtMs          = 4;
    fc->ser0BaudRate            = IS_BAUDRATE_921600;
    fc->ser1BaudRate            = IS_BAUDRATE_921600;
    fc->ser2BaudRate            = IS_BAUDRATE_921600;
    fc->lastLlaUpdateDistance   = 1000.0f;
    fc->ioConfig                = IO_CONFIG_DEFAULT;
    fc->platformConfig          = PLATFORM_CFG_TYPE_NONE;
    fc->gpsTimeSyncPeriodMs     = 1000;
    fc->dynamicModel            = DEFAULT_DYNAMIC_MODEL;
    fc->gnssSatSigConst         = GNSS_SAT_SIG_CONST_DEFAULT;
    fc->sensorConfig            = (SENSOR_CFG_GYR_FS_MAX<<SENSOR_CFG_GYR_FS_OFFSET) |
                                  (SENSOR_CFG_ACC_FS_MAX<<SENSOR_CFG_ACC_FS_OFFSET);
    fc->gpsMinimumElevation     = DEFAULT_GNSS_MIN_ELEVATION_ANGLE;
    fc->gnssCn0Minimum          = DEFAULT_GNSS_RTK_CN0_MINIMUM;
    fc->gnssCn0DynMinOffset     = DEFAULT_GNSS_RTK_CN0_DYN_MIN_OFFSET;
    fc->magInterferenceThreshold = 3.0f;
    fc->magCalibrationQualityThreshold = 10.0f;
    fc->imuRejectThreshGyroLow  = 2;
    fc->imuRejectThreshGyroHigh = 3;
}

static void flashCfgDefaultsGPX(gpx_flash_cfg_t* fc)
{
    memset(fc, 0, sizeof(gpx_flash_cfg_t));
    fc->size                    = sizeof(gpx_flash_cfg_t);
    fc->key                     = 2;
    fc->dynamicModel            = DEFAULT_DYNAMIC_MODEL;
    fc->gpsTimeSyncPeriodMs     = 1000;         // GPX_MINIMUM_SYNC_RATE_MS
    fc->startupGPSDtMs          = 200;          // GPX_DEFAULT_GPS_DT_MS
    fc->gnssSatSigConst         = GNSS_SAT_SIG_CONST_DEFAULT;
    fc->ser0BaudRate            = IS_BAUDRATE_921600;
    fc->ser1BaudRate            = IS_BAUDRATE_921600;
    fc->ser2BaudRate            = IS_BAUDRATE_921600;
    fc->gpsMinimumElevation     = DEFAULT_GNSS_MIN_ELEVATION_ANGLE;
    fc->gnssCn0Minimum          = DEFAULT_GNSS_RTK_CN0_MINIMUM;
    fc->gnssCn0DynMinOffset     = DEFAULT_GNSS_RTK_CN0_DYN_MIN_OFFSET;
}

string cLogStats::FormatFlashConfigDiffSection(uint32_t did, const char* label)
{
    auto it = m_diagCache.find(did);
    if (it == m_diagCache.end() || !it->second.observed)
        return "";

    const DiagnosticDIDCache& cache = it->second;

    // Generate defaults for comparison
    std::vector<uint8_t> defaultBuf;
    if (did == DID_FLASH_CONFIG)
    {
        defaultBuf.resize(sizeof(nvm_flash_cfg_t), 0);
        flashCfgDefaultsIMX(reinterpret_cast<nvm_flash_cfg_t*>(defaultBuf.data()));
    }
    else if (did == DID_GPX_FLASH_CFG)
    {
        defaultBuf.resize(sizeof(gpx_flash_cfg_t), 0);
        flashCfgDefaultsGPX(reinterpret_cast<gpx_flash_cfg_t*>(defaultBuf.data()));
    }
    else
    {
        return "";
    }

    const map_name_to_info_t* infoMap = cISDataMappings::NameToInfoMap(did);
    if (!infoMap)
        return "";

    data_mapping_string_t stringBuffer;
    std::stringstream diffFields;
    int diffCount = 0;

    for (const auto& entry : *infoMap)
    {
        const data_info_t& info = entry.second;

        // Skip metadata fields
        if (info.name == "size" || info.name == "checksum" || info.name == "key")
            continue;

        uint32_t offset = info.offset;
        uint32_t fieldSize = info.size;

        if (info.arraySize > 0)
        {
            for (int i = 0; i < (int)info.arraySize; i++)
            {
                uint32_t elemOffset = offset + i * fieldSize;
                if (elemOffset + fieldSize > cache.data.size() || elemOffset + fieldSize > defaultBuf.size())
                    continue;

                if (memcmp(cache.data.data() + elemOffset, defaultBuf.data() + elemOffset, fieldSize) != 0)
                {
                    if (cISDataMappings::DataToString(info, nullptr, cache.data.data(), stringBuffer, i))
                    {
                        diffFields << info.name << "[" << i << "] = " << stringBuffer << endl;
                        diffCount++;
                    }
                }
            }
        }
        else
        {
            if (offset + fieldSize > cache.data.size() || offset + fieldSize > defaultBuf.size())
                continue;

            if (memcmp(cache.data.data() + offset, defaultBuf.data() + offset, fieldSize) != 0)
            {
                if (cISDataMappings::DataToString(info, nullptr, cache.data.data(), stringBuffer))
                {
                    diffFields << info.name << " = " << stringBuffer << endl;
                    diffCount++;
                }
            }
        }
    }

    if (diffCount == 0)
        return "";

    std::stringstream ss;
    ss << "=== " << label << " Flash Config (non-default) ===" << endl;
    ss << "(last received: " << FormatTimestamp(cache.timestamp) << ")" << endl;
    ss << diffFields.str();
    ss << endl;
    return ss.str();
}

string cLogStats::FormatCuratedSection(uint32_t did, const char* label, const std::string& fields)
{
    auto it = m_diagCache.find(did);
    if (it == m_diagCache.end() || !it->second.observed)
        return "";

    const DiagnosticDIDCache& cache = it->second;

    std::string output;
    if (!cISDataMappings::DidBufferToString(did, cache.data.data(), output, fields))
        return "";

    std::stringstream ss;
    ss << "=== " << label << " ===" << endl;
    ss << "(last received: " << FormatTimestamp(cache.timestamp) << ")" << endl;
    ss << output;
    ss << endl;
    return ss.str();
}

string cLogStats::DiagnosticSummary()
{
    std::stringstream ss;

    // Device Info sections (at top)
    ss << FormatDevInfoSection(DID_DEV_INFO, "IMX");
    ss << FormatDevInfoSection(DID_GPX_DEV_INFO, "GPX");

    // Flash Config diff sections
    ss << FormatFlashConfigDiffSection(DID_FLASH_CONFIG, "IMX");
    ss << FormatFlashConfigDiffSection(DID_GPX_FLASH_CFG, "GPX");

    // BIT sections
    ss << FormatCuratedSection(DID_BIT, "BIT (IMX)", "hdwBitStatus,calBitStatus");
    ss << FormatCuratedSection(DID_GPX_BIT, "BIT (GPX)", "results,state,testMode,detectedHardwareId");

    // SysParams / Status sections
    ss << FormatCuratedSection(DID_SYS_PARAMS, "SysParams (IMX)", "insStatus,hdwStatus,genFaultCode,imuTemp,baroTemp,mcuTemp");
    ss << FormatCuratedSection(DID_GPX_STATUS, "Status (GPX)", "status,hdwStatus,mcuTemp,gnssStatus0.initState,gnssStatus0.runState,gnssStatus1.initState,gnssStatus1.runState,rtkMode");

    return ss.str();
}
