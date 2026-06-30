/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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


class cDeviceLog {
public:
    // Legacy v1 .idx record (host-uptime ms + record counter, 16 bytes).
    // Kept for back-compat readers per D-01 (SN-7879). Writers emit v2
    // exclusively; the v1 layout is here only so legacy-aware reader
    // code can deserialize old `.idx` files produced by SDK <= 2.x.
    typedef struct index_record_v1_s {
        uint32_t time;
        uint32_t offset;
        uint32_t msg_id;
        uint32_t reserved;
    } index_record_v1_t;

    // Public spelling now aliases the v2 record. New code uses the v2
    // fields (timestamp = payload-derived ms, did = data ID, flags
    // bit 0 = ToW signal, 64-bit offset).
    using index_record_s = inertial_sense::idx::is_log_idx_record_v2_t;
    using index_record_t = inertial_sense::idx::is_log_idx_record_v2_t;

    cDeviceLog();

    cDeviceLog(device_handle_t dev);

    cDeviceLog(uint16_t hdwId, uint32_t serial);

    virtual ~cDeviceLog();

    virtual void InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFileSize);

    virtual void InitDeviceForReading();

    virtual void ResetToStart();

    virtual bool CloseAllFiles();

    virtual bool FlushToFile() { return true; };

    virtual bool OpenWithSystemApp();

    virtual bool SaveData(p_data_hdr_t *dataHdr, const uint8_t *dataBuf, protocol_type_t ptype = _PTYPE_INERTIAL_SENSE_DATA);

    virtual bool SaveData(int dataSize, const uint8_t *dataBuf, cLogStats &globalLogStats);

    virtual p_data_buf_t *ReadData() = 0;

    virtual packet_t* ReadPacket(protocol_type_t& ptype) { (void)ptype; return NULL; };

    virtual void SetSerialNumber(uint32_t serialNumber) = 0;

    device_handle_t getDevice() { return device; }

    virtual std::string LogFileExtention() = 0;

    virtual void Flush() {}

    bool SetupReadInfo(const std::string &directory, const std::string &deviceName, const std::string &timeStamp);

    device_handle_t Device();

    dev_info_t DeviceInfo();

    uint16_t HardwareId() { return m_devHdwId; }
    uint32_t SerialNumber() { return m_devSerialNo; }
    std::string& getDeviceId() { return m_deviceId; }

    // void SetDeviceInfo(const dev_info_t *info);
    uint64_t FileSize() { return m_fileSize; }

    uint64_t LogSize() { return m_logSize; }

    uint32_t FileCount() { return m_fileCount; }

    std::string GetNewBaseFileName(uint32_t serialNumber, uint32_t fileCount, const char* suffix);
    std::string GetNewFileName(uint32_t serialNumber, uint32_t fileCount, const char *suffix);

    void SetKmlConfig(bool gpsData = true, bool showTracks = true, bool showPoints = true, bool showPointTimestamps = true, double pointUpdatePeriodSec = 1.0, bool altClampToGround = true) {
        m_enableGpsLogging = gpsData;
        m_showTracks = showTracks;
        m_showPoints = showPoints;
        m_showPointTimestamps = showPointTimestamps;
        m_pointUpdatePeriodSec = pointUpdatePeriodSec;
        m_altClampToGround = altClampToGround;
    }

    void ShowParseErrors(bool show) { m_showParseErrors = show; }

    void UpdateStatsFromFile(p_data_buf_t *data);
    void UpdateStatsFromFile(protocol_type_t ptype, int id, double timestamp);
    std::string LogStatsString() { return m_logStats.Stats(); }

    virtual is_comm_instance_t* IsCommInstance() { return NULL; }

    // D-01 / SN-7879: addIndexRecord populates a v2 record from the
    // packet header / buffer when available, else falls back to a
    // streaming-path entry with did=0 and timestamp=host-uptime-delta.
    // The default-args overload preserves the existing void signature
    // for the streaming-only callers (cDeviceLogRaw) that have no
    // parsed header.
    void addIndexRecord(const p_data_hdr_t* dataHdr = nullptr,
                        const uint8_t* dataBuf = nullptr);
    bool writeIndexChunk();
    // Rewrites the header at offset 0 with final totals + sets the
    // FINALIZED flag. Idempotent. Called by CloseAllFiles().
    bool finalizeIndex();

protected:
    bool OpenNewSaveFile();

    bool OpenNextReadFile();

    void OnReadPacket(packet_t* pkt, protocol_type_t ptype);
    void OnReadData(p_data_buf_t *data);

    device_handle_t device;                                     //!< ISDevice reference to source of data

    uint16_t m_devHdwId = 0;                                    //!< used when reading a file and no ISDevice is available
    uint32_t m_devSerialNo = static_cast<uint32_t>(-1);        //!< used when reading a file, and no ISDevice is available
    std::string m_deviceId;                                     //!< a string representation of a unique device id (hdwid+sn)

    std::vector<std::string> m_fileNames;
    cISLogFileBase *m_pFile = NULL;
    cISLogFileBase *m_indexFile = NULL;
    std::string m_directory;
    std::string m_timeStamp;
    std::string m_fileName;
    bool m_writeMode = false;                                   //!< Logger initialized for writing
    bool m_showParseErrors = false;
    uint64_t m_fileSize = 0;
    uint64_t m_logSize = 0;
    uint32_t m_fileCount = 0;
    uint64_t m_maxDiskSpace;
    uint32_t m_maxFileSize;
    bool m_altClampToGround = true;
    bool m_enableGpsLogging = true;
    bool m_showTracks = true;
    bool m_showPoints;
    bool m_showPointTimestamps = true;
    double m_pointUpdatePeriodSec = 1.0f;
    cLogStats m_logStats;
    std::vector<index_record_t> m_indexChunks;      //!< v2 records buffered in memory, waiting to be flushed via writeIndexChunk()
    uint64_t m_lastIndexOffset = 0;                 //!< running byte offset into the .raw segment; v2 widened from u32 to u64 since segments can exceed 4GB
    uint32_t m_logStartUpTime = 0;                  //!< the system uptime (in millis) at the moment this index was created
    uint32_t m_lastIndexTime = 0;                   //!< system uptime (in ms) of the last index record; v2 still tracks this for the host-uptime fallback path

    // D-01 / SN-7879 v2 .idx bookkeeping. The header at offset 0 of
    // the .idx file carries these aggregates; finalizeIndex() seeks(0)
    // and rewrites the header on close.
    bool     m_idxHeaderWritten = false;            //!< true once the v2 header has been emitted (first writeIndexChunk call)
    uint64_t m_idxTotalRecords  = 0;                //!< running count of records written to the .idx file
    uint64_t m_idxFirstTimestampMs = 0;             //!< timestamp of the first record (set on first record)
    uint64_t m_idxLastTimestampMs  = 0;             //!< timestamp of the most recent record

};

#endif // DEVICE_LOG_H
