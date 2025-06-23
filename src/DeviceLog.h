/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DEVICE_LOG_H
#define DEVICE_LOG_H

#include <stdio.h>
#include <string.h>
#include <vector>
#include "ISLogFileBase.h"
#include "ISLogStats.h"

extern "C"
{
#include "com_manager.h"
}

// never change these!
#define IS_LOG_FILE_PREFIX "LOG_SN"
#define IS_LOG_TIMESTAMP_LENGTH 15

class ISDevice;

class cDeviceLog {
public:
    cDeviceLog();

    cDeviceLog(const ISDevice *dev);

    cDeviceLog(uint16_t hdwId, uint32_t serial);

    virtual ~cDeviceLog();

    virtual void InitDeviceForWriting(std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize);

    virtual void InitDeviceForReading();

    virtual bool CloseAllFiles();

    virtual bool FlushToFile() { return true; };

    virtual bool OpenWithSystemApp();

    virtual bool SaveData(p_data_hdr_t *dataHdr, const uint8_t *dataBuf, protocol_type_t ptype = _PTYPE_INERTIAL_SENSE_DATA);

    virtual bool SaveData(int dataSize, const uint8_t *dataBuf, cLogStats &globalLogStats);

    virtual p_data_buf_t *ReadData() = 0;

    virtual void SetSerialNumber(uint32_t serialNumber) = 0;

    virtual std::string LogFileExtention() = 0;

    virtual void Flush() {}

    bool SetupReadInfo(const std::string &directory, const std::string &deviceName, const std::string &timeStamp);

    ISDevice* Device();

    const dev_info_t *DeviceInfo();

    uint16_t HardwareId() { return m_devHdwId; }
    uint32_t SerialNumber() { return m_devSerialNo; }

    // void SetDeviceInfo(const dev_info_t *info);
    uint64_t FileSize() { return m_fileSize; }

    uint64_t LogSize() { return m_logSize; }

    uint32_t FileCount() { return m_fileCount; }

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

protected:
    bool OpenNewSaveFile();

    bool OpenNextReadFile();

    const ISDevice *device = nullptr;               //! ISDevice reference to source of data

    uint16_t m_devHdwId = 0;                          //! used when reading a file and no ISDevice is available
    uint32_t m_devSerialNo = -1;                      //! used when reading a file, and no ISDevice is available

    std::vector<std::string> m_fileNames;
    cISLogFileBase *m_pFile = NULL;
    std::string m_directory;
    std::string m_timeStamp;
    std::string m_fileName;
    bool m_writeMode = false;                       //! Logger initialized for writing
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
};

#endif // DEVICE_LOG_H
