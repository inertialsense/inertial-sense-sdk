/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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

#include "DeviceLogSerial.h"
#include "ISLogger.h"
#include "ISLogFileFactory.h"

using namespace std;

cDeviceLogSerial::cDeviceLogSerial() : cDeviceLog() {
    m_chunk.Clear();
}

cDeviceLogSerial::cDeviceLogSerial(device_handle_t dev) : cDeviceLog(dev) {
    m_chunk.Clear();
    m_chunk.SetDevInfo(dev->devInfo);
    m_chunk.m_hdr.devSerialNum = SerialNumber();    // set this seperately, in case the devInfo above doesn't contain it
    if (device) {
        m_chunk.m_hdr.portId = portId(device->port);
        m_chunk.m_hdr.portType = portType(device->port);
    }
}

cDeviceLogSerial::cDeviceLogSerial(uint16_t hdwId, uint32_t serialNo) : cDeviceLog(hdwId, serialNo) {
    m_chunk.Clear();
    m_chunk.m_hdr.devSerialNum = SerialNumber();
}

void cDeviceLogSerial::InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFileSize) {
    m_chunk.Clear();
    m_chunk.m_hdr.devSerialNum = SerialNumber();
    if (device) {
        m_chunk.m_hdr.portId = portId(device->port);
        m_chunk.m_hdr.portType = portType(device->port);
    }
    cDeviceLog::InitDeviceForWriting(timestamp, directory, maxDiskSpace, maxFileSize);
}

void cDeviceLogSerial::InitDeviceForReading()
{
    m_chunk.Clear();
    cDeviceLog::InitDeviceForReading();
}

bool cDeviceLogSerial::CloseAllFiles() {
    // D-119/SN-8626 (mirrors cDeviceLogRaw's SN-8328 fix): flush the buffered
    // chunk to disk BEFORE the base class writes/finalizes the .idx.
    // WriteChunkToFile() lazily creates the real segment file via
    // OpenNewSaveFile(), which is the only place m_fileName is assigned. If the
    // base's CloseAllFiles() ran first (as this did before), a log small enough
    // that no chunk was ever flushed during logging would still have an empty
    // m_fileName at finalize time -- writeIndexChunk()/finalizeIndex() would
    // land on an orphan "./.idx", and the subsequent lazy OpenNewSaveFile()
    // would reset the index state and re-emit an empty, non-finalized
    // <segment>.idx (0 records).
    FlushToFile();

    // Flush any remaining buffered index records and finalize the .idx header
    // against the now-existing segment file.
    cDeviceLog::CloseAllFiles();

    // Close file
    CloseISLogFile(m_pFile);

    // D-119/SN-8626: this segment is finalized. Reset the physical-offset
    // accounting so the NEXT segment's records index from 0 -- otherwise
    // SaveData()'s m_lastIndexOffset computation (below) would use this
    // now-closed file's stale size for the first record(s) of the next
    // segment. (The base's lazy OpenNewSaveFile() also zeroes m_fileSize, but
    // that fires only at the next chunk flush -- too late for records indexed
    // in between.)
    m_fileSize = 0;

    return true;
}

bool cDeviceLogSerial::FlushToFile() {
    cDeviceLog::FlushToFile();

    if (m_writeMode) {   // Write any remaining chunk data to file
        WriteChunkToFile();

        return true;
    }

    return false;
}


bool cDeviceLogSerial::SaveData(p_data_hdr_t *dataHdr, const uint8_t *dataBuf, protocol_type_t ptype) {
    // D-119/SN-8626: ensure this record's chunk fits BEFORE indexing it (moved up
    // from below cDeviceLog::SaveData()'s call, which stamps the .idx offset from
    // m_lastIndexOffset). Indexing must see the post-flush/post-rotation chunk
    // state, or a record that triggers a flush gets stamped with the PRE-flush
    // offset -- wrong by a full chunk. Mirrors cDeviceLogRaw's SN-8328 ordering.
    int32_t dataBytes = sizeof(p_data_hdr_t) + dataHdr->size;
    int32_t buffFree = m_chunk.GetBuffFree();
    if (dataBytes > buffFree) {
        // Save chunk to file and clear
        if (!WriteChunkToFile()) {
            return false;
        } else if (m_fileSize >= m_maxFileSize) {
            // Close existing file
            CloseAllFiles();
        }
    }

    // D-119/SN-8626: stamp this record's TRUE physical .dat file offset before
    // indexing. Unlike .raw (whose on-disk chunks carry no header), .dat's
    // chunks ARE header-prefixed on disk (WriteChunkToFile's default
    // writeHeader=true) -- the still-buffered m_chunk will itself be preceded by
    // a not-yet-written sChunkHeader once flushed, so that header's size counts
    // toward this record's eventual on-disk position even though it hasn't been
    // written yet.
    m_lastIndexOffset = static_cast<uint64_t>(m_fileSize) + sizeof(sChunkHeader)
                       + static_cast<uint64_t>(m_chunk.GetDataSize());

    cDeviceLog::SaveData(dataHdr, dataBuf, ptype);

    dev_info_t tmpInfo = {};
    dev_info_t* devInfo = &tmpInfo;

    if (dataHdr->id == DID_DEV_INFO) {
        // if we have a device struct, let's use it, otherwise we'll just copy into our local copy
        if (device != nullptr)
            devInfo = (dev_info_t *) &(device->devInfo);

        // Record the serial number, protocol and firmware version in the chunk header if available
        if (!copyDataPToStructP2((void *) devInfo, dataHdr, dataBuf, sizeof(dev_info_t))) {
            int start = dataHdr->offset;
            int end = dataHdr->offset + dataHdr->size;

            // Did we really get the protocol version?
            int protOffset = offsetof(dev_info_t, protocolVer);
            if (start <= protOffset && (int) (protOffset + sizeof(uint32_t)) <= end) {
                memcpy(m_chunk.m_hdr.fwVersion, devInfo->protocolVer, 4);
            }

            // Did we really get the firmware version?
            int fwOffset = offsetof(dev_info_t, firmwareVer);
            if (start <= fwOffset && (int) (fwOffset + sizeof(uint32_t)) <= end) {
                memcpy(m_chunk.m_hdr.fwVersion, devInfo->firmwareVer, 4);
            }

            // Did we really get the serial number?
            int snOffset = offsetof(dev_info_t, serialNumber);
            if (start <= snOffset && (int) (snOffset + sizeof(uint32_t)) <= end) {
                m_chunk.m_hdr.devSerialNum = devInfo->serialNumber;
            }
        }
    } else
        m_chunk.m_hdr.devSerialNum = m_devSerialNo;

    // Add data header and data buffer to chunk
    m_logSize += dataHdr->size;
    if (!m_chunk.PushBack((unsigned char *) dataHdr, sizeof(p_data_hdr_t), (unsigned char *) dataBuf, dataHdr->size)) {
        return false;
    }

    return true;
}


bool cDeviceLogSerial::WriteChunkToFile() {
    // Make sure we have data to write
    if (m_chunk.GetDataSize() == 0) {
        return false;
    }

    // Create first file if it doesn't exist
    if (m_pFile == NULLPTR) {
        OpenNewSaveFile();
    }

    // Validate file pointer
    if (m_pFile == NULLPTR) {
        return false;
    }

    // Write chunk to file
    int fileBytes = m_chunk.WriteToFile(m_pFile, 0);
    if (!m_pFile->good()) {
        return false;
    }

    // File byte size
    m_fileSize += fileBytes;

    return true;
}


p_data_buf_t *cDeviceLogSerial::ReadData() {
    p_data_buf_t *data = NULL;

    // Read data from chunk
    while (!(data = ReadDataFromChunk())) {
        // Read next chunk from file
        if (!ReadChunkFromFile()) {
            return NULL;
        }
    }

    // Read is good
    cDeviceLog::UpdateStatsFromFile(data);
    return data;
}


p_data_buf_t *cDeviceLogSerial::ReadDataFromChunk() {
    // Ensure chunk has data
    if (m_chunk.GetDataSize() <= 0) {
        return NULL;
    }

    p_data_buf_t *data = (p_data_buf_t *) m_chunk.GetDataPtr();
    int size = data->hdr.size + sizeof(p_data_hdr_t);
    if (m_chunk.PopFront(size)) {
        return data;
    } else {
        return NULL;
    }
}


bool cDeviceLogSerial::ReadChunkFromFile() {
    // Read next chunk from file
    while (m_chunk.ReadFromFile(m_pFile) < 0) {
        if (!OpenNextReadFile()) {
            // No more data or error opening next file
            return false;
        }
    }
    return true;
}


void cDeviceLogSerial::SetSerialNumber(uint32_t serialNumber) {
    m_devSerialNo = serialNumber;
    m_chunk.m_hdr.devSerialNum = serialNumber;
}


void cDeviceLogSerial::Flush() {
    if (WriteChunkToFile()) {
        m_pFile->flush();
    }
}


