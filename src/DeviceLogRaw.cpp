/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string>
#include <cstdio>
#include <cstddef>
#include <functional>

#include "DeviceLogRaw.h"
#include "ISDataMappings.h"
#include "ISDisplay.h"
#include "ISLogger.h"
#include "ISLogFileFactory.h"
#include "message_stats.h"
#include "protocol_nmea.h"

using namespace std;


cDeviceLogRaw::cDeviceLogRaw() : cDeviceLog()
{
    is_comm_init(&m_comm, m_commBuf, sizeof(m_commBuf), NULL); // TODO: Should we be using callbacks??  Probably
}

cDeviceLogRaw::cDeviceLogRaw(const ISDevice *dev) : cDeviceLog(dev) {
    is_comm_init(&m_comm, m_commBuf, sizeof(m_commBuf), NULL); // TODO: Should we be using callbacks??  Probably
}

cDeviceLogRaw::cDeviceLogRaw(uint16_t hdwId, uint32_t serialNo) : cDeviceLog(hdwId, serialNo) {
    is_comm_init(&m_comm, m_commBuf, sizeof(m_commBuf), NULL); // TODO: Should we be using callbacks??  Probably
};


void cDeviceLogRaw::InitDeviceForWriting(const std::string& timestamp, const std::string& directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
//     m_chunk.Init(chunkSize);
    m_chunk.Clear();
    m_chunk.m_hdr.devSerialNum = SerialNumber();
    if (device) {
        m_chunk.m_hdr.portId = portId(device->port);
        m_chunk.m_hdr.portType = portType(device->port);
    }
    cDeviceLog::InitDeviceForWriting(timestamp, directory, maxDiskSpace, maxFileSize);
}


bool cDeviceLogRaw::CloseAllFiles()
{
    cDeviceLog::CloseAllFiles();

    // Write remaining data to file
    FlushToFile();

    // Close file
    CloseISLogFile(m_pFile);

    return true;
}


bool cDeviceLogRaw::FlushToFile()
{
    cDeviceLog::FlushToFile();

    if (m_writeMode)
    {   // Write any remaining chunk data to file
        WriteChunkToFile();

        return true;
    }

    return false;
}

bool cDeviceLogRaw::SaveData(int dataSize, const uint8_t* dataBuf, cLogStats &globalLogStats)
{
    cDeviceLog::SaveData(dataSize, dataBuf, globalLogStats);    // call into the super, in case it needs to do something special

    // Parse messages for statistics and DID_DEV_INFO
    for (const uint8_t *dPtr = dataBuf; dPtr < dataBuf+dataSize; dPtr++)
    {
        protocol_type_t ptype;
        if ((ptype = is_comm_parse_byte(&m_comm, *dPtr)) != _PTYPE_NONE)
        {
            double timestamp = 0.0;

            switch (ptype)
            {
            default:
                timestamp = current_timeSecD();
                break;

            case _PTYPE_PARSE_ERROR:
                if (m_showParseErrors) {
                    if (m_comm.rxErrorCount > 1) {
                        printf("SN%d SaveData() parse errors: %d\n", m_devSerialNo, m_comm.rxErrorCount);
                    }
                }
                break;

            case _PTYPE_INERTIAL_SENSE_DATA:
            case _PTYPE_INERTIAL_SENSE_CMD:
                {
                    timestamp = cISDataMappings::TimestampOrCurrentTime(&m_comm.rxPkt.dataHdr, m_comm.rxPkt.data.ptr);

                    dev_info_t tmpInfo = {};
                    dev_info_t* devInfo = &tmpInfo;

                    if (m_comm.rxPkt.dataHdr.id == DID_DEV_INFO) {
                        // if we have a device struct, let's use it, otherwise we'll just copy into our local copy
                        if (device != nullptr)
                            devInfo = (dev_info_t *) &(device->devInfo);

                        // Record the serial number, protocol and firmware version in the chunk header if available
                        if (!copyDataPToStructP2((void *) devInfo, &m_comm.rxPkt.dataHdr, m_comm.rxPkt.data.ptr, sizeof(dev_info_t))) {
                            int start = m_comm.rxPkt.dataHdr.offset;
                            int end = m_comm.rxPkt.dataHdr.offset + m_comm.rxPkt.dataHdr.size;

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
                    }
                }
                break;
            }

            // Update log statistics
            m_logStats.LogData(ptype, m_comm.rxPkt.id, timestamp);
            globalLogStats.LogData(ptype, m_comm.rxPkt.id, timestamp);
        }
    }

    // Ensure data will fit in chunk.  If not, create new chunk
    if (dataSize > m_chunk.GetBuffFree())
    {
        // Save chunk to file and clear
        if (!WriteChunkToFile())
        {
            return false;
        }
        else if (m_fileSize >= m_maxFileSize)
        {
            // Close existing file
            CloseAllFiles();
        }
    }

    // Add data header and data buffer to chunk
    m_logSize += dataSize;
    if (!m_chunk.PushBack((unsigned char*)dataBuf, dataSize))
    {
        return false;
    }

    return true;
}


bool cDeviceLogRaw::WriteChunkToFile()
{
    // Make sure we have data to write
    if (m_chunk.GetDataSize() == 0)
    {
        return false;
    }

    // Create first file if it doesn't exist
    if (m_pFile == NULLPTR)
    {
        OpenNewSaveFile();
    }

    // Validate file pointer
    if (m_pFile == NULLPTR)
    {
        return false;
    }

    // Write chunk to file
    int fileBytes = m_chunk.WriteToFile(m_pFile, 0, false);
    if (!m_pFile->good())
    {
        return false;
    }

    // File byte size
    m_fileSize += fileBytes;

    writeIndexChunk();

    return true;
}


packet_t* cDeviceLogRaw::ReadPacket(protocol_type_t &ptype) 
{
    packet_t* pkt = NULL;

    // Read data from chunk
    while (!(pkt = ReadPacketFromChunk(ptype)))
    {
        // Read next chunk from file
        if (!ReadChunkFromFile())
        {
            return NULL;
        }
    }

    return pkt;
}


p_data_buf_t* cDeviceLogRaw::ReadData()
{
    // Read data from chunk
    while (1)
    {
        protocol_type_t ptype;
        ReadPacketFromChunk(ptype);
        switch (ptype)
        {
        default:    // Skip other protocols
            break;

        case _PTYPE_INERTIAL_SENSE_CMD:
        case _PTYPE_INERTIAL_SENSE_DATA:
            return &m_pData;

        case _PTYPE_NONE:   
            // Read next chunk from file
            if (!ReadChunkFromFile())
            {   // File is empty
                return NULL;
            }
        }
    }

    return NULL;
}


packet_t* cDeviceLogRaw::ReadPacketFromChunk(protocol_type_t& ptype)
{
    int chunkSize = m_chunk.GetDataSize();

    // Ensure chunk has data
    if (chunkSize <= 0)
    {
        ptype = _PTYPE_NONE;
        return NULL;
    }

    while (m_chunk.GetDataSize())
    {
        uint8_t *dataPtr = m_chunk.GetDataPtr();

        if (dataPtr == NULL)
        {   // No more data
            ptype = _PTYPE_NONE;
            return NULL;
        }

        // Read one byte at a time
        uint8_t data = *dataPtr;
        m_chunk.PopFront(1);

        if ((ptype = is_comm_parse_byte(&m_comm, data)) != _PTYPE_NONE)
        {
            switch (ptype)
            {
            case _PTYPE_PARSE_ERROR:
                if (m_showParseErrors)
                {
                    if (m_comm.rxErrorCount > 1) { printf("SN%d ReadDataFromChunk() parse errors: %d\n", m_devSerialNo, m_comm.rxErrorCount); }
                }
                break;

            default:
                m_logStats.LogData(ptype, m_comm.rxPkt.id);
                break;

            case _PTYPE_INERTIAL_SENSE_DATA:
            case _PTYPE_INERTIAL_SENSE_CMD:
                m_logStats.LogData(ptype, m_comm.rxPkt.id, cISDataMappings::TimestampOrCurrentTime(&m_comm.rxPkt.dataHdr, m_comm.rxPkt.data.ptr));

                m_pData.hdr = m_comm.rxPkt.dataHdr;
                memcpy(m_pData.buf, m_comm.rxPkt.data.ptr + m_comm.rxPkt.dataHdr.offset, m_comm.rxPkt.dataHdr.size);
            }
            return &m_comm.rxPkt;
        }
    }

    ptype = _PTYPE_NONE;
    return NULL;
}


bool cDeviceLogRaw::ReadChunkFromFile()
{
    // Read next chunk from file
    while (m_chunk.ReadFromFile(m_pFile, false) < 0)
    {
        if (!OpenNextReadFile())
        {
            // No more data or error opening next file
            return false;
        }
    }
    return true;
}

void cDeviceLogRaw::SetSerialNumber(uint32_t serialNumber)
{
    m_devSerialNo = serialNumber;
    m_chunk.m_hdr.devSerialNum = serialNumber;
}


void cDeviceLogRaw::Flush()
{
    if (WriteChunkToFile())
    {
        m_pFile->flush();
    }
}
