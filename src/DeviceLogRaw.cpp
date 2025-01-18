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
    is_comm_init(&m_comm, m_commBuf, sizeof(m_commBuf));
}

cDeviceLogRaw::cDeviceLogRaw(const ISDevice *dev) : cDeviceLog(dev) {
    is_comm_init(&m_comm, m_commBuf, sizeof(m_commBuf));
}

cDeviceLogRaw::cDeviceLogRaw(uint16_t hdwId, uint32_t serialNo) : cDeviceLog(hdwId, serialNo) {
    is_comm_init(&m_comm, m_commBuf, sizeof(m_commBuf));
};


void cDeviceLogRaw::InitDeviceForWriting(std::string timestamp, std::string directory, uint64_t maxDiskSpace, uint32_t maxFileSize)
{
//     m_chunk.Init(chunkSize);
    m_chunk.Clear();
    m_chunk.m_hdr.pHandle = (device != nullptr ? device->portHandle : -1);

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
    {	// Write any remaining chunk data to file
        WriteChunkToFile();

        return true;
    }

    return false;
}

bool cDeviceLogRaw::SaveData(int dataSize, const uint8_t* dataBuf, cLogStats &globalLogStats)
{
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
				if (m_showParseErrors)
				{ 
					if (m_comm.rxErrorCount>1) 
					{ 
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

                        // Record the serial number in the chunk header if available
                        if (!copyDataPToStructP2((void *) devInfo, &m_comm.rxPkt.dataHdr, m_comm.rxPkt.data.ptr, sizeof(dev_info_t)))
                        {
                            int start = m_comm.rxPkt.dataHdr.offset;
                            int end = m_comm.rxPkt.dataHdr.offset + m_comm.rxPkt.dataHdr.size;
                            int snOffset = offsetof(dev_info_t, serialNumber);

                            // Did we really get the serial number?
                            if (start <= snOffset && (int) (snOffset + sizeof(uint32_t)) <= end) {
                                m_chunk.m_hdr.devSerialNum = devInfo->serialNumber;
                            }
                        }
                    }
                }
                break;
            }

            // Update log statistics
            m_logStats.LogData(ptype, m_comm.rxPkt.id, m_comm.rxPkt.size, timestamp);
            globalLogStats.LogData(ptype, m_comm.rxPkt.id, m_comm.rxPkt.size, timestamp);
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

    return true;
}


p_data_buf_t* cDeviceLogRaw::ReadData()
{
    p_data_buf_t* data = NULL;

    // Read data from chunk
    while (!(data = ReadDataFromChunk()))
    {
        // Read next chunk from file
        if (!ReadChunkFromFile())
        {   // File is empty
            return NULL;
        }
    }

    return data;
}


p_data_buf_t* cDeviceLogRaw::ReadDataFromChunk()
{
    int chunkSize = m_chunk.GetDataSize();

    // Ensure chunk has data
    if (chunkSize <= 0)
    {
        return NULL;
    }

    while (m_chunk.GetDataSize())
    {
        uint8_t *dataPtr = m_chunk.GetDataPtr();

        if (dataPtr == NULL)
        {	// No more data
            return NULL;
        }

        // Read one byte at a time
        uint8_t data = *dataPtr;
        m_chunk.PopFront(1);

        protocol_type_t ptype;
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
                m_logStats.LogData(ptype, m_comm.rxPkt.id, m_comm.rxPkt.size);
                break;

            case _PTYPE_INERTIAL_SENSE_DATA:
            case _PTYPE_INERTIAL_SENSE_CMD:
                m_logStats.LogData(ptype, m_comm.rxPkt.id, m_comm.rxPkt.size, cISDataMappings::TimestampOrCurrentTime(&m_comm.rxPkt.dataHdr, m_comm.rxPkt.data.ptr));

                m_pData.hdr = m_comm.rxPkt.dataHdr;
                memcpy(m_pData.buf, m_comm.rxPkt.data.ptr + m_comm.rxPkt.dataHdr.offset, m_comm.rxPkt.dataHdr.size);
                return &m_pData;
            }
        }
    }

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


