/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderAPP.h"
#include "ISComm.h"

#include <mutex>
#include <algorithm>

using namespace ISBootloader;

std::vector<uint32_t> cISBootloaderAPP::serial_list;
std::mutex cISBootloaderAPP::serial_list_mutex;

is_operation_result cISBootloaderAPP::match_test(void* param)
{
    const char* serial_name = (const char*)param;

    if(strnlen(serial_name, 100) != 0 && strncmp(serial_name, m_port->port, 100) == 0)
    {
        return IS_OP_OK;
    }

    return IS_OP_ERROR;
}

eImageSignature cISBootloaderAPP::check_is_compatible()
{
    serialPortFlush(m_port);

    // Get DID_DEV_INFO from the uINS.
    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    int messageSize;

    messageSize = is_comm_get_data(&comm, DID_DEV_INFO, 0, 0, 0);
    for(int i = 0; i < 2; i++)  // HACK: Send this twice. After leaving DFU mode, the serial port doesn't respond to the first request.
    if (messageSize != serialPortWrite(m_port, comm.buf.start, messageSize))
    {
        //serialPortClose(m_port);
        return IS_IMAGE_SIGN_NONE;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_DEV_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(m_port, comm.buf.start, messageSize))
    {
        //serialPortClose(m_port);
        return IS_IMAGE_SIGN_NONE;
    }

    protocol_type_t ptype;
    int n = is_comm_free(&comm);
    dev_info_t* dev_info = NULL;
    dev_info_t* evb_dev_info = NULL;
    uint32_t valid_signatures = 0;
    if ((n = serialPortReadTimeout(m_port, comm.buf.start, n, 200)))
    {
        comm.buf.tail += n;
        while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
        {
            if(ptype == _PTYPE_IS_V1_DATA)
            {
                switch(comm.dataHdr.id)
                {
                case DID_DEV_INFO:
                    dev_info = (dev_info_t*)comm.dataPtr;
                    m_sn = dev_info->serialNumber;
                    if(dev_info->hardwareVer[0] == 5)
                    {   /** IMX-5 */
                        valid_signatures |= IS_IMAGE_SIGN_IMX_5p0;
                        valid_signatures |= IS_IMAGE_SIGN_ISB_STM32L4;
                    }
                    else if (dev_info->hardwareVer[0] == 3 || dev_info->hardwareVer[0] == 4)
                    {   /** uINS-3/4 */
                        valid_signatures |= IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K;
                        valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
                    }
                    break;    
                case DID_EVB_DEV_INFO:
                    evb_dev_info = (dev_info_t*)comm.dataPtr;
                    if (evb_dev_info->hardwareVer[0] == 2)
                    {   /** EVB-2 - all firmwares are valid except for STM32 bootloader (no VCP support) */
                        valid_signatures |= IS_IMAGE_SIGN_IMX_5p0;
                        valid_signatures |= IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K;
                        valid_signatures |= IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K;
                        valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
                    }
                    break;
                }
            }
        }
    }

    return (eImageSignature)valid_signatures;
}

is_operation_result cISBootloaderAPP::reboot()
{
    // TODO: Implement
    // SYS_CMD_SOFTWARE_RESET

    m_info_callback(this, "(APP) Rebooting...", IS_LOG_LEVEL_INFO);

    return IS_OP_OK;
}

is_operation_result cISBootloaderAPP::reboot_down(uint8_t major, char minor, bool force)
{
    (void)force;
    (void)minor;
    (void)major;

    m_info_callback(this, "(APP) Rebooting to ISB mode...", IS_LOG_LEVEL_INFO);

    // In case we are in program mode, try and send the commands to go into bootloader mode
    uint8_t c = 0;
  
    for (size_t loop = 0; loop < 10; loop++)
    {
        if (!serialPortWriteAscii(m_port, "STPB", 4)) break;     // If the write fails, assume the device is now in bootloader mode.
        if (!serialPortWriteAscii(m_port, m_app.enable_command, 4)) break;
        c = 0;
        if (serialPortReadCharTimeout(m_port, &c, 13) == 1)
        {
            if (c == '$')
            {
                // done, we got into bootloader mode
                break;
            }
        }
        else serialPortFlush(m_port);
    }

    return IS_OP_OK;
}

uint32_t cISBootloaderAPP::get_device_info()
{
    serialPortFlush(m_port);

    // Get DID_DEV_INFO from the uINS.
    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    int messageSize;
    
    messageSize = is_comm_get_data(&comm, DID_DEV_INFO, 0, 0, 0);
    for(int i = 0; i < 2; i++)  // HACK: Send this twice. After leaving DFU mode, the serial port doesn't respond to the first request.
    if (messageSize != serialPortWrite(m_port, comm.buf.start, messageSize))
    {
        // serialPortClose(&ctx->handle.port);
        return 0;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_DEV_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(m_port, comm.buf.start, messageSize))
    {
        // serialPortClose(&ctx->handle.port);
        return 0;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_STATUS, 0, 0, 0);
    if (messageSize != serialPortWrite(m_port, comm.buf.start, messageSize))
    {
        // serialPortClose(&ctx->handle.port);
        return 0;
    }

    // Wait 10ms for messages to come back
    serialPortSleep(m_port, 10);

    protocol_type_t ptype;
    int n = is_comm_free(&comm);
    dev_info_t* dev_info = NULL;
    dev_info_t* evb_dev_info = NULL;
    evb_status_t* evb_status = NULL;
    uint8_t evb_version[4];
    if ((n = serialPortReadTimeout(m_port, comm.buf.start, n, 200)))
    {
        comm.buf.tail += n;
        while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
        {
            if(ptype == _PTYPE_IS_V1_DATA)
            {
                switch(comm.dataHdr.id)
                {
                case DID_DEV_INFO:
                    dev_info = (dev_info_t*)comm.dataPtr;
                    memcpy(m_app.uins_version, dev_info->hardwareVer, 4);
                    m_sn = dev_info->serialNumber;
                    break;    
                case DID_EVB_DEV_INFO:
                    evb_dev_info = (dev_info_t*)comm.dataPtr;
                    memcpy(evb_version, evb_dev_info->hardwareVer, 4);
                    break;
                case DID_EVB_STATUS:
                    evb_status = (evb_status_t*)comm.dataPtr;
                    if(evb_status->firmwareVer[0]) memcpy(m_app.evb_version, evb_version, 4);
                    else memset(m_app.evb_version, 0, 4);
                    break;
                }
            }
        }
    }

    return m_sn;
}
