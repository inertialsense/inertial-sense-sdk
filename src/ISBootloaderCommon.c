/**
 * @file ISBootloaderCommon.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating embedded systems
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISUtilities.h"
#include "ISComm.h"
#include "ISBootloaderCommon.h"
#include "serialPort.h"
#include "serialPortPlatform.h"
#include "inertialSenseBootLoader.h"

const char* is_uins_5_firmware_needle = "uINS_5";
const char* is_uins_3_firmware_needle = "uINS_3";
const char* is_evb_2_firmware_needle = "EVB_2";

void is_destroy_context(is_device_context* ctx)
{
    free(ctx);
}

is_operation_result is_check_version(is_device_context* ctx)
{
    if (serialPortOpenRetry(&ctx->handle.port, ctx->handle.port_name, ctx->baud_rate, 1) == 0)
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }

    // Get DID_DEV_INFO from the uINS.
    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    size_t messageSize = is_comm_get_data(&comm, DID_DEV_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        printf("Failed to request device info\r\n");
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    protocol_type_t ptype;
    int n = is_comm_free(&comm);
    dev_info_t* dev_info = NULL;
    if (n = serialPortReadTimeout(&ctx->handle.port, comm.buf.tail, n, 50))
    {
        comm.buf.tail += n;
        while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
        {
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_DEV_INFO)
            {
                dev_info = (dev_info_t*)comm.dataPtr;
            }
        }
    }

    uint8_t major = 0;

    if(dev_info)
    {
        major = dev_info->hardwareVer[0];

        if(major != ctx->match_props.major)
        {
            printf("Wrong device version\r\n");
            serialPortClose(&ctx->handle.port);
            ctx->scheme = IS_SCHEME_UNKNOWN;
            return IS_OP_ERROR; 
        }
    }
    else
    {
        printf("Failed to get device info, device may already be in bootloader mode\r\n");
    }

    serialPortClose(&ctx->handle.port);
    return IS_OP_OK;
}

is_operation_result is_jump_to_bootloader(is_device_context* ctx)
{
    int baudRates[] = { ctx->baud_rate, IS_BAUD_RATE_BOOTLOADER, IS_BAUD_RATE_BOOTLOADER_RS232, IS_BAUD_RATE_BOOTLOADER_SLOW };

    is_dfu_list list;
    is_list_dfu(&list);

    size_t start_num = list.present;

    // in case we are in program mode, try and send the commands to go into bootloader mode
    unsigned char c = 0;
    for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(baudRates); i++)
    {
        if (baudRates[i] == 0)
            continue;

        serialPortClose(&ctx->handle.port);
        if (serialPortOpenRetry(&ctx->handle.port, ctx->handle.port_name, baudRates[i], 1) == 0)
        {
            serialPortClose(&ctx->handle.port);
            return IS_OP_ERROR;
        }

        for(size_t loop = 0; loop < 10; loop++)
        {
            serialPortWriteAscii(&ctx->handle.port, "STPB", 4);
            serialPortWriteAscii(&ctx->handle.port, ctx->bl_enable_command, 4);
            if(ctx->match_props.major < 5)
            {
                c = 0;
                if(serialPortReadCharTimeout(&ctx->handle.port, &c, 10) == 1)
                {
                    if(c == '$')
                    {
                        // done, we got into bootloader mode
                        i = 9999;
                        break;
                    }
                }
                else
                {
                    // Flush and close the port to prepare for DFU check
                    serialPortFlush(&ctx->handle.port);
                }
            }
            else if(ctx->match_props.major == 5)
            {
                // List DFU devices
                is_list_dfu(&list);
                if(list.present > start_num) 
                {   // If a new DFU device has shown up, break
                    i = 9999;
                    break;
                }
            }
            
        }
    }

    serialPortClose(&ctx->handle.port);
    SLEEP_MS(BOOTLOADER_REFRESH_DELAY);
    return IS_OP_OK;
}

void is_update_flash(void* context)
{
    is_device_context* ctx = (is_device_context*)context;
    int ret = IS_OP_ERROR;
    ctx->success = false;
    
    if(!ctx->firmware_file_path) 
    {
        is_destroy_context(ctx);
        return;  // No firmware present
    }

    if(ctx->scheme == IS_SCHEME_DFU)
    {
        ret = is_dfu_flash(ctx);
    }
    else if(ctx->scheme == IS_SCHEME_SAMBA)
    {
        ret = is_samba_flash(ctx);
    }
    else
    {
        strcpy(ctx->error, "Wrong bootloader scheme for device");
        return;
    }

    if(ret == IS_OP_OK) ctx->success = true;

    is_destroy_context(ctx);

    return;
}