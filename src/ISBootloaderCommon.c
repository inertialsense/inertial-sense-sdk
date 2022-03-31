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

is_device_context* is_create_context(
    is_device_handle* handle, 
    is_firmware_settings* firmware,
    int baud_rate,
    is_verification_style verify,
    pfnBootloadProgress upload_cb,
    pfnBootloadProgress verify_cb,
    pfnBootloadStatus info_cb
)
{
    is_device_context* ctx = malloc(sizeof(is_device_context));
    memset(ctx, 0, sizeof(is_device_context));

    memcpy(&ctx->handle, handle, sizeof(is_device_handle));
    memcpy(&ctx->firmware, firmware, sizeof(is_firmware_settings));
    ctx->baud_rate = baud_rate;
    ctx->verification_style = verify;
    ctx->update_progress_callback = upload_cb;
    ctx->verify_progress_callback = verify_cb;
    ctx->info_callback = info_cb;
    ctx->success = false;
    memset(ctx->error, 0, BOOTLOADER_ERROR_LENGTH);

    return ctx;
}

void is_destroy_context(is_device_context* ctx)
{
    free(ctx);
}

static void IntToUnicode(uint32_t value, uint8_t* pbuf, uint8_t len)
{
    size_t idx = 0;

    for(idx = 0; idx < len; idx++)
    {
        if((value >> 28) < 0xA) pbuf[2*idx] = (value >> 28) + '0';
        else pbuf[2*idx] = (value >> 28) + 'A' - 10; 
        
        value = value << 4;
        
        pbuf[2*idx + 1] = 0;
    }
}

is_operation_result is_check_version(is_device_context* ctx)
{
    if(ctx->handle.status == IS_HANDLE_TYPE_LIBUSB) return IS_OP_OK;

    serialPortPlatformInit(&ctx->handle.port);
    serialPortSetPort(&ctx->handle.port, ctx->handle.port_name);
    if (serialPortOpenRetry(&ctx->handle.port, ctx->handle.port_name, ctx->baud_rate, 1) == 0)
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }

    serialPortFlush(&ctx->handle.port);

    // Get DID_DEV_INFO from the uINS.
    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    int messageSize;
    
    messageSize = is_comm_get_data(&comm, DID_DEV_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    // HACK: Send this twice. After leaving DFU mode, the serial port doesn't respond to the first request.
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_DEV_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_STATUS, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    messageSize = is_comm_get_data(&comm, DID_MANUFACTURING_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }

    SLEEP_MS(10);

    protocol_type_t ptype;
    int n = is_comm_free(&comm);
    dev_info_t* dev_info = NULL;
    dev_info_t* evb_dev_info = NULL;
    evb_status_t* evb_status = NULL;
    manufacturing_info_t* manufacturing_info = NULL;
    uint8_t evb_version[4];
    if ((n = serialPortReadTimeout(&ctx->handle.port, comm.buf.start, n, 50)))
    {
        comm.buf.tail += n;
        while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
        {
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_DEV_INFO)
            {
                dev_info = (dev_info_t*)comm.dataPtr;
                memcpy(ctx->hdw_info.uins_version, dev_info->hardwareVer, 4);
            }
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_EVB_DEV_INFO)
            {
                evb_dev_info = (dev_info_t*)comm.dataPtr;
                memcpy(evb_version, evb_dev_info->hardwareVer, 4);
            }
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_EVB_STATUS)
            {
                evb_status = (evb_status_t*)comm.dataPtr;
                if(evb_status->firmwareVer) memcpy(ctx->hdw_info.evb_version, evb_version, 4);   // Only copy EVB status stuff if it is plugged through EVB port
            }
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_MANUFACTURING_INFO)
            {
                manufacturing_info = (manufacturing_info_t*)comm.dataPtr;
                sprintf(ctx->match_props.serial_number, "%X%X", manufacturing_info->serialNumber + manufacturing_info->lotNumber, (uint16_t)(manufacturing_info->key >> 16));
            }
        }
    }
    
    serialPortClose(&ctx->handle.port);
    return IS_OP_OK;
}

is_operation_result is_jump_to_bootloader(is_device_context* ctx)
{
    if(ctx->handle.status == IS_HANDLE_TYPE_LIBUSB)
    {
        return IS_OP_OK;
    }

    int baudRates[] = { ctx->baud_rate, IS_BAUD_RATE_BOOTLOADER, IS_BAUD_RATE_BOOTLOADER_RS232, IS_BAUD_RATE_BOOTLOADER_SLOW };

    is_dfu_list list;
    is_list_dfu(&list);

    size_t start_num = list.present;

    char info[256] = { 0 };
    snprintf(info, 256, "%s: Starting bootloader...", ctx->handle.port_name);
    if(ctx->info_callback) ctx->info_callback((void*)ctx, info);

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
            if(ctx->hdw_info.uins_version[0] != 5 ) // || ctx->hdw_info.evb_version[0] == 2
            {
                c = 0;
                if(serialPortReadCharTimeout(&ctx->handle.port, &c, 10) == 1)
                {
                    if(c == '$')
                    {
                        // done, we got into bootloader mode
                        snprintf(info, 256, "%s: Successfully entered SAM-BA bootloader", ctx->handle.port_name);
                        if(ctx->info_callback) ctx->info_callback((void*)ctx, info);
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
            else if(ctx->hdw_info.uins_version[0] == 5)
            {
                // List DFU devices
                is_list_dfu(&list);
                if(list.present > start_num) 
                {   // If a new DFU device has shown up, break
                    snprintf(info, 256, "%s: Successfully entered DFU bootloader", ctx->handle.port_name);
                    if(ctx->info_callback) ctx->info_callback((void*)ctx, info);
                    ctx->handle.status = IS_HANDLE_TYPE_LIBUSB;
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
    else strcpy(ctx->error, "Error in update flash");

    return;
}