/**
 * @file ISBootloaderSamba.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating SAM-BA capable devices
 * 
 * @note This is a compatibility layer that calls inertialSenseBootLoader 
 *  routines for consistency with the new bootloader stuff (ISBootloaderCommon)
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderSamba.h"
#include "inertialSenseBootLoader.h"
#include "serialPort.h"

is_device_context* is_create_samba_context(
    const char* firmware_file_name,
    const char* port_name
)
{
    is_device_context* ctx = malloc(sizeof(is_device_context));

    if(strstr(firmware_file_name, is_uins_3_firmware_needle))
    {   // uINS-3/4
        ctx->scheme = IS_SCHEME_SAMBA;
        ctx->match_props.type = IS_DEVICE_UINS;
        ctx->match_props.match = 
            IS_DEVICE_MATCH_FLAG_VID | 
            IS_DEVICE_MATCH_FLAG_PID | 
            IS_DEVICE_MATCH_FLAG_TYPE | 
            IS_DEVICE_MATCH_FLAG_MAJOR;
        ctx->match_props.major = 3;
        ctx->match_props.vid = SAMBA_DESCRIPTOR_VENDOR_ID;
        ctx->match_props.pid = SAMBA_DESCRIPTOR_PRODUCT_ID;
    } 
    else if(strstr(firmware_file_name, is_evb_2_firmware_needle))
    {   // EVB-2
        ctx->scheme = IS_SCHEME_SAMBA;
        ctx->match_props.type = IS_DEVICE_EVB;
        ctx->match_props.match = 
            IS_DEVICE_MATCH_FLAG_VID | 
            IS_DEVICE_MATCH_FLAG_PID | 
            IS_DEVICE_MATCH_FLAG_TYPE | 
            IS_DEVICE_MATCH_FLAG_MAJOR;
        ctx->match_props.major = 2;
        ctx->match_props.vid = SAMBA_DESCRIPTOR_VENDOR_ID;
        ctx->match_props.pid = SAMBA_DESCRIPTOR_PRODUCT_ID;
    }
    else
    {
        free(ctx);
        return NULL;
    }

    strncpy(ctx->handle.port_name, port_name, 64);
    ctx->handle.status = IS_HANDLE_TYPE_SERIAL;

    return ctx;
}

is_operation_result is_samba_flash(is_device_context* ctx)
{
    bootload_params_t params;

    serialPortPlatformInit((serial_port_t*)ctx->handle.port);
    serialPortSetPort((serial_port_t*)ctx->handle.port, (const char*)ctx->handle.port_name);
    params.uploadProgress = ctx->update_progress_callback;
    params.verifyProgress = ctx->verify_progress_callback;
    params.statusText = ctx->info_callback;
    params.fileName = (const char*)ctx->firmware_file_path;
    params.bootName = ctx->bootloader_file_path;
    params.forceBootloaderUpdate = ctx->force_bootloader_update;
    params.port = (serial_port_t*)ctx->handle.port;
    params.verifyFileName = NULL;   // TODO: Add verify
    params.flags.bitFields.enableVerify = (ctx->verification_style == IS_VERIFY_ON);
    params.baudRate = ctx->baud_rate;

    bootloadFileEx(&params);

    strncpy(ctx->error, params.error, BOOTLOADER_ERROR_LENGTH);

    return IS_OP_OK;
}