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

#include "ISUtilities.h"
#include "ISBootloaderSamba.h"
#include "inertialSenseBootLoader.h"
#include "serialPort.h"
#include "serialPortPlatform.h"

is_device_context* is_init_samba_context(is_device_context* ctx)
{
    ctx->scheme = IS_SCHEME_SAMBA;
    ctx->match_props.match = 
        IS_DEVICE_MATCH_FLAG_VID | 
        IS_DEVICE_MATCH_FLAG_PID | 
        IS_DEVICE_MATCH_FLAG_TYPE | 
        IS_DEVICE_MATCH_FLAG_MAJOR;
    ctx->match_props.vid = SAMBA_DESCRIPTOR_VENDOR_ID;
    ctx->match_props.pid = SAMBA_DESCRIPTOR_PRODUCT_ID;

    return ctx;
}

is_operation_result is_samba_flash(is_device_context* ctx)
{
    bootload_params_t params;

    params.uploadProgress = ctx->update_progress_callback;
    params.verifyProgress = ctx->verify_progress_callback;
    params.statusText = ctx->info_callback;
    params.forceBootloaderUpdate = ctx->firmware.samba_force_update;
    params.port = &ctx->handle.port;
    params.verifyFileName = NULL;   // TODO: Add verify
    params.flags.bitFields.enableVerify = (ctx->verification_style == IS_VERIFY_ON);
    params.baudRate = ctx->baud_rate;
    params.obj = ctx;

    if(ctx->firmware.samba_bootloader_path[0] != 0 )
    {
        params.bootName = (const char*)ctx->firmware.samba_bootloader_path;
    }

    int sleep_time = 0;

    /* Upload EVB firmware */
    if(ctx->hdw_info.evb_version[0] == 2 && strlen(ctx->firmware.evb_2_firmware_path) != 0)
    {
        params.fileName = (const char*)ctx->firmware.evb_2_firmware_path;
        strncpy(params.bootloadEnableCmd, "EBLE", 5);
        bootloadFileEx(&params);
        sleep_time = 5000;
    }

    /* Upload uINS-3/4 firmware */
    if(ctx->hdw_info.uins_version[0] == 3 && strlen(ctx->firmware.uins_3_firmware_path) != 0)
    {
        SLEEP_MS(sleep_time);   // Wait for boot
        params.fileName = (const char*)ctx->firmware.uins_3_firmware_path;
        strncpy(params.bootloadEnableCmd, "BLEN", 5);
        bootloadFileEx(&params);
    }
    else if(ctx->hdw_info.uins_version[0] == 4 && strlen(ctx->firmware.uins_4_firmware_path) != 0)
    {
        SLEEP_MS(sleep_time);   // Wait for boot
        params.fileName = (const char*)ctx->firmware.uins_4_firmware_path;
        strncpy(params.bootloadEnableCmd, "BLEN", 5);
        bootloadFileEx(&params);
    }

    strncpy(ctx->error, params.error, BOOTLOADER_ERROR_LENGTH);

    return IS_OP_OK;
}