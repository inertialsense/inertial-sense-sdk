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

#include "ISBootloaderCommon.h"

#include "serialPort.h"
#include "serialPortPlatform.h"
#include "inertialSenseBootLoader.h"

#include "ISUtilities.h"

const char* is_uins_5_firmware_needle = "uINS_5";
const char* is_uins_3_firmware_needle = "uINS_3";
const char* is_evb_2_firmware_needle = "EVB_2";

void is_destroy_context(is_device_context* ctx)
{
    free(ctx);
}

is_operation_result is_jump_to_bootloader(const char* portName, int baudRate, const char* bootloadEnableCmd)
{
    int baudRates[] = { baudRate, IS_BAUD_RATE_BOOTLOADER, IS_BAUD_RATE_BOOTLOADER_RS232, IS_BAUD_RATE_BOOTLOADER_SLOW };

    // detect if device is already in bootloader mode
    serial_port_t port;

    unsigned char c = 0;
    for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(baudRates); i++)
    {
        if (baudRates[i] == 0)
            continue;

        serialPortClose(&port);
        if (serialPortOpenRetry(&port, portName, baudRates[i], 1) == 0)
        {
            serialPortClose(&port);
            return IS_OP_ERROR;
        }
        for (size_t loop = 0; loop < 10; loop++)
        {
            serialPortWriteAscii(&port, "STPB", 4);
            serialPortWriteAscii(&port, bootloadEnableCmd, 4);
            
            // TODO: Probe to check if this is a uINS-5 DFU device
            // TODO: Check for DFU/SAM-BA mode here (maybe by looking if the port disappeared for DFU?)
        }
    }

    serialPortClose(&port);
    SLEEP_MS(BOOTLOADER_REFRESH_DELAY);

    return IS_OP_OK;
}

void is_update_flash(void* context)
{
    is_device_context* ctx = (is_device_context*)context;
    int ret = IS_OP_ERROR;
    ctx->success = false;
    
    if(!ctx->firmware_file_path) return;  // No firmware present

    if(ctx->scheme == IS_SCHEME_DFU)
    {

        ret = is_dfu_flash(ctx);
    }
    else if(ctx->scheme == IS_SCHEME_SAMBA)
    {
        ret = is_samba_flash(ctx);
    }

    if(ret == IS_OP_OK) ctx->success = true;

    is_destroy_context(ctx);

    return;
}