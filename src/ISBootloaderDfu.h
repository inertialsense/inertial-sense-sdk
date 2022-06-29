/**
 * @file ISBootloaderDfu.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating DFU capable devices (uINS-5)
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_DFU_H
#define __IS_BOOTLOADER_DFU_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>

#include "ihex.h"
#include "ISBootloaderTypes.h"
#include "ISBootloaderCompat.h"
#include "libusb/libusb.h"

#ifdef __cplusplus
extern "C" {
#endif


// Recipe for DFU serial number:
// sprintf(ctx->match_props.uid, "%X%X", manufacturing_info->uid[0] + manufacturing_info->uid[2], (uint16_t)(manufacturing_info->uid[1] >> 16));

typedef struct 
{
    char uid[IS_UID_MAX_SIZE];   // DFU device serial number, from descriptors
    uint32_t sn;                // Inertial Sense serial number
    is_device_vid_pid usb;      // VID/PID combo
} is_dfu_id;

#define IS_DFU_LIST_LEN     256

typedef struct 
{
    is_dfu_id id[IS_DFU_LIST_LEN];
    size_t present;
} is_dfu_list;

/**
 * @brief List the available DFU devices
 * 
 * @param list strict to be filled with info about the devices
 * @return is_operation_result 
 */
is_operation_result is_list_dfu(is_dfu_list* list);

/**
 * @brief Flash a firmware image
 * 
 * @param context info about the device
 * @param image array of sections in the hex file
 * @param image_sections number of sections populated in the image array
 * @param dev_handle handle to device
 * @return is_operation_result 
 */
is_operation_result is_dfu_flash(is_device_context* context);

#ifdef __cplusplus
}
#endif

#endif	// __IS_BOOTLOADER_DFU_H
