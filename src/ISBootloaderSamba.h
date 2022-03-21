/**
 * @file ISBootloaderSamba.h
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

#ifndef __IS_BOOTLOADER_SAMBA_H
#define __IS_BOOTLOADER_SAMBA_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>

#include "ihex.h"
#include "ISBootloaderTypes.h"
#include "ISBootloaderCommon.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SAMBA_DESCRIPTOR_VENDOR_ID = 0x03eb,
    SAMBA_DESCRIPTOR_PRODUCT_ID = 0x6124
} samba_descriptor;

/**
 * @brief Create a SAM-BA bootloader context
 * 
 * @param firmware_file_name 
 * @param port_name i.e. COMx, /dev/ttyACMx
 * @return is_device_context* 
 */
is_device_context* is_create_samba_context(
    const char* firmware_file_name,
    const char* port_name
);

/**
 * @brief Flash a firmware image
 * 
 * @param context info about the device
 * @param firmware_path system path to the firmware image
 * @return is_operation_result 
 */
is_operation_result is_samba_flash(is_device_context* ctx);

#ifdef __cplusplus
}
#endif

#endif	// __IS_BOOTLOADER_SAMBA_H
