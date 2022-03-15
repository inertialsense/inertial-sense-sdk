/**
 * @file ISBootloaderDfu.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating DFU capable devices (STM32)
 * @version 0.1
 * @date 2022-03-15
 * 
 * @copyright Copyright (c) 2022 Inertial Sense, Inc
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_DFU_H
#define __IS_BOOTLOADER_DFU_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <libusb.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h>

#include "ihex.h"
#include "ISBootloaderLog.h"
#include "ISBootloaderTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	UINS5_DESCRIPTOR_VENDOR_ID = 0x0483,
	UINS5_DESCRIPTOR_PRODUCT_ID = 0xdf11
} uins5_descriptor;

typedef enum
{
	UINS5_DFU_INTERFACE_FLASH    = 0, // @Internal Flash  /0x08000000/0256*0002Kg
	UINS5_DFU_INTERFACE_OPTIONS  = 1, // @Option Bytes  /0x1FFF7800/01*040 e
	UINS5_DFU_INTERFACE_OTP      = 2, // @OTP Memory /0x1FFF7000/01*0001Ke
	UINS5_DFU_INTERFACE_FEATURES = 3  // @Device Feature/0xFFFF0000/01*004 e
} uins5_dfu_interface_alternatives;

typedef struct 
{
	ihex_image_section_t* image;
	int num_image_sections;
} is_dfu_config;

/**
 * @brief Probes for DFU devices (currently ones with VID and PID of STM32 bootloader)
 * 
 * @param uri_list list of URIs that will be filled
 * @param callback_fn callback when device is found
 */
void is_dfu_probe(is_device_uri_list* uri_list, is_list_devices_callback_fn callback_fn);

/**
 * @brief 
 * 
 * @param context contains info about specific device
 * @param config contains info about the image to be flashed
 * @param dev_handle 
 * @return is_operation_result 
 */
is_operation_result is_dfu_flash(const is_device_context const * context);

#ifdef __cplusplus
}
#endif

#endif	// __IS_BOOTLOADER_DFU_H
