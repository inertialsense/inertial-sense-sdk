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

#define STM32_PAGE_SIZE 0x800
#define STM32_PAGE_ERROR_MASK 0x7FF

typedef enum
{
	DFU_ERROR_NONE = 0,
	DFU_ERROR_NO_DEVICE = -1,
	DFU_ERROR_LIBUSB = -2,
	DFU_ERROR_STATUS = -3,
	DFU_ERROR_INVALID_ARG = -4,
	DFU_ERROR_NO_FILE = -5,
	DFU_ERROR_TIMEOUT = -6,
} eDfuError;

typedef enum
{
	DFU_STATUS_OK = 0,
	DFU_STATUS_ERR_TARGET,
	DFU_STATUS_ERR_FILE,
	DFU_STATUS_ERR_WRITE,
	DFU_STATUS_ERR_ERASED,
	DFU_STATUS_ERR_CHECK_ERASED,
	DFU_STATUS_ERR_PROG,
	DFU_STATUS_ERR_VERIFY,
	DFU_STATUS_ERR_ADDRESS,
	DFU_STATUS_ERR_NOTDONE,
	DFU_STATUS_ERR_FIRMWARE,
	DFU_STATUS_ERR_VENDOR,
	DFU_STATUS_ERR_USBR,
	DFU_STATUS_ERR_POR,
	DFU_STATUS_ERR_UNKNOWN,
	DFU_STATUS_ERR_STALLEDPKT,
	
	DFU_STATUS_NUM,
} eDfuStatus;

typedef enum
{
	DFU_STATE_APP_IDLE = 0,
	DFU_STATE_APP_DETACH,
	DFU_STATE_IDLE,
	DFU_STATE_DNLOAD_SYNC,
	DFU_STATE_DNBUSY, 
	DFU_STATE_DNLOAD_IDLE,
	DFU_STATE_MANIFEST_SYNC,
	DFU_STATE_MANIFEST,
	DFU_STATE_MANIFEST_WAIT_RESET,
	DFU_STATE_UPLOAD_IDLE,
	DFU_STATE_ERROR,

	DFU_STATE_NUM,
} eDfuState;

typedef enum uins5_descriptor_enum
{
	UINS5_DESCRIPTOR_VENDOR_ID = 0x0483,
	UINS5_DESCRIPTOR_PRODUCT_ID = 0xdf11
} uins5_descriptor;

typedef enum uins5_dfu_interface_alternatives_enum
{
	UINS5_DFU_INTERFACE_FLASH    = 0, // @Internal Flash  /0x08000000/0256*0002Kg
	UINS5_DFU_INTERFACE_OPTIONS  = 1, // @Option Bytes  /0x1FFF7800/01*040 e
	UINS5_DFU_INTERFACE_OTP      = 2, // @OTP Memory /0x1FFF7000/01*0001Ke
	UINS5_DFU_INTERFACE_FEATURES = 3  // @Device Feature/0xFFFF0000/01*004 e
} uins5_dfu_interface_alternatives;

typedef struct
{
	/* Address the image section needs to be programmed to */
	uint32_t address;

	/* Pointer to the section data */
	uint8_t* image;

	/* Length of this section*/
	uint32_t len;
} ihex_image_section_t;

typedef struct 
{
	/* Filename of the hex file */
	const char* filename;

	/* USB vendor ID */
	uint16_t vendor_id;
	
	/* USB product ID */
	uint16_t product_id;
} uins_dfu_config;

void uinsProbeDfuDevices(uins_device_uri_list* uri_list, uins_list_devices_callback_fn callback_fn);
int uinsBootloadFileExDfu(const uins_device_context const * context, uins_dfu_config* config);

eDfuError dfu_DETACH(libusb_device_handle** dev_handle, uint8_t timeout);
eDfuError dfu_DNLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len);
eDfuError dfu_UPLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len);
eDfuError dfu_GETSTATUS(libusb_device_handle** dev_handle, eDfuStatus* status, uint32_t *delay, eDfuState* state, uint8_t *i_string);
eDfuError dfu_CLRSTATUS(libusb_device_handle** dev_handle);
eDfuError dfu_GETSTATE(libusb_device_handle** dev_handle, uint8_t* buf);
eDfuError dfu_ABORT(libusb_device_handle** dev_handle);

eDfuError dfu_wait_for_state(libusb_device* dev, libusb_device_handle** dev_handle, eDfuState required_state);

#ifdef __cplusplus
}
#endif

#endif	// __IS_BOOTLOADER_DFU_H
