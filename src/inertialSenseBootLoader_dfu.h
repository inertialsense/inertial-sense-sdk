/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __INERTIALSENSEBOOTLOADER_DFU_H
#define __INERTIALSENSEBOOTLOADER_DFU_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <libusb.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>

#include "uins_types.h"
#include "uins_log.h"

#include "dfu_types.h"
#include "dfu_portable.h"
#include "dfu.h"
#include "dfu_file.h"
#include "dfu_load.h"
#include "dfu_util.h"
#include "dfuse.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum uins5_descriptor_enum
{
  UINS5_DESCRIPTOR_VENDOR_ID = 0x0483,
  UINS5_DESCRIPTOR_PRODUCT_ID = 0xdf11
} uins5_descriptor;

typedef enum uins5_dfu_interface_alternatives_enum
{
  UINS5_DFU_INTERFACE_ALTERNATIVE_FLASH    = 0, // @Internal Flash  /0x08000000/0256*0002Kg
  UINS5_DFU_INTERFACE_ALTERNATIVE_OPTIONS  = 1, // @Option Bytes  /0x1FFF7800/01*040 e
  UINS5_DFU_INTERFACE_ALTERNATIVE_OTP      = 2, // @OTP Memory /0x1FFF7000/01*0001Ke
  UINS5_DFU_INTERFACE_ALTERNATIVE_FEATURES = 3  // @Device Feature/0xFFFF0000/01*004 e
} uins5_dfu_interface_alternatives;

void uinsProbeDfuDevices(uins_device_uri_list* uri_list, uins_list_devices_callback_fn callback_fn);

// WIP: still need to figure out final form of input parameters, should not be dfu_config
int uinsBootloadFileExDfu(const uins_device_context const * context, struct dfu_config config);

#ifdef __cplusplus
}
#endif

#endif	// __INERTIALSENSEBOOTLOADER_DFU_H
