/**
 * @file ISBootloaderSAMBA.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating ISB (Inertial Sense Bootloader)
 *  images using the SAM-BA protocol.
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_SAMBA_H
#define __IS_BOOTLOADER_SAMBA_H

#include "ISBootloaderTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Flash the bootloader to the device via SAM-BA
 * 
 * @param ctx a fully populated device context  
 * @return is_operation_result 
 */
is_operation_result is_samba_flash(is_device_context* ctx);

is_operation_result is_samba_init(is_device_context* ctx);

is_operation_result is_samba_boot_from_flash(is_device_context* ctx);
is_operation_result is_samba_reset(is_device_context* ctx);

#ifdef __cplusplus
}
#endif

#endif	// __IS_BOOTLOADER_SAMBA_H
