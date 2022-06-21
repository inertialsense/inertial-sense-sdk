/**
 * @file ISBootloaderCommon.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating embedded systems
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_COMMON_H
#define __IS_BOOTLOADER_COMMON_H

#include "ISBootloaderTypes.h"
#include "ISBootloaderDfu.h"
#include "inertialSenseBootLoader.h"

#ifdef __cplusplus
extern "C" {

// TODO: namespace
// TODO: api version constant
// TODO: winapi macros

#endif

is_device_context* is_create_context(
    is_device_handle* handle, 
    is_device_match_properties* match_props,
    is_firmware_settings* firmware,
    int baud_rate,
    is_verification_style verify,
    pfnBootloadProgress upload_cb,
    pfnBootloadProgress verify_cb,
    pfnBootloadStatus info_cb,
    void* user_data
);
void is_destroy_context(is_device_context* ctx);

is_operation_result is_check_version(is_device_context* ctx);
is_operation_result is_jump_to_bootloader(is_device_context* ctx);

/**
 * @brief Write flash to device
 * 
 * @param context setup struct of type is_device_context
 */
void is_update_flash(void* context);

#ifdef __cplusplus
}
#endif

#endif // __IS_BOOTLOADER_COMMON_H
