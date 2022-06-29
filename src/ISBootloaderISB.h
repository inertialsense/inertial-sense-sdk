/**
 * @file ISBootloaderISB.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating application images 
 *  using ISB (Inertial Sense Bootloader) protocol
 *  
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_H
#define __IS_BOOTLOADER_H

#include "ISBootloaderTypes.h"
#include "ISBootloader.h"

#ifdef __cplusplus
extern "C" {
#endif

void is_isb_restart_rom(serial_port_t* s);
void is_isb_restart(serial_port_t* s);
is_operation_result is_isb_flash(is_device_context* ctx);
is_operation_result is_isb_get_version_from_file(const char* filename, uint8_t* major, char* minor);
is_operation_result is_isb_reboot_to_app(serial_port_t* port);
is_operation_result is_isb_get_version(is_device_context* ctx);
is_operation_result is_isb_enable(is_device_context* ctx, const char* enable_cmd);

#ifdef __cplusplus
}
#endif

#endif // __IS_BOOTLOADER_H
