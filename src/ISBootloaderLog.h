/**
 * @file ISBootloaderLog.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense logging functions for bootloaders
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

#ifndef __IS_BOOTLOADER_LOG_H
#define __IS_BOOTLOADER_LOG_H

#include <stdio.h>
#include <stdarg.h>

#include <libusb.h>

#include "ISBootloaderTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void uinsLog(const uins_device_interface_log_level message_level, const uins_device_context const * context, const int error_code, const char * error_message);
void uinsLogError(const uins_device_context const * context, const int error_code, const char * error_message);
void uinsLogWarn(const uins_device_context const * context, const int error_code, const char * error_message);

void uinsLogDebug(const uins_device_context const * context, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif	// __IS_BOOTLOADER_LOG_H

