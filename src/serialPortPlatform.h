/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file serialPortPlatform.h
 * @brief Platform-specific initialization and baud-rate helpers backing the serialPort.h function-pointer API.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef __IS_SERIALPORT_PLATFORM_H
#define __IS_SERIALPORT_PLATFORM_H

#include "serialPort.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Zeros the serial_port_t struct then assigns the function-pointer table for the current
 * platform (e.g. Windows, Linux, macOS).
 * @param port the port to initialize
 * @return non-zero if success, 0 if the current platform is not implemented
 */
int serialPortPlatformInit(port_handle_t port);

#if !defined(_WIN32)
/**
 * SN-8239: returns the termios Bxxx constant for a known standard baud rate. Declared only
 * off-Windows to match its definition (the Windows branch of serialPortPlatform.c does not
 * define it), so a stray Windows caller fails at compile time rather than link time. Exposed
 * (non-static) for unit testing.
 * @param baudRate the baud rate to look up
 * @return the termios Bxxx constant for baudRate, or 0 if the rate must use the custom baud path
 */
int serialPortStandardBaudRate(int baudRate);

/**
 * SN-8239: reports whether baudRate can be opened, either via a standard termios Bxxx constant or
 * via the platform custom-rate path. Declared only off-Windows to match its definition (the Windows
 * branch of serialPortPlatform.c does not define it), so a stray Windows caller fails at compile
 * time rather than link time. Exposed (non-static) for unit testing.
 * @param baudRate the baud rate to check
 * @return 1 if baudRate is in (0, SERIAL_PORT_BAUDRATE_MAX], 0 otherwise
 */
int serialPortBaudRateSupported(int baudRate);
#endif

#if defined(__linux__)
/**
 * SN-8239: sets an arbitrary custom baud rate on an open fd via termios2/BOTHER (see
 * serialPortLinuxCustomBaud.c). Used for rates that have no standard termios Bxxx constant.
 * @param fd the open file descriptor of the serial device
 * @param baudRate the desired baud rate
 * @return 0 on success, -1 on failure (errno set by the underlying ioctl)
 */
int serialPortSetCustomBaudLinux(int fd, int baudRate);
#endif

#ifdef __cplusplus
}
#endif

#endif // __IS_SERIALPORT_PLATFORM_H
