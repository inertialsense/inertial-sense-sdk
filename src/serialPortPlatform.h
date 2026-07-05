/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_SERIALPORT_PLATFORM_H
#define __IS_SERIALPORT_PLATFORM_H

#include "serialPort.h"

#ifdef __cplusplus
extern "C" {
#endif

// zero the struct then assign function pointers for common platforms such as Windows
// returns non-zero if success, 0 if platform not implemented
int serialPortPlatformInit(port_handle_t port);

// SN-8239: baud-rate helpers (POSIX). serialPortStandardBaudRate() returns the termios Bxxx constant
// for a known standard rate or 0 if the rate must use the custom path; serialPortBaudRateSupported()
// returns 1 for any rate in (0, SERIAL_PORT_BAUDRATE_MAX]. Exposed (non-static) for unit testing.
int serialPortStandardBaudRate(int baudRate);
int serialPortBaudRateSupported(int baudRate);

#if defined(__linux__)
// SN-8239: set an arbitrary custom baud rate on an open fd via termios2/BOTHER (see serialPortLinuxCustomBaud.c).
int serialPortSetCustomBaudLinux(int fd, int baudRate);
#endif

#ifdef __cplusplus
}
#endif

#endif // __IS_SERIALPORT_PLATFORM_H
