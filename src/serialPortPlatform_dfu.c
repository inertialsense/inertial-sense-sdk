/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "serialPort_dfu.h"
#include "serialPortPlatform_dfu.h"
#include "ISConstants.h"

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

// cygwin defines FIONREAD in socket.h instead of ioctl.h
#ifndef FIONREAD
#include <sys/socket.h>
#endif

#if PLATFORM_IS_APPLE

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#include <IOKit/IOBSD.h>

#endif

#ifndef error_message
#define error_message printf
#endif

#ifndef B460800
#define B460800 460800
#endif
#ifndef B921600
#define B921600 921600
#endif
#ifndef B1500000
#define B1500000 1500000
#endif
#ifndef B2000000
#define B2000000 2000000
#endif
#ifndef B2500000
#define B2500000 2500000
#endif
#ifndef B3000000
#define B3000000 3000000
#endif

#endif

typedef struct
{
    int blocking;

#if PLATFORM_IS_WINDOWS

    void* platformHandle;
    OVERLAPPED ovRead;
    OVERLAPPED ovWrite;

#else

    int fd;

#endif

} serialPortHandle;

static int serialPortOpenPlatform(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
    return 0;
}

static int serialPortIsOpenPlatform(serial_port_t* serialPort)
{
    return 0;
}

static int serialPortClosePlatform(serial_port_t* serialPort)
{
    return 0;
}

static int serialPortFlushPlatform(serial_port_t* serialPort)
{
    return 0;
}

#if PLATFORM_IS_WINDOWS

static int serialPortReadTimeoutPlatformWindowsDfu(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    return 0;
}

#else

static int serialPortReadTimeoutPlatformLinuxDfu(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    return 0;
}

#endif

static int serialPortReadTimeoutPlatformDfu(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
}

static int serialPortAsyncReadPlatformDfu(serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion completion)
{
    return 1;
}

static int serialPortWritePlatformDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
    return 0;
}

static int serialPortGetByteCountAvailableToReadPlatformDfu(serial_port_t* serialPort)
{
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;


#if PLATFORM_IS_WINDOWS

    COMSTAT commStat;
    if (ClearCommError(handle->platformHandle, 0, &commStat))
    {
        return commStat.cbInQue;
    }
    return 0;

#else

    int bytesAvailable;
    ioctl(handle->fd, FIONREAD, &bytesAvailable);
    return bytesAvailable;

#endif

}

static int serialPortGetByteCountAvailableToWritePlatformDfu(serial_port_t* serialPort)
{
    // serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    (void)serialPort;

    return 65536;

    /*
    int bytesUsed;
    struct serial_struct serinfo;
    memset(&serinfo, 0, sizeof(serial_struct));
    ioctl(handle->fd, TIOCGSERIAL, &serinfo);
    ioctl(handle->fd, TIOCOUTQ, &bytesUsed);
    return serinfo.xmit_fifo_size - bytesUsed;
    */
}

static int serialPortSleepPlatformDfu(int sleepMilliseconds)
{
#if PLATFORM_IS_WINDOWS

    Sleep(sleepMilliseconds);

#else

    usleep(sleepMilliseconds * 1000);

#endif

    return 1;
}

int serialPortPlatformInitDfu(serial_port_t* serialPort)
{
    return 0;
}
