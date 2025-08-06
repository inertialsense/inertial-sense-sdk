/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "serialPort.h"
#include "serialPortPlatform.h"
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
#include <sys/file.h>
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

static int serialPortSleepPlatform(int sleepMilliseconds);
static int serialPortFlushPlatform(serial_port_t* serialPort);
static int serialPortDrainPlatform(serial_port_t* serialPort);
static int serialPortReadTimeoutPlatform(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds);
// static int serialPortReadTimeoutPlatformLinux(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds);


// #define DEBUG_COMMS   // Enabling this will cause all traffic to be printed on the console, with timestamps and direction (<< = received, >> = transmitted).
#ifdef DEBUG_COMMS
#define IS_PRINTABLE(n) (((n >= 0x20) && (n <= 0x7E)) || ((n >= 0xA1) && (n <= 0xFF)))
static inline void debugDumpBuffer(const char* prefix, const unsigned char* buffer, int len) {
    if (len > 0) {
        struct timeval start;
        gettimeofday(&start, NULL);
        printf("%ld.%03d: %s", start.tv_sec, (uint16_t) (start.tv_usec / 1000), prefix);
        for (int i = 0; i < len; i++)
            printf(" %02x", buffer[i]);

        int linePos = 16 + strlen(prefix) + (len * 3);
        printf("%*c", 80 - linePos, ' ');

        for (int i = 0; i < len; i++)
            printf("%c", IS_PRINTABLE(buffer[i]) ? buffer[i] : 0xB7);

        printf("\n");
    }
}
#else
    #define error_message(...)
    #define debugDumpBuffer(...)
#endif
#ifndef error_message
    #define error_message printf
#endif


#if PLATFORM_IS_WINDOWS

#define WINDOWS_OVERLAPPED_BUFFER_SIZE 8192

typedef struct
{
    OVERLAPPED ov;
    pfnSerialPortAsyncReadCompletion externalCompletion;
    serial_port_t* serialPort;
    unsigned char* buffer;
} readFileExCompletionStruct;

static void CALLBACK readFileExCompletion(DWORD errorCode, DWORD bytesTransferred, LPOVERLAPPED ov)
{
    readFileExCompletionStruct* c = (readFileExCompletionStruct*)ov;
    c->externalCompletion(c->serialPort, c->buffer, bytesTransferred, errorCode);
    free(c);
}

#else

static int validate_baud_rate(int baudRate)
{
    switch (baudRate)
    {
    default:      return 0;
    case 300:     return B300;
    case 600:     return B600;
    case 1200:    return B1200;
    case 2400:    return B2400;
    case 4800:    return B4800;
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    }
}

static int configure_serial_port(int fd, int baudRate)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) 
    {
        error_message("error getting tty settings: tcgetattr");
        return -1;
    }

    // Restrict baudrate to predefined values (standard and high speed)
    baudRate = validate_baud_rate(baudRate);    
    if (baudRate == 0)
    {
        error_message("error invalid baudrate");
        return -1;
    }

    // Set Baud Rate
#if PLATFORM_IS_APPLE

    // HACK: Mac will not allow higher baud rate until after set lower valid rate: e.g. 230400
    cfsetospeed(&tty, 230400);
    cfsetispeed(&tty, 230400);
    // Now baud rate can be set higher than 230400
    if (ioctl(fd, IOSSIOSPEED, &baudRate) == -1)
    {
        error_message("error %d from ioctl IOSSIOSPEED", errno);
    }

#else

    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

#endif

    // Set 8N1 (8 data bits, No parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;                     // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;                     // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;                      // Clear all bits that set the data size
    tty.c_cflag |= CS8;                         // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;                    // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;              // Turn on READ & ignore model ctrl lines (CLOCAL = 1)

    // Set in non-canonical mode
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                       // Disable echo
    tty.c_lflag &= ~ECHOE;                      // Disable erasure
    tty.c_lflag &= ~ECHONL;                     // Disable new-line echo
    tty.c_lflag &= ~ISIG;                       // Disable interpretation of INTR, QUIT and SUSP
    // No line processing
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    tty.c_lflag = 0;

    // Disable input processing options (raw mode)
    tty.c_iflag &= ~IGNBRK;                     // Disable break processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // Turn off xon/xoff software flow ctrl
    tty.c_iflag &= ~(ICRNL | INLCR);            // Disable any special handling of received bytes
    // No convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    tty.c_iflag = 0;

    // Disable output processing options (raw mode)
    tty.c_oflag &= ~OPOST;                      // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;                      // Prevent conversion of newline to carriage return/line feed
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    tty.c_oflag = 0;

    // Set the timeout and minimum characters.  Read doesn't block
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
    {
        error_message("error saving tty settings: tcsetattr");
        return -1;
    }

    return 0;
}

// Set the serial port to non-blocking mode so read() and write() return immediately not waiting for hardware. Use modern O_NONBLOCK instead of legacy O_NDELAY.
// Because of non-blocking mode, we have to retry serial write() to handle partial writes until all data received by the OS.
int set_nonblocking(int fd) 
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) 
    {
        error_message("error fcntl F_GETFL");
        return -1;
    }

    flags |= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, flags) == -1) 
    {
        error_message("error setting O_NONBLOCK");
        return -1;
    }

    return 0;
}

#endif

// Return 1 on success, 0 on failure
static int serialPortOpenPlatform(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
    if (serialPort->handle != 0)
    {
        // FIXME: Should we be closing the port and then reopen??
        // serialPortClose(serialPort);
        // already open
        return 0;
    }

    serialPortSetPort(serialPort, port);

#if PLATFORM_IS_WINDOWS

    void* platformHandle = 0;
    platformHandle = CreateFileA(serialPort->port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, blocking ? FILE_FLAG_OVERLAPPED : 0, 0);
    if (platformHandle == INVALID_HANDLE_VALUE)
    {
        // don't modify the originally requested port value, just create a new value that Windows needs for COM10 and above
        char tmpPort[MAX_SERIAL_PORT_NAME_LENGTH];
        sprintf_s(tmpPort, sizeof(tmpPort), "\\\\.\\%s", port);
        platformHandle = CreateFileA(tmpPort, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, blocking ? FILE_FLAG_OVERLAPPED : 0, 0);
        if (platformHandle == INVALID_HANDLE_VALUE)
        {
            return 0;
        }
    }

    DCB serialParams;
    serialParams.DCBlength = sizeof(DCB);
    if (GetCommState(platformHandle, &serialParams))
    {
        serialParams.BaudRate = baudRate;
        serialParams.ByteSize = DATABITS_8;
        serialParams.StopBits = ONESTOPBIT;

        switch(serialPort->options & OPT_PARITY_MASK)
        {
        case OPT_PARITY_EVEN:
            serialParams.Parity = EVENPARITY;
            break;
        case OPT_PARITY_ODD:    
            serialParams.Parity = ODDPARITY;
            break;
        case OPT_PARITY_NONE:
        default:
            serialParams.Parity = NOPARITY;
            break;
        }

        serialParams.fBinary = 1;
        serialParams.fInX = 0;
        serialParams.fOutX = 0;
        serialParams.fAbortOnError = 0;
        serialParams.fNull = 0;
        serialParams.fErrorChar = 0;
        serialParams.fDtrControl = DTR_CONTROL_ENABLE;
        serialParams.fRtsControl = RTS_CONTROL_ENABLE;
        if (!SetCommState(platformHandle, &serialParams))
        {
            serialPortClose(serialPort);
            return 0;
        }
    }
    else
    {
        serialPortClose(serialPort);
        return 0;
    }
    COMMTIMEOUTS timeouts = { (blocking ? 1 : MAXDWORD), (blocking ? 1 : 0), (blocking ? 1 : 0), (blocking ? 1 : 0), (blocking ? 10 : 0) };
    if (!SetCommTimeouts(platformHandle, &timeouts))
    {
        serialPortClose(serialPort);
        return 0;
    }
    serialPortHandle* handle = (serialPortHandle*)calloc(sizeof(serialPortHandle), 1);
    handle->blocking = blocking;
    handle->platformHandle = platformHandle;
    if (blocking)
    {
        handle->ovRead.hEvent = CreateEvent(0, 1, 0, 0);
        handle->ovWrite.hEvent = CreateEvent(0, 1, 0, 0);
    }
    serialPort->handle = handle;

#else

    int fd = open(port, O_RDWR | O_NOCTTY);     // enable read/write and disable flow control
    if (fd < 0)
    {
        error_message("[%s] open():: Error opening port: %d\n", port, errno);
        serialPort->errorCode = errno;
        return 0;
    }

    if (configure_serial_port(fd, baudRate) != 0) 
    {
        error_message("[%s] open():: Error configuring port: %d\n", port, errno);
        serialPort->errorCode = errno;
        return 0;
    }

    // Disable blocking port reads and writes.
    if (set_nonblocking(fd) != 0) 
    {
        close(fd);
        return 0;
    }

    ioctl(fd, TIOCEXCL);            // Exclusive Access Mode: prevent other processes from opening the port while its open
    flock(fd, LOCK_EX | LOCK_NB);   // Exclusive & Non-Blocking Lock: prevent other process read/write of the fd file  

    serialPortHandle* handle = (serialPortHandle*)calloc(sizeof(serialPortHandle), 1);
    handle->fd = fd;
    handle->blocking = blocking;
    serialPort->handle = handle;

    // we're doing a quick and dirty check to make sure we can even attempt to read data successfully.  Some bad devices will fail here if they aren't initialized correctly
    uint8_t tmp;
    if (serialPortReadTimeoutPlatform(serialPort, &tmp, 1, 10) < 0) {
        if (serialPort->errorCode == ENOENT) {
            serialPortClose(serialPort);
            return 0;
        }
    }

#endif

    return 1;	// success
}

static int serialPortIsOpenPlatform(serial_port_t* serialPort)
{
    if (!serialPort->handle)
        return 0;

#if PLATFORM_IS_WINDOWS

    DCB serialParams;
    serialParams.DCBlength = sizeof(DCB);
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    return GetCommState(handle->platformHandle, &serialParams);

#else

    struct stat sb;
    if (fstat(((serialPortHandle*)serialPort->handle)->fd, &sb) != 0) {
        serialPort->errorCode = errno;
        return 0;
    }
    return 1; // return success
#endif

}

static int serialPortClosePlatform(serial_port_t* serialPort)
{
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
    {
        // not open, no close needed
        return 0;
    }

    // When closing, let's flush any pending data, as this could potentially hang the close()
    serialPortFlushPlatform(serialPort);

#if PLATFORM_IS_WINDOWS

    //DWORD dwRead = 0;
    //DWORD error = 0;

    CancelIo(handle->platformHandle);
    //GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1);
    /*if ((error = GetLastError()) != ERROR_SUCCESS)
    {
        while (1) {}
    }*/
    if (handle->blocking)
    {
        CloseHandle(handle->ovRead.hEvent);
        CloseHandle(handle->ovWrite.hEvent);
    }
    CloseHandle(handle->platformHandle);
    handle->platformHandle = 0;

#else

    close(handle->fd);
    handle->fd = 0;

#endif

    free(handle);
    serialPort->handle = 0;

    return 1;
}

static int serialPortFlushPlatform(serial_port_t* serialPort)
{
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;

#if PLATFORM_IS_WINDOWS

    if(!FlushFileBuffers(handle->platformHandle))
    {
        return 0;
    }

#else

    if (tcflush(handle->fd, TCIOFLUSH) < 0)
        serialPort->errorCode = errno;

#endif

    return 1;
}

static int serialPortDrainPlatform(serial_port_t* serialPort)
{
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;

#if PLATFORM_IS_WINDOWS

    if(!FlushFileBuffers(handle->platformHandle))
    {
        return 0;
    }

#else

    if (tcdrain(handle->fd) < 0)
        serialPort->errorCode = errno;

#endif

    return 1;
}


#if PLATFORM_IS_WINDOWS

static int serialPortReadTimeoutPlatformWindows(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    if (readCount < 1)
    {
        return 0;
    }

    DWORD dwRead = 0;
    int totalRead = 0;
    ULONGLONG startTime = GetTickCount64();
    do
    {
        if (ReadFile(handle->platformHandle, buffer + totalRead, readCount - totalRead, &dwRead, handle->blocking ? &handle->ovRead : 0))
        {
            if (handle->blocking)
            {
                GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1);
            }
            totalRead += dwRead;
        }
        else if (handle->blocking)
        {
            DWORD dwRes = GetLastError();
            if (dwRes == ERROR_IO_PENDING)
            {
                dwRes = WaitForSingleObject(handle->ovRead.hEvent, _MAX(1, timeoutMilliseconds - (int)(GetTickCount64() - startTime)));
                switch (dwRes)
                {
                case WAIT_OBJECT_0:
                    if (!GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 0))
                    {
                        CancelIo(handle->platformHandle);
                    }
                    else
                    {
                        totalRead += dwRead;
                    }
                    break;

                default:
                    // cancel io and just take whatever was in the buffer
                    CancelIo(handle->platformHandle);
                    GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1);
                    totalRead += dwRead;
                    break;
                }
            }
            else
            {
                CancelIo(handle->platformHandle);
            }
        }

        if (handle->blocking && totalRead < readCount && timeoutMilliseconds > 0)
        {
            Sleep(1);
        }
    }
    while (totalRead < readCount && GetTickCount64() - startTime < timeoutMilliseconds);
    return totalRead;
}

#else

static int serialPortReadTimeoutPlatformLinux(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    int totalRead = 0;
    int dtMs;
    int n;
    struct timeval start, curr;
    if (timeoutMilliseconds > 0)
    {
        gettimeofday(&start, NULL);
    }

    while (1)
    {
        if (timeoutMilliseconds > 0)
        {
            struct pollfd fds[1];
            fds[0].fd = handle->fd;
            fds[0].events = POLLIN;
            int pollrc = poll(fds, 1, timeoutMilliseconds);
            if (pollrc <= 0 || !(fds[0].revents & POLLIN))
            {
                if (fds[0].revents & POLLERR) {
                    return -1; // more than a timeout occurred.
                }
                break;
            }
        }
        n = read(handle->fd, buffer + totalRead, readCount - totalRead);
        if (n <= -1)
        {
            if ((errno != EAGAIN) && (errno != EWOULDBLOCK)) {
                // error_message("Error reading from file %d : %s (%d)\n", handle->fd, strerror(errno), errno);
            }
            return -1;
        }
        else if (n != -1)
        {
            totalRead += n;
        }

        if (timeoutMilliseconds > 0 && totalRead < readCount)
        {
            gettimeofday(&curr, NULL);
            dtMs = ((curr.tv_sec - start.tv_sec) * 1000) + ((curr.tv_usec - start.tv_usec) / 1000);
            if (dtMs >= timeoutMilliseconds)
            {
                break;
            }

            // try for another loop around with a lower timeout
            timeoutMilliseconds = _MAX(0, timeoutMilliseconds - dtMs);
        }
        else
        {
            break;
        }
    }
    debugDumpBuffer("<< ", buffer, totalRead);
    return totalRead;
}

#endif

static int serialPortReadTimeoutPlatform(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENODEV;
        return -1;
    }

    if (timeoutMilliseconds < 0)
    {
        timeoutMilliseconds = (handle->blocking ? SERIAL_PORT_DEFAULT_TIMEOUT : 0);
    }

#if PLATFORM_IS_WINDOWS
    int result = serialPortReadTimeoutPlatformWindows(handle, buffer, readCount, timeoutMilliseconds);
#else
    int result = serialPortReadTimeoutPlatformLinux(handle, buffer, readCount, timeoutMilliseconds);
#endif

    if ((result < 0) && !((errno == EAGAIN) && !handle->blocking)) {
        error_message("Error reading from %s : %s (%d)\n", serialPort->port, strerror(errno), errno);
        serialPort->errorCode = errno;  // NOTE: If you are here looking at errno = -11 (EAGAIN) remember that if this is a non-blocking tty, returning EAGAIN on a read() just means there was no data available.
    } else
        serialPort->errorCode = 0; // clear any previous errorcode

    debugDumpBuffer("{{ ", buffer, result);
    return result;
}

static int serialPortAsyncReadPlatform(serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion completion)
{
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENODEV;
        return -1;
    }

#if PLATFORM_IS_WINDOWS

	readFileExCompletionStruct c;
	c.externalCompletion = completion;
    c.serialPort = serialPort;
    c.buffer = buffer;
    memset(&(c.ov), 0, sizeof(c.ov));

    if (!ReadFileEx(handle->platformHandle, buffer, readCount, (LPOVERLAPPED)&c, readFileExCompletion))
    {
        return 0;
    }

#else

    // no support for async, just call the completion right away
    int n = read(handle->fd, buffer, readCount);
    if (n < 0)
        serialPort->errorCode = errno;

    completion(serialPort, buffer, (n < 0 ? 0 : n), (n >= 0 ? 0 : n));

#endif

    return 1;
}


static int serialPortWritePlatform(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENODEV;
        return -1;
    }

#if PLATFORM_IS_WINDOWS

    DWORD dwWritten;
    if (!WriteFile(handle->platformHandle, buffer, writeCount, &dwWritten, handle->blocking ? &handle->ovWrite : 0))
    {
        DWORD result = GetLastError();
        if (result != ERROR_IO_PENDING)
        {
            CancelIo(handle->platformHandle);
            return 0;
        }
    }

    if (handle->blocking)
    {
        if (!GetOverlappedResult(handle->platformHandle, &handle->ovWrite, &dwWritten, 1))
        {
            CancelIo(handle->platformHandle);
            return 0;
        }
    }
    return dwWritten;


#else

    struct stat sb;
    errno = 0;
    if(fstat(((serialPortHandle*)serialPort->handle)->fd, &sb) != 0)
    {   // Serial port not open
        serialPort->errorCode = errno;
        return 0;
    }

    // Ensure all data is queued by OS for sending.  This step is necessary because of O_NONBLOCK non-blocking mode. 
    // Note that this only blocks for partial writes until the OS accepts all input data.  This does NOT block until 
    // the data is physically transmitted.
    int bytes_written = 0, retry = 0;
    while ((bytes_written < writeCount) && (retry < 10))
    {
        ssize_t result = write(handle->fd, buffer + bytes_written, writeCount - bytes_written);
        if (result < 0) 
        {
            if ((errno == EINTR) ||     // Interrupted by signal, continue writing
                (errno == EAGAIN) || (errno == EWOULDBLOCK))  // Non-blocking mode, and no data written, continue trying
            {
                serialPortSleepPlatform(1);
                retry++;
                continue;
            }
            // Other errors
            // error_message("serialPortWritePlatform() write error. ");
            return -1;
        }
        bytes_written += result;
    }

    if(handle->blocking)
    {   // Block until output data has been physically transmitted 
        int error = tcdrain(handle->fd);
        if (error != 0)
        {   // Drain error
            return 0;
        }
    }

    debugDumpBuffer(">> ", buffer, bytes_written);
    return bytes_written;

#endif

}

static int serialPortGetByteCountAvailableToReadPlatform(serial_port_t* serialPort)
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
    if (ioctl(handle->fd, FIONREAD, &bytesAvailable) < 0)
        serialPort->errorCode = errno;

    return bytesAvailable;

#endif

}

static int serialPortGetByteCountAvailableToWritePlatform(serial_port_t* serialPort)
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

static int serialPortSleepPlatform(int sleepMilliseconds)
{
#if PLATFORM_IS_WINDOWS

    Sleep(sleepMilliseconds);

#else

    usleep(sleepMilliseconds * 1000);

#endif

    return 1;
}

int serialPortPlatformInit(serial_port_t* serialPort)
{
	// very important - the serial port must be initialized to zeros
	memset(serialPort, 0, sizeof(serial_port_t));

    serialPort->pfnClose = serialPortClosePlatform;
    serialPort->pfnFlush = serialPortFlushPlatform;
    serialPort->pfnDrain = serialPortDrainPlatform;
    serialPort->pfnOpen = serialPortOpenPlatform;
    serialPort->pfnIsOpen = serialPortIsOpenPlatform;
    serialPort->pfnRead = serialPortReadTimeoutPlatform;
    serialPort->pfnAsyncRead = serialPortAsyncReadPlatform;
    serialPort->pfnWrite = serialPortWritePlatform;
    serialPort->pfnGetByteCountAvailableToRead = serialPortGetByteCountAvailableToReadPlatform;
    serialPort->pfnGetByteCountAvailableToWrite = serialPortGetByteCountAvailableToWritePlatform;
    serialPort->pfnSleep = serialPortSleepPlatform;
    return 0;
}
