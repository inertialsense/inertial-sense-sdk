/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "core/base_port.h"
#include "core/msg_logger.h"
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
#include <sys/socket.hh>
#endif

#if PLATFORM_IS_LINUX
#include <linux/serial.h>
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

/**
 * @brief Sleep for a specified number of milliseconds.
 * This function is a simple wrapper around the platform-specific sleep function.
 * On Windows, it uses `Sleep()`, and on other platforms, it uses `usleep()`.
 *
 * @param sleepMilliseconds The number of milliseconds to sleep.
 * @return int 1 on success.
 */
static int serialPortSleepPlatform(int sleepMilliseconds);
/**
 * @brief Flush the serial port.
 * This function clears the serial port's receive buffer.
 * On Windows, it uses `PurgeComm` with `PURGE_RXCLEAR`.
 * On other platforms, it uses `tcflush` with `TCIOFLUSH`.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortFlushPlatform(port_handle_t port);
/**
 * @brief Drain the serial port.
 * This function waits for all written data to be transmitted.
 * On Windows, it uses `PurgeComm` with `PURGE_TXCLEAR`.
 * On other platforms, it uses `tcdrain`.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortDrainPlatform(port_handle_t port);
/**
 * @brief Read from the serial port with a timeout.
 * This function reads a specified number of bytes from the serial port, with a timeout.
 * It is a wrapper around the platform-specific read functions.
 * If the timeout is negative, a default timeout is used.
 *
 * @param port The port handle.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @param timeoutMilliseconds The timeout in milliseconds.
 * @return int The number of bytes read, or -1 on error.
 */
static int serialPortReadTimeoutPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, int timeoutMilliseconds);
// static int serialPortReadTimeoutPlatformLinux(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds);


// #define DEBUG_COMMS   // Enabling this will cause all traffic to be printed on the console, with timestamps and direction (<< = received, >> = transmitted).
#ifdef DEBUG_COMMS
    #define debugDumpBuffer(...) static_log_buffer(__VA_ARGS__)
#else
    #define debugDumpBuffer(...)
#endif


#if PLATFORM_IS_WINDOWS

#define WINDOWS_OVERLAPPED_BUFFER_SIZE 8192

typedef struct {
    OVERLAPPED ov;
    pfnSerialPortAsyncReadCompletion externalCompletion;
    port_handle_t port;
    unsigned char* buffer;
} readFileExCompletionStruct;

/**
 * @brief Completion routine for ReadFileEx.
 * This function is called when an asynchronous read operation completes.
 * It calls the external completion function and frees the completion structure.
 *
 * @param errorCode The error code.
 * @param bytesTransferred The number of bytes transferred.
 * @param ov The overlapped structure.
 */
static void CALLBACK readFileExCompletion(DWORD errorCode, DWORD bytesTransferred, LPOVERLAPPED ov)
{
    readFileExCompletionStruct* c = (readFileExCompletionStruct*)ov;
    c->externalCompletion(c->port, c->buffer, bytesTransferred, errorCode);
    free(c);
}

#else

/**
 * @brief Validate the baud rate.
 * This function checks if the given baud rate is a standard, supported value.
 * It returns the corresponding termios speed flag for the given baud rate.
 * If the baud rate is not supported, it returns 0.
 *
 * @param baudRate The baud rate to validate.
 * @return int The validated baud rate, or 0 if invalid.
 */
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

/**
 * @brief Configure the serial port.
 * This function configures the serial port with the specified baud rate and other settings.
 * It sets the port to 8N1, disables flow control, and sets the port to raw mode.
 * On Apple platforms, there is a special hack to set high baud rates.
 *
 * @param fd The file descriptor.
 * @param baudRate The baud rate.
 * @return int 0 on success, -1 on failure.
 */
static int configure_serial_port(int fd, int baudRate)
{
    struct termios tty = {};

    if (tcgetattr(fd, &tty) != 0) 
    {
        log_error(IS_LOG_PORT, "config_serial_port():: tcgetattr() : error getting tty settings: %s (%d)", strerror(errno), errno);
        return -1;
    }

    // Restrict baudrate to predefined values (standard and high speed)
    baudRate = validate_baud_rate(baudRate);    
    if (baudRate == 0)
    {
        log_error(IS_LOG_PORT, "config_serial_port():: error invalid baudrate: %s (%d)", strerror(errno), errno);
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
        log_error(IS_LOG_PORT, "config_serial_port():: error %d from ioctl IOSSIOSPEED", errno);
    }

#else

    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    // Attempt to configure LOW_LATENCY for UART/serial ports - though doesn't appear to improve things much.
    struct serial_struct serial;
    ioctl(fd, TIOCGSERIAL, &serial);
    serial.flags |= ASYNC_LOW_LATENCY;
    serial.closing_wait = ASYNC_CLOSING_WAIT_NONE;
    ioctl(fd, TIOCSSERIAL, &serial);

#endif

    // Control Flags: Set 8N1 (8 data bits, No parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;                     // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;                     // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;                      // Clear all bits that set the data size
    tty.c_cflag |= CS8;                         // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;                    // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;              // Turn on READ & ignore model ctrl lines (CLOCAL = 1)

    // Local Modes: Set in non-canonical mode - Canonical mode is line-by-line processing; we want this DISABLED
    tty.c_lflag &= ~ICANON;                     // Disable Canonical Mode
    tty.c_lflag &= ~ECHO;                       // Disable echo
    tty.c_lflag &= ~ECHOE;                      // Disable erasure
    tty.c_lflag &= ~ECHONL;                     // Disable new-line echo
    tty.c_lflag &= ~ISIG;                       // Disable interpretation of INTR, QUIT and SUSP

    // Disable input processing options (raw mode)
    tty.c_iflag &= ~(IGNBRK | BRKINT);          // Disable break processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // Turn off xon/xoff software flow ctrl
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR );   // Disable any special handling of received bytes

    // Disable output processing options (raw mode)
    tty.c_oflag &= ~OPOST;                      // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;                      // Prevent conversion of newline to carriage return/line feed

    // Set the timeout and minimum characters.  Read doesn't block
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
    {
        log_error(IS_LOG_PORT, "config_serial_port():: tcsetattr() : error setting tty settings: %s (%d)", strerror(errno), errno);
        return -1;
    } else {
        // TODO: Note that tcsetattr() returns success if any of the requested changes could be successfully carried out.
        //  Therefore, when making multiple changes it may be necessary to follow this call with a further call to
        //  tcgetattr() to check that all changes have been performed successfully.
        //  Ie, we are probably not seeing this error as we think we are...

        struct termios new_tty = {};
        if (tcgetattr(fd, &new_tty) != 0)
        {
            log_error(IS_LOG_PORT, "config_serial_port():: tcgetattr() : error confirming successful setting of tty settings: %s (%d)", strerror(errno), errno);
            return -1;
        }
        if (memcmp(&new_tty, &tty, sizeof(struct termios)) != 0) {
            // what was set didn't match what was just read back (confirmation failed);
            // Let's figure out what didn't get set correctly...
            log_error(IS_LOG_PORT, "config_serial_port():: termios confirmation failed to match expected values:");
            if (new_tty.c_iflag != tty.c_iflag) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_iflag mismatch: expected: %x, actual: %x", tty.c_iflag, new_tty.c_iflag); }
            if (new_tty.c_oflag != tty.c_oflag) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_oflag mismatch: expected: %x, actual: %x", tty.c_oflag, new_tty.c_oflag); }
            if (new_tty.c_cflag != tty.c_cflag) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_cflag mismatch: expected: %x, actual: %x", tty.c_cflag, new_tty.c_cflag); }
            if (new_tty.c_lflag != tty.c_lflag) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_lflag mismatch: expected: %x, actual: %x", tty.c_lflag, new_tty.c_lflag); }
            for (int i = 0; i < 32; i++)
                if (new_tty.c_cc[i] != tty.c_cc[i]) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_cc[%d] mismatch: expected: %d, actual: %d", i, tty.c_cc[i], new_tty.c_cc[i]); }

            #if PLATFORM_IS_LINUX
            if (new_tty.c_line != tty.c_line) { log_error(IS_LOG_PORT, "config_serial_port():: setting c_line mismatch: expected: %d, actual: %d", tty.c_line, new_tty.c_line); }
            #endif

            return -1;
        }
    }

    return 0;
}

/**
 * @brief configures flow control on the specified file descriptor
 * @param fd
 * @param control
 * @return
 */
int set_flowcontrol(int fd, int control)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("error from tggetattr");
        return -1;
    }

    if(control) tty.c_cflag |= CRTSCTS;
    else tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("error setting term attributes");
        return -1;
    }
    return 0;
}

/**
 * @brief Set the serial port to non-blocking mode.
 * @brief Use modern O_NONBLOCK instead of legacy O_NDELAY. Because of non-blocking mode, we have to retry serial write() to handle partial writes until all data received by the OS.
 * This is done by getting the current flags, adding O_NONBLOCK, and then setting the new flags.
 * It also uses `flock` to get an exclusive, non-blocking lock on the file descriptor.
 *
 * @param fd The file descriptor.
 * @return int 0 on success, -1 on failure.
 */
int set_nonblocking(int fd) 
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) 
    {
        log_error(IS_LOG_PORT, "set_nonblocking():: error fcntl F_GETFL : %s (%d)", strerror(errno), errno);
        return -1;
    }

    flags |= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, flags) == -1) 
    {
        log_error(IS_LOG_PORT, "set_nonblocking():: error setting O_NONBLOCK : %s (%d)", strerror(errno), errno);
        return -1;
    }

    // Alternate method - this maybe redundant, but better to be safe, eh?
    flock(fd, LOCK_EX | LOCK_NB);

    return 0;
}

#endif

/**
 * @brief Open the serial port.
 * This function opens and configures the serial port with the specified parameters.
 * On Windows, it handles the `\\\\.\\` prefix for COM ports above 9.
 * It also sets up the port for overlapped I/O if non-blocking.
 * On other platforms, it opens the port and then configures it using `configure_serial_port`.
 *
 * @param port The port handle.
 * @param portName The name of the port to open.
 * @param baudRate The baud rate.
 * @param blocking 1 for blocking, 0 for non-blocking.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortOpenPlatform(port_handle_t port, const char* portName, int baudRate, int blocking)
{
    log_debug(IS_LOG_PORT, "serialPortOpenPlatform(%s:B%d, %sblocking) called.", portName, baudRate, blocking ? "" : "non-");

    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort->handle != 0)
    {
        // already open --  FIXME: Should we be closing the port and then reopen??
        // serialPortClose(serialPort);
        return 1;
    }

    serialPortSetName(port, portName);
    serialPort->baudRate = baudRate;

#if PLATFORM_IS_WINDOWS

    void* platformHandle = 0;
    platformHandle = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, !blocking ? FILE_FLAG_OVERLAPPED : 0, 0);
    serialPort->errorCode = (int)GetLastError();
    if (platformHandle == INVALID_HANDLE_VALUE)
    {
        // don't modify the originally requested port value, just create a new value that Windows needs for COM10 and above
        char tmpPort[MAX_SERIAL_PORT_NAME_LENGTH];
        sprintf_s(tmpPort, sizeof(tmpPort), "\\\\.\\%s", portName);
        platformHandle = CreateFileA(tmpPort, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, !blocking ? FILE_FLAG_OVERLAPPED : 0, 0);
        if (platformHandle == INVALID_HANDLE_VALUE)
        {
            serialPort->errorCode = errno;
            serialPort->error = strerror(errno);
            log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to open port: %s (%d)", portName, serialPort->error, serialPort->errorCode);
            return 0;
        }
    }

    // clear any pending data associated with the port, incoming or outgoing; and ignore any errors
    PurgeComm(platformHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);

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
            serialPort->errorCode = errno;
            serialPort->error = strerror(errno);
            log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to set COMM port parameters: %s (%d)", portName, serialPort->error, serialPort->errorCode);
            serialPortClose(port);
            return 0;
        }
    }
    else
    {
        serialPort->errorCode = errno;
        serialPort->error = strerror(errno);
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to retreive COMM port parameters: %s (%d)", portName, serialPort->error, serialPort->errorCode);
        serialPortClose(port);
        return 0;
    }

    COMMTIMEOUTS timeouts;
    if (!GetCommTimeouts(platformHandle, &timeouts))
    {
        serialPortClose(port);
        return 0;
    }

    // COMMTIMEOUTS timeouts = { (blocking ? 1 : MAXDWORD), (blocking ? 1 : 0), (blocking ? 1 : 0), (blocking ? 1 : 0), (blocking ? 10 : 0) };

    if (blocking)
    {
        // For blocking, ReadFile returns immediately with whatever is in the buffer.
        // The read loop in serialPortReadTimeoutPlatformWindows will poll.
        timeouts.ReadIntervalTimeout = 1;
        timeouts.ReadTotalTimeoutMultiplier = 1;
        timeouts.ReadTotalTimeoutConstant = 1;
    }
    else // non-blocking
    {
        // For non-blocking (overlapped), we want ReadFile to pend if no data is available.
        // Setting read timeouts to 0 disables them, and the wait is handled by WaitForSingleObject.
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 0;
    }

    // Set a reasonable write timeout for both modes. - In our baud rates of 921600, these numbers should be more then sufficient to ensure data is sent
    timeouts.WriteTotalTimeoutConstant = 30;            // a minimum timeout of 30 milliseconds, per write (but maybe less to actually send) - this should support large sends upto 2.7k if necessary
    timeouts.WriteTotalTimeoutMultiplier = 0;           // plus an additional 0 milliseconds per byte sent

    if (!SetCommTimeouts(platformHandle, &timeouts))
    {
        serialPort->errorCode = errno;
        serialPort->error = strerror(errno);
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform() failed to configure COMM port timeouts: %s (%d)", portName, serialPort->error, serialPort->errorCode);
        serialPortClose(port);
        return 0;
    }

    serialPortHandle* handle = (serialPortHandle*)calloc(sizeof(serialPortHandle), 1);
    handle->blocking = blocking;
    handle->platformHandle = platformHandle;
    if (!blocking)
    {
        handle->ovRead.hEvent = CreateEvent(0, 1, 0, 0);
        handle->ovWrite.hEvent = CreateEvent(0, 1, 0, 0);
    }
    serialPort->handle = handle;

#else

    int fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);     // enable read/write and disable flow control
    if (fd < 0)
    {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform():: Error opening port: %s (%d)", portName, serialPort->error, serialPort->errorCode);
        return 0;
    }

    if (configure_serial_port(fd, baudRate) != 0)
    {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform():: Error configuring port: %s (%d)", port, serialPort->error, serialPort->errorCode);
        return 0;
    }

    ioctl(fd, TIOCEXCL);            // Exclusive Access Mode: prevent other processes from opening the port while its open
    flock(fd, LOCK_EX | LOCK_NB);   // Exclusive & Non-Blocking Lock: prevent other process read/write of the fd file

    // Disable blocking port reads and writes.
    if (set_nonblocking(fd) != 0) 
    {
        close(fd);
        return 0;
    }

    serialPortHandle* handle = (serialPortHandle*)calloc(sizeof(serialPortHandle), 1);
    handle->fd = fd;
    handle->blocking = blocking;
    serialPort->handle = handle;

    // we're doing a quick and dirty check to make sure we can even attempt to read data successfully.  Some bad devices will fail here if they aren't initialized correctly
    uint8_t tmp = 0;
    if (serialPortReadTimeoutPlatform(port, &tmp, 1, 10) < 0) {
        if (serialPort->errorCode == ENOENT) {
            serialPortClose(port);
            return 0;
        }
    }

#endif

    return 1;    // success
}

/**
 * @brief Check if the serial port is open.
 * This function checks if the serial port is open and valid.
 * On Windows, it uses `GetCommState`.
 * On other platforms, it uses `fstat`.
 *
 * @param port The port handle.
 * @return int 1 if open, 0 if not.
 */
static int serialPortIsOpenPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (!serialPort->handle)
        return 0;

    log_more_debug(IS_LOG_PORT, "[%s] serialPortIsOpenPlatform() called.", portName(port));

#if PLATFORM_IS_WINDOWS

    DCB serialParams;
    serialParams.DCBlength = sizeof(DCB);
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    return GetCommState(handle->platformHandle, &serialParams);

#else

    struct stat sb;
    if (fstat(((serialPortHandle*)(serialPort->handle))->fd, &sb) != 0) {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        return 0;
    }
    return 1; // return success
#endif

}

/**
 * @brief Close the serial port.
 * This function closes the serial port and frees the associated handle.
 * On Windows, it cancels any pending I/O and closes the handle.
 * On other platforms, it simply closes the file descriptor.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortClosePlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
    {
        // not open, no close needed
        return 0;
    }

    log_debug(IS_LOG_PORT, "[%s] serialPortClosePlatform() called.", portName(port));

    #if PLATFORM_IS_WINDOWS

    //DWORD dwRead = 0;
    //DWORD error = 0;

    CancelIo(handle->platformHandle);
    //GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1);
    /*if ((error = GetLastError()) != ERROR_SUCCESS)
    {
        while (1) {}
    }*/
    if (!handle->blocking)
    {
        CloseHandle(handle->ovRead.hEvent);
        CloseHandle(handle->ovWrite.hEvent);
    }
    CloseHandle(handle->platformHandle);
    handle->platformHandle = 0;

#else

    // we need to do some extended checking here... It seems linux/posix close(fd) can block if data is in the TX buffer, but can't be sent
    set_flowcontrol(handle->fd, 0);
    if (tcflush(handle->fd, TCIOFLUSH) < 0) {
        // something bad happened...
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        log_error(IS_LOG_PORT, "[%s] serialPortClosePlatform():: Error flushing: %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
    }

    close(handle->fd);
    handle->fd = -1;

#endif

    free(serialPort->handle);
    serialPort->handle = 0;

    serialPortSleepPlatform(500);   // this is strange, but occasionally some processes can spam Open/Close requests - so just a little something to slow things down; generally close() shouldn't be called often.

    return 1;
}

/**
 * @brief Flush the serial port.
 * This function clears the serial port's receive buffer.
 * On Windows, it uses `PurgeComm` with `PURGE_RXCLEAR`.
 * On other platforms, it uses `tcflush` with `TCIOFLUSH`.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortFlushPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
    {
        // not open, no close needed
        return 0;
    }

    log_more_debug(IS_LOG_PORT, "[%s] serialPortFlushPlatform() called.", portName(port));

#if PLATFORM_IS_WINDOWS

    // Use PurgeComm to clear receive (RX) buffer.
    if (!PurgeComm(handle->platformHandle, PURGE_RXCLEAR))
    {
        serialPort->errorCode = errno;
        serialPort->error = strerror(errno);
        log_error(IS_LOG_PORT, "[%s] serialPortDrainPlatform():: Error draining: %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
        return 0;
    }

#else

    if (tcflush(handle->fd, TCIOFLUSH) < 0) {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        log_error(IS_LOG_PORT, "[%s] serialPortDrainPlatform():: Error draining: %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
    }

#endif

    return 1;
}

/**
 * @brief Drain the serial port.
 * This function waits for all written data to be transmitted.
 * On Windows, it uses `PurgeComm` with `PURGE_TXCLEAR`.
 * On other platforms, it uses `tcdrain`.
 *
 * @param port The port handle.
 * @return int 1 on success, 0 on failure.
 */
static int serialPortDrainPlatform(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (handle == 0)
    {
        // not open, no close needed
        return 0;
    }

    log_more_debug(IS_LOG_PORT, "[%s] serialPortDrainPlatform() called.", portName(port));

#if PLATFORM_IS_WINDOWS

    // Use PurgeComm to clear transmit (TX) buffer.
    if (!PurgeComm(handle->platformHandle, PURGE_TXCLEAR))
    {
        serialPort->errorCode = errno;
        serialPort->error = strerror(errno);
        log_error(IS_LOG_PORT, "[%s] serialPortDrainPlatform():: Error draining: %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
        return 0;
    }

#else

    if (tcdrain(handle->fd) < 0) {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        log_error(IS_LOG_PORT, "[%s] serialPortDrainPlatform():: Error draining: %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
    }

#endif

    return 1;
}


#if PLATFORM_IS_WINDOWS

/**
 * @brief Reads a specified number of bytes from the serial port with a timeout.
 * This function reads from the serial port, handling both blocking and non-blocking (overlapped) I/O.
 * For non-blocking I/O, it uses `WaitForSingleObject` to wait for the read to complete.
 * If the read times out, it cancels the I/O and returns the bytes read so far.
 *
 * @param handle   Handle to the COM port (must be opened with FILE_FLAG_OVERLAPPED).
 * @param buffer    Pointer to the destination buffer.
 * @param readCount   The number of bytes requested to read.
 * @param timeoutMilliseconds Maximum time to wait in milliseconds.
 * @return          The actual number of bytes read (may be less than readCount on timeout).
 */
static int serialPortReadTimeoutPlatformWindows(serialPortHandle* handle, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    if (readCount < 1)
    {
        return 0;
    }

    DWORD dwRes = 0;
    DWORD dwRead = 0;
    int totalRead = 0;
    ULONGLONG startTime = GetTickCount64();
    do {
        if (ReadFile(handle->platformHandle, buffer + totalRead, readCount - totalRead, &dwRead, !handle->blocking ? &handle->ovRead : 0)) {
            dwRes = GetLastError();
            if (!handle->blocking) {
                GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1);
            }
            totalRead += dwRead;
        } else if (!handle->blocking) {
            dwRes = GetLastError();
            if (dwRes == ERROR_IO_PENDING) {
                dwRes = WaitForSingleObject(handle->ovRead.hEvent, _MAX(5, timeoutMilliseconds - (int)(GetTickCount64() - startTime)));
                switch (dwRes) {
                    case WAIT_OBJECT_0:
                        if (!GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 1))
                            CancelIo(handle->platformHandle);
                        else
                            totalRead += dwRead;
                        break;
                    case WAIT_TIMEOUT:
                    default:
                        // cancel io and just take whatever was in the buffer
                        CancelIo(handle->platformHandle);
                        GetOverlappedResult(handle->platformHandle, &handle->ovRead, &dwRead, 0);
                        totalRead += dwRead;
                        break;
                }
            } else {
                CancelIo(handle->platformHandle);
            }
        }
    } while ((totalRead < readCount) && (GetTickCount64() - startTime < timeoutMilliseconds));

    return totalRead;
}

#else

/**
 * @brief Read from the serial port with a timeout on Linux.
 * This function reads from the serial port using `poll` to wait for data to become available.
 * It loops until the requested number of bytes are read or the timeout is reached.
 * It handles `EAGAIN` and `EWOULDBLOCK` errors by continuing to try and read.
 *
 * When a timeout occurs, this function will return the number of bytes received so far -
 * this may mean that the function returns 0 or a positive number less than readCount. This
 * is NOT an error condition, since zero or more bytes, had they been available, could have
 * been read.
 *
 * @param serialPort The serial port.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @param timeoutMilliseconds The timeout in milliseconds.
 * @return int The number of bytes read, or a PORT_ERROR__* code if an error occurred.
 */
static int serialPortReadTimeoutPlatformLinux(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMs)
{
    int totalRead = 0;
    int dtMs = 0;
    int n = 0;
    struct timeval start, curr;

    if (!serialPort || !serialPort->handle || !buffer)
        return PORT_ERROR__INVALID_PARAMETER;

    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    gettimeofday(&start, NULL);

    while (1) {
        if (timeoutMs > 0) {
            struct pollfd fds[1];
            fds[0].fd = handle->fd;
            fds[0].events = POLLIN;

            // we will poll, for upto timeoutMs for any number of bytes.
            int pollrc = poll(fds, 1, timeoutMs);
            if (pollrc <= 0 || !(fds[0].revents & POLLIN)) {
                if (fds[0].revents & POLLIN) {
                    // do nothing - we'll fall thru to the read() below...
                } else if (fds[0].revents & POLLERR) {
                    return PORT_ERROR__READ_FAILURE; // more than a timeout occurred.
                } else {
                    break;  // no data before timeout expired
                }
            }
        }

        if ((n = read(handle->fd, buffer + totalRead, readCount - totalRead)) < 0) {
            if ((errno != EAGAIN) && (errno != EWOULDBLOCK)) {
                serialPort->errorCode = errno;
                serialPort->error = strerror(errno);
                log_error(IS_LOG_PORT, "[%s] serialPortOpenPlatform():: Error reading from file %d : %s (%d)", serialPort->portName, handle->fd, serialPort->error, serialPort->errorCode);
            }
            return PORT_ERROR__TIMEOUT;
        } else if (n > 0) {
            totalRead += n;
        }

        if ((timeoutMs > 0) && (totalRead < readCount))
        {
            gettimeofday(&curr, NULL);
            dtMs = ((curr.tv_sec - start.tv_sec) * 1000) + ((curr.tv_usec - start.tv_usec) / 1000);
            if (dtMs >= timeoutMs)
            {
                break;
            }

            // try for another loop around with a lower timeout
            timeoutMs = _MAX(0, timeoutMs - dtMs);
        }
        else
        {
            break;
        }
    }
    // debugDumpBuffer("{{ ", buffer, totalRead);
    return totalRead;
}

#endif

/**
 * @brief Read from the serial port with a timeout.
 * This function reads a specified number of bytes from the serial port, with a timeout.
 * It is a wrapper around the platform-specific read functions.
 * If the timeout is negative, a default timeout is used.
 *
 * @param port The port handle.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @param timeoutMilliseconds The timeout in milliseconds.
 * @return int The number of bytes read, or -1 on error.
 */
static int serialPortReadTimeoutPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, int timeoutMs)
{
    log_bombastic(IS_LOG_PORT, "[%s] serialPortReadTimeoutPlatform() called.", portName(port));

    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENOENT;
        serialPort->error = "Internal port handle is NULL; Port is closed.";
        return PORT_ERROR__NOT_CONNECTED;
    }

    if (timeoutMs < 0)
    {
        timeoutMs = (handle->blocking ? SERIAL_PORT_DEFAULT_TIMEOUT : 0);
    }

#if PLATFORM_IS_WINDOWS
    int result = serialPortReadTimeoutPlatformWindows(handle, buffer, readCount, timeoutMs);
#else
    int result = serialPortReadTimeoutPlatformLinux(serialPort, buffer, readCount, timeoutMs);
#endif

    if ((result < 0) && !((errno == EAGAIN) && !handle->blocking)) {
        serialPort->errorCode = errno;  // NOTE: If you are here looking at errno = -11 (EAGAIN) remember that if this is a non-blocking tty, returning EAGAIN on a read() just means there was no data available.
        serialPort->error = strerror(serialPort->errorCode);
        log_error(IS_LOG_PORT, "[%s] serialPortReadTimeoutPlatform():: Error reading: %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
    } else {
        serialPort->errorCode = 0; // clear any previous errorcode
        serialPort->error = NULL;
    }

    log_bombastic(IS_LOG_PORT, "[%s] serialPortReadTimeoutPlatform() received %d bytes", portName(port), result);
    debugDumpBuffer("<< ", buffer, result);
    return result;
}

/**
 * @brief Read from the serial port.
 * This function is a convenience wrapper around `serialPortReadTimeoutPlatform` with a timeout of 0.
 * This means it will return immediately with any available data.
 *
 * @param port The port handle.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @return int The number of bytes read, or -1 on error.
 */
static int serialPortReadPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount) {
    return serialPortReadTimeoutPlatform(port, buffer, readCount, 0);
}

/**
 * @brief Asynchronously read from the serial port.
 * This function initiates an asynchronous read from the serial port.
 * On Windows, it uses `ReadFileEx` and a completion routine.
 * On other platforms, it performs a simple blocking read and calls the completion routine directly.
 *
 * @param port The port handle.
 * @param buffer The buffer to read into.
 * @param readCount The number of bytes to read.
 * @param completion The completion routine.
 * @return int 1 on success, -1 on failure.
 */
static int serialPortAsyncReadPlatform(port_handle_t port, unsigned char* buffer, unsigned int readCount, pfnSerialPortAsyncReadCompletion completion)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENODEV;
        serialPort->error = strerror(serialPort->errorCode);
        return -1;
    }

#if PLATFORM_IS_WINDOWS

    readFileExCompletionStruct c;
    c.externalCompletion = completion;
    c.port = port;
    c.buffer = buffer;
    memset(&(c.ov), 0, sizeof(c.ov));

    if (!ReadFileEx(handle->platformHandle, buffer, readCount, (LPOVERLAPPED)&c, readFileExCompletion))
    {
        return 0;
    }

#else

    // no support for async, just call the completion right away
    int n = read(handle->fd, buffer, readCount);
    if (n < 0) {
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
    }

    completion(port, buffer, (n < 0 ? 0 : n), (n >= 0 ? 0 : n));

#endif

    return 1;
}

/**
 * @brief Write to the serial port.
 * This function writes a buffer of data to the serial port.
 * On Windows, it handles overlapped I/O for non-blocking writes.
 * On other platforms, it retries on partial writes and handles `EINTR`, `EAGAIN`, and `EWOULDBLOCK` errors.
 *
 * @param port The port handle.
 * @param buffer The buffer to write from.
 * @param writeCount The number of bytes to write.
 * @return int The number of bytes written, or -1 on error.
 */
static int serialPortWritePlatform(port_handle_t port, const unsigned char* buffer, unsigned int writeCount)
{
    log_bombastic(IS_LOG_PORT, "[%s] serialPortWritePlatform() called.", portName(port));

    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;
    if (!handle) {
        serialPort->errorCode = ENODEV;
        serialPort->error = strerror(serialPort->errorCode);
        return -1;
    }

#if PLATFORM_IS_WINDOWS

    DWORD dwWritten;
    if (!WriteFile(handle->platformHandle, buffer, writeCount, &dwWritten, !handle->blocking ? &handle->ovWrite : 0))
    {
        DWORD result = GetLastError();
        if (result != ERROR_IO_PENDING)
        {
            serialPort->errorCode = errno;
            serialPort->error = strerror(serialPort->errorCode);
            log_error(IS_LOG_PORT, "[%s] serialPortWrite():: Error writing: %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
            CancelIo(handle->platformHandle);
            if (result == ERROR_NOT_SAME_DEVICE) {
                // this should probably be expanded to include other likely errors, but...
                // this indicates the handle is invalid. The port should be closed and invalidated.
                portClose(port);
                portInvalidate(port);
            }
            return 0;
        }
    }

    if (!handle->blocking)
    {
        if (!GetOverlappedResult(handle->platformHandle, &handle->ovWrite, &dwWritten, 1))
        {
            serialPort->errorCode = errno;
            serialPort->error = strerror(serialPort->errorCode);
            log_error(IS_LOG_PORT, "[%s] serialPortWrite():: Error fetching 'overlapped result': %s (%d)", portName(port), serialPort->error, serialPort->errorCode);
            DWORD result = GetLastError();  // read this before we call CancelIo
            CancelIo(handle->platformHandle);
            if (result == 433) {    // 433 is an undocumented "STATUS_NO_SUCH_DEVICE"
                // this should probably be expanded to include other likely errors, but...
                // this indicates the handle is invalid. The port should be closed and invalidated.
                portClose(port);
                portInvalidate(port);
            }
            return 0;
        }
    }

    if (dwWritten != writeCount)
        log_bombastic(IS_LOG_PORT, "[%s] serialPortWritePlatform() wrote %d bytes (%d requested)", portName(port), dwWritten, writeCount);

    debugDumpBuffer(">> ", buffer, dwWritten);
    return dwWritten;

#else

    struct stat sb;
    errno = 0;
    if (fstat(((serialPortHandle*)serialPort->handle)->fd, &sb) != 0)
    {   // Serial port not open
        serialPort->errorCode = errno;
        serialPort->error = strerror(serialPort->errorCode);
        return 0;
    }

    // Ensure all data is queued by OS for sending.  This step is necessary because of O_NONBLOCK non-blocking mode. 
    // Note that this only blocks for partial writes until the OS accepts all input data.  This does NOT block until 
    // the data is physically transmitted.
    uint32_t bytes_written = 0, retry = 0;
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
            serialPort->errorCode = errno;
            serialPort->error = strerror(serialPort->errorCode);
            log_error(IS_LOG_PORT, "[%s] serialPortWritePlatform():: Error writing: %s (%d)", serialPort->portName, serialPort->error, serialPort->errorCode);
            if ((errno == ENOENT) || (errno == ENODEV) || (errno ==  EIO)) {
                // these errors indicate the underlying OS port is bad, and needs to be closed/invalidated - there is usually no other recovery from here.
                portClose(port);
                portInvalidate(port);
            }
            return -1;
        }
        bytes_written += result;
    }

    if (handle->blocking)
    {   // Block until output data has been physically transmitted 
        int error = tcdrain(handle->fd);
        if (error != 0)
        {   // Drain error
            // TODO: report the error (probably as a warning)
            return 0;
        }
    }

    debugDumpBuffer(">> ", buffer, bytes_written);
    return bytes_written;

#endif

}

/**
 * @brief Get the number of bytes available to read from the serial port.
 * This function returns the number of bytes available to be read from the serial port.
 * On Windows, it uses `ClearCommError` and the `COMSTAT` structure.
 * On other platforms, it uses `poll` and `ioctl` with `FIONREAD`.
 *
 * @param port The port handle.
 * @return int The number of bytes available to read, or PORT_ERROR__INVALID on error.
 */
static int serialPortGetByteCountAvailableToReadPlatform(port_handle_t port)
{
    if (!port || !portIsValid(port))
        return PORT_ERROR__INVALID;

    log_bombastic(IS_LOG_PORT, "[%s] serialPortGetByteCountAvailableToReadPlatform() called.", portName(port));

    serial_port_t* serialPort = (serial_port_t*)port;
    serialPortHandle* handle = (serialPortHandle*)serialPort->handle;

#if PLATFORM_IS_WINDOWS

    COMSTAT commStat;
    if (ClearCommError(handle->platformHandle, 0, &commStat))
    {
        return commStat.cbInQue;
    }
    return 0;

#else

    int bytesAvailable = 0;
    struct pollfd p = { .fd = handle->fd, .events = POLLIN };
    int rc;

again:
    rc = poll(&p, 1, 0);
    if (rc > 0) {
        /* Treat POLLIN or urgent/hangup with data as readable */
        if (p.revents & (POLLIN | POLLPRI)) {
            if (ioctl(handle->fd, FIONREAD, &bytesAvailable) < 0) {
                serialPort->errorCode = errno;
                serialPort->error = strerror(serialPort->errorCode);
            }
            return bytesAvailable;
        }
        if (p.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            errno = EIO;
            return -1;
        }
        return 0; // unexpected, but keep contract
    } else if (rc == 0) {
        return 0; // timeout
    } else { // rc < 0
        if (errno == EINTR) goto again;
        return -1;
    }
#endif

}

/**
 * @brief Get the number of bytes available to write to the serial port.
 * This function returns the number of bytes that can be written to the serial port without blocking.
 * Currently, it returns a fixed value of 65536.
 * The commented-out code shows how it could be implemented on Linux using `ioctl`.
 *
 * @param port The port handle.
 * @return int The number of bytes available to write, or PORT_ERROR__INVALID on error.
 */
static int serialPortGetByteCountAvailableToWritePlatform(port_handle_t port)
{
    if (!port || !portIsValid(port))
        return PORT_ERROR__INVALID;

    log_bombastic(IS_LOG_PORT, "[%s] serialPortGetByteCountAvailableToWritePlatform() called.", portName(port));

    serial_port_t* serialPort = (serial_port_t*)port;
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

/**
 * @brief Sleep for a specified number of milliseconds.
 * This function is a simple wrapper around the platform-specific sleep function.
 * On Windows, it uses `Sleep()`, and on other platforms, it uses `usleep()`.
 *
 * @param sleepMilliseconds The number of milliseconds to sleep.
 * @return int 1 on success.
 */
static int serialPortSleepPlatform(int sleepMilliseconds)
{
#if PLATFORM_IS_WINDOWS

    Sleep(sleepMilliseconds);

#else

    usleep(sleepMilliseconds * 1000);

#endif

    return 1;
}

/**
 * @brief Initialize the serial port platform.
 * This function initializes the serial port structure with platform-specific function pointers.
 * It also sets the default baud rate and initializes the base port structure.
 * It is important that the serial port structure is zeroed out before calling this function.
 *
 * @param port The port handle.
 * @return int 0 on success.
 */
int serialPortPlatformInit(port_handle_t port) // unsigned int portOptions
{
    serial_port_t* serialPort = (serial_port_t*)port;
    // very important - the serial port must be initialized to zeros
    base_port_t tmp = { .pnum = portId(port), .ptype = portType(port), .pflags = portFlags(port), .chksum = BASE_PORT(port)->chksum };

    // FIXME:  I really don't like this having to copy and clean, and copy back.  It shouldn't be necessary.
    char tmpName[64] = {0};
    memcpy(tmpName, serialPort->portName, _MIN(sizeof(serialPort->portName), sizeof(tmpName)));
    memset(serialPort, 0, sizeof(serial_port_t));
    memcpy(serialPort->portName, tmpName, _MIN(sizeof(serialPort->portName), sizeof(tmpName)));
    log_more_debug(IS_LOG_PORT, "serialPortPlatformInit() called [%s].", serialPort->portName);

    serialPort->base = tmp;
    portRecalcChksum(port);

    serialPort->base.portName = serialPortName;
    // serialPort->base.portValidate = serialPortValidate;
    serialPort->base.portOpen = serialPortOpen_internal;
    serialPort->base.portClose = serialPortClose;
    serialPort->base.portFree = serialPortGetByteCountAvailableToWrite;
    serialPort->base.portAvailable = serialPortGetByteCountAvailableToRead;
    serialPort->base.portFlush = serialPortFlush;
    serialPort->base.portDrain = serialPortDrain;
    serialPort->base.portRead = serialPortRead;
    serialPort->base.portWrite = serialPortWrite;
    serialPort->base.portReadTimeout = (pfnPortReadTimeout)serialPortReadTimeout;

    serialPort->base.stats = (port_stats_t*)&serialPort->stats;

    if (portType(port) & PORT_TYPE__COMM)
        is_comm_port_init(COMM_PORT(port), NULL);

    serialPort->baudRate = 921600; // default for InertialSense

    // platform specific functions
    serialPort->pfnOpen = serialPortOpenPlatform;
    serialPort->pfnIsOpen = serialPortIsOpenPlatform;
    serialPort->pfnReadTimeout = serialPortReadTimeoutPlatform;
    serialPort->pfnAsyncRead = serialPortAsyncReadPlatform;
    serialPort->pfnFlush = serialPortFlushPlatform;
    serialPort->pfnDrain = serialPortDrainPlatform;
    serialPort->pfnClose = serialPortClosePlatform;
    serialPort->pfnGetByteCountAvailableToWrite = serialPortGetByteCountAvailableToWritePlatform;
    serialPort->pfnGetByteCountAvailableToRead = serialPortGetByteCountAvailableToReadPlatform;
    serialPort->pfnRead = serialPortReadPlatform;
    serialPort->pfnWrite = serialPortWritePlatform;
    serialPort->pfnSleep = serialPortSleepPlatform;
    return 0;
}
