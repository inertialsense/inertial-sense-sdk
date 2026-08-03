/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file serialPort.h
 * @brief Platform-independent C API for reading and writing a serial port through a pluggable
 * serial_port_t/base_port_t function-pointer table.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef __IS_SERIALPORT_H
#define __IS_SERIALPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "ISComm.h"

extern int SERIAL_PORT_DEFAULT_TIMEOUT;  //!< default read/write timeout, in milliseconds, used when a blocking port is opened without an explicit timeout

#define MAX_SERIAL_PORT_NAME_LENGTH 63   //!< maximum length (excluding the null terminator) of a serial_port_t name

// Standard Baud Rates - FTDI Functional.   // Bit period = 1/baudrate, Actual baud (FTDI,AVR,ARM)
#define BAUDRATE_300        300             //!< 300 baud (3333 us bit period)
#define BAUDRATE_600        600             //!< 600 baud (1667 us bit period)
#define BAUDRATE_1200       1200            //!< 1200 baud (833 us bit period)
#define BAUDRATE_2400       2400            //!< 2400 baud (417 us bit period)
#define BAUDRATE_4800       4800            //!< 4800 baud (208 us bit period)
#define BAUDRATE_9600       9600            //!< 9600 baud (104 us bit period)
#define BAUDRATE_19200      19200           //!< 19200 baud (52 us bit period)
#define BAUDRATE_38400      38400           //!< 38400 baud (26 us bit period)
#define BAUDRATE_57600      57600           //!< 57600 baud (17 us bit period)
#define BAUDRATE_115200     115200          //!< 115200 baud (8680 ns bit period)
#define BAUDRATE_230400     230400          //!< 230400 baud (4340 ns bit period)
#define BAUDRATE_460800     460800          //!< 460800 baud (2170 ns bit period)
#define BAUDRATE_921600     921600          //!< 921600 baud (1085 ns bit period)
#define BAUDRATE_1000000    1000000         //!< 1,000,000 baud (1000 ns bit period)
#define BAUDRATE_1220000    1220000         //!< 1,220,000 baud (820 ns bit period)
#define BAUDRATE_1440000    1440000         //!< 1,440,000 baud (794 ns bit period)
#define BAUDRATE_1500000    1500000         //!< 1,500,000 baud (667 ns bit period; FTDI 1520, AFR 1500)
#define BAUDRATE_2000000    2000000         //!< 2,000,000 baud (500 ns bit period; FTDI 2080, AVR/ARM 2016)
#define BAUDRATE_3000000    3000000         //!< 3,000,000 baud (333 ns bit period; FTDI 3150, AVR/ARM 3030)
#define BAUDRATE_4000000    4000000         //!< 4,000,000 baud (250 ns bit period; SN-8239)

// SN-8239: maximum baud rate accepted by the SDK serial layer. Standard rates use their termios Bxxx
// constant; any other rate up to this ceiling is applied via the platform custom-rate path
// (Linux termios2/BOTHER, macOS IOSSIOSPEED). Actual achievable rate is bounded by the USB-serial
// bridge/driver, so a successful open does not guarantee the exact line rate.
#define SERIAL_PORT_BAUDRATE_MAX  10000000   //!< maximum baud rate accepted by the SDK serial layer (SN-8239); see comment above

/** Bitfield options for serial line encoding (parity, etc.), combined into serial_port_s::options. */
enum eSerialPortOptions
{
    OPT_PARITY_NONE = 0x0,          //!< no parity bit
    OPT_PARITY_ODD = 0x1,           //!< odd parity
    OPT_PARITY_EVEN = 0x2,          //!< even parity
    OPT_PARITY_MASK = 0x3,          //!< mask isolating the parity bits above

    SERIAL_PORT_OPTIONS_MASK = OPT_PARITY_MASK,  //!< mask of all bits recognized by serialPortSetOptions()
};

typedef int(*pfnSerialPortOpen)(port_handle_t port, const char* portName, int baudRate, int blocking);  //!< function pointer type for opening a port, see serialPortOpen()
typedef int(*pfnSerialPortIsOpen)(port_handle_t port);  //!< function pointer type for checking whether a port is open, see serialPortIsOpen()
typedef int(*pfnSerialPortRead)(port_handle_t port, unsigned char* buf, unsigned int len);  //!< function pointer type for a synchronous, non-blocking read, see serialPortRead()
typedef int(*pfnSerialPortReadTimeout)(port_handle_t port, unsigned char* buf, unsigned int len, int timeoutMs);  //!< function pointer type for a synchronous read with a timeout, see serialPortReadTimeout()
typedef void(*pfnSerialPortAsyncReadCompletion)(port_handle_t port, unsigned char* buf, unsigned int len, int errorCode);  //!< completion callback invoked when an asynchronous read finishes, see serialPortReadTimeoutAsync()
typedef int(*pfnSerialPortAsyncRead)(port_handle_t port, unsigned char* buf, unsigned int len, pfnSerialPortAsyncReadCompletion completion);  //!< function pointer type for starting an asynchronous read, see serialPortReadTimeoutAsync()
typedef int(*pfnSerialPortWrite)(port_handle_t port, const unsigned char* buf, unsigned int len);  //!< function pointer type for a write, see serialPortWrite()
typedef int(*pfnSerialPortClose)(port_handle_t port);  //!< function pointer type for closing a port, see serialPortClose()
typedef int(*pfnSerialPortFlush)(port_handle_t port);  //!< function pointer type shared by the flush and drain operations, see serialPortFlush() and serialPortDrain()
typedef int(*pfnSerialPortGetByteCountAvailableToRead)(port_handle_t port);  //!< function pointer type for querying receive-buffer depth, see serialPortGetByteCountAvailableToRead()
typedef int(*pfnSerialPortGetByteCountAvailableToWrite)(port_handle_t port);  //!< function pointer type for querying send-buffer headroom, see serialPortGetByteCountAvailableToWrite()
typedef int(*pfnSerialPortSleep)(int sleepMilliseconds);  //!< function pointer type for a platform sleep, see serialPortSleep()
typedef int(*pfnSerialPortOnErrorCB)(port_handle_t port, int errCode, const char *errMsg);  //!< error callback invoked by the serialPort implementation when an operation fails, see serialPortSetErrorCB()

/**
 * Represents a serial port, combining the shared base_port_t/comm_port_t state with a
 * function-pointer table (the pfn* members) that a platform backend (see serialPortPlatform.h)
 * fills in to provide the actual OS-level implementation.
 */
struct serial_port_s
{
    union {
        base_port_t base;   //!< base "implementation" viewed as a generic port
        comm_port_t comm;   //!< base "implementation" viewed as a comm port
    };

    port_monitor_set_t stats;              //!< communication statistics for this port

    rmci_t rmci;                                       //!< realtime message controller interface state for this port
    uint8_t rmciUPMcnt[DID_COUNT];                     //!< per-DID broadcast counters used by the realtime message controller
    uint8_t rmciNMEAcnt[NMEA_MSG_ID_COUNT];            //!< per-NMEA-message-id broadcast counters used by the realtime message controller

    void* handle;                                       //!< platform-specific handle (e.g. file descriptor or OS handle)

    char portName[MAX_SERIAL_PORT_NAME_LENGTH + 1];     //!< the port name (do not modify directly; use serialPortSetName())

    int baudRate;                                       //!< the current (or expected) baud rate to communicate at

    int blocking;                                       //!< non-zero if this port is configured for blocking calls

    int errorCode;                                       //!< latest errno that was reported from an operation on this port

    char* error;                                         //!< optional caller-owned buffer used to store the latest error message

    int errorLength;                                     //!< length, in bytes, of the error buffer above

    uint32_t options;                                    //!< options for encoding like parity, stop bits, etc. (see eSerialPortOptions)

    pfnSerialPortOpen pfnOpen;                                                       //!< opens the serial port
    pfnSerialPortIsOpen pfnIsOpen;                                                   //!< reports whether the serial port is open
    pfnSerialPortRead pfnRead;                                                       //!< reads data synchronously
    pfnSerialPortReadTimeout pfnReadTimeout;                                         //!< reads data synchronously with a timeout
    pfnSerialPortAsyncRead pfnAsyncRead;                                             //!< reads data asynchronously
    pfnSerialPortWrite pfnWrite;                                                     //!< writes data synchronously
    pfnSerialPortClose pfnClose;                                                     //!< closes the serial port
    pfnSerialPortFlush pfnFlush;                                                     //!< discards all data from all buffers
    pfnSerialPortFlush pfnDrain;                                                     //!< blocks until all queued TX data has been sent
    pfnSerialPortGetByteCountAvailableToRead pfnGetByteCountAvailableToRead;         //!< gets the number of bytes in the receive buffer that can be read
    pfnSerialPortGetByteCountAvailableToWrite pfnGetByteCountAvailableToWrite;       //!< gets the number of available bytes in the send buffer
    pfnSerialPortSleep pfnSleep;                                                     //!< sleeps for a specified number of milliseconds

    pfnSerialPortOnErrorCB pfnError;                                                 //!< optional callback invoked when an operation on this port reports an error
};

typedef struct serial_port_s serial_port_t;   //!< see struct serial_port_s
#define SERIAL_PORT(n)  ((serial_port_t*)n)    //!< reinterprets a port_handle_t (or any compatible pointer) as a pointer to serial_port_t

/**
 * Initializes a serial_port_t: sets its port number/type/flags, invokes serialPortPlatformInit()
 * to install the platform function-pointer table, and marks the port valid.
 * @param port the port to initialize
 * @param id the port number to assign
 * @param type the port type flags (see PORT_TYPE__* constants)
 * @param flags additional port flags to set
 * @return PORT_ERROR__NONE on success, otherwise a PORT_ERROR__* code
 */
int serialPortInit(port_handle_t port, int id, int type, int flags);

/**
 * Sets the port name for a serial port, in case you are opening it later.
 * @param port the port to update
 * @param portName the null-terminated port name to store (e.g. "COM1" or "/dev/ttyACM0")
 * @return PORT_ERROR__NONE on success, otherwise a PORT_ERROR__* code
 */
int serialPortSetName(port_handle_t port, const char* portName);

/**
 * Returns the name associated with this port (this is usually the OS's identifier).
 * @param port the port to query
 * @return the port name (this is usually the OS's identifier)
 */
const char *serialPortName(port_handle_t port);

/**
 * Opens a serial port. portName is null terminated, i.e. COM1\0, COM2\0, etc. Use blocking = 0
 * when data is being streamed from the serial port rapidly and blocking = 1 for uses such as a
 * boot loader where a write would then require n bytes to be read in a single operation. Blocking
 * simply determines the default timeout value of the serialPortRead function.
 * @param port the port to open
 * @param portName the null-terminated OS port name to open, e.g. "COM1" or "/dev/ttyACM0"
 * @param baudRate the baud rate to open the port at
 * @param blocking whether reads on this port default to blocking (non-zero) or non-blocking (zero)
 * @return PORT_ERROR__NONE (0) on success, otherwise a negative PORT_ERROR__* code (e.g. PORT_ERROR__OPEN_FAILURE)
 */
int serialPortOpen(port_handle_t port, const char* portName, int baudRate, int blocking);

/**
 * Opens the specified serial port, using previously set options for name, baudrate, and flags/options.
 * @param port the port to open
 * @return PORT_ERROR__NONE (0) on success, otherwise a negative PORT_ERROR__* code
 */
int serialPortOpen_internal(port_handle_t port);

/**
 * Opens a serial port, retrying on failure. portName is null terminated, i.e. COM1\0, COM2\0, etc.
 * Use blocking = 0 when data is being streamed from the serial port rapidly and blocking = 1 for
 * uses such as a boot loader where a write would then require n bytes to be read in a single
 * operation. Blocking simply determines the default timeout value of the serialPortRead function.
 * @param port the port to open
 * @param portName the null-terminated OS port name to open, e.g. "COM1" or "/dev/ttyACM0"
 * @param baudRate the baud rate to open the port at
 * @param blocking whether reads on this port default to blocking (non-zero) or non-blocking (zero)
 * @return PORT_ERROR__NONE (0) on success, otherwise a negative PORT_ERROR__* code
 */
int serialPortOpenRetry(port_handle_t port, const char* portName, int baudRate, int blocking);

/**
 * Checks if the port is open.
 * @param port the port to query
 * @return 1 if open, 0 if not open
 */
int serialPortIsOpen(port_handle_t port);

/**
 * Checks if the port is open, but avoids an expensive OS/kernel call if possible.
 * If the internal handle is NOT null, and there are recent errors on the port
 * this function will return true, indicating that the port is open. However,
 * it should be noted that the port may still be closed by the OS or another
 * mechanism which may not be reflected in the local state, which could cause
 * this to report incorrectly and then leading to a future error state when
 * operating on the closed port.
 * @param port the port to query
 * @return 1 if open, 0 if not open
 */
int serialPortIsOpenQuick(port_handle_t port);

/**
 * Closes the serial port - this object can be re-used by calling open again.
 * @param port the port to close
 * @return 1 if closed, 0 if the port was not closed
 */
int serialPortClose(port_handle_t port);

/**
 * Clears all buffers and pending reads and writes.
 * @param port the port to flush
 * @return 1 if success, 0 if failure
 */
int serialPortFlush(port_handle_t port);

/**
 * Blocks until all pending TX writes have completed, and the TX buffer is empty.
 * @param port the port to drain
 * @param timeoutMs the number of milliseconds to wait, at most before data is discarded
 * @return 1 if success, 0 if failure
 */
int serialPortDrain(port_handle_t port, uint32_t timeoutMs);

/**
 * Reads up to readCount bytes into buffer. Call is forwarded to serialPortReadTimeout with
 * timeoutMs of 0 for non-blocking, or SERIAL_PORT_DEFAULT_TIMEOUT for blocking.
 * @param port the port to read from
 * @param buffer the buffer to read data into
 * @param readCount the maximum number of bytes to read
 * @return number of bytes read which is less than or equal to readCount, or a negative PORT_ERROR__* code on failure.
 */
int serialPortRead(port_handle_t port, unsigned char* buffer, unsigned int readCount);

/**
 * Reads up to the number of bytes requested.
 * @param port the port to read from
 * @param buffer the buffer to read data into
 * @param readCount the maximum number of bytes to read
 * @param timeoutMs the maximum number of milliseconds to wait for data
 * @return number of bytes read which is less than or equal to readCount, or a negative PORT_ERROR__* code on failure
 */
int serialPortReadTimeout(port_handle_t port, unsigned char* buffer, unsigned int readCount, uint32_t timeoutMs);

/**
 * Starts an async read - not all platforms will support an async read and may call the callback
 * function immediately. Reads up to readCount bytes into buffer. buffer must exist until callback
 * is executed; if it needs to be freed, free it in the callback or later.
 * @param port the port to read from
 * @param buffer the buffer to read data into
 * @param readCount the maximum number of bytes to read
 * @param callback function invoked when the read completes
 * @return 1 if success, 0 if failed to start async operation
 */
int serialPortReadTimeoutAsync(port_handle_t port, unsigned char* buffer, unsigned int readCount, pfnSerialPortAsyncReadCompletion callback);

/**
 * Reads up until a CRLF (\\r\\n) sequence has been read. buffer will not contain the CRLF sequence.
 * @param port the port to read from
 * @param buffer the buffer to read data into
 * @param bufferLength the size of buffer, in bytes
 * @return number of bytes read or -1 if timeout or buffer overflow, count does not include the null terminator
 */
int serialPortReadLine(port_handle_t port, unsigned char* buffer, unsigned int bufferLength);

/**
 * Reads up until a CRLF (\\r\\n) sequence has been read. Result will not contain the CRLF sequence.
 * @param port the port to read from
 * @param buffer the buffer to read data into
 * @param bufferLength the size of buffer, in bytes
 * @param timeoutMilliseconds the maximum number of milliseconds to wait for the line
 * @return number of bytes read or -1 if timeout or buffer overflow, count does not include the null terminator
 */
int serialPortReadLineTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, int timeoutMilliseconds);

/**
 * Reads ASCII data (starts with $ and ends with CRLF, based on NMEA format). Will ignore data
 * that fails checksum. asciiData gets set to the start of the ASCII data.
 * @param port the port to read from
 * @param buffer the buffer to read data into
 * @param bufferLength the size of buffer, in bytes
 * @param asciiData receives a pointer into buffer at which the validated ASCII data starts
 * @return -1 if timeout or buffer overflow or checksum failure
 */
int serialPortReadAscii(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned char** asciiData);

/**
 * Reads ASCII data (starts with $ and ends with CRLF, based on NMEA format). Will ignore data
 * that fails checksum. asciiData gets set to the start of the ASCII data.
 * @param port the port to read from
 * @param buffer the buffer to read data into
 * @param bufferLength the size of buffer, in bytes
 * @param timeoutMilliseconds the maximum number of milliseconds to wait for the data
 * @param asciiData receives a pointer into buffer at which the validated ASCII data starts
 * @return -1 if timeout or buffer overflow or checksum failure
 */
int serialPortReadAsciiTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, int timeoutMilliseconds, unsigned char** asciiData);

/**
 * Reads one char, waiting SERIAL_PORT_DEFAULT_TIMEOUT milliseconds to get a char.
 * @param port the port to read from
 * @param c receives the character read
 * @return 1 if a character was read, 0 on timeout, or a negative PORT_ERROR__* code on failure
 */
int serialPortReadChar(port_handle_t port, unsigned char* c);

/**
 * Reads one char, waiting timeoutMilliseconds to get a char.
 * @param port the port to read from
 * @param c receives the character read
 * @param timeoutMilliseconds the maximum number of milliseconds to wait for the character
 * @return 1 if a character was read, 0 on timeout, or a negative PORT_ERROR__* code on failure
 */
int serialPortReadCharTimeout(port_handle_t port, unsigned char* c, int timeoutMilliseconds);

/**
 * Writes data to the port.
 * @param port the port to write to
 * @param buffer the data to write
 * @param writeCount the number of bytes to write
 * @return the number of bytes written, or a negative PORT_ERROR__* code on failure
 */
int serialPortWrite(port_handle_t port, const unsigned char* buffer, unsigned int writeCount);

/**
 * Writes data to the port with a CRLF (\\r\\n) added at the end; buffer should not already include it.
 * @param port the port to write to
 * @param buffer the data to write (without a trailing CRLF)
 * @param writeCount the number of bytes in buffer to write
 * @return the number of bytes written
 */
int serialPortWriteLine(port_handle_t port, const unsigned char* buffer, unsigned int writeCount);

/**
 * Writes ASCII data - if buffer does not start with $, a $ will be written first, followed by
 * buffer, followed by *xx and a trailing CRLF, where xx is a two hex character checksum.
 * @param port the port to write to
 * @param buffer the ASCII data to write
 * @param bufferLength the number of bytes in buffer to write
 * @return the number of bytes written
 */
int serialPortWriteAscii(port_handle_t port, const char* buffer, unsigned int bufferLength);

/**
 * Writes data and waits for a response, using the default timeout.
 * @param port the port to write to and wait on
 * @param buffer the data to write
 * @param writeCount the number of bytes in buffer to write
 * @param waitFor the byte sequence to wait for in the response
 * @param waitForLength the number of bytes in waitFor
 * @return 1 if success, 0 if failure
 */
int serialPortWriteAndWaitFor(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength);

/**
 * Writes data and waits for a response, with an explicit timeout.
 * @param port the port to write to and wait on
 * @param buffer the data to write
 * @param writeCount the number of bytes in buffer to write
 * @param waitFor the byte sequence to wait for in the response
 * @param waitForLength the number of bytes in waitFor
 * @param timeoutMilliseconds the maximum number of milliseconds to wait for waitFor
 * @return 1 if success, 0 if failure
 */
int serialPortWriteAndWaitForTimeout(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength, const int timeoutMilliseconds);

/**
 * Waits for a specific byte sequence to be received, using the default timeout.
 * @param port the port to wait on
 * @param waitFor the byte sequence to wait for
 * @param waitForLength the number of bytes in waitFor
 * @return 0 if failure, 1 if success
 */
int serialPortWaitFor(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength);

/**
 * Waits for a specific byte sequence to be received, with an explicit timeout.
 * @param port the port to wait on
 * @param waitFor the byte sequence to wait for
 * @param waitForLength the number of bytes in waitFor
 * @param timeoutMilliseconds the maximum number of milliseconds to wait for waitFor
 * @return 0 if failure, 1 if success
 */
int serialPortWaitForTimeout(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength, int timeoutMilliseconds);

/**
 * Gets the number of available bytes in the receive buffer that can be read.
 * @param port the port to query
 * @return number of bytes available to read
 */
int serialPortGetByteCountAvailableToRead(port_handle_t port);

/**
 * Gets the number of available bytes in the send buffer.
 * @param port the port to query
 * @return number of bytes of headroom available in the send buffer
 */
int serialPortGetByteCountAvailableToWrite(port_handle_t port);

/**
 * Sleeps for the specified number of milliseconds, if supported.
 * @param port the port whose platform sleep implementation to use
 * @param sleepMilliseconds the number of milliseconds to sleep
 * @return 1 if success, 0 if failed to sleep
 */
int serialPortSleep(port_handle_t port, int sleepMilliseconds);

/**
 * Sets the port options (see eSerialPortOptions).
 * @param port the port to update
 * @param options the option bits to set (see eSerialPortOptions)
 * @return PORT_ERROR__NONE on success, otherwise a PORT_ERROR__* code
 */
int serialPortSetOptions(port_handle_t port, uint32_t options);

/**
 * Sets the port baud rate.
 * @param port the port to update
 * @param baudRate the baud rate to record for this port
 * @return PORT_ERROR__NONE on success, otherwise a PORT_ERROR__* code
 */
int serialPortSetBaud(port_handle_t port, int baudRate);

/**
 * Sets the callback for error events reported by operations on this port.
 * @param port the port to update
 * @param onErrorCb callback invoked when an operation on this port reports an error
 * @return PORT_ERROR__NONE on success, otherwise a PORT_ERROR__* code (e.g. PORT_ERROR__NOT_SUPPORTED)
 */
int serialPortSetErrorCB(port_handle_t port, pfnSerialPortOnErrorCB onErrorCb);


#ifdef __cplusplus
}
#endif

#endif // __IS_SERIALPORT_H
