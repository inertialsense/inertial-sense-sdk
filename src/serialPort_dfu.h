/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_SERIALPORT_DFU_H
#define __IS_SERIALPORT_DFU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "serialPortTypes.h"

// set the port name for a serial port, in case you are opening it later
void serialPortSetPortDfu(serial_port_t* serialPort, const char* port);

// open a serial port
// port is null terminated, i.e. COM1\0, COM2\0, etc.
// use blocking = 0 when data is being streamed from the serial port rapidly and blocking = 1 for
// uses such as a boot loader where a write would then require n bytes to be read in a single operation.
// blocking simply determines the default timeout value of the serialPortRead function
// returns 1 if success, 0 if failure
int serialPortOpenDfu(serial_port_t* serialPort, const char* port, int baudRate, int blocking);

// open a serial port with retry
// port is null terminated, i.e. COM1\0, COM2\0, etc.
// use blocking = 0 when data is being streamed from the serial port rapidly and blocking = 1 for
// uses such as a boot loader where a write would then require n bytes to be read in a single operation.
// blocking simply determines the default timeout value of the serialPortRead function
// returns 1 if success, 0 if failure
int serialPortOpenRetryDfu(serial_port_t* serialPort, const char* port, int baudRate, int blocking);

// check if the port is open
// returns 1 if open, 0 if not open
int serialPortIsOpenDfu(serial_port_t* serialPort);

// close the serial port - this object can be re-used by calling open again, returns 1 if closed and returns 0 if the port was not closed
int serialPortCloseDfu(serial_port_t* serialPort);

// clear all buffers and pending reads and writes - returns 1 if success, 0 if failure
int serialPortFlushDfu(serial_port_t* serialPort);

// read up to readCount bytes into buffer
// call is forwarded to serialPortReadTimeout with timeoutMilliseconds of 0 for non-blocking, or SERIAL_PORT_DEFAULT_TIMEOUT for blocking.  Returns number of bytes read which is less than or equal to readCount.
int serialPortReadDfu(serial_port_t* serialPort, unsigned char* buffer, int readCount);

// read up to thue number of bytes requested, returns number of bytes read which is less than or equal to readCount
int serialPortReadTimeoutDfu(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds);

// start an async read - not all platforms will support an async read and may call the callback function immediately
// reads up to readCount bytes into buffer
// buffer must exist until callback is executed, if it needs to be freed, free it in the callback or later
// returns 1 if success, 0 if failed to start async operation
int serialPortReadTimeoutAsyncDfu(serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion callback);

// read up until a \r\n sequence has been read
// buffer will not contain \r\n sequence
// returns number of bytes read or -1 if timeout or buffer overflow, count does not include the null terminator
int serialPortReadLineDfu(serial_port_t* serialPort, unsigned char* buffer, int bufferLength);

// read up until a \r\n sequence has been read
// result will not contain \r\n sequence
// returns number of bytes read or -1 if timeout or buffer overflow, count does not include the null terminator
int serialPortReadLineTimeoutDfu(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds);

// read ASCII data Dfu(starts with $ and ends with \r\n, based on NMEA format)
// will ignore data that fails checksum
// asciiData gets set to start of ASCII data
// return -1 if timeout or buffer overflow or checksum failure
int serialPortReadAsciiDfu(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, unsigned char** asciiData);

// read ASCII data Dfu(starts with $ and ends with \r\n, based on NMEA format)
// will ignore data that fails checksum
// asciiData gets set to start of ASCII data
// return -1 if timeout or buffer overflow or checksum failure
int serialPortReadAsciiTimeoutDfu(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds, unsigned char** asciiData);

// read one char, waiting SERIAL_PORT_DEFAULT_TIMEOUT milliseconds to get a char
int serialPortReadCharDfu(serial_port_t* serialPort, unsigned char* c);

// read one char, waiting timeoutMilliseconds to get a char, returns number of chars read
int serialPortReadCharTimeoutDfu(serial_port_t* serialPort, unsigned char* c, int timeoutMilliseconds);

// write, returns the number of bytes written
int serialPortWriteDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount);

// write with a \r\n added at the end, \r\n should not be part of buffer, returns the number of bytes written
int serialPortWriteLineDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount);

// write ascii data - if buffer does not start with $, a $ will be written first, followed by buffer, followed by *xx\r\n, where xx is a two hex character checksum
int serialPortWriteAsciiDfu(serial_port_t* serialPort, const char* buffer, int bufferLength);

// write and wait for a response, returns 1 if success, 0 if failure
int serialPortWriteAndWaitForDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength);
int serialPortWriteAndWaitForTimeoutDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength, const int timeoutMilliseconds);

// wait for a response, returns 0 if failure, 1 if success
int serialPortWaitForDfu(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength);
int serialPortWaitForTimeoutDfu(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength, int timeoutMilliseconds);

// get available bytes in the receive buffer
int serialPortGetByteCountAvailableToReadDfu(serial_port_t* serialPort);

// get available bytes in the send buffer
int serialPortGetByteCountAvailableToWriteDfu(serial_port_t* serialPort);

// sleep for the specified number of milliseconds if supported, returns 1 if success, 0 if failed to sleep
int serialPortSleepDfu(serial_port_t* serialPort, int sleepMilliseconds);

#ifdef __cplusplus
}
#endif

#endif // __IS_SERIALPORT_DFU_H
