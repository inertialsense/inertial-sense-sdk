/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_SERIALPORT_TYPES_H
#define __IS_SERIALPORT_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

extern int SERIAL_PORT_DEFAULT_TIMEOUT;

#define MAX_SERIAL_PORT_NAME_LENGTH 63

// Standard Baud Rates - FTDI Functional.	// Bit period = 1/baudrate, Actual baud (FTDI,AVR,ARM)
#define BAUDRATE_300			300			// 3333 us
#define BAUDRATE_600			600			// 1667 us
#define BAUDRATE_1200			1200		//  833 us
#define BAUDRATE_2400			2400		//  417 us
#define BAUDRATE_4800			4800		//  208 us
#define BAUDRATE_9600			9600		//  104 us
#define BAUDRATE_19200			19200		//   52 us
#define BAUDRATE_38400			38400		//   26 us
#define BAUDRATE_57600			57600		//   17 us
#define BAUDRATE_115200			115200		// 8680 ns
#define BAUDRATE_230400			230400		// 4340 ns
#define BAUDRATE_460800			460800		// 2170 ns
#define BAUDRATE_921600			921600		// 1085 ns
#define BAUDRATE_1000000		1000000		// 1000 ns
#define BAUDRATE_1220000		1220000		//  820 ns
#define BAUDRATE_1440000		1440000		//  794 ns
#define BAUDRATE_1500000		1500000		//  667 ns	(FTDI 1520, AFR 1500)
#define BAUDRATE_2000000		2000000		//  500 ns	(FTDI 2080, AVR/ARM 2016)
#define BAUDRATE_3000000		3000000		//  333 ns	(FTDI 3150, AVR/ARM 3030)

typedef struct serial_port_t serial_port_t;

typedef int(*pfnSerialPortOpen)(serial_port_t* serialPort, const char* port, int baudRate, int blocking);
typedef int(*pfnSerialPortIsOpen)(serial_port_t* serialPort);
typedef int(*pfnSerialPortRead)(serial_port_t* serialPort, unsigned char* buf, int len, int timeoutMilliseconds);
typedef void(*pfnSerialPortAsyncReadCompletion)(serial_port_t* serialPort, unsigned char* buf, int len, int errorCode);
typedef int(*pfnSerialPortAsyncRead)(serial_port_t* serialPort, unsigned char* buf, int len, pfnSerialPortAsyncReadCompletion completion);
typedef int(*pfnSerialPortWrite)(serial_port_t* serialPort, const unsigned char* buf, int len);
typedef int(*pfnSerialPortClose)(serial_port_t* serialPort);
typedef int(*pfnSerialPortFlush)(serial_port_t* serialPort);
typedef int(*pfnSerialPortGetByteCountAvailableToRead)(serial_port_t* serialPort);
typedef int(*pfnSerialPortGetByteCountAvailableToWrite)(serial_port_t* serialPort);
typedef int(*pfnSerialPortSleep)(int sleepMilliseconds);

// Allows communicating over a serial port
struct serial_port_t
{
	// platform specific handle
	void* handle;

	// the port name (do not modify directly)
	char port[MAX_SERIAL_PORT_NAME_LENGTH + 1];

	// optional error buffer to store errors
	char* error;

	// length of error
	int errorLength;

	// open the serial port
	pfnSerialPortOpen pfnOpen;

	// is the serial port open?
	pfnSerialPortIsOpen pfnIsOpen;

	// read data synchronously
	pfnSerialPortRead pfnRead;

	// read data asynchronously
	pfnSerialPortAsyncRead pfnAsyncRead;

	// write data synchronously
	pfnSerialPortWrite pfnWrite;

	// close the serial port
	pfnSerialPortClose pfnClose;

	// remove all data from all buffers
	pfnSerialPortFlush pfnFlush;

	// get number of bytes in the receive buffer that can be read
	pfnSerialPortGetByteCountAvailableToRead pfnGetByteCountAvailableToRead;

	// get the number of available bytes in the send buffer
	pfnSerialPortGetByteCountAvailableToWrite pfnGetByteCountAvailableToWrite;

	// sleep for a specified number of milliseconds
	pfnSerialPortSleep pfnSleep;
};


#ifdef __cplusplus
}
#endif

#endif // __IS_SERIALPORT_TYPES_H
