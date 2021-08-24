/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "serialPort_dfu.h"
#include <stdlib.h>

int SERIAL_PORT_DEFAULT_TIMEOUT = 2500;

void serialPortSetPort(serial_port_t* serialPort, const char* port)
{
}

int serialPortOpen(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
	return 0;
}

int serialPortOpenRetry(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
	return 0;
}

int serialPortIsOpen(serial_port_t* serialPort)
{
	return 0;
}

int serialPortClose(serial_port_t* serialPort)
{
	return 0;
}

int serialPortFlush(serial_port_t* serialPort)
{
	return 0;
}

int serialPortRead(serial_port_t* serialPort, unsigned char* buffer, int readCount)
{
	return 0;
}

int serialPortReadTimeout(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
	return 0;
}

int serialPortReadTimeoutAsync(serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion completion)
{
	return 0;
}

int serialPortReadLine(serial_port_t* serialPort, unsigned char* buffer, int bufferLength)
{
	return 0;
}

int serialPortReadLineTimeout(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds)
{
	return 0;
}

int serialPortReadAscii(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, unsigned char** asciiData)
{
	return 0;
}

int serialPortReadAsciiTimeout(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds, unsigned char** asciiData)
{
	return 0;
}

int serialPortReadChar(serial_port_t* serialPort, unsigned char* c)
{
	return 0;
}

int serialPortReadCharTimeout(serial_port_t* serialPort, unsigned char* c, int timeoutMilliseconds)
{
	return 0;
}

int serialPortWrite(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
	return 0;
}

int serialPortWriteLine(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
	return 0;
}

int serialPortWriteAscii(serial_port_t* serialPort, const char* buffer, int bufferLength)
{
	return 0;
}

int serialPortWriteAndWaitFor(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength)
{
	return 0;
}

int serialPortWriteAndWaitForTimeout(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength, const int timeoutMilliseconds)
{
	return 0;
}

int serialPortWaitFor(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength)
{
	return 0;
}

int serialPortWaitForTimeout(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength, int timeoutMilliseconds)
{
	return 0;
}

int serialPortGetByteCountAvailableToRead(serial_port_t* serialPort)
{
	return 0;
}

int serialPortGetByteCountAvailableToWrite(serial_port_t* serialPort)
{
	return 0;
}

int serialPortSleep(serial_port_t* serialPort, int sleepMilliseconds)
{
	return 0;
}
