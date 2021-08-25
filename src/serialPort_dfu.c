/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "serialPort_dfu.h"
#include <stdlib.h>

void serialPortSetPortDfu(serial_port_t* serialPort, const char* port)
{
}

int serialPortOpenDfu(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
	return 0;
}

int serialPortOpenRetryDfu(serial_port_t* serialPort, const char* port, int baudRate, int blocking)
{
	return 0;
}

int serialPortIsOpenDfu(serial_port_t* serialPort)
{
	return 0;
}

int serialPortCloseDfu(serial_port_t* serialPort)
{
	return 0;
}

int serialPortFlushDfu(serial_port_t* serialPort)
{
	return 0;
}

int serialPortReadDfu(serial_port_t* serialPort, unsigned char* buffer, int readCount)
{
	return 0;
}

int serialPortReadTimeoutDfu(serial_port_t* serialPort, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
	return 0;
}

int serialPortReadTimeoutAsyncDfu(serial_port_t* serialPort, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion completion)
{
	return 0;
}

int serialPortReadLineDfu(serial_port_t* serialPort, unsigned char* buffer, int bufferLength)
{
	return 0;
}

int serialPortReadLineTimeoutDfu(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds)
{
	return 0;
}

int serialPortReadAsciiDfu(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, unsigned char** asciiData)
{
	return 0;
}

int serialPortReadAsciiTimeoutDfu(serial_port_t* serialPort, unsigned char* buffer, int bufferLength, int timeoutMilliseconds, unsigned char** asciiData)
{
	return 0;
}

int serialPortReadCharDfu(serial_port_t* serialPort, unsigned char* c)
{
	return 0;
}

int serialPortReadCharTimeoutDfu(serial_port_t* serialPort, unsigned char* c, int timeoutMilliseconds)
{
	return 0;
}

int serialPortWriteDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
	return 0;
}

int serialPortWriteLineDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount)
{
	return 0;
}

int serialPortWriteAsciiDfu(serial_port_t* serialPort, const char* buffer, int bufferLength)
{
	return 0;
}

int serialPortWriteAndWaitForDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength)
{
	return 0;
}

int serialPortWriteAndWaitForTimeoutDfu(serial_port_t* serialPort, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength, const int timeoutMilliseconds)
{
	return 0;
}

int serialPortWaitForDfu(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength)
{
	return 0;
}

int serialPortWaitForTimeoutDfu(serial_port_t* serialPort, const unsigned char* waitFor, int waitForLength, int timeoutMilliseconds)
{
	return 0;
}

int serialPortGetByteCountAvailableToReadDfu(serial_port_t* serialPort)
{
	return 0;
}

int serialPortGetByteCountAvailableToWriteDfu(serial_port_t* serialPort)
{
	return 0;
}

int serialPortSleepDfu(serial_port_t* serialPort, int sleepMilliseconds)
{
	return 0;
}
