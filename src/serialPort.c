/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "serialPort.h"
#include <stdlib.h>
#include <stdint.h>

int SERIAL_PORT_DEFAULT_TIMEOUT = 2500;

void serialPortInit(port_handle_t port, int id, int type) {
    serial_port_t* serialPort = (serial_port_t*)port;
    serialPort->pnum = id;
    serialPort->ptype = type;
}

void serialPortSetOptions(port_handle_t port, uint32_t options)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if ((serialPort != NULL) && ((options & SERIAL_PORT_OPTIONS_MASK) == 0))
	{
        serialPort->options = options;
	}
}

void serialPortSetPort(port_handle_t port, const char* portName)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if ((serialPort != NULL) && (portName != NULL))
	{
		int portLength = (int)strnlen(portName, MAX_SERIAL_PORT_NAME_LENGTH);
		memcpy(serialPort->portName, portName, portLength);
        serialPort->portName[portLength] = '\0';
	}
}

int serialPortOpen(port_handle_t port, const char* portName, int baudRate, int blocking)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort == 0 || port == 0 || serialPort->pfnOpen == 0)
	{
		return 0;
	}
    return serialPort->pfnOpen(port, portName, baudRate, blocking);
}

int serialPortOpenRetry(port_handle_t port, const char* portName, int baudRate, int blocking)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (serialPort == 0 || port == 0 || serialPort->pfnOpen == 0)
    {
        return 0;
    }

    serialPortClose(port);
    for (int retry = 0; retry < 30; retry++)
    {
        if (serialPortOpen(port, portName, baudRate, blocking))
        {
            return 1;
        }
        serialPortSleep(port, 100);
    }
    serialPortClose(port);
    return 0;
}

int serialPortIsOpen(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0)
	{
		return 0;
	}
	return (serialPort->pfnIsOpen ? serialPort->pfnIsOpen(port) : 1);
}

int serialPortClose(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort != 0 && serialPort->pfnClose != 0)
	{
		return serialPort->pfnClose(port);
	}
	return 0;
}

int serialPortFlush(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnFlush == 0)
	{
		return 0;
	}
	return serialPort->pfnFlush(port);
}

int serialPortRead(port_handle_t port, unsigned char* buffer, int readCount)
{
    //serial_port_t* serialPort = (serial_port_t*)port;
	return serialPortReadTimeout(port, buffer, readCount, -1);
}

int serialPortReadTimeout(port_handle_t port, unsigned char* buffer, int readCount, int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || readCount < 1 || serialPort->pfnRead == 0)
	{
		return 0;
	}

	int count = serialPort->pfnRead(port, buffer, readCount, timeoutMilliseconds);

	if (count < 0)
	{
		return 0;
	}

	return count;
}

int serialPortReadTimeoutAsync(port_handle_t port, unsigned char* buffer, int readCount, pfnSerialPortAsyncReadCompletion completion)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || readCount < 1 || serialPort->pfnAsyncRead == 0 || completion == 0)
	{
		return 0;
	}

	return serialPort->pfnAsyncRead(port, buffer, readCount, completion);
}

int serialPortReadLine(port_handle_t port, unsigned char* buffer, int bufferLength)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
	return serialPortReadLineTimeout(port, buffer, bufferLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortReadLineTimeout(port_handle_t port, unsigned char* buffer, int bufferLength, int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (port == 0 || serialPort->handle == 0 || buffer == 0 || bufferLength < 8 || serialPort->pfnRead == 0)
	{
		return 0;
	}

	int prevCR = 0;
	int bufferIndex = 0;
	unsigned char c;
	while (bufferIndex < bufferLength && serialPortReadCharTimeout(port, &c, timeoutMilliseconds) == 1)
	{
		buffer[bufferIndex++] = c;
		if (c == '\n' && prevCR)
		{
			// remove \r\n and null terminate and return count of chars
			buffer[bufferIndex -= 2] = '\0';
			return bufferIndex;
		}
		prevCR = (c == '\r');
	}
	return -1;
}

int serialPortReadAscii(port_handle_t port, unsigned char* buffer, int bufferLength, unsigned char** asciiData)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
	return serialPortReadAsciiTimeout(port, buffer, bufferLength, SERIAL_PORT_DEFAULT_TIMEOUT, asciiData);
}

int serialPortReadAsciiTimeout(port_handle_t port, unsigned char* buffer, int bufferLength, int timeoutMilliseconds, unsigned char** asciiData)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
	int count = serialPortReadLineTimeout(port, buffer, bufferLength, timeoutMilliseconds);
	unsigned char* ptr = buffer;
	unsigned char* ptrEnd = buffer + count;
	while (*ptr != '$' && ptr < ptrEnd)
	{
		ptr++;
	}

	// if at least 8 chars available
	if (ptrEnd - ptr > 7)
	{
		if (asciiData != 0)
		{
			*asciiData = ptr;
		}
		int checksum = 0;
		int existingChecksum;

		// calculate checksum, skipping leading $ and trailing *XX\r\n
		unsigned char* ptrEndNoChecksum = ptrEnd - 3;
		while (++ptr < ptrEndNoChecksum)
		{
			checksum ^= *ptr;
		}

		if (*ptr == '*')
		{
			// read checksum from buffer, skipping the * char
			existingChecksum = strtol((void*)++ptr, NULL, 16);
			if (existingChecksum == checksum)
			{
				return count;
			}
		}
	}

	return -1;
}

int serialPortReadChar(port_handle_t port, unsigned char* c)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
	return serialPortReadCharTimeout(port, c, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortReadCharTimeout(port_handle_t port, unsigned char* c, int timeoutMilliseconds)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
	return serialPortReadTimeout(port, c, 1, timeoutMilliseconds);
}

int serialPortWrite(port_handle_t port, const unsigned char* buffer, int writeCount)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || writeCount < 1 || serialPort->pfnWrite == 0)
	{
		return 0;
	}

	int count = serialPort->pfnWrite(port, buffer, writeCount);

	if (count < 0)
	{
		return 0;
	}

	return count;
}

int serialPortWriteLine(port_handle_t port, const unsigned char* buffer, int writeCount)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || writeCount < 1)
	{
		return 0;
	}

	int count = serialPortWrite(port, buffer, writeCount);
	count += serialPortWrite(port, (unsigned char*)"\r\n", 2);
	return count;
}

int serialPortWriteAscii(port_handle_t port, const char* buffer, int bufferLength)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || bufferLength < 2)
	{
		return 0;
	}

	int checkSum = 0;
	const unsigned char* ptr = (const unsigned char*)buffer;
	int count = 0;

	if (*buffer == '$')
	{
		ptr++;
        bufferLength--;
	}
	else
	{
		count += serialPortWrite(port, (const unsigned char*)"$", 1);
	}

    const unsigned char* ptrEnd = ptr + bufferLength;
    unsigned char buf[16];

	count += serialPortWrite(port, (const unsigned char*)buffer, bufferLength);

	while (ptr != ptrEnd)
	{
		checkSum ^= *ptr++;
	}

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable: 4996)

#endif

	snprintf((char*)buf, sizeof(buf), "*%.2x\r\n", checkSum);

#ifdef _MSC_VER

#pragma warning(pop)

#endif

	count += serialPortWrite(port, buf, 5);

	return count;
}

int serialPortWriteAndWaitFor(port_handle_t port, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength)
{
    //serial_port_t* serialPort = (serial_port_t*)port;
	return serialPortWriteAndWaitForTimeout(port, buffer, writeCount, waitFor, waitForLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortWriteAndWaitForTimeout(port_handle_t port, const unsigned char* buffer, int writeCount, const unsigned char* waitFor, int waitForLength, const int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || buffer == 0 || writeCount < 1 || waitFor == 0 || waitForLength < 1)
	{
		return 0;
	}

	int actuallyWrittenCount = serialPortWrite(port, buffer, writeCount);

	if (actuallyWrittenCount != writeCount)
	{
		return 0;
	}

	return serialPortWaitForTimeout(port, waitFor, waitForLength, timeoutMilliseconds);
}

int serialPortWaitFor(port_handle_t port, const unsigned char* waitFor, int waitForLength)
{
    // serial_port_t* serialPort = (serial_port_t*)port;
	return serialPortWaitForTimeout(port, waitFor, waitForLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortWaitForTimeout(port_handle_t port, const unsigned char* waitFor, int waitForLength, int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || waitFor == 0 || waitForLength < 1)
	{
		return 1;
	}
	else if (waitForLength > 128)
	{
		return 0;
	}

	unsigned char buf[128] = { 0 };
	int count = serialPortReadTimeout(port, buf, waitForLength, timeoutMilliseconds);

	if (count == waitForLength && memcmp(buf, waitFor, waitForLength) == 0)
	{
		return 1;
	}
	return 0;
}

int serialPortGetByteCountAvailableToRead(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnGetByteCountAvailableToRead == 0)
	{
		return 0;
	}

	return serialPort->pfnGetByteCountAvailableToRead(port);
}

int serialPortGetByteCountAvailableToWrite(port_handle_t port)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->handle == 0 || serialPort->pfnGetByteCountAvailableToWrite == 0)
	{
		return 0;
	}

	return serialPort->pfnGetByteCountAvailableToWrite(port);
}

int serialPortSleep(port_handle_t port, int sleepMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
	if (serialPort == 0 || serialPort->pfnSleep == 0)
	{
		return 0;
	}

	return serialPort->pfnSleep(sleepMilliseconds);
}
