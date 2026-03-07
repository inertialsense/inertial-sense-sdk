/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 *
 *  NOTE - March 6, 2026 - All functions are changed to report PORT_ERROR__* rather than legacy 0 on success on non-zero on failure
 *  Since all PORT_ERROR__* are negative numbers, some functions (read/write, etc) can return positive numbers when successful.
 *  ONLY negative numbers should be considered an error, >=0 is considered a success.
 *
 */

#include "serialPort.h"

#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include "serialPortPlatform.h"
#include "ISUtilities.h"

int SERIAL_PORT_DEFAULT_TIMEOUT = 500;

#define IS_PORT_FUNCTION_SUPPORTED(port, functionPtr, message)   { do {                                                 \
    if (!portIsValid(port)) return PORT_ERROR__INVALID;                                                                 \
    if (SERIAL_PORT(port)->functionPtr == 0) {                                                                          \
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, PORT_ERROR__NOT_SUPPORTED, ((message)));     \
        return PORT_ERROR__NOT_SUPPORTED;                                                                               \
    }                                                                                                                   \
} while(0); }


int serialPortInit(port_handle_t port, int id, int type, int flags) {
    if (!port) return PORT_ERROR__INVALID;

    SERIAL_PORT(port)->base.pnum = id;
    SERIAL_PORT(port)->base.ptype = type;
    SERIAL_PORT(port)->base.pflags = flags;
    serialPortPlatformInit(port);

    portFlagsSet(port, PORT_FLAG__VALID);
    return PORT_ERROR__NONE;
}

int serialPortSetOptions(port_handle_t port, uint32_t options)
{
    if (!port) return PORT_ERROR__INVALID;
    if ((options & SERIAL_PORT_OPTIONS_MASK) == 0)
    {
        SERIAL_PORT(port)->options = options;
    }
    return PORT_ERROR__NONE;
}

int serialPortSetBaud(port_handle_t port, int baudRate) {
    if (!port) return PORT_ERROR__INVALID;
    // TODO: this should perform the baud rate validation and return PORT_ERROR__INVALID_ARGUMENT if invalid
    SERIAL_PORT(port)->baudRate = baudRate;
    return PORT_ERROR__NONE;
}

int serialPortSetName(port_handle_t port, const char* portName)
{
    if (!port) return PORT_ERROR__INVALID;
    if (!portName) return PORT_ERROR__INVALID_PARAMETER;

    int portLength = (int)strnlen(portName, MAX_SERIAL_PORT_NAME_LENGTH);
    memcpy(SERIAL_PORT(port)->portName, portName, portLength);
    SERIAL_PORT(port)->portName[portLength] = '\0';
    return PORT_ERROR__NONE;
}

const char *serialPortName(port_handle_t port)
{
    return (port ? SERIAL_PORT(port)->portName : NULL);
}

int serialPortOpen(port_handle_t port, const char* portName, int baudRate, int blocking)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnOpen, "serialPortOpen() is not supported for this port.");
    if (!portName) {
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, PORT_ERROR__INVALID_PARAMETER, "The port name is invalid.");
        return PORT_ERROR__INVALID_PARAMETER;
    }
    if (SERIAL_PORT(port)->pfnOpen(port, portName, baudRate, blocking) != 1) {
        // the errorCode/error should be populated by the pfnOpen() call.
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, SERIAL_PORT(port)->errorCode, SERIAL_PORT(port)->error);
        return PORT_ERROR__OPEN_FAILURE;
    }
    portFlagsSet(port, PORT_FLAG__OPENED);
    return PORT_ERROR__NONE;
}

int serialPortOpen_internal(port_handle_t port) {
    if (!portIsValid(port)) return PORT_ERROR__INVALID;
    return serialPortOpen(port, SERIAL_PORT(port)->portName, SERIAL_PORT(port)->baudRate, (portFlagsIsSet(port, PORT_FLAG__BLOCKING) ? 1 : 0));
}

int serialPortOpenRetry(port_handle_t port, const char* portName, int baudRate, int blocking)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnOpen, "serialPortOpen() is not supported for this port.");

    if (serialPortIsOpen(port))
        return PORT_ERROR__NONE;

    int resultCode = PORT_ERROR__NONE;
    for (int retry = 0; retry < 5; retry++)
    {
        int resultCode = serialPortOpen(port, portName, baudRate, blocking);
        if (resultCode == PORT_ERROR__NONE)
            return PORT_ERROR__NONE;

        if (SERIAL_PORT(port)->errorCode == ENOENT)
            return PORT_ERROR__INVALID;  // don't retry if the port doesn't even exist

        serialPortSleep(port, 250); // wait a quarter second and try again
    }
    if (SERIAL_PORT(port)->pfnError)
        SERIAL_PORT(port)->pfnError(port, SERIAL_PORT(port)->errorCode, SERIAL_PORT(port)->error);
    if (serialPortIsOpen(port))
        serialPortClose(port);  // If, for what ever reason, the port is determined to be opened, close it.

    return resultCode;
}

int serialPortIsOpen(port_handle_t port)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnIsOpen, "serialPortIsOpen() is not supported for this port.");
    return SERIAL_PORT(port)->pfnIsOpen(port);      // returns 1 = open, 0 = not open
}

int serialPortIsOpenQuick(port_handle_t port)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnIsOpen, "serialPortIsOpen() is not supported for this port.");

    if ((SERIAL_PORT(port)->handle != NULL) && (SERIAL_PORT(port)->errorCode == 0) && portFlagsIsSet(port, PORT_FLAG__OPENED))
        return PORT_ERROR__NONE;

    return SERIAL_PORT(port)->pfnIsOpen(port);      // returns 1 = open, 0 = not open
}

int serialPortClose(port_handle_t port)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnClose, "serialPortClose() is not supported for this port.");
    portFlagsClear(port, PORT_FLAG__OPENED);       // safe to do before closing - because if the close fails, its fair the say the port is still invalid
    return SERIAL_PORT(port)->pfnClose(port);
}

int serialPortFlush(port_handle_t port)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnFlush, "serialPortFlush() is not supported for this port.");
    return SERIAL_PORT(port)->pfnFlush(port);
}

int serialPortDrain(port_handle_t port, uint32_t timeoutMs)
{
    (void) timeoutMs;
    IS_PORT_FUNCTION_SUPPORTED(port, pfnDrain, "serialPortDrain() is not supported for this port.");
    return SERIAL_PORT(port)->pfnDrain(port); // currently, our implementation ignores timeoutMs
}


int serialPortRead(port_handle_t port, unsigned char* buffer, unsigned int readCount)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnRead, "serialPortRead() is not supported for this port.");
    if (!buffer || (readCount <= 0)) {
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, PORT_ERROR__INVALID_PARAMETER, "serialPortRead() called with an invalid parameter.");
        return PORT_ERROR__INVALID_PARAMETER;
    }

    return SERIAL_PORT(port)->pfnRead(port, buffer, readCount);
}

int serialPortReadTimeout(port_handle_t port, unsigned char* buffer, unsigned int readCount, uint32_t timeoutMs)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnReadTimeout, "serialPortReadTimeout() is not supported for this port.");
    if (!buffer || (readCount <= 0)) {
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, PORT_ERROR__INVALID_PARAMETER, "serialPortRead() called with an invalid parameter.");
        return PORT_ERROR__INVALID_PARAMETER;
    }

    return SERIAL_PORT(port)->pfnReadTimeout(port, buffer, readCount, timeoutMs);
}

// [[deprecated("Use portReadTimeoutAsync() instead.")]]
int serialPortReadTimeoutAsync(port_handle_t port, unsigned char* buffer, unsigned int readCount, pfnSerialPortAsyncReadCompletion completionFn)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnAsyncRead, "serialPortReadTimeoutAsync() is not supported for this port.");
    if ((buffer == 0) || (readCount < 1) || (SERIAL_PORT(port)->pfnAsyncRead == 0) || (completionFn == 0)) {
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, PORT_ERROR__INVALID_PARAMETER, "serialPortRead() called with an invalid parameter.");
        return PORT_ERROR__INVALID_PARAMETER;
    }

    return SERIAL_PORT(port)->pfnAsyncRead(port, buffer, readCount, completionFn);
}

// [[deprecated("Use portReadLine() instead.")]]
int serialPortReadLine(port_handle_t port, unsigned char* buffer, unsigned int bufferLength)
{
    return serialPortReadLineTimeout(port, buffer, bufferLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

// [[deprecated("Use portReadLineTimeout() instead.")]]
int serialPortReadLineTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, int timeoutMilliseconds)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnAsyncRead, "serialPortReadTimeoutAsync() is not supported for this port.");
    if ((buffer == 0) || (bufferLength < 8)) {
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, PORT_ERROR__INVALID_PARAMETER, "serialPortReadLineTimeout() called with an invalid parameter.");
        return PORT_ERROR__INVALID_PARAMETER;
    }

    int prevCR = 0;
    unsigned int bufferIndex = 0;
    unsigned char c;
    while (bufferIndex < bufferLength && serialPortReadCharTimeout(port, &c, timeoutMilliseconds) == 1)
    {
        buffer[bufferIndex++] = c;
        if (c == '\n' && prevCR)
        {
            // remove \r\n and null terminate and return count of chars
            buffer[bufferIndex -= 2] = '\0';
            return (int)bufferIndex;
        }
        prevCR = (c == '\r');
    }
    return -1;
}

// [[deprecated("Use portReadAscii() instead.")]]
int serialPortReadAscii(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, unsigned char** asciiData)
{
    return serialPortReadAsciiTimeout(port, buffer, bufferLength, SERIAL_PORT_DEFAULT_TIMEOUT, asciiData);
}

// [[deprecated("Use portReadAsciiTimeout() instead.")]]
int serialPortReadAsciiTimeout(port_handle_t port, unsigned char* buffer, unsigned int bufferLength, int timeoutMilliseconds, unsigned char** asciiData)
{
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
    return serialPortReadCharTimeout(port, c, SERIAL_PORT_DEFAULT_TIMEOUT);
}

int serialPortReadCharTimeout(port_handle_t port, unsigned char* c, int timeoutMilliseconds)
{
    return serialPortReadTimeout(port, c, 1, timeoutMilliseconds);
}

int serialPortWrite(port_handle_t port, const unsigned char* buffer, unsigned int writeCount)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnWrite, "serialPortWrite() is not supported for this port.");
    if (!buffer || (writeCount <= 0)) {
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, PORT_ERROR__INVALID_PARAMETER, "serialPortWrite() called with an invalid parameter.");
        return PORT_ERROR__INVALID_PARAMETER;
    }

    return SERIAL_PORT(port)->pfnWrite(port, buffer, writeCount);
}

// [[deprecated("Use portWriteLine() instead.")]]
int serialPortWriteLine(port_handle_t port, const unsigned char* buffer, unsigned int writeCount)
{
    int count = serialPortWrite(port, buffer, writeCount);
    if (count == (int)writeCount)
        count += serialPortWrite(port, (unsigned char*)"\r\n", 2);  // FIXME: handle if this returns an error
    return count;
}

// [[deprecated("Use portWriteAscii() instead.")]]
int serialPortWriteAscii(port_handle_t port, const char* buffer, unsigned int bufferLength)
{
    if (!buffer || (bufferLength < 2)) {
        if (SERIAL_PORT(port)->pfnError) SERIAL_PORT(port)->pfnError(port, PORT_ERROR__INVALID_PARAMETER, "serialPortWriteAscii() called with an invalid parameter.");
        return PORT_ERROR__INVALID_PARAMETER;
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

// [[deprecated("Use portWriteAndWaitFor() instead.")]]
int serialPortWriteAndWaitFor(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength)
{
    return serialPortWriteAndWaitForTimeout(port, buffer, writeCount, waitFor, waitForLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

// [[deprecated("Use portWriteAndWaitForTimeout() instead.")]]
int serialPortWriteAndWaitForTimeout(port_handle_t port, const unsigned char* buffer, unsigned int writeCount, const unsigned char* waitFor, unsigned int waitForLength, const int timeoutMilliseconds)
{
    if (serialPortWrite(port, buffer, writeCount) != (int)writeCount)
    {
        return 0;
    }

    return serialPortWaitForTimeout(port, waitFor, waitForLength, timeoutMilliseconds);
}

// [[deprecated("Use portWaitFor() instead.")]]
int serialPortWaitFor(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength)
{
    return serialPortWaitForTimeout(port, waitFor, waitForLength, SERIAL_PORT_DEFAULT_TIMEOUT);
}

// [[deprecated("Use portWaitForTimeout() instead.")]]
int serialPortWaitForTimeout(port_handle_t port, const unsigned char* waitFor, unsigned int waitForLength, int timeoutMilliseconds)
{
    serial_port_t* serialPort = (serial_port_t*)port;
    if (!serialPort) return 0;
    if (!serialPort->handle) {
        if (serialPort->pfnError) serialPort->pfnError(port, EBADF, strerror(EBADF));
        return 0;
    }

    if ((waitFor == 0) || (waitForLength < 1))
    {
        return 1;
    }
    else if (waitForLength > 128)
    {
        if (serialPort->pfnError) serialPort->pfnError(port, EBADF, strerror(EBADF));
        return 0;
    }

    unsigned char buf[128] = { 0 };
    int count = serialPortReadTimeout(port, buf, waitForLength, timeoutMilliseconds);
    if ((count == (int)waitForLength) && !memcmp(buf, waitFor, waitForLength))
    {
        return 1;
    }
    return 0;
}

int serialPortGetByteCountAvailableToRead(port_handle_t port)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnGetByteCountAvailableToRead, "serialPortGetByteCountAvailableToRead() is not supported for this port.");
    return SERIAL_PORT(port)->pfnGetByteCountAvailableToRead(port);
}

int serialPortGetByteCountAvailableToWrite(port_handle_t port)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnGetByteCountAvailableToWrite, "serialPortGetByteCountAvailableToWrite() is not supported for this port.");
    return SERIAL_PORT(port)->pfnGetByteCountAvailableToWrite(port);
}

int serialPortSleep(port_handle_t port, int sleepMilliseconds)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnSleep, "serialPortSleep() is not supported for this port.");
    return SERIAL_PORT(port)->pfnSleep(sleepMilliseconds);
}

int setSerialPortOnErrorCB(port_handle_t port, pfnSerialPortOnErrorCB onErrorCb)
{
    IS_PORT_FUNCTION_SUPPORTED(port, pfnError, "setSerialPortOnErrorCB() is not supported for this port.");
    SERIAL_PORT(port)->pfnError = onErrorCb;
    return PORT_ERROR__NONE;
}

