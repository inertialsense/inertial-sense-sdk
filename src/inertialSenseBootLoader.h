/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __INERTIALSENSEBOOTLOADER_H
#define __INERTIALSENSEBOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "serialPort.h"

/*! uINS bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER 2000000

/*! uINS rs232 bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER_RS232 230400

/*! uINS standard baud rate, used by bootloader code to enable bootloader mode */
#define IS_BAUD_RATE_BOOTLOADER_COM 3000000

#define ENABLE_BOOTLOADER_BAUD_DETECTION 1
#define BOOTLOADER_REFRESH_DELAY   20
#define BOOTLOADER_RESPONSE_DELAY  15 // needs to be > 10 for 2M baud
#if ENABLE_BOOTLOADER_BAUD_DETECTION
#define BOOTLOADER_RETRIES         16
#else
#define BOOTLOADER_RETRIES         1
#endif

/*! Bootloader callback function prototype, return value unused currently so return 0 */
typedef int(*pfnBootloadProgress)(const void* obj, float percent);

typedef struct
{
	const char* fileName; // read from this file
	serial_port_t* port; // connect with this serial port
	char* error; // error buffer, optional
	int errorLength; // number of bytes in error
	const void* obj; // user defined pointer
	pfnBootloadProgress uploadProgress; // upload progress
	pfnBootloadProgress verifyProgress; // verify progress
	const char* verifyFileName; // optional, writes verify file to the path if not 0
	int numberOfDevices; // number of devices if bootloading in parallel
    union
    {
        unsigned int bits;
        struct
        {
            unsigned int enableVerify : 1; // whether to enable the verify phase
			unsigned int enableAutoBaud : 1; // whether to enable auto-baud detection
        } bitFields;
    } flags;

} bootload_params_t;

/*!
Boot load a .hex or .bin file to a device

@param port the serial port to bootload to, will be opened and closed, must have port set
@param error a buffer to store any error messages in - can be NULL
@param errorLength the number of bytes available in error
@param obj custom user object that will be passed in the callback
@param uploadProgress called periodically during firmware upload - can be NULL
@param verifyProgress called periodically during firmware verification - can be NULL

@return 0 if failure, non-zero if success
*/
int bootloadFile(const char* fileName, serial_port_t* port, char* error, int errorLength, const void* obj, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress);
int bootloadFileEx(bootload_params_t* params);

/*!
Enable bootloader mode for a device

@param port the port to enable the bootloader on
@param error a buffer to store any error messages - can be NULL
@param errorLength the number of bytes available in error

@return 0 if failure, non-zero if success
*/
int enableBootloader(serial_port_t* port, char* error, int errorLength);

/*!
Disables the bootloader and goes back to program mode

@port the port to go back to program mode on
@error a buffer to store any error messages - can be NULL
@errorLength the number of bytes available in error

@return 0 if failure, non-zero if success
*/
int disableBootloader(serial_port_t* port, char* error, int errorLength);

#ifdef __cplusplus
}
#endif

#endif	// __INERTIALSENSEBOOTLOADER_H
