/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
#include "../../src/inertialSenseBootLoader.h"

// print out upload progress
static int bootloaderUploadProgress(const void* obj, float percent)
{
	printf("Bootloader upload: %d percent...         \r", (int)(percent * 100.0f));
	if (percent >= 1.0f)
	{
		printf("\n");
	}
	return 1; // return zero to abort
}

// print out verify progress
static int bootloaderVerifyProgress(const void* obj, float percent)
{
	printf("Bootloader verify: %d percent...         \r", (int)(percent * 100.0f));
	if (percent >= 1.0f)
	{
		printf("\n");
	}
	return 1; // return zero to abort
}

int main(int argc, char* argv[])
{
	// serial port to communicate with the bootloader
	serial_port_t serialPort;

	// very important - the serial port must be initialized to zeros
	memset(&serialPort, 0, sizeof(serialPort));

	// buffer to show any errors
	char errorBuffer[512];

	// bootloader parameters
	bootload_params_t param;

	// very important - initialize the bootloader params to zeros
	memset(&param, 0, sizeof(param));

	if (argc < 3)
	{
		printf("Please pass the com port and then file name to bootload as the only arguments\r\n");
		// In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3 IS_uINS-3.hex" 
		return -1;
	}

	// initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
	//  you will need to either bootload from Windows, MAC or Linux, or implement your own code that
	//  implements all the function pointers on the serial_port_t struct.
	serialPortPlatformInit(&serialPort);

	// set the port - the bootloader uses this to open the port and enable bootload mode, etc.
	serialPortSetPort(&serialPort, argv[1]);

	// error buffer, useful if something fails
	param.error = errorBuffer;
	param.errorLength = sizeof(errorBuffer);

	// the file to bootload, *.hex
	param.fileName = argv[2];

	// the serial port
	param.port = &serialPort;

	// progress indicators
	param.uploadProgress = bootloaderUploadProgress;
	param.verifyProgress = bootloaderVerifyProgress;

	// enable verify to read back the firmware and ensure it is correct
	param.flags.bitFields.enableVerify = 1;

	// enable auto-baud, in the event that fast serial communications is not available,
	//  the bootloader will attempt to fall back to a slower speed
	param.flags.bitFields.enableAutoBaud = 1;

	// perform the bootload
	if (bootloadFileEx(&param))
	{
		printf("Bootloader success on port %s with file %s\n", serialPort.port, param.fileName);
		return 0;
	}
	else
	{
		printf("Bootloader failed! Error: %s\n", errorBuffer);
		return -1;
	}
}

