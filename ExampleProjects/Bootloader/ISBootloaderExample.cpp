/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <vector>
#include <string>
#include <algorithm>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
#include "../../src/ISBootloaderThread.h"
#include "../../src/ISBootloaderBase.h"
#include "../../src/ISSerialPort.h"

using namespace ISBootloader;
using namespace std;

// print out upload progress
static is_operation_result bootloaderUploadProgress(void* obj, float pct)
{
	if (obj == NULL) return IS_OP_OK;

	cISBootloaderBase* ctx = (cISBootloaderBase*)obj;
	int percent = (int)(pct * 100.0f);
	printf("\rUpload Progress: %d%%\r", percent);
	ctx->m_update_progress = percent;

	return IS_OP_OK;
}

// print out verify progress
static is_operation_result bootloaderVerifyProgress(void* obj, float pct)
{
	if (obj == NULL) return IS_OP_OK;

	cISBootloaderBase* ctx = (cISBootloaderBase*)obj;
	int percent = (int)(pct * 100.0f);
	printf("\rVerify Progress: %d%%\r", percent);
	ctx->m_verify_progress = percent;

	return IS_OP_OK;
}

static void bootloaderStatusText(void* obj, eLogLevel level, const char* str, ...)
{
	if (obj == NULL) return;

    static char buffer[256];

    va_list ap;
    va_start(ap, str);
    vsnprintf(buffer, sizeof(buffer) - 1, str, ap);
    va_end(ap);


	cISBootloaderBase* ctx = (cISBootloaderBase*)obj;

	if (ctx->m_sn != 0 && ctx->m_port_name.size() != 0)
	{
		printf("%s (SN%d):", ctx->m_port_name.c_str(), ctx->m_sn);
	}
	else if(ctx->m_sn != 0)
	{
		printf("(SN%d):", ctx->m_sn);
	}
	else if (ctx->m_port_name.size() != 0)
	{
		printf("%s:", ctx->m_port_name.c_str());
	}
	else
	{
		printf("SN?:");
	}

	printf("\t\t\t%s\r\n", buffer);
}

int main(int argc, char* argv[])
{
	if (argc < 4 || argc > 5)
	{
		printf("Please pass the com port, baudrate, firmware file name to bootload, and optionally bootloader file name as the only arguments\r\n");
		printf("usage: %s {COMx} {Baudrate} {Firmware file} {Bootloader file (optional)}\r\n", argv[0]);
		// In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3 IS_uINS-3.hex" 
		return -1;
	}

	// For now, we will use all present devices.
	std::vector<std::string> portStrings;
	cISSerialPort::GetComPorts(portStrings);

	// Set all files the same, the bootloader logic will identify the file and only put it onto the appropriate devices.
	firmwares_t files;
	files.fw_uINS_3.path = std::string(argv[2]);
	files.bl_uINS_3.path = std::string(argv[2]);
	files.fw_IMX_5.path = std::string(argv[2]);
	files.bl_IMX_5.path = std::string(argv[2]);
	files.fw_EVB_2.path = std::string(argv[2]);
	files.bl_EVB_2.path = std::string(argv[2]);

	

	vector<string> all_ports;                   // List of ports connected

	// For now, we will use all present devices.
    cISSerialPort::GetComPorts(all_ports);

    // Update the firmware on any port that was open
    std::vector<cISBootloaderThread::confirm_bootload_t> confirm_device_list;
	if (!cISBootloaderThread::set_mode_and_check_devices(
			all_ports,
			atoi(argv[1]),
			files,
			bootloaderUploadProgress,
			bootloaderVerifyProgress,
			bootloaderStatusText,
			NULL,
			&confirm_device_list))
		return -1;   // Error or no devices found

    cISSerialPort::GetComPorts(all_ports);

    // Update the firmware on any port that wasn't initially deselected
    // update the firmware on any port that was open
	cISBootloaderThread::update(
		portStrings,
		true,
		atoi(argv[1]),
		files,
		bootloaderUploadProgress,
		bootloaderVerifyProgress,
		bootloaderStatusText,
		NULL);


	return 0;
}

