/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
#include "../../src/ISBootloader.h"
#include "../../src/ISBootloaderDFU.h"

// print out upload progress
static is_operation_result bootloaderUploadProgress(const void* obj, float percent)
{
	if (obj == NULL) return IS_OP_OK;

	is_device_context* ctx = (is_device_context*)obj;
	percent = (int)(percent * 100.0f);
	printf("\rUpload Progress: %d%%\r", percent);
	ctx->update_progress = percent;

	return ctx->update_in_progress ? IS_OP_OK : IS_OP_CANCELLED;
}

// print out verify progress
static is_operation_result bootloaderVerifyProgress(const void* obj, float percent)
{
	if (obj == NULL) return IS_OP_OK;

	is_device_context* ctx = (is_device_context*)obj;
	percent = (int)(percent * 100.0f);
	printf("\rVerify Progress: %d%%\r", percent);
	ctx->update_progress = percent;

	return ctx->update_in_progress ? IS_OP_OK : IS_OP_CANCELLED;
}

static void bootloaderStatusText(const void* obj, const char* info)
{
	if (obj == NULL) return;

	is_device_context* ctx = (is_device_context*)obj;

	if (ctx->props.serial != 0)
	{
		printf("SN%d: %s\r\n", ctx->props.serial, info);
	}
	else if (ctx->handle.dfu.sn != 0)
	{
		printf("SN%d: %s\r\n", ctx->handle.dfu.sn, info);
	}
	else if (strlen(ctx->handle.port_name) != 0)
	{
		printf("%s: %s\r\n", ctx->handle.port_name, info);
	}
	else
	{
		printf("Unknown: %s\r\n", info);
	}
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

	is_device_handle handle;
	memset(&handle, 0, sizeof(is_device_handle));
	strncpy(handle.port_name, argv[1], 100);
	handle.status = IS_HANDLE_TYPE_SERIAL;
	handle.baud = atoi(argv[2]);
	is_device_context* ctx = is_create_context(
		&handle,
		argv[3],
		bootloaderUploadProgress,
		bootloaderVerifyProgress,
		bootloaderStatusText,
		NULL
	);

	is_update_flash(ctx);

	is_destroy_context(ctx);
}

