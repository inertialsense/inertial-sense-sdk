/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/uins_sdk_compat.h"

static void on_error(uins_device_interface* interface, const void* user_data, int error_code, const char * error_message)
{
	printf("Bootloader failed! Error: %d %s\n", error_code, error_message);
	// printf("user data: %s\n", user_data);
}

static int on_upload_progress(uins_device_interface* interface, const void* user_data, float percent)
{
	printf("Upload: %d percent...         \r", (int)(percent * 100.0f));
	if (percent >= 1.0f)
	{
		printf("\n");
	}

	// printf("user data: %s\n", user_data);
	return 1; // return zero to abort
}

static int on_verify_progress(uins_device_interface* interface, const void* user_data, float percent)
{
	printf("Verify: %d percent...         \r", (int)(percent * 100.0f));
	if (percent >= 1.0f)
	{
		printf("\n");
	}
	// printf("user data: %s\n", user_data);
	return 1; // return zero to abort
}

static void bootloaderStatusText(const void* obj, const char* info)
{
	printf("%s\n", info);
}

int main(int argc, char* argv[])
{
	// uins_create_device_interface(uins_31(), "file://dev/ttyACM0");
	uins_device_uri uri = "dfu://0483/df11/0/....";

	const char* hex_file = "/tmp/uins5-firmware.hex";

	uins_device_interface* uins = uins_create_device_interface(uins_50(), uri);

	char* user_data = "my own data";
	
	uins_operation_result bootloader_update_ok = uins_update_flash(
		uins,
		hex_file,
		IS_UPDATE_APPLICATION_FIRMWARE,
		IS_VERIFY_ON,
		on_error,
		on_upload_progress,
		on_verify_progress,
		user_data
	);

	uins_destroy_device_interface(uins);
	
	if (bootloader_update_ok)
	{
		printf("Bootloader success on %s with file %s\n", uri, hex_file);
		return 0;
	}
	else
	{
		printf("Bootloader failed!\n");
		return -1;
	}
}

