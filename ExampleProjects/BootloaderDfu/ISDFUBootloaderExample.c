/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

#include "../../src/uins_sdk_compat.h"

static void on_error(const uins_device_interface const * interface, const void* user_data, int error_code, const char * error_message)
{
	printf("ISDFUBootloaderExample::on_error Bootloader failed! Error: %d %s\n", error_code, error_message);
	printf("ISDFUBootloaderExample::on_error user data: %s\n", (const char *) user_data);
}

static int on_update_progress(const uins_device_interface const * interface, const void* user_data, float percent)
{
	printf("ISDFUBootloaderExample::on_update_progress %f percent\n", percent);
	printf("user data: %s\n", (const char *) user_data);
	return 1; // return zero to abort
}

static int on_verify_progress(const uins_device_interface const * interface, const void* user_data, float percent)
{
	printf("ISDFUBootloaderExample::on_verify_progress %f percent\n", percent);
	printf("user data: %s\n", (const char *) user_data);
	return 1; // return zero to abort
}

static void bootloaderStatusText(const void* obj, const char* info)
{
	printf("%s\n", info);

}

static void listCallback(uins_device_uri uri)
{
	printf("found %s\n", uri);
}


int main(int argc, char* argv[])
{
	int rc = -1;

	// uins_create_device_interface(uins_31(), "file://dev/ttyACM0");

	const char* firmware_file_path = argv[1];
	printf("firmware path: %s\n", firmware_file_path);

	uins_device_uri_list uri_list;
	uins_probe_device_list(&uri_list, listCallback);

	if (uri_list.size < 1)
	{
		printf("failed to find dfu device");
		uins_free_device_list(&uri_list);
		return -1;
	}

	uins_device_uri uri = uri_list.devices[0];
	uins_device_interface* uins = uins_create_device_interface(uins_50(), uri);

	uins_change_log_level(uins, IS_LOG_LEVEL_DEBUG);

	const char* user_data = "my own data";
	
	uins_operation_result bootloader_update_ok = uins_update_flash(
		uins,
		firmware_file_path,
		IS_UPDATE_APPLICATION_FIRMWARE,
		IS_VERIFY_OFF,	// TODO: add verify
		on_error,
		on_update_progress,
		NULL,	// TODO: on_verify_progress
		user_data
	);

	if (bootloader_update_ok)
	{
		printf("ISDFUBootloaderExample: Bootloader success on %s with file %s\n", uri, firmware_file_path);
		rc = 0;
	}
	else
	{
		printf("ISDFUBootloaderExample: Bootloader failed to update %s with file %s!\n", uri, firmware_file_path);
		rc = -1;
	}

	uins_free_device_list(&uri_list);
	uins_destroy_device_interface(uins);
	
	return rc;
}

