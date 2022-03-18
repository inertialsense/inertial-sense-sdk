/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

#include "../../src/ISBootloaderCommon.h"

static void on_error(const is_device_interface const * interface, const void* user_data, int error_code, const char * error_message)
{
	printf("ISDFUBootloaderExample::on_error Bootloader failed! Error: %d %s\n", error_code, error_message);
	printf("ISDFUBootloaderExample::on_error user data: %s\n", (const char *) user_data);
}

static int on_update_progress(const is_device_interface const * interface, const void* user_data, float percent)
{
	printf("ISDFUBootloaderExample::on_update_progress %f percent\n", percent);
	printf("user data: %s\n", (const char *) user_data);
	return 1; // return zero to abort
}

static int on_verify_progress(const is_device_interface const * interface, const void* user_data, float percent)
{
	printf("ISDFUBootloaderExample::on_verify_progress %f percent\n", percent);
	printf("user data: %s\n", (const char *) user_data);
	return 1; // return zero to abort
}

static void bootloaderStatusText(const void* obj, const char* info)
{
	printf("%s\n", info);

}

static void listCallback(is_device_uri uri)
{
	printf("found %s\n", uri);
}


int main(int argc, char* argv[])
{
	int rc = -1;

	const char* firmware_file_path = argv[1];
	printf("firmware path: %s\n", firmware_file_path);
	
	is_device_interface* interface = is_create_device_interface(uins_5(0));

	libusb_context* context = NULL;
	libusb_device** device_list = NULL;	// Libusb allocates memory for the pointers
	libusb_device_handle* match_list[256];	// Libusb does not allocate memory for the pointers
	size_t device_count = 0, match_count = 0;
	is_get_libusb_handles(interface, context, device_list, &device_count, match_list, &match_count);
	if(match_count < 1)
	{
		printf("No devices\n");
	}

	is_device_change_log_level(interface, IS_LOG_LEVEL_DEBUG);

	const char* user_data = "my own data";
	
	is_operation_result bootloader_update_ok = is_update_flash(
		interface,
		firmware_file_path,
		IS_UPDATE_UINS_FIRMWARE,
		IS_VERIFY_OFF,	// TODO: add verify
		on_error,
		on_update_progress,
		NULL,	// TODO: on_verify_progress
		user_data,
		(void*)match_list[0]	// TODO: Multiple updates
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

	is_release_libusb_handles(device_list, match_list, match_count);
	is_destroy_device_interface(interface);
	is_free_device_list(&uri_list);
	
	return rc;
}

