/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

#include "../../src/ISBootloaderCommon.h"

static void on_error(const is_device_context const * ctx, const void* user_data, int error_code, const char * error_message)
{
	printf("ISDFUBootloaderExample::on_error Bootloader failed! Error: %d %s\n", error_code, error_message);
	printf("ISDFUBootloaderExample::on_error user data: %s\n", (const char *) user_data);
}

static int on_update_progress(const is_device_context const * ctx, const void* user_data, float percent)
{
	printf("ISDFUBootloaderExample::on_update_progress %f percent\n", percent);
	printf("user data: %s\n", (const char *) user_data);
	return 1; // return zero to abort
}

static int on_verify_progress(const is_device_context const * ctx, const void* user_data, float percent)
{
	printf("ISDFUBootloaderExample::on_verify_progress %f percent\n", percent);
	printf("user data: %s\n", (const char *) user_data);
	return 1; // return zero to abort
}

int main(int argc, char* argv[])
{
	int rc = -1;

	const char* firmware_file_path = argv[1];
	printf("firmware path: %s\n", firmware_file_path);
	
	is_device_context* ctx = is_create_device_context(firmware_file_path, "\0");

	libusb_context* lusb_ctx = NULL;
	libusb_device** device_list = NULL;	// Libusb allocates memory for the pointers
	libusb_device_handle* match_list[256];	// Libusb does not allocate memory for the pointers
	size_t device_count = 0, match_count = 0;
	is_get_libusb_handles(ctx, lusb_ctx, device_list, &device_count, match_list, &match_count);
	if(match_count < 1)
	{
		printf("No devices\n");
	}
	const char* user_data = "my own data";
	
	ctx->baud_rate = 921600;
	ctx->bootloader_file_path = NULL;
	ctx->firmware_file_path = firmware_file_path;
	ctx->force_bootloader_update = false;
	ctx->info_callback = info_callback;
	ctx->port_handle = match_list[0];
	ctx->port_id = NULL;
	ctx->update_progress_callback = progress_callback;
	ctx->user_data = user_data;
	ctx->verification_style = IS_VERIFY_OFF;
	ctx->verify_progress_callback = verify_callback;

	is_update_flash((void*)ctx);

	if (ctx->success)
	{
		printf("ISDFUBootloaderExample: Bootloader success on %s:%s with file %s\n", ctx->port_id, ctx->match_props.serial_number, firmware_file_path);
		rc = 0;
	}
	else
	{
		printf("ISDFUBootloaderExample: Bootloader failed to update %s:%s with file %s!\n", ctx->port_id, ctx->match_props.serial_number, firmware_file_path);
		rc = -1;
	}

	is_release_libusb_handles(device_list, match_list, match_count);
	is_destroy_device_context(ctx);
	
	return rc;
}

