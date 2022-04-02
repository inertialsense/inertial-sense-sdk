/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

#include "../../src/ISBootloaderCommon.h"

static void infoProgress(const void* obj, const char* info_string)
{
	printf("ISDFUBootloaderExample::on_error Bootloader failed! Error: %s\n", info_string);
	//printf("ISDFUBootloaderExample::on_error user data: %s\n", (const char *) user_data);
}

static int uploadProgress(const void* obj, float percent)
{
	printf("ISDFUBootloaderExample::on_update_progress %f percent\n", percent);
	// printf("user data: %s\n", (const char *) user_data);
	return 1; // return zero to abort
}

static int verifyProgress(const void* obj, float percent)
{
	printf("ISDFUBootloaderExample::on_verify_progress %f percent\n", percent);
	// printf("user data: %s\n", (const char *) user_data);
	return 1; // return zero to abort
}

int main(int argc, char* argv[])
{
	int rc = -1;

	char* fileName = argv[1];
	printf("firmware path: %s\n", fileName);
	char* comPort = argv[2];

	is_dfu_serial_list dfu_list;

	is_list_dfu(&dfu_list, STM32_DESCRIPTOR_VENDOR_ID, STM32_DESCRIPTOR_PRODUCT_ID);

	is_device_context* ctx;

	if (strstr(fileName, is_evb_2_firmware_needle) != NULL)
	{   // Enable EVB bootloader
		is_jump_to_bootloader(comPort, 921600, "EBLE");
		ctx = is_create_samba_context(fileName, comPort);
	}
	else if(strstr(fileName, is_uins_5_firmware_needle) != NULL)
	{	// Enable uINS-5 bootoader
		if(dfu_list.present == 0) return 0;
		if(comPort) is_jump_to_bootloader(comPort, 921600, "BLEN");
		ctx = is_create_dfu_context(fileName, dfu_list.list[0].sn);
	}
	else if(strstr(fileName, is_uins_3_firmware_needle) != NULL)
	{	// Enable uINS-3 bootloader
		is_jump_to_bootloader(comPort, 921600, "BLEN");
		ctx = is_create_samba_context(fileName, comPort);
	}

	if(!ctx) return 0;

	// Update application and bootloader firmware
	memset(ctx->error, 0, BOOTLOADER_ERROR_LENGTH);
	ctx->baud_rate = 921600;
	ctx->firmware_file_path = fileName;
	ctx->bootloader_file_path = NULL;
	ctx->update_progress_callback = uploadProgress;
	ctx->verify_progress_callback = verifyProgress;
	ctx->info_callback = infoProgress;
	ctx->force_bootloader_update = false;
	ctx->success = false;
	is_update_flash(ctx);

	if (ctx->success)
	{
		printf("ISDFUBootloaderExample: Bootloader success on %s:%s with file %s\n", ctx->handle.port_name, ctx->match_props.serial_number, fileName);
		rc = 0;
	}
	else
	{
		printf("ISDFUBootloaderExample: Bootloader failed to update %s:%s with file %s!\n", ctx->handle.port_name, ctx->match_props.serial_number, fileName);
		rc = -1;
	}
	
	return rc;
}

