/**
 * @file ISBootloader.cpp
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating embedded systems
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloader.h"

vector<is_device_context*> ISBootloader::ctx;

is_operation_result ISBootloader::update(
	vector<string>&             comPorts,
	int                         baudRate,
	is_firmware_settings*       firmware,
	pfnBootloadProgress         uploadProgress, 
	pfnBootloadProgress         verifyProgress, 
	pfnBootloadStatus           infoProgress
)
{
	is_dfu_list dfu_list;
	size_t i;

	if(ctx.size() > 0) 
	{
		infoProgress(NULL, "Error: bootloader already busy - devices present in bootload context list");
		return IS_OP_ERROR;
	}

    if(libusb_init(NULL) < 0) return IS_OP_ERROR;
	is_list_dfu(&dfu_list);

	for(i = 0; i < dfu_list.present; i++)
	{	// Create contexts for devices already in DFU mode
		is_device_handle handle;
		memset(&handle, 0, sizeof(is_device_handle));
		handle.status = IS_HANDLE_TYPE_LIBUSB;
		ctx.push_back(is_create_context(
			&handle, 
			firmware, 
			baudRate, 
			verifyProgress != NULL ? IS_VERIFY_ON : IS_VERIFY_OFF, 
			uploadProgress, 
			verifyProgress, 
			infoProgress
		));
		memcpy(&ctx[i]->match_props.serial_number, &dfu_list.id[i].sn, IS_SN_MAX_SIZE);
		ctx[i]->match_props.vid = dfu_list.id[i].usb.vid;
		ctx[i]->match_props.pid = dfu_list.id[i].usb.pid;
	}

	for(i = 0; i < comPorts.size(); i++)
	{	// Create contexts for devices still in serial mode
		is_device_handle handle;
		memset(&handle, 0, sizeof(is_device_handle));
		strncpy(handle.port_name, comPorts[i].c_str(), 256);
		handle.status = IS_HANDLE_TYPE_SERIAL;
		ctx.push_back(is_create_context(
			&handle, 
			firmware, 
			baudRate, 
			verifyProgress != NULL ? IS_VERIFY_ON : IS_VERIFY_OFF, 
			uploadProgress, 
			verifyProgress, 
			infoProgress
		));
	}

	for(i = 0; i < ctx.size(); i++)
	{	// Start threads
		ctx[i]->thread = threadCreateAndStart(update_thread, (void*)ctx[i]);
	}

	for (i = 0; i < ctx.size(); i++)
	{	// Wait for threads to finish
		if(ctx[i] != NULL) threadJoinAndFree(ctx[i]->thread);
	}

	libusb_exit(NULL);

	return IS_OP_OK;
}

void ISBootloader::update_thread(void* context)
{
	is_device_context* ctx = (is_device_context*)context;

	is_check_version(ctx);

	if((ctx->hdw_info.uins_version[0] == 5 || ctx->handle.status == IS_HANDLE_TYPE_LIBUSB) &&
		strstr(ctx->firmware.uins_5_firmware_path, is_uins_5_firmware_needle))
	{
		ctx->match_props.vid = STM32_DESCRIPTOR_VENDOR_ID;
		ctx->match_props.pid = STM32_DESCRIPTOR_PRODUCT_ID;

		is_init_dfu_context(ctx);
		is_jump_to_bootloader(ctx);
	}
	else if(ctx->hdw_info.uins_version[0] == 4 && strstr(ctx->firmware.uins_4_firmware_path, is_uins_3_firmware_needle))
	{
		is_init_samba_context(ctx);
	}
	else if(ctx->hdw_info.uins_version[0] == 3 && strstr(ctx->firmware.uins_3_firmware_path, is_uins_3_firmware_needle))
	{
		is_init_samba_context(ctx);
	}
	else if(ctx->hdw_info.evb_version[0] == 2 && strstr(ctx->firmware.evb_2_firmware_path, is_evb_2_firmware_needle))
	{
		is_init_samba_context(ctx);
	}
	else
	{
		// Assume that we have a SAM-BA bootloader, and bootload based on filename, with uINS-4, uINS-3, EVB-2 in that order
		if(strstr(ctx->firmware.uins_4_firmware_path, is_uins_3_firmware_needle))
		{
			ctx->hdw_info.uins_version[0] = 4;
			ctx->hdw_info.evb_version[0] = 0;
		}
		else if(strstr(ctx->firmware.uins_3_firmware_path, is_uins_3_firmware_needle))
		{
			ctx->hdw_info.uins_version[0] = 3;
			ctx->hdw_info.evb_version[0] = 0;
		}
		else if(strstr(ctx->firmware.evb_2_firmware_path, is_evb_2_firmware_needle))
		{
			ctx->hdw_info.uins_version[0] = 0;
			ctx->hdw_info.evb_version[0] = 2;
		}
		else
		{
			return;
		}
		
		is_init_samba_context(ctx);
	}

	is_update_flash((void*)ctx);
}