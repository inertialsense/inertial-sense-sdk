/**
 * @file ISBootloaderCommon.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating embedded systems
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISUtilities.h"
#include "ISComm.h"
#include "ISBootloaderCommon.h"
#include "serialPort.h"
#include "serialPortPlatform.h"
#include "bootloaderShared.h"

const char* is_uins_5_firmware_needle = "uINS-5";
const char* is_uins_3_firmware_needle = "uINS-3";
const char* is_evb_2_firmware_needle = "EVB-2";

is_device_context* is_create_context(
    is_device_handle* handle,
    is_device_match_properties* match_props,
    is_firmware_settings* firmware,
    int baud_rate,
    is_verification_style verify,
    pfnBootloadProgress upload_cb,
    pfnBootloadProgress verify_cb,
    pfnBootloadStatus info_cb,
    void* user_data
)
{
    is_device_context* ctx = malloc(sizeof(is_device_context));
    memset(ctx, 0, sizeof(is_device_context));

    memcpy(&ctx->handle, handle, sizeof(is_device_handle));
    memcpy(&ctx->match_props, match_props, sizeof(is_device_match_properties));
    memcpy(&ctx->firmware, firmware, sizeof(is_firmware_settings));
    ctx->baud_rate = baud_rate;
    ctx->verification_style = verify;
    ctx->update_progress_callback = upload_cb;
    ctx->verify_progress_callback = verify_cb;
    ctx->info_callback = info_cb;
    ctx->success = false;
    memset(ctx->error, 0, BOOTLOADER_ERROR_LENGTH);
    ctx->updateProgress = 0.0;
    ctx->verifyProgress = 0.0;
    ctx->user_data = user_data;
    ctx->update_in_progress = true;
    ctx->infoString_new = false;
    ctx->was_in_app = false;
    ctx->was_updated = false;

    return ctx;
}

void is_destroy_context(is_device_context* ctx)
{
    free(ctx);
}

is_operation_result is_jump_to_bootloader_from_app(is_device_context* ctx)
{
    // TODO: Implement
}

/**
 * @brief Look for a signature in the first section of a hex file
 * 
 * @param firmware 
 * @return is_device_type 
 */
is_device_type is_get_device_type_heximage(is_firmware_settings* firmware)
{
    ihex_image_section_t image;
    int sections = ihex_load_sections(firmware->firmware_path, &image, 1);
    size_t image_type;

    if(sections == 1)   // Signature must be in the first section of the image
    {
        uint8_t *target_signature;

        for(image_type = 0; image_type < IS_DEVICE_TYPE_UNKNOWN; image_type++)
        {
            switch(image_type)
            {
            case IS_DEVICE_TYPE_UINS_3: target_signature = bootloaderRequiredSignature_uINS_3; break;
            case IS_DEVICE_TYPE_UINS_5: target_signature = bootloaderRequiredSignature_uINS_5; break;
            case IS_DEVICE_TYPE_EVB_2: target_signature = bootloaderRequiredSignature_EVB_2; break;
            case IS_DEVICE_TYPE_STM32L4_BOOT: target_signature = bootloaderRequiredSignature_STM32L4_bootloader; break;
            case IS_DEVICE_TYPE_SAMX70_BOOT: target_signature = bootloaderRequiredSignature_SAMx70_bootloader; break;
            default: return IS_DEVICE_TYPE_UNKNOWN;
            }

            size_t k = 0;
            for(size_t j = 0; j < image.len; j++)
            {
                if(image.image[j] == target_signature[k]) k++;  // Found the right char, continue
                else k = 0; // Didn't find the right char, reset to beginning of search

                if(k >= BOOTLOADER_SIGNATURE_SIZE) return image_type;   // Found all the chars required
            }
        }
        
    }
    
    return IS_DEVICE_TYPE_UNKNOWN;
}

is_device_type is_get_device_type_binimage(is_firmware_settings* firmware)
{
    // TODO: Implement

    return IS_DEVICE_TYPE_UNKNOWN;
}

is_operation_result is_get_device_type_hdw(is_device_context* ctx)
{
    if(ctx->handle.status == IS_HANDLE_TYPE_LIBUSB) 
    {
        
    }
    else 
    {
        serialPortPlatformInit(&ctx->handle.port);
        serialPortSetPort(&ctx->handle.port, ctx->handle.port_name);
        if (serialPortOpenRetry(&ctx->handle.port, ctx->handle.port_name, ctx->baud_rate, 1) == 0)
        {
            serialPortClose(&ctx->handle.port);
            return IS_OP_ERROR;
        }    
        serialPortFlush(&ctx->handle.port);

        if(is_samba_init(ctx) == IS_OP_OK)
        {   // We have a SAM-BA device
            
        }
        else 
        {   // Assume we have an IS device or other device
            is_jump_to_bootloader_from_app(ctx);
            ctx->was_in_app = true;

            // TODO: Read parameters from the bootloader 
        }
    }
   




    

    

    

    // Get DID_DEV_INFO from the uINS.
    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    int messageSize;
    
    messageSize = is_comm_get_data(&comm, DID_DEV_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    // HACK: Send this twice. After leaving DFU mode, the serial port doesn't respond to the first request.
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_DEV_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_STATUS, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    messageSize = is_comm_get_data(&comm, DID_MANUFACTURING_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }

    SLEEP_MS(10);

    protocol_type_t ptype;
    int n = is_comm_free(&comm);
    dev_info_t* dev_info = NULL;
    dev_info_t* evb_dev_info = NULL;
    evb_status_t* evb_status = NULL;
    manufacturing_info_t* manufacturing_info = NULL;
    uint8_t evb_version[4];
    if ((n = serialPortReadTimeout(&ctx->handle.port, comm.buf.start, n, 200)))
    {
        comm.buf.tail += n;
        while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
        {
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_DEV_INFO)
            {
                dev_info = (dev_info_t*)comm.dataPtr;
                memcpy(ctx->hdw_info.uins_version, dev_info->hardwareVer, 4);
            }
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_EVB_DEV_INFO)
            {
                evb_dev_info = (dev_info_t*)comm.dataPtr;
                memcpy(evb_version, evb_dev_info->hardwareVer, 4);
            }
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_EVB_STATUS)
            {
                evb_status = (evb_status_t*)comm.dataPtr;
                if(evb_status->firmwareVer[0]) memcpy(ctx->hdw_info.evb_version, evb_version, 4);   // Only copy EVB status stuff if it is plugged into the EVB port
            }
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA && comm.dataHdr.id == DID_MANUFACTURING_INFO)
            {
                manufacturing_info = (manufacturing_info_t*)comm.dataPtr;
                sprintf(ctx->match_props.serial_number, "%X%X", manufacturing_info->uid[0] + manufacturing_info->uid[2], (uint16_t)(manufacturing_info->uid[1] >> 16));
            }
        }
    }
    
    serialPortClose(&ctx->handle.port);
    return IS_OP_OK;
}

/**
 * @brief Compatibility layer with old bootloader code in "inertialSenseBootLoader.c"
 * 
 * @param ctx 
 * @return is_operation_result 
 */
is_operation_result is_flash_compat(is_device_context* ctx)
{
    bootload_params_t params;

    params.uploadProgress = ctx->update_progress_callback;
    params.verifyProgress = ctx->verify_progress_callback;
    params.statusText = ctx->info_callback;
    params.fileName = ctx->firmware.firmware_path;
    params.forceBootloaderUpdate = ctx->firmware.bootloader_force_update;
    params.port = &ctx->handle.port;
    params.verifyFileName = NULL;   // TODO: Add verify
    params.flags.bitFields.enableVerify = (ctx->verification_style == IS_VERIFY_ON);
    params.baudRate = ctx->baud_rate;
    params.obj = ctx;
    params.bootName = (const char*)ctx->firmware.bootloader_path;
    params.ctx = (void*)ctx;

    if(ctx->hdw_info.evb_version[0] == 2) strncpy(params.bootloadEnableCmd, "EBLE", 5);
    else strncpy(params.bootloadEnableCmd, "BLEN", 5);

    if(ctx->scheme == IS_SCHEME_DFU) params.bootloaderUpdateCb = is_dfu_flash;
    else if(ctx->scheme == IS_SCHEME_SAMBA) params.bootloaderUpdateCb = is_samba_flash;
    else return IS_OP_ERROR;

    bootloadFileEx(&params);

    strncpy(ctx->error, params.error, BOOTLOADER_ERROR_LENGTH);

    return IS_OP_OK;
}

void is_update_flash(void* context)
{
    is_device_context* ctx = (is_device_context*)context;

	is_check_device_version(ctx);

    strncpy(ctx->bl_enable_command, "BLEN", 5);
    
	if((ctx->hdw_info.uins_version[0] == 5) && strstr(ctx->firmware.firmware_path, is_uins_5_firmware_needle))
	{
        ctx->scheme = IS_SCHEME_DFU;
	}
	else if((ctx->hdw_info.uins_version[0] == 4 || ctx->hdw_info.uins_version[0] == 3) && strstr(ctx->firmware.firmware_path, is_uins_3_firmware_needle))
	{
		ctx->scheme = IS_SCHEME_SAMBA;
	}
	else if(ctx->hdw_info.evb_version[0] == 2 && strstr(ctx->firmware.firmware_path, is_evb_2_firmware_needle))
	{
		strncpy(ctx->bl_enable_command, "EBLE", 5);
        ctx->scheme = IS_SCHEME_SAMBA;
	}
	else
	{   // Bootload based on filename alone
	    if(strstr(ctx->firmware.firmware_path, is_uins_5_firmware_needle))
		{
            ctx->scheme = IS_SCHEME_DFU;

			ctx->hdw_info.uins_version[0] = 5;
			ctx->hdw_info.evb_version[0] = 0;
		}
        else if(strstr(ctx->firmware.firmware_path, is_uins_3_firmware_needle))
		{
            ctx->scheme = IS_SCHEME_SAMBA;

			ctx->hdw_info.uins_version[0] = 4;
			ctx->hdw_info.evb_version[0] = 0;
		}
		else if(strstr(ctx->firmware.firmware_path, is_evb_2_firmware_needle))
		{
            strncpy(ctx->bl_enable_command, "EBLE", 5);

            ctx->scheme = IS_SCHEME_SAMBA;

			ctx->hdw_info.uins_version[0] = 0;
			ctx->hdw_info.evb_version[0] = 2;
		}
		else
		{
            ctx->update_in_progress = false;
			return;
		}
	}

    if(ctx->scheme == IS_SCHEME_SAMBA)
    {
        ctx->match_props.match = 
            IS_DEVICE_MATCH_FLAG_TYPE | 
            IS_DEVICE_MATCH_FLAG_MAJOR;
    }
    else if(ctx->scheme == IS_SCHEME_DFU)
    {
        ctx->match_props.match = 
            IS_DEVICE_MATCH_FLAG_VID | 
            IS_DEVICE_MATCH_FLAG_PID | 
            IS_DEVICE_MATCH_FLAG_TYPE | 
            IS_DEVICE_MATCH_FLAG_MAJOR |
            IS_DEVICE_MATCH_FLAG_SN;
    }
    else
    {
        ctx->update_in_progress = false;
        strcpy(ctx->error, "Scheme is not supported");
        return;
    }

    // Match serial number if present
    // if(strlen(ctx->match_props.serial_number[0]) != 0) ctx->match_props.match |= IS_DEVICE_MATCH_FLAG_SN;
    
    int ret = IS_OP_ERROR;
    ctx->success = false;
    
    // Start the bootloader and update flash
    ret = is_flash_compat(ctx);
    
    if(ret == IS_OP_OK) ctx->success = true;
    else strcpy(ctx->error, "Error in update flash");

    ctx->update_in_progress = false;
    return;
}
