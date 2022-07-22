/**
 * @file ISBootloader.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating firmware and bootloaders
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "serialPort.h"
#include "serialPortPlatform.h"

#include "ISComm.h"

#include "ISBootloader.h"
#include "ISBootloaderDFU.h"
#include "ISBootloaderISB.h"
#include "ISBootloaderSAMBA.h"
#include "ISUtilities.h"

#include "../hw-libs/bootloader/bootloaderShared.h"

const char* is_samx70_bootloader_needle = "SAMx70-Bootloader";

static is_operation_result dummy_update_callback(void* obj, float percent) 
{
    is_device_context* ctx = (is_device_context*)obj;
    ctx->update_progress = percent;
    return IS_OP_OK;
}

static is_operation_result dummy_verify_callback(void* obj, float percent) 
{
    is_device_context* ctx = (is_device_context*)obj;
    ctx->verify_progress = percent;
    return IS_OP_OK;
}

static void dummy_info_callback(void* obj, const char* infoString, is_log_level level)
{

}

is_device_context* is_create_context(
    is_device_handle* handle,
    const char* firmware,
    pfnBootloadProgress upload_cb,
    pfnBootloadProgress verify_cb,
    pfnBootloadStatus info_cb,
    void* user_data
)
{
    is_device_context* ctx = malloc(sizeof(is_device_context));
    memset(ctx, 0, sizeof(is_device_context));

    memcpy(&ctx->handle, handle, sizeof(is_device_handle));
    ctx->verify = verify_cb == NULL ? IS_VERIFY_OFF : IS_VERIFY_ON;
    ctx->update_callback = upload_cb;
    ctx->verify_callback = verify_cb;
    ctx->info_callback = info_cb;
    ctx->user_data = user_data;

    ctx->success = false;
    ctx->update_progress = 0.0;
    ctx->verify_progress = 0.0;
    ctx->update_in_progress = true;
    ctx->retries_left = 3;
    ctx->step_update_in_progress = false;
    ctx->device_type = IS_DEV_TYPE_NONE;

    serialPortPlatformInit(&ctx->handle.port);
    serialPortSetPort(&ctx->handle.port, handle->port_name);

    strncpy(ctx->firmware_path, firmware, IS_FIRMWARE_PATH_LENGTH);

    if(upload_cb == NULL) ctx->update_callback = dummy_update_callback;
    if(verify_cb == NULL) ctx->verify_callback = dummy_verify_callback;
    if(info_cb == NULL) ctx->info_callback = dummy_info_callback;

    return ctx;
}

void is_destroy_context(is_device_context* ctx)
{
    free(ctx);
}

/**
 * @brief Get the file extension from a file name
 * 
 * @param filename
 * @return const char* 
 */
static const char* get_file_ext(const char *filename) 
{
    const char *dot = strrchr(filename, '.');   // Find last '.' in file name
    if(!dot || dot == filename) return "";
    return dot + 1;
}

/**
 * @brief Fill the hardware version info from the application
 * 
 * @param ctx Device context, with configured serial port
 * @return is_operation_result IS_OP_OK if it successfully read parameters
 */
static is_operation_result is_app_get_version(is_device_context* ctx)
{
    // serialPortClose(&ctx->handle.port);
    // if (serialPortOpenRetry(&ctx->handle.port, ctx->handle.port_name, ctx->handle.baud, 1) == 0)
    // {
    //     serialPortClose(&ctx->handle.port);
    //     return IS_OP_ERROR;
    // }    
    serialPortFlush(&ctx->handle.port);

    // Get DID_DEV_INFO from the uINS.
    is_comm_instance_t comm;
    uint8_t buffer[2048];
    is_comm_init(&comm, buffer, sizeof(buffer));
    int messageSize;
    
    messageSize = is_comm_get_data(&comm, DID_DEV_INFO, 0, 0, 0);
    for(int i = 0; i < 2; i++)  // HACK: Send this twice. After leaving DFU mode, the serial port doesn't respond to the first request.
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        // serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_DEV_INFO, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        // serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }
    messageSize = is_comm_get_data(&comm, DID_EVB_STATUS, 0, 0, 0);
    if (messageSize != serialPortWrite(&ctx->handle.port, comm.buf.start, messageSize))
    {
        // serialPortClose(&ctx->handle.port);
        return IS_OP_ERROR;
    }

    serialPortSleep(&ctx->handle.port, 10);

    protocol_type_t ptype;
    int n = is_comm_free(&comm);
    dev_info_t* dev_info = NULL;
    dev_info_t* evb_dev_info = NULL;
    evb_status_t* evb_status = NULL;
    uint8_t evb_version[4];
    if ((n = serialPortReadTimeout(&ctx->handle.port, comm.buf.start, n, 200)))
    {
        comm.buf.tail += n;
        while ((ptype = is_comm_parse(&comm)) != _PTYPE_NONE)
        {
            if(ptype == _PTYPE_INERTIAL_SENSE_DATA)
            {
                switch(comm.dataHdr.id)
                {
                case DID_DEV_INFO:
                    dev_info = (dev_info_t*)comm.dataPtr;
                    memcpy(ctx->props.app.uins_version, dev_info->hardwareVer, 4);
                    ctx->props.serial = dev_info->serialNumber;
                    break;    
                case DID_EVB_DEV_INFO:
                    evb_dev_info = (dev_info_t*)comm.dataPtr;
                    memcpy(evb_version, evb_dev_info->hardwareVer, 4);
                    break;
                case DID_EVB_STATUS:
                    evb_status = (evb_status_t*)comm.dataPtr;
                    if(evb_status->firmwareVer[0]) memcpy(ctx->props.app.evb_version, evb_version, 4);
                    else memset(ctx->props.app.evb_version, 0, 4);
                    break;
                }
            }
        }
    }
    
    // serialPortClose(&ctx->handle.port);

    if(ctx->props.app.uins_version[0] == 0 && ctx->props.app.evb_version[0] == 0) return IS_OP_ERROR;

    return IS_OP_OK;
}

/**
 * @brief Look for a signature in the first section of a hex file
 * 
 * @param firmware 
 * @return is_device_type 
 */
static is_image_signature is_get_hex_image_signature(is_device_context* ctx)
{
    ihex_image_section_t image;
    size_t sections = ihex_load_sections(ctx->firmware_path, &image, 1);
    size_t image_type;

    if(sections == 1)   // Signature must be in the first section of the image
    {
        uint8_t *target_signature;

        for(image_type = 0; image_type < IS_IMAGE_SIGN_NUM_BITS_USED; image_type++)
        {
            switch(1 << image_type)
            {
            case IS_IMAGE_SIGN_UINS_3_16K: target_signature = bootloaderRequiredSignature_uINS_3_16K; break;
            case IS_IMAGE_SIGN_UINS_3_24K: target_signature = bootloaderRequiredSignature_uINS_3_24K; break;
            case IS_IMAGE_SIGN_EVB_2_16K: target_signature = bootloaderRequiredSignature_EVB_2_16K; break;
            case IS_IMAGE_SIGN_EVB_2_24K: target_signature = bootloaderRequiredSignature_EVB_2_24K; break;
            case IS_IMAGE_SIGN_UINS_5: target_signature = bootloaderRequiredSignature_uINS_5; break;
            case IS_IMAGE_SIGN_ISB_STM32L4: target_signature = bootloaderRequiredSignature_STM32L4_bootloader; break;
            case IS_IMAGE_SIGN_ISB_SAMx70_24K: target_signature = bootloaderRequiredSignature_SAMx70_bootloader_24K; break;
            default: continue;
            }

            size_t k = 0;
            for(size_t j = 0; j < image.len; j++)
            {
                if(image.image[j] == target_signature[k]) k++;  // Found the right char, continue
                else k = 0; // Didn't find the right char, reset to beginning of search

                if(k >= BOOTLOADER_SIGNATURE_SIZE) return 1 << image_type;   // Found all the chars required
            }
        }
        
    }
    
    // Backup for old (16K) bootloader image
    if (strstr(ctx->firmware_path, is_samx70_bootloader_needle))
    {
        return IS_IMAGE_SIGN_ISB_SAMx70_16K;
    }

    return 0;
}

static is_image_signature is_get_bin_image_signature(is_device_context* ctx)
{
    return IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
}

is_operation_result is_check_signature_compatibility(is_device_context* ctx)
{
    const char * extension = get_file_ext(ctx->firmware_path);
    is_image_signature file_signature = 0;
    is_image_signature valid_signatures = 0;
    bool old_bootloader_version = false;

    if(strcmp(extension, "bin") == 0)
    {
        file_signature = is_get_bin_image_signature(ctx);
    }
    else if(strcmp(extension, "hex") == 0)
    {
        file_signature = is_get_hex_image_signature(ctx);
    }
    else
    {
        ctx->info_callback(ctx, "Invalid firmware filename extension", IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    // Get a bit field of valid signatures based on the type of device connected

    if(ctx->handle.status == IS_HANDLE_TYPE_LIBUSB)
    {   /** DFU MODE */
        valid_signatures |= IS_IMAGE_SIGN_ISB_STM32L4;
        ctx->device_type = IS_DEV_TYPE_DFU;
    }
    else if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL)
    {
        if(is_samba_init(ctx) == IS_OP_OK)
        {   /** SAM-BA MODE */
            valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
            ctx->device_type = IS_DEV_TYPE_SAMBA;
        }
        else if(is_isb_handshake(ctx) == IS_OP_OK && is_isb_negotiate_version(ctx) == IS_OP_OK && is_isb_get_version(ctx) == IS_OP_OK)
        {   /** IS BOOTLOADER MODE */
            if(ctx->props.isb.major >= 6)   
            {   // v6 and has EVB detection built-in
                if(ctx->props.isb.processor == IS_PROCESSOR_SAMx70)
                {   
                    valid_signatures |= ctx->props.isb.is_evb ? IS_IMAGE_SIGN_EVB_2_24K : IS_IMAGE_SIGN_UINS_3_24K;
                    valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
                }
                else if(ctx->props.isb.processor == IS_PROCESSOR_STM32L4)
                {
                    valid_signatures |= IS_IMAGE_SIGN_UINS_5;
                    valid_signatures |= IS_IMAGE_SIGN_ISB_STM32L4;
                }
            }
            else
            {
                valid_signatures |= IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_UINS_3_16K;
                valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
                old_bootloader_version = true;
            }

            ctx->device_type = IS_DEV_TYPE_ISB;
        }
        else if (is_app_get_version(ctx) == IS_OP_OK)
        {   /** APP MODE */
            /* In App mode, we don't have a way to know whether we have a 16K or
             * 24K bootloader. Accept all uINS and EVB images that match. Call
             * this function again when you are in ISB mode.
             */
            if (ctx->props.app.uins_version[0] == 5)
            {   /** uINS-5 */
                valid_signatures |= IS_IMAGE_SIGN_UINS_5;
                valid_signatures |= IS_IMAGE_SIGN_ISB_STM32L4;
            }
            else if (ctx->props.app.uins_version[0] == 3 || ctx->props.app.uins_version[0] == 4)
            {   /** uINS-3/4 */
                valid_signatures |= IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K;
                valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
            }

            if (ctx->props.app.evb_version[0] == 2)
            {   /** EVB-2 */
                valid_signatures |= IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K;
                valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
            }

            ctx->device_type = IS_DEV_TYPE_APP;
        }
        else
        {
            return IS_OP_ERROR;
        }
    }
    else
    {   // Invalid handle status
        return IS_OP_ERROR;
    }

    // Compare the bitfields

    if (file_signature & valid_signatures)
    {
        return IS_OP_OK;
    }
    else if (old_bootloader_version && file_signature & (IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_UINS_3_24K))
    {
        ctx->info_callback(ctx, "If operation fails, please update the bootloader image to continue", IS_LOG_LEVEL_INFO);
    }

    return IS_OP_ERROR;
}

void is_update_flash(void* context)
{
    is_device_context* ctx = (is_device_context*)context;

    ctx->info_callback(ctx, "Checking compatibility of image with selected device", IS_LOG_LEVEL_INFO);
    if(is_check_signature_compatibility(ctx) != IS_OP_OK)
    {
        ctx->info_callback(ctx, "The image is incompatible with this device", IS_LOG_LEVEL_ERROR);
        ctx->step_update_in_progress = false;       // Allow retries with step_"
        return;
    }

    const char * extension = get_file_ext(ctx->firmware_path);
    is_image_signature file_signature = 0;

    if(strcmp(extension, "bin") == 0)
    {
        file_signature = is_get_bin_image_signature(ctx);
        if(!(file_signature & (IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_SAMx70_16K)))
        {
            ctx->info_callback(ctx, "Image must be .hex", IS_LOG_LEVEL_ERROR);
            ctx->update_in_progress = false;
            return;
        }
    }
    else if(strcmp(extension, "hex") == 0)
    {
        file_signature = is_get_hex_image_signature(ctx);
        if(file_signature & (IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_SAMx70_16K))
        {
            ctx->info_callback(ctx, "SAMx70 bootloader files must be .bin", IS_LOG_LEVEL_ERROR);
            ctx->update_in_progress = false;
            return;
        }
    }
    else
    {
        ctx->info_callback(ctx, "Invalid firmware filename extension", IS_LOG_LEVEL_ERROR);
        ctx->update_in_progress = false;
        return;
    }

    // In case we are updating using the inertial sense bootloader (ISB), set the entry command based on the signature
    if(file_signature & (IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K))
    {
        strncpy(ctx->props.isb.enable_command, "EBLE", 4);
    }
    else
    {
        strncpy(ctx->props.isb.enable_command, "BLEN", 4);
    }
    
    if(ctx->handle.status == IS_HANDLE_TYPE_LIBUSB)
    {
        if(ctx->device_type == IS_DEV_TYPE_DFU)
        {   /** DFU MODE */
            if(is_dfu_flash(ctx) == IS_OP_OK) ctx->update_in_progress = false;
            else { ctx->step_update_in_progress = false; return; }
        }
    }
    else if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL)
    {
        if(ctx->device_type == IS_DEV_TYPE_SAMBA)
        {   /** SAM-BA MODE */
            if(file_signature & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4))
            {
                if(is_samba_flash(ctx) == IS_OP_OK) ctx->update_in_progress = false;
                else { ctx->step_update_in_progress = false; return; }
            }
        }
        else if(ctx->device_type == IS_DEV_TYPE_ISB)
        {
            ctx->info_callback(ctx, "Found device in bootloader mode", IS_LOG_LEVEL_INFO);
            if(file_signature & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4))
            {
                is_isb_restart_rom(&ctx->handle.port);
                ctx->step_update_in_progress = false;
                return;
            }
            else
            {
                if(is_isb_flash(ctx) == IS_OP_OK) ctx->update_in_progress = false;
                else { ctx->step_update_in_progress = false; return; }
            }
        }
        else if(ctx->device_type == IS_DEV_TYPE_APP)
        {
            ctx->info_callback(ctx, "Found device in application mode", IS_LOG_LEVEL_INFO);
            is_isb_enable(ctx, ctx->props.isb.enable_command);
            ctx->step_update_in_progress = false;
            return;
        }
        else
        {
            ctx->info_callback(ctx, "Invalid device", IS_LOG_LEVEL_INFO);
            ctx->step_update_in_progress = false;
            return;
        }
    }
    else
    {   // Invalid handle status
        ctx->info_callback(ctx, "Invalid device", IS_LOG_LEVEL_INFO);
        ctx->update_in_progress = false;
        return;
    }

    ctx->success = true;
    return;
}
