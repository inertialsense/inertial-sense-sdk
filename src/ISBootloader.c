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

const char* is_uins_5_firmware_needle = "uINS-5";
const char* is_uins_3_firmware_needle = "uINS-3";
const char* is_evb_2_firmware_needle = "EVB-2";
const char* is_samx70_bootloader_needle = "SAMx70";
const char* is_stm32l4_bootloader_needle = "STM32L4";

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

    serialPortPlatformInit(&ctx->handle.port);
    serialPortSetPort(&ctx->handle.port, handle->port_name);

    strncpy(ctx->firmware_path, firmware, IS_FIRMWARE_PATH_LENGTH);

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
static const char * get_file_ext(const char *filename) 
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
                    break;    
                case DID_EVB_DEV_INFO:
                    evb_dev_info = (dev_info_t*)comm.dataPtr;
                    memcpy(evb_version, evb_dev_info->hardwareVer, 4);
                    break;
                case DID_EVB_STATUS:
                    evb_status = (evb_status_t*)comm.dataPtr;
                    if(evb_status->firmwareVer[0]) memcpy(ctx->props.app.evb_version, evb_version, 4);
                    break;
                }
            }
        }
    }
    
    serialPortClose(&ctx->handle.port);

    if(ctx->props.app.uins_version[0] == 0 && ctx->props.app.evb_version[0] == 0) return IS_OP_ERROR;

    return IS_OP_OK;
}

/**
 * @brief Look for a signature in the first section of a hex file
 * 
 * @param firmware 
 * @return is_device_type 
 */
static is_image_signature is_get_hex_image_signature(const char* firmware)
{
    ihex_image_section_t image;
    size_t sections = ihex_load_sections(firmware, &image, 1);
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
            case IS_IMAGE_SIGN_ISB_SAMx70_16K: target_signature = bootloaderRequiredSignature_SAMx70_bootloader_16K; break;
            case IS_IMAGE_SIGN_ISB_SAMx70_24K: target_signature = bootloaderRequiredSignature_SAMx70_bootloader_24K; break;
            default: return 0;
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
    
    return 0;
}

static is_image_signature is_get_bin_image_signature(const char* firmware)
{
    // TODO: Implement

    return 0;
}

is_operation_result is_check_signature_compatibility(is_device_context* ctx)
{
    const char * extension = get_file_ext(ctx->firmware_path);
    is_image_signature file_signature = 0;
    is_image_signature valid_signatures = 0;

    if(strcmp(extension, "bin") == 0)
    {
        file_signature = is_get_bin_image_signature(ctx->firmware_path);
    }
    else if(strcmp(extension, "hex") == 0)
    {
        file_signature = is_get_hex_image_signature(ctx->firmware_path);
    }
    else
    {
        ctx->info_callback(ctx, "Invalid firmware filename extension");
        return IS_OP_ERROR;
    }

    // Get a bit field of valid signatures based on the type of device connected

    if(ctx->handle.status == IS_HANDLE_TYPE_LIBUSB)
    {   /** DFU MODE */
        valid_signatures |= IS_IMAGE_SIGN_ISB_STM32L4;
    }
    else if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL)
    {
        if(is_samba_init(ctx) == IS_OP_OK)
        {   /** SAM-BA MODE */
            valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
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
            }
        }
        else if(is_app_get_version(ctx) == IS_OP_OK)
        {   /** APP MODE */
            /* In App mode, we don't have a way to know whether we have a 16K or
             * 24K bootloader. Accept all uINS and EVB images that match. Call 
             * this function again when you are in ISB mode.
             */ 
            if(ctx->props.app.uins_version[0] == 5)
            {   /** uINS-5 */
                valid_signatures |= IS_IMAGE_SIGN_UINS_5;
                valid_signatures |= IS_IMAGE_SIGN_ISB_STM32L4;
            }
            else if(ctx->props.app.uins_version[0] == 3 || ctx->props.app.uins_version[0] == 4)
            {   /** uINS-3/4 */
                valid_signatures |= IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K;
                valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
            }
            
            if(ctx->props.app.evb_version[0] == 2)
            {   /** EVB-2 */
                valid_signatures |= IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K;
                valid_signatures |= IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K;
            }
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

    if(file_signature & valid_signatures)
    {
        return IS_OP_OK;
    }

    return IS_OP_ERROR;
}

void is_update_flash(void* context)
{
    is_device_context* ctx = (is_device_context*)context;

    if(is_check_signature_compatibility(ctx) != IS_OP_OK)
    {
        ctx->update_in_progress = false;
        return;
    }

    const char * extension = get_file_ext(ctx->firmware_path);
    is_image_signature file_signature = 0;

    if(strcmp(extension, "bin") == 0)
    {
        file_signature = is_get_bin_image_signature(ctx->firmware_path);
    }
    else if(strcmp(extension, "hex") == 0)
    {
        file_signature = is_get_hex_image_signature(ctx->firmware_path);
    }
    else
    {
        ctx->info_callback(ctx, "Invalid firmware filename extension");
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
    {   /** DFU MODE */
        is_dfu_flash(ctx);
    }
    else if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL)
    {
        if(is_samba_init(ctx) == IS_OP_OK)
        {   /** SAM-BA MODE */
            is_samba_flash(ctx);
        }
        else 
        {
            serialPortClose(&ctx->handle.port);
            serialPortOpenRetry(&ctx->handle.port, ctx->handle.port.port, IS_BAUDRATE_921600, 1);   // TODO make this work at other baudrates

            if(is_isb_get_version(ctx) == IS_OP_OK)
            {
                
            }
            else if(is_app_get_version(ctx) == IS_OP_OK)
            {
                is_isb_enable(ctx, ctx->props.isb.enable_command);   
                SLEEP_MS(1000);
                /*if(is_isb_get_version(ctx) != IS_OP_OK)
                {
                    ctx->update_in_progress = false;
                    return;
                }*/
            }
            else
            {
                ctx->update_in_progress = false;
                return;
            }
           
            if(file_signature & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4))
            {
                if(file_signature & IS_IMAGE_SIGN_ISB_STM32L4) 
                {
                    ctx->handle.status = IS_HANDLE_TYPE_LIBUSB;
                    ctx->handle.dfu.pid = STM32_DESCRIPTOR_PRODUCT_ID;
                    ctx->handle.dfu.vid = STM32_DESCRIPTOR_VENDOR_ID;
                    ctx->handle.dfu.sn = ctx->props.serial;
                    ctx->handle.dfu.uid[0] = 0; // Don't match the UID
                }

                is_isb_restart_rom(&ctx->handle.port);
                SLEEP_MS(1000);

                if(ctx->handle.status == IS_HANDLE_TYPE_LIBUSB)
                {   /** DFU MODE */
                    is_dfu_flash(ctx);
                }
                else if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL && is_samba_init(ctx) == IS_OP_OK)
                {
                    is_samba_flash(ctx);
                }
            }
            else
            {
                is_isb_flash(ctx);
            }
        }
    }
    else
    {   // Invalid handle status
        ctx->update_in_progress = false;
        return;
    }

    ctx->update_in_progress = false;
    return;
}
