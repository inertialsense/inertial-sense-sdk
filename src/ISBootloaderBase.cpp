/**
 * @file ISBootloaderBase.cpp
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

#include "ISBootloaderBase.h"
#include "ISBootloaderDFU.h"
#include "ISBootloaderSAMBA.h"
#include "ISBootloaderISB.h"
#include "ISBootloaderAPP.h"
#include "ihex.h"
#include "../hw-libs/bootloader/bootloaderShared.h"
#include "libusb.h"

using namespace ISBootloader;

const char* is_samx70_bootloader_needle = "SAMx70-Bootloader";

is_operation_result ISBootloader::dummy_update_callback(void* obj, float percent) 
{
    cISBootloaderBase* ctx = (cISBootloaderBase*)obj;
    ctx->m_update_progress = percent;
    return IS_OP_OK;
}

is_operation_result ISBootloader::dummy_verify_callback(void* obj, float percent) 
{
    cISBootloaderBase* ctx = (cISBootloaderBase*)obj;
    ctx->m_verify_progress = percent;
    return IS_OP_OK;
}

eImageSignature cISBootloaderBase::get_hex_image_signature(std::string filename)
{
    ihex_image_section_t image;
    size_t sections = ihex_load_sections(filename.c_str(), &image, 1);
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

                if(k >= BOOTLOADER_SIGNATURE_SIZE) return (eImageSignature)(1 << image_type);   // Found all the chars required
            }
        }
        
    }
    
    // Backup for old (16K) bootloader image
    if (strstr(filename.c_str(), is_samx70_bootloader_needle))
    {
        return IS_IMAGE_SIGN_ISB_SAMx70_16K;
    }

    return IS_IMAGE_SIGN_NONE;
}

eImageSignature cISBootloaderBase::get_bin_image_signature(std::string filename)
{
    return (eImageSignature)(IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);
}

const char* cISBootloaderBase::get_file_ext(const char *filename) 
{
    const char *dot = strrchr(filename, '.');   // Find last '.' in file name
    if(!dot || dot == filename) return "";
    return dot + 1;
}

is_operation_result cISBootloaderBase::add_device_to_list(std::string filename, libusb_device_handle* handle, const char* uid, cISBootloaderBase** obj, pfnBootloadStatus statusfn)
{
    const char * extension = cISBootloaderBase::get_file_ext(filename.c_str());
    eImageSignature file_signature = IS_IMAGE_SIGN_NONE;
    eImageSignature valid_signatures = IS_IMAGE_SIGN_NONE;

    if(strcmp(extension, "bin") == 0)
    {
        file_signature = cISBootloaderBase::get_bin_image_signature(filename);
    }
    else if(strcmp(extension, "hex") == 0)
    {
        file_signature = cISBootloaderBase::get_hex_image_signature(filename);
    }
    else
    {
        // TODO: Error message
        return IS_OP_ERROR;
    }

    if(cISBootloaderDFU::check_is_compatible(handle, file_signature) == IS_OP_OK)
    {
        *obj = new cISBootloaderDFU(dummy_update_callback, dummy_verify_callback, statusfn, handle, uid);  // TODO: Fix dummy callback
    }

    return IS_OP_ERROR;
}

is_operation_result cISBootloaderBase::add_device_to_list(std::string filename, const char* handle, cISBootloaderBase** obj, pfnBootloadStatus statusfn)
{
    const char * extension = cISBootloaderBase::get_file_ext(filename.c_str());
    eImageSignature file_signature = IS_IMAGE_SIGN_NONE;
    eImageSignature valid_signatures = IS_IMAGE_SIGN_NONE;
    bool old_bootloader_version = false;

    if(strcmp(extension, "bin") == 0)
    {
        file_signature = cISBootloaderBase::get_bin_image_signature(filename);
    }
    else if(strcmp(extension, "hex") == 0)
    {
        file_signature = cISBootloaderBase::get_hex_image_signature(filename);
    }
    else
    {
        // TODO: Error message
        return IS_OP_ERROR;
    }



    if(cISBootloaderSAMBA::check_is_compatible(handle, file_signature) == IS_OP_OK)
    {   /** SAM-BA MODE */
        *obj = new cISBootloaderSAMBA(dummy_update_callback, dummy_verify_callback, statusfn, handle);  // TODO: Fix dummy callback
    }
    else if(cISBootloaderISB::check_is_compatible(handle, file_signature) == IS_OP_OK)
    {   /** IS BOOTLOADER MODE */
        *obj = new cISBootloaderISB(dummy_update_callback, dummy_verify_callback, statusfn, handle);
    }
    else if (cISBootloaderAPP::check_is_compatible(handle, file_signature) == IS_OP_OK)
    {   /** APP MODE */
        *obj = new cISBootloaderAPP(dummy_update_callback, dummy_verify_callback, statusfn, handle);
    }
    else
    {
        return IS_OP_ERROR;
    }

    return IS_OP_OK;
}

is_operation_result cISBootloaderBase::reboot_to_update_level(std::string filename)
{
    const char * extension = cISBootloaderBase::get_file_ext(filename.c_str());
    eImageSignature file_signature = IS_IMAGE_SIGN_NONE;
    eImageSignature valid_signatures = IS_IMAGE_SIGN_NONE;
    bool old_bootloader_version = false;

    if(strcmp(extension, "bin") == 0)
    {
        file_signature = cISBootloaderBase::get_bin_image_signature(filename);
    }
    else if(strcmp(extension, "hex") == 0)
    {
        file_signature = cISBootloaderBase::get_hex_image_signature(filename);
    }
    else
    {
        // TODO: Error message
        return IS_OP_ERROR;
    }

    // Determine which mode the device needs to be in
    if(file_signature & IS_IMAGE_SIGN_APP)
    {
        // Should never be true
    }
    else if(file_signature & IS_IMAGE_SIGN_ISB)
    {
        switch(m_device_type)
        {
        case IS_DEV_TYPE_APP:
            reboot_down();
            break;
        case IS_DEV_TYPE_ISB:
            break;
        case IS_DEV_TYPE_SAMBA:
        case IS_DEV_TYPE_DFU:
            reboot_up();
            break;
        }
    }
    else if(file_signature & IS_IMAGE_SIGN_SAMBA || file_signature & IS_IMAGE_SIGN_DFU)
    {
        switch(m_device_type)
        {
        case IS_DEV_TYPE_APP:
        case IS_DEV_TYPE_ISB:
            reboot_down();
            break;
        case IS_DEV_TYPE_SAMBA:
        case IS_DEV_TYPE_DFU:
            break;
        }
    }

    return IS_OP_OK;
}

