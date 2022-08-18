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
#include "ISUtilities.h"

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

eImageSignature cISBootloaderBase::get_image_signature(std::string filename)
{
    const char * extension = cISBootloaderBase::get_file_ext(filename.c_str());

    if(strcmp(extension, "bin") == 0)
    {
        return cISBootloaderBase::get_bin_image_signature(filename);
    }
    else if(strcmp(extension, "hex") == 0)
    {
        return cISBootloaderBase::get_hex_image_signature(filename);
    }
    
    return IS_IMAGE_SIGN_NONE;
}

is_operation_result cISBootloaderBase::mode_device
(
    firmwares_t filenames,
    serial_port_t* handle,
    pfnBootloadStatus statusfn,
    pfnBootloadProgress updateProgress,
    pfnBootloadProgress verifyProgress,
    std::vector<cISBootloaderBase*>& contexts,
    std::mutex* addMutex,
    cISBootloaderBase** new_context
)
{
    cISBootloaderBase* obj;
    *new_context = NULL;

    uint32_t device = IS_IMAGE_SIGN_NONE;
    uint32_t fw_uINS_3 = get_image_signature(filenames.fw_uINS_3.path) & (IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K);
    uint32_t bl_uINS_3 = get_image_signature(filenames.bl_uINS_3.path) & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);
    uint32_t fw_IMX_5 = get_image_signature(filenames.fw_IMX_5.path) & IS_IMAGE_SIGN_UINS_5;
    uint32_t bl_IMX_5 = get_image_signature(filenames.bl_IMX_5.path) & IS_IMAGE_SIGN_ISB_STM32L4;
    uint32_t fw_EVB_2  = get_image_signature(filenames.fw_EVB_2.path)  & (IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K);
    uint32_t bl_EVB_2  = get_image_signature(filenames.bl_EVB_2.path)  & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);

    obj = new cISBootloaderISB(updateProgress, verifyProgress, statusfn, handle);
    (obj)->m_port_name = std::string(handle->port);
    device = (obj)->check_is_compatible(); 
    if(device == IS_IMAGE_SIGN_ERROR)
    {
        delete obj;
    }
    else if(device)
    {
        if ((device & IS_IMAGE_SIGN_ISB) & bl_EVB_2)
        {
            (obj)->m_filename = filenames.bl_EVB_2.path;
            is_operation_result op = (obj)->reboot_down();
            if (op == IS_OP_OK || op == IS_OP_CLOSED)
            {
                delete obj;
                return IS_OP_CLOSED;
            }
            else if (op == IS_OP_RETRY)
            {
                (obj)->reboot_force();
                delete obj;
                return IS_OP_CLOSED;
            }
        }
        else if ((device & IS_IMAGE_SIGN_ISB) & bl_IMX_5)
        {
            (obj)->m_filename = filenames.bl_IMX_5.path;
            is_operation_result op = (obj)->reboot_down();
            if (op == IS_OP_OK || op == IS_OP_CLOSED)
            {
                delete obj;
                return IS_OP_CLOSED;
            }
            else if (op == IS_OP_RETRY)
            {
                (obj)->reboot_force();
                delete obj;
                return IS_OP_CLOSED;
            }
        }
        else if ((device & IS_IMAGE_SIGN_ISB) & bl_uINS_3)
        {
            (obj)->m_filename = filenames.bl_uINS_3.path;
            is_operation_result op = (obj)->reboot_down();
            if (op == IS_OP_OK || op == IS_OP_CLOSED)
            {
                delete obj;
                return IS_OP_CLOSED;
            }
            else if (op == IS_OP_RETRY)
            {
                (obj)->reboot_force();
                delete obj;
                return IS_OP_CLOSED;
            }
        }
        else
        {
            delete obj;
            return IS_OP_CLOSED;
        }
    }
    else
    {
        delete obj;
    }

    obj = new cISBootloaderAPP(updateProgress, verifyProgress, statusfn, handle);
    (obj)->m_port_name = std::string(handle->port);
    device = (obj)->check_is_compatible();
    if(device)
    {
        if ((device & IS_IMAGE_SIGN_APP) & fw_EVB_2)    // Firmware for EVB-2 must be specified to update its bootloader
        {
            strncpy((obj)->m_app.enable_command, "EBLE", 5);
            (obj)->reboot_down();
            delete obj;
            return IS_OP_CLOSED;
        }
        else if ((device & IS_IMAGE_SIGN_APP) & fw_IMX_5)
        {
            (obj)->m_filename = filenames.fw_IMX_5.path;
            strncpy((obj)->m_app.enable_command, "BLEN", 5);
            (obj)->reboot_down();
            delete obj;
            return IS_OP_CLOSED;
        }
        else if ((device & IS_IMAGE_SIGN_APP) & fw_uINS_3)
        {
            (obj)->m_filename = filenames.fw_uINS_3.path;
            strncpy((obj)->m_app.enable_command, "BLEN", 5);
            (obj)->reboot_down();
            delete obj;
            return IS_OP_CLOSED;
        }
        else if ((device & IS_IMAGE_SIGN_APP) & bl_uINS_3)
        {
            (obj)->m_filename = filenames.bl_uINS_3.path;
            strncpy((obj)->m_app.enable_command, "BLEN", 5);
            (obj)->reboot_down();
            delete obj;
            return IS_OP_CLOSED;
        }
        else if ((device & IS_IMAGE_SIGN_APP) & bl_IMX_5)
        {
            (obj)->m_filename = filenames.bl_IMX_5.path;
            strncpy((obj)->m_app.enable_command, "BLEN", 5);
            (obj)->reboot_down();
            delete obj;
            return IS_OP_CLOSED;
        }
        
        delete obj;
        return IS_OP_CANCELLED;
    }
    else
    {
        delete obj;
    }

    return IS_OP_OK;
}

is_operation_result cISBootloaderBase::update_device
(
    firmwares_t filenames,
    libusb_device_handle* handle,
    pfnBootloadStatus statusfn,
    pfnBootloadProgress updateProgress,
    pfnBootloadProgress verifyProgress,
    std::vector<cISBootloaderBase*>& contexts,
    std::mutex* addMutex,
    cISBootloaderBase** new_context
)
{
    cISBootloaderBase* obj;
    *new_context = NULL;

    uint32_t device = IS_IMAGE_SIGN_NONE;
    uint32_t bl_IMX_5 = get_image_signature(filenames.bl_IMX_5.path) & IS_IMAGE_SIGN_ISB_STM32L4;

    obj = new cISBootloaderDFU(updateProgress, verifyProgress, statusfn, handle);
    device = (obj)->check_is_compatible();
    if ((device & IS_IMAGE_SIGN_DFU) & bl_IMX_5)
    {
        (obj)->m_filename = filenames.bl_IMX_5.path;
        
        (obj)->get_device_info();
        (obj)->m_use_progress = true;
        addMutex->lock();
        contexts.push_back(obj);
        *new_context = obj;
        addMutex->unlock();
        if((obj)->download_image(filenames.bl_IMX_5.path) != IS_OP_OK)
        {
            (obj)->m_info_callback((obj), "(DFU) Update failed, retrying...", IS_LOG_LEVEL_ERROR);
            (obj)->m_use_progress = false;
            libusb_close(handle);
            return IS_OP_CLOSED;
        }
        (obj)->reboot_up();    // Reboot up right away so an App update can happen
        return IS_OP_CLOSED;
    }
    else
    {
        delete obj;
    }

    return IS_OP_ERROR;
}

is_operation_result cISBootloaderBase::update_device
(
    firmwares_t filenames,
    serial_port_t* handle,
    pfnBootloadStatus statusfn,
    pfnBootloadProgress updateProgress,
    pfnBootloadProgress verifyProgress,
    std::vector<cISBootloaderBase*>& contexts,
    std::mutex* addMutex,
    cISBootloaderBase** new_context
)
{
    cISBootloaderBase* obj;
    *new_context = NULL;

    uint32_t device = IS_IMAGE_SIGN_NONE;
    uint32_t fw_uINS_3 = get_image_signature(filenames.fw_uINS_3.path) & (IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K);
    uint32_t bl_uINS_3 = get_image_signature(filenames.bl_uINS_3.path) & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);
    uint32_t fw_IMX_5 = get_image_signature(filenames.fw_IMX_5.path) & IS_IMAGE_SIGN_UINS_5;
    uint32_t fw_EVB_2  = get_image_signature(filenames.fw_EVB_2.path)  & (IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K);
    uint32_t bl_EVB_2  = get_image_signature(filenames.bl_EVB_2.path)  & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);

    if(bl_EVB_2 || bl_uINS_3)
    {
        obj = new cISBootloaderSAMBA(updateProgress, verifyProgress, statusfn, handle);
        obj->m_port_name = std::string(handle->port);
        device = (obj)->check_is_compatible();
        if (device)
        {
            if((device & IS_IMAGE_SIGN_SAMBA) & bl_EVB_2)
            {
                (obj)->m_filename = filenames.bl_EVB_2.path;
                (obj)->get_device_info();
                (obj)->m_use_progress = true;
                addMutex->lock();
                contexts.push_back(obj);
                *new_context = obj;
                addMutex->unlock();
                if((obj)->download_image(filenames.bl_EVB_2.path) != IS_OP_OK)
                {
                    (obj)->m_use_progress = false;
                    //delete obj;  // Don't delete, since we have probably called the update and verify callbacks
                    return IS_OP_CLOSED;
                }
                if((obj)->verify_image(filenames.bl_EVB_2.path) != IS_OP_OK)
                {
                    (obj)->m_use_progress = false;
                    //delete obj;  // Don't delete, since we have probably called the update and verify callbacks
                    return IS_OP_CLOSED;
                }
                (obj)->reboot_up();    // Reboot up right away so an App update can happen
                return IS_OP_CLOSED;
            } 
            else if((device & IS_IMAGE_SIGN_SAMBA) & bl_uINS_3)
            {
                (obj)->m_filename = filenames.bl_uINS_3.path;
                (obj)->get_device_info();
                (obj)->m_use_progress = true;
                addMutex->lock();
                contexts.push_back(obj);
                *new_context = obj;
                addMutex->unlock();
                if((obj)->download_image(filenames.bl_uINS_3.path) != IS_OP_OK)
                {
                    (obj)->m_use_progress = false;
                    //delete obj;  // Don't delete, since we have probably called the update and verify callbacks
                    return IS_OP_CLOSED;
                }
                if((obj)->verify_image(filenames.bl_uINS_3.path) != IS_OP_OK)
                {
                    (obj)->m_use_progress = false;
                    //delete obj;  // Don't delete, since we have probably called the update and verify callbacks
                    return IS_OP_CLOSED;
                }
                (obj)->reboot_up();    // Reboot up right away so an App update can happen
                return IS_OP_CLOSED;
            } 
            else
            {
                delete obj;
                return IS_OP_CANCELLED;
            }
        }
        else
        {
            delete obj;
        }
    }

    obj = new cISBootloaderISB(updateProgress, verifyProgress, statusfn, handle);
    (obj)->m_port_name = std::string(handle->port);
    device = (obj)->check_is_compatible(); 
    if(device == IS_IMAGE_SIGN_ERROR)
    {
        delete obj;
    }
    else if(device)
    {
        // Bootloader was already updated or not specified
        if ((device & IS_IMAGE_SIGN_ISB) & fw_EVB_2)
        {
            (obj)->m_filename = filenames.fw_EVB_2.path;

            if((obj)->get_device_info() != IS_OP_OK)
            {
                delete obj;
                return IS_OP_CLOSED;
            }

            is_operation_result reboot_status = (obj)->reboot();
            if (reboot_status == IS_OP_OK)
            {
                delete obj;
                return IS_OP_CLOSED;
            }
            else if (reboot_status == IS_OP_CLOSED)
            {
                delete obj;
                return IS_OP_ERROR;
            }
            else
            {
                // usually IS_OP_ERROR, this indicates the serial number has already been reset. Continue to update
            }

            (obj)->m_use_progress = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            if((obj)->download_image(filenames.fw_EVB_2.path) != IS_OP_OK)
            {
                (obj)->m_info_callback((obj), "(ISB) Update failed, retrying...", IS_LOG_LEVEL_ERROR);
                (obj)->m_use_progress = false;
                (obj)->reboot_force();
                return IS_OP_CLOSED;
            }
            return IS_OP_OK;
        }
        else if ((device & IS_IMAGE_SIGN_ISB) & fw_IMX_5)
        {
            (obj)->m_filename = filenames.fw_IMX_5.path;

            if((obj)->get_device_info() != IS_OP_OK)
            {
                delete obj;
                return IS_OP_CLOSED;
            }

            is_operation_result reboot_status = (obj)->reboot();
            if (reboot_status == IS_OP_OK)
            {
                delete obj;
                return IS_OP_CLOSED;
            }
            else if (reboot_status == IS_OP_CLOSED)
            {
                delete obj;
                return IS_OP_ERROR;
            }
            else
            {
                // usually IS_OP_ERROR, this indicates the serial number has already been reset. Continue to update
            }

            (obj)->m_use_progress = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            if((obj)->download_image(filenames.fw_IMX_5.path) != IS_OP_OK)
            {
                (obj)->m_info_callback((obj), "(ISB) Update failed, retrying...", IS_LOG_LEVEL_ERROR);
                (obj)->m_use_progress = false;
                (obj)->reboot_force();
                return IS_OP_CLOSED;
            }
            return IS_OP_OK;
        }
        else if ((device & IS_IMAGE_SIGN_ISB) & fw_uINS_3)
        {
            (obj)->m_filename = filenames.fw_uINS_3.path;

            if((obj)->get_device_info() != IS_OP_OK)
            {
                delete obj;
                return IS_OP_CLOSED;
            }

            is_operation_result reboot_status = (obj)->reboot();
            if (reboot_status == IS_OP_OK)
            {
                delete obj;
                return IS_OP_CLOSED;
            }
            else if (reboot_status == IS_OP_CLOSED)
            {
                delete obj;
                return IS_OP_ERROR;
            }
            else
            {
                // usually IS_OP_ERROR, this indicates the serial number has already been reset. Continue to update
            }

            (obj)->m_use_progress = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            if((obj)->download_image(filenames.fw_uINS_3.path) != IS_OP_OK)
            {
                (obj)->m_info_callback((obj), "(ISB) Update failed, retrying...", IS_LOG_LEVEL_ERROR);
                (obj)->m_use_progress = false;
                (obj)->reboot_force();
                return IS_OP_CLOSED;
            }
            return IS_OP_OK;
        }
        else
        {
            delete obj;
        }
    }
    else
    {
        delete obj;
    }

    return IS_OP_ERROR;
}
