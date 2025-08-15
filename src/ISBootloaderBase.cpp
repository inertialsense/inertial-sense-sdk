/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

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
#include "intel_hex_utils.h"

using namespace ISBootloader;

// const char* is_samx70_bootloader_needle = "bootloader-SAMx70";

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

eImageSignature cISBootloaderBase::get_hex_image_signature(std::string filename, uint8_t* major, char* minor)
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
            case IS_IMAGE_SIGN_IMX_5p0: target_signature = bootloaderRequiredSignature_IMX_5; break;
            case IS_IMAGE_SIGN_ISB_STM32L4: target_signature = bootloaderRequiredSignature_STM32L4_bootloader; break;
            case IS_IMAGE_SIGN_ISB_SAMx70_24K: target_signature = bootloaderRequiredSignature_SAMx70_bootloader_24K; break;
            default: continue;
            }

            size_t k = 0;
            for(size_t j = 0; j < image.len; j++)
            {
                if(image.image[j] == target_signature[k]) k++;  // Found the right char, continue
                else k = 0; // Didn't find the right char, reset to beginning of search

                if(k >= BOOTLOADER_SIGNATURE_SIZE) 
                {
                    // In the bootloader images, the version bytes are stored directly after the signature
                    // A third byte is added as a checksum of the two preceding bytes. 
                    if(image.image[j + 1] + image.image[j + 2] == image.image[j + 3])
                    {
                        if(major) *major = image.image[j + 1];
                        if(minor) *minor = (char)image.image[j + 2];
                    }
                    else
                    {
                        if(major) *major = 0;
                        if(minor) *minor = 0;           
                    }
                    return (eImageSignature)(1 << image_type);   // Found all the chars required
                }
            }
        }
        
    }
    
    // // Backup for old (16K) bootloader image
    // if (strstr(filename.c_str(), is_samx70_bootloader_needle))
    // {
    //     return IS_IMAGE_SIGN_ISB_SAMx70_16K;
    // }

    return IS_IMAGE_SIGN_NONE;
}

eImageSignature cISBootloaderBase::get_bin_image_signature(std::string filename, uint8_t* major, char* minor)
{
    FILE* blfile = 0;

#ifdef _MSC_VER
    fopen_s(&blfile, filename.c_str(), "rb");
#else
    blfile = fopen(filename.c_str(), "rb");
#endif

    if (blfile == 0)
        return IS_IMAGE_SIGN_NONE;

    fseek(blfile, 0x5FFC, SEEK_SET);
    unsigned char ver_info[4];
	size_t n = fread(ver_info, 1, 4, blfile);
    (void)n;

    //Check for marker for valid version info
    if (ver_info[0] == 0xAA && ver_info[1] == 0x55)
    {
        if(major) *major = ver_info[2];
        if(minor) *minor = ver_info[3];
    }
    else
    {
        if(major) *major = 0;
        if(minor) *minor = 0;

        // Look in the old location for this info (v5 and earler)
        fseek(blfile, 0x3DFC, SEEK_SET);
        size_t n = fread(ver_info, 1, 4, blfile);
        (void)n;

        //Check for marker for valid version info
        if (ver_info[0] == 0xAA && ver_info[1] == 0x55)
        {
            if(major) *major = ver_info[2];
            if(minor) *minor = ver_info[3];
        }
    }

    fclose(blfile);
    return (eImageSignature)(IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);
}

const char* cISBootloaderBase::get_file_ext(const char *filename) 
{
    const char *dot = strrchr(filename, '.');   // Find last '.' in file name
    if(!dot || dot == filename) return "";
    return dot + 1;
}

eImageSignature cISBootloaderBase::get_image_signature(std::string filename, uint8_t* major, char* minor)
{
    const char * extension = cISBootloaderBase::get_file_ext(filename.c_str());

    if(strcmp(extension, "bin") == 0)
    {
        return cISBootloaderBase::get_bin_image_signature(filename, major, minor);
    }
    else if(strcmp(extension, "hex") == 0)
    {
        return cISBootloaderBase::get_hex_image_signature(filename, major, minor);
    }
    
    return IS_IMAGE_SIGN_NONE;
}

is_operation_result cISBootloaderBase::mode_device_app
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
    (void)contexts;
    (void)addMutex;

    cISBootloaderBase* obj;
    *new_context = NULL;

    uint32_t device = IS_IMAGE_SIGN_NONE;
    uint32_t fw_uINS_3 = get_image_signature(filenames.fw_uINS_3.path) & (IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K);
    uint32_t fw_IMX_5  = get_image_signature(filenames.fw_IMX_5.path)  & (IS_IMAGE_SIGN_IMX_5p0);
    uint32_t fw_EVB_2  = get_image_signature(filenames.fw_EVB_2.path)  & (IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K);

    obj = new cISBootloaderAPP(updateProgress, verifyProgress, statusfn, handle);
    
    // Tell device to stop broadcasting
    serialPortWriteAscii(obj->m_port, "STPB", 4);

    (obj)->m_port_name = std::string(handle->port);
    device = (obj)->check_is_compatible();
    if(device)
    {   
        if ((device & IS_IMAGE_SIGN_APP) & fw_EVB_2)   
        {
            (obj)->m_filename = filenames.fw_EVB_2.path;
            strncpy((obj)->m_app.enable_command, "EBLE", 5);
            (obj)->reboot_down();
            delete obj;
            SLEEP_MS(3000);     // Delay 3 seconds to avoid port being re-used
            return IS_OP_CLOSED;
        }
        else if ((device & IS_IMAGE_SIGN_APP) & fw_IMX_5)
        {
            (obj)->m_filename = filenames.fw_IMX_5.path;
            strncpy((obj)->m_app.enable_command, "BLEN", 5);
            (obj)->reboot_down();
            delete obj;
            SLEEP_MS(3000);
            return IS_OP_CLOSED;
        }
        else if ((device & IS_IMAGE_SIGN_APP) & fw_uINS_3)
        {
            (obj)->m_filename = filenames.fw_uINS_3.path;
            strncpy((obj)->m_app.enable_command, "BLEN", 5);
            (obj)->reboot_down();
            delete obj;
            SLEEP_MS(3000);
            return IS_OP_CLOSED;
        }
    }

    delete obj;
    SLEEP_MS(3000);
    return IS_OP_CLOSED;    // Assume we found something besides app mode
}

is_operation_result cISBootloaderBase::get_device_isb_version(
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

    uint8_t major;
    char minor;

    uint32_t device = IS_IMAGE_SIGN_NONE;
    uint32_t fw_uINS_3 = get_image_signature(filenames.fw_uINS_3.path) & (IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K);
    uint32_t bl_uINS_3 = get_image_signature(filenames.bl_uINS_3.path, &major, &minor) & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);
    uint32_t fw_IMX_5  = get_image_signature(filenames.fw_IMX_5.path)  & (IS_IMAGE_SIGN_IMX_5p0);
    uint32_t bl_IMX_5  = get_image_signature(filenames.bl_IMX_5.path, &major, &minor) & IS_IMAGE_SIGN_ISB_STM32L4;
    uint32_t fw_EVB_2  = get_image_signature(filenames.fw_EVB_2.path)  & (IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K);
    uint32_t bl_EVB_2  = get_image_signature(filenames.bl_EVB_2.path, &major, &minor)  & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);

    obj = new cISBootloaderISB(updateProgress, verifyProgress, statusfn, handle);
    (obj)->m_port_name = std::string(handle->port);
    device = (obj)->check_is_compatible(); 
    if(device == IS_IMAGE_SIGN_ERROR)
    {
        delete obj;
    }
    else if(device)
    {   // Firmware for a device must be specified to update its bootloader
        if (((device & IS_IMAGE_SIGN_APP) & fw_EVB_2) && ((device & IS_IMAGE_SIGN_ISB) & bl_EVB_2))
        {
            (obj)->isb_mightUpdate = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            return IS_OP_CLOSED;
        }
        else if (((device & IS_IMAGE_SIGN_ISB) & bl_IMX_5) && ((device & IS_IMAGE_SIGN_APP) & fw_IMX_5))
        {
            (obj)->isb_mightUpdate = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            return IS_OP_CLOSED;
        }
        else if (((device & IS_IMAGE_SIGN_ISB) & bl_uINS_3) && ((device & IS_IMAGE_SIGN_APP) & fw_uINS_3))
        {
            (obj)->isb_mightUpdate = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            return IS_OP_CLOSED;
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

    return IS_OP_OK;
}

// Update bootloader firmware
is_operation_result cISBootloaderBase::mode_device_isb
(
    firmwares_t filenames,
    bool force,
    serial_port_t* handle,
    pfnBootloadStatus statusfn,
    pfnBootloadProgress updateProgress,
    pfnBootloadProgress verifyProgress,
    std::vector<cISBootloaderBase*>& contexts,
    std::mutex* addMutex,
    cISBootloaderBase** new_context
)
{
    (void)contexts;
    (void)addMutex;

    cISBootloaderBase* obj;
    *new_context = NULL;

    uint8_t major;
    char minor;

    uint32_t device = IS_IMAGE_SIGN_NONE;
    //uint32_t fw_uINS_3 = get_image_signature(filenames.fw_uINS_3.path) & (IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K);
    uint32_t bl_uINS_3 = get_image_signature(filenames.bl_uINS_3.path, &major, &minor) & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);
    //uint32_t fw_IMX_5  = get_image_signature(filenames.fw_IMX_5.path) & IS_IMAGE_SIGN_IMX_5p0;
    uint32_t bl_IMX_5  = get_image_signature(filenames.bl_IMX_5.path,  &major, &minor) & IS_IMAGE_SIGN_ISB_STM32L4;
    //uint32_t fw_EVB_2  = get_image_signature(filenames.fw_EVB_2.path)  & (IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K);
    uint32_t bl_EVB_2  = get_image_signature(filenames.bl_EVB_2.path,  &major, &minor) & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);

    obj = new cISBootloaderISB(updateProgress, verifyProgress, statusfn, handle);
    (obj)->m_port_name = std::string(handle->port);
    device = (obj)->check_is_compatible(); 
    if(device == IS_IMAGE_SIGN_ERROR)
    {
        delete obj;
    }
    else if(device)
    {   // Firmware for a device must be specified to update its bootloader
        if ((device & IS_IMAGE_SIGN_ISB) & bl_EVB_2) // & ((device & IS_IMAGE_SIGN_APP) & fw_EVB_2))
        {
            (obj)->m_filename = filenames.bl_EVB_2.path;
            is_operation_result op = (obj)->reboot_down(major, minor, force);
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
        else if ((device & IS_IMAGE_SIGN_ISB) & bl_IMX_5) // && ((device & IS_IMAGE_SIGN_APP) & fw_IMX_5))
        {
            (obj)->m_filename = filenames.bl_IMX_5.path;
            is_operation_result op = (obj)->reboot_down(major, minor, force);
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
        else if ((device & IS_IMAGE_SIGN_ISB) & bl_uINS_3) // && ((device & IS_IMAGE_SIGN_APP) & fw_uINS_3))
        {
            (obj)->m_filename = filenames.bl_uINS_3.path;
            is_operation_result op = (obj)->reboot_down(major, minor, force);
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
        else if (bl_uINS_3 | bl_IMX_5 | bl_EVB_2)
        {
            (obj)->m_info_callback(obj, IS_LOG_LEVEL_INFO, "(ISB) Bootloader upgrade not supported on this port. Trying APP update...");
            delete obj;
            return IS_OP_CLOSED;
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
    (obj)->get_device_info();
    device = (obj)->check_is_compatible();
    if ((device & IS_IMAGE_SIGN_DFU) & bl_IMX_5)
    {
        (obj)->m_filename = filenames.bl_IMX_5.path;
        (obj)->m_use_progress = true;
        addMutex->lock();
        contexts.push_back(obj);
        addMutex->unlock();

        // Retry update up to 3 times, return if cancel flag gets set.
        for(size_t i = 0; i < 3; i++)
        {
            is_operation_result result = (obj)->download_image(filenames.bl_IMX_5.path);
            if(result == IS_OP_CANCELLED)
            {
                return IS_OP_CLOSED;
            }
            else if(result != IS_OP_OK)
            {
                (obj)->m_info_callback((obj), IS_LOG_LEVEL_ERROR, "(DFU) Update failed, retrying...");
                (obj)->m_use_progress = false;
                (obj)->reboot();
                continue;
            }
            *new_context = obj;
            (obj)->reboot_up();    // Reboot up right away so an App update can happen
            return IS_OP_CLOSED;
        }

        (obj)->m_info_callback((obj), IS_LOG_LEVEL_ERROR, "(DFU) Update failed, too many retries");
        return IS_OP_CLOSED;
    }
    else
    {
        delete obj;
    }

    statusfn(NULL, IS_LOG_LEVEL_INFO, "    | (DFU) Firmware image incompatible with DFU device");
    return IS_OP_ERROR;
}

// Update application firmware
is_operation_result cISBootloaderBase::update_device
(
    firmwares_t filenames,
    serial_port_t* handle,
    pfnBootloadStatus statusfn,
    pfnBootloadProgress updateProgress,
    pfnBootloadProgress verifyProgress,
    std::vector<cISBootloaderBase*>& contexts,
    std::mutex* addMutex,
    cISBootloaderBase** new_context,
    uint32_t baud
)
{
    cISBootloaderBase* obj;
    *new_context = NULL;

    uint32_t device = IS_IMAGE_SIGN_NONE;
    uint32_t fw_uINS_3 = get_image_signature(filenames.fw_uINS_3.path) & (IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K);
    uint32_t bl_uINS_3 = get_image_signature(filenames.bl_uINS_3.path) & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);
    uint32_t fw_IMX_5  = get_image_signature(filenames.fw_IMX_5.path)  & (IS_IMAGE_SIGN_IMX_5p0);
    uint32_t fw_EVB_2  = get_image_signature(filenames.fw_EVB_2.path)  & (IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K);
    uint32_t bl_EVB_2  = get_image_signature(filenames.bl_EVB_2.path)  & (IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K);

    if(bl_EVB_2 || bl_uINS_3)
    {
        obj = new cISBootloaderSAMBA(updateProgress, verifyProgress, statusfn, handle);
        obj->m_port_name = std::string(handle->port);
        device = obj->check_is_compatible();
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
                statusfn(NULL, IS_LOG_LEVEL_ERROR, "    | (SAM-BA) Firmware image incompatible with SAM-BA device");
                delete obj;
                return IS_OP_CANCELLED;
            }
        }
        else
        {
            delete obj;
        }
    }

    char* name = handle->port;
    serialPortClose(handle);
    if (!serialPortOpenRetry(handle, name, baud, 1))
    {
        char msg[120] = { 0 };
        SNPRINTF(msg, sizeof(msg), "    | (%s) Unable to open port at %d baud", handle->port, baud);
        statusfn(NULL, IS_LOG_LEVEL_ERROR, msg);
        return IS_OP_ERROR;
    }

    obj = new cISBootloaderISB(updateProgress, verifyProgress, statusfn, handle);
    (obj)->m_port_name = std::string(handle->port);
    device = (obj)->check_is_compatible(); 
    if (device == IS_IMAGE_SIGN_NONE)
    {
        delete obj;
        char msg[120] = { 0 };
        SNPRINTF(msg, sizeof(msg), "    | (%s) Device response missing.", handle->port);
        statusfn(NULL, IS_LOG_LEVEL_ERROR, msg);
        return IS_OP_ERROR;
    }
    else if(device == IS_IMAGE_SIGN_ERROR)
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

            (obj)->m_use_progress = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            is_operation_result result = (obj)->download_image(filenames.fw_EVB_2.path);
            if(result == IS_OP_CANCELLED)
            {
                return IS_OP_CLOSED;
            }
            else if(result != IS_OP_OK)
            {
                (obj)->m_info_callback((obj), IS_LOG_LEVEL_ERROR, "(ISB) Update failed, retrying...");
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

            //////////////////////////////////////////////////////////////////////
            // Check IMX-5 bootloader / application firmware compatibility
            if (((obj)->m_isb_major < 6) || 
                ((obj)->m_isb_major == 6 && (obj)->m_isb_minor < 'i'))
            {
                // Check that firmware size will fit with current bootloader
                size_t pages = calculateFlashPagesUsed(filenames.fw_IMX_5.path, IMX5_FLASH_PAGE_SIZE);
                if (pages >= 8)
                {   // IMX-5 application requires bootloader v6i or newer to write into 8th page of flash memory
                    (obj)->m_info_callback(NULL, IS_LOG_LEVEL_ERROR, "    | (ISB) UPDATE ABORTED! " IMX5_BOOTLOADER_INCOMPATIBLE_MSG);
                    delete obj;
                    return IS_OP_INCOMPATIBLE;
                }
            }
            //////////////////////////////////////////////////////////////////////

            (obj)->m_use_progress = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            is_operation_result result = (obj)->download_image(filenames.fw_IMX_5.path);
            if(result == IS_OP_CANCELLED)
            {
                return IS_OP_CLOSED;
            }
            else if(result != IS_OP_OK)
            {
                (obj)->m_info_callback((obj), IS_LOG_LEVEL_ERROR, "(ISB) Update failed, retrying...");
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

            (obj)->m_use_progress = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            is_operation_result result = (obj)->download_image(filenames.fw_uINS_3.path);
            if(result == IS_OP_CANCELLED)
            {
                return IS_OP_CLOSED;
            }
            else if(result != IS_OP_OK)
            {
                (obj)->m_info_callback((obj), IS_LOG_LEVEL_ERROR, "(ISB) Update failed, retrying...");
                (obj)->m_use_progress = false;
                (obj)->reboot_force();
                return IS_OP_CLOSED;
            }
            return IS_OP_OK;
        }
        else
        {
            statusfn(NULL, IS_LOG_LEVEL_ERROR, "    | (ISB) Firmware image incompatible with ISB device");
            delete obj;
            return IS_OP_CANCELLED;
        }
    }
    else
    {
        delete obj;
    }

    char msg[120] = {0};
    SNPRINTF(msg, sizeof(msg), "    | (%s) Incompatible device selected", handle->port);
    statusfn(NULL, IS_LOG_LEVEL_ERROR, msg);
    return IS_OP_ERROR;
}
