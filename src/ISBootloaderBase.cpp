/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderBase.h"
#include "ISBootloaderDFU.h"
#include "ISBootloaderAPP.h"
#include "ISBootloaderSTM32.h"

#include "../hw-libs/firmwareSignatures.h"
#include "ISUtilities.h"
#include "ISConstants.h"
#include "ihex.h"
#include "libusb.h"
#include <sys/stat.h>

#if PLATFORM_IS_WINDOWS
#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#endif

using namespace ISBootloader;

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
        const uint8_t *target_signature;

        // 31 because 0x80000000 is an error code
        for(image_type = 0; image_type < 31; image_type++)
        {
            switch(1 << image_type)
            {
            case IS_IMAGE_SIGN_IMX5p0: target_signature = bootloaderRequiredSignature_IMX5p0; break;
            case IS_IMAGE_SIGN_GPX1p0: target_signature = bootloaderRequiredSignature_GPX1p0; break;
            default: continue;
            }

            size_t k = 0;
            for(size_t j = 0; j < image.len; j++)
            {
                if(image.image[j] == target_signature[k]) k++;  // Found the right char, continue
                else k = 0; // Didn't find the right char, reset to beginning of search

                if(k >= BOOTLOADER_SIGNATURE_SIZE) 
                {
                    return (eImageSignature)(1 << image_type);   // Found all the chars required
                }
            }
        }
        
    }

    return IS_IMAGE_SIGN_NONE;
}

const char* cISBootloaderBase::get_file_ext(const char *filename) 
{
    const char *dot = strrchr(filename, '.');   // Find last '.' in file name
    if(!dot || dot == filename) return nullptr;
    return dot + 1;
}

eImageSignature cISBootloaderBase::get_image_signature(std::string filename)
{
    const char* extension = get_file_ext(filename.c_str());

    if(extension && strncmp(extension, "hex", 5) == 0)
    {
        return cISBootloaderBase::get_hex_image_signature(filename);
    }
    

    return IS_IMAGE_SIGN_NONE;
}

is_operation_result cISBootloaderBase::mode_device_app
(
    std::vector<std::string> filenames,
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

    *new_context = NULL;

    uint32_t imageSignature = get_image_signature(filename) & (IS_IMAGE_SIGN_IMX5p0 | IS_IMAGE_SIGN_GPX1p0);

    if (imageSignature)
    {
        cISBootloaderBase* obj = new cISBootloaderAPP(filename, "BLEN", updateProgress, verifyProgress, statusfn, handle);

        if(obj->check_is_compatible(imageSignature))
        {
            obj->reboot_down();
        }
        
        delete obj;
    }

    SLEEP_MS(3000);
    return IS_OP_CLOSED;    // Assume we found something besides app mode
}

is_operation_result cISBootloaderBase::update_device
(
    std::vector<std::string> filenames,
    libusb_device_handle* handle,       // LIBUSB
    pfnBootloadStatus statusfn,
    pfnBootloadProgress updateProgress,
    pfnBootloadProgress verifyProgress,
    std::vector<cISBootloaderBase*>& contexts,
    std::mutex* addMutex,
    cISBootloaderBase** new_context
)
{
    *new_context = NULL;

    uint32_t imageSignature = get_image_signature(filename) & (IS_IMAGE_SIGN_IMX5p0 | IS_IMAGE_SIGN_GPX1p0);

    if (imageSignature)
    {
        cISBootloaderBase* obj = new cISBootloaderDFU(filename, updateProgress, verifyProgress, statusfn, handle);

        if(obj->check_is_compatible(imageSignature))
        {
            // Get device info
            obj->get_device_info();
            
            // Add object to list of contexts
            addMutex->lock();
            contexts.push_back(obj);
            addMutex->unlock();

            is_operation_result result = obj->download_image();

            if(result == IS_OP_CANCELLED)
            {
                return IS_OP_CLOSED;
            }
            else if(result != IS_OP_OK)
            {
                obj->m_info_callback(obj, "(DFU) Update failed, retrying...", IS_LOG_LEVEL_ERROR);
                obj->m_use_progress = false;
                obj->reboot();
                return IS_OP_RETRY;
            }

            *new_context = obj;
            return IS_OP_OK;
        }
        else
        {
            delete obj;
            return IS_OP_CLOSED;
        }
    }
    
    statusfn(NULL, "    | (DFU) Firmware image incompatible with DFU device", IS_LOG_LEVEL_ERROR);
    return IS_OP_ERROR;
}

is_operation_result cISBootloaderBase::update_device
(
    std::vector<std::string> filenames,
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

    uint32_t imageSignatureApp = get_image_signature(filename) & (IS_IMAGE_SIGN_IMX5p0 | IS_IMAGE_SIGN_GPX1p0);
    uint32_t imageSignatureSony = get_image_signature(filename) & (IS_IMAGE_SIGN_CXD5610);

    char* name = handle->port;
    serialPortClose(handle);
    if (!serialPortOpenRetry(handle, name, baud, 1))
    {
        char msg[120] = { 0 };
        SNPRINTF(msg, sizeof(msg), "    | (%s) Unable to open port at %d baud", handle->port, baud);
        statusfn(NULL, msg, IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }

    obj = new cISBootloaderSTM32(updateProgress, verifyProgress, statusfn, handle);
    obj->m_port_name = std::string(handle->port);
    device = obj->check_is_compatible(); 
    if (device == IS_IMAGE_SIGN_NONE)
    {
        delete obj;
        char msg[120] = { 0 };
        SNPRINTF(msg, sizeof(msg), "    | (%s) Device response missing.", handle->port);
        statusfn(NULL, msg, IS_LOG_LEVEL_ERROR);
        return IS_OP_ERROR;
    }
    else if(device == IS_IMAGE_SIGN_ERROR)
    {
        delete obj;
    }
    else if(device)
    {
        if ((device & IS_IMAGE_SIGN_ISB) & fw_IMX_5)
        {
            obj->m_filename = filenames.fw_IMX_5.path;

            if(obj->get_device_info() != IS_OP_OK)
            {
                delete obj;
                return IS_OP_CLOSED;
            }

            obj->m_use_progress = true;
            addMutex->lock();
            contexts.push_back(obj);
            *new_context = obj;
            addMutex->unlock();
            is_operation_result result = obj->download_image();
            if(result == IS_OP_CANCELLED)
            {
                return IS_OP_CLOSED;
            }
            else if(result != IS_OP_OK)
            {
                obj->m_info_callback(obj, "(STM) Update failed, retrying...", IS_LOG_LEVEL_ERROR);
                obj->m_use_progress = false;
                obj->reboot();
                return IS_OP_CLOSED;
            }
            return IS_OP_OK;
        }
        else
        {
            statusfn(NULL, "    | (STM) Firmware image incompatible with device", IS_LOG_LEVEL_ERROR);
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
    statusfn(NULL, msg, IS_LOG_LEVEL_ERROR);
    return IS_OP_ERROR;
}
