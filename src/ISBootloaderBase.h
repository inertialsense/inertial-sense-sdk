/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_BASE_H_
#define __IS_BOOTLOADER_BASE_H_

#include "ISConstants.h"
#include "ISSerialPort.h"
#include "libusb.h"
#include "ISUtilities.h"

#include <string>
#include <mutex>

namespace ISBootloader {

static constexpr int IS_DEVICE_LIST_LEN = 256;
static constexpr int IS_FIRMWARE_PATH_LENGTH = 256;

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_DEBUG = 4,
    IS_LOG_LEVEL_SILLY = 5
} eLogLevel;

typedef enum {
    // Inertial sense application modes
    IS_IMAGE_SIGN_IMX5p0 = 0x00000001,
    IS_IMAGE_SIGN_GPX1p0 = 0x00000002,
    IS_IMAGE_SIGN_CXD5610 = 0x00000003,

    IS_IMAGE_SIGN_NONE = 0,
    IS_IMAGE_SIGN_ERROR = 0x80000000,
} eImageSignature;

typedef enum {
    IS_DEVICE_MODE_GPX1p0_APP       = 0x00000001U,   // GPX-1 application mode
    IS_DEVICE_MODE_IMX5p0_APP       = 0x00000002U,   // IMX-5 application mode
    IS_DEVICE_MODE_STM32L4_DFU      = 0x00000004U,   // STM32L4 DFU (USB) bootloader
    IS_DEVICE_MODE_STM32U5_DFU      = 0x00000008U,   // STM32U5 DFU (USB) bootloader
    IS_DEVICE_MODE_STM32L4_USART    = 0x00000010U,   // STM32L4 USART bootloader
    IS_DEVICE_MODE_STM32U5_USART    = 0x00000020U,   // STM32U5 USART bootloader
    IS_DEVICE_MODE_SONY_CXD5610     = 0x00000040U,   // Sony CXD5610 GNSS LSI bootloader

    IS_DEVICE_MODE_NONE             = 0U,
    IS_DEVICE_MODE_ERROR            = 0x80000000U,
} eDeviceMode;

typedef is_operation_result (*pfnBootloadProgress)(void* obj, float percent);
typedef void (*pfnBootloadStatus)(void* obj, const char* infoString, eLogLevel level);

is_operation_result dummy_update_callback(void* obj, float percent);
is_operation_result dummy_verify_callback(void* obj, float percent);
static inline void dummy_info_callback(void* obj, const char* infoString, eLogLevel level)
{
    (void)obj;
    (void)infoString;
    (void)level;
}

class cISBootloaderBase
{
public:
    cISBootloaderBase(
        std::string filename,
        pfnBootloadProgress upload_cb,
        pfnBootloadProgress verify_cb,
        pfnBootloadStatus info_cb
    ) : 
        m_update_callback{upload_cb}, 
        m_verify_callback{verify_cb}, 
        m_info_callback{info_cb}
    {
        m_filename = filename;
        m_success = false;
        m_update_progress = 0.0;
        m_verify_progress = 0.0;
        m_use_progress = false;
        m_retries_left = 3;
        m_start_time_ms = 0;
        m_finished_flash = false;
        m_verify = false;

        if(m_update_callback == NULL) m_update_callback = dummy_update_callback;
        if(m_verify_callback == NULL) m_verify_callback = dummy_verify_callback;
        if(m_info_callback == NULL) m_info_callback = dummy_info_callback;
    }

    virtual ~cISBootloaderBase() {};

    static eImageSignature get_image_signature(std::string filename);

    virtual is_operation_result match_test(void* param) = 0;

    virtual uint8_t check_is_compatible(uint32_t imgSign) = 0;

    /**
     * @brief Reboots into the same mode, giving the bootloader a fresh start
     */
    virtual is_operation_result reboot() = 0;
    
    /**
     * @brief Reboots into the level above, if available.
     *  Make sure to call the destructor after a successful call to this function
     */
    virtual is_operation_result reboot_up() = 0;

    /**
     * @brief Reboots into the next level down, if available.
     *  Make sure to call the destructor after a successful call to this function
     */
    virtual is_operation_result reboot_down() { return IS_OP_OK; };

    /**
     * @brief Get the serial number from the device, and fill out m_ctx with other info
     */
    virtual uint32_t get_device_info() = 0;

    /**
     * @brief Write an image to the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result download_image(void) = 0;

    /**
     * @brief Read an image from the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result upload_image(void) = 0;
    
    /**
     * @brief Verify an image against the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result verify_image(void) = 0;

    virtual bool is_serial_device() { return true; }
    
    int m_retries_left;
    float m_update_progress;
    float m_verify_progress;
    bool m_verify;
    bool m_success;

    // Callbacks
    pfnBootloadProgress m_update_callback;
    pfnBootloadProgress m_verify_callback;
    pfnBootloadStatus m_info_callback; 

    void* m_thread;
    bool m_finished_flash;
    bool m_use_progress;
    int m_start_time_ms;

    serial_port_t* m_port;
    std::string m_port_name;
    int m_baud;

    uint32_t m_sn;                      // Inertial Sense serial number, i.e. SN60000

    static is_operation_result mode_device_app(
        std::vector<std::string> filenames,
        serial_port_t* handle,
        pfnBootloadStatus statusfn,
        pfnBootloadProgress updateProgress,
        pfnBootloadProgress verifyProgress,
        std::vector<cISBootloaderBase*>& contexts,
        std::mutex* addMutex,
        cISBootloaderBase** new_context
    );

    static is_operation_result update_device(
        std::vector<std::string> filenames,
        serial_port_t* handle,
        pfnBootloadStatus statusfn,
        pfnBootloadProgress updateprogress,
        pfnBootloadProgress verifyProgress,
        std::vector<cISBootloaderBase*>& contexts,
        std::mutex* addMutex,
        cISBootloaderBase** new_context,
        uint32_t baud = BAUDRATE_921600
    );

    static is_operation_result update_device(
        std::vector<std::string> filenames,
        libusb_device_handle* handle,
        pfnBootloadStatus statusfn,
        pfnBootloadProgress updateprogress,
        pfnBootloadProgress verifyProgress,
        std::vector<cISBootloaderBase*>& contexts,
        std::mutex* addMutex,
        cISBootloaderBase** new_context
    );

    std::string m_filename;
    uint32_t m_image_signature;
    bool m_isISB;

protected:
    void status_update(const char* info, eLogLevel level) 
    { 
        if(m_info_callback) m_info_callback((void*)this, info, level); 
    }

    struct
    {
        uint8_t uins_version[4];
        char enable_command[5];         // "BLEN" for IMX-5
    } m_app;

    /**
     * @brief Get the file extension from a file name
     */
    static const char* get_file_ext(const char* filename);
    static eImageSignature get_hex_image_signature(std::string filename);
    static eImageSignature get_folder_image_signature(std::string filename);
};

}

#endif
