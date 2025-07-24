/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_BASE_H_
#define __IS_BOOTLOADER_BASE_H_

#include <cstdarg>
#include <cstdio>
#include <string>
#include <mutex>

#include "core/base_port.h"
#include "ISConstants.h"
#include "ISSerialPort.h"
#include "libusb.h"
#include "ISUtilities.h"
#include "protocol/FirmwareUpdate.h"

namespace ISBootloader {

static constexpr int IS_DEVICE_LIST_LEN = 256;
static constexpr int IS_FIRMWARE_PATH_LENGTH = 256;

static constexpr int IS_REBOOT_DELAY_MS = 3000;

typedef enum {
    IS_PROCESSOR_UNKNOWN = -1,
    IS_PROCESSOR_SAMx70 = 0,        // uINS-3/4, EVB-2
    IS_PROCESSOR_STM32L4,           // IMX-5
    IS_PROCESSOR_STM32U5,           // GPX-1, IMX-6

    IS_PROCESSOR_NUM,               // Must be last
} eProcessorType;

typedef enum {
    // Bootloaders must be first because bootloaders may contain app signatures
    IS_IMAGE_SIGN_ISB_STM32L4 = 0x00000001,
    IS_IMAGE_SIGN_ISB_SAMx70_16K = 0x00000002,
    IS_IMAGE_SIGN_ISB_SAMx70_24K = 0x00000004,

    IS_IMAGE_SIGN_UINS_3_16K = 0x00000008,
    IS_IMAGE_SIGN_UINS_3_24K = 0x00000010,
    IS_IMAGE_SIGN_EVB_2_16K = 0x00000020,
    IS_IMAGE_SIGN_EVB_2_24K = 0x00000040,
    IS_IMAGE_SIGN_IMX_5p0 = 0x00000080,
    
    IS_IMAGE_SIGN_NUM_BITS_USED = 8,

    IS_IMAGE_SIGN_APP = IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K | IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_IMX_5p0 | IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4,
    IS_IMAGE_SIGN_ISB = IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K | IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_IMX_5p0 | IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4,
    
    IS_IMAGE_SIGN_SAMBA = IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K,
    IS_IMAGE_SIGN_DFU = IS_IMAGE_SIGN_ISB_STM32L4,

    IS_IMAGE_SIGN_EVB = IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K,

    IS_IMAGE_SIGN_NONE = 0,
    IS_IMAGE_SIGN_ERROR = 0x80000000,
} eImageSignature;

typedef struct 
{
    std::string path;
} firmware_t;

typedef struct 
{
    firmware_t fw_IMX_5;
    firmware_t bl_IMX_5;
    firmware_t fw_uINS_3;
    firmware_t bl_uINS_3;
    firmware_t fw_EVB_2;
    firmware_t bl_EVB_2;
} firmwares_t;

// typedef is_operation_result (*fwUpdate::pfnProgressCb)(void* obj, float percent);
// typedef void (*fwUpdate::pfnStatusCb)(void* obj, int level, const char* infoString, ...);

is_operation_result dummy_update_callback(const std::any& obj, float percent, const std::string& stepName, int stepNo, int totalSteps);
is_operation_result dummy_verify_callback(const std::any& obj, float percent, const std::string& stepName, int stepNo, int totalSteps);
static inline void dummy_info_callback(const std::any& obj, eLogLevel level, const char* infoString, ...)
{
    (void)obj;
    (void)infoString;
    (void)level;
}

class cISBootloaderBase
{
public:
    cISBootloaderBase(
        fwUpdate::pfnProgressCb upload_cb,
        fwUpdate::pfnProgressCb verify_cb,
        fwUpdate::pfnStatusCb info_cb
  ) :
        m_update_callback{upload_cb}, 
        m_verify_callback{verify_cb}, 
        m_info_callback{info_cb}
    {
        m_success = false;
        m_update_progress = 0.0;
        m_verify_progress = 0.0;
        m_use_progress = false;
        m_retries_left = 3;
        m_start_time_ms = 0;
        m_finished_flash = false;
        m_verify = false;

        if (m_update_callback == NULL)  m_update_callback = dummy_update_callback;
        if (m_verify_callback == NULL)  m_verify_callback = dummy_verify_callback;
        if (m_info_callback == NULL)    m_info_callback = dummy_info_callback;
    }

    virtual ~cISBootloaderBase() {};

    static eImageSignature get_image_signature(std::string filename, uint8_t* major = NULL, char* minor = NULL);

    virtual is_operation_result match_test(void* param) = 0;

    virtual eImageSignature check_is_compatible() = 0;

    /**
     * @brief Reboots into the same mode, giving the bootloader a fresh start
     */
    virtual is_operation_result reboot() = 0;
    virtual is_operation_result reboot_force() { return IS_OP_OK; }
    
    /**
     * @brief Reboots into the level above, if available:
     *  - ISB to App
     *  - SAM-BA to ISB
     *  - DFU to ISB
     *  Make sure to tall the destructor after a successful call to this function
     */
    virtual is_operation_result reboot_up() = 0;

    /**
     * @brief Reboots into the level below, if available:
     *  - App to ISB
     *  - ISB to DFU
     *  - ISB to SAM-BA
     *  Make sure to call the destructor after a successful call to this function
     */
    virtual is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false) = 0;

    /**
     * @brief Get the serial number from the device, and fill out m_ctx with other info
     */
    virtual uint32_t get_device_info() = 0;

    /**
     * @brief Write an image to the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result download_image(std::string image) = 0;

    /**
     * @brief Read an image from the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result upload_image(std::string image) = 0;
    
    /**
     * @brief Verify an image against the device
     * 
     * @param image path to the image
     */
    virtual is_operation_result verify_image(std::string image) = 0;

    virtual bool is_serial_device() { return true; }

    template<typename... Args> void logStatus(eLogLevel level, const char* infoString, Args... args) {
        if (m_info_callback)
            m_info_callback(this, level, infoString, args...);
    }


    int m_retries_left = 3;
    float m_update_progress = 0;
    float m_verify_progress = 0;
    bool m_verify = false;
    bool m_success = false;

    // Callbacks
    fwUpdate::pfnProgressCb m_update_callback = nullptr;
    fwUpdate::pfnProgressCb m_verify_callback = nullptr;
    fwUpdate::pfnStatusCb m_info_callback = nullptr;

    void* m_thread = nullptr;
    bool m_finished_flash = false;
    int m_bootloader_type = false;
    bool m_use_progress = false;
    int m_start_time_ms = 0;

    port_handle_t m_port = nullptr;
    std::string m_port_name;
    int m_baud = 0;

    uint32_t m_sn = 0;                      // Inertial Sense serial number, i.e. SN60000
    uint16_t m_hdw = 0;                     // Inertial Sense Hardware Type (IMX, GPX, etc)
    uint8_t m_isb_major = 0;                // ISB Major revision on device
    char m_isb_minor = 0;                   // ISB Minor revision on device
    bool isb_mightUpdate = 0;               // true if device will be updated if bootloader continues

    static is_operation_result mode_device_app(
        firmwares_t filenames,
        port_handle_t port,
        fwUpdate::pfnStatusCb statusfn,
        fwUpdate::pfnProgressCb updateProgress,
        fwUpdate::pfnProgressCb verifyProgress,
        std::vector<cISBootloaderBase*>& contexts,
        std::mutex* addMutex,
        cISBootloaderBase** new_context
);

    static is_operation_result get_device_isb_version(
        firmwares_t filenames,
        port_handle_t port,

        fwUpdate::pfnStatusCb statusfn,
        fwUpdate::pfnProgressCb updateProgress,
        fwUpdate::pfnProgressCb verifyProgress,
        std::vector<cISBootloaderBase*>& contexts,
        std::mutex* addMutex,
        cISBootloaderBase** new_context
);

    static is_operation_result mode_device_isb(
        firmwares_t filenames,
        bool force,
        port_handle_t port,
        fwUpdate::pfnStatusCb statusfn,
        fwUpdate::pfnProgressCb updateProgress,
        fwUpdate::pfnProgressCb verifyProgress,
        std::vector<cISBootloaderBase*>& contexts,
        std::mutex* addMutex,
        cISBootloaderBase** new_context
);

    static is_operation_result update_device(
        firmwares_t filenames,
        port_handle_t port,
        fwUpdate::pfnStatusCb statusfn,
        fwUpdate::pfnProgressCb updateprogress,
        fwUpdate::pfnProgressCb verifyProgress,
        std::vector<cISBootloaderBase*>& contexts,
        std::mutex* addMutex,
        cISBootloaderBase** new_context,
        uint32_t baud = BAUDRATE_921600
);
    static is_operation_result update_device(
        firmwares_t filenames,
        libusb_device_handle* handle,
        fwUpdate::pfnStatusCb statusfn,
        fwUpdate::pfnProgressCb updateprogress,
        fwUpdate::pfnProgressCb verifyProgress,
        std::vector<cISBootloaderBase*>& contexts,
        std::mutex* addMutex,
        cISBootloaderBase** new_context
);

    std::string m_filename;
    bool m_isISB = false;

protected:
    void status_update(const char* info, eLogLevel level)
    { 
        if (m_info_callback) m_info_callback(std::any_cast<cISBootloaderBase*>(this), level, info);
    }

    struct
    {
        uint8_t uins_version[4];
        uint8_t evb_version[4];

        char enable_command[5];         // "EBLE" (EVB) or "BLEN" (uINS) 
    } m_app;

    /**
     * @brief Get the file extension from a file name
     */
    static const char* get_file_ext(const char* filename);
    
    static eImageSignature get_hex_image_signature(std::string image, uint8_t* major = NULL, char* minor = NULL);
    static eImageSignature get_bin_image_signature(std::string image, uint8_t* major = NULL, char* minor = NULL);
};

}

#endif
