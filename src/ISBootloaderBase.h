/**
 * @file ISBootloaderBase.h
 * @brief Common contract for the per-target bootloader protocol implementations
 *        (cISBootloaderBase and its ISBootloaderAPP/DFU/ISB/SAMBA/STM32/Sony derivatives):
 *        image-signature detection, reboot-chain navigation between bootloader levels, and the
 *        download/upload/verify operations each concrete transport must implement.
 *
 * @author Dave Cutting
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. All rights reserved. See the MIT
 *            license text below.
 */

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

#define IMX5_FLASH_PAGE_SIZE 65536      //!< 64K flash page size for IMX-5
#define IMX5_BOOTLOADER_INCOMPATIBLE_MSG "IMX firmware incompatible with bootloader. Update IMX-5 bootloader to v6i or newer required for selected IMX firmware."  //!< error text shown when the IMX-5 bootloader is too old for the selected firmware image

namespace ISBootloader {

static constexpr int IS_DEVICE_LIST_LEN = 256;        //!< maximum number of devices that can be tracked/enumerated at once
static constexpr int IS_FIRMWARE_PATH_LENGTH = 256;   //!< maximum length, in bytes, of a firmware/bootloader image file path

static constexpr int IS_REBOOT_DELAY_MS = 3000;       //!< delay (ms) allowed for a device to complete a reboot before it is considered unresponsive

/** Processor family of the connected device, detected from its bootloader/image signature. */
typedef enum {
    IS_PROCESSOR_UNKNOWN = -1,      //!< not yet identified
    IS_PROCESSOR_SAMx70 = 0,        //!< uINS-3/4, EVB-2
    IS_PROCESSOR_STM32L4,           //!< IMX-5
    IS_PROCESSOR_STM32U5,           //!< GPX-1, IMX-6

    IS_PROCESSOR_NUM,               //!< number of known processor types; must be last
} eProcessorType;

/**
 * Bitmask identifying which target(s) a firmware/bootloader image is valid for, detected from the
 * image file itself (see cISBootloaderBase::get_image_signature()). Bootloader signature bits are
 * assigned first because a bootloader image may also satisfy an application image's signature check.
 */
typedef enum {
    IS_IMAGE_SIGN_ISB_STM32L4 = 0x00000001,     //!< ISB bootloader image, STM32L4 (IMX-5)
    IS_IMAGE_SIGN_ISB_SAMx70_16K = 0x00000002,  //!< ISB bootloader image, SAMx70 16K (uINS-3/4, EVB-2)
    IS_IMAGE_SIGN_ISB_SAMx70_24K = 0x00000004,  //!< ISB bootloader image, SAMx70 24K (uINS-3/4, EVB-2)

    IS_IMAGE_SIGN_UINS_3_16K = 0x00000008,      //!< application image, uINS-3, 16K bootloader
    IS_IMAGE_SIGN_UINS_3_24K = 0x00000010,       //!< application image, uINS-3, 24K bootloader
    IS_IMAGE_SIGN_EVB_2_16K = 0x00000020,        //!< application image, EVB-2, 16K bootloader
    IS_IMAGE_SIGN_EVB_2_24K = 0x00000040,        //!< application image, EVB-2, 24K bootloader
    IS_IMAGE_SIGN_IMX_5p0 = 0x00000080,          //!< application image, IMX-5.0

    IS_IMAGE_SIGN_NUM_BITS_USED = 8,             //!< number of signature bits actually assigned above

    IS_IMAGE_SIGN_APP = IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K | IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_IMX_5p0 | IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4,  //!< union of all application-image signature bits
    IS_IMAGE_SIGN_ISB = IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K | IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_IMX_5p0 | IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4,  //!< union of signature bits valid for an ISB-protocol update

    IS_IMAGE_SIGN_SAMBA = IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K,  //!< union of signature bits valid for a SAM-BA update
    IS_IMAGE_SIGN_DFU = IS_IMAGE_SIGN_ISB_STM32L4,                                       //!< union of signature bits valid for a USB DFU update

    IS_IMAGE_SIGN_EVB = IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K,  //!< union of signature bits identifying an EVB-2 image

    IS_IMAGE_SIGN_NONE = 0,             //!< no recognized signature
    IS_IMAGE_SIGN_ERROR = 0x80000000,   //!< the image could not be read/parsed to determine a signature
} eImageSignature;

/** A single firmware or bootloader image's file path, used by the legacy multi-target update entry points. */
typedef struct
{
    std::string path;   //!< path to the image file (empty if not provided/applicable)
} firmware_t;

/** The set of firmware/bootloader image paths for every legacy-supported target, used by mode_device_app()/update_device(). */
typedef struct
{
    firmware_t fw_IMX_5;    //!< IMX-5 application image
    firmware_t bl_IMX_5;    //!< IMX-5 bootloader image
    firmware_t fw_uINS_3;   //!< uINS-3 application image
    firmware_t bl_uINS_3;   //!< uINS-3 bootloader image
    firmware_t fw_EVB_2;    //!< EVB-2 application image
    firmware_t bl_EVB_2;    //!< EVB-2 bootloader image
} firmwares_t;

// typedef is_operation_result (*fwUpdate::pfnProgressCb)(void* obj, float percent);
// typedef void (*fwUpdate::pfnStatusCb)(void* obj, int level, const char* infoString, ...);

/**
 * Default upload-progress callback used when the caller does not supply one; discards the report.
 * @param obj the calling cISBootloaderBase instance (as std::any)
 * @param percent upload completion percentage (0-100)
 * @param stepName human-readable name of the current step
 * @param stepNo the index of the current step
 * @param totalSteps the total number of steps
 * @return IS_OP_OK always
 */
is_operation_result dummy_update_callback(const std::any& obj, float percent, const std::string& stepName, int stepNo, int totalSteps);

/**
 * Default verify-progress callback used when the caller does not supply one; discards the report.
 * @param obj the calling cISBootloaderBase instance (as std::any)
 * @param percent verify completion percentage (0-100)
 * @param stepName human-readable name of the current step
 * @param stepNo the index of the current step
 * @param totalSteps the total number of steps
 * @return IS_OP_OK always
 */
is_operation_result dummy_verify_callback(const std::any& obj, float percent, const std::string& stepName, int stepNo, int totalSteps);

/**
 * Default status/info callback used when the caller does not supply one; discards the message.
 * @param obj the calling cISBootloaderBase instance (as std::any)
 * @param level the severity of the message (one of eLogLevel)
 * @param infoString a printf-style format string
 * @param ... format arguments for infoString
 */
static inline void dummy_info_callback(const std::any& obj, eLogLevel level, const char* infoString, ...)
{
    (void)obj;
    (void)infoString;
    (void)level;
}

/**
 * Abstract base for a single bootloader-protocol session with one connected device. A concrete
 * subclass implements the wire transport for one bootloader level (application/ISB/DFU/SAM-BA/
 * STM32-ROM/Sony) -- image compatibility checking, download/upload/verify, and navigating the
 * reboot chain to the adjacent bootloader level above or below the one this instance represents.
 * Progress and status are reported via the upload/verify/info callbacks passed to the constructor.
 */
class cISBootloaderBase
{
public:
    /**
     * @param upload_cb callback invoked to report image-download progress; dummy_update_callback if null
     * @param verify_cb callback invoked to report image-verify progress; dummy_verify_callback if null
     * @param info_cb callback invoked to report status/log messages; dummy_info_callback if null
     */
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

    /**
     * Determines which target(s) the given image file is valid for, by inspecting its contents
     * (not just its extension/filename).
     * @param filename path to the firmware/bootloader image file
     * @param major if non-null, receives the image's major version, when encoded in the image
     * @param minor if non-null, receives the image's minor version, when encoded in the image
     * @return an eImageSignature bitmask of the target(s) the image is valid for, or IS_IMAGE_SIGN_ERROR/IS_IMAGE_SIGN_NONE
     */
    static eImageSignature get_image_signature(std::string filename, uint8_t* major = NULL, char* minor = NULL);

    /**
     * Tests whether param identifies a device this bootloader-protocol implementation can drive
     * (e.g. a matching port or libusb device, depending on the concrete subclass).
     * @param param an implementation-specific device/port identifier to test
     * @return IS_OP_OK if param matches this implementation's transport, otherwise an error
     */
    virtual is_operation_result match_test(void* param) = 0;

    /**
     * Queries the connected device to determine which image signature(s) it can currently accept.
     * @return the eImageSignature bitmask of images this device's bootloader will accept, or IS_IMAGE_SIGN_ERROR
     */
    virtual eImageSignature check_is_compatible() = 0;

    /**
     * @brief Reboots into the same mode, giving the bootloader a fresh start
     * @return IS_OP_OK on success, otherwise an error
     */
    virtual is_operation_result reboot() = 0;

    /** @return IS_OP_OK always (default no-op; subclasses may override to force a reboot even from an unresponsive state) */
    virtual is_operation_result reboot_force() { return IS_OP_OK; }

    /**
     * @brief Reboots into the level above, if available:
     *  - ISB to App
     *  - SAM-BA to ISB
     *  - DFU to ISB
     *  Make sure to tall the destructor after a successful call to this function
     * @return IS_OP_OK on success, otherwise an error
     */
    virtual is_operation_result reboot_up() = 0;

    /**
     * @brief Reboots into the level below, if available:
     *  - App to ISB
     *  - ISB to DFU
     *  - ISB to SAM-BA
     *  Make sure to call the destructor after a successful call to this function
     * @param major if the target level requires a compatible image, its major version (0 = don't care)
     * @param minor if the target level requires a compatible image, its minor version (0 = don't care)
     * @param force if true, reboot even if the target level's compatibility can't be confirmed
     * @return IS_OP_OK on success, otherwise an error
     */
    virtual is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false) = 0;

    /**
     * @brief Get the serial number from the device, and fill out m_ctx with other info
     * @return the device's Inertial Sense serial number, or 0 if it could not be read
     */
    virtual uint32_t get_device_info() = 0;

    /**
     * @brief Write an image to the device
     *
     * @param image path to the image
     * @return IS_OP_OK on success, otherwise an error
     */
    virtual is_operation_result download_image(std::string image) = 0;

    /**
     * @brief Read an image from the device
     *
     * @param image path to write the read-back image to
     * @return IS_OP_OK on success, otherwise an error
     */
    virtual is_operation_result upload_image(std::string image) = 0;

    /**
     * @brief Verify an image against the device
     *
     * @param image path to the image to verify against the device's current contents
     * @return IS_OP_OK if the device's contents match image, otherwise an error
     */
    virtual is_operation_result verify_image(std::string image) = 0;

    /** @return true if this bootloader-protocol implementation communicates over a serial port, false for USB/other transports */
    virtual bool is_serial_device() { return true; }

    /**
     * Formats and forwards a status/log message to the info callback supplied at construction.
     * @param level the severity of the message (one of eLogLevel)
     * @param infoString a printf-style format string
     * @param args format arguments for infoString
     */
    template<typename... Args> void logStatus(eLogLevel level, const char* infoString, Args... args) {
        if (m_info_callback)
            m_info_callback(this, level, infoString, args...);
    }


    int m_retries_left = 3;                 //!< number of remaining retry attempts for the current operation
    float m_update_progress = 0;            //!< current download (upload-to-device) progress, 0-100
    float m_verify_progress = 0;            //!< current verify progress, 0-100
    bool m_verify = false;                  //!< true if a verify pass should be performed after download
    bool m_success = false;                 //!< true if the most recent operation completed successfully

    // Callbacks
    fwUpdate::pfnProgressCb m_update_callback = nullptr;  //!< invoked to report download progress (never null after construction)
    fwUpdate::pfnProgressCb m_verify_callback = nullptr;  //!< invoked to report verify progress (never null after construction)
    fwUpdate::pfnStatusCb m_info_callback = nullptr;      //!< invoked to report status/log messages (never null after construction)

    void* m_thread = nullptr;               //!< opaque handle to a worker thread driving this session, if any (owned/typed by the concrete subclass)
    bool m_finished_flash = false;          //!< true once the download/verify sequence has completed (success or failure)
    int m_bootloader_type = false;          //!< implementation-specific bootloader type/variant identifier
    bool m_use_progress = false;            //!< true if this session should report progress via the callbacks
    int m_start_time_ms = 0;                //!< timestamp (ms) when the current operation started, for timeout/duration tracking

    port_handle_t m_port = nullptr;         //!< the serial port this session communicates over, if is_serial_device()
    std::string m_port_name;                //!< the name of m_port (e.g. "/dev/ttyACM0"), for diagnostics
    int m_baud = 0;                         //!< serial baud rate in use, if is_serial_device()

    uint32_t m_sn = 0;                      //!< Inertial Sense serial number, i.e. SN60000
    uint16_t m_hdw = 0;                     //!< Inertial Sense Hardware Type (IMX, GPX, etc)
    uint8_t m_isb_major = 0;                //!< ISB Major revision on device
    char m_isb_minor = 0;                   //!< ISB Minor revision on device
    bool isb_mightUpdate = 0;               //!< true if device will be updated if bootloader continues

    /**
     * Legacy multi-target entry point: given a device already running its application, determine
     * its bootloader mode/version and add a matching cISBootloaderBase context for it.
     * @param filenames candidate firmware/bootloader image paths for every supported target
     * @param port the serial port the device is connected on
     * @param statusfn callback invoked to report status/log messages
     * @param updateProgress callback invoked to report download progress
     * @param verifyProgress callback invoked to report verify progress
     * @param contexts the shared list of active bootloader contexts to append the new context to
     * @param addMutex mutex guarding concurrent access to contexts
     * @param new_context if non-null, receives the newly-created context
     * @return IS_OP_OK on success, otherwise an error
     */
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

    /**
     * Legacy multi-target entry point: queries a device already in ISB mode for its bootloader
     * version and adds a matching cISBootloaderBase context for it.
     * @param filenames candidate firmware/bootloader image paths for every supported target
     * @param port the serial port the device is connected on
     * @param statusfn callback invoked to report status/log messages
     * @param updateProgress callback invoked to report download progress
     * @param verifyProgress callback invoked to report verify progress
     * @param contexts the shared list of active bootloader contexts to append the new context to
     * @param addMutex mutex guarding concurrent access to contexts
     * @param new_context if non-null, receives the newly-created context
     * @return IS_OP_OK on success, otherwise an error
     */
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

    /**
     * Legacy multi-target entry point: given a device already in ISB mode, add a matching
     * cISBootloaderBase context for it and (if force or a version mismatch requires it) drive an
     * ISB bootloader update using filenames.
     * @param filenames candidate firmware/bootloader image paths for every supported target
     * @param force if true, update the bootloader even if its version already appears compatible
     * @param port the serial port the device is connected on
     * @param statusfn callback invoked to report status/log messages
     * @param updateProgress callback invoked to report download progress
     * @param verifyProgress callback invoked to report verify progress
     * @param contexts the shared list of active bootloader contexts to append the new context to
     * @param addMutex mutex guarding concurrent access to contexts
     * @param new_context if non-null, receives the newly-created context
     * @return IS_OP_OK on success, otherwise an error
     */
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

    /**
     * Legacy multi-target entry point: identifies a serial-connected device's current mode and
     * drives it through however many reboot/update steps are needed to reach an up-to-date
     * application image, per filenames.
     * @param filenames candidate firmware/bootloader image paths for every supported target
     * @param port the serial port the device is connected on
     * @param statusfn callback invoked to report status/log messages
     * @param updateprogress callback invoked to report download progress
     * @param verifyProgress callback invoked to report verify progress
     * @param contexts the shared list of active bootloader contexts to append the new context to
     * @param addMutex mutex guarding concurrent access to contexts
     * @param new_context if non-null, receives the newly-created context
     * @param baud the serial baud rate to use
     * @return IS_OP_OK on success, otherwise an error
     */
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

    /**
     * Legacy multi-target entry point: identifies a USB-connected (DFU-mode) device and drives it
     * through however many reboot/update steps are needed to reach an up-to-date application image.
     * @param filenames candidate firmware/bootloader image paths for every supported target
     * @param handle an open libusb device handle for the connected device
     * @param statusfn callback invoked to report status/log messages
     * @param updateprogress callback invoked to report download progress
     * @param verifyProgress callback invoked to report verify progress
     * @param contexts the shared list of active bootloader contexts to append the new context to
     * @param addMutex mutex guarding concurrent access to contexts
     * @param new_context if non-null, receives the newly-created context
     * @return IS_OP_OK on success, otherwise an error
     */
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

    std::string m_filename;                 //!< path to the image most recently downloaded/uploaded/verified
    bool m_isISB = false;                   //!< true if this context represents an ISB-protocol session

protected:
    /**
     * Convenience wrapper to forward a single already-formatted message to the info callback.
     * @param info the message text (no printf-style formatting is applied)
     * @param level the severity of the message (one of eLogLevel)
     */
    void status_update(const char* info, eLogLevel level)
    {
        if (m_info_callback) m_info_callback(std::any_cast<cISBootloaderBase*>(this), level, info);
    }

    /** Legacy application-mode version info and the app-specific bootloader-enable command string. */
    struct
    {
        uint8_t uins_version[4];        //!< uINS-3 application version, as reported by the device
        uint8_t evb_version[4];         //!< EVB-2 application version, as reported by the device

        char enable_command[5];         //!< the ASCII command that switches this app into bootloader mode: "EBLE" (EVB) or "BLEN" (uINS)
    } m_app;

    /**
     * @brief Get the file extension from a file name
     * @param filename the file name/path to extract an extension from
     * @return pointer to the extension within filename (including the leading '.'), or "" if none
     */
    static const char* get_file_ext(const char* filename);
    
    /**
     * Determines the image signature of an Intel-HEX (.hex) image by inspecting its records.
     * @param image path to the .hex image file
     * @param major if non-null, receives the image's major version, when encoded in the image
     * @param minor if non-null, receives the image's minor version, when encoded in the image
     * @return an eImageSignature bitmask of the target(s) the image is valid for, or IS_IMAGE_SIGN_ERROR/IS_IMAGE_SIGN_NONE
     */
    static eImageSignature get_hex_image_signature(std::string image, uint8_t* major = NULL, char* minor = NULL);

    /**
     * Determines the image signature of a raw binary (.bin) image by inspecting its contents.
     * @param image path to the .bin image file
     * @param major if non-null, receives the image's major version, when encoded in the image
     * @param minor if non-null, receives the image's minor version, when encoded in the image
     * @return an eImageSignature bitmask of the target(s) the image is valid for, or IS_IMAGE_SIGN_ERROR/IS_IMAGE_SIGN_NONE
     */
    static eImageSignature get_bin_image_signature(std::string image, uint8_t* major = NULL, char* minor = NULL);
};

}

#endif
