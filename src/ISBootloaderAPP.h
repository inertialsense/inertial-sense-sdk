/**
 * @file ISBootloaderAPP.h
 * @brief Bootloader-protocol implementation for a device currently running its normal
 *        application firmware. Used to detect an application-mode device and command it to
 *        reboot down into IS-bootloader (ISB) mode so a real bootloader update can proceed;
 *        download/upload/verify are no-ops since an application image can't be flashed
 *        through this transport.
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

#ifndef __IS_BOOTLOADER_APP_H
#define __IS_BOOTLOADER_APP_H

#include "ISBootloaderBase.h"

#include <mutex>

/**
 * cISBootloaderBase implementation for a device running its normal application firmware
 * (not an actual bootloader). Used to detect an application-mode device on a serial port,
 * query its device info, and command it to reboot down into IS-bootloader (ISB) mode;
 * download/upload/verify are no-ops for this transport.
 */
class cISBootloaderAPP : public ISBootloader::cISBootloaderBase
{
public:
    /**
     * @param upload_cb callback invoked to report image-download progress; dummy_update_callback if null
     * @param verify_cb callback invoked to report image-verify progress; dummy_verify_callback if null
     * @param info_cb callback invoked to report status/log messages; dummy_info_callback if null
     * @param port the serial port the application-mode device is connected on
     */
    cISBootloaderAPP(
        fwUpdate::pfnProgressCb upload_cb,
        fwUpdate::pfnProgressCb verify_cb,
        fwUpdate::pfnStatusCb info_cb,
        port_handle_t port
  ) : cISBootloaderBase{ upload_cb, verify_cb, info_cb }
    {
        m_port = port;
        m_bootloader_type = IS_BL_TYPE_APP;
        // TODO? m_device_type = ISBootloader::IS_DEV_TYPE_APP;
        m_port_name = std::string(portName(port));
    }

    /** Destructor; the serial port is owned by the caller, not closed here. */
    ~cISBootloaderAPP()
    {

    }

    /** @return the eImageSignature bitmask of images this application-mode device will accept, queried over its normal comm protocol, or IS_IMAGE_SIGN_NONE if it could not be determined */
    ISBootloader::eImageSignature check_is_compatible();

    /**
     * @param param a null-terminated serial port name to compare against this device's port
     * @return IS_OP_OK if param matches this device's serial port name, otherwise IS_OP_ERROR
     */
    is_operation_result match_test(void* param);

    /** @return IS_OP_OK always (application-level reboot is not yet implemented) */
    is_operation_result reboot();
    /** @return IS_OP_OK always; there is no level above the running application */
    is_operation_result reboot_up() { return IS_OP_OK; }
    /**
     * Sends the app-specific bootloader-enable command sequence (m_app.enable_command) over the
     * serial port to reboot the device down into IS-bootloader (ISB) mode.
     * @param major unused for this transport
     * @param minor unused for this transport
     * @param force unused for this transport
     * @return IS_OP_OK always
     */
    is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false);

    /** @return the device's Inertial Sense serial number, queried over its normal comm protocol, or 0 if it could not be read */
    uint32_t get_device_info();

    /** @return IS_OP_OK always (no-op; an application-mode device can't be flashed through this transport) */
    is_operation_result download_image(std::string image) { return IS_OP_OK; }
    /** @return IS_OP_OK always (no-op; an application-mode device can't be read back through this transport) */
    is_operation_result upload_image(std::string image) { return IS_OP_OK; }
    /** @return IS_OP_OK always (no-op; an application-mode device can't be verified through this transport) */
    is_operation_result verify_image(std::string image) { return IS_OP_OK; }

    /** Clears the process-wide list of serial numbers already seen in application mode. */
    static void reset_serial_list() { serial_list_mutex.lock(); serial_list.clear(); serial_list_mutex.unlock(); }

private:
    static std::vector<uint32_t> serial_list;
    static std::mutex serial_list_mutex;
};

#endif    // __IS_BOOTLOADER_APP_H
