/**
 * @file ISBootloaderSony.h
 * @brief Bootloader-protocol implementation for the Sony CXD5610 GNSS receiver's bootloader
 *        protocol (framed opcode/checksum messages over UART), used to inject and execute the
 *        chip's updater program and write new SDK/app/lib/cfg firmware images.
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

#ifndef __IS_BOOTLOADER_SONY_H
#define __IS_BOOTLOADER_SONY_H

#include "ISBootloaderBase.h"

#ifndef MAX_PATH
#define MAX_PATH_SONY 260
#else
#define MAX_PATH_SONY MAX_PATH
#endif

/**
 * cISBootloaderBase implementation for the Sony CXD5610 GNSS receiver's bootloader protocol: a
 * framed, checksummed opcode message set sent over UART (CXD_SET_STATUS/PROGRAM_CODE_INJECTION/
 * PROGRAM_EXECUTION/WRITE_PROGRAM/etc.) used to restart the chip into bootloader mode, inject and
 * run its updater program, and write the SDK/app/lib/cfg firmware images that make up a release.
 */
class cISBootloaderSONY : public ISBootloader::cISBootloaderBase
{
public:
    /**
     * @param filename path to the firmware image this session will operate on
     * @param upload_cb callback invoked to report image-download progress; dummy_update_callback if null
     * @param verify_cb callback invoked to report image-verify progress; dummy_verify_callback if null
     * @param info_cb callback invoked to report status/log messages; dummy_info_callback if null
     * @param port the serial port the device is connected on
     */
    cISBootloaderSONY(
        std::string filename,
        fwUpdate::pfnProgressCb upload_cb,
        fwUpdate::pfnProgressCb verify_cb,
        fwUpdate::pfnStatusCb info_cb,
        port_handle_t port
  ) : cISBootloaderBase{ filename, upload_cb, verify_cb, info_cb }
    {
        m_port = port;
    }

    /** Destructor; the serial port is owned by the caller, not closed here. */
    ~cISBootloaderSONY()
    {

    }

    /**
     * @param param a null-terminated serial port name to compare against this device's port
     * @return IS_OP_OK if param matches this device's serial port name, otherwise IS_OP_ERROR
     */
    is_operation_result match_test(void* param);

    /** @return IS_OP_OK on success, otherwise IS_OP_ERROR */
    is_operation_result reboot();
    /** @return IS_OP_OK on success, otherwise IS_OP_ERROR; reboots up into the next bootloader/application level */
    is_operation_result reboot_up();

    /** @return 0 always (not implemented for this transport) */
    uint32_t get_device_info() {return 0; }

    /** @return IS_OP_OK on success, otherwise IS_OP_ERROR; writes the SDK/app/lib/cfg images from m_sony_filenames to the device */
    is_operation_result download_image(void);
    /** @return IS_OP_OK always (no-op; reading an image back is not supported over this transport) */
    is_operation_result upload_image(void) { return IS_OP_OK; }
    /** @return IS_OP_OK if the device's contents match the previously-downloaded image, otherwise IS_OP_ERROR */
    is_operation_result verify_image(void);

    /**
     * @param imgSign the eImageSignature bitmask of the candidate image to check against this device
     * @return non-zero if the connected CXD5610 device could be restarted into bootloader mode and imgSign is compatible with it, otherwise 0
     */
    uint8_t check_is_compatible(uint32_t imgSign);

    /** Paths to the individual firmware components that together make up one Sony CXD5610 release. */
    typedef struct
    {
        char updater[MAX_PATH_SONY];    //!< path to the bootloader updater program to inject and run on the device
        char app[MAX_PATH_SONY];        //!< path to the application firmware image
        char sdk[MAX_PATH_SONY];        //!< path to the Sony GNSS SDK image
        char lib[MAX_PATH_SONY];        //!< path to the supporting library image
        char cfg[MAX_PATH_SONY];        //!< path to the configuration image
    } m_sony_filenames;

private:
    int send_msg(uint8_t opcode, uint8_t* data, uint16_t len, uint16_t timeoutMs);
    int read_header(uint8_t* buf);
    uint8_t checksum(uint8_t* buf, uint16_t len);
    int read_bytes(FILE* file, uint8_t line[4086], int *bytesLeft);

    uint16_t m_oplen;
    uint8_t m_opcode;
    uint8_t m_data[4086];
};

#endif    // __IS_BOOTLOADER_ISB_H
