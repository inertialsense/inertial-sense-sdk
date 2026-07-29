/**
 * @file ISBootloaderISB.h
 * @brief Bootloader-protocol implementation for the legacy Inertial-Sense-Bootloader (ISB)
 *        ASCII/hex-record protocol used to update application images over a serial port
 *        (uINS-3/4, EVB-2, IMX-5), and to navigate up to APP or down to the ROM bootloader
 *        (SAM-BA/DFU) level.
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

#ifndef __IS_BOOTLOADER_ISB_H
#define __IS_BOOTLOADER_ISB_H

#include "ISBootloaderBase.h"

#include <mutex>

/**
 * cISBootloaderBase implementation for the legacy Inertial-Sense-Bootloader (ISB) protocol: an
 * ASCII/hex-record command set sent over a serial port to query device/version info, and to
 * erase/program/verify application flash. Also drives the reboot chain into APP mode (up) or
 * into the ROM bootloader (SAM-BA/DFU) mode (down).
 */
class cISBootloaderISB : public ISBootloader::cISBootloaderBase
{
public:
    /**
     * @param upload_cb callback invoked to report image-download progress; dummy_update_callback if null
     * @param verify_cb callback invoked to report image-verify progress; dummy_verify_callback if null
     * @param info_cb callback invoked to report status/log messages; dummy_info_callback if null
     * @param port the serial port the ISB-mode device is connected on
     */
    cISBootloaderISB(
        fwUpdate::pfnProgressCb upload_cb,
        fwUpdate::pfnProgressCb verify_cb,
        fwUpdate::pfnStatusCb info_cb,
        port_handle_t port
  ) : cISBootloaderBase{ upload_cb, verify_cb, info_cb }
    {
        m_port = port;
        m_bootloader_type = IS_BL_TYPE_ISB;
        m_port_name = std::string(portName(port));
    }

    /** Destructor; the serial port is owned by the caller, not closed here. */
    ~cISBootloaderISB()
    {

    }

    /**
     * @param param a null-terminated serial port name to compare against this device's port
     * @return IS_OP_OK if param matches this device's serial port name, otherwise IS_OP_ERROR
     */
    is_operation_result match_test(void* param);

    /** @return IS_OP_OK if the device was reset (see reboot_force()), IS_OP_CLOSED if the port had to be closed, otherwise IS_OP_ERROR */
    is_operation_result reboot();
    /** @return IS_OP_OK if the restart-bootloader command was sent successfully, otherwise IS_OP_ERROR */
    is_operation_result reboot_force();
    /** @return IS_OP_OK always; sends the "reboot to program mode" command and closes the port */
    is_operation_result reboot_up();
    /**
     * Reboots down into the ROM bootloader (SAM-BA/DFU) level if the device's ISB bootloader
     * version is older than major.minor (or force is set).
     * @param major if the target level requires a compatible image, its major version (0 = don't care)
     * @param minor if the target level requires a compatible image, its minor version (0 = don't care)
     * @param force if true, reboot even though the device's ISB version already appears up to date
     * @return IS_OP_OK if no update was needed or the reboot command was sent, otherwise IS_OP_ERROR
     */
    is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false);

    /** @return the device's Inertial Sense serial number and ISB bootloader version, or 0 if they could not be read */
    uint32_t get_device_info();

    /** @return the eImageSignature bitmask of images this device's ISB bootloader will accept, or IS_IMAGE_SIGN_NONE/error if it could not be determined */
    ISBootloader::eImageSignature check_is_compatible();

    /**
     * @brief Erases and programs the device's flash from an Intel-HEX image, page by page, over
     *        the ISB ASCII protocol.
     * @param image path to the Intel-HEX (.hex) image to flash
     * @return IS_OP_OK on success, otherwise IS_OP_ERROR
     */
    is_operation_result download_image(std::string image);
    /** @return IS_OP_OK always (no-op; reading an image back is not supported over ISB) */
    is_operation_result upload_image(std::string image) { return IS_OP_OK; }
    /**
     * @brief Verifies the device's flash contents against an Intel-HEX image over the ISB protocol.
     * @param image path to the Intel-HEX (.hex) image to verify against the device
     * @return IS_OP_OK if the device's contents match image, otherwise IS_OP_ERROR
     */
    is_operation_result verify_image(std::string image);

    /**
     * @brief Gets the version (e.g. 6a) from the bootloader file. Should be used in
     *  conjunction with the function that gets the signature from the firmware
     *  image.
     *
     * @param filename file name of the bootloader
     * @param major filled with major version
     * @param minor filled with minor version
     * @return is_operation_result
     */
    static is_operation_result get_version_from_file(const char* filename, uint8_t* major, char* minor);

    /**
     * Performs the ISB handshake sequence (repeated 'U' characters) required before the bootloader
     * will accept commands. A no-op if this instance has already handshaken successfully.
     * @param port the serial port to handshake over
     * @return IS_OP_OK once a handshake response is received, otherwise IS_OP_ERROR
     */
    is_operation_result handshake_sync(port_handle_t port);

    /** Clears the process-wide list of serial numbers already reset by reboot(). */
    static void reset_serial_list() { serial_list_mutex.lock(); serial_list.clear(); serial_list_mutex.unlock(); }

private:
    
    /**
     * @brief Calculate checksum for ISB
     * 
     * @param checkSum 
     * @param ptr 
     * @param start INCLUSIVE
     * @param end EXCLUSIVE
     * @param checkSumPosition if not 0, the checkSum is written to ptr + checkSumPosition
     * @param finalCheckSum 
     * @return int 
     */
    int checksum(int checkSum, uint8_t* ptr, int start, int end, int checkSumPosition, int finalCheckSum);

    is_operation_result erase_flash();
    is_operation_result select_page(int page);
    is_operation_result begin_program_for_current_page(int startOffset, int endOffset);
    
    int is_isb_read_line(FILE* file, char line[1024]);

    is_operation_result upload_hex_page(unsigned char* hexData, int byteCount, int* currentOffset, int* totalBytes, int* verifyCheckSum);
    is_operation_result upload_hex(unsigned char* hexData, int charCount, int* currentOffset, int* currentPage, int* totalBytes, int* verifyCheckSum);
    is_operation_result fill_current_page(int* currentPage, int* currentOffset, int* totalBytes, int* verifyCheckSum);
    is_operation_result download_data(int startOffset, int endOffset);

    bool hasHandshake = false;          // true if we've negotiated a handshake previously on this port/connection

    // Verification parameters
    int m_currentPage;
    int m_verifyCheckSum;

    is_operation_result process_hex_record(const std::string& record, int* verifyCheckSum);
    is_operation_result process_hex_file(FILE* file);

    struct {
        bool is_evb;                    // Available on version 6+, otherwise false
        ISBootloader::eProcessorType processor;       // Differentiates between uINS-3 and IMX-5
        bool rom_available;             // ROM bootloader is available on this port
        
        uint32_t app_offset;            // Helps in loading bin files
        uint32_t verify_size;           // Chunk size, limited on Windows
    } m_isb_props;

    static std::vector<uint32_t> serial_list;
    static std::mutex serial_list_mutex;

    static std::vector<uint32_t> rst_serial_list;
    static std::mutex rst_serial_list_mutex;

    int currentPage = 0;
    int currentOffset = m_isb_props.app_offset;

    static const int HEX_BUFFER_SIZE = 1024;
    unsigned char output[HEX_BUFFER_SIZE * 2]; // big enough to store an entire extra line of buffer if needed

};

#endif    // __IS_BOOTLOADER_ISB_H
