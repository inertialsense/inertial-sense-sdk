/**
 * @file ISBootloaderSAMBA.h
 * @brief Bootloader-protocol implementation for the Atmel SAM-BA ROM bootloader protocol used
 *        by SAMx70 parts (uINS-3/4, EVB-2) to erase and program the ISB bootloader image itself.
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

#ifndef __IS_BOOTLOADER_SAMBA_H
#define __IS_BOOTLOADER_SAMBA_H

#include "ISBootloaderBase.h"

/**
 * cISBootloaderBase implementation for the Atmel SAM-BA ROM bootloader protocol, present on
 * SAMx70 parts (uINS-3/4, EVB-2). This level sits below ISB in the reboot chain and is used
 * specifically to erase and reprogram the ISB bootloader image itself (word-level peripheral
 * reads/writes and page-level flash programming over a serial UART/USB CDC connection).
 */
class cISBootloaderSAMBA : public ISBootloader::cISBootloaderBase
{
public:
    /**
     * @param upload_cb callback invoked to report image-download progress; dummy_update_callback if null
     * @param verify_cb callback invoked to report image-verify progress; dummy_verify_callback if null
     * @param info_cb callback invoked to report status/log messages; dummy_info_callback if null
     * @param port the serial port the SAM-BA-mode device is connected on
     */
    cISBootloaderSAMBA(
        fwUpdate::pfnProgressCb upload_cb,
        fwUpdate::pfnProgressCb verify_cb,
        fwUpdate::pfnStatusCb info_cb,
        port_handle_t port
  ) : cISBootloaderBase{ upload_cb, verify_cb, info_cb }
    {
        m_port = (port_handle_t)port;
        m_port_name = std::string(portName(port));
        m_bootloader_type = IS_BL_TYPE_SAMBA;
    }

    /** Destructor; the serial port is owned by the caller, not closed here. */
    ~cISBootloaderSAMBA()
    {

    }

    /**
     * @param param a null-terminated serial port name to compare against this device's port
     * @return IS_OP_OK if param matches this device's serial port name, otherwise IS_OP_ERROR
     */
    is_operation_result match_test(void* param);

    /** @return IS_OP_OK always; issues a processor reset via the reset controller register */
    is_operation_result reboot();
    /** @return IS_OP_OK on success, otherwise IS_OP_ERROR; sets the GPNVM boot-from-flash bit then reboots into IS-bootloader mode */
    is_operation_result reboot_up();
    /** @return IS_OP_OK always (no level below SAM-BA to reboot into for this transport) */
    is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false) { (void)major; (void)minor; (void)force; return IS_OP_OK; }

    /** @return the device's Inertial Sense serial number, or 0 if it could not be read */
    uint32_t get_device_info();

    /**
     * @brief Erases and programs the device's flash from an Intel-HEX image, page by page, over
     *        the SAM-BA protocol.
     * @param image path to the Intel-HEX (.hex) image to flash (the ISB bootloader image)
     * @return IS_OP_OK on success, otherwise IS_OP_ERROR
     */
    is_operation_result download_image(std::string image);
    /** @return IS_OP_OK always (no-op; reading an image back is not supported over SAM-BA) */
    is_operation_result upload_image(std::string image) { return IS_OP_OK; }
    /**
     * @brief Verifies the device's flash contents against an Intel-HEX image over the SAM-BA protocol.
     * @param image path to the Intel-HEX (.hex) image to verify against the device
     * @return IS_OP_OK if the device's contents match image, otherwise IS_OP_ERROR
     */
    is_operation_result verify_image(std::string image);

    /**
     * @brief Check if the referenced device is a SAM-BA device, and that the image matches
     *
     */
    ISBootloader::eImageSignature check_is_compatible();

private:

    static constexpr int SAMBA_PAGE_SIZE = 512;

    is_operation_result erase_flash();

    /**
     * @brief Read a single (32-bit) word from the device. Can read from any peripheral or memory
     *
     * @param address Address to read at
     * @param word Filled with the read value at return
     */
    is_operation_result read_word(uint32_t address, uint32_t* word);

    /**
     * @brief Write a single (32-bit) word to the device. Can write to any peripheral or memory
     *
     * @param address Address to write at
     * @param word Value to write
     */
    is_operation_result write_word(uint32_t address, uint32_t word);

    /**
     * @brief Wait for the embedded flash controller to be ready or not ready
     *
     * @param waitReady if `true`, wait until controller is ready. `false`, wait until not ready
     */
    is_operation_result wait_eefc_ready(bool waitReady);

    /**
     * @brief Write a buffer to the device if connected via UART, minding rules (not
     *  sure where the rules are documented) of the UART connection
     *
     * @param buf buffer to send to device
     * @param len length of buffer
     */
    is_operation_result write_uart_modem(uint8_t* buf, size_t len);

   /**
     * @brief Erase, then write a page of flash on the device
     *
     * @param offset offset from the base address of the flash memory to write at
     * @param data buffer to write. Must be at least `SAMBA_PAGE_SIZE` long
     * @param isUSB if the device is connected over UART, different rules apply for transmission
     */
    is_operation_result flash_erase_write_page(size_t offset, uint8_t data[SAMBA_PAGE_SIZE], bool isUSB);

    /**
     * @brief CRC16 generator
     * 
     * @param data buffer to compute crc16 on
     * @param size length of buffer
     * @return uint16_t crc16 value
     */
    uint16_t crc16(uint8_t* data, uint16_t size);

    /**
     * @brief CRC16 helper function
     * 
     */
    uint16_t crc_update(uint16_t crc_in, int incr);

    // For verification
    uint32_t checksum;
};

#endif    // __IS_BOOTLOADER_ISB_H
