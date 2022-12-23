/**
 * @file ISBootloaderSAMBA.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating ISB images using SAM-BA protocol.
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_SAMBA_H
#define __IS_BOOTLOADER_SAMBA_H

#include "ISBootloaderBase.h"

class cISBootloaderSAMBA : public ISBootloader::cISBootloaderBase
{
public:
    cISBootloaderSAMBA( 
        ISBootloader::pfnBootloadProgress upload_cb,
        ISBootloader::pfnBootloadProgress verify_cb,
        ISBootloader::pfnBootloadStatus info_cb,
        serial_port_t* port
    ) : cISBootloaderBase{ upload_cb, verify_cb, info_cb } 
    {
        m_port = port;
        m_device_type = ISBootloader::IS_DEV_TYPE_SAMBA;
    }
    
    ~cISBootloaderSAMBA() 
    {
        
    }

    is_operation_result match_test(void* param);
    
    is_operation_result reboot();
    is_operation_result reboot_up();
    is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false) { (void)major; (void)minor; (void)force; return IS_OP_OK; }

    uint32_t get_device_info();
    
    is_operation_result download_image(std::string image);
    is_operation_result upload_image(std::string image) { return IS_OP_OK; }
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
     * @param ctx device context with open serial port registered under `handler`
     * @param address Address to read at
     * @param word Filled with the read value at return
     */
    is_operation_result read_word(uint32_t address, uint32_t* word);

    /**
     * @brief Write a single (32-bit) word to the device. Can write to any peripheral or memory
     * 
     * @param ctx device context with open serial port registered under `handler`
     * @param address Address to write at
     * @param word Value to write
     */
    is_operation_result write_word(uint32_t address, uint32_t word);

    /**
     * @brief Wait for the embedded flash controller to be ready or not ready
     * 
     * @param ctx device context with open serial port registered under `handler`
     * @param waitReady if `true`, wait until controller is ready. `false`, wait until not ready
     */
    is_operation_result wait_eefc_ready(bool waitReady);

    /**
     * @brief Write a buffer to the device if connected via UART, minding rules (not 
     *  sure where the rules are documented) of the UART connection
     * 
     * @param ctx device context with open serial port registered under `handler`
     * @param buf buffer to send to device
     * @param len length of buffer
     */
    is_operation_result write_uart_modem(uint8_t* buf, size_t len);

   /**
     * @brief Erase, then write a page of flash on the device
     * 
     * @param ctx device context with open serial port registered under `handler`
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

#endif	// __IS_BOOTLOADER_ISB_H
