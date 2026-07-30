/**
 * @file ISBootloaderSTM32.h
 * @brief Bootloader-protocol implementation for the STM32 UART ROM bootloader (the same
 *        USART-based command set exposed by the chip's built-in bootloader before DFU/ISB is
 *        available), used to erase/read/write flash and jump to an application entry point.
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

#ifndef __IS_BOOTLOADER_STM32_H
#define __IS_BOOTLOADER_STM32_H

#include "ISBootloaderBase.h"

/**
 * cISBootloaderBase implementation for the STM32 ROM bootloader's UART command protocol
 * (GET/GET_VERSION/GET_ID/READ_MEMORY/WRITE_MEMORY/GO/erase, per ST AN3155). Used to identify
 * the connected STM32 part, and to erase, program, and jump into a flashed application image.
 */
class cISBootloaderSTM32 : public ISBootloader::cISBootloaderBase
{
public:
    /**
     * @param filename path to the firmware image this session will operate on
     * @param upload_cb callback invoked to report image-download progress; dummy_update_callback if null
     * @param verify_cb callback invoked to report image-verify progress; dummy_verify_callback if null
     * @param info_cb callback invoked to report status/log messages; dummy_info_callback if null
     * @param port the serial port the device is connected on
     */
    cISBootloaderSTM32(
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
    ~cISBootloaderSTM32()
    {

    }

    /**
     * @param param a null-terminated serial port name to compare against this device's port
     * @return IS_OP_OK if param matches this device's serial port name, otherwise IS_OP_ERROR
     */
    is_operation_result match_test(void* param);

    /** @return IS_OP_OK on success, otherwise IS_OP_ERROR */
    is_operation_result reboot() {}
    /** @return IS_OP_OK on success, otherwise IS_OP_ERROR; reboots up into the next bootloader/application level */
    is_operation_result reboot_up();

    /** @return the device's Inertial Sense serial number, or 0 if it could not be read */
    uint32_t get_device_info();

    /** @return IS_OP_OK on success, otherwise IS_OP_ERROR; erases and programs flash from the image path passed to the constructor */
    is_operation_result download_image(void);
    /** @return IS_OP_OK always (no-op; reading an image back is not supported over this transport) */
    is_operation_result upload_image(void) { return IS_OP_OK; }
    /** @return IS_OP_OK always (no-op; verification is not supported over this transport) */
    is_operation_result verify_image(void) { return IS_OP_OK; }

    /**
     * @param imgSign the eImageSignature bitmask of the candidate image to check against this device
     * @return non-zero if the connected STM32 device ID matches a known/supported part and imgSign is compatible with it, otherwise 0
     */
    uint8_t check_is_compatible(uint32_t imgSign);

private:
    typedef struct
    {
        uint32_t addr;
        uint8_t *data;
        uint8_t len;    // Length to read/write *MINUS ONE*
    } stm32_data_t;

    uint8_t send_command(uint8_t cmd);
    uint8_t checkAck(void);
    void xorCompute(uint8_t *chksum, uint8_t *data, uint16_t len);
    
    /** Copy address into correct byte oredr in buffer, and validate it */
    uint8_t addrBufCopy(uint32_t addr, uint8_t *buf);
    
    /** Get commands */
    uint8_t get(void);
    uint8_t get_version(void);
    uint8_t get_id(void);

    /** Memory manipulation commands */
    uint8_t mass_erase(void);
    uint8_t read_memory(stm32_data_t *data);
    uint8_t write_memory(stm32_data_t *data);

    /** Execution commands */
    uint8_t go(uint32_t addr);

    uint8_t m_valid_commands[32];
    uint8_t m_version;
    uint8_t m_pid;
};

#endif    // __IS_BOOTLOADER_ISB_H
