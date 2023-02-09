/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_STM32_H
#define __IS_BOOTLOADER_STM32_H

#include "ISBootloaderBase.h"

class cISBootloaderSTM32 : public ISBootloader::cISBootloaderBase
{
public:
    cISBootloaderSTM32(
        std::string filename,
        ISBootloader::pfnBootloadProgress upload_cb,
        ISBootloader::pfnBootloadProgress verify_cb,
        ISBootloader::pfnBootloadStatus info_cb,
        serial_port_t* port
    ) : cISBootloaderBase{ filename, upload_cb, verify_cb, info_cb } 
    {
        m_port = port;
    }
    
    ~cISBootloaderSTM32() 
    {
        
    }

    is_operation_result match_test(void* param);
    
    is_operation_result reboot() {}
    is_operation_result reboot_up();

    uint32_t get_device_info();
    
    is_operation_result download_image(void);
    is_operation_result upload_image(void) { return IS_OP_OK; }
    is_operation_result verify_image(void) { return IS_OP_OK; }
    
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

#endif	// __IS_BOOTLOADER_ISB_H
