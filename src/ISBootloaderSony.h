/**
 * @file ISBootloaderSony.h
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

#ifndef __IS_BOOTLOADER_SONY_H
#define __IS_BOOTLOADER_SONY_H

#include "ISBootloaderBase.h"

#ifndef MAX_PATH
#define MAX_PATH_SONY 260
#else
#define MAX_PATH_SONY MAX_PATH
#endif

class cISBootloaderSONY : public ISBootloader::cISBootloaderBase
{
public:
    cISBootloaderSONY(
        std::string filename,
        ISBootloader::pfnBootloadProgress upload_cb,
        ISBootloader::pfnBootloadProgress verify_cb,
        ISBootloader::pfnBootloadStatus info_cb,
        serial_port_t* port
    ) : cISBootloaderBase{ filename, upload_cb, verify_cb, info_cb } 
    {
        m_port = port;
    }
    
    ~cISBootloaderSONY() 
    {
        
    }

    is_operation_result match_test(void* param);
    
    is_operation_result reboot();
    is_operation_result reboot_up();

    uint32_t get_device_info() {return 0; }
    
    is_operation_result download_image(void);
    is_operation_result upload_image(void) { return IS_OP_OK; }
    is_operation_result verify_image(void);
    
    uint8_t check_is_compatible(uint32_t imgSign);

    typedef struct 
    {
        char updater[MAX_PATH_SONY];
        char app[MAX_PATH_SONY];
        char sdk[MAX_PATH_SONY];
        char lib[MAX_PATH_SONY];
        char cfg[MAX_PATH_SONY];
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

#endif	// __IS_BOOTLOADER_ISB_H
