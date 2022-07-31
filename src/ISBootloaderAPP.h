/**
 * @file ISBootloaderAPP.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for putting APP mode devices in ISB mode
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_APP_H
#define __IS_BOOTLOADER_APP_H

#include "ISBootloaderBase.h"

class cISBootloaderAPP : public ISBootloader::cISBootloaderBase
{
public:
    cISBootloaderAPP(
        ISBootloader::pfnBootloadProgress upload_cb,
        ISBootloader::pfnBootloadProgress verify_cb,
        ISBootloader::pfnBootloadStatus info_cb,
        const char* port_name
    ) : cISBootloaderBase{ upload_cb, verify_cb, info_cb } 
    {
        serialPortPlatformInit(&m_port);
        serialPortSetPort(&m_port, port_name);
        serialPortOpen(&m_port, port_name, 921600, 100);
        m_device_type = ISBootloader::IS_DEV_TYPE_APP;
    }

    ~cISBootloaderAPP() 
    {
        serialPortClose(&m_port);
    }

    static is_operation_result check_is_compatible(const char* handle, ISBootloader::eImageSignature file);

    is_operation_result match_test(void* param);

    is_operation_result reboot();
    is_operation_result reboot_up() { return IS_OP_OK; }
    is_operation_result reboot_down();

    uint32_t get_device_info();

    is_operation_result download_image(std::string image) { return IS_OP_OK; }
    is_operation_result upload_image(std::string image) { return IS_OP_OK; }
    is_operation_result verify_image(std::string image) { return IS_OP_OK; }

private:
    struct
    {
        uint8_t uins_version[4]; 
        uint8_t evb_version[4];        
        
        char enable_command[5];         // "EBLE" (EVB) or "BLEN" (uINS) 
    } m_app;
};

#endif	// __IS_BOOTLOADER_APP_H
