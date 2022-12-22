/**
 * @file ISBootloaderAPP.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for putting APP mode devices in ISB mode
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_APP_H
#define __IS_BOOTLOADER_APP_H

#include "ISBootloaderBase.h"

#include <mutex>

class cISBootloaderAPP : public ISBootloader::cISBootloaderBase
{
public:
    cISBootloaderAPP(
        ISBootloader::pfnBootloadProgress upload_cb,
        ISBootloader::pfnBootloadProgress verify_cb,
        ISBootloader::pfnBootloadStatus info_cb,
        serial_port_t* port
    ) : cISBootloaderBase{ upload_cb, verify_cb, info_cb } 
    {
        m_port = port;
        m_device_type = ISBootloader::IS_DEV_TYPE_APP;
    }

    ~cISBootloaderAPP() 
    {
        
    }

    ISBootloader::eImageSignature check_is_compatible();

    is_operation_result match_test(void* param);

    is_operation_result reboot();
    is_operation_result reboot_up() { return IS_OP_OK; }
    is_operation_result reboot_down(uint8_t major = 0, char minor = 0, bool force = false);

    uint32_t get_device_info();

    is_operation_result download_image(std::string image) { return IS_OP_OK; }
    is_operation_result upload_image(std::string image) { return IS_OP_OK; }
    is_operation_result verify_image(std::string image) { return IS_OP_OK; }

    static void reset_serial_list() { serial_list_mutex.lock(); serial_list.clear(); serial_list_mutex.unlock(); }

private:
    static std::vector<uint32_t> serial_list;
    static std::mutex serial_list_mutex;
};

#endif	// __IS_BOOTLOADER_APP_H
