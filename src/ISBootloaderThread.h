/**
 * @file ISBootloaderThread.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating embedded systems in parallel
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_H_
#define __IS_BOOTLOADER_H_

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "ISUtilities.h"
#include "ISBootloaderBase.h"

class cISBootloaderThread
{
public:
    cISBootloaderThread() {};
    ~cISBootloaderThread() {};

    static is_operation_result update(
        std::vector<std::string>&   comPorts,
        int                         baudRate,
        std::string                 firmware,
        ISBootloader::pfnBootloadProgress         uploadProgress, 
        ISBootloader::pfnBootloadProgress         verifyProgress,
        ISBootloader::pfnBootloadStatus           infoProgress,
        void						(*waitAction)()
    );

    static std::vector<ISBootloader::cISBootloaderBase*> ctx;

private:
    typedef enum {
        IS_BOOTLOADER_RUNMODE_REBOOT_DOWN,
        IS_BOOTLOADER_RUNMODE_FLASH,
        IS_BOOTLOADER_RUNMODE_REBOOT_UP
    } eBootloaderRunmode;
   
    static is_operation_result manage_devices(bool use_dfu, eBootloaderRunmode mode);
    static void update_thread(void* context);
    static void update_finish(void* context);
    static void put_device_in_mode(void* context);
    static std::string m_firmware;
    static ISBootloader::pfnBootloadProgress m_uploadProgress; 
    static ISBootloader::pfnBootloadProgress m_verifyProgress;
    static ISBootloader::pfnBootloadStatus m_infoProgress;
    static int m_baudRate;
    static void (*m_waitAction)();
    static uint32_t m_timeStart;
    static std::vector<std::string> ports_user_ignore;
    static std::vector<std::string> ports_active;

    
};

#endif // __IS_BOOTLOADER_H_
