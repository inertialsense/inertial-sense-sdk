/**
 * @file ISBootloader.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating embedded systems
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

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

#include "ISBootloaderCommon.h"
#include "ISUtilities.h"

using namespace std;

class ISBootloader
{
public:
    ISBootloader() {};
    ~ISBootloader() {};

    /**
     * @brief Update flash on specified Inertial Sense devices
     * 
     * @param comPorts 
     * @param baudRate 
     * @param firmware 
     * @param results 
     * @param uploadProgress 
     * @param verifyProgress 
     * @param infoProgress 
     * @return is_operation_result 
     */
    static is_operation_result update(
        vector<string>&             comPorts,
        int                         baudRate,
        is_firmware_settings*       firmware,
        pfnBootloadProgress         uploadProgress, 
        pfnBootloadProgress         verifyProgress, 
        pfnBootloadStatus           infoProgress
    );

    static vector<is_device_context*> ctx;

private:
    static void update_thread(void* context);
    
};

#endif // __IS_BOOTLOADER_H_