/**
 * @file ISBootloaderSamba.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating SAM-BA capable devices
 * 
 * @note This is a compatibility layer that calls inertialSenseBootLoader 
 *  routines for consistency with the new bootloader stuff (ISBootloaderCommon)
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderSamba.h"
#include "inertialSenseBootLoader.h"
#include "InertialSense.h"

is_operation_result is_samba_flash(
    const is_device_context const * context, 
    const char * firmware_path
)
{
    bootload_state_t state;

    memset(state.param.error, 0, BOOTLOADER_ERROR_LENGTH);
    serialPortPlatformInit(&state.serial);
    serialPortSetPort(&state.serial, portStrings.c_str());
    state.param.uploadProgress = uploadProgress;
    state.param.verifyProgress = verifyProgress;
    state.param.statusText = infoProgress;
    state.param.fileName = fileName.c_str();
    state.param.bootName = bootloaderFileName.c_str();
    state.param.forceBootloaderUpdate = forceBootloaderUpdate;
    state.param.port = &state.serial;
    state.param.verifyFileName = NULLPTR;
    state.param.flags.bitFields.enableVerify = (verifyProgress != NULLPTR);
    state.param.numberOfDevices = (int)state.size();
    state.param.baudRate = baudRate;			
    if (strstr(state.param.fileName, "EVB") != NULL)
    {   // Enable EVB bootloader
        strncpy(state.param.bootloadEnableCmd, "EBLE", 4);
    }
    else
    {	// Enable uINS bootloader
        strncpy(state.param.bootloadEnableCmd, "BLEN", 4);
    }

    // Update application and bootloader firmware
    state.thread = threadCreateAndStart(bootloaderUpdateBootloaderThread, &state);

    int bootloadFileEx(bootload_params_t* params);

    return IS_OP_OK;
}