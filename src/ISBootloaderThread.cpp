/**
 * @file ISBootloaderThread.cpp
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

#include "ISBootloaderThread.h"
#include "ISBootloaderDFU.h"
#include "ISSerialPort.h"

#include <algorithm>

using namespace std;

vector<is_device_context*> ISBootloader::ctx;
std::string ISBootloader::m_firmware;
pfnBootloadProgress ISBootloader::m_uploadProgress; 
pfnBootloadProgress ISBootloader::m_verifyProgress;
pfnBootloadStatus ISBootloader::m_infoProgress;
void* ISBootloader::m_user_data;
int ISBootloader::m_baudRate;
void (*ISBootloader::m_waitAction)();
uint32_t ISBootloader::m_timeStart;

void ISBootloader::update_thread(void* context)
{
    is_device_context* ctx = (is_device_context*)context;

    if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL)
    {
        serialPortClose(&ctx->handle.port);
        if(!serialPortOpenRetry(&ctx->handle.port, ctx->handle.port_name, m_baudRate, 1)) 
        {
            return;
        }
    }

    SLEEP_MS(100);

    is_update_flash(context);

    if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL)
    {
        serialPortClose(&ctx->handle.port);
    }

    SLEEP_MS(1000);
}

void ISBootloader::update_finish(void* context)
{
    is_device_context* ctx = (is_device_context*)context;

    if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL)
    {
        serialPortClose(&ctx->handle.port);
        if(!serialPortOpenRetry(&ctx->handle.port, ctx->handle.port_name, m_baudRate, 1)) 
        {
            return;
        }
    }

    is_update_finish(context);

    if(ctx->handle.status == IS_HANDLE_TYPE_SERIAL)
    {
        serialPortClose(&ctx->handle.port);
    }
}

is_operation_result ISBootloader::update(
    vector<string>&             comPorts,   // ISB and SAM-BA and APP
    vector<string>&             uids,       // DFU only
    int                         baudRate,
    const char*                 firmware,
    pfnBootloadProgress         uploadProgress,
    pfnBootloadProgress         verifyProgress,
    pfnBootloadStatus           infoProgress,
    void*                       user_data,
    void						(*waitAction)()
)
{
    (void)uids;

    m_firmware = firmware;
    m_uploadProgress = uploadProgress;
    m_verifyProgress = verifyProgress;
    m_infoProgress = infoProgress;
    m_user_data = user_data;
    m_baudRate = baudRate;
    m_waitAction = waitAction;
    m_timeStart = current_timeMs();

    int noChange = 0;

    ctx.clear();
    
    // DFU stuff
    is_dfu_list dfu_list;
    bool use_dfu = false;
    
    // Serial port stuff
    vector<string> ports;
    vector<string> ports_user_ignore;
    vector<string> ports_active;
    cISSerialPort::GetComPorts(ports);
    sort(ports.begin(), ports.end());
    sort(comPorts.begin(), comPorts.end());
    set_symmetric_difference(   // Get the difference between the specified ports and the ports present TODO: make this not symmetric
        ports.begin(), ports.end(), 
        comPorts.begin(), comPorts.end(),
        back_inserter(ports_user_ignore));

    // Only turn on DFU updates if signature is for a STM32L4 bootloader
    const char * extension = get_file_ext(firmware);
    is_image_signature file_signature = IS_IMAGE_SIGN_NONE;
    if(strcmp(extension, "hex") == 0)
    {
        file_signature = is_get_hex_image_signature(firmware);
    }
    if(file_signature == IS_IMAGE_SIGN_ISB_STM32L4 && libusb_init(NULL) >= 0)
    {
        use_dfu = true;
    }

    while(1)
    {
//        for(int i = 0; i < 10; i++)
        {   // Sleep and update UI for 100ms before looping through device management again
            SLEEP_MS(10);
            if(m_waitAction) m_waitAction();
        }

        if(use_dfu)
        {
            is_dfu_list_devices(&dfu_list);

            for(size_t i = 0; i < dfu_list.present; i++)
            {	// Create contexts for devices in DFU mode
                bool found = false;

                for(size_t j = 0; j < ctx.size(); j++)
                {
                    if( ctx[j]->handle.status == IS_HANDLE_TYPE_LIBUSB && 
                        (strncmp(ctx[j]->handle.dfu.uid, dfu_list.id[i].uid, IS_DFU_UID_MAX_SIZE) == 0)
                    )
                    {   // We found the device in the context list
                        found = true;
                        break;
                    }
                    else
                    {
                        found = false;
                    }
                }

                if(!found && strlen(dfu_list.id[i].uid) != 0)
                {   // If we didn't find the device
                    is_device_handle handle;
                    memset(&handle, 0, sizeof(is_device_handle));
                    handle.status = IS_HANDLE_TYPE_LIBUSB;
                    strncpy(handle.dfu.uid, dfu_list.id[i].uid, IS_DFU_UID_MAX_SIZE);
                    handle.dfu.vid = dfu_list.id[i].vid;
                    handle.dfu.pid = dfu_list.id[i].pid;
                    handle.dfu.handle_libusb = dfu_list.id[i].handle_libusb;
                    ctx.push_back(is_create_context(
                        &handle,
                        m_firmware.c_str(), 
                        m_uploadProgress, 
                        m_verifyProgress, 
                        m_infoProgress,
                        m_user_data
                    ));
                }
            }
        }

        
        // Get a list of com ports that have threads associated
        ports_active.clear();
        for(size_t i = 0; i < ctx.size(); i++)
        {
            if((ctx[i]->thread || ctx[i]->finished_flash) && ctx[i]->handle.status == IS_HANDLE_TYPE_SERIAL)
            {
                ports_active.push_back(string(ctx[i]->handle.port_name));
            }
        }

        // Get list of all com ports
        cISSerialPort::GetComPorts(ports);

        for(size_t i = 0; i < ports.size(); i++)
        {	
            if(find(ports_active.begin(), ports_active.end(), ports[i]) != ports_active.end() ||
                find(ports_user_ignore.begin(), ports_user_ignore.end(), ports[i]) != ports_user_ignore.end())
            {
                // Ignoring this device because it was not specified by user, 
                //  or it already has a thread associated
            }
            else
            {   // Create context for device
                is_device_handle handle;
                memset(&handle, 0, sizeof(is_device_handle));
                handle.status = IS_HANDLE_TYPE_SERIAL;
                handle.baud = m_baudRate;
                strncpy(handle.port_name, ports[i].c_str(), 100);
                ctx.push_back(is_create_context(
                    &handle,
                    m_firmware.c_str(), 
                    m_uploadProgress, 
                    m_verifyProgress, 
                    m_infoProgress,
                    m_user_data
                ));
            }
        }

        for(size_t i = 0; i < ctx.size(); i++)
        {   // Join threads that have finished
            if((ctx[i]->thread != NULL) && (!ctx[i]->update_in_progress))
            {
                threadJoinAndFree(ctx[i]->thread);
                ctx[i]->thread = NULL;
                m_timeStart = current_timeMs();
            }
        }

        noChange = 0;

        for(size_t i = 0; i < ctx.size(); i++)
        {   
            if(!ctx[i]->thread && ctx[i]->update_in_progress && ctx[i]->retries_left-- > 0)
            {
                ctx[i]->start_time_ms = current_timeMs();   // Unused so far
                ctx[i]->thread = threadCreateAndStart(update_thread, (void*)ctx[i]);
            }
            
            if(ctx[i]->thread || ctx[i]->update_in_progress)
            {
                noChange = 1;
            }
        }

        if(!noChange && (current_timeMs() - m_timeStart > 5000))
        {
            break;    // After 5 seconds of no changes and no threads running, quit
        }
    }

    for(size_t i = 0; i < ctx.size(); i++)
    {   
        if (ctx[i]->finished_flash)
        {
            ctx[i]->thread = threadCreateAndStart(update_finish, (void*)ctx[i]);
        }
    }

    for(size_t i = 0; i < ctx.size(); i++)
    {   // Free context memory
        if (ctx[i]->thread)
        {
            threadJoinAndFree(ctx[i]->thread);
        }
        // is_destroy_context(ctx[i]);
    }

    if(use_dfu) { libusb_exit(NULL); }

    return IS_OP_OK;
}
