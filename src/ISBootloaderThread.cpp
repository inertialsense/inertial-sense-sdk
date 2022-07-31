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
using namespace ISBootloader;

vector<cISBootloaderBase*> cISBootloaderThread::ctx;
std::string cISBootloaderThread::m_firmware;
pfnBootloadProgress cISBootloaderThread::m_uploadProgress; 
pfnBootloadProgress cISBootloaderThread::m_verifyProgress;
pfnBootloadStatus cISBootloaderThread::m_infoProgress;
int cISBootloaderThread::m_baudRate;
void (*cISBootloaderThread::m_waitAction)();
uint32_t cISBootloaderThread::m_timeStart;
std::vector<std::string> cISBootloaderThread::ports_user_ignore;
std::vector<std::string> cISBootloaderThread::ports_active;

void cISBootloaderThread::update_thread(void* context)
{
    cISBootloaderBase* ctx = (cISBootloaderBase*)context;

    ctx->get_device_info();
    ctx->download_image(m_firmware);

    ctx->m_update_in_progress = false;
}

void cISBootloaderThread::update_finish(void* context)
{
    cISBootloaderBase* ctx = (cISBootloaderBase*)context;

    ctx->get_device_info();
    ctx->reboot_up();

    ctx->m_update_in_progress = false;
}

void cISBootloaderThread::put_device_in_mode(void* context)
{
    cISBootloaderBase* ctx = (cISBootloaderBase*)context;

    ctx->get_device_info();
    ctx->reboot_to_update_level(m_firmware);

    ctx->m_update_in_progress = false;
}

is_operation_result cISBootloaderThread::manage_devices(bool use_dfu, eBootloaderRunmode mode)
{
    is_dfu_list dfu_list;
    vector<string> ports;

    if(use_dfu)
    {
        cISBootloaderDFU::list_devices(&dfu_list);

        for(size_t i = 0; i < dfu_list.present; i++)
        {	// Create contexts for devices in DFU mode
            bool found = false;

            for(size_t j = 0; j < ctx.size(); j++)
            {
                if(ctx[j]->match_test((void*)dfu_list.id[i].uid))
                {   // We found the device in the context list
                    found = true;
                    break;
                }
            }

            if(!found && strlen(dfu_list.id[i].uid) != 0)
            {   // If we didn't find the device
                cISBootloaderBase* ctx_new;
                if (cISBootloaderBase::add_device_to_list(m_firmware, dfu_list.id[i].handle_libusb, dfu_list.id[i].uid, &ctx_new, m_infoProgress) == IS_OP_OK)
                {
                    ctx.push_back(ctx_new);
                }
            }
        }
    }

    // Get a list of com ports that have threads of completed devices associated
    ports_active.clear();
    for(size_t i = 0; i < ctx.size(); i++)
    {
        if((ctx[i]->m_thread || ctx[i]->m_finished_flash) && ctx[i]->is_serial_device())
        {
            ports_active.push_back(string(ctx[i]->m_port.port));
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
            cISBootloaderBase* ctx_new;
            if (cISBootloaderBase::add_device_to_list(m_firmware, ports[i].c_str(), &ctx_new, m_infoProgress) == IS_OP_OK)
            {
                ctx.push_back(ctx_new);
            }
        }
    }
    
    int devicesRunning = 0;

    for(size_t i = 0; i < ctx.size(); i++)
    {   
        if(!ctx[i]->m_thread && ctx[i]->m_update_in_progress && ctx[i]->m_retries_left-- > 0)
        {
            switch(mode)
            {
            case IS_BOOTLOADER_RUNMODE_REBOOT_DOWN:
                ctx[i]->m_thread = threadCreateAndStart(put_device_in_mode, (void*)ctx[i]);  
                break;
            case IS_BOOTLOADER_RUNMODE_FLASH:
                ctx[i]->m_thread = threadCreateAndStart(update_thread, (void*)ctx[i]);
                break;
            case IS_BOOTLOADER_RUNMODE_REBOOT_UP:
                ctx[i]->m_thread = threadCreateAndStart(update_finish, (void*)ctx[i]);
                break;
            }
        }
        
        if(ctx[i]->m_thread || ctx[i]->m_update_in_progress)
        {
            devicesRunning++;
        }
    }

    if(devicesRunning == 0 && (current_timeMs() - m_timeStart > 5000))
    {
        return IS_OP_CANCELLED;    // After 5 seconds of no changes and no threads running, quit
    }

    bool join_and_free = true;
    for(size_t i = 0; i < ctx.size(); i++)
    {   // Check that all threads have finished updating
        if(ctx[i]->m_update_in_progress)
        {
            join_and_free = false;
        }
    }

    if(join_and_free)
    {
        for(size_t i = 0; i < ctx.size(); i++)
        {   // Join threads if all have finished
            if(ctx[i]->m_thread)
            {
                threadJoinAndFree(ctx[i]->m_thread);
                ctx[i]->m_thread = NULL;
            }
        }

        return IS_OP_CANCELLED;
    }

    return IS_OP_OK;
}

is_operation_result cISBootloaderThread::update(
    vector<string>&             comPorts,   // ISB and SAM-BA and APP
    int                         baudRate,
    std::string                 firmware,
    pfnBootloadProgress         uploadProgress,
    pfnBootloadProgress         verifyProgress,
    pfnBootloadStatus           infoProgress,
    void						(*waitAction)()
)
{
    m_firmware = firmware;
    m_uploadProgress = uploadProgress;
    m_verifyProgress = verifyProgress;
    m_infoProgress = infoProgress;
    m_baudRate = baudRate;
    m_waitAction = waitAction;
    m_timeStart = current_timeMs();

    int devicesRunning = 0;

    for(size_t i = 0; i < ctx.size(); i++)
    {
        if(ctx[i] != nullptr) 
        {
            delete ctx[i]; 
            ctx[i] = nullptr;
        }
    }
    ctx.clear();
    
    // DFU stuff
    bool use_dfu = false;
    vector<string> ports;
    
    // Serial port stuff
    cISSerialPort::GetComPorts(ports);
    sort(ports.begin(), ports.end());
    sort(comPorts.begin(), comPorts.end());
    set_symmetric_difference(   // Get the difference between the specified ports and the ports present TODO: make this not symmetric
        ports.begin(), ports.end(), 
        comPorts.begin(), comPorts.end(),
        back_inserter(ports_user_ignore));

    if(libusb_init(NULL) >= 0)
    {
        use_dfu = true;
    }

    while(1)
    {
        SLEEP_MS(10);
        if(m_waitAction) m_waitAction();

        if(manage_devices(use_dfu, IS_BOOTLOADER_RUNMODE_REBOOT_DOWN) == IS_OP_CANCELLED) break;
    }

    while(1)
    {
        SLEEP_MS(10);
        if(m_waitAction) m_waitAction();

        if(manage_devices(use_dfu, IS_BOOTLOADER_RUNMODE_FLASH) == IS_OP_CANCELLED) break;
    }

    while(1)
    {
        SLEEP_MS(10);
        if(m_waitAction) m_waitAction();

        if(manage_devices(use_dfu, IS_BOOTLOADER_RUNMODE_REBOOT_UP) == IS_OP_CANCELLED) break;
    }

    if(use_dfu) { libusb_exit(NULL); }

    return IS_OP_OK;
}
