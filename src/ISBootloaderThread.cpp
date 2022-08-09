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
#include "ISBootloaderAPP.h"
#include "ISBootloaderISB.h"
#include "ISBootloaderSAMBA.h"
#include "ISSerialPort.h"

#include <algorithm>

using namespace std;
using namespace ISBootloader;

vector<cISBootloaderBase*> cISBootloaderThread::ctx;
firmwares_t cISBootloaderThread::m_firmware;
pfnBootloadProgress cISBootloaderThread::m_uploadProgress; 
pfnBootloadProgress cISBootloaderThread::m_verifyProgress;
pfnBootloadStatus cISBootloaderThread::m_infoProgress;
int cISBootloaderThread::m_baudRate;
void (*cISBootloaderThread::m_waitAction)();
uint32_t cISBootloaderThread::m_timeStart;
std::mutex cISBootloaderThread::ctx_mutex;
std::vector<void*> cISBootloaderThread::threads;
bool cISBootloaderThread::m_update_rom = false;
std::mutex cISBootloaderThread::serial_thread_mutex;
std::mutex cISBootloaderThread::libusb_thread_mutex;
bool cISBootloaderThread::m_update_in_progress = false;

typedef struct 
{
    void* thread;
    char serial_name[100];
    cISBootloaderBase* ctx;
    bool done;
    bool reuse_port;
} thread_serial_t;

typedef struct 
{
    void* thread;
    libusb_device_handle* handle;
    char uid[100];
    cISBootloaderBase* ctx;
    bool done;
} thread_libusb_t;

void cISBootloaderThread::update_thread_serial(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context; 

    cISBootloaderBase* ctx_new;
    serial_port_t port;
    serialPortPlatformInit(&port);
    serial_thread_mutex.lock();
    const char* serial_name = thread_info->serial_name;
    serial_thread_mutex.unlock();

    serial_thread_mutex.lock();
    thread_info->reuse_port = false;
    serial_thread_mutex.unlock();

    serialPortSetPort(&port, serial_name);
    if (!serialPortOpenRetry(&port, serial_name, 921600, 1))
    {
        serialPortClose(&port);
        serial_thread_mutex.lock();
        thread_info->done = true;
        serial_thread_mutex.unlock();
        return;
    }

    is_operation_result result = cISBootloaderBase::update_device(m_firmware, &port, &ctx_new, m_infoProgress, m_uploadProgress, m_verifyProgress);

    if (result == IS_OP_OK)
    {   
        // Device is updated, add it to the ctx list so we can reset it later
        ctx_mutex.lock();
        ctx.push_back(ctx_new);
        ctx_new->m_port_name = string(thread_info->serial_name);
        ctx_new->m_finished_flash = true;
        ctx_new->m_update_in_progress = false;
        ctx_mutex.unlock();

        serial_thread_mutex.lock();
        thread_info->ctx = ctx_new;
        serial_thread_mutex.unlock();
    }
    else if(result == IS_OP_CLOSED)
    {
        // Device is resetting (may have updated if it was a SAMBA device)
        serial_thread_mutex.lock();
        thread_info->reuse_port = true;
        serial_thread_mutex.unlock();
    }
    else if (result == IS_OP_CANCELLED)
    {
        // Device has already been updated
    }
    
    serialPortClose(&port);

    SLEEP_MS(1000);

    serial_thread_mutex.lock();
    thread_info->done = true;
    serial_thread_mutex.unlock();
}

void cISBootloaderThread::update_thread_libusb(void* context)
{
    thread_libusb_t* thread_info = (thread_libusb_t*)context; 

    cISBootloaderBase* ctx_new;

    is_operation_result result = cISBootloaderBase::update_device(m_firmware, thread_info->handle, &ctx_new, m_infoProgress, m_uploadProgress, m_verifyProgress);

    if (result == IS_OP_OK)
    {   
        // Device is updated, add it to the ctx list so we can reset it later
        ctx_mutex.lock();
        ctx.push_back(ctx_new);
        ctx_new->m_finished_flash = true;
        ctx_new->m_update_in_progress = false;
        ctx_mutex.unlock();

        libusb_thread_mutex.lock();
        thread_info->ctx = ctx_new;
        libusb_thread_mutex.unlock();
    }
    else if(result == IS_OP_CLOSED)
    {
        // Device is resetting
    }
    else if (result == IS_OP_CANCELLED)
    {
        // Device has already been updated
    }

    SLEEP_MS(1000);

    libusb_thread_mutex.lock();
    thread_info->done = true;
    libusb_thread_mutex.unlock();
}

    

is_operation_result cISBootloaderThread::update(
    vector<string>& comPorts,   // ISB and SAM-BA and APP
    int                         baudRate,
    const firmwares_t& firmware,
    pfnBootloadProgress         uploadProgress,
    pfnBootloadProgress         verifyProgress,
    pfnBootloadStatus           infoProgress,
    void						(*waitAction)()
)
{
    m_update_in_progress = true;

    m_firmware = firmware;
    m_uploadProgress = uploadProgress;
    m_verifyProgress = verifyProgress;
    m_infoProgress = infoProgress;
    m_baudRate = baudRate;
    m_waitAction = waitAction;
    m_timeStart = current_timeMs();

    vector<string> ports;
    vector<thread_serial_t*> serial_threads;
    vector<thread_libusb_t*> libusb_threads;
    vector<string> ports_user_ignore;

    cISSerialPort::GetComPorts(ports);

    sort(ports.begin(), ports.end());
    sort(comPorts.begin(), comPorts.end());
    set_symmetric_difference(   // Get the difference between the specified ports and the ports present TODO: make this not symmetric
        ports.begin(), ports.end(),
        comPorts.begin(), comPorts.end(),
        back_inserter(ports_user_ignore));

    int waiter = 100;

    bool use_dfu = false;
    //if (libusb_init(NULL) >= 0) use_dfu = true;

    is_dfu_list dfu_list;

    cISBootloaderISB::reset_serial_list();
    cISBootloaderAPP::reset_serial_list();

    while (1)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        cISSerialPort::GetComPorts(ports);

        int devicesActive = 0;

        serial_thread_mutex.lock();

        for (size_t l = 0; l < serial_threads.size(); l++)
        {
            if (serial_threads[l]->thread != NULL && serial_threads[l]->done)
            {
                threadJoinAndFree(serial_threads[l]->thread);
                serial_threads[l]->thread = NULL;
            }

            if (!serial_threads[l]->done)
            {
                devicesActive++;
            }
        }

        for (size_t i = 0; i < ports.size(); i++)
        {
            bool found = false;

            for (size_t j = 0; j < serial_threads.size(); j++)
            {
                // Device is actively running or has finished in a state where we don't care to start a new thread for it
                if (string(serial_threads[j]->serial_name) == ports[i] && (serial_threads[j]->ctx != NULL || serial_threads[j]->thread != NULL))
                {
                    found = true;
                    break;
                }
                if (string(serial_threads[j]->serial_name) == ports[i] && serial_threads[j]->ctx == NULL && serial_threads[j]->done && !serial_threads[j]->reuse_port)
                {
                    found = true;
                    break;
                }
            }

            for (size_t k = 0; k < ports_user_ignore.size(); k++)
            {
                if (ports_user_ignore[k] == ports[i])
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                thread_serial_t* new_thread = (thread_serial_t*)malloc(sizeof(thread_serial_t));
                strncpy(new_thread->serial_name, ports[i].c_str(), 100);
                new_thread->ctx = NULL;
                new_thread->done = false;
                serial_threads.push_back(new_thread);
                serial_threads[serial_threads.size() - 1]->thread = threadCreateAndStart(update_thread_serial, serial_threads[serial_threads.size() - 1]);

                devicesActive++;
            }
        }

        serial_thread_mutex.unlock();

        if (use_dfu)
        {
            cISBootloaderDFU::list_devices(&dfu_list);

            libusb_thread_mutex.lock();

            for (size_t l = 0; l < libusb_threads.size(); l++)
            {
                if (libusb_threads[l]->thread != NULL && libusb_threads[l]->done)
                {
                    threadJoinAndFree(libusb_threads[l]->thread);
                    libusb_threads[l]->thread = NULL;
                }

                if (!libusb_threads[l]->done)
                {
                    devicesActive++;
                }
            }

            for (size_t i = 0; i < dfu_list.present; i++)
            {	// Create contexts for devices in DFU mode
                bool found = false;

                for (size_t j = 0; j < ctx.size(); j++)
                {
                    if (ctx[j]->match_test((void*)dfu_list.id[i].uid) == IS_OP_OK)
                    {   // We found the device in the context list
                        found = true;
                        break;
                    }
                }

                if (!found)
                {   // If we didn't find the device
                    thread_libusb_t* new_thread = (thread_libusb_t*)malloc(sizeof(thread_libusb_t));
                    new_thread->ctx = NULL;
                    new_thread->done = false;
                    new_thread->handle = dfu_list.id[i].handle_libusb;
                    libusb_threads.push_back(new_thread);
                    libusb_threads[libusb_threads.size() - 1]->thread = threadCreateAndStart(update_thread_libusb, libusb_threads[libusb_threads.size() - 1]);

                    devicesActive++;
                }
            }

            libusb_thread_mutex.unlock();
        }

        // Break after 1 second of no threads active
        if (devicesActive != 0) waiter = 100;
        else if (waiter-- < 0) break;
    }

    for (size_t i = 0; i < ctx.size(); i++)
    {
        serial_port_t port;
        serialPortPlatformInit(&port);
        if (!serialPortOpenRetry(&port, ctx[i]->m_port_name.c_str(), 921600, 1))
        {
            continue;
        }

        ctx[i]->m_port = &port;

        if (ctx[i]) ctx[i]->reboot_up();

        serialPortFlush(&port);
        serialPortClose(&port);
    }
    for (size_t i = 0; i < ctx.size(); i++)
    {
        delete ctx[i];
        ctx[i] = nullptr;
    }

    ctx.clear();

    if(use_dfu) { libusb_exit(NULL); }

    m_update_in_progress = false;

    return IS_OP_OK;
}
