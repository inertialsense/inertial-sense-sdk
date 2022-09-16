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
mutex cISBootloaderThread::m_ctx_mutex;
mutex cISBootloaderThread::m_serial_thread_mutex;
mutex cISBootloaderThread::m_libusb_thread_mutex;
bool cISBootloaderThread::m_update_in_progress = false;
mutex cISBootloaderThread::m_update_mutex;
bool cISBootloaderThread::m_use_dfu;
uint32_t cISBootloaderThread::m_libusb_devicesActive;
uint32_t cISBootloaderThread::m_serial_devicesActive;
bool cISBootloaderThread::m_continue_update;
vector<cISBootloaderThread::thread_serial_t*> cISBootloaderThread::m_serial_threads;
vector<cISBootloaderThread::thread_libusb_t*> cISBootloaderThread::m_libusb_threads;

void cISBootloaderThread::mgmt_thread_libusb(void* context)
{
    (void)context;

    // Initialize libusb
    m_use_dfu = libusb_init(NULL) == LIBUSB_SUCCESS;

    is_dfu_list dfu_list;                       // List of libusb devices connected

    m_libusb_threads.clear();

    cISBootloaderDFU::m_DFUmutex.lock();

    while(m_continue_update)
    {
        m_libusb_thread_mutex.lock();

        m_libusb_devicesActive = 0;

        for (size_t l = 0; l < m_libusb_threads.size(); l++)
        {
            if (m_libusb_threads[l]->thread != NULL && m_libusb_threads[l]->done)
            {
                threadJoinAndFree(m_libusb_threads[l]->thread);
                m_libusb_threads[l]->thread = NULL;
                libusb_close(m_libusb_threads[l]->handle);
            }

            if (!m_libusb_threads[l]->done)
            {
                m_libusb_devicesActive++;
            }
        }

        cISBootloaderDFU::list_devices(&dfu_list);  // TODO: Put this in a separate thread since it takes a long time

        for (size_t i = 0; i < dfu_list.present; i++)
        {	// Create contexts for devices in DFU mode
            bool found = false;

            for (size_t j = 0; j < ctx.size(); j++)
            {
                m_ctx_mutex.lock();
                if (!(ctx[j]->is_serial_device()) && ctx[j]->match_test((void*)dfu_list.id[i].uid) == IS_OP_OK)
                {   // We found the device in the context list
                    found = true;
                    break;
                }
                m_ctx_mutex.unlock();
            }

            if (!found)
            {   // If we didn't find the device
                thread_libusb_t* new_thread = (thread_libusb_t*)malloc(sizeof(thread_libusb_t));
                new_thread->ctx = NULL;
                new_thread->done = false;
                new_thread->handle = dfu_list.id[i].handle_libusb;
                m_libusb_threads.push_back(new_thread);
                m_libusb_threads[m_libusb_threads.size() - 1]->thread = threadCreateAndStart(update_thread_libusb, m_libusb_threads[m_libusb_threads.size() - 1]);

                m_libusb_devicesActive++;
            }
        }

        m_libusb_thread_mutex.unlock();

        SLEEP_MS(100);
    }

    cISBootloaderDFU::m_DFUmutex.unlock();
    
    if(m_use_dfu) { libusb_exit(NULL); }
}

void cISBootloaderThread::mode_thread_serial_app(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    serial_port_t port;
    serialPortPlatformInit(&port);
    m_serial_thread_mutex.lock();
    const char* serial_name = thread_info->serial_name;
    m_serial_thread_mutex.unlock();

    serialPortSetPort(&port, serial_name);
    if (!serialPortOpenRetry(&port, serial_name, m_baudRate, 1))
    {
        serialPortClose(&port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        thread_info->reuse_port = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    cISBootloaderBase::mode_device_app(m_firmware, &port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(&port);
    serialPortClose(&port);

    m_serial_thread_mutex.lock();
    thread_info->reuse_port = true;
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

void cISBootloaderThread::get_device_isb_version_thread(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    serial_port_t port;
    serialPortPlatformInit(&port);
    m_serial_thread_mutex.lock();
    const char* serial_name = thread_info->serial_name;
    m_serial_thread_mutex.unlock();

    serialPortSetPort(&port, serial_name);
    if (!serialPortOpenRetry(&port, serial_name, m_baudRate, 1))
    {
        serialPortClose(&port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        thread_info->reuse_port = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    cISBootloaderBase::get_device_isb_version(m_firmware, &port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(&port);
    serialPortClose(&port);

    m_serial_thread_mutex.lock();
    thread_info->reuse_port = true;
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

void cISBootloaderThread::mode_thread_serial_isb(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(2500);     // Wait for all other threads to start

    serial_port_t port;
    serialPortPlatformInit(&port);
    m_serial_thread_mutex.lock();
    const char* serial_name = thread_info->serial_name;
    m_serial_thread_mutex.unlock();

    serialPortSetPort(&port, serial_name);
    if (!serialPortOpenRetry(&port, serial_name, m_baudRate, 1))
    {
        serialPortClose(&port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        thread_info->reuse_port = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    cISBootloaderBase::mode_device_isb(m_firmware, thread_info->force_isb, &port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(&port);
    serialPortClose(&port);

    m_serial_thread_mutex.lock();
    thread_info->reuse_port = true;
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

void cISBootloaderThread::update_thread_serial(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context; 
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    serial_port_t port;
    serialPortPlatformInit(&port);
    m_serial_thread_mutex.lock();
    const char* serial_name = thread_info->serial_name;
    thread_info->reuse_port = false;
    m_serial_thread_mutex.unlock();

    serialPortSetPort(&port, serial_name);
    if (!serialPortOpenRetry(&port, serial_name, m_baudRate, 1))
    {
        serialPortClose(&port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    is_operation_result result = cISBootloaderBase::update_device(m_firmware, &port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    if (result == IS_OP_OK)
    {   
        // Device is updated, add it to the ctx list so we can reset it later
        m_ctx_mutex.lock();
        new_context->m_port_name = string(thread_info->serial_name);
        new_context->m_finished_flash = true;
        m_ctx_mutex.unlock();

        m_serial_thread_mutex.lock();
        thread_info->ctx = new_context;
        m_serial_thread_mutex.unlock();
    }
    else if(result == IS_OP_CLOSED)
    {
        // Device is resetting (may have updated if it was a SAMBA device)
        m_serial_thread_mutex.lock();
        thread_info->reuse_port = true;
        m_serial_thread_mutex.unlock();
    }
    else if (result == IS_OP_CANCELLED)
    {
        // Device has already been updated
    }
    else // (IS_OP_ERROR usually)
    {
        // Other device
    }

    serialPortFlush(&port);
    serialPortClose(&port);

    m_serial_thread_mutex.lock();
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

void cISBootloaderThread::update_thread_libusb(void* context)
{
    thread_libusb_t* thread_info = (thread_libusb_t*)context; 
    cISBootloaderBase* new_context;

    is_operation_result result = cISBootloaderBase::update_device(m_firmware, thread_info->handle, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    if (result == IS_OP_OK)
    {   
        // Device is updated, add it to the ctx list so we can reset it later
        m_ctx_mutex.lock();
        new_context->m_finished_flash = true;
        m_ctx_mutex.unlock();

        m_libusb_thread_mutex.lock();
        thread_info->ctx = new_context;
        m_libusb_thread_mutex.unlock();
    }
    else if(result == IS_OP_CLOSED)
    {
        // Device is resetting
    }
    else if (result == IS_OP_CANCELLED)
    {
        // Device has already been updated
    }
    else
    {

    }

    m_libusb_thread_mutex.lock();
    thread_info->done = true;
    m_libusb_thread_mutex.unlock();
}

vector<cISBootloaderThread::confirm_bootload_t> cISBootloaderThread::set_mode_and_check_devices(
    vector<string>&               comPorts,
    int                                     baudRate,
    const ISBootloader::firmwares_t&        firmware,
    ISBootloader::pfnBootloadProgress       uploadProgress, 
    ISBootloader::pfnBootloadProgress       verifyProgress,
    ISBootloader::pfnBootloadStatus         infoProgress,
    void						            (*waitAction)()
)
{
    // Only allow one firmware update sequence to happen at a time
    m_update_mutex.lock();
    m_update_in_progress = true;
    
    // Copy in the firmware update settings
    m_firmware = firmware;
    m_uploadProgress = uploadProgress;
    m_verifyProgress = verifyProgress;
    m_infoProgress = infoProgress;
    m_baudRate = baudRate;
    m_waitAction = waitAction;

    vector<string> ports;                       // List of ports connected
    vector<string> ports_user_ignore;           // List of ports that were connected at startup but not selected. Will ignore in update.

    m_serial_threads.clear();

    cISSerialPort::GetComPorts(ports);

    // Get the list of ports to ignore during the bootloading process
    sort(ports.begin(), ports.end());
    sort(comPorts.begin(), comPorts.end());
    set_symmetric_difference(   
        ports.begin(), ports.end(),
        comPorts.begin(), comPorts.end(),
        back_inserter(ports_user_ignore));

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `mode_thread_serial_app` to put all APP devices into ISB mode
    ////////////////////////////////////////////////////////////////////////////

    m_infoProgress(NULL, "Resetting devices into ISB mode... (5 seconds)", IS_LOG_LEVEL_INFO);

    // Put all devices in the correct mode
    while(m_continue_update)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        cISSerialPort::GetComPorts(ports);

        m_serial_thread_mutex.lock();

        for (size_t i = 0; i < ports.size(); i++)
        {
            bool found = false;

            for (size_t j = 0; j < m_serial_threads.size(); j++)
            {
                if (string(m_serial_threads[j]->serial_name) == ports[i])
                {
                    if (!m_serial_threads[j]->done)    //(m_serial_threads[j]->ctx != NULL || 
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (m_serial_threads[j]->done && !m_serial_threads[j]->reuse_port)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
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
                m_serial_threads.push_back(new_thread);
                m_serial_threads[m_serial_threads.size() - 1]->thread = threadCreateAndStart(mode_thread_serial_app, m_serial_threads[m_serial_threads.size() - 1]);

                m_serial_devicesActive++;
            }
        }

        m_serial_thread_mutex.unlock();

        // Break after 5 seconds
        if (current_timeMs() - m_timeStart > 5000) 
        {
            m_continue_update = false;
        }
    }

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Join and free
    ////////////////////////////////////////////////////////////////////////////

    m_infoProgress(NULL, "Waiting for devices to re-enumerate... (5 seconds max.)", IS_LOG_LEVEL_INFO);
    
    // Join and free all mode threads
    while (m_continue_update)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        m_continue_update = false;

        m_serial_thread_mutex.lock();

        for (size_t l = 0; l < m_serial_threads.size(); l++)
        {
            if(!m_serial_threads[l]->done)
            {
                m_continue_update = true;
            }
            else if (m_serial_threads[l]->thread != NULL)
            {
                threadJoinAndFree(m_serial_threads[l]->thread);
                m_serial_threads[l]->thread = NULL;
            }
        }

        SLEEP_MS(100);

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 5000) 
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
    }

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `get_device_isb_version_thread` to get version from ISB bootloaders
    ////////////////////////////////////////////////////////////////////////////

    m_infoProgress(NULL, "Finding devices in ISB mode... (3 seconds)", IS_LOG_LEVEL_INFO);

    // Put all devices in the correct mode
    while(m_continue_update)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        cISSerialPort::GetComPorts(ports);

        m_serial_thread_mutex.lock();

        for (size_t i = 0; i < ports.size(); i++)
        {
            bool found = false;

            for (size_t j = 0; j < m_serial_threads.size(); j++)
            {
                if (string(m_serial_threads[j]->serial_name) == ports[i])
                {
                    if (!m_serial_threads[j]->done)    //(m_serial_threads[j]->ctx != NULL || 
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (m_serial_threads[j]->done && !m_serial_threads[j]->reuse_port)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
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
                m_serial_threads.push_back(new_thread);
                m_serial_threads[m_serial_threads.size() - 1]->thread = threadCreateAndStart(get_device_isb_version_thread, m_serial_threads[m_serial_threads.size() - 1]);

                m_serial_devicesActive++;
            }
        }

        m_serial_thread_mutex.unlock();

        // Break after 5 seconds
        if (current_timeMs() - m_timeStart > 3000) 
        {
            m_continue_update = false;
        }
    }

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Join threads
    ////////////////////////////////////////////////////////////////////////////

    m_infoProgress(NULL, "Waiting for devices to re-enumerate... (5 seconds max.)", IS_LOG_LEVEL_INFO);
    
    // Join and free all mode threads
    while (m_continue_update)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        m_continue_update = false;

        m_serial_thread_mutex.lock();

        for (size_t l = 0; l < m_serial_threads.size(); l++)
        {
            if(!m_serial_threads[l]->done)
            {
                m_continue_update = true;
            }
            else if (m_serial_threads[l]->thread != NULL)
            {
                threadJoinAndFree(m_serial_threads[l]->thread);
                m_serial_threads[l]->thread = NULL;
            }
        }

        SLEEP_MS(100);

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 5000) 
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
    }

    vector<confirm_bootload_t> updatesPending;

    m_ctx_mutex.lock();

    for(size_t i = 0; i < ctx.size(); i++)
    {
        if(ctx[i]->isb_mightUpdate)
        {
            confirm_bootload_t confirm;
            confirm.major = ctx[i]->m_isb_major;
            confirm.minor = ctx[i]->m_isb_minor;
            confirm.sn = ctx[i]->m_sn;

            updatesPending.push_back(confirm);
        }
    }

    m_ctx_mutex.unlock();

    m_update_mutex.unlock();

    return updatesPending;
}

is_operation_result cISBootloaderThread::update(
    vector<string>&             comPorts,   // ISB and SAM-BA and APP
    bool                        force_isb_update,
    int                         baudRate,
    const firmwares_t&          firmware,
    pfnBootloadProgress         uploadProgress,
    pfnBootloadProgress         verifyProgress,
    pfnBootloadStatus           infoProgress,
    void						(*waitAction)()
)
{
    // Only allow one firmware update sequence to happen at a time
    m_update_mutex.lock();
    m_update_in_progress = true;
    
    // Copy in the firmware update settings
    m_firmware = firmware;
    m_uploadProgress = uploadProgress;
    m_verifyProgress = verifyProgress;
    m_infoProgress = infoProgress;
    m_baudRate = baudRate;
    m_waitAction = waitAction;

    vector<string> ports;                       // List of ports connected
    vector<string> ports_user_ignore;           // List of ports that were connected at startup but not selected. Will ignore in update.

    m_serial_threads.clear();

    cISSerialPort::GetComPorts(ports);

    // Get the list of ports to ignore during the bootloading process
    sort(ports.begin(), ports.end());
    sort(comPorts.begin(), comPorts.end());
    set_symmetric_difference(   
        ports.begin(), ports.end(),
        comPorts.begin(), comPorts.end(),
        back_inserter(ports_user_ignore));

    m_continue_update = true;
    m_timeStart = current_timeMs();

    m_infoProgress(NULL, "Resetting devices into ISB mode... (5 seconds)", IS_LOG_LEVEL_INFO);

    ////////////////////////////////////////////////////////////////////////////
    // Run `mode_thread_serial_isb` to put all ISB devices into DFU/SAMBA mode
    // 
    // Only happens if 
    ////////////////////////////////////////////////////////////////////////////

    while(m_continue_update)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        cISSerialPort::GetComPorts(ports);

        m_serial_thread_mutex.lock();

        for (size_t i = 0; i < ports.size(); i++)
        {
            bool found = false;

            for (size_t j = 0; j < m_serial_threads.size(); j++)
            {
                if (string(m_serial_threads[j]->serial_name) == ports[i])
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
                new_thread->force_isb = force_isb_update;
                m_serial_threads.push_back(new_thread);
                m_serial_threads[m_serial_threads.size() - 1]->thread = threadCreateAndStart(mode_thread_serial_isb, m_serial_threads[m_serial_threads.size() - 1]);

                m_serial_devicesActive++;
            }
        }

        m_serial_thread_mutex.unlock();

        // Break after 5 seconds
        if (current_timeMs() - m_timeStart > 5000) 
        {
            m_continue_update = false;
        }
    }

    m_continue_update = true;
    m_timeStart = current_timeMs();

    m_infoProgress(NULL, "Waiting for devices to re-enumerate... (5 seconds max.)", IS_LOG_LEVEL_INFO);
    
    ////////////////////////////////////////////////////////////////////////////
    // Join and free 
    ////////////////////////////////////////////////////////////////////////////
    
    while (m_continue_update)
    {
        m_continue_update = false;

        m_serial_thread_mutex.lock();

        for (size_t l = 0; l < m_serial_threads.size(); l++)
        {
            if(!m_serial_threads[l]->done)
            {
                m_continue_update = true;
            }
            else if (m_serial_threads[l]->thread != NULL)
            {
                threadJoinAndFree(m_serial_threads[l]->thread);
                m_serial_threads[l]->thread = NULL;
            }
        }

        SLEEP_MS(100);

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 5000) 
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Run `mgmt_thread_libusb` to update DFU devices
    ////////////////////////////////////////////////////////////////////////////

    m_libusb_devicesActive = 0;

    void* libusb_thread = threadCreateAndStart(mgmt_thread_libusb, NULL);

    m_continue_update = true;
    m_timeStart = current_timeMs();
    uint32_t timeoutLong = current_timeMs();

    m_infoProgress(NULL, "Updating devices... (60 seconds max.)", IS_LOG_LEVEL_INFO);

    ////////////////////////////////////////////////////////////////////////////
    // Run `update_thread_serial` to update devices
    ////////////////////////////////////////////////////////////////////////////

    while (m_continue_update)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        m_serial_devicesActive = 0;

        cISSerialPort::GetComPorts(ports);

        m_serial_thread_mutex.lock();

        for (size_t l = 0; l < m_serial_threads.size(); l++)
        {
            if (m_serial_threads[l]->thread != NULL && m_serial_threads[l]->done)
            {
                threadJoinAndFree(m_serial_threads[l]->thread);
                m_serial_threads[l]->thread = NULL;
            }

            if (!m_serial_threads[l]->done)
            {
                m_serial_devicesActive++;
            }
        }

        for (size_t i = 0; i < ports.size(); i++)
        {
            bool found = false;

            for (size_t j = 0; j < m_serial_threads.size(); j++)
            {
                if (string(m_serial_threads[j]->serial_name) == ports[i])
                {
                    if (!m_serial_threads[j]->done)    //(m_serial_threads[j]->ctx != NULL || 
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (m_serial_threads[j]->done && !m_serial_threads[j]->reuse_port)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
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
                new_thread->force_isb = force_isb_update;
                m_serial_threads.push_back(new_thread);
                m_serial_threads[m_serial_threads.size() - 1]->thread = threadCreateAndStart(update_thread_serial, m_serial_threads[m_serial_threads.size() - 1]);

                m_serial_devicesActive++;
            }
        }

        m_libusb_thread_mutex.lock();

        // Break after 3 seconds of no threads active
        if (m_libusb_devicesActive != 0 || m_serial_devicesActive != 0) 
        {
            m_timeStart = current_timeMs();
        }
        else if (current_timeMs() - m_timeStart > 3000) 
        {
            m_continue_update = false;
        }

        m_libusb_thread_mutex.unlock();
        m_serial_thread_mutex.unlock();

        // Timeout after 60 seconds
        if (current_timeMs() - timeoutLong > 60000) 
        {
            m_continue_update = false;
        }
    }

    threadJoinAndFree(libusb_thread);

    m_infoProgress(NULL, "Resetting devices into final mode...", IS_LOG_LEVEL_INFO);
    
    // Reset all serial devices up a level into APP or ISB mode
    for (size_t i = 0; i < ctx.size(); i++)
    {
        if(!ctx[i]->m_port_name.empty())
        {
            serial_port_t port;
            serialPortPlatformInit(&port);
            if (!serialPortOpenRetry(&port, ctx[i]->m_port_name.c_str(), 921600, 1))
            {
                continue;
            }

            ctx[i]->m_port = &port;

            if (ctx[i] && ctx[i]->m_finished_flash) ctx[i]->reboot_up();

            serialPortFlush(&port);
            serialPortClose(&port);
        }

        m_waitAction();
    }
    
    // Clear the ctx list
    for (size_t i = 0; i < ctx.size(); i++)
    {
        delete ctx[i];
        ctx[i] = nullptr;
    }
    ctx.clear();

    m_update_in_progress = false;
    m_update_mutex.unlock();

    m_infoProgress(NULL, "Done!", IS_LOG_LEVEL_INFO);

    m_waitAction();     // Final UI update

    return IS_OP_OK;
}
