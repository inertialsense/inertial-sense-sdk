/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

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
#include "protocol/FirmwareUpdate.h"

#include <algorithm>
#include <vector>

#if !PLATFORM_IS_WINDOWS
#include <unistd.h>
#endif

using namespace std;
using namespace ISBootloader;

vector<cISBootloaderBase*> cISBootloaderThread::ctx;
firmwares_t cISBootloaderThread::m_firmware;
fwUpdate::pfnProgressCb cISBootloaderThread::m_uploadProgress;
fwUpdate::pfnProgressCb cISBootloaderThread::m_verifyProgress;
fwUpdate::pfnStatusCb cISBootloaderThread::m_infoProgress;
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
map<std::string, cISBootloaderThread::thread_serial_t*> cISBootloaderThread::m_serial_threads;
vector<cISBootloaderThread::thread_libusb_t*> cISBootloaderThread::m_libusb_threads;

void cISBootloaderThread::mgmt_thread_libusb(void* context)
{
    (void)context;

    // Initialize libusb
    m_use_dfu = libusb_init(NULL) == LIBUSB_SUCCESS;

    is_dfu_list dfu_list;                       // List of libusb devices connected

    m_libusb_threads.clear();

    cISBootloaderDFU::m_DFUmutex.lock();

    m_libusb_thread_mutex.lock();
    cISBootloaderDFU::list_devices(&dfu_list);
    for (size_t i = 0; i < dfu_list.present; i++)
    {   // Create contexts for devices in DFU mode
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
            thread_libusb_t* new_thread = new thread_libusb_t();
            new_thread->ctx = NULL;
            new_thread->done = false;
            new_thread->handle = dfu_list.id[i].handle_libusb;
            m_libusb_threads.push_back(new_thread);
            m_libusb_threads[m_libusb_threads.size() - 1]->thread = threadCreateAndStart(update_thread_libusb, m_libusb_threads[m_libusb_threads.size() - 1], "isb-libusb");

            m_libusb_devicesActive++;
        }
    }
    m_libusb_thread_mutex.unlock();

    while (m_continue_update)
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

        m_libusb_thread_mutex.unlock();

        SLEEP_MS(100);
    }

    cISBootloaderDFU::m_DFUmutex.unlock();
    
    if (m_use_dfu) { libusb_exit(NULL); }
}

/**
 * Thread handler to validate and then enable a serial-device to enter APP mode (ie, boot to application firmware).
 * @param context the thread context, a thread_serial_t providing details about the device to query/configure
 */
void cISBootloaderThread::mode_thread_serial_app(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    port_handle_t port = (port_handle_t)&(thread_info->serialPort);
    if (!serialPortOpenRetry(port, portName(port), m_baudRate, 1))
    {
        serialPortClose(port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        thread_info->reuse_port = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    thread_info->opResult = cISBootloaderBase::mode_device_app(m_firmware, port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(port);
    serialPortClose(port);

    m_serial_thread_mutex.lock();
    thread_info->reuse_port = false;
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

/**
 * Thread handler to query a device (provided in the context) for its ISbootloader version
 * @param context the thread context, a thread_serial_t providing details about the device to query/configure
 */
void cISBootloaderThread::get_device_isb_version_thread(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    port_handle_t port = (port_handle_t)&(thread_info->serialPort);
    if (!serialPortOpenRetry(port, portName(port), m_baudRate, 1))
    {
        serialPortClose(port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        thread_info->reuse_port = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    thread_info->opResult = cISBootloaderBase::get_device_isb_version(m_firmware, port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(port);
    serialPortClose(port);

    m_serial_thread_mutex.lock();
    thread_info->reuse_port = true;
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

/**
 * Thread handler to validating and then enabling a serial-device to enter ISB mode.
 * @param context the thread context, a thread_serial_t providing details about the device to query/configure
 */
void cISBootloaderThread::mode_thread_serial_isb(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(IS_REBOOT_DELAY_MS);     // Wait for all other threads to start

    // attempt to open the target port; if unable to open, terminate this thread
    port_handle_t port = (port_handle_t)&(thread_info->serialPort);
    if (!serialPortOpenRetry(port, portName(port), m_baudRate, 1))
    {
        serialPortClose(port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        thread_info->reuse_port = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    // set device to ISB mode
    thread_info->opResult = cISBootloaderBase::mode_device_isb(m_firmware, thread_info->force_isb, port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(port);
    serialPortClose(port);

    m_serial_thread_mutex.lock();
    thread_info->reuse_port = true;
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

/**
 * Thread handler to perform a firmware update on a specific device.
 * @param context the thread context, a thread_serial_t providing details about the device to query/configure
 */
void cISBootloaderThread::update_thread_serial(void* context)
{
    thread_serial_t* thread_info = (thread_serial_t*)context; 
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    port_handle_t port = (port_handle_t)&(thread_info->serialPort);
    serialPortPlatformInit(port);
    m_serial_thread_mutex.lock();
    const char* serial_name = portName(port);
    thread_info->reuse_port = false;
    m_serial_thread_mutex.unlock();

    // Start at 115200 always, we will switch to user specified rate after we check for SAM-BA devices
    serialPortSetPort(port, serial_name);
    if (!serialPortOpenRetry(port, serial_name, BAUDRATE_115200, 1))
    {
        serialPortClose(port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    thread_info->opResult = cISBootloaderBase::update_device(m_firmware, port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context, m_baudRate);

    if (thread_info->opResult == IS_OP_OK)
    {   
        // Device is updated, add it to the ctx list so we can reset it later
        m_ctx_mutex.lock();
        new_context->m_port_name = std::string(portName(port));
        new_context->m_finished_flash = true;
        m_ctx_mutex.unlock();

        m_serial_thread_mutex.lock();
        thread_info->ctx = new_context;
        m_serial_thread_mutex.unlock();
    }
    else if (thread_info->opResult == IS_OP_CLOSED)
    {
        // Device is resetting (may have updated if it was a SAM-BA device)
        m_serial_thread_mutex.lock();
        thread_info->reuse_port = true;
        m_serial_thread_mutex.unlock();
    }
    else if (thread_info->opResult == IS_OP_CANCELLED)
    {
        // Device has already been updated
    }
    else // (IS_OP_ERROR usually)
    {
        // Other device
    }

    serialPortFlush(port);
    // serialPortClose(port);  DON'T CLOSE THE PORT - We may need it later to finalize/validate everything...

    m_serial_thread_mutex.lock();
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

void cISBootloaderThread::update_thread_libusb(void* context)
{
    thread_libusb_t* thread_info = (thread_libusb_t*)context; 
    cISBootloaderBase* new_context;

    thread_info->opResult = cISBootloaderBase::update_device(m_firmware, thread_info->handle, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    if (thread_info->opResult == IS_OP_OK)
    {   
        // Device is updated, add it to the ctx list so we can reset it later
        m_ctx_mutex.lock();
        new_context->m_finished_flash = true;
        m_ctx_mutex.unlock();

        m_libusb_thread_mutex.lock();
        thread_info->ctx = new_context;
        m_libusb_thread_mutex.unlock();
    }
    else if (thread_info->opResult == IS_OP_CLOSED)
    {
        // Device is resetting
    }
    else if (thread_info->opResult == IS_OP_CANCELLED)
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

bool cISBootloaderThread::true_if_cancelled(void)
{
    if(m_uploadProgress(std::any(), 0.0f, "", 0, 0) == IS_OP_CANCELLED)
    {
        m_continue_update = false;
        return true;
    }

    return false;
}

vector<cISBootloaderThread::confirm_bootload_t> cISBootloaderThread::set_mode_and_check_devices(
    vector<string>&                         comPorts,
    int                                     baudRate,
    const ISBootloader::firmwares_t&        firmware,
    fwUpdate::pfnProgressCb                 uploadProgress,
    fwUpdate::pfnProgressCb                 verifyProgress,
    fwUpdate::pfnStatusCb                   infoProgress,
    void                                    (*waitAction)()
)
{
    // Only allow one firmware update sequence to happen at a time
    m_update_mutex.lock();
    m_update_in_progress = true;

    // Clear old entries
    m_ctx_mutex.lock();
    ctx.clear();
    m_ctx_mutex.unlock();

    // Copy in the firmware update settings
    m_firmware = firmware;
    m_uploadProgress = uploadProgress;
    m_verifyProgress = verifyProgress;
    m_infoProgress = infoProgress;
    m_baudRate = baudRate;
    m_waitAction = waitAction;

    vector<string> portNames;                       // List of all ports connected, including ignored ports
    vector<string> ports_user_ignore;               // List of ports that were connected at startup but not selected. Will ignore in update.
    vector<confirm_bootload_t> updatesPending;

    m_serial_threads.clear();

    cISSerialPort::GetComPorts(portNames);

    // Get the list of ports to ignore during the bootloading process
    sort(portNames.begin(), portNames.end());
    sort(comPorts.begin(), comPorts.end());
    set_symmetric_difference(
            portNames.begin(), portNames.end(),
            comPorts.begin(), comPorts.end(),
            back_inserter(ports_user_ignore));

    m_continue_update = true;
    m_timeStart = current_timeMs();

    m_infoProgress(std::any(), IS_LOG_LEVEL_INFO, "Initializing devices for update...");

    ////////////////////////////////////////////////////////////////////////////
    // Run `mode_thread_serial_app` to put all APP devices into IS-bootloader mode
    ////////////////////////////////////////////////////////////////////////////

    // Put all devices in the correct mode
    m_infoProgress(std::any(), IS_LOG_LEVEL_INFO, "Waiting for devices to initialize...");
    while (m_continue_update && !true_if_cancelled())
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(100);

        cISSerialPort::GetComPorts(portNames);

        m_serial_thread_mutex.lock();

        for (auto port_name : portNames)
        {
            bool found = false;

            for (auto& [portName, serialThread] : m_serial_threads)
            {
                if (portName == port_name)
                {
                    if (!serialThread->done)    //(m_serial_threads[j]->ctx != NULL ||
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (serialThread->done && !serialThread->reuse_port)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
                }
            }

            std::for_each(ports_user_ignore.begin(), ports_user_ignore.end(),  [&port_name, &found] (std::string ignoredPort) {
                if (ignoredPort == port_name)
                    found = true;
            });

            if (!found)
            {
                thread_serial_t* new_thread = new thread_serial_t(port_name); // (thread_serial_t*)malloc(sizeof(thread_serial_t));
                if (new_thread->serialPort.errorCode == 0) {
                    m_serial_threads[port_name] = new_thread;
                    new_thread->thread = threadCreateAndStart(mode_thread_serial_app, m_serial_threads[port_name], "isb-serial-app");

                    m_infoProgress(std::any(), IS_LOG_LEVEL_DEBUG, "mode_thread_serial_app found viable port: %s", port_name.c_str());
                    m_serial_devicesActive++;
                }
            }
        }

        // Break after 5 seconds
        if (current_timeMs() - m_timeStart > 5000)
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
    }

    m_continue_update = true;
    m_timeStart = current_timeMs();

    SLEEP_MS(IS_REBOOT_DELAY_MS);

    ////////////////////////////////////////////////////////////////////////////
    // Join and free
    ////////////////////////////////////////////////////////////////////////////
    
    // Join and free all mode threads
    while (m_continue_update)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        m_continue_update = false;

        m_serial_thread_mutex.lock();

        for (auto& [portName, serialThread] : m_serial_threads)
        {
            if (!serialThread->done)
            {
                m_continue_update = true;
            }
            else if (serialThread->thread != NULL)
            {
                threadJoinAndFree(serialThread->thread);
                serialThread->thread = NULL;
                delete serialThread;
                m_serial_threads[portName] = NULL;
            }
        }

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 5000) 
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
        m_serial_threads.clear();
    }

    if(m_uploadProgress(std::any(), 0.0f, "", 0, 0) == IS_OP_CANCELLED)
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if (m_waitAction) m_waitAction();
        return vector<confirm_bootload_t>(); 
    }

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `get_device_isb_version_thread` to get version from ISB bootloaders
    ////////////////////////////////////////////////////////////////////////////

    // Put all devices in the correct mode
    while (m_continue_update && !true_if_cancelled())
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        cISSerialPort::GetComPorts(portNames);

        m_serial_thread_mutex.lock();

        for (auto port_name : portNames)
        {
            bool found = false;
            for (auto& [portName, serialThread] : m_serial_threads)
            {
                if (portName == port_name)
                {
                    if (!serialThread->done)    //(m_serial_threads[j]->ctx != NULL ||
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (serialThread->done && !serialThread->reuse_port)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
                }
               
            }

            std::for_each(ports_user_ignore.begin(), ports_user_ignore.end(),  [&port_name, &found] (std::string ignoredPort) {
                if (ignoredPort == port_name)
                    found = true;
            });


            if (!found)
            {
                m_infoProgress(std::any(), IS_LOG_LEVEL_DEBUG, "get_device_isb_version_thread found viable port: %s", port_name.c_str());
                thread_serial_t* new_thread = new thread_serial_t(port_name); // (thread_serial_t*)malloc(sizeof(thread_serial_t));
                m_serial_threads[port_name] = new_thread;
                new_thread->thread = threadCreateAndStart(get_device_isb_version_thread, new_thread, "isb-dev-version");

                m_serial_devicesActive++;
            }
        }

        // Break after 3 seconds
        if (current_timeMs() - m_timeStart > 3000)
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
    }

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Join threads
    ////////////////////////////////////////////////////////////////////////////
    
    // Join and free all mode threads
    while (m_continue_update)
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        m_continue_update = false;

        m_serial_thread_mutex.lock();

        for (auto& [portName, serialThread] : m_serial_threads)
        {
            if (!serialThread->done)
            {
                m_continue_update = true;
            }
            else if (serialThread->thread != NULL)
            {
                threadJoinAndFree(serialThread->thread);
                serialThread->thread = NULL;
                serialPortClose((port_handle_t)&serialThread->serialPort);
                delete serialThread;
                m_serial_threads[portName] = NULL;
            }
        }

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 3000)
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
        m_serial_threads.clear();
    }

    if(m_uploadProgress(std::any(), 0.0f, "Waiting for device response.", 0, 0) == IS_OP_CANCELLED)
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if (m_waitAction) m_waitAction();
        return vector<confirm_bootload_t>(); 
    }

    m_ctx_mutex.lock();
    for (auto& cur_ctx : ctx)
    {
        if (cur_ctx->isb_mightUpdate)
        {
            confirm_bootload_t confirm;
            confirm.major = cur_ctx->m_isb_major;
            confirm.minor = cur_ctx->m_isb_minor;
            confirm.sn = cur_ctx->m_sn;
            confirm.port = cur_ctx->m_port;

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
    fwUpdate::pfnProgressCb         uploadProgress,
    fwUpdate::pfnProgressCb         verifyProgress,
    fwUpdate::pfnStatusCb           infoProgress,
    void                        (*waitAction)()
)
{
    string tmp;
    uint32_t timeDeltaMs; 
    uint32_t beginTimeMs;
    uint32_t timeout;

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

    vector<string> portNames;                       // List of ports connected
    vector<string> ports_user_ignore;           // List of ports that were connected at startup but not selected. Will ignore in update.

    m_serial_threads.clear();

    cISSerialPort::GetComPorts(portNames);

    // Get the list of ports to ignore during the bootloading process
    sort(portNames.begin(), portNames.end());
    sort(comPorts.begin(), comPorts.end());
    set_symmetric_difference(
            portNames.begin(), portNames.end(),
            comPorts.begin(), comPorts.end(),
            back_inserter(ports_user_ignore));

    if(m_uploadProgress(std::any(), 0.0f, "Writing Flash", 0, 0) == IS_OP_CANCELLED)
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if (m_waitAction) m_waitAction();
        return IS_OP_CANCELLED; 
    }
    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `mode_thread_serial_isb` to put all ISB devices into ROM-bootloader (DFU/SAM-BA) mode if necessary
    ////////////////////////////////////////////////////////////////////////////

    while (m_continue_update && !true_if_cancelled())
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(1000);

        cISSerialPort::GetComPorts(portNames);

        m_serial_thread_mutex.lock();

        for (auto port_name : portNames)
        {
            bool found = false;

            for (auto& [portName, serialThread] : m_serial_threads)
            {
                if (portName == port_name)
                {
                    found = true;
                    break;
                }
            }

            std::for_each(ports_user_ignore.begin(), ports_user_ignore.end(),  [&port_name, &found] (std::string ignoredPort) {
                if (ignoredPort == port_name)
                    found = true;
            });

            if (!found)
            {
                m_infoProgress(std::any(), IS_LOG_LEVEL_DEBUG, "mode_thread_serial_isb found viable port: %s", port_name.c_str());
                thread_serial_t* new_thread = new thread_serial_t(port_name, force_isb_update); // (thread_serial_t*)malloc(sizeof(thread_serial_t));
                m_serial_threads[port_name] = new_thread;
                new_thread->thread = threadCreateAndStart(mode_thread_serial_isb, new_thread, "isb-serial-isb");

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
    
    while (m_continue_update)
    {
        m_continue_update = false;

        m_serial_thread_mutex.lock();

        for (auto& [portName, serialThread] : m_serial_threads)
        {
            if (!serialThread->done)
            {
                m_continue_update = true;
            }
            else if (serialThread->thread != NULL)
            {
                threadJoinAndFree(serialThread->thread);
                serialThread->thread = NULL;
            }
        }

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 5000)
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
    }

    if(m_uploadProgress(std::any(), 0.0f, "Writing Flash", 0, 0) == IS_OP_CANCELLED)
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if (m_waitAction) m_waitAction();
        return IS_OP_CANCELLED; 
    }
    m_infoProgress(std::any(), IS_LOG_LEVEL_INFO, "Updating...");

    ////////////////////////////////////////////////////////////////////////////
    // Run `mgmt_thread_libusb` to update DFU devices
    ////////////////////////////////////////////////////////////////////////////

    m_libusb_devicesActive = 0;

    void* libusb_thread = threadCreateAndStart(mgmt_thread_libusb, NULL, "isb-update-libusb");

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `update_thread_serial` to update devices
    ////////////////////////////////////////////////////////////////////////////

    beginTimeMs = current_timeMs();

    while (m_continue_update && !true_if_cancelled())
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        m_serial_devicesActive = 0;

        cISSerialPort::GetComPorts(portNames);

        m_serial_thread_mutex.lock();

        for (auto& [portName, serialThread] : m_serial_threads)
        {
            if (serialThread->thread != NULL && serialThread->done)
            {
                threadJoinAndFree(serialThread->thread);
                serialThread->thread = NULL;
            }

            if (!serialThread->done)
            {
                m_serial_devicesActive++;
            }
        }

        for (auto port_name : portNames)
        {
            bool found = false;

            for (auto& [portName, serialThread] : m_serial_threads)
            {
                if (portName == port_name)
                {
                    if (!serialThread->done)    //(m_serial_threads[j]->ctx != NULL ||
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (serialThread->done && !serialThread->reuse_port)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
                }
            }

            std::for_each(ports_user_ignore.begin(), ports_user_ignore.end(),  [&port_name, &found] (std::string ignoredPort) {
                if (ignoredPort == port_name)
                    found = true;
            });


            if (!found)
            {
                m_infoProgress(std::any(), IS_LOG_LEVEL_DEBUG, "update_thread_serial found viable port: %s", port_name.c_str());
                thread_serial_t* new_thread = new thread_serial_t(port_name, force_isb_update); // (thread_serial_t*)malloc(sizeof(thread_serial_t));
                m_serial_threads[port_name] = new_thread;
                new_thread->thread = threadCreateAndStart(update_thread_serial, new_thread, "isb-update-serial");

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

        // Timeout after 180 seconds
        timeout = (baudRate < 921600) ? 360000 : 230000;
        timeDeltaMs = current_timeMs() - beginTimeMs;

        if (timeDeltaMs > timeout)
        {
            m_continue_update = false;

            tmp = "\nUpdate timeout... Timeout of " + to_string(((double)timeout) / 1000) + " Seconds reached.";

            m_infoProgress(std::any(), IS_LOG_LEVEL_ERROR, tmp.c_str());
        }
    }

    timeDeltaMs = current_timeMs() - beginTimeMs;

    tmp = "\nUpdate run time: " + to_string(((double)timeDeltaMs) / 1000) + " Seconds.";

    m_infoProgress(std::any(), IS_LOG_LEVEL_INFO, tmp.c_str());

    threadJoinAndFree(libusb_thread);

    if(m_uploadProgress(std::any(), 0.0f, "", 0, 0) == IS_OP_CANCELLED)
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if (m_waitAction) m_waitAction();
        return IS_OP_CANCELLED; 
    }
    
    // Reset all serial devices up a level into APP or IS-bootloader mode
    // At this point, its likely that all ports will be closed at completion of the update process, so we need to reopen and then issue reboot_up()
    m_serial_thread_mutex.lock();
    for (auto& [portName, serialThread] : m_serial_threads)
    {
        if (serialThread && serialThread->done)
        {
            if (serialThread->ctx)
                serialThread->ctx->reboot_up();

            port_handle_t port = (port_handle_t)&serialThread->serialPort;
            if (serialPortIsOpenQuick(port)) {
                serialPortFlush(port);
                serialPortClose(port);
            }
        }
    }

    // Clear the ctx list
    for (auto& cur_ctx : ctx)
    {
        delete cur_ctx;
        cur_ctx = nullptr;
    }
    ctx.clear();

    m_update_in_progress = false;
    m_update_mutex.unlock();

    if (m_waitAction) m_waitAction();     // Final UI update

    return IS_OP_OK;
}
