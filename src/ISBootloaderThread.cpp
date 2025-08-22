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
#include "intel_hex_utils.h"

#include <algorithm>

#if !PLATFORM_IS_WINDOWS
#include <unistd.h>
#endif

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

    m_libusb_thread_mutex.lock();
    cISBootloaderDFU::list_devices(&dfu_list);
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
            create_and_start_libusb_thread(update_thread_libusb, dfu_list.id[i].handle_libusb);
        }
    }
    m_libusb_thread_mutex.unlock();

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

    is_operation_result result = cISBootloaderBase::mode_device_app(m_firmware, &port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(&port);
    serialPortClose(&port);

    m_serial_thread_mutex.lock();
    thread_info->result = result;
    thread_info->reuse_port = false;
    thread_info->done = true;
    m_serial_thread_mutex.unlock();
}

void cISBootloaderThread::create_and_start_serial_thread(const string& port, void(*function)(void*), bool force_isb_update)
{
    thread_serial_t* new_thread = (thread_serial_t*)malloc(sizeof(thread_serial_t));
    memset(new_thread->serial_name, 0, 100);
    strncpy(new_thread->serial_name, port.c_str(), _MIN(port.size(), 100));
    new_thread->ctx = NULL;
    new_thread->done = false;
    new_thread->result = IS_OP_OK;
    new_thread->force_isb = force_isb_update;
    m_serial_threads.push_back(new_thread);
    new_thread->thread = threadCreateAndStart(function, new_thread);
    m_serial_devicesActive++;
}

void cISBootloaderThread::create_and_start_libusb_thread(void(*function)(void*), libusb_device_handle* handle)
{
    thread_libusb_t* new_thread = (thread_libusb_t*)malloc(sizeof(thread_libusb_t));
    new_thread->ctx = NULL;
    new_thread->done = false;
    new_thread->handle = handle;
    new_thread->result = IS_OP_OK;
    m_libusb_threads.push_back(new_thread);
    new_thread->thread = threadCreateAndStart(function, new_thread);
    m_libusb_devicesActive++;
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

    is_operation_result result = cISBootloaderBase::get_device_isb_version(m_firmware, &port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(&port);
    serialPortClose(&port);

    m_serial_thread_mutex.lock();
    thread_info->result = result;
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

    is_operation_result result = cISBootloaderBase::mode_device_isb(m_firmware, thread_info->force_isb, &port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    serialPortFlush(&port);
    serialPortClose(&port);

    m_serial_thread_mutex.lock();
    thread_info->result = result;
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

    // Start at 115200 always, we will switch to user specified rate after we check for SAM-BA devices
    serialPortSetPort(&port, serial_name);
    if (!serialPortOpenRetry(&port, serial_name, BAUDRATE_115200, 1))
    {
        serialPortClose(&port);
        m_serial_thread_mutex.lock();
        thread_info->done = true;
        m_serial_thread_mutex.unlock();
        return;
    }

    is_operation_result result = cISBootloaderBase::update_device(m_firmware, &port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context, m_baudRate);

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
        // Device is resetting (may have updated if it was a SAM-BA device)
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

    SLEEP_MS(1000);

    serialPortFlush(&port);
    serialPortClose(&port);

    m_serial_thread_mutex.lock();
    thread_info->result = result;
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
    thread_info->result = result;
    thread_info->done = true;
    m_libusb_thread_mutex.unlock();
}

bool cISBootloaderThread::true_if_cancelled(void)
{
    if(m_uploadProgress(NULL, 0.0f) == IS_OP_CANCELLED)
    {
        m_continue_update = false;
        return true;
    }

    return false;
}

bool cISBootloaderThread::set_mode_and_check_devices(
    vector<string>&                         comPorts,
    int                                     baudRate,
    const ISBootloader::firmwares_t&        firmware,
    ISBootloader::pfnBootloadProgress       uploadProgress, 
    ISBootloader::pfnBootloadProgress       verifyProgress,
    ISBootloader::pfnBootloadStatus         infoProgress,
    void						            (*waitAction)(),
    vector<confirm_bootload_t>*             updatesPending
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

    vector<string> ports;                       // List of all ports connected, including ignored ports
    vector<string> ports_user_ignore;           // List of ports that were connected at startup but not selected. Will ignore in update.
    if (updatesPending) updatesPending->clear();// Clear the updates pending list

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

    /////////////////////////////////////////////////////////////////////////////
    // IMX-5 firmware/bootloader error checking
     if (!fileExists(firmware.fw_IMX_5.path)) {
        m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, "Update Aborted: IMX firmware file does not exist: %s\n", firmware.fw_IMX_5.path.c_str());
        cancel_update();
        return false;
    }

    // Validate the HEX file before starting
    std::string error;
    bool valid = validateHexFile(firmware.fw_IMX_5.path, error);
    if (!valid) {
        m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, "Update Aborted: IMX firmware file corrupt: %s\n", firmware.fw_IMX_5.path.c_str());
        m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, "Error: %s\n", error.c_str());
        cancel_update();
        return false;
    }

    if (!firmware.bl_IMX_5.path.empty())
    {   // Bootloader file is specified
        if (!fileExists(firmware.bl_IMX_5.path)) {
            m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, "Update Aborted: IMX bootloader file does not exist: %s\n", firmware.bl_IMX_5.path.c_str());
            cancel_update();
            return false;
        }
                
        // Check that firmware size will fit using specified bootloader
        size_t pages = calculateFlashPagesUsed(firmware.fw_IMX_5.path, IMX5_FLASH_PAGE_SIZE);
        uint8_t major = 0, minor = 0;
        if (pages >= 8 && extractBootloaderVersionFromHex(firmware.bl_IMX_5.path, major, minor))
        {   // IMX-5 application requires bootloader v6i or newer to write into 8th page of flash memory
            // std::cout << "Bootloader file: v" << static_cast<int>(major) << static_cast<char>(minor) << "\n";

            if (major < 6 || (major == 6 && minor < 'i')) {
                m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, "Update Aborted: IMX-5 bootloader incompatible with firmware. Bootloader v6i or newer required for selected IMX-5 firmware.\n");
                cancel_update();
                return false;
            } 
        }
    }
    /////////////////////////////////////////////////////////////////////////////

    m_infoProgress(NULL, IS_LOG_LEVEL_INFO, "Initializing devices for update...");

    ////////////////////////////////////////////////////////////////////////////
    // Run `mode_thread_serial_app` to put all APP devices into IS-bootloader mode
    ////////////////////////////////////////////////////////////////////////////

    // Put all devices in the correct mode
    m_infoProgress(NULL, IS_LOG_LEVEL_INFO, "Waiting for devices to initialize...");
    while(m_continue_update && !true_if_cancelled())
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
                m_infoProgress(NULL, IS_LOG_LEVEL_INFO, "Discovered device on port %s", ports[i].c_str());
                create_and_start_serial_thread(ports[i], mode_thread_serial_app);
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

    if(m_uploadProgress(NULL, 0.0f) == IS_OP_CANCELLED) 
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if(m_waitAction) m_waitAction(); 
        return false; 
    }

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `get_device_isb_version_thread` to get version from ISB bootloaders
    ////////////////////////////////////////////////////////////////////////////

    // Put all devices in the correct mode
    while(m_continue_update && !true_if_cancelled())
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
                create_and_start_serial_thread(ports[i], get_device_isb_version_thread);
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
        if (current_timeMs() - m_timeStart > 3000)
        {
            m_continue_update = false;
        }

        m_serial_thread_mutex.unlock();
    }

    if(m_uploadProgress(NULL, 0.0f) == IS_OP_CANCELLED) 
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if(m_waitAction) m_waitAction(); 
        return false; 
    }

    m_ctx_mutex.lock();
    for(size_t i = 0; i < ctx.size(); i++)
    {
        if(ctx[i]->isb_mightUpdate)
        {
            confirm_bootload_t confirm;
            confirm.major = ctx[i]->m_isb_major;
            confirm.minor = ctx[i]->m_isb_minor;
            confirm.sn = ctx[i]->m_sn;

            if (updatesPending) updatesPending->push_back(confirm);
        }
    }
    m_ctx_mutex.unlock();

    m_update_mutex.unlock();

    return true;
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

    if(m_uploadProgress(NULL, 0.0f) == IS_OP_CANCELLED) 
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if(m_waitAction) m_waitAction(); 
        return IS_OP_CANCELLED; 
    }
    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `mode_thread_serial_isb` to put all ISB devices into ROM-bootloader (DFU/SAM-BA) mode if necessary
    ////////////////////////////////////////////////////////////////////////////

    while(m_continue_update && !true_if_cancelled())
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
                create_and_start_serial_thread(ports[i], mode_thread_serial_isb, force_isb_update);
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

    if(m_uploadProgress(NULL, 0.0f) == IS_OP_CANCELLED) 
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if(m_waitAction) m_waitAction(); 
        return IS_OP_CANCELLED; 
    }
    m_infoProgress(NULL, IS_LOG_LEVEL_INFO, "Updating...");
    SLEEP_MS(2500);

    ////////////////////////////////////////////////////////////////////////////
    // Run `mgmt_thread_libusb` to update DFU devices
    ////////////////////////////////////////////////////////////////////////////

    m_libusb_devicesActive = 0;

    void* libusb_thread = threadCreateAndStart(mgmt_thread_libusb, NULL);

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `update_thread_serial` to update devices
    ////////////////////////////////////////////////////////////////////////////

    beginTimeMs = current_timeMs();

    is_operation_result overall_result = IS_OP_OK;
    while (m_continue_update && !true_if_cancelled())
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
                // JOIN the finished worker
                threadJoinAndFree(m_serial_threads[l]->thread);
                m_serial_threads[l]->thread = NULL;

                thread_serial_t* t = m_serial_threads[l];        // set by update_thread_serial
                if (t->result != IS_OP_OK && t->result != IS_OP_CANCELLED && t->result != IS_OP_CLOSED)
                {
                    if (overall_result == IS_OP_OK)      // keep the first non-OK as the return
                    {
                        overall_result = t->result;
                    }
                }
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
                create_and_start_serial_thread(ports[i], update_thread_serial, force_isb_update);
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

            tmp = "Update timeout... Timeout of " + to_string(((double)timeout) / 1000) + " Seconds reached.";

            m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, tmp.c_str());
        }
    }

    timeDeltaMs = current_timeMs() - beginTimeMs;

    threadJoinAndFree(libusb_thread);

    // Only report run time if the update was successful
    if (overall_result == IS_OP_OK)
    {
        tmp = "Update succeeded in " + to_string(((double)timeDeltaMs) / 1000) + " seconds.";
        m_infoProgress(NULL, IS_LOG_LEVEL_INFO, tmp.c_str());
    }

    if(m_uploadProgress(NULL, 0.0f) == IS_OP_CANCELLED) 
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if(m_waitAction) m_waitAction(); 
        return IS_OP_CANCELLED; 
    }
    
    // Reset all serial devices up a level into APP or IS-bootloader mode
    for (size_t i = 0; i < ctx.size(); i++)
    {
        if(!ctx[i]->m_port_name.empty())
        {
            serial_port_t port;
            serialPortPlatformInit(&port);
            if (!serialPortOpenRetry(&port, ctx[i]->m_port_name.c_str(), m_baudRate, 1))
            {
                continue;
            }

            ctx[i]->m_port = &port;

            if (ctx[i] && ctx[i]->m_finished_flash) ctx[i]->reboot_up();

            serialPortFlush(&port);
            serialPortClose(&port);
        }

        if (m_waitAction) m_waitAction();
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

    if (overall_result != IS_OP_OK) {
        m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, "Update failed!");
        if(m_waitAction) m_waitAction();     // Final UI update
        return overall_result;
    }

    if(m_waitAction) m_waitAction();     // Final UI update
    return IS_OP_OK;
}

void cISBootloaderThread::cancel_update()
{
    m_continue_update = false; 
    m_update_in_progress = false; 
    m_update_mutex.unlock(); 
    if(m_waitAction) m_waitAction(); 
}
