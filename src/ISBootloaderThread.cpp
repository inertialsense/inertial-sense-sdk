/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderThread.h"
#include "TcpPortFactory.h"
#include "ISBootloaderDFU.h"
#include "ISBootloaderAPP.h"
#include "ISBootloaderISB.h"
#include "ISBootloaderSAMBA.h"
#include "ISSerialPort.h"
#include "protocol/FirmwareUpdate.h"
#include "intel_hex_utils.h"

#include <algorithm>
#include <set>
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
mutex cISBootloaderThread::m_port_thread_mutex;
mutex cISBootloaderThread::m_libusb_thread_mutex;
bool cISBootloaderThread::m_update_in_progress = false;
mutex cISBootloaderThread::m_update_mutex;
bool cISBootloaderThread::m_use_dfu;
uint32_t cISBootloaderThread::m_libusb_devicesActive;
uint32_t cISBootloaderThread::m_port_devicesActive;
bool cISBootloaderThread::m_continue_update;
map<std::string, cISBootloaderThread::thread_port_t*> cISBootloaderThread::m_port_threads;
map<std::string, port_handle_t> cISBootloaderThread::m_boundPorts;
set<std::string> cISBootloaderThread::m_urlPhaseStarted;
vector<cISBootloaderThread::thread_libusb_t*> cISBootloaderThread::m_libusb_threads;

void cISBootloaderThread::mgmt_thread_libusb(void* context)
{
    (void)context;

    // Initialize libusb
    m_use_dfu = libusb_init(NULL) == LIBUSB_SUCCESS;

    is_dfu_list dfu_list;                       // List of libusb devices connected

    m_libusb_threads.clear();

    cISBootloaderDFU::m_DFUmutex.lock();

    std::set<std::string> claimed_uids;         // UIDs of DFU devices already dispatched to a thread

    while (m_continue_update)
    {
        m_libusb_thread_mutex.lock();

        // Rescan for newly-enumerated DFU devices each iteration so that devices
        // which take longer than the initial 2.5s wait to enumerate (e.g. 8
        // simultaneous ISB->DFU reboots on a shared hub) are still discovered.
        cISBootloaderDFU::list_devices(&dfu_list);
        for (size_t i = 0; i < dfu_list.present; i++)
        {
            std::string uid(dfu_list.id[i].uid);
            if (claimed_uids.count(uid))
            {   // Already launched a thread for this device; close the newly-opened handle.
                libusb_close(dfu_list.id[i].handle_libusb);
                continue;
            }

            // Also skip if already tracked in the context list
            bool in_ctx = false;
            m_ctx_mutex.lock();
            for (size_t j = 0; j < ctx.size(); j++)
            {
                if (!ctx[j]->is_serial_device() && ctx[j]->match_test((void*)dfu_list.id[i].uid) == IS_OP_OK)
                {
                    in_ctx = true;
                    break;
                }
            }
            m_ctx_mutex.unlock();

            if (in_ctx)
            {
                libusb_close(dfu_list.id[i].handle_libusb);
                continue;
            }

            // New DFU device — start an update thread
            claimed_uids.insert(uid);
            create_and_start_libusb_thread(update_thread_libusb, dfu_list.id[i].handle_libusb);
        }

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
 * @param context the thread context, a thread_port_t providing details about the device to query/configure
 */
void cISBootloaderThread::mode_thread_port_app(void* context)
{
    thread_port_t* thread_info = (thread_port_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    port_handle_t port = thread_info->port();
    if (thread_info->openIfNeeded(m_baudRate) != PORT_ERROR__NONE)
    {
        m_infoProgress(std::any(), IS_LOG_LEVEL_ERROR, "Error opening port '%s': port error %d", portName(port), portError(port));
        portClose(port);
        m_port_thread_mutex.lock();
        thread_info->done = true;
        thread_info->allow_new_worker = true;
        m_port_thread_mutex.unlock();
        return;
    }

    is_operation_result result = cISBootloaderBase::mode_device_app(m_firmware, port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    portFlush(port);
    portClose(port);

    m_port_thread_mutex.lock();
    thread_info->opResult = result;
    thread_info->allow_new_worker = false;
    thread_info->done = true;
    m_port_thread_mutex.unlock();
}

void cISBootloaderThread::create_and_start_port_thread(const string& port_name, void(*function)(void*), bool force_isb_update)
{
    thread_port_t* new_thread = new thread_port_t(port_name); // (thread_port_t*)malloc(sizeof(thread_port_t));
    if (new_thread->portUsable()) {
        m_port_threads[port_name] = new_thread;
        new_thread->ctx = NULL;
        new_thread->done = false;
        new_thread->opResult = IS_OP_OK;
        new_thread->force_isb = force_isb_update;
        new_thread->thread = threadCreateAndStart(function, new_thread, port_name.c_str());

        m_infoProgress(std::any(), IS_LOG_LEVEL_MORE_DEBUG, "mode_thread_port_app found viable port: %s", port_name.c_str());
        m_port_devicesActive++;
    }
}

void cISBootloaderThread::create_and_start_port_thread(port_handle_t port, const string& target, void(*function)(void*), bool force_isb_update)
{
    thread_port_t* new_thread = new thread_port_t(port, force_isb_update);
    if (!new_thread->portUsable()) {
        delete new_thread;      // don't leak the worker when its port is unusable
        return;
    }

    m_port_threads[target] = new_thread;
    new_thread->ctx = NULL;
    new_thread->done = false;
    new_thread->opResult = IS_OP_OK;
    new_thread->thread = threadCreateAndStart(function, new_thread, target.c_str());

    m_infoProgress(std::any(), IS_LOG_LEVEL_INFO, "Discovered device on port %s", target.c_str());
    m_port_devicesActive++;
}

void cISBootloaderThread::start_threads_for_url_targets(const std::set<std::string>& targetPorts, void(*function)(void*), bool force_isb_update)
{
    for (const std::string& target : targetPorts) {
        // A URL target is one the serial enumeration can never report. Keyed off the scheme separator
        // rather than "absent from GetComPorts()", so that a serial tty which is momentarily missing
        // from enumeration (mid-reboot) is never mistaken for a URL and bound as a TCP port.
        if (target.find("://") == std::string::npos)
            continue;

        // EXACTLY ONE worker per target per phase -- no retries.
        //
        // These discovery loops re-run every ~100ms for several seconds, so anything that merely checks
        // "is a worker running?" re-creates a failed worker over and over. That is not a harmless retry:
        // each live worker keeps m_port_devicesActive non-zero, which resets the loop's own no-device
        // bail-out timer, so the sequence never terminates, and every attempt after the first operates on
        // a device the previous attempt left mid-flash -- so the log fills with errors that describe
        // damage rather than the original fault. Once the handshake has happened, a failure is a failure.
        //
        // The phase is identified by the worker function, so app-mode, isb-version, isb-mode and update
        // each still get their single attempt and the sequence progresses normally; only re-attempts
        // WITHIN a phase are suppressed. (The ISbl layer keeps its own handshake retry, which is a
        // different thing and deliberately left alone.)
        char fnKey[24];
        snprintf(fnKey, sizeof(fnKey), "|%p", (void*)function);
        const std::string phaseKey = target + fnKey;
        if (m_urlPhaseStarted.count(phaseKey))
            continue;

        auto existing = m_port_threads.find(target);
        if (existing != m_port_threads.end())
        {
            thread_port_t* prev = existing->second;
            if (prev && (!prev->done || (prev->thread != nullptr)))
                continue;               // a worker from an earlier phase is still running or unjoined
            // ERASE the entry rather than leaving it NULL. Six of the loops that walk m_port_threads
            // dereference the value with no null check (e.g. `if (portThread->thread != NULL ...)` and
            // `if (!portThread->done)`) -- only the final cleanup pass guards it. A retired-to-NULL slot
            // was therefore dereferenced on the next pass and segfaulted the process.
            delete prev;                            // safe when already null
            m_port_threads.erase(existing);         // iterator is dead after this; do not reuse it
        }

        m_urlPhaseStarted.insert(phaseKey);

        // Bind once per target per sequence, caching FAILURES as well as successes: these loops re-run
        // every ~100ms for several seconds, so retrying a bind that cannot succeed both spams the log
        // and re-resolves the host each time.
        auto bound = m_boundPorts.find(target);
        if (bound == m_boundPorts.end()) {
            // PORT_TYPE__TCP is required, not optional: TcpPortFactory::validatePort() rejects anything
            // without that bit set, and bindPort() validates before parsing -- so omitting it makes
            // every bind fail regardless of how good the URL is.
            port_handle_t newPort = TcpPortFactory::getInstance().bindPort(target, PORT_TYPE__TCP);
            if (!newPort)
                m_infoProgress(std::any(), IS_LOG_LEVEL_ERROR, "Unable to bind a port for target %s", target.c_str());
            m_boundPorts[target] = newPort;
            bound = m_boundPorts.find(target);
        }
        if (!bound->second)
            continue;

        create_and_start_port_thread(bound->second, target, function, force_isb_update);
    }
}

void cISBootloaderThread::release_bound_ports()
{
    for (auto& [target, port] : m_boundPorts) {
        if (port)
            TcpPortFactory::getInstance().releasePort(port);
    }
    m_boundPorts.clear();
    m_urlPhaseStarted.clear();
}

void cISBootloaderThread::create_and_start_libusb_thread(void(*function)(void*), libusb_device_handle* handle)
{
    thread_libusb_t* new_thread = new thread_libusb_t();
    new_thread->ctx = NULL;
    new_thread->done = false;
    new_thread->handle = handle;
    new_thread->opResult = IS_OP_OK;
    m_libusb_threads.push_back(new_thread);
    new_thread->thread = threadCreateAndStart(function, new_thread, "isb-libusb");
    m_libusb_devicesActive++;
}

void cISBootloaderThread::get_device_isb_version_thread(void* context)
{
    thread_port_t* thread_info = (thread_port_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    port_handle_t port = thread_info->port();
    // m_baudRate here is part of a real defect, but do NOT "fix" it in isolation. A freshly reset ISbl
    // locks its autobaud onto the FIRST coherent 'U' burst it can measure and thereafter ignores every
    // other rate. Shortly after BLEN a burst at 115200 locks on the first try, while one at 921600 may not
    // take for another ~10-15 bursts -- which is why a from-APP update can appear to wait ~20 s for a
    // device that reboots in 1-2 s.
    //
    // Changing only this phase to 115200 made things WORSE, not better: it locked the device at 115200
    // at +8 s, after which mode_thread_port_isb (m_baudRate) and update_device's
    // reopen_port_for_update(port, baud) at ISBootloaderBase.cpp:565 both spoke 921600 to a device
    // locked at 115200 and got permanent silence -- 0/1 where the unchanged code was 4/4.
    //
    // The fix has to make the negotiation baud consistent across ALL ISbl phases, and decide what
    // happens to the bulk transfer rate once the device is locked. Until then, leaving every phase on
    // m_baudRate is at least self-consistent.
    if (thread_info->openIfNeeded(m_baudRate) != PORT_ERROR__NONE)
    {
        m_infoProgress(std::any(), IS_LOG_LEVEL_ERROR, "Error opening port '%s': port error %d", portName(port), portError(port));
        portClose(port);
        m_port_thread_mutex.lock();
        thread_info->done = true;
        thread_info->allow_new_worker = true;
        m_port_thread_mutex.unlock();
        return;
    }

    is_operation_result result = cISBootloaderBase::get_device_isb_version(m_firmware, port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    portFlush(port);
    portClose(port);

    m_port_thread_mutex.lock();
    thread_info->opResult = result;
    // FALSE, not true. This worker reached a terminal answer -- the device either is an ISB device
    // (its context is now in `ctx`) or definitively is not. Neither warrants another attempt.
    //
    // Setting it true here meant the discovery loop's gate (`done && !allow_new_worker`) never matched, so
    // a brand-new worker was spawned for this same port every time one finished, for the entire 3 s
    // phase window -- each one constructing a fresh cISBootloaderISB, re-opening the port and
    // re-querying a version already known. The flag
    // is only read by those spawn gates, so `true` here is a spawn instruction and nothing else.
    // The open-failure path above still sets it true: that IS transient and does warrant a retry.
    thread_info->allow_new_worker = false;
    thread_info->done = true;
    m_port_thread_mutex.unlock();
}

/**
 * Thread handler to validating and then enabling a serial-device to enter ISB mode.
 * @param context the thread context, a thread_port_t providing details about the device to query/configure
 */
void cISBootloaderThread::mode_thread_port_isb(void* context)
{
    thread_port_t* thread_info = (thread_port_t*)context;
    cISBootloaderBase* new_context;

    SLEEP_MS(IS_REBOOT_DELAY_MS);     // Wait for all other threads to start

    // attempt to open the target port; if unable to open, terminate this thread
    port_handle_t port = thread_info->port();
    if (thread_info->openIfNeeded(m_baudRate) != PORT_ERROR__NONE)
    {
        m_infoProgress(std::any(), IS_LOG_LEVEL_ERROR, "Error opening port '%s': port error %d", portName(port), portError(port));
        portClose(port);
        m_port_thread_mutex.lock();
        thread_info->done = true;
        thread_info->allow_new_worker = true;
        m_port_thread_mutex.unlock();
        return;
    }

    is_operation_result result = cISBootloaderBase::mode_device_isb(m_firmware, thread_info->force_isb, port, m_infoProgress, m_uploadProgress, m_verifyProgress, ctx, &m_ctx_mutex, &new_context);

    portFlush(port);
    portClose(port);

    m_port_thread_mutex.lock();
    thread_info->opResult = result;
    // MUST stay true, and this is the one place where the flag genuinely does reach across a phase
    // boundary. Unlike the app and ISB-version phases, this phase's join loop does NOT clear
    // m_port_threads, so this entry survives into the UPDATE phase -- whose spawn gate reads this same
    // flag. Setting it false here therefore does not merely suppress a retry: it makes the update
    // phase find a finished, non-retryable entry for the port and create no update worker at all,
    // reporting "No devices were updated (succeeded 0, failed 0)" with the device left in ISbl.
    //
    // So the flag answers a single question -- "an entry already exists for this port; should a worker
    // still be started?" -- whose scope is per-phase only where the map is cleared between phases.
    // Clearing m_port_threads consistently (or having each phase track its own workers) is what would
    // let this be false; until then, true is required.
    thread_info->allow_new_worker = true;
    thread_info->done = true;
    m_port_thread_mutex.unlock();
}

/**
 * Thread handler to perform a firmware update on a specific device.
 * @param context the thread context, a thread_port_t providing details about the device to query/configure
 */
void cISBootloaderThread::update_thread_port(void* context)
{
    thread_port_t* thread_info = (thread_port_t*)context; 
    cISBootloaderBase* new_context;

    SLEEP_MS(100);

    port_handle_t port = thread_info->port();
    // Re-initialise only a port this worker OWNS. serialPortInit()/serialPortSetName() install the
    // serial implementation's function pointers, so running them against a borrowed port of another
    // transport (a relayed tcp:// port) would overwrite it into a broken half-serial port. A borrowed
    // port arrives ready to use from its own factory and must be left alone.
    // A thread_port_t is carried over from the previous phase, so clear any retry request it left
    // behind before this phase's work begins. This is independent of port ownership.
    m_port_thread_mutex.lock();
    thread_info->allow_new_worker = false;
    m_port_thread_mutex.unlock();

    if (thread_info->ownsPort()) {
        serialPortInit(port, BASE_PORT(port)->pnum, BASE_PORT(port)->ptype, BASE_PORT(port)->pflags);
        m_port_thread_mutex.lock();
        const char* serial_name = portName(port);
        m_port_thread_mutex.unlock();

        // Start at 115200 always, we will switch to user specified rate after we check for SAM-BA devices
        serialPortSetName(port, serial_name);
    }
    if (thread_info->openIfNeeded(BAUDRATE_115200) != PORT_ERROR__NONE)
    {
        m_infoProgress(std::any(), IS_LOG_LEVEL_ERROR, "Error opening port '%s': port error %d", portName(port), portError(port));
        portClose(port);
        m_port_thread_mutex.lock();
        thread_info->done = true;
        m_port_thread_mutex.unlock();
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

        m_port_thread_mutex.lock();
        thread_info->ctx = new_context;
        m_port_thread_mutex.unlock();
    }
    else if (thread_info->opResult == IS_OP_CLOSED)
    {
        // Device is resetting (may have updated if it was a SAM-BA device)
        m_port_thread_mutex.lock();
        thread_info->allow_new_worker = true;
        m_port_thread_mutex.unlock();
    }
    else if (thread_info->opResult == IS_OP_CANCELLED)
    {
        // Device has already been updated
    }
    else // (IS_OP_ERROR usually)
    {
        // Other device
    }

    portFlush(port);
    // portClose(port);  DON'T CLOSE THE PORT - We may need it later to finalize/validate everything...

    m_port_thread_mutex.lock();
    thread_info->done = true;
    m_port_thread_mutex.unlock();
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
    if (m_uploadProgress(std::any(), 0.0f, "", 0, 0) == IS_OP_CANCELLED)
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
    fwUpdate::pfnProgressCb                 uploadProgress,
    fwUpdate::pfnProgressCb                 verifyProgress,
    fwUpdate::pfnStatusCb                   infoProgress,
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

    vector<string> portNames;                   // List of all ports currently connected
    if (updatesPending) updatesPending->clear();// Clear the updates pending list

    m_port_threads.clear();
    release_bound_ports();

    // Only ever operate on the ports the caller explicitly asked for.
    //
    // An INCLUSION set, deliberately: it cannot grow behind the caller's back. An exclusion list would,
    // because a port appearing after the snapshot is absent from it and so counts as selected -- and a
    // device re-enumerating its USB CDC node (which is what happens seconds after "Rebooting to APP
    // mode...") appears exactly that way. That would let `-c /dev/ttyACM99` reboot an unrelated device
    // into the bootloader and strand it there.
    std::set<std::string> targetPorts(comPorts.begin(), comPorts.end());

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
    // Run `mode_thread_port_app` to put all APP devices into IS-bootloader mode
    ////////////////////////////////////////////////////////////////////////////

    // Put all devices in the correct mode
    m_infoProgress(std::any(), IS_LOG_LEVEL_INFO, "Waiting for devices to initialize...");
    while (m_continue_update && !true_if_cancelled())
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(100);

        cISSerialPort::GetComPorts(portNames);

        m_port_thread_mutex.lock();

            // GetComPorts() can only report local serial ports, so a requested tcp:// target
            // would never appear in portNames and never get a worker. Resolve and adopt those here.
            start_threads_for_url_targets(targetPorts, mode_thread_port_app);

        for (auto port_name : portNames)
        {
            bool found = false;

            for (auto& [portName, portThread] : m_port_threads)
            {
                if (portName == port_name)
                {
                    if (!portThread->done)    //(m_port_threads[j]->ctx != NULL ||
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (portThread->done && !portThread->allow_new_worker)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
                }
            }

            if (!targetPorts.count(port_name))
                found = true;    // not a requested target -- never touch it

            if (!found)
            {
                m_infoProgress(NULL, IS_LOG_LEVEL_INFO, "Discovered device on port %s", port_name.c_str());
                create_and_start_port_thread(port_name, mode_thread_port_app);
            }
        }

        // Break after 5 seconds
        if (current_timeMs() - m_timeStart > 5000)
        {
            m_continue_update = false;
        }

        m_port_thread_mutex.unlock();
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

        m_port_thread_mutex.lock();

        for (auto& [portName, portThread] : m_port_threads)
        {
            if (!portThread->done)
            {
                m_continue_update = true;
            }
            else if (portThread->thread != NULL)
            {
                threadJoinAndFree(portThread->thread);
                portThread->thread = NULL;
                delete portThread;
                m_port_threads[portName] = NULL;
            }
        }

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 5000) 
        {
            m_continue_update = false;
        }

        m_port_thread_mutex.unlock();
        m_port_threads.clear();
    }

    if (m_uploadProgress(std::any(), 0.0f, "", 0, 0) == IS_OP_CANCELLED)
    {
        m_continue_update = false;
        m_update_in_progress = false;
        release_bound_ports();      // cancelled here means update() never runs, so nothing else will
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
    while (m_continue_update && !true_if_cancelled())
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        cISSerialPort::GetComPorts(portNames);

        m_port_thread_mutex.lock();

            // GetComPorts() can only report local serial ports, so a requested tcp:// target
            // would never appear in portNames and never get a worker. Resolve and adopt those here.
            start_threads_for_url_targets(targetPorts, get_device_isb_version_thread);

        for (auto port_name : portNames)
        {
            bool found = false;
            for (auto& [portName, portThread] : m_port_threads)
            {
                if (portName == port_name)
                {
                    if (!portThread->done)    //(m_port_threads[j]->ctx != NULL ||
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (portThread->done && !portThread->allow_new_worker)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
                }
               
            }

            if (!targetPorts.count(port_name))
                found = true;    // not a requested target -- never touch it


            if (!found)
            {
                create_and_start_port_thread(port_name, get_device_isb_version_thread);
            }
        }

        // Break after 3 seconds
        if (current_timeMs() - m_timeStart > 3000)
        {
            m_continue_update = false;
        }

        m_port_thread_mutex.unlock();
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

        m_port_thread_mutex.lock();

        for (auto& [portName, portThread] : m_port_threads)
        {
            if (!portThread->done)
            {
                m_continue_update = true;
            }
            else if (portThread->thread != NULL)
            {
                threadJoinAndFree(portThread->thread);
                portThread->thread = NULL;
                portClose(portThread->port());
                delete portThread;
                m_port_threads[portName] = NULL;
            }
        }

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 3000)
        {
            m_continue_update = false;
        }

        m_port_thread_mutex.unlock();
        m_port_threads.clear();
    }

    if (m_uploadProgress(std::any(), 0.0f, ""/*"Waiting for device response."*/, 0, 0) == IS_OP_CANCELLED)
    {
        m_continue_update = false;
        m_update_in_progress = false;
        release_bound_ports();      // cancelled here means update() never runs, so nothing else will
        m_update_mutex.unlock();
        if(m_waitAction) m_waitAction();
        return false;
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

    vector<string> portNames;                       // List of ports currently connected

    m_port_threads.clear();

    // Backstop the release of URL-bound ports, so no exit from here can leave a TCP handle bound or a
    // stale phase key in m_urlPhaseStarted -- a leftover key suppresses that phase's single worker on the
    // NEXT sequence, which then reports "no devices" rather than the cancellation that caused it.
    // The normal path still releases explicitly, before the update mutex is unlocked; this covers the
    // cancellation returns, which unlock and return directly. release_bound_ports() is idempotent, so
    // running it twice on the normal path costs a walk of an empty map.
    struct bound_port_guard_t {
        ~bound_port_guard_t() { release_bound_ports(); }
    } boundPortGuard;

    // Only ever operate on the ports the caller explicitly asked for. See the note in
    // set_mode_and_check_devices() for why an exclusion list was unsafe here.
    std::set<std::string> targetPorts(comPorts.begin(), comPorts.end());

    if (m_uploadProgress(std::any(), 0.0f, ""/*"Writing Flash"*/, 0, 0) == IS_OP_CANCELLED)
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
    // Run `mode_thread_port_isb` to put all ISB devices into ROM-bootloader (DFU/SAM-BA) mode if necessary
    ////////////////////////////////////////////////////////////////////////////

    while (m_continue_update && !true_if_cancelled())
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(1000);

        cISSerialPort::GetComPorts(portNames);

        m_port_thread_mutex.lock();

            // GetComPorts() can only report local serial ports, so a requested tcp:// target
            // would never appear in portNames and never get a worker. Resolve and adopt those here.
            start_threads_for_url_targets(targetPorts, mode_thread_port_isb, force_isb_update);

        for (auto port_name : portNames)
        {
            bool found = false;

            for (auto& [portName, portThread] : m_port_threads)
            {
                if (portName == port_name)
                {
                    found = true;
                    break;
                }
            }

            if (!targetPorts.count(port_name))
                found = true;    // not a requested target -- never touch it

            if (!found)
            {
                create_and_start_port_thread(port_name, mode_thread_port_isb, force_isb_update);
            }
        }

        m_port_thread_mutex.unlock();

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

        m_port_thread_mutex.lock();

        for (auto& [portName, portThread] : m_port_threads)
        {
            if (!portThread->done)
            {
                m_continue_update = true;
            }
            else if (portThread->thread != NULL)
            {
                threadJoinAndFree(portThread->thread);
                portThread->thread = NULL;
            }
        }

        // Timeout after 5 seconds
        if (current_timeMs() - m_timeStart > 5000)
        {
            m_continue_update = false;
        }

        m_port_thread_mutex.unlock();
    }

    if (m_uploadProgress(std::any(), 0.0f, "Writing Flash", 0, 0) == IS_OP_CANCELLED)
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if (m_waitAction) m_waitAction();
        return IS_OP_CANCELLED; 
    }
    m_infoProgress(std::any(), IS_LOG_LEVEL_INFO, "Updating...");
    SLEEP_MS(2500);

    ////////////////////////////////////////////////////////////////////////////
    // Run `mgmt_thread_libusb` to update DFU devices
    ////////////////////////////////////////////////////////////////////////////

    m_libusb_devicesActive = 0;

    void* libusb_thread = threadCreateAndStart(mgmt_thread_libusb, NULL, "isb-update-libusb");

    m_continue_update = true;
    m_timeStart = current_timeMs();

    ////////////////////////////////////////////////////////////////////////////
    // Run `update_thread_port` to update devices
    ////////////////////////////////////////////////////////////////////////////

    beginTimeMs = current_timeMs();

    is_operation_result overall_result = IS_OP_OK;
    int devicesSucceeded = 0;
    int devicesFailed = 0;
    while (m_continue_update && !true_if_cancelled())
    {
        if (m_waitAction) m_waitAction();
        SLEEP_MS(10);

        m_port_devicesActive = 0;

        cISSerialPort::GetComPorts(portNames);

        m_port_thread_mutex.lock();

            // GetComPorts() can only report local serial ports, so a requested tcp:// target
            // would never appear in portNames and never get a worker. Resolve and adopt those here.
            start_threads_for_url_targets(targetPorts, update_thread_port, force_isb_update);

        for (auto& [portName, portThread] : m_port_threads)
        {
            if (portThread->thread != NULL && portThread->done)
            {
                // JOIN the finished worker
                threadJoinAndFree(portThread->thread);
                portThread->thread = NULL;

                thread_port_t* t = portThread;        // set by update_thread_port
                if (t->opResult == IS_OP_OK)
                {
                    devicesSucceeded++;
                }
                else if (t->opResult != IS_OP_CANCELLED && t->opResult != IS_OP_CLOSED)
                {
                    devicesFailed++;
                    if (overall_result == IS_OP_OK)      // keep the first non-OK as the return
                    {
                        overall_result = t->opResult;
                    }
                }
            }

            if (!portThread->done)
            {
                m_port_devicesActive++;
            }
        }

        for (auto port_name : portNames)
        {
            bool found = false;

            for (auto& [portName, portThread] : m_port_threads)
            {
                if (portName == port_name)
                {
                    if (!portThread->done)    //(m_port_threads[j]->ctx != NULL ||
                    {   // Thread hasn't finished
                        found = true;
                        break;
                    }
                    if (portThread->done && !portThread->allow_new_worker)
                    {   // Thread finished and the reuse flag isn't set
                        found = true;
                        break;
                    }
                }
            }

            if (!targetPorts.count(port_name))
                found = true;    // not a requested target -- never touch it


            if (!found)
            {
                create_and_start_port_thread(port_name, update_thread_port, force_isb_update);
            }
        }

        m_libusb_thread_mutex.lock();

        // Break after 3 seconds of no threads active
        if (m_libusb_devicesActive != 0 || m_port_devicesActive != 0) 
        {
            m_timeStart = current_timeMs();
        }
        else if (current_timeMs() - m_timeStart > 3000)
        {
            m_continue_update = false;
        }

        m_libusb_thread_mutex.unlock();
        m_port_thread_mutex.unlock();

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

    threadJoinAndFree(libusb_thread);

    // Report final status
    if (devicesSucceeded > 0 && devicesFailed == 0)
    {
        tmp = "Update succeeded (" + to_string(devicesSucceeded) + " device(s)) in " + to_string(((double)timeDeltaMs) / 1000) + " seconds.";
        m_infoProgress(NULL, IS_LOG_LEVEL_INFO, tmp.c_str());
    }
    else if (devicesSucceeded > 0 && devicesFailed > 0)
    {
        tmp = "Update succeeded on " + to_string(devicesSucceeded) + " device(s), failed on " + to_string(devicesFailed) + " device(s).";
        m_infoProgress(NULL, IS_LOG_LEVEL_WARN, tmp.c_str());
    }
    else
    {
        // Nothing was updated, which is not success. overall_result only goes non-OK when a device thread
        // actually fails, so it is still IS_OP_OK when no device was ever found or initialized and no
        // thread ran at all -- the 3s no-device bail-out above lands here. Return non-OK; the message
        // stays descriptive because this also covers "no eligible target", not only a botched flash.
        if (overall_result == IS_OP_OK)
            overall_result = IS_OP_ERROR;

        tmp = "No devices were updated (succeeded 0, failed " + to_string(devicesFailed) +
              ") after " + to_string(((double)timeDeltaMs) / 1000) + " seconds.";
        m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, tmp.c_str());
    }

    if (m_uploadProgress(std::any(), 0.0f, "", 0, 0) == IS_OP_CANCELLED)
    { 
        m_continue_update = false; 
        m_update_in_progress = false; 
        m_update_mutex.unlock(); 
        if (m_waitAction) m_waitAction();
        return IS_OP_CANCELLED; 
    }
    
    // Reset all serial devices up a level into APP or IS-bootloader mode
    // At this point, its likely that all ports will be closed at completion of the update process, so we need to reopen and then issue reboot_up()
    m_port_thread_mutex.lock();
    for (auto& [portName, portThread] : m_port_threads)
    {
        if (portThread && portThread->done)
        {
            if (portThread->ctx)
                portThread->ctx->reboot_up();

            port_handle_t port = portThread->port();
            if (portIsOpened(port)) {
                portFlush(port);
                portClose(port);
            }
        }
    }
    m_port_thread_mutex.unlock();

    // Clear the ctx list
    for (auto& cur_ctx : ctx)
    {
        delete cur_ctx;
        cur_ctx = nullptr;
    }
    ctx.clear();

    // Release any ports bound from URL targets. Workers only ever borrow them, so nothing else will.
    //
    // Bound once per sequence rather than per phase, to avoid needless connect/teardown churn on a
    // relayed endpoint that every phase targets anyway. Note the relay's endpoint-persistence contract:
    // a commanded reset does NOT move the listening port -- within the reconnect timeout the port is
    // preserved across the device's drop and return, even while its tty re-enumerates underneath. So a
    // borrowed handle survives a reset and there is nothing to re-bind mid-reboot. It is dead only if the
    // outage exceeds that timeout, in which case the device returns on a NEW port -- so never cache a
    // port across a full drop or a daemon restart, and never key anything off the tty name.
    release_bound_ports();

    m_update_in_progress = false;
    m_update_mutex.unlock();

    // Keyed on devicesSucceeded alone: the old `overall_result != IS_OP_OK && devicesSucceeded == 0`
    // could never fire when no device thread ran, because overall_result was still IS_OP_OK -- so the
    // function returned success for an update that never happened. ISv1 has no if-newer/skip policy,
    // so it always attempts an update; zero successes therefore always means nothing was written.
    if (devicesSucceeded == 0) {
        if (overall_result == IS_OP_OK)
            overall_result = IS_OP_ERROR;
        m_infoProgress(NULL, IS_LOG_LEVEL_ERROR, "Update failed!");
        if(m_waitAction) m_waitAction();     // Final UI update
        return overall_result;
    }

    // A PARTIAL failure is a failure. This function updates every port it was given, concurrently, so
    // "some worked" is the normal shape of a bad multi-device run -- and keying the result on
    // devicesSucceeded alone reported IS_OP_OK for 1-of-2, 1-of-8, 1-of-N. That was survivable only
    // while the caller drove one port per call and did its own counting; it is not survivable now that
    // callers pass the whole target list to get the parallelism this class exists to provide.
    if (devicesFailed > 0) {
        if (overall_result == IS_OP_OK)
            overall_result = IS_OP_ERROR;
        if(m_waitAction) m_waitAction();     // Final UI update
        return overall_result;               // the per-device counts were already reported above
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
