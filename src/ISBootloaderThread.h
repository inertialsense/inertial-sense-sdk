/**
 * @file ISBootloaderThread.h
 * @brief Drives one or more cISBootloaderBase bootloader sessions concurrently, one worker
 *        thread per connected serial port (or per libusb DFU device), so multiple devices can
 *        be mode-switched and flashed in parallel. See cISBootloaderThread for the thread
 *        lifecycle (start/poll/cancel/join) and callback contract.
 *
 * @author Dave Cutting
 * @copyright Copyright (c) 2014-2025 Inertial Sense, Inc. All rights reserved. See the MIT
 *            license text below.
 */

/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_THREAD_H_
#define __IS_BOOTLOADER_THREAD_H_

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <mutex>

#include "serialPort.h"
#include "ISUtilities.h"
#include "ISBootloaderBase.h"

/**
 * Coordinates one bootloader "pass" across every connected device by fanning work out to a pool
 * of worker threads -- one thread per serial port (thread_port_t) or per libusb DFU device
 * (thread_libusb_t) -- and joining them back together before returning to the caller. All state
 * (device contexts, thread maps, progress/status callbacks) is static/process-wide: only one
 * update sequence may be in flight at a time, serialized by m_update_mutex.
 *
 * Thread lifecycle:
 *  - **Start**: set_mode_and_check_devices() and update() each spin up worker threads on demand
 *    (via create_and_start_port_thread()/create_and_start_libusb_thread()) as new ports/devices
 *    are discovered, up to once per port/device per phase.
 *  - **Poll**: the calling thread polls each worker's `done` flag in a loop, optionally invoking
 *    a caller-supplied waitAction() callback every iteration (e.g. to pump a UI event loop) so the
 *    call remains effectively synchronous to the caller despite the internal threading.
 *  - **Cancel**: cancel_update() (or the progress callbacks returning IS_OP_CANCELLED, checked via
 *    true_if_cancelled()) sets m_continue_update = false, which the poll loops observe to stop
 *    launching new work and unwind.
 *  - **Join**: once every worker's `done` flag is set (or cancellation/timeout is observed), each
 *    thread is joined and freed with threadJoinAndFree() before the next phase begins or the call
 *    returns.
 *
 * Callback contract: uploadProgress/verifyProgress (fwUpdate::pfnProgressCb) are invoked with the
 * current step name/index and 0-100 percent complete, and their return value doubles as a cancel
 * signal -- returning IS_OP_CANCELLED (checked by true_if_cancelled()) aborts the in-progress
 * update. infoProgress (fwUpdate::pfnStatusCb) receives printf-style status/log messages. All
 * three callbacks may be invoked from worker threads, not just the calling thread.
 */
class cISBootloaderThread
{
public:
    cISBootloaderThread() {};
    ~cISBootloaderThread() {};

    /**
     * Per-port worker-thread context used while mode-switching or updating one device.
     *
     * The port may either be BORROWED from the caller (any transport -- serial, TCP, a relayed port)
     * or OWNED by this worker, constructed from a serial port name for callers that only have a name.
     * A borrowed port is never closed here; closing someone else's port is not ours to do.
     */
    class thread_port_t{
    public:
        void* thread;                                //!< opaque handle to the running worker thread, or NULL once joined
        ISBootloader::cISBootloaderBase* ctx;         //!< the bootloader session created for this device, once identified (NULL until then)
        is_operation_result opResult;                 //!< result of the worker's most recent operation
        bool done;                                    //!< true once the worker thread has finished and can be joined
        bool reuse_port;                              //!< true if the port should be reopened and reused in the next phase rather than treated as new
        bool force_isb;                                //!< true if an ISB bootloader update should be forced even if the version already appears compatible

        /**
         * Adopts a port the caller already owns, of any transport. This is the transport-agnostic path:
         * the ISbl protocol layer only needs a port_handle_t, so a TCP/relayed port works here exactly
         * as a serial one does.
         * @param existingPort the caller's port; borrowed, not owned, and never closed by this worker
         * @param force_isb_update true to force an ISB bootloader update regardless of version
         */
        explicit thread_port_t(port_handle_t existingPort, bool force_isb_update = false)
            : thread(nullptr), ctx(nullptr), opResult(IS_OP_OK), done(false), reuse_port(false),
              force_isb(force_isb_update), m_port(existingPort), m_ownsPort(false) { }

        /**
         * Convenience for callers that only have a serial port NAME: constructs and owns a serial port.
         * @param port_name the serial port name (e.g. "/dev/ttyACM0") this worker will operate on
         * @param force_isb_update true to force an ISB bootloader update regardless of version
         */
        explicit thread_port_t(const std::string& port_name, bool force_isb_update = false)
            : thread(nullptr), ctx(nullptr), opResult(IS_OP_OK), done(false), reuse_port(false),
              force_isb(force_isb_update), m_ownsPort(true)
        {
            m_port = (port_handle_t)&m_ownedSerialPort;
            serialPortSetName(m_port, port_name.c_str());
            serialPortInit(m_port, (int)m_port_threads.size(), PORT_TYPE__UART | PORT_TYPE__COMM, 0);
        }

        /** Closes the port only if this worker created it, then marks the worker done. */
        virtual ~thread_port_t() {
            if (m_ownsPort)
                serialPortClose(m_port);
            thread = NULL;
            done = true;
        }

        /** @return the port this worker operates on, whatever its transport. */
        port_handle_t port() const { return m_port; }

        /** @return true if this worker constructed (and therefore closes) its own port. */
        bool ownsPort() const { return m_ownsPort; }

        /** @return true if the port is usable. Owned ports report via their serial errorCode; borrowed ports are validated generically. */
        bool portUsable() const { return m_ownsPort ? (m_ownedSerialPort.errorCode == 0) : portIsValid(m_port); }

        /**
         * Opens the port if it is not already open. Owned serial ports use the retrying serial open (the
         * device may still be re-enumerating); borrowed ports are opened through their own transport.
         * @param baudRate baud rate to apply when opening an owned serial port
         * @return PORT_ERROR__NONE on success
         */
        int openIfNeeded(int baudRate) {
            if (portIsOpened(m_port))
                return PORT_ERROR__NONE;
            if (m_ownsPort)
                return serialPortOpenRetry(m_port, portName(m_port), baudRate, 1);
            return portOpen(m_port);
        }

    private:
        serial_port_t m_ownedSerialPort = {};        //!< backing storage; used only by the name-based constructor
        port_handle_t m_port = nullptr;              //!< the port in use, borrowed or owned
        bool m_ownsPort = false;                     //!< true when m_port refers to m_ownedSerialPort
    };

    /** Per-libusb-device worker-thread context used while updating one DFU device. */
    class thread_libusb_t {
    public:
        void* thread;                              //!< opaque handle to the running worker thread, or NULL once joined
        libusb_device_handle* handle;               //!< open libusb handle for the device this worker operates on
        char uid[100];                              //!< the device's DFU UID string, used to avoid double-dispatching the same device
        ISBootloader::cISBootloaderBase* ctx;        //!< the bootloader session created for this device, once identified (NULL until then)
        is_operation_result opResult;                //!< result of the worker's most recent operation
        bool done;                                   //!< true once the worker thread has finished and can be joined
    };

    /** Identifies one ISB-mode device whose bootloader version is older than the supplied image and would be updated if the update proceeds. */
    typedef struct
    {
        uint32_t sn;         //!< the device's Inertial Sense serial number
        uint8_t major;       //!< the device's current ISB bootloader major version
        char minor;          //!< the device's current ISB bootloader minor version
        port_handle_t port;  //!< the serial port the device is connected on
    } confirm_bootload_t;

    /**
     * Puts every device on the given ports into the correct mode to begin an update (APP devices
     * are rebooted into ISB, ISB devices are queried for their bootloader version) and reports
     * which ones would actually be updated, without performing the update itself. Intended to be
     * called before update() so the caller can confirm with the user first.
     * @param comPorts the serial ports to consider; ports not connected at call time are ignored
     * @param baudRate the serial baud rate to use once devices are in ISB mode
     * @param firmware candidate firmware/bootloader image paths for every supported target
     * @param uploadProgress callback invoked to report download progress; also polled for cancellation
     * @param verifyProgress callback invoked to report verify progress
     * @param infoProgress callback invoked to report status/log messages
     * @param waitAction optional callback invoked on every poll iteration (e.g. to pump a UI event loop); may be NULL
     * @param updatesPending if non-null, receives one confirm_bootload_t per ISB device that would be updated
     * @return true on success, false if the update was aborted (bad firmware paths, incompatible bootloader, or cancellation)
     */
    static bool set_mode_and_check_devices(
        std::vector<std::string>&               comPorts,
        int                                     baudRate,
        const ISBootloader::firmwares_t&        firmware,
        fwUpdate::pfnProgressCb                 uploadProgress,
        fwUpdate::pfnProgressCb                 verifyProgress,
        fwUpdate::pfnStatusCb                   infoProgress,
        void                                    (*waitAction)() = NULL,
        std::vector<confirm_bootload_t>*        updatesPending = NULL
    );

    /**
     * Drives every device on the given ports (and any DFU devices found via libusb) through
     * however many reboot/flash steps are needed to reach an up-to-date application image,
     * running one worker thread per device/port and joining them all before returning.
     * @param comPorts the serial ports to consider; ports not connected at call time are ignored
     * @param force_isb_update if true, force an ISB bootloader update even if the version already appears compatible
     * @param baudRate the serial baud rate to use for the update
     * @param firmware candidate firmware/bootloader image paths for every supported target
     * @param uploadProgress callback invoked to report download progress; also polled for cancellation
     * @param verifyProgress callback invoked to report verify progress
     * @param infoProgress callback invoked to report status/log messages
     * @param waitAction optional callback invoked on every poll iteration (e.g. to pump a UI event loop); may be NULL
     * @return IS_OP_OK if at least one device updated successfully and none failed, IS_OP_CANCELLED if cancelled via a progress callback, otherwise the first error encountered
     */
    static is_operation_result update(
        std::vector<std::string>&               comPorts,
        bool                                    force_isb_update,
        int                                     baudRate,
        const ISBootloader::firmwares_t&        firmware,
        fwUpdate::pfnProgressCb                 uploadProgress,
        fwUpdate::pfnProgressCb                 verifyProgress,
        fwUpdate::pfnStatusCb                   infoProgress,
        void                                    (*waitAction)()
    );

    /** Requests that the in-progress update() or set_mode_and_check_devices() call stop launching new work and unwind as soon as possible. */
    static void cancel_update();

    static std::vector<ISBootloader::cISBootloaderBase*> ctx;   //!< bootloader sessions created for devices identified during the current/most recent update pass
    static std::mutex m_ctx_mutex;                                //!< guards concurrent access to ctx across worker threads

    static bool m_update_in_progress;   //!< true while update() or set_mode_and_check_devices() is running

private:
    static void create_and_start_port_thread(const std::string& port, void(*function)(void*), bool force_isb_update = false);

    /**
     * Starts a worker on a port the caller already owns, rather than on a serial port name.
     * @param port the port to adopt; borrowed, not owned, and never closed by the worker
     * @param target the requested target string this port was resolved from, used as the worker's key
     * @param function the worker entry point to run
     * @param force_isb_update true to force an ISB bootloader update regardless of version
     */
    static void create_and_start_port_thread(port_handle_t port, const std::string& target, void(*function)(void*), bool force_isb_update = false);

    /**
     * Starts workers for every requested target that is a URL rather than a local serial port.
     *
     * The enumeration-driven loops can only ever see what cISSerialPort::GetComPorts() reports, so a
     * requested `tcp://host:port` target was in the target set but never in the enumerated list, and
     * therefore never got a worker at all -- the work set came back empty and the update reported
     * "No devices were updated". This resolves such targets through TcpPortFactory and adopts them.
     *
     * @param targetPorts the caller's requested targets
     * @param function the worker entry point to run for each URL target
     * @param force_isb_update true to force an ISB bootloader update regardless of version
     */
    static void start_threads_for_url_targets(const std::set<std::string>& targetPorts, void(*function)(void*), bool force_isb_update = false);

    /** Releases every port bound by start_threads_for_url_targets(); workers borrow, so they never do. */
    static void release_bound_ports();
    static void create_and_start_libusb_thread(void(*function)(void*), libusb_device_handle* handle);
    static void get_device_isb_version_thread(void* context);
    static void mode_thread_port_app(void* context);
    static void mode_thread_port_isb(void* context);
    static void update_thread_port(void* context);
    static void update_thread_libusb(void* context);
    static void mgmt_thread_libusb(void* context);
    static bool true_if_cancelled(void);

    static ISBootloader::firmwares_t m_firmware;
    
    static int m_baudRate;

    static std::mutex m_update_mutex;
    
    static fwUpdate::pfnProgressCb m_uploadProgress;
    static fwUpdate::pfnProgressCb m_verifyProgress;
    static fwUpdate::pfnStatusCb m_infoProgress;
    static void (*m_waitAction)();

    static uint32_t m_timeStart;
    static bool m_use_dfu;
    static uint32_t m_libusb_devicesActive;
    static uint32_t m_port_devicesActive;

    /** Ports bound from URL targets by start_threads_for_url_targets(), keyed by target string.
     *  Workers borrow these, so ownership stays here and release_bound_ports() frees them. */
    static std::map<std::string, port_handle_t> m_boundPorts;

    static bool m_continue_update;

    static std::map<std::string, thread_port_t*> m_port_threads;    // map of ports to corresponding threads
    static std::vector<thread_libusb_t*> m_libusb_threads;    // List of all libusb threads that have run or are running
    static std::mutex m_port_thread_mutex;
    static std::mutex m_libusb_thread_mutex;
};

#endif // __IS_BOOTLOADER_THREAD_H_
