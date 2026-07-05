/**
 * @file ISDeviceFwUpdate.cpp
 * @brief Example: update the firmware of an Inertial Sense IMX-5 over a serial/UART connection.
 *
 * Demonstrates the SDK's `ISDevice` interface together with the ISv2 firmware-update framework
 * (a `.fpkg` package driven through `ISDevice::updateFirmware()`), with an emphasis on making the
 * update work over a *fixed* serial port (e.g. a Raspberry Pi's /dev/ttyAMA0, or a USB-serial
 * bridge) rather than a USB-CDC connection.
 *
 * Why that distinction matters: partway through an update the device reboots into its serial
 * bootloader (ISbl). On USB the device re-enumerates as a brand-new port, so the SDK naturally
 * re-discovers it. On a fixed UART the OS device node never changes, so the application must let
 * the SDK re-discover/re-associate the device on that same port. The key to that (see main_explained)
 * is routing the connection through the PortManager/DeviceManager instead of binding a port directly.
 *
 * @author Kyle Mallory on 6/3/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <cstdarg>

#include "PortFactory.h"      // SerialPortFactory - allocates/enumerates serial port handles
#include "PortManager.h"      // PortManager - discovers ports (via registered PortFactories)
#include "DeviceManager.h"    // DeviceManager - owns ISDevice instances, matches them across reboots
#include "DeviceFactory.h"    // ImxDeviceFactory - constructs an ISDevice for a discovered IMX
#include "ISDevice.h"         // ISDevice - a single connected device (comms, validation, fw update)
#include "ISDisplay.h"        // cInertialSenseDisplay - optional helper to pretty-print received data

/** device_handle_t is a std::shared_ptr<ISDevice>. Global here only so the background step thread
 *  (below) can service it; a real application would pass it around explicitly. */
device_handle_t device;

/** Set when the worker thread should stop calling ISDevice::step(). */
std::atomic<bool> stop(false);

/** Optional helper for formatting device data messages for console output (used by isbDataHandler). */
cInertialSenseDisplay isDisplay(cInertialSenseDisplay::DMODE_PRETTY);

/** Firmware-update messages less severe than this are suppressed by fwUpdateCb(). Recall that a lower
 *  eLogLevel is MORE severe (ERROR=1, WARN=2, INFO=3, ...), so IS_LOG_LEVEL_INFO keeps the milestones
 *  and errors while hiding the fine-grained per-chunk progress. Raise it (e.g. IS_LOG_LEVEL_DEBUG) to
 *  see everything. */
static const eLogLevel FW_MSG_THRESHOLD = IS_LOG_LEVEL_INFO;

/**
 * Progress/status callback passed to ISDevice::updateFirmware(). The updater invokes this for every
 * step of the update (reset-to-bootloader, erase, upload progress, completion, errors, ...). We print
 * only messages at FW_MSG_THRESHOLD or more severe, so the console shows the important milestones
 * without the full chunk-by-chunk progress.
 *
 * @param obj    opaque context (the ISFirmwareUpdater*); unused in this example
 * @param level  severity of this message (eLogLevel)
 * @param fmt    printf-style format string, followed by its arguments
 */
void fwUpdateCb(const std::any& obj, eLogLevel level, const char* fmt, ...) {
    if (level > FW_MSG_THRESHOLD)   // less severe than our threshold -> skip it
        return;

    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    std::cout << "[FW] " << buf << std::endl;
}

/**
 * Optional: a handler for normal (non-firmware) data received from the device. Registering one with
 * ISDevice::registerIsbDataHandler() is how an application consumes streamed data. It is included
 * here only to show the pattern - this firmware-update example quiets the data stream (see below),
 * so it will rarely fire during the update itself.
 *
 * @param ctx  the ISDevice* that received the message (passed back so we can let it do its own parsing)
 * @param data the received data packet (data ID + payload)
 * @param port the port the data arrived on
 * @return 0 if handled (stop further processing), non-zero otherwise
 */
int isbDataHandler(void* ctx, p_data_t* data, port_handle_t port) {
    if (ctx) ((ISDevice*)ctx)->onIsbDataHandler(data, port);

    // Illustrative: this is how a received message would be displayed once parsed. Filter on the
    // data ID (DID) for the message(s) of interest and format the payload with the display helper;
    // extend this block to handle whichever DIDs your application consumes.
    if ((data->hdr.id == DID_SYS_PARAMS) || (data->hdr.id == DID_GNSS1_POS) || (data->hdr.id == DID_INS_1))
        std::cout << isDisplay.DataToString((const p_data_t*)data);
    return 0;
}

/**
 * The SDK is cooperatively scheduled: ISDevice::step() must be called continuously to service the
 * port I/O AND to advance the firmware-update state machine (including across the device's bootloader
 * reboot). We run it on a background thread so the main thread can just wait for the update to finish.
 *
 * The loop keeps running while the device is connected OR an update is in progress OR the update
 * session is still in a non-error state (getUpdateStatus() >= NOT_STARTED), so it survives the window
 * where the connection is briefly dropped/re-established during the reboot.
 */
void step_thread() {
    while (!stop.load() && (device->isConnected() || device->fwUpdateInProgress() || (device->getUpdateStatus() >= 0))) {
        device->step();
        SLEEP_MS(1);
    }
}

/**
 * Connects to the device on @p portStr, applies the firmware package @p fwPackage, and waits for the
 * update to complete, with each SDK step explained inline. This is the recommended pattern for
 * firmware update over a fixed serial port: the connection is routed through the
 * PortManager/DeviceManager so the SDK can re-discover and re-associate the device across the
 * bootloader reboot that occurs partway through the update.
 *
 * @param portStr   the serial port the device is connected to (e.g. "/dev/ttyAMA0")
 * @param fwPackage filesystem path to the firmware package (.fpkg) to apply
 * @return 0 if the firmware update completed successfully; non-zero if the device could not be found,
 *         the update could not be started, or the updater reported errors
 */
int main_explained(const char* portStr, const char* fwPackage) {

    // Route SDK log output to the console. INFO is a reasonable default; raise to
    // IS_LOG_LEVEL_DEBUG / _MORE_DEBUG to see the discovery and updater internals.
    IS_SET_LOG_LEVEL(IS_LOG_LEVEL_INFO);
    IS_LOG_OUTPUT(stdout);

    PortManager&   portManager   = PortManager::getInstance();    // both are process-wide singletons
    DeviceManager& deviceManager = DeviceManager::getInstance();

    // Tell the PortManager how to enumerate serial ports, and the DeviceManager how to build an
    // ISDevice for a discovered IMX. Neither is registered automatically, and connecting through
    // these managers (rather than binding a port handle directly) is what lets the SDK re-discover
    // and re-associate the device when it reboots into the bootloader mid-update: because the
    // DeviceManager owns the ISDevice, the updater re-attaches the same device object to the fresh
    // port that discovery produces after the reboot.
    portManager.addPortFactory((PortFactory*)&SerialPortFactory::getInstance());
    deviceManager.addDeviceFactory(&ImxDeviceFactory::getInstance());

    // IMX default serial baud. (Over a USB-CDC connection the baud is ignored; over a real UART it
    // must match the device - and the ISbl bootloader auto-negotiates baud during its handshake.)
    SerialPortFactory::getInstance().setBaudRate(BAUDRATE_921600);

    // Discover ONLY the requested port. discoverPorts() takes a regex; passing the exact port name
    // scopes enumeration (and every periodic re-scan during the update) to just this device instead
    // of opening every serial port on the host.
    portManager.discoverPorts(portStr);

    // Ask the DeviceManager to probe the discovered port(s) and construct/validate an ISDevice for
    // whatever responds (using the ImxDeviceFactory registered above). This fetches devInfo and the
    // flash configuration, so no separate validate() call is needed.
    deviceManager.discoverDevices(IS_HARDWARE_ANY, 3000);

    // Retrieve the ISDevice the DeviceManager built for our port: portManager.getPort(portStr) maps
    // the port name to its handle, and deviceManager.getDevice(handle) returns the ISDevice bound to
    // that port (nullptr if discovery found nothing there). This ISDevice is owned by the
    // DeviceManager, which is what allows it to be re-associated after the bootloader reboot.
    device = deviceManager.getDevice(portManager.getPort(portStr));
    if (!device) {
        std::cerr << "No Inertial Sense device found on " << portStr << std::endl;
        return 1;
    }
    std::cout << "Discovered device " << device->getDescription() << std::endl;

    // Start servicing the device (see step_thread). Do this before the update so the updater's
    // state machine is pumped throughout, including across the reboot.
    std::thread t(step_thread);

    // Optional: register a data handler and quiet the device's default broadcasts. Quieting the
    // stream keeps the link clean for the update; the handler is here only to show the pattern.
    device->registerIsbDataHandler(isbDataHandler);
    device->StopBroadcasts(true);

    // Build the update command list:
    //   force=true     - apply even if the device already runs this version (skip the newer-check)
    //   package=<path> - the .fpkg to apply
    std::vector<std::string> cmds;
    cmds.push_back("force=true");
    cmds.push_back(std::string("package=") + fwPackage);

    // TARGET_UNKNOWN lets the package manifest's "target:" steps decide which device(s) to update
    // (this rig is an IMX-5, but the same package can also carry IMX-6 / GPX-1 images).
    if (device->updateFirmware(fwUpdate::TARGET_UNKNOWN, cmds, fwUpdateCb, nullptr) != IS_OP_OK) {
        std::cerr << "Failed to start firmware update." << std::endl;
        stop.store(true);
        t.join();
        return 1;
    }
    std::cout << "Starting update..." << std::endl;

    // Wait for the update to complete. The step_thread services the updater; here we just wait and
    // periodically re-scan the target port so the SDK can re-associate the device after it reboots
    // into ISbl (on a fixed UART the node persists, so this scan is what drives re-discovery).
    uint32_t nextPortCheck = 0;
    while (device->fwUpdateInProgress()) {
        if (current_timeMs() > nextPortCheck) {
            portManager.discoverPorts(portStr);
            nextPortCheck = current_timeMs() + 1500;
        }
        SLEEP_MS(10);
    }

    stop.store(true);
    t.join();

    // The definitive outcome lives in the updater's state object (ISFwUpdateState), which persists on
    // the ISDevice after the updater is torn down. (getUpdateStatus() only reflects the transient
    // per-session status and reverts to NOT_STARTED at the end, so it is not a reliable completion
    // indicator.) Take a thread-safe snapshot and report from it.
    ISFwUpdateState result = device->fwUpdateState.getSnapshot();
    bool success = (result.state == ISFwUpdateState::UPDATER_SUCCESSFUL) ||
                   (result.state == ISFwUpdateState::SUCCESS_WITH_NOTIFICATIONS);
    std::cout << (success ? "Firmware update finished successfully." : "Firmware update FAILED.") << std::endl;

    // The message log is tagged per target, so for a multi-device package (e.g. an IMX + GPX chain)
    // we can report which device produced each notable message. Print anything WARN-or-more-severe
    // (recall lower eLogLevel == more severe: ERROR=1, WARN=2).
    for (const ISFwUpdateState::message& m : result.messages) {
        if (m.severity <= IS_LOG_LEVEL_WARN)
            std::cout << "  [" << (m.target.empty() ? "(host)" : m.target) << "] " << m.msg << std::endl;
    }

    return success ? 0 : 1;
}

/**
 * The same firmware-update flow as main_explained(), condensed. See main_explained() for the
 * rationale behind each call - in particular why the connection is routed through the
 * PortManager/DeviceManager rather than binding a port handle directly.
 *
 * @param portStr   the serial port the device is connected to (e.g. "/dev/ttyAMA0")
 * @param fwPackage filesystem path to the firmware package (.fpkg) to apply
 * @return 0 if the firmware update completed successfully; non-zero on failure
 */
int main_minimal(const char* portStr, const char* fwPackage) {
    PortManager&   portManager   = PortManager::getInstance();
    DeviceManager& deviceManager = DeviceManager::getInstance();
    portManager.addPortFactory((PortFactory*)&SerialPortFactory::getInstance());
    deviceManager.addDeviceFactory(&ImxDeviceFactory::getInstance());
    SerialPortFactory::getInstance().setBaudRate(BAUDRATE_921600);

    portManager.discoverPorts(portStr);                 // discover only the target port
    deviceManager.discoverDevices(IS_HARDWARE_ANY, 3000);
    device = deviceManager.getDevice(portManager.getPort(portStr));
    if (!device) {
        std::cerr << "No Inertial Sense device found on " << portStr << std::endl;
        return 1;
    }

    std::thread t(step_thread);

    std::vector<std::string> cmds = { "force=true", std::string("package=") + fwPackage };
    device->updateFirmware(fwUpdate::TARGET_UNKNOWN, cmds, fwUpdateCb, nullptr);

    uint32_t nextPortCheck = 0;
    while (device->fwUpdateInProgress()) {
        if (current_timeMs() > nextPortCheck) {         // re-scan only the target port
            portManager.discoverPorts(portStr);
            nextPortCheck = current_timeMs() + 1500;
        }
        SLEEP_MS(10);
    }

    stop.store(true);
    t.join();

    // Definitive outcome + per-target message summary (see main_explained() for details).
    ISFwUpdateState result = device->fwUpdateState.getSnapshot();
    bool success = (result.state == ISFwUpdateState::UPDATER_SUCCESSFUL) ||
                   (result.state == ISFwUpdateState::SUCCESS_WITH_NOTIFICATIONS);
    for (const ISFwUpdateState::message& m : result.messages)
        if (m.severity <= IS_LOG_LEVEL_WARN)
            std::cout << "  [" << (m.target.empty() ? "(host)" : m.target) << "] " << m.msg << std::endl;
    std::cout << (success ? "Firmware update finished successfully." : "Firmware update FAILED.") << std::endl;
    return success ? 0 : 1;
}

#define EXPLAINED

/**
 * Entry point.
 * Usage: ISDevice_FwUpdate <port> <firmware.fpkg>
 *   port          serial port the device is on (e.g. /dev/ttyAMA0, /dev/ttyUSB0, or COMx)
 *   firmware.fpkg firmware package (.fpkg) to apply
 */
int main(int argc, const char** argv) {

#if PLATFORM_IS_LINUX
    const char* portArg = "/dev/ttyAMA0";
#else
    const char* portArg = "COM1";
#endif
    if (argc > 1) portArg = argv[1];

    if (argc <= 2) {
        std::cerr << "Usage: " << argv[0] << " <port> <firmware.fpkg>" << std::endl;
        return 1;
    }
    const char* fwPackage = argv[2];

#ifdef EXPLAINED
    return main_explained(portArg, fwPackage);
#else
    return main_minimal(portArg, fwPackage);
#endif
}
