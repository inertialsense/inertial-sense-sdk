/**
 * @file DeviceFactory.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/10/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "DeviceFactory.h"

void DeviceFactory::locateDevices(std::function<void(DeviceFactory*, const dev_info_t&, port_handle_t)> deviceCallback, const PortManager& portManager, uint16_t hdwId) {

    for (auto& port : portManager) {
        // can we open the port?
        if (portIsValid(port) && !serialPortIsOpen(port)) {
            debug_message("[DBG] Opening serial port '%s'\n", portName(port));
            if (serialPortOpen(port, portName(port), SERIAL_PORT(port)->baudRate, SERIAL_PORT(port)->blocking) == 0) {
                debug_message("[DBG] Error opening serial port '%s'.  Ignoring.  Error was: %s\n", portName(port), SERIAL_PORT(port)->error);
                serialPortClose(port);              // failed to open
                portInvalidate(port);
                continue;                           // don't try to do anything more with this port...
                // m_ignoredPorts.push_back(curPortName);     // record this port name as bad, so we don't try and reopen it again
            }
        }

        // at this point, the port should be opened...
        ISDevice localDev(hdwId, port);
        for (int i = 0; i < 5; i++) {
            localDev.step();
            if (localDev.validateDeviceAlt() == true) {
                deviceCallback(this, localDev.devInfo, port);
                return;
            }
            SLEEP_MS(5);
        }
    }
}

void DeviceFactory::locateDevice(std::function<void(DeviceFactory*, const dev_info_t&, port_handle_t)> deviceCallback, port_handle_t port, uint16_t hdwId) {
    // can we open the port?
    if (portIsValid(port) && !serialPortIsOpen(port)) {
        debug_message("[DBG] Opening serial port '%s'\n", portName(port));
        if (serialPortOpen(port, portName(port), SERIAL_PORT(port)->baudRate, SERIAL_PORT(port)->blocking) == 0) {
            debug_message("[DBG] Error opening serial port '%s'.  Ignoring.  Error was: %s\n", portName(port), SERIAL_PORT(port)->error);
            serialPortClose(port);              // failed to open
            portInvalidate(port);
            return;
            // m_ignoredPorts.push_back(curPortName);     // record this port name as bad, so we don't try and reopen it again
        }
    }

    // at this point, the port should be opened...
    ISDevice localDev(hdwId, port);
    localDev.step();
    if (localDev.validateDeviceAlt() == true)
        deviceCallback(this, localDev.devInfo, port);
}
