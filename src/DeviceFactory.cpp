/**
 * @file DeviceFactory.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/10/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "DeviceFactory.h"

#include "DeviceManager.h"

void DeviceFactory::locateDevices(std::function<void(DeviceFactory*, const dev_info_t&, port_handle_t)> deviceCallback, uint16_t hdwId) {
    for (auto& port : PortManager::getInstance()) {
        locateDevice(deviceCallback, port, hdwId);
    }
}

void DeviceFactory::locateDevice(std::function<void(DeviceFactory*, const dev_info_t&, port_handle_t)> deviceCallback, port_handle_t port, uint16_t hdwId) {
    if (!port || !portIsValid(port))
        return;     // TODO: Should we do anything special if the port is invalid?  Really, we should never get here with an invalid port...

    // can we open the port?
    if (!portIsOpened(port)) {
        debug_message("[DBG] Opening serial port '%s'\n", portName(port));
        if (portValidate(port) && portOpen(port) != PORT_ERROR__NONE) {
        // if (serialPortOpen(port, portName(port), SERIAL_PORT(port)->baudRate, SERIAL_PORT(port)->blocking) == 0) {
            debug_message("[DBG] Error opening serial port '%s'.  Ignoring.  Error was: %s\n", portName(port), SERIAL_PORT(port)->error);
            portClose(port);              // failed to open
            portInvalidate(port);
            return;
            // m_ignoredPorts.push_back(curPortName);     // record this port name as bad, so we don't try and reopen it again
        }
    }

    // at this point, the port should be opened...
    ISDevice* dev = DeviceManager::getInstance().getDevice(port);
    if (!dev) {
        // no previous device exists, so identify the device and then register it with the manager
        ISDevice localDev(hdwId, port);
        for (int i = 0; i < 5; i++) {
            localDev.step();
            if (localDev.hasDeviceInfo() && ((hdwId == IS_HARDWARE_ANY) || ((localDev.hdwId & hdwId) == localDev.hdwId))) {
                deviceCallback(this, localDev.devInfo, port);
                return;
            }
            SLEEP_MS(5);
        }
    } else if ((hdwId == IS_HARDWARE_ANY) || ((dev->hdwId & hdwId) == dev->hdwId)) {
        // a device exists associated with this port already, there isn't anything to do.
        printf("Rediscovered previously identified device %s on port %s.\n", dev->getIdAsString().c_str(), dev->getPortName().c_str());
        dev->validateDevice(3000);
    }
}
