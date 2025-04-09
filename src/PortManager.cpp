/**
 * @file PortManager.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/20/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "PortManager.h"

/**
 * Called by the PortFactories when a port is identified.
 * This is a low-level callback indicating that port identified by portName was detected as a
 * type of portType.  This handler is responsible for make further determinations and callbacks
 * for specifics events such as whether the port is a new port, or if an old port no longer
 * exists.  NOTE: this function will likely be called frequently, since it will be called once
 * for each port identified, for each locator register, for every call of checkForNewPorts()
 * @param portType
 * @param portName
 */
void PortManager::portHandler(PortFactory* factory, std::string portName) {
    std::pair<PortFactory*, std::string> portId(factory, portName);

    // check if port is previously known
    for (auto& kp : knownPorts) {
        if ((kp.first == portId.first) && (kp.second == portId.second)) {
            return;
        }
    }

    // if not, then do we need to allocate it?
    knownPorts.push_back(portId);
    port_handle_t port = factory->bindPort(PORT_TYPE__UART, portName);
    insert(port);

    // finally, call our handler
    for (port_listener& l : listeners) {
        l(PORT_ADDED, PORT_TYPE__UART, portName, port);
    }
}

/**
 * @return a vector of available ports
 * NOTE that this may return ports which do not have a corresponding ISDevice
 */
std::vector<port_handle_t> PortManager::getPorts() {
    std::vector<port_handle_t> ports;
    for (auto port : *this) {
        ports.push_back((port_handle_t)port);
    }
    return ports;
}

size_t PortManager::getPortCount() { return size(); }



