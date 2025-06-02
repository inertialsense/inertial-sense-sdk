/**
 * @file PortManager.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/20/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "PortManager.h"

/**
 * Queries all factories to identify and enumerate all ports which can be discovered by all registered factories
 * Note that only newly discovered ports which have not been previously discovered will trigger a port_listener callback.
 * @param pattern a regex name pattern; any discovered port which matches this regex will be discovered, default pattern
 *  will match all ports.
 * @param pType a PORT_TYPE__ value indicating that only ports matching the specified type will be discovered, default
 *  value of PORT_TYPE__UNKNOWN will match all port types
 */
bool PortManager::discoverPorts(const std::string& pattern, uint16_t pType) {
    portsChanged = false;   // always clear this flag every time we call discoverPorts - the process will set it back, if needed.

    // look for ports which are no longer valid and remove them
    std::vector<const port_entry_t*> lostPorts; // a vector of ports which no longer are available and need to be cleaned up
    for (auto& [entry, port] : knownPorts) {
        bool invalid = !(portIsValid(port) && entry.factory->validatePort(entry.type, entry.name));

        // check if port still exists...
        if (invalid) {
            erase(port);    // remove the port from our primary set of ports
            lostPorts.push_back(&entry);
            // notify listeners before we actually invalidate the port
            for (port_listener& listener : listeners) {
                listener(PORT_REMOVED, portType(port), entry.name, port);
            }
            entry.factory->releasePort(port);
            port = nullptr;
            portsChanged = true;   // note that we removed/update the list of ports
        }
    }
    for (auto entry : lostPorts) knownPorts.erase(*entry);

    // now look for new ports
    for (auto factory : factories) {
        auto cb = std::bind(&PortManager::portHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        factory->locatePorts(cb, pattern, pType);
    }

    // check to make sure all knownPorts are also representing in the top-level PortManager's set
    for (auto& [entry, port] : knownPorts ) {
        if (!this->contains(port)) {
            this->insert(port);
        }
    }
    return portsChanged;
}

/**
 * Called by individual PortFactories when a port is identified.
 * This is a low-level callback indicating that a port identified by portName was detected as a
 * type of portType.  This handler is responsible for making further determinations and callbacks
 * for specifics events such as whether the port is a new port, or if an old port no longer
 * exists.  NOTE: this function will likely be called frequently, since it will be called once
 * for each port identified, for each locator register, for every call of checkForNewPorts()
 * @param portType
 * @param portName
 */
void PortManager::portHandler(PortFactory* factory, uint16_t portType, const std::string& portName) {
    port_entry_t portEntry(factory, portType, portName);

    // check if port is previously known
    for (auto& [entry, port] : knownPorts) {
        if ((entry.factory == portEntry.factory) && (entry.name == portEntry.name)) {
            if (port && portIsValid(port))
                return; // this is a previously known/discovered port that is still valid
            else {
                // the port was previously identified, but the port handle is invalid.
                // we probably should release to port and reallocate a new one
                entry.factory->releasePort(port);
                port = nullptr;
                break;
            }
        }
    }

    // if not, then do we need to allocate it?
    port_handle_t port = factory->bindPort(portType, portName);
    knownPorts[portEntry] = port;
    insert(port);

    // finally, call our handler
    for (port_listener& l : listeners) {
        l(PORT_ADDED, portType, portName, port);
    }

    portsChanged = true;
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



