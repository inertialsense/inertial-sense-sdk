/**
 * @file PortManager.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/20/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <util.h>
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
    FnProfiler fn("PortManager::discoverPorts", 10000); // this can take a long time, but it generally should be fast

    std::lock_guard<std::recursive_mutex> lock(mutex);
    fn.mark("Got mutex.");
    portsChanged = false;   // always clear this flag every time we call discoverPorts - the process will set it back, if needed.

    // Use the erase-remove idiom to clean up lost ports
    for (auto it = knownPorts.begin(); it != knownPorts.end(); ) {
        auto& entry = it->first;
        auto& port = it->second;

        if (!portIsValid(port) || !entry.factory->validatePort(entry.name, entry.type)) {
            erase(port);
            for (auto& listener : listeners) {
                (*listener)(PORT_REMOVED, portType(port), entry.name, port, *entry.factory);
            }
            entry.factory->releasePort(port);
            it = knownPorts.erase(it);
            portsChanged = true;
        } else {
            ++it;
        }
    }
    fn.mark("Removed stale ports.");

    // now look for new ports
    for (auto factory : factories) {
        auto cb = std::bind(&PortManager::portHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        factory->locatePorts(cb, pattern, pType);
    }
    fn.mark("Added new ports.");

/*
    // check to make sure all knownPorts are also representing in the top-level PortManager's set
    for (auto& [entry, port] : knownPorts ) {
        if (std::find_if(begin(), end(), [&](port_handle_t p){ return p == port; }) == end()) { // C++17 compliant, since we can't used set::contains()
            insert(port);
            portsChanged = true;   // note that we added/updated the list of ports
        }
    }
*/
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
    std::lock_guard<std::recursive_mutex> lock(mutex);
    port_entry_t portEntry(factory, portType, portName);

    // check if port is previously known
    for (auto& [entry, port] : knownPorts) {
        if ((entry.factory == portEntry.factory) && (entry.name == portEntry.name)) {
            if (port && portIsValid(port))
                return; // this is a previously known/discovered port that is still valid
            else {
                // the port was previously identified, but the port handle is invalid.
                // we probably should release to port and reallocate a new one
                // finally, call our handler
                for (auto& listener : listeners) {
                    (*listener)(PORT_REMOVED, entry.type, entry.name, port, *entry.factory);
                }
                entry.factory->releasePort(port);
                port = nullptr;
                break;
            }
        }
    }

    // Alias-aware safety net: check if any existing known port has the same name
    // (regardless of factory). This catches cases where the same tcp://host:port URL
    // is emitted by multiple factories or multiple iterations of the same factory
    // (e.g., dual-stack mDNS producing alias URLs that resolve to the same endpoint).
    for (auto& [entry, existingPort] : knownPorts) {
        if (entry.name == portName && existingPort && portIsValid(existingPort)) {
            // Skip duplicate port allocation, but give the new factory a chance to do
            // post-bind decoration on the existing port (e.g. RelayPortFactory seeding
            // a device hint into DeviceManager). Factories that don't override
            // onPortAlias() default to a no-op.
            factory->onPortAlias(existingPort, portName, portType);
            return; // already bound under a different factory — don't duplicate
        }
    }

    // if not, then do we need to allocate it?
    port_handle_t port = factory->bindPort(portName, portType);
    if (port) {
        knownPorts[portEntry] = port;
        insert(port);

        // finally, call our handler
        for (auto& listener : listeners) {
            (*listener)(PORT_ADDED, portType, portName, port, *factory);
        }

        portsChanged = true;
    }
}

/**
 * Returns a vector of all currently discovered/managed ports
 * @return
 */
std::vector<port_handle_t> PortManager::getPorts() {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    std::vector<port_handle_t> ports;
    for (auto port : *this) {
        ports.push_back((port_handle_t)port);
    }
    return ports;
}

/**
 * Attempts to locate and return a previously discovered/managed port by its name, and optionally port type flags
 * @param name the name of the port to locate and return
 * @param portType an optional bitmask indicating the port type to match
 * @return the port handle if found, otherwise returns NULL
 */
port_handle_t PortManager::getPort(const std::string& name, uint16_t portType) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    for (auto& [entry, port] : knownPorts) {
        if (((entry.type & portType) == entry.type) && (entry.name == name))
            return port;
    }
    return NULLPTR;
}


port_handle_t PortManager::getPort(uint16_t idx) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    for (auto iter = begin(); iter != end(); iter++, idx--  ) {
        if (idx == 0)
            return *iter;
    }
    return (port_handle_t)nullptr;
}
