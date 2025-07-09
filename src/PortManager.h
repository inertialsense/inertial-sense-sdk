/**
 * @file PortManager.h 
 * @brief Handles the discovery and maintenance of all available ports
 *
 * This class ONLY manages ports, it does not make any assumptions about any possible devices connected to those ports.
 * Since ports are abstract notions in the SDK, this class attempts to treat them indifferently by abstracting some of
 * the unique behaviors out to specific port implementations; for example, TCP, UDP, USB/Serial all have different
 * underlying discovery and connection mechanisms; some of this (such as open/close) are handle by the port itself, but
 * discovery and enumeration is more nuanced.
 *
 * @author Kyle Mallory on 2/20/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef EVALTOOL_PORTMANAGER_H
#define EVALTOOL_PORTMANAGER_H

#include <unordered_set>
#include <map>
#include <functional>
#include <string>

#include "core/base_port.h"

#include "PortFactory.h"


class PortManager : public std::unordered_set<port_handle_t> {
public:
    typedef std::function<void(uint8_t, uint16_t, std::string, port_handle_t)> port_listener;

    enum port_event_e : uint8_t {
        PORT_ADDED,
        PORT_REMOVED,
    };

    static PortManager& getInstance() {
        static PortManager instance;
        return instance;
    }

    /**
     * Queries all factories to identify and enumerate all ports which can be discovered by all registered factories
     * Note that only newly discovered ports which have not been previously discovered will trigger a port_listener callback.
     * @param pattern a regex name pattern; any discovered port which matches this regex will be discovered, default pattern
     *  will match all ports.
     * @param pType a PORT_TYPE__ value indicating that only ports matching the specified type will be discovered, default
     *  value of PORT_TYPE__UNKNOWN will match all port types
     * @returns true if one or more ports were added or removed from the list of managed ports
     */
    bool discoverPorts(const std::string& pattern = "(.+)", uint16_t pType = PORT_TYPE__UNKNOWN);

    void addPortFactory(PortFactory* pl) {
        factories.push_back(pl);
    };

    void addPortListener(const port_listener& listener) {
        listeners.push_back(listener);
    }

    void removePortListener(port_listener listener) {
        // FIXME: this needs to be implemented correctly
/*
        for (auto it = listeners.begin(); it != listeners.end(); it++) {
            if (*it == listener) {
                listeners.erase(listener);
                break;
            }
        }
*/
    }

    /**
     * Returns the number of currently discovered/managed ports
     * @return the number of currently discovered/managed ports
     */
    size_t getPortCount() { return size(); }

    /**
     * Returns a vector of all currently discovered/managed ports
     * @return
     */
    std::vector<port_handle_t> getPorts();

    /**
     * Attempts to locate and return a previously discovered/managed port by its name, and optionally port type flags
     * @param name the name of the port to locate and return
     * @param portType an optional bitmask indicating the port type to match
     * @return the port handle if found, otherwise returns NULL
     */
    port_handle_t getPort(const std::string& name, uint16_t portType = PORT_TYPE__UNKNOWN);

    /**
     * Release the requested port, deallocating any associated memory
     */
    bool releasePort(port_handle_t port) {
        for (auto& [portEntry, knownPort] : knownPorts ) {
            if ((port == knownPort) && portEntry.factory) {
                portEntry.factory->releasePort(port);
                return true;
            }
        }
        return false;
    }

    void clear() {
        std::unordered_set<port_handle_t>::clear();
        knownPorts.clear();
    }

protected:
    PortManager() = default;
    ~PortManager() {
        factories.clear(); // Note that all factories should be pointers to static instances, so we shouldn't ever delete them
    };

    /**
     * Callback handler used by factories when a port is located (but not yet allocated)
     * @param factory - the factory which discovered this port
     * @oaram portType - the type of port (as determined by the factory)
     * @param portName - the name of the port (as determined by the factory, should be unique)
     */
    void portHandler(PortFactory* factory, uint16_t portType, const std::string& portName);

private:
    PortManager(PortManager const &) = delete;
    PortManager& operator=(PortManager const&) = delete;

    struct port_entry_t {
        PortFactory* factory;
        uint16_t type;
        std::string name;
        // port_handle_t port;

        port_entry_t(PortFactory* f, uint16_t t, const std::string& n) { // , port_handle_t* p) {
            factory = f, type = t, name = n; // , port = p;
        }
        bool operator< (port_entry_t const& op) const { return name.compare(op.name) < 0; }
        bool operator== (port_entry_t const& op) const { return name == op.name; }
    };

    std::vector<PortFactory*> factories;                         //!< list of port factories responsible for detecting, allocating and freeing ports of different types.
    std::vector<port_listener> listeners;                        //!< list of listeners who should be notified when ports are discovered, lost, opened, closed, etc
    std::map<port_entry_t, port_handle_t> knownPorts;            //!< a map previously discovered ports keyed on factory + name (some string identifier)
    bool portsChanged = false;                                   //!< a flag indicating (true) that list of managed ports has changed, either ports added or removed during the last call to discoverPorts()

};


#endif //EVALTOOL_PORTMANAGER_H
