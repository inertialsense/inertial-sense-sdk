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
#include <functional>
#include <string>

#include "core/types.h"

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
     * will match all ports.
     * @param pType a PORT_TYPE__ value indicating that only ports matching the specified type will be discovered, default
     * value of PORT_TYPE__UNKNOWN will match all port types
     */
    void discoverPorts(const std::string& pattern = "(.+)", uint16_t pType = PORT_TYPE__UNKNOWN) {
        for (auto l : factories) {
            auto cb = std::bind(&PortManager::portHandler, this, std::placeholders::_1, std::placeholders::_2);
            l->locatePorts(cb, pattern, pType);
        }
    }

    void addPortFactory(PortFactory* pl) {
        factories.push_back(pl);
    };

    void addPortListener(const port_listener& listener) {
        listeners.push_back(listener);
    }

    void removePortListener(port_listener listener) {
/*
        for (auto it = listeners.begin(); it != listeners.end(); it++) {
            if (*it == listener) {
                listeners.erase(listener);
                break;
            }
        }
*/
    }

    std::vector<port_handle_t> getPorts();

    size_t getPortCount();

    /**
     * Release the requested port, deallocating any associated memory
     */
    void releasePort(port_handle_t port) {
        for (auto f : factories )
            if (f->releasePort(port))
                return;
    }

protected:
    PortManager() = default;
    ~PortManager() {
        factories.clear(); // Note that all factories should be pointers to static instances, so we shouldn't ever delete them
    };

    /**
     * Callback handler used by factories when a port is located (but not yet allocated)
     * @param factory - the factory which discovered this port
     * @param portName - the name of the port (as determined by the factory, should be unique)
     */
    void portHandler(PortFactory* factory, std::string portName);

    void checkForNewPorts() {
        for (auto l : factories) {
            auto cb = std::bind(&PortManager::portHandler, this, std::placeholders::_1, std::placeholders::_2);
            l->locatePorts(cb);
        }
    }


private:
    PortManager(PortManager const &) = delete;
    PortManager& operator=(PortManager const&) = delete;

    std::vector<PortFactory*> factories;                             //!< list of port factories responsible for detecting, allocating and freeing ports of different types.
    std::vector<port_listener> listeners;                            //!< list of listeners who should be notified when ports are discovered, lost, opened, closed, etc
    std::vector<std::pair<PortFactory*, std::string>> knownPorts;    //!< list of previously discovered port names with associated factory and name - different than actual, allocated port handles

};


#endif //EVALTOOL_PORTMANAGER_H
