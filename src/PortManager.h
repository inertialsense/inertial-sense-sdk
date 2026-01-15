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

#ifndef IS_SDK__PORT_MANAGER_H
#define IS_SDK__PORT_MANAGER_H

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_set>

#include "core/base_port.h"
#include "PortFactory.h"

class PortManager : public std::set<port_handle_t> {
public:

    enum port_event_e : uint8_t {
        PORT_ADDED,
        PORT_REMOVED,
    };

    typedef std::function<void(port_event_e, uint16_t, std::string, port_handle_t, PortFactory& factory)> port_listener;
    typedef std::shared_ptr<port_listener> port_listener_handle_t;

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

    /**
     * Removes/clears all previously registered port factories from the PortManager.  No ports (of any type) will be discovered
     * if there are no factories registered.
     */
    void clearPortFactories() { factories.clear(); }

    /**
     * Sets the list of available port factories to those defined in the passed vector. Only ports managed/located by the listed
     * factories will be discovered, and any previously registered factories will be removed.
     * @param _factories a vector of pointers to factory instances.
     */
    void setPortFactories(std::vector<PortFactory*>& _factories) { factories = _factories; }

    /**
     * Registers a factory instance to the PortManager.
     * @param factory the port factory instance to register
     */
    void addPortFactory(PortFactory* factory) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        factories.push_back(factory);
    }

    /**
     * Returns a vector containing all registered port factories.
     * @return
     */
    std::vector<PortFactory*> getPortFactories() {
        return factories;
    }

    port_listener_handle_t addPortListener(const port_listener& listener) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        port_listener_handle_t listenerPtr = std::make_shared<port_listener>(listener);
        listeners.insert(listenerPtr);
        return listenerPtr;
    }

    bool removePortListener(const port_listener_handle_t& listener) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        bool didIt = (listeners.erase(listener) != 0);
        return didIt;
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

    port_handle_t getPort(uint16_t index);
    inline port_handle_t operator[](int index) { return getPort(index); }

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
        std::lock_guard<std::recursive_mutex> lock(mutex);
        for (auto& [portEntry, knownPort] : knownPorts ) {
            if ((port == knownPort) && portEntry.factory) {
                portEntry.factory->releasePort(port);
                return true;
            }
        }
        return false;
    }

    void clear(){
        std::set<port_handle_t>::clear();
        knownPorts.clear();
    }

protected:
    PortManager() = default;
    ~PortManager() {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        factories.clear(); // Note that all factories should be pointers to static instances, so we shouldn't ever delete them
        // if the PortManager is destroyed, it should destroy all ports which it knows about
        for (auto [entry, port] : knownPorts) {
            portClose(port);
            if (entry.factory)
                entry.factory->releasePort(port);
        }
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

        port_entry_t(PortFactory* f, uint16_t t, const std::string& n) { // , port_handle_t* p) {
            factory = f, type = t, name = n; // , port = p;
        }
        bool operator< (port_entry_t const& op) const { return name.compare(op.name) < 0; }
        bool operator== (port_entry_t const& op) const { return name == op.name; }
    };

    std::vector<PortFactory*> factories;                             //!< list of port factories responsible for detecting, allocating and freeing ports of different types.
    std::unordered_set<port_listener_handle_t > listeners;           //!< list of listeners who should be notified when ports are discovered, lost, opened, closed, etc
    std::map<port_entry_t, port_handle_t> knownPorts;                //!< a map previously discovered ports keyed on factory + name (some string identifier)
    bool portsChanged = false;                                       //!< a flag indicating (true) that list of managed ports has changed, either ports added or removed during the last call to discoverPorts()

    mutable std::recursive_mutex mutex;                                        // Mutex must be mutable if the range needs to support const containers

    class LockedRangeProxy {
    private:
        PortManager& container;
        std::scoped_lock<std::recursive_mutex> lock_guard;                     // The lock_guard/scoped_lock ensures RAII

    public:
        LockedRangeProxy(PortManager& container_ref) : container(container_ref), lock_guard(container_ref.mutex) { }

        // The destructor will be called automatically when the for loop ends,
        // releasing the lock_guard and thus the mutex.
        ~LockedRangeProxy() = default;

        // Provide begin and end iterators to the underlying data
        auto begin() { return container.begin(); }
        auto end() { return container.end(); }
        auto begin() const { return container.begin(); }
        auto end() const { return container.end(); }
    };

public:
    LockedRangeProxy locked_range() { return LockedRangeProxy(*this); }

};


#endif //IS_SDK__PORT_MANAGER_H
