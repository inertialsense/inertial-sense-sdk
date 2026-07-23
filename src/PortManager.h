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

/** @brief Singleton owner of the discovered/managed port set; also implements std::set<port_handle_t> so it can be iterated directly. */
class PortManager : public std::set<port_handle_t> {
public:

    /** @brief Port-set change events delivered to registered port_listener callbacks. */
    enum port_event_e : uint8_t {
        PORT_ADDED,      //!< A new port was discovered and added to the managed set
        PORT_REMOVED,    //!< A previously managed port was removed (no longer discoverable)
    };

    inline static const char* port_event_names[] = { "PORT_ADDED", "PORT_REMOVED" };  //!< Human-readable names for port_event_e, indexed by enum value

    /** Callback signature: (event, portType, portName, port, factory). */
    typedef std::function<void(port_event_e, uint16_t, std::string, port_handle_t, PortFactory& factory)> port_listener;
    /** Opaque handle returned by addPortListener(), used to unregister via removePortListener(). */
    typedef std::shared_ptr<port_listener> port_listener_handle_t;

    /**
     * @brief Gets the singleton instance of the PortManager.
     * @return reference to the singleton PortManager.
     */
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
     * @return a vector of pointers to the currently registered port factories.
     */
    std::vector<PortFactory*> getPortFactories() {
        return factories;
    }

    /**
     * @brief Registers a callback to be notified when ports are added or removed.
     * @param listener the callback to invoke on port_event_e changes.
     * @return an opaque handle to the registered listener, used to unregister via removePortListener().
     */
    port_listener_handle_t addPortListener(const port_listener& listener) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        port_listener_handle_t listenerPtr = std::make_shared<port_listener>(listener);
        listeners.insert(listenerPtr);
        return listenerPtr;
    }

    /**
     * @brief Unregisters a previously registered port listener.
     * @param listener the handle returned by addPortListener().
     * @return true if the listener was found and removed, false otherwise.
     */
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
     * @return a vector of all currently discovered/managed port handles.
     */
    std::vector<port_handle_t> getPorts();

    /**
     * @brief Gets a managed port by its index in the set.
     * @param index the index of the port to retrieve.
     * @return the port handle at that index, or NULL if index is out of range.
     */
    port_handle_t getPort(uint16_t index);

    /** @brief Equivalent to getPort(index); provided for convenient array-style access. */
    inline port_handle_t operator[](int index) { return getPort(index); }

    /**
     * Attempts to locate and return a previously discovered/managed port by its name, and optionally port type flags
     * @param name the name of the port to locate and return
     * @param portType an optional bitmask indicating the port type to match
     * @return the port handle if found, otherwise returns NULL
     */
    port_handle_t getPort(const std::string& name, uint16_t portType = PORT_TYPE__UNKNOWN);

    /**
     * @brief Release the requested port, deallocating any associated memory.
     * @param port the port to release.
     * @return true if port was found among the known ports and released, false otherwise.
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

    /** @brief Removes all managed ports (and their factory-tracking entries) without closing or releasing them. */
    void clear(){
        std::set<port_handle_t>::clear();
        knownPorts.clear();
    }

protected:
    PortManager() = default;

    /** @brief Closes and releases every still-valid known port before the singleton is destroyed. */
    ~PortManager() {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        // if the PortManager is destroyed, it should destroy all ports which it knows about
        for (auto& [entry, port] : knownPorts) {
            if (portIsValid(port)) {
                portClose(port);
                if (entry.factory)
                    entry.factory->releasePort(port);
            }
        }
        factories.clear(); // Note that all factories should be pointers to static instances, so we shouldn't ever delete them
    };

    /**
     * @brief Callback handler used by factories when a port is located (but not yet allocated).
     * @param factory the factory which discovered this port.
     * @param portType the type of port (as determined by the factory).
     * @param portName the name of the port (as determined by the factory, should be unique).
     */
    void portHandler(PortFactory* factory, uint16_t portType, const std::string& portName);

private:
    PortManager(PortManager const &) = delete;
    PortManager& operator=(PortManager const&) = delete;

    /** @brief Tracks which factory discovered a given port, under what type/name, so it can be located and released later. */
    struct port_entry_t {
        PortFactory* factory;    //!< Factory that discovered this port
        uint16_t type;           //!< Port type (a PORT_TYPE__ value) as determined by the factory
        std::string name;        //!< Unique port name, as determined by the factory

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

    /**
     * @brief RAII range-adapter that holds the PortManager's mutex for the lifetime of a
     * range-based for loop, so iteration over the managed port set is safe against concurrent
     * modification from other threads. See locked_range().
     */
    class LockedRangeProxy {
    private:
        PortManager& container;
        std::scoped_lock<std::recursive_mutex> lock_guard;                     // The lock_guard/scoped_lock ensures RAII

    public:
        /**
         * @brief Locks container's mutex for the lifetime of this proxy.
         * @param container_ref the PortManager whose port set will be iterated.
         */
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
    /**
     * @brief Get a range-adapter over the managed port set that holds the internal mutex for the
     * duration of iteration, e.g. `for (auto& port : portManager.locked_range()) { ... }`.
     * @return a LockedRangeProxy usable directly in a range-based for loop.
     */
    LockedRangeProxy locked_range() { return LockedRangeProxy(*this); }

};


#endif //IS_SDK__PORT_MANAGER_H
