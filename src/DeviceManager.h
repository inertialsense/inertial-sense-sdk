/**
 * @file DeviceManager.h 
 * @brief A singleton which maintains and manages a collection of ISDevice instances for discovered devices.
 * This class does NOT manage ports, nor is it responsible for monitoring ports to determine when devices are
 * instanced.  Rather, its the implementation that should call into the DeviceManager when a new port is
 * discovered or lost/disconnected.  The device manager also handles communications which are intended for
 * all connected devices; ie, sending the same data/commands to all known, connected devices.
 *
 * NOTE that the DeviceManager is NOT required to use the SDK. If you know you are always communicating with
 * a single Device, you can just create a single ISDevice instance and bind it to the appropriate port. The
 * DeviceManager is for managing the possibility of multiple devices.
 *
 * @author Kyle Mallory on 3/4/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__DEVICE_MANAGER_H
#define IS_SDK__DEVICE_MANAGER_H

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <functional>
#include <unordered_set>

#include "core/msg_logger.h"

#include "ISDevice.h"
#include "PortManager.h"
#include "DeviceFactory.h"

typedef device_handle_t(*pfnOnNewDeviceHandler)(port_handle_t port, const dev_info_t& devInfo);   //!< Custom-allocator callback signature: given a port and its discovered dev_info_t, return a newly allocated device (or subclass)

typedef device_handle_t(*pfnOnCloneDeviceHandler)(const ISDevice& orig);   //!< Custom-clone callback signature: return a newly allocated copy of orig

typedef std::function<void(uint8_t, device_handle_t)> device_listener;    //!< Callback signature for addDeviceListener(): receives (device_event_e, device)
typedef std::shared_ptr<device_listener> device_listener_handle_t;        //!< Opaque handle returned by addDeviceListener(), used to unregister via removeDeviceListener()

// typedef void(*pfnStepLogFunction)(void* ctx, const p_data_t* data, port_handle_t port);
typedef std::function<void(void* ctx, p_data_t* data, port_handle_t port)> pfnHandleBinaryData;   //!< Callback signature for custom binary-data handling: (user context, parsed data, source port)


/** @brief Singleton owner of the discovered/managed device set; also implements std::list<device_handle_t> so it can be iterated directly. */
class DeviceManager : public std::list<device_handle_t>
{
public:
    /** @brief Device lifecycle events delivered to registered device_listener callbacks. */
    enum device_event_e : uint8_t {
        DEVICE_ADDED,                   //!< a previously unknown device was discovered and added to the manager
        DEVICE_PORT_BOUND,              //!< a previously known device had a new port bound to it
        DEVICE_CONNECTED,               //!< a previously known, but disconnected device was connected
        DEVICE_INFO_CHANGED,            //!< a previously known device received updated information about itself
        DEVICE_DISCONNECTED,            //!< a previously known device was disconnected
        DEVICE_PORT_LOST,               //!< a previously known device's port was marked invalid or no longer exists
        DEVICE_REMOVED,                 //!< a previously known device was removed from the manager
    };

    inline static const char* device_event_names[] = { "DEVICE_ADDED", "DEVICE_PORT_BOUND", "DEVICE_CONNECTED", "DEVICE_INFO_CHANGED", "DEVICE_DISCONNECTED", "DEVICE_PORT_LOST", "DEVICE_REMOVED" };  //!< Human-readable names for device_event_e, indexed by enum value

    static const uint16_t OPTIONS_USE_DEFAULTS                    = 0xFFFF;       //!< used to indicate that higher-order options, if set should be used
    static const uint16_t DISCOVERY__IGNORE_CLOSED_PORTS          = 0x0001;       //!< when set, this will cause closed ports to be skipped/ignored, otherwise open the port before attempting discovery
    static const uint16_t DISCOVERY__CLOSE_PORT_ON_FAILURE        = 0x0002;       //!< when set, this will cause the port to be closed when discovery fails, otherwise the port will be left open
    static const uint16_t DISCOVERY__CLOSE_PORT_ON_COMPLETION     = 0x0004;       //!< when set, this will cause the port to be closed once discovery is completed, regardless of failure.
    static const uint16_t DISCOVERY__FORCE_REVALIDATION           = 0x0010;       //!< when set, if an existing device entry if found for this port, the device will be forced to validate again.

    static const uint16_t DISCOVERY__DEFAULTS                     = DISCOVERY__CLOSE_PORT_ON_FAILURE; // (DISCOVERY__IGNORE_CLOSED_PORTS | DISCOVERY__CLOSE_PORT_ON_FAILURE);

    static const uint32_t DISCOVERY__DEFAULT_TIMEOUT               = 3000;         //!< default timeout (ms) for device discovery; matches DeviceFactory::deviceTimeout

    DeviceManager(DeviceManager const &) = delete;
    DeviceManager& operator=(DeviceManager const&) = delete;

    static DeviceManager& getInstance() {
        static DeviceManager instance;
        return instance;
    }

    /**
     * Concurrently validates all currently known PortManager ports against all registered factories:
     * each port is probed by each factory in round-robin, so every factory gets fair time to validate,
     * and the first factory to successfully validate on a port wins that port and allocates the device.
     * Total discovery time is bounded by max(factory timeout) rather than (num_ports * num_factories * timeout).
     * Note that only newly discovered devices which have not been previously discovered will trigger a device_listener callback.
     * This function does not return a value, and provides no direct indication that any devices were successfully discovered.
     * @param hdwId a IS_HARDWARE_* type used to restrict the discovery to only matching device types, default value of IS_HARDWARE_ANY
     *  will match all device types.
     * @param timeoutMs the number of milliseconds to allow for each device to complete discovery before failing.
     * @param options a bitmask of misc options that will can affect the discovery behavior.
     * @return true if one more more devices were discovered, otherwise false
     */
    bool discoverDevices(uint16_t hdwId = IS_HARDWARE_ANY, uint32_t timeoutMs = 0, uint32_t options = OPTIONS_USE_DEFAULTS);

    /**
     * Iterates through all registered factories attempting to identify a discoverable device on the specified port.
     * Note that only newly discovered devices which have not been previously discovered will trigger a device_listener callback.
     * @param port a port_handle_t which is queried for a viable device. If port is invalid or closed, no attempt will be made to
     *  discover a device, and will immediately return false.
     * @param hdwId a IS_HARDWARE_* type used to restrict the discovery to only matching device types, default value of IS_HARDWARE_ANY
     *  will match all device types.
     * @param timeoutMs the number of milliseconds to wait for a device to respond before failing
     * @param options a bitmask of DISCOVERY__* options; defaults to this manager's managementOptions if OPTIONS_USE_DEFAULTS
     * @return true if a device was discovered on the specified port, otherwise false
     */
    bool discoverDevice(port_handle_t port, uint16_t hdwId = IS_HARDWARE_ANY, uint32_t timeoutMs = 0, uint32_t options = OPTIONS_USE_DEFAULTS) {
        options = (options != OPTIONS_USE_DEFAULTS) ? options : managementOptions;
        options = (options == OPTIONS_USE_DEFAULTS) ? DISCOVERY__DEFAULTS : options;

        if (!portIsValid(port))
            return false;

        // Narrow the manager lock to only the parts that touch knownDevices/factories.
        // factory->locateDevice() below can block up to timeoutMs doing port I/O and
        // invokes user packet callbacks; holding the manager mutex across it causes
        // AB-BA deadlock with locks taken by those callbacks (SN-8057).
        std::vector<DeviceFactory*> factoriesSnapshot;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex);

            // first check if this port is already associated with another device
            for (auto d: *this) {
                if (d && d->hasDeviceInfo() && (d->port == port)) {
                    if ((options & DISCOVERY__FORCE_REVALIDATION)) {
                        // Invalidate the device, which will force a rediscovery, without losing the identity of the device itself.
                        d->devInfo.hdwRunState = HDW_STATE_UNKNOWN;
                        memset(d->devInfo.firmwareVer, 0, sizeof(d->devInfo.firmwareVer));
                    } else {
                        if (options & DISCOVERY__CLOSE_PORT_ON_COMPLETION)
                            portClose(port);
                        return true;    // this is a success (rediscovered an existing device), without a FORCE_VALIDATION
                    }
                }
            }

            factoriesSnapshot = factories;
        }

        // open the port, if needed - if we can't open it, fail.
        if ( (!portIsOpened(port) && (options & DISCOVERY__IGNORE_CLOSED_PORTS)) ||
             (portOpen(port) != PORT_ERROR__NONE) )
            return false;


        for (auto l : factoriesSnapshot) {
            std::function<bool(DeviceFactory *, const dev_info_t &, port_handle_t)> cb = std::bind(&DeviceManager::deviceHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, options);
            if (l->locateDevice(cb, port, hdwId, timeoutMs)) {
                if (options & DISCOVERY__CLOSE_PORT_ON_COMPLETION)  // locateDevice should already handle this, but just in case.
                    portClose(port);
                return true;    // successfully located/allocated a device
            }
        }
        return false;
    }

    /**
     * Tests if a given ISDevice pointer references a known ISDevice instance
     * @param device a pointer to a device instance
     * @return true if the device is a known device, otherwise false
     */
    bool contains(device_handle_t device) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        for (auto kd : knownDevices) {
            if (kd.device == device)
                return true;
        }
        return false;
    }

    /**
     * Removed all previously registered DeviceFactories
     */
    void clearDeviceFactories() { std::lock_guard<std::recursive_mutex> lock(mutex); factories.clear(); }

    /**
     * Adds a custom DeviceFactory to be used when new Devices are discovered
     * @param df a pointer to the DeviceFactory instance
     */
    void addDeviceFactory(DeviceFactory* df) { std::lock_guard<std::recursive_mutex> lock(mutex); factories.push_back(df); };

    /**
     * Gets all registered DeviceFactories
     * @return A vector of DeviceFactory Instances
     */
    std::vector<DeviceFactory*> getDeviceFactories() { std::lock_guard<std::recursive_mutex> lock(mutex); return factories; }

    /**
     * Convenience function that clears any existing registered DeviceFactories, and adds a new custom DeviceFactory to be used when new Devices are discovered.
     *   Calls clearDeviceFactories() followed by addDeviceFactory()
     * @param df a pointer to the DeviceFactory instance
     */
    void setDeviceFactory(DeviceFactory* df) { clearDeviceFactories(); addDeviceFactory(df); };

    /**
     * Registers a device_listener to be notified of device_event_e events (added, connected,
     * disconnected, removed, etc). Multiple listeners may be registered concurrently.
     * @param listener the callback to register
     * @return a handle identifying this registration, to be passed to removeDeviceListener() later
     */
    device_listener_handle_t addDeviceListener(const device_listener& listener) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        device_listener_handle_t listenerPtr = std::make_shared<device_listener>(listener);
        listeners.insert(listenerPtr);
        return listenerPtr;
    }

    /**
     * Removes a previously registered device listener.
     * @param listener the handle returned by addDeviceListener()
     * @return true if the listener was found and removed, otherwise false
     */
    bool removeDeviceListener(const device_listener_handle_t& listener) {
        std::lock_guard<std::recursive_mutex> lock(mutex);
        return (listeners.erase(listener) != 0);
    }

    /**
     * Seed a device hint for a port, allowing device validation to skip the DID_DEV_INFO probe.
     * Called by RelayPortFactory (and potentially other factories) when the relay has already
     * identified the device's serial number, hardware type, firmware version, etc.
     * The hint is keyed on port_handle_t and is valid for the lifetime of that port handle.
     * @param port the port handle (from bindPort) to associate the hint with
     * @param hint the relay-authoritative device info
     */
    void seedDeviceHint(port_handle_t port, const dev_info_t& hint);

    /**
     * Retrieve a previously seeded device hint for a port, or nullptr if no hint exists.
     * @param port the port handle whose hint should be retrieved
     * @return pointer to the seeded dev_info_t, or nullptr if no hint was seeded for port
     */
    const dev_info_t* getDeviceHint(port_handle_t port) const;

    /**
     * Remove a previously seeded device hint.
     * @param port the port handle whose hint should be removed
     */
    void clearDeviceHint(port_handle_t port);

    /**
     * Notifies all listeners of a particular device event.
     *
     * Listeners are invoked WITHOUT holding DeviceManager::mutex. Holding the
     * mutex during dispatch creates an AB/BA deadlock with any caller that
     * already holds another resource (e.g. ISDevice::portMutex) and reaches into
     * DeviceManager — for example `ISDevice::step()` (which holds portMutex,
     * fires DEVICE_DISCONNECTED on a port-state change) racing against
     * `DeviceManager::discoverDevices()` (which holds DeviceManager::mutex and
     * eventually takes portMutex via `validate→SendRaw`). Snapshotting the
     * listener vector under the lock and dispatching after release is safe:
     * the listener registry is only mutated through addPortListener/clear-style
     * helpers that already take the same mutex.
     */
    void notifyListeners(device_handle_t device, uint8_t event) {
        std::vector<device_listener_handle_t> snapshot;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex);
            snapshot.assign(listeners.begin(), listeners.end());
        }
        for (auto& l : snapshot) if (l) (*l)(event, device);
    }


    /**
     * Registers a previously allocated device. If a device matching the same hardware Id and
     * serial number is already registered (including the exact same instance), this is a no-op.
     * @param device a pointer to an ISDevice instance to add to the list of managed devices
     * @return true if the device was registered (or an equivalent device was already registered), false if device is null
     */
    bool registerDevice(device_handle_t device);

    /**
     * Allocates and registers a new ISDevice, by making a copy of the original device instance.
     * This is primarily used when statically allocating a ISDevice instance that needs to be managed
     * @param orig a reference to the original instance to make a copy of.
     * @return a pointer to the new ISDevice instance which is managed by the DeviceManager
     */
    device_handle_t registerNewDevice(const ISDevice& orig);

    /**
     * Allocates and registers a new ISDevice from the associated port, and respective device info.
     * This is primarily used to register a new ISDevice instance for a particular port once the port.
     * If devInfo is provided, it will be used to initially seed the ISDevice's dev info, otherwise the
     * device info will be queried as soon as the connection is opened.  Note that this call does not
     * automatically connect the associate port.
     * @param port the port to which this device is connected (even if not opened)
     * @param devInfo (optional) the essential device information for this device, if known
     * @return a pointer to the new ISDevice instance which is managed by the DeviceManager
     */
    device_handle_t registerNewDevice(port_handle_t port, dev_info_t devInfo = {});

    /**
     * Removes the specified device from being managed by the DeviceManager, and can optionally (by
     * default) close the associated port and free/delete the memory associated with the device.
     * The closePort/deleteDevice arguments can be used to decouple a device from the DeviceManager
     * while keeping the device functional, in which case it becomes the caller's responsibility to
     * ensure the device is properly closed/released when no longer needed.
     * @param device a pointer/reference to the device to release
     * @param closePort if true (default) the port will be closed (if open) prior to the device being released, otherwise the port is left in its current state.
     * @param deleteDevice if true (default) the memory associated with the device will be deallocated, otherwise the device is left in an operational state.
     * @return true if the device was found, and released otherwise false
     */
    bool releaseDevice(device_handle_t device, bool closePort = true, bool deleteDevice = true);

    /**
     * Remove and release/free all known/discovered devices.
     */
    void clear(bool closePorts = true);

    /**
    * Get the number of open devices
    * @return the number of open devices
    */
    size_t DeviceCount() { return size(); }

    /**
     * Returns a reference to the backing list of available, connected devices
     * @return a reference to the DeviceManager's underlying std::list<device_handle_t>
     */
    std::list<device_handle_t>& getDevices() { return *this; };

    /**
     * @returns a std::vector<device_handle_t> of known devices
     */
    std::vector<device_handle_t> getDevicesAsVector();

    /**
     * @returns an device_handle_t instance identified by the specified UID, or NULL if not found
     */
    device_handle_t getDevice(uint64_t uid);

    /**
     * @returns an device_handle_t instance associated with the specified port, or NULL if not found
     */
    device_handle_t getDevice(port_handle_t port);

    /**
     * @returns an device_handle_t instance at the specified index, or NULL if not found
     */
    device_handle_t getDeviceByIndex(int index);

    /**
     * @returns an device_handle_t instance identified by the deviceId string (as provided by ISDevice::getIdAsString()), or NULL if not found
     */
    device_handle_t getDevice(const std::string& deviceId);

    /**
     * Returns the first ISDevice instance matching the specified criteria
     * @param serialNum the serial number of the device to return
     * @param hdwId an optional hdwId to further filter on
     * @return an device_handle_t instance or NULL of not found
     */
    device_handle_t getDevice(uint32_t serialNum, is_hardware_t hdwId = IS_HARDWARE_ANY);

    /**
     * Returns a subset of connected devices filtered by the passed devInfo and filterFlags.
     * filterFlags is a bitmask the matches the returned bitmap from compareDevInfo, in which
     * each bit corresponds to a field in devInfo, which must be matched in order to be
     * selected. All bits which are set in filterFlags must also be set in the result from
     * compareDevInfo in order to selected.  Passing 0x0000 for filterFlags will return all available
     * devices (any device matches), while passing 0xFFFF will only match an exact match, including
     * the serial number.
     * @param devInfo the reference device info to compare each known device against (via utils::compareDevInfo)
     * @param filterFlags bitmask of the devInfo fields that must match for a device to be selected
     * @return a vector of device_handle_t which match the filter criteria (devInfo/filterFlags)
     */
    std::vector<device_handle_t> selectByDevInfo(const dev_info_t& devInfo, uint32_t filterFlags);

    /**
     * Returns a subset of connected devices filtered by the passed hardware id.
     * Note that any HdwId component (TYPE, MAJOR, MINOR) which bit mask is all ones, will
     * be ignored in the filter criteria.  Ie, to filter on ALL IMX devices, regardless of
     * version, pass hdwId = ENCODE_HDW_ID(HDW_TYPE__IMX, 0xFF, 0xFF), or to filter on any
     * IMX-5.x devices, pass hdwId = ENCODE_HDW_ID(HDW_TYPE__IMX, 5, 0xFF)
     * @param hdwId the encoded hardware Id (type/major/minor) to filter on; a component whose bits are all ones is treated as a wildcard for that component
     * @return a vector of device_handle_t which match the filter criteria (hdwId)
     */
    std::vector<device_handle_t> selectByHdwId(const uint16_t hdwId = 0xFFFF);


    /**
     * Provided a directory of firmware files, this call returns a tuple of device_handle_t and
     * a path to a firmware file which can be used to update that device to the latest version.
     * @param firmwarePath a string indicating a base directory which contains firmware files to
     *  evaluate to determine if any are suitable for each device.
     * @returns a vector of pairs of devices (first) which can be upgraded using the firmware
     *  file (second).
     */
    std::vector<std::pair<device_handle_t, std::string>> getUpgradableDevices(const std::string& firmwarePath);

protected:
    /**
     * @brief PortManager port_listener callback, registered by the constructor. On PORT_REMOVED,
     * releases the port from whichever device it was bound to (notifying DEVICE_PORT_LOST) and
     * erases the port-to-device mapping. PORT_ADDED is currently a no-op (see the TODO in the .cpp).
     * @param event a PortManager::port_event_e value
     * @param pType the port's type bitmask
     * @param pName the port's name
     * @param port the port handle the event pertains to
     * @param factory the PortFactory that owns/produced the port
     */
    void portHandler(uint8_t event, uint16_t pType, std::string pName, port_handle_t port, PortFactory& factory);

    /** @brief Private constructor (singleton, use getInstance()); registers portHandler() as a PortManager port listener. */
    DeviceManager() {
        PortManager::getInstance().addPortListener([this](auto && PH1, auto && PH2, auto && PH3, auto && PH4, auto && PH5) { portHandler(PH1, PH2, PH3, PH4, PH5); });
    };

    ~DeviceManager()  = default;

    /**
     * Callback handler used by factories when a device is detected (but not yet allocated)
     * @param factory the factory which discovered this device
     * @param devInfo the dev_info_t identifying the discovered device, as reported by factory
     * @param port the port which the device was discovered on
     * @param options a bitmask of DISCOVERY__* options controlling whether the port is closed on rejection
     * @returns true if the device was properly allocated and added to the manager, otherwise false
     */
    bool deviceHandler(DeviceFactory* factory, const dev_info_t& devInfo, port_handle_t port, int options);


private:
    /** @brief A previously-discovered device's identity, tracked in knownDevices even after the device itself has been released. */
    struct device_entry_t {
        DeviceFactory* factory;    //!< the factory that discovered/claimed this device
        uint64_t hdwId;            //!< encoded hardware Id + serial number uniquely identifying this device (see ENCODE_UNIQUE_ID)
        device_handle_t device;    //!< the device instance, if still allocated/managed; may be stale once released

        device_entry_t(DeviceFactory* f, uint64_t i, device_handle_t d) : factory(f), hdwId(i), device(d) {};
    };

    PortManager& portManager = PortManager::getInstance();             //!< reference to the PortManager singleton, used by portHandler() registration

    std::vector<DeviceFactory*> factories;                              //!< list of device factories responsible for detecting, allocating and freeing ports of different types. -- Note that DeviceFactories should always be static singletons, DO NOT FREE/DELETE the factory!
    std::unordered_set<device_listener_handle_t> listeners;             //!< list of listeners who should be notified when new devices are discovered, lost, opened, closed, etc
    std::vector<device_entry_t> knownDevices;                           //!< vector of previously discovered devices, by factory & hdwid (bits 47-63) + serial (bits 0-31) - different than actual, allocated devices
    std::map<port_handle_t, device_handle_t> portToDeviceMap;           //!< map of port handles to device handles for fast lookups

    int managementOptions = DISCOVERY__DEFAULTS;                        //!< a bit mask of various options used to modify the behavior of the device manager during various operations
    std::recursive_mutex mutex;                                         //!< guards factories/listeners/knownDevices/portToDeviceMap/deviceHints_ against concurrent access
    std::map<port_handle_t, dev_info_t> deviceHints_;                  //!< seeded hints keyed by port_handle_t; valid for the lifetime of the port handle

    /**
     * @brief RAII range-adapter returned by locked_range(): holds DeviceManager::mutex for its
     * entire lifetime (typically the duration of a range-based for loop) while exposing the
     * begin()/end() iterators of the underlying device list, so callers can safely iterate the
     * DeviceManager's devices without a separate explicit lock/unlock.
     */
    class LockedRangeProxy {
    private:
        DeviceManager& container;
        std::scoped_lock<std::recursive_mutex> lock_guard;                     // The lock_guard/scoped_lock ensures RAII

    public:
        LockedRangeProxy(DeviceManager& container_ref) : container(container_ref), lock_guard(container_ref.mutex) { }

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
    /** @brief Returns a LockedRangeProxy holding DeviceManager::mutex for safe range-based iteration over the managed devices. */
    LockedRangeProxy locked_range() { return LockedRangeProxy(*this); }

};


#endif //IS_SDK__DEVICE_MANAGER_H
