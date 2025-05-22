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

#ifndef EVALTOOL_DEVICEMANAGER_H
#define EVALTOOL_DEVICEMANAGER_H

#include <list>

#include "core/msg_logger.h"

#include "ISDevice.h"
#include "PortManager.h"
#include "DeviceFactory.h"

typedef ISDevice*(*pfnOnNewDeviceHandler)(port_handle_t port, const dev_info_t& devInfo);

typedef ISDevice*(*pfnOnCloneDeviceHandler)(const ISDevice& orig);

typedef std::function<void(uint8_t, ISDevice*)> device_listener;

// typedef void(*pfnStepLogFunction)(void* ctx, const p_data_t* data, port_handle_t port);
typedef std::function<void(void* ctx, p_data_t* data, port_handle_t port)> pfnHandleBinaryData;


class DeviceManager : public std::list<ISDevice*>
{
public:
    enum device_event_e : uint8_t {
        DEVICE_ADDED,
        DEVICE_REMOVED,
    };

    DeviceManager(DeviceManager const &) = delete;
    DeviceManager& operator=(DeviceManager const&) = delete;

    static DeviceManager& getInstance() {
        static DeviceManager instance;
        return instance;
    }

    // TODO: we need a mechanism to manage ports when discovering devices.  The following options need to be implemented; there maybe more options still undetermined
    //  - IgnoreClosedPorts - don't attempt to discover devices for ports which are not actively open.
    //  - ClosePortsOnDiscovery - if the discovery succeeds, and a device is validated, if set, close the port regardless
    //  - ClosePortsOnError - if the discovery fails to validate a device on the current port (for any reason), close the port
    //  - ClosePortAfterDiscovery - essentially the combination of "ClosePortsOn" because, if set, the port will always be closed after attempting to discover, regardless of success/failure.
    //  - rediscoverKnownDevices - if true, will force ports known to belong to an existing device to be rediscovered; if false, ports already bound to a device, and validated will be ignored

    bool getOption_IgnoreClosedPorts() { return option_ignoreClosedPorts; }
    void setOption_IgnoreClosedPorts(bool v) { option_ignoreClosedPorts = v; }


    bool getOption_ClosePortOnError() { return option_closePortOnError; }
    void setOption_ClosePortOnFault(bool v) { option_closePortOnError = v; }

    bool getOption_ClosePortOnDiscovery() { return option_closePortOnDiscovery; }
    void setOption_ClosePortOnDiscovery(bool v) { option_closePortOnDiscovery = v; }

    /**
     * Queries all factories to identify and enumerate all ports which can be discovered by all registered factories
     * Note that only newly discovered devices which have not been previously discovered will trigger a device_listener callback.
     * This function does not return a value, and provides no direct indication that any devices were successfully discovered.
     * @param pm a reference to a PortManager instance (but, its a singleton??) which will be used for ports to query for new devices.
     * @param hdwId a IS_HARDWARE_* type used to restrict the discovery to only matching device types, default value of IS_HARDWARE_ANY
     *  will match all device types.
     * @return true if one more more devices were discovered, otherwise false
     */
    bool discoverDevices(uint16_t hdwId = IS_HARDWARE_ANY, int timeoutMs = 0) {
        bool result = false;
        for (auto& port : portManager) {
            result |= discoverDevice(port, hdwId, timeoutMs);
        }
        return result;
    }

    /**
     * Iterates through all registered factories attempting to identify a discoverable device on the specified port.
     * Note that only newly discovered devices which have not been previously discovered will trigger a device_listener callback.
     * @param port a port_handle_t which is queried for a viable device.
     * @param hdwId a IS_HARDWARE_* type used to restrict the discovery to only matching device types, default value of IS_HARDWARE_ANY
     *  will match all device types.
     * @param timeoutMs the number of milliseconds to wait for a device to respond before failing
     * @return true if a device was discovered on the specified port, otherwise false
     */
    bool discoverDevice(port_handle_t port, uint16_t hdwId = IS_HARDWARE_ANY, int timeoutMs = 0) {
        if (!portIsValid(port))
            return false;

        // If the port isn't opened, do we ignore it, or attempt to open it?
        if (!portIsOpened(port) && (option_ignoreClosedPorts || (portOpen(port) != PORT_ERROR__NONE)))
            return false;

        // first check if this port is already associated with another device
        for (auto d: *this) {
            if (d && d->hasDeviceInfo() && (d->port == port))
                return false;
        }

        for (auto l : factories) {
            std::function<void(DeviceFactory *, const dev_info_t &, port_handle_t)> cb = std::bind(&DeviceManager::deviceHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
            if (l->locateDevice(cb, port, hdwId, timeoutMs))
                return true;    // successfully located/allocated a device
        }
        return false;
    }

    /**
     * Removed all previously registered DeviceFactories
     */
    void clearDeviceFactories() { factories.clear(); }

    /**
     * Adds a custom DeviceFactory to be used when new Devices are discovered
     * @param df a pointer to the DeviceFactory instance
     */
    void addDeviceFactory(DeviceFactory* df) { factories.push_back(df); };

    /**
     * Convenience function that clears any existing registered DeviceFactories, and adds a new custom DeviceFactory to be used when new Devices are discovered.
     *   Calls clearDeviceFactories() followed by addDeviceFactory()
     * @param df a pointer to the DeviceFactory instance
     */
    void setDeviceFactory(DeviceFactory* df) { clearDeviceFactories(); addDeviceFactory(df); };

    /**
     * Registers a custom handler to instantiate discovered devices. Default behavior is to
     * create new ISDevice instances for each new device discovered. Setting a NewDeviceHandler
     * to a custom function allows for instancing a custom ISDevice subclass and/or doing any
     * additional initialization of that device at creation. The handler is provided the port
     * and the device info for the newly discovered device.
     * @param handler a function pointer to be called when a new device is discovered
     * @return the previously registered handler, if any
     */
    void addDeviceListener(const device_listener& listener) {
        listeners.push_back(listener);
    }


    /**
     * Registers a previously allocated device
     * @param device a pointer to an ISDevice instance to add to the list of managed devices
     * @return
     */
    bool registerDevice(ISDevice* device);

    /**
     * Allocates and registers a new ISDevice, by making a copy of the original device instance.
     * This is primarily used when statically allocating a ISDevice instance that needs to be managed
     * @param orig a reference to the original instance to make a copy of.
     * @return a pointer to the new ISDevice instance which is managed by the DeviceManager
     */
    ISDevice* registerNewDevice(const ISDevice& orig);

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
    ISDevice* registerNewDevice(port_handle_t port, dev_info_t devInfo = {});

    /**
     * Releases the specified device, freeing any associated memory, and optionally closing any connected ports
     * @param device
     * @param closePort
     * @return true if the device was found, and released otherwise false
     */
    bool releaseDevice(ISDevice* device, bool closePort = true);

    /**
    * Get the number of open devices
    * @return the number of open devices
    */
    size_t DeviceCount() { return size(); }

    /**
     * Returns a reference to the backing list available, connected devices
     * @return
     */
    std::list<ISDevice*>& getDevices() { return *this; };

    /**
     * Returns a vector of available, connected devices
     * @return
     */
    std::vector<ISDevice*> getDevicesAsVector();

    /**
     * @return an ISDevice* instance associated with the specified port, or NULL if not found
     */
    ISDevice* getDevice(port_handle_t port);

    /**
     * @return an ISDevice* instance identified by the specified UID, or NULL if not found
     */
    ISDevice* getDevice(uint64_t uid);

    /**
     * Returns the first ISDevice instance matching the specified criteria
     * @param serialNum the serial number of the device to return
     * @param hdwId an optional hdwId to further filter on
     * @return an ISDevice* instance or NULL of not found
     */
    ISDevice* getDevice(uint32_t serialNum, is_hardware_t hdwId = IS_HARDWARE_ANY);

    /**
     * Returns a subset of connected devices filtered by the passed devInfo and filterFlags.
     * filterFlags is a bitmask the matches the returned bitmap from compareDevInfo, in which
     * each bit corresponds to a field in devInfo, which must be matched in order to be
     * selected. All bits which are set in filterFlags must also be set in the result from
     * compareDevInfo in order to selected.  Passing 0x0000 for filterFlags will return all available
     * devices (any device matches), while passing 0xFFFF will only match an exact match, including
     * the serial number.
     * @param devInfo
     * @param filterFlags
     * @return a vector of ISDevice* which match the filter criteria (devInfo/filterFlags)
     */
    std::vector<ISDevice*> selectByDevInfo(const dev_info_t& devInfo, uint32_t filterFlags);

    /**
     * Returns a subset of connected devices filtered by the passed hardware id.
     * Note that any HdwId component (TYPE, MAJOR, MINOR) which bit mask is all ones, will
     * be ignored in the filter criteria.  Ie, to filter on ALL IMX devices, regardless of
     * version, pass hdwId = ENCODE_HDW_ID(HDW_TYPE__IMX, 0xFF, 0xFF), or to filter on any
     * IMX-5.x devices, pass hdwId = ENCODE_HDW_ID(HDW_TYPE__IMX, 5, 0xFF)
     * @param hdwId
     * @return a vector of ISDevice* which match the filter criteria (hdwId)
     */
    std::vector<ISDevice*> selectByHdwId(const uint16_t hdwId = 0xFFFF);

    void clear() {
        std::list<ISDevice*>::clear();
        knownDevices.clear();
    }

protected:
    void portHandler(uint8_t event, uint16_t pType, std::string pName, port_handle_t port);

    DeviceManager() {
        PortManager::getInstance().addPortListener([this](auto && PH1, auto && PH2, auto && PH3, auto && PH4) { portHandler(PH1, PH2, PH3, PH4); });
    };

    ~DeviceManager()  = default;

    /**
     * Callback handler used by factories when a device is detected (but not yet allocated)
     * @param factory - the factory which discovered this device
     * @param port - the port which the device was discovered on
     * @param devInfo - the name of the port (as determined by the factory, should be unique)
     */
    void deviceHandler(DeviceFactory* factory, const dev_info_t& devInfo, port_handle_t port);


private:
    struct device_entry_t {
        DeviceFactory* factory;
        uint64_t hdwId;
        ISDevice* device;

        device_entry_t(DeviceFactory* f, uint64_t i, ISDevice* d) {
            factory = f, hdwId = i, device = d;
        };
    };

    PortManager& portManager = PortManager::getInstance();

    std::vector<DeviceFactory*> factories;                              //!< list of device factories responsible for detecting, allocating and freeing ports of different types. -- Note that DeviceFactories should always be static singletons, DO NOT FREE/DELETE the factory!
    std::vector<device_listener> listeners;                             //!< list of listeners who should be notified when new devices are discovered, lost, opened, closed, etc
    std::vector<device_entry_t> knownDevices;                           //!< vector of previously discovered devices, by factory & hdwid (bits 47-63) + serial (bits 0-31) - different than actual, allocated devices

    bool option_ignoreClosedPorts = true;                               //!< option indicating whether to attempt to discovery of devices on ports which are closed - if true, only currently opened ports will be used for discovery; if false, closed ports will be opened before attempting to discover
    bool option_closePortOnError = true;                                //!< option indicating whether to close a port when the discovery fails - if true, ports would be closed after a failure to discovery; if true, ports would remain open (or fall through to closePortOnDiscovery)
    bool option_closePortOnDiscovery = false;                           //!< option indicating whether to close a port when the discovery succeeds - if true, ports would be closed after successful discovery, and would need to be reopened explicitly, if false ports are left open after discovery
};


#endif //EVALTOOL_DEVICEMANAGER_H
