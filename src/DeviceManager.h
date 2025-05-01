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

    /**
     * Queries all factories to identify and enumerate all ports which can be discovered by all registered factories
     * Note that only newly discovered devices which have not been previously discovered will trigger a device_listener callback.
     * This function does not return a value, and provides no direct indication that any devices were successfully discovered.
     * @param pm a reference to a PortManager instance (but, its a singleton??) which will be used for ports to query for new devices.
     * @param hdwId a IS_HARDWARE_* type used to restrict the discovery to only matching device types, default value of IS_HARDWARE_ANY
     * will match all device types.
     */
    void discoverDevices(uint16_t hdwId = IS_HARDWARE_ANY) {
        for (auto l : factories) {
            auto cb = std::bind(&DeviceManager::deviceHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
            l->locateDevices(cb, hdwId);
        }
    }

    /**
     * Iterates through all registered factories attempting to identify a discoverable device on the specified port.
     * Note that only newly discovered devices which have not been previously discovered will trigger a device_listener callback.
     * This function does not return a value, and provides no direct indication that any devices were successfully discovered.
     * @param port a port_handle_t which is queried for a viable device.
     * @param hdwId a IS_HARDWARE_* type used to restrict the discovery to only matching device types, default value of IS_HARDWARE_ANY
     * will match all device types.
     */
    void discoverDevice(port_handle_t port, uint16_t hdwId = IS_HARDWARE_ANY) {
        if (!port)
            return;

        // first check if this port is already associated with another device
        for (auto d: *this) {
            if (d && d->port == port)
                return;
        }

        for (auto l : factories) {
            auto cb = std::bind(&DeviceManager::deviceHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
            l->locateDevice(cb, port, hdwId);
        }
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
     * Returns a reference to an is_device_t struct that contains information about the specified device
     * @return
     */
    // ISDevice* getDevice(uint32_t index);
    ISDevice* getDevice(port_handle_t port);

    /**
     * Returns the ISDevice instance associated with the specified port, or NULL if there is no associated device
     * @param port
     * @return
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

    void checkForNewDevices(uint16_t hdwId = IS_HARDWARE_ANY) { }


private:
    struct device_entry_t {
        DeviceFactory* factory;
        uint64_t hdwId;
        ISDevice* device;

        device_entry_t(DeviceFactory* f, uint64_t i, ISDevice* d) {
            factory = f, hdwId = i, device = d;
        };
    };

    std::vector<DeviceFactory*> factories;                              //!< list of device factories responsible for detecting, allocating and freeing ports of different types. -- Note that DeviceFactories should always be static singletons, DO NOT FREE/DELETE the factory!
    std::vector<device_listener> listeners;                             //!< list of listeners who should be notified when new devices are discovered, lost, opened, closed, etc
    std::vector<device_entry_t> knownDevices;                           //!< vector of previously discovered devices, by factory & hdwid (bits 47-63) + serial (bits 0-31) - different than actual, allocated devices
};


#endif //EVALTOOL_DEVICEMANAGER_H
