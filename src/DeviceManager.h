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
    void discoverDevices(const PortManager& pm, uint16_t hdwId = IS_HARDWARE_ANY) {
        checkForNewDevices(pm);
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
        // first check if this port is already associated with another device
        for (auto d : *this) {
            if (d && d->port == port)
                return;
        }

        for (auto l : factories) {
            auto cb = std::bind(&DeviceManager::deviceHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
            l->locateDevice(cb, port);
        }
    }


    void addDeviceFactory(DeviceFactory* df) {
        factories.push_back(df);
    };

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
     * Step through all known devices; performing validation, data processing, and firmware upgrades.
     * This should be called at periodic intervals in order to allow all devices to process.
     * TODO: Ideally this should not be called, and each device will have its own thread to process
     *   its own data, etc.  But, that doesn't exists yet.
     */
    void step() { for (auto device : *this) { device->step(); } }

    /**
    * Request device(s) version information (dev_info_t).
    */
    void QueryDeviceInfo();

    /**
    * Turn off all messages.  Current port only if allPorts = false.
    */
    void StopBroadcasts(bool allPorts=true);

    /**
     * Current data streaming will continue streaming at boot.
     */
    void SavePersistent();

    /**
     * Software reset device(s) with open serial port.
     */
    void SoftwareReset();

    /**
     * @brief Request a specific data set by DID.
     *
     * @param dataId Data set ID
     * @param length Byte length of data requested.  Zero means entire data set.
     * @param offset Byte offset into data
     * @param period Broadcast period multiple
     */
    void GetData(eDataIDs dataId, uint16_t length=0, uint16_t offset=0, uint16_t period=0);

    /**
    * Send packet payload data to all devices; the payload data is wrapped according to the pktInfo parameter
    * and the appropriate checksum is calculated and appended.  This function can be used to send non-standard packets
    * and data sets, such as RTCM, UBLOX, etc.
    * @param pktInfo a field indication the type of, and flags for, the packet to be sent
    * @param dataId the data id of the data to send
    * @param payload the data to send
    * @param length length of data to send
    * @param offset offset into data to send at
     */
    void Send(uint8_t pktInfo, void *data=NULL, uint16_t did=0, uint16_t size=0, uint16_t offset=0);

    /**
     * Send IS packet payload data to all devices; the payload data is wrapped in an ISB packet with the specified dataId
     * and the appropriate checksum is calculated and appended.  This function can be used to send a subset of a data set.
     * For example, to set only a portion of DID_FLASH_CONFIG, you could use SendData like this:
     *   SendData(DID_FLASH_CONFIG, &cfg.refLla[0], sizeof(double)*3, offsetof(nvm_flash_cfg_t, refLla));
     * @param dataId the data id of the data to send
     * @param payload the data to send
     * @param length length of data to send
     * @param offset offset into data to send at
     */
    void SendData(eDataIDs dataId, void* data, uint32_t length, uint32_t offset = 0);

    /**
    * Send raw (bare) data directly to serial port
    * @param data the data to send
    * @param length length of data to send
    */
    void SendRaw(void* data, uint32_t length);

    /**
     * Request a specific device broadcast binary data
     * @param port the device's port to request data from
     * @param dataId the data id (DID_* - see data_sets.h) to broadcast
     * @param periodMultiple a scalar that the source period is multiplied by to give the output period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
     * @return true if success, false if error - if callback is NULL and no global callback was passed to the constructor, this will return false
     */
    void BroadcastBinaryData(uint32_t dataId, int periodMultiple);

    /**
    * Broadcast binary data
    * @param dataId the data id (DID_* - see data_sets.h) to broadcast
    * @param periodMultiple a scalar that the source period is multiplied by to give the output period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
    * @param callback optional callback for this dataId
    * @return true if success, false if error - if callback is NULL and no global callback was passed to the constructor, this will return false
    */
    // bool BroadcastBinaryData(uint32_t dataId, int periodMultiple, pfnHandleBinaryData callback = NULL);

    /**
    * Enable streaming of predefined set of messages.  The default preset, RMC_PRESET_INS, stream data necessary for post processing.
    * @param rmcPreset realtimeMessageController preset
    */
    void BroadcastBinaryDataRmcPreset(uint64_t rmcPreset=RMC_PRESET_INS, uint32_t rmcOptions=0);

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

    /**
    * Get the device info
    * @param device device to get device info for.
    * @return the device info
    */
    const dev_info_t DeviceInfo(port_handle_t port = 0);

    /**
    * Get current device system command
    * @param port the port to get sysCmd for
    * @return current device system command
    */
    system_command_t GetSysCmd(port_handle_t port = 0);

    /**
    * Set device configuration
    * @param port the port to set sysCmd for
    * @param command system command value (see eSystemCommand)
    */
    void SetSysCmd(const uint32_t command, port_handle_t port = 0);

    /**
     * Sends message to device to set devices Event Filter
     * param Target: 0 = device,
     *               1 = forward to device GNSS 1 port (ie GPX),
     *               2 = forward to device GNSS 2 port (ie GPX),
     *               else will return
     *       port: Send in target COM port.
     *                If arg is < 0 default port will be used
    */
    void SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel, port_handle_t port = 0);

    /**
    * Get the flash config, returns the latest flash config read from the IMX flash memory
    * @param flashCfg the flash config value
    * @param port the port to get flash config for
    * @return bool whether the flash config is valid, currently synchronized
    */
    bool FlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port = 0);

    /**
    * Indicates whether the current IMX flash config has been downloaded and available via FlashConfig().
    * @param port the port to get flash config for
    * @return true if the flash config is valid, currently synchronized, otherwise false.
    */
    bool FlashConfigSynced(port_handle_t port = 0)
    {
        ISDevice* device = NULL;
        if (!port) {
            device = front();
        } else {
            device = DeviceByPort(port);
        }

        if (device)
            return  (device->flashCfg.checksum == device->sysParams.flashCfgChecksum) &&
                    (device->flashCfgUploadTimeMs==0) && !FlashConfigUploadFailure(device->port);

        return false;
    }

    /**
     * @brief Failed to upload flash configuration for any reason.
     *
     * @param port the port to get flash config for
     * @return true Flash config upload was either not received or rejected.
     */
    bool FlashConfigUploadFailure(port_handle_t port = 0)
    {
        ISDevice* device = NULL;
        if (!port) {
            device = front();
        } else {
            device = DeviceByPort(port);
        }

        if (!device)
            return true;

        return device->flashCfgUpload.checksum && (device->flashCfgUpload.checksum != device->sysParams.flashCfgChecksum);
    }

    /**
    * Set the flash config and update flash config on the IMX flash memory
    * @param flashCfg the flash config
    * @param port the port to set flash config for
    * @return true if success
    */
    bool SetFlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port = 0);

    /**
     * @brief Blocking wait calling Update() and SLEEP(10ms) until the flash config has been synchronized.
     *
     * @param port the port
     * @return false When failed to synchronize
     */
    bool WaitForFlashSynced(port_handle_t port = 0);


protected:
    DeviceManager() = default;
    ~DeviceManager() {
        for (auto f : factories)
            delete f;
    };

    ISDevice* DeviceByPort(port_handle_t port)  {
        for (auto device : *this) {
            if (device->port == port)
                return device;
        }
        return nullptr;
    }

    /**
     * Callback handler used by factories when a device is detected (but not yet allocated)
     * @param factory - the factory which discovered this device
     * @param port - the port which the device was discovered on
     * @param devInfo - the name of the port (as determined by the factory, should be unique)
     */
    void deviceHandler(DeviceFactory* factory, const dev_info_t& devInfo, port_handle_t port);

    void checkForNewDevices(const PortManager& pm) {
        for (auto l : factories) {
            auto cb = std::bind(&DeviceManager::deviceHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
            l->locateDevices(cb, pm);
        }
    }


private:
    DeviceManager(PortManager const &) = delete;
    DeviceManager& operator=(DeviceManager const&) = delete;

    std::vector<DeviceFactory*> factories;                              //!< list of device factories responsible for detecting, allocating and freeing ports of different types.
    std::vector<device_listener> listeners;                             //!< list of listeners who should be notified when new devices are discovered, lost, opened, closed, etc
    std::vector<std::pair<DeviceFactory*, uint64_t>> knownDevices;      //!< vector of previously discovered devices, by factory & hdwid (bits 47-63) + serial (bits 0-31) - different than actual, allocated devices

};


#endif //EVALTOOL_DEVICEMANAGER_H
