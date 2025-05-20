/**
 * @file DeviceManager.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/4/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "DeviceManager.h"

/**
 * Registers a previously created ISDevice instance
 * @param device
 * @return
 */
bool DeviceManager::registerDevice(ISDevice* device) {
    if (!device)
        return NULL;

    // first, ensure there isn't a matching device already
    for (auto d : *this) {
        if (d == device)
            return true;
        if ((d->hdwId == ENCODE_DEV_INFO_TO_HDW_ID(device->devInfo)) && (d->devInfo.serialNumber == device->devInfo.serialNumber))
            return true;
    }

    push_back(device);
    return true;
}

/**
 * Creates a new ISDevice instance by calling the newDeviceHandler function, with the port and dev_info_t that will
 * be associated with the device. This attempt to avoid redundant entries by checking if any previously registered
 * devices exists for the same HdwID and Serial No; if found, that existing device will be returned.
 * If m_newDeviceHandler is null, then a generic ISDevice will be created.
 * @param port the port that the new device is connected to
 * @param devInfo the dev_info_t that describes the device
 * @return a pointer to an ISDevice instance
 */
ISDevice* DeviceManager::registerNewDevice(const ISDevice& device) {
    // first, ensure there isn't a previously allocated device which matches this hdwId/SerialNo
    for (auto d : *this) {
        if ((d->hdwId == ENCODE_DEV_INFO_TO_HDW_ID(device.devInfo)) && (d->devInfo.serialNumber == device.devInfo.serialNumber)) {
            // debug_message("[DBG] Found existing ISDevice reference '%s'; Updating port %s.\n", d->getIdAsString().c_str(), d->getPortName().c_str());
            // d->port = nullptr; FIXME: do we want to update the port?  I don't know...
            d->devInfo = device.devInfo;
            return d;
        }
    }

    // go through all the registered factories, to find which factory should instance this devices
    for (DeviceFactory* f : factories) {
        // If we're here, we didn't find the device above
        ISDevice *newDevice = f->allocateDevice(device.devInfo);
        if (newDevice) {
            if (portIsValid(device.port)) {
                newDevice->assignPort(device.port);
                // device.assignPort(nullptr);
            }
            push_back(newDevice);
            // debug_message("[DBG] Allocating new ISDevice '%s' on port %s.\n", newDevice->getIdAsString().c_str(), newDevice->getPortName().c_str());
            return empty() ? NULL : (ISDevice *) back();
        }
    }

    return nullptr;
}

/**
 * Creates a new ISDevice instance by calling the newDeviceHandler function, with the port and dev_info_t that will
 * be associated with the device. This attempt to avoid redundant entries by checking if any previously registered
 * devices exists for the same HdwID and Serial No; if found, that existing device will be returned.
 * If m_newDeviceHandler is null, then a generic ISDevice will be created.
 * @param port the port that the new device is connected to
 * @param devInfo the dev_info_t that describes the device
 * @return a pointer to an ISDevice instance
 */
ISDevice* DeviceManager::registerNewDevice(port_handle_t port, dev_info_t devInfo) {
    // go through all the registered factories, to find which factory should instance this devices
    for (DeviceFactory* f : factories) {
        ISDevice* newDevice = f->allocateDevice(devInfo);
        if (newDevice) {
            push_back(newDevice);
            newDevice->assignPort(port);
            return newDevice;
        }
    }
    return nullptr;
}

/**
 * Removes the specified device and associated port from being managed by the InertialSense's comManager instance.
 * This does not free/delete/release the device or port, but the underlying call into comManagerRemovePort() will
 * close the port. This is a special-use function as there is generally little utility is retaining an ISDevice
 * instance which is not attached to the InertialSense class; you should probably be using releaseDevice() instead.
 * NOTE: if you use RemoveDevice() it is the callers responsibility to delete/release the ISDevice instance, as
 * the InertialSense class will no longer manage it.
 */
bool DeviceManager::releaseDevice(ISDevice* device, bool closePort)
{
    auto deviceIter = std::find(begin(), end(), device);
    if (deviceIter == end())
        return false;

    debug_message("[DBG] Releasing device '%s' on port '%s'\n", device->getIdAsString().c_str(), device->getPortName().c_str());

    // FIXME: where does Logger()
    // auto dl = Logger()->getDeviceLogByPort(device->port);
    // if (dl) dl->CloseAllFiles();

    if (closePort && portIsValid(device->port) && portIsOpened(device->port)) {
        portClose(device->port);
        // portManager.releasePort(device->port);
    }

    erase(deviceIter); // erase only remove the ISDevice* from the list, but doesn't release/free the instance itself
    device->port = NULL;
    // delete device; // causes a double free?? -- FIXME This maybe problematic, since there may be external references to this device, which likely won't be notified of it being deleted (DeviceCollector, etc)

    return true;
}

/**
 * Called by the DeviceFactories when a device is identified.
 * This is a low-level callback indicating that a device identified by devInfo was discovered.
 * This handler is responsible for makings further determinations of viability and making callbacks
 * for specifics events such as whether the device is a new device, or a previously known device.
 * NOTE: this function will likely be called frequently, since it will be called once
 * for each device identified, for each locator register, for every call of checkForNewPort()
 * @param factory - the factory which discovered this device
 * @param devInfo - the device info for the discovered device
 * @param port - the port the device was discovered on, if any
 */
void DeviceManager::deviceHandler(DeviceFactory *factory, const dev_info_t &devInfo, port_handle_t port) {

    uint64_t devId = ENCODE_DEV_INFO_TO_UNIQUE_ID(devInfo);
    if (!devId)
        return; // this is an invalid device Id -- no hdwId and no serialNo

    device_entry_t deviceEntry(factory, devId, nullptr);

    // check if Device is previously known
    for (auto& kd : knownDevices) {
        if ((kd.factory == deviceEntry.factory) && (kd.hdwId == deviceEntry.hdwId)) {
            // We've re-discovered an old device, but we don't know the status of its port... we should try and figure that out, before we just blindly return...
            debug_message("[DBG] Rediscovered previously known device [%s] on serial port '%s'.\n", ISDevice::getIdAsString(devInfo).c_str(), portName(port));
            ISDevice* device = getDevice(port);
            if (!device) {
                device = getDevice(devInfo.serialNumber, ENCODE_DEV_INFO_TO_HDW_ID(devInfo));
            }

            if (device && portIsValid(port)) {
                device->assignPort(port);
            } else {
                // FIXME: if we're here, it means we had a deviceEntry that matched the discovered device, but its associated device is invalid.
                //  we don't want to reallocate the device, since there is already one there, but just need to reassign the devInfo, etc.
                debug_message("[DBG] -- Device or port is invalid. Dropping device, and attempting a rebind on port '%s'.\n", portName(port));
                delete deviceEntry.device;
                deviceEntry.device = nullptr;
                break;
            }
            return;
        }
    }

    // if not, then do we need to allocate it?
    deviceEntry.device = factory->allocateDevice(devInfo, port);
    if (!deviceEntry.device)
        return;

    // debug_message("[DBG] Allocated new device: %s.\n", device->getDescription().c_str());
    knownDevices.push_back(deviceEntry);
    push_back(deviceEntry.device);

    // finally, call our handler
    for (device_listener& l : listeners) {
        l(DEVICE_ADDED, deviceEntry.device);
    }
}


void DeviceManager::portHandler(uint8_t event, uint16_t pType, std::string pName, port_handle_t port) {
    switch ((PortManager::port_event_e)event) {
        case PortManager::PORT_ADDED:
            // TODO: If "automatic device validation" is true, we should use this event to automatically open the port and validate the device.
            debug_message("[DBG] DeviceManager-->PortManager::PORT_ADDED '%s'\n", pName.c_str());
            break;
        case PortManager::PORT_REMOVED:
            debug_message("[DBG] DeviceManager-->PortManager::PORT_REMOVED '%s'.\n", pName.c_str());
            ISDevice* device = getDevice(port);
            if (device) {
                device->assignPort(nullptr); // revoke the removed port from the device...
            }
            break;
    }
}


std::vector<ISDevice *> DeviceManager::getDevicesAsVector() {
    std::vector<ISDevice*> vecOut;
    for (auto device : *this) {
        vecOut.push_back(device);
    }
    return vecOut;
}

ISDevice *DeviceManager::getDevice(port_handle_t port) {
    for (auto device : *this) {
        if (device->port == port)
            return device;
    }
    return NULL;
}

ISDevice *DeviceManager::getDevice(uint64_t uid) {
    for (auto device : *this) {
        if (device && (ENCODE_DEV_INFO_TO_UNIQUE_ID(device->devInfo) == uid))
            return device;
    }
    return NULL;
}

ISDevice *DeviceManager::getDevice(uint32_t serialNum, is_hardware_t hdwId) {
    for (auto device : *this) {
        if ((device->hdwId == hdwId) && (device->devInfo.serialNumber == serialNum))
            return device;
    }
    return NULL;
}

/*
std::vector<std::string> DeviceManager::GetPortNames()
{
    std::vector<std::string> ports;
    for (auto device : *this) { ports.push_back(portName(device->port)); }
    return ports;
}

*/

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
 * @return a vector of ISDevice which match the filter criteria
 */
std::vector<ISDevice *> DeviceManager::selectByDevInfo(const dev_info_t &devInfo, uint32_t filterFlags) {
    std::vector<ISDevice*> selected;

    for (auto device : *this) {
        uint32_t matchy = utils::compareDevInfo(devInfo, device->devInfo) & filterFlags;
        if (matchy == filterFlags)
            selected.push_back(device);
    }
    return selected;
}



/**
 * Returns a subset of connected devices filtered by the passed hardware id.
 * Note that any HdwId component (TYPE, MAJOR, MINOR) which bit mask is all ones, will
 * be ignored in the filter criteria.  Ie, to filter on ALL IMX devices, regardless of
 * version, pass hdwId = ENCODE_HDW_ID(HDW_TYPE__IMX, 0xFF, 0xFF), or to filter on any
 * IMX-5.x devices, pass hdwId = ENCODE_HDW_ID(HDW_TYPE__IMX, 5, 0xFF)
 * @param hdwId
 * @return a vector of ISDevice* which match the filter criteria (hdwId)
 */
std::vector<ISDevice *> DeviceManager::selectByHdwId(const uint16_t hdwId) {
    dev_info_t devInfo = { };
    uint32_t filterFlags = 0;

    // filter hdw type
    devInfo.hardwareType = DECODE_HDW_TYPE(hdwId);
    if (devInfo.hardwareType != (HDW_TYPE__MASK >> HDW_TYPE__SHIFT)) {
        filterFlags |= (1 << 1);
    }

    // filter major hdw version
    devInfo.hardwareVer[0] = DECODE_HDW_MAJOR(hdwId);
    if (devInfo.hardwareVer[0] != (HDW_MAJOR__MASK >> HDW_MAJOR__SHIFT)) {
        filterFlags |= (1 << 4);
    }

    // filter minor hdw version
    devInfo.hardwareVer[1] = DECODE_HDW_MINOR(hdwId);
    if (devInfo.hardwareVer[1] != (HDW_MINOR__MASK >> HDW_MINOR__SHIFT)) {
        filterFlags |= (1 << 5);
    }

    return selectByDevInfo(devInfo, filterFlags);
}

