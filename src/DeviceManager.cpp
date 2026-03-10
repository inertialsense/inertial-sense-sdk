/**
 * @file DeviceManager.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/4/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "DeviceManager.h"

/**
 * Concurrently validates all ports against all registered factories. Each port is probed by
 * each factory in round-robin, so every factory gets fair time to validate. The first factory
 * to successfully validate on a port wins that port and allocates the device.
 *
 * Total discovery time is bounded by max(factory timeout) rather than (num_ports * num_factories * timeout).
 */
bool DeviceManager::discoverDevices(uint16_t hdwId, uint32_t timeoutMs, uint32_t options) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    bool result = false;
    options = (options != OPTIONS_USE_DEFAULTS) ? options : managementOptions;
    options = (options == OPTIONS_USE_DEFAULTS) ? DISCOVERY__DEFAULTS : options;

    // Per-factory validation slot for a single port
    struct FactorySlot {
        DeviceFactory* factory;
        std::unique_ptr<DeviceFactory::ValidationContext> ctx;
        bool eliminated = false;  // true if this factory timed out or declined this port
    };

    // Per-port state tracking all factory slots and round-robin position
    struct PendingPort {
        port_handle_t port;
        std::vector<FactorySlot> slots;
        int activeSlot = 0;
        bool resolved = false;
        DeviceFactory* winner = nullptr;
        dev_info_t devInfo = {};
    };

    std::vector<PendingPort> pending;

    // Phase 1: For each port, ask each factory to beginValidation
    for (auto port : portManager.locked_range()) {
        if (!portIsValid(port))
            continue;

        // Check if port is already associated with a known device
        bool alreadyHandled = false;
        for (auto d : *this) {
            if (d && d->hasDeviceInfo() && (d->port == port)) {
                if (options & DISCOVERY__FORCE_REVALIDATION) {
                    d->devInfo.hdwRunState = HDW_STATE_UNKNOWN;
                    memset(d->devInfo.firmwareVer, 0, sizeof(d->devInfo.firmwareVer));
                } else {
                    if (options & DISCOVERY__CLOSE_PORT_ON_COMPLETION)
                        portClose(port);
                    result = true;
                    alreadyHandled = true;
                    break;
                }
            }
        }
        if (alreadyHandled)
            continue;

        // Open port if needed
        if ((!portIsOpened(port) && (options & DISCOVERY__IGNORE_CLOSED_PORTS)) ||
            (portOpen(port) != PORT_ERROR__NONE))
            continue;

        // Only COMM ports can be validated via ISComm protocol
        if (!(portType(port) & PORT_TYPE__COMM))
            continue;

        // Check if DeviceManager already has a device for this port
        device_handle_t existingDev = getDevice(port);
        if (existingDev) {
            if (existingDev->matchesHdwId(hdwId)) {
                if (!existingDev->port)
                    existingDev->assignPort(port);
                result |= existingDev->validate(timeoutMs);
            }
            if (options & DISCOVERY__CLOSE_PORT_ON_COMPLETION)
                portClose(port);
            continue;
        }

        // Open port if needed (once, before any factory sees it)
        if (!portIsOpened(port)) {
            if (portOpen(port) != PORT_ERROR__NONE)
                continue;
        }

        // Only COMM ports can be validated via ISComm protocol
        if (!(portType(port) & PORT_TYPE__COMM))
            continue;

        // Create ONE shared ISDevice probe per port — all factories share this instance
        uint32_t effectiveTimeout = (timeoutMs > 0) ? timeoutMs : DISCOVERY__DEFAULT_TIMEOUT;
        auto sharedDevice = std::make_shared<ISDevice>(hdwId, port);

        PendingPort pp;
        pp.port = port;
        for (auto f : factories) {
            auto ctx = f->beginValidation(port, hdwId, effectiveTimeout, sharedDevice);
            if (ctx) {
                pp.slots.push_back({f, std::move(ctx), false});
            }
        }
        if (!pp.slots.empty()) {
            pending.push_back(std::move(pp));
        }
    }

    // Phase 2+3: Concurrent validation loop with round-robin across factories per port
    if (!pending.empty()) {
        log_debug(IS_LOG_DEVICE_MANAGER, "Concurrently validating %zu port(s) across %zu factory(ies)",
            pending.size(), factories.size());

        bool allResolved = false;
        while (!allResolved) {
            allResolved = true;
            for (auto& pp : pending) {
                if (pp.resolved)
                    continue;

                // Find the active non-eliminated slot
                int startSlot = pp.activeSlot;
                bool foundActive = false;
                do {
                    if (!pp.slots[pp.activeSlot].eliminated) {
                        foundActive = true;
                        break;
                    }
                    pp.activeSlot = (pp.activeSlot + 1) % (int)pp.slots.size();
                } while (pp.activeSlot != startSlot);

                if (!foundActive) {
                    // All factories eliminated — port failed
                    log_more_debug(IS_LOG_DEVICE_MANAGER, "All factories eliminated for port '%s'.", portName(pp.port));
                    pp.resolved = true;
                    if (options & DISCOVERY__CLOSE_PORT_ON_FAILURE)
                        portClose(pp.port);
                    continue;
                }

                auto& slot = pp.slots[pp.activeSlot];
                int stepResult = slot.factory->stepValidation(*slot.ctx);

                if (stepResult == 1) {
                    // Phase 3 inline: this factory validated — check if it claims the device
                    dev_info_t devInfo;
                    if (slot.factory->completeValidation(*slot.ctx, devInfo)) {
                        pp.resolved = true;
                        pp.winner = slot.factory;
                        pp.devInfo = devInfo;

                        // Register via deviceHandler
                        std::function<bool(DeviceFactory*, const dev_info_t&, port_handle_t)> cb =
                            std::bind(&DeviceManager::deviceHandler, this,
                                std::placeholders::_1, std::placeholders::_2,
                                std::placeholders::_3, options);
                        if (cb(pp.winner, pp.devInfo, pp.port)) {
                            log_debug(IS_LOG_DEVICE_MANAGER, "deviceHandler accepted %s on port '%s'. DeviceCount=%zu",
                                ISDevice::getIdAsString(pp.devInfo).c_str(), portName(pp.port), size());
                            result = true;
                        } else {
                            // The winning factory's deviceHandler rejected — try remaining factories
                            bool handled = false;
                            for (auto f : factories) {
                                if (f == pp.winner)
                                    continue;
                                if (cb(f, pp.devInfo, pp.port)) {
                                    log_debug(IS_LOG_DEVICE_MANAGER, "deviceHandler accepted %s via fallback factory on port '%s'. DeviceCount=%zu",
                                        ISDevice::getIdAsString(pp.devInfo).c_str(), portName(pp.port), size());
                                    result = true;
                                    handled = true;
                                    break;
                                }
                            }
                            if (!handled) {
                                log_debug(IS_LOG_DEVICE_MANAGER, "deviceHandler REJECTED %s on port '%s' by all factories. DeviceCount=%zu",
                                    ISDevice::getIdAsString(pp.devInfo).c_str(), portName(pp.port), size());
                                if (options & DISCOVERY__CLOSE_PORT_ON_FAILURE)
                                    portClose(pp.port);
                            }
                        }
                    } else {
                        // Factory validated but rejected the device — eliminate and try others
                        slot.eliminated = true;
                    }
                } else if (stepResult == -1) {
                    // This factory timed out on this port — eliminate it
                    slot.eliminated = true;
                } else {
                    allResolved = false;
                }

                // Advance round-robin to next factory for this port's next cycle
                pp.activeSlot = (pp.activeSlot + 1) % (int)pp.slots.size();

                if (!pp.resolved)
                    allResolved = false;
            }
            if (!allResolved)
                SLEEP_MS(2);
        }
    }

    return result;
}

/**
 * Registers a previously created ISDevice instance - use this when a device which is manually allocated (statically, etc) needs to be managed
 * @param device
 * @return
 */
bool DeviceManager::registerDevice(device_handle_t device) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
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
    if (device->port) {
        portToDeviceMap[device->port] = device;
    }
    return true;
}

/**
 * Creates a new ISDevice instance by calling the newDeviceHandler function, with the port and dev_info_t that will
 * be associated with the device. This attempts to avoid redundant entries by checking if any previously registered
 * devices exists for the same HdwID and Serial No; if found, that existing device will be returned.
 * If m_newDeviceHandler is null, then a generic ISDevice will be created.
 * @param port the port that the new device is connected to
 * @param devInfo the dev_info_t that describes the device
 * @return a pointer to an ISDevice instance
 */
device_handle_t DeviceManager::registerNewDevice(const ISDevice& device) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    // first, ensure there isn't a previously allocated device which matches this hdwId/SerialNo
    for (auto d : *this) {
        if ((d->hdwId == ENCODE_DEV_INFO_TO_HDW_ID(device.devInfo)) && (d->devInfo.serialNumber == device.devInfo.serialNumber)) {
            // debug_message(IS_LOG_FACILITY_NONE, "Found existing ISDevice reference '%s'; Updating port %s.", d->getIdAsString().c_str(), d->getPortName().c_str());
            // d->port = nullptr; FIXME: do we want to update the port?  I don't know...
            d->devInfo = device.devInfo;
            return d;
        }
    }

    // go through all the registered factories, to find which factory should instance this devices
    for (DeviceFactory* f : factories) {
        // If we're here, we didn't find the device above
        device_handle_t newDevice = f->allocateDevice(device.devInfo);
        if (newDevice) {
            if (portIsValid(device.port)) {
                newDevice->assignPort(device.port);
                if (newDevice->port) {
                    portToDeviceMap[newDevice->port] = newDevice;
                }
                // device.assignPort(nullptr);
            }
            push_back(newDevice);
            // debug_message(IS_LOG_FACILITY_NONE, "Allocating new ISDevice '%s' on port %s.", newDevice->getIdAsString().c_str(), newDevice->getPortName().c_str());
            return empty() ? NULL : back();
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
device_handle_t DeviceManager::registerNewDevice(port_handle_t port, dev_info_t devInfo) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    // go through all the registered factories, to find which factory should instance this devices
    for (DeviceFactory* f : factories) {
        device_handle_t newDevice = f->allocateDevice(devInfo);
        if (newDevice) {
            push_back(newDevice);
            newDevice->assignPort(port);
            if (newDevice->port) {
                portToDeviceMap[newDevice->port] = newDevice;
            }
            return newDevice;
        }
    }
    return nullptr;
}

/**
 * Removes the specified device from being managed by the DeviceManager, and can optionally (by default) close
 * the associated port, and free/delete the memory associated with the device.  This can be used with the closePort/deleteDevice
 * arguments to decouple a device from the DeviceManager, and still keep the device functional, however it becomes to developers
 * responsibility to ensure that the device is properly closed/released when no longer needed.
 * @param device a pointer reference the device to release
 * @param closePort if true (default) the port will be closed (if open) prior to the device being released, otherwise the port is left in its current state.
 * @param deleteDevice if true (default) the memory associated with the device will be deallocated, otherwise the device is left in an operational state.
 */
bool DeviceManager::releaseDevice(device_handle_t device, bool closePort, bool deleteDevice)
{
    std::lock_guard<std::recursive_mutex> lock(mutex);
    auto deviceIter = std::find(begin(), end(), device);
    if (deviceIter == end())
        return false;

    log_debug(IS_LOG_DEVICE_MANAGER, "Releasing device '%s' on port '%s'", device->getIdAsString().c_str(), device->getPortName().c_str());

    if (device->devLogger) {
        device->devLogger->CloseAllFiles();
    }

    if (closePort && portIsValid(device->port) && portIsOpened(device->port)) {
        portClose(device->port);
    }

    if (device->port) {
        portToDeviceMap.erase(device->port);
    }
    erase(deviceIter); // erase only remove the device_handle_t from the list, but doesn't release/free the instance itself
    device->port = NULL;

    // also remove from knownDevices
    uint64_t devId = ENCODE_DEV_INFO_TO_UNIQUE_ID(device->devInfo);
    device_entry_t deviceEntry(nullptr, devId, nullptr);
    auto knownIter = std::remove_if(knownDevices.begin(), knownDevices.end(), [&deviceEntry](const device_entry_t& e) {
        if (e.hdwId == deviceEntry.hdwId) {
            // if we found the matching ID, populate the missing entry fields
            deviceEntry.factory = e.factory;
            deviceEntry.device = e.device;
            return true;
        }
        return false;
    });
    knownDevices.erase(knownIter);

    if (deleteDevice)
        deviceEntry.factory->releaseDevice(deviceEntry.device);

    return true;
}

/**
 * Called by the DeviceFactories when a device is identified.
 * This is a low-level callback indicating that a device identified by devInfo was discovered.
 * This handler is responsible for makings further determinations of viability and making callbacks
 * for specifics events such as whether the device is a new device, or a previously known device.
 * NOTE: this function will likely be called frequently, since it will be called once for each device
 * identified, for each locator register, for every call of checkForNewPort()
 * @param factory - the factory which discovered this device
 * @param devInfo - the device info for the discovered device
 * @param port - the port the device was discovered on, if any
 */
bool DeviceManager::deviceHandler(DeviceFactory *factory, const dev_info_t &devInfo, port_handle_t port, int options) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    options = (options != OPTIONS_USE_DEFAULTS) ? options : managementOptions;
    uint64_t devId = ENCODE_DEV_INFO_TO_UNIQUE_ID(devInfo);
    if (!devId) {
        if (options & DISCOVERY__CLOSE_PORT_ON_FAILURE)
            portClose(port);
        return false; // this is an invalid device Id -- no hdwId and no serialNo
    }

    device_entry_t deviceEntry(factory, devId, nullptr);

    // check if Device is previously known
    for (auto& kd : knownDevices) {
        if ((kd.factory == deviceEntry.factory) && (kd.hdwId == deviceEntry.hdwId)) {
            // We've re-discovered an old device, but we don't know the status of its port... we should try and figure that out, before we just blindly return...
            log_debug(IS_LOG_DEVICE_MANAGER, "Rediscovered previously known device [%s] on serial port '%s'.", ISDevice::getIdAsString(devInfo).c_str(), portName(port));
            device_handle_t device = getDevice(port);
            if (!device) {
                // If we weren't able to locate the Device by its port (perhaps because its not valid anymore, check by its device info instead)
                device = getDevice(devInfo.serialNumber, ENCODE_DEV_INFO_TO_HDW_ID(devInfo));
            }

            if (device) {
                if (!portIsValid(port)) {
                    // FIXME: if we're here, it means we had a deviceEntry that matched the discovered device, but its associated device is invalid.
                    //  we don't want to reallocate the device, since there is already one there, but just need to reassign the devInfo, etc.
                    //  Don't forget to remove the old device entry from the primary device set!!
                    log_debug(IS_LOG_DEVICE_MANAGER, "Device or port is invalid. Dropping device, and attempting a rebind on port '%s'.", portName(port));
                    notifyListeners(deviceEntry.device, DEVICE_PORT_LOST);
                    remove(deviceEntry.device);
                    //delete deviceEntry.device;
                    //deviceEntry.device = nullptr;
                    if (options & DISCOVERY__CLOSE_PORT_ON_FAILURE)
                        portClose(port);
                    break;  // we'll drop out of the 'for' loop, and still update known_devices and call the listeners, etc.
                } else {
                    device->assignPort(port);
                    if (device->port) {
                        portToDeviceMap[device->port] = device;
                    }
                    notifyListeners(device, DEVICE_PORT_BOUND);    // notify that this device's port has been updated
                }

                if (utils::compareDevInfo(device->devInfo, devInfo)) {
                    device->devInfo = devInfo;
                    notifyListeners(device, DEVICE_INFO_CHANGED);    // notify that this device's information changed (version, etc)
                }
            }

            if (options & DISCOVERY__CLOSE_PORT_ON_COMPLETION) {
                // notifyListeners(deviceEntry.device, DEVICE_DISCONNECTED);  technically we should send this, but conceptually, we never connected...
                portClose(port);
            }

            return true;    // successfully handled
        }
    }

    // if not, then we need to allocate it
    deviceEntry.device = factory->allocateDevice(devInfo, port);
    if (!deviceEntry.device) {
        if (options & DISCOVERY__CLOSE_PORT_ON_FAILURE)
            portClose(port);
        return false;   // allocated returned null, so no device created
    }

    // log_debug(IS_LOG_DEVICE_MANAGER, "Allocated new device: %s.", device->getDescription().c_str());
    knownDevices.push_back(deviceEntry);
    push_back(deviceEntry.device);
    if (deviceEntry.device->port) {
        portToDeviceMap[deviceEntry.device->port] = deviceEntry.device;
    }

    notifyListeners(deviceEntry.device, DEVICE_ADDED);  // notify

    if (portIsValid(deviceEntry.device->port))
        notifyListeners(deviceEntry.device, DEVICE_PORT_BOUND);  // notify that we're bound, even if we close the port below (because the port is still valid)

    if (options & DISCOVERY__CLOSE_PORT_ON_COMPLETION)
        portClose(port);

    if (portIsOpened(deviceEntry.device->port))
        notifyListeners(deviceEntry.device, DEVICE_CONNECTED);  // notify that we've connected (if we are)

    return true;    // successfully handled
}


void DeviceManager::portHandler(uint8_t event, uint16_t pType, std::string pName, port_handle_t port, PortFactory& factory) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    switch ((PortManager::port_event_e)event) {
        case PortManager::PORT_ADDED:
            // TODO: If "automatic device validation" is true, we should use this event to automatically open the port and validate the device.
            // log_more_debug(IS_LOG_DEVICE_MANAGER, "DeviceManager::portHandler( PORT_ADDED, '%s' )", pName.c_str());
            break;
        case PortManager::PORT_REMOVED:
            device_handle_t device = getDevice(port);
            if (device) {
                log_more_debug(IS_LOG_DEVICE_MANAGER, "DeviceManager::portHandler( PORT_REMOVED, '%s' ) - releasing port from %s.", pName.c_str(), device->getIdAsString().c_str());
                notifyListeners(device, DEVICE_PORT_LOST);
                device->assignPort(nullptr); // revoke the removed port from the device...
            }
            portToDeviceMap.erase(port);
            break;
    }
}

void DeviceManager::clear(bool closePorts) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    auto tmpSet = getDevicesAsVector();
    for (auto d : tmpSet) releaseDevice(d, closePorts, true);

    // just to make sure we didn't miss anything (though this could cause memory leaks)
    std::list<device_handle_t>::clear();
    knownDevices.clear();
    portToDeviceMap.clear();
}

/**
 * @returns a vector of available devices
 */
std::vector<device_handle_t> DeviceManager::getDevicesAsVector() {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    std::vector<device_handle_t> vecOut;
    for (auto device : *this) {
        vecOut.push_back(device);
    }
    return vecOut;
}

// TODO: FIXME: The following getDevice() should probably be refactored:
//  -- to use an overridden [] operator to get a device by uid, port, and string.
//  -- to consolidate the lookup as much as possible, so they are always consistent
//     in how they operate, and fail, etc.

/**
 * @returns an device_handle_t instance identified by the specified UID, or NULL if not found
 */
device_handle_t DeviceManager::getDevice(uint64_t uid) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    for (auto device : *this) {
        if (device && (ENCODE_DEV_INFO_TO_UNIQUE_ID(device->devInfo) == uid))
            return device;
    }
    return NULL;
}

/**
 * @returns an device_handle_t instance associated with the specified port, or NULL if not found
 */
device_handle_t DeviceManager::getDevice(port_handle_t port) {
    auto it = portToDeviceMap.find(port);
    if (it != portToDeviceMap.end()) {
        return it->second;
    }
    return NULL;
}

/**
 * @returns an device_handle_t instance at the specified index, or NULL if not found
 */
device_handle_t DeviceManager::getDeviceByIndex(int index) {
    if ((index < 0) || (index >= (int)size())) {
        return NULL;
    }
    auto it = begin();
    std::advance(it, index);
    return *it;
}

/**
 * @returns an device_handle_t instance identified by the deviceId string (as provided by ISDevice::getIdAsString()), or NULL if not found
 */
device_handle_t DeviceManager::getDevice(const std::string& deviceId) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    for (auto device : *this) {
        if (device->getIdAsString() == deviceId)
            return device;
    }
    return NULL;
}

/**
 * Returns the first ISDevice instance matching the specified criteria.
 * @param serialNum the serial number of the device to return
 * @param hdwId an optional hdwId to further filter on
 * @return an device_handle_t instance or NULL if not found
 */
device_handle_t DeviceManager::getDevice(uint32_t serialNum, is_hardware_t hdwId) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    for (const auto& device : *this) {
        if (device && device->matchesHdwId(hdwId, serialNum))
            return device;
    }

    // we didn't find it locally, so lets check out knownDevices set
    uint64_t uid = ENCODE_UNIQUE_ID(hdwId, serialNum);
    for (const auto& de : knownDevices) {
        if (de.hdwId == uid)
            return de.device;
    }

    return NULL;
}


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
std::vector<device_handle_t> DeviceManager::selectByDevInfo(const dev_info_t &devInfo, uint32_t filterFlags) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    std::vector<device_handle_t> selected;

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
 * @return a vector of device_handle_t which match the filter criteria (hdwId)
 */
std::vector<device_handle_t> DeviceManager::selectByHdwId(const uint16_t hdwId) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
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

/**
 * @returns a map of devices (key) which can be upgraded with the firmware image version (value) to
 *  which it can be upgraded to.
 */
std::vector<std::pair<device_handle_t, std::string>> DeviceManager::getUpgradableDevices(const std::string& firmwarePath) {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    std::vector<std::pair<device_handle_t, std::string>> results;

    // first, let's check the images path and find the latest firmware image
    std::vector<std::string> fwImageFiles;
    ISFileManager::GetAllFilesInDirectory(firmwarePath, true, "", fwImageFiles);

    // build a map of devInfo to files; we want to use a custom sorted map, where the map is ordered by version info
    std::map<uint16_t, std::map<dev_info_t, std::string, decltype(&utils::compareFirmwareVersions)>> fwImages;

    for (auto imgPath : fwImageFiles) {
        std::string imgDir, imgFile, imgExt;
        ISFileManager::getPathComponents(imgPath, imgDir, imgFile, imgExt);
        dev_info_t tmpDevInfo = {};
        printf("Evaluating %s\n", imgFile.c_str());
        if (utils::devInfoFromString(imgFile, tmpDevInfo)) {
            auto hdwId = ENCODE_DEV_INFO_TO_HDW_ID(tmpDevInfo);
            if (fwImages.find(hdwId) != fwImages.end())
                fwImages[hdwId] = std::map<dev_info_t, std::string, decltype(&utils::compareFirmwareVersions)>(&utils::compareFirmwareVersions);
            fwImages[hdwId][tmpDevInfo] = imgPath;
        }
    }

    for (auto device : *this) {
        auto hdwId = ENCODE_DEV_INFO_TO_HDW_ID(device->devInfo);
        for (auto& img : fwImages[hdwId]) {
            if (utils::isDevInfoCompatible(device->devInfo, (const dev_info_t&)img.first)) {
                // check if the img version is newer than the device's version
                const dev_info_t& deviceVersion = (const dev_info_t&)device->devInfo;
                const dev_info_t& imageVersion = (const dev_info_t&)img.first;
                // returns true if imageVersion is greater than deviceVersion
                if (utils::compareFirmwareVersions(imageVersion, deviceVersion)) {
                    results.push_back(std::make_pair(device, img.second));
                    break;
                }
            }
        }
    }

    return results;
}
