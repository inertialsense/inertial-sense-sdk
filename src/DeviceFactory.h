/**
 * @file DeviceFactory.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/10/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_DEVICE_FACTORY_H
#define IS_SDK_DEVICE_FACTORY_H

#include <unordered_set>
#include <functional>
#include <string>
#include <cctype>

#include "core/msg_logger.h"
#include "core/base_port.h"
#include "ISConstants.h"
#include "ISDevice.h"
#include "PortManager.h"

/**
 * An interface, to be implemented as a singleton, that is responsible for identifying various types of ISDevice and allocating and releasing them.
 */
class DeviceFactory {
public:
    virtual ~DeviceFactory() = default;

    /**
     * A function to be implemented in the factory responsible for allocating the underlying device type and returning a pointer to it
     * This function should NOT manipulate the underlying port, such as opening, etc.
     * @param devInfo the device information uniquely identifying the specific device
     * @param port an associated port (optional) that this device should be bound to.
     * @return a ISDevice pointer to the newly allocated ISDevice or null of not allocated
     */
    virtual ISDevice* allocateDevice(const dev_info_t &devInfo, port_handle_t port = nullptr) { return new ISDevice(devInfo, port); };

    /**
     * A function responsible for freeing the allocated memory of the ISDevice instance.
     * This function should NOT manipulate the Device's underlying port, such as flushing, closing it, etc.
     * Because this does not close the port, the port may remain open, and so may be discovered at a later point
     * assuming the device is still physically connected to that port, and the port is valid.
     * @param device the pointer/handle of the Device to release
     * @return true if the device specified was valid, and it was successfully released, otherwise false.
     */
    virtual bool releaseDevice(ISDevice* device) {
        // cleanup some memory, so if this accidentally gets used after being free, it won't be catastrophic.
        device->port = nullptr;
        device->fwUpdater = nullptr;
        device->hdwId = IS_HARDWARE_NONE;
        device->devInfo.hdwRunState = HDW_STATE_UNKNOWN;

        delete device;
        return true;
    };

    /**
     * Attempts to identify a specific type of device on all currently known ports, within the timeout period
     * @param deviceCallback a function to be called if this Factory identified a possible/viable Inertial Sense device on the specified port
     * @param hdwId a hardware Id qualifier that can be used to narrow the type of device
     * @param timeout the maximum time to attempt to identify a device before giving up
     * @return true if a device was detected, otherwise false (indicating a timeout occurred)
     */
    virtual void locateDevices(std::function<void(DeviceFactory*, const dev_info_t&, port_handle_t)>& deviceCallback, uint16_t hdwId, uint16_t timeout);

    /**
     * Attempts to identify a specific type of device on the specified port, within the timeout period.
     * @param deviceCallback a function to be called if this Factory identified a possible/viable Inertial Sense device on the specified port
     * @param port the port to check for an Inertial Sense device. In most uses, the port specified should already be opened, however if the
     *   port is not opened, this function will attempt to open it in order for ensure discovery.
     * @param hdwId a hardware Id qualifier that can be used to narrow the type of device.  There is no direct indication that a hardware type
     *   failed to match.
     * @param timeout the maximum time to attempt to identify a device before giving up. There is no direct indication that a timeout occurred.
     * @return true if a device was detected, otherwise false. Note that a false can result for any number of reasons, including invalid port,
     *   hdwId mismatch, or a timeout.
     */
    virtual bool locateDevice(std::function<void(DeviceFactory*, const dev_info_t&, port_handle_t)>& deviceCallback, port_handle_t port, uint16_t hdwId, uint16_t timeout);

    /**
     * Assigns a Factory-specific timeout period for each port
     * @param timeout
     */
    void setPerDeviceTimeout(uint32_t timeout) { deviceTimeout = timeout; };

private:
    uint32_t deviceTimeout = 3000;
};

class ImxDeviceFactory : public DeviceFactory {
public:
    static DeviceFactory& getInstance() {
        static ImxDeviceFactory instance;
        return instance;
    }

private:
    ImxDeviceFactory() = default;
    // ~ImxDeviceFactory() override = default;

    virtual ISDevice* allocateDevice(const dev_info_t &devInfo, port_handle_t port = nullptr) override {
        if (ENCODE_DEV_INFO_TO_HDW_ID(devInfo) == IS_HARDWARE_IMX_5_0)
            return new ISDevice(devInfo, port);

        return nullptr;
    }
};

class GpxDeviceFactory : public DeviceFactory {
public:
    static DeviceFactory& getInstance() {
        static GpxDeviceFactory instance;
        return instance;
    }

private:
    GpxDeviceFactory() = default;
    // ~GpxDeviceFactory() override = default;

    ISDevice* allocateDevice(const dev_info_t &devInfo, port_handle_t port = nullptr) override {
        if (ENCODE_DEV_INFO_TO_HDW_ID(devInfo) == IS_HARDWARE_GPX_1_0)
            return new ISDevice(devInfo, port);

        return nullptr;
    }
};


#endif //IS_SDK_DEVICE_FACTORY_H
