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
// #include "DeviceManager.h"

typedef ISDevice* (*ISDeviceFactoryMethod)();

/**
 * An interface which, to be implemented as a singleton, that is responsible for identifying various types of ISDevice and allocating them
 */
class DeviceFactory {
public:
    virtual ~DeviceFactory() = default;

    /**
     * A function to be implemented in the factory responsible for allocating the underlying device type and returning a pointer to it
     * This function should NOT manipulate the underlying port, such as opening, etc.
     * @param devInfo the device information uniquely identifying the specific device
     * @return a ISDevice pointer to the newly allocated ISDevice or null of not allocated
     */
    virtual ISDevice* allocateDevice(const dev_info_t &devInfo) = 0;

    /**
     * A function responsible for freeing the allocated memory of the ISDevice instance.
     * This function should NOT manipulate the Device's underlying port, such as flushing, closing it, etc.
     * Because this does not close the port, the port may remain open, and so may be discovered at a later point
     * assuming the device is still physically connected to that port, and the port is valid.
     * @param device the pointer/handle of the Device to release
     * @return true if the device specified was a valid, and it was successfully released, otherwise false.
     */
    virtual bool releaseDevice(ISDevice* device) { return false; };

    /**
     * @param deviceCallback - A function to be called when this Factory identifies a possible Inertial Sense device; callback parameters are factory and the device information
     */
    virtual void locateDevices(std::function<void(DeviceFactory*, const dev_info_t&, port_handle_t)> deviceCallback, uint16_t hdwId);

    virtual void locateDevice(std::function<void(DeviceFactory*, const dev_info_t&, port_handle_t)> deviceCallback, port_handle_t port, uint16_t hdwId);


    uint32_t discoveryTimeout = 10000;
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

    virtual ISDevice* allocateDevice(const dev_info_t &devInfo) override {
        if (ENCODE_DEV_INFO_TO_HDW_ID(devInfo) == IS_HARDWARE_IMX_5_0)
            return new ISDevice(devInfo, nullptr);

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

    ISDevice* allocateDevice(const dev_info_t &devInfo) override {
        if (ENCODE_DEV_INFO_TO_HDW_ID(devInfo) == IS_HARDWARE_GPX_1_0)
            return new ISDevice(devInfo, nullptr);

        return nullptr;
    }
};


#endif //IS_SDK_DEVICE_FACTORY_H
