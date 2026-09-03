/**
 * @file RobotDeviceFactory.h
 * @brief ${BRIEF_DESC}
 *
 * @remark Based upon the SDK src/DeviceFactory.h
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef ROBOT_DEVICE_FACTORY_H
#define ROBOT_DEVICE_FACTORY_H

#ifdef __cplusplus

/** STEP 3:  Create a custom child class that inherits from DeviceFactory
 */

/**Include IS core and other needed SDK header files here;
 * include any of your own custom application device definition headers
 */
#include "DeviceFactory.h"
#include "RobotDevice.h"

/** @brief DeviceFactory singleton for IMX devices; allocates an ISDevice only for devices whose hardware
 * type resolves to IS_HARDWARE_TYPE_IMX, matching any IMX hardware version.
 */
class RobotDeviceFactory : public DeviceFactory {
public:
    /**
     * @brief Gets the singleton instance of this factory.
     * @return reference to the singleton RobotDeviceFactory, as a DeviceFactory.
     */
    static DeviceFactory& getInstance() {
        static RobotDeviceFactory instance;
        return instance;
    }

private:
    RobotDeviceFactory() = default;
    ~RobotDeviceFactory() override = default;

    /**
     * @brief Implements DeviceFactory::allocateDevice(); allocates an ISDevice only if devInfo resolves to IMX hardware.
     * @param devInfo the device information uniquely identifying the specific device.
     * @param port an associated port (optional) that this device should be bound to.
     * @return a new ISDevice if devInfo is IMX hardware, otherwise nullptr.
     */
    device_handle_t allocateDevice(const dev_info_t &devInfo, port_handle_t port) override {

        /** When we find IMX dev info (any hardware version), we know we want to allocate a new custom device */
        if (devInfo.hardwareType == IS_HARDWARE_TYPE_IMX)
            return std::make_shared<RobotDevice>(devInfo, port);

        return nullptr;
    }
};


#endif //  __cplusplus

#endif // ROBOT_DEVICE_FACTORY_H
