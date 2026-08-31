/**
 * @file CustomDeviceFactory.h 
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the SDK src/DeviceFactory.h
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef CUSTOM_DEVICE_FACTORY_H
#define CUSTOM_DEVICE_FACTORY_H

#ifdef __cplusplus

/** STEP 3:  Create a custom child class that inherits from DeviceFactory
 */

/**Include IS core and other needed SDK header files here;
 * include any of your own custom application device definition headers
 */
#include <functional>
#include "DeviceFactory.h"
#include "CustomRobotDevice.h"

/** @brief DeviceFactory singleton for IMX-5 devices; allocates an ISDevice only for devices whose hardware
 * Id resolves to IS_HARDWARE_IMX_5_0.
 */
class CustomDeviceFactory : public DeviceFactory {
public:
    /**
     * @brief Gets the singleton instance of this factory.
     * @return reference to the singleton CustomDeviceFactory, as a DeviceFactory.
     */
    static DeviceFactory& getInstance() {
        static CustomDeviceFactory instance;
        return instance;
    }

    /**
     * @brief Sets the data-received callback that every device this factory allocates
     * will be wired to, so the application can share a single sink (e.g. a display) across every discovered
     * device
     * @param cb the callback to invoke with every parsed data message any allocated device receives.
     */
    static void setDataCallback(std::function<void(const p_data_t* data)> cb) {
        s_dataCallback = std::move(cb);
    }

private:
    CustomDeviceFactory() = default;
    ~CustomDeviceFactory() override = default;

    inline static std::function<void(const p_data_t* data)> s_dataCallback;

    /**
     * @brief Implements DeviceFactory::allocateDevice(); allocates an ISDevice only if devInfo resolves to an IMX-5 hardware Id.
     * @param devInfo the device information uniquely identifying the specific device.
     * @param port an associated port (optional) that this device should be bound to.
     * @return a new ISDevice if devInfo is an IMX-5, otherwise nullptr.
     */
    device_handle_t allocateDevice(const dev_info_t &devInfo, port_handle_t port) override {

        /** When we find IMX-5 dev info, we know we want to allocate a new custom device */
        if (ENCODE_DEV_INFO_TO_HDW_ID(devInfo) == IS_HARDWARE_IMX_5_0) {
            auto device = std::make_shared<CustomRobotDevice>(devInfo, port);
            device->onDataReceived = s_dataCallback;   // every device shares same sink the application provides
            return device;
        }

        return nullptr;
    }
};


#endif //  __cplusplus

#endif // CUSTOM_DEVICE_FACTORY_H
