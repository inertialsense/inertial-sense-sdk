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
 *
 * Supports both blocking single-port discovery (locateDevice) and concurrent multi-port discovery
 * via a three-phase protocol (beginValidation / stepValidation / completeValidation) driven by
 * DeviceManager::discoverDevices(). Subclasses customize behavior by overriding the virtual hooks
 * (onBeginValidation, onStepValidation, onCompleteValidation) without managing ValidationContext.
 */
class DeviceFactory {
public:
    virtual ~DeviceFactory() = default;

    /**
     * Per-port, per-factory state during concurrent validation. Managed by the base class
     * beginValidation/stepValidation/completeValidation methods. Subclasses should not need
     * to interact with this directly — override the onXxx hooks instead.
     */
    struct ValidationContext {
        port_handle_t port = nullptr;
        std::shared_ptr<ISDevice> device;
        uint16_t hdwId = IS_HARDWARE_ANY;
        uint32_t timeoutMs = 3000;
        bool complete = false;
        int result = 0;  // -1 = failed/timeout, 0 = in-progress, 1 = success
    };

    /**
     * A function to be implemented in the factory responsible for allocating the underlying device type and returning a pointer to it
     * This function should NOT manipulate the underlying port, such as opening, etc.
     * @param devInfo the device information uniquely identifying the specific device
     * @param port an associated port (optional) that this device should be bound to.
     * @return a ISDevice pointer to the newly allocated ISDevice or null of not allocated
     */
    virtual device_handle_t allocateDevice(const dev_info_t &devInfo, port_handle_t port = nullptr) { return std::make_shared<ISDevice>(devInfo, port); };

    /**
     * A function responsible for freeing the allocated memory of the ISDevice instance.
     * This function should NOT manipulate the Device's underlying port, such as flushing, closing it, etc.
     * Because this does not close the port, the port may remain open, and so may be discovered at a later point
     * assuming the device is still physically connected to that port, and the port is valid.
     * @param device the pointer/handle of the Device to release
     * @return true if the device specified was valid, and it was successfully released, otherwise false.
     */
    virtual bool releaseDevice(device_handle_t device) {
        // cleanup some memory, so if this accidentally gets used after being free, it won't be catastrophic.
        device->port = nullptr;
        // device->fwUpdater = nullptr;
        device->hdwId = IS_HARDWARE_NONE;
        device->devInfo.hdwRunState = HDW_STATE_UNKNOWN;
        device.reset();
        // delete device;
        return true;
    };

    /**
     * Attempts to identify a specific type of device on all currently known ports, within the timeout period
     * @param deviceCallback a function to be called if this Factory identified a possible/viable Inertial Sense device on the specified port
     * @param hdwId a hardware Id qualifier that can be used to narrow the type of device
     * @param timeout the maximum time to attempt to identify a device before giving up
     * @return true if a device was detected, otherwise false (indicating a timeout occurred)
     */
    virtual void locateDevices(std::function<bool(DeviceFactory*, const dev_info_t&, port_handle_t)>& deviceCallback, uint16_t hdwId, uint16_t timeout);

    /**
     * Attempts to identify a specific type of device on the specified port, within the timeout period.
     * Implemented as a blocking wrapper around beginValidation/stepValidation/completeValidation.
     * @param deviceCallback a function to be called if this Factory identified a possible/viable Inertial Sense device on the specified port
     * @param port the port to check for an Inertial Sense device. In most uses, the port specified should already be opened, however if the
     *   port is not opened, this function will attempt to open it in order for ensure discovery.
     * @param hdwId a hardware Id qualifier that can be used to narrow the type of device.  There is no direct indication that a hardware type
     *   failed to match.
     * @param timeout the maximum time to attempt to identify a device before giving up. There is no direct indication that a timeout occurred.
     * @return true if a device was detected, otherwise false. Note that a false can result for any number of reasons, including invalid port,
     *   hdwId mismatch, or a timeout.
     */
    virtual bool locateDevice(std::function<bool(DeviceFactory*, const dev_info_t&, port_handle_t)>& deviceCallback, port_handle_t port, uint16_t hdwId, uint16_t timeout);

    /**
     * Phase 1: Prepare a ValidationContext for concurrent validation on this port.
     * If a shared ISDevice probe is provided, it will be used (concurrent discovery shares one
     * ISDevice per port across all factories). Otherwise, opens the port and creates a new probe.
     * Calls onBeginValidation() to allow the factory to decline or do custom setup.
     * @return a ValidationContext, or nullptr if this port cannot be validated
     */
    std::unique_ptr<ValidationContext> beginValidation(port_handle_t port, uint16_t hdwId, uint32_t timeoutMs, std::shared_ptr<ISDevice> sharedDevice = nullptr);

    /**
     * Phase 2: Non-blocking validation step. Reads pending data from the port, then calls
     * onStepValidation() to advance validation state. Called repeatedly during the concurrent
     * loop, only when this factory is the active round-robin slot for this port.
     * @return -1 = failed/timeout, 0 = still in progress, 1 = validated successfully
     */
    int stepValidation(ValidationContext& ctx);

    /**
     * Phase 3: After stepValidation returns 1, checks that the device has valid info and calls
     * onCompleteValidation() to let the factory accept or reject the device.
     * @param ctx the completed validation context
     * @param devInfoOut populated with the validated device info on success
     * @return true if this factory claims the device, false otherwise
     */
    bool completeValidation(ValidationContext& ctx, dev_info_t& devInfoOut);

    /**
     * Assigns a Factory-specific timeout period for each port
     * @param timeout
     */
    void setPerDeviceTimeout(uint32_t timeout) { deviceTimeout = timeout; };

    uint32_t getPerDeviceTimeout() const { return deviceTimeout; };

protected:
    /**
     * Virtual hook called during Phase 1. Override to decline certain ports or perform
     * custom port setup before validation begins.
     * @param port the port about to be validated
     * @param hdwId the hardware Id filter
     * @return true to proceed with validation, false to skip this port
     */
    virtual bool onBeginValidation(port_handle_t port, uint16_t hdwId) { (void)port; (void)hdwId; return true; }

    /**
     * Virtual hook called during Phase 2. Override to inject custom validation logic
     * (e.g., factory-specific queries or handshakes). The base class has already called
     * is_comm_port_parse_messages() before invoking this hook.
     * @param device the ISDevice probe to validate against
     * @param timeoutMs the timeout for this validation attempt
     * @return -1 = failed/timeout, 0 = still in progress, 1 = validated successfully
     */
    virtual int onStepValidation(ISDevice& device, uint32_t timeoutMs) { return device.validateAsync(timeoutMs); }

    /**
     * Virtual hook called during Phase 3. Override to accept or reject a validated device
     * based on factory-specific criteria (e.g., hardware type, firmware version).
     * @param devInfo the validated device info
     * @param hdwId the hardware Id filter from the discovery request
     * @return true to claim this device, false to pass to the next factory
     */
    virtual bool onCompleteValidation(const dev_info_t& devInfo, uint16_t hdwId) { (void)devInfo; (void)hdwId; return true; }

private:
    uint32_t deviceTimeout = 3000;  // should match DeviceManager::DISCOVERY__DEFAULT_TIMEOUT
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

    device_handle_t allocateDevice(const dev_info_t &devInfo, port_handle_t port) override {
        if (ENCODE_DEV_INFO_TO_HDW_ID(devInfo) == IS_HARDWARE_IMX_5_0)
            return std::make_shared<ISDevice>(devInfo, port);

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

    device_handle_t allocateDevice(const dev_info_t &devInfo, port_handle_t port) override {
        if (ENCODE_DEV_INFO_TO_HDW_ID(devInfo) == IS_HARDWARE_GPX_1_0)
            return std::make_shared<ISDevice>(devInfo, port);

        return nullptr;
    }
};


#endif //IS_SDK_DEVICE_FACTORY_H
