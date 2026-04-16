/**
 * @file DeviceFactory.cpp
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/10/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "DeviceFactory.h"

#include "DeviceManager.h"

/**
 * Attempts to identify a specific type of device on all currently known ports, within the timeout period
 * @param deviceCallback a function to be called if this Factory identified a possible/viable Inertial Sense device on the specified port
 * @param hdwId a hardware Id qualifier that can be used to narrow the type of device
 * @param timeout the maximum time to attempt to identify a device before giving up
 * @return true if a device was detected, otherwise false (indicating a timeout occurred)
 */
void DeviceFactory::locateDevices(std::function<bool(DeviceFactory*, const dev_info_t&, port_handle_t)>& deviceCallback, uint16_t hdwId, uint16_t timeoutMs) {
    for (auto& port : PortManager::getInstance()) {
        locateDevice(deviceCallback, port, hdwId, timeoutMs);
    }
}

/**
 * Blocking single-port discovery. Implemented as a wrapper around the three-phase validation protocol.
 */
bool DeviceFactory::locateDevice(std::function<bool(DeviceFactory*, const dev_info_t&, port_handle_t)>& deviceCallback, port_handle_t port, uint16_t hdwId, uint16_t timeoutMs) {
    // Check for existing device on this port
    device_handle_t dev = DeviceManager::getInstance().getDevice(port);
    if (dev) {
        if (dev->matchesHdwId(hdwId)) {
            if (!dev->port)
                dev->assignPort(port);
            return dev->validate(timeoutMs);
        }
        return false;
    }

    // Phase 1: begin validation
    auto ctx = beginValidation(port, hdwId, timeoutMs);
    if (!ctx)
        return false;

    // Phase 2: blocking loop
    while (!ctx->complete) {
        stepValidation(*ctx);
        if (!ctx->complete)
            SLEEP_MS(2);
    }

    // Phase 3: complete validation
    dev_info_t devInfo;
    if (completeValidation(*ctx, devInfo)) {
        return deviceCallback(this, devInfo, port);
    }
    return false;
}

/**
 * Phase 1: Opens the port, validates it, creates a base ISDevice for probing, and calls the
 * onBeginValidation() hook to allow the factory to decline or do custom setup.
 */
std::unique_ptr<DeviceFactory::ValidationContext> DeviceFactory::beginValidation(port_handle_t port, uint16_t hdwId, uint32_t timeoutMs, std::shared_ptr<ISDevice> sharedDevice) {
    if (!portIsValid(port)) {
        log_more_debug(IS_LOG_DEVICE_FACTORY, "beginValidation: port '%s' is invalid, skipping.", portName(port));
        return nullptr;
    }

    // Open port if needed (only when no shared device provided, i.e. standalone/blocking path)
    if (!sharedDevice && !portIsOpened(port)) {
        if (!portValidate(port) || (portOpen(port) != PORT_ERROR__NONE)) {
            portClose(port);
            portInvalidate(port);
            return nullptr;
        }
    }

    // Only COMM ports can be validated via ISComm protocol
    if (!(portType(port) & PORT_TYPE__COMM)) {
        log_more_debug(IS_LOG_DEVICE_FACTORY, "beginValidation: port '%s' is not a COMM port (type=0x%04X), skipping.", portName(port), portType(port));
        return nullptr;
    }

    if (timeoutMs <= 0)
        timeoutMs = deviceTimeout;

    // Let the factory decline this port
    if (!onBeginValidation(port, hdwId)) {
        log_more_debug(IS_LOG_DEVICE_FACTORY, "beginValidation: factory declined port '%s'.", portName(port));
        return nullptr;
    }

    log_more_debug(IS_LOG_DEVICE_FACTORY, "beginValidation: created validation context for port '%s' (hdwId=0x%04X, timeout=%dms)", portName(port), hdwId, timeoutMs);
    auto ctx = std::make_unique<ValidationContext>();
    ctx->port = port;
    ctx->device = sharedDevice ? sharedDevice : std::make_shared<ISDevice>(hdwId, port);
    ctx->hdwId = hdwId;
    ctx->timeoutMs = timeoutMs;

    // Check for a pre-seeded hint (e.g., from RelayPortFactory). If the relay has already
    // identified this device, skip the DID_DEV_INFO probe entirely.
    const dev_info_t* hint = DeviceManager::getInstance().getDeviceHint(port);
    if (hint && hint->serialNumber != 0 && hint->hardwareType != IS_HARDWARE_TYPE_UNKNOWN) {
        ctx->device->devInfo = *hint;
        ctx->device->hdwId = ENCODE_DEV_INFO_TO_HDW_ID((*hint));
        ctx->complete = true;
        ctx->result = 1;
        log_info(IS_LOG_DEVICE_FACTORY, "beginValidation: using seeded hint for port '%s' (SN=%u, hwType=%d) — skipping probe.",
                 portName(port), hint->serialNumber, hint->hardwareType);
    }

    return ctx;
}

/**
 * Phase 2: Reads pending data from the port, then calls the onStepValidation() hook to
 * advance validation state.
 */
int DeviceFactory::stepValidation(ValidationContext& ctx) {
    if (ctx.complete)
        return ctx.result;

    is_comm_port_parse_messages(ctx.port);
    ctx.result = onStepValidation(*ctx.device, ctx.timeoutMs);
    ctx.complete = (ctx.result != 0);
    if (ctx.result == 1) {
        log_debug(IS_LOG_DEVICE_FACTORY, "stepValidation: port '%s' validated successfully.", portName(ctx.port));
    } else if (ctx.result == -1) {
        log_debug(IS_LOG_DEVICE_FACTORY, "stepValidation: port '%s' timed out.", portName(ctx.port));
    }
    return ctx.result;
}

/**
 * Phase 3: Checks that validation succeeded and the device has info, then calls the
 * onCompleteValidation() hook to let the factory accept or reject.
 */
bool DeviceFactory::completeValidation(ValidationContext& ctx, dev_info_t& devInfoOut) {
    if (ctx.result <= 0)
        return false;
    if (!ctx.device || !ctx.device->hasDeviceInfo()) {
        log_debug(IS_LOG_DEVICE_FACTORY, "completeValidation: port '%s' validated but has no device info.", portName(ctx.port));
        return false;
    }
    if (!onCompleteValidation(ctx.device->devInfo, ctx.hdwId)) {
        log_debug(IS_LOG_DEVICE_FACTORY, "completeValidation: factory rejected device on port '%s' (hdwId=0x%04X).", portName(ctx.port), ctx.hdwId);
        return false;
    }

    devInfoOut = ctx.device->devInfo;
    log_debug(IS_LOG_DEVICE_FACTORY, "completeValidation: factory accepted device %s on port '%s'.",
        ISDevice::getIdAsString(devInfoOut).c_str(), portName(ctx.port));
    return true;
}