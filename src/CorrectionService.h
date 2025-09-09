/**
 * @file CorrectionService.h
 * @brief Class used to distribute an incoming RTCM stream of corrections to "Rover" devices
 *
 * @author FiriusFoxx on 9/4/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#ifndef IS_SDK__CORRECTION_SERVICE_H
#define IS_SDK__CORRECTION_SERVICE_H

#include <vector>

#include "core/types.h"
#include "core/base_port.h"
#include "ISConstants.h"

#include "PortFactory.h"
#include "ISDevice.h"

/**
 * The CorrectionService is a "Rover"-side manager responsible for establishing a connection to
 * a correction service, such as an NTRIP caster, or raw RTCM3 provider, reading correction data
 * from the source port, and forwarding it to one or more associated ISDevices.
 */
class CorrectionService {
public:

    /**
     * The base constructor, binds the specified port as the source for corrections data that
     * will be forwarded to all associated ISDevices
     * @param srcPort The port on which incoming RCTM3 data to recieved from
     */
    explicit CorrectionService(port_handle_t srcPort);

    /**
     * A convenience constructor which creates/binds the necessary port as described by the portName, by
     * attempting to allocate the named port from each of the provided factories.  This is particularly
     * useful when the port is not directly discoverable, such as TCP, where the portName is a URL to
     * a remote TCP server.
     * @param portName The name of the port to be allocated
     * @param factories The factories to ask to create the given port name
     */
    CorrectionService(const std::string& portName, const std::vector<PortFactory*>& factories);

    /**
     * A convenience constructor which creates/binds the necessary port as described by the portName, by
     * attempting to allocate the named port from each of the factories provided to PortManager. This is
     * particularly useful when the port is not directly discoverable, such as TCP, where the portName is
     * a URL to a remote TCP server.
     * @param portName The name of the port to be allocated
     */
    explicit CorrectionService(const std::string& portName);

    /**
     * Adds a device to the recieve corrections from this service
     * @param device A reference to the device to send corrections to
     */
    void addDevice(ISDevice* device);

    /**
     * Adds multiple devices to recieve corrections from this service
     * @param devices A vector of references to devices to send corrections to
     */
    void addDevices(const std::vector<ISDevice*>& devices) { for (auto& d : devices) { addDevice(d); } }

    /**
     * Cease sending corrections from this service a device
     * @param device A reference to the device to cease sending corrections to
     */
    void removeDevice(ISDevice* device);

    /**
     * Cease sending corrections from this service to multiple devices
     * @param devices A vector of references to devices to cease corrections to
     */
    void removeDevices(const std::vector<ISDevice*>& devices) { for (auto& d : devices) { removeDevice(d); } }

    /**
     * Check if this CorrectionService is sending correction data to a given device
     * @param device A reference of a device to check for
     */
    bool hasDevice(ISDevice* device);

    /**
     * Checks the source port for data and forwards it to all devices
     * @return 0 if no packets processed, positive number representing number of packets processed, negative number representing errno
     */
    int step();

private:
    port_handle_t source{};
    std::vector<ISDevice*> devices;
};


#endif //IS_SDK__CORRECTION_SERVICE_H
