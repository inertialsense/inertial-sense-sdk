/**
 * @file CustomRobotDevice.h
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the Example Projects NTRIP_rover, Simple_Discovery, Minimal_ISDevice
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef CUSTOM_ROBOT_DEVICE_H
#define CUSTOM_ROBOT_DEVICE_H

#ifdef __cplusplus

/** STEP 4:  Create a custom child class that inherits from PortFactory 
 */

/**Include IS core and other needed SDK header files here; 
 * include any of your own custom application port definition headers, the lower-level
 * code that defines the interface used by this custom port factory; in this case the new
 * virtual test ports
 */
#include "PortFactory.h"

class CustomRobotDevice : public ISDevice {

public:
    gnss_pos_t       gps = {};
    gnss_rtk_rel_t   rel = {};
    double lastImxUptime = 0;
    double lastGpxUptime = 0;

    CustomRobotDevice(const std::string& serPort, const std::string& ntrip_url) : ISDevice(), ntripUrl(ntrip_url) {
        // bind to the physical serial port (hardware) and assign to the device
        assignPort(SerialPortFactory::getInstance().bindPort(serPort, PORT_TYPE__UNKNOWN));

        // tell the NtripCorrectionService to forward the received corrections to this device's port
        ntrip.addPort(port);
    }

    // add disable data command here?
    ~CustomRobotDevice() override = default;

    bool configure();

    bool step() override;
    int onIsbDataHandler(p_data_t* data, port_handle_t port) override;
};


#endif //  __cplusplus

#endif // CUSTOM_ROBOT_DEVICE_H
