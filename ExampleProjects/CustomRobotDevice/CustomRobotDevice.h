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

/** STEP 4:  Create a custom child class that inherits from ISDevice
 */

/**Include IS core and other needed SDK header files here; 
 * include any of your own custom application port definition headers, the lower-level
 * code that defines the interface used by this custom port factory; in this case the new
 * virtual test ports
 */
#include "PortFactory.h"
#include "ISDevice.h"
#include "ISDisplay.h"

class CustomRobotDevice : public ISDevice {

public:
    /** Structures to hold the data we are interested in for this device. from data_sets.h */
    ins_1_t insData = {};

    /** instance of a utility class that handles printing/formatting of various data sets received from the device */
    cInertialSenseDisplay isDisplay = cInertialSenseDisplay(cInertialSenseDisplay::DMODE_PRETTY);
    
    CustomRobotDevice(const dev_info_t& _devInfo, port_handle_t _port) : ISDevice(_devInfo, _port) { }

    // add disable data command here?
    ~CustomRobotDevice() override = default;

    /** This is an optional configuration function to set up our device and pick the data we want to see, etc */
    bool configure();

    /** Used by every ISDevice to "run" and process messages */
    bool step() override;

    /** Various message handler types */
    int onIsbDataHandler(p_data_t* data, port_handle_t port) override;
    //int onIsbAckHandler(p_ack_t* ack, unsigned char packetIdentifier, port_handle_t port) override;
    //int onNmeaHandler(const unsigned char *msg, int msgSize, port_handle_t port) override;
    
};


#endif //  __cplusplus

#endif // CUSTOM_ROBOT_DEVICE_H
