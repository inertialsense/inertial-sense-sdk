/**
 * @file CustomRobotDevice.cpp
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the Example Projects NTRIP_rover, Simple_Discovery, Minimal_ISDevice
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

/** STEP 5: Implement the required functions for a new ISDevice
 */

/** Include C++ libraries for use by your custom port class member functions defined here
 */
//#include <regex>

/** Include the header file for child port factory class derived from parent PortFactory.h
 */
#include "CustomRobotDevice.h"

/** Include utility functions for use by custom port factory class member functions defined here
 */
#include "ISUtilities.h"
#include "core/msg_logger.h"
#include "ISDisplay.h"

/**
 * Called by our application to set up this device and control the data broadcast
 * @return true or false for success of broadcast control commands
 */
bool CustomRobotDevice::configure() {
    if (!isConnected())
        return false;

    /** Devices can be configured to stream data by default on powerup - lets stop all other messages before enabling ours    
     * Stop all message broadcasts from the device (in case any messaging was persistently enabled previously);
     * true argument means all ports
     */
    if (StopBroadcasts(true) < 0) {
        log_msg(IS_LOG_ISDEVICE, IS_LOG_LEVEL_ERROR, "Failed to send \"Stop Broadcasts\" request." );
        return false;
    }
    
    /** Let's stream DID_INS_1 at 1/25th the default DID_INS_1 rate (device dependent, but approx 1x = 7ms)
     * Add any other streams desired here, as shown in the commented-out example
     * For example, add request SYS_PARAMS every 100 ms (SYS_PARAMS is ran on the 1ms "Maintenance Task") 
     */
    //bool bcast_success =
    //BroadcastBinaryData(DID_INS_1, 25)
        //&& BroadcastBinaryData(DID_SYS_PARAMS, 100)
    //  ;

    std::vector<std::function<bool()>> bcast_calls = {
        []() { return BroadcastBinaryData(DID_INS_1, 25); } //,
       //[]() { return BroadcastBinaryData(DID_SYS_PARAMS, 100); }  
    };

    // iterate and execute, logging failures if any
    for (const auto& bcast_success : bcast_calls) {
        if (!bcast_success) {
            log_msg(IS_LOG_ISDEVICE, IS_LOG_LEVEL_ERROR, "Failed to send \"Broadcast Binary Data\" request." );
            return false;
        }
    }

    
    // if (!bcast_success) {
    //     log_msg(IS_LOG_ISDEVICE, IS_LOG_LEVEL_ERROR, "Failed to send \"Broadcast Binary Data\" request." );
    //     return false;
    // }

    return true;
}

/**
 * Steps the communications for this device
 * @return the results of ISDevice step
 */
bool CustomRobotDevice::step() {
    /** Custom step operations here if desired */
    
    return ISDevice::step();            // call the parent's step() function to do all the usual ISDevice functions
}


/**
 * This is a callback handler that we will register with the ISDevice once its created, and which will be called every time data arrives from the device
 * @param data a pointer to a p_data_t struct, which represents the buffer of data received from the device, including the data ID, associated flags,
 *   and the actual data payload
 * @param port the port_handle_t that this data was received from
 * @returns 0 if this message was successfully processed by a protocol-specific handler, and should not be further processed, otherwise return !0
 */
int CustomRobotDevice::onIsbDataHandler(p_data_t* data, port_handle_t port) {

    if ( ISDevice::onIsbDataHandler(data, port) ) {    // let ISDevice do its handling
    
        if (data->hdr.id == DID_INS_1) {
            
            copyDataPToStructP(&insData, data, sizeof(ins_1_t));
            //TODO
            std::cout << isDisplay.DataToString((const p_data_t*)data);
        }

        return 0;  //success
    }

    return 1;
}
