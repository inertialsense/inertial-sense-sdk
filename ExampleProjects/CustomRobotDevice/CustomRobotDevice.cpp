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


bool CustomRobotDevice::configure() {
    if (!isConnected())
        return false;

    // Devices can be configured to stream data by default on powerup - lets stop all other messages before enabling ours
    
    // Stop all message broadcasts from the device (in case any messaging was persistently enabled previously);
    // true argument means all ports
    if (StopBroadcasts(true) < 0) {
        log_msg(IS_LOG_ISDEVICE, IS_LOG_LEVEL_ERROR, "Failed to send \"Stop Broadcasts\" request." );

        return false;
    }
    
    // Enable message broadcasting
    // GetData(DID_SYS_PARAMS, 0, 0, 100);         // Request SYS_PARAMS every 100 ms (SYS_PARAMS is ran on the 1ms "Maintenance Task")
    // GetData(DID_GPX_STATUS, 0, 0, 100);         // Request GPX_STATUS every 100 ms (GPX_STATUS is ran on the 1ms "Maintenance Task")
    // GetData(DID_GNSS1_POS, 0, 0, 1);             // Request GPS1_POS every nvm_flash_cfg_t.startupGnssDtMs * 1 period
    // GetData(DID_GNSS1_RTK_POS_REL, 0, 0, 1);     // Request GPS1_RTK_POS_REL every nvm_flash_cfg_t.startupGnssDtMs * 1 period

    // Let's stream DID_INS_1
    // Stream at 1/25th the default DID_INS_1 rate (device dependent, but approx 1x = 7ms)
    bool bcast_success = BroadcastBinaryData(DID_INS_1, 25);

    if (!bcast_success) {
        log_msg(IS_LOG_ISDEVICE, IS_LOG_LEVEL_ERROR, "Failed to send \"Broadcast Binary Data\" request." );
        return false;
    }

    return true;
}

/**
 * Steps the communications for this device
 */
bool CustomRobotDevice::step() {
    /** Custom step operations here */
    
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
