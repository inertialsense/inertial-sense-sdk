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
//#include "CustomVirtualPortFactory.h"
#include "CustomRobotDevice.h"

/** Include utility functions for use by custom port factory class member functions defined here
 */
#include "ISUtilities.h"
#include "core/msg_logger.h"
#include "ISDisplay.h"


// bool CustomRobotDevice::configure() {
//     if (!isConnected())
//         return false;

//     // Stop all message broadcasts from the device (in case any messaging was persistently enabled previously)
//     if (StopBroadcasts(true)) {
//         printf("Failed to send \"Stop Broadcasts\" request.\r\n");
//         return -5;
//     }
    
//     // Enable message broadcasting
//     GetData(DID_SYS_PARAMS, 0, 0, 100);         // Request SYS_PARAMS every 100 ms (SYS_PARAMS is ran on the 1ms "Maintenance Task")
//     GetData(DID_GPX_STATUS, 0, 0, 100);         // Request GPX_STATUS every 100 ms (GPX_STATUS is ran on the 1ms "Maintenance Task")
//     GetData(DID_GNSS1_POS, 0, 0, 1);             // Request GPS1_POS every nvm_flash_cfg_t.startupGnssDtMs * 1 period
//     GetData(DID_GNSS1_RTK_POS_REL, 0, 0, 1);     // Request GPS1_RTK_POS_REL every nvm_flash_cfg_t.startupGnssDtMs * 1 period
    
//     return true;
// }

/**
 * Steps the communications for this device
 */
bool CustomRobotDevice::step() {
    /** Custom step operations here */
    
    return ISDevice::step();            // call the parent's step() function to do all the usual ISDevice functions
}


/**
 * This is a callback handler that we will register with the ISDevice once its created, and which will be called every time data arrives from the device
 * @param ctx this is an opaque context pointer for this message - in this example, it will be the ISDevice* that received it the message - the ISDevice
 *   still need to process the data that it receives, so we dereference this, and call OnIsbDataHandler()
 * @param data a pointer to a p_data_t struct, which represents the buffer of data received from the device, including the data ID, associated flags,
 *   and the actual data payload
 * @param port the port_handle_t that this data was received from
 * @returns 0 if this message was successfully processed by a protocol-specific handler, and should not be further processed, otherwise return !0
 */
int CustomRobotDevice::onIsbDataHandler(p_data_t* data, port_handle_t port) {

    if (data->hdr.id == DID_INS_1)
        std::cout << isDisplay.DataToString((const p_data_t*)data);

    return ISDevice::onIsbDataHandler(data, port);    // let ISDevice do additional handling;
}
