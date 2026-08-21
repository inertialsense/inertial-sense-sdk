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
#include <regex>

/** Include the header file for child port factory class derived from parent PortFactory.h
 */
#include "CustomVirtualPortFactory.h"

/** Include utility functions for use by custom port factory class member functions defined here
 */
#include "ISUtilities.h"
#include "core/msg_logger.h"



bool CustomRobotDevice::configure() {
    if (!isConnected())
        return false;

    // Stop all message broadcasts from the device (in case any messaging was persistently enabled previously)
    if (StopBroadcasts(true)) {
        printf("Failed to send \"Stop Broadcasts\" request.\r\n");
        return -5;
    }
    
    // Enable message broadcasting
    GetData(DID_SYS_PARAMS, 0, 0, 100);         // Request SYS_PARAMS every 100 ms (SYS_PARAMS is ran on the 1ms "Maintenance Task")
    GetData(DID_GPX_STATUS, 0, 0, 100);         // Request GPX_STATUS every 100 ms (GPX_STATUS is ran on the 1ms "Maintenance Task")
    GetData(DID_GNSS1_POS, 0, 0, 1);             // Request GPS1_POS every nvm_flash_cfg_t.startupGnssDtMs * 1 period
    GetData(DID_GNSS1_RTK_POS_REL, 0, 0, 1);     // Request GPS1_RTK_POS_REL every nvm_flash_cfg_t.startupGnssDtMs * 1 period
    
    return true;
}

bool CustomRobotDevice::step() override {
    if ((GNSS_STATUS_FIX_MASK & gps.status) >= GNSS_STATUS_FIX_3D) {
        // Once we have a GNSS position, we can start to do NTRIP things...
        if (!ntrip.isConnected())
            ntrip.connect(ntripUrl);    // if we're not connected, connect
        else
            ntrip.step();               // if we are connected, call the CorrectionService's step() function to process and forward received data from the service
    }
    
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
int CustomRobotDevice::onIsbDataHandler(p_data_t* data, port_handle_t port) override {
    // printf("Received ISB message [%d]...\n", data->hdr.id);
    switch (data->hdr.id)
        {
        case DID_SYS_PARAMS:
            sysParams = *(sys_params_t*)data->ptr;
            if (lastImxUptime > sysParams.upTime) {
                printf("IMX reset detected... \n");
            }
            lastImxUptime = sysParams.upTime;
            break;
        case DID_GPX_STATUS:
            gpxStatus = *(gpx_status_t*)data->ptr;
            if (lastGpxUptime > gpxStatus.upTime) {
                printf("GPX reset detected... \n");
            }
            lastGpxUptime = gpxStatus.upTime;
            break;
            
        case DID_GNSS1_RTK_POS_REL:
            rel = *(gnss_rtk_rel_t*)data->ptr;
            break;
            
        case DID_GNSS1_POS:
            gps = *(gnss_pos_t*)data->ptr;
            
            std::string fix;
            switch (gps.status&GNSS_STATUS_FIX_MASK)
                {
                default:                        fix = "None      ";        break;
                case GNSS_STATUS_FIX_3D:         fix = "3D        ";        break;
                case GNSS_STATUS_FIX_RTK_SINGLE: fix = "RTK-Single";        break;
                case GNSS_STATUS_FIX_RTK_FLOAT:  fix = "RTK-Float ";        break;
                case GNSS_STATUS_FIX_RTK_FIX:    fix = "RTK       ";        break;
                }
            
            auto stats = ntrip.getMessageStats();
            int baseMsgCount = 0;
            for (auto& [msgId, msgStats] : stats->rtcm3) {
                baseMsgCount += msgStats.count;
            }
            
            printf("LL %12.9f %12.9f, hacc %4.2fm, age %3.1fs, fix-%s  %s\n",
                   gps.lla[0],
                   gps.lla[1],
                   gps.hAcc,
                   rel.differentialAge,    // time since last base message
                   fix.c_str(),
                   (gps.status&GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_DATA_MISSING ? "BASE: No data" : (std::string("BASE: ")+std::to_string(baseMsgCount).c_str())).c_str()
                   );
            
            // Forward our position via GGA every 5 seconds to the RTK base.
            static time_t lastTime;
            time_t currentTime = time(NULLPTR);
            if (abs(currentTime - lastTime) > 5)
                {   // Update every 5 seconds
                    lastTime = currentTime;
                    if ((gps.status&GNSS_STATUS_FIX_MASK) >= GNSS_STATUS_FIX_3D)
                        {   // GPS position is valid
                            ntrip.updatePosition(gps);
                            // printf("Sending position to Base: \n%s\n", std::string(rxBuf,n).c_str());
                        }
                    else
                        {
                            printf("Waiting for fix...\n");
                        }
                }
            break;
        }
    return ISDevice::onIsbDataHandler(data, port); // be sure to call the parent implementation so we don't break ISDevice functionality
}

