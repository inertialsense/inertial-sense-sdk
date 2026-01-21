/*
MIT LICENSE

Copyright 2014-2018 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string>
#include <memory>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/PortManager.h"
#include "../../src/DeviceManager.h"
#include "../../src/ISDevice.h"
#include "../../src/NtripCorrectionService.h"

// STEP 2:   Extend ISDevice to implement our capability, and specifically
//  override onIsbDataHandler() to parse the data we are interested in.
//  Technically, this is optional, but it simplifies the design by allowing
//  us to override onIsbDataHandler() to do our own processing.
class NtripRover : public ISDevice {

public:
    std::string ntripUrl;
    NtripCorrectionService ntrip;
    gps_pos_t       gps = {};
    gps_rtk_rel_t   rel = {};
    double lastImxUptime = 0;
    double lastGpxUptime = 0;

    NtripRover(const std::string& serPort, const std::string& ntrip_url) : ISDevice(), ntripUrl(ntrip_url) {
        // bind to the physical serial port (hardware) and assign to the device
        assignPort(SerialPortFactory::getInstance().bindPort(serPort, PORT_TYPE__UNKNOWN));

        // tell the NtripCorrectionService to forward the received corrections to this device's port
        ntrip.addPort(port);
    }

    ~NtripRover() override = default;

    bool configure() {
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
        GetData(DID_GPS1_POS, 0, 0, 1);             // Request GPS1_POS every nvm_flash_cfg_t.startupGPSDtMs * 1 period
        GetData(DID_GPS1_RTK_POS_REL, 0, 0, 1);     // Request GPS1_RTK_POS_REL every nvm_flash_cfg_t.startupGPSDtMs * 1 period

        return true;
    }

    bool step() override {
        if ((GPS_STATUS_FIX_MASK & gps.status) >= GPS_STATUS_FIX_3D) {
            // Once we have a GNSS position, we can start to do NTRIP things...
            if (!ntrip.isConnected())
                ntrip.connect(ntripUrl);    // if we're not connected, connect
            else
                ntrip.step();               // if we are connected, call the CorrectionService's step() function to process and forward received data from the service
        }

        return ISDevice::step();            // call the parent's step() function to do all the usual ISDevice functions
    }

    int onIsbDataHandler(p_data_t* data, port_handle_t port) override {
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

            case DID_GPS1_RTK_POS_REL:
                rel = *(gps_rtk_rel_t*)data->ptr;
                break;

            case DID_GPS1_POS:
                gps = *(gps_pos_t*)data->ptr;

                std::string fix;
                switch (gps.status&GPS_STATUS_FIX_MASK)
                {
                    default:                        fix = "None      ";        break;
                    case GPS_STATUS_FIX_3D:         fix = "3D        ";        break;
                    case GPS_STATUS_FIX_RTK_SINGLE: fix = "RTK-Single";        break;
                    case GPS_STATUS_FIX_RTK_FLOAT:  fix = "RTK-Float ";        break;
                    case GPS_STATUS_FIX_RTK_FIX:    fix = "RTK       ";        break;
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
                       (gps.status&GPS_STATUS_FLAGS_GPS1_RTK_BASE_DATA_MISSING ? "BASE: No data" : (std::string("BASE: ")+std::to_string(baseMsgCount).c_str())).c_str()
                );

                // Forward our position via GGA every 5 seconds to the RTK base.
                static time_t lastTime;
                time_t currentTime = time(NULLPTR);
                if (abs(currentTime - lastTime) > 5)
                {   // Update every 5 seconds
                    lastTime = currentTime;
                    if ((gps.status&GPS_STATUS_FIX_MASK) >= GPS_STATUS_FIX_3D)
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
};

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printf("Please pass the com port and the RTK base connection string as the 1st and 2nd arguments.\r\n");
        printf("In Visual Studio IDE, this can be done through \"Project Properties -> Debugging ->.\r\n");
        printf("Command Arguments\": /dev/ttyACM0 ntrip://<username>:<password>@192.168.1.100:7777/<mountpoint>\r\n");
        return -1;
    }

    std::string serialPort(argv[1]);
    std::string ntripUrl(argv[2]);

    NtripRover myRover(serialPort, ntripUrl);   // instantiate our custom ISDevice

    if (!myRover.connect()) {
        printf("Unable to connect to the specified port.\r\n");
        return -2;
    }

    myRover.configure();    // Once we're connected, we need to configure the device

    // Main loop
    while (1) {
        myRover.step();
        SLEEP_MS(1);
    }
}

