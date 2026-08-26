/**
 * @file main.cpp
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the SDK ExampleProjects/ISComm/ISCommExample.cpp
 *
 * @details This demonstrates an example application creating a custom port factory 
 * with a virtual port implemented by a modified version of the test_serial_utils code
 *
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */


/** STEP 6: The example application */

/** Include user IO capabilities for this demo so when run visual indicators appear */
#include <stdio.h>

/** Include utility functions for use by your custom port class member functions defined here
 */
#include "ISUtilities.h"
#include "DeviceManager.h"
#include "ISDevice.h"

/** The port factory child class the user creates, inheriting from PortFactory.h definition */
#include "CustomRobotDevice.h"
#include "CustomDeviceFactory.h"

/** Include the SDK PortManager if desired for building and maintaining a list of all ports by name */
#include "PortManager.h"


/**
 * Uses arg for identifying virtual serial port in a "minimal" example of setting up a custom serial port
 * connection.  Use PortManager and PortFactory to bind a port_handle_t to the named port. With the handle,
 * the port is opened, which is in loopback mode in this case, and a simple write/read test is performed.
 */
int main_discovery(const char* portPattern)
{
  
    /** STEP 7: Set the SDK message logger verbosity level, as we use the logger for
     * some custom status and error messages
     */
    IS_SET_LOG_LEVEL(IS_LOG_LEVEL_INFO);


    std::shared_ptr<CustomRobotDevice> device = nullptr; // this will be our discovered device... but null for now.
    
    /** STEP 8: We need a singleton Port Manager and SerialPortFactory,
     * we'll make local references, and we register the virtual port factory
     */    
    PortManager& pm = PortManager::getInstance();
    pm.addPortFactory(&SerialPortFactory::getInstance());   // tell the PortManager that we are interested in Serial Ports

    DeviceManager& dm = DeviceManager::getInstance();
    dm.addDeviceFactory(&CustomDeviceFactory::getInstance());  // tell the DeviceManager that we are interested in Custom Devices

    /** Let the PM find all available ports, then ask for the one specifically indicated on the command line by name */
    int retry = 3;
    while (retry-- >= 0) {              // some port discovery mechanisms (mDNS, etc) may require multiple calls before ports begin to show
        pm.discoverPorts(portPattern);  // first let's attempt to discover ports - returns true, if the list of known ports changed (either added or removed), otherwise false
        if (!pm.empty()) {              // so let's also check that there is at least one port available
            // at least one port was found...

            // next let's attempt to discover devices on known ports - returns true if the list of known devices changed, but doesn't indicate if a device was found or lost...
            //  - we're interested in ANY IMX devices
            //  - we'll wait upto 1.5 seconds at most for the port to negotiate
            //  - and we want to override the default discovery options (DO NOT use DISCOVERY__IGNORE_CLOSED_PORTS, but still "Close On Failure")
            dm.discoverDevices(IS_HARDWARE_IMX, 1500, DeviceManager::DISCOVERY__CLOSE_PORT_ON_FAILURE);
            if (!dm.empty()) {                      // again, we're checking here that we have at least one known device which matched our selection criteria (ANY)

                // we found some devices  - The DeviceManager maintains an internal collection of all the devices that were detected.
                // From here, we can use a number of options to select all (getDevices) or specific (getDevice) from the list of available
                // devices.

                // Let's get the first device in our collection of discovered devices                
                device = dm.getDevices().front()->as<CustomRobotDevice>();
                break;  // we're done, so we'll break out of our retry loop.

            }
        }
        SLEEP_MS(500);  // give a brief pause for new ports/devices to become available.
    } //while

    if ( (retry < 0) && !device ) {
        // we failed to locate any devices - report and exit gracefully
        std::cout << "No Inertial Sense devices were found after 3 attempts. Exiting." << std::endl;
        exit(0);    // this is NOT an error
    }

    // from here, the rest of the example is the same as the Minimal_Device::main_minimal() example.
    // connect to the device
    if ( !device->connect() ) {
        std::cerr << "Could not connect to device on port " << device->getPortName() << std::endl;
        //exit(1);
        return -2;
    }

    printf("Found and connected %lu IMX device(s) on port(s): \r\n", dm.DeviceCount());
    for (const auto& dev : dm.getDevices()) {
        printf(" %s\r\n", dev->getPortName().c_str());
    }
    printf("\r\n");


    if ( !device->configure() ) {
        printf("Configuration fail\r\n");
        return -3;
    }

    while ( portIsOpened(device->port) ) {                // and then spin as long as the port is open
        device->step();                                 // process incoming data
        SLEEP_MS(10);                                   // we can sleep for spell and keep our CPU happy
    }

    return 0;    
} //main_discovery



/**
 * The main entry point for the application - note that this calls the discovery code
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, const char** argv) {

    const char* portPattern = "(.+)";   // NOTE: this is a MATCHING REGEX pattern (this one matches everything)

    if (argc > 2)
    {
        printf("Usage: No argument allows automatic port discovery based on universal pattern.  Or if desired, a single argument selects the port (i.e. /dev/ttyACM0)\r\n");
        return -1;
    }

    printf("CustomRobotDevice example application started (ctrl+\\ to quit)");
    if (argc == 2) {
        portPattern = argv[1];
        printf(", attempting to use port %s", portPattern);
    }
    else
        printf(", attempting port discovery");
    printf("\r\n");

    return main_discovery(portPattern);
} //main
