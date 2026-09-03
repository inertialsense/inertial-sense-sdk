/**
 * @file main.cpp
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the SDK ExampleProjects/Simple_Discovery/Example-2_Device-Discovery.cpp
 *
 * @details This demonstrates an example application creating a custom ISDevice extension and
 * custom device factory, and then streaming and processing data from IMX hardware that is
 * discovered using DeviceManager and PortManager/PortFactory.
 *
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */


/** STEP 4: The example application */

/** Include user IO capabilities for this demo so when run visual indicators appear */
#include <stdio.h>

/** We need the DeviceManager, and the custom device and device factory */
#include "DeviceManager.h"
#include "CustomRobotDevice.h"
#include "CustomDeviceFactory.h"

/** Include the SDK PortManager for building and maintaining a list of all ports by name */
#include "PortManager.h"


/**
 * Uses portPattern for discovering a device connected to a port in an example of setting up a custom device
 * connection. This is similar to previous examples, but leverages Port/Device discovery to locate a dynamic or
 * unknown device.  This will attempt to discover ports, retrying up to 3 times. Use PortManager and PortFactory
 * to bind a port_handle_t to the port.  Each time ports are discovered, the port is opened, and DeviceManager
 * and DeviceFactory search for a device of a certain hardware ID.  It attempts to discover devices on all known
 * ports.  If an IMX 5.0 device is found, the discovery phase will end, and a connection attempt
 * is made.  We then receive data from the found device.
 */
int main_discovery(const char* portPattern)
{
    /** STEP 5: Set the SDK message logger verbosity level, as we use the logger for
     * some custom status and error messages
     */
    IS_SET_LOG_LEVEL(IS_LOG_LEVEL_INFO);
    
    /** STEP 6: We need a singleton PortManager and SerialPortFactory, and DeviceManager and CustomDeviceFactory;
     * we'll make local references, and we register the virtual port factory
     */    
    PortManager& pm = PortManager::getInstance();
    pm.addPortFactory(&SerialPortFactory::getInstance());   // tell the PortManager that we are interested in Serial Ports

    DeviceManager& dm = DeviceManager::getInstance();
    dm.addDeviceFactory(&CustomDeviceFactory::getInstance());  // tell the DeviceManager that we are interested in Custom Devices

    /** STEP 7: Discover ports and devices, connect and configure device */
    std::shared_ptr<CustomRobotDevice> device = nullptr;      // this will be our discovered device... but null for now
    
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

    if ( !device ) {
        // we failed to locate any devices - report and exit gracefully
        std::cout << "No Inertial Sense devices were found after 3 attempts. Exiting." << std::endl;
        return 0;    // this is NOT an error
    }

    if ( !device->connect() ) {
        std::cerr << "Could not connect to device on port " << device->getPortName() << std::endl;
        return -2;
    }

    std::cout << "Found and connected " << dm.DeviceCount() << " IMX device(s) on port(s): " << std::endl;
    for (const auto& dev : dm.getDevices()) {
        std::cout << " " << dev->getPortName() << std::endl;
    }
    std::cout << std::endl;


    if ( !device->configure() ) {
        std::cout << "Configuration fail" << std::endl;
        return -3;
    }

    /** STEP 8: Read and process device data in a loop */
    while ( portIsOpened(device->port) ) {              // and then spin as long as the port is open
        device->step();                                 // process incoming data
        SLEEP_MS(10);                                   // we can sleep for spell and keep our CPU happy
    }

    return 0;    
} //main_discovery



/**
 * The main entry point for the application - note that this calls the discovery code
 * @param argc
 * @param argv
 * @return main_discovery
 */
int main(int argc, const char** argv) {

    const char* portPattern = "(.+)";   // NOTE: this is a MATCHING REGEX pattern (this one matches everything)

    if (argc > 2)
    {
        std::cout << "Usage: No argument allows automatic port discovery based on universal pattern.  Or if desired, a single argument selects the port (i.e. /dev/ttyACM0)" << std::endl;
        return -1;
    }

    std::cout << "CustomRobotDevice example application started (ctrl+C or ctrl+\\ to quit)";
    if (argc == 2) {
        portPattern = argv[1];
        std::cout << ", attempting to use port " << portPattern;
    }
    else
        std::cout << ", attempting port discovery";
    std::cout << std::endl;

    return main_discovery(portPattern);
} //main
