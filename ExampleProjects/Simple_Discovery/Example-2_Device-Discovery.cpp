/**
 * @file ISDeviceExample1.cpp 
 * @brief An application that builds from the Minimal ISDevice example, and which demonstrates Port/Device discovery.
 *
 * @author Kyle Mallory on 6/3/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 *
 * In the previous example, we used the SerialPortFactory to create an instance of a serial port, and we constructed
 * an instance of an ISDevice attached to that port.  In this example we will use the PortManager and the DeviceManager
 * to automatically discover available ports and connected devices.
 *
 * The PortManager uses a "locatePorts()" function of each PortFactory to identify possible ports of each factory type.
 * The DeviceManager attempts to identify devices on each of the discovered ports, and create ISDevice instances for
 * each port which responds to the Inertial Sense protocol.  Together, these two mechanisms can provide a very simple
 * and effective means to establish communications with one or more devices, when the location of the device/port is
 * dynamic.
 *
 */

#include <iostream>

#include "PortManager.h"
#include "DeviceManager.h"
#include "ISDevice.h"
#include "ISDisplay.h"


/** this is a global instance of a utility class that handles printing/formatting of various data sets received from the device */
cInertialSenseDisplay isDisplay = cInertialSenseDisplay(cInertialSenseDisplay::DMODE_PRETTY);

/**
 * This is a callback handler that we will register with the ISDevice once its created, and which will be called every time data arrives from the device
 * @param ctx this is an opaque context pointer for this message - in this example, it will be the ISDevice* that received the message - the ISDevice
 *            still needs to process the data that it receives, so we dereference this, and call OnIsbDataHandler()
 * @param data a pointer to a p_data_t struct, which represents the buffer of data received from the device, including the data ID, associated flags,
 *            and the data payload
 * @param port the port_handle_t that this data was received from
 * @returns 0 if this message was successfully processed by a protocol-specific handler, and should not be further processed, otherwise return !0
 */
int isbDataHandler(void* ctx, p_data_t* data, port_handle_t port) {
    if (ctx) ((ISDevice*)ctx)->onIsbDataHandler(data, port);    // dereference Ctx and let ISDevice do additional handling

    if (data->hdr.id == DID_INS_1)
        std::cout << isDisplay.DataToString((const p_data_t*)data);

    return 0;
}


/**
 * This is basically the "minimal" example, but leverages Port/Device discovery to locate a dynamic or unknown device.
 * This will attempt to discover ports, retrying upto 3 times. Each time ports are discovered, it attempts to discover
 * devices on all known ports.  If an IMX 5.0 device is found, the discovery phase will end, and a connection attempt
 * is made. Note that we don't know to instance the ISDevice or the port, because the PortManager/DeviceManager has done
 * that for us.  We only need to open a connection to the device and start communicating.
 */
int main_discovery(const char* portPattern) {
    ISDevice* device = nullptr; // this will be our discovered device... but null for now.

    // Both DeviceManager & PortManager are singletons, but we'll make a local reference to both to keep the code clean
    PortManager& pm = PortManager::getInstance();
    pm.addPortFactory(&SerialPortFactory::getInstance());   // tell the PortManager that we are interested in Serial Ports  // TODO: this should probably be a default option

    DeviceManager& dm = DeviceManager::getInstance();
    dm.addDeviceFactory(&ImxDeviceFactory::getInstance());  // tell the DeviceManager that we are interested in IMX Devices // TODO: this should probably be a default option

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
                device = dm.getDevices().front();
                break;  // we're done, so we'll break out of our retry loop.

            }
        }
        SLEEP_MS(500);  // give a brief pause for new ports/devices to become available.
    }

    if ((retry < 0) && !device) {
        // we failed to locate any devices - report and exit gracefully
        std::cout << "No Inertial Sense devices were found after 3 attempts. Exiting." << std::endl;
        exit(0);    // this is NOT an error
    }

    // from here, the rest of the example is the same as the Minimal_Device::main_minimal() example.
    // connect to the device
    if (!device->connect()) {
        std::cerr << "Could not connect to device on port " << device->getPortName() << std::endl;
        exit(1);
    }

    // device->validate();                              // NOTE: device validation is already done during DeviceManager::discoverDevice() so no need to do it again

    // Before we can get useful data from the device, we need to tell the SDK where to send the data it received from the device...
    // Let's use the function created at the stop of this source file
    device->registerIsbDataHandler(isbDataHandler);

    // Devices can be configured to stream data by default on powerup - lets stop all other messages before enabling ours
    device->StopBroadcasts(true);

    // Let's stream DID_INS_1
    device->BroadcastBinaryData(DID_INS_1, 25);         // Stream at 1/25th the default DID_INS_1 rate (device dependent, but approx 1x = 7ms)

    while (portIsOpened(device->port)) {                // and then spin as long as the port is open
        device->step();                                 // process incoming data
        SLEEP_MS(10);                                   // we can sleep for spell and keep our CPU happy
    }
    return 0;
}

/**
 * The main entry point for the application - note that this calls one of two examples, both do the same thing with slight differences.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, const char** argv) {

#if PLATFORM_IS_LINUX
    const char* portPattern = "(.+)";   // NOTE: this is a MATCHING REGEX pattern (this one matches everything)
#else
    const char* portArg = "COMM1";
#endif
    if (argc > 1)
        portPattern = argv[1];     // take the first argument as the port to connect with

#ifdef EXPLAINED
    return main_explained(portArg);
#else
    return main_discovery(portPattern);
#endif
}