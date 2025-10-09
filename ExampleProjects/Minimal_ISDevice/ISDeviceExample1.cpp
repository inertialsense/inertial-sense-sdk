/**
 * @file ISDeviceExample1.cpp 
 * @brief An application that demonstrates to mode basic C++ example of connecting to an Inertial Sense device and streaming data from it.
 *
 * @author Kyle Mallory on 6/3/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <iostream>

#include "PortFactory.h"
#include "ISDevice.h"
#include "ISDisplay.h"


/** this is a global instance of a utility class that handles printing/formatting of various data sets received from the device */
cInertialSenseDisplay isDisplay = cInertialSenseDisplay(cInertialSenseDisplay::DMODE_PRETTY);

/**
 * This is a callback handler that we will register with the ISDevice once its created, and which will be called every time data arrives from the device
 * @param ctx this is an opaque context pointer for this message - in this example, it will be the ISDevice* that received it the message - the ISDevice
 *   still need to process the data that it receives, so we dereference this, and call OnIsbDataHandler()
 * @param data a pointer to a p_data_t struct, which represents the buffer of data received from the device, including the data ID, associated flags,
 *   and the actual data payload
 * @param port the port_handle_t that this data was received from
 * @returns 0 if this message was successfully processed by a protocol-specific handler, and should not be further processed, otherwise return !0
 */
int isbDataHandler(void* ctx, p_data_t* data, port_handle_t port) {
    if (ctx) ((ISDevice*)ctx)->onIsbDataHandler(data, port);

    std::cout << isDisplay.DataToString((const p_data_t*)data);
    return 0;
}

/**
 * This is the "explained" example, it exposes a few more aspects of the SDK but is essentially functionally equivalent to the
 * "minimal" example below.  Both usages bind a port_handle_t to the named port. With the handle, the port is opened, and an
 * ISDevice is created using that handle.  With the resulting device we validate its connectivity and configure it to stream
 * DID_SYS_PARAMS messages every 5 seconds.  Data is output to the console, and the application loops until the port is closed.
 */
int main_explained(const char* portStr) {

    // First, we need to get a port_handle_t that the Device is connected to...

    // Assuming we know the name of the port we want to use, we will create a port_handle_t from its name.
    // PortFactory::bindPort() is used to allocate and initialize the underlying port_handle_t.
    // Note that here we are using SerialPortFactory because we KNOW we want a serial port
    port_handle_t port = SerialPortFactory::getInstance().bindPort(portStr);

    // For the sake of demonstration, let's check that the port is valid...
    //    portIsValid(...) returns true if the port is properly initialized (as opposed to a invalid/null pointer)
    if (!portIsValid(port)) {
        std::cerr << "Port is NOT valid!" << std::endl; // NOTE that if the port is invalid, we cannot safely call any other port*() functions using the handle.
        exit(1);
    }

    // and open it, if it's not already opened (probably not)
    //    portIsOpened(...) returns true if the port is already opened,
    //    portOpen(...) attempts to open the port, and returns PORT_ERROR__NONE if successful
    //      for future simplicity, portOpen() will both validate, and open, so the above portIsValid() and portIsOpened() aren't actually necessary here - we just wanted to show their usage.
    if (!portIsOpened(port) && (portOpen(port) != PORT_ERROR__NONE)) {
        std::cerr << "Unable to open the port " << portName(port) << std::endl;   // since we have a valid port, we can get the name of the port if we want, using portName(...).
        exit(2);
    }

    // With a valid, opened port, we can instance an ISDevice - in this case, an IMX-5.0 and associate the port to it.
    ISDevice* device = new ISDevice(IS_HARDWARE_IMX_5_0, port);

    if (!device->isConnected())
        exit(3); // this is another way we can confirm the connected status of the device

    // At this point, we have an unknown device associated with our port.
    // we can view (and confirm) that its 'unknown' by outputting the device description
    std::cout << "Allocated device " << device->getDescription() << std::endl;

    // However, we want to communicate with the device and do things with it...

    // ISDevice::validate(timeoutMs) will block until the device can be validated, devInfo fetched and flash configuration synchronized
    if (!device->validate(3000)) {
        std::cerr << "Timeout occurred while attempting to validate the device on port " << portName(port) << std::endl;
        exit(3);
    }

    // for demonstration purposes, let's print our device description now that we've validated
    std::cout << "Validated device " << device->getDescription() << std::endl;

    // Now that we have the device, and all its information has been validated, we can start to do real work with it...

    // Before we can get useful data from the device, we need to tell the SDK where to send the data it received from the device...
    // Let's use the function created at the stop of this source file
    device->registerIsbDataHandler(isbDataHandler);

    // Devices can be configured to stream data by default on powerup - lets stop all other messages before enabling ours
    device->StopBroadcasts(true);

    // Next, we need to indicate the specific data that we are interested in receiving, and how frequently we'd like to receive it.
    // Let's get the System Status (SYS_PARAMS) including uptime
    device->BroadcastBinaryData(DID_SYS_PARAMS, 5000);   // DID_SYS_PARAMS has a normal period of 1ms, so every 5000 * 1ms (5 seconds)

    // Finally, operate in a communications loop (this could be a thread, etc) and call ISDevice::step() periodically (ideally about every 1ms),
    // allowing the SDK to exchange and parse data with the connected device.
    while (portIsOpened(device->port)) {
        device->step();
        SLEEP_MS(1);
    }
    return 0;
}

/**
 * This is the "minimal" example, it demonstrates the most direct and basic interfaces to connect to a known device, and stream
 * data from it. This example binds a port_handle_t to the named port, and then creates an ISDevice instance bound to that port.
 * The device instances is connected, all default data streaming is disabled, and then subsequently configured to stream
 * DID_SYS_PARAMS messages every 5 seconds.  Data is output to the console, and the application loops until the port is closed.
 */
int main_minimal(const char* portStr) {
    // get a port handle for the specified serial port
    port_handle_t port = SerialPortFactory::getInstance().bindPort(portStr);

    // create a new IMX-5.0 device and bind the associated port
    ISDevice* device = new ISDevice(IS_HARDWARE_IMX_5_0, port);

    // connect to the device
    if (!device->connect()) {
        std::cerr << "Could not connect to device on port " << device->getPortName() << std::endl;
        exit(1);
    }

    p_data_hdr_t hdr = { .id = DID_SYS_PARAMS };        // just a little hack to make isDisplay.DataToStringSysParams() work nicely for the demo

    //device->validate();
    // device->StopBroadcasts(true);                       // stop all other messages
    // device->registerIsbDataHandler(isbDataHandler);     // register our data handler
    device->BroadcastBinaryData(DID_SYS_PARAMS, 5000);  // request the data of interest at the specified interval
    while (portIsOpened(device->port)) {                // and then spin as long as the port is open
        device->step();                                 // this processes all incoming data, and calls out handler, etc
        std::cout << isDisplay.DataToStringSysParams(device->sysParams, hdr);
        SLEEP_MS(5000);
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
    const char* portArg = "/dev/ttyACM0";
#else
    const char* portArg = "COMM1";
#endif
    if (argc > 1)
        portArg = argv[1];     // take the first argument as the port to connect with

#ifdef EXPLAINED
    return main_explained(portArg);
#else
    return main_minimal(portArg);
#endif
}