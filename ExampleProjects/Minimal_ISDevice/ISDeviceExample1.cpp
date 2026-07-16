/**
 * @file ISDeviceExample1.cpp
 * @brief An application that demonstrates to mode basic C++ example of connecting to an Inertial Sense device and streaming data from it.
 *
 * @author Kyle Mallory on 6/3/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <iostream>
#include <signal.h>

#include "PortFactory.h"
#include "ISDevice.h"
#include "ISDisplay.h"
#include "ISLogger.h"

// Volatile flag to ensure safe modification inside the signal handler
volatile sig_atomic_t keep_running = 1;

// The signal handler function
void handle_sigint(int sig) {
    // Only use async-signal-safe functions inside handlers
    keep_running = 0;
}

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

    if ((data->hdr.id == DID_SYS_PARAMS) || (data->hdr.id == DID_GNSS1_POS) || (data->hdr.id == DID_INS_1))
        std::cout << isDisplay.DataToString((const p_data_t*)data);
    return 0;
}

/**
 * This is a callback handler that we will register with the ISDevice once its created, and which will be called any time there is an error parsing data.
 *   In order to identify the nature of the error (which is not passed here), we can inspect the ISComm instances associated with the port and extract
 *   the last reported parse error.
 * @param ctx this is an opaque context pointer for this message - in this example, it will be the ISDevice* that received it the message - the ISDevice
 *   still need to process the data that it receives, so we dereference this, and call OnIsbDataHandler()
 * @param msg a pointer to a unsigned char *, which represents the buffer of data received which caused the error. This will very and is not always
 *   useful, but is provided to allow inspection of the data to determine a possible cause for the error.  In some cases, this could be
 * @param msgSize the number of bytes in the msg payload
 * @param port the port_handle_t that this data was received from
 * @returns 0 if this message was successfully processed by a protocol-specific handler, and should not be further processed, otherwise return !0
 */
int errorHandler(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port) {
    if ((portType(port) & PORT_TYPE__COMM) != PORT_TYPE__COMM)
    {
        // if the port isn't a COMM port, there is a problem... this should never happen, but we'll guard against it anyway
        return 0;   // acknowledge the error, but we really can't do anything with it.
    }

    std::string rawMsgData = utils::raw_hexdump((const char*)msg, msgSize, 64);

    // We'll extract the ISComm instance associated with the port in order to gain access to the parser details
    comm_port_t* commPort = COMM_PORT(port);
    switch (commPort->comm.rxErrorType)
    {
    case EPARSE_INVALID_PREAMBLE:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm parsed an invalid preamble:  %s", rawMsgData.c_str());
        break;
    case EPARSE_INVALID_SIZE:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm parsed an invalid payload size:  %s", rawMsgData.c_str());
        break;
    case EPARSE_INVALID_CHKSUM:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm parsed an invalid checksum:  %s", rawMsgData.c_str());
        break;
    case EPARSE_INVALID_DATATYPE:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm parsed an invalid datatype:  %s", rawMsgData.c_str());
        break;
    case EPARSE_MISSING_EOS_MARKER:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm could not identify an valid EOS marker:  %s", rawMsgData.c_str());
        break;
    case EPARSE_INCOMPLETE_PACKET:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm parsed an incomplete packet/payload:  %s", rawMsgData.c_str());
        break;
    case EPARSE_INVALID_HEADER:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm parsed an invalid packet header:  %s", rawMsgData.c_str());
        break;
    case EPARSE_INVALID_PAYLOAD:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm parsed an invalid payload:  %s", rawMsgData.c_str());
        break;
    case EPARSE_RXBUFFER_FLUSHED:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm forced a flush of te RX buffer; data may be lossed: %s", rawMsgData.c_str());
        break;
    case EPARSE_STREAM_UNPARSABLE:
        log_warn(IS_LOG_FACILITY_NONE, "ISComm reported an error parsing the byte stream: %s", rawMsgData.c_str());
        break;
    }
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
    // NOTE: ISDevice derives from std::enable_shared_from_this, and several of its methods (connect/validate/step)
    // call shared_from_this() internally. It MUST therefore be owned by a std::shared_ptr - allocating it with a raw
    // 'new' would leave it without a control block and throw std::bad_weak_ptr at the first such call.
    std::shared_ptr<ISDevice> device = std::make_shared<ISDevice>(IS_HARDWARE_IMX_5_0, port);

    if (!device->isConnected())
        exit(3); // this is another way we can confirm the connected status of the device

    // In this example, we used port-related functions to open and connect to a device.
    // However, we could have reduced some lines by passing the port handle from the bindPort() directly to the ISDevice
    // constructor and then used ISDevice::connect() which would have validated and opened the port (if not already).
    //
    // ISDevice::connect() also performs some additional validation on the connected device by validating that speaks an
    // Inertial Sense protocol, and exchanges device hardware and firmware information as well as configuration data.
    //
    // These techniques are demonstrated in the main_minimal() example below.


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
    // Let's use the function created at the top of this source file - isbDataHandler()
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
    SerialPortFactory::getInstance().portOptions.defaultBaudRate = 115200;
    port_handle_t port = SerialPortFactory::getInstance().bindPort(portStr);

    // create a new IMX-5.0 device and bind the associated port
    // NOTE: ISDevice uses shared_from_this() internally, so it must be owned by a std::shared_ptr (make_shared),
    // not a raw 'new' - otherwise connect()/validate()/step() throw std::bad_weak_ptr.
    std::shared_ptr<ISDevice> device = std::make_shared<ISDevice>(IS_HARDWARE_IMX_5_0, port);

    // connect to the device; by default the connect will ALSO validate the device; but we'll skip that for this example
    if (!device->connect(false)) {                              // false == bypass validation
        std::cerr << "Could not connect to device on port " << device->getPortName() << std::endl;
        exit(1);
    }

    // once we've got a good connection, now let's make sure we're talking to an Inertial Sense device
    if (!device->validate())
    {
        std::cerr << "Could not validate an Inertial Sense device on port " << device->getPortName() << std::endl;
        exit(1);
    }

    // now that we've validated, we're free to do all our regular things...

    // first, let's setup a data logger so we can capture data to log for any troubleshooting
    cISLogger logger;
    cISLogger::sSaveOptions options;
    options.logType = cISLogger::LOGTYPE_RAW;
    logger.InitSave("test_log", options);
    auto devLog = logger.registerDevice(device);   // if you know this information, you can pass it, but it's not important that it match your actual hardware.
    logger.EnableLogging(true);

    device->StopBroadcasts(true);                                // stop all other messages
    SLEEP_MS(10);                                                       // wait for the device to process this message before we do anything more

    device->registerIsbDataHandler(isbDataHandler);                     // register our data handler
    device->registerProtocolHandler(_PTYPE_PARSE_ERROR, errorHandler);  // register a handler for parse errors
    device->BroadcastBinaryData(DID_SYS_PARAMS, 500);       // request the data of interest at the specified interval
    while (portIsOpened(device->port) && keep_running) {                // and then spin as long as the port is open, and our "keep_running" variable is true
        device->step();                                                 // this processes all incoming data, and calls out handler, etc
        SLEEP_MS(1);
    }

    device->disconnect();
    logger.CloseAllFiles();
    return 0;
}

int initSignalHandler()
{
    // first things first - let's setup a signal handler so you can use 'CTRL-C' to exit out of this example...
    struct sigaction act;

    // Set the handler function
    act.sa_handler = handle_sigint;

    // Clear the mask to block no additional signals during execution
    sigemptyset(&act.sa_mask);

    // No special flags needed
    act.sa_flags = 0;

    // Register the handler for SIGINT
    if (sigaction(SIGINT, &act, NULL) < 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }

    if (sigaction(SIGTERM, &act, NULL) < 0) {
        perror("sigaction");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
 * The main entry point for the application - note that this calls one of two examples, both do the same thing with slight differences.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, const char** argv) {
    IS_SET_LOG_LEVEL(IS_LOG_LEVEL_MORE_DEBUG);
    IS_LOG_OUTPUT(stdout);

    if (initSignalHandler() != EXIT_SUCCESS)
    {
        log_warn(IS_LOG_FACILITY_NONE, "Unable to register signal handler - we'll stop after 15 seconds.");
    }

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
    return main_minimal(portArg);   // look at main_explained() to see a more detailed explanation
#endif
}