/**
 * @file CustomVirtualExample.cpp
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the SDK ExampleProjects/ISComm/ISCommExample.cpp
 *
 * @details This demonstrates an example application creating a custom port factory 
 * with a virtual port implemented by the test_serial_utils code
 *
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */


/** STEP 4: Include user IO capabilities for this demo so when run visual indicators appear */
#include <stdio.h>

/** Include utility functions for use by your custom port class member functions defined here
 */
#include "ISUtilities.h"

/** The port factory child class the user creates, inheriting from PortFactory.h definition */
#include "CustomVirtualPortFactory.h"

/** Function declarations for this file */
void portHandler(PortFactory* factory, uint16_t pType, const std::string& pName);

/**
 * Uses arg for com port to connect to IS physical (or virtual?) device in a 
 * "minimal" example of setting up a custom serial port connection.  Bind a 
 * port_handle_t to the named port. With the handle, the port is opened, and an
 * TODO involve ISDevice here?
 * ISDevice is created using that handle.  With the resulting device we validate its connectivity and configure it to stream
 * DID_SYS_PARAMS messages every 5 seconds.  Data is output to the console, and the application loops until the port is closed.
 */
int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printf("Please pass the virtual loopback port as the only argument (i.e. TEST0 or TEST1)\r\n");
        // In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3" 
        return -1;
    }

    printf("Attempting to allocate and open virtual port %s\r\n", argv[1]);
        
    /** STEP 5: Initialize and open comms port, which is virtual loopback in this case */
    CustomVirtualPortFactory& vpf =  CustomVirtualPortFactory::getInstance();
    port_handle_t port = vpf.bindPort(argv[1], PORT_TYPE__COMM | PORT_TYPE__LOOPBACK);
       
    if (port == nullptr) {
        printf("Failed to allocate port\r\n");
        return -2;
    }

    // Binding a port does not open a port.. so let's open it using base_port functions
    if (!portIsOpened(port) && (portOpen(port) != PORT_ERROR__NONE)) {
        printf("Failed to open port\r\n");
        return -3;
    }

    /** STEP 6: In a loop, send to and receive messages from the loopback port.   
     */
    const unsigned char wbuf[] = "IMPORTANT MESSAGE";
    unsigned int wlen = strlen(reinterpret_cast<const char*>(wbuf));
    unsigned char rbuf[PORT_BUFFER_SIZE];

    // Note that base_port read/write returns signed integers
    int rbytes, wbytes;
    // arbitrary run count, iterations through the loop
    int run_cnt = 2;  

    /** 
     * We use the base_port API open/free/write etc functions, for which the user creates the 
     * underlying implementation.
     *
     * For this simple example, we pass an arbitrary string into a loopback virtual test port
     * underlying implementation for base_port from Inertial Sense, and compare the read back. 
     */
    while ( portIsOpened(port) && (run_cnt > 0) ) {
        run_cnt--;
        rbytes = 0;
        wbytes = 0;

        // write fixed message if space available
        if (portFree(port) >= wlen) {
            wbytes = portWrite(port, wbuf, wlen);
        }
        SLEEP_MS(1000);

        // this is a loopback port test, read back same message
        // (note does not attempt to revisit and completely empty buffer if written by another source)
        if (portAvailable(port) > 0) {
            rbytes = portRead(port, rbuf, PORT_BUFFER_SIZE);
        }

        if ( (wbytes > 0) && (rbytes == wbytes) ) {
            if ( memcmp(rbuf, wbuf, wlen) == 0 ) {
                log_msg(IS_LOG_PORT, IS_LOG_LEVEL_INFO, "Loopback test good on comm port '%s'", portName(port));
            }
            else {
                log_msg(IS_LOG_PORT, IS_LOG_LEVEL_INFO, "Loopback test FAIL on comm port '%s'", portName(port));
            }
        }
    } //while

    /** STEP 7:  Demonstrate the locatePorts function, which is given a callback to help target a different port among
     * those available, using a name search pattern in regex
     */
    auto cb = std::bind(portHandler, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    vpf.locatePorts(cb, R"(TEST\d\0?)", PORT_TYPE__COMM | PORT_TYPE__LOOPBACK );


    /** Demonstrate the releasePorts function
     */
    if ( vpf.releasePort(port) )
        printf("Program complete, see inertial_sense.log for results\r\n");
    else
        printf("Program complete (w/errors), see inertial_sense.log for results\r\n");
    
} //main



/**
 * @brief User function to do something like manage a change of ports when a port has been located by the
 * Port Factory locatePorts, provided as a callback
 * @param pType the type of said port (loopback, USB, SPI, etc)
 * @param pName name of the port the application is trying to locate and reference
 */
void portHandler(PortFactory* factory, uint16_t pType, const std::string& pName)
{
    /** STEP 8:  Demonstrate logging */
    log_msg(IS_LOG_PORT, IS_LOG_LEVEL_INFO, "portHandler call success");
    
} //portHandler
