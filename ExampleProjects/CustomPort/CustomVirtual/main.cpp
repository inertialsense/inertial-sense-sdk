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

/** The port factory child class the user creates, inheriting from PortFactory.h definition */
#include "CustomVirtualPortFactory.h"

/** Include the SDK PortManager if desired for building and maintaining a list of all ports by name */
#include "PortManager.h"


/**
 * Uses arg for identifying virtual serial port in a "minimal" example of setting up a custom serial port
 * connection.  Use PortManager and PortFactory to bind a port_handle_t to the named port. With the handle,
 * the port is opened, which is in loopback mode in this case, and a simple write/read test is performed.
 */
int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printf("Please pass the virtual loopback port as the only argument (i.e. TEST0 or TEST1)\r\n");
        // In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3" 
        return -1;
    }

    printf("Attempting to bind and open virtual port %s\r\n", argv[1]);

    
    /** STEP 7: We need a singleton Port Manager and CustomVirtualPortFactory,
     * we'll make local references, and we register the virtual port factory
     */
    PortManager& pm = PortManager::getInstance();
    CustomVirtualPortFactory& vpf =  CustomVirtualPortFactory::getInstance();
    pm.addPortFactory(&vpf);
          
    /** We are interested in finding all ports matching a certain name pattern, but then we'll reference the one
     specifically indicated on the command line, which is virtual loopback in this case
    */
    pm.discoverPorts(R"(TEST\d\0?)", PORT_TYPE__COMM | PORT_TYPE__LOOPBACK);
    port_handle_t port = pm.getPort(argv[1], PORT_TYPE__COMM | PORT_TYPE__LOOPBACK);    
    
    if (port == nullptr) {
        printf("Failed to allocate port\r\n");
        return -2;
    }

    /** Binding a port does not open a port.. so let's open it using base_port functions */
    if (!portIsOpened(port) && (portOpen(port) != PORT_ERROR__NONE)) {
        printf("Failed to open port\r\n");
        return -3;
    }

    /** STEP 8: In a loop, send to and receive messages from the loopback port.   
     */
    const unsigned char wbuf[] = "IMPORTANT MESSAGE";
    unsigned int wlen = strlen(reinterpret_cast<const char*>(wbuf));
    unsigned char rbuf[PORT_BUFFER_SIZE];

    int rbytes, wbytes;    // note that base_port read/write returns signed integers
    int run_cnt = 2;       // arbitrary run count, iterations through the loop

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

        log_msg(IS_LOG_PORT, IS_LOG_LEVEL_INFO, "Attempting to send msg '%s'", wbuf);
                        
        if (portFree(port) >= wlen) {          // write fixed message if space available
            wbytes = portWrite(port, wbuf, wlen);
        }
        SLEEP_MS(1000);

        /** This is a loopback port test, read back same message
         * (note does not attempt to revisit and completely empty buffer if written by another source)
         */
        if (portAvailable(port) > 0) {
            rbytes = portRead(port, rbuf, PORT_BUFFER_SIZE);
        }
        
        bool test_success = false;          // verification and logging of results

        /** STEP 9: Use the SDK's msg logger utility to add valuable user messages to a log output file (inertial_sense.log)
         */        
        if ( (wbytes > 0) && (rbytes == wbytes) ) {
            if ( memcmp(rbuf, wbuf, wlen) == 0 ) {
                log_msg(IS_LOG_PORT, IS_LOG_LEVEL_INFO, "Loopback test good on comm port '%s', %d bytes sent/recvd", portName(port), rbytes);
                test_success = true;
            }
        }

        if ( !test_success ) {
            log_msg(IS_LOG_PORT, IS_LOG_LEVEL_INFO, "Loopback test FAIL on comm port '%s', wrote %d, read %d", portName(port), wbytes, rbytes);
        }

        memset(rbuf, 0, PORT_BUFFER_SIZE);  // clear the recv buffer
        
    } //while


    printf("Program complete, see inertial_sense.log for results\r\n");
    
    
} //main
