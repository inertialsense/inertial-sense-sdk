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


    /** STEP 7: Set the SDK message logger verbosity level, as we use the logger for
     * some custom status and error messages
     */
    IS_SET_LOG_LEVEL(IS_LOG_LEVEL_INFO);

    
    /** STEP 8: We need a singleton Port Manager and CustomVirtualPortFactory,
     * we'll make local references, and we register the virtual port factory
     */
    PortManager& pm = PortManager::getInstance();
    CustomVirtualPortFactory& vpf =  CustomVirtualPortFactory::getInstance();
    pm.addPortFactory(&vpf);
          
    /** Let the PM find all available ports, then ask for the one specifically indicated on the command line by name */
    pm.discoverPorts();
    port_handle_t port = pm.getPort(argv[1]);


    /** STEP 9:  Now that we have a port handle, begin use of base_port interface, check on validity
     */
    if ( !portIsValid(port) ) {
        printf("Failed to allocate port\r\n");
        return -2;
    }

    /** Open port using base_port functions */
    if (!portIsOpened(port) && (portOpen(port) != PORT_ERROR__NONE)) {
        printf("Failed to open port\r\n");
        return -3;
    }

    
    /** STEP 10: In a loop, send to and receive messages from the loopback port.   
     */
    const unsigned char wbuf[] = "IMPORTANT MESSAGE";
    unsigned int wlen = strlen(reinterpret_cast<const char*>(wbuf));
    unsigned char rbuf[PORT_BUFFER_SIZE];

    int rbytes, wbytes;    // note that base_port read/write returns signed integers
    int run_cnt = 1;       // arbitrary run count, iterations through the loop

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

        printf("Attempting to send msg '%s'\r\n", reinterpret_cast<const char*>(wbuf));
                        
        if (portFree(port) >= wlen) {          // write fixed message if space available
            wbytes = portWrite(port, wbuf, wlen);
            
            // write error check
            if ( wbytes < 0 ) {
                printf("Exiting with write error\r\n");
                if ( wbytes == PORT_ERROR__WRITE_FAILURE )
                    printf("PORT_ERROR__WRITE_FAILURE\r\n");
                return -4;
            }
        }
        SLEEP_MS(1000);

        /** This is a loopback port test, read back same message
         * (note does not attempt to revisit and completely empty buffer if written by another source)
         */
        if (portAvailable(port) > 0) {
            rbytes = portRead(port, rbuf, PORT_BUFFER_SIZE);
        }
        
        bool test_success = false;          // verification and logging of results

        if ( (wbytes > 0) && (rbytes == wbytes) ) {
            if ( memcmp(rbuf, wbuf, wlen) == 0 ) {
                printf("Loopback test good on comm port '%s', %d bytes sent/recvd\r\n", portName(port), rbytes);
                test_success = true;
            }
        }

        if ( !test_success ) {
            printf("Loopback test FAIL on comm port '%s', wrote %d, read %d\r\n", portName(port), wbytes, rbytes);
        }

        memset(rbuf, 0, PORT_BUFFER_SIZE);  // clear the recv buffer
        
    } //while


    printf("Program complete, see inertial_sense.log for more results details\r\n");
    
    
} //main
