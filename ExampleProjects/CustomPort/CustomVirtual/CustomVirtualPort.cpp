/**
 * @file CustomVirtualPort.cpp
 * @brief From a collection of functions and classes that might be useful when writing/running unit tests,
 * ported to use as customer example of building a custom port implementation to extend base_port
 *
 * @author TylerS
 * @remark Originated as Walt Johnson's tests/test_serial_utils.cpp
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

/** Include C++ library and other needed SDK header files here
 */
#include "CustomVirtualPort.h"
#include "util/util.h"


#define TIME_USEC()             current_timeUs()
#define TIME_DELAY_USEC(us)     SLEEP_US(us)

/** STEP 2: For this example, we use a fixed stack allocation of our ports
 */
custom_port_t g_customPorts[NUM_COM_PORTS] = {};


/** This indexes pointers to the custom ports, AKA test ports
 */
static custom_port_t* boundPorts[NUM_COM_PORTS] {
        #if (NUM_COM_PORTS > 0)
            TEST0_PORT, // loopback
        #endif
        #if (NUM_COM_PORTS > 1)
            TEST1_PORT, // loopback
        #endif
        #if (NUM_COM_PORTS > 3)
            TEST3_PORT, TEST2_PORT, // PORT2 <-> PORT3
        #endif
        #if (NUM_COM_PORTS > 5)
            TEST5_PORT, TEST4_PORT, // PORT4 <-> PORT5
        #endif
};

/** STEP 3: Implementations of our core functions for this custom virtual port
 */
static int customPortRead(port_handle_t port, unsigned char* buf, unsigned int len)
{
    return ringBufRead(&((custom_port_t*)port)->portRingBuf, buf, len);
}

static int customPortWrite(port_handle_t port, const unsigned char* buf, unsigned int len)
{
    custom_port_t* destPort = boundPorts[portId(port)];

    if (ringBufWrite(&destPort->portRingBuf, (unsigned char*)buf, len))
    {   
        // Buffer overflow
        throw new std::out_of_range(utils::string_format("customPortWrite ring buffer overflow on %s: %d !!!\n",
                                                         portName(destPort), ringBufUsed(&destPort->portRingBuf) + len));

        return PORT_ERROR__WRITE_FAILURE;
    }
    return len;
}

static int customPortFree(port_handle_t port) {
    return ringBufFree(&((custom_port_t*)port)->portRingBuf);
}

static int customPortAvailable(port_handle_t port) {
    return ringBufUsed(&((custom_port_t*)port)->portRingBuf);
}

static const char* customPortName(port_handle_t port) {
    return (const char*)((custom_port_t*)port)->name;
}

/** We can add port-specific validations here outside the Port Factory, if any;
 * returns 1 for true on success with validation
*/
static int customPortValidate(port_handle_t port) {

    if ( port )
        return 1;
    else
        return 0;
}

/** Implementations of our support functions for this custom virtual port go here; for now we have an
 * initializer that configures all the virtual ports in one loop
 */
void initCustomPorts() {
    int portNum = 0;
    for (custom_port_t& port : g_customPorts) {
        port.base.pnum = portNum;
        port.base.ptype = PORT_TYPE__COMM;
        if (portNum <= 1)
            port.base.ptype |= PORT_TYPE__LOOPBACK;  // only PORT0 and PORT1 are Loopbacks

        port.base.portRead = customPortRead;
        port.base.portWrite = customPortWrite;
        port.base.portFree = customPortFree;
        port.base.portAvailable = customPortAvailable;
        port.base.portName = customPortName;
        port.base.portValidate = customPortValidate;
        portFlagsSet(&port, PORT_FLAG__VALID);
        portFlagsSet(&port, PORT_FLAG__OPENED);

        ringBufInit(&port.portRingBuf, port.portBuffer, PORT_BUFFER_SIZE, 1);
        SNPRINTF((char *)port.name, PORT_NAME_SIZE, "TEST%1d", portNum);

        portNum++;
    }
} //initCustomPorts



