/**
 * @file custom_virtual_port.cpp
 * @brief From a collection of functions and classes that might be useful when writing/running unit tests,
 * ported to use as customer example of building a custom port implementation to extend base_port
 *
 * @author TylerS
 * @remark Originated as Walt Johnson's tests/test_serial_utils.cpp
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

/** Include C++ library and other needed SDK header files here
 */
#include "custom_virtual_port.h"


#define TIME_USEC()             current_timeUs()
#define TIME_DELAY_USEC(us)     SLEEP_US(us)


/** STEP 3: Implementations of our core functions for this custom virtual port
 */
static int customPortRead(port_handle_t port, unsigned char* buf, unsigned int len)
{
    return ringBufRead(&((custom_port_t*)port)->portRingBuf, buf, len);
}

static int customPortWrite(port_handle_t port, const unsigned char* buf, unsigned int len)
{
    custom_port_t* destPort = static_cast<custom_port_t*>(port);  //loopback

    if (ringBufWrite(&destPort->portRingBuf, (unsigned char*)buf, len))
    {   
        // Buffer overflow
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

/** Implementations of our support functions for this custom virtual port go here; we have an
 * initializer that configures a virtual port after allocation/bind
 */
void initCustomPort(custom_port_t& port, const std::string& pName, const uint16_t pType) {

    port.base.ptype = pType;

    port.base.portRead = customPortRead;
    port.base.portWrite = customPortWrite;
    port.base.portFree = customPortFree;
    port.base.portAvailable = customPortAvailable;
    port.base.portName = customPortName;
    port.base.portValidate = customPortValidate;
    portFlagsSet(&port, PORT_FLAG__VALID);
    portFlagsSet(&port, PORT_FLAG__OPENED);

    ringBufInit(&port.portRingBuf, port.portBuffer, PORT_BUFFER_SIZE, 1);
    SNPRINTF((char *)port.name, PORT_NAME_SIZE, "%s", pName.c_str());
      
} //initCustomPort



