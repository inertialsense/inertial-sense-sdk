/**
 * @file CustomVirtualPortFactory.cpp
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the SDK src/PortFactory.cpp
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

/** STEP 5: Implement the required functions for a Port Factory
 */

/** Include C++ libraries for use by your custom port class member functions defined here
 */
#include <vector>
#include <regex>

/** Include the header file for your child port factory class derived from parent PortFactory.h
 */
#include "CustomVirtualPortFactory.h"

/** Include utility functions for use by your custom port class member functions defined here
 */
#include "ISUtilities.h"


/**
 * @brief  Required minimum method, validates name and type, locates and/or instantiates new port
 */
port_handle_t CustomVirtualPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    /** Calls the port factory validation, prior to binding, to make sure we can bind */
    if (!validatePort(pName, pType))
        return nullptr;
   
    /** In this example we use a virtual port, so there is no baud rate or blocking to set; our port is defined by
     *  CustomVirtualPort; defined g_customPorts given by TESTn_PORT is an array of custom_port_t, 0 and 1 are loopback ports
     *  and present the only ports utilized in this simple example
     */
    custom_port_t* customPort;

    if (pName == "TEST0") {
        customPort = TEST0_PORT;
    }
    else if (pName == "TEST1") {
        customPort = TEST1_PORT;
    }
    else
        return nullptr;

    /** Need a port_handle_t reference to our port to use for our own validation and to return from bind */
    port_handle_t port = (port_handle_t) customPort;

    /** Open and validate and configure the port as needed for the port implementation here; in this virtual port
     * example, do not need to open or configure, but we use the port's own validation method
     */
    portValidate(port);
       
    log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_INFO, "Bound new comm port '%s'", portName(port));
    return port;
}


/**
 * @brief Required minimum method, clears memory associated with port created in bindPort
 * 
 */
bool CustomVirtualPortFactory::releasePort(port_handle_t port) {
    if (!port)
        return false;

    log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_INFO, "Releasing comm port '%s'", portName(port) );

    /** If you allocated your port object on the heap, free the memory (delete) here;
     * In this example we use a virtual port with the static object, so there is no memory to free, only clear
     */
    memset(port, 0, sizeof(custom_port_t));      //or for example, delete (serial_port_t*)port;

    return true;
}

/**
 * @brief Required minimum method, checks port name/type for viability
 * 
 */
bool CustomVirtualPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    /** Check port type to make sure it is allowed, in this case both these types
     */
    if (pType != (PORT_TYPE__LOOPBACK | PORT_TYPE__COMM) ) {
        log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_INFO, "Port type validation failed: %d", pType );
        return false;   // we can only validate this port type - all others fail
    }

    /** Check port name to make sure it would be found, matching the naming described in the custom port definition */
    const std::regex pattern("^TEST[0-5]$");
    if (! std::regex_match(pName, pattern) ) {
        log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_INFO, "Port name validation failed: '%s'", pName.c_str() );
        return false;
    }
     
    return true;
}

/**
 * @brief Required minimum method, looks for matching name to port and calls portCallback
 * 
 */
void CustomVirtualPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    std::regex matchPattern(pattern);

    log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_INFO, "Locating ports with regex pattern '%s'", pattern.c_str());

    /** User implementation to populate names of ports as present on system
     */
    std::vector<std::string> portNames = {};
    portNames.clear();
    portNames.resize(NUM_COM_PORTS);
    
    /** Grab each string with the unique identifying names the underlying test port implementation dictates
     */
    int i = 0;      // Populate the vector using index into global custom port array
    for (auto& str : portNames) {
        str = std::string( reinterpret_cast<const char*>(g_customPorts[i].name));        
        log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_INFO, "Found port '%s'", str.c_str());
        ++i;
    }

    /** For each port that is found by name matching search pattern, validate it and call user-defined callback function
     */
    for (auto& name : portNames) {
        auto match = std::regex_match(name, matchPattern);
        if (validatePort(name, (PORT_TYPE__LOOPBACK | PORT_TYPE__COMM) ) && match) {
            portCallback(this, (PORT_TYPE__LOOPBACK | PORT_TYPE__COMM), name);
        }
    }
} //locatePorts

