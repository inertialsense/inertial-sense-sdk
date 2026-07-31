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
#include <regex>

/** Include the header file for your child port factory class derived from parent PortFactory.h
 */
#include "CustomVirtualPortFactory.h"

/** Include utility functions for use by your custom port class member functions defined here
 */
#include "ISUtilities.h"
#include "core/msg_logger.h"


/**
 * @brief  Required minimum method, validates name and type, locates and/or instantiates new port
 */
port_handle_t CustomVirtualPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    /** Calls the port factory validation, prior to binding, to make sure we can bind */
    if (!validatePort(pName, pType))
        return nullptr;
   
    /** In this example we use a virtual port, so there is no baud rate or blocking to set; our port is defined by
     *  custom_virtual_port; we allocate a new custom_port_t loopback port
     */
    custom_port_t* customPort = new custom_port_t();

    if (customPort) {
        /** Set up our new port */
        initCustomPort(*customPort, pName, pType);
    }
    else
        return nullptr;
        

    /** Need a port_handle_t reference to our port to use for our own validation and to return from bind */
    port_handle_t port = (port_handle_t) customPort;

    /** Open and validate and configure the port as needed for the port implementation here; in this virtual port
     * example, do not need to open or configure, but we use the port's own validation method
     */
    portValidate(port);
       
    log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_INFO, "Bind new comm port '%s'", portName(port));

    return port;
}


/**
 * @brief Required minimum method, clears memory associated with port created in bindPort
 * 
 */
bool CustomVirtualPortFactory::releasePort(port_handle_t port) {
    if (!port)
        return false;

    log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_INFO, "Release comm port '%s'", portName(port) );
        
    /** If you allocated your port object on the heap, free the memory (delete) here;
     * or clear only for static allocation
     */
    memset(port, 0, sizeof(custom_port_t));
    delete static_cast<custom_port_t*>(port);
       
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
        log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_ERROR, "Port type validation failed: %d", pType );
        return false;   // we can only validate this port type - all others fail
    }
   
    /** Check port name to make sure it could be found, matching the naming prescribed by custom port definition */
    const std::regex pattern(validatePattern);
    std::smatch m;
    if (! std::regex_match(pName, m, pattern) ) {
        log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_ERROR, "Port name validation failed: '%s'", pName.c_str() );
        return false;
    }

    int index = std::stoi(m[1].str());
    if (index < 0 || index >= portNames.size()) {
        log_msg(IS_LOG_PORT_FACTORY, IS_LOG_LEVEL_ERROR, "Port index out of range: '%s'", pName.c_str() );
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

    /** For each port that is found by name matching search pattern, validate it and call PortManager callback function
     */
    for (auto& name : portNames) {
        auto match = std::regex_match(name, matchPattern);
        
        if (validatePort(name, pType ) && match) {
            portCallback(this, pType, name);
        }
    }
} //locatePorts

