/**
 * @file ISArduinoSerialPortFactory.cpp 
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the SDK src/PortFactory.cpp
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "PortFactory.h"

#include <string>
#include <vector>
#include <regex>

// #define REMOTE_SOCAT_PORTS      // only does anything on linux

#ifdef REMOTE_SOCAT_PORTS
    #include <filesystem>
#endif

#include "util/util.h"
#include "ISUtilities.h"

#include "PortManager.h"
#include "serialPort.h"
#include "serialPortPlatform.h"


/**
 * @brief Required minimum method, checks port name for viability
 * 
 */
bool ISArduinoSerialPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    if (pType != PORT_TYPE__UART | PORT_TYPE__HDW)
        return false;   // we can only validate this port type - all others fail

    if (!viableNames.contains(pName))
        return false;   // if a name is passed which doesn't exist in our "viableNames" map, it fails

    return true;
}

/**
 * @brief Required minimum method, validates name and type, locates and instantiates new port
 * 
 */
port_handle_t ISArduinoSerialPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    if (!validatePort(pName, pType))    // we should always validate the port, before attempting to bind it
        return nullptr;

    auto basePort = viableNames[pName];   // because we validated above, we know that this name is valid
    return (port_handle_t) new ISArduinoSerialPort(basePort, defaultBaudRate, defaultConfig);
}

/**
 * @brief Required minimum method, clears memory associated with port created in bindPort
 * 
 */
bool ISArduinoSerialPortFactory::releasePort(port_handle_t port) {
    if (!port)
        return false;

    delete (ISArduinoSerialPort*)port;

    return true;
}

/**
 * @brief Required minimum method, looks for matching name to port and calls portCallback
 * 
 */
void ISArduinoSerialPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)>
  portCallback, const std::string& pattern, uint16_t pType) {

    std::regex matchPattern(pattern);
    for (auto& [name, base] : viableNames) {
        auto match = std::regex_match(name, matchPattern);
        if (match && validatePort(name, PORT_TYPE__UART | PORT_TYPE__HDW))
            portCallback(this, PORT_TYPE__UART | PORT_TYPE__HDW, name);  // NOTE: don't use 'base', locatePorts only returns names
    }
}


#endif // #if PLATFORM_IS_LINUX
