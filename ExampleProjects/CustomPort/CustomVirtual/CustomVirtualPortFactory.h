/**
 * @file CustomVirtualPortFactory.h 
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the SDK src/PortFactory.h
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef CUSTOM_SERIAL_PORT_FACTORY_H
#define CUSTOM_SERIAL_PORT_FACTORY_H

#ifdef __cplusplus

/** STEP 4: Include IS core and other needed SDK header files here; 
 * include any of your own custom application port definition headers, the lower-level
 * code that defines the interface used by this custom port factory; in this case the new
 * virtual test ports
 */
#include "core/base_port.h"
#include "core/msg_logger.h"
#include "PortFactory.h"
#include "CustomVirtualPort.h"


/** STEP 5: Create a custom child class that inherits from PortFactory 
 */
class CustomVirtualPortFactory : public PortFactory {
public:
    /** We need a way to get an instance of the singleton port factory */
    static CustomVirtualPortFactory& getInstance() {
        static CustomVirtualPortFactory instance;
        return instance;
    }

    /** Remove certain constructors/operators, because we're a singleton... */
    CustomVirtualPortFactory(CustomVirtualPortFactory const &) = delete;
    CustomVirtualPortFactory& operator=(CustomVirtualPortFactory const&) = delete;

    /**
     * These four functions will be required for your implementation, can be 
     * defined in your .cpp file for this class 
     */
    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;
    bool releasePort(port_handle_t port) override;


private:
    /** Make these private for our singleton pattern */
    CustomVirtualPortFactory() = default;
    ~CustomVirtualPortFactory() = default;

    /** These are the possible names allowed for identifying the base ports of this port factory
     * and users must implement a function to populate according to system setup
     */
    std::vector<std::string> portNames = {};

    /**
     * An internal static function which identifies all available serial ports on the host device. It populates a referenced
     * std::vector<std::string> with their names, as suitable identifiers. This does NOT do any port_handle allocation, validation,
     * or other operations necessary to USE the port - it merely identifies them.
     * @param portNames a reference to a vector of strings which will be cleared, and populated with virtual ports known to
     *  the application.
     * @return the number of port names populated into the vector.
     */
    static int getComPorts(std::vector<std::string>& portNames);   

    /** Demonstrates optional methods for the base_port hooks here, as opposed to within the base_port channel implementation;
     * for this port we have a validation_ but not an open_
     */
    static int validate_port(port_handle_t port) { return CustomVirtualPortFactory::getInstance().validatePort(portName(port), portType(port)); }
    //for example, static int open_port(port_handle_t port) { return serialPortOpen(port, SERIAL_PORT(port)->portName, SERIAL_PORT(port)->baudRate, SERIAL_PORT(port)->blocking); }

};


#endif //  __cplusplus

#endif // MY_CUSTOM_SERIAL_PORT_FACTORY_H
