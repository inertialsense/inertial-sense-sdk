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

/** STEP 4:  Create a custom child class that inherits from PortFactory 
 */

/**Include IS core and other needed SDK header files here; 
 * include any of your own custom application port definition headers, the lower-level
 * code that defines the interface used by this custom port factory; in this case the new
 * virtual test ports
 */
#include "core/base_port.h"
#include "PortFactory.h"
#include "custom_virtual_port.h"

class CustomVirtualPortFactory : public PortFactory {
public:
    /** We need a way to get an instance of the singleton port factory */
    static CustomVirtualPortFactory& getInstance() {
        static CustomVirtualPortFactory instance;
        return instance;
    }
    CustomVirtualPortFactory() { }
    
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
    /** Make this private for our singleton pattern */
    ~CustomVirtualPortFactory() = default;

};


#endif //  __cplusplus

#endif // MY_CUSTOM_SERIAL_PORT_FACTORY_H
