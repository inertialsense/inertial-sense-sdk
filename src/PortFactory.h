/**
 * @file PortLocator.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/20/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef EVALTOOL_PORTLOCATOR_H
#define EVALTOOL_PORTLOCATOR_H

#include <unordered_set>
#include <functional>
#include <string>
#include <cctype>

#include "core/types.h"
#include "core/msg_logger.h"
#include "ISConstants.h"

/**
 * PortFactory is an abstract class that is responsible for discovery available ports of a particular type.
 * There should be one implementation for each type of discoverable port.  This does NOT return a port,
 * only a name or some other identifier that can be used by the port implementation to create the port.
 * As an abstract class, this allows for third-party locators to be implemented for custom port types.
 */
class PortFactory {
public:
    // virtual ~PortFactory() = default;

    /**
     * @param portCallback - A function to be called when this Factory identifies a possible port; callback parameters are port-type and name
     */
    virtual void locatePorts(std::function<void(PortFactory*, std::string)> portCallback, const std::string& pattern = "", uint16_t pType = PORT_TYPE__UNKNOWN) = 0;

    /**
     * A function responsible for allocating the underlying port type and returning a port_handle_t to it
     * This function should NOT manipulate the underlying port, such as opening, etc.
     * @param pType the type of the port being allocated
     * @param pName the binding name of the port to be allocated.
     * @return a port_handle_t to the allocated port
     */
    virtual port_handle_t bindPort(u_int16_t pType, std::string pName) = 0;

    /**
     * A function responsible for freeing the allocated memory of the underlying port.
     * This function should NOT manipulate the underlying port, such as flushing, closing it, etc.
     * @param port the handle of the port to release
     * @return true if the port specified was a valid port, and it was successfully released, otherwise false.
     */
    virtual bool releasePort(port_handle_t port) = 0;
};

class SerialPortFactory : public PortFactory {
public:
    static SerialPortFactory& getInstance() {
        static SerialPortFactory instance;
        return instance;
    }


private:
    SerialPortFactory() = default;
    ~SerialPortFactory() = default;

    SerialPortFactory(SerialPortFactory const &) = delete;
    SerialPortFactory& operator=(SerialPortFactory const&) = delete;


    std::vector<std::string> ports;

#if PLATFORM_IS_LINUX
    static std::string get_driver__linux(const std::string& tty);
    static void register_comport__linux(std::vector<std::string>& comList, std::vector<std::string>& comList8250, const std::string& dir);
    static void probe_serial8250_comports__linux(std::vector<std::string>& comList, std::vector<std::string> comList8250);
#elif PLATFORM_IS_WINDOWS
#endif

    int getComPorts(std::vector<std::string>& ports);

    port_handle_t bindPort(u_int16_t pType, std::string pName) override;

    bool releasePort(port_handle_t port) override;

    void locatePorts(std::function<void(PortFactory*, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    static int onPortError(port_handle_t port, int errCode, const char *errMsg);
};

/*
class DfuPortLocator : PortLocator {
    void locatePorts(std::function<void(u_int16_t, std::string)> portCallback) override;
};

class TcpPortLocator : PortLocator {
    void locatePorts(std::function<void(u_int16_t, std::string)> portCallback) override;
};

class UdpPortLocator : PortLocator {
    void locatePorts(std::function<void(u_int16_t, std::string)> portCallback) override;
};
*/


#endif //EVALTOOL_PORTLOCATOR_H
