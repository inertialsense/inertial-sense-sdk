/**
 * @file PortLocator.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/20/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__PORT_FACTORY_H
#define IS_SDK__PORT_FACTORY_H

#ifdef __cplusplus

#include <unordered_set>
#include <functional>
#include <string>
#include <cctype>

#include "core/base_port.h"
#include "core/msg_logger.h"
#include "ISConstants.h"

#include "serialPort.h"

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
    virtual void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern = "", uint16_t pType = PORT_TYPE__UNKNOWN) = 0;

    /**
     * Checks to determine if "the essence" of a port is valid. This should probably not perform any operation on the port
     * that could impact the ability of the port to operate. Rather, perform any reasonable checks to confirm if the port
     * actually exists and can be operated on (ie, does the device exist in the OS, or does the target host respond to a ping?).
     * Note that this is a factory-specific function typically called by the PortManager in order to determine if a port is
     * no longer viable as a precursory check
     * @param pName the string identifier of the port - this must be unique and is required
     * @param pType the type of port to validate - this is optional, but maybe modified by the underlying implementation
     * @return true if the port is viable/valid, otherwise false
     */
    virtual bool validatePort(const std::string& pName, uint16_t pType = 0) = 0;

    /**
     * A function responsible for allocating the underlying port type and returning a port_handle_t to it
     * This function should NOT manipulate the underlying port, such as opening, etc.
     * @param pType the type of the port being allocated
     * @param pName the binding name of the port to be allocated.
     * @return a port_handle_t to the allocated port
     */
    virtual port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) = 0;

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
    struct {
        int defaultBaudRate = BAUDRATE_921600;
        bool defaultBlocking = false;
    } portOptions = {};

    static SerialPortFactory& getInstance() {
        static SerialPortFactory instance;
        return instance;
    }

    SerialPortFactory(SerialPortFactory const &) = delete;
    SerialPortFactory& operator=(SerialPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    bool releasePort(port_handle_t port) override;

private:
    SerialPortFactory() = default;
    ~SerialPortFactory() = default;

    /**
     * An internal static function which identifies all available serial ports on the host device. It populates a referenced
     * std::vector<std::string> with their names, as suitable identifiers. This does NOT do any port_handle allocation, validation,
     * or other operations necessary to USE the port - it merely identifies them.
     * @param portNames a reference to a vector of strings which will be cleared, and populated with UART/Serial ports known to
     *  the host operating system.
     * @return the number of port names populated into the vector.
     */
    static int getComPorts(std::vector<std::string>& portNames);

    /**
     * A static function which is used to report errors that occur on a port created by this factory
     * @param port the port the error occurred on
     * @param errCode the error code (usually errno) of the error that occurred
     * @param errMsg an optional string message which describes the error the occurred
     * @return
     */
    static int onPortError(port_handle_t port, int errCode, const char *errMsg);

    std::vector<std::string> portNames = {};

#if PLATFORM_IS_LINUX
    static std::string get_driver__linux(const std::string& tty);
    static void register_comport__linux(std::vector<std::string>& comList, std::vector<std::string>& comList8250, const std::string& dir);
    static void probe_serial8250_comports__linux(std::vector<std::string>& comList, std::vector<std::string> comList8250);

    static bool validate_port__linux(uint16_t pType, const std::string& pName);
#elif PLATFORM_IS_WINDOWS
#endif

    // THESE ARE LOCALIZED HELPER FUNCTIONS to provide basic functionality that is not normally provided by the original SerialPort/SerialPortPlatform implementation
    // TODO: at some point, these should be moved into the implementation directly, and removed from the factory
    static int validate_port(port_handle_t port) { return SerialPortFactory::getInstance().validatePort(portName(port), portType(port)); }
    static int open_port(port_handle_t port) {
        if (!portIsValid(port)) return PORT_ERROR__INVALID;
        serial_port_t* serialPort = (serial_port_t*)port;
        return serialPortOpen(port, serialPort->portName, serialPort->baudRate, serialPort->blocking) == 1 ? PORT_ERROR__NONE : PORT_ERROR__OPEN_FAILURE;
    }

};


#endif

#endif // IS_SDK__PORT_FACTORY_H
