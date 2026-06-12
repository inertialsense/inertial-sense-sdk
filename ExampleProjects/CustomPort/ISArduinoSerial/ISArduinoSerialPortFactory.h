/**
 * @file ISArduinoSerialPortFactory.h 
 * @brief ${BRIEF_DESC}
 * 
 * @remark Based upon the Serial Port Refactor slides
 * @author TylerS
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef ARDUINO_SERIAL_PORT_FACTORY_H
#define ARDUINO_SERIAL_PORT_FACTORY_H

#ifdef __cplusplus

#include <unordered_set>
#include <functional>
#include <string>
#include <cctype>

#include "core/base_port.h"
#include "core/msg_logger.h"
#include "ISConstants.h"

#include "serialPort.h"


class ISArduinoSerialPortFactory : public PortFactory {
public:
    // default port config options for the factory
    struct {
        uint32_t defaultBaudRate = 921600;
        uint8_t defaultConfig = SERIAL_8N1;
    } portOptions = {};

    // we need a way to get an instance of the singleton
    static SerialPortFactory& getInstance() {
        static SerialPortFactory instance;
        return instance;
    }

    // remove these, because we're a singleton... 
    SerialPortFactory(SerialPortFactory const &) = delete;
    SerialPortFactory& operator=(SerialPortFactory const&) = delete;

    // override these from PortFactory
    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback,
                     const std::string& pattern, uint16_t pType) override;
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;
    bool releasePort(port_handle_t port) override;

private:

    // Make these private for our singleton pattern
    SerialPortFactory() = default;
    ~SerialPortFactory() = default;

    // initialize a map of the valid port names and the associate Serial reference that exists for the Arduino platform
    std::map<std::string, Serial_&> viableNames = {
      { "Serial", Serial}, 
#ifdef Serial1
      { "Serial1", Serial1},
#endif
#ifdef Serial2
      { "Serial2", Serial2},
#endif
#ifdef Serial3
      { "Serial3", Serial3},
#endif
#ifdef Serial4
      { "Serial4", Serial4},
#endif
    };
}

#endif // __cplusplus

#endif // ARDUINO_SERIAL_PORT_FACTORY_H
