/**
 * @file TcpPortFactory.h
 * @brief This is a port factory used to a single known devices over TCP/IP
 *
 * @author FiriusFoxx on 2025-06-12.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#ifndef IS_SDK__TCP_PORT_FACTORY_H
#define IS_SDK__TCP_PORT_FACTORY_H

#include <csignal>
#include "PortFactory.h"
#include "core/tcpPort.h"

/**
 * Singleton class passed to PortManager to allow a user to connect to a remote serial port over the network using a URL
 *
 * @code{.cpp} portManager.addPortFactory((PortFactory*)&(TcpPortFactory::getInstance())); @endcode
 * Call to a PortManager adding a ISManufacturingPortFactory as an available PortFactory
 */
class TcpPortFactory : public PortFactory {
public:
    struct {
        bool defaultBlocking = false;
    } portOptions = {};

    static TcpPortFactory& getInstance() {
        static TcpPortFactory instance;
        return instance;
    }

    TcpPortFactory(TcpPortFactory const &) = delete;
    TcpPortFactory& operator=(TcpPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    bool releasePort(port_handle_t port) override;

private:
    TcpPortFactory() {
#ifdef PLATFORM_IS_LINUX
        signal(SIGPIPE, SIG_IGN); // ignore broken pipes
#endif
    };
    ~TcpPortFactory() = default;
};

#endif //IS_SDK__TCP_PORT_FACTORY_H
