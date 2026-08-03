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

#include "ISConstants.h"

#if PLATFORM_IS_WINDOWS
#include <winsock2.h>
#endif

#include "core/tcpPort.h"
#include "PortFactory.h"

/**
 * Singleton class passed to PortManager to allow a user to connect to a remote serial port over the network using a URL
 *
 * @code{.cpp} portManager.addPortFactory((PortFactory*)&(TcpPortFactory::getInstance())); @endcode
 * Call to a PortManager adding a ISManufacturingPortFactory as an available PortFactory
 */
class TcpPortFactory : public PortFactory {
public:
    /** Default options applied to ports created by this factory. */
    struct {
        bool defaultBlocking = false;   //!< default blocking mode applied to ports bound by this factory
    } portOptions = {};   //!< default options applied to ports created by this factory

    /** @return the process-wide singleton TcpPortFactory instance. */
    static TcpPortFactory& getInstance() {
        static TcpPortFactory instance;
        return instance;
    }

    TcpPortFactory(TcpPortFactory const &) = delete;
    TcpPortFactory& operator=(TcpPortFactory const&) = delete;

    /**
     * This factory doesn't provide device discovery; it only reports whether @p pattern itself
     * is a valid "tcp://host:port" URL, invoking @p portCallback with it if so (a one-port stub).
     * @param portCallback function invoked with (this factory, PORT_TYPE__TCP, pattern) if pattern validates
     * @param pattern the "tcp://host:port" URL to validate and "discover"
     * @param pType ignored
     */
    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    /**
     * Validates that @p pName is a well-formed "tcp://host:port" URL whose host resolves.
     * @param pName the URL to validate, starting with tcp://
     * @param pType must include PORT_TYPE__TCP
     * @return true if a port can be created from pName, otherwise false
     */
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    /**
     * Parses and creates a new port_handle_t representing a TCP port for a URL in the format
     * tcp://ipAddr:port.
     * @param pName the URL and name of the new port to bind a port_handle_t to
     * @param pType the port type requested to be generated
     * @return a port_handle_t bound to the newly created TCP port for the connection pName represents, or nullptr on failure
     */
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    /**
     * Releases and frees the memory used by this port.
     * @param port the TCP port handle to deinitialize
     * @return true if successful, false otherwise
     */
    bool releasePort(port_handle_t port) override;

private:
    TcpPortFactory() {
#ifdef PLATFORM_IS_LINUX
        signal(SIGPIPE, SIG_IGN); // ignore broken pipes
#endif
#ifdef _WIN32
        WSADATA wsa_data;
        WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif
    };
    ~TcpPortFactory() {
#ifdef _WIN32
        WSACleanup();
#endif
    }
};

#endif //IS_SDK__TCP_PORT_FACTORY_H
