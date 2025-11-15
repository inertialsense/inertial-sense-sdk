/**
 * @file TcpServerPortFactory.h
 * @brief This is a TcpPortFactory which binds a socket listener to a specified server port, and creates new tcpPorts for each
 *  incoming connection request on that port.
 *
 * @author Kyle Mallory on 10/17/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__TCP_SERVER_PORT_FACTORY_H
#define IS_SDK__TCP_SERVER_PORT_FACTORY_H

#include <csignal>
#include <set>
#ifdef _WIN32
#include <winsock2.h>
#endif

#include "core/msg_logger.h"
#include "PortFactory.h"
#include "core/tcpPort.h"

/**
 * Singleton class passed to PortManager to allow incoming TCP connections on a listening tcp port, to create new tcpPorts
 *
 * @code{.cpp} portManager.addPortFactory((PortFactory*)&(TcpPortFactory::getInstance())); @endcode
 * Call to a PortManager adding a ISManufacturingPortFactory as an available PortFactory
 */

class TcpServerPortFactory : public PortFactory {
public:
    struct {
        uint16_t listenerPort = 4321;       //!< the tcp port to listen for incoming connections on
        struct sockaddr_in listeningAddr;   //!< listening address (this binds to a specific interface, defaults to 127.0.0.1
        bool backgroundListener = false;    //!< if true, we'll setup a thread to process incoming connections -- note that this doesn't service those connections, just the listener
        int maxConnections = 10;
        bool portDefaultBlocking = false;   //!< if true, created tcpPorts will be configured for blocking by default (usually, we don't want that).
        //!< the maximum number of connections that can be kept open
    } factoryOptions = {};

    static TcpServerPortFactory& getInstance() {
        static TcpServerPortFactory instance;
        return instance;
    }

    TcpServerPortFactory(TcpServerPortFactory const &) = delete;
    TcpServerPortFactory& operator=(TcpServerPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    bool releasePort(port_handle_t port) override;

private:
    TcpServerPortFactory() {
#ifdef PLATFORM_IS_LINUX
        signal(SIGPIPE, SIG_IGN); // ignore broken pipes
#endif
#ifdef _WIN32
        WSADATA wsa_data;
        WSAStartup(MAKEWORD(2, 2), &wsa_data);
#endif
    };
    ~TcpServerPortFactory() {
#ifdef _WIN32
        WSACleanup();
#endif
    }

    struct socket_entry_t {
        int socket = 0;
        std::string portName = "";
        mutable port_handle_t port = nullptr;

        socket_entry_t(int _s, std::string _n, port_handle_t _p = nullptr) : socket(_s), portName(_n), port(_p) {};

        // Overload operator< for strict weak ordering
        bool operator<(const socket_entry_t& other) const {
            if (socket != other.socket) {
                return socket < other.socket;
            }
            return portName < other.portName; // Secondary sorting criterion
        }

    };

    std::set<socket_entry_t> knownSockets;
    std::set<socket_entry_t> prevSockets;

    int listen_fd = 0; /* listener socket */

    bool startListening();

    void stopListening();

    /**
     * The primary service routine - this should be called periodically (and frequently) to service incoming connections.
     * If this is not called, no ports will ever be discovered/created
     */
    bool processPendingConnections(std::function<void(socket_entry_t)> cb);
};


#endif //IS_SDK__TCP_SERVER_PORT_FACTORY_H
