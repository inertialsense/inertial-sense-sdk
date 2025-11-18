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
    #pragma comment(lib, "ws2_32.lib") // Link with ws2_32.lib
#else
    #include <arpa/inet.h>
    #include <sys/socket.h> // For AF_INET
#endif

#include "core/msg_logger.h"
#include "PortFactory.h"
#include "core/tcpPort.h"

/**
 * Unlike other PortFactories, TcpServerPortFactory is NOT a singleton - since there maybe multiple instances which listen an unique ports, etc.
 * By this same logic, it may make sense there no PortFactory should be a singleton; but this is definitely the first case that warrants it
 */
class TcpServerPortFactory : public PortFactory {
public:
    struct {
        uint16_t listenerPort = 4321;       //!< the tcp port to listen for incoming connections on
        struct sockaddr_in listeningAddr;   //!< listening address (this binds to a specific interface, defaults to 127.0.0.1
        bool backgroundListener = false;    //!< if true, we'll setup a thread to process incoming connections -- note that this doesn't service those connections, just the listener
        int maxConnections = 10;            //!< the maximum number of connections that can be kept open - additional connection requests will be rejected
        bool portDefaultBlocking = false;   //!< if true, created tcpPorts will be configured for blocking by default (usually, we don't want that).
        //!< the maximum number of connections that can be kept open
    } factoryOptions = {};

    TcpServerPortFactory(uint16_t listenPort = 4321, std::string listenAddr = "127.0.0.1", int maxConnections = 10, bool portDefaultBlocking = false, bool backgroundListener = false) {
#ifdef PLATFORM_IS_LINUX
        signal(SIGPIPE, SIG_IGN); // ignore broken pipes
#endif
        factoryOptions.listenerPort = listenPort;
        factoryOptions.maxConnections = maxConnections;
        factoryOptions.portDefaultBlocking = portDefaultBlocking;
        factoryOptions.backgroundListener = backgroundListener;
        factoryOptions.listeningAddr.sin_family = AF_INET;
        factoryOptions.listeningAddr.sin_port = htons(listenPort);

        // we expect either a string "<address>" or "<address> (<name>)" - in either case, we just want the <address> part (upto the space)
        std::string ipAddr = listenAddr.substr(0, listenAddr.find_first_of(' '));

        struct in_addr addr;
        if (inet_pton(AF_INET, ipAddr.c_str(), &addr) <= 0) {
            // Handle error: invalid address or address not supported
            factoryOptions.listeningAddr.sin_addr.s_addr = INADDR_NONE; // A common error indicator for in_addr_t
        }
        factoryOptions.listeningAddr.sin_addr = addr;

    };
    ~TcpServerPortFactory() = default;

    // TcpServerPortFactory(TcpServerPortFactory const &) = delete;
    // TcpServerPortFactory& operator=(TcpServerPortFactory const&) = delete;

    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    bool releasePort(port_handle_t port) override;

protected:
    struct socket_entry_t {
        int socket = 0;
        std::string portName = "";
        mutable port_handle_t port = nullptr;

        socket_entry_t(int _s, std::string _n, port_handle_t _p = nullptr) : socket(_s), portName(_n), port(_p) {};

        // Overload operator< for strict weak ordering
        bool operator<(const socket_entry_t& other) const {
            return (socket != other.socket) ? socket < other.socket : portName < other.portName; // Secondary sorting criterion
        }

    };

    bool startListening();

    void stopListening();

    std::vector<socket_entry_t> getClientSockets() {
        std::vector<socket_entry_t> out;
        for (auto ks : knownSockets)
            out.emplace_back(ks);
        return out;
    }

    void shutdownAllClients() {
        for (auto ks : knownSockets)
            shutdown(ks.socket, SHUT_RDWR);
    }


/**
     * The primary service routine - this should be called periodically (and frequently) to service incoming connections.
     * If this is not called, no ports will ever be discovered/created
     */
    bool processPendingConnections(std::function<void(socket_entry_t)> cb);

private:
    std::set<socket_entry_t> knownSockets;
    std::set<socket_entry_t> prevSockets;

    int listen_fd = 0; /* listener socket */
};


#endif //IS_SDK__TCP_SERVER_PORT_FACTORY_H
