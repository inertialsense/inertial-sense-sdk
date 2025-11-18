/**
 * @file TcpServerPortFactory.cpp 
 * @brief This is a PortFactory which binds a socket listener to a specified server port, and creates new tcpPorts for each
 *  incoming connection request on that port. This class ideally is a base class for specific services which listen on specific
 *  ports. For example, a Rtcm3ServicePortFactory might extend TcpServerPortFactory to handle incoming connections and bind
 *  the requesting client to the source device.
 *
 * @author Kyle Mallory on 10/17/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifdef _WIN32
// Windows.h is included somewhere and this prevents it from max as a macro which breaks uri.hpp
#define NOMINMAX
#endif

#include <cerrno>
#include <iostream>
#include <uri.hpp>
#include <util.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

#include "TcpPortFactory.h"
#include "TcpServerPortFactory.h"
#include "PortManager.h"


/**
 * TCP Server Port Factory implements a tcp socket listener, which then creates new "discoverable" ports when an incoming
 * connection request is made on the listening tcp port.
 * @param portCallback The function to callback into to indicate that this port has been "found"
 * @param pattern The URL to validate and "discover"
 * @param pType Ignored
 */
void TcpServerPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {

    if (!listen_fd)
        startListening();

    processPendingConnections([&](socket_entry_t e) {
        // The base TCP Port Factory doesn't provide a discovery service, but we must still "locate" any ports we determine are valid
        if (validatePort(e.portName, PORT_TYPE__TCP | PORT_TYPE__COMM)) {
            portCallback(this, PORT_TYPE__TCP | PORT_TYPE__COMM, e.portName);
        }
    });
}

/**
 * This function parses and creates a new port_handle_t repersenting a TCP Port
 * when passed a URL in the format tcp://ipAddr:port to pName and a pType of PORT_TYPE__TCP | PORT_TYPE__COMM
 * @param pName The URL and name of the new port to bind a port_handle_to
 * @param pType The port type requested to be generated
 * @return A port_handle_t bound to the newly created TCP port for the connection pName represents
 */
port_handle_t TcpServerPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    if (!validatePort(pName, pType)) {
        return nullptr;
    }

    // Parse pName for address
    const FIX8::uri url {pName};
    if (url.get_scheme() != "tcp" || url.get_port().empty() || url.get_host().empty()) {
        return nullptr;
    }
    std::string uriHost {url.get_host()};
    if (uriHost.starts_with("[") && uriHost.ends_with("]")) {
        uriHost = uriHost.substr(1, uriHost.size() - 2);
    }
    std::string uriPort {url.get_port()};

    // locate this port name in our list of known sockets
    for (auto& e : knownSockets) {
        if (e.portName == pName) {
            // all the socket management/setup has already been done - we just need to create a tcpPort tied to the socket and name
            auto* tcpPort = new tcp_port_t;
            *tcpPort = {};
            e.port = (port_handle_t)tcpPort;
            auto id = static_cast<uint16_t>(PortManager::getInstance().getPortCount());
            tcpPortInitWithSocket(e.port, id, pType, e.portName.c_str(), e.socket, this->factoryOptions.portDefaultBlocking);
            return e.port;
        }
    }
    return nullptr;
}

/**
 * Releases and frees the memory used by this port
 * @param port The TCP Port handle to deinitialize
 * @return True if successful, false otherwise
 */
bool TcpServerPortFactory::releasePort(port_handle_t port) {
    if (!port) {
        return false;
    }

    log_debug(IS_LOG_PORT_FACTORY, "Releasing TCP/network port '%s'\n", portName(port));

    for (auto it = knownSockets.begin(); it != knownSockets.end(); it++) {
        if (it->port == port) {
            knownSockets.erase(it);
            break;  // once we erase ourselves, we can't iterate anymore, so exit the for-loop
        }
    }
    tcpPortDelete(port);
    delete static_cast<tcp_port_t*>(port);

    return true;
}

/**
 * Validate that a provided pName can create a TCP Port
 * @param pName The URL to validate starting with tcp://
 * @param pType Must be PORT_TYPE__TCP | PORT_TYPE__COMM
 * @return True if port can be created, false otherwise
 */
bool TcpServerPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    const FIX8::uri url {pName};
    if (url.get_scheme() != "tcp" || url.get_port().empty() || url.get_host().empty()) {
        return false;
    }
    std::string uriHost {url.get_host()};
    if (uriHost.starts_with("[") && uriHost.ends_with("]")) {
        uriHost = uriHost.substr(1, uriHost.size() - 2);
    }
    std::string uriPort {url.get_port()};

    if (pType != (PORT_TYPE__TCP | PORT_TYPE__COMM)) {
        return false;
    }

    sockaddr_storage addr = {};
    if (inet_pton(AF_INET, uriHost.c_str(), &addr)) {
        return true;
    }
    if (inet_pton(AF_INET6, uriHost.c_str(), &addr)) {
        return true;
    }
    struct addrinfo hints = {}, *dns_addr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    if (getaddrinfo(uriHost.c_str(), uriPort.c_str(), &hints, &dns_addr) == 0) {
        return true;
    }

    return false;
}

bool TcpServerPortFactory::startListening() {
    struct sockaddr_in serveraddr; /* server's addr */

    // socket: create the parent socket
    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0)
        return false;   // ERROR opening socket;

#ifdef PLATFORM_IS_WINDOWS
    // setsockopt: Handy debugging trick that lets us rerun the server immediately after we kill it;
    // otherwise we have to wait about 20 secs.  Eliminates "ERROR on binding: Address already in use" error.
    int optval = 1;  /* flag value for setsockopt */
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, (char*)&optval , sizeof(int));

    DWORD nonBlocking = 1;
    ioctlsocket(listen_fd, FIONBIO, &nonBlocking);
#else
    // setsockopt: Handy debugging trick that lets us rerun the server immediately after we kill it;
    // otherwise we have to wait about 20 secs.  Eliminates "ERROR on binding: Address already in use" error.
    int optval = 1;  /* flag value for setsockopt */
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, (void*)&optval , sizeof(int));

    // Get the current flags for the socket file descript
    int flags;
    if ((flags = fcntl(listen_fd, F_GETFL, 0)) == -1) {
        return -1; // Error getting flags
    }

    // Add the O_NONBLOCK flag to the current flags
    if (fcntl(listen_fd, F_SETFL, flags | O_NONBLOCK) == -1) {
        return -1; // Error setting non-blocking flag
    }
#endif

    // build the server's Internet address
    memset((char *) &serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;                                            // this is an Internet address
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);                             // let the system figure out our IP address
    serveraddr.sin_port = htons((unsigned short)factoryOptions.listenerPort);   // this is the port we will listen on

    // bind: associate the parent socket with a port
    if (bind(listen_fd, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0)
        return false; // ERROR on binding

    // listen: make this socket ready to accept connection requests
    if (listen(listen_fd, factoryOptions.maxConnections) < 0) /* allow 5 requests to queue up */
        return false; // ERROR on listen

    return true;
}

void TcpServerPortFactory::stopListening() {
    close(listen_fd);
}

/**
 * This is a private function, typically called by locatePorts() to process all pending connection requests.
 * @return
 */
bool TcpServerPortFactory::processPendingConnections(std::function<void(socket_entry_t)> cb) {
    struct sockaddr_in clientaddr;      // client addr
    struct hostent *hostp;              // client host info
    char *hostaddrp;                    // dotted decimal host addr string
    int clientfd = 0;                   // socket associated to the client
    socklen_t clientlen = sizeof(clientaddr); // byte size of client's address
    bool result = false;                // true if one or more connections are successfully accepted

    do {
        // accept: wait for a connection request
        clientfd = accept(listen_fd, (struct sockaddr *) &clientaddr, &clientlen);
        if (clientfd >= 0) {
            // gethostbyaddr: determine who has connected
            hostp = gethostbyaddr((const char *)&clientaddr.sin_addr.s_addr, sizeof(clientaddr.sin_addr.s_addr), AF_INET);
            if (hostp) {
                hostaddrp = inet_ntoa(clientaddr.sin_addr);
                if (hostaddrp) {
                    std::string tcpPortName = utils::string_format("tcp://%s:%d", hostaddrp, clientaddr.sin_port);

                    auto out = knownSockets.emplace(clientfd, tcpPortName);
                    cb(*out.first);
                    result = out.second;
                    log_info(IS_LOG_PORT_FACTORY, "tcpServerPortFactory accepted incoming TCP port %s", tcpPortName.c_str());
                } else {
                    log_warn(IS_LOG_PORT_FACTORY, "tcpServerPortFactory received an invalid socket when accepting ")
                }
            }
            else {
                log_warn(IS_LOG_PORT_FACTORY, "tcpServerPortFactory unable to extract remote IP address from socket.");
            }
        } else if ((clientfd != EAGAIN) && (clientfd != EWOULDBLOCK)) {
            log_error(IS_LOG_PORT_FACTORY, "tcpServerPortFactory::accept() reported an error while accepting incoming connections.")
            return result;
        }
    } while ((clientfd != EAGAIN) && (clientfd != EWOULDBLOCK));

    return result;
}
