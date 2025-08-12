/**
 * @file TcpPortFactory.cpp
 * @brief This is a port factory used to a single known devices over TCP/IP
 *
 * @author FiriusFoxx on 2025-06-12.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#ifdef _WIN32
// Windows.h is included somewhere and this prevents it from max as a macro which breaks uri.hpp
#define NOMINMAX
#endif

#include "TcpPortFactory.h"
#include "PortManager.h"
#include <iostream>
#include <uri.hpp>
#include <util.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#endif

/**
 * This function parses and creates a new port_handle_t repersenting a TCP Port
 * when passed a URL in the format tcp://ipAddr:port to pName and a pType of PORT_TYPE__TCP | PORT_TYPE__COMM
 * @param pName The URL and name of the new port to bind a port_handle_to
 * @param pType The port type requested to be generated
 * @return A port_handle_t bound to the newly created TCP port for the connection pName represents
 */
port_handle_t TcpPortFactory::bindPort(const std::string& pName, uint16_t pType) {
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

    sockaddr_storage addr = {};
    sockaddr_storage ipaddr = {};
    struct addrinfo hints = {}, *dns_addr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    if (inet_pton(AF_INET, uriHost.c_str(), &ipaddr)) {
        addr.ss_family = AF_INET;
        auto* ipv4 = reinterpret_cast<sockaddr_in*>(&addr);
        ipv4->sin_port = htons(std::stoi(uriPort));
        ipv4->sin_addr = *reinterpret_cast<in_addr*>(&ipaddr);
    } else if (inet_pton(AF_INET6, uriHost.c_str(), &ipaddr)) {
        addr.ss_family = AF_INET6;
        auto* ipv6 = reinterpret_cast<sockaddr_in6*>(&addr);
        ipv6->sin6_port = htons(std::stoi(uriPort));
        ipv6->sin6_addr = *reinterpret_cast<in6_addr*>(&ipaddr);
    } else if (getaddrinfo(uriHost.c_str(), uriPort.c_str(), &hints, &dns_addr) == 0) {
        addr.ss_family = dns_addr->ai_family;
        if (addr.ss_family == AF_INET) {
            auto* ipv4 = reinterpret_cast<sockaddr_in*>(&addr);
            auto* ipv4_from_dns = reinterpret_cast<sockaddr_in*>(dns_addr->ai_addr);
            ipv4->sin_family = ipv4_from_dns->sin_family;
            ipv4->sin_port = ipv4_from_dns->sin_port;
            ipv4->sin_addr = ipv4_from_dns->sin_addr;
        } else if (addr.ss_family == AF_INET6) {
            auto* ipv6 = reinterpret_cast<sockaddr_in6*>(&addr);
            auto* ipv6_from_dns = reinterpret_cast<sockaddr_in6*>(dns_addr->ai_addr);
            ipv6->sin6_family = ipv6_from_dns->sin6_family;
            ipv6->sin6_port = ipv6_from_dns->sin6_port;
            ipv6->sin6_flowinfo = ipv6_from_dns->sin6_flowinfo;
            ipv6->sin6_addr = ipv6_from_dns->sin6_addr;
            ipv6->sin6_scope_id = ipv6_from_dns->sin6_scope_id;
        }
        freeaddrinfo(dns_addr);
    } else {
        return nullptr;
    }

    auto* tcpPort = new tcp_port_t;
    auto port = (port_handle_t)tcpPort;
    *tcpPort = {};
    auto id = static_cast<uint16_t>(PortManager::getInstance().getPortCount());
    tcpPortInit(port, id, this->portOptions.defaultBlocking, pName.c_str(), &addr);

    return port;
}

/**
 * Releases and frees the memory used by this port
 * @param port The TCP Port handle to deinitialize
 * @return True if successful, false otherwise
 */
bool TcpPortFactory::releasePort(port_handle_t port) {
    if (!port) {
        return false;
    }

    debug_message("[DBG] Releasing network port '%s'\n", portName(port));
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
bool TcpPortFactory::validatePort(const std::string& pName, uint16_t pType) {
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

/**
 * TCP Port Factory implements a stub function that only creates 1 port if the provided pattern is a valid TCP port url
 * @param portCallback The function to callback into to indicate that this port has been "found"
 * @param pattern The URL to validate and "discover"
 * @param pType Ignored
 */
void TcpPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    // The base TCP Port Factory doesn't provide a discovery service, but we must still "locate" any ports we determine are valid
    if (validatePort(pattern, PORT_TYPE__TCP | PORT_TYPE__COMM)) {
        portCallback(this, PORT_TYPE__TCP | PORT_TYPE__COMM, pattern);
    }
}