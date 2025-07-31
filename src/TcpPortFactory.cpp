//
// Created by firiusfoxx on 6/12/25.
//

#include "TcpPortFactory.h"

#ifdef PLATFORM_IS_WINDOWS
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif

#include <iostream>
#include <regex>
#include "PortManager.h"

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
    const URL url = parseURL(pName);
    if (url.protocol != "tcp") {
        return nullptr;
    }

    sockaddr_storage addr = {};
    sockaddr ipaddr = {};
    if (inet_pton(AF_INET, url.address.c_str(), &ipaddr)) {
        addr.ss_family = AF_INET;
        auto* ipv4 = reinterpret_cast<sockaddr_in*>(&addr);
        ipv4->sin_port = htons(stoi(url.port));
        ipv4->sin_addr = *reinterpret_cast<in_addr*>(&ipaddr);
    } else if (inet_pton(AF_INET6, url.address.c_str(), &ipaddr)) {
        addr.ss_family = AF_INET6;
        auto* ipv6 = reinterpret_cast<sockaddr_in6*>(&addr);
        ipv6->sin6_port = htons(stoi(url.port));
        ipv6->sin6_addr = *reinterpret_cast<in6_addr*>(&ipaddr);
    } else {
        return nullptr;
    }

    auto* tcpPort = new tcp_port_t;
    auto port = (port_handle_t)tcpPort;
    *tcpPort = {};
    auto id = static_cast<uint16_t>(PortManager::getInstance().getPortCount());
    tcpPortInit(port, id, this->portOptions.defaultBlocking, pName.c_str(), reinterpret_cast<const sockaddr*>(&addr));

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

    debug_message("[DBG] Releasing network port '%s'\n", ((tcp_port_t*)port)->portName);
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
    const URL url = parseURL(pName);
    if (url.protocol != "tcp") {
        return false;
    }

    if (pType != (PORT_TYPE__TCP | PORT_TYPE__COMM)) {
        return false;
    }

    sockaddr_storage addr = {};
    if (inet_pton(AF_INET, url.address.c_str(), &addr)) {
        return true;
    }
    if (inet_pton(AF_INET6, url.address.c_str(), &addr)) {
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

/**
 * Parse a URL into a TCPPortFactory::URL
 * @param pName URL to attempt to parse
 * @return TCPPortFactory:URL that represents the parsed URL
 */
TcpPortFactory::URL TcpPortFactory::parseURL(const std::string& pName) {
    std::regex regexp(R"(^([^:\/?#]+):\/\/:?([^\/ ]*)?:([^\/?#\D]*)\/?([^?#]*)?\??([^#]*)?#?(.*)?$)");
    std::smatch match;
    URL retval = {};
    if (std::regex_search(pName, match, regexp)) {
        retval.fullurl = match[0].str();
        retval.protocol = match[1].str();
        retval.address = match[2].str();
        retval.port = match[3].str();
        retval.path = match[4].str();
        retval.params = match[5].str();
        retval.tags = match[6].str();
    }
    return retval;
}
