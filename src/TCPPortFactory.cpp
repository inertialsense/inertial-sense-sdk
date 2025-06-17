//
// Created by firiusfoxx on 6/12/25.
//

#include "TCPPortFactory.h"

#include <arpa/inet.h>

#include <iostream>
#include <regex>
#include "PortManager.h"

port_handle_t TCPPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    if (!validatePort(pName, pType))
        return nullptr;

    // Parse pName for address
    const URL url = parseURL(pName);
    if (url.protocol != "tcp")
        return nullptr;

    sockaddr addr = {};
    sockaddr ipaddr = {};
    if (inet_pton(AF_INET, url.address.c_str(), &ipaddr)) {
        addr.sa_family = AF_INET;
        const auto ipv4 = reinterpret_cast<sockaddr_in*>(&addr);
        ipv4->sin_port = htons(stoi(url.port));
        ipv4->sin_addr = *reinterpret_cast<in_addr*>(&ipaddr);
    } else if (inet_pton(AF_INET6, url.address.c_str(), &ipaddr)) {
        addr.sa_family = AF_INET6;
        const auto ipv6 = reinterpret_cast<sockaddr_in6*>(&addr);
        ipv6->sin6_port = htons(stoi(url.port));
        ipv6->sin6_addr = *reinterpret_cast<in6_addr*>(&ipaddr);
    } else {
        return nullptr;
    }

    auto* tcpPort = new tcp_port_t;
    auto port = (port_handle_t)tcpPort;
    *tcpPort = {};
    auto id = static_cast<uint16_t>(PortManager::getInstance().getPortCount());
    tcpPortInit(port, id, pType,  this->portOptions.defaultBlocking, pName.c_str(), &addr);

    return port;
}

bool TCPPortFactory::releasePort(port_handle_t port) {
    if (!port)
        return false;

    debug_message("[DBG] Releasing network port '%s'\n", ((tcp_port_t*)port)->portName);
    tcpPortDelete(port);
    delete static_cast<tcp_port_t*>(port);

    return true;
}

bool TCPPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    const URL url = parseURL(pName);
    if (url.protocol != "tcp")
        return false;

    sockaddr addr = {};
    if (inet_pton(AF_INET, url.address.c_str(), &addr)) {
        return true;
    }
    if (inet_pton(AF_INET6, url.address.c_str(), &addr)) {
        return true;
    }

    return false;
}

void TCPPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    // The base TCP Port Factory doesn't provide a discovery service, but we must still "locate" any ports we determine are valid
    if (validatePort(pattern, pType)) {
        portCallback(this, PORT_TYPE__TCP | PORT_TYPE__COMM, pattern);
    }
}

TCPPortFactory::URL TCPPortFactory::parseURL(const std::string& pName) {
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
