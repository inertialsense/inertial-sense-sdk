/**
 * @file TcpPortFactory.cpp
 * @brief This is a port factory used to a single known devices over TCP/IP
 *
 * @author FiriusFoxx on 2025-06-12.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. Licensed under the MIT license
 */

#include "ISConstants.h"
#if PLATFORM_IS_WINDOWS
    // Windows.h is included somewhere and this prevents it from defining max as a macro which breaks uri.hpp
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
#endif

#include "TcpPortFactory.h"
#include "PortManager.h"
#include <iostream>
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <util/util.h>

#if PLATFORM_IS_WINDOWS
#include <winsock2.h>
#elif !PLATFORM_IS_EMBEDDED
#include <arpa/inet.h>
#include <netdb.h>
#endif

namespace {

/**
 * Cache host -> resolved address (port intentionally left zero) with a short TTL.
 *
 * A relay host serving many device ports hands out URIs that all share one hostname
 * (e.g. tcp://golden-planet-ce38.local:34663, :34664, ...). Resolving a `.local` name
 * goes through nss-mdns and blocks on a socket read. Without caching, validatePort and
 * bindPort each resolve, and every one of N same-host ports re-resolves every discovery
 * pass — tens of blocking syscalls per pass. Collapsing those to one resolve per host
 * per TTL keeps discovery responsive (SN-8175). Numeric literals bypass the cache.
 */
struct ResolvedHost {
    sockaddr_storage addr;   //!< family + address; sin_port / sin6_port left zero
    int              family;
    std::chrono::steady_clock::time_point when;
};

std::mutex                                  g_resolveCacheMutex;
std::unordered_map<std::string, ResolvedHost> g_resolveCache;
constexpr auto                              RESOLVE_CACHE_TTL = std::chrono::seconds(30);

/**
 * Resolve @p host (no service/port) into @p out (family + address only; caller sets the
 * port). Thread-safe; numeric addresses skip getaddrinfo and the cache.
 *
 * @param host        hostname or numeric IP literal to resolve
 * @param[out] out     filled with the resolved address (port left zero)
 * @param[out] family  the resolved address family (AF_INET / AF_INET6)
 * @return true on success, false if the host could not be resolved
 */
bool resolveHostCached(const std::string& host, sockaddr_storage& out, int& family) {
    sockaddr_storage tmp = {};
    if (inet_pton(AF_INET, host.c_str(), &reinterpret_cast<sockaddr_in*>(&tmp)->sin_addr) == 1) {
        out = {};
        out.ss_family = AF_INET;
        reinterpret_cast<sockaddr_in*>(&out)->sin_addr = reinterpret_cast<sockaddr_in*>(&tmp)->sin_addr;
        family = AF_INET;
        return true;
    }
    if (inet_pton(AF_INET6, host.c_str(), &reinterpret_cast<sockaddr_in6*>(&tmp)->sin6_addr) == 1) {
        out = {};
        out.ss_family = AF_INET6;
        reinterpret_cast<sockaddr_in6*>(&out)->sin6_addr = reinterpret_cast<sockaddr_in6*>(&tmp)->sin6_addr;
        family = AF_INET6;
        return true;
    }

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(g_resolveCacheMutex);
        auto it = g_resolveCache.find(host);
        if (it != g_resolveCache.end() && (now - it->second.when) < RESOLVE_CACHE_TTL) {
            out    = it->second.addr;
            family = it->second.family;
            return true;
        }
    }

    struct addrinfo hints = {}, *dns_addr = nullptr;
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    if (getaddrinfo(host.c_str(), nullptr, &hints, &dns_addr) != 0 || !dns_addr) {
        return false;
    }

    sockaddr_storage resolved = {};
    resolved.ss_family = dns_addr->ai_family;
    if (dns_addr->ai_family == AF_INET) {
        *reinterpret_cast<sockaddr_in*>(&resolved)  = *reinterpret_cast<sockaddr_in*>(dns_addr->ai_addr);
        reinterpret_cast<sockaddr_in*>(&resolved)->sin_port = 0;
    } else if (dns_addr->ai_family == AF_INET6) {
        *reinterpret_cast<sockaddr_in6*>(&resolved) = *reinterpret_cast<sockaddr_in6*>(dns_addr->ai_addr);
        reinterpret_cast<sockaddr_in6*>(&resolved)->sin6_port = 0;
    } else {
        freeaddrinfo(dns_addr);
        return false;
    }
    family = dns_addr->ai_family;
    freeaddrinfo(dns_addr);

    {
        std::lock_guard<std::mutex> lock(g_resolveCacheMutex);
        g_resolveCache[host] = ResolvedHost{resolved, family, now};
    }
    out = resolved;
    return true;
}

} // namespace

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
    const utils::UriParts url = utils::parseUri(pName);
    if (url.scheme != "tcp" || !url.hasPort() || !url.hasHost()) {
        return nullptr;
    }

    // Reuse the cached resolution from validatePort() above (same host, same TTL window)
    // so we don't re-run a blocking .local getaddrinfo here. resolveHostCached fills the
    // address only; set the port from this URI.
    sockaddr_storage addr = {};
    int family = 0;
    if (!resolveHostCached(url.host, addr, family)) {
        return nullptr;
    }
    if (family == AF_INET) {
        reinterpret_cast<sockaddr_in*>(&addr)->sin_port = htons(static_cast<uint16_t>(url.port));
    } else if (family == AF_INET6) {
        reinterpret_cast<sockaddr_in6*>(&addr)->sin6_port = htons(static_cast<uint16_t>(url.port));
    } else {
        return nullptr;
    }

    auto* tcpPort = new tcp_port_t;
    auto port = (port_handle_t)tcpPort;
    *tcpPort = {};
    auto id = static_cast<uint16_t>(PortManager::getInstance().getPortCount());
    tcpPortInit(port, id, pName.c_str(), &addr, this->portOptions.defaultBlocking ? PORT_FLAG__BLOCKING : 0);

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

    log_debug(IS_LOG_PORT, "Releasing TCP/network port '%s'", portName(port));
    tcpPortDelete(port);
    delete static_cast<tcp_port_t*>(port);

    return true;
}

/**
 * Validate that a provided pName can create a TCP Port
 * @param pName The URL to validate starting with tcp://
 * @param pType Must be PORT_TYPE__TCP
 * @return True if port can be created, false otherwise
 */
bool TcpPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    const utils::UriParts url = utils::parseUri(pName);
    if (url.scheme != "tcp" || !url.hasPort() || !url.hasHost()) {
        return false;
    }

    if ((pType & PORT_TYPE__TCP) != PORT_TYPE__TCP) {
        return false;
    }

    // Resolvability is the validation. Cached so a host shared by many device ports
    // resolves once per TTL window, and so bindPort() reuses this result (SN-8175).
    sockaddr_storage addr = {};
    int family = 0;
    return resolveHostCached(url.host, addr, family);
}

/**
 * TCP Port Factory implements a stub function that only creates 1 port if the provided pattern is a valid TCP port url
 * @param portCallback The function to callback into to indicate that this port has been "found"
 * @param pattern The URL to validate and "discover"
 * @param pType Ignored
 */
void TcpPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    // The base TCP Port Factory doesn't provide a discovery service, but we must still "locate" any ports we determine are valid
    if (validatePort(pattern, PORT_TYPE__TCP)) {
        portCallback(this, PORT_TYPE__TCP, pattern);
    }
}