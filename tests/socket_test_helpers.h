/**
 * @file socket_test_helpers.h
 * @brief Portable (POSIX and Winsock) loopback socket helpers for tests that need to look at a TCP server from the outside.
 *
 * tcp_test_helpers.h and ntrip_test_helpers.h are POSIX-only; these helpers cover the small set of raw-socket operations that the
 * correction-server tests need on every platform: a listener on an ephemeral port, a connect probe, an accept with a timeout and a way
 * to find an address of this host that is not the loopback address.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_SOCKET_TEST_HELPERS_H
#define IS_SDK_SOCKET_TEST_HELPERS_H

#include "TcpServerPortFactory.h"  // pulls in the platform socket headers (winsock2/ws2tcpip, or the POSIX ones)

#include <cstddef>
#include <cstdint>
#include <string>

#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace socket_test {

#if defined(_WIN32)
using sock_t = SOCKET;
constexpr sock_t INVALID_SOCK = INVALID_SOCKET;
inline void closeSock(sock_t s) { closesocket(s); }
#else
using sock_t = int;
constexpr sock_t INVALID_SOCK = -1;
inline void closeSock(sock_t s) { close(s); }
#endif

constexpr uint32_t LOOPBACK_ADDR = 0x7F000001;  //!< 127.0.0.1 in host byte order

/** Starts Winsock once per process (a no-op on POSIX). Every helper below calls it. */
inline void init() {
#if defined(_WIN32)
    static const bool started = [] {
        WSADATA data;
        return WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }();
    (void)started;
#endif
}

/** @return a sockaddr_in for @p hostOrderAddr : @p port */
inline sockaddr_in makeAddr(uint32_t hostOrderAddr, uint16_t port) {
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(hostOrderAddr);
    addr.sin_port = htons(port);
    return addr;
}

/** @return @p hostOrderAddr as a dotted-quad string */
inline std::string toString(uint32_t hostOrderAddr) {
    in_addr a = {};
    a.s_addr = htonl(hostOrderAddr);
    char buf[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &a, buf, sizeof(buf));
    return buf;
}

/**
 * Opens a TCP listener on @p hostOrderAddr and an ephemeral port.
 * @param[out] port the port that was assigned
 * @return the listening socket, or INVALID_SOCK
 */
inline sock_t listenOn(uint32_t hostOrderAddr, uint16_t& port) {
    init();
    const sock_t s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == INVALID_SOCK) return INVALID_SOCK;
    sockaddr_in addr = makeAddr(hostOrderAddr, 0);
    socklen_t len = sizeof(addr);
    if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 || listen(s, 16) != 0 || getsockname(s, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        closeSock(s);
        return INVALID_SOCK;
    }
    port = ntohs(addr.sin_port);
    return s;
}

/** @return a TCP port that is currently unused on the loopback interface (0 on failure) */
inline uint16_t freeLoopbackPort() {
    uint16_t port = 0;
    const sock_t s = listenOn(LOOPBACK_ADDR, port);
    if (s != INVALID_SOCK) closeSock(s);
    return port;
}

/** @return true if a TCP connection to @p hostOrderAddr : @p port succeeds */
inline bool canConnect(uint32_t hostOrderAddr, uint16_t port) {
    init();
    const sock_t s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == INVALID_SOCK) return false;
    const sockaddr_in addr = makeAddr(hostOrderAddr, port);
    const bool ok = connect(s, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0;
    closeSock(s);
    return ok;
}

/** @return a connected client socket to @p hostOrderAddr : @p port, or INVALID_SOCK */
inline sock_t connectTo(uint32_t hostOrderAddr, uint16_t port) {
    init();
    const sock_t s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == INVALID_SOCK) return INVALID_SOCK;
    const sockaddr_in addr = makeAddr(hostOrderAddr, port);
    if (connect(s, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        closeSock(s);
        return INVALID_SOCK;
    }
    return s;
}

/** @return the connection accepted on @p listener within @p timeoutMs, or INVALID_SOCK */
inline sock_t acceptWithTimeout(sock_t listener, int timeoutMs) {
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(listener, &readSet);
    timeval tv = {timeoutMs / 1000, (timeoutMs % 1000) * 1000};
    if (select(static_cast<int>(listener) + 1, &readSet, nullptr, nullptr, &tv) != 1) return INVALID_SOCK;
    return accept(listener, nullptr, nullptr);
}

/** @return true if all @p length bytes were sent on @p s */
inline bool sendAll(sock_t s, const uint8_t* data, size_t length) {
    size_t sent = 0;
    while (sent < length) {
        const int n = send(s, reinterpret_cast<const char*>(data + sent), static_cast<int>(length - sent), 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

/**
 * @return an IPv4 address of this host that is not a loopback address (host byte order), or 0 if there is none. The address is the one the
 * operating system would use to reach an external destination: a UDP socket is connected to a documentation address (RFC 5737), which
 * sends no traffic, and its local address is read back.
 */
inline uint32_t nonLoopbackAddress() {
    init();
    const sock_t s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s == INVALID_SOCK) return 0;
    uint32_t result = 0;
    const sockaddr_in remote = makeAddr(0xC0000201 /* 192.0.2.1 */, 9);
    sockaddr_in local = {};
    socklen_t len = sizeof(local);
    if (connect(s, reinterpret_cast<const sockaddr*>(&remote), sizeof(remote)) == 0 && getsockname(s, reinterpret_cast<sockaddr*>(&local), &len) == 0) {
        const uint32_t addr = ntohl(local.sin_addr.s_addr);
        if ((addr >> 24) != 127 && addr != 0) result = addr;
    }
    closeSock(s);
    return result;
}

/**
 * Control for assertions that a loopback-only server is NOT reachable through @p otherAddr: opens a plain wildcard listener, independent of
 * the code under test, and checks that it is reachable through @p otherAddr. A host firewall or an unusual network setup that blocks this makes
 * a "refused" result meaningless, so callers should skip the assertion when this returns false. The result is cached for the process.
 */
inline bool nonLoopbackReachable(uint32_t otherAddr) {
    static const bool reachable = [otherAddr] {
        uint16_t port = 0;
        const sock_t s = listenOn(0 /* INADDR_ANY */, port);
        if (s == INVALID_SOCK) return false;
        const bool ok = canConnect(otherAddr, port);
        closeSock(s);
        return ok;
    }();
    return reachable;
}

}  // namespace socket_test

#endif  // IS_SDK_SOCKET_TEST_HELPERS_H
