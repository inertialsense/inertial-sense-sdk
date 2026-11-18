/**
 * @file test_TcpServerBind.cpp
 * @brief SN-9076 (GitHub #1380): TcpServerPortFactory must listen on the address it was configured with.
 *
 * The factory stores its requested address (default 127.0.0.1) in factoryOptions.listeningAddr. These tests read the address of the
 * listening socket with getsockname() and, where the host has a non-loopback IPv4 address, check that a loopback-only server cannot be
 * reached through it.
 */
#if !defined(_WIN32)

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>

// startListening() is protected and listen_fd is private; the tests inspect the listener directly.
#define private   public
#define protected public
#include "Rtcm3CorrectionServer.h"
#include "TcpServerPortFactory.h"
#undef protected
#undef private

namespace {

/** @return a currently unused TCP port on the loopback interface. */
uint16_t freeLoopbackPort() {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);
    close(fd);
    return ntohs(addr.sin_port);
}

/** @return the IPv4 address (host order) the listening socket @p fd is bound to, or 0xFFFFFFFF if it cannot be read. */
uint32_t boundAddress(int fd) {
    sockaddr_in addr = {};
    socklen_t len = sizeof(addr);
    if (fd <= 0 || getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) return 0xFFFFFFFFu;
    return ntohl(addr.sin_addr.s_addr);
}

/** @return a non-loopback IPv4 address of this host (host order), or 0 if there is none. */
uint32_t nonLoopbackAddress() {
    uint32_t result = 0;
    ifaddrs* list = nullptr;
    if (getifaddrs(&list) != 0) return 0;
    for (ifaddrs* a = list; a && !result; a = a->ifa_next) {
        if (!a->ifa_addr || a->ifa_addr->sa_family != AF_INET || (a->ifa_flags & IFF_LOOPBACK)) continue;
        result = ntohl(reinterpret_cast<sockaddr_in*>(a->ifa_addr)->sin_addr.s_addr);
    }
    freeifaddrs(list);
    return result;
}

/** @return true if a TCP connection to @p hostOrderAddr : @p port succeeds. */
bool canConnect(uint32_t hostOrderAddr, uint16_t port) {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(hostOrderAddr);
    addr.sin_port = htons(port);
    const bool ok = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    close(fd);
    return ok;
}

}  // namespace

TEST(test_TcpServerBind, listensOnConfiguredLoopbackAddress) {
    TcpServerPortFactory factory(freeLoopbackPort(), "127.0.0.1");
    ASSERT_TRUE(factory.startListening());
    EXPECT_EQ(INADDR_LOOPBACK, boundAddress(factory.listen_fd));
}

TEST(test_TcpServerBind, defaultAddressIsLoopback) {
    TcpServerPortFactory factory(freeLoopbackPort());
    ASSERT_TRUE(factory.startListening());
    EXPECT_EQ(INADDR_LOOPBACK, boundAddress(factory.listen_fd));
}

TEST(test_TcpServerBind, explicitWildcardBindsAllInterfaces) {
    TcpServerPortFactory factory(freeLoopbackPort(), "0.0.0.0");
    ASSERT_TRUE(factory.startListening());
    EXPECT_EQ(INADDR_ANY, boundAddress(factory.listen_fd));
}

TEST(test_TcpServerBind, addressWithNameSuffixUsesTheAddressPart) {
    // configure() documents "<address> (<name>)" as an accepted form.
    TcpServerPortFactory factory(freeLoopbackPort(), "127.0.0.1 (lo)");
    ASSERT_TRUE(factory.startListening());
    EXPECT_EQ(INADDR_LOOPBACK, boundAddress(factory.listen_fd));
}

TEST(test_TcpServerBind, unparsableAddressDoesNotListenOnAllInterfaces) {
    for (const char* bad : {"not-an-address", "::1", "999.1.1.1"}) {
        TcpServerPortFactory factory(freeLoopbackPort(), bad);
        EXPECT_FALSE(factory.startListening()) << "address \"" << bad << "\" must not start a listener";
        EXPECT_EQ(TCP_LISTEN_CTX__BIND, factory.getLastListenError().first) << "address \"" << bad << "\"";
        EXPECT_NE(INADDR_ANY, boundAddress(factory.listen_fd)) << "address \"" << bad << "\"";
    }
}

TEST(test_TcpServerBind, rtcm3CorrectionServer_ExplicitAddressAndUriDefault) {
    {
        Rtcm3CorrectionServer server(static_cast<int>(freeLoopbackPort()), "127.0.0.1");
        EXPECT_EQ(INADDR_LOOPBACK, boundAddress(server.listen_fd));
    }
    {
        // "tcp://:PORT" omits the host: the documented default (127.0.0.1) must apply, as in `cltool -base=tcp://:7777`.
        Rtcm3CorrectionServer server("tcp://:" + std::to_string(freeLoopbackPort()));
        EXPECT_EQ(INADDR_LOOPBACK, boundAddress(server.listen_fd));
    }
}

TEST(test_TcpServerBind, loopbackServerIsNotReachableThroughOtherInterfaces) {
    const uint32_t other = nonLoopbackAddress();
    if (!other) GTEST_SKIP() << "no non-loopback IPv4 address on this host";

    const uint16_t port = freeLoopbackPort();
    Rtcm3CorrectionServer server(static_cast<int>(port), "127.0.0.1");
    EXPECT_TRUE(canConnect(INADDR_LOOPBACK, port));
    EXPECT_FALSE(canConnect(other, port)) << "a server configured for 127.0.0.1 accepted a connection on " << inet_ntoa({htonl(other)});
}

#endif  // !defined(_WIN32)
