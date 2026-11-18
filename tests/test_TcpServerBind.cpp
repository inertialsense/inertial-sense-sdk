/**
 * @file test_TcpServerBind.cpp
 * @brief SN-9076 (GitHub #1380): TcpServerPortFactory must listen on the address it was configured with.
 *
 * The factory stores its requested address (default 127.0.0.1) in factoryOptions.listeningAddr. The tests observe the listener from the
 * outside, on every platform: a loopback-only server must accept a loopback connection and refuse one made through the host's non-loopback
 * address, an explicit 0.0.0.0 must accept both, and an address that cannot be parsed as IPv4 must not start a listener at all.
 */
#include <gtest/gtest.h>

#include <string>

#include "Rtcm3CorrectionServer.h"
#include "TcpServerPortFactory.h"
#include "socket_test_helpers.h"

namespace {

using namespace socket_test;

/** A TcpServerPortFactory whose (protected) startListening() the tests can call. */
class TestFactory : public TcpServerPortFactory {
   public:
    using TcpServerPortFactory::startListening;
    using TcpServerPortFactory::TcpServerPortFactory;
};

/** @return the non-loopback address to probe with, or 0 if this host cannot tell a loopback-only listener from a wildcard one */
uint32_t probeAddress() {
    const uint32_t other = nonLoopbackAddress();
    return (other && nonLoopbackReachable(other)) ? other : 0;
}

/**
 * Asserts that the listener on @p port accepts a loopback connection and, where this host can tell, refuses one made through its
 * non-loopback address.
 */
void expectLoopbackOnly(uint16_t port) {
    EXPECT_TRUE(canConnect(LOOPBACK_ADDR, port)) << "the listener does not accept loopback connections";
    const uint32_t other = probeAddress();
    if (other) EXPECT_FALSE(canConnect(other, port)) << "a loopback-only server accepted a connection on " << toString(other);
}

}  // namespace

TEST(test_TcpServerBind, listensOnConfiguredLoopbackAddress) {
    const uint16_t port = freeLoopbackPort();
    ASSERT_NE(0, port);
    TestFactory factory(port, "127.0.0.1");
    ASSERT_TRUE(factory.startListening());
    expectLoopbackOnly(port);
}

TEST(test_TcpServerBind, defaultAddressIsLoopback) {
    const uint16_t port = freeLoopbackPort();
    ASSERT_NE(0, port);
    TestFactory factory(port);
    ASSERT_TRUE(factory.startListening());
    expectLoopbackOnly(port);
}

TEST(test_TcpServerBind, explicitWildcardBindsAllInterfaces) {
    const uint16_t port = freeLoopbackPort();
    ASSERT_NE(0, port);
    TestFactory factory(port, "0.0.0.0");
    ASSERT_TRUE(factory.startListening());
    EXPECT_TRUE(canConnect(LOOPBACK_ADDR, port));
    const uint32_t other = probeAddress();
    if (other) EXPECT_TRUE(canConnect(other, port)) << "a server configured for 0.0.0.0 refused a connection on " << toString(other);
}

TEST(test_TcpServerBind, addressWithNameSuffixUsesTheAddressPart) {
    // configure() documents "<address> (<name>)" as an accepted form.
    const uint16_t port = freeLoopbackPort();
    ASSERT_NE(0, port);
    TestFactory factory(port, "127.0.0.1 (lo)");
    ASSERT_TRUE(factory.startListening());
    expectLoopbackOnly(port);
}

TEST(test_TcpServerBind, unparsableAddressDoesNotListenOnAllInterfaces) {
    for (const char* bad : {"not-an-address", "::1", "999.1.1.1"}) {
        const uint16_t port = freeLoopbackPort();
        ASSERT_NE(0, port);
        TestFactory factory(port, bad);
        EXPECT_FALSE(factory.startListening()) << "address \"" << bad << "\" must not start a listener";
        EXPECT_EQ(TCP_LISTEN_CTX__BIND, factory.getLastListenError().first) << "address \"" << bad << "\"";
        EXPECT_FALSE(canConnect(LOOPBACK_ADDR, port)) << "address \"" << bad << "\" left a listener running";
    }
}

TEST(test_TcpServerBind, rtcm3CorrectionServer_ExplicitAddressAndUriDefault) {
    {
        const uint16_t port = freeLoopbackPort();
        ASSERT_NE(0, port);
        Rtcm3CorrectionServer server(static_cast<int>(port), "127.0.0.1");
        expectLoopbackOnly(port);
    }
    {
        // "tcp://:PORT" omits the host: the documented default (127.0.0.1) must apply, as in `cltool -base=tcp://:7777`.
        const uint16_t port = freeLoopbackPort();
        ASSERT_NE(0, port);
        Rtcm3CorrectionServer server("tcp://:" + std::to_string(port));
        expectLoopbackOnly(port);
    }
}

TEST(test_TcpServerBind, loopbackServerIsNotReachableThroughOtherInterfaces) {
    const uint32_t other = probeAddress();
    if (!other) GTEST_SKIP() << "this host has no non-loopback IPv4 address that a plain wildcard listener can be reached on";

    const uint16_t port = freeLoopbackPort();
    ASSERT_NE(0, port);
    Rtcm3CorrectionServer server(static_cast<int>(port), "127.0.0.1");
    EXPECT_TRUE(canConnect(LOOPBACK_ADDR, port));
    EXPECT_FALSE(canConnect(other, port)) << "a server configured for 127.0.0.1 accepted a connection on " << toString(other);
}
