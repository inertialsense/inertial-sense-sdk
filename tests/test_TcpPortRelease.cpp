/**
 * @file test_TcpPortRelease.cpp
 * @brief Releasing an open TCP port must close its socket.
 */

#include <gtest/gtest.h>

#if !defined(_WIN32)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <string>

#include "TcpPortFactory.h"
#include "TcpServerPortFactory.h"
#include "tcp_test_helpers.h"

TEST(TcpPortRelease, ReleaseWithoutCloseClosesSocket)
{
    // Local listener standing in for the remote end.
    int listener = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_GE(listener, 0);
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ASSERT_EQ(bind(listener, (sockaddr*)&addr, sizeof(addr)), 0);
    ASSERT_EQ(listen(listener, 1), 0);
    socklen_t len = sizeof(addr);
    ASSERT_EQ(getsockname(listener, (sockaddr*)&addr, &len), 0);

    const std::string url = "tcp://127.0.0.1:" + std::to_string(ntohs(addr.sin_port));
    TcpPortFactory& factory = TcpPortFactory::getInstance();
    port_handle_t port = factory.bindPort(url, PORT_TYPE__TCP | PORT_TYPE__COMM);
    ASSERT_NE(port, nullptr);
    ASSERT_EQ(portOpenRetry(port, 2000, 10), PORT_ERROR__NONE);
    ASSERT_TRUE(portIsOpened(port));

    int peer = accept(listener, nullptr, nullptr);
    ASSERT_GE(peer, 0);

    // Release without portClose(): the factory must not leak the connection.
    EXPECT_TRUE(factory.releasePort(port));

    pollfd pfd = { peer, POLLIN, 0 };
    ASSERT_EQ(poll(&pfd, 1, 1000), 1) << "peer never saw the connection close -- socket leaked";
    char c;
    EXPECT_EQ(recv(peer, &c, 1, 0), 0) << "expected EOF on the peer";

    close(peer);
    close(listener);
}


// Everything below extends the original test (from PR #1375 by @petrbel) to the other release paths and the no-op cases.
namespace {
class TestableServer : public TcpServerPortFactory {
public:
    using TcpServerPortFactory::TcpServerPortFactory;
    using TcpServerPortFactory::processPendingConnections;
    using TcpServerPortFactory::startListening;
    using TcpServerPortFactory::getClientConnectionCount;
};
}

TEST(TcpPortRelease, ServerFactoryReleaseWithoutCloseClosesSocket)
{
    uint16_t freePort;
    { tcp_test::Listener probe; freePort = probe.port; }      // pick a free port number
    TestableServer server(freePort, "127.0.0.1", 10);
    ASSERT_TRUE(server.startListening());
    // A client connects to the server's listening port.
    int client = -1;
    for (int attempt = 0; attempt < 50 && client < 0; attempt++) {
        client = tcp_test::connectTo(server.factoryOptions.listenerPort);
        if (client < 0) usleep(20000);
    }
    ASSERT_GE(client, 0);

    port_handle_t port = nullptr;
    for (int attempt = 0; attempt < 50 && !port; attempt++) {
        server.processPendingConnections([&](const auto& e) { port = server.bindPort(e.portName, PORT_TYPE__TCP | PORT_TYPE__COMM); });
        if (!port) usleep(20000);
    }
    ASSERT_NE(port, nullptr);
    ASSERT_EQ(server.getClientConnectionCount(), 1);

    EXPECT_TRUE(server.releasePort(port));
    EXPECT_EQ(server.getClientConnectionCount(), 0);
    EXPECT_TRUE(tcp_test::peerSeesEof(client)) << "client never saw the server-side connection close -- socket leaked";
    close(client);
}

TEST(TcpPortRelease, TcpPortDeleteClosesOpenSocket)
{
    // The mDNS and TCP-server factories also funnel through tcpPortDelete(); cover it directly.
    tcp_test::Listener listener;
    sockaddr_storage ss = {};
    auto* in = reinterpret_cast<sockaddr_in*>(&ss);
    in->sin_family = AF_INET;
    in->sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    in->sin_port = htons(listener.port);

    auto* tcp = new tcp_port_t;
    *tcp = {};
    tcpPortInit((port_handle_t)tcp, 1, "tcp-delete-test", &ss, 0);
    ASSERT_EQ(portOpenRetry((port_handle_t)tcp, 2000, 10), PORT_ERROR__NONE);
    const int sock = tcp->socket;
    ASSERT_GE(sock, 0);
    int peer = listener.acceptPeer();
    ASSERT_GE(peer, 0);

    tcpPortDelete((port_handle_t)tcp);
    EXPECT_FALSE(tcp_test::fdIsOpen(sock));
    EXPECT_TRUE(tcp_test::peerSeesEof(peer));
    close(peer);
    delete tcp;
}

TEST(TcpPortRelease, ReleaseAfterCloseIsNoOp)
{
    tcp_test::Listener listener;
    TcpPortFactory& factory = TcpPortFactory::getInstance();
    port_handle_t port = factory.bindPort(listener.url(), PORT_TYPE__TCP | PORT_TYPE__COMM);
    ASSERT_NE(port, nullptr);
    ASSERT_EQ(portOpenRetry(port, 2000, 10), PORT_ERROR__NONE);
    int peer = listener.acceptPeer();
    ASSERT_GE(peer, 0);

    portClose(port);
    // The kernel will hand out the just-freed descriptor number again; releasing the (already closed) port must not close it.
    int innocent = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_GE(innocent, 0);
    EXPECT_TRUE(factory.releasePort(port));
    EXPECT_TRUE(tcp_test::fdIsOpen(innocent)) << "release closed a descriptor it no longer owned";
    close(innocent);
    close(peer);
}

TEST(TcpPortRelease, ReleaseNeverOpenedPortIsSafe)
{
    tcp_test::Listener listener;
    TcpPortFactory& factory = TcpPortFactory::getInstance();
    port_handle_t port = factory.bindPort(listener.url(), PORT_TYPE__TCP | PORT_TYPE__COMM);
    ASSERT_NE(port, nullptr);

    const int before = tcp_test::countOpenFds();
    EXPECT_TRUE(factory.releasePort(port));
    EXPECT_EQ(tcp_test::countOpenFds(), before);
    EXPECT_TRUE(tcp_test::fdIsOpen(listener.fd));
    EXPECT_FALSE(factory.releasePort(nullptr));
}

TEST(TcpPortRelease, RepeatedConnectReleaseDoesNotGrowFdCount)
{
    tcp_test::Listener listener;
    TcpPortFactory& factory = TcpPortFactory::getInstance();
    auto cycle = [&]() {
        port_handle_t port = factory.bindPort(listener.url(), PORT_TYPE__TCP | PORT_TYPE__COMM);
        ASSERT_NE(port, nullptr);
        ASSERT_EQ(portOpenRetry(port, 2000, 10), PORT_ERROR__NONE);
        int peer = listener.acceptPeer();
        ASSERT_GE(peer, 0);
        close(peer);
        EXPECT_TRUE(factory.releasePort(port));
    };
    cycle();    // warm up any one-time descriptors (resolver cache, etc.)
    const int baseline = tcp_test::countOpenFds();
    for (int i = 0; i < 50; i++) cycle();
    EXPECT_EQ(tcp_test::countOpenFds(), baseline);
}

#endif
