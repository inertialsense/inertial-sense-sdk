/**
 * @file test_TcpServerLifecycle.cpp
 * @brief Client lifecycle of the RTCM3 correction server (TcpServerPortFactory + CorrectionService): dropped clients are released,
 *        maxConnections is enforced, destroying a server with clients is clean, and failed clients don't exhaust descriptors.
 */

#include <gtest/gtest.h>

#if !defined(_WIN32)

#include <sys/resource.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// CorrectionService::sendData() is private; the tests drive it directly, the way step() does once the source delivers RTCM3.
#define private public
#include "Rtcm3CorrectionServer.h"
#undef private
#include "tcp_test_helpers.h"

namespace {

/** Rtcm3CorrectionServer on a free loopback port, with test access to the accept loop and the client bookkeeping. */
class TestServer : public Rtcm3CorrectionServer {
public:
    explicit TestServer(int maxConnections = 10) : Rtcm3CorrectionServer(freePort(), "127.0.0.1", maxConnections) {}

    /** Accepts pending connections; each one is bound to a port and added as a downstream port when @p bind is true (as step() does). */
    void acceptClients(bool bind = true) {
        processPendingConnections([&](const socket_entry_t& e) {
            if (bind) addPort(TcpServerPortFactory::bindPort(e.portName, PORT_TYPE__TCP | PORT_TYPE__COMM));
        });
    }
    /** Forwards @p len bytes to every downstream port, as the source's RTCM3 handler does. */
    void send(unsigned len = 64) {
        const std::vector<uint8_t> data(len, 0xD3);
        sendData(data.data(), len);
    }
    int known() { return getClientConnectionCount(); }
    port_handle_t firstPort() { return ports.empty() ? nullptr : ports.front(); }
    int clientPort() const { return factoryOptions.listenerPort; }

private:
    static uint16_t freePort() { tcp_test::Listener probe; return probe.port; }
};

/** Connects @p n clients and accepts them on the server. @return the client-side sockets */
std::vector<int> connectClients(TestServer& server, int n, bool bind = true) {
    std::vector<int> clients;
    for (int i = 0; i < n; i++) {
        int c = -1;
        for (int attempt = 0; attempt < 50 && c < 0; attempt++) {
            c = tcp_test::connectTo(server.clientPort());
            if (c < 0) usleep(10000);
        }
        if (c >= 0) clients.push_back(c);
    }
    // The kernel completes the handshakes before accept(); poll until the server has accepted everything it will.
    for (int attempt = 0; attempt < 100; attempt++) {
        server.acceptClients(bind);
        if (server.known() >= n) break;
        usleep(5000);
    }
    return clients;
}

void closeAll(std::vector<int>& fds) { for (int fd : fds) if (fd >= 0) close(fd); fds.clear(); }

/** Sends on the server until its downstream list reaches @p expected (or gives up). */
bool sendUntilClients(TestServer& server, int expected) {
    for (int i = 0; i < 200; i++) {
        if (server.getActiveClients() == expected) return true;
        server.send();
        usleep(5000);
    }
    return server.getActiveClients() == expected;
}

} // namespace

TEST(CorrectionServiceDeadClient, ResetClientIsRemoved)
{
    TestServer server;
    auto clients = connectClients(server, 3);
    ASSERT_EQ(server.getActiveClients(), 3);

    tcp_test::closeWithReset(clients[0]);
    clients[0] = -1;
    ASSERT_TRUE(sendUntilClients(server, 2)) << "the reset client stayed in the downstream list";
    closeAll(clients);
}

TEST(CorrectionServiceDeadClient, SlowClientIsNotRemoved)
{
    TestServer server;
    auto clients = connectClients(server, 1);
    ASSERT_EQ(server.getActiveClients(), 1);
    int small = 4096;
    setsockopt(TCP_PORT(server.firstPort())->socket, SOL_SOCKET, SO_SNDBUF, &small, sizeof(small));
    setsockopt(clients[0], SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));

    // The client never reads, so the send buffer fills and writes report EAGAIN; the client must stay connected.
    for (int i = 0; i < 5000; i++) server.send(4096);
    EXPECT_EQ(server.getActiveClients(), 1);
    EXPECT_GE(TCP_PORT(server.firstPort())->socket, 0);
    closeAll(clients);
}

TEST(TcpServerLifecycle, RemovedClientIsReleased)
{
    TestServer server;
    auto clients = connectClients(server, 3);
    ASSERT_EQ(server.known(), 3);
    const int serverSideSock = TCP_PORT(server.firstPort())->socket;
    ASSERT_GE(serverSideSock, 0);

    close(clients[0]);      // orderly close (FIN); the first write after it may succeed, a later one fails
    clients[0] = -1;
    ASSERT_TRUE(sendUntilClients(server, 2));
    EXPECT_EQ(server.known(), 2) << "the dropped client's entry was never released";
    EXPECT_FALSE(tcp_test::fdIsOpen(serverSideSock)) << "the dropped client's descriptor leaked";
    closeAll(clients);
}

TEST(TcpServerLifecycle, MaxConnectionsIsEnforced)
{
    TestServer server(2);
    auto clients = connectClients(server, 3);
    ASSERT_EQ(clients.size(), 3u);      // TCP-level connects succeed (kernel backlog); the server refuses the extra one at accept
    EXPECT_EQ(server.known(), 2);
    EXPECT_EQ(server.getActiveClients(), 2);

    int refused = 0;
    for (int fd : clients) if (tcp_test::peerSeesEof(fd, 200)) refused++;
    EXPECT_EQ(refused, 1) << "exactly the connection over the limit should be closed";
    closeAll(clients);
}

TEST(TcpServerLifecycle, DestructorWithClientsDoesNotCrash)
{
    // Run in a child process: before the fix the destructor crashed (iterator invalidation), which would take the whole test binary with it.
    EXPECT_EXIT({
        {
            TestServer server;
            auto bound = connectClients(server, 3);
            auto unbound = connectClients(server, 2, false);    // accepted, but never given a port
            (void)bound; (void)unbound;
        }
        exit(0);
    }, ::testing::ExitedWithCode(0), "");
}

TEST(TcpServerLifecycle, DestructorClosesAllClients)
{
    std::vector<int> clients;
    {
        TestServer server;
        clients = connectClients(server, 2);
        auto unbound = connectClients(server, 1, false);
        clients.insert(clients.end(), unbound.begin(), unbound.end());
        ASSERT_EQ(server.known(), 3);
    }
    for (int fd : clients) EXPECT_TRUE(tcp_test::peerSeesEof(fd)) << "a client connection survived the server";
    closeAll(clients);
}

TEST(TcpServerLifecycle, ChurnDoesNotGrow)
{
    TestServer server;
    { auto warm = connectClients(server, 1); closeAll(warm); sendUntilClients(server, 0); }     // warm up one-time descriptors
    const int baseline = tcp_test::countOpenFds();

    for (int i = 0; i < 300; i++) {
        auto c = connectClients(server, 1);
        ASSERT_EQ(c.size(), 1u);
        closeAll(c);
        ASSERT_TRUE(sendUntilClients(server, 0)) << "cycle " << i;
    }
    EXPECT_EQ(server.known(), 0);
    EXPECT_EQ(tcp_test::countOpenFds(), baseline);
}

TEST(CorrectionServiceDeadClient, ResetClientsDoNotExhaustDescriptors)
{
    TestServer server;
    rlimit saved;
    ASSERT_EQ(getrlimit(RLIMIT_NOFILE, &saved), 0);
    rlimit lowered = saved;
    lowered.rlim_cur = std::min<rlim_t>(saved.rlim_cur, (rlim_t)tcp_test::countOpenFds() + 40);
    ASSERT_EQ(setrlimit(RLIMIT_NOFILE, &lowered), 0);

    // 100 clients connect and reset; each leaked descriptor would count against the lowered limit.
    for (int i = 0; i < 100; i++) {
        auto c = connectClients(server, 1);
        ASSERT_EQ(c.size(), 1u) << "descriptors exhausted after " << i << " reset clients";
        tcp_test::closeWithReset(c[0]);
        ASSERT_TRUE(sendUntilClients(server, 0)) << "cycle " << i;
    }

    // A new client still connects and receives data.
    auto fresh = connectClients(server, 1);
    ASSERT_EQ(fresh.size(), 1u);
    server.send(16);
    pollfd pfd = { fresh[0], POLLIN, 0 };
    EXPECT_EQ(poll(&pfd, 1, 1000), 1);
    closeAll(fresh);
    setrlimit(RLIMIT_NOFILE, &saved);
}

#endif
