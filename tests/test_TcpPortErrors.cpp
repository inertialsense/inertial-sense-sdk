/**
 * @file test_TcpPortErrors.cpp
 * @brief A TCP port that hits a fatal socket error must close its descriptor, while a transient one (full send buffer) must not.
 */

#include <gtest/gtest.h>

#if !defined(_WIN32)

#include <vector>

#include "TcpPortFactory.h"
#include "tcp_test_helpers.h"

namespace {

/** A TcpPortFactory port connected to a loopback listener, with the listener's accepted peer socket. */
struct ConnectedPort {
    tcp_test::Listener listener;
    port_handle_t port = nullptr;
    int peer = -1;

    bool connect() {
        port = TcpPortFactory::getInstance().bindPort(listener.url(), PORT_TYPE__TCP | PORT_TYPE__COMM);
        if (!port || portOpenRetry(port, 2000, 10) != PORT_ERROR__NONE) return false;
        peer = listener.acceptPeer();
        return peer >= 0;
    }
    ~ConnectedPort() {
        if (peer >= 0) close(peer);
        if (port) TcpPortFactory::getInstance().releasePort(port);
    }
};

} // namespace

TEST(TcpPortWriteError, ResetPeerClosesSocket)
{
    ConnectedPort c;
    ASSERT_TRUE(c.connect());
    const int sock = TCP_PORT(c.port)->socket;
    ASSERT_GE(sock, 0);

    tcp_test::closeWithReset(c.peer);
    c.peer = -1;

    // The first write after the reset may still be accepted by the kernel; the error surfaces within a few writes.
    const uint8_t data[64] = {};
    int rc = 0;
    for (int i = 0; i < 20 && rc >= 0; i++) {
        rc = portWrite(c.port, data, sizeof(data));
        usleep(10000);
    }
    EXPECT_LT(rc, 0);
    EXPECT_FALSE(tcp_test::fdIsOpen(sock)) << "descriptor " << sock << " leaked after a fatal send error";
    EXPECT_FALSE(portIsValid(c.port));
}

TEST(TcpPortWriteError, SlowPeerStaysUsable)
{
    ConnectedPort c;
    ASSERT_TRUE(c.connect());
    const int sock = TCP_PORT(c.port)->socket;
    int small = 4096;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &small, sizeof(small));
    setsockopt(c.peer, SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));

    // The peer never reads, so the send buffer eventually fills and a non-blocking send reports EAGAIN.
    const std::vector<uint8_t> chunk(8192, 0x5A);
    int rc = 1;
    int guard = 0;
    while (rc > 0 && guard++ < 100000)
        rc = portWrite(c.port, chunk.data(), (unsigned)chunk.size());
    EXPECT_EQ(rc, 0) << "expected the send buffer to fill (0 bytes written), not an error";
    EXPECT_TRUE(tcp_test::fdIsOpen(sock));
    EXPECT_GE(TCP_PORT(c.port)->socket, 0);
    EXPECT_TRUE(portIsValid(c.port));

    // Once the peer drains, the same connection accepts data again.
    std::vector<char> sink(65536);
    int writes = 0;
    for (int i = 0; i < 200 && writes == 0; i++) {
        while (recv(c.peer, sink.data(), sink.size(), MSG_DONTWAIT) > 0) {}
        if (portWrite(c.port, chunk.data(), (unsigned)chunk.size()) > 0) writes++;
        usleep(5000);
    }
    EXPECT_GT(writes, 0) << "connection never became writable again";
}

TEST(TcpPortReadError, ResetPeerClosesSocket)
{
    ConnectedPort c;
    ASSERT_TRUE(c.connect());
    const int sock = TCP_PORT(c.port)->socket;
    ASSERT_GE(sock, 0);

    tcp_test::closeWithReset(c.peer);
    c.peer = -1;
    usleep(50000);      // let the RST arrive

    uint8_t buf[16];
    const int rc = portRead(c.port, buf, sizeof(buf));
    EXPECT_LT(rc, 0);
    EXPECT_FALSE(tcp_test::fdIsOpen(sock)) << "descriptor " << sock << " leaked after a fatal recv error";
}

#endif
