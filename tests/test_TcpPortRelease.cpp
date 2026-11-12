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

#endif
