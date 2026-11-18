/**
 * @file test_TcpPortReconnect.cpp
 * @brief Reconnection behaviour of tcp_port_t, and of NtripCorrectionService on top of it.
 *
 * The bug these cover: a fatal read/open error (RST from the peer, a refused connect) left the port
 * reporting portIsOpened() == true after its descriptor had been closed. Every reconnect path in the SDK is
 * gated on portIsOpened(), so such a port was never reopened and a correction stream interrupted by a brief
 * network outage stayed dead until the process restarted. A fatal write error (EPIPE on a GGA update)
 * invalidates the port by design, so NtripCorrectionService has to replace the port rather than reopen it.
 *
 * Each test drives a real loopback socket, because the failure modes are kernel-reported conditions
 * (ECONNRESET, EPIPE) that cannot be reproduced through the port API itself.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>

#if !defined(_WIN32)

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "NtripCorrectionService.h"
#include "TcpPortFactory.h"
#include "core/tcpPort.h"

namespace {

/** Upper bound on how long a loopback connect/accept/reconnect is allowed to take in these tests. */
constexpr int LOOPBACK_TIMEOUT_MS = 5000;

/**
 * A minimal TCP server on 127.0.0.1 used as the peer for the port under test.
 *
 * Accepts one connection at a time and exposes the two disconnect modes that matter: an orderly FIN
 * (which a later write turns into EPIPE) and an abortive RST (which a read turns into ECONNRESET).
 */
class LoopbackServer {
public:
    LoopbackServer() {
        // Non-blocking listener: acceptClient() is polled, and with a zero timeout it is also used to
        // ask "is a connection pending right now?" from inside a step() loop.
        listenFd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
        EXPECT_GE(listenFd, 0);

        int reuse = 1;
        ::setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;      // let the kernel pick a free port
        EXPECT_EQ(0, ::bind(listenFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)));
        EXPECT_EQ(0, ::listen(listenFd, 4));

        socklen_t len = sizeof(addr);
        EXPECT_EQ(0, ::getsockname(listenFd, reinterpret_cast<sockaddr*>(&addr), &len));
        boundPort = ntohs(addr.sin_port);
    }

    ~LoopbackServer() {
        dropClient();
        if (listenFd >= 0)
            ::close(listenFd);
    }

    /** @return the "tcp://127.0.0.1:<port>" URL of this server */
    std::string tcpUrl() const { return "tcp://127.0.0.1:" + std::to_string(boundPort); }

    /** @return the port number the kernel assigned to this server */
    uint16_t port() const { return boundPort; }

    /**
     * Accepts the next pending connection, replacing any previously accepted one.
     * @param timeoutMs how long to wait for a connection to arrive
     * @return true if a connection was accepted
     */
    bool acceptClient(int timeoutMs = LOOPBACK_TIMEOUT_MS) {
        dropClient();
        for (int waited = 0; waited <= timeoutMs; waited += 10) {
            clientFd = ::accept4(listenFd, nullptr, nullptr, SOCK_NONBLOCK);
            if (clientFd >= 0)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    /** Closes the accepted connection with a FIN, the orderly shutdown case. */
    void closeClient() {
        if (clientFd >= 0) {
            ::close(clientFd);
            clientFd = -1;
        }
    }

    /**
     * Closes the accepted connection with an RST, the abortive case. SO_LINGER with a zero timeout is
     * what makes close() emit a reset rather than a FIN, which is how the peer's next read gets
     * ECONNRESET instead of a clean end-of-stream.
     */
    void resetClient() {
        if (clientFd >= 0) {
            linger lin = {1, 0};
            ::setsockopt(clientFd, SOL_SOCKET, SO_LINGER, &lin, sizeof(lin));
            ::close(clientFd);
            clientFd = -1;
        }
    }

    /**
     * Writes to the accepted connection.
     * @param data the bytes to send
     */
    void send(const std::string& data) {
        if (clientFd >= 0)
            ASSERT_EQ((ssize_t)data.size(), ::send(clientFd, data.data(), data.size(), MSG_NOSIGNAL));
    }

    /**
     * Reads whatever the client has sent so far, waiting for the first byte to arrive.
     * @param timeoutMs how long to wait for data
     * @return the bytes received, or an empty string on timeout
     */
    std::string receive(int timeoutMs = LOOPBACK_TIMEOUT_MS) {
        std::string out;
        char buf[1024];
        for (int waited = 0; waited <= timeoutMs; waited += 10) {
            const ssize_t n = ::recv(clientFd, buf, sizeof(buf), 0);
            if (n > 0) {
                out.append(buf, (size_t)n);
                if (out.find("\r\n\r\n") != std::string::npos)
                    break;
            }
            if (!out.empty() && n < 0)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return out;
    }

private:
    void dropClient() { closeClient(); }

    int listenFd = -1;
    int clientFd = -1;
    uint16_t boundPort = 0;
};

/**
 * Binds a port to the server and drives portOpen() until the non-blocking connect completes.
 * @param server the server to connect to
 * @return the opened port, or nullptr if it never opened
 */
port_handle_t openPortTo(const LoopbackServer& server) {
    port_handle_t port = TcpPortFactory::getInstance().bindPort(server.tcpUrl(), PORT_TYPE__TCP | PORT_TYPE__COMM);
    EXPECT_NE(nullptr, port);
    if (!port)
        return nullptr;
    EXPECT_EQ(PORT_ERROR__NONE, portOpenRetry(port, LOOPBACK_TIMEOUT_MS, 2));
    return port;
}

/**
 * Polls portOpen() until the port reports open or the timeout expires, mirroring how the correction
 * services drive a non-blocking reconnect across successive step() calls.
 * @param port the port to reopen
 * @param timeoutMs how long to keep trying
 * @return true if the port opened
 */
bool reopen(port_handle_t port, int timeoutMs = LOOPBACK_TIMEOUT_MS) {
    for (int waited = 0; waited <= timeoutMs; waited += 10) {
        portOpen(port);
        if (portIsOpened(port))
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

}   // namespace

// ---------------------------------------------------------------------------------------------
// Fatal read errors
// ---------------------------------------------------------------------------------------------

TEST(TcpPortReconnect, ResetByPeerClosesPortAndAllowsReopen) {
    LoopbackServer server;
    port_handle_t port = openPortTo(server);
    ASSERT_NE(nullptr, port);
    ASSERT_TRUE(server.acceptClient());
    ASSERT_TRUE(portIsOpened(port));

    server.resetClient();

    // Drive reads until the reset surfaces. A reset is reported to the first read that touches the
    // socket after the RST is processed, which may be one poll later than the close() itself.
    uint8_t buf[64];
    for (int waited = 0; portIsOpened(port) && (waited <= LOOPBACK_TIMEOUT_MS); waited += 10) {
        portRead(port, buf, sizeof(buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // The core of the fix: a connection that died is no longer reported as open...
    EXPECT_FALSE(portIsOpened(port));
    // ...the descriptor was closed rather than overwritten with -errno and leaked...
    EXPECT_EQ(-EBADF, TCP_PORT(port)->socket);
    // ...and the port itself stays usable, so it can be reopened.
    EXPECT_TRUE(portIsValid(port));

    EXPECT_TRUE(reopen(port));
    EXPECT_TRUE(server.acceptClient());

    portClose(port);
    TcpPortFactory::getInstance().releasePort(port);
}

TEST(TcpPortReconnect, OrderlyCloseByPeerClosesPort) {
    LoopbackServer server;
    port_handle_t port = openPortTo(server);
    ASSERT_NE(nullptr, port);
    ASSERT_TRUE(server.acceptClient());

    server.closeClient();

    uint8_t buf[64];
    for (int waited = 0; portIsOpened(port) && (waited <= LOOPBACK_TIMEOUT_MS); waited += 10) {
        portRead(port, buf, sizeof(buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_FALSE(portIsOpened(port));
    EXPECT_TRUE(portIsValid(port));
    EXPECT_TRUE(reopen(port));
    EXPECT_TRUE(server.acceptClient());

    portClose(port);
    TcpPortFactory::getInstance().releasePort(port);
}

// ---------------------------------------------------------------------------------------------
// Silent link loss detection
// ---------------------------------------------------------------------------------------------

TEST(TcpPortReconnect, KeepAliveIsConfiguredOnOpen) {
    LoopbackServer server;
    port_handle_t port = openPortTo(server);
    ASSERT_NE(nullptr, port);
    ASSERT_TRUE(server.acceptClient());

    const int fd = TCP_PORT(port)->socket;
    ASSERT_GE(fd, 0);

    int value = 0;
    socklen_t len = sizeof(value);
    ASSERT_EQ(0, ::getsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &value, &len));
    EXPECT_NE(0, value);

    len = sizeof(value);
    ASSERT_EQ(0, ::getsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &value, &len));
    EXPECT_EQ(10, value);

    len = sizeof(value);
    ASSERT_EQ(0, ::getsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &value, &len));
    EXPECT_EQ(3, value);

    // Must be read back at IPPROTO_TCP; at SOL_SOCKET this option number means SO_RCVLOWAT, which is
    // exactly the mistake this asserts against.
    len = sizeof(value);
    ASSERT_EQ(0, ::getsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &value, &len));
    EXPECT_EQ(30000, value);

    portClose(port);
    TcpPortFactory::getInstance().releasePort(port);
}

// ---------------------------------------------------------------------------------------------
// End to end: the NTRIP client recovers a dropped caster connection
// ---------------------------------------------------------------------------------------------

namespace {

/** The Ntrip 1.0 success response, after which a caster streams RTCM3. */
constexpr const char* NTRIP_OK_RESPONSE = "ICY 200 OK\r\n\r\n";

/**
 * Accepts a client, consumes its mount-point request and answers with a success response.
 * @param server the caster stand-in
 * @return the request the client sent
 */
std::string serveMountPoint(LoopbackServer& server) {
    EXPECT_TRUE(server.acceptClient());
    const std::string request = server.receive();
    server.send(NTRIP_OK_RESPONSE);
    return request;
}

/**
 * Drives step() until the caster stand-in has served a (re)connection.
 *
 * The caster has to run on another thread: the mount-point request is only sent from inside step(), and
 * step() blocks on the caster's reply, so a single-threaded test would deadlock with each side waiting
 * for the other.
 * @param ntrip the service to drive
 * @param caster the pending result of a serveMountPoint() call
 * @return the mount-point request the service sent, empty if it never reconnected
 */
std::string stepUntilServed(NtripCorrectionService& ntrip, std::future<std::string>& caster) {
    for (int waited = 0; waited <= LOOPBACK_TIMEOUT_MS; waited += 20) {
        ntrip.step();
        if (caster.wait_for(std::chrono::milliseconds(20)) == std::future_status::ready)
            return caster.get();
    }
    return "";
}

}   // namespace

TEST(NtripReconnect, ReconnectsAndRenegotiatesAfterCasterReset) {
    LoopbackServer server;
    NtripCorrectionService ntrip;

    std::future<std::string> caster = std::async(std::launch::async, serveMountPoint, std::ref(server));
    ASSERT_TRUE(ntrip.connect("ntrip://user:pass@127.0.0.1:" + std::to_string(server.port()) + "/MOUNT"));
    ASSERT_NE(std::string::npos, caster.get().find("GET /MOUNT"));
    ASSERT_TRUE(ntrip.isConnected());

    server.resetClient();

    // step() must notice the dead socket, reopen it, and re-send the mount-point request: an NTRIP caster
    // streams nothing until it has been asked, so merely reopening the socket would leave the service
    // connected but permanently silent.
    std::future<std::string> recaster = std::async(std::launch::async, serveMountPoint, std::ref(server));
    const std::string secondRequest = stepUntilServed(ntrip, recaster);

    EXPECT_NE(std::string::npos, secondRequest.find("GET /MOUNT"));
    EXPECT_NE(std::string::npos, secondRequest.find("Authorization: Basic"));
    EXPECT_TRUE(ntrip.isConnected());
    EXPECT_EQ(NtripCorrectionService::NTRIP_RESULT_OK, ntrip.getLastResult());
}

TEST(NtripReconnect, ReconnectsAfterWatchdogClosesStaleStream) {
    LoopbackServer server;
    NtripCorrectionService ntrip;

    std::future<std::string> caster = std::async(std::launch::async, serveMountPoint, std::ref(server));
    ASSERT_TRUE(ntrip.connect("ntrip://user:pass@127.0.0.1:" + std::to_string(server.port()) + "/MOUNT"));
    ASSERT_NE(std::string::npos, caster.get().find("GET /MOUNT"));
    ASSERT_TRUE(ntrip.isConnected());

    // What a stalled-stream watchdog does: a caster that keeps the TCP connection up but stops sending
    // corrections is only recoverable by closing the port, so that step() rebuilds it.
    portClose(ntrip.getSourcePort());
    ASSERT_FALSE(ntrip.isConnected());

    std::future<std::string> recaster = std::async(std::launch::async, serveMountPoint, std::ref(server));
    EXPECT_NE(std::string::npos, stepUntilServed(ntrip, recaster).find("GET /MOUNT"));
    EXPECT_TRUE(ntrip.isConnected());
    EXPECT_EQ(NtripCorrectionService::NTRIP_RESULT_OK, ntrip.getLastResult());
}

TEST(NtripReconnect, ReconnectsAfterGgaWriteToDeadCaster) {
    LoopbackServer server;
    NtripCorrectionService ntrip;

    std::future<std::string> caster = std::async(std::launch::async, serveMountPoint, std::ref(server));
    ASSERT_TRUE(ntrip.connect("ntrip://user:pass@127.0.0.1:" + std::to_string(server.port()) + "/MOUNT"));
    ASSERT_NE(std::string::npos, caster.get().find("GET /MOUNT"));

    server.resetClient();

    // The first write after a reset is often accepted by the local send buffer; the error is reported to a
    // subsequent one, once the kernel has seen the RST. A fatal write invalidates the port for good.
    const std::string gga = "$GPGGA,000000.00,0000.0,N,00000.0,E,1,00,1.0,0.0,M,0.0,M,,*00\r\n";
    for (int waited = 0; portIsValid(ntrip.getSourcePort()) && (waited <= LOOPBACK_TIMEOUT_MS); waited += 10) {
        ntrip.updatePosition(gga);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_FALSE(portIsValid(ntrip.getSourcePort()));

    std::future<std::string> recaster = std::async(std::launch::async, serveMountPoint, std::ref(server));
    EXPECT_NE(std::string::npos, stepUntilServed(ntrip, recaster).find("GET /MOUNT"));
    EXPECT_TRUE(ntrip.isConnected());
}

#endif  // !defined(_WIN32)
