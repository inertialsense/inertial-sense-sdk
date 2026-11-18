/**
 * @file ntrip_test_helpers.h
 * @brief A fake NTRIP caster for end-to-end correction-stream tests (SN-9070).
 *
 * Adapted from rtk_reconnect_demo by Petr Belohlavek (petrbel), https://gist.github.com/petrbel/912c104a3907fcd08474e3c9ced20b42,
 * which accompanied the TCP/NTRIP recovery fix in PR #1388.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_NTRIP_TEST_HELPERS_H
#define IS_SDK_NTRIP_TEST_HELPERS_H

#if !defined(_WIN32)

#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "ISComm.h"

namespace ntrip_test {

/** A network fault the caster can inflict on the connection it is serving. */
enum class Fault {
    NONE,       //!< Behave.
    RESET,      //!< close() with SO_LINGER{1,0}: the client sees an RST and ECONNRESET.
    FIN,        //!< Ordinary close(): the client sees an orderly shutdown.
    STALL,      //!< Keep the socket open but stop sending: no error is ever raised.
};

/**
 * Builds one syntactically valid RTCM3 frame: 0xD3, 6 reserved bits, a 10-bit payload length, the payload, then a CRC-24Q over everything before it.
 * The SDK's parser validates exactly this framing and CRC, so the payload past the 12-bit message number does not matter to whether the frame is counted.
 * @param messageNumber the RTCM3 message number
 * @param payloadLength the payload length in bytes, at least 2
 * @return the complete frame
 */
inline std::vector<uint8_t> buildRtcm3Frame(uint16_t messageNumber, uint16_t payloadLength) {
    std::vector<uint8_t> frame;
    frame.reserve(payloadLength + 6);
    frame.push_back(0xD3);
    frame.push_back(static_cast<uint8_t>((payloadLength >> 8) & 0x03));
    frame.push_back(static_cast<uint8_t>(payloadLength & 0xFF));
    frame.push_back(static_cast<uint8_t>(messageNumber >> 4));
    frame.push_back(static_cast<uint8_t>((messageNumber & 0x0F) << 4));
    for (uint16_t i = 2; i < payloadLength; ++i)
        frame.push_back(static_cast<uint8_t>(i));

    const unsigned int crc = calculate24BitCRCQ(frame.data(), static_cast<unsigned int>(frame.size()));
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    return frame;
}

/** @return a fixed NMEA GGA sentence with a correct XOR checksum. Real VRS mount points stop streaming unless the rover keeps reporting its position. */
inline std::string buildGgaSentence() {
    const std::string body = "GPGGA,120000.00,5005.0000,N,01425.0000,E,1,12,0.80,200.0,M,45.0,M,,";
    uint8_t checksum = 0;
    for (char c : body)
        checksum ^= static_cast<uint8_t>(c);

    char out[160];
    snprintf(out, sizeof(out), "$%s*%02X\r\n", body.c_str(), checksum);
    return out;
}

/**
 * A single-client NTRIP 1.0 caster on the loopback interface. Accepts a client, answers its mount-point request with "ICY 200 OK", then streams RTCM3
 * until told to misbehave. After a fault it goes back to accepting, so a client that recovers correctly is served again.
 *
 * It runs on its own thread because the mount-point request is only written from inside NtripCorrectionService::connect()/step(), which then blocks on
 * the reply: driving both sides from one thread deadlocks.
 */
class FakeCaster {
public:
    /** Binds a listener on an ephemeral loopback port. The caster does not serve anyone until start(). */
    FakeCaster() {
        listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int reuse = 1;
        ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        bound_ = ::bind(listenFd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0 && ::listen(listenFd_, 4) == 0;

        socklen_t len = sizeof(addr);
        if (::getsockname(listenFd_, reinterpret_cast<sockaddr*>(&addr), &len) == 0)
            port_ = ntohs(addr.sin_port);
    }

    ~FakeCaster() {
        stop();
        if (listenFd_ >= 0)
            ::close(listenFd_);
    }

    FakeCaster(const FakeCaster&) = delete;
    FakeCaster& operator=(const FakeCaster&) = delete;

    /** @return true if the listener was bound successfully */
    bool isBound() const { return bound_; }

    /** @return the "ntrip://" URL of this caster, for the mount point "MOUNT" */
    std::string ntripUrl() const { return "ntrip://user:pass@127.0.0.1:" + std::to_string(port_) + "/MOUNT"; }

    /** @return the number of mount-point requests served, which is the number of connections the client has made */
    int handshakes() const { return handshakes_.load(); }

    /** Starts serving on a background thread. */
    void start() { thread_ = std::thread([this] { run(); }); }

    /** Stops the background thread and drops the client. Also called by the destructor. */
    void stop() {
        stop_ = true;
        if (thread_.joinable())
            thread_.join();
        closeClientFd(false);
    }

    /** @param fault the fault to inflict on the current connection, applied on the caster's next pass */
    void requestFault(Fault fault) { faultRequest_ = fault; }

private:
    void run() {
        auto lastFrame = std::chrono::steady_clock::now();
        while (!stop_) {
            // A reconnecting client shows up as a new connection and a real caster abandons the old one. Checking for an inbound connection on every pass
            // matters for the stall scenario, where the previous connection is still open: a caster that only accepted when idle would never serve it.
            if (clientFd_ < 0 || connectionPending()) {
                closeClientFd(false);
                if (!acceptClient(100))
                    continue;
                if (!serveHandshake()) {
                    closeClientFd(false);
                    continue;
                }
                ++handshakes_;
                stalled_ = false;
            }

            const Fault fault = faultRequest_.exchange(Fault::NONE);
            if (fault == Fault::RESET || fault == Fault::FIN) {
                closeClientFd(fault == Fault::RESET);
                continue;
            }
            if (fault == Fault::STALL)
                stalled_ = true;

            drainClient();

            if (!stalled_ && std::chrono::steady_clock::now() - lastFrame >= FRAME_INTERVAL) {
                sendFrame();
                lastFrame = std::chrono::steady_clock::now();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    bool connectionPending() {
        pollfd pfd = {listenFd_, POLLIN, 0};
        return ::poll(&pfd, 1, 0) > 0;
    }

    bool acceptClient(int timeoutMs) {
        pollfd pfd = {listenFd_, POLLIN, 0};
        if (::poll(&pfd, 1, timeoutMs) <= 0)
            return false;
        clientFd_ = ::accept(listenFd_, nullptr, nullptr);
        return clientFd_ >= 0;
    }

    /** Reads the mount-point request up to the blank line, then answers it. */
    bool serveHandshake() {
        std::string request;
        for (int waited = 0; waited < HANDSHAKE_TIMEOUT_MS; waited += 20) {
            char buf[1024];
            const ssize_t n = ::recv(clientFd_, buf, sizeof(buf), MSG_DONTWAIT);
            if (n > 0)
                request.append(buf, static_cast<size_t>(n));
            if (request.find("\r\n\r\n") != std::string::npos)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (request.find("\r\n\r\n") == std::string::npos)
            return false;

        static const char OK_RESPONSE[] = "ICY 200 OK\r\n\r\n";
        return ::send(clientFd_, OK_RESPONSE, sizeof(OK_RESPONSE) - 1, MSG_NOSIGNAL) > 0;
    }

    /** Consumes whatever the client sends (GGA updates) so its writes keep succeeding and the behaviour under test is reconnection, not a full socket buffer. */
    void drainClient() {
        char buf[1024];
        while (::recv(clientFd_, buf, sizeof(buf), MSG_DONTWAIT) > 0) {
        }
    }

    void sendFrame() {
        const std::vector<uint8_t> frame = buildRtcm3Frame(1005, 19);
        ::send(clientFd_, frame.data(), frame.size(), MSG_NOSIGNAL);
    }

    /**
     * @param withReset send an RST instead of a FIN. SO_LINGER with a zero timeout makes close() discard queued data and reset the connection, which is what
     *        a carrier-grade NAT or a restarted caster looks like from the client.
     */
    void closeClientFd(bool withReset) {
        if (clientFd_ < 0)
            return;
        if (withReset) {
            linger lin = {1, 0};
            ::setsockopt(clientFd_, SOL_SOCKET, SO_LINGER, &lin, sizeof(lin));
        }
        ::close(clientFd_);
        clientFd_ = -1;
    }

    static constexpr int HANDSHAKE_TIMEOUT_MS = 2000;
    static constexpr std::chrono::milliseconds FRAME_INTERVAL{100};

    int listenFd_ = -1;
    int clientFd_ = -1;
    uint16_t port_ = 0;
    bool bound_ = false;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> stalled_{false};
    std::atomic<Fault> faultRequest_{Fault::NONE};
    std::atomic<int> handshakes_{0};
};

}   // namespace ntrip_test

#endif  // !_WIN32
#endif  // IS_SDK_NTRIP_TEST_HELPERS_H
