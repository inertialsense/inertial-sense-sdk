/**
 * @file test_NtripRequestHeaders.cpp
 * @brief Headers set via setConnectionRequestHeaders() must be sent in the NTRIP mount-point request.
 */

#include <gtest/gtest.h>

#if !defined(_WIN32)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <map>
#include <string>
#include <thread>

#include "NtripCorrectionService.h"

namespace {

// One-shot NTRIP caster: accepts one connection, records the request head and answers "ICY 200 OK".
class RecordingCaster {
public:
    RecordingCaster() {
        listener = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        bind(listener, (sockaddr*)&addr, sizeof(addr));
        listen(listener, 1);
        socklen_t len = sizeof(addr);
        getsockname(listener, (sockaddr*)&addr, &len);
        port = ntohs(addr.sin_port);
        thread = std::thread([this] { run(); });
    }
    ~RecordingCaster() {
        if (thread.joinable()) thread.join();
        if (client >= 0) close(client);
        close(listener);
    }

    std::string url() const { return "ntrip://127.0.0.1:" + std::to_string(port) + "/MOUNT"; }

    // Waits for the request to have been received, then returns it.
    const std::string& request() { if (thread.joinable()) thread.join(); return req; }

private:
    void run() {
        pollfd pfd = { listener, POLLIN, 0 };
        if (poll(&pfd, 1, 3000) != 1) return;
        client = accept(listener, nullptr, nullptr);
        if (client < 0) return;
        char buf[256];
        while (req.find("\r\n\r\n") == std::string::npos) {
            ssize_t n = recv(client, buf, sizeof(buf), 0);
            if (n <= 0) break;
            req.append(buf, (size_t)n);
        }
        const char* resp = "ICY 200 OK\r\n\r\n";
        send(client, resp, strlen(resp), MSG_NOSIGNAL);
    }

    int listener = -1;
    int client = -1;
    uint16_t port = 0;
    std::thread thread;
    std::string req;
};

} // namespace

TEST(NtripRequestHeaders, CustomHeadersAreSent)
{
    RecordingCaster caster;
    NtripCorrectionService ntrip;
    ntrip.setConnectionRequestHeaders({ { "Ntrip-Version", "Ntrip/2.0" }, { "X-Client-Id", "abc123" } });
    ASSERT_TRUE(ntrip.connect(caster.url()));

    const std::string& req = caster.request();
    EXPECT_NE(req.find("\r\nNtrip-Version: Ntrip/2.0\r\n"), std::string::npos) << req;
    EXPECT_NE(req.find("\r\nX-Client-Id: abc123\r\n"), std::string::npos) << req;
    EXPECT_EQ(req.find("\r\n\r\n"), req.size() - 4) << "headers must precede the blank line ending the request";
}

TEST(NtripRequestHeaders, HeadersCannotInjectLines)
{
    RecordingCaster caster;
    NtripCorrectionService ntrip;
    ntrip.setConnectionRequestHeaders({ { "X-Evil", "a\r\nInjected: 1" }, { "Bad:Name", "x" }, { "X-Ok", "1" } });
    ASSERT_TRUE(ntrip.connect(caster.url()));

    const std::string& req = caster.request();
    EXPECT_EQ(req.find("Injected"), std::string::npos) << req;
    EXPECT_EQ(req.find("Bad:Name"), std::string::npos) << req;
    EXPECT_NE(req.find("\r\nX-Ok: 1\r\n"), std::string::npos) << req;
}

#endif
