/**
 * @file test_NtripCredentialsDecode.cpp
 * @brief URI userinfo is percent-decoded, so NTRIP Basic auth carries the real credentials.
 */

#include <gtest/gtest.h>

#include "util/util.h"

using utils::parseUri;
using utils::UriParts;

TEST(UriUserinfoDecode, CredentialsArePercentDecoded)
{
    // user "me@corp.com", password "p@ss:w/rd 100%"
    const UriParts u = parseUri("ntrip://me%40corp.com:p%40ss%3Aw%2Frd%20100%25@caster.example.com:2101/MOUNT");
    EXPECT_EQ(u.user, "me@corp.com");
    EXPECT_EQ(u.password, "p@ss:w/rd 100%");
    EXPECT_EQ(u.host, "caster.example.com");
    EXPECT_EQ(u.port, 2101);
    EXPECT_EQ(u.path, "/MOUNT");
}

TEST(UriUserinfoDecode, MalformedSequencesAndPlusAreKept)
{
    const UriParts u = parseUri("ntrip://a+b:50%zz%4@caster.example.com:2101/MOUNT");
    EXPECT_EQ(u.user, "a+b");           // '+' is not a space in URIs
    EXPECT_EQ(u.password, "50%zz%4");   // invalid / truncated escapes stay literal
}

#if !defined(_WIN32)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <string>
#include <thread>

#include "NtripCorrectionService.h"

namespace {

// One-shot NTRIP caster: records the request head and answers "ICY 200 OK".
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

    uint16_t port = 0;

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
    std::thread thread;
    std::string req;
};

} // namespace

TEST(NtripCredentialsDecode, BasicAuthUsesDecodedCredentials)
{
    RecordingCaster caster;
    NtripCorrectionService ntrip;
    const std::string url = "ntrip://me%40corp.com:p%40ss%3Aw%2Frd%20100%25@127.0.0.1:" + std::to_string(caster.port) + "/MOUNT";
    ASSERT_TRUE(ntrip.connect(url));

    // base64("me@corp.com:p@ss:w/rd 100%")
    const std::string& req = caster.request();
    EXPECT_NE(req.find("\r\nAuthorization: Basic bWVAY29ycC5jb206cEBzczp3L3JkIDEwMCU=\r\n"), std::string::npos) << req;
}

#endif
