/**
 * @file test_ISHttpRequest.cpp
 * @brief Unit tests for ISHttpRequest HTTP client
 *
 * Uses a simple socket server to simulate HTTP responses.
 * Cross-platform: POSIX sockets on Linux/macOS, Winsock on Windows.
 *
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <string>
#include <thread>
#include <atomic>
#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    typedef SOCKET socket_t;
    #define CLOSE_SOCKET closesocket
    #define SOCKET_INVALID INVALID_SOCKET
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    #include <arpa/inet.h>
    typedef int socket_t;
    #define CLOSE_SOCKET close
    #define SOCKET_INVALID (-1)
#endif

#include <gtest/gtest.h>
#include "gtest_helpers.h"

#include "ISHttpRequest.h"
#include "ISUtilities.h"
#include "json.hpp"

using json = nlohmann::json;

#ifdef _WIN32
// RAII helper to initialize/cleanup Winsock
struct WinsockInit {
    WinsockInit() { WSADATA wsa; WSAStartup(MAKEWORD(2, 2), &wsa); }
    ~WinsockInit() { WSACleanup(); }
};
static WinsockInit s_winsockInit;
#endif

// Simple cross-platform socket HTTP mock server
class MockHttpServer {
public:
    MockHttpServer(int port, const std::string& response)
        : m_port(port), m_response(response), m_listenFd(SOCKET_INVALID), m_running(false) {}

    bool start()
    {
        m_listenFd = socket(AF_INET, SOCK_STREAM, 0);
        if (m_listenFd == SOCKET_INVALID) return false;

        int opt = 1;
        setsockopt(m_listenFd, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(m_port);

        if (bind(m_listenFd, (struct sockaddr*)&addr, sizeof(addr)) != 0)
        {
            CLOSE_SOCKET(m_listenFd);
            m_listenFd = SOCKET_INVALID;
            return false;
        }

        if (listen(m_listenFd, 1) != 0)
        {
            CLOSE_SOCKET(m_listenFd);
            m_listenFd = SOCKET_INVALID;
            return false;
        }

        m_running = true;
        m_thread = std::thread([this]() {
            struct sockaddr_in clientAddr = {};
            socklen_t clientLen = sizeof(clientAddr);
            socket_t clientFd = accept(m_listenFd, (struct sockaddr*)&clientAddr, &clientLen);
            if (clientFd == SOCKET_INVALID) return;

            // Read request (drain until we see end of headers)
            char buf[4096];
            std::string received;
            while (received.find("\r\n\r\n") == std::string::npos)
            {
                int n = recv(clientFd, buf, sizeof(buf), 0);
                if (n <= 0) break;
                received.append(buf, n);
            }

            // Send canned response
            const char* data = m_response.data();
            int remaining = (int)m_response.size();
            while (remaining > 0)
            {
                int sent = send(clientFd, data, remaining, 0);
                if (sent <= 0) break;
                data += sent;
                remaining -= sent;
            }

            // Small delay then close
            SLEEP_MS(50);
            CLOSE_SOCKET(clientFd);
        });

        return true;
    }

    void stop()
    {
        m_running = false;
        if (m_listenFd != SOCKET_INVALID)
        {
            CLOSE_SOCKET(m_listenFd);
            m_listenFd = SOCKET_INVALID;
        }
        if (m_thread.joinable())
            m_thread.join();
    }

    ~MockHttpServer() { stop(); }

private:
    int m_port;
    std::string m_response;
    socket_t m_listenFd;
    std::atomic<bool> m_running;
    std::thread m_thread;
};

static std::string makeHttpResponse(int statusCode, const std::string& statusMsg, const std::string& body)
{
    std::string resp = "HTTP/1.1 " + std::to_string(statusCode) + " " + statusMsg + "\r\n";
    resp += "Content-Type: application/json\r\n";
    resp += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    resp += "Connection: close\r\n";
    resp += "\r\n";
    resp += body;
    return resp;
}

// ---- Tests ----

TEST(test_ISHttpRequest, connectionFailure_refusedPort)
{
    // Connect to localhost on a port nothing is listening on — should fail fast (connection refused)
    auto resp = ISHttpRequest::get("http://127.0.0.1:19999/nonexistent", 5000);
    EXPECT_EQ(resp.statusCode, -1);
}

TEST(test_ISHttpRequest, successfulGet_200)
{
    int port = 18301;
    json body;
    body["message"] = "hello";
    body["count"] = 42;
    std::string httpResp = makeHttpResponse(200, "OK", body.dump());

    MockHttpServer server(port, httpResp);
    ASSERT_TRUE(server.start());
    SLEEP_MS(100);  // Give server time to start listening

    auto resp = ISHttpRequest::get("http://127.0.0.1:" + std::to_string(port) + "/api/test", 10000);

    EXPECT_EQ(resp.statusCode, 200);
    EXPECT_EQ(resp.statusMessage, "OK");
    EXPECT_FALSE(resp.body.empty());

    // Parse the body as JSON and verify
    json parsed = json::parse(resp.body);
    EXPECT_EQ(parsed["message"].get<std::string>(), "hello");
    EXPECT_EQ(parsed["count"].get<int>(), 42);

    server.stop();
}

TEST(test_ISHttpRequest, notFound_404)
{
    int port = 18302;
    std::string httpResp = makeHttpResponse(404, "Not Found", "{\"error\": \"not found\"}");

    MockHttpServer server(port, httpResp);
    ASSERT_TRUE(server.start());
    SLEEP_MS(100);

    auto resp = ISHttpRequest::get("http://127.0.0.1:" + std::to_string(port) + "/api/missing", 10000);

    EXPECT_EQ(resp.statusCode, 404);
    EXPECT_EQ(resp.statusMessage, "Not Found");

    server.stop();
}

TEST(test_ISHttpRequest, serverError_500)
{
    int port = 18303;
    std::string httpResp = makeHttpResponse(500, "Internal Server Error", "{\"error\": \"server error\"}");

    MockHttpServer server(port, httpResp);
    ASSERT_TRUE(server.start());
    SLEEP_MS(100);

    auto resp = ISHttpRequest::get("http://127.0.0.1:" + std::to_string(port) + "/api/broken", 10000);

    EXPECT_EQ(resp.statusCode, 500);

    server.stop();
}

TEST(test_ISHttpRequest, largeBody)
{
    int port = 18304;

    // Generate a larger body (~10KB)
    json body;
    json items = json::array();
    for (int i = 0; i < 100; i++)
    {
        json item;
        item["index"] = i;
        item["value"] = "test_value_" + std::to_string(i);
        item["data"] = std::string(80, 'A' + (i % 26));
        items.push_back(item);
    }
    body["items"] = items;
    std::string bodyStr = body.dump();
    std::string httpResp = makeHttpResponse(200, "OK", bodyStr);

    MockHttpServer server(port, httpResp);
    ASSERT_TRUE(server.start());
    SLEEP_MS(100);

    auto resp = ISHttpRequest::get("http://127.0.0.1:" + std::to_string(port) + "/api/large", 10000);

    EXPECT_EQ(resp.statusCode, 200);
    EXPECT_EQ(resp.body.size(), bodyStr.size());

    // Verify JSON round-trips correctly
    json parsed = json::parse(resp.body);
    EXPECT_EQ(parsed["items"].size(), 100u);
    EXPECT_EQ(parsed["items"][0]["index"].get<int>(), 0);
    EXPECT_EQ(parsed["items"][99]["index"].get<int>(), 99);

    server.stop();
}

TEST(test_ISHttpRequest, headersAreParsed)
{
    int port = 18305;

    std::string httpResp = "HTTP/1.1 200 OK\r\n";
    httpResp += "Content-Type: application/json\r\n";
    httpResp += "X-Custom-Header: test-value\r\n";
    httpResp += "Content-Length: 2\r\n";
    httpResp += "Connection: close\r\n";
    httpResp += "\r\n";
    httpResp += "{}";

    MockHttpServer server(port, httpResp);
    ASSERT_TRUE(server.start());
    SLEEP_MS(100);

    auto resp = ISHttpRequest::get("http://127.0.0.1:" + std::to_string(port) + "/api/headers", 10000);

    EXPECT_EQ(resp.statusCode, 200);
    EXPECT_TRUE(resp.headers.count("Content-Type") > 0);
    EXPECT_EQ(resp.headers["Content-Type"], "application/json");
    EXPECT_TRUE(resp.headers.count("X-Custom-Header") > 0);
    EXPECT_EQ(resp.headers["X-Custom-Header"], "test-value");

    server.stop();
}