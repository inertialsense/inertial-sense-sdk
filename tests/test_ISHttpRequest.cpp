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

    const std::string& lastRequest() const { return m_lastRequest; }

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

            // Read request headers
            char buf[4096];
            std::string received;
            while (received.find("\r\n\r\n") == std::string::npos)
            {
                int n = recv(clientFd, buf, sizeof(buf), 0);
                if (n <= 0) break;
                received.append(buf, n);
            }

            // Check for Content-Length and read remaining body if present
            size_t headerEnd = received.find("\r\n\r\n");
            if (headerEnd != std::string::npos)
            {
                std::string headers = received.substr(0, headerEnd);
                size_t bodyStart = headerEnd + 4;
                size_t bodyReceived = received.size() - bodyStart;

                // Parse Content-Length (case-insensitive)
                std::string headersLower = headers;
                std::transform(headersLower.begin(), headersLower.end(), headersLower.begin(), ::tolower);
                size_t clPos = headersLower.find("content-length:");
                if (clPos != std::string::npos)
                {
                    size_t valueStart = headersLower.find(':', clPos) + 1;
                    size_t valueEnd = headersLower.find("\r\n", clPos);
                    std::string clValue = headers.substr(valueStart, valueEnd - valueStart);
                    int contentLength = std::stoi(clValue);

                    // Read remaining body bytes
                    while ((int)bodyReceived < contentLength)
                    {
                        int n = recv(clientFd, buf, sizeof(buf), 0);
                        if (n <= 0) break;
                        received.append(buf, n);
                        bodyReceived += n;
                    }
                }
            }

            m_lastRequest = received;

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
    std::string m_lastRequest;
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

TEST(test_ISHttpRequest, post_200)
{
    int port = 18306;
    json respBody;
    respBody["status"] = "created";
    respBody["id"] = 123;
    std::string httpResp = makeHttpResponse(200, "OK", respBody.dump());

    MockHttpServer server(port, httpResp);
    ASSERT_TRUE(server.start());
    SLEEP_MS(100);

    json reqBody;
    reqBody["name"] = "test-device";
    reqBody["serial"] = 12345;

    auto resp = ISHttpRequest::post("http://127.0.0.1:" + std::to_string(port) + "/api/device", reqBody.dump(), 10000);

    EXPECT_EQ(resp.statusCode, 200);
    EXPECT_FALSE(resp.body.empty());

    json parsed = json::parse(resp.body);
    EXPECT_EQ(parsed["status"].get<std::string>(), "created");
    EXPECT_EQ(parsed["id"].get<int>(), 123);

    // Verify the request was a POST with JSON body
    const std::string& req = server.lastRequest();
    EXPECT_TRUE(req.find("POST /api/device") != std::string::npos);
    EXPECT_TRUE(req.find("Content-Type: application/json") != std::string::npos);
    EXPECT_TRUE(req.find("\"name\":\"test-device\"") != std::string::npos);

    server.stop();
}

TEST(test_ISHttpRequest, put_200)
{
    int port = 18307;
    json respBody;
    respBody["status"] = "updated";
    std::string httpResp = makeHttpResponse(200, "OK", respBody.dump());

    MockHttpServer server(port, httpResp);
    ASSERT_TRUE(server.start());
    SLEEP_MS(100);

    json reqBody;
    reqBody["hw_type"] = "IMX-5.0";
    reqBody["dev_serial_num"] = 62797;

    auto resp = ISHttpRequest::put("http://127.0.0.1:" + std::to_string(port) + "/api/device", reqBody.dump(), 10000);

    EXPECT_EQ(resp.statusCode, 200);

    json parsed = json::parse(resp.body);
    EXPECT_EQ(parsed["status"].get<std::string>(), "updated");

    // Verify the request was a PUT with JSON body
    const std::string& req = server.lastRequest();
    EXPECT_TRUE(req.find("PUT /api/device") != std::string::npos);
    EXPECT_TRUE(req.find("Content-Type: application/json") != std::string::npos);
    EXPECT_TRUE(req.find("\"hw_type\":\"IMX-5.0\"") != std::string::npos);

    server.stop();
}

TEST(test_ISHttpRequest, putMultipart_200)
{
    int port = 18308;
    json respBody;
    respBody["status"] = "uploaded";
    std::string httpResp = makeHttpResponse(200, "OK", respBody.dump());

    MockHttpServer server(port, httpResp);
    ASSERT_TRUE(server.start());
    SLEEP_MS(100);

    json calData;
    calData["info"]["calUuid"] = "test-uuid";
    calData["info"]["hwType"] = "IMX-5.0";

    std::vector<ISHttpRequest::MultipartField> fields;
    fields.push_back({"data", "application/json", calData.dump()});

    auto resp = ISHttpRequest::putMultipart("http://127.0.0.1:" + std::to_string(port) + "/api/calibration", fields, 10000);

    EXPECT_EQ(resp.statusCode, 200);

    json parsed = json::parse(resp.body);
    EXPECT_EQ(parsed["status"].get<std::string>(), "uploaded");

    // Verify multipart structure in request
    const std::string& req = server.lastRequest();
    EXPECT_TRUE(req.find("PUT /api/calibration") != std::string::npos);
    EXPECT_TRUE(req.find("Content-Type: multipart/form-data; boundary=") != std::string::npos);
    EXPECT_TRUE(req.find("Content-Disposition: form-data; name=\"data\"") != std::string::npos);
    EXPECT_TRUE(req.find("Content-Type: application/json") != std::string::npos);
    EXPECT_TRUE(req.find("\"calUuid\":\"test-uuid\"") != std::string::npos);

    server.stop();
}

// ---- UID Encoding Tests ----

#include "util/util.h"

TEST(test_encodeSTM32UID, knownExample_SN522807)
{
    // Known example from WEB.md: SN522807 → "20313933-534B-8EF4-996B-5016002b0016"
    // We need to reverse-engineer the uid values from the expected output.
    // Field 1 (BE): 20 31 39 33 → LE bytes: 33 39 31 20 → uid[0] = 0x20313933
    // Field 2 (BE): 53 4B → LE bytes: 4B 53 → uid[1] low 2 bytes = 0x534B (as uint16)
    // Remaining from uid[1]: bytes[6],bytes[7] = 50 16 → uid[1] = 0x1650534B
    // uid[2]: bytes[8..11] = 00 2b 00 16 → uid[2] = 0x16002b00
    //
    // uid[0] as LE: bytes 0x33, 0x39, 0x31, 0x20 → uid[0] = 0x20313933
    // uid[1] as LE: bytes 0x4B, 0x53, 0x50, 0x16 → uid[1] = 0x1650534B
    // uid[2] as LE: bytes 0x00, 0x2b, 0x00, 0x16 → uid[2] = 0x16002b00

    uint32_t uid[3] = { 0x20313933, 0x1650534B, 0x16002b00 };
    std::string result = utils::encodeSTM32UID(uid);
    EXPECT_EQ(result, "20313933-534b-8EF4-996B-5016002b0016");
}

TEST(test_encodeSTM32UID, zeroUid)
{
    uint32_t uid[3] = { 0, 0, 0 };
    std::string result = utils::encodeSTM32UID(uid);
    EXPECT_EQ(result, "00000000-0000-8EF4-996B-000000000000");
}

TEST(test_generateUUIDv4, format)
{
    std::string uuid = utils::generateUUIDv4();
    // UUID v4 format: 8-4-4-4-12 hex chars with dashes
    EXPECT_EQ(uuid.length(), 36u);
    EXPECT_EQ(uuid[8], '-');
    EXPECT_EQ(uuid[13], '-');
    EXPECT_EQ(uuid[18], '-');
    EXPECT_EQ(uuid[23], '-');
    // Version nibble should be '4'
    EXPECT_EQ(uuid[14], '4');
    // Variant nibble should be 8, 9, a, or b
    char variant = uuid[19];
    EXPECT_TRUE(variant == '8' || variant == '9' || variant == 'a' || variant == 'b');
}

TEST(test_generateUUIDv4, uniqueness)
{
    std::string uuid1 = utils::generateUUIDv4();
    std::string uuid2 = utils::generateUUIDv4();
    EXPECT_NE(uuid1, uuid2);
}