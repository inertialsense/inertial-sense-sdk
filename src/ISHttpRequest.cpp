/**
 * @file ISHttpRequest.cpp
 * @brief Lightweight HTTP client using tcpPort infrastructure
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "ISHttpRequest.h"
#include "ISUtilities.h"
#include "uri.hpp"
#include "TcpPortFactory.h"
#include "core/msg_logger.h"

#include <sstream>
#include <algorithm>
#include <cstring>

std::string ISHttpRequest::buildGetRequest(const std::string& host, const std::string& path)
{
    return buildRequest("GET", host, path, "", "");
}

std::string ISHttpRequest::buildRequest(const std::string& method, const std::string& host, const std::string& path,
                                        const std::string& contentType, const std::string& body)
{
    std::string req = method + " " + path + " HTTP/1.1\r\n";
    req += "Host: " + host + "\r\n";
    req += "Accept: */*\r\n";
    if (!contentType.empty())
        req += "Content-Type: " + contentType + "\r\n";
    if (!body.empty())
        req += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    req += "Connection: close\r\n";
    req += "\r\n";
    req += body;
    return req;
}

ISHttpRequest::Response ISHttpRequest::parseResponse(port_handle_t port, int timeoutMs)
{
    Response resp;
    unsigned char lineBuf[4096];

    // Read status line
    int bytesRead = portReadLineTimeout(port, lineBuf, sizeof(lineBuf), timeoutMs);
    if (bytesRead <= 0)
    {
        log_error(IS_LOG_HTTP_REQUEST, "Timeout reading HTTP status line");
        return resp;
    }

    std::string statusLine(reinterpret_cast<char*>(lineBuf), bytesRead);
    // Parse "HTTP/1.x <code> <message>"
    size_t firstSpace = statusLine.find(' ');
    if (firstSpace == std::string::npos)
    {
        log_error(IS_LOG_HTTP_REQUEST, "Malformed HTTP status line");
        return resp;
    }
    size_t secondSpace = statusLine.find(' ', firstSpace + 1);
    std::string codeStr = statusLine.substr(firstSpace + 1, secondSpace - firstSpace - 1);
    resp.statusCode = std::stoi(codeStr);
    if (secondSpace != std::string::npos)
    {
        resp.statusMessage = statusLine.substr(secondSpace + 1);
        // Trim trailing whitespace/CR/LF
        while (!resp.statusMessage.empty() && (resp.statusMessage.back() == '\r' || resp.statusMessage.back() == '\n' || resp.statusMessage.back() == ' '))
            resp.statusMessage.pop_back();
    }

    // Read headers
    int contentLength = -1;
    bool chunked = false;
    while (true)
    {
        bytesRead = portReadLineTimeout(port, lineBuf, sizeof(lineBuf), timeoutMs);
        if (bytesRead <= 0)
            break;

        std::string line(reinterpret_cast<char*>(lineBuf), bytesRead);
        // Trim trailing CR/LF
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
            line.pop_back();

        if (line.empty())
            break;  // End of headers

        size_t colonPos = line.find(':');
        if (colonPos != std::string::npos)
        {
            std::string key = line.substr(0, colonPos);
            std::string value = line.substr(colonPos + 1);
            // Trim leading whitespace from value
            size_t start = value.find_first_not_of(' ');
            if (start != std::string::npos)
                value = value.substr(start);

            resp.headers[key] = value;

            // Case-insensitive header checks
            std::string keyLower = key;
            std::transform(keyLower.begin(), keyLower.end(), keyLower.begin(), ::tolower);

            if (keyLower == "content-length")
                contentLength = std::stoi(value);
            else if (keyLower == "transfer-encoding")
            {
                std::string valueLower = value;
                std::transform(valueLower.begin(), valueLower.end(), valueLower.begin(), ::tolower);
                chunked = (valueLower.find("chunked") != std::string::npos);
            }
        }
    }

    // Read body
    if (chunked)
    {
        // Chunked transfer encoding
        while (true)
        {
            bytesRead = portReadLineTimeout(port, lineBuf, sizeof(lineBuf), timeoutMs);
            if (bytesRead <= 0)
                break;

            std::string chunkSizeLine(reinterpret_cast<char*>(lineBuf), bytesRead);
            while (!chunkSizeLine.empty() && (chunkSizeLine.back() == '\r' || chunkSizeLine.back() == '\n'))
                chunkSizeLine.pop_back();

            int chunkSize = (int)std::strtol(chunkSizeLine.c_str(), nullptr, 16);
            if (chunkSize == 0)
                break;

            // Read chunk data
            int remaining = chunkSize;
            while (remaining > 0)
            {
                int toRead = std::min(remaining, (int)sizeof(lineBuf));
                int totalRead = 0;
                uint32_t startMs = current_timeMs();
                while (totalRead < toRead && (current_timeMs() - startMs) < (uint32_t)timeoutMs)
                {
                    int n = portRead(port, lineBuf + totalRead, toRead - totalRead);
                    if (n > 0)
                        totalRead += n;
                    else
                        SLEEP_MS(1);
                }
                resp.body.append(reinterpret_cast<char*>(lineBuf), totalRead);
                remaining -= totalRead;
                if (totalRead == 0)
                    break;
            }
            // Read trailing CRLF after chunk data
            portReadLineTimeout(port, lineBuf, sizeof(lineBuf), timeoutMs);
        }
    }
    else if (contentLength > 0)
    {
        // Read exactly contentLength bytes
        int remaining = contentLength;
        while (remaining > 0)
        {
            int toRead = std::min(remaining, (int)sizeof(lineBuf));
            int totalRead = 0;
            uint32_t startMs = current_timeMs();
            while (totalRead < toRead && (current_timeMs() - startMs) < (uint32_t)timeoutMs)
            {
                int n = portRead(port, lineBuf + totalRead, toRead - totalRead);
                if (n > 0)
                    totalRead += n;
                else
                    SLEEP_MS(1);
            }
            resp.body.append(reinterpret_cast<char*>(lineBuf), totalRead);
            remaining -= totalRead;
            if (totalRead == 0)
                break;
        }
    }
    else
    {
        // Read until connection closes (Connection: close)
        uint32_t startMs = current_timeMs();
        while ((current_timeMs() - startMs) < (uint32_t)timeoutMs)
        {
            int n = portRead(port, lineBuf, sizeof(lineBuf));
            if (n > 0)
            {
                resp.body.append(reinterpret_cast<char*>(lineBuf), n);
                startMs = current_timeMs();  // Reset timeout on data received
            }
            else if (n < 0)
            {
                break;  // Connection closed or error
            }
            else
            {
                SLEEP_MS(1);
            }
        }
    }

    return resp;
}

ISHttpRequest::Response ISHttpRequest::sendRequest(const std::string& url, const std::string& method,
                                                    const std::string& contentType, const std::string& body, int timeoutMs)
{
    Response resp;

    // Parse URL
    FIX8::basic_uri uri(url);
    uri.parse();

    std::string host(uri.get_host());
    std::string portStr(uri.get_port());
    std::string path(uri.get_path());

    if (host.empty())
    {
        log_error(IS_LOG_HTTP_REQUEST, "Failed to parse host from URL: %s", url.c_str());
        return resp;
    }

    int port = portStr.empty() ? 80 : std::strtol(portStr.c_str(), nullptr, 10);
    if (path.empty())
        path = "/";

    log_info(IS_LOG_HTTP_REQUEST, "%s %s (host=%s, port=%d, path=%s)", method.c_str(), url.c_str(), host.c_str(), port, path.c_str());

    // Create TCP connection
    std::string serverUrl = "tcp://" + host + ":" + std::to_string(port);
    port_handle_t tcpPort = TcpPortFactory::getInstance().bindPort(serverUrl, PORT_TYPE__TCP | PORT_TYPE__COMM);

    if (!tcpPort || (portOpen(tcpPort) != PORT_ERROR__NONE))
    {
        log_error(IS_LOG_HTTP_REQUEST, "Failed to connect to %s:%d", host.c_str(), port);
        if (tcpPort)
            TcpPortFactory::getInstance().releasePort(tcpPort);
        return resp;
    }

    // Build and send request
    std::string request = buildRequest(method, host, path, contentType, body);
    int bytesSent = portWrite(tcpPort, (uint8_t*)request.data(), (int)request.length());
    if ((size_t)bytesSent != request.length())
    {
        log_error(IS_LOG_HTTP_REQUEST, "Failed to send request to %s", url.c_str());
        portClose(tcpPort);
        TcpPortFactory::getInstance().releasePort(tcpPort);
        return resp;
    }

    // Parse response
    resp = parseResponse(tcpPort, timeoutMs);

    // Cleanup
    portClose(tcpPort);
    TcpPortFactory::getInstance().releasePort(tcpPort);

    log_info(IS_LOG_HTTP_REQUEST, "Response %d %s (%d bytes)", resp.statusCode, resp.statusMessage.c_str(), (int)resp.body.size());

    return resp;
}

ISHttpRequest::Response ISHttpRequest::get(const std::string& url, int timeoutMs)
{
    return sendRequest(url, "GET", "", "", timeoutMs);
}

ISHttpRequest::Response ISHttpRequest::post(const std::string& url, const std::string& jsonBody, int timeoutMs)
{
    return sendRequest(url, "POST", "application/json", jsonBody, timeoutMs);
}

ISHttpRequest::Response ISHttpRequest::put(const std::string& url, const std::string& jsonBody, int timeoutMs)
{
    return sendRequest(url, "PUT", "application/json", jsonBody, timeoutMs);
}

ISHttpRequest::Response ISHttpRequest::putMultipart(const std::string& url, const std::vector<MultipartField>& fields, int timeoutMs)
{
    static const std::string boundary = "----ISHttpRequestBoundary";

    // Build multipart body
    std::string body;
    for (const auto& field : fields)
    {
        body += "--" + boundary + "\r\n";
        body += "Content-Disposition: form-data; name=\"" + field.name + "\"\r\n";
        if (!field.contentType.empty())
            body += "Content-Type: " + field.contentType + "\r\n";
        body += "\r\n";
        body += field.data + "\r\n";
    }
    body += "--" + boundary + "--\r\n";

    std::string contentType = "multipart/form-data; boundary=" + boundary;
    return sendRequest(url, "PUT", contentType, body, timeoutMs);
}
