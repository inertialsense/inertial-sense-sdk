/**
 * @file ISHttpRequest.h
 * @brief Lightweight HTTP client using tcpPort infrastructure
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_ISHTTPREQUEST_H
#define IS_SDK_ISHTTPREQUEST_H

#include <string>
#include <map>

extern "C"
{
    #include "core/base_port.h"
}

class ISHttpRequest {
public:
    struct Response {
        int statusCode = -1;
        std::string statusMessage;
        std::string body;
        std::map<std::string, std::string> headers;
    };

    /**
     * Perform an HTTP GET request to the given URL.
     * @param url Full HTTP URL (e.g., "http://host:port/path")
     * @param timeoutMs Timeout in milliseconds for the entire request
     * @return Response with status code, headers, and body. statusCode == -1 on error.
     */
    static Response get(const std::string& url, int timeoutMs = 10000);

private:
    static std::string buildGetRequest(const std::string& host, const std::string& path);
    static Response parseResponse(port_handle_t port, int timeoutMs);
};

#endif // IS_SDK_ISHTTPREQUEST_H
