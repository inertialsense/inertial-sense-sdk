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
#include <vector>

extern "C"
{
    #include "core/base_port.h"
}

/** Lightweight, blocking HTTP/1.1 client built on the tcpPort transport; supports GET/POST/PUT and multipart PUT. */
class ISHttpRequest {
public:
    /** Result of an HTTP request performed by ISHttpRequest. */
    struct Response {
        int statusCode = -1;                        //!< HTTP status code (e.g. 200), or -1 if the request failed before a response was received
        std::string statusMessage;                   //!< HTTP status reason phrase (e.g. "OK")
        std::string body;                             //!< response body, with any chunked/length framing already removed
        std::map<std::string, std::string> headers;   //!< response headers, keyed by header name
    };

    /** One field of a multipart/form-data body sent via putMultipart(). */
    struct MultipartField {
        std::string name;          //!< form field name
        std::string contentType;   //!< MIME type of data, e.g. "application/json", "application/zip"
        std::string data;          //!< raw field content
    };

    /**
     * Perform an HTTP GET request to the given URL.
     * @param url Full HTTP URL (e.g., "http://host:port/path")
     * @param timeoutMs Timeout in milliseconds for the entire request
     * @return Response with status code, headers, and body. statusCode == -1 on error.
     */
    static Response get(const std::string& url, int timeoutMs = 10000);

    /**
     * Perform an HTTP POST request with a JSON body.
     * @param url Full HTTP URL
     * @param jsonBody JSON string to send as the request body
     * @param timeoutMs Timeout in milliseconds
     * @return Response with status code, headers, and body. statusCode == -1 on error.
     */
    static Response post(const std::string& url, const std::string& jsonBody, int timeoutMs = 10000);

    /**
     * Perform an HTTP PUT request with a JSON body.
     * @param url Full HTTP URL
     * @param jsonBody JSON string to send as the request body
     * @param timeoutMs Timeout in milliseconds
     * @return Response with status code, headers, and body. statusCode == -1 on error.
     */
    static Response put(const std::string& url, const std::string& jsonBody, int timeoutMs = 10000);

    /**
     * Perform an HTTP PUT request with a multipart/form-data body.
     * @param url Full HTTP URL
     * @param fields Vector of multipart fields to include in the body
     * @param timeoutMs Timeout in milliseconds
     * @return Response with status code, headers, and body. statusCode == -1 on error.
     */
    static Response putMultipart(const std::string& url, const std::vector<MultipartField>& fields, int timeoutMs = 10000);

private:
    /** Builds a raw HTTP/1.1 GET request line and headers for host/path. */
    static std::string buildGetRequest(const std::string& host, const std::string& path);

    /** Builds a raw HTTP/1.1 request (any method) with optional content type and body. */
    static std::string buildRequest(const std::string& method, const std::string& host, const std::string& path,
                                    const std::string& contentType, const std::string& body);

    /** Parses url, opens a tcpPort connection, sends the request, and returns the parsed Response. */
    static Response sendRequest(const std::string& url, const std::string& method,
                                const std::string& contentType, const std::string& body, int timeoutMs);

    /** Reads and parses an HTTP status line, headers, and body from an already-connected port. */
    static Response parseResponse(port_handle_t port, int timeoutMs);
};

#endif // IS_SDK_ISHTTPREQUEST_H
