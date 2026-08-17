/**
 * @file NtripCorrectionService.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 1/17/26.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "NtripCorrectionService.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

#include "protocol_nmea.h"
#include "TcpPortFactory.h"
#include "util/util.h"

// Max time to wait for the (non-blocking) TCP connection to the caster to complete before giving up.
#define NTRIP_CONNECT_TIMEOUT_MS 5000
// Minimum delay between reconnect attempts after a hard connection failure (avoids hammering the caster).
#define NTRIP_RECONNECT_BACKOFF_MS 2500
// Max time to wait for each line of the caster's response head.
#define NTRIP_RESPONSE_TIMEOUT_MS 1000
// Guard against a caster that answers with headers forever instead of a blank line.
#define NTRIP_MAX_RESPONSE_LINES 64

/**
 * @brief Case-insensitive prefix test.
 * @param s the string to test
 * @param prefix the prefix to look for
 * @return true if s begins with prefix, ignoring case
 */
static bool ntripStartsWith(const std::string& s, const char* prefix) {
    const size_t n = strlen(prefix);
    if (s.size() < n)
        return false;
    for (size_t i = 0; i < n; i++) {
        if (tolower((unsigned char)s[i]) != tolower((unsigned char)prefix[i]))
            return false;
    }
    return true;
}

/**
 * @brief Case-insensitive substring test.
 * @param s the string to search
 * @param needle the substring to look for
 * @return true if needle occurs anywhere in s, ignoring case
 */
static bool ntripContains(const std::string& s, const char* needle) {
    const size_t n = strlen(needle);
    if (n == 0 || s.size() < n)
        return false;
    for (size_t i = 0; i + n <= s.size(); i++) {
        if (ntripStartsWith(s.substr(i, n), needle))
            return true;
    }
    return false;
}

/**
 * @brief Extracts a header's value when the line carries the named field.
 *
 * HTTP field names are case-insensitive and casters vary in how they spell them, so the comparison
 * must be too.
 * @param line the raw response line
 * @param name the field name including its colon, e.g. "Content-Type:"
 * @param[out] value the trimmed field value, only written on a match
 * @return true if the line carried this field
 */
static bool ntripHeaderValue(const std::string& line, const char* name, std::string& value) {
    if (!ntripStartsWith(line, name))
        return false;
    value = utils::trim_copy(line.substr(strlen(name)));
    return true;
}

/**
 * @brief Pulls the numeric status code and reason phrase out of a status line.
 *
 * Handles both "HTTP/1.x <code> <reason>" (Ntrip 2.0) and "ICY <code> <reason>" -- some casters
 * answer a poorly-formed Ntrip 1.0 request with "ICY 401 Unauthorized".
 * @param statusLine the first line of the response
 * @param[out] code the parsed status code
 * @param[out] reason the reason phrase, may be empty
 * @return true if a status code was found
 */
static bool ntripParseStatus(const std::string& statusLine, int& code, std::string& reason) {
    size_t sp = statusLine.find(' ');
    if (sp == std::string::npos)
        return false;

    const std::string token = statusLine.substr(0, sp);
    if (!ntripStartsWith(token, "HTTP/1.") && !ntripStartsWith(token, "ICY"))
        return false;

    const std::string rest = utils::trim_copy(statusLine.substr(sp + 1));
    if (rest.empty() || !isdigit((unsigned char)rest[0]))
        return false;

    code = atoi(rest.c_str());
    const size_t sp2 = rest.find(' ');
    reason = (sp2 == std::string::npos) ? "" : utils::trim_copy(rest.substr(sp2 + 1));
    return true;
}

bool NtripCorrectionService::connect(const std::string& connectUrl, std::string userAgent) {
    // parse the URL; NTRIP defaults to port 2101 when the caster URL omits one
    const utils::UriParts uri = utils::parseUri(connectUrl, "ntrip://:2101");
    const std::string& host = uri.host;
    const int port = uri.port;

    /*
     * connect() is more than portOpen(): once the TCP socket is up we must send the mount-point request
     * and read the caster's response before corrections flow. Because that negotiation has to be repeated
     * whenever the TCP link is re-established, the request is cached (requestMsg) and the negotiation is
     * factored into negotiateMountPoint(); step() replays it on reconnect. (A future refactor could instead
     * hook a port "opened" notification callback so the negotiation is driven by the port layer itself.)
     */

    // Build and cache the mount-point request up front so it can be re-sent verbatim on reconnect
    // (see step() / negotiateMountPoint()) without having to re-parse the URL.
    casterUrl = connectUrl;
    // A redacted copy for logs and for anything a caller may put on screen. casterUrl itself carries
    // the caster password in the clear, so it must never reach either.
    casterUrlSafe = "ntrip://" + std::string(uri.hasUserinfo() ? "[hidden]@" : "") + host + ":" + std::to_string(port) + uri.path;

    requestMsg = "GET " + uri.path + " HTTP/1.1\r\n";
    requestMsg += "User-Agent: " + userAgent + "\r\n";
    if (uri.hasUserinfo()) {
        std::string auth = uri.user + ":" + uri.password;
        requestMsg += "Authorization: Basic " + base64Encode((const unsigned char*)auth.data(), (int)auth.length()) + "\r\n";
    }
    requestMsg += "Accept: */*\r\nConnection: close\r\n\r\n";

    // extract the host & port, and make the socket/port connection
    std::string serverUrl = "tcp://" + host + ":" + std::to_string(port);
    source = TcpPortFactory::getInstance().bindPort(serverUrl, PORT_TYPE__TCP | PORT_TYPE__COMM);

    // Remember, binding a port doesn't open it - it just creates the underlying instance that references the underlying hardware.
    // tcpPort uses a non-blocking connect: portOpen() returns PORT_ERROR__NONE while the TCP handshake is still in flight and
    // only sets PORT_FLAG__OPENED once connected. We must wait for the connection to actually complete (via portOpenRetry, which
    // polls until portIsOpened()) before writing the NTRIP request below - otherwise the write races the handshake and fails.
    if (!source || (portOpenRetry(source, NTRIP_CONNECT_TIMEOUT_MS, 2) != PORT_ERROR__NONE))
        return setResult(NTRIP_RESULT_NOT_CONNECTED, "Could not reach the caster at " + casterUrlSafe);

    if (!negotiateMountPoint()) {
        // The socket is up but the caster refused us, so it will never carry corrections. Close it,
        // both to avoid leaking it and so isConnected() reflects the truth to the caller.
        portClose(source);
        return false;
    }

    setSourcePort(source);  // set the sourcePort for the underlying CorrectionService to this port
    return true;
}

/**
 * Sends the cached mount-point request and consumes the caster's response headers. The source port
 * must already be open. Factored out of connect() so the same negotiation can be replayed on reconnect
 * from step() (an NTRIP caster only starts streaming after it receives the mount-point request).
 */
bool NtripCorrectionService::setResult(eNtripResult result, const std::string& message) {
    lastResult = result;
    lastResultStr = (result == NTRIP_RESULT_OK) ? "" : message;
    if (result != NTRIP_RESULT_OK)
        log_debug(IS_LOG_PORT, "NTRIP %s: %s", casterUrlSafe.c_str(), message.c_str());
    return (result == NTRIP_RESULT_OK);
}

bool NtripCorrectionService::negotiateMountPoint() {
    setResult(NTRIP_RESULT_OK, "");

    if (!portIsOpened(source) || requestMsg.empty())
        return setResult(NTRIP_RESULT_NOT_CONNECTED, "Not connected to the caster");

    int bytesSent = portWrite(source, (uint8_t*)requestMsg.data(), (int)requestMsg.length());
    if ((size_t)bytesSent != requestMsg.length())
        return setResult(NTRIP_RESULT_SEND_FAILED, "Could not send the mount-point request");

    /*
     * Read only the response head -- the status line, then headers, stopping at the blank line that
     * separates it from the payload. On success that payload is the live correction stream, which
     * belongs to the normal RTCM3 path, so reading past the blank line would eat corrections.
     */
    unsigned char buffer[512];
    std::string statusLine, contentType;
    long contentLength = -1;

    for (int line = 0; line < NTRIP_MAX_RESPONSE_LINES; line++) {
        const int bytesRead = portReadLineTimeout(source, buffer, sizeof(buffer), NTRIP_RESPONSE_TIMEOUT_MS);
        if (bytesRead < 0)
            return setResult(NTRIP_RESULT_TIMEOUT, "No response from the caster");
        if (bytesRead == 0)
            break;                  // blank line -- end of the response head

        // portReadLineTimeout() does not NUL-terminate, so always bound by bytesRead.
        const std::string rxLine = utils::trim_copy(std::string(reinterpret_cast<const char*>(buffer), (size_t)bytesRead));

        if (line == 0) {
            statusLine = rxLine;
            continue;
        }
        std::string value;
        if (ntripHeaderValue(rxLine, "Content-Type:", value))
            contentType = value;
        else if (ntripHeaderValue(rxLine, "Content-Length:", value))
            contentLength = atol(value.c_str());
    }

    std::string message;
    return setResult(classifyResponse(statusLine, contentType, contentLength, message), message);
}

NtripCorrectionService::eNtripResult NtripCorrectionService::classifyResponse(
        const std::string& statusLine, const std::string& contentType, long contentLength, std::string& message) {
    /*
     * Classify the status line. Ntrip 1.0 answers "ICY 200 OK" -- Shoutcast-derived, and NOT a valid
     * HTTP status line -- while Ntrip 2.0 answers a proper HTTP one. We send an HTTP/1.1 request, so a
     * caster may legitimately reply in either dialect and both have to be accepted.
     */
    bool statusOk = false;
    if (ntripStartsWith(statusLine, "ICY 200 OK")) {
        statusOk = true;                                                    // Ntrip 1.0 success
    } else if (ntripStartsWith(statusLine, "SOURCETABLE")) {
        // Per the Ntrip spec a caster answers a request for an unavailable mount point with its
        // source table rather than an error, so this 200 is a failure for our purposes.
        message = "Mount point is not available on this caster";
        return NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE;
    } else if (ntripContains(statusLine, "Bad Password")) {
        // Ntrip 1.0's custom "ERROR - Bad Password" status code.
        message = "The caster rejected the credentials";
        return NTRIP_RESULT_UNAUTHORIZED;
    } else if (ntripStartsWith(statusLine, "ERROR")) {
        message = "The caster reported: " + statusLine;
        return NTRIP_RESULT_CASTER_ERROR;
    } else {
        int code = 0;
        std::string reason;
        if (!ntripParseStatus(statusLine, code, reason)) {
            // Report an unknown reply rather than assuming success -- silently accepting it is how a
            // new caster quirk turns into a connection that never explains itself.
            message = "Unrecognized response from the caster: \"" + statusLine.substr(0, 64) + "\"";
            return NTRIP_RESULT_UNRECOGNIZED;
        }
        const std::string suffix = " (" + std::to_string(code) + (reason.empty() ? "" : " " + reason) + ")";
        switch (code) {
            case 200:
                statusOk = true;
                break;
            case 401:
            case 403:
            case 407:
                message = "The caster rejected the credentials" + suffix;
                return NTRIP_RESULT_UNAUTHORIZED;
            case 404:
                message = "Mount point not found on this caster" + suffix;
                return NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE;
            default:
                message = "The caster refused the connection" + suffix;
                return NTRIP_RESULT_CASTER_ERROR;
        }
    }

    /*
     * A 200 is not yet a stream. The caster may have answered with its source table instead, which is
     * what it does for an unavailable mount point. Two independent tells:
     *  - an explicit source-table content type (Ntrip 2.0 spells this "gnss/sourcetable")
     *  - any Content-Length at all: a correction stream is endless and carries none, while a source
     *    table is a finite document and always does. This also covers Ntrip 1.0, which serves its
     *    source table as "text/plain" -- too generic to key on by itself.
     */
    if (statusOk) {
        if (ntripContains(contentType, "sourcetable")) {
            message = "Mount point is not available (the caster returned its source table)";
            return NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE;
        }
        if (contentLength > 0) {
            message = "Mount point is not available (the caster returned a document rather than a stream)";
            return NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE;
        }
    }

    message.clear();
    return NTRIP_RESULT_OK;
}

int NtripCorrectionService::step() {
    if (!portIsOpened(source)) {
        // The NTRIP TCP stream is down (initial connect failed, or the caster dropped us). Reconnecting
        // means more than reopening the socket: the caster only streams after it receives the mount-point
        // request, so we must re-negotiate it. CorrectionService::step() would only reopen the socket and
        // leave us connected-but-silent. The TCP (re)connect is non-blocking (portOpen() is polled across
        // successive step() calls, just like the base class), so we don't stall the caller here.
        if (source && !requestMsg.empty() && (lastConnAttemptTs < current_timeMs())) {
            if (portOpen(source) < PORT_ERROR__NONE) {
                lastConnAttemptTs = current_timeMs() + NTRIP_RECONNECT_BACKOFF_MS;   // hard failure -> back off
            } else if (portIsOpened(source)) {
                // TCP is (re)connected -- re-negotiate the mount point before corrections can resume.
                if (negotiateMountPoint()) {
                    setSourcePort(source);
                } else {
                    portClose(source);                                              // negotiation failed; drop and retry later
                    lastConnAttemptTs = current_timeMs() + NTRIP_RECONNECT_BACKOFF_MS;
                }
            }
        }
        if (!portIsOpened(source))
            return -1;
    }
    return CorrectionService::step();
}

bool NtripCorrectionService::updatePosition(const gnss_pos_t& gps) {
    char rxBuf[512];
    int n = nmea_gga(rxBuf, sizeof(rxBuf), (gnss_pos_t &)gps);
    return (portWrite(source, (uint8_t*)rxBuf, n) == n);
}

bool NtripCorrectionService::updatePosition(const std::string nmeaGGA) {
    return ((size_t)portWriteAscii(source, nmeaGGA.c_str(), nmeaGGA.size()) == nmeaGGA.size());
}

void NtripCorrectionService::setConnectionRequestHeaders(std::map<std::string, std::string> hdrs) {
    headers = hdrs;
}
