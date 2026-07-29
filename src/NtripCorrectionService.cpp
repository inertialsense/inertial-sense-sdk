/**
 * @file NtripCorrectionService.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 1/17/26.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "NtripCorrectionService.h"

#include "protocol_nmea.h"
#include "TcpPortFactory.h"
#include "util/util.h"

// Max time to wait for the (non-blocking) TCP connection to the caster to complete before giving up.
#define NTRIP_CONNECT_TIMEOUT_MS 5000
// Minimum delay between reconnect attempts after a hard connection failure (avoids hammering the caster).
#define NTRIP_RECONNECT_BACKOFF_MS 2500

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
        return false;

    if (!negotiateMountPoint())
        return false;

    setSourcePort(source);  // set the sourcePort for the underlying CorrectionService to this port
    return true;
}

/**
 * Sends the cached mount-point request and consumes the caster's response headers. The source port
 * must already be open. Factored out of connect() so the same negotiation can be replayed on reconnect
 * from step() (an NTRIP caster only starts streaming after it receives the mount-point request).
 */
bool NtripCorrectionService::negotiateMountPoint() {
    if (!portIsOpened(source) || requestMsg.empty())
        return false;

    int bytesSent = portWrite(source, (uint8_t*)requestMsg.data(), (int)requestMsg.length());
    if ((size_t)bytesSent != requestMsg.length()) {
        log_debug(IS_LOG_PORT, "Error submitting NTRIP connection request to %s", casterUrl.c_str());
        return false;
    }

    // Wait for a response that the request was good.
    unsigned char buffer[512];
    int bytesRead = 0, contentLength = 0;
    std::string contentType;
    do {
        bytesRead = portReadLineTimeout(source, buffer, 512, 1000);
        contentLength -= bytesRead;

        if (bytesRead < 0) {
            log_debug(IS_LOG_PORT, "Timeout waiting for response from %s", casterUrl.c_str());
            return false;
        }
        printf("%s\n", buffer);

        std::string rxString(reinterpret_cast<const char*>(buffer), bytesRead);
        if (rxString.compare(0, 14, "Content-Type: ") == 0) {
            contentType = rxString.substr(14);
        } else if (rxString.compare(0, 16, "Content-Length: ") == 0) {
            contentLength = std::stoi(rxString.substr(16));
        }
    } while ((bytesRead != 0) || (contentLength > 0));

    return true;
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
