/**
 * @file NtripCorrectionService.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 1/17/26.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "NtripCorrectionService.h"

#include "uri.hpp"
#include "protocol_nmea.h"
#include "TcpPortFactory.h"

bool NtripCorrectionService::connect(const std::string& connectUrl, std::string userAgent) {
    FIX8::basic_uri uri(connectUrl);

    // parse the URL
    uri.parse();
    std::string host(uri.get_host());
    std::string portStr(uri.get_port());
    int port = std::strtol(portStr.c_str(), NULL, 10);

    /***
     * One issue here is that connect() is not the same as portOpen(), because connect needs to do some talking
     * to negotiate the connection - we open the port, and then we send the following parameters and wait for a
     * response. If the parameters aren't right, the remote end will shut down, and we didn't really connect.
     *
     * If we try and call portOpen() again to reconnect (which we will in CorrectionService::step()), we'll
     * reconnect the TCP socket, but won't call connect(), which mean no negotiations.
     *
     * One option here is to replace tcp_port_t.base.portOpen function pointer with a Ntrip-specific function,
     * injecting itself into the middle, so that portOpen() calls the Ntrip's connect() and directly calls
     * tcpPortOpen().  We could do this because NTRIP should only ever use tcpPort.  Should being the operative
     * word there.
     *
     * However, we also have an issue of context - since NtripCorrectionService is C++ and needs its 'this'
     * but tcpPort doesn't known anything about that... How do you call portOpen() and will carry over the
     * object instance that holds the tcp_port_s struct?  These are good questions!
     *
     * Solving the instance issue, we could add a "context" or "userdata" parameter to base_port_t. This would
     * be pretty minimally invasive, and wouldn't change any functions signatures, etc. This would only require
     * two additional functions, portSetUserContext() and portGetUserContext(), and a corresponding void pointer
     * in base_port_t. The context pointer is assigned per port though, not per call, etc. I think this can work.
     *
     * In addition to the userContext pointer, we'd need to implement an additional callback into base_port_t, a
     * portNotify callback which would get called (if defined) when port operations (like an Open, Close, etc) are
     * made with a port. Similar to the portLogger callback, this would include the port_handle_t and a PORT_OP__*
     * argument indicating the event type that we are being notified of.  This could grow, if we're not careful
     * since there might be need for things like PORT_OP__OPENING vs PORT_OP__OPENED, etc.  Still, might be worth
     * exploring.
     *
     * In our case here for the Ntrip, we'd do the following:
     *      portSetUserContext(source, this);
     *      portSetNotifyCallback(source, NtripCorrectionService::portConnected);
     *
     * Where NtripCorrectionService::portConnected() would look like:
     *      static void NtripCorrectionService::portConnected(port_handle_t port, int port_op) {
     *          if (port_op == PORT_OP__OPENED) {
     *              NtripCorrectionService* ntrip = (NtripCorrectionService*)portGetUserContext(port);
     *              if (ntrip)
     *                  ntrip->makeConnectionRequest();
     *          }
     *      }
     ****/

    // extract the host & port, and make the socket/port connection
    std::string serverUrl = "tcp://" + host + ":" + std::to_string(port);
    source = TcpPortFactory::getInstance().bindPort(serverUrl, PORT_TYPE__TCP | PORT_TYPE__COMM);

    // Remember, binding a port doesn't open it - it just creates the underlying instance that references the underlying hardware
    if (!source || (portOpen(source) != PORT_ERROR__NONE))
        return false;

    std::string msg = "GET " + std::string(uri.get_path()) + " HTTP/1.1\r\n";
    msg += "User-Agent: " + userAgent + "\r\n";
    if (uri.has_userinfo()) {
        std::string auth = std::string(uri.get_user()) + ":" + std::string(uri.get_password());
        msg += "Authorization: Basic " + base64Encode((const unsigned char*)auth.data(), (int)auth.length()) + "\r\n";
    }
    msg += "Accept: */*\r\nConnection: close\r\n\r\n";

    int bytesSent = portWrite(source, (uint8_t*)msg.data(), (int)msg.length());
    if ((size_t)bytesSent != msg.length()) {
        log_debug(IS_LOG_PORT, "Error submitting NTRIP connection request to %s", connectUrl.c_str());
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
            log_debug(IS_LOG_PORT, "Timeout waiting for response from %s", connectUrl.c_str());
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

    setSourcePort(source);  // set the sourcePort for the underlying CorrectionService to this port
    return true;
}

bool NtripCorrectionService::updatePosition(const gps_pos_t& gps) {
    char rxBuf[512];
    int n = nmea_gga(rxBuf, sizeof(rxBuf), (gps_pos_t &)gps);
    return (portWrite(source, (uint8_t*)rxBuf, n) == n);
}

bool NtripCorrectionService::updatePosition(const std::string nmeaGGA) {
    return ((size_t)portWriteAscii(source, nmeaGGA.c_str(), nmeaGGA.size()) == nmeaGGA.size());
}

void NtripCorrectionService::setConnectionRequestHeaders(std::map<std::string, std::string> hdrs) {
    headers = hdrs;
}
