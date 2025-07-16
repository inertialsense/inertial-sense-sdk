//
// Created by firiusfoxx on 7/3/25.
//

#include <util.h>
#include <chrono>
#include "MDnsTcpPortFactory.h"
#include "PortManager.h"
#include "mdns.h"

static std::chrono::time_point<std::chrono::steady_clock> lastQueryTime;

/**
 * This function parses and creates a new port_handle_t repersenting a TCP Port
 * when passed a URL in the format is-manu://hostname/port to pName and a pType of PORT_TYPE__TCP | PORT_TYPE__COMM
 * @param pName The URL and name of the new port to bind a port_handle_to
 * @param pType The port type requested to be generated
 * @return A port_handle_t bound to the newly created TCP port for the connection pName represents
 */
port_handle_t MDnsTcpPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    if (!validatePort(pName, pType)) {
        return nullptr;
    }

    // Parse pName for address
    const utils::URL url = utils::parseURL(pName);
    if (url.protocol != "is-manu") {
        return nullptr;
    }

    tick(); // Tick everything to ensure we have the latest data

    // MDNS / DNS-SD Resolve goes here

    return nullptr;

    auto* tcpPort = new tcp_port_t;
    auto port = (port_handle_t)tcpPort;
    *tcpPort = {};
    auto id = static_cast<uint16_t>(PortManager::getInstance().getPortCount());
    //tcpPortInit(port, id, this->portOptions.defaultBlocking, pName.c_str(), &addr);

    return port;
}

/**
 * Releases and frees the memory used by this port
 * @param port The TCP Port handle to deinitialize
 * @return True if successful, false otherwise
 */
bool MDnsTcpPortFactory::releasePort(port_handle_t port) {
    if (!port) {
        return false;
    }

    debug_message("[DBG] Releasing network port '%s'\n", ((tcp_port_t*)port)->portName);
    tcpPortDelete(port);
    delete static_cast<tcp_port_t*>(port);

    return true;
}

/**
 * Validate that a provided pName can create a TCP Port
 * @param pName The URL to validate starting with is-manu://
 * @param pType Must be PORT_TYPE__TCP | PORT_TYPE__COMM
 * @return True if port can be created, false otherwise
 */
bool MDnsTcpPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    const utils::URL url = utils::parseURL(pName);
    if (url.protocol != "is-manu") {
        return false;
    }

    if (pType != (PORT_TYPE__TCP | PORT_TYPE__COMM)) {
        return false;
    }

    tick(); // Tick everything to ensure we have the latest data

    return false;
    // Validate existence of MDNS / DNS-SD records
}

/**
 * TCP Port Factory implements a stub function that only creates 1 port if the provided pattern is a valid TCP port url
 * @param portCallback The function to callback into to indicate that this port has been "found"
 * @param pattern The URL to validate and "discover"
 * @param pType Ignored
 */
void MDnsTcpPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) {
    tick(); // Tick everything to ensure we have the latest data

    // Locate ports matching pattern using MDNS / DNS-SD records
}

/**
 * Tick function used to call the mdns::tick() function
 */
void MDnsTcpPortFactory::tick() {
    // Send a lookup for all devices advertising inertialsense-manufacturing every 100 ms
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastQueryTime).count() > MDNS_RECORD_TIMEOUT) {
        mdns::send_mdns_query(MDNS_RECORDTYPE_PTR, "_inertialsense-manufacturing._tcp.local");
        lastQueryTime = std::chrono::steady_clock::now();
    }

    // Allow the MDNS responder to do all of it's processing
    mdns::tick();
}