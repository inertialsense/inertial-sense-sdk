/**
 * @file RelayPortFactory.cpp
 * @brief Discovers IS device ports through remote HTTP-based relay hosts.
 *
 * @author Kyle Mallory on 4/10/26.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "RelayPortFactory.h"
#include "PortManager.h"
#include "core/msg_logger.h"

#include <algorithm>
#include <regex>

// cpp-httplib and nlohmann/json are used only in this .cpp — not exposed via the header.
// Phase 1: polling stubs only; actual HTTP calls will be wired in Step 3 (HTTP polling implementation).
// #include "httplib.h"
// #include "json.hpp"

// ============================================================
// Service-style management API
// ============================================================

void RelayPortFactory::addRelayHost(const std::string& url) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (relayHosts_.find(url) != relayHosts_.end())
        return; // already known

    RelayHost host;
    host.url = url;
    host.enabled = false;
    host.viaMdns = false;
    relayHosts_[url] = std::move(host);
    log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: added manual relay host '%s'", url.c_str());
}

bool RelayPortFactory::removeRelayHost(const std::string& url) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = relayHosts_.find(url);
    if (it == relayHosts_.end())
        return false;

    log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: removed relay host '%s'", url.c_str());
    relayHosts_.erase(it);
    return true;
}

void RelayPortFactory::setRelayHostEnabled(const std::string& url, bool enabled) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto it = relayHosts_.find(url);
    if (it == relayHosts_.end())
        return;

    if (it->second.enabled != enabled) {
        it->second.enabled = enabled;
        log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: relay host '%s' %s",
                 url.c_str(), enabled ? "enabled" : "disabled");

        if (!enabled) {
            // Clear cached devices so locatePorts() stops emitting them
            it->second.devices.clear();
        }
    }
}

std::vector<RelayPortFactory::RelayHostStatus> RelayPortFactory::getRelayHosts() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<RelayHostStatus> result;
    result.reserve(relayHosts_.size());
    for (const auto& [url, host] : relayHosts_) {
        RelayHostStatus status;
        status.url = host.url;
        status.enabled = host.enabled;
        status.viaMdns = host.viaMdns;
        status.lastPollTime = host.lastPollTime;
        status.lastError = host.lastError;
        status.consecutiveFailures = host.consecutiveFailures;
        status.deviceCount = host.devices.size();
        result.push_back(std::move(status));
    }
    return result;
}

// ============================================================
// PortFactory interface (stubs — wired in Steps 2-4)
// ============================================================

void RelayPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback,
                                    const std::string& pattern, uint16_t pType) {
    tick();
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // Phase 1 Step 4 will populate this from cached device records.
    // For now, emit nothing — the skeleton compiles and integrates safely.
    (void)portCallback;
    (void)pattern;
    (void)pType;
}

bool RelayPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    tick();
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // Phase 1 Step 4 will check pName against cached device records.
    (void)pName;
    (void)pType;
    return false;
}

port_handle_t RelayPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    tick();
    if (!validatePort(pName, pType)) {
        return nullptr;
    }

    // Phase 1 Step 4 will create a tcp_port_t here (mirroring TcpPortFactory::bindPort).
    return nullptr;
}

bool RelayPortFactory::releasePort(port_handle_t port) {
    if (!port) {
        return false;
    }

    // Phase 1 Step 4 will call tcpPortDelete + delete here (mirroring TcpPortFactory::releasePort).
    log_debug(IS_LOG_FACILITY_NONE, "RelayPortFactory: releasing port '%s'", portName(port));
    tcpPortDelete(port);
    delete static_cast<tcp_port_t*>(port);
    return true;
}

// ============================================================
// Polling driver
// ============================================================

void RelayPortFactory::tick() {
    auto& self = getInstance();
    std::lock_guard<std::recursive_mutex> lock(self.mutex_);

    // Phase 1 Step 2 will wire mDNS host discovery here (discoverRelayHostsViaMdns).
    // Phase 1 Step 3 will wire HTTP polling for enabled hosts here (pollRelayHost).
}

// ============================================================
// Internal helpers (stubs — wired in Steps 2-3)
// ============================================================

void RelayPortFactory::discoverRelayHostsViaMdns() {
    // Phase 1 Step 2: query mdns::getRecords() for _inertialsense-discovery._tcp.local SRV records,
    // extract hostnames, register new hosts at http://<hostname>:DEFAULT_HTTP_PORT/api/status
    // with enabled=false and viaMdns=true.
}

void RelayPortFactory::pollRelayHost(RelayHost& host) {
    // Phase 1 Step 3: HTTP GET host.url via cpp-httplib, parse JSON response via nlohmann/json,
    // populate host.devices with DeviceRecord entries, update host.lastPollTime / lastError /
    // consecutiveFailures. Error resilience: retain last-known-good devices on failure.
}