/**
 * @file RelayPortFactory.cpp
 * @brief Discovers IS device ports through remote HTTP-based relay hosts.
 *
 * @author Kyle Mallory on 4/10/26.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "RelayPortFactory.h"
#include "TcpPortFactory.h"
#include "PortManager.h"
#include "core/msg_logger.h"
#include "protocol/mdns.hpp"

#include <set>

// cpp-httplib and nlohmann/json — used only in this .cpp, not exposed via the header.
#include "httplib.h"
#include "json.hpp"
#include <util/uri.hpp>

using json = nlohmann::json;

namespace {

/// Map bridgeboard's module "state" string to IS hardware type enum.
uint8_t stateToHardwareType(const std::string& state) {
    if (state == "imx5" || state == "imx6") return IS_HARDWARE_TYPE_IMX;
    if (state == "gpx")                     return IS_HARDWARE_TYPE_GPX;
    if (state == "isbl")                    return IS_HARDWARE_TYPE_UINS; // bootloader — type ambiguous, will be refined by probe
    return IS_HARDWARE_TYPE_UNKNOWN;
}

/// Parse a firmware version string like "fw3.0.0-snap" or "2.4.0" into a 4-byte array [major, minor, patch, build].
void parseFirmwareVer(const std::string& verStr, uint8_t out[4]) {
    out[0] = out[1] = out[2] = out[3] = 0;
    // Strip leading "fw" prefix if present
    std::string s = verStr;
    if (s.size() > 2 && (s[0] == 'f' || s[0] == 'F') && (s[1] == 'w' || s[1] == 'W'))
        s = s.substr(2);
    // Parse "major.minor.patch" — ignore anything after a dash or non-numeric
    int idx = 0;
    size_t pos = 0;
    while (idx < 4 && pos < s.size()) {
        size_t dot = s.find_first_of(".-", pos);
        if (dot == std::string::npos) dot = s.size();
        std::string part = s.substr(pos, dot - pos);
        if (!part.empty()) {
            try { out[idx] = static_cast<uint8_t>(std::stoi(part)); } catch (...) {}
        }
        idx++;
        pos = dot + 1;
        if (dot < s.size() && s[dot] == '-') break; // stop at -snap, -rc1, etc.
    }
}

/// Extract host (without scheme/port/path) from a URL like "http://host.local:8080/api/status".
std::string extractHost(const std::string& url) {
    const FIX8::uri parsed{url};
    return std::string{parsed.get_host()};
}

/// HTTP connection timeout (seconds)
static constexpr int HTTP_CONNECT_TIMEOUT_S = 3;
/// HTTP read timeout (seconds)
static constexpr int HTTP_READ_TIMEOUT_S = 5;

} // anonymous namespace

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
            // Clear cached state so locatePorts() stops emitting ports and
            // validatePort() returns false — PortManager will evict them.
            it->second.devices.clear();
            it->second.knownPortUrls.clear();
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
// PortFactory interface
// ============================================================

void RelayPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback,
                                    const std::string& pattern, uint16_t pType) {
    tick();
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // Emit cached device ports from all enabled hosts, matching against pattern.
    // Ports are attributed to this factory (not TcpPortFactory) so that PortManager
    // calls our validatePort() for eviction checks — this lets us remove ports
    // promptly when a host is disabled or its cached device list changes.
    std::regex regexPattern;
    try {
        regexPattern = std::regex(pattern);
    } catch (const std::regex_error&) {
        return;
    }

    for (const auto& [url, host] : relayHosts_) {
        if (!host.enabled)
            continue;
        // Emit from knownPortUrls (high-water mark), NOT from the latest poll's devices.
        // This ensures ports persist across device reboots — bridgeboard's slot-based TCP
        // sockets survive device resets (WaitRecover keeps the listener open), so the
        // tcp:// URL remains valid even when the device is temporarily absent from /api/status.
        for (const auto& portUrl : host.knownPortUrls) {
            if (std::regex_match(portUrl, regexPattern)) {
                portCallback(this, PORT_TYPE__TCP | PORT_TYPE__COMM | pType, portUrl);
            }
        }
    }
}

bool RelayPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    // PortManager calls this to decide whether to keep or evict a port.
    // A port is valid as long as its relay host is enabled and the port URL is in that
    // host's knownPortUrls set. Ports survive device reboots because bridgeboard's
    // slot-based TCP sockets persist across device resets.
    tick();
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if ((pType & PORT_TYPE__TCP) != PORT_TYPE__TCP)
        return false;

    for (const auto& [url, host] : relayHosts_) {
        if (!host.enabled)
            continue;
        if (host.knownPortUrls.count(pName))
            return true;
    }
    return false;
}

port_handle_t RelayPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    // Port binding is handled by TcpPortFactory (ports are attributed to it in locatePorts).
    // Delegate to TcpPortFactory if called directly as a fallback.
    return TcpPortFactory::getInstance().bindPort(pName, pType);
}

bool RelayPortFactory::releasePort(port_handle_t port) {
    // Port release is handled by TcpPortFactory (ports are attributed to it in locatePorts).
    // Delegate to TcpPortFactory if called directly as a fallback.
    return TcpPortFactory::getInstance().releasePort(port);
}

// ============================================================
// Polling driver
// ============================================================

void RelayPortFactory::tick() {
    auto& self = getInstance();
    std::lock_guard<std::recursive_mutex> lock(self.mutex_);

    // Refresh the shared mDNS cache (rate-limited, same query ISmDnsPortFactory uses)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - self.lastMdnsQueryTime_);
    if (elapsed.count() > MDNS_QUERY_INTERVAL_MS) {
        mdns::sendQuery(MDNS_RECORDTYPE_PTR, "_inertialsense-discovery._tcp.local");
        self.lastMdnsQueryTime_ = now;
    }
    mdns::tick();

    // Discover new relay hosts from mDNS cache
    self.discoverRelayHostsViaMdns();

    // Poll each enabled host (rate-limited per host by pollInterval_)
    for (auto& [url, host] : self.relayHosts_) {
        if (!host.enabled)
            continue;
        auto sincePoll = std::chrono::duration_cast<std::chrono::milliseconds>(now - host.lastPollTime);
        if (sincePoll >= self.pollInterval_ || host.lastPollTime == std::chrono::steady_clock::time_point{}) {
            self.pollRelayHost(host);
        }
    }
}

// ============================================================
// Internal helpers (stubs — wired in Steps 2-3)
// ============================================================

void RelayPortFactory::discoverRelayHostsViaMdns() {
    // mutex_ is already held by tick()

    // Step 1: Get all PTR records for the IS discovery service
    auto ptrRecords = mdns::getRecords([](const mdns::mdns_record_cpp_t& r) {
        return r.type == MDNS_RECORDTYPE_PTR && r.name == "_inertialsense-discovery._tcp.local.";
    });

    // Step 2: For each PTR, resolve the SRV to get the relay's hostname
    std::set<std::string> seenHostnames;
    for (const auto& ptr : ptrRecords) {
        auto srvRecords = mdns::getRecords([&ptr](const mdns::mdns_record_cpp_t& r) {
            return r.type == MDNS_RECORDTYPE_SRV && r.name == ptr.data.ptr.name;
        });
        if (srvRecords.empty())
            continue;

        // Normalize hostname: strip trailing dot
        std::string hostname = srvRecords[0].data.srv.name;
        if (!hostname.empty() && hostname.back() == '.')
            hostname.pop_back();

        // Deduplicate (a single Pi may appear via multiple PTR records on different interfaces)
        if (hostname.empty() || !seenHostnames.insert(hostname).second)
            continue;

        // Build the relay URL with the default HTTP port
        std::string url = "http://" + hostname + ":" + std::to_string(DEFAULT_HTTP_PORT) + "/api/status";

        // Register if not already known
        if (relayHosts_.find(url) == relayHosts_.end()) {
            RelayHost host;
            host.url = url;
            host.enabled = false;
            host.viaMdns = true;
            relayHosts_[url] = std::move(host);
            log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: discovered relay host via mDNS: '%s'", url.c_str());
        }
    }
}

void RelayPortFactory::pollRelayHost(RelayHost& host) {
    // mutex_ is already held by tick()

    // Parse host URL to extract scheme://host:port and path
    const FIX8::uri parsed{host.url};
    std::string hostname = std::string{parsed.get_host()};
    std::string portStr = std::string{parsed.get_port()};
    std::string path = std::string{parsed.get_path()};
    if (path.empty()) path = "/api/status";
    int port = portStr.empty() ? DEFAULT_HTTP_PORT : std::stoi(portStr);

    // HTTP GET with timeouts
    httplib::Client client(hostname, port);
    client.set_connection_timeout(HTTP_CONNECT_TIMEOUT_S);
    client.set_read_timeout(HTTP_READ_TIMEOUT_S);

    auto res = client.Get(path);
    if (!res || res->status != 200) {
        host.consecutiveFailures++;
        if (res) {
            host.lastError = "HTTP " + std::to_string(res->status);
        } else {
            host.lastError = httplib::to_string(res.error());
        }
        if (host.consecutiveFailures == DEFAULT_FAILURE_GRACE_COUNT) {
            log_warn(IS_LOG_FACILITY_NONE, "RelayPortFactory: relay host '%s' unreachable after %u polls: %s",
                     host.url.c_str(), host.consecutiveFailures, host.lastError.c_str());
        }
        // Error resilience: retain last-known-good devices — do NOT clear host.devices
        return;
    }

    // Parse JSON response
    json statusJson;
    try {
        statusJson = json::parse(res->body);
    } catch (const json::parse_error& e) {
        host.consecutiveFailures++;
        host.lastError = std::string("JSON parse error: ") + e.what();
        log_warn(IS_LOG_FACILITY_NONE, "RelayPortFactory: malformed JSON from '%s': %s",
                 host.url.c_str(), host.lastError.c_str());
        return;
    }

    // Build new device list from the JSON response
    // Structure: array of testbed objects, each with a "modules" array
    std::vector<DeviceRecord> newDevices;

    if (!statusJson.is_array()) {
        host.consecutiveFailures++;
        host.lastError = "Expected JSON array of testbeds";
        return;
    }

    for (const auto& testbed : statusJson) {
        if (!testbed.contains("modules") || !testbed["modules"].is_array())
            continue;

        for (const auto& module : testbed["modules"]) {
            std::string state = module.value("state", "none");
            if (state == "none" || state == "cdc" || state == "dfu")
                continue; // no IS device identified yet

            int tcpPort = module.value("tcp_port", 0);
            if (tcpPort <= 0)
                continue;

            // Build the tcp:// URL using the relay's hostname
            std::string portUrl = "tcp://" + hostname + ":" + std::to_string(tcpPort);

            // Populate dev_info_t hint from the relay's authoritative metadata
            dev_info_t hint = {};
            hint.hardwareType = stateToHardwareType(state);
            hint.hdwRunState = (state == "isbl") ? HDW_STATE_BOOTLOADER : HDW_STATE_APP;
            hint.serialNumber = module.value("serial_number", 0u);

            std::string fwVer = module.value("firmware_ver", "");
            if (!fwVer.empty())
                parseFirmwareVer(fwVer, hint.firmwareVer);

            newDevices.push_back({portUrl, hint});
        }
    }

    // Success — update host state
    host.devices = std::move(newDevices);
    host.lastPollTime = std::chrono::steady_clock::now();
    host.lastError.clear();
    host.consecutiveFailures = 0;

    // Accumulate port URLs into the high-water mark set. Ports are never removed from this
    // set by polling — only by explicit host disable/remove. This reflects the relay's
    // slot-based persistent TCP sockets: a device rebooting doesn't close the listener,
    // so the tcp:// URL remains valid even when the device is temporarily absent from /api/status.
    for (const auto& device : host.devices) {
        host.knownPortUrls.insert(device.portUrl);
    }

    log_debug(IS_LOG_FACILITY_NONE, "RelayPortFactory: polled '%s' — %zu devices, %zu known ports",
              host.url.c_str(), host.devices.size(), host.knownPortUrls.size());
}