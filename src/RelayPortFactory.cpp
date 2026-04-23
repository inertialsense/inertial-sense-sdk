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
#include "DeviceManager.h"
#include "core/msg_logger.h"
#include "protocol/mdns.hpp"
#include "ISComm.h"

#include <algorithm>
#include <chrono>
#include <set>
#include <thread>

// cpp-httplib and nlohmann/json — used only in this .cpp, not exposed via the header.
#include "httplib.h"
#include "json.hpp"
#include <util/uri.hpp>

using json = nlohmann::json;

namespace {

/// Map bridgeboard's module "state" string to a full encoded is_hardware_t (type + major + minor).
is_hardware_t stateToHardwareId(const std::string& state) {
    if (state == "imx5") return IS_HARDWARE_IMX_5_0;
    if (state == "imx6") return IS_HARDWARE_IMX_6_0;
    if (state == "gpx")  return IS_HARDWARE_GPX_1_0;
    if (state == "isbl") return ENCODE_HDW_ID(IS_HARDWARE_TYPE_UINS, 0, 0); // bootloader — type ambiguous
    return IS_HARDWARE_NONE;
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

/// Canonicalize any relay input (bare hostname, IP, full URL, URL-with-path) into
/// the factory's storage key form: "http://<host>:<port>" with no trailing slash and no path.
///
/// Examples:
///   "http://host.local:8080/api/status"   -> "http://host.local:8080"
///   "http://host.local:9090/"             -> "http://host.local:9090"
///   "http://host.local"                   -> "http://host.local:8080"   (default port applied)
///   "host.local:9090"                     -> "http://host.local:9090"   (scheme defaulted)
///   "10.1.2.3"                            -> "http://10.1.2.3:8080"
///
/// Returns an empty string on unparseable input.
std::string normalizeBaseUrl(const std::string& input, uint16_t defaultPort) {
    std::string s = input;
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))  s.pop_back();
    if (s.empty()) return {};

    if (s.find("://") == std::string::npos) {
        s = "http://" + s;
    }

    const FIX8::uri parsed{s};
    std::string host = std::string{parsed.get_host()};
    std::string portStr = std::string{parsed.get_port()};
    if (host.empty()) return {};

    uint16_t port = defaultPort;
    if (!portStr.empty()) {
        try { port = static_cast<uint16_t>(std::stoi(portStr)); } catch (...) { return {}; }
    }

    return "http://" + host + ":" + std::to_string(port);
}

/// Split a canonical "http://host:port" URL into hostname + port. Returns {"", defaultPort} on failure.
std::pair<std::string, int> splitBaseUrl(const std::string& baseUrl, uint16_t defaultPort) {
    const FIX8::uri parsed{baseUrl};
    std::string host = std::string{parsed.get_host()};
    std::string portStr = std::string{parsed.get_port()};
    int port = defaultPort;
    if (!portStr.empty()) {
        try { port = std::stoi(portStr); } catch (...) { port = defaultPort; }
    }
    return {host, port};
}

/// HTTP timeouts (seconds)
static constexpr int HTTP_CONNECT_TIMEOUT_S = 3;
static constexpr int HTTP_READ_TIMEOUT_S    = 5;
/// SSE read timeout — generous enough to outlive server keepalive (15 s per SN-7804)
static constexpr int SSE_READ_TIMEOUT_S     = 30;
/// SSE reconnect backoff (ms) — bounded exponential
static constexpr int SSE_RECONNECT_INITIAL_MS = 250;
static constexpr int SSE_RECONNECT_MAX_MS     = 2000;

/// Parse a single device entry from the SN-7804 `/api/availableDevices` schema
/// (or from a `device.added` / `device.changed` SSE event payload — same shape).
bool parseDeviceJson(const json& dev, RelayPortFactory::DeviceRecord& out) {
    if (!dev.is_object()) return false;

    std::string state = dev.value("state", "none");
    if (state == "none" || state == "cdc" || state == "dfu") return false;

    std::string uri = dev.value("uri", "");
    if (uri.empty()) return false;

    dev_info_t hint = {};
    is_hardware_t hdwId = stateToHardwareId(state);
    hint.hardwareType = DECODE_HDW_TYPE(hdwId);
    hint.hardwareVer[0] = DECODE_HDW_MAJOR(hdwId);
    hint.hardwareVer[1] = DECODE_HDW_MINOR(hdwId);
    hint.hdwRunState = (state == "isbl") ? HDW_STATE_BOOTLOADER : HDW_STATE_APP;
    hint.serialNumber = dev.value("serial_number", 0u);
    hint.protocolVer[0] = PROTOCOL_VERSION_CHAR0;

    std::string fwVer = dev.value("firmware_ver", "");
    if (!fwVer.empty()) parseFirmwareVer(fwVer, hint.firmwareVer);

    out.portUrl = std::move(uri);
    out.hint = hint;
    return true;
}

/// Parse the SN-7804 snapshot envelope (shared between `/api/availableDevices` HTTP response
/// and SSE `snapshot` event payload). Expected shape:
/// `{ "server_instance_id": "...", "snapshot_id": N, "devices": [ ... ] }`
struct SnapshotResult {
    std::string serverInstanceId;
    uint64_t    snapshotId = 0;
    std::vector<RelayPortFactory::DeviceRecord> devices;
};

bool parseSnapshotJson(const json& doc, SnapshotResult& out) {
    if (!doc.is_object()) return false;

    out.serverInstanceId = doc.value("server_instance_id", "");
    out.snapshotId       = doc.value("snapshot_id", 0ull);

    if (!doc.contains("devices") || !doc["devices"].is_array())
        return false;

    out.devices.clear();
    out.devices.reserve(doc["devices"].size());
    for (const auto& dev : doc["devices"]) {
        RelayPortFactory::DeviceRecord rec;
        if (parseDeviceJson(dev, rec))
            out.devices.push_back(std::move(rec));
    }
    return true;
}

/// Split a composite SSE id "<instance_uuid>:<snapshot_id>" into its parts.
/// Returns {instance, snapshotId}; instance is empty and snapshotId is 0 on malformed input.
std::pair<std::string, uint64_t> splitEventId(const std::string& id) {
    auto colon = id.find(':');
    if (colon == std::string::npos) return {"", 0};
    std::string inst = id.substr(0, colon);
    uint64_t snap = 0;
    try { snap = std::stoull(id.substr(colon + 1)); } catch (...) { return {"", 0}; }
    return {std::move(inst), snap};
}

/// HTTP paths on a relay host (appended to the stored base URL).
static constexpr const char* PATH_AVAILABLE_DEVICES = "/api/availableDevices";
static constexpr const char* PATH_EVENTS_DEVICES    = "/api/events/devices";

/// SSE event type names emitted by bridgeboard per SN-7804.
static constexpr const char* EVT_SNAPSHOT        = "snapshot";
static constexpr const char* EVT_DEVICE_ADDED    = "device.added";
static constexpr const char* EVT_DEVICE_CHANGED  = "device.changed";
static constexpr const char* EVT_DEVICE_REMOVED  = "device.removed";

} // anonymous namespace

// ============================================================
// Destructor — tears down all SSE workers cleanly
// ============================================================

RelayPortFactory::~RelayPortFactory() {
    // Collect workers under the lock, then signal + join OUTSIDE the lock so a worker
    // callback that's currently blocked acquiring mutex_ can make progress and exit.
    std::vector<std::unique_ptr<std::thread>> threads;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        for (auto& [url, host] : relayHosts_) {
            host->stopRequested.store(true);
            if (host->streamThread) threads.push_back(std::move(host->streamThread));
        }
    }
    for (auto& t : threads) {
        if (t && t->joinable()) t->join();
    }
}

// ============================================================
// Service-style management API
// ============================================================

void RelayPortFactory::addRelayHost(const std::string& url) {
    std::string canonical = normalizeBaseUrl(url, DEFAULT_HTTP_PORT);
    if (canonical.empty()) {
        log_warn(IS_LOG_FACILITY_NONE, "RelayPortFactory: ignored unparseable relay URL '%s'", url.c_str());
        return;
    }

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (relayHosts_.find(canonical) != relayHosts_.end())
        return; // already known

    auto host = std::make_unique<RelayHost>();
    host->url = canonical;
    host->enabled = false;
    host->viaMdns = false;
    relayHosts_[canonical] = std::move(host);
    log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: added manual relay host '%s'", canonical.c_str());
}

bool RelayPortFactory::removeRelayHost(const std::string& url) {
    std::string canonical = normalizeBaseUrl(url, DEFAULT_HTTP_PORT);
    if (canonical.empty()) return false;

    // Pull the host out of the map, then tear down its worker OUTSIDE the lock.
    std::unique_ptr<RelayHost> removed;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto it = relayHosts_.find(canonical);
        if (it == relayHosts_.end())
            return false;

        it->second->stopRequested.store(true);
        removed = std::move(it->second);
        relayHosts_.erase(it);
    }
    if (removed && removed->streamThread && removed->streamThread->joinable()) {
        removed->streamThread->join();
    }
    log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: removed relay host '%s'", canonical.c_str());
    return true;
}

void RelayPortFactory::setRelayHostEnabled(const std::string& url, bool enabled) {
    std::string canonical = normalizeBaseUrl(url, DEFAULT_HTTP_PORT);
    if (canonical.empty()) return;

    // Capture the thread to join outside the lock on a disable.
    std::unique_ptr<std::thread> toJoin;

    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto it = relayHosts_.find(canonical);
        if (it == relayHosts_.end()) return;

        RelayHost& host = *it->second;
        if (host.enabled == enabled) return;

        host.enabled = enabled;
        log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: relay host '%s' %s",
                 canonical.c_str(), enabled ? "enabled" : "disabled");

        if (enabled) {
            host.activeTransport = RelayFeedType::Auto;
            host.sseConsecutiveFailures = 0;
            host.consecutiveFailures = 0;
            host.stopRequested.store(false);
            startSseWorker(host);
        } else {
            // Signal the worker to stop and hand its handle out of the lock scope.
            host.stopRequested.store(true);
            if (host.streamThread) toJoin = std::move(host.streamThread);
            host.devices.clear();
            host.knownPortUrls.clear();
            host.activeTransport = RelayFeedType::Auto;
            host.streamConnected.store(false);
            host.serverInstanceId.clear();
            host.lastSnapshotId = 0;
            host.lastError.clear();
            host.consecutiveFailures = 0;
            host.sseConsecutiveFailures = 0;
        }
    }

    if (toJoin && toJoin->joinable()) toJoin->join();
}

std::vector<RelayPortFactory::RelayHostStatus> RelayPortFactory::getRelayHosts() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<RelayHostStatus> result;
    result.reserve(relayHosts_.size());
    for (const auto& [url, hostPtr] : relayHosts_) {
        const RelayHost& host = *hostPtr;
        RelayHostStatus status;
        status.url = host.url;
        status.enabled = host.enabled;
        status.viaMdns = host.viaMdns;
        status.lastPollTime = host.lastPollTime;
        status.lastEventTime = host.lastEventTime;
        status.lastError = host.lastError;
        status.consecutiveFailures = host.consecutiveFailures;
        status.deviceCount = host.devices.size();
        status.feedType = host.activeTransport;
        status.streamConnected = host.streamConnected.load();
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

    std::regex regexPattern;
    try {
        regexPattern = std::regex(pattern);
    } catch (const std::regex_error&) {
        return;
    }

    for (const auto& [url, hostPtr] : relayHosts_) {
        const RelayHost& host = *hostPtr;
        if (!host.enabled)
            continue;
        for (const auto& portUrl : host.knownPortUrls) {
            if (std::regex_match(portUrl, regexPattern)) {
                portCallback(this, PORT_TYPE__TCP | PORT_TYPE__COMM | pType, portUrl);
            }
        }
    }
}

bool RelayPortFactory::validatePort(const std::string& pName, uint16_t pType) {
    tick();
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if ((pType & PORT_TYPE__TCP) != PORT_TYPE__TCP)
        return false;

    for (const auto& [url, hostPtr] : relayHosts_) {
        const RelayHost& host = *hostPtr;
        if (!host.enabled)
            continue;
        if (host.knownPortUrls.count(pName))
            return true;
    }
    return false;
}

port_handle_t RelayPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    auto port = TcpPortFactory::getInstance().bindPort(pName, pType);
    if (port) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        for (const auto& [url, hostPtr] : relayHosts_) {
            const RelayHost& host = *hostPtr;
            if (!host.enabled) continue;
            for (const auto& device : host.devices) {
                if (device.portUrl == pName) {
                    DeviceManager::getInstance().seedDeviceHint(port, device.hint);
                    return port;
                }
            }
        }
    }
    return port;
}

bool RelayPortFactory::releasePort(port_handle_t port) {
    // See the long comment in SN-7719 — intentional: no clearDeviceHint() call during shutdown.
    return TcpPortFactory::getInstance().releasePort(port);
}

// ============================================================
// tick() — mDNS refresh + polling-mode hosts
// ============================================================

void RelayPortFactory::tick() {
    auto& self = getInstance();
    std::lock_guard<std::recursive_mutex> lock(self.mutex_);

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - self.lastMdnsQueryTime_);
    if (elapsed.count() > MDNS_QUERY_INTERVAL_MS) {
        mdns::sendQuery(MDNS_RECORDTYPE_PTR, "_inertialsense-discovery._tcp.local");
        self.lastMdnsQueryTime_ = now;
    }
    mdns::tick();

    self.discoverRelayHostsViaMdns();

    // Only poll hosts that have fallen back to Polling transport.
    // SSE hosts drive their own state via the worker thread.
    for (auto& [url, hostPtr] : self.relayHosts_) {
        RelayHost& host = *hostPtr;
        if (!host.enabled) continue;
        if (host.activeTransport != RelayFeedType::Polling) continue;
        auto sincePoll = std::chrono::duration_cast<std::chrono::milliseconds>(now - host.lastPollTime);
        if (sincePoll >= self.pollInterval_ || host.lastPollTime == std::chrono::steady_clock::time_point{}) {
            self.pollRelayHost(host);
        }
    }
}

// ============================================================
// mDNS relay-host discovery (honors SN-7804 http_port TXT key)
// ============================================================

void RelayPortFactory::discoverRelayHostsViaMdns() {
    // mutex_ is already held by tick()

    auto ptrRecords = mdns::getRecords([](const mdns::mdns_record_cpp_t& r) {
        return r.type == MDNS_RECORDTYPE_PTR && r.name == "_inertialsense-discovery._tcp.local.";
    });

    std::set<std::string> seenHostnames;
    for (const auto& ptr : ptrRecords) {
        auto srvRecords = mdns::getRecords([&ptr](const mdns::mdns_record_cpp_t& r) {
            return r.type == MDNS_RECORDTYPE_SRV && r.name == ptr.data.ptr.name;
        });
        if (srvRecords.empty()) continue;

        std::string hostname = srvRecords[0].data.srv.name;
        if (!hostname.empty() && hostname.back() == '.') hostname.pop_back();

        if (hostname.empty() || !seenHostnames.insert(hostname).second)
            continue;

        // SN-7804: honor the `http_port=N` TXT key on the same service instance.
        // Fall back to DEFAULT_HTTP_PORT for older bridgeboards that don't publish it.
        uint16_t httpPort = DEFAULT_HTTP_PORT;
        auto txtRecords = mdns::getRecords([&ptr](const mdns::mdns_record_cpp_t& r) {
            return r.type == MDNS_RECORDTYPE_TXT && r.name == ptr.data.ptr.name;
        });
        for (const auto& txt : txtRecords) {
            if (txt.data.txt.key == "http_port") {
                try {
                    int parsed = std::stoi(txt.data.txt.valueAsString());
                    if (parsed > 0 && parsed < 65536) httpPort = static_cast<uint16_t>(parsed);
                } catch (...) {}
                break;
            }
        }

        std::string url = "http://" + hostname + ":" + std::to_string(httpPort);
        if (relayHosts_.find(url) == relayHosts_.end()) {
            auto host = std::make_unique<RelayHost>();
            host->url = url;
            host->enabled = false;
            host->viaMdns = true;
            relayHosts_[url] = std::move(host);
            log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: discovered relay host via mDNS: '%s'", url.c_str());
        }
    }
}

// ============================================================
// Polling transport (fallback path)
// ============================================================

void RelayPortFactory::pollRelayHost(RelayHost& host) {
    // mutex_ is held by tick() (caller).
    auto [hostname, port] = splitBaseUrl(host.url, DEFAULT_HTTP_PORT);

    httplib::Client client(hostname, port);
    client.set_connection_timeout(HTTP_CONNECT_TIMEOUT_S);
    client.set_read_timeout(HTTP_READ_TIMEOUT_S);

    auto res = client.Get(PATH_AVAILABLE_DEVICES);
    if (!res || res->status != 200) {
        host.consecutiveFailures++;
        host.lastError = res ? ("HTTP " + std::to_string(res->status))
                             : httplib::to_string(res.error());
        if (host.consecutiveFailures == DEFAULT_FAILURE_GRACE_COUNT) {
            log_warn(IS_LOG_FACILITY_NONE,
                     "RelayPortFactory: relay host '%s' unreachable after %u polls: %s",
                     host.url.c_str(), host.consecutiveFailures, host.lastError.c_str());
        }
        return;
    }

    json doc;
    try {
        doc = json::parse(res->body);
    } catch (const json::parse_error& e) {
        host.consecutiveFailures++;
        host.lastError = std::string("JSON parse error: ") + e.what();
        return;
    }

    SnapshotResult snap;
    if (!parseSnapshotJson(doc, snap)) {
        host.consecutiveFailures++;
        host.lastError = "Expected SN-7804 availableDevices envelope (missing 'devices' array)";
        return;
    }

    // Detect bridgeboard restart by server_instance_id change; clear high-water mark
    // so stale ports from the prior instance don't linger.
    if (!snap.serverInstanceId.empty() && !host.serverInstanceId.empty() &&
        snap.serverInstanceId != host.serverInstanceId) {
        host.knownPortUrls.clear();
        log_info(IS_LOG_FACILITY_NONE, "RelayPortFactory: relay '%s' restarted (new instance id); resyncing",
                 host.url.c_str());
    }
    host.serverInstanceId = snap.serverInstanceId;
    host.lastSnapshotId = snap.snapshotId;

    host.devices = std::move(snap.devices);
    host.lastPollTime = std::chrono::steady_clock::now();
    host.lastError.clear();
    host.consecutiveFailures = 0;

    // Polling can't distinguish "device offline" from "brief hiccup" — keep high-water mark.
    for (const auto& device : host.devices) {
        host.knownPortUrls.insert(device.portUrl);
    }

    log_debug(IS_LOG_FACILITY_NONE, "RelayPortFactory: polled '%s' — %zu devices, %zu known ports",
              host.url.c_str(), host.devices.size(), host.knownPortUrls.size());
}

// ============================================================
// SSE transport (primary push path)
// ============================================================

void RelayPortFactory::startSseWorker(RelayHost& host) {
    // mutex_ is held by caller (setRelayHostEnabled).
    if (host.streamThread) {
        // Defensive — a prior worker should have been torn down by disable/remove.
        return;
    }
    host.stopRequested.store(false);
    host.streamThread = std::make_unique<std::thread>([this, &host]() {
        this->sseWorkerLoop(host);
    });
}

void RelayPortFactory::stopSseWorker(RelayHost& host) {
    // Called with mutex_ held or not — safe either way: we only signal here.
    // Joining must happen outside the lock (see setRelayHostEnabled / removeRelayHost / dtor).
    if (host.streamThread) host.stopRequested.store(true);
}

// SSE worker body. Owns its own I/O; updates host state under mutex_ per event.
// The worker thread itself never holds mutex_ while blocked in network I/O.
void RelayPortFactory::sseWorkerLoop(RelayHost& host) {
    auto [hostname, port] = splitBaseUrl(host.url, DEFAULT_HTTP_PORT);

    // -- Step 1: one-shot initial snapshot so ports surface with minimum latency ----
    // Mirrors what polling would have produced on the first poll.
    {
        httplib::Client snapClient(hostname, port);
        snapClient.set_connection_timeout(HTTP_CONNECT_TIMEOUT_S);
        snapClient.set_read_timeout(HTTP_READ_TIMEOUT_S);
        auto res = snapClient.Get(PATH_AVAILABLE_DEVICES);
        if (res && res->status == 200) {
            try {
                auto doc = json::parse(res->body);
                SnapshotResult snap;
                if (parseSnapshotJson(doc, snap)) {
                    std::lock_guard<std::recursive_mutex> lock(mutex_);
                    host.serverInstanceId = snap.serverInstanceId;
                    host.lastSnapshotId = snap.snapshotId;
                    host.devices = std::move(snap.devices);
                    for (const auto& d : host.devices) host.knownPortUrls.insert(d.portUrl);
                    host.lastEventTime = std::chrono::steady_clock::now();
                    host.lastError.clear();
                    log_debug(IS_LOG_FACILITY_NONE,
                              "RelayPortFactory: relay '%s' initial snapshot — %zu devices",
                              host.url.c_str(), host.devices.size());
                }
            } catch (const json::parse_error&) {
                // Fall through — the first SSE frame should be a snapshot anyway.
            }
        }
    }

    // -- Step 2: persistent SSE loop --------------------------------------------
    int backoffMs = SSE_RECONNECT_INITIAL_MS;
    while (!host.stopRequested.load()) {
        httplib::Client sseClient(hostname, port);
        sseClient.set_connection_timeout(HTTP_CONNECT_TIMEOUT_S);
        sseClient.set_read_timeout(SSE_READ_TIMEOUT_S);

        httplib::Headers headers;
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            if (!host.serverInstanceId.empty() && host.lastSnapshotId > 0) {
                headers.emplace("Last-Event-ID",
                                host.serverInstanceId + ":" + std::to_string(host.lastSnapshotId));
            }
        }

        // Per-session SSE parse state (accumulates across chunk callbacks).
        std::string buffer;
        std::string curEvent;
        std::string curId;
        std::string curData;

        auto applyFrame = [this, &host](const std::string& evt, const std::string& id, const std::string& data) {
            auto [evtInstance, evtSnapshotId] = splitEventId(id);

            json payload;
            if (!data.empty()) {
                try { payload = json::parse(data); }
                catch (const json::parse_error&) { payload = {}; }
            }

            std::lock_guard<std::recursive_mutex> lock(mutex_);

            // server_instance_id change → force fresh snapshot state on all events
            if (!evtInstance.empty() && !host.serverInstanceId.empty() &&
                evtInstance != host.serverInstanceId) {
                host.knownPortUrls.clear();
                host.devices.clear();
                log_info(IS_LOG_FACILITY_NONE,
                         "RelayPortFactory: relay '%s' restarted (SSE id instance changed); resyncing",
                         host.url.c_str());
            }
            if (!evtInstance.empty())      host.serverInstanceId = evtInstance;
            if (evtSnapshotId > host.lastSnapshotId) host.lastSnapshotId = evtSnapshotId;
            host.lastEventTime = std::chrono::steady_clock::now();
            host.lastError.clear();
            // First byte received → SSE connection confirmed
            host.activeTransport = RelayFeedType::SSE;

            if (evt == EVT_SNAPSHOT) {
                SnapshotResult snap;
                if (parseSnapshotJson(payload, snap)) {
                    // Replace devices; rebuild knownPortUrls from snapshot (authoritative resync)
                    host.knownPortUrls.clear();
                    host.devices = std::move(snap.devices);
                    for (const auto& d : host.devices) host.knownPortUrls.insert(d.portUrl);
                }
            } else if (evt == EVT_DEVICE_ADDED || evt == EVT_DEVICE_CHANGED) {
                DeviceRecord rec;
                if (parseDeviceJson(payload, rec)) {
                    // Replace any prior record for the same URI (device.changed), then upsert.
                    auto it = std::find_if(host.devices.begin(), host.devices.end(),
                        [&](const DeviceRecord& d) { return d.portUrl == rec.portUrl; });
                    if (it != host.devices.end()) *it = rec;
                    else                          host.devices.push_back(rec);
                    host.knownPortUrls.insert(rec.portUrl);

                    // If the device is already bound to a port_handle, re-seed the hint so
                    // DeviceManager reflects updated metadata (e.g., a firmware-version change
                    // after a reboot). No-op for ports that aren't currently bound.
                    if (auto handle = PortManager::getInstance().getPort(rec.portUrl, PORT_TYPE__TCP)) {
                        DeviceManager::getInstance().seedDeviceHint(handle, rec.hint);
                    }
                }
            } else if (evt == EVT_DEVICE_REMOVED) {
                std::string uri = payload.is_object() ? payload.value("uri", std::string{}) : std::string{};
                if (!uri.empty()) {
                    host.devices.erase(std::remove_if(host.devices.begin(), host.devices.end(),
                                        [&](const DeviceRecord& d) { return d.portUrl == uri; }),
                                       host.devices.end());
                    host.knownPortUrls.erase(uri);
                    // PortManager will evict on its next sweep via validatePort() returning false.
                }
            }
            // Unknown event types ignored per SSE spec.
        };

        auto on_chunk = [&](const char* data, size_t size) -> bool {
            if (host.stopRequested.load()) return false;
            host.streamConnected.store(true);

            buffer.append(data, size);
            for (;;) {
                size_t nl = buffer.find('\n');
                if (nl == std::string::npos) break;

                std::string line = buffer.substr(0, nl);
                if (!line.empty() && line.back() == '\r') line.pop_back();
                buffer.erase(0, nl + 1);

                if (line.empty()) {
                    // End of frame — dispatch if populated.
                    if (!curEvent.empty() || !curData.empty() || !curId.empty()) {
                        applyFrame(curEvent, curId, curData);
                    }
                    curEvent.clear(); curId.clear(); curData.clear();
                    continue;
                }
                if (line[0] == ':') continue; // comment / keepalive

                size_t colon = line.find(':');
                std::string name  = (colon == std::string::npos) ? line : line.substr(0, colon);
                std::string value = (colon == std::string::npos) ? std::string{} : line.substr(colon + 1);
                if (!value.empty() && value.front() == ' ') value.erase(0, 1);

                if      (name == "event") curEvent = std::move(value);
                else if (name == "id")    curId    = std::move(value);
                else if (name == "data") {
                    if (!curData.empty()) curData.push_back('\n');
                    curData.append(value);
                }
                // other fields (retry: etc.) silently ignored
            }
            return true;
        };

        auto res = sseClient.Get(PATH_EVENTS_DEVICES, headers, on_chunk);
        host.streamConnected.store(false);

        if (host.stopRequested.load()) break;

        // Stream ended unexpectedly — count as a failure, back off, retry. Transition to
        // Polling transport after DEFAULT_MAX_SSE_RETRIES consecutive failures.
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            host.sseConsecutiveFailures++;
            if (res) host.lastError = "SSE HTTP " + std::to_string(res->status);
            else     host.lastError = "SSE: " + httplib::to_string(res.error());

            if (host.sseConsecutiveFailures >= DEFAULT_MAX_SSE_RETRIES) {
                host.activeTransport = RelayFeedType::Polling;
                host.lastError += " — falling back to polling";
                log_warn(IS_LOG_FACILITY_NONE,
                         "RelayPortFactory: relay '%s' SSE failed %u times; switching to polling",
                         host.url.c_str(), host.sseConsecutiveFailures);
                break; // exit loop; tick() takes over via pollRelayHost()
            }
        }

        // Backoff, responsive to stopRequested.
        int slept = 0;
        while (slept < backoffMs && !host.stopRequested.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            slept += 50;
        }
        backoffMs = std::min(backoffMs * 2, SSE_RECONNECT_MAX_MS);
    }
}
