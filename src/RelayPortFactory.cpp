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
#include "util/util.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <set>
#include <thread>

// cpp-httplib and nlohmann/json — used only in this .cpp, not exposed via the header.
#include "httplib.h"
#include "json.hpp"

using json = nlohmann::json;

namespace {

/**
 * Map bridgeboard's module "state" string to a full encoded is_hardware_t (type + major + minor).
 *
 * @param state  bridgeboard module state ("imx5", "imx6", "gpx", "isbl", ...)
 * @return the encoded hardware id, or IS_HARDWARE_NONE if unrecognized
 */
is_hardware_t stateToHardwareId(const std::string& state) {
    if (state == "imx5") return IS_HARDWARE_IMX_5_0;
    if (state == "imx6") return IS_HARDWARE_IMX_6_0;
    if (state == "gpx")  return IS_HARDWARE_GPX_1_0;
    if (state == "isbl") return ENCODE_HDW_ID(IS_HARDWARE_TYPE_UINS, 0, 0); // bootloader — type ambiguous
    return IS_HARDWARE_NONE;
}

/**
 * Canonicalize any relay input (bare hostname, IP, full URL, URL-with-path) into
 * the factory's storage key form: "http://<host>:<port>" with no trailing slash and no path.
 *
 * Examples:
 *   "http://host.local:8080/api/status"   -> "http://host.local:8080"
 *   "http://host.local:9090/"             -> "http://host.local:9090"
 *   "http://host.local"                   -> "http://host.local:8080"   (default port applied)
 *   "host.local:9090"                     -> "http://host.local:9090"   (scheme defaulted)
 *   "10.1.2.3"                            -> "http://10.1.2.3:8080"
 *
 * @param input        relay address in any of the accepted forms
 * @param defaultPort  port to apply when the input omits one
 * @return the canonical "http://host:port", or an empty string on unparseable input
 */
std::string normalizeBaseUrl(const std::string& input, uint16_t defaultPort) {
    std::string s = input;
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))  s.pop_back();
    if (s.empty()) return {};

    if (s.find("://") == std::string::npos) {
        s = "http://" + s;
    }

    const utils::UriParts parsed = utils::parseUri(s);
    if (!parsed.hasHost()) return {};

    int port = parsed.hasPort() ? parsed.port : defaultPort;
    return "http://" + parsed.host + ":" + std::to_string(port);
}

/**
 * Split a canonical "http://host:port" URL into hostname + port.
 *
 * @param baseUrl      canonical relay base URL
 * @param defaultPort  port returned when the URL omits or has an invalid port
 * @return {hostname, port}; {"", defaultPort} on failure
 */
std::pair<std::string, int> splitBaseUrl(const std::string& baseUrl, uint16_t defaultPort) {
    const utils::UriParts parsed = utils::parseUri(baseUrl);
    int port = parsed.hasPort() ? parsed.port : defaultPort;
    return {parsed.host, port};
}

/**
 * Rewrite the host component of a device URI to @p newHost, preserving scheme + port.
 *
 * A bridgeboard relays its OWN locally-attached devices, so every device URI it reports
 * is on the same host we already connected to for the relay's HTTP/SSE API. The relayed
 * URIs, however, frequently use the bridgeboard's mDNS `.local` name, which is only
 * resolvable on the bridgeboard's own link. A client reaching the relay over routed/VPN
 * transport (e.g. Tailscale) connected via the relay's configured host but cannot resolve
 * the `.local` form at all — so binding those ports blocks in nss-mdns and then fails,
 * and no ports ever surface (SN-8175). Substituting the relay's known-reachable host (the
 * one normalizeBaseUrl already validated and the SSE/poll client connected to) makes the
 * device ports resolvable exactly the way the relay itself was reached.
 *
 * @param uri      the relayed device URI (e.g. "tcp://host.local:34663")
 * @param newHost  the relay's reachable host to substitute in
 * @return the rewritten URI, or @p uri unchanged if it can't be parsed or @p newHost is empty
 */
std::string rewriteUriHost(const std::string& uri, const std::string& newHost) {
    if (newHost.empty()) return uri;
    const utils::UriParts parsed = utils::parseUri(uri);
    if (!parsed.hasScheme()) return uri;
    std::string out = parsed.scheme + "://" + newHost;
    if (parsed.hasPort()) out += ":" + std::to_string(parsed.port);
    return out;
}

/**
 * HTTP timeouts (seconds).
 * SN-8177: keep the connect timeout short — a dead/unreachable relay must not block the
 * IO thread for long per attempt (a powered-off host over VPN won't RST; it just hangs).
 */
static constexpr int HTTP_CONNECT_TIMEOUT_S = 2;
static constexpr int HTTP_READ_TIMEOUT_S    = 5;
/** SSE read timeout — generous enough to outlive server keepalive (15 s per SN-7804) */
static constexpr int SSE_READ_TIMEOUT_S     = 30;
/** SSE reconnect backoff (ms) — bounded exponential */
static constexpr int SSE_RECONNECT_INITIAL_MS = 250;
static constexpr int SSE_RECONNECT_MAX_MS     = 2000;

/**
 * Parse a "YYYY-MM-DD HH:MM:SS" timestamp (as bridgeboard's build_date field) into
 * the dev_info_t date/time byte fields. Missing or malformed input leaves everything zero.
 *
 * @param s          the build_date string
 * @param[out] hint  dev_info_t whose build* fields are populated
 */
void parseBuildDate(const std::string& s, dev_info_t& hint) {
    int y = 0, mo = 0, d = 0, h = 0, mi = 0, se = 0;
    if (std::sscanf(s.c_str(), "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &se) < 3)
        return;
    if (y >= 2000 && y < 2000 + 256) hint.buildYear = static_cast<uint8_t>(y - 2000);
    if (mo > 0 && mo < 13) hint.buildMonth = static_cast<uint8_t>(mo);
    if (d > 0 && d < 32)   hint.buildDay   = static_cast<uint8_t>(d);
    if (h >= 0 && h < 24)  hint.buildHour  = static_cast<uint8_t>(h);
    if (mi >= 0 && mi < 60) hint.buildMinute = static_cast<uint8_t>(mi);
    if (se >= 0 && se < 60) hint.buildSecond = static_cast<uint8_t>(se);
}

/**
 * Parse the first hex word of bridgeboard's firmware_commit ("deadbeef abc123.0") into
 * repoRevision. Non-hex input leaves it zero.
 *
 * @param s          the firmware_commit string
 * @param[out] hint  dev_info_t whose repoRevision is populated
 */
void parseRepoRevision(const std::string& s, dev_info_t& hint) {
    try {
        hint.repoRevision = static_cast<uint32_t>(std::stoul(s, nullptr, 16));
    } catch (...) {}
}

/**
 * Parse a single device entry from the SN-7804 `/api/availableDevices` schema
 * (or from a `device.added` / `device.changed` SSE event payload — same shape).
 *
 * Populates every dev_info_t field the relay can supply so consumers don't need to
 * issue a follow-up DID_DEV_INFO query: hardwareType/Ver, serialNumber, full
 * protocolVer[], firmwareVer, manufacturer, build date/time, repo revision.
 *
 * @param dev        the JSON device object to parse
 * @param[out] out   the populated DeviceRecord; its portUrl is rebased onto @p relayHost
 * @param relayHost  the relay's reachable host, substituted for the device URI's (often
 *                   `.local`) host so the port resolves off-link (see rewriteUriHost / SN-8175)
 * @return true if the entry is a usable device, false if it should be skipped
 */
bool parseDeviceJson(const json& dev, RelayPortFactory::DeviceRecord& out, const std::string& relayHost) {
    if (!dev.is_object()) return false;

    std::string state = dev.value("state", "none");
    if (state == "none" || state == "cdc" || state == "dfu") return false;

    std::string uri = dev.value("uri", "");
    if (uri.empty()) return false;

    dev_info_t hint = {};

    // Prefer the consolidated "hdw" string (e.g. "IMX-5.0", "GPX-1.0.2") when present — it
    // preserves the cached pre-ISBL identity, so we can still tell IMX-5/IMX-6/GPX apart
    // when state == "isbl". Older bridgeboards don't emit it, so fall back to state mapping.
    std::string hdwStr = dev.value("hdw", "");
    if (hdwStr.empty() || !utils::parseHardwareFromString(hdwStr, hint)) {
        is_hardware_t hdwId = stateToHardwareId(state);
        hint.hardwareType = DECODE_HDW_TYPE(hdwId);
        hint.hardwareVer[0] = DECODE_HDW_MAJOR(hdwId);
        hint.hardwareVer[1] = DECODE_HDW_MINOR(hdwId);
    }
    hint.hdwRunState = (state == "isbl") ? HDW_STATE_BOOTLOADER : HDW_STATE_APP;
    hint.serialNumber = dev.value("serial_number", 0u);

    // Protocol version is the SDK's compile-time constant; populating all four
    // components keeps version-compatibility checks consistent with what a real
    // probe response would report. Auto-OPEN side effects are gated separately:
    // hint-driven device registration (DeviceManager::seedDeviceHint) deliberately
    // never opens the port, so DEVICE_CONNECTED only fires when the user explicitly
    // opens the port via Find/Open.
    hint.protocolVer[0] = PROTOCOL_VERSION_CHAR0;
    hint.protocolVer[1] = PROTOCOL_VERSION_CHAR1;
    hint.protocolVer[2] = PROTOCOL_VERSION_CHAR2;
    hint.protocolVer[3] = PROTOCOL_VERSION_CHAR3;

    std::string fwVer = dev.value("firmware_ver", "");
    if (!fwVer.empty()) utils::parseFirmwareFromString(fwVer, hint);

    std::string fwCommit = dev.value("firmware_commit", "");
    if (!fwCommit.empty()) parseRepoRevision(fwCommit, hint);

    std::string buildDate = dev.value("build_date", "");
    if (!buildDate.empty()) parseBuildDate(buildDate, hint);

    // Manufacturer isn't in the SN-7804 schema — bridgeboard only relays IS devices,
    // so we hard-code a sane default. This matches what real devices report.
    std::strncpy(hint.manufacturer, "Inertial Sense", sizeof(hint.manufacturer) - 1);

    // Device URIs from the relay often carry the bridgeboard's .local host, which a
    // routed/VPN client can't resolve. Rebind to the relay's reachable host (SN-8175).
    out.portUrl = rewriteUriHost(uri, relayHost);
    out.hint = hint;
    return true;
}

/**
 * Parsed form of the SN-7804 snapshot envelope (shared between the `/api/availableDevices`
 * HTTP response and the SSE `snapshot` event payload). Expected shape:
 * `{ "server_instance_id": "...", "snapshot_id": N, "devices": [ ... ] }`
 */
struct SnapshotResult {
    std::string serverInstanceId;
    uint64_t    snapshotId = 0;
    std::vector<RelayPortFactory::DeviceRecord> devices;
};

/**
 * Parse the SN-7804 snapshot envelope into a SnapshotResult.
 *
 * @param doc        the snapshot JSON (HTTP response body or SSE `snapshot` payload)
 * @param[out] out   the parsed envelope: server instance id, snapshot id, and devices
 * @param relayHost  the relay's reachable host, propagated to each device's portUrl
 *                   (see parseDeviceJson / SN-8175)
 * @return true if the envelope was well-formed (had a `devices` array), false otherwise
 */
bool parseSnapshotJson(const json& doc, SnapshotResult& out, const std::string& relayHost) {
    if (!doc.is_object()) return false;

    out.serverInstanceId = doc.value("server_instance_id", "");
    out.snapshotId       = doc.value("snapshot_id", 0ull);

    if (!doc.contains("devices") || !doc["devices"].is_array())
        return false;

    out.devices.clear();
    out.devices.reserve(doc["devices"].size());
    for (const auto& dev : doc["devices"]) {
        RelayPortFactory::DeviceRecord rec;
        if (parseDeviceJson(dev, rec, relayHost))
            out.devices.push_back(std::move(rec));
    }
    return true;
}

/**
 * Split a composite SSE id "<instance_uuid>:<snapshot_id>" into its parts.
 *
 * @param id  the composite event id
 * @return {instance, snapshotId}; {"", 0} on malformed input
 */
std::pair<std::string, uint64_t> splitEventId(const std::string& id) {
    auto colon = id.find(':');
    if (colon == std::string::npos) return {"", 0};
    std::string inst = id.substr(0, colon);
    uint64_t snap = 0;
    try { snap = std::stoull(id.substr(colon + 1)); } catch (...) { return {"", 0}; }
    return {std::move(inst), snap};
}

/** HTTP paths on a relay host (appended to the stored base URL). */
static constexpr const char* PATH_AVAILABLE_DEVICES = "/api/availableDevices";
static constexpr const char* PATH_EVENTS_DEVICES    = "/api/events/devices";

/** SSE event type names emitted by bridgeboard per SN-7804. */
static constexpr const char* EVT_SNAPSHOT        = "snapshot";
static constexpr const char* EVT_DEVICE_ADDED    = "device.added";
static constexpr const char* EVT_DEVICE_CHANGED  = "device.changed";
static constexpr const char* EVT_DEVICE_REMOVED  = "device.removed";

} // anonymous namespace

// ============================================================
// Destructor — tears down all SSE workers cleanly
// ============================================================

RelayPortFactory::~RelayPortFactory() {
    // Collect workers + their abort hooks under the lock, then signal + join OUTSIDE
    // the lock so a worker callback currently blocked acquiring mutex_ can make progress.
    // The abort hook unblocks the worker's blocking httplib recv() so join() doesn't hang
    // waiting for the 15 s server keepalive (or a 30 s read timeout).
    std::vector<std::unique_ptr<std::thread>> threads;
    std::vector<std::function<void()>> abortHooks;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        for (auto& [url, host] : relayHosts_) {
            host->stopRequested.store(true);
            if (host->sseAbortHook)    abortHooks.push_back(host->sseAbortHook);
            if (host->streamThread)    threads.push_back(std::move(host->streamThread));
        }
    }
    for (auto& hook : abortHooks)  if (hook) hook();
    for (auto& t    : threads)     if (t && t->joinable()) t->join();
}

// ============================================================
// Service-style management API
// ============================================================

void RelayPortFactory::addRelayHost(const std::string& url) {
    std::string canonical = normalizeBaseUrl(url, DEFAULT_HTTP_PORT);
    if (canonical.empty()) {
        log_warn(IS_LOG_PORT_FACTORY, "RelayPortFactory: ignored unparseable relay URL '%s'", url.c_str());
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
    log_info(IS_LOG_PORT_FACTORY, "RelayPortFactory: added manual relay host '%s'", canonical.c_str());
}

bool RelayPortFactory::removeRelayHost(const std::string& url) {
    std::string canonical = normalizeBaseUrl(url, DEFAULT_HTTP_PORT);
    if (canonical.empty()) return false;

    // Pull the host out of the map, then tear down its worker OUTSIDE the lock.
    std::unique_ptr<RelayHost> removed;
    std::function<void()> abortHook;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto it = relayHosts_.find(canonical);
        if (it == relayHosts_.end())
            return false;

        it->second->stopRequested.store(true);
        abortHook = it->second->sseAbortHook;
        removed = std::move(it->second);
        relayHosts_.erase(it);
    }
    if (abortHook) abortHook();
    if (removed && removed->streamThread && removed->streamThread->joinable()) {
        removed->streamThread->join();
    }
    log_info(IS_LOG_PORT_FACTORY, "RelayPortFactory: removed relay host '%s'", canonical.c_str());
    return true;
}

void RelayPortFactory::setRelayHostEnabled(const std::string& url, bool enabled) {
    std::string canonical = normalizeBaseUrl(url, DEFAULT_HTTP_PORT);
    if (canonical.empty()) return;

    // Capture the thread to join outside the lock on a disable.
    std::unique_ptr<std::thread> toJoin;
    std::function<void()> abortHook;

    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto it = relayHosts_.find(canonical);
        if (it == relayHosts_.end()) return;

        RelayHost& host = *it->second;
        if (host.enabled == enabled) return;

        host.enabled = enabled;
        log_info(IS_LOG_PORT_FACTORY, "RelayPortFactory: relay host '%s' %s",
                 canonical.c_str(), enabled ? "enabled" : "disabled");

        if (enabled) {
            host.activeTransport = RelayFeedType::Auto;
            host.sseConsecutiveFailures = 0;
            host.consecutiveFailures = 0;
            // Start the offline-eviction / backoff clock now: an enabled host that never
            // makes contact begins its grace period here (SN-8177).
            host.lastContact = std::chrono::steady_clock::now();
            host.lastAttemptTime = {};
            host.stopRequested.store(false);
            startSseWorker(host);
        } else {
            // Signal the worker to stop; hand its handle and abort hook out of the lock scope.
            host.stopRequested.store(true);
            abortHook = host.sseAbortHook;
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

    if (abortHook) abortHook();
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

    std::regex regexPattern;
    try {
        regexPattern = std::regex(pattern);
    } catch (const std::regex_error&) {
        return;
    }

    // Snapshot the matching port URLs under mutex_, then invoke portCallback OUTSIDE the
    // lock. portCallback -> PortManager::portHandler -> bindPort -> TcpPortFactory resolves
    // the port's host via getaddrinfo(); for a relay serving .local device URIs that drops
    // into nss-mdns and can block for seconds. Holding mutex_ across that starves the UI
    // thread's getRelayHosts() and freezes the Port Options dialog (SN-8175). Same
    // "collect under lock, act outside" pattern used by the dtor and setRelayHostEnabled.
    std::vector<std::string> toEmit;
    {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        for (const auto& [url, hostPtr] : relayHosts_) {
            const RelayHost& host = *hostPtr;
            if (!host.enabled)
                continue;
            for (const auto& portUrl : host.knownPortUrls) {
                if (std::regex_match(portUrl, regexPattern)) {
                    toEmit.push_back(portUrl);
                }
            }
        }
    }

    for (const auto& portUrl : toEmit) {
        portCallback(this, PORT_TYPE__TCP | PORT_TYPE__COMM | pType, portUrl);
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

// If we know a hint for this port (it appears in any enabled relay host's device list),
// seed it into DeviceManager. Used by both bindPort (when we win the bind race) and
// onPortAlias (when TcpPortFactory wins for a relay-known URL — the port handle is the
// same TCP port either way; only the hint-seeding side-effect differs).
void RelayPortFactory::seedHintForPortIfKnown(port_handle_t port, const std::string& portUrl) {
    if (!port) return;
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    for (const auto& [url, hostPtr] : relayHosts_) {
        const RelayHost& host = *hostPtr;
        if (!host.enabled) continue;
        for (const auto& device : host.devices) {
            if (device.portUrl == portUrl) {
                DeviceManager::getInstance().seedDeviceHint(port, device.hint);
                return;
            }
        }
    }
}

port_handle_t RelayPortFactory::bindPort(const std::string& pName, uint16_t pType) {
    auto port = TcpPortFactory::getInstance().bindPort(pName, pType);
    seedHintForPortIfKnown(port, pName);
    return port;
}

void RelayPortFactory::onPortAlias(port_handle_t existing, const std::string& pName, uint16_t /*pType*/) {
    seedHintForPortIfKnown(existing, pName);
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

    // Teardown collected under the lock, executed OUTSIDE it: reaped mDNS hosts (whose
    // announcements aged out) keep their RelayHost alive in `reaped` until their SSE worker
    // is joined, so a worker currently blocked acquiring mutex_ can finish first. Mirrors
    // the dtor / removeRelayHost pattern; joining under the lock would deadlock (SN-8177).
    std::vector<std::unique_ptr<RelayHost>> reaped;
    std::vector<std::function<void()>> abortHooks;

    {
        std::lock_guard<std::recursive_mutex> lock(self.mutex_);

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - self.lastMdnsQueryTime_);
        if (elapsed.count() > MDNS_QUERY_INTERVAL_MS) {
            mdns::sendQuery(MDNS_RECORDTYPE_PTR, "_inertialsense-discovery._tcp.local");
            self.lastMdnsQueryTime_ = now;
        }
        mdns::tick();

        self.discoverRelayHostsViaMdns(reaped, abortHooks);

        // SN-8177: evict ports for any enabled host gone silent past the grace window
        // (no SSE bytes incl. keepalive, no successful poll). Clearing devices +
        // knownPortUrls makes validatePort() fail for those ports, so PortManager's next
        // sweep removes them and closes their sockets — including ports the user had open.
        // They reappear from the next snapshot if the host comes back.
        for (auto& [url, hostPtr] : self.relayHosts_) {
            RelayHost& host = *hostPtr;
            if (!host.enabled) continue;
            auto silentMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - host.lastContact).count();
            if (silentMs > OFFLINE_EVICT_MS && (!host.devices.empty() || !host.knownPortUrls.empty())) {
                log_warn(IS_LOG_PORT_FACTORY,
                         "RelayPortFactory: relay '%s' silent %lldms; evicting %zu port(s)",
                         host.url.c_str(), (long long)silentMs, host.knownPortUrls.size());
                host.devices.clear();
                host.knownPortUrls.clear();
                if (host.lastError.empty()) host.lastError = "relay offline; ports evicted";
            }
        }

        // Poll hosts on the Polling transport, gated by an escalating backoff while lost
        // (SN-8177) so a dead host isn't hammered. SSE hosts drive their own reconnect on
        // the worker thread; the eviction pass above still drops their ports when silent.
        for (auto& [url, hostPtr] : self.relayHosts_) {
            RelayHost& host = *hostPtr;
            if (!host.enabled) continue;
            if (host.activeTransport != RelayFeedType::Polling) continue;
            auto interval = self.reconnectInterval(host, now);
            auto sinceAttempt = std::chrono::duration_cast<std::chrono::milliseconds>(now - host.lastAttemptTime);
            if (host.lastAttemptTime == std::chrono::steady_clock::time_point{} || sinceAttempt >= interval) {
                self.pollRelayHost(host);
            }
        }
    }

    // Signal + join reaped mDNS workers outside the lock.
    for (auto& hook : abortHooks) if (hook) hook();
    for (auto& host : reaped) {
        if (host && host->streamThread && host->streamThread->joinable())
            host->streamThread->join();
    }
}

// Escalating reconnect/poll cadence for a host based on how long it's been silent.
std::chrono::milliseconds RelayPortFactory::reconnectInterval(
        const RelayHost& host, std::chrono::steady_clock::time_point now) const {
    auto silentMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - host.lastContact).count();
    // Healthy / recently-lost (still inside the eviction grace): normal cadence so a brief
    // blip recovers quickly.
    if (silentMs < OFFLINE_EVICT_MS) {
        return pollInterval_;
    }
    // Lost: grow ~ one step per minute, capped at LOST_BACKOFF_MAX_MS (reached by ~10 min).
    int64_t minutesLost = silentMs / 60000;
    int64_t backoffMs = 6000 * (minutesLost + 1);   // 6s, 12s, ... -> 60s
    if (backoffMs > LOST_BACKOFF_MAX_MS) backoffMs = LOST_BACKOFF_MAX_MS;
    return std::chrono::milliseconds(backoffMs);
}

// ============================================================
// mDNS relay-host discovery (honors SN-7804 http_port TXT key)
// ============================================================

void RelayPortFactory::discoverRelayHostsViaMdns(
        std::vector<std::unique_ptr<RelayHost>>& reaped,
        std::vector<std::function<void()>>& abortHooks) {
    // mutex_ is already held by tick()

    auto ptrRecords = mdns::getRecords([](const mdns::mdns_record_cpp_t& r) {
        return r.type == MDNS_RECORDTYPE_PTR && r.name == "_inertialsense-discovery._tcp.local.";
    });

    std::set<std::string> seenHostnames;
    std::set<std::string> advertisedUrls;   // canonical URLs currently announced via mDNS
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
        advertisedUrls.insert(url);
        if (relayHosts_.find(url) == relayHosts_.end()) {
            auto host = std::make_unique<RelayHost>();
            host->url = url;
            host->enabled = false;
            host->viaMdns = true;
            relayHosts_[url] = std::move(host);
            log_info(IS_LOG_PORT_FACTORY, "RelayPortFactory: discovered relay host via mDNS: '%s'", url.c_str());
        }
    }

    // SN-8177: reap mDNS-discovered hosts whose announcement has aged out of the cache.
    // mDNS is link-local, so a vanished announcement means the host is genuinely gone —
    // drop it and stop reconnecting (it re-adds itself if it reappears). Guards:
    //   * only viaMdns hosts (manual hosts persist and keep retrying with backoff);
    //   * not while a stream is still connected (reachable by IP despite no mDNS) —
    //     let it be reaped when the connection also drops, not kill a live link;
    //   * only once it's also been silent past the eviction grace, so a transient mDNS
    //     cache gap can't wipe a host that's still in contact.
    // Reaped hosts are moved out (kept alive) and joined OUTSIDE mutex_ by the caller.
    auto now = std::chrono::steady_clock::now();
    for (auto it = relayHosts_.begin(); it != relayHosts_.end(); ) {
        RelayHost& host = *it->second;
        const bool recordGone = host.viaMdns && advertisedUrls.find(host.url) == advertisedUrls.end();
        auto silentMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - host.lastContact).count();
        if (recordGone && !host.streamConnected.load() && silentMs > OFFLINE_EVICT_MS) {
            log_info(IS_LOG_PORT_FACTORY,
                     "RelayPortFactory: mDNS relay host '%s' announcement expired; dropping (%zu port(s))",
                     host.url.c_str(), host.knownPortUrls.size());
            host.stopRequested.store(true);
            if (host.sseAbortHook) abortHooks.push_back(host.sseAbortHook);
            if (host.streamThread) reaped.push_back(std::move(it->second));
            it = relayHosts_.erase(it);
        } else {
            ++it;
        }
    }
}

// ============================================================
// Polling transport (fallback path)
// ============================================================

void RelayPortFactory::pollRelayHost(RelayHost& host) {
    // mutex_ is held by tick() (caller).
    // Stamp the attempt time up front (success or fail) so the backoff cadence in tick()
    // advances even when the host is unreachable (SN-8177).
    host.lastAttemptTime = std::chrono::steady_clock::now();
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
            log_warn(IS_LOG_PORT_FACTORY,
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
    if (!parseSnapshotJson(doc, snap, hostname)) {
        host.consecutiveFailures++;
        host.lastError = "Expected SN-7804 availableDevices envelope (missing 'devices' array)";
        return;
    }

    // Detect bridgeboard restart by server_instance_id change; clear high-water mark
    // so stale ports from the prior instance don't linger.
    if (!snap.serverInstanceId.empty() && !host.serverInstanceId.empty() &&
        snap.serverInstanceId != host.serverInstanceId) {
        host.knownPortUrls.clear();
        log_info(IS_LOG_PORT_FACTORY, "RelayPortFactory: relay '%s' restarted (new instance id); resyncing",
                 host.url.c_str());
    }
    host.serverInstanceId = snap.serverInstanceId;
    host.lastSnapshotId = snap.snapshotId;

    host.devices = std::move(snap.devices);
    host.lastPollTime = std::chrono::steady_clock::now();
    host.lastContact = host.lastPollTime;   // successful poll counts as contact (SN-8177)
    host.lastError.clear();
    host.consecutiveFailures = 0;

    // Polling can't distinguish "device offline" from "brief hiccup" — keep high-water mark.
    for (const auto& device : host.devices) {
        host.knownPortUrls.insert(device.portUrl);
    }

    log_debug(IS_LOG_PORT_FACTORY, "RelayPortFactory: polled '%s' — %zu devices, %zu known ports",
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
//
// Rationale — no separate one-shot /api/availableDevices pre-fetch:
// SN-7804's SSE stream emits a `snapshot` event as its FIRST frame whenever the client
// connects without a matching Last-Event-ID. That frame carries exactly the same payload
// as /api/availableDevices, so a pre-fetch adds latency (an extra round-trip on the same
// socket) without adding information. The SSE worker's `applyFrame` populates devices,
// knownPortUrls, serverInstanceId, and lastSnapshotId from that snapshot event directly.
void RelayPortFactory::sseWorkerLoop(RelayHost& host) {
    auto [hostname, port] = splitBaseUrl(host.url, DEFAULT_HTTP_PORT);

    log_debug(IS_LOG_PORT_FACTORY, "RelayPortFactory: starting SSE loop for '%s'", host.url.c_str());
    int backoffMs = SSE_RECONNECT_INITIAL_MS;
    while (!host.stopRequested.load()) {
        httplib::Client sseClient(hostname, port);
        sseClient.set_connection_timeout(HTTP_CONNECT_TIMEOUT_S);
        sseClient.set_read_timeout(SSE_READ_TIMEOUT_S);

        // Publish an abort hook so a concurrent setRelayHostEnabled(false) / removeRelayHost
        // / ~RelayPortFactory() can unblock the pending recv() without waiting for server silence.
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            host.sseAbortHook = [&sseClient]() { sseClient.stop(); };
        }

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

            // Relay-reachable host for rewriting device URIs off their (possibly unresolvable)
            // .local form — see rewriteUriHost / SN-8175. host.url is fixed at creation.
            const std::string relayHost = splitBaseUrl(host.url, DEFAULT_HTTP_PORT).first;

            // server_instance_id change → force fresh snapshot state on all events
            if (!evtInstance.empty() && !host.serverInstanceId.empty() &&
                evtInstance != host.serverInstanceId) {
                host.knownPortUrls.clear();
                host.devices.clear();
                log_info(IS_LOG_PORT_FACTORY,
                         "RelayPortFactory: relay '%s' restarted (SSE id instance changed); resyncing",
                         host.url.c_str());
            }
            if (!evtInstance.empty())      host.serverInstanceId = evtInstance;
            if (evtSnapshotId > host.lastSnapshotId) host.lastSnapshotId = evtSnapshotId;
            host.lastEventTime = std::chrono::steady_clock::now();
            host.lastContact = host.lastEventTime;   // event = contact (SN-8177)
            host.lastError.clear();
            // First byte received → SSE connection confirmed
            host.activeTransport = RelayFeedType::SSE;

            if (evt == EVT_SNAPSHOT) {
                SnapshotResult snap;
                if (parseSnapshotJson(payload, snap, relayHost)) {
                    // Replace devices; rebuild knownPortUrls from snapshot (authoritative resync)
                    host.knownPortUrls.clear();
                    host.devices = std::move(snap.devices);
                    for (const auto& d : host.devices) host.knownPortUrls.insert(d.portUrl);
                }
            } else if (evt == EVT_DEVICE_ADDED || evt == EVT_DEVICE_CHANGED) {
                DeviceRecord rec;
                if (parseDeviceJson(payload, rec, relayHost)) {
                    // Replace any prior record for the same URI (device.changed), then upsert.
                    auto it = std::find_if(host.devices.begin(), host.devices.end(),
                        [&](const DeviceRecord& d) { return d.portUrl == rec.portUrl; });
                    if (it != host.devices.end()) *it = rec;
                    else                          host.devices.push_back(rec);
                    host.knownPortUrls.insert(rec.portUrl);

                    // NOTE: we intentionally do NOT call into PortManager or DeviceManager
                    // from here. PortManager's discoverPorts/validatePort path acquires
                    // PortManager.mutex and then calls into RelayPortFactory::validatePort
                    // which takes our mutex_ — the inverse lock order. Touching PortManager
                    // here (we used to call getPort + seedDeviceHint for already-bound ports
                    // so device.changed would refresh the hint) races that path and deadlocks
                    // under load. The freshened hint is picked up naturally on the next
                    // bindPort; if a caller needs an updated hint on an already-bound port,
                    // they can rebind, or we can add a deferred-apply queue later.
                }
            } else if (evt == EVT_DEVICE_REMOVED) {
                std::string uri = payload.is_object() ? payload.value("uri", std::string{}) : std::string{};
                // Rewrite to match the stored (relay-host-rebound) portUrls — see SN-8175.
                uri = rewriteUriHost(uri, relayHost);
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
            bool firstChunk = !host.streamConnected.exchange(true);
            if (firstChunk) {
                log_debug(IS_LOG_PORT_FACTORY, "RelayPortFactory: SSE stream connected to '%s' (%zu bytes first chunk)",
                          host.url.c_str(), size);
            }

            // Any bytes — including SSE `:` keepalive comments that never reach applyFrame —
            // count as contact, so a healthy idle stream isn't falsely evicted (SN-8177).
            {
                std::lock_guard<std::recursive_mutex> lock(mutex_);
                host.lastContact = std::chrono::steady_clock::now();
            }

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

        // Clear the abort hook now that the client is about to be destroyed —
        // prevents use-after-free if the shutdown path races here.
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);
            host.sseAbortHook = nullptr;
        }

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
                log_warn(IS_LOG_PORT_FACTORY,
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
