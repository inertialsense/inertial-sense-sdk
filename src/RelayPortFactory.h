/**
 * @file RelayPortFactory.h
 * @brief Discovers IS device ports through remote HTTP-based relay hosts (e.g., bridgeboard, future cltool-as-relay).
 *
 * @author Kyle Mallory on 4/10/26.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__RELAY_PORT_FACTORY_H
#define IS_SDK__RELAY_PORT_FACTORY_H

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "ISConstants.h"
#include "core/base_port.h"
#include "core/tcpPort.h"
#include "PortFactory.h"
#include "data_sets.h"

/**
 * Singleton PortFactory that discovers IS device ports through remote relay hosts.
 *
 * A "relay" is any host that has IS devices physically attached and exposes them to
 * remote SDK consumers over TCP, with an HTTP discovery contract (e.g., GET /api/status).
 * The bridgeboard daemon is the first known relay; future cltool-as-relay and third-party
 * implementations will also work.
 *
 * Relay hosts are discovered via mDNS (_inertialsense-discovery._tcp.local) or added
 * manually. Each host starts as disabled — consumers must explicitly enable the hosts
 * they want to use as port sources. Only enabled hosts are polled and contribute ports
 * to PortManager.
 *
 * Phase 1 (SN-7719): HTTP polling of /api/status at a configurable interval (default 1 Hz).
 * Phase 3 (SN-7805): HTTP polling switched to SN-7804's /api/availableDevices slim snapshot,
 *                    SSE event stream for real-time push (<200 ms add/remove latency),
 *                    per-host polling fallback when SSE isn't available, and mDNS http_port
 *                    TXT key honoring so the factory doesn't assume :8080.
 *
 * Relay-host URLs are canonicalized on entry to "http://<host>:<port>" (scheme + authority,
 * no path). Users may pass any of the following to addRelayHost(): a bare hostname or IP,
 * a host:port pair, a full URL with or without a path — all are normalized to the same
 * storage key. The HTTP paths /api/availableDevices and /api/events/devices are appended
 * by the factory at request time.
 *
 * @code{.cpp}
 * portManager.addPortFactory(&RelayPortFactory::getInstance());
 * RelayPortFactory::getInstance().addRelayHost("http://192.168.1.50:8080");
 * RelayPortFactory::getInstance().setRelayHostEnabled("http://192.168.1.50:8080", true);
 * @endcode
 */
class RelayPortFactory : public PortFactory {
public:

    // -- Singleton --
    static RelayPortFactory& getInstance() {
        static RelayPortFactory instance;
        return instance;
    }

    RelayPortFactory(RelayPortFactory const&) = delete;
    RelayPortFactory& operator=(RelayPortFactory const&) = delete;

    // -- Per-device record parsed from the relay's HTTP response --
    struct DeviceRecord {
        std::string  portUrl;       ///< tcp://host:port — the actual port to bind/connect
        dev_info_t   hint = {};     ///< bridgeboard-authoritative device info for seedDeviceHint()
    };

    /// Active transport in use for a given relay host. Exposed via RelayHostStatus
    /// so UIs can render a badge (and so tests can assert fallback behavior).
    enum class RelayFeedType : uint8_t {
        Auto    = 0, ///< initial / negotiating — not yet connected via any transport
        SSE     = 1, ///< push-based /api/events/devices stream is connected
        Polling = 2, ///< one-shot polling of /api/availableDevices (SSE unavailable or failed)
    };

    // -- Per-host status snapshot (returned by getRelayHosts()) --
    struct RelayHostStatus {
        std::string  url;                                           ///< http://host:port (canonical base URL; no path)
        bool         enabled = false;                               ///< whether this host contributes ports
        bool         viaMdns = false;                               ///< true if discovered via mDNS, false if manually added
        std::chrono::steady_clock::time_point lastPollTime = {};    ///< time of last successful poll (Polling mode)
        std::chrono::steady_clock::time_point lastEventTime = {};   ///< time of last SSE event received (SSE mode)
        std::string  lastError;                                     ///< empty on success; descriptive string on failure
        uint32_t     consecutiveFailures = 0;                       ///< reset to 0 on each successful poll/event
        size_t       deviceCount = 0;                               ///< number of devices reported by this host
        RelayFeedType feedType = RelayFeedType::Auto;               ///< active transport in use
        bool         streamConnected = false;                       ///< true iff an SSE stream is currently open
    };

    // -- Service-style management API --

    /**
     * Manually register a relay host URL (e.g., "http://192.168.1.50:8080/api/status").
     * Hosts added this way are marked viaMdns=false and start enabled=false.
     */
    void addRelayHost(const std::string& url);

    /**
     * Remove a relay host (manual or mDNS-discovered). Returns true if the host was found and removed.
     */
    bool removeRelayHost(const std::string& url);

    /**
     * Enable or disable a known relay host. Disabled hosts are still listed by getRelayHosts()
     * but do not contribute ports to PortManager.
     */
    void setRelayHostEnabled(const std::string& url, bool enabled);

    /**
     * Returns the current set of known relay hosts (mDNS-discovered + manually added) with per-host status.
     * Thread-safe (snapshot under mutex).
     */
    std::vector<RelayHostStatus> getRelayHosts() const;

    // -- PortFactory interface --
    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;
    bool releasePort(port_handle_t port) override;

    /**
     * External polling driver — same pattern as ISmDnsPortFactory::tick().
     * Drives mDNS host discovery refresh and HTTP polling for all enabled hosts.
     * Rate-limited internally per the configured polling interval.
     * Call from cltool's main loop, EvalTool's event loop, or wherever periodic work is driven.
     */
    static void tick();

    /**
     * Set the HTTP polling interval for enabled hosts.
     * @param interval polling period (default 1 second)
     */
    void setPollInterval(std::chrono::milliseconds interval) { pollInterval_ = interval; }
    std::chrono::milliseconds getPollInterval() const { return pollInterval_; }

    /// Default HTTP port assumed for mDNS-discovered hosts (bridgeboard default).
    static constexpr uint16_t DEFAULT_HTTP_PORT = 8080;

    /// Default number of consecutive failures before logging a warning (ports are retained).
    static constexpr uint32_t DEFAULT_FAILURE_GRACE_COUNT = 3;

    /// Default SSE retry budget — after this many consecutive connect/read failures,
    /// the host falls back to polling transport for the remainder of its enabled lifetime.
    static constexpr uint32_t DEFAULT_MAX_SSE_RETRIES = 3;

    /// mDNS query interval (ms) — matches ISmDnsPortFactory's rate
    static constexpr int64_t MDNS_QUERY_INTERVAL_MS = 200;

private:
    RelayPortFactory() = default;
    ~RelayPortFactory();

    // -- Internal per-host state --
    struct RelayHost {
        std::string  url;                                           ///< canonical "http://host:port" (no path)
        bool         enabled = false;
        bool         viaMdns = false;
        std::vector<DeviceRecord> devices;                          ///< latest poll/snapshot result (metadata + hints)
        std::set<std::string> knownPortUrls;                        ///< high-water mark of tcp:// URLs ever seen from this host.
                                                                    ///< Only cleared on host disable/remove. Ports persist across
                                                                    ///< device reboots because bridgeboard's slot-based TCP sockets
                                                                    ///< survive device resets (WaitRecover keeps the listener open).
        std::chrono::steady_clock::time_point lastPollTime = {};    ///< last successful poll (Polling mode)
        std::chrono::steady_clock::time_point lastEventTime = {};   ///< last SSE event received (SSE mode)
        std::string  lastError;
        uint32_t     consecutiveFailures = 0;                       ///< polling failure counter (resets on success)

        // -- SSE worker state (populated while feedType == SSE) --
        std::string  serverInstanceId;                              ///< remote bridgeboard's UUID (restart detection)
        uint64_t     lastSnapshotId = 0;                            ///< numeric half of <instance_id>:<snapshot_id> for Last-Event-ID
        std::unique_ptr<std::thread> streamThread;                  ///< worker running the /api/events/devices reader
        std::atomic<bool> stopRequested{false};                     ///< asks the SSE worker to exit cleanly
        std::function<void()> sseAbortHook;                         ///< set by worker; aborts the current httplib request (unblocks recv)
        std::atomic<bool> streamConnected{false};                   ///< true while the SSE stream is open
        uint32_t     sseConsecutiveFailures = 0;                    ///< drives fallback to Polling after DEFAULT_MAX_SSE_RETRIES

        // -- Transport mode --
        RelayFeedType activeTransport = RelayFeedType::Auto;

        RelayHost() = default;
        // Non-copyable/movable because of the thread member.
        RelayHost(const RelayHost&) = delete;
        RelayHost& operator=(const RelayHost&) = delete;
        RelayHost(RelayHost&&) = delete;
        RelayHost& operator=(RelayHost&&) = delete;
    };

    /// Hosts are keyed by canonical URL. std::map gives stable iterator/reference semantics,
    /// which matters because the SSE worker holds a reference to its RelayHost for the
    /// lifetime of the stream. unique_ptr keeps RelayHost move-stable on container growth.
    std::map<std::string, std::unique_ptr<RelayHost>> relayHosts_;
    mutable std::recursive_mutex mutex_;
    std::chrono::milliseconds pollInterval_ = std::chrono::seconds(1);
    std::chrono::steady_clock::time_point lastMdnsQueryTime_ = {}; ///< rate-limit mDNS queries

    /// Rate-limited mDNS host discovery (reads from shared mdns:: cache).
    void discoverRelayHostsViaMdns();

    /// Poll a single relay host's /api/availableDevices endpoint (fallback transport).
    void pollRelayHost(RelayHost& host);

    /// Start the SSE worker thread for a host (call with mutex_ held; releases during I/O).
    void startSseWorker(RelayHost& host);

    /// Ask the SSE worker to stop and join it. Safe to call with mutex_ held.
    void stopSseWorker(RelayHost& host);

    /// SSE worker body — runs on host.streamThread. Owns its own I/O and respects stopRequested.
    void sseWorkerLoop(RelayHost& host);
};

#endif // IS_SDK__RELAY_PORT_FACTORY_H
