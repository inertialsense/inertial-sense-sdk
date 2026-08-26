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
    /** @return the process-wide singleton RelayPortFactory instance. */
    static RelayPortFactory& getInstance() {
        static RelayPortFactory instance;
        return instance;
    }

    RelayPortFactory(RelayPortFactory const&) = delete;
    RelayPortFactory& operator=(RelayPortFactory const&) = delete;

    // -- Per-device record parsed from the relay's HTTP response --
    /** A single device entry parsed from a relay host's /api/availableDevices response or SSE snapshot. */
    struct DeviceRecord {
        std::string  portUrl;       //!< tcp://host:port — the actual port to bind/connect
        dev_info_t   hint = {};     //!< bridgeboard-authoritative device info for seedDeviceHint()
    };

    /**
     * Active transport in use for a given relay host. Exposed via RelayHostStatus
     * so UIs can render a badge (and so tests can assert fallback behavior).
     */
    enum class RelayFeedType : uint8_t {
        Auto    = 0, //!< initial / negotiating — not yet connected via any transport
        SSE     = 1, //!< push-based /api/events/devices stream is connected
        Polling = 2, //!< one-shot polling of /api/availableDevices (SSE unavailable or failed)
    };

    // -- Per-host status snapshot (returned by getRelayHosts()) --
    /** A point-in-time snapshot of a single relay host's configuration and connection status. */
    struct RelayHostStatus {
        std::string  url;                                           //!< http://host:port (canonical base URL; no path)
        bool         enabled = false;                               //!< whether this host contributes ports
        bool         viaMdns = false;                               //!< true if discovered via mDNS, false if manually added
        std::chrono::steady_clock::time_point lastPollTime = {};    //!< time of last successful poll (Polling mode)
        std::chrono::steady_clock::time_point lastEventTime = {};   //!< time of last SSE event received (SSE mode)
        std::string  lastError;                                     //!< empty on success; descriptive string on failure
        uint32_t     consecutiveFailures = 0;                       //!< reset to 0 on each successful poll/event
        size_t       deviceCount = 0;                               //!< number of devices reported by this host
        RelayFeedType feedType = RelayFeedType::Auto;               //!< active transport in use
        bool         streamConnected = false;                       //!< true iff an SSE stream is currently open
    };

    // -- Service-style management API --

    /**
     * Manually register a relay host URL (e.g., "http://192.168.1.50:8080/api/status").
     * Hosts added this way are marked viaMdns=false and start enabled=false.
     *
     * @param url  relay URL in any accepted form (bare host, host:port, or full URL);
     *             normalized internally to the canonical "http://host:port" key
     */
    void addRelayHost(const std::string& url);

    /**
     * Remove a relay host (manual or mDNS-discovered) and tear down its SSE worker.
     *
     * @param url  relay URL identifying the host (normalized internally)
     * @return true if the host was found and removed, false if it was not known
     */
    bool removeRelayHost(const std::string& url);

    /**
     * Enable or disable a known relay host. Disabled hosts are still listed by
     * getRelayHosts() but do not contribute ports to PortManager.
     *
     * @param url      relay URL identifying the host (normalized internally)
     * @param enabled  true to start contributing ports (spawns the SSE worker),
     *                 false to stop and release the host's ports
     */
    void setRelayHostEnabled(const std::string& url, bool enabled);

    /**
     * Returns the current set of known relay hosts (mDNS-discovered + manually added)
     * with per-host status. Thread-safe (snapshot taken under the factory mutex).
     *
     * @return a RelayHostStatus snapshot for every known relay host
     */
    std::vector<RelayHostStatus> getRelayHosts() const;

    /**
     * Turns automatic mDNS relay-host discovery on or off. Enabled by default.
     *
     * Disabling it stops tick() both announcing interest and acting on answers: no
     * "_inertialsense-discovery._tcp.local" query is sent, and no host is added, enabled or reaped on
     * the strength of one. Only hosts given to addRelayHost() are ever known.
     *
     * This exists for consumers that must not touch hardware they were not pointed at. Leaving
     * discovery on and declining to enable what it finds is not equivalent: the query still solicits
     * responses from every fixture on the network, and hosts still appear in getRelayHosts(). A
     * consumer sharing a network with manufacturing or calibration equipment wants the query never
     * sent, which is what this switches off.
     *
     * Disabling also drops any viaMdns hosts already known. They cannot be left in place: the only
     * code that reaps them lives inside the discovery pass this switches off, so they would otherwise
     * persist for the process's lifetime with no way to age out. Manually added hosts are untouched.
     *
     * @param enabled false to stop querying, stop acting on announcements, and drop what mDNS found
     */
    void setMdnsDiscoveryEnabled(bool enabled);

    /** @return true if automatic mDNS relay-host discovery is active (the default). */
    bool isMdnsDiscoveryEnabled() const;

    // ============================================================
    // PortFactory interface
    // ============================================================

    /**
     * Emit every port known from enabled relay hosts whose URL matches @p pattern.
     *
     * @param portCallback  invoked once per matching port (factory, type flags, tcp:// URL)
     * @param pattern       regex matched against each known relay port URL
     * @param pType         caller's requested port-type flags, OR'd into the emitted type
     */
    void locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType) override;

    /**
     * @param pName  candidate tcp:// port URL
     * @param pType  port-type flags; must include PORT_TYPE__TCP
     * @return true if @p pName is a TCP port currently known to an enabled relay host
     */
    bool validatePort(const std::string& pName, uint16_t pType = 0) override;

    /**
     * Bind a relay-known TCP port (delegated to TcpPortFactory) and seed its device hint.
     *
     * @param pName  tcp:// port URL to bind
     * @param pType  port-type flags
     * @return the bound port handle, or nullptr if validation or binding failed
     */
    port_handle_t bindPort(const std::string& pName, uint16_t pType = 0) override;

    /**
     * Release a port previously returned by bindPort().
     *
     * @param port  the port handle to release
     * @return true on success
     */
    bool releasePort(port_handle_t port) override;

    /**
     * PortManager calls this when our locatePorts() emit was deduped against an existing
     * port (typically TcpPortFactory got there first for a relay-known tcp:// URL). The
     * existing port handle is fine; we just need to seed our device hint into
     * DeviceManager so beginValidation can use it. Does NOT create a new port.
     *
     * @param existing  the already-bound port handle our emitted URL aliased onto
     * @param pName     the tcp:// URL we emitted (matches an enabled host's device)
     * @param pType     port-type flags (unused; the alias target's type is authoritative)
     */
    void onPortAlias(port_handle_t existing, const std::string& pName, uint16_t pType) override;

private:
    /**
     * Internal helper: if @p portUrl appears in any enabled relay host's device list,
     * seed that device's hint into DeviceManager keyed by @p port. Shared by
     * bindPort() and onPortAlias().
     *
     * @param port     the bound port handle to associate the hint with
     * @param portUrl  the tcp:// URL whose relay device hint should be seeded
     */
    void seedHintForPortIfKnown(port_handle_t port, const std::string& portUrl);

public:

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
    /** @return the currently configured HTTP polling interval for enabled hosts. */
    std::chrono::milliseconds getPollInterval() const { return pollInterval_; }

    /** Default HTTP port assumed for mDNS-discovered hosts (bridgeboard default). */
    static constexpr uint16_t DEFAULT_HTTP_PORT = 8080;

    /** Default number of consecutive failures before logging a warning (ports are retained). */
    static constexpr uint32_t DEFAULT_FAILURE_GRACE_COUNT = 3;

    /**
     * Default SSE retry budget — after this many consecutive connect/read failures,
     * the host falls back to polling transport for the remainder of its enabled lifetime.
     */
    static constexpr uint32_t DEFAULT_MAX_SSE_RETRIES = 3;

    /**
     * SN-8177: a host with no contact (no SSE bytes incl. keepalive, no successful poll)
     * for this long is considered offline — its ports are evicted (PortManager closes +
     * removes them on its next sweep). Ports reappear from the next snapshot if it returns.
     */
    static constexpr int64_t OFFLINE_EVICT_MS = 30000;

    /**
     * Override the offline eviction threshold at runtime. The default is OFFLINE_EVICT_MS (30 s).
     * Intended for unit tests that need a short timeout to avoid long waits; not for production use.
     * @param ms  new threshold in milliseconds
     */
    void setOfflineEvictMs(int64_t ms) { offlineEvictMs_ = ms; }

    /**
     * Override the per-step base for the escalating reconnect backoff. The default is 6000 ms
     * (so the first step past the eviction window is 6 s). Intended for unit tests only.
     * In production, with this default, the interval reaches the LOST_BACKOFF_MAX_MS cap after
     * ~10 minutes of being lost.
     * @param ms  new per-step base in milliseconds
     */
    void setLostBackoffBaseMs(int64_t ms) { lostBackoffBaseMs_ = ms; }

    /**
     * SN-8177: ceiling on the escalating reconnect backoff interval for a "lost"
     * manually-configured host — 60 s. With the default lostBackoffBaseMs_ (6000 ms), the
     * probe interval grows roughly one step per minute the host has been silent (6 s, 12 s,
     * ...), reaching this 60 s cap after ~10 minutes lost. Keeps a dead-but-maybe-returning
     * relay from being hammered (and blocking the IO thread).
     */
    static constexpr int64_t LOST_BACKOFF_MAX_MS = 60000;

    /** mDNS query interval (ms) — matches ISmDnsPortFactory's rate */
    static constexpr int64_t MDNS_QUERY_INTERVAL_MS = 200;

private:
    RelayPortFactory() = default;
    ~RelayPortFactory();

    // -- Internal per-host state --
    /** Full internal state tracked for one relay host, including SSE worker state. */
    struct RelayHost {
        std::string  url;                                           //!< canonical "http://host:port" (no path)
        bool         enabled = false;                               //!< whether this host currently contributes ports
        bool         viaMdns = false;                               //!< true if discovered via mDNS, false if manually added
        std::vector<DeviceRecord> devices;                          //!< latest poll/snapshot result (metadata + hints)
        std::set<std::string> knownPortUrls;                        //!< high-water mark of tcp:// URLs ever seen from this host.
                                                                    //!< Only cleared on host disable/remove. Ports persist across
                                                                    //!< device reboots because bridgeboard's slot-based TCP sockets
                                                                    //!< survive device resets (WaitRecover keeps the listener open).
        std::chrono::steady_clock::time_point lastPollTime = {};    //!< last successful poll (Polling mode)
        std::chrono::steady_clock::time_point lastEventTime = {};   //!< last SSE event received (SSE mode)
        std::chrono::steady_clock::time_point lastContact = {};     //!< last contact of ANY kind: SSE bytes (incl. keepalive) or a
                                                                    //!< successful poll. Drives offline port eviction + reconnect backoff.
                                                                    //!< Seeded to "now" on enable so a never-reachable host starts its grace then.
        std::chrono::steady_clock::time_point lastAttemptTime = {}; //!< last poll ATTEMPT (success or failure) — gates the backoff cadence.
        std::string  lastError;                                     //!< empty on success; descriptive string on the most recent failure
        uint32_t     consecutiveFailures = 0;                       //!< polling failure counter (resets on success)

        // -- SSE worker state (populated while feedType == SSE) --
        std::string  serverInstanceId;                              //!< remote bridgeboard's UUID (restart detection)
        uint64_t     lastSnapshotId = 0;                            //!< numeric half of <instance_id>:<snapshot_id> for Last-Event-ID
        std::unique_ptr<std::thread> streamThread;                  //!< worker running the /api/events/devices reader
        std::atomic<bool> stopRequested{false};                     //!< asks the SSE worker to exit cleanly
        std::function<void()> sseAbortHook;                         //!< set by worker; aborts the current httplib request (unblocks recv)
        std::atomic<bool> streamConnected{false};                   //!< true while the SSE stream is open
        uint32_t     sseConsecutiveFailures = 0;                    //!< drives fallback to Polling after DEFAULT_MAX_SSE_RETRIES

        // -- Transport mode --
        RelayFeedType activeTransport = RelayFeedType::Auto;        //!< the transport currently in use for this host

        RelayHost() = default;
        // Non-copyable/movable because of the thread member.
        RelayHost(const RelayHost&) = delete;
        RelayHost& operator=(const RelayHost&) = delete;
        RelayHost(RelayHost&&) = delete;
        RelayHost& operator=(RelayHost&&) = delete;
    };

    /**
     * Hosts are keyed by canonical URL. std::map gives stable iterator/reference semantics,
     * which matters because the SSE worker holds a reference to its RelayHost for the
     * lifetime of the stream. unique_ptr keeps RelayHost move-stable on container growth.
     */
    std::map<std::string, std::unique_ptr<RelayHost>> relayHosts_;
    mutable std::recursive_mutex mutex_;
    std::chrono::milliseconds pollInterval_ = std::chrono::seconds(1);
    int64_t offlineEvictMs_ = OFFLINE_EVICT_MS;    //!< mutable threshold used by tick() and reconnectInterval(); override via setOfflineEvictMs()
    int64_t lostBackoffBaseMs_ = 6000;             //!< per-step multiplier for escalating backoff; override via setLostBackoffBaseMs()
    std::chrono::steady_clock::time_point lastMdnsQueryTime_ = {}; //!< rate-limit mDNS queries
    std::atomic<bool> mdnsDiscoveryEnabled_{true};  //!< false stops tick() querying and acting on mDNS; see setMdnsDiscoveryEnabled()

    /**
     * Rate-limited mDNS host discovery (reads from shared mdns:: cache). Adds newly-seen
     * hosts and reaps viaMdns hosts whose announcement has aged out of the cache. Reaped
     * hosts are moved into @p reaped (kept alive until their worker is joined) with their
     * abort hooks pushed to @p abortHooks; the caller signals + joins OUTSIDE mutex_ so a
     * worker blocked acquiring mutex_ can make progress (see SN-8177 / the dtor pattern).
     * Call with mutex_ held.
     *
     * @param[out] reaped      hosts removed this pass, moved here so they outlive their join
     * @param[out] abortHooks  abort hooks for the reaped hosts' workers, fired before joining
     */
    void discoverRelayHostsViaMdns(std::vector<std::unique_ptr<RelayHost>>& reaped,
                                   std::vector<std::function<void()>>& abortHooks);

    /**
     * Compute the next reconnect/poll interval for a host based on how long it's been silent:
     * the normal poll interval while healthy/recently-lost, escalating toward LOST_BACKOFF_MAX_MS
     * the longer it stays lost. Call with mutex_ held.
     *
     * @param host  the host whose cadence to compute
     * @param now   the current steady_clock time
     * @return the interval to wait before the next poll/reconnect attempt
     */
    std::chrono::milliseconds reconnectInterval(const RelayHost& host,
                                                std::chrono::steady_clock::time_point now) const;

    /**
     * Poll a single relay host's /api/availableDevices endpoint (fallback transport).
     *
     * @param host  the host to poll; updated in place with results or failure state
     */
    void pollRelayHost(RelayHost& host);

    /**
     * Start the SSE worker thread for a host (call with mutex_ held; releases during I/O).
     *
     * @param host  the host to start streaming
     */
    void startSseWorker(RelayHost& host);

    /**
     * Ask the SSE worker to stop and join it. Safe to call with mutex_ held.
     *
     * @param host  the host whose worker to stop
     */
    void stopSseWorker(RelayHost& host);

    /**
     * SSE worker body — runs on host.streamThread. Owns its own I/O and respects stopRequested.
     *
     * @param host  the host this worker streams for
     */
    void sseWorkerLoop(RelayHost& host);
};

#endif // IS_SDK__RELAY_PORT_FACTORY_H
