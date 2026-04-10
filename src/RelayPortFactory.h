/**
 * @file RelayPortFactory.h
 * @brief Discovers IS device ports through remote HTTP-based relay hosts (e.g., bridgeboard, future cltool-as-relay).
 *
 * @author Kyle Mallory on 4/10/26.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__RELAY_PORT_FACTORY_H
#define IS_SDK__RELAY_PORT_FACTORY_H

#include <chrono>
#include <map>
#include <mutex>
#include <set>
#include <string>
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
 * Phase 1: HTTP polling of /api/status at a configurable interval (default 1 Hz).
 * Phase 3 (SN-7805) will upgrade the internal transport to SSE-driven push without
 * changing the public API.
 *
 * @code{.cpp}
 * portManager.addPortFactory(&RelayPortFactory::getInstance());
 * RelayPortFactory::getInstance().addRelayHost("http://192.168.1.50:8080/api/status");
 * RelayPortFactory::getInstance().setRelayHostEnabled("http://192.168.1.50:8080/api/status", true);
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

    // -- Per-host status snapshot (returned by getRelayHosts()) --
    struct RelayHostStatus {
        std::string  url;                                           ///< http://host:port/api/status
        bool         enabled = false;                               ///< whether this host contributes ports
        bool         viaMdns = false;                               ///< true if discovered via mDNS, false if manually added
        std::chrono::steady_clock::time_point lastPollTime = {};    ///< time of last successful poll
        std::string  lastError;                                     ///< empty on success; descriptive string on failure
        uint32_t     consecutiveFailures = 0;                       ///< reset to 0 on each successful poll
        size_t       deviceCount = 0;                               ///< number of devices reported by this host
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

    /// mDNS query interval (ms) — matches ISmDnsPortFactory's rate
    static constexpr int64_t MDNS_QUERY_INTERVAL_MS = 200;

private:
    RelayPortFactory() = default;
    ~RelayPortFactory() = default;

    // -- Internal per-host state --
    struct RelayHost {
        std::string  url;                                           ///< http://host:port/api/status
        bool         enabled = false;
        bool         viaMdns = false;
        std::vector<DeviceRecord> devices;                          ///< latest poll result (metadata + hints)
        std::set<std::string> knownPortUrls;                        ///< high-water mark of tcp:// URLs ever seen from this host.
                                                                    ///< Only cleared on host disable/remove. Ports persist across
                                                                    ///< device reboots because bridgeboard's slot-based TCP sockets
                                                                    ///< survive device resets (WaitRecover keeps the listener open).
        std::chrono::steady_clock::time_point lastPollTime = {};
        std::string  lastError;
        uint32_t     consecutiveFailures = 0;
    };

    std::map<std::string, RelayHost> relayHosts_;                   ///< keyed by URL
    mutable std::recursive_mutex mutex_;
    std::chrono::milliseconds pollInterval_ = std::chrono::seconds(1);
    std::chrono::steady_clock::time_point lastMdnsQueryTime_ = {}; ///< rate-limit mDNS queries

    /// Rate-limited mDNS host discovery (reads from shared mdns:: cache).
    void discoverRelayHostsViaMdns();

    /// Poll a single enabled relay host's HTTP endpoint, update its cached state.
    void pollRelayHost(RelayHost& host);
};

#endif // IS_SDK__RELAY_PORT_FACTORY_H
