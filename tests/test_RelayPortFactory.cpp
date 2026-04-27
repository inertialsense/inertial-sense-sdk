/**
 * @file test_RelayPortFactory.cpp
 * @brief SDK unit tests for RelayPortFactory (SN-7805).
 *
 * Stands up a loopback cpp-httplib server that serves the SN-7804 wire contract
 * (/api/availableDevices snapshot + /api/events/devices SSE stream) and drives
 * RelayPortFactory against it.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include "gtest_helpers.h"

#include "httplib.h"
#include "json.hpp"
#include "RelayPortFactory.h"
#include "PortManager.h"
#include "DeviceManager.h"

using json = nlohmann::json;
using namespace std::chrono_literals;

namespace {

/// Compose one device entry in the SN-7804 schema. The optional `hdw` parameter mirrors
/// bridgeboard's consolidated hardware identity string (the cached pre-ISBL identity). When
/// empty, it is derived from `state` for convenience; pass it explicitly for `state == "isbl"`
/// to exercise the identity-preservation path.
json makeDevice(int testbed, int slot, const std::string& uri, uint32_t sn = 0,
                const std::string& state = "imx6", const std::string& hdw = "") {
    json d = {
        {"testbed", testbed},
        {"slot", slot},
        {"uri", uri},
        {"state", state},
        {"rdev", 40000 + slot},
        {"device_path", std::string("/dev/ttyACM") + std::to_string(slot)},
        {"has_tcp_client", false},
    };
    std::string hdwOut = hdw;
    if (hdwOut.empty()) {
        if      (state == "imx5") hdwOut = "IMX-5.0";
        else if (state == "imx6") hdwOut = "IMX-6.0";
        else if (state == "gpx")  hdwOut = "GPX-1.0";
    }
    if (!hdwOut.empty()) d["hdw"] = hdwOut;
    if (sn) {
        d["serial_number"] = sn;
        d["firmware_ver"] = "fw3.0.0-test";
        d["firmware_commit"] = "deadbeef";
        d["build_date"] = "2026-04-01 00:00:00";
    }
    return d;
}

/// Fake bridgeboard — implements just enough of SN-7804 to exercise RelayPortFactory.
/// Thread-safe; tests push device-list state changes and the server emits the appropriate SSE events.
class FakeBridgeboard {
public:
    FakeBridgeboard(const std::string& instanceId = "11111111-2222-3333-4444-555555555555")
        : instanceId_(instanceId) {}

    ~FakeBridgeboard() { stop(); }

    /// Start listening on a random loopback port. Returns the bound port, or 0 on failure.
    int start() {
        // Bind to an ephemeral port on localhost.
        port_ = server_.bind_to_any_port("127.0.0.1");
        if (port_ <= 0) return 0;

        server_.Get("/api/availableDevices", [this](const httplib::Request&, httplib::Response& res) {
            json body = {
                {"host", "fake.local"},
                {"server_instance_id", instanceId_},
                {"snapshot_id", snapshotId_.load()},
                {"devices", snapshotDevices()},
            };
            res.set_content(body.dump(), "application/json");
        });

        server_.Get("/api/events/devices", [this](const httplib::Request& req, httplib::Response& res) {
            const std::string lastId = req.get_header_value("Last-Event-ID");
            res.set_chunked_content_provider("text/event-stream",
                [this, lastId](size_t /*offset*/, httplib::DataSink& sink) {
                    handleStream(lastId, sink);
                    return true;
                });
        });

        listenThread_ = std::thread([this] { server_.listen_after_bind(); });
        // cpp-httplib needs a moment to actually start accepting connections.
        while (!server_.is_running()) std::this_thread::sleep_for(5ms);
        return port_;
    }

    void stop() {
        stopping_.store(true);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cv_.notify_all();  // wake any handler blocked in cv_.wait_for so they can observe is_running=false
        }
        if (server_.is_running()) server_.stop();
        // Any active worker threads need to observe stopping_ and exit sink.write loops.
        // Give them a beat to drain before we tear down the server object itself.
        if (listenThread_.joinable()) listenThread_.join();
    }

    /// Replace the device list and emit a `device.added`/`device.changed`/`device.removed`
    /// event as appropriate. Bumps snapshot_id.
    void emitAdded(const json& device) {
        std::lock_guard<std::mutex> lock(mutex_);
        devices_.push_back(device);
        uint64_t id = ++snapshotId_;
        pending_.push_back({"device.added", id, device.dump()});
        cv_.notify_all();
    }

    void emitRemoved(const std::string& uri) {
        std::lock_guard<std::mutex> lock(mutex_);
        devices_.erase(std::remove_if(devices_.begin(), devices_.end(),
            [&](const json& d) { return d.value("uri", std::string{}) == uri; }), devices_.end());
        uint64_t id = ++snapshotId_;
        json payload = {{"uri", uri}};
        pending_.push_back({"device.removed", id, payload.dump()});
        cv_.notify_all();
    }

    /// Drop the event ring (simulate buffer overflow) — next reconnect gets a fresh snapshot.
    void clearEventRing() {
        std::lock_guard<std::mutex> lock(mutex_);
        ring_.clear();
        ringBase_ = snapshotId_.load() + 1; // anything older is now out-of-range
    }

    /// Simulate a bridgeboard restart — new instanceId and reset snapshot_id.
    void restart(const std::string& newInstanceId) {
        std::lock_guard<std::mutex> lock(mutex_);
        instanceId_ = newInstanceId;
        snapshotId_.store(0);
        pending_.clear();
        ring_.clear();
        ringBase_ = 1;
        cv_.notify_all();
    }

    /// Override: serve SSE with a 500 status so transport-fallback tests can force retries.
    void setFailSse(bool fail) { failSse_ = fail; }

    int port() const { return port_; }
    std::string instanceId() const { std::lock_guard<std::mutex> lock(mutex_); return instanceId_; }

private:
    struct Event { std::string type; uint64_t id; std::string data; };

    void handleStream(const std::string& lastId, httplib::DataSink& sink) {
        if (failSse_.load()) {
            // Emulate a server error — write nothing and let the client time out / retry.
            sink.done();
            return;
        }

        // Decide: can we replay from the ring, or do we need a fresh snapshot?
        bool sendSnapshot = true;
        uint64_t resumeFromId = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!lastId.empty()) {
                auto colon = lastId.find(':');
                if (colon != std::string::npos) {
                    std::string clientInstance = lastId.substr(0, colon);
                    uint64_t clientId = 0;
                    try { clientId = std::stoull(lastId.substr(colon + 1)); } catch (...) {}
                    if (clientInstance == instanceId_ && clientId >= ringBase_ && clientId <= snapshotId_.load()) {
                        sendSnapshot = false;
                        resumeFromId = clientId;
                    }
                }
            }
        }

        if (sendSnapshot) {
            std::string frame;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                json data = {
                    {"server_instance_id", instanceId_},
                    {"devices", snapshotDevices()},
                };
                std::string id = instanceId_ + ":" + std::to_string(snapshotId_.load());
                frame = "event: snapshot\nid: " + id + "\ndata: " + data.dump() + "\n\n";
            }
            sink.write(frame.c_str(), frame.size());
        } else {
            // Replay events since resumeFromId
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& ev : ring_) {
                if (ev.id > resumeFromId) {
                    std::string id = instanceId_ + ":" + std::to_string(ev.id);
                    std::string frame = "event: " + ev.type + "\nid: " + id + "\ndata: " + ev.data + "\n\n";
                    sink.write(frame.c_str(), frame.size());
                }
            }
        }

        // Pump pending events until the sink closes (write returns false) or stop is requested.
        while (!stopping_.load() && server_.is_running()) {
            Event ev;
            bool have = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait_for(lock, 100ms,
                    [this] { return !pending_.empty() || stopping_.load(); });
                if (stopping_.load()) break;
                if (!pending_.empty()) {
                    ev = pending_.front();
                    pending_.pop_front();
                    ring_.push_back(ev);
                    if (ring_.size() > ringSize_) ring_.erase(ring_.begin());
                    have = true;
                }
            }
            if (have) {
                std::string id = instanceId_ + ":" + std::to_string(ev.id);
                std::string frame = "event: " + ev.type + "\nid: " + id + "\ndata: " + ev.data + "\n\n";
                if (!sink.write(frame.c_str(), frame.size())) break;
            }
        }
        sink.done();
    }

    json snapshotDevices() { /* must be called with mutex_ held externally, or from a safe-thread */
        return devices_;
    }

    mutable std::mutex mutex_;
    std::condition_variable cv_;

    httplib::Server server_;
    std::thread listenThread_;
    int port_ = 0;

    std::string instanceId_;
    std::atomic<uint64_t> snapshotId_{0};
    std::vector<json> devices_;
    std::deque<Event> pending_;    // events the server will emit
    std::deque<Event> ring_;       // history for Last-Event-ID replay
    size_t ringSize_ = 256;
    uint64_t ringBase_ = 1;        // snapshot_ids >= this are in the ring
    std::atomic<bool> failSse_{false};
    std::atomic<bool> stopping_{false};
};

/// Reset the RelayPortFactory singleton's state between tests. The factory is a
/// process-wide singleton, so we have to scrub everything it tracks.
void resetFactory() {
    auto& rpf = RelayPortFactory::getInstance();
    for (const auto& h : rpf.getRelayHosts()) {
        rpf.setRelayHostEnabled(h.url, false);
    }
    for (const auto& h : rpf.getRelayHosts()) {
        rpf.removeRelayHost(h.url);
    }
}

/// Spin-wait with timeout, running tick() periodically to drive the factory.
template <typename Pred>
bool waitFor(Pred pred, std::chrono::milliseconds timeout, std::chrono::milliseconds poll = 20ms) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(poll);
    }
    return pred();
}

} // anonymous namespace

// ============================================================
// URL normalization
// ============================================================

TEST(test_RelayPortFactory, urlNormalization_roundtrip) {
    auto& rpf = RelayPortFactory::getInstance();
    resetFactory();

    // Bare hostname with default port applied.
    rpf.addRelayHost("host.local");
    auto hosts = rpf.getRelayHosts();
    ASSERT_EQ(hosts.size(), 1u);
    EXPECT_EQ(hosts[0].url, "http://host.local:8080");

    // Same host, different input — should be deduped under the same canonical key.
    rpf.addRelayHost("http://host.local:8080/api/availableDevices");
    hosts = rpf.getRelayHosts();
    EXPECT_EQ(hosts.size(), 1u);

    // Different port → different canonical key.
    rpf.addRelayHost("host.local:9090");
    hosts = rpf.getRelayHosts();
    EXPECT_EQ(hosts.size(), 2u);

    // Remove accepts any equivalent form.
    EXPECT_TRUE(rpf.removeRelayHost("http://host.local:9090"));
    EXPECT_EQ(rpf.getRelayHosts().size(), 1u);

    resetFactory();
}

TEST(test_RelayPortFactory, unparseableUrlIgnored) {
    auto& rpf = RelayPortFactory::getInstance();
    resetFactory();
    rpf.addRelayHost("");
    rpf.addRelayHost("   ");
    EXPECT_EQ(rpf.getRelayHosts().size(), 0u);
    resetFactory();
}

// ============================================================
// SSE transport — end-to-end against fake bridgeboard
// ============================================================

TEST(test_RelayPortFactory, sseInitialSnapshot_200msBudget) {
    FakeBridgeboard fake;
    {
        // Seed an initial device in the snapshot.
        fake.emitAdded(makeDevice(1, 0, "tcp://127.0.0.1:44001", 12345));
    }
    ASSERT_GT(fake.start(), 0) << "fake bridgeboard failed to bind";

    auto& rpf = RelayPortFactory::getInstance();
    resetFactory();
    std::string url = "http://127.0.0.1:" + std::to_string(fake.port());
    rpf.addRelayHost(url);

    auto enableStart = std::chrono::steady_clock::now();
    rpf.setRelayHostEnabled(url, true);

    // Wait for the SSE transport to report devices.
    bool ok = waitFor([&]() {
        for (const auto& h : rpf.getRelayHosts()) {
            if (h.url == url && h.deviceCount > 0 && h.streamConnected) return true;
        }
        return false;
    }, 2s);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - enableStart);

    EXPECT_TRUE(ok) << "SSE didn't surface device within 2s";
    TEST_COUT << "initial snapshot latency: " << elapsed.count() << "ms" << std::endl;
    EXPECT_LT(elapsed.count(), 500) << "should beat polling (1s) by a wide margin";

    // AC: transport badge says SSE.
    bool foundSse = false;
    for (const auto& h : rpf.getRelayHosts()) {
        if (h.url == url) {
            EXPECT_EQ(h.feedType, RelayPortFactory::RelayFeedType::SSE);
            foundSse = (h.feedType == RelayPortFactory::RelayFeedType::SSE);
        }
    }
    EXPECT_TRUE(foundSse);

    rpf.setRelayHostEnabled(url, false);
    resetFactory();
    fake.stop();
}

TEST(test_RelayPortFactory, sseDeviceAddedRemoved) {
    FakeBridgeboard fake;
    ASSERT_GT(fake.start(), 0);

    auto& rpf = RelayPortFactory::getInstance();
    resetFactory();
    std::string url = "http://127.0.0.1:" + std::to_string(fake.port());
    rpf.addRelayHost(url);
    rpf.setRelayHostEnabled(url, true);

    // Wait for initial snapshot (zero devices).
    ASSERT_TRUE(waitFor([&]() {
        for (const auto& h : rpf.getRelayHosts())
            if (h.url == url && h.streamConnected) return true;
        return false;
    }, 2s));

    // Emit a device.added event — verify it surfaces.
    fake.emitAdded(makeDevice(1, 3, "tcp://127.0.0.1:55003", 99001));
    EXPECT_TRUE(waitFor([&]() {
        for (const auto& h : rpf.getRelayHosts())
            if (h.url == url && h.deviceCount == 1) return true;
        return false;
    }, 1s));

    fake.emitAdded(makeDevice(1, 4, "tcp://127.0.0.1:55004", 99002));
    EXPECT_TRUE(waitFor([&]() {
        for (const auto& h : rpf.getRelayHosts())
            if (h.url == url && h.deviceCount == 2) return true;
        return false;
    }, 1s));

    // Emit a device.removed — verify the port drops out.
    fake.emitRemoved("tcp://127.0.0.1:55003");
    EXPECT_TRUE(waitFor([&]() {
        for (const auto& h : rpf.getRelayHosts())
            if (h.url == url && h.deviceCount == 1) return true;
        return false;
    }, 1s));

    rpf.setRelayHostEnabled(url, false);
    resetFactory();
    fake.stop();
}

TEST(test_RelayPortFactory, sseServerRestart_forcesFreshSnapshot) {
    FakeBridgeboard fake("instance-A");
    fake.emitAdded(makeDevice(1, 0, "tcp://127.0.0.1:55000", 42));
    ASSERT_GT(fake.start(), 0);

    auto& rpf = RelayPortFactory::getInstance();
    resetFactory();
    std::string url = "http://127.0.0.1:" + std::to_string(fake.port());
    rpf.addRelayHost(url);
    rpf.setRelayHostEnabled(url, true);

    ASSERT_TRUE(waitFor([&]() {
        for (const auto& h : rpf.getRelayHosts())
            if (h.url == url && h.deviceCount == 1 && h.streamConnected) return true;
        return false;
    }, 2s));

    // Simulate bridgeboard restart with a new instance ID and no devices.
    fake.restart("instance-B");
    fake.emitAdded(makeDevice(1, 7, "tcp://127.0.0.1:55777", 777));

    // After a brief propagation, the factory should see exactly one device — the post-restart one —
    // without any duplicate ports from the pre-restart instance.
    EXPECT_TRUE(waitFor([&]() {
        for (const auto& h : rpf.getRelayHosts())
            if (h.url == url && h.deviceCount == 1) return true;
        return false;
    }, 5s));

    rpf.setRelayHostEnabled(url, false);
    resetFactory();
    fake.stop();
}

TEST(test_RelayPortFactory, pollingFallbackAfterSseFailures) {
    FakeBridgeboard fake;
    fake.setFailSse(true); // SSE endpoint returns immediately; the stream never stays open
    fake.emitAdded(makeDevice(1, 0, "tcp://127.0.0.1:55100", 100));
    ASSERT_GT(fake.start(), 0);

    auto& rpf = RelayPortFactory::getInstance();
    resetFactory();
    std::string url = "http://127.0.0.1:" + std::to_string(fake.port());
    rpf.addRelayHost(url);
    rpf.setRelayHostEnabled(url, true);

    // After 3 retries (backoff 250ms+500ms+1000ms ≈ 1.75s) + a few event-loop ticks,
    // feedType should transition to Polling.
    EXPECT_TRUE(waitFor([&]() {
        for (const auto& h : rpf.getRelayHosts())
            if (h.url == url && h.feedType == RelayPortFactory::RelayFeedType::Polling) return true;
        return false;
    }, 5s));

    // With polling active, tick() eventually populates devices from /api/availableDevices.
    EXPECT_TRUE(waitFor([&]() {
        RelayPortFactory::tick();
        for (const auto& h : rpf.getRelayHosts())
            if (h.url == url && h.deviceCount == 1) return true;
        return false;
    }, 3s));

    rpf.setRelayHostEnabled(url, false);
    resetFactory();
    fake.stop();
}
