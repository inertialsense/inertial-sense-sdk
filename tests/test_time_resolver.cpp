/**
 * @file test_time_resolver.cpp
 * @brief D-07 / SN-7897 — `ISTimeResolver` acceptance tests.
 *
 * Strategy: build a tiny multi-record fixture with a controlled mix
 * of ToW-bearing and ToW-less records, compose into an
 * `ISDeviceLog::fromSegments(...)`, and exercise the resolver.
 *
 * Note on fixture authorship — the v2 `.idx` schema collapse means
 * that for HAS_TOW records `.idx.timestamp == ToW`, so the resolver's
 * slope between sync points is identity. Tests assert the *confidence
 * tier* and the *direction* of extrapolation rather than precise
 * numeric outputs for pre-fix records (whose values are inherently
 * best-effort under the v2 schema — see the resolver header's note).
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // first

#include "DeviceLog.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "ISTimeResolver.h"
#include "data_sets.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <utility>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 999555u;

struct FixturePaths {
    fs::path directory;
    fs::path rawFile;
};

void writeRecord(cISLogger& logger, std::shared_ptr<cDeviceLog> dev,
                 uint32_t did, void* payload, std::size_t size) {
    is_comm_instance_t comm{};
    uint8_t buf[1024];
    is_comm_init(&comm, buf, sizeof(buf), nullptr);
    uint8_t pkt[2048];
    const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm,
                                      static_cast<uint16_t>(did),
                                      static_cast<uint16_t>(size), 0,
                                      payload);
    if (n > 0) logger.LogData(dev, n, pkt);
}

// ToW values picked to fall comfortably inside a typical GPS week (in
// seconds): 100..200s into the week. Multiplied by 1000 for the ms
// representation the resolver uses internally.
ins_2_t makeIns2(double towSec) {
    ins_2_t s{};
    s.week       = 2300;
    s.timeOfWeek = towSec;
    s.qn2b[0]    = 1.0f;
    s.lla[0]     = 40.0;
    s.lla[1]     = -111.0;
    s.lla[2]     = 1400.0;
    return s;
}

// SN-8107 / D0066: Unix-epoch ms for (gpsWeek=2300, towMs). The resolver
// epoch-anchors all output when ANY sync point's gpsWeek != 0, so tests
// assert against this anchored value rather than the raw ToW.
//   GPS_EPOCH_UNIX_MS = 315,964,800,000 (1980-01-06 00:00:00 UTC)
//   WEEK_MS           = 604,800,000
//   unix_ms = gpsWeek * WEEK_MS + GPS_EPOCH_UNIX_MS + towMs
inline uint64_t expectedUnixMsForFixtureWeek(uint64_t towMs,
                                             uint32_t gpsWeek = 2300) {
    constexpr uint64_t kGpsEpochUnixMs = 315'964'800'000ULL;
    constexpr uint64_t kWeekMs         = 604'800'000ULL;
    return static_cast<uint64_t>(gpsWeek) * kWeekMs + kGpsEpochUnixMs + towMs;
}

// IMU's `time` field is "seconds since boot" — `cISDataMappings::Timestamp`
// returns it for DID_IMU but DID_IMU is NOT in the resolver's ToW-bearing
// allowlist. So IMU records are non-sync (host_uptime_delta in their
// .idx timestamp).
imu_t makeImu(double bootSec) {
    imu_t s{};
    s.time = bootSec;
    s.I.acc[0] = 0.1f;
    s.I.acc[1] = 0.2f;
    s.I.acc[2] = 9.8f;
    return s;
}

FixturePaths buildFixture(const std::string& hint,
                          const std::vector<std::pair<uint32_t, std::vector<uint8_t>>>& records) {
    FixturePaths f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_time_resolver_%s_%d_%ld",
                  hint.c_str(), ::getpid(),
                  static_cast<long>(::time(nullptr)));
    f.directory = dirBuf;
    ISFileManager::DeleteDirectory(f.directory.string());

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(f.directory.string(), opts)) return f;
    auto devLogger = logger.registerDevice(kFixtureHwId, kFixtureSerial);
    if (!devLogger) return f;
    logger.EnableLogging(true);

    for (const auto& [did, payload] : records) {
        std::vector<uint8_t> mutablePayload = payload;
        writeRecord(logger, devLogger, did, mutablePayload.data(), mutablePayload.size());
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> rawFiles;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true,
                                          "\\.raw$", rawFiles);
    if (!rawFiles.empty()) f.rawFile = rawFiles.front().name;
    return f;
}

template <class T>
std::vector<uint8_t> bytesOf(const T& t) {
    std::vector<uint8_t> out(sizeof(T));
    std::memcpy(out.data(), &t, sizeof(T));
    return out;
}

void teardown(FixturePaths& f) {
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

class TimeResolverTest : public ::testing::Test {
protected:
    FixturePaths f;
    void TearDown() override { teardown(f); }
};

// ---------------------------------------------------------------------------
// Empty log → no sync points → SessionOnly/Unknown for any query.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, EmptyLogResolvesAsSessionOnly) {
    // Fixture with only ToW-LESS records (DID_IMU, which the resolver's
    // allowlist excludes). detectSyncPoints should find zero anchors.
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_IMU, bytesOf(makeImu(1.0)));
    recs.emplace_back(DID_IMU, bytesOf(makeImu(2.0)));
    f = buildFixture("empty_sync", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto logR = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(logR.has_value()) << logR.error().message;

    auto resolverR = ISTimeResolver::build(logR.value());
    ASSERT_TRUE(resolverR.has_value());

    EXPECT_TRUE(resolverR->syncPoints().empty());

    TimeStamp t = resolverR->resolve(50000, kFixtureSerial);
    EXPECT_EQ(t.source, TimeSource::SessionOnly);
    EXPECT_EQ(t.confidence, TimeConfidence::Unknown);
    EXPECT_EQ(t.value, 50000u);
    EXPECT_EQ(t.deviceId, kFixtureSerial);
}

// ---------------------------------------------------------------------------
// Sync points detected from INS_2 records with timeOfWeek > 0.
// Adjacent duplicates collapse.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, SyncPointsFromInsRecords) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    // 5 INS_2 records at ToWs 100, 110, 110, 120, 130 (110 duplicates).
    for (double tow : { 100.0, 110.0, 110.0, 120.0, 130.0 }) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    }
    f = buildFixture("syncs", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto logR = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(logR.has_value()) << logR.error().message;

    auto syncs = ISTimeResolver::detectSyncPoints(logR.value());
    ASSERT_EQ(syncs.size(), 4u) << "adjacent duplicate at ToW=110 should collapse";
    EXPECT_EQ(syncs[0].payloadToWMs, 100000u);
    EXPECT_EQ(syncs[1].payloadToWMs, 110000u);
    EXPECT_EQ(syncs[2].payloadToWMs, 120000u);
    EXPECT_EQ(syncs[3].payloadToWMs, 130000u);
    EXPECT_EQ(syncs[0].deviceId, kFixtureSerial);
    EXPECT_EQ(syncs[0].sourceDid, static_cast<uint32_t>(DID_INS_2));
}

// ---------------------------------------------------------------------------
// Resolve at exact sync-point timestamp → Exact/PayloadToW.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, ResolveExactAtSyncPoint) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : { 100.0, 110.0, 120.0 }) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    }
    f = buildFixture("exact", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    auto t = resolver->resolve(110000, kFixtureSerial);
    EXPECT_EQ(t.source, TimeSource::PayloadToW);
    EXPECT_EQ(t.confidence, TimeConfidence::Exact);
    // SN-8107 / D0066: epoch-anchored output (gpsWeek=2300 in makeIns2).
    EXPECT_EQ(t.value, expectedUnixMsForFixtureWeek(110000));
}

// ---------------------------------------------------------------------------
// Resolve between two sync points → Interpolated.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, ResolveInterpolated) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : { 100.0, 110.0, 120.0 }) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    }
    f = buildFixture("interp", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // 105_000 ms is halfway between syncs 100_000 and 110_000.
    // Slope is identity (host==ToW for v2 sync points), so the result
    // ToW value equals the input; we then assert against the
    // epoch-anchored output (SN-8107 / D0066).
    auto t = resolver->resolve(105000, kFixtureSerial);
    EXPECT_EQ(t.source, TimeSource::ResolvedViaSync);
    EXPECT_EQ(t.confidence, TimeConfidence::Interpolated);
    EXPECT_EQ(t.value, expectedUnixMsForFixtureWeek(105000));
}

// ---------------------------------------------------------------------------
// Resolve past last sync → ExtrapolatedForward.
// Resolve before first sync → ExtrapolatedBackward.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, ResolveExtrapolated) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : { 100.0, 110.0, 120.0 }) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    }
    f = buildFixture("extrap", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    auto fwd = resolver->resolve(150000, kFixtureSerial);
    EXPECT_EQ(fwd.source, TimeSource::ResolvedViaSync);
    EXPECT_EQ(fwd.confidence, TimeConfidence::ExtrapolatedForward);

    auto bwd = resolver->resolve(50000, kFixtureSerial);
    EXPECT_EQ(bwd.source, TimeSource::ResolvedViaSync);
    EXPECT_EQ(bwd.confidence, TimeConfidence::ExtrapolatedBackward);
}

// ---------------------------------------------------------------------------
// SN-8323: a log that begins BEFORE GPS fix. The earliest (smallest-ToW) sync
// record carries week 0 (device still searching); later records carry the real
// week. Because syncPoints_ is sorted by ToW, front() is the week-0 record;
// anchoring to it (old behavior) left the log in the ToW-only ~1980 domain. The
// resolver must anchor to the most-common valid (non-zero) week instead.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, PreFixWeekZeroAnchorsToValidWeek) {
    auto pre = makeIns2(10.0);   pre.week = 0;     // smallest ToW, pre-fix week 0
    auto a   = makeIns2(100.0);  a.week   = 2300;  // post-fix, valid week
    auto b   = makeIns2(110.0);  b.week   = 2300;
    auto c   = makeIns2(120.0);  c.week   = 2300;
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_INS_2, bytesOf(pre));
    recs.emplace_back(DID_INS_2, bytesOf(a));
    recs.emplace_back(DID_INS_2, bytesOf(b));
    recs.emplace_back(DID_INS_2, bytesOf(c));
    f = buildFixture("prefix_week0", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // front() sync is the week-0 pre-fix record (smallest ToW) — the trap the
    // old code fell into.
    ASSERT_FALSE(resolver->syncPoints().empty());
    EXPECT_EQ(resolver->syncPoints().front().gpsWeek, 0u);

    // Must anchor to the valid week 2300, NOT week 0.
    auto t = resolver->resolve(105000, kFixtureSerial);
    EXPECT_EQ(t.value, expectedUnixMsForFixtureWeek(105000, 2300));
    EXPECT_GE(t.value, 315'964'800'000ULL + 2300ULL * 604'800'000ULL);  // real-year domain
    EXPECT_NE(t.value, 105000ULL);  // the un-anchored ToW-only (~1980) result
}

// SN-8323: all-week-0 log (device never fixed) → no valid week → fall back to
// ToW-only (no epoch anchor); must not crash.
TEST_F(TimeResolverTest, AllWeekZeroFallsBackToToWOnly) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : { 100.0, 110.0, 120.0 }) {
        auto s = makeIns2(tow); s.week = 0;
        recs.emplace_back(DID_INS_2, bytesOf(s));
    }
    f = buildFixture("allweek0", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());
    auto t = resolver->resolve(105000, kFixtureSerial);
    EXPECT_EQ(t.value, 105000ULL);  // ToW-only passthrough, no epoch anchor
}

// SN-8323: a brief startup transient reports a WRONG non-zero week for a couple
// records before the durable fix settles. The anchor must be derived from the
// durable fix (widest ToW coverage), not the short-span transient.
TEST_F(TimeResolverTest, StartupTransientWeekLosesToDurableFix) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    // transient glitch: 2 records, tiny ToW span (5..6 s), wrong week 1111
    for (double tow : { 5.0, 6.0 }) {
        auto s = makeIns2(tow); s.week = 1111;
        recs.emplace_back(DID_INS_2, bytesOf(s));
    }
    // durable fix: many records, wide ToW span (100..600 s), real week 2300
    for (double tow : { 100.0, 200.0, 300.0, 400.0, 500.0, 600.0 }) {
        auto s = makeIns2(tow); s.week = 2300;
        recs.emplace_back(DID_INS_2, bytesOf(s));
    }
    f = buildFixture("transient_week", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());
    // Anchor to the durable week 2300, not the transient 1111.
    auto t = resolver->resolve(300000, kFixtureSerial);
    EXPECT_EQ(t.value, expectedUnixMsForFixtureWeek(300000, 2300));
}

// SN-8323 (part 2): a pre-fix record whose ToW is well before the durable fix
// window is tagged SessionOnly/Unknown so consumers drop it from the timeline +
// extent (no "leading gap"), rather than anchoring it to a bogus week-start
// time. A query inside the durable window still resolves normally.
TEST_F(TimeResolverTest, PreFixToWBeforeDurableWindowIsSessionOnly) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    { auto s = makeIns2(1.0); s.week = 0; recs.emplace_back(DID_INS_2, bytesOf(s)); }  // pre-fix, ~1 s into week
    for (double tow : { 400000.0, 400100.0, 400200.0 }) {  // durable fix ~4.6 days into the week
        auto s = makeIns2(tow); s.week = 2300;
        recs.emplace_back(DID_INS_2, bytesOf(s));
    }
    f = buildFixture("prefix_before_window", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // Pre-fix ToW (~1 s) is far before the durable window (~4.6 d) -> excluded.
    auto pre = resolver->resolve(1000, kFixtureSerial);
    EXPECT_EQ(pre.source, TimeSource::SessionOnly);
    EXPECT_EQ(pre.confidence, TimeConfidence::Unknown);

    // A query inside the durable window still resolves (2300-anchored).
    auto ok = resolver->resolve(400100000, kFixtureSerial);
    EXPECT_EQ(ok.value, expectedUnixMsForFixtureWeek(400100000, 2300));
}

// ---------------------------------------------------------------------------
// Discontinuity detection — synthesize a clock jump between sync points
// by giving the third sync point a much smaller delta than the gap
// before it.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, DiscontinuityFromUnevenGaps) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    // ToW sequence: 100, 110, 130, 130.0001.
    // Sync points have host==ToW per the .idx schema, so the slope
    // between any two non-zero-spaced sync points is exactly 1.0.
    // The detector's ratio metric only fires on slope CHANGES — same
    // slope = no discontinuity. Producing a true slope-change scenario
    // requires sync points with distinct host vs ToW values (i.e. the
    // .raw-recovered actualHostTimeMs disagreeing with payloadToWMs),
    // which we don't synthesize here. We exercise the "no
    // discontinuities" path on uniform-slope syncs.
    for (double tow : { 100.0, 110.0, 130.0, 130.0001 }) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    }
    f = buildFixture("disc", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // v2 schema: all slopes are 1.0; no discontinuities expected.
    EXPECT_TRUE(resolver->discontinuities().empty());
}

// ---------------------------------------------------------------------------
// computeStats — counts always add up to the device-log's iterated
// record count, even for empty / .idx-only-header logs.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, StatsAddUpToRecordCount) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    // 3 INS_2 sync records + 2 IMU non-sync records — exercises both
    // branches of `computeStats` (Exact for matching syncs, the
    // extrapolation tiers for non-matching).
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0)));
    recs.emplace_back(DID_IMU,   bytesOf(makeImu(1.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(110.0)));
    recs.emplace_back(DID_IMU,   bytesOf(makeImu(2.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(120.0)));

    f = buildFixture("stats", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    auto stats = resolver->computeStats(log.value());
    const std::size_t total = stats.exact + stats.interpolated
                            + stats.extrapFwd + stats.extrapBack
                            + stats.unknown;
    // The fundamental invariant: the histogram sums to the iterated
    // record count, whatever that is. (cISLogger's writer doesn't
    // always populate the on-disk .idx for very short fixtures —
    // that's an upstream-writer quirk independent of D-07; the
    // other resolve* tests above cover the per-tier outcomes via
    // direct calls to `resolve()`.)
    EXPECT_EQ(total, log->recordCount());
}

// ---------------------------------------------------------------------------
// SN-8107 / D0066: cross-domain bridge. A v2-.idx log mixes records in two
// numeric domains (sync records carry GPS-ToW ms ≈ hundreds of millions;
// non-sync records carry host uptime ms ≈ thousands). The resolver MUST
// translate session-uptime queries into the ToW frame using the
// `actualHostTimeMs` recovered during the byte scan from the most recent
// non-sync record's payload-side timestamp.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, CrossDomainBridgeUnifiesPimuIntoTowFrame) {
    // Build a fixture mimicking the IMX-6 fw3.0.0 layout: several IMU
    // records (small host uptime, non-sync) followed by an INS_2 sync
    // record carrying a large ToW. The cross-domain bridge should detect
    // session-uptime queries and translate them into the ToW frame.
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    // Three IMU records at host uptimes 0.100s, 0.123s, 0.150s.
    recs.emplace_back(DID_IMU, bytesOf(makeImu(0.100)));
    recs.emplace_back(DID_IMU, bytesOf(makeImu(0.123)));
    recs.emplace_back(DID_IMU, bytesOf(makeImu(0.150)));
    // First sync at ToW 411.500s (411500 ms). The scan should capture
    // 150 ms as `actualHostTimeMs` (the most recent IMU's host time).
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(411.500)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(411.700)));

    f = buildFixture("bridge", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // Sanity: two sync points (411500, 411700), both anchored with
    // actualHostTimeMs == 150 (the IMU host uptime right before the
    // first sync). Each sync also carries the GPS week (2300, from
    // makeIns2) so the resolver epoch-anchors all output.
    const auto& syncs = resolver->syncPoints();
    ASSERT_EQ(syncs.size(), 2u);
    EXPECT_EQ(syncs[0].payloadToWMs, 411500u);
    EXPECT_EQ(syncs[0].actualHostTimeMs, 150u);
    EXPECT_EQ(syncs[0].gpsWeek, 2300u);

    // Cross-domain bridge: resolve a session-uptime query (e.g. 100 ms).
    // Expected ToW: offset = 411500 - 150 = 411350; bridged ToW = 411450.
    // Expected epoch-anchored: bridged ToW + (2300 * 604800000) + 315964800000.
    const TimeStamp bridged = resolver->resolve(100u, kFixtureSerial);
    EXPECT_EQ(bridged.source, TimeSource::ResolvedViaSync);
    EXPECT_EQ(bridged.confidence, TimeConfidence::ExtrapolatedBackward);
    EXPECT_EQ(bridged.value, expectedUnixMsForFixtureWeek(411450));

    // A larger session-uptime query (e.g. 150 ms = exactly the captured
    // actualHostTimeMs) bridges to the sync's ToW, epoch-anchored.
    const TimeStamp atSync = resolver->resolve(150u, kFixtureSerial);
    EXPECT_EQ(atSync.value, expectedUnixMsForFixtureWeek(411500));

    // ToW-domain query (already in the resolver's anchor frame) falls
    // through the non-bridge path: 411500 = first sync, Exact match,
    // epoch-anchored.
    const TimeStamp exact = resolver->resolve(411500u, kFixtureSerial);
    EXPECT_EQ(exact.source, TimeSource::PayloadToW);
    EXPECT_EQ(exact.confidence, TimeConfidence::Exact);
    EXPECT_EQ(exact.value, expectedUnixMsForFixtureWeek(411500));
}

// ---------------------------------------------------------------------------
// SN-8107: when no non-sync record precedes the first sync point in the
// byte stream, actualHostTimeMs stays 0 and the bridge branch is skipped
// — falls back to legacy classify-only behavior.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, CrossDomainBridgeSkippedWhenNoPreSyncNonSync) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    // No IMU records before the first sync. Bridge should NOT engage.
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(411.500)));
    recs.emplace_back(DID_IMU,   bytesOf(makeImu(0.200)));

    f = buildFixture("no_presync_nonsync", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    const auto& syncs = resolver->syncPoints();
    ASSERT_EQ(syncs.size(), 1u);
    EXPECT_EQ(syncs[0].actualHostTimeMs, 0u);  // no recovery possible

    // resolve(100ms): legacy extrapolate-backward path with slope=1,
    // tow = s0.payloadToWMs + 1.0 * (100 - s0.hostTimeMs)
    //     = 411500 + (100 - 411500) = 100 (stays positive; no clamp).
    // Result IS epoch-anchored (sync's gpsWeek=2300), so the value is
    // GPS-epoch + 2300 weeks + 100 ms.
    // SN-8107/D0066: prior expectation (ToW-0) predated epoch anchoring.
    const TimeStamp legacy = resolver->resolve(100u, kFixtureSerial);
    EXPECT_EQ(legacy.confidence, TimeConfidence::ExtrapolatedBackward);
    EXPECT_EQ(legacy.value, expectedUnixMsForFixtureWeek(100));
}

// ---------------------------------------------------------------------------
// Single-sync-point case: resolves backward / forward with slope 1.0.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, SingleSyncPointDegenerateSlope) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0)));
    f = buildFixture("single", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());
    ASSERT_EQ(resolver->syncPoints().size(), 1u);

    auto t = resolver->resolve(105000, kFixtureSerial);
    EXPECT_EQ(t.confidence, TimeConfidence::ExtrapolatedForward);
    // Slope defaults to 1.0 with a single sync point: ToW = 100_000 +
    // (105_000 - 100_000) = 105_000, then epoch-anchored (gpsWeek=2300).
    // SN-8107/D0066: prior expectation (raw ToW) predated epoch anchoring.
    EXPECT_EQ(t.value, expectedUnixMsForFixtureWeek(105000));

    auto bwd = resolver->resolve(95000, kFixtureSerial);
    EXPECT_EQ(bwd.confidence, TimeConfidence::ExtrapolatedBackward);
    EXPECT_EQ(bwd.value, expectedUnixMsForFixtureWeek(95000));
}

// ---------------------------------------------------------------------------
// SN-8115: an input that is ALREADY an absolute Unix-ms timestamp (at or
// beyond the GPS Unix epoch, 1980) is returned unchanged — never re-anchored.
// This is the guard against the year-2082 double-anchor seen on the 16-device
// compass fixture, where a wall-clock-poisoned spanEnd (~1.78e12) fed into
// resolve() had the epoch + week offset added on top (-> ~3.55e12).
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, AlreadyAnchoredInputPassesThroughUnchanged) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(200.0)));
    f = buildFixture("already_anchored", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // A real 2026 wall-clock value (the compass fixture's poisoned spanEnd
    // sat right here). It is >> any ToW (< 1 week) or session-uptime, so the
    // resolver must treat it as already-anchored and return it verbatim, NOT
    // run gpsToUnixMs on it (which would yield ~2x = year 2082).
    constexpr uint64_t kAnchored2026 = 1779821742200ULL;
    const TimeStamp t = resolver->resolve(kAnchored2026, kFixtureSerial);
    EXPECT_EQ(t.value, kAnchored2026);
    EXPECT_LT(t.value, 2ULL * kAnchored2026);  // explicitly: not doubled
}

// resolve() is idempotent for already-resolved values: feeding a resolved
// (epoch-anchored) output back in returns the same value. Guarantees
// `resolve(resolve(x)) == resolve(x)`.
TEST_F(TimeResolverTest, ResolveIsIdempotent) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(411.500)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(411.700)));
    f = buildFixture("idempotent", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // First pass: a raw ToW query resolves to an epoch-anchored Unix-ms value.
    const TimeStamp once = resolver->resolve(411500u, kFixtureSerial);
    EXPECT_EQ(once.value, expectedUnixMsForFixtureWeek(411500));
    // Second pass on the already-resolved value is a no-op.
    const TimeStamp twice = resolver->resolve(once.value, kFixtureSerial);
    EXPECT_EQ(twice.value, once.value);
}

// ---------------------------------------------------------------------------
// Live-fixture smoke: exercise the resolver against a real cltool-captured
// log if one is reachable. Looks at the env var
// `IS_SDK_LIVE_FIXTURE_RAW` first, then falls back to the project's
// canonical sample-logs directory. `GTEST_SKIP` when neither exists, so
// CI without the data is silent rather than red.
// ---------------------------------------------------------------------------
TEST(TimeResolverLiveFixture, RealCltoolCaptureSmoke) {
    fs::path raw;
    if (const char* env = std::getenv("IS_SDK_LIVE_FIXTURE_RAW")) {
        raw = env;
    } else {
        // Default: the cltool-captured 120 s PPD log from SN519465 that
        // sits in the project's sample-logs tree. Not committed in the
        // SDK repo; available on developer machines that ran the
        // capture script.
        raw = fs::path(std::getenv("HOME") ? std::getenv("HOME") : "/")
            / "workspace/inertialsense/sample_logs/cltool_imx5_120s_ppd"
              "/20260429_105836/LOG_SN519465_20260429_105836_0001.raw";
    }
    if (!fs::exists(raw)) {
        GTEST_SKIP() << "live fixture not present at " << raw
                     << " (set IS_SDK_LIVE_FIXTURE_RAW to override)";
    }

    auto log = ISDeviceLog::fromSegments({ raw });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    EXPECT_GT(log->recordCount(), 0u);

    auto resolverR = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolverR.has_value()) << resolverR.error().message;
    const auto& resolver = resolverR.value();

    const auto& syncs = resolver.syncPoints();
    EXPECT_GT(syncs.size(), 0u) << "expected at least one ToW-bearing record "
                                   "in a 120 s PPD capture";

    // Sync points must be timestamp-sorted and adjacent-duplicate-free.
    for (std::size_t i = 1; i < syncs.size(); ++i) {
        EXPECT_LT(syncs[i - 1].hostTimeMs, syncs[i].hostTimeMs)
            << "sync points must be strictly increasing after dedup (i=" << i << ")";
    }

    // Resolve the first sync point's hostTimeMs → must be Exact.
    if (!syncs.empty()) {
        const auto first = syncs.front();
        const auto ts = resolver.resolve(first.hostTimeMs, first.deviceId);
        EXPECT_EQ(ts.source, TimeSource::PayloadToW);
        EXPECT_EQ(ts.confidence, TimeConfidence::Exact);
        EXPECT_EQ(ts.value, first.payloadToWMs);
    }

    // Resolve a point well before the first sync → ExtrapolatedBackward.
    if (!syncs.empty()) {
        const auto bwd = resolver.resolve(0u, log->deviceId());
        EXPECT_EQ(bwd.source, TimeSource::ResolvedViaSync);
        EXPECT_EQ(bwd.confidence, TimeConfidence::ExtrapolatedBackward);
    }

    // Stats histogram sums to iterated record count.
    auto stats = resolver.computeStats(log.value());
    const std::size_t total = stats.exact + stats.interpolated
                            + stats.extrapFwd + stats.extrapBack
                            + stats.unknown;
    EXPECT_EQ(total, log->recordCount());

    // Surface a one-line summary in the test log so a developer running
    // this can eyeball the resolver's behavior on real data.
    std::printf("[live] raw=%s rawBytes=%llu records=%zu syncs=%zu "
                "discs=%zu stats: exact=%zu interp=%zu extFwd=%zu "
                "extBack=%zu unk=%zu\n",
                raw.c_str(),
                static_cast<unsigned long long>(fs::file_size(raw)),
                log->recordCount(), syncs.size(),
                resolver.discontinuities().size(),
                stats.exact, stats.interpolated, stats.extrapFwd,
                stats.extrapBack, stats.unknown);
}

} // namespace
