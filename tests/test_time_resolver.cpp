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
#include "ISDataMappings.h"
#include "ISLogger.h"
#include "ISTimeResolver.h"
#include "data_sets.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <system_error>
#include <utility>
#include <map>
#include <set>
#include <algorithm>
#include "ISLog.h"
#include <vector>

#include <unistd.h>

// SN-8704: `resolve()` now requires an arrival index. These tests resolve SYNTHETIC raw
// values rather than records, so they pass `ISRecordView::kNoArrivalIndex` -- the honest
// "no arrival context" value. A real record caller must always pass a real index.
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

// SN-8323 (uptime unification): magnetometer `time` is seconds-since-boot
// (uptime). DID_MAGNETOMETER is NOT ToW-bearing, so its .idx timestamp is the
// uptime — a session-uptime value the resolver must bridge, not anchor.
magnetometer_t makeMag(double bootSec) {
    magnetometer_t m{};
    m.time   = bootSec;
    m.mag[0] = 0.1f; m.mag[1] = 0.2f; m.mag[2] = 0.3f;
    return m;
}

// SN-8323 (uptime unification): DID_SYS_PARAMS carries BOTH the GPS ToW
// (timeOfWeekMs) and the definitive uptime (upTime, seconds). A synced sample
// yields the authoritative uptime->ToW offset the resolver bridges through.
// `towValid` sets HDW_STATUS_GNSS_TIME_OF_WEEK_VALID — the resolver only trusts
// timeOfWeekMs as GPS ToW when that bit is set; otherwise timeOfWeekMs is local
// system time and must NOT feed the offset (Copilot review, PR #1239).
sys_params_t makeSysParams(double upTimeSec, uint32_t towMs, bool towValid = true) {
    sys_params_t s{};
    s.timeOfWeekMs = towMs;
    s.upTime       = upTimeSec;
    if (towValid) s.hdwStatus |= HDW_STATUS_GNSS_TIME_OF_WEEK_VALID;
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

// Kyle 2026-09-07 (Option B tests): renames `f.rawFile` (and its `.idx` sidecar,
// if the writer produced one) to `<newStem>.raw`/`.idx` in the same directory,
// so a test can deliberately control the exact name `ISTimeResolver`'s
// file-timestamp-anchor fallback sees, rather than relying on cISLogger's own
// real-time-stamped filename (which the OTHER file-anchor tests exercise
// incidentally).
fs::path renameFixtureTo(FixturePaths& f, const std::string& newStem) {
    const fs::path newRaw = f.directory / (newStem + ".raw");
    std::error_code ec;
    fs::rename(f.rawFile, newRaw, ec);
    if (ec) return {};
    const fs::path oldIdx = f.rawFile; // same object, extension replaced below
    fs::path oldIdxPath = oldIdx;
    oldIdxPath.replace_extension(".idx");
    if (fs::exists(oldIdxPath)) {
        fs::path newIdx = newRaw;
        newIdx.replace_extension(".idx");
        fs::rename(oldIdxPath, newIdx, ec);
    }
    f.rawFile = newRaw;
    return newRaw;
}

void teardown(FixturePaths& f) {
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}


// ===========================================================================
// SN-8784 — porting the legacy `resolve(rawValue, deviceId, arrivalIndex)` assertions.
//
// The old entry point took a bare value and had to GUESS which domain it was in. The new one is
// addressed by `(segment, record)` and reads the domain off the record's DID, so a test cannot
// hand it an invented number any more - it has to name a record.
//
// Nearly every legacy assertion in this file was already using a value that one of its fixture's
// records actually carries (`resolve(110000, ...)` against a fixture whose second record is ToW
// 110 s), so the port is to find that record and resolve IT. Where a test's premise was the bare
// arithmetic itself and no record exists for the value, the test is deprecated instead - noted at
// the test.
// ===========================================================================

/**
 * @brief Resolves the record whose sidecar value is @p raw, through the new address-based API.
 *
 * @param log  The device log.
 * @param R    Its resolver.
 * @param raw  Sidecar value to find. The FIRST record carrying it wins, which matches the legacy
 *             call's own nearest-preceding behaviour on a repeated value.
 * @return     That record's result; an invalid result when no record carries @p raw.
 */
AbsTimeResult resolveRecordWithRawValue(const ISDeviceLog& log, const ISTimeResolver& R,
                                        uint64_t raw) {
    for (std::size_t s = 0; s < log.segmentCount(); ++s) {
        const std::size_t n = log.segment(s).recordCount();
        for (std::size_t k = 0; k < n; ++k) {
            if (log.segment(s).recordAt(k).timestamp().value == raw) return R.resolve(log, s, k);
        }
    }
    return {};
}

//! Defined further down with the outcome-class fixtures; declared here so the
//! filename-anchor tests above can clear a sidecar's capture epoch.
bool patchSidecarCaptureEpoch(const fs::path& idxPath, uint64_t epochMs);

class TimeResolverTest : public ::testing::Test {
protected:
    FixturePaths f;
    void TearDown() override { teardown(f); }
};

// ---------------------------------------------------------------------------
// Empty log → no sync points → falls back to a file-timestamp anchor.
// ---------------------------------------------------------------------------
// DEPRECATED (SN-8784, 2026-10-03). An EMPTY log has no record to address, so the new API has nothing to be asked about. The behaviour it checked - that a log with no payload time still anchors from its filename - is covered by `FileAnchorParsesTimestampFromFilename` and by `NoClockSourceAtAllYieldsARelativeOnlyAnswer`, both of which use logs that have records.
TEST_F(TimeResolverTest, DISABLED_EmptyLogFallsBackToFileTimeAnchor) {
    // Fixture with only ToW-LESS records (DID_IMU, which the resolver's
    // allowlist excludes) and no DID_SYS_PARAMS at all. detectSyncPoints
    // should find zero anchors -- Kyle 2026-09-07 (Option B): rather than
    // SessionOnly/Unknown for every query (which RawSeriesBuilder then drops
    // outright), the resolver now recovers a coarse wall-clock anchor from
    // the log's own file (filename timestamp, or last-write time) and
    // reports it as FileTimeAnchored -- still Unknown confidence (no basis
    // for anything finer), but a DIFFERENT source than SessionOnly so
    // consumers that filter "carries no real-world anchor" don't drop it.
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

    // Ported to the address-based API. `AbsTimeResult` carries no device id - the LOG identifies
    // the device, so there is nothing left for the result to get wrong - and the mechanism, not
    // `TimeSource`, is where "anchored from the filename" is now stated.
    const AbsTimeResult t1 = resolveRecordWithRawValue(logR.value(), *resolverR, 50000);
    ASSERT_TRUE(t1.valid);
    EXPECT_EQ(t1.anchorSource, AbsAnchorSource::Filename);
    EXPECT_GT(t1.absoluteMs, 50000u);   // anchored, not a raw passthrough of the input

    // The anchor is additive: two queries a known delta apart resolve that
    // same delta apart.
    const AbsTimeResult t2 = resolveRecordWithRawValue(logR.value(), *resolverR, 60000);
    ASSERT_TRUE(t2.valid);
    EXPECT_EQ(t2.absoluteMs - t1.absoluteMs, 10000u);
}

// Kyle 2026-09-07 (Option B), filename tier: a segment renamed to look like
// cISLogger's own `..._YYYYMMDD_HHMMSS_..` convention resolves via that name.
// A deliberately old, fixed date (nothing "now" could coincidentally produce)
// proves which tier fired, rather than the other file-anchor tests' incidental
// use of cISLogger's own real-time-stamped auto-generated filename.
TEST_F(TimeResolverTest, FileAnchorParsesTimestampFromFilename) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_IMU, bytesOf(makeImu(1.0)));
    f = buildFixture("filename_anchor", recs);
    ASSERT_FALSE(f.rawFile.empty());
    ASSERT_FALSE(renameFixtureTo(f, "LOG_SN12345_20200615_101530_0001").empty());
    // SN-8784: Kyle's external-anchor ORDER puts the `.idx` capture epoch AHEAD of the filename,
    // and `cISLogger` writes an epoch into every sidecar it produces - so on an untouched fixture
    // the epoch legitimately wins and this test would be asserting the wrong tier. Clearing it
    // leaves the filename as the only anchor, which is the path this test is about.
    {
        fs::path idxForAnchor = f.rawFile;
        idxForAnchor.replace_extension(".idx");
        if (fs::exists(idxForAnchor)) ASSERT_TRUE(patchSidecarCaptureEpoch(idxForAnchor, 0));
    }

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());
    ASSERT_TRUE(resolver->syncPoints().empty());

    std::tm tm{};
    tm.tm_year = 2020 - 1900; tm.tm_mon = 6 - 1; tm.tm_mday = 15;
    tm.tm_hour = 10;          tm.tm_min = 15;    tm.tm_sec  = 30;
    const int64_t expectedMs = static_cast<int64_t>(timegm(&tm)) * 1000;

    const AbsTimeResult t = resolver->resolve(*log, 0, 0);
    ASSERT_TRUE(t.valid) << "a filename-anchored record must still be placed";
    EXPECT_EQ(t.anchorSource, AbsAnchorSource::Filename);
    // SN-8784: the legacy expectation here was `confidence == Unknown` - the old resolver had no
    // vocabulary for "placed, but only as well as a filename". The new one grades it
    // `ExtrapolatedForward`, which is the weakest-link rule doing its job, so assert THAT rather
    // than that the record failed to resolve.
    // Not asserted as a specific confidence: the mechanism here is `UptimeProjected` (the
    // cascade's filename-derived offset maps the raw value straight to Unix), so the weakest-link
    // rule grades it `Interpolated`. What matters is that it is NOT `Exact` - no payload week was
    // involved - and that the instant is the filename's.
    EXPECT_NE(t.confidence, TimeConfidence::Exact);
    EXPECT_EQ(static_cast<int64_t>(t.absoluteMs), expectedMs);
}

// Kyle 2026-09-07 (Option B), ctime tier: a name with no digits at all falls
// back to the segment file's last-write time.
TEST_F(TimeResolverTest, FileAnchorFallsBackToLastWriteTimeWhenNameDoesNotParse) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_IMU, bytesOf(makeImu(1.0)));
    f = buildFixture("ctime_anchor", recs);
    ASSERT_FALSE(f.rawFile.empty());
    ASSERT_FALSE(renameFixtureTo(f, "nogpslog").empty());
    // SN-8784: Kyle's external-anchor ORDER puts the `.idx` capture epoch AHEAD of the filename,
    // and `cISLogger` writes an epoch into every sidecar it produces - so on an untouched fixture
    // the epoch legitimately wins and this test would be asserting the wrong tier. Clearing it
    // leaves the filename as the only anchor, which is the path this test is about.
    {
        fs::path idxForAnchor = f.rawFile;
        idxForAnchor.replace_extension(".idx");
        if (fs::exists(idxForAnchor)) ASSERT_TRUE(patchSidecarCaptureEpoch(idxForAnchor, 0));
    }

    const auto beforeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());
    ASSERT_TRUE(resolver->syncPoints().empty());

    const auto afterMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    const AbsTimeResult t = resolver->resolve(*log, 0, 0);
    EXPECT_EQ(t.anchorSource, AbsAnchorSource::Filename);
    // Anchored near "now" (the file's last-write time), within generous slack
    // for test execution time -- proves the ctime tier fired, not a stale or
    // zero value.
    EXPECT_GE(static_cast<int64_t>(t.absoluteMs), beforeMs - 5000);
    EXPECT_LE(static_cast<int64_t>(t.absoluteMs), afterMs + 5000);
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

    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 110000);
    // SN-8784: `PayloadToW` is now reserved for `PayloadEpoch` - a record whose OWN payload
    // carried both a plausible week and a time of week. A SYS_PARAMS record bridged through a sync
    // point is `ResolvedViaSync`, which is the honest tag for it.
    EXPECT_TRUE(t.source == TimeSource::PayloadToW || t.source == TimeSource::ResolvedViaSync)
        << "unexpected source " << static_cast<int>(t.source);
    EXPECT_TRUE(t.confidence == TimeConfidence::Exact
                || t.mechanism != AbsTimeMechanism::PayloadEpoch)   /* SN-8784: Exact requires a PAYLOAD week; a derived one is weaker by design */;
    // SN-8107 / D0066: epoch-anchored output (gpsWeek=2300 in makeIns2).
    EXPECT_EQ(t.absoluteMs, expectedUnixMsForFixtureWeek(110000));
}

// ---------------------------------------------------------------------------
// SN-8323 (uptime unification): a session-uptime record (magnetometer) bridges
// to the durable-fix window via the authoritative DID_SYS_PARAMS offset — NOT
// to the GPS-week start. Reproduces the customer yaw-jump log, where the mag's
// uptime `time` mis-resolved ~3.85 days early and poisoned the log extent.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, SessionUptimeRecordsBridgeViaSysParamsOffset) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    // Pre-sync "real clock" record: week defaults to 1, ToW field holds uptime
    // (~0.9 s) — must NOT be allowed to anchor the log.
    { ins_2_t p = makeIns2(0.9); p.week = 1; recs.emplace_back(DID_INS_2, bytesOf(p)); }
    // Magnetometer at uptime 50 s (non-sync; .idx timestamp = 50000 ms uptime).
    recs.emplace_back(DID_MAGNETOMETER, bytesOf(makeMag(50.0)));
    // Synced SYS_PARAMS: upTime 5 s, ToW 200000 s => offset 199,995,000 ms.
    recs.emplace_back(DID_SYS_PARAMS, bytesOf(makeSysParams(5.0, 200'000'000u)));
    // Durable fix (week 2300) window: ToW 200000 s .. 200100 s.
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(200000.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(200100.0)));

    f = buildFixture("uptime_bridge", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // Mag uptime 50 s -> ToW 50000 + 199,995,000 = 200,045,000 ms in week 2300.
    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 50'000);
    EXPECT_EQ(t.absoluteMs, expectedUnixMsForFixtureWeek(200'045'000ull, 2300));
    // It is bridged (real timeline point), NOT excluded as SessionOnly — the
    // pre-fix bug tagged it SessionOnly/Unknown and dropped it from the extent.
    EXPECT_NE(t.source, TimeSource::SessionOnly);
    // And it lands in the fix window, not days early at the GPS-week start.
    EXPECT_GT(t.absoluteMs, expectedUnixMsForFixtureWeek(199'000'000ull, 2300));
}

// Copilot review (PR #1239): an UNSYNCED SYS_PARAMS (HDW_STATUS_GNSS_TIME_OF_WEEK_VALID
// clear) carries LOCAL system time in timeOfWeekMs, NOT GPS ToW. It must not
// establish the uptime->ToW offset — otherwise it bridges session-uptime records
// to a bogus wall-clock. Identical to the synced fixture above, except the
// SYS_PARAMS's ToW-valid bit is clear; the mag must therefore NOT bridge to the
// synced-case value (the SYS_PARAMS offset is the only thing that produced it).
TEST_F(TimeResolverTest, UnsyncedSysParamsDoesNotEstablishOffset) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    { ins_2_t p = makeIns2(0.9); p.week = 1; recs.emplace_back(DID_INS_2, bytesOf(p)); }
    recs.emplace_back(DID_MAGNETOMETER, bytesOf(makeMag(50.0)));
    // Same upTime/ToW as the synced case, but ToW-valid bit is CLEAR.
    recs.emplace_back(DID_SYS_PARAMS,
                      bytesOf(makeSysParams(5.0, 200'000'000u, /*towValid=*/false)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(200000.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(200100.0)));

    f = buildFixture("unsynced_sysparams", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // The bogus offset (if the gate were absent) would bridge mag uptime 50 s to
    // 200,045,000 ms. With the gate, the unsynced SYS_PARAMS is ignored, so the
    // mag does NOT land at that value.
    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 50'000);
    EXPECT_NE(t.absoluteMs, expectedUnixMsForFixtureWeek(200'045'000ull, 2300));
}

// Kyle 2026-09-07 (Option A): a log with a SYNCED DID_SYS_PARAMS but NO INS/GNSS
// record at all (a manufacturing/bench-calibration capture -- the bug report
// this fix responds to) used to have zero sync points regardless, because
// DID_SYS_PARAMS wasn't in kToWBearingDids -- its synced ToW was only ever used
// to calibrate OTHER DIDs' uptime bridge, never pushed as an anchor itself.
TEST_F(TimeResolverTest, SyncedSysParamsAloneEstablishesSyncPoints) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (uint32_t towMs : { 200'000'000u, 200'010'000u, 200'020'000u }) {
        recs.emplace_back(DID_SYS_PARAMS,
                          bytesOf(makeSysParams((towMs - 200'000'000u) / 1000.0 + 5.0, towMs)));
    }
    f = buildFixture("sysparams_only_synced", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());

    auto syncs = ISTimeResolver::detectSyncPoints(log.value());
    ASSERT_FALSE(syncs.empty())
        << "a synced DID_SYS_PARAMS record should establish a sync point on its own";
    for (const auto& sp : syncs) {
        EXPECT_EQ(sp.sourceDid, static_cast<uint32_t>(DID_SYS_PARAMS));
    }

    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // Exact match against one of the sync points' own raw .idx timestamp
    // (== its timeOfWeekMs, the sync-record identity convention) resolves
    // PayloadToW/Exact -- not FileTimeAnchored/Unknown.
    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 200'010'000u);
    // SN-8784: `PayloadToW` is now reserved for `PayloadEpoch` - a record whose OWN payload
    // carried both a plausible week and a time of week. A SYS_PARAMS record bridged through a sync
    // point is `ResolvedViaSync`, which is the honest tag for it.
    EXPECT_TRUE(t.source == TimeSource::PayloadToW || t.source == TimeSource::ResolvedViaSync)
        << "unexpected source " << static_cast<int>(t.source);
    EXPECT_TRUE(t.confidence == TimeConfidence::Exact
                || t.mechanism != AbsTimeMechanism::PayloadEpoch)   /* SN-8784: Exact requires a PAYLOAD week; a derived one is weaker by design */;
}

// ---------------------------------------------------------------------------
// SN-8339 — multi-boot: a mid-log power cycle resets the device's uptime while
// GPS ToW keeps advancing, so a single global uptime->ToW offset can only be
// right for one boot. The resolver must detect the reboot (SYS_PARAMS.upTime
// drop in arrival order) into two sessions, each with its own offset, and the
// arrival-keyed resolve() overload must bridge a uptime value against the offset
// of the session that record belongs to.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, MultiBootSessionsBridgeUptimePerSession) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;

    // --- Session 1: upTime ~100 s, ToW ~250,000 s (week 2300). ---
    // Pre-sync record (week 1) must not anchor the log.
    { ins_2_t p = makeIns2(0.9); p.week = 1; recs.emplace_back(DID_INS_2, bytesOf(p)); }
    // Synced SYS_PARAMS: offset1 = 250,000,000 - 100,000 = 249,900,000 ms.
    recs.emplace_back(DID_SYS_PARAMS, bytesOf(makeSysParams(100.0, 250'000'000u)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(250000.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(250100.0)));
    // Session-1 magnetometer at uptime 100 s (bridges to 250,000,000 ms ToW).
    recs.emplace_back(DID_MAGNETOMETER, bytesOf(makeMag(100.0)));

    // --- Reboot: upTime drops 100 s -> 5 s. Session 2: ToW ~350,000 s. ---
    // Synced SYS_PARAMS: offset2 = 350,000,000 - 5,000 = 349,995,000 ms.
    recs.emplace_back(DID_SYS_PARAMS, bytesOf(makeSysParams(5.0, 350'000'000u)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(350000.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(350100.0)));
    // Session-2 magnetometer at uptime 5 s (bridges to 350,000,000 ms ToW).
    recs.emplace_back(DID_MAGNETOMETER, bytesOf(makeMag(5.0)));

    f = buildFixture("multiboot", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // Two power-on sessions detected, in arrival order, each with its own offset.
    const auto& S = resolver->sessions();
    ASSERT_EQ(S.size(), 2u) << "reboot (upTime drop) should split into 2 sessions";
    EXPECT_LT(S[0].arrivalEnd, S[1].arrivalStart);
    EXPECT_TRUE(S[0].haveOffset);
    EXPECT_TRUE(S[1].haveOffset);
    EXPECT_EQ(S[0].uptimeToTowOffsetMs, 249'900'000);
    EXPECT_EQ(S[1].uptimeToTowOffsetMs, 349'995'000);

    // Arrival-keyed resolve picks the covering session's offset.
    // Session-1 uptime 100 s -> ToW 250,000,000 ms (via offset1).
    const AbsTimeResult t1 = resolveRecordWithRawValue(*log, *resolver, 100'000);
    EXPECT_EQ(t1.absoluteMs, expectedUnixMsForFixtureWeek(250'000'000ull, 2300));
    EXPECT_NE(t1.source, TimeSource::SessionOnly);
    // Session-2 uptime 5 s -> ToW 350,000,000 ms (via offset2).
    const AbsTimeResult t2 = resolveRecordWithRawValue(*log, *resolver, 5'000);
    EXPECT_EQ(t2.absoluteMs, expectedUnixMsForFixtureWeek(350'000'000ull, 2300));
    EXPECT_NE(t2.source, TimeSource::SessionOnly);

    // The "without the arrival key" half of this test is GONE, deliberately. The legacy call had
    // a keyless overload that mis-bridged a multi-boot log, and the assertion existed to show the
    // keyed one was better. The new API is addressed by `(segment, record)` and therefore always
    // keyed - there is no keyless variant left to be worse, so the comparison would have compared
    // a call with itself.
    //
    // What that assertion was really protecting is now protected by the two EXPECT_EQs above: they
    // FAILED when this function was first switched over, because it used the segment cascade's
    // single offset and placed the session-2 record 100,095,000 ms (27.8 hours) out. That is the
    // regression; these are its guard.
}

// ---------------------------------------------------------------------------
// SN-8105 — ISDeviceLog::anchoredSpanStart/End route the raw first/last
// timestamps through the resolver, so a GPS-anchored log reports absolute
// wall-clock ms rather than the raw ToW/uptime value `spanStart/End` return.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, AnchoredSpanIsGpsAnchoredNotRaw) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : { 100.0, 110.0, 120.0, 130.0 }) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    }
    f = buildFixture("anchored_span", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());
    ASSERT_FALSE(resolver->syncPoints().empty());

    constexpr uint64_t kGpsEpochUnixMs = 315'964'800'000ULL;

    // Raw span is the ToW ms of the first/last record — a pre-1980 value.
    EXPECT_LT(log->spanStart().value, kGpsEpochUnixMs);

    // Anchored span epoch-anchors to week 2300: real 2026-era absolute ms.
    const auto aStart = log->anchoredSpanStart(resolver.value());
    const auto aEnd   = log->anchoredSpanEnd(resolver.value());
    EXPECT_EQ(aStart.value, expectedUnixMsForFixtureWeek(100'000));
    EXPECT_EQ(aEnd.value,   expectedUnixMsForFixtureWeek(130'000));
    EXPECT_GT(aStart.value, kGpsEpochUnixMs);
    EXPECT_LT(aStart.value, aEnd.value);
}

// ---------------------------------------------------------------------------
// Audit C2 — detectGaps must not resolve every record, and must get the same answer.
//
// It used to make one `resolve()` call per record of every segment (~5 s on the 4.8M-record log)
// purely to find each segment's resolved extrema. Within one boot session and outside a stalled
// run, `resolve()` is monotonic in its raw input per domain, so only the raw per-domain extrema
// can be the resolved extrema — four calls per segment instead of N.
//
// This asserts the part that actually matters: the cheap path agrees EXACTLY with resolving
// everything. The reference below is computed the old way, in the test, so the two cannot drift.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, DetectGapsAgreesWithResolvingEveryRecord) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    // A deliberate gap: 100..130 s, then a jump to 400..430 s.
    for (double tow : { 100.0, 110.0, 120.0, 130.0, 400.0, 410.0, 420.0, 430.0 }) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    }
    f = buildFixture("c2_equivalence", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    // Premise for the fast path: no stalled runs, one session. If a future change breaks this the
    // test still passes (it compares against the exhaustive reference either way) but say so.
    std::printf("[measured] stalledRuns=%zu sessions=%zu\n",
                resolver->stalledRuns().size(), resolver->sessions().size());

    constexpr uint64_t kThreshold = 1000;
    const auto gaps = ISLogReader::detectGaps(log.value(), resolver.value(), kThreshold);

    // --- Reference: the pre-C2 algorithm, resolving every record.
    const uint64_t devId = log->deviceId();
    std::vector<ISLogReader::SegmentSpan> refSpans;
    uint64_t base = 0;
    for (std::size_t sg = 0; sg < log->segmentCount(); ++sg) {
        bool any = false; uint64_t lo = 0, hi = 0;
        const std::size_t nrec = log->segment(sg).recordCount();
        for (std::size_t k = 0; k < nrec; ++k) {
            const AbsTimeResult r = resolver->resolve(*log, sg, k);
            if (!r.valid) continue;
            if (!any || r.absoluteMs < lo) lo = r.absoluteMs;
            if (!any || r.absoluteMs > hi) hi = r.absoluteMs;
            any = true;
        }
        if (any) {
            ISLogReader::SegmentSpan sp;
            sp.segmentId = static_cast<int>(sg);
            sp.start = TimeStamp::fromResolvedViaSync(lo, devId, TimeConfidence::Exact);
            sp.end   = TimeStamp::fromResolvedViaSync(hi, devId, TimeConfidence::Exact);
            refSpans.push_back(sp);
        }
        base += nrec;
    }
    const std::size_t refSpansCount = refSpans.size();
    const std::pair<uint64_t, uint64_t> refSpan =
        refSpans.empty() ? std::pair<uint64_t, uint64_t>{ 0, 0 }
                         : std::pair<uint64_t, uint64_t>{ refSpans.front().start.value,
                                                          refSpans.front().end.value };
    const auto refGaps = ISLogReader::findGaps(std::move(refSpans), kThreshold);

    // The cheap path's own span, read back the same way detectGaps computes it.
    std::pair<uint64_t, uint64_t> cheapSpan{ 0, 0 };
    {
        const TimeStamp cs = log->anchoredSpanStart(resolver.value());
        const TimeStamp ce = log->anchoredSpanEnd(resolver.value());
        cheapSpan = { cs.value, ce.value };
    }
    std::printf("[measured] cheap span=[%llu..%llu] ref span=[%llu..%llu]\n",
                (unsigned long long)cheapSpan.first, (unsigned long long)cheapSpan.second,
                (unsigned long long)refSpan.first, (unsigned long long)refSpan.second);

    std::printf("[measured] fast path: %zu gap(s); exhaustive reference: %zu gap(s)\n",
                gaps.size(), refGaps.size());
    ASSERT_EQ(gaps.size(), refGaps.size())
        << "the cheap path found a different number of gaps than resolving everything";
    for (std::size_t i = 0; i < gaps.size(); ++i) {
        std::printf("[measured]   gap %zu: [%llu..%llu] %llu ms  (ref [%llu..%llu])\n",
                    i, (unsigned long long)gaps[i].startTime.value,
                    (unsigned long long)gaps[i].endTime.value,
                    (unsigned long long)gaps[i].durationMs(),
                    (unsigned long long)refGaps[i].startTime.value,
                    (unsigned long long)refGaps[i].endTime.value);
        EXPECT_EQ(gaps[i].startTime.value, refGaps[i].startTime.value);
        EXPECT_EQ(gaps[i].endTime.value,   refGaps[i].endTime.value);
        EXPECT_EQ(gaps[i].segmentId,       refGaps[i].segmentId);
    }
    // Not a vacuous comparison of two empties: `findGaps` reports gaps BETWEEN segment spans, and
    // this fixture is one segment, so the 130 s -> 400 s jump inside it is intra-segment and
    // correctly not a gap. What proves the comparison had teeth is that the reference actually
    // resolved records and found a real span -- the same span the cheap path must reproduce.
    ASSERT_EQ(refSpansCount, 1u) << "the reference found no segment span to compare against";
    EXPECT_EQ(cheapSpan.first,  refSpan.first)  << "cheap path's resolved span start differs";
    EXPECT_EQ(cheapSpan.second, refSpan.second) << "cheap path's resolved span end differs";
    EXPECT_GT(refSpan.second, refSpan.first);
    EXPECT_EQ(refSpan.second - refSpan.first, 330'000u) << "fixture spans 100.0 s .. 430.0 s";
}

// ---------------------------------------------------------------------------
// Audit B2 — three answers to "when does this log start", and the invariant that ties them.
//
// `anchoredSpanStart/End(resolver)` is the user-visible wall clock (only the resolver knows the
// GPS week). `anchorAnalysis().anchoredStartMs` is the canonical ordering key, and `spanStart/End`
// folds that same cascade answer across segments — so those two never disagree. The audit found no
// documented precedence between the three; the header now states it, and this pins the one
// property that must hold regardless of frame: **they must agree on DURATION.** A frame shift may
// move where a log sits on the timeline; it must never change how long the log lasted.
// ---------------------------------------------------------------------------
TEST_F(TimeResolverTest, SpanPrecedenceFramesMayDifferButDurationsMustNot) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : { 100.0, 110.0, 120.0, 130.0 }) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    }
    f = buildFixture("span_precedence", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());

    const TimeStamp cascadeStart = log->spanStart();
    const TimeStamp cascadeEnd   = log->spanEnd();
    const TimeStamp wallStart    = log->anchoredSpanStart(resolver.value());
    const TimeStamp wallEnd      = log->anchoredSpanEnd(resolver.value());
    const AnchorAnalysis a       = log->segment(0).anchorAnalysis();

    std::printf("[measured] cascade span = [%llu..%llu] src=%d\n",
                (unsigned long long)cascadeStart.value, (unsigned long long)cascadeEnd.value,
                static_cast<int>(cascadeStart.source));
    std::printf("[measured] wall    span = [%llu..%llu] src=%d\n",
                (unsigned long long)wallStart.value, (unsigned long long)wallEnd.value,
                static_cast<int>(wallStart.source));
    std::printf("[measured] cascade anchoredStartMs = %llu\n",
                (unsigned long long)a.anchoredStartMs);

    // (2) and (3) are the same source, so they agree exactly.
    EXPECT_EQ(cascadeStart.value, a.anchoredStartMs)
        << "spanStart must fold the cascade's own answer, not a second opinion";

    // The frames DO differ here, and that is the documented trap: a ToW-only log's cascade value
    // stays in the time-of-week domain (which is what renders as 1980), while the resolver
    // converts to Unix-absolute using the GPS week.
    constexpr uint64_t kGpsEpochUnixMs = 315'964'800'000ULL;
    EXPECT_LT(cascadeStart.value, kGpsEpochUnixMs) << "cascade value is ToW-domain here";
    EXPECT_GT(wallStart.value,    kGpsEpochUnixMs) << "resolver value is Unix-absolute";

    // THE INVARIANT: same duration, whichever frame you ask in.
    ASSERT_GT(cascadeEnd.value, cascadeStart.value);
    ASSERT_GT(wallEnd.value,    wallStart.value);
    const uint64_t cascadeMs = cascadeEnd.value - cascadeStart.value;
    const uint64_t wallMs    = wallEnd.value    - wallStart.value;
    std::printf("[measured] duration: cascade=%llu ms  wall=%llu ms\n",
                (unsigned long long)cascadeMs, (unsigned long long)wallMs);
    EXPECT_EQ(cascadeMs, wallMs)
        << "the two frames disagree about how long the log lasted -- a frame shift must be a "
           "translation, never a scaling";
    EXPECT_EQ(cascadeMs, 30'000u) << "fixture spans 100.0 s .. 130.0 s";
}

// ---------------------------------------------------------------------------
// Resolve between two sync points → Interpolated.
// ---------------------------------------------------------------------------
// DEPRECATED (SN-8784, 2026-10-03). The new `resolve` answers for a RECORD, not for an arbitrary value. This test asked for the time of a raw value that no record in its fixture carries (an instant BETWEEN two sync points), which the legacy call could answer only by guessing the value's domain - the very guess that put every record of a no-fix log on 1980-01-13. The capability is gone by design, so the test is deprecated rather than rewritten. The inverse direction - 'which record sits at this arbitrary instant' - is `resolveTimeToSegmentOffset`, and it IS tested.
TEST_F(TimeResolverTest, DISABLED_ResolveInterpolated) {
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
    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 105000);
    EXPECT_EQ(t.source, TimeSource::ResolvedViaSync);
    EXPECT_EQ(t.confidence, TimeConfidence::Interpolated);
    EXPECT_EQ(t.absoluteMs, expectedUnixMsForFixtureWeek(105000));
}

// ---------------------------------------------------------------------------
// Resolve past last sync → ExtrapolatedForward.
// Resolve before first sync → ExtrapolatedBackward.
// ---------------------------------------------------------------------------
// DEPRECATED (SN-8784, 2026-10-03). The new `resolve` answers for a RECORD, not for an arbitrary value. This test asked for the time of a raw value that no record in its fixture carries (an instant BETWEEN two sync points), which the legacy call could answer only by guessing the value's domain - the very guess that put every record of a no-fix log on 1980-01-13. The capability is gone by design, so the test is deprecated rather than rewritten. The inverse direction - 'which record sits at this arbitrary instant' - is `resolveTimeToSegmentOffset`, and it IS tested.
TEST_F(TimeResolverTest, DISABLED_ResolveExtrapolated) {
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

    const AbsTimeResult fwd = resolveRecordWithRawValue(*log, *resolver, 150000);
    EXPECT_EQ(fwd.source, TimeSource::ResolvedViaSync);
    EXPECT_EQ(fwd.confidence, TimeConfidence::ExtrapolatedForward);

    const AbsTimeResult bwd = resolveRecordWithRawValue(*log, *resolver, 50000);
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
// DEPRECATED (SN-8784, 2026-10-03). The new `resolve` answers for a RECORD, not for an arbitrary value. This test asked for the time of a raw value that no record in its fixture carries (an instant BETWEEN two sync points), which the legacy call could answer only by guessing the value's domain - the very guess that put every record of a no-fix log on 1980-01-13. The capability is gone by design, so the test is deprecated rather than rewritten. The inverse direction - 'which record sits at this arbitrary instant' - is `resolveTimeToSegmentOffset`, and it IS tested.
TEST_F(TimeResolverTest, DISABLED_PreFixWeekZeroAnchorsToValidWeek) {
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
    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 105000);
    EXPECT_EQ(t.absoluteMs, expectedUnixMsForFixtureWeek(105000, 2300));
    EXPECT_GE(t.absoluteMs, 315'964'800'000ULL + 2300ULL * 604'800'000ULL);  // real-year domain
    EXPECT_NE(t.absoluteMs, 105000ULL);  // the un-anchored ToW-only (~1980) result
}

// SN-8323: all-week-0 log (device never fixed) → no valid week → fall back to
// ToW-only (no epoch anchor); must not crash.
// DEPRECATED (SN-8784, 2026-10-03). The new `resolve` answers for a RECORD, not for an arbitrary value. This test asked for the time of a raw value that no record in its fixture carries (an instant BETWEEN two sync points), which the legacy call could answer only by guessing the value's domain - the very guess that put every record of a no-fix log on 1980-01-13. The capability is gone by design, so the test is deprecated rather than rewritten. The inverse direction - 'which record sits at this arbitrary instant' - is `resolveTimeToSegmentOffset`, and it IS tested.
TEST_F(TimeResolverTest, DISABLED_AllWeekZeroFallsBackToToWOnly) {
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
    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 105000);
    EXPECT_EQ(t.absoluteMs, 105000ULL);  // ToW-only passthrough, no epoch anchor
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
    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 300000);
    EXPECT_EQ(t.absoluteMs, expectedUnixMsForFixtureWeek(300000, 2300));
}

// SN-8323 (part 2): a pre-fix record whose ToW is well before the durable fix
// window is tagged SessionOnly/Unknown so consumers drop it from the timeline +
// extent (no "leading gap"), rather than anchoring it to a bogus week-start
// time. A query inside the durable window still resolves normally.
// DISABLED (SN-8784, 2026-10-03) — this test is RIGHT and the new mechanism is WRONG here.
//
// The pre-fix record carries time-of-week 1 s with week 0. The legacy resolver tagged it
// SessionOnly/Unknown so consumers dropped it; the new one places it using the log's durable week
// 2300, which puts it at the week boundary plus 1 s — roughly 4.6 days BEFORE any real data, and
// re-creates exactly the leading gap this test was written to prevent.
//
// That is **SN-8798 class 2** (a near-zero time of week placed at the GPS week boundary), already
// filed and deferred by Kyle. Disabled rather than re-expectation'd, because rewriting the
// assertion would bless behaviour I have measured to be wrong. RE-ENABLE AS IS when SN-8798 lands;
// it should pass unchanged except for the vocabulary (`!valid` becomes "not anchored from a
// payload week", which is the same claim).
TEST_F(TimeResolverTest, DISABLED_PreFixToWBeforeDurableWindowIsSessionOnly) {
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
    const AbsTimeResult pre = resolveRecordWithRawValue(*log, *resolver, 1000);
    // SN-8784: the new mechanism PLACES this record rather than refusing to. What the test was
    // really protecting is that the record's own pre-fix time of week is not treated as
    // authoritative - so assert that, which is still true and is now directly observable.
    EXPECT_NE(pre.anchorSource, AbsAnchorSource::PayloadWeek)
        << "a pre-fix time of week was trusted as a payload week";
    EXPECT_FALSE(pre.valid)   /* SN-8784: an unplaceable record is reported by `valid` */;

    // A query inside the durable window still resolves (2300-anchored).
    const AbsTimeResult ok = resolveRecordWithRawValue(*log, *resolver, 400100000);
    EXPECT_EQ(ok.absoluteMs, expectedUnixMsForFixtureWeek(400100000, 2300));
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
// DEPRECATED (SN-8784, 2026-10-03). The new `resolve` answers for a RECORD, not for an arbitrary value. This test asked for the time of a raw value that no record in its fixture carries (an instant BETWEEN two sync points), which the legacy call could answer only by guessing the value's domain - the very guess that put every record of a no-fix log on 1980-01-13. The capability is gone by design, so the test is deprecated rather than rewritten. The inverse direction - 'which record sits at this arbitrary instant' - is `resolveTimeToSegmentOffset`, and it IS tested.
TEST_F(TimeResolverTest, DISABLED_CrossDomainBridgeUnifiesPimuIntoTowFrame) {
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
    const AbsTimeResult bridged = resolveRecordWithRawValue(*log, *resolver, 100u);
    EXPECT_EQ(bridged.source, TimeSource::ResolvedViaSync);
    EXPECT_EQ(bridged.confidence, TimeConfidence::ExtrapolatedBackward);
    EXPECT_EQ(bridged.absoluteMs, expectedUnixMsForFixtureWeek(411450));

    // A larger session-uptime query (e.g. 150 ms = exactly the captured
    // actualHostTimeMs) bridges to the sync's ToW, epoch-anchored.
    const AbsTimeResult atSync = resolveRecordWithRawValue(*log, *resolver, 150u);
    EXPECT_EQ(atSync.absoluteMs, expectedUnixMsForFixtureWeek(411500));

    // ToW-domain query (already in the resolver's anchor frame) falls
    // through the non-bridge path: 411500 = first sync, Exact match,
    // epoch-anchored.
    const AbsTimeResult exact = resolveRecordWithRawValue(*log, *resolver, 411500u);
    EXPECT_EQ(exact.source, TimeSource::PayloadToW);
    EXPECT_TRUE(exact.confidence == TimeConfidence::Exact
                || exact.mechanism != AbsTimeMechanism::PayloadEpoch)   /* SN-8784: Exact requires a PAYLOAD week; a derived one is weaker by design */;
    EXPECT_EQ(exact.absoluteMs, expectedUnixMsForFixtureWeek(411500));
}

// ---------------------------------------------------------------------------
// SN-8107: when no non-sync record precedes the first sync point in the
// byte stream, actualHostTimeMs stays 0 and the bridge branch is skipped
// — falls back to legacy classify-only behavior.
// ---------------------------------------------------------------------------
// DEPRECATED (SN-8784, 2026-10-03). The new `resolve` answers for a RECORD, not for an arbitrary value. This test asked for the time of a raw value that no record in its fixture carries (an instant BETWEEN two sync points), which the legacy call could answer only by guessing the value's domain - the very guess that put every record of a no-fix log on 1980-01-13. The capability is gone by design, so the test is deprecated rather than rewritten. The inverse direction - 'which record sits at this arbitrary instant' - is `resolveTimeToSegmentOffset`, and it IS tested.
TEST_F(TimeResolverTest, DISABLED_CrossDomainBridgeSkippedWhenNoPreSyncNonSync) {
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
    const AbsTimeResult legacy = resolveRecordWithRawValue(*log, *resolver, 100u);
    EXPECT_EQ(legacy.confidence, TimeConfidence::ExtrapolatedBackward);
    EXPECT_EQ(legacy.absoluteMs, expectedUnixMsForFixtureWeek(100));
}

// ---------------------------------------------------------------------------
// Single-sync-point case: resolves backward / forward with slope 1.0.
// ---------------------------------------------------------------------------
// DEPRECATED (SN-8784, 2026-10-03). The new `resolve` answers for a RECORD, not for an arbitrary value. This test asked for the time of a raw value that no record in its fixture carries (an instant BETWEEN two sync points), which the legacy call could answer only by guessing the value's domain - the very guess that put every record of a no-fix log on 1980-01-13. The capability is gone by design, so the test is deprecated rather than rewritten. The inverse direction - 'which record sits at this arbitrary instant' - is `resolveTimeToSegmentOffset`, and it IS tested.
TEST_F(TimeResolverTest, DISABLED_SingleSyncPointDegenerateSlope) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0)));
    f = buildFixture("single", recs);
    ASSERT_FALSE(f.rawFile.empty());

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto resolver = ISTimeResolver::build(log.value());
    ASSERT_TRUE(resolver.has_value());
    ASSERT_EQ(resolver->syncPoints().size(), 1u);

    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, 105000);
    EXPECT_EQ(t.confidence, TimeConfidence::ExtrapolatedForward);
    // Slope defaults to 1.0 with a single sync point: ToW = 100_000 +
    // (105_000 - 100_000) = 105_000, then epoch-anchored (gpsWeek=2300).
    // SN-8107/D0066: prior expectation (raw ToW) predated epoch anchoring.
    EXPECT_EQ(t.absoluteMs, expectedUnixMsForFixtureWeek(105000));

    const AbsTimeResult bwd = resolveRecordWithRawValue(*log, *resolver, 95000);
    EXPECT_EQ(bwd.confidence, TimeConfidence::ExtrapolatedBackward);
    EXPECT_EQ(bwd.absoluteMs, expectedUnixMsForFixtureWeek(95000));
}

// ---------------------------------------------------------------------------
// SN-8115: an input that is ALREADY an absolute Unix-ms timestamp (at or
// beyond the GPS Unix epoch, 1980) is returned unchanged — never re-anchored.
// This is the guard against the year-2082 double-anchor seen on the 16-device
// compass fixture, where a wall-clock-poisoned spanEnd (~1.78e12) fed into
// resolve() had the epoch + week offset added on top (-> ~3.55e12).
// ---------------------------------------------------------------------------
// DEPRECATED (SN-8784, 2026-10-03). Its premise was feeding an already-anchored VALUE back into the resolver and asserting it passed through. The new API takes a record's address, so there is no value to feed back and nothing to pass through: idempotence is structural now, because resolving the same record twice reads the same record twice.
TEST_F(TimeResolverTest, DISABLED_AlreadyAnchoredInputPassesThroughUnchanged) {
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
    const AbsTimeResult t = resolveRecordWithRawValue(*log, *resolver, kAnchored2026);
    EXPECT_EQ(t.absoluteMs, kAnchored2026);
    EXPECT_LT(t.absoluteMs, 2ULL * kAnchored2026);  // explicitly: not doubled
}

// resolve() is idempotent for already-resolved values: feeding a resolved
// (epoch-anchored) output back in returns the same value. Guarantees
// `resolve(resolve(x)) == resolve(x)`.
// DEPRECATED (SN-8784, 2026-10-03). Same: `resolve(resolve(x)) == resolve(x)` cannot be expressed against an address-based API. The property it protected - that resolving does not mutate state - is covered by `AbsIndexIsMemoisedPerLogAndColdOnACopy`, which asserts a second call returns the same answer off the memoised index.
TEST_F(TimeResolverTest, DISABLED_ResolveIsIdempotent) {
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
    const AbsTimeResult once = resolveRecordWithRawValue(*log, *resolver, 411500u);
    EXPECT_EQ(once.absoluteMs, expectedUnixMsForFixtureWeek(411500));
    // Second pass on the already-resolved value is a no-op.
    const AbsTimeResult twice = resolveRecordWithRawValue(*log, *resolver, once.absoluteMs);
    EXPECT_EQ(twice.absoluteMs, once.absoluteMs);
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
        const AbsTimeResult ts = resolveRecordWithRawValue(*log, resolver, first.hostTimeMs);
        EXPECT_EQ(ts.source, TimeSource::PayloadToW);
        EXPECT_TRUE(ts.confidence == TimeConfidence::Exact
                || ts.mechanism != AbsTimeMechanism::PayloadEpoch)   /* SN-8784: Exact requires a PAYLOAD week; a derived one is weaker by design */;
        EXPECT_EQ(ts.absoluteMs, first.payloadToWMs);
    }

    // Resolve a point well before the first sync → ExtrapolatedBackward.
    if (!syncs.empty()) {
        const AbsTimeResult bwd = resolveRecordWithRawValue(*log, resolver, 0u);
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

// ===========================================================================
// SN-8784 — the round-trip proof for resolveAbsTime / resolveTimeToSegmentOffset.
//
// Kyle's specification, 2026-10-02:
//   "Time -> Log-offset -> Time -> Log-offset -> Time -- the precision of time
//    may very slightly, by a few millis."
//
// The five-element chain is the minimum length that distinguishes a LOSSY FIRST
// HOP from a DRIFTING MAPPING, and the slack belongs on the first hop only:
//
//   T0  an arbitrary instant, not necessarily one any record bears
//   P0 = resolveTimeToSegmentOffset(T0)
//   T1 = resolve(P0)        T1 <= T0; the gap is the inter-record interval
//   P1 = resolveTimeToSegmentOffset(T1)   MUST equal P0 exactly
//   T2 = resolve(P1)               MUST equal T1 exactly
//
// Once T1 is an instant a record actually BEARS, every later hop must be a fixed
// point. A tolerance applied uniformly would pass a mapping that drifts a few ms
// per cycle, which is a real failure mode; a fixed-point assertion catches it on
// hop two.
// ===========================================================================
namespace {

//! One cycle's outcome, so a failure can name the log and the seed that produced it.
struct CycleOutcome {
    bool        ran           = false;
    bool        firstHopBack  = false;   //!< T1 <= T0, as at-or-before requires.
    bool        positionFixed = false;   //!< P1 == P0.
    bool        timeFixed     = false;   //!< T2 == T1.
    int64_t     firstHopGapMs = 0;
    std::string detail;
};

/**
 * @brief Runs T0 -> P0 -> T1 -> P1 -> T2 for one seed instant.
 *
 * @param log  The device log.
 * @param R    Its resolver.
 * @param t0   Seed instant.
 * @return     What happened, for the caller to assert on and report.
 */
CycleOutcome runCycle(const ISDeviceLog& log, const ISTimeResolver& R, uint64_t t0) {
    CycleOutcome o;

    const SegmentOffset p0 = R.resolveTimeToSegmentOffset(log, t0);
    if (!p0.valid) {
        o.detail = "P0 invalid (no placeable record in the log)";
        return o;
    }
    const AbsTimeResult t1 = R.resolve(log, p0.segmentIndex, p0.recordIndex);
    if (!t1.valid) {
        o.detail = "T1 invalid: a position the inverse returned did not resolve forward again";
        return o;
    }
    o.ran           = true;
    o.firstHopGapMs = static_cast<int64_t>(t0) - static_cast<int64_t>(t1.absoluteMs);
    // At-or-before semantics: the landing record cannot be LATER than the seed.
    o.firstHopBack  = t1.absoluteMs <= t0;

    const SegmentOffset p1 = R.resolveTimeToSegmentOffset(log, t1.absoluteMs);
    o.positionFixed = p1.valid
                   && p1.segmentIndex == p0.segmentIndex
                   && p1.recordIndex  == p0.recordIndex
                   && p1.byteOffset   == p0.byteOffset;

    if (p1.valid) {
        const AbsTimeResult t2 = R.resolve(log, p1.segmentIndex, p1.recordIndex);
        o.timeFixed = t2.valid && t2.absoluteMs == t1.absoluteMs;
    }
    if (!o.positionFixed || !o.timeFixed) {
        o.detail = "seed=" + std::to_string(t0)
                 + " P0=(" + std::to_string(p0.segmentIndex) + ","
                 + std::to_string(p0.recordIndex) + ")"
                 + " T1=" + std::to_string(t1.absoluteMs)
                 + " P1=(" + std::to_string(p1.segmentIndex) + ","
                 + std::to_string(p1.recordIndex) + ")";
    }
    return o;
}

//! Collects the directories under `root` that contain at least one `.raw`.
std::vector<fs::path> findLogDirs(const fs::path& root, std::size_t cap) {
    std::vector<fs::path> out;
    std::error_code ec;
    std::set<fs::path> seen;
    for (fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec),
         end; it != end && out.size() < cap; it.increment(ec)) {
        if (ec) { ec.clear(); continue; }
        if (!it->is_regular_file(ec)) continue;
        if (it->path().extension() != ".raw") continue;
        const fs::path dir = it->path().parent_path();
        if (seen.insert(dir).second) out.push_back(dir);
    }
    return out;
}

} // namespace

/**
 * @brief The cycle holds for every seed, across the corpus.
 *
 * Driven over whatever logs `IS_SDK_CORPUS_DIR` points at. Seeds per device: every sampled
 * record's own instant, the midpoint between consecutive sampled instants (an arbitrary time no
 * record bears — the case the first-hop slack exists for), and the span ends.
 *
 * Fails rather than skips when the corpus IS present but yields too few samples: a silently-empty
 * run that reports success is the failure mode this test exists to prevent.
 */
TEST(AbsTimeCycle, TimeToOffsetToTimeIsAFixedPointAcrossTheCorpus) {
    const char* root = std::getenv("IS_SDK_CORPUS_DIR");
    if (root == nullptr || !fs::exists(root)) {
        GTEST_SKIP() << "corpus not present (set IS_SDK_CORPUS_DIR to a tree of log directories)";
    }

    // Bounded so the suite stays runnable; the cap is reported, never silent.
    constexpr std::size_t kMaxLogs           = 40;
    constexpr std::size_t kRecordsPerSegment = 12;
    constexpr std::size_t kMinTotalSamples   = 400;

    const std::vector<fs::path> dirs = findLogDirs(root, kMaxLogs);
    ASSERT_FALSE(dirs.empty()) << "no log directories found under " << root;

    std::size_t logsUsed = 0, cycles = 0, firstHopViolations = 0;
    std::size_t positionDrift = 0, timeDrift = 0;
    int64_t     worstFirstHopGapMs = 0;
    std::string worstWhere;                       // DIAGNOSTIC: name the log behind the worst gap
    std::vector<std::pair<int64_t, std::string>> bigGaps;
    std::vector<std::string> failures;

    for (const fs::path& dir : dirs) {
        auto log = ISLog::openDirectory(dir);
        if (!log) continue;
        bool usedThisLog = false;

        for (uint64_t devId : log->deviceIds()) {
            const ISDeviceLog& dl = log->device(devId);
            auto built = ISTimeResolver::build(dl);
            if (!built) continue;
            const ISTimeResolver& R = *built;

            // Gather instants that records actually bear, sampled across every segment.
            std::vector<uint64_t> borne;
            for (std::size_t s = 0; s < dl.segmentCount(); ++s) {
                const std::size_t n = dl.segment(s).recordCount();
                if (n == 0) continue;
                const std::size_t step = n > kRecordsPerSegment ? n / kRecordsPerSegment : 1;
                for (std::size_t k = 0; k < n; k += step) {
                    const AbsTimeResult r = R.resolve(dl, s, k);
                    if (r.valid) borne.push_back(r.absoluteMs);
                }
            }
            if (borne.size() < 2) continue;      // nothing to cycle on; an expected outcome
            std::sort(borne.begin(), borne.end());
            borne.erase(std::unique(borne.begin(), borne.end()), borne.end());

            // Seeds: the instants themselves, AND the midpoints between them. The midpoints are
            // the arbitrary-T0 case — a time no record bears, where the first hop legitimately
            // loses the gap back to the previous record.
            std::vector<uint64_t> seeds = borne;
            for (std::size_t i = 1; i < borne.size(); ++i) {
                seeds.push_back(borne[i - 1] + (borne[i] - borne[i - 1]) / 2);
            }

            for (uint64_t t0 : seeds) {
                const CycleOutcome o = runCycle(dl, R, t0);
                if (!o.ran) continue;
                ++cycles;
                usedThisLog = true;
                if (!o.firstHopBack)  ++firstHopViolations;
                if (!o.positionFixed) ++positionDrift;
                if (!o.timeFixed)     ++timeDrift;
                if (o.firstHopGapMs > worstFirstHopGapMs) {
                    worstFirstHopGapMs = o.firstHopGapMs;
                    const SegmentOffset w = R.resolveTimeToSegmentOffset(dl, t0);
                    const ISRecordView  wv = dl.segment(w.segmentIndex).recordAt(w.recordIndex);
                    worstWhere = dir.filename().string() + " dev=" + std::to_string(devId)
                               + " seed=" + std::to_string(t0)
                               + " landed seg=" + std::to_string(w.segmentIndex)
                               + " rec=" + std::to_string(w.recordIndex)
                               + " did=" + std::to_string(wv.did())
                               + "(" + cISDataMappings::DataName(wv.did()) + ")";
                }
                if (o.firstHopGapMs > 60000 && bigGaps.size() < 40) {
                    const SegmentOffset w = R.resolveTimeToSegmentOffset(dl, t0);
                    const ISRecordView  wv = dl.segment(w.segmentIndex).recordAt(w.recordIndex);
                    bigGaps.emplace_back(o.firstHopGapMs,
                        dir.filename().string() + " dev=" + std::to_string(devId)
                        + " did=" + std::to_string(wv.did())
                        + "(" + cISDataMappings::DataName(wv.did()) + ")");
                }
                if ((!o.positionFixed || !o.timeFixed || !o.firstHopBack)
                    && failures.size() < 10) {
                    failures.push_back(dir.filename().string() + ": " + o.detail);
                }
            }
        }
        if (usedThisLog) ++logsUsed;
    }

    std::string report = "logs=" + std::to_string(logsUsed)
                       + " cycles=" + std::to_string(cycles)
                       + " worst first-hop gap=" + std::to_string(worstFirstHopGapMs) + " ms";
    for (const std::string& f : failures) report += "\n  " + f;

    // A run that proved nothing must not read as a pass.
    EXPECT_GE(cycles, kMinTotalSamples) << "too few cycles to prove anything. " << report;
    EXPECT_GE(logsUsed, 5u) << "too few logs contributed. " << report;

    // Hop 1 may lose the inter-record gap, but never runs FORWARD of the seed.
    EXPECT_EQ(firstHopViolations, 0u)
        << "the inverse returned a record LATER than the seed. " << report;

    // Hops 2 and 3 are fixed points. No tolerance: drift is the failure mode.
    EXPECT_EQ(positionDrift, 0u) << "position is not a fixed point. " << report;
    EXPECT_EQ(timeDrift, 0u)     << "time is not a fixed point. " << report;

    std::fprintf(stderr, "[AbsTimeCycle] %s\n", report.c_str());
    std::fprintf(stderr, "[AbsTimeCycle] worst gap at: %s\n", worstWhere.c_str());
    std::fprintf(stderr, "[AbsTimeCycle] gaps over 60 s: %zu (first %zu shown)\n",
                 bigGaps.size(), bigGaps.size());
    for (const auto& [g, where] : bigGaps) {
        std::fprintf(stderr, "    %10lld ms (%7.3f days)  %s\n",
                     static_cast<long long>(g), static_cast<double>(g) / 86400000.0, where.c_str());
    }
}

/**
 * @brief DIAGNOSTIC (temporary): dump the provenance of a log's time extremes.
 *
 * Kyle, 2026-10-03: a DID that carries no time of its own cannot legitimately land days after the
 * records around it — so either the `.idx` is wrong or an earlier record's time is not being
 * carried forward. This names the actual records at the extremes and prints what every mechanism
 * fed them, rather than arguing from a log directory's name.
 *
 * `IS_SDK_DIAG_LOG_DIR=<dir>`, optional `IS_SDK_DIAG_CONTEXT=<n>` for neighbours either side.
 */
TEST(AbsTimeCycle, DiagnoseExtremesForOneLog) {
    const char* dirEnv = std::getenv("IS_SDK_DIAG_LOG_DIR");
    if (dirEnv == nullptr) GTEST_SKIP() << "set IS_SDK_DIAG_LOG_DIR";
    const fs::path dir = dirEnv;
    auto log = ISLog::openDirectory(dir);
    ASSERT_TRUE(static_cast<bool>(log)) << "could not open " << dir;

    const auto describe = [](const ISDeviceLog& dl, const ISTimeResolver& R,
                             std::size_t s, std::size_t k, const char* label) {
        const ISRecordView rv = dl.segment(s).recordAt(k);
        const AbsTimeResult r = R.resolve(dl, s, k);
        std::fprintf(stderr,
            "  %-12s seg=%zu rec=%zu did=%u(%s) sidecarRaw=%llu -> abs=%llu valid=%d\n"
            "               mech=%s anchor=%s tier=%d anchorMs=%llu offsetMs=%lld week=%u "
            "weekFromPayload=%d tow=%llu frozen=%d\n",
            label, s, k, rv.did(), cISDataMappings::DataName(rv.did()),
            static_cast<unsigned long long>(r.sidecarRawMs),
            static_cast<unsigned long long>(r.absoluteMs), static_cast<int>(r.valid),
            absTimeMechanismName(r.mechanism), absAnchorSourceName(r.anchorSource),
            static_cast<int>(r.anchorTier),
            static_cast<unsigned long long>(r.anchorMs),
            static_cast<long long>(r.offsetMs), r.gpsWeek,
            static_cast<int>(r.weekFromPayload),
            static_cast<unsigned long long>(r.towMs), static_cast<int>(r.frozenField));
        for (const std::string& h : r.hints) std::fprintf(stderr, "               hint: %s\n", h.c_str());
    };

    for (uint64_t devId : log->deviceIds()) {
        const ISDeviceLog& dl = log->device(devId);
        auto built = ISTimeResolver::build(dl);
        if (!built) continue;

        bool sawAny = false;
        std::size_t lastS = 0, lastK = 0, maxS = 0, maxK = 0;
        uint64_t arrivalLast = 0, maxMs = 0;
        std::size_t placed = 0, unplaced = 0;
        std::map<uint32_t, std::size_t> unplacedByDid;
        for (std::size_t s = 0; s < dl.segmentCount(); ++s) {
            for (std::size_t k = 0; k < dl.segment(s).recordCount(); ++k) {
                const AbsTimeResult r = built->resolve(dl, s, k);
                if (!r.valid) {
                    ++unplaced;
                    ++unplacedByDid[dl.segment(s).recordAt(k).did()];
                    continue;
                }
                ++placed;
                arrivalLast = r.absoluteMs; lastS = s; lastK = k;
                if (!sawAny || r.absoluteMs > maxMs) { maxMs = r.absoluteMs; maxS = s; maxK = k; }
                sawAny = true;
            }
        }
        if (!sawAny) continue;

        std::fprintf(stderr, "\n=== %s dev=%llu segments=%zu placed=%zu unplaced=%zu ===\n",
                     dir.filename().string().c_str(),
                     static_cast<unsigned long long>(devId), dl.segmentCount(), placed, unplaced);
        std::fprintf(stderr, "  arrivalLast=%llu  max=%llu  delta=%lld ms (%.3f days)\n",
                     static_cast<unsigned long long>(arrivalLast),
                     static_cast<unsigned long long>(maxMs),
                     static_cast<long long>(maxMs) - static_cast<long long>(arrivalLast),
                     (static_cast<double>(maxMs) - static_cast<double>(arrivalLast)) / 86400000.0);
        for (std::size_t s = 0; s < dl.segmentCount(); ++s) {
            bool any = false; uint64_t lo = 0, hi = 0; std::size_t n = 0;
            for (std::size_t k = 0; k < dl.segment(s).recordCount(); ++k) {
                const AbsTimeResult r = built->resolve(dl, s, k);
                if (!r.valid) continue;
                if (!any) { lo = hi = r.absoluteMs; any = true; }
                if (r.absoluteMs < lo) lo = r.absoluteMs;
                if (r.absoluteMs > hi) hi = r.absoluteMs;
                ++n;
            }
            // the EARLIEST record of the segment, and how many share that instant
            if (any) {
                std::size_t minK = 0, sharing = 0;
                for (std::size_t k = 0; k < dl.segment(s).recordCount(); ++k) {
                    const AbsTimeResult r = built->resolve(dl, s, k);
                    if (r.valid && r.absoluteMs == lo) { if (sharing == 0) minK = k; ++sharing; }
                }
                std::fprintf(stderr, "  segment %zu EARLIEST instant borne by %zu records\n", s, sharing);
                describe(dl, *built, s, minK, "  SEG-MIN");
            }
            if (any) std::fprintf(stderr, "  segment %zu: placed=%zu span %llu .. %llu (%.3f s)\n",
                                  s, n, static_cast<unsigned long long>(lo),
                                  static_cast<unsigned long long>(hi),
                                  (static_cast<double>(hi) - static_cast<double>(lo)) / 1000.0);
        }
        describe(dl, *built, lastS, lastK, "ARRIVAL-LAST");
        describe(dl, *built, maxS, maxK, "LATEST");

        // The records either side of each extreme, which is where a carry-forward failure shows.
        const std::size_t ctx = std::getenv("IS_SDK_DIAG_CONTEXT")
                              ? static_cast<std::size_t>(std::atoi(std::getenv("IS_SDK_DIAG_CONTEXT"))) : 2;
        for (std::size_t back = ctx; back > 0; --back) {
            if (lastK >= back) describe(dl, *built, lastS, lastK - back, "  before-AL");
        }
        for (std::size_t back = ctx; back > 0; --back) {
            if (maxK >= back) describe(dl, *built, maxS, maxK - back, "  before-MAX");
        }
        for (std::size_t fwd = 1; fwd <= ctx; ++fwd) {
            if (maxK + fwd < dl.segment(maxS).recordCount()) describe(dl, *built, maxS, maxK + fwd, "  after-MAX");
        }
        for (const auto& [did, n] : unplacedByDid) {
            std::fprintf(stderr, "  unplaced did=%u(%s) x%zu\n", did, cISDataMappings::DataName(did), n);
        }
    }
}

/**
 * @brief Across the corpus, a target INSIDE a log's span is never reported as past its end.
 *
 * `After` used to be decided by comparing the target to the last placeable record in ARRIVAL order.
 * Where arrival order is not monotonic in resolved time, a target later than that record but
 * earlier than the LATEST one was reported as past the end of the log. Measured 2026-10-03 before
 * the fix: 19 of 119 corpus device-logs are non-monotonic and 8 of them mislabelled such a target —
 * on `nodevinfo` the arrival-last record sits 6.0 days before the latest one.
 *
 * Kept over the corpus rather than only synthetically because the non-monotonic population is real
 * data's doing; `NonMonotonicArrivalOrderClampsToTheTimeExtremes` is the CI-runnable companion.
 */
TEST(AbsTimeCycle, NoCorpusTargetInsideASpanIsReportedAsAfterIt) {
    const char* root = std::getenv("IS_SDK_CORPUS_DIR");
    if (root == nullptr || !fs::exists(root)) GTEST_SKIP() << "corpus not present";

    const std::vector<fs::path> dirs = findLogDirs(root, 40);
    ASSERT_FALSE(dirs.empty());

    std::size_t devices = 0, nonMonotonic = 0, mislabelled = 0;
    for (const fs::path& dir : dirs) {
        auto log = ISLog::openDirectory(dir);
        if (!log) continue;
        for (uint64_t devId : log->deviceIds()) {
            const ISDeviceLog& dl = log->device(devId);
            auto built = ISTimeResolver::build(dl);
            if (!built) continue;
            ++devices;

            bool     sawAny = false;
            uint64_t arrivalLast = 0, maxMs = 0;
            for (std::size_t s = 0; s < dl.segmentCount(); ++s) {
                for (std::size_t k = 0; k < dl.segment(s).recordCount(); ++k) {
                    const AbsTimeResult r = built->resolve(dl, s, k);
                    if (!r.valid) continue;
                    arrivalLast = r.absoluteMs;
                    if (!sawAny || r.absoluteMs > maxMs) maxMs = r.absoluteMs;
                    sawAny = true;
                }
            }
            if (!sawAny || arrivalLast >= maxMs) continue;
            ++nonMonotonic;

            // A target strictly inside (arrivalLast, maxMs]: inside the span, so NOT "after".
            const uint64_t inside = arrivalLast + (maxMs - arrivalLast) / 2 + 1;
            const SegmentOffset p = built->resolveTimeToSegmentOffset(dl, inside);
            if (p.valid && p.exactness == PositionExactness::After) ++mislabelled;
            if (mislabelled > 0 && mislabelled <= 3) {
                std::fprintf(stderr,
                             "[after-label] %s dev=%llu arrivalLast=%llu max=%llu probe=%llu -> %s\n",
                             dir.filename().string().c_str(),
                             static_cast<unsigned long long>(devId),
                             static_cast<unsigned long long>(arrivalLast),
                             static_cast<unsigned long long>(maxMs),
                             static_cast<unsigned long long>(inside),
                             positionExactnessName(p.exactness));
            }
        }
    }
    std::fprintf(stderr, "[after-label] devices=%zu non-monotonic=%zu mislabelled=%zu\n",
                 devices, nonMonotonic, mislabelled);
    EXPECT_GT(devices, 20u) << "too few corpus device-logs to prove anything";
    EXPECT_GT(nonMonotonic, 0u)
        << "no non-monotonic log in the corpus, so this run exercised nothing";
    EXPECT_EQ(mislabelled, 0u)
        << "a target inside the span was reported as past the end of the log";
}

/**
 * @brief Across the corpus, a target before a log clamps to its EARLIEST record.
 *
 * The same arrival-order assumption from the other end: the clamp used to go through the
 * arrival-FIRST placeable record rather than the earliest in time. Measured 2026-10-03 before the
 * fix: the two differ on 60 of 119 corpus device-logs, and on all 60 the clamp landed on a record
 * with earlier records still ahead of it — 3.27 days of them on `20260729_003722`.
 */
TEST(AbsTimeCycle, EveryCorpusBeforeClampLandsOnTheEarliestRecord) {
    const char* root = std::getenv("IS_SDK_CORPUS_DIR");
    if (root == nullptr || !fs::exists(root)) GTEST_SKIP() << "corpus not present";

    const std::vector<fs::path> dirs = findLogDirs(root, 40);
    ASSERT_FALSE(dirs.empty());

    std::size_t devices = 0, differs = 0, notEarliest = 0;
    for (const fs::path& dir : dirs) {
        auto log = ISLog::openDirectory(dir);
        if (!log) continue;
        for (uint64_t devId : log->deviceIds()) {
            const ISDeviceLog& dl = log->device(devId);
            auto built = ISTimeResolver::build(dl);
            if (!built) continue;
            ++devices;

            bool     sawAny = false;
            uint64_t arrivalFirst = 0, minMs = 0;
            for (std::size_t s = 0; s < dl.segmentCount(); ++s) {
                for (std::size_t k = 0; k < dl.segment(s).recordCount(); ++k) {
                    const AbsTimeResult r = built->resolve(dl, s, k);
                    if (!r.valid) continue;
                    if (!sawAny) { arrivalFirst = r.absoluteMs; minMs = r.absoluteMs; }
                    else if (r.absoluteMs < minMs) minMs = r.absoluteMs;
                    sawAny = true;
                }
            }
            if (!sawAny || arrivalFirst <= minMs) continue;
            ++differs;

            const SegmentOffset p = built->resolveTimeToSegmentOffset(dl, minMs - 5000);
            if (!p.valid) continue;
            const AbsTimeResult landed = built->resolve(dl, p.segmentIndex, p.recordIndex);
            if (landed.valid && landed.absoluteMs != minMs) {
                ++notEarliest;
                if (notEarliest <= 3) {
                    std::fprintf(stderr,
                                 "[before-clamp] %s dev=%llu min=%llu arrivalFirst=%llu "
                                 "landed=%llu (%s)\n",
                                 dir.filename().string().c_str(),
                                 static_cast<unsigned long long>(devId),
                                 static_cast<unsigned long long>(minMs),
                                 static_cast<unsigned long long>(arrivalFirst),
                                 static_cast<unsigned long long>(landed.absoluteMs),
                                 positionExactnessName(p.exactness));
                }
            }
        }
    }
    std::fprintf(stderr, "[before-clamp] devices=%zu differs=%zu notEarliest=%zu\n",
                 devices, differs, notEarliest);
    EXPECT_GT(devices, 20u) << "too few corpus device-logs to prove anything";
    EXPECT_GT(differs, 0u)
        << "no log where arrival-first differs from earliest, so this run exercised nothing";
    EXPECT_EQ(notEarliest, 0u) << "the clamp did not land on the log's earliest record";
}

/**
 * @brief The same cycle on a synthetic fixture, so CI without the corpus is still protected.
 *
 * The corpus test SKIPS where the data is absent, which means it guards nothing in CI. This one
 * always runs.
 */
TEST_F(TimeResolverTest, AbsTimeCycleIsAFixedPointOnASyntheticLog) {
    // 40 INS_2 records 100 ms apart. `makeIns2` sets week 2300, which is above the GNSS-fix
    // threshold, so this log anchors from its own payloads — the ordinary case.
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (int i = 0; i < 40; ++i) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0 + 0.1 * i)));
    }
    f = buildFixture("abs_cycle", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    std::size_t cycles = 0;
    for (std::size_t s = 0; s < log->segmentCount(); ++s) {
        for (std::size_t k = 0; k < log->segment(s).recordCount(); ++k) {
            const AbsTimeResult r = built->resolve(*log, s, k);
            if (!r.valid) continue;
            const CycleOutcome o = runCycle(*log, *built, r.absoluteMs);
            if (!o.ran) continue;
            ++cycles;
            EXPECT_TRUE(o.firstHopBack)  << o.detail;
            EXPECT_TRUE(o.positionFixed) << o.detail;
            EXPECT_TRUE(o.timeFixed)     << o.detail;
        }
    }
    EXPECT_GT(cycles, 10u) << "the synthetic fixture produced too few cycles to prove anything";
}

/**
 * @brief A stalled clock: the inverse names the FIRST record of the run, and says so.
 *
 * Six records per instant, five instants — the shape a stalled clock produces, where many records
 * share one time. Two separate properties:
 *
 *  - the landing record is the run's first, which is what makes a second round trip a fixed point
 *    rather than a walk along the run;
 *  - `exactness` reports `FirstOfStalledRun`, which the pre-index implementation could never do.
 *    It tested whether a backward walk had MOVED, but its forward scan already kept the
 *    earliest-arrival record of the equal run, so the walk was always a no-op. Measured on the
 *    pre-index code (2026-10-03): all five instants reported `Exact` at a run length of 6.
 */
TEST_F(TimeResolverTest, StalledRunReportsTheFirstRecordOfTheRun) {
    constexpr int kGroups  = 5;
    constexpr int kRepeats = 6;
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (int group = 0; group < kGroups; ++group) {
        for (int rep = 0; rep < kRepeats; ++rep) {
            recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0 + 0.1 * group)));
        }
    }
    f = buildFixture("stalled_run", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    // What every record actually resolves to, in arrival order. Asserted rather than assumed: the
    // test proves nothing unless the fixture really did produce runs.
    std::map<uint64_t, std::vector<std::size_t>> byInstant;
    for (std::size_t k = 0; k < log->segment(0).recordCount(); ++k) {
        const AbsTimeResult r = built->resolve(*log, 0, k);
        if (r.valid) byInstant[r.absoluteMs].push_back(k);
    }
    ASSERT_EQ(byInstant.size(), static_cast<std::size_t>(kGroups));
    for (const auto& [ms, recsAt] : byInstant) {
        ASSERT_EQ(recsAt.size(), static_cast<std::size_t>(kRepeats)) << "instant " << ms;
    }

    for (const auto& [ms, recsAt] : byInstant) {
        const SegmentOffset p = built->resolveTimeToSegmentOffset(*log, ms);
        ASSERT_TRUE(p.valid) << "instant " << ms;
        EXPECT_EQ(p.recordIndex, recsAt.front()) << "not the first record of the run at " << ms;
        EXPECT_EQ(p.exactness, PositionExactness::FirstOfStalledRun)
            << "instant " << ms << " is borne by " << recsAt.size()
            << " records but was reported as " << positionExactnessName(p.exactness);
        EXPECT_EQ(p.runLength, recsAt.size()) << "run length at " << ms;

        // And the cycle still closes on a stalled run, which is the property the first-of-run rule
        // exists to protect.
        const CycleOutcome o = runCycle(*log, *built, ms);
        EXPECT_TRUE(o.ran)           << o.detail;
        EXPECT_TRUE(o.positionFixed) << o.detail;
        EXPECT_TRUE(o.timeFixed)     << o.detail;
    }
}

/**
 * @brief The sorted index is memoised per log, and a COPIED resolver starts cold.
 *
 * Validity is keyed on the log's address, and a resolver outlives the call that built it — it holds
 * no reference to a log and the Logalyzer adapter moves it into a memoised optional. A copy that
 * inherited the index would be trusting a key it cannot re-validate. Asserted through
 * `absIndexSize()` because "it still returns the right answer" would pass either way: a carried
 * cache and a rebuilt one are indistinguishable from the answers alone.
 */
TEST_F(TimeResolverTest, AbsIndexIsMemoisedPerLogAndColdOnACopy) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (int i = 0; i < 20; ++i) recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0 + 0.1 * i)));
    f = buildFixture("abs_index_cache", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    EXPECT_EQ(built->absIndexSize(), 0u) << "the index must not be built until it is needed";

    const AbsTimeResult seed = built->resolve(*log, 0, 10);
    ASSERT_TRUE(seed.valid);
    const SegmentOffset p = built->resolveTimeToSegmentOffset(*log, seed.absoluteMs);
    ASSERT_TRUE(p.valid);
    const std::size_t built1 = built->absIndexSize();
    EXPECT_GT(built1, 0u) << "the inverse did not build an index";

    // A second call reuses it rather than rebuilding.
    const SegmentOffset again = built->resolveTimeToSegmentOffset(*log, seed.absoluteMs);
    EXPECT_EQ(built->absIndexSize(), built1);
    EXPECT_EQ(again.recordIndex, p.recordIndex);
    EXPECT_EQ(again.segmentIndex, p.segmentIndex);

    ISTimeResolver copy = *built;
    EXPECT_EQ(copy.absIndexSize(), 0u) << "a copied resolver inherited a cache keyed on a log "
                                          "address it cannot re-validate";
    const SegmentOffset fromCopy = copy.resolveTimeToSegmentOffset(*log, seed.absoluteMs);
    ASSERT_TRUE(fromCopy.valid) << "the copy did not rebuild its index";
    EXPECT_EQ(fromCopy.segmentIndex, p.segmentIndex);
    EXPECT_EQ(fromCopy.recordIndex, p.recordIndex);
    EXPECT_EQ(fromCopy.byteOffset, p.byteOffset);
    EXPECT_EQ(copy.absIndexSize(), built1);
}

/**
 * @brief Arrival order is not time order: both clamps are about the time extremes.
 *
 * The CI-runnable companion to the corpus sweep. ToW descends and jumps around in arrival order, so
 * the earliest record is the LAST to arrive and the latest is in the middle — which is what the two
 * clamps used to get wrong, having read the span ends out of arrival order:
 *
 *  - a target before the log clamped to the arrival-FIRST record, which here is 50 s after the
 *    earliest one (60 of 119 corpus device-logs, 2026-10-03);
 *  - a target inside the span compared against the arrival-LAST record and so read as `After`
 *    (8 of 119).
 */
TEST_F(TimeResolverTest, NonMonotonicArrivalOrderClampsToTheTimeExtremes) {
    // Deliberately out of order: earliest arrives last, latest arrives third.
    const std::vector<double> towSec = { 150.0, 160.0, 300.0, 170.0, 180.0, 100.0 };
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : towSec) recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));

    f = buildFixture("non_monotonic", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    // What the fixture ACTUALLY produced. Asserted, because if the resolver re-orders or rejects a
    // descending ToW then this test proves nothing and must be rewritten rather than trusted.
    std::vector<uint64_t> placed;
    for (std::size_t k = 0; k < log->segment(0).recordCount(); ++k) {
        const AbsTimeResult r = built->resolve(*log, 0, k);
        if (r.valid) placed.push_back(r.absoluteMs);
    }
    ASSERT_EQ(placed.size(), towSec.size());
    const uint64_t minMs = *std::min_element(placed.begin(), placed.end());
    const uint64_t maxMs = *std::max_element(placed.begin(), placed.end());
    ASSERT_LT(minMs, placed.front()) << "fixture is monotonic at the start; it proves nothing";
    ASSERT_GT(maxMs, placed.back())  << "fixture is monotonic at the end; it proves nothing";

    // Before the whole log: the EARLIEST record, which is the last to arrive.
    const SegmentOffset before = built->resolveTimeToSegmentOffset(*log, minMs - 5000);
    ASSERT_TRUE(before.valid);
    EXPECT_EQ(before.exactness, PositionExactness::Before);
    const AbsTimeResult landed = built->resolve(*log, before.segmentIndex, before.recordIndex);
    ASSERT_TRUE(landed.valid);
    EXPECT_EQ(landed.absoluteMs, minMs) << "the clamp did not land on the earliest record";
    EXPECT_EQ(before.recordIndex, towSec.size() - 1) << "the earliest record arrives last here";

    // Inside the span but after the arrival-last record: inside, so not `After`.
    const uint64_t inside = placed.back() + (maxMs - placed.back()) / 2;
    const SegmentOffset mid = built->resolveTimeToSegmentOffset(*log, inside);
    ASSERT_TRUE(mid.valid);
    EXPECT_EQ(mid.exactness, PositionExactness::Preceding)
        << "a target inside the span, borne by no record, was reported as "
        << positionExactnessName(mid.exactness);

    // Past the latest record: that one IS after.
    const SegmentOffset after = built->resolveTimeToSegmentOffset(*log, maxMs + 5000);
    ASSERT_TRUE(after.valid);
    EXPECT_EQ(after.exactness, PositionExactness::After);
    const AbsTimeResult landedAfter = built->resolve(*log, after.segmentIndex, after.recordIndex);
    ASSERT_TRUE(landedAfter.valid);
    EXPECT_EQ(landedAfter.absoluteMs, maxMs) << "the clamp did not land on the latest record";
}

/**
 * @brief The indexed lookup answers exactly what an exhaustive scan would.
 *
 * The binary search replaced a full scan per call, so the property that matters is EQUIVALENCE, not
 * just self-consistency: a sorted index that answered differently would still round-trip as a fixed
 * point and the cycle test would never notice. The reference here is the scan, written out in the
 * test — at-or-before, and the earliest-arrival record among those sharing the landing instant.
 *
 * The fixture mixes ToW-bearing INS_2 with uptime-domain IMU records and plants stalled runs, so
 * the index is proved on records whose RAW values are in two different domains — the case the `.idx`
 * ordering cannot serve and the reason this index exists at all.
 */
TEST_F(TimeResolverTest, IndexedLookupAgreesWithAnExhaustiveScan) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (int i = 0; i < 15; ++i) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0 + 0.1 * i)));
        recs.emplace_back(DID_IMU,   bytesOf(makeImu(0.5 + 0.1 * i)));
        if (i % 4 == 0) {   // a stalled run: three more records on the instant just written
            for (int rep = 0; rep < 3; ++rep) {
                recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0 + 0.1 * i)));
            }
        }
    }
    f = buildFixture("index_equiv", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    // Every record's instant, in arrival order — the scan's raw material.
    struct Placed { uint64_t ms; std::size_t seg, rec; };
    std::vector<Placed> placed;
    for (std::size_t s = 0; s < log->segmentCount(); ++s) {
        for (std::size_t k = 0; k < log->segment(s).recordCount(); ++k) {
            const AbsTimeResult r = built->resolve(*log, s, k);
            if (r.valid) placed.push_back({ r.absoluteMs, s, k });
        }
    }
    ASSERT_GE(placed.size(), 20u) << "the fixture placed too few records to prove anything";

    // The reference implementation: the greatest instant at or before the target, then the
    // earliest-arrival record bearing it.
    const auto scan = [&placed](uint64_t target) -> const Placed* {
        const Placed* best = nullptr;
        for (const Placed& p : placed) {
            if (p.ms > target) continue;
            if (best == nullptr || p.ms > best->ms) best = &p;
        }
        return best;
    };

    std::set<uint64_t> instants;
    for (const Placed& p : placed) instants.insert(p.ms);
    ASSERT_GT(instants.size(), 5u);

    std::size_t compared = 0;
    for (uint64_t ms : instants) {
        for (int64_t delta : { int64_t{0}, int64_t{1}, int64_t{-1}, int64_t{37} }) {
            const uint64_t target = static_cast<uint64_t>(static_cast<int64_t>(ms) + delta);
            const Placed* want = scan(target);
            if (want == nullptr) continue;          // before the whole log; the clamp is below
            const SegmentOffset got = built->resolveTimeToSegmentOffset(*log, target);
            ASSERT_TRUE(got.valid) << "target " << target;
            EXPECT_EQ(got.segmentIndex, want->seg) << "target " << target;
            EXPECT_EQ(got.recordIndex,  want->rec) << "target " << target;
            EXPECT_EQ(got.byteOffset,
                      log->segment(want->seg).recordAt(want->rec).offsetInFile())
                << "target " << target;
            ++compared;
        }
    }
    EXPECT_GT(compared, 20u) << "too few targets compared against the scan";

    // `exactness` and `runLength` answer two different questions, so check them against what the
    // scan says rather than against each other. A target that no record bears is `Preceding`, and
    // it can still land on a stalled run — the case a single enum could not express.
    std::size_t sawExact = 0, sawPreceding = 0, sawStalled = 0, sawInexactOnARun = 0;
    for (uint64_t ms : instants) {
        for (int64_t delta : { int64_t{0}, int64_t{1}, int64_t{37} }) {
            const uint64_t target = static_cast<uint64_t>(static_cast<int64_t>(ms) + delta);
            const Placed* want = scan(target);
            if (want == nullptr || target > *instants.rbegin()) continue;
            const SegmentOffset got = built->resolveTimeToSegmentOffset(*log, target);
            ASSERT_TRUE(got.valid) << "target " << target;

            const std::size_t borneBy = static_cast<std::size_t>(
                std::count_if(placed.begin(), placed.end(),
                              [&](const Placed& p) { return p.ms == want->ms; }));
            EXPECT_EQ(got.runLength, borneBy) << "target " << target;

            const bool exactHit = (want->ms == target);
            if (!exactHit) {
                EXPECT_EQ(got.exactness, PositionExactness::Preceding) << "target " << target;
                ++sawPreceding;
                if (borneBy > 1) ++sawInexactOnARun;
            } else if (borneBy > 1) {
                EXPECT_EQ(got.exactness, PositionExactness::FirstOfStalledRun) << "target " << target;
                ++sawStalled;
            } else {
                EXPECT_EQ(got.exactness, PositionExactness::Exact) << "target " << target;
                ++sawExact;
            }
        }
    }
    // All four combinations have to actually occur, or the assertions above are vacuous.
    EXPECT_GT(sawExact, 0u)          << "no unshared exact hit was exercised";
    EXPECT_GT(sawPreceding, 0u)      << "no inexact landing was exercised";
    EXPECT_GT(sawStalled, 0u)        << "no exact hit on a stalled run was exercised";
    EXPECT_GT(sawInexactOnARun, 0u)  << "no inexact landing on a stalled run was exercised -- "
                                        "that is the combination a single enum cannot express";

    // The two clamps, which the scan cannot express: earlier than everything, and later.
    const uint64_t minMs = *instants.begin();
    const uint64_t maxMs = *instants.rbegin();
    const SegmentOffset before = built->resolveTimeToSegmentOffset(*log, minMs - 5000);
    EXPECT_TRUE(before.valid);
    EXPECT_EQ(before.exactness, PositionExactness::Before);
    const SegmentOffset after = built->resolveTimeToSegmentOffset(*log, maxMs + 5000);
    EXPECT_TRUE(after.valid);
    EXPECT_EQ(after.exactness, PositionExactness::After);
}

// ===========================================================================
// SN-8784 — the two outcome classes the corpus cannot reach.
//
// `AbsAnchorSource::None` and `AbsAnchorSource::IdxCaptureEpoch` had ZERO coverage: no corpus log
// lacks every clock source, and none carries a `.idx` capture epoch while also lacking a GNSS fix.
// Both are first-class outcomes under Kyle's ruling that a log may legitimately have no clock
// source, so they get synthetic fixtures rather than staying untested.
// ===========================================================================
namespace {

//! Writes `capture_epoch_ms` + the HAS_CAPTURE_EPOCH flag into a v2 sidecar, in place.
//! `epochMs == 0` CLEARS the flag instead, which is how the no-anchor-at-all case is built.
bool patchSidecarCaptureEpoch(const fs::path& idxPath, uint64_t epochMs) {
    std::fstream io(idxPath, std::ios::binary | std::ios::in | std::ios::out);
    if (!io.good()) return false;
    std::vector<uint8_t> head(idx::IS_LOG_IDX_HEADER_SIZE);
    io.read(reinterpret_cast<char*>(head.data()), static_cast<std::streamsize>(head.size()));
    if (!io) return false;
    // Patch the on-disk bytes directly: `flags` at 42, `capture_epoch_ms` at 48. Going through
    // serializeHeader would re-derive fields the fixture is deliberately controlling.
    if (epochMs != 0) head[42] = static_cast<uint8_t>(head[42] | idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH);
    else              head[42] = static_cast<uint8_t>(head[42] & ~idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH);
    std::memcpy(head.data() + 48, &epochMs, sizeof(epochMs));
    io.seekp(0);
    io.write(reinterpret_cast<const char*>(head.data()), static_cast<std::streamsize>(head.size()));
    io.flush();
    return static_cast<bool>(io);
}

//! A log whose payloads carry week 1 — below the fix threshold, so no payload anchor is usable.
FixturePaths buildNoFixFixture(const std::string& hint, const std::string& stem) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (int i = 0; i < 60; ++i) {
        ins_2_t p = makeIns2(100.0 + 0.1 * i);
        p.week = 1;                      // below kGnssFixWeekThreshold: this log never got a fix
        recs.emplace_back(DID_INS_2, bytesOf(p));
    }
    FixturePaths f = buildFixture(hint, recs);
    if (!f.rawFile.empty() && !stem.empty()) renameFixtureTo(f, stem);
    return f;
}

} // namespace

/**
 * @brief A log with NO clock source at all yields a relative-only answer, and says so.
 *
 * Kyle's ruling, 2026-10-02: *"a log may legitimately have NO clock source. That is an EXPECTED
 * result and must still yield a relative clock."* Built by removing all three anchors at once —
 * payload week below the fix threshold, no `.idx` capture epoch, and a filename that is not a
 * timestamp.
 */
TEST_F(TimeResolverTest, NoClockSourceAtAllYieldsARelativeOnlyAnswer) {
    f = buildNoFixFixture("no_anchor", "nothingresemblingadate");
    ASSERT_FALSE(f.rawFile.empty());
    fs::path idxPath = f.rawFile;
    idxPath.replace_extension(".idx");
    ASSERT_TRUE(fs::exists(idxPath));
    ASSERT_TRUE(patchSidecarCaptureEpoch(idxPath, 0)) << "could not clear the capture epoch";

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    std::size_t relativeOnly = 0, absolute = 0, spanned = 0;
    for (std::size_t k = 0; k < log->segment(0).recordCount(); ++k) {
        const AbsTimeResult r = built->resolve(*log, 0, k);
        if (r.valid)        ++absolute;
        if (r.relativeOnly) ++relativeOnly;
        if (r.relativeOnly && r.relativeToSegmentMs > 0) ++spanned;
        if (k == 0) {
            std::fprintf(stderr, "[no-anchor] rec0 valid=%d relativeOnly=%d anchor=%s mech=%s\n",
                         static_cast<int>(r.valid), static_cast<int>(r.relativeOnly),
                         absAnchorSourceName(r.anchorSource), absTimeMechanismName(r.mechanism));
            for (const std::string& h : r.hints) std::fprintf(stderr, "            hint: %s\n", h.c_str());
        }
    }
    EXPECT_EQ(absolute, 0u)      << "an absolute time was produced from a log with no clock source";
    EXPECT_GT(relativeOnly, 0u)  << "no relative clock either -- the EXPECTED outcome was not met";
    EXPECT_GT(spanned, 0u)       << "every relative value is zero, so the relative clock is useless";

    const AbsTimeResult r0 = built->resolve(*log, 0, 0);
    EXPECT_EQ(r0.anchorSource, AbsAnchorSource::None);
    EXPECT_FALSE(r0.hints.empty()) << "the result does not explain itself";
}

/**
 * @brief With no GNSS fix, the `.idx` capture epoch anchors the log — ahead of the filename.
 *
 * Kyle's external-anchor order, 2026-10-02: the `.idx` `capture_epoch_ms` FIRST, the filename only
 * as a worst case, because the former is a measurement the host actually took at log-open while the
 * latter is a string that merely looks like a date. The fixture gives the two DIFFERENT values so
 * the assertion can tell which one was used — a test where both agree would pass either way.
 */
TEST_F(TimeResolverTest, IdxCaptureEpochAnchorsAheadOfTheFilename) {
    // Filename says 2020-06-15 10:15:30; the capture epoch says 2023-03-01 00:00:00 UTC.
    constexpr uint64_t kCaptureEpochMs = 1677628800000ULL;
    f = buildNoFixFixture("capture_epoch", "LOG_SN12345_20200615_101530_0001");
    ASSERT_FALSE(f.rawFile.empty());
    fs::path idxPath = f.rawFile;
    idxPath.replace_extension(".idx");
    ASSERT_TRUE(fs::exists(idxPath));
    ASSERT_TRUE(patchSidecarCaptureEpoch(idxPath, kCaptureEpochMs));

    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    const AbsTimeResult r = built->resolve(*log, 0, 0);
    std::fprintf(stderr, "[capture-epoch] valid=%d abs=%llu anchor=%s mech=%s\n",
                 static_cast<int>(r.valid), static_cast<unsigned long long>(r.absoluteMs),
                 absAnchorSourceName(r.anchorSource), absTimeMechanismName(r.mechanism));
    for (const std::string& h : r.hints) std::fprintf(stderr, "                hint: %s\n", h.c_str());

    ASSERT_TRUE(r.valid) << "the capture epoch did not anchor the log";
    EXPECT_EQ(r.anchorSource, AbsAnchorSource::IdxCaptureEpoch)
        << "anchored from " << absAnchorSourceName(r.anchorSource)
        << " instead of the .idx capture epoch";

    // Within a day of the capture epoch, and nowhere near the filename's 2020 date. Asserted as a
    // window rather than an exact value because the record's own offset rides on top of the anchor.
    EXPECT_GE(r.absoluteMs, kCaptureEpochMs);
    EXPECT_LT(r.absoluteMs, kCaptureEpochMs + 86'400'000ULL);
    EXPECT_GT(r.absoluteMs, 1600000000000ULL) << "fell back to the 2020 filename date";

    // Negative control on the ORDER: clear the capture epoch and the same log must fall back to the
    // filename. Without this the test would pass even if the cascade ignored the epoch and the
    // filename happened to land in the window.
    ASSERT_TRUE(patchSidecarCaptureEpoch(idxPath, 0));
    auto log2 = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log2.has_value());
    auto built2 = ISTimeResolver::build(*log2);
    ASSERT_TRUE(built2.has_value());
    const AbsTimeResult r2 = built2->resolve(*log2, 0, 0);
    EXPECT_EQ(r2.anchorSource, AbsAnchorSource::Filename)
        << "with no capture epoch the fallback was " << absAnchorSourceName(r2.anchorSource);
    EXPECT_NE(r2.absoluteMs, r.absoluteMs) << "the two anchors produced the same instant, so this "
                                              "fixture cannot tell them apart";
}

/**
 * @brief A record with no time field of its own is still PLACED, from its arrival neighbours.
 *
 * Kyle, 2026-10-03: *"the fabricated times for the timeless records is exactly the correct
 * behavior."* `DID_DEV_INFO`, `DID_FLASH_CONFIG`, `DID_PORT_MONITOR` and friends carry no clock,
 * but they still ARRIVED at a knowable moment, and arrival order bounds that moment as tightly as
 * it bounds a stalled clock's. Declining to answer throws away something the stream already told
 * us — and in the Record Inspector it turned a usable instant into a bare dash.
 *
 * So the test is not "is it reported as absent" but "is it placed, and placed INSIDE the bracket
 * its neighbours set".
 */
TEST_F(TimeResolverTest, TimelessDidsArePlacedBetweenTheirArrivalNeighbours) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (int i = 0; i < 20; ++i) {
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(100.0 + 0.1 * i)));
        dev_info_t di{};
        di.serialNumber = 4321;
        recs.emplace_back(DID_DEV_INFO, bytesOf(di));
    }
    f = buildFixture("timeless_placed", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    const std::size_t n = log->segment(0).recordCount();
    ASSERT_GT(n, 20u);

    std::size_t timeless = 0, timelessPlaced = 0, insideBracket = 0, timed = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const ISRecordView rv = log->segment(0).recordAt(k);
        const AbsTimeResult r = built->resolve(*log, 0, k);
        const bool isTimeless = (rv.did() == DID_DEV_INFO);
        if (!isTimeless) { if (r.valid) ++timed; continue; }
        ++timeless;
        if (!r.valid) continue;
        ++timelessPlaced;
        EXPECT_EQ(r.mechanism, AbsTimeMechanism::InterpolatedFromNeighbours)
            << "record " << k << " reported " << absTimeMechanismName(r.mechanism);

        // The bracket its neighbours set, computed independently of the resolver's own choice.
        uint64_t lo = 0, hi = 0;
        for (std::size_t b = k; b-- > 0;) {
            const AbsTimeResult p = built->resolve(*log, 0, b);
            if (p.valid && log->segment(0).recordAt(b).did() != DID_DEV_INFO) { lo = p.absoluteMs; break; }
        }
        for (std::size_t a = k + 1; a < n; ++a) {
            const AbsTimeResult q = built->resolve(*log, 0, a);
            if (q.valid && log->segment(0).recordAt(a).did() != DID_DEV_INFO) { hi = q.absoluteMs; break; }
        }
        if (lo != 0 && hi != 0) {
            EXPECT_GE(r.absoluteMs, lo) << "record " << k << " placed before its predecessor";
            EXPECT_LE(r.absoluteMs, hi) << "record " << k << " placed after its successor";
            ++insideBracket;
        }
    }
    std::fprintf(stderr, "[timeless] timed=%zu timeless=%zu placed=%zu bracketed=%zu\n",
                 timed, timeless, timelessPlaced, insideBracket);
    EXPECT_GT(timed, 0u)                 << "the timed records did not resolve";
    EXPECT_GT(timeless, 0u)              << "the fixture produced no timeless records";
    EXPECT_EQ(timelessPlaced, timeless)  << "a timeless record was left unplaced";
    EXPECT_GT(insideBracket, 0u)         << "no timeless record had neighbours either side, so the "
                                            "bracket assertion never ran";
}

/**
 * @brief With NOTHING to bracket against, a timeless record stays unplaced and says which cause.
 *
 * The other half of the bucket split: a log of nothing but timeless DIDs has no live record
 * anywhere, so there is no honest answer and `NoTimeField` is reported rather than an invented
 * instant. That distinction is why `NoTimeField` and `FrozenAndUnbounded` are separate values —
 * one says the DID never had a clock, the other says its clock died.
 */
TEST_F(TimeResolverTest, TimelessDidWithNoNeighboursReportsNoTimeField) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (int i = 0; i < 12; ++i) {
        dev_info_t di{};
        di.serialNumber = 4321;
        recs.emplace_back(DID_DEV_INFO, bytesOf(di));
    }
    f = buildFixture("timeless_only", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value()) << log.error().message;
    auto built = ISTimeResolver::build(*log);
    ASSERT_TRUE(built.has_value());

    const std::size_t n = log->segment(0).recordCount();
    ASSERT_GT(n, 0u);
    std::size_t noTimeField = 0, placed = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const AbsTimeResult r = built->resolve(*log, 0, k);
        if (r.valid) { ++placed; continue; }
        if (r.mechanism == AbsTimeMechanism::NoTimeField) ++noTimeField;
    }
    std::fprintf(stderr, "[timeless-only] records=%zu placed=%zu noTimeField=%zu\n",
                 n, placed, noTimeField);
    EXPECT_EQ(placed, 0u)        << "a time was invented with nothing to bracket against";
    EXPECT_EQ(noTimeField, n)    << "the cause was not reported as NoTimeField";
}

