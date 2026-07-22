/**
 * @file test_log_bounds.cpp
 * @brief SN-8328 — log time bounds / span tests (whole-log AND per-DID).
 *
 * First cut: the J-0 framework + invariants (per-DID span within whole-log
 * span; whole-log span is the envelope of per-DID spans; all sync DIDs resolve
 * to the same GPS week — the D0066 cross-DID invariant that fails on the
 * 1980 / 46-year / PIMU-vs-GNSS bugs), plus J-1 whole-log and J-2 per-DID
 * clean-data bounds.
 *
 * Uses a controlled synthetic fixture (cISLogger) so timestamps/weeks are
 * exact. Bounds here are the RESOLVED bounds (what a consumer sees), computed
 * via ISTimeResolver over the records — that is the surface the anomalies show
 * up on.
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // must precede ISComm-pulling headers

#include "DeviceLog.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "ISTimeResolver.h"
#include "data_sets.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <utility>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kHwId       = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kSerial     = 888111u;
constexpr uint64_t kGpsEpochMs = 315'964'800'000ULL;  //!< 1980-01-06 UTC in Unix ms
constexpr uint64_t kWeekMs     = 604'800'000ULL;

struct FixturePaths { fs::path directory; fs::path rawFile; };

void writeRecord(cISLogger& logger, std::shared_ptr<cDeviceLog> dev,
                 uint32_t did, void* payload, std::size_t size) {
    is_comm_instance_t comm{};
    uint8_t buf[1024];
    is_comm_init(&comm, buf, sizeof(buf), nullptr);
    uint8_t pkt[2048];
    const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm,
                                      static_cast<uint16_t>(did),
                                      static_cast<uint16_t>(size), 0, payload);
    if (n > 0) logger.LogData(dev, n, pkt);
}

// ins_1_t and ins_2_t both lead with { uint32_t week; double timeOfWeek; } —
// both are ToW-bearing sync DIDs, so this gives two independent sync DIDs.
ins_2_t makeIns2(double towSec, uint32_t week = 2300) {
    ins_2_t s{}; s.week = week; s.timeOfWeek = towSec;
    s.qn2b[0] = 1.0f; s.lla[0] = 40.0; s.lla[1] = -111.0; s.lla[2] = 1400.0;
    return s;
}
ins_1_t makeIns1(double towSec, uint32_t week = 2300) {
    ins_1_t s{}; s.week = week; s.timeOfWeek = towSec;
    s.theta[0] = 0.1f; s.lla[0] = 40.0; s.lla[1] = -111.0; s.lla[2] = 1400.0;
    return s;
}
template <class T> std::vector<uint8_t> bytesOf(const T& t) {
    std::vector<uint8_t> out(sizeof(T));
    std::memcpy(out.data(), &t, sizeof(T));
    return out;
}

FixturePaths buildFixture(const std::string& hint,
                          const std::vector<std::pair<uint32_t, std::vector<uint8_t>>>& records) {
    FixturePaths f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf), "/tmp/test_log_bounds_%s_%d_%ld",
                  hint.c_str(), ::getpid(), static_cast<long>(::time(nullptr)));
    f.directory = dirBuf;
    ISFileManager::DeleteDirectory(f.directory.string());

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(f.directory.string(), opts)) return f;
    auto dev = logger.registerDevice(kHwId, kSerial);
    if (!dev) return f;
    logger.EnableLogging(true);
    for (const auto& [did, payload] : records) {
        std::vector<uint8_t> mutablePayload = payload;
        writeRecord(logger, dev, did, mutablePayload.data(), mutablePayload.size());
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> rawFiles;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, "\\.raw$", rawFiles);
    if (!rawFiles.empty()) f.rawFile = rawFiles.front().name;

    // Remove any .idx the writer emitted so fromSegments rebuilds a fresh v2
    // index from the .raw scan (populates the record index used by allRecords).
    std::vector<ISFileManager::file_info_t> idxFiles;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, "\\.idx$", idxFiles);
    for (const auto& fi : idxFiles) { std::error_code ec; fs::remove(fi.name, ec); }
    return f;
}

void teardown(FixturePaths& f) {
    if (!f.directory.empty() && fs::exists(f.directory))
        ISFileManager::DeleteDirectory(f.directory.string());
}

//! Resolved [min,max] over every record of the log (all DIDs), skipping
//! records the resolver can't place (SessionOnly/Unknown). {0,0} if none.
std::pair<uint64_t, uint64_t> resolvedSpan(const ISDeviceLog& log, const ISTimeResolver& r) {
    uint64_t lo = UINT64_MAX, hi = 0; bool any = false;
    const uint64_t dev = log.deviceId();
    for (auto rv : log.allRecords()) {
        const TimeStamp t = r.resolve(rv.timestamp().value, dev);
        if (t.source == TimeSource::SessionOnly && t.confidence == TimeConfidence::Unknown) continue;
        lo = std::min(lo, t.value); hi = std::max(hi, t.value); any = true;
    }
    return any ? std::make_pair(lo, hi) : std::pair<uint64_t, uint64_t>{0, 0};
}

//! Resolved [min,max] over one DID's records. {0,0} if none placeable.
std::pair<uint64_t, uint64_t> resolvedDidSpan(const ISDeviceLog& log, const ISTimeResolver& r,
                                              uint32_t did) {
    uint64_t lo = UINT64_MAX, hi = 0; bool any = false;
    const uint64_t dev = log.deviceId();
    for (auto rv : log.records(did)) {
        const TimeStamp t = r.resolve(rv.timestamp().value, dev);
        if (t.source == TimeSource::SessionOnly && t.confidence == TimeConfidence::Unknown) continue;
        lo = std::min(lo, t.value); hi = std::max(hi, t.value); any = true;
    }
    return any ? std::make_pair(lo, hi) : std::pair<uint64_t, uint64_t>{0, 0};
}

uint64_t gpsWeekOf(uint64_t unixMs) {
    return (unixMs >= kGpsEpochMs) ? (unixMs - kGpsEpochMs) / kWeekMs : 0;
}

class LogBoundsTest : public ::testing::Test {
protected:
    FixturePaths f;
    void TearDown() override { teardown(f); }
};

// -------------------- J-1: whole-log bounds, clean --------------------

TEST_F(LogBoundsTest, WholeLogSpanIsFirstToLastResolved) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : { 100.0, 110.0, 120.0 })
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    f = buildFixture("wholelog", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto r = ISTimeResolver::build(log.value());
    ASSERT_TRUE(r.has_value());

    auto [lo, hi] = resolvedSpan(log.value(), *r);
    EXPECT_LT(lo, hi);
    EXPECT_EQ(gpsWeekOf(lo), 2300u);
    EXPECT_EQ(gpsWeekOf(hi), 2300u);
    // Span width ~ 20 s (100..120 s of ToW).
    EXPECT_EQ(hi - lo, 20'000u);
}

TEST_F(LogBoundsTest, NoGpsRecordsSpanIsEmptyNotCrash) {
    // A log with no ToW-bearing records → resolver has no anchors → every
    // record resolves SessionOnly/Unknown → resolved span is empty, no crash.
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    imu_t imu{}; imu.time = 1.0; imu.I.acc[2] = 9.8f;
    recs.emplace_back(DID_IMU, bytesOf(imu));
    f = buildFixture("nogps", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto r = ISTimeResolver::build(log.value());
    ASSERT_TRUE(r.has_value());
    auto [lo, hi] = resolvedSpan(log.value(), *r);
    EXPECT_EQ(lo, 0u);
    EXPECT_EQ(hi, 0u);
}

// -------------------- J-2: per-DID bounds, clean --------------------

TEST_F(LogBoundsTest, PerDidSpanMatchesThatDidsRecords) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    for (double tow : { 100.0, 200.0, 300.0 })
        recs.emplace_back(DID_INS_2, bytesOf(makeIns2(tow)));
    f = buildFixture("perdid", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto r = ISTimeResolver::build(log.value());
    ASSERT_TRUE(r.has_value());

    auto [lo, hi] = resolvedDidSpan(log.value(), *r, DID_INS_2);
    EXPECT_EQ(gpsWeekOf(lo), 2300u);
    EXPECT_EQ(hi - lo, 200'000u);  // 100..300 s
}

// -------------------- J-0: cross-DID framework invariants --------------------

TEST_F(LogBoundsTest, PerDidSpansWithinWholeLogSpan_AndEnvelope) {
    // Two sync DIDs at interleaved ToWs, same week.
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_INS_1, bytesOf(makeIns1(100.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(150.0)));
    recs.emplace_back(DID_INS_1, bytesOf(makeIns1(200.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(250.0)));
    f = buildFixture("envelope", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto r = ISTimeResolver::build(log.value());
    ASSERT_TRUE(r.has_value());

    auto whole = resolvedSpan(log.value(), *r);
    auto d1    = resolvedDidSpan(log.value(), *r, DID_INS_1);
    auto d2    = resolvedDidSpan(log.value(), *r, DID_INS_2);

    // Each per-DID span sits inside the whole-log span.
    EXPECT_GE(d1.first,  whole.first);  EXPECT_LE(d1.second, whole.second);
    EXPECT_GE(d2.first,  whole.first);  EXPECT_LE(d2.second, whole.second);
    // Whole-log span is exactly the envelope of the per-DID spans.
    EXPECT_EQ(whole.first,  std::min(d1.first,  d2.first));
    EXPECT_EQ(whole.second, std::max(d1.second, d2.second));
}

TEST_F(LogBoundsTest, AllSyncDidsResolveToSameGpsWeek) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_INS_1, bytesOf(makeIns1(100.0)));
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(150.0)));
    f = buildFixture("sameweek", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto r = ISTimeResolver::build(log.value());
    ASSERT_TRUE(r.has_value());

    auto d1 = resolvedDidSpan(log.value(), *r, DID_INS_1);
    auto d2 = resolvedDidSpan(log.value(), *r, DID_INS_2);
    EXPECT_EQ(gpsWeekOf(d1.first), gpsWeekOf(d2.first));  // D0066 domain-class equality
    EXPECT_EQ(gpsWeekOf(d1.first), 2300u);
}

// The powerful cross-DID regression: one DID carries an early PRE-FIX (week 0)
// record while the durable fix is week 2300. Before SN-8323 this DID would
// resolve to ~1980 while the other stayed 2026, violating the invariant. With
// the fix, both anchor to the durable week → the invariant holds.
TEST_F(LogBoundsTest, PreFixRecordDoesNotSplitCrossDidDomain) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(10.0, /*week*/ 0)));   // pre-fix, week 0
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(150.0, 2300)));        // durable fix
    recs.emplace_back(DID_INS_2, bytesOf(makeIns2(160.0, 2300)));
    recs.emplace_back(DID_INS_1, bytesOf(makeIns1(155.0, 2300)));
    f = buildFixture("prefix_crossdid", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto log = ISDeviceLog::fromSegments({ f.rawFile });
    ASSERT_TRUE(log.has_value());
    auto r = ISTimeResolver::build(log.value());
    ASSERT_TRUE(r.has_value());

    auto d1 = resolvedDidSpan(log.value(), *r, DID_INS_1);
    auto d2 = resolvedDidSpan(log.value(), *r, DID_INS_2);
    // Both DIDs land in the durable week — no 1980/2026 split.
    EXPECT_EQ(gpsWeekOf(d1.first), 2300u);
    EXPECT_EQ(gpsWeekOf(d2.first), 2300u);
    EXPECT_EQ(gpsWeekOf(d2.second), 2300u);
}

}  // namespace
