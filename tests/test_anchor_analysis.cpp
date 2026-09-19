/**
 * @file test_anchor_analysis.cpp
 * @brief SN-8629 tests for the per-segment time-anchor cascade.
 *
 * The bulk of these are pure `AnchorCollector` tests: records are fed in directly and the
 * resolved analysis asserted, with no file, no `.idx` and no I/O of any kind. That is deliberate
 * — the requirement driving this work was that the anchor analysis be testable and queryable
 * WITHOUT rebuilding an index, so the cascade is exercised at the level where that is true.
 *
 * The fixture-backed tests at the end cover the three properties that only show up end-to-end:
 * that `analyzeSegment()` writes nothing, that the sidecar route and the byte-scan route agree,
 * and that a composed device log orders its segments by anchored start.
 */

#include <gtest/gtest.h>

// com_manager.h FIRST — the legacy ISFirmwareUpdater.h wraps it in `extern "C" { ... }`, which is
// illegal for the C++-overload-bearing com_manager.h. Same trick as test_log_reader.cpp.
#include "com_manager.h"

#include "ISAnchorAnalysis.h"
#include "ISAnchorCollector.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogReader.h"
#include "ISLogger.h"
#include "data_sets.h"
#include "test_data_utils.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <list>
#include <string>
#include <vector>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

// A ToW comfortably inside a GPS week, and an uptime comfortably below the domain split.
constexpr uint64_t kTowMs      = 342'227'671ULL;
constexpr uint64_t kUptimeMs   = 13'531'649ULL;
constexpr int64_t  kExpectedOff = static_cast<int64_t>(kTowMs) - static_cast<int64_t>(kUptimeMs);

//! Build a DID_SYS_PARAMS payload with the fields the cascade gates on.
sys_params_t makeSysParams(uint32_t towMs, double upSec, bool towValid) {
    sys_params_t sp{};
    sp.timeOfWeekMs = towMs;
    sp.upTime       = upSec;
    sp.hdwStatus    = towValid ? HDW_STATUS_GNSS_TIME_OF_WEEK_VALID : 0u;
    return sp;
}

//! Build a DID_GPX_STATUS payload — the GPX-side equivalent bridge record.
gpx_status_t makeGpxStatus(uint32_t towMs, double upSec, bool towValid) {
    gpx_status_t gs{};
    gs.timeOfWeekMs = towMs;
    gs.upTime       = upSec;
    gs.hdwStatus    = towValid ? GPX_HDW_STATUS_GNSS1_TIME_OF_WEEK_VALID : 0u;
    return gs;
}

//! Offer a whole record whose payload is @p p.
template <class T>
void feed(AnchorCollector& c, uint32_t did, const T& p, uint64_t recordTsMs) {
    c.consume(did, /*structOffset=*/0, reinterpret_cast<const uint8_t*>(&p),
              static_cast<uint32_t>(sizeof(T)), recordTsMs);
}

//! Offer a record the caller has no payload for — the index-driven path's shape.
void feedTimeOnly(AnchorCollector& c, uint32_t did, uint64_t recordTsMs) {
    c.consume(did, /*structOffset=*/0, nullptr, 0, recordTsMs);
}

bool hasAnomalyContaining(const AnchorAnalysis& a, const std::string& needle) {
    for (const auto& s : a.anomalies) {
        if (s.find(needle) != std::string::npos) return true;
    }
    return false;
}

} // namespace

// =====================================================================================
// Tier 5 — dual-domain bridge records
// =====================================================================================

TEST(AnchorCascade, SysParamsBridgeResolvesTopTier) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWBridge);
    EXPECT_EQ(a.anchorDid, DID_SYS_PARAMS);
    EXPECT_EQ(a.anchorTowMs, kTowMs);
    EXPECT_EQ(a.anchorUptimeMs, 379'369ULL);
    EXPECT_EQ(a.offsetMs, static_cast<int64_t>(kTowMs) - 379'369LL);
    EXPECT_TRUE(a.anchored());
}

TEST(AnchorCascade, GpxStatusBridgeIsEquallyTrusted) {
    // A GPX-sourced log may never contain a DID_SYS_PARAMS at all, so DID_GPX_STATUS has to
    // reach the same tier on its own.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_GPX_STATUS, makeGpxStatus(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWBridge);
    EXPECT_EQ(a.anchorDid, DID_GPX_STATUS);
    EXPECT_EQ(a.anchorTowMs, kTowMs);
}

TEST(AnchorCascade, BridgeRejectedWhenTowValidFlagIsClear) {
    // The presence of a bridge record is NOT enough — its time-of-week must be flagged valid.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, /*towValid=*/false),
         kTowMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::None);
    EXPECT_FALSE(a.anchored());
}

TEST(AnchorCascade, BridgeRejectedWhenTowIsZero) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(0u, 379.369, /*towValid=*/true), kUptimeMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::None);
}

TEST(AnchorCascade, BridgeRejectedWhenTowExceedsAGpsWeek) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(700'000'000u, 379.369, /*towValid=*/true), kUptimeMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::None);
}

TEST(AnchorCascade, EarliestBridgeWinsAndDisagreementIsReported) {
    // Both bridges are equally trustworthy, so the earliest one in the segment wins and the two
    // disagreeing is itself the finding.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);
    // GPX bridge 10 s out of step with the IMX bridge.
    feed(c, DID_GPX_STATUS, makeGpxStatus(static_cast<uint32_t>(kTowMs + 10'000), 379.369, true),
         kTowMs + 10'000);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.anchorDid, DID_SYS_PARAMS) << "earliest bridge must win";
    EXPECT_EQ(a.anchorTowMs, kTowMs);
    EXPECT_TRUE(hasAnomalyContaining(a, "bridge clocks disagree"));
}

TEST(AnchorCascade, AgreeingBridgesRaiseNoAnomaly) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);
    feed(c, DID_GPX_STATUS, makeGpxStatus(static_cast<uint32_t>(kTowMs + 100), 379.469, true),
         kTowMs + 100);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_FALSE(hasAnomalyContaining(a, "bridge clocks disagree"));
}

// =====================================================================================
// Tier 4 — ToW-only records
// =====================================================================================

TEST(AnchorCascade, TowOnlyRecordCorrelatesAgainstPrecedingUptime) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);          // the correlation partner
    feedTimeOnly(c, DID_GNSS1_POS, kTowMs);        // ToW-domain record timestamp

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWSingle);
    EXPECT_EQ(a.anchorDid, DID_GNSS1_POS);
    EXPECT_EQ(a.anchorTowMs, kTowMs);
    EXPECT_EQ(a.anchorUptimeMs, kUptimeMs);
    EXPECT_EQ(a.offsetMs, kExpectedOff);
}

TEST(AnchorCascade, TowOnlyAnchorNeedsNoPayload) {
    // Regression: tier 4 is recognized from the record's index timestamp alone. It used to be
    // evaluated only AFTER a null-payload early-out, which made it unreachable from the
    // index-driven path — that path supplies payloads only for the bridge DIDs. A log with GNSS
    // records but no bridge record would then resolve a whole tier lower from a trusted sidecar
    // than from a byte scan of the same bytes.
    for (uint32_t did : { DID_INS_1, DID_INS_2, DID_GNSS1_POS, DID_GNSS2_POS,
                          DID_GNSS1_VEL, DID_GNSS2_VEL }) {
        AnchorCollector c;
        feedTimeOnly(c, DID_PIMU, kUptimeMs);
        feedTimeOnly(c, did, kTowMs);

        const AnchorAnalysis a = c.finish(nullptr);
        EXPECT_EQ(a.tier, AnchorTier::PayloadToWSingle) << "DID " << did;
        EXPECT_EQ(a.anchorDid, did);
    }
}

TEST(AnchorCascade, BridgeOutranksTowOnly) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feedTimeOnly(c, DID_GNSS1_POS, kTowMs);   // tier 4 candidate, seen first
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWBridge) << "tier is resolved strongest-first, not "
                                                        "first-seen-wins";
    EXPECT_EQ(a.anchorDid, DID_SYS_PARAMS);
}

// =====================================================================================
// Tiers 3..1 — the chained and filename fallbacks
// =====================================================================================

TEST(AnchorCascade, PreviousOffsetCarriesForwardWhenUptimeIsContinuous) {
    AnchorAnalysis prev{};
    prev.tier         = AnchorTier::PayloadToWBridge;
    prev.offsetMs     = kExpectedOff;
    prev.uptimeMinMs  = kUptimeMs;
    prev.uptimeMaxMs  = kUptimeMs + 100'000;
    prev.anchoredEndMs = static_cast<uint64_t>(static_cast<int64_t>(prev.uptimeMaxMs) + kExpectedOff);

    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, prev.uptimeMaxMs + 20);     // picks up where prev left off
    feedTimeOnly(c, DID_PIMU, prev.uptimeMaxMs + 50'000);

    const AnchorAnalysis a = c.finish(&prev);
    EXPECT_EQ(a.tier, AnchorTier::BridgedToW);
    EXPECT_EQ(a.offsetMs, kExpectedOff);
    EXPECT_EQ(a.anchoredStartMs,
              static_cast<uint64_t>(static_cast<int64_t>(prev.uptimeMaxMs + 20) + kExpectedOff));
    EXPECT_TRUE(hasAnomalyContaining(a, "carried the previous"));
}

TEST(AnchorCascade, ChainsFromPreviousEndWhenUptimeIsNotContinuous) {
    AnchorAnalysis prev{};
    prev.tier          = AnchorTier::PayloadToWBridge;
    prev.offsetMs      = kExpectedOff;
    prev.uptimeMinMs   = kUptimeMs;
    prev.uptimeMaxMs   = kUptimeMs + 100'000;
    prev.anchoredEndMs = 999'000'000ULL;

    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, 500);       // device rebooted — uptime restarted near zero
    feedTimeOnly(c, DID_PIMU, 60'500);

    const AnchorAnalysis a = c.finish(&prev);
    EXPECT_EQ(a.tier, AnchorTier::PrevSegmentChained);
    EXPECT_EQ(a.anchoredStartMs, prev.anchoredEndMs);
    EXPECT_TRUE(hasAnomalyContaining(a, "chained from the previous"));
}

TEST(AnchorCascade, FilenameAnchorIsTheLastResortBeforeNone) {
    AnchorCollector c;
    c.setFilenameAnchorMs(1'779'363'435'000ULL);
    feedTimeOnly(c, DID_PIMU, 1000);
    feedTimeOnly(c, DID_PIMU, 61'000);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::FilenameAnchor);
    EXPECT_EQ(a.anchoredStartMs, 1'779'363'435'000ULL);
    EXPECT_EQ(a.anchoredEndMs, 1'779'363'435'000ULL + 60'000);
    EXPECT_TRUE(hasAnomalyContaining(a, "segment filename"));
}

TEST(AnchorCascade, NoAnchorAtAllIsNoneAndSaysSo) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, 1000);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::None);
    EXPECT_FALSE(a.anchored());
    EXPECT_EQ(a.anchoredStartMs, 0u);
    EXPECT_TRUE(hasAnomalyContaining(a, "no anchor of any kind"));
}

TEST(AnchorCascade, TierLadderIsOrderedByDurability) {
    // The ordering is load-bearing: fromSegments refuses to sort below minimumOrderableTier, and
    // the regression check is a `<` comparison between adjacent segments.
    EXPECT_LT(AnchorTier::None,               AnchorTier::FilenameAnchor);
    EXPECT_LT(AnchorTier::FilenameAnchor,     AnchorTier::PrevSegmentChained);
    EXPECT_LT(AnchorTier::PrevSegmentChained, AnchorTier::BridgedToW);
    EXPECT_LT(AnchorTier::BridgedToW,         AnchorTier::PayloadToWSingle);
    EXPECT_LT(AnchorTier::PayloadToWSingle,   AnchorTier::PayloadToWBridge);
    EXPECT_GE(AnchorAnalysis::minimumOrderableTier, AnchorTier::FilenameAnchor);
}

TEST(AnchorCascade, DurabilityRegressionIsReported) {
    AnchorAnalysis prev{};
    prev.tier     = AnchorTier::PayloadToWBridge;
    prev.offsetMs = kExpectedOff;

    AnchorCollector c;
    c.setFilenameAnchorMs(1'779'363'435'000ULL);
    feedTimeOnly(c, DID_PIMU, 1000);

    const AnchorAnalysis a = c.finish(&prev);
    EXPECT_LT(a.tier, prev.tier);
    EXPECT_TRUE(hasAnomalyContaining(a, "durability dropped"));
}

// =====================================================================================
// Record accounting, extrema, and anomalies
// =====================================================================================

TEST(AnchorCascade, RecordsAreBucketedByTimeDomain) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, 1000);
    feedTimeOnly(c, DID_PIMU, 2000);
    feedTimeOnly(c, DID_GNSS1_POS, kTowMs);
    feedTimeOnly(c, DID_DEBUG_ARRAY, 0);     // carries no internal time

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.uptimeRecords, 2u);
    EXPECT_EQ(a.towRecords, 1u);
    EXPECT_EQ(a.untimedRecords, 1u);
    EXPECT_EQ(a.uptimeMinMs, 1000u);
    EXPECT_EQ(a.uptimeMaxMs, 2000u);
    EXPECT_EQ(a.towMinMs, kTowMs);
    EXPECT_EQ(a.towMaxMs, kTowMs);
    EXPECT_EQ(c.recordsSeen(), 4u);
}

TEST(AnchorCascade, AnchoredSpanProjectsTheUptimeExtrema) {
    // The header must come from the per-domain extrema, NOT from whichever record happened to be
    // physically first and last — that positional key is the defect this whole cascade replaces.
    AnchorCollector c;
    feedTimeOnly(c, DID_GNSS1_POS, kTowMs + 5000);   // a ToW-domain record lands FIRST
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feedTimeOnly(c, DID_PIMU, kUptimeMs + 30'000);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 13'531.649, true), kTowMs);

    const AnchorAnalysis a = c.finish(nullptr);
    ASSERT_EQ(a.tier, AnchorTier::PayloadToWBridge);
    EXPECT_EQ(a.anchoredStartMs, static_cast<uint64_t>(static_cast<int64_t>(kUptimeMs) + a.offsetMs));
    EXPECT_EQ(a.anchoredEndMs,
              static_cast<uint64_t>(static_cast<int64_t>(kUptimeMs + 30'000) + a.offsetMs));
}

TEST(AnchorCascade, StalledDidClockIsReported) {
    // The customer-log signature: a device dies but keeps emitting records, all stamped with the
    // instant its clock stopped. Reported rather than drawn through as real time.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    for (int i = 0; i < 200; ++i) {
        feedTimeOnly(c, DID_GPX_STATUS, 342'615'500ULL);
    }

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_TRUE(hasAnomalyContaining(a, "clock stalled"));
    EXPECT_TRUE(hasAnomalyContaining(a, std::to_string(DID_GPX_STATUS)));
}

TEST(AnchorCascade, ShortRunsOfRepeatedTimestampsAreNotReported) {
    // A DID output faster than its time field's resolution legitimately repeats a few times.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    for (int i = 0; i < 4; ++i) {
        feedTimeOnly(c, DID_GPX_STATUS, 342'615'500ULL);
    }

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_FALSE(hasAnomalyContaining(a, "clock stalled"));
}

TEST(AnchorCascade, PartialRecordsCountTowardExtremaButCannotAnchor) {
    // A partial record (structOffset != 0) is one chunk of a larger struct: its bytes do not hold
    // the anchor fields where they are expected, and its siblings share a timestamp.
    AnchorCollector c;
    const sys_params_t sp = makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true);
    c.consume(DID_SYS_PARAMS, /*structOffset=*/16, reinterpret_cast<const uint8_t*>(&sp),
              sizeof(sp), kTowMs);
    feedTimeOnly(c, DID_PIMU, kUptimeMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::None) << "a partial record must not anchor";
    EXPECT_EQ(a.towRecords, 1u) << "but it is still a record in the ToW domain";
    EXPECT_EQ(a.uptimeRecords, 1u);
}

TEST(AnchorCascade, NeedsPayloadNamesOnlyTheBridgeDids) {
    EXPECT_TRUE(AnchorCollector::needsPayload(DID_SYS_PARAMS));
    EXPECT_TRUE(AnchorCollector::needsPayload(DID_GPX_STATUS));
    EXPECT_FALSE(AnchorCollector::needsPayload(DID_GNSS1_POS));
    EXPECT_FALSE(AnchorCollector::needsPayload(DID_INS_1));
    EXPECT_FALSE(AnchorCollector::needsPayload(DID_PIMU));
}

TEST(AnchorCascade, TierNamesAreAllDistinctAndNonEmpty) {
    const AnchorTier all[] = { AnchorTier::None, AnchorTier::FilenameAnchor,
                               AnchorTier::PrevSegmentChained, AnchorTier::BridgedToW,
                               AnchorTier::PayloadToWSingle, AnchorTier::PayloadToWBridge };
    std::vector<std::string> names;
    for (AnchorTier t : all) {
        const std::string n = anchorTierName(t);
        EXPECT_FALSE(n.empty());
        EXPECT_NE(n, "?");
        names.push_back(n);
    }
    std::sort(names.begin(), names.end());
    EXPECT_EQ(std::adjacent_find(names.begin(), names.end()), names.end());
}

// =====================================================================================
// Filename anchor parsing
// =====================================================================================

TEST(FilenameAnchor, ParsesTheWriterPattern) {
    const uint64_t ms = ISLogReader::filenameAnchorMs("LOG_SN60050_20260521_113715_0001.raw");
    EXPECT_EQ(ms, 1'779'363'435'000ULL);
}

TEST(FilenameAnchor, SurvivesALongSerialNumber) {
    // Regression: a 9-digit serial creates an earlier 8-digit window that straddles the serial
    // and the date — "42742854_202605" in LOG_SN942742854_20260521_113715_0001 reads as year
    // 4274, month 28. The scan used to give up at that window instead of continuing, so every
    // IMX-6 log silently lost its filename anchor while 5-digit IMX-5 serials parsed fine.
    const uint64_t nineDigit = ISLogReader::filenameAnchorMs(
        "LOG_SN942742854_20260521_113715_0001.raw");
    const uint64_t fiveDigit = ISLogReader::filenameAnchorMs(
        "LOG_SN60050_20260521_113715_0001.raw");
    EXPECT_NE(nineDigit, 0u) << "long serials must not defeat the parse";
    EXPECT_EQ(nineDigit, fiveDigit) << "same date, same answer, regardless of serial length";
}

TEST(FilenameAnchor, ReturnsZeroWhenThereIsNoDate) {
    EXPECT_EQ(ISLogReader::filenameAnchorMs("capture.raw"), 0u);
    EXPECT_EQ(ISLogReader::filenameAnchorMs("LOG_SN60050_0001.raw"), 0u);
}

TEST(FilenameAnchor, RejectsImplausibleDates) {
    EXPECT_EQ(ISLogReader::filenameAnchorMs("LOG_19990101_000000_0001.raw"), 0u)
        << "before the year bound";
    EXPECT_EQ(ISLogReader::filenameAnchorMs("LOG_20261301_000000_0001.raw"), 0u) << "month 13";
    EXPECT_EQ(ISLogReader::filenameAnchorMs("LOG_20260532_000000_0001.raw"), 0u) << "day 32";
    EXPECT_EQ(ISLogReader::filenameAnchorMs("LOG_20260521_256100_0001.raw"), 0u) << "hour 25";
}

// =====================================================================================
// End-to-end properties that need a real segment
// =====================================================================================

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 519465u;

struct Fixture {
    fs::path                         directory;
    fs::path                         rawFile;
    fs::path                         idxFile;
    std::list<std::vector<uint8_t>*> messages;
};

/**
 * @brief Portable, process-unique temp directory.
 *
 * Deliberately avoids `<unistd.h>`, `getpid()` and a hardcoded `/tmp`, following the convention
 * noted in tests/CMakeLists.txt: a new test file is written portable and kept OFF the WIN32
 * exclusion list rather than excluded. That matters here — 27 of this file's 32 tests are pure
 * cascade tests that touch no filesystem at all, and excluding the file to satisfy the five that
 * do would drop the whole anchor cascade from Windows CI coverage.
 *
 * The per-process token is what the plain counter used by the sibling helpers lacks: two test
 * binaries running concurrently on one runner would otherwise resolve the same directory.
 */
fs::path makeTempDir(const std::string& prefix) {
    static const std::string token = std::to_string(
        std::chrono::system_clock::now().time_since_epoch().count());
    static unsigned counter = 0;
    return fs::temp_directory_path() /
           ("test_anchor_" + prefix + "_" + token + "_" + std::to_string(counter++));
}

//! Generate a small single-segment log. Same approach as test_log_reader.cpp: commit the
//! generator, not a binary.
Fixture generateFixture(const std::string& hint) {
    Fixture f;
    f.directory = makeTempDir(hint);
    ISFileManager::DeleteDirectory(f.directory.string());

    GenerateRawLogData(f.messages, 1.0f);
    if (f.messages.empty()) return f;

    {
        cISLogger logger;
        cISLogger::sSaveOptions opts;
        opts.logType               = cISLogger::LOGTYPE_RAW;
        opts.useSubFolderTimestamp = false;
        if (!logger.InitSave(f.directory.string(), opts)) return f;
        auto devLogger = logger.registerDevice(kFixtureHwId, kFixtureSerial);
        if (!devLogger) return f;
        logger.EnableLogging(true);
        for (auto* msg : f.messages) {
            logger.LogData(devLogger, msg->size(), reinterpret_cast<const uint8_t*>(msg->data()));
        }
        logger.CloseAllFiles();
    }

    std::vector<ISFileManager::file_info_t> raws, idxs;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, "\\.raw$", raws);
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, "\\.idx$", idxs);
    if (!raws.empty()) f.rawFile = raws.front().name;
    if (!idxs.empty()) f.idxFile = idxs.front().name;
    return f;
}

void teardown(Fixture& f) {
    for (auto* m : f.messages) delete m;
    f.messages.clear();
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

class AnchorSegmentTest : public ::testing::Test {
protected:
    void SetUp() override {
        f_ = generateFixture(::testing::UnitTest::GetInstance()->current_test_info()->name());
        ASSERT_FALSE(f_.rawFile.empty()) << "fixture produced no .raw";
    }
    void TearDown() override { teardown(f_); }
    Fixture f_;
};

std::size_t countIdxFiles(const fs::path& dir) {
    std::size_t n = 0;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() == ".idx") ++n;
    }
    return n;
}

} // namespace

TEST_F(AnchorSegmentTest, AnalyzeSegmentCreatesNoIndexFile) {
    // The stand-alone analyze must be queryable without mutating the log directory. Routing it
    // through openSegment broke this: that path persists a rebuilt sidecar when one is missing,
    // so asking "when does this segment start?" wrote an .idx as a side effect.
    ASSERT_TRUE(fs::exists(f_.idxFile));
    fs::remove(f_.idxFile);
    ASSERT_EQ(countIdxFiles(f_.directory), 0u);

    auto a = ISLogReader::analyzeSegment(f_.rawFile, nullptr);
    ASSERT_TRUE(a.has_value()) << "analyzeSegment failed";
    EXPECT_EQ(countIdxFiles(f_.directory), 0u) << "analyzeSegment must not write a sidecar";
}

TEST_F(AnchorSegmentTest, AnalyzeSegmentIsRepeatableAndDoesNotNeedAnIndex) {
    fs::remove(f_.idxFile);
    auto first  = ISLogReader::analyzeSegment(f_.rawFile, nullptr);
    auto second = ISLogReader::analyzeSegment(f_.rawFile, nullptr);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(first->tier, second->tier);
    EXPECT_EQ(first->anchoredStartMs, second->anchoredStartMs);
    EXPECT_EQ(first->anchoredEndMs, second->anchoredEndMs);
    EXPECT_EQ(first->offsetMs, second->offsetMs);
}

TEST_F(AnchorSegmentTest, SidecarRouteAgreesWithScanRoute) {
    // Opening with a trusted sidecar derives the anchor from the record index; opening without
    // one derives it from the byte scan. Both run the same cascade and must land in the same
    // place, or the ordering key would depend on whether an .idx happened to be present.
    ASSERT_TRUE(fs::exists(f_.idxFile));
    auto withIdx = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(withIdx.has_value());
    const AnchorAnalysis viaSidecar = withIdx->anchorAnalysis();
    ASSERT_TRUE(withIdx->hadOnDiskIndex()) << "fixture was expected to have a trusted sidecar";

    auto viaScan = ISLogReader::analyzeSegment(f_.rawFile, nullptr);
    ASSERT_TRUE(viaScan.has_value());

    EXPECT_EQ(viaSidecar.tier, viaScan->tier);
    EXPECT_EQ(viaSidecar.anchorDid, viaScan->anchorDid);
    EXPECT_EQ(viaSidecar.uptimeMinMs, viaScan->uptimeMinMs);
    EXPECT_EQ(viaSidecar.uptimeMaxMs, viaScan->uptimeMaxMs);
    EXPECT_EQ(viaSidecar.anchoredStartMs, viaScan->anchoredStartMs);
    EXPECT_EQ(viaSidecar.anchoredEndMs, viaScan->anchoredEndMs);
}

TEST_F(AnchorSegmentTest, OpenedSegmentAlwaysCarriesAnAnalysis) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    // Whatever route produced the index, the analysis is populated -- fromSegments depends on
    // every segment carrying one before it will order by time.
    EXPECT_NE(r->anchorAnalysis().tier, AnchorTier::None);
    EXPECT_NE(r->anchorAnalysis().anchoredStartMs, 0u);
}

TEST_F(AnchorSegmentTest, ComposedDeviceLogOrdersByAnchoredStart) {
    auto dl = ISDeviceLog::fromSegments({ f_.rawFile });
    ASSERT_TRUE(dl.has_value()) << "fromSegments failed";
    ASSERT_EQ(dl->segmentCount(), 1u);

    const AnchorAnalysis& a = dl->segment(0).anchorAnalysis();
    EXPECT_TRUE(a.anchored());
    EXPECT_GE(a.tier, AnchorAnalysis::minimumOrderableTier);
    EXPECT_LE(a.anchoredStartMs, a.anchoredEndMs);
}
