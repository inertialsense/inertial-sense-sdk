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
#include "ISDataMappings.h"
#include "ISAnchorCollector.h"
#include "ISDeviceLog.h"
#include "ISLogIndex.h"
#include "ISFileManager.h"
#include "ISLogReader.h"
#include "ISTimeResolver.h"
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

//! Build a DID_INS_2 payload. `week`/`hdwStatus` are what the tier-4 gate reads --
//! ins_2_t.hdwStatus is documented as a copy of DID_SYS_PARAMS.hdwStatus.
ins_2_t makeIns2(double towSec, bool towValid, uint32_t week = 2338) {
    ins_2_t v{};
    v.week       = week;
    v.timeOfWeek = towSec;
    v.hdwStatus  = towValid ? HDW_STATUS_GNSS_TIME_OF_WEEK_VALID : 0u;
    return v;
}

//! Build a DID_INS_1 payload. Same gate fields as ins_2_t, but a DIFFERENT struct size -- the
//! gate is per-DID and checks `payloadSize >= sizeof(ins_1_t)`, so feeding an ins_2_t here is
//! silently rejected. (That mistake cost a test failure; hence one builder per struct.)
ins_1_t makeIns1(double towSec, bool towValid, uint32_t week = 2338) {
    ins_1_t v{};
    v.week       = week;
    v.timeOfWeek = towSec;
    v.hdwStatus  = towValid ? HDW_STATUS_GNSS_TIME_OF_WEEK_VALID : 0u;
    return v;
}

//! Build a DID_GNSS1/2_VEL payload. gnss_vel_t carries NO `week`, so the fix type is the only
//! signal the gate can use -- and its `status` sits at a different offset than gnss_pos_t's.
gnss_vel_t makeGnssVel(uint32_t towMs, bool haveFix) {
    gnss_vel_t v{};
    v.timeOfWeekMs = towMs;
    v.status       = haveFix ? static_cast<uint32_t>(GNSS_STATUS_FIX_3D)
                             : static_cast<uint32_t>(GNSS_STATUS_FIX_NONE);
    return v;
}

//! Build a DID_GNSS1_POS payload. The tier-4 gate reads `week` and the fix type in `status`.
gnss_pos_t makeGnssPos(uint32_t towMs, bool haveFix, uint32_t week = 2338) {
    gnss_pos_t v{};
    v.week         = week;
    v.timeOfWeekMs = towMs;
    v.status       = haveFix ? static_cast<uint32_t>(GNSS_STATUS_FIX_3D)
                             : static_cast<uint32_t>(GNSS_STATUS_FIX_NONE);
    return v;
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

TEST(AnchorCascade, EqualAuthorityDisagreementIsUnresolvedAndRaisedLoudly) {
    // SUPERSEDES the old "earliest bridge wins" behaviour. Two bridges are equally trustworthy,
    // so when they disagree there is no basis to prefer either -- taking whichever happened to
    // appear first in the byte stream was an arbitrary tiebreak dressed up as determinism, and
    // it silently discarded the finding. The anchor is still populated (deterministically, from
    // the first of the tied candidates) so nothing blocks the user, but the state says plainly
    // that it was not adjudicated.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);
    // GPX bridge 10 s out of step with the IMX bridge.
    feed(c, DID_GPX_STATUS, makeGpxStatus(static_cast<uint32_t>(kTowMs + 10'000), 379.369, true),
         kTowMs + 10'000);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWBridge) << "authority is unchanged by the conflict";
    EXPECT_EQ(a.consensus, AnchorConsensus::DisagreedUnresolved);
    EXPECT_TRUE(a.anchored()) << "must not block the application";
    EXPECT_EQ(a.candidates.size(), 2u) << "both claims retained for the user to choose between";
    EXPECT_TRUE(hasAnomalyContaining(a, "EQUAL-AUTHORITY clock disagreement"));
    EXPECT_TRUE(hasAnomalyContaining(a, "NO CONSENSUS"));
}

TEST(AnchorCascade, HigherAuthorityWinsOutrightOverADisagreeingLesserSource) {
    // Kyle's option (b): always anchor to the most trustworthy source. A tier-5 bridge needs no
    // correlation step, so it beats a tier-4 ToW-only claim even when the two disagree -- but
    // the dissent is still reported, because for an analysis tool that IS the finding.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_GNSS1_POS, makeGnssPos(static_cast<uint32_t>(kTowMs + 30'000), true),
         kTowMs + 30'000);                              // tier 4, 30 s out
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWBridge);
    EXPECT_EQ(a.anchorDid, DID_SYS_PARAMS) << "highest authority wins outright";
    EXPECT_EQ(a.consensus, AnchorConsensus::DisagreedResolvedByAuthority);
    EXPECT_TRUE(hasAnomalyContaining(a, "lesser-authority clock disagrees"));
    EXPECT_FALSE(hasAnomalyContaining(a, "NO CONSENSUS")) << "this case IS adjudicated";
}

TEST(AnchorCascade, MajorityOfThreeEqualSourcesRejectsTheOutlier) {
    // Kyle's three-source rule: "if we have 3, and 2 of them generally align, we can reasonably
    // reject the 3rd (and call it out as suspect)."
    //
    // All three must be of EQUAL authority for this to be a majority question at all -- if they
    // differ in tier, the authority rule decides and no vote is needed (see
    // HigherAuthorityWinsOutrightOverADisagreeingLesserSource). Three distinct ToW-only DIDs
    // with no bridge record present gives three tier-4 peers. Distinct DIDs matter: repeated
    // claims from ONE DID at one offset are folded as corroboration, not counted as votes.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);              // the correlation partner for all three
    {   const ins_1_t p1 = makeIns1(static_cast<double>(kTowMs) / 1000.0, true);
        const ins_2_t p2 = makeIns2(static_cast<double>(kTowMs) / 1000.0, true);
        c.consume(DID_INS_1, 0, reinterpret_cast<const uint8_t*>(&p1), sizeof(p1), kTowMs);
        c.consume(DID_INS_2, 0, reinterpret_cast<const uint8_t*>(&p2), sizeof(p2), kTowMs); }
    feed(c, DID_GNSS1_POS, makeGnssPos(static_cast<uint32_t>(kTowMs + 45'000), true),
         kTowMs + 45'000);                              // the outlier, 45 s out

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWSingle) << "no bridge present; all peers are tier 4";
    EXPECT_EQ(a.candidates.size(), 3u);
    EXPECT_EQ(a.consensus, AnchorConsensus::DisagreedResolvedByAuthority)
        << "a real majority agreed, so this IS adjudicated -- not a deadlock";
    EXPECT_EQ(a.offsetMs, kExpectedOff) << "the majority's offset must win, not the outlier's";
    EXPECT_TRUE(hasAnomalyContaining(a, "rejected in favour of the more trustworthy source"));
    EXPECT_TRUE(hasAnomalyContaining(a, std::to_string(DID_GNSS1_POS)))
        << "the outlier must be named so a human can go look at it";
}

TEST(AnchorCascade, LesserSourceDissentIsReportedWithoutDowngradingConsensus) {
    // A lesser authority that disagrees earns a warning but must not weaken the verdict, nor
    // block anything -- two equal peers agreeing is still the strongest state available.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);
    feed(c, DID_GPX_STATUS, makeGpxStatus(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);
    feed(c, DID_GNSS1_POS, makeGnssPos(static_cast<uint32_t>(kTowMs + 45'000), true),
         kTowMs + 45'000);                              // tier 4 dissenter

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.consensus, AnchorConsensus::Corroborated)
        << "the two equal-authority bridges agree; a lesser dissent does not change that";
    EXPECT_TRUE(hasAnomalyContaining(a, "lesser-authority clock disagrees"))
        << "but we still say so -- for an analysis tool that IS the finding";
}

TEST(AnchorCascade, LoneCandidateIsUncorroborated) {
    // One claim, nothing contradicting it -- and nothing confirming it either. Distinct from
    // two sources that agree, which is the state the old single `tier` field could not express.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWBridge);
    EXPECT_EQ(a.consensus, AnchorConsensus::Uncorroborated);
    EXPECT_EQ(a.candidates.size(), 1u);
}

TEST(AnchorCascade, ConsensusNamesAreAllDistinct) {
    const AnchorConsensus all[] = {
        AnchorConsensus::NotApplicable, AnchorConsensus::Uncorroborated,
        AnchorConsensus::Corroborated, AnchorConsensus::DisagreedResolvedByAuthority,
        AnchorConsensus::DisagreedUnresolved };
    std::vector<std::string> names;
    for (AnchorConsensus c : all) {
        const std::string n = anchorConsensusName(c);
        EXPECT_FALSE(n.empty());
        EXPECT_NE(n, "?");
        names.push_back(n);
    }
    std::sort(names.begin(), names.end());
    EXPECT_EQ(std::adjacent_find(names.begin(), names.end()), names.end());
}

TEST(AnchorCascade, AgreeingBridgesRaiseNoAnomaly) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_SYS_PARAMS, makeSysParams(static_cast<uint32_t>(kTowMs), 379.369, true), kTowMs);
    feed(c, DID_GPX_STATUS, makeGpxStatus(static_cast<uint32_t>(kTowMs + 100), 379.469, true),
         kTowMs + 100);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_FALSE(hasAnomalyContaining(a, "disagree"));
    EXPECT_EQ(a.consensus, AnchorConsensus::Corroborated)
        << "two independent sources agreeing is the strongest state available";
}

// =====================================================================================
// Tier 4 — ToW-only records
// =====================================================================================

TEST(AnchorCascade, TowOnlyRecordCorrelatesAgainstPrecedingUptime) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);          // the correlation partner
    // The TIME comes from the record's index timestamp; the VALIDITY comes from the payload.
    feed(c, DID_GNSS1_POS, makeGnssPos(static_cast<uint32_t>(kTowMs), /*haveFix=*/true), kTowMs);

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.tier, AnchorTier::PayloadToWSingle);
    EXPECT_EQ(a.anchorDid, DID_GNSS1_POS);
    EXPECT_EQ(a.anchorTowMs, kTowMs);
    EXPECT_EQ(a.anchorUptimeMs, kUptimeMs);
    EXPECT_EQ(a.offsetMs, kExpectedOff);
}

TEST(AnchorCascade, TowOnlyTakesTimeFromTheIndexButValidityFromThePayload) {
    // SUPERSEDES TowOnlyAnchorNeedsNoPayload. The record's index timestamp IS the time-of-week,
    // so the time needs no payload -- but whether that ToW MEANS anything does. Every other tier
    // is validity-gated; tier 4 was not, and the only thing hiding it was an accident in the old
    // magnitude domain test.
    const double towSec = static_cast<double>(kTowMs) / 1000.0;
    const ins_1_t   i1 = makeIns1(towSec, true);
    const ins_2_t   i2 = makeIns2(towSec, true);
    const gnss_pos_t gp = makeGnssPos(static_cast<uint32_t>(kTowMs), true);
    const gnss_vel_t gv = makeGnssVel(static_cast<uint32_t>(kTowMs), true);

    struct Case { uint32_t did; const uint8_t* p; uint32_t n; };
    const Case cases[] = {
        { DID_INS_1,     reinterpret_cast<const uint8_t*>(&i1), sizeof(i1) },
        { DID_INS_2,     reinterpret_cast<const uint8_t*>(&i2), sizeof(i2) },
        { DID_GNSS1_POS, reinterpret_cast<const uint8_t*>(&gp), sizeof(gp) },
        { DID_GNSS2_POS, reinterpret_cast<const uint8_t*>(&gp), sizeof(gp) },
        { DID_GNSS1_VEL, reinterpret_cast<const uint8_t*>(&gv), sizeof(gv) },
        { DID_GNSS2_VEL, reinterpret_cast<const uint8_t*>(&gv), sizeof(gv) },
    };
    for (const auto& cs : cases) {
        AnchorCollector c;
        feedTimeOnly(c, DID_PIMU, kUptimeMs);
        c.consume(cs.did, 0, cs.p, cs.n, kTowMs);

        const AnchorAnalysis a = c.finish(nullptr);
        EXPECT_EQ(a.tier, AnchorTier::PayloadToWSingle) << "DID " << cs.did;
        EXPECT_EQ(a.anchorDid, cs.did);
        EXPECT_EQ(a.anchorTowMs, kTowMs) << "time still comes from the index timestamp";
    }
}

TEST(AnchorCascade, TowOnlyRejectedWhenThePayloadSaysTheToWIsNotValid) {
    // THE AHRS CASE. Four golden-corpus AHRS captures (22 segments) began anchoring at
    // PayloadToWSingle off DID_INS_2.timeOfWeek the moment the domain classifier was fixed --
    // a field with no meaning on a device that never had GNSS. FilenameAnchor is the honest
    // answer there; claiming tier 4 asserts confidence we do not have.
    {   // ToW-valid flag clear
        AnchorCollector c;
        feedTimeOnly(c, DID_PIMU, kUptimeMs);
        const ins_2_t p = makeIns2(static_cast<double>(kTowMs) / 1000.0, /*towValid=*/false);
        c.consume(DID_INS_2, 0, reinterpret_cast<const uint8_t*>(&p), sizeof(p), kTowMs);
        EXPECT_EQ(c.finish(nullptr).tier, AnchorTier::None);
    }
    {   // GPS week zero -- the device never had a week number
        AnchorCollector c;
        feedTimeOnly(c, DID_PIMU, kUptimeMs);
        const ins_2_t p = makeIns2(static_cast<double>(kTowMs) / 1000.0, /*towValid=*/true,
                                   /*week=*/0);
        c.consume(DID_INS_2, 0, reinterpret_cast<const uint8_t*>(&p), sizeof(p), kTowMs);
        EXPECT_EQ(c.finish(nullptr).tier, AnchorTier::None);
    }
    {   // GNSS receiver reporting no fix has no GPS time to offer, whatever its ToW field says
        AnchorCollector c;
        feedTimeOnly(c, DID_PIMU, kUptimeMs);
        const gnss_pos_t p = makeGnssPos(static_cast<uint32_t>(kTowMs), /*haveFix=*/false);
        c.consume(DID_GNSS1_POS, 0, reinterpret_cast<const uint8_t*>(&p), sizeof(p), kTowMs);
        EXPECT_EQ(c.finish(nullptr).tier, AnchorTier::None);
    }
    {   // and with no payload at all the candidate cannot be gated, so it is refused
        AnchorCollector c;
        feedTimeOnly(c, DID_PIMU, kUptimeMs);
        feedTimeOnly(c, DID_GNSS1_POS, kTowMs);
        EXPECT_EQ(c.finish(nullptr).tier, AnchorTier::None);
    }
}

TEST(AnchorCascade, BridgeOutranksTowOnly) {
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, kUptimeMs);
    feed(c, DID_GNSS1_POS, makeGnssPos(static_cast<uint32_t>(kTowMs), true), kTowMs);  // tier 4, first
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

TEST(AnchorCascade, DomainComesFromTheFieldNotTheMagnitude) {
    // Regression for the defect this cascade shipped with: the domain was decided by a
    // magnitude test (>= 1e8 meant "GPS time-of-week"). A ToW inside the first ~27.8 hours of
    // the week is BELOW that boundary, so it was filed as host uptime — mixing two bases in one
    // bucket. `uptimeMaxMs` then held a ToW value, and projecting it through the offset
    // double-counted it: a real segment reported a 7,149,064 ms span for ~218 s of data.
    //
    // These are the actual values from
    // goldenlogs/imx_gpx/imx5_gpx1/Compassing_Drive_GPX/20250405_200614 — every timestamp here
    // is below the old boundary, so under the magnitude test ALL of them landed in the uptime
    // bucket. 451,040 records across the golden corpus were misfiled this way.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU,          661'365);     // field `time`         -> uptime
    feedTimeOnly(c, DID_MAGNETOMETER,  661'369);     // field `time`         -> uptime
    feedTimeOnly(c, DID_PIMU,          879'681);     // field `time`         -> uptime
    feedTimeOnly(c, DID_INS_1,       7'592'210);     // field `timeOfWeek`   -> ToW
    feedTimeOnly(c, DID_GNSS1_POS,   7'592'200);     // field `timeOfWeekMs` -> ToW
    feedTimeOnly(c, DID_INS_1,       7'810'429);     // field `timeOfWeek`   -> ToW

    const AnchorAnalysis a = c.finish(nullptr);
    EXPECT_EQ(a.uptimeRecords, 3u);
    EXPECT_EQ(a.towRecords, 3u);
    EXPECT_EQ(a.uptimeMinMs, 661'365u) << "a ToW value must not pollute the uptime extrema";
    EXPECT_EQ(a.uptimeMaxMs, 879'681u) << "this is the value the old magnitude test corrupted";
    EXPECT_EQ(a.towMinMs, 7'592'200u);
    EXPECT_EQ(a.towMaxMs, 7'810'429u);
}

TEST(AnchorCascade, SmallTimeOfWeekStillBridgesCorrectly) {
    // The end-to-end consequence of the fix: with a sub-boundary ToW, the anchored span must
    // reflect the UPTIME extrema projected once — not the ToW extrema projected a second time.
    AnchorCollector c;
    feedTimeOnly(c, DID_PIMU, 661'365);
    feed(c, DID_SYS_PARAMS, makeSysParams(7'592'736u, 661.960, /*towValid=*/true), 7'592'736);
    feedTimeOnly(c, DID_INS_1, 7'810'429);   // ToW-domain: must NOT become uptimeMax
    feedTimeOnly(c, DID_PIMU, 879'681);

    const AnchorAnalysis a = c.finish(nullptr);
    ASSERT_EQ(a.tier, AnchorTier::PayloadToWBridge);
    const int64_t off = a.offsetMs;
    EXPECT_EQ(a.anchoredStartMs, static_cast<uint64_t>(661'365 + off));
    EXPECT_EQ(a.anchoredEndMs,   static_cast<uint64_t>(879'681 + off));
    EXPECT_EQ(a.anchoredEndMs - a.anchoredStartMs, 218'316u)
        << "span must match the real data extent, not the inter-domain gap";
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

TEST(AnchorCascade, NeedsPayloadNamesEveryAnchorCandidate) {
    // Tier-4 DIDs joined this list when tier 4 became validity-gated: the index-driven path must
    // re-frame their payloads too, or it would accept an ungated candidate and reach a tier the
    // byte-scan path correctly refuses -- a route disagreement.
    EXPECT_TRUE(AnchorCollector::needsPayload(DID_SYS_PARAMS));
    EXPECT_TRUE(AnchorCollector::needsPayload(DID_GPX_STATUS));
    EXPECT_TRUE(AnchorCollector::needsPayload(DID_GNSS1_POS));
    EXPECT_TRUE(AnchorCollector::needsPayload(DID_INS_1));
    EXPECT_FALSE(AnchorCollector::needsPayload(DID_PIMU)) << "not an anchor candidate";
    EXPECT_FALSE(AnchorCollector::needsPayload(DID_MAGNETOMETER));
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
// Session offset propagation — pure, no files (planSessionAdoptions)
// =====================================================================================

namespace {

//! A segment analysis with only the fields the planner reads.
AnchorAnalysis seg(AnchorTier tier, int64_t offsetMs, uint64_t upMin, uint64_t upMax,
                   uint32_t anchorDid = 0,
                   AnchorConsensus cons = AnchorConsensus::Uncorroborated) {
    AnchorAnalysis a;
    a.tier          = tier;
    a.consensus     = tier == AnchorTier::None ? AnchorConsensus::NotApplicable : cons;
    a.offsetMs      = offsetMs;
    a.uptimeMinMs   = upMin;
    a.uptimeMaxMs   = upMax;
    a.uptimeRecords = (upMin != 0) ? 100 : 0;
    a.anchorDid     = anchorDid;
    if (tier != AnchorTier::None) {
        a.anchoredStartMs = static_cast<uint64_t>(static_cast<int64_t>(upMin) + offsetMs);
        a.anchoredEndMs   = static_cast<uint64_t>(static_cast<int64_t>(upMax) + offsetMs);
    }
    return a;
}

} // namespace

TEST(SessionAdoption, OffsetPropagatesBACKWARDToEarlierSegments) {
    // THE case the old forward-only chain could not do. Segments 0..2 carry no absolute time;
    // segment 3 acquires a GPS fix. Everything needed to place 0..2 is in the log, and the
    // offset is a session constant, so all three must be re-anchored from a LATER segment.
    const int64_t off = 306'081'792;
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::None,             0,   1'000,  60'000),
        seg(AnchorTier::None,             0,  60'500, 120'000),
        seg(AnchorTier::None,             0, 120'500, 180'000),
        seg(AnchorTier::PayloadToWBridge, off, 180'500, 240'000, DID_SYS_PARAMS),
    };
    const auto plan = planSessionAdoptions(in);

    ASSERT_EQ(plan.size(), 3u) << "all three unanchored segments must adopt";
    for (const auto& a : plan) {
        EXPECT_LT(a.segment, 3u);
        EXPECT_EQ(a.offsetMs, off);
        EXPECT_EQ(a.donorDid, static_cast<uint32_t>(DID_SYS_PARAMS));
        EXPECT_FALSE(a.donorIsEarlier) << "the donor is segment 3 -- later than every adopter";
    }
}

TEST(SessionAdoption, OffsetPropagatesForwardToo) {
    const int64_t off = 306'081'792;
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::PayloadToWBridge, off, 1'000, 60'000, DID_SYS_PARAMS),
        seg(AnchorTier::None,             0,  60'500, 120'000),
    };
    const auto plan = planSessionAdoptions(in);
    ASSERT_EQ(plan.size(), 1u);
    EXPECT_EQ(plan[0].segment, 1u);
    EXPECT_TRUE(plan[0].donorIsEarlier);
}

TEST(SessionAdoption, OffsetDoesNotCrossAnUptimeReset) {
    // D0069 s4 / SN-8339: the offset is per-BOOT-SESSION. Segment 2's uptime restarts near
    // zero, so the device rebooted and segment 0's constant no longer applies to it.
    const int64_t off = 306'081'792;
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::PayloadToWBridge, off, 100'000, 160'000, DID_SYS_PARAMS),
        seg(AnchorTier::None,             0,   160'500, 220'000),
        seg(AnchorTier::None,             0,       500,  60'000),   // <-- reboot
        seg(AnchorTier::None,             0,    60'500, 120'000),
    };
    const auto plan = planSessionAdoptions(in);
    ASSERT_EQ(plan.size(), 1u) << "only the pre-reboot sibling may adopt";
    EXPECT_EQ(plan[0].segment, 1u);
}

TEST(SessionAdoption, EachSessionUsesItsOwnDonor) {
    const int64_t offA = 306'081'792, offB = 500'000'000;
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::PayloadToWBridge, offA, 100'000, 160'000, DID_SYS_PARAMS),
        seg(AnchorTier::None,             0,    160'500, 220'000),
        seg(AnchorTier::None,             0,        500,  60'000),        // reboot
        seg(AnchorTier::PayloadToWBridge, offB,  60'500, 120'000, DID_GPX_STATUS),
    };
    const auto plan = planSessionAdoptions(in);
    ASSERT_EQ(plan.size(), 2u);
    for (const auto& a : plan) {
        if (a.segment == 1) { EXPECT_EQ(a.offsetMs, offA); EXPECT_TRUE(a.donorIsEarlier); }
        if (a.segment == 2) { EXPECT_EQ(a.offsetMs, offB); EXPECT_FALSE(a.donorIsEarlier); }
    }
}

TEST(SessionAdoption, FirstHandAnchorsAreNeverOverridden) {
    const int64_t offA = 306'081'792, offB = 306'081'000;
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::PayloadToWBridge, offA, 1'000, 60'000, DID_SYS_PARAMS),
        seg(AnchorTier::PayloadToWSingle, offB, 60'500, 120'000, DID_GNSS1_POS),
    };
    const auto plan = planSessionAdoptions(in);
    EXPECT_TRUE(plan.empty()) << "both established their own anchor from their own payload";
}

TEST(SessionAdoption, StrongerTierWinsTheDonorRole) {
    const int64_t offSingle = 1'000, offBridge = 2'000;
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::PayloadToWSingle, offSingle, 1'000, 60'000, DID_GNSS1_POS),
        seg(AnchorTier::None,             0,        60'500, 120'000),
        seg(AnchorTier::PayloadToWBridge, offBridge, 120'500, 180'000, DID_SYS_PARAMS),
    };
    const auto plan = planSessionAdoptions(in);
    ASSERT_EQ(plan.size(), 1u);
    EXPECT_EQ(plan[0].offsetMs, offBridge) << "tier 5 outranks tier 4 regardless of position";
    EXPECT_EQ(plan[0].donorDid, static_cast<uint32_t>(DID_SYS_PARAMS));
}

TEST(SessionAdoption, BetterCorroborationBreaksATierTie) {
    const int64_t offLone = 1'000, offCorrob = 2'000;
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::PayloadToWBridge, offLone,   1'000, 60'000, DID_SYS_PARAMS,
            AnchorConsensus::Uncorroborated),
        seg(AnchorTier::None,             0,        60'500, 120'000),
        seg(AnchorTier::PayloadToWBridge, offCorrob, 120'500, 180'000, DID_GPX_STATUS,
            AnchorConsensus::Corroborated),
    };
    const auto plan = planSessionAdoptions(in);
    ASSERT_EQ(plan.size(), 1u);
    EXPECT_EQ(plan[0].offsetMs, offCorrob) << "equal tier -> the corroborated claim donates";
}

TEST(SessionAdoption, SegmentWithNoUptimeRecordsCannotAdopt) {
    // An uptime->ToW offset has nothing to project onto without uptime-domain records.
    const int64_t off = 306'081'792;
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::PayloadToWBridge, off, 1'000, 60'000, DID_SYS_PARAMS),
        seg(AnchorTier::None,             0,       0,      0),   // no uptime records at all
    };
    EXPECT_TRUE(planSessionAdoptions(in).empty());
}

TEST(SessionAdoption, NoDonorMeansNoPlan) {
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::None, 0, 1'000, 60'000),
        seg(AnchorTier::None, 0, 60'500, 120'000),
    };
    EXPECT_TRUE(planSessionAdoptions(in).empty());
    EXPECT_TRUE(planSessionAdoptions({}).empty());
}

TEST(SessionAdoption, AnInheritedAnchorIsNotItselfADonor) {
    // BridgedToW is a relay, not a source. If it could donate, one weak claim would propagate
    // indefinitely while looking increasingly well-established.
    std::vector<AnchorAnalysis> in = {
        seg(AnchorTier::BridgedToW, 306'081'792, 1'000, 60'000, DID_SYS_PARAMS),
        seg(AnchorTier::None,       0,          60'500, 120'000),
    };
    EXPECT_TRUE(planSessionAdoptions(in).empty());
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

TEST_F(AnchorSegmentTest, RebuiltIndexRecordsItsOwnProvenance) {
    // The rebuild used to hardcode `rec.flags = 0` and leave the header's seeded defaults in
    // place, so a scan-built index claimed: no record carried a GPS time-of-week, the log had
    // zero sync points, and its units were host-uptime. All three were false, and because a
    // rebuilt sidecar is persisted then trusted, the claim became authoritative on disk.
    fs::remove(f_.idxFile);
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    ASSERT_FALSE(r->hadOnDiskIndex()) << "expected the scan path";

    std::size_t towRecords = 0;
    for (auto v : r->allRecords()) {
        const bool hasTow = (v.flags() & idx::IS_LOG_IDX_REC_FLAG_HAS_TOW) != 0;
        if (hasTow) ++towRecords;
        // The bit must track the DID's timestamp DOMAIN, not merely "has a timestamp" --
        // that is the documented meaning ("a real GPS time-of-week field ... usable as a sync
        // anchor"), and it is what stops an uptime-domain DID being offered as an anchor.
        const bool towDomain =
            cISDataMappings::TimestampDomain(v.did())
                == cISDataMappings::eTimestampDomain::TIMESTAMP_DOMAIN_GPS_TOW;
        if (v.timestamp().value != 0) {
            EXPECT_EQ(hasTow, towDomain) << "DID " << v.did();
        } else {
            EXPECT_FALSE(hasTow) << "a record with no time cannot be a sync anchor";
        }
    }

    const auto& h = r->header();
    EXPECT_EQ(h.sync_point_count, towRecords) << "counter must agree with the per-record bits";
    EXPECT_NE(h.flags & idx::IS_LOG_IDX_HDR_FLAG_FINALIZED, 0u);
    EXPECT_EQ(h.flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_LOCAL_DELTA, 0u)
        << "a rebuild cannot recover receipt time -- it must not claim to have it";
}

TEST_F(AnchorSegmentTest, RebuiltHeaderUnitsMatchTheRecordsPresent) {
    fs::remove(f_.idxFile);
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());

    std::size_t tow = 0;
    for (auto v : r->allRecords())
        if (v.flags() & idx::IS_LOG_IDX_REC_FLAG_HAS_TOW) ++tow;

    const auto& h = r->header();
    const bool anyTow = tow > 0, anyNonTow = tow < r->recordCount();
    if (anyTow && anyNonTow) {
        EXPECT_EQ(h.ts_units, static_cast<uint8_t>(idx::TimestampUnits::Mixed));
        EXPECT_EQ(h.ts_source, static_cast<uint8_t>(idx::HeaderTimeSource::Mixed));
    } else if (anyTow) {
        EXPECT_EQ(h.ts_units, static_cast<uint8_t>(idx::TimestampUnits::GpsTowMs));
    } else {
        EXPECT_EQ(h.ts_units, static_cast<uint8_t>(idx::TimestampUnits::HostUptimeMs));
    }
}

TEST(TimestampDomain, ClassifiesByFieldNotMagnitude) {
    using D = cISDataMappings::eTimestampDomain;
    // `time` -> uptime
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_PIMU),         D::TIMESTAMP_DOMAIN_UPTIME);
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_MAGNETOMETER), D::TIMESTAMP_DOMAIN_UPTIME);
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_BAROMETER),    D::TIMESTAMP_DOMAIN_UPTIME);
    // `timeOfWeek` / `timeOfWeekMs` -> GPS ToW
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_INS_1),        D::TIMESTAMP_DOMAIN_GPS_TOW);
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_INS_2),        D::TIMESTAMP_DOMAIN_GPS_TOW);
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_SYS_PARAMS),   D::TIMESTAMP_DOMAIN_GPS_TOW);
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_GNSS1_POS),    D::TIMESTAMP_DOMAIN_GPS_TOW);
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_GPX_STATUS),   D::TIMESTAMP_DOMAIN_GPS_TOW);
    // Raw GNSS passthrough carries an absolute observation time in a different domain
    // entirely, and Timestamp() deliberately returns 0 for it — so: no domain.
    EXPECT_EQ(cISDataMappings::TimestampDomain(DID_GNSS1_RAW),    D::TIMESTAMP_DOMAIN_NONE);
    // Out of range is None, not a crash.
    EXPECT_EQ(cISDataMappings::TimestampDomain(0xFFFFFFFFu),      D::TIMESTAMP_DOMAIN_NONE);
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

// =====================================================================================
// SN-8704 — arrival-order interpolation (lifted from RawSeriesBuilder, SN-8131)
// =====================================================================================

TEST(ArrivalInterpolation, BracketsBetweenNeighbouringAnchors) {
    // (arrivalIndex, absoluteMs)
    const std::vector<std::pair<uint64_t, uint64_t>> anchors = {
        {100, 1'000}, {200, 2'000}, {400, 4'000},
    };
    EXPECT_EQ(ISTimeResolver::interpolateArrivalTime(anchors, 150), 1'500u);
    EXPECT_EQ(ISTimeResolver::interpolateArrivalTime(anchors, 200), 2'000u);
    EXPECT_EQ(ISTimeResolver::interpolateArrivalTime(anchors, 300), 3'000u);
}

TEST(ArrivalInterpolation, ClampsOutsideTheAnchorRange) {
    const std::vector<std::pair<uint64_t, uint64_t>> anchors = { {100, 1'000}, {200, 2'000} };
    EXPECT_EQ(ISTimeResolver::interpolateArrivalTime(anchors, 50),   1'000u) << "before first";
    EXPECT_EQ(ISTimeResolver::interpolateArrivalTime(anchors, 5'000), 2'000u) << "after last";
}

TEST(ArrivalInterpolation, IsMonotonicInArrivalIndex) {
    // Load-bearing: re-timed records must not step backwards relative to each other, or the
    // fix would trade one artefact for another.
    const std::vector<std::pair<uint64_t, uint64_t>> anchors = {
        {0, 500}, {37, 1'200}, {900, 9'999}, {1'000, 10'000},
    };
    uint64_t prev = 0;
    for (uint64_t k = 0; k <= 1'100; ++k) {
        const uint64_t t = ISTimeResolver::interpolateArrivalTime(anchors, k);
        EXPECT_GE(t, prev) << "backward step at arrival " << k;
        prev = t;
    }
}

TEST(ArrivalInterpolation, DegenerateInputsAreSafe) {
    EXPECT_EQ(ISTimeResolver::interpolateArrivalTime({}, 42), 0u) << "empty anchors";
    const std::vector<std::pair<uint64_t, uint64_t>> one = { {10, 777} };
    EXPECT_EQ(ISTimeResolver::interpolateArrivalTime(one, 5),  777u);
    EXPECT_EQ(ISTimeResolver::interpolateArrivalTime(one, 99), 777u);
    // Duplicate arrival indices must not divide by zero.
    const std::vector<std::pair<uint64_t, uint64_t>> dup = { {10, 100}, {10, 200}, {20, 300} };
    EXPECT_NO_FATAL_FAILURE(ISTimeResolver::interpolateArrivalTime(dup, 10));
}

// =====================================================================================
// SN-8704 — stalled-run ruler selection (planStallRetiming), pure, no files
// =====================================================================================

TEST(StallRetiming, CadenceCoversDidsWithNoCompanionUptime) {
    // 25 of the 27 ToW-bearing record structs have NO companion upTime (only sys_params_t and
    // gpx_status_t do), so most stalls MUST be repairable without one. The DID's own median
    // inter-record interval is the evidence: it assumes the DID's output rate is steady, which
    // is far weaker than assuming the global record rate is steady -- and a stall routinely
    // breaks the latter, because other DIDs stop emitting at the same instant.
    ISTimeResolver::StallEvidence ev;
    ev.stalledTsMs   = 500'000;
    ev.runArrivals   = { 100, 140, 181, 219, 260 };      // 5 records, irregular ARRIVAL spacing
    ev.advanceDeltas = { 98, 100, 102, 100, 101 };        // ~100 ms cadence, median 100
    // no ownSamples, no lastHealthyOwnMs -- the 25-of-27 case

    ISTimeResolver::StalledRun::Ruler kind{};
    const auto out = ISTimeResolver::planStallRetiming(ev, kind);

    EXPECT_EQ(kind, ISTimeResolver::StalledRun::Ruler::Cadence);
    ASSERT_EQ(out.size(), 5u);
    EXPECT_EQ(out[0].second, 500'000u) << "the run's first record keeps its genuine time";
    EXPECT_EQ(out[1].second, 500'100u);
    EXPECT_EQ(out[4].second, 500'400u) << "uniform at the DID's own cadence";
    for (std::size_t i = 0; i < out.size(); ++i)
        EXPECT_EQ(out[i].first, ev.runArrivals[i]) << "arrival keys preserved in order";
}

TEST(StallRetiming, OwnClockIsPreferredOverCadenceWhenBothExist) {
    // sys_params_t / gpx_status_t give a per-record answer, which beats an assumed-uniform one.
    ISTimeResolver::StallEvidence ev;
    ev.stalledTsMs      = 500'000;
    ev.runArrivals      = { 10, 20, 30 };
    ev.lastHealthyOwnMs = 1'000;
    ev.ownSamples       = { {10, 1'000}, {20, 1'503}, {30, 2'009} };  // real, slightly irregular
    ev.advanceDeltas    = { 500, 500, 500 };                          // cadence also available

    ISTimeResolver::StalledRun::Ruler kind{};
    const auto out = ISTimeResolver::planStallRetiming(ev, kind);

    EXPECT_EQ(kind, ISTimeResolver::StalledRun::Ruler::OwnClock);
    ASSERT_EQ(out.size(), 3u);
    EXPECT_EQ(out[0].second, 500'000u) << "seam is exact by construction";
    EXPECT_EQ(out[1].second, 500'503u) << "reproduces the device's OWN irregular spacing";
    EXPECT_EQ(out[2].second, 501'009u) << "not the uniform 500 ms a cadence ruler would assume";
}

TEST(StallRetiming, NoEvidenceMeansNoRuler) {
    ISTimeResolver::StalledRun::Ruler kind{};
    {   // nothing at all -> caller must bracket against the collective timeline
        ISTimeResolver::StallEvidence ev;
        ev.stalledTsMs = 500'000;
        ev.runArrivals = { 10, 20, 30 };
        EXPECT_TRUE(ISTimeResolver::planStallRetiming(ev, kind).empty());
        EXPECT_EQ(kind, ISTimeResolver::StalledRun::Ruler::None);
    }
    {   // a degenerate cadence of zero must not produce a run of identical times
        ISTimeResolver::StallEvidence ev;
        ev.stalledTsMs   = 500'000;
        ev.runArrivals   = { 10, 20 };
        ev.advanceDeltas = { 0, 0, 0 };
        EXPECT_TRUE(ISTimeResolver::planStallRetiming(ev, kind).empty());
        EXPECT_EQ(kind, ISTimeResolver::StalledRun::Ruler::None);
    }
    {   // a one-record "run" is not a run
        ISTimeResolver::StallEvidence ev;
        ev.stalledTsMs   = 500'000;
        ev.runArrivals   = { 10 };
        ev.advanceDeltas = { 100 };
        EXPECT_TRUE(ISTimeResolver::planStallRetiming(ev, kind).empty());
    }
    {   // own-clock samples present but no healthy pairing to anchor them -> cadence instead
        ISTimeResolver::StallEvidence ev;
        ev.stalledTsMs   = 500'000;
        ev.runArrivals   = { 10, 20 };
        ev.ownSamples    = { {10, 1'000}, {20, 1'500} };
        ev.lastHealthyOwnMs = 0;
        ev.advanceDeltas = { 250 };
        const auto out = ISTimeResolver::planStallRetiming(ev, kind);
        EXPECT_EQ(kind, ISTimeResolver::StalledRun::Ruler::Cadence);
        ASSERT_EQ(out.size(), 2u);
        EXPECT_EQ(out[1].second, 500'250u);
    }
}

TEST(StallRetiming, CadenceUsesTheMedianSoOneOutlierCannotSkewIt) {
    // A single long gap (a dropped record, a mode change) must not stretch the whole run.
    ISTimeResolver::StallEvidence ev;
    ev.stalledTsMs   = 0;
    ev.runArrivals   = { 1, 2, 3, 4 };
    ev.advanceDeltas = { 100, 100, 9'000, 100, 100 };   // median 100, mean would be ~1880
    ISTimeResolver::StalledRun::Ruler kind{};
    const auto out = ISTimeResolver::planStallRetiming(ev, kind);
    ASSERT_EQ(out.size(), 4u);
    EXPECT_EQ(out[3].second, 300u) << "median cadence, not mean";
}

TEST(StallRetiming, RetimedTimesAreStrictlyAscendingUnderBothRulers) {
    // Load-bearing: re-timed records must not step backwards among themselves.
    ISTimeResolver::StalledRun::Ruler kind{};
    {
        ISTimeResolver::StallEvidence ev;
        ev.stalledTsMs   = 1'000;
        ev.advanceDeltas = { 7 };
        for (uint64_t i = 0; i < 500; ++i) ev.runArrivals.push_back(i * 3);
        const auto out = ISTimeResolver::planStallRetiming(ev, kind);
        ASSERT_EQ(out.size(), 500u);
        for (std::size_t i = 1; i < out.size(); ++i)
            EXPECT_GT(out[i].second, out[i-1].second) << "cadence, at i=" << i;
    }
    {
        ISTimeResolver::StallEvidence ev;
        ev.stalledTsMs      = 1'000;
        ev.lastHealthyOwnMs = 100;
        for (uint64_t i = 0; i < 500; ++i) ev.ownSamples.emplace_back(i, 100 + i * 11);
        ev.runArrivals.assign(500, 0);
        for (uint64_t i = 0; i < 500; ++i) ev.runArrivals[i] = i;
        const auto out = ISTimeResolver::planStallRetiming(ev, kind);
        ASSERT_EQ(out.size(), 500u);
        for (std::size_t i = 1; i < out.size(); ++i)
            EXPECT_GT(out[i].second, out[i-1].second) << "own clock, at i=" << i;
    }
}
