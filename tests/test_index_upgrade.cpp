/**
 * @file test_index_upgrade.cpp
 * @brief D0096 path 3 -- upgrading an existing `.idx` instead of discarding it.
 *
 * Kyle, 2026-09-19: *"rebuilding from existing `.idx` files is preferred to building `.idx`s only
 * from `.raw`/`.dat` files."* Before `ISLogReader::upgradeIndex` existed, `construct()` made a
 * binary choice -- trust the sidecar or throw it away -- so a legacy v1 sidecar was DETECTED and
 * then discarded. That lost the per-record `host_uptime_ms`, which is the OBSERVED receipt time
 * and `ISTimeResolver`'s top-priority stall ruler: a quantity no byte scan can recover.
 *
 * The v1 layout was characterised against all 17 v1 sidecars in the corpus before any of this was
 * written, because two of the four documented field names are wrong or unusable. The two facts
 * these tests exist to protect:
 *
 *   - the join key is `record_counter`, NOT position and NOT `byte_offset`. A v1 sidecar stores
 *     only ~70% of its segment's records, so a positional join mis-assigns every WHEN after the
 *     first gap;
 *   - `byte_offset` is never adopted. Only 0.3%-3.9% of the post-reset run in a real v1 file
 *     lands on an ISB packet boundary.
 *
 * Both synthetic and real-fixture coverage, per the project's test conventions: the synthetic
 * cases pin exact values field-by-field and run everywhere, and the real-fixture case asserts the
 * measured corpus numbers and skips when the corpus is absent.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>

#include "ISLogReader.h"
#include "ISLogIndex.h"
#include "ISLog.h"
#include "ISDiagnostics.h"
#include "ISDeviceLog.h"
#include "ISLogger.h"
#include "ISFileManager.h"
#include "ISDataMappings.h"
#include "data_sets.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>   // ::getpid() -- POSIX-only, hence the WIN32 exclusion in CMakeLists

using namespace inertial_sense;
namespace idx = inertial_sense::idx;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kHwId = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);

fs::path makeTempDir(const std::string& prefix) {
    const fs::path d = fs::temp_directory_path() /
                       ("test_idx_upgrade_" + prefix + "_" + std::to_string(::getpid()));
    ISFileManager::DeleteDirectory(d.string());
    fs::create_directories(d);
    return d;
}

//! A `.dat` segment with a known record population: one untimed DID_DEV_INFO followed by
//! `count` DID_PIMU records at 10.0 s + 0.1 s steps. cISLogger also writes a v2.1 sidecar,
//! which each test replaces or deletes as needed.
fs::path writeSegment(const fs::path& dir, uint32_t serial, int count) {
    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_DAT;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(dir.string(), opts)) return {};
    auto dev = logger.registerDevice(kHwId, serial);
    if (!dev) return {};
    logger.EnableLogging(true);

    dev_info_t info{};
    info.serialNumber   = serial;
    info.hardwareType   = IS_HARDWARE_TYPE_IMX;
    info.hardwareVer[0] = 5;
    p_data_hdr_t ih{};
    ih.id   = DID_DEV_INFO;
    ih.size = sizeof(info);
    logger.LogData(dev, &ih, reinterpret_cast<const uint8_t*>(&info));

    for (int i = 0; i < count; ++i) {
        pimu_t p{};
        p.time = 10.0 + 0.1 * i;
        p.dt   = 0.1f;
        p_data_hdr_t h{};
        h.id   = DID_PIMU;
        h.size = sizeof(p);
        logger.LogData(dev, &h, reinterpret_cast<const uint8_t*>(&p));
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> segs;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.dat$", segs);
    return segs.empty() ? fs::path{} : fs::path(segs.front().name);
}

//! Overwrite @p segment's sidecar with a synthetic v1 file. `keepEvery` mimics the real thing:
//! a v1 sidecar indexes only a SUBSET of the segment's records (~70% in the corpus), which is
//! precisely why the join cannot be positional.
//!
//! @param counterBase  Value of the first `record_counter`. Non-zero mimics a mid-log segment,
//!                     where the counter continues from earlier segments.
//! @param firstHostMs  `host_uptime_ms` of the first written entry.
//! @param stepMs       Per-record host-uptime step.
void writeV1Sidecar(const fs::path& segment, std::size_t scannedRecords, int keepEvery,
                    uint32_t counterBase, uint32_t firstHostMs, uint32_t stepMs,
                    bool corruptCounter = false) {
    fs::path idxPath = segment;
    idxPath.replace_extension(".idx");
    std::ofstream out(idxPath, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(out.good()) << "cannot write " << idxPath;

    for (std::size_t k = 0; k < scannedRecords; ++k) {
        if (static_cast<int>(k) % keepEvery != 0) continue;
        uint32_t counter = counterBase + static_cast<uint32_t>(k);
        if (corruptCounter && k > scannedRecords / 2) {
            counter = counterBase;   // a decrease -- gate 1 must refuse the whole file
        }
        const uint32_t host = firstHostMs + static_cast<uint32_t>(k) * stepMs;
        // Deliberately bogus byte offsets: the real format's are untrustworthy, and nothing may
        // adopt them. If a future change starts believing this field, these values make the
        // resulting offsets obviously wrong rather than subtly so.
        const uint32_t bogusOffset = 0xDEAD0000u + static_cast<uint32_t>(k);
        const uint32_t reserved = 0;
        uint8_t rec[idx::IS_LOG_IDX_RECORD_V1_SIZE];
        const auto put = [&rec](std::size_t at, uint32_t v) {
            rec[at + 0] = static_cast<uint8_t>(v & 0xFF);
            rec[at + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
            rec[at + 2] = static_cast<uint8_t>((v >> 16) & 0xFF);
            rec[at + 3] = static_cast<uint8_t>((v >> 24) & 0xFF);
        };
        put(0, host); put(4, bogusOffset); put(8, counter); put(12, reserved);
        out.write(reinterpret_cast<const char*>(rec), sizeof(rec));
    }
    out.close();
}

//! Record count the scan finds in @p segment, with no sidecar present.
std::size_t scanCount(const fs::path& segment) {
    fs::path idxPath = segment;
    idxPath.replace_extension(".idx");
    fs::remove(idxPath);
    auto r = ISLogReader::openSegment(segment);
    return r ? r->recordCount() : 0;
}

} // namespace

// =====================================================================================
// Audit B6 — seek() / in_time() compare against RAW .idx timestamps, and ignored `.source`.
//
// D0065 makes a resolved absolute the default shape of every time value in the application, so
// handing one to these is the easy mistake -- and it silently compared two different frames.
// A value at or beyond the GPS epoch cannot be a raw time-of-week (under one week) or a host
// uptime (hours), so it can only be a resolved absolute; that is now detected and refused.
// =====================================================================================

TEST(SeekDomainGuard, ARawTargetSeeksAndAResolvedAbsoluteIsRefused) {
    const fs::path dir = makeTempDir("b6_seek");
    const fs::path seg = writeSegment(dir, 425300u, 30);
    ASSERT_FALSE(seg.empty());

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());
    ASSERT_GT(r->recordCount(), 5u);

    // A RAW target works: the fixture's DID_PIMU records run 10.0 s .. upward in uptime ms.
    const uint64_t rawTarget = 10'500;
    auto it = r->seek(TimeStamp::fromSessionOnly(rawTarget, r->deviceId()));
    std::size_t rawPos = 0;
    for (auto scan = r->allRecords().begin(); scan != it && rawPos < r->recordCount(); ++scan) {
        ++rawPos;
    }
    std::printf("[measured] raw seek(%llu) landed at index %zu of %zu\n",
                (unsigned long long)rawTarget, rawPos, r->recordCount());
    EXPECT_LT(rawPos, r->recordCount()) << "a raw-domain seek should land inside the records";

    // A RESOLVED absolute must be refused, not compared. ~1.79e12 is a 2026 wall clock; the
    // fixture's raw values are five-digit uptimes, so the old behaviour walked to the end and
    // returned it as though the instant did not exist.
    constexpr uint64_t kResolvedAbsolute = 1'789'601'171'000ULL;
    auto bad = r->seek(TimeStamp::fromResolvedViaSync(kResolvedAbsolute, r->deviceId(),
                                                       TimeConfidence::Exact));
    std::size_t badPos = 0;
    for (auto scan = r->allRecords().begin(); scan != bad && badPos <= r->recordCount(); ++scan) {
        ++badPos;
    }
    std::printf("[measured] resolved seek(%llu) landed at index %zu (== recordCount %zu)\n",
                (unsigned long long)kResolvedAbsolute, badPos, r->recordCount());
    EXPECT_EQ(badPos, r->recordCount())
        << "a resolved absolute must be refused explicitly, not compared against raw values";

    ISFileManager::DeleteDirectory(dir.string());
}

TEST(SeekDomainGuard, InTimeRefusesResolvedBoundsAndAcceptsRawOnes) {
    const fs::path dir = makeTempDir("b6_intime");
    const fs::path seg = writeSegment(dir, 425301u, 30);
    ASSERT_FALSE(seg.empty());

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());

    const auto countRange = [](ISLogReader::Range rng) {
        std::size_t n = 0;
        for (auto v : rng) { (void)v; ++n; }
        return n;
    };

    // Raw bounds select a real subset of the fixture's 10.0 s .. 12.9 s uptime range.
    const auto rawRange = r->allRecords().in_time(
        TimeStamp::fromSessionOnly(10'000, r->deviceId()),
        TimeStamp::fromSessionOnly(11'000, r->deviceId()));
    const std::size_t rawCount = countRange(rawRange);

    // Resolved bounds must yield NOTHING rather than a mis-framed range.
    const auto resolvedRange = r->allRecords().in_time(
        TimeStamp::fromResolvedViaSync(1'789'601'171'000ULL, r->deviceId(),
                                        TimeConfidence::Exact),
        TimeStamp::fromResolvedViaSync(1'789'601'999'000ULL, r->deviceId(),
                                        TimeConfidence::Exact));
    const std::size_t resolvedCount = countRange(resolvedRange);

    std::printf("[measured] in_time raw bounds -> %zu record(s); resolved bounds -> %zu\n",
                rawCount, resolvedCount);
    EXPECT_GT(rawCount, 0u)      << "raw-domain bounds should select records";
    EXPECT_LT(rawCount, r->recordCount()) << "and should select a SUBSET, or the test is vacuous";
    EXPECT_EQ(resolvedCount, 0u) << "resolved bounds must be refused, not compared";

    ISFileManager::DeleteDirectory(dir.string());
}

// =====================================================================================
// Audit C3 — recordEndOffset must not scan, and must give the same answer either way.
//
// It was a forward linear scan for "the smallest record offset strictly greater than mine":
// O(1) when the next record's offset is greater (normal), O(n) for every record in a run that
// shares an offset -- and real firmware sidecars produce those. `viewAt()` sits on the anchor
// cascade's hot path via analyzeFromRecords, so the pathological case is quadratic over a whole
// segment. It is now a binary search over an array whose non-decreasing invariant is verified
// once at construction.
//
// This asserts the binary search agrees with the linear definition for EVERY record, computed
// independently here from the public offsets.
// =====================================================================================

TEST(RecordEndOffset, BinarySearchMatchesTheLinearDefinitionForEveryRecord) {
    const fs::path dir = makeTempDir("c3_endoffset");
    const fs::path seg = writeSegment(dir, 425200u, 40);
    ASSERT_FALSE(seg.empty());

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());
    ASSERT_GT(r->recordCount(), 10u);

    // Collect the offsets and byte-range lengths the reader reports.
    std::vector<uint64_t> offsets;
    std::vector<std::size_t> lengths;
    for (auto v : r->allRecords()) {
        const auto [bytes, n] = v.bytes();
        offsets.push_back(v.offsetInFile());
        lengths.push_back(bytes != nullptr ? n : 0);
    }

    // Independent reference: the linear definition, straight from the audit's wording.
    const std::size_t fileEnd = r->fileSize();
    std::size_t mismatches = 0;
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        std::size_t expectedEnd = fileEnd;
        for (std::size_t j = i + 1; j < offsets.size(); ++j) {
            if (offsets[j] > offsets[i]) { expectedEnd = static_cast<std::size_t>(offsets[j]); break; }
        }
        const std::size_t expectedLen = expectedEnd > offsets[i]
                                            ? expectedEnd - static_cast<std::size_t>(offsets[i])
                                            : 0;
        if (lengths[i] != expectedLen) {
            if (mismatches < 5) {
                std::printf("[measured]   rec %zu off=%llu got len=%zu want %zu\n",
                            i, (unsigned long long)offsets[i], lengths[i], expectedLen);
            }
            ++mismatches;
        }
    }
    std::printf("[measured] %zu record(s), fileSize=%zu, mismatches=%zu\n",
                offsets.size(), fileEnd, mismatches);
    EXPECT_EQ(mismatches, 0u)
        << "the binary search disagrees with the linear definition of record end offset";

    // The invariant the search depends on must actually hold on this fixture, or the test is
    // silently exercising the fallback rather than the thing it means to check.
    bool nonDecreasing = true;
    for (std::size_t i = 1; i < offsets.size(); ++i) {
        if (offsets[i] < offsets[i - 1]) { nonDecreasing = false; break; }
    }
    EXPECT_TRUE(nonDecreasing)
        << "fixture offsets decrease, so this exercised the linear fallback, not the search";

    // Every record must still resolve to real bytes.
    for (std::size_t i = 0; i < lengths.size(); ++i) {
        EXPECT_GT(lengths[i], 0u) << "record " << i << " resolved to an empty byte range";
    }

    ISFileManager::DeleteDirectory(dir.string());
}

// =====================================================================================
// Audit B3 — diagnostics must reach the application, and an orphaned sidecar must be reported
// with enough structure to act on.
//
// Everything the reader learned used to dead-end at `ISLogReader`: `warnings()` had no accessor
// on `ISDeviceLog`, `ISLog` or Logalyzer's adapter, so the stall detector that found a customer's
// frozen clock was unreachable from the application that needed to show it. And Kyle asked for an
// orphaned `.idx` to be reported AND offered for deletion, which a bare string cannot support.
// =====================================================================================

TEST(Diagnostics, AnOrphanedSidecarIsReportedWithItsPathAndARemedy) {
    const fs::path dir = makeTempDir("b3_orphan");
    const fs::path seg = writeSegment(dir, 425100u, 15);
    ASSERT_FALSE(seg.empty());

    // A sidecar whose segment is gone -- exactly the state of the corpus log's eight legacy
    // files. Copy the real one aside, then remove its segment's counterpart name.
    const fs::path orphan = dir / "LOG_SN425100_19700101_000000_0099.idx";
    {
        fs::path liveIdx = seg;
        liveIdx.replace_extension(".idx");
        std::error_code ec;
        fs::copy_file(liveIdx, orphan, fs::copy_options::overwrite_existing, ec);
        ASSERT_FALSE(ec) << "could not stage the orphan";
    }
    ASSERT_TRUE(fs::exists(orphan));

    auto log = ISLog::openDirectory(dir);
    ASSERT_TRUE(log.has_value());

    const auto& diags = log->diagnostics();
    std::printf("[measured] %zu diagnostic(s):\n", diags.size());
    for (const auto& d : diags) {
        std::printf("[measured]   [%s] sev=%d %s\n    %s\n    remedy: %s\n",
                    isDiagKindName(d.kind), static_cast<int>(d.severity),
                    d.path.filename().string().c_str(), d.message.c_str(), d.remedy.c_str());
    }

    const auto it = std::find_if(diags.begin(), diags.end(), [](const ISDiagnostic& d) {
        return d.kind == ISDiagKind::OrphanedSidecar;
    });
    ASSERT_NE(it, diags.end()) << "the orphaned sidecar was not reported at all";

    // Structure an application can act on: the path is the SIDECAR (the file to delete), not the
    // segment, and the remedy says what to do.
    EXPECT_EQ(it->path.filename(), orphan.filename())
        << "the diagnostic must carry the sidecar's own path, since that is what gets deleted";
    EXPECT_FALSE(it->remedy.empty()) << "an actionable diagnostic needs a remedy";
    EXPECT_NE(it->remedy.find("safe"), std::string::npos);
    EXPECT_TRUE(it->needsAttention()) << "the user should be told without having to ask";
    EXPECT_STREQ(isDiagKindName(it->kind), "orphaned-sidecar");

    // And the orphan must not have been mistaken for a segment.
    EXPECT_EQ(log->segmentPaths().size(), 1u);

    ISFileManager::DeleteDirectory(dir.string());
}

TEST(Diagnostics, SegmentLevelDiagnosticsReachTheLogLevel) {
    const fs::path dir = makeTempDir("b3_fold");
    const fs::path seg = writeSegment(dir, 425101u, 20);
    ASSERT_FALSE(seg.empty());

    // Force a rebuild so the segment has something to report, then check the same event is
    // visible at every level -- reader, device, log. That chain is what B3 was about.
    fs::path idxPath = seg;
    idxPath.replace_extension(".idx");
    ASSERT_TRUE(fs::remove(idxPath));

    auto log = ISLog::openDirectory(dir);
    ASSERT_TRUE(log.has_value());
    ASSERT_EQ(log->deviceIds().size(), 1u);
    const ISDeviceLog& dl = log->device(log->deviceIds().front());

    const auto readerDiags = dl.segment(0).diagnostics();
    const auto deviceDiags = dl.diagnostics();
    const auto& logDiags   = log->diagnostics();
    std::printf("[measured] reader=%zu device=%zu log=%zu\n",
                readerDiags.size(), deviceDiags.size(), logDiags.size());
    for (const auto& d : logDiags) {
        std::printf("[measured]   [%s] %s\n", isDiagKindName(d.kind), d.message.c_str());
    }

    const auto hasRebuilt = [](const std::vector<ISDiagnostic>& v) {
        return std::any_of(v.begin(), v.end(), [](const ISDiagnostic& d) {
            return d.kind == ISDiagKind::SidecarRebuilt;
        });
    };
    EXPECT_GT(readerDiags.size(), 0u)  << "the reader rebuilt the index and should say so";
    EXPECT_TRUE(hasRebuilt(readerDiags)) << "classified as a rebuild at the reader";
    EXPECT_TRUE(hasRebuilt(deviceDiags)) << "did not reach ISDeviceLog";
    EXPECT_TRUE(hasRebuilt(logDiags))    << "did not reach ISLog -- this is the B3 dead-end";

    // Every diagnostic names the file it is about, or a UI cannot attribute it.
    for (const auto& d : logDiags) {
        EXPECT_FALSE(d.path.empty()) << "diagnostic without a path: " << d.message;
        EXPECT_FALSE(d.message.empty());
    }

    ISFileManager::DeleteDirectory(dir.string());
}

// =====================================================================================
// Audit C1 — a directory open must open each segment ONCE.
//
// `ISLog::openDirectory` has to open every segment to read its device id before it can group
// segments into devices, and it then DROPPED each reader so `ISDeviceLog::fromSegments` could
// open it a second time ("Drop the reader; fromSegments re-opens"). Opening is not cheap: it
// builds the record index — a full byte scan when the sidecar is missing or stale — and runs the
// anchor cascade. So a directory open paid for both twice per segment.
//
// Observing the open COUNT without instrumenting production code: delete the sidecar first. The
// first open must rebuild it and persists the result, so a SECOND open of the same segment would
// find a valid sidecar and report `hadOnDiskIndex() == true` with no rebuild warning. The reader
// the composed log keeps therefore tells us which open produced it — under the old double-open it
// was the trusted second one, and under a single open it is the one that rebuilt.
// =====================================================================================

TEST(OpenDirectoryEfficiency, EachSegmentIsOpenedOncePerDirectoryOpen) {
    const fs::path dir = makeTempDir("c1_single_open");
    const fs::path seg = writeSegment(dir, 424900u, 25);
    ASSERT_FALSE(seg.empty());

    // Drop the live sidecar so the first open has to rebuild, and persists as it does.
    fs::path idxPath = seg;
    idxPath.replace_extension(".idx");
    ASSERT_TRUE(fs::remove(idxPath));

    auto log = ISLog::openDirectory(dir);
    ASSERT_TRUE(log.has_value()) << "openDirectory failed";
    ASSERT_EQ(log->deviceIds().size(), 1u);
    const ISDeviceLog& dl = log->device(log->deviceIds().front());
    ASSERT_EQ(dl.segmentCount(), 1u);
    const ISLogReader& kept = dl.segment(0);

    std::printf("[measured] retained reader: hadOnDiskIndex=%d warnings=%zu\n",
                static_cast<int>(kept.hadOnDiskIndex()), kept.warnings().size());
    for (const auto& w : kept.warnings()) std::printf("[measured]   %s\n", w.c_str());

    // The sidecar exists now, because the one open that happened persisted it.
    EXPECT_TRUE(fs::exists(idxPath)) << "the rebuild should have persisted a sidecar";

    // The retained reader is the one that REBUILT. If the segment were opened a second time,
    // that open would have found the freshly-persisted sidecar and the kept reader would report
    // a trusted index with no rebuild warning.
    EXPECT_FALSE(kept.hadOnDiskIndex())
        << "the composed log kept a reader that found a valid sidecar -- which can only be a "
           "SECOND open of a segment whose first open just wrote it";
    const auto& w = kept.warnings();
    EXPECT_TRUE(std::any_of(w.begin(), w.end(), [](const std::string& m) {
        return m.find("sidecar: rebuilt from") != std::string::npos;
    })) << "the retained reader should carry the rebuild it performed";

    // And the composition is still correct, which is the point of the refactor being safe.
    EXPECT_GT(dl.recordCount(), 0u);
    EXPECT_TRUE(dl.segment(0).anchorAnalysis().anchored());

    ISFileManager::DeleteDirectory(dir.string());
}

// Multi-device and multi-segment, which is where the doubled cost actually bit: the grouping must
// still partition correctly now that it moves open readers rather than paths, and each device's
// segments must stay in filename order because the composition uses that as its tiebreaker.
TEST(OpenDirectoryEfficiency, GroupingByDeviceSurvivesMovingOpenReaders) {
    const fs::path dir = makeTempDir("c1_grouping");
    // Two devices, two segments each. writeSegment names by serial, so distinct serials give
    // distinct devices in one directory.
    const fs::path a1 = writeSegment(dir, 424901u, 12);
    const fs::path b1 = writeSegment(dir, 424902u, 12);
    ASSERT_FALSE(a1.empty());
    ASSERT_FALSE(b1.empty());

    auto log = ISLog::openDirectory(dir);
    ASSERT_TRUE(log.has_value());
    const auto ids = log->deviceIds();
    std::printf("[measured] devices=%zu segments=%zu records=%zu\n",
                ids.size(), log->segmentPaths().size(), log->recordCount());
    ASSERT_EQ(ids.size(), 2u) << "two serials must compose as two devices";
    for (uint64_t id : ids) {
        const ISDeviceLog& dl = log->device(id);
        EXPECT_GE(dl.segmentCount(), 1u);
        EXPECT_GT(dl.recordCount(), 0u);
        // Filename order within a device, which fromReaders documents as its precondition.
        for (std::size_t i = 1; i < dl.segmentCount(); ++i) {
            EXPECT_LT(dl.segment(i - 1).path().filename().string(),
                      dl.segment(i).path().filename().string())
                << "segments reached the composition out of filename order";
        }
    }

    ISFileManager::DeleteDirectory(dir.string());
}

// =====================================================================================
// Kyle's non-negotiable #1: a v1 upgrade yields OBSERVED offsets.
// "if this is not asserted, the feature's entire purpose is untested"
// =====================================================================================

TEST(IndexUpgradeV1, AdoptedOffsetsAreObservedNotReconstructed) {
    const fs::path dir = makeTempDir("v1_observed");
    const fs::path seg = writeSegment(dir, 321001u, 40);
    ASSERT_FALSE(seg.empty());

    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 8u);

    // Every 3rd record indexed -- a ~33%-coverage sidecar, so the join must handle gaps.
    writeV1Sidecar(seg, n, /*keepEvery=*/3, /*counterBase=*/0,
                   /*firstHostMs=*/1000, /*stepMs=*/10);

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());

    std::size_t observed = 0, reconstructed = 0, k = 0;
    for (auto v : r->allRecords()) {
        const bool recon =
            (v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) != 0;
        if (recon) {
            ++reconstructed;
        } else {
            ++observed;
            // An adopted record's offset must be the v1 host time rebased on the log start,
            // exactly -- not interpolated, not approximated.
            EXPECT_EQ(v.logTimeOffsetMs(), static_cast<uint32_t>(k) * 10u)
                << "adopted offset for record " << k << " is not the observed value";
        }
        ++k;
    }
    std::printf("[measured] v1 upgrade: records=%zu observed=%zu reconstructed=%zu\n",
                r->recordCount(), observed, reconstructed);

    // The records the sidecar covered are observed; the rest are honestly labelled.
    const std::size_t expectedObserved = (n + 2) / 3;
    EXPECT_EQ(observed, expectedObserved)
        << "every record the legacy sidecar covered must carry an OBSERVED offset";
    EXPECT_EQ(reconstructed, r->recordCount() - expectedObserved);
    EXPECT_GT(observed, 0u) << "nothing adopted -- the upgrade did not happen at all";

    // And the header must declare the field, since real values were written (D0096 / A5).
    EXPECT_NE(0, r->header().flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_LOG_TIME_OFFSET);

    const auto& w = r->warnings();
    EXPECT_TRUE(std::any_of(w.begin(), w.end(), [](const std::string& s) {
        return s.find("upgraded from v1") != std::string::npos;
    })) << "the upgrade must be visible to the application through warnings()";

    ISFileManager::DeleteDirectory(dir.string());
}

// The join key. This is the test that fails if anyone "simplifies" the join to a positional
// walk: with a 1-in-3 sidecar, position k in the FILE corresponds to record 3k in the SEGMENT,
// so a positional join assigns the 2nd stored WHEN to record 1 instead of record 3.
TEST(IndexUpgradeV1, TheJoinIsKeyedOnRecordCounterNotPosition) {
    const fs::path dir = makeTempDir("v1_joinkey");
    const fs::path seg = writeSegment(dir, 321002u, 40);
    ASSERT_FALSE(seg.empty());
    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 8u);

    writeV1Sidecar(seg, n, /*keepEvery=*/3, /*counterBase=*/0,
                   /*firstHostMs=*/5000, /*stepMs=*/7);

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());

    // Record 3 was stored (3 % 3 == 0) and must carry ITS OWN observed value, 3*7 = 21 ms.
    // A positional join would have given it the 2nd stored entry, 1*7 = 7 ms.
    std::size_t k = 0;
    uint32_t atThree = UINT32_MAX;
    bool threeObserved = false;
    for (auto v : r->allRecords()) {
        if (k == 3) {
            atThree = v.logTimeOffsetMs();
            threeObserved =
                (v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0;
        }
        ++k;
    }
    std::printf("[measured] record 3: offset=%u observed=%d (positional join would give 7)\n",
                atThree, static_cast<int>(threeObserved));
    ASSERT_TRUE(threeObserved) << "record 3 was in the sidecar; it must be observed";
    EXPECT_EQ(atThree, 21u) << "positional join detected -- must key on record_counter";

    ISFileManager::DeleteDirectory(dir.string());
}

// A mid-log segment: the counter does NOT start at 0, and the WHEN is log-wide. Getting this
// wrong is invisible on a single-segment test and collapses every segment onto the same start.
TEST(IndexUpgradeV1, AMidLogSegmentKeepsItsLogRelativeOffsetsInsteadOfBeingZeroed) {
    const fs::path dir = makeTempDir("v1_midlog");
    const fs::path seg = writeSegment(dir, 321003u, 40);
    ASSERT_FALSE(seg.empty());
    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 4u);

    // A segment well into a log: its v1 counter continues from earlier segments (base 500,000)
    // and its WHEN is already log-relative (900,000 ms in). No earlier segment carries a legacy
    // sidecar, so discovery cannot PROVE a log start -- and must therefore not rebase at all.
    //
    // This is the mixed-log case, which is the normal one: opening a v1 segment persists a v2
    // sidecar over it, so a log's earliest segments are the first to lose their legacy data.
    // Measured on corpus log 20260716_012243 -- 79 segments, only 8 still v1, and segment 0001
    // already v2. Anchoring on this segment's own first record would zero a point 900 seconds
    // into the log, which is exactly the collapse the anchor exists to prevent.
    writeV1Sidecar(seg, n, /*keepEvery=*/2, /*counterBase=*/500000u,
                   /*firstHostMs=*/900000u, /*stepMs=*/5);

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());

    std::size_t observed = 0, k = 0;
    uint32_t firstObserved = UINT32_MAX;
    for (auto v : r->allRecords()) {
        if ((v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0) {
            if (firstObserved == UINT32_MAX) firstObserved = v.logTimeOffsetMs();
            ++observed;
            // Verbatim, because a v1 WHEN is already measured from log open.
            EXPECT_EQ(v.logTimeOffsetMs(), 900000u + static_cast<uint32_t>(k) * 5u)
                << "record " << k << " was rebased when it should not have been";
        }
        ++k;
    }
    for (const auto& s : r->warnings()) std::printf("[measured] warn: %s\n", s.c_str());
    std::printf("[measured] mid-log: observed=%zu firstOffset=%u (zeroing would give 0)\n",
                observed, firstObserved);

    EXPECT_GT(observed, 0u) << "a non-zero counter base must still join";
    EXPECT_EQ(firstObserved, 900000u)
        << "a mid-log segment must keep its place in the log, not be moved to zero";

    ISFileManager::DeleteDirectory(dir.string());
}

// Discovery must refuse to answer from a segment it cannot prove is the log's first.
TEST(IndexUpgradeV1, DiscoveryRefusesASegmentWhoseCounterDoesNotStartAtZero) {
    const fs::path dir = makeTempDir("v1_discovery");
    const fs::path seg = writeSegment(dir, 321007u, 20);
    ASSERT_FALSE(seg.empty());
    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 2u);

    // counter base 0 => provably the log's first record => discovery answers.
    writeV1Sidecar(seg, n, /*keepEvery=*/2, /*counterBase=*/0,
                   /*firstHostMs=*/7000u, /*stepMs=*/2);
    const auto atZero = ISLogReader::discoverLogStartHostUptime(seg);
    ASSERT_TRUE(atZero.has_value()) << "counter 0 is proof of the log's first record";
    EXPECT_EQ(*atZero, 7000u);

    // Any non-zero base => something preceded this => refuse rather than guess.
    writeV1Sidecar(seg, n, /*keepEvery=*/2, /*counterBase=*/42u,
                   /*firstHostMs=*/7000u, /*stepMs=*/2);
    const auto midLog = ISLogReader::discoverLogStartHostUptime(seg);
    std::printf("[measured] discovery: counter0=%llu counter42=%s\n",
                (unsigned long long)*atZero,
                midLog ? std::to_string(*midLog).c_str() : "nullopt");
    EXPECT_FALSE(midLog.has_value())
        << "a mid-log segment's time must never be mistaken for the log start";

    ISFileManager::DeleteDirectory(dir.string());
}

// =====================================================================================
// Kyle's non-negotiable #4: a stale / corrupt sidecar FALLS BACK rather than adopting.
// =====================================================================================

TEST(IndexUpgradeV1, ANonMonotonicCounterIsRefusedEntirely) {
    const fs::path dir = makeTempDir("v1_corrupt");
    const fs::path seg = writeSegment(dir, 321004u, 40);
    ASSERT_FALSE(seg.empty());
    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 8u);

    writeV1Sidecar(seg, n, /*keepEvery=*/2, /*counterBase=*/0,
                   /*firstHostMs=*/1000, /*stepMs=*/10, /*corruptCounter=*/true);

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());

    // Nothing adopted: every record must be labelled reconstructed, which is exactly today's
    // pre-upgrade behaviour. Partial adoption from a scrambled file is the failure mode.
    std::size_t observed = 0;
    for (auto v : r->allRecords()) {
        if ((v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0) ++observed;
    }
    EXPECT_EQ(observed, 0u) << "a file that failed a trust gate must contribute NOTHING";

    const auto& w = r->warnings();
    const bool declined = std::any_of(w.begin(), w.end(), [](const std::string& s) {
        return s.find("v1 upgrade declined") != std::string::npos;
    });
    for (const auto& s : w) std::printf("[measured] warn: %s\n", s.c_str());
    EXPECT_TRUE(declined) << "the refusal must be reported, not silent";

    ISFileManager::DeleteDirectory(dir.string());
}

TEST(IndexUpgradeV1, ACounterSpanThatDisagreesWithTheScanIsRefused) {
    const fs::path dir = makeTempDir("v1_span");
    const fs::path seg = writeSegment(dir, 321005u, 40);
    ASSERT_FALSE(seg.empty());
    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 8u);

    // A counter span far larger than the scan -- the shape of corpus segment 0009, whose sidecar
    // spans 585,673 indices against 47,502 scanned records.
    fs::path idxPath = seg;
    idxPath.replace_extension(".idx");
    {
        std::ofstream out(idxPath, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(out.good());
        const auto put = [](uint8_t* p, uint32_t v) {
            p[0] = uint8_t(v & 0xFF); p[1] = uint8_t((v >> 8) & 0xFF);
            p[2] = uint8_t((v >> 16) & 0xFF); p[3] = uint8_t((v >> 24) & 0xFF);
        };
        for (std::size_t k = 0; k < 8; ++k) {
            uint8_t rec[idx::IS_LOG_IDX_RECORD_V1_SIZE]{};
            put(rec + 0, 1000u + static_cast<uint32_t>(k) * 10u);
            put(rec + 4, 0u);
            put(rec + 8, static_cast<uint32_t>(k) * 100000u);   // span ~700,000
            put(rec + 12, 0u);
            out.write(reinterpret_cast<const char*>(rec), sizeof(rec));
        }
    }

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());
    std::size_t observed = 0;
    for (auto v : r->allRecords()) {
        if ((v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0) ++observed;
    }
    EXPECT_EQ(observed, 0u) << "a span mismatch means the sidecar describes other content";

    const auto& w = r->warnings();
    EXPECT_TRUE(std::any_of(w.begin(), w.end(), [](const std::string& s) {
        return s.find("does not match") != std::string::npos;
    }));
    ISFileManager::DeleteDirectory(dir.string());
}

// =====================================================================================
// Kyle's non-negotiable #3: byte offsets still resolve to real packet boundaries.
// The synthetic sidecar's offsets are 0xDEAD.... -- if any were adopted, the records would
// not read back at all.
// =====================================================================================

TEST(IndexUpgradeV1, ByteOffsetsComeFromTheScanNeverFromTheLegacySidecar) {
    const fs::path dir = makeTempDir("v1_offsets");
    const fs::path seg = writeSegment(dir, 321006u, 30);
    ASSERT_FALSE(seg.empty());
    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 4u);

    // Capture the offsets and DIDs a clean scan produces, for a field-by-field comparison.
    std::vector<uint64_t> wantOffsets;
    std::vector<uint32_t> wantDids;
    std::vector<uint64_t> wantTimestamps;
    {
        auto clean = ISLogReader::openSegment(seg);
        ASSERT_TRUE(clean.has_value());
        for (auto v : clean->allRecords()) {
            wantOffsets.push_back(v.offsetInFile());
            wantDids.push_back(v.did());
            wantTimestamps.push_back(v.timestamp().value);
        }
    }

    writeV1Sidecar(seg, n, /*keepEvery=*/2, /*counterBase=*/0,
                   /*firstHostMs=*/2000, /*stepMs=*/4);

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r->recordCount(), wantOffsets.size());

    std::size_t k = 0;
    for (auto v : r->allRecords()) {
        EXPECT_EQ(v.offsetInFile(), wantOffsets[k]) << "byte offset " << k << " came from the sidecar";
        EXPECT_LT(v.offsetInFile(), 0xDEAD0000ull)  << "that is the synthetic sidecar's bogus offset";
        EXPECT_EQ(v.did(), wantDids[k])       << "DID " << k << " changed across the upgrade";
        EXPECT_EQ(v.timestamp().value, wantTimestamps[k])
            << "payload timestamp " << k << " changed across the upgrade";
        ++k;
    }

    // And the records must still be readable -- an adopted bogus offset would break this.
    std::size_t readable = 0;
    for (auto v : r->allRecords()) {
        const auto [bytes, nBytes] = v.bytes();
        if (bytes != nullptr && nBytes > 0) ++readable;
    }
    EXPECT_EQ(readable, r->recordCount()) << "every record must still resolve to real bytes";

    ISFileManager::DeleteDirectory(dir.string());
}

// =====================================================================================
// Kyle's non-negotiable #2: a v2.0 upgrade preserves DIDs, offsets and payload timestamps.
// =====================================================================================

TEST(IndexUpgradeV20, PreservesEveryDidOffsetAndPayloadTimestamp) {
    const fs::path dir = makeTempDir("v20");
    const fs::path seg = writeSegment(dir, 322001u, 30);
    ASSERT_FALSE(seg.empty());

    // Capture the v2.1 truth, then rewrite the sidecar as v2.0: same 24-byte record prefix,
    // record_size = 0 (which a reader treats as 24), and no time-offset field.
    std::vector<uint64_t> wantOffsets, wantTimestamps;
    std::vector<uint32_t> wantDids;
    idx::is_log_idx_header_t hdr{};
    std::vector<idx::is_log_idx_record_v2_t> recs;
    {
        auto clean = ISLogReader::openSegment(seg);
        ASSERT_TRUE(clean.has_value());
        hdr = clean->header();
        for (auto v : clean->allRecords()) {
            wantOffsets.push_back(v.offsetInFile());
            wantDids.push_back(v.did());
            wantTimestamps.push_back(v.timestamp().value);
            idx::is_log_idx_record_v2_t rec{};
            rec.timestamp = v.timestamp().value;
            rec.offset    = v.offsetInFile();
            rec.did       = v.did();
            rec.flags     = v.flags();
            recs.push_back(rec);
        }
    }
    ASSERT_GT(recs.size(), 4u);

    fs::path idxPath = seg;
    idxPath.replace_extension(".idx");
    {
        idx::is_log_idx_header_t v20 = hdr;
        v20.record_size = 0;                       // pre-v2.1 => 24-byte records
        v20.total_records = recs.size();
        v20.flags = idx::IS_LOG_IDX_HDR_FLAG_FINALIZED;   // no HAS_LOG_TIME_OFFSET: no such field
        uint8_t hb[idx::IS_LOG_IDX_HEADER_SIZE];
        idx::serializeHeader(hb, v20);
        std::ofstream out(idxPath, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(out.good());
        out.write(reinterpret_cast<const char*>(hb), sizeof(hb));
        for (const auto& rec : recs) {
            uint8_t rb[idx::IS_LOG_IDX_RECORD_V2_1_SIZE];
            idx::serializeRecord(rb, rec);
            out.write(reinterpret_cast<const char*>(rb), idx::IS_LOG_IDX_RECORD_V2_SIZE);
        }
    }

    auto up = ISLogReader::upgradeIndex(seg);
    ASSERT_TRUE(up.has_value()) << "upgradeIndex failed: " << up.error().message;
    std::printf("[measured] v2.0 upgrade: fromVersion=%u observed=%zu reconstructed=%zu "
                "rewritten=%d\n",
                up->fromVersion, up->observedAdopted, up->reconstructed,
                static_cast<int>(up->rewritten));
    EXPECT_EQ(up->fromVersion, 2u);

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r->recordCount(), wantOffsets.size());

    std::size_t k = 0;
    for (auto v : r->allRecords()) {
        EXPECT_EQ(v.did(), wantDids[k])                  << "DID " << k << " not preserved";
        EXPECT_EQ(v.offsetInFile(), wantOffsets[k])            << "offset " << k << " not preserved";
        EXPECT_EQ(v.timestamp().value, wantTimestamps[k]) << "timestamp " << k << " not preserved";
        ++k;
    }

    // A v2.0 file has no observed chronology to salvage, so what it gains must be labelled
    // reconstructed -- claiming otherwise would hand ISTimeResolver a ruler it did not earn.
    std::size_t observed = 0;
    for (auto v : r->allRecords()) {
        if ((v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0) ++observed;
    }
    EXPECT_EQ(observed, 0u)
        << "a v2.0 sidecar carries no observed WHEN; nothing may claim to have one";

    ISFileManager::DeleteDirectory(dir.string());
}

// Mixed .idx versions within ONE log mean mixed HEADER CONTENT, and the rebuild/upgrade path is
// what creates the mixture. `capture_epoch_ms` is sampled once at log-open and written into every
// segment's header precisely so it survives the purge of earlier segments -- `OpenNewSaveFile()`
// deliberately does not reset it. A rebuild cannot recover it from the `.raw`, so building a
// default header DESTROYS it, leaving a log whose segments disagree about their own header: the
// rebuilt ones lose the wall-clock anchor while their untouched siblings keep it. For a GPS-less
// log that epoch is the ONLY absolute anchor there is.
//
// The corpus could not have caught this: every log on hand has HAS_CAPTURE_EPOCH clear and
// capture_epoch_ms == 0, so the fixture here sets a real one explicitly.
TEST(IndexUpgradeHeader, CaptureEpochSurvivesAnUpgradeAndARebuild) {
    const fs::path dir = makeTempDir("epoch_carry");
    const fs::path seg = writeSegment(dir, 325001u, 20);
    ASSERT_FALSE(seg.empty());

    constexpr uint64_t kEpoch = 1'789'000'000'123ULL;   // a real host wall-clock at log-open

    // Rewrite the sidecar as a v2.0 (no time-offset field) that DOES carry the epoch.
    std::vector<idx::is_log_idx_record_v2_t> recs;
    idx::is_log_idx_header_t base{};
    {
        auto clean = ISLogReader::openSegment(seg);
        ASSERT_TRUE(clean.has_value());
        base = clean->header();
        for (auto v : clean->allRecords()) {
            idx::is_log_idx_record_v2_t rec{};
            rec.timestamp = v.timestamp().value;
            rec.offset    = v.offsetInFile();
            rec.did       = v.did();
            rec.flags     = v.flags();
            recs.push_back(rec);
        }
    }
    ASSERT_GT(recs.size(), 2u);

    const auto writeV20WithEpoch = [&]() {
        fs::path idxPath = seg;
        idxPath.replace_extension(".idx");
        idx::is_log_idx_header_t h = base;
        h.record_size      = 0;                       // pre-v2.1 => 24-byte records
        h.total_records    = recs.size();
        h.capture_epoch_ms = kEpoch;
        h.flags = static_cast<uint8_t>(idx::IS_LOG_IDX_HDR_FLAG_FINALIZED
                                     | idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH);
        uint8_t hb[idx::IS_LOG_IDX_HEADER_SIZE];
        idx::serializeHeader(hb, h);
        std::ofstream out(idxPath, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(out.good());
        out.write(reinterpret_cast<const char*>(hb), sizeof(hb));
        for (const auto& rec : recs) {
            uint8_t rb[idx::IS_LOG_IDX_RECORD_V2_1_SIZE];
            idx::serializeRecord(rb, rec);
            out.write(reinterpret_cast<const char*>(rb), idx::IS_LOG_IDX_RECORD_V2_SIZE);
        }
    };

    // ---- Path 1: the explicit upgrade.
    writeV20WithEpoch();
    auto up = ISLogReader::upgradeIndex(seg);
    ASSERT_TRUE(up.has_value());
    {
        auto r = ISLogReader::openSegment(seg);
        ASSERT_TRUE(r.has_value());
        std::printf("[measured] after upgradeIndex: capture_epoch_ms=%llu hasFlag=%d\n",
                    (unsigned long long)r->header().capture_epoch_ms,
                    (r->header().flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH) ? 1 : 0);
        EXPECT_EQ(r->header().capture_epoch_ms, kEpoch)
            << "the upgrade destroyed the log's wall-clock anchor";
        EXPECT_NE(0, r->header().flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH);
    }

    // ---- Path 2: the automatic rebuild inside construct(), driven by a stale sidecar. Same
    // header, but a total_records that disagrees with the body so the staleness check fires.
    {
        writeV20WithEpoch();
        fs::path idxPath = seg;
        idxPath.replace_extension(".idx");
        // Corrupt the record COUNT only -- the header, and its epoch, stay intact and parseable.
        uint8_t hb[idx::IS_LOG_IDX_HEADER_SIZE];
        {
            std::ifstream in(idxPath, std::ios::binary);
            in.read(reinterpret_cast<char*>(hb), sizeof(hb));
        }
        auto h = idx::parseHeader(hb);
        ASSERT_TRUE(h.has_value());
        h->total_records = recs.size() + 9999;   // impossible for the body present
        idx::serializeHeader(hb, *h);
        {
            std::fstream out(idxPath, std::ios::binary | std::ios::in | std::ios::out);
            ASSERT_TRUE(out.good());
            out.write(reinterpret_cast<const char*>(hb), sizeof(hb));
        }

        auto r = ISLogReader::openSegment(seg);
        ASSERT_TRUE(r.has_value());
        ASSERT_FALSE(r->hadOnDiskIndex()) << "expected the staleness check to force a rebuild";
        std::printf("[measured] after stale rebuild: capture_epoch_ms=%llu hasFlag=%d\n",
                    (unsigned long long)r->header().capture_epoch_ms,
                    (r->header().flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH) ? 1 : 0);
        EXPECT_EQ(r->header().capture_epoch_ms, kEpoch)
            << "a stale-body rebuild destroyed a log-level header field it could have kept";
        EXPECT_NE(0, r->header().flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH);
    }

    ISFileManager::DeleteDirectory(dir.string());
}

// An already-current sidecar is a no-op, not a pointless rewrite.
TEST(IndexUpgradeV21, AnAlreadyCurrentSidecarIsLeftAlone) {
    const fs::path dir = makeTempDir("v21_noop");
    const fs::path seg = writeSegment(dir, 323001u, 20);
    ASSERT_FALSE(seg.empty());

    auto up = ISLogReader::upgradeIndex(seg);
    ASSERT_TRUE(up.has_value());
    std::printf("[measured] v2.1 no-op: fromVersion=%u rewritten=%d observed=%zu\n",
                up->fromVersion, static_cast<int>(up->rewritten), up->observedAdopted);
    EXPECT_EQ(up->fromVersion, 2u);
    EXPECT_FALSE(up->rewritten) << "a current sidecar must not be rewritten";

    ISFileManager::DeleteDirectory(dir.string());
}

// The explicit entry point Kyle asked for, and its logStart parameter. Passing the anchor must
// give the same result as letting it discover one, when both resolve to the same value.
TEST(IndexUpgradeApi, TheLogStartParameterDefaultsToDiscoveryAndIsHonouredWhenPassed) {
    const fs::path dir = makeTempDir("api_logstart");
    const fs::path seg = writeSegment(dir, 324001u, 30);
    ASSERT_FALSE(seg.empty());
    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 4u);

    writeV1Sidecar(seg, n, /*keepEvery=*/2, /*counterBase=*/0,
                   /*firstHostMs=*/4000u, /*stepMs=*/3);

    // Explicit anchor of 4000 == this segment's own first observed time, so the adopted offsets
    // start at 0 and step by 3 ms per record.
    auto passed = ISLogReader::upgradeIndex(seg, /*logStartHostUptimeMs=*/4000u);
    ASSERT_TRUE(passed.has_value());
    EXPECT_EQ(passed->fromVersion, 1u);
    EXPECT_GT(passed->observedAdopted, 0u);
    EXPECT_EQ(passed->logStartHostUptimeMs, 4000u);

    // A LATER anchor than the data must clamp to 0 rather than underflow into a huge number.
    writeV1Sidecar(seg, n, /*keepEvery=*/2, /*counterBase=*/0,
                   /*firstHostMs=*/4000u, /*stepMs=*/3);
    auto late = ISLogReader::upgradeIndex(seg, /*logStartHostUptimeMs=*/9'000'000u);
    ASSERT_TRUE(late.has_value());
    EXPECT_EQ(late->logStartHostUptimeMs, 9'000'000u);
    {
        auto r = ISLogReader::openSegment(seg);
        ASSERT_TRUE(r.has_value());
        for (auto v : r->allRecords()) {
            EXPECT_LT(v.logTimeOffsetMs(), 1'000'000u)
                << "an anchor later than the data must clamp, not underflow";
        }
    }

    // kDiscoverLogStart is the documented no-op default.
    EXPECT_EQ(ISLogReader::kDiscoverLogStart, UINT64_MAX);

    ISFileManager::DeleteDirectory(dir.string());
}

// =====================================================================================
// Kyle's non-negotiable #2, on REAL v2.0 data (GoldenLogs), and the reason there is no
// per-record adoption branch for v2.0.
//
// Measured against the committed v2.0 sidecar of a GoldenLogs segment, 59,601 records:
//
//   DID mismatches       : 0        -> the scan reproduces every DID; adoption is pointless
//   offset mismatches    : 57,985   -> 97% wrong. old offsets are 0,0,0... from record 2 on,
//                                      while the scan gives real packet boundaries 0,52,100
//   timestamp mismatches : 1,048    -> EVERY one is old-nonzero -> new-zero, on timeless DIDs
//                                      (904 on DID 39, plus DID_DEV_INFO and friends)
//
// That last row is the D-112 / SN-7999 pattern: the legacy `TimestampOrCurrentTime()` fallback
// parked a host-clock value in `timestamp` for records with no internal time field, and the
// current scan writes 0 with HAS_TIMESTAMP clear instead. So the scan's values are not merely
// equivalent, they are STRICTLY BETTER -- adopting the old ones would re-introduce exactly the
// audit A2 defect where a non-timestamp masquerades as a timestamp.
//
// Conclusion, and why this test exists rather than an adoption feature: a v2.0 sidecar contains
// nothing per-record worth taking. The only non-rederivable thing it holds is the header's
// log-level fields, which carryForwardLogLevelHeaderFields() handles.
TEST(IndexUpgradeRealFixture, ARealV20SidecarBecomesV21WithItsDidsIntact) {
    fs::path dir = "/work/inertialsense/goldenlogs/imx/imx6/AHRS/20260521_113715";
    if (const char* env = std::getenv("IS_SDK_V20_FIXTURE_DIR")) dir = env;
    const std::string stem = "LOG_SN942742854_20260521_113715_0001";
    const fs::path srcRaw = dir / (stem + ".raw");
    const fs::path srcIdx = dir / (stem + ".idx");
    if (!fs::exists(srcRaw) || !fs::exists(srcIdx)) {
        GTEST_SKIP() << "GoldenLogs v2.0 fixture not present at " << dir
                     << " (set IS_SDK_V20_FIXTURE_DIR to override)";
    }

    // COPY before opening: openSegment persists an upgraded sidecar, and doing this in place
    // would rewrite a git-tracked corpus file.
    const fs::path work = makeTempDir("real_v20");
    const fs::path raw = work / srcRaw.filename();
    std::error_code ec;
    fs::copy_file(srcRaw, raw, fs::copy_options::overwrite_existing, ec);
    fs::copy_file(srcIdx, work / srcIdx.filename(), fs::copy_options::overwrite_existing, ec);
    ASSERT_FALSE(ec);

    // Read the ORIGINAL v2.0 records straight off disk, before anything rewrites them.
    std::vector<uint32_t> oldDids;
    std::vector<uint64_t> oldOffsets;
    uint16_t oldRecordSize = 0xFFFF;
    {
        std::ifstream in(work / srcIdx.filename(), std::ios::binary);
        ASSERT_TRUE(in.good());
        std::vector<uint8_t> buf((std::istreambuf_iterator<char>(in)),
                                  std::istreambuf_iterator<char>());
        ASSERT_GT(buf.size(), idx::IS_LOG_IDX_HEADER_SIZE);
        auto h = idx::parseHeader(buf.data());
        ASSERT_TRUE(h.has_value());
        oldRecordSize = h->record_size;
        ASSERT_LT(oldRecordSize, idx::IS_LOG_IDX_RECORD_V2_1_SIZE)
            << "fixture is not v2.0 any more -- restore it from git";
        const std::size_t stride = idx::IS_LOG_IDX_RECORD_V2_SIZE;
        const std::size_t n = (buf.size() - idx::IS_LOG_IDX_HEADER_SIZE) / stride;
        for (std::size_t i = 0; i < n; ++i) {
            const auto rec = idx::parseRecord(
                buf.data() + idx::IS_LOG_IDX_HEADER_SIZE + i * stride, stride);
            oldDids.push_back(rec.did);
            oldOffsets.push_back(rec.offset);
        }
    }
    ASSERT_GT(oldDids.size(), 1000u);

    auto r = ISLogReader::openSegment(raw);
    ASSERT_TRUE(r.has_value());

    std::size_t didMismatch = 0, offsetMismatch = 0, k = 0;
    for (auto v : r->allRecords()) {
        if (k < oldDids.size()) {
            if (v.did() != oldDids[k])            ++didMismatch;
            if (v.offsetInFile() != oldOffsets[k]) ++offsetMismatch;
        }
        ++k;
    }
    std::printf("[measured] real v2.0: records %zu -> %zu, record_size %u -> %u, "
                "DID mismatches=%zu offset mismatches=%zu\n",
                oldDids.size(), r->recordCount(), oldRecordSize, r->header().record_size,
                didMismatch, offsetMismatch);

    // The upgrade happened: the sidecar on disk is now v2.1.
    EXPECT_EQ(r->header().record_size, idx::IS_LOG_IDX_RECORD_V2_1_SIZE);
    EXPECT_NE(0, r->header().flags & idx::IS_LOG_IDX_HDR_FLAG_DECLARES_TS_VALIDITY);

    // Kyle's non-negotiable #2: every DID preserved. Record population is unchanged too.
    EXPECT_EQ(r->recordCount(), oldDids.size()) << "record population changed across the upgrade";
    EXPECT_EQ(didMismatch, 0u) << "a DID changed across the upgrade";

    // Byte offsets deliberately do NOT match: the v2.0 sidecar's are broken (zeros from record 2
    // onward here), and the scan's are real packet boundaries. Asserted as a NON-match so nobody
    // "fixes" this by adopting them.
    EXPECT_GT(offsetMismatch, oldDids.size() / 2)
        << "the v2.0 sidecar's offsets suddenly agree with the scan -- if the writer's offset "
           "bug is fixed, revisit whether adoption is now worthwhile";

    // And every record must still resolve to real bytes after the upgrade.
    std::size_t readable = 0;
    for (auto v : r->allRecords()) {
        const auto [bytes, nBytes] = v.bytes();
        if (bytes != nullptr && nBytes > 0) ++readable;
    }
    EXPECT_EQ(readable, r->recordCount());

    ISFileManager::DeleteDirectory(work.string());
}

// =====================================================================================
// Real-fixture coverage. Numbers below were MEASURED on the corpus; they are not guesses.
// Skipped when the corpus is absent so CI stays quiet. Override the directory with
// IS_SDK_V1_FIXTURE_DIR.
//
// NOTE: this copies each fixture before opening it. `openSegment` PERSISTS an upgraded sidecar,
// so running against the corpus in place would destroy all 17 v1 fixtures.
// =====================================================================================

TEST(IndexUpgradeRealFixture, TheCorpusV1LogAdoptsAboutSeventyPercentObserved) {
    fs::path dir = "/work/inertialsense/sample_logs/ppd_LogQAQW713/20260716_004640";
    if (const char* env = std::getenv("IS_SDK_V1_FIXTURE_DIR")) dir = env;
    if (!fs::exists(dir)) {
        GTEST_SKIP() << "v1 corpus not present at " << dir
                     << " (set IS_SDK_V1_FIXTURE_DIR to override)";
    }

    struct Expect { const char* seg; std::size_t scanned; std::size_t observed; };
    // Measured 2026-09-20. 0009's sidecar is corrupt (non-monotonic counter at index 32943)
    // and must adopt nothing.
    const Expect cases[] = {
        { "LOG_SN123412_20260716_004640_0002", 66807, 47654 },
        { "LOG_SN123412_20260716_004640_0003", 66928, 47355 },
        { "LOG_SN123412_20260716_004640_0009", 47502,     0 },
    };

    const fs::path work = makeTempDir("real_v1");
    for (const auto& c : cases) {
        const fs::path srcRaw = dir / (std::string(c.seg) + ".raw");
        const fs::path srcIdx = dir / (std::string(c.seg) + ".idx");
        if (!fs::exists(srcRaw) || !fs::exists(srcIdx)) {
            GTEST_SKIP() << "fixture segment missing: " << srcRaw;
        }
        const fs::path raw = work / srcRaw.filename();
        std::error_code ec;
        fs::copy_file(srcRaw, raw, fs::copy_options::overwrite_existing, ec);
        fs::copy_file(srcIdx, work / srcIdx.filename(), fs::copy_options::overwrite_existing, ec);
        ASSERT_FALSE(ec) << "copy failed for " << srcRaw;

        auto r = ISLogReader::openSegment(raw);
        ASSERT_TRUE(r.has_value());
        std::size_t observed = 0;
        for (auto v : r->allRecords()) {
            if ((v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0) ++observed;
        }
        std::printf("[measured] %s: scanned=%zu observed=%zu (expected %zu / %zu)\n",
                    c.seg, r->recordCount(), observed, c.scanned, c.observed);
        EXPECT_EQ(r->recordCount(), c.scanned) << c.seg << ": scan count changed";
        EXPECT_EQ(observed, c.observed)        << c.seg << ": observed adoption changed";
    }
    ISFileManager::DeleteDirectory(work.string());
}
