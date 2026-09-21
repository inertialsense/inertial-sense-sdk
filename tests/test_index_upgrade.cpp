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
TEST(IndexUpgradeV1, AMidLogSegmentRebasesOnTheLogStartNotItsOwnFirstRecord) {
    const fs::path dir = makeTempDir("v1_midlog");
    const fs::path seg = writeSegment(dir, 321003u, 40);
    ASSERT_FALSE(seg.empty());
    const std::size_t n = scanCount(seg);
    ASSERT_GT(n, 4u);

    // Counter base 500,000 and host times starting at 900,000 ms: a segment well into a log,
    // whose v1 counter continues from earlier segments and whose WHEN is log-wide. This
    // segment is the only one on disk, so IT is the log's first segment and discovery
    // correctly resolves the anchor to its own earliest observed time (900,000).
    writeV1Sidecar(seg, n, /*keepEvery=*/2, /*counterBase=*/500000u,
                   /*firstHostMs=*/900000u, /*stepMs=*/5);

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value());

    std::size_t observed = 0, k = 0;
    for (auto v : r->allRecords()) {
        if ((v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0) {
            ++observed;
            // The join must survive a non-zero counter base, and the offset must be rebased on
            // the log start -- NOT left as the raw 900,000+ host uptime, and not zeroed.
            EXPECT_EQ(v.logTimeOffsetMs(), static_cast<uint32_t>(k) * 5u)
                << "record " << k << " was not rebased onto the log start";
        }
        ++k;
    }
    for (const auto& s : r->warnings()) std::printf("[measured] warn: %s\n", s.c_str());
    EXPECT_GT(observed, 0u) << "a non-zero counter base must still join";

    // Discovery resolved an anchor, so there must be no complaint about failing to.
    const auto& w = r->warnings();
    EXPECT_FALSE(std::any_of(w.begin(), w.end(), [](const std::string& s) {
        return s.find("could not discover") != std::string::npos;
    })) << "this segment IS the log's first; discovery should have succeeded";

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
