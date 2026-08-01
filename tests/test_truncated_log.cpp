/**
 * @file test_truncated_log.cpp
 * @brief D-04 / SN-7895 — truncated-tail + missing-index acceptance tests.
 *
 * Each test uses a fresh fixture under /tmp generated through the
 * cISLogger framework (same path D-02 uses). The fixture is then
 * mutated (truncated, .idx deleted, .idx hand-mangled, ...) and
 * re-opened via ISLogReader.
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // first — see test_log_reader.cpp comment

#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISLogIndex.h"
#include "ISLogReader.h"
#include "ISLogger.h"
#include "test_data_utils.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 999111u;

struct Fixture {
    fs::path                          directory;
    fs::path                          rawFile;
    fs::path                          idxFile;
    std::list<std::vector<uint8_t>*>  messages;
    std::size_t                       rawBytes = 0;
};

Fixture buildFixture(const std::string& hint, float sizeMB = 1.0f) {
    Fixture f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_truncated_log_%s_%d_%ld",
                  hint.c_str(), ::getpid(),
                  static_cast<long>(::time(nullptr)));
    f.directory = dirBuf;
    ISFileManager::DeleteDirectory(f.directory.string());

    GenerateRawLogData(f.messages, sizeMB);
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
            logger.LogData(devLogger, msg->size(),
                           reinterpret_cast<const uint8_t*>(msg->data()));
        }
        logger.CloseAllFiles();
    }

    std::vector<ISFileManager::file_info_t> rawFiles, idxFiles;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true,
                                          "\\.raw$", rawFiles);
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true,
                                          "\\.idx$", idxFiles);
    if (rawFiles.empty()) return f;
    f.rawFile  = rawFiles.front().name;
    if (!idxFiles.empty()) f.idxFile = idxFiles.front().name;
    f.rawBytes = static_cast<std::size_t>(rawFiles.front().size);
    return f;
}

void teardown(Fixture& f) {
    for (auto* m : f.messages) delete m;
    f.messages.clear();
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

class TruncatedLogTest : public ::testing::Test {
protected:
    void TearDown() override { teardown(f_); }
    Fixture f_;
};

} // namespace

// ============================================================
// Truncation: chop the .raw mid-packet, verify reader stops cleanly
// ============================================================

TEST_F(TruncatedLogTest, MidPacketTruncation_StopsAtLastValidRecord) {
    f_ = buildFixture("midpkt");
    ASSERT_FALSE(f_.rawFile.empty());

    // Read the original record count from the .idx (D-01 wrote it).
    auto fullR = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(fullR.has_value());
    const std::size_t fullCount = fullR->recordCount();
    ASSERT_GT(fullCount, 100u) << "fixture too small to test truncation meaningfully";
    // Drop the reader to release its mmap before mutating the file.
    fullR = tl::unexpected<ISError>{ ISError{ ISErrorCode::Internal, "drop" } };

    // Truncate the .raw at 90% of its size to land somewhere mid-packet.
    const std::size_t truncatedSize = (f_.rawBytes * 9) / 10;
    fs::resize_file(f_.rawFile, truncatedSize);
    // Delete the .idx so the reader rebuilds and exercises the
    // truncation-detection path (the existing .idx still claims the
    // full count and would mask the truncation in this test).
    ASSERT_FALSE(f_.idxFile.empty());   // guard: fs::remove("") is implementation-defined
    fs::remove(f_.idxFile);

    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value()) << "open should succeed even on truncation: "
                               << r.error().message;

    EXPECT_TRUE(r->isTruncated())
        << "expected isTruncated() == true after mid-packet chop";
    EXPECT_LT(r->recordCount(), fullCount)
        << "truncated read should produce fewer records";
    EXPECT_GT(r->recordCount(), 0u)
        << "should still see records that completed before the cut";
    EXPECT_LE(r->truncationOffset(), truncatedSize);
    EXPECT_EQ(r->fileSize(), truncatedSize);

    // warnings() should mention the truncation.
    bool foundTruncationWarning = false;
    for (const auto& w : r->warnings()) {
        if (w.find("truncation") != std::string::npos) {
            foundTruncationWarning = true;
            break;
        }
    }
    EXPECT_TRUE(foundTruncationWarning)
        << "warnings() should carry a truncation notice";
}

// ============================================================
// Missing .idx: scan-rebuild + persist
// ============================================================

TEST_F(TruncatedLogTest, MissingIdx_RebuildsAndPersists) {
    f_ = buildFixture("missing");
    ASSERT_FALSE(f_.rawFile.empty());
    ASSERT_FALSE(f_.idxFile.empty());   // buildFixture only sets idxFile when an .idx was found
    ASSERT_TRUE(fs::exists(f_.idxFile));

    fs::remove(f_.idxFile);
    ASSERT_FALSE(fs::exists(f_.idxFile));

    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(r->hadOnDiskIndex());
    EXPECT_GT(r->recordCount(), 0u);
    EXPECT_FALSE(r->isTruncated());

    // The persist path should have written a fresh sidecar back.
    EXPECT_TRUE(fs::exists(f_.idxFile))
        << "scan-rebuild should have persisted a v2 .idx next to the .raw";

    // Re-opening with the persisted sidecar should use it directly.
    const auto persistedMtime = fs::last_write_time(f_.idxFile);
    auto r2 = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r2.has_value());
    EXPECT_TRUE(r2->hadOnDiskIndex())
        << "second open should consume the persisted sidecar, not rebuild";
    EXPECT_EQ(r2->recordCount(), r->recordCount());
    EXPECT_EQ(fs::last_write_time(f_.idxFile), persistedMtime)
        << "second open should not re-touch the .idx";

    bool sawRebuildWarning = false;
    for (const auto& w : r->warnings()) {
        if (w.find("sidecar: rebuilt") != std::string::npos) {
            sawRebuildWarning = true;
            break;
        }
    }
    EXPECT_TRUE(sawRebuildWarning)
        << "first open should report a rebuild in warnings()";
}

// ============================================================
// Stale .idx: hand-mangle total_records → trigger rebuild
// ============================================================

TEST_F(TruncatedLogTest, StaleIdx_TriggersRebuild) {
    f_ = buildFixture("stale");
    ASSERT_FALSE(f_.rawFile.empty());

    // Hand-mangle the .idx header's total_records field. v2 layout
    // puts total_records at offset 12 (8 bytes, little-endian).
    {
        std::fstream f(f_.idxFile, std::ios::in | std::ios::out | std::ios::binary);
        ASSERT_TRUE(f) << "cannot reopen .idx for mutation";
        const uint64_t bogus = 0xDEADBEEF;
        f.seekp(12);
        f.write(reinterpret_cast<const char*>(&bogus), sizeof(bogus));
        ASSERT_TRUE(f) << "could not write to .idx";
    }

    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(r->hadOnDiskIndex())
        << "stale sidecar should be ignored and rebuilt";
    EXPECT_GT(r->recordCount(), 0u);

    bool sawStale = false;
    for (const auto& w : r->warnings()) {
        if (w.find("stale") != std::string::npos) {
            sawStale = true;
            break;
        }
    }
    EXPECT_TRUE(sawStale) << "warnings() should explicitly cite stale-sidecar";
}

// ============================================================
// Clean log with on-disk index: NOT truncated, no warnings
// ============================================================

TEST_F(TruncatedLogTest, CleanFixture_NoTruncationNoWarnings) {
    f_ = buildFixture("clean");
    ASSERT_FALSE(f_.rawFile.empty());

    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    // SN-8328: a clean (non-truncated) log must never be flagged as truncated.
    // This ~1 MB cISLogger fixture spans several 128 KB chunks; the RAW writer
    // now stamps true physical .raw offsets, so the reader TRUSTS the sidecar
    // (no rebuild). The property under test is that this emits NO *truncation*
    // warning and the segment reads to its end.
    EXPECT_TRUE(r->hadOnDiskIndex());
    EXPECT_FALSE(r->isTruncated());
    EXPECT_EQ(r->truncationOffset(), r->fileSize());
    for (const auto& w : r->warnings()) {
        EXPECT_EQ(w.find("truncation"), std::string::npos)
            << "clean log emitted a truncation warning: " << w;
    }
}

// ============================================================
// Atomic-write: no .idx.tmp leftover after a successful persist
// ============================================================

TEST_F(TruncatedLogTest, RebuildLeavesNoTmpFile) {
    f_ = buildFixture("atomic");
    ASSERT_FALSE(f_.rawFile.empty());
    ASSERT_FALSE(f_.idxFile.empty());   // guard: fs::remove("") is implementation-defined

    fs::remove(f_.idxFile);
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());

    const fs::path tmp = f_.idxFile.string() + ".tmp";
    EXPECT_FALSE(fs::exists(tmp))
        << "atomic write should have renamed (or removed) the .tmp file";
}
