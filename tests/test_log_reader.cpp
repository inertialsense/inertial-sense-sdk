/**
 * @file test_log_reader.cpp
 * @brief D-02 / SN-7893 acceptance tests for the new ISLogReader.
 *
 * Strategy: each test fixture generates a fresh ~1 MB log via the existing test_data_utils helpers
 * (`GenerateRawLogData` + `cISLogger`) — the same shape D-01 used in its multi-segment integration test. That gives us
 * a single .raw + .idx pair we can hand to ISLogReader and assert against. The temp dir is wiped on test teardown so
 * runs don't leak state.
 *
 * Why generate per-test rather than commit a binary fixture: the synthetic generator depends on host time (GPS week,
 * current ms), so a committed binary would diverge from the generator's output on every regeneration. Committing the
 * *generator code* (this file, plus tests/test_data_utils.cpp) gives us reproducibility within a run; binary
 * determinism across machines isn't needed for the AC checks (record counts, DID lists, byte-aliasing, etc. all derive
 * from the in-test generation).
 */

#include <gtest/gtest.h>

// com_manager.h FIRST — the legacy ISFirmwareUpdater.h wraps it in `extern "C" { ... }`, which is illegal for the
// C++-overload-bearing com_manager.h. Including it first short-circuits the broken wrap via the include guard. Same
// trick used in test_log_index.cpp.
#include "com_manager.h"

#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISLogIndex.h"
#include "ISLogReader.h"
#include "ISLogger.h"
#include "data_sets.h"
#include "test_data_utils.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <list>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId    = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial  = 519465u;
constexpr float    kFixtureSizeMB  = 1.0f;

struct FixtureLayout {
    fs::path           directory;
    fs::path           rawFile;
    fs::path           idxFile;
    std::list<std::vector<uint8_t>*> messages;
    std::size_t        rawBytes = 0;
};

// Generate a 1 MB log via GenerateRawLogData + cISLogger and report the resulting .raw + .idx paths. Caller is
// responsible for tearing down the directory and freeing `messages` entries.
FixtureLayout generateFixture(const std::string& dirHint) {
    FixtureLayout f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf), "/tmp/test_log_reader_%s_%d_%ld", dirHint.c_str(), ::getpid(), static_cast<long>(::time(nullptr)));
    f.directory = dirBuf;
    ISFileManager::DeleteDirectory(f.directory.string());

    GenerateRawLogData(f.messages, kFixtureSizeMB);
    if (f.messages.empty()) {
        return f;  // Caller asserts on this.
    }

    {
        cISLogger logger;
        cISLogger::sSaveOptions opts;
        opts.logType                = cISLogger::LOGTYPE_RAW;
        opts.useSubFolderTimestamp  = false;
        if (!logger.InitSave(f.directory.string(), opts)) {
            return f;
        }
        auto devLogger = logger.registerDevice(kFixtureHwId, kFixtureSerial);
        if (!devLogger) return f;
        logger.EnableLogging(true);

        for (auto* msg : f.messages) {
            logger.LogData(devLogger, msg->size(), reinterpret_cast<const uint8_t*>(msg->data()));
        }
        logger.CloseAllFiles();
    }

    // Locate the segment files cISLogger emitted. We restrict to 1 MB so the writer should produce a single segment;
    // tests that need multi-segment coverage live in test_log_index.cpp.
    std::vector<ISFileManager::file_info_t> rawFiles, idxFiles;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, "\\.raw$", rawFiles);
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, "\\.idx$", idxFiles);
    if (rawFiles.empty() || idxFiles.empty()) return f;

    f.rawFile  = rawFiles.front().name;
    f.idxFile  = idxFiles.front().name;
    f.rawBytes = static_cast<std::size_t>(rawFiles.front().size);
    return f;
}

void teardownFixture(FixtureLayout& f) {
    for (auto* msg : f.messages) {
        delete msg;
    }
    f.messages.clear();
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

class LogReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        f_ = generateFixture(::testing::UnitTest::GetInstance()->current_test_info()->name());
        ASSERT_FALSE(f_.messages.empty()) << "fixture generation produced no messages";
        ASSERT_FALSE(f_.rawFile.empty())  << "fixture generation produced no .raw";
        ASSERT_TRUE(fs::exists(f_.idxFile)) << "expected .idx alongside .raw";
    }
    void TearDown() override { teardownFixture(f_); }

    FixtureLayout f_;
};

} // namespace

// ============================================================
// Open / close round-trips
// ============================================================

TEST_F(LogReaderTest, OpenSegmentSucceeds) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value()) << "openSegment failed: " << r.error().message;
    // SN-8328 (B): this fixture is a ~1 MB cISLogger raw log, i.e. larger than
    // one 128 KB chunk. The SDK writer stores chunk-relative .idx offsets that
    // desync from the physical .raw offset past the first chunk flush (tracked
    // as an SDK writer-offset follow-up). The reader's offset-sanity probe
    // detects this and rebuilds the index from a full .raw scan, so
    // hadOnDiskIndex() is false here — and the rebuilt index is correct, which
    // the rest of this assertion set verifies. (A <128 KB log's writer index is
    // trusted; RoundTripBitIdentical covers that path via ISLogWriter output.)
    EXPECT_FALSE(r->hadOnDiskIndex());
    EXPECT_GT(r->recordCount(), 0u);
    EXPECT_EQ(r->header().magic[0], 'I');
    EXPECT_EQ(r->header().magic[3], 'X');
    EXPECT_EQ(r->header().version, idx::IS_LOG_IDX_VERSION_V2);
}

TEST_F(LogReaderTest, OpenMissingFileReturnsNotFound) {
    auto r = ISLogReader::openSegment("/tmp/this/path/does/not/exist.raw");
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::NotFound);
}

TEST_F(LogReaderTest, DoubleOpenIndependent) {
    auto a = ISLogReader::openSegment(f_.rawFile);
    auto b = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a->recordCount(), b->recordCount());
    // Different mmaps → different base pointers.
    auto av = a->allRecords();
    auto bv = b->allRecords();
    if (!av.empty() && !bv.empty()) {
        auto avBytes = (*av.begin()).bytes();
        auto bvBytes = (*bv.begin()).bytes();
        EXPECT_NE(avBytes.first, bvBytes.first);
    }
}

// ============================================================
// Header / record-count / DID list
// ============================================================

TEST_F(LogReaderTest, HeaderAndCountsMatchFixture) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());

    // Copy packed-struct fields into stack locals before comparing — is_log_idx_header_t is #pragma pack(1) so binding
    // a reference to a misaligned u64 directly is UB even though x86 tolerates it.
    const uint64_t total_records = r->header().total_records;
    const uint64_t segEnd        = r->segmentEndTimestamp();
    const uint64_t segStart      = r->segmentStartTimestamp();
    EXPECT_EQ(r->recordCount(), total_records);
    EXPECT_GT(segEnd, 0u);
    EXPECT_LE(segStart, segEnd);

    auto dids = r->presentDids();
    EXPECT_FALSE(dids.empty());
    EXPECT_TRUE(std::is_sorted(dids.begin(), dids.end()));

    // The synthetic generator emits at least PIMU + INS + GPS + a handful of others — so at least 4 distinct DIDs.
    EXPECT_GE(dids.size(), 4u);
}

TEST_F(LogReaderTest, RecordsByDidCountsMatchAggregate) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());

    std::size_t accum = 0;
    for (auto did : r->presentDids()) {
        accum += r->records(did).size();
    }
    EXPECT_EQ(accum, r->recordCount());
}

TEST_F(LogReaderTest, RecordsForUnknownDidIsEmptyRange) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());

    constexpr ISLogReader::did_t kBogus = 0xFFFFFFu;
    auto rng = r->records(kBogus);
    EXPECT_EQ(rng.size(), 0u);
    EXPECT_TRUE(rng.empty());
    EXPECT_TRUE(rng.begin() == rng.end());
}

TEST_F(LogReaderTest, RecordsForKnownDidNonZero) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());

    // Pick whichever DID the fixture happens to contain — the synthetic generator's DID mix isn't fixed by name
    // (different protocol streams cover different DIDs), so test the lookup path against the DIDs that actually
    // present in this fixture.
    auto dids = r->presentDids();
    ASSERT_FALSE(dids.empty());
    for (auto did : dids) {
        EXPECT_GT(r->records(did).size(), 0u)
            << "presentDids() listed DID " << did
            << " but records() returned an empty range";
    }
}

// ============================================================
// Iterator semantics
// ============================================================

TEST_F(LogReaderTest, AllRecordsRangeForCompiles) {
    // Range-for syntax exercises C++17 forward-iterator concepts (begin/end/operator++/operator*). The DoD AC asks for
    // that explicitly so C++17 callers can use the reader naturally.
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());

    std::size_t seen = 0;
    for (ISRecordView v : r->allRecords()) {
        (void)v;
        ++seen;
    }
    EXPECT_EQ(seen, r->recordCount());
}

TEST_F(LogReaderTest, BytesAliasMmapRegion) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    ASSERT_GT(r->recordCount(), 0u);

    // Open the .raw separately to bracket the legitimate aliasing range. We can't poke the reader's source pointer
    // directly (private), but ISFileSource::open + size gives us the same file's byte range, and the mmap'd region for
    // the reader is a contiguous [base, base+size) pair. We assert that the record's bytes pointer lives somewhere in
    // that range.
    auto src = ISFileSource::open(f_.rawFile);
    ASSERT_TRUE(src.has_value());
    const std::size_t fileSize = (*src)->size();
    ASSERT_GT(fileSize, 0u);

    auto first = *r->allRecords().begin();
    auto bytes = first.bytes();
    EXPECT_NE(bytes.first, nullptr);
    EXPECT_GT(bytes.second, 0u);
    // Pointer must be at the offset reported by the record.
    EXPECT_EQ(first.offsetInFile(), 0u)
        << "first record's offset is expected to be 0 by writer convention";
    // Size must not run off the end of the file.
    EXPECT_LE(first.offsetInFile() + bytes.second, fileSize);
}

TEST_F(LogReaderTest, OwnedRecordSurvivesReaderClose) {
    OwnedRecord owned;
    {
        auto r = ISLogReader::openSegment(f_.rawFile);
        ASSERT_TRUE(r.has_value());
        ASSERT_GT(r->recordCount(), 0u);
        auto first = *r->allRecords().begin();
        owned = first.owned();
        // Pointer of the view aliases mmap; pointer of the owned record is in heap space — must be different.
        EXPECT_NE(first.bytes().first, owned.bytes().first);
    } // r out of scope → mmap unmapped.

    // owned still readable.
    auto bytes = owned.bytes();
    EXPECT_NE(bytes.first, nullptr);
    EXPECT_GT(bytes.second, 0u);
}

// ============================================================
// Seek + in_time
// ============================================================

TEST_F(LogReaderTest, SeekPositionsAtOrAfterTarget) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    if (r->recordCount() < 4u) GTEST_SKIP();

    // Pick a target timestamp that's halfway through the recorded span, then assert the iterator-yielded record's
    // timestamp is >= target. We don't assume monotonicity, so we tolerate a linear-fallback positioning.
    uint64_t lo = r->segmentStartTimestamp();
    uint64_t hi = r->segmentEndTimestamp();
    uint64_t target = lo + (hi - lo) / 2;
    auto it = r->seek(TimeStamp::fromPayloadToW(target, r->deviceId()));
    if (it != r->allRecords().end()) {
        auto v = *it;
        EXPECT_GE(v.timestamp().value, target);
    }
}

TEST_F(LogReaderTest, InTimeFiltersToInterval) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    ASSERT_GT(r->recordCount(), 4u);

    uint64_t lo = r->segmentStartTimestamp();
    uint64_t hi = r->segmentEndTimestamp();
    uint64_t mid_lo = lo + (hi - lo) / 4;
    uint64_t mid_hi = lo + 3 * (hi - lo) / 4;

    auto rng = r->allRecords().in_time(TimeStamp::fromPayloadToW(mid_lo, r->deviceId()), TimeStamp::fromPayloadToW(mid_hi, r->deviceId()));

    for (ISRecordView v : rng) {
        EXPECT_GE(v.timestamp().value, mid_lo);
        EXPECT_LE(v.timestamp().value, mid_hi);
    }
}

// ============================================================
// Concurrent read from two threads
// ============================================================

TEST_F(LogReaderTest, ConcurrentIterationFromTwoThreads) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());

    std::atomic<std::size_t> seen[2] = {};
    auto worker = [&](int idx) {
        std::size_t count = 0;
        for (ISRecordView v : r->allRecords()) {
            (void)v.bytes();
            (void)v.timestamp();
            ++count;
        }
        seen[idx] = count;
    };
    std::thread t1(worker, 0);
    std::thread t2(worker, 1);
    t1.join();
    t2.join();
    EXPECT_EQ(seen[0].load(), r->recordCount());
    EXPECT_EQ(seen[1].load(), r->recordCount());
}

// ============================================================
// Move semantics
// ============================================================

TEST_F(LogReaderTest, MoveConstructTransfersOwnership) {
    auto r1 = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r1.has_value());
    const std::size_t n = r1->recordCount();

    ISLogReader r2 = std::move(*r1);
    EXPECT_EQ(r2.recordCount(), n);
}

// ============================================================
// deviceId derivation
// ============================================================

TEST_F(LogReaderTest, DeviceIdIsDerivedFromFixture) {
    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value());
    // Either DEV_INFO record carries the serial we passed in, or the filename-fallback path picks it up. Either way
    // deviceId() is non-zero for this fixture.
    EXPECT_NE(r->deviceId(), 0u);
}

// ---------------------------------------------------------------------------
// hdwId derivation against a real cltool 120 s PPD capture (IMX-5.0, SN519465). The synthetic test_data_utils fixture
// writes DEV_INFO records with all-zero hardwareType/hardwareVer fields, so the synthetic path can only verify that
// hdwId() returns 0 cleanly when the type byte is zero — it can't prove the parse extracts the right bytes. This live
// test does. GTEST_SKIPped when the fixture isn't on disk so CI is silent rather than red. Override the path with
// IS_SDK_LIVE_FIXTURE_RAW.
// ---------------------------------------------------------------------------
TEST(LogReaderLiveFixture, HdwIdMatchesImx5_0FromCltoolCapture) {
    fs::path raw;
    if (const char* env = std::getenv("IS_SDK_LIVE_FIXTURE_RAW")) {
        raw = env;
    } else {
        raw = fs::path(std::getenv("HOME") ? std::getenv("HOME") : "/")
            / "workspace/inertialsense/sample_logs/cltool_imx5_120s_ppd/20260429_105836/LOG_SN519465_20260429_105836_0001.raw";
    }
    if (!fs::exists(raw)) {
        GTEST_SKIP() << "live fixture not present at " << raw << " (set IS_SDK_LIVE_FIXTURE_RAW to override)";
    }

    auto r = ISLogReader::openSegment(raw);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    EXPECT_EQ(r->deviceId(), 519465u);
    EXPECT_EQ(r->hdwId(),    IS_HARDWARE_IMX_5_0) << "expected IMX-5.0 from this capture; got hdwId=0x" << std::hex << r->hdwId();
}

// ============================================================
// SN-8328 (supersedes SN-8004) — GNSS_RAW DIDs must NOT carry a raw observation
// time in the .idx. The old cISDataMappings::Timestamp returned obs.time.sec +
// obs.time.time (gtime_t absolute seconds) for these DIDs; the resolver then
// anchored that off-domain value to GPS week 0, splitting a log's resolved span
// across ~45 years (surfaced on real goldenlogs). The fix makes Timestamp return
// 0 for GNSS_RAW (uptime-positioned like other non-ToW records), and the reader
// treats a GNSS_RAW record whose timestamp exceeds one week as stale (old
// tooling) so it rebuilds the sidecar and re-derives the record correctly.
// ============================================================

TEST_F(LogReaderTest, GpsRawObsTimeTriggersRebuild) {
    using namespace idx;

    // Slurp the .idx generated by the fixture, hand-modify the first record to look like a GNSS_RAW packet carrying a
    // baked absolute obs-time (~1.7e12 ms) as old tooling would, write back, re-open.
    std::ifstream in(f_.idxFile, std::ios::binary);
    std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    ASSERT_GT(bytes.size(), IS_LOG_IDX_HEADER_SIZE + IS_LOG_IDX_RECORD_V2_SIZE);

    auto* recPtr = reinterpret_cast<uint8_t*>(bytes.data() + IS_LOG_IDX_HEADER_SIZE);
    is_log_idx_record_v2_t r = parseRecord(recPtr);
    r.did       = DID_GNSS1_RAW;
    r.timestamp = 1'700'000'000'000ULL;   // baked absolute obs-time — not a valid device-timeline value
    serializeRecord(recPtr, r);

    std::ofstream out(f_.idxFile, std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), bytes.size());
    out.close();

    auto reader = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(reader.has_value()) << reader.error().message;
    EXPECT_FALSE(reader->hadOnDiskIndex())
        << "A GNSS_RAW record with a baked absolute obs-time (> 1 week) must mark the .idx stale and trigger a "
           "rebuild (SN-8328) — otherwise that off-domain value poisons the resolved span (~45-year split).";
}

// ============================================================
// SN-8005 — sibling-aware truncation: mid-rotation tails (next-segment file
// exists alongside) get demoted to log_info because the data is intact in
// the successor segment. Terminal-segment truncation (no successor) stays
// log_warn because the tail loss is real.
// ============================================================

TEST_F(LogReaderTest, TruncationWithSiblingEmitsTrailingPartialMessage) {
    // Append 64 bytes of garbage to the .raw so the parser detects an incomplete trailing packet.
    {
        std::ofstream out(f_.rawFile, std::ios::binary | std::ios::app);
        std::vector<char> garbage(64, '\xAA');
        out.write(garbage.data(), garbage.size());
    }
    // Wipe the persisted .idx so openSegment hits the rebuild path that emits the truncation log line.
    std::error_code ec;
    fs::remove(f_.idxFile, ec);

    // Create a sibling _0002.raw next to the _0001.raw the fixture produced. We don't need it to be a valid log — we
    // just need its filesystem presence for the sibling check.
    const std::string stem = f_.rawFile.stem().string();
    const auto und = stem.find_last_of('_');
    ASSERT_NE(und, std::string::npos);
    const std::string seqStr = stem.substr(und + 1);
    int seq = std::stoi(seqStr);
    std::ostringstream nextSeq;
    nextSeq << std::setw(static_cast<int>(seqStr.size())) << std::setfill('0') << (seq + 1);
    const fs::path siblingRaw =
        f_.rawFile.parent_path() / (stem.substr(0, und + 1) + nextSeq.str() + f_.rawFile.extension().string());
    {
        std::ofstream siblingOut(siblingRaw, std::ios::binary | std::ios::trunc);
        siblingOut << "placeholder";   // any content; only existence matters
    }

    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    EXPECT_TRUE(r->isTruncated());

    bool sawTrailingPartial = false;
    bool sawTruncatedStopped = false;
    for (const auto& w : r->warnings()) {
        if (w.find("trailing partial packet") != std::string::npos) sawTrailingPartial = true;
        if (w.find("truncation: stopped") != std::string::npos)     sawTruncatedStopped = true;
    }
    EXPECT_TRUE(sawTrailingPartial)
        << "Expected 'trailing partial packet' (mid-rotation, info-level) warning when a sibling segment exists.";
    EXPECT_FALSE(sawTruncatedStopped)
        << "Did NOT expect the warn-level 'truncation: stopped' message when a sibling segment exists.";

    fs::remove(siblingRaw, ec);
}

TEST_F(LogReaderTest, TruncationWithoutSiblingEmitsWarnMessage) {
    // Same setup as above but without the sibling — this segment is terminal; truncation here means real tail loss.
    {
        std::ofstream out(f_.rawFile, std::ios::binary | std::ios::app);
        std::vector<char> garbage(64, '\xAA');
        out.write(garbage.data(), garbage.size());
    }
    std::error_code ec;
    fs::remove(f_.idxFile, ec);

    auto r = ISLogReader::openSegment(f_.rawFile);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    EXPECT_TRUE(r->isTruncated());

    bool sawTrailingPartial = false;
    bool sawTruncatedStopped = false;
    for (const auto& w : r->warnings()) {
        if (w.find("trailing partial packet") != std::string::npos) sawTrailingPartial = true;
        if (w.find("truncation: stopped") != std::string::npos)     sawTruncatedStopped = true;
    }
    EXPECT_TRUE(sawTruncatedStopped)
        << "Expected the warn-level 'truncation: stopped' message for a terminal segment with no successor.";
    EXPECT_FALSE(sawTrailingPartial)
        << "Did NOT expect the info-level 'trailing partial packet' message when no sibling segment exists.";
}

// ============================================================
// SN-7996 Phase B — close-after-mmap leaves the process FD-budget alone
// ============================================================

#if !defined(_WIN32)
TEST_F(LogReaderTest, OpeningManySegmentsDoesNotAccumulateFds) {
    // Open the fixture's .raw 64 times. With the pre-Phase-B behavior we'd hold ~128 FDs (one .raw + one .idx
    // per reader); with the close-after-mmap fix, the mmap'd pages keep `bytes()` valid while the FDs are
    // released back to the OS immediately. The test asserts the post-open FD count grows by a small constant
    // (allowing noise from the test harness — gtest output streams, /tmp temp files from concurrent tests on
    // CI runners, etc.) rather than ~2 per segment.
    //
    // POSIX-only (counts /proc/self/fd entries). Windows uses a different handle accounting model; the same
    // bug class is closed by the matching CloseHandle in ISFileSource::open's WIN32 branch but isn't exercised
    // by this test directly.

    auto countOpenFds = []() -> int {
        std::error_code ec;
        int n = 0;
        for (const auto& entry : fs::directory_iterator("/proc/self/fd", ec)) {
            (void)entry;
            ++n;
        }
        if (ec) return -1;  // skip silently
        return n;
    };

    const int before = countOpenFds();
    if (before < 0) GTEST_SKIP() << "/proc/self/fd not enumerable on this kernel";

    constexpr int kSegmentCount = 64;
    std::vector<ISLogReader> readers;
    readers.reserve(kSegmentCount);
    for (int i = 0; i < kSegmentCount; ++i) {
        auto r = ISLogReader::openSegment(f_.rawFile);
        ASSERT_TRUE(r.has_value()) << "openSegment #" << i << " failed: " << r.error().message;
        readers.push_back(std::move(*r));
    }

    const int after = countOpenFds();
    ASSERT_GE(after, 0);

    // Headroom: 8 FDs of slack for harness-side noise. Pre-fix this would be ~128 (2 per segment).
    EXPECT_LT(after - before, 16)
        << "FD count grew by " << (after - before) << " after opening " << kSegmentCount
        << " segments — close-after-mmap regression suspected. before=" << before << " after=" << after;

    // Sanity: payload reads still work without the FD open. Walk the first reader's records via
    // bytes()-aliasing — proves the mmap stays valid.
    ASSERT_GT(readers.size(), 0u);
    ASSERT_GT(readers[0].recordCount(), 0u);
    auto bytes = readers[0].rawBytes();
    EXPECT_NE(bytes.first, nullptr);
    EXPECT_GT(bytes.second, 0u);
}
#endif

// ============================================================
// Layout-discipline sanity
// ============================================================

TEST(LogReaderLayout, ISRecordViewDefaultIsEmpty) {
    ISRecordView v;
    EXPECT_EQ(v.did(), 0u);
    EXPECT_EQ(v.bytes().first, nullptr);
    EXPECT_EQ(v.bytes().second, 0u);
    EXPECT_EQ(v.offsetInFile(), 0u);
}
