/**
 * @file test_log_writer.cpp
 * @brief D-08 / SN-7898 — ISLogWriter acceptance tests.
 *
 * Strategy mirrors test_log_reader / test_truncated_log: synthesize a
 * fresh fixture under /tmp via cISLogger, open it with ISLogReader,
 * pipe records through ISLogWriter, then re-open the writer's output
 * and compare. Byte-for-byte equality of every record's payload is the
 * single most important property — the writer must never repacketize.
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // first — see test_log_reader.cpp comment

#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISLogIndex.h"
#include "ISLogReader.h"
#include "ISLogWriter.h"
#include "ISLogger.h"
#include "test_data_utils.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <list>
#include <string>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 999222u;

struct WriterFixture {
    fs::path                          directory;
    fs::path                          rawFile;
    fs::path                          idxFile;
    std::list<std::vector<uint8_t>*>  messages;
};

WriterFixture buildFixture(const std::string& hint, float sizeMB = 0.5f) {
    WriterFixture f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_log_writer_%s_%d_%ld",
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
    f.rawFile = rawFiles.front().name;
    if (!idxFiles.empty()) f.idxFile = idxFiles.front().name;
    return f;
}

void teardown(WriterFixture& f) {
    for (auto* m : f.messages) delete m;
    f.messages.clear();
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

class LogWriterTest : public ::testing::Test {
protected:
    WriterFixture fixture;

    void TearDown() override { teardown(fixture); }
};

// ---------------------------------------------------------------------------
// Round-trip: read every record, write through ISLogWriter, re-read, compare.
// ---------------------------------------------------------------------------
TEST_F(LogWriterTest, RoundTripBitIdentical) {
    fixture = buildFixture("roundtrip");
    ASSERT_FALSE(fixture.rawFile.empty()) << "fixture generation failed";

    auto sourceR = ISLogReader::openSegment(fixture.rawFile);
    ASSERT_TRUE(sourceR.has_value()) << sourceR.error().message;
    const ISLogReader& source = sourceR.value();
    const std::size_t  inCount = source.recordCount();
    ASSERT_GT(inCount, 0u);

    const fs::path outRaw = fixture.directory / "rebake.raw";

    // Capture every source record into owning copies so we can diff
    // after the writer has run (its append moves the source's mmap
    // out from under raw views — the owned copy survives).
    std::vector<OwnedRecord> sourceRecords;
    sourceRecords.reserve(inCount);
    for (auto v : source.allRecords()) {
        sourceRecords.push_back(v.owned());
    }

    {
        auto writerR = ISLogWriter::create(outRaw, source.deviceId());
        ASSERT_TRUE(writerR.has_value()) << writerR.error().message;
        ISLogWriter writer = std::move(writerR.value());

        for (auto v : source.allRecords()) {
            auto r = writer.append(v);
            ASSERT_TRUE(r.has_value()) << r.error().message;
        }
        EXPECT_EQ(writer.recordCount(), inCount);

        auto fin = writer.finalize();
        ASSERT_TRUE(fin.has_value()) << fin.error().message;
        EXPECT_TRUE(writer.isFinalized());
    }

    // Both files exist at final paths; tempfiles do not.
    EXPECT_TRUE(fs::exists(outRaw));
    fs::path outIdx = outRaw;
    outIdx.replace_extension(".idx");
    EXPECT_TRUE(fs::exists(outIdx));
    EXPECT_FALSE(fs::exists(fs::path(outRaw.string() + ".tmp")));
    EXPECT_FALSE(fs::exists(fs::path(outIdx.string() + ".tmp")));

    // Re-read and diff.
    auto destR = ISLogReader::openSegment(outRaw);
    ASSERT_TRUE(destR.has_value()) << destR.error().message;
    const ISLogReader& dest = destR.value();
    EXPECT_TRUE(dest.hadOnDiskIndex());
    EXPECT_FALSE(dest.isTruncated());
    ASSERT_EQ(dest.recordCount(), inCount);

    std::size_t i = 0;
    for (auto v : dest.allRecords()) {
        const OwnedRecord& src = sourceRecords[i++];
        EXPECT_EQ(v.did(), src.did()) << "record " << (i - 1);
        ASSERT_EQ(v.bytes().second, src.bytes().second) << "size mismatch at " << (i - 1);
        EXPECT_EQ(0, std::memcmp(v.bytes().first,
                                 src.bytes().first,
                                 v.bytes().second))
            << "byte mismatch at record " << (i - 1);
    }
    EXPECT_EQ(i, inCount);
}

// ---------------------------------------------------------------------------
// SN-8383: the writer always emits the current .idx version (v2.1) and carries
// the source's per-record host-uptime delta through, declaring HAS_LOCAL_DELTA.
// Guards against the writer emitting an older format or zeroing the delta.
// ---------------------------------------------------------------------------
TEST_F(LogWriterTest, PreservesLocalUptimeDeltaAndAlwaysWritesV21) {
    fixture = buildFixture("delta");   // only needed for a scratch directory
    ASSERT_FALSE(fixture.directory.empty());

    // Drive the writer with synthetic views carrying known, non-zero per-record
    // deltas. Benign (non-GNSS-raw) DIDs + small host-uptime timestamps keep the
    // reader on its trust-the-sidecar path (no poison/obs-time rebuild), so we
    // observe exactly what the writer emitted.
    struct SrcRec { uint32_t did; uint64_t ts; uint16_t flags; uint32_t delta; std::vector<uint8_t> payload; };
    const std::vector<SrcRec> src = {
        { 4u, 1000u, idx::IS_LOG_IDX_REC_FLAG_HAS_TOW, 5u,    {1, 2, 3, 4} },
        { 5u, 2000u, 0u,                                123u,  {9, 9}       },
        { 4u, 3000u, 0u,                                4567u, {7, 7, 7}    },
    };
    const uint64_t devId = 0x42u;

    const fs::path outRaw = fixture.directory / "delta_synth.raw";
    {
        auto writerR = ISLogWriter::create(outRaw, devId);
        ASSERT_TRUE(writerR.has_value()) << writerR.error().message;
        ISLogWriter writer = std::move(writerR.value());
        for (const auto& r : src) {
            ISRecordView v{ r.did, r.ts, devId, /*offset*/ 0u,
                            r.payload.data(), r.payload.size(), r.flags };
            v.setLocalUptimeMs(r.delta);
            ASSERT_TRUE(writer.append(v).has_value());
        }
        ASSERT_TRUE(writer.finalize().has_value());
    }

    // Inspect the writer's on-disk .idx bytes directly — this is the writer's
    // output contract. (Round-tripping through ISLogReader::openSegment would
    // conflate the writer with the reader's .raw-validation / rebuild logic,
    // which a synthetic garbage .raw would trip.)
    fs::path outIdx = outRaw;
    outIdx.replace_extension(".idx");
    std::ifstream in(outIdx, std::ios::binary);
    ASSERT_TRUE(in.good());
    const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                                     std::istreambuf_iterator<char>());
    ASSERT_GE(bytes.size(),
              idx::IS_LOG_IDX_HEADER_SIZE + src.size() * idx::IS_LOG_IDX_RECORD_V2_1_SIZE);

    auto hdr = idx::parseHeader(bytes.data());
    ASSERT_TRUE(hdr.has_value()) << hdr.error().message;

    // Always v2.1: 32-byte stride + HAS_LOCAL_DELTA declared (never a downgrade).
    EXPECT_EQ(hdr->record_size, idx::IS_LOG_IDX_RECORD_V2_1_SIZE);
    EXPECT_NE(0, hdr->flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_LOCAL_DELTA);
    EXPECT_EQ(hdr->total_records, src.size());

    // Each record, in write order, carries its source delta verbatim.
    for (std::size_t i = 0; i < src.size(); ++i) {
        const auto rec = idx::parseRecord(
            bytes.data() + idx::IS_LOG_IDX_HEADER_SIZE + i * idx::IS_LOG_IDX_RECORD_V2_1_SIZE,
            idx::IS_LOG_IDX_RECORD_V2_1_SIZE);
        EXPECT_EQ(rec.timestamp,       src[i].ts)    << "ts mismatch at record " << i;
        EXPECT_EQ(rec.local_uptime_ms, src[i].delta) << "delta mismatch at record " << i;
    }
}

// ---------------------------------------------------------------------------
// Filtered round-trip via appendFiltered + a timestamp predicate.
// ---------------------------------------------------------------------------
TEST_F(LogWriterTest, FilteredRoundTrip) {
    fixture = buildFixture("filter");
    ASSERT_FALSE(fixture.rawFile.empty());

    auto sourceR = ISLogReader::openSegment(fixture.rawFile);
    ASSERT_TRUE(sourceR.has_value()) << sourceR.error().message;
    const ISLogReader& source = sourceR.value();
    ASSERT_GT(source.recordCount(), 4u);

    // Pick a window covering the middle 50% of the records by sampling
    // their timestamps. The synthetic fixture's record timestamps are
    // monotonic within a DID; over allRecords() they bounce around per
    // DID — but the min and max still bracket every record.
    std::vector<uint64_t> tss;
    tss.reserve(source.recordCount());
    for (auto v : source.allRecords()) {
        tss.push_back(v.timestamp().value);
    }
    std::sort(tss.begin(), tss.end());
    const uint64_t t0 = tss[tss.size() / 4];
    const uint64_t t1 = tss[3 * tss.size() / 4];

    auto in_window = [t0, t1](const ISRecordView& v) {
        const uint64_t ts = v.timestamp().value;
        return ts >= t0 && ts < t1;
    };

    // Reference: count records the predicate keeps.
    std::size_t expected = 0;
    for (auto v : source.allRecords()) if (in_window(v)) ++expected;
    ASSERT_GT(expected, 0u);
    ASSERT_LT(expected, source.recordCount());

    const fs::path outRaw = fixture.directory / "filtered.raw";
    auto writerR = ISLogWriter::create(outRaw, source.deviceId());
    ASSERT_TRUE(writerR.has_value()) << writerR.error().message;
    ISLogWriter writer = std::move(writerR.value());

    auto count = writer.appendFiltered(source.allRecords(), in_window);
    ASSERT_TRUE(count.has_value()) << count.error().message;
    EXPECT_EQ(count.value(), expected);

    auto fin = writer.finalize();
    ASSERT_TRUE(fin.has_value()) << fin.error().message;

    auto destR = ISLogReader::openSegment(outRaw);
    ASSERT_TRUE(destR.has_value()) << destR.error().message;
    EXPECT_EQ(destR.value().recordCount(), expected);
    for (auto v : destR.value().allRecords()) {
        EXPECT_TRUE(in_window(v)) << "record outside the requested window leaked";
    }
}

// ---------------------------------------------------------------------------
// State-machine misuse: finalize-without-create, append-after-finalize, etc.
// ---------------------------------------------------------------------------
TEST_F(LogWriterTest, StateMachineMisuse) {
    fixture = buildFixture("misuse");
    ASSERT_FALSE(fixture.rawFile.empty());

    auto sourceR = ISLogReader::openSegment(fixture.rawFile);
    ASSERT_TRUE(sourceR.has_value());
    const ISLogReader& source = sourceR.value();

    // "finalize without create" via a moved-from writer.
    {
        auto writerR = ISLogWriter::create(fixture.directory / "moved.raw",
                                           source.deviceId());
        ASSERT_TRUE(writerR.has_value());
        ISLogWriter src = std::move(writerR.value());
        ISLogWriter dst = std::move(src);  // src is now moved-from

        auto firstView = *source.allRecords().begin();
        auto a = src.append(firstView);
        EXPECT_FALSE(a.has_value());
        EXPECT_EQ(a.error().code, ISErrorCode::InvalidArgument);

        auto f = src.finalize();
        EXPECT_FALSE(f.has_value());
        EXPECT_EQ(f.error().code, ISErrorCode::InvalidArgument);

        // Clean up dst.
        EXPECT_TRUE(dst.finalize().has_value());
    }

    // append-after-finalize, double-finalize.
    {
        const fs::path out = fixture.directory / "double_finalize.raw";
        auto writerR = ISLogWriter::create(out, source.deviceId());
        ASSERT_TRUE(writerR.has_value());
        ISLogWriter w = std::move(writerR.value());
        for (auto v : source.allRecords()) {
            ASSERT_TRUE(w.append(v).has_value());
        }
        ASSERT_TRUE(w.finalize().has_value());

        auto firstView = *source.allRecords().begin();
        auto a = w.append(firstView);
        EXPECT_FALSE(a.has_value());
        EXPECT_EQ(a.error().code, ISErrorCode::InvalidArgument);

        auto f = w.finalize();
        EXPECT_FALSE(f.has_value());
        EXPECT_EQ(f.error().code, ISErrorCode::InvalidArgument);
    }
}

// ---------------------------------------------------------------------------
// Refuses to overwrite an existing path unless `overwrite = true`.
// ---------------------------------------------------------------------------
TEST_F(LogWriterTest, RefusesExistingPath) {
    fixture = buildFixture("noclobber");
    ASSERT_FALSE(fixture.rawFile.empty());

    const fs::path out = fixture.directory / "exists.raw";
    {
        std::ofstream(out) << "existing content";
    }
    ASSERT_TRUE(fs::exists(out));

    auto bad = ISLogWriter::create(out, 0);
    EXPECT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error().code, ISErrorCode::InvalidArgument);

    ISLogWriter::Options o;
    o.rawOutPath     = out;
    o.sourceDeviceId = 0;
    o.overwrite      = true;
    auto good = ISLogWriter::create(o);
    ASSERT_TRUE(good.has_value()) << good.error().message;
    auto fin = good.value().finalize();
    EXPECT_TRUE(fin.has_value()) << fin.error().message;
    EXPECT_TRUE(fs::exists(out));
}

// ---------------------------------------------------------------------------
// .idx v2 header reflects actual stats (record count, timestamps,
// FINALIZED flag) after finalize.
// ---------------------------------------------------------------------------
TEST_F(LogWriterTest, HeaderReflectsStats) {
    fixture = buildFixture("header");
    ASSERT_FALSE(fixture.rawFile.empty());

    auto sourceR = ISLogReader::openSegment(fixture.rawFile);
    ASSERT_TRUE(sourceR.has_value());
    const ISLogReader& source = sourceR.value();

    const fs::path outRaw = fixture.directory / "stats.raw";
    auto writerR = ISLogWriter::create(outRaw, source.deviceId());
    ASSERT_TRUE(writerR.has_value());
    ISLogWriter writer = std::move(writerR.value());
    for (auto v : source.allRecords()) {
        ASSERT_TRUE(writer.append(v).has_value());
    }
    ASSERT_TRUE(writer.finalize().has_value());

    auto destR = ISLogReader::openSegment(outRaw);
    ASSERT_TRUE(destR.has_value());
    const idx::is_log_idx_header_t& h = destR.value().header();
    EXPECT_TRUE((h.flags & idx::IS_LOG_IDX_HDR_FLAG_FINALIZED) != 0);
    EXPECT_EQ(h.total_records, source.recordCount());
    EXPECT_EQ(h.first_timestamp_ms, writer.firstTimestamp());
    EXPECT_EQ(h.last_timestamp_ms,  writer.lastTimestamp());
    EXPECT_EQ(h.sync_point_count,   writer.syncPointCount());
    EXPECT_EQ(h.version, idx::IS_LOG_IDX_VERSION_V2);
}

// ---------------------------------------------------------------------------
// Device-id mismatch fails on the first non-matching record, no partial
// output left at the final path (tempfiles cleaned up by destructor).
// ---------------------------------------------------------------------------
TEST_F(LogWriterTest, DeviceIdMismatchRejected) {
    fixture = buildFixture("device_mismatch");
    ASSERT_FALSE(fixture.rawFile.empty());

    auto sourceR = ISLogReader::openSegment(fixture.rawFile);
    ASSERT_TRUE(sourceR.has_value());
    const ISLogReader& source = sourceR.value();
    ASSERT_NE(source.deviceId(), 0u);

    const fs::path outRaw = fixture.directory / "mismatch.raw";
    const uint64_t bogusId = source.deviceId() + 1u;
    {
        auto writerR = ISLogWriter::create(outRaw, bogusId);
        ASSERT_TRUE(writerR.has_value());
        ISLogWriter writer = std::move(writerR.value());

        auto firstView = *source.allRecords().begin();
        auto r = writer.append(firstView);
        EXPECT_FALSE(r.has_value());
        EXPECT_EQ(r.error().code, ISErrorCode::InvalidArgument);
        // Drop without finalize — destructor cleans up tempfiles.
    }
    EXPECT_FALSE(fs::exists(outRaw));
    fs::path outIdx = outRaw;
    outIdx.replace_extension(".idx");
    EXPECT_FALSE(fs::exists(outIdx));
    EXPECT_FALSE(fs::exists(fs::path(outRaw.string() + ".tmp")));
    EXPECT_FALSE(fs::exists(fs::path(outIdx.string() + ".tmp")));
}

// ---------------------------------------------------------------------------
// Destructor without finalize leaves no files at the final path and
// removes the tempfiles.
// ---------------------------------------------------------------------------
TEST_F(LogWriterTest, AbandonedWriterCleansTempfiles) {
    fixture = buildFixture("abandoned");
    ASSERT_FALSE(fixture.rawFile.empty());

    auto sourceR = ISLogReader::openSegment(fixture.rawFile);
    ASSERT_TRUE(sourceR.has_value());
    const ISLogReader& source = sourceR.value();

    const fs::path outRaw = fixture.directory / "abandoned.raw";
    {
        auto writerR = ISLogWriter::create(outRaw, source.deviceId());
        ASSERT_TRUE(writerR.has_value());
        ISLogWriter writer = std::move(writerR.value());
        // Append a few records, then drop without finalize.
        std::size_t n = 0;
        for (auto v : source.allRecords()) {
            ASSERT_TRUE(writer.append(v).has_value());
            if (++n >= 5) break;
        }
    }
    fs::path outIdx = outRaw;
    outIdx.replace_extension(".idx");
    EXPECT_FALSE(fs::exists(outRaw));
    EXPECT_FALSE(fs::exists(outIdx));
    EXPECT_FALSE(fs::exists(fs::path(outRaw.string() + ".tmp")));
    EXPECT_FALSE(fs::exists(fs::path(outIdx.string() + ".tmp")));
}

} // namespace
