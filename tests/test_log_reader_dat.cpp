/**
 * @file test_log_reader_dat.cpp
 * @brief D-119 / SN-8626 acceptance tests for `.dat` (LOGTYPE_DAT) support in `ISLogReader`.
 *
 * Mirrors test_log_reader.cpp's fixture strategy (generate a fresh log per test via the existing
 * test_data_utils helpers, assert against the result, wipe the temp dir on teardown) — see that
 * file's header comment for the rationale (host-time-dependent generator, so no committed binary
 * fixture).
 *
 * Two fixture styles are used here:
 *   - GenerateDataLogFiles(..., LOGTYPE_DAT, ...) for tests that only need *a* .dat log (open,
 *     header/counts, truncation, sidecar reuse) — same helper test_ISLogger.cpp's dat_conversion
 *     test already exercises against the legacy cDeviceLogSerial::ReadData() path.
 *   - A decode-and-refeed helper (decodeIsbMessages) for the record-for-record equivalence test
 *     and the multi-segment test, which both need the *same* underlying messages available in
 *     both .raw's wire-format form and .dat's parsed (p_data_hdr_t, payload) form.
 */

#include <gtest/gtest.h>

// com_manager.h FIRST — see test_log_reader.cpp's comment on the ISFirmwareUpdater.h extern-C wrap.
#include "com_manager.h"

#include "DeviceLog.h"
#include "ISComm.h"
#include "ISFileManager.h"
#include "ISLogIndex.h"
#include "ISLogReader.h"
#include "ISLogger.h"
#include "data_sets.h"
#include "test_data_utils.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <list>
#include <string>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 583201u;
constexpr float    kFixtureSizeMB = 1.0f;

fs::path uniqueTempDir(const std::string& hint) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "/tmp/test_log_reader_dat_%s_%d_%ld",
                  hint.c_str(), ::getpid(), static_cast<long>(::time(nullptr)));
    return fs::path{buf};
}

// ============================================================
// Fixture style A: GenerateDataLogFiles — "just give me a .dat log"
// ============================================================

struct DatFixture {
    fs::path directory;
    fs::path datFile;
    fs::path idxFile;
};

DatFixture generateDatFixture(const std::string& dirHint) {
    DatFixture f;
    f.directory = uniqueTempDir(dirHint);
    ISFileManager::DeleteDirectory(f.directory.string());

    GenerateDataLogFiles(1, f.directory.string(), cISLogger::LOGTYPE_DAT, kFixtureSizeMB);

    std::vector<ISFileManager::file_info_t> datFiles, idxFiles;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, "\\.dat$", datFiles);
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, "\\.idx$", idxFiles);
    if (datFiles.empty() || idxFiles.empty()) return f;

    std::sort(datFiles.begin(), datFiles.end(),
              [](const auto& a, const auto& b) { return a.name < b.name; });
    f.datFile = datFiles.front().name;
    f.idxFile = idxFiles.front().name;
    return f;
}

void teardownDatFixture(DatFixture& f) {
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

class LogReaderDatTest : public ::testing::Test {
protected:
    void SetUp() override {
        f_ = generateDatFixture(::testing::UnitTest::GetInstance()->current_test_info()->name());
        ASSERT_FALSE(f_.datFile.empty()) << "fixture generation produced no .dat";
        ASSERT_TRUE(fs::exists(f_.idxFile)) << "expected .idx alongside .dat";
    }
    void TearDown() override { teardownDatFixture(f_); }

    DatFixture f_;
};

// ============================================================
// Fixture style B: decode-and-refeed — .raw and .dat from the SAME messages
// ============================================================

struct DecodedRecord {
    p_data_hdr_t         hdr{};
    std::vector<uint8_t> payload;
};

/**
 * @brief Decodes GenerateRawLogData()'s wire-format messages into parsed (header, payload) pairs.
 *
 * Only ISB (_PTYPE_INERTIAL_SENSE_DATA / _CMD) messages decode to a p_data_hdr_t — a .dat log has
 * no representation for NMEA/RTCM3/u-blox (DeviceLogSerial's SaveData() only ever receives
 * already-parsed p_data_hdr_t/payload; see D0082 / DeviceLogSerial.h), so those are skipped here
 * exactly as ISLogReader's own .raw scan skips them when building records_ (see
 * ISLogReader::buildIndexFromScan — non-ISB packets consume bytes but are never indexed either).
 * A single is_comm_instance_t is fed the full concatenated stream (not reset between messages) so
 * multi-byte framing spanning message boundaries decodes exactly as it would from one .raw file.
 */
std::vector<DecodedRecord> decodeIsbMessages(const std::list<std::vector<uint8_t>*>& wireMessages) {
    std::vector<DecodedRecord> out;

    is_comm_instance_t comm{};
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);
    is_comm_enable_protocol(&comm, _PTYPE_NMEA);
    is_comm_enable_protocol(&comm, _PTYPE_RTCM3);
    is_comm_enable_protocol(&comm, _PTYPE_UBLOX);

    for (auto* msg : wireMessages) {
        for (uint8_t b : *msg) {
            protocol_type_t ptype = is_comm_parse_byte(&comm, b);
            if (ptype == _PTYPE_INERTIAL_SENSE_DATA || ptype == _PTYPE_INERTIAL_SENSE_CMD) {
                DecodedRecord rec;
                rec.hdr = comm.rxPkt.dataHdr;
                rec.payload.assign(comm.rxPkt.data.ptr, comm.rxPkt.data.ptr + rec.hdr.size);
                out.push_back(std::move(rec));
            }
        }
    }
    return out;
}

/** @brief Writes `wireMessages` to a fresh `.raw` log under `directory`. Returns the segment paths (sorted). */
std::vector<fs::path> writeRawSegments(const fs::path& directory,
                                       const std::list<std::vector<uint8_t>*>& wireMessages,
                                       uint32_t maxFileSize = 0) {
    ISFileManager::DeleteDirectory(directory.string());
    {
        cISLogger logger;
        cISLogger::sSaveOptions opts;
        opts.logType               = cISLogger::LOGTYPE_RAW;
        opts.useSubFolderTimestamp = false;
        if (maxFileSize) opts.maxFileSize = maxFileSize;
        if (!logger.InitSave(directory.string(), opts)) return {};
        auto dev = logger.registerDevice(kFixtureHwId, kFixtureSerial);
        if (!dev) return {};
        logger.EnableLogging(true);
        for (auto* msg : wireMessages) {
            logger.LogData(dev, static_cast<int>(msg->size()), msg->data());
        }
        logger.CloseAllFiles();
    }
    std::vector<ISFileManager::file_info_t> files;
    ISFileManager::GetAllFilesInDirectory(directory.string(), true, "\\.raw$", files);
    std::sort(files.begin(), files.end(), [](const auto& a, const auto& b) { return a.name < b.name; });
    std::vector<fs::path> out;
    for (auto& fi : files) out.emplace_back(fi.name);
    return out;
}

/** @brief Writes `decoded` records to a fresh `.dat` log under `directory`. Returns the segment paths (sorted). */
std::vector<fs::path> writeDatSegments(const fs::path& directory,
                                       const std::vector<DecodedRecord>& decoded,
                                       uint32_t maxFileSize = 0) {
    ISFileManager::DeleteDirectory(directory.string());
    {
        cISLogger logger;
        cISLogger::sSaveOptions opts;
        opts.logType               = cISLogger::LOGTYPE_DAT;
        opts.useSubFolderTimestamp = false;
        if (maxFileSize) opts.maxFileSize = maxFileSize;
        if (!logger.InitSave(directory.string(), opts)) return {};
        auto dev = logger.registerDevice(kFixtureHwId, kFixtureSerial);
        if (!dev) return {};
        logger.EnableLogging(true);
        for (const auto& rec : decoded) {
            // Copy the header — LogData() takes a non-const pointer.
            p_data_hdr_t hdr = rec.hdr;
            logger.LogData(dev, &hdr, rec.payload.data());
        }
        logger.CloseAllFiles();
    }
    std::vector<ISFileManager::file_info_t> files;
    ISFileManager::GetAllFilesInDirectory(directory.string(), true, "\\.dat$", files);
    std::sort(files.begin(), files.end(), [](const auto& a, const auto& b) { return a.name < b.name; });
    std::vector<fs::path> out;
    for (auto& fi : files) out.emplace_back(fi.name);
    return out;
}

} // namespace

// ============================================================
// Open / header / DID-list — same shape as test_log_reader.cpp's .raw coverage
// ============================================================

TEST_F(LogReaderDatTest, OpenSegmentSucceeds) {
    auto r = ISLogReader::openSegment(f_.datFile);
    ASSERT_TRUE(r.has_value()) << "openSegment failed: " << r.error().message;
    EXPECT_EQ(r->format(), ISLogReader::SegmentFormat::Dat);
    EXPECT_GT(r->recordCount(), 0u);
    EXPECT_FALSE(r->isTruncated());
}

TEST_F(LogReaderDatTest, HeaderAndCountsMatchFixture) {
    auto r = ISLogReader::openSegment(f_.datFile);
    ASSERT_TRUE(r.has_value());

    const uint64_t total_records = r->header().total_records;
    const uint64_t segEnd        = r->segmentEndTimestamp();
    const uint64_t segStart      = r->segmentStartTimestamp();
    EXPECT_EQ(r->recordCount(), total_records);
    EXPECT_GT(segEnd, 0u);
    EXPECT_LE(segStart, segEnd);

    auto dids = r->presentDids();
    EXPECT_FALSE(dids.empty());
    EXPECT_TRUE(std::is_sorted(dids.begin(), dids.end()));
}

TEST_F(LogReaderDatTest, RecordsByDidCountsMatchAggregate) {
    auto r = ISLogReader::openSegment(f_.datFile);
    ASSERT_TRUE(r.has_value());

    std::size_t accum = 0;
    for (auto did : r->presentDids()) {
        accum += r->records(did).size();
    }
    EXPECT_EQ(accum, r->recordCount());
}

// ============================================================
// Sidecar rebuild-and-reuse (D-119 AC: mirrors .raw + .idx v2 behavior)
// ============================================================

TEST_F(LogReaderDatTest, SidecarRebuildThenReuse) {
    // Fixture already has a writer-produced .idx; delete it to force a scan-rebuild on first open.
    std::error_code ec;
    fs::remove(f_.idxFile, ec);
    ASSERT_FALSE(fs::exists(f_.idxFile));

    auto first = ISLogReader::openSegment(f_.datFile);
    ASSERT_TRUE(first.has_value()) << first.error().message;
    EXPECT_FALSE(first->hadOnDiskIndex());
    EXPECT_GT(first->recordCount(), 0u);
    ASSERT_TRUE(fs::exists(f_.idxFile)) << "rebuilt sidecar should have been persisted";

    auto second = ISLogReader::openSegment(f_.datFile);
    ASSERT_TRUE(second.has_value()) << second.error().message;
    EXPECT_TRUE(second->hadOnDiskIndex());
    EXPECT_EQ(second->recordCount(), first->recordCount());
}

// ============================================================
// Truncation (D-119 AC: mirrors D-04's .raw truncated-tail handling)
// ============================================================

TEST_F(LogReaderDatTest, TruncatedMidChunkDetected) {
    std::error_code ec;
    fs::remove(f_.idxFile, ec);   // force a scan so truncation is actually detected, not trusted-from-sidecar

    const auto fullSize = fs::file_size(f_.datFile);
    ASSERT_GT(fullSize, 100u);

    // Truncate partway through — with a ~1 MB fixture spanning many 128 KB chunks, cutting at the
    // halfway point lands inside a chunk's body (a record header or payload), not exactly on a
    // chunk boundary.
    {
        std::error_code truncEc;
        fs::resize_file(f_.datFile, fullSize / 2, truncEc);
        ASSERT_FALSE(truncEc) << truncEc.message();
    }

    auto r = ISLogReader::openSegment(f_.datFile);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    EXPECT_TRUE(r->isTruncated());
    EXPECT_LE(r->truncationOffset(), fullSize / 2);
    EXPECT_GT(r->recordCount(), 0u) << "records before the truncation point should still be readable";
}

// ============================================================
// Record-for-record equivalence vs. .raw (D-119 AC)
// ============================================================

TEST(LogReaderDat, RawAndDatAgreeOnSameMessages) {
    std::list<std::vector<uint8_t>*> wireMessages;
    GenerateRawLogData(wireMessages, kFixtureSizeMB);
    ASSERT_FALSE(wireMessages.empty());

    auto decoded = decodeIsbMessages(wireMessages);
    ASSERT_FALSE(decoded.empty()) << "no ISB records decoded from the synthetic wire stream";

    const fs::path rawDir = uniqueTempDir("equiv_raw");
    const fs::path datDir = uniqueTempDir("equiv_dat");

    auto rawSegments = writeRawSegments(rawDir, wireMessages);
    auto datSegments = writeDatSegments(datDir, decoded);

    for (auto* msg : wireMessages) delete msg;

    ASSERT_FALSE(rawSegments.empty());
    ASSERT_FALSE(datSegments.empty());

    auto rawReader = ISLogReader::openSegment(rawSegments.front());
    auto datReader = ISLogReader::openSegment(datSegments.front());
    ASSERT_TRUE(rawReader.has_value()) << rawReader.error().message;
    ASSERT_TRUE(datReader.has_value()) << datReader.error().message;
    EXPECT_EQ(rawReader->format(), ISLogReader::SegmentFormat::Raw);
    EXPECT_EQ(datReader->format(), ISLogReader::SegmentFormat::Dat);

    // .raw's scan never indexes NMEA/RTCM3/UBX packets either (see buildIndexFromScan), so
    // rawReader->allRecords() is already exactly the ISB subsequence — directly comparable to
    // datReader's full record set with no extra filtering.
    std::vector<std::pair<uint32_t, uint64_t>> rawSeq, datSeq;
    for (auto v : rawReader->allRecords()) rawSeq.emplace_back(v.did(), v.timestamp().value);
    for (auto v : datReader->allRecords()) datSeq.emplace_back(v.did(), v.timestamp().value);

    ASSERT_EQ(rawSeq.size(), datSeq.size());
    for (std::size_t i = 0; i < rawSeq.size(); ++i) {
        EXPECT_EQ(rawSeq[i].first, datSeq[i].first)   << "DID mismatch at record " << i;
        EXPECT_EQ(rawSeq[i].second, datSeq[i].second) << "timestamp mismatch at record " << i;
    }

    ISFileManager::DeleteDirectory(rawDir.string());
    ISFileManager::DeleteDirectory(datDir.string());
}

// ============================================================
// Multi-file rotation (D-119 AC)
// ============================================================

TEST(LogReaderDat, MultiSegmentDatReadableAcrossFiles) {
    std::list<std::vector<uint8_t>*> wireMessages;
    GenerateRawLogData(wireMessages, 2.0f);   // 2 MB — enough to force rollover at the small cap below
    ASSERT_FALSE(wireMessages.empty());

    auto decoded = decodeIsbMessages(wireMessages);
    for (auto* msg : wireMessages) delete msg;
    ASSERT_FALSE(decoded.empty());

    const fs::path dir = uniqueTempDir("multiseg");
    auto segments = writeDatSegments(dir, decoded, /*maxFileSize=*/262144u);   // 256 KB segments
    ASSERT_GE(segments.size(), 2u) << "expected rollover to produce multiple .dat segments";

    std::size_t totalRecords = 0;
    for (const auto& seg : segments) {
        auto r = ISLogReader::openSegment(seg);
        ASSERT_TRUE(r.has_value()) << r.error().message;
        EXPECT_EQ(r->format(), ISLogReader::SegmentFormat::Dat);
        totalRecords += r->recordCount();
    }
    EXPECT_EQ(totalRecords, decoded.size());

    ISFileManager::DeleteDirectory(dir.string());
}
