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
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogIndex.h"
#include "ISLogReader.h"
#include "ISLogger.h"
#include "ISTimeResolver.h"
#include "data_sets.h"
#include "test_data_utils.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <list>
#include <string>
#include <vector>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 583201u;
constexpr float    kFixtureSizeMB = 1.0f;

// Portable unique temp directory (works on POSIX + Windows CI) — mirrors test_log_bounds.cpp's
// makeTempDir(). Copilot review (PR #1298): the original ::getpid()/hardcoded-"/tmp" version
// wasn't excluded from the Windows build in tests/CMakeLists.txt like test_log_reader.cpp is, so
// it would have broken that build; ported to std::filesystem::temp_directory_path() instead of
// adding this file to that exclusion list, since nothing else here needs POSIX.
fs::path uniqueTempDir(const std::string& hint) {
    static unsigned counter = 0;
    return fs::temp_directory_path() / ("test_log_reader_dat_" + hint + "_" + std::to_string(counter++));
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

// Copilot review (PR #1298): formatFromExtension() originally did a strict `==` compare, which
// regressed ISLog::openDirectory's pre-existing case-insensitive `.raw`/`.RAW` contract
// (ISLog.cpp's isRawExtension) for BOTH extensions once this function started gating `.dat` too.
// Exercises uppercase/mixed-case for each extension directly against openSegment, one level below
// the directory scan that would otherwise mask the regression by never trying an uppercase name.
TEST_F(LogReaderDatTest, OpenSegmentAcceptsMixedCaseExtension) {
    // Same stem as f_.datFile -> replace_extension(".idx") resolves to f_.idxFile, already on
    // disk from fixture generation, so only the .dat itself needs copying under the new name.
    const fs::path upperDat = f_.datFile.parent_path() / (f_.datFile.stem().string() + ".DAT");

    // Windows CI failure (2026-09-08): NTFS/APFS are case-insensitive, so upperDat and
    // f_.datFile already name the SAME file there -- fs::exists(upperDat) is true before any
    // copy, and copy_file(f_.datFile, upperDat) then fails with "file exists" (source and
    // destination are equivalent). That IS the case-insensitive-filesystem behavior this test
    // wants to exercise (a plain path-string case change reaches the same on-disk bytes with no
    // extra step) -- skip the copy rather than treating it as an error. Linux's ext4 is
    // case-sensitive, so upperDat doesn't exist yet there and genuinely needs a real copy.
    if (!fs::exists(upperDat)) {
        std::error_code ec;
        fs::copy_file(f_.datFile, upperDat, ec);
        ASSERT_FALSE(ec) << ec.message();
    }

    auto r = ISLogReader::openSegment(upperDat);
    ASSERT_TRUE(r.has_value()) << "openSegment(.DAT) failed: " << r.error().message;
    EXPECT_EQ(r->format(), ISLogReader::SegmentFormat::Dat);
    EXPECT_GT(r->recordCount(), 0u);
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

// ============================================================
// ISDeviceLog::format() + mixed-format rejection (D-119 AC)
// ============================================================

TEST(LogReaderDat, DeviceLogFormatIsDat) {
    auto f = generateDatFixture("device_log_format");
    ASSERT_FALSE(f.datFile.empty());

    auto log = ISDeviceLog::fromSegments({f.datFile});
    ASSERT_TRUE(log.has_value()) << log.error().message;
    EXPECT_EQ(log->format(), ISLogReader::SegmentFormat::Dat);

    teardownDatFixture(f);
}

TEST(LogReaderDat, MixedFormatSegmentsRejected) {
    std::list<std::vector<uint8_t>*> wireMessages;
    GenerateRawLogData(wireMessages, kFixtureSizeMB);
    ASSERT_FALSE(wireMessages.empty());
    auto decoded = decodeIsbMessages(wireMessages);
    ASSERT_FALSE(decoded.empty());

    const fs::path rawDir = uniqueTempDir("mixed_raw");
    const fs::path datDir = uniqueTempDir("mixed_dat");
    auto rawSegments = writeRawSegments(rawDir, wireMessages);
    auto datSegments = writeDatSegments(datDir, decoded);
    for (auto* msg : wireMessages) delete msg;
    ASSERT_FALSE(rawSegments.empty());
    ASSERT_FALSE(datSegments.empty());

    // Same device (kFixtureHwId/kFixtureSerial are shared constants) but mixed formats — must be
    // rejected even though the device-id check alone would pass.
    auto log = ISDeviceLog::fromSegments({rawSegments.front(), datSegments.front()});
    ASSERT_FALSE(log.has_value());
    EXPECT_EQ(log.error().code, ISErrorCode::Unsupported);

    ISFileManager::DeleteDirectory(rawDir.string());
    ISFileManager::DeleteDirectory(datDir.string());
}

// ============================================================
// Device-info extraction from a TRUSTED on-disk index (D-119 AC)
// ============================================================

// Copilot review (PR #1298): none of the tests above actually exercise
// deriveDeviceIdDat()'s offset-based dev_info_t read against a writer-produced
// .idx that's trusted as-is (hadOnDiskIndex() == true, no rebuild) --
// GenerateDataLogFiles()'s synthetic messages never include DID_DEV_INFO, so
// every test above that opens its fixture unmodified silently falls through
// to the filename "SN<n>" fallback (deriveDeviceId()'s step 2), which sets
// deviceId_ but leaves hdwId_ at 0 -- it can't detect a wrong `.idx` offset
// because it never reads through one. This test writes a real DID_DEV_INFO
// record via the production LOGTYPE_DAT path (mirrors Logalyzer's
// writeDatFixture in test_raw_series_builder.cpp) and opens the segment
// WITHOUT touching its .idx, so deriveDeviceIdDat() must decode dev_info_t at
// records_[i].offset to pass -- hdwId() only comes from that path (the
// filename fallback never sets it), so a wrong offset reads garbage bytes,
// fails the `hdr.size == sizeof(dev_info_t)` guard, and this test fails.
TEST(LogReaderDat, DeviceInfoDerivedFromTrustedOnDiskIndex) {
    const fs::path dir = uniqueTempDir("devinfo_trusted");
    ISFileManager::DeleteDirectory(dir.string());

    {
        cISLogger logger;
        cISLogger::sSaveOptions opts;
        opts.logType               = cISLogger::LOGTYPE_DAT;
        opts.useSubFolderTimestamp = false;
        ASSERT_TRUE(logger.InitSave(dir.string(), opts));
        auto dev = logger.registerDevice(kFixtureHwId, kFixtureSerial);
        ASSERT_TRUE(dev != nullptr);
        logger.EnableLogging(true);

        dev_info_t info{};
        info.serialNumber   = kFixtureSerial;
        info.hardwareType   = IS_HARDWARE_TYPE_IMX;
        info.hardwareVer[0] = 5;
        p_data_hdr_t hdr{};
        hdr.id   = DID_DEV_INFO;
        hdr.size = sizeof(info);
        logger.LogData(dev, &hdr, reinterpret_cast<const uint8_t*>(&info));

        // A few ordinary records after it so DID_DEV_INFO isn't the only/last
        // thing in the chunk -- exercises the offset arithmetic for a record
        // that isn't chunk-start.
        ins_2_t ins{};
        ins.week        = 2300;
        ins.timeOfWeek  = 100.0;
        p_data_hdr_t insHdr{};
        insHdr.id   = DID_INS_2;
        insHdr.size = sizeof(ins);
        for (int i = 0; i < 5; ++i) {
            logger.LogData(dev, &insHdr, reinterpret_cast<const uint8_t*>(&ins));
        }

        logger.CloseAllFiles();
    }

    std::vector<ISFileManager::file_info_t> datFiles, idxFiles;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.dat$", datFiles);
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.idx$", idxFiles);
    ASSERT_FALSE(datFiles.empty());
    ASSERT_FALSE(idxFiles.empty()) << "expected a writer-produced .idx alongside the .dat";

    // Deliberately do NOT delete the .idx -- openSegment must trust it as-is.
    auto r = ISLogReader::openSegment(datFiles.front().name);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    EXPECT_TRUE(r->hadOnDiskIndex())
        << "this test only proves what it claims to if the on-disk .idx was trusted, not rebuilt";

    EXPECT_TRUE(r->hasDevInfo());
    EXPECT_EQ(r->hdwId(), kFixtureHwId)
        << "hdwId() is only ever set by the offset-based dev_info_t decode -- a wrong .idx offset "
           "leaves it at 0 regardless of the filename-fallback serial";
    EXPECT_EQ(r->devInfo().serialNumber, kFixtureSerial);
    EXPECT_EQ(r->deviceId(), kFixtureSerial);

    ISFileManager::DeleteDirectory(dir.string());
}

// ============================================================
// ISTimeResolver Option A on .dat (Kyle 2026-09-07)
// ============================================================

// The bug report that prompted Option A was against a .dat manufacturing/
// calibration capture (thermal-chamber run, no INS/GNSS record at all) --
// scanSegmentForSyncsDat mirrors scanSegmentForSyncs's Option A fix
// line-for-line, but exercised through the .dat-native decode path rather
// than the .raw comm-parser, since that's the format this bug actually
// surfaced against.
TEST(LogReaderDat, SyncedSysParamsAloneEstablishesSyncPointsInDatLog) {
    const fs::path dir = uniqueTempDir("sysparams_only_dat");
    ISFileManager::DeleteDirectory(dir.string());

    {
        cISLogger logger;
        cISLogger::sSaveOptions opts;
        opts.logType               = cISLogger::LOGTYPE_DAT;
        opts.useSubFolderTimestamp = false;
        ASSERT_TRUE(logger.InitSave(dir.string(), opts));
        auto dev = logger.registerDevice(kFixtureHwId, kFixtureSerial);
        ASSERT_TRUE(dev != nullptr);
        logger.EnableLogging(true);

        for (uint32_t towMs : { 200'000'000u, 200'010'000u, 200'020'000u }) {
            sys_params_t sp{};
            sp.timeOfWeekMs = towMs;
            sp.upTime       = (towMs - 200'000'000u) / 1000.0 + 5.0;
            sp.hdwStatus    = HDW_STATUS_GNSS_TIME_OF_WEEK_VALID;
            p_data_hdr_t hdr{};
            hdr.id   = DID_SYS_PARAMS;
            hdr.size = sizeof(sp);
            logger.LogData(dev, &hdr, reinterpret_cast<const uint8_t*>(&sp));
        }
        logger.CloseAllFiles();
    }

    std::vector<ISFileManager::file_info_t> datFiles;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.dat$", datFiles);
    ASSERT_FALSE(datFiles.empty());

    auto log = ISDeviceLog::fromSegments({ datFiles.front().name });
    ASSERT_TRUE(log.has_value()) << log.error().message;

    auto syncs = ISTimeResolver::detectSyncPoints(*log);
    ASSERT_FALSE(syncs.empty())
        << "a synced DID_SYS_PARAMS record should establish a sync point on its own, "
           "even with no INS/GNSS DID present";
    for (const auto& sp : syncs) {
        EXPECT_EQ(sp.sourceDid, static_cast<uint32_t>(DID_SYS_PARAMS));
    }

    auto resolverR = ISTimeResolver::build(*log);
    ASSERT_TRUE(resolverR.has_value());
    const TimeStamp t = resolverR->resolve(200'010'000u, log->deviceId());
    EXPECT_EQ(t.source, TimeSource::PayloadToW);
    EXPECT_EQ(t.confidence, TimeConfidence::Exact);

    ISFileManager::DeleteDirectory(dir.string());
}

// ============================================================
// ISTimeResolver on .dat (D0066 compliance — D-119 / SN-8626)
// ============================================================

// Regression for a bug caught only by full end-to-end integration testing (Logalyzer's
// RawSeriesBuilder + LogLoader), not by this file's own reader-level tests: ISTimeResolver's
// sync-point detection (scanSegmentForSyncs) used to unconditionally re-scan
// `reader.rawBytes()` via `is_comm_parse_byte` -- which finds nothing in a `.dat` file (no wire
// protocol to parse, D0082), so EVERY `.dat` record resolved as SessionOnly/Unknown regardless of
// how many valid ToW-bearing records it had. Without scanSegmentForSyncsDat's fix, `.dat` support
// would open and index correctly but never produce a usable chart -- every series would be
// filtered out by the resolver. This test would have caught that at the SDK level.
TEST(LogReaderDat, TimeResolverFindsSyncPointsInDatLog) {
    std::list<std::vector<uint8_t>*> wireMessages;
    GenerateRawLogData(wireMessages, kFixtureSizeMB);
    ASSERT_FALSE(wireMessages.empty());

    auto decoded = decodeIsbMessages(wireMessages);
    for (auto* msg : wireMessages) delete msg;
    ASSERT_FALSE(decoded.empty());

    const fs::path dir = uniqueTempDir("resolver");
    auto segments = writeDatSegments(dir, decoded);
    ASSERT_FALSE(segments.empty());

    auto log = ISDeviceLog::fromSegments(segments);
    ASSERT_TRUE(log.has_value()) << log.error().message;
    ASSERT_EQ(log->format(), ISLogReader::SegmentFormat::Dat);

    auto syncs = ISTimeResolver::detectSyncPoints(*log);
    ASSERT_FALSE(syncs.empty())
        << "no sync points found in a .dat log -- scanSegmentForSyncs isn't reaching .dat records";

    auto resolverR = ISTimeResolver::build(*log);
    ASSERT_TRUE(resolverR.has_value()) << resolverR.error().message;
    const auto& resolver = *resolverR;

    // At least one ToW-bearing record must resolve as PayloadToW/Exact -- SessionOnly/Unknown
    // across the board is exactly the symptom the bug above produced.
    bool foundAnchored = false;
    for (auto v : log->allRecords()) {
        if (v.timestamp().value == 0) continue;
        const auto resolved = resolver.resolve(v.timestamp().value, log->deviceId());
        if (resolved.source == TimeSource::PayloadToW && resolved.confidence == TimeConfidence::Exact) {
            foundAnchored = true;
            break;
        }
    }
    EXPECT_TRUE(foundAnchored) << "no .dat record resolved as PayloadToW/Exact";

    ISFileManager::DeleteDirectory(dir.string());
}
