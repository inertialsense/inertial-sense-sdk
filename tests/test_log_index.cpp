// D-01 / SN-7879 smoke tests for the .idx v2 sidecar format.
//
// Coverage:
//   * Round-trip — serialize a header + N records, parse them back,
//     all fields match.
//   * Legacy detection — synthetic v1 byte stream (no magic) must
//     report IsLogIndexResult::LegacyFormat, not silently misparse.
//   * Malformed — wrong magic / unsupported version / impossible
//     header_size each return a distinct error code.
//   * Layout discipline — sizeof / static asserts.
//
// File-level wrappers (cISLogFileBase) aren't exercised here; their
// only job is to copy bytes to/from the underlying file, and the pure
// serialize/parse layer that does the actual work is covered above.

#include <gtest/gtest.h>

// Include com_manager.h first so its include guard skips the broken
// extern-C wrap inside ISFirmwareUpdater.h that DeviceLog.h transitively
// pulls in (com_manager.h declares C++ overloads, which are illegal under
// extern "C").
#include "com_manager.h"
#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISLogFile.h"
#include "ISLogIndex.h"
#include "ISLogger.h"
#include "data_sets.h"
#include "test_data_utils.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>
#include <vector>

using namespace inertial_sense;
using namespace inertial_sense::idx;

namespace {

is_log_idx_header_t makeRoundTripHeader() {
    auto h = makeDefaultHeader(0x02010000u,
                               TimestampUnits::GpsTowMs,
                               HeaderTimeSource::PayloadToW);
    h.total_records       = 42;
    h.first_timestamp_ms  = 100ULL;
    h.last_timestamp_ms   = 99'999'999ULL;
    h.sync_point_count    = 7;
    h.flags               = IS_LOG_IDX_HDR_FLAG_FINALIZED;
    return h;
}

} // namespace

TEST(IdxLayout, StaticSizes) {
    // The on-disk layout depends on these — mirrors the static_assert
    // in the header but exercises it through gtest so we get a named
    // failure if some future struct edit breaks the discipline.
    EXPECT_EQ(sizeof(is_log_idx_header_t),    IS_LOG_IDX_HEADER_SIZE);
    EXPECT_EQ(sizeof(is_log_idx_record_v2_t), IS_LOG_IDX_RECORD_V2_SIZE);
    EXPECT_EQ(IS_LOG_IDX_HEADER_SIZE,    64u);
    EXPECT_EQ(IS_LOG_IDX_RECORD_V2_SIZE, 24u);
}

TEST(IdxRoundTrip, HeaderSerializeAndParse) {
    const auto src = makeRoundTripHeader();

    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    serializeHeader(buf, src);

    // Magic is in the first 4 bytes regardless of the rest.
    EXPECT_EQ(buf[0], 'I');
    EXPECT_EQ(buf[1], 'S');
    EXPECT_EQ(buf[2], 'I');
    EXPECT_EQ(buf[3], 'X');

    is_log_idx_header_t parsed{};
    ASSERT_EQ(parseHeader(buf, parsed), IsLogIndexResult::Ok)
        << "parse failed";

    EXPECT_EQ(parsed.version,             src.version);
    EXPECT_EQ(parsed.header_size,         src.header_size);
    EXPECT_EQ(parsed.producer_version,    src.producer_version);
    EXPECT_EQ(parsed.total_records,       src.total_records);
    EXPECT_EQ(parsed.first_timestamp_ms,  src.first_timestamp_ms);
    EXPECT_EQ(parsed.last_timestamp_ms,   src.last_timestamp_ms);
    EXPECT_EQ(parsed.sync_point_count,    src.sync_point_count);
    EXPECT_EQ(parsed.ts_units,            src.ts_units);
    EXPECT_EQ(parsed.ts_source,           src.ts_source);
    EXPECT_EQ(parsed.flags,               src.flags);
}

TEST(IdxRoundTrip, RecordSerializeAndParse) {
    const is_log_idx_record_v2_t src{
        /* timestamp */ 1'234'567'890ULL,
        /* offset    */ 0xDEADBEEFCAFEBABEULL,
        /* did       */ 0x1234,
        /* flags     */ IS_LOG_IDX_REC_FLAG_HAS_TOW,
        /* reserved  */ 0,
    };
    uint8_t buf[IS_LOG_IDX_RECORD_V2_SIZE];
    serializeRecord(buf, src);

    const auto parsed = parseRecord(buf);
    EXPECT_EQ(parsed.timestamp, src.timestamp);
    EXPECT_EQ(parsed.offset,    src.offset);
    EXPECT_EQ(parsed.did,       src.did);
    EXPECT_EQ(parsed.flags,     src.flags);
}

TEST(IdxRoundTrip, ManyRecordsViaContiguousBuffer) {
    // Simulates the actual writer: header followed by N records in a
    // single contiguous buffer. The reader walks: parseHeader at
    // offset 0, then parseRecord at offset 64, 64+24, 64+48, ....
    constexpr std::size_t N = 10;
    std::vector<uint8_t> buf(IS_LOG_IDX_HEADER_SIZE + N * IS_LOG_IDX_RECORD_V2_SIZE, 0);

    auto hdr = makeRoundTripHeader();
    hdr.total_records = N;
    serializeHeader(buf.data(), hdr);

    std::vector<is_log_idx_record_v2_t> sources;
    sources.reserve(N);
    for (std::size_t i = 0; i < N; ++i) {
        is_log_idx_record_v2_t r{};
        r.timestamp = 1000ULL + i * 100;
        r.offset    = static_cast<uint64_t>(i) * 4096ULL;
        r.did       = 0x100u + static_cast<uint32_t>(i);
        r.flags     = (i % 2 == 0) ? IS_LOG_IDX_REC_FLAG_HAS_TOW : 0;
        r.reserved  = 0;
        sources.push_back(r);
        serializeRecord(buf.data() + IS_LOG_IDX_HEADER_SIZE + i * IS_LOG_IDX_RECORD_V2_SIZE, r);
    }

    is_log_idx_header_t parsedHdr{};
    ASSERT_EQ(parseHeader(buf.data(), parsedHdr), IsLogIndexResult::Ok);
    EXPECT_EQ(parsedHdr.total_records, N);

    for (std::size_t i = 0; i < N; ++i) {
        const auto p = parseRecord(
            buf.data() + IS_LOG_IDX_HEADER_SIZE + i * IS_LOG_IDX_RECORD_V2_SIZE);
        EXPECT_EQ(p.timestamp, sources[i].timestamp);
        EXPECT_EQ(p.offset,    sources[i].offset);
        EXPECT_EQ(p.did,       sources[i].did);
        EXPECT_EQ(p.flags,     sources[i].flags);
    }
}

TEST(IdxLegacyDetection, V1FileReturnsLegacyFormat) {
    // A v1 .idx file starts with the first record's u32 `time`
    // field (host uptime delta) — definitely not "ISIX". Use a
    // representative byte pattern (0x12, 0x34, 0x56, 0x78 = 0x78563412
    // host uptime) for the first 4 bytes, with arbitrary remaining
    // bytes filling out 64.
    uint8_t v1Buf[IS_LOG_IDX_HEADER_SIZE]{};
    v1Buf[0] = 0x12;
    v1Buf[1] = 0x34;
    v1Buf[2] = 0x56;
    v1Buf[3] = 0x78;
    for (std::size_t i = 4; i < IS_LOG_IDX_HEADER_SIZE; ++i) {
        v1Buf[i] = static_cast<uint8_t>(i);
    }

    is_log_idx_header_t hdr{};
    EXPECT_EQ(parseHeader(v1Buf, hdr), IsLogIndexResult::LegacyFormat);
}

TEST(IdxMalformed, WrongMagicNonV1ByteSequence) {
    // Non-"ISIX" magic gets reported as LegacyFormat (since v1 is the
    // only recognized non-v2 format). Same code path as v1 detection.
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE]{};
    buf[0] = 'X';  buf[1] = 'X';  buf[2] = 'X';  buf[3] = 'X';
    is_log_idx_header_t hdr{};
    EXPECT_EQ(parseHeader(buf, hdr), IsLogIndexResult::LegacyFormat);
}

TEST(IdxMalformed, UnsupportedVersionFromValidMagic) {
    // Magic OK but version is something we don't know (e.g. 99 — a
    // hypothetical v99 written by a far-future SDK). Should report
    // Unsupported, distinct from LegacyFormat.
    auto h = makeRoundTripHeader();
    h.version = 99;
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    serializeHeader(buf, h);
    is_log_idx_header_t hdr{};
    EXPECT_EQ(parseHeader(buf, hdr), IsLogIndexResult::Unsupported);
}

TEST(IdxMalformed, ImpossibleHeaderSizeReportsCorrupted) {
    // header_size < 64 means the writer crashed mid-format or the
    // bytes were tampered with. v2 readers can't continue.
    auto h = makeRoundTripHeader();
    h.header_size = 32;  // < 64
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    serializeHeader(buf, h);
    is_log_idx_header_t hdr{};
    EXPECT_EQ(parseHeader(buf, hdr), IsLogIndexResult::Corrupted);
}

// ---------------------------------------------------------------------------
// File-level I/O round-trips via cISLogFile.
// These exercise the cISLogFileBase wrappers (writeHeader / writeRecord /
// readHeader / readRecord) against a real on-disk file — the path the SDK
// actually uses in production.
// ---------------------------------------------------------------------------

namespace {

// Returns a unique tmp path under /tmp. Deleted by the caller (or by
// the OS on reboot — these are smoke tests, not crash-resilient).
std::string makeTmpPath(const char* tag) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "/tmp/test_log_index_%s_%d_%ld.idx",
                  tag, ::getpid(), static_cast<long>(::time(nullptr)));
    return std::string{buf};
}

} // namespace

TEST(IdxFileIO, HeaderAndRecordRoundTripViaCISLogFile) {
    const auto path = makeTmpPath("hdrrec");
    {
        cISLogFile out(path, "wb");
        ASSERT_TRUE(out.isOpened());

        auto hdr = makeRoundTripHeader();
        hdr.total_records = 3;
        ASSERT_EQ(writeHeader(out, hdr), IsLogIndexResult::Ok);

        is_log_idx_record_v2_t r1{ 1000, 0,    0xAAA, IS_LOG_IDX_REC_FLAG_HAS_TOW, 0 };
        is_log_idx_record_v2_t r2{ 2000, 100,  0xBBB, 0,                            0 };
        is_log_idx_record_v2_t r3{ 3000, 4096, 0xCCC, IS_LOG_IDX_REC_FLAG_HAS_TOW, 0 };
        ASSERT_EQ(writeRecord(out, r1), IsLogIndexResult::Ok);
        ASSERT_EQ(writeRecord(out, r2), IsLogIndexResult::Ok);
        ASSERT_EQ(writeRecord(out, r3), IsLogIndexResult::Ok);
    }

    {
        cISLogFile in(path, "rb");
        ASSERT_TRUE(in.isOpened());
        is_log_idx_header_t hdrR{};
        ASSERT_EQ(readHeader(in, hdrR), IsLogIndexResult::Ok);
        EXPECT_EQ(hdrR.version, IS_LOG_IDX_VERSION_V2);
        EXPECT_EQ(hdrR.total_records, 3u);

        is_log_idx_record_v2_t a{};
        ASSERT_EQ(readRecord(in, a), IsLogIndexResult::Ok);
        EXPECT_EQ(a.did, 0xAAAu);
        EXPECT_EQ(a.flags, IS_LOG_IDX_REC_FLAG_HAS_TOW);

        is_log_idx_record_v2_t b{};
        ASSERT_EQ(readRecord(in, b), IsLogIndexResult::Ok);
        EXPECT_EQ(b.did, 0xBBBu);
        EXPECT_EQ(b.flags, 0u);

        is_log_idx_record_v2_t c{};
        ASSERT_EQ(readRecord(in, c), IsLogIndexResult::Ok);
        EXPECT_EQ(c.did, 0xCCCu);
        EXPECT_EQ(c.offset, 4096u);
    }
    std::remove(path.c_str());
}

TEST(IdxFileIO, TruncatedHeaderReturnsTruncated) {
    const auto path = makeTmpPath("trunc");
    // Write a file with only 32 bytes — half a header.
    {
        cISLogFile out(path, "wb");
        ASSERT_TRUE(out.isOpened());
        uint8_t halfBuf[32]{};
        halfBuf[0] = 'I'; halfBuf[1] = 'S'; halfBuf[2] = 'I'; halfBuf[3] = 'X';
        ASSERT_EQ(out.write(halfBuf, sizeof(halfBuf)), sizeof(halfBuf));
    }

    cISLogFile in(path, "rb");
    ASSERT_TRUE(in.isOpened());
    is_log_idx_header_t hdr{};
    EXPECT_EQ(readHeader(in, hdr), IsLogIndexResult::Truncated);

    std::remove(path.c_str());
}

TEST(IdxFileIO, TruncatedRecordReturnsTruncated) {
    const auto path = makeTmpPath("rectrunc");
    {
        cISLogFile out(path, "wb");
        ASSERT_TRUE(out.isOpened());
        auto hdr = makeRoundTripHeader();
        ASSERT_EQ(writeHeader(out, hdr), IsLogIndexResult::Ok);
        // Write only 12 bytes of a 24-byte record.
        uint8_t partial[12]{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
        ASSERT_EQ(out.write(partial, sizeof(partial)), sizeof(partial));
    }

    cISLogFile in(path, "rb");
    ASSERT_TRUE(in.isOpened());
    is_log_idx_header_t hdr{};
    ASSERT_EQ(readHeader(in, hdr), IsLogIndexResult::Ok);
    is_log_idx_record_v2_t rec{};
    EXPECT_EQ(readRecord(in, rec), IsLogIndexResult::Truncated);

    std::remove(path.c_str());
}

TEST(IdxRecordRange, LargeOffsetRoundTrip) {
    // v2 widened the offset field to uint64_t. Verify a value
    // >4GB (i.e. wouldn't fit in v1's u32) round-trips cleanly.
    constexpr uint64_t big_offset = 0x0000'0001'0000'0000ULL;  // 4 GiB exactly
    is_log_idx_record_v2_t src{ 1234, big_offset, 100, 0, 0 };
    uint8_t buf[IS_LOG_IDX_RECORD_V2_SIZE];
    serializeRecord(buf, src);
    auto p = parseRecord(buf);
    EXPECT_EQ(p.offset, big_offset);

    constexpr uint64_t huge_offset = 0xFFFF'FFFF'FFFF'0000ULL;
    src.offset = huge_offset;
    serializeRecord(buf, src);
    p = parseRecord(buf);
    EXPECT_EQ(p.offset, huge_offset);
}

TEST(IdxFinalize, FinalizedFlagSemantics) {
    // The FINALIZED flag is the writer's signal that total_records,
    // first_timestamp_ms, and last_timestamp_ms are authoritative.
    // Producing a header with the flag clear means the writer
    // crashed or hasn't called finalizeIndex yet — readers can fall
    // back to scanning records to reconstruct totals.
    auto h = makeDefaultHeader(0, TimestampUnits::HostUptimeMs,
                                  HeaderTimeSource::Mixed);
    EXPECT_EQ(h.flags & IS_LOG_IDX_HDR_FLAG_FINALIZED, 0u)
        << "default header must NOT have FINALIZED set — set it on close only";

    h.flags = IS_LOG_IDX_HDR_FLAG_FINALIZED;
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    serializeHeader(buf, h);
    is_log_idx_header_t p{};
    ASSERT_EQ(parseHeader(buf, p), IsLogIndexResult::Ok);
    EXPECT_EQ(p.flags & IS_LOG_IDX_HDR_FLAG_FINALIZED,
              IS_LOG_IDX_HDR_FLAG_FINALIZED);
}

// ---------------------------------------------------------------------------
// Integration test: drive the actual cDeviceLog::addIndexRecord +
// writeIndexChunk + finalizeIndex API end-to-end and verify the resulting
// .idx file is a viable v2 sidecar readable by ISLogIndex helpers. This is
// the "ISLogger generates a viable .idx alongside .raw" check Kyle asked
// for in PR #1123 review.
// ---------------------------------------------------------------------------

namespace {

// Minimal cDeviceLog subclass for testing the index-emission machinery.
// Stubs the pure virtuals so we can instantiate; exposes a helper that
// sets m_fileName + m_writeMode without going through full
// InitDeviceForWriting (we don't need a real .raw file for this test).
class TestDeviceLog : public cDeviceLog {
public:
    p_data_buf_t* ReadData() override { return nullptr; }
    void          SetSerialNumber(uint32_t serialNumber) override { m_devSerialNo = serialNumber; }
    std::string   LogFileExtention() override { return ".raw"; }

    void prepareForIndexEmissionTest(const std::string& baseFilename) {
        m_fileName = baseFilename;
        m_writeMode = true;
        m_logStartUpTime = current_uptimeMs();
    }
};

} // namespace

TEST(IdxIntegration, EndToEndViaDeviceLogApi) {
    TestDeviceLog log;
    const auto basePath = makeTmpPath("e2e");
    // Strip the .idx suffix the helper adds — addIndexRecord +
    // writeIndexChunk both append ".idx" to m_fileName themselves.
    const std::string baseNoExt = basePath.substr(0, basePath.size() - 4);
    log.prepareForIndexEmissionTest(baseNoExt);

    // Three synthetic records with distinct DIDs. cISDataMappings::Timestamp
    // returns 0.0 for these (no payload-ToW field plumbed in a synthetic
    // header), so the records take the host-uptime fallback with flags=0
    // — exactly the expected behavior the writer documents.
    auto makeHdr = [](uint32_t id, uint16_t size) {
        p_data_hdr_t h{};
        h.id = id;
        h.size = size;
        h.offset = 0;
        return h;
    };
    uint8_t fakeBuf[64]{};

    p_data_hdr_t h1 = makeHdr(DID_DEV_INFO,    sizeof(dev_info_t));
    p_data_hdr_t h2 = makeHdr(DID_INS_1,       128);
    p_data_hdr_t h3 = makeHdr(DID_GPS1_POS,    64);

    log.addIndexRecord(&h1, fakeBuf);
    log.addIndexRecord(&h2, fakeBuf);
    log.addIndexRecord(&h3, fakeBuf);

    ASSERT_TRUE(log.writeIndexChunk());
    ASSERT_TRUE(log.finalizeIndex());

    // Read the .idx file back via the same ISLogIndex helpers a
    // consumer would use.
    cISLogFile in(baseNoExt + ".idx", "rb");
    ASSERT_TRUE(in.isOpened());

    is_log_idx_header_t hdrR{};
    ASSERT_EQ(readHeader(in, hdrR), IsLogIndexResult::Ok);
    EXPECT_EQ(hdrR.version, IS_LOG_IDX_VERSION_V2);
    EXPECT_EQ(hdrR.total_records, 3u);
    EXPECT_NE(hdrR.flags & IS_LOG_IDX_HDR_FLAG_FINALIZED, 0u)
        << "finalizeIndex must set the FINALIZED flag";
    EXPECT_NE(hdrR.producer_version, 0u)
        << "producer_version must be filled from PROTOCOL_VERSION_CHAR0..3";

    is_log_idx_record_v2_t rec1{};
    ASSERT_EQ(readRecord(in, rec1), IsLogIndexResult::Ok);
    EXPECT_EQ(rec1.did, static_cast<uint32_t>(DID_DEV_INFO));

    is_log_idx_record_v2_t rec2{};
    ASSERT_EQ(readRecord(in, rec2), IsLogIndexResult::Ok);
    EXPECT_EQ(rec2.did, static_cast<uint32_t>(DID_INS_1));

    is_log_idx_record_v2_t rec3{};
    ASSERT_EQ(readRecord(in, rec3), IsLogIndexResult::Ok);
    EXPECT_EQ(rec3.did, static_cast<uint32_t>(DID_GPS1_POS));

    std::remove((baseNoExt + ".idx").c_str());
}

TEST(IdxIntegration, FinalizeWithoutAnyRecordsIsIdempotent) {
    // Opening + closing without writing any records shouldn't crash
    // and shouldn't produce a stray .idx file.
    TestDeviceLog log;
    const auto basePath = makeTmpPath("noop");
    const std::string baseNoExt = basePath.substr(0, basePath.size() - 4);
    log.prepareForIndexEmissionTest(baseNoExt);

    EXPECT_TRUE(log.finalizeIndex());  // no-op since no header written

    // No .idx file should exist.
    cISLogFile in(baseNoExt + ".idx", "rb");
    EXPECT_FALSE(in.isOpened());
}

// ---------------------------------------------------------------------------
// Full-stack end-to-end: drive cISLogger with simulated telemetry through
// the actual production logging framework and verify the .idx sidecar that
// lands alongside the .raw is internally consistent with what the reader
// sees when re-parsing the .raw stream.
//
// This is the test Kyle requested on PR #1123: "before this can be merged,
// we need to see it work in a real log — even if the log is created with
// simulated data — so long as it uses the logging framework. Not just
// timestamp and indexes, but the whole workflow."
// ---------------------------------------------------------------------------
TEST(IdxIntegration, ISLoggerEndToEndProducesViableIdx) {
    using namespace std;

    // Generate ~1MB of synthetic but real-shaped telemetry — same utility
    // that test_ISLogger.cpp::logReader_raw uses, so we know the writer
    // path through cISLogger has been exercised on this data shape.
    list<vector<uint8_t>*> messages;
    GenerateRawLogData(messages, 1.0f);
    ASSERT_FALSE(messages.empty());

    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_log_idx_e2e_%d_%ld",
                  ::getpid(), static_cast<long>(::time(nullptr)));
    const string logPath = dirBuf;
    ISFileManager::DeleteDirectory(logPath);

    // -- Write phase: feed packets through cISLogger (the real framework).
    {
        cISLogger logger;
        cISLogger::sSaveOptions options;
        options.logType = cISLogger::LOGTYPE_RAW;
        options.useSubFolderTimestamp = false;
        ASSERT_TRUE(logger.InitSave(logPath, options));
        auto devLogger = logger.registerDevice(
            ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0), 12345u);
        ASSERT_TRUE(devLogger != nullptr);
        logger.EnableLogging(true);

        for (auto* msg : messages) {
            ASSERT_TRUE(logger.LogData(devLogger, msg->size(),
                                       reinterpret_cast<const uint8_t*>(msg->data())));
        }
        logger.CloseAllFiles();
    }

    // -- Locate the .raw + .idx files the framework actually emitted.
    vector<ISFileManager::file_info_t> rawFiles, idxFiles;
    ISFileManager::GetAllFilesInDirectory(logPath, true, "\\.raw$", rawFiles);
    ISFileManager::GetAllFilesInDirectory(logPath, true, "\\.idx$", idxFiles);
    ASSERT_GE(rawFiles.size(), 1u) << "writer did not produce a .raw file";
    ASSERT_EQ(idxFiles.size(), rawFiles.size())
        << "every .raw segment should have a matching .idx sidecar";

    // -- Read all index records out of every .idx file (in alphabetical
    // segment order — same order the reader walks them).
    std::sort(idxFiles.begin(), idxFiles.end(),
              [](const ISFileManager::file_info_t& a,
                 const ISFileManager::file_info_t& b) {
                  return a.name < b.name;
              });

    vector<is_log_idx_record_v2_t> allIdxRecords;
    for (const auto& idxInfo : idxFiles) {
        cISLogFile in(idxInfo.name, "rb");
        ASSERT_TRUE(in.isOpened()) << "could not open " << idxInfo.name;

        is_log_idx_header_t hdrR{};
        ASSERT_EQ(readHeader(in, hdrR), IsLogIndexResult::Ok)
            << idxInfo.name;
        EXPECT_EQ(hdrR.version, IS_LOG_IDX_VERSION_V2)
            << idxInfo.name << ": framework must emit v2 sidecar";
        EXPECT_NE(hdrR.flags & IS_LOG_IDX_HDR_FLAG_FINALIZED, 0u)
            << idxInfo.name << ": clean shutdown via CloseAllFiles must "
                                "set FINALIZED";
        EXPECT_NE(hdrR.producer_version, 0u)
            << idxInfo.name << ": producer_version must be filled in";
        EXPECT_GT(hdrR.total_records, 0u)
            << idxInfo.name << ": some ISB packets should have been indexed";

        for (uint32_t i = 0; i < hdrR.total_records; ++i) {
            is_log_idx_record_v2_t recR{};
            ASSERT_EQ(readRecord(in, recR), IsLogIndexResult::Ok)
                << idxInfo.name << " record " << i;
            allIdxRecords.push_back(recR);
        }
    }
    ASSERT_FALSE(allIdxRecords.empty());

    // -- Walk the .raw stream back through the same cISLogger reader path
    // a consumer would use, collect every ISB packet's DID in order.
    // Bound the iteration by the number of source messages — at most one
    // packet per message — to mirror the model in test_ISLogger.cpp's
    // logReader_raw, which doesn't rely on ReadNextPacket returning null
    // at EOF (the reader holds onto a static buffer past stream-end).
    cISLogger reader;
    ASSERT_TRUE(reader.LoadFromDirectory(logPath, cISLogger::LOGTYPE_RAW));
    reader.ShowParseErrors(true);

    vector<uint16_t> readerIsbDids;
    size_t deviceIndex = 0;
    for (size_t k = 0; k < messages.size(); ++k) {
        protocol_type_t pt = _PTYPE_NONE;
        packet_t* pkt = reader.ReadNextPacket(pt, deviceIndex);
        if (pkt == nullptr) break;
        if (pt == _PTYPE_INERTIAL_SENSE_DATA || pt == _PTYPE_INERTIAL_SENSE_CMD) {
            readerIsbDids.push_back(pkt->dataHdr.id);
        }
    }
    reader.CloseAllFiles();

    // -- The .idx must mirror the ISB packets the reader sees in count + order.
    // This is the "the whole workflow" check: the writer's per-packet
    // addIndexRecord (DeviceLogRaw.cpp parser-loop fix) and the reader's
    // ISB-packet stream agree byte-for-byte on what got logged.
    ASSERT_EQ(allIdxRecords.size(), readerIsbDids.size())
        << ".idx record count must equal the number of ISB packets the "
           "reader extracts from the .raw";
    for (size_t i = 0; i < allIdxRecords.size(); ++i) {
        EXPECT_EQ(allIdxRecords[i].did,
                  static_cast<uint32_t>(readerIsbDids[i]))
            << "DID mismatch at index record " << i;
    }

    // -- Sanity: at least one record carries a payload-ToW timestamp
    // (HAS_TOW flag set). GenerateRawLogData produces DID_INS_2 / GPS
    // messages with real GPS-ToW fields, which the writer's payload-ToW
    // path is supposed to capture.
    bool sawTowFlag = false;
    for (const auto& r : allIdxRecords) {
        if (r.flags & IS_LOG_IDX_REC_FLAG_HAS_TOW) { sawTowFlag = true; break; }
    }
    EXPECT_TRUE(sawTowFlag)
        << "expected at least one indexed record to carry a payload-ToW "
           "timestamp from a GPS/INS DID";

    // -- One-line summary of what the framework actually produced.
    std::printf("[idx-e2e] %zu source msgs -> %zu .raw seg(s), %zu .idx record(s), "
                "%zu reader-ISB packets matched, sawTowFlag=%d\n",
                messages.size(), rawFiles.size(), allIdxRecords.size(),
                readerIsbDids.size(), sawTowFlag ? 1 : 0);

    // -- Cleanup
    for (auto* msg : messages) delete msg;
    ISFileManager::DeleteDirectory(logPath);
}

TEST(IdxLittleEndian, ByteOrderIsExplicit) {
    // Sanity: a known value at a known field should produce the
    // expected little-endian bytes regardless of host endianness.
    is_log_idx_header_t h{};
    h.magic[0] = 'I'; h.magic[1] = 'S'; h.magic[2] = 'I'; h.magic[3] = 'X';
    h.version             = 0x0201;  // bytes 4..5 should read 01 02
    h.header_size         = IS_LOG_IDX_HEADER_SIZE;
    h.producer_version    = 0xCAFEBABEu;  // bytes 8..11 should read BE BA FE CA
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    serializeHeader(buf, h);
    EXPECT_EQ(buf[4], 0x01);
    EXPECT_EQ(buf[5], 0x02);
    EXPECT_EQ(buf[8],  0xBE);
    EXPECT_EQ(buf[9],  0xBA);
    EXPECT_EQ(buf[10], 0xFE);
    EXPECT_EQ(buf[11], 0xCA);
}

// ---------------------------------------------------------------------------
// Multi-segment regression: when the .raw exceeds maxFileSize the framework
// rotates to a new segment. Each segment must get its own valid .idx sidecar
// — its own header, its own per-segment record count, and intact records
// (the prior bug was that segment N+1 inherited segment N's m_idxHeaderWritten
// flag and m_idxTotalRecords counter, so the second segment's writeIndexChunk
// skipped the header write, and finalizeIndex later overwrote the first ~3
// records with a [stale-totaled] header at offset 0).
// ---------------------------------------------------------------------------
TEST(IdxIntegration, ISLoggerMultiSegmentRotationProducesValidIdxPerSegment) {
    using namespace std;

    list<vector<uint8_t>*> messages;
    GenerateRawLogData(messages, 8.0f);  // ~8 MB ⇒ 2+ default-size segments
    ASSERT_FALSE(messages.empty());

    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_log_idx_multiseg_%d_%ld",
                  ::getpid(), static_cast<long>(::time(nullptr)));
    const string logPath = dirBuf;
    ISFileManager::DeleteDirectory(logPath);

    {
        cISLogger logger;
        cISLogger::sSaveOptions options;
        options.logType = cISLogger::LOGTYPE_RAW;
        options.useSubFolderTimestamp = false;
        // Force segment rotation by setting a small max file size.
        options.maxFileSize = 2u * 1024u * 1024u;  // 2 MB per segment
        ASSERT_TRUE(logger.InitSave(logPath, options));
        auto devLogger = logger.registerDevice(
            ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0), 12345u);
        logger.EnableLogging(true);
        for (auto* msg : messages) {
            ASSERT_TRUE(logger.LogData(devLogger, msg->size(),
                                       reinterpret_cast<const uint8_t*>(msg->data())));
        }
        logger.CloseAllFiles();
    }

    vector<ISFileManager::file_info_t> rawFiles, idxFiles;
    ISFileManager::GetAllFilesInDirectory(logPath, true, "\\.raw$", rawFiles);
    ISFileManager::GetAllFilesInDirectory(logPath, true, "\\.idx$", idxFiles);
    ASSERT_GE(rawFiles.size(), 2u)
        << "8 MB at 2 MB/segment should rotate to multiple segments";
    ASSERT_EQ(idxFiles.size(), rawFiles.size())
        << "every .raw segment needs its own .idx sidecar";

    // Each .idx must independently parse as a valid v2 file: header
    // present, FINALIZED, total_records consistent with on-disk size,
    // and every record reads back without short-read.
    for (const auto& f : idxFiles) {
        cISLogFile in(f.name, "rb");
        ASSERT_TRUE(in.isOpened()) << f.name;
        is_log_idx_header_t hdrR{};
        ASSERT_EQ(readHeader(in, hdrR), IsLogIndexResult::Ok) << f.name;
        EXPECT_EQ(hdrR.version, IS_LOG_IDX_VERSION_V2) << f.name;
        EXPECT_NE(hdrR.flags & IS_LOG_IDX_HDR_FLAG_FINALIZED, 0u)
            << f.name << ": each segment must be finalized";

        const size_t expectedFromSize =
            (f.size - IS_LOG_IDX_HEADER_SIZE) / IS_LOG_IDX_RECORD_V2_SIZE;
        EXPECT_EQ(hdrR.total_records, expectedFromSize)
            << f.name << ": header total_records must match on-disk record count "
                         "(= file_size - header) / record_size";

        for (uint32_t i = 0; i < hdrR.total_records; ++i) {
            is_log_idx_record_v2_t rec{};
            ASSERT_EQ(readRecord(in, rec), IsLogIndexResult::Ok)
                << f.name << " record " << i << "/" << hdrR.total_records;
        }
    }

    for (auto* msg : messages) delete msg;
    ISFileManager::DeleteDirectory(logPath);
}

