// D-01 / SN-7879 smoke tests for the .idx v2 sidecar format.
//
// Coverage:
//   * Round-trip — serialize a header + N records, parse them back,
//     all fields match.
//   * Legacy detection — synthetic v1 byte stream (no magic) must
//     report ISErrorCode::LegacyFormat, not silently misparse.
//   * Malformed — wrong magic / unsupported version / impossible
//     header_size each return a distinct error code.
//   * Layout discipline — sizeof / static asserts.
//
// File-level wrappers (cISLogFileBase) aren't exercised here; their
// only job is to copy bytes to/from the underlying file, and the pure
// serialize/parse layer that does the actual work is covered above.

#include <gtest/gtest.h>

#include "ISLogIndex.h"

#include <array>
#include <cstdint>
#include <cstring>
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

    auto parsed = parseHeader(buf);
    ASSERT_TRUE(parsed.has_value()) << "parse failed: " << parsed.error().message;

    EXPECT_EQ(parsed->version,             src.version);
    EXPECT_EQ(parsed->header_size,         src.header_size);
    EXPECT_EQ(parsed->producer_version,    src.producer_version);
    EXPECT_EQ(parsed->total_records,       src.total_records);
    EXPECT_EQ(parsed->first_timestamp_ms,  src.first_timestamp_ms);
    EXPECT_EQ(parsed->last_timestamp_ms,   src.last_timestamp_ms);
    EXPECT_EQ(parsed->sync_point_count,    src.sync_point_count);
    EXPECT_EQ(parsed->ts_units,            src.ts_units);
    EXPECT_EQ(parsed->ts_source,           src.ts_source);
    EXPECT_EQ(parsed->flags,               src.flags);
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

    auto parsedHdr = parseHeader(buf.data());
    ASSERT_TRUE(parsedHdr.has_value());
    EXPECT_EQ(parsedHdr->total_records, N);

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

    auto r = parseHeader(v1Buf);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::LegacyFormat);
}

TEST(IdxMalformed, WrongMagicNonV1ByteSequence) {
    // Non-"ISIX" magic gets reported as LegacyFormat (since v1 is the
    // only recognized non-v2 format). Same code path as v1 detection.
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE]{};
    buf[0] = 'X';  buf[1] = 'X';  buf[2] = 'X';  buf[3] = 'X';
    auto r = parseHeader(buf);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::LegacyFormat);
}

TEST(IdxMalformed, UnsupportedVersionFromValidMagic) {
    // Magic OK but version is something we don't know (e.g. 99 — a
    // hypothetical v99 written by a far-future SDK). Should report
    // Unsupported, distinct from LegacyFormat.
    auto h = makeRoundTripHeader();
    h.version = 99;
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    serializeHeader(buf, h);
    auto r = parseHeader(buf);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::Unsupported);
}

TEST(IdxMalformed, ImpossibleHeaderSizeReportsCorrupted) {
    // header_size < 64 means the writer crashed mid-format or the
    // bytes were tampered with. v2 readers can't continue.
    auto h = makeRoundTripHeader();
    h.header_size = 32;  // < 64
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    serializeHeader(buf, h);
    auto r = parseHeader(buf);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::Corrupted);
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
