/**
 * @file ISLogIndex.cpp
 * @brief Pure little-endian serialize/parse + cISLogFileBase wrappers
 *        for the `.idx` v2 format.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISLogIndex.h"

#include "ISLogFileBase.h"

#include <cstring>

namespace inertial_sense {
namespace idx {

// ----- Byte-order helpers ---------------------------------------------------
//
// All on-disk fields are little-endian. We never memcpy structs to
// disk — that would inherit the host byte order, which fails on BE
// embedded targets. These helpers do shift-and-mask reads/writes,
// which the compiler optimizes back to single instructions on LE
// hosts and stay correct on BE.

namespace {

void put_u16(uint8_t* p, uint16_t v) noexcept {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

void put_u32(uint8_t* p, uint32_t v) noexcept {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

void put_u64(uint8_t* p, uint64_t v) noexcept {
    for (int i = 0; i < 8; ++i) {
        p[i] = static_cast<uint8_t>((v >> (i * 8)) & 0xFFu);
    }
}

uint16_t get_u16(const uint8_t* p) noexcept {
    return static_cast<uint16_t>(p[0])
         | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t get_u32(const uint8_t* p) noexcept {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

uint64_t get_u64(const uint8_t* p) noexcept {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
        v |= (static_cast<uint64_t>(p[i]) << (i * 8));
    }
    return v;
}

} // namespace

// ----- Pure serialize / parse ----------------------------------------------

void serializeHeader(uint8_t out[IS_LOG_IDX_HEADER_SIZE],
                     const is_log_idx_header_t& hdr) noexcept {
    // Zero the buffer so reserved bytes are deterministic on disk.
    std::memset(out, 0, IS_LOG_IDX_HEADER_SIZE);

    out[0] = static_cast<uint8_t>(hdr.magic[0]);
    out[1] = static_cast<uint8_t>(hdr.magic[1]);
    out[2] = static_cast<uint8_t>(hdr.magic[2]);
    out[3] = static_cast<uint8_t>(hdr.magic[3]);
    put_u16(out +  4, hdr.version);
    put_u16(out +  6, hdr.header_size);
    put_u32(out +  8, hdr.producer_version);
    put_u64(out + 12, hdr.total_records);
    put_u64(out + 20, hdr.first_timestamp_ms);
    put_u64(out + 28, hdr.last_timestamp_ms);
    put_u32(out + 36, hdr.sync_point_count);
    out[40] = hdr.ts_units;
    out[41] = hdr.ts_source;
    out[42] = hdr.flags;
    out[43] = hdr.reserved8;
    // out[44..63] left zero from memset.
}

ISExpected<is_log_idx_header_t> parseHeader(
    const uint8_t in[IS_LOG_IDX_HEADER_SIZE]) noexcept {
    // First 4 bytes are the v1/v2 discriminator. v1 had no magic; its
    // first 4 bytes are the start of the first record's u32 `time`
    // field, which is essentially never going to read as "ISIX" by
    // accident. So: magic mismatch → assume v1.
    if (in[0] != 'I' || in[1] != 'S' || in[2] != 'I' || in[3] != 'X') {
        return fail(ISErrorCode::LegacyFormat,
                    "v1 .idx — no v2 magic at offset 0");
    }

    is_log_idx_header_t hdr{};
    hdr.magic[0] = static_cast<char>(in[0]);
    hdr.magic[1] = static_cast<char>(in[1]);
    hdr.magic[2] = static_cast<char>(in[2]);
    hdr.magic[3] = static_cast<char>(in[3]);
    hdr.version             = get_u16(in +  4);
    hdr.header_size         = get_u16(in +  6);
    hdr.producer_version    = get_u32(in +  8);
    hdr.total_records       = get_u64(in + 12);
    hdr.first_timestamp_ms  = get_u64(in + 20);
    hdr.last_timestamp_ms   = get_u64(in + 28);
    hdr.sync_point_count    = get_u32(in + 36);
    hdr.ts_units            = in[40];
    hdr.ts_source           = in[41];
    hdr.flags               = in[42];
    hdr.reserved8           = in[43];
    std::memcpy(hdr.reserved, in + 44, sizeof(hdr.reserved));

    if (hdr.version != IS_LOG_IDX_VERSION_V2) {
        return fail(ISErrorCode::Unsupported,
                    "magic OK but version != 2 (probably v3+ written by newer SDK)");
    }
    if (hdr.header_size < IS_LOG_IDX_HEADER_SIZE) {
        // A v2 reader REQUIRES at least 64 bytes of header. Less than
        // that means the bytes are corrupted or truncated mid-write.
        return fail(ISErrorCode::Corrupted,
                    "header_size < 64 — corrupted header");
    }
    return hdr;
}

void serializeRecord(uint8_t out[IS_LOG_IDX_RECORD_V2_SIZE],
                     const is_log_idx_record_v2_t& rec) noexcept {
    put_u64(out +  0, rec.timestamp);
    put_u64(out +  8, rec.offset);
    put_u32(out + 16, rec.did);
    put_u16(out + 20, rec.flags);
    put_u16(out + 22, rec.reserved);
}

is_log_idx_record_v2_t parseRecord(
    const uint8_t in[IS_LOG_IDX_RECORD_V2_SIZE]) noexcept {
    is_log_idx_record_v2_t rec{};
    rec.timestamp = get_u64(in +  0);
    rec.offset    = get_u64(in +  8);
    rec.did       = get_u32(in + 16);
    rec.flags     = get_u16(in + 20);
    rec.reserved  = get_u16(in + 22);
    return rec;
}

// ----- cISLogFileBase wrappers ---------------------------------------------

ISExpected<void> writeHeader(cISLogFileBase& file, const is_log_idx_header_t& hdr) {
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    serializeHeader(buf, hdr);
    const std::size_t written = file.write(buf, IS_LOG_IDX_HEADER_SIZE);
    if (written != IS_LOG_IDX_HEADER_SIZE) {
        return fail(ISErrorCode::Io, "short write on .idx header");
    }
    return {};
}

ISExpected<void> writeRecord(cISLogFileBase& file, const is_log_idx_record_v2_t& rec) {
    uint8_t buf[IS_LOG_IDX_RECORD_V2_SIZE];
    serializeRecord(buf, rec);
    const std::size_t written = file.write(buf, IS_LOG_IDX_RECORD_V2_SIZE);
    if (written != IS_LOG_IDX_RECORD_V2_SIZE) {
        return fail(ISErrorCode::Io, "short write on .idx record");
    }
    return {};
}

ISExpected<is_log_idx_header_t> readHeader(cISLogFileBase& file) {
    uint8_t buf[IS_LOG_IDX_HEADER_SIZE];
    const std::size_t got = file.read(buf, IS_LOG_IDX_HEADER_SIZE);
    if (got < IS_LOG_IDX_HEADER_SIZE) {
        return fail(ISErrorCode::Truncated,
                    "fewer than 64 bytes available — truncated or empty .idx");
    }
    return parseHeader(buf);
}

ISExpected<is_log_idx_record_v2_t> readRecord(cISLogFileBase& file) {
    uint8_t buf[IS_LOG_IDX_RECORD_V2_SIZE];
    const std::size_t got = file.read(buf, IS_LOG_IDX_RECORD_V2_SIZE);
    if (got < IS_LOG_IDX_RECORD_V2_SIZE) {
        return fail(ISErrorCode::Truncated,
                    "short read on .idx record");
    }
    return parseRecord(buf);
}

is_log_idx_header_t makeDefaultHeader(uint32_t producer_version,
                                      TimestampUnits units,
                                      HeaderTimeSource source) noexcept {
    is_log_idx_header_t hdr{};
    hdr.magic[0] = IS_LOG_IDX_MAGIC[0];
    hdr.magic[1] = IS_LOG_IDX_MAGIC[1];
    hdr.magic[2] = IS_LOG_IDX_MAGIC[2];
    hdr.magic[3] = IS_LOG_IDX_MAGIC[3];
    hdr.version             = IS_LOG_IDX_VERSION_V2;
    hdr.header_size         = IS_LOG_IDX_HEADER_SIZE;
    hdr.producer_version    = producer_version;
    hdr.total_records       = 0;
    hdr.first_timestamp_ms  = 0;
    hdr.last_timestamp_ms   = 0;
    hdr.sync_point_count    = 0;
    hdr.ts_units            = static_cast<uint8_t>(units);
    hdr.ts_source           = static_cast<uint8_t>(source);
    hdr.flags               = 0;  // FINALIZED set on clean close
    hdr.reserved8           = 0;
    std::memset(hdr.reserved, 0, sizeof(hdr.reserved));
    return hdr;
}

} // namespace idx
} // namespace inertial_sense
