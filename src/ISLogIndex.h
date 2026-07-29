/**
 * @file ISLogIndex.h
 * @brief `.idx` v2 sidecar format — header layout, per-record struct,
 *        writer / reader helpers.
 *
 * D-01 / SN-7879 / D0017 / D0023 / D0043 / D0049: replaces the coarse
 * v1 `.idx` (host-time + record-counter, 16 bytes per record) with a
 * versioned format that carries the actual DID and a payload-derived
 * timestamp per record. Consumers can do per-DID time-range queries
 * directly against the index.
 *
 * On-disk layout is **little-endian** regardless of host byte-order
 * (the SDK targets ARM embedded). All multi-byte fields go through
 * the explicit serialize/parse helpers in `ISLogIndex.cpp` — never
 * memcpy a struct directly to disk.
 *
 * Versioning discipline: once v2 ships, its on-disk layout is
 * immutable. Any future change becomes v3 with its own version
 * number; v2 readers stay correct on v2 files forever.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISError.h"

#include <cstddef>
#include <cstdint>

class cISLogFileBase;  // forward — defined in ISLogFileBase.h

namespace inertial_sense {
namespace idx {

// ----- File identification --------------------------------------------------

/// 4-byte ASCII magic at the start of every v2+ `.idx` file. v1 files
/// have no magic — first 4 bytes are the start of the first record's
/// `time` field (a `uint32_t`), so missing magic is the v1 signal.
inline constexpr char IS_LOG_IDX_MAGIC[4] = { 'I', 'S', 'I', 'X' };

/// Format version stored in the header. v2 = this format.
inline constexpr uint16_t IS_LOG_IDX_VERSION_V2 = 2;

// ----- Header / record sizes (also static_assert'd below) ------------------

inline constexpr std::size_t IS_LOG_IDX_HEADER_SIZE = 64;
inline constexpr std::size_t IS_LOG_IDX_RECORD_V2_SIZE = 24;

/// SN-8383: v2.1 grows the per-record entry by a trailing `local_uptime_ms`
/// (+ pad) — appended at the END so the 24-byte v2.0 prefix is byte-identical
/// and a v2.1 reader can still stride v2.0 files (which lack the trailing
/// field). The writer stamps v2.1 records; the on-disk record size lives in the
/// header (`record_size`) so the reader strides correctly for either.
inline constexpr std::size_t IS_LOG_IDX_RECORD_V2_1_SIZE = 32;

// ----- Timestamp interpretation enums --------------------------------------

/// What `is_log_idx_record_v2_t::timestamp` means in this file. Stored
/// in the header so a v3 with nanosecond-precision logs (or some
/// other unit) doesn't re-break the world.
enum class TimestampUnits : uint8_t {
    HostUptimeMs   = 0,  ///< Milliseconds since `m_logStartUpTime` on the writer host.
    GpsTowMs       = 1,  ///< GPS time-of-week in ms (resets every Sunday 00:00:00 UTC).
    UnixEpochMs    = 2,  ///< Milliseconds since 1970-01-01T00:00:00Z.
    Mixed          = 3,  ///< Mixed across records — readers must check per-record `flags`.
};

/// The dominant `TimeSource` for records in this file (per-record
/// flags bit 0 still tells you whether the individual record was a
/// real ToW). Mirrors `inertial_sense::TimeSource` from D-06.
enum class HeaderTimeSource : uint8_t {
    PayloadToW       = 0,
    ResolvedViaSync  = 1,
    HostReceived     = 2,
    SessionOnly      = 3,
    Mixed            = 4,
};

// ----- Record flag bits ----------------------------------------------------

/// Bit 0: this record's payload carried a real GPS time-of-week field
/// (i.e. it can be used as a sync anchor by `ISTimeResolver`, D-07).
inline constexpr uint16_t IS_LOG_IDX_REC_FLAG_HAS_TOW = 1u << 0;

// ----- Header flag bits ----------------------------------------------------

/// Bit 0: `total_records` / `first_timestamp_ms` / `last_timestamp_ms`
/// were rewritten on close and are authoritative. If unset, the writer
/// crashed or hasn't finalized yet — readers can scan records to
/// reconstruct the totals.
inline constexpr uint8_t IS_LOG_IDX_HDR_FLAG_FINALIZED = 1u << 0;

/// Bit 1 (SN-8383, v2.1): per-record entries carry the trailing
/// `local_uptime_ms` field (record_size == 32). Absent ⇒ plain v2.0 (24-byte
/// records, no local delta).
inline constexpr uint8_t IS_LOG_IDX_HDR_FLAG_HAS_LOCAL_DELTA = 1u << 1;

/// Bit 2 (SN-8340, v2.1): `capture_epoch_ms` in the header is set — a durable
/// absolute host wall-clock (ms since Unix epoch) captured at log-open, so a
/// log that never acquires GPS still has a real wall-clock anchor.
inline constexpr uint8_t IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH = 1u << 2;

// ----- Structs (logical, not on-disk) --------------------------------------
//
// These mirror the on-disk layout 1:1, but we never serialize them via
// memcpy — `serialize*` / `parse*` below do explicit little-endian
// byte writes / reads. The struct is just the logical container.

#pragma pack(push, 1)

/**
 * @brief Fixed-size header at offset 0 of every v2 `.idx` file.
 *
 * 64 bytes total. `header_size` lets future v3+ readers ignore unknown
 * trailing fields gracefully — a v2 reader stops at byte 64 and
 * doesn't misinterpret v3 additions.
 */
struct is_log_idx_header_t {
    char     magic[4];              ///<  0..3 : `"ISIX"`
    uint16_t version;               ///<  4..5 : 2 = v2
    uint16_t header_size;           ///<  6..7 : on-disk header size in bytes (= 64 for v2)
    uint32_t producer_version;      ///<  8..11: ENCODE_VERSION-style value of the writing SDK
    uint64_t total_records;         ///< 12..19: count of records following the header (0 if not finalized)
    uint64_t first_timestamp_ms;    ///< 20..27: first record's `timestamp` (0 if unset)
    uint64_t last_timestamp_ms;     ///< 28..35: last record's `timestamp`  (0 if unset)
    uint32_t sync_point_count;      ///< 36..39: D-07 sync-event count tracked by writer (0 default)
    uint8_t  ts_units;              ///< 40    : `TimestampUnits`
    uint8_t  ts_source;             ///< 41    : `HeaderTimeSource`
    uint8_t  flags;                 ///< 42    : `IS_LOG_IDX_HDR_FLAG_*`
    uint8_t  reserved8;             ///< 43    : pad
    uint16_t record_size;           ///< 44..45: on-disk per-record size in bytes (24=v2.0, 32=v2.1). 0 (pre-v2.1 header) ⇒ reader treats as 24. SN-8383.
    uint16_t reserved16;            ///< 46..47: pad
    uint64_t capture_epoch_ms;      ///< 48..55: absolute host wall-clock ms at log-open (0 = unset; valid only when HAS_CAPTURE_EPOCH). SN-8340.
    uint8_t  reserved[8];           ///< 56..63: explicit pad to 64 bytes
};

/**
 * @brief Per-record entry in the body of a v2 `.idx` file.
 *
 * 24 bytes total. v2 stores meaningful payload-derived timestamps and
 * the actual DID, so `(did, ts_lo, ts_hi)` queries can binary-search
 * the index without touching the `.raw` segment.
 */
struct is_log_idx_record_v2_t {
    uint64_t timestamp;             ///<  0..7 : payload-derived ms (units per header `ts_units`)
    uint64_t offset;                ///<  8..15: byte offset into the `.raw` segment
    uint32_t did;                   ///< 16..19: data ID; 0 = no associated DID (raw stream)
    uint16_t flags;                 ///< 20..21: `IS_LOG_IDX_REC_FLAG_*`
    uint16_t reserved;              ///< 22..23: pad  — end of the v2.0 (24-byte) prefix
    // ----- v2.1 trailing fields (present when header record_size == 32) -----
    uint32_t local_uptime_ms;       ///< 24..27: host-uptime-since-log-start ms for THIS record, stamped by the writer for EVERY record (SN-8383). 0 on v2.0 records.
    uint32_t reserved2;             ///< 28..31: pad to an 8-byte multiple
};

#pragma pack(pop)

static_assert(sizeof(is_log_idx_header_t) == IS_LOG_IDX_HEADER_SIZE,
              "is_log_idx_header_t must be exactly 64 bytes — pack discipline.");
static_assert(sizeof(is_log_idx_record_v2_t) == IS_LOG_IDX_RECORD_V2_1_SIZE,
              "is_log_idx_record_v2_t must be exactly 32 bytes (v2.1) — pack discipline.");
static_assert(IS_LOG_IDX_RECORD_V2_SIZE == 24,
              "v2.0 on-disk record prefix stays 24 bytes.");

// ----- Pure serialize / parse helpers --------------------------------------
//
// These work on raw byte buffers and are the canonical primitives.
// File-level wrappers below are thin and just call these. Tests
// exercise these directly — no I/O needed for round-trip checks.

/**
 * @brief Serialize a header to a 64-byte little-endian buffer.
 *
 * Always writes exactly `IS_LOG_IDX_HEADER_SIZE` bytes; out buffer
 * must be at least that large. The caller is responsible for sizing.
 * Always succeeds — there's no failure mode at this layer.
 */
void serializeHeader(uint8_t out[IS_LOG_IDX_HEADER_SIZE],
                     const is_log_idx_header_t& hdr) noexcept;

/**
 * @brief Parse a 64-byte little-endian buffer into a header.
 *
 * Returns `LegacyFormat` if the magic is absent (almost certainly v1),
 * `Unsupported` if the magic is present but the version isn't v2, and
 * `Corrupted` if `header_size` is impossible (< minimum). Callers
 * pattern-match on the error code to fall back to a v1 reader when
 * `LegacyFormat` is reported.
 */
ISExpected<is_log_idx_header_t> parseHeader(
    const uint8_t in[IS_LOG_IDX_HEADER_SIZE]) noexcept;

/**
 * @brief Serialize a record to a 32-byte v2.1 little-endian buffer.
 *
 * Always writes exactly `IS_LOG_IDX_RECORD_V2_1_SIZE` bytes (the full v2.1
 * layout, including the trailing `local_uptime_ms`). All newly-written `.idx`
 * files are v2.1-format; `record_size` in the header tells readers the stride.
 */
void serializeRecord(uint8_t out[IS_LOG_IDX_RECORD_V2_1_SIZE],
                     const is_log_idx_record_v2_t& rec) noexcept;

/**
 * @brief Parse a little-endian buffer into a record.
 *
 * Reads the 24-byte v2.0 prefix always; reads the trailing `local_uptime_ms`
 * (SN-8383) only when `record_size >= IS_LOG_IDX_RECORD_V2_1_SIZE` (else it's
 * left 0). `record_size` comes from the header (`hdr.record_size`, 0 ⇒ legacy
 * 24). Pure layout decode — never fails; caller ensures `in` holds at least
 * `record_size` bytes.
 */
is_log_idx_record_v2_t parseRecord(
    const uint8_t* in,
    std::size_t record_size = IS_LOG_IDX_RECORD_V2_1_SIZE) noexcept;

// ----- File-level wrappers (cISLogFileBase) --------------------------------

/**
 * @brief Write a header to an open `cISLogFileBase`.
 * @return `Ok` on success, `Io` if the underlying write didn't consume
 *         the full 64 bytes.
 */
ISExpected<void> writeHeader(cISLogFileBase& file, const is_log_idx_header_t& hdr);

/**
 * @brief Append a single v2 record to an open `cISLogFileBase`.
 * @return `Ok` on success, `Io` on short write.
 */
ISExpected<void> writeRecord(cISLogFileBase& file, const is_log_idx_record_v2_t& rec);

/**
 * @brief Read and validate a header from an open `cISLogFileBase`.
 *
 * Reads exactly `IS_LOG_IDX_HEADER_SIZE` bytes from the file's current
 * position, then defers to `parseHeader` for validation. Errors:
 *
 * - `Truncated` — file shorter than 64 bytes.
 * - `LegacyFormat` — bytes parsed but magic absent (v1 file).
 * - `Unsupported` — magic present but version isn't v2.
 * - `Corrupted` — magic + version OK but `header_size` is impossible.
 * - `Io` — underlying read returned an unexpected count.
 */
ISExpected<is_log_idx_header_t> readHeader(cISLogFileBase& file);

/**
 * @brief Read a single record (`record_size` bytes) from the current file
 *        position. Pass `hdr.record_size` (0 ⇒ legacy 24) so v2.0 and v2.1
 *        files both stride correctly.
 * @return `Truncated` if fewer than `record_size` bytes remained; `Io` on
 *         underlying read error.
 */
ISExpected<is_log_idx_record_v2_t> readRecord(
    cISLogFileBase& file,
    std::size_t record_size = IS_LOG_IDX_RECORD_V2_1_SIZE);

// ----- Convenience constructors --------------------------------------------

/**
 * @brief Build a header with sensible defaults for the `.idx` writer.
 *
 * `total_records`, `first_timestamp_ms`, `last_timestamp_ms`, and
 * `sync_point_count` are all 0 — the writer fills them in when it
 * finalizes the file via a seek(0) + rewrite-header dance. `flags`
 * starts cleared; the writer sets `FINALIZED` only on clean close.
 */
is_log_idx_header_t makeDefaultHeader(uint32_t producer_version,
                                      TimestampUnits units,
                                      HeaderTimeSource source,
                                      uint64_t capture_epoch_ms = 0) noexcept;

} // namespace idx
} // namespace inertial_sense
