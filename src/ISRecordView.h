/**
 * @file ISRecordView.h
 * @brief Non-owning view of a single record in an ISLogReader segment.
 *
 * D-02 / SN-7893 / D0021 / D0022: the type-erased core of the new
 * SDK reader yields `ISRecordView` values. Views point directly into
 * the segment's mmap'd region — no allocation per record. Use
 * `ISRecordView::owned()` to materialize an `OwnedRecord` whose
 * lifetime is independent of the reader.
 *
 * Header-only. The struct is trivially-copyable (POD-ish — pointer +
 * counters); the `owned()` method is the one that allocates.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISTimeStamp.h"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace inertial_sense {

class OwnedRecord;  // forward — defined below.

/**
 * @brief Non-owning view of a single record's metadata + payload bytes.
 *
 * Lifetime: valid as long as the parent `ISLogReader` is alive AND its
 * mmap'd region hasn't been moved out from under the view (the reader
 * is move-only; moving it invalidates outstanding views). Trivially-
 * copyable; pass by value cheaply.
 *
 * Thread-safety: `const` methods are safe to call concurrently from
 * multiple threads — the underlying mmap region is read-only.
 */
class ISRecordView {
public:
    /**
     * Default-constructs the "empty" / end-of-range sentinel view.
     * `bytes().first == nullptr` and `bytes().second == 0`.
     */
    constexpr ISRecordView() noexcept = default;

    /**
     * Constructs a view over a single record. Used by `ISLogReader`
     * when materializing iterator results; not normally called
     * directly by application code.
     *
     * @param did           Data identifier of the record (from `data_sets.h`).
     * @param timestampMs   Payload-derived timestamp in milliseconds.
     * @param deviceId      Device serial number this record belongs to;
     *                      0 if not associated with a device.
     * @param offsetInFile  Byte offset of the record's bytes in the
     *                      backing `.raw` segment.
     * @param data          Pointer to the first byte of the record's
     *                      payload inside the mmap'd region. May be
     *                      `nullptr` for the empty sentinel.
     * @param size          Number of payload bytes addressable from
     *                      `data`.
     * @param flags         `IS_LOG_IDX_REC_FLAG_*` bitmask from the
     *                      source `.idx` record. Bit 0
     *                      (`HAS_TOW`) marks a sync-eligible record.
     *                      Defaults to 0 for backward compatibility.
     */
    constexpr ISRecordView(uint32_t did,
                           uint64_t timestampMs,
                           uint64_t deviceId,
                           uint64_t offsetInFile,
                           const uint8_t* data,
                           std::size_t size,
                           uint16_t flags = 0) noexcept
        : did_(did),
          timestampMs_(timestampMs),
          deviceId_(deviceId),
          offset_(offsetInFile),
          data_(data),
          size_(size),
          flags_(flags) {}

    /**
     * Returns the record's data identifier (`DID_*` from `data_sets.h`).
     * A return value of 0 means "no DID associated" — used for raw
     * stream chunks that the writer did not tag with a DID.
     *
     * @return  The record's DID, or 0 if untagged.
     */
    constexpr uint32_t did() const noexcept { return did_; }

    /**
     * Returns the record's payload-derived timestamp, tagged with
     * provenance per D-06. Source defaults to `PayloadToW` (matches
     * the writer's `HeaderTimeSource::PayloadToW` for this story);
     * D-07 will refine the provenance based on `ISTimeResolver`
     * outputs.
     *
     * @return  A `TimeStamp` whose `value` is the record's milliseconds,
     *          `source` is `PayloadToW`, `confidence` is `Exact`, and
     *          `deviceId` matches the parent reader's device.
     */
    constexpr TimeStamp timestamp() const noexcept {
        return TimeStamp{ timestampMs_,
                          TimeSource::PayloadToW,
                          TimeConfidence::Exact,
                          deviceId_ };
    }

    /**
     * Returns the byte offset at which this record's bytes begin
     * inside the segment's `.raw` file. Verbatim from the `.idx`
     * record's `offset` field.
     *
     * @return  Byte offset (0 == start-of-file).
     */
    constexpr uint64_t offsetInFile() const noexcept { return offset_; }

    /**
     * Returns the `IS_LOG_IDX_REC_FLAG_*` bitmask from the source
     * `.idx` record. Bit 0 (`HAS_TOW`) indicates the record's payload
     * carried a real GPS time-of-week field — used by D-07's
     * `ISTimeResolver` as a sync anchor and by D-08's `ISLogWriter`
     * to preserve the flag through bake/trim.
     *
     * @return  Flags bitmask, or 0 for the empty sentinel.
     */
    constexpr uint16_t flags() const noexcept { return flags_; }

    //! SN-8339: sentinel for "arrival index not assigned" (a bare reader view
    //! that never passed through `ISDeviceLog`'s cross-segment iterator).
    static constexpr uint64_t kNoArrivalIndex = UINT64_MAX;

    /**
     * Returns this record's global arrival index — its 0-based position in the
     * device log's cross-segment arrival order (`ISDeviceLog::allRecords()`).
     * Set by `ISDeviceLog`'s iterator; `kNoArrivalIndex` on a view obtained
     * directly from a single-segment reader (no cross-segment context).
     *
     * SN-8339: this is the key the multi-boot resolver uses to select which
     * power-on session's uptime->ToW offset bridges a session-uptime record —
     * it matches `ISTimeResolver`'s own arrival numbering during its byte scan.
     *
     * @return  Global arrival index, or `kNoArrivalIndex` if unassigned.
     */
    constexpr uint64_t arrivalIndex() const noexcept { return arrivalIndex_; }

    //! Stamp the global arrival index onto this view. Called by
    //! `ISDeviceLog`'s cross-segment iterator; not normally used directly.
    constexpr void setArrivalIndex(uint64_t idx) noexcept { arrivalIndex_ = idx; }

    /**
     * SN-8383: host-uptime-since-log-start (ms) for THIS record, copied verbatim
     * from the source `.idx` v2.1 record's `local_uptime_ms`. 0 when the source
     * was v2.0 (which had no per-record delta). Surfacing it here lets a
     * re-writer (`ISLogWriter`) carry the delta through bake/trim into its
     * (always v2.1) output instead of emitting a zeroed field.
     *
     * @return  Per-record host-uptime delta in ms (0 if the source lacked it).
     */
    constexpr uint32_t localUptimeMs() const noexcept { return localUptimeMs_; }

    //! Stamp the per-record host-uptime delta onto this view. Called by `ISLogReader`.
    constexpr void setLocalUptimeMs(uint32_t ms) noexcept { localUptimeMs_ = ms; }

    /**
     * Returns the record's bytes as a (pointer, size) pair. The
     * pointer aliases the parent reader's mmap'd region; do not
     * dereference after the reader is destroyed or moved-from.
     *
     * @return  `{ data, size }`. `data` is `nullptr` and `size` is
     *          0 for an empty / sentinel view.
     */
    constexpr std::pair<const uint8_t*, std::size_t> bytes() const noexcept {
        return { data_, size_ };
    }

    /**
     * Reinterpret-casts the record's bytes to `const T*`. Returns
     * `nullptr` if `sizeof(T)` does not match the record's byte count
     * — guards against silent over- or under-reads when the caller
     * assumes a structure layout. (D-03 will introduce a
     * `DIDTraits`-aware checked variant that also asserts DID match.)
     *
     * @tparam T  Expected payload type. Must be standard-layout and
     *            its `sizeof` must equal the record's `bytes().second`.
     * @return    Pointer to the record's bytes typed as `const T*`,
     *            or `nullptr` on size mismatch (or on the empty view).
     */
    template <class T>
    const T* as() const noexcept {
        if (data_ == nullptr || size_ != sizeof(T)) return nullptr;
        return reinterpret_cast<const T*>(data_);
    }

    /**
     * Materializes an owning copy of this record. The returned
     * `OwnedRecord` holds its own heap buffer and outlives the
     * parent reader.
     *
     * @return  An `OwnedRecord` whose bytes / metadata equal this
     *          view's at the moment of the call.
     */
    OwnedRecord owned() const;

private:
    uint32_t       did_         = 0;
    uint64_t       timestampMs_ = 0;
    uint64_t       deviceId_    = 0;
    uint64_t       offset_      = 0;
    const uint8_t* data_         = nullptr;
    std::size_t    size_         = 0;
    uint16_t       flags_        = 0;
    uint64_t       arrivalIndex_ = kNoArrivalIndex;
    uint32_t       localUptimeMs_ = 0;   //!< SN-8383: per-record host-uptime delta from the source v2.1 .idx.
};

/**
 * @brief Owning copy of a record's metadata + bytes.
 *
 * Shape mirrors `ISRecordView` (same accessors) so generic code can
 * be written against either. The bytes are copied into an internal
 * `std::vector<uint8_t>` at construction; the record is freely
 * copyable and survives the parent reader.
 */
class OwnedRecord {
public:
    /**
     * Default-constructs an empty owning record. `bytes()` returns
     * `{ nullptr, 0 }`; `did()` returns 0; the timestamp is at
     * epoch 0.
     */
    OwnedRecord() = default;

    /**
     * Constructs an owning record with the given metadata and a
     * heap-resident byte buffer.
     *
     * @param did           Data identifier of the record.
     * @param timestampMs   Payload-derived timestamp in milliseconds.
     * @param deviceId      Device serial number this record belongs to;
     *                      0 if not associated with a device.
     * @param offsetInFile  Original byte offset in the source `.raw`
     *                      segment, preserved for debugging.
     * @param bytes         Heap-allocated byte buffer; ownership
     *                      transfers in (moved). Empty buffer is
     *                      legal.
     */
    OwnedRecord(uint32_t did,
                uint64_t timestampMs,
                uint64_t deviceId,
                uint64_t offsetInFile,
                std::vector<uint8_t> bytes,
                uint16_t flags = 0,
                uint64_t arrivalIndex = ISRecordView::kNoArrivalIndex,
                uint32_t localUptimeMs = 0)
        : did_(did),
          timestampMs_(timestampMs),
          deviceId_(deviceId),
          offset_(offsetInFile),
          bytes_(std::move(bytes)),
          flags_(flags),
          arrivalIndex_(arrivalIndex),
          localUptimeMs_(localUptimeMs) {}

    /** @return  The record's DID, or 0 if untagged. */
    uint32_t did() const noexcept { return did_; }

    /**
     * @return  A `TimeStamp` matching the original record (source
     *          `PayloadToW`, confidence `Exact`).
     */
    TimeStamp timestamp() const noexcept {
        return TimeStamp{ timestampMs_,
                          TimeSource::PayloadToW,
                          TimeConfidence::Exact,
                          deviceId_ };
    }

    /** @return  Original byte offset in the source `.raw` segment. */
    uint64_t offsetInFile() const noexcept { return offset_; }

    /**
     * @return  `IS_LOG_IDX_REC_FLAG_*` bitmask preserved from the
     *          source view at construction.
     */
    uint16_t flags() const noexcept { return flags_; }

    /** @return  Global arrival index preserved from the source view, or
     *           `ISRecordView::kNoArrivalIndex` if it was unassigned. */
    uint64_t arrivalIndex() const noexcept { return arrivalIndex_; }

    /** @return  SN-8383 per-record host-uptime delta (ms) preserved from the
     *           source view; 0 if the source `.idx` was v2.0. */
    uint32_t localUptimeMs() const noexcept { return localUptimeMs_; }

    /**
     * @return  `{ data, size }` over the owning buffer; `data` may
     *          be `nullptr` if the record was default-constructed.
     */
    std::pair<const uint8_t*, std::size_t> bytes() const noexcept {
        return { bytes_.data(), bytes_.size() };
    }

    /**
     * Reinterpret-casts the record's bytes to `const T*`. Same
     * size-check semantics as `ISRecordView::as<T>`.
     *
     * @tparam T  Expected payload type; `sizeof(T)` must equal
     *            `bytes().second`.
     * @return    Typed pointer or `nullptr` on size mismatch.
     */
    template <class T>
    const T* as() const noexcept {
        if (bytes_.size() != sizeof(T)) return nullptr;
        return reinterpret_cast<const T*>(bytes_.data());
    }

private:
    uint32_t             did_         = 0;
    uint64_t             timestampMs_ = 0;
    uint64_t             deviceId_    = 0;
    uint64_t             offset_      = 0;
    std::vector<uint8_t> bytes_;
    uint16_t             flags_        = 0;
    uint64_t             arrivalIndex_ = ISRecordView::kNoArrivalIndex;
    uint32_t             localUptimeMs_ = 0;   //!< SN-8383: per-record host-uptime delta.
};

inline OwnedRecord ISRecordView::owned() const {
    std::vector<uint8_t> copy(data_, data_ + size_);
    return OwnedRecord{ did_, timestampMs_, deviceId_, offset_,
                        std::move(copy), flags_, arrivalIndex_, localUptimeMs_ };
}

} // namespace inertial_sense
