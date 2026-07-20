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
    const uint8_t* data_        = nullptr;
    std::size_t    size_        = 0;
    uint16_t       flags_       = 0;
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
                uint16_t flags = 0)
        : did_(did),
          timestampMs_(timestampMs),
          deviceId_(deviceId),
          offset_(offsetInFile),
          bytes_(std::move(bytes)),
          flags_(flags) {}

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
    uint16_t             flags_       = 0;
};

inline OwnedRecord ISRecordView::owned() const {
    std::vector<uint8_t> copy(data_, data_ + size_);
    return OwnedRecord{ did_, timestampMs_, deviceId_, offset_, std::move(copy), flags_ };
}

} // namespace inertial_sense
