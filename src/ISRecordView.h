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
     * Default-constructed view is the "empty" / end-of-range sentinel.
     * `bytes().first == nullptr` and `bytes().second == 0`.
     */
    constexpr ISRecordView() noexcept = default;

    /**
     * Construct a view over a record. Used by `ISLogReader`.
     */
    constexpr ISRecordView(uint32_t did,
                           uint64_t timestampMs,
                           uint64_t deviceId,
                           uint64_t offsetInFile,
                           const uint8_t* data,
                           std::size_t size) noexcept
        : did_(did),
          timestampMs_(timestampMs),
          deviceId_(deviceId),
          offset_(offsetInFile),
          data_(data),
          size_(size) {}

    /**
     * Data identifier (`DID_*` from `data_sets.h`). 0 means "no DID
     * associated" — used for raw stream chunks the writer didn't tag.
     */
    constexpr uint32_t did() const noexcept { return did_; }

    /**
     * Payload-derived timestamp, tagged with provenance per D-06.
     * Source defaults to `PayloadToW` (matches the writer's
     * `HeaderTimeSource::PayloadToW` for this story); D-07 will
     * refine the provenance based on `ISTimeResolver` outputs.
     */
    constexpr TimeStamp timestamp() const noexcept {
        return TimeStamp{ timestampMs_,
                          TimeSource::PayloadToW,
                          TimeConfidence::Exact,
                          deviceId_ };
    }

    /**
     * Byte offset into the segment's `.raw` file where this record's
     * bytes begin. Verbatim from the `.idx` record's `offset` field.
     */
    constexpr uint64_t offsetInFile() const noexcept { return offset_; }

    /**
     * Pointer + size of this record's bytes inside the mmap'd region.
     * Pointer aliases the segment's mmap; do not dereference after
     * the parent reader is destroyed or moved-from.
     */
    constexpr std::pair<const uint8_t*, std::size_t> bytes() const noexcept {
        return { data_, size_ };
    }

    /**
     * Reinterpret-cast helper. Returns `nullptr` if `sizeof(T)` does
     * not match the record's byte count — guards against silent
     * over-/under-reads when the caller assumes a structure layout.
     * (D-03 will introduce a `DIDTraits`-aware checked variant.)
     */
    template <class T>
    const T* as() const noexcept {
        if (data_ == nullptr || size_ != sizeof(T)) return nullptr;
        return reinterpret_cast<const T*>(data_);
    }

    /**
     * Materialize an owning copy. The returned record holds its own
     * buffer and outlives the parent reader.
     */
    OwnedRecord owned() const;

private:
    uint32_t       did_         = 0;
    uint64_t       timestampMs_ = 0;
    uint64_t       deviceId_    = 0;
    uint64_t       offset_      = 0;
    const uint8_t* data_        = nullptr;
    std::size_t    size_        = 0;
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
    OwnedRecord() = default;
    OwnedRecord(uint32_t did,
                uint64_t timestampMs,
                uint64_t deviceId,
                uint64_t offsetInFile,
                std::vector<uint8_t> bytes)
        : did_(did),
          timestampMs_(timestampMs),
          deviceId_(deviceId),
          offset_(offsetInFile),
          bytes_(std::move(bytes)) {}

    uint32_t did() const noexcept { return did_; }
    TimeStamp timestamp() const noexcept {
        return TimeStamp{ timestampMs_,
                          TimeSource::PayloadToW,
                          TimeConfidence::Exact,
                          deviceId_ };
    }
    uint64_t offsetInFile() const noexcept { return offset_; }
    std::pair<const uint8_t*, std::size_t> bytes() const noexcept {
        return { bytes_.data(), bytes_.size() };
    }
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
};

inline OwnedRecord ISRecordView::owned() const {
    std::vector<uint8_t> copy(data_, data_ + size_);
    return OwnedRecord{ did_, timestampMs_, deviceId_, offset_, std::move(copy) };
}

} // namespace inertial_sense
