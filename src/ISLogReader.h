/**
 * @file ISLogReader.h
 * @brief Segment-level reader for SDK 3.0 — reads `.raw` + v2 `.idx`.
 *
 * D-02 / SN-7893 / D0019 / D0020 / D0021 / D0022 / D0049 / D0051: the
 * type-erased core of the new SDK reader. Operates on **one segment**
 * (one `.raw` file) at a time; segment-grouping into device logs and
 * sessions sits above this class (D-05).
 *
 * Backing storage is memory-mapped where the host filesystem supports
 * it; falls back to buffered I/O otherwise. Records are exposed as
 * non-owning `ISRecordView` values (D0022 — copy on demand via
 * `ISRecordView::owned()`). Iterators satisfy the C++17 forward-
 * iterator concepts so `std::ranges` consumers (Logalyzer at C++20
 * via the D-23 adapter) work cleanly alongside C++17 callers.
 *
 * **Thread-safety:** an `ISLogReader` is read-only and `const`-safe
 * after construction. Multiple threads may concurrently iterate or
 * seek on the same instance — the mmap'd region is immutable, the
 * in-memory `.idx` is built once at open time, and no internal
 * buffer is written after construction.
 *
 * **Error model:** all fallible entry points return
 * `ISExpected<T>` per D-10. Missing `.idx` sidecar is **not** an
 * error at this story's scope — `openSegment` succeeds with a
 * lazily-built in-memory index. Persistent rebuild is D-04.
 *
 * **Move-only:** copying a reader would duplicate the mmap'd state in
 * ways that are easy to get wrong. Deleted copy ops, defaulted move
 * ops.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISError.h"
#include "ISLogIndex.h"
#include "ISLogSource.h"
#include "ISRecordView.h"
#include "ISTimeStamp.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iterator>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace inertial_sense {

class ISLogReader {
public:
    using did_t = uint32_t;

    // -----------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------

    /**
     * @brief Open a single segment.
     *
     * Sidecar discovery rule: replace the `.raw` suffix with `.idx`
     * (e.g. `LOG_..._0001.raw` → `LOG_..._0001.idx`) — matches the
     * writer convention (cf. `cDeviceLog::OpenNewSaveFile`).
     *
     * @param raw  Path to the `.raw` segment file.
     * @return     Reader on success; `ISErrorCode` on failure:
     *             `NotFound`, `PermissionDenied`, `Corrupted`, `Io`.
     */
    static ISExpected<ISLogReader> openSegment(const std::filesystem::path& raw);

    ~ISLogReader();

    ISLogReader(const ISLogReader&)            = delete;
    ISLogReader& operator=(const ISLogReader&) = delete;
    ISLogReader(ISLogReader&&) noexcept;
    ISLogReader& operator=(ISLogReader&&) noexcept;

    // -----------------------------------------------------------------
    // Header / segment-level metadata
    // -----------------------------------------------------------------

    /// `.idx` v2 header. If the sidecar was absent, this returns the
    /// in-memory header constructed from the lazy index (magic =
    /// "ISIX", version = 2, FINALIZED unset).
    const idx::is_log_idx_header_t& header() const noexcept { return header_; }

    /// True if the segment had a v2 `.idx` sidecar that parsed cleanly.
    /// False if the sidecar was missing — readers fall back to a
    /// `.raw`-scan-built in-memory index. (D-04 will persist the
    /// rebuild; until then the lazy index is recomputed each open.)
    bool hadOnDiskIndex() const noexcept { return hadOnDiskIndex_; }

    /// Earliest record timestamp (units per `header().ts_units`).
    /// 0 if the segment is empty.
    uint64_t segmentStartTimestamp() const noexcept;

    /// Latest record timestamp. 0 if the segment is empty.
    uint64_t segmentEndTimestamp() const noexcept;

    /// Total record count across all DIDs.
    std::size_t recordCount() const noexcept { return records_.size(); }

    /// Sorted list of DIDs that appear at least once in this segment.
    std::vector<did_t> presentDids() const;

    /// Device serial number derived from the first `DID_DEV_INFO`
    /// record's payload. Falls back to filename parsing
    /// (`LOG_SN<N>_*.raw`) if no DEV_INFO record was logged.
    /// Returns 0 if neither path produces a value.
    uint64_t deviceId() const noexcept { return deviceId_; }

    // -----------------------------------------------------------------
    // Iteration
    // -----------------------------------------------------------------

    class RangeIterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type        = ISRecordView;
        using difference_type   = std::ptrdiff_t;
        using reference         = ISRecordView;
        using pointer           = const ISRecordView*;

        RangeIterator() noexcept = default;
        RangeIterator(const ISLogReader* parent,
                      const std::vector<std::size_t>* indices,
                      std::size_t pos) noexcept
            : parent_(parent), indices_(indices), pos_(pos) {}

        ISRecordView operator*() const noexcept;
        RangeIterator& operator++() noexcept { ++pos_; return *this; }
        RangeIterator  operator++(int) noexcept { auto t = *this; ++pos_; return t; }

        friend bool operator==(const RangeIterator& a, const RangeIterator& b) noexcept {
            return a.parent_ == b.parent_ && a.indices_ == b.indices_ && a.pos_ == b.pos_;
        }
        friend bool operator!=(const RangeIterator& a, const RangeIterator& b) noexcept {
            return !(a == b);
        }

    private:
        const ISLogReader*               parent_  = nullptr;
        const std::vector<std::size_t>*  indices_ = nullptr;
        std::size_t                      pos_     = 0;
    };

    class Range {
    public:
        Range(const ISLogReader* parent,
              const std::vector<std::size_t>* indices,
              std::size_t begin, std::size_t end) noexcept
            : parent_(parent), indices_(indices), begin_(begin), end_(end) {}

        RangeIterator begin() const noexcept {
            return RangeIterator{ parent_, indices_, begin_ };
        }
        RangeIterator end() const noexcept {
            return RangeIterator{ parent_, indices_, end_ };
        }

        std::size_t size() const noexcept { return end_ - begin_; }
        bool empty() const noexcept { return begin_ == end_; }

        /// Returns the sub-range of records in the closed interval
        /// `[t0, t1]` (timestamps compared by `value` only — see
        /// `TimeStamp` ordering rules in D-06). Records are ordered
        /// in arrival order, NOT timestamp order; this filter does a
        /// linear scan with early-exit since the sub-range is built
        /// over the same `indices_` array. For the common case of
        /// monotonic timestamps within one DID, callers should expect
        /// O(N) here. A binary-search variant lands when D-07 anchors
        /// per-DID timestamps in monotonic order.
        Range in_time(TimeStamp t0, TimeStamp t1) const;

    private:
        const ISLogReader*               parent_;
        const std::vector<std::size_t>*  indices_;
        std::size_t                      begin_;
        std::size_t                      end_;
    };

    /// Records for a single DID. If the DID is not present, returns
    /// an empty range (`begin == end`).
    Range records(did_t did) const noexcept;

    /// Records across all DIDs, in arrival order.
    Range allRecords() const noexcept;

    /// Position an iterator over `allRecords()` at the first record
    /// with `record.timestamp() >= target`. Uses `.idx` v2 binary
    /// search internally — O(log N) when records are
    /// timestamp-monotonic.
    ///
    /// **Note on monotonicity:** v2 `.idx` records are written in
    /// arrival order, which is not always timestamp-monotonic
    /// (different DIDs ship time from different clocks; cf. the
    /// PIMU-vs-INS divergence captured in the D-01 cltool
    /// validation). For non-monotonic input this method falls back
    /// to a linear scan. D-07's `ISTimeResolver` outputs are
    /// timestamp-monotonic, so the binary-search fast path will be
    /// the norm post-D-07.
    RangeIterator seek(TimeStamp target) const noexcept;

private:
    ISLogReader() = default;

    // Construction helpers (called from openSegment).
    static ISExpected<ISLogReader>
        construct(std::unique_ptr<ISLogSource> raw,
                  const std::filesystem::path& rawPath);
    void buildIndexFromIdx(const std::vector<idx::is_log_idx_record_v2_t>& records);
    void buildIndexFromScan();   // lazy fallback when .idx is missing.
    void deriveDeviceId(const std::filesystem::path& rawPath);
    std::size_t recordEndOffset(std::size_t recordIdx) const noexcept;

    // ISRecordView materialization for a record at the given index.
    ISRecordView viewAt(std::size_t recordIdx) const noexcept;

    // Backing storage.
    std::unique_ptr<ISLogSource>           rawSource_;
    idx::is_log_idx_header_t               header_{};
    std::vector<idx::is_log_idx_record_v2_t> records_;

    // DID → indices into records_, sorted by arrival order.
    // Built once at open time. The empty-range case for a missing DID
    // is handled by returning a static empty vector pointer.
    std::unordered_map<uint32_t, std::vector<std::size_t>> byDid_;
    std::vector<std::size_t>               allIndices_;        // 0..N-1
    static const std::vector<std::size_t>  kEmptyIndices_;

    bool                                   hadOnDiskIndex_ = false;
    uint64_t                               deviceId_       = 0;

    friend class RangeIterator;
    friend class Range;
};

} // namespace inertial_sense
