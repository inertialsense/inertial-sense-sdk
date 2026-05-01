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

    /** Destroys the reader and releases the mmap (or buffer) and file handle. */
    ~ISLogReader();

    ISLogReader(const ISLogReader&)            = delete;
    ISLogReader& operator=(const ISLogReader&) = delete;

    /**
     * Move-construct. Transfers the source's mmap and index;
     * iterators / views obtained from the source are invalidated.
     *
     * @param other  Source reader; left in a valid but unspecified
     *               state (safe to destroy).
     */
    ISLogReader(ISLogReader&&) noexcept;

    /**
     * Move-assign. Releases this reader's resources (if any) before
     * adopting the source's state.
     *
     * @param other  Source reader; left in a valid but unspecified
     *               state.
     * @return       Reference to `*this`.
     */
    ISLogReader& operator=(ISLogReader&&) noexcept;

    // -----------------------------------------------------------------
    // Header / segment-level metadata
    // -----------------------------------------------------------------

    /**
     * Returns the parsed `.idx` v2 header. If the sidecar was absent
     * at open time, this returns an in-memory header constructed
     * from the lazy index (magic = "ISIX", version = 2, FINALIZED
     * unset).
     *
     * @return  Reference to the segment's header. Lifetime tied to
     *          this reader.
     */
    const idx::is_log_idx_header_t& header() const noexcept { return header_; }

    /**
     * Reports whether the segment had a v2 `.idx` sidecar on disk
     * that parsed cleanly at open time. If false, the reader either
     * rebuilt the in-memory index by scanning the `.raw` (D-04) or,
     * if the sidecar was absent / stale / v1, optionally persisted
     * a fresh v2 `.idx` next to the `.raw`.
     *
     * @return  `true` if the on-disk `.idx` was used as-is; `false`
     *          if the in-memory index was rebuilt from a `.raw` scan.
     */
    bool hadOnDiskIndex() const noexcept { return hadOnDiskIndex_; }

    /**
     * Reports whether the `.raw` ended mid-record — typically because
     * the embedded logger was killed or power-cut while writing the
     * tail packet. Records before the truncation are valid and
     * iterable; the truncation itself is non-fatal.
     *
     * @return  `true` if a truncation was detected; `false` if the
     *          file ended on a clean packet boundary.
     */
    bool isTruncated() const noexcept { return isTruncated_; }

    /**
     * @return  Byte offset into the `.raw` where parsing stopped.
     *          Equal to `fileSize()` when the file ends cleanly.
     *          When `isTruncated() == true`, this is the start of
     *          the discarded partial packet.
     */
    uint64_t truncationOffset() const noexcept { return truncationOffset_; }

    /**
     * @return  Total byte count of the backing `.raw` file. Same
     *          as `ISLogSource::size()`; exposed here for
     *          symmetry with `truncationOffset()`.
     */
    uint64_t fileSize() const noexcept;

    /**
     * Direct accessor for the segment's underlying byte buffer.
     *
     * Most callers should iterate via `records()` / `allRecords()`
     * — this hatch is for code that needs to re-parse the raw stream
     * (e.g. the derivation catalog at D-09, which extracts typed
     * payload structs that the index-record-shared-offset semantics
     * don't expose cleanly through `ISRecordView::bytes()`).
     *
     * @return  `{ data, size }`. `data` aliases the mmap'd region
     *          (or buffer fallback); the lifetime is tied to this
     *          reader. Empty pair if the segment was opened against
     *          a zero-byte source.
     */
    std::pair<const uint8_t*, std::size_t> rawBytes() const noexcept;

    /**
     * Side-channel diagnostic strings collected during open.
     * Categories observed today (added by D-04):
     *
     *   - `"truncation: stopped at offset N (file size M)"` when
     *     `isTruncated() == true`.
     *   - `"sidecar: rebuilt from .raw scan (reason: missing|stale|v1)"`
     *     when a `.idx` was rebuilt rather than read from disk.
     *   - `"sidecar: persist failed (read-only filesystem?)"` when
     *     the rebuilt index couldn't be written back next to the
     *     `.raw`. Reading still works from the in-memory index.
     *
     * @return  Reference to the warnings vector. Empty when the
     *          open path was straightforward.
     */
    const std::vector<std::string>& warnings() const noexcept { return warnings_; }

    /**
     * @return  Earliest record timestamp in this segment, in the
     *          units indicated by `header().ts_units`. Returns 0
     *          if the segment contains no records.
     */
    uint64_t segmentStartTimestamp() const noexcept;

    /**
     * @return  Latest record timestamp in this segment, in the units
     *          indicated by `header().ts_units`. Returns 0 if the
     *          segment contains no records.
     */
    uint64_t segmentEndTimestamp() const noexcept;

    /**
     * @return  Total record count across all DIDs in this segment.
     */
    std::size_t recordCount() const noexcept { return records_.size(); }

    /**
     * @return  Sorted ascending list of DIDs that appear at least
     *          once in this segment.
     */
    std::vector<did_t> presentDids() const;

    /**
     * Returns the device's serial number. Derived from the first
     * `DID_DEV_INFO` record's payload; falls back to filename
     * parsing (`LOG_SN<N>_*.raw`) if no DEV_INFO record was logged.
     *
     * @return  Device serial number, or 0 if neither path produces
     *          a value.
     */
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

        /**
         * Default-constructs the past-the-end / singular sentinel.
         * `*it` on a default-constructed iterator yields the empty
         * `ISRecordView`.
         */
        RangeIterator() noexcept = default;

        /**
         * Constructs an iterator into a reader's index. Used by
         * `Range::begin()` / `Range::end()`; not normally called
         * by application code.
         *
         * @param parent   Parent reader; pointer aliased, must
         *                 outlive the iterator.
         * @param indices  Vector of record-array indices the
         *                 iterator walks over (per-DID or all).
         * @param pos      Initial position in `*indices`.
         */
        RangeIterator(const ISLogReader* parent,
                      const std::vector<std::size_t>* indices,
                      std::size_t pos) noexcept
            : parent_(parent), indices_(indices), pos_(pos) {}

        /**
         * @return  An `ISRecordView` over the record at the current
         *          position, or the empty sentinel if past the end.
         */
        ISRecordView operator*() const noexcept;

        /** Pre-increment. @return  Reference to `*this` after advancing. */
        RangeIterator& operator++() noexcept { ++pos_; return *this; }

        /** Post-increment. @return  Copy of `*this` before advancing. */
        RangeIterator  operator++(int) noexcept { auto t = *this; ++pos_; return t; }

        /**
         * @param a  Left-hand iterator.
         * @param b  Right-hand iterator.
         * @return   `true` iff both iterators reference the same
         *           parent + indices array + position.
         */
        friend bool operator==(const RangeIterator& a, const RangeIterator& b) noexcept {
            return a.parent_ == b.parent_ && a.indices_ == b.indices_ && a.pos_ == b.pos_;
        }

        /**
         * @param a  Left-hand iterator.
         * @param b  Right-hand iterator.
         * @return   `!(a == b)`.
         */
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
        /**
         * Constructs a range over a slice of a reader's index.
         * Used by `records()` / `allRecords()` / `in_time()`; not
         * normally called by application code.
         *
         * @param parent   Parent reader; pointer aliased, must
         *                 outlive the range.
         * @param indices  Vector of record-array indices the range
         *                 walks over.
         * @param begin    Inclusive starting position in `*indices`.
         * @param end      Exclusive ending position in `*indices`.
         */
        Range(const ISLogReader* parent,
              const std::vector<std::size_t>* indices,
              std::size_t begin, std::size_t end) noexcept
            : parent_(parent), indices_(indices), begin_(begin), end_(end) {}

        /** @return  Forward iterator at the first record of the range. */
        RangeIterator begin() const noexcept {
            return RangeIterator{ parent_, indices_, begin_ };
        }

        /** @return  Past-the-end forward iterator for the range. */
        RangeIterator end() const noexcept {
            return RangeIterator{ parent_, indices_, end_ };
        }

        /** @return  Number of records in this range. O(1). */
        std::size_t size() const noexcept { return end_ - begin_; }

        /** @return  `true` iff `size() == 0`. */
        bool empty() const noexcept { return begin_ == end_; }

        /**
         * Filters this range to records whose timestamp lies in the
         * closed interval `[t0, t1]`. Records are ordered in arrival
         * order, NOT timestamp order; this filter does a linear scan
         * with early-exit since the sub-range is built over the same
         * `indices_` array. For the common case of monotonic
         * timestamps within one DID, callers should expect O(N)
         * here. A binary-search variant lands when D-07 anchors
         * per-DID timestamps in monotonic order.
         *
         * Comparison uses `TimeStamp::value` only; `source` and
         * `confidence` are ignored (per D-06 ordering rules).
         *
         * @param t0  Inclusive start of the interval.
         * @param t1  Inclusive end of the interval. Must satisfy
         *            `t0.value <= t1.value` for a non-empty result.
         * @return    Sub-range whose iteration yields only records
         *            with timestamps in `[t0, t1]`. Lifetime tied
         *            to the parent reader.
         *
         * @see TimeStamp, ISLogReader::seek
         */
        Range in_time(TimeStamp t0, TimeStamp t1) const;

    private:
        const ISLogReader*               parent_;
        const std::vector<std::size_t>*  indices_;
        std::size_t                      begin_;
        std::size_t                      end_;
    };

    /**
     * Returns a range over all records carrying the given DID.
     * If the DID is not present in this segment, returns an empty
     * range (`begin == end`).
     *
     * @param did  Data identifier to filter on.
     * @return     A range of records, in arrival order, whose
     *             `did()` equals `did`. Lifetime tied to this
     *             reader.
     */
    Range records(did_t did) const noexcept;

    /**
     * @return  A range over every record in the segment, in arrival
     *          order across all DIDs. Lifetime tied to this reader.
     */
    Range allRecords() const noexcept;

    /**
     * Materializes an `ISRecordView` over the record at the given
     * arrival-order index. Composition classes (`ISDeviceLog`,
     * `ISLog` — D-05) use this to dereference cross-segment
     * iterators into the underlying segment's bytes.
     *
     * @param recordIdx  Arrival-order position in this segment's
     *                   record vector.
     * @return           A view, or the empty sentinel if
     *                   `recordIdx >= recordCount()` or the source
     *                   is null.
     */
    ISRecordView recordAt(std::size_t recordIdx) const noexcept {
        return viewAt(recordIdx);
    }

    /**
     * Positions an iterator over `allRecords()` at the first record
     * with `record.timestamp() >= target`. Uses `.idx` v2 binary
     * search internally — O(log N) when records are
     * timestamp-monotonic.
     *
     * **Note on monotonicity:** v2 `.idx` records are written in
     * arrival order, which is not always timestamp-monotonic
     * (different DIDs ship time from different clocks; cf. the
     * PIMU-vs-INS divergence captured in the D-01 cltool
     * validation). For non-monotonic input this method falls back
     * to a linear scan. D-07's `ISTimeResolver` outputs are
     * timestamp-monotonic, so the binary-search fast path will be
     * the norm post-D-07.
     *
     * @param target  Timestamp to seek to. Comparison uses
     *                `TimeStamp::value` only.
     * @return        Iterator positioned at the first record with
     *                `timestamp().value >= target.value`, or
     *                `allRecords().end()` if no such record exists.
     */
    RangeIterator seek(TimeStamp target) const noexcept;

private:
    ISLogReader() = default;

    // Construction helpers — called from openSegment(). Private,
    // lightly documented; full responsibility split + invariants live
    // in the .cpp.

    /**
     * Finishes constructing a reader given an opened source. Reads
     * the `.idx` sidecar (if present), populates the in-memory
     * record vector + byDid_ map, and derives the device id.
     *
     * @param raw      Opened byte source for the `.raw` segment.
     * @param rawPath  Path the segment was opened from; used for
     *                 sidecar discovery and filename-based device-id
     *                 fallback.
     * @return         A fully-initialized reader, or an `ISError`
     *                 if `.idx` parsing reports `Corrupted`.
     */
    static ISExpected<ISLogReader>
        construct(std::unique_ptr<ISLogSource> raw,
                  const std::filesystem::path& rawPath);

    /**
     * Builds the in-memory index from an already-parsed `.idx`
     * record vector. Populates `records_`, `byDid_`, and
     * `allIndices_`.
     *
     * @param records  Parsed v2 `.idx` records, in arrival order.
     */
    void buildIndexFromIdx(const std::vector<idx::is_log_idx_record_v2_t>& records);

    /**
     * Lazy fallback when the `.idx` sidecar is missing. D-02 lands
     * an empty stub here; D-04 will populate the index by scanning
     * the `.raw` and persisting the result.
     */
    void buildIndexFromScan();

    /**
     * Resolves the device id by inspecting the first
     * `DID_DEV_INFO` record's serial number, falling back to
     * filename parsing.
     *
     * @param rawPath  Path the segment was opened from. Used only
     *                 for the filename-fallback path.
     */
    void deriveDeviceId(const std::filesystem::path& rawPath);

    /**
     * Computes the end-of-bytes offset for the record at the given
     * index. Used by `viewAt` to derive `bytes().second`. Records
     * sharing an offset (chunk-input artifact from the writer)
     * resolve to the same end offset.
     *
     * @param recordIdx  Index into `records_`.
     * @return           Byte offset one past the last byte of the
     *                   record's payload, clamped to the source
     *                   size.
     */
    std::size_t recordEndOffset(std::size_t recordIdx) const noexcept;

    /**
     * Materializes an `ISRecordView` over the record at the given
     * index. Out-of-range indices yield the empty sentinel view.
     *
     * @param recordIdx  Index into `records_`.
     * @return           View over the record, or the empty sentinel
     *                   if `recordIdx >= records_.size()` or the
     *                   source is null.
     */
    ISRecordView viewAt(std::size_t recordIdx) const noexcept;

    /**
     * Atomically writes `records_` + `header_` out as a v2 `.idx`
     * sidecar at `idxPath_` (`.idx.tmp` then rename). Called after a
     * successful scan-rebuild when `IS_LOG_READER_NO_PERSIST_INDEX`
     * is unset. Failures are non-fatal — the in-memory index still
     * works.
     *
     * @return  `true` if the file was written successfully.
     */
    bool persistIndex() const;

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

    bool                                   hadOnDiskIndex_     = false;
    bool                                   isTruncated_        = false;
    uint64_t                               truncationOffset_   = 0;
    uint64_t                               deviceId_           = 0;
    std::vector<std::string>               warnings_;
    std::filesystem::path                  rawPath_;
    std::filesystem::path                  idxPath_;

    friend class RangeIterator;
    friend class Range;
};

} // namespace inertial_sense
