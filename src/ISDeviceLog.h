/**
 * @file ISDeviceLog.h
 * @brief Composition class — multiple `.raw` segments from one device.
 *
 * D-05 / SN-7896 / D0051: the second tier of the three-tier model.
 * `ISLogReader` operates at one segment; `ISDeviceLog` stitches
 * rollover segments belonging to a single device into a single
 * logical record stream.
 *
 * Move-only. Composes `ISLogReader`s by value (also move-only). The
 * cross-segment iteration is read-only; multiple threads may
 * iterate or seek concurrently per the same const-safety contract
 * as `ISLogReader`.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISError.h"
#include "ISLogReader.h"
#include "ISRecordView.h"
#include "ISTimeStamp.h"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <iterator>
#include <string>
#include <unordered_map>
#include <vector>

namespace inertial_sense {

class ISDeviceLog {
public:
    using did_t = uint32_t;

    /**
     * (segment_index, in_segment_record_index) — internal record
     * locator. Exposed because the iterator and Range types need
     * to materialize views from it. Not normally constructed by
     * application code.
     */
    struct Locator {
        std::size_t segment;
        std::size_t record;
    };

    /**
     * Composes the given `.raw` segment paths into one device-log.
     * Segments are ordered by their first-record timestamp (or by
     * filename-embedded timestamp when the header isn't yet
     * available). Refuses to compose segments from different
     * `deviceId()` values.
     *
     * @param segmentPaths  `.raw` files to compose. Must all have
     *                      the same device id; order doesn't matter
     *                      (the constructor sorts).
     * @return              An `ISDeviceLog` on success; on failure,
     *                      an `ISError` with one of:
     *                      - `InvalidArgument` — empty input.
     *                      - `NotFound` / `PermissionDenied` — any
     *                        segment failed to open at the
     *                        `ISLogReader::openSegment` boundary.
     *                      - `Corrupted` — segments belong to
     *                        different devices; message names both
     *                        ids.
     */
    static ISExpected<ISDeviceLog>
        fromSegments(std::vector<std::filesystem::path> segmentPaths);

    ~ISDeviceLog();
    ISDeviceLog(const ISDeviceLog&)            = delete;
    ISDeviceLog& operator=(const ISDeviceLog&) = delete;
    ISDeviceLog(ISDeviceLog&&) noexcept;
    ISDeviceLog& operator=(ISDeviceLog&&) noexcept;

    // ----- Metadata --------------------------------------------------

    /** @return  Shared device id of every segment in this composition. */
    uint64_t deviceId() const noexcept { return deviceId_; }

    /**
     * Returns the device's packed hardware id (`is_hardware_t`).
     * Forwarded from the first segment — within a composition, every
     * segment shares the same physical device, so the hardware id is
     * a single shared value (asserted in `compose`).
     *
     * @return  Packed hardware id, or 0 if no DEV_INFO record was
     *          logged in any segment.
     */
    uint16_t hdwId() const noexcept {
        return segments_.empty() ? uint16_t{0} : segments_.front().hdwId();
    }

    /** @return  Total record count across all segments. */
    std::size_t recordCount() const noexcept { return total_; }

    /**
     * @return  Sorted ascending list of DIDs that appear at least
     *          once across any segment.
     */
    std::vector<did_t> presentDids() const;

    /**
     * @return  Earliest record timestamp across all segments
     *          (`PayloadToW` source, `Exact` confidence).
     *          Empty composition returns
     *          `TimeStamp::fromPayloadToW(0, 0)`.
     */
    TimeStamp spanStart() const noexcept;

    /**
     * @return  Latest record timestamp across all segments. Same
     *          empty-composition semantics as `spanStart`.
     */
    TimeStamp spanEnd() const noexcept;

    /** @return  Number of underlying segments composed. */
    std::size_t segmentCount() const noexcept { return segments_.size(); }

    /**
     * Provides direct access to a composed segment's reader. Useful
     * for segment-level metadata (truncation, warnings, on-disk
     * header) that doesn't surface through the cross-segment range
     * API.
     *
     * @param i  Segment index, in composition order
     *           (`0 <= i < segmentCount()`).
     * @return   Reference to the underlying `ISLogReader`. Lifetime
     *           tied to this `ISDeviceLog`.
     */
    const ISLogReader& segment(std::size_t i) const noexcept { return segments_[i]; }

    // ----- Iteration -------------------------------------------------

    class RangeIterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type        = ISRecordView;
        using difference_type   = std::ptrdiff_t;
        using reference         = ISRecordView;
        using pointer           = const ISRecordView*;

        /** Default-constructs the past-the-end / singular sentinel. */
        RangeIterator() noexcept = default;

        /**
         * @param parent     Parent device-log; aliased pointer.
         * @param locators   Vector of (segment, record) locators
         *                   the iterator walks over.
         * @param pos        Initial position in `*locators`.
         */
        RangeIterator(const ISDeviceLog* parent,
                      const std::vector<Locator>* locators,
                      std::size_t pos) noexcept
            : parent_(parent), locators_(locators), pos_(pos) {}

        /** @return  View over the current locator's record. */
        ISRecordView operator*() const noexcept;

        /** Pre-increment. May fire the segment-boundary callback. */
        RangeIterator& operator++() noexcept;

        /** Post-increment. */
        RangeIterator  operator++(int) noexcept { auto t = *this; ++(*this); return t; }

        /**
         * @param a  Left.  @param b  Right.
         * @return  `true` iff parent + locator-array + position match.
         */
        friend bool operator==(const RangeIterator& a, const RangeIterator& b) noexcept {
            return a.parent_ == b.parent_ && a.locators_ == b.locators_ && a.pos_ == b.pos_;
        }

        /** @param a Left. @param b Right. @return  `!(a == b)`. */
        friend bool operator!=(const RangeIterator& a, const RangeIterator& b) noexcept {
            return !(a == b);
        }

    private:
        const ISDeviceLog*               parent_   = nullptr;
        const std::vector<Locator>*      locators_ = nullptr;
        std::size_t                      pos_      = 0;
    };

    class Range {
    public:
        /**
         * @param parent    Parent device-log; aliased pointer.
         * @param locators  Locator vector to walk.
         * @param begin     Inclusive starting position in `*locators`.
         * @param end       Exclusive ending position in `*locators`.
         */
        Range(const ISDeviceLog* parent,
              const std::vector<Locator>* locators,
              std::size_t begin, std::size_t end) noexcept
            : parent_(parent), locators_(locators), begin_(begin), end_(end) {}

        /** @return  Iterator at the first record. */
        RangeIterator begin() const noexcept {
            return RangeIterator{ parent_, locators_, begin_ };
        }
        /** @return  Past-the-end iterator. */
        RangeIterator end() const noexcept {
            return RangeIterator{ parent_, locators_, end_ };
        }
        /** @return  O(1) record count of the range. */
        std::size_t size() const noexcept { return end_ - begin_; }
        /** @return  `true` iff `size() == 0`. */
        bool empty() const noexcept { return begin_ == end_; }

        /**
         * Filters this range to records whose timestamp lies in
         * `[t0, t1]` (inclusive). Linear scan in arrival order; same
         * complexity as `ISLogReader::Range::in_time`.
         *
         * @param t0  Inclusive start. Compared by `value` only.
         * @param t1  Inclusive end.   Compared by `value` only.
         * @return    Sub-range; lifetime tied to the parent device-log.
         */
        Range in_time(TimeStamp t0, TimeStamp t1) const;

    private:
        const ISDeviceLog*               parent_;
        const std::vector<Locator>*      locators_;
        std::size_t                      begin_;
        std::size_t                      end_;
    };

    /**
     * @param did  DID to filter on.
     * @return     Range over every record with this DID across all
     *             segments, in arrival order.
     */
    Range records(did_t did) const noexcept;

    /** @return  Range over every record across all segments. */
    Range allRecords() const noexcept;

    /**
     * Positions an iterator over `allRecords()` at the first record
     * with `record.timestamp().value >= target.value`. Linear scan
     * across segments; D-07 will replace this with a monotonic
     * binary-search after time resolution.
     *
     * @param target  Timestamp to seek to.
     * @return        Iterator at the first matching record, or
     *                `allRecords().end()` if no record satisfies.
     */
    RangeIterator seek(TimeStamp target) const noexcept;

    // ----- Optional segment-boundary callback ------------------------

    /**
     * Registers a callback invoked when iteration crosses from
     * segment `i` to segment `i+1`. Useful for D-07's time resolver
     * to flag potential mid-session clock jumps.
     *
     * @param cb  Callback; receives the index of the segment just
     *            completed (i.e., the boundary is `cb(i)` →
     *            iteration is now at the first record of segment
     *            `i+1`). Pass an empty `std::function` to clear.
     */
    void setOnSegmentBoundary(std::function<void(std::size_t)> cb) noexcept {
        onSegmentBoundary_ = std::move(cb);
    }

private:
    ISDeviceLog() = default;

    /**
     * Builds the cross-segment locator vectors after `segments_` is
     * populated and ordered.
     */
    void buildIndex();

    std::vector<ISLogReader>           segments_;       ///< Owning, in composition order.
    uint64_t                           deviceId_ = 0;
    std::size_t                        total_    = 0;

    // (segment_idx, record_idx) locators in arrival order, plus
    // per-DID filtered subsets.
    std::vector<Locator>                              all_;
    std::unordered_map<did_t, std::vector<Locator>>   byDid_;
    static const std::vector<Locator>                 kEmptyLocators_;

    //! SN-8339: per-segment base of the global arrival index. A record at
    //! (segment s, record r) has global arrival index `segmentBase_[s] + r` —
    //! its 0-based position in cross-segment arrival order. Stamped onto every
    //! view the range iterator yields so consumers can key the multi-boot
    //! resolver. Matches `ISTimeResolver`'s own byte-scan arrival numbering.
    std::vector<std::size_t>                          segmentBase_;

    std::function<void(std::size_t)>   onSegmentBoundary_;

    friend class RangeIterator;
};

} // namespace inertial_sense
