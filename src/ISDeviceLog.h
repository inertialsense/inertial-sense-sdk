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

class ISTimeResolver;  // forward — SN-8105 anchored-span accessors take one by
                       // ref; ISTimeResolver.h includes this header (it builds
                       // from an ISDeviceLog), so we cannot include it back.

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

    /**
     * @brief Compose from segments that are ALREADY OPEN, without re-opening them.
     *
     * Audit C1. `ISLog::openDirectory` has to open every segment anyway, to read its device id
     * before it can group segments into devices — and it then dropped each reader so
     * @ref fromSegments could open it a second time. Opening a segment is not cheap: it builds
     * the record index (a full byte scan when the sidecar is missing or stale) and runs the
     * anchor cascade, so a directory open paid for both **twice per segment**.
     *
     * @param readers  Open readers for one device's segments, **in filename order** — the
     *                 composition uses that order as its tiebreaker when anchored starts are
     *                 equal or missing, so an unsorted vector changes the result. Moved from.
     * @return The composed log, or `ISErrorCode::InvalidArgument` if @p readers is empty,
     *         `Corrupted` on a device-id mismatch, `Unsupported` on mixed `.raw`/`.dat`.
     */
    static ISExpected<ISDeviceLog>
        fromReaders(std::vector<ISLogReader>&& readers);

    /**
     * @brief Every segment's diagnostics, in segment order — audit B3.
     *
     * The link that was missing: `ISLogReader` knew about stalled clocks, rebuilt sidecars and
     * weak anchors, and nothing above it could ask.
     */
    std::vector<ISDiagnostic> diagnostics() const;

    ~ISDeviceLog();
    ISDeviceLog(const ISDeviceLog&)            = delete;
    ISDeviceLog& operator=(const ISDeviceLog&) = delete;
    ISDeviceLog(ISDeviceLog&&) noexcept;
    ISDeviceLog& operator=(ISDeviceLog&&) noexcept;

    // ----- Metadata --------------------------------------------------

    /** @return  Shared device id of every segment in this composition. */
    uint64_t deviceId() const noexcept { return deviceId_; }

    /**
     * @return  Shared on-disk format of every segment in this composition
     *          (D-119 / SN-8626). `fromSegments` rejects a mix of `.raw`
     *          and `.dat` segments, so any segment's `format()` — here,
     *          the first — is representative of the whole log. Consumers
     *          that branch on record-byte layout (`ISRecordView::bytes()`
     *          means something different per format) check this once per
     *          log rather than per record.
     */
    ISLogReader::SegmentFormat format() const noexcept { return segments_.front().format(); }

    /**
     * Returns the full `dev_info_t` from the first segment that carries one.
     *
     * Not simply `segments_.front()` as `hdwId()` does: a composition is
     * ordered by time, and the first segment is not guaranteed to be the one
     * holding a device-info record — a capture that rolled a new file mid-run
     * commonly emits DEV_INFO only in a later segment. Scan for the first
     * segment that actually has one.
     *
     * @return  Reference to the retained payload, or an all-zero struct if NO
     *          segment carried a device-info record. Gate on @ref hasDevInfo.
     */
    const dev_info_t& devInfo() const noexcept {
        for (const auto& s : segments_) {
            if (s.hasDevInfo()) return s.devInfo();
        }
        static const dev_info_t kEmpty{};
        return kEmpty;
    }

    //! True when any segment carried a real device-info record.
    bool hasDevInfo() const noexcept {
        for (const auto& s : segments_) {
            if (s.hasDevInfo()) return true;
        }
        return false;
    }

    /**
     * Returns the device's packed hardware id (`is_hardware_t`). Within a
     * composition every segment is the same physical device (asserted in
     * `compose`), so this is a single shared value.
     *
     * @return  Packed hardware id, or 0 if no DEV_INFO record was logged in
     *          ANY segment.
     */
    uint16_t hdwId() const noexcept {
        // First segment with a NON-ZERO hdwId, not simply the first segment
        // (SN-8445). Segment order is chronological, and a capture that rolled
        // files mid-run may emit DEV_INFO only in a later segment; taking
        // `front()` unconditionally then reported `???-0.0` for a device whose
        // hardware id was sitting in segment 1. Falls back to 0 when no segment
        // has one, which is the pre-existing "no device-info anywhere" case.
        for (const auto& s : segments_) {
            if (s.hdwId() != 0) return s.hdwId();
        }
        return uint16_t{0};
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

    /**
     * @brief SN-8105: earliest record timestamp, GPS-anchored via the resolver.
     *
     * `spanStart()` returns the raw first/last `TimeStamp::value()` in whatever
     * time domain the originating record used (session-uptime for radio modules,
     * GPS-ToW for GPS-anchored devices), so on a GPS-anchored log it can report a
     * ~0 ms session-uptime value that renders as a 1970 wall-clock. This variant
     * routes every record's raw timestamp through `resolver` (SN-8339: keyed on
     * the record's global arrival index, so multi-boot logs bridge per session)
     * and folds the minimum of the anchored, non-`SessionOnly` results.
     *
     * @param resolver  Resolver built from this device log.
     * @return  Earliest GPS-anchored absolute-ms timestamp, or `spanStart()`
     *          (raw) if the log has no anchorable records.
     */
    TimeStamp anchoredSpanStart(const ISTimeResolver& resolver) const noexcept;

    /**
     * @name Which "when does this log start" to use — audit B2
     *
     * There are three, they answer different questions, and before this note there was no
     * documented precedence between them (D0065/D0066 ask for one canonical frame). In order of
     * what a caller usually wants:
     *
     * 1. **`anchoredSpanStart/End(resolver)` — the user-visible wall clock.** Every record routed
     *    through `ISTimeResolver`, folded to the extremes. This is the only one guaranteed to be
     *    an absolute Unix-epoch time, because only the resolver knows the GPS week. Costs a full
     *    per-record pass and needs a resolver, so it is for display and export, not for ordering.
     * 2. **`anchorAnalysis().anchoredStartMs` — the canonical ORDERING key.** The cascade's
     *    per-segment placement, and what `fromReaders` sorts on. Cheap: it comes free with the
     *    index build.
     * 3. **`spanStart/End()` — the cascade's answer folded across segments**, tagged with the
     *    provenance it earned (audit A2). Same source as (2), so they never disagree.
     *
     * **The trap, and why (1) exists at all: (2) and (3) are NOT always a wall clock.** For a
     * segment anchored only by payload time-of-week the cascade leaves the value in the ToW
     * domain — it has no GPS week to convert with — so the value is a few hundred million ms and
     * renders as 1980. That is the symptom customers report, and `TimeStamp::source` is how a
     * consumer tells the cases apart: `PayloadToW` may still be ToW-domain, whereas
     * `FileTimeAnchored` and a resolver-anchored value are Unix-absolute.
     *
     * What all three share, and what `SpanPrecedence.*` asserts: they must agree on **duration**.
     * A frame shift may move where a log sits; it must never change how long it lasted.
     */
    ///@{
    ///@}

    /**
     * @brief SN-8105: latest record timestamp, GPS-anchored via the resolver.
     *        See `anchoredSpanStart`; folds the maximum instead.
     *
     * @param resolver  Resolver built from this device log.
     * @return  Latest GPS-anchored absolute-ms timestamp, or `spanEnd()` (raw)
     *          if the log has no anchorable records.
     */
    TimeStamp anchoredSpanEnd(const ISTimeResolver& resolver) const noexcept;

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
