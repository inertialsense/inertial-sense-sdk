/**
 * @file ISLogReader.h
 * @brief Segment-level reader for SDK 3.0 — reads `.raw` or `.dat`, both
 *        indexed via the same v2 `.idx` sidecar schema.
 *
 * D-02 / SN-7893 / D0019 / D0020 / D0021 / D0022 / D0049 / D0051: the
 * type-erased core of the new SDK reader. Operates on **one segment**
 * (one `.raw` or `.dat` file) at a time; segment-grouping into device
 * logs and sessions sits above this class (D-05).
 *
 * D-119 / SN-8626 / D0082: `.dat` (`LOGTYPE_DAT`, `cDeviceLogSerial`'s
 * chunk-header-framed, already-parsed `p_data_hdr_t`/payload format) is
 * supported as a second segment format alongside `.raw`, dispatched
 * internally via `format()` — never as a subclass (see D0082 for why).
 * `cDeviceLogSerial`/`DataChunk.h` are consulted only for the on-disk
 * byte layout, never as an interface template.
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
#include "data_sets.h"      // dev_info_t, returned by devInfo()
#include "ISLogSource.h"
#include "ISRecordView.h"
#include "ISAnchorAnalysis.h"
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

// Forward declarations for the D-03 / SN-7894 templated sugar layer.
// Full definitions live in `DIDTraits.h` + `ISLogReaderSugar.h`,
// kept out of this header so type-erased-core consumers don't pay
// the `data_sets.h` compile cost.
template <std::uint32_t DID> struct DIDTraits;
template <class T>           class  TypedRange;

// Gap detection (below) spans a whole device log; forward-declared so the
// single-segment header stays light. Full definitions in ISDeviceLog.h /
// ISTimeResolver.h, pulled in by ISLogReader.cpp.
class ISDeviceLog;
class ISTimeResolver;

class ISLogReader {
public:
    using did_t = uint32_t;

    /**
     * @brief On-disk segment format this reader was opened against.
     *
     * Recognized by extension at `openSegment()` time (D-119 / SN-8626).
     * `.raw` and `.dat` share the same `.idx` v2 sidecar schema and the
     * same public API — this enum only gates a handful of internal byte-
     * layout decisions (`buildIndexFromScan*`, `recordEndOffset`,
     * `deriveDeviceId*`). See D0082.
     */
    enum class SegmentFormat : uint8_t {
        Raw,   ///< Undecoded, multi-protocol byte stream (`cDeviceLogRaw`, `.raw`).
        Dat,   ///< Chunk-header-framed, already-parsed records (`cDeviceLogSerial`, `.dat`).
    };

    // -----------------------------------------------------------------
    // Gap detection (SN-8345) — coverage gaps on the resolved timeline
    // -----------------------------------------------------------------

    /// Sentinel segment id: a gap NO segment covers (a whole segment absent, or
    /// a recording pause between segments). A valid (>= 0) id instead marks a
    /// gap WITHIN a present segment — a dropped-data interval (future). Callers
    /// colour the two cases differently on the timeline.
    static constexpr int kNoSegment = -1;

    /// A segment's resolved wall-clock coverage, `[start, end]`. Input to
    /// @ref findGaps. `start`/`end` are resolved (unified-domain) TimeStamps.
    struct SegmentSpan {
        int       segmentId = kNoSegment;  ///< composition index (0-based)
        TimeStamp start{};                 ///< earliest resolved record time
        TimeStamp end{};                   ///< latest resolved record time

        /// True when the span is a usable, non-degenerate interval.
        bool valid() const noexcept {
            return end.value >= start.value && (start.value != 0 || end.value != 0);
        }
    };

    /// A detected coverage gap on the unified resolved timeline.
    struct DataGap {
        TimeStamp startTime{};             ///< gap start = end of prior coverage
        TimeStamp endTime{};               ///< gap end   = start of next coverage
        int       segmentId = kNoSegment;  ///< @ref kNoSegment = no owning segment
                                           ///< (missing/pause); valid = within-segment drop

        /// Gap width in ms (0 if degenerate).
        uint64_t durationMs() const noexcept {
            return endTime.value > startTime.value ? endTime.value - startTime.value : 0;
        }
    };

    /**
     * @brief Detect coverage gaps between segment spans. PURE — no I/O.
     *
     * Spans need not be sorted or disjoint. Sweeps them in start order tracking
     * the running coverage high-water mark; whenever the next span begins more
     * than @p thresholdMs after that mark, the interval is reported as a gap
     * (with @ref kNoSegment — no segment covers it). Overlapping / contiguous
     * spans never produce a gap.
     *
     * @param spans        Per-segment resolved spans; invalid spans ignored.
     * @param thresholdMs  Minimum gap width to report (`<= thresholdMs` skipped).
     * @return             Gaps in ascending start order.
     */
    static std::vector<DataGap> findGaps(std::vector<SegmentSpan> spans,
                                         uint64_t thresholdMs);

    /**
     * @brief Resolve each segment's coverage from @p log and detect gaps.
     *
     * Scans every record of every segment through @p resolver, tracking the
     * min/max resolved value per segment (records that resolve to
     * @ref TimeSource::SessionOnly are excluded — an unanchored segment yields
     * no span). A full scan is deliberate: within a segment records can be
     * mixed-domain, so the raw first/last record is not a reliable extent.
     * Then applies @ref findGaps.
     *
     * @param log          Composed device log.
     * @param resolver     Resolver built from @p log.
     * @param thresholdMs  Minimum gap width to report.
     * @return             Coverage gaps on the resolved timeline.
     */
    static std::vector<DataGap> detectGaps(const ISDeviceLog& log,
                                           const ISTimeResolver& resolver,
                                           uint64_t thresholdMs);

    // -----------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------

    /**
     * @brief Open a single segment.
     *
     * The segment format is recognized by extension: `.raw` or `.dat`
     * (D-119 / SN-8626). Sidecar discovery rule: replace the segment
     * extension with `.idx` (e.g. `LOG_..._0001.raw` → `LOG_..._0001.idx`,
     * `LOG_..._0001.dat` → `LOG_..._0001.idx`) — matches the writer
     * convention (cf. `cDeviceLog::OpenNewSaveFile`).
     *
     * @param raw  Path to the segment file (`.raw` or `.dat`).
     * @return     Reader on success; `ISErrorCode` on failure:
     *             `NotFound`, `PermissionDenied`, `Corrupted`, `Io`,
     *             `Unsupported` (unrecognized extension).
     */
    static ISExpected<ISLogReader> openSegment(const std::filesystem::path& raw);

    /**
     * @brief Determine a segment's absolute start/end time and how trustworthy that is, WITHOUT
     *        building or writing an index.
     *
     * Opens @p raw directly — bypassing sidecar discovery entirely — runs one byte scan with the
     * anchor collector attached, and returns the result. The segment's own `.idx` is neither
     * read nor written: not read, so the answer always comes from the bytes on disk rather than
     * from whatever an older writer recorded; not written, so calling this never mutates the log
     * directory. The reader is discarded on return. That combination is what makes the cascade
     * testable and queryable without a rebuild.
     *
     * @warning Do NOT implement this by delegating to @ref openSegment. That path persists a
     *          rebuilt sidecar when one is missing (see `persistIndex()`), which silently
     *          created `.idx` files in the caller's log directory — the exact side effect this
     *          entry point exists to avoid.
     *
     * @param raw   Path to the segment file (`.raw` or `.dat`).
     * @param prev  Analysis of the preceding segment, or `nullptr`. Supplies the chained-hint
     *              fallbacks (`BridgedToW`, `PrevSegmentChained`) for a segment that carries no
     *              absolute time of its own, and enables the durability-regression check.
     * @return      The analysis on success; `ISErrorCode` if the segment cannot be opened.
     *
     * @note SN-8629. Prefer the analysis carried on an already-open reader
     *       (`anchorAnalysis()`) when you have one — it was produced by the same scan that
     *       built the index, so asking for it costs nothing.
     */
    static ISExpected<AnchorAnalysis> analyzeSegment(const std::filesystem::path& raw,
                                                     const AnchorAnalysis* prev = nullptr);

    /**
     * @brief This segment's anchor analysis — its domain-normalized position on the timeline.
     *
     * Populated for every successfully-opened segment, by whichever route produced the record
     * index: the scan itself when the index was rebuilt, or @ref analyzeFromRecords when a
     * trusted sidecar made a scan unnecessary. Both routes run the same cascade and are
     * expected to agree; `test_anchor_analysis` asserts they do.
     *
     * This must hold for the sidecar path too, not just the rebuild path. `ISDeviceLog::
     * fromSegments` orders segments by `anchoredStartMs` and requires EVERY segment to carry an
     * orderable anchor before it will re-sort — so leaving the analysis empty whenever a valid
     * `.idx` was present (which is the common case for a captured log) left the ordering fix
     * dormant exactly where it was needed, silently falling back to filename order.
     */
    const AnchorAnalysis& anchorAnalysis() const noexcept { return anchor_; }

    /**
     * @brief Re-resolve this segment's anchor with its predecessor's analysis as a hint.
     *
     * A reader is constructed from one segment and knows nothing about its siblings, so its
     * initial analysis can only reach the tiers a segment can establish alone. The chained tiers
     * — `BridgedToW` (reuse the predecessor's offset because uptime is continuous) and
     * `PrevSegmentChained` — are by definition unavailable to it, as is the
     * durability-regression check. Whoever composes segments into a device log must supply that
     * context; `ISDeviceLog::fromSegments` does, walking the segments in filename order.
     *
     * Costs no I/O — it re-resolves from the existing record index. A segment already at the top
     * tier is left alone: a hint cannot improve it and it cannot have regressed.
     *
     * @param prev  Predecessor's analysis, or `nullptr` for the first segment.
     */
    void reanalyzeWithPrevious(const AnchorAnalysis* prev) {
        if (anchor_.tier == AnchorTier::PayloadToWBridge) return;
        analyzeFromRecords(prev);
    }

    /**
     * @brief Adopt an absolute-time offset established by ANOTHER segment of the same recording
     *        session, when this segment could not establish one itself.
     *
     * The `(ToW - uptime)` offset is a constant for a boot session: uptime and GPS time advance
     * together until the device reboots. So once ANY segment of a session pins that constant,
     * every other segment of the same session is anchored by it — including segments **earlier**
     * in the log than the one that supplied it. That is the point of this entry: a log whose
     * first five segments have no absolute time of their own, followed by one that acquires a
     * GPS fix, can have all six placed correctly rather than the first five being stranded.
     *
     * Self-established anchors are never overridden: a segment that pinned the constant from its
     * own payload keeps that, since it is first-hand evidence.
     *
     * @param offsetMs       The session's `(ToW - uptime)` constant.
     * @param donorDid       DID that established it, for the audit trail.
     * @param donorIsEarlier True when the donating segment precedes this one in the log.
     *
     * @note SN-8629 / SN-8704, Kyle 2026-09-19: "as soon as we have a TRUSTWORTHY absolute time
     *       - we can go back and re-anchor the entire log". Replaces a forward-only pairwise
     *       chain, which could only ever push information later in the log, never earlier.
     */
    /**
     * @brief D0096 path 2: fill `log_time_offset_ms` for every record from the payload
     *        timestamps that exist, distributing un-clocked records between their bookends.
     *
     * A file rebuild cannot observe receipt time, so every value written here is
     * RECONSTRUCTED and each record is flagged
     * `IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET` to say so. Only the ONE dominant domain
     * is used as bookends: interpolating between an uptime-domain and a GPS-ToW-domain
     * neighbour would yield a number that is a duration in neither frame.
     * `IS_LOG_IDX_HDR_FLAG_HAS_LOG_TIME_OFFSET` is declared only if values were actually
     * written -- declaring an all-zero field is the A5 defect this exists to avoid.
     *
     * No-op when no record carries a usable timestamp: there is nothing to reconstruct from,
     * and zeros must not be declared as a chronology.
     */
    void populateReconTimeOffsets();

    void adoptSessionOffset(int64_t offsetMs, uint32_t donorDid, bool donorIsEarlier);

    /**
     * @brief Parse the `YYYYMMDD_HHMMSS` field of a segment filename into Unix ms (UTC).
     *
     * The lowest rung of the anchor cascade: a log with no absolute time anywhere in its records
     * can still be placed on the timeline by the writer's filename pattern
     * (`LOG_SN<serial>_<YYYYMMDD>_<HHMMSS>_<NNNN>`). Returns 0 when the filename carries no
     * parseable date, which the cascade reads as "no filename anchor available".
     *
     * Public so the parse can be tested directly — it has a history of failing on serial-number
     * lengths that create a false date-shaped window earlier in the name.
     */
    static uint64_t filenameAnchorMs(const std::filesystem::path& p);

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
     * Direct accessor for the segment's underlying byte buffer. Despite
     * the name, this aliases whatever backing file was opened — `.raw`
     * or `.dat` (D-119) — the name predates `.dat` support.
     *
     * Most callers should iterate via `records()` / `allRecords()`
     * — this hatch is for code that needs to re-parse the raw stream
     * (e.g. D-03's templated sugar `extractTypedRange`, which extracts
     * typed payload structs the chunk-shared-offset semantics of
     * `ISRecordView::bytes()` don't expose cleanly; D-07's
     * `ISTimeResolver` scanning ToW-bearing payloads for sync points).
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
     *          units indicated by `header().ts_anchor`. Returns 0
     *          if the segment contains no records.
     */
    uint64_t segmentStartTimestamp() const noexcept;

    /**
     * @return  Latest record timestamp in this segment, in the units
     *          indicated by `header().ts_anchor`. Returns 0 if the
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

    /**
     * Returns the device's packed hardware id (`is_hardware_t`),
     * encoding hardware type + major + minor revs. Derived from the
     * first `dev_info_t`-bearing record via the same scan that
     * populates `deviceId()` — `DID_DEV_INFO`, `DID_GPX_DEV_INFO` or
     * `DID_EVB_DEV_INFO`, all of which carry the same payload struct.
     * (Before SN-8445 only `DID_DEV_INFO` was accepted, so a GPX-only
     * log yielded 0 here and rendered as `???-0.0::SN<serial>`.)
     * The filename fallback cannot produce an `hdwId`, so this still
     * returns 0 (`IS_HARDWARE_NONE`) for a log carrying NO device-info
     * record of any kind.
     *
     * Pair with `deviceId()` to form a canonical device label via
     * `utils::deviceIdString(hdwId(), deviceId())`.
     *
     * @return  Packed hardware id, or 0 if no DEV_INFO record was
     *          logged.
     */
    uint16_t hdwId() const noexcept { return hdwId_; }

    /**
     * Returns the full `dev_info_t` recovered by the device-id scan.
     *
     * `deviceId()` and `hdwId()` are both distillations of this struct, and
     * everything else it carries — firmware version, build number, build date
     * and time, protocol version, hardware type/rev, manufacturer, add-on info
     * — was previously parsed and discarded, leaving a consumer no way to report
     * a device beyond serial + hardware id. Retained for SN-8463 (Logalyzer
     * Devices-tab tooltips), which needs the firmware/build summary.
     *
     * Only populated by the DEV_INFO scan path. A log carrying no device-info
     * record of any kind (or one whose first such record has a zero serial)
     * leaves this default-constructed, so callers MUST gate on
     * @ref hasDevInfo before formatting it — an all-zero `dev_info_t` renders
     * as a plausible-looking but entirely fictitious device.
     *
     * @return  Const reference to the retained payload; all-zero if none was
     *          found.
     */
    const dev_info_t& devInfo() const noexcept { return devInfo_; }

    /**
     * Reports whether @ref devInfo carries a real parsed payload.
     *
     * True only when the scan found a `dev_info_t`-bearing record with a
     * non-zero serial. False when the serial came from the filename fallback,
     * which recovers nothing else.
     */
    bool hasDevInfo() const noexcept { return hasDevInfo_; }

    /**
     * @return  The on-disk format this segment was opened as (D-119 /
     *          SN-8626). `ISDeviceLog::fromSegments` uses this to reject
     *          a device log composed of mixed `.raw`/`.dat` segments.
     */
    SegmentFormat format() const noexcept { return format_; }

    /**
     * @return  The path this segment was opened from. Kyle 2026-09-07 (Option B):
     *          `ISTimeResolver`'s file-timestamp-anchor fallback (for a log with no
     *          payload-level sync at all) parses this path's filename/ancestor
     *          directory names for a timestamp, falling back to the file's
     *          last-write time when none parse.
     */
    const std::filesystem::path& path() const noexcept { return rawPath_; }

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
     * @brief Templated sugar — yields `const T&` payload references
     *        for compile-time-known DIDs (D-03 / SN-7894).
     *
     * The full definition lives in `ISLogReaderSugar.h`. Including
     * `ISLogReader.h` alone gives you the type-erased core
     * (`records(did_t)`); `#include "ISLogReaderSugar.h"` adds the
     * typed view.
     *
     * @tparam DID  Must have a `DIDTraits<DID>` specialization.
     *              Compile error otherwise.
     * @return      Eagerly-materialized range of `(TimeStamp, T)`
     *              pairs; iteration dereferences to `const T&`.
     */
    template <did_t DID>
    TypedRange<typename DIDTraits<DID>::type> records() const;

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
     * the `.raw` and persisting the result. `.raw`-specific — see
     * @ref buildIndexFromScanDat for the `.dat` equivalent (D-119).
     */
    //! SN-8629: when non-null, the collector is attached to the scan and the resulting
    //! analysis stamped into `anchor_` and the index header. Pass the previous segment's
    //! analysis to enable the chained-hint tiers.
    void buildIndexFromScan(const AnchorAnalysis* prev = nullptr, bool collectAnchor = true);


    /**
     * @brief Run the anchor cascade over an already-populated `records_`, without a byte scan.
     *
     * The route used when a trusted `.idx` sidecar made a scan unnecessary, and for `.dat`
     * segments (whose index build does not carry the collector). Every record contributes its
     * DID and timestamp — that is what the per-domain extrema and the stall detector are built
     * from — while only the few DIDs that can actually anchor
     * (`AnchorCollector::needsPayload`) have their payload materialized. The source is mmap'd
     * or fully buffered, so materializing one is pointer arithmetic plus a re-frame, not I/O.
     *
     * @param prev  Previous segment's analysis, or `nullptr`. Same role as in
     *              @ref buildIndexFromScan.
     */
    void analyzeFromRecords(const AnchorAnalysis* prev = nullptr);

    /**
     * @brief Open a segment for analysis only — no sidecar read, no sidecar write.
     *
     * Deliberately NOT @ref construct: that performs sidecar discovery and, on a miss, persists
     * the rebuilt index. @ref analyzeSegment promises neither, so it needs a source-only open.
     *
     * @param raw  Path to the segment file.
     * @return     A reader with its source, path, format and a default header set, and an empty
     *             record index; or an `ISError` if the file cannot be opened or its extension
     *             is unrecognized.
     */
    static ISExpected<ISLogReader> openForAnalysis(const std::filesystem::path& raw);

    /**
     * @brief `.dat` equivalent of @ref buildIndexFromScan (D-119 / SN-8626).
     *
     * `.dat`'s on-disk shape (`sChunkHeader`-framed chunks of
     * `p_data_hdr_t`/payload pairs — see `DeviceLogSerial.h`) is
     * self-delimiting, so this walks chunk/record headers directly
     * instead of running `.raw`'s `is_comm_parse_byte` state machine.
     * Populates `records_`/`byDid_` exactly like the `.raw` scan; a
     * malformed/truncated chunk or record stops the scan and marks
     * `isTruncated_`/`truncationOffset_`, mirroring D-04's `.raw`
     * behavior (including the sibling-segment discrimination via
     * `hasSiblingSuccessor`).
     */
    void buildIndexFromScanDat();

    /**
     * @brief Resolves the device id from the segment's `dev_info_t` records, falling back to filename parsing.
     *
     * Prefers `DID_DEV_INFO` (the logging device's own record). `DID_GPX_DEV_INFO` / `DID_EVB_DEV_INFO` describe an
     * attached peripheral and are adopted only when the segment carries no primary record — which is the GPX-only
     * capture case. Records with a zero serial are skipped rather than treated as terminal.
     *
     * Dispatches to @ref deriveDeviceIdDat for `.dat` segments (D-119); the body below is `.raw`-specific.
     *
     * @param rawPath  Path the segment was opened from. Used only for the filename-fallback path.
     * @note SN-8445 / SN-8463. The ranking (rather than first-match) is what keeps the segments of one mixed
     *       IMX+GPX log agreeing on a device id, which `ISDeviceLog::fromSegments` requires.
     */
    void deriveDeviceId(const std::filesystem::path& rawPath);

    /**
     * @brief `.dat` equivalent of the `.raw`-specific body of @ref deriveDeviceId (D-119 / SN-8626).
     *
     * Unlike `.raw` (whose `.idx` record `offset` is the packet START, not the payload — recovering the payload
     * requires re-running the full comm parser), a `.dat` record's `offset` IS its `p_data_hdr_t` start with no wire
     * framing in between. This walks the already-built `records_` (populated by the time this runs, whether from a
     * trusted on-disk `.idx` or from @ref buildIndexFromScanDat) rather than re-parsing chunk headers from scratch.
     * Same DID-ranking / zero-serial-skip rules as the `.raw` path (SN-8445), same filename fallback.
     *
     * @param rawPath  Path the segment was opened from. Used only for the filename-fallback path.
     */
    void deriveDeviceIdDat(const std::filesystem::path& rawPath);

    /**
     * @brief Adopts a `dev_info_t` as this segment's identity.
     *
     * Sets the device id, the packed hardware id, and retains the whole struct so consumers can report firmware and
     * build info rather than just serial plus hardware id.
     *
     * @param info  A `dev_info_t` with a non-zero serial number.
     * @note SN-8463 — the retained struct is what `devInfo()` / `hasDevInfo()` expose.
     */
    void adoptDevInfo(const dev_info_t& info) noexcept;

    /**
     * Computes the end-of-bytes offset for the record at the given
     * index. Used by `viewAt` to derive `bytes().second`.
     *
     * `.raw`: records sharing an offset (chunk-input artifact from the
     * writer) resolve to the same end offset — see the `.cpp` for the
     * "next differing offset" scan. `.dat` (D-119): each record is
     * self-delimiting (`p_data_hdr_t.size`), so the end is computed
     * directly with no scan.
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

    SegmentFormat                          format_             = SegmentFormat::Raw;
    bool                                   hadOnDiskIndex_     = false;

    //! SN-8629: anchor analysis from the scan that built this index; `None` tier when the
    //! index came off disk instead.
    AnchorAnalysis anchor_{};
    bool                                   isTruncated_        = false;
    uint64_t                               truncationOffset_   = 0;
    uint64_t                               deviceId_           = 0;
    uint16_t                               hdwId_              = 0;
    dev_info_t                             devInfo_            {};
    bool                                   hasDevInfo_         = false;
    std::vector<std::string>               warnings_;
    std::filesystem::path                  rawPath_;
    std::filesystem::path                  idxPath_;

    friend class RangeIterator;
    friend class Range;
};

} // namespace inertial_sense
