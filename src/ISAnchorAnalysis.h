/**
 * @file ISAnchorAnalysis.h
 * @brief Per-segment time-anchor analysis — the ordering key for multi-segment logs.
 *
 * A `.raw` segment carries record timestamps in more than one time domain (D0066): some DIDs
 * stamp GPS time-of-week, others stamp host uptime. Before SN-8629 the segment's index header
 * took `first_timestamp_ms` / `last_timestamp_ms` from whichever record happened to be
 * physically first and last in the file, in whatever domain that record used — so the value
 * was a coin flip on DID write order. `ISDeviceLog::fromSegments` then sorted segments by that
 * key and could place them out of order, manufacturing time jumps and rewinds that are not in
 * the data.
 *
 * `AnchorAnalysis` replaces that with a domain-normalized key plus an explicit statement of how
 * trustworthy it is. It is produced by `ISLogReader::analyzeSegment()`, which is deliberately
 * side-effect free: it writes no `.idx`, allocates no record index, and can be called on a
 * segment purely to ask "what time does this segment start, and how sure are we?".
 *
 * @note SN-8629. See also `ISTimeResolver`, which performs the equivalent bridge but is built
 *       from an already-composed `ISDeviceLog` — strictly downstream of the ordering decision
 *       that needs it, which is why it cannot be used here.
 */

#ifndef IS_ANCHOR_ANALYSIS_H
#define IS_ANCHOR_ANALYSIS_H

#include <cstdint>
#include <string>
#include <vector>

namespace inertial_sense {

/**
 * @brief How a segment's absolute start time was established, ordered by trustworthiness.
 *
 * The numeric order is meaningful: a higher value is a more durable anchor. Comparing tiers
 * across adjacent segments is the anomaly test — a segment whose anchor is *less* durable than
 * its predecessor's is reported so a mitigation can walk down the ladder deliberately rather
 * than silently accepting a weaker key.
 *
 * Both `PayloadToWBridge` candidates are equally trustworthy by design: `DID_SYS_PARAMS` is
 * the IMX bridge and `DID_GPX_STATUS` is the GPX equivalent, and a GPX-sourced log may never
 * contain a `DID_SYS_PARAMS` at all. Whichever appears earliest in the segment wins; the two
 * disagreeing is itself an anomaly.
 */
enum class AnchorTier : uint8_t {
    //! No anchor of any kind. The segment's timestamps are session-relative and cannot be
    //! ordered against anything but themselves.
    None = 0,

    //! Absolute time taken from the `YYYYMMDD_HHMMSS` field of the segment filename. Durable
    //! only to the extent the writer's clock was set; no per-record correspondence.
    FilenameAnchor = 1,

    //! No absolute time in this segment. Start assumed to continue from the previous segment's
    //! end plus the rollover gap. Accumulates error across a run of unanchored segments.
    PrevSegmentChained = 2,

    //! No usable ToW-bearing record in this segment, but the previous segment's
    //! (ToW - uptime) offset applies because this segment's uptime is continuous with it.
    BridgedToW = 3,

    //! A validity-gated ToW-bearing record (`DID_GNSS1/2_POS`, `DID_INS_1..4`). Carries GPS
    //! time only, so recovering the offset requires correlating it against a neighbouring
    //! uptime-domain record — one correlation step away from exact.
    PayloadToWSingle = 4,

    //! A validity-gated record carrying BOTH GPS time-of-week and host uptime in one payload
    //! (`DID_SYS_PARAMS`, or `DID_GPX_STATUS` on the GPX side). The offset falls out of a
    //! single record with no correlation, making this the highest-confidence anchor.
    PayloadToWBridge = 5,
};

/** @return  Human-readable name for @p t, for logging and UI. */
const char* anchorTierName(AnchorTier t) noexcept;

/**
 * @brief Everything known about one segment's placement on an absolute timeline.
 *
 * Produced by `ISLogReader::analyzeSegment()`. Consumed by `ISLogReader::buildIndexFromScan()`
 * (which stamps `anchoredStartMs` / `anchoredEndMs` into the index header instead of an
 * arbitrary record's raw timestamp) and by `ISDeviceLog::fromSegments()` (which orders by
 * `anchoredStartMs` and refuses to order on anything below `minimumOrderableTier`).
 */
struct AnchorAnalysis {
    //! How `anchoredStartMs` was established. `None` means it was not.
    AnchorTier tier = AnchorTier::None;

    //! DID of the record the anchor came from; 0 when `tier` is `None`, `FilenameAnchor` or
    //! `PrevSegmentChained` (no record involved).
    uint32_t anchorDid = 0;

    //! GPS time-of-week of the anchor record, ms. 0 when no ToW-bearing anchor was found.
    uint64_t anchorTowMs = 0;

    //! Host uptime of the anchor record, ms. Only populated for `PayloadToWBridge`, where a
    //! single record carries both domains.
    uint64_t anchorUptimeMs = 0;

    //! `anchorTowMs - anchorUptimeMs`, the constant that maps this segment's uptime-domain
    //! record timestamps onto the absolute frame. Carried forward as the hint to the next
    //! segment's analysis.
    int64_t offsetMs = 0;

    //! Extrema of the segment's uptime-domain record timestamps. These are the values the
    //! index header should be derived from — NOT `records_.front()/back().timestamp`, which
    //! are positional and domain-agnostic.
    uint64_t uptimeMinMs = 0;
    uint64_t uptimeMaxMs = 0;

    //! Extrema of the segment's ToW-domain record timestamps, where any were seen.
    uint64_t towMinMs = 0;
    uint64_t towMaxMs = 0;

    //! The segment's span on the absolute timeline: `uptimeMin/MaxMs + offsetMs`. This is the
    //! ordering key. Zero and meaningless when `tier == None`.
    uint64_t anchoredStartMs = 0;
    uint64_t anchoredEndMs   = 0;

    //! Record counts by domain, and records carrying no internal timestamp at all.
    std::size_t uptimeRecords = 0;
    std::size_t towRecords    = 0;
    std::size_t untimedRecords = 0;

    //! Conditions worth surfacing: a stalled timestamp on a still-flowing DID, a DID that
    //! stops appearing mid-segment, disagreement between the IMX and GPX bridges, a tier
    //! weaker than the previous segment's. Never fatal on their own — they are what the
    //! application layer calls out instead of drawing through.
    std::vector<std::string> anomalies;

    /** @return  True when `anchoredStartMs` is meaningful and may be used for ordering. */
    bool anchored() const noexcept { return tier != AnchorTier::None; }

    /**
     * @brief Weakest tier that still yields a usable ordering key.
     *
     * `PrevSegmentChained` and above place a segment on the absolute frame well enough to sort
     * against its siblings. `FilenameAnchor` is included because the writer's filename pattern
     * is timestamp-sortable by construction (D0051), which is the same information a
     * filename-lexicographic sort would use — just made explicit.
     */
    static constexpr AnchorTier minimumOrderableTier = AnchorTier::FilenameAnchor;
};

}  // namespace inertial_sense

#endif  // IS_ANCHOR_ANALYSIS_H
