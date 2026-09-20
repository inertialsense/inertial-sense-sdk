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

#include "ISTimeStamp.h"

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
 * @brief The `TimeSource` an anchored value established at tier @p t may honestly claim.
 *
 * Audit A2: the span accessors used to assert `TimeSource::PayloadToW` unconditionally, which
 * is a D0024/D0066 contract violation on any log that never carried a time-of-week — measured
 * on an uptime-only log that reached `FilenameAnchor` and still reported `PayloadToW`. The tag
 * is DERIVED from the cascade's own verdict here rather than asserted at each call site, so
 * the three layers that fold spans cannot disagree about it.
 *
 * - `PayloadToWSingle` / `PayloadToWBridge` — a real payload time-of-week ⇒ `PayloadToW`.
 * - `BridgedToW` / `PrevSegmentChained` — the anchor was reconstructed from a sibling
 *   segment's evidence rather than witnessed here ⇒ `ResolvedViaSync`.
 * - `FilenameAnchor` — a coarse wall-clock off the filename ⇒ `FileTimeAnchored`, which exists
 *   precisely so consumers keep such records instead of discarding them as unanchored.
 * - `None` — no anchor at all ⇒ `SessionOnly`.
 */
TimeSource timeSourceForTier(AnchorTier t) noexcept;

/**
 * @brief Compose an anchored value and its tier into a `TimeStamp`, via the right factory.
 *
 * Goes through `TimeStamp::from*` rather than brace-initializing, because each factory also
 * fixes the `TimeConfidence` that belongs to its source — notably `Unknown` for
 * `FileTimeAnchored` and `SessionOnly`, which `RawSeriesBuilder`'s "carries no real-world
 * anchor" filter keys on together with the source. Hand-composing the struct is how those two
 * fields drift apart.
 *
 * @param ms        Value on the absolute frame (or the raw session value at tier `None`).
 * @param t         Tier the anchor was established at.
 * @param deviceId  Owning device, or 0 for a fold across devices.
 */
TimeStamp anchoredTimeStamp(uint64_t ms, AnchorTier t, uint64_t deviceId) noexcept;

/**
 * @brief Everything known about one segment's placement on an absolute timeline.
 *
 * Produced by `ISLogReader::analyzeSegment()`. Consumed by `ISLogReader::buildIndexFromScan()`
 * (which stamps `anchoredStartMs` / `anchoredEndMs` into the index header instead of an
 * arbitrary record's raw timestamp) and by `ISDeviceLog::fromSegments()` (which orders by
 * `anchoredStartMs` and refuses to order on anything below `minimumOrderableTier`).
 */
/**
 * @brief How well a segment's anchor is CORROBORATED — orthogonal to how it was established.
 *
 * `AnchorTier` ranks the *kind* of evidence. This ranks *agreement among* evidence, which is a
 * different question: a lone bridge record, two bridge records that agree, and two bridge
 * records that disagree are three very different confidence states that all reach
 * `AnchorTier::PayloadToWBridge`. Collapsing them lost exactly the finding an analysis tool
 * exists to surface — two clocks that should converge (allowing for drift) and don't.
 *
 * @note SN-8629 / SN-8704. Kyle 2026-09-19: always anchor to the most trustworthy source;
 *       raise the alarm only when sources of EQUAL authority disagree. A lesser authority
 *       that conflicts still earns a warning, but nothing blocks the user or the application.
 */
enum class AnchorConsensus : uint8_t {
    //! No absolute anchor was established, so there is nothing to corroborate.
    NotApplicable = 0,

    //! Exactly one candidate was found. Nothing contradicts it — and nothing confirms it.
    Uncorroborated,

    //! Two or more candidates agree within tolerance. The strongest state available.
    Corroborated,

    //! Candidates disagree, but they differ in authority, so the most trustworthy one was
    //! taken and the dissenter recorded as an anomaly. Not a blocker (Kyle's option (b)).
    DisagreedResolvedByAuthority,

    //! Candidates of EQUAL authority disagree and no majority exists to break the tie. The
    //! anchor is still populated — from the first of the tied candidates, so behaviour stays
    //! deterministic — but this is the state that must be raised loudly.
    DisagreedUnresolved,
};

/** @return  Human-readable name for @p c, for logging and UI. */
const char* anchorConsensusName(AnchorConsensus c) noexcept;

/**
 * @brief One absolute-time claim made by one record, before consensus is resolved.
 *
 * Candidates are collected across the whole segment rather than "first valid one wins", which
 * is what makes the majority/outlier logic possible at all.
 */
struct AnchorCandidate {
    AnchorTier tier       = AnchorTier::None;  //!< Authority of this claim.
    uint32_t   did        = 0;                 //!< DID that made it.
    uint64_t   towMs      = 0;                 //!< Claimed GPS time-of-week, ms.
    uint64_t   uptimeMs   = 0;                 //!< Paired uptime, ms (0 if none).
    int64_t    offsetMs   = 0;                 //!< `towMs - uptimeMs`, the comparable quantity.
    bool       accepted   = false;             //!< True for the candidate that supplied the anchor.
};

struct AnchorAnalysis {
    //! How `anchoredStartMs` was established. `None` means it was not.
    AnchorTier tier = AnchorTier::None;

    //! How well that anchor is corroborated by OTHER candidates. Independent of `tier`.
    AnchorConsensus consensus = AnchorConsensus::NotApplicable;

    //! Every absolute-time claim seen in the segment, in the order encountered, with the one
    //! that won marked `accepted`. Retained so a caller can show the user what disagreed and
    //! offer a re-anchor (D0095's "call it out, list them, let the user re-anchor").
    std::vector<AnchorCandidate> candidates;

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
     * @brief True when this anchor was established from the segment's OWN payload.
     *
     * First-hand evidence. Only such a segment may donate its offset to its session's siblings —
     * a segment that itself inherited one is a relay, not a source, and treating it as a source
     * would let a single weak claim propagate indefinitely while looking corroborated.
     */
    bool firstHand() const noexcept { return tier >= AnchorTier::PayloadToWSingle; }

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

/**
 * @brief One segment's instruction to adopt an offset established elsewhere in its session.
 *
 * Output of @ref planSessionAdoptions.
 */
struct SessionAdoption {
    std::size_t segment        = 0;      //!< Index of the segment that should adopt.
    int64_t     offsetMs       = 0;      //!< The session's `(ToW - uptime)` constant.
    uint32_t    donorDid       = 0;      //!< DID that established it.
    bool        donorIsEarlier = false;  //!< True when the donor precedes @ref segment.
};

/**
 * @brief Decide which segments should adopt which session offset — PURE, no I/O.
 *
 * The `(ToW - uptime)` offset is constant for a boot session: uptime and GPS time advance
 * together until the device reboots. So once ANY segment of a session pins that constant, every
 * other segment of the same session is anchored by it — **including segments earlier in the log
 * than the one that supplied it**. That bidirectionality is the point: a log whose first five
 * segments carry no absolute time, followed by a sixth that acquires a GPS fix, can have all six
 * placed correctly instead of the first five being stranded.
 *
 * Session boundaries come from uptime RESETS. Uptime rises monotonically within a session, so a
 * segment whose uptime starts below its predecessor's means the device rebooted, and the offset
 * must NOT cross that boundary (D0069 §4 / SN-8339 — the offset is per-boot-session).
 *
 * Donor selection is authority-before-position, matching the within-segment consensus rule:
 * strongest tier, then best-corroborated, then earliest. Only `firstHand()` analyses may donate.
 *
 * Kept pure so the policy is testable without building an index or touching a file — the same
 * reason `ISLogReader::findGaps` is pure.
 *
 * @param perSegment  Per-segment analyses **in log (filename) order**.
 * @return            Adoption instructions; segments that need none are absent. A segment that
 *                    already has a first-hand anchor, or that has no uptime-domain records to
 *                    project onto, never appears.
 *
 * @note SN-8629 / SN-8704. Replaces a forward-only pairwise chain that could only push
 *       information later in the log, never earlier.
 */
std::vector<SessionAdoption>
    planSessionAdoptions(const std::vector<AnchorAnalysis>& perSegment);

}  // namespace inertial_sense

#endif  // IS_ANCHOR_ANALYSIS_H
