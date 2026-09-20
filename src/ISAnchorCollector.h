/**
 * @file ISAnchorCollector.h
 * @brief Single-pass sink that accumulates anchor candidates during a `.raw` byte scan.
 *
 * `AnchorCollector` is fed one decoded ISB record at a time and yields an `AnchorAnalysis` when
 * the scan completes. It is a *sink*, not a scanner: it holds no file, does no I/O, and does
 * not allocate a record index. That separation is what lets the same byte pass serve both the
 * index build and the anchor analysis, so building an index never costs a second scan, while
 * `ISLogReader::analyzeSegment()` can run the collector alone with no index side effects.
 *
 * Deliberately a forward accumulator rather than a search. A bisection over the raw bytes would
 * need ToW-validity to be monotone in the file, and it is not: a receiver that drops out
 * mid-segment goes valid -> invalid while continuing to emit a frozen time-of-week, so a
 * bisection can land after the dropout and anchor to a stale value. The forward pass sees the
 * transition instead, and costs nothing extra because the index build already pays for it.
 *
 * @note SN-8629.
 */

#ifndef IS_ANCHOR_COLLECTOR_H
#define IS_ANCHOR_COLLECTOR_H

#include "ISAnchorAnalysis.h"

#include <cstdint>
#include <map>

namespace inertial_sense {

/**
 * @brief Accumulates per-segment anchor candidates and anomalies across one byte scan.
 *
 * Usage:
 * @code
 *   AnchorCollector c;
 *   c.setFilenameAnchorMs(anchorFromName(path));   // optional lowest-tier fallback
 *   for (each decoded record) c.consume(did, structOffset, payload, size, recordTsMs);
 *   AnchorAnalysis a = c.finish(prevAnalysisOrNull);
 * @endcode
 */
class AnchorCollector {
public:
    /**
     * @brief Offer one decoded ISB record to the collector.
     *
     * @param did           Record's data ID.
     * @param structOffset  `dataHdr.offset` — the payload's offset within its struct. Anchor
     *                      candidates are only read from whole records (offset 0); a partial
     *                      record does not carry the needed fields at the expected positions.
     * @param payload       Pointer to the record payload, or `nullptr` when the caller has not
     *                      materialized it. A null payload still contributes the record to the
     *                      per-domain extrema, the stall detector and the running uptime a
     *                      ToW-only anchor correlates against — only the payload-reading tiers
     *                      are skipped. See `needsPayload()`.
     * @param payloadSize   Bytes available at @p payload.
     * @param recordTsMs    The record's index timestamp in ms, in whichever domain the DID
     *                      stamps (0 when the record carries no internal time).
     */
    void consume(uint32_t did, uint16_t structOffset, const uint8_t* payload,
                 uint32_t payloadSize, uint64_t recordTsMs);

    /**
     * @brief Resolve the cascade and produce the analysis.
     *
     * @param prev  Analysis of the preceding segment, or `nullptr` when this is the first
     *              segment (or the caller has none). Supplies the `BridgedToW` and
     *              `PrevSegmentChained` fallbacks and the durability-regression check.
     * @return      The completed analysis. Never fails; a segment with nothing usable comes
     *              back as `AnchorTier::None` with an anomaly explaining why.
     */
    AnchorAnalysis finish(const AnchorAnalysis* prev);

    /**
     * @brief Supply the absolute time parsed from the segment filename, in Unix ms.
     *
     * Used only as the lowest tier, when the segment carries no in-log absolute time and there
     * is no previous segment to chain from. Pass 0 (the default) when the filename has no
     * parseable timestamp.
     */
    void setFilenameAnchorMs(uint64_t ms) noexcept { filenameAnchorMs_ = ms; }

    /** @return  Number of records offered so far. */
    std::size_t recordsSeen() const noexcept { return seen_; }

    /**
     * @brief Whether @p did's payload has to be materialized for the cascade to use it.
     *
     * Lets a caller working from an existing index re-frame only the records that can actually
     * anchor, instead of every record in the segment. Records for which this is false still must
     * be offered to `consume()` (with a null payload) — they carry the extrema and stall
     * evidence.
     */
    static bool needsPayload(uint32_t did) noexcept;

private:
    //! Consecutive identical-timestamp records from one DID before it is reported as stalled.
    //! A handful of repeats is normal for a DID output faster than its time field's resolution;
    //! a long run means the source's clock stopped while it kept emitting.
    static constexpr std::size_t kStallReportThreshold = 32;

    //! Upper bound on retained candidates. A 1 Hz bridge over a long segment would otherwise
    //! accumulate thousands of near-identical claims; consensus only needs enough to establish
    //! a majority and name the dissenters. One claim per (DID, distinct offset) is kept, so
    //! this is reached only by a source whose offset is genuinely wandering.
    static constexpr std::size_t kMaxCandidates = 64;

    //! Per-DID timestamp-stall tracking.
    struct StallState {
        uint64_t    lastTsMs     = 0;
        std::size_t count        = 0;
        std::size_t repeats      = 0;
        std::size_t worstRepeats = 0;
    };

    /**
     * @brief Record one absolute-time claim.
     *
     * Deliberately NOT first-wins. Every distinct claim is retained so `finish()` can rank by
     * authority, find a majority, and name the outliers — none of which is possible if the
     * first valid candidate short-circuits collection. Duplicate claims from the same DID at
     * the same offset are folded (they corroborate rather than add information).
     *
     * @param tier      Authority of this claim.
     * @param did       DID making it.
     * @param towMs     Claimed GPS time-of-week.
     * @param upMs      Paired uptime, or 0 when the claim carries none.
     */
    void offerCandidate(AnchorTier tier, uint32_t did, uint64_t towMs, uint64_t upMs);

    //! Resolve `candidates` into a winner + an `AnchorConsensus`, appending any anomalies.
    //! Implements Kyle's option (b): highest authority wins outright; an equal-authority
    //! disagreement with no majority is `DisagreedUnresolved` and is raised loudly.
    void resolveConsensus();

    //! Update per-DID stall state and remember the most recent uptime-domain timestamp (the
    //! correlation partner for a ToW-only anchor).
    void trackStall(uint32_t did, uint64_t recordTsMs);

    /** @return  True when @p did stamps GPS time-of-week but carries no uptime to pair it with. */
    static bool isTowOnlyCandidate(uint32_t did) noexcept;

    AnchorAnalysis out_{};
    std::size_t    seen_ = 0;

    //! The winning claim, filled in by resolveConsensus().
    uint32_t winnerDid_   = 0;
    uint64_t winnerTowMs_ = 0;
    uint64_t winnerUpMs_  = 0;
    int64_t  winnerOffMs_ = 0;
    bool     haveWinner_  = false;

    // Tier 4 — ToW-only, paired with the nearest preceding uptime record.
    uint32_t towOnlyDid_ = 0;
    uint64_t towOnlyTowMs = 0;
    uint64_t towOnlyUpMs  = 0;

    uint64_t lastUptimeMs_    = 0;
    uint64_t filenameAnchorMs_ = 0;

    std::map<uint32_t, StallState> stalls_;
};

}  // namespace inertial_sense

#endif  // IS_ANCHOR_COLLECTOR_H
