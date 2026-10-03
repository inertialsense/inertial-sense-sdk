/**
 * @file ISTimeResolver.h
 * @brief Piecewise-linear wall-clock reconstruction over a device log.
 *
 * D-07 / SN-7897 / D0024 / D0025 / D0026: produces a `TimeStamp` for
 * any record in a device log, including pre-fix records that lack
 * payload ToW. Sync points are records that carry HAS_TOW; queries
 * outside the sync-anchored region are extrapolated forward / backward.
 *
 * **v2 `.idx` constraint.** The writer overwrites a record's stored
 * `.idx` timestamp with the payload ToW when the HAS_TOW flag is set —
 * host-side capture time isn't preserved alongside ToW for sync points
 * in the .idx itself. The resolver models a single time axis (the
 * `.idx` `timestamp` field) where:
 *   - **Sync records** (HAS_TOW=1): stored timestamp = payload ToW
 *     (ms). These are the anchor points.
 *   - **Non-sync records** (HAS_TOW=0): stored timestamp = host
 *     uptime-domain time-offset (ms since logger session start).
 * These two clusters typically don't overlap on the same numeric
 * axis (host uptime is a few seconds; ToW is ~hundreds of millions
 * of ms into the GPS week). The host-side timestamp at sync time is
 * recovered at scan time from the .raw byte stream — the writer emits
 * records in arrival order on one host thread, so a sync record's
 * host-time is ms-adjacent to the most recent non-sync record's
 * payload timestamp. `ISSyncPoint::actualHostTimeMs` carries that
 * recovered host-time, and `resolve()` uses it to bridge session-uptime
 * queries into the ToW frame (SN-8107 / D0066).
 *
 * **Pure C++17.** No Qt, no exceptions; fallible paths return
 * `ISExpected<T>`.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISDeviceLog.h"
#include "ISError.h"
#include "ISSyncPoint.h"
#include "ISTimeStamp.h"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

namespace inertial_sense {

/**
 * @brief Which mechanism produced an @ref AbsTimeResult.
 *
 * Ordered strongest first. Returned rather than kept internal so a caller can see WHY a time is
 * what it is — the thing that was impossible with `resolve()` alone.
 */
enum class AbsTimeMechanism : uint8_t {
    /**
     * @brief The record HAS a time, and nothing available could place it on a clock.
     *
     * The honest "we failed" value. Distinct from @ref NoTimeField since 2026-10-03: the two were
     * one bucket, which made the failure count unreadable — a log full of DIDs that simply do not
     * carry a time looked identical to a log whose timed records could not be anchored, and only
     * the second is a problem anyone can act on.
     */
    Unresolved = 0,
    /**
     * @brief The record's DID declares no timestamp field at all, so there was never a time to place.
     *
     * Not a failure, and not actionable: `DID_DEV_INFO` and `DID_FLASH_CONFIG` are supposed to have
     * no time. Counting these as unresolved inflated the figure on every real log — 7,105 of
     * 130,253 records on `ppd_LogQAQW713/20260716_000102`, spread over 15 DIDs, none of them a
     * defect.
     */
    NoTimeField,
    PayloadEpoch,      //!< The record's own payload carried a plausible week AND a time of week.
    CarriedWeek,       //!< This record's time of week, with a week from an earlier record.
    SyncMatched,       //!< Uptime mapped to the ToW frame via a sync point, then weeked.
    UptimeProjected,   //!< Uptime mapped via the segment anchor's offset, then weeked.
    FileAnchor,        //!< No usable week anywhere; placed from the segment filename.
    //! The record's own time field is DEAD (constant across every record of its DID), so the
    //! instant was interpolated from the nearest live records either side of it in arrival order.
    InterpolatedFromNeighbours,
    //! A frozen-field record with no live record either side of it, so nothing bounded it.
    FrozenAndUnbounded,
};

/** @brief Names an @ref AbsTimeMechanism. @param m The mechanism. @return Its name. */
const char* absTimeMechanismName(AbsTimeMechanism m) noexcept;

/**
 * @brief Where the absolute reference came from, independent of how the record was mapped onto it.
 *
 * Ordered strongest to weakest. `None` is an EXPECTED, first-class outcome, not a failure: a log
 * may legitimately have no means to anchor to any clock source (Kyle, 2026-10-02), and when that
 * happens the record still gets a relative clock — see `AbsTimeResult::relativeOnly`.
 */
enum class AbsAnchorSource : uint8_t {
    None = 0,          //!< No clock source of any kind. Relative time only, and that is fine.
    PayloadWeek,       //!< A GPS week from the log's own payloads, at or above the fix threshold.
    IdxCaptureEpoch,   //!< `.idx` header `capture_epoch_ms` — the host's wall-clock at log-open.
    Filename,          //!< The `YYYYMMDD_HHMMSS` in the segment filename. Last resort.
};

/** @brief Names an @ref AbsAnchorSource. @param a The source. @return Its name. */
const char* absAnchorSourceName(AbsAnchorSource a) noexcept;

/**
 * @brief The GPS week at or above which a log is taken to have achieved a GNSS fix.
 *
 * Kyle, 2026-10-02: *"any log which never achieves a week >= 1500 is a reasonable assertion that
 * the log never achieves a GNSS fix."* Week 1500 is late 2008, comfortably before any IMX existed,
 * so a smaller week is a device reporting that it does not know rather than a date. Week 1 is the
 * value the customer corpus actually carries, and week 1 is 1980-01-13.
 */
inline constexpr uint32_t kGnssFixWeekThreshold = 1500;

/**
 * @brief One record's probable absolute time, with the full shape of how it was determined.
 *
 * SN-8784. The point of returning provenance alongside the answer is that a timestamp nobody can
 * explain is a timestamp nobody can trust: the `hints` and the tier/week/offset fields are what
 * let a reader — or a test — say which mechanism fired and what it was given.
 */
struct AbsTimeResult {
    //! The answer, in Unix epoch milliseconds. Meaningful only when @ref valid.
    uint64_t         absoluteMs = 0;
    //! `true` when @ref absoluteMs is meaningful. `false` is an expected outcome, not an error.
    bool             valid      = false;
    /**
     * @brief `true` when no absolute anchor existed, but the relative fields ARE meaningful.
     *
     * The deliberate middle state. A log with no clock source still has a basis for a relative
     * clock, and saying so is more useful than reporting nothing — provided the caller cannot
     * mistake it for an absolute. Mutually exclusive with @ref valid.
     */
    bool             relativeOnly = false;
    AbsAnchorSource  anchorSource = AbsAnchorSource::None;
    AbsTimeMechanism mechanism  = AbsTimeMechanism::Unresolved;
    TimeSource       source     = TimeSource::SessionOnly;
    TimeConfidence   confidence = TimeConfidence::Unknown;

    // ---- how it got there -------------------------------------------------------------------
    std::size_t segmentIndex = 0;          //!< Index into `ISDeviceLog::segment(i)`.
    AnchorTier  anchorTier   = AnchorTier::None;
    uint64_t    anchorMs     = 0;          //!< The segment anchor used, in the ToW frame.
    int64_t     offsetMs     = 0;          //!< Cascade offset applied to a raw uptime value.
    uint32_t    gpsWeek      = 0;          //!< The week used to leave the ToW frame.
    bool        weekFromPayload = false;   //!< `false` when the week was carried or derived.
    uint64_t    towMs        = 0;          //!< The record's instant within the GPS week.
    uint64_t    sidecarRawMs = 0;          //!< What the `.idx` said, before any mapping.
    /**
     * @brief `true` when this record's DID has a STUCK time field.
     *
     * Detected by variance, not range: a value constant across every record of its DID carries no
     * time information, whatever its magnitude. Measured on `20260916_232611` — the known stuck-GPX
     * log — where all 1,539 `DID_GPX_*` records report an identical 342,615,500 while the IMX
     * clock beside them advances over 699.6 s.
     *
     * Range would be the wrong test: a genuine second-device uptime (one device rebooted, the
     * other did not) also sits far outside its sibling's range and IS valid (Kyle, 2026-10-02).
     */
    bool        frozenField = false;

    //! Absolute minus the log's / segment's own anchored start. Relative time is a SUBTRACTION,
    //! never its own mechanism — which is the point of the exercise.
    uint64_t relativeToLogMs     = 0;
    uint64_t relativeToSegmentMs = 0;

    //! Human-readable notes on anything irregular: an implausible week, a cleared ToW-valid bit,
    //! a sidecar that disagrees with the payload. Empty is the ordinary case.
    std::vector<std::string> hints;
};

/**
 * @brief How exactly a time mapped back onto a record position.
 *
 * Answers one question — how does the target relate to the instant of the record that came back —
 * so the values are mutually exclusive. The separate question of whether that instant is shared by
 * several records is `SegmentOffset::runLength`, because it is orthogonal: a target can be inexact
 * AND land on a stalled run, and folding both into this enum loses one of them.
 */
enum class PositionExactness : uint8_t {
    NotInLog = 0,       //!< Nothing in the log can be placed, so there is no position to return.
    Exact,              //!< A record bears exactly this instant.
    FirstOfStalledRun,  //!< A record bears exactly this instant, and others share it.
    /**
     * @brief No record bears this instant; this is the nearest one BEFORE it.
     *
     * The ordinary case for an arbitrary time — a scrub position, a marker, a midpoint — which
     * falls between two records. Added 2026-10-03: before it existed the inexact landing was
     * reported as `Exact`, which is false, and the caller had no way to tell a hit from a
     * round-back. `absoluteMs - resolveAbsTime(returned position)` is the amount rounded off.
     */
    Preceding,
    Before,             //!< Earlier than the EARLIEST record; clamped to it.
    After,              //!< Later than the LATEST record; clamped to it.
};

/** @brief Names a @ref PositionExactness. @param e The value. @return Its name. */
const char* positionExactnessName(PositionExactness e) noexcept;

/**
 * @brief A time mapped back to a place in the log.
 *
 * Carries index AND path AND handle deliberately: the handle is the convenient form, but
 * `ISDeviceLog::segments_` is a vector, so index plus path is the pair that stays meaningful
 * across a reload or in a serialised test expectation.
 */
struct SegmentOffset {
    bool                  valid        = false;
    std::size_t           segmentIndex = 0;
    std::filesystem::path segmentPath;
    const ISLogReader*    segment      = nullptr;   //!< Valid while the log lives.
    uint64_t              byteOffset   = 0;
    std::size_t           recordIndex  = 0;   //!< Index within that segment; pairs with
                                              //!< `segmentIndex` for `resolve`.
    uint64_t              arrivalIndex = 0;   //!< Log-wide, for callers that need it.
    PositionExactness     exactness    = PositionExactness::NotInLog;
    /**
     * @brief How many records share the instant this position landed on. `1` is the ordinary case.
     *
     * Orthogonal to @ref exactness, and separate from it for that reason: a stalled clock parks
     * many records on one instant, and a caller needs to know that whether or not the target hit
     * the instant exactly. The returned position is always the FIRST of the run, so a caller
     * wanting the rest reads `recordIndex .. recordIndex + runLength - 1` only when the run lies
     * within one segment — it need not, so iterate forward and compare instants instead.
     *
     * Zero when @ref valid is `false`.
     */
    std::size_t           runLength    = 0;
};

class ISTimeResolver {
public:
    //! Default discontinuity threshold: 1000 ppm clock-drift equivalent.
    //! If the slope between two adjacent sync-point pairs differs by
    //! more than this fraction, the boundary is reported via
    //! `discontinuities()`.
    //!
    //! @note D0024.
    static constexpr double kDefaultDiscontinuityThreshold = 1.0e-3;

    /**
     * @brief Marks a clock-correction event between two sync segments.
     *
     * Surfaced via `discontinuities()` so UI consumers (D-58 playback
     * strip, time-axis confidence display) can annotate the timeline
     * honestly rather than smoothing over a real jump.
     */
    struct Discontinuity {
        //! Host-time of the boundary (the second sync point's host).
        uint64_t hostTimeMs;
        //! Slope (ToW-ms per host-ms) over the segment ending at
        //! `hostTimeMs`. Identity (1.0) for .idx HAS_TOW-only logs.
        double   slopeBefore;
        //! Slope (ToW-ms per host-ms) over the segment starting at
        //! `hostTimeMs`.
        double   slopeAfter;
    };

    /**
     * @brief SN-8339: one power-on session, delimited by a `SYS_PARAMS.upTime`
     *        drop (device reboot) in arrival order.
     *
     * Each session owns its own uptime->ToW offset because uptime resets to ~0
     * on every boot while GPS ToW continues — so a single global offset (the
     * pre-SN-8339 model) mis-resolves every session after the first, and a small
     * session-uptime value can't pick its session by value alone. The
     * arrival-keyed `resolve()` overload selects the session whose
     * `[arrivalStart, arrivalEnd]` window (in the device's global record-arrival
     * order) contains the record's arrival index.
     */
    struct Session {
        uint64_t arrivalStart        = 0;      //!< first record's global arrival index (inclusive)
        uint64_t arrivalEnd          = 0;      //!< last record's global arrival index (inclusive)
        int64_t  uptimeToTowOffsetMs = 0;      //!< this session's median uptime->ToW offset
        bool     haveOffset          = false;  //!< a synced SYS_PARAMS gave this session an offset
    };

    /**
     * @brief SN-8704: a run of records whose DID's stamped clock STOPPED while the rest of the
     *        log kept advancing.
     *
     * A device that loses its time source can keep running and keep emitting — reporting the
     * last time-of-week it knew, forever. Because a record's index timestamp IS that field, every
     * such record lands on one instant, and the whole tail of the device's data collapses onto a
     * single point on the timeline. Observed on a customer capture: a GPX lost GNSS and 2,107
     * `DID_GPX_STATUS` records across four segments all carry ToW 342,615,500 while the IMX
     * clock advanced 16 real minutes beside them.
     *
     * The records are not wrong about anything except *when* — they are retained, ordered, and
     * their payloads are intact. So rather than plotting them on top of each other, the resolver
     * distrusts the stalled field and re-times them against the collective timeline of the
     * witnesses that were still working. See `interpolateArrivalTime`.
     *
     * @note The detectable signature is "stamped time static while the log advances". NOT "the
     *       payload's time leaps" — in the observed case the device's own `upTime` advances a
     *       tidy 0.501 s per record throughout, so a leap test never fires.
     */
    struct StalledRun {
        uint32_t    did           = 0;      //!< DID whose stamped clock stopped.
        uint64_t    stalledTsMs   = 0;      //!< The frozen value every record in the run carries.
        uint64_t    arrivalStart  = 0;      //!< First affected record's arrival index (inclusive).
        uint64_t    arrivalEnd    = 0;      //!< Last affected record's arrival index (inclusive).
        std::size_t recordCount   = 0;      //!< Records in the run.

        /**
         * @brief Arrival indices of the STALLED DID's own records in this run, ascending.
         *
         * Copilot review, #1316: the repair used to be selected by arrival INTERVAL alone
         * (`arrivalStart <= i <= arrivalEnd`), so every OTHER DID's record arriving inside a
         * stalled DID's window was re-timed as though its own clock were frozen — discarding a
         * perfectly good timestamp in favour of an interpolation. A stall belongs to one DID;
         * membership here is what says so, and `resolve()` has no DID parameter to test instead.
         *
         * Truncated to `kMaxRetimedPerRun`; when that happens the ruler is invalidated (see
         * `ruler`), because partial evidence must not be presented as a complete one.
         */
        std::vector<uint64_t> arrivals;

        //! Which evidence supplied the replacement times in `retimed`.
        enum class Ruler : uint8_t {
            None,        //!< Nothing usable; the resolver brackets against the collective timeline.
            Cadence,     //!< The DID's own pre-stall inter-record interval, applied uniformly.
            OwnClock,    //!< The DID's own companion uptime, per record.
            LogTimeOffset,  //!< The `.idx` per-record RECEIPT delta. Exact, and works for any DID.
        };

        /**
         * @brief Per-record replacement times, `(arrivalIndex, towMs)`, ascending. Empty when no
         *        ruler could be established or corroborated — the resolver then brackets against
         *        the collective timeline.
         *
         * Why a ruler at all, rather than always bracketing: bracketing against the GLOBAL
         * arrival order assumes the log's record RATE is locally steady, and at a stall it
         * usually is not. In the motivating capture the GNSS position/velocity DIDs stop emitting
         * at the same instant the clock freezes, so records-per-second drops exactly where the
         * run begins. Measured, arrival-order bracketing produced 3 ms..611 ms spacing (mean
         * 454.7 against an expected 500) and left the run ~1.7 s short.
         *
         * Two rulers, in preference order:
         *
         * - `OwnClock` — a companion `upTime` in the same payload, giving a per-record answer.
         *   Only `sys_params_t` and `gpx_status_t` have one: **2 of the 27 ToW-bearing record
         *   types**. It is emphatically NOT the case that the DIDs which can stall are the ones
         *   with a companion uptime — ANY DID can stall, and 25 of 27 have no such field.
         * - `Cadence` — the DID's own median inter-record interval measured while its clock was
         *   still advancing, applied uniformly across the run. Needs no payload field, so it
         *   covers the other 25. Assumes the DID's output rate is steady, which is a far weaker
         *   assumption than the global record rate being steady.
         *
         * Either way the run's FIRST record keeps its genuine timestamp, so the repair is
         * continuous at the seam by construction (measured: 0 ms).
         *
         * The eventual exact answer is `log_time_offset_ms` — a per-record receipt delta the format
         * already defines for EVERY record regardless of DID. It is zero in every log to hand
         * because only the live capture writer stamps it.
         */
        std::vector<std::pair<uint64_t, uint64_t>> retimed;

        Ruler ruler = Ruler::None;          //!< Which evidence `retimed` came from.

        //! `rulerDelta / collectiveTimelineDelta` over the run. 1.0 is perfect agreement.
        double rulerRatio = 0.0;

                //! True when `retimed` is populated, i.e. the device's own clock agreed with the
        //! collective timeline closely enough to be trusted as the ruler.
        bool rulerCorroborated = false;
    };

    /**
     * @brief Diagnostic counters from `computeStats(deviceLog)`.
     *
     * Counts of the per-record confidence outcomes when resolving
     * every record in the source log. The five fields sum to the
     * log's total record count.
     */
    struct Stats {
        std::size_t exact          = 0;  //!< Confidence::Exact (PayloadToW).
        std::size_t interpolated   = 0;  //!< Between sync points.
        std::size_t extrapFwd      = 0;  //!< Past last sync.
        std::size_t extrapBack     = 0;  //!< Before first sync.
        std::size_t unknown        = 0;  //!< No sync points at all.
    };

    // -----------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------

    /**
     * @brief Build a resolver for one device log.
     *
     * Walks `log`'s records across all segments, identifying HAS_TOW-
     * flagged records as sync points. Adjacent duplicates (same
     * `hostTimeMs`) collapse to one. Discontinuities between sync
     * segments are computed using the default threshold.
     *
     * @param log  Source device log; the resolver does not retain a
     *             reference to it after construction (sync points are
     *             snapshotted into the resolver's owned vector).
     * @return     A fully-built resolver, or `ISError` on internal
     *             failure (none expected at v1 — kept as the API
     *             shape so future fallible-parse scenarios have a
     *             place to surface errors).
     */
    static ISExpected<ISTimeResolver> build(const ISDeviceLog& log);

    /**
     * @brief Same as `build` but with a custom discontinuity threshold.
     *
     * @param log        Source device log.
     * @param threshold  Slope-ratio change beyond which a discontinuity
     *                   is reported. Default
     *                   `kDefaultDiscontinuityThreshold` (1000 ppm).
     */
    static ISExpected<ISTimeResolver> build(const ISDeviceLog& log,
                                            double threshold);

    ISTimeResolver()                                 = default;
    ISTimeResolver(const ISTimeResolver&)            = default;
    ISTimeResolver(ISTimeResolver&&) noexcept        = default;
    ISTimeResolver& operator=(const ISTimeResolver&) = default;
    ISTimeResolver& operator=(ISTimeResolver&&)
                                              noexcept = default;

    // -----------------------------------------------------------------
    // Detection
    // -----------------------------------------------------------------

    /**
     * @brief Walk a device log and return its sync points without
     *        constructing a full resolver.
     *
     * Useful for diagnostic tooling. `build` calls this internally.
     *
     * @param log  Source device log.
     * @return     Sync points sorted by `hostTimeMs` ascending,
     *             adjacent-duplicate-filtered.
     */
    static std::vector<ISSyncPoint> detectSyncPoints(const ISDeviceLog& log);

    // -----------------------------------------------------------------
    // Resolution
    // -----------------------------------------------------------------

    /**
     * @brief Resolve a record's stored timestamp to a tagged `TimeStamp`.
     *
     * @param hostTimeMs    The record's `.idx` `timestamp` field. For HAS_TOW records this is
     *                      already the ToW; for non-HAS_TOW records it is an uptime-domain value.
     * @param deviceId      Source device id; baked into the returned `TimeStamp`.
     * @param arrivalIndex  The record's position in the device's global record-arrival order.
     *                      Use `ISRecordView::arrivalIndex()`, which `ISDeviceLog` populates.
     *                      Pass `ISRecordView::kNoArrivalIndex` ONLY when the query is not a
     *                      record at all (e.g. resolving a span endpoint): the arrival-keyed
     *                      behaviours are then skipped, which is correct for a non-record but
     *                      WRONG for a record. Never pass it to avoid plumbing an index.
     *
     * @return  Tagged time: `PayloadToW / Exact` on an exact sync-point match;
     *          `ResolvedViaSync` with `Interpolated` / `ExtrapolatedForward` /
     *          `ExtrapolatedBackward` around the sync-anchored region; `SessionOnly / Unknown`
     *          when there is nothing to anchor against.
     *
     * @note The arrival index is REQUIRED, not optional. There used to be a
     *       `resolve(hostTimeMs, deviceId)` overload, and every application call site used it —
     *       which silently disabled two arrival-keyed behaviours those callers needed:
     *       SN-8339's per-boot-session offset selection, and SN-8704's re-timing of records
     *       whose DID's clock stalled. The latter is not a refinement: 2,107 records sharing
     *       one frozen timestamp cannot be told apart by `hostTimeMs`, so without the arrival
     *       index the resolver returns the same known-bad instant for all of them. The overload
     *       was deleted rather than deprecated so the compiler finds every caller.
     *
     * @param hostTimeMs   Record's `.idx` timestamp field.
     * @param deviceId     Source device id.
     * @param arrivalIndex Record's global arrival index (see `ISRecordView`).
     */
    /**
     * @warning `arrivalIndex` is **0-BASED** — the first record of the first segment is index 0,
     *          matching the resolver's own build scan (`thisArrival = arrivalIndex++`) and
     *          `ISLogReader::detectGaps`. An off-by-one is silent for the session-selection path
     *          (session windows are thousands of records wide) but NOT for the stalled-run path
     *          added in SN-8704: a caller counting from 1 mis-resolves the record at each run
     *          boundary, which looks like a lone ~16-minute backward jump. Count with a
     *          post-increment over `allRecords()` in composition order.
     */
    /**
     * @brief One record's probable absolute time, from every mechanism available (SN-8784).
     *
     * Sits ALONGSIDE @ref resolve rather than replacing it, deliberately and temporarily: the two
     * are expected to DISAGREE where `resolve` is wrong, and the migration is gated on that
     * difference set being enumerated rather than on the two agreeing.
     *
     * What it does that `resolve` cannot: `resolve` returns a value in whichever frame the sync
     * points happened to be in, and the anchor cascade's `anchoredStartMs` is in the GPS
     * time-of-week frame on every ToW-tier segment — measured 2026-10-02 across 75 corpus logs,
     * 661 of 709 segments land in the ToW frame with no week ever applied. This function composes
     * the two: the cascade's offset maps a raw value onto the segment's ToW frame, and then a week
     * — from the payload, carried from an earlier record, or derived from the filename anchor —
     * takes it to Unix. That second step is the one nothing else performs.
     *
     * Addressed by `(segment, record)` rather than by an `ISRecordView`, because a view obtained
     * from `ISLogReader::recordAt` carries `arrivalIndex() == UINT64_MAX` — arrival indices are
     * only populated when iterating `ISDeviceLog::allRecords()` (measured 2026-10-02 on a
     * 3-segment log: every per-segment view reports -1). The pair is also what
     * @ref resolveTimeToSegmentOffset returns, which makes the round trip symmetrical.
     *
     * @param log           The owning device log; the resolver holds no reference to one.
     * @param segmentIndex  Segment, `0 <= i < log.segmentCount()`.
     * @param recordIndex   Record within that segment.
     * @return              The answer and its provenance. `valid == false` for a record with no
     *                      time, or an out-of-range address.
     */
    AbsTimeResult resolve(const ISDeviceLog& log,
                          std::size_t segmentIndex,
                          std::size_t recordIndex) const;

    /**
     * @brief The inverse: where in the log does a given absolute instant sit? (SN-8784)
     *
     * Deliberately implemented over @ref resolve rather than over the `.idx` timestamps, so
     * the two directions cannot drift apart — that round trip being provable is the whole point.
     *
     * @param log         The owning device log.
     * @param absoluteMs  Target instant, Unix epoch ms.
     * @return            Segment index, path, handle, byte offset and arrival index of the record
     *                    at or before @p absoluteMs, with how exactly it matched.
     */
    SegmentOffset resolveTimeToSegmentOffset(const ISDeviceLog& log, uint64_t absoluteMs) const;

    /**
     * @brief The first record at or after @p fromSegment that @ref resolve can place.
     *
     * The origin for relative time. Forward-scanning rather than a minimum over every record: a
     * minimum lets one mis-framed record anywhere in the log redefine the origin, which is a
     * failure already observed (`anchoredSpanStart` is a minimum, and on 6 of 13 corpus
     * device-logs it is dragged into 1970 by a single record).
     *
     * @param log          The device log.
     * @param fromSegment  Segment to start at. Zero scans the whole log; any other value is
     *                     scoped to that one segment.
     * @return             The first placeable record's result, or an invalid result.
     */
    AbsTimeResult firstResolvableIn(const ISDeviceLog& log, std::size_t fromSegment) const;

    /**
     * @brief Diagnostic: how many records the memoised absolute-time index currently holds.
     *
     * Zero means cold — either nothing has asked for a reverse lookup yet, or this resolver is a
     * copy. Exposed so a test can prove the memoisation and the cold-on-copy rule actually hold;
     * the answers alone cannot distinguish a carried cache from a rebuilt one.
     *
     * @return  Entry count, or zero when the index is not built.
     */
    std::size_t absIndexSize() const noexcept { return absIndex_.entries.size(); }

private:
    /**
     * @brief @ref resolve without the relative-time fields.
     *
     * Exists so the public function can compute relative time from an origin that is itself found
     * by resolving records, without recursing into itself.
     *
     * @param log           The device log.
     * @param segmentIndex  Segment.
     * @param recordIndex   Record within that segment.
     * @return              Absolute time and provenance; `relativeTo*Ms` are left zero.
     */
    AbsTimeResult resolveCore(const ISDeviceLog& log,
                              std::size_t segmentIndex,
                              std::size_t recordIndex) const;

    /**
     * @brief Fills the memoised per-log caches: relative-time origins and stuck-field DIDs.
     *
     * One pass over every record, memoised against the log pointer. Costs what a sidecar scan
     * costs and is paid once; without it a single record lookup would scan the whole log.
     *
     * @param log  The device log.
     */
    void ensureOrigins(const ISDeviceLog& log) const;

    /**
     * @brief Is this DID's time field stuck — the same value on every record it appears in?
     *
     * @param did  The DID.
     * @return     `true` when the field never varies and so carries no time.
     */
    bool didFieldIsFrozen(uint32_t did) const;

    /** @brief Nearest live record BEFORE this one in arrival order. @return Its result, or invalid. */
    AbsTimeResult lastLiveBefore(const ISDeviceLog& log, std::size_t segmentIndex,
                                 std::size_t recordIndex) const;

    /** @brief Nearest live record AFTER this one in arrival order. @return Its result, or invalid. */
    AbsTimeResult firstLiveAfter(const ISDeviceLog& log, std::size_t segmentIndex,
                                 std::size_t recordIndex) const;

    /**
     * @brief Builds the sorted absolute-time index for @p log, unless it is already built.
     *
     * One resolving pass over every record, then a sort. Paid once per log; without it every
     * @ref resolveTimeToSegmentOffset call scans the whole log, which measured 953 s for the
     * 6,793-cycle corpus round-trip test (2026-10-02).
     *
     * @param log  The device log.
     */
    void ensureAbsIndex(const ISDeviceLog& log) const;

    //! One placeable record, as @ref AbsIndexCache holds it.
    struct AbsIndexEntry {
        uint64_t absoluteMs;     //!< Where @ref resolveCore placed this record.
        //! 32-bit deliberately: the index is one entry per record of the log, so halving the
        //! entry matters, and neither count can approach 2^32 (a segment that large would not
        //! fit on a filesystem, and `segmentCount` is a handful).
        uint32_t segmentIndex;
        uint32_t recordIndex;
    };

    /**
     * @brief The memoised sorted absolute-time index. Deliberately does NOT survive a copy.
     *
     * Validity is keyed on the log's ADDRESS, which is only sound while that log is alive — and a
     * resolver outlives the call that built it (`ISTimeResolver` holds no reference to a log and is
     * moved into a memoised optional by the Logalyzer adapter). A copy therefore starts COLD rather
     * than inheriting a key it cannot re-validate, which also keeps the copy from carrying a
     * multi-megabyte vector it may never read.
     *
     * The user-declared copy operations suppress the implicit move ones, so a MOVED resolver starts
     * cold as well — intended, and cheap, since the reset allocates nothing.
     */
    struct AbsIndexCache {
        const ISDeviceLog*         log = nullptr;
        /**
         * @brief Sorted by `(absoluteMs, segmentIndex, recordIndex)`.
         *
         * The tie-break on the indices is what makes "the first record of a stalled run" a single
         * stable answer. Being sorted by instant is also what lets the span ends be read off
         * `front()` and `back()` — the two clamps are about the EARLIEST and LATEST records, and
         * reading them from arrival order instead was wrong on half the corpus.
         */
        std::vector<AbsIndexEntry> entries;

        AbsIndexCache() = default;
        AbsIndexCache(const AbsIndexCache&) noexcept {}
        AbsIndexCache& operator=(const AbsIndexCache&) noexcept {
            log = nullptr;
            entries.clear();
            entries.shrink_to_fit();
            return *this;
        }
    };
    mutable AbsIndexCache absIndex_;

    /**
     * @brief The smallest non-zero raw sidecar value in a segment, or 0 if it has none.
     *
     * The last-resort origin for relative time on a segment whose cascade reports no uptime extrema
     * at all — which is every segment built entirely from declared-time-of-week DIDs. Populated by
     * `ensureOrigins` during the pass it already makes over every record, so it costs nothing extra.
     *
     * @param segmentIndex  Segment.
     * @return              The floor, or 0 when unknown.
     */
    uint64_t segmentRawFloorMs(std::size_t segmentIndex) const;

    //! Per-segment smallest non-zero raw sidecar value. Populated by `ensureOrigins`.
    mutable std::vector<uint64_t> segmentRawFloorMs_;

    //! DIDs whose sidecar timestamp never varies across the log. Populated by `ensureOrigins`.
    mutable std::set<uint32_t> frozenDids_;

    //! Memoised relative-time origins, keyed on the log they were computed for. Mutable because
    //! resolving is logically const; a different log invalidates them wholesale.
    mutable const ISDeviceLog*  originsLog_  = nullptr;
    mutable uint64_t            logOriginMs_ = 0;
    mutable std::vector<uint64_t> segmentOriginMs_;

public:

    /**
     * @brief DEPRECATED. The pre-SN-8784 resolver, kept only so its behaviour stays inspectable.
     *
     * Renamed from `resolve` on 2026-10-03 (Kyle's instruction) so that `resolve` could become the
     * NEW mechanism and every existing call site move onto it. Nothing in the SDK or in Logalyzer
     * calls this any more.
     *
     * Why it had to go: it is handed a RAW sidecar value and must guess which domain that value is
     * in. On a log that never achieved a GNSS fix it guesses wrong, and the error is 46 years -
     * measured across four logs on 2026-10-03, this function placed every record on 1980-01-13
     * (GPS week 1) while the log's own filename said 2026. It also cannot see the record's DID, its
     * segment, or the segment's anchor, so it has nothing to cross-check the guess against.
     *
     * @param hostTimeMs    Raw `.idx` timestamp, domain unknown to this function.
     * @param deviceId      Device the value came from.
     * @param arrivalIndex  Log-wide arrival index; disambiguates a stalled run.
     * @return              Best-effort stamp.
     *
     * @deprecated Use @ref resolve, which takes the record's ADDRESS instead of a bare value.
     */
    [[deprecated("SN-8784: use resolve(log, segmentIndex, recordIndex) - this guesses the raw "
                 "value's domain and is 46 years wrong on a log with no GNSS fix")]]
    TimeStamp resolveLegacy(uint64_t hostTimeMs, uint64_t deviceId,
                            uint64_t arrivalIndex) const;

    /**
     * @return  Per-power-on sessions detected during build (SN-8339), in
     *          arrival order. Size 1 for a single-boot log.
     */
    const std::vector<Session>& sessions() const noexcept { return sessions_; }

    /**
     * @return  Sync points used to build this resolver, sorted by
     *          `hostTimeMs`. Lifetime tied to the resolver.
     */
    const std::vector<ISSyncPoint>& syncPoints() const noexcept {
        return syncPoints_;
    }

    /**
     * @return  Clock-correction events detected during build, in
     *          chronological order. Empty for clean logs.
     */
    /**
     * @return  Stalled-clock runs found during build (SN-8704), in arrival order. Empty for a
     *          healthy log. Surfaced so the application can CALL THIS OUT rather than quietly
     *          presenting reconstructed times as measured ones.
     */
    const std::vector<StalledRun>& stalledRuns() const noexcept { return stalledRuns_; }

    /**
     * @brief Interpolate an absolute time for a record from its position in the arrival order.
     *
     * Used when a record's own stamped time cannot be trusted: its neighbours in the arrival
     * stream can be, so the record is bracketed between them. Lifted from Logalyzer's
     * `RawSeriesBuilder` (SN-8131), where it placed *timeless* records — a record whose clock
     * stalled is the same problem, a record whose own claim is worthless, so it gets the same
     * treatment rather than a second mechanism. Kyle 2026-09-20 approved the move SDK-side.
     *
     * @param anchors       `(arrivalIndex, absoluteMs)` pairs, ascending by arrival index and
     *                      non-empty. Clamps to the first/last anchor outside their range.
     * @param arrivalIndex  Record to place.
     * @return              Interpolated absolute ms.
     *
     * @note Interpolating against the GLOBAL arrival order assumes the log's overall record rate
     *       is locally steady, which is what makes it safe here: the stalled DID is by definition
     *       the misbehaving one, while the anchors come from sources that were still healthy.
     */
    /**
     * @brief Evidence for re-timing one stalled run — PURE, no I/O, no log.
     *
     * Extracted so the ruler selection is testable without synthesising a log with a stalled
     * clock, for the same reason `findGaps` and `planSessionAdoptions` are pure: the interesting
     * cases (cadence covering the 25 record types with no companion uptime, own-clock preference,
     * degenerate inputs) are otherwise unreachable from a test.
     */
    struct StallEvidence {
        //! Arrival indices of every record in the run, ascending. The run's FIRST record is the
        //! one whose timestamp legitimately advanced to the value that then froze, so it keeps
        //! its own time and the repair is continuous at the seam.
        std::vector<uint64_t> runArrivals;

        //! The frozen timestamp — also the run's first record's genuine time.
        uint64_t stalledTsMs = 0;

        //! `(arrivalIndex, ownClockMs)` where the DID carries a companion uptime. Empty for the
        //! 25 of 27 ToW-bearing record types that do not.
        std::vector<std::pair<uint64_t, uint64_t>> ownSamples;

        //! The companion uptime at the run's first (still-healthy) record. 0 when absent.
        uint64_t lastHealthyOwnMs = 0;

        //! Inter-record intervals seen while this DID's clock was still advancing. The median is
        //! the cadence ruler.
        std::vector<uint64_t> advanceDeltas;

        /**
         * @brief `(arrivalIndex, log_time_offset_ms)` for the run, when the segment's `.idx`
         *        declares `IS_LOG_IDX_HDR_FLAG_HAS_LOG_TIME_OFFSET`.
         *
         * This is the WHEN — a log-start time-offset the live capture writer stamps for EVERY record
         * independent of any payload (SN-8383). It is the exact ruler and the only one that works
         * for any DID, so it is preferred over both the companion-uptime and cadence rulers.
         *
         * Empty for every log to hand: only `DeviceLog.cpp` stamps it, and a reader-rebuilt index
         * cannot recover it (receipt time exists nowhere in the `.raw`). A rebuilt index leaves
         * `HAS_LOCAL_DELTA` clear, which is what makes this check meaningful rather than a trap.
         */
        std::vector<std::pair<uint64_t, uint64_t>> logTimeOffsets;
    };

    /**
     * @brief Choose a ruler and produce per-record replacement times. PURE.
     *
     * Preference: `OwnClock` (per-record, most faithful) then `Cadence` (the DID's own output
     * rate, applied uniformly) then `None` (caller brackets against the collective timeline).
     *
     * @param ev       Evidence gathered during the scan.
     * @param outKind  Receives which ruler was chosen.
     * @return         `(arrivalIndex, towMs)` pairs, ascending; empty when no ruler applies.
     */
    static std::vector<std::pair<uint64_t, uint64_t>>
        planStallRetiming(const StallEvidence& ev, StalledRun::Ruler& outKind);

    static uint64_t interpolateArrivalTime(
        const std::vector<std::pair<uint64_t, uint64_t>>& anchors, uint64_t arrivalIndex);

    const std::vector<Discontinuity>& discontinuities() const noexcept {
        return discontinuities_;
    }

    /**
     * @return  Per-record confidence-tier histogram for `log`. Useful
     *          for diagnostics / "how trustworthy is this log's
     *          timing?" summaries.
     */
    Stats computeStats(const ISDeviceLog& log) const;

private:
    explicit ISTimeResolver(std::vector<ISSyncPoint> syncs,
                            std::vector<Discontinuity> discs,
                            uint32_t anchorWeek,
                            uint64_t anchorTowStart,
                            uint64_t anchorTowEnd,
                            int64_t  uptimeToTowOffsetMs,
                            bool     haveUptimeOffset,
                            uint64_t fileAnchorMs,
                            bool     haveFileAnchor,
                            std::vector<Session> sessions = {}) noexcept
        : syncPoints_(std::move(syncs)),
          discontinuities_(std::move(discs)),
          anchorWeek_(anchorWeek),
          anchorTowStart_(anchorTowStart),
          anchorTowEnd_(anchorTowEnd),
          uptimeToTowOffsetMs_(uptimeToTowOffsetMs),
          haveUptimeOffset_(haveUptimeOffset),
          fileAnchorMs_(fileAnchorMs),
          haveFileAnchor_(haveFileAnchor),
          sessions_(std::move(sessions)) {}

    //! Core detection: scans all segments for sync points AND (SN-8323 uptime
    //! unification) authoritative uptime->ToW offset samples from DID_SYS_PARAMS.
    //! `detectSyncPoints` and `build` both delegate here.
    //! @param stalledOut   SN-8704: receives the stalled-clock runs found during the scan.
    //! @param timelineOut   SN-8704: receives `(arrivalIndex, payloadToWMs)` samples from
    //!                      ToW sources that were still advancing. Both optional.
    static std::vector<ISSyncPoint> detectSyncPointsImpl(
        const ISDeviceLog& log, std::vector<int64_t>& upOffsetsOut,
        std::vector<Session>& sessionsOut,
        std::vector<StalledRun>* stalledOut = nullptr,
        std::vector<std::pair<uint64_t, uint64_t>>* timelineOut = nullptr);

    //! SN-8339: shared resolve body, parameterized on the uptime->ToW offset so
    //! both the global (no-key) path and the per-session (arrival-keyed) path
    //! reuse identical logic. `resolve(h,d)` passes the global offset; the
    //! arrival-keyed overload passes the selected session's offset.
    TimeStamp resolveImpl(uint64_t hostTimeMs, uint64_t deviceId,
                          int64_t uptimeOffsetMs, bool haveOffset) const;

    std::vector<ISSyncPoint>    syncPoints_;
    std::vector<Discontinuity>  discontinuities_;
    //! SN-8323: epoch-anchor GPS week, derived in build() from the log's durable
    //! fix period (the non-zero week with the widest ToW coverage — see
    //! chooseAnchorWeek). 0 => no valid week seen (pre-fix log); resolve() then
    //! falls back to ToW-only, pre-D0066 behavior.
    uint32_t                    anchorWeek_ = 0;
    //! SN-8323: earliest ToW (ms into week) of the durable fix period. A
    //! ToW-domain input well before this began has no stable GPS time (a
    //! pre-fix / startup record) and resolve() tags it SessionOnly/Unknown so
    //! consumers exclude it from the timeline + extent.
    uint64_t                    anchorTowStart_ = 0;
    //! SN-8323 (uptime unification): latest ToW (ms into week) of the durable
    //! fix period. With anchorTowStart_ it bounds the plausible ToW window used
    //! to classify a resolve() input as ToW-domain vs uptime-domain.
    uint64_t                    anchorTowEnd_ = 0;
    //! SN-8323 (uptime unification): authoritative uptime->GPS-ToW offset (ms),
    //! derived from DID_SYS_PARAMS (timeOfWeekMs - upTime) during build(). Kyle
    //! 2026-07-23: SYS_PARAMS.upTime is the definitive relative uptime; session-
    //! only records (magnetometer, imu) and pre-sync "real clock" records (whose
    //! week/ToW default to uptime until GPS sync) are bridged through this single
    //! offset instead of the fragile per-sync actualHostTimeMs heuristic.
    int64_t                     uptimeToTowOffsetMs_ = 0;
    //! True when a synced DID_SYS_PARAMS gave a usable uptime->ToW offset.
    bool                        haveUptimeOffset_ = false;
    //! Kyle 2026-09-07 (Option B): wall-clock instant (Unix-epoch ms) corresponding
    //! to host-uptime == 0, recovered from the log's own file (filename/directory
    //! name, or last-write time) when the log has zero sync points of any kind.
    //! Only meaningful when `haveFileAnchor_` is true; see `deriveFileAnchorMs`.
    uint64_t                    fileAnchorMs_ = 0;
    //! True when `build()` found a usable file-timestamp anchor (Option B). Only
    //! ever set when `syncPoints_` is empty -- a log with any real sync point
    //! never needs this fallback.
    bool                        haveFileAnchor_ = false;
    //! SN-8339: per-power-on sessions (reboot = SYS_PARAMS.upTime drop in
    //! arrival order). Size 1 for a single-boot log; the arrival-keyed resolve()
    //! overload uses per-session offsets when size > 1.
    std::vector<Session>        sessions_;

    //! SN-8704: runs where one DID's stamped clock stopped while the log advanced.
    std::vector<StalledRun>     stalledRuns_;

    //! SN-8704: `(arrivalIndex, payloadToWMs)` samples from ToW sources that were still
    //! ADVANCING — the collective timeline a stalled record is bracketed against. Only
    //! strictly-increasing ToW values are admitted, so a stalled source excludes itself by
    //! construction: it can never advance past the last admitted sample.
    std::vector<std::pair<uint64_t, uint64_t>> towTimeline_;
};

} // namespace inertial_sense
