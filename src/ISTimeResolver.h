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
 *     uptime delta (ms since logger session start).
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
#include <vector>

namespace inertial_sense {

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
     * @brief Resolve a single record's stored timestamp to a tagged
     *        `TimeStamp`.
     *
     * @param hostTimeMs  The record's `.idx` `timestamp` field. For
     *                    HAS_TOW records this is already the ToW; for
     *                    non-HAS_TOW records it's host uptime delta.
     * @param deviceId    Source device id; baked into the returned
     *                    `TimeStamp`.
     * @return            Tagged time:
     *                    - `PayloadToW / Exact` if `hostTimeMs` matches
     *                      a sync point exactly.
     *                    - `ResolvedViaSync / Interpolated` between
     *                      sync points.
     *                    - `ResolvedViaSync / ExtrapolatedForward` past
     *                      the last sync point.
     *                    - `ResolvedViaSync / ExtrapolatedBackward`
     *                      before the first sync point.
     *                    - `SessionOnly / Unknown` when the resolver
     *                      has no sync points to anchor against.
     */
    TimeStamp resolve(uint64_t hostTimeMs, uint64_t deviceId) const;

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
                            bool     haveUptimeOffset) noexcept
        : syncPoints_(std::move(syncs)),
          discontinuities_(std::move(discs)),
          anchorWeek_(anchorWeek),
          anchorTowStart_(anchorTowStart),
          anchorTowEnd_(anchorTowEnd),
          uptimeToTowOffsetMs_(uptimeToTowOffsetMs),
          haveUptimeOffset_(haveUptimeOffset) {}

    //! Core detection: scans all segments for sync points AND (SN-8323 uptime
    //! unification) authoritative uptime->ToW offset samples from DID_SYS_PARAMS.
    //! `detectSyncPoints` and `build` both delegate here.
    static std::vector<ISSyncPoint> detectSyncPointsImpl(
        const ISDeviceLog& log, std::vector<int64_t>& upOffsetsOut);

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
};

} // namespace inertial_sense
