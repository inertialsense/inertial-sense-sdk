/**
 * @file ISTimeResolver.cpp
 * @brief Piecewise-linear resolver implementation.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISTimeResolver.h"

// com_manager.h FIRST — see ISLogReader.cpp's note on the extern-C
// wrap collision in ISFirmwareUpdater.h.
#include "com_manager.h"

#include "ISComm.h"
#include "ISDataMappings.h"
#include "core/msg_logger.h"
#include "data_sets.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <map>
#include <optional>
#include <regex>
#include <system_error>

namespace inertial_sense {

namespace {

//! DIDs whose payloads carry a usable GPS time-of-week field. Used by
//! `detectSyncPoints` as the candidate set when scanning a segment.
//! `cISDataMappings::Timestamp(...)` is consulted on each candidate to
//! extract the actual ToW (returns 0 if the field is absent / zero,
//! which the detection loop treats as "not a sync anchor").
constexpr std::array<uint32_t, 6> kToWBearingDids = {
    DID_INS_1, DID_INS_2, DID_INS_3, DID_INS_4,
    DID_GNSS1_POS, DID_GNSS2_POS,
};

//! GPS epoch (1980-01-06 00:00:00 UTC) expressed as
//! milliseconds-since-Unix-epoch. Used together with `gpsWeek` and
//! `payloadToWMs` to anchor resolver output to Unix epoch so QDateTime
//! can render wall-clock dates directly.
//!
//! @note SN-8107 / D0066.
constexpr uint64_t kGpsEpochUnixMs = 315'964'800'000ULL;
constexpr uint64_t kGpsWeekMs      = 604'800'000ULL;  //!< 7 days in ms
//! SN-8323: how far before the durable fix period a ToW-domain record may fall
//! and still be treated as a (backward-extrapolated) part of the timeline.
//! Beyond this it is a pre-fix / startup record with no stable GPS time and is
//! tagged SessionOnly/Unknown. Sized to preserve legitimate backward
//! extrapolation (GPS cold-start acquisition is well under an hour) while
//! rejecting the pathological case, which is grossly off — a ToW at the GPS
//! week start while the durable fix is days into the week. A record more than
//! an hour before the first stable fix predates any plausible continuous
//! session (prior power cycle / uninitialized time).
constexpr uint64_t kPreFixGuardMs  = 3'600'000ULL;    //!< 1 hour

/**
 * @brief Convert `(gpsWeek, towMs)` into a Unix-epoch ms timestamp.
 *
 * @param gpsWeek GPS week number (weeks since 1980-01-06).
 * @param towMs   Time-of-week in milliseconds.
 * @return        Milliseconds since Unix epoch (1970-01-01 UTC).
 *
 * @note SN-8107 / D0066.
 */
inline uint64_t gpsToUnixMs(uint32_t gpsWeek, uint64_t towMs) noexcept {
    return static_cast<uint64_t>(gpsWeek) * kGpsWeekMs + kGpsEpochUnixMs + towMs;
}

inline bool isToWBearing(uint32_t did) noexcept {
    for (auto d : kToWBearingDids) if (d == did) return true;
    return false;
}

/**
 * @brief Slope between two sync points in ToW-ms per host-ms.
 *
 * @param a Earlier sync point.
 * @param b Later sync point.
 * @return  ToW-delta divided by host-delta; returns 1.0 when the host
 *          distance is zero (degenerate / colocated points) so
 *          downstream comparisons don't divide by zero.
 */
double slopeBetween(const ISSyncPoint& a, const ISSyncPoint& b) noexcept {
    if (b.hostTimeMs == a.hostTimeMs) return 1.0;
    const double hostDelta = static_cast<double>(b.hostTimeMs) - static_cast<double>(a.hostTimeMs);
    const double towDelta  = static_cast<double>(b.payloadToWMs) - static_cast<double>(a.payloadToWMs);
    return towDelta / hostDelta;
}

/**
 * @brief Walk one segment's `.raw` bytes via `is_comm_parse_byte`,
 *        emitting a sync point for every ToW-bearing-DID packet whose
 *        payload carries a non-zero ToW.
 *
 * The walk per segment matches the writer's per-packet emission
 * pattern (`cDeviceLogRaw::SaveData`), so we recover the same anchors
 * the writer would have flagged with `HAS_TOW`.
 *
 * The walker also tracks the most recent non-sync record's payload-side
 * timestamp (when the payload happens to carry a `tsSec` value but the
 * record is NOT ToW-bearing) so each emitted sync point can carry an
 * `actualHostTimeMs` approximation (see `ISSyncPoint::actualHostTimeMs`
 * doc). For records whose payload doesn't expose a timestamp field at
 * all we leave `actualHostTimeMs == 0` for syncs that don't see a
 * non-sync predecessor.
 *
 * @param reader   Segment reader yielding `.raw` bytes.
 * @param deviceId Source device ID baked into each emitted sync point.
 * @param out      Sync points appended to (caller's vector).
 *
 * @note Host-time recovery heuristic: the writer (`cDeviceLog`) emits
 *       records in arrival order on one host thread, so the host-uptime
 *       of a sync record is millisecond-adjacent to the host-uptime of
 *       the most recently emitted non-sync record. The byte walk only
 *       sees payload bytes (no direct .idx access here), so we
 *       approximate via `cISDataMappings::Timestamp` which reads the
 *       payload's own time field. For DIDs like `DID_PIMU` the
 *       payload's `time` field IS the host uptime — that's the value
 *       we capture.
 *
 * @note SN-8107 / D0066.
 */
//! SN-8339: accumulates one power-on session during the arrival-order scan.
struct SessionAccum {
    uint64_t             arrivalStart = 0;   //!< global arrival index of the session's first record
    std::vector<int64_t> upOffsets;          //!< synced SYS_PARAMS (ToW - upTime) samples in this session
};

//! Median of a copy (sorts in place); 0 for empty.
int64_t medianOf(std::vector<int64_t> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

/**
 * @brief `.dat` equivalent of `scanSegmentForSyncs` (D-119 / SN-8626 / D0082).
 *
 * `.dat` has no wire protocol to parse (D0082): each `ISRecordView::bytes()` is already a bare
 * `p_data_hdr_t` + payload, and `reader.allRecords()` already walks them cleanly (no NMEA/RTCM3/
 * UBX noise to skip, unlike `.raw`'s byte-by-byte comm scan). Same DID_SYS_PARAMS / ToW-bearing
 * logic as the `.raw` path below — see its comments for the "why" of each step.
 */
/**
 * @brief The device's OWN steady clock from a dual-domain payload, in ms, if it has one.
 *
 * Only `DID_SYS_PARAMS` and `DID_GPX_STATUS` carry an `upTime` alongside `timeOfWeekMs` — which
 * is precisely why they are the DIDs whose stalled ToW can be repaired from first-hand evidence
 * instead of inferred from neighbours.
 */
/**
 * @brief The uptime<->ToW bridge pair out of a dual-domain payload — audit B1.
 *
 * `DID_SYS_PARAMS` (IMX) and `DID_GPX_STATUS` (GPX) each carry a GPS time-of-week AND the
 * device's own uptime in one payload, which is what makes the offset between the two domains
 * recoverable without correlating neighbours.
 *
 * Before this, the resolver read the pair from `DID_SYS_PARAMS` **only**, while the anchor
 * cascade treated `DID_GPX_STATUS` as a co-equal tier-5 bridge. Segments were therefore ORDERED
 * by one reconstruction of the offset and MEASURED by another, which D0066 forbids — one shared
 * frame. It was latent because the corpus contains no GPX-standalone log (`probe4`: 542 segments,
 * 539 with SYS_PARAMS, 177 with GPX_STATUS, **0 GPX-only**), so the sets never disagreed on any
 * real data, and the cascade's GPX branch was exercised by unit tests only.
 *
 * The failure mode it was hiding: on a GPX-only log the cascade anchors at tier 5 while the
 * resolver finds no offset source at all, so every record classifies `SessionOnly`, `detectGaps`
 * skips them and reports **zero gaps on a fully-anchored log**, and per D0066 every display path
 * treats it as unanchored.
 *
 * Validity is gated per device family — the GPX reports it on either of its two GNSS receivers.
 * An invalid bit means `timeOfWeekMs` is LOCAL system time, and differencing that against uptime
 * yields a bogus offset that corrupts the median bridge.
 */
struct BridgePair {
    uint64_t towMs = 0;      //!< Claimed GPS time-of-week, ms.
    uint64_t upMs  = 0;      //!< The device's own uptime, ms.
    double   upSec = 0.0;    //!< Same uptime in seconds, for the reboot test.
    bool     towValid = false;
};

std::optional<BridgePair> bridgePair(uint32_t did, const uint8_t* payload, uint32_t size) {
    if (payload == nullptr) return std::nullopt;
    BridgePair p;
    if (did == DID_SYS_PARAMS && size >= sizeof(sys_params_t)) {
        sys_params_t v{};
        std::memcpy(&v, payload, sizeof(v));
        p.towMs    = v.timeOfWeekMs;
        p.upSec    = v.upTime;
        p.towValid = (v.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
    } else if (did == DID_GPX_STATUS && size >= sizeof(gpx_status_t)) {
        gpx_status_t v{};
        std::memcpy(&v, payload, sizeof(v));
        p.towMs    = v.timeOfWeekMs;
        p.upSec    = v.upTime;
        p.towValid = (v.hdwStatus & (GPX_HDW_STATUS_GNSS1_TIME_OF_WEEK_VALID |
                                     GPX_HDW_STATUS_GNSS2_TIME_OF_WEEK_VALID)) != 0;
    } else {
        return std::nullopt;
    }
    p.upMs = static_cast<uint64_t>(p.upSec * 1000.0);
    return p;
}

/** @return True for a DID that carries the dual-domain bridge pair. */
inline bool isBridgeDid(uint32_t did) noexcept {
    return did == DID_SYS_PARAMS || did == DID_GPX_STATUS;
}

std::optional<uint64_t> ownClockMs(uint32_t did, const uint8_t* payload, uint32_t size) {
    if (payload == nullptr) return std::nullopt;
    if (did == DID_SYS_PARAMS && size >= sizeof(sys_params_t)) {
        sys_params_t v{}; std::memcpy(&v, payload, sizeof(v));
        if (v.upTime > 0.0) return static_cast<uint64_t>(v.upTime * 1000.0);
    } else if (did == DID_GPX_STATUS && size >= sizeof(gpx_status_t)) {
        gpx_status_t v{}; std::memcpy(&v, payload, sizeof(v));
        if (v.upTime > 0.0) return static_cast<uint64_t>(v.upTime * 1000.0);
    }
    return std::nullopt;
}

/**
 * @brief SN-8704: watches the record stream for a DID whose stamped clock stops advancing, and
 *        simultaneously collects the timeline of sources that are still advancing.
 *
 * Fed by both the `.raw` and `.dat` scans so the two cannot drift apart. Holds no file and does
 * no I/O; it is a sink, like `AnchorCollector`.
 */
class StallWatcher {
public:
    //! Consecutive identical-timestamp records from one DID before the run is reported. A few
    //! repeats are normal for a DID emitted faster than its time field's resolution; a long run
    //! means the clock stopped while the device kept talking.
    static constexpr std::size_t kStallThreshold = 32;

    //! Keep every Nth advancing sample. The timeline only has to be dense enough to bracket a
    //! record; at typical rates this is a sample every few hundred ms.
    static constexpr uint64_t kTimelineStride = 16;

    //! Cap on retained own-clock samples per run. 2,107 was the motivating case; this bounds a
    //! pathological log without touching any realistic one. Exceeding it falls back to
    //! arrival-order bracketing rather than growing without limit.
    static constexpr std::size_t kMaxRetimedPerRun = 500'000;

    //! Cadence samples retained per DID. A median needs far fewer than this.
    static constexpr std::size_t kMaxCadenceSamples = 256;

    void observe(uint32_t did, uint64_t arrivalIndex, uint64_t recordTsMs,
                 const uint8_t* payload = nullptr, uint32_t payloadSize = 0,
                 uint32_t logTimeOffsetMs = 0) {
        if (recordTsMs == 0) return;
        const auto domain = cISDataMappings::TimestampDomain(did);

        // ---- Collective timeline: admit a ToW sample only when it ADVANCES. A stalled source
        // repeats one value, so it can never advance past the last admitted sample and excludes
        // itself by construction -- no retroactive fix-up needed once a run is recognized.
        if (domain == cISDataMappings::eTimestampDomain::TIMESTAMP_DOMAIN_GPS_TOW &&
            recordTsMs > lastTimelineTow_) {
            if (timelineCounter_++ % kTimelineStride == 0 || timeline_.empty()) {
                timeline_.emplace_back(arrivalIndex, recordTsMs);
            }
            lastTimelineTow_ = recordTsMs;
        }

        // ---- Per-DID stall tracking.
        //
        // Copilot review, #1316: a DID that declares NO timestamp domain must not enter stall
        // tracking at all. The `recordTsMs == 0` guard above is not enough, because the live
        // writer deliberately parks the record's `log_time_offset_ms` in the `timestamp` field
        // for a timeless DID (DeviceLog.cpp) — a NON-zero value that repeats freely. Measured on
        // a real fixture: all 41 records shared `offsetMs = 5`. That is indistinguishable from a
        // frozen clock here, so a timeless DID could form a false `kStallThreshold`-long run and
        // drag unrelated data into a repair. Its `timestamp` is not a clock and cannot stall.
        if (domain == cISDataMappings::eTimestampDomain::TIMESTAMP_DOMAIN_NONE) return;

        const auto own = ownClockMs(did, payload, payloadSize);
        auto& st = perDid_[did];
        if (st.count == 0 || recordTsMs != st.lastTsMs) {
            // The clock advanced: this interval is evidence of the DID's own cadence, which is
            // what lets a stalled DID with no companion uptime still be re-timed from first-hand
            // evidence rather than from the global record rate.
            if (st.count != 0 && recordTsMs > st.lastTsMs &&
                st.advanceDeltas.size() < kMaxCadenceSamples) {
                st.advanceDeltas.push_back(recordTsMs - st.lastTsMs);
            }
            closeRun(did, st);
            // This record's clock is advancing, so it is the last HEALTHY pairing we have seen:
            // remember it, because the repair offset is anchored on it (giving a zero-ms seam
            // where the good data meets the rebuilt data).
            if (own) { st.lastHealthyTow = recordTsMs; st.lastHealthyOwn = *own; }
            st.lastTsMs      = recordTsMs;
            st.runStart      = arrivalIndex;
            st.runEnd        = arrivalIndex;
            st.runLen        = 1;
            st.count         = 1;
            st.ownSamples.clear();
            st.runArrivals.clear();
            st.localSamples.clear();
            if (own) st.ownSamples.emplace_back(arrivalIndex, *own);
            if (logTimeOffsetMs != 0) st.localSamples.emplace_back(arrivalIndex, logTimeOffsetMs);
            st.runArrivals.push_back(arrivalIndex);
            return;
        }
        ++st.count;
        ++st.runLen;
        st.runEnd = arrivalIndex;
        if (own && st.ownSamples.size() < kMaxRetimedPerRun) {
            st.ownSamples.emplace_back(arrivalIndex, *own);
        }
        if (st.runArrivals.size() < kMaxRetimedPerRun) st.runArrivals.push_back(arrivalIndex);
        if (logTimeOffsetMs != 0 && st.localSamples.size() < kMaxRetimedPerRun) {
            st.localSamples.emplace_back(arrivalIndex, logTimeOffsetMs);
        }
    }

    //! Close any run still open at end-of-log. A stall that runs to the last record -- the
    //! customer case -- would otherwise never be reported.
    void finish() {
        for (auto& [did, st] : perDid_) closeRun(did, st);
    }

    std::vector<ISTimeResolver::StalledRun> takeRuns() { return std::move(runs_); }
    std::vector<std::pair<uint64_t, uint64_t>> takeTimeline() { return std::move(timeline_); }

private:
    struct DidState {
        uint64_t    lastTsMs  = 0;
        uint64_t    runStart  = 0;
        uint64_t    runEnd    = 0;
        std::size_t runLen    = 0;
        std::size_t count     = 0;
        //! `(arrivalIndex, ownClockMs)` for the run in progress, when the DID has an own clock.
        std::vector<std::pair<uint64_t, uint64_t>> ownSamples;
        //! Arrival indices of the run in progress. Needed for the cadence ruler, which has no
        //! per-record evidence and so must map arrivalIndex -> ordinal within the run.
        std::vector<uint64_t> runArrivals;
        //! Inter-record intervals observed while THIS DID's clock was still advancing. The
        //! median is the cadence ruler. Bounded -- a few hundred samples is ample for a median.
        std::vector<uint64_t> advanceDeltas;
        //! The last pairing seen while this DID's clock was still ADVANCING -- the repair anchor.
        uint64_t    lastHealthyTow = 0;
        uint64_t    lastHealthyOwn = 0;
        //! `(arrivalIndex, log_time_offset_ms)` for the run in progress, when the index has them.
        std::vector<std::pair<uint64_t, uint64_t>> localSamples;
    };

    void closeRun(uint32_t did, DidState& st) {
        if (st.runLen >= kStallThreshold) {
            ISTimeResolver::StalledRun r;
            r.did          = did;
            r.stalledTsMs  = st.lastTsMs;
            r.arrivalStart = st.runStart;
            r.arrivalEnd   = st.runEnd;
            r.recordCount  = st.runLen;
            // Copilot review, #1316: carry the stalled DID's OWN arrival indices, so `resolve()`
            // can tell a member of the run from another DID's record that merely arrived inside
            // the same window.
            r.arrivals     = st.runArrivals;
            ISTimeResolver::StallEvidence ev;
            ev.runArrivals       = st.runArrivals;
            ev.stalledTsMs       = st.lastTsMs;
            ev.ownSamples        = st.ownSamples;
            ev.lastHealthyOwnMs  = st.lastHealthyOwn;
            ev.advanceDeltas     = st.advanceDeltas;
            ev.logTimeOffsets       = st.localSamples;
            r.retimed = ISTimeResolver::planStallRetiming(ev, r.ruler);

            // Copilot review, #1316: when the run outgrew the retention cap, `runArrivals` (and
            // the sample vectors built alongside it) stopped growing while `runLen`/`runEnd` kept
            // going. The resulting `retimed` covers only the first `kMaxRetimedPerRun` records,
            // and `resolve()`'s nearest-preceding fallback then hands every later arrival the
            // LAST planned value -- silently collapsing the tail of a pathological stall onto the
            // cap boundary. Partial evidence is not a ruler: drop it and let the resolver bracket
            // against the collective timeline, which is the documented no-evidence path.
            if (st.runLen > st.runArrivals.size()) {
                log_warn(IS_LOG_ISLOG,
                         "stalled DID %u: run of %zu record(s) exceeds the %zu-record evidence cap; "
                         "discarding the partial ruler and bracketing instead",
                         did, st.runLen, kMaxRetimedPerRun);
                r.retimed.clear();
                r.ruler = ISTimeResolver::StalledRun::Ruler::None;
            }
            runs_.push_back(r);
        }
        st.runLen = 0;
        st.ownSamples.clear();
        st.runArrivals.clear();
        st.localSamples.clear();
    }

    std::map<uint32_t, DidState>                perDid_;
    std::vector<ISTimeResolver::StalledRun>     runs_;
    std::vector<std::pair<uint64_t, uint64_t>>  timeline_;
    uint64_t                                    lastTimelineTow_ = 0;
    uint64_t                                    timelineCounter_ = 0;
};

void scanSegmentForSyncsDat(const ISLogReader& reader,
                            uint64_t deviceId,
                            std::vector<ISSyncPoint>& out,
                            std::vector<int64_t>& upOffsetsOut,
                            uint64_t& arrivalIndex,
                            double& prevUpTimeSec,
                            std::vector<SessionAccum>& sessAccum,
                            StallWatcher& stalls) {
    uint64_t lastNonSyncHostTimeMs = 0;
    // SN-8704: only trust log_time_offset_ms when the index DECLARES it. A reader-rebuilt index
    // leaves the flag clear and the field zero, and reading zeros as receipt times would be
    // worse than having none.
    const bool haveLogTimeOffset =
        (reader.header().flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_LOG_TIME_OFFSET) != 0;

    for (auto v : reader.allRecords()) {
        const auto bytes = v.bytes();
        if (!bytes.first || bytes.second < sizeof(p_data_hdr_t)) continue;
        p_data_hdr_t hdr{};
        std::memcpy(&hdr, bytes.first, sizeof(hdr));
        if (sizeof(p_data_hdr_t) + hdr.size > bytes.second) continue;
        const uint8_t* payloadPtr = bytes.first + sizeof(p_data_hdr_t);

        const uint64_t thisArrival = arrivalIndex++;
        // D0096: pass the offset ONLY when it was observed. A reconstructed one is derived
        // from the payload clock, so it cannot corroborate that clock -- during a stall the
        // payload timestamps are frozen and the reconstruction is flat exactly where the
        // correction is needed. Withholding it here drops this run to the next ruler
        // (own-clock, then cadence), which is the honest outcome.
        const bool observedOffset =
            haveLogTimeOffset &&
            (v.flags() & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0;
        stalls.observe(hdr.id, thisArrival, v.timestamp().value, payloadPtr, hdr.size,
                       observedOffset ? v.logTimeOffsetMs() : 0u);

        // Audit B1: either bridge DID, via the shared extractor -- the resolver used to read the
        // pair from DID_SYS_PARAMS only while the cascade accepted DID_GPX_STATUS as co-equal.
        if (isBridgeDid(hdr.id) && hdr.offset == 0) {
          if (const auto bp = bridgePair(hdr.id, payloadPtr, hdr.size)) {
            if (bp->upSec > 0.0) {
                if (prevUpTimeSec >= 0.0 && bp->upSec < prevUpTimeSec - 0.5) {
                    sessAccum.push_back(SessionAccum{ thisArrival, {} });
                }
                prevUpTimeSec = bp->upSec;
            }
            const bool towValid = bp->towValid;
            const auto& sp2 = *bp;
            if (towValid && sp2.towMs > 0 && sp2.upSec > 0.0) {
                const int64_t off = static_cast<int64_t>(sp2.towMs)
                                  - static_cast<int64_t>(sp2.upMs);
                upOffsetsOut.push_back(off);
                if (!sessAccum.empty()) sessAccum.back().upOffsets.push_back(off);
            }
            // Kyle 2026-09-07 (Option A): DID_SYS_PARAMS wasn't in kToWBearingDids, so even a
            // synced SYS_PARAMS record (towValid) never became a sync point itself -- only used
            // above to calibrate OTHER DIDs' uptime->ToW bridge. A log with SYS_PARAMS as its
            // ONLY DID ever carrying a valid GPS time (no INS/GNSS at all) therefore had zero
            // sync points regardless. Push it as a genuine anchor too, same identity convention
            // as the generic ToW-bearing path below (hostTimeMs == payloadToWMs -- a "sync"
            // record's .idx timestamp field IS the ToW, no separate host-time stored). No
            // gpsWeek: sys_params_t carries no week field, unlike ins_x_t/gnss_pos_t, so this
            // sync point can bridge host-uptime->ToW but never itself supply the epoch anchor
            // (chooseAnchorWeek skips week==0 candidates).
            if (towValid && sp2.towMs > 0) {
                ISSyncPoint sysSp{};
                sysSp.hostTimeMs       = sp2.towMs;
                sysSp.payloadToWMs     = sp2.towMs;
                sysSp.deviceId         = deviceId;
                sysSp.sourceDid        = hdr.id;   // B1: SYS_PARAMS or GPX_STATUS
                sysSp.actualHostTimeMs = lastNonSyncHostTimeMs;
                out.push_back(sysSp);
            }
          }
        }

        const double tsSec = cISDataMappings::Timestamp(&hdr, payloadPtr);

        if (!isToWBearing(hdr.id)) {
            if (tsSec > 0.0) {
                lastNonSyncHostTimeMs = static_cast<uint64_t>(tsSec * 1000.0);
            }
            continue;
        }
        if (tsSec <= 0.0) continue;

        const uint64_t towMs = static_cast<uint64_t>(tsSec * 1000.0);
        ISSyncPoint sp{};
        sp.hostTimeMs       = towMs;
        sp.payloadToWMs     = towMs;
        sp.deviceId         = deviceId;
        sp.sourceDid        = hdr.id;
        sp.actualHostTimeMs = lastNonSyncHostTimeMs;
        if (hdr.size >= sizeof(uint32_t)) {
            uint32_t weekRaw = 0;
            std::memcpy(&weekRaw, payloadPtr, sizeof(weekRaw));
            sp.gpsWeek = weekRaw;
        }
        out.push_back(sp);
    }
}

void scanSegmentForSyncs(const ISLogReader& reader,
                         uint64_t deviceId,
                         std::vector<ISSyncPoint>& out,
                         std::vector<int64_t>& upOffsetsOut,
                         uint64_t& arrivalIndex,
                         double& prevUpTimeSec,
                         std::vector<SessionAccum>& sessAccum,
                         StallWatcher& stalls) {
    // D-119 / SN-8626: .dat has no wire protocol for this function's is_comm_parse_byte scan to
    // find anything in — route to the .dat-native equivalent instead.
    if (reader.format() == ISLogReader::SegmentFormat::Dat) {
        scanSegmentForSyncsDat(reader, deviceId, out, upOffsetsOut, arrivalIndex, prevUpTimeSec,
                               sessAccum, stalls);
        return;
    }

    auto bytes = reader.rawBytes();
    if (!bytes.first || bytes.second == 0) return;

    is_comm_instance_t comm{};
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);

    // SN-8107 / D0066: most recent non-sync host-uptime observed during
    // the scan. Updated on every non-ToW-bearing record whose payload
    // exposes a timestamp; carried onto each subsequent sync point until
    // a newer non-sync time arrives.
    uint64_t lastNonSyncHostTimeMs = 0;

    // SN-8704: only trust log_time_offset_ms when the index DECLARES it (see the .dat note).
    const bool haveLogTimeOffset =
        (reader.header().flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_LOG_TIME_OFFSET) != 0;
    // The byte walk has no ISRecordView, so step the segment's index records in lockstep. Both
    // this walk and buildIndexFromScan emit ISB packets in file order, which is the same
    // invariant the arrival index itself rests on -- verified equal across 125 corpus segments
    // (ISB-only parse == all-protocol parse == recordCount()).
    std::size_t segOrdinal = 0;

    for (std::size_t i = 0; i < bytes.second; ++i) {
        protocol_type_t p = is_comm_parse_byte(&comm, bytes.first[i]);
        if (p != _PTYPE_INERTIAL_SENSE_DATA && p != _PTYPE_INERTIAL_SENSE_CMD) {
            continue;
        }
        const auto& hdr = comm.rxPkt.dataHdr;
        // SN-8339: global record-arrival index (matches ISDeviceLog's ISB-only,
        // arrival-ordered record stream — both parse the same .raw for ISB).
        const uint64_t thisArrival = arrivalIndex++;
        {
            // Same value the index build stamps for this record -- Timestamp(), never
            // TimestampOrCurrentTime() (D0069 #1).
            const double   tsSec = cISDataMappings::Timestamp(&hdr, comm.rxPkt.data.ptr);
            uint32_t localMs = 0;
            // D0096: observed offsets only -- see the note on the other scan path. A
            // reconstructed offset comes FROM the payload clock and so cannot be used to
            // correct it.
            if (haveLogTimeOffset && segOrdinal < reader.recordCount()
                && (reader.recordAt(segOrdinal).flags()
                        & idx::IS_LOG_IDX_REC_FLAG_RECONSTRUCTED_TIME_OFFSET) == 0) {
                localMs = reader.recordAt(segOrdinal).logTimeOffsetMs();
            }
            stalls.observe(hdr.id, thisArrival, static_cast<uint64_t>(tsSec * 1000.0),
                           static_cast<const uint8_t*>(comm.rxPkt.data.ptr), hdr.size, localMs);
            ++segOrdinal;
        }

        // SN-8323 (uptime unification): DID_SYS_PARAMS carries BOTH the GPS
        // time-of-week (timeOfWeekMs) and the definitive system uptime (upTime,
        // seconds since boot). When the device is synced (timeOfWeekMs > 0),
        // one such record yields the authoritative uptime->ToW offset that
        // bridges every session-uptime value (magnetometer/imu records, and
        // pre-sync "real clock" records whose week/ToW default to uptime until
        // GPS lock). This replaces the fragile per-sync actualHostTimeMs
        // heuristic. (Kyle 2026-07-23: SYS_PARAMS.upTime is the definitive
        // relative uptime; GNSS/INS are the accurate absolute clock.)
        // Audit B1: either bridge DID, through the shared extractor. This read the pair from
        // DID_SYS_PARAMS only, while the anchor cascade accepted DID_GPX_STATUS as a co-equal
        // tier-5 bridge -- so a GPX-standalone log would be ORDERED by an offset the resolver
        // could not find, leaving every record SessionOnly on a fully-anchored log. See
        // `bridgePair` for the measurements and the failure mode.
        if (isBridgeDid(hdr.id) && comm.rxPkt.data.ptr && hdr.offset == 0) {
          if (const auto bp = bridgePair(hdr.id, static_cast<const uint8_t*>(comm.rxPkt.data.ptr),
                                          hdr.size)) {
            // SN-8339: an upTime that DROPS relative to the previous bridge record (in arrival
            // order) means the device rebooted — open a new power-on session starting at this
            // record. Checked regardless of GPS-time validity, since a fresh boot is typically
            // pre-fix.
            if (bp->upSec > 0.0) {
                if (prevUpTimeSec >= 0.0 && bp->upSec < prevUpTimeSec - 0.5) {
                    sessAccum.push_back(SessionAccum{ thisArrival, {} });
                }
                prevUpTimeSec = bp->upSec;
            }
            // Only trust timeOfWeekMs as GPS ToW when the device says so. Otherwise it is LOCAL
            // system time (uptime-like), and differencing it against upTime yields a bogus offset
            // that corrupts the median bridge.
            const bool towValid = bp->towValid;
            if (towValid && bp->towMs > 0 && bp->upSec > 0.0) {
                const int64_t off = static_cast<int64_t>(bp->towMs)
                                  - static_cast<int64_t>(bp->upMs);
                upOffsetsOut.push_back(off);                       // global (SN-8323) offset samples
                if (!sessAccum.empty()) sessAccum.back().upOffsets.push_back(off);  // per-session (SN-8339)
            }
            // Kyle 2026-09-07 (Option A): a bridge record was never a sync point itself, only
            // used to calibrate OTHER DIDs' bridge, so a log whose ONLY valid GPS time came from
            // one had zero sync points. Push it too.
            if (towValid && bp->towMs > 0) {
                ISSyncPoint sysSp{};
                sysSp.hostTimeMs       = bp->towMs;
                sysSp.payloadToWMs     = bp->towMs;
                sysSp.deviceId         = deviceId;
                sysSp.sourceDid        = hdr.id;   // B1: SYS_PARAMS or GPX_STATUS
                sysSp.actualHostTimeMs = lastNonSyncHostTimeMs;
                out.push_back(sysSp);
            }
          }
        }

        // Probe every record for a payload-side timestamp. ToW-bearing
        // records use this for the sync's payloadToW; non-ToW-bearing
        // records' values feed lastNonSyncHostTimeMs.
        const double tsSec = cISDataMappings::Timestamp(&hdr, comm.rxPkt.data.ptr);

        if (!isToWBearing(hdr.id)) {
            // Non-sync candidate: if the payload carries a usable time
            // field, snapshot it. Most non-sync DIDs (DID_PIMU, DID_IMU,
            // etc.) carry a host-uptime time field in seconds;
            // cISDataMappings::Timestamp converts.
            if (tsSec > 0.0) {
                lastNonSyncHostTimeMs = static_cast<uint64_t>(tsSec * 1000.0);
            }
            continue;
        }

        if (tsSec <= 0.0) continue;  // payload's ToW field is zero / absent.

        const uint64_t towMs = static_cast<uint64_t>(tsSec * 1000.0);
        ISSyncPoint sp{};
        // v2 .idx schema collapse: the writer's rec.timestamp for
        // HAS_TOW records is also the ToW (no separate host-time stored
        // in the .idx). Carry the same value in both fields; the
        // .raw-recovered host-time at sync rides on actualHostTimeMs
        // (set below from the byte scan's lastNonSyncHostTimeMs tracker).
        sp.hostTimeMs        = towMs;
        sp.payloadToWMs      = towMs;
        sp.deviceId          = deviceId;
        sp.sourceDid         = hdr.id;
        // SN-8107 / D0066: recovered host-uptime at sync time.
        sp.actualHostTimeMs  = lastNonSyncHostTimeMs;
        // SN-8107 / D0066: GPS week from payload. All ToW-bearing DIDs
        // (ins_1_t, ins_2_t, ins_3_t, ins_4_t, gnss_pos_t) start with a
        // uint32_t week — read it from the first 4 bytes of the payload.
        // Zero means the device hasn't established a GPS week yet (still
        // searching); we keep the sync point but the resolver won't
        // epoch-anchor against it.
        if (comm.rxPkt.data.ptr && comm.rxPkt.dataHdr.size >= sizeof(uint32_t)) {
            uint32_t weekRaw = 0;
            std::memcpy(&weekRaw, comm.rxPkt.data.ptr, sizeof(weekRaw));
            sp.gpsWeek = weekRaw;
        }
        out.push_back(sp);
    }
}

/**
 * @brief SN-8323: choose the epoch-anchor GPS week from the log's durable,
 *        consistent fix period.
 *
 * The whole log is epoch-anchored to one GPS week (correct for a log spanning
 * < 1 GPS week — the common case). Picking `syncPoints_.front()` is wrong: the
 * sync list is sorted by ToW, so the front is typically a smallest-ToW pre-fix
 * `gpsWeek == 0` record (device still searching) — leaving the log unanchored
 * (ToW-only ~1980) while already-absolute records show the real year.
 *
 * During startup the reported time can be unstable (week 0, or a brief
 * transient/garbage week) before the device settles into a durable fix. So we
 * do NOT trust any single record or a raw popularity count. Instead, for each
 * non-zero week we measure the ToW SPAN it covers across the log; the week
 * backed by the widest span is the one the device held a stable fix at. A
 * startup transient covers a tiny span (a few close-together records) and loses
 * to the sustained fix. Ties break toward more sync points, then the larger
 * (more recent) week.
 *
 * @return  The durable-fix GPS week, or 0 if no non-zero week is present
 *          (caller falls back to ToW-only, pre-D0066 behavior).
 */
uint32_t chooseAnchorWeek(const std::vector<ISSyncPoint>& syncs) {
    struct Agg { uint64_t minTow; uint64_t maxTow; uint32_t count; };
    std::map<uint32_t, Agg> byWeek;   // week -> coverage (ordered ascending)
    for (const auto& sp : syncs) {
        if (sp.gpsWeek == 0) continue;
        auto it = byWeek.find(sp.gpsWeek);
        if (it == byWeek.end()) {
            byWeek.emplace(sp.gpsWeek, Agg{ sp.payloadToWMs, sp.payloadToWMs, 1 });
        } else {
            it->second.minTow = std::min(it->second.minTow, sp.payloadToWMs);
            it->second.maxTow = std::max(it->second.maxTow, sp.payloadToWMs);
            ++it->second.count;
        }
    }
    uint32_t bestWeek = 0, bestCount = 0;
    uint64_t bestSpan = 0;
    for (const auto& [wk, a] : byWeek) {   // ascending week -> larger week wins ties
        const uint64_t span = a.maxTow - a.minTow;
        if (span > bestSpan ||
            (span == bestSpan && a.count >= bestCount)) {
            bestSpan = span; bestCount = a.count; bestWeek = wk;
        }
    }
    return bestWeek;
}

// ============================================================
// Option B (Kyle 2026-09-07): file-timestamp anchor fallback
// ============================================================
//
// A log that never receives an external clock sync (no INS/GNSS DID, and no
// DID_SYS_PARAMS record ever reports HDW_STATUS_GNSS_TIME_OF_WEEK_VALID --
// Option A above still leaves such a log with zero sync points) has no
// payload-derived basis for a wall-clock anchor at all. Rather than resolve
// every record to SessionOnly/Unknown (which RawSeriesBuilder then drops
// entirely -- the log becomes completely unplottable), recover ONE coarse
// anchor from the log's own file: the segment's filename or an ancestor
// directory name, if it looks like cISLogger's own `..._YYYYMMDD_HHMMSS_..`
// convention (the common case -- this IS how cltool/cISLogger names a
// session), else the segment file's last-write time.

namespace fs = std::filesystem;

//! Days from the Unix epoch (1970-01-01) to (y, m, d), proleptic Gregorian.
//! Howard Hinnant's civil_from_days algorithm (public domain), needed because
//! this file is pure C++17 -- no <chrono> calendar support until C++20.
int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) noexcept {
    y -= (m <= 2);
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

//! Matches cISLogger's `..._YYYYMMDD_HHMMSS_...` (or trailing `_YYYYMMDD_HHMMSS`)
//! naming convention in a filename stem or directory name and converts it to
//! Unix-epoch ms. Basic range-sanity-checked (year/month/day/hour/min/sec) to
//! avoid treating an unrelated 8+6-digit run (e.g. a serial number followed by
//! a counter) as a timestamp. Treated as UTC -- the writer stamps local wall-
//! clock at capture time with no timezone recorded, and this anchor is already
//! explicitly a coarse approximation (FileTimeAnchored / TimeConfidence::Unknown),
//! so a timezone-sized offset doesn't change the plottability outcome.
std::optional<uint64_t> parseTimestampFromName(const std::string& name) noexcept {
    static const std::regex re(R"((\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2}))");
    std::smatch m;
    if (!std::regex_search(name, m, re)) return std::nullopt;

    const int year  = std::stoi(m[1].str());
    const int month = std::stoi(m[2].str());
    const int day   = std::stoi(m[3].str());
    const int hour  = std::stoi(m[4].str());
    const int min   = std::stoi(m[5].str());
    const int sec   = std::stoi(m[6].str());
    if (year < 2000 || year > 2100)  return std::nullopt;
    if (month < 1 || month > 12)     return std::nullopt;
    if (day < 1 || day > 31)         return std::nullopt;
    if (hour > 23 || min > 59 || sec > 59) return std::nullopt;

    const int64_t days = daysFromCivil(year, static_cast<unsigned>(month),
                                       static_cast<unsigned>(day));
    const int64_t secs = days * 86400 + hour * 3600 + min * 60 + sec;
    if (secs < 0) return std::nullopt;
    return static_cast<uint64_t>(secs) * 1000ull;
}

//! Tries `path`'s filename stem, then each ancestor directory name (up to 4
//! levels, past which a match is unlikely to actually describe this capture),
//! returning the first that parses as a timestamp.
std::optional<uint64_t> anchorFromPathNames(const fs::path& path) noexcept {
    if (auto v = parseTimestampFromName(path.stem().string())) return v;
    fs::path dir = path.parent_path();
    for (int depth = 0; depth < 4 && !dir.empty(); ++depth) {
        if (auto v = parseTimestampFromName(dir.filename().string())) return v;
        if (dir == dir.parent_path()) break;   // reached root
        dir = dir.parent_path();
    }
    return std::nullopt;
}

//! `path`'s last-write time as Unix-epoch ms, or nullopt on any filesystem error.
//! `std::filesystem::file_time_type`'s clock is unspecified pre-C++20, but on
//! every platform this project targets (libstdc++/libc++) it IS
//! `std::chrono::system_clock` in practice; this is the standard C++17
//! workaround (comparing against `system_clock::now()`/`file_time_type::clock::
//! now()` taken back-to-back) rather than an unavailable `clock_cast` (C++20).
std::optional<uint64_t> fileLastWriteTimeMs(const fs::path& path) noexcept {
    std::error_code ec;
    const auto ftime = fs::last_write_time(path, ec);
    if (ec) return std::nullopt;
    const auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        sctp.time_since_epoch()).count();
    if (ms < 0) return std::nullopt;
    return static_cast<uint64_t>(ms);
}

//! Largest non-zero raw `.idx` timestamp observed across every segment of
//! `log` -- used only as the ctime fallback's session-length estimate (see
//! `deriveFileAnchorMs`). A second full pass over the log, but this only runs
//! when the primary sync scan already found ZERO sync points, which is rare.
uint64_t maxRawTimestampMs(const ISDeviceLog& log) noexcept {
    uint64_t best = 0;
    for (std::size_t s = 0; s < log.segmentCount(); ++s) {
        for (auto v : log.segment(s).allRecords()) {
            best = std::max(best, v.timestamp().value);
        }
    }
    return best;
}

//! Kyle 2026-09-07 (Option B): the wall-clock instant corresponding to
//! host-uptime == 0 for `log`, recovered from its own file when the log has
//! no payload-level sync to derive one from. Tries the EARLIEST segment's
//! filename/ancestor-directory names first (cISLogger stamps the session
//! start into the name, so this needs no adjustment); falls back to that
//! segment's last-write time, adjusted backward by the log's observed
//! session span (last-write time approximates when the file was CLOSED, not
//! opened) so the anchor still lands near session start rather than session
//! end.
//!
//! @return  Anchor in Unix-epoch ms, or `std::nullopt` if neither the name
//!          nor the filesystem produced anything usable (caller's existing
//!          SessionOnly/Unknown fallback still applies in that case).
std::optional<uint64_t> deriveFileAnchorMs(const ISDeviceLog& log) {
    if (log.segmentCount() == 0) return std::nullopt;
    const fs::path& firstSegPath = log.segment(0).path();

    if (auto v = anchorFromPathNames(firstSegPath)) return v;

    if (auto ctimeMs = fileLastWriteTimeMs(firstSegPath)) {
        const uint64_t spanMs = maxRawTimestampMs(log);
        return (*ctimeMs > spanMs) ? (*ctimeMs - spanMs) : *ctimeMs;
    }
    return std::nullopt;
}

} // namespace

// ============================================================
// Detection
// ============================================================

std::vector<ISSyncPoint> ISTimeResolver::detectSyncPoints(const ISDeviceLog& log) {
    std::vector<int64_t> upOffsets;   // discarded here; build() consumes them.
    std::vector<Session> sessions;    // discarded here.
    return detectSyncPointsImpl(log, upOffsets, sessions);
}

std::vector<ISSyncPoint> ISTimeResolver::detectSyncPointsImpl(
    const ISDeviceLog& log, std::vector<int64_t>& upOffsetsOut,
    std::vector<Session>& sessionsOut,
    std::vector<StalledRun>* stalledOut,
    std::vector<std::pair<uint64_t, uint64_t>>* timelineOut) {
    std::vector<ISSyncPoint> out;
    const uint64_t deviceId = log.deviceId();

    // SN-8339: partition records into power-on sessions during the scan. The
    // first session starts at arrival index 0; each SYS_PARAMS.upTime drop opens
    // another. Arrival index is global across segments (matches allRecords()).
    uint64_t arrivalIndex  = 0;
    double   prevUpTimeSec = -1.0;
    std::vector<SessionAccum> sessAccum;
    sessAccum.push_back(SessionAccum{ 0, {} });
    StallWatcher stalls;

    for (std::size_t s = 0; s < log.segmentCount(); ++s) {
        scanSegmentForSyncs(log.segment(s), deviceId, out, upOffsetsOut,
                            arrivalIndex, prevUpTimeSec, sessAccum, stalls);
    }
    // A stall that runs to the final record -- the customer case -- is only reportable once the
    // scan ends, so close any run still open.
    stalls.finish();
    if (stalledOut)  *stalledOut  = stalls.takeRuns();
    if (timelineOut) *timelineOut = stalls.takeTimeline();

    // Finalize sessions: arrivalEnd = next session's start - 1 (last record for
    // the final session); per-session offset = median of its own SYS_PARAMS
    // samples (haveOffset=false when it had none — build() fills the fallback).
    const uint64_t lastArrival = (arrivalIndex == 0) ? 0 : arrivalIndex - 1;
    for (std::size_t i = 0; i < sessAccum.size(); ++i) {
        Session sess{};
        sess.arrivalStart = sessAccum[i].arrivalStart;
        sess.arrivalEnd   = (i + 1 < sessAccum.size())
                                ? (sessAccum[i + 1].arrivalStart - 1)
                                : lastArrival;
        sess.haveOffset   = !sessAccum[i].upOffsets.empty();
        sess.uptimeToTowOffsetMs =
            sess.haveOffset ? medianOf(sessAccum[i].upOffsets) : 0;
        sessionsOut.push_back(sess);
    }

    // Sort by hostTimeMs (the build's downstream expectation). Records
    // are already in arrival order across segments, but a multi-segment
    // log with overlapping ranges or a clock jump may produce out-of-
    // order timestamps — be safe.
    std::sort(out.begin(), out.end(),
              [](const ISSyncPoint& a, const ISSyncPoint& b) {
                  return a.hostTimeMs < b.hostTimeMs;
              });

    // Adjacent-duplicate filtering: many DIDs share a parent update's
    // ToW, so consecutive sync points often carry the same value. The
    // resolver's slope math doesn't benefit from duplicates (and the
    // discontinuity detector needs distinct neighbors).
    out.erase(std::unique(out.begin(), out.end(),
                          [](const ISSyncPoint& a, const ISSyncPoint& b) {
                              return a.hostTimeMs == b.hostTimeMs;
                          }),
              out.end());
    return out;
}

// ============================================================
// Build
// ============================================================

ISExpected<ISTimeResolver>
ISTimeResolver::build(const ISDeviceLog& log) {
    return build(log, kDefaultDiscontinuityThreshold);
}

ISExpected<ISTimeResolver>
ISTimeResolver::build(const ISDeviceLog& log, double threshold) {
    std::vector<int64_t> upOffsets;
    std::vector<Session> sessions;
    std::vector<StalledRun> stalled;
    std::vector<std::pair<uint64_t, uint64_t>> towTimeline;
    auto syncs = detectSyncPointsImpl(log, upOffsets, sessions, &stalled, &towTimeline);

    std::vector<Discontinuity> discs;
    if (syncs.size() >= 3) {
        // Walk consecutive triplets; compare the slope of segment
        // (i-1, i) against (i, i+1). A ratio change beyond `threshold`
        // marks a discontinuity at sync `i`.
        for (std::size_t i = 1; i + 1 < syncs.size(); ++i) {
            const double sBefore = slopeBetween(syncs[i - 1], syncs[i]);
            const double sAfter  = slopeBetween(syncs[i],     syncs[i + 1]);
            if (sBefore <= 0.0 || sAfter <= 0.0) continue;
            const double ratio = std::abs(sAfter - sBefore) / std::max(sBefore, sAfter);
            if (ratio > threshold) {
                discs.push_back(Discontinuity{
                    syncs[i].hostTimeMs,
                    sBefore,
                    sAfter,
                });
            }
        }
    }

    // SN-8323: pick the epoch-anchor week + the start of its durable fix window
    // (earliest ToW at that week) before moving `syncs`.
    const uint32_t anchorWeek = chooseAnchorWeek(syncs);
    uint64_t anchorTowStart = 0;
    uint64_t anchorTowEnd   = 0;
    if (anchorWeek != 0) {
        uint64_t minTow = UINT64_MAX;
        uint64_t maxTow = 0;
        for (const auto& sp : syncs) {
            if (sp.gpsWeek == anchorWeek) {
                minTow = std::min(minTow, sp.payloadToWMs);
                maxTow = std::max(maxTow, sp.payloadToWMs);
            }
        }
        anchorTowStart = (minTow == UINT64_MAX) ? 0 : minTow;
        anchorTowEnd   = maxTow;
    }

    // SN-8323 (uptime unification): authoritative uptime->ToW offset = median of
    // the synced DID_SYS_PARAMS (timeOfWeekMs - upTime) samples. upTime and ToW
    // advance 1:1, so all synced samples agree modulo clock drift; the median
    // resists an occasional transient sample.
    int64_t uptimeToTowOffsetMs = 0;
    bool    haveUptimeOffset    = false;
    if (!upOffsets.empty()) {
        std::sort(upOffsets.begin(), upOffsets.end());
        uptimeToTowOffsetMs = upOffsets[upOffsets.size() / 2];
        haveUptimeOffset    = true;
    }

    // SN-8339: a session with no synced SYS_PARAMS of its own inherits the
    // log-global offset (best available) rather than 0, so its uptime records
    // still bridge; sessions with their own samples keep their per-session median.
    for (auto& sess : sessions) {
        if (!sess.haveOffset && haveUptimeOffset) {
            sess.uptimeToTowOffsetMs = uptimeToTowOffsetMs;
            sess.haveOffset          = true;
        }
    }

    // Kyle 2026-09-07 (Option B): a log with zero sync points (no INS/GNSS, and
    // Option A above found no synced DID_SYS_PARAMS either) has no payload basis
    // for a wall-clock anchor at all -- recover one from the log's own file
    // rather than leave every record SessionOnly/Unknown (RawSeriesBuilder drops
    // those outright).
    uint64_t fileAnchorMs  = 0;
    bool     haveFileAnchor = false;
    if (syncs.empty()) {
        if (auto anchor = deriveFileAnchorMs(log)) {
            fileAnchorMs   = *anchor;
            haveFileAnchor = true;
        }
    }

    ISTimeResolver r{ std::move(syncs), std::move(discs), anchorWeek,
                      anchorTowStart, anchorTowEnd,
                      uptimeToTowOffsetMs, haveUptimeOffset,
                      fileAnchorMs, haveFileAnchor,
                      std::move(sessions) };
    // SN-8704: assigned rather than threaded through the constructor -- these are diagnostic
    // state, not part of the resolve identity, and the ctor is already ten arguments deep.
    r.stalledRuns_  = std::move(stalled);
    r.towTimeline_  = std::move(towTimeline);

    // SN-8704: decide, per run, whether the device's OWN clock may serve as the ruler.
    //
    // Kyle's rule: do not trust the concussed witness -- but once independent witnesses
    // corroborate its account, its detailed account is the best source available. So the
    // collective timeline decides WHETHER to trust; the device's own counter then decides WHERE
    // each record goes, because it has 0.5 s resolution while arrival-order bracketing skews
    // wherever the record rate shifts (and it shifts exactly at a stall -- the GNSS DIDs stop).
    for (auto& run : r.stalledRuns_) {
        if (run.retimed.size() < 2 || r.towTimeline_.size() < 2) continue;
        const uint64_t ownDelta = run.retimed.back().second - run.retimed.front().second;
        const uint64_t tlStart  = interpolateArrivalTime(r.towTimeline_, run.arrivalStart);
        const uint64_t tlEnd    = interpolateArrivalTime(r.towTimeline_, run.arrivalEnd);
        if (tlEnd <= tlStart || ownDelta == 0) { run.retimed.clear(); continue; }
        const uint64_t tlDelta = tlEnd - tlStart;
        run.rulerRatio = static_cast<double>(ownDelta) / static_cast<double>(tlDelta);
        // 5% is generous for an oscillator but tight enough to reject a clock that is not
        // actually tracking real time. The motivating case measured 1.0016.
        run.rulerCorroborated = (run.rulerRatio > 0.95 && run.rulerRatio < 1.05);
        if (!run.rulerCorroborated) {
            log_warn(IS_LOG_ISLOG,
                     "ISTimeResolver: DID %u %s ruler advanced %llu ms while the collective "
                     "timeline advanced %llu ms (ratio %.4f) -- NOT corroborated, falling back "
                     "to arrival-order bracketing",
                     run.did,
                     run.ruler == StalledRun::Ruler::LogTimeOffset ? "idx-local-delta"
                         : run.ruler == StalledRun::Ruler::OwnClock ? "own-clock" : "cadence",
                     (unsigned long long)ownDelta, (unsigned long long)tlDelta, run.rulerRatio);
            run.retimed.clear();
            run.ruler = StalledRun::Ruler::None;
        }
    }

    if (!r.stalledRuns_.empty()) {
        for (const auto& run : r.stalledRuns_) {
            log_warn(IS_LOG_ISLOG,
                     "ISTimeResolver: DID %u stamped clock STALLED at %llu ms for %zu records "
                     "(arrival %llu..%llu) -- those records will be re-timed against the "
                     "collective timeline, not their own clock",
                     run.did, (unsigned long long)run.stalledTsMs, run.recordCount,
                     (unsigned long long)run.arrivalStart, (unsigned long long)run.arrivalEnd);
        }
    }
    return r;
}

std::vector<std::pair<uint64_t, uint64_t>>
ISTimeResolver::planStallRetiming(const StallEvidence& ev, StalledRun::Ruler& outKind) {
    outKind = StalledRun::Ruler::None;
    std::vector<std::pair<uint64_t, uint64_t>> out;
    if (ev.runArrivals.size() < 2) return out;

    // MOST PREFERRED: the .idx per-record receipt delta. This is the WHEN, stamped for every
    // record regardless of DID, so it needs no payload field and no assumption about output
    // rate. The run's first record is still healthy, so its delta is the zero point.
    if (ev.logTimeOffsets.size() >= 2) {
        const uint64_t base = ev.logTimeOffsets.front().second;
        out.reserve(ev.logTimeOffsets.size());
        for (const auto& [arr, local] : ev.logTimeOffsets) {
            out.emplace_back(arr, ev.stalledTsMs + (local >= base ? local - base : 0));
        }
        outKind = StalledRun::Ruler::LogTimeOffset;
        return out;
    }

    // NEXT: the DID's own companion uptime, one answer per record. Projected into the ToW
    // frame through the run's first (still-healthy) pairing, so the seam is exact.
    if (ev.lastHealthyOwnMs != 0 && ev.ownSamples.size() >= 2) {
        const int64_t off = static_cast<int64_t>(ev.stalledTsMs) -
                            static_cast<int64_t>(ev.lastHealthyOwnMs);
        out.reserve(ev.ownSamples.size());
        for (const auto& [arr, own] : ev.ownSamples) {
            out.emplace_back(arr, static_cast<uint64_t>(static_cast<int64_t>(own) + off));
        }
        outKind = StalledRun::Ruler::OwnClock;
        return out;
    }

    // GENERAL FALLBACK: 25 of the 27 ToW-bearing record types have no companion uptime, so most
    // stalls land here. The DID's own median inter-record interval, applied uniformly, assumes
    // only that its output rate is steady -- far weaker than assuming the GLOBAL record rate is
    // steady, which is what arrival-order bracketing needs and which a stall routinely breaks
    // (the GNSS DIDs stop emitting at the same instant the clock freezes).
    if (!ev.advanceDeltas.empty()) {
        std::vector<uint64_t> d = ev.advanceDeltas;
        std::sort(d.begin(), d.end());
        const uint64_t cadence = d[d.size() / 2];
        if (cadence > 0) {
            out.reserve(ev.runArrivals.size());
            for (std::size_t k = 0; k < ev.runArrivals.size(); ++k) {
                out.emplace_back(ev.runArrivals[k],
                                 ev.stalledTsMs + static_cast<uint64_t>(k) * cadence);
            }
            outKind = StalledRun::Ruler::Cadence;
        }
    }
    return out;
}

uint64_t ISTimeResolver::interpolateArrivalTime(
    const std::vector<std::pair<uint64_t, uint64_t>>& anchors, uint64_t arrivalIndex) {
    // Lifted from Logalyzer's RawSeriesBuilder (SN-8131). Precondition: non-empty, ascending.
    if (anchors.empty()) return 0;
    auto hi = std::lower_bound(
        anchors.begin(), anchors.end(), arrivalIndex,
        [](const std::pair<uint64_t, uint64_t>& a, uint64_t k) { return a.first < k; });
    if (hi == anchors.begin()) return anchors.front().second;   // at/before the first anchor
    if (hi == anchors.end())   return anchors.back().second;    // after the last anchor
    const auto& lo = *(hi - 1);
    const auto& up = *hi;
    const uint64_t span = up.first - lo.first;
    if (span == 0) return lo.second;
    const double frac = static_cast<double>(arrivalIndex - lo.first) /
                        static_cast<double>(span);
    return lo.second + static_cast<uint64_t>(
               (static_cast<double>(up.second) - static_cast<double>(lo.second)) * frac);
}

// ============================================================
// Resolve
// ============================================================

// SN-8339: the two public overloads are thin wrappers; all resolution logic
// lives here, parameterized by the uptime->ToW bridge offset to use. The no-key
// overload passes the log-global offset (byte-identical to pre-SN-8339
// behavior); the arrival-keyed overload passes the per-session offset selected
// from `sessions_`, so a uptime value from any power-on session bridges against
// that session's own median rather than a single global offset that can only be
// right for one boot.
TimeStamp ISTimeResolver::resolveImpl(uint64_t hostTimeMs, uint64_t deviceId,
                                      int64_t uptimeOffsetMs,
                                      bool haveOffset) const {
    // SN-8115: idempotency / already-anchored guard. Legitimate raw inputs are
    // either a GPS time-of-week (always < one week, 604,800,000 ms) or a
    // session-uptime (ms since session start — at most hours). Neither can
    // reach the GPS Unix epoch (1980-01-06 = 315,964,800,000 ms). An input at
    // or beyond that is therefore ALREADY an absolute Unix-ms timestamp — a
    // value that previously went through `resolve()` (re-resolving it must be a
    // no-op for `resolve(resolve(x)) == resolve(x)` to hold), or a
    // wall-clock-poisoned `.idx` span endpoint. Re-anchoring it would
    // double-add the epoch + week offset via `gpsToUnixMs`, producing ~2x the
    // wall-clock (the year-2082 spanEnd seen on the 16-device compass fixture).
    // Pass it through unchanged.
    if (hostTimeMs >= kGpsEpochUnixMs) {
        return TimeStamp::fromResolvedViaSync(hostTimeMs, deviceId,
                                              TimeConfidence::Exact);
    }

    if (syncPoints_.empty()) {
        // No payload-derived anchors. Kyle 2026-09-07 (Option B): if build() found
        // a usable file-timestamp anchor (the log's filename/directory name, or its
        // last-write time), every input here IS host-uptime domain (there's no
        // sync point to have classified it otherwise) -- add the anchor directly.
        if (haveFileAnchor_) {
            return TimeStamp::fromFileTimeAnchored(fileAnchorMs_ + hostTimeMs, deviceId);
        }
        // No anchor of any kind. Best we can do is a SessionOnly tag with the
        // input value passed through.
        return TimeStamp::fromSessionOnly(hostTimeMs, deviceId);
    }

    // SN-8107 / D0066: select a representative GPS week for the epoch anchor.
    // SN-8323: use the durable-fix week (`anchorWeek_`, precomputed in build()
    // as the non-zero week with the widest ToW coverage), NOT
    // `syncPoints_.front().gpsWeek`.
    // The sync list is sorted by ToW, so front() is the smallest-ToW record,
    // which on a pre-GPS-fix log is a week-0 record — anchoring to it left the
    // log unanchored (ToW-only ~1980) while already-absolute records showed the
    // real year, giving a ~46-year mixed-domain span. `firstSync` is still the
    // ToW-frame reference for the session-uptime bridge below. anchorWeek_ == 0
    // (no valid week anywhere) falls back to ToW-only (pre-D0066 behavior).
    const ISSyncPoint& firstSync = syncPoints_.front();
    const uint32_t anchorWeek = anchorWeek_;
    const bool     epochAnchor = (anchorWeek != 0);
    auto unixOrToW = [&](uint64_t towMs) -> uint64_t {
        return epochAnchor ? gpsToUnixMs(anchorWeek, towMs) : towMs;
    };

    // SN-8323 (uptime unification): authoritative uptime->ToW bridge. When a
    // synced DID_SYS_PARAMS supplied the definitive offset, classify the input
    // against the durable-fix ToW window [anchorTowStart_, anchorTowEnd_]:
    //   - already inside the window  -> ToW-domain, fall through to the normal
    //     sync-point resolution below;
    //   - outside, but (input + offset) lands inside -> a session-uptime value
    //     (magnetometer/imu, or a pre-sync "real clock" record whose ToW field
    //     defaulted to uptime until GPS lock) -> bridge via the authoritative
    //     offset.
    // This supersedes the fragile per-sync actualHostTimeMs heuristic (below)
    // and stops tiny pre-sync ToW values from poisoning the bridge — the mag
    // records that used to resolve to the GPS-week start (days early) now land
    // correctly in the fix window. (Kyle 2026-07-23.)
    if (haveOffset && epochAnchor && anchorTowEnd_ >= anchorTowStart_) {
        const int64_t guard    = static_cast<int64_t>(kPreFixGuardMs);
        const int64_t lo       = static_cast<int64_t>(anchorTowStart_);
        const int64_t hi       = static_cast<int64_t>(anchorTowEnd_);
        const int64_t raw      = static_cast<int64_t>(hostTimeMs);
        const bool    rawIsTow = (raw >= lo - guard && raw <= hi + guard);
        if (!rawIsTow) {
            const int64_t bridged = raw + uptimeOffsetMs;
            if (bridged >= lo - guard && bridged <= hi + guard) {
                const uint64_t towMs = (bridged < 0) ? 0u
                                                     : static_cast<uint64_t>(bridged);
                const TimeConfidence conf =
                    (bridged < lo) ? TimeConfidence::ExtrapolatedBackward
                                   : TimeConfidence::Interpolated;
                return TimeStamp::fromResolvedViaSync(
                    gpsToUnixMs(anchorWeek, towMs), deviceId, conf);
            }
        }
    }

    // SN-8107 / D0066: cross-domain bridge. v2 .idx puts sync records'
    // timestamp field in the GPS-ToW domain (hundreds of millions of ms
    // into the GPS week) while non-sync records' timestamp field is host
    // uptime (small ms since session start). An input hostTimeMs that's
    // dramatically smaller than the first sync's ToW is a session-uptime
    // query — translate it into the ToW frame using the recovered
    // actualHostTimeMs of the first sync, then epoch-anchor the result
    // if GPS week is known.
    if (firstSync.actualHostTimeMs > 0 &&
        hostTimeMs < firstSync.payloadToWMs / 2) {
        const int64_t offset =
            static_cast<int64_t>(firstSync.payloadToWMs) -
            static_cast<int64_t>(firstSync.actualHostTimeMs);
        const int64_t bridged = static_cast<int64_t>(hostTimeMs) + offset;
        const uint64_t towMs = (bridged < 0) ? 0u : static_cast<uint64_t>(bridged);
        return TimeStamp::fromResolvedViaSync(unixOrToW(towMs), deviceId,
                                              TimeConfidence::ExtrapolatedBackward);
    }

    // SN-8323 (part 2): a ToW-domain input that falls well before the durable
    // fix period began is a pre-fix / startup record — the device had no stable
    // GPS time yet (e.g. a ToW near the GPS week start while the fix is days
    // into the week). Epoch-anchoring it would place it at a bogus early time
    // and drag the log extent back (the "leading gap"). Tag it
    // SessionOnly/Unknown — the same contract as a record with no anchor at all
    // — so every SDK consumer excludes it from the timeline + extent uniformly.
    if (epochAnchor && anchorTowStart_ > kPreFixGuardMs &&
        hostTimeMs + kPreFixGuardMs < anchorTowStart_) {
        return TimeStamp::fromSessionOnly(hostTimeMs, deviceId);
    }

    // Binary search for the first sync point whose hostTimeMs >= input.
    auto it = std::lower_bound(
        syncPoints_.begin(), syncPoints_.end(), hostTimeMs,
        [](const ISSyncPoint& sp, uint64_t v) { return sp.hostTimeMs < v; });

    if (it != syncPoints_.end() && it->hostTimeMs == hostTimeMs) {
        // Exact match against a sync point.
        return TimeStamp::fromPayloadToW(unixOrToW(it->payloadToWMs), deviceId);
    }

    if (it == syncPoints_.begin()) {
        // Before the first sync. Project backward using the slope of
        // the first two sync points (or 1.0 if only one sync point).
        const ISSyncPoint& s0 = syncPoints_.front();
        double slope = 1.0;
        if (syncPoints_.size() >= 2) {
            slope = slopeBetween(syncPoints_[0], syncPoints_[1]);
        }
        const double delta = static_cast<double>(hostTimeMs) - static_cast<double>(s0.hostTimeMs);
        const double tow   = static_cast<double>(s0.payloadToWMs) + slope * delta;
        const uint64_t towMs = (tow < 0.0) ? 0u : static_cast<uint64_t>(tow);
        return TimeStamp::fromResolvedViaSync(unixOrToW(towMs), deviceId,
                                              TimeConfidence::ExtrapolatedBackward);
    }

    if (it == syncPoints_.end()) {
        // Past the last sync. Project forward using the slope of the
        // last two sync points (or 1.0 if only one).
        const ISSyncPoint& sLast = syncPoints_.back();
        double slope = 1.0;
        if (syncPoints_.size() >= 2) {
            const ISSyncPoint& sPrev = syncPoints_[syncPoints_.size() - 2];
            slope = slopeBetween(sPrev, sLast);
        }
        const double delta = static_cast<double>(hostTimeMs) - static_cast<double>(sLast.hostTimeMs);
        const double tow   = static_cast<double>(sLast.payloadToWMs) + slope * delta;
        const uint64_t towMs = (tow < 0.0) ? 0u : static_cast<uint64_t>(tow);
        return TimeStamp::fromResolvedViaSync(unixOrToW(towMs), deviceId,
                                              TimeConfidence::ExtrapolatedForward);
    }

    // Interpolation between `prev` (it - 1) and `it`.
    const ISSyncPoint& prev = *(it - 1);
    const ISSyncPoint& next = *it;
    const double hostSpan = static_cast<double>(next.hostTimeMs) - static_cast<double>(prev.hostTimeMs);
    const double towSpan  = static_cast<double>(next.payloadToWMs) - static_cast<double>(prev.payloadToWMs);
    const double frac = (hostSpan == 0.0)
                      ? 0.0
                      : (static_cast<double>(hostTimeMs) - static_cast<double>(prev.hostTimeMs)) / hostSpan;
    const double tow  = static_cast<double>(prev.payloadToWMs) + frac * towSpan;
    return TimeStamp::fromResolvedViaSync(unixOrToW(static_cast<uint64_t>(tow)),
                                          deviceId,
                                          TimeConfidence::Interpolated);
}

TimeStamp ISTimeResolver::resolve(uint64_t hostTimeMs, uint64_t deviceId,
                                  uint64_t arrivalIndex) const {
    // SN-8704: does this record belong to a run where its DID's stamped clock had STOPPED?
    // If so its own timestamp is worthless -- every record in the run carries the same frozen
    // value, which is what collapses a whole device's tail onto one instant. Re-time it from
    // its position among the witnesses that were still working, rather than believing it.
    //
    // This is checked BEFORE the session/offset logic below because the input `hostTimeMs` is
    // the very value we have decided not to trust; bridging it would just launder a known-bad
    // number through an otherwise-correct offset.
    if (!stalledRuns_.empty() && !towTimeline_.empty() &&
        arrivalIndex != ISRecordView::kNoArrivalIndex) {
        for (const auto& run : stalledRuns_) {
            if (arrivalIndex < run.arrivalStart || arrivalIndex > run.arrivalEnd) continue;
            // Copilot review, #1316: the interval test above is necessary but NOT sufficient. A
            // stall belongs to ONE DID, and every other DID's record arriving inside its window
            // was being re-timed as though its own clock were frozen -- throwing away a good
            // timestamp for an interpolation. On the customer capture a GPX_STATUS stall spanning
            // 959 s would have re-timed every PIMU and INS record in that window.
            //
            // `resolve()` has no DID parameter (and adding one would change every caller), so
            // membership is tested against the stalled DID's OWN arrival indices instead.
            // `run.arrivals` is ascending, so this is a binary search.
            if (!std::binary_search(run.arrivals.begin(), run.arrivals.end(), arrivalIndex)) {
                continue;
            }
            // Prefer the device's own corroborated clock; fall back to bracketing against the
            // collective timeline when it has none or it could not be corroborated.
            uint64_t towMs = 0;
            if (!run.retimed.empty()) {
                const auto it = std::lower_bound(
                    run.retimed.begin(), run.retimed.end(), arrivalIndex,
                    [](const std::pair<uint64_t, uint64_t>& a, uint64_t k) { return a.first < k; });
                if (it != run.retimed.end() && it->first == arrivalIndex) {
                    towMs = it->second;
                } else if (it != run.retimed.begin()) {
                    towMs = (it - 1)->second;   // a record of another DID inside the run's span
                }
            }
            if (towMs == 0) towMs = interpolateArrivalTime(towTimeline_, arrivalIndex);
            if (towMs == 0) break;
            // Same epoch-anchoring rule the normal path uses (SN-8323): project onto Unix ms
            // via the durable-fix week when there is one, else stay in the ToW frame. Using a
            // different rule here would put re-timed records in a different frame from their
            // healthy neighbours -- the exact D0066 violation this work exists to remove.
            const uint64_t absMs = (anchorWeek_ != 0) ? gpsToUnixMs(anchorWeek_, towMs) : towMs;
            // Reconstructed, and forward of the last anchor this device itself supplied --
            // exactly what {ResolvedViaSync, ExtrapolatedForward} means. D-58's dashed
            // rendering keys off confidence, so these draw as reconstructed for free rather
            // than passing for measured.
            return TimeStamp::fromResolvedViaSync(absMs, deviceId,
                                                  TimeConfidence::ExtrapolatedForward);
        }
    }

    // SN-8339: with a known arrival index and more than one power-on session,
    // pick the session whose arrival range covers this record and bridge with
    // that session's own offset. A single-session log (the common case) is
    // identical to the no-key path, so this overload is a strict superset.
    if (sessions_.size() > 1) {
        for (const auto& sess : sessions_) {
            if (arrivalIndex >= sess.arrivalStart &&
                arrivalIndex <= sess.arrivalEnd) {
                return resolveImpl(hostTimeMs, deviceId,
                                   sess.uptimeToTowOffsetMs, sess.haveOffset);
            }
        }
    }
    return resolveImpl(hostTimeMs, deviceId, uptimeToTowOffsetMs_,
                       haveUptimeOffset_);
}

ISTimeResolver::Stats ISTimeResolver::computeStats(const ISDeviceLog& log) const {
    Stats s{};
    for (auto v : log.allRecords()) {
        const TimeStamp t = resolve(v.timestamp().value, log.deviceId(),
                                    v.arrivalIndex());
        switch (t.confidence) {
            case TimeConfidence::Exact:                ++s.exact;        break;
            case TimeConfidence::Interpolated:         ++s.interpolated; break;
            case TimeConfidence::ExtrapolatedForward:  ++s.extrapFwd;    break;
            case TimeConfidence::ExtrapolatedBackward: ++s.extrapBack;   break;
            case TimeConfidence::Unknown:              ++s.unknown;      break;
        }
    }
    return s;
}


// ============================================================================================
// SN-8784 — one mechanism for a record's probable absolute time, plus its inverse.
// ============================================================================================

const char* absTimeMechanismName(AbsTimeMechanism m) noexcept {
    switch (m) {
        case AbsTimeMechanism::Unresolved:      return "Unresolved";
        case AbsTimeMechanism::NoTimeField:     return "NoTimeField";
        case AbsTimeMechanism::FrozenAndUnbounded: return "FrozenAndUnbounded";
        case AbsTimeMechanism::PayloadEpoch:    return "PayloadEpoch";
        case AbsTimeMechanism::CarriedWeek:     return "CarriedWeek";
        case AbsTimeMechanism::SyncMatched:     return "SyncMatched";
        case AbsTimeMechanism::UptimeProjected: return "UptimeProjected";
        case AbsTimeMechanism::FileAnchor:      return "FileAnchor";
        case AbsTimeMechanism::InterpolatedFromNeighbours: return "InterpolatedFromNeighbours";
    }
    return "?";
}

const char* absAnchorSourceName(AbsAnchorSource a) noexcept {
    switch (a) {
        case AbsAnchorSource::None:            return "None";
        case AbsAnchorSource::PayloadWeek:     return "PayloadWeek";
        case AbsAnchorSource::IdxCaptureEpoch: return "IdxCaptureEpoch";
        case AbsAnchorSource::Filename:        return "Filename";
    }
    return "?";
}

const char* positionExactnessName(PositionExactness e) noexcept {
    switch (e) {
        case PositionExactness::NotInLog:          return "NotInLog";
        case PositionExactness::Exact:             return "Exact";
        case PositionExactness::FirstOfStalledRun: return "FirstOfStalledRun";
        case PositionExactness::Preceding:         return "Preceding";
        case PositionExactness::Before:            return "Before";
        case PositionExactness::After:             return "After";
    }
    return "?";
}

namespace {

//! Weeks below this cannot be a real capture. GPS week 1043 is 2000-01-01; a device with no fix
//! reports a small week (week 1 is routine on the customer corpus) and week 1 is 1980-01-13, so a
//! value in that range is a device saying "I do not know" rather than a date.
//! Kyle's fix threshold, 2026-10-02. See `kGnssFixWeekThreshold` for the reasoning.
constexpr uint32_t kMinPlausibleWeek = kGnssFixWeekThreshold;

//! 2000-01-01, the floor below which a value cannot be a plausible Unix capture time.
constexpr uint64_t kUnixPlausibleFloorMs = 946684800000ULL;

/**
 * @brief The GPS week implied by an absolute instant.
 *
 * Used to recover a week from the filename anchor when no payload in the log supplies a plausible
 * one — the last resort, and tagged as such by the caller.
 *
 * @param unixMs  Absolute instant.
 * @return        The GPS week containing it, or 0 when @p unixMs predates the GPS epoch.
 */
uint32_t weekOfUnixMs(uint64_t unixMs) {
    constexpr uint64_t kGpsEpochUnixMs = 315964800000ULL;
    constexpr uint64_t kMsPerWeek      = 604800000ULL;
    if (unixMs < kGpsEpochUnixMs) return 0;
    return static_cast<uint32_t>((unixMs - kGpsEpochUnixMs) / kMsPerWeek);
}

/**
 * @brief Downgrades a result to relative-only: no absolute, but a usable relative clock.
 *
 * Kyle, 2026-10-02: a log may legitimately have no means to anchor to any clock source, and that
 * is an EXPECTED result — but it must still have a basis for a relative clock. The cascade already
 * computes the log's uptime zero for exactly this purpose ("subtracting this zero turns any
 * record's uptime into elapsed-time-into-the-log"), so that is what is used.
 *
 * @param out     The partially-filled result; its hints are preserved.
 * @param anchor  The owning segment's anchor analysis.
 * @param rawMs   The record's raw timestamp.
 * @return        @p out with `valid == false`, `relativeOnly` set when a relative clock exists.
 */
/**
 * @brief The relative-only answer for a record nothing could place absolutely.
 *
 * @param out        Partially-filled result.
 * @param anchor     The segment's cascade analysis.
 * @param rawMs      The record's raw sidecar value.
 * @param floorRawMs Last-resort origin: the smallest raw value seen in this segment, or 0 if
 *                   unknown. Used only when the cascade reports no uptime extrema of its own.
 *
 * Kyle's ruling, 2026-10-02: *"A log may legitimately have NO clock source. That is an EXPECTED
 * result and must still yield a relative clock."* The cascade's `uptimeMinMs` / `logStartUptimeMs`
 * are both zero on a segment whose every record is a declared-time-of-week DID, because the
 * collector never gathered an uptime extreme from one — so a log of nothing but week-1 `DID_INS_2`
 * records produced NO clock at all, absolute or relative. Measured 2026-10-03 on exactly that
 * fixture: `valid=0 relativeOnly=0`, three hints, and no usable time of any kind.
 *
 * The raw values are themselves a monotonic series, which is all a relative clock needs, so the
 * segment's own raw floor is the honest last-resort origin. It is explicitly LAST: a real uptime
 * extreme from the cascade is better evidence and keeps precedence.
 */
AbsTimeResult relativeOnlyResult(AbsTimeResult out, const AnchorAnalysis& anchor, uint64_t rawMs,
                                 uint64_t floorRawMs = 0) {
    out.valid      = false;
    out.absoluteMs = 0;
    out.source     = TimeSource::SessionOnly;
    out.confidence = TimeConfidence::Unknown;
    if (anchor.logStartUptimeMs != 0 && rawMs >= anchor.logStartUptimeMs) {
        out.relativeToLogMs = rawMs - anchor.logStartUptimeMs;
        out.relativeOnly    = true;
    }
    if (anchor.uptimeMinMs != 0 && rawMs >= anchor.uptimeMinMs) {
        out.relativeToSegmentMs = rawMs - anchor.uptimeMinMs;
        out.relativeOnly        = true;
    }
    if (!out.relativeOnly && floorRawMs != 0 && rawMs >= floorRawMs) {
        out.relativeToSegmentMs = rawMs - floorRawMs;
        out.relativeOnly        = true;
        out.hints.emplace_back("no uptime extrema on this segment; relative time is measured from "
                               "its smallest raw value instead");
    }
    if (!out.relativeOnly) {
        out.hints.emplace_back("no relative origin either: this segment reports no uptime extrema "
                               "and no usable raw floor");
    }
    return out;
}

} // namespace

AbsTimeResult ISTimeResolver::resolveAbsTimeCore(const ISDeviceLog& log,
                                                 std::size_t segmentIndex,
                                                 std::size_t recordIndex) const {
    AbsTimeResult out;
    if (segmentIndex >= log.segmentCount()) {
        out.hints.emplace_back("segment index out of range");
        return out;
    }
    const ISLogReader& seg = log.segment(segmentIndex);
    if (recordIndex >= seg.recordCount()) {
        out.hints.emplace_back("record index out of range for this segment");
        return out;
    }
    out.segmentIndex = segmentIndex;

    const ISRecordView rec = seg.recordAt(recordIndex);
    out.sidecarRawMs       = rec.timestamp().value;
    const AnchorAnalysis& anchor = seg.anchorAnalysis();
    out.anchorTier = anchor.tier;
    out.offsetMs   = anchor.offsetMs;
    out.anchorMs   = anchor.anchoredStartMs;

    // ---- Step 1: what domain is the raw value in? The DID's DECLARED domain decides, never the
    // value's magnitude — the same rule `ISAnchorAnalysis` follows, and for the same reason: a
    // legal zero (ToW 0 is Sunday midnight, uptime 0 the first ms after boot) is indistinguishable
    // from "absent" by magnitude alone.
    using TsDomain = cISDataMappings::eTimestampDomain;
    const TsDomain domain = cISDataMappings::TimestampDomain(rec.did());

    // A stuck field is refused outright. `resolveAbsTime` interpolates these from the nearest live
    // records either side in arrival order; the core must not try, or it would recurse.
    ensureOrigins(log);
    if (didFieldIsFrozen(rec.did())) {
        out.frozenField = true;
        out.hints.emplace_back("this DID's time field is STUCK - identical on every record of this "
                               "DID across the log - so it carries no time of its own");
        return out;
    }
    if (domain == TsDomain::TIMESTAMP_DOMAIN_NONE) {
        // Not a failure to place a time — there was no time to place. Its own mechanism value so
        // that a caller counting failures is not counting `DID_DEV_INFO`.
        out.mechanism = AbsTimeMechanism::NoTimeField;
        out.hints.emplace_back("this DID declares no timestamp field");
        return out;
    }
    if (out.sidecarRawMs == 0 && domain == TsDomain::TIMESTAMP_DOMAIN_GPS_TOW) {
        // A genuine ToW of zero is legal, so this is not an early return on magnitude; it is only
        // noted, because a run of zeros is also what a device with no time at all writes.
        out.hints.emplace_back("time of week is zero; legal, but also what an untimed device writes");
    }

    // ---- Step 2: apply the segment anchor's offset, and work out WHICH FRAME it lands in.
    //
    // The cascade's `offsetMs` does not target a consistent frame: on a filename-anchored segment
    // it maps a raw uptime straight to Unix (this log: offset 1,789,601,170,131, so raw 869 ->
    // 2026-09-16 23:26:11.000), while on a ToW-tier segment it maps uptime into the GPS
    // time-of-week frame instead. Measured 2026-10-02 across 75 corpus logs / 709 segments:
    //
    //     tier                unix-like   ToW-like
    //     FilenameAnchor             21         24
    //     PayloadToWBridge            0        613
    //     PayloadToWSingle            0         48
    //     None                        0          3
    //
    // Note the FilenameAnchor split: the frame is NOT a function of the tier, so it cannot be
    // inferred from one. It is determined here by trying the Unix reading and checking whether the
    // result is a plausible capture date — self-checking, and it reports which branch fired.
    // A DECLARED time-of-week field is only a real time of week if the log ever got a fix.
    //
    // Kyle's rule, 2026-10-02: a log that never reaches week >= 1500 never achieved a GNSS fix.
    // That verdict governs the FIELD as well as the week. Measured on
    // `goldenlogs/imx/imx6/AHRS/20260521_113715`, whose payload week is 1: record 59640 declares
    // `GPS_TOW` and carries 296,725 — which is plainly uptime (296.7 s), sitting beside record 0's
    // 115,902 — and trusting it as a time of week placed that record on 2026-05-17, four days
    // before record 0's correct 2026-05-21 11:37:15. Two records, one segment, one anchor, four
    // days apart, purely because they took different branches here.
    //
    // So with no fix, every record goes through the anchor offset, whatever its DID declares.
    const bool logHasFix = anchorWeek_ >= kGnssFixWeekThreshold;
    const bool towDomain = (domain == TsDomain::TIMESTAMP_DOMAIN_GPS_TOW) && logHasFix;
    if (domain == TsDomain::TIMESTAMP_DOMAIN_GPS_TOW && !logHasFix) {
        out.hints.emplace_back("this DID declares a GPS time of week, but the log never reached "
                               "week " + std::to_string(kGnssFixWeekThreshold) +
                               " so the field is not a real time of week; treated as uptime");
    }
    if (!towDomain && anchor.tier == AnchorTier::None) {
        out.hints.emplace_back("uptime-domain record in a segment with no anchor at all");
        return relativeOnlyResult(out, anchor, out.sidecarRawMs,
                                  segmentRawFloorMs(segmentIndex));
    }

    const int64_t mapped = towDomain ? static_cast<int64_t>(out.sidecarRawMs)
                                     : static_cast<int64_t>(out.sidecarRawMs) + anchor.offsetMs;
    if (mapped < 0) {
        out.hints.emplace_back("segment offset drives this record's time below zero");
        return out;
    }

    // Kyle's external-anchor ORDER, 2026-10-02: with no usable payload week, the `.idx`
    // `capture_epoch_ms` comes FIRST and the filename is the worst case — the former is a
    // measurement the host actually took at log-open, the latter a string that merely looks like a
    // date. The direct-mapping branch below used to short-circuit that order entirely: on a
    // filename-anchored segment the cascade's own offset already maps a raw uptime straight to
    // Unix, so the function returned before the order was ever consulted, and a sidecar carrying a
    // perfectly good capture epoch went unused. Measured 2026-10-03 on a no-fix fixture whose
    // filename said 2020-06-15 and whose `.idx` capture epoch said 2023-03-01: the answer came back
    // 2020-06-15, anchored `Filename`, with the epoch present and visible in the header.
    //
    // So when the cascade's mapping is only as good as the filename, and the sidecar has an epoch,
    // the epoch wins. A payload-anchored tier is NOT overridden — it is stronger than both.
    const bool cascadeIsFilenameGrade = (anchor.tier == AnchorTier::FilenameAnchor
                                      || anchor.tier == AnchorTier::None);
    const auto& segHdr = seg.header();
    const bool haveCaptureEpoch =
        (segHdr.flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH) != 0
        && segHdr.capture_epoch_ms >= kUnixPlausibleFloorMs;

    if (!towDomain && !logHasFix && cascadeIsFilenameGrade && haveCaptureEpoch) {
        // The raw value is an uptime; the epoch is the log-open wall clock. Offsetting the uptime
        // from the segment's own uptime floor keeps the records in order and rooted at the epoch.
        const uint64_t within = (anchor.uptimeMinMs != 0 && out.sidecarRawMs >= anchor.uptimeMinMs)
                              ? out.sidecarRawMs - anchor.uptimeMinMs
                              : out.sidecarRawMs;
        out.absoluteMs   = segHdr.capture_epoch_ms + within;
        out.valid        = true;
        out.mechanism    = AbsTimeMechanism::UptimeProjected;
        out.towMs        = out.absoluteMs % 604800000ULL;
        out.gpsWeek      = weekOfUnixMs(out.absoluteMs);
        out.anchorSource = AbsAnchorSource::IdxCaptureEpoch;
        out.hints.emplace_back("anchored from the .idx capture epoch (host wall-clock at log-open), "
                               "which outranks the filename; this log never achieved a GNSS fix");
    } else if (!towDomain && static_cast<uint64_t>(mapped) >= kUnixPlausibleFloorMs) {
        // The offset already targets Unix. Applying a week on top would double-count the epoch —
        // which is precisely the 46.68-year error this whole exercise started from.
        out.absoluteMs = static_cast<uint64_t>(mapped);
        out.valid      = true;
        out.mechanism  = AbsTimeMechanism::UptimeProjected;
        out.towMs      = out.absoluteMs % 604800000ULL;
        out.gpsWeek    = weekOfUnixMs(out.absoluteMs);
        out.anchorSource = AbsAnchorSource::Filename;   // refined below if a better one applies
        out.hints.emplace_back("segment offset maps directly to absolute time; no week applied");
    } else {
        // ---- Step 3: the value is in the GPS time-of-week frame, so it needs a week. This is the
        // step nothing else in the codebase performs, and why 661 of 709 segments report a span in
        // the ToW frame.
        out.towMs     = static_cast<uint64_t>(mapped);
        out.mechanism = towDomain ? AbsTimeMechanism::PayloadEpoch
                                  : (anchor.tier == AnchorTier::PayloadToWBridge ||
                                     anchor.tier == AnchorTier::PayloadToWSingle)
                                        ? AbsTimeMechanism::SyncMatched
                                        : AbsTimeMechanism::UptimeProjected;

        uint32_t week            = 0;
        bool     weekFromPayload = false;
        if (anchorWeek_ >= kMinPlausibleWeek) {
            week             = anchorWeek_;
            weekFromPayload  = true;
            out.anchorSource = AbsAnchorSource::PayloadWeek;
        } else {
            if (anchorWeek_ != 0) {
                out.hints.emplace_back("payload week " + std::to_string(anchorWeek_) +
                                       " is implausible (< " + std::to_string(kMinPlausibleWeek) +
                                       "); it was not used");
            }
            // No usable week, so an EXTERNAL anchor is required. Kyle's order, 2026-10-02: the
            // host-recorded timestamp in the `.idx` first, the filename only as a worst case.
            // The `.idx` one is a measurement the host actually took at log-open; the filename is
            // a string that merely looks like a date.
            uint64_t anchorUnixMs = 0;
            const auto& hdr = seg.header();
            if ((hdr.flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH) != 0
                && hdr.capture_epoch_ms >= kUnixPlausibleFloorMs) {
                anchorUnixMs     = hdr.capture_epoch_ms;
                out.anchorSource = AbsAnchorSource::IdxCaptureEpoch;
                out.hints.emplace_back("anchored from the .idx capture epoch (host wall-clock at "
                                       "log-open); this log never achieved a GNSS fix");
            } else if (haveFileAnchor_ && fileAnchorMs_ >= kUnixPlausibleFloorMs) {
                anchorUnixMs     = fileAnchorMs_;
                out.anchorSource = AbsAnchorSource::Filename;
                out.hints.emplace_back("anchored from the log FILENAME - no GNSS fix and no "
                                       "capture epoch in the .idx; the weakest anchor available");
            } else if (anchor.anchoredStartMs >= kUnixPlausibleFloorMs) {
                anchorUnixMs     = anchor.anchoredStartMs;
                out.anchorSource = AbsAnchorSource::Filename;
                out.hints.emplace_back("anchored from the segment cascade's absolute start");
            }
            if (anchorUnixMs == 0) {
                // EXPECTED outcome, not a failure: this log has no clock source at all. Fall
                // through to a relative-only answer below rather than returning nothing.
                out.hints.emplace_back("no GNSS fix, no .idx capture epoch and no filename "
                                       "timestamp; this log cannot be placed on any clock");
                return relativeOnlyResult(out, anchor, out.sidecarRawMs,
                                          segmentRawFloorMs(segmentIndex));
            }
            week          = weekOfUnixMs(anchorUnixMs);
            out.mechanism = AbsTimeMechanism::FileAnchor;
            out.hints.emplace_back("week " + std::to_string(week) +
                                   " derived from the log's anchor, not from any payload");
        }
        out.gpsWeek         = week;
        out.weekFromPayload = weekFromPayload;
        out.absoluteMs      = gpsToUnixMs(week, out.towMs);
        out.valid           = out.absoluteMs >= kUnixPlausibleFloorMs;
        if (!out.valid) {
            out.hints.emplace_back("composed time is not a plausible capture date");
            return relativeOnlyResult(out, anchor, out.sidecarRawMs,
                                      segmentRawFloorMs(segmentIndex));
        }
    }

    // ---- Step 4: provenance tags, and relative time as a SUBTRACTION rather than a mechanism.
    out.source = (out.mechanism == AbsTimeMechanism::PayloadEpoch) ? TimeSource::PayloadToW
                                                                   : TimeSource::ResolvedViaSync;
    // Certainty follows the WEAKEST link in the chain, never the strongest. A record whose week
    // had to be inferred from a filename is not Exact however exact its time of week was — that
    // conflation is what produced a full certainty dial beside a 1980 date (review B102).
    out.confidence = (out.mechanism == AbsTimeMechanism::PayloadEpoch && out.weekFromPayload)
                         ? TimeConfidence::Exact
                     : (out.mechanism == AbsTimeMechanism::FileAnchor)
                         ? TimeConfidence::ExtrapolatedForward
                         : TimeConfidence::Interpolated;

    return out;
}

AbsTimeResult ISTimeResolver::resolveAbsTime(const ISDeviceLog& log,
                                             std::size_t segmentIndex,
                                             std::size_t recordIndex) const {
    AbsTimeResult out = resolveAbsTimeCore(log, segmentIndex, recordIndex);

    // A record whose own time field is dead is placed from its NEIGHBOURS instead. Kyle's
    // principle, 2026-10-01: "bytes that arrive in a stream after other bytes MUST, BY
    // DEFINITION, come LATER in TIME" — so arrival order bounds this record between the last live
    // record before it and the first live one after, and anything inside that interval is
    // defensible. Interpolating by position within the interval is the obvious choice inside it.
    //
    // Cheap in practice: on the stuck-GPX log the dead records are 1,539 of 193,218, so a live
    // neighbour is a record or two away. The neighbours cannot themselves be frozen-interpolated,
    // so this cannot recurse.
    if (out.frozenField) {
        const AbsTimeResult before = lastLiveBefore(log, segmentIndex, recordIndex);
        const AbsTimeResult after  = firstLiveAfter(log, segmentIndex, recordIndex);
        if (before.valid && after.valid && after.absoluteMs >= before.absoluteMs) {
            // Midpoint of the bracket. A position-weighted split would need the record counts
            // between the three points and buys nothing a reader would notice at these gaps.
            out.absoluteMs = before.absoluteMs + (after.absoluteMs - before.absoluteMs) / 2;
            out.valid      = true;
            out.mechanism  = AbsTimeMechanism::InterpolatedFromNeighbours;
            out.source     = TimeSource::ResolvedViaSync;
            out.confidence = TimeConfidence::Interpolated;
            out.anchorSource = before.anchorSource;
            out.segmentIndex = segmentIndex;
            out.hints.emplace_back("placed between the live records either side of it in arrival "
                                   "order: " + std::to_string(before.absoluteMs) + " .. " +
                                   std::to_string(after.absoluteMs) + " ms");
        } else if (before.valid) {
            // At the tail with nothing live after it: the lower bound is still a real constraint.
            out.absoluteMs   = before.absoluteMs;
            out.valid        = true;
            out.mechanism    = AbsTimeMechanism::InterpolatedFromNeighbours;
            out.source       = TimeSource::ResolvedViaSync;
            out.confidence   = TimeConfidence::ExtrapolatedForward;
            out.anchorSource = before.anchorSource;
            out.segmentIndex = segmentIndex;
            out.hints.emplace_back("no live record after it; pinned to the last live one before");
        } else {
            out.mechanism = AbsTimeMechanism::FrozenAndUnbounded;
            out.hints.emplace_back("no live record either side; this record cannot be placed");
            return out;
        }
    }
    if (!out.valid) return out;

    // Relative time is a SUBTRACTION from an origin, never its own derivation — the whole point of
    // SN-8784. The origins come from `resolveAbsTimeCore`, so they cannot be on a different frame
    // from the value being subtracted from them.
    //
    // Split from the core for two reasons: the core must not recurse (the origin is itself found
    // by resolving records), and the origins are memoised per log, without which placing one
    // record would cost a scan of the whole log.
    ensureOrigins(log);
    if (originsLog_ == &log) {
        if (logOriginMs_ != 0 && out.absoluteMs >= logOriginMs_) {
            out.relativeToLogMs = out.absoluteMs - logOriginMs_;
        }
        if (out.segmentIndex < segmentOriginMs_.size()) {
            const uint64_t segOrigin = segmentOriginMs_[out.segmentIndex];
            if (segOrigin != 0 && out.absoluteMs >= segOrigin) {
                out.relativeToSegmentMs = out.absoluteMs - segOrigin;
            }
        }
    }
    return out;
}

uint64_t ISTimeResolver::segmentRawFloorMs(std::size_t segmentIndex) const {
    return (segmentIndex < segmentRawFloorMs_.size()) ? segmentRawFloorMs_[segmentIndex] : 0;
}

bool ISTimeResolver::didFieldIsFrozen(uint32_t did) const {
    return frozenDids_.count(did) != 0;
}

void ISTimeResolver::ensureOrigins(const ISDeviceLog& log) const {
    if (originsLog_ == &log) return;
    originsLog_  = &log;
    logOriginMs_ = 0;
    segmentOriginMs_.assign(log.segmentCount(), 0);
    segmentRawFloorMs_.assign(log.segmentCount(), 0);
    frozenDids_.clear();

    // ---- Stuck-field detection, by VARIANCE rather than range.
    //
    // A DID whose sidecar timestamp is identical on every record it appears in carries no time
    // information at all, whatever the magnitude looks like. On the known stuck-GPX log
    // `20260916_232611` every one of 1,539 `DID_GPX_*` records reports 342,615,500 while the IMX
    // records beside them advance over 699.6 s.
    //
    // Range would be the WRONG test and would reject valid data: if one device reboots and the
    // other does not, the second device's uptime also sits far outside its sibling's range and is
    // entirely real (Kyle, 2026-10-02). Variance separates a dead clock from an offset one.
    {
        struct Extent { uint64_t lo = 0, hi = 0; std::size_t n = 0; };
        std::map<uint32_t, Extent> byDid;
        for (std::size_t i = 0; i < log.segmentCount(); ++i) {
            const ISLogReader& seg = log.segment(i);
            const std::size_t  n   = seg.recordCount();
            for (std::size_t k = 0; k < n; ++k) {
                const ISRecordView rv  = seg.recordAt(k);
                const uint64_t     raw = rv.timestamp().value;
                if (raw == 0) continue;
                // Same pass: the segment's raw floor, the last-resort relative origin.
                uint64_t& floor = segmentRawFloorMs_[i];
                if (floor == 0 || raw < floor) floor = raw;
                Extent& e = byDid[rv.did()];
                if (e.n == 0) { e.lo = e.hi = raw; }
                else { if (raw < e.lo) e.lo = raw; if (raw > e.hi) e.hi = raw; }
                ++e.n;
            }
        }
        // A handful of identical values proves nothing — a 2 Hz DID in a short log can legitimately
        // repeat. The floor keeps the detector off low-rate DIDs where it would be guessing.
        constexpr std::size_t kMinRecordsToCallItFrozen = 32;
        for (const auto& [did, e] : byDid) {
            if (e.n >= kMinRecordsToCallItFrozen && e.lo == e.hi) frozenDids_.insert(did);
        }
    }

    // One forward pass per segment, stopping at that segment's first placeable record. The log
    // origin is the first segment's that yields anything, scanning forward — NOT a minimum over
    // every record, because a minimum lets a single mis-framed record redefine the origin
    // (`anchoredSpanStart` does exactly that, and is dragged into 1970 on 6 of 13 corpus logs).
    for (std::size_t i = 0; i < log.segmentCount(); ++i) {
        const ISLogReader& seg = log.segment(i);
        const std::size_t  n   = seg.recordCount();
        for (std::size_t k = 0; k < n; ++k) {
            const AbsTimeResult r = resolveAbsTimeCore(log, i, k);
            if (!r.valid) continue;
            segmentOriginMs_[i] = r.absoluteMs;
            if (logOriginMs_ == 0) logOriginMs_ = r.absoluteMs;
            break;
        }
    }
}

AbsTimeResult ISTimeResolver::lastLiveBefore(const ISDeviceLog& log,
                                             std::size_t segmentIndex,
                                             std::size_t recordIndex) const {
    // Walks back in arrival order: within the segment, then into the previous ones. Bounded by
    // `kNeighbourSearchLimit` so a log that is mostly dead records cannot turn one lookup into a
    // full scan.
    constexpr std::size_t kNeighbourSearchLimit = 4096;
    std::size_t searched = 0;
    for (std::size_t s = segmentIndex + 1; s-- > 0;) {
        std::size_t k = (s == segmentIndex) ? recordIndex : log.segment(s).recordCount();
        while (k-- > 0) {
            if (++searched > kNeighbourSearchLimit) return {};
            const AbsTimeResult r = resolveAbsTimeCore(log, s, k);
            if (r.valid) return r;
        }
    }
    return {};
}

AbsTimeResult ISTimeResolver::firstLiveAfter(const ISDeviceLog& log,
                                             std::size_t segmentIndex,
                                             std::size_t recordIndex) const {
    constexpr std::size_t kNeighbourSearchLimit = 4096;
    std::size_t searched = 0;
    for (std::size_t s = segmentIndex; s < log.segmentCount(); ++s) {
        const std::size_t start = (s == segmentIndex) ? recordIndex + 1 : 0;
        const std::size_t n     = log.segment(s).recordCount();
        for (std::size_t k = start; k < n; ++k) {
            if (++searched > kNeighbourSearchLimit) return {};
            const AbsTimeResult r = resolveAbsTimeCore(log, s, k);
            if (r.valid) return r;
        }
    }
    return {};
}

AbsTimeResult ISTimeResolver::firstResolvableIn(const ISDeviceLog& log,
                                                std::size_t fromSegment) const {
    // Scans forward from `fromSegment` for the first record this function can place. Forward, not
    // a minimum over everything: the origin of relative time has to be the FIRST record, and a
    // minimum lets one mis-framed record anywhere in the log redefine the origin.
    for (std::size_t i = fromSegment; i < log.segmentCount(); ++i) {
        const ISLogReader& seg = log.segment(i);
        const std::size_t  n   = seg.recordCount();
        for (std::size_t k = 0; k < n; ++k) {
            const AbsTimeResult r = resolveAbsTimeCore(log, i, k);
            if (r.valid) return r;
        }
        if (fromSegment != 0) break;   // segment-scoped request: do not spill into the next one
    }
    return {};
}

void ISTimeResolver::ensureAbsIndex(const ISDeviceLog& log) const {
    if (absIndex_.log == &log) return;
    absIndex_ = AbsIndexCache{};                 // the reset is the cache's own copy assignment
    absIndex_.log = &log;

    // Deliberately built from `resolveAbsTimeCore`, never from the raw `.idx` values: the index is
    // the inverse direction's only input, so reading anything else would let the two directions
    // drift apart, and the round trip being provable is the objective. It also cannot reuse the
    // `.idx` order — that is arrival-ordered and mixed-domain, which is exactly why
    // `ISLogReader::seek()` is raw-domain-only and returns `end()` for a resolved absolute.
    std::size_t total = 0;
    for (std::size_t i = 0; i < log.segmentCount(); ++i) total += log.segment(i).recordCount();
    absIndex_.entries.reserve(total);

    for (std::size_t i = 0; i < log.segmentCount(); ++i) {
        const std::size_t n = log.segment(i).recordCount();
        for (std::size_t k = 0; k < n; ++k) {
            const AbsTimeResult r = resolveAbsTimeCore(log, i, k);
            if (!r.valid) continue;
            absIndex_.entries.push_back({ r.absoluteMs,
                                          static_cast<uint32_t>(i),
                                          static_cast<uint32_t>(k) });
        }
    }

    // Sorting on the indices as well as the instant is load-bearing, not tidiness: a stalled clock
    // parks many records on one instant, and the tie-break is what makes "the first of them" a
    // single stable answer. Without it the lookup could land anywhere in the run and the second
    // round trip would walk along it instead of being a fixed point.
    std::sort(absIndex_.entries.begin(), absIndex_.entries.end(),
              [](const AbsIndexEntry& a, const AbsIndexEntry& b) {
                  if (a.absoluteMs   != b.absoluteMs)   return a.absoluteMs   < b.absoluteMs;
                  if (a.segmentIndex != b.segmentIndex) return a.segmentIndex < b.segmentIndex;
                  return a.recordIndex < b.recordIndex;
              });
}

SegmentOffset ISTimeResolver::resolveTimeToSegmentOffset(const ISDeviceLog& log,
                                                         uint64_t absoluteMs) const {
    SegmentOffset out;

    ensureAbsIndex(log);
    const std::vector<AbsIndexEntry>& idx = absIndex_.entries;
    if (idx.empty()) return out;                 // nothing in the log can be placed

    // The last entry at or before the target. `upper_bound` then stepping back is the at-or-before
    // query; on an exact hit it lands past the whole equal run, so the step back gives its LAST
    // element and the walk below gives its first.
    auto it = std::upper_bound(idx.begin(), idx.end(), absoluteMs,
                               [](uint64_t t, const AbsIndexEntry& e) { return t < e.absoluteMs; });

    // Earlier than every record: clamp to the EARLIEST one rather than failing, and say so, because
    // a marker before the log start is an ordinary UI state. `idx.front()` is that record — the
    // sort's index tie-break makes it the earliest-arrival of the minimum instant.
    //
    // This used to clamp via the first placeable record in ARRIVAL order, which is a different
    // record whenever arrival order is not monotonic in resolved time. Measured over the corpus
    // 2026-10-03: the two differ on 60 of 119 device-logs, and on all 60 the clamp returned a
    // record that was NOT the earliest — on `goldenlogs/.../20260729_003722` it landed 3.27 days
    // after the log's earliest record.
    const bool before = (it == idx.begin());

    const auto lastOfRun = before ? idx.begin() : it - 1;
    const uint64_t bestMs = lastOfRun->absoluteMs;

    // Walk the equal run both ways. Backwards finds the record to RETURN; forwards is only needed
    // for the count, and only ever moves in the `before` clamp, where `lastOfRun` is the run's
    // first rather than its last. `upper_bound` guarantees it is the last in every other case.
    auto firstOfRun = lastOfRun;
    while (firstOfRun != idx.begin() && (firstOfRun - 1)->absoluteMs == bestMs) --firstOfRun;
    auto endOfRun = lastOfRun + 1;
    while (endOfRun != idx.end() && endOfRun->absoluteMs == bestMs) ++endOfRun;
    const std::size_t runLength = static_cast<std::size_t>(endOfRun - firstOfRun);

    const std::size_t  segIdx = firstOfRun->segmentIndex;
    const std::size_t  recIdx = firstOfRun->recordIndex;
    const ISLogReader& seg    = log.segment(segIdx);

    uint64_t base = 0;
    for (std::size_t i = 0; i < segIdx; ++i) base += static_cast<uint64_t>(log.segment(i).recordCount());

    out.valid        = true;
    out.segmentIndex = segIdx;
    out.recordIndex  = recIdx;
    out.segmentPath  = seg.path();
    out.segment      = &seg;
    out.byteOffset   = seg.recordAt(recIdx).offsetInFile();
    out.arrivalIndex = base + static_cast<uint64_t>(recIdx);
    out.runLength    = runLength;

    // One question, mutually exclusive answers: how does the target relate to the instant that came
    // back? Whether that instant is SHARED is `runLength`, reported independently — a target can be
    // inexact and land on a stalled run at the same time.
    //
    // Three things here were wrong before the index, all measured rather than reasoned about:
    //
    //  - `Preceding` did not exist, so a target falling between two records was reported `Exact`.
    //    Every midpoint seed in the corpus cycle test takes this branch.
    //  - `FirstOfStalledRun` was unreachable. The old code tested whether a backward walk had
    //    MOVED, but its forward scan already kept the earliest-arrival record of the equal run, so
    //    the walk was always a no-op. A fixture of 5 instants x 6 records reported `Exact` five
    //    times on the pre-index binary.
    //  - `After` was tested against the last placeable record in ARRIVAL order, so on a log whose
    //    arrival order is not monotonic in resolved time a target INSIDE the span was reported as
    //    past the end of it. 19 of 119 corpus device-logs are non-monotonic and 8 mislabelled such
    //    a target; it is now tested against `idx.back()`, the LATEST instant.
    out.exactness    = before                               ? PositionExactness::Before
                     : (absoluteMs > idx.back().absoluteMs) ? PositionExactness::After
                     : (bestMs != absoluteMs)               ? PositionExactness::Preceding
                     : (runLength > 1)                      ? PositionExactness::FirstOfStalledRun
                                                            : PositionExactness::Exact;
    return out;
}

} // namespace inertial_sense
