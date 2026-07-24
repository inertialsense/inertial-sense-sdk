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
#include "data_sets.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>

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
void scanSegmentForSyncs(const ISLogReader& reader,
                         uint64_t deviceId,
                         std::vector<ISSyncPoint>& out,
                         std::vector<int64_t>& upOffsetsOut) {
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

    for (std::size_t i = 0; i < bytes.second; ++i) {
        protocol_type_t p = is_comm_parse_byte(&comm, bytes.first[i]);
        if (p != _PTYPE_INERTIAL_SENSE_DATA && p != _PTYPE_INERTIAL_SENSE_CMD) {
            continue;
        }
        const auto& hdr = comm.rxPkt.dataHdr;

        // SN-8323 (uptime unification): DID_SYS_PARAMS carries BOTH the GPS
        // time-of-week (timeOfWeekMs) and the definitive system uptime (upTime,
        // seconds since boot). When the device is synced (timeOfWeekMs > 0),
        // one such record yields the authoritative uptime->ToW offset that
        // bridges every session-uptime value (magnetometer/imu records, and
        // pre-sync "real clock" records whose week/ToW default to uptime until
        // GPS lock). This replaces the fragile per-sync actualHostTimeMs
        // heuristic. (Kyle 2026-07-23: SYS_PARAMS.upTime is the definitive
        // relative uptime; GNSS/INS are the accurate absolute clock.)
        if (hdr.id == DID_SYS_PARAMS && comm.rxPkt.data.ptr &&
            hdr.offset == 0 && hdr.size >= sizeof(sys_params_t)) {
            sys_params_t sp2{};
            std::memcpy(&sp2, comm.rxPkt.data.ptr, sizeof(sp2));
            // Only trust timeOfWeekMs as GPS ToW when the device says so:
            // HDW_STATUS_GNSS_TIME_OF_WEEK_VALID. Otherwise timeOfWeekMs is
            // LOCAL system time (uptime-like), and differencing it against
            // upTime yields a bogus offset that corrupts the median bridge.
            const bool towValid =
                (sp2.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
            if (towValid && sp2.timeOfWeekMs > 0 && sp2.upTime > 0.0) {
                const int64_t upMs = static_cast<int64_t>(sp2.upTime * 1000.0);
                upOffsetsOut.push_back(
                    static_cast<int64_t>(sp2.timeOfWeekMs) - upMs);
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

} // namespace

// ============================================================
// Detection
// ============================================================

std::vector<ISSyncPoint> ISTimeResolver::detectSyncPoints(const ISDeviceLog& log) {
    std::vector<int64_t> upOffsets;  // discarded here; build() consumes them.
    return detectSyncPointsImpl(log, upOffsets);
}

std::vector<ISSyncPoint> ISTimeResolver::detectSyncPointsImpl(
    const ISDeviceLog& log, std::vector<int64_t>& upOffsetsOut) {
    std::vector<ISSyncPoint> out;
    const uint64_t deviceId = log.deviceId();

    for (std::size_t s = 0; s < log.segmentCount(); ++s) {
        scanSegmentForSyncs(log.segment(s), deviceId, out, upOffsetsOut);
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
    auto syncs = detectSyncPointsImpl(log, upOffsets);

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

    return ISTimeResolver{ std::move(syncs), std::move(discs), anchorWeek,
                           anchorTowStart, anchorTowEnd,
                           uptimeToTowOffsetMs, haveUptimeOffset };
}

// ============================================================
// Resolve
// ============================================================

TimeStamp ISTimeResolver::resolve(uint64_t hostTimeMs, uint64_t deviceId) const {
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
        // No anchors. Best we can do is a SessionOnly tag with the
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
    if (haveUptimeOffset_ && epochAnchor && anchorTowEnd_ >= anchorTowStart_) {
        const int64_t guard    = static_cast<int64_t>(kPreFixGuardMs);
        const int64_t lo       = static_cast<int64_t>(anchorTowStart_);
        const int64_t hi       = static_cast<int64_t>(anchorTowEnd_);
        const int64_t raw      = static_cast<int64_t>(hostTimeMs);
        const bool    rawIsTow = (raw >= lo - guard && raw <= hi + guard);
        if (!rawIsTow) {
            const int64_t bridged = raw + uptimeToTowOffsetMs_;
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

ISTimeResolver::Stats ISTimeResolver::computeStats(const ISDeviceLog& log) const {
    Stats s{};
    for (auto v : log.allRecords()) {
        const TimeStamp t = resolve(v.timestamp().value, log.deviceId());
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

} // namespace inertial_sense
