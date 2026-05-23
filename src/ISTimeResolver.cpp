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

namespace inertial_sense {

namespace {

/// DIDs whose payloads carry a usable GPS time-of-week field. Used by
/// `detectSyncPoints` as the candidate set when scanning a segment.
/// `cISDataMappings::Timestamp(...)` is consulted on each candidate to
/// extract the actual ToW (returns 0 if the field is absent / zero,
/// which the detection loop treats as "not a sync anchor").
constexpr std::array<uint32_t, 6> kToWBearingDids = {
    DID_INS_1, DID_INS_2, DID_INS_3, DID_INS_4,
    DID_GNSS1_POS, DID_GNSS2_POS,
};

inline bool isToWBearing(uint32_t did) noexcept {
    for (auto d : kToWBearingDids) if (d == did) return true;
    return false;
}

/// Slope between two sync points in ToW-ms per host-ms. Returns 1.0
/// when the host distance is zero (degenerate / colocated points) so
/// downstream comparisons don't divide-by-zero.
double slopeBetween(const ISSyncPoint& a, const ISSyncPoint& b) noexcept {
    if (b.hostTimeMs == a.hostTimeMs) return 1.0;
    const double hostDelta = static_cast<double>(b.hostTimeMs) - static_cast<double>(a.hostTimeMs);
    const double towDelta  = static_cast<double>(b.payloadToWMs) - static_cast<double>(a.payloadToWMs);
    return towDelta / hostDelta;
}

/// Walk one segment's `.raw` bytes via `is_comm_parse_byte`, emitting
/// a sync point for every ToW-bearing-DID packet whose payload carries
/// a non-zero ToW. The walk per segment matches the writer's per-
/// packet emission pattern (`cDeviceLogRaw::SaveData`), so we recover
/// the same anchors the writer would have flagged with `HAS_TOW`.
void scanSegmentForSyncs(const ISLogReader& reader,
                         uint64_t deviceId,
                         std::vector<ISSyncPoint>& out) {
    auto bytes = reader.rawBytes();
    if (!bytes.first || bytes.second == 0) return;

    is_comm_instance_t comm{};
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);

    for (std::size_t i = 0; i < bytes.second; ++i) {
        protocol_type_t p = is_comm_parse_byte(&comm, bytes.first[i]);
        if (p != _PTYPE_INERTIAL_SENSE_DATA && p != _PTYPE_INERTIAL_SENSE_CMD) {
            continue;
        }
        const auto& hdr = comm.rxPkt.dataHdr;
        if (!isToWBearing(hdr.id)) continue;

        const double tsSec = cISDataMappings::Timestamp(&hdr, comm.rxPkt.data.ptr);
        if (tsSec <= 0.0) continue;  // payload's ToW field is zero / absent.

        const uint64_t towMs = static_cast<uint64_t>(tsSec * 1000.0);
        ISSyncPoint sp{};
        // v2 `.idx` schema collapse: the writer's `rec.timestamp` for
        // HAS_TOW records is also the ToW (no separate host-time
        // recorded). Carry the same value in both fields so a future
        // v3 schema with a distinct host-time can populate `hostTimeMs`
        // independently without touching the resolver's API.
        sp.hostTimeMs   = towMs;
        sp.payloadToWMs = towMs;
        sp.deviceId     = deviceId;
        sp.sourceDid    = hdr.id;
        out.push_back(sp);
    }
}

} // namespace

// ============================================================
// Detection
// ============================================================

std::vector<ISSyncPoint> ISTimeResolver::detectSyncPoints(const ISDeviceLog& log) {
    std::vector<ISSyncPoint> out;
    const uint64_t deviceId = log.deviceId();

    for (std::size_t s = 0; s < log.segmentCount(); ++s) {
        scanSegmentForSyncs(log.segment(s), deviceId, out);
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
    auto syncs = detectSyncPoints(log);

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

    return ISTimeResolver{ std::move(syncs), std::move(discs) };
}

// ============================================================
// Resolve
// ============================================================

TimeStamp ISTimeResolver::resolve(uint64_t hostTimeMs, uint64_t deviceId) const {
    if (syncPoints_.empty()) {
        // No anchors. Best we can do is a SessionOnly tag with the
        // input value passed through.
        return TimeStamp::fromSessionOnly(hostTimeMs, deviceId);
    }

    // Binary search for the first sync point whose hostTimeMs >= input.
    auto it = std::lower_bound(
        syncPoints_.begin(), syncPoints_.end(), hostTimeMs,
        [](const ISSyncPoint& sp, uint64_t v) { return sp.hostTimeMs < v; });

    if (it != syncPoints_.end() && it->hostTimeMs == hostTimeMs) {
        // Exact match against a sync point.
        return TimeStamp::fromPayloadToW(it->payloadToWMs, deviceId);
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
        const uint64_t value = (tow < 0.0) ? 0u : static_cast<uint64_t>(tow);
        return TimeStamp::fromResolvedViaSync(value, deviceId,
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
        const uint64_t value = (tow < 0.0) ? 0u : static_cast<uint64_t>(tow);
        return TimeStamp::fromResolvedViaSync(value, deviceId,
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
    return TimeStamp::fromResolvedViaSync(static_cast<uint64_t>(tow),
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
