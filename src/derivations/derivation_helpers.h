/**
 * @file derivation_helpers.h
 * @brief Internal helpers shared across built-in derivation evaluators.
 *
 * `ISRecordView::bytes()` exposes the chunk-input range a record was
 * parsed from — for production logs that range may contain several
 * packets sharing one offset. The clean way to extract a typed payload
 * is to re-parse the segment's raw byte buffer via
 * `is_comm_parse_byte`. `walkPayloads<T>()` does that once per
 * derivation evaluation.
 *
 * Not part of the SDK's public API surface.
 */

#pragma once

#include "DerivationRegistry.h"

#include "ISComm.h"
#include "ISDataMappings.h"
#include "ISLogReader.h"
#include "ISPose.h"
#include "ISEarth.h"
#include "data_sets.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace inertial_sense {
namespace derivations {
namespace detail {

inline constexpr double kStandardGravity = 9.80665;

/// Wraps a heading to either [-pi, pi] (default) or [0, 2pi].
inline double wrapHeading(double h, bool positiveOnly) {
    constexpr double kTwoPi = 2.0 * M_PI;
    h = std::fmod(h, kTwoPi);
    if (positiveOnly) {
        if (h < 0.0) h += kTwoPi;
        return h;
    }
    if (h >  M_PI) h -= kTwoPi;
    if (h < -M_PI) h += kTwoPi;
    return h;
}

/// Reads the `wrap` parameter (`"signed"` (default) or `"positive"`/
/// `"unsigned"`).
inline bool parseWrapPositive(const DerivationContext& ctx) {
    auto v = ctx.param("wrap");
    if (!v) return false;
    return *v == std::string_view{"positive"} || *v == std::string_view{"unsigned"};
}

/// Parses the `origin_lla` parameter as `lat,lon,alt` (lat/lon in
/// degrees, alt in metres). Returns nullopt on malformed input.
inline std::optional<std::array<double, 3>> parseLlaCsv(std::string_view s) {
    std::array<double, 3> out{};
    std::size_t prev = 0;
    int field = 0;
    while (field < 3) {
        std::size_t comma = s.find(',', prev);
        std::string_view tok = s.substr(prev, comma - prev);
        try {
            out[field++] = std::stod(std::string(tok));
        } catch (...) {
            return std::nullopt;
        }
        if (comma == std::string_view::npos) break;
        prev = comma + 1;
    }
    if (field != 3) return std::nullopt;
    return out;
}

/// Walks the segment's raw bytes via `is_comm_parse_byte`, invoking
/// `cb(timestamp, payload)` for every ISB packet whose DID equals
/// `did` and whose payload size equals `sizeof(T)`. Mismatched sizes
/// (incomplete packets, alternate format DIDs) are silently skipped.
///
/// This is the canonical way for derivations to consume payload
/// structs, working independently of how the index was built.
template <class T, class Fn>
void walkPayloads(const ISLogReader& reader, uint32_t did, Fn cb) {
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
        if (comm.rxPkt.dataHdr.id != did) continue;
        if (comm.rxPkt.dataHdr.size != sizeof(T)) continue;

        const double tsSec = cISDataMappings::TimestampOrCurrentTime(
            &comm.rxPkt.dataHdr, comm.rxPkt.data.ptr);
        const uint64_t tsMs = static_cast<uint64_t>(tsSec * 1000.0);
        TimeStamp ts{ tsMs, TimeSource::PayloadToW, TimeConfidence::Exact,
                      reader.deviceId() };

        T copy;
        std::memcpy(&copy, comm.rxPkt.data.ptr, sizeof(T));
        cb(ts, copy);
    }
}

/// Materializes one DID's payloads into a vector for multi-input
/// derivations that need to scan secondary inputs by timestamp.
template <class T>
std::vector<std::pair<TimeStamp, T>> collectPayloads(const ISLogReader& reader,
                                                     uint32_t did) {
    std::vector<std::pair<TimeStamp, T>> out;
    walkPayloads<T>(reader, did, [&](TimeStamp ts, const T& v) {
        out.emplace_back(ts, v);
    });
    return out;
}

/// Find the entry with timestamp closest to `t`. O(N); D-07's
/// timestamp-monotonic outputs will let callers binary-search later.
/// Returns `nullptr` if `samples` is empty.
template <class T>
const T* closestByTimestamp(const std::vector<std::pair<TimeStamp, T>>& samples,
                            TimeStamp t) {
    if (samples.empty()) return nullptr;
    const T* best = &samples.front().second;
    uint64_t bestDelta = (samples.front().first.value > t.value)
                       ? (samples.front().first.value - t.value)
                       : (t.value - samples.front().first.value);
    for (const auto& [ts, val] : samples) {
        const uint64_t d = (ts.value > t.value)
                         ? (ts.value - t.value)
                         : (t.value - ts.value);
        if (d < bestDelta) { bestDelta = d; best = &val; }
    }
    return best;
}

/// Convenience: builds an `ISError` for "required input not found".
inline tl::unexpected<ISError> missingInput(uint32_t did, std::string_view field) {
    std::string m = "derivation: required input DID=" + std::to_string(did)
                  + " field='" + std::string(field) + "' not present in segment";
    return fail(ISErrorCode::NotFound, std::move(m));
}

} // namespace detail
} // namespace derivations
} // namespace inertial_sense
