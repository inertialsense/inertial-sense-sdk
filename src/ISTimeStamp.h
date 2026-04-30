/**
 * @file ISTimeStamp.h
 * @brief Tagged time-value type — pairs a millisecond instant with its
 *        provenance (source) and quality (confidence).
 *
 * D-06 / SN-7880 / D0024: every component that handles time in the SDK
 * (ISLogReader, ISTimeResolver, ISLogWriter, derivations) speaks
 * `TimeStamp` instead of naked `uint64_t`, so callers can't lose track
 * of where a time came from or how trustworthy it is.
 *
 * Pure C++17, freestanding (no Qt, no exceptions). The `.cpp`
 * formatter pulls in `<sstream>` / `<chrono>`; the header itself is
 * cheap to include.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include <cstdint>
#include <string>
#include <type_traits>

namespace inertial_sense {

/**
 * @brief Where a time value was sourced from.
 *
 * Choosing the right source matters for downstream consumers — a
 * `PayloadToW` time can be cross-referenced across devices, while a
 * `SessionOnly` time is meaningful only within the current capture.
 */
enum class TimeSource : uint8_t {
    PayloadToW,         ///< Device payload's time-of-week field; canonical when available.
    ResolvedViaSync,    ///< Reconstructed by `ISTimeResolver` (D-07) from sync edges.
    HostReceived,       ///< Host clock at the moment the packet arrived.
    SessionOnly,        ///< Sample index converted to ms; carries no real-world anchor.
};

/**
 * @brief How trustworthy the time value is.
 *
 * `Interpolated` and the two `Extrapolated*` variants signal that the
 * value was reconstructed; UI-side code may render those differently
 * (D-58 dashed segments, time-axis confidence markers).
 */
enum class TimeConfidence : uint8_t {
    Exact,                  ///< Direct read from device payload or host clock.
    Interpolated,           ///< Between two known anchors; resolver inferred.
    ExtrapolatedForward,    ///< Past the last known anchor in the forward direction.
    ExtrapolatedBackward,   ///< Before the first known anchor.
    Unknown,                ///< No basis for confidence judgement (e.g. SessionOnly).
};

/**
 * @brief Unit of `TimeStamp::value`. v1: milliseconds.
 *
 * Defined as a constant rather than a runtime field so the type stays
 * trivially-copyable and ABI-stable. If the SDK ever needs sub-ms
 * resolution, a separate `TimeStampMicros` type is added — we don't
 * grow a unit field on this one.
 */
inline constexpr uint64_t TIMESTAMP_UNIT_MS = 1;

/**
 * @brief A time value plus its provenance + confidence + source device.
 *
 * Trivially-copyable POD: safe to `memcpy`, store in arrays, pass by
 * value. Equality compares all four fields; ordering uses `value` only
 * (so two timestamps from different devices at the same instant are
 * equivalent under `<` even though they're not equal).
 *
 * `deviceId == 0` is valid and means "no associated device" — used for
 * global / synthetic events (cursor positions, derived-series anchors).
 */
struct TimeStamp {
    uint64_t       value;       ///< Milliseconds; interpretation depends on `source`.
    TimeSource     source;
    TimeConfidence confidence;
    uint64_t       deviceId;    ///< 0 = no associated device.

    // ------------ Factories (constexpr) ----------------------------------

    /// `PayloadToW` source. Confidence is always `Exact` — the device sent it directly.
    static constexpr TimeStamp fromPayloadToW(uint64_t ms, uint64_t deviceId) noexcept {
        return TimeStamp{ ms, TimeSource::PayloadToW, TimeConfidence::Exact, deviceId };
    }

    /// `ResolvedViaSync` source. Confidence is caller-specified — the
    /// resolver knows whether this value was interpolated or extrapolated.
    static constexpr TimeStamp fromResolvedViaSync(uint64_t ms,
                                                   uint64_t deviceId,
                                                   TimeConfidence confidence) noexcept {
        return TimeStamp{ ms, TimeSource::ResolvedViaSync, confidence, deviceId };
    }

    /// `HostReceived` source. Confidence is `Exact` — host wall-clock at
    /// packet arrival is exactly what was observed.
    static constexpr TimeStamp fromHostReceived(uint64_t ms, uint64_t deviceId) noexcept {
        return TimeStamp{ ms, TimeSource::HostReceived, TimeConfidence::Exact, deviceId };
    }

    /// `SessionOnly` source. Confidence is `Unknown` — the value is a
    /// sample index in time units, not anchored to any wall clock.
    static constexpr TimeStamp fromSessionOnly(uint64_t ms, uint64_t deviceId) noexcept {
        return TimeStamp{ ms, TimeSource::SessionOnly, TimeConfidence::Unknown, deviceId };
    }
};

// ----- Comparisons ----------------------------------------------------------
//
// AC: "Order by `value` only; source/confidence ignored for ordering.
//      Equality is on all four fields."
//
// Provided as the C++17 sextet (==, !=, <, <=, >, >=). C++20 callers
// (Logalyzer at C++20 per D0020) can introduce their own `<=>` via a
// `using` alias if they want the spaceship; this header stays C++17.

constexpr bool operator==(const TimeStamp& a, const TimeStamp& b) noexcept {
    return a.value == b.value
        && a.source == b.source
        && a.confidence == b.confidence
        && a.deviceId == b.deviceId;
}
constexpr bool operator!=(const TimeStamp& a, const TimeStamp& b) noexcept { return !(a == b); }
constexpr bool operator<(const TimeStamp& a, const TimeStamp& b) noexcept { return a.value < b.value; }
constexpr bool operator<=(const TimeStamp& a, const TimeStamp& b) noexcept { return a.value <= b.value; }
constexpr bool operator>(const TimeStamp& a, const TimeStamp& b) noexcept { return a.value > b.value; }
constexpr bool operator>=(const TimeStamp& a, const TimeStamp& b) noexcept { return a.value >= b.value; }

static_assert(std::is_trivially_copyable_v<TimeStamp>,
              "TimeStamp must stay trivially-copyable: ABI-portable, memcpy-safe, "
              "and cheap to pass through SDK boundaries.");

// ----- Formatter ------------------------------------------------------------

/**
 * @brief Output format selector for `toString`.
 *
 * The formatter doesn't infer format from `TimeSource`; the caller
 * picks. A `HostReceived` value can legitimately be formatted as
 * `UtcIso8601` (it's a host wall-clock), as `HostReceivedMs` (raw ms),
 * or whatever else is useful in context.
 */
enum class TimeFormat : uint8_t {
    SessionSeconds,     ///< "12.345 s"  (value treated as session-relative ms).
    GpsToW,             ///< "DDD HH:MM:SS.mmm" (value treated as GPS time-of-week ms; DDD = day-of-week).
    UtcIso8601,         ///< "2026-04-28T20:45:23.123Z" (value treated as Unix-epoch ms).
    HostReceivedMs,     ///< "1234567 ms" (raw ms, no calendar interpretation).
};

/**
 * @brief Render a `TimeStamp` for display.
 *
 * Output is ASCII-only; UI-side code is responsible for any locale
 * formatting (thousands separators, etc.). The returned string never
 * carries provenance metadata — that's the caller's job to render
 * separately if they want it (e.g. an icon next to the value).
 *
 * @param ts  The timestamp to format.
 * @param fmt The format selector.
 * @return    A short ASCII string suitable for log lines / UI cells.
 */
std::string toString(const TimeStamp& ts, TimeFormat fmt);

} // namespace inertial_sense
