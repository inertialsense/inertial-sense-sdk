/**
 * @file ISTimeStamp.cpp
 * @brief `toString` formatter for the tagged `TimeStamp` type.
 *
 * Pure C++17, no Qt, no exceptions on the hot path. UTC formatting
 * uses POSIX `gmtime_r` because <chrono> calendar formatting is C++20.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISTimeStamp.h"

#include <array>
#include <cstdio>
#include <ctime>
#include <string>

namespace inertial_sense {

namespace {

// One ms day = 86_400_000. One ms week = 7 * 86_400_000.
constexpr uint64_t kMsPerSecond = 1000ULL;
constexpr uint64_t kMsPerMinute = 60ULL * kMsPerSecond;
constexpr uint64_t kMsPerHour   = 60ULL * kMsPerMinute;
constexpr uint64_t kMsPerDay    = 24ULL * kMsPerHour;

// SessionSeconds: "12.345 s". Stable to ~9 chars + decimals.
std::string fmtSessionSeconds(uint64_t ms) {
    std::array<char, 64> buf{};
    const uint64_t whole = ms / kMsPerSecond;
    const uint64_t milli = ms % kMsPerSecond;
    std::snprintf(buf.data(), buf.size(), "%llu.%03llu s",
                  static_cast<unsigned long long>(whole),
                  static_cast<unsigned long long>(milli));
    return std::string(buf.data());
}

// GpsToW: value as ms within the GPS week. "DDD HH:MM:SS.mmm" where
// DDD is day-of-week (0=Sunday, GPS convention).
std::string fmtGpsToW(uint64_t ms) {
    const uint64_t day  = ms / kMsPerDay;
    const uint64_t rem1 = ms % kMsPerDay;
    const uint64_t hour = rem1 / kMsPerHour;
    const uint64_t rem2 = rem1 % kMsPerHour;
    const uint64_t min  = rem2 / kMsPerMinute;
    const uint64_t rem3 = rem2 % kMsPerMinute;
    const uint64_t sec  = rem3 / kMsPerSecond;
    const uint64_t milli = rem3 % kMsPerSecond;
    std::array<char, 64> buf{};
    std::snprintf(buf.data(), buf.size(), "%llu %02llu:%02llu:%02llu.%03llu",
                  static_cast<unsigned long long>(day),
                  static_cast<unsigned long long>(hour),
                  static_cast<unsigned long long>(min),
                  static_cast<unsigned long long>(sec),
                  static_cast<unsigned long long>(milli));
    return std::string(buf.data());
}

// UtcIso8601: value as Unix epoch ms. "YYYY-MM-DDTHH:MM:SS.mmmZ".
std::string fmtUtcIso8601(uint64_t ms) {
    const std::time_t whole = static_cast<std::time_t>(ms / kMsPerSecond);
    const uint64_t milli = ms % kMsPerSecond;
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &whole);
#else
    gmtime_r(&whole, &tm);
#endif
    std::array<char, 64> buf{};
    // YYYY-MM-DDTHH:MM:SS = 19 chars + ".mmmZ" = 5 chars + null = 25 chars.
    std::snprintf(buf.data(), buf.size(),
                  "%04d-%02d-%02dT%02d:%02d:%02d.%03lluZ",
                  tm.tm_year + 1900,
                  tm.tm_mon + 1,
                  tm.tm_mday,
                  tm.tm_hour,
                  tm.tm_min,
                  tm.tm_sec,
                  static_cast<unsigned long long>(milli));
    return std::string(buf.data());
}

// HostReceivedMs: just the raw value with a unit suffix.
std::string fmtHostReceivedMs(uint64_t ms) {
    std::array<char, 32> buf{};
    std::snprintf(buf.data(), buf.size(), "%llu ms",
                  static_cast<unsigned long long>(ms));
    return std::string(buf.data());
}

} // namespace

std::string toString(const TimeStamp& ts, TimeFormat fmt) {
    switch (fmt) {
        case TimeFormat::SessionSeconds:  return fmtSessionSeconds(ts.value);
        case TimeFormat::GpsToW:          return fmtGpsToW(ts.value);
        case TimeFormat::UtcIso8601:      return fmtUtcIso8601(ts.value);
        case TimeFormat::HostReceivedMs:  return fmtHostReceivedMs(ts.value);
    }
    // Unreachable in well-formed callers; defend against future enum
    // additions that forget to extend the switch.
    return std::string{"(unknown TimeFormat)"};
}

} // namespace inertial_sense
