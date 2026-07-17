// D-06 / SN-7880 smoke test for the tagged TimeStamp type.
//
// Covers the AC scenarios:
//   - Factories produce the expected source/confidence pair.
//   - Ordering is by `value` only (cross-source / cross-confidence
//     ties go however ordering says, NOT by other fields).
//   - Equality is on all four fields.
//   - memcpy round-trip preserves the value (trivial-copy guarantee).
//   - Formatter emits the expected strings for each TimeFormat.

#include <gtest/gtest.h>

#include "ISTimeStamp.h"

#include <cstring>
#include <string>
#include <type_traits>

using inertial_sense::TimeConfidence;
using inertial_sense::TimeFormat;
using inertial_sense::TimeSource;
using inertial_sense::TimeStamp;
using inertial_sense::toString;

TEST(ISTimeStamp, FactoriesAssignSourceAndConfidence) {
    const auto a = TimeStamp::fromPayloadToW(1000, 42);
    EXPECT_EQ(a.source, TimeSource::PayloadToW);
    EXPECT_EQ(a.confidence, TimeConfidence::Exact);
    EXPECT_EQ(a.value, 1000u);
    EXPECT_EQ(a.deviceId, 42u);

    const auto b = TimeStamp::fromResolvedViaSync(2000, 99,
                                                  TimeConfidence::Interpolated);
    EXPECT_EQ(b.source, TimeSource::ResolvedViaSync);
    EXPECT_EQ(b.confidence, TimeConfidence::Interpolated);

    const auto c = TimeStamp::fromHostReceived(3000, 7);
    EXPECT_EQ(c.source, TimeSource::HostReceived);
    EXPECT_EQ(c.confidence, TimeConfidence::Exact);

    const auto d = TimeStamp::fromSessionOnly(4000, 0);  // deviceId=0 is valid
    EXPECT_EQ(d.source, TimeSource::SessionOnly);
    EXPECT_EQ(d.confidence, TimeConfidence::Unknown);
    EXPECT_EQ(d.deviceId, 0u);
}

TEST(ISTimeStamp, OrderingIsByValueOnly) {
    // Two timestamps from completely different sources at different
    // values: the smaller-value one should compare less, regardless
    // of source / confidence / device.
    const auto early = TimeStamp::fromHostReceived(100, 1);
    const auto late  = TimeStamp::fromPayloadToW(500, 2);

    EXPECT_LT(early, late);
    EXPECT_LE(early, late);
    EXPECT_GT(late, early);
    EXPECT_GE(late, early);
    EXPECT_FALSE(late < early);

    // Same value, different source/confidence/device — ordering ties.
    const auto t1 = TimeStamp::fromPayloadToW(1234, 1);
    const auto t2 = TimeStamp::fromHostReceived(1234, 99);
    EXPECT_FALSE(t1 < t2);
    EXPECT_FALSE(t2 < t1);
    EXPECT_LE(t1, t2);
    EXPECT_GE(t1, t2);
}

TEST(ISTimeStamp, EqualityIsAllFields) {
    const auto a = TimeStamp::fromPayloadToW(1234, 1);
    const auto b = TimeStamp::fromPayloadToW(1234, 1);
    const auto c = TimeStamp::fromPayloadToW(1234, 2);  // different deviceId
    const auto d = TimeStamp::fromHostReceived(1234, 1); // different source

    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
    EXPECT_NE(a, d);
}

TEST(ISTimeStamp, IsTriviallyCopyable) {
    // Static side already in the header; this gives a runtime memcpy
    // round-trip per the AC.
    static_assert(std::is_trivially_copyable_v<TimeStamp>);

    const auto src = TimeStamp::fromResolvedViaSync(
        9999999999ULL, 12345, TimeConfidence::ExtrapolatedForward);
    TimeStamp dst{};
    std::memcpy(&dst, &src, sizeof(TimeStamp));
    EXPECT_EQ(src, dst);
}

TEST(ISTimeStamp, FormatterSessionSeconds) {
    const auto ts = TimeStamp::fromSessionOnly(12345, 0);  // 12.345 s
    EXPECT_EQ(toString(ts, TimeFormat::SessionSeconds), std::string{"12.345 s"});

    const auto zero = TimeStamp::fromSessionOnly(0, 0);
    EXPECT_EQ(toString(zero, TimeFormat::SessionSeconds), std::string{"0.000 s"});
}

TEST(ISTimeStamp, FormatterGpsToW) {
    // Day 2 (Tuesday in GPS), 03:04:05.006 → ms =
    //   2 * 86_400_000 + 3 * 3_600_000 + 4 * 60_000 + 5 * 1000 + 6
    //   = 172_800_000 + 10_800_000 + 240_000 + 5_000 + 6
    //   = 183_845_006
    const auto ts = TimeStamp::fromPayloadToW(183'845'006ULL, 1);
    EXPECT_EQ(toString(ts, TimeFormat::GpsToW), std::string{"2 03:04:05.006"});
}

TEST(ISTimeStamp, FormatterUtcIso8601) {
    // 2026-04-28T20:45:23.123Z → seconds since epoch:
    //   `date -u -d '2026-04-28T20:45:23Z' +%s` = 1777409123
    //   ms = 1_777_409_123_123
    const auto ts = TimeStamp::fromHostReceived(1'777'409'123'123ULL, 0);
    EXPECT_EQ(toString(ts, TimeFormat::UtcIso8601),
              std::string{"2026-04-28T20:45:23.123Z"});
}

TEST(ISTimeStamp, FormatterHostReceivedMs) {
    const auto ts = TimeStamp::fromHostReceived(1'234'567ULL, 0);
    EXPECT_EQ(toString(ts, TimeFormat::HostReceivedMs),
              std::string{"1234567 ms"});
}
