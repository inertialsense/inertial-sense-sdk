// SN-8308: unit tests for MessageStats, covering the counting/dt
// bookkeeping shared by EvalTool's Log Summary and cltool's stream
// stats display. Exercises MessageStats::append() directly against
// non-ISB protocol types (NMEA/RTCM3/ACK/parse-error) so the tests
// don't depend on cISDataMappings being initialized -- only the ISB
// path (DID lookups) needs that, and it's not what's under test here.

#include <gtest/gtest.h>

#include "message_stats.h"
#include "ISComm.h"

TEST(MessageStats, InitialStateIsEmpty) {
    MessageStats::mul_stats_t stats;
    EXPECT_TRUE(stats.isb.empty());
    EXPECT_TRUE(stats.nmea.empty());
    EXPECT_TRUE(stats.ublox.empty());
    EXPECT_TRUE(stats.rtcm3.empty());
    EXPECT_EQ(stats.ack.count, 0);
    EXPECT_EQ(stats.parseError.count, 0);
}

TEST(MessageStats, FirstAppendCreatesEntryWithCountOne) {
    MessageStats::mul_stats_t stats;
    MessageStats::append("", stats, _PTYPE_NMEA, /*id=*/1234, /*bytes=*/50, /*timeMs=*/1000);

    ASSERT_EQ(stats.nmea.count(1234), 1u);
    const MessageStats::stats_t &s = stats.nmea.at(1234);
    EXPECT_EQ(s.count, 1);
    EXPECT_EQ(s.timeMs, 1000u);
    EXPECT_EQ(s.prevTimeMs, 1000u);   // first sample: prev == current, so dt reads as 0
    EXPECT_EQ(s.bytes, 50u);
}

TEST(MessageStats, RepeatedAppendsIncrementCountAndTrackDt) {
    MessageStats::mul_stats_t stats;
    MessageStats::append("", stats, _PTYPE_NMEA, 1234, 50, 1000);
    MessageStats::append("", stats, _PTYPE_NMEA, 1234, 50, 1500);
    MessageStats::append("", stats, _PTYPE_NMEA, 1234, 50, 2100);

    const MessageStats::stats_t &s = stats.nmea.at(1234);
    EXPECT_EQ(s.count, 3);
    // dt is derived by callers (e.g. MessageStats::summary()) as timeMs - prevTimeMs.
    EXPECT_EQ(s.timeMs, 2100u);
    EXPECT_EQ(s.prevTimeMs, 1500u);
    EXPECT_EQ(s.timeMs - s.prevTimeMs, 600u);
}

TEST(MessageStats, BytesPerSecondResetsOnceWindowElapses) {
    MessageStats::mul_stats_t stats;
    MessageStats::append("", stats, _PTYPE_NMEA, 1234, 50, 1000);   // startTimeMs := 1000
    MessageStats::append("", stats, _PTYPE_NMEA, 1234, 50, 1500);   // dtMs=500 <1000s window: no rate update yet

    {
        const MessageStats::stats_t &s = stats.nmea.at(1234);
        EXPECT_EQ(s.bytesPerSec, 0);
        EXPECT_EQ(s.bytes, 100u);   // accumulating, not yet flushed
    }

    MessageStats::append("", stats, _PTYPE_NMEA, 1234, 50, 2100);   // dtMs=1100 >=1000: flush window

    const MessageStats::stats_t &s = stats.nmea.at(1234);
    EXPECT_EQ(s.bytesPerSec, (1000 * 150) / 1100);
    EXPECT_EQ(s.bytes, 0u);   // reset after computing the rate
}

TEST(MessageStats, DifferentIdsGetIndependentEntries) {
    MessageStats::mul_stats_t stats;
    MessageStats::append("", stats, _PTYPE_NMEA, 1111, 10, 1000);
    MessageStats::append("", stats, _PTYPE_NMEA, 2222, 20, 1000);

    ASSERT_EQ(stats.nmea.size(), 2u);
    EXPECT_EQ(stats.nmea.at(1111).count, 1);
    EXPECT_EQ(stats.nmea.at(2222).count, 1);
    EXPECT_EQ(stats.nmea.at(1111).bytes, 10u);
    EXPECT_EQ(stats.nmea.at(2222).bytes, 20u);
}

TEST(MessageStats, ProtocolTypesAreTrackedInSeparateBuckets) {
    MessageStats::mul_stats_t stats;
    MessageStats::append("", stats, _PTYPE_NMEA, 1, 10, 1000);
    MessageStats::append("", stats, _PTYPE_RTCM3, 1005, 20, 1000);
    MessageStats::append("", stats, _PTYPE_INERTIAL_SENSE_ACK, 0, 5, 1000);
    MessageStats::append("", stats, _PTYPE_PARSE_ERROR, 0, 1, 1000);

    EXPECT_EQ(stats.nmea.size(), 1u);
    EXPECT_EQ(stats.rtcm3.size(), 1u);
    EXPECT_EQ(stats.ack.count, 1);
    EXPECT_EQ(stats.parseError.count, 1);

    // Buckets are independent: touching one doesn't perturb the others.
    EXPECT_TRUE(stats.ublox.empty());
}

TEST(MessageStats, ClearResetsAllBucketsAndHistory) {
    MessageStats ms;
    ms.history.push_back("some event");
    MessageStats::append("", ms.stats, _PTYPE_NMEA, 1, 10, 1000);
    MessageStats::append("", ms.stats, _PTYPE_RTCM3, 1005, 20, 1000);
    ms.update = true;

    ms.clear();

    EXPECT_TRUE(ms.stats.isb.empty());
    EXPECT_TRUE(ms.stats.nmea.empty());
    EXPECT_TRUE(ms.stats.rtcm3.empty());
    EXPECT_TRUE(ms.history.empty());
    EXPECT_FALSE(ms.update);
    EXPECT_FALSE(ms.historyPaused);
}
