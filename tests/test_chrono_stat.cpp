// D-10-adjacent / SN-8308: unit tests for ChronoStat, covering the
// existing dt/ddt tracking plus the Welford online variance/stddev
// added for the EvalTool GNSS dt-guard work, and the clear()+sample()
// reset pattern that the guarded wrapper (EvalTool-side) relies on to
// rebase after a backwards/absurd time jump.

#include <gtest/gtest.h>

#include <cmath>

#include "ChronoStat.h"

TEST(ChronoStat, InitialStateHasNoData) {
    ChronoStat stat;
    EXPECT_EQ(stat.count(), 0);
    EXPECT_FALSE(stat.hasData());
    EXPECT_EQ(stat.dtCnt, 0);
    EXPECT_EQ(stat.dtVariance, 0);
}

TEST(ChronoStat, FirstSampleProducesNoDt) {
    ChronoStat stat;
    stat.sample(10.0);
    EXPECT_EQ(stat.count(), 1);
    // A single sample can't produce a dt yet.
    EXPECT_FALSE(stat.hasData());
    EXPECT_EQ(stat.dtCnt, 0);
    EXPECT_EQ(stat.dtMin, 0);
    EXPECT_EQ(stat.dtMax, 0);
}

TEST(ChronoStat, MinMaxTrackAcrossSamples) {
    ChronoStat stat;
    stat.sample(0.0);
    stat.sample(1.0);   // dt = 1.0
    stat.sample(1.5);   // dt = 0.5
    stat.sample(3.5);   // dt = 2.0

    EXPECT_TRUE(stat.hasData());
    EXPECT_EQ(stat.dtCnt, 3);
    EXPECT_DOUBLE_EQ(stat.dtMin, 0.5);
    EXPECT_DOUBLE_EQ(stat.dtMax, 2.0);
}

TEST(ChronoStat, AverageMatchesSimpleMean) {
    ChronoStat stat;
    // dt sequence: 1, 3, 5 -> mean 3
    stat.sample(0.0);
    stat.sample(1.0);
    stat.sample(4.0);
    stat.sample(9.0);

    EXPECT_EQ(stat.dtCnt, 3);
    EXPECT_NEAR(stat.dtAvg, 3.0, 1e-9);
}

TEST(ChronoStat, VarianceMatchesReferenceCalculation) {
    ChronoStat stat;
    // dt sequence chosen to have an easy-to-verify population variance.
    // times: 0, 2, 5, 6 -> dts: 2, 3, 1 ; mean = 2 ; population variance = ((0)^2+(1)^2+(-1)^2)/3 = 2/3
    stat.sample(0.0);
    stat.sample(2.0);
    stat.sample(5.0);
    stat.sample(6.0);

    EXPECT_EQ(stat.dtCnt, 3);
    EXPECT_NEAR(stat.dtAvg, 2.0, 1e-9);
    EXPECT_NEAR(stat.dtVariance, 2.0 / 3.0, 1e-9);
    EXPECT_NEAR(stat.dtStdDev(), std::sqrt(2.0 / 3.0), 1e-9);
}

TEST(ChronoStat, VarianceIsZeroForConstantDt) {
    ChronoStat stat;
    stat.sample(0.0);
    stat.sample(1.0);
    stat.sample(2.0);
    stat.sample(3.0);

    EXPECT_NEAR(stat.dtVariance, 0.0, 1e-12);
    EXPECT_NEAR(stat.dtStdDev(), 0.0, 1e-12);
}

TEST(ChronoStat, ClearResetsAllDtStats) {
    ChronoStat stat;
    stat.sample(0.0);
    stat.sample(1.0);
    stat.sample(3.0);
    ASSERT_TRUE(stat.hasData());
    ASSERT_GT(stat.dtVariance, 0.0);

    stat.clear();

    EXPECT_EQ(stat.count(), 0);
    EXPECT_FALSE(stat.hasData());
    EXPECT_EQ(stat.dtCnt, 0);
    EXPECT_EQ(stat.dtMin, 0);
    EXPECT_EQ(stat.dtMax, 0);
    EXPECT_EQ(stat.dtAvg, 0);
    EXPECT_EQ(stat.dtVariance, 0);
}

// This is the pattern the EvalTool-side guarded wrapper relies on: on a
// backwards/absurd jump, call clear() and re-baseline with the new
// timestamp as if it were the first sample ever seen, rather than
// recording a bogus dt into min/max/avg/variance.
TEST(ChronoStat, ClearThenResampleBehavesAsFreshStart) {
    ChronoStat stat;
    stat.sample(0.0);
    stat.sample(1.0);   // dt = 1.0 -- establishes a baseline
    stat.sample(2.0);   // dt = 1.0

    // Simulate a GPS-week wraparound / backwards jump: the guarded
    // wrapper detects this externally and rebases instead of sampling
    // the raw (huge negative) delta.
    stat.clear();
    stat.sample(0.05);  // new baseline post-wrap; still no dt yet
    EXPECT_FALSE(stat.hasData());
    EXPECT_EQ(stat.dtCnt, 0);

    stat.sample(0.15);  // dt = 0.10, first "real" dt after the rebase
    EXPECT_TRUE(stat.hasData());
    EXPECT_EQ(stat.dtCnt, 1);
    EXPECT_NEAR(stat.dtMin, 0.10, 1e-9);
    EXPECT_NEAR(stat.dtMax, 0.10, 1e-9);
    EXPECT_NEAR(stat.dtAvg, 0.10, 1e-9);
    EXPECT_NEAR(stat.dtVariance, 0.0, 1e-12);
}

TEST(ChronoStat, LabelRoundTrips) {
    ChronoStat stat;
    EXPECT_FALSE(stat.hasLabel());
    stat.setLabel("DID_GNSS1_POS");
    EXPECT_TRUE(stat.hasLabel());
    EXPECT_EQ(stat.getLabel(), "DID_GNSS1_POS");
}
