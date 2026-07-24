// SN-8345: unit tests for ISLogReader::findGaps — coverage-gap detection on
// the resolved timeline (the pure algorithm; segment spans in, gaps out).

#include <gtest/gtest.h>

#include "ISLogReader.h"
#include "ISTimeStamp.h"

#include <vector>

using namespace inertial_sense;

namespace {
// A resolved span [startMs, endMs] with the given segment id. findGaps only
// reads TimeStamp::value, so the source/confidence are immaterial here.
ISLogReader::SegmentSpan span(int id, uint64_t startMs, uint64_t endMs) {
    ISLogReader::SegmentSpan s;
    s.segmentId = id;
    s.start = TimeStamp::fromResolvedViaSync(startMs, 1, TimeConfidence::Exact);
    s.end   = TimeStamp::fromResolvedViaSync(endMs,   1, TimeConfidence::Exact);
    return s;
}
} // namespace

// Contiguous / touching spans → no gaps.
TEST(FindGaps, ContiguousSpansNoGap) {
    auto gaps = ISLogReader::findGaps(
        { span(0, 0, 1000), span(1, 1000, 2000), span(2, 2000, 3000) }, 100);
    EXPECT_TRUE(gaps.empty());
}

// A single interior gap wider than the threshold is reported, with the correct
// boundaries and the no-segment sentinel.
TEST(FindGaps, SingleGapReportedWithBoundaries) {
    auto gaps = ISLogReader::findGaps(
        { span(0, 0, 1000), span(1, 5000, 6000) }, 1000);
    ASSERT_EQ(gaps.size(), 1u);
    EXPECT_EQ(gaps[0].startTime.value, 1000u);   // end of prior coverage
    EXPECT_EQ(gaps[0].endTime.value,   5000u);   // start of next coverage
    EXPECT_EQ(gaps[0].segmentId, ISLogReader::kNoSegment);
    EXPECT_EQ(gaps[0].durationMs(), 4000u);
}

// A gap at or below the threshold is normal inter-segment spacing — not reported.
TEST(FindGaps, GapAtOrBelowThresholdSkipped) {
    // 500 ms gap, threshold 500 -> not reported (strictly greater required).
    EXPECT_TRUE(ISLogReader::findGaps({ span(0, 0, 1000), span(1, 1500, 2000) }, 500).empty());
    // 501 ms gap, threshold 500 -> reported.
    EXPECT_EQ(ISLogReader::findGaps({ span(0, 0, 1000), span(1, 1501, 2000) }, 500).size(), 1u);
}

// Overlapping spans never produce a gap (coverage high-water handles it).
TEST(FindGaps, OverlappingSpansNoGap) {
    auto gaps = ISLogReader::findGaps(
        { span(0, 0, 3000), span(1, 1000, 2000), span(2, 2500, 4000) }, 100);
    EXPECT_TRUE(gaps.empty());
}

// A long span that swallows a later short one must not open a false gap after it.
TEST(FindGaps, EnclosedSpanDoesNotOpenGap) {
    // span0 covers [0,10000]; span1 [2000,3000] is inside it; span2 [10500,11000]
    // is only 500 past coverEnd(=10000) -> below threshold 1000, no gap.
    auto gaps = ISLogReader::findGaps(
        { span(0, 0, 10000), span(1, 2000, 3000), span(2, 10500, 11000) }, 1000);
    EXPECT_TRUE(gaps.empty());
}

// Unsorted input is handled; multiple gaps come back in ascending order.
TEST(FindGaps, UnsortedInputMultipleGaps) {
    auto gaps = ISLogReader::findGaps(
        { span(2, 9000, 10000), span(0, 0, 1000), span(1, 4000, 5000) }, 1000);
    ASSERT_EQ(gaps.size(), 2u);
    EXPECT_EQ(gaps[0].startTime.value, 1000u);
    EXPECT_EQ(gaps[0].endTime.value,   4000u);
    EXPECT_EQ(gaps[1].startTime.value, 5000u);
    EXPECT_EQ(gaps[1].endTime.value,   9000u);
}

// Degenerate inputs: empty and single-span produce no gaps.
TEST(FindGaps, EmptyAndSingleNoGap) {
    EXPECT_TRUE(ISLogReader::findGaps({}, 100).empty());
    EXPECT_TRUE(ISLogReader::findGaps({ span(0, 0, 1000) }, 100).empty());
}

// Invalid spans (inverted or all-zero) are filtered before detection.
TEST(FindGaps, InvalidSpansFiltered) {
    ISLogReader::SegmentSpan inverted = span(9, 5000, 4000);   // end < start
    ISLogReader::SegmentSpan zero     = span(8, 0, 0);         // all-zero
    auto gaps = ISLogReader::findGaps(
        { span(0, 0, 1000), inverted, zero, span(1, 6000, 7000) }, 1000);
    ASSERT_EQ(gaps.size(), 1u);
    EXPECT_EQ(gaps[0].startTime.value, 1000u);
    EXPECT_EQ(gaps[0].endTime.value,   6000u);
}
