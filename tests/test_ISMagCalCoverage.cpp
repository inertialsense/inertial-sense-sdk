/**
 * @file test_ISMagCalCoverage.cpp
 * @brief Tests for the magnetometer calibration coverage map (DID_MAG_CAL_COVERAGE): wire layout, equal-area cell
 *        indexing, 4-bit cell storage, coverage statistics, suggested direction, data mapping and cltool display.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */
#include <gtest/gtest.h>
#include <cmath>
#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include "data_sets.h"
#include "ISMagCalCoverage.h"
#include "ISDataMappings.h"
#include "ISDisplay.h"

namespace {

mag_cal_coverage_t blank() { mag_cal_coverage_t m; memset(&m, 0, sizeof(m)); return m; }

TEST(MagCalCoverage, WireLayoutIsStable)
{   // Changing any of these breaks hosts/firmware built against the old layout: bump cellScheme or add a new DID instead.
    EXPECT_EQ(sizeof(mag_cal_coverage_t), 116u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, timeOfWeekMs), 0u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, cellScheme), 4u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, flags), 11u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, fitQuality), 12u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, sampleCount), 16u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, poseDir), 20u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, guidePhase), 23u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, fieldDir), 24u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, fitStage), 27u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, poseErrDeg), 28u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, ringPct), 29u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, gravDir), 30u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, inclSin), 33u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, qualityWhy), 34u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, normErrPct), 35u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, required), 36u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, cells), 52u);
    EXPECT_EQ((int)DID_MAG_CAL_COVERAGE, 113);
}

TEST(MagCalCoverage, CellCenterRoundTripsThroughIndex)
{
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++)
    {
        float c[3]; magCalCellCenter(i, c);
        EXPECT_NEAR(c[0]*c[0] + c[1]*c[1] + c[2]*c[2], 1.0f, 1e-5f);
        EXPECT_EQ(magCalCellIndex(c), i) << "cell " << i;
    }
}

TEST(MagCalCoverage, CellsAreEqualArea)
{   // Uniform points on the sphere must land in every cell with equal probability (4*pi/128 sr each).
    std::mt19937 rng(7); std::normal_distribution<float> n(0.f, 1.f);
    std::vector<unsigned> hits(MAG_CAL_CELL_COUNT, 0);
    const int N = 400000;
    for (int k = 0; k < N; k++)
    {
        float v[3] = { n(rng), n(rng), n(rng) }; unsigned idx;
        ASSERT_TRUE(magCalCellIndexOfVector(v, &idx));
        hits[idx]++;
    }
    const double expected = (double)N / MAG_CAL_CELL_COUNT;      // 3125
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++)
        EXPECT_NEAR(hits[i], expected, 5.0 * std::sqrt(expected)) << "cell " << i << " is not 1/128 of the sphere";   // 5 sigma
}

TEST(MagCalCoverage, IndexRejectsDegenerateVectors)
{
    unsigned idx = 99; float zero[3] = {0, 0, 0}, nan3[3] = { NAN, 0, 0 };
    EXPECT_FALSE(magCalCellIndexOfVector(zero, &idx)); EXPECT_EQ(idx, 0u);
    EXPECT_FALSE(magCalCellIndexOfVector(nan3, &idx));
}

TEST(MagCalCoverage, FourBitCellsDoNotDisturbNeighboursAndSaturate)
{
    mag_cal_coverage_t m = blank();
    magCalCellSet(&m, 0, 5); magCalCellSet(&m, 1, 9); magCalCellSet(&m, 2, 1);
    EXPECT_EQ(magCalCellGet(&m, 0), 5u); EXPECT_EQ(magCalCellGet(&m, 1), 9u); EXPECT_EQ(magCalCellGet(&m, 2), 1u); EXPECT_EQ(magCalCellGet(&m, 3), 0u);
    EXPECT_EQ(m.cells[0], 0x95);                                    // even cell = low nibble
    for (int k = 0; k < 40; k++) magCalCellAdd(&m, 1);
    EXPECT_EQ(magCalCellGet(&m, 1), (unsigned)MAG_CAL_CELL_SATURATED);
    EXPECT_EQ(magCalCellGet(&m, 0), 5u);                            // neighbour untouched
    magCalCellSet(&m, 127, 200);                                    // clamps
    EXPECT_EQ(magCalCellGet(&m, 127), (unsigned)MAG_CAL_CELL_SATURATED);
}

TEST(MagCalCoverage, RequiredMaskAndStats)
{
    mag_cal_coverage_t m = blank();
    magCalRequireAll(&m);
    auto s = magCalCoverageStats(&m);
    EXPECT_EQ(s.requiredCells, 128u); EXPECT_EQ(s.requiredCovered, 0u); EXPECT_EQ(magCalProgressPct(&s), 0);
    for (unsigned i = 0; i < 64; i++) magCalCellSet(&m, i, MAG_CAL_COVERAGE_MIN_COUNT);
    s = magCalCoverageStats(&m);
    EXPECT_EQ(s.requiredCovered, 64u); EXPECT_EQ(magCalProgressPct(&s), 50);

    // Single-axis style: only an equatorial band is required; covering other cells must not raise progress
    mag_cal_coverage_t b = blank();
    for (unsigned i = 3 * 16; i < 5 * 16; i++) magCalCellSetRequired(&b, i, 1);
    for (unsigned i = 0; i < 16; i++) magCalCellSet(&b, i, 15);
    s = magCalCoverageStats(&b);
    EXPECT_EQ(s.requiredCells, 32u); EXPECT_EQ(s.requiredCovered, 0u); EXPECT_EQ(s.coveredCells, 16u); EXPECT_EQ(magCalProgressPct(&s), 0);
    EXPECT_EQ(magCalCellRequired(&b, 3 * 16), 1); EXPECT_EQ(magCalCellRequired(&b, 5 * 16), 0);
    s.requiredCells = 0; EXPECT_EQ(magCalProgressPct(&s), 0);      // nothing required: no divide by zero
}

// ---- Gravity-referenced pose guidance ---------------------------------------------------------------------------------------
mag_cal_coverage_t full()
{
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) magCalCellSet(&m, i, 15);
    return m;
}
int poseIndexOf(float x, float y, float z)
{
    float want[3] = { x, y, z }; float n = std::sqrt(x*x + y*y + z*z); for (int k = 0; k < 3; k++) want[k] /= n;
    for (int p = 0; p < MAG_CAL_POSE_COUNT; p++) { float a[3]; magCalPoseDir(p, a); if (magCalAngleDeg(a, want) < 0.1f) return p; }
    return -1;
}

TEST(MagCalPose, SixFacePosesAreDistinctUnitVectorsOnTheAxes)
{
    EXPECT_EQ(MAG_CAL_POSE_COUNT, 6);
    for (int p = 0; p < MAG_CAL_POSE_COUNT; p++)
    {
        float a[3]; magCalPoseDir(p, a);
        EXPECT_NEAR(a[0]*a[0] + a[1]*a[1] + a[2]*a[2], 1.0f, 1e-6f);
        int nz = (std::fabs(a[0]) > 0.01f) + (std::fabs(a[1]) > 0.01f) + (std::fabs(a[2]) > 0.01f);
        EXPECT_EQ(nz, 1) << "pose " << p << " must be a face (one axis), never an edge or corner: those need two faces equally low";
        for (int q = p + 1; q < MAG_CAL_POSE_COUNT; q++) { float b[3]; magCalPoseDir(q, b); EXPECT_GT(magCalAngleDeg(a, b), 89.0f) << p << " and " << q; }   // faces are 90 or 180 deg apart
    }
}

TEST(MagCalPose, PoseNamesAreWhatAPersonHoldingTheUnitWouldSay)
{
    char n[32]; int8_t d[3]; float a[3];
    magCalPoseDir(poseIndexOf(0, 0, 1), a);  magCalPackUnit(a, d); magCalPoseName(d, n, sizeof(n)); EXPECT_STREQ(n, "+Z face (bottom)");
    magCalPoseDir(poseIndexOf(0, 0, -1), a); magCalPackUnit(a, d); magCalPoseName(d, n, sizeof(n)); EXPECT_STREQ(n, "-Z face (top)");
    magCalPoseDir(poseIndexOf(1, 0, 0), a);  magCalPackUnit(a, d); magCalPoseName(d, n, sizeof(n)); EXPECT_STREQ(n, "+X face (front)");
    magCalPoseDir(poseIndexOf(-1, 0, 0), a); magCalPackUnit(a, d); magCalPoseName(d, n, sizeof(n)); EXPECT_STREQ(n, "-X face (rear)");
    magCalPoseDir(poseIndexOf(0, 1, 0), a);  magCalPackUnit(a, d); magCalPoseName(d, n, sizeof(n)); EXPECT_STREQ(n, "+Y face (right side)");
    magCalPoseDir(poseIndexOf(0, -1, 0), a); magCalPackUnit(a, d); magCalPoseName(d, n, sizeof(n)); EXPECT_STREQ(n, "-Y face (left side)");
    int8_t zero[3] = { 0, 0, 0 }; magCalPoseName(zero, n, sizeof(n)); EXPECT_STREQ(n, "");
    char tiny[4]; magCalPoseName(d, tiny, sizeof(tiny)); EXPECT_LT(std::strlen(tiny), sizeof(tiny));      // never overruns
}

TEST(MagCalPose, CellCentresFromTablesMatchTheTrigonometricDefinition)
{
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++)
    {
        float c[3]; magCalCellCenter(i, c);
        unsigned iz = i / 16, il = i % 16; float z = ((iz + 0.5f) * 0.25f) - 1.0f, r = std::sqrt(1 - z*z), lon = ((il + 0.5f) * (6.2831853f / 16)) - 3.14159265f;
        EXPECT_NEAR(c[0], r * std::cos(lon), 2e-6f); EXPECT_NEAR(c[1], r * std::sin(lon), 2e-6f); EXPECT_NEAR(c[2], z, 1e-6f);
    }
}

TEST(MagCalPose, RingIsTheSetOfCellsTheCirclePassesThrough)
{
    float a[3] = { 0, 0, 1 };
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    uint8_t mask[MAG_CAL_RING_MASK_BYTES];
    magCalRingMask(a, 0.0f, mask);                  // field horizontal: a great circle round the pose axis, along the z = 0 band edge
    mag_cal_ring_t eq = magCalPoseRing(&m, a, 0.0f);
    EXPECT_GE(eq.cells, 16); EXPECT_LE(eq.cells, 32); EXPECT_EQ(eq.missing, eq.cells);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) if (magCalRingHas(mask, i)) { unsigned z = i / MAG_CAL_CELL_LON_SECTORS; EXPECT_TRUE(z == 3 || z == 4) << "cell " << i; }
    const float s66 = std::sin(66.0f * 3.14159265f / 180.0f);       // Utah: field 66 deg below the horizon
    mag_cal_ring_t steep = magCalPoseRing(&m, a, s66);
    EXPECT_EQ(steep.cells, 16) << "a small circle about 24 deg from the pose axis, wholly in the top band";
    for (unsigned i = 0; i < 16; i++) magCalCellSet(&m, 7 * 16 + i, 15);                       // cover the top band: that ring is done
    EXPECT_EQ(magCalPoseRing(&m, a, s66).missing, 0);
}

TEST(MagCalPose, ACellTheCircleOnlyClipsIsNotPartOfTheRing)
{   // The old definition took every cell whose CENTRE was near the circle, including cells the circle never enters: progress through the ring could not reach 100%.
    const float s = std::sin(40.0f * 3.14159265f / 180.0f);
    float g[3] = { 0.3f, -0.5f, 0.8124f }; float gn = std::sqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]); for (float &v : g) v /= gn;
    uint8_t mask[MAG_CAL_RING_MASK_BYTES]; magCalRingMask(g, s, mask);
    // Every ring cell is genuinely visited: walk the circle at 1 degree and require that each ring cell is entered, and that the walk never enters a cell the ring lacks
    // for more than the clipping allowance
    float h[3] = { 1, 0, 0 }; float d = h[0]*g[0]; float e1[3] = { h[0]-d*g[0], -d*g[1], -d*g[2] }; float en = std::sqrt(e1[0]*e1[0]+e1[1]*e1[1]+e1[2]*e1[2]); for (float &v : e1) v /= en;
    float e2[3] = { g[1]*e1[2]-g[2]*e1[1], g[2]*e1[0]-g[0]*e1[2], g[0]*e1[1]-g[1]*e1[0] };
    int visits[MAG_CAL_CELL_COUNT] = { 0 }; float cI = std::sqrt(1 - s*s);
    for (int deg = 0; deg < 360; deg++) { float c = std::cos(deg * 0.0174532925f), sn = std::sin(deg * 0.0174532925f); float u[3]; for (int k = 0; k < 3; k++) u[k] = s*g[k] + cI*(c*e1[k] + sn*e2[k]); visits[magCalCellIndex(u)]++; }
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++)
    {
        if (magCalRingHas(mask, i)) EXPECT_GE(visits[i], 3) << "ring cell " << i << " is never really entered";
        else EXPECT_LE(visits[i], 6) << "a cell with a long arc is missing from the ring: " << i;
    }
}

TEST(MagCalPose, ChoosesThePoseWhoseRingHoldsTheGaps)
{
    // Field horizontal (sinI = 0): the ring of pose +X is the great circle x ~ 0. Open exactly those cells and nothing else.
    mag_cal_coverage_t m = full();
    float ax[3] = { 1, 0, 0 }; uint8_t rm[MAG_CAL_RING_MASK_BYTES]; magCalRingMask(ax, 0.0f, rm);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) if (magCalRingHas(rm, i)) magCalCellSet(&m, i, 0);
    float g[3] = { 0, 0, 1 };
    int p = magCalChoosePose(&m, g, 0.0f, -1);
    ASSERT_GE(p, 0);
    float a[3]; magCalPoseDir(p, a);
    EXPECT_GT(std::fabs(a[0]), 0.9f) << "should pick +X or -X (same ring)";
    EXPECT_EQ(magCalChoosePose(&m, g, 0.0f, p) == p, false) << "an excluded pose must not be returned";
}

TEST(MagCalPose, NothingToDoWhenNothingIsMissing)
{
    mag_cal_coverage_t m = full(); float g[3] = { 0, 0, 1 };
    EXPECT_EQ(magCalChoosePose(&m, g, 0.9f, -1), -1);
    mag_cal_coverage_t none = blank();                      // nothing required
    EXPECT_EQ(magCalChoosePose(&none, g, 0.9f, -1), -1);
}

TEST(MagCalPose, ARequiredMaskIsRespected)
{
    mag_cal_coverage_t m = blank();                          // nothing covered, but only one cell is required
    magCalCellSetRequired(&m, 5 * 16 + 3, 1);
    float g[3] = { 0, 0, 1 }, c[3]; magCalCellCenter(5 * 16 + 3, c);
    int p = magCalChoosePose(&m, g, 0.5f, -1);
    if (p >= 0) { float a[3]; magCalPoseDir(p, a); uint8_t rm[MAG_CAL_RING_MASK_BYTES]; magCalRingMask(a, 0.5f, rm); EXPECT_TRUE(magCalRingHas(rm, 5 * 16 + 3)) << "the chosen pose must actually sweep the one required cell"; }
}

TEST(MagCalPose, NearerPoseBreaksATieButGainWins)
{   // two identical rings (opposite poses +Z / -Z share the same great circle when the field is horizontal): the nearer one is chosen
    mag_cal_coverage_t m = full();
    float az[3] = { 0, 0, 1 }; uint8_t rm[MAG_CAL_RING_MASK_BYTES]; magCalRingMask(az, 0.0f, rm);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) if (magCalRingHas(rm, i)) magCalCellSet(&m, i, 0);
    float gDown[3] = { 0, 0, 1 }, gUp[3] = { 0, 0, -1 };
    float a[3];
    magCalPoseDir(magCalChoosePose(&m, gDown, 0.0f, -1), a); EXPECT_GT(a[2], 0.9f);
    magCalPoseDir(magCalChoosePose(&m, gUp, 0.0f, -1), a);   EXPECT_LT(a[2], -0.9f);
}

TEST(MagCalPose, UnitPackingRoundsAndClamps)
{
    float u[3] = { 0.f, -1.f, 1.f / std::sqrt(2.f) }; int8_t p[3]; magCalPackUnit(u, p);
    EXPECT_EQ(p[0], 0); EXPECT_EQ(p[1], -127); EXPECT_EQ(p[2], 90);
}

TEST(MagCalCoverage, DataMappingIsRegistered)
{
    EXPECT_STREQ(cISDataMappings::DataName(DID_MAG_CAL_COVERAGE), "DID_MAG_CAL_COVERAGE");
    EXPECT_EQ(cISDataMappings::DataSize(DID_MAG_CAL_COVERAGE), sizeof(mag_cal_coverage_t));
    const map_name_to_info_t *map = cISDataMappings::NameToInfoMap(DID_MAG_CAL_COVERAGE);
    ASSERT_NE(map, nullptr);
    for (const char *name : { "lifecycle", "reason", "quality", "progressPct", "coveragePct", "fitQuality", "sampleCount", "poseDir", "guidePhase", "fieldDir", "poseErrDeg", "ringPct", "gravDir", "required", "cells" })
        EXPECT_NE(map->find(name), map->end()) << name;
    EXPECT_EQ(map->at("cells").offset, offsetof(mag_cal_coverage_t, cells));
    EXPECT_EQ(map->at("progressPct").offset, offsetof(mag_cal_coverage_t, progressPct));
}

TEST(MagCalCoverage, DataMappingCoversEveryByteOfTheStructExactlyOnce)
{   // DataMapper only prints a warning when sizes disagree; make it a failure. Every byte on the wire must belong to a named field.
    const map_name_to_info_t *map = cISDataMappings::NameToInfoMap(DID_MAG_CAL_COVERAGE);
    ASSERT_NE(map, nullptr);
    std::vector<int> owner(sizeof(mag_cal_coverage_t), 0);
    for (const auto &kv : *map)
        for (uint32_t b = kv.second.offset; b < kv.second.offset + kv.second.size; b++)
        { ASSERT_LT(b, owner.size()) << kv.first << " extends past the struct"; owner[b]++; }
    for (size_t b = 0; b < owner.size(); b++) EXPECT_EQ(owner[b], 1) << "byte " << b << " is mapped " << owner[b] << " times";
}

TEST(MagCalCoverage, DisplayShowsLifecycleReasonAndMap)
{
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    m.cellScheme = MAG_CAL_CELLS_8Z_16LON; m.lifecycle = MAG_CAL_LC_FITTING; m.mode = MAG_CAL_STATE_MULTI_AXIS; m.reason = MAG_CAL_REASON_FIT_REJECTED;
    m.quality = MAG_CAL_QUALITY_NONE; m.progressPct = 99; m.flags = MAG_CAL_FLAG_COVERAGE_VALID; m.sampleCount = 2897; m.fitQuality = 12.5f;
    magCalCellSet(&m, 0, 1); magCalCellSet(&m, 1, 4); magCalCellSet(&m, 2, 8); magCalCellSet(&m, 3, 15);
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;

    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    std::string s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("FITTING"), std::string::npos);
    EXPECT_NE(s.find("multi-axis"), std::string::npos);
    EXPECT_NE(s.find("fit not accepted yet"), std::string::npos);
    EXPECT_NE(s.find("99%"), std::string::npos);
    EXPECT_NE(s.find("map 0%"), std::string::npos) << s;
    EXPECT_NE(s.find(": o O #"), std::string::npos) << s;           // cells 0..3 glyphs: thin, ok, good, full
    // Constant layout: status, instruction, legend, longitude header, then one row per latitude band
    const int expectedLines = 1 + 1 + 1 + 1 + 1 + MAG_CAL_CELL_Z_BANDS;   // status, detail, instruction, legend, lon header, rows
    EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), expectedLines) << s;
    // No line may be long enough to wrap on an ordinary terminal: a wrapped line changes the line count and the screen jumps
    size_t start = 0; int lineNo = 0;
    while (start < s.size()) { size_t nl = s.find('\n', start); if (nl == std::string::npos) nl = s.size(); EXPECT_LE(nl - start, 80u) << "line " << lineNo << " is " << (nl - start) << " columns"; start = nl + 1; lineNo++; }
}

TEST(MagCalCoverage, DisplayShowsPoseGuidanceAndTheRingItFills)
{
    mag_cal_coverage_t m = full();
    float ax[3] = { 0, 0, 1 }; uint8_t ringMask[MAG_CAL_RING_MASK_BYTES]; magCalRingMask(ax, 0.0f, ringMask);
    int ringCells = 0; for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) if (magCalRingHas(ringMask, i)) { magCalCellSet(&m, i, 0); ringCells++; }   // the ring of +Z is empty
    m.lifecycle = MAG_CAL_LC_COLLECTING; m.mode = MAG_CAL_STATE_MULTI_AXIS; m.flags = MAG_CAL_FLAG_COVERAGE_VALID | MAG_CAL_FLAG_REQUIRED_VALID | MAG_CAL_FLAG_GRAVITY_VALID;
    magCalPackUnit(ax, m.poseDir);
    // inclination 0: field perpendicular to the floor direction
    float fld[3] = { 1, 0, 0 }, gr[3] = { 0, 0, 1 }; magCalPackUnit(fld, m.fieldDir); magCalPackUnit(gr, m.gravDir);
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);

    m.guidePhase = MAG_CAL_PHASE_AIM; m.poseErrDeg = 37;
    std::string s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("point the +Z face (bottom) at the floor: 37 deg to go"), std::string::npos) << s;
    // the empty ring is drawn with '*' (32 cells of it, plus the one in the legend), and the field-now marker is '@' (plus the legend)
    float nowDir[3] = { 1, 0, 0 }; unsigned nowCell = magCalCellIndex(nowDir);
    EXPECT_EQ(std::count(s.begin(), s.end(), '*'), ringCells - (magCalRingHas(ringMask, nowCell) ? 1 : 0) + 1) << s;      // the "now" cell shows as '@' instead (plus the legend's '*')
    EXPECT_EQ(std::count(s.begin(), s.end(), '@'), 1 + 1) << s;

    m.guidePhase = MAG_CAL_PHASE_TURN; m.ringPct = 40;
    s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("keep the +Z face down; turn slowly in place, like a turntable (ring 40%)"), std::string::npos) << s;

    // A fit that has not been accepted yet does not stop the guidance while gaps remain
    m.lifecycle = MAG_CAL_LC_FITTING; m.reason = MAG_CAL_REASON_FIT_REJECTED;
    s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("turn slowly in place"), std::string::npos) << s;
    EXPECT_NE(s.find("fit not accepted yet"), std::string::npos) << s;

    // Single axis: no pose, plain instruction
    mag_cal_coverage_t sa = blank(); sa.lifecycle = MAG_CAL_LC_COLLECTING; sa.mode = MAG_CAL_STATE_SINGLE_AXIS; sa.flags = MAG_CAL_FLAG_COVERAGE_VALID;
    s = d.DataToStringMagCalCoverage(sa, hdr);
    EXPECT_NE(s.find("keep turning about the vertical axis"), std::string::npos) << s;
}

TEST(MagCalCoverage, FitNotYetAcceptedStillShowsGuidanceWhileGapsRemain)
{   // Found on hardware: a rejected fit at PARTIAL coverage hid the target behind a wrong 'coverage complete' message.
    mag_cal_coverage_t m = full();
    for (int dl = -1; dl <= 1; dl++) magCalCellSet(&m, 2 * 16 + 5 + dl, 0);
    m.lifecycle = MAG_CAL_LC_FITTING; m.mode = MAG_CAL_STATE_MULTI_AXIS; m.reason = MAG_CAL_REASON_FIT_REJECTED; m.progressPct = 99; m.coveragePct = 97;
    m.flags = MAG_CAL_FLAG_COVERAGE_VALID | MAG_CAL_FLAG_REQUIRED_VALID;
    float a[3] = { 1, 0, 0 }; magCalPackUnit(a, m.poseDir); m.guidePhase = MAG_CAL_PHASE_AIM; m.poseErrDeg = 12;
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    std::string s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("point the +X face (front) at the floor"), std::string::npos) << s;
    EXPECT_NE(s.find("fit not accepted yet"), std::string::npos) << s;
    EXPECT_EQ(s.find("will not help"), std::string::npos) << "gaps remain: more rotation DOES help\n" << s;
    m.guidePhase = MAG_CAL_PHASE_NONE; memset(m.poseDir, 0, 3); m.coveragePct = 100;
    s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("the map is complete, more rotation will not help"), std::string::npos) << s;
}

TEST(MagCalCoverage, QualityHeldIsShownSoAFlickerFreeValueIsNotMistakenForALiveOne)
{
    mag_cal_coverage_t m = blank(); m.lifecycle = MAG_CAL_LC_COMPLETE; m.quality = MAG_CAL_QUALITY_GOOD; m.fitQuality = 2.0f; m.flags = MAG_CAL_FLAG_QUALITY_HELD;
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    EXPECT_NE(d.DataToStringMagCalCoverage(m, hdr).find("quality good (held: turning fast)"), std::string::npos);
    m.flags = 0;
    EXPECT_EQ(d.DataToStringMagCalCoverage(m, hdr).find("held"), std::string::npos);
}

TEST(MagCalCoverage, DisplayLineCountDoesNotDependOnTheContent)
{   // The TUI redraws in place; a different number of lines per update is what made the screen "jump".
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    mag_cal_coverage_t quiet = blank();
    mag_cal_coverage_t busy = blank(); magCalRequireAll(&busy);
    busy.lifecycle = MAG_CAL_LC_FITTING; busy.mode = MAG_CAL_STATE_MULTI_AXIS; busy.reason = MAG_CAL_REASON_REGION_FULL; busy.sampleCount = 4000000000u; busy.progressPct = 99; busy.coveragePct = 100;
    busy.fitQuality = 123456.7f; busy.poseDir[0] = 90; busy.poseDir[1] = -90; busy.poseDir[2] = 90; busy.guidePhase = MAG_CAL_PHASE_TURN; busy.poseErrDeg = 180; busy.ringPct = 100; busy.fieldDir[0] = 127; busy.gravDir[2] = 127; busy.flags = MAG_CAL_FLAG_QUALITY_HELD;
    auto lines = [&](const mag_cal_coverage_t &c) { std::string s = d.DataToStringMagCalCoverage(c, hdr); return (int)std::count(s.begin(), s.end(), '\n'); };
    EXPECT_EQ(lines(quiet), lines(busy));
}

TEST(MagCalCoverage, ChimePolicyRingsOncePerNewInstructionAndThriceOnCompletion)
{
    mag_cal_coverage_t a = blank(), b = blank();
    a.lifecycle = b.lifecycle = MAG_CAL_LC_COLLECTING;
    a.guidePhase = MAG_CAL_PHASE_AIM; a.poseDir[1] = -127; b = a;
    EXPECT_EQ(magCalChimeCount(a, false, b), 0) << "the first status seen never chimes";
    EXPECT_EQ(magCalChimeCount(a, true, b), 0) << "an unchanged instruction does not chime";
    b.poseErrDeg = 40; b.ringPct = 30;
    EXPECT_EQ(magCalChimeCount(a, true, b), 0) << "progress numbers changing do not chime";
    b.guidePhase = MAG_CAL_PHASE_TURN;
    EXPECT_EQ(magCalChimeCount(a, true, b), 1) << "aim -> turn chimes";
    mag_cal_coverage_t c = b; c.guidePhase = MAG_CAL_PHASE_AIM; c.poseDir[1] = 0; c.poseDir[0] = 127;
    EXPECT_EQ(magCalChimeCount(b, true, c), 1) << "a new face chimes";
    mag_cal_coverage_t none = c; none.guidePhase = MAG_CAL_PHASE_NONE;
    EXPECT_EQ(magCalChimeCount(c, true, none), 0) << "guidance going away is silent";
    mag_cal_coverage_t done = c; done.lifecycle = MAG_CAL_LC_COMPLETE; done.guidePhase = MAG_CAL_PHASE_NONE;
    EXPECT_EQ(magCalChimeCount(c, true, done), 3);
    EXPECT_EQ(magCalChimeCount(done, true, done), 0) << "completion chimes once";
}

TEST(MagCalCoverage, DisplayEmitsTheBellOnlyWhenTheInstructionChanges)
{
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    m.lifecycle = MAG_CAL_LC_COLLECTING; m.mode = MAG_CAL_STATE_MULTI_AXIS; m.guidePhase = MAG_CAL_PHASE_AIM; m.poseDir[2] = 127;
    auto bells = [&](const mag_cal_coverage_t &c) { std::string s = d.DataToStringMagCalCoverage(c, hdr); return (int)std::count(s.begin(), s.end(), '\a'); };
    EXPECT_EQ(bells(m), 0);
    EXPECT_EQ(bells(m), 0);
    m.guidePhase = MAG_CAL_PHASE_TURN;
    EXPECT_EQ(bells(m), 1);
    EXPECT_EQ(bells(m), 0);
    m.lifecycle = MAG_CAL_LC_COMPLETE; m.guidePhase = MAG_CAL_PHASE_NONE;
    EXPECT_EQ(bells(m), 3);
}

TEST(MagCalCoverage, RingUsesTheFirmwareInclinationWhenItIsReported)
{
    mag_cal_coverage_t m = blank();
    float fld[3] = { 0, 0, 1 }, gr[3] = { 0, 0, 1 }; magCalPackUnit(fld, m.fieldDir); magCalPackUnit(gr, m.gravDir);      // instantaneous field . floor = 1
    float s = -1;
    ASSERT_TRUE(magCalSinI(&m, &s));
    EXPECT_NEAR(s, 1.0f, 0.02f) << "fallback: from the two vectors";
    m.inclSin = 64; m.flags |= MAG_CAL_FLAG_INCL_VALID;
    ASSERT_TRUE(magCalSinI(&m, &s));
    EXPECT_NEAR(s, 64.0f / 127.0f, 1e-5f) << "the firmware's estimate wins";
    mag_cal_coverage_t none = blank();
    EXPECT_FALSE(magCalSinI(&none, &s)) << "nothing known";

    // And the display's ring follows it: the same pose draws a different ring for a different inclination
    m.lifecycle = MAG_CAL_LC_COLLECTING; m.mode = MAG_CAL_STATE_MULTI_AXIS; magCalRequireAll(&m); m.flags |= MAG_CAL_FLAG_COVERAGE_VALID | MAG_CAL_FLAG_REQUIRED_VALID;
    float ax[3] = { 0, 0, 1 }; magCalPackUnit(ax, m.poseDir); m.guidePhase = MAG_CAL_PHASE_AIM;
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    auto stars = [&](int8_t incl) { mag_cal_coverage_t c = m; c.inclSin = incl; cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY); std::string o = d.DataToStringMagCalCoverage(c, hdr); return (int)std::count(o.begin(), o.end(), '*'); };
    mag_cal_coverage_t fullMap = m; for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) magCalCellSet(&fullMap, i, MAG_CAL_COVERAGE_MIN_COUNT);
    { cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY); std::string o = d.DataToStringMagCalCoverage(fullMap, hdr); EXPECT_EQ(std::count(o.begin(), o.end(), '*'), 1) << "a full map has no ring cells left to fill: only the legend's '*'"; }
    (void)stars;
    mag_cal_coverage_t empty = m; for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) magCalCellSet(&empty, i, 0);
    auto starsEmpty = [&](int8_t incl) { mag_cal_coverage_t c = empty; c.inclSin = incl; cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY); std::string o = d.DataToStringMagCalCoverage(c, hdr); return (int)std::count(o.begin(), o.end(), '*'); };
    EXPECT_NE(starsEmpty(0), starsEmpty(100)) << "inclination 0 (ring at the equator) vs 52 deg (a small circle) fill different cells";
}

TEST(MagCalCoverage, QualityWhyHasPlainWordsForEveryReason)
{
    EXPECT_STREQ(magCalQualityWhyText(MAG_CAL_WHY_NONE), "");
    for (unsigned w = MAG_CAL_WHY_FIELD_VARIES; w <= MAG_CAL_WHY_NORM_INVALID; w++) EXPECT_GT(strlen(magCalQualityWhyText(w)), 20u) << w;
    EXPECT_STREQ(magCalQualityWhyText(200), "") << "unknown values from a newer firmware say nothing rather than something wrong";
}

} // namespace
