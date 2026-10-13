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
    EXPECT_EQ(sizeof(mag_cal_coverage_t), 108u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, timeOfWeekMs), 0u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, cellScheme), 4u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, flags), 11u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, fitQuality), 12u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, sampleCount), 16u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, guideRot), 20u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, targetCell), 23u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, fieldDir), 24u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, fitStage), 27u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, required), 28u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, cells), 44u);
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

// ---- Target selection ------------------------------------------------------------------------------------------------
mag_cal_coverage_t full()
{
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) magCalCellSet(&m, i, 15);
    return m;
}
void hole(mag_cal_coverage_t *m, int zc, int lc, int radius = 1)
{   // empty (2r+1)^2 block of cells centred on band zc, sector lc (longitude wraps)
    for (int dz = -radius; dz <= radius; dz++) for (int dl = -radius; dl <= radius; dl++)
    { int z = zc + dz; if (z < 0 || z >= MAG_CAL_CELL_Z_BANDS) continue; magCalCellSet(m, z * 16 + ((lc + dl + 16) % 16), 0); }
}

TEST(MagCalGuide, NoTargetWhenNothingRequiredIsMissing)
{
    mag_cal_coverage_t m = full();
    EXPECT_EQ(magCalChooseTarget(&m, nullptr, -1, -1), -1);
    mag_cal_coverage_t none = blank();                               // nothing required at all
    EXPECT_EQ(magCalChooseTarget(&none, nullptr, -1, -1), -1);
}

TEST(MagCalGuide, PrefersTheLargestGapOverAnIsolatedMissingCell)
{
    mag_cal_coverage_t m = full(); hole(&m, 2, 5); magCalCellSet(&m, 6 * 16 + 12, 0);
    EXPECT_EQ(magCalChooseTarget(&m, nullptr, -1, -1), 2 * 16 + 5) << "should aim at the middle of the 3x3 hole (gain 9), not the lone cell (gain 1)";
}

TEST(MagCalGuide, LargestGainBeatsANearerSmallerGap)
{
    mag_cal_coverage_t m = full(); hole(&m, 2, 5); magCalCellSet(&m, 6 * 16 + 12, 0);
    float near[3]; magCalCellCenter(6 * 16 + 12, near);               // the user is sitting on the lone cell
    EXPECT_EQ(magCalChooseTarget(&m, near, -1, -1), 2 * 16 + 5);
}

TEST(MagCalGuide, AmongComparableGapsTheNearestWins)
{
    mag_cal_coverage_t m = full(); hole(&m, 2, 2); hole(&m, 5, 10);   // two equal 3x3 holes
    float nearA[3], nearB[3]; magCalCellCenter(2 * 16 + 3, nearA); magCalCellCenter(5 * 16 + 9, nearB);
    EXPECT_EQ(magCalChooseTarget(&m, nearA, -1, -1), 2 * 16 + 2);
    EXPECT_EQ(magCalChooseTarget(&m, nearB, -1, -1), 5 * 16 + 10);
}

TEST(MagCalGuide, KeepsTheTargetWhileMovingToItAndMovesOnOnceCovered)
{
    mag_cal_coverage_t m = full(); hole(&m, 2, 2); hole(&m, 5, 10);
    float start[3]; magCalCellCenter(2 * 16 + 3, start);
    int t = magCalChooseTarget(&m, start, -1, -1);
    ASSERT_EQ(t, 2 * 16 + 2);
    // User wanders closer to the OTHER hole: the target must not flip while it is still missing and not yet reached
    float wander[3]; magCalCellCenter(4 * 16 + 9, wander);
    EXPECT_EQ(magCalChooseTarget(&m, wander, t, -1), t);
    // Once the target cell is covered, the next target is chosen
    for (int dz = -1; dz <= 1; dz++) for (int dl = -1; dl <= 1; dl++) magCalCellSet(&m, (2 + dz) * 16 + (2 + dl), 15);
    EXPECT_EQ(magCalChooseTarget(&m, wander, t, -1), 5 * 16 + 10);
}

TEST(MagCalGuide, AStaleTargetCanBeExcludedSoItIsNotPickedAgain)
{
    mag_cal_coverage_t m = full(); hole(&m, 2, 2); hole(&m, 5, 10);
    float start[3]; magCalCellCenter(2 * 16 + 3, start);
    int first = magCalChooseTarget(&m, start, -1, -1);
    ASSERT_EQ(first, 2 * 16 + 2);
    int second = magCalChooseTarget(&m, start, -1, first);          // timed out: must move on even though it is still the best gap
    EXPECT_NE(second, first);
    EXPECT_GE(second, 0);
}

TEST(MagCalGuide, SmoothingDampsJumpsAndTakesTheFirstValueAsIs)
{
    float st[3] = { 0, 0, 0 }, in1[3] = { 100, -50, 20 };
    magCalSmoothRot(st, in1, 1, 0.35f);
    EXPECT_FLOAT_EQ(st[0], 100.f); EXPECT_FLOAT_EQ(st[1], -50.f);   // reset: no lag at the start of a new target
    float jump[3] = { -100, 50, -20 };                                // the raw rotation flips sign (user rotated the device)
    magCalSmoothRot(st, jump, 0, 0.35f);
    EXPECT_NEAR(st[0], 100.f + 0.35f * (-200.f), 1e-3f);               // moved only part of the way
    for (int i = 0; i < 40; i++) magCalSmoothRot(st, jump, 0, 0.35f);
    EXPECT_NEAR(st[0], -100.f, 0.01f);                                  // and converges
}

TEST(MagCalGuide, OnlyRequiredCellsAreTargeted)
{
    mag_cal_coverage_t m = blank();
    for (unsigned i = 3 * 16; i < 5 * 16; i++) { magCalCellSetRequired(&m, i, 1); magCalCellSet(&m, i, 15); }
    magCalCellSet(&m, 4 * 16 + 7, 0);                                  // the only required cell that is missing
    EXPECT_EQ(magCalChooseTarget(&m, nullptr, -1, -1), 4 * 16 + 7) << "empty cells outside the required region must be ignored";
}

// ---- Rotation guidance -----------------------------------------------------------------------------------------------
static void rotateVecByBodyRotation(const float rotDeg[3], const float u[3], float out[3])
{   // Apply a body rotation (rotation vector, degrees) to the device: the sensor-frame direction of a world-fixed vector becomes R^T u (Rodrigues)
    float w[3] = { rotDeg[0] * (float)M_PI / 180.f, rotDeg[1] * (float)M_PI / 180.f, rotDeg[2] * (float)M_PI / 180.f };
    float th = std::sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
    if (th < 1e-9f) { memcpy(out, u, 12); return; }
    float k[3] = { w[0]/th, w[1]/th, w[2]/th };
    float kxu[3] = { k[1]*u[2]-k[2]*u[1], k[2]*u[0]-k[0]*u[2], k[0]*u[1]-k[1]*u[0] }, kdu = k[0]*u[0]+k[1]*u[1]+k[2]*u[2];
    float c = std::cos(-th), s = std::sin(-th);                         // R^T = rotation by -theta about k
    for (int i = 0; i < 3; i++) out[i] = u[i]*c + kxu[i]*s + k[i]*kdu*(1.f - c);
}

TEST(MagCalGuide, FollowingTheRotationBringsTheFieldOntoTheTarget)
{
    std::mt19937 rng(11); std::normal_distribution<float> n(0.f, 1.f);
    auto unit = [&](float v[3]) { float l = std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); for (int i = 0; i < 3; i++) v[i] /= l; };
    for (int k = 0; k < 2000; k++)
    {
        float u[3] = { n(rng), n(rng), n(rng) }, d[3] = { n(rng), n(rng), n(rng) }; unit(u); unit(d);
        float rot[3]; float ang = magCalGuideRotationDeg(u, d, rot);
        float mag = std::sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]);
        EXPECT_NEAR(mag, ang, 1e-2f);
        EXPECT_LE(ang, 180.001f);
        float out[3]; rotateVecByBodyRotation(rot, u, out);
        EXPECT_LT(magCalAngleDeg(out, d), 0.05f) << "applying the commanded device rotation must put the field direction on the target";
    }
}

TEST(MagCalGuide, RotationSignConventionYawNegativeMovesFieldFromAheadToRight)
{   // Field straight ahead (+X). Turning the nose RIGHT (positive yaw) moves the world-fixed field to the LEFT of the nose (-Y).
    // To bring it to the right (+Y) the device must yaw negative: roll 0, pitch 0, yaw -90.
    float u[3] = { 1, 0, 0 }, d[3] = { 0, 1, 0 }, rot[3];
    magCalGuideRotationDeg(u, d, rot);
    EXPECT_NEAR(rot[0], 0.f, 1e-3f); EXPECT_NEAR(rot[1], 0.f, 1e-3f); EXPECT_NEAR(rot[2], -90.f, 1e-3f);
}

TEST(MagCalGuide, ZeroWhenOnTargetAndDefinedWhenOpposite)
{
    float u[3] = { 0, 0, 1 }, rot[3];
    EXPECT_NEAR(magCalGuideRotationDeg(u, u, rot), 0.f, 1e-3f);
    EXPECT_NEAR(rot[0] + rot[1] + rot[2], 0.f, 1e-6f);
    float opp[3] = { 0, 0, -1 };
    EXPECT_NEAR(magCalGuideRotationDeg(u, opp, rot), 180.f, 1e-3f);
    EXPECT_NEAR(std::sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]), 180.f, 1e-2f);
    float out[3]; rotateVecByBodyRotation(rot, u, out); EXPECT_LT(magCalAngleDeg(out, opp), 0.1f);
}

TEST(MagCalGuide, PackingClampsAndRounds)
{
    EXPECT_EQ(magCalPackRotDeg(0.f), 0); EXPECT_EQ(magCalPackRotDeg(45.f), 23); EXPECT_EQ(magCalPackRotDeg(-45.f), -23);     // 22.5 LSB rounds away from zero
    EXPECT_EQ(magCalPackRotDeg(44.f), 22); EXPECT_EQ(magCalPackRotDeg(-44.f), -22);
    EXPECT_EQ(magCalPackRotDeg(180.f), 90); EXPECT_EQ(magCalPackRotDeg(1000.f), 127); EXPECT_EQ(magCalPackRotDeg(-1000.f), -127);
    EXPECT_EQ(magCalPackTarget(-1), MAG_CAL_GUIDE_NONE); EXPECT_EQ(magCalPackTarget(0), 1); EXPECT_EQ(magCalPackTarget(127), 128);
    EXPECT_EQ(magCalUnpackTarget(0), -1); EXPECT_EQ(magCalUnpackTarget(1), 0); EXPECT_EQ(magCalUnpackTarget(128), 127); EXPECT_EQ(magCalUnpackTarget(129), -1); EXPECT_EQ(magCalUnpackTarget(255), -1);
    float u[3] = { 0.f, -1.f, 0.f }; int8_t p[3]; magCalPackUnit(u, p);
    EXPECT_EQ(p[0], 0); EXPECT_EQ(p[1], -127); EXPECT_EQ(p[2], 0);
}

TEST(MagCalCoverage, DataMappingIsRegistered)
{
    EXPECT_STREQ(cISDataMappings::DataName(DID_MAG_CAL_COVERAGE), "DID_MAG_CAL_COVERAGE");
    EXPECT_EQ(cISDataMappings::DataSize(DID_MAG_CAL_COVERAGE), sizeof(mag_cal_coverage_t));
    const map_name_to_info_t *map = cISDataMappings::NameToInfoMap(DID_MAG_CAL_COVERAGE);
    ASSERT_NE(map, nullptr);
    for (const char *name : { "lifecycle", "reason", "quality", "progressPct", "coveragePct", "fitQuality", "sampleCount", "guideRot", "targetCell", "fieldDir", "required", "cells" })
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

TEST(MagCalCoverage, DisplayShowsRelativeGuidanceAndMarkers)
{
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) magCalCellSet(&m, i, 15);
    m.lifecycle = MAG_CAL_LC_COLLECTING; m.mode = MAG_CAL_STATE_MULTI_AXIS; m.flags = MAG_CAL_FLAG_COVERAGE_VALID | MAG_CAL_FLAG_REQUIRED_VALID;
    m.targetCell = magCalPackTarget(3 * 16 + 4); m.guideRot[0] = 17; m.guideRot[1] = -10; m.guideRot[2] = 0;          // roll +34, pitch -20, yaw 0
    float f[3]; magCalCellCenter(5 * 16 + 9, f); magCalPackUnit(f, m.fieldDir);
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    std::string s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("roll  +34"), std::string::npos) << s;      // fixed-width fields: %+4.0f
    EXPECT_NE(s.find("pitch  -20"), std::string::npos) << s;
    EXPECT_NE(s.find("yaw   +0"), std::string::npos) << s;
    EXPECT_NE(s.find("( 39 to go)"), std::string::npos) << s;
    EXPECT_EQ(std::count(s.begin(), s.end(), '*'), 1 + 1);        // one on the map plus one in the legend
    EXPECT_EQ(std::count(s.begin(), s.end(), '@'), 1 + 1);

    // A fit that has not been accepted yet does not stop the guidance while gaps remain (covered in FitNotYetAcceptedStillShowsTheTarget...)
    m.lifecycle = MAG_CAL_LC_FITTING; m.reason = MAG_CAL_REASON_FIT_REJECTED;
    s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("turn:"), std::string::npos) << s;
    EXPECT_NE(s.find("fit not accepted yet"), std::string::npos) << s;

    // Single axis: no target, plain instruction
    mag_cal_coverage_t sa = blank(); sa.lifecycle = MAG_CAL_LC_COLLECTING; sa.mode = MAG_CAL_STATE_SINGLE_AXIS; sa.targetCell = MAG_CAL_GUIDE_NONE; sa.flags = MAG_CAL_FLAG_COVERAGE_VALID;
    s = d.DataToStringMagCalCoverage(sa, hdr);
    EXPECT_NE(s.find("keep turning about the vertical axis"), std::string::npos) << s;
}

TEST(MagCalCoverage, FitNotYetAcceptedStillShowsTheTargetWhileGapsRemain)
{   // Found on hardware: a rejected fit at PARTIAL coverage hid the target behind a wrong 'coverage complete' message.
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) magCalCellSet(&m, i, 15);
    for (int dl = -1; dl <= 1; dl++) magCalCellSet(&m, 2 * 16 + 5 + dl, 0);
    m.lifecycle = MAG_CAL_LC_FITTING; m.mode = MAG_CAL_STATE_MULTI_AXIS; m.reason = MAG_CAL_REASON_FIT_REJECTED; m.progressPct = 99; m.coveragePct = 97;
    m.flags = MAG_CAL_FLAG_COVERAGE_VALID | MAG_CAL_FLAG_REQUIRED_VALID;
    m.targetCell = magCalPackTarget(2 * 16 + 5); m.guideRot[0] = 10; m.guideRot[1] = -5; m.guideRot[2] = 3;
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    std::string s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("turn: roll"), std::string::npos) << s;
    EXPECT_NE(s.find("fit not accepted yet"), std::string::npos) << s;
    EXPECT_EQ(s.find("will not help"), std::string::npos) << "gaps remain: more rotation DOES help\n" << s;
    EXPECT_EQ(s.find("coverage complete"), std::string::npos) << s;
    // Only when nothing is left to cover is it honest to say more rotation will not help
    m.targetCell = MAG_CAL_GUIDE_NONE; m.coveragePct = 100; memset(m.guideRot, 0, 3);
    s = d.DataToStringMagCalCoverage(m, hdr);
    EXPECT_NE(s.find("the map is complete, more rotation will not help"), std::string::npos) << s;
}

TEST(MagCalCoverage, GuidanceNumbersHaveFixedWidthSoTheLineDoesNotShift)
{
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    auto instrLine = [&](int8_t r, int8_t p, int8_t y) {
        mag_cal_coverage_t m = blank(); magCalRequireAll(&m); m.lifecycle = MAG_CAL_LC_COLLECTING; m.mode = MAG_CAL_STATE_MULTI_AXIS; m.flags = MAG_CAL_FLAG_COVERAGE_VALID;
        m.targetCell = magCalPackTarget(5); m.guideRot[0] = r; m.guideRot[1] = p; m.guideRot[2] = y;
        std::string s = d.DataToStringMagCalCoverage(m, hdr); size_t a = s.find("turn:"); size_t b = s.find("to go)", a);
        return s.substr(a, b - a + 6); };
    EXPECT_EQ(instrLine(0, 0, 0).size(), instrLine(-127, 127, -127).size()) << instrLine(0, 0, 0) << " | " << instrLine(-127, 127, -127);
    EXPECT_EQ(instrLine(3, -2, 1).size(), instrLine(60, -60, 60).size());
}

TEST(MagCalCoverage, DisplayLineCountDoesNotDependOnTheContent)
{   // The TUI redraws in place; a different number of lines per update is what made the screen "jump".
    cInertialSenseDisplay d; d.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
    p_data_hdr_t hdr = {}; hdr.id = DID_MAG_CAL_COVERAGE;
    mag_cal_coverage_t quiet = blank();
    mag_cal_coverage_t busy = blank(); magCalRequireAll(&busy);
    busy.lifecycle = MAG_CAL_LC_FITTING; busy.mode = MAG_CAL_STATE_MULTI_AXIS; busy.reason = MAG_CAL_REASON_REGION_FULL; busy.sampleCount = 4000000000u; busy.progressPct = 99; busy.coveragePct = 100;
    busy.fitQuality = 123456.7f; busy.guideRot[0] = -127; busy.guideRot[1] = 127; busy.guideRot[2] = -127; busy.targetCell = magCalPackTarget(17); busy.fieldDir[0] = 127; busy.flags = 0;
    auto lines = [&](const mag_cal_coverage_t &c) { std::string s = d.DataToStringMagCalCoverage(c, hdr); return (int)std::count(s.begin(), s.end(), '\n'); };
    EXPECT_EQ(lines(quiet), lines(busy));
}

} // namespace
