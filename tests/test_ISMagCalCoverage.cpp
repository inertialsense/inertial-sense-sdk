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
    EXPECT_EQ(sizeof(mag_cal_coverage_t), 104u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, timeOfWeekMs), 0u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, cellScheme), 4u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, flags), 11u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, fitQuality), 12u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, sampleCount), 16u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, suggestDir), 20u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, required), 24u);
    EXPECT_EQ(offsetof(mag_cal_coverage_t, cells), 40u);
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

TEST(MagCalCoverage, SuggestDirectionPointsAtLargestMissingRegion)
{
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) magCalCellSet(&m, i, 15);
    magCalSuggestDirection(&m);
    EXPECT_EQ(m.suggestDir[0], 0); EXPECT_EQ(m.suggestDir[1], 0); EXPECT_EQ(m.suggestDir[2], 0);   // nothing missing

    // Open a 3x3 hole centred on z-band 2, longitude sector 5, plus one isolated missing cell elsewhere
    for (int dz = -1; dz <= 1; dz++) for (int dl = -1; dl <= 1; dl++) magCalCellSet(&m, (2 + dz) * 16 + (5 + dl), 0);
    magCalCellSet(&m, 6 * 16 + 12, 0);
    magCalSuggestDirection(&m);
    float want[3]; magCalCellCenter(2 * 16 + 5, want);
    EXPECT_NEAR(m.suggestDir[0] / 127.0f, want[0], 0.02f); EXPECT_NEAR(m.suggestDir[1] / 127.0f, want[1], 0.02f); EXPECT_NEAR(m.suggestDir[2] / 127.0f, want[2], 0.02f);
}

TEST(MagCalCoverage, SuggestDirectionWrapsLongitude)
{
    mag_cal_coverage_t m = blank(); magCalRequireAll(&m);
    for (unsigned i = 0; i < MAG_CAL_CELL_COUNT; i++) magCalCellSet(&m, i, 15);
    for (int dz = -1; dz <= 1; dz++) for (int dl = -1; dl <= 1; dl++) magCalCellSet(&m, (4 + dz) * 16 + ((0 + dl + 16) % 16), 0);   // hole straddling the +-180 deg seam
    magCalSuggestDirection(&m);
    float want[3]; magCalCellCenter(4 * 16 + 0, want);
    EXPECT_NEAR(m.suggestDir[0] / 127.0f, want[0], 0.02f); EXPECT_NEAR(m.suggestDir[1] / 127.0f, want[1], 0.02f);
}

TEST(MagCalCoverage, DataMappingIsRegistered)
{
    EXPECT_STREQ(cISDataMappings::DataName(DID_MAG_CAL_COVERAGE), "DID_MAG_CAL_COVERAGE");
    EXPECT_EQ(cISDataMappings::DataSize(DID_MAG_CAL_COVERAGE), sizeof(mag_cal_coverage_t));
    const map_name_to_info_t *map = cISDataMappings::NameToInfoMap(DID_MAG_CAL_COVERAGE);
    ASSERT_NE(map, nullptr);
    for (const char *name : { "lifecycle", "reason", "quality", "progressPct", "coveragePct", "fitQuality", "sampleCount", "suggestDir", "required", "cells" })
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
    EXPECT_NE(s.find("fit rejected"), std::string::npos);
    EXPECT_NE(s.find("99%"), std::string::npos);
    EXPECT_NE(s.find(": o O #"), std::string::npos) << s;           // cells 0..3 glyphs: thin, ok, good, full
    EXPECT_EQ(std::count(s.begin(), s.end(), '\n'), 1 + MAG_CAL_CELL_Z_BANDS + 1);   // summary + header + 8 rows
}

} // namespace
