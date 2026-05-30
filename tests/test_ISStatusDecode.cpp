/**
 * @file test_ISStatusDecode.cpp
 * @brief Unit tests for the structured status-decode API (SN-7919 / D-53).
 *
 * Two guarantees are tested:
 *  1. Round-trip fidelity: `RenderStatusFromDecode` over the insStatus table reproduces the
 *     ORIGINAL hand-written renderer byte-for-byte. The oracle below is a faithful copy of the
 *     pre-refactor `renderInsStatus` body, so any drift between table and legacy semantics fails
 *     the test.
 *  2. Structured decode: the table exposes the masks/shifts/values/error-classification a
 *     consumer (Logalyzer's status-ribbon chart) needs.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "ISStatusDecode.h"
#include "data_sets.h"

namespace {

/**
 * @brief Oracle: a faithful copy of the original (pre-SN-7919) `renderInsStatus` body, sans the
 *        data_info_t type guard. Equality with `RenderStatusFromDecode` proves the table-driven
 *        renderer reproduces legacy output exactly.
 */
std::string legacyRenderInsStatusReference(uint32_t insStatus)
{
    std::stringstream buff;

#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }

    BIT_MSG(insStatus, INS_STATUS_HDG_ALIGN_COARSE                       ,"0x00000001 - Heading estimate is usable but outside spec (COARSE)");
    BIT_MSG(insStatus, INS_STATUS_VEL_ALIGN_COARSE                       ,"0x00000002 - Velocity estimate is usable but outside spec (COARSE)");
    BIT_MSG(insStatus, INS_STATUS_POS_ALIGN_COARSE                       ,"0x00000004 - Position estimate is usable but outside spec (COARSE)");
    BIT_MSG(insStatus, INS_STATUS_WHEEL_AIDING_VEL                       ,"0x00000008 - Velocity aided by wheel sensor");
    BIT_MSG(insStatus, INS_STATUS_HDG_ALIGN_FINE                         ,"0x00000010 - Heading estimate is within spec (FINE).");
    BIT_MSG(insStatus, INS_STATUS_VEL_ALIGN_FINE                         ,"0x00000020 - Velocity estimate is within spec (FINE)");
    BIT_MSG(insStatus, INS_STATUS_POS_ALIGN_FINE                         ,"0x00000040 - Position estimate is within spec (FINE)");
    BIT_MSG(insStatus, INS_STATUS_GNSS_AIDING_HEADING                    ,"0x00000080 - Heading aided by GPS");
    BIT_MSG(insStatus, INS_STATUS_GNSS_AIDING_POS                        ,"0x00000100 - Position aided by GPS position");
    BIT_MSG(insStatus, INS_STATUS_GNSS_UPDATE_IN_SOLUTION                ,"0x00000200 - GPS update event occurred in solution, potentially causing discontinuity in position path");
    BIT_MSG(insStatus, INS_STATUS_EKF_USING_REFERENCE_IMU               ,"0x00000400 - Reference IMU used in EKF");
    BIT_MSG(insStatus, INS_STATUS_MAG_AIDING_HEADING                    ,"0x00000800 - Heading aided by magnetic heading");
    BIT_MSG(insStatus, INS_STATUS_NAV_MODE                              ,"0x00001000 - Nav Mode - estimating velocity and position.");
    BIT_MSG(insStatus, INS_STATUS_STATIONARY_MODE                       ,"0x00002000 - INS in stationary mode.");
    BIT_MSG(insStatus, INS_STATUS_GNSS_AIDING_VEL                       ,"0x00004000 - Velocity aided by GPS velocity");
    BIT_MSG(insStatus, INS_STATUS_KINEMATIC_CAL_GOOD                    ,"0x00008000 - Vehicle kinematic calibration is good");

    uint32_t insSol = INS_STATUS_SOLUTION(insStatus);
    switch (insSol) {
        case INS_STATUS_SOLUTION_OFF:                     buff << "0x000(0)0000 - System is off" << std::endl; break;
        case INS_STATUS_SOLUTION_ALIGNING:                buff << "0x000(1)0000 - System is in alignment mode" << std::endl; break;
        case INS_STATUS_SOLUTION_NAV:                     buff << "0x000(3)0000 - System is in navigation mode" << std::endl; break;
        case INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE:       buff << "0x000(4)0000 - System is in navigation mode but the attitude uncertainty has exceeded the threshold." << std::endl; break;
        case INS_STATUS_SOLUTION_AHRS:                    buff << "0x000(5)0000 - System is in AHRS mode and solution is good." << std::endl; break;
        case INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE:      buff << "0x000(6)0000 - System is in AHRS mode but the attitude uncertainty has exceeded the threshold." << std::endl; break;
        case INS_STATUS_SOLUTION_VRS:                     buff << "0x000(7)0000 - System is in VRS mode (no earth relative heading) and roll and pitch are good." << std::endl; break;
        case INS_STATUS_SOLUTION_VRS_HIGH_VARIANCE:       buff << "0x000(8)0000 - System is in VRS mode (no earth relative heading) but roll and pitch uncertainty has exceeded the threshold." << std::endl; break;
    }

    BIT_MSG(insStatus, INS_STATUS_RTK_COMPASSING_BASELINE_UNSET         ,"0x00100000 - GPS compassing antenna offsets are not set in flashCfg.");
    BIT_MSG(insStatus, INS_STATUS_RTK_COMPASSING_BASELINE_BAD           ,"0x00200000 - GPS antenna baseline specified in flashCfg and measured by GPS do not match.");
    BIT_MSG(insStatus, INS_STATUS_MAG_RECALIBRATING                     ,"0x00400000 - Magnetometer is being recalibrated.");
    BIT_MSG(insStatus, INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL_OR_NO_CAL ,"0x00800000 - Magnetometer is experiencing interference or calibration is bad.");
    BIT_MSG(insStatus, INS_STATUS_RTK_COMPASSING_VALID                  ,"0x04000000 - RTK compassing heading is accurate and aiding INS heading.");
    BIT_MSG(insStatus, INS_STATUS_RTK_RAW_GNSS_DATA_ERROR               ,"0x08000000 - RTK error: Observations invalid or not received.");

    if (insStatus & INS_STATUS_RTK_ERROR_MASK) {
        uint32_t rtkErr = (insStatus & INS_STATUS_RTK_ERR_BASE_MASK);
        switch (rtkErr) {
            case 0:                                            buff << "0x(0)0000000 - RTK error: NO base position received." << std::endl; break;
            case INS_STATUS_RTK_ERR_BASE_DATA_MISSING:         buff << "0x(1)0000000 - RTK error: Either base observations or antenna position have not been received." << std::endl; break;
            case INS_STATUS_RTK_ERR_BASE_POSITION_MOVING:      buff << "0x(2)0000000 - RTK error: base position moved when it should be stationary." << std::endl; break;
            case INS_STATUS_RTK_ERR_BASE_POSITION_INVALID:     buff << "0x(3)0000000 - RTK error: base position invalid or not surveyed." << std::endl; break;
        }
    }

    BIT_MSG(insStatus, INS_STATUS_RTOS_TASK_PERIOD_OVERRUN              ,"0x40000000 - RTOS task ran longer than allotted period.");
    BIT_MSG(insStatus, INS_STATUS_GENERAL_FAULT                         ,"0x80000000 - General fault (see sys_params_t.genFaultCode).");

#undef BIT_MSG
    return buff.str();
}

/** @brief Deterministic xorshift32 so the sweep is reproducible across runs/platforms. */
uint32_t xorshift32(uint32_t& s)
{
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}

} // namespace

// ---- Round-trip fidelity ------------------------------------------------------

TEST(ISStatusDecode, InsStatus_RoundTrip_Zero)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), legacyRenderInsStatusReference(0u));
}

TEST(ISStatusDecode, InsStatus_RoundTrip_EverySingleBit)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderInsStatusReference(v))
            << "bit " << b;
    }
}

TEST(ISStatusDecode, InsStatus_RoundTrip_EverySolutionState)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);
    for (uint32_t sol = 0; sol <= 0xF; ++sol) {
        const uint32_t v = (sol << INS_STATUS_SOLUTION_OFFSET);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderInsStatusReference(v))
            << "solution " << sol;
    }
}

TEST(ISStatusDecode, InsStatus_RoundTrip_RtkErrorCombos)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);
    // Including the hybrid case: RAW_GNSS_DATA_ERROR set but base bits clear -> "NO base position".
    const uint32_t cases[] = {
        INS_STATUS_RTK_RAW_GNSS_DATA_ERROR,
        INS_STATUS_RTK_ERR_BASE_DATA_MISSING,
        INS_STATUS_RTK_ERR_BASE_POSITION_MOVING,
        INS_STATUS_RTK_ERR_BASE_POSITION_INVALID,
        INS_STATUS_RTK_RAW_GNSS_DATA_ERROR | INS_STATUS_RTK_ERR_BASE_POSITION_INVALID,
    };
    for (uint32_t v : cases) {
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderInsStatusReference(v))
            << "rtk combo 0x" << std::hex << v;
    }
}

TEST(ISStatusDecode, InsStatus_RoundTrip_AllBits)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0xFFFFFFFFu),
              legacyRenderInsStatusReference(0xFFFFFFFFu));
}

TEST(ISStatusDecode, InsStatus_RoundTrip_RandomSweep)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);
    uint32_t s = 0xC0FFEEu;
    for (int i = 0; i < 20000; ++i) {
        const uint32_t v = xorshift32(s);
        ASSERT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderInsStatusReference(v))
            << "iteration " << i << " value 0x" << std::hex << v;
    }
}

// ---- Structured decode --------------------------------------------------------

TEST(ISStatusDecode, InsStatus_TableMetadata)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(dec->fieldName, "insStatus");
    EXPECT_EQ(dec->errorMask, (uint32_t)INS_STATUS_ERROR_MASK);
    EXPECT_FALSE(dec->subfields.empty());

    // GetStatusDecode(did, name) is DID-agnostic in v1 and must agree with the by-field lookup.
    EXPECT_EQ(GetStatusDecode(DID_INS_2, "insStatus"), dec);
    EXPECT_EQ(GetStatusDecode(DID_SYS_PARAMS, "insStatus"), dec);
    EXPECT_EQ(GetStatusDecodeByField("nonexistentField"), nullptr);
}

TEST(ISStatusDecode, InsStatus_SolutionSubfieldDecodes)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);

    const status_subfield_t* sol = nullptr;
    for (const auto& sf : dec->subfields)
        if (sf.name == "Solution mode") { sol = &sf; break; }
    ASSERT_NE(sol, nullptr);
    EXPECT_EQ(sol->kind, eStatusSubfieldKind::Enum);
    EXPECT_EQ(sol->mask, (uint32_t)INS_STATUS_SOLUTION_MASK);
    EXPECT_EQ(sol->shift, (uint32_t)INS_STATUS_SOLUTION_OFFSET);

    // The decoded post-shift value of a NAV solution must map to the "Nav" state.
    const uint32_t navVal = (uint32_t)INS_STATUS_SOLUTION_NAV;
    bool found = false;
    for (const auto& vl : sol->values)
        if (vl.value == navVal) { EXPECT_EQ(vl.label, "Nav"); EXPECT_FALSE(vl.isError); found = true; }
    EXPECT_TRUE(found);
}

TEST(ISStatusDecode, InsStatus_RtkBaseErrorIsHybridGated)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);

    const status_subfield_t* rtk = nullptr;
    for (const auto& sf : dec->subfields)
        if (sf.name == "RTK base error") { rtk = &sf; break; }
    ASSERT_NE(rtk, nullptr);
    EXPECT_EQ(rtk->kind, eStatusSubfieldKind::Enum);
    EXPECT_EQ(rtk->gateMask, (uint32_t)INS_STATUS_RTK_ERROR_MASK);
    EXPECT_TRUE(rtk->isError);
    // All four base-error states are errors.
    for (const auto& vl : rtk->values) EXPECT_TRUE(vl.isError);
}

TEST(ISStatusDecode, InsStatus_ErrorRollup)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("insStatus");
    ASSERT_NE(dec, nullptr);
    // A pure-info value (e.g. NAV mode + fine alignment) is NOT an error.
    const uint32_t okVal = INS_STATUS_NAV_MODE | INS_STATUS_HDG_ALIGN_FINE |
                           ((uint32_t)INS_STATUS_SOLUTION_NAV << INS_STATUS_SOLUTION_OFFSET);
    EXPECT_EQ(okVal & dec->errorMask, 0u);
    // A general fault trips the error roll-up.
    EXPECT_NE((uint32_t)(INS_STATUS_GENERAL_FAULT) & dec->errorMask, 0u);
}
