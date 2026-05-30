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

/** @brief Oracle: faithful copy of the original (pre-SN-7919) `renderHdwStatus` body. */
std::string legacyRenderHdwStatusReference(uint32_t hdwStatus)
{
    std::stringstream buff;

#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }
    BIT_MSG(hdwStatus, HDW_STATUS_MOTION_GYR                       , "0x00000001 - Gyro motion detected.");
    BIT_MSG(hdwStatus, HDW_STATUS_MOTION_ACC                       , "0x00000002 - Accelerometer motion detected.");
    BIT_MSG(hdwStatus, HDW_STATUS_IMU_FAULT_REJECT_GYR             , "0x00000004 - IMU gyro fault rejection. A Gyro sensor is divergent and being excluded.");
    BIT_MSG(hdwStatus, HDW_STATUS_IMU_FAULT_REJECT_ACC             , "0x00000008 - IMU accelerometer fault rejection. An accelerometer sensors is divergent and being excluded.");
    BIT_MSG(hdwStatus, HDW_STATUS_GNSS_SATELLITE_RX_VALID          , "0x00000010 - GPS satellite signals are being received (antenna and cable are good).");
    BIT_MSG(hdwStatus, HDW_STATUS_STROBE_IN_EVENT                  , "0x00000020 - Event occurred on strobe input pin.");
    BIT_MSG(hdwStatus, HDW_STATUS_GNSS_TIME_OF_WEEK_VALID          , "0x00000040 - GPS time of week is valid and reported.");
    BIT_MSG(hdwStatus, HDW_STATUS_REFERENCE_IMU_RX                 , "0x00000080 - Reference IMU data being received.");
    BIT_MSG(hdwStatus, HDW_STATUS_SATURATION_GYR                   , "0x00000100 - Sensor saturation on gyro.");
    BIT_MSG(hdwStatus, HDW_STATUS_SATURATION_ACC                   , "0x00000200 - Sensor saturation on accelerometer.");
    BIT_MSG(hdwStatus, HDW_STATUS_SATURATION_MAG                   , "0x00000400 - Sensor saturation on magnetometer.");
    BIT_MSG(hdwStatus, HDW_STATUS_SATURATION_BARO                  , "0x00000800 - Sensor saturation on barometric pressure.");
    BIT_MSG(hdwStatus, HDW_STATUS_SYSTEM_RESET_REQUIRED            , "0x00001000 - System Reset is required for proper function.");
    BIT_MSG(hdwStatus, HDW_STATUS_ERR_GNSS_PPS_NOISE               , "0x00002000 - GPS PPS timepulse signal has noise and occurred too frequently.");
    BIT_MSG(hdwStatus, HDW_STATUS_MAG_RECAL_COMPLETE              , "0x00004000 - Magnetometer recalibration has finished (when INS_STATUS_MAG_RECALIBRATING is unset).");
    BIT_MSG(hdwStatus, HDW_STATUS_FLASH_WRITE_PENDING              , "0x00008000 - System flash write staging or occurring now.");
    BIT_MSG(hdwStatus, HDW_STATUS_ERR_COM_TX_LIMITED              , "0x00010000 - Communications Tx buffer limited.");
    BIT_MSG(hdwStatus, HDW_STATUS_ERR_COM_RX_OVERRUN             , "0x00020000 - Communications Rx buffer overrun.");
    BIT_MSG(hdwStatus, HDW_STATUS_ERR_NO_GNSS_PPS                 , "0x00040000 - GPS PPS timepulse signal has not been received or is in error.");
    BIT_MSG(hdwStatus, HDW_STATUS_GNSS_PPS_TIMESYNC               , "0x00080000 - Time synchronized by GPS PPS.");

    uint8_t parseErrCount = (uint8_t)HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus);
    if (parseErrCount) {
        char b[256];
        std::snprintf(b, sizeof(b), "0x00F00000 - Communications parse errors (%d).", parseErrCount);
        buff << b << std::endl;
    }

    switch (hdwStatus & HDW_STATUS_BIT_MASK) {
        case HDW_STATUS_BIT_RUNNING:  buff << "0x01000000 - (BIT) Built-in self-test is running." << std::endl; break;
        case HDW_STATUS_BIT_PASSED:   buff << "0x02000000 - (BIT) Built-in self-test passed." << std::endl; break;
        case HDW_STATUS_BIT_FAILED:   buff << "0x03000000 - (BIT) Built-in self-test failed." << std::endl; break;
    }

    BIT_MSG(hdwStatus, HDW_STATUS_ERR_TEMPERATURE                 , "0x04000000 - Temperature outside operating range.");
    BIT_MSG(hdwStatus, HDW_STATUS_SPI_INTERFACE_ENABLED          , "0x08000000 - IMX pins G5-G8 are configure for SPI use.");

    switch (hdwStatus & HDW_STATUS_RESET_CAUSE_MASK) {
        case HDW_STATUS_RESET_CAUSE_BACKUP_MODE:    buff << "0x10000000 - Reset from backup mode (low-power state w/ CPU off)." << std::endl; break;
        case HDW_STATUS_RESET_CAUSE_WATCHDOG_FAULT: buff << "0x20000000 - Reset from watchdog fault." << std::endl; break;
        case HDW_STATUS_RESET_CAUSE_SOFT:           buff << "0x30000000 - Reset from software." << std::endl; break;
        case HDW_STATUS_RESET_CAUSE_HDW:            buff << "0x40000000 - Reset from hardware." << std::endl; break;
    }

    BIT_MSG(hdwStatus, HDW_STATUS_FAULT_SYS_CRITICAL             , "0x80000000 - Critical System Fault, CPU error (see DID_SYS_FAULT.status).");

#undef BIT_MSG
    return buff.str();
}

/** @brief Oracle: faithful copy of the original `renderSysStatus` body. */
std::string legacyRenderSysStatusReference(uint32_t sysStatus)
{
    std::stringstream buff;
#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }
    BIT_MSG(sysStatus, SYS_STATUS_TBED3_LEDS_ENABLED            , "0x00000001 - IMX to drive Testbed-3 status LEDs.");
    BIT_MSG(sysStatus, SYS_STATUS_PRIMARY_GNSS_SOURCE_IS_GNSS2  , "0x00000004 - NMEA source is GNSS2.");
#undef BIT_MSG
    return buff.str();
}

/** @brief Oracle: faithful copy of the original `renderGenFaultCode` body. */
std::string legacyRenderGenFaultCodeReference(uint32_t genFault)
{
    std::stringstream buff;
#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }
    BIT_MSG(genFault, GFC_INS_STATE_ORUN_UVW        , "0x00000001 - INS state limit overrun - UVW.");
    BIT_MSG(genFault, GFC_INS_STATE_ORUN_LAT        , "0x00000002 - INS state limit overrun - Latitude.");
    BIT_MSG(genFault, GFC_INS_STATE_ORUN_ALT        , "0x00000004 - INS state limit overrun - Altitude.");
    BIT_MSG(genFault, GFC_UNHANDLED_INTERRUPT       , "0x00000010 - Unhandled interrupt.");
    BIT_MSG(genFault, GFC_GNSS_CRITICAL_FAULT       , "0x00000020 - GNSS receiver critical fault (See the corresponding GPS status fault flags).");
    BIT_MSG(genFault, GFC_GNSS_TX_LIMITED           , "0x00000040 - GNSS Tx limited.");
    BIT_MSG(genFault, GFC_GNSS_RX_OVERRUN           , "0x00000080 - GNSS Rx overrun.");
    BIT_MSG(genFault, GFC_INIT_SENSORS              , "0x00000100 - Fault: sensor initialization.");
    BIT_MSG(genFault, GFC_INIT_SPI                  , "0x00000200 - Fault: SPI bus initialization.");
    BIT_MSG(genFault, GFC_CONFIG_SPI                , "0x00000400 - Fault: SPI configuration.");
    BIT_MSG(genFault, GFC_GNSS1_INIT                , "0x00000800 - Fault: GNSS1 init.");
    BIT_MSG(genFault, GFC_GNSS2_INIT                , "0x00001000 - Fault: GNSS2 init>");
    BIT_MSG(genFault, GFC_FLASH_INVALID_VALUES      , "0x00002000 - Flash failed to load valid values.");
    BIT_MSG(genFault, GFC_FLASH_CHECKSUM_FAILURE    , "0x00004000 - Flash checksum failure.");
    BIT_MSG(genFault, GFC_FLASH_WRITE_FAILURE       , "0x00008000 - Flash write failure.");
    BIT_MSG(genFault, GFC_SYS_FAULT_GENERAL         , "0x00010000 - System Fault: general.");
    BIT_MSG(genFault, GFC_SYS_FAULT_CRITICAL        , "0x00020000 - System Fault: CRITICAL system fault (see DID_SYS_FAULT).");
    BIT_MSG(genFault, GFC_SENSOR_SATURATION         , "0x00040000 - Sensor(s) saturated.");
    BIT_MSG(genFault, GFC_EKF_STATES_INVALID        , "0x00080000 - EKF states invalid.");
    BIT_MSG(genFault, GFC_INIT_IMU                  , "0x00100000 - Fault: IMU initialization.");
    BIT_MSG(genFault, GFC_INIT_BAROMETER            , "0x00200000 - Fault: Barometer initialization.");
    BIT_MSG(genFault, GFC_INIT_MAGNETOMETER         , "0x00400000 - Fault: Magnetometer initialization.");
    BIT_MSG(genFault, GFC_INIT_I2C                  , "0x00800000 - Fault: I2C initialization.");
    BIT_MSG(genFault, GFC_CHIP_ERASE_INVALID        , "0x01000000 - Fault: Chip erase line toggled but did not meet required hold time.");
    BIT_MSG(genFault, GFC_EKF_GNSS_TIME_FAULT       , "0x02000000 - Fault: EKF GPS time fault.");
    BIT_MSG(genFault, GFC_GNSS_RECEIVER_TIME        , "0x04000000 - Fault: GPS receiver time fault.");
    BIT_MSG(genFault, GFC_GNSS_GENERAL_FAULT        , "0x08000000 - Fault: GNSS receiver general fault (See the corresponding GPS status fault flags).");
    BIT_MSG(genFault, GFC_EKF_INPUT_INVALID_IMU     , "0x10000000 - Fault: Invalid IMU input rejected by EKF.");
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

// ---- hdwStatus ----------------------------------------------------------------

TEST(ISStatusDecode, HdwStatus_RoundTrip_EverySingleBit)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("hdwStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), legacyRenderHdwStatusReference(0u));
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderHdwStatusReference(v)) << "bit " << b;
    }
}

TEST(ISStatusDecode, HdwStatus_RoundTrip_BitStateEnum)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("hdwStatus");
    ASSERT_NE(dec, nullptr);
    for (uint32_t st = 0; st <= 3; ++st) {
        const uint32_t v = (st << 24);   // HDW_STATUS_BIT_MASK = 0x03000000
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderHdwStatusReference(v)) << "bit-state " << st;
    }
}

TEST(ISStatusDecode, HdwStatus_RoundTrip_ResetCauseEnum)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("hdwStatus");
    ASSERT_NE(dec, nullptr);
    for (uint32_t rc = 0; rc <= 7; ++rc) {
        const uint32_t v = (rc << 28);   // HDW_STATUS_RESET_CAUSE_MASK = 0x70000000
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderHdwStatusReference(v)) << "reset-cause " << rc;
    }
}

TEST(ISStatusDecode, HdwStatus_RoundTrip_ParseErrorCounts)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("hdwStatus");
    ASSERT_NE(dec, nullptr);
    for (uint32_t c = 0; c <= 15; ++c) {
        const uint32_t v = (c << 20);    // HDW_STATUS_COM_PARSE_ERR_COUNT_MASK = 0x00F00000
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderHdwStatusReference(v)) << "parse-count " << c;
    }
}

TEST(ISStatusDecode, HdwStatus_RoundTrip_AllBits)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("hdwStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0xFFFFFFFFu), legacyRenderHdwStatusReference(0xFFFFFFFFu));
}

TEST(ISStatusDecode, HdwStatus_RoundTrip_RandomSweep)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("hdwStatus");
    ASSERT_NE(dec, nullptr);
    uint32_t s = 0xBADF00Du;
    for (int i = 0; i < 20000; ++i) {
        const uint32_t v = xorshift32(s);
        ASSERT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderHdwStatusReference(v))
            << "iteration " << i << " value 0x" << std::hex << v;
    }
}

TEST(ISStatusDecode, HdwStatus_StructuredDecode)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("hdwStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(dec->errorMask, (uint32_t)HDW_STATUS_ERROR_MASK);

    const status_subfield_t* count = nullptr;
    const status_subfield_t* bit   = nullptr;
    for (const auto& sf : dec->subfields) {
        if (sf.name == "COM parse error count") count = &sf;
        if (sf.name == "Built-in test (BIT)")   bit   = &sf;
    }
    ASSERT_NE(count, nullptr);
    EXPECT_EQ(count->kind, eStatusSubfieldKind::Count);
    EXPECT_EQ(count->mask, (uint32_t)HDW_STATUS_COM_PARSE_ERR_COUNT_MASK);

    ASSERT_NE(bit, nullptr);
    EXPECT_EQ(bit->kind, eStatusSubfieldKind::Enum);
    bool failedIsError = false;
    for (const auto& vl : bit->values)
        if (vl.label == "Failed") failedIsError = vl.isError;
    EXPECT_TRUE(failedIsError);
}

// ---- sysStatus / genFaultCode -------------------------------------------------

TEST(ISStatusDecode, SysStatus_RoundTrip)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("sysStatus");
    ASSERT_NE(dec, nullptr);
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderSysStatusReference(v)) << "bit " << b;
    }
    EXPECT_EQ(dec->errorMask, 0u);   // sysStatus has no error states
}

TEST(ISStatusDecode, GenFaultCode_RoundTrip_EverySingleBit)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("genFaultCode");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), legacyRenderGenFaultCodeReference(0u));
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGenFaultCodeReference(v)) << "bit " << b;
    }
}

TEST(ISStatusDecode, GenFaultCode_RoundTrip_RandomSweep)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("genFaultCode");
    ASSERT_NE(dec, nullptr);
    uint32_t s = 0xFEEDFACEu;
    for (int i = 0; i < 20000; ++i) {
        const uint32_t v = xorshift32(s);
        ASSERT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGenFaultCodeReference(v))
            << "iteration " << i << " value 0x" << std::hex << v;
    }
}

TEST(ISStatusDecode, GenFaultCode_AllBitsAreErrors)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("genFaultCode");
    ASSERT_NE(dec, nullptr);
    EXPECT_FALSE(dec->subfields.empty());
    uint32_t orMask = 0;
    for (const auto& sf : dec->subfields) {
        EXPECT_TRUE(sf.isError) << sf.name;
        orMask |= sf.mask;
    }
    EXPECT_EQ(dec->errorMask, orMask);   // roll-up = OR of all fault bits
}
