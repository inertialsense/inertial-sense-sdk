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
#include <vector>

#include "ISStatusDecode.h"
#include "data_sets.h"
#include "../src/data_sets.h"

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
    BIT_MSG(insStatus, INS_STATUS_GNSS_AIDING_HEADING                    ,"0x00000080 - Heading aided by GNSS");
    BIT_MSG(insStatus, INS_STATUS_GNSS_AIDING_POS                        ,"0x00000100 - Position aided by GNSS position");
    BIT_MSG(insStatus, INS_STATUS_GNSS_UPDATE_IN_SOLUTION                ,"0x00000200 - GNSS update event occurred in solution, potentially causing discontinuity in position path");
    BIT_MSG(insStatus, INS_STATUS_EKF_USING_REFERENCE_IMU               ,"0x00000400 - Reference IMU used in EKF");
    BIT_MSG(insStatus, INS_STATUS_MAG_AIDING_HEADING                    ,"0x00000800 - Heading aided by magnetic heading");
    BIT_MSG(insStatus, INS_STATUS_NAV_MODE                              ,"0x00001000 - Nav Mode - estimating velocity and position.");
    BIT_MSG(insStatus, INS_STATUS_STATIONARY_MODE                       ,"0x00002000 - INS in stationary mode.");
    BIT_MSG(insStatus, INS_STATUS_GNSS_AIDING_VEL                       ,"0x00004000 - Velocity aided by GNSS velocity");
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

    BIT_MSG(insStatus, INS_STATUS_RTK_COMPASSING_BASELINE_UNSET         ,"0x00100000 - GNSS compassing antenna offsets are not set in flashCfg.");
    BIT_MSG(insStatus, INS_STATUS_RTK_COMPASSING_BASELINE_BAD           ,"0x00200000 - GNSS antenna baseline specified in flashCfg and measured by GNSS do not match.");
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
    BIT_MSG(hdwStatus, HDW_STATUS_GNSS_SATELLITE_RX_VALID          , "0x00000010 - GNSS satellite signals are being received (antenna and cable are good).");
    BIT_MSG(hdwStatus, HDW_STATUS_STROBE_IN_EVENT                  , "0x00000020 - Event occurred on strobe input pin.");
    BIT_MSG(hdwStatus, HDW_STATUS_GNSS_TIME_OF_WEEK_VALID          , "0x00000040 - GNSS time of week is valid and reported.");
    BIT_MSG(hdwStatus, HDW_STATUS_REFERENCE_IMU_RX                 , "0x00000080 - Reference IMU data being received.");
    BIT_MSG(hdwStatus, HDW_STATUS_SATURATION_GYR                   , "0x00000100 - Sensor saturation on gyro.");
    BIT_MSG(hdwStatus, HDW_STATUS_SATURATION_ACC                   , "0x00000200 - Sensor saturation on accelerometer.");
    BIT_MSG(hdwStatus, HDW_STATUS_SATURATION_MAG                   , "0x00000400 - Sensor saturation on magnetometer.");
    BIT_MSG(hdwStatus, HDW_STATUS_SATURATION_BARO                  , "0x00000800 - Sensor saturation on barometric pressure.");
    BIT_MSG(hdwStatus, HDW_STATUS_SYSTEM_RESET_REQUIRED            , "0x00001000 - System Reset is required for proper function.");
    BIT_MSG(hdwStatus, HDW_STATUS_ERR_GNSS_PPS_NOISE               , "0x00002000 - GNSS PPS timepulse signal has noise and occurred too frequently.");
    BIT_MSG(hdwStatus, HDW_STATUS_MAG_RECAL_COMPLETE              , "0x00004000 - Magnetometer recalibration has finished (when INS_STATUS_MAG_RECALIBRATING is unset).");
    BIT_MSG(hdwStatus, HDW_STATUS_FLASH_WRITE_PENDING              , "0x00008000 - System flash write staging or occurring now.");
    BIT_MSG(hdwStatus, HDW_STATUS_ERR_COM_TX_LIMITED              , "0x00010000 - Communications Tx buffer limited.");
    BIT_MSG(hdwStatus, HDW_STATUS_ERR_COM_RX_OVERRUN             , "0x00020000 - Communications Rx buffer overrun.");
    BIT_MSG(hdwStatus, HDW_STATUS_ERR_NO_GNSS_PPS                 , "0x00040000 - GNSS PPS timepulse signal has not been received or is in error.");
    BIT_MSG(hdwStatus, HDW_STATUS_GNSS_PPS_TIMESYNC               , "0x00080000 - Time synchronized by GNSS PPS.");

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
    BIT_MSG(genFault, GFC_GNSS_CRITICAL_FAULT       , "0x00000020 - GNSS receiver critical fault (See the corresponding GNSS status fault flags).");
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
    BIT_MSG(genFault, GFC_EKF_GNSS_TIME_FAULT       , "0x02000000 - Fault: EKF GNSS time fault.");
    BIT_MSG(genFault, GFC_GNSS_RECEIVER_TIME        , "0x04000000 - Fault: GNSS receiver time fault.");
    BIT_MSG(genFault, GFC_GNSS_GENERAL_FAULT        , "0x08000000 - Fault: GNSS receiver general fault (See the corresponding GNSS status fault flags).");
    BIT_MSG(genFault, GFC_EKF_INPUT_INVALID_IMU     , "0x10000000 - Fault: Invalid IMU input rejected by EKF.");
#undef BIT_MSG
    return buff.str();
}

/** @brief Oracle: faithful copy of the original `renderGnssStatusBits` body. */
std::string legacyRenderGnssStatusBitsReference(uint32_t gpsStatusBits)
{
    std::stringstream buff;
#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }

    uint8_t satCount = (gpsStatusBits & GNSS_STATUS_NUM_SATS_USED_MASK);
    {
        char b[256];
        std::snprintf(b, sizeof(b), "0x000000%02X - %d satellites used in solution (deprecated)", satCount, satCount);
        buff << b << std::endl;
    }

    switch (gpsStatusBits & GNSS_STATUS_FIX_MASK) {
        case GNSS_STATUS_FIX_NONE                : buff << "0x00000000 - No GNSS" << std::endl; break;
        case GNSS_STATUS_FIX_DEAD_RECKONING_ONLY : buff << "0x00000100 - GNSS Dead Reckoning Only" << std::endl; break;
        case GNSS_STATUS_FIX_2D                  : buff << "0x00000200 - 2D Fix" << std::endl; break;
        case GNSS_STATUS_FIX_3D                  : buff << "0x00000300 - 3D Fix" << std::endl; break;
        case GNSS_STATUS_FIX_GNSS_PLUS_DEAD_RECK  : buff << "0x00000400 - 3D Fix + Dead Reckoning" << std::endl; break;
        case GNSS_STATUS_FIX_TIME_ONLY           : buff << "0x00000500 - Time-Only Fix" << std::endl; break;
        case GNSS_STATUS_FIX_REF_LLA             : buff << "0x00000600 - Usign Reference LLA" << std::endl; break;
        case GNSS_STATUS_FIX_UNUSED2             : buff << "0x00000700 - << UNUSED >>" << std::endl; break;
        case GNSS_STATUS_FIX_DGPS                : buff << "0x00000800 - Using DGPS" << std::endl; break;
        case GNSS_STATUS_FIX_SBAS                : buff << "0x00000900 - Using SBAS" << std::endl; break;
        case GNSS_STATUS_FIX_RTK_SINGLE          : buff << "0x00000A00 - RTK Single" << std::endl; break;
        case GNSS_STATUS_FIX_RTK_FLOAT           : buff << "0x00000B00 - RTK Float" << std::endl; break;
        case GNSS_STATUS_FIX_RTK_FIX             : buff << "0x00000C00 - RTK Fix" << std::endl; break;
    }

    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_FIX_OK                          , "0x00010000 - within limits (e.g. DOP & accuracy)");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_DGPS_USED                       , "0x00020000 - Differential GPS (DGPS) used.");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_RTK_FIX_AND_HOLD                , "0x00040000 - RTK feedback on the integer solutions to drive the float biases towards the resolved integers");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_UNUSED_1                        , "0x00080000 - << UNUSED >>");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_ENABLED       , "0x00100000 - GNSS1 RTK precision positioning mode enabled");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_STATIC_MODE                     , "0x00200000 - Static mode");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_ENABLED        , "0x00400000 - GNSS2 RTK moving base mode enabled");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS1_RTK_RAW_GNSS_DATA_ERROR     , "0x00800000 - GNSS1 RTK error: observations or ephemeris are invalid or not received (i.e. RTK differential corrections)");

    uint32_t rtkError = (gpsStatusBits & GNSS_STATUS_FLAGS_ERROR_MASK);
    switch (rtkError) {
        case GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_DATA_MISSING        : buff << "0x01000000 - GNSS1 RTK error: Either base observations or antenna position have not been received." << std::endl; break;
        case GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_MOVING     : buff << "0x02000000 - GNSS1 RTK error: base position moved when it should be stationary" << std::endl; break;
        case GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_INVALID    : buff << "0x03000000 - GNSS1 RTK error: base position is invalid or not surveyed well" << std::endl; break;
    }

    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_VALID         , "0x04000000 - GNSS1 RTK precision position and carrier phase range solution with fixed ambiguities.");
    if (gpsStatusBits & GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_MASK) {
        BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_VALID          , "0x08000000 - GNSS2 RTK moving base heading valid and available in DID_GNSS2_RTK_CMP_REL.");
        BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_BAD   , "0x00002000 - GNSS2 RTK Compassing Baseline distance is invalid");
        BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_UNSET , "0x00004000 - GNSS2 RTK Compassing Baseline distance is unset (must be > 0)");
    }
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS_NMEA_DATA                   , "0x00008000 - Data from NMEA message. GNSS velocity is NED (not ECEF).");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_GNSS_PPS_TIMESYNC                , "0x10000000 - Time is synchronized by GNSS PPS.");

    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_RTK_COV_ECEF_PACKED_VALID       , "0x20000000 - RTK ECEF covariance matrix is valid (if provided by RTK REL message).");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_UNUSED_3                        , "0x40000000 - <<UNUSED>>");
    BIT_MSG(gpsStatusBits, GNSS_STATUS_FLAGS_UNUSED_4                        , "0x80000000 - <<UNUSED>>");
#undef BIT_MSG
    return buff.str();
}

/** @brief Oracle: faithful copy of the original `renderGpxStatus_status` body. */
std::string legacyRenderGpxStatusReference(uint32_t status)
{
    std::stringstream buff;
#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }
    BIT_MSG(status, GPX_STATUS_COM_PARSE_ERR_COUNT_MASK     , "0x0000000F - Communications parse error count");
    BIT_MSG(status, GPX_STATUS_COM0_RX_TRAFFIC_DETECTED     , "0x00000010 - COM0 RX traffic detected in last 30 seconds.");
    BIT_MSG(status, GPX_STATUS_COM1_RX_TRAFFIC_DETECTED     , "0x00000020 - COM1 RX traffic detected in last 30 seconds.");
    BIT_MSG(status, GPX_STATUS_COM2_RX_TRAFFIC_DETECTED     , "0x00000040 - COM2 RX traffic detected in last 30 seconds.");
    BIT_MSG(status, GPX_STATUS_USB_RX_TRAFFIC_DETECTED      , "0x00000080 - USB RX traffic detected in last 30 seconds.");
    BIT_MSG(status, GPX_STATUS_UPDATE_CONFIRMED             , "0x00000100 - Update confirmed.");
    BIT_MSG(status, GPX_STATUS_FAULT_RTK_QUEUE_LIMITED      , "0x00010000 - RTK buffer overflow.");
    BIT_MSG(status, GPX_STATUS_FAULT_GNSS_RCVR_TIME         , "0x00100000 - GNSS receiver time fault");
    BIT_MSG(status, GPX_STATUS_FAULT_RTOS_TASK_PERIOD_OVERRUN, "0x00200000 - RTOS task period overrun");

    BIT_MSG(status, GPX_STATUS_FAULT_DMA                    , "0x00800000 - DMA fault");
    uint32_t fatalStatus = ((status & GPX_STATUS_FATAL_MASK) >> GPX_STATUS_FATAL_OFFSET);
    switch (fatalStatus) {
        case GPX_STATUS_FATAL_RESET_LOW_POW:        buff << "0x01000000 - Reset from low power" << std::endl; break;
        case GPX_STATUS_FATAL_RESET_BROWN:          buff << "0x02000000 - Reset from brown out" << std::endl; break;
        case GPX_STATUS_FATAL_RESET_WATCHDOG:       buff << "0x03000000 - Reset from watchdog" << std::endl; break;
        case GPX_STATUS_FATAL_CPU_EXCEPTION:        buff << "0x04000000 - CPU exception" << std::endl; break;
        case GPX_STATUS_FATAL_UNHANDLED_INTERRUPT:  buff << "0x05000000 - Unhandled interrupt" << std::endl; break;
        case GPX_STATUS_FATAL_STACK_OVERFLOW:       buff << "0x06000000 - Stack overflow" << std::endl; break;
        case GPX_STATUS_FATAL_KERNEL_OOPS:          buff << "0x07000000 - Kernel oops" << std::endl; break;
        case GPX_STATUS_FATAL_KERNEL_PANIC:         buff << "0x08000000 - Kernel panic" << std::endl; break;
        case GPX_STATUS_FATAL_UNALIGNED_ACCESS:     buff << "0x09000000 - Unaligned access" << std::endl; break;
        case GPX_STATUS_FATAL_MEMORY_ERROR:         buff << "0x0A000000 - Memory error" << std::endl; break;
        case GPX_STATUS_FATAL_BUS_ERROR:            buff << "0x0B000000 - Bus error" << std::endl; break;
        case GPX_STATUS_FATAL_USAGE_ERROR:          buff << "0x0C000000 - Usage error" << std::endl; break;
        case GPX_STATUS_FATAL_DIV_ZERO:             buff << "0x0D000000 - Division by zero" << std::endl; break;
        case GPX_STATUS_FATAL_SER0_REINIT:          buff << "0x0E000000 - SER0 reinit" << std::endl; break;
        case GPX_STATUS_FATAL_UNKNOWN:              buff << "0x1F000000 - Unknown" << std::endl; break;
    }
    BIT_MSG(status, GPX_STATUS_FAULT_RP                     , "0x20000000 - RP fault");
#undef BIT_MSG
    return buff.str();
}

/** @brief Oracle: faithful copy of the original `renderGpxStatus_hdwStatus` body. */
std::string legacyRenderGpxHdwStatusReference(uint32_t hdwStatus)
{
    std::stringstream buff;
#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_GNSS1_SATELLITE_RX            , "0x00000001 - GNSS1 satellite signals are being received (antenna and cable are good)");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_GNSS2_SATELLITE_RX            , "0x00000002 - GNSS2 satellite signals are being received (antenna and cable are good)");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_GNSS1_TIME_OF_WEEK_VALID      , "0x00000004 - GNSS time of week is valid and reported.  Otherwise the timeOfWeek is local system time.");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_GNSS2_TIME_OF_WEEK_VALID      , "0x00000008 - GNSS time of week is valid and reported.  Otherwise the timeOfWeek is local system time.");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_FAULT_GNSS1_INIT              , "0x00000080 - Failed to communicate or setup GNSS receiver 1");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_FAULT_GNSS2_INIT              , "0x00000800 - Failed to communicate or setup GNSS receiver 2");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_GNSS_FW_UPDATE_REQUIRED       , "0x00001000 - GNSS is faulting firmware update REQUIRED");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_LED_ENABLED                   , "0x00002000 - Enables LED in Manufacturing TBed");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_SYSTEM_RESET_REQUIRED         , "0x00004000 - System Reset is Required for proper function");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_FLASH_WRITE_PENDING           , "0x00008000 - System flash write staging or occuring now.");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_COM_TX_LIMITED            , "0x00010000 - Communications Tx buffer limited");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_COM_RX_OVERRUN            , "0x00020000 - Communications Rx buffer overrun");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_NO_GNSS1_PPS              , "0x00040000 - GNSS1 PPS timepulse signal has not been received or is in error");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_NO_GNSS2_PPS              , "0x00080000 - GNSS2 PPS timepulse signal has not been received or is in error");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_LOW_CNO_GNSS1             , "0x00100000 - GNSS1 signal strength low (<20)");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_LOW_CNO_GNSS2             , "0x00200000 - GNSS2 signal strength low (<20)");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_CNO_GNSS1_IR             , "0x00400000 - GNSS1 signal irregular. High Cno standard deviation over 5 second period detected.");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_CNO_GNSS2_IR             , "0x00800000 - GNSS2 signal irregular. High Cno standard deviation over 5 second period detected.");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_BIT_RUNNING                   , "0x01000000 - (BIT) Built-in self-test running");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_BIT_PASSED                    , "0x02000000 - (BIT) Built-in self-test passed");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_BIT_FAULT                     , "0x03000000 - (BIT) Built-in self-test failure");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_ERR_TEMPERATURE               , "0x04000000 - Temperature outside spec'd operating range");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_GNSS_PPS_TIMESYNC             , "0x08000000 - Time synchronized by GNSS PPS");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_RESET_CAUSE_BACKUP_MODE       , "0x10000000 - Reset from Backup mode (low-power state w/ CPU off)");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_RESET_CAUSE_SOFT              , "0x20000000 - Reset from Software");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_RESET_CAUSE_HDW               , "0x40000000 - Reset from Hardware (NRST pin low)");
    BIT_MSG(hdwStatus, GPX_HDW_STATUS_FAULT_SYS_CRITICAL            , "0x80000000 - Critical System Fault, CPU error.");
#undef BIT_MSG
    return buff.str();
}

// --- GPX-GNSS scalar-enum oracles (faithful copies of the originals) --------------------------

std::string legacyRenderGnssLastResetCauseReference(uint8_t v)
{
    static const char* rstReasons[] = {
        "Power On", "Watchdog", "ErrOpCode", "ErrorOpCode_FwUp",
        "ErrorOpCode_init", "UserRequested", "FWUpdate", "SysCmd",
        "InitTimeout", "Status5", "StatusNot0", "flashUpdate", "RTKEphMissing"
    };
    if (v < cxdRst_Max) return std::string(rstReasons[v]);
    return "";
}

std::string legacyRenderGnssInitStateReference(uint8_t v)
{
    static const char* initStates[] = {
        "Bootup", "UartSetting", "UartWait", "UartDone",
        "VersionCheck", "StopPos", "SetL5", "SetSats",
        "SetSatLimits", "SetOutput", "SetAlgo", "SetPeriod",
        "SetRtcmMsgs", "SetRtcmTimeMode", "SetPinningMode", "SetVelocitySmoothing",
        "SetAltituedSmoothing", "SetEphmOutputPeriod", "StartPos", "Done"
    };
    if (v <= 19) return std::string(initStates[v]);
    return "";
}

std::string legacyRenderGnssRunStateReference(uint8_t v)
{
    static const char* runStates[] = {
        "Reset", "Initializing", "Running", "Passthrough",
        "FwUpdate Init", "FwUpdate", "Error", "Shutdown", "ReInit", "Hard Reset"
    };
    if (v <= kHardReset) return std::string(runStates[v]);
    return "";
}

std::string legacyRenderGnssFwUpdateStateReference(uint8_t v)
{
    static const char* fwStates[] = {
        "LockoutWait", "ResetSet", "ResetWait", "StartSet", "StartWait",
        "BootModeSet", "BootModeWait", "BaudSet", "BaudWait", "BaudFinish",
        "InjectWait", "InjectFinish", "ProgramExecutionWait", "ProgramExecutionFinish",
        "WriteNvmWait", "WriteNvmFinish", "Done",
    };
    if (v < 17) return std::string(fwStates[v]);   // SN-7919: corrected bound (array size), was cxdRst_Max
    return "";
}

/** @brief Oracle: faithful copy of the original `renderImxHdwBitStatus` body. */
std::string legacyRenderImxHdwBitReference(uint32_t hdwBitStatus)
{
    std::stringstream buff;
#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }
    BIT_MSG(hdwBitStatus, HDW_BIT_PASSED_ALL                      ,"0x00000001 - Passed all tests");
    BIT_MSG(hdwBitStatus, HDW_BIT_PASSED_NO_GNSS                   ,"0x00000002 - Passed without valid GNSS signal");
    if (HDW_BIT_MODE(hdwBitStatus)) {
        buff << "0x000000" << std::hex << (hdwBitStatus & HDW_BIT_MODE_MASK) << std::dec
             << " - BIT mode: " << HDW_BIT_MODE(hdwBitStatus) << std::endl;
    }
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_NOISE_PQR                 ,"0x00000100 - FAULT: Gyro noise");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_NOISE_ACC                 ,"0x00000200 - FAULT: Accelerometer noise");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_MAGNETOMETER              ,"0x00000400 - FAULT: Magnetometer");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_BAROMETER                 ,"0x00000800 - FAULT: Barometer");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_GNSS_NO_COM                ,"0x00001000 - FAULT: No GNSS serial communications");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_GNSS_POOR_CNO              ,"0x00002000 - FAULT: Poor GNSS signal strength");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_GNSS_POOR_ACCURACY         ,"0x00004000 - FAULT: GNSS poor accuracy");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_GNSS_NOISE                 ,"0x00008000 - FAULT: GNSS noise");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_IMU_FAULT_REJECTION       ,"0x00010000 - FAULT: IMU fault rejection failure");
    BIT_MSG(hdwBitStatus, HDW_BIT_FAULT_INCORRECT_HARDWARE_TYPE   ,"0x01000000 - FAULT: Hardware type does not match firmware");
#undef BIT_MSG
    return buff.str();
}

/** @brief Oracle: faithful copy of the original `renderImxCalBitStatus` body. */
std::string legacyRenderImxCalBitReference(uint32_t calBitStatus)
{
    std::stringstream buff;
#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }
    BIT_MSG(calBitStatus, CAL_BIT_PASSED_ALL                      ,"0x00000001 - Passed all calibration tests");
    if (CAL_BIT_MODE(calBitStatus)) {
        buff << "0x000000" << std::hex << (calBitStatus & CAL_BIT_MODE_MASK) << std::dec
             << " - CAL BIT mode: " << CAL_BIT_MODE(calBitStatus) << std::endl;
    }
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_EMPTY                ,"0x00000100 - FAULT: Temperature calibration not present");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_TSPAN                ,"0x00000200 - FAULT: Temperature calibration range inadequate");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_INCONSISTENT         ,"0x00000400 - FAULT: Temperature calibration inconsistent");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_CORRUPT              ,"0x00000800 - FAULT: Temperature calibration corrupt");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_PQR_BIAS             ,"0x00001000 - FAULT: Gyro bias temp cal");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_PQR_SLOPE            ,"0x00002000 - FAULT: Gyro slope temp cal");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_PQR_LIN              ,"0x00004000 - FAULT: Gyro linearity temp cal");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_ACC_BIAS             ,"0x00008000 - FAULT: Accel bias temp cal");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_ACC_SLOPE            ,"0x00010000 - FAULT: Accel slope temp cal");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_TCAL_ACC_LIN              ,"0x00020000 - FAULT: Accel linearity temp cal");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_CAL_SERIAL_NUM            ,"0x00040000 - FAULT: Calibration serial number mismatch");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_MCAL_MAG_INVALID          ,"0x00080000 - FAULT: Magnetometer cross-axis alignment invalid");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_MCAL_EMPTY                ,"0x00100000 - FAULT: Motion calibration not present");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_MCAL_IMU_INVALID          ,"0x00200000 - FAULT: IMU cross-axis alignment invalid");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_MOTION_PQR                ,"0x00400000 - FAULT: Motion detected on gyros");
    BIT_MSG(calBitStatus, CAL_BIT_FAULT_MOTION_ACC                ,"0x00800000 - FAULT: Motion detected on accelerometers");
    BIT_MSG(calBitStatus, CAL_BIT_NOTICE_IMU1_PQR_BIAS            ,"0x01000000 - NOTICE: IMU 1 gyro bias offset detected");
    BIT_MSG(calBitStatus, CAL_BIT_NOTICE_IMU2_PQR_BIAS            ,"0x02000000 - NOTICE: IMU 2 gyro bias offset detected");
    BIT_MSG(calBitStatus, CAL_BIT_NOTICE_IMU1_ACC_BIAS            ,"0x10000000 - NOTICE: IMU 1 accel bias offset detected");
    BIT_MSG(calBitStatus, CAL_BIT_NOTICE_IMU2_ACC_BIAS            ,"0x20000000 - NOTICE: IMU 2 accel bias offset detected");
#undef BIT_MSG
    return buff.str();
}

/** @brief Oracle: faithful copy of the original `renderGpxBitResults` body. */
std::string legacyRenderGpxBitResultsReference(uint32_t results)
{
    std::stringstream buff;
#define BIT_MSG(_F_, _B_, _M_)    if (_F_ & _B_) { buff << _M_ << std::endl; }
    BIT_MSG(results, GPXBit_resultsBit_PPS1      ,"0x01 - PPS1 test passed");
    BIT_MSG(results, GPXBit_resultsBit_PPS2      ,"0x02 - PPS2 test passed");
    BIT_MSG(results, GPXBit_resultsBit_UART      ,"0x04 - UART test passed");
    BIT_MSG(results, GPXBit_resultsBit_IO        ,"0x08 - IO test passed");
    BIT_MSG(results, GPXBit_resultsBit_GNSS       ,"0x10 - GNSS test passed");
    BIT_MSG(results, GPXBit_resultsBit_FINISHED  ,"0x20 - Test finished");
    BIT_MSG(results, GPXBit_resultsBit_CANCELED  ,"0x40 - Test canceled");
    BIT_MSG(results, GPXBit_resultsBit_ERROR     ,"0x80 - Test error");
#undef BIT_MSG
    return buff.str();
}

/** @brief Oracle: faithful copy of the original `renderGpxBitState` body. */
std::string legacyRenderGpxBitStateReference(uint8_t state)
{
    std::stringstream buff;
    switch (state) {
        case 0: buff << "NOT_RUNNING" << std::endl; break;
        case 1: buff << "MANUF_INIT" << std::endl; break;
        case 2: buff << "MANUF_BLINK" << std::endl; break;
        case 3: buff << "MANUF_UART" << std::endl; break;
        case 4: buff << "MANUF_IO" << std::endl; break;
        case 5: buff << "MANUF_PPS" << std::endl; break;
        case 6: buff << "MANUF_GPS" << std::endl; break;
        case 7: buff << "MANUF_REPORT" << std::endl; break;
        default: buff << "UNKNOWN(" << (int)state << ")" << std::endl; break;
    }
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

// ---- GNSS status --------------------------------------------------------------

TEST(ISStatusDecode, GnssStatus_RoundTrip_EverySingleBit)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), legacyRenderGnssStatusBitsReference(0u));
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGnssStatusBitsReference(v)) << "bit " << b;
    }
}

TEST(ISStatusDecode, GnssStatus_RoundTrip_FixTypes)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssStatus");
    ASSERT_NE(dec, nullptr);
    for (uint32_t fix = 0; fix <= 0x1F; ++fix) {        // covers defined (0..C) + undefined (D..1F)
        const uint32_t v = (fix << 8);                  // GNSS_STATUS_FIX_MASK = 0x1F00
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGnssStatusBitsReference(v)) << "fix " << fix;
    }
}

TEST(ISStatusDecode, GnssStatus_RoundTrip_SatCounts)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssStatus");
    ASSERT_NE(dec, nullptr);
    for (uint32_t n = 0; n <= 255; ++n) {               // always-emitted, dual-format count
        EXPECT_EQ(RenderStatusFromDecode(*dec, n), legacyRenderGnssStatusBitsReference(n)) << "sats " << n;
    }
}

TEST(ISStatusDecode, GnssStatus_RoundTrip_RtkErrorMaskQuirk)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssStatus");
    ASSERT_NE(dec, nullptr);
    // Exercise the legacy quirk: the base-error switch keys on (value & FLAGS_ERROR_MASK), so when
    // the raw-data-error bit is also set, no base-error line is emitted.
    const uint32_t cases[] = {
        GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_DATA_MISSING,
        GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_MOVING,
        GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_INVALID,
        GNSS_STATUS_FLAGS_GNSS1_RTK_RAW_GNSS_DATA_ERROR,
        GNSS_STATUS_FLAGS_GNSS1_RTK_RAW_GNSS_DATA_ERROR | GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_DATA_MISSING,
        GNSS_STATUS_FLAGS_GNSS1_RTK_RAW_GNSS_DATA_ERROR | GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_INVALID,
    };
    for (uint32_t v : cases) {
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGnssStatusBitsReference(v))
            << "rtk combo 0x" << std::hex << v;
    }
}

TEST(ISStatusDecode, GnssStatus_RoundTrip_CompassGate)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssStatus");
    ASSERT_NE(dec, nullptr);
    // Baseline-bad bit alone is gated off (compass mask not set) unless a compass-mask bit is set.
    const uint32_t cases[] = {
        GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_BAD,                                                  // gated OFF
        GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_ENABLED | GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_BAD,    // gate open
        GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_VALID,                                                         // gate open (valid is in mask)
    };
    for (uint32_t v : cases) {
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGnssStatusBitsReference(v))
            << "compass combo 0x" << std::hex << v;
    }
}

TEST(ISStatusDecode, GnssStatus_RoundTrip_RandomSweep)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssStatus");
    ASSERT_NE(dec, nullptr);
    uint32_t s = 0x5A7C0DEu;
    for (int i = 0; i < 40000; ++i) {                   // larger sweep — this field has the most quirks
        const uint32_t v = xorshift32(s);
        ASSERT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGnssStatusBitsReference(v))
            << "iteration " << i << " value 0x" << std::hex << v;
    }
}

TEST(ISStatusDecode, GnssStatus_StructuredDecode)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(dec->errorMask, (uint32_t)GNSS_STATUS_FLAGS_ERROR_MASK);

    const status_subfield_t* sats = nullptr;
    const status_subfield_t* fix  = nullptr;
    for (const auto& sf : dec->subfields) {
        if (sf.kind == eStatusSubfieldKind::Count) sats = &sf;
        if (sf.name == "Fix type")                 fix  = &sf;
    }
    ASSERT_NE(sats, nullptr);
    EXPECT_TRUE(sats->emitZero);                       // satellite count always shown
    ASSERT_NE(fix, nullptr);
    EXPECT_EQ(fix->shift, 8u);
    // 13 defined fix states (0..0xC).
    EXPECT_EQ(fix->values.size(), 13u);
}

// ---- GNSS status2 (jam/spoof, SN-8126) ----------------------------------------

TEST(ISStatusDecode, GnssStatus2_RegisteredForBothGnssPosDids)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("status2");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(GetStatusDecode(DID_GNSS1_POS, "status2"), dec);
    EXPECT_EQ(GetStatusDecode(DID_GNSS2_POS, "status2"), dec);
}

TEST(ISStatusDecode, GnssStatus2_AllFourFlagsPresentWithDistinctLabels)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("status2");
    ASSERT_NE(dec, nullptr);
    ASSERT_EQ(dec->subfields.size(), 4u);

    const uint32_t expectedMasks[4] = {
        (uint32_t)GNSS_STATUS2_FLAGS_GNSS_POSSIBLE_JAM_DETECT,
        (uint32_t)GNSS_STATUS2_FLAGS_GNSS_JAM_DETECTED,
        (uint32_t)GNSS_STATUS2_FLAGS_GNSS_POSSIBLE_SPOOF_DETECT,
        (uint32_t)GNSS_STATUS2_FLAGS_GNSS_SPOOF_DETECTED,
    };

    for (const auto& sf : dec->subfields)
    {
        EXPECT_EQ(sf.kind, eStatusSubfieldKind::Bit);

        bool maskMatched = false;
        for (uint32_t m : expectedMasks)
            if (sf.mask == m) maskMatched = true;
        EXPECT_TRUE(maskMatched) << "unexpected mask 0x" << std::hex << sf.mask;

        // No two sub-fields share a label.
        int labelCount = 0;
        for (const auto& other : dec->subfields)
            if (other.name == sf.name) ++labelCount;
        EXPECT_EQ(labelCount, 1) << "duplicate label \"" << sf.name << "\"";
    }

    // Every expected mask is present exactly once.
    for (uint32_t m : expectedMasks)
    {
        int matchCount = 0;
        for (const auto& sf : dec->subfields)
            if (sf.mask == m) ++matchCount;
        EXPECT_EQ(matchCount, 1) << "mask 0x" << std::hex << m << " not present exactly once";
    }
}

TEST(ISStatusDecode, GnssStatus2_ErrorMaskCoversOnlyConfirmedBits)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("status2");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(dec->errorMask, (uint32_t)GNSS_STATUS2_FLAGS_JAM_SPOOF_DETECTED_MASK);
    EXPECT_EQ(dec->errorMask & (uint32_t)GNSS_STATUS2_FLAGS_JAM_SPOOF_POSSIBLE_MASK, 0u);
}

TEST(ISStatusDecode, GnssStatus2_RenderEachFlagIndividually)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("status2");
    ASSERT_NE(dec, nullptr);

    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), "");

    const std::string possibleJam  = RenderStatusFromDecode(*dec, (uint32_t)GNSS_STATUS2_FLAGS_GNSS_POSSIBLE_JAM_DETECT);
    const std::string jamDetected  = RenderStatusFromDecode(*dec, (uint32_t)GNSS_STATUS2_FLAGS_GNSS_JAM_DETECTED);
    const std::string possibleSpoof = RenderStatusFromDecode(*dec, (uint32_t)GNSS_STATUS2_FLAGS_GNSS_POSSIBLE_SPOOF_DETECT);
    const std::string spoofDetected = RenderStatusFromDecode(*dec, (uint32_t)GNSS_STATUS2_FLAGS_GNSS_SPOOF_DETECTED);

    for (const std::string& s : {possibleJam, jamDetected, possibleSpoof, spoofDetected})
        EXPECT_FALSE(s.empty());

    // "Possible" and "confirmed" must render as visibly distinct text for the same interference type.
    EXPECT_NE(possibleJam, jamDetected);
    EXPECT_NE(possibleSpoof, spoofDetected);
    // All four flags must be distinguishable from each other.
    EXPECT_NE(possibleJam, possibleSpoof);
    EXPECT_NE(jamDetected, spoofDetected);
}

TEST(ISStatusDecode, GnssStatus2_RenderJamAndSpoofDetectedTogether)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("status2");
    ASSERT_NE(dec, nullptr);

    const uint32_t both = (uint32_t)GNSS_STATUS2_FLAGS_GNSS_JAM_DETECTED | (uint32_t)GNSS_STATUS2_FLAGS_GNSS_SPOOF_DETECTED;
    const std::string rendered = RenderStatusFromDecode(*dec, both);

    const std::string jamOnly   = RenderStatusFromDecode(*dec, (uint32_t)GNSS_STATUS2_FLAGS_GNSS_JAM_DETECTED);
    const std::string spoofOnly = RenderStatusFromDecode(*dec, (uint32_t)GNSS_STATUS2_FLAGS_GNSS_SPOOF_DETECTED);
    ASSERT_FALSE(jamOnly.empty());
    ASSERT_FALSE(spoofOnly.empty());

    // Search for the full single-flag rendering (including its trailing newline) as a substring
    // of the combined rendering, so an empty/broken render can't produce a vacuous match.
    EXPECT_NE(rendered.find(jamOnly), std::string::npos);
    EXPECT_NE(rendered.find(spoofOnly), std::string::npos);
    EXPECT_NE(both & dec->errorMask, 0u);   // both bits set -> field is in an error state
}

// ---- DID-aware lookup ---------------------------------------------------------

TEST(ISStatusDecode, DidAwareLookup_StatusFieldDisambiguation)
{
    // "status" on a GNSS DID resolves to the gnssStatus table.
    const status_field_decode_t* gnss = GetStatusDecodeByField("gnssStatus");
    ASSERT_NE(gnss, nullptr);
    EXPECT_EQ(GetStatusDecode(DID_GNSS1_POS, "status"), gnss);
    EXPECT_EQ(GetStatusDecode(DID_GNSS2_VEL, "status"), gnss);
    // GPX "status"/"hdwStatus" resolve to GPX keys (not yet registered -> nullptr for now, but must
    // NOT return the GNSS/IMX tables).
    EXPECT_NE(GetStatusDecode(DID_GPX_STATUS, "status"), gnss);
    EXPECT_NE(GetStatusDecode(DID_GPX_STATUS, "hdwStatus"), GetStatusDecodeByField("hdwStatus"));
    // GPX variants now registered: (DID_GPX_STATUS, status/hdwStatus) resolve to the GPX tables.
    EXPECT_EQ(GetStatusDecode(DID_GPX_STATUS, "status"),    GetStatusDecodeByField("gpxStatus"));
    EXPECT_EQ(GetStatusDecode(DID_GPX_STATUS, "hdwStatus"), GetStatusDecodeByField("gpxHdwStatus"));
}

// ---- GPX status / hdwStatus ---------------------------------------------------

TEST(ISStatusDecode, GpxStatus_RoundTrip_EverySingleBit)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), legacyRenderGpxStatusReference(0u));
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGpxStatusReference(v)) << "bit " << b;
    }
}

TEST(ISStatusDecode, GpxStatus_RoundTrip_FatalResets)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxStatus");
    ASSERT_NE(dec, nullptr);
    for (uint32_t f = 0; f <= 0x1F; ++f) {
        const uint32_t v = (f << 24);   // GPX_STATUS_FATAL_MASK = 0x1F000000
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGpxStatusReference(v)) << "fatal " << f;
    }
}

TEST(ISStatusDecode, GpxStatus_RoundTrip_RandomSweep)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxStatus");
    ASSERT_NE(dec, nullptr);
    uint32_t s = 0x60B5A1u;
    for (int i = 0; i < 20000; ++i) {
        const uint32_t v = xorshift32(s);
        ASSERT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGpxStatusReference(v))
            << "iteration " << i << " value 0x" << std::hex << v;
    }
}

// SN-8402: GPX_STATUS_COM*_RX_TRAFFIC_DETECTED (bits 4-7) were redefined from the prior
// ..._NOT_DETECTED (same mask, inverted meaning) so the wire bit itself carries positive
// polarity -- these are a status, not a fault, so isError stays false. No display-side
// inversion needed: the raw bit now directly matches the positive name.
TEST(ISStatusDecode, GpxStatus_RxTrafficSubfields_PositivePolarityNotFault)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxStatus");
    ASSERT_NE(dec, nullptr);

    struct Want { const char* name; uint32_t mask; };
    const Want wants[] = {
        { "COM0 RX traffic detected", (uint32_t)GPX_STATUS_COM0_RX_TRAFFIC_DETECTED },
        { "COM1 RX traffic detected", (uint32_t)GPX_STATUS_COM1_RX_TRAFFIC_DETECTED },
        { "COM2 RX traffic detected", (uint32_t)GPX_STATUS_COM2_RX_TRAFFIC_DETECTED },
        { "USB RX traffic detected",  (uint32_t)GPX_STATUS_USB_RX_TRAFFIC_DETECTED },
    };

    for (const auto& w : wants) {
        const status_subfield_t* sf = nullptr;
        for (const auto& cand : dec->subfields)
            if (cand.mask == w.mask) { sf = &cand; break; }
        ASSERT_NE(sf, nullptr) << w.name;
        EXPECT_EQ(sf->name, w.name);
        EXPECT_EQ(sf->kind, eStatusSubfieldKind::Bit);
        EXPECT_FALSE(sf->isError) << w.name << " is a status, not a fault";
        // legacyText / RenderStatusFromDecode's line output: emitted (positive-worded) only when
        // the bit is actually set -- i.e. only when traffic IS detected.
        EXPECT_NE(sf->legacyText.find("detected"), std::string::npos) << w.name;
        EXPECT_EQ(sf->legacyText.find("not detected"), std::string::npos) << w.name;
        EXPECT_EQ(RenderStatusFromDecode(*dec, w.mask), legacyRenderGpxStatusReference(w.mask));
    }
}

TEST(ISStatusDecode, GpxHdwStatus_RoundTrip_EverySingleBit)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxHdwStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), legacyRenderGpxHdwStatusReference(0u));
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGpxHdwStatusReference(v)) << "bit " << b;
    }
}

TEST(ISStatusDecode, GpxHdwStatus_RoundTrip_RandomSweep)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxHdwStatus");
    ASSERT_NE(dec, nullptr);
    uint32_t s = 0x9D7E11u;
    for (int i = 0; i < 20000; ++i) {
        const uint32_t v = xorshift32(s);
        ASSERT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGpxHdwStatusReference(v))
            << "iteration " << i << " value 0x" << std::hex << v;
    }
}

// ---- GPX-GNSS scalar-enum state fields ----------------------------------------

TEST(ISStatusDecode, GnssStateEnums_RoundTrip_FullByteRange)
{
    const status_field_decode_t* init = GetStatusDecodeByField("gnssInitState");
    const status_field_decode_t* run  = GetStatusDecodeByField("gnssRunState");
    const status_field_decode_t* fw   = GetStatusDecodeByField("gnssFwUpdateState");
    const status_field_decode_t* rst  = GetStatusDecodeByField("gnssLastResetCause");
    ASSERT_NE(init, nullptr); ASSERT_NE(run, nullptr); ASSERT_NE(fw, nullptr); ASSERT_NE(rst, nullptr);
    EXPECT_TRUE(init->scalarEnum);
    for (uint32_t v = 0; v <= 255; ++v) {
        EXPECT_EQ(RenderStatusFromDecode(*init, v), legacyRenderGnssInitStateReference((uint8_t)v)) << "init " << v;
        EXPECT_EQ(RenderStatusFromDecode(*run,  v), legacyRenderGnssRunStateReference((uint8_t)v))  << "run " << v;
        EXPECT_EQ(RenderStatusFromDecode(*fw,   v), legacyRenderGnssFwUpdateStateReference((uint8_t)v)) << "fw " << v;
        EXPECT_EQ(RenderStatusFromDecode(*rst,  v), legacyRenderGnssLastResetCauseReference((uint8_t)v)) << "rst " << v;
    }
}

TEST(ISStatusDecode, GnssFwUpdateState_AllStatesDecode)
{
    // SN-7919 fix: the original renderer's `cxdRst_Max` (13) bound left states 13..16 undecoded;
    // now bounded by the array size (17) so every firmware-update state decodes.
    const status_field_decode_t* fw = GetStatusDecodeByField("gnssFwUpdateState");
    ASSERT_NE(fw, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*fw, 12u), "ProgramExecutionWait");
    EXPECT_EQ(RenderStatusFromDecode(*fw, 13u), "ProgramExecutionFinish");   // fixed (was empty)
    EXPECT_EQ(RenderStatusFromDecode(*fw, 16u), "Done");                     // fixed (was empty)
    EXPECT_EQ(RenderStatusFromDecode(*fw, 17u), "");                         // genuinely out of range
}

// ---- IMX / GPX built-in-test fields -------------------------------------------

TEST(ISStatusDecode, ImxHdwBit_RoundTrip)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("hdwBitStatus");
    ASSERT_NE(dec, nullptr);
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderImxHdwBitReference(v)) << "bit " << b;
    }
    for (uint32_t mode = 0; mode <= 0xF; ++mode) {                 // exercise the hex/dec BIT-mode line
        const uint32_t v = (mode << 4);                           // HDW_BIT_MODE_MASK = 0x000000F0
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderImxHdwBitReference(v)) << "mode " << mode;
    }
    uint32_t s = 0x1357BDu;
    for (int i = 0; i < 20000; ++i) {
        const uint32_t v = xorshift32(s);
        ASSERT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderImxHdwBitReference(v))
            << "iteration " << i << " value 0x" << std::hex << v;
    }
}

TEST(ISStatusDecode, ImxCalBit_RoundTrip)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("calBitStatus");
    ASSERT_NE(dec, nullptr);
    for (int b = 0; b < 32; ++b) {
        const uint32_t v = (1u << b);
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderImxCalBitReference(v)) << "bit " << b;
    }
    for (uint32_t mode = 0; mode <= 0xF; ++mode) {
        const uint32_t v = (mode << 4);                           // CAL_BIT_MODE_MASK = 0x000000F0
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderImxCalBitReference(v)) << "mode " << mode;
    }
    uint32_t s = 0x2468ACu;
    for (int i = 0; i < 20000; ++i) {
        const uint32_t v = xorshift32(s);
        ASSERT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderImxCalBitReference(v))
            << "iteration " << i << " value 0x" << std::hex << v;
    }
}

TEST(ISStatusDecode, GpxBitResults_RoundTrip)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxBitResults");
    ASSERT_NE(dec, nullptr);
    for (uint32_t v = 0; v <= 0xFF; ++v) {                        // results live in the low byte
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGpxBitResultsReference(v)) << "results " << v;
    }
}

TEST(ISStatusDecode, GpxBitState_RoundTrip)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxBitState");
    ASSERT_NE(dec, nullptr);
    for (uint32_t v = 0; v <= 255; ++v) {                        // includes the UNKNOWN(n) default path
        EXPECT_EQ(RenderStatusFromDecode(*dec, v), legacyRenderGpxBitStateReference((uint8_t)v)) << "state " << v;
    }
}

// ---- imu_t / pimu_t status (eImuStatus, SN-8491) --------------------------------
//
// The "imuStatus" table itself was added alongside the original SN-7919 work but was never
// exercised by a test or wired to a renderer (SN-8491 does the wiring in ISDataMappings.cpp).
// There is no legacy hand-written renderer to reproduce byte-for-byte here (unlike the other
// tables above), so these tests assert the table's structure/semantics directly against the
// eImuStatus enum instead of a round-trip oracle.

TEST(ISStatusDecode, ImuStatus_RegisteredAndRoutesForImuAndPimu)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("imuStatus");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(GetStatusDecode(DID_IMU, "status"), dec);
    EXPECT_EQ(GetStatusDecode(DID_PIMU, "status"), dec);
    EXPECT_EQ(GetStatusDecode(DID_REFERENCE_IMU, "status"), dec);
    EXPECT_EQ(GetStatusDecode(DID_REFERENCE_PIMU, "status"), dec);
    // Must not be confused with the GNSS/GPX "status" tables that share the same on-wire name.
    EXPECT_NE(GetStatusDecode(DID_GNSS1_POS, "status"), dec);
    EXPECT_NE(GetStatusDecode(DID_GPX_STATUS, "status"), dec);
}

TEST(ISStatusDecode, ImuStatus_AllSixSensorOkBitsPresentAndNotErrors)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("imuStatus");
    ASSERT_NE(dec, nullptr);

    const uint32_t okMasks[6] = {
        (uint32_t)IMU_STATUS_GYR_X_OK, (uint32_t)IMU_STATUS_GYR_Y_OK, (uint32_t)IMU_STATUS_GYR_Z_OK,
        (uint32_t)IMU_STATUS_ACC_X_OK, (uint32_t)IMU_STATUS_ACC_Y_OK, (uint32_t)IMU_STATUS_ACC_Z_OK,
    };
    for (uint32_t m : okMasks) {
        const status_subfield_t* sf = nullptr;
        for (const auto& cand : dec->subfields)
            if (cand.mask == m) { sf = &cand; break; }
        ASSERT_NE(sf, nullptr) << "mask 0x" << std::hex << m;
        EXPECT_EQ(sf->kind, eStatusSubfieldKind::Bit);
        EXPECT_FALSE(sf->isError) << "sensor-OK bit 0x" << std::hex << m << " must not be an error";
    }
}

TEST(ISStatusDecode, ImuStatus_FaultAndSaturationBitsAreErrors)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("imuStatus");
    ASSERT_NE(dec, nullptr);

    const uint32_t errorMasks[4] = {
        (uint32_t)IMU_STATUS_GYR_FAULT_REJECT, (uint32_t)IMU_STATUS_ACC_FAULT_REJECT,
        (uint32_t)IMU_STATUS_SATURATION_GYR,   (uint32_t)IMU_STATUS_SATURATION_ACC,
    };
    for (uint32_t m : errorMasks) {
        const status_subfield_t* sf = nullptr;
        for (const auto& cand : dec->subfields)
            if (cand.mask == m) { sf = &cand; break; }
        ASSERT_NE(sf, nullptr) << "mask 0x" << std::hex << m;
        EXPECT_TRUE(sf->isError) << "fault/saturation bit 0x" << std::hex << m << " must be an error";
        EXPECT_NE(dec->errorMask & m, 0u) << "errorMask must cover 0x" << std::hex << m;
    }

    // errorMask must cover exactly these four bits, nothing else.
    uint32_t expectedErrorMask = 0;
    for (uint32_t m : errorMasks) expectedErrorMask |= m;
    EXPECT_EQ(dec->errorMask, expectedErrorMask);
}

TEST(ISStatusDecode, ImuStatus_RenderEachBitIndividually_NonEmptyAndDistinct)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("imuStatus");
    ASSERT_NE(dec, nullptr);

    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), "");

    std::vector<std::string> rendered;
    for (const auto& sf : dec->subfields)
        rendered.push_back(RenderStatusFromDecode(*dec, sf.mask));

    for (size_t i = 0; i < rendered.size(); ++i) {
        EXPECT_FALSE(rendered[i].empty()) << "subfield \"" << dec->subfields[i].name << "\"";
        for (size_t j = i + 1; j < rendered.size(); ++j)
            EXPECT_NE(rendered[i], rendered[j])
                << "subfields \"" << dec->subfields[i].name << "\" and \"" << dec->subfields[j].name << "\" render identically";
    }
}

TEST(ISStatusDecode, ImuStatus_RenderIsStableAndAdditive)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("imuStatus");
    ASSERT_NE(dec, nullptr);

    const uint32_t combo = (uint32_t)IMU_STATUS_GYR_X_OK | (uint32_t)IMU_STATUS_ACC_FAULT_REJECT | (uint32_t)IMU_STATUS_SATURATION_GYR;
    const std::string a = RenderStatusFromDecode(*dec, combo);
    const std::string b = RenderStatusFromDecode(*dec, combo);
    EXPECT_EQ(a, b);   // deterministic/stable for the same input

    const std::string gyrXOk       = RenderStatusFromDecode(*dec, (uint32_t)IMU_STATUS_GYR_X_OK);
    const std::string accFaultRej  = RenderStatusFromDecode(*dec, (uint32_t)IMU_STATUS_ACC_FAULT_REJECT);
    const std::string satGyr       = RenderStatusFromDecode(*dec, (uint32_t)IMU_STATUS_SATURATION_GYR);
    EXPECT_NE(a.find(gyrXOk), std::string::npos);
    EXPECT_NE(a.find(accFaultRej), std::string::npos);
    EXPECT_NE(a.find(satGyr), std::string::npos);
}

// ---- Flash-config fields (nvm_flash_cfg_t / gpx_flash_cfg_t, SN-8491) ---------

TEST(ISStatusDecode, GnssSatSigConst_AllEightConstellationsPresentAndDistinct)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssSatSigConst");
    ASSERT_NE(dec, nullptr);
    ASSERT_EQ(dec->subfields.size(), 8u);

    const uint32_t masks[8] = {
        (uint32_t)GNSS_SAT_SIG_CONST_GPS, (uint32_t)GNSS_SAT_SIG_CONST_QZS,
        (uint32_t)GNSS_SAT_SIG_CONST_GAL, (uint32_t)GNSS_SAT_SIG_CONST_BDS,
        (uint32_t)GNSS_SAT_SIG_CONST_GLO, (uint32_t)GNSS_SAT_SIG_CONST_SBS,
        (uint32_t)GNSS_SAT_SIG_CONST_IRN, (uint32_t)GNSS_SAT_SIG_CONST_IME,
    };
    for (uint32_t m : masks) {
        int matchCount = 0;
        for (const auto& sf : dec->subfields)
            if (sf.mask == m) ++matchCount;
        EXPECT_EQ(matchCount, 1) << "mask 0x" << std::hex << m << " not present exactly once";
    }

    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), "");

    // GNSS_SAT_SIG_CONST_DEFAULT excludes IRNSS and IMES -- the rendered default must mention
    // every constellation it includes and NEITHER of the two it excludes.
    const std::string rendered = RenderStatusFromDecode(*dec, (uint32_t)GNSS_SAT_SIG_CONST_DEFAULT);
    for (const char* included : { "GPS", "QZSS", "Galileo", "BeiDou", "GLONASS", "SBAS" })
        EXPECT_NE(rendered.find(included), std::string::npos) << included << " missing from default rendering";
    for (const char* excluded : { "IRNSS", "IMES" })
        EXPECT_EQ(rendered.find(excluded), std::string::npos) << excluded << " should not be in the default rendering";
}

TEST(ISStatusDecode, GnssSatSigConst_ErrorMaskAlwaysZero)
{
    // Constellation selection is a config choice, not a fault condition.
    const status_field_decode_t* dec = GetStatusDecodeByField("gnssSatSigConst");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(dec->errorMask, 0u);
}

TEST(ISStatusDecode, DynamicModel_IsScalarEnumCoveringEveryDefinedModel)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("dynamicModel");
    ASSERT_NE(dec, nullptr);
    EXPECT_TRUE(dec->scalarEnum);
    ASSERT_EQ(dec->subfields.size(), 1u);
    EXPECT_EQ(dec->subfields.front().values.size(), (size_t)DYNAMIC_MODEL_COUNT);

    EXPECT_EQ(RenderStatusFromDecode(*dec, (uint32_t)DYNAMIC_MODEL_PORTABLE), "Portable");
    EXPECT_EQ(RenderStatusFromDecode(*dec, (uint32_t)DYNAMIC_MODEL_GROUND_VEHICLE), "Ground vehicle");
    EXPECT_EQ(RenderStatusFromDecode(*dec, (uint32_t)DYNAMIC_MODEL_WRIST), "Wrist");
    EXPECT_EQ(RenderStatusFromDecode(*dec, (uint32_t)DYNAMIC_MODEL_INDOOR), "Indoor");
    EXPECT_EQ(RenderStatusFromDecode(*dec, (uint32_t)DYNAMIC_MODEL_COUNT), "");   // one past the last defined model
}

TEST(ISStatusDecode, ImxSysCfgBits_IndependentBitsRenderNonEmptyAndDistinct)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("sysCfgBits");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(dec->errorMask, 0u);   // configuration bits, not faults

    const uint32_t bits[] = {
        (uint32_t)SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL, (uint32_t)SYS_CFG_BITS_AUTO_MAG_RECAL,
        (uint32_t)SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION, (uint32_t)SYS_CFG_BITS_DISABLE_LEDS,
        (uint32_t)SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION, (uint32_t)SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION,
        (uint32_t)SYS_CFG_BITS_DISABLE_BAROMETER_FUSION, (uint32_t)SYS_CFG_BITS_DISABLE_GNSS1_FUSION,
        (uint32_t)SYS_CFG_BITS_DISABLE_GNSS2_FUSION, (uint32_t)SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES,
        (uint32_t)SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES, (uint32_t)SYS_CFG_BITS_DISABLE_INS_EKF,
        (uint32_t)SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP, (uint32_t)SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION,
        (uint32_t)SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION, (uint32_t)SYS_CFG_USE_REFERENCE_IMU_IN_EKF,
        (uint32_t)SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT,
    };

    std::vector<std::string> rendered;
    for (uint32_t b : bits) {
        const std::string s = RenderStatusFromDecode(*dec, b);
        EXPECT_FALSE(s.empty()) << "bit 0x" << std::hex << b;
        rendered.push_back(s);
    }
    for (size_t i = 0; i < rendered.size(); ++i)
        for (size_t j = i + 1; j < rendered.size(); ++j)
            EXPECT_NE(rendered[i], rendered[j]) << "bits " << i << " and " << j << " render identically";
}

TEST(ISStatusDecode, ImxSysCfgBits_MagRecalModeSubfield)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("sysCfgBits");
    ASSERT_NE(dec, nullptr);

    const uint32_t multiAxis  = 1u << SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET;
    const uint32_t singleAxis = 2u << SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET;
    const std::string multi  = RenderStatusFromDecode(*dec, multiAxis);
    const std::string single = RenderStatusFromDecode(*dec, singleAxis);
    EXPECT_NE(multi, single);
    EXPECT_NE(multi.find("Mag recal mode: Multi-axis"), std::string::npos);
    EXPECT_NE(single.find("Mag recal mode: Single-axis"), std::string::npos);
    // Value 0 (disabled) still renders a line -- it's a meaningful Enum state, not absence.
    EXPECT_NE(RenderStatusFromDecode(*dec, 0u).find("Mag recal mode: Disabled"), std::string::npos);
}

TEST(ISStatusDecode, ImxSysCfgBits_BrownoutThresholdSubfield)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("sysCfgBits");
    ASSERT_NE(dec, nullptr);

    const uint32_t level3 = (uint32_t)SYS_CFG_BITS_BOR_LEVEL_3 << SYS_CFG_BITS_BOR_THRESHOLD_OFFSET;
    EXPECT_NE(RenderStatusFromDecode(*dec, level3).find("2.5-2.6V"), std::string::npos);
}

TEST(ISStatusDecode, ImxSysCfgBits_UnusedBit0ContributesNoBitOfItsOwn)
{
    // UNUSED1 (bit 0) is reserved/unused and is deliberately not given its own Bit subfield, so
    // setting it must not change the rendering at all vs. an all-zero value -- the two always-on
    // Enum subfields (mag recal mode, brownout threshold) still render their "off"/default state
    // either way, but UNUSED1 itself must not add a line.
    const status_field_decode_t* dec = GetStatusDecodeByField("sysCfgBits");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, (uint32_t)UNUSED1), RenderStatusFromDecode(*dec, 0u));
}

TEST(ISStatusDecode, GpxFlashCfg_SysCfgBits_NotYetRoutedReturnsNullNotImxTable)
{
    // gpx_flash_cfg_t::sysCfgBits shares the on-wire field name "sysCfgBits" with
    // nvm_flash_cfg_t but uses a DIFFERENT enum (eGpxSysConfigBits). Until SN-8491 phase 2 adds
    // its own table, DID-aware lookup must return nullptr here -- NOT silently hand back the IMX
    // table, which would misrender every GPX config bit.
    EXPECT_EQ(GetStatusDecode(DID_GPX_FLASH_CFG, "sysCfgBits"), nullptr);
    EXPECT_NE(GetStatusDecode(DID_FLASH_CONFIG, "sysCfgBits"), nullptr);
    EXPECT_EQ(GetStatusDecode(DID_FLASH_CONFIG, "sysCfgBits"), GetStatusDecodeByField("sysCfgBits"));
}

// ---- rmc_t::options (RMC_OPTIONS_*, SN-8491) ----------------------------------

TEST(ISStatusDecode, RmcOptions_PortBitsRenderNonEmptyAndDistinct)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("rmcOptions");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(dec->errorMask, 0u);
    EXPECT_EQ(RenderStatusFromDecode(*dec, (uint32_t)RMC_OPTIONS_PORT_CURRENT), "");   // 0x00: "current port" is silent, not an error

    const uint32_t ports[] = {
        (uint32_t)RMC_OPTIONS_PORT_SER0, (uint32_t)RMC_OPTIONS_PORT_SER1,
        (uint32_t)RMC_OPTIONS_PORT_SER2, (uint32_t)RMC_OPTIONS_PORT_USB,
    };
    std::vector<std::string> rendered;
    for (uint32_t p : ports) {
        const std::string s = RenderStatusFromDecode(*dec, p);
        EXPECT_FALSE(s.empty()) << "port bit 0x" << std::hex << p;
        rendered.push_back(s);
    }
    for (size_t i = 0; i < rendered.size(); ++i)
        for (size_t j = i + 1; j < rendered.size(); ++j)
            EXPECT_NE(rendered[i], rendered[j]);
}

TEST(ISStatusDecode, RmcOptions_AllPortsSetRendersAllFourIndividually)
{
    // RMC_OPTIONS_PORT_ALL == RMC_OPTIONS_PORT_MASK (0xFF) implies all 4 meaningful port bits,
    // so it should render each port's line individually rather than needing a distinct "All" label.
    const status_field_decode_t* dec = GetStatusDecodeByField("rmcOptions");
    ASSERT_NE(dec, nullptr);
    const std::string rendered = RenderStatusFromDecode(*dec, (uint32_t)RMC_OPTIONS_PORT_ALL);
    for (const char* port : { "Serial 0", "Serial 1", "Serial 2", "USB" })
        EXPECT_NE(rendered.find(port), std::string::npos) << port << " missing when all ports selected";
}

TEST(ISStatusDecode, RmcOptions_PreserveCtrlAndPersistentBits)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("rmcOptions");
    ASSERT_NE(dec, nullptr);
    const std::string preserve   = RenderStatusFromDecode(*dec, (uint32_t)RMC_OPTIONS_PRESERVE_CTRL);
    const std::string persistent = RenderStatusFromDecode(*dec, (uint32_t)RMC_OPTIONS_PERSISTENT);
    EXPECT_NE(preserve.find("Preserve"), std::string::npos);
    EXPECT_NE(persistent.find("persists across reboot"), std::string::npos);
    EXPECT_NE(preserve, persistent);
}

TEST(ISStatusDecode, RmcOptions_NmeaSpeedFilterSubfield)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("rmcOptions");
    ASSERT_NE(dec, nullptr);

    const uint32_t enable  = (uint32_t)RMC_OPTIONS_NMEA_SPEED_FILTER_ENABLE  << RMC_OPTIONS_NMEA_SPEED_FILTER_OFFSET;
    const uint32_t disable = (uint32_t)RMC_OPTIONS_NMEA_SPEED_FILTER_DISABLE << RMC_OPTIONS_NMEA_SPEED_FILTER_OFFSET;
    const std::string enabled  = RenderStatusFromDecode(*dec, enable);
    const std::string disabled = RenderStatusFromDecode(*dec, disable);
    EXPECT_NE(enabled.find("Enabled"), std::string::npos);
    EXPECT_NE(disabled.find("Disabled"), std::string::npos);
    EXPECT_NE(enabled, disabled);

    // Unlike sysCfgBits' mag-recal-mode Enum, 0 ("not specified") is NOT a meaningful state here --
    // only Enable(1)/Disable(2) are defined, so an all-zero value renders nothing at all.
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u), "");
}

TEST(ISStatusDecode, RmcOptions_RegisteredForBothRmcDids)
{
    // "options" is ambiguous (nmea_msgs_t also has one), so rmcOptions is looked up only via its
    // internal key from renderRmcOptions -- never through GetStatusDecode(did, "options"). Confirm
    // the table itself is DID-agnostic content-wise: it applies equally whichever DID uses it.
    const status_field_decode_t* dec = GetStatusDecodeByField("rmcOptions");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(dec->fieldName, "options");
}

// ---- nvm_flash_cfg_t::ioConfig2 (eIoConfig2, SN-8491) -------------------------

TEST(ISStatusDecode, IoConfig2_G11FunctionSubfield)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig2");
    ASSERT_NE(dec, nullptr);
    const std::string swdio  = RenderStatusFromDecode(*dec, (uint32_t)IO_CFG2_G11_SWDIO);
    const std::string strobe = RenderStatusFromDecode(*dec, (uint32_t)IO_CFG2_G11_STROBE_INPUT_val);
    EXPECT_NE(swdio.find("G11: SWDIO"), std::string::npos);
    EXPECT_NE(strobe.find("G11: Strobe input"), std::string::npos);
    EXPECT_NE(swdio, strobe);
}

TEST(ISStatusDecode, IoConfig2_G12AndG13AreIndependentSubfields)
{
    // G11 (bit 0), G12 (bits 2-1), and G13 (bits 4-3) must decode from their own bits only -- not
    // "no cross-talk means silence": all four ioConfig2 Enum subfields always render SOME line
    // (every pin always has a function selected), so setting only G12's bits still leaves G11,
    // G13, and GNSS2-PPS-source at their default ("value 0") states, each rendering its own line.
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig2");
    ASSERT_NE(dec, nullptr);
    const std::string g12Xscl = RenderStatusFromDecode(*dec, (uint32_t)IO_CFG2_G12_XSCL_val);
    EXPECT_NE(g12Xscl.find("G12: XSCL"), std::string::npos);
    // G13 is still at its default (DRDY), G11 at its default (SWDIO) -- verify those specific
    // defaults appear, not some other G12-driven value bleeding into them.
    EXPECT_NE(g12Xscl.find("G13: DRDY"), std::string::npos);
    EXPECT_NE(g12Xscl.find("G11: SWDIO"), std::string::npos);
    EXPECT_EQ(g12Xscl.find("G13: Strobe"), std::string::npos);
    EXPECT_EQ(g12Xscl.find("G13: XSDA"), std::string::npos);

    const std::string g13Strobe = RenderStatusFromDecode(*dec, (uint32_t)IO_CFG2_G13_STROBE_INPUT_val);
    EXPECT_NE(g13Strobe.find("G13: Strobe input"), std::string::npos);
    EXPECT_NE(g13Strobe.find("G12: SWO"), std::string::npos);   // G12's own default, unaffected by G13's bits
    EXPECT_EQ(g13Strobe.find("G12: XSCL"), std::string::npos);
    EXPECT_EQ(g13Strobe.find("G12: Strobe"), std::string::npos);
}

TEST(ISStatusDecode, IoConfig2_Gnss2PpsSourceAndUseGnss2AsSource)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig2");
    ASSERT_NE(dec, nullptr);
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u).empty(), false) << "always-on Enum defaults should still render something";

    const std::string useGnss2 = RenderStatusFromDecode(*dec, (uint32_t)IO_CFG2_USE_GNSS2_AS_SOURCE);
    EXPECT_NE(useGnss2.find("Use GNSS2"), std::string::npos);

    const std::string ppsG13 = RenderStatusFromDecode(*dec, (uint32_t)IO_CFG2_GNSS2_PPS_SOURCE_G13_val);
    EXPECT_NE(ppsG13.find("GNSS2 PPS source: G13"), std::string::npos);
}

// ---- nvm_flash_cfg_t::ioConfig (eIoConfig, SN-8491) ---------------------------

TEST(ISStatusDecode, IoConfig_ZeroBaselineIsTheFiveAlwaysOnEnumDefaults)
{
    // Unlike ioConfig2, ioConfig's G1/G2, G9, G6/G7, and G5/G8 function Enums have no named value
    // for raw 0 (the source enum simply doesn't define one), so they stay silent. But GNSS1 PPS
    // source and the GNSS1/2 source+type Enums DO have an explicitly named "Disabled"/"None" value
    // at 0 (matching eIoConfig's own definitions), so those five always render -- this is the
    // baseline every other bit's rendering gets compared against below.
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);
    const std::string baseline = RenderStatusFromDecode(*dec, 0u);
    for (const char* expected : { "GNSS1 PPS source: Disabled", "GNSS1 source: Disabled", "GNSS2 source: Disabled", "GNSS1 type: None", "GNSS2 type: None" })
        EXPECT_NE(baseline.find(expected), std::string::npos) << expected;
    EXPECT_EQ(baseline.find("G1/G2:"), std::string::npos) << "G1/G2 has no named 0 value, should stay silent";
}

TEST(ISStatusDecode, IoConfig_StrobeAndG15BitsIndependent)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);
    const std::string baseline = RenderStatusFromDecode(*dec, 0u);

    const std::string strobe = RenderStatusFromDecode(*dec, (uint32_t)IO_CONFIG_STROBE_TRIGGER_HIGH);
    const std::string g15    = RenderStatusFromDecode(*dec, (uint32_t)IO_CONFIG_G15_STROBE_INPUT);
    EXPECT_NE(strobe.find("Strobe (input and output) trigger on rising edge"), std::string::npos);
    EXPECT_NE(g15.find("G15 (GNSS PPS) strobe input"), std::string::npos);
    EXPECT_NE(strobe, g15);
    // Both still carry the always-on Enum baseline alongside their own bit.
    EXPECT_NE(strobe.find("GNSS1 PPS source: Disabled"), std::string::npos);
    EXPECT_GT(strobe.length(), baseline.length());
}

TEST(ISStatusDecode, IoConfig_G1G2FunctionSubfield)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);
    const std::string canBus = RenderStatusFromDecode(*dec, (uint32_t)IO_CONFIG_G1G2_CAN_BUS);
    const std::string i2c    = RenderStatusFromDecode(*dec, (uint32_t)IO_CONFIG_G1G2_I2C);
    EXPECT_NE(canBus.find("G1/G2: CAN Bus"), std::string::npos);
    EXPECT_NE(i2c.find("G1/G2: I2C"), std::string::npos);
    EXPECT_NE(canBus, i2c);
    // Raw 0 in the 3-bit G1/G2 field has no named value -- must not render anything for it,
    // even though the always-on Enums elsewhere in the table still render their own defaults.
    EXPECT_EQ(RenderStatusFromDecode(*dec, 0u).find("G1/G2:"), std::string::npos);
}

TEST(ISStatusDecode, IoConfig_Gnss1PpsSourceSubfield)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);
    const uint32_t g9Source = (uint32_t)IO_CFG_GNSS1_PPS_SOURCE_G9 << IO_CFG_GNSS1_PPS_SOURCE_OFFSET;
    const std::string rendered = RenderStatusFromDecode(*dec, g9Source);
    EXPECT_NE(rendered.find("GNSS1 PPS source: G9"), std::string::npos);
}

TEST(ISStatusDecode, IoConfig_Gnss1AndGnss2SourceDoNotCrossContaminate)
{
    // GNSS1 source (bits 18-16) and GNSS2 source (bits 21-19) are adjacent, same value set --
    // exactly where a mask/shift mistake would silently leak one into the other.
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);

    uint32_t v = 0;
    SET_IO_CFG_GNSS1_SOURCE(v, (uint32_t)IO_CONFIG_GNSS_SOURCE_SER0);
    SET_IO_CFG_GNSS2_SOURCE(v, (uint32_t)IO_CONFIG_GNSS_SOURCE_SER2);
    const std::string rendered = RenderStatusFromDecode(*dec, v);
    EXPECT_NE(rendered.find("GNSS1 source: Ser0"), std::string::npos);
    EXPECT_NE(rendered.find("GNSS2 source: Ser2"), std::string::npos);
    EXPECT_EQ(rendered.find("GNSS1 source: Ser2"), std::string::npos);
    EXPECT_EQ(rendered.find("GNSS2 source: Ser0"), std::string::npos);
}

TEST(ISStatusDecode, IoConfig_Gnss1AndGnss2TypeDoNotCrossContaminate)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);

    uint32_t v = 0;
    SET_IO_CFG_GNSS1_TYPE(v, (uint32_t)IO_CONFIG_GNSS_TYPE_UBLOX);
    SET_IO_CFG_GNSS2_TYPE(v, (uint32_t)IO_CONFIG_GNSS_TYPE_SEPTENTRIO);
    const std::string rendered = RenderStatusFromDecode(*dec, v);
    EXPECT_NE(rendered.find("GNSS1 type: UBLOX"), std::string::npos);
    EXPECT_NE(rendered.find("GNSS2 type: Septentrio"), std::string::npos);
    EXPECT_EQ(rendered.find("GNSS1 type: Septentrio"), std::string::npos);
    EXPECT_EQ(rendered.find("GNSS2 type: UBLOX"), std::string::npos);
}

TEST(ISStatusDecode, IoConfig_ImuDisableBitsIndependent)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);
    const std::string imu1 = RenderStatusFromDecode(*dec, (uint32_t)IO_CONFIG_IMU_1_DISABLE);
    const std::string imu2 = RenderStatusFromDecode(*dec, (uint32_t)IO_CONFIG_IMU_2_DISABLE);
    const std::string imu3 = RenderStatusFromDecode(*dec, (uint32_t)IO_CONFIG_IMU_3_DISABLE);
    EXPECT_NE(imu1.find("IMU 1 disable"), std::string::npos);
    EXPECT_NE(imu2.find("IMU 2 disable"), std::string::npos);
    EXPECT_NE(imu3.find("IMU 3 disable"), std::string::npos);
    EXPECT_NE(imu1, imu2);
    EXPECT_NE(imu2, imu3);
}

TEST(ISStatusDecode, IoConfig_AllBitsSetDoesNotCrash)
{
    // Fuzz-lite: every bit set at once must not throw, crash, or produce an empty result (many
    // subfields have a defined value at their max range).
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);
    const std::string rendered = RenderStatusFromDecode(*dec, 0xFFFFFFFFu);
    EXPECT_FALSE(rendered.empty());
}

// ---- nvm_flash_cfg_t::sensorConfig (eSensorConfig, SN-8491) -------------------

TEST(ISStatusDecode, SensorConfig_GyroAndAccelFullScaleAreIndependentAlwaysOnEnums)
{
    // Every value in both tables (including 0) is explicitly named in the source enum, so both
    // always render -- verify the zero baseline includes both defaults.
    const status_field_decode_t* dec = GetStatusDecodeByField("sensorConfig");
    ASSERT_NE(dec, nullptr);
    const std::string baseline = RenderStatusFromDecode(*dec, 0u);
    EXPECT_NE(baseline.find("Gyro FS: 250 deg/s"), std::string::npos);
    EXPECT_NE(baseline.find("Accel FS: 2g"), std::string::npos);

    const uint32_t gyro4000 = (uint32_t)SENSOR_CFG_GYR_FS_4000 << SENSOR_CFG_GYR_FS_OFFSET;
    const uint32_t acc16g   = (uint32_t)SENSOR_CFG_ACC_FS_16G  << SENSOR_CFG_ACC_FS_OFFSET;
    const std::string rendered = RenderStatusFromDecode(*dec, gyro4000 | acc16g);
    EXPECT_NE(rendered.find("Gyro FS: 4000 deg/s"), std::string::npos);
    EXPECT_NE(rendered.find("Accel FS: 16g"), std::string::npos);
    EXPECT_EQ(rendered.find("Gyro FS: 250 deg/s"), std::string::npos);
    EXPECT_EQ(rendered.find("Accel FS: 2g"), std::string::npos);
}

TEST(ISStatusDecode, SensorConfig_GyroAndAccelDlpfDoNotCrossContaminate)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("sensorConfig");
    ASSERT_NE(dec, nullptr);
    const uint32_t gyroDlpf20 = (uint32_t)SENSOR_CFG_GYR_DLPF_20HZ << SENSOR_CFG_GYR_DLPF_OFFSET;
    const uint32_t accDlpf45 = (uint32_t)SENSOR_CFG_ACC_DLPF_45HZ << SENSOR_CFG_ACC_DLPF_OFFSET;
    const std::string rendered = RenderStatusFromDecode(*dec, gyroDlpf20 | accDlpf45);
    EXPECT_NE(rendered.find("Gyro DLPF: 20 Hz"), std::string::npos);
    EXPECT_NE(rendered.find("Accel DLPF: 45 Hz"), std::string::npos);
    EXPECT_EQ(rendered.find("Gyro DLPF: 45 Hz"), std::string::npos);
    EXPECT_EQ(rendered.find("Accel DLPF: 20 Hz"), std::string::npos);
}

TEST(ISStatusDecode, SensorConfig_AccelDlpf218HzAliasesAreDistinctLabels)
{
    // ACC_DLPF_218HZ(0) and ACC_DLPF_218HZb(1) are two distinct register values that both mean
    // "218 Hz" per the source enum -- confirm they render as distinguishable text, not identical.
    const status_field_decode_t* dec = GetStatusDecodeByField("sensorConfig");
    ASSERT_NE(dec, nullptr);
    const uint32_t a = (uint32_t)SENSOR_CFG_ACC_DLPF_218HZ  << SENSOR_CFG_ACC_DLPF_OFFSET;
    const uint32_t b = (uint32_t)SENSOR_CFG_ACC_DLPF_218HZb << SENSOR_CFG_ACC_DLPF_OFFSET;
    EXPECT_NE(RenderStatusFromDecode(*dec, a), RenderStatusFromDecode(*dec, b));
}

TEST(ISStatusDecode, SensorConfig_MountingRotationAllTwentyFourValuesAreDistinct)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("sensorConfig");
    ASSERT_NE(dec, nullptr);
    std::vector<std::string> rendered;
    for (uint32_t r = 0; r < 24; ++r)
        rendered.push_back(RenderStatusFromDecode(*dec, r << SENSOR_CFG_SENSOR_ROTATION_OFFSET));
    for (size_t i = 0; i < rendered.size(); ++i) {
        EXPECT_FALSE(rendered[i].empty()) << "rotation " << i;
        for (size_t j = i + 1; j < rendered.size(); ++j)
            EXPECT_NE(rendered[i], rendered[j]) << "rotations " << i << " and " << j << " render identically";
    }
    // 25-31 (out of the 24 defined range but still representable in the 5-bit field) render nothing.
    EXPECT_EQ(RenderStatusFromDecode(*dec, 30u << SENSOR_CFG_SENSOR_ROTATION_OFFSET).find("Sensor rotation"), std::string::npos);
}

TEST(ISStatusDecode, SensorConfig_MagBaroDisableAndImuFaultDetectBits)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("sensorConfig");
    ASSERT_NE(dec, nullptr);
    const std::string mag  = RenderStatusFromDecode(*dec, (uint32_t)SENSOR_CFG_DISABLE_MAGNETOMETER);
    const std::string baro = RenderStatusFromDecode(*dec, (uint32_t)SENSOR_CFG_DISABLE_BAROMETER);
    const std::string gyrFd = RenderStatusFromDecode(*dec, (uint32_t)SENSOR_CFG_IMU_FAULT_DETECT_GYR);
    const std::string accFd = RenderStatusFromDecode(*dec, (uint32_t)SENSOR_CFG_IMU_FAULT_DETECT_ACC);
    EXPECT_NE(mag.find("Disable magnetometer sensor"), std::string::npos);
    EXPECT_NE(baro.find("Disable barometer sensor"), std::string::npos);
    EXPECT_NE(gyrFd.find("multiple-IMU gyro fault detection"), std::string::npos);
    EXPECT_NE(accFd.find("multiple-IMU accelerometer fault detection"), std::string::npos);
    EXPECT_NE(mag, baro);
    EXPECT_NE(gyrFd, accFd);
}

TEST(ISStatusDecode, SensorConfig_AllBitsSetDoesNotCrash)
{
    const status_field_decode_t* dec = GetStatusDecodeByField("sensorConfig");
    ASSERT_NE(dec, nullptr);
    const std::string rendered = RenderStatusFromDecode(*dec, 0xFFFFFFFFu);
    EXPECT_FALSE(rendered.empty());
}
