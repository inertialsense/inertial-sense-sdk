/**
 * @file ISStatusDecode.cpp
 * @brief Structured status-field decode tables + table-driven renderer.
 *        See ISStatusDecode.h.
 *
 * @note D-53 / SN-7919.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISStatusDecode.h"

#include "data_sets.h"
#include "util/util.h"

#include <map>
#include <sstream>

namespace {

/** @brief Trailing-zero count of `mask` — the right-shift that lands a sub-field at bit 0. */
uint32_t maskShift(uint32_t mask)
{
    if (mask == 0) return 0;
    uint32_t s = 0;
    while (!(mask & 1u)) { mask >>= 1; ++s; }
    return s;
}

/** @brief Build a single on/off-bit sub-field descriptor. */
status_subfield_t bitField(const char* name, uint32_t mask, bool isError, const char* legacy)
{
    status_subfield_t s;
    s.name       = name;
    s.kind       = eStatusSubfieldKind::Bit;
    s.mask       = mask;
    s.shift      = 0;
    s.isError    = isError;
    s.gateMask   = 0;
    s.legacyText = legacy;
    return s;
}

/**
 * @brief insStatus decode table (eInsStatusFlags). Sub-fields are listed in the exact order the
 *        legacy `renderInsStatus` emitted them so `RenderStatusFromDecode` reproduces it byte-for-byte.
 *        All masks/shifts/values derive from the data_sets.h symbols — no transcribed hex.
 */
status_field_decode_t buildInsStatusDecode()
{
    using K = eStatusSubfieldKind;
    status_field_decode_t d;
    d.fieldName = "insStatus";
    d.errorMask = (uint32_t)INS_STATUS_ERROR_MASK;

    d.subfields.push_back(bitField("Heading align (coarse)", INS_STATUS_HDG_ALIGN_COARSE, false,
        "0x00000001 - Heading estimate is usable but outside spec (COARSE)"));
    d.subfields.push_back(bitField("Velocity align (coarse)", INS_STATUS_VEL_ALIGN_COARSE, false,
        "0x00000002 - Velocity estimate is usable but outside spec (COARSE)"));
    d.subfields.push_back(bitField("Position align (coarse)", INS_STATUS_POS_ALIGN_COARSE, false,
        "0x00000004 - Position estimate is usable but outside spec (COARSE)"));
    d.subfields.push_back(bitField("Wheel velocity aiding", INS_STATUS_WHEEL_AIDING_VEL, false,
        "0x00000008 - Velocity aided by wheel sensor"));
    d.subfields.push_back(bitField("Heading align (fine)", INS_STATUS_HDG_ALIGN_FINE, false,
        "0x00000010 - Heading estimate is within spec (FINE)."));
    d.subfields.push_back(bitField("Velocity align (fine)", INS_STATUS_VEL_ALIGN_FINE, false,
        "0x00000020 - Velocity estimate is within spec (FINE)"));
    d.subfields.push_back(bitField("Position align (fine)", INS_STATUS_POS_ALIGN_FINE, false,
        "0x00000040 - Position estimate is within spec (FINE)"));
    d.subfields.push_back(bitField("GNSS heading aiding", INS_STATUS_GNSS_AIDING_HEADING, false,
        "0x00000080 - Heading aided by GPS"));
    d.subfields.push_back(bitField("GNSS position aiding", INS_STATUS_GNSS_AIDING_POS, false,
        "0x00000100 - Position aided by GPS position"));
    d.subfields.push_back(bitField("GNSS update in solution", INS_STATUS_GNSS_UPDATE_IN_SOLUTION, false,
        "0x00000200 - GPS update event occurred in solution, potentially causing discontinuity in position path"));
    d.subfields.push_back(bitField("Reference IMU in EKF", INS_STATUS_EKF_USING_REFERENCE_IMU, false,
        "0x00000400 - Reference IMU used in EKF"));
    d.subfields.push_back(bitField("Mag heading aiding", INS_STATUS_MAG_AIDING_HEADING, false,
        "0x00000800 - Heading aided by magnetic heading"));
    d.subfields.push_back(bitField("Nav mode", INS_STATUS_NAV_MODE, false,
        "0x00001000 - Nav Mode - estimating velocity and position."));
    d.subfields.push_back(bitField("Stationary mode", INS_STATUS_STATIONARY_MODE, false,
        "0x00002000 - INS in stationary mode."));
    d.subfields.push_back(bitField("GNSS velocity aiding", INS_STATUS_GNSS_AIDING_VEL, false,
        "0x00004000 - Velocity aided by GPS velocity"));
    d.subfields.push_back(bitField("Kinematic cal good", INS_STATUS_KINEMATIC_CAL_GOOD, false,
        "0x00008000 - Vehicle kinematic calibration is good"));

    // Solution / nav stage (enum). Not in the error mask -> isError=false for every state.
    {
        status_subfield_t s;
        s.name     = "Solution mode";
        s.kind     = K::Enum;
        s.mask     = (uint32_t)INS_STATUS_SOLUTION_MASK;
        s.shift    = maskShift((uint32_t)INS_STATUS_SOLUTION_MASK);
        s.isError  = false;
        s.gateMask = 0;
        s.values = {
            { (uint32_t)INS_STATUS_SOLUTION_OFF,               "Off",                "0x000(0)0000 - System is off",                                                                          false },
            { (uint32_t)INS_STATUS_SOLUTION_ALIGNING,          "Aligning",           "0x000(1)0000 - System is in alignment mode",                                                            false },
            { (uint32_t)INS_STATUS_SOLUTION_NAV,               "Nav",                "0x000(3)0000 - System is in navigation mode",                                                           false },
            { (uint32_t)INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE, "Nav (high variance)","0x000(4)0000 - System is in navigation mode but the attitude uncertainty has exceeded the threshold.",  false },
            { (uint32_t)INS_STATUS_SOLUTION_AHRS,              "AHRS",               "0x000(5)0000 - System is in AHRS mode and solution is good.",                                           false },
            { (uint32_t)INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE,"AHRS (high variance)","0x000(6)0000 - System is in AHRS mode but the attitude uncertainty has exceeded the threshold.",       false },
            { (uint32_t)INS_STATUS_SOLUTION_VRS,               "VRS",                "0x000(7)0000 - System is in VRS mode (no earth relative heading) and roll and pitch are good.",          false },
            { (uint32_t)INS_STATUS_SOLUTION_VRS_HIGH_VARIANCE, "VRS (high variance)","0x000(8)0000 - System is in VRS mode (no earth relative heading) but roll and pitch uncertainty has exceeded the threshold.", false },
        };
        d.subfields.push_back(s);
    }

    d.subfields.push_back(bitField("RTK compassing baseline unset", INS_STATUS_RTK_COMPASSING_BASELINE_UNSET, true,
        "0x00100000 - GPS compassing antenna offsets are not set in flashCfg."));
    d.subfields.push_back(bitField("RTK compassing baseline bad", INS_STATUS_RTK_COMPASSING_BASELINE_BAD, true,
        "0x00200000 - GPS antenna baseline specified in flashCfg and measured by GPS do not match."));
    d.subfields.push_back(bitField("Mag recalibrating", INS_STATUS_MAG_RECALIBRATING, false,
        "0x00400000 - Magnetometer is being recalibrated."));
    d.subfields.push_back(bitField("Mag interference / bad cal", INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL_OR_NO_CAL, true,
        "0x00800000 - Magnetometer is experiencing interference or calibration is bad."));
    d.subfields.push_back(bitField("RTK compassing valid", INS_STATUS_RTK_COMPASSING_VALID, false,
        "0x04000000 - RTK compassing heading is accurate and aiding INS heading."));
    d.subfields.push_back(bitField("RTK raw GNSS data error", INS_STATUS_RTK_RAW_GNSS_DATA_ERROR, true,
        "0x08000000 - RTK error: Observations invalid or not received."));

    // RTK base-position error (enum) — hybrid: only decoded when any RTK error bit is set.
    {
        const uint32_t mask  = (uint32_t)INS_STATUS_RTK_ERR_BASE_MASK;
        const uint32_t shift = maskShift(mask);
        status_subfield_t s;
        s.name     = "RTK base error";
        s.kind     = K::Enum;
        s.mask     = mask;
        s.shift    = shift;
        s.isError  = true;
        s.gateMask = (uint32_t)INS_STATUS_RTK_ERROR_MASK;
        s.values = {
            { 0u,                                                          "No base position", "0x(0)0000000 - RTK error: NO base position received.",                                              true },
            { ((uint32_t)INS_STATUS_RTK_ERR_BASE_DATA_MISSING)     >> shift, "Base data missing","0x(1)0000000 - RTK error: Either base observations or antenna position have not been received.",   true },
            { ((uint32_t)INS_STATUS_RTK_ERR_BASE_POSITION_MOVING)  >> shift, "Base moving",      "0x(2)0000000 - RTK error: base position moved when it should be stationary.",                       true },
            { ((uint32_t)INS_STATUS_RTK_ERR_BASE_POSITION_INVALID) >> shift, "Base invalid",     "0x(3)0000000 - RTK error: base position invalid or not surveyed.",                                  true },
        };
        d.subfields.push_back(s);
    }

    d.subfields.push_back(bitField("RTOS task overrun", INS_STATUS_RTOS_TASK_PERIOD_OVERRUN, true,
        "0x40000000 - RTOS task ran longer than allotted period."));
    d.subfields.push_back(bitField("General fault", INS_STATUS_GENERAL_FAULT, true,
        "0x80000000 - General fault (see sys_params_t.genFaultCode)."));

    return d;
}

/**
 * @brief hdwStatus decode table (eHdwStatusFlags). Sub-fields in legacy `renderHdwStatus` emission
 *        order. Exercises all three kinds: bits, a Count (COM parse-error count), and two Enums
 *        (built-in-test state, reset cause). isError follows HDW_STATUS_ERROR_MASK membership.
 */
status_field_decode_t buildHdwStatusDecode()
{
    using K = eStatusSubfieldKind;
    status_field_decode_t d;
    d.fieldName = "hdwStatus";
    d.errorMask = (uint32_t)HDW_STATUS_ERROR_MASK;

    d.subfields.push_back(bitField("Gyro motion", HDW_STATUS_MOTION_GYR, false,
        "0x00000001 - Gyro motion detected."));
    d.subfields.push_back(bitField("Accel motion", HDW_STATUS_MOTION_ACC, false,
        "0x00000002 - Accelerometer motion detected."));
    d.subfields.push_back(bitField("IMU gyro fault reject", HDW_STATUS_IMU_FAULT_REJECT_GYR, true,
        "0x00000004 - IMU gyro fault rejection. A Gyro sensor is divergent and being excluded."));
    d.subfields.push_back(bitField("IMU accel fault reject", HDW_STATUS_IMU_FAULT_REJECT_ACC, true,
        "0x00000008 - IMU accelerometer fault rejection. An accelerometer sensors is divergent and being excluded."));
    d.subfields.push_back(bitField("GNSS satellite RX valid", HDW_STATUS_GNSS_SATELLITE_RX_VALID, false,
        "0x00000010 - GPS satellite signals are being received (antenna and cable are good)."));
    d.subfields.push_back(bitField("Strobe input event", HDW_STATUS_STROBE_IN_EVENT, false,
        "0x00000020 - Event occurred on strobe input pin."));
    d.subfields.push_back(bitField("GNSS ToW valid", HDW_STATUS_GNSS_TIME_OF_WEEK_VALID, false,
        "0x00000040 - GPS time of week is valid and reported."));
    d.subfields.push_back(bitField("Reference IMU RX", HDW_STATUS_REFERENCE_IMU_RX, false,
        "0x00000080 - Reference IMU data being received."));
    d.subfields.push_back(bitField("Saturation: gyro", HDW_STATUS_SATURATION_GYR, true,
        "0x00000100 - Sensor saturation on gyro."));
    d.subfields.push_back(bitField("Saturation: accel", HDW_STATUS_SATURATION_ACC, true,
        "0x00000200 - Sensor saturation on accelerometer."));
    d.subfields.push_back(bitField("Saturation: mag", HDW_STATUS_SATURATION_MAG, true,
        "0x00000400 - Sensor saturation on magnetometer."));
    d.subfields.push_back(bitField("Saturation: baro", HDW_STATUS_SATURATION_BARO, true,
        "0x00000800 - Sensor saturation on barometric pressure."));
    d.subfields.push_back(bitField("System reset required", HDW_STATUS_SYSTEM_RESET_REQUIRED, false,
        "0x00001000 - System Reset is required for proper function."));
    d.subfields.push_back(bitField("GNSS PPS noise", HDW_STATUS_ERR_GNSS_PPS_NOISE, true,
        "0x00002000 - GPS PPS timepulse signal has noise and occurred too frequently."));
    d.subfields.push_back(bitField("Mag recal complete", HDW_STATUS_MAG_RECAL_COMPLETE, false,
        "0x00004000 - Magnetometer recalibration has finished (when INS_STATUS_MAG_RECALIBRATING is unset)."));
    d.subfields.push_back(bitField("Flash write pending", HDW_STATUS_FLASH_WRITE_PENDING, false,
        "0x00008000 - System flash write staging or occurring now."));
    d.subfields.push_back(bitField("COM Tx limited", HDW_STATUS_ERR_COM_TX_LIMITED, true,
        "0x00010000 - Communications Tx buffer limited."));
    d.subfields.push_back(bitField("COM Rx overrun", HDW_STATUS_ERR_COM_RX_OVERRUN, true,
        "0x00020000 - Communications Rx buffer overrun."));
    d.subfields.push_back(bitField("GNSS PPS not received", HDW_STATUS_ERR_NO_GNSS_PPS, true,
        "0x00040000 - GPS PPS timepulse signal has not been received or is in error."));
    d.subfields.push_back(bitField("GNSS PPS timesync", HDW_STATUS_GNSS_PPS_TIMESYNC, false,
        "0x00080000 - Time synchronized by GPS PPS."));

    // COM parse-error count (Count). Legacy used utils::string_format with a %d.
    {
        const uint32_t mask = (uint32_t)HDW_STATUS_COM_PARSE_ERR_COUNT_MASK;
        status_subfield_t s;
        s.name       = "COM parse error count";
        s.kind       = K::Count;
        s.mask       = mask;
        s.shift      = maskShift(mask);
        s.isError    = false;   // not in HDW_STATUS_ERROR_MASK
        s.gateMask   = 0;
        s.legacyText = "0x00F00000 - Communications parse errors (%d).";
        d.subfields.push_back(s);
    }

    // Built-in self-test state (Enum). Value 0 (mask clear) emits nothing -> no 0 entry.
    {
        const uint32_t mask  = (uint32_t)HDW_STATUS_BIT_MASK;
        const uint32_t shift = maskShift(mask);
        status_subfield_t s;
        s.name     = "Built-in test (BIT)";
        s.kind     = K::Enum;
        s.mask     = mask;
        s.shift    = shift;
        s.isError  = true;   // FAILED is in the error mask
        s.gateMask = 0;
        s.values = {
            { ((uint32_t)HDW_STATUS_BIT_RUNNING) >> shift, "Running", "0x01000000 - (BIT) Built-in self-test is running.", false },
            { ((uint32_t)HDW_STATUS_BIT_PASSED)  >> shift, "Passed",  "0x02000000 - (BIT) Built-in self-test passed.",     false },
            { ((uint32_t)HDW_STATUS_BIT_FAILED)  >> shift, "Failed",  "0x03000000 - (BIT) Built-in self-test failed.",     true  },
        };
        d.subfields.push_back(s);
    }

    d.subfields.push_back(bitField("Temperature error", HDW_STATUS_ERR_TEMPERATURE, true,
        "0x04000000 - Temperature outside operating range."));
    d.subfields.push_back(bitField("SPI interface enabled", HDW_STATUS_SPI_INTERFACE_ENABLED, false,
        "0x08000000 - IMX pins G5-G8 are configure for SPI use."));

    // Reset cause (Enum). Value 0 emits nothing -> no 0 entry. Not in the error mask.
    {
        const uint32_t mask  = (uint32_t)HDW_STATUS_RESET_CAUSE_MASK;
        const uint32_t shift = maskShift(mask);
        status_subfield_t s;
        s.name     = "Reset cause";
        s.kind     = K::Enum;
        s.mask     = mask;
        s.shift    = shift;
        s.isError  = false;
        s.gateMask = 0;
        s.values = {
            { ((uint32_t)HDW_STATUS_RESET_CAUSE_BACKUP_MODE)    >> shift, "Backup mode", "0x10000000 - Reset from backup mode (low-power state w/ CPU off).", false },
            { ((uint32_t)HDW_STATUS_RESET_CAUSE_WATCHDOG_FAULT) >> shift, "Watchdog",    "0x20000000 - Reset from watchdog fault.",                          false },
            { ((uint32_t)HDW_STATUS_RESET_CAUSE_SOFT)           >> shift, "Software",    "0x30000000 - Reset from software.",                                false },
            { ((uint32_t)HDW_STATUS_RESET_CAUSE_HDW)            >> shift, "Hardware",    "0x40000000 - Reset from hardware.",                                false },
        };
        d.subfields.push_back(s);
    }

    d.subfields.push_back(bitField("Critical system fault", HDW_STATUS_FAULT_SYS_CRITICAL, true,
        "0x80000000 - Critical System Fault, CPU error (see DID_SYS_FAULT.status)."));

    return d;
}

/** @brief sysStatus decode table (eSysStatusFlags). Two informational bits; no error states. */
status_field_decode_t buildSysStatusDecode()
{
    status_field_decode_t d;
    d.fieldName = "sysStatus";
    d.errorMask = 0;
    d.subfields.push_back(bitField("Testbed-3 LEDs enabled", SYS_STATUS_TBED3_LEDS_ENABLED, false,
        "0x00000001 - IMX to drive Testbed-3 status LEDs."));
    d.subfields.push_back(bitField("Primary GNSS source is GNSS2", SYS_STATUS_PRIMARY_GNSS_SOURCE_IS_GNSS2, false,
        "0x00000004 - NMEA source is GNSS2."));
    return d;
}

/**
 * @brief genFaultCode decode table (eGenFaultCodes). All defined bits are fault conditions, so
 *        every subfield isError=true and errorMask = OR of all bits. Note the verbatim legacy
 *        text quirk on GNSS2 init ("init>").
 */
status_field_decode_t buildGenFaultCodeDecode()
{
    status_field_decode_t d;
    d.fieldName = "genFaultCode";

    d.subfields.push_back(bitField("INS state overrun: UVW", GFC_INS_STATE_ORUN_UVW, true,
        "0x00000001 - INS state limit overrun - UVW."));
    d.subfields.push_back(bitField("INS state overrun: Latitude", GFC_INS_STATE_ORUN_LAT, true,
        "0x00000002 - INS state limit overrun - Latitude."));
    d.subfields.push_back(bitField("INS state overrun: Altitude", GFC_INS_STATE_ORUN_ALT, true,
        "0x00000004 - INS state limit overrun - Altitude."));
    d.subfields.push_back(bitField("Unhandled interrupt", GFC_UNHANDLED_INTERRUPT, true,
        "0x00000010 - Unhandled interrupt."));
    d.subfields.push_back(bitField("GNSS critical fault", GFC_GNSS_CRITICAL_FAULT, true,
        "0x00000020 - GNSS receiver critical fault (See the corresponding GPS status fault flags)."));
    d.subfields.push_back(bitField("GNSS Tx limited", GFC_GNSS_TX_LIMITED, true,
        "0x00000040 - GNSS Tx limited."));
    d.subfields.push_back(bitField("GNSS Rx overrun", GFC_GNSS_RX_OVERRUN, true,
        "0x00000080 - GNSS Rx overrun."));
    d.subfields.push_back(bitField("Sensor init fault", GFC_INIT_SENSORS, true,
        "0x00000100 - Fault: sensor initialization."));
    d.subfields.push_back(bitField("SPI bus init fault", GFC_INIT_SPI, true,
        "0x00000200 - Fault: SPI bus initialization."));
    d.subfields.push_back(bitField("SPI config fault", GFC_CONFIG_SPI, true,
        "0x00000400 - Fault: SPI configuration."));
    d.subfields.push_back(bitField("GNSS1 init fault", GFC_GNSS1_INIT, true,
        "0x00000800 - Fault: GNSS1 init."));
    d.subfields.push_back(bitField("GNSS2 init fault", GFC_GNSS2_INIT, true,
        "0x00001000 - Fault: GNSS2 init>"));
    d.subfields.push_back(bitField("Flash invalid values", GFC_FLASH_INVALID_VALUES, true,
        "0x00002000 - Flash failed to load valid values."));
    d.subfields.push_back(bitField("Flash checksum failure", GFC_FLASH_CHECKSUM_FAILURE, true,
        "0x00004000 - Flash checksum failure."));
    d.subfields.push_back(bitField("Flash write failure", GFC_FLASH_WRITE_FAILURE, true,
        "0x00008000 - Flash write failure."));
    d.subfields.push_back(bitField("System fault: general", GFC_SYS_FAULT_GENERAL, true,
        "0x00010000 - System Fault: general."));
    d.subfields.push_back(bitField("System fault: critical", GFC_SYS_FAULT_CRITICAL, true,
        "0x00020000 - System Fault: CRITICAL system fault (see DID_SYS_FAULT)."));
    d.subfields.push_back(bitField("Sensor saturation", GFC_SENSOR_SATURATION, true,
        "0x00040000 - Sensor(s) saturated."));
    d.subfields.push_back(bitField("EKF states invalid", GFC_EKF_STATES_INVALID, true,
        "0x00080000 - EKF states invalid."));
    d.subfields.push_back(bitField("IMU init fault", GFC_INIT_IMU, true,
        "0x00100000 - Fault: IMU initialization."));
    d.subfields.push_back(bitField("Barometer init fault", GFC_INIT_BAROMETER, true,
        "0x00200000 - Fault: Barometer initialization."));
    d.subfields.push_back(bitField("Magnetometer init fault", GFC_INIT_MAGNETOMETER, true,
        "0x00400000 - Fault: Magnetometer initialization."));
    d.subfields.push_back(bitField("I2C init fault", GFC_INIT_I2C, true,
        "0x00800000 - Fault: I2C initialization."));
    d.subfields.push_back(bitField("Chip erase invalid", GFC_CHIP_ERASE_INVALID, true,
        "0x01000000 - Fault: Chip erase line toggled but did not meet required hold time."));
    d.subfields.push_back(bitField("EKF GPS time fault", GFC_EKF_GNSS_TIME_FAULT, true,
        "0x02000000 - Fault: EKF GPS time fault."));
    d.subfields.push_back(bitField("GPS receiver time fault", GFC_GNSS_RECEIVER_TIME, true,
        "0x04000000 - Fault: GPS receiver time fault."));
    d.subfields.push_back(bitField("GNSS general fault", GFC_GNSS_GENERAL_FAULT, true,
        "0x08000000 - Fault: GNSS receiver general fault (See the corresponding GPS status fault flags)."));
    d.subfields.push_back(bitField("EKF invalid IMU input", GFC_EKF_INPUT_INVALID_IMU, true,
        "0x10000000 - Fault: Invalid IMU input rejected by EKF."));

    // No single error-mask symbol exists; every defined bit is a fault, so the roll-up is their OR.
    d.errorMask = 0;
    for (const auto& sf : d.subfields) d.errorMask |= sf.mask;
    return d;
}

/**
 * @brief GNSS pos/vel `status` decode table (eGnssStatus). Registered under the unambiguous key
 *        "gnssStatus" (the on-wire field name "status" is shared with GPX — see GetStatusDecode).
 *        Sub-fields in legacy `renderGnssStatusBits` emission order.
 */
status_field_decode_t buildGnssStatusDecode()
{
    using K = eStatusSubfieldKind;
    status_field_decode_t d;
    d.fieldName = "status";   // on-wire field name (the registry key is "gnssStatus")
    d.errorMask = (uint32_t)GNSS_STATUS_FLAGS_ERROR_MASK;

    // Deprecated satellite count (low byte) — always emitted, dual-substitution format.
    {
        status_subfield_t s;
        s.name       = "Satellites used (deprecated)";
        s.kind       = K::Count;
        s.mask       = (uint32_t)GNSS_STATUS_NUM_SATS_USED_MASK;
        s.shift      = 0;
        s.isError    = false;
        s.emitZero   = true;
        s.legacyText = "0x000000%02X - %d satellites used in solution (deprecated)";
        d.subfields.push_back(s);
    }

    // Fix type (Enum). Includes value 0 ("No GNSS"); only 0..0xC are defined.
    {
        const uint32_t mask  = (uint32_t)GNSS_STATUS_FIX_MASK;
        const uint32_t shift = maskShift(mask);
        auto fix = [&](uint32_t sym, const char* label, const char* legacy) {
            return status_value_label_t{ ((uint32_t)sym & mask) >> shift, label, legacy, false };
        };
        status_subfield_t s;
        s.name  = "Fix type";
        s.kind  = K::Enum;
        s.mask  = mask;
        s.shift = shift;
        s.values = {
            fix(GNSS_STATUS_FIX_NONE,                "No fix",               "0x00000000 - No GNSS"),
            fix(GNSS_STATUS_FIX_DEAD_RECKONING_ONLY, "Dead reckoning only",  "0x00000100 - GNSS Dead Reckoning Only"),
            fix(GNSS_STATUS_FIX_2D,                  "2D fix",               "0x00000200 - 2D Fix"),
            fix(GNSS_STATUS_FIX_3D,                  "3D fix",               "0x00000300 - 3D Fix"),
            fix(GNSS_STATUS_FIX_GPS_PLUS_DEAD_RECK,  "3D + dead reckoning",  "0x00000400 - 3D Fix + Dead Reckoning"),
            fix(GNSS_STATUS_FIX_TIME_ONLY,           "Time only",            "0x00000500 - Time-Only Fix"),
            fix(GNSS_STATUS_FIX_REF_LLA,             "Reference LLA",        "0x00000600 - Usign Reference LLA"),
            fix(GNSS_STATUS_FIX_UNUSED2,             "Unused",               "0x00000700 - << UNUSED >>"),
            fix(GNSS_STATUS_FIX_DGPS,                "DGPS",                 "0x00000800 - Using DGPS"),
            fix(GNSS_STATUS_FIX_SBAS,                "SBAS",                 "0x00000900 - Using SBAS"),
            fix(GNSS_STATUS_FIX_RTK_SINGLE,          "RTK single",           "0x00000A00 - RTK Single"),
            fix(GNSS_STATUS_FIX_RTK_FLOAT,           "RTK float",            "0x00000B00 - RTK Float"),
            fix(GNSS_STATUS_FIX_RTK_FIX,             "RTK fix",              "0x00000C00 - RTK Fix"),
        };
        d.subfields.push_back(s);
    }

    d.subfields.push_back(bitField("Fix OK", GNSS_STATUS_FLAGS_FIX_OK, false,
        "0x00010000 - within limits (e.g. DOP & accuracy)"));
    d.subfields.push_back(bitField("DGPS used", GNSS_STATUS_FLAGS_DGPS_USED, false,
        "0x00020000 - Differential GPS (DGPS) used."));
    d.subfields.push_back(bitField("RTK fix and hold", GNSS_STATUS_FLAGS_RTK_FIX_AND_HOLD, false,
        "0x00040000 - RTK feedback on the integer solutions to drive the float biases towards the resolved integers"));
    d.subfields.push_back(bitField("Unused 1", GNSS_STATUS_FLAGS_UNUSED_1, false,
        "0x00080000 - << UNUSED >>"));
    d.subfields.push_back(bitField("GNSS1 RTK positioning enabled", GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_ENABLED, false,
        "0x00100000 - GNSS1 RTK precision positioning mode enabled"));
    d.subfields.push_back(bitField("Static mode", GNSS_STATUS_FLAGS_STATIC_MODE, false,
        "0x00200000 - Static mode"));
    d.subfields.push_back(bitField("GNSS2 RTK compassing enabled", GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_ENABLED, false,
        "0x00400000 - GNSS2 RTK moving base mode enabled"));
    d.subfields.push_back(bitField("GNSS1 RTK raw data error", GNSS_STATUS_FLAGS_GNSS1_RTK_RAW_GPS_DATA_ERROR, true,
        "0x00800000 - GNSS1 RTK error: observations or ephemeris are invalid or not received (i.e. RTK differential corrections)"));

    // RTK base error (Enum). Legacy switches on (value & FLAGS_ERROR_MASK), which folds in the
    // raw-data-error bit — so when that bit is also set, no case matches and nothing emits. Using
    // mask=FLAGS_ERROR_MASK reproduces that exactly: only raw values {2,4,6} resolve to a label.
    {
        const uint32_t mask  = (uint32_t)GNSS_STATUS_FLAGS_ERROR_MASK;
        const uint32_t shift = maskShift(mask);
        auto be = [&](uint32_t sym, const char* label, const char* legacy) {
            return status_value_label_t{ ((uint32_t)sym & mask) >> shift, label, legacy, true };
        };
        status_subfield_t s;
        s.name    = "RTK base error";
        s.kind    = K::Enum;
        s.mask    = mask;
        s.shift   = shift;
        s.isError = true;
        s.values = {
            be(GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_DATA_MISSING,     "Base data missing", "0x01000000 - GNSS1 RTK error: Either base observations or antenna position have not been received."),
            be(GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_MOVING,  "Base moving",       "0x02000000 - GNSS1 RTK error: base position moved when it should be stationary"),
            be(GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_INVALID, "Base invalid",      "0x03000000 - GNSS1 RTK error: base position is invalid or not surveyed well"),
        };
        d.subfields.push_back(s);
    }

    d.subfields.push_back(bitField("GNSS1 RTK position valid", GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_VALID, false,
        "0x04000000 - GNSS1 RTK precision position and carrier phase range solution with fixed ambiguities."));

    // GNSS2 compass block — only emitted when any compass bit is set (gated on COMPASS_MASK).
    {
        const uint32_t gate = (uint32_t)GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_MASK;
        auto compassBit = [&](const char* name, uint32_t mask, const char* legacy) {
            status_subfield_t s = bitField(name, mask, false, legacy);
            s.gateMask = gate;
            return s;
        };
        d.subfields.push_back(compassBit("GNSS2 RTK compassing valid", GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_VALID,
            "0x08000000 - GNSS2 RTK moving base heading valid and available in DID_GNSS2_RTK_CMP_REL."));
        d.subfields.push_back(compassBit("GNSS2 compassing baseline bad", GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_BAD,
            "0x00002000 - GNSS2 RTK Compassing Baseline distance is invalid"));
        d.subfields.push_back(compassBit("GNSS2 compassing baseline unset", GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_UNSET,
            "0x00004000 - GNSS2 RTK Compassing Baseline distance is unset (must be > 0)"));
    }

    d.subfields.push_back(bitField("NMEA data", GNSS_STATUS_FLAGS_GNSS_NMEA_DATA, false,
        "0x00008000 - Data from NMEA message. GPS velocity is NED (not ECEF)."));
    d.subfields.push_back(bitField("PPS timesync", GNSS_STATUS_FLAGS_GNSS_PPS_TIMESYNC, false,
        "0x10000000 - Time is synchronized by GPS PPS."));
    d.subfields.push_back(bitField("Unused 2", GNSS_STATUS_FLAGS_UNUSED_2, false, "0x20000000 - <<UNUSED>>"));
    d.subfields.push_back(bitField("Unused 3", GNSS_STATUS_FLAGS_UNUSED_3, false, "0x40000000 - <<UNUSED>>"));
    d.subfields.push_back(bitField("Unused 4", GNSS_STATUS_FLAGS_UNUSED_4, false, "0x80000000 - <<UNUSED>>"));

    return d;
}

/** @brief Process-wide registry of decode tables, keyed by an unambiguous internal key. Built once. */
const std::map<std::string, status_field_decode_t>& registry()
{
    static const std::map<std::string, status_field_decode_t> r = [] {
        std::map<std::string, status_field_decode_t> m;
        m.emplace("insStatus",    buildInsStatusDecode());
        m.emplace("hdwStatus",    buildHdwStatusDecode());
        m.emplace("sysStatus",    buildSysStatusDecode());
        m.emplace("genFaultCode", buildGenFaultCodeDecode());
        m.emplace("gnssStatus",   buildGnssStatusDecode());
        return m;
    }();
    return r;
}

} // namespace

std::string RenderStatusFromDecode(const status_field_decode_t& dec, uint32_t value)
{
    std::stringstream buff;
    for (const auto& sf : dec.subfields)
    {
        if (sf.gateMask && !(value & sf.gateMask)) continue;

        switch (sf.kind)
        {
        case eStatusSubfieldKind::Bit:
            if (value & sf.mask)
                buff << (sf.legacyText.empty() ? sf.name : sf.legacyText) << std::endl;
            break;

        case eStatusSubfieldKind::Enum:
        {
            const uint32_t raw = (value & sf.mask) >> sf.shift;
            for (const auto& vl : sf.values)
            {
                if (vl.value == raw)
                {
                    buff << (vl.legacyText.empty() ? vl.label : vl.legacyText) << std::endl;
                    break;
                }
            }
            break;
        }

        case eStatusSubfieldKind::Count:
        {
            const uint32_t cnt = (value & sf.mask) >> sf.shift;
            if (cnt || sf.emitZero)
            {
                const std::string& fmt = sf.legacyText.empty() ? sf.name : sf.legacyText;
                if (fmt.find('%') != std::string::npos)
                {
                    // Supply the value up to twice so single- and dual-substitution formats both
                    // work (a single-conversion format harmlessly ignores the extra argument).
                    buff << utils::string_format(fmt, cnt, cnt) << std::endl;
                }
                else
                {
                    buff << fmt << std::endl;
                }
            }
            break;
        }
        }
    }
    return buff.str();
}

const status_field_decode_t* GetStatusDecodeByField(const std::string& fieldName)
{
    const auto& r = registry();
    auto it = r.find(fieldName);
    return (it == r.end()) ? nullptr : &it->second;
}

const status_field_decode_t* GetStatusDecode(uint32_t did, const std::string& fieldName)
{
    // The on-wire field names "status" and "hdwStatus" are shared across device variants
    // (GNSS pos/vel vs GPX; IMX vs GPX), so disambiguate by DID. Unambiguous field names
    // (insStatus, sysStatus, genFaultCode, ...) fall through to a direct key lookup.
    if (did == DID_GPX_STATUS)
    {
        if (fieldName == "status")    return GetStatusDecodeByField("gpxStatus");
        if (fieldName == "hdwStatus") return GetStatusDecodeByField("gpxHdwStatus");
    }
    if (fieldName == "status")
        return GetStatusDecodeByField("gnssStatus");   // GNSS pos/vel status (DIDs 13/14/6/30/31/54)
    return GetStatusDecodeByField(fieldName);
}
