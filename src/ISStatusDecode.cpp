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
        "0x00000080 - Heading aided by GNSS"));
    d.subfields.push_back(bitField("GNSS position aiding", INS_STATUS_GNSS_AIDING_POS, false,
        "0x00000100 - Position aided by GNSS position"));
    d.subfields.push_back(bitField("GNSS update in solution", INS_STATUS_GNSS_UPDATE_IN_SOLUTION, false,
        "0x00000200 - GNSS update event occurred in solution, potentially causing discontinuity in position path"));
    d.subfields.push_back(bitField("Reference IMU in EKF", INS_STATUS_EKF_USING_REFERENCE_IMU, false,
        "0x00000400 - Reference IMU used in EKF"));
    d.subfields.push_back(bitField("Mag heading aiding", INS_STATUS_MAG_AIDING_HEADING, false,
        "0x00000800 - Heading aided by magnetic heading"));
    d.subfields.push_back(bitField("Nav mode", INS_STATUS_NAV_MODE, false,
        "0x00001000 - Nav Mode - estimating velocity and position."));
    d.subfields.push_back(bitField("Stationary mode", INS_STATUS_STATIONARY_MODE, false,
        "0x00002000 - INS in stationary mode."));
    d.subfields.push_back(bitField("GNSS velocity aiding", INS_STATUS_GNSS_AIDING_VEL, false,
        "0x00004000 - Velocity aided by GNSS velocity"));
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
        "0x00100000 - GNSS compassing antenna offsets are not set in flashCfg."));
    d.subfields.push_back(bitField("RTK compassing baseline bad", INS_STATUS_RTK_COMPASSING_BASELINE_BAD, true,
        "0x00200000 - GNSS antenna baseline specified in flashCfg and measured by GNSS do not match."));
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
        "0x00000010 - GNSS satellite signals are being received (antenna and cable are good)."));
    d.subfields.push_back(bitField("Strobe input event", HDW_STATUS_STROBE_IN_EVENT, false,
        "0x00000020 - Event occurred on strobe input pin."));
    d.subfields.push_back(bitField("GNSS ToW valid", HDW_STATUS_GNSS_TIME_OF_WEEK_VALID, false,
        "0x00000040 - GNSS time of week is valid and reported."));
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
        "0x00002000 - GNSS PPS timepulse signal has noise and occurred too frequently."));
    d.subfields.push_back(bitField("Mag recal complete", HDW_STATUS_MAG_RECAL_COMPLETE, false,
        "0x00004000 - Magnetometer recalibration has finished (when INS_STATUS_MAG_RECALIBRATING is unset)."));
    d.subfields.push_back(bitField("Flash write pending", HDW_STATUS_FLASH_WRITE_PENDING, false,
        "0x00008000 - System flash write staging or occurring now."));
    d.subfields.push_back(bitField("COM Tx limited", HDW_STATUS_ERR_COM_TX_LIMITED, true,
        "0x00010000 - Communications Tx buffer limited."));
    d.subfields.push_back(bitField("COM Rx overrun", HDW_STATUS_ERR_COM_RX_OVERRUN, true,
        "0x00020000 - Communications Rx buffer overrun."));
    d.subfields.push_back(bitField("GNSS PPS not received", HDW_STATUS_ERR_NO_GNSS_PPS, true,
        "0x00040000 - GNSS PPS timepulse signal has not been received or is in error."));
    d.subfields.push_back(bitField("GNSS PPS timesync", HDW_STATUS_GNSS_PPS_TIMESYNC, false,
        "0x00080000 - Time synchronized by GNSS PPS."));

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
        "0x00000020 - GNSS receiver critical fault (See the corresponding GNSS status fault flags)."));
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
        "0x02000000 - Fault: EKF GNSS time fault."));
    d.subfields.push_back(bitField("GPS receiver time fault", GFC_GNSS_RECEIVER_TIME, true,
        "0x04000000 - Fault: GNSS receiver time fault."));
    d.subfields.push_back(bitField("GNSS general fault", GFC_GNSS_GENERAL_FAULT, true,
        "0x08000000 - Fault: GNSS receiver general fault (See the corresponding GNSS status fault flags)."));
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
        // Gauge, not an error count: 0 sats = bad (red), many = good (green).
        // Ramp saturates to green at ~25 used satellites (a strong fix).
        s.countRampMax    = 25;
        s.countRampInvert = true;
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
            fix(GNSS_STATUS_FIX_GNSS_PLUS_DEAD_RECK,  "3D + dead reckoning",  "0x00000400 - 3D Fix + Dead Reckoning"),
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
    d.subfields.push_back(bitField("GNSS1 RTK raw data error", GNSS_STATUS_FLAGS_GNSS1_RTK_RAW_GNSS_DATA_ERROR, true,
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
        "0x00008000 - Data from NMEA message. GNSS velocity is NED (not ECEF)."));
    d.subfields.push_back(bitField("PPS timesync", GNSS_STATUS_FLAGS_GNSS_PPS_TIMESYNC, false,
        "0x10000000 - Time is synchronized by GNSS PPS."));
    d.subfields.push_back(bitField("ECEF covariance valid", GNSS_STATUS_FLAGS_RTK_COV_ECEF_PACKED_VALID, false, "0x20000000 - RTK ECEF covariance matrix is valid and packed in rel->covEcefPacked"));
    d.subfields.push_back(bitField("Unused 3", GNSS_STATUS_FLAGS_UNUSED_3, false, "0x40000000 - <<UNUSED>>"));
    d.subfields.push_back(bitField("Unused 4", GNSS_STATUS_FLAGS_UNUSED_4, false, "0x80000000 - <<UNUSED>>"));

    return d;
}

/** @brief GPX status decode table (eGpxStatus). Registered under key "gpxStatus" (field "status"). */
status_field_decode_t buildGpxStatusDecode()
{
    using K = eStatusSubfieldKind;
    status_field_decode_t d;
    d.fieldName = "status";   // registry key is "gpxStatus"
    d.errorMask = (uint32_t)GPX_STATUS_GENERAL_FAULT_MASK;

    auto gerr = [](const char* name, uint32_t mask, const char* legacy) {
        return bitField(name, mask, (mask & (uint32_t)GPX_STATUS_GENERAL_FAULT_MASK) != 0, legacy);
    };

    // Parse-error count is rendered by the legacy code as a presence flag (any of the low nibble),
    // not a number — model it as a Bit on the count mask.
    d.subfields.push_back(gerr("COM parse errors", GPX_STATUS_COM_PARSE_ERR_COUNT_MASK, "0x0000000F - Communications parse error count"));
    d.subfields.push_back(gerr("COM0 RX traffic lost", GPX_STATUS_COM0_RX_TRAFFIC_NOT_DETECTED, "0x00000010 - COM0 RX traffic not detected in last 30 seconds."));
    d.subfields.push_back(gerr("COM1 RX traffic lost", GPX_STATUS_COM1_RX_TRAFFIC_NOT_DETECTED, "0x00000020 - COM1 RX traffic not detected in last 30 seconds."));
    d.subfields.push_back(gerr("COM2 RX traffic lost", GPX_STATUS_COM2_RX_TRAFFIC_NOT_DETECTED, "0x00000040 - COM2 RX traffic not detected in last 30 seconds."));
    d.subfields.push_back(gerr("USB RX traffic lost", GPX_STATUS_USB_RX_TRAFFIC_NOT_DETECTED, "0x00000080 - USB RX traffic not detected in last 30 seconds."));
    d.subfields.push_back(gerr("Firmware image confirmed", GPX_STATUS_UPDATE_CONFIRMED, "0x00000100 - Update confirmed."));
    d.subfields.push_back(gerr("RTK buffer overflow", GPX_STATUS_FAULT_RTK_QUEUE_LIMITED, "0x00010000 - RTK buffer overflow."));
    d.subfields.push_back(gerr("GNSS receiver time fault", GPX_STATUS_FAULT_GNSS_RCVR_TIME, "0x00100000 - GNSS receiver time fault"));
    d.subfields.push_back(gerr("RTOS task period overrun", GPX_STATUS_FAULT_RTOS_TASK_PERIOD_OVERRUN, "0x00200000 - RTOS task period overrun"));   // added from SDK develop merge
    d.subfields.push_back(gerr("DMA fault", GPX_STATUS_FAULT_DMA, "0x00800000 - DMA fault"));

    // Fatal reset cause (Enum). Value 0 emits nothing.
    {
        const uint32_t mask  = (uint32_t)GPX_STATUS_FATAL_MASK;
        const uint32_t shift = maskShift(mask);
        auto fv = [](uint32_t v, const char* label, const char* legacy) {
            return status_value_label_t{ v, label, legacy, true };
        };
        status_subfield_t s;
        s.name    = "Fatal reset";
        s.kind    = K::Enum;
        s.mask    = mask;
        s.shift   = shift;
        s.isError = true;
        s.values = {
            fv(GPX_STATUS_FATAL_RESET_LOW_POW,       "Low power",          "0x01000000 - Reset from low power"),
            fv(GPX_STATUS_FATAL_RESET_BROWN,         "Brown out",          "0x02000000 - Reset from brown out"),
            fv(GPX_STATUS_FATAL_RESET_WATCHDOG,      "Watchdog",           "0x03000000 - Reset from watchdog"),
            fv(GPX_STATUS_FATAL_CPU_EXCEPTION,       "CPU exception",      "0x04000000 - CPU exception"),
            fv(GPX_STATUS_FATAL_UNHANDLED_INTERRUPT, "Unhandled interrupt","0x05000000 - Unhandled interrupt"),
            fv(GPX_STATUS_FATAL_STACK_OVERFLOW,      "Stack overflow",     "0x06000000 - Stack overflow"),
            fv(GPX_STATUS_FATAL_KERNEL_OOPS,         "Kernel oops",        "0x07000000 - Kernel oops"),
            fv(GPX_STATUS_FATAL_KERNEL_PANIC,        "Kernel panic",       "0x08000000 - Kernel panic"),
            fv(GPX_STATUS_FATAL_UNALIGNED_ACCESS,    "Unaligned access",   "0x09000000 - Unaligned access"),
            fv(GPX_STATUS_FATAL_MEMORY_ERROR,        "Memory error",       "0x0A000000 - Memory error"),
            fv(GPX_STATUS_FATAL_BUS_ERROR,           "Bus error",          "0x0B000000 - Bus error"),
            fv(GPX_STATUS_FATAL_USAGE_ERROR,         "Usage error",        "0x0C000000 - Usage error"),
            fv(GPX_STATUS_FATAL_DIV_ZERO,            "Division by zero",   "0x0D000000 - Division by zero"),
            fv(GPX_STATUS_FATAL_SER0_REINIT,         "SER0 reinit",        "0x0E000000 - SER0 reinit"),
            fv(GPX_STATUS_FATAL_UNKNOWN,             "Unknown",            "0x1F000000 - Unknown"),
        };
        d.subfields.push_back(s);
    }

    d.subfields.push_back(gerr("RP fault", GPX_STATUS_FAULT_RP, "0x20000000 - RP fault"));
    return d;
}

/**
 * @brief GPX hdwStatus decode table (eGPXHdwStatusFlags). Registered under key "gpxHdwStatus".
 *        All flat bits in legacy order — including the BIT running/passed/fault and reset-cause,
 *        which the legacy renderer tests individually (not as switches). isError is derived from
 *        GPX_HDW_STATUS_ERROR_MASK, with BIT_FAULT forced true (a failed self-test is an error
 *        even though the BIT bits aren't in the SDK error mask).
 */
status_field_decode_t buildGpxHdwStatusDecode()
{
    status_field_decode_t d;
    d.fieldName = "hdwStatus";   // registry key is "gpxHdwStatus"
    d.errorMask = (uint32_t)GPX_HDW_STATUS_ERROR_MASK;

    auto gbit = [](const char* name, uint32_t mask, const char* legacy) {
        return bitField(name, mask, (mask & (uint32_t)GPX_HDW_STATUS_ERROR_MASK) != 0, legacy);
    };

    d.subfields.push_back(gbit("GNSS1 satellite RX", GPX_HDW_STATUS_GNSS1_SATELLITE_RX,
        "0x00000001 - GNSS1 satellite signals are being received (antenna and cable are good)"));
    d.subfields.push_back(gbit("GNSS2 satellite RX", GPX_HDW_STATUS_GNSS2_SATELLITE_RX,
        "0x00000002 - GNSS2 satellite signals are being received (antenna and cable are good)"));
    d.subfields.push_back(gbit("GNSS1 ToW valid", GPX_HDW_STATUS_GNSS1_TIME_OF_WEEK_VALID,
        "0x00000004 - GNSS time of week is valid and reported.  Otherwise the timeOfWeek is local system time."));
    d.subfields.push_back(gbit("GNSS2 ToW valid", GPX_HDW_STATUS_GNSS2_TIME_OF_WEEK_VALID,
        "0x00000008 - GNSS time of week is valid and reported.  Otherwise the timeOfWeek is local system time."));
    d.subfields.push_back(gbit("GNSS1 init fault", GPX_HDW_STATUS_FAULT_GNSS1_INIT,
        "0x00000080 - Failed to communicate or setup GNSS receiver 1"));
    d.subfields.push_back(gbit("GNSS2 init fault", GPX_HDW_STATUS_FAULT_GNSS2_INIT,
        "0x00000800 - Failed to communicate or setup GNSS receiver 2"));
    d.subfields.push_back(gbit("GNSS FW update required", GPX_HDW_STATUS_GNSS_FW_UPDATE_REQUIRED,
        "0x00001000 - GNSS is faulting firmware update REQUIRED"));
    d.subfields.push_back(gbit("LED enabled", GPX_HDW_STATUS_LED_ENABLED,
        "0x00002000 - Enables LED in Manufacturing TBed"));
    d.subfields.push_back(gbit("System reset required", GPX_HDW_STATUS_SYSTEM_RESET_REQUIRED,
        "0x00004000 - System Reset is Required for proper function"));
    d.subfields.push_back(gbit("Flash write pending", GPX_HDW_STATUS_FLASH_WRITE_PENDING,
        "0x00008000 - System flash write staging or occuring now."));
    d.subfields.push_back(gbit("COM Tx limited", GPX_HDW_STATUS_ERR_COM_TX_LIMITED,
        "0x00010000 - Communications Tx buffer limited"));
    d.subfields.push_back(gbit("COM Rx overrun", GPX_HDW_STATUS_ERR_COM_RX_OVERRUN,
        "0x00020000 - Communications Rx buffer overrun"));
    d.subfields.push_back(gbit("GNSS1 no PPS", GPX_HDW_STATUS_ERR_NO_GNSS1_PPS,
        "0x00040000 - GNSS1 PPS timepulse signal has not been received or is in error"));
    d.subfields.push_back(gbit("GNSS2 no PPS", GPX_HDW_STATUS_ERR_NO_GNSS2_PPS,
        "0x00080000 - GNSS2 PPS timepulse signal has not been received or is in error"));
    d.subfields.push_back(gbit("GNSS1 low C/No", GPX_HDW_STATUS_ERR_LOW_CNO_GNSS1,
        "0x00100000 - GNSS1 signal strength low (<20)"));
    d.subfields.push_back(gbit("GNSS2 low C/No", GPX_HDW_STATUS_ERR_LOW_CNO_GNSS2,
        "0x00200000 - GNSS2 signal strength low (<20)"));
    d.subfields.push_back(gbit("GNSS1 C/No irregular", GPX_HDW_STATUS_ERR_CNO_GNSS1_IR,
        "0x00400000 - GNSS1 signal irregular. High Cno standard deviation over 5 second period detected."));
    d.subfields.push_back(gbit("GNSS2 C/No irregular", GPX_HDW_STATUS_ERR_CNO_GNSS2_IR,
        "0x00800000 - GNSS2 signal irregular. High Cno standard deviation over 5 second period detected."));
    d.subfields.push_back(gbit("BIT running", GPX_HDW_STATUS_BIT_RUNNING,
        "0x01000000 - (BIT) Built-in self-test running"));
    d.subfields.push_back(gbit("BIT passed", GPX_HDW_STATUS_BIT_PASSED,
        "0x02000000 - (BIT) Built-in self-test passed"));
    d.subfields.push_back(bitField("BIT fault", GPX_HDW_STATUS_BIT_FAULT, true,
        "0x03000000 - (BIT) Built-in self-test failure"));
    d.subfields.push_back(gbit("Temperature error", GPX_HDW_STATUS_ERR_TEMPERATURE,
        "0x04000000 - Temperature outside spec'd operating range"));
    d.subfields.push_back(gbit("GNSS PPS timesync", GPX_HDW_STATUS_GNSS_PPS_TIMESYNC,
        "0x08000000 - Time synchronized by GNSS PPS"));
    d.subfields.push_back(gbit("Reset cause: backup mode", GPX_HDW_STATUS_RESET_CAUSE_BACKUP_MODE,
        "0x10000000 - Reset from Backup mode (low-power state w/ CPU off)"));
    d.subfields.push_back(gbit("Reset cause: software", GPX_HDW_STATUS_RESET_CAUSE_SOFT,
        "0x20000000 - Reset from Software"));
    d.subfields.push_back(gbit("Reset cause: hardware", GPX_HDW_STATUS_RESET_CAUSE_HDW,
        "0x40000000 - Reset from Hardware (NRST pin low)"));
    d.subfields.push_back(gbit("Critical system fault", GPX_HDW_STATUS_FAULT_SYS_CRITICAL,
        "0x80000000 - Critical System Fault, CPU error."));
    return d;
}

/**
 * @brief Build a scalar-enum decode (single UINT8 value -> bare label, no newline) from a name
 *        array. `count` may be fewer than the array length to reproduce a legacy bound.
 */
status_field_decode_t makeScalarEnum(const char* fieldName, const char* const* names, uint32_t count)
{
    status_field_decode_t d;
    d.fieldName  = fieldName;
    d.scalarEnum = true;
    status_subfield_t s;
    s.name  = fieldName;
    s.kind  = eStatusSubfieldKind::Enum;
    s.mask  = 0xFFu;
    s.shift = 0;
    for (uint32_t i = 0; i < count; ++i)
        s.values.push_back({ i, names[i], std::string(), false });
    d.subfields.push_back(std::move(s));
    return d;
}

// --- GPX-GNSS per-receiver state enums (gpx_status_t.gnssStatus[N].*) --------------------------
// These render as a bare state name (no hex prefix / newline). Registered under semantic keys;
// the array-indexed field names ("gnssStatus0.initState", ...) resolve via GetStatusDecode.

status_field_decode_t buildGnssInitStateDecode()
{
    static const char* const initStates[] = {
        "Bootup", "UartSetting", "UartWait", "UartDone",
        "VersionCheck", "StopPos", "SetL5", "SetSats",
        "SetSatLimits", "SetOutput", "SetAlgo", "SetPeriod",
        "SetRtcmMsgs", "SetRtcmTimeMode", "SetPinningMode", "SetVelocitySmoothing",
        "SetAltituedSmoothing", "SetEphmOutputPeriod", "StartPos", "Done"
    };
    return makeScalarEnum("gnssInitState", initStates, 20);   // legacy bound: msgIdx <= 19
}

status_field_decode_t buildGnssRunStateDecode()
{
    static const char* const runStates[] = {
        "Reset", "Initializing", "Running", "Passthrough",
        "FwUpdate Init", "FwUpdate", "Error", "Shutdown",
        "ReInit", "Hard Reset"
    };
    return makeScalarEnum("gnssRunState", runStates, 10);     // legacy bound: msgIdx <= kHardReset(9)
}

status_field_decode_t buildGnssFwUpdateStateDecode()
{
    static const char* const fwStates[] = {
        "LockoutWait", "ResetSet", "ResetWait", "StartSet", "StartWait",
        "BootModeSet", "BootModeWait", "BaudSet", "BaudWait", "BaudFinish",
        "InjectWait", "InjectFinish", "ProgramExecutionWait", "ProgramExecutionFinish",
        "WriteNvmWait", "WriteNvmFinish", "Done",
    };
    // SN-7919: the original renderer bounded the index by `cxdRst_Max` (13) — a copy-paste bug
    // that left fwStates 13..16 (ProgramExecutionFinish..Done) undecoded. Fixed here to the array
    // size (17) so every state decodes. This is the one intentional behavior change vs the
    // original renderer (all other fields stay byte-identical).
    return makeScalarEnum("gnssFwUpdateState", fwStates, 17);
}

status_field_decode_t buildGnssLastResetCauseDecode()
{
    static const char* const rstReasons[] = {
        "Power On", "Watchdog", "ErrOpCode", "ErrorOpCode_FwUp",
        "ErrorOpCode_init", "UserRequested", "FWUpdate", "SysCmd",
        "InitTimeout", "Status5", "StatusNot0", "flashUpdate",
        "RTKEphMissing"
    };
    return makeScalarEnum("gnssLastResetCause", rstReasons, 13);  // legacy bound: msgIdx < cxdRst_Max(13)
}

// --- IMX built-in-test fields (DID_BIT) -------------------------------------------------------

status_field_decode_t buildImxHdwBitDecode()
{
    using K = eStatusSubfieldKind;
    status_field_decode_t d;
    d.fieldName = "hdwBitStatus";

    d.subfields.push_back(bitField("Passed all", HDW_BIT_PASSED_ALL, false,
        "0x00000001 - Passed all tests"));
    d.subfields.push_back(bitField("Passed without GPS", HDW_BIT_PASSED_NO_GNSS, false,
        "0x00000002 - Passed without valid GNSS signal"));
    {
        const uint32_t mask = (uint32_t)HDW_BIT_MODE_MASK;
        status_subfield_t s;
        s.name       = "BIT mode";
        s.kind       = K::Count;
        s.mask       = mask;
        s.shift      = maskShift(mask);
        s.modeHexDec = true;
        s.legacyText = "0x000000%x - BIT mode: %d";
        d.subfields.push_back(s);
    }
    d.subfields.push_back(bitField("FAULT: gyro noise", HDW_BIT_FAULT_NOISE_PQR, true,
        "0x00000100 - FAULT: Gyro noise"));
    d.subfields.push_back(bitField("FAULT: accel noise", HDW_BIT_FAULT_NOISE_ACC, true,
        "0x00000200 - FAULT: Accelerometer noise"));
    d.subfields.push_back(bitField("FAULT: magnetometer", HDW_BIT_FAULT_MAGNETOMETER, true,
        "0x00000400 - FAULT: Magnetometer"));
    d.subfields.push_back(bitField("FAULT: barometer", HDW_BIT_FAULT_BAROMETER, true,
        "0x00000800 - FAULT: Barometer"));
    d.subfields.push_back(bitField("FAULT: no GPS comms", HDW_BIT_FAULT_GNSS_NO_COM, true,
        "0x00001000 - FAULT: No GNSS serial communications"));
    d.subfields.push_back(bitField("FAULT: poor GPS C/No", HDW_BIT_FAULT_GNSS_POOR_CNO, true,
        "0x00002000 - FAULT: Poor GNSS signal strength"));
    d.subfields.push_back(bitField("FAULT: poor GPS accuracy", HDW_BIT_FAULT_GNSS_POOR_ACCURACY, true,
        "0x00004000 - FAULT: GNSS poor accuracy"));
    d.subfields.push_back(bitField("FAULT: GNSS noise", HDW_BIT_FAULT_GNSS_NOISE, true,
        "0x00008000 - FAULT: GNSS noise"));
    d.subfields.push_back(bitField("FAULT: IMU fault rejection", HDW_BIT_FAULT_IMU_FAULT_REJECTION, true,
        "0x00010000 - FAULT: IMU fault rejection failure"));
    d.subfields.push_back(bitField("FAULT: wrong hardware type", HDW_BIT_FAULT_INCORRECT_HARDWARE_TYPE, true,
        "0x01000000 - FAULT: Hardware type does not match firmware"));

    for (const auto& sf : d.subfields) if (sf.isError) d.errorMask |= sf.mask;
    return d;
}

status_field_decode_t buildImxCalBitDecode()
{
    using K = eStatusSubfieldKind;
    status_field_decode_t d;
    d.fieldName = "calBitStatus";

    d.subfields.push_back(bitField("Passed all", CAL_BIT_PASSED_ALL, false,
        "0x00000001 - Passed all calibration tests"));
    {
        const uint32_t mask = (uint32_t)CAL_BIT_MODE_MASK;
        status_subfield_t s;
        s.name       = "CAL BIT mode";
        s.kind       = K::Count;
        s.mask       = mask;
        s.shift      = maskShift(mask);
        s.modeHexDec = true;
        s.legacyText = "0x000000%x - CAL BIT mode: %d";
        d.subfields.push_back(s);
    }
    d.subfields.push_back(bitField("FAULT: temp cal absent", CAL_BIT_FAULT_TCAL_EMPTY, true,
        "0x00000100 - FAULT: Temperature calibration not present"));
    d.subfields.push_back(bitField("FAULT: temp cal range", CAL_BIT_FAULT_TCAL_TSPAN, true,
        "0x00000200 - FAULT: Temperature calibration range inadequate"));
    d.subfields.push_back(bitField("FAULT: temp cal inconsistent", CAL_BIT_FAULT_TCAL_INCONSISTENT, true,
        "0x00000400 - FAULT: Temperature calibration inconsistent"));
    d.subfields.push_back(bitField("FAULT: temp cal corrupt", CAL_BIT_FAULT_TCAL_CORRUPT, true,
        "0x00000800 - FAULT: Temperature calibration corrupt"));
    d.subfields.push_back(bitField("FAULT: gyro bias temp cal", CAL_BIT_FAULT_TCAL_PQR_BIAS, true,
        "0x00001000 - FAULT: Gyro bias temp cal"));
    d.subfields.push_back(bitField("FAULT: gyro slope temp cal", CAL_BIT_FAULT_TCAL_PQR_SLOPE, true,
        "0x00002000 - FAULT: Gyro slope temp cal"));
    d.subfields.push_back(bitField("FAULT: gyro linearity temp cal", CAL_BIT_FAULT_TCAL_PQR_LIN, true,
        "0x00004000 - FAULT: Gyro linearity temp cal"));
    d.subfields.push_back(bitField("FAULT: accel bias temp cal", CAL_BIT_FAULT_TCAL_ACC_BIAS, true,
        "0x00008000 - FAULT: Accel bias temp cal"));
    d.subfields.push_back(bitField("FAULT: accel slope temp cal", CAL_BIT_FAULT_TCAL_ACC_SLOPE, true,
        "0x00010000 - FAULT: Accel slope temp cal"));
    d.subfields.push_back(bitField("FAULT: accel linearity temp cal", CAL_BIT_FAULT_TCAL_ACC_LIN, true,
        "0x00020000 - FAULT: Accel linearity temp cal"));
    d.subfields.push_back(bitField("FAULT: cal serial mismatch", CAL_BIT_FAULT_CAL_SERIAL_NUM, true,
        "0x00040000 - FAULT: Calibration serial number mismatch"));
    d.subfields.push_back(bitField("FAULT: mag alignment invalid", CAL_BIT_FAULT_MCAL_MAG_INVALID, true,
        "0x00080000 - FAULT: Magnetometer cross-axis alignment invalid"));
    d.subfields.push_back(bitField("FAULT: motion cal absent", CAL_BIT_FAULT_MCAL_EMPTY, true,
        "0x00100000 - FAULT: Motion calibration not present"));
    d.subfields.push_back(bitField("FAULT: IMU alignment invalid", CAL_BIT_FAULT_MCAL_IMU_INVALID, true,
        "0x00200000 - FAULT: IMU cross-axis alignment invalid"));
    d.subfields.push_back(bitField("FAULT: motion on gyros", CAL_BIT_FAULT_MOTION_PQR, true,
        "0x00400000 - FAULT: Motion detected on gyros"));
    d.subfields.push_back(bitField("FAULT: motion on accels", CAL_BIT_FAULT_MOTION_ACC, true,
        "0x00800000 - FAULT: Motion detected on accelerometers"));
    d.subfields.push_back(bitField("NOTICE: IMU1 gyro bias", CAL_BIT_NOTICE_IMU1_PQR_BIAS, false,
        "0x01000000 - NOTICE: IMU 1 gyro bias offset detected"));
    d.subfields.push_back(bitField("NOTICE: IMU2 gyro bias", CAL_BIT_NOTICE_IMU2_PQR_BIAS, false,
        "0x02000000 - NOTICE: IMU 2 gyro bias offset detected"));
    d.subfields.push_back(bitField("NOTICE: IMU1 accel bias", CAL_BIT_NOTICE_IMU1_ACC_BIAS, false,
        "0x10000000 - NOTICE: IMU 1 accel bias offset detected"));
    d.subfields.push_back(bitField("NOTICE: IMU2 accel bias", CAL_BIT_NOTICE_IMU2_ACC_BIAS, false,
        "0x20000000 - NOTICE: IMU 2 accel bias offset detected"));

    for (const auto& sf : d.subfields) if (sf.isError) d.errorMask |= sf.mask;
    return d;
}

// --- GPX built-in-test fields (DID_GPX_BIT) ---------------------------------------------------

status_field_decode_t buildGpxBitResultsDecode()
{
    status_field_decode_t d;
    d.fieldName = "results";   // registry key "gpxBitResults"
    d.subfields.push_back(bitField("PPS1 passed", GPXBit_resultsBit_PPS1, false,     "0x01 - PPS1 test passed"));
    d.subfields.push_back(bitField("PPS2 passed", GPXBit_resultsBit_PPS2, false,     "0x02 - PPS2 test passed"));
    d.subfields.push_back(bitField("UART passed", GPXBit_resultsBit_UART, false,     "0x04 - UART test passed"));
    d.subfields.push_back(bitField("IO passed",   GPXBit_resultsBit_IO,   false,     "0x08 - IO test passed"));
    d.subfields.push_back(bitField("GPS passed",  GPXBit_resultsBit_GNSS, false,     "0x10 - GNSS test passed"));
    d.subfields.push_back(bitField("Finished",    GPXBit_resultsBit_FINISHED, false, "0x20 - Test finished"));
    d.subfields.push_back(bitField("Canceled",    GPXBit_resultsBit_CANCELED, false, "0x40 - Test canceled"));
    d.subfields.push_back(bitField("Error",       GPXBit_resultsBit_ERROR, true,     "0x80 - Test error"));
    d.errorMask = (uint32_t)GPXBit_resultsBit_ERROR;
    return d;
}

status_field_decode_t buildGpxBitStateDecode()
{
    status_field_decode_t d;
    d.fieldName = "state";   // registry key "gpxBitState"
    status_subfield_t s;
    s.name              = "BIT state";
    s.kind              = eStatusSubfieldKind::Enum;
    s.mask              = 0xFFu;
    s.shift             = 0;
    s.defaultLegacyText = "UNKNOWN(%d)";
    s.values = {
        { 0, "Not running", "NOT_RUNNING",  false },
        { 1, "Manuf init",  "MANUF_INIT",   false },
        { 2, "Manuf blink", "MANUF_BLINK",  false },
        { 3, "Manuf UART",  "MANUF_UART",   false },
        { 4, "Manuf IO",    "MANUF_IO",     false },
        { 5, "Manuf PPS",   "MANUF_PPS",    false },
        { 6, "Manuf GPS",   "MANUF_GPS",    false },
        { 7, "Manuf report","MANUF_REPORT", false },
    };
    d.subfields.push_back(std::move(s));
    return d;
}

/**
 * @brief imu_t / pimu_t `status` decode table (eImuStatus). Registered under the key "imuStatus";
 *        the on-wire field name "status" is shared across many DIDs, so GetStatusDecode routes the
 *        IMU/PIMU DIDs here explicitly. All masks derive from the data_sets.h symbols.
 */
status_field_decode_t buildImuStatusDecode()
{
    status_field_decode_t d;
    d.fieldName = "status";   // on-wire field name (registry key is "imuStatus")
    d.errorMask = (uint32_t)(IMU_STATUS_GYR_FAULT_REJECT | IMU_STATUS_ACC_FAULT_REJECT |
                             IMU_STATUS_SATURATION_GYR   | IMU_STATUS_SATURATION_ACC);

    // Per-axis sensor-OK bits (set = healthy; not errors).
    d.subfields.push_back(bitField("Gyro X OK", IMU_STATUS_GYR_X_OK, false, "0x00000001 - Gyro X sensor OK"));
    d.subfields.push_back(bitField("Gyro Y OK", IMU_STATUS_GYR_Y_OK, false, "0x00000002 - Gyro Y sensor OK"));
    d.subfields.push_back(bitField("Gyro Z OK", IMU_STATUS_GYR_Z_OK, false, "0x00000004 - Gyro Z sensor OK"));
    d.subfields.push_back(bitField("Accel X OK", IMU_STATUS_ACC_X_OK, false, "0x00000008 - Accel X sensor OK"));
    d.subfields.push_back(bitField("Accel Y OK", IMU_STATUS_ACC_Y_OK, false, "0x00000010 - Accel Y sensor OK"));
    d.subfields.push_back(bitField("Accel Z OK", IMU_STATUS_ACC_Z_OK, false, "0x00000020 - Accel Z sensor OK"));

    d.subfields.push_back(bitField("Shock present", IMU_STATUS_SHOCK_PRESENT, false,
        "0x00000040 - Sensor saturation / shock event present in this sample window"));
    d.subfields.push_back(bitField("Mag update", IMU_STATUS_MAG_UPDATE, false,
        "0x00000100 - Magnetometer sample updated"));
    d.subfields.push_back(bitField("Reference IMU present", IMU_STATUS_REFERENCE_IMU_PRESENT, false,
        "0x00000200 - Reference IMU data present"));

    d.subfields.push_back(bitField("Gyro fault reject", IMU_STATUS_GYR_FAULT_REJECT, true,
        "0x01000000 - Gyro sample(s) rejected by fault detection"));
    d.subfields.push_back(bitField("Accel fault reject", IMU_STATUS_ACC_FAULT_REJECT, true,
        "0x02000000 - Accel sample(s) rejected by fault detection"));
    d.subfields.push_back(bitField("Gyro saturation", IMU_STATUS_SATURATION_GYR, true,
        "0x40000000 - Gyro saturation occurred"));
    d.subfields.push_back(bitField("Accel saturation", IMU_STATUS_SATURATION_ACC, true,
        "0x80000000 - Accel saturation occurred"));
    return d;
}

/** @brief Process-wide registry of decode tables, keyed by an unambiguous internal key. Built once. */
const std::map<std::string, status_field_decode_t>& registry()
{
    static const std::map<std::string, status_field_decode_t> r = [] {
        std::map<std::string, status_field_decode_t> m;
        m.emplace("insStatus",          buildInsStatusDecode());
        m.emplace("imuStatus",          buildImuStatusDecode());
        m.emplace("hdwStatus",          buildHdwStatusDecode());
        m.emplace("sysStatus",          buildSysStatusDecode());
        m.emplace("genFaultCode",       buildGenFaultCodeDecode());
        m.emplace("gnssStatus",         buildGnssStatusDecode());
        m.emplace("gpxStatus",          buildGpxStatusDecode());
        m.emplace("gpxHdwStatus",       buildGpxHdwStatusDecode());
        m.emplace("gnssInitState",      buildGnssInitStateDecode());
        m.emplace("gnssRunState",       buildGnssRunStateDecode());
        m.emplace("gnssFwUpdateState",  buildGnssFwUpdateStateDecode());
        m.emplace("gnssLastResetCause", buildGnssLastResetCauseDecode());
        m.emplace("hdwBitStatus",       buildImxHdwBitDecode());
        m.emplace("calBitStatus",       buildImxCalBitDecode());
        m.emplace("gpxBitResults",      buildGpxBitResultsDecode());
        m.emplace("gpxBitState",        buildGpxBitStateDecode());
        return m;
    }();
    return r;
}

} // namespace

std::string RenderStatusFromDecode(const status_field_decode_t& dec, uint32_t value)
{
    // Scalar-enum fields render as a single bare label (no hex prefix, no trailing newline),
    // and an empty string when the value is out of range — matching the GPX-GNSS state renderers.
    if (dec.scalarEnum)
    {
        if (!dec.subfields.empty())
        {
            const auto& sf = dec.subfields.front();
            const uint32_t raw = (value & sf.mask) >> sf.shift;
            for (const auto& vl : sf.values)
                if (vl.value == raw)
                    return vl.legacyText.empty() ? vl.label : vl.legacyText;
        }
        return std::string();
    }

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
            bool matched = false;
            for (const auto& vl : sf.values)
            {
                if (vl.value == raw)
                {
                    buff << (vl.legacyText.empty() ? vl.label : vl.legacyText) << std::endl;
                    matched = true;
                    break;
                }
            }
            if (!matched && !sf.defaultLegacyText.empty())
                buff << utils::string_format(sf.defaultLegacyText, raw) << std::endl;
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
                    // modeHexDec: format the masked value (hex) and the shifted value (dec).
                    // Otherwise supply the value up to twice so single- and dual-substitution
                    // formats both work (a single-conversion format ignores the extra argument).
                    if (sf.modeHexDec)
                        buff << utils::string_format(fmt, (value & sf.mask), cnt) << std::endl;
                    else
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
    // The on-wire field names "status"/"hdwStatus"/"results"/"state" are shared across device
    // variants, so disambiguate by DID. Unambiguous names fall through to a direct key lookup.
    auto ends = [&](const char* suffix) {
        const std::string s(suffix);
        return fieldName.size() >= s.size() &&
               fieldName.compare(fieldName.size() - s.size(), s.size(), s) == 0;
    };

    if (did == DID_GPX_STATUS)
    {
        if (fieldName == "status")    return GetStatusDecodeByField("gpxStatus");
        if (fieldName == "hdwStatus") return GetStatusDecodeByField("gpxHdwStatus");
        // Per-receiver array fields: "gnssStatus<N>.<state>".
        if (ends("initState"))        return GetStatusDecodeByField("gnssInitState");
        if (ends("runState"))         return GetStatusDecodeByField("gnssRunState");
        if (ends("fwUpdateState"))    return GetStatusDecodeByField("gnssFwUpdateState");
        if (ends("lastRstCause"))     return GetStatusDecodeByField("gnssLastResetCause");
    }
    if (did == DID_GPX_BIT)
    {
        if (fieldName == "results")   return GetStatusDecodeByField("gpxBitResults");
        if (fieldName == "state")     return GetStatusDecodeByField("gpxBitState");
    }
    // IMU / PIMU `status` decodes via eImuStatus (shared on-wire name "status"); route these
    // explicitly so they don't fall into the GNSS catch-all below.
    if (fieldName == "status" &&
        (did == DID_IMU || did == DID_PIMU ||
         did == DID_REFERENCE_IMU || did == DID_REFERENCE_PIMU))
        return GetStatusDecodeByField("imuStatus");
    if (fieldName == "status")
        return GetStatusDecodeByField("gnssStatus");   // GNSS pos/vel status (DIDs 13/14/6/30/31/54)
    return GetStatusDecodeByField(fieldName);
}
