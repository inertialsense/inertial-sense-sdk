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

#include <cstdio>
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

/** @brief Process-wide registry of decode tables, keyed by field name. Built once. */
const std::map<std::string, status_field_decode_t>& registry()
{
    static const std::map<std::string, status_field_decode_t> r = [] {
        std::map<std::string, status_field_decode_t> m;
        m.emplace("insStatus", buildInsStatusDecode());
        m.emplace("hdwStatus", buildHdwStatusDecode());
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
            if (cnt)
            {
                if (!sf.legacyText.empty() && sf.legacyText.find("%d") != std::string::npos)
                {
                    char line[256];
                    std::snprintf(line, sizeof(line), sf.legacyText.c_str(), cnt);
                    buff << line << std::endl;
                }
                else
                {
                    buff << (sf.legacyText.empty() ? sf.name : sf.legacyText) << std::endl;
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

const status_field_decode_t* GetStatusDecode(uint32_t /*did*/, const std::string& fieldName)
{
    // v1: status semantics are DID-independent for shared fields (insStatus on DIDs 4/5/10/65/66
    // all decode identically). The `did` parameter is reserved for future DID-specific variants.
    return GetStatusDecodeByField(fieldName);
}
