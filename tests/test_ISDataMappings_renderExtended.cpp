/**
 * @file test_ISDataMappings_renderExtended.cpp
 * @brief Wiring-level tests for `data_info_t::renderExtended` assignments in ISDataMappings.cpp
 *        (SN-8491). See test_ISDataMappings_helpers.h for the harness and its rationale, and
 *        test_ISStatusDecode.cpp for table-level tests of the underlying decode tables.
 *
 * Each `RenderExtendedWiringCase` below is one field this ticket wires up. As later SN-8491
 * phases land (flash-config bitfields, GPX rtkMode/grmcBits, the RTKCfgBits migration), add rows
 * here rather than one-off tests -- this list is meant to be the single sweep asserting "every
 * field this ticket touches is actually reachable via NameToInfoMap() and renders something."
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>

#include <vector>

#include "test_ISDataMappings_helpers.h"
#include "ISStatusDecode.h"
#include "data_sets.h"

namespace {

const std::vector<RenderExtendedWiringCase>& WiringCases()
{
    static const std::vector<RenderExtendedWiringCase> cases = {
        { DID_GNSS1_POS, "status2", (uint8_t)GNSS_STATUS2_FLAGS_GNSS_JAM_DETECTED,   "jamming confirmed",  "DID_GNSS1_POS.status2" },
        { DID_GNSS2_POS, "status2", (uint8_t)GNSS_STATUS2_FLAGS_GNSS_SPOOF_DETECTED, "spoofing confirmed", "DID_GNSS2_POS.status2" },
        { DID_IMU,  "status", (uint32_t)IMU_STATUS_GYR_X_OK,         "Gyro X sensor OK",                     "DID_IMU.status" },
        { DID_PIMU, "status", (uint32_t)IMU_STATUS_ACC_FAULT_REJECT, "rejected by fault detection",          "DID_PIMU.status" },
        { DID_FLASH_CONFIG,   "gnssSatSigConst", (uint16_t)GNSS_SAT_SIG_CONST_GPS, "GPS", "DID_FLASH_CONFIG.gnssSatSigConst" },
        { DID_GPX_FLASH_CFG,  "gnssSatSigConst", (uint16_t)GNSS_SAT_SIG_CONST_GLO, "GLONASS", "DID_GPX_FLASH_CFG.gnssSatSigConst" },
        { DID_FLASH_CONFIG,   "dynamicModel", (uint8_t)DYNAMIC_MODEL_GROUND_VEHICLE, "Ground vehicle", "DID_FLASH_CONFIG.dynamicModel" },
        { DID_GPX_FLASH_CFG,  "dynamicModel", (uint8_t)DYNAMIC_MODEL_AIRBORNE_2G,    "Airborne <2g",   "DID_GPX_FLASH_CFG.dynamicModel" },
        { DID_FLASH_CONFIG,   "sysCfgBits", (uint32_t)SYS_CFG_BITS_AUTO_MAG_RECAL, "automatic mag recalibration", "DID_FLASH_CONFIG.sysCfgBits" },
        { DID_RMC,     "options", (uint32_t)RMC_OPTIONS_PORT_USB,     "USB",       "DID_RMC.options" },
        { DID_GPX_RMC, "options", (uint32_t)RMC_OPTIONS_PERSISTENT,   "persists across reboot", "DID_GPX_RMC.options" },
        { DID_FLASH_CONFIG,  "RTKCfgBits", (uint32_t)RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0, "GNSS1 UBLOX Ser0", "DID_FLASH_CONFIG.RTKCfgBits" },
        { DID_GPX_FLASH_CFG, "RTKCfgBits", (uint32_t)RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_MASK, "RTK Compassing", "DID_GPX_FLASH_CFG.RTKCfgBits" },
        { DID_GPX_STATUS, "grmcBitsSer0",     (uint64_t)GRMC_BITS_STATUS,   "GPX status", "DID_GPX_STATUS.grmcBitsSer0" },
        { DID_GPX_STATUS, "grmcNMEABitsSer0", (uint64_t)NMEA_RMC_BITS_GNGGA, "$GNGGA",     "DID_GPX_STATUS.grmcNMEABitsSer0" },
        { DID_RMC,     "bits", (uint64_t)RMC_BITS_GNSS1_POS, "GNSS1 position", "DID_RMC.bits" },
        { DID_GPX_RMC, "bits", (uint64_t)RMC_BITS_GPX_STATUS, "GPX status",    "DID_GPX_RMC.bits" },
        { DID_FLASH_CONFIG, "ioConfig",  (uint32_t)IO_CONFIG_IMU_1_DISABLE, "IMU 1 disable", "DID_FLASH_CONFIG.ioConfig" },
        { DID_FLASH_CONFIG, "ioConfig2", (uint8_t)IO_CFG2_USE_GNSS2_AS_SOURCE, "Use GNSS2", "DID_FLASH_CONFIG.ioConfig2" },
        { DID_FLASH_CONFIG, "sensorConfig", (uint32_t)SENSOR_CFG_DISABLE_MAGNETOMETER, "Disable magnetometer sensor", "DID_FLASH_CONFIG.sensorConfig" },
        { DID_GPX_FLASH_CFG, "sysCfgBits", (uint32_t)GPX_SYS_CFG_BITS_DISABLE_VCC_RF, "VCC_RF", "DID_GPX_FLASH_CFG.sysCfgBits" },
        { DID_GPX_STATUS, "rtkMode", (uint32_t)RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB, "GNSS2 RTCM3 USB", "DID_GPX_STATUS.rtkMode" },
    };
    return cases;
}

} // namespace

TEST(ISDataMappingsRenderExtended, AllSn8491FieldsAreWiredAndRenderNonEmpty)
{
    for (const auto& c : WiringCases())
        ExpectRenderExtendedWired(c);
}

TEST(ISDataMappingsRenderExtended, GnssStatus2_ZeroRendersEmpty)
{
    const data_info_t* info = FindMappedField(DID_GNSS1_POS, "status2");
    ASSERT_NE(info, nullptr);
    EXPECT_EQ(CallRenderExtended(*info, (uint8_t)0), "");
}

TEST(ISDataMappingsRenderExtended, GnssStatus2_MatchesDecodeTableDirectly)
{
    const data_info_t* info = FindMappedField(DID_GNSS1_POS, "status2");
    ASSERT_NE(info, nullptr);
    const status_field_decode_t* dec = GetStatusDecodeByField("status2");
    ASSERT_NE(dec, nullptr);

    const uint8_t value = (uint8_t)(GNSS_STATUS2_FLAGS_GNSS_POSSIBLE_JAM_DETECT | GNSS_STATUS2_FLAGS_GNSS_SPOOF_DETECTED);
    EXPECT_EQ(CallRenderExtended(*info, value), RenderStatusFromDecode(*dec, value));
}

TEST(ISDataMappingsRenderExtended, ImuStatus_ZeroRendersEmpty)
{
    const data_info_t* imu = FindMappedField(DID_IMU, "status");
    const data_info_t* pimu = FindMappedField(DID_PIMU, "status");
    ASSERT_NE(imu, nullptr);
    ASSERT_NE(pimu, nullptr);
    EXPECT_EQ(CallRenderExtended(*imu, (uint32_t)0), "");
    EXPECT_EQ(CallRenderExtended(*pimu, (uint32_t)0), "");
}

TEST(ISDataMappingsRenderExtended, ImuStatus_SharedRendererProducesIdenticalOutputForImuAndPimu)
{
    // DID_IMU and DID_PIMU both wire to the same renderer/table (renderImuStatus / "imuStatus"),
    // so the same raw value must render identically regardless of which DID it came from.
    const data_info_t* imu = FindMappedField(DID_IMU, "status");
    const data_info_t* pimu = FindMappedField(DID_PIMU, "status");
    ASSERT_NE(imu, nullptr);
    ASSERT_NE(pimu, nullptr);

    const uint32_t value = (uint32_t)IMU_STATUS_GYR_X_OK | (uint32_t)IMU_STATUS_SATURATION_ACC;
    EXPECT_EQ(CallRenderExtended(*imu, value), CallRenderExtended(*pimu, value));
    EXPECT_FALSE(CallRenderExtended(*imu, value).empty());
}

TEST(ISDataMappingsRenderExtended, ImuStatus_MatchesDecodeTableDirectly)
{
    const data_info_t* info = FindMappedField(DID_IMU, "status");
    ASSERT_NE(info, nullptr);
    const status_field_decode_t* dec = GetStatusDecodeByField("imuStatus");
    ASSERT_NE(dec, nullptr);

    const uint32_t value = (uint32_t)IMU_STATUS_ACC_Y_OK | (uint32_t)IMU_STATUS_GYR_FAULT_REJECT;
    EXPECT_EQ(CallRenderExtended(*info, value), RenderStatusFromDecode(*dec, value));
}

TEST(ISDataMappingsRenderExtended, GnssSatSigConstAndDynamicModel_SharedBetweenImxAndGpx)
{
    // nvm_flash_cfg_t and gpx_flash_cfg_t both wire the same renderer/table for these two fields
    // (same enums per the data_sets.h doc comments), so identical raw values must render
    // identically regardless of which struct/DID they came from.
    const data_info_t* imxSat  = FindMappedField(DID_FLASH_CONFIG, "gnssSatSigConst");
    const data_info_t* gpxSat  = FindMappedField(DID_GPX_FLASH_CFG, "gnssSatSigConst");
    const data_info_t* imxDyn  = FindMappedField(DID_FLASH_CONFIG, "dynamicModel");
    const data_info_t* gpxDyn  = FindMappedField(DID_GPX_FLASH_CFG, "dynamicModel");
    ASSERT_NE(imxSat, nullptr); ASSERT_NE(gpxSat, nullptr);
    ASSERT_NE(imxDyn, nullptr); ASSERT_NE(gpxDyn, nullptr);

    const uint16_t satValue = (uint16_t)(GNSS_SAT_SIG_CONST_GPS | GNSS_SAT_SIG_CONST_GAL);
    EXPECT_EQ(CallRenderExtended(*imxSat, satValue), CallRenderExtended(*gpxSat, satValue));
    EXPECT_FALSE(CallRenderExtended(*imxSat, satValue).empty());

    const uint8_t dynValue = (uint8_t)DYNAMIC_MODEL_MARINE;
    EXPECT_EQ(CallRenderExtended(*imxDyn, dynValue), CallRenderExtended(*gpxDyn, dynValue));
    EXPECT_EQ(CallRenderExtended(*imxDyn, dynValue), "Marine");
}

TEST(ISDataMappingsRenderExtended, GpxFlashCfg_SysCfgBits_UsesItsOwnTableNotImxDecoder)
{
    // gpx_flash_cfg_t::sysCfgBits uses its own enum (eGpxSysConfigBits) and its own renderer
    // (renderGpxSysCfgBits). Confirm IMX-specific decoded text never leaks in, even when fed a
    // raw value that happens to also set a bit meaningful in the (unrelated) IMX enum.
    const data_info_t* info = FindMappedField(DID_GPX_FLASH_CFG, "sysCfgBits");
    ASSERT_NE(info, nullptr);
    const std::string rendered = CallRenderExtended(*info, (uint32_t)SYS_CFG_BITS_AUTO_MAG_RECAL);
    EXPECT_EQ(rendered.find("Auto mag recal"), std::string::npos) << "leaked IMX-specific decoded text: \"" << rendered << "\"";
    // SYS_CFG_BITS_AUTO_MAG_RECAL (0x4) is not GPX_SYS_CFG_BITS_DISABLE_VCC_RF (0x1), so the GPX
    // table's own Bit subfield shouldn't fire either -- only the always-on brownout default.
    EXPECT_EQ(rendered.find("VCC_RF"), std::string::npos);
    EXPECT_NE(rendered.find("1.65-1.75V (default)"), std::string::npos);
}

TEST(ISDataMappingsRenderExtended, GpxSysCfgBits_MatchesDecodeTableDirectly)
{
    const data_info_t* info = FindMappedField(DID_GPX_FLASH_CFG, "sysCfgBits");
    ASSERT_NE(info, nullptr);
    const status_field_decode_t* dec = GetStatusDecodeByField("gpxSysCfgBits");
    ASSERT_NE(dec, nullptr);

    const uint32_t value = (uint32_t)GPX_SYS_CFG_BITS_DISABLE_VCC_RF;
    EXPECT_EQ(CallRenderExtended(*info, value), RenderStatusFromDecode(*dec, value));
}

TEST(ISDataMappingsRenderExtended, RmcOptions_SharedBetweenRmcAndGpxRmc)
{
    // DID_RMC and DID_GPX_RMC are both registered by the single PopulateMapRmc() function, so the
    // same raw options value must render identically regardless of which DID it came from.
    const data_info_t* rmc    = FindMappedField(DID_RMC, "options");
    const data_info_t* gpxRmc = FindMappedField(DID_GPX_RMC, "options");
    ASSERT_NE(rmc, nullptr);
    ASSERT_NE(gpxRmc, nullptr);

    const uint32_t value = (uint32_t)RMC_OPTIONS_PORT_SER0 | (uint32_t)RMC_OPTIONS_PRESERVE_CTRL;
    EXPECT_EQ(CallRenderExtended(*rmc, value), CallRenderExtended(*gpxRmc, value));
    EXPECT_FALSE(CallRenderExtended(*rmc, value).empty());
}

TEST(ISDataMappingsRenderExtended, RmcOptions_MatchesDecodeTableDirectly)
{
    const data_info_t* info = FindMappedField(DID_RMC, "options");
    ASSERT_NE(info, nullptr);
    const status_field_decode_t* dec = GetStatusDecodeByField("rmcOptions");
    ASSERT_NE(dec, nullptr);

    const uint32_t value = (uint32_t)RMC_OPTIONS_PORT_SER2 | (uint32_t)RMC_OPTIONS_PERSISTENT;
    EXPECT_EQ(CallRenderExtended(*info, value), RenderStatusFromDecode(*dec, value));
}

TEST(ISDataMappingsRenderExtended, NmeaMsgsOptions_DeliberatelyNotWiredYet)
{
    // nmea_msgs_t::options (DID_NMEA_BCAST_PERIOD) shares RMC_OPTIONS_* semantics but is a
    // different field/DID, out of this ticket's stated scope (DID_RMC/DID_GPX_RMC only). Confirm
    // it wasn't accidentally wired just by virtue of also being literally named "options".
    const data_info_t* info = FindMappedField(DID_NMEA_BCAST_PERIOD, "options");
    ASSERT_NE(info, nullptr);
    const std::string rendered = CallRenderExtended(*info, (uint32_t)RMC_OPTIONS_PORT_USB);
    EXPECT_EQ(rendered.find("USB"), std::string::npos) << "leaked decoded text: \"" << rendered << "\"";
    EXPECT_NE(rendered.find("0x"), std::string::npos) << "expected the generic hex fallback, got: \"" << rendered << "\"";
}

// ---- RTKCfgBits (flattened, list-style, Kyle 2026-08-13) ----------------------

TEST(ISDataMappingsRenderExtended, RTKCfgBits_SharedBetweenImxAndGpx)
{
    const data_info_t* imx = FindMappedField(DID_FLASH_CONFIG, "RTKCfgBits");
    const data_info_t* gpx = FindMappedField(DID_GPX_FLASH_CFG, "RTKCfgBits");
    ASSERT_NE(imx, nullptr);
    ASSERT_NE(gpx, nullptr);

    const uint32_t value = (uint32_t)RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB | (uint32_t)RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_MASK;
    EXPECT_EQ(CallRenderExtended(*imx, value), CallRenderExtended(*gpx, value));
    EXPECT_FALSE(CallRenderExtended(*imx, value).empty());
}

TEST(ISDataMappingsRenderExtended, RTKCfgBits_EachBitIndependentNoGating)
{
    // Flattened rendering: a per-port output bit renders on its own, with no dependency on
    // RTK_CFG_BITS_BASE_MODE or any GNSS1/GNSS2 UBLOX/RTCM3 sub-mask also being "set" as a
    // precondition (those are themselves just ORs of the per-port bits, not independent gates).
    const data_info_t* info = FindMappedField(DID_FLASH_CONFIG, "RTKCfgBits");
    ASSERT_NE(info, nullptr);
    const std::string rendered = CallRenderExtended(*info, (uint32_t)RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1);
    EXPECT_NE(rendered.find("GNSS1 RTCM3 Ser1"), std::string::npos);
    // The BASE_MODE summary line is present too, since this bit is part of that OR-mask.
    EXPECT_NE(rendered.find("RTK Base enabled"), std::string::npos);
}

TEST(ISDataMappingsRenderExtended, RTKCfgBits_ZeroRendersEmpty)
{
    const data_info_t* info = FindMappedField(DID_FLASH_CONFIG, "RTKCfgBits");
    ASSERT_NE(info, nullptr);
    EXPECT_EQ(CallRenderExtended(*info, (uint32_t)0), "");
}

// ---- grmcBits* / grmcNMEABits* (list-style, Kyle 2026-08-13) ------------------

TEST(ISDataMappingsRenderExtended, GrmcBits_SharedAcrossAllFourPorts)
{
    const data_info_t* ser0 = FindMappedField(DID_GPX_STATUS, "grmcBitsSer0");
    const data_info_t* ser1 = FindMappedField(DID_GPX_STATUS, "grmcBitsSer1");
    const data_info_t* ser2 = FindMappedField(DID_GPX_STATUS, "grmcBitsSer2");
    const data_info_t* usb  = FindMappedField(DID_GPX_STATUS, "grmcBitsUSB");
    ASSERT_NE(ser0, nullptr); ASSERT_NE(ser1, nullptr); ASSERT_NE(ser2, nullptr); ASSERT_NE(usb, nullptr);

    const uint64_t value = (uint64_t)GRMC_BITS_GNSS1_RAW | (uint64_t)GRMC_BITS_GPX_SYS_FAULT;
    const std::string rendered = CallRenderExtended(*ser0, value);
    EXPECT_EQ(rendered, CallRenderExtended(*ser1, value));
    EXPECT_EQ(rendered, CallRenderExtended(*ser2, value));
    EXPECT_EQ(rendered, CallRenderExtended(*usb, value));
    EXPECT_NE(rendered.find("GNSS1 raw"), std::string::npos);
    EXPECT_NE(rendered.find("GPX system fault"), std::string::npos);
}

TEST(ISDataMappingsRenderExtended, GrmcNmeaBits_SharedAcrossAllFourPorts)
{
    const data_info_t* ser0 = FindMappedField(DID_GPX_STATUS, "grmcNMEABitsSer0");
    const data_info_t* usb  = FindMappedField(DID_GPX_STATUS, "grmcNMEABitsUSB");
    ASSERT_NE(ser0, nullptr); ASSERT_NE(usb, nullptr);

    const uint64_t value = (uint64_t)NMEA_RMC_BITS_GNRMC | (uint64_t)NMEA_RMC_BITS_PIMU;
    const std::string rendered = CallRenderExtended(*ser0, value);
    EXPECT_EQ(rendered, CallRenderExtended(*usb, value));
    EXPECT_NE(rendered.find("$GNRMC"), std::string::npos);
    EXPECT_NE(rendered.find("$PIMU"), std::string::npos);
}

TEST(ISDataMappingsRenderExtended, GrmcBitsAndGrmcNmeaBits_ZeroRendersEmpty)
{
    const data_info_t* grmc = FindMappedField(DID_GPX_STATUS, "grmcBitsSer0");
    const data_info_t* nmea = FindMappedField(DID_GPX_STATUS, "grmcNMEABitsSer0");
    ASSERT_NE(grmc, nullptr);
    ASSERT_NE(nmea, nullptr);
    EXPECT_EQ(CallRenderExtended(*grmc, (uint64_t)0), "");
    EXPECT_EQ(CallRenderExtended(*nmea, (uint64_t)0), "");
}

// ---- rmc_t.bits (list-style, Kyle 2026-08-13) ---------------------------------

TEST(ISDataMappingsRenderExtended, RmcBits_SharedBetweenRmcAndGpxRmc)
{
    const data_info_t* rmc    = FindMappedField(DID_RMC, "bits");
    const data_info_t* gpxRmc = FindMappedField(DID_GPX_RMC, "bits");
    ASSERT_NE(rmc, nullptr);
    ASSERT_NE(gpxRmc, nullptr);

    const uint64_t value = (uint64_t)RMC_BITS_PIMU | (uint64_t)RMC_BITS_GPX_STATUS;
    const std::string rendered = CallRenderExtended(*rmc, value);
    EXPECT_EQ(rendered, CallRenderExtended(*gpxRmc, value));
    EXPECT_NE(rendered.find("PIMU"), std::string::npos);
    EXPECT_NE(rendered.find("GPX status"), std::string::npos);
}

TEST(ISDataMappingsRenderExtended, RmcBits_NotConfusedWithGrmcBits)
{
    // rmc_t.bits (RMC_BITS_*) and gpx_status_t.grmcBits* (GRMC_BITS_*) are different namespaces
    // despite the similar name -- RMC_BITS_GPX_STATUS (bit 46) and GRMC_BITS_STATUS (bit 2) both
    // decode to "GPX status" text but via completely different bit positions/renderers. Confirm
    // a raw grmcBits-shaped value fed to rmc_t.bits does NOT coincidentally decode the same way.
    const data_info_t* rmcBits  = FindMappedField(DID_RMC, "bits");
    const data_info_t* grmcBits = FindMappedField(DID_GPX_STATUS, "grmcBitsSer0");
    ASSERT_NE(rmcBits, nullptr);
    ASSERT_NE(grmcBits, nullptr);

    const uint64_t value = (uint64_t)GRMC_BITS_STATUS;   // bit 2 -- meaningless in RMC_BITS_* space
    EXPECT_NE(CallRenderExtended(*rmcBits, value), CallRenderExtended(*grmcBits, value));
}

TEST(ISDataMappingsRenderExtended, RmcBits_ZeroRendersEmpty)
{
    const data_info_t* info = FindMappedField(DID_RMC, "bits");
    ASSERT_NE(info, nullptr);
    EXPECT_EQ(CallRenderExtended(*info, (uint64_t)0), "");
}

// ---- ioConfig / ioConfig2 (IMX-only, SN-8491) ---------------------------------

TEST(ISDataMappingsRenderExtended, IoConfig_MatchesDecodeTableDirectly)
{
    const data_info_t* info = FindMappedField(DID_FLASH_CONFIG, "ioConfig");
    ASSERT_NE(info, nullptr);
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig");
    ASSERT_NE(dec, nullptr);

    const uint32_t value = (uint32_t)IO_CONFIG_G15_STROBE_INPUT | (uint32_t)IO_CONFIG_IMU_3_DISABLE;
    EXPECT_EQ(CallRenderExtended(*info, value), RenderStatusFromDecode(*dec, value));
}

TEST(ISDataMappingsRenderExtended, IoConfig2_MatchesDecodeTableDirectly)
{
    const data_info_t* info = FindMappedField(DID_FLASH_CONFIG, "ioConfig2");
    ASSERT_NE(info, nullptr);
    const status_field_decode_t* dec = GetStatusDecodeByField("ioConfig2");
    ASSERT_NE(dec, nullptr);

    const uint8_t value = (uint8_t)IO_CFG2_G13_XSDA_val;
    EXPECT_EQ(CallRenderExtended(*info, value), RenderStatusFromDecode(*dec, (uint32_t)value));
}

TEST(ISDataMappingsRenderExtended, IoConfig_NotWiredForGpxFlashCfg)
{
    // gpx_flash_cfg_t has no ioConfig/ioConfig2 field at all -- confirm looking it up on the GPX
    // DID simply fails cleanly rather than somehow resolving to the IMX field.
    const map_name_to_info_t* gpxMap = cISDataMappings::NameToInfoMap(DID_GPX_FLASH_CFG);
    ASSERT_NE(gpxMap, nullptr);
    EXPECT_EQ(gpxMap->find("ioConfig"), gpxMap->end());
    EXPECT_EQ(gpxMap->find("ioConfig2"), gpxMap->end());
}

TEST(ISDataMappingsRenderExtended, SensorConfig_MatchesDecodeTableDirectly)
{
    const data_info_t* info = FindMappedField(DID_FLASH_CONFIG, "sensorConfig");
    ASSERT_NE(info, nullptr);
    const status_field_decode_t* dec = GetStatusDecodeByField("sensorConfig");
    ASSERT_NE(dec, nullptr);

    const uint32_t value = (uint32_t)SENSOR_CFG_IMU_FAULT_DETECT_GYR | ((uint32_t)SENSOR_CFG_ACC_FS_8G << SENSOR_CFG_ACC_FS_OFFSET);
    EXPECT_EQ(CallRenderExtended(*info, value), RenderStatusFromDecode(*dec, value));
}

TEST(ISDataMappingsRenderExtended, SensorConfig_NotWiredForGpxFlashCfg)
{
    // gpx_flash_cfg_t has no sensorConfig field at all.
    const map_name_to_info_t* gpxMap = cISDataMappings::NameToInfoMap(DID_GPX_FLASH_CFG);
    ASSERT_NE(gpxMap, nullptr);
    EXPECT_EQ(gpxMap->find("sensorConfig"), gpxMap->end());
}

// ---- gpx_status_t::rtkMode (reuses eRTKConfigBits, Kyle 2026-08-13) -----------

TEST(ISDataMappingsRenderExtended, RtkMode_RendersIdenticallyToRTKCfgBitsForSameRawValue)
{
    // Per Kyle's 2026-08-13 confirmation, rtkMode's actual bit positions ARE eRTKConfigBits' --
    // the field's own pre-existing hand-written description string (now replaced) implied a
    // different, inconsistent layout with no enum ever backing it. Confirm the same raw value
    // decodes identically whether read from rtkMode or RTKCfgBits.
    const data_info_t* rtkMode    = FindMappedField(DID_GPX_STATUS, "rtkMode");
    const data_info_t* rtkCfgBits = FindMappedField(DID_FLASH_CONFIG, "RTKCfgBits");
    ASSERT_NE(rtkMode, nullptr);
    ASSERT_NE(rtkCfgBits, nullptr);

    const uint32_t value = (uint32_t)RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_MASK | (uint32_t)RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1;
    EXPECT_EQ(CallRenderExtended(*rtkMode, value), CallRenderExtended(*rtkCfgBits, value));
    EXPECT_FALSE(CallRenderExtended(*rtkMode, value).empty());
}

TEST(ISDataMappingsRenderExtended, RtkMode_ZeroRendersEmpty)
{
    const data_info_t* info = FindMappedField(DID_GPX_STATUS, "rtkMode");
    ASSERT_NE(info, nullptr);
    EXPECT_EQ(CallRenderExtended(*info, (uint32_t)0), "");
}
