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

TEST(ISDataMappingsRenderExtended, GpxFlashCfg_SysCfgBits_DeliberatelyNotWiredYet)
{
    // Phase-2 scope boundary: gpx_flash_cfg_t::sysCfgBits uses its own enum (eGpxSysConfigBits)
    // and is NOT rendered by this ticket's phase-1 work -- it still falls through to the generic
    // renderVariableAndStatsToString formatter every field gets by default (a plain hex string),
    // NOT the IMX decoder (see the GetStatusDecode disambiguation this ticket added in
    // ISStatusDecode.cpp). Guards against it silently "fixing itself" by leaking the IMX table.
    const data_info_t* info = FindMappedField(DID_GPX_FLASH_CFG, "sysCfgBits");
    ASSERT_NE(info, nullptr);
    const std::string rendered = CallRenderExtended(*info, (uint32_t)SYS_CFG_BITS_AUTO_MAG_RECAL);
    EXPECT_EQ(rendered.find("Auto mag recal"), std::string::npos) << "leaked IMX-specific decoded text: \"" << rendered << "\"";
    EXPECT_NE(rendered.find("0x"), std::string::npos) << "expected the generic hex fallback, got: \"" << rendered << "\"";
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
