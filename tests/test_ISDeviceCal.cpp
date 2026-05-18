/**
 * @file test_ISDeviceCal.cpp
 * @brief Unit tests for ISDeviceCal calibration load/save refactoring
 *
 * Tests the three loading methods:
 *   - loadCalibrationFromJsonFile() (file path)
 *   - loadCalibrationFromJsonObj() (pre-parsed json object)
 *   - loadCalibrationFromJsonString() (JSON string)
 *
 * Also verifies round-trip: save → load → compare
 *
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <fstream>
#include <cstring>
#include <cstdio>

#include <gtest/gtest.h>
#include "gtest_helpers.h"

#include "ISDeviceCal.h"
#include "ISLogFile.h"
#include "IS_calibration_convert.h"
#include "data_sets.h"
#include "json.hpp"

using json = nlohmann::json;

// Helper: build a minimal but valid calibration JSON object
static json makeTestCalJson(uint32_t serialNum = 12345)
{
    json jObj;

    // Info section
    json jInfo;
    jInfo["version"] = "1.2.3 0";
    jInfo["calDate"] = "2025-06-15 0";
    jInfo["calTime"] = "14:30:00 0";
    jInfo["devSerialNum"] = (int)serialNum;
    jObj["info"] = jInfo;

    // Minimal tempComp section with 2 points for gyr1
    json jTC;
    json jGyr1 = json::array();
    for (int i = 0; i < 2; i++)
    {
        json pt;
        pt["index"] = i;
        pt["tmp"] = 25.0f + i * 10.0f;
        pt["SS"] = json::array({0.1f * (i + 1), 0.2f * (i + 1), 0.3f * (i + 1)});
        jGyr1.push_back(pt);
    }
    jTC["gyr1"] = jGyr1;
    jObj["tempComp"] = jTC;

    return jObj;
}

// Helper: write JSON to a temp file, return path
static std::string writeTempCalFile(const json& jObj, const std::string& filename = "test_cal.json")
{
    std::string path = "/tmp/" + filename;
    std::ofstream f(path);
    f << jObj.dump(2);
    f.close();
    return path;
}

class test_ISDeviceCal : public ::testing::Test {
protected:
    void SetUp() override {
        memset(&info, 0, sizeof(info));
        memset(&dinfo, 0, sizeof(dinfo));
        memset(&tcal, 0, sizeof(tcal));
        memset(&mcal, 0, sizeof(mcal));
    }

    sensor_cal_info_t info = {};
    sensor_data_info_t dinfo = {};
    sensor_tcal_group_t tcal = {};
    sensor_mcal_group_t mcal = {};
};

// ---- loadCalibrationFromJsonFile tests ----

TEST_F(test_ISDeviceCal, loadFromFile_validJson)
{
    json jObj = makeTestCalJson(99001);
    std::string path = writeTempCalFile(jObj, "test_cal_file.json");

    bool result = ISDeviceCal::loadCalibrationFromJsonFile(path, NULL, &info, &dinfo, &tcal, &mcal);
    ASSERT_TRUE(result);

    // Verify info was populated
    EXPECT_EQ(info.version[0], 1);
    EXPECT_EQ(info.version[1], 2);
    EXPECT_EQ(info.version[2], 3);
    EXPECT_EQ(info.devSerialNum, 99001u);

    // Verify date
    EXPECT_EQ(info.calDate[0], 25);  // 2025 - 2000
    EXPECT_EQ(info.calDate[1], 6);
    EXPECT_EQ(info.calDate[2], 15);

    // Verify tcal was populated
    EXPECT_EQ(tcal.gyr[0].numPts, 2u);
    EXPECT_NEAR(tcal.gyr[0].pt[0].temp, 25.0f, 0.001f);
    EXPECT_NEAR(tcal.gyr[0].pt[1].temp, 35.0f, 0.001f);

    std::remove(path.c_str());
}

TEST_F(test_ISDeviceCal, loadFromFile_missingFile)
{
    bool result = ISDeviceCal::loadCalibrationFromJsonFile("/tmp/nonexistent_cal_file.json", NULL, &info);
    EXPECT_FALSE(result);
}

TEST_F(test_ISDeviceCal, loadFromFile_invalidJson)
{
    std::string path = "/tmp/test_cal_invalid.json";
    std::ofstream f(path);
    f << "{ this is not valid json }}}";
    f.close();

    bool result = ISDeviceCal::loadCalibrationFromJsonFile(path, NULL, &info);
    EXPECT_FALSE(result);

    std::remove(path.c_str());
}

TEST_F(test_ISDeviceCal, loadFromFile_fallbackFilename)
{
    // JSON without "info" section — should extract serial/date from filename
    json jObj;
    jObj["tempComp"] = json::object();  // minimal non-empty
    std::string path = writeTempCalFile(jObj, "tc_SN12345_20250615_1430.json");

    bool result = ISDeviceCal::loadCalibrationFromJsonFile(path, NULL, &info);
    ASSERT_TRUE(result);

    EXPECT_EQ(info.devSerialNum, 12345u);
    EXPECT_EQ(info.calDate[0], 25);  // 2025 - 2000
    EXPECT_EQ(info.calDate[1], 6);
    EXPECT_EQ(info.calDate[2], 15);
    EXPECT_EQ(info.calTime[0], 14);
    EXPECT_EQ(info.calTime[1], 30);

    std::remove(path.c_str());
}

// ---- loadCalibrationFromJsonObj tests ----

TEST_F(test_ISDeviceCal, loadFromJsonObj_validJson)
{
    json jObj = makeTestCalJson(88001);

    bool result = ISDeviceCal::loadCalibrationFromJsonObj(jObj, NULL, &info, &dinfo, &tcal, &mcal);
    ASSERT_TRUE(result);

    EXPECT_EQ(info.devSerialNum, 88001u);
    EXPECT_EQ(info.version[0], 1);
    EXPECT_EQ(tcal.gyr[0].numPts, 2u);
}

TEST_F(test_ISDeviceCal, loadFromJsonObj_emptyJson)
{
    json jObj = json::object();
    // Empty object with no sections — should return false
    bool result = ISDeviceCal::loadCalibrationFromJsonObj(jObj, NULL, &info);
    EXPECT_FALSE(result);
}

TEST_F(test_ISDeviceCal, loadFromJsonObj_noInfoNoFilePath)
{
    // JSON without "info" section and no filePath — info should remain zeroed (no filename fallback)
    json jObj;
    jObj["tempComp"] = json::object();

    memset(&info, 0xFF, sizeof(info));  // fill with garbage to verify it gets cleared/set
    bool result = ISDeviceCal::loadCalibrationFromJsonObj(jObj, NULL, &info, &dinfo, &tcal, &mcal);
    ASSERT_TRUE(result);

    // Without filePath and no "info" in JSON, info fields should not be populated from filename
    // The info size and checksum should still be computed
    EXPECT_EQ(info.size, sizeof(sensor_cal_info_t));
}

// ---- loadCalibrationFromJsonString tests ----

TEST_F(test_ISDeviceCal, loadFromJsonString_validJson)
{
    json jObj = makeTestCalJson(77001);
    std::string jsonStr = jObj.dump();

    bool result = ISDeviceCal::loadCalibrationFromJsonString(jsonStr, NULL, &info, &dinfo, &tcal, &mcal);
    ASSERT_TRUE(result);

    EXPECT_EQ(info.devSerialNum, 77001u);
    EXPECT_EQ(tcal.gyr[0].numPts, 2u);
}

TEST_F(test_ISDeviceCal, loadFromJsonString_invalidJson)
{
    std::string badJson = "not json at all {{{";
    bool result = ISDeviceCal::loadCalibrationFromJsonString(badJson, NULL, &info);
    EXPECT_FALSE(result);
}

TEST_F(test_ISDeviceCal, loadFromJsonString_emptyString)
{
    // Empty string should fail to parse
    bool result = ISDeviceCal::loadCalibrationFromJsonString("", NULL, &info);
    EXPECT_FALSE(result);
}

// ---- Consistency: all three methods produce the same result ----

TEST_F(test_ISDeviceCal, allMethods_produceConsistentResults)
{
    json jObj = makeTestCalJson(55001);
    std::string path = writeTempCalFile(jObj, "test_cal_consistency.json");
    std::string jsonStr = jObj.dump();

    // Load via file
    sensor_cal_info_t infoFile = {};
    sensor_tcal_group_t tcalFile = {};
    ASSERT_TRUE(ISDeviceCal::loadCalibrationFromJsonFile(path, NULL, &infoFile, NULL, &tcalFile));

    // Load via json object
    sensor_cal_info_t infoObj = {};
    sensor_tcal_group_t tcalObj = {};
    ASSERT_TRUE(ISDeviceCal::loadCalibrationFromJsonObj(jObj, NULL, &infoObj, NULL, &tcalObj));

    // Load via string
    sensor_cal_info_t infoStr = {};
    sensor_tcal_group_t tcalStr = {};
    ASSERT_TRUE(ISDeviceCal::loadCalibrationFromJsonString(jsonStr, NULL, &infoStr, NULL, &tcalStr));

    // All should produce the same info
    EXPECT_EQ(infoFile.devSerialNum, infoObj.devSerialNum);
    EXPECT_EQ(infoObj.devSerialNum, infoStr.devSerialNum);
    EXPECT_EQ(infoFile.version[0], infoObj.version[0]);
    EXPECT_EQ(infoObj.version[0], infoStr.version[0]);

    // All should produce the same tcal
    EXPECT_EQ(tcalFile.gyr[0].numPts, tcalObj.gyr[0].numPts);
    EXPECT_EQ(tcalObj.gyr[0].numPts, tcalStr.gyr[0].numPts);
    EXPECT_NEAR(tcalFile.gyr[0].pt[0].temp, tcalObj.gyr[0].pt[0].temp, 0.001f);
    EXPECT_NEAR(tcalObj.gyr[0].pt[0].temp, tcalStr.gyr[0].pt[0].temp, 0.001f);

    std::remove(path.c_str());
}

// ---- Round-trip: save → load → compare ----

TEST_F(test_ISDeviceCal, roundTrip_saveAndLoad)
{
    // Set up calibration data
    sensor_cal_info_t origInfo = {};
    origInfo.version[0] = 2;
    origInfo.version[1] = 1;
    origInfo.version[2] = 0;
    origInfo.version[3] = 0;
    origInfo.devSerialNum = 66001;
    origInfo.calDate[0] = 25;  // 2025
    origInfo.calDate[1] = 3;
    origInfo.calDate[2] = 13;
    origInfo.calTime[0] = 10;
    origInfo.calTime[1] = 30;
    origInfo.calTime[2] = 0;
    origInfo.size = sizeof(sensor_cal_info_t);

    sensor_mcal_group_t origMcal = {};
    // Set identity matrices for gyro
    for (int d = 0; d < MAX_IMU_DEVICES; d++)
    {
        origMcal.pqr[d].orth[0] = 1.01f; origMcal.pqr[d].orth[4] = 1.0f; origMcal.pqr[d].orth[8] = 1.0f;
        origMcal.pqr[d].bias[0] = 0.001f; origMcal.pqr[d].bias[1] = 0.002f; origMcal.pqr[d].bias[2] = 0.003f;
        origMcal.acc[d].orth[0] = 1.0f; origMcal.acc[d].orth[4] = 1.02f; origMcal.acc[d].orth[8] = 1.0f;
        origMcal.acc[d].bias[0] = 0.01f;
    }

    std::string path = "/tmp/test_cal_roundtrip.json";
    ASSERT_TRUE(ISDeviceCal::saveCalibrationToJsonObj(path, NULL, &origInfo, NULL, &origMcal));

    // Load it back
    sensor_cal_info_t loadInfo = {};
    sensor_mcal_group_t loadMcal = {};
    ASSERT_TRUE(ISDeviceCal::loadCalibrationFromJsonFile(path, NULL, &loadInfo, NULL, NULL, &loadMcal));

    EXPECT_EQ(loadInfo.devSerialNum, origInfo.devSerialNum);
    EXPECT_EQ(loadInfo.version[0], origInfo.version[0]);
    EXPECT_EQ(loadInfo.version[1], origInfo.version[1]);

    // Verify motion cal data survived round-trip
    EXPECT_NEAR(loadMcal.pqr[0].orth[0], origMcal.pqr[0].orth[0], 0.001f);
    EXPECT_NEAR(loadMcal.pqr[0].bias[0], origMcal.pqr[0].bias[0], 0.001f);
    EXPECT_NEAR(loadMcal.acc[0].orth[4], origMcal.acc[0].orth[4], 0.001f);

    std::remove(path.c_str());
}


// =====================================================================================
// SN-7966: v1.3 <-> v1.4 conversion + version-aware upload tests
// =====================================================================================

namespace {

// Populate a v1p4 cal with distinguishable per-device values, then compute info checksum.
static void buildV1p4Sample(sensor_cal_v1p4_t &cal, uint32_t serialNum = 60010)
{
    memset(&cal, 0, sizeof(cal));
    cal.info.size = sizeof(sensor_cal_info_t);
    cal.info.version[0] = 1;
    cal.info.version[1] = 4;
    cal.info.devSerialNum = serialNum;
    cal.info.checksum = flashChecksum32(&cal.info, cal.info.size);

    for (int d = 0; d < NUM_IMU_DEVICES_V1P4; d++)
    {
        cal.data.mcal.pqr[d].orth[0] = 1.0f + 0.01f * d;
        cal.data.mcal.pqr[d].bias[0] = 0.001f * (d + 1);
        cal.data.mcal.acc[d].orth[4] = 1.0f + 0.02f * d;
    }
    for (int d = 0; d < NUM_MAG_DEVICES_V1P4; d++)
    {
        cal.data.mcal.mag[d].orth[0] = 2.5f;
        cal.data.mcal.mag[d].bias[1] = 0.5f;
    }
    cal.data.dinfo.size = sizeof(sensor_cal_v1p4_data_t);
    cal.data.dinfo.checksum = flashChecksum32(&cal.data, cal.data.dinfo.size);
}

static void buildV1p3Sample(sensor_cal_v1p3_t &cal, uint32_t serialNum = 60010)
{
    memset(&cal, 0, sizeof(cal));
    cal.info.size = sizeof(sensor_cal_info_t);
    cal.info.version[0] = 1;
    cal.info.version[1] = 3;
    cal.info.devSerialNum = serialNum;
    cal.info.checksum = flashChecksum32(&cal.info, cal.info.size);

    for (int d = 0; d < NUM_IMU_DEVICES_V1P3; d++)
    {
        cal.data.mcal.pqr[d].orth[0] = 1.0f + 0.01f * d;
        cal.data.mcal.pqr[d].bias[0] = 0.001f * (d + 1);
    }
    for (int d = 0; d < NUM_MAG_DEVICES_V1P3; d++)
    {
        cal.data.mcal.mag[d].orth[0] = 2.5f + 0.1f * d;
    }
    cal.data.dinfo.size = sizeof(sensor_cal_v1p3_data_t);
    cal.data.dinfo.checksum = flashChecksum32(&cal.data, cal.data.dinfo.size);
}

} // namespace

TEST(ISDeviceCalConvert, V1p4ToV1p3PreservesSharedFields)
{
    sensor_cal_v1p4_t v1p4;
    buildV1p4Sample(v1p4, 60010);

    sensor_cal_v1p3_t v1p3;
    convert_sensor_cal_v1p4_to_v1p3(&v1p4, &v1p3);

    EXPECT_EQ(v1p3.info.version[0], 1);
    EXPECT_EQ(v1p3.info.version[1], 3);
    EXPECT_EQ(v1p3.info.devSerialNum, 60010u);

    // First NUM_IMU_DEVICES_V1P3 IMUs survive the downgrade
    for (int d = 0; d < NUM_IMU_DEVICES_V1P3; d++)
    {
        EXPECT_FLOAT_EQ(v1p3.data.mcal.pqr[d].orth[0], v1p4.data.mcal.pqr[d].orth[0]);
        EXPECT_FLOAT_EQ(v1p3.data.mcal.pqr[d].bias[0], v1p4.data.mcal.pqr[d].bias[0]);
    }
    // First mag survives
    EXPECT_FLOAT_EQ(v1p3.data.mcal.mag[0].orth[0], v1p4.data.mcal.mag[0].orth[0]);
    // v1p3.mag[1] (no v1.4 source) is set to identity defaults by set_sensor_cal_data_defaults_v1p3
    EXPECT_FLOAT_EQ(v1p3.data.mcal.mag[1].orth[0], 1.0f);
    EXPECT_FLOAT_EQ(v1p3.data.mcal.mag[1].orth[4], 1.0f);
    EXPECT_FLOAT_EQ(v1p3.data.mcal.mag[1].orth[8], 1.0f);
}

TEST(ISDeviceCalConvert, V1p3ToV1p4PreservesSharedFields)
{
    sensor_cal_v1p3_t v1p3;
    buildV1p3Sample(v1p3, 60011);

    sensor_cal_v1p4_t v1p4;
    convert_sensor_cal_v1p3_to_v1p4(&v1p3, &v1p4);

    EXPECT_EQ(v1p4.info.version[0], 1);
    EXPECT_EQ(v1p4.info.version[1], 4);
    EXPECT_EQ(v1p4.info.devSerialNum, 60011u);

    for (int d = 0; d < NUM_IMU_DEVICES_V1P3; d++)
    {
        EXPECT_FLOAT_EQ(v1p4.data.mcal.pqr[d].orth[0], v1p3.data.mcal.pqr[d].orth[0]);
        EXPECT_FLOAT_EQ(v1p4.data.mcal.pqr[d].bias[0], v1p3.data.mcal.pqr[d].bias[0]);
    }
    EXPECT_FLOAT_EQ(v1p4.data.mcal.mag[0].orth[0], v1p3.data.mcal.mag[0].orth[0]);
    // v1p4 IMU slots beyond v1p3 are identity
    for (int d = NUM_IMU_DEVICES_V1P3; d < NUM_IMU_DEVICES_V1P4; d++)
    {
        EXPECT_FLOAT_EQ(v1p4.data.mcal.pqr[d].orth[0], 1.0f);
        EXPECT_FLOAT_EQ(v1p4.data.mcal.pqr[d].orth[4], 1.0f);
        EXPECT_FLOAT_EQ(v1p4.data.mcal.pqr[d].orth[8], 1.0f);
    }
}

TEST(ISDeviceCalConvert, V1p4ToV1p3RecomputesChecksums)
{
    sensor_cal_v1p4_t v1p4;
    buildV1p4Sample(v1p4);

    sensor_cal_v1p3_t v1p3;
    convert_sensor_cal_v1p4_to_v1p3(&v1p4, &v1p3);

    EXPECT_EQ(v1p3.info.checksum, flashChecksum32(&v1p3.info, v1p3.info.size));
    EXPECT_EQ(v1p3.data.dinfo.checksum, flashChecksum32(&v1p3.data, v1p3.data.dinfo.size));
    EXPECT_EQ(v1p3.info.size, sizeof(sensor_cal_info_t));
    EXPECT_EQ(v1p3.data.dinfo.size, sizeof(sensor_cal_v1p3_data_t));
}

TEST(ISDeviceCalConvert, RoundTripV1p4ToV1p3ToV1p4PreservesSharedSlots)
{
    sensor_cal_v1p4_t orig;
    buildV1p4Sample(orig, 60012);

    sensor_cal_v1p3_t intermediate;
    convert_sensor_cal_v1p4_to_v1p3(&orig, &intermediate);

    sensor_cal_v1p4_t out;
    convert_sensor_cal_v1p3_to_v1p4(&intermediate, &out);

    EXPECT_EQ(out.info.devSerialNum, orig.info.devSerialNum);
    // Slots that fit in v1.3 (first 3 IMUs, first mag) round-trip exactly
    for (int d = 0; d < NUM_IMU_DEVICES_V1P3; d++)
    {
        EXPECT_FLOAT_EQ(out.data.mcal.pqr[d].orth[0], orig.data.mcal.pqr[d].orth[0]);
        EXPECT_FLOAT_EQ(out.data.mcal.pqr[d].bias[0], orig.data.mcal.pqr[d].bias[0]);
    }
    EXPECT_FLOAT_EQ(out.data.mcal.mag[0].orth[0], orig.data.mcal.mag[0].orth[0]);
}

TEST(ISDeviceCalUpload, RefusesUnknownHardwareVersion)
{
    sensor_cal_v1p4_t cal;
    buildV1p4Sample(cal);

    dev_info_t devInfo = {};
    devInfo.hardwareVer[0] = 0; // unresolved (e.g. bootloader / unconnected)

    int state = 0;
    ISDeviceCal::cal_upload_ctx_t ctx;
    int result = ISDeviceCal::uploadSensorCalStep(/*port=*/nullptr, state, cal, devInfo, ctx);
    EXPECT_EQ(result, -1) << "Upload must refuse when hardware version is unresolved";
}

TEST(ISDeviceCalUpload, AcceptsImx5AndImx6HardwareVersions)
{
    sensor_cal_v1p4_t cal;
    buildV1p4Sample(cal);

    // Note: full upload would call comManagerSendData on a port, which will fail
    // without a registered port and return non-zero from comManagerSendData. That
    // surfaces as state machine returning 0 (still "in progress"), not -1. So we
    // assert "not the refusal code" rather than "success".
    dev_info_t imx5 = {};
    imx5.hardwareVer[0] = 5;
    int state5 = 0;
    ISDeviceCal::cal_upload_ctx_t ctx5;
    EXPECT_NE(-1, ISDeviceCal::uploadSensorCalStep(nullptr, state5, cal, imx5, ctx5));

    dev_info_t imx6 = {};
    imx6.hardwareVer[0] = 6;
    int state6 = 0;
    ISDeviceCal::cal_upload_ctx_t ctx6;
    EXPECT_NE(-1, ISDeviceCal::uploadSensorCalStep(nullptr, state6, cal, imx6, ctx6));
}
