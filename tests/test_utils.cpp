/**
 * @file test_utils.cpp
 * @brief a collection of functions and classes that might be useful when writing/running unit tests
 *
 * Also includes a few unit tests for testing utils::*
 *
 * @author Kyle Mallory on 1/18/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "test_utils.h"
#include "util.h"

TEST(test_utils, Format_devInfo) {
    dev_info_t devInfo = { };

    std::string origStr = "SN102934: IMX-5.0 fw2.1.7-devel.128 E753C.83 deadbeaf 2024-09-18 15:35:43 (p12 cmp)";      // with a comma after the hardware info
    uint16_t dvBits = utils::devInfoFromString(origStr, devInfo);

    // Test the output of various alternative formatting
    std::string devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_SERIALNO);
    EXPECT_EQ(devInfoStr, "SN102934:");

    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_HARDWARE_INFO);
    EXPECT_EQ(devInfoStr, "IMX-5.0");

    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_FIRMWARE_VER);
    EXPECT_EQ(devInfoStr, "fw2.1.7-devel.128");

    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_BUILD_KEY);
    EXPECT_EQ(devInfoStr, "e753c.83");

    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_BUILD_COMMIT);
    EXPECT_EQ(devInfoStr, "deadbeaf");

    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_BUILD_DATE);
    EXPECT_EQ(devInfoStr, "2024-09-18");

    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_BUILD_TIME);
    EXPECT_EQ(devInfoStr, "15:35:43");

    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_ADDITIONAL_INFO);
    EXPECT_EQ(devInfoStr, "(p12 cmp)");


    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_SERIALNO | utils::DV_BIT_HARDWARE_INFO | utils::DV_BIT_FIRMWARE_VER);
    EXPECT_EQ(devInfoStr, "SN102934: IMX-5.0 fw2.1.7-devel.128");

    devInfoStr = utils::devInfoToString(devInfo, utils::DV_BIT_BUILD_DATE | utils::DV_BIT_BUILD_TIME);
    EXPECT_EQ(devInfoStr, "2024-09-18 15:35:43");

    devInfoStr = utils::devInfoToString(devInfo);
    EXPECT_EQ(devInfoStr, "SN102934: IMX-5.0 fw2.1.7-devel.128 deadbeaf e753c.83 20240918 153543 (p12 cmp)");
}


TEST(test_utils, FormatAndParse_devInfo) {
    dev_info_t devInfo = { };

    std::string origStr;
    uint16_t dvBits = 0;

    // test legacy formats
    origStr = "SN102934: IMX-5.0, fw2.1.7 b83c 2024-09-18 15:35:43 (p12 cmp)";      // with a comma after the hardware info
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 319) << "Unable to parse '" << origStr << "'" << std::endl;
    std::string devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "SN102934: IMX-5.0 fw2.1.7-rc b83 2024-09-18 15:35:43 (p12 cmp)");

    origStr = "SN032014: uINS-3.2 fw1.0.9 b12 2022-11-03 00:31:18 (p0)";      // with-out the comma after the hardware info
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 319) << "Unable to parse '" << origStr << "'" << std::endl;
    devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "SN032014: uINS-3.2 fw1.0.9 b12 2022-11-03 00:31:18 (p0)");

    origStr = "SN23914: GPX-1.0 fw0.1.12 b0 2023-11-03 9:09:13 ()";
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 319) << "Unable to parse '" << origStr << "'" << std::endl;
    devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "SN023914: GPX-1.0 fw0.1.12 2023-11-03 09:09:13");

    origStr = "SN505192: IMX-5.1.0 fw0.1.12.8 b123 2025-07-31 13:38:22";
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 63) << "Unable to parse '" << origStr << "'" << std::endl;
    devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "SN505192: IMX-5.1 fw0.1.12.8 b123 2025-07-31 13:38:22");

    origStr = "SN98712178: GPX-1.0.4 fw2.18.5.3 b123c 2025-08-22 18:56:03";
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 63) << "Unable to parse '" << origStr << "'" << std::endl;
    devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "SN98712178: GPX-1.0.4 fw2.18.5-rc.3 b123 2025-08-22 18:56:03");


    // newer 2.1.0 format
    origStr = "SN102934: IMX-5.0 fw2.1.7-rc.83 9d849cba 1739C.5 2024-09-18 15:35:43 (p12 cmp)";
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 383) << "Unable to parse '" << origStr << "'" << std::endl;
    devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "SN102934: IMX-5.0 fw2.1.7-rc.83 9d849cba 1739c.5 2024-09-18 15:35:43 (p12 cmp)");

    origStr = "SN102934: IMX-5.0 fw2.1.7 b83 2024-09-18 15:35:43 (p12 cmp)";
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 319) << "Unable to parse '" << origStr << "'" << std::endl;
    devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "SN102934: IMX-5.0 fw2.1.7 b83 2024-09-18 15:35:43 (p12 cmp)");
}

TEST(test_utils, parse_devInfo_from_filename) {
    dev_info_t devInfo = { };

    std::string origStr;
    uint16_t dvBits = 0;

    // test legacy formats
    origStr = "IS_IMX-5_v2.0.6_b80_2024-05-30_145347.hex";
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 62) << "Unable to parse '" << origStr << "'" << std::endl;
    std::string devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "IMX-5.0 fw2.0.6 b80 2024-05-30 14:53:47");


    origStr = "IS_GPX-1_v2.3.0-snap.153+2024-12-17_002837.encrypted.bin";
    EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 54) << "Unable to parse '" << origStr << "'" << std::endl;
    devInfoStr = utils::devInfoToString(devInfo, dvBits);
    EXPECT_EQ(devInfoStr, "GPX-1.0 fw2.3.0-snap.153 2024-12-17 00:28:37");

    // origStr = "cxd5610_v0.134_lib.efpk"  // this is be ignored
    // EXPECT_EQ(dvBits = utils::devInfoFromString(origStr, devInfo), 54) << "Unable to parse '" << origStr << "'" << std::endl;
    // devInfoStr = utils::devInfoToString(devInfo, dvBits);
    // EXPECT_EQ(devInfoStr, "GPX-1.0 fw2.3.0-snap.153 2024-12-17 00:28:37");

}

TEST(test_utils, parseHardwareFromString_roundtrip) {
    dev_info_t devInfo = {};

    ASSERT_TRUE(utils::parseHardwareFromString("IMX-5.0", devInfo));
    EXPECT_EQ(devInfo.hardwareType, IS_HARDWARE_TYPE_IMX);
    EXPECT_EQ(devInfo.hardwareVer[0], 5);
    EXPECT_EQ(devInfo.hardwareVer[1], 0);
    EXPECT_EQ(devInfo.hardwareVer[2], 0);
    EXPECT_EQ(devInfo.hardwareVer[3], 0);

    ASSERT_TRUE(utils::parseHardwareFromString("GPX-1.0.2", devInfo));
    EXPECT_EQ(devInfo.hardwareType, IS_HARDWARE_TYPE_GPX);
    EXPECT_EQ(devInfo.hardwareVer[0], 1);
    EXPECT_EQ(devInfo.hardwareVer[1], 0);
    EXPECT_EQ(devInfo.hardwareVer[2], 2);

    ASSERT_TRUE(utils::parseHardwareFromString("uINS-3.2.1.4", devInfo));
    EXPECT_EQ(devInfo.hardwareType, IS_HARDWARE_TYPE_UINS);
    EXPECT_EQ(devInfo.hardwareVer[0], 3);
    EXPECT_EQ(devInfo.hardwareVer[1], 2);
    EXPECT_EQ(devInfo.hardwareVer[2], 1);
    EXPECT_EQ(devInfo.hardwareVer[3], 4);

    // Round-trip: emit then parse.
    devInfo = {};
    devInfo.hardwareType = IS_HARDWARE_TYPE_IMX;
    devInfo.hardwareVer[0] = 5; devInfo.hardwareVer[1] = 1;
    std::string s = utils::getHardwareAsString(devInfo);
    dev_info_t parsed = {};
    ASSERT_TRUE(utils::parseHardwareFromString(s, parsed));
    EXPECT_EQ(parsed.hardwareType, devInfo.hardwareType);
    EXPECT_EQ(parsed.hardwareVer[0], devInfo.hardwareVer[0]);
    EXPECT_EQ(parsed.hardwareVer[1], devInfo.hardwareVer[1]);

    // Malformed input is rejected without mutating devInfo.
    dev_info_t before = parsed;
    EXPECT_FALSE(utils::parseHardwareFromString("", parsed));
    EXPECT_FALSE(utils::parseHardwareFromString("IMX", parsed));
    EXPECT_FALSE(utils::parseHardwareFromString("BOGUS-1.0", parsed));
    EXPECT_EQ(parsed.hardwareType, before.hardwareType);
    EXPECT_EQ(parsed.hardwareVer[0], before.hardwareVer[0]);
}

TEST(test_utils, parseFirmwareFromString_roundtrip) {
    dev_info_t devInfo = {};

    ASSERT_TRUE(utils::parseFirmwareFromString("fw3.0.0", devInfo));
    EXPECT_EQ(devInfo.firmwareVer[0], 3);
    EXPECT_EQ(devInfo.firmwareVer[1], 0);
    EXPECT_EQ(devInfo.firmwareVer[2], 0);
    EXPECT_EQ(devInfo.firmwareVer[3], 0);
    EXPECT_EQ(devInfo.buildType, 'r');

    ASSERT_TRUE(utils::parseFirmwareFromString("fw3.0.0-devel.175", devInfo));
    EXPECT_EQ(devInfo.firmwareVer[0], 3);
    EXPECT_EQ(devInfo.firmwareVer[3], 175);
    EXPECT_EQ(devInfo.buildType, 'd');

    ASSERT_TRUE(utils::parseFirmwareFromString("2.1.7-rc.83", devInfo)); // no 'fw' prefix
    EXPECT_EQ(devInfo.firmwareVer[0], 2);
    EXPECT_EQ(devInfo.firmwareVer[1], 1);
    EXPECT_EQ(devInfo.firmwareVer[2], 7);
    EXPECT_EQ(devInfo.firmwareVer[3], 83);
    EXPECT_EQ(devInfo.buildType, 'c');

    // Round-trip: emit then parse.
    dev_info_t src = {};
    src.firmwareVer[0] = 2; src.firmwareVer[1] = 4; src.firmwareVer[2] = 0; src.firmwareVer[3] = 12;
    src.buildType = 's';
    std::string s = utils::getFirmwareAsString(src);
    dev_info_t parsed = {};
    ASSERT_TRUE(utils::parseFirmwareFromString(s, parsed));
    EXPECT_EQ(parsed.firmwareVer[0], src.firmwareVer[0]);
    EXPECT_EQ(parsed.firmwareVer[1], src.firmwareVer[1]);
    EXPECT_EQ(parsed.firmwareVer[2], src.firmwareVer[2]);
    EXPECT_EQ(parsed.firmwareVer[3], src.firmwareVer[3]);
    EXPECT_EQ(parsed.buildType, src.buildType);

    // ISBL "firmware_ver" like "ISbl.v6j **BOOTLOADER**" is not parseable — helper returns false,
    // caller leaves fields at whatever they were.
    EXPECT_FALSE(utils::parseFirmwareFromString("ISbl.v6j **BOOTLOADER**", parsed));
}
