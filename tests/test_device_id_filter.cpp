/**
 * @file test_device_id_filter.cpp
 * @brief Device selection by hardware identity: utils::parseHardwareIdMask/parseDeviceIdMask/
 *        devInfoMatchesIdMask, and the ISDevice methods that build on them.
 *
 * Hardware type and serial number are two masks over ONE identity (the packed unique id), not separate
 * concepts, so a selection may name either or both and whatever it omits is a wildcard. These tests pin
 * that, and pin the wildcard semantics per field -- which is where the subtle failure lives: a flat
 * bitwise-subset test over the packed value lets a concrete field value match a different concrete value,
 * so an IMX-5.0 would satisfy a mask for IMX-5.2 because minor 0 is a bit-subset of every value.
 *
 * No hardware, no ports, no platform-specific scaffolding -- identity matching is pure computation.
 */

#include <gtest/gtest.h>

#include "ISDevice.h"
#include "util/util.h"

namespace {

dev_info_t devInfoFor(uint8_t type, uint8_t major, uint8_t minor, uint32_t serialNo) {
    dev_info_t devInfo = {};
    devInfo.hardwareType = type;
    devInfo.hardwareVer[0] = major;
    devInfo.hardwareVer[1] = minor;
    devInfo.serialNumber = serialNo;
    return devInfo;
}

const dev_info_t IMX_5_0 = devInfoFor(IS_HARDWARE_TYPE_IMX, 5, 0, 62913);
const dev_info_t IMX_5_2 = devInfoFor(IS_HARDWARE_TYPE_IMX, 5, 2, 11111);
const dev_info_t IMX_6_0 = devInfoFor(IS_HARDWARE_TYPE_IMX, 6, 0, 64139);
const dev_info_t GPX_1_0 = devInfoFor(IS_HARDWARE_TYPE_GPX, 1, 0, 82531);

//! Parses a selection and asserts it parsed, returning the mask.
uint64_t maskFor(const std::string& spec) {
    uint64_t idMask = 0;
    EXPECT_TRUE(utils::parseDeviceIdMask(spec, idMask)) << "failed to parse device spec '" << spec << "'";
    return idMask;
}

} // namespace


// ---------------------------------------------------------------- parseHardwareIdMask

TEST(device_id_filter, hardware_family_matches_every_version) {
    uint16_t hdwId = 0;
    ASSERT_TRUE(utils::parseHardwareIdMask("IMX", hdwId));
    EXPECT_TRUE(DEV_INFO_MATCHES_HDW_ID(IMX_5_0, hdwId));
    EXPECT_TRUE(DEV_INFO_MATCHES_HDW_ID(IMX_5_2, hdwId));
    EXPECT_TRUE(DEV_INFO_MATCHES_HDW_ID(IMX_6_0, hdwId));
    EXPECT_FALSE(DEV_INFO_MATCHES_HDW_ID(GPX_1_0, hdwId)) << "an IMX family mask must not match a GPX";
}

TEST(device_id_filter, hardware_major_only_wildcards_the_minor) {
    uint16_t hdwId = 0;
    ASSERT_TRUE(utils::parseHardwareIdMask("IMX-5", hdwId));
    EXPECT_TRUE(DEV_INFO_MATCHES_HDW_ID(IMX_5_0, hdwId));
    EXPECT_TRUE(DEV_INFO_MATCHES_HDW_ID(IMX_5_2, hdwId));
    EXPECT_FALSE(DEV_INFO_MATCHES_HDW_ID(IMX_6_0, hdwId));
}

TEST(device_id_filter, hardware_full_version_is_exact) {
    uint16_t hdwId = 0;
    ASSERT_TRUE(utils::parseHardwareIdMask("IMX-5.0", hdwId));
    EXPECT_TRUE(DEV_INFO_MATCHES_HDW_ID(IMX_5_0, hdwId));
    EXPECT_FALSE(DEV_INFO_MATCHES_HDW_ID(IMX_6_0, hdwId));

    // The regression this guards: minor 0 is a bit-subset of every value, so a flat bitwise-subset test
    // would report a match here.
    uint16_t imx5_2 = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 2);
    EXPECT_FALSE(DEV_INFO_MATCHES_HDW_ID(IMX_5_0, imx5_2))
        << "IMX-5.0 must not satisfy a mask for IMX-5.2";
    EXPECT_FALSE(DEV_INFO_MATCHES_HDW_ID(IMX_5_2, hdwId))
        << "IMX-5.2 must not satisfy a mask for IMX-5.0";
}

TEST(device_id_filter, hardware_names_are_case_insensitive) {
    uint16_t lower = 0, upper = 0;
    ASSERT_TRUE(utils::parseHardwareIdMask("imx-5.0", lower));
    ASSERT_TRUE(utils::parseHardwareIdMask("IMX-5.0", upper));
    EXPECT_EQ(lower, upper);
}

TEST(device_id_filter, hardware_rejects_malformed_specs) {
    uint16_t hdwId = 0;
    EXPECT_FALSE(utils::parseHardwareIdMask("", hdwId));
    EXPECT_FALSE(utils::parseHardwareIdMask("NOTAPRODUCT-1.0", hdwId)) << "unknown type name";
    EXPECT_FALSE(utils::parseHardwareIdMask("IMX-", hdwId))            << "dash with no version";
    EXPECT_FALSE(utils::parseHardwareIdMask("IMX-x", hdwId))           << "non-numeric major";
    EXPECT_FALSE(utils::parseHardwareIdMask("-5.0", hdwId))            << "no type name";
}


// ---------------------------------------------------------------- parseDeviceIdMask

TEST(device_id_filter, serial_only_leaves_hardware_wild) {
    for (const std::string& spec : { std::string("SN62913"), std::string("62913") }) {
        uint64_t idMask = maskFor(spec);
        EXPECT_EQ(DECODE_UNIQUE_ID_TO_SERIALNO(idMask), 62913u) << spec;
        EXPECT_EQ(DECODE_UNIQUE_ID_TO_HDW_ID(idMask), IS_HARDWARE_ANY) << spec;
        // Any hardware type, so long as the serial matches.
        EXPECT_TRUE(utils::devInfoMatchesIdMask(IMX_5_0, idMask)) << spec;
        EXPECT_FALSE(utils::devInfoMatchesIdMask(IMX_6_0, idMask)) << spec << " (different serial)";
    }
}

TEST(device_id_filter, type_only_leaves_serial_wild) {
    uint64_t idMask = maskFor("IMX-5.0");
    EXPECT_EQ(DECODE_UNIQUE_ID_TO_SERIALNO(idMask), 0u) << "no serial named means any serial";
    EXPECT_TRUE(utils::devInfoMatchesIdMask(IMX_5_0, idMask));

    // Same hardware, different serial -- still permitted.
    dev_info_t otherImx5 = devInfoFor(IS_HARDWARE_TYPE_IMX, 5, 0, 99999);
    EXPECT_TRUE(utils::devInfoMatchesIdMask(otherImx5, idMask));
    EXPECT_FALSE(utils::devInfoMatchesIdMask(IMX_6_0, idMask));
}

TEST(device_id_filter, family_only_spec_is_accepted) {
    uint64_t idMask = maskFor("IMX");
    EXPECT_TRUE(utils::devInfoMatchesIdMask(IMX_5_0, idMask));
    EXPECT_TRUE(utils::devInfoMatchesIdMask(IMX_6_0, idMask));
    EXPECT_FALSE(utils::devInfoMatchesIdMask(GPX_1_0, idMask));
}

TEST(device_id_filter, type_and_serial_together_require_both) {
    // Both separators are accepted.
    for (const std::string& spec : { std::string("IMX-5.0::SN62913"), std::string("IMX-5.0:62913") }) {
        uint64_t idMask = maskFor(spec);
        EXPECT_TRUE(utils::devInfoMatchesIdMask(IMX_5_0, idMask)) << spec;

        dev_info_t rightSerialWrongType = devInfoFor(IS_HARDWARE_TYPE_GPX, 1, 0, 62913);
        dev_info_t rightTypeWrongSerial = devInfoFor(IS_HARDWARE_TYPE_IMX, 5, 0, 11111);
        EXPECT_FALSE(utils::devInfoMatchesIdMask(rightSerialWrongType, idMask)) << spec;
        EXPECT_FALSE(utils::devInfoMatchesIdMask(rightTypeWrongSerial, idMask)) << spec;
    }
}

TEST(device_id_filter, device_spec_rejects_garbage) {
    uint64_t idMask = 0;
    EXPECT_FALSE(utils::parseDeviceIdMask("", idMask));
    EXPECT_FALSE(utils::parseDeviceIdMask("NOTAPRODUCT", idMask));
    EXPECT_FALSE(utils::parseDeviceIdMask("SNxyz", idMask))    << "SN prefix without digits";
    EXPECT_FALSE(utils::parseDeviceIdMask("IMX-5.0::SNxyz", idMask));
    EXPECT_FALSE(utils::parseDeviceIdMask("BADTYPE-1.0::SN1", idMask));
}

TEST(device_id_filter, a_fully_wild_mask_matches_everything) {
    // Not reachable from a spec string (something must be named), but a caller may construct it.
    uint64_t wild = ((uint64_t)IS_HARDWARE_ANY << 48);
    EXPECT_TRUE(utils::devInfoMatchesIdMask(IMX_5_0, wild));
    EXPECT_TRUE(utils::devInfoMatchesIdMask(GPX_1_0, wild));
}


// ---------------------------------------------------------------- ISDevice

TEST(device_id_filter, isdevice_matchesIdMask_delegates_to_utils) {
    ISDevice device(IMX_5_0);
    EXPECT_TRUE(device.matchesIdMask(maskFor("IMX")));
    EXPECT_TRUE(device.matchesIdMask(maskFor("IMX-5.0")));
    EXPECT_TRUE(device.matchesIdMask(maskFor("SN62913")));
    EXPECT_TRUE(device.matchesIdMask(maskFor("IMX-5.0::SN62913")));
    EXPECT_FALSE(device.matchesIdMask(maskFor("IMX-6.0")));
    EXPECT_FALSE(device.matchesIdMask(maskFor("SN64139")));
    EXPECT_FALSE(device.matchesIdMask(maskFor("GPX")));
}

TEST(device_id_filter, isdevice_matchesIdSpec_parses_then_matches) {
    ISDevice device(IMX_5_0);
    EXPECT_TRUE(device.matchesIdSpec("IMX-5.0::SN62913"));
    EXPECT_FALSE(device.matchesIdSpec("IMX-6.0"));
    EXPECT_FALSE(device.matchesIdSpec("NOTAPRODUCT")) << "an unparseable spec must not match";
}

TEST(device_id_filter, isdevice_matchesHdwId_is_per_field) {
    ISDevice imx5(IMX_5_0);
    EXPECT_TRUE(imx5.matchesHdwId(IS_HARDWARE_IMX))     << "family mask";
    EXPECT_TRUE(imx5.matchesHdwId(IS_HARDWARE_IMX_5_0)) << "exact mask";
    EXPECT_TRUE(imx5.matchesHdwId(IS_HARDWARE_ANY))     << "ANY is a full wildcard";
    EXPECT_FALSE(imx5.matchesHdwId(IS_HARDWARE_IMX_6_0));
    EXPECT_FALSE(imx5.matchesHdwId(IS_HARDWARE_GPX));

    // The regression: a flat bitwise-subset test reported this as a match.
    EXPECT_FALSE(imx5.matchesHdwId(ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 2)))
        << "an IMX-5.0 must not match a mask for IMX-5.2";

    // The optional serial argument narrows further; 0 means any.
    EXPECT_TRUE(imx5.matchesHdwId(IS_HARDWARE_IMX_5_0, 0));
    EXPECT_TRUE(imx5.matchesHdwId(IS_HARDWARE_IMX_5_0, 62913));
    EXPECT_FALSE(imx5.matchesHdwId(IS_HARDWARE_IMX_5_0, 64139));
}

TEST(device_id_filter, isdevice_matchesHdwId_accepts_anything_when_device_is_unidentified) {
    // A device that has not been validated reports IS_HARDWARE_ANY and must not be filtered out on
    // identity it has not yet reported.
    ISDevice unknown(IS_HARDWARE_ANY);
    EXPECT_TRUE(unknown.matchesHdwId(IS_HARDWARE_IMX_5_0));
    EXPECT_TRUE(unknown.matchesHdwId(IS_HARDWARE_GPX));
}
