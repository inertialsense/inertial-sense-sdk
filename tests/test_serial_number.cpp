/**
 * SN-8985: serial numbers are uint32 end to end. Values >= 2^31 must not render negative, and an unprogrammed serial
 * (0xFFFFFFFF, or 0) renders as the explicit "SN-unset" marker.
 */
#include <gtest/gtest.h>
#include <cstdio>
#include <string>

#include "ISDataMappings.h"
#include "ISDevice.h"
#include "data_sets.h"
#include "protocol_nmea.h"
#include "util/util.h"

namespace {
const uint32_t kSerials[] = { 0u, 63736u, 910564648u, 2147483648u, 0xFFFFFFFEu, 0xFFFFFFFFu };
}

TEST(SerialNumber, is_set)
{
    EXPECT_FALSE(serialNumberIsSet(0));
    EXPECT_TRUE(serialNumberIsSet(63736u));
    EXPECT_TRUE(serialNumberIsSet(2147483648u));
    EXPECT_TRUE(serialNumberIsSet(0xFFFFFFFEu));
    EXPECT_FALSE(serialNumberIsSet(SERIAL_NUMBER_UNSET));
}

TEST(SerialNumber, to_string)
{
    EXPECT_EQ(utils::serialNumberToString(0), "SN-unset");
    EXPECT_EQ(utils::serialNumberToString(63736u), "SN63736");
    EXPECT_EQ(utils::serialNumberToString(910564648u), "SN910564648");
    EXPECT_EQ(utils::serialNumberToString(2147483648u), "SN2147483648");
    EXPECT_EQ(utils::serialNumberToString(0xFFFFFFFEu), "SN4294967294");
    EXPECT_EQ(utils::serialNumberToString(0xFFFFFFFFu), "SN-unset");
    EXPECT_EQ(utils::serialNumberToString(63736u, 9), "SN000063736");
    EXPECT_EQ(utils::serialNumberToString(2147483648u, 9), "SN2147483648");
    EXPECT_EQ(utils::serialNumberToString(SERIAL_NUMBER_UNSET, 9), "SN-unset");
}

TEST(SerialNumber, parse)
{
    for (uint32_t sn : kSerials) {
        uint32_t out = 12345u;
        EXPECT_TRUE(utils::parseSerialNumber(std::to_string(sn), out)) << sn;
        EXPECT_EQ(out, sn);
    }
    for (const char *bad : { "", "-1", "+5", " 5", "5 ", "12a", "4294967296", "99999999999", "0x10" }) {
        uint32_t out = 777u;
        EXPECT_FALSE(utils::parseSerialNumber(bad, out)) << bad;
        EXPECT_EQ(out, 777u) << "unchanged on failure: " << bad;
    }
}

TEST(SerialNumber, device_name_never_negative)
{
    for (uint32_t sn : kSerials) {
        dev_info_t di = {};
        di.serialNumber = sn;
        di.hardwareType = IS_HARDWARE_TYPE_IMX;
        std::string name = ISDevice::getName(di, 0);
        EXPECT_NE(name.find(utils::serialNumberToString(sn, 9)), std::string::npos) << name;
        if (serialNumberIsSet(sn)) EXPECT_EQ(name.find("SN-"), std::string::npos) << name;
    }
}

TEST(SerialNumber, nmea_info_round_trip)
{
    for (uint32_t sn : kSerials) {
        dev_info_t in = {}, out = {};
        in.serialNumber = sn;
        char buf[512];
        int n = nmea_dev_info(buf, sizeof(buf), in);
        ASSERT_GT(n, 0);
        EXPECT_EQ(nmea_parse_info(out, buf, n), 0) << buf;
        EXPECT_EQ(out.serialNumber, sn) << buf;
    }
}

TEST(SerialNumber, data_grid_rendering)
{
    const map_name_to_info_t& infoMap = *cISDataMappings::NameToInfoMap(DID_DEV_INFO);
    const data_info_t& info = infoMap.at("serialNumber");
    for (uint32_t sn : kSerials) {
        std::string expected = serialNumberIsSet(sn) ? std::to_string(sn) : "SN-unset";
        EXPECT_EQ(info.renderExtended(info, std::any(sn), 0, 0), expected);
        // CSV/log output stays numeric
        EXPECT_EQ(info.renderBasic(info, std::any(sn), 0, 0), std::to_string(sn));
    }
}
