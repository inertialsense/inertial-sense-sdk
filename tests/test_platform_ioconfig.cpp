/**
 * @file test_platform_ioconfig.cpp
 * @brief Platform and IMX io-configuration rendering: utils::platformTypeName/platformDescription/
 *        ioConfigDescription/imxConfigDescription.
 *
 * The io renderer shows only what DEVIATES from the configuration its platform implies, so most of
 * these cases assert an empty string -- a board configured as its carrier intends has nothing to say,
 * and anything rendered is a deliberate difference.
 *
 * That comparison rests on one detail worth pinning: imxPlatformConfigToFlashCfgIoConfig() is an
 * overlay that sets only the platform-derived GNSS fields, so the baseline must be seeded with
 * IO_CONFIG_DEFAULT first. Without the seed the pin-function fields of every stock board read as
 * deviations, which the StockBoardsRenderNothing cases below would catch.
 *
 * The five StockBoards/DeviatingBoards vectors are captured from real hardware (an IMX-5+GPX-1, an
 * IMX-6+u-blox-X20 on a RUG-4, an IMX-6+Septentrio on a BRK, an IG-1.1 and an IG-2), which is why
 * they cover Ser0/Ser2 sourcing, mixed sourcing, a non-zero ioConfig2 and CAN.
 *
 * No hardware, no ports -- rendering is pure computation.
 */

#include <gtest/gtest.h>

#include "data_sets.h"
#include "ISDevice.h"
#include "util/util.h"

namespace {

/** Builds a synchronized flash config carrying just the fields these renderers read. */
static nvm_flash_cfg_t makeCfg(uint32_t platformConfig, uint32_t ioConfig, uint8_t ioConfig2) {
    nvm_flash_cfg_t cfg = {};
    cfg.checksum = 0;                  // anything but 0xFFFFFFFF: synchronized
    cfg.platformConfig = platformConfig;
    cfg.ioConfig = ioConfig;
    cfg.ioConfig2 = ioConfig2;
    return cfg;
}

TEST(platform_ioconfig, PlatformTypeNames) {
    EXPECT_STREQ("none",            utils::platformTypeName(PLATFORM_CFG_TYPE_NONE));
    EXPECT_STREQ("BRK-GPX",         utils::platformTypeName(PLATFORM_CFG_TYPE_BRK_GPX));
    EXPECT_STREQ("BRK-X20",         utils::platformTypeName(PLATFORM_CFG_TYPE_BRK_2_X20));
    EXPECT_STREQ("BRK-SG5",         utils::platformTypeName(PLATFORM_CFG_TYPE_BRK_2_SG5));
    EXPECT_STREQ("RUG4-X20",        utils::platformTypeName(PLATFORM_CFG_TYPE_RUG4_X20));
    EXPECT_STREQ("RUG4-GPX",        utils::platformTypeName(PLATFORM_CFG_TYPE_RUG4_GPX));
    EXPECT_STREQ("RUG3-G2",         utils::platformTypeName(PLATFORM_CFG_TYPE_RUG3_G2));
    EXPECT_STREQ("IG1.0-G2",        utils::platformTypeName(PLATFORM_CFG_TYPE_IG1_0_G2));
    EXPECT_STREQ("IG1.1-G2",        utils::platformTypeName(PLATFORM_CFG_TYPE_IG1_G2));
    EXPECT_STREQ("IG2",             utils::platformTypeName(PLATFORM_CFG_TYPE_IG2));
    EXPECT_STREQ("IG2.1",           utils::platformTypeName(PLATFORM_CFG_TYPE_IG2_1));
    EXPECT_STREQ("TBED2-G1-LAMBDA", utils::platformTypeName(PLATFORM_CFG_TYPE_TBED2_G1_W_LAMBDA));

    // 4 is an unassigned gap in ePlatformConfig, and anything at/after COUNT is undefined.
    EXPECT_EQ(nullptr, utils::platformTypeName(4));
    EXPECT_EQ(nullptr, utils::platformTypeName(PLATFORM_CFG_TYPE_COUNT));
    EXPECT_EQ(nullptr, utils::platformTypeName(42));
}

TEST(platform_ioconfig, PlatformDescription) {
    EXPECT_EQ("",          utils::platformDescription(PLATFORM_CFG_TYPE_NONE));
    EXPECT_EQ("RUG4-X20",  utils::platformDescription(0x00760705));   // preset and ioexp are not rendered
    EXPECT_EQ("IG2",       utils::platformDescription(0x00000090));   // nor is the OTP-lock bit
    EXPECT_EQ("IG1.1-G2",  utils::platformDescription(0x0000008F));
    EXPECT_EQ("BRK-SG5",   utils::platformDescription(0x00000003));

    // An unrecognised type stays identifiable rather than being dropped.
    EXPECT_EQ("PT-42",     utils::platformDescription(42));
    EXPECT_EQ("PT-4",      utils::platformDescription(4));

    // VERBOSE adds the preset id and the OTP write-protect.
    EXPECT_EQ("RUG4-X20:7", utils::platformDescription(0x00760705, utils::CFGI_VERBOSE));
    EXPECT_EQ("IG2:0 (OTP)", utils::platformDescription(0x00000090, utils::CFGI_VERBOSE));
}

// Captured from real boards; each is configured as its platform intends, so each renders nothing.
TEST(platform_ioconfig, StockBoardsRenderNothing) {
    EXPECT_EQ("", utils::ioConfigDescription(0x026D2040, 0x00, 0x00760705));  // RUG4-X20, preset 7
    EXPECT_EQ("", utils::ioConfigDescription(0x06DB2046, 0x00, 0x00000090));  // IG2, OTP-locked
    EXPECT_EQ("", utils::ioConfigDescription(0x06DB2046, 0x00, 0x00000010));  // IG2, unlocked
}

TEST(platform_ioconfig, DeviatingBoards) {
    // IG-1.1 with G1,G2 turned off where the platform implies COM2.
    EXPECT_EQ("G1G2=off", utils::ioConfigDescription(0x026B2040, 0x00, 0x0000008F));

    // BRK-SG5 with CAN enabled on G1,G2. Its PPS2=G13 comes from ioConfig2 and matches the
    // platform, so the GNSS2 group stays silent.
    EXPECT_EQ("G1G2=CAN", utils::ioConfigDescription(0x091B2044, 0xC0, 0x00000003));
}

TEST(platform_ioconfig, GnssGroupsRenderWholeGroup) {
    // Move GNSS2 off the RUG-4's Ser2. Only the source changed, but the whole group renders.
    uint32_t io = 0x026D2040;
    SET_IO_CFG_GNSS2_SOURCE(io, IO_CONFIG_GNSS_SOURCE_SER0);
    EXPECT_EQ("GNSS2=uBlox@Ser0", utils::ioConfigDescription(io, 0x00, 0x00760705));

    // A type change renders the group the same way.
    io = 0x026D2040;
    SET_IO_CFG_GNSS1_TYPE(io, IO_CONFIG_GNSS_TYPE_NMEA);
    EXPECT_EQ("GNSS1=NMEA@Ser2 PPS1=G15", utils::ioConfigDescription(io, 0x00, 0x00760705));
}

TEST(platform_ioconfig, UnknownEnumValuesRenderNumerically) {
    // Neither 1 nor 2 is a defined GNSS source, and types at/after 6 are undefined. Rendering the
    // raw value keeps an unexpected configuration visible instead of silently dropping it.
    uint32_t io = 0x026D2040;
    SET_IO_CFG_GNSS1_TYPE(io, 6);
    SET_IO_CFG_GNSS1_SOURCE(io, 2);
    EXPECT_EQ("GNSS1=T6@S2 PPS1=G15", utils::ioConfigDescription(io, 0x00, 0x00760705));
}

TEST(platform_ioconfig, SpiSuppressesG6G7) {
    // SPI is a value of the G5,G8 field and overrides G6,G7, so it renders as its own token and the
    // G6,G7 one is withheld rather than printing a contradiction.
    uint32_t io = 0x06DB2046;
    io = (io & ~(uint32_t)IO_CONFIG_G5G8_MASK) | IO_CONFIG_G5G8_G6G7_SPI_ENABLE;
    std::string out = utils::ioConfigDescription(io, 0x00, 0x00000090);
    EXPECT_NE(std::string::npos, out.find("SPI"));
    EXPECT_EQ(std::string::npos, out.find("G6G7"));
}

TEST(platform_ioconfig, VerboseRendersEveryField) {
    std::string out = utils::ioConfigDescription(0x026D2040, 0x00, 0x00760705, utils::CFGI_VERBOSE);
    // Matching fields appear under VERBOSE even though they are not deviations.
    EXPECT_NE(std::string::npos, out.find("GNSS1=uBlox@Ser2 PPS1=G15"));
    EXPECT_NE(std::string::npos, out.find("GNSS2=uBlox@Ser2"));
    EXPECT_NE(std::string::npos, out.find("G6G7=COM1"));
    // Pin detail is VERBOSE-only.
    EXPECT_NE(std::string::npos, out.find("strobe="));
    EXPECT_NE(std::string::npos, out.find("G15="));
    EXPECT_NE(std::string::npos, out.find("ioexp~0x76"));

    // A board with no GNSS says so under VERBOSE, and says nothing without it.
    EXPECT_NE(std::string::npos,
              utils::ioConfigDescription(IO_CONFIG_DEFAULT, 0x00, PLATFORM_CFG_TYPE_NONE,
                                         utils::CFGI_VERBOSE).find("no GNSS"));
    EXPECT_EQ("", utils::ioConfigDescription(IO_CONFIG_DEFAULT, 0x00, PLATFORM_CFG_TYPE_NONE));
}

TEST(platform_ioconfig, ImxConfigDescriptionCombinesBothParts) {
    // Stock board: platform only, no io stanza.
    EXPECT_EQ("RUG4-X20", utils::imxConfigDescription(makeCfg(0x00760705, 0x026D2040, 0x00)));

    // Deviating board: both parts, io bracketed.
    EXPECT_EQ("BRK-SG5 [G1G2=CAN]", utils::imxConfigDescription(makeCfg(0x00000003, 0x091B2044, 0xC0)));

    // Bare module configured as a bare module: neither part has anything to say.
    EXPECT_EQ("", utils::imxConfigDescription(makeCfg(PLATFORM_CFG_TYPE_NONE, IO_CONFIG_DEFAULT, 0x00)));

    // An unsynchronised flash config has not been read from the device, so it describes nothing --
    // even though its zeroed fields would otherwise render as deviations.
    nvm_flash_cfg_t unsynced = makeCfg(0x00760705, 0x026D2040, 0x00);
    unsynced.checksum = 0xFFFFFFFF;
    EXPECT_EQ("", utils::imxConfigDescription(unsynced));
}

/** A device carrying a devInfo and a synchronized IMX flash config, with no port bound. */
static ISDevice makeDevice(uint32_t platformConfig, uint32_t ioConfig, uint8_t ioConfig2) {
    dev_info_t di = {};
    di.hardwareType = IS_HARDWARE_TYPE_IMX;
    di.hardwareVer[0] = 5;  di.hardwareVer[1] = 0;  di.hardwareVer[2] = 4;
    di.serialNumber = 60246;
    di.firmwareVer[0] = 3;  di.firmwareVer[1] = 1;  di.firmwareVer[2] = 0;

    ISDevice dev(di);
    dev.imxFlashCfg = makeCfg(platformConfig, ioConfig, ioConfig2);
    return dev;
}

TEST(platform_ioconfig, ISDeviceFlagsDefaultOff) {
    ISDevice dev = makeDevice(0x00000003, 0x091B2044, 0xC0);   // BRK-SG5 with CAN: plenty to say

    // Nothing is added unless asked for, so existing output is untouched.
    EXPECT_EQ("SN60246 (IMX-5.0.4)", dev.getName(ISDevice::COMPACT_SERIALNO));
    EXPECT_EQ(std::string::npos, dev.getDescription(ISDevice::COMPACT_SERIALNO).find('['));
    EXPECT_EQ(std::string::npos, dev.getDescription(ISDevice::COMPACT_SERIALNO).find("BRK"));
}

TEST(platform_ioconfig, ISDeviceShowPlatformSitsInsideTheParens) {
    ISDevice dev = makeDevice(0x00760705, 0x026D2040, 0x00);   // stock RUG4-X20
    EXPECT_EQ("SN60246 (IMX-5.0.4, RUG4-X20)",
              dev.getName(ISDevice::COMPACT_SERIALNO | ISDevice::SHOW_PLATFORM));

    // A stock board deviates in nothing, so asking for the io config adds no stanza.
    EXPECT_EQ(std::string::npos,
              dev.getDescription(ISDevice::COMPACT_SERIALNO | ISDevice::SHOW_IO_CONFIG).find('['));
}

TEST(platform_ioconfig, ISDeviceShowsBothWhenConfigDeviates) {
    ISDevice dev = makeDevice(0x00000003, 0x091B2044, 0xC0);   // BRK-SG5, CAN on G1,G2
    std::string out = dev.getDescription(ISDevice::COMPACT_SERIALNO | ISDevice::ESSENTIAL_FIRMWARE_INFO |
                                         ISDevice::SHOW_PLATFORM | ISDevice::SHOW_IO_CONFIG);
    EXPECT_NE(std::string::npos, out.find("(IMX-5.0.4, BRK-SG5)"));
    EXPECT_NE(std::string::npos, out.find("[G1G2=CAN]"));
}

TEST(platform_ioconfig, ISDeviceRendersNothingWhenFlashConfigUnsynchronized) {
    ISDevice dev = makeDevice(0x00000003, 0x091B2044, 0xC0);
    dev.imxFlashCfg.checksum = 0xFFFFFFFF;

    int flags = ISDevice::COMPACT_SERIALNO | ISDevice::SHOW_PLATFORM | ISDevice::SHOW_IO_CONFIG;
    EXPECT_EQ("SN60246 (IMX-5.0.4)", dev.getName(flags));
    EXPECT_EQ(std::string::npos, dev.getDescription(flags).find('['));
}

TEST(platform_ioconfig, StaticOverloadsIgnoreTheFlags) {
    dev_info_t di = {};
    di.hardwareType = IS_HARDWARE_TYPE_IMX;
    di.hardwareVer[0] = 5;
    di.serialNumber = 60246;

    // A dev_info_t carries no flash config, so neither group is renderable from the statics.
    int flags = ISDevice::COMPACT_SERIALNO | ISDevice::SHOW_PLATFORM | ISDevice::SHOW_IO_CONFIG;
    EXPECT_EQ(ISDevice::getName(di, ISDevice::COMPACT_SERIALNO), ISDevice::getName(di, flags));
    EXPECT_EQ(ISDevice::getDescription(di, ISDevice::COMPACT_SERIALNO),
              ISDevice::getDescription(di, flags));
}

}  // namespace
