/**
 * @file test_rug_preset_ioconfig.cpp
 * @brief SN-8688 -- RUG-3 (IMX-5) / RUG-4 (IMX-6) platform preset -> ioConfig mapping, host side.
 *
 * imxPlatformConfigToRug3FlashCfgIoConfig() / imxPlatformConfigToRug4FlashCfgIoConfig() turn a platformConfig
 * (carrier type + preset 0-9) into the GNSS source/type fields and the G1/G2 and G5-G8 pin-function fields of
 * ioConfig. Nothing covered that mapping, so a release shipped in which a "no GNSS" RUG-4-G0 left a GNSS UART
 * routed to a receiver that does not exist (SN-8689, a false GFC_GNSS1_INIT).
 *
 * The expectations below are written from what each preset NAME promises (e.g. "...__S2_GNSS1__S0_GNSS2" means
 * GNSS1 on Ser2 and GNSS2 on Ser0, "CAN_11_12" means G1/G2 are the CAN bus), not read back from the code under
 * test, so a table edit that drifts from the preset's meaning fails here.
 *
 * Why SOURCE and TYPE must agree: gnssInitialize() (imx_tasks.cpp) enables a receiver slot when its SOURCE is not
 * DISABLE and then calls cGNSS_common::init() with that slot's TYPE; TYPE_NONE falls to init()'s default and
 * returns false, which raises GFC_GNSSx_INIT. A live source with no type is therefore a false fault.
 *
 * Pure computation: no hardware, no ports. The I/O-expander side of the presets lives in the imx repo
 * (tests/unit_tests/test_rug_io_expander.cpp) because it needs hw-libs/boards/RUG.cpp.
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "data_sets.h"
#include "imx_defaults.h"

// Not declared in imx_defaults.h (they are the per-family halves behind imxPlatformConfigToFlashCfgIoConfig(),
// which picks one at compile time for the shared RUGn_G0 type), but they are external symbols.
extern "C" {
void imxPlatformConfigToRug3FlashCfgIoConfig(uint32_t *ioConfig, uint32_t platformConfig);
void imxPlatformConfigToRug4FlashCfgIoConfig(uint32_t *ioConfig, uint32_t platformConfig);
}

namespace {

enum Family { RUG3, RUG4 };

/** What a preset promises, independent of carrier type. */
struct PresetExpect {
    uint32_t gnss1Source;     //!< IO_CONFIG_GNSS_SOURCE_*
    uint32_t gnss2Source;
    uint32_t g1g2;            //!< IO_CONFIG_G1G2_CAN_BUS, _COM2, or 0 (nothing on pins 8/10 from ioConfig)
    bool     spi;             //!< SPI enabled on G6/G7 (pins 7-10)
};

const uint32_t DIS  = IO_CONFIG_GNSS_SOURCE_DISABLE;
const uint32_t S0   = IO_CONFIG_GNSS_SOURCE_SER0;
const uint32_t S1   = IO_CONFIG_GNSS_SOURCE_SER1;
const uint32_t S2   = IO_CONFIG_GNSS_SOURCE_SER2;
const uint32_t CAN  = IO_CONFIG_G1G2_CAN_BUS;
const uint32_t COM2 = IO_CONFIG_G1G2_COM2;

// RUG-3, indexed by preset 1..9 (names: PLATFORM_CFG_RUG3_PRESET__n__...). GNSS2 only exists where the name says S0_GNSS2.
const PresetExpect kRug3[PLATFORM_CFG_RUG3_PRESET__COUNT] = {
    /* 0 */ { DIS, DIS, 0,    false },   // presets disabled: not exercised through this table
    /* 1 */ { S1,  DIS, CAN,  false },   // S0_RS232 CAN S1_GNSS1
    /* 2 */ { S1,  DIS, CAN,  false },   // S0_TTL   CAN S1_GNSS1
    /* 3 */ { S1,  DIS, COM2, false },   // S0_TTL  S2_TTL S1_GNSS1
    /* 4 */ { S2,  DIS, 0,    false },   // S0_RS232 S1_RS232 S2_GNSS1
    /* 5 */ { S2,  S0,  0,    false },   // S1_RS485 S2_GNSS1 S0_GNSS2
    /* 6 */ { S2,  S0,  0,    true  },   // SPI S2_GNSS1 S0_GNSS2
    /* 7 */ { S2,  S0,  0,    false },   // S1_RS232 S2_GNSS1 S0_GNSS2
    /* 8 */ { S1,  S0,  CAN,  false },   // CAN S1_GNSS1 S0_GNSS2
    /* 9 */ { S1,  S0,  COM2, false },   // S2_TTL S1_GNSS1 S0_GNSS2
};

// RUG-4, indexed by preset 1..9 (names: PLATFORM_CFG_RUG4_PRESET__n__...). One GNSS UART carries both receivers (GNSS1+2).
const PresetExpect kRug4[PLATFORM_CFG_RUG4_PRESET__COUNT] = {
    /* 0 */ { DIS, DIS, 0,    false },   // presets disabled: GNSS routing is left disabled
    /* 1 */ { S1,  S1,  CAN,  false },   // S0_RS232 CAN S1_GNSS
    /* 2 */ { S1,  S1,  CAN,  false },   // S0_TTL   CAN S1_GNSS
    /* 3 */ { S1,  S1,  COM2, false },   // S0_TTL  S2_TTL S1_GNSS
    /* 4 */ { S2,  S2,  0,    false },   // S0_RS232 S1_RS232 S2_GNSS
    /* 5 */ { S2,  S2,  0,    false },   // S1_RS485 S2_GNSS
    /* 6 */ { S2,  S2,  0,    true  },   // SPI S2_GNSS
    /* 7 */ { S2,  S2,  0,    false },   // S1_RS232 S2_GNSS (RUG-4 default)
    /* 8 */ { S1,  S1,  CAN,  false },   // CAN S1_GNSS
    /* 9 */ { S1,  S1,  COM2, false },   // S2_TTL S1_GNSS
};

/** Carrier types the family functions accept, with the receiver types each is wired for. */
struct TypeExpect {
    Family   family;
    uint32_t type;
    const char* name;
    uint32_t gnss1Type;
    uint32_t gnss2Type;
    bool     noGnss;          //!< "-G0": no receiver fitted, so both sources must be DISABLE whatever the preset says
};

const TypeExpect kTypes[] = {
    { RUG3, PLATFORM_CFG_TYPE_RUGn_G0,  "RUGn_G0 (RUG3 half)", IO_CONFIG_GNSS_TYPE_NONE,      IO_CONFIG_GNSS_TYPE_NONE,      true  },
    { RUG3, PLATFORM_CFG_TYPE_RUG3_G1,  "RUG3_G1",             IO_CONFIG_GNSS_TYPE_UBLOX,     IO_CONFIG_GNSS_TYPE_NONE,      false },
    { RUG3, PLATFORM_CFG_TYPE_RUG3_G2,  "RUG3_G2",             IO_CONFIG_GNSS_TYPE_UBLOX,     IO_CONFIG_GNSS_TYPE_UBLOX,     false },
    { RUG4, PLATFORM_CFG_TYPE_RUGn_G0,  "RUGn_G0 (RUG4 half)", IO_CONFIG_GNSS_TYPE_NONE,      IO_CONFIG_GNSS_TYPE_NONE,      true  },
    { RUG4, PLATFORM_CFG_TYPE_RUG4_X20, "RUG4_X20",            IO_CONFIG_GNSS_TYPE_UBLOX,     IO_CONFIG_GNSS_TYPE_UBLOX,     false },
    { RUG4, PLATFORM_CFG_TYPE_RUG4_SG5, "RUG4_SG5",            IO_CONFIG_GNSS_TYPE_SEPTENTRIO, IO_CONFIG_GNSS_TYPE_SEPTENTRIO, false },
    { RUG4, PLATFORM_CFG_TYPE_RUG4_GPX, "RUG4_GPX",            IO_CONFIG_GNSS_TYPE_GPX,       IO_CONFIG_GNSS_TYPE_GPX,       false },
};

uint32_t platformConfigOf(uint32_t type, uint32_t preset) { return type | (preset << PLATFORM_CFG_PRESET_OFFSET); }

/** Runs the family function from a given starting ioConfig. */
uint32_t mapIoConfig(Family f, uint32_t type, uint32_t preset, uint32_t seed) {
    uint32_t io = seed;
    if (f == RUG3) imxPlatformConfigToRug3FlashCfgIoConfig(&io, platformConfigOf(type, preset));
    else           imxPlatformConfigToRug4FlashCfgIoConfig(&io, platformConfigOf(type, preset));
    return io;
}

std::string label(const TypeExpect& t, uint32_t preset) { return std::string(t.name) + " preset " + std::to_string(preset); }

const uint32_t kSeeds[] = { IO_CONFIG_DEFAULT, 0xFFFFFFFFu, 0u };   //!< stock, every bit set (stale fields must be cleared), all clear

} // namespace


// Every preset 1-9 on every carrier: GNSS routing, G1/G2 function and SPI follow the preset; types follow the carrier.
TEST(RugPresetIoConfig, EveryPresetMapsGnssRoutingPinFunctionAndTypes) {
    for (const TypeExpect& t : kTypes) {
        const PresetExpect* table = (t.family == RUG3) ? kRug3 : kRug4;
        const uint32_t count = (t.family == RUG3) ? PLATFORM_CFG_RUG3_PRESET__COUNT : PLATFORM_CFG_RUG4_PRESET__COUNT;
        for (uint32_t preset = 1; preset < count; preset++) {
            for (uint32_t seed : kSeeds) {
                SCOPED_TRACE(label(t, preset) + " seed 0x" + std::to_string(seed));
                const uint32_t io = mapIoConfig(t.family, t.type, preset, seed);
                const PresetExpect& e = table[preset];

                EXPECT_EQ(IO_CONFIG_GNSS1_SOURCE(io), t.noGnss ? DIS : e.gnss1Source) << "GNSS1 source";
                EXPECT_EQ(IO_CONFIG_GNSS2_SOURCE(io), t.noGnss ? DIS : e.gnss2Source) << "GNSS2 source";
                EXPECT_EQ(IO_CONFIG_GNSS1_TYPE(io), t.gnss1Type) << "GNSS1 type";
                EXPECT_EQ(IO_CONFIG_GNSS2_TYPE(io), t.gnss2Type) << "GNSS2 type";
                EXPECT_EQ(io & IO_CONFIG_G1G2_MASK, e.g1g2) << "G1/G2 function";
                EXPECT_EQ(io & IO_CONFIG_G5G8_MASK, e.spi ? (uint32_t)IO_CONFIG_G5G8_G6G7_SPI_ENABLE : (uint32_t)IO_CONFIG_G5G8_DEFAULT) << "G5-G8 / SPI";
            }
        }
    }
}

// The CAN and SPI presets, called out because the loopback assertions behind them are DISABLED pending bench hardware (see SN-8688):
// the ioConfig side must still hold.
TEST(RugPresetIoConfig, CanAndSpiPresetsSelectTheirPinFunctions) {
    for (const TypeExpect& t : kTypes) {
        const uint32_t canRug3[] = { PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GNSS1, PLATFORM_CFG_RUG3_PRESET__2__S0_TTL_7_9_____CAN_11_12______S1_GNSS1, PLATFORM_CFG_RUG3_PRESET__8_________________CAN_11_12______S1_GNSS1__S0_GNSS2 };
        const uint32_t canRug4[] = { PLATFORM_CFG_RUG4_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GNSS, PLATFORM_CFG_RUG4_PRESET__2__S0_TTL_7_9_____CAN_11_12______S1_GNSS, PLATFORM_CFG_RUG4_PRESET__8_________________CAN_11_12______S1_GNSS };
        for (uint32_t p : (t.family == RUG3 ? std::vector<uint32_t>(canRug3, canRug3 + 3) : std::vector<uint32_t>(canRug4, canRug4 + 3)))
            EXPECT_EQ(mapIoConfig(t.family, t.type, p, IO_CONFIG_DEFAULT) & IO_CONFIG_G1G2_MASK, (uint32_t)IO_CONFIG_G1G2_CAN_BUS) << label(t, p);
        const uint32_t spi = (t.family == RUG3) ? PLATFORM_CFG_RUG3_PRESET__6__SPI_7_8_9_10__________________S2_GNSS1__S0_GNSS2 : PLATFORM_CFG_RUG4_PRESET__6__SPI_7_8_9_10__________________S2_GNSS;
        EXPECT_EQ(mapIoConfig(t.family, t.type, spi, IO_CONFIG_DEFAULT) & IO_CONFIG_G5G8_MASK, (uint32_t)IO_CONFIG_G5G8_G6G7_SPI_ENABLE) << label(t, spi);
    }
}

// A "-G0" board has no receiver. Whatever preset is selected, nothing may be left routed to one: both sources DISABLE and
// both types NONE, so gnssInitialize() leaves both slots disabled and raises no GFC_GNSSx_INIT (SN-8689, the RUG-4-G0 incident).
// Covered for both halves of the shared RUGn_G0 type: IMX-5 builds dispatch it to the RUG-3 function, IMX-6 builds to the RUG-4 function.
TEST(RugPresetIoConfig, NoGnssVariantsNeverRouteAUartToAReceiver) {
    for (const TypeExpect& t : kTypes) {
        if (!t.noGnss) continue;
        const uint32_t count = (t.family == RUG3) ? PLATFORM_CFG_RUG3_PRESET__COUNT : PLATFORM_CFG_RUG4_PRESET__COUNT;
        for (uint32_t preset = 0; preset < count; preset++) {
            for (uint32_t seed : kSeeds) {
                SCOPED_TRACE(label(t, preset));
                const uint32_t io = mapIoConfig(t.family, t.type, preset, seed);
                EXPECT_EQ(IO_CONFIG_GNSS1_SOURCE(io), (uint32_t)IO_CONFIG_GNSS_SOURCE_DISABLE);
                EXPECT_EQ(IO_CONFIG_GNSS2_SOURCE(io), (uint32_t)IO_CONFIG_GNSS_SOURCE_DISABLE);
                EXPECT_EQ(IO_CONFIG_GNSS1_TYPE(io), (uint32_t)IO_CONFIG_GNSS_TYPE_NONE);
                EXPECT_EQ(IO_CONFIG_GNSS2_TYPE(io), (uint32_t)IO_CONFIG_GNSS_TYPE_NONE);
            }
        }
    }
}

// The general form of the rule above, for every carrier and preset: a slot with a live GNSS source must have a receiver TYPE.
// (SOURCE != DISABLE with TYPE_NONE is what gnssInitialize() turns into GFC_GNSSx_INIT.)
// RUG3_G1 is excluded here and covered by the DISABLED_ test below, which fails today; see its comment.
static void expectLiveSourceHasType(const TypeExpect& t) {
    const uint32_t count = (t.family == RUG3) ? PLATFORM_CFG_RUG3_PRESET__COUNT : PLATFORM_CFG_RUG4_PRESET__COUNT;
    for (uint32_t preset = 0; preset < count; preset++) {
        const uint32_t io = mapIoConfig(t.family, t.type, preset, IO_CONFIG_DEFAULT);
        if (IO_CONFIG_GNSS1_SOURCE(io) != IO_CONFIG_GNSS_SOURCE_DISABLE)
            EXPECT_NE(IO_CONFIG_GNSS1_TYPE(io), (uint32_t)IO_CONFIG_GNSS_TYPE_NONE) << label(t, preset) << ": GNSS1 source live, type NONE";
        if (IO_CONFIG_GNSS2_SOURCE(io) != IO_CONFIG_GNSS_SOURCE_DISABLE)
            EXPECT_NE(IO_CONFIG_GNSS2_TYPE(io), (uint32_t)IO_CONFIG_GNSS_TYPE_NONE) << label(t, preset) << ": GNSS2 source live, type NONE";
    }
}

TEST(RugPresetIoConfig, LiveGnssSourceAlwaysHasAReceiverType) {
    for (const TypeExpect& t : kTypes)
        if (t.type != PLATFORM_CFG_TYPE_RUG3_G1)
            expectLiveSourceHasType(t);
}

// KNOWN FAILING (SN-8688 finding, reported not fixed): RUG3_G1 has one receiver (GNSS2 type NONE) but presets 5-9 route GNSS2 to Ser0,
// and G1's stock preset is 7 (imxPlatformConfigTypeToDefaultPlatformPreset() gives every RUG3 carrier but -G0 the G2 default). The ioConfig
// result -- GNSS2 source Ser0, type NONE -- is exactly the shape of the SN-8689 false-GFC_GNSS2_INIT. Whether a RUG-3-G1 board in the field
// is affected has not been shown on hardware (none on the bench). Re-enable by removing DISABLED_ once the mapping or the preset choice for
// G1 is settled by whoever owns the RUG-3 products.
TEST(RugPresetIoConfig, DISABLED_Rug3G1LiveGnss2SourceHasAReceiverType) {
    expectLiveSourceHasType(kTypes[1]);
}

// Preset 0 ("presets disabled") must not touch GNSS routing on RUG-3 (the user owns it); the pin-function fields are still normalised.
TEST(RugPresetIoConfig, Rug3PresetZeroLeavesGnssSourcesAlone) {
    uint32_t seed = IO_CONFIG_DEFAULT;
    SET_IO_CFG_GNSS1_SOURCE(seed, IO_CONFIG_GNSS_SOURCE_SER0);
    SET_IO_CFG_GNSS2_SOURCE(seed, IO_CONFIG_GNSS_SOURCE_SER2);
    const uint32_t io = mapIoConfig(RUG3, PLATFORM_CFG_TYPE_RUG3_G2, PLATFORM_CFG_RUG3_PRESET__0__PRESETS_DISABLED, seed);
    EXPECT_EQ(IO_CONFIG_GNSS1_SOURCE(io), (uint32_t)IO_CONFIG_GNSS_SOURCE_SER0);
    EXPECT_EQ(IO_CONFIG_GNSS2_SOURCE(io), (uint32_t)IO_CONFIG_GNSS_SOURCE_SER2);
}

// A carrier that belongs to the other family is ignored by a family function (it must not rewrite someone else's ioConfig).
TEST(RugPresetIoConfig, FamilyFunctionsIgnoreForeignCarriers) {
    const uint32_t seed = 0x12345678u;
    uint32_t io = seed;
    imxPlatformConfigToRug3FlashCfgIoConfig(&io, platformConfigOf(PLATFORM_CFG_TYPE_RUG4_X20, 7));
    EXPECT_EQ(io, seed) << "RUG-3 function touched a RUG-4 carrier";
    io = seed;
    imxPlatformConfigToRug4FlashCfgIoConfig(&io, platformConfigOf(PLATFORM_CFG_TYPE_RUG3_G2, 7));
    EXPECT_EQ(io, seed) << "RUG-4 function touched a RUG-3 carrier";
    io = seed;
    imxPlatformConfigToRug4FlashCfgIoConfig(&io, platformConfigOf(PLATFORM_CFG_TYPE_IG2, 1));
    EXPECT_EQ(io, seed);
}

// The public entry point, per product: stock default preset per carrier gives the documented default routing.
TEST(RugPresetIoConfig, StockDefaultPresetThroughThePublicEntryPoint) {
    struct { uint32_t type; uint32_t g1Src, g2Src; } expect[] = {
        { PLATFORM_CFG_TYPE_RUG3_G2,  S2, S0 },     // RUG-3-G2 default: Ser2 GNSS1, Ser0 GNSS2
        { PLATFORM_CFG_TYPE_RUG4_X20, S2, S2 },     // RUG-4 default: Ser2 GNSS1+2
        { PLATFORM_CFG_TYPE_RUG4_SG5, S2, S2 },
        { PLATFORM_CFG_TYPE_RUG4_GPX, S2, S2 },
    };
    for (auto& e : expect) {
        uint32_t io = IO_CONFIG_DEFAULT; uint8_t io2 = 0;
        imxPlatformConfigToFlashCfgIoConfig(&io, &io2, imxPlatformConfigTypeToDefaultPlatformConfig(e.type));
        EXPECT_EQ(IO_CONFIG_GNSS1_SOURCE(io), e.g1Src) << "type " << e.type;
        EXPECT_EQ(IO_CONFIG_GNSS2_SOURCE(io), e.g2Src) << "type " << e.type;
    }
}
