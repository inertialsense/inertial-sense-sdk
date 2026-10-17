/**
 * @file test_cbor_rmc.cpp
 * @brief RMC_STORED over per-port rmci_t: encoding of stored DIDs and options, selection, and all-or-nothing writes
 *        (preset first, DID entries after, a multiple of 0 removes, phase-1 rejections, RMC_SAVE_PORTS).
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "cbor/cbor_registry.h"
#include "cbor/cbor_rmc.h"

typedef std::vector<uint8_t> Bytes;

struct Store
{
    rmci_t ports[3] = {};
    rmci_t runtime[3] = {};
    int presetCalls = 0;
    cbor_rmc_store_t s;

    Store()
    {
        s = { ports, 3, presetDefaults, savePort, this };
    }
    static void presetDefaults(rmci_t *r, uint64_t, void *ctx)
    {
        ((Store *)ctx)->presetCalls++;
        memset(r->periodMultiple, 1, sizeof(r->periodMultiple));
    }
    static void savePort(uint8_t port, void *ctx)
    {
        Store *st = (Store *)ctx;
        st->ports[port] = st->runtime[port];
    }

    uint8_t write(const Bytes &msg, cbor_decode_result_t *res)
    {
        nanocbor_value_t it;
        nanocbor_decoder_init(&it, msg.data(), msg.size());
        memset(res, 0, sizeof(*res));
        uint8_t st = cbor_rmc_decode(&s, &it, res);
        EXPECT_TRUE(nanocbor_at_end(&it));
        return st;
    }
    Bytes read(const std::vector<cbor_path_t> &paths = {}, size_t size = 512)
    {
        Bytes buf(size);
        int n = cbor_rmc_encode(&s, paths.empty() ? nullptr : paths.data(), paths.size(), buf.data(), buf.size());
        buf.resize(n > 0 ? (size_t)n : 0);
        return buf;
    }
};

static Bytes did(uint16_t d)
{
    return d < 24 ? Bytes{ (uint8_t)d } : Bytes{ 0x18, (uint8_t)d };
}

static Bytes cat(std::initializer_list<Bytes> parts)
{
    Bytes out;
    for (auto &p : parts)
        out.insert(out.end(), p.begin(), p.end());
    return out;
}

static const Bytes SUB_MULTI = { 0x19, 0x93, 0x01 };
static const Bytes SUB_PERIOD = { 0x19, 0x93, 0x00 };
static const Bytes PRESET = { 0x19, 0x80, 0x01 };
static const Bytes SPEED = { 0x19, 0x80, 0x02 };
static const Bytes SAVE_PORTS = { 0x19, 0x80, 0x00 };

TEST(CborRmc, EncodesStoredDidsWithTheirMultiples)
{
    Store st;
    st.ports[1].rmc.bits = RMC_BITS_INS2 | RMC_BITS_GNSS1_POS;
    st.ports[1].periodMultiple[DID_INS_2] = 4;
    st.ports[1].periodMultiple[DID_GNSS1_POS] = 1;
    st.ports[1].rmc.options = (uint32_t)RMC_OPTIONS_NMEA_SPEED_FILTER_ENABLE << RMC_OPTIONS_NMEA_SPEED_FILTER_OFFSET;
    // {0: {}, 1: {INS_2: {SUB_RATE_MULTI: 4}, GNSS1_POS: {SUB_RATE_MULTI: 1}, NMEA_SPEED_FILTER: ENABLE}, 2: {}}
    Bytes want = cat({ { 0xA3, 0x00, 0xA0, 0x01, 0xA3 }, did(DID_INS_2), { 0xA1 }, SUB_MULTI, { 0x04 },
                       did(DID_GNSS1_POS), { 0xA1 }, SUB_MULTI, { 0x01 }, SPEED, { 0x01 }, { 0x02, 0xA0 } });
    EXPECT_EQ(st.read(), want);
}

TEST(CborRmc, SelectionPicksPortsAndEntries)
{
    Store st;
    st.ports[0].rmc.bits = RMC_BITS_INS2 | RMC_BITS_GNSS1_POS;
    st.ports[0].periodMultiple[DID_INS_2] = 2;
    cbor_path_t p = { 2, { 0, DID_INS_2 } };
    EXPECT_EQ(st.read({ p }), cat({ { 0xA1, 0x00, 0xA1 }, did(DID_INS_2), { 0xA1 }, SUB_MULTI, { 0x02 } }));
    cbor_path_t q = { 1, { 2 } };
    EXPECT_EQ(st.read({ q }), (Bytes{ 0xA1, 0x02, 0xA0 }));
}

TEST(CborRmc, PortThatDoesNotFitIsMarked)
{
    Store st;
    st.ports[0].rmc.bits = RMC_BITS_INS2 | RMC_BITS_GNSS1_POS;
    Bytes b = st.read({}, 16);
    // port 0 marked; ports 1 and 2 ({}) still fit
    EXPECT_EQ(b, (Bytes{ 0xA3, 0x00, 0xA1, 0x19, 0xC0, 0x08, CBOR_MSG_STATUS_RESP_TOO_LARGE, 0x01, 0xA0, 0x02, 0xA0 }));
}

TEST(CborRmc, WriteAddsChangesAndRemovesDids)
{
    Store st;
    st.ports[0].rmc.bits = RMC_BITS_GNSS1_POS;
    st.ports[0].periodMultiple[DID_GNSS1_POS] = 5;
    cbor_decode_result_t res;
    // {0: {INS_2: {SUB_RATE_MULTI: 3}, GNSS1_POS: {SUB_RATE_MULTI: 0}}}
    Bytes msg = cat({ { 0xA1, 0x00, 0xA2 }, did(DID_INS_2), { 0xA1 }, SUB_MULTI, { 0x03 },
                      did(DID_GNSS1_POS), { 0xA1 }, SUB_MULTI, { 0x00 } });
    ASSERT_EQ(st.write(msg, &res), CBOR_MSG_STATUS_OK);
    EXPECT_EQ(st.ports[0].rmc.bits, RMC_BITS_INS2);
    EXPECT_EQ(st.ports[0].periodMultiple[DID_INS_2], 3);

    // an entry without a rate keeps the current multiple, or uses the source rate for a new DID
    msg = cat({ { 0xA1, 0x00, 0xA2 }, did(DID_INS_2), { 0xA0 }, did(DID_GNSS1_POS), { 0xA0 } });
    ASSERT_EQ(st.write(msg, &res), CBOR_MSG_STATUS_OK);
    EXPECT_EQ(st.ports[0].periodMultiple[DID_INS_2], 3);
    EXPECT_EQ(st.ports[0].periodMultiple[DID_GNSS1_POS], 1);
}

TEST(CborRmc, PresetAppliesBeforeEntriesWhateverTheOrder)
{
    Store st;
    cbor_decode_result_t res;
    // {1: {INS_2: {SUB_RATE_MULTI: 9}, RMC_PRESET: INS}} -- the entry is written after the preset
    Bytes msg = cat({ { 0xA1, 0x01, 0xA2 }, did(DID_INS_2), { 0xA1 }, SUB_MULTI, { 0x09 }, PRESET,
                      { (uint8_t)CBOR_RMC_PRESET_INS } });
    ASSERT_EQ(st.write(msg, &res), CBOR_MSG_STATUS_OK);
    EXPECT_EQ(st.presetCalls, 1);
    EXPECT_EQ(st.ports[1].rmc.bits, RMC_PRESET_INS | RMC_BITS_INS2);
    EXPECT_EQ(st.ports[1].periodMultiple[DID_INS_2], 9);
}

TEST(CborRmc, InvalidValuesRejectTheWholeWrite)
{
    struct Case { const char *what; Bytes msg; Bytes errPath; };
    const Case cases[] = {
        { "a period (phase 1 stores multiples)",
          cat({ { 0xA2, 0x00, 0xA1 }, did(DID_INS_2), { 0xA1 }, SUB_PERIOD, { 0x18, 0x64 },
                { 0x01, 0xA1 }, did(DID_INS_1), { 0xA1 }, SUB_MULTI, { 0x01 } }),
          cat({ { 0x83, 0x00 }, did(DID_INS_2), SUB_PERIOD }) },
        { "a DID without an RMC bit",
          cat({ { 0xA2, 0x00, 0xA1 }, did(DID_FLASH_CONFIG), { 0xA0 }, { 0x01, 0xA1 }, did(DID_INS_1), { 0xA0 } }),
          cat({ { 0x82, 0x00 }, did(DID_FLASH_CONFIG) }) },
        { "a port the device does not have",
          cat({ { 0xA2, 0x05, 0xA0, 0x01, 0xA1 }, did(DID_INS_1), { 0xA0 } }), { 0x05 } },
        { "an unknown preset",
          cat({ { 0xA2, 0x00, 0xA1 }, PRESET, { 0x17 }, { 0x01, 0xA1 }, did(DID_INS_1), { 0xA0 } }),
          cat({ { 0x82, 0x00 }, PRESET }) },
        { "a multiple out of range",
          cat({ { 0xA2, 0x00, 0xA1 }, did(DID_INS_2), { 0xA1 }, SUB_MULTI, { 0x19, 0x01, 0x00 },
                { 0x01, 0xA1 }, did(DID_INS_1), { 0xA0 } }),
          cat({ { 0x83, 0x00 }, did(DID_INS_2), SUB_MULTI }) },
    };
    for (const Case &c : cases)
    {
        SCOPED_TRACE(c.what);
        Store st;
        cbor_decode_result_t res;
        EXPECT_EQ(st.write(c.msg, &res), CBOR_MSG_STATUS_BAD_VALUE);
        EXPECT_EQ(st.ports[1].rmc.bits, 0u) << "the valid port is not applied either";
        ASSERT_EQ(res.errCount, 1);
        uint8_t buf[16];
        nanocbor_encoder_t enc;
        nanocbor_encoder_init(&enc, buf, sizeof buf);
        cbor_codec_encode_path(&enc, &res.err[0]);
        EXPECT_EQ(Bytes(buf, (uint8_t *)enc.cur), c.errPath);
    }
}

TEST(CborRmc, SavePortsCopiesRuntimeBeforeEntries)
{
    Store st;
    st.runtime[2].rmc.bits = RMC_BITS_INS2 | RMC_BITS_GNSS1_POS;
    st.runtime[2].periodMultiple[DID_INS_2] = 7;
    cbor_decode_result_t res;
    // {2: {GNSS1_POS: {SUB_RATE_MULTI: 0}}, RMC_SAVE_PORTS: [2]} -- saved first, then the entry removes GNSS1_POS
    Bytes msg = cat({ { 0xA2, 0x02, 0xA1 }, did(DID_GNSS1_POS), { 0xA1 }, SUB_MULTI, { 0x00 }, SAVE_PORTS, { 0x81, 0x02 } });
    ASSERT_EQ(st.write(msg, &res), CBOR_MSG_STATUS_OK);
    EXPECT_EQ(st.ports[2].rmc.bits, RMC_BITS_INS2);
    EXPECT_EQ(st.ports[2].periodMultiple[DID_INS_2], 7);

    msg = cat({ { 0xA1 }, SAVE_PORTS, { 0x81, 0x09 } });
    EXPECT_EQ(st.write(msg, &res), CBOR_MSG_STATUS_BAD_VALUE) << "a port the device does not have";
}

TEST(CborRmc, UnknownKeysAreIgnored)
{
    Store st;
    cbor_decode_result_t res;
    Bytes msg = cat({ { 0xA2, 0x19, 0xC0, 0x04, 0x02 /* MSG_ACK_TYPE */, 0x00, 0xA2 }, did(DID_INS_2),
                      { 0xA2, 0x19, 0x93, 0x7F, 0x01 }, SUB_MULTI, { 0x02 }, { 0x19, 0x8F, 0x00, 0x01 } });
    ASSERT_EQ(st.write(msg, &res), CBOR_MSG_STATUS_OK);
    EXPECT_EQ(st.ports[0].periodMultiple[DID_INS_2], 2);
}

TEST(CborRmc, NmeaSentencesAreNmeaPseudoDids)
{
    Store st;
    st.ports[0].rmcNmea.nmeaBits = 1u << NMEA_MSG_ID_GNGGA;
    st.ports[0].rmcNmea.nmeaPeriod[NMEA_MSG_ID_GNGGA] = 5;
    Bytes gga = { 0x19, 0x3F, (uint8_t)NMEA_MSG_ID_GNGGA };
    EXPECT_EQ(st.read({ cbor_path_t{ 1, { 0 } } }), cat({ { 0xA1, 0x00, 0xA1 }, gga, { 0xA1 }, SUB_MULTI, { 0x05 } }));

    cbor_decode_result_t res;
    Bytes rmcMsg = { 0x19, 0x3F, (uint8_t)NMEA_MSG_ID_GNRMC };
    // {0: {NMEA_GNGGA: {SUB_RATE_MULTI: 0}, NMEA_GNRMC: {SUB_RATE_MULTI: 2}}}
    Bytes msg = cat({ { 0xA1, 0x00, 0xA2 }, gga, { 0xA1 }, SUB_MULTI, { 0x00 }, rmcMsg, { 0xA1 }, SUB_MULTI, { 0x02 } });
    ASSERT_EQ(st.write(msg, &res), CBOR_MSG_STATUS_OK);
    EXPECT_EQ(st.ports[0].rmcNmea.nmeaBits, 1u << NMEA_MSG_ID_GNRMC);
    EXPECT_EQ(st.ports[0].rmcNmea.nmeaPeriod[NMEA_MSG_ID_GNRMC], 2);
}

TEST(CborRmc, LoadSkipsEntriesItCannotApply)
{
    Store st;
    // {0: {FLASH_CONFIG (no RMC bit): {}, INS_2: {SUB_RATE_PERIOD: 100}, GNSS1_POS: {SUB_RATE_MULTI: 2}}, 9: {}}
    Bytes msg = cat({ { 0xA2, 0x00, 0xA3 }, did(DID_FLASH_CONFIG), { 0xA0 }, did(DID_INS_2), { 0xA1 }, SUB_PERIOD,
                      { 0x18, 0x64 }, did(DID_GNSS1_POS), { 0xA1 }, SUB_MULTI, { 0x02 }, { 0x09, 0xA0 } });
    nanocbor_value_t it;
    nanocbor_decoder_init(&it, msg.data(), msg.size());
    cbor_rmc_load(&st.s, &it);
    EXPECT_TRUE(nanocbor_at_end(&it));
    EXPECT_EQ(st.ports[0].rmc.bits, RMC_BITS_GNSS1_POS);
    EXPECT_EQ(st.ports[0].periodMultiple[DID_GNSS1_POS], 2);
}
