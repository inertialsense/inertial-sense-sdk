/**
 * @file test_cbor_msg.cpp
 * @brief The CBOR message layer: envelope parsing and defaults, device-filter matching, and a device answering DATA
 *        and GET per DID — all-or-nothing outcomes, unowned DIDs left out, too-large marking, acknowledgement levels,
 *        CAPABILITIES, and SUBSCRIBE answered UNSUPPORTED.
 */

#include <gtest/gtest.h>
#include <cstddef>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "cbor/cbor_msg.h"
#include "cbor/cbor_registry.h"
#include "ISComm.h"

typedef std::vector<uint8_t> Bytes;

/*-------------------------------------------------------------------------------------------- CBOR helpers */

/** Builds CBOR with nanocbor */
class Cbor
{
public:
    Cbor() : m_buf(4096) { nanocbor_encoder_init(&m_enc, m_buf.data(), m_buf.size()); }
    Cbor &map(size_t n)         { nanocbor_fmt_map(&m_enc, n); return *this; }
    Cbor &arr(size_t n)         { nanocbor_fmt_array(&m_enc, n); return *this; }
    Cbor &u(uint64_t v)         { nanocbor_fmt_uint(&m_enc, v); return *this; }
    Cbor &i(int64_t v)          { nanocbor_fmt_int(&m_enc, v); return *this; }
    Cbor &f(float v)            { nanocbor_fmt_float(&m_enc, v); return *this; }
    Cbor &t(const char *s)      { nanocbor_put_tstr(&m_enc, s); return *this; }
    Cbor &b(bool v)             { nanocbor_fmt_bool(&m_enc, v); return *this; }
    Bytes bytes() const         { return Bytes(m_buf.data(), (const uint8_t *)m_enc.cur); }
private:
    Bytes m_buf;
    nanocbor_encoder_t m_enc;
};

/** A decoded CBOR item, for inspecting replies */
struct Item
{
    int type = -1;
    uint64_t u = 0;
    int64_t i = 0;
    double f = 0;
    std::vector<Item> arr;
    std::vector<std::pair<uint64_t, Item>> map;

    const Item *get(uint64_t key) const
    {
        for (auto &e : map)
            if (e.first == key)
                return &e.second;
        return nullptr;
    }
    std::vector<uint64_t> keys() const
    {
        std::vector<uint64_t> k;
        for (auto &e : map)
            k.push_back(e.first);
        return k;
    }
};

static bool parse(nanocbor_value_t *it, Item &out)
{
    out.type = nanocbor_get_type(it);
    switch (out.type)
    {
    case NANOCBOR_TYPE_UINT: return nanocbor_get_uint64(it, &out.u) >= 0;
    case NANOCBOR_TYPE_NINT: return nanocbor_get_int64(it, &out.i) >= 0;
    case NANOCBOR_TYPE_FLOAT:
    {
        bool bv;
        if (nanocbor_get_bool(it, &bv) >= 0) { out.u = bv; return true; }
        return nanocbor_get_double(it, &out.f) >= 0;
    }
    case NANOCBOR_TYPE_ARR:
    {
        nanocbor_value_t a;
        if (nanocbor_enter_array(it, &a) < 0) return false;
        while (!nanocbor_at_end(&a))
        {
            out.arr.emplace_back();
            if (!parse(&a, out.arr.back())) return false;
        }
        return nanocbor_leave_container(it, &a) >= 0;
    }
    case NANOCBOR_TYPE_MAP:
    {
        nanocbor_value_t m;
        if (nanocbor_enter_map(it, &m) < 0) return false;
        while (!nanocbor_at_end(&m))
        {
            uint64_t k;
            if (nanocbor_get_uint64(&m, &k) < 0) return false;
            out.map.emplace_back(k, Item());
            if (!parse(&m, out.map.back().second)) return false;
        }
        return nanocbor_leave_container(it, &m) >= 0;
    }
    default:
        return nanocbor_skip(it) >= 0;
    }
}

static Item parseReply(const uint8_t *buf, int len)
{
    Item root;
    nanocbor_value_t it;
    nanocbor_decoder_init(&it, buf, (size_t)len);
    EXPECT_TRUE(parse(&it, root));
    EXPECT_TRUE(nanocbor_at_end(&it));
    return root;
}

/*-------------------------------------------------------------------------------------------- test device */

static const uint64_t IMX_ID = 0x0D400000'0000F54Dull;
static const uint64_t GPX_ID = 0x10400000'3756473Dull;

static const uint16_t DID_CFG = 0x0100;     // writable; clamps `rate` to 100
static const uint16_t DID_RO  = 0x0101;     // read-only

struct Cfg
{
    uint32_t rate;
    int16_t  offset[3];
    uint8_t  enabled;
    char     name[16];
};

static const cbor_field_t s_cfgFields[] = {
    { 0, CBOR_T_UINT,  4, 0,  offsetof(Cfg, rate),    nullptr },
    { 1, CBOR_T_INT,   2, 3,  offsetof(Cfg, offset),  nullptr },
    { 2, CBOR_T_BOOL,  1, 0,  offsetof(Cfg, enabled), nullptr },
    { 3, CBOR_T_TEXT,  1, 16, offsetof(Cfg, name),    nullptr },
};
static const cbor_schema_t s_cfg = { s_cfgFields, 4 };

struct Bench
{
    Cfg cfg = {};
    Cfg ro = {};
    nvm_flash_cfg_t flash = {};
    int writes = 0;
    uint64_t gpsNs = 0;
    uint64_t uptimeNs = 4812000000ull;
    uint8_t reply[ISB_MAX_DATASET_SIZE];
    alignas(8) uint8_t scratch[sizeof(nvm_flash_cfg_t)];
    cbor_did_t dids[3];
    cbor_device_t dev;

    Bench(size_t replySize = ISB_MAX_DATASET_SIZE)
    {
        cfg.rate = 10;
        strcpy(cfg.name, "imx");
        ro.rate = 7;
        flash.ser0BaudRate = 921600;
        dids[0] = { DID_CFG, &s_cfg, sizeof(Cfg), readFn, writeCfg };
        dids[1] = { DID_RO, &s_cfg, sizeof(Cfg), readFn, nullptr };
        dids[2] = { CBOR_DID_FLASH_CFG, &g_cborSchema_FLASH_CFG, sizeof(nvm_flash_cfg_t), readFn, writeFlash };
        dev = { IMX_ID, dids, 3, reply, replySize, scratch, sizeof(scratch), timeFn, this };
    }

    static const void *readFn(const cbor_did_t *d, void *ctx)
    {
        Bench *b = (Bench *)ctx;
        if (d->did == DID_CFG) return &b->cfg;
        if (d->did == DID_RO) return &b->ro;
        return &b->flash;
    }
    static uint8_t writeCfg(const cbor_did_t *, void *ctx, const void *values)
    {
        Bench *b = (Bench *)ctx;
        memcpy(&b->cfg, values, sizeof(Cfg));
        if (b->cfg.rate > 100)
            b->cfg.rate = 100;
        b->writes++;
        return CBOR_MSG_STATUS_OK;
    }
    static uint8_t writeFlash(const cbor_did_t *, void *ctx, const void *values)
    {
        Bench *b = (Bench *)ctx;
        const nvm_flash_cfg_t *v = (const nvm_flash_cfg_t *)values;
        if (v->ser0BaudRate == 0)
            return CBOR_MSG_STATUS_BAD_VALUE;   // rejected by the device's own validation
        b->flash = *v;
        b->writes++;
        return CBOR_MSG_STATUS_OK;
    }
    static void timeFn(void *ctx, uint64_t *gps, uint64_t *up)
    {
        Bench *b = (Bench *)ctx;
        *gps = b->gpsNs;
        *up = b->uptimeNs;
    }

    int handle(const Bytes &msg) { return cbor_device_handle(&dev, msg.data(), msg.size()); }
    Item send(const Bytes &msg)
    {
        int n = handle(msg);
        EXPECT_GT(n, 0);
        return n > 0 ? parseReply(reply, n) : Item();
    }
};

/*-------------------------------------------------------------------------------------------- envelope */

TEST(CborMsg, EnvelopeDefaults)
{
    Bytes msg = Cbor().map(1).u(DID_CFG).arr(0).bytes();
    cbor_envelope_t env;
    ASSERT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_OK);
    EXPECT_EQ(env.type, CBOR_MSG_TYPE_GET) << "keys only means GET";
    EXPECT_EQ(env.ackType, CBOR_ACK_TYPE_ACK_MINIMAL);
    EXPECT_FALSE(env.hasSrc);
    EXPECT_FALSE(env.hasDst);
    EXPECT_FALSE(env.hasReqId);
    EXPECT_EQ(env.routeDepth, 0);
    EXPECT_EQ(env.didCount, 1);

    msg = Cbor().map(1).u(DID_CFG).map(1).u(0).u(5).bytes();
    ASSERT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_OK);
    EXPECT_EQ(env.type, CBOR_MSG_TYPE_DATA) << "values mean DATA";

    msg = Cbor().map(1).u(DID_CFG).map(1).u(CBOR_MSG_FIELDS).arr(1).u(0).bytes();
    ASSERT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_OK);
    EXPECT_EQ(env.type, CBOR_MSG_TYPE_GET) << "an options map without values is a GET";
}

TEST(CborMsg, EnvelopeRoundTrips)
{
    cbor_envelope_t in = {};
    in.type = CBOR_MSG_TYPE_DATA;
    in.ackType = CBOR_ACK_TYPE_ACK_DETAILED;
    in.hasSrc = true;  in.srcDevId = GPX_ID;
    in.hasDst = true;  in.dstFilter = 0;
    in.hasReqId = true; in.reqId = 0xBEEF;
    in.routeDepth = 2; in.route[0] = 3; in.route[1] = 1;
    in.hasTimeGps = true; in.timeGpsNs = 1460000000123456789ull;
    in.status = CBOR_MSG_STATUS_READ_ONLY;

    uint8_t buf[128];
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf + 1, sizeof(buf) - 1);
    int n = cbor_msg_encode_envelope(&enc, &in);
    ASSERT_EQ(n, 8);
    buf[0] = (uint8_t)(0xA0 + n);

    cbor_envelope_t out;
    ASSERT_EQ(cbor_msg_parse_envelope(buf, (size_t)(enc.cur - buf), &out), CBOR_CODEC_OK);
    EXPECT_EQ(out.type, in.type);
    EXPECT_EQ(out.ackType, in.ackType);
    EXPECT_EQ(out.srcDevId, in.srcDevId);
    EXPECT_TRUE(out.hasDst);
    EXPECT_EQ(out.dstFilter, 0u);
    EXPECT_EQ(out.reqId, in.reqId);
    ASSERT_EQ(out.routeDepth, 2);
    EXPECT_EQ(out.route[0], 3);
    EXPECT_EQ(out.route[1], 1);
    EXPECT_EQ(out.timeGpsNs, in.timeGpsNs);
    EXPECT_EQ(out.status, in.status);
}

TEST(CborMsg, EnvelopeIgnoresUnknownKeysAndRejectsBadValues)
{
    cbor_envelope_t env;
    Bytes msg = Cbor().map(2).u(0xF123).t("future").u(CBOR_MSG_REQ_ID).u(9).bytes();
    ASSERT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_OK);
    EXPECT_EQ(env.reqId, 9);

    msg = Cbor().map(1).u(CBOR_MSG_REQ_ID).u(70000).bytes();
    EXPECT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_ERR_VALUE);

    msg = Cbor().map(1).u(CBOR_MSG_ROUTE).arr(5).u(1).u(2).u(3).u(4).u(5).bytes();
    EXPECT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_ERR_VALUE) << "route too deep";

    msg = Cbor().map(1).u(CBOR_MSG_ACK_TYPE).u(3).bytes();
    EXPECT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_ERR_VALUE);
}

TEST(CborMsg, EnvelopeRejectsAnythingButOneMap)
{
    cbor_envelope_t env;
    Bytes msg = Cbor().arr(0).bytes();
    EXPECT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_ERR_MALFORMED);
    msg = Cbor().map(0).map(0).bytes();
    EXPECT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_ERR_MALFORMED) << "trailing bytes";
    msg = { 0xA1, 0x01 };
    EXPECT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_ERR_MALFORMED);
}

TEST(CborMsg, DeviceFilterMatchesPerSubField)
{
    EXPECT_TRUE(cbor_dev_filter_match(IMX_ID, IMX_ID));
    EXPECT_FALSE(cbor_dev_filter_match(IMX_ID, GPX_ID));
    EXPECT_TRUE(cbor_dev_filter_match(0x13FFFFFF'FFFFFFFFull, GPX_ID)) << "any GPX";
    EXPECT_FALSE(cbor_dev_filter_match(0x13FFFFFF'FFFFFFFFull, IMX_ID));
    EXPECT_TRUE(cbor_dev_filter_match(0x107FFFFF'FFFFFFFFull, GPX_ID)) << "any GPX-1.x";
    EXPECT_TRUE(cbor_dev_filter_match(0x1040FFFF'FFFFFFFFull, GPX_ID)) << "GPX-1.0, any serial";
    EXPECT_FALSE(cbor_dev_filter_match(0x1080FFFF'FFFFFFFFull, GPX_ID)) << "GPX-2.0";
    EXPECT_TRUE(cbor_dev_filter_match(UINT64_MAX, IMX_ID));
    EXPECT_FALSE(cbor_dev_filter_match(0, IMX_ID)) << "0 is the host";
    // IMX (3 = 0b11) is a bit-superset of UINS (1); a flat AND would wrongly match
    EXPECT_FALSE(cbor_dev_filter_match(0x07FFFFFF'FFFFFFFFull, IMX_ID));
}

/*-------------------------------------------------------------------------------------------- device: GET */

TEST(CborDevice, GetReturnsTheSelectedFieldsWithTheReplyEnvelope)
{
    Bench b;
    Item r = b.send(Cbor().map(2).u(CBOR_MSG_REQ_ID).u(3).u(DID_CFG).arr(2).u(0).u(3).bytes());
    EXPECT_EQ(r.keys(), (std::vector<uint64_t>{ CBOR_MSG_TYPE, CBOR_MSG_SRC_DEV_ID, CBOR_MSG_REQ_ID, CBOR_MSG_UPTIME_NS,
                                                DID_CFG }));
    EXPECT_EQ(r.get(CBOR_MSG_TYPE)->u, cbor_msg_reply_type(CBOR_MSG_TYPE_GET));
    EXPECT_EQ(r.get(CBOR_MSG_SRC_DEV_ID)->u, IMX_ID);
    EXPECT_EQ(r.get(CBOR_MSG_REQ_ID)->u, 3u);
    EXPECT_EQ(r.get(CBOR_MSG_UPTIME_NS)->u, b.uptimeNs);
    const Item *cfg = r.get(DID_CFG);
    ASSERT_NE(cfg, nullptr);
    EXPECT_EQ(cfg->keys(), (std::vector<uint64_t>{ 0, 3 }));
    EXPECT_EQ(cfg->get(0)->u, 10u);
}

TEST(CborDevice, GetFormsAndEmptySelection)
{
    Bench b;
    Item r = b.send(Cbor().map(1).u(DID_CFG).arr(0).bytes());
    EXPECT_EQ(r.get(DID_CFG)->keys(), (std::vector<uint64_t>{ 0, 1, 2, 3 })) << "[] = every field";

    r = b.send(Cbor().map(1).u(DID_CFG).map(1).u(CBOR_MSG_FIELDS).arr(1).u(2).bytes());
    EXPECT_EQ(r.get(DID_CFG)->keys(), (std::vector<uint64_t>{ 2 })) << "options-map form";

    r = b.send(Cbor().map(1).u(DID_CFG).map(0).bytes());
    EXPECT_EQ(r.get(DID_CFG)->keys().size(), 4u) << "options map without MSG_FIELDS = every field";
}

TEST(CborDevice, TimeIsGpsWhenSyncedAndCanBeOmitted)
{
    Bench b;
    b.gpsNs = 1460000000000000000ull;
    Item r = b.send(Cbor().map(1).u(DID_CFG).arr(0).bytes());
    EXPECT_EQ(r.get(CBOR_MSG_TIME_GPS_NS)->u, b.gpsNs);
    EXPECT_EQ(r.get(CBOR_MSG_UPTIME_NS), nullptr);

    r = b.send(Cbor().map(2).u(CBOR_MSG_OPTION_NO_TIME).b(true).u(DID_CFG).arr(0).bytes());
    EXPECT_EQ(r.get(CBOR_MSG_TIME_GPS_NS), nullptr);
    EXPECT_EQ(r.get(CBOR_MSG_UPTIME_NS), nullptr);
}

TEST(CborDevice, UnownedDidsAreLeftOut)
{
    Bench b;
    Item r = b.send(Cbor().map(2).u(0x0123).arr(0).u(DID_RO).arr(1).u(0).bytes());
    EXPECT_EQ(r.get(0x0123), nullptr);
    ASSERT_NE(r.get(DID_RO), nullptr);
    EXPECT_EQ(r.get(DID_RO)->get(0)->u, 7u);
}

TEST(CborDevice, BadSelectionIsABadRequestForThatDid)
{
    Bench b;
    Item r = b.send(Cbor().map(3).u(CBOR_MSG_TYPE).u(CBOR_MSG_TYPE_GET).u(DID_CFG).map(1).u(0).u(5)
                        .u(DID_RO).arr(1).u(0).bytes());
    EXPECT_EQ(r.get(DID_CFG)->get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_BAD_REQUEST) << "values in a GET";
    EXPECT_EQ(r.get(DID_RO)->get(0)->u, 7u) << "other DIDs are answered";
    EXPECT_EQ(b.cfg.rate, 10u);
}

TEST(CborDevice, ReplyIsAddressedToTheRequester)
{
    Bench b;
    // from a device
    Item r = b.send(Cbor().map(2).u(CBOR_MSG_SRC_DEV_ID).u(GPX_ID).u(DID_CFG).arr(0).bytes());
    EXPECT_EQ(r.get(CBOR_MSG_DST_DEV_FILTER)->u, GPX_ID);
    // from a host behind a forwarder: addressed to the host, route echoed
    r = b.send(Cbor().map(2).u(CBOR_MSG_ROUTE).arr(1).u(3).u(DID_CFG).arr(0).bytes());
    EXPECT_EQ(r.get(CBOR_MSG_DST_DEV_FILTER)->u, 0u);
    ASSERT_EQ(r.get(CBOR_MSG_ROUTE)->arr.size(), 1u);
    EXPECT_EQ(r.get(CBOR_MSG_ROUTE)->arr[0].u, 3u);
    // from the link peer: no destination
    r = b.send(Cbor().map(1).u(DID_CFG).arr(0).bytes());
    EXPECT_EQ(r.get(CBOR_MSG_DST_DEV_FILTER), nullptr);
}

TEST(CborDevice, ActsOnlyWhenAddressed)
{
    Bench b;
    EXPECT_EQ(b.handle(Cbor().map(2).u(CBOR_MSG_DST_DEV_FILTER).u(0x13FFFFFF'FFFFFFFFull).u(DID_CFG).arr(0).bytes()),
              CBOR_MSG_NOT_ADDRESSED);
    EXPECT_EQ(b.handle(Cbor().map(2).u(CBOR_MSG_DST_DEV_FILTER).u(0).u(DID_CFG).map(1).u(0).u(50).bytes()),
              CBOR_MSG_NOT_ADDRESSED) << "a message to the host is never applied";
    EXPECT_EQ(b.cfg.rate, 10u);
    EXPECT_GT(b.handle(Cbor().map(2).u(CBOR_MSG_DST_DEV_FILTER).u(UINT64_MAX).u(DID_CFG).arr(0).bytes()), 0);
}

/*-------------------------------------------------------------------------------------------- device: DATA */

TEST(CborDevice, DataAppliesOnTopOfCurrentValues)
{
    Bench b;
    b.cfg.offset[1] = 44;
    Item r = b.send(Cbor().map(2).u(CBOR_MSG_REQ_ID).u(4).u(DID_CFG).map(2).u(1).arr(1).i(-5).u(3).t("rover").bytes());
    EXPECT_EQ(b.cfg.offset[0], -5);
    EXPECT_EQ(b.cfg.offset[1], 44) << "elements not sent are unchanged";
    EXPECT_STREQ(b.cfg.name, "rover");
    EXPECT_EQ(b.cfg.rate, 10u) << "fields not sent are unchanged";
    // ACK_MINIMAL with nothing to report: envelope only
    EXPECT_EQ(r.get(CBOR_MSG_TYPE)->u, cbor_msg_reply_type(CBOR_MSG_TYPE_DATA));
    EXPECT_EQ(r.get(DID_CFG), nullptr);
    EXPECT_EQ(r.get(CBOR_MSG_REQ_ID)->u, 4u);
}

TEST(CborDevice, MinimalAckReportsOnlyValuesKeptDifferently)
{
    Bench b;
    Item r = b.send(Cbor().map(1).u(DID_CFG).map(2).u(0).u(500).u(2).b(true).bytes());
    EXPECT_EQ(b.cfg.rate, 100u);
    ASSERT_NE(r.get(DID_CFG), nullptr);
    EXPECT_EQ(r.get(DID_CFG)->keys(), (std::vector<uint64_t>{ 0 })) << "only the clamped field";
    EXPECT_EQ(r.get(DID_CFG)->get(0)->u, 100u);
}

TEST(CborDevice, DetailedAckReturnsEveryValueSentAsKept)
{
    Bench b;
    Item r = b.send(Cbor().map(2).u(CBOR_MSG_ACK_TYPE).u(CBOR_ACK_TYPE_ACK_DETAILED)
                        .u(DID_CFG).map(2).u(0).u(50).u(2).b(true).bytes());
    EXPECT_EQ(r.get(DID_CFG)->keys(), (std::vector<uint64_t>{ 0, 2 }));
    EXPECT_EQ(r.get(DID_CFG)->get(0)->u, 50u);
}

TEST(CborDevice, AckNoneSendsNothing)
{
    Bench b;
    EXPECT_EQ(b.handle(Cbor().map(2).u(CBOR_MSG_ACK_TYPE).u(CBOR_ACK_TYPE_ACK_NONE).u(DID_CFG).map(1).u(0).u(500)
                           .bytes()), 0);
    EXPECT_EQ(b.cfg.rate, 100u);
    EXPECT_EQ(b.handle(Cbor().map(2).u(CBOR_MSG_ACK_TYPE).u(CBOR_ACK_TYPE_ACK_NONE).u(DID_CFG).map(1).u(0).i(-1)
                           .bytes()), 0) << "not even errors";
}

TEST(CborDevice, DidLevelAckOverridesTheEnvelope)
{
    Bench b;
    Item r = b.send(Cbor().map(2).u(CBOR_MSG_ACK_TYPE).u(CBOR_ACK_TYPE_ACK_NONE)
                        .u(DID_CFG).map(2).u(CBOR_MSG_ACK_TYPE).u(CBOR_ACK_TYPE_ACK_DETAILED).u(0).u(20).bytes());
    EXPECT_EQ(r.get(DID_CFG)->get(0)->u, 20u);
}

TEST(CborDevice, OneInvalidValueRejectsOnlyItsDid)
{
    Bench b;
    Item r = b.send(Cbor().map(3).u(CBOR_MSG_REQ_ID).u(5)
                        .u(CBOR_DID_FLASH_CFG).map(2).u(CBOR_FLASH_CFG__SER1BAUDRATE).i(-9600)
                                                     .u(CBOR_FLASH_CFG__SER2BAUDRATE).u(115200)
                        .u(DID_CFG).map(1).u(0).u(60).bytes());
    const Item *fc = r.get(CBOR_DID_FLASH_CFG);
    ASSERT_NE(fc, nullptr);
    EXPECT_EQ(fc->get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_BAD_VALUE);
    ASSERT_EQ(fc->get(CBOR_MSG_ERR_KEYS)->arr.size(), 1u);
    EXPECT_EQ(fc->get(CBOR_MSG_ERR_KEYS)->arr[0].u, CBOR_FLASH_CFG__SER1BAUDRATE);
    EXPECT_EQ(fc->keys().size(), 2u) << "a failed DID carries no values";
    EXPECT_EQ(b.flash.ser2BaudRate, 0u) << "nothing in the failed DID is applied";
    EXPECT_EQ(b.cfg.rate, 60u) << "the other DID applies";
    EXPECT_EQ(r.get(DID_CFG), nullptr);
}

TEST(CborDevice, WriteRejectedByTheDeviceIsReported)
{
    Bench b;
    Item r = b.send(Cbor().map(1).u(CBOR_DID_FLASH_CFG).map(1).u(CBOR_FLASH_CFG__SER0BAUDRATE).u(0).bytes());
    EXPECT_EQ(r.get(CBOR_DID_FLASH_CFG)->get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_BAD_VALUE);
    EXPECT_EQ(b.flash.ser0BaudRate, 921600u);
}

TEST(CborDevice, ReadOnlyDidsRefuseData)
{
    Bench b;
    Item r = b.send(Cbor().map(2).u(DID_RO).map(1).u(0).u(1).u(CBOR_DID_CAPABILITIES).map(1).u(0).u(1).bytes());
    EXPECT_EQ(r.get(DID_RO)->get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_READ_ONLY);
    EXPECT_EQ(r.get(CBOR_DID_CAPABILITIES)->get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_READ_ONLY);
    EXPECT_EQ(b.ro.rate, 7u);
}

TEST(CborDevice, ArrayFormInADataMessageIsABadRequest)
{
    Bench b;
    Item r = b.send(Cbor().map(3).u(CBOR_MSG_TYPE).u(CBOR_MSG_TYPE_DATA).u(DID_CFG).arr(1).u(0)
                        .u(DID_RO).map(1).u(0).u(1).bytes());
    EXPECT_EQ(r.get(DID_CFG)->get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_BAD_REQUEST);
}

TEST(CborDevice, UnknownFieldsAreIgnored)
{
    Bench b;
    Item r = b.send(Cbor().map(1).u(DID_CFG).map(2).u(0x7123).u(1).u(0).u(30).bytes());
    EXPECT_EQ(b.cfg.rate, 30u);
    EXPECT_EQ(r.get(DID_CFG), nullptr);
}

/*-------------------------------------------------------------------------------------------- device: whole message */

TEST(CborDevice, SubscribeIsUnsupported)
{
    Bench b;
    Item r = b.send(Cbor().map(3).u(CBOR_MSG_TYPE).u(CBOR_MSG_TYPE_SUBSCRIBE).u(CBOR_SUB_RATE_PERIOD).u(200)
                        .u(DID_CFG).arr(0).bytes());
    EXPECT_EQ(r.get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_UNSUPPORTED);
    EXPECT_EQ(r.get(CBOR_MSG_TYPE)->u, cbor_msg_reply_type(CBOR_MSG_TYPE_SUBSCRIBE));
    EXPECT_EQ(r.get(DID_CFG), nullptr);
}

TEST(CborDevice, UnknownFlagsAreUnsupported)
{
    Bench b;
    Item r = b.send(Cbor().map(2).u(CBOR_MSG_TYPE).u(0x40 | CBOR_MSG_TYPE_DATA).u(DID_CFG).map(1).u(0).u(50).bytes());
    EXPECT_EQ(r.get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_UNSUPPORTED);
    EXPECT_EQ(b.cfg.rate, 10u);
}

TEST(CborDevice, RepliesAreNeverAppliedOrAnswered)
{
    Bench b;
    // a reply holding values of a DID this device owns, addressed to it
    Bytes reply = Cbor().map(4).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET))
                      .u(CBOR_MSG_SRC_DEV_ID).u(GPX_ID).u(CBOR_MSG_DST_DEV_FILTER).u(IMX_ID)
                      .u(DID_CFG).map(1).u(0).u(55).bytes();
    EXPECT_EQ(b.handle(reply), 0);
    EXPECT_EQ(b.cfg.rate, 10u);
    // not even a malformed-looking one is answered
    EXPECT_EQ(b.handle(Cbor().map(2).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_DATA))
                           .u(CBOR_MSG_REQ_ID).u(70000).bytes()), 0);
}

TEST(CborDevice, ExchangeBetweenTwoDevicesEnds)
{
    // Device A asks device B to apply a value; B's acknowledgement reaches A, which neither applies nor answers it.
    // Both own the same DID, so a reply mistaken for a write would change A's configuration.
    Bench a, b;
    b.dev.devId = GPX_ID;
    Bytes msg = Cbor().map(4).u(CBOR_MSG_SRC_DEV_ID).u(IMX_ID).u(CBOR_MSG_REQ_ID).u(8)
                    .u(CBOR_MSG_ACK_TYPE).u(CBOR_ACK_TYPE_ACK_DETAILED).u(DID_CFG).map(1).u(0).u(42).bytes();
    int n = b.handle(msg);
    ASSERT_GT(n, 0);
    EXPECT_EQ(b.cfg.rate, 42u);
    Item ack = parseReply(b.reply, n);
    EXPECT_EQ(ack.get(CBOR_MSG_DST_DEV_FILTER)->u, IMX_ID);
    EXPECT_EQ(ack.get(DID_CFG)->get(0)->u, 42u);

    EXPECT_EQ(a.handle(Bytes(b.reply, b.reply + n)), 0) << "the ack is not answered";
    EXPECT_EQ(a.cfg.rate, 10u) << "nor applied";
}

TEST(CborDevice, MalformedAndBadEnvelopesAreBadRequests)
{
    Bench b;
    Item r = b.send(Bytes{ 0xA2, 0x01 });
    EXPECT_EQ(r.get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_BAD_REQUEST);
    r = b.send(Cbor().map(2).u(CBOR_MSG_REQ_ID).u(70000).u(DID_CFG).map(1).u(0).u(50).bytes());
    EXPECT_EQ(r.get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_BAD_REQUEST);
    EXPECT_EQ(b.cfg.rate, 10u);
}

TEST(CborDevice, CapabilitiesListsOwnedDidsAndTheirFields)
{
    Bench b;
    Item r = b.send(Cbor().map(1).u(CBOR_DID_CAPABILITIES).arr(0).bytes());
    const Item *caps = r.get(CBOR_DID_CAPABILITIES);
    ASSERT_NE(caps, nullptr);
    EXPECT_EQ(caps->keys(), (std::vector<uint64_t>{ DID_CFG, DID_RO, CBOR_DID_FLASH_CFG, CBOR_DID_CAPABILITIES }));
    EXPECT_EQ(caps->get(DID_CFG)->arr.size(), 4u);
    EXPECT_EQ(caps->get(CBOR_DID_FLASH_CFG)->arr.size(), g_cborSchema_FLASH_CFG.count);
    EXPECT_EQ(caps->get(CBOR_DID_CAPABILITIES)->arr.size(), 4u);

    r = b.send(Cbor().map(1).u(CBOR_DID_CAPABILITIES).arr(1).u(DID_RO).bytes());
    EXPECT_EQ(r.get(CBOR_DID_CAPABILITIES)->keys(), (std::vector<uint64_t>{ DID_RO }));
}

/*-------------------------------------------------------------------------------------------- device: too large */

TEST(CborDevice, WhatDoesNotFitIsMarkedAndEveryDidIsAnswered)
{
    Bench b(200);   // room for the envelope, DID_CFG and part of FLASH_CFG
    Item r = b.send(Cbor().map(4).u(CBOR_MSG_REQ_ID).u(14).u(CBOR_DID_FLASH_CFG).arr(0).u(DID_CFG).arr(0)
                        .u(DID_RO).arr(0).bytes());
    const Item *fc = r.get(CBOR_DID_FLASH_CFG);
    ASSERT_NE(fc, nullptr);
    ASSERT_NE(r.get(DID_CFG), nullptr);
    ASSERT_NE(r.get(DID_RO), nullptr);
    // FLASH_CFG is either partly written with fields marked, or marked whole; nothing is ever cut
    bool marked = fc->get(CBOR_MSG_STATUS) && fc->get(CBOR_MSG_STATUS)->u == CBOR_MSG_STATUS_RESP_TOO_LARGE;
    for (auto &e : fc->map)
        if (e.second.type == NANOCBOR_TYPE_MAP)
            marked |= e.second.get(CBOR_MSG_STATUS)->u == CBOR_MSG_STATUS_RESP_TOO_LARGE;
    EXPECT_TRUE(marked);
}

TEST(CborDevice, TinyReplyBufferStillMarksEachDid)
{
    Bench b(56);
    Item r = b.send(Cbor().map(3).u(CBOR_DID_FLASH_CFG).arr(0).u(DID_CFG).arr(0).u(CBOR_DID_CAPABILITIES).arr(0)
                        .bytes());
    for (uint64_t did : { (uint64_t)CBOR_DID_FLASH_CFG, (uint64_t)DID_CFG, (uint64_t)CBOR_DID_CAPABILITIES })
    {
        SCOPED_TRACE(did);
        ASSERT_NE(r.get(did), nullptr);
        if (r.get(did)->get(CBOR_MSG_STATUS))
            EXPECT_EQ(r.get(did)->get(CBOR_MSG_STATUS)->u, CBOR_MSG_STATUS_RESP_TOO_LARGE);
    }
}

TEST(CborDevice, FlashConfigGetAndDataThroughTheGeneratedSchema)
{
    Bench b;
    Item r = b.send(Cbor().map(1).u(CBOR_DID_FLASH_CFG).map(1).u(CBOR_FLASH_CFG__SER0BAUDRATE).u(460800).bytes());
    EXPECT_EQ(b.flash.ser0BaudRate, 460800u);
    r = b.send(Cbor().map(1).u(CBOR_DID_FLASH_CFG).arr(1).u(CBOR_FLASH_CFG__SER0BAUDRATE).bytes());
    EXPECT_EQ(r.get(CBOR_DID_FLASH_CFG)->get(CBOR_FLASH_CFG__SER0BAUDRATE)->u, 460800u);
}
