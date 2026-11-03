/**
 * @file test_cbor_envelope.cpp
 * @brief The CBOR message envelope: parsing and defaults, round trips, unknown keys and bad values, and device-filter
 *        matching.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <gtest/gtest.h>
#include <cstddef>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "cbor/cbor_envelope.h"
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

static const uint64_t IMX_ID = 0x0D400000'0000F54Dull;
static const uint64_t GPX_ID = 0x10400000'3756473Dull;

static const uint16_t DID_CFG = 0x0100;     // a DID key for the envelope tests


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

TEST(CborMsg, NextKeySkipsKeysThatAreNotUint16)
{
    Bytes msg = Cbor().map(3).t("name").u(1).u(0x10000).u(2).u(7).u(3).bytes();
    nanocbor_value_t it, map;
    nanocbor_decoder_init(&it, msg.data(), msg.size());
    ASSERT_GE(nanocbor_enter_map(&it, &map), 0);
    uint16_t key = 0;
    EXPECT_EQ(cbor_msg_next_key(&map, &key), 1) << "a text key is skipped with its value";
    EXPECT_EQ(cbor_msg_next_key(&map, &key), 1) << "a key above 0xFFFF is skipped with its value";
    ASSERT_EQ(cbor_msg_next_key(&map, &key), CBOR_CODEC_OK);
    EXPECT_EQ(key, 7);
}

TEST(CborMsg, DecodeRouteAcceptsUpToTheMaximumDepth)
{
    cbor_envelope_t env = {};
    Bytes ok = Cbor().arr(2).u(3).u(1).bytes();
    nanocbor_value_t it;
    nanocbor_decoder_init(&it, ok.data(), ok.size());
    ASSERT_EQ(cbor_msg_decode_route(&it, &env), CBOR_CODEC_OK);
    ASSERT_EQ(env.routeDepth, 2);
    EXPECT_EQ(env.route[0], 3);
    EXPECT_EQ(env.route[1], 1);

    Cbor deep;
    deep.arr(CBOR_ROUTE_MAX + 1);
    for (int i = 0; i <= CBOR_ROUTE_MAX; i++)
        deep.u(i);
    Bytes tooDeep = deep.bytes();
    nanocbor_decoder_init(&it, tooDeep.data(), tooDeep.size());
    EXPECT_EQ(cbor_msg_decode_route(&it, &env), CBOR_CODEC_ERR_VALUE);
    EXPECT_EQ(env.routeDepth, 0) << "an over-long route is rejected whole";

    Bytes badPort = Cbor().arr(1).u(300).bytes();
    nanocbor_decoder_init(&it, badPort.data(), badPort.size());
    EXPECT_EQ(cbor_msg_decode_route(&it, &env), CBOR_CODEC_ERR_VALUE);
    EXPECT_EQ(env.routeDepth, 0);
}
