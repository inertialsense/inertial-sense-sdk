/**
 * @file test_cbor_forward.cpp
 * @brief Forwarding: the downstream copy (MSG_ROUTE pushed, envelope ahead of DIDs, denied DIDs left out), popping
 *        MSG_ROUTE on the way back, FORWARD_DENIED replies, and a host -> IMX -> GPX -> host exchange on these pieces.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "cbor/cbor_msg.h"
#include "cbor/cbor_registry.h"
#include "cbor/dids/cbor_route_filter.h"

typedef std::vector<uint8_t> Bytes;

static const uint64_t IMX_ID = 0x0D400000'0000F54Dull;
static const uint64_t GPX_ID = 0x10400000'3756473Dull;

class Cbor
{
public:
    Cbor() : m_buf(1024) { nanocbor_encoder_init(&m_enc, m_buf.data(), m_buf.size()); }
    Cbor &map(size_t n)     { nanocbor_fmt_map(&m_enc, n); return *this; }
    Cbor &arr(size_t n)     { nanocbor_fmt_array(&m_enc, n); return *this; }
    Cbor &u(uint64_t v)     { nanocbor_fmt_uint(&m_enc, v); return *this; }
    Bytes bytes() const     { return Bytes(m_buf.data(), (const uint8_t *)m_enc.cur); }
private:
    Bytes m_buf;
    nanocbor_encoder_t m_enc;
};

static const cbor_route_rule_t s_rules[] = {
    { 0x1040FFFF'FFFFFFFFull, 1, { CBOR_MSG_TYPE_DATA }, 1, { CBOR_DID_GPX_FLASH_CFG }, CBOR_ROUTE_ACTION_DENY },
};

struct DenyCtx { uint8_t op; };

static bool denyFn(void *ctx, uint16_t did)
{
    return cbor_route_action(s_rules, 1, GPX_ID, ((DenyCtx *)ctx)->op, did) == CBOR_ROUTE_ACTION_DENY;
}

/** Runs cbor_msg_forward_build() with the test rules; returns the downstream copy (empty if none) */
static Bytes forward(const Bytes &msg, uint8_t port, cbor_forward_result_t *res, int *rc = nullptr)
{
    cbor_envelope_t env;
    cbor_msg_parse_envelope(msg.data(), msg.size(), &env);
    DenyCtx ctx = { cbor_msg_op(env.type) };
    uint8_t out[512];
    int n = cbor_msg_forward_build(msg.data(), msg.size(), port, denyFn, &ctx, out, sizeof(out), res);
    if (rc)
        *rc = n;
    return n > 0 ? Bytes(out, out + n) : Bytes();
}

TEST(CborForward, PushesTheArrivalPortAheadOfTheDids)
{
    // { MSG_DST_DEV_FILTER: any GPX, MSG_REQ_ID: 6, GPX_FLASH_CFG: [0] }
    Bytes msg = Cbor().map(3).u(CBOR_MSG_DST_DEV_FILTER).u(0x13FFFFFF'FFFFFFFFull).u(CBOR_MSG_REQ_ID).u(6)
                    .u(CBOR_DID_GPX_FLASH_CFG).arr(1).u(0).bytes();
    cbor_forward_result_t res;
    Bytes out = forward(msg, 3, &res);
    Bytes want = Cbor().map(4).u(CBOR_MSG_DST_DEV_FILTER).u(0x13FFFFFF'FFFFFFFFull).u(CBOR_MSG_REQ_ID).u(6)
                     .u(CBOR_MSG_ROUTE).arr(1).u(3).u(CBOR_DID_GPX_FLASH_CFG).arr(1).u(0).bytes();
    EXPECT_EQ(out, want);
    EXPECT_EQ(res.deniedCount, 0);

    // a second forwarder appends its port
    Bytes out2 = forward(out, 1, &res);
    cbor_envelope_t env;
    ASSERT_EQ(cbor_msg_parse_envelope(out2.data(), out2.size(), &env), CBOR_CODEC_OK);
    ASSERT_EQ(env.routeDepth, 2);
    EXPECT_EQ(env.route[0], 3);
    EXPECT_EQ(env.route[1], 1);
}

TEST(CborForward, DeniedDidsAreLeftOutAndReported)
{
    // DATA to GPX_FLASH_CFG (denied) and CAPABILITIES (allowed)
    Bytes msg = Cbor().map(3).u(CBOR_MSG_DST_DEV_FILTER).u(GPX_ID).u(CBOR_DID_GPX_FLASH_CFG).map(1).u(0).u(460800)
                    .u(CBOR_DID_CAPABILITIES).map(1).u(0).u(1).bytes();
    cbor_forward_result_t res;
    Bytes out = forward(msg, 3, &res);
    ASSERT_EQ(res.deniedCount, 1);
    EXPECT_EQ(res.denied[0], CBOR_DID_GPX_FLASH_CFG);
    EXPECT_EQ(res.forwardedDids, 1);
    cbor_envelope_t env;
    ASSERT_EQ(cbor_msg_parse_envelope(out.data(), out.size(), &env), CBOR_CODEC_OK);
    EXPECT_EQ(env.didCount, 1);

    // only denied DIDs: nothing to forward
    msg = Cbor().map(2).u(CBOR_MSG_DST_DEV_FILTER).u(GPX_ID).u(CBOR_DID_GPX_FLASH_CFG).map(1).u(0).u(460800).bytes();
    int rc;
    forward(msg, 3, &res, &rc);
    EXPECT_EQ(rc, 0);
    EXPECT_EQ(res.deniedCount, 1);
}

TEST(CborForward, RouteFullIsReported)
{
    Bytes msg = Cbor().map(2).u(CBOR_MSG_ROUTE).arr(4).u(1).u(2).u(3).u(4).u(CBOR_DID_CAPABILITIES).arr(0).bytes();
    int rc;
    cbor_forward_result_t res;
    forward(msg, 5, &res, &rc);
    EXPECT_EQ(rc, CBOR_MSG_ROUTE_FULL);
}

TEST(CborForward, RoutePopRemovesTheLastEntry)
{
    Bytes msg = Cbor().map(3).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET)).u(CBOR_MSG_ROUTE).arr(2).u(3).u(1)
                    .u(CBOR_DID_CAPABILITIES).map(0).bytes();
    size_t len = msg.size();
    uint8_t port = 0xFF;
    ASSERT_EQ(cbor_msg_route_pop(msg.data(), &len, &port), CBOR_CODEC_OK);
    EXPECT_EQ(port, 1);
    msg.resize(len);
    EXPECT_EQ(msg, Cbor().map(3).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET)).u(CBOR_MSG_ROUTE).arr(1).u(3)
                       .u(CBOR_DID_CAPABILITIES).map(0).bytes());

    // the last entry: MSG_ROUTE goes away and the map has one entry fewer
    ASSERT_EQ(cbor_msg_route_pop(msg.data(), &len, &port), CBOR_CODEC_OK);
    EXPECT_EQ(port, 3);
    msg.resize(len);
    EXPECT_EQ(msg, Cbor().map(2).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET)).u(CBOR_DID_CAPABILITIES)
                       .map(0).bytes());
    EXPECT_EQ(cbor_msg_route_pop(msg.data(), &len, &port), CBOR_CODEC_ERR_VALUE) << "nothing left to pop";
}

TEST(CborForward, RoutePopShrinksTheRootHeader)
{
    // 24 entries (2-byte map header) -> 23 (1 byte)
    Cbor c;
    c.map(24).u(CBOR_MSG_ROUTE).arr(1).u(2);
    for (int i = 0; i < 23; i++)
        c.u(0x0100 + i).map(0);
    Bytes msg = c.bytes();
    size_t len = msg.size();
    uint8_t port;
    ASSERT_EQ(cbor_msg_route_pop(msg.data(), &len, &port), CBOR_CODEC_OK);
    msg.resize(len);
    cbor_envelope_t env;
    ASSERT_EQ(cbor_msg_parse_envelope(msg.data(), msg.size(), &env), CBOR_CODEC_OK);
    EXPECT_EQ(env.didCount, 23);
    EXPECT_EQ(env.routeDepth, 0);
}

/*-------------------------------------------------------------------------------------------- end to end */

struct Gpx
{
    uint32_t baud = 921600;
    uint8_t reply[512];
    alignas(8) uint8_t scratch[64];
    cbor_did_t dids[1];
    cbor_device_t dev;
    static const cbor_field_t fields[1];
    static const cbor_schema_t schema;

    Gpx()
    {
        dids[0] = { CBOR_DID_GPX_FLASH_CFG, &schema, sizeof(uint32_t), readFn, writeFn, nullptr, nullptr, nullptr };
        dev = { GPX_ID, dids, 1, reply, sizeof(reply), scratch, sizeof(scratch), nullptr, this };
    }
    static const void *readFn(const cbor_did_t *, void *ctx)       { return &((Gpx *)ctx)->baud; }
    static uint8_t writeFn(const cbor_did_t *, void *ctx, const void *v)
    {
        memcpy(&((Gpx *)ctx)->baud, v, 4);
        return CBOR_MSG_STATUS_OK;
    }
};
const cbor_field_t Gpx::fields[1] = { { 0, CBOR_T_UINT, 4, 0, 0, nullptr } };
const cbor_schema_t Gpx::schema = { Gpx::fields, 1 };

TEST(CborForward, HostImxGpxHostExchange)
{
    Gpx gpx;
    // Host -> IMX port 3: GET GPX_FLASH_CFG from any GPX
    Bytes req = Cbor().map(3).u(CBOR_MSG_DST_DEV_FILTER).u(0x13FFFFFF'FFFFFFFFull).u(CBOR_MSG_REQ_ID).u(6)
                    .u(CBOR_DID_GPX_FLASH_CFG).arr(1).u(0).bytes();
    cbor_forward_result_t fres;
    Bytes fwd = forward(req, 3, &fres);
    ASSERT_FALSE(fwd.empty());

    // GPX answers; its reply is addressed to the host behind the IMX and echoes the route
    int n = cbor_device_handle(&gpx.dev, fwd.data(), fwd.size());
    ASSERT_GT(n, 0);
    Bytes rep(gpx.reply, gpx.reply + n);
    cbor_envelope_t env;
    ASSERT_EQ(cbor_msg_parse_envelope(rep.data(), rep.size(), &env), CBOR_CODEC_OK);
    EXPECT_TRUE(cbor_msg_is_reply(env.type));
    EXPECT_EQ(env.srcDevId, GPX_ID);
    EXPECT_TRUE(env.hasDst);
    EXPECT_EQ(env.dstFilter, 0u);
    EXPECT_EQ(env.reqId, 6);

    // IMX pops the route and sends the reply out of port 3, unchanged otherwise
    size_t len = rep.size();
    uint8_t port;
    ASSERT_EQ(cbor_msg_route_pop(rep.data(), &len, &port), CBOR_CODEC_OK);
    EXPECT_EQ(port, 3);
    rep.resize(len);
    ASSERT_EQ(cbor_msg_parse_envelope(rep.data(), rep.size(), &env), CBOR_CODEC_OK);
    EXPECT_EQ(env.routeDepth, 0);
    EXPECT_EQ(env.srcDevId, GPX_ID);
}

TEST(CborForward, DeniedReplyComesFromTheForwarder)
{
    uint8_t buf[128];
    cbor_device_t imx = { IMX_ID, nullptr, 0, buf, sizeof(buf), nullptr, 0, nullptr, nullptr };
    Bytes req = Cbor().map(3).u(CBOR_MSG_DST_DEV_FILTER).u(GPX_ID).u(CBOR_MSG_REQ_ID).u(7)
                    .u(CBOR_DID_GPX_FLASH_CFG).map(1).u(0).u(460800).bytes();
    cbor_envelope_t env;
    cbor_msg_parse_envelope(req.data(), req.size(), &env);
    uint16_t did = CBOR_DID_GPX_FLASH_CFG;
    int n = cbor_device_reply_status(&imx, &env, &did, 1, CBOR_MSG_STATUS_FORWARD_DENIED);
    ASSERT_GT(n, 0);
    Bytes want = Cbor().map(4).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_DATA)).u(CBOR_MSG_SRC_DEV_ID).u(IMX_ID)
                     .u(CBOR_MSG_REQ_ID).u(7).u(CBOR_DID_GPX_FLASH_CFG).map(1).u(CBOR_MSG_STATUS)
                     .u(CBOR_MSG_STATUS_FORWARD_DENIED).bytes();
    EXPECT_EQ(Bytes(buf, buf + n), want);

    env.ackType = CBOR_ACK_TYPE_ACK_NONE;
    EXPECT_EQ(cbor_device_reply_status(&imx, &env, &did, 1, CBOR_MSG_STATUS_FORWARD_DENIED), 0);
}
