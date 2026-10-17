/**
 * @file test_cbor_route.cpp
 * @brief ROUTE_FILTER: first-match rule evaluation with device-filter, operation and DID matching (default forward),
 *        its CBOR encoding and selection, and a device serving it read-only through the message layer's custom hooks.
 */

#include <gtest/gtest.h>
#include <vector>

#include "cbor/cbor_msg.h"
#include "cbor/cbor_registry.h"
#include "cbor/cbor_route.h"

static const uint64_t IMX_ID = 0x0D400000'0000F54Dull;
static const uint64_t GPX_ID = 0x10400000'3756473Dull;
static const uint64_t GPX_1_0_ANY = 0x1040FFFF'FFFFFFFFull;

/** The IMX built-in default: deny DATA to GPX_FLASH_CFG on GPX-1.0 */
static const cbor_route_rule_t s_default[] = {
    { GPX_1_0_ANY, 1, { CBOR_MSG_TYPE_DATA }, 1, { CBOR_DID_GPX_FLASH_CFG }, CBOR_ROUTE_ACTION_DENY },
};

TEST(CborRoute, DefaultRuleDeniesOnlyGpxFlashConfigWrites)
{
    EXPECT_EQ(cbor_route_action(s_default, 1, GPX_ID, CBOR_MSG_TYPE_DATA, CBOR_DID_GPX_FLASH_CFG), CBOR_ROUTE_ACTION_DENY);
    EXPECT_EQ(cbor_route_action(s_default, 1, GPX_ID, CBOR_MSG_TYPE_GET, CBOR_DID_GPX_FLASH_CFG), CBOR_ROUTE_ACTION_ALLOW)
        << "reads pass";
    EXPECT_EQ(cbor_route_action(s_default, 1, GPX_ID, CBOR_MSG_TYPE_DATA, CBOR_DID_CAPABILITIES), CBOR_ROUTE_ACTION_ALLOW);
    EXPECT_EQ(cbor_route_action(s_default, 1, 0x10800000'00000001ull, CBOR_MSG_TYPE_DATA, CBOR_DID_GPX_FLASH_CFG),
              CBOR_ROUTE_ACTION_ALLOW) << "another GPX model is not matched";
    EXPECT_EQ(cbor_route_action(nullptr, 0, GPX_ID, CBOR_MSG_TYPE_DATA, CBOR_DID_GPX_FLASH_CFG), CBOR_ROUTE_ACTION_ALLOW)
        << "no rules: forward";
}

TEST(CborRoute, FirstMatchWinsAndEmptyListsMatchEverything)
{
    const cbor_route_rule_t rules[] = {
        { GPX_ID,     0, {}, 1, { CBOR_DID_GPX_FLASH_CFG }, CBOR_ROUTE_ACTION_ALLOW },   // this one GPX may be written
        { UINT64_MAX, 0, {}, 0, {},                         CBOR_ROUTE_ACTION_DENY },    // nothing else passes
    };
    EXPECT_EQ(cbor_route_action(rules, 2, GPX_ID, CBOR_MSG_TYPE_DATA, CBOR_DID_GPX_FLASH_CFG), CBOR_ROUTE_ACTION_ALLOW);
    EXPECT_EQ(cbor_route_action(rules, 2, GPX_ID, CBOR_MSG_TYPE_GET, CBOR_DID_CAPABILITIES), CBOR_ROUTE_ACTION_DENY);
    EXPECT_EQ(cbor_route_action(rules, 2, IMX_ID, CBOR_MSG_TYPE_GET, CBOR_DID_FLASH_CFG), CBOR_ROUTE_ACTION_DENY);
}

static std::vector<uint8_t> encode(const std::vector<cbor_path_t> &paths, size_t size = 128)
{
    std::vector<uint8_t> buf(size);
    int n = cbor_route_encode(s_default, 1, paths.empty() ? nullptr : paths.data(), paths.size(), buf.data(), buf.size());
    buf.resize(n > 0 ? (size_t)n : 0);
    return buf;
}

TEST(CborRoute, EncodesRulesByIndex)
{
    // {0: {0: 0x1040FFFFFFFFFFFF, 1: [DATA], 2: [GPX_FLASH_CFG (121)], 3: DENY}}
    std::vector<uint8_t> want = { 0xA1, 0x00, 0xA4, 0x00, 0x1B, 0x10, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                  0x01, 0x81, 0x01, 0x02, 0x81, 0x18, 0x79, 0x03, 0x00 };
    EXPECT_EQ(encode({}), want);
}

TEST(CborRoute, SelectionPicksRulesAndFields)
{
    cbor_path_t action = { 2, { 0, CBOR_ROUTE_FILTER__ACTION } };
    EXPECT_EQ(encode({ action }), (std::vector<uint8_t>{ 0xA1, 0x00, 0xA1, 0x03, 0x00 }));
    cbor_path_t other = { 1, { 5 } };
    EXPECT_EQ(encode({ other }), (std::vector<uint8_t>{ 0xA0 })) << "a rule that does not exist is left out";
}

TEST(CborRoute, RuleThatDoesNotFitIsMarked)
{
    std::vector<uint8_t> b = encode({}, 12);
    EXPECT_EQ(b, (std::vector<uint8_t>{ 0xA1, 0x00, 0xA1, 0x19, 0xC0, 0x08, CBOR_MSG_STATUS_RESP_TOO_LARGE }));
}

/*-------------------------------------------------------------------------------------------- served by a device */

static int routeEncode(const cbor_did_t *, void *, const cbor_path_t *paths, size_t n, uint8_t *buf, size_t size)
{
    return cbor_route_encode(s_default, 1, paths, n, buf, size);
}

static int routeKeys(const cbor_did_t *, void *, nanocbor_encoder_t *enc)
{
    return cbor_route_encode_keys(1, enc);
}

struct RouteDevice
{
    uint8_t reply[512];
    alignas(8) uint8_t scratch[256];
    cbor_did_t dids[1] = { { CBOR_DID_ROUTE_FILTER, nullptr, 0, nullptr, nullptr, routeEncode, nullptr, routeKeys } };
    cbor_device_t dev = { IMX_ID, dids, 1, reply, sizeof(reply), scratch, sizeof(scratch), nullptr, nullptr };

    std::vector<uint8_t> send(const std::vector<uint8_t> &msg)
    {
        int n = cbor_device_handle(&dev, msg.data(), msg.size());
        return n > 0 ? std::vector<uint8_t>(reply, reply + n) : std::vector<uint8_t>();
    }
};

/** Returns the value bytes of root key `key` in a reply map */
static std::vector<uint8_t> rootValue(const std::vector<uint8_t> &msg, uint16_t key)
{
    nanocbor_value_t it, map;
    nanocbor_decoder_init(&it, msg.data(), msg.size());
    if (nanocbor_enter_map(&it, &map) < 0)
        return {};
    while (!nanocbor_at_end(&map))
    {
        uint32_t k;
        nanocbor_get_uint32(&map, &k);
        const uint8_t *start = map.cur;
        nanocbor_skip(&map);
        if (k == key)
            return std::vector<uint8_t>(start, map.cur);
    }
    return {};
}

TEST(CborRoute, DeviceServesRouteFilterReadOnly)
{
    RouteDevice d;
    // GET: { ROUTE_FILTER: [[0, ACTION]] }
    std::vector<uint8_t> get = { 0xA1, 0x19, 0x7F, 0xFF, 0x81, 0x82, 0x00, 0x03 };
    EXPECT_EQ(rootValue(d.send(get), CBOR_DID_ROUTE_FILTER), (std::vector<uint8_t>{ 0xA1, 0x00, 0xA1, 0x03, 0x00 }));

    // DATA: { ROUTE_FILTER: {0: {3: ALLOW}} } -> READ_ONLY
    std::vector<uint8_t> data = { 0xA1, 0x19, 0x7F, 0xFF, 0xA1, 0x00, 0xA1, 0x03, 0x01 };
    EXPECT_EQ(rootValue(d.send(data), CBOR_DID_ROUTE_FILTER),
              (std::vector<uint8_t>{ 0xA1, 0x19, 0xC0, 0x08, CBOR_MSG_STATUS_READ_ONLY }));

    // CAPABILITIES lists the rule indices
    std::vector<uint8_t> caps = { 0xA1, 0x19, 0x7F, 0xFC, 0x81, 0x19, 0x7F, 0xFF };
    EXPECT_EQ(rootValue(d.send(caps), CBOR_DID_CAPABILITIES), (std::vector<uint8_t>{ 0xA1, 0x19, 0x7F, 0xFF, 0x81, 0x00 }));
}
