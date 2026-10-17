/**
 * @file test_cbor_store.cpp
 * @brief Stored configuration documents: deltas against defaults keyed by DID, lookup, and loading onto defaults.
 */

#include <gtest/gtest.h>
#include <cstddef>
#include <cstring>
#include <vector>

#include "cbor/cbor_store.h"

struct Cfg
{
    uint32_t a;
    float    b[2];
    uint8_t  c;
};

static const cbor_field_t s_fields[] = {
    { 0, CBOR_T_UINT,  4, 0, offsetof(Cfg, a), nullptr },
    { 1, CBOR_T_FLOAT, 4, 2, offsetof(Cfg, b), nullptr },
    { 2, CBOR_T_UINT,  1, 0, offsetof(Cfg, c), nullptr },
};
static const cbor_schema_t s_schema = { s_fields, 3 };
static const Cfg s_defaults = { 10, { 1.0f, 2.0f }, 3 };

static std::vector<uint8_t> store(const Cfg &c, uint16_t did = 0x0100)
{
    std::vector<uint8_t> buf(128);
    cbor_map_writer_t w;
    EXPECT_TRUE(cbor_store_begin(&w, buf.data(), buf.size()));
    EXPECT_EQ(cbor_store_add_delta(&w, did, &s_schema, &c, &s_defaults), 0);
    buf.resize(cbor_store_end(&w));
    return buf;
}

TEST(CborStore, StoresOnlyFieldsThatDifferFromDefaults)
{
    Cfg c = s_defaults;
    EXPECT_EQ(store(c), (std::vector<uint8_t>{ 0xA0 })) << "nothing differs: the DID is left out";
    c.c = 9;
    EXPECT_EQ(store(c), (std::vector<uint8_t>{ 0xA1, 0x19, 0x01, 0x00, 0xA1, 0x02, 0x09 }));
    c.b[1] = 2.5f;      // an array is stored whole
    EXPECT_EQ(store(c), (std::vector<uint8_t>{ 0xA1, 0x19, 0x01, 0x00, 0xA2, 0x01, 0x82, 0xF9, 0x3C, 0x00, 0xF9, 0x41,
                                               0x00, 0x02, 0x09 }));
}

TEST(CborStore, LoadsOntoDefaults)
{
    Cfg c = s_defaults;
    c.a = 77;
    std::vector<uint8_t> doc = store(c);
    Cfg newer = s_defaults;
    newer.c = 4;
    Cfg out;
    ASSERT_EQ(cbor_store_load_delta(doc.data(), doc.size(), 0x0100, &s_schema, &out, &newer, sizeof(out)), CBOR_CODEC_OK);
    EXPECT_EQ(out.a, 77u);
    EXPECT_EQ(out.c, 4) << "a field never set follows the current default";

    ASSERT_EQ(cbor_store_load_delta(doc.data(), doc.size(), 0x0101, &s_schema, &out, &s_defaults, sizeof(out)),
              CBOR_CODEC_OK) << "a DID the document does not hold loads as its defaults";
    EXPECT_EQ(out.a, 10u);
}

TEST(CborStore, InvalidStoredValuesLoadDefaults)
{
    std::vector<uint8_t> doc = { 0xA1, 0x19, 0x01, 0x00, 0xA2, 0x00, 0x05, 0x02, 0x19, 0x01, 0x00 };   // c = 256
    Cfg out;
    EXPECT_EQ(cbor_store_load_delta(doc.data(), doc.size(), 0x0100, &s_schema, &out, &s_defaults, sizeof(out)),
              CBOR_CODEC_ERR_VALUE);
    EXPECT_EQ(out.a, 10u);
}

TEST(CborStore, DoesNotFitIsAnError)
{
    Cfg c = s_defaults;
    c.a = 0xFFFFFFFF;
    uint8_t buf[8];
    cbor_map_writer_t w;
    ASSERT_TRUE(cbor_store_begin(&w, buf, sizeof(buf)));
    EXPECT_EQ(cbor_store_add_delta(&w, 0x0100, &s_schema, &c, &s_defaults), CBOR_CODEC_ERR_NO_SPACE);
}
