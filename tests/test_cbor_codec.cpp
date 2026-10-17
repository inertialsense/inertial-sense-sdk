/**
 * @file test_cbor_codec.cpp
 * @brief The schema-driven CBOR codec: round trips, minimal encodings, selections, too-large marking within a byte
 *        budget, the decoding rules (exact encodings accepted, meaning-changing ones rejected), the array overlap
 *        rule, all-or-nothing map decoding with error key paths, and the generated legacy DID schemas.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <gtest/gtest.h>
#include <cstddef>
#include <cstring>
#include <vector>

#include "cbor/cbor_codec.h"
#include "cbor/cbor_registry.h"
#include "ISComm.h"

typedef std::vector<uint8_t> Bytes;

struct Inner
{
    int16_t a;
    float   b[2];
};

struct Sample
{
    uint8_t  u8;
    uint32_t u32;
    uint64_t u64;
    int8_t   i8;
    int32_t  i32;
    float    f;
    double   d;
    uint8_t  flag;
    char     name[8];
    uint8_t  blob[4];
    int16_t  arr[3];
    Inner    inner;
};

static const cbor_field_t s_innerFields[] = {
    { 0, CBOR_T_INT,   2, 0, offsetof(Inner, a), nullptr },
    { 1, CBOR_T_FLOAT, 4, 2, offsetof(Inner, b), nullptr },
};
static const cbor_schema_t s_inner = { s_innerFields, 2 };

static const cbor_field_t s_sampleFields[] = {
    {  0, CBOR_T_UINT,  1, 0, offsetof(Sample, u8),    nullptr },
    {  1, CBOR_T_UINT,  4, 0, offsetof(Sample, u32),   nullptr },
    {  2, CBOR_T_UINT,  8, 0, offsetof(Sample, u64),   nullptr },
    {  3, CBOR_T_INT,   1, 0, offsetof(Sample, i8),    nullptr },
    {  4, CBOR_T_INT,   4, 0, offsetof(Sample, i32),   nullptr },
    {  5, CBOR_T_FLOAT, 4, 0, offsetof(Sample, f),     nullptr },
    {  6, CBOR_T_FLOAT, 8, 0, offsetof(Sample, d),     nullptr },
    {  7, CBOR_T_BOOL,  1, 0, offsetof(Sample, flag),  nullptr },
    {  8, CBOR_T_TEXT,  1, 8, offsetof(Sample, name),  nullptr },
    {  9, CBOR_T_BYTES, 1, 4, offsetof(Sample, blob),  nullptr },
    { 10, CBOR_T_INT,   2, 3, offsetof(Sample, arr),   nullptr },
    { 11, CBOR_T_MAP,   0, 0, offsetof(Sample, inner), &s_inner },
};
static const cbor_schema_t s_sample = { s_sampleFields, sizeof(s_sampleFields) / sizeof(s_sampleFields[0]) };

/** A Sample with a distinct non-zero value in every field */
static Sample makeSample()
{
    Sample s = {};
    s.u8 = 200;
    s.u32 = 3000000000u;
    s.u64 = 0xFEDCBA9876543210ull;
    s.i8 = -100;
    s.i32 = -2000000000;
    s.f = 1.1f;
    s.d = 6378137.123456789;
    s.flag = 1;
    strcpy(s.name, "gpx-1");
    memcpy(s.blob, "\x01\x02\x03\x04", 4);
    s.arr[0] = -1; s.arr[1] = 1000; s.arr[2] = -32768;
    s.inner.a = -7;
    s.inner.b[0] = 0.5f; s.inner.b[1] = -2.25f;
    return s;
}

/** Expects every field of two Samples to be equal */
static void expectSampleEq(const Sample &x, const Sample &y)
{
    EXPECT_EQ(x.u8, y.u8);
    EXPECT_EQ(x.u32, y.u32);
    EXPECT_EQ(x.u64, y.u64);
    EXPECT_EQ(x.i8, y.i8);
    EXPECT_EQ(x.i32, y.i32);
    EXPECT_EQ(x.f, y.f);
    EXPECT_EQ(x.d, y.d);
    EXPECT_EQ(x.flag, y.flag);
    EXPECT_STREQ(x.name, y.name);
    EXPECT_EQ(0, memcmp(x.blob, y.blob, sizeof x.blob));
    EXPECT_EQ(0, memcmp(x.arr, y.arr, sizeof x.arr));
    EXPECT_EQ(x.inner.a, y.inner.a);
    EXPECT_EQ(x.inner.b[0], y.inner.b[0]);
    EXPECT_EQ(x.inner.b[1], y.inner.b[1]);
}

/** Encodes s through cbor_codec_encode_map() with a selection and budget; the result code goes to *rc */
static Bytes encode(const Sample &s, const std::vector<cbor_path_t> &paths, size_t bufSize, int *rc = nullptr,
                    uint16_t *tooLarge = nullptr)
{
    Bytes buf(bufSize);
    int n = cbor_codec_encode_map(&s_sample, &s, paths.empty() ? nullptr : paths.data(), paths.size(), buf.data(),
                                  buf.size(), tooLarge);
    if (rc)
        *rc = n;
    buf.resize(n > 0 ? (size_t)n : 0);
    return buf;
}

static int decode(const Bytes &in, Sample *s, cbor_decode_result_t *res)
{
    nanocbor_value_t it;
    nanocbor_decoder_init(&it, in.data(), in.size());
    return cbor_codec_decode_map(&it, &s_sample, s, res);
}

static cbor_path_t path(std::initializer_list<uint16_t> keys)
{
    cbor_path_t p = {};
    for (uint16_t k : keys)
        p.key[p.depth++] = k;
    return p;
}

/*-------------------------------------------------------------------------------------------- encoding */

TEST(CborCodec, RoundTripsEveryType)
{
    Sample in = makeSample(), out = {};
    int rc;
    Bytes b = encode(in, {}, 256, &rc);
    ASSERT_GT(rc, 0);
    cbor_decode_result_t res;
    ASSERT_EQ(decode(b, &out, &res), CBOR_CODEC_OK);
    EXPECT_EQ(res.errCount, 0);
    EXPECT_EQ(res.appliedCount, 12);
    expectSampleEq(in, out);
}

TEST(CborCodec, EncodesValuesInTheirShortestExactForm)
{
    Sample s = makeSample();
    s.u32 = 5;
    s.f = 1.5f;     // exact as a half float
    EXPECT_EQ(encode(s, { path({ 1 }) }, 32), (Bytes{ 0xA1, 0x01, 0x05 }));
    EXPECT_EQ(encode(s, { path({ 5 }) }, 32), (Bytes{ 0xA1, 0x05, 0xF9, 0x3E, 0x00 }));
    EXPECT_EQ(encode(s, { path({ 8 }) }, 32), (Bytes{ 0xA1, 0x08, 0x65, 'g', 'p', 'x', '-', '1' }));
    EXPECT_EQ(encode(s, { path({ 7 }) }, 32), (Bytes{ 0xA1, 0x07, 0xF5 }));
}

TEST(CborCodec, SelectionWritesOnlyTheSelectedFields)
{
    Sample s = makeSample();
    s.u32 = 5;
    s.inner.a = 3;
    // field 1, and only field 0 of the nested map
    EXPECT_EQ(encode(s, { path({ 1 }), path({ 11, 0 }) }, 64), (Bytes{ 0xA2, 0x01, 0x05, 0x0B, 0xA1, 0x00, 0x03 }));
    // unknown keys in a selection are ignored
    EXPECT_EQ(encode(s, { path({ 1 }), path({ 99 }) }, 64), (Bytes{ 0xA1, 0x01, 0x05 }));
}

TEST(CborCodec, MarksAFieldThatDoesNotFitAndNeverSplitsIt)
{
    Sample s = makeSample();
    s.u8 = 5;
    s.arr[0] = s.arr[1] = s.arr[2] = 1000;      // 11 bytes with its key
    uint16_t tooLarge = 0;
    int rc;
    // 3 reserved header bytes + 2 for field 0 + 6 for the marker: field 10 cannot fit, its marker can
    Bytes b = encode(s, { path({ 0 }), path({ 10 }) }, 11, &rc, &tooLarge);
    EXPECT_EQ(b, (Bytes{ 0xA2, 0x00, 0x05, 0x0A, 0xA1, 0x19, 0xC0, 0x08, CBOR_MSG_STATUS_RESP_TOO_LARGE }));
    EXPECT_EQ(tooLarge, 1);
    static_assert(CBOR_MSG_STATUS == 0xC008, "marker bytes above assume MSG_STATUS 0xC008");
}

TEST(CborCodec, MarksInsideANestedMapThatDoesNotFitWhole)
{
    Sample s = makeSample();
    s.inner.a = 1;
    s.inner.b[0] = s.inner.b[1] = 1.1f;     // single precision: 12 bytes with its key
    uint16_t tooLarge = 0;
    Bytes b = encode(s, { path({ 11 }) }, 15, nullptr, &tooLarge);
    EXPECT_EQ(b, (Bytes{ 0xA1, 0x0B, 0xA2, 0x00, 0x01, 0x01, 0xA1, 0x19, 0xC0, 0x08, CBOR_MSG_STATUS_RESP_TOO_LARGE }));
    EXPECT_EQ(tooLarge, 1);
}

TEST(CborCodec, ReportsNoSpaceWhenNotEvenTheHeaderFits)
{
    int rc;
    encode(makeSample(), {}, 2, &rc);
    EXPECT_EQ(rc, CBOR_CODEC_ERR_NO_SPACE);
}

TEST(CborCodec, EncodeValueOfAMapFailsRatherThanMarking)
{
    Sample s = makeSample();
    uint8_t buf[6];
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, sizeof buf);
    EXPECT_LT(cbor_codec_encode_value(&enc, &s_sampleFields[11], &s.inner), 0);
}

/*-------------------------------------------------------------------------------------------- decoding */

TEST(CborCodec, IgnoresUnknownAndNonIntegerKeys)
{
    Sample s = {};
    cbor_decode_result_t res;
    // {0: 7, 99: "x", "k": 1, 0xF000: [1]}
    Bytes in = { 0xA4, 0x00, 0x07, 0x18, 0x63, 0x61, 'x', 0x61, 'k', 0x01, 0x19, 0xF0, 0x00, 0x81, 0x01 };
    ASSERT_EQ(decode(in, &s, &res), CBOR_CODEC_OK);
    EXPECT_EQ(s.u8, 7);
    EXPECT_EQ(res.appliedCount, 1);
}

TEST(CborCodec, AcceptsExactEncodings)
{
    Sample s = {};
    cbor_decode_result_t res;
    Bytes in = {
        0xA6,
        0x05, 0x03,                                                 // integer 3 into a float
        0x06, 0xF9, 0x3E, 0x00,                                     // half 1.5 into a double
        0x01, 0x1B, 0, 0, 0, 0, 0, 0, 0, 0x2A,                      // 42 in a non-minimal 8-byte integer
        0x07, 0x01,                                                 // bool as 1
        0x03, 0x38, 0x7F,                                           // int8 -128
        0x0B, 0xA1, 0x01, 0x82, 0xFB, 0x3F, 0xF8, 0, 0, 0, 0, 0, 0, 0x00,   // inner.b = [double 1.5, 0]
    };
    ASSERT_EQ(decode(in, &s, &res), CBOR_CODEC_OK) << int(res.errCount);
    EXPECT_EQ(s.f, 3.0f);
    EXPECT_EQ(s.d, 1.5);
    EXPECT_EQ(s.u32, 42u);
    EXPECT_EQ(s.flag, 1);
    EXPECT_EQ(s.i8, -128);
    EXPECT_EQ(s.inner.b[0], 1.5f);
    EXPECT_EQ(s.inner.b[1], 0.0f);
}

struct RejectCase
{
    const char *what;
    Bytes       entry;      // one key/value pair
};

TEST(CborCodec, RejectsValuesThatChangeTheirMeaning)
{
    const RejectCase cases[] = {
        { "uint8 256",                  { 0x00, 0x19, 0x01, 0x00 } },
        { "negative into uint",         { 0x01, 0x20 } },
        { "int8 -129",                  { 0x03, 0x38, 0x80 } },
        { "inexact integer into float", { 0x05, 0x1A, 0x01, 0x00, 0x00, 0x01 } },
        { "double beyond float range",  { 0x05, 0xFB, 0x7E, 0x37, 0xE4, 0x3C, 0x88, 0x00, 0x75, 0x9C } },
        { "bool 2",                     { 0x07, 0x02 } },
        { "text longer than its field", { 0x08, 0x68, 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h' } },
        { "bytes longer than its field",{ 0x09, 0x45, 1, 2, 3, 4, 5 } },
        { "text into bytes",            { 0x09, 0x61, 'a' } },
        { "tagged value",               { 0x00, 0xC1, 0x05 } },
        { "null",                       { 0x00, 0xF6 } },
        { "indefinite array",           { 0x0A, 0x9F, 0x01, 0xFF } },
        { "scalar into an array",       { 0x0A, 0x01 } },
        { "array into a scalar",        { 0x00, 0x81, 0x01 } },
        { "bad array element",          { 0x0A, 0x82, 0x01, 0x61, 'x' } },
    };
    for (const RejectCase &c : cases)
    {
        SCOPED_TRACE(c.what);
        Sample s = {};
        s.u32 = 77;
        Bytes in = { 0xA2, 0x01, 0x05 };    // a valid field that must not be applied
        in.insert(in.end(), c.entry.begin(), c.entry.end());
        cbor_decode_result_t res;
        nanocbor_value_t it;
        nanocbor_decoder_init(&it, in.data(), in.size());
        EXPECT_EQ(cbor_codec_decode_map(&it, &s_sample, &s, &res), CBOR_CODEC_ERR_VALUE);
        EXPECT_EQ(s.u32, 77u) << "nothing is written when any value is invalid";
        ASSERT_EQ(res.errCount, 1);
        EXPECT_EQ(res.err[0].depth, 1);
        EXPECT_EQ(res.err[0].key[0], c.entry[0]);
        EXPECT_TRUE(nanocbor_at_end(&it)) << "the decoder moves past the rejected map";
    }
}

TEST(CborCodec, ReportsNestedErrorsWithTheirFullPath)
{
    Sample s = {};
    cbor_decode_result_t res;
    // {11: {0: "x"}, 3: 1000}
    Bytes in = { 0xA2, 0x0B, 0xA1, 0x00, 0x61, 'x', 0x03, 0x19, 0x03, 0xE8 };
    ASSERT_EQ(decode(in, &s, &res), CBOR_CODEC_ERR_VALUE);
    ASSERT_EQ(res.errCount, 2);
    EXPECT_EQ(res.err[0].depth, 2);
    EXPECT_EQ(res.err[0].key[0], 11);
    EXPECT_EQ(res.err[0].key[1], 0);
    EXPECT_EQ(res.err[1].depth, 1);
    EXPECT_EQ(res.err[1].key[0], 3);
}

TEST(CborCodec, ArraysApplyTheOverlappingElements)
{
    Sample s = makeSample();
    cbor_decode_result_t res;
    ASSERT_EQ(decode({ 0xA1, 0x0A, 0x81, 0x07 }, &s, &res), CBOR_CODEC_OK);    // shorter: only arr[0] changes
    EXPECT_EQ(s.arr[0], 7);
    EXPECT_EQ(s.arr[1], 1000);
    EXPECT_EQ(s.arr[2], -32768);
    ASSERT_EQ(decode({ 0xA1, 0x0A, 0x85, 1, 2, 3, 4, 5 }, &s, &res), CBOR_CODEC_OK);   // longer: extras ignored
    EXPECT_EQ(s.arr[0], 1);
    EXPECT_EQ(s.arr[1], 2);
    EXPECT_EQ(s.arr[2], 3);
}

TEST(CborCodec, ShortTextAndBytesAreZeroFilled)
{
    Sample s = makeSample();
    cbor_decode_result_t res;
    ASSERT_EQ(decode({ 0xA2, 0x08, 0x61, 'z', 0x09, 0x41, 0x09 }, &s, &res), CBOR_CODEC_OK);
    EXPECT_STREQ(s.name, "z");
    EXPECT_EQ(0, memcmp(s.name, "z\0\0\0\0\0\0\0", 8));
    EXPECT_EQ(0, memcmp(s.blob, "\x09\0\0\0", 4));
}

TEST(CborCodec, RejectsMalformedInput)
{
    Sample s = {};
    cbor_decode_result_t res;
    EXPECT_EQ(decode({ 0xA2, 0x00, 0x05, 0x01 }, &s, &res), CBOR_CODEC_ERR_MALFORMED);     // truncated
    EXPECT_EQ(decode({ 0x81, 0x00 }, &s, &res), CBOR_CODEC_ERR_MALFORMED);                 // not a map
    EXPECT_EQ(decode({ 0xBF, 0x00, 0x05, 0xFF }, &s, &res), CBOR_CODEC_ERR_MALFORMED);     // indefinite map
    EXPECT_EQ(s.u8, 0);
}

TEST(CborCodec, ValidateOnlyWritesNothing)
{
    cbor_decode_result_t res;
    Bytes in = { 0xA1, 0x00, 0x05 };
    nanocbor_value_t it;
    nanocbor_decoder_init(&it, in.data(), in.size());
    EXPECT_EQ(cbor_codec_decode_map(&it, &s_sample, nullptr, &res), CBOR_CODEC_OK);
    EXPECT_EQ(res.appliedCount, 1);
}

/*-------------------------------------------------------------------------------------------- key paths */

TEST(CborCodec, ParsesAndEncodesKeyPaths)
{
    Bytes in = { 0x83, 0x01, 0x82, 0x0B, 0x00, 0x19, 0xC0, 0x08 };    // [1, [11, 0], 0xC008]
    nanocbor_value_t it;
    nanocbor_decoder_init(&it, in.data(), in.size());
    cbor_path_t p[4];
    ASSERT_EQ(cbor_codec_decode_paths(&it, p, 4), 3);
    EXPECT_EQ(p[0].depth, 1);
    EXPECT_EQ(p[0].key[0], 1);
    EXPECT_EQ(p[1].depth, 2);
    EXPECT_EQ(p[1].key[1], 0);
    EXPECT_EQ(p[2].key[0], 0xC008);

    uint8_t buf[16];
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, sizeof buf);
    ASSERT_GE(cbor_codec_encode_path(&enc, &p[0]), 0);
    ASSERT_GE(cbor_codec_encode_path(&enc, &p[1]), 0);
    EXPECT_EQ(Bytes(buf, enc.cur), (Bytes{ 0x01, 0x82, 0x0B, 0x00 }));
}

TEST(CborCodec, RejectsBadKeyPaths)
{
    auto parse = [](const Bytes &in, size_t max) {
        nanocbor_value_t it;
        nanocbor_decoder_init(&it, in.data(), in.size());
        cbor_path_t p[2];
        return cbor_codec_decode_paths(&it, p, max);
    };
    EXPECT_EQ(parse({ 0x80 }, 2), 0);                                                   // empty = every field
    EXPECT_EQ(parse({ 0x01 }, 2), CBOR_CODEC_ERR_MALFORMED);                            // not an array
    EXPECT_EQ(parse({ 0x83, 0x01, 0x02, 0x03 }, 2), CBOR_CODEC_ERR_TOO_MANY);
    EXPECT_EQ(parse({ 0x81, 0x1A, 0x00, 0x01, 0x00, 0x00 }, 2), CBOR_CODEC_ERR_VALUE);  // key above uint16
    EXPECT_EQ(parse({ 0x81, 0x85, 1, 2, 3, 4, 5 }, 2), CBOR_CODEC_ERR_VALUE);           // deeper than the maximum
    EXPECT_EQ(parse({ 0x81, 0x80 }, 2), CBOR_CODEC_ERR_VALUE);                          // empty path
}


TEST(CborCodec, FlashConfigRoundTripsThroughItsSchema)
{
    nvm_flash_cfg_t in = {}, out = {};
    in.ser0BaudRate = 921600;
    in.insRotation[0] = 0.25f;
    in.insRotation[2] = -1.5f;
    in.refLla[0] = 40.2518;
    in.refLla[1] = -111.6493;
    in.ioConfig = 0x12345678;

    uint8_t buf[ISB_MAX_DATASET_SIZE];
    uint16_t tooLarge = 0;
    int n = cbor_codec_encode_map(&g_cborSchema_FLASH_CFG, &in, nullptr, 0, buf, sizeof buf, &tooLarge);
    ASSERT_GT(n, 0);
    EXPECT_EQ(tooLarge, 0);

    nanocbor_value_t it;
    nanocbor_decoder_init(&it, buf, (size_t)n);
    cbor_decode_result_t res;
    ASSERT_EQ(cbor_codec_decode_map(&it, &g_cborSchema_FLASH_CFG, &out, &res), CBOR_CODEC_OK);
    EXPECT_EQ(res.appliedCount, g_cborSchema_FLASH_CFG.count);
    for (uint16_t i = 0; i < g_cborSchema_FLASH_CFG.count; i++)
    {
        const cbor_field_t &f = g_cborSchema_FLASH_CFG.fields[i];
        size_t len = (size_t)f.size * (f.count ? f.count : 1);
        EXPECT_EQ(0, memcmp((uint8_t *)&in + f.offset, (uint8_t *)&out + f.offset, len)) << "field " << f.key;
    }
}
