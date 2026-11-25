/**
 * @file test_cbor_format.cpp
 * @brief CBOR messages as text: requests from the -get / -set YAML form (domain names, numbers, DID names inside
 *        subscriptions, envelope keys, enums, element writes), replies as YAML, and diagnostic notation with and
 *        without registry names.
 *
 * @author Kyle Mallory on 2026-10-23
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>
#include <cstring>
#include <string>
#include <vector>

#include "ISCborFormat.h"
#include "ISComm.h"

typedef std::vector<uint8_t> Bytes;

/** Builds CBOR with nanocbor */
class Cbor
{
public:
    Cbor() : m_buf(1024) { nanocbor_encoder_init(&m_enc, m_buf.data(), m_buf.size()); }
    Cbor &map(size_t n)         { nanocbor_fmt_map(&m_enc, n); return *this; }
    Cbor &arr(size_t n)         { nanocbor_fmt_array(&m_enc, n); return *this; }
    Cbor &u(uint64_t v)         { nanocbor_fmt_uint(&m_enc, v); return *this; }
    Cbor &i(int64_t v)          { nanocbor_fmt_int(&m_enc, v); return *this; }
    Cbor &f(float v)            { nanocbor_fmt_float(&m_enc, v); return *this; }
    Cbor &t(const char *s)      { nanocbor_put_tstr(&m_enc, s); return *this; }
    Cbor &b(bool v)             { nanocbor_fmt_bool(&m_enc, v); return *this; }
    Cbor &tag(uint64_t v)       { nanocbor_fmt_tag(&m_enc, v); return *this; }
    Cbor &bytes(const Bytes &v) { nanocbor_put_bstr(&m_enc, v.data(), v.size()); return *this; }
    Bytes out() const           { return Bytes(m_buf.data(), (const uint8_t *)m_enc.cur); }
private:
    Bytes m_buf;
    nanocbor_encoder_t m_enc;
};

/** Builds a request from YAML text; fails the test if it does not convert */
static CborRequest request(const std::string &yaml, uint8_t op, const CborCurrentValues &current = nullptr)
{
    CborRequest req;
    std::string error;
    EXPECT_TRUE(cborRequestFromYaml(YAML::Load(yaml), op, req, error, current)) << error;
    return req;
}

/** The error a YAML request is refused with */
static std::string refusal(const std::string &yaml, uint8_t op)
{
    CborRequest req;
    std::string error;
    EXPECT_FALSE(cborRequestFromYaml(YAML::Load(yaml), op, req, error));
    return error;
}

/*-------------------------------------------------------------------------------------------- YAML -> request */

TEST(CborFormat, GetNamesFieldsLikeMinusGet)
{
    CborRequest req = request("{CONFIG_INS: {dynamicModel, rotation}}", CBOR_MSG_TYPE_GET);
    EXPECT_EQ(req.type, CBOR_MSG_TYPE_GET);
    ASSERT_EQ(req.entries.size(), 1u);
    EXPECT_EQ(req.entries[0].first, CBOR_DID_CONFIG_INS);
    EXPECT_EQ(req.entries[0].second, Cbor().arr(2).u(CBOR_CONFIG_INS__DYNAMICMODEL).u(CBOR_CONFIG_INS__ROTATION).out());
}

TEST(CborFormat, GetAcceptsDomainNamesNumbersAndWholeDomains)
{
    CborRequest req = request("{CONFIG_INS: [dynamicModel], 0x0102, CONFIG_CAN}", CBOR_MSG_TYPE_GET);
    ASSERT_EQ(req.entries.size(), 3u);
    EXPECT_EQ(req.entries[0].first, CBOR_DID_CONFIG_INS);
    EXPECT_EQ(req.entries[0].second, Cbor().arr(1).u(CBOR_CONFIG_INS__DYNAMICMODEL).out());
    EXPECT_EQ(req.entries[1].first, CBOR_DID_CONFIG_SYSTEM);
    EXPECT_EQ(req.entries[1].second, Cbor().arr(0).out()) << "no fields = every field";
    EXPECT_EQ(req.entries[2].first, CBOR_DID_CONFIG_CAN);
}

TEST(CborFormat, ALegacyDidNameIsNotADomain)
{
    // DID_FLASH_CONFIG's fields are keys of the domains it maps to; only inside a subscription is a DID a key
    EXPECT_NE(refusal("{DID_FLASH_CONFIG}", CBOR_MSG_TYPE_GET).find("DID_FLASH_CONFIG"), std::string::npos);
}

TEST(CborFormat, GetNestedPathsAndDidKeys)
{
    // RMC_STORED: port 0's INS_2 entry, and port 1's preset
    CborRequest req = request("{RMC_STORED: {0: {DID_INS_2}, 1: {RMC_PRESET}}}", CBOR_MSG_TYPE_GET);
    ASSERT_EQ(req.entries.size(), 1u);
    EXPECT_EQ(req.entries[0].second, Cbor().arr(2).arr(2).u(0).u(DID_INS_2).arr(2).u(1).u(CBOR_RMC_STORED__RMC_PRESET).out());
}

TEST(CborFormat, DataEncodesEachFieldAsItsType)
{
    CborRequest req = request("{CONFIG_INS: {rotation: [0.5, 0, 1.25], dynamicModel: 4, ekfEnabled: true}}",
                              CBOR_MSG_TYPE_DATA);
    ASSERT_EQ(req.entries.size(), 1u);
    EXPECT_EQ(req.entries[0].second, Cbor().map(3).u(CBOR_CONFIG_INS__ROTATION).arr(3).f(0.5f).f(0.0f).f(1.25f)
                                         .u(CBOR_CONFIG_INS__DYNAMICMODEL).u(4)
                                         .u(CBOR_CONFIG_INS__EKFENABLED).b(true).out())
        << "floats stay floats even when written as integers";
}

TEST(CborFormat, DataSetsOneElementFromTheCurrentValues)
{
    cbor_config_ins_t cur = {};
    cur.rotation[0] = 0.5f;
    cur.rotation[1] = -0.25f;
    // An indexed key is quoted, as cltool's -set argument handling does, so YAML reads it as a key
    CborRequest req = request("{CONFIG_INS: {'rotation[2]': 1.5}}", CBOR_MSG_TYPE_DATA,
                              [&](uint16_t did) -> const void * { return did == CBOR_DID_CONFIG_INS ? &cur : nullptr; });
    ASSERT_EQ(req.entries.size(), 1u);
    EXPECT_EQ(req.entries[0].second,
              Cbor().map(1).u(CBOR_CONFIG_INS__ROTATION).arr(3).f(0.5f).f(-0.25f).f(1.5f).out());

    EXPECT_NE(refusal("{CONFIG_INS: {'rotation[2]': 1.5}}", CBOR_MSG_TYPE_DATA).find("current values"),
              std::string::npos);
}

TEST(CborFormat, DataWithoutASchemaUsesNamesAndEnums)
{
    CborRequest req = request("{RMC_STORED: {0: {RMC_PRESET: INS, DID_INS_2: {SUB_RATE_MULTI: 2}}}}", CBOR_MSG_TYPE_DATA);
    ASSERT_EQ(req.entries.size(), 1u);
    EXPECT_EQ(req.entries[0].second, Cbor().map(1).u(0).map(2).u(CBOR_RMC_STORED__RMC_PRESET).u(CBOR_RMC_PRESET_INS)
                                         .u(DID_INS_2).map(1).u(CBOR_SUB_RATE_MULTI).u(2).out());
}

TEST(CborFormat, EnvelopeKeysSetTheRequestOptions)
{
    CborRequest req = request("{MSG_DST_DEV_FILTER: 'GPX-1.0::-1', MSG_ACK_TYPE: ACK_DETAILED, MSG_TYPE: GET, "
                              "CONFIG_GNSS}", CBOR_MSG_TYPE_DATA);
    EXPECT_EQ(req.type, CBOR_MSG_TYPE_GET) << "MSG_TYPE overrides the default operation";
    EXPECT_EQ(req.ackType, CBOR_ACK_TYPE_ACK_DETAILED);
    uint64_t gpxAny;
    ASSERT_TRUE(cborDevFilterFromString("GPX-1.0::-1", gpxAny));
    ASSERT_TRUE(req.dst.has_value());
    EXPECT_EQ(*req.dst, gpxAny);
    ASSERT_EQ(req.entries.size(), 1u);
}

TEST(CborFormat, AFilterIsAWildcardOnlyWhenAPartMatchesEveryValue)
{
    // A GPX-1's major version is 1, so its notation holds "-1" even when every part is specific
    for (const char* specific : { "GPX-1.0::SN928401213", "IMX-5.0::SN62797", "IMX-6.0::942755895", "host" })
    {
        uint64_t f;
        ASSERT_TRUE(cborDevFilterFromString(specific, f)) << specific;
        EXPECT_FALSE(cborDevFilterIsWildcard(f)) << specific;
    }
    for (const char* wildcard : { "GPX", "GPX-1", "GPX-1.0", "GPX-1.0::-1", "IMX::62797", "all" })
    {
        uint64_t f;
        ASSERT_TRUE(cborDevFilterFromString(wildcard, f)) << wildcard;
        EXPECT_TRUE(cborDevFilterIsWildcard(f)) << wildcard;
    }
}

TEST(CborFormat, NumericKeysWriteARawMessage)
{
    // MSG_ROUTE given by number goes out as given, ahead of the domains
    CborRequest req = request("{0xC005: [3], 258: {0: 921600}}", CBOR_MSG_TYPE_DATA);
    ASSERT_EQ(req.entries.size(), 2u);
    EXPECT_EQ(req.entries[0].first, CBOR_MSG_ROUTE);
    EXPECT_EQ(req.entries[0].second, Cbor().arr(1).u(3).out());
    EXPECT_EQ(req.entries[1].first, CBOR_DID_CONFIG_SYSTEM);
    EXPECT_EQ(req.entries[1].second, Cbor().map(1).u(0).u(921600).out());
}

TEST(CborFormat, BadInputIsRefusedWithAReason)
{
    EXPECT_NE(refusal("{NO_SUCH_DID}", CBOR_MSG_TYPE_GET).find("NO_SUCH_DID"), std::string::npos);
    EXPECT_NE(refusal("{CONFIG_INS: {noSuchField}}", CBOR_MSG_TYPE_GET).find("noSuchField"), std::string::npos);
    EXPECT_NE(refusal("{CONFIG_SYSTEM: {navPeriodMs: 5000000000}}", CBOR_MSG_TYPE_DATA).find("navPeriodMs"),
              std::string::npos) << "too large for uint32";
    EXPECT_NE(refusal("{CONFIG_INS: {rotation: [1, 2, 3, 4]}}", CBOR_MSG_TYPE_DATA).find("3 elements"),
              std::string::npos);
    EXPECT_NE(refusal("{RMC_STORED: {0: {RMC_PRESET: NOPE}}}", CBOR_MSG_TYPE_DATA).find("rmc_preset"), std::string::npos);
    EXPECT_NE(refusal("{MSG_REQ_ID: 4, CONFIG_INS}", CBOR_MSG_TYPE_GET).find("MSG_REQ_ID"), std::string::npos);
    EXPECT_FALSE(refusal("[CONFIG_INS]", CBOR_MSG_TYPE_GET).empty()) << "a list is not a map";
}

TEST(CborFormat, AnIndexMustBeADecimalNumber)
{
    for (const char* bad : { "abc", "1x", "-1", "" })
    {
        std::string key = std::string("'rotation[") + bad + "]'";
        EXPECT_NE(refusal("{CONFIG_INS: {" + key + ": 9}}", CBOR_MSG_TYPE_DATA).find("invalid index"), std::string::npos)
            << key;
        EXPECT_NE(refusal("{CONFIG_INS: [" + key + "]}", CBOR_MSG_TYPE_GET).find("invalid index"), std::string::npos)
            << key;
    }
}

TEST(CborFormat, AFieldIsGivenWholeOrByElementNotBoth)
{
    EXPECT_NE(refusal("{CONFIG_INS: {rotation: [1, 2, 3], 'rotation[2]': 9}}", CBOR_MSG_TYPE_DATA)
                  .find("both whole and by element"), std::string::npos);
    EXPECT_NE(refusal("{CONFIG_INS: {'rotation[2]': 9, rotation: [1, 2, 3]}}", CBOR_MSG_TYPE_DATA)
                  .find("both whole and by element"), std::string::npos);
}

TEST(CborFormat, AFloatMustBeFiniteInItsField)
{
    for (const char* bad : { "1e300", "nan", "inf", "-inf" })
        EXPECT_NE(refusal(std::string("{CONFIG_INS: {rotation: [") + bad + ", 0, 0]}}", CBOR_MSG_TYPE_DATA)
                      .find("rotation"), std::string::npos) << bad << " in a float32";
    request("{CONFIG_INS: {rotation: [1e30, 0, 0]}}", CBOR_MSG_TYPE_DATA);     // within float32's range
}

TEST(CborFormat, AValueTooLargeForOneMessageSaysSo)
{
    std::string big = "[0";
    for (int i = 0; i < 2040; i++)
        big += ", 100000";
    big += "]";
    EXPECT_NE(refusal("{49407: " + big + "}", CBOR_MSG_TYPE_DATA).find("do not fit"), std::string::npos)
        << "a raw message key";
    EXPECT_NE(refusal("{CONFIG_CAN: {200: " + big + "}}", CBOR_MSG_TYPE_DATA).find("do not fit"), std::string::npos)
        << "a field the schema does not know";
}

/*-------------------------------------------------------------------------------------------- message -> YAML */

TEST(CborFormat, ReplyAsYamlInTheMinusGetShape)
{
    Bytes msg = Cbor().map(3).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET))
                    .u(CBOR_DID_CONFIG_INS).map(2).u(CBOR_CONFIG_INS__DYNAMICMODEL).u(4)
                                              .u(CBOR_CONFIG_INS__ROTATION).arr(3).f(0.5f).f(0.0f).f(-1.0f)
                    .u(CBOR_DID_RMC_STORED).map(1).u(CBOR_MSG_STATUS).u(CBOR_MSG_STATUS_RESP_TOO_LARGE).out();
    YAML::Node y = cborToYaml(msg.data(), msg.size());
    ASSERT_TRUE(y.IsMap());
    EXPECT_FALSE(y["MSG_TYPE"]) << "envelope left out by default";
    EXPECT_EQ(y["CONFIG_INS"]["dynamicModel"].as<uint32_t>(), 4u);
    EXPECT_FLOAT_EQ(y["CONFIG_INS"]["rotation"][2].as<float>(), -1.0f);
    EXPECT_EQ(y["RMC_STORED"]["MSG_STATUS"].as<std::string>(), "RESP_TOO_LARGE");

    YAML::Node all = cborToYaml(msg.data(), msg.size(), true);
    EXPECT_EQ(all["MSG_TYPE"].as<std::string>(), "REPLY|GET");

    Bytes bad = { 0xA1, 0x0C };
    EXPECT_TRUE(cborToYaml(bad.data(), bad.size()).IsNull());
}

TEST(CborFormat, YamlRoundTripOfAWrite)
{
    std::string yaml = "{CONFIG_SYSTEM: {navPeriodMs: 16}}";
    CborRequest req = request(yaml, CBOR_MSG_TYPE_DATA);
    Bytes msg = req.encode(9);
    YAML::Node y = cborToYaml(msg.data(), msg.size());
    EXPECT_EQ(y["CONFIG_SYSTEM"]["navPeriodMs"].as<uint32_t>(), 16u);
}

TEST(CborFormat, NestedKeysAreNamedWithinTheirMap)
{
    // Keys are scoped to their map: key 0 is navPeriodMs at the top of CONFIG_SYSTEM and type inside platform
    Bytes msg = Cbor().map(3).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET))
                    .u(CBOR_DID_CONFIG_SYSTEM).map(2).u(CBOR_CONFIG_SYSTEM__NAVPERIODMS).u(5)
                                                 .u(CBOR_CONFIG_SYSTEM__PLATFORM).map(1)
                                                     .u(CBOR_CONFIG_SYSTEM__PLATFORM_TYPE).u(1)
                    .u(CBOR_DID_CONFIG_PORTS).map(1).u(CBOR_CONFIG_PORTS__PORT).map(1)
                                                 .u(CBOR_PORT_ID_SER2).map(1).u(0).u(460800).out();
    YAML::Node y = cborToYaml(msg.data(), msg.size());
    EXPECT_EQ(y["CONFIG_SYSTEM"]["navPeriodMs"].as<uint32_t>(), 5u);
    EXPECT_EQ(y["CONFIG_SYSTEM"]["platform"]["type"].as<uint32_t>(), 1u);
    EXPECT_FALSE(y["CONFIG_SYSTEM"]["platform"]["navPeriodMs"]);
    // An instance map's entries are named by their enum symbol, and the key set's fields inside them
    EXPECT_EQ(y["CONFIG_PORTS"]["Port"]["SER2"]["baudRate"].as<uint32_t>(), 460800u);
    EXPECT_NE(cborToDiag(msg.data(), msg.size()).find("platform(5): {type(0): 1}"), std::string::npos)
        << cborToDiag(msg.data(), msg.size());
}

TEST(CborFormat, NestedNamesWriteTheirOwnKeys)
{
    // timeSync.periodMs is key 0 inside timeSync, a different field from CONFIG_GNSS's own periodMs (key 0)
    CborRequest req = request("{CONFIG_GNSS: {timeSync: {periodMs: 500}}, CONFIG_PORTS: {Port: {SER1: {baudRate: 115200}}}}",
                              CBOR_MSG_TYPE_DATA);
    Bytes msg = req.encode(9);
    std::string diag = cborToDiag(msg.data(), msg.size(), false);
    EXPECT_NE(diag.find("265: {2: {0: 500}}"), std::string::npos) << diag;
    EXPECT_NE(diag.find("262: {1: {1: {0: 115200}}}"), std::string::npos) << diag;
}

/*-------------------------------------------------------------------------------------------- diagnostic notation */

TEST(CborFormat, DiagnosticNotationWithNames)
{
    uint64_t imx;
    ASSERT_TRUE(cborDevFilterFromString("IMX-5.0::62797", imx));
    Bytes msg = Cbor().map(3).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET)).u(CBOR_MSG_SRC_DEV_ID).u(imx)
                    .u(CBOR_DID_CONFIG_SYSTEM).map(1).u(CBOR_CONFIG_SYSTEM__NAVPERIODMS).u(16).out();
    EXPECT_EQ(cborToDiag(msg.data(), msg.size()),
              "{MSG_TYPE(0xC000): REPLY|GET, MSG_SRC_DEV_ID(0xC001): IMX-5.0::62797, CONFIG_SYSTEM(258): {navPeriodMs(0): 16}}");
    EXPECT_EQ(cborToDiag(msg.data(), msg.size(), false),
              "{49152: 130, 49153: " + std::to_string(imx) + ", 258: {0: 16}}");
}

TEST(CborFormat, DiagnosticNotationNamesStatusErrKeysAndCapabilities)
{
    Bytes msg = Cbor().map(2)
                    .u(CBOR_DID_CONFIG_SYSTEM).map(2).u(CBOR_MSG_STATUS).u(CBOR_MSG_STATUS_BAD_VALUE)
                                                 .u(CBOR_MSG_ERR_KEYS).arr(1).u(CBOR_CONFIG_SYSTEM__IMUODR)
                    .u(CBOR_DID_CAPABILITIES).map(1).u(CBOR_DID_CONFIG_INS).arr(1).u(CBOR_CONFIG_INS__ROTATION)
                    .out();
    EXPECT_EQ(cborToDiag(msg.data(), msg.size()),
              "{CONFIG_SYSTEM(258): {MSG_STATUS(0xC008): BAD_VALUE, MSG_ERR_KEYS(0xC009): [imuOdr(1)]}, "
              "CAPABILITIES(32764): {CONFIG_INS(259): [rotation(0)]}}");
}

TEST(CborFormat, DiagnosticNotationOfEveryItemType)
{
    Bytes msg = Cbor().arr(9).u(1).i(-5).f(0.375f).f(2.0f).t("a\"b").bytes({ 0xDE, 0xAD }).b(true).tag(1).u(7)
                    .map(1).u(0x1234).arr(0).out();
    EXPECT_EQ(cborToDiag(msg.data(), msg.size(), false), "[1, -5, 0.375, 2.0, \"a\\\"b\", h'dead', true, 1(7), {4660: []}]");
}

TEST(CborFormat, DiagnosticNotationMarksMalformedInput)
{
    Bytes truncated = { 0xA2, 0x0C, 0xA0 };     // a map of 2 with one entry
    EXPECT_EQ(cborToDiag(truncated.data(), truncated.size(), false), "{12: {}<malformed>");
}

TEST(CborFormat, DiagnosticNotationShowsMessageTime)
{
    // GPS week 2390, 100.5 s into the week
    uint64_t ns = (uint64_t)2390 * 604800ull * 1000000000ull + 100500000000ull;
    Bytes msg = Cbor().map(1).u(CBOR_MSG_TIME_GPS_NS).u(ns).out();
    EXPECT_EQ(cborToDiag(msg.data(), msg.size()), "{MSG_TIME_GPS_NS(0xC006): week 2390 tow 100.500000}");
}

TEST(CborFormat, FloatsPrintAtTheirEncodedPrecision)
{
    Bytes msg = Cbor().arr(2).f(0.17453292f).out();
    nanocbor_encoder_t enc;
    uint8_t dbl[9];
    nanocbor_encoder_init(&enc, dbl, sizeof(dbl));
    nanocbor_fmt_double(&enc, 0.1);
    msg.insert(msg.end(), dbl, (uint8_t *)enc.cur);
    EXPECT_EQ(cborToDiag(msg.data(), msg.size(), false), "[0.17453292, 0.1]");

    Bytes reply = Cbor().map(1).u(CBOR_DID_CONFIG_INS).map(1).u(CBOR_CONFIG_INS__LASTPOSITIONUPDATEDISTANCEM)
                        .f(0.17453292f).out();
    YAML::Node y = cborToYaml(reply.data(), reply.size());
    EXPECT_EQ(y["CONFIG_INS"]["lastPositionUpdateDistanceM"].Scalar(), "0.17453292");
}

TEST(CborFormat, IndicesAreNotNamedAsFields)
{
    // ROUTE_FILTER rule 0 and RMC_STORED port 0 are indices; the fields inside them are named
    Bytes msg = Cbor().map(2).u(CBOR_DID_ROUTE_FILTER).map(1).u(0).map(1).u(CBOR_ROUTE_FILTER__ACTION).u(CBOR_ROUTE_ACTION_DENY)
                    .u(CBOR_DID_RMC_STORED).map(1).u(0).map(1).u(CBOR_RMC_STORED__RMC_PRESET).u(CBOR_RMC_PRESET_INS).out();
    EXPECT_EQ(cborToDiag(msg.data(), msg.size()),
              "{ROUTE_FILTER(32767): {0: {ACTION(3): DENY}}, RMC_STORED(32765): {0: {RMC_PRESET(0x8001): INS}}}");
}
