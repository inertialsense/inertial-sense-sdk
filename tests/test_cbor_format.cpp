/**
 * @file test_cbor_format.cpp
 * @brief CBOR messages as text: requests from the -get / -set YAML form (names, legacy DID names, numbers, envelope
 *        keys, enums, element writes), replies as YAML, and diagnostic notation with and without registry names.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
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
    CborRequest req = request("{FLASH_CFG: {ser0BaudRate, insRotation}}", CBOR_MSG_TYPE_GET);
    EXPECT_EQ(req.type, CBOR_MSG_TYPE_GET);
    ASSERT_EQ(req.entries.size(), 1u);
    EXPECT_EQ(req.entries[0].first, CBOR_DID_FLASH_CFG);
    EXPECT_EQ(req.entries[0].second, Cbor().arr(2).u(CBOR_FLASH_CFG__SER0BAUDRATE).u(CBOR_FLASH_CFG__INSROTATION).out());
}

TEST(CborFormat, GetAcceptsLegacyDidNamesNumbersAndWholeDids)
{
    CborRequest req = request("{DID_FLASH_CONFIG: [ser1BaudRate], 121, CAN_CONFIG}", CBOR_MSG_TYPE_GET);
    ASSERT_EQ(req.entries.size(), 3u);
    EXPECT_EQ(req.entries[0].first, CBOR_DID_FLASH_CFG);
    EXPECT_EQ(req.entries[0].second, Cbor().arr(1).u(CBOR_FLASH_CFG__SER1BAUDRATE).out());
    EXPECT_EQ(req.entries[1].first, CBOR_DID_GPX_FLASH_CFG);
    EXPECT_EQ(req.entries[1].second, Cbor().arr(0).out()) << "no fields = every field";
    EXPECT_EQ(req.entries[2].first, CBOR_DID_CAN_CONFIG);
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
    CborRequest req = request("{FLASH_CFG: {ser2BaudRate: 460800, insRotation: [0.5, 0, 1.25], dynamicModel: 4}}",
                              CBOR_MSG_TYPE_DATA);
    ASSERT_EQ(req.entries.size(), 1u);
    EXPECT_EQ(req.entries[0].second, Cbor().map(3).u(CBOR_FLASH_CFG__SER2BAUDRATE).u(460800)
                                         .u(CBOR_FLASH_CFG__INSROTATION).arr(3).f(0.5f).f(0.0f).f(1.25f)
                                         .u(CBOR_FLASH_CFG__DYNAMICMODEL).u(4).out())
        << "floats stay floats even when written as integers";
}

TEST(CborFormat, DataSetsOneElementFromTheCurrentValues)
{
    nvm_flash_cfg_t cur = {};
    cur.gnss1AntOffset[0] = 0.5f;
    cur.gnss1AntOffset[1] = -0.25f;
    // An indexed key is quoted, as cltool's -set argument handling does, so YAML reads it as a key
    CborRequest req = request("{FLASH_CFG: {'gnss1AntOffset[2]': 1.5}}", CBOR_MSG_TYPE_DATA,
                              [&](uint16_t did) -> const void * { return did == CBOR_DID_FLASH_CFG ? &cur : nullptr; });
    ASSERT_EQ(req.entries.size(), 1u);
    EXPECT_EQ(req.entries[0].second,
              Cbor().map(1).u(CBOR_FLASH_CFG__GNSS1ANTOFFSET).arr(3).f(0.5f).f(-0.25f).f(1.5f).out());

    EXPECT_NE(refusal("{FLASH_CFG: {'gnss1AntOffset[2]': 1.5}}", CBOR_MSG_TYPE_DATA).find("current values"),
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
                              "GPX_FLASH_CFG}", CBOR_MSG_TYPE_DATA);
    EXPECT_EQ(req.type, CBOR_MSG_TYPE_GET) << "MSG_TYPE overrides the default operation";
    EXPECT_EQ(req.ackType, CBOR_ACK_TYPE_ACK_DETAILED);
    uint64_t gpxAny;
    ASSERT_TRUE(cborDevFilterFromString("GPX-1.0::-1", gpxAny));
    ASSERT_TRUE(req.dst.has_value());
    EXPECT_EQ(*req.dst, gpxAny);
    ASSERT_EQ(req.entries.size(), 1u);
}

TEST(CborFormat, NumericKeysWriteARawMessage)
{
    // MSG_ROUTE given by number goes out as given, ahead of the DIDs
    CborRequest req = request("{0xC005: [3], 12: {2: 921600}}", CBOR_MSG_TYPE_DATA);
    ASSERT_EQ(req.entries.size(), 2u);
    EXPECT_EQ(req.entries[0].first, CBOR_MSG_ROUTE);
    EXPECT_EQ(req.entries[0].second, Cbor().arr(1).u(3).out());
    EXPECT_EQ(req.entries[1].first, CBOR_DID_FLASH_CFG);
    EXPECT_EQ(req.entries[1].second, Cbor().map(1).u(2).u(921600).out());
}

TEST(CborFormat, BadInputIsRefusedWithAReason)
{
    EXPECT_NE(refusal("{NO_SUCH_DID}", CBOR_MSG_TYPE_GET).find("NO_SUCH_DID"), std::string::npos);
    EXPECT_NE(refusal("{FLASH_CFG: {noSuchField}}", CBOR_MSG_TYPE_GET).find("noSuchField"), std::string::npos);
    EXPECT_NE(refusal("{GPX_FLASH_CFG: {ser2BaudRate: 5000000000}}", CBOR_MSG_TYPE_DATA).find("ser2BaudRate"),
              std::string::npos) << "too large for uint32";
    EXPECT_NE(refusal("{FLASH_CFG: {insRotation: [1, 2, 3, 4]}}", CBOR_MSG_TYPE_DATA).find("3 elements"),
              std::string::npos);
    EXPECT_NE(refusal("{RMC_STORED: {0: {RMC_PRESET: NOPE}}}", CBOR_MSG_TYPE_DATA).find("rmc_preset"), std::string::npos);
    EXPECT_NE(refusal("{MSG_REQ_ID: 4, FLASH_CFG}", CBOR_MSG_TYPE_GET).find("MSG_REQ_ID"), std::string::npos);
    EXPECT_FALSE(refusal("[FLASH_CFG]", CBOR_MSG_TYPE_GET).empty()) << "a list is not a map";
}

/*-------------------------------------------------------------------------------------------- message -> YAML */

TEST(CborFormat, ReplyAsYamlInTheMinusGetShape)
{
    Bytes msg = Cbor().map(3).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET))
                    .u(CBOR_DID_FLASH_CFG).map(2).u(CBOR_FLASH_CFG__SER0BAUDRATE).u(921600)
                                             .u(CBOR_FLASH_CFG__INSROTATION).arr(3).f(0.5f).f(0.0f).f(-1.0f)
                    .u(CBOR_DID_RMC_STORED).map(1).u(CBOR_MSG_STATUS).u(CBOR_MSG_STATUS_RESP_TOO_LARGE).out();
    YAML::Node y = cborToYaml(msg.data(), msg.size());
    ASSERT_TRUE(y.IsMap());
    EXPECT_FALSE(y["MSG_TYPE"]) << "envelope left out by default";
    EXPECT_EQ(y["FLASH_CFG"]["ser0BaudRate"].as<uint32_t>(), 921600u);
    EXPECT_FLOAT_EQ(y["FLASH_CFG"]["insRotation"][2].as<float>(), -1.0f);
    EXPECT_EQ(y["RMC_STORED"]["MSG_STATUS"].as<std::string>(), "RESP_TOO_LARGE");

    YAML::Node all = cborToYaml(msg.data(), msg.size(), true);
    EXPECT_EQ(all["MSG_TYPE"].as<std::string>(), "REPLY|GET");

    Bytes bad = { 0xA1, 0x0C };
    EXPECT_TRUE(cborToYaml(bad.data(), bad.size()).IsNull());
}

TEST(CborFormat, YamlRoundTripOfAWrite)
{
    std::string yaml = "{GPX_FLASH_CFG: {ser2BaudRate: 460800}}";
    CborRequest req = request(yaml, CBOR_MSG_TYPE_DATA);
    Bytes msg = req.encode(9);
    YAML::Node y = cborToYaml(msg.data(), msg.size());
    EXPECT_EQ(y["GPX_FLASH_CFG"]["ser2BaudRate"].as<uint32_t>(), 460800u);
}

/*-------------------------------------------------------------------------------------------- diagnostic notation */

TEST(CborFormat, DiagnosticNotationWithNames)
{
    uint64_t imx;
    ASSERT_TRUE(cborDevFilterFromString("IMX-5.0::62797", imx));
    Bytes msg = Cbor().map(3).u(CBOR_MSG_TYPE).u(cbor_msg_reply_type(CBOR_MSG_TYPE_GET)).u(CBOR_MSG_SRC_DEV_ID).u(imx)
                    .u(CBOR_DID_FLASH_CFG).map(1).u(CBOR_FLASH_CFG__SER0BAUDRATE).u(921600).out();
    EXPECT_EQ(cborToDiag(msg.data(), msg.size()),
              "{MSG_TYPE(0xC000): REPLY|GET, MSG_SRC_DEV_ID(0xC001): IMX-5.0::62797, FLASH_CFG(12): {ser0BaudRate(2): 921600}}");
    EXPECT_EQ(cborToDiag(msg.data(), msg.size(), false),
              "{49152: 130, 49153: " + std::to_string(imx) + ", 12: {2: 921600}}");
}

TEST(CborFormat, DiagnosticNotationNamesStatusErrKeysAndCapabilities)
{
    Bytes msg = Cbor().map(2)
                    .u(CBOR_DID_FLASH_CFG).map(2).u(CBOR_MSG_STATUS).u(CBOR_MSG_STATUS_BAD_VALUE)
                                             .u(CBOR_MSG_ERR_KEYS).arr(1).u(CBOR_FLASH_CFG__SER1BAUDRATE)
                    .u(CBOR_DID_CAPABILITIES).map(1).u(CBOR_DID_GPX_FLASH_CFG).arr(1).u(CBOR_GPX_FLASH_CFG__SER2BAUDRATE)
                    .out();
    EXPECT_EQ(cborToDiag(msg.data(), msg.size()),
              "{FLASH_CFG(12): {MSG_STATUS(0xC008): BAD_VALUE, MSG_ERR_KEYS(0xC009): [ser1BaudRate(3)]}, "
              "CAPABILITIES(32764): {GPX_FLASH_CFG(121): [ser2BaudRate(2)]}}");
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
