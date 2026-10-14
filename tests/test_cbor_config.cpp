/**
 * @file test_cbor_config.cpp
 * @brief DID_CBOR_CONFIG on the host: ISDevice's request engine and the cborcfg helpers.
 *
 * ISDevice drives one end of a bridge test-port pair (test_serial_utils.h); the test plays the device on the
 * other end with a small responder built on the SDK codec. The responder pages its replies with a deliberately
 * small budget, and can drop messages or answer with an error, so paging, SET splitting and read-back, request-ID
 * matching, retries and timeouts all run as they would against firmware.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <functional>
#include <set>
#include <thread>

#include "ISCborConfig.h"
#include "ISDataMappings.h"
#include "ISDevice.h"
#include "nanocbor/nanocbor.h"
#include "test_serial_utils.h"

namespace {

/** Fills a struct with a deterministic, mostly non-default pattern */
template <typename T>
void fillPattern(T& v, uint8_t seed)
{
    uint8_t* b = (uint8_t*)&v;
    for (size_t i = 0; i < sizeof(T); i++)
        b[i] = (uint8_t)(i * 7 + seed);
}

/** True if every CBOR-addressable field of a and b is equal (bytes outside the field tables are not compared) */
bool sameFields(CborConfigImage& a, CborConfigImage& b)
{
    CborConfigFields fa = a.encode(), fb = b.encode();
    if (fa.size() != fb.size())
        return false;
    for (size_t i = 0; i < fa.size(); i++)
        if (fa[i].key != fb[i].key || fa[i].value != fb[i].value)
        {
            ADD_FAILURE() << "field " << cborcfg::keyName(a.hdwId, fa[i].key) << " differs";
            return false;
        }
    return true;
}

/** Plays a DID_CBOR_CONFIG device on the far end of the bridge */
struct Responder
{
    port_handle_t port;
    is_comm_instance_t comm = {};
    uint8_t commBuf[4096] = {};
    CborConfigImage image;
    size_t pageBudget = 512;            //!< CBOR bytes per reply (the firmware minimum page, which holds any single field)
    int dropCount = 0;                  //!< requests to ignore before answering
    int forceStatus = -1;               //!< answer every request with this status, if >= 0
    int requests = 0;                   //!< requests received (including dropped ones)
    std::vector<uint8_t> msgTypes;      //!< msgType of each request received
    std::vector<uint16_t> reqIds;
    bool staleReply = false;            //!< ignore the first request, then answer it with ERROR just before answering the resend
    cbor_cfg_msg_t firstReq = {};

    Responder(port_handle_t p, uint16_t hdwId) : port(p), image(hdwId)
    {
        is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
        fillPattern(image.imx, 3);
        fillPattern(image.can, 5);
        fillPattern(image.rmc, 9);
        fillPattern(image.gpx, 11);
        // keep float fields finite so values survive comparisons
        for (auto& g : image.groups())
            for (uint16_t i = 0; i < g.count; i++)
                if (g.fields[i].type == FCTYPE_FLOAT || g.fields[i].type == FCTYPE_FLOAT_ARRAY)
                    for (unsigned e = 0; e < g.fields[i].count; e++)
                        ((float*)((uint8_t*)g.data + g.fields[i].offset))[e] = 1.5f + e;
                else if (g.fields[i].type == FCTYPE_DOUBLE || g.fields[i].type == FCTYPE_DOUBLE_ARRAY)
                    for (unsigned e = 0; e < g.fields[i].count; e++)
                        ((double*)((uint8_t*)g.data + g.fields[i].offset))[e] = 2.25 + e;
    }

    void sendDevInfo()
    {
        dev_info_t info = {};
        info.hardwareType = (uint8_t)DECODE_HDW_TYPE(image.hdwId);
        info.hardwareVer[0] = (uint8_t)DECODE_HDW_MAJOR(image.hdwId);
        info.serialNumber = 12345;
        info.hdwRunState = HDW_STATE_APP;
        is_comm_data(port, DID_DEV_INFO, sizeof(info), 0, &info);
    }

    void reply(const cbor_cfg_msg_t& req, int status, const std::vector<uint8_t>& cbor = {}, uint16_t cursor = 0)
    {
        std::vector<uint8_t> msg(CBOR_CFG_MSG_HEADER_SIZE + cbor.size());
        auto* m = (cbor_cfg_msg_t*)msg.data();
        m->hdwId = image.hdwId;
        m->msgType = CBOR_CFG_MSG_REPLY;
        m->status = (uint8_t)status;
        m->reqId = req.reqId;
        m->cursor = cursor;
        m->schema = CBOR_CFG_SCHEMA_DEVICE;
        m->length = (uint16_t)cbor.size();
        if (!cbor.empty())
            memcpy(m->data, cbor.data(), cbor.size());
        is_comm_data(port, DID_CBOR_CONFIG, (uint16_t)msg.size(), 0, msg.data());
    }

    /** Encodes from the image, paged at pageBudget, and replies */
    void replyFields(const cbor_cfg_msg_t& req, const uint16_t* keys, size_t keyCount)
    {
        auto g = image.groups();
        uint16_t cursor = req.cursor;
        bool complete = false;
        std::vector<uint8_t> buf(pageBudget);
        int n = flash_cfg_cbor_encode_groups(g.data(), g.size(), keys, keyCount, &cursor, &complete, buf.data(), buf.size());
        ASSERT_GE(n, 0);
        buf.resize(n);
        reply(req, complete ? CBOR_CFG_STATUS_OK : CBOR_CFG_STATUS_PARTIAL, buf, complete ? 0 : cursor);
    }

    void handle(const cbor_cfg_msg_t& req, const uint8_t* data)
    {
        requests++;
        msgTypes.push_back(req.msgType);
        reqIds.push_back(req.reqId);
        if (staleReply && requests == 1) { firstReq = req; return; }
        if (staleReply && requests == 2) reply(firstReq, CBOR_CFG_STATUS_ERROR);
        if (dropCount > 0) { dropCount--; return; }
        if (forceStatus >= 0) { reply(req, forceStatus); return; }

        switch (req.msgType)
        {
        case CBOR_CFG_MSG_GET_ALL:
            replyFields(req, nullptr, 0);
            break;
        case CBOR_CFG_MSG_GET_KEYS:
        {
            std::vector<uint16_t> keys(256);
            int n = flash_cfg_cbor_decode_keys(data, req.length, keys.data(), keys.size());
            ASSERT_GE(n, 0);
            replyFields(req, keys.data(), (size_t)n);
            break;
        }
        case CBOR_CFG_MSG_SET:
        {
            auto g = image.groups();
            std::vector<uint16_t> applied(256);
            size_t count = 0;
            ASSERT_EQ(flash_cfg_cbor_decode_groups(g.data(), g.size(), data, req.length, applied.data(), applied.size(), &count), FLASH_CFG_CBOR_OK);
            cbor_cfg_msg_t r = req;
            r.cursor = 0;
            replyFields(r, applied.data(), count);
            break;
        }
        }
    }

    /** Reads what ISDevice sent and answers DID_CBOR_CONFIG requests */
    void poll()
    {
        uint8_t buf[512];
        int n;
        while ((n = portRead(port, buf, sizeof(buf))) > 0)
        {
            for (int i = 0; i < n; i++)
            {
                if (is_comm_parse_byte(&comm, buf[i]) == _PTYPE_INERTIAL_SENSE_DATA && comm.rxPkt.hdr.id == DID_CBOR_CONFIG)
                {
                    cbor_cfg_msg_t req;
                    memcpy(&req, comm.rxPkt.data.ptr, CBOR_CFG_MSG_HEADER_SIZE);
                    std::vector<uint8_t> payload(comm.rxPkt.data.ptr + CBOR_CFG_MSG_HEADER_SIZE,
                                                 comm.rxPkt.data.ptr + comm.rxPkt.data.size);
                    handle(req, payload.data());
                }
            }
        }
    }
};

class CborConfigDevice : public ::testing::Test
{
protected:
    std::shared_ptr<ISDevice> device;
    std::unique_ptr<Responder> responder;

    void start(uint16_t hdwId)
    {
        initTestPorts();
        port_handle_t port = (port_handle_t)TEST2_PORT;
        responder = std::make_unique<Responder>((port_handle_t)TEST3_PORT, hdwId);
        dev_info_t seed = {};
        seed.hardwareType = (uint8_t)DECODE_HDW_TYPE(hdwId);
        seed.serialNumber = 12345;
        seed.hdwRunState = HDW_STATE_APP;
        device = std::make_shared<ISDevice>(seed, port);
        responder->sendDevInfo();
        pump(20);
    }

    /** Runs device and responder together until done() or about limitMs */
    void pump(int limitMs, const std::function<bool()>& done = [] { return false; })
    {
        for (int t = 0; t < limitMs && !done(); t++)
        {
            device->step();
            responder->poll();
            SLEEP_MS(1);
        }
    }

    CborConfigResult run(const std::function<bool(CborConfigCallback)>& startRequest, int limitMs = 3000)
    {
        CborConfigResult result;
        bool finished = false;
        EXPECT_TRUE(startRequest([&](const CborConfigResult& r) { result = r; finished = true; }));
        pump(limitMs, [&] { return finished; });
        EXPECT_TRUE(finished) << "request never completed";
        return result;
    }

    static std::vector<uint16_t> keysOf(const CborConfigFields& f)
    {
        std::vector<uint16_t> k;
        for (auto& x : f) k.push_back(x.key);
        return k;
    }
};

} // namespace

TEST_F(CborConfigDevice, GetAllPagesAndReassembles)
{
    start(IS_HARDWARE_IMX_6_0);
    CborConfigResult r = run([&](CborConfigCallback cb) { return device->cborConfigGetAll(cb, 0, 200); });
    ASSERT_TRUE(r.ok()) << cborcfg::statusName(r.status);
    EXPECT_EQ(r.hdwId, IS_HARDWARE_IMX_6_0);
    EXPECT_GT(responder->requests, 3) << "the dump should have needed several pages";
    EXPECT_EQ(keysOf(r.fields), cborcfg::keys(IS_HARDWARE_IMX_6_0));

    CborConfigImage got(IS_HARDWARE_IMX_6_0);
    ASSERT_TRUE(got.apply(r.fields));
    EXPECT_TRUE(sameFields(got, responder->image));
    EXPECT_FALSE(device->cborConfigBusy());
}

TEST_F(CborConfigDevice, GetKeysBatchesLongListsAndKeepsOrder)
{
    start(IS_HARDWARE_IMX_6_0);
    std::vector<uint16_t> keys = cborcfg::keys(IS_HARDWARE_IMX_6_0);
    std::reverse(keys.begin(), keys.end());
    keys.insert(keys.begin() + 10, 999);    // unknown to the device
    CborConfigResult r = run([&](CborConfigCallback cb) { return device->cborConfigGetKeys(keys, cb, 0, 200); });
    ASSERT_TRUE(r.ok()) << cborcfg::statusName(r.status);
    keys.erase(keys.begin() + 10);
    EXPECT_EQ(keysOf(r.fields), keys);
    EXPECT_GE(std::count(responder->msgTypes.begin(), responder->msgTypes.end(), CBOR_CFG_MSG_GET_KEYS), 2);
}

TEST_F(CborConfigDevice, SetSplitsLargeWritesAndReadsBackTruncatedReplies)
{
    start(IS_HARDWARE_IMX_6_0);
    CborConfigImage want(IS_HARDWARE_IMX_6_0);
    fillPattern(want.imx, 40);
    fillPattern(want.can, 41);
    fillPattern(want.rmc, 42);
    for (auto& g : want.groups())       // finite floats/doubles
        for (uint16_t i = 0; i < g.count; i++)
            for (unsigned e = 0; e < g.fields[i].count; e++)
                if (g.fields[i].type == FCTYPE_FLOAT || g.fields[i].type == FCTYPE_FLOAT_ARRAY)
                    ((float*)((uint8_t*)g.data + g.fields[i].offset))[e] = -3.0f - e;
                else if (g.fields[i].type == FCTYPE_DOUBLE || g.fields[i].type == FCTYPE_DOUBLE_ARRAY)
                    ((double*)((uint8_t*)g.data + g.fields[i].offset))[e] = -4.5 - e;
    CborConfigFields fields = want.encode();
    ASSERT_GT(cborcfg::encodeMap(fields).size(), (size_t)CBOR_CFG_MSG_DATA_MAX) << "test needs a SET that must be split";

    CborConfigResult r = run([&](CborConfigCallback cb) { return device->cborConfigSet(fields, cb, 0, 200); });
    ASSERT_TRUE(r.ok()) << cborcfg::statusName(r.status);
    EXPECT_GE(std::count(responder->msgTypes.begin(), responder->msgTypes.end(), CBOR_CFG_MSG_SET), 2);
    EXPECT_GE(std::count(responder->msgTypes.begin(), responder->msgTypes.end(), CBOR_CFG_MSG_GET_KEYS), 1)
        << "truncated SET replies should be completed by reading the rest back";
    EXPECT_TRUE(sameFields(responder->image, want));

    std::vector<uint16_t> got = keysOf(r.fields), sent = keysOf(fields);
    std::sort(got.begin(), got.end());
    std::sort(sent.begin(), sent.end());
    EXPECT_EQ(got, sent) << "every applied field is reported once";
}

TEST_F(CborConfigDevice, LostRequestIsResentWithNewRequestId)
{
    start(IS_HARDWARE_GPX_1_0);
    responder->dropCount = 1;
    CborConfigResult r = run([&](CborConfigCallback cb) { return device->cborConfigGetAll(cb, 0, 50); });
    ASSERT_TRUE(r.ok()) << cborcfg::statusName(r.status);
    ASSERT_EQ(responder->requests, 2);
    EXPECT_NE(responder->reqIds[0], responder->reqIds[1]);
    EXPECT_EQ(keysOf(r.fields), cborcfg::keys(IS_HARDWARE_GPX_1_0));
}

TEST_F(CborConfigDevice, LateReplyToResentRequestIsIgnored)
{
    start(IS_HARDWARE_GPX_1_0);
    responder->staleReply = true;
    CborConfigResult r = run([&](CborConfigCallback cb) { return device->cborConfigGetAll(cb, 0, 50); });
    ASSERT_EQ(responder->requests, 2);
    EXPECT_TRUE(r.ok()) << "the late ERROR reply carries the first request's ID and must not end the request: "
                        << cborcfg::statusName(r.status);
}

TEST_F(CborConfigDevice, NoReplyTimesOutAfterRetries)
{
    start(IS_HARDWARE_GPX_1_0);
    responder->dropCount = 100;
    CborConfigResult r = run([&](CborConfigCallback cb) { return device->cborConfigGetAll(cb, 0, 30); });
    EXPECT_EQ(r.status, CBOR_CFG_RESULT_TIMEOUT);
    EXPECT_EQ(responder->requests, 3) << "first send plus two retries";
    EXPECT_FALSE(device->cborConfigBusy());
}

TEST_F(CborConfigDevice, DeviceErrorStatusIsReported)
{
    start(IS_HARDWARE_GPX_1_0);
    responder->forceStatus = CBOR_CFG_STATUS_UNSUPPORTED;
    CborConfigResult r = run([&](CborConfigCallback cb) { return device->cborConfigGetAll(cb, IS_HARDWARE_GPX); });
    EXPECT_EQ(r.status, CBOR_CFG_STATUS_UNSUPPORTED);
    EXPECT_EQ(responder->requests, 1);
}

TEST_F(CborConfigDevice, OnlyOneRequestAtATime)
{
    start(IS_HARDWARE_GPX_1_0);
    responder->dropCount = 100;
    EXPECT_TRUE(device->cborConfigGetAll([](const CborConfigResult&) {}, 0, 1000));
    EXPECT_TRUE(device->cborConfigBusy());
    EXPECT_FALSE(device->cborConfigGetAll([](const CborConfigResult&) {}));
}

TEST_F(CborConfigDevice, BlockingCallReturnsResult)
{
    start(IS_HARDWARE_GPX_1_0);
    std::atomic<bool> stop{false};
    // Test ports are not thread-safe; hold the device's port lock (which step() also takes) while polling.
    std::thread t([&] {
        while (!stop)
        {
            if (device->lockPort())
            {
                responder->poll();
                device->unlockPort();
            }
            SLEEP_MS(1);
        }
    });
    CborConfigResult r = device->cborConfigGetKeysBlocking({ GPX_CFG_FIELD_DYNAMIC_MODEL, GPX_CFG_FIELD_SER0_BAUD });
    stop = true;
    t.join();
    ASSERT_TRUE(r.ok()) << cborcfg::statusName(r.status);
    EXPECT_EQ(responder->requests, 1) << "answered without a resend";
    EXPECT_EQ(keysOf(r.fields), (std::vector<uint16_t>{ GPX_CFG_FIELD_DYNAMIC_MODEL, GPX_CFG_FIELD_SER0_BAUD }));
}

TEST(CborConfigNames, EveryFlashConfigKeyHasItsLegacyFieldName)
{
    for (uint16_t hdwId : { IS_HARDWARE_IMX_6_0, IS_HARDWARE_GPX_1_0 })
    {
        uint32_t did = (hdwId == IS_HARDWARE_GPX_1_0) ? DID_GPX_FLASH_CFG : DID_FLASH_CONFIG;
        for (uint16_t key : cborcfg::keys(hdwId))
        {
            std::string name = cborcfg::keyName(hdwId, key);
            EXPECT_NE(name.rfind("key", 0), 0u) << "key " << key << " has no name";
            uint16_t back = 0;
            EXPECT_TRUE(cborcfg::keyFromName(hdwId, name, back)) << name;
            EXPECT_EQ(back, key) << name;
            if (key < CAN_CFG_FIELD_PERIOD_MULT || hdwId == IS_HARDWARE_GPX_1_0)
                EXPECT_NE(cISDataMappings::NameToInfoMap(did)->find(name), cISDataMappings::NameToInfoMap(did)->end()) << name;
        }
    }
    EXPECT_EQ(cborcfg::keyName(IS_HARDWARE_IMX_6_0, flash_cfg_cbor_rmc_port_id_base(2) + RMC_CFG_FIELD_PERIOD_MULT), "rmc[2].periodMultiple");
    EXPECT_EQ(cborcfg::keyName(IS_HARDWARE_IMX_6_0, CAN_CFG_FIELD_SETTING), "can_setting");
    EXPECT_EQ(cborcfg::keyName(IS_HARDWARE_IMX_6_0, 999), "key999");

    uint16_t key = 0;
    EXPECT_TRUE(cborcfg::keyFromName(IS_HARDWARE_IMX_6_0, "SER0BAUDRATE", key)) << "names are case-insensitive";
    EXPECT_TRUE(cborcfg::keyFromName(IS_HARDWARE_IMX_6_0, "key999", key));
    EXPECT_EQ(key, 999);
    EXPECT_FALSE(cborcfg::keyFromName(IS_HARDWARE_IMX_6_0, "noSuchField", key));
}

TEST(CborConfigMap, RoundTripSplitAndTruncation)
{
    CborConfigImage img(IS_HARDWARE_IMX_6_0);
    fillPattern(img.imx, 1);
    CborConfigFields fields = img.encode();
    ASSERT_FALSE(fields.empty());

    std::vector<uint8_t> map = cborcfg::encodeMap(fields);
    CborConfigFields back;
    ASSERT_TRUE(cborcfg::parseMap(map.data(), map.size(), back));
    ASSERT_EQ(back.size(), fields.size());
    for (size_t i = 0; i < fields.size(); i++)
    {
        EXPECT_EQ(back[i].key, fields[i].key);
        EXPECT_EQ(back[i].value, fields[i].value);
    }

    CborConfigFields tail;
    EXPECT_FALSE(cborcfg::parseMap(map.data(), map.size() - 1, tail)) << "a truncated map is rejected";

    for (const auto& part : cborcfg::splitFields(fields, 100))
        if (part.size() > 1)
            EXPECT_LE(cborcfg::encodeMap(part).size(), 100u);
    size_t total = 0;
    for (const auto& part : cborcfg::splitFields(fields, 100))
        total += part.size();
    EXPECT_EQ(total, fields.size());
}

TEST(CborConfigText, AssignmentsUseLegacySyntaxAndChangeOnlyTheNamedElement)
{
    CborConfigImage img(IS_HARDWARE_IMX_6_0);
    img.imx.insOffset[0] = 1.0f;
    img.imx.insOffset[2] = 3.0f;
    uint16_t key = 0;

    ASSERT_TRUE(cborcfg::parseAssignment(img, "insOffset[1] = 1.25", key));
    EXPECT_EQ(cborcfg::keyName(img.hdwId, key), "insOffset");
    EXPECT_FLOAT_EQ(img.imx.insOffset[0], 1.0f);
    EXPECT_FLOAT_EQ(img.imx.insOffset[1], 1.25f);
    EXPECT_FLOAT_EQ(img.imx.insOffset[2], 3.0f);

    ASSERT_TRUE(cborcfg::parseAssignment(img, "ser2BaudRate=115200", key));
    EXPECT_EQ(img.imx.ser2BaudRate, 115200u);

    ASSERT_TRUE(cborcfg::parseAssignment(img, "can_setting=500", key));
    EXPECT_EQ(img.can.can_setting, 500);
    ASSERT_TRUE(cborcfg::parseAssignment(img, "rmc[2].bits=0x10", key));
    EXPECT_EQ(img.rmc[2].rmc.bits, 0x10u);
    EXPECT_EQ(key, flash_cfg_cbor_rmc_port_id_base(2) + RMC_CFG_FIELD_BITS);
    ASSERT_TRUE(cborcfg::parseAssignment(img, "rmc[2].periodMultiple[5]=3", key));
    EXPECT_EQ(img.rmc[2].periodMultiple[5], 3);
    EXPECT_EQ(img.rmc[2].periodMultiple[4], 0);

    EXPECT_FALSE(cborcfg::parseAssignment(img, "rmc[2].periodMultiple=3", key)) << "an array element must be named";
    EXPECT_FALSE(cborcfg::parseAssignment(img, "rmc[2].periodMultiple[999]=3", key)) << "index out of range";
    EXPECT_FALSE(cborcfg::parseAssignment(img, "rmc[2].periodMultiple[5]=300", key)) << "uint8 overflow";
    EXPECT_FALSE(cborcfg::parseAssignment(img, "rmc[2].bits=abc", key));
    EXPECT_FALSE(cborcfg::parseAssignment(img, "noSuchField=1", key));
    EXPECT_FALSE(cborcfg::parseAssignment(img, "ser2BaudRate", key)) << "no value";

    CborConfigImage gpx(IS_HARDWARE_GPX_1_0);
    ASSERT_TRUE(cborcfg::parseAssignment(gpx, "dynamicModel=4", key));
    EXPECT_EQ(gpx.gpx.dynamicModel, 4);
    EXPECT_EQ(key, GPX_CFG_FIELD_DYNAMIC_MODEL);
}

TEST(CborConfigText, FormatMatchesTheLegacyListing)
{
    CborConfigImage img(IS_HARDWARE_IMX_6_0);
    img.imx.ser0BaudRate = 921600;
    img.imx.insOffset[1] = 1.25f;
    img.rmc[1].periodMultiple[3] = 7;
    img.rmc[1].rmc.bits = 0x40;

    std::string legacy;
    ASSERT_TRUE(cISDataMappings::DidBufferToString(DID_FLASH_CONFIG, (uint8_t*)&img.imx, legacy, "ser0BaudRate"));
    EXPECT_EQ(cborcfg::formatField(img, IMX_CFG_FIELD_SER0_BAUD), legacy.substr(legacy.find('\n') + 1));
    EXPECT_NE(cborcfg::formatField(img, IMX_CFG_FIELD_INS_OFFSET).find("insOffset[1] = 1.25"), std::string::npos);

    std::string rmc = cborcfg::formatField(img, flash_cfg_cbor_rmc_port_id_base(1) + RMC_CFG_FIELD_PERIOD_MULT);
    EXPECT_NE(rmc.find("rmc[1].periodMultiple[3] = 7\n"), std::string::npos);
    EXPECT_EQ(std::count(rmc.begin(), rmc.end(), '\n'), DID_COUNT);
    EXPECT_EQ(cborcfg::formatField(img, flash_cfg_cbor_rmc_port_id_base(1) + RMC_CFG_FIELD_BITS), "rmc[1].bits = 0x40\n");
    EXPECT_EQ(cborcfg::formatField(img, 999), "");
}
