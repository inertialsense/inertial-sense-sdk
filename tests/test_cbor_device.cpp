/**
 * @file test_cbor_device.cpp
 * @brief The host CBOR API end to end: ISDevice drives one end of a bridge test-port pair (test_serial_utils.h) and the
 *        far end is a device built on the same message layer the firmware runs (cbor_device_handle()), serving
 *        FLASH_CFG through its generated schema. Covers request/reply matching, ACK levels, automatic narrowing on
 *        RESP_TOO_LARGE, timeouts, collecting several replies, and the device-filter notation and time helpers.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <thread>
#include <functional>
#include <memory>

#include "ISCbor.h"
#include "ISDevice.h"
#include "test_serial_utils.h"

namespace {

static const uint64_t IMX_ID = 0x0D400000'0000F54Dull;

/** A device on the far end of the bridge, answering DID_CBOR with the firmware's message layer */
struct Responder
{
    port_handle_t port;
    is_comm_instance_t comm = {};
    uint8_t commBuf[4096] = {};
    nvm_flash_cfg_t flash = {};
    uint8_t reply[ISB_MAX_DATASET_SIZE];
    alignas(8) uint8_t scratch[sizeof(nvm_flash_cfg_t)];
    cbor_did_t dids[1];
    size_t replySize = sizeof(reply);
    int requests = 0;
    bool silent = false;
    int extraReplies = 0;       //!< answer each request this many more times (as further devices would)

    explicit Responder(port_handle_t p) : port(p)
    {
        is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
        flash.ser0BaudRate = 921600;
        dids[0] = { CBOR_DID_FLASH_CFG, &g_cborSchema_FLASH_CFG, sizeof(nvm_flash_cfg_t), readFn, writeFn, nullptr,
                    nullptr, nullptr };
    }
    static const void* readFn(const cbor_did_t*, void* ctx) { return &((Responder*)ctx)->flash; }
    static uint8_t writeFn(const cbor_did_t*, void* ctx, const void* v)
    {
        Responder* r = (Responder*)ctx;
        r->flash = *(const nvm_flash_cfg_t*)v;
        if (r->flash.ser1BaudRate > 921600)
            r->flash.ser1BaudRate = 921600;     // the device adjusts a value
        return CBOR_MSG_STATUS_OK;
    }

    void sendDevInfo()
    {
        dev_info_t info = {};
        info.hardwareType = IS_HARDWARE_TYPE_IMX;
        info.hardwareVer[0] = 5;
        info.serialNumber = 62797;
        info.hdwRunState = HDW_STATE_APP;
        is_comm_data(port, DID_DEV_INFO, sizeof(info), 0, &info);
    }

    void poll()
    {
        uint8_t buf[512];
        int n;
        while ((n = portRead(port, buf, sizeof(buf))) > 0)
        {
            for (int i = 0; i < n; i++)
            {
                if (is_comm_parse_byte(&comm, buf[i]) != _PTYPE_INERTIAL_SENSE_DATA || comm.rxPkt.hdr.id != DID_CBOR)
                    continue;
                requests++;
                if (silent)
                    continue;
                cbor_device_t dev = { IMX_ID, dids, 1, reply, replySize, scratch, sizeof(scratch), nullptr, this };
                int len = cbor_device_handle(&dev, comm.rxPkt.data.ptr, comm.rxPkt.data.size);
                for (int k = 0; len > 0 && k <= extraReplies; k++)
                    is_comm_data(port, DID_CBOR, (uint16_t)len, 0, reply);
            }
        }
    }
};

class CborDeviceApi : public ::testing::Test
{
protected:
    std::shared_ptr<ISDevice> device;
    std::unique_ptr<Responder> responder;

    void SetUp() override
    {
        initTestPorts();
        responder = std::make_unique<Responder>((port_handle_t)TEST3_PORT);
        dev_info_t seed = {};
        seed.hardwareType = IS_HARDWARE_TYPE_IMX;
        seed.serialNumber = 62797;
        seed.hdwRunState = HDW_STATE_APP;
        device = std::make_shared<ISDevice>(seed, (port_handle_t)TEST2_PORT);
        responder->sendDevInfo();
        pump(20);
    }

    void pump(int limitMs, const std::function<bool()>& done = [] { return false; })
    {
        for (int t = 0; t < limitMs && !done(); t++)
        {
            device->step();
            responder->poll();
            SLEEP_MS(1);
        }
    }

    std::vector<CborReply> run(const CborRequest& req, uint32_t timeoutMs = 300, bool collectAll = false)
    {
        std::vector<CborReply> result;
        bool finished = false;
        EXPECT_TRUE(device->cborRequestAsync(req, [&](const std::vector<CborReply>& r) { result = r; finished = true; },
                                             timeoutMs, collectAll));
        pump(3000, [&] { return finished; });
        EXPECT_TRUE(finished) << "request never finished";
        return result;
    }
};

}   // namespace

TEST_F(CborDeviceApi, GetReadsFieldsByKey)
{
    auto replies = run(CborRequest(CBOR_MSG_TYPE_GET).getKeys(CBOR_DID_FLASH_CFG, { CBOR_FLASH_CFG__SER0BAUDRATE }));
    ASSERT_EQ(replies.size(), 1u);
    const CborReply& r = replies[0];
    EXPECT_TRUE(cbor_msg_is_reply(r.env.type));
    EXPECT_EQ(r.env.srcDevId, IMX_ID);
    nvm_flash_cfg_t got = {};
    EXPECT_EQ(r.decode(CBOR_DID_FLASH_CFG, g_cborSchema_FLASH_CFG, &got), 1);
    EXPECT_EQ(got.ser0BaudRate, 921600u);
}

TEST_F(CborDeviceApi, DataIsAppliedAndDetailedAckReturnsWhatWasKept)
{
    nvm_flash_cfg_t v = responder->flash;
    v.ser0BaudRate = 460800;
    v.ser1BaudRate = 3000000;
    auto replies = run(CborRequest().ack(CBOR_ACK_TYPE_ACK_DETAILED)
                           .set(CBOR_DID_FLASH_CFG, g_cborSchema_FLASH_CFG, &v,
                                { CBOR_FLASH_CFG__SER0BAUDRATE, CBOR_FLASH_CFG__SER1BAUDRATE }));
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(responder->flash.ser0BaudRate, 460800u);
    nvm_flash_cfg_t kept = {};
    EXPECT_EQ(replies[0].decode(CBOR_DID_FLASH_CFG, g_cborSchema_FLASH_CFG, &kept), 2);
    EXPECT_EQ(kept.ser1BaudRate, 921600u) << "the value the device kept";
}

TEST_F(CborDeviceApi, RejectedValueReportsItsKey)
{
    std::vector<uint8_t> bad = { 0xA1, (uint8_t)CBOR_FLASH_CFG__SER1BAUDRATE, 0x20 };   // {ser1BaudRate: -1}
    auto replies = run(CborRequest().value(CBOR_DID_FLASH_CFG, bad));
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies[0].status(CBOR_DID_FLASH_CFG), CBOR_MSG_STATUS_BAD_VALUE);
    auto keys = replies[0].errKeys(CBOR_DID_FLASH_CFG);
    ASSERT_EQ(keys.size(), 1u);
    EXPECT_EQ(keys[0].key[0], CBOR_FLASH_CFG__SER1BAUDRATE);
}

TEST_F(CborDeviceApi, GetStructNarrowsAfterTooLarge)
{
    responder->replySize = 200;     // far smaller than FLASH_CFG
    responder->flash.refLla[0] = 40.2518;
    responder->flash.insRotation[2] = -1.5f;
    nvm_flash_cfg_t got = {};
    std::atomic<bool> stop{ false };
    std::thread t([&] {     // the blocking call runs step() itself; the responder answers alongside it
        while (!stop) { if (device->lockPort()) { responder->poll(); device->unlockPort(); } SLEEP_MS(1); }
    });
    int n = device->cborGetStruct(CBOR_DID_FLASH_CFG, g_cborSchema_FLASH_CFG, &got, std::nullopt, 300);
    stop = true;
    t.join();
    EXPECT_EQ(n, g_cborSchema_FLASH_CFG.count) << "every field read";
    EXPECT_GT(responder->requests, 1) << "needed more than one request";
    EXPECT_EQ(got.refLla[0], 40.2518);
    EXPECT_EQ(got.insRotation[2], -1.5f);
}

TEST_F(CborDeviceApi, NoReplyTimesOutEmpty)
{
    responder->silent = true;
    auto replies = run(CborRequest(CBOR_MSG_TYPE_GET).get(CBOR_DID_FLASH_CFG), 100);
    EXPECT_TRUE(replies.empty());
    EXPECT_FALSE(device->cborBusy());
}

TEST_F(CborDeviceApi, CollectAllGathersEveryReply)
{
    responder->extraReplies = 1;
    auto replies = run(CborRequest(CBOR_MSG_TYPE_GET).to(UINT64_MAX).get(CBOR_DID_CAPABILITIES), 100, true);
    EXPECT_EQ(replies.size(), 2u);
}

TEST_F(CborDeviceApi, OneRequestAtATime)
{
    responder->silent = true;
    EXPECT_TRUE(device->cborRequestAsync(CborRequest().get(CBOR_DID_CAPABILITIES), nullptr, 100));
    EXPECT_FALSE(device->cborRequestAsync(CborRequest().get(CBOR_DID_CAPABILITIES), nullptr, 100));
    pump(500, [&] { return !device->cborBusy(); });
    EXPECT_FALSE(device->cborBusy());
}

TEST(CborNotation, DeviceFiltersRoundTrip)
{
    struct Case { const char* str; uint64_t filter; };
    const Case cases[] = {
        { "IMX-5.0::62797",  0x0D400000'0000F54Dull },
        { "GPX-1.0::-1",     0x1040FFFF'FFFFFFFFull },
        { "GPX--1.-1::-1",   0x13FFFFFF'FFFFFFFFull },
        { "-1::-1",          UINT64_MAX },
        { "host",            0 },
    };
    for (const Case& c : cases)
    {
        SCOPED_TRACE(c.str);
        uint64_t f = 1;
        ASSERT_TRUE(cborDevFilterFromString(c.str, f));
        EXPECT_EQ(f, c.filter);
        EXPECT_EQ(cborDevFilterToString(c.filter), c.str);
    }
    uint64_t f;
    EXPECT_TRUE(cborDevFilterFromString("GPX", f)) << "missing version and serial are wildcards";
    EXPECT_EQ(f, 0x13FFFFFF'FFFFFFFFull);
    EXPECT_TRUE(cborDevFilterFromString("all", f));
    EXPECT_EQ(f, UINT64_MAX);
    EXPECT_TRUE(cborDevFilterFromString("0x1040FFFFFFFFFFFF", f));
    EXPECT_EQ(f, 0x1040FFFF'FFFFFFFFull);
    EXPECT_FALSE(cborDevFilterFromString("GPX-99.0::-1", f)) << "major version out of range";
}

TEST(CborNotation, GpsTimeConversions)
{
    uint32_t week;
    double tow;
    cborGpsNsToWeekTow(2441ull * 604800 * 1000000000ull + 412804'000000000ull + 5000, week, tow);
    EXPECT_EQ(week, 2441u);
    EXPECT_NEAR(tow, 412804.000005, 1e-9);
    // GPS epoch: 1980-01-06T00:00:00Z = Unix 315964800 (before any leap second)
    EXPECT_DOUBLE_EQ(cborGpsNsToUnixUtc(0, 0), 315964800.0);
    EXPECT_DOUBLE_EQ(cborGpsNsToUnixUtc(1000000000ull, 18), 315964800.0 + 1 - 18);
}
