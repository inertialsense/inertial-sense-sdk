/**
 * @file test_isdevice_broadcast_binary_data.cpp
 * @brief ISDevice::BroadcastBinaryData() sends to a device with validation disabled (SN-8896).
 *
 * It refuses a device whose protocol version it doesn't recognize.  A device that was never validated (cltool -vd)
 * never learns its protocol version, so cltool -vd -did requested nothing.  The check now applies only to devices that
 * are validated.
 *
 * Uses the SDK bridge test-port pair (test_serial_utils.h): ISDevice writes to TEST2, and the bytes land in TEST3.
 */
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include "ISDevice.h"
#include "test_serial_utils.h"

namespace {

//! @return everything written to the bridge's far end since the last call
std::string drain(port_handle_t peer)
{
    std::string out;
    uint8_t buf[256];
    int n;
    while ((n = portRead(peer, buf, sizeof(buf))) > 0)
        out.append((const char*)buf, n);
    return out;
}

class ISDeviceBroadcastBinaryData : public ::testing::Test
{
protected:
    void SetUp() override
    {
        initTestPorts();
        port = (port_handle_t)TEST2_PORT;
        peer = (port_handle_t)TEST3_PORT;
        ASSERT_TRUE(portIsOpened(port)) << "bridge TEST2 should be VALID|OPENED after initTestPorts()";
        dev_info_t seed = {};       // never validated, so the protocol version is unknown
        device = std::make_shared<ISDevice>(seed, port);
        ASSERT_NE(device->devInfo.protocolVer[0], PROTOCOL_VERSION_CHAR0);
        drain(peer);
    }

    port_handle_t port = nullptr;
    port_handle_t peer = nullptr;
    std::shared_ptr<ISDevice> device;
};

} // namespace


TEST_F(ISDeviceBroadcastBinaryData, RefusesValidatedDeviceWithUnknownProtocolVersion)
{
    EXPECT_FALSE(device->BroadcastBinaryData(DID_INS_1, 10));
    EXPECT_EQ(drain(peer), "");
}

TEST_F(ISDeviceBroadcastBinaryData, SendsToDeviceWithValidationDisabled)
{
    device->disableValidation();
    EXPECT_TRUE(device->BroadcastBinaryData(DID_INS_1, 10));
    EXPECT_NE(drain(peer), "") << "the data request was never sent";
}

TEST_F(ISDeviceBroadcastBinaryData, RefusesInvalidPort)
{
    dev_info_t seed = {};
    ISDevice noPort(seed, nullptr);
    EXPECT_FALSE(noPort.BroadcastBinaryData(DID_INS_1, 10));
    EXPECT_EQ(drain(peer), "");
}
