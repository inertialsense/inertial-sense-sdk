/**
 * @file test_isdevice_send_nmea.cpp
 * @brief ISDevice::SendNmea() framing and its bound on message size (SN-8678).
 *
 * SendNmea() copies the message into a 1024-byte buffer and adds the '$' header and "*XX\r\n" footer. It must reject an
 * empty message, and one too long to fit, instead of overflowing the buffer, and must not write anything when it does.
 *
 * Uses the SDK bridge test-port pair (test_serial_utils.h): ISDevice writes to TEST2, and the bytes land in TEST3.
 */
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include "ISDevice.h"
#include "test_serial_utils.h"

namespace {

const size_t SEND_NMEA_MAX_LEN = 1017;     // 1024-byte buffer, less '$' header, "*XX\r\n" footer and null terminator

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

class ISDeviceSendNmea : public ::testing::Test
{
protected:
    void SetUp() override
    {
        initTestPorts();
        port = (port_handle_t)TEST2_PORT;
        peer = (port_handle_t)TEST3_PORT;
        ASSERT_TRUE(portIsOpened(port)) << "bridge TEST2 should be VALID|OPENED after initTestPorts()";
        dev_info_t seed = {};
        device = std::make_shared<ISDevice>(seed, port);
        drain(peer);
    }

    port_handle_t port = nullptr;
    port_handle_t peer = nullptr;
    std::shared_ptr<ISDevice> device;
};

} // namespace


TEST_F(ISDeviceSendNmea, AddsHeaderAndChecksumFooter)
{
    EXPECT_GE(device->SendNmea("ASCE,0,GxGGA,1"), 0);
    EXPECT_EQ(drain(peer), "$ASCE,0,GxGGA,1*47\r\n");

    // An existing '$' header is not doubled
    EXPECT_GE(device->SendNmea("$STPC"), 0);
    EXPECT_EQ(drain(peer), "$STPC*14\r\n");
}

TEST_F(ISDeviceSendNmea, RejectsEmptyMessageWithoutWriting)
{
    EXPECT_EQ(device->SendNmea(""), -1);
    EXPECT_EQ(drain(peer), "");
}

TEST_F(ISDeviceSendNmea, SendsLongestMessageThatFits)
{
    const std::string msg(SEND_NMEA_MAX_LEN, 'A');
    EXPECT_GE(device->SendNmea(msg), 0);
    std::string sent = drain(peer);
    ASSERT_EQ(sent.size(), 1 + msg.size() + 5) << "expected '$' + message + \"*XX\\r\\n\"";
    EXPECT_EQ(sent.substr(0, 1 + msg.size()), "$" + msg);
    EXPECT_EQ(sent.substr(sent.size() - 2), "\r\n");
}

TEST_F(ISDeviceSendNmea, RejectsMessageTooLongWithoutWriting)
{
    EXPECT_EQ(device->SendNmea(std::string(SEND_NMEA_MAX_LEN + 1, 'A')), -1);
    EXPECT_EQ(device->SendNmea(std::string(4096, 'A')), -1);
    EXPECT_EQ(drain(peer), "");
}
