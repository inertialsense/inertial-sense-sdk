/**
 * @file test_devinfo_confirmation.cpp
 * @brief Regression tests: a discovery hint must not pass as a device response (SN-8567).
 *
 * `DeviceFactory::beginValidation()` pre-seeds `devInfo` from a discovery hint, and RelayPortFactory's
 * hint carries `hdwRunState` and `protocolVer[]` -- every field `hasDeviceInfo()` inspects. Because
 * `validateAsync()` reported ASYNC_STATE__SUCCESS as soon as `hasDeviceInfo()` was true, discovery
 * vouched for devices it had never exchanged a byte with, and a caller waiting for a device's run state
 * to CHANGE was handed a snapshot taken before the change and never asked again.
 *
 * Two independent consequences are pinned here, one per test:
 *
 *  1. An unresponsive port validated anyway. The relay keeps advertising a device's last known
 *     identity, so a device that answers nothing still produced a fully-populated ISDevice.
 *  2. A firmware update could never leave its "waiting for ISbl" phase. ISBFirmwareUpdater resets the
 *     device and then waits for `hdwRunState == HDW_STATE_BOOTLOADER`; the hint says APP, and the relay
 *     does not refresh a hint while the port stays bound to the same endpoint
 *     (`RelayPortFactory.cpp`, the `device.changed` handler), so the value it was waiting on was frozen.
 *
 * No hardware and no OS-specific port: validation reaches the port only through
 * portWrite/portRead/portReadTimeout/portFlush/portAvailable, so an SDK bridge test-port pair is
 * indistinguishable from a real serial port here. The test holds one end and plays the device.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "ISBootloaderBase.h"
#include "ISDevice.h"
#include "test_serial_utils.h"

namespace {

//! The literal 15-byte version query queryDeviceInfoISbl() emits (ISDevice.cpp).
constexpr char ISBL_QUERY[] = ":020000041000EA";

constexpr uint32_t HINT_SERIAL = 60246;     //!< the bench unit this was diagnosed on

/**
 * A dev_info_t as complete as RelayPortFactory::parseDeviceJson() makes it -- i.e. one that satisfies
 * every field hasDeviceInfo() checks, while describing a device nobody has spoken to yet.
 *
 * @param runState what the relay last announced; APP is the case that caused the ISbl wait to hang.
 */
dev_info_t makeRelayHint(eHdwRunStates runState) {
    dev_info_t hint = {};
    hint.hardwareType = IS_HARDWARE_TYPE_IMX;
    hint.hardwareVer[0] = 5;
    hint.hardwareVer[1] = 0;
    hint.serialNumber = HINT_SERIAL;
    hint.hdwRunState = runState;
    hint.protocolVer[0] = PROTOCOL_VERSION_CHAR0;
    hint.protocolVer[1] = PROTOCOL_VERSION_CHAR1;
    hint.protocolVer[2] = PROTOCOL_VERSION_CHAR2;
    hint.protocolVer[3] = PROTOCOL_VERSION_CHAR3;
    return hint;
}

/** Applies a hint the way beginValidation() does, then asserts the precondition these tests rely on. */
std::shared_ptr<ISDevice> makeHintSeededDevice(port_handle_t port, eHdwRunStates runState) {
    dev_info_t hint = makeRelayHint(runState);
    dev_info_t empty = {};
    auto device = std::make_shared<ISDevice>(empty, port);
    device->devInfo = hint;
    device->hdwId = ENCODE_DEV_INFO_TO_HDW_ID(hint);
    device->clearDevInfoConfirmed();
    return device;
}

/**
 * A valid 14-byte ISbl v6j version frame for an IMX-5, accepted by both the prefix/length check and the
 * buf[11]=='.' tail guard. Byte [5] is the processor type: IS_PROCESSOR_STM32L4 is what makes
 * queryDeviceInfoISbl() resolve this to IMX-5.0.
 */
void buildIsblReply(uint8_t out[14]) {
    memset(out, 0, 14);
    out[0] = 0xAA;
    out[1] = 0x55;
    out[2] = 0x06;                                              // major version 6
    out[3] = 0x6A;                                              // 'j'
    out[4] = 0x01;                                              // rom_available
    out[5] = (uint8_t)ISBootloader::IS_PROCESSOR_STM32L4;       // -> IMX-5.0
    out[6] = 0x00;                                              // evb flag
    const uint32_t sn = HINT_SERIAL;
    memcpy(&out[7], &sn, sizeof(sn));
    out[11] = '.';
    out[12] = '\r';
    out[13] = '\n';
}

/**
 * Plays a device sitting in ISbl on the far end of the bridge: ignores everything until a version query
 * arrives, then answers it -- every time, since validateAsync() cycles query types and may reach the
 * ISbl one more than once.
 *
 * Answering only after seeing the query matters: queryDeviceInfoISbl() flushes the port before each
 * attempt, so anything sent earlier would be discarded and the read would time out for the wrong reason.
 */
void playIsblDevice(port_handle_t peer, std::atomic<int>& queriesSeen, std::atomic<bool>& stop) {
    uint8_t reply[14];
    buildIsblReply(reply);

    std::string acc;
    uint8_t buf[256];
    while (!stop.load()) {
        int n = portRead(peer, buf, sizeof(buf));
        if (n > 0) {
            acc.append(reinterpret_cast<const char*>(buf), static_cast<size_t>(n));
            size_t at;
            while ((at = acc.find(ISBL_QUERY)) != std::string::npos) {
                acc.erase(0, at + sizeof(ISBL_QUERY) - 1);
                queriesSeen++;
                portWrite(peer, reply, sizeof(reply));
            }
            if (acc.size() > 4096)
                acc.clear();    // nothing we care about spans this much; don't grow without bound
            continue;
        }
        // A test port never blocks, so yield rather than spin the ring buffer.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/** Drives validateAsync() to a terminal state, or gives up. @return the last state returned. */
int driveValidation(std::shared_ptr<ISDevice>& device, uint32_t timeoutMs, uint32_t wallClockCapMs) {
    const uint32_t deadline = current_timeMs() + wallClockCapMs;
    int state = ISDevice::ASYNC_STATE__PENDING;
    while (current_timeMs() < deadline) {
        state = device->validateAsync(timeoutMs);
        if (state != ISDevice::ASYNC_STATE__PENDING)
            return state;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return state;
}

} // namespace

/**
 * A complete hint for a device that answers nothing must not validate. Before the fix, the very first
 * validateAsync() call returned SUCCESS -- in ~100us, without a byte on the wire.
 */
TEST(devinfo_confirmation, hint_alone_does_not_validate_an_unresponsive_port) {
    initTestPorts();
    port_handle_t port = (port_handle_t)TEST4_PORT;
    ASSERT_TRUE(portIsOpened(port)) << "bridge TEST4 should be VALID|OPENED after initTestPorts()";

    // The peer is deliberately never read from or written to: this is the unresponsive-device case.
    auto device = makeHintSeededDevice(port, HDW_STATE_APP);

    ASSERT_TRUE(device->hasDeviceInfo())
        << "precondition: a relay hint fills every field hasDeviceInfo() checks -- if this fails the "
           "test is no longer exercising the defect";
    ASSERT_EQ(device->devInfoConfirmedAt(), 0u) << "a hint is not a confirmation";

    const int state = driveValidation(device, /*timeoutMs*/ 300, /*wallClockCapMs*/ 5000);

    EXPECT_NE(state, ISDevice::ASYNC_STATE__SUCCESS)
        << "validateAsync() reported SUCCESS for a device that never answered -- the hint is being "
           "treated as a response again (SN-8567)";
    EXPECT_EQ(state, ISDevice::ASYNC_STATE__TIMEOUT)
        << "an unresponsive port should end in TIMEOUT, not " << state;
    EXPECT_EQ(device->devInfoConfirmedAt(), 0u)
        << "nothing answered, so devInfo must still be unconfirmed";
}

/**
 * The hang reproducer. The hint says APP (the relay's pre-reset snapshot) but the device is really in
 * ISbl. Validation must reach the truth, because ISBFirmwareUpdater's wait for HDW_STATE_BOOTLOADER
 * ends on nothing else.
 */
TEST(devinfo_confirmation, stale_app_hint_does_not_mask_a_device_in_isbl) {
    initTestPorts();
    port_handle_t port = (port_handle_t)TEST4_PORT;
    port_handle_t peer = (port_handle_t)TEST5_PORT;
    ASSERT_TRUE(portIsOpened(port)) << "bridge TEST4 should be VALID|OPENED after initTestPorts()";

    std::atomic<int> queriesSeen{0};
    std::atomic<bool> stop{false};
    std::thread responder(playIsblDevice, peer, std::ref(queriesSeen), std::ref(stop));

    auto device = makeHintSeededDevice(port, HDW_STATE_APP);
    ASSERT_TRUE(device->hasDeviceInfo()) << "precondition: the stale hint looks complete";
    ASSERT_EQ(device->devInfo.hdwRunState, HDW_STATE_APP) << "precondition: the hint claims APP";

    const int state = driveValidation(device, /*timeoutMs*/ 1500, /*wallClockCapMs*/ 10000);

    stop = true;
    if (responder.joinable())
        responder.join();

    // Coverage first: if the ISbl query never reached the responder, the assertions below could pass or
    // fail for reasons unrelated to the defect, so an unexercised run must not read as green.
    EXPECT_GT(queriesSeen.load(), 0)
        << "the ISbl version query never reached the responder -- this run did not exercise the defect";

    EXPECT_EQ(state, ISDevice::ASYNC_STATE__SUCCESS)
        << "validation should have identified the device through the ISbl query, got " << state;
    EXPECT_EQ(device->devInfo.hdwRunState, HDW_STATE_BOOTLOADER)
        << "the stale APP hint was reported as the device's run state; a firmware update waiting for "
           "HDW_STATE_BOOTLOADER would wait forever (SN-8567)";
    EXPECT_GT(device->devInfoConfirmedAt(), 0u)
        << "a real ISbl version frame must count as a confirmation";
    EXPECT_EQ(device->devInfo.firmwareVer[0], 6) << "ISbl major version from the reply frame";
    EXPECT_EQ(device->devInfo.firmwareVer[1], 'j') << "ISbl minor version from the reply frame";
}
