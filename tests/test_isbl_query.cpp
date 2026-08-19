/**
 * @file test_isbl_query.cpp
 * @brief Regression test: a FAILED ISbl identity query must not modify devInfo.
 *
 * ISDevice::queryDeviceInfoISbl() used to write devInfo.firmwareVer[0..1] as soon as the 0xAA55
 * response prefix matched, BEFORE the buf[11]=='.' && buf[12]=='\r' && buf[13]=='\n' packet-validity
 * guard. A reply that was long enough and correctly prefixed but had a malformed tail therefore
 * mutated devInfo and then returned false -- a failed read dirtying the identity it was only meant to
 * inspect.
 *
 * That strands bootloader version bytes in a devInfo whose remaining fields are later filled in by an
 * APP-mode path, producing a self-contradictory identity (observed on the bench as firmware_ver
 * "fw6.106.0" -- v6*, since firmwareVer[1] == 106 == 'j' -- with build_date 2000-00-00, alongside a
 * run-state of APP). That is worse than cosmetic: it blocks recovery, because ISv2 trusts hdwRunState
 * and fires the APP-mode STPB/BLEN reset at a device already sitting in ISbl, which cannot answer, so
 * it never reaches the "Target is in bootloader mode; promoting policy to FORCE" branch.
 *
 * No hardware needed, and no OS-specific port either. queryDeviceInfoISbl() only ever touches the port
 * through portWrite/portReadTimeout/portFlush/portAvailable, so any port implementing those is
 * indistinguishable from a real serial port to it. This uses an SDK bridge test-port pair
 * (test_serial_utils.h): everything written to one port lands in the paired port's ring buffer, so the
 * test holds one end and plays the device while ISDevice drives the other -- the same topology a pty
 * gives, without the pty.
 *
 * What it pins, going forward: the query rejects a malformed reply, and rejecting it leaves devInfo
 * exactly as it was. The original recipe reported failing on the commit before the guard; that is no
 * longer reproducible by reintroducing the pre-guard write alone, since several other defects on this
 * branch have been fixed in between, so treat the assertions rather than that claim as the contract.
 *
 * Test recipe contributed by the RemoteSerialPorts/bridgeboard agent, which found the ordering defect.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "ISDevice.h"
#include "test_serial_utils.h"

namespace {

//! The literal 15-byte version query queryDeviceInfoISbl() emits (ISDevice.cpp).
constexpr char ISBL_QUERY[] = ":020000041000EA";

/**
 * A 14-byte ISbl reply that is accepted by the prefix/length check but REJECTED by the tail check --
 * exactly the window the fix closes.
 *
 *   [0..1] = AA 55  -> satisfies buf[0]/buf[1]
 *   [2]    = 0x06   -> the version bytes the old code wrote before validating: v6...
 *   [3]    = 0x6A   -> ...'j', i.e. the "fw6.106.0" seen in the field
 *   [4..10]         -> plausible filler (rom_available, processor, evb flag, serial number)
 *   [11]   = 0x00   -> NOT '.', so the validity guard fails
 *   [12..13]= 0D 0A
 */
constexpr uint8_t ISBL_REPLY_BAD_TAIL[14] = {
    0xAA, 0x55, 0x06, 0x6A, 0x01, 0x03, 0x00, 0x11, 0x22, 0x33, 0x44, 0x00, 0x0D, 0x0A
};

//! Sentinels written into firmwareVer before the call; a dirtying query overwrites them with 6/'j'.
constexpr uint8_t SENTINEL_0 = 0xAB;
constexpr uint8_t SENTINEL_1 = 0xCD;

/**
 * Plays the device side of the exchange on the far end of the bridge: swallow the handshake 'U's and the
 * five-byte "\n" Tx-clear loop, answer nothing until the version query arrives, then send the reply.
 *
 * Replying only after seeing the query matters: the clear loop calls portFlush(), so anything sent
 * earlier would be discarded and the read would legitimately time out for the wrong reason.
 *
 * @param peer  the paired port -- reads see what ISDevice wrote, writes land in ISDevice's ring buffer
 * @param stop  set by the test so this returns even if the query never arrives
 */
void respondOnce(port_handle_t peer, std::atomic<bool>& sawQuery, std::atomic<bool>& stop) {
    std::string acc;
    uint8_t buf[128];
    while (!stop.load()) {
        int n = portRead(peer, buf, sizeof(buf));
        if (n > 0) {
            acc.append(reinterpret_cast<const char*>(buf), static_cast<size_t>(n));
            if (acc.find(ISBL_QUERY) != std::string::npos) {
                sawQuery = true;
                portWrite(peer, ISBL_REPLY_BAD_TAIL, sizeof(ISBL_REPLY_BAD_TAIL));
                return;
            }
            continue;
        }
        // A test port never blocks, so yield rather than spin the ring buffer.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

} // namespace

TEST(isbl_query, malformed_tail_response_leaves_devInfo_untouched) {
    // TEST2 <-> TEST3 is a bridge pair: ISDevice drives TEST2, the responder plays the device on TEST3.
    initTestPorts();
    port_handle_t port = (port_handle_t)TEST2_PORT;
    port_handle_t peer = (port_handle_t)TEST3_PORT;
    ASSERT_TRUE(portIsOpened(port)) << "bridge TEST2 should be VALID|OPENED after initTestPorts()";

    std::atomic<bool> sawQuery{false};
    std::atomic<bool> stop{false};
    std::thread responder(respondOnce, peer, std::ref(sawQuery), std::ref(stop));

    dev_info_t seed = {};
    auto device = std::make_shared<ISDevice>(seed, port);
    device->devInfo = {};
    device->devInfo.firmwareVer[0] = SENTINEL_0;
    device->devInfo.firmwareVer[1] = SENTINEL_1;

    const bool queryOk = device->queryDeviceInfoISbl(400);

    // Coverage first: if the responder never saw the query, the assertions below would pass for the
    // wrong reason (a timeout writes nothing either way), so an unexercised run must not read as green.
    EXPECT_TRUE(sawQuery.load())
        << "the query never reached the responder -- this run did not exercise the defect";

    EXPECT_FALSE(queryOk) << "a reply with a malformed tail must not be reported as a successful query";
    EXPECT_EQ(device->devInfo.firmwareVer[0], SENTINEL_0)
        << "failed query overwrote firmwareVer[0] (got 0x" << std::hex
        << (int)device->devInfo.firmwareVer[0] << ") -- the pre-guard write is back";
    EXPECT_EQ(device->devInfo.firmwareVer[1], SENTINEL_1)
        << "failed query overwrote firmwareVer[1] (got 0x" << std::hex
        << (int)device->devInfo.firmwareVer[1] << ") -- the pre-guard write is back";

    stop = true;
    if (responder.joinable())
        responder.join();
}
