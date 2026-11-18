/**
 * @file test_NtripFaultRecovery.cpp
 * @brief End-to-end recovery of the NTRIP correction stream after a network fault (SN-9070).
 *
 * test_TcpPortReconnect.cpp proves the port-level and handshake-level fixes of PR #1388. These tests prove the outcome that matters to a rover: after
 * the caster connection is broken in each of the ways a mobile link breaks it, RTCM corrections are STILL ARRIVING at the end of the run. A real
 * NtripCorrectionService is pointed at a fake caster that streams valid RTCM3 frames, one fault is injected, and recovery is judged only from the end of
 * the run, so frames already in flight when the fault lands cannot be mistaken for a recovery.
 *
 * Ported from rtk_reconnect_demo by Petr Belohlavek (petrbel), https://gist.github.com/petrbel/912c104a3907fcd08474e3c9ced20b42. The real-caster and
 * silent-blackhole scenarios of that program need credentials, root and a public network, so they stay manual: see tools/ntrip_fault/README.md.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>

#if !defined(_WIN32)

#include <chrono>
#include <ostream>
#include <string>
#include <thread>

#include "NtripCorrectionService.h"
#include "core/tcpPort.h"
#include "ntrip_test_helpers.h"
#include "tcp_test_helpers.h"

namespace {

using Clock = std::chrono::steady_clock;

constexpr double FAULT_AT_SEC = 1.0;                //!< When the fault is injected, after the stream is established.
constexpr double TAIL_SEC = 1.5;                    //!< Corrections must have arrived within this much of the end of the run to count as flowing.
constexpr double POSITION_PERIOD_SEC = 0.5;         //!< How often the rover reports its position to the caster.
constexpr int LOOP_SLEEP_MS = 10;

/** One fault and what the service is expected to do about it. */
struct Scenario {
    const char* name;
    ntrip_test::Fault fault;
    bool writeIntoDeadPeer;     //!< After the fault, write GGA without stepping so the failure surfaces on the send path (EPIPE) instead of the receive path.
    long watchdogMs;            //!< Application-level stalled-stream watchdog, 0 for none. The SDK has no stall detection of its own.
    double durationSec;
    bool expectRecovery;
};

/** Names the scenario in failure output instead of gtest's default byte dump. */
void PrintTo(const Scenario& scenario, std::ostream* os) {
    *os << scenario.name;
}

/** What a run observed. */
struct Outcome {
    long rtcmTotal = 0;
    long rtcmAtFault = 0;
    double longestSilenceSec = 0.0;
    double silentForSec = 0.0;              //!< Time between the last correction and the end of the run.
    int handshakes = 0;
    bool opened = false;
    bool valid = false;
    int fdsAfterConnect = 0;
    int fdsAtEnd = 0;
};

/**
 * Runs a real NtripCorrectionService against a fake caster, injects the scenario's fault once, and records what was observed.
 * @param scenario the fault to inject
 * @return what was observed, with handshakes == -1 if the service could not connect in the first place
 */
Outcome runScenario(const Scenario& scenario) {
    Outcome outcome;
    outcome.handshakes = -1;

    ntrip_test::FakeCaster caster;
    if (!caster.isBound())
        return outcome;
    caster.start();

    const Clock::time_point start = Clock::now();
    auto elapsed = [&start] { return std::chrono::duration<double>(Clock::now() - start).count(); };

    std::atomic<long> rtcm{0};
    NtripCorrectionService ntrip;
    ntrip.addRTCM3PacketListeners([&rtcm](uint16_t, const void*, uint32_t) { ++rtcm; });
    if (!ntrip.connect(caster.ntripUrl()))
        return outcome;
    outcome.fdsAfterConnect = tcp_test::countOpenFds();

    const std::string gga = ntrip_test::buildGgaSentence();
    Clock::time_point lastPosition = Clock::now();
    bool faultInjected = false;
    long lastSeenRtcm = 0;
    double lastFrameAt = elapsed();

    while (elapsed() < scenario.durationSec) {
        if (!faultInjected && scenario.fault != ntrip_test::Fault::NONE && elapsed() >= FAULT_AT_SEC) {
            faultInjected = true;
            outcome.rtcmAtFault = rtcm.load();
            caster.requestFault(scenario.fault);

            if (scenario.writeIntoDeadPeer) {
                // EPIPE is only reachable while the SDK still holds a valid descriptor: once a read has observed the error, a write returns early on the
                // negative descriptor and never reaches its EPIPE branch. So write repeatedly without stepping and let the peer's RST land between two writes.
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                for (int i = 0; i < 10; ++i) {
                    ntrip.updatePosition(gga);
                    if (TCP_PORT(ntrip.getSourcePort())->socket < 0)
                        break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }

        ntrip.step();

        if (Clock::now() - lastPosition >= std::chrono::duration<double>(POSITION_PERIOD_SEC)) {
            lastPosition = Clock::now();
            if (ntrip.isConnected())
                ntrip.updatePosition(gga);
        }

        // Gated on isReceivingCorrections() because getLastRtcm3PacketAge() is measured from the UNIX epoch and is enormous before the first packet.
        if (scenario.watchdogMs > 0 && ntrip.isConnected() && ntrip.isReceivingCorrections() && ntrip.getLastRtcm3PacketAge() > static_cast<uint32_t>(scenario.watchdogMs))
            portClose(ntrip.getSourcePort());

        const long rtcmNow = rtcm.load();
        if (rtcmNow != lastSeenRtcm) {
            const double now = elapsed();
            if (faultInjected && now - lastFrameAt > outcome.longestSilenceSec)
                outcome.longestSilenceSec = now - lastFrameAt;
            lastSeenRtcm = rtcmNow;
            lastFrameAt = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_SLEEP_MS));
    }

    outcome.rtcmTotal = rtcm.load();
    outcome.silentForSec = elapsed() - lastFrameAt;
    if (outcome.silentForSec > outcome.longestSilenceSec)
        outcome.longestSilenceSec = outcome.silentForSec;
    outcome.handshakes = caster.handshakes();
    outcome.opened = portIsOpened(ntrip.getSourcePort());
    outcome.valid = portIsValid(ntrip.getSourcePort());
    outcome.fdsAtEnd = tcp_test::countOpenFds();
    return outcome;
}

}   // namespace

class NtripFaultRecovery : public ::testing::TestWithParam<Scenario> {};

TEST_P(NtripFaultRecovery, CorrectionsFlowAtEndOfRun) {
    const Scenario& scenario = GetParam();
    const Outcome outcome = runScenario(scenario);

    ASSERT_NE(-1, outcome.handshakes) << "could not connect to the fake caster";
    ASSERT_GT(outcome.rtcmTotal, 0) << "the caster never streamed, so nothing was tested";

    if (scenario.fault != ntrip_test::Fault::NONE)
        EXPECT_GT(outcome.rtcmAtFault, 0) << "the fault landed before the stream was established";

    if (scenario.expectRecovery) {
        // The verdict comes from the end of the run, not from the first frames after the fault.
        EXPECT_LT(outcome.silentForSec, TAIL_SEC) << "no corrections for the last " << outcome.silentForSec << " s";
        EXPECT_GT(outcome.rtcmTotal, outcome.rtcmAtFault);
        EXPECT_TRUE(outcome.opened);
        EXPECT_TRUE(outcome.valid);

        // The connection must be a new one, otherwise the fault never interrupted anything and this proves nothing. A fault with no new connection
        // (baseline) must not have caused one either.
        if (scenario.fault == ntrip_test::Fault::NONE)
            EXPECT_EQ(1, outcome.handshakes);
        else
            EXPECT_GE(outcome.handshakes, 2);

        // A recovered stream must not leave the old descriptor behind: PR #1388's defect stored -errno in the socket field and leaked the fd.
        EXPECT_EQ(outcome.fdsAfterConnect, outcome.fdsAtEnd);

        // A stalled stream is only recovered once the watchdog has fired, so the silence must have lasted at least that long.
        if (scenario.fault == ntrip_test::Fault::STALL)
            EXPECT_GE(outcome.longestSilenceSec, 0.9 * scenario.watchdogMs / 1000.0);
    } else {
        // A caster that keeps the connection up but stops sending raises no socket error, so there is nothing for the SDK to react to. Only an
        // application-level watchdog can recover it. This documents the limit rather than endorsing it.
        EXPECT_GE(outcome.silentForSec, TAIL_SEC);
        EXPECT_EQ(1, outcome.handshakes);
        EXPECT_TRUE(outcome.opened);
    }
}

INSTANTIATE_TEST_SUITE_P(
    Faults, NtripFaultRecovery,
    ::testing::Values(
        Scenario{"Baseline", ntrip_test::Fault::NONE, false, 0, 2.5, true},
        Scenario{"Reset", ntrip_test::Fault::RESET, false, 0, 4.0, true},
        Scenario{"ResetWithWatchdog", ntrip_test::Fault::RESET, false, 1000, 4.0, true},
        Scenario{"Fin", ntrip_test::Fault::FIN, false, 0, 4.0, true},
        Scenario{"StallWithWatchdog", ntrip_test::Fault::STALL, false, 1000, 4.0, true},
        Scenario{"StallWithoutWatchdogDoesNotRecover", ntrip_test::Fault::STALL, false, 0, 4.0, false},
        Scenario{"WriteToDeadCaster", ntrip_test::Fault::FIN, true, 0, 4.0, true}),
    [](const ::testing::TestParamInfo<Scenario>& info) { return std::string(info.param.name); });

#endif  // !defined(_WIN32)
