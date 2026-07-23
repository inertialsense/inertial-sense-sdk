/**
 * @file SimulatedDevice.h
 * @brief SN-8328 seed — a hardware-free synthetic device that emits
 *        structurally-valid ISB packets for driving cISLogger / ISLogWriter
 *        in tests.
 *
 * This is a deliberately MINIMAL seed for a future "Simulated Device" test
 * framework (see the Simulated-Device Epic). The intent: a device exposes a
 * small set of generators that produce the same message shapes an idle IMX /
 * GPX device would (DEV_INFO, INS_*, PIMU, SYS_PARAMS, FLASH_CFG, GNSS#_*, …),
 * so a test can wire it into the log writer and get a log that is, for all
 * intents and purposes, structurally equivalent to a real capture — without
 * hardware and without a recorded fixture.
 *
 * Scope of THIS seed: the `ISimulatedDevice` interface, a `frame()` helper, and
 * one concrete `SimulatedImxDevice` emitting DEV_INFO + INS_2 + IMU. Richer
 * devices (GPX position walk + gaussian noise, GNSS satellite sets/statuses,
 * SYS_PARAMS/FLASH_CFG, configurable operating modes) grow under the Epic as
 * per-device / per-message stories. Kept header-only so tests adopt it with no
 * build-graph change. Generators are DETERMINISTIC (no RNG, no wall-clock) so
 * fixtures are reproducible; a future gaussian-walk device would take an
 * explicit seed argument.
 */
#ifndef IS_TEST_SIMULATED_DEVICE_H
#define IS_TEST_SIMULATED_DEVICE_H

#include "com_manager.h"  // must precede ISComm-pulling headers
#include "ISComm.h"
#include "data_sets.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace inertial_sense {
namespace testsim {

//! A single framed ISB packet, ready to hand to cISLogger::LogData.
struct SimPacket {
    uint32_t             did;
    std::vector<uint8_t> framed;
};

//! Frame a DID payload into a standalone ISB packet (shared by concrete devices).
inline SimPacket frame(uint32_t did, const void* payload, std::size_t size) {
    is_comm_instance_t comm{};
    uint8_t initBuf[64];
    is_comm_init(&comm, initBuf, sizeof(initBuf), nullptr);
    uint8_t pkt[2048];
    const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm,
                                      static_cast<uint16_t>(did),
                                      static_cast<uint16_t>(size), 0,
                                      const_cast<void*>(payload));
    SimPacket p;
    p.did = did;
    if (n > 0) p.framed.assign(pkt, pkt + n);
    return p;
}

/**
 * @brief Interface for a synthetic device that emits ISB telemetry.
 *
 * A device is driven tick-by-tick at a caller-supplied GPS time; each tick
 * returns the packets the device would emit at that instant. `deviceInfo()`
 * is logged once at the start (mirrors a real device announcing itself).
 */
class ISimulatedDevice {
public:
    virtual ~ISimulatedDevice() = default;
    virtual uint16_t hardwareId() const   = 0;   //!< ENCODE_HDW_ID(...) for registerDevice
    virtual uint32_t serialNumber() const = 0;
    virtual SimPacket deviceInfo() const  = 0;                       //!< DID_DEV_INFO, logged once
    virtual std::vector<SimPacket> tick(double towSec, uint32_t gpsWeek) = 0;
};

/**
 * @brief Minimal IMX-shaped device: DEV_INFO once, then INS_2 (sync/ToW) + IMU
 *        (non-sync) each tick. Fixed LLA with an optional deterministic
 *        per-tick latitude drift (a stand-in for a future gaussian walk).
 */
class SimulatedImxDevice : public ISimulatedDevice {
public:
    struct Config {
        uint32_t serial  = 60001u;
        double   lla[3]  = {40.0, -111.0, 1400.0};
        double   llaStep = 0.0;   //!< deterministic latitude increment per tick
    };
    SimulatedImxDevice() : cfg_(Config{}) {}
    explicit SimulatedImxDevice(const Config& cfg) : cfg_(cfg) {}

    uint16_t hardwareId() const override { return ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0); }
    uint32_t serialNumber() const override { return cfg_.serial; }

    SimPacket deviceInfo() const override {
        dev_info_t di{};
        di.serialNumber = cfg_.serial;
        di.hdwRunState  = HDW_STATE_APP;
        return frame(DID_DEV_INFO, &di, sizeof(di));
    }

    std::vector<SimPacket> tick(double towSec, uint32_t gpsWeek) override {
        std::vector<SimPacket> out;

        ins_2_t ins{};
        ins.week       = gpsWeek;
        ins.timeOfWeek = towSec;
        ins.qn2b[0]    = 1.0f;
        ins.lla[0]     = cfg_.lla[0] + cfg_.llaStep * static_cast<double>(tickCount_);
        ins.lla[1]     = cfg_.lla[1];
        ins.lla[2]     = cfg_.lla[2];
        out.push_back(frame(DID_INS_2, &ins, sizeof(ins)));

        imu_t imu{};
        imu.time       = towSec;
        imu.I.acc[2]   = -9.81f;
        out.push_back(frame(DID_IMU, &imu, sizeof(imu)));

        ++tickCount_;
        return out;
    }

private:
    Config   cfg_;
    uint64_t tickCount_ = 0;
};

}  // namespace testsim
}  // namespace inertial_sense

#endif  // IS_TEST_SIMULATED_DEVICE_H
