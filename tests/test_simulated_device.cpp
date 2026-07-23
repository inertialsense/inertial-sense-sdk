/**
 * @file test_simulated_device.cpp
 * @brief SN-8328 — smoke test for the SimulatedDevice seed. Drives a synthetic
 *        device through cISLogger and verifies the written log reads back with
 *        the expected message mix via the same fromSegments path Logalyzer uses.
 *
 * This proves the seed produces structurally-valid logs end-to-end; richer
 * device/message coverage grows under the Simulated-Device Epic.
 */
#include <gtest/gtest.h>

#include "com_manager.h"  // first — see test_log_reader.cpp comment

#include "DeviceLog.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "data_sets.h"

#include "SimulatedDevice.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace inertial_sense;
using namespace inertial_sense::testsim;
namespace fs = std::filesystem;

namespace {

//! Portable unique temp directory (POSIX + Windows CI).
fs::path makeTempDir(const std::string& prefix) {
    static unsigned counter = 0;
    return fs::temp_directory_path() / (prefix + "_" + std::to_string(counter++));
}

//! Drive a simulated device through cISLogger into a fresh raw-log directory,
//! returning the first .raw segment path ({} on failure).
fs::path driveToLog(ISimulatedDevice& dev, const std::string& hint,
                    int ticks, double tow0, uint32_t week) {
    const fs::path dir = makeTempDir("test_simdev_" + hint);
    ISFileManager::DeleteDirectory(dir.string());

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(dir.string(), opts)) return {};
    auto dl = logger.registerDevice(dev.hardwareId(), dev.serialNumber());
    if (!dl) return {};
    logger.EnableLogging(true);

    const SimPacket di = dev.deviceInfo();
    logger.LogData(dl, static_cast<int>(di.framed.size()), di.framed.data());
    for (int i = 0; i < ticks; ++i) {
        for (const auto& p : dev.tick(tow0 + i, week))
            logger.LogData(dl, static_cast<int>(p.framed.size()), p.framed.data());
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> raws;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.raw$", raws);
    return raws.empty() ? fs::path{} : fs::path(raws.front().name);
}

//! Drive several simulated devices INTERLEAVED into one log directory (each
//! device gets its own per-serial segment file). Returns the log directory.
fs::path driveMultiToLog(const std::string& hint,
                         const std::vector<ISimulatedDevice*>& devs,
                         const std::vector<int>& ticks,
                         double tow0, uint32_t week) {
    const fs::path dir = makeTempDir("test_simdev_" + hint);
    ISFileManager::DeleteDirectory(dir.string());

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(dir.string(), opts)) return {};

    std::vector<std::shared_ptr<cDeviceLog>> dls;
    for (auto* d : devs) {
        auto dl = logger.registerDevice(d->hardwareId(), d->serialNumber());
        if (!dl) return {};
        dls.push_back(dl);
    }
    logger.EnableLogging(true);

    for (std::size_t i = 0; i < devs.size(); ++i) {
        const SimPacket di = devs[i]->deviceInfo();
        logger.LogData(dls[i], static_cast<int>(di.framed.size()), di.framed.data());
    }
    const int maxTicks = *std::max_element(ticks.begin(), ticks.end());
    for (int t = 0; t < maxTicks; ++t) {
        for (std::size_t i = 0; i < devs.size(); ++i) {
            if (t >= ticks[i]) continue;
            for (const auto& p : devs[i]->tick(tow0 + t, week))
                logger.LogData(dls[i], static_cast<int>(p.framed.size()), p.framed.data());
        }
    }
    logger.CloseAllFiles();
    return dir;
}

TEST(SimulatedDeviceTest, ImxDeviceProducesStructurallyValidLog) {
    SimulatedImxDevice dev;
    const fs::path raw = driveToLog(dev, "imx", /*ticks*/ 10, /*tow0*/ 100.0, /*week*/ 2300);
    ASSERT_FALSE(raw.empty());

    auto log = ISDeviceLog::fromSegments({ raw });
    ASSERT_TRUE(log.has_value());

    std::size_t devInfo = 0, ins2 = 0, imu = 0, total = 0;
    for (auto rv : log.value().allRecords()) {
        ++total;
        switch (rv.did()) {
            case DID_DEV_INFO: ++devInfo; break;
            case DID_INS_2:    ++ins2;    break;
            case DID_IMU:      ++imu;     break;
            default: break;
        }
    }
    EXPECT_EQ(devInfo, 1u) << "device announces itself once";
    EXPECT_EQ(ins2, 10u)   << "one INS_2 per tick";
    EXPECT_EQ(imu, 10u)    << "one IMU per tick";
    EXPECT_EQ(total, 21u)  << "1 DEV_INFO + 10 ticks x 2 messages";

    ISFileManager::DeleteDirectory(raw.parent_path().string());
}

// W-4: two devices logged interleaved into one directory must land in separate
// per-device segments and read back correctly partitioned — each device's
// record set isolated, with its own count, no cross-contamination.
TEST(SimulatedDeviceTest, MultiDeviceLogsPartitionByDevice) {
    SimulatedImxDevice::Config ca; ca.serial = 60001u;
    SimulatedImxDevice::Config cb; cb.serial = 60002u; cb.lla[0] = 47.0;
    SimulatedImxDevice devA(ca), devB(cb);
    const int ticksA = 8, ticksB = 5;

    const fs::path dir = driveMultiToLog("multi", { &devA, &devB },
                                         { ticksA, ticksB }, /*tow0*/ 100.0, /*week*/ 2300);
    ASSERT_FALSE(dir.empty());

    std::vector<ISFileManager::file_info_t> raws;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.raw$", raws);
    ASSERT_EQ(raws.size(), 2u) << "one per-device segment (small logs, no rotation)";

    std::vector<std::size_t> counts;
    std::set<uint64_t> deviceIds;
    for (const auto& r : raws) {
        auto log = ISDeviceLog::fromSegments({ fs::path(r.name) });
        ASSERT_TRUE(log.has_value());
        deviceIds.insert(log.value().deviceId());
        std::size_t n = 0;
        for (auto rv : log.value().allRecords()) { (void)rv; ++n; }
        counts.push_back(n);
    }
    EXPECT_EQ(deviceIds.size(), 2u) << "records must resolve to two distinct devices";

    // Each device: 1 DEV_INFO + ticks x {INS_2, IMU}. Isolated, exact.
    std::sort(counts.begin(), counts.end());
    std::vector<std::size_t> expected = {
        1u + static_cast<std::size_t>(ticksB) * 2u,
        1u + static_cast<std::size_t>(ticksA) * 2u,
    };
    std::sort(expected.begin(), expected.end());
    EXPECT_EQ(counts, expected) << "per-device record sets isolated with correct counts";

    ISFileManager::DeleteDirectory(dir.string());
}

}  // namespace
