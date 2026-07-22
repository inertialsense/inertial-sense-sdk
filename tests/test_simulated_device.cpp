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

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
using namespace inertial_sense::testsim;
namespace fs = std::filesystem;

namespace {

//! Drive a simulated device through cISLogger into a fresh raw-log directory,
//! returning the first .raw segment path ({} on failure).
fs::path driveToLog(ISimulatedDevice& dev, const std::string& hint,
                    int ticks, double tow0, uint32_t week) {
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf), "/tmp/test_simdev_%s_%d_%ld",
                  hint.c_str(), ::getpid(), static_cast<long>(::time(nullptr)));
    const fs::path dir = dirBuf;
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

}  // namespace
