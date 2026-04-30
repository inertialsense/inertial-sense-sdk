/**
 * @file test_log.cpp
 * @brief D-05 / SN-7896 — ISLog directory-walking tests.
 */

#include <gtest/gtest.h>

#include "com_manager.h"

#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISLog.h"
#include "ISLogger.h"
#include "test_data_utils.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kIMX5 = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint16_t kIMX6 = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 6, 0);
constexpr uint32_t kSerialA = 49351u;
constexpr uint32_t kSerialB = 61208u;

struct LogFixture {
    fs::path                          directory;
    std::list<std::vector<uint8_t>*>  messages;
};

// Build a multi-device fixture: two devices, both logging into the
// same directory. Returns (directory, messages-to-delete).
LogFixture buildMultiDeviceFixture(const std::string& hint, float sizeMB) {
    LogFixture f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_log_%s_%d_%ld",
                  hint.c_str(), ::getpid(),
                  static_cast<long>(::time(nullptr)));
    f.directory = dirBuf;
    ISFileManager::DeleteDirectory(f.directory.string());

    GenerateRawLogData(f.messages, sizeMB);
    if (f.messages.empty()) return f;

    {
        cISLogger logger;
        cISLogger::sSaveOptions opts;
        opts.logType               = cISLogger::LOGTYPE_RAW;
        opts.useSubFolderTimestamp = false;
        if (!logger.InitSave(f.directory.string(), opts)) return f;
        auto devA = logger.registerDevice(kIMX5, kSerialA);
        auto devB = logger.registerDevice(kIMX6, kSerialB);
        if (!devA || !devB) return f;
        logger.EnableLogging(true);
        // Alternate writes between devices so each gets data.
        bool toA = true;
        for (auto* msg : f.messages) {
            logger.LogData(toA ? devA : devB,
                           msg->size(),
                           reinterpret_cast<const uint8_t*>(msg->data()));
            toA = !toA;
        }
        logger.CloseAllFiles();
    }
    return f;
}

void teardown(LogFixture& f) {
    for (auto* m : f.messages) delete m;
    f.messages.clear();
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

} // namespace

TEST(ISLog, OpenDirectoryGroupsByDevice) {
    auto f = buildMultiDeviceFixture("twoDev", 1.0f);
    ASSERT_FALSE(f.messages.empty());

    auto log = ISLog::openDirectory(f.directory);
    ASSERT_TRUE(log.has_value()) << log.error().message;

    auto ids = log->deviceIds();
    EXPECT_EQ(ids.size(), 2u);
    EXPECT_NE(std::find(ids.begin(), ids.end(), kSerialA), ids.end());
    EXPECT_NE(std::find(ids.begin(), ids.end(), kSerialB), ids.end());

    EXPECT_GT(log->recordCount(), 0u);
    EXPECT_EQ(log->directory(), f.directory);

    teardown(f);
}

TEST(ISLog, EmptyDirectoryReturnsNotFound) {
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf), "/tmp/test_log_empty_%d_%ld",
                  ::getpid(), static_cast<long>(::time(nullptr)));
    fs::create_directories(dirBuf);

    auto log = ISLog::openDirectory(dirBuf);
    ASSERT_FALSE(log.has_value());
    EXPECT_EQ(log.error().code, ISErrorCode::NotFound);

    fs::remove_all(dirBuf);
}

TEST(ISLog, NonexistentDirectoryReturnsNotFound) {
    auto log = ISLog::openDirectory("/no/such/path/anywhere");
    ASSERT_FALSE(log.has_value());
    EXPECT_EQ(log.error().code, ISErrorCode::NotFound);
}

TEST(ISLog, IgnoresNonRawFiles) {
    auto f = buildMultiDeviceFixture("strayfile", 0.5f);
    ASSERT_FALSE(f.messages.empty());

    // Drop a stray .txt into the dir — should not crash, should not
    // affect device count.
    {
        std::ofstream stray(f.directory / "stray-readme.txt");
        stray << "ignore me";
    }

    auto log = ISLog::openDirectory(f.directory);
    ASSERT_TRUE(log.has_value()) << log.error().message;
    EXPECT_EQ(log->deviceIds().size(), 2u);

    teardown(f);
}

TEST(ISLog, SpanStartEndStraddleAllDevices) {
    auto f = buildMultiDeviceFixture("span", 1.0f);
    ASSERT_FALSE(f.messages.empty());

    auto log = ISLog::openDirectory(f.directory);
    ASSERT_TRUE(log.has_value());

    const uint64_t lo = log->spanStart().value;
    const uint64_t hi = log->spanEnd().value;
    EXPECT_LE(lo, hi);

    // Each device-log's span must be inside [lo, hi].
    for (auto id : log->deviceIds()) {
        const auto& dl = log->device(id);
        EXPECT_GE(dl.spanStart().value, 0u);
        EXPECT_LE(dl.spanStart().value, hi);
        EXPECT_GE(dl.spanEnd().value, lo);
    }

    teardown(f);
}

TEST(ISLog, SegmentPathsListsEverySegment) {
    auto f = buildMultiDeviceFixture("paths", 1.0f);
    ASSERT_FALSE(f.messages.empty());

    auto log = ISLog::openDirectory(f.directory);
    ASSERT_TRUE(log.has_value());

    auto paths = log->segmentPaths();
    EXPECT_FALSE(paths.empty());
    for (const auto& p : paths) {
        EXPECT_TRUE(fs::exists(p)) << p;
        EXPECT_EQ(p.extension(), ".raw");
    }

    teardown(f);
}

TEST(ISLog, DeviceLookupForUnknownIdThrowsOutOfRange) {
    auto f = buildMultiDeviceFixture("missing", 0.5f);
    ASSERT_FALSE(f.messages.empty());

    auto log = ISLog::openDirectory(f.directory);
    ASSERT_TRUE(log.has_value());

    EXPECT_THROW({ (void)log->device(99999u); }, std::out_of_range);

    teardown(f);
}
