/**
 * @file test_device_log.cpp
 * @brief D-05 / SN-7896 — ISDeviceLog composition tests.
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // first — see test_log_reader.cpp comment

#include "DeviceLog.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "test_data_utils.h"

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <list>
#include <string>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kIMX5HwId  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kSerialA   = 49351u;
constexpr uint32_t kSerialB   = 61208u;

struct DeviceLogFixture {
    fs::path                          directory;
    std::vector<fs::path>             segmentsA;       // device kSerialA
    std::vector<fs::path>             segmentsB;       // device kSerialB (multi-device variant)
    std::list<std::vector<uint8_t>*>  messages;
};

// Build a fixture with N rollover segments for the given device, by
// forcing a small maxFileSize on cISLogger.
DeviceLogFixture buildSingleDeviceFixture(const std::string& hint,
                                          uint32_t serial,
                                          float totalSizeMB,
                                          uint32_t maxSegBytes) {
    DeviceLogFixture f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_device_log_%s_%d_%ld",
                  hint.c_str(), ::getpid(),
                  static_cast<long>(::time(nullptr)));
    f.directory = dirBuf;
    ISFileManager::DeleteDirectory(f.directory.string());

    GenerateRawLogData(f.messages, totalSizeMB);
    if (f.messages.empty()) return f;

    {
        cISLogger logger;
        cISLogger::sSaveOptions opts;
        opts.logType               = cISLogger::LOGTYPE_RAW;
        opts.useSubFolderTimestamp = false;
        opts.maxFileSize           = maxSegBytes;
        if (!logger.InitSave(f.directory.string(), opts)) return f;
        auto devLogger = logger.registerDevice(kIMX5HwId, serial);
        if (!devLogger) return f;
        logger.EnableLogging(true);
        for (auto* msg : f.messages) {
            logger.LogData(devLogger, msg->size(),
                           reinterpret_cast<const uint8_t*>(msg->data()));
        }
        logger.CloseAllFiles();
    }

    std::vector<ISFileManager::file_info_t> files;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true,
                                          "\\.raw$", files);
    std::sort(files.begin(), files.end(),
              [](const auto& a, const auto& b){ return a.name < b.name; });
    for (auto& info : files) {
        f.segmentsA.emplace_back(info.name);
    }
    return f;
}

void teardown(DeviceLogFixture& f) {
    for (auto* m : f.messages) delete m;
    f.messages.clear();
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

} // namespace

TEST(ISDeviceLog, ComposesMultipleSegmentsFromOneDevice) {
    // 2 MB log split into ~500 KB segments → ~4 segments.
    auto f = buildSingleDeviceFixture("multi", kSerialA, 2.0f, 512 * 1024);
    ASSERT_FALSE(f.segmentsA.empty()) << "fixture failed to produce segments";
    ASSERT_GE(f.segmentsA.size(), 2u) << "need ≥2 segments for this test";

    auto dl = ISDeviceLog::fromSegments(f.segmentsA);
    ASSERT_TRUE(dl.has_value()) << "fromSegments: " << dl.error().message;

    EXPECT_EQ(dl->deviceId(), kSerialA);
    EXPECT_EQ(dl->segmentCount(), f.segmentsA.size());
    EXPECT_GT(dl->recordCount(), 0u);

    // Sum-of-segments equals composed total.
    std::size_t sum = 0;
    for (std::size_t i = 0; i < dl->segmentCount(); ++i) {
        sum += dl->segment(i).recordCount();
    }
    EXPECT_EQ(dl->recordCount(), sum);

    // allRecords iterates every record across segments.
    std::size_t walked = 0;
    for ([[maybe_unused]] ISRecordView v : dl->allRecords()) ++walked;
    EXPECT_EQ(walked, dl->recordCount());

    teardown(f);
}

TEST(ISDeviceLog, RefusesSegmentsFromDifferentDevices) {
    auto fA = buildSingleDeviceFixture("mismatch_a", kSerialA, 0.5f, 1024 * 1024);
    auto fB = buildSingleDeviceFixture("mismatch_b", kSerialB, 0.5f, 1024 * 1024);
    ASSERT_FALSE(fA.segmentsA.empty());
    ASSERT_FALSE(fB.segmentsA.empty());

    std::vector<fs::path> mixed = fA.segmentsA;
    mixed.push_back(fB.segmentsA.front());

    auto dl = ISDeviceLog::fromSegments(mixed);
    ASSERT_FALSE(dl.has_value())
        << "expected fromSegments to refuse a mixed-device list";
    EXPECT_EQ(dl.error().code, ISErrorCode::Corrupted);
    // Message should name both serials.
    EXPECT_NE(dl.error().message.find(std::to_string(kSerialA)), std::string::npos);
    EXPECT_NE(dl.error().message.find(std::to_string(kSerialB)), std::string::npos);

    teardown(fA);
    teardown(fB);
}

TEST(ISDeviceLog, EmptyInputReturnsInvalidArgument) {
    auto dl = ISDeviceLog::fromSegments({});
    ASSERT_FALSE(dl.has_value());
    EXPECT_EQ(dl.error().code, ISErrorCode::InvalidArgument);
}

TEST(ISDeviceLog, RecordsByDidFiltersAcrossSegments) {
    auto f = buildSingleDeviceFixture("perdid", kSerialA, 1.5f, 384 * 1024);
    ASSERT_GE(f.segmentsA.size(), 2u);

    auto dl = ISDeviceLog::fromSegments(f.segmentsA);
    ASSERT_TRUE(dl.has_value());

    auto dids = dl->presentDids();
    ASSERT_FALSE(dids.empty());

    // The sum of records(did).size() across all DIDs must equal the
    // composed total (every record has a DID).
    std::size_t accum = 0;
    for (auto did : dids) {
        accum += dl->records(did).size();
    }
    EXPECT_EQ(accum, dl->recordCount());

    teardown(f);
}

TEST(ISDeviceLog, SeekCrossesSegmentBoundary) {
    auto f = buildSingleDeviceFixture("seek", kSerialA, 1.0f, 256 * 1024);
    ASSERT_GE(f.segmentsA.size(), 2u);

    auto dl = ISDeviceLog::fromSegments(f.segmentsA);
    ASSERT_TRUE(dl.has_value());

    // Pick a target timestamp halfway through the composed span.
    const uint64_t lo = dl->spanStart().value;
    const uint64_t hi = dl->spanEnd().value;
    if (lo == 0 || hi == 0 || lo == hi) {
        teardown(f);
        GTEST_SKIP() << "fixture span isn't usable for seek test";
    }
    const uint64_t target = lo + (hi - lo) / 2;

    auto it = dl->seek(TimeStamp::fromPayloadToW(target, kSerialA));
    ASSERT_NE(it, dl->allRecords().end());
    EXPECT_GE((*it).timestamp().value, target);

    teardown(f);
}

TEST(ISDeviceLog, SegmentBoundaryCallbackFires) {
    auto f = buildSingleDeviceFixture("boundary", kSerialA, 1.0f, 256 * 1024);
    ASSERT_GE(f.segmentsA.size(), 2u);

    auto dl = ISDeviceLog::fromSegments(f.segmentsA);
    ASSERT_TRUE(dl.has_value());

    std::atomic<std::size_t> boundaryHits{0};
    dl->setOnSegmentBoundary([&](std::size_t){ ++boundaryHits; });

    for ([[maybe_unused]] ISRecordView v : dl->allRecords()) { /* drain */ }
    // For N segments we expect (N-1) boundary crossings.
    EXPECT_EQ(boundaryHits.load(), dl->segmentCount() - 1);

    teardown(f);
}
