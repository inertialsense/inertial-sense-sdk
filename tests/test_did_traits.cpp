/**
 * @file test_did_traits.cpp
 * @brief D-03 / SN-7894 — DIDTraits + templated reader sugar tests.
 *
 * Strategy mirrors test_log_reader / test_log_writer: synthesize a
 * fixture via cISLogger from a small list of records, then exercise
 * `reader.records<DID>()` and `records_typed<DID>(reader)`.
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // first

#include "DIDTraits.h"
#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISLogReader.h"
#include "ISLogReaderSugar.h"
#include "ISLogger.h"
#include "data_sets.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <list>
#include <string>
#include <type_traits>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 999444u;

// ---------------------------------------------------------------------------
// Compile-time tests — these run at compile time via static_assert.
// ---------------------------------------------------------------------------
static_assert(std::is_same_v<DIDTraits<DID_INS_2>::type, ins_2_t>,
              "DIDTraits<DID_INS_2> should map to ins_2_t");
static_assert(std::is_same_v<DIDTraits<DID_IMU>::type, imu_t>,
              "DIDTraits<DID_IMU> should map to imu_t");
static_assert(std::is_same_v<DIDTraits<DID_BAROMETER>::type, barometer_t>,
              "DIDTraits<DID_BAROMETER> should map to barometer_t");
static_assert(DIDTraits<DID_INS_2>::expected_size == sizeof(ins_2_t));
static_assert(has_traits_v<DID_INS_2>);
static_assert(has_traits_v<DID_IMU>);
static_assert(has_traits_v<DID_GNSS1_POS>);

// Helper to assemble a fixture .raw with a hand-built sequence of records.
struct FixturePaths {
    fs::path directory;
    fs::path rawFile;
};

void writeRecord(cISLogger& logger, std::shared_ptr<cDeviceLog> dev,
                 uint32_t did, const void* payload, std::size_t size) {
    void* mutablePayload = const_cast<void*>(payload);
    is_comm_instance_t comm{};
    uint8_t buf[1024];
    is_comm_init(&comm, buf, sizeof(buf), nullptr);
    uint8_t pkt[2048];
    const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm,
                                      static_cast<uint16_t>(did),
                                      static_cast<uint16_t>(size), 0,
                                      mutablePayload);
    if (n > 0) logger.LogData(dev, n, pkt);
}

FixturePaths buildFixture(const std::string& hint,
                          const std::vector<std::pair<uint32_t, std::vector<uint8_t>>>& records) {
    FixturePaths f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_did_traits_%s_%d_%ld",
                  hint.c_str(), ::getpid(),
                  static_cast<long>(::time(nullptr)));
    f.directory = dirBuf;
    ISFileManager::DeleteDirectory(f.directory.string());

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(f.directory.string(), opts)) return f;
    auto devLogger = logger.registerDevice(kFixtureHwId, kFixtureSerial);
    if (!devLogger) return f;
    logger.EnableLogging(true);

    for (const auto& [did, payload] : records) {
        writeRecord(logger, devLogger, did, payload.data(), payload.size());
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> rawFiles;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true,
                                          "\\.raw$", rawFiles);
    if (!rawFiles.empty()) f.rawFile = rawFiles.front().name;
    return f;
}

template <class T>
std::vector<uint8_t> bytesOf(const T& t) {
    std::vector<uint8_t> out(sizeof(T));
    std::memcpy(out.data(), &t, sizeof(T));
    return out;
}

void teardown(FixturePaths& f) {
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

class DIDTraitsTest : public ::testing::Test {
protected:
    FixturePaths f;
    void TearDown() override { teardown(f); }
};

// ---------------------------------------------------------------------------
// Runtime: DIDTraits names and sizes.
// ---------------------------------------------------------------------------
TEST(DIDTraitsRuntime, NamesAndSizes) {
    EXPECT_STREQ(DIDTraits<DID_INS_2>::name,        "DID_INS_2");
    EXPECT_STREQ(DIDTraits<DID_IMU>::name,          "DID_IMU");
    EXPECT_STREQ(DIDTraits<DID_BAROMETER>::name,    "DID_BAROMETER");
    EXPECT_EQ(DIDTraits<DID_INS_2>::expected_size,    sizeof(ins_2_t));
    EXPECT_EQ(DIDTraits<DID_IMU>::expected_size,      sizeof(imu_t));
    EXPECT_EQ(DIDTraits<DID_PIMU>::expected_size,     sizeof(pimu_t));
    EXPECT_EQ(DIDTraits<DID_GNSS1_POS>::expected_size, sizeof(gnss_pos_t));
}

// ---------------------------------------------------------------------------
// records<DID_INS_2>() yields const ins_2_t& with the values written.
// ---------------------------------------------------------------------------
TEST_F(DIDTraitsTest, TypedRangeOverInsRecords) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;

    auto makeIns = [](double tow, float yawQuatW, double lat) {
        ins_2_t s{};
        s.week       = 2300;
        s.timeOfWeek = tow;
        s.qn2b[0]    = yawQuatW;
        s.lla[0]     = lat;
        s.lla[1]     = -111.0;
        s.lla[2]     = 1400.0;
        return s;
    };
    const ins_2_t a = makeIns(100.0, 1.0f, 40.0);
    const ins_2_t b = makeIns(100.1, 0.5f, 40.5);
    const ins_2_t c = makeIns(100.2, 0.0f, 41.0);
    recs.emplace_back(DID_INS_2, bytesOf(a));
    recs.emplace_back(DID_INS_2, bytesOf(b));
    recs.emplace_back(DID_INS_2, bytesOf(c));

    f = buildFixture("ins", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value()) << reader.error().message;

    auto range = reader->records<DID_INS_2>();
    ASSERT_EQ(range.size(), 3u);
    auto it = range.begin();
    EXPECT_DOUBLE_EQ((*it).lla[0], 40.0);  ++it;
    EXPECT_DOUBLE_EQ((*it).lla[0], 40.5);  ++it;
    EXPECT_DOUBLE_EQ((*it).lla[0], 41.0);  ++it;
    EXPECT_EQ(it, range.end());

    // arrow operator yields const T*.
    auto first = range.begin();
    EXPECT_FLOAT_EQ(first->qn2b[0], 1.0f);

    // timestamps survive too.
    EXPECT_EQ(range.begin().timestamp().value, static_cast<uint64_t>(100.0 * 1000));
}

// ---------------------------------------------------------------------------
// Range-for over the typed range yields const T&.
// ---------------------------------------------------------------------------
TEST_F(DIDTraitsTest, RangeForYieldsConstRef) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    imu_t s{};
    s.time = 1.0;
    s.I.acc[0] = 0.1f; s.I.acc[1] = 0.2f; s.I.acc[2] = 9.8f;
    recs.emplace_back(DID_IMU, bytesOf(s));

    f = buildFixture("imu", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());

    std::size_t count = 0;
    for (const imu_t& imu : reader->records<DID_IMU>()) {
        EXPECT_FLOAT_EQ(imu.I.acc[2], 9.8f);
        ++count;
    }
    EXPECT_EQ(count, 1u);
}

// ---------------------------------------------------------------------------
// Empty range when the DID is absent.
// ---------------------------------------------------------------------------
TEST_F(DIDTraitsTest, EmptyRangeForAbsentDid) {
    // Fixture only contains DID_IMU; querying DID_GNSS1_POS returns empty.
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    imu_t s{};
    recs.emplace_back(DID_IMU, bytesOf(s));
    f = buildFixture("absent", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());

    auto gpsRange = reader->records<DID_GNSS1_POS>();
    EXPECT_TRUE(gpsRange.empty());
    EXPECT_EQ(gpsRange.size(), 0u);

    auto imuRange = reader->records<DID_IMU>();
    EXPECT_FALSE(imuRange.empty());
    EXPECT_EQ(imuRange.size(), 1u);
}

// ---------------------------------------------------------------------------
// Free-function spelling (records_typed<DID>(reader)) works the same.
// ---------------------------------------------------------------------------
TEST_F(DIDTraitsTest, FreeFunctionSpelling) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;
    barometer_t b{};
    b.bar = 101325.0f;
    b.barTemp = 22.5f;
    recs.emplace_back(DID_BAROMETER, bytesOf(b));
    f = buildFixture("free_fn", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());

    auto range = records_typed<DID_BAROMETER>(*reader);
    ASSERT_EQ(range.size(), 1u);
    EXPECT_FLOAT_EQ(range.begin()->bar, 101325.0f);
}

// ---------------------------------------------------------------------------
// Multi-DID: write a mixed stream, ensure each typed range filters correctly.
// ---------------------------------------------------------------------------
TEST_F(DIDTraitsTest, MultipleDidsCoexist) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> recs;

    ins_2_t ins{};
    ins.lla[0] = 40.0;
    ins.qn2b[0] = 1.0f;

    imu_t imu{};
    imu.time = 1.0;
    imu.I.acc[2] = 9.8f;

    recs.emplace_back(DID_INS_2,  bytesOf(ins));
    recs.emplace_back(DID_IMU,    bytesOf(imu));
    recs.emplace_back(DID_INS_2,  bytesOf(ins));
    recs.emplace_back(DID_IMU,    bytesOf(imu));
    recs.emplace_back(DID_INS_2,  bytesOf(ins));

    f = buildFixture("mixed", recs);
    ASSERT_FALSE(f.rawFile.empty());
    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());

    EXPECT_EQ(reader->records<DID_INS_2>().size(), 3u);
    EXPECT_EQ(reader->records<DID_IMU>().size(),   2u);
}

} // namespace
