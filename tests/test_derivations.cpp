/**
 * @file test_derivations.cpp
 * @brief D-09 / SN-7899 — built-in derivation catalog tests.
 *
 * Strategy: hand-build a tiny ISLogReader fixture by writing a few
 * INS_2 + IMU records via the cISLogger framework, then run each
 * derivation against it and assert numerical results within
 * documented tolerances. Numerical references are computed locally
 * (Hamilton quat → ZYX Euler, lla→NED via `lla2ned_d` itself) so
 * the tests don't depend on a Python reference at this story's
 * scope (deferred to D-11).
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // first

#include "derivations/DerivationRegistry.h"
#include "DeviceLog.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "ISLogReader.h"
#include "ISPose.h"
#include "ISEarth.h"
#include "data_sets.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <list>
#include <string>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
using namespace inertial_sense::derivations;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 999333u;

struct FixturePaths {
    fs::path directory;
    fs::path rawFile;
};

FixturePaths buildFixture(const std::string& hint,
                          const std::vector<ins_2_t>& insSamples,
                          const std::vector<imu_t>&   imuSamples) {
    FixturePaths f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "/tmp/test_derivations_%s_%d_%ld",
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

    auto wrap = [](uint32_t did, void* payload, std::size_t size) {
        std::vector<uint8_t> out;
        is_comm_instance_t comm{};
        uint8_t buf[1024];
        is_comm_init(&comm, buf, sizeof(buf), NULL);
        uint8_t pkt[2048];
        const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm,
                                          static_cast<uint16_t>(did),
                                          static_cast<uint16_t>(size), 0,
                                          payload);
        if (n > 0) out.assign(pkt, pkt + n);
        return out;
    };

    for (const auto& s : insSamples) {
        ins_2_t copy = s;
        auto bytes = wrap(DID_INS_2, &copy, sizeof(copy));
        if (!bytes.empty()) {
            logger.LogData(devLogger, bytes.size(), bytes.data());
        }
    }
    for (const auto& s : imuSamples) {
        imu_t copy = s;
        auto bytes = wrap(DID_IMU, &copy, sizeof(copy));
        if (!bytes.empty()) {
            logger.LogData(devLogger, bytes.size(), bytes.data());
        }
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> rawFiles;
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true,
                                          "\\.raw$", rawFiles);
    if (!rawFiles.empty()) f.rawFile = rawFiles.front().name;
    return f;
}

void teardown(FixturePaths& f) {
    if (!f.directory.empty() && fs::exists(f.directory)) {
        ISFileManager::DeleteDirectory(f.directory.string());
    }
}

// Build an INS_2 sample at a given timeOfWeek with quat / uvw / lla.
ins_2_t makeIns(double tow, const float quat[4], const float uvw[3],
                const double lla[3]) {
    ins_2_t s{};
    s.week       = 2300;
    s.timeOfWeek = tow;
    s.insStatus  = 0;
    s.hdwStatus  = 0;
    std::memcpy(s.qn2b, quat, sizeof(s.qn2b));
    std::memcpy(s.uvw,  uvw,  sizeof(s.uvw));
    std::memcpy(s.lla,  lla,  sizeof(s.lla));
    return s;
}

imu_t makeImu(double t, const float pqr[3], const float acc[3]) {
    imu_t s{};
    s.time = t;
    s.status = 0;
    std::memcpy(s.I.pqr, pqr, sizeof(s.I.pqr));
    std::memcpy(s.I.acc, acc, sizeof(s.I.acc));
    return s;
}

class DerivationTest : public ::testing::Test {
protected:
    FixturePaths f;
    void TearDown() override { teardown(f); }
};

// ---------------------------------------------------------------------------
// Registry sanity: every advertised derivation is present and reachable.
// ---------------------------------------------------------------------------
TEST(DerivationRegistry, AllV1EntriesPresent) {
    const auto& reg = DerivationRegistry::instance();
    const std::vector<std::string> expected = {
        "quat_to_euler",
        "heading_from_quat",
        "lla_to_ned@first_fix",
        "lla_to_ned@user_origin",
        "velocity_magnitude",
        "accel_magnitude",
        "rate_magnitude",
        "body_accel_to_ned_accel",
        "body_rate_to_euler_rate",
        "heading_from_velocity",
    };
    EXPECT_EQ(reg.size(), expected.size());
    for (const auto& name : expected) {
        auto d = reg.get(name);
        ASSERT_TRUE(d.has_value()) << "missing entry: " << name;
        EXPECT_NE((*d)->evaluate, nullptr) << name;
        EXPECT_FALSE((*d)->outputUnit.empty()) << name;
    }
}

TEST(DerivationRegistry, UnknownNameReturnsNullopt) {
    const auto& reg = DerivationRegistry::instance();
    EXPECT_FALSE(reg.get("definitely_not_a_real_derivation").has_value());
}

// ---------------------------------------------------------------------------
// quat_to_euler: identity quat → all zeros; +90° yaw quat → yaw == π/2.
// ---------------------------------------------------------------------------
TEST_F(DerivationTest, QuatToEulerIdentityAndYaw) {
    const float identity[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    // Hamilton (W,X,Y,Z) for +90° rotation about Z: (cos(45°), 0, 0, sin(45°))
    const float yaw90  [4] = { 0.7071068f, 0.0f, 0.0f, 0.7071068f };
    const float uvw[3]     = { 0.0f, 0.0f, 0.0f };
    const double lla[3]    = { 40.0, -111.0, 1400.0 };

    f = buildFixture("quat", {
        makeIns(100.0, identity, uvw, lla),
        makeIns(100.1, yaw90,    uvw, lla),
    }, {});
    ASSERT_FALSE(f.rawFile.empty());

    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());
    DerivationContext ctx{ reader.value() };

    auto d = DerivationRegistry::instance().get("quat_to_euler");
    ASSERT_TRUE(d.has_value());
    auto r = (*d)->evaluate(ctx);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    ASSERT_EQ(r->samples.size(), 2u);

    EXPECT_NEAR(r->samples[0].values[0], 0.0, 1e-5);  // roll
    EXPECT_NEAR(r->samples[0].values[1], 0.0, 1e-5);  // pitch
    EXPECT_NEAR(r->samples[0].values[2], 0.0, 1e-5);  // yaw

    EXPECT_NEAR(r->samples[1].values[0], 0.0,    1e-5);
    EXPECT_NEAR(r->samples[1].values[1], 0.0,    1e-5);
    EXPECT_NEAR(r->samples[1].values[2], M_PI/2, 1e-5);
}

// ---------------------------------------------------------------------------
// velocity_magnitude / accel_magnitude / rate_magnitude.
// ---------------------------------------------------------------------------
TEST_F(DerivationTest, MagnitudesComputeNorms) {
    const float quat[4]    = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float uvw[3]     = { 3.0f, 4.0f, 0.0f };  // ‖·‖ = 5
    const double lla[3]    = { 0.0, 0.0, 0.0 };
    const float pqr[3]     = { 0.6f, 0.8f, 0.0f };  // ‖·‖ = 1.0
    const float acc[3]     = { 1.0f, 2.0f, 2.0f };  // ‖·‖ = 3

    f = buildFixture("mags",
                     { makeIns(100.0, quat, uvw, lla) },
                     { makeImu(0.5, pqr, acc) });
    ASSERT_FALSE(f.rawFile.empty());

    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());
    DerivationContext ctx{ reader.value() };

    {
        auto d = DerivationRegistry::instance().get("velocity_magnitude");
        ASSERT_TRUE(d.has_value());
        auto r = (*d)->evaluate(ctx);
        ASSERT_TRUE(r.has_value()) << r.error().message;
        ASSERT_EQ(r->samples.size(), 1u);
        EXPECT_NEAR(r->samples[0].values[0], 5.0, 1e-5);
    }
    {
        auto d = DerivationRegistry::instance().get("accel_magnitude");
        ASSERT_TRUE(d.has_value());
        auto r = (*d)->evaluate(ctx);
        ASSERT_TRUE(r.has_value()) << r.error().message;
        ASSERT_EQ(r->samples.size(), 1u);
        EXPECT_NEAR(r->samples[0].values[0], 3.0, 1e-5);
    }
    {
        auto d = DerivationRegistry::instance().get("rate_magnitude");
        ASSERT_TRUE(d.has_value());
        auto r = (*d)->evaluate(ctx);
        ASSERT_TRUE(r.has_value()) << r.error().message;
        ASSERT_EQ(r->samples.size(), 1u);
        EXPECT_NEAR(r->samples[0].values[0], 1.0, 1e-5);
    }
}

// ---------------------------------------------------------------------------
// lla_to_ned@first_fix: first record is the origin (0,0,0); second is
// 1° North → expect ~111 km positive North, ~0 East, ~0 Down.
// ---------------------------------------------------------------------------
TEST_F(DerivationTest, LlaToNedFirstFix) {
    const float quat[4]    = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float uvw[3]     = { 0.0f, 0.0f, 0.0f };
    const double origin[3] = { 40.0,  -111.0, 1400.0 };
    const double oneDegN[3]= { 41.0,  -111.0, 1400.0 };

    f = buildFixture("lla_first", {
        makeIns(100.0, quat, uvw, origin),
        makeIns(100.1, quat, uvw, oneDegN),
    }, {});
    ASSERT_FALSE(f.rawFile.empty());

    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());
    DerivationContext ctx{ reader.value() };

    auto d = DerivationRegistry::instance().get("lla_to_ned@first_fix");
    ASSERT_TRUE(d.has_value());
    auto r = (*d)->evaluate(ctx);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    ASSERT_EQ(r->samples.size(), 2u);

    EXPECT_NEAR(r->samples[0].values[0], 0.0, 0.5);
    EXPECT_NEAR(r->samples[0].values[1], 0.0, 0.5);
    EXPECT_NEAR(r->samples[0].values[2], 0.0, 0.5);

    EXPECT_GT(r->samples[1].values[0],  100000.0);  // ≥ 100 km North
    EXPECT_LT(r->samples[1].values[0],  120000.0);  // ≤ 120 km
    EXPECT_NEAR(r->samples[1].values[1], 0.0, 100.0);
    EXPECT_NEAR(r->samples[1].values[2], 0.0, 5.0);
}

// ---------------------------------------------------------------------------
// lla_to_ned@user_origin: missing parameter → InvalidArgument.
// ---------------------------------------------------------------------------
TEST_F(DerivationTest, LlaToNedUserOriginNeedsParam) {
    const float quat[4]  = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float uvw[3]   = { 0.0f, 0.0f, 0.0f };
    const double lla[3]  = { 40.0, -111.0, 1400.0 };
    f = buildFixture("lla_user", { makeIns(100.0, quat, uvw, lla) }, {});
    ASSERT_FALSE(f.rawFile.empty());

    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());
    DerivationContext ctx{ reader.value() };
    auto d = DerivationRegistry::instance().get("lla_to_ned@user_origin");
    ASSERT_TRUE(d.has_value());

    auto r = (*d)->evaluate(ctx);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::InvalidArgument);

    ctx.setParam("origin_lla", "39.9,-111.0,1400.0");
    r = (*d)->evaluate(ctx);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    ASSERT_EQ(r->samples.size(), 1u);
    EXPECT_GT(r->samples[0].values[0], 0.0);  // sample is North of origin
}

// ---------------------------------------------------------------------------
// heading_from_quat: identity quat → 0; +90° yaw quat → π/2.
// ---------------------------------------------------------------------------
TEST_F(DerivationTest, HeadingFromQuat) {
    const float identity[4]= { 1.0f, 0.0f, 0.0f, 0.0f };
    const float yaw90  [4] = { 0.7071068f, 0.0f, 0.0f, 0.7071068f };
    const float uvw[3]     = { 0.0f, 0.0f, 0.0f };
    const double lla[3]    = { 0.0, 0.0, 0.0 };

    f = buildFixture("hdg_q", {
        makeIns(100.0, identity, uvw, lla),
        makeIns(100.1, yaw90,    uvw, lla),
    }, {});
    ASSERT_FALSE(f.rawFile.empty());

    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());
    DerivationContext ctx{ reader.value() };
    auto d = DerivationRegistry::instance().get("heading_from_quat");
    ASSERT_TRUE(d.has_value());

    auto r = (*d)->evaluate(ctx);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    ASSERT_EQ(r->samples.size(), 2u);
    EXPECT_NEAR(r->samples[0].values[0], 0.0,    1e-5);
    EXPECT_NEAR(r->samples[1].values[0], M_PI/2, 1e-5);

    // wrap=positive — same numeric values for inputs in [0, π).
    ctx.setParam("wrap", "positive");
    auto r2 = (*d)->evaluate(ctx);
    ASSERT_TRUE(r2.has_value());
    EXPECT_NEAR(r2->samples[0].values[0], 0.0,    1e-5);
    EXPECT_NEAR(r2->samples[1].values[0], M_PI/2, 1e-5);
}

// ---------------------------------------------------------------------------
// body_accel_to_ned_accel: identity quat + body acc (0,0,9.80665) →
// NED (0, 0, 9.80665). With include_gravity=false → (0, 0, 0).
// ---------------------------------------------------------------------------
TEST_F(DerivationTest, BodyAccelToNedGravityToggle) {
    const float identity[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float uvw[3]      = { 0.0f, 0.0f, 0.0f };
    const double lla[3]     = { 0.0, 0.0, 0.0 };
    const float pqr[3]      = { 0.0f, 0.0f, 0.0f };
    const float accDown[3]  = { 0.0f, 0.0f, 9.80665f };

    f = buildFixture("body2ned", {
        makeIns(100.0, identity, uvw, lla),
    }, {
        makeImu(100.0, pqr, accDown),
    });
    ASSERT_FALSE(f.rawFile.empty());

    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());
    DerivationContext ctx{ reader.value() };
    auto d = DerivationRegistry::instance().get("body_accel_to_ned_accel");
    ASSERT_TRUE(d.has_value());

    auto withG = (*d)->evaluate(ctx);
    ASSERT_TRUE(withG.has_value()) << withG.error().message;
    ASSERT_EQ(withG->samples.size(), 1u);
    EXPECT_NEAR(withG->samples[0].values[0], 0.0,    1e-5);
    EXPECT_NEAR(withG->samples[0].values[1], 0.0,    1e-5);
    EXPECT_NEAR(withG->samples[0].values[2], 9.80665, 1e-4);

    ctx.setParam("include_gravity", "false");
    auto noG = (*d)->evaluate(ctx);
    ASSERT_TRUE(noG.has_value());
    ASSERT_EQ(noG->samples.size(), 1u);
    EXPECT_NEAR(noG->samples[0].values[2], 0.0, 1e-4);
}

// ---------------------------------------------------------------------------
// Missing-input error: call quat_to_euler on a fixture with only IMU.
// ---------------------------------------------------------------------------
TEST_F(DerivationTest, MissingInputProducesError) {
    const float pqr[3] = { 0.0f, 0.0f, 0.0f };
    const float acc[3] = { 0.0f, 0.0f, 9.80665f };
    f = buildFixture("missing", {}, { makeImu(100.0, pqr, acc) });
    ASSERT_FALSE(f.rawFile.empty());

    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());
    DerivationContext ctx{ reader.value() };
    auto d = DerivationRegistry::instance().get("quat_to_euler");
    ASSERT_TRUE(d.has_value());

    auto r = (*d)->evaluate(ctx);
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::NotFound);
}

// ---------------------------------------------------------------------------
// body_rate_to_euler_rate: identity quat → output rates equal input rates.
// ---------------------------------------------------------------------------
TEST_F(DerivationTest, BodyRateToEulerRateIdentity) {
    const float identity[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float uvw[3]      = { 0.0f, 0.0f, 0.0f };
    const double lla[3]     = { 0.0, 0.0, 0.0 };
    const float pqr[3]      = { 0.1f, 0.2f, 0.3f };
    const float acc[3]      = { 0.0f, 0.0f, 9.80665f };

    f = buildFixture("ber", {
        makeIns(100.0, identity, uvw, lla),
    }, {
        makeImu(100.0, pqr, acc),
    });
    ASSERT_FALSE(f.rawFile.empty());

    auto reader = ISLogReader::openSegment(f.rawFile);
    ASSERT_TRUE(reader.has_value());
    DerivationContext ctx{ reader.value() };
    auto d = DerivationRegistry::instance().get("body_rate_to_euler_rate");
    ASSERT_TRUE(d.has_value());

    auto r = (*d)->evaluate(ctx);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    ASSERT_EQ(r->samples.size(), 1u);
    // At zero roll/pitch the transform reduces to identity.
    EXPECT_NEAR(r->samples[0].values[0], 0.1, 1e-5);
    EXPECT_NEAR(r->samples[0].values[1], 0.2, 1e-5);
    EXPECT_NEAR(r->samples[0].values[2], 0.3, 1e-5);
}

} // namespace
