/**
 * @file test_log_reader_devinfo.cpp
 * @brief Coverage for how `ISLogReader` derives a segment's identity from `dev_info_t` records.
 *
 * Three DIDs carry an identical `dev_info_t`: `DID_DEV_INFO` is the logging device's own record, while
 * `DID_GPX_DEV_INFO` and `DID_EVB_DEV_INFO` describe an attached peripheral. They are NOT interchangeable as an
 * identity source, and these tests pin that ranking.
 *
 * @note SN-8445 / SN-8463. Written after a regression that shipped with no coverage here at all: broadening the
 *       accepted DID set to fix GPX-only logs made identity first-match-wins, so two rollover segments of one mixed
 *       IMX+GPX capture derived two different device ids and `ISDeviceLog::fromSegments` rejected the log as
 *       Corrupted. It surfaced two repositories downstream, in imx#1648's goldenlogs suite, rather than here.
 *
 * Deliberately portable (no `::getpid()`, no hard-coded `/tmp`) so it runs on Windows too — the regression failed on
 * both platforms, so the guard has to exist on both.
 */

#include <gtest/gtest.h>

#include "ISLogReader.h"
#include "ISDeviceLog.h"
#include "ISComm.h"
#include "data_sets.h"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint32_t kImxSerial = 509040u;    //!< The logging IMX in the goldenlogs capture that regressed.
constexpr uint32_t kGpxSerial = 808527428u; //!< The attached GPX's record from that same capture.

/**
 * @brief Builds a `dev_info_t` with the fields `ENCODE_DEV_INFO_TO_HDW_ID` reads.
 *
 * @param serial    Serial number to report.
 * @param hwType    `eIsHardwareType` value (3=IMX, 4=GPX).
 * @param major     Hardware version major.
 * @param minor     Hardware version minor.
 * @return A populated `dev_info_t`.
 */
dev_info_t makeDevInfo(uint32_t serial, uint8_t hwType, uint8_t major, uint8_t minor) {
    dev_info_t d{};
    d.serialNumber  = serial;
    d.hardwareType  = hwType;
    d.hardwareVer[0] = major;
    d.hardwareVer[1] = minor;
    d.firmwareVer[0] = 2;
    d.firmwareVer[1] = 4;
    return d;
}

/**
 * @brief Serialises one record as a real ISB packet (framing + header + payload + checksum).
 *
 * @param did   DID to stamp on the packet.
 * @param info  Payload.
 * @return The framed bytes, or empty on failure.
 * @note `deriveDeviceId` walks the raw byte stream with `is_comm_parse_byte`, so a payload-only fixture would be
 *       invisible to it. The packet must be genuinely framed.
 */
std::vector<uint8_t> framedPacket(uint16_t did, const dev_info_t& info) {
    is_comm_instance_t comm{};
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);

    uint8_t pkt[PKT_BUF_SIZE];
    dev_info_t payload = info;
    const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm, did,
                                      static_cast<uint16_t>(sizeof(payload)), 0, &payload);
    if (n <= 0) return {};
    return std::vector<uint8_t>(pkt, pkt + n);
}

/**
 * @brief Writes a `.raw` segment containing the given (DID, dev_info) records in order.
 *
 * @param path  Destination file.
 * @param recs  Records to emit, in stream order.
 * @note Filenames here deliberately contain no "SN<digits>", so `deriveDeviceId`'s filename fallback yields nothing
 *       and cannot mask a failure in the byte-scan path these tests are about.
 */
void writeRawSegment(const fs::path& path, const std::vector<std::pair<uint16_t, dev_info_t>>& recs) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(out.good()) << "could not open " << path.string();
    for (const auto& [did, info] : recs) {
        const auto bytes = framedPacket(did, info);
        ASSERT_FALSE(bytes.empty()) << "failed to frame DID " << did;
        out.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
    }
    out.close();
}

/**
 * @brief Per-test scratch directory, cleaned up on teardown.
 */
class LogReaderDevInfoTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = fs::temp_directory_path() /
               ("is_devinfo_" + std::string(::testing::UnitTest::GetInstance()
                                                ->current_test_info()->name()));
        std::error_code ec;
        fs::remove_all(dir_, ec);
        ASSERT_TRUE(fs::create_directories(dir_, ec)) << ec.message();
    }
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }
    fs::path dir_;
};

}  // namespace

/**
 * The regression's direct cause: a peripheral record physically earlier in the stream must not outrank the logging
 * device's own record.
 */
TEST_F(LogReaderDevInfoTest, PrimaryDevInfoWinsOverPeripheralThatAppearsFirst) {
    const fs::path seg = dir_ / "seg_0001.raw";
    writeRawSegment(seg, {
        { DID_GPX_DEV_INFO, makeDevInfo(kGpxSerial, IS_HARDWARE_TYPE_GPX, 1, 0) },
        { DID_DEV_INFO,     makeDevInfo(kImxSerial, IS_HARDWARE_TYPE_IMX, 5, 0) },
    });

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value()) << "openSegment failed";
    EXPECT_EQ(r->deviceId(), kImxSerial);
    EXPECT_EQ(r->hdwId(), static_cast<uint16_t>(ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0)));
    ASSERT_TRUE(r->hasDevInfo());
    EXPECT_EQ(r->devInfo().serialNumber, kImxSerial);
}

/**
 * SN-8445's actual target: a GPX-only capture has no `DID_DEV_INFO`, so the peripheral record is the only identity
 * available and must still yield a hardware id — otherwise such devices render as "???-0.0::SN<serial>".
 */
TEST_F(LogReaderDevInfoTest, PeripheralDevInfoUsedWhenNoPrimaryPresent) {
    const fs::path seg = dir_ / "seg_0001.raw";
    writeRawSegment(seg, {
        { DID_GPX_DEV_INFO, makeDevInfo(500123u, IS_HARDWARE_TYPE_GPX, 1, 0) },
    });

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value()) << "openSegment failed";
    EXPECT_EQ(r->deviceId(), 500123u);
    EXPECT_EQ(r->hdwId(), static_cast<uint16_t>(ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, 1, 0)))
        << "a GPX-only log must still recover a hardware id (SN-8445)";
    EXPECT_TRUE(r->hasDevInfo());
}

/**
 * An unprovisioned unit or a partial-update record reports serial 0. That must not terminate the scan, or one stub
 * early in a segment hides every valid record behind it.
 */
TEST_F(LogReaderDevInfoTest, ZeroSerialRecordDoesNotHideALaterValidRecord) {
    const fs::path seg = dir_ / "seg_0001.raw";
    writeRawSegment(seg, {
        { DID_DEV_INFO, makeDevInfo(0u,         IS_HARDWARE_TYPE_IMX, 5, 0) },
        { DID_DEV_INFO, makeDevInfo(kImxSerial, IS_HARDWARE_TYPE_IMX, 5, 0) },
    });

    auto r = ISLogReader::openSegment(seg);
    ASSERT_TRUE(r.has_value()) << "openSegment failed";
    EXPECT_EQ(r->deviceId(), kImxSerial);
    EXPECT_EQ(r->hdwId(), static_cast<uint16_t>(ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0)));
}

/**
 * The regression at the level it actually broke. A mixed capture writes both records per segment, in an order that
 * varies between rollover files. Every segment must still agree on the device id, or `fromSegments` rejects the whole
 * log as Corrupted — which is exactly what happened to
 * `gpx/gpx1/intellian_ut/.../20250313_054706` in imx#1648.
 */
TEST_F(LogReaderDevInfoTest, MixedSegmentsInEitherOrderAgreeOnDeviceId) {
    const dev_info_t imx = makeDevInfo(kImxSerial, IS_HARDWARE_TYPE_IMX, 5, 0);
    const dev_info_t gpx = makeDevInfo(kGpxSerial, IS_HARDWARE_TYPE_GPX, 1, 0);

    const fs::path segA = dir_ / "seg_0001.raw";   // primary first
    const fs::path segB = dir_ / "seg_0002.raw";   // peripheral first — the ordering that regressed
    writeRawSegment(segA, { { DID_DEV_INFO, imx }, { DID_GPX_DEV_INFO, gpx } });
    writeRawSegment(segB, { { DID_GPX_DEV_INFO, gpx }, { DID_DEV_INFO, imx } });

    auto a = ISLogReader::openSegment(segA);
    auto b = ISLogReader::openSegment(segB);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    ASSERT_EQ(a->deviceId(), b->deviceId())
        << "segments of one log disagreed on device id; fromSegments will reject the log";

    auto log = ISDeviceLog::fromSegments({ segA, segB });
    ASSERT_TRUE(log.has_value())
        << "fromSegments rejected a mixed IMX+GPX log — the SN-8445 regression";
    EXPECT_EQ(log->deviceId(), kImxSerial);
}
