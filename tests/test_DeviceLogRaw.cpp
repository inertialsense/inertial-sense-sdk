/**
 * @file test_DeviceLogRaw.cpp
 * @brief cDeviceLogRaw read-back of ISB data packets, in particular partial-DID packets (non-zero offset).
 *
 * For a partial-DID packet, rxPkt.data.ptr points at the payload and dataHdr.offset is where that payload belongs inside
 * the DID (the p_data_buf_t convention). ReadData() must return the payload at buf[0..size) with hdr.offset carried through.
 *
 * Reported by Petr Belohlavek (@petrbel), PR #1382.
 */

#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

#include "ISLogger.h"
#include "ISComm.h"
#include "ISFileManager.h"

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kHwId  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kSerial = 777019u;

/** One ISB data packet to write: @p size payload bytes starting at @p payload, destined for @p offset within @p did. */
struct PacketSpec {
    uint16_t did;
    uint16_t offset;
    std::vector<uint8_t> payload;
};

/** A data set as read back from the log. */
struct ReadBack {
    p_data_hdr_t hdr;
    std::vector<uint8_t> bytes;     //!< buf[0..hdr.size)
};

std::vector<uint8_t> sequence(uint8_t first, size_t n) {
    std::vector<uint8_t> v(n);
    for (size_t i = 0; i < n; i++) v[i] = (uint8_t)(first + i);
    return v;
}

/** Encodes @p specs as ISB data packets, back to back. */
std::vector<uint8_t> encode(const std::vector<PacketSpec>& specs) {
    std::vector<uint8_t> stream;
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_instance_t comm;
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    for (const auto& s : specs) {
        std::vector<uint8_t> pkt(ISB_MAX_PKT_BODY_SIZE + 64);
        std::vector<uint8_t> data(s.payload);
        int n = is_comm_data_to_buf(pkt.data(), (uint32_t)pkt.size(), &comm, s.did, (uint16_t)s.payload.size(), s.offset, data.data());
        EXPECT_GT(n, 0);
        if (n > 0) stream.insert(stream.end(), pkt.begin(), pkt.begin() + n);
    }
    return stream;
}

/** Writes @p stream through cISLogger as a raw log in a fresh directory and returns the directory. */
fs::path writeRawLog(const std::vector<uint8_t>& stream, const std::string& name) {
    fs::path dir = fs::temp_directory_path() / ("test_DeviceLogRaw_" + name);
    fs::remove_all(dir);
    fs::create_directories(dir);
    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    EXPECT_TRUE(logger.InitSave(dir.string(), opts));
    auto dev = logger.registerDevice(kHwId, kSerial);
    EXPECT_NE(dev, nullptr);
    logger.EnableLogging(true);
    logger.LogData(dev, (int)stream.size(), stream.data());
    logger.CloseAllFiles();
    return dir;
}

/** Reads every ISB data set back from the raw log in @p dir. */
std::vector<ReadBack> readAll(const fs::path& dir) {
    std::vector<ReadBack> out;
    cISLogger reader;
    if (!reader.LoadFromDirectory(dir.string(), cISLogger::LOGTYPE_RAW)) return out;
    while (p_data_buf_t* d = reader.ReadData(0)) {
        ReadBack r;
        r.hdr = d->hdr;
        r.bytes.assign(d->buf, d->buf + std::min<size_t>(d->hdr.size, sizeof(d->buf)));
        out.push_back(std::move(r));
    }
    return out;
}

std::vector<ReadBack> roundTrip(const std::vector<PacketSpec>& specs, const std::string& name) {
    fs::path dir = writeRawLog(encode(specs), name);
    auto out = readAll(dir);
    fs::remove_all(dir);
    return out;
}

} // namespace

TEST(DeviceLogRaw, PartialDidOffsetReadsPayloadFromStart)
{
    auto got = roundTrip({ { DID_FLASH_CONFIG, 16, sequence(0x20, 8) } }, "a1");
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].hdr.id, DID_FLASH_CONFIG);
    EXPECT_EQ(got[0].hdr.offset, 16);
    EXPECT_EQ(got[0].hdr.size, 8);
    EXPECT_EQ(got[0].bytes, sequence(0x20, 8)) << "payload was read from the wrong position";
}

TEST(DeviceLogRaw, PartialDidOffsetZeroUnchanged)
{
    auto got = roundTrip({ { DID_FLASH_CONFIG, 0, sequence(0x40, 24) } }, "a2");
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].hdr.offset, 0);
    EXPECT_EQ(got[0].hdr.size, 24);
    EXPECT_EQ(got[0].bytes, sequence(0x40, 24));
}

TEST(DeviceLogRaw, LargeOffsetNoOutOfBoundsRead)
{
    // The parser accepts an offset only while offset + payloadSize <= MAX_MSG_SIZE_ISB (2048).  The largest offsets it lets through
    // used to be added to the source pointer, so the copy came from up to ~2 KB past the payload (other bytes of the parser buffer,
    // or past its end when the packet sits near the end of it).
    const uint16_t maxOffset = (uint16_t)(MAX_MSG_SIZE_ISB - (8 + 2));     // payloadSize includes the 2-byte offset field
    auto got = roundTrip({ { DID_FLASH_CONFIG, 2000, sequence(0x70, 8) }, { DID_FLASH_CONFIG, maxOffset, sequence(0x10, 8) } }, "a3");
    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(got[0].hdr.offset, 2000);
    EXPECT_EQ(got[0].bytes, sequence(0x70, 8));
    EXPECT_EQ(got[1].hdr.offset, maxOffset);
    EXPECT_EQ(got[1].bytes, sequence(0x10, 8));
}

TEST(DeviceLogRaw, OffsetBeyondParserLimitIsNotDelivered)
{
    // Not changed by the fix: the parser drops a packet whose offset + payload exceeds MAX_MSG_SIZE_ISB, so a 16-bit offset of
    // up to 65535 never reaches ReadData() (and so cannot drive a copy from far outside the parser buffer).
    auto got = roundTrip({ { DID_FLASH_CONFIG, 65000, sequence(0x70, 8) } }, "a3b");
    EXPECT_TRUE(got.empty());
}

TEST(DeviceLogRaw, PartialDidMultiPacketRoundTrip)
{
    const std::vector<PacketSpec> specs = {
        { DID_FLASH_CONFIG, 8,   sequence(0x01, 4) },
        { DID_INS_1,        0,   sequence(0x11, 40) },
        { DID_FLASH_CONFIG, 100, sequence(0x21, 33) },
        { DID_DEV_INFO,     4,   sequence(0x31, 12) },
        { DID_FLASH_CONFIG, 0,   sequence(0x41, 64) },
    };
    auto got = roundTrip(specs, "b1");
    ASSERT_EQ(got.size(), specs.size());
    for (size_t i = 0; i < specs.size(); i++) {
        EXPECT_EQ(got[i].hdr.id, specs[i].did) << "packet " << i;
        EXPECT_EQ(got[i].hdr.offset, specs[i].offset) << "packet " << i;
        EXPECT_EQ(got[i].hdr.size, specs[i].payload.size()) << "packet " << i;
        EXPECT_EQ(got[i].bytes, specs[i].payload) << "packet " << i;
    }
}

TEST(DeviceLogRaw, ZeroSizePayloadYieldsZeroSize)
{
    // An offset-only packet has no payload bytes: data.ptr is NULL and the read must not touch it.
    auto got = roundTrip({ { DID_FLASH_CONFIG, 12, {} }, { DID_FLASH_CONFIG, 12, sequence(0x50, 4) } }, "b3a");
    ASSERT_GE(got.size(), 1u);
    EXPECT_EQ(got.front().hdr.size, got.front().bytes.size());
    EXPECT_EQ(got.back().bytes, sequence(0x50, 4)) << "the packet after an empty one was not read correctly";
}

TEST(DeviceLogRaw, MaxSizePayloadRoundTrips)
{
    // ISB_MAX_DATASET_SIZE == ISB_MAX_PKT_BODY_SIZE, so the parser cannot deliver a payload larger than m_pData.buf;
    // the size guard in ReadPacketFromChunk() is defensive.  The boundary case (largest payload that fits) must still round trip.
    const size_t maxPayload = ISB_MAX_PKT_BODY_SIZE - 2;    // 2 bytes of the body carry the offset
    auto got = roundTrip({ { DID_FLASH_CONFIG, 6, sequence(0x00, maxPayload) } }, "b3b");
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].hdr.size, maxPayload);
    EXPECT_EQ(got[0].bytes, sequence(0x00, maxPayload));
}

// B2: no real .raw fixture with partial-DID sets is checked in (gap; a follow-up ticket tracks capturing one).  When
// IS_TEST_RAW_PARTIAL_DID points at a directory holding such a log, every data set read back must match an independent
// parse of the same file with the ISComm parser (payload at buf[0..size), offset in hdr.offset).
TEST(DeviceLogRaw, PartialDidRealFixture)
{
    const char* env = std::getenv("IS_TEST_RAW_PARTIAL_DID");
    if (!env || !*env) GTEST_SKIP() << "IS_TEST_RAW_PARTIAL_DID not set";

    std::vector<ISFileManager::file_info_t> raws;
    ISFileManager::GetAllFilesInDirectory(env, true, "\\.raw$", raws);
    ASSERT_EQ(raws.size(), 1u) << "expected exactly one .raw file in " << env;
    std::ifstream in(raws[0].name, std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    std::vector<ReadBack> expected;
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_instance_t comm;
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    for (uint8_t b : bytes) {
        protocol_type_t t = is_comm_parse_byte(&comm, b);
        if (t == _PTYPE_INERTIAL_SENSE_DATA || t == _PTYPE_INERTIAL_SENSE_CMD) {
            ReadBack r;
            r.hdr = comm.rxPkt.dataHdr;
            if (comm.rxPkt.data.ptr) r.bytes.assign(comm.rxPkt.data.ptr, comm.rxPkt.data.ptr + comm.rxPkt.dataHdr.size);
            expected.push_back(std::move(r));
        }
    }
    ASSERT_FALSE(expected.empty());

    auto got = readAll(env);
    ASSERT_EQ(got.size(), expected.size());
    size_t partial = 0;
    for (size_t i = 0; i < got.size(); i++) {
        EXPECT_EQ(got[i].hdr.id, expected[i].hdr.id) << "record " << i;
        EXPECT_EQ(got[i].hdr.offset, expected[i].hdr.offset) << "record " << i;
        EXPECT_EQ(got[i].bytes, expected[i].bytes) << "record " << i;
        if (expected[i].hdr.offset != 0) partial++;
    }
    EXPECT_GT(partial, 0u) << "the fixture contains no partial-DID records, so it does not exercise the fix";
}
