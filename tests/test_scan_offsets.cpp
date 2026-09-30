/**
 * @file test_scan_offsets.cpp
 * @brief SN-8765 — does the `.idx` rebuild write offsets that are actually packet starts?
 *
 * `buildIndexFromScan` used to feed `is_comm_parse_byte` one byte at a time and assume that a
 * packet emitted while feeding byte `i` therefore ended at `i` and began wherever the previous
 * emit ended. Both halves are false: `is_comm_reset_parser` REWINDS `rxBuf.scan` to `rxBuf.head`
 * on a parse error, so buffered bytes are re-scanned and a later one-byte call can complete a
 * whole packet out of the backlog without the new byte contributing anything.
 *
 * These tests are written so the central one FAILS against that old implementation — that is the
 * point of them. Measured on `sn8339_log/20260729_003722` segment 0, a rebuild under the old code
 * produced 2,113 consecutive offset gaps shorter than a minimum ISB packet and 316 offsets (of
 * 4,179 sampled) that were not packet starts at all; under the fix both are zero and the record
 * count rises from 58,500 to 58,521.
 *
 * Everything here is synthetic and portable (`std::filesystem` temp dirs, no `getpid()`), so it
 * runs on Windows CI too — see the NOTE in `tests/CMakeLists.txt` about which tests are excluded.
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // must precede ISComm-pulling headers

#include "ISComm.h"
#include "ISDataMappings.h"
#include "ISLogIndex.h"
#include "ISLogReader.h"
#include "data_sets.h"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

/**
 * @brief The smallest a framed ISB packet can possibly be: preamble + header + checksum, no
 *        payload.
 *
 * A consecutive offset gap below this is arithmetically impossible for two real packet starts, so
 * it is a self-contradiction in the sidecar rather than a judgement call about what "correct"
 * means.
 */
constexpr uint64_t kMinIsbPacketBytes = 8;

//! Portable unique temp directory (POSIX + Windows CI). Mirrors `test_log_bounds.cpp`.
fs::path makeTempDir(const std::string& prefix) {
    static unsigned counter = 0;
    const fs::path d = fs::temp_directory_path() / (prefix + "_" + std::to_string(counter++));
    std::error_code ec;
    fs::remove_all(d, ec);
    fs::create_directories(d, ec);
    return d;
}

/**
 * @brief Appends one framed ISB packet to @p out using the SDK's own writer.
 *
 * Framed rather than payload-only on purpose: a `.raw` written as bare payloads parses as ZERO
 * records, which would make every assertion here vacuously true.
 */
void appendIsb(std::vector<uint8_t>& out, uint16_t did, uint16_t size, const void* payload) {
    is_comm_instance_t comm{};
    uint8_t            commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    uint8_t   pkt[PKT_BUF_SIZE];
    const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm, did, size, 0,
                                      const_cast<void*>(payload));
    ASSERT_GT(n, 0);
    out.insert(out.end(), pkt, pkt + n);
}

/**
 * @brief Appends a run of bytes the parser cannot frame.
 *
 * This is what makes the parser report an error and rewind its scan pointer, which is the
 * precondition for the drain. `0x11` is not any enabled protocol's start byte.
 */
void appendJunk(std::vector<uint8_t>& out, std::size_t bytes) {
    out.insert(out.end(), bytes, static_cast<uint8_t>(0x11));
}

//! Writes @p bytes to `<dir>/LOG_SN<serial>_..._0001.raw`, with NO sidecar, so opening it forces
//! a rebuild. The filename follows the logger's convention because the anchor cascade reads it.
fs::path writeRaw(const fs::path& dir, const std::vector<uint8_t>& bytes) {
    const fs::path raw = dir / "LOG_SN777001_20260101_000000_0001.raw";
    std::ofstream  out(raw, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
    out.close();
    return raw;
}

/**
 * @brief Re-parses one packet starting exactly at @p offset and reports its DID.
 *
 * The independent check that an offset really is a packet start: hand the parser a window that
 * BEGINS at the offset and require that the first packet it finds occupies the window from byte
 * zero. If the offset pointed mid-packet, or at junk, the parser locks on somewhere later and the
 * head position will not equal the packet size.
 *
 * @return The DID found, or `UINT32_MAX` when the offset is not a packet start.
 */
uint32_t didAtPacketStart(const uint8_t* base, std::size_t total, uint64_t offset) {
    if (offset >= total) return UINT32_MAX;
    std::vector<uint8_t> win(std::min<std::size_t>(total - offset, 4096));
    is_comm_instance_t   c{};
    is_comm_init(&c, win.data(), static_cast<int>(win.size()), nullptr);
    is_comm_enable_protocol(&c, _PTYPE_INERTIAL_SENSE_DATA);
    is_comm_enable_protocol(&c, _PTYPE_NMEA);
    is_comm_enable_protocol(&c, _PTYPE_RTCM3);
    is_comm_enable_protocol(&c, _PTYPE_UBLOX);
    std::memcpy(win.data(), base + offset, win.size());
    c.rxBuf.tail = win.data() + win.size();

    protocol_type_t pt;
    while ((pt = is_comm_parse(&c)) != _PTYPE_NONE) {
        if (pt == _PTYPE_PARSE_ERROR) return UINT32_MAX;
        const uint64_t headPos = static_cast<uint64_t>(c.rxBuf.head - win.data());
        if (headPos != c.rxPkt.size) return UINT32_MAX;   // did not start at byte 0
        if (pt != _PTYPE_INERTIAL_SENSE_DATA && pt != _PTYPE_INERTIAL_SENSE_CMD) return UINT32_MAX;
        return c.rxPkt.dataHdr.id;
    }
    return UINT32_MAX;
}

/**
 * @brief Asserts the two properties that define a correct rebuild.
 *
 * @param reader  A reader whose index was rebuilt by scan.
 * @param label   Included in failure messages.
 */
void expectEveryOffsetIsAPacketStart(const ISLogReader& reader, const char* label) {
    const auto bytes = reader.rawBytes();
    ASSERT_NE(bytes.first, nullptr) << label;

    uint64_t prev = 0;
    for (std::size_t k = 0; k < reader.recordCount(); ++k) {
        const auto     rv  = reader.recordAt(k);
        const uint64_t off = rv.offsetInFile();

        // (1) Internally consistent: two real packet starts cannot be closer together than the
        //     smallest packet that can exist.
        if (k > 0) {
            EXPECT_GE(off, prev + kMinIsbPacketBytes)
                << label << ": record " << k << " at offset " << off
                << " is less than a minimum ISB packet after the previous record at " << prev;
        }
        prev = off;

        // (2) Externally true: parsing from that offset finds this record's packet, starting at
        //     byte zero.
        EXPECT_EQ(didAtPacketStart(bytes.first, bytes.second, off), rv.did())
            << label << ": record " << k << " offset " << off << " is not the start of a "
            << "DID " << rv.did() << " packet";
    }
}

} // namespace

/**
 * @brief A clean, contiguous stream: the fix must change nothing.
 *
 * On a stream with no parse errors the parser never drains, so "byte after the previous emit" and
 * "packet start" are the same number. This is the no-regression half, and it is also why the
 * defect went unnoticed — an undamaged log indexes correctly either way. It therefore passes
 * against the old implementation by design; it is here to catch a regression, not to prove the
 * fix.
 */
TEST(ScanOffsets, CleanStreamIndexesContiguousPacketStarts) {
    const fs::path dir = makeTempDir("sn8765_clean");
    std::vector<uint8_t> bytes;

    pimu_t pimu{};
    ins_2_t ins{};
    for (int i = 0; i < 40; ++i) {
        pimu.time = 1.0 + i;
        appendIsb(bytes, DID_PIMU, sizeof(pimu), &pimu);
        ins.timeOfWeek = 100000.0 + i;
        appendIsb(bytes, DID_INS_2, sizeof(ins), &ins);
    }
    const fs::path raw = writeRaw(dir, bytes);

    auto reader = ISLogReader::openSegment(raw);
    ASSERT_TRUE(reader.has_value());
    EXPECT_EQ(reader->recordCount(), 80u);
    expectEveryOffsetIsAPacketStart(*reader, "clean");

    // First record starts at byte 0 and offsets ascend by the packet size, since nothing
    // intervenes.
    EXPECT_EQ(reader->recordAt(0).offsetInFile(), 0u);

    std::error_code ec;
    fs::remove_all(dir, ec);
}

/**
 * @brief THE NEGATIVE CONTROL — a junk run makes the parser drain, and the old code got it wrong.
 *
 * The junk forces a parse error, which rewinds the scan pointer; the back-to-back packets that
 * follow are then completed out of the backlog on consecutive one-byte calls. Under the old
 * implementation those records were stamped with offsets one byte apart, which this test rejects
 * on both counts. **If this test passes against `buildIndexFromScan` as it was before SN-8765,
 * the test is broken, not the implementation.**
 */
TEST(ScanOffsets, DrainAfterJunkStillYieldsRealPacketStarts) {
    const fs::path dir = makeTempDir("sn8765_drain");
    std::vector<uint8_t> bytes;

    pimu_t  pimu{};
    ins_2_t ins{};
    sys_params_t sys{};
    barometer_t baro{};
    magnetometer_t mag{};

    // A few clean packets first, so the failure is localised to the drained region rather than
    // the start of the file.
    pimu.time = 1.0;
    appendIsb(bytes, DID_PIMU, sizeof(pimu), &pimu);
    ins.timeOfWeek = 100000.0;
    appendIsb(bytes, DID_INS_2, sizeof(ins), &ins);

    // The junk run. Long enough that the parser walks a long way past before locking on, which is
    // what makes the drained packets' bogus offsets visibly wrong.
    appendJunk(bytes, 700);

    // Several back-to-back packets immediately after the junk. These are the ones the old code
    // mis-stamped: they complete out of the re-scanned backlog on consecutive calls.
    for (int i = 0; i < 12; ++i) {
        mag.time = 2.0 + i;
        appendIsb(bytes, DID_MAGNETOMETER, sizeof(mag), &mag);
        baro.time = 2.0 + i;
        appendIsb(bytes, DID_BAROMETER, sizeof(baro), &baro);
        pimu.time = 2.0 + i;
        appendIsb(bytes, DID_PIMU, sizeof(pimu), &pimu);
        sys.upTime = 2.0 + i;
        appendIsb(bytes, DID_SYS_PARAMS, sizeof(sys), &sys);
    }
    const fs::path raw = writeRaw(dir, bytes);

    auto reader = ISLogReader::openSegment(raw);
    ASSERT_TRUE(reader.has_value());
    // 2 leading + 48 after the junk. Asserted exactly: the old code also LOST records, so a
    // "greater than" bound would let a regression through.
    EXPECT_EQ(reader->recordCount(), 50u);
    expectEveryOffsetIsAPacketStart(*reader, "drain");

    std::error_code ec;
    fs::remove_all(dir, ec);
}

/**
 * @brief An offset is the packet's OWN start, not the byte after the previous emit.
 *
 * The two definitions differ by exactly the intervening junk. The live writer documents the
 * intended semantic as the packet's physical start position in the file
 * (`cDeviceLogRaw::SaveData`), so the rebuild must agree with it.
 */
TEST(ScanOffsets, OffsetIsThePacketStartNotTheByteAfterThePreviousEmit) {
    const fs::path dir = makeTempDir("sn8765_semantic");
    std::vector<uint8_t> bytes;

    pimu_t pimu{};
    pimu.time = 1.0;
    appendIsb(bytes, DID_PIMU, sizeof(pimu), &pimu);
    const std::size_t afterFirst = bytes.size();

    constexpr std::size_t kJunk = 37;
    appendJunk(bytes, kJunk);
    const std::size_t secondStart = bytes.size();

    ins_2_t ins{};
    ins.timeOfWeek = 100000.0;
    appendIsb(bytes, DID_INS_2, sizeof(ins), &ins);
    const fs::path raw = writeRaw(dir, bytes);

    auto reader = ISLogReader::openSegment(raw);
    ASSERT_TRUE(reader.has_value());
    ASSERT_EQ(reader->recordCount(), 2u);

    EXPECT_EQ(reader->recordAt(0).offsetInFile(), 0u);
    // The whole point: `secondStart`, not `afterFirst`. They differ by the 37 junk bytes.
    EXPECT_EQ(reader->recordAt(1).offsetInFile(), static_cast<uint64_t>(secondStart));
    EXPECT_NE(reader->recordAt(1).offsetInFile(), static_cast<uint64_t>(afterFirst));

    std::error_code ec;
    fs::remove_all(dir, ec);
}

/**
 * @brief A segment larger than one internal scan window indexes correctly across the boundaries.
 *
 * The scan works in bounded windows so a multi-MB segment does not cost its own size in scratch
 * memory, which means packets straddle window boundaries. A straddling packet must be indexed
 * exactly once, at its true offset — never dropped between two windows and never counted twice.
 * The file is deliberately sized past several window boundaries and the packet sizes are mixed so
 * the boundaries do not land on a repeating stride.
 *
 * **Scope honestly stated: this is a REGRESSION GUARD, not a proof.** It passes against the
 * pre-SN-8765 implementation too — that code had no windows at all, and a clean 800 KB stream
 * indexes correctly either way. What it can catch is a future change to the windowing that starts
 * dropping or duplicating a straddling packet. The tests that actually pin the defect are
 * `DrainAfterJunkStillYieldsRealPacketStarts` and
 * `OffsetIsThePacketStartNotTheByteAfterThePreviousEmit`, both of which fail against the old code.
 */
TEST(ScanOffsets, IndexesCorrectlyAcrossInternalWindowBoundaries) {
    const fs::path dir = makeTempDir("sn8765_windows");
    std::vector<uint8_t> bytes;
    bytes.reserve(900u * 1024u);

    pimu_t   pimu{};
    ins_2_t  ins{};
    barometer_t baro{};
    std::size_t expected = 0;
    // Past the 256 KB window a few times over.
    while (bytes.size() < 800u * 1024u) {
        pimu.time = 1.0 + expected;
        appendIsb(bytes, DID_PIMU, sizeof(pimu), &pimu);
        ins.timeOfWeek = 100000.0 + expected;
        appendIsb(bytes, DID_INS_2, sizeof(ins), &ins);
        baro.time = 1.0 + expected;
        appendIsb(bytes, DID_BAROMETER, sizeof(baro), &baro);
        expected += 3;
    }
    const fs::path raw = writeRaw(dir, bytes);

    auto reader = ISLogReader::openSegment(raw);
    ASSERT_TRUE(reader.has_value());
    EXPECT_EQ(reader->recordCount(), expected)
        << "a packet straddling a window boundary was dropped or double-counted";
    expectEveryOffsetIsAPacketStart(*reader, "windows");

    // Offsets strictly ascend across the whole file — a duplicated record from an overlapping
    // window re-scan would show up here as a repeat.
    for (std::size_t k = 1; k < reader->recordCount(); ++k) {
        ASSERT_GT(reader->recordAt(k).offsetInFile(), reader->recordAt(k - 1).offsetInFile())
            << "offsets must strictly ascend; record " << k << " repeats or regresses";
    }

    std::error_code ec;
    fs::remove_all(dir, ec);
}

/**
 * @brief A trailing partial packet is still reported as truncation, measured from the last
 *        complete packet.
 *
 * The truncation signal is derived from the same cursor the offsets are, so it has to keep working
 * after the rewrite. A mid-file junk run must NOT be mistaken for truncation — only bytes after
 * the final complete packet count.
 */
TEST(ScanOffsets, TrailingPartialPacketIsStillTruncation) {
    const fs::path dir = makeTempDir("sn8765_trunc");
    std::vector<uint8_t> bytes;

    pimu_t pimu{};
    for (int i = 0; i < 5; ++i) {
        pimu.time = 1.0 + i;
        appendIsb(bytes, DID_PIMU, sizeof(pimu), &pimu);
    }
    const std::size_t afterComplete = bytes.size();

    // Half a packet: build one, then keep only its first few bytes.
    std::vector<uint8_t> partial;
    pimu.time = 99.0;
    appendIsb(partial, DID_PIMU, sizeof(pimu), &pimu);
    ASSERT_GT(partial.size(), 8u);
    bytes.insert(bytes.end(), partial.begin(), partial.begin() + 8);

    const fs::path raw = writeRaw(dir, bytes);
    auto reader = ISLogReader::openSegment(raw);
    ASSERT_TRUE(reader.has_value());

    EXPECT_EQ(reader->recordCount(), 5u);
    EXPECT_TRUE(reader->isTruncated());
    EXPECT_EQ(reader->truncationOffset(), static_cast<uint64_t>(afterComplete));
    expectEveryOffsetIsAPacketStart(*reader, "trunc");

    std::error_code ec;
    fs::remove_all(dir, ec);
}
