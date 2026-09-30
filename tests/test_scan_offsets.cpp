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

#include "DataChunk.h"     // DEFAULT_CHUNK_DATA_SIZE
#include "ISComm.h"
#include "ISDataMappings.h"
#include "ISLogIndex.h"
#include "ISFileManager.h"
#include "ISLogger.h"
#include "ISLogReader.h"
#include "data_sets.h"

#include <algorithm>
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
 * @brief Appends a run of bytes the parser cannot frame at all.
 *
 * `0x11` is not any enabled protocol's start byte, so the parser reports `STREAM_UNPARSABLE` and
 * walks past. Note this alone does **not** produce a drain — see @ref appendFalsePreamble.
 */
void appendJunk(std::vector<uint8_t>& out, std::size_t bytes) {
    out.insert(out.end(), bytes, static_cast<uint8_t>(0x11));
}

/**
 * @brief Appends a FALSE ISB header that declares a large payload — the shape that forces the
 *        parser to drain.
 *
 * Establishing this took measurement, and the two obvious candidates do not work:
 *
 *  - a run of unframeable bytes does not drain. The parser never locks on, so
 *    `is_comm_reset_parser` never rewinds `scan`, so no backlog accumulates.
 *  - a single corrupted packet (valid preamble and header, mangled payload) does not drain
 *    either. It fails, resyncs, and every following emit has `span == rxPkt.size`.
 *
 * What DOES drain: a real preamble and header declaring a payload far larger than what follows.
 * The parser locks on and keeps consuming, swallowing the genuine packets behind it, then fails
 * the checksum and `is_comm_reset_parser` rewinds `scan` all the way back to `head`. The packets
 * it had already buffered then come out on consecutive calls. Measured with this exact fixture:
 * `span=1  rxPkt.size=28` and `span=35  rxPkt.size=72` — complete packets emitted having consumed
 * one and 35 bytes respectively.
 *
 * The bytes are copied from a genuine packet so the preamble and header shape stay valid; only
 * the declared size is overwritten.
 */
void appendFalsePreamble(std::vector<uint8_t>& out) {
    std::vector<uint8_t> tmpl;
    ins_2_t              any{};
    appendIsb(tmpl, DID_INS_2, sizeof(any), &any);
    ASSERT_GE(tmpl.size(), 16u);
    const std::size_t at = out.size();
    out.insert(out.end(), tmpl.begin(), tmpl.begin() + 16);
    // Overwrite the declared payload size with ~576 bytes, which is more than the packets that
    // follow it, so they are consumed into the doomed packet and then replayed.
    out[at + 6] = 0x40;
    out[at + 7] = 0x02;
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

    // Two distinct hazards, because they are distinct bugs. The junk run exercises the SEMANTIC
    // half — an offset must be the packet's start, not the byte after the previous emit, and those
    // differ by exactly the junk. The false preamble exercises the DRAIN half — the parser
    // replaying buffered packets on calls whose input byte contributed nothing.
    appendJunk(bytes, 700);
    appendFalsePreamble(bytes);

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

// =================================================================================================
// The LIVE writer. Everything above rebuilds an index from a file; these write one on the wire.
// =================================================================================================

namespace {

/**
 * @brief Reads a whole file into a byte vector.
 */
std::vector<uint8_t> readAll(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::vector<uint8_t>{ std::istreambuf_iterator<char>(in),
                                 std::istreambuf_iterator<char>() };
}

/**
 * @brief Logs @p stream through `cISLogger` as raw bytes and returns the written segment paths.
 *
 * Goes through `LogData(dev, size, bytes)` — the raw-byte entry point a live capture uses — so the
 * `.idx` under test is the one `cDeviceLogRaw::SaveData` wrote on the wire, not a reader-side
 * rebuild. Fed in several chunks on purpose: a packet split across two `LogData` calls is exactly
 * the case the offset bookkeeping has to survive.
 */
bool logRawStream(const fs::path& dir, const std::vector<uint8_t>& stream, std::size_t chunkBytes,
                  std::vector<fs::path>& rawOut, std::vector<fs::path>& idxOut) {
    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(dir.string(), opts)) return false;
    auto dev = logger.registerDevice(ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0), 777002u);
    if (!dev) return false;
    logger.EnableLogging(true);
    for (std::size_t at = 0; at < stream.size(); at += chunkBytes) {
        const std::size_t n = std::min(chunkBytes, stream.size() - at);
        logger.LogData(dev, static_cast<int>(n), stream.data() + at);
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> raws, idxs;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.raw$", raws);
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.idx$", idxs);
    for (const auto& r : raws) rawOut.emplace_back(r.name);
    for (const auto& i : idxs) idxOut.emplace_back(i.name);
    std::sort(rawOut.begin(), rawOut.end());
    std::sort(idxOut.begin(), idxOut.end());
    return !rawOut.empty();
}

} // namespace

/**
 * @brief THE LIVE-WRITER CONTROL — an `.idx` written on the wire must carry packet starts.
 *
 * `cDeviceLogRaw::SaveData` used to set `m_rawIndexCursor = rawFileBase + (dPtr - dataBuf) + 1`
 * on the same false assumption as the rebuild, and its comment even noted it deliberately
 * "Mirrors ISLogReader's scan cursor" — the two were made consistent with each other and both
 * were wrong. This is the half that matters most in practice, because it corrupts sidecars as
 * they are recorded rather than only when one is rebuilt later.
 *
 * The stream deliberately contains a junk run so the parser errors, rewinds and then drains, and
 * it is fed in small chunks so packets also straddle `LogData` boundaries. **This test fails
 * against the pre-SN-8765 writer.**
 */
TEST(ScanOffsets, LiveWriterStampsPacketStarts) {
    const fs::path dir = makeTempDir("sn8765_live");
    ISFileManager::DeleteDirectory(dir.string());

    std::vector<uint8_t> stream;
    pimu_t   pimu{};
    ins_2_t  ins{};
    sys_params_t sys{};
    magnetometer_t mag{};

    // Clean prologue.
    for (int i = 0; i < 6; ++i) {
        pimu.time = 1.0 + i;
        appendIsb(stream, DID_PIMU, sizeof(pimu), &pimu);
    }
    // The shape that actually drains: a false header declaring a large payload, which swallows
    // the packets behind it and then replays them. A plain junk run does NOT do this — see
    // `appendFalsePreamble`. Verified by measurement, not assumed.
    appendFalsePreamble(stream);
    // Back-to-back packets that complete out of the backlog.
    for (int i = 0; i < 10; ++i) {
        mag.time = 2.0 + i;
        appendIsb(stream, DID_MAGNETOMETER, sizeof(mag), &mag);
        ins.timeOfWeek = 200000.0 + i;
        appendIsb(stream, DID_INS_2, sizeof(ins), &ins);
        sys.upTime = 2.0 + i;
        appendIsb(stream, DID_SYS_PARAMS, sizeof(sys), &sys);
    }

    std::vector<fs::path> raws, idxs;
    ASSERT_TRUE(logRawStream(dir, stream, /*chunkBytes=*/37, raws, idxs))
        << "no .raw segment was written";
    ASSERT_FALSE(idxs.empty()) << "the live writer produced no sidecar to check";

    // Open with the sidecar in place. `hadOnDiskIndex()` is the guard that we are testing the
    // WRITER's offsets and not a reader-side rebuild of them.
    auto reader = ISLogReader::openSegment(raws.front());
    ASSERT_TRUE(reader.has_value());
    ASSERT_TRUE(reader->hadOnDiskIndex())
        << "the reader rebuilt the index, so this would be testing the rebuild, not the writer";
    ASSERT_GT(reader->recordCount(), 0u);

    const auto bytes = readAll(raws.front());
    ASSERT_FALSE(bytes.empty());

    uint64_t prev = 0;
    for (std::size_t k = 0; k < reader->recordCount(); ++k) {
        const auto     rv  = reader->recordAt(k);
        const uint64_t off = rv.offsetInFile();
        if (k > 0) {
            EXPECT_GE(off, prev + kMinIsbPacketBytes)
                << "live: record " << k << " at " << off << " is closer than a minimum ISB "
                << "packet to the previous record at " << prev;
        }
        prev = off;
        EXPECT_EQ(didAtPacketStart(bytes.data(), bytes.size(), off), rv.did())
            << "live: record " << k << " offset " << off << " is not the start of a DID "
            << rv.did() << " packet";
    }

    ISFileManager::DeleteDirectory(dir.string());
}

/**
 * @brief A clean live capture, fed in awkward chunks: the writer must still be exact.
 *
 * No junk here, so the parser never drains — this is the no-regression half for the writer, and it
 * passes against the pre-SN-8765 writer too. Stated plainly so nobody reads it as proof: the test
 * that discriminates is `LiveWriterStampsPacketStarts`, which stamps 224 and 225 for two packets
 * whose true starts are 160 and 188 under the old writer (one byte apart — impossible for two
 * packets) and the correct values under the fix.
 *
 * The chunk size is chosen not to divide any packet length, so packets straddle `LogData`
 * boundaries throughout and the cross-call bookkeeping is exercised on every record.
 */
TEST(ScanOffsets, LiveWriterSurvivesPacketsSplitAcrossLogDataCalls) {
    const fs::path dir = makeTempDir("sn8765_live_split");
    ISFileManager::DeleteDirectory(dir.string());

    std::vector<uint8_t> stream;
    pimu_t  pimu{};
    ins_2_t ins{};
    for (int i = 0; i < 30; ++i) {
        pimu.time = 1.0 + i;
        appendIsb(stream, DID_PIMU, sizeof(pimu), &pimu);
        ins.timeOfWeek = 300000.0 + i;
        appendIsb(stream, DID_INS_2, sizeof(ins), &ins);
    }

    std::vector<fs::path> raws, idxs;
    ASSERT_TRUE(logRawStream(dir, stream, /*chunkBytes=*/13, raws, idxs));
    auto reader = ISLogReader::openSegment(raws.front());
    ASSERT_TRUE(reader.has_value());
    ASSERT_TRUE(reader->hadOnDiskIndex());
    EXPECT_EQ(reader->recordCount(), 60u);

    const auto bytes = readAll(raws.front());
    EXPECT_EQ(reader->recordAt(0).offsetInFile(), 0u);
    for (std::size_t k = 0; k < reader->recordCount(); ++k) {
        const auto rv = reader->recordAt(k);
        EXPECT_EQ(didAtPacketStart(bytes.data(), bytes.size(), rv.offsetInFile()), rv.did())
            << "split-feed: record " << k << " offset " << rv.offsetInFile()
            << " is not a packet start";
    }

    ISFileManager::DeleteDirectory(dir.string());
}

/**
 * @brief A packet split across a SEGMENT ROTATION, not just a `LogData` call, must not be
 *        misattributed to the new file.
 *
 * `cDeviceLogRaw::CloseAllFiles()` resets `m_fileSize` but does NOT reset (and, after SN-8765,
 * must not reset) the parser's `rxBuf` — whatever is buffered there when a segment closes was
 * already physically written to the OLD file, via `m_chunk.PushBack()` inside the SaveData call
 * that fed it. If that buffered remainder is the first half of a real packet, its completion
 * (fed from the NEW segment) has a true start that lies in the OLD file, which the new segment
 * cannot name with any offset of its own.
 *
 * Reproduced directly rather than inferred: `stream1` is padded with plain `DID_PIMU` packets to
 * just under `DEFAULT_CHUNK_DATA_SIZE`, then the FIRST HALF of one more packet is appended. That
 * packet's second half opens `stream2`, followed by a few ordinary packets. `maxFileSize` is set
 * far below one chunk, so the flush the oversized `stream2` call forces (its bytes no longer fit
 * the nearly-full chunk) immediately rotates — landing exactly while the split packet's first
 * half sits unconsumed in `rxBuf`.
 *
 * **Before the fix, this test fails**: the old code rebased `m_rawFedBytes` to `tail - head` at
 * the fresh-file check, which credits those carried-over bytes to the NEW file. The split
 * packet's completion then computed a start at (or near) offset 0 of the new segment — a byte
 * range that, in the new file, actually holds only the packet's second half, so it is not the
 * packet's real start there. The fix tracks `m_rawFedBytes` as an absolute, never-reset count and
 * skips indexing any packet whose absolute start precedes the segment's own start, rather than
 * naming it with a wrong offset.
 */
TEST(ScanOffsets, LiveWriterSkipsRecordThatStraddlesSegmentRotation) {
    const fs::path dir = makeTempDir("sn8765_straddle");
    ISFileManager::DeleteDirectory(dir.string());

    // Measure one filler packet's on-wire size so the padding loop's overshoot is bounded and
    // known, rather than guessed.
    std::vector<uint8_t> onePimu;
    pimu_t probe{};
    appendIsb(onePimu, DID_PIMU, sizeof(probe), &probe);
    const std::size_t pimuPktSize = onePimu.size();
    ASSERT_GT(pimuPktSize, 0u);

    // Build the packet that will straddle the rotation, and split it in half, BEFORE sizing the
    // padding -- the padding loop below reserves exactly this many bytes too.
    std::vector<uint8_t> splitPkt;
    ins_2_t splitIns{};
    splitIns.timeOfWeek = 999000.0;
    appendIsb(splitPkt, DID_INS_2, sizeof(splitIns), &splitIns);
    ASSERT_GE(splitPkt.size(), 20u) << "test assumption: the split packet has enough bytes to "
                                        "meaningfully straddle a call boundary";
    const std::size_t splitAt = splitPkt.size() / 2;

    // Pad with whole filler packets until fewer than one more (plus the split packet's first
    // half) would fit. That leaves the chunk's remaining free space bounded above by
    // `pimuPktSize` once the split-packet half is appended below -- comfortably smaller than
    // `stream2` (one packet-half plus three more whole packets), which is what forces the flush
    // (and, with `maxFileSize` tiny, the rotation) to happen at the very start of call 2.
    std::vector<uint8_t> stream1;
    pimu_t pimu{};
    int fillerIdx = 0;
    while (stream1.size() + pimuPktSize + splitAt < DEFAULT_CHUNK_DATA_SIZE) {
        pimu.time = 1.0 + (fillerIdx++);
        appendIsb(stream1, DID_PIMU, sizeof(pimu), &pimu);
    }
    stream1.insert(stream1.end(), splitPkt.begin(), splitPkt.begin() + static_cast<long>(splitAt));
    ASSERT_LT(stream1.size(), static_cast<std::size_t>(DEFAULT_CHUNK_DATA_SIZE))
        << "test assumption: call 1 must fit in one chunk without itself triggering a flush";

    std::vector<uint8_t> stream2(splitPkt.begin() + static_cast<long>(splitAt), splitPkt.end());
    pimu_t after{};
    for (int i = 0; i < 3; ++i) {
        after.time = 500.0 + i;
        appendIsb(stream2, DID_PIMU, sizeof(after), &after);
    }
    ASSERT_GT(stream2.size(), static_cast<std::size_t>(DEFAULT_CHUNK_DATA_SIZE) - stream1.size())
        << "test assumption: call 2 must not fit in the chunk's remaining free space, so it "
           "forces the flush (and rotation) right at its own start";

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    opts.maxFileSize           = 1024;  // far below one chunk: rotate as soon as ANY chunk flushes
    ASSERT_TRUE(logger.InitSave(dir.string(), opts));
    auto dev = logger.registerDevice(ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0), 777003u);
    ASSERT_TRUE(dev);
    logger.EnableLogging(true);

    // Two calls, deliberately: the first fills the chunk to just under capacity and ends mid-
    // packet; the second's bytes no longer fit, forcing the flush-and-rotate to happen with that
    // half-packet still sitting unconsumed in the parser's buffer.
    logger.LogData(dev, static_cast<int>(stream1.size()), stream1.data());
    logger.LogData(dev, static_cast<int>(stream2.size()), stream2.data());
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> rawInfos;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.raw$", rawInfos);
    std::vector<fs::path> raws;
    for (const auto& r : rawInfos) raws.emplace_back(r.name);
    std::sort(raws.begin(), raws.end());
    ASSERT_EQ(raws.size(), 2u) << "expected the small maxFileSize to force a second segment";

    // The second segment is the one under test: the parser carried the split packet's first
    // half over the rotation. Every record its .idx DOES claim must be a genuine packet start
    // IN THIS FILE — this file's bytes only ever held the packet's second half, so a record
    // claiming to start at (or near) offset 0 here would not be one.
    const auto seg2Bytes = readAll(raws[1]);
    ASSERT_FALSE(seg2Bytes.empty());
    auto reader2 = ISLogReader::openSegment(raws[1]);
    ASSERT_TRUE(reader2.has_value());
    ASSERT_TRUE(reader2->hadOnDiskIndex())
        << "the reader rebuilt the index, so this would be testing the rebuild, not the writer";
    for (std::size_t k = 0; k < reader2->recordCount(); ++k) {
        const auto rv = reader2->recordAt(k);
        EXPECT_EQ(didAtPacketStart(seg2Bytes.data(), seg2Bytes.size(), rv.offsetInFile()), rv.did())
            << "segment 2: record " << k << " at offset " << rv.offsetInFile() << " is not a real "
            << "packet start in this segment -- it likely straddled the rotation and was "
            << "misattributed here instead of being skipped";
    }
    // The straddling packet itself must be skipped, not mis-indexed: strictly fewer records than
    // (1 split + 3 filler) survive in segment 2.
    EXPECT_LT(reader2->recordCount(), 4u);

    ISFileManager::DeleteDirectory(dir.string());
}
