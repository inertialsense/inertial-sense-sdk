/**
 * @file test_device_log.cpp
 * @brief D-05 / SN-7896 — ISDeviceLog composition tests.
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // first — see test_log_reader.cpp comment

#include "DeviceLog.h"
#include "ISDevice.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogFile.h"      // cISLogFile (SN-8383/8340 .idx inspection)
#include "ISLogIndex.h"     // idx::readHeader / readRecord / v2.1 flags
#include "ISLogger.h"
#include "test_data_utils.h"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
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

// SN-8339 — the cross-segment iterator stamps a dense global arrival index
// (0..N-1 in composition order), and a per-DID filtered range carries the SAME
// global index (not a per-DID local counter) so the multi-boot resolver keys
// correctly regardless of which range produced the view.
TEST(ISDeviceLog, StampsGlobalArrivalIndexAcrossSegments) {
    auto f = buildSingleDeviceFixture("arrival", kSerialA, 1.0f, 256 * 1024);
    ASSERT_GE(f.segmentsA.size(), 2u) << "need >=2 segments to prove global (not per-segment) index";

    auto dl = ISDeviceLog::fromSegments(f.segmentsA);
    ASSERT_TRUE(dl.has_value()) << dl.error().message;

    // allRecords() yields a dense 0..N-1 arrival index, strictly in order.
    std::vector<uint32_t> didByArrival(dl->recordCount(), 0u);
    uint64_t expected = 0;
    for (ISRecordView v : dl->allRecords()) {
        ASSERT_LT(expected, dl->recordCount());
        EXPECT_EQ(v.arrivalIndex(), expected);
        didByArrival[expected] = v.did();
        ++expected;
    }
    EXPECT_EQ(expected, dl->recordCount());

    // A per-DID filtered range carries the GLOBAL arrival index — each view's
    // index maps back to its own DID in the allRecords() ordering.
    auto dids = dl->presentDids();
    ASSERT_FALSE(dids.empty());
    uint64_t checked = 0;
    for (ISRecordView v : dl->records(dids.front())) {
        ASSERT_NE(v.arrivalIndex(), ISRecordView::kNoArrivalIndex);
        ASSERT_LT(v.arrivalIndex(), dl->recordCount());
        EXPECT_EQ(didByArrival[v.arrivalIndex()], v.did());
        ++checked;
    }
    EXPECT_EQ(checked, dl->records(dids.front()).size());

    teardown(f);
}

// SN-8383 / SN-8340 — the live logger writes a v2.1 .idx: 32-byte records with
// a per-record host-uptime delta (arrival-monotonic), and a durable capture
// epoch + the two feature flags in the header.
TEST(ISDeviceLog, WriterStampsV21LocalDeltaAndCaptureEpoch) {
    using namespace inertial_sense::idx;
    auto f = buildSingleDeviceFixture("v21", kSerialA, 0.3f, 1u << 30);  // one segment
    ASSERT_FALSE(f.segmentsA.empty());

    fs::path idxPath = f.segmentsA.front();
    idxPath.replace_extension(".idx");
    ASSERT_TRUE(fs::exists(idxPath)) << idxPath;

    cISLogFile in(idxPath.string(), "rb");
    ASSERT_TRUE(in.isOpened());
    auto hdrR = readHeader(in);
    ASSERT_TRUE(hdrR.has_value()) << hdrR.error().message;

    // v2.1 header: 32-byte records, both feature flags, a real host wall-clock.
    EXPECT_EQ(hdrR->record_size, IS_LOG_IDX_RECORD_V2_1_SIZE);
    EXPECT_NE(hdrR->flags & IS_LOG_IDX_HDR_FLAG_HAS_LOCAL_DELTA, 0u);
    EXPECT_NE(hdrR->flags & IS_LOG_IDX_HDR_FLAG_HAS_CAPTURE_EPOCH, 0u);
    EXPECT_GT(hdrR->capture_epoch_ms, 1'400'000'000'000ULL)   // > ~2014 => a real epoch, not uptime
        << "capture_epoch_ms should be a recent host wall-clock (SN-8340)";
    ASSERT_GT(hdrR->total_records, 0u);

    // Per-record local_uptime_ms is present and arrival-monotonic (the host
    // clock only advances). Not asserting non-zero: a very fast write could keep
    // every delta within the same millisecond; monotonicity is the invariant.
    uint64_t prev = 0;
    for (uint64_t i = 0; i < hdrR->total_records; ++i) {
        auto recR = readRecord(in, hdrR->record_size);
        ASSERT_TRUE(recR.has_value()) << "record " << i << ": " << recR.error().message;
        EXPECT_GE(recR->local_uptime_ms, prev)
            << "local_uptime_ms must be arrival-monotonic (record " << i << ")";
        prev = recR->local_uptime_ms;
    }

    teardown(f);
}

// ---------------------------------------------------------------------------
// No Jira card (rolled into SN-8629 at Kyle's request): a log segment that
// never itself streams a DID_DEV_INFO/FLASH_CONFIG record (a mid-session
// rollover, or a device that was already configured before logging started)
// has no way for a reader that opens ONLY that segment to identify the
// device. Fix: cDeviceLog::OpenNewSaveFile() now writes a one-time ".dvi"
// sidecar (same base name as the segment) containing a single DID_DEV_INFO
// record and a single DID_FLASH_CONFIG/DID_GPX_FLASH_CFG record, in raw ISB
// wire format, snapshotting whatever the bound ISDevice has cached at that
// instant. Only cDeviceLogRaw/cDeviceLogSerial (the raw-ISB-wire formats)
// opt in; CSV/JSON/KML don't.
// ---------------------------------------------------------------------------

namespace {

// Parses a .dvi (or any raw-ISB-wire byte stream) and returns each
// DID_DEV_INFO/DID_FLASH_CONFIG/DID_GPX_FLASH_CFG record found, in order.
struct DviRecord {
    uint32_t did;
    std::vector<uint8_t> payload;
};

std::vector<DviRecord> parseDviFile(const fs::path& path) {
    std::vector<DviRecord> out;
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return out;
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    is_comm_instance_t comm{};
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), NULLPTR);

    for (uint8_t b : bytes) {
        protocol_type_t p = is_comm_parse_byte(&comm, b);
        if (p != _PTYPE_INERTIAL_SENSE_DATA && p != _PTYPE_INERTIAL_SENSE_CMD) continue;
        DviRecord rec;
        rec.did = comm.rxPkt.dataHdr.id;
        rec.payload.assign(comm.rxPkt.data.ptr, comm.rxPkt.data.ptr + comm.rxPkt.dataHdr.size);
        out.push_back(std::move(rec));
    }
    return out;
}

// Builds a device with fully-controlled devInfo/flash-config content and logs
// one small buffer through a fresh cISLogger (LOGTYPE_RAW or LOGTYPE_DAT) so
// exactly one segment opens, then closes. Returns the segment's base path
// (without extension) and the fixture directory (caller deletes it).
struct DviFixture {
    fs::path directory;
    fs::path segmentBase;  // directory / basename, no extension
};

DviFixture buildDviFixture(const char* tag, cISLogger::eLogType logType,
                           uint8_t hardwareType, uint32_t serial) {
    DviFixture f;
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf), "/tmp/test_device_log_dvi_%s_%d_%ld",
                  tag, ::getpid(), static_cast<long>(::time(nullptr)));
    f.directory = dirBuf;
    ISFileManager::DeleteDirectory(f.directory.string());

    dev_info_t devInfo{};
    devInfo.hardwareType = hardwareType;
    devInfo.serialNumber = serial;
    devInfo.hardwareVer[0] = 1;
    devInfo.hardwareVer[1] = 2;

    auto device = std::make_shared<ISDevice>(devInfo);
    if (hardwareType == IS_HARDWARE_TYPE_GPX) {
        device->gpxFlashCfg.checksum = 0xABCD1234u;
    } else {
        device->imxFlashCfg.checksum = 0xABCD1234u;
    }

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType = logType;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(f.directory.string(), opts)) return f;
    auto devLogger = logger.registerDevice(device);
    if (!devLogger) return f;
    logger.EnableLogging(true);

    const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    if (logType == cISLogger::LOGTYPE_RAW) {
        logger.LogData(devLogger, sizeof(payload), payload);
    } else {
        // LogData(devLogger, int, const uint8_t*) is RAW-only; every other
        // format (including DAT) takes an already-parsed p_data_hdr_t.
        p_data_hdr_t hdr{};
        hdr.id = DID_DIAGNOSTIC_MESSAGE;
        hdr.size = sizeof(payload);
        hdr.offset = 0;
        logger.LogData(devLogger, &hdr, payload);
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> files;
    const char* ext = (logType == cISLogger::LOGTYPE_RAW) ? "\\.raw$" : "\\.dat$";
    ISFileManager::GetAllFilesInDirectory(f.directory.string(), true, ext, files);
    if (!files.empty()) {
        fs::path p(files.front().name);
        f.segmentBase = p.parent_path() / p.stem();
    }
    return f;
}

} // namespace

TEST(DeviceInfoSidecar, RawWriterEmitsDviWithDevInfoAndImxFlashConfig) {
    auto f = buildDviFixture("raw_imx", cISLogger::LOGTYPE_RAW, IS_HARDWARE_TYPE_IMX, 71501u);
    ASSERT_FALSE(f.segmentBase.empty()) << "fixture failed to produce a segment";

    const fs::path dviPath = fs::path(f.segmentBase.string() + ".dvi");
    ASSERT_TRUE(fs::exists(dviPath)) << dviPath;

    auto recs = parseDviFile(dviPath);
    ASSERT_EQ(recs.size(), 2u) << "expected exactly DID_DEV_INFO + DID_FLASH_CONFIG";

    EXPECT_EQ(recs[0].did, static_cast<uint32_t>(DID_DEV_INFO));
    ASSERT_EQ(recs[0].payload.size(), sizeof(dev_info_t));
    dev_info_t devInfo{};
    std::memcpy(&devInfo, recs[0].payload.data(), sizeof(devInfo));
    EXPECT_EQ(devInfo.serialNumber, 71501u);
    EXPECT_EQ(devInfo.hardwareType, static_cast<uint8_t>(IS_HARDWARE_TYPE_IMX));

    EXPECT_EQ(recs[1].did, static_cast<uint32_t>(DID_FLASH_CONFIG));
    ASSERT_EQ(recs[1].payload.size(), sizeof(nvm_flash_cfg_t));
    nvm_flash_cfg_t flashCfg{};
    std::memcpy(&flashCfg, recs[1].payload.data(), sizeof(flashCfg));
    EXPECT_EQ(flashCfg.checksum, 0xABCD1234u);

    ISFileManager::DeleteDirectory(f.directory.string());
}

TEST(DeviceInfoSidecar, RawWriterEmitsGpxFlashConfigForGpxHardware) {
    auto f = buildDviFixture("raw_gpx", cISLogger::LOGTYPE_RAW, IS_HARDWARE_TYPE_GPX, 71502u);
    ASSERT_FALSE(f.segmentBase.empty()) << "fixture failed to produce a segment";

    auto recs = parseDviFile(fs::path(f.segmentBase.string() + ".dvi"));
    ASSERT_EQ(recs.size(), 2u);

    // DID_DEV_INFO stays DID_DEV_INFO regardless of hardware type -- only the
    // flash-config DID is conditional (per spec).
    EXPECT_EQ(recs[0].did, static_cast<uint32_t>(DID_DEV_INFO));

    EXPECT_EQ(recs[1].did, static_cast<uint32_t>(DID_GPX_FLASH_CFG));
    ASSERT_EQ(recs[1].payload.size(), sizeof(gpx_flash_cfg_t));
    gpx_flash_cfg_t flashCfg{};
    std::memcpy(&flashCfg, recs[1].payload.data(), sizeof(flashCfg));
    EXPECT_EQ(flashCfg.checksum, 0xABCD1234u);

    ISFileManager::DeleteDirectory(f.directory.string());
}

TEST(DeviceInfoSidecar, DatWriterAlsoEmitsDvi) {
    auto f = buildDviFixture("dat_imx", cISLogger::LOGTYPE_DAT, IS_HARDWARE_TYPE_IMX, 71503u);
    ASSERT_FALSE(f.segmentBase.empty()) << "fixture failed to produce a segment";

    const fs::path dviPath = fs::path(f.segmentBase.string() + ".dvi");
    EXPECT_TRUE(fs::exists(dviPath)) << dviPath << " -- .dat (cDeviceLogSerial) must also opt in";

    auto recs = parseDviFile(dviPath);
    ASSERT_EQ(recs.size(), 2u);
    EXPECT_EQ(recs[0].did, static_cast<uint32_t>(DID_DEV_INFO));
    EXPECT_EQ(recs[1].did, static_cast<uint32_t>(DID_FLASH_CONFIG));

    ISFileManager::DeleteDirectory(f.directory.string());
}

TEST(DeviceInfoSidecar, NoDviWhenNoDeviceBound) {
    // A cDeviceLog constructed from bare hdwId/serialNo (no live ISDevice) has
    // nothing to snapshot -- must silently skip the sidecar, not crash or emit
    // an empty one.
    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf), "/tmp/test_device_log_dvi_nodev_%d_%ld",
                  ::getpid(), static_cast<long>(::time(nullptr)));
    const std::string dir = dirBuf;
    ISFileManager::DeleteDirectory(dir);

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    ASSERT_TRUE(logger.InitSave(dir, opts));
    auto devLogger = logger.registerDevice(kIMX5HwId, 71504u);
    ASSERT_TRUE(devLogger != nullptr);
    logger.EnableLogging(true);

    const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    logger.LogData(devLogger, sizeof(payload), payload);
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> dviFiles;
    ISFileManager::GetAllFilesInDirectory(dir, true, "\\.dvi$", dviFiles);
    EXPECT_TRUE(dviFiles.empty()) << "no bound ISDevice -- must not create a .dvi";

    ISFileManager::DeleteDirectory(dir);
}
