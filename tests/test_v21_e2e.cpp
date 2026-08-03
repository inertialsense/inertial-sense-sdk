// End-to-end (SN-8383 / SN-8339 / D0066): drive the COMPLETE writer path
// (cISLogger -> DeviceLog -> v2.1 .idx) with ~30-45 s of pseudo-random,
// multi-rate DID traffic that INCLUDES a timeless DID (DID_PORT_MONITOR), then
// verify:
//   1. the .idx is v2.1 (32-byte records + HAS_LOCAL_DELTA) and carries
//      per-record host-uptime deltas that grow with the log's real elapsed time
//      (i.e. reflect each DID's logging periodicity),
//   2. ISLogReader parses the resulting pair, and
//   3. ISTimeResolver, built from the log, resolves the timeless PORT_MONITOR
//      records (which have NO payload timestamp) to the correct wall-clock times
//      by bridging their per-record uptime delta through the SYS_PARAMS
//      uptime->ToW offset.
//
// This runs in REAL TIME on purpose: the per-record delta is stamped from the
// host monotonic clock (current_uptimeMs), which is not injectable, so the only
// way the delta and the SYS_PARAMS.upTime land on the same scale — the property
// that lets a timeless record resolve correctly — is to let real time pass. Set
// IS_E2E_DURATION_MS to shorten the run for local iteration / CI smoke.
//
// Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.

#include <gtest/gtest.h>

// com_manager.h first — short-circuits the broken extern-C wrap in
// ISFirmwareUpdater.h (same trick as test_log_index.cpp / test_log_reader.cpp).
#include "com_manager.h"

#include "ISComm.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLogIndex.h"
#include "ISLogReader.h"
#include "ISLogger.h"
#include "ISTimeResolver.h"
#include "ISUtilities.h"       // current_uptimeMs
#include "data_sets.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <filesystem>
#include <random>
#include <thread>
#include <vector>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kHwId       = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kSerial     = 990321u;
constexpr uint32_t kGpsWeek    = 2300u;
constexpr uint32_t kStartTowMs = 200'000'000u;   // GPS ToW ms at log start (well inside a 604,800,000 ms week)

// Unix-epoch ms for (kGpsWeek, towMs) — the resolver epoch-anchors output once
// any sync point carries a non-zero gpsWeek (SN-8107 / D0066).
uint64_t expectedUnixMs(uint64_t towMs) {
    constexpr uint64_t kGpsEpochUnixMs = 315'964'800'000ULL;
    constexpr uint64_t kWeekMs         = 604'800'000ULL;
    return static_cast<uint64_t>(kGpsWeek) * kWeekMs + kGpsEpochUnixMs + towMs;
}

void writeRecord(cISLogger& logger, std::shared_ptr<cDeviceLog> dev,
                 uint32_t did, const void* payload, std::size_t size) {
    is_comm_instance_t comm{};
    uint8_t buf[1024];
    is_comm_init(&comm, buf, sizeof(buf), nullptr);
    uint8_t pkt[2048];
    const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm,
                                      static_cast<uint16_t>(did),
                                      static_cast<uint16_t>(size), 0,
                                      const_cast<void*>(payload));
    if (n > 0) logger.LogData(dev, n, pkt);
}

// A timed source (ToW-bearing). timeOfWeek is seconds.
ins_2_t makeIns2(uint32_t elapsedMs) {
    ins_2_t s{};
    s.week       = kGpsWeek;
    s.timeOfWeek = (kStartTowMs + elapsedMs) / 1000.0;
    s.qn2b[0]    = 1.0f;
    s.lla[0] = 40.0; s.lla[1] = -111.0; s.lla[2] = 1400.0;
    return s;
}

// The uptime<->ToW anchor. upTime is fractional seconds so the offset is exact.
sys_params_t makeSysParams(uint32_t elapsedMs) {
    sys_params_t s{};
    s.timeOfWeekMs = kStartTowMs + elapsedMs;
    s.upTime       = elapsedMs / 1000.0;
    s.hdwStatus   |= HDW_STATUS_GNSS_TIME_OF_WEEK_VALID;
    return s;
}

// A non-ToW uptime source (payload `time` is seconds-since-boot).
imu_t makeImu(uint32_t elapsedMs) {
    imu_t s{};
    s.time = elapsedMs / 1000.0;
    s.I.acc[0] = 0.1f; s.I.acc[1] = 0.2f; s.I.acc[2] = 9.8f;
    return s;
}

// The timeless subject: DID_PORT_MONITOR has no payload timestamp field, so its
// .idx timestamp is the host-uptime delta and HAS_TOW is clear.
port_monitor_t makePortMon() {
    port_monitor_t s{};
    s.activePorts = 2;
    return s;
}

uint32_t durationMs() {
    if (const char* e = std::getenv("IS_E2E_DURATION_MS")) {
        const long v = std::strtol(e, nullptr, 10);
        if (v > 0) return static_cast<uint32_t>(v);
    }
    return 15'000u;   // >128KB (multi-chunk) at the rates below; overridable
}

}  // namespace

// Guarded end-to-end soak: OFF by default (runs in real time and sleeps), so it
// does not slow routine CI. Run it explicitly to revalidate the writer/reader/
// resolver chain end-to-end: `IS_RUN_E2E_TESTS=1 ./IS-SDK_unit-tests
// --gtest_filter=V21EndToEnd*` (optionally IS_E2E_DURATION_MS=<ms>). It still
// compiles every build, so it can't bit-rot.
TEST(V21EndToEnd, PseudoRandomMultiDidWithTimelessPortMonitor) {
    if (std::getenv("IS_RUN_E2E_TESTS") == nullptr) {
        GTEST_SKIP() << "end-to-end soak disabled by default; set IS_RUN_E2E_TESTS=1 to run";
    }
    const uint32_t durMs = durationMs();

    char dirBuf[256];
    std::snprintf(dirBuf, sizeof(dirBuf), "/tmp/test_v21_e2e_%d_%ld",
                  ::getpid(), static_cast<long>(::time(nullptr)));
    const fs::path dir = dirBuf;
    ISFileManager::DeleteDirectory(dir.string());

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    ASSERT_TRUE(logger.InitSave(dir.string(), opts));
    auto dev = logger.registerDevice(kHwId, kSerial);
    ASSERT_TRUE(dev != nullptr);
    logger.EnableLogging(true);

    // Per-DID nominal periods (ms). Pseudo-random jitter is applied per tick.
    struct Stream { uint32_t did; uint32_t period; uint32_t nextDue; };
    std::vector<Stream> streams = {
        { DID_INS_2,        20,  0 },   // 50 Hz timed
        { DID_IMU,          5,   0 },   // 200 Hz uptime-only (fastest — Kyle: ~5 ms periodicity)
        { DID_SYS_PARAMS,   500, 0 },   // 2 Hz uptime<->ToW anchor
        { DID_PORT_MONITOR, 100, 0 },   // 10 Hz timeless subject
    };

    std::mt19937 rng(0xC0FFEEu);
    std::uniform_int_distribution<int> jitter(-8, 8);   // ms of pseudo-random jitter

    std::size_t portMonEmitted = 0;
    const uint32_t t0 = current_uptimeMs();
    for (;;) {
        const uint32_t elapsed = current_uptimeMs() - t0;
        if (elapsed >= durMs) break;

        // Pseudo-randomize emission order within this tick.
        std::shuffle(streams.begin(), streams.end(), rng);
        for (auto& s : streams) {
            if (elapsed < s.nextDue) continue;
            switch (s.did) {
                case DID_INS_2:      { auto p = makeIns2(elapsed);     writeRecord(logger, dev, s.did, &p, sizeof(p)); } break;
                case DID_IMU:        { auto p = makeImu(elapsed);      writeRecord(logger, dev, s.did, &p, sizeof(p)); } break;
                case DID_SYS_PARAMS: { auto p = makeSysParams(elapsed);writeRecord(logger, dev, s.did, &p, sizeof(p)); } break;
                case DID_PORT_MONITOR:{ auto p = makePortMon();        writeRecord(logger, dev, s.did, &p, sizeof(p)); ++portMonEmitted; } break;
                default: break;
            }
            const int j = jitter(rng);
            s.nextDue = elapsed + static_cast<uint32_t>(std::max<int>(1, static_cast<int>(s.period) + j));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));   // fine granularity for the ~5 ms streams
    }
    logger.CloseAllFiles();
    // Nominal 4 Hz => ~durMs/250 records; require at least half (jitter/scheduling slack).
    const std::size_t minPortMon = durMs / 500u;
    ASSERT_GT(portMonEmitted, minPortMon) << "expected many PORT_MONITOR records over the run";

    // Locate the single .raw segment cISLogger produced.
    std::vector<ISFileManager::file_info_t> rawFiles;
    ISFileManager::GetAllFilesInDirectory(dir.string(), true, "\\.raw$", rawFiles);
    ASSERT_FALSE(rawFiles.empty());
    const fs::path rawFile = rawFiles.front().name;

    // ---- (1) v2.1 .idx with periodicity-based deltas -----------------------
    auto readerR = ISLogReader::openSegment(rawFile);
    ASSERT_TRUE(readerR.has_value()) << readerR.error().message;
    const ISLogReader& reader = readerR.value();
    ASSERT_TRUE(reader.hadOnDiskIndex()) << "cISLogger's v2.1 sidecar should be trusted, not rebuilt";

    EXPECT_EQ(reader.header().record_size, idx::IS_LOG_IDX_RECORD_V2_1_SIZE);
    EXPECT_NE(0, reader.header().flags & idx::IS_LOG_IDX_HDR_FLAG_HAS_LOCAL_DELTA);

    // PORT_MONITOR records: timeless (HAS_TOW clear); .idx timestamp == the
    // host-uptime delta, which must climb across the run reflecting the 4 Hz
    // periodicity — not collapse to ~0.
    std::vector<uint64_t> pmDeltas;
    std::vector<std::size_t> pmSpans;   // physical byte span of each record via the trusted offsets
    for (auto v : reader.records(DID_PORT_MONITOR)) {
        EXPECT_EQ(0, v.flags() & idx::IS_LOG_IDX_REC_FLAG_HAS_TOW) << "PORT_MONITOR must be timeless";
        pmDeltas.push_back(v.timestamp().value);      // .idx timestamp == local delta for a timeless record
        pmSpans.push_back(v.bytes().second);          // bytes sliced from the .raw at rec.offset
    }
    ASSERT_GT(pmDeltas.size(), minPortMon);
    EXPECT_TRUE(std::is_sorted(pmDeltas.begin(), pmDeltas.end())) << "deltas must be arrival-monotonic";
    EXPECT_LT(pmDeltas.front(), 2'000u)          << "first PORT_MONITOR near log start";
    EXPECT_GT(pmDeltas.back(),  durMs - 3'000u)  << "last PORT_MONITOR near log end (deltas track real elapsed)";

    // Byte-level offset proof: PORT_MONITOR packets are a fixed size, so the
    // physical byte span the reader slices at each rec.offset must be identical
    // and non-empty across all records except possibly the last (which runs to
    // EOF). If the writer stamped chunk-relative/garbage offsets, these spans
    // would be wrong or zero. (The whole read path also asserts hadOnDiskIndex
    // above, i.e. the sidecar was TRUSTED, not rebuilt.)
    ASSERT_GE(pmSpans.size(), 2u);
    const std::size_t firstSpan = pmSpans.front();
    EXPECT_GT(firstSpan, 0u) << "PORT_MONITOR payload span must be non-empty";
    for (std::size_t i = 0; i + 1 < pmSpans.size(); ++i) {
        EXPECT_EQ(pmSpans[i], firstSpan) << "PORT_MONITOR physical byte span differs at record " << i
                                         << " -> offsets are not correct physical positions";
    }

    // ---- (2)+(3) resolver places the timeless records at correct times -----
    auto logR = ISDeviceLog::fromSegments({ rawFile });
    ASSERT_TRUE(logR.has_value()) << logR.error().message;
    auto resolverR = ISTimeResolver::build(logR.value());
    ASSERT_TRUE(resolverR.has_value());
    EXPECT_FALSE(resolverR->syncPoints().empty()) << "SYS_PARAMS/INS should yield sync points";

    // Each PORT_MONITOR delta, bridged through the SYS_PARAMS uptime->ToW offset
    // (which is exactly kStartTowMs here), must resolve to the wall-clock time of
    // (kStartTowMs + delta) — i.e. the moment it was actually logged.
    uint64_t prevResolved = 0;
    for (const uint64_t d : pmDeltas) {
        const TimeStamp r = resolverR->resolve(d, kSerial);
        EXPECT_EQ(r.source, TimeSource::ResolvedViaSync) << "timeless record must bridge via sync, at delta " << d;
        const uint64_t want = expectedUnixMs(kStartTowMs + d);
        EXPECT_NEAR(static_cast<double>(r.value), static_cast<double>(want), 100.0)
            << "PORT_MONITOR at delta " << d << " resolved to the wrong wall-clock time";
        EXPECT_GE(r.value, prevResolved) << "resolved PORT_MONITOR times must be monotonic";
        prevResolved = r.value;
    }

    ISFileManager::DeleteDirectory(dir.string());
}
