// idx_benchmark — perf-impact harness for the v2 .idx sidecar.
//
// Drives the full cISLogger framework with simulated-but-real-shaped
// telemetry from tests/test_data_utils.cpp::GenerateRawLogData(),
// measures write-side cost and search-time speedup, and writes a CSV
// row per size point. See ./README.md for build + usage.
//
// Per D-01 / SN-7879.

#include "ISDataMappings.h"
#include "ISFileManager.h"
#include "ISLogFile.h"
#include "ISLogIndex.h"
#include "ISLogger.h"
#include "data_sets.h"
#include "test_data_utils.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <list>
#include <map>
#include <string>
#include <unistd.h>
#include <vector>

using namespace inertial_sense;
using namespace inertial_sense::idx;

namespace {

struct Result {
    float    sizeMb;
    size_t   numMessages;
    size_t   rawBytes;
    size_t   idxBytes;
    double   writeMs;
    double   idxFullReadMs;
    size_t   totalRecords;
    uint32_t targetDid;
    size_t   matchCount;
    double   searchViaIdxMs;
    double   searchViaRawMs;
};

void printUsage(const char* argv0) {
    std::fprintf(stderr,
        "usage: %s [--sizes <comma-separated MB>] [--out-csv <path>]\n"
        "          [--out-dir <dir>] [--quiet]\n"
        "\n"
        "options:\n"
        "  --sizes    comma-separated list of log sizes in MB (default:\n"
        "             0.25,0.5,1,2,3,4,6,8,12,16,24,32,48,64,96,128)\n"
        "  --out-csv  write CSV to this file (default: stdout only)\n"
        "  --out-dir  scratch dir for log files (default: /tmp)\n"
        "  --quiet    suppress per-row progress lines\n"
        "\n"
        "CSV columns:\n"
        "  size_mb, num_messages, raw_bytes, idx_bytes, write_ms,\n"
        "  total_records, idx_full_read_ms, target_did, match_count,\n"
        "  search_via_idx_ms, search_via_raw_ms\n",
        argv0);
}

std::vector<float> parseSizes(const std::string& csv) {
    std::vector<float> out;
    std::string cur;
    for (char c : csv) {
        if (c == ',' || c == ' ') {
            if (!cur.empty()) { out.push_back(std::stof(cur)); cur.clear(); }
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) out.push_back(std::stof(cur));
    return out;
}

Result runOne(float sizeMB, const std::string& outDir, bool quiet) {
    using clock_t = std::chrono::steady_clock;
    auto ms_of = [](clock_t::duration d) {
        return std::chrono::duration<double, std::milli>(d).count();
    };

    std::list<std::vector<uint8_t>*> messages;
    GenerateRawLogData(messages, sizeMB);

    char dirBuf[512];
    std::snprintf(dirBuf, sizeof(dirBuf),
                  "%s/idx_bench_%.2fmb_%d_%ld",
                  outDir.c_str(), sizeMB,
                  ::getpid(), static_cast<long>(::time(nullptr)));
    const std::string logPath = dirBuf;
    ISFileManager::DeleteDirectory(logPath);

    auto t0 = clock_t::now();
    {
        cISLogger logger;
        cISLogger::sSaveOptions options;
        options.logType = cISLogger::LOGTYPE_RAW;
        options.useSubFolderTimestamp = false;
        if (!logger.InitSave(logPath, options)) {
            std::fprintf(stderr, "InitSave failed for %s\n", logPath.c_str());
            std::exit(2);
        }
        auto devLogger = logger.registerDevice(
            ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0), 12345u);
        logger.EnableLogging(true);
        for (auto* msg : messages) {
            logger.LogData(devLogger, msg->size(),
                           reinterpret_cast<const uint8_t*>(msg->data()));
        }
        logger.CloseAllFiles();
    }
    auto t1 = clock_t::now();

    std::vector<ISFileManager::file_info_t> rawFiles, idxFiles;
    ISFileManager::GetAllFilesInDirectory(logPath, true, "\\.raw$", rawFiles);
    ISFileManager::GetAllFilesInDirectory(logPath, true, "\\.idx$", idxFiles);
    size_t rawBytes = 0, idxBytes = 0;
    for (const auto& f : rawFiles) rawBytes += f.size;
    for (const auto& f : idxFiles) idxBytes += f.size;

    auto t2 = clock_t::now();
    std::vector<is_log_idx_record_v2_t> records;
    for (const auto& f : idxFiles) {
        cISLogFile in(f.name, "rb");
        if (!in.isOpened()) {
            std::fprintf(stderr, "open failed: %s\n", f.name.c_str());
            std::exit(2);
        }
        auto hdrR = readHeader(in);
        if (!hdrR.has_value()) {
            std::fprintf(stderr, "%s: %s\n",
                         f.name.c_str(), hdrR.error().message.c_str());
            std::exit(2);
        }
        records.reserve(records.size() + hdrR->total_records);
        for (uint32_t i = 0; i < hdrR->total_records; ++i) {
            auto rec = readRecord(in);
            if (!rec.has_value()) {
                std::fprintf(stderr, "%s record %u: %s\n",
                             f.name.c_str(), i, rec.error().message.c_str());
                std::exit(2);
            }
            records.push_back(*rec);
        }
    }
    auto t3 = clock_t::now();

    // Pick the most-frequent DID as the search target so both methods
    // hit a non-trivial match count.
    std::map<uint32_t, size_t> didCounts;
    for (const auto& r : records) didCounts[r.did]++;
    uint32_t targetDid = 0;
    size_t   maxCount  = 0;
    for (const auto& kv : didCounts) {
        if (kv.second > maxCount) { targetDid = kv.first; maxCount = kv.second; }
    }
    if (targetDid == 0) {
        std::fprintf(stderr, "no records produced at sizeMB=%.2f\n", sizeMB);
        std::exit(2);
    }

    auto t4 = clock_t::now();
    size_t matchIdx = 0;
    for (const auto& r : records) {
        if (r.did == targetDid) ++matchIdx;
    }
    auto t5 = clock_t::now();

    auto t6 = clock_t::now();
    cISLogger reader;
    reader.LoadFromDirectory(logPath, cISLogger::LOGTYPE_RAW);
    reader.ShowParseErrors(false);
    size_t devIdx   = 0;
    size_t matchRaw = 0;
    for (size_t k = 0; k < messages.size(); ++k) {
        protocol_type_t pt = _PTYPE_NONE;
        packet_t* pkt = reader.ReadNextPacket(pt, devIdx);
        if (pkt == nullptr) break;
        if ((pt == _PTYPE_INERTIAL_SENSE_DATA || pt == _PTYPE_INERTIAL_SENSE_CMD) &&
            pkt->dataHdr.id == static_cast<uint16_t>(targetDid)) {
            ++matchRaw;
        }
    }
    reader.CloseAllFiles();
    auto t7 = clock_t::now();

    if (matchIdx != matchRaw) {
        std::fprintf(stderr,
            "WARNING: search counts disagree at sizeMB=%.2f (idx=%zu raw=%zu)\n",
            sizeMB, matchIdx, matchRaw);
    }

    Result r{};
    r.sizeMb         = sizeMB;
    r.numMessages    = messages.size();
    r.rawBytes       = rawBytes;
    r.idxBytes       = idxBytes;
    r.writeMs        = ms_of(t1 - t0);
    r.idxFullReadMs  = ms_of(t3 - t2);
    r.totalRecords   = records.size();
    r.targetDid      = targetDid;
    r.matchCount     = matchIdx;
    r.searchViaIdxMs = ms_of(t5 - t4);
    r.searchViaRawMs = ms_of(t7 - t6);

    if (!quiet) {
        std::fprintf(stderr,
            "[bench] %.2f MB: raw=%.2fMB idx=%.2fMB (%.1f%%)  "
            "write=%.0fms search idx=%.3fms raw=%.1fms (%.0fx)\n",
            sizeMB,
            rawBytes / (1024.0 * 1024.0), idxBytes / (1024.0 * 1024.0),
            (100.0 * idxBytes) / (rawBytes ? rawBytes : 1),
            r.writeMs,
            r.searchViaIdxMs, r.searchViaRawMs,
            r.searchViaRawMs / (r.searchViaIdxMs > 0 ? r.searchViaIdxMs : 1));
    }

    for (auto* msg : messages) delete msg;
    ISFileManager::DeleteDirectory(logPath);
    return r;
}

void writeCsvRow(FILE* fp, const Result& r) {
    std::fprintf(fp, "%.2f,%zu,%zu,%zu,%.3f,%zu,%.3f,%u,%zu,%.4f,%.4f\n",
                 r.sizeMb, r.numMessages, r.rawBytes, r.idxBytes, r.writeMs,
                 r.totalRecords, r.idxFullReadMs, r.targetDid, r.matchCount,
                 r.searchViaIdxMs, r.searchViaRawMs);
}

const char* kCsvHeader =
    "size_mb,num_messages,raw_bytes,idx_bytes,write_ms,total_records,"
    "idx_full_read_ms,target_did,match_count,search_via_idx_ms,"
    "search_via_raw_ms\n";

}  // namespace

int main(int argc, char** argv) {
    std::vector<float> sizes = {
        0.25f, 0.5f, 1.0f, 2.0f, 3.0f, 4.0f, 6.0f, 8.0f,
        12.0f, 16.0f, 24.0f, 32.0f, 48.0f, 64.0f, 96.0f, 128.0f,
    };
    std::string outCsv;
    std::string outDir = "/tmp";
    bool quiet = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        if (a == "--quiet") { quiet = true; continue; }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing argument for %s\n", a.c_str());
            return 1;
        }
        if (a == "--sizes") { sizes = parseSizes(argv[++i]); continue; }
        if (a == "--out-csv") { outCsv = argv[++i]; continue; }
        if (a == "--out-dir") { outDir = argv[++i]; continue; }
        std::fprintf(stderr, "unknown option: %s\n", a.c_str());
        printUsage(argv[0]);
        return 1;
    }

    std::vector<Result> results;
    for (float s : sizes) results.push_back(runOne(s, outDir, quiet));

    // Always emit CSV between sentinels on stdout (compat with the
    // gtest version's plot script) — and optionally also to a file.
    std::printf("\n#CSV-START\n%s", kCsvHeader);
    for (const auto& r : results) writeCsvRow(stdout, r);
    std::printf("#CSV-END\n");

    if (!outCsv.empty()) {
        FILE* fp = std::fopen(outCsv.c_str(), "wb");
        if (!fp) {
            std::fprintf(stderr, "could not open %s for write\n", outCsv.c_str());
            return 2;
        }
        std::fprintf(fp, "%s", kCsvHeader);
        for (const auto& r : results) writeCsvRow(fp, r);
        std::fclose(fp);
        if (!quiet) {
            std::fprintf(stderr, "[bench] wrote %zu rows to %s\n",
                         results.size(), outCsv.c_str());
        }
    }
    return 0;
}
