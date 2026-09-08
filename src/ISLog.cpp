/**
 * @file ISLog.cpp
 * @brief See ISLog.h.
 *
 * D-05 / SN-7896.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISLog.h"

#include "ISLogReader.h"
#include "core/msg_logger.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <stdexcept>
#include <system_error>

namespace inertial_sense {

namespace fs = std::filesystem;

ISLog::~ISLog()                              = default;
ISLog::ISLog(ISLog&&) noexcept               = default;
ISLog& ISLog::operator=(ISLog&&) noexcept    = default;

namespace {

// D-119 / SN-8626/SN-8627: a directory scan must discover `.dat` segments too, not just `.raw` --
// LogLoader::loadDirectory routes through this function, and before this fix a directory holding
// only `.dat`+`.idx` files (a real manufacturing/calibration capture, not a synthetic test fixture)
// failed outright with "no .raw files", well before any `.dat`-aware code got a chance to run.
bool isSupportedSegmentExtension(const fs::path& p) {
    // Linux is case-sensitive; Windows / default macOS are not.
    // Accept ".raw"/".RAW" and ".dat"/".DAT" (and any case in between).
    auto ext = p.extension().string();
    if (ext.size() != 4) return false;
    if (ext[0] != '.') return false;
    auto low = [](char c){ return static_cast<char>(std::tolower(static_cast<unsigned char>(c))); };
    const char a = low(ext[1]), b = low(ext[2]), c = low(ext[3]);
    return (a == 'r' && b == 'a' && c == 'w') || (a == 'd' && b == 'a' && c == 't');
}

} // namespace

ISExpected<ISLog> ISLog::openDirectory(const fs::path& logDir) {
    log_info(IS_LOG_ISLOG, "ISLog::openDirectory: %s", logDir.string().c_str());

    std::error_code ec;
    if (!fs::exists(logDir, ec)) {
        log_error(IS_LOG_ISLOG, "directory does not exist: %s", logDir.string().c_str());
        return fail(ISErrorCode::NotFound,
                    "ISLog::openDirectory: directory does not exist: "
                    + logDir.string());
    }
    if (!fs::is_directory(logDir, ec)) {
        log_error(IS_LOG_ISLOG, "not a directory: %s", logDir.string().c_str());
        return fail(ISErrorCode::InvalidArgument,
                    "ISLog::openDirectory: not a directory: " + logDir.string());
    }

    // Walk non-recursively for `.raw`/`.dat` files. Drive the iterator explicitly with
    // the error_code-taking increment: the ec-form constructor only makes
    // construction non-throwing; a range-for's implicit operator++ can still
    // throw filesystem_error (e.g. the directory mutates mid-scan).
    std::vector<fs::path> segmentFiles;
    fs::directory_iterator it(logDir, ec);
    const fs::directory_iterator end;
    for (; !ec && it != end; it.increment(ec)) {
        const auto& entry = *it;
        if (!entry.is_regular_file()) continue;
        if (isSupportedSegmentExtension(entry.path())) {
            segmentFiles.push_back(entry.path());
        }
    }
    if (ec) {
        log_error(IS_LOG_ISLOG, "directory iteration failed: %s",
                  ec.message().c_str());
        return fail(ISErrorCode::PermissionDenied,
                    "ISLog::openDirectory: directory iteration failed: "
                    + ec.message());
    }
    if (segmentFiles.empty()) {
        log_warn(IS_LOG_ISLOG, "no .raw/.dat files in %s", logDir.string().c_str());
        return fail(ISErrorCode::NotFound,
                    "ISLog::openDirectory: no .raw/.dat files in " + logDir.string());
    }
    log_more_info(IS_LOG_ISLOG, "discovered %zu segment(s)", segmentFiles.size());

    // Sort lexicographically so per-device segment order is stable.
    std::sort(segmentFiles.begin(), segmentFiles.end());

    // Open each segment, group by device-id.
    std::map<uint64_t, std::vector<fs::path>> byDevice;
    for (const auto& path : segmentFiles) {
        auto r = ISLogReader::openSegment(path);
        if (!r) {
            log_error(IS_LOG_ISLOG, "openSegment failed for %s: %s",
                      path.string().c_str(), r.error().message.c_str());
            return tl::unexpected<ISError>{ r.error() };
        }
        const uint64_t devId = r->deviceId();
        log_debug(IS_LOG_ISLOG, "  %s -> device 0x%016llx",
                  path.filename().string().c_str(), static_cast<unsigned long long>(devId));
        byDevice[devId].push_back(path);
        // Drop the reader; ISDeviceLog::fromSegments re-opens.
        r = tl::unexpected<ISError>{ ISError{ ISErrorCode::Internal, "drop" } };
    }

    ISLog out;
    out.directory_   = logDir;
    out.allSegments_ = std::move(segmentFiles);

    out.orderedIds_.reserve(byDevice.size());
    for (auto& [devId, paths] : byDevice) {
        log_more_info(IS_LOG_ISLOG,
                      "  building device 0x%016llx: %zu segment(s)",
                      static_cast<unsigned long long>(devId), paths.size());
        auto dl = ISDeviceLog::fromSegments(std::move(paths));
        if (!dl) {
            log_error(IS_LOG_ISLOG, "ISDeviceLog::fromSegments failed for "
                      "device 0x%016llx: %s",
                      static_cast<unsigned long long>(devId),
                      dl.error().message.c_str());
            return tl::unexpected<ISError>{ dl.error() };
        }
        out.devicesById_.emplace(devId, std::move(*dl));
        out.orderedIds_.push_back(devId);
    }

    log_info(IS_LOG_ISLOG, "ISLog::openDirectory: %s -> %zu device(s), "
             "%zu segment(s), %zu record(s) total",
             logDir.string().c_str(),
             out.orderedIds_.size(),
             out.allSegments_.size(),
             out.recordCount());
    return out;
}

std::vector<uint64_t> ISLog::deviceIds() const {
    std::vector<uint64_t> out = orderedIds_;
    std::sort(out.begin(), out.end());
    return out;
}

const ISDeviceLog& ISLog::device(uint64_t deviceId) const {
    auto it = devicesById_.find(deviceId);
    if (it == devicesById_.end()) {
        // Per the doc — undefined for an absent id. Throwing here
        // gives a more helpful diagnostic than UB; the SDK's broader
        // policy is no-exceptions, so callers should check
        // deviceIds() first.
        throw std::out_of_range(
            "ISLog::device: deviceId " + std::to_string(deviceId)
            + " not present");
    }
    return it->second;
}

TimeStamp ISLog::spanStart() const noexcept {
    uint64_t best = 0;
    bool any = false;
    for (const auto& [_, dl] : devicesById_) {
        const uint64_t v = dl.spanStart().value;
        if (v == 0) continue;
        if (!any || v < best) { best = v; any = true; }
    }
    return TimeStamp::fromPayloadToW(best, /*deviceId*/ 0);
}

TimeStamp ISLog::spanEnd() const noexcept {
    uint64_t best = 0;
    bool any = false;
    for (const auto& [_, dl] : devicesById_) {
        const uint64_t v = dl.spanEnd().value;
        if (v == 0) continue;
        if (!any || v > best) { best = v; any = true; }
    }
    return TimeStamp::fromPayloadToW(best, /*deviceId*/ 0);
}

std::vector<fs::path> ISLog::segmentPaths() const {
    return allSegments_;
}

std::size_t ISLog::recordCount() const noexcept {
    std::size_t n = 0;
    for (const auto& [_, dl] : devicesById_) n += dl.recordCount();
    return n;
}

std::vector<ISDeviceLog> ISLog::takeDevices() {
    std::vector<ISDeviceLog> out;
    out.reserve(orderedIds_.size());
    for (const uint64_t id : orderedIds_) {
        auto it = devicesById_.find(id);
        if (it == devicesById_.end()) continue;
        out.push_back(std::move(it->second));
    }
    devicesById_.clear();
    orderedIds_.clear();
    return out;
}

} // namespace inertial_sense
