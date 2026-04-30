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

bool isRawExtension(const fs::path& p) {
    // Linux is case-sensitive; Windows / default macOS are not.
    // Accept ".raw" and ".RAW" (and any case in between).
    auto ext = p.extension().string();
    if (ext.size() != 4) return false;
    if (ext[0] != '.') return false;
    auto low = [](char c){ return static_cast<char>(std::tolower(static_cast<unsigned char>(c))); };
    return low(ext[1]) == 'r' && low(ext[2]) == 'a' && low(ext[3]) == 'w';
}

} // namespace

ISExpected<ISLog> ISLog::openDirectory(const fs::path& logDir) {
    std::error_code ec;
    if (!fs::exists(logDir, ec)) {
        return fail(ISErrorCode::NotFound,
                    "ISLog::openDirectory: directory does not exist: "
                    + logDir.string());
    }
    if (!fs::is_directory(logDir, ec)) {
        return fail(ISErrorCode::InvalidArgument,
                    "ISLog::openDirectory: not a directory: " + logDir.string());
    }

    // Walk non-recursively for `.raw` files.
    std::vector<fs::path> rawFiles;
    for (const auto& entry : fs::directory_iterator(logDir, ec)) {
        if (ec) {
            return fail(ISErrorCode::PermissionDenied,
                        "ISLog::openDirectory: directory iteration failed: "
                        + ec.message());
        }
        if (!entry.is_regular_file()) continue;
        if (isRawExtension(entry.path())) {
            rawFiles.push_back(entry.path());
        }
    }
    if (rawFiles.empty()) {
        return fail(ISErrorCode::NotFound,
                    "ISLog::openDirectory: no .raw files in " + logDir.string());
    }

    // Sort lexicographically so per-device segment order is stable.
    std::sort(rawFiles.begin(), rawFiles.end());

    // Open each segment, group by device-id.
    std::map<uint64_t, std::vector<fs::path>> byDevice;
    for (const auto& path : rawFiles) {
        auto r = ISLogReader::openSegment(path);
        if (!r) {
            return tl::unexpected<ISError>{ r.error() };
        }
        const uint64_t devId = r->deviceId();
        byDevice[devId].push_back(path);
        // Drop the reader; ISDeviceLog::fromSegments re-opens.
        r = tl::unexpected<ISError>{ ISError{ ISErrorCode::Internal, "drop" } };
    }

    ISLog out;
    out.directory_   = logDir;
    out.allSegments_ = std::move(rawFiles);

    out.orderedIds_.reserve(byDevice.size());
    for (auto& [devId, paths] : byDevice) {
        auto dl = ISDeviceLog::fromSegments(std::move(paths));
        if (!dl) {
            return tl::unexpected<ISError>{ dl.error() };
        }
        out.devicesById_.emplace(devId, std::move(*dl));
        out.orderedIds_.push_back(devId);
    }

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

} // namespace inertial_sense
