/**
 * @file ISLog.h
 * @brief Composition class — multiple `ISDeviceLog`s in one directory.
 *
 * D-05 / SN-7896 / D0051: top tier of the three-tier model. Walks
 * a single log directory non-recursively, groups `.raw` segments by
 * detected device-id, and exposes one `ISDeviceLog` per device.
 *
 * Move-only. Recursing into parent-directory cases (the "log of
 * logs" pattern) is the caller's job — D-33 / D-34 wrap this.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISDeviceLog.h"
#include "ISError.h"
#include "ISTimeStamp.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace inertial_sense {

class ISLog {
public:
    /**
     * Walks `logDir` non-recursively, opens each `.raw` segment as
     * an `ISLogReader`, groups them by detected device-id, and
     * builds one `ISDeviceLog` per device.
     *
     * @param logDir  Path to the log directory.
     * @return        Populated `ISLog` on success; on failure, an
     *                `ISError` with one of:
     *                - `NotFound`         — `logDir` doesn't exist
     *                                       or contains zero
     *                                       `.raw` files.
     *                - `PermissionDenied` — directory iteration
     *                                       blocked.
     *                - `Corrupted` / `Io` — at least one segment
     *                                       failed to open
     *                                       cleanly.
     */
    static ISExpected<ISLog> openDirectory(const std::filesystem::path& logDir);

    ~ISLog();
    ISLog(const ISLog&)            = delete;
    ISLog& operator=(const ISLog&) = delete;
    ISLog(ISLog&&) noexcept;
    ISLog& operator=(ISLog&&) noexcept;

    // ----- Query API -------------------------------------------------

    /** @return  Sorted ascending list of device ids present in this log. */
    std::vector<uint64_t> deviceIds() const;

    /**
     * @param deviceId  Device id to look up.
     * @return          Reference to the device-log, lifetime tied to
     *                  this `ISLog`. Behavior undefined for an id not
     *                  in `deviceIds()`; callers should check first.
     */
    const ISDeviceLog& device(uint64_t deviceId) const;

    /**
     * @return  Earliest timestamp across all device-logs (`PayloadToW`,
     *          `Exact`, `deviceId == 0` since this spans devices).
     *          `value == 0` for an empty log.
     */
    TimeStamp spanStart() const noexcept;

    /** @return  Latest timestamp across all device-logs. */
    TimeStamp spanEnd() const noexcept;

    /** @return  Path the log was opened from. */
    const std::filesystem::path& directory() const noexcept { return directory_; }

    /**
     * @return  Flat list of every `.raw` segment path discovered in
     *          this log, across all devices, in directory-iteration
     *          order. Useful for workspace serialization (D-24's
     *          per-segment SHA256 + size).
     */
    std::vector<std::filesystem::path> segmentPaths() const;

    /** @return  Total record count summed across every device-log. */
    std::size_t recordCount() const noexcept;

private:
    ISLog() = default;

    std::filesystem::path                              directory_;
    std::unordered_map<uint64_t, ISDeviceLog>          devicesById_;
    std::vector<uint64_t>                              orderedIds_;  // for stable iteration
    std::vector<std::filesystem::path>                 allSegments_; // flat
};

} // namespace inertial_sense
