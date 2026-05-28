/**
 * @file ISSyncPoint.h
 * @brief Value type representing a single ToW-bearing record observed
 *        in a device log. D-07 / SN-7897.
 *
 * `ISTimeResolver::detectSyncPoints` produces a sorted vector of these,
 * one per HAS_TOW-flagged record in the source `ISDeviceLog` (after
 * adjacent-duplicate filtering).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include <cstdint>

namespace inertial_sense {

/**
 * @brief One sync anchor — a record carrying both a host-side timestamp
 *        and a payload-derived GPS time-of-week.
 *
 * **v2 `.idx` schema note.** In the current writer (`cDeviceLog`), a
 * record with the `IS_LOG_IDX_REC_FLAG_HAS_TOW` bit set has its `.idx`
 * `timestamp` field overwritten with the payload ToW (in ms) — the
 * host-side capture time isn't stored separately. So for v2-produced
 * sync points, `hostTimeMs == payloadToWMs`. This is documented in
 * the resolver's header (`ISTimeResolver.h`) since it constrains
 * what the slope-fit / discontinuity-detection algorithms can do
 * meaningfully on v2 logs. A future `.idx` v3 may add an explicit
 * host-time field; sync points adopt it transparently when it lands.
 */
struct ISSyncPoint {
    /// Host-side timestamp of the record, in ms. For v2 `.idx` HAS_TOW
    /// records this equals `payloadToWMs`; for hypothetical v3 schemas
    /// with a separate host field it carries the distinct host clock.
    uint64_t hostTimeMs;

    /// Payload's GPS time-of-week, in ms.
    uint64_t payloadToWMs;

    /// Source device's serial number.
    uint64_t deviceId;

    /// DID that supplied the sync (e.g. `DID_INS_2`, `DID_GNSS1_POS`).
    uint32_t sourceDid;

    /// **SN-8107 / D0066.** Recovered host-uptime at which this sync
    /// record was written, captured from the most recent non-sync
    /// record observed before this sync during the segment byte scan.
    /// On v2 `.idx`, sync records' own `.idx` timestamp field is
    /// overwritten with the payload ToW, so the "real" host clock at
    /// sync time isn't stored — but the adjacent non-sync record's
    /// host-uptime is an excellent approximation (the writer emits
    /// records in arrival order on one host thread, so consecutive
    /// records are millisecond-adjacent in host time).
    ///
    /// `ISTimeResolver::resolve` uses this to bridge session-uptime
    /// queries into the ToW frame: any non-sync record's stored
    /// host-uptime `H` resolves to `payloadToWMs + (H - actualHostTimeMs)`
    /// where the sync point used is the nearest one (typically the
    /// first sync, since pre-fix records dominate the session-uptime
    /// query population).
    ///
    /// `0` means "no pre-sync non-sync record observed" — e.g. the
    /// very first record in the segment is the sync point. In that
    /// degenerate case the resolver falls back to the legacy
    /// classify-only behavior.
    uint64_t actualHostTimeMs = 0;
};

} // namespace inertial_sense
