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
 * **v2 `.idx` schema note.** In the writer (`cDeviceLog`), a record
 * with the `IS_LOG_IDX_REC_FLAG_HAS_TOW` bit set has its `.idx`
 * `timestamp` field overwritten with the payload ToW (in ms) — the
 * host-side capture time isn't stored separately in the .idx. So for
 * .idx-only readers, `hostTimeMs == payloadToWMs`. The host-time at
 * sync is recovered from the .raw byte stream during scan and carried
 * on `actualHostTimeMs` (SN-8107 / D0066); the resolver uses that to
 * bridge session-uptime queries into the ToW frame.
 */
struct ISSyncPoint {
    //! Host-side timestamp of the record, in ms. For HAS_TOW records
    //! this equals `payloadToWMs` (the .idx-stored value); the
    //! `.raw`-recovered host-time at sync is on `actualHostTimeMs`.
    uint64_t hostTimeMs;

    //! Payload's GPS time-of-week, in ms.
    uint64_t payloadToWMs;

    //! Source device's serial number.
    uint64_t deviceId;

    //! DID that supplied the sync (e.g. `DID_INS_2`, `DID_GNSS1_POS`).
    uint32_t sourceDid;

    //! Recovered host-uptime at which this sync record was written,
    //! captured from the most recent non-sync record observed before
    //! this sync during the segment byte scan. On v2 `.idx`, sync
    //! records' own `.idx` timestamp field is overwritten with the
    //! payload ToW, so the "real" host clock at sync time isn't
    //! stored — but the adjacent non-sync record's host-uptime is an
    //! excellent approximation (the writer emits records in arrival
    //! order on one host thread, so consecutive records are
    //! millisecond-adjacent in host time).
    //!
    //! `ISTimeResolver::resolve` uses this to bridge session-uptime
    //! queries into the ToW frame: any non-sync record's stored
    //! host-uptime `H` resolves to `payloadToWMs + (H - actualHostTimeMs)`
    //! where the sync point used is the nearest one (typically the
    //! first sync, since pre-fix records dominate the session-uptime
    //! query population).
    //!
    //! `0` means "no pre-sync non-sync record observed" — e.g. the
    //! very first record in the segment is the sync point. In that
    //! degenerate case the resolver falls back to the legacy
    //! classify-only behavior.
    //!
    //! @note SN-8107 / D0066.
    uint64_t actualHostTimeMs = 0;

    //! GPS week number from the payload (e.g. `ins_2_t::week`,
    //! `gnss_pos_t::week`). Used together with `payloadToWMs` to
    //! compute an absolute Unix-epoch wall-clock ms:
    //! `unix_ms = (gpsWeek × 604,800,000) + GPS_EPOCH_UNIX_MS + payloadToWMs`,
    //! where `GPS_EPOCH_UNIX_MS = 315,964,800,000` (1980-01-06 UTC).
    //!
    //! When ANY sync point in the resolver's vector has `gpsWeek != 0`,
    //! `ISTimeResolver::resolve()` anchors all output values to Unix
    //! epoch so they're directly suitable for
    //! `QDateTime::fromMSecsSinceEpoch` / wall-clock display. When all
    //! sync points have `gpsWeek == 0` (logs from devices without a
    //! usable week field), the resolver returns ToW-domain values
    //! (pre-D0066 behavior).
    //!
    //! All ToW-bearing DID payload structs (`ins_1_t`, `ins_2_t`,
    //! `ins_3_t`, `ins_4_t`, `gnss_pos_t`) start with a `uint32_t week`
    //! field, so the scanner reads it from the first 4 bytes of the
    //! payload.
    //!
    //! @note SN-8107 / D0066.
    uint32_t gpsWeek = 0;
};

} // namespace inertial_sense
