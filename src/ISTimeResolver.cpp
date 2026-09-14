/**
 * @file ISTimeResolver.cpp
 * @brief Piecewise-linear resolver implementation.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISTimeResolver.h"

// com_manager.h FIRST — see ISLogReader.cpp's note on the extern-C
// wrap collision in ISFirmwareUpdater.h.
#include "com_manager.h"

#include "ISComm.h"
#include "ISDataMappings.h"
#include "data_sets.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <map>
#include <optional>
#include <regex>
#include <system_error>

namespace inertial_sense {

namespace {

//! DIDs whose payloads carry a usable GPS time-of-week field. Used by
//! `detectSyncPoints` as the candidate set when scanning a segment.
//! `cISDataMappings::Timestamp(...)` is consulted on each candidate to
//! extract the actual ToW (returns 0 if the field is absent / zero,
//! which the detection loop treats as "not a sync anchor").
constexpr std::array<uint32_t, 6> kToWBearingDids = {
    DID_INS_1, DID_INS_2, DID_INS_3, DID_INS_4,
    DID_GNSS1_POS, DID_GNSS2_POS,
};

//! GPS epoch (1980-01-06 00:00:00 UTC) expressed as
//! milliseconds-since-Unix-epoch. Used together with `gpsWeek` and
//! `payloadToWMs` to anchor resolver output to Unix epoch so QDateTime
//! can render wall-clock dates directly.
//!
//! @note SN-8107 / D0066.
constexpr uint64_t kGpsEpochUnixMs = 315'964'800'000ULL;
constexpr uint64_t kGpsWeekMs      = 604'800'000ULL;  //!< 7 days in ms
//! SN-8323: how far before the durable fix period a ToW-domain record may fall
//! and still be treated as a (backward-extrapolated) part of the timeline.
//! Beyond this it is a pre-fix / startup record with no stable GPS time and is
//! tagged SessionOnly/Unknown. Sized to preserve legitimate backward
//! extrapolation (GPS cold-start acquisition is well under an hour) while
//! rejecting the pathological case, which is grossly off — a ToW at the GPS
//! week start while the durable fix is days into the week. A record more than
//! an hour before the first stable fix predates any plausible continuous
//! session (prior power cycle / uninitialized time).
constexpr uint64_t kPreFixGuardMs  = 3'600'000ULL;    //!< 1 hour

/**
 * @brief Convert `(gpsWeek, towMs)` into a Unix-epoch ms timestamp.
 *
 * @param gpsWeek GPS week number (weeks since 1980-01-06).
 * @param towMs   Time-of-week in milliseconds.
 * @return        Milliseconds since Unix epoch (1970-01-01 UTC).
 *
 * @note SN-8107 / D0066.
 */
inline uint64_t gpsToUnixMs(uint32_t gpsWeek, uint64_t towMs) noexcept {
    return static_cast<uint64_t>(gpsWeek) * kGpsWeekMs + kGpsEpochUnixMs + towMs;
}

inline bool isToWBearing(uint32_t did) noexcept {
    for (auto d : kToWBearingDids) if (d == did) return true;
    return false;
}

/**
 * @brief Slope between two sync points in ToW-ms per host-ms.
 *
 * @param a Earlier sync point.
 * @param b Later sync point.
 * @return  ToW-delta divided by host-delta; returns 1.0 when the host
 *          distance is zero (degenerate / colocated points) so
 *          downstream comparisons don't divide by zero.
 */
double slopeBetween(const ISSyncPoint& a, const ISSyncPoint& b) noexcept {
    if (b.hostTimeMs == a.hostTimeMs) return 1.0;
    const double hostDelta = static_cast<double>(b.hostTimeMs) - static_cast<double>(a.hostTimeMs);
    const double towDelta  = static_cast<double>(b.payloadToWMs) - static_cast<double>(a.payloadToWMs);
    return towDelta / hostDelta;
}

/**
 * @brief Walk one segment's `.raw` bytes via `is_comm_parse_byte`,
 *        emitting a sync point for every ToW-bearing-DID packet whose
 *        payload carries a non-zero ToW.
 *
 * The walk per segment matches the writer's per-packet emission
 * pattern (`cDeviceLogRaw::SaveData`), so we recover the same anchors
 * the writer would have flagged with `HAS_TOW`.
 *
 * The walker also tracks the most recent non-sync record's payload-side
 * timestamp (when the payload happens to carry a `tsSec` value but the
 * record is NOT ToW-bearing) so each emitted sync point can carry an
 * `actualHostTimeMs` approximation (see `ISSyncPoint::actualHostTimeMs`
 * doc). For records whose payload doesn't expose a timestamp field at
 * all we leave `actualHostTimeMs == 0` for syncs that don't see a
 * non-sync predecessor.
 *
 * @param reader   Segment reader yielding `.raw` bytes.
 * @param deviceId Source device ID baked into each emitted sync point.
 * @param out      Sync points appended to (caller's vector).
 *
 * @note Host-time recovery heuristic: the writer (`cDeviceLog`) emits
 *       records in arrival order on one host thread, so the host-uptime
 *       of a sync record is millisecond-adjacent to the host-uptime of
 *       the most recently emitted non-sync record. The byte walk only
 *       sees payload bytes (no direct .idx access here), so we
 *       approximate via `cISDataMappings::Timestamp` which reads the
 *       payload's own time field. For DIDs like `DID_PIMU` the
 *       payload's `time` field IS the host uptime — that's the value
 *       we capture.
 *
 * @note SN-8107 / D0066.
 */
//! SN-8339: accumulates one power-on session during the arrival-order scan.
struct SessionAccum {
    uint64_t             arrivalStart = 0;   //!< global arrival index of the session's first record
    std::vector<int64_t> upOffsets;          //!< synced SYS_PARAMS (ToW - upTime) samples in this session
};

//! Median of a copy (sorts in place); 0 for empty.
int64_t medianOf(std::vector<int64_t> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

/**
 * @brief `.dat` equivalent of `scanSegmentForSyncs` (D-119 / SN-8626 / D0082).
 *
 * `.dat` has no wire protocol to parse (D0082): each `ISRecordView::bytes()` is already a bare
 * `p_data_hdr_t` + payload, and `reader.allRecords()` already walks them cleanly (no NMEA/RTCM3/
 * UBX noise to skip, unlike `.raw`'s byte-by-byte comm scan). Same DID_SYS_PARAMS / ToW-bearing
 * logic as the `.raw` path below — see its comments for the "why" of each step.
 */
void scanSegmentForSyncsDat(const ISLogReader& reader,
                            uint64_t deviceId,
                            std::vector<ISSyncPoint>& out,
                            std::vector<int64_t>& upOffsetsOut,
                            uint64_t& arrivalIndex,
                            double& prevUpTimeSec,
                            std::vector<SessionAccum>& sessAccum) {
    uint64_t lastNonSyncHostTimeMs = 0;

    for (auto v : reader.allRecords()) {
        const auto bytes = v.bytes();
        if (!bytes.first || bytes.second < sizeof(p_data_hdr_t)) continue;
        p_data_hdr_t hdr{};
        std::memcpy(&hdr, bytes.first, sizeof(hdr));
        if (sizeof(p_data_hdr_t) + hdr.size > bytes.second) continue;
        const uint8_t* payloadPtr = bytes.first + sizeof(p_data_hdr_t);

        const uint64_t thisArrival = arrivalIndex++;

        if (hdr.id == DID_SYS_PARAMS && hdr.offset == 0 && hdr.size >= sizeof(sys_params_t)) {
            sys_params_t sp2{};
            std::memcpy(&sp2, payloadPtr, sizeof(sp2));
            if (sp2.upTime > 0.0) {
                if (prevUpTimeSec >= 0.0 && sp2.upTime < prevUpTimeSec - 0.5) {
                    sessAccum.push_back(SessionAccum{ thisArrival, {} });
                }
                prevUpTimeSec = sp2.upTime;
            }
            const bool towValid =
                (sp2.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
            if (towValid && sp2.timeOfWeekMs > 0 && sp2.upTime > 0.0) {
                const int64_t upMs = static_cast<int64_t>(sp2.upTime * 1000.0);
                const int64_t off  = static_cast<int64_t>(sp2.timeOfWeekMs) - upMs;
                upOffsetsOut.push_back(off);
                if (!sessAccum.empty()) sessAccum.back().upOffsets.push_back(off);
            }
            // Kyle 2026-09-07 (Option A): DID_SYS_PARAMS wasn't in kToWBearingDids, so even a
            // synced SYS_PARAMS record (towValid) never became a sync point itself -- only used
            // above to calibrate OTHER DIDs' uptime->ToW bridge. A log with SYS_PARAMS as its
            // ONLY DID ever carrying a valid GPS time (no INS/GNSS at all) therefore had zero
            // sync points regardless. Push it as a genuine anchor too, same identity convention
            // as the generic ToW-bearing path below (hostTimeMs == payloadToWMs -- a "sync"
            // record's .idx timestamp field IS the ToW, no separate host-time stored). No
            // gpsWeek: sys_params_t carries no week field, unlike ins_x_t/gnss_pos_t, so this
            // sync point can bridge host-uptime->ToW but never itself supply the epoch anchor
            // (chooseAnchorWeek skips week==0 candidates).
            if (towValid && sp2.timeOfWeekMs > 0) {
                ISSyncPoint sysSp{};
                sysSp.hostTimeMs       = sp2.timeOfWeekMs;
                sysSp.payloadToWMs     = sp2.timeOfWeekMs;
                sysSp.deviceId         = deviceId;
                sysSp.sourceDid        = DID_SYS_PARAMS;
                sysSp.actualHostTimeMs = lastNonSyncHostTimeMs;
                out.push_back(sysSp);
            }
        }

        const double tsSec = cISDataMappings::Timestamp(&hdr, payloadPtr);

        if (!isToWBearing(hdr.id)) {
            if (tsSec > 0.0) {
                lastNonSyncHostTimeMs = static_cast<uint64_t>(tsSec * 1000.0);
            }
            continue;
        }
        if (tsSec <= 0.0) continue;

        const uint64_t towMs = static_cast<uint64_t>(tsSec * 1000.0);
        ISSyncPoint sp{};
        sp.hostTimeMs       = towMs;
        sp.payloadToWMs     = towMs;
        sp.deviceId         = deviceId;
        sp.sourceDid        = hdr.id;
        sp.actualHostTimeMs = lastNonSyncHostTimeMs;
        if (hdr.size >= sizeof(uint32_t)) {
            uint32_t weekRaw = 0;
            std::memcpy(&weekRaw, payloadPtr, sizeof(weekRaw));
            sp.gpsWeek = weekRaw;
        }
        out.push_back(sp);
    }
}

void scanSegmentForSyncs(const ISLogReader& reader,
                         uint64_t deviceId,
                         std::vector<ISSyncPoint>& out,
                         std::vector<int64_t>& upOffsetsOut,
                         uint64_t& arrivalIndex,
                         double& prevUpTimeSec,
                         std::vector<SessionAccum>& sessAccum) {
    // D-119 / SN-8626: .dat has no wire protocol for this function's is_comm_parse_byte scan to
    // find anything in — route to the .dat-native equivalent instead.
    if (reader.format() == ISLogReader::SegmentFormat::Dat) {
        scanSegmentForSyncsDat(reader, deviceId, out, upOffsetsOut, arrivalIndex, prevUpTimeSec, sessAccum);
        return;
    }

    auto bytes = reader.rawBytes();
    if (!bytes.first || bytes.second == 0) return;

    is_comm_instance_t comm{};
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);

    // SN-8107 / D0066: most recent non-sync host-uptime observed during
    // the scan. Updated on every non-ToW-bearing record whose payload
    // exposes a timestamp; carried onto each subsequent sync point until
    // a newer non-sync time arrives.
    uint64_t lastNonSyncHostTimeMs = 0;

    for (std::size_t i = 0; i < bytes.second; ++i) {
        protocol_type_t p = is_comm_parse_byte(&comm, bytes.first[i]);
        if (p != _PTYPE_INERTIAL_SENSE_DATA && p != _PTYPE_INERTIAL_SENSE_CMD) {
            continue;
        }
        const auto& hdr = comm.rxPkt.dataHdr;
        // SN-8339: global record-arrival index (matches ISDeviceLog's ISB-only,
        // arrival-ordered record stream — both parse the same .raw for ISB).
        const uint64_t thisArrival = arrivalIndex++;

        // SN-8323 (uptime unification): DID_SYS_PARAMS carries BOTH the GPS
        // time-of-week (timeOfWeekMs) and the definitive system uptime (upTime,
        // seconds since boot). When the device is synced (timeOfWeekMs > 0),
        // one such record yields the authoritative uptime->ToW offset that
        // bridges every session-uptime value (magnetometer/imu records, and
        // pre-sync "real clock" records whose week/ToW default to uptime until
        // GPS lock). This replaces the fragile per-sync actualHostTimeMs
        // heuristic. (Kyle 2026-07-23: SYS_PARAMS.upTime is the definitive
        // relative uptime; GNSS/INS are the accurate absolute clock.)
        if (hdr.id == DID_SYS_PARAMS && comm.rxPkt.data.ptr &&
            hdr.offset == 0 && hdr.size >= sizeof(sys_params_t)) {
            sys_params_t sp2{};
            std::memcpy(&sp2, comm.rxPkt.data.ptr, sizeof(sp2));
            // SN-8339: a SYS_PARAMS.upTime that DROPS relative to the previous
            // SYS_PARAMS (in arrival order) means the device rebooted — open a
            // new power-on session starting at this record. (Checked on every
            // SYS_PARAMS, regardless of GPS-time validity, since a fresh boot is
            // typically pre-fix.)
            if (sp2.upTime > 0.0) {
                if (prevUpTimeSec >= 0.0 && sp2.upTime < prevUpTimeSec - 0.5) {
                    sessAccum.push_back(SessionAccum{ thisArrival, {} });
                }
                prevUpTimeSec = sp2.upTime;
            }
            // Only trust timeOfWeekMs as GPS ToW when the device says so:
            // HDW_STATUS_GNSS_TIME_OF_WEEK_VALID. Otherwise timeOfWeekMs is
            // LOCAL system time (uptime-like), and differencing it against
            // upTime yields a bogus offset that corrupts the median bridge.
            const bool towValid =
                (sp2.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
            if (towValid && sp2.timeOfWeekMs > 0 && sp2.upTime > 0.0) {
                const int64_t upMs = static_cast<int64_t>(sp2.upTime * 1000.0);
                const int64_t off  = static_cast<int64_t>(sp2.timeOfWeekMs) - upMs;
                upOffsetsOut.push_back(off);                       // global (SN-8323) offset samples
                if (!sessAccum.empty()) sessAccum.back().upOffsets.push_back(off);  // per-session (SN-8339)
            }
            // Kyle 2026-09-07 (Option A) -- same rationale as scanSegmentForSyncsDat's mirror of
            // this block: DID_SYS_PARAMS wasn't in kToWBearingDids, so even a synced record never
            // became a sync point itself, only used to calibrate OTHER DIDs' bridge. Push it too.
            if (towValid && sp2.timeOfWeekMs > 0) {
                ISSyncPoint sysSp{};
                sysSp.hostTimeMs       = sp2.timeOfWeekMs;
                sysSp.payloadToWMs     = sp2.timeOfWeekMs;
                sysSp.deviceId         = deviceId;
                sysSp.sourceDid        = DID_SYS_PARAMS;
                sysSp.actualHostTimeMs = lastNonSyncHostTimeMs;
                out.push_back(sysSp);
            }
        }

        // Probe every record for a payload-side timestamp. ToW-bearing
        // records use this for the sync's payloadToW; non-ToW-bearing
        // records' values feed lastNonSyncHostTimeMs.
        const double tsSec = cISDataMappings::Timestamp(&hdr, comm.rxPkt.data.ptr);

        if (!isToWBearing(hdr.id)) {
            // Non-sync candidate: if the payload carries a usable time
            // field, snapshot it. Most non-sync DIDs (DID_PIMU, DID_IMU,
            // etc.) carry a host-uptime time field in seconds;
            // cISDataMappings::Timestamp converts.
            if (tsSec > 0.0) {
                lastNonSyncHostTimeMs = static_cast<uint64_t>(tsSec * 1000.0);
            }
            continue;
        }

        if (tsSec <= 0.0) continue;  // payload's ToW field is zero / absent.

        const uint64_t towMs = static_cast<uint64_t>(tsSec * 1000.0);
        ISSyncPoint sp{};
        // v2 .idx schema collapse: the writer's rec.timestamp for
        // HAS_TOW records is also the ToW (no separate host-time stored
        // in the .idx). Carry the same value in both fields; the
        // .raw-recovered host-time at sync rides on actualHostTimeMs
        // (set below from the byte scan's lastNonSyncHostTimeMs tracker).
        sp.hostTimeMs        = towMs;
        sp.payloadToWMs      = towMs;
        sp.deviceId          = deviceId;
        sp.sourceDid         = hdr.id;
        // SN-8107 / D0066: recovered host-uptime at sync time.
        sp.actualHostTimeMs  = lastNonSyncHostTimeMs;
        // SN-8107 / D0066: GPS week from payload. All ToW-bearing DIDs
        // (ins_1_t, ins_2_t, ins_3_t, ins_4_t, gnss_pos_t) start with a
        // uint32_t week — read it from the first 4 bytes of the payload.
        // Zero means the device hasn't established a GPS week yet (still
        // searching); we keep the sync point but the resolver won't
        // epoch-anchor against it.
        if (comm.rxPkt.data.ptr && comm.rxPkt.dataHdr.size >= sizeof(uint32_t)) {
            uint32_t weekRaw = 0;
            std::memcpy(&weekRaw, comm.rxPkt.data.ptr, sizeof(weekRaw));
            sp.gpsWeek = weekRaw;
        }
        out.push_back(sp);
    }
}

/**
 * @brief SN-8323: choose the epoch-anchor GPS week from the log's durable,
 *        consistent fix period.
 *
 * The whole log is epoch-anchored to one GPS week (correct for a log spanning
 * < 1 GPS week — the common case). Picking `syncPoints_.front()` is wrong: the
 * sync list is sorted by ToW, so the front is typically a smallest-ToW pre-fix
 * `gpsWeek == 0` record (device still searching) — leaving the log unanchored
 * (ToW-only ~1980) while already-absolute records show the real year.
 *
 * During startup the reported time can be unstable (week 0, or a brief
 * transient/garbage week) before the device settles into a durable fix. So we
 * do NOT trust any single record or a raw popularity count. Instead, for each
 * non-zero week we measure the ToW SPAN it covers across the log; the week
 * backed by the widest span is the one the device held a stable fix at. A
 * startup transient covers a tiny span (a few close-together records) and loses
 * to the sustained fix. Ties break toward more sync points, then the larger
 * (more recent) week.
 *
 * @return  The durable-fix GPS week, or 0 if no non-zero week is present
 *          (caller falls back to ToW-only, pre-D0066 behavior).
 */
uint32_t chooseAnchorWeek(const std::vector<ISSyncPoint>& syncs) {
    struct Agg { uint64_t minTow; uint64_t maxTow; uint32_t count; };
    std::map<uint32_t, Agg> byWeek;   // week -> coverage (ordered ascending)
    for (const auto& sp : syncs) {
        if (sp.gpsWeek == 0) continue;
        auto it = byWeek.find(sp.gpsWeek);
        if (it == byWeek.end()) {
            byWeek.emplace(sp.gpsWeek, Agg{ sp.payloadToWMs, sp.payloadToWMs, 1 });
        } else {
            it->second.minTow = std::min(it->second.minTow, sp.payloadToWMs);
            it->second.maxTow = std::max(it->second.maxTow, sp.payloadToWMs);
            ++it->second.count;
        }
    }
    uint32_t bestWeek = 0, bestCount = 0;
    uint64_t bestSpan = 0;
    for (const auto& [wk, a] : byWeek) {   // ascending week -> larger week wins ties
        const uint64_t span = a.maxTow - a.minTow;
        if (span > bestSpan ||
            (span == bestSpan && a.count >= bestCount)) {
            bestSpan = span; bestCount = a.count; bestWeek = wk;
        }
    }
    return bestWeek;
}

// ============================================================
// Option B (Kyle 2026-09-07): file-timestamp anchor fallback
// ============================================================
//
// A log that never receives an external clock sync (no INS/GNSS DID, and no
// DID_SYS_PARAMS record ever reports HDW_STATUS_GNSS_TIME_OF_WEEK_VALID --
// Option A above still leaves such a log with zero sync points) has no
// payload-derived basis for a wall-clock anchor at all. Rather than resolve
// every record to SessionOnly/Unknown (which RawSeriesBuilder then drops
// entirely -- the log becomes completely unplottable), recover ONE coarse
// anchor from the log's own file: the segment's filename or an ancestor
// directory name, if it looks like cISLogger's own `..._YYYYMMDD_HHMMSS_..`
// convention (the common case -- this IS how cltool/cISLogger names a
// session), else the segment file's last-write time.

namespace fs = std::filesystem;

//! Days from the Unix epoch (1970-01-01) to (y, m, d), proleptic Gregorian.
//! Howard Hinnant's civil_from_days algorithm (public domain), needed because
//! this file is pure C++17 -- no <chrono> calendar support until C++20.
int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) noexcept {
    y -= (m <= 2);
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

//! Matches cISLogger's `..._YYYYMMDD_HHMMSS_...` (or trailing `_YYYYMMDD_HHMMSS`)
//! naming convention in a filename stem or directory name and converts it to
//! Unix-epoch ms. Basic range-sanity-checked (year/month/day/hour/min/sec) to
//! avoid treating an unrelated 8+6-digit run (e.g. a serial number followed by
//! a counter) as a timestamp. Treated as UTC -- the writer stamps local wall-
//! clock at capture time with no timezone recorded, and this anchor is already
//! explicitly a coarse approximation (FileTimeAnchored / TimeConfidence::Unknown),
//! so a timezone-sized offset doesn't change the plottability outcome.
std::optional<uint64_t> parseTimestampFromName(const std::string& name) noexcept {
    static const std::regex re(R"((\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2}))");
    std::smatch m;
    if (!std::regex_search(name, m, re)) return std::nullopt;

    const int year  = std::stoi(m[1].str());
    const int month = std::stoi(m[2].str());
    const int day   = std::stoi(m[3].str());
    const int hour  = std::stoi(m[4].str());
    const int min   = std::stoi(m[5].str());
    const int sec   = std::stoi(m[6].str());
    if (year < 2000 || year > 2100)  return std::nullopt;
    if (month < 1 || month > 12)     return std::nullopt;
    if (day < 1 || day > 31)         return std::nullopt;
    if (hour > 23 || min > 59 || sec > 59) return std::nullopt;

    const int64_t days = daysFromCivil(year, static_cast<unsigned>(month),
                                       static_cast<unsigned>(day));
    const int64_t secs = days * 86400 + hour * 3600 + min * 60 + sec;
    if (secs < 0) return std::nullopt;
    return static_cast<uint64_t>(secs) * 1000ull;
}

//! Tries `path`'s filename stem, then each ancestor directory name (up to 4
//! levels, past which a match is unlikely to actually describe this capture),
//! returning the first that parses as a timestamp.
std::optional<uint64_t> anchorFromPathNames(const fs::path& path) noexcept {
    if (auto v = parseTimestampFromName(path.stem().string())) return v;
    fs::path dir = path.parent_path();
    for (int depth = 0; depth < 4 && !dir.empty(); ++depth) {
        if (auto v = parseTimestampFromName(dir.filename().string())) return v;
        if (dir == dir.parent_path()) break;   // reached root
        dir = dir.parent_path();
    }
    return std::nullopt;
}

//! `path`'s last-write time as Unix-epoch ms, or nullopt on any filesystem error.
//! `std::filesystem::file_time_type`'s clock is unspecified pre-C++20, but on
//! every platform this project targets (libstdc++/libc++) it IS
//! `std::chrono::system_clock` in practice; this is the standard C++17
//! workaround (comparing against `system_clock::now()`/`file_time_type::clock::
//! now()` taken back-to-back) rather than an unavailable `clock_cast` (C++20).
std::optional<uint64_t> fileLastWriteTimeMs(const fs::path& path) noexcept {
    std::error_code ec;
    const auto ftime = fs::last_write_time(path, ec);
    if (ec) return std::nullopt;
    const auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        sctp.time_since_epoch()).count();
    if (ms < 0) return std::nullopt;
    return static_cast<uint64_t>(ms);
}

//! Largest non-zero raw `.idx` timestamp observed across every segment of
//! `log` -- used only as the ctime fallback's session-length estimate (see
//! `deriveFileAnchorMs`). A second full pass over the log, but this only runs
//! when the primary sync scan already found ZERO sync points, which is rare.
uint64_t maxRawTimestampMs(const ISDeviceLog& log) noexcept {
    uint64_t best = 0;
    for (std::size_t s = 0; s < log.segmentCount(); ++s) {
        for (auto v : log.segment(s).allRecords()) {
            best = std::max(best, v.timestamp().value);
        }
    }
    return best;
}

//! Kyle 2026-09-07 (Option B): the wall-clock instant corresponding to
//! host-uptime == 0 for `log`, recovered from its own file when the log has
//! no payload-level sync to derive one from. Tries the EARLIEST segment's
//! filename/ancestor-directory names first (cISLogger stamps the session
//! start into the name, so this needs no adjustment); falls back to that
//! segment's last-write time, adjusted backward by the log's observed
//! session span (last-write time approximates when the file was CLOSED, not
//! opened) so the anchor still lands near session start rather than session
//! end.
//!
//! @return  Anchor in Unix-epoch ms, or `std::nullopt` if neither the name
//!          nor the filesystem produced anything usable (caller's existing
//!          SessionOnly/Unknown fallback still applies in that case).
std::optional<uint64_t> deriveFileAnchorMs(const ISDeviceLog& log) {
    if (log.segmentCount() == 0) return std::nullopt;
    const fs::path& firstSegPath = log.segment(0).path();

    if (auto v = anchorFromPathNames(firstSegPath)) return v;

    if (auto ctimeMs = fileLastWriteTimeMs(firstSegPath)) {
        const uint64_t spanMs = maxRawTimestampMs(log);
        return (*ctimeMs > spanMs) ? (*ctimeMs - spanMs) : *ctimeMs;
    }
    return std::nullopt;
}

} // namespace

// ============================================================
// Detection
// ============================================================

std::vector<ISSyncPoint> ISTimeResolver::detectSyncPoints(const ISDeviceLog& log) {
    std::vector<int64_t> upOffsets;   // discarded here; build() consumes them.
    std::vector<Session> sessions;    // discarded here.
    return detectSyncPointsImpl(log, upOffsets, sessions);
}

std::vector<ISSyncPoint> ISTimeResolver::detectSyncPointsImpl(
    const ISDeviceLog& log, std::vector<int64_t>& upOffsetsOut,
    std::vector<Session>& sessionsOut) {
    std::vector<ISSyncPoint> out;
    const uint64_t deviceId = log.deviceId();

    // SN-8339: partition records into power-on sessions during the scan. The
    // first session starts at arrival index 0; each SYS_PARAMS.upTime drop opens
    // another. Arrival index is global across segments (matches allRecords()).
    uint64_t arrivalIndex  = 0;
    double   prevUpTimeSec = -1.0;
    std::vector<SessionAccum> sessAccum;
    sessAccum.push_back(SessionAccum{ 0, {} });

    for (std::size_t s = 0; s < log.segmentCount(); ++s) {
        scanSegmentForSyncs(log.segment(s), deviceId, out, upOffsetsOut,
                            arrivalIndex, prevUpTimeSec, sessAccum);
    }

    // Finalize sessions: arrivalEnd = next session's start - 1 (last record for
    // the final session); per-session offset = median of its own SYS_PARAMS
    // samples (haveOffset=false when it had none — build() fills the fallback).
    const uint64_t lastArrival = (arrivalIndex == 0) ? 0 : arrivalIndex - 1;
    for (std::size_t i = 0; i < sessAccum.size(); ++i) {
        Session sess{};
        sess.arrivalStart = sessAccum[i].arrivalStart;
        sess.arrivalEnd   = (i + 1 < sessAccum.size())
                                ? (sessAccum[i + 1].arrivalStart - 1)
                                : lastArrival;
        sess.haveOffset   = !sessAccum[i].upOffsets.empty();
        sess.uptimeToTowOffsetMs =
            sess.haveOffset ? medianOf(sessAccum[i].upOffsets) : 0;
        sessionsOut.push_back(sess);
    }

    // Sort by hostTimeMs (the build's downstream expectation). Records
    // are already in arrival order across segments, but a multi-segment
    // log with overlapping ranges or a clock jump may produce out-of-
    // order timestamps — be safe.
    std::sort(out.begin(), out.end(),
              [](const ISSyncPoint& a, const ISSyncPoint& b) {
                  return a.hostTimeMs < b.hostTimeMs;
              });

    // Adjacent-duplicate filtering: many DIDs share a parent update's
    // ToW, so consecutive sync points often carry the same value. The
    // resolver's slope math doesn't benefit from duplicates (and the
    // discontinuity detector needs distinct neighbors).
    out.erase(std::unique(out.begin(), out.end(),
                          [](const ISSyncPoint& a, const ISSyncPoint& b) {
                              return a.hostTimeMs == b.hostTimeMs;
                          }),
              out.end());
    return out;
}

// ============================================================
// Build
// ============================================================

ISExpected<ISTimeResolver>
ISTimeResolver::build(const ISDeviceLog& log) {
    return build(log, kDefaultDiscontinuityThreshold);
}

ISExpected<ISTimeResolver>
ISTimeResolver::build(const ISDeviceLog& log, double threshold) {
    std::vector<int64_t> upOffsets;
    std::vector<Session> sessions;
    auto syncs = detectSyncPointsImpl(log, upOffsets, sessions);

    std::vector<Discontinuity> discs;
    if (syncs.size() >= 3) {
        // Walk consecutive triplets; compare the slope of segment
        // (i-1, i) against (i, i+1). A ratio change beyond `threshold`
        // marks a discontinuity at sync `i`.
        for (std::size_t i = 1; i + 1 < syncs.size(); ++i) {
            const double sBefore = slopeBetween(syncs[i - 1], syncs[i]);
            const double sAfter  = slopeBetween(syncs[i],     syncs[i + 1]);
            if (sBefore <= 0.0 || sAfter <= 0.0) continue;
            const double ratio = std::abs(sAfter - sBefore) / std::max(sBefore, sAfter);
            if (ratio > threshold) {
                discs.push_back(Discontinuity{
                    syncs[i].hostTimeMs,
                    sBefore,
                    sAfter,
                });
            }
        }
    }

    // SN-8323: pick the epoch-anchor week + the start of its durable fix window
    // (earliest ToW at that week) before moving `syncs`.
    const uint32_t anchorWeek = chooseAnchorWeek(syncs);
    uint64_t anchorTowStart = 0;
    uint64_t anchorTowEnd   = 0;
    if (anchorWeek != 0) {
        uint64_t minTow = UINT64_MAX;
        uint64_t maxTow = 0;
        for (const auto& sp : syncs) {
            if (sp.gpsWeek == anchorWeek) {
                minTow = std::min(minTow, sp.payloadToWMs);
                maxTow = std::max(maxTow, sp.payloadToWMs);
            }
        }
        anchorTowStart = (minTow == UINT64_MAX) ? 0 : minTow;
        anchorTowEnd   = maxTow;
    }

    // SN-8323 (uptime unification): authoritative uptime->ToW offset = median of
    // the synced DID_SYS_PARAMS (timeOfWeekMs - upTime) samples. upTime and ToW
    // advance 1:1, so all synced samples agree modulo clock drift; the median
    // resists an occasional transient sample.
    int64_t uptimeToTowOffsetMs = 0;
    bool    haveUptimeOffset    = false;
    if (!upOffsets.empty()) {
        std::sort(upOffsets.begin(), upOffsets.end());
        uptimeToTowOffsetMs = upOffsets[upOffsets.size() / 2];
        haveUptimeOffset    = true;
    }

    // SN-8339: a session with no synced SYS_PARAMS of its own inherits the
    // log-global offset (best available) rather than 0, so its uptime records
    // still bridge; sessions with their own samples keep their per-session median.
    for (auto& sess : sessions) {
        if (!sess.haveOffset && haveUptimeOffset) {
            sess.uptimeToTowOffsetMs = uptimeToTowOffsetMs;
            sess.haveOffset          = true;
        }
    }

    // Kyle 2026-09-07 (Option B): a log with zero sync points (no INS/GNSS, and
    // Option A above found no synced DID_SYS_PARAMS either) has no payload basis
    // for a wall-clock anchor at all -- recover one from the log's own file
    // rather than leave every record SessionOnly/Unknown (RawSeriesBuilder drops
    // those outright).
    uint64_t fileAnchorMs  = 0;
    bool     haveFileAnchor = false;
    if (syncs.empty()) {
        if (auto anchor = deriveFileAnchorMs(log)) {
            fileAnchorMs   = *anchor;
            haveFileAnchor = true;
        }
    }

    return ISTimeResolver{ std::move(syncs), std::move(discs), anchorWeek,
                           anchorTowStart, anchorTowEnd,
                           uptimeToTowOffsetMs, haveUptimeOffset,
                           fileAnchorMs, haveFileAnchor,
                           std::move(sessions) };
}

// ============================================================
// Resolve
// ============================================================

// SN-8339: the two public overloads are thin wrappers; all resolution logic
// lives here, parameterized by the uptime->ToW bridge offset to use. The no-key
// overload passes the log-global offset (byte-identical to pre-SN-8339
// behavior); the arrival-keyed overload passes the per-session offset selected
// from `sessions_`, so a uptime value from any power-on session bridges against
// that session's own median rather than a single global offset that can only be
// right for one boot.
TimeStamp ISTimeResolver::resolveImpl(uint64_t hostTimeMs, uint64_t deviceId,
                                      int64_t uptimeOffsetMs,
                                      bool haveOffset) const {
    // SN-8115: idempotency / already-anchored guard. Legitimate raw inputs are
    // either a GPS time-of-week (always < one week, 604,800,000 ms) or a
    // session-uptime (ms since session start — at most hours). Neither can
    // reach the GPS Unix epoch (1980-01-06 = 315,964,800,000 ms). An input at
    // or beyond that is therefore ALREADY an absolute Unix-ms timestamp — a
    // value that previously went through `resolve()` (re-resolving it must be a
    // no-op for `resolve(resolve(x)) == resolve(x)` to hold), or a
    // wall-clock-poisoned `.idx` span endpoint. Re-anchoring it would
    // double-add the epoch + week offset via `gpsToUnixMs`, producing ~2x the
    // wall-clock (the year-2082 spanEnd seen on the 16-device compass fixture).
    // Pass it through unchanged.
    if (hostTimeMs >= kGpsEpochUnixMs) {
        return TimeStamp::fromResolvedViaSync(hostTimeMs, deviceId,
                                              TimeConfidence::Exact);
    }

    if (syncPoints_.empty()) {
        // No payload-derived anchors. Kyle 2026-09-07 (Option B): if build() found
        // a usable file-timestamp anchor (the log's filename/directory name, or its
        // last-write time), every input here IS host-uptime domain (there's no
        // sync point to have classified it otherwise) -- add the anchor directly.
        if (haveFileAnchor_) {
            return TimeStamp::fromFileTimeAnchored(fileAnchorMs_ + hostTimeMs, deviceId);
        }
        // No anchor of any kind. Best we can do is a SessionOnly tag with the
        // input value passed through.
        return TimeStamp::fromSessionOnly(hostTimeMs, deviceId);
    }

    // SN-8107 / D0066: select a representative GPS week for the epoch anchor.
    // SN-8323: use the durable-fix week (`anchorWeek_`, precomputed in build()
    // as the non-zero week with the widest ToW coverage), NOT
    // `syncPoints_.front().gpsWeek`.
    // The sync list is sorted by ToW, so front() is the smallest-ToW record,
    // which on a pre-GPS-fix log is a week-0 record — anchoring to it left the
    // log unanchored (ToW-only ~1980) while already-absolute records showed the
    // real year, giving a ~46-year mixed-domain span. `firstSync` is still the
    // ToW-frame reference for the session-uptime bridge below. anchorWeek_ == 0
    // (no valid week anywhere) falls back to ToW-only (pre-D0066 behavior).
    const ISSyncPoint& firstSync = syncPoints_.front();
    const uint32_t anchorWeek = anchorWeek_;
    const bool     epochAnchor = (anchorWeek != 0);
    auto unixOrToW = [&](uint64_t towMs) -> uint64_t {
        return epochAnchor ? gpsToUnixMs(anchorWeek, towMs) : towMs;
    };

    // SN-8323 (uptime unification): authoritative uptime->ToW bridge. When a
    // synced DID_SYS_PARAMS supplied the definitive offset, classify the input
    // against the durable-fix ToW window [anchorTowStart_, anchorTowEnd_]:
    //   - already inside the window  -> ToW-domain, fall through to the normal
    //     sync-point resolution below;
    //   - outside, but (input + offset) lands inside -> a session-uptime value
    //     (magnetometer/imu, or a pre-sync "real clock" record whose ToW field
    //     defaulted to uptime until GPS lock) -> bridge via the authoritative
    //     offset.
    // This supersedes the fragile per-sync actualHostTimeMs heuristic (below)
    // and stops tiny pre-sync ToW values from poisoning the bridge — the mag
    // records that used to resolve to the GPS-week start (days early) now land
    // correctly in the fix window. (Kyle 2026-07-23.)
    if (haveOffset && epochAnchor && anchorTowEnd_ >= anchorTowStart_) {
        const int64_t guard    = static_cast<int64_t>(kPreFixGuardMs);
        const int64_t lo       = static_cast<int64_t>(anchorTowStart_);
        const int64_t hi       = static_cast<int64_t>(anchorTowEnd_);
        const int64_t raw      = static_cast<int64_t>(hostTimeMs);
        const bool    rawIsTow = (raw >= lo - guard && raw <= hi + guard);
        if (!rawIsTow) {
            const int64_t bridged = raw + uptimeOffsetMs;
            if (bridged >= lo - guard && bridged <= hi + guard) {
                const uint64_t towMs = (bridged < 0) ? 0u
                                                     : static_cast<uint64_t>(bridged);
                const TimeConfidence conf =
                    (bridged < lo) ? TimeConfidence::ExtrapolatedBackward
                                   : TimeConfidence::Interpolated;
                return TimeStamp::fromResolvedViaSync(
                    gpsToUnixMs(anchorWeek, towMs), deviceId, conf);
            }
        }
    }

    // SN-8107 / D0066: cross-domain bridge. v2 .idx puts sync records'
    // timestamp field in the GPS-ToW domain (hundreds of millions of ms
    // into the GPS week) while non-sync records' timestamp field is host
    // uptime (small ms since session start). An input hostTimeMs that's
    // dramatically smaller than the first sync's ToW is a session-uptime
    // query — translate it into the ToW frame using the recovered
    // actualHostTimeMs of the first sync, then epoch-anchor the result
    // if GPS week is known.
    if (firstSync.actualHostTimeMs > 0 &&
        hostTimeMs < firstSync.payloadToWMs / 2) {
        const int64_t offset =
            static_cast<int64_t>(firstSync.payloadToWMs) -
            static_cast<int64_t>(firstSync.actualHostTimeMs);
        const int64_t bridged = static_cast<int64_t>(hostTimeMs) + offset;
        const uint64_t towMs = (bridged < 0) ? 0u : static_cast<uint64_t>(bridged);
        return TimeStamp::fromResolvedViaSync(unixOrToW(towMs), deviceId,
                                              TimeConfidence::ExtrapolatedBackward);
    }

    // SN-8323 (part 2): a ToW-domain input that falls well before the durable
    // fix period began is a pre-fix / startup record — the device had no stable
    // GPS time yet (e.g. a ToW near the GPS week start while the fix is days
    // into the week). Epoch-anchoring it would place it at a bogus early time
    // and drag the log extent back (the "leading gap"). Tag it
    // SessionOnly/Unknown — the same contract as a record with no anchor at all
    // — so every SDK consumer excludes it from the timeline + extent uniformly.
    if (epochAnchor && anchorTowStart_ > kPreFixGuardMs &&
        hostTimeMs + kPreFixGuardMs < anchorTowStart_) {
        return TimeStamp::fromSessionOnly(hostTimeMs, deviceId);
    }

    // Binary search for the first sync point whose hostTimeMs >= input.
    auto it = std::lower_bound(
        syncPoints_.begin(), syncPoints_.end(), hostTimeMs,
        [](const ISSyncPoint& sp, uint64_t v) { return sp.hostTimeMs < v; });

    if (it != syncPoints_.end() && it->hostTimeMs == hostTimeMs) {
        // Exact match against a sync point.
        return TimeStamp::fromPayloadToW(unixOrToW(it->payloadToWMs), deviceId);
    }

    if (it == syncPoints_.begin()) {
        // Before the first sync. Project backward using the slope of
        // the first two sync points (or 1.0 if only one sync point).
        const ISSyncPoint& s0 = syncPoints_.front();
        double slope = 1.0;
        if (syncPoints_.size() >= 2) {
            slope = slopeBetween(syncPoints_[0], syncPoints_[1]);
        }
        const double delta = static_cast<double>(hostTimeMs) - static_cast<double>(s0.hostTimeMs);
        const double tow   = static_cast<double>(s0.payloadToWMs) + slope * delta;
        const uint64_t towMs = (tow < 0.0) ? 0u : static_cast<uint64_t>(tow);
        return TimeStamp::fromResolvedViaSync(unixOrToW(towMs), deviceId,
                                              TimeConfidence::ExtrapolatedBackward);
    }

    if (it == syncPoints_.end()) {
        // Past the last sync. Project forward using the slope of the
        // last two sync points (or 1.0 if only one).
        const ISSyncPoint& sLast = syncPoints_.back();
        double slope = 1.0;
        if (syncPoints_.size() >= 2) {
            const ISSyncPoint& sPrev = syncPoints_[syncPoints_.size() - 2];
            slope = slopeBetween(sPrev, sLast);
        }
        const double delta = static_cast<double>(hostTimeMs) - static_cast<double>(sLast.hostTimeMs);
        const double tow   = static_cast<double>(sLast.payloadToWMs) + slope * delta;
        const uint64_t towMs = (tow < 0.0) ? 0u : static_cast<uint64_t>(tow);
        return TimeStamp::fromResolvedViaSync(unixOrToW(towMs), deviceId,
                                              TimeConfidence::ExtrapolatedForward);
    }

    // Interpolation between `prev` (it - 1) and `it`.
    const ISSyncPoint& prev = *(it - 1);
    const ISSyncPoint& next = *it;
    const double hostSpan = static_cast<double>(next.hostTimeMs) - static_cast<double>(prev.hostTimeMs);
    const double towSpan  = static_cast<double>(next.payloadToWMs) - static_cast<double>(prev.payloadToWMs);
    const double frac = (hostSpan == 0.0)
                      ? 0.0
                      : (static_cast<double>(hostTimeMs) - static_cast<double>(prev.hostTimeMs)) / hostSpan;
    const double tow  = static_cast<double>(prev.payloadToWMs) + frac * towSpan;
    return TimeStamp::fromResolvedViaSync(unixOrToW(static_cast<uint64_t>(tow)),
                                          deviceId,
                                          TimeConfidence::Interpolated);
}

TimeStamp ISTimeResolver::resolve(uint64_t hostTimeMs, uint64_t deviceId) const {
    // No arrival key: bridge with the log-global uptime offset. Byte-identical
    // to the pre-SN-8339 single-offset behavior — every existing caller keeps
    // its exact results and multi-boot logs simply use one offset (best effort).
    return resolveImpl(hostTimeMs, deviceId, uptimeToTowOffsetMs_,
                       haveUptimeOffset_);
}

TimeStamp ISTimeResolver::resolve(uint64_t hostTimeMs, uint64_t deviceId,
                                  uint64_t arrivalIndex) const {
    // SN-8339: with a known arrival index and more than one power-on session,
    // pick the session whose arrival range covers this record and bridge with
    // that session's own offset. A single-session log (the common case) is
    // identical to the no-key path, so this overload is a strict superset.
    if (sessions_.size() > 1) {
        for (const auto& sess : sessions_) {
            if (arrivalIndex >= sess.arrivalStart &&
                arrivalIndex <= sess.arrivalEnd) {
                return resolveImpl(hostTimeMs, deviceId,
                                   sess.uptimeToTowOffsetMs, sess.haveOffset);
            }
        }
    }
    return resolveImpl(hostTimeMs, deviceId, uptimeToTowOffsetMs_,
                       haveUptimeOffset_);
}

ISTimeResolver::Stats ISTimeResolver::computeStats(const ISDeviceLog& log) const {
    Stats s{};
    for (auto v : log.allRecords()) {
        const TimeStamp t = resolve(v.timestamp().value, log.deviceId());
        switch (t.confidence) {
            case TimeConfidence::Exact:                ++s.exact;        break;
            case TimeConfidence::Interpolated:         ++s.interpolated; break;
            case TimeConfidence::ExtrapolatedForward:  ++s.extrapFwd;    break;
            case TimeConfidence::ExtrapolatedBackward: ++s.extrapBack;   break;
            case TimeConfidence::Unknown:              ++s.unknown;      break;
        }
    }
    return s;
}

} // namespace inertial_sense
