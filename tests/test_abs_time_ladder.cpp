/**
 * @file test_abs_time_ladder.cpp
 * @brief SN-8784 — the self-healing ladder: when the absolute-time mechanism disagrees with
 *        itself across the two read paths, decide WHOSE fault it is.
 *
 * Kyle's design, 2026-10-02. Four rungs:
 *
 *   1. the fast path — trust the on-disk `.idx`;
 *   2. on failure, re-run from a full `.raw` scan that ignores the `.idx` entirely;
 *   3. if THAT passes, the sidecar is implicated — rebuild it and retry;
 *   4. still failing = fail loudly, because the mechanism itself is wrong.
 *
 * Plus the attribution: diff the original sidecar against the rebuilt one and say which class of
 * difference explains the failure. An identical rebuild that still fails EXONERATES the sidecar and
 * convicts the mechanism.
 *
 * Two things this file does not do, both deliberate:
 *
 * - **It never mutates a log.** Every open of an original passes `persistRebuiltIndex = false`, and
 *   rung 3 rebuilds into a TEMP COPY (Kyle picked this over rebuilding in place). The corpus is
 *   read-only as far as this file is concerned, and `LadderNeverMutatesTheLogDirectory` asserts it.
 * - **It does not use the round-trip fixed point as the rung-1 property on its own.** A sidecar can
 *   be WRONG and perfectly self-consistent, and then every fixed-point check passes — measured
 *   under SN-8784 when a deliberately broken index still round-tripped cleanly. So the rung-1
 *   property is AGREEMENT BETWEEN THE TWO PATHS, which is the only thing a liar cannot satisfy.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>

#include "com_manager.h"  // first — see test_time_resolver.cpp

#include "DeviceLog.h"
#include "ISDataMappings.h"
#include "ISDeviceLog.h"
#include "ISFileManager.h"
#include "ISLog.h"
#include "ISLogIndex.h"
#include "ISLogReader.h"
#include "ISLogger.h"
#include "ISTimeResolver.h"
#include "data_sets.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

#include <unistd.h>

using namespace inertial_sense;
namespace fs = std::filesystem;

namespace {

constexpr uint16_t kFixtureHwId   = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
constexpr uint32_t kFixtureSerial = 777321u;

// ===========================================================================
// Sidecar diff — the four classes the attribution is expressed in.
// ===========================================================================

/**
 * @brief How an original `.idx` differs from one rebuilt from the same `.raw`.
 *
 * The classes are Kyle's: a record absent, a byte offset different, a timestamp different, the
 * header anchor different. The timestamp class is SPLIT, because measuring it showed the two halves
 * mean opposite things:
 *
 * - **declined** — the rebuild wrote 0 where the original had a value. Expected and benign: a
 *   record whose DID carries no payload time gets the host's RECEIPT time stamped in at write, and
 *   a byte scan can never recover that (D0099 / D0096 path 3). Measured on a healthy 5,170-record
 *   fixture: exactly one such record. Treating this as a difference would convict every log.
 * - **contradicts** — both values are non-zero and they disagree. This is the sidecar and the bytes
 *   making incompatible claims about when something happened, and it is the real signal.
 */
struct SidecarDiff {
    bool        parsed            = false;
    std::size_t recordsOriginal   = 0;
    std::size_t recordsRebuilt    = 0;
    std::size_t recordAbsent      = 0;   //!< Count difference: records one side does not list.
    std::size_t didDiffers        = 0;
    std::size_t byteOffsetDiffers = 0;
    std::size_t timestampDeclined = 0;   //!< Rebuild wrote 0 where the original had a value.
    std::size_t timestampContradicts = 0;//!< Both non-zero and different — the real signal.
    /**
     * @brief The header anchor was ABSENT and the rebuild supplied one. Expected, not a conflict.
     *
     * `cISLogger` does not write `anchor_offset_ms`; the anchor cascade only exists on the read
     * side, so a rebuild of ANY writer-produced sidecar fills in a field that was zero. Measured on
     * a healthy 400-record fixture: this is the ONLY difference a rebuild produces (`0 ->
     * 1,790,997,364,000`). Counting it as a conflict made `exoneratesTheSidecar` false for every
     * writer-produced log, which would have made rung 4 unreachable — it would report
     * `RebuildDidNotHeal` in exactly the case whose whole purpose is to convict the mechanism.
     */
    bool        headerAnchorAdded       = false;
    //! Both anchors are non-zero and they disagree. The real signal, same split as the timestamps.
    bool        headerAnchorContradicts = false;
    int64_t     anchorOriginal    = 0;
    int64_t     anchorRebuilt     = 0;

    //! Is the rebuild materially identical — i.e. does the sidecar deserve to be exonerated?
    bool exoneratesTheSidecar() const noexcept {
        return parsed && recordAbsent == 0 && didDiffers == 0 && byteOffsetDiffers == 0
            && timestampContradicts == 0 && !headerAnchorContradicts;
    }

    std::string summary() const {
        if (!parsed) return "sidecar diff unavailable (one side did not parse)";
        std::string s = "records " + std::to_string(recordsOriginal) + " vs "
                      + std::to_string(recordsRebuilt)
                      + "; absent=" + std::to_string(recordAbsent)
                      + " did=" + std::to_string(didDiffers)
                      + " offset=" + std::to_string(byteOffsetDiffers)
                      + " ts-contradicts=" + std::to_string(timestampContradicts)
                      + " ts-declined=" + std::to_string(timestampDeclined)
                      + " anchor=" + (headerAnchorContradicts ? "CONTRADICTS "
                                      : headerAnchorAdded      ? "added "
                                                               : "same")
                      + (headerAnchorAdded || headerAnchorContradicts
                         ? (std::to_string(anchorOriginal) + "->" + std::to_string(anchorRebuilt))
                         : std::string{});
        return s;
    }
};

//! Reads a whole v2 `.idx` into header + records. Empty optional when it is not a v2 sidecar.
struct ParsedIdx {
    idx::is_log_idx_header_t              header{};
    std::vector<idx::is_log_idx_record_v2_t> records;
};

bool parseIdxFile(const fs::path& p, ParsedIdx& out) {
    std::ifstream in(p, std::ios::binary);
    if (!in.good()) return false;
    const std::vector<uint8_t> buf((std::istreambuf_iterator<char>(in)),
                                    std::istreambuf_iterator<char>());
    if (buf.size() < idx::IS_LOG_IDX_HEADER_SIZE) return false;
    auto h = idx::parseHeader(buf.data());
    if (!h) return false;                      // v1 has no header at all; not comparable here
    out.header = *h;
    const std::size_t stride = out.header.record_size ? out.header.record_size
                                                      : idx::IS_LOG_IDX_RECORD_V2_SIZE;
    if (stride == 0) return false;
    const std::size_t n = (buf.size() - idx::IS_LOG_IDX_HEADER_SIZE) / stride;
    out.records.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.records.push_back(
            idx::parseRecord(buf.data() + idx::IS_LOG_IDX_HEADER_SIZE + i * stride, stride));
    }
    return true;
}

SidecarDiff diffSidecars(const fs::path& original, const fs::path& rebuilt) {
    SidecarDiff d;
    ParsedIdx a, b;
    if (!parseIdxFile(original, a) || !parseIdxFile(rebuilt, b)) return d;
    d.parsed          = true;
    d.recordsOriginal = a.records.size();
    d.recordsRebuilt  = b.records.size();
    d.recordAbsent    = (a.records.size() > b.records.size())
                      ? a.records.size() - b.records.size()
                      : b.records.size() - a.records.size();

    // Positional join: both describe the same `.raw` in arrival order, so index i is the same
    // record on both sides. A count difference is reported as `recordAbsent` rather than silently
    // shifting the comparison.
    const std::size_t n = std::min(a.records.size(), b.records.size());
    for (std::size_t i = 0; i < n; ++i) {
        const auto& ra = a.records[i];
        const auto& rb = b.records[i];
        if (ra.did != rb.did)       ++d.didDiffers;
        if (ra.offset != rb.offset)  ++d.byteOffsetDiffers;
        if (ra.timestamp != rb.timestamp) {
            if (rb.timestamp == 0) ++d.timestampDeclined;
            else                   ++d.timestampContradicts;
        }
    }

    d.anchorOriginal = a.header.anchor_offset_ms;
    d.anchorRebuilt  = b.header.anchor_offset_ms;
    if (d.anchorOriginal != d.anchorRebuilt) {
        if (d.anchorOriginal == 0) d.headerAnchorAdded       = true;
        else                       d.headerAnchorContradicts = true;
    }
    return d;
}

// ===========================================================================
// The properties each rung is judged on.
// ===========================================================================

/**
 * @brief Outcome of the round-trip check.
 *
 * `NothingToCheck` is NOT a failure. Kyle's ruling, 2026-10-02: a log may legitimately have no
 * clock source at all, and that is an expected, first-class result. Collapsing it into "violated"
 * made the ladder convict the mechanism on a corpus log whose FIRST DEVICE is an empty segment
 * (`ppd_LogQAQW713/20260716_000102` device 123412: one segment, zero records) while its second
 * device placed 123,148 records perfectly.
 */
enum class CycleResult { Held, Violated, NothingToCheck };

//! The round-trip fixed point, on sampled seeds. Necessary, never sufficient — see the file header.
CycleResult cycleHolds(const ISDeviceLog& dl, const ISTimeResolver& R, std::string& why) {
    constexpr std::size_t kPerSegment = 8;
    std::size_t checked = 0;
    for (std::size_t s = 0; s < dl.segmentCount(); ++s) {
        const std::size_t n = dl.segment(s).recordCount();
        if (n == 0) continue;
        const std::size_t step = n > kPerSegment ? n / kPerSegment : 1;
        for (std::size_t k = 0; k < n; k += step) {
            const AbsTimeResult t0 = R.resolve(dl, s, k);
            if (!t0.valid) continue;
            const SegmentOffset p0 = R.resolveTimeToSegmentOffset(dl, t0.absoluteMs);
            if (!p0.valid) {
                why = "a borne instant did not map back to any record";
                return CycleResult::Violated;
            }
            const AbsTimeResult t1 = R.resolve(dl, p0.segmentIndex, p0.recordIndex);
            if (!t1.valid) {
                why = "a position the inverse returned did not resolve forward";
                return CycleResult::Violated;
            }
            const SegmentOffset p1 = R.resolveTimeToSegmentOffset(dl, t1.absoluteMs);
            if (!p1.valid || p1.segmentIndex != p0.segmentIndex
                || p1.recordIndex != p0.recordIndex) {
                why = "position is not a fixed point";
                return CycleResult::Violated;
            }
            const AbsTimeResult t2 = R.resolve(dl, p1.segmentIndex, p1.recordIndex);
            if (!t2.valid || t2.absoluteMs != t1.absoluteMs) {
                why = "time is not a fixed point";
                return CycleResult::Violated;
            }
            ++checked;
        }
    }
    if (checked == 0) {
        why = "no record could be placed, so there was nothing to check";
        return CycleResult::NothingToCheck;
    }
    return CycleResult::Held;
}

//! Counts of how two read paths of the SAME log disagree.
struct Agreement {
    std::size_t records        = 0;
    std::size_t countMismatch  = 0;
    std::size_t didDiffers     = 0;
    std::size_t offsetDiffers  = 0;
    std::size_t timeDiffers    = 0;   //!< Both raw values present, resolved instants differ.
    std::size_t timeIncomparable = 0; //!< One side has no raw value, so there is nothing to compare.
    std::string firstDetail;

    bool agree() const noexcept {
        return countMismatch == 0 && didDiffers == 0 && offsetDiffers == 0 && timeDiffers == 0;
    }
    std::string summary() const {
        return "records=" + std::to_string(records)
             + " countMismatch=" + std::to_string(countMismatch)
             + " did=" + std::to_string(didDiffers)
             + " offset=" + std::to_string(offsetDiffers)
             + " time=" + std::to_string(timeDiffers)
             + " incomparable=" + std::to_string(timeIncomparable)
             + (firstDetail.empty() ? std::string{} : ("  first: " + firstDetail));
    }
};

/**
 * @brief Do two readings of one log place its records identically?
 *
 * DIDs and byte offsets are properties of the byte stream, so they must match exactly. Resolved
 * instants are compared only where BOTH paths have a raw value to work from: where one side's
 * sidecar value is absent the two are not making competing claims, and demanding equality there
 * would fail on every healthy log (the receipt-time case in @ref SidecarDiff).
 */
Agreement agreementBetween(const ISDeviceLog& a, const ISTimeResolver& ra,
                           const ISDeviceLog& b, const ISTimeResolver& rb) {
    Agreement g;
    if (a.segmentCount() != b.segmentCount()) {
        ++g.countMismatch;
        g.firstDetail = "segment count " + std::to_string(a.segmentCount()) + " vs "
                      + std::to_string(b.segmentCount());
        return g;
    }
    for (std::size_t s = 0; s < a.segmentCount(); ++s) {
        const std::size_t na = a.segment(s).recordCount();
        const std::size_t nb = b.segment(s).recordCount();
        if (na != nb) {
            ++g.countMismatch;
            if (g.firstDetail.empty()) {
                g.firstDetail = "segment " + std::to_string(s) + " record count "
                              + std::to_string(na) + " vs " + std::to_string(nb);
            }
            continue;
        }
        for (std::size_t k = 0; k < na; ++k) {
            const ISRecordView va = a.segment(s).recordAt(k);
            const ISRecordView vb = b.segment(s).recordAt(k);
            ++g.records;
            if (va.did() != vb.did()) {
                ++g.didDiffers;
                if (g.firstDetail.empty()) {
                    g.firstDetail = "seg " + std::to_string(s) + " rec " + std::to_string(k)
                                  + " did " + std::to_string(va.did()) + " vs "
                                  + std::to_string(vb.did());
                }
                continue;
            }
            if (va.offsetInFile() != vb.offsetInFile()) {
                ++g.offsetDiffers;
                if (g.firstDetail.empty()) {
                    g.firstDetail = "seg " + std::to_string(s) + " rec " + std::to_string(k)
                                  + " offset " + std::to_string(va.offsetInFile()) + " vs "
                                  + std::to_string(vb.offsetInFile());
                }
                continue;
            }
            if (va.timestamp().value == 0 || vb.timestamp().value == 0) {
                ++g.timeIncomparable;
                continue;
            }
            const AbsTimeResult ta = ra.resolve(a, s, k);
            const AbsTimeResult tb = rb.resolve(b, s, k);
            if (ta.valid != tb.valid || (ta.valid && ta.absoluteMs != tb.absoluteMs)) {
                ++g.timeDiffers;
                if (g.firstDetail.empty()) {
                    g.firstDetail = "seg " + std::to_string(s) + " rec " + std::to_string(k)
                                  + " did " + std::to_string(va.did())
                                  + "(" + cISDataMappings::DataName(va.did()) + ")"
                                  + " resolved " + std::to_string(ta.absoluteMs) + " vs "
                                  + std::to_string(tb.absoluteMs);
                }
            }
        }
    }
    return g;
}

// ===========================================================================
// The ladder itself.
// ===========================================================================

enum class LadderVerdict {
    CouldNotOpen = 0,
    //! No record of this device can be placed at all. An EXPECTED outcome, not a failure — a log
    //! may legitimately have no clock source (Kyle, 2026-10-02). Reported, never counted as a pass.
    NothingToJudge,
    Healthy,                 //!< Rung 1: the two paths agree and the round trip closes.
    SidecarHealedByRebuild,  //!< Rung 3: rebuilding the sidecar made the disagreement go away.
    RebuildDidNotHeal,       //!< Rung 3 ran, differences were found, and it still disagrees.
    MechanismAtFault,        //!< Rung 4: the scan path fails too, or the rebuild was identical.
};

const char* verdictName(LadderVerdict v) noexcept {
    switch (v) {
        case LadderVerdict::CouldNotOpen:           return "CouldNotOpen";
        case LadderVerdict::NothingToJudge:         return "NothingToJudge";
        case LadderVerdict::Healthy:                return "Healthy";
        case LadderVerdict::SidecarHealedByRebuild: return "SidecarHealedByRebuild";
        case LadderVerdict::RebuildDidNotHeal:      return "RebuildDidNotHeal";
        case LadderVerdict::MechanismAtFault:       return "MechanismAtFault";
    }
    return "?";
}

struct LadderReport {
    LadderVerdict verdict = LadderVerdict::CouldNotOpen;
    int           rungsClimbed = 0;
    Agreement     rung1;          //!< fast path vs scan path, as found
    Agreement     rung3;          //!< rebuilt-and-trusted vs scan path
    SidecarDiff   diff;           //!< original vs rebuilt, when rung 3 ran
    std::string   detail;
};

fs::path makeTempDir(const std::string& hint) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "/tmp/is_ladder_%s_%d_%ld", hint.c_str(), ::getpid(),
                  static_cast<long>(::time(nullptr)));
    fs::path p = buf;
    std::error_code ec;
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    return p;
}

//! Copies a log directory's segments and sidecars. The copy is what rung 3 is allowed to rewrite.
bool copyLogDir(const fs::path& from, const fs::path& to) {
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(from, ec)) {
        if (ec) return false;
        if (!e.is_regular_file()) continue;
        const std::string ext = e.path().extension().string();
        if (ext != ".raw" && ext != ".dat" && ext != ".idx") continue;
        fs::copy_file(e.path(), to / e.path().filename(), fs::copy_options::overwrite_existing, ec);
        if (ec) return false;
    }
    return true;
}

/**
 * @brief Climb the ladder for one device inside one log directory.
 *
 * @param dir       Log directory. NEVER written to: every open of it suppresses persistence.
 * @param devIndex  Which device of the directory to judge (0 = the first).
 * @return          The verdict and the evidence behind it.
 */
LadderReport climbLadder(const fs::path& dir, std::size_t devIndex = 0) {
    LadderReport out;

    ISLogReader::OpenOptions readOnly;              // trust the sidecar, but never write one
    readOnly.persistRebuiltIndex = false;
    ISLogReader::OpenOptions scanOnly;              // ignore the sidecar, and never write one
    scanOnly.ignoreOnDiskIndex   = true;
    scanOnly.persistRebuiltIndex = false;

    auto fast = ISLog::openDirectory(dir, readOnly);
    auto scan = ISLog::openDirectory(dir, scanOnly);
    if (!fast || !scan) { out.detail = "openDirectory failed"; return out; }
    if (fast->deviceIds().size() <= devIndex || scan->deviceIds().size() <= devIndex) {
        out.detail = "no such device index";
        return out;
    }
    const uint64_t devId = fast->deviceIds()[devIndex];
    const ISDeviceLog& fastDl = fast->device(devId);
    const ISDeviceLog& scanDl = scan->device(devId);

    auto fastR = ISTimeResolver::build(fastDl);
    auto scanR = ISTimeResolver::build(scanDl);
    if (!fastR || !scanR) { out.detail = "resolver build failed"; return out; }

    // ---- Rung 1: the fast path, judged against the scan path.
    out.rungsClimbed = 1;
    out.rung1 = agreementBetween(fastDl, *fastR, scanDl, *scanR);
    std::string why;
    const CycleResult fastCycle = cycleHolds(fastDl, *fastR, why);
    if (fastCycle == CycleResult::NothingToCheck && out.rung1.agree()) {
        // Both paths place nothing and they agree about placing nothing. There is no property to
        // violate here, and calling it a fault was a defect in this harness, not in the mechanism.
        out.verdict = LadderVerdict::NothingToJudge;
        out.detail  = why;
        return out;
    }
    if (out.rung1.agree() && fastCycle == CycleResult::Held) {
        out.verdict = LadderVerdict::Healthy;
        return out;
    }
    out.detail = out.rung1.agree() ? ("fast path round trip: " + why)
                                   : ("paths disagree: " + out.rung1.summary());

    // ---- Rung 2: does the scan path, which owes the sidecar nothing, hold up on its own?
    out.rungsClimbed = 2;
    std::string scanWhy;
    if (cycleHolds(scanDl, *scanR, scanWhy) != CycleResult::Held) {
        // Both paths broken. Nothing about the sidecar can explain that.
        out.verdict = LadderVerdict::MechanismAtFault;
        out.detail += "; the scan path fails too: " + scanWhy;
        return out;
    }

    // ---- Rung 3: the sidecar is implicated. Rebuild it — on a COPY — and retry the fast path.
    out.rungsClimbed = 3;
    const fs::path work = makeTempDir("rung3");
    if (!copyLogDir(dir, work)) {
        out.detail += "; could not copy the log to a temp dir, so rung 3 did not run";
        out.verdict = LadderVerdict::RebuildDidNotHeal;
        return out;
    }

    // The sidecar to diff is the one belonging to THIS DEVICE's first segment. Taking whichever
    // `.idx` happened to sort first attributed a multi-device log to the wrong device's file and
    // reported "diff unavailable".
    fs::path idxName;
    if (fastDl.segmentCount() > 0) {
        fs::path p = fastDl.segment(0).path();
        p.replace_extension(".idx");
        idxName = p.filename();
    }

    ISLogReader::OpenOptions rebuild;
    rebuild.ignoreOnDiskIndex   = true;
    rebuild.persistRebuiltIndex = true;            // writes into the COPY, never the original
    {
        auto rebuilt = ISLog::openDirectory(work, rebuild);
        if (!rebuilt) {
            out.detail += "; rebuild open failed";
            out.verdict = LadderVerdict::RebuildDidNotHeal;
            std::error_code ec; fs::remove_all(work, ec);
            return out;
        }
    }

    // Attribute: original sidecar vs the one the rebuild just wrote. First pair wins; a log with
    // several segments gets its first segment attributed, which is enough to name the class.
    if (!idxName.empty() && fs::exists(dir / idxName)) {
        out.diff = diffSidecars(dir / idxName, work / idxName);
    }

    auto healed = ISLog::openDirectory(work, readOnly);   // fast path over the REBUILT sidecar
    if (!healed || healed->deviceIds().size() <= devIndex) {
        out.detail += "; reopen after rebuild failed";
        out.verdict = LadderVerdict::RebuildDidNotHeal;
        std::error_code ec; fs::remove_all(work, ec);
        return out;
    }
    const uint64_t healedDev = healed->deviceIds()[devIndex];
    const ISDeviceLog& healedDl = healed->device(healedDev);
    auto healedR = ISTimeResolver::build(healedDl);
    if (healedR) {
        out.rung3 = agreementBetween(healedDl, *healedR, scanDl, *scanR);
        std::string healedWhy;
        if (out.rung3.agree() && cycleHolds(healedDl, *healedR, healedWhy) == CycleResult::Held) {
            out.verdict = LadderVerdict::SidecarHealedByRebuild;
        } else {
            out.verdict = LadderVerdict::RebuildDidNotHeal;
            out.detail += "; after rebuild: " + out.rung3.summary();
        }
    } else {
        out.verdict = LadderVerdict::RebuildDidNotHeal;
        out.detail += "; resolver build failed after rebuild";
    }

    // ---- Rung 4: an identical rebuild that still fails exonerates the sidecar.
    if (out.verdict == LadderVerdict::RebuildDidNotHeal && out.diff.exoneratesTheSidecar()) {
        out.verdict = LadderVerdict::MechanismAtFault;
        out.detail += "; the rebuild is materially identical to the original, so the sidecar is "
                      "exonerated and the mechanism is at fault";
    }

    std::error_code ec;
    fs::remove_all(work, ec);
    return out;
}

// ===========================================================================
// Fixtures
// ===========================================================================

ins_2_t makeIns2(double towSec) {
    ins_2_t s{};
    s.week       = 2300;
    s.timeOfWeek = towSec;
    s.qn2b[0]    = 1.0f;
    s.lla[0]     = 40.0; s.lla[1] = -111.0; s.lla[2] = 1400.0;
    return s;
}

void writeRecord(cISLogger& logger, std::shared_ptr<cDeviceLog> dev,
                 uint32_t did, void* payload, std::size_t size) {
    is_comm_instance_t comm{};
    uint8_t buf[1024];
    is_comm_init(&comm, buf, sizeof(buf), nullptr);
    uint8_t pkt[2048];
    const int n = is_comm_data_to_buf(pkt, sizeof(pkt), &comm, static_cast<uint16_t>(did),
                                      static_cast<uint16_t>(size), 0, payload);
    if (n > 0) logger.LogData(dev, n, pkt);
}

struct Fixture {
    fs::path dir;
    fs::path raw;
    fs::path idx;
};

Fixture buildFixture(const std::string& hint, std::size_t records) {
    Fixture f;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "/tmp/is_ladder_fix_%s_%d_%ld", hint.c_str(), ::getpid(),
                  static_cast<long>(::time(nullptr)));
    f.dir = buf;
    ISFileManager::DeleteDirectory(f.dir.string());

    cISLogger logger;
    cISLogger::sSaveOptions opts;
    opts.logType               = cISLogger::LOGTYPE_RAW;
    opts.useSubFolderTimestamp = false;
    if (!logger.InitSave(f.dir.string(), opts)) return f;
    auto dev = logger.registerDevice(kFixtureHwId, kFixtureSerial);
    if (!dev) return f;
    logger.EnableLogging(true);
    for (std::size_t i = 0; i < records; ++i) {
        ins_2_t p = makeIns2(100.0 + 0.1 * static_cast<double>(i));
        writeRecord(logger, dev, DID_INS_2, &p, sizeof(p));
    }
    logger.CloseAllFiles();

    std::vector<ISFileManager::file_info_t> raws, idxs;
    ISFileManager::GetAllFilesInDirectory(f.dir.string(), true, "\\.raw$", raws);
    ISFileManager::GetAllFilesInDirectory(f.dir.string(), true, "\\.idx$", idxs);
    if (!raws.empty()) f.raw = raws.front().name;
    if (!idxs.empty()) f.idx = idxs.front().name;
    return f;
}

/**
 * @brief Rewrite the `timestamp` of every Nth record of a v2 sidecar, in place.
 *
 * The lie has to be one the fast path will BELIEVE, so it keeps the HAS_TIMESTAMP flag and the byte
 * offsets intact and only moves the claimed instant. That is what makes the fast path place those
 * records somewhere the bytes do not support, while leaving the file structurally perfect.
 *
 * @return  How many records were altered.
 */
std::size_t corruptSidecarTimestamps(const fs::path& idxPath, std::size_t everyNth,
                                     uint64_t addMs) {
    std::fstream io(idxPath, std::ios::binary | std::ios::in | std::ios::out);
    if (!io.good()) return 0;
    std::vector<uint8_t> head(idx::IS_LOG_IDX_HEADER_SIZE);
    io.read(reinterpret_cast<char*>(head.data()), static_cast<std::streamsize>(head.size()));
    auto h = idx::parseHeader(head.data());
    if (!h) return 0;
    const std::size_t stride = h->record_size ? h->record_size : idx::IS_LOG_IDX_RECORD_V2_SIZE;

    io.seekg(0, std::ios::end);
    const auto total = static_cast<std::size_t>(io.tellg());
    const std::size_t n = (total - idx::IS_LOG_IDX_HEADER_SIZE) / stride;

    std::size_t altered = 0;
    for (std::size_t i = 0; i < n; i += everyNth) {
        const auto pos = static_cast<std::streamoff>(idx::IS_LOG_IDX_HEADER_SIZE + i * stride);
        std::vector<uint8_t> rec(stride);
        io.seekg(pos);
        io.read(reinterpret_cast<char*>(rec.data()), static_cast<std::streamsize>(stride));
        uint64_t ts = 0;
        std::memcpy(&ts, rec.data(), sizeof(ts));          // `timestamp` is bytes 0..7
        if (ts == 0) continue;                              // leave absent values absent
        ts += addMs;
        std::memcpy(rec.data(), &ts, sizeof(ts));
        io.seekp(pos);
        io.write(reinterpret_cast<const char*>(rec.data()), static_cast<std::streamsize>(stride));
        ++altered;
    }
    io.flush();
    return altered;
}

std::vector<fs::path> findLogDirs(const fs::path& root, std::size_t cap) {
    std::vector<fs::path> out;
    std::error_code ec;
    std::vector<fs::path> seen;
    for (fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec),
         end; it != end && out.size() < cap; it.increment(ec)) {
        if (ec) { ec.clear(); continue; }
        if (!it->is_regular_file(ec)) continue;
        if (it->path().extension() != ".raw") continue;
        const fs::path dir = it->path().parent_path();
        if (std::find(seen.begin(), seen.end(), dir) == seen.end()) {
            seen.push_back(dir);
            out.push_back(dir);
        }
    }
    return out;
}

class LadderTest : public ::testing::Test {
protected:
    Fixture f_;
    void TearDown() override {
        if (!f_.dir.empty() && fs::exists(f_.dir)) {
            ISFileManager::DeleteDirectory(f_.dir.string());
        }
    }
};

} // namespace

// ===========================================================================
// Rung 1 — a healthy log is judged healthy, and nothing is written.
// ===========================================================================

TEST_F(LadderTest, HealthyLogStopsAtRungOne) {
    f_ = buildFixture("healthy", 400);
    ASSERT_FALSE(f_.raw.empty());
    ASSERT_TRUE(fs::exists(f_.idx)) << "fixture has no sidecar, so rung 1 has nothing to trust";

    const LadderReport r = climbLadder(f_.dir);
    EXPECT_EQ(r.verdict, LadderVerdict::Healthy)
        << verdictName(r.verdict) << " -- " << r.detail;
    EXPECT_EQ(r.rungsClimbed, 1) << "a healthy log climbed past rung 1";
    EXPECT_GT(r.rung1.records, 0u) << "no record was compared, so nothing was proved";
    EXPECT_TRUE(r.rung1.agree()) << r.rung1.summary();
}

/**
 * @brief The ladder never writes to the log directory it is judging.
 *
 * Kyle's constraint, and the reason `persistRebuiltIndex` exists: rung 3 rebuilds on a temp copy so
 * the corpus is never mutated. Asserted by content and mtime, with the sidecar deliberately removed
 * for the second half — the case where the old all-or-nothing behaviour WOULD have written one.
 */
TEST_F(LadderTest, LadderNeverMutatesTheLogDirectory) {
    f_ = buildFixture("nomutate", 300);
    ASSERT_FALSE(f_.raw.empty());
    ASSERT_TRUE(fs::exists(f_.idx));

    const auto idxBytesBefore = fs::file_size(f_.idx);
    const auto idxTimeBefore  = fs::last_write_time(f_.idx);
    std::vector<fs::path> before;
    for (const auto& e : fs::directory_iterator(f_.dir)) before.push_back(e.path().filename());
    std::sort(before.begin(), before.end());

    (void)climbLadder(f_.dir);

    std::vector<fs::path> after;
    for (const auto& e : fs::directory_iterator(f_.dir)) after.push_back(e.path().filename());
    std::sort(after.begin(), after.end());
    EXPECT_EQ(before, after)         << "the ladder added or removed a file";
    EXPECT_EQ(fs::file_size(f_.idx), idxBytesBefore) << "the sidecar changed size";
    EXPECT_TRUE(fs::last_write_time(f_.idx) == idxTimeBefore) << "the sidecar was rewritten";

    // Harder case: no sidecar at all. The default open would REBUILD AND PERSIST one here, so this
    // is what actually proves the ladder's opens are suppressed rather than merely unnecessary.
    std::error_code ec;
    fs::remove(f_.idx, ec);
    ASSERT_FALSE(fs::exists(f_.idx));
    (void)climbLadder(f_.dir);
    EXPECT_FALSE(fs::exists(f_.idx))
        << "the ladder persisted a sidecar into the directory it was judging";
}

// ===========================================================================
// Rungs 2 and 3 — a lying sidecar, caught and attributed.
// ===========================================================================

/**
 * @brief A sidecar whose timestamps lie: rung 1 fails, rung 2 clears the mechanism, rung 3 heals.
 *
 * This is the test that makes the ladder more than scaffolding. Every healthy log stops at rung 1,
 * so without a deliberately broken sidecar rungs 2 through 4 would never run and would prove
 * nothing — the same trap the index tie-break negative control exposed under item 1.
 *
 * The lie is structurally perfect: offsets, DIDs, flags and record count all intact, only the
 * claimed instants moved. So the fast path believes it completely, and the round-trip fixed point
 * still closes on it — which is exactly why rung 1's property has to be AGREEMENT with the scan
 * path rather than self-consistency.
 */
TEST_F(LadderTest, LyingTimestampsAreCaughtAtRungOneAndHealedAtRungThree) {
    f_ = buildFixture("liar", 400);
    ASSERT_FALSE(f_.raw.empty());
    ASSERT_TRUE(fs::exists(f_.idx));

    // Healthy first, so the failure below is attributable to the corruption and nothing else.
    ASSERT_EQ(climbLadder(f_.dir).verdict, LadderVerdict::Healthy)
        << "the fixture was not healthy before corruption";

    constexpr uint64_t kLieMs = 600'000;                 // ten minutes into the future
    const std::size_t altered = corruptSidecarTimestamps(f_.idx, /*everyNth=*/7, kLieMs);
    ASSERT_GT(altered, 10u) << "the corruption altered too few records to prove anything";

    const LadderReport r = climbLadder(f_.dir);
    std::fprintf(stderr, "[ladder] verdict=%s rungs=%d altered=%zu\n  rung1: %s\n  diff: %s\n",
                 verdictName(r.verdict), r.rungsClimbed, altered,
                 r.rung1.summary().c_str(), r.diff.summary().c_str());

    EXPECT_EQ(r.verdict, LadderVerdict::SidecarHealedByRebuild)
        << verdictName(r.verdict) << " -- " << r.detail;
    EXPECT_EQ(r.rungsClimbed, 3) << "the ladder did not reach rung 3";

    // Rung 1 caught it, and caught it as a TIME disagreement rather than a structural one.
    EXPECT_FALSE(r.rung1.agree());
    EXPECT_GT(r.rung1.timeDiffers, 0u)  << r.rung1.summary();
    EXPECT_EQ(r.rung1.didDiffers, 0u)   << "a timestamp lie should not move any DID";
    EXPECT_EQ(r.rung1.offsetDiffers, 0u)<< "a timestamp lie should not move any byte offset";

    // The attribution names the right class, and clears the others.
    ASSERT_TRUE(r.diff.parsed) << "no sidecar diff was produced";
    EXPECT_EQ(r.diff.timestampContradicts, altered) << r.diff.summary();
    EXPECT_EQ(r.diff.recordAbsent, 0u)              << r.diff.summary();
    EXPECT_EQ(r.diff.byteOffsetDiffers, 0u)         << r.diff.summary();
    EXPECT_EQ(r.diff.didDiffers, 0u)                << r.diff.summary();
    EXPECT_FALSE(r.diff.exoneratesTheSidecar())     << "the sidecar was wrongly exonerated";

    // And after the rebuild the two paths agree again.
    EXPECT_TRUE(r.rung3.agree()) << r.rung3.summary();
}

/**
 * @brief The round trip alone would NOT have caught the liar — stated as a test, not a claim.
 *
 * The reason rung 1 compares the two paths instead of checking the fast path against itself. A
 * sidecar can be wrong and perfectly self-consistent, and then every fixed-point assertion passes.
 */
TEST_F(LadderTest, TheRoundTripAloneDoesNotCatchALyingSidecar) {
    f_ = buildFixture("selfconsistent", 400);
    ASSERT_FALSE(f_.raw.empty());
    ASSERT_GT(corruptSidecarTimestamps(f_.idx, /*everyNth=*/7, 600'000), 10u);

    ISLogReader::OpenOptions readOnly;
    readOnly.persistRebuiltIndex = false;
    auto log = ISLog::openDirectory(f_.dir, readOnly);
    ASSERT_TRUE(static_cast<bool>(log));
    ASSERT_FALSE(log->deviceIds().empty());
    const ISDeviceLog& dl = log->device(log->deviceIds().front());
    auto R = ISTimeResolver::build(dl);
    ASSERT_TRUE(R.has_value());

    std::string why;
    EXPECT_EQ(cycleHolds(dl, *R, why), CycleResult::Held)
        << "the corrupted log failed the round trip (" << why << ") -- if that becomes true, this "
           "test's premise is gone and rung 1 could be simplified";
}

/**
 * @brief Rebuilding a HEALTHY sidecar changes nothing that matters, so it exonerates it.
 *
 * Rung 4 depends on this: "an identical rebuild that still fails convicts the mechanism" is only
 * meaningful if a rebuild of a good sidecar actually compares as identical. It very nearly did not
 * — the one field that always differs is the header anchor, which `cISLogger` never writes and a
 * rebuild always supplies. Measured here rather than assumed, because getting it wrong makes rung 4
 * unreachable and the mechanism permanently un-convictable.
 */
TEST_F(LadderTest, RebuildingAHealthySidecarExoneratesIt) {
    f_ = buildFixture("healthy_rebuild", 400);
    ASSERT_FALSE(f_.raw.empty());
    ASSERT_TRUE(fs::exists(f_.idx));

    const fs::path work = makeTempDir("healthy_rebuild");
    ASSERT_TRUE(copyLogDir(f_.dir, work));
    ISLogReader::OpenOptions rebuild;
    rebuild.ignoreOnDiskIndex   = true;
    rebuild.persistRebuiltIndex = true;
    { auto r = ISLog::openDirectory(work, rebuild); ASSERT_TRUE(static_cast<bool>(r)); }

    const SidecarDiff d = diffSidecars(f_.idx, work / f_.idx.filename());
    std::fprintf(stderr, "[healthy-rebuild] %s\n", d.summary().c_str());
    ASSERT_TRUE(d.parsed);
    EXPECT_EQ(d.recordsOriginal, d.recordsRebuilt);
    EXPECT_EQ(d.recordAbsent, 0u)          << d.summary();
    EXPECT_EQ(d.didDiffers, 0u)            << d.summary();
    EXPECT_EQ(d.byteOffsetDiffers, 0u)     << d.summary();
    EXPECT_EQ(d.timestampContradicts, 0u)  << d.summary();
    EXPECT_FALSE(d.headerAnchorContradicts)<< d.summary();
    // The anchor the writer never wrote, supplied by the rebuild. Expected, and the reason the
    // anchor class is split rather than a single "differs" flag.
    EXPECT_TRUE(d.headerAnchorAdded)
        << "the rebuild no longer supplies a header anchor; if that is intentional, this test and "
           "the anchor split can be simplified. " << d.summary();
    EXPECT_TRUE(d.exoneratesTheSidecar())
        << "a rebuild of a healthy sidecar does not compare as identical, so rung 4 can never "
           "convict the mechanism. " << d.summary();

    std::error_code ec; fs::remove_all(work, ec);
}

/**
 * @brief The exoneration predicate itself, class by class.
 *
 * `MechanismAtFault` has NO positive end-to-end coverage, and cannot have any: reaching it needs
 * either a `.raw` whose own scan fails the round trip, or a sidecar that compares identical to its
 * rebuild while the two paths still disagree — and if the two files hold the same data, the two
 * paths read the same data and necessarily agree. It is the alarm state, not a reachable outcome of
 * a working mechanism. What IS testable is the decision that leads there, so it is tested directly:
 * every class must be able to deny exoneration on its own, or a real conviction could slip past.
 */
TEST(LadderVerdictLogic, EveryDiffClassCanDenyExoneration) {
    SidecarDiff base;
    base.parsed = true;
    EXPECT_TRUE(base.exoneratesTheSidecar()) << "an empty diff must exonerate";

    SidecarDiff unparsed;                      // never exonerate on absent evidence
    EXPECT_FALSE(unparsed.exoneratesTheSidecar());

    { auto d = base; d.recordAbsent = 1;          EXPECT_FALSE(d.exoneratesTheSidecar()); }
    { auto d = base; d.didDiffers = 1;            EXPECT_FALSE(d.exoneratesTheSidecar()); }
    { auto d = base; d.byteOffsetDiffers = 1;     EXPECT_FALSE(d.exoneratesTheSidecar()); }
    { auto d = base; d.timestampContradicts = 1;  EXPECT_FALSE(d.exoneratesTheSidecar()); }
    { auto d = base; d.headerAnchorContradicts = true; EXPECT_FALSE(d.exoneratesTheSidecar()); }

    // The two benign classes must NOT deny it — this is the half that was wrong before measuring.
    { auto d = base; d.timestampDeclined = 99;    EXPECT_TRUE(d.exoneratesTheSidecar())
        << "a declined timestamp is the scan refusing to invent a receipt time, not a conflict"; }
    { auto d = base; d.headerAnchorAdded = true;  EXPECT_TRUE(d.exoneratesTheSidecar())
        << "an anchor the writer never wrote is the rebuild adding information, not a conflict"; }
}

// ===========================================================================
// Corpus — every real log is judged, and none may convict the mechanism.
// ===========================================================================

/**
 * @brief Diagnostic entry point: what devices does one log have, and how placeable is each?
 *
 * Skipped unless `IS_SDK_DIAG_LOG_DIR` names a log directory. Kept because it is the tool that
 * found the empty-first-device case above, and the next verdict that needs explaining will want it
 * again — reconstructing it from scratch costs more than carrying it.
 */
TEST(AbsTimeLadderCorpus, DescribeDevicePlaceabilityForOneLog) {
    const char* d = std::getenv("IS_SDK_DIAG_LOG_DIR");
    if (d == nullptr) GTEST_SKIP() << "set IS_SDK_DIAG_LOG_DIR";
    ISLogReader::OpenOptions ro; ro.persistRebuiltIndex = false;
    auto log = ISLog::openDirectory(fs::path{d}, ro);
    ASSERT_TRUE(static_cast<bool>(log));
    std::size_t i = 0;
    for (uint64_t devId : log->deviceIds()) {
        const ISDeviceLog& dl = log->device(devId);
        auto R = ISTimeResolver::build(dl);
        std::size_t total = 0, placed = 0;
        if (R) {
            for (std::size_t s = 0; s < dl.segmentCount(); ++s) {
                for (std::size_t k = 0; k < dl.segment(s).recordCount(); ++k) {
                    ++total;
                    if (R->resolve(dl, s, k).valid) ++placed;
                }
            }
        }
        std::fprintf(stderr, "[devices] idx=%zu dev=%llu segs=%zu total=%zu placed=%zu resolver=%d\n",
                     i++, static_cast<unsigned long long>(devId), dl.segmentCount(), total, placed,
                     static_cast<int>(R.has_value()));
    }
}

/**
 * @brief Climb the ladder over the corpus. No log may reach `MechanismAtFault`.
 *
 * The expensive one: every log is opened twice, and the second open is a full byte scan of every
 * segment. Capped, and the cap is REPORTED — a silent truncation would read as full coverage.
 */
TEST(AbsTimeLadderCorpus, NoCorpusLogConvictsTheMechanism) {
    const char* root = std::getenv("IS_SDK_CORPUS_DIR");
    if (root == nullptr || !fs::exists(root)) {
        GTEST_SKIP() << "corpus not present (set IS_SDK_CORPUS_DIR)";
    }
    const std::size_t cap = std::getenv("IS_SDK_LADDER_MAX_LOGS")
                          ? static_cast<std::size_t>(std::atoi(std::getenv("IS_SDK_LADDER_MAX_LOGS")))
                          : 12;
    const std::vector<fs::path> dirs = findLogDirs(root, cap);
    ASSERT_FALSE(dirs.empty());

    std::size_t healthy = 0, healed = 0, notHealed = 0, mechanism = 0, couldNotOpen = 0,
                nothingToJudge = 0, devices = 0;
    std::vector<std::string> convictions;
    for (const fs::path& dir : dirs) {
        // EVERY device, not just the first. Judging device 0 only made coverage depend on device
        // ordering, and on `ppd_LogQAQW713/20260716_000102` device 0 is an empty segment while
        // device 1 carries 123,148 placeable records — so the log's real content went unjudged.
        ISLogReader::OpenOptions ro;
        ro.persistRebuiltIndex = false;
        auto probe = ISLog::openDirectory(dir, ro);
        const std::size_t deviceCount = probe ? probe->deviceIds().size() : 0;
        if (deviceCount == 0) { ++couldNotOpen; continue; }

        for (std::size_t devIndex = 0; devIndex < deviceCount; ++devIndex) {
            const LadderReport r = climbLadder(dir, devIndex);
            ++devices;
            switch (r.verdict) {
                case LadderVerdict::Healthy:                ++healthy;        break;
                case LadderVerdict::SidecarHealedByRebuild: ++healed;         break;
                case LadderVerdict::RebuildDidNotHeal:      ++notHealed;      break;
                case LadderVerdict::MechanismAtFault:       ++mechanism;      break;
                case LadderVerdict::NothingToJudge:         ++nothingToJudge; break;
                case LadderVerdict::CouldNotOpen:           ++couldNotOpen;   break;
            }
            if (r.verdict != LadderVerdict::Healthy && r.verdict != LadderVerdict::NothingToJudge) {
                convictions.push_back(dir.filename().string() + " dev#" + std::to_string(devIndex)
                                      + ": " + verdictName(r.verdict)
                                      + " [" + r.detail + "] diff{" + r.diff.summary() + "}");
            }
        }
    }

    std::fprintf(stderr,
        "[ladder-corpus] logs=%zu (cap=%zu) devices=%zu healthy=%zu healed=%zu notHealed=%zu "
        "mechanism=%zu nothingToJudge=%zu couldNotOpen=%zu\n",
        dirs.size(), cap, devices, healthy, healed, notHealed, mechanism, nothingToJudge,
        couldNotOpen);
    for (const std::string& c : convictions) std::fprintf(stderr, "  %s\n", c.c_str());

    // `nothingToJudge` deliberately does NOT count toward coverage: a device that places no record
    // exercises no property, and letting it count would let a run of empty logs read as a pass.
    EXPECT_GE(healthy + healed, 5u) << "too few devices contributed a usable verdict";
    EXPECT_EQ(mechanism, 0u)  << "a corpus device convicts the absolute-time mechanism itself";
    EXPECT_EQ(notHealed, 0u)  << "a corpus device disagrees across the two read paths and a "
                                 "rebuilt sidecar did not resolve it";
}

//! DIAGNOSTIC: dump the first N records of a segment with full provenance.
//! `IS_SDK_DIAG_LOG_DIR=<dir>`, optional `IS_SDK_DIAG_N=<count>`.
TEST(AbsTimeLadderCorpus, DumpFirstRecordsWithProvenance) {
    const char* d = std::getenv("IS_SDK_DIAG_LOG_DIR");
    if (d == nullptr) GTEST_SKIP() << "set IS_SDK_DIAG_LOG_DIR";
    const std::size_t N = std::getenv("IS_SDK_DIAG_N")
                        ? static_cast<std::size_t>(std::atoi(std::getenv("IS_SDK_DIAG_N"))) : 24;
    ISLogReader::OpenOptions ro; ro.persistRebuiltIndex = false;
    auto log = ISLog::openDirectory(fs::path{d}, ro);
    ASSERT_TRUE(static_cast<bool>(log));
    ASSERT_FALSE(log->deviceIds().empty());
    const ISDeviceLog& dl = log->device(log->deviceIds().front());
    auto R = ISTimeResolver::build(dl);
    ASSERT_TRUE(R.has_value());

    // The relative-time ORIGIN the UI uses: first record in ARRIVAL order that resolves.
    const auto origin = R->firstResolvableIn(dl, 0);
    std::fprintf(stderr, "[origin] abs=%llu mech=%s anchor=%s\n",
                 (unsigned long long)origin.absoluteMs,
                 absTimeMechanismName(origin.mechanism),
                 absAnchorSourceName(origin.anchorSource));

    for (std::size_t k = 0; k < N && k < dl.segment(0).recordCount(); ++k) {
        const ISRecordView rv = dl.segment(0).recordAt(k);
        const AbsTimeResult r = R->resolve(dl, 0, k);
        const int64_t rel = static_cast<int64_t>(r.absoluteMs)
                          - static_cast<int64_t>(origin.absoluteMs);
        std::fprintf(stderr,
            "k=%-3zu did=%-3u %-26s raw=%-12llu abs=%llu rel=%+.3fs mech=%-18s anchor=%-16s "
            "week=%u towMs=%llu wkFromPayload=%d\n",
            k, rv.did(), cISDataMappings::DataName(rv.did()),
            (unsigned long long)r.sidecarRawMs, (unsigned long long)r.absoluteMs,
            static_cast<double>(rel)/1000.0,
            absTimeMechanismName(r.mechanism), absAnchorSourceName(r.anchorSource),
            r.gpsWeek, (unsigned long long)r.towMs, (int)r.weekFromPayload);
        for (const std::string& h : r.hints) std::fprintf(stderr, "        hint: %s\n", h.c_str());
    }
}

//! DIAGNOSTIC: what does the ISB parser actually REPORT for Kyle's checksum-failed packet?
//! Mirrors RecordWalker's setup exactly: the window IS the parser's rxBuf.
TEST(AbsTimeLadderCorpus, DiagnoseChecksumFailedPacketReporting) {
    static const uint8_t kPkt[] = {
        0xEF,0x49,0x04,0x30,0x60,0x00, 0xB1,0x38,0x89,0x09,0x46,0xD5,0x28,0x40,
        0x1C,0xDD,0x01,0x3E,0x51,0xDD,0x13,0xBF,0xE1,0xAB,0x34,0xBF,0x47,0xC2,0xC7,0xBE,
        0,0,0,0,0,0,0,0,0,0,0,0,
        0xDB,0xB1,0xB5,0xFA,0xA8,0x70,0x3B,0xC1, 0xF6,0x15,0x58,0x08,0x2F,0x4D,0x51,0xC1,
        0x98,0xAF,0x2D,0x89,0xC8,0x40,0x4F,0x41,
        0x60,0xB6,0xEC,0x3A, 0xE6,0xB6,0xD5,0xB9, 0x33,0x03,0xD5,0xBA,
        0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,
        0x28,0x02 };

    // Realistic framing: a VALID packet, then the bad one, then another valid one - which is how
    // it sits in the log (between DEBUG_ARRAY and INL2_MAG_OBS_INFO). A lone packet with nothing
    // after it cannot be told apart from "stream ended mid-packet".
    uint8_t good[256];
    is_comm_instance_t wcomm{};
    uint8_t wbuf[512];
    is_comm_init(&wcomm, wbuf, sizeof(wbuf), nullptr);
    imu_t payload{};
    payload.time = 1.5;
    const int goodLen = is_comm_data_to_buf(good, sizeof(good), &wcomm, DID_IMU,
                                            sizeof(imu_t), 0, &payload);
    ASSERT_GT(goodLen, 0);

    std::vector<uint8_t> window;
    window.insert(window.end(), good, good + goodLen);
    const std::size_t badAt = window.size();
    window.insert(window.end(), kPkt, kPkt + sizeof(kPkt));
    window.insert(window.end(), good, good + goodLen);
    std::vector<uint8_t> buf(window.size());
    is_comm_instance_t comm{};
    is_comm_init(&comm, buf.data(), static_cast<int>(buf.size()), nullptr);
    std::memcpy(buf.data(), window.data(), window.size());
    std::fprintf(stderr, "[parse] good=%d bytes, bad packet at +%zu, total %zu\n",
                 goodLen, badAt, window.size());
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);
    is_comm_enable_protocol(&comm, _PTYPE_NMEA);
    is_comm_enable_protocol(&comm, _PTYPE_RTCM3);
    is_comm_enable_protocol(&comm, _PTYPE_UBLOX);
    // Exactly what RecordWalker does: declare the whole window available by setting tail ONLY,
    // and leave head/scan as is_comm_init left them. is_comm_free is deliberately not called.
    comm.rxBuf.tail = buf.data() + buf.size();

    for (int i = 0; i < 8; ++i) {
        const protocol_type_t pt = is_comm_parse(&comm);
        std::fprintf(stderr,
            "[parse] iter=%d ptype=%d rxErrorType=%d head=+%ld pktSize=%u did=%u\n",
            i, (int)pt, (int)comm.rxErrorType,
            (long)(comm.rxBuf.head - buf.data()), comm.rxPkt.size,
            (unsigned)comm.rxPkt.hdr.id);
        if (pt == _PTYPE_NONE) break;
    }
    std::fprintf(stderr, "[parse] EPARSE_INVALID_CHKSUM = %d\n", (int)EPARSE_INVALID_CHKSUM);
}
