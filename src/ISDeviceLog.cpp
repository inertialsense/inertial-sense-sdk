/**
 * @file ISDeviceLog.cpp
 * @brief See ISDeviceLog.h.
 *
 * D-05 / SN-7896.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDeviceLog.h"
#include "ISAnchorAnalysis.h"
#include "ISDiagnostics.h"

#include "ISLogIndex.h"       // SN-8629 timestampsLookMixedDomain()
#include "ISTimeResolver.h"   // SN-8105 anchored-span accessors
#include "core/msg_logger.h"

#include <algorithm>
#include <cstdint>
#include <set>
#include <utility>

namespace inertial_sense {

namespace fs = std::filesystem;

const std::vector<ISDeviceLog::Locator> ISDeviceLog::kEmptyLocators_ = {};

ISDeviceLog::~ISDeviceLog()                                = default;
ISDeviceLog::ISDeviceLog(ISDeviceLog&&) noexcept           = default;
ISDeviceLog& ISDeviceLog::operator=(ISDeviceLog&&) noexcept = default;

ISExpected<ISDeviceLog>
    ISDeviceLog::fromSegments(std::vector<fs::path> segmentPaths) {
    log_debug(IS_LOG_ISLOG, "ISDeviceLog::fromSegments: %zu segment(s)",
              segmentPaths.size());
    if (segmentPaths.empty()) {
        log_error(IS_LOG_ISLOG, "ISDeviceLog::fromSegments: empty segment list");
        return fail(ISErrorCode::InvalidArgument,
                    "ISDeviceLog::fromSegments: empty segment list");
    }

    // Stable, deterministic order: composition uses lexicographic path order as the tiebreaker
    // when segment start-times are equal/missing, so sort up front (openDirectory already
    // sorts, but this is a public entry point).
    std::sort(segmentPaths.begin(), segmentPaths.end());

    // Open each segment.
    std::vector<ISLogReader> readers;
    readers.reserve(segmentPaths.size());
    for (const auto& path : segmentPaths) {
        auto r = ISLogReader::openSegment(path);
        if (!r) {
            log_error(IS_LOG_ISLOG, "openSegment failed for %s: %s",
                      path.string().c_str(), r.error().message.c_str());
            return tl::unexpected<ISError>{ r.error() };
        }
        readers.push_back(std::move(*r));
    }

    return fromReaders(std::move(readers));
}

ISExpected<ISDeviceLog> ISDeviceLog::fromReaders(std::vector<ISLogReader>&& readers) {
    if (readers.empty()) {
        log_error(IS_LOG_ISLOG, "ISDeviceLog::fromReaders: empty reader list");
        return fail(ISErrorCode::InvalidArgument,
                    "ISDeviceLog::fromReaders: empty reader list");
    }

    // Validate same deviceId across all segments.
    const uint64_t expected = readers.front().deviceId();
    for (std::size_t i = 1; i < readers.size(); ++i) {
        const uint64_t got = readers[i].deviceId();
        if (got != expected) {
            log_error(IS_LOG_ISLOG, "ISDeviceLog::fromReaders: device-id "
                      "mismatch — first segment is SN%llu, segment %zu (%s) "
                      "is SN%llu",
                      static_cast<unsigned long long>(expected), i,
                      readers[i].path().filename().string().c_str(),
                      static_cast<unsigned long long>(got));
            return fail(ISErrorCode::Corrupted,
                std::string{"ISDeviceLog::fromReaders: device-id mismatch — "
                            "first segment is SN"} + std::to_string(expected)
                + ", segment " + std::to_string(i) + " is SN" + std::to_string(got));
        }
    }

    // D-119 / SN-8626: reject a device log composed of mixed .raw/.dat segments. Nothing today
    // requires mixing formats within one composed log, and the simpler invariant is safer —
    // same pattern as the deviceId check above.
    auto formatName = [](ISLogReader::SegmentFormat f) noexcept {
        return f == ISLogReader::SegmentFormat::Dat ? ".dat" : ".raw";
    };
    const ISLogReader::SegmentFormat expectedFormat = readers.front().format();
    for (std::size_t i = 1; i < readers.size(); ++i) {
        const ISLogReader::SegmentFormat got = readers[i].format();
        if (got != expectedFormat) {
            log_error(IS_LOG_ISLOG, "ISDeviceLog::fromReaders: format mismatch — "
                      "first segment is %s, segment %zu (%s) is %s",
                      formatName(expectedFormat), i,
                      readers[i].path().filename().string().c_str(), formatName(got));
            return fail(ISErrorCode::Unsupported,
                std::string{"ISDeviceLog::fromReaders: mixed segment formats — first segment is "}
                + formatName(expectedFormat) + ", segment " + std::to_string(i) + " is "
                + formatName(got));
        }
    }

    // Order by segment-start timestamp. Use header.first_timestamp_ms
    // when present (D-01 writer fills this; D-04 scan-rebuild does too)
    // and fall back to the path's lexicographic order otherwise — the
    // writer's filename pattern is timestamp-sortable per D0051.
    //
    // SN-8629: a numeric comparison across TWO segments is only meaningful
    // when both segments' own timestamps are internally self-consistent
    // (start <= end) — a segment whose first/last landed on different
    // domains (e.g. one host-uptime, one GPS time-of-week; D0066) fails
    // that check, and its header value is not safely comparable against
    // another segment's. Rather than pick and choose per-pair (which is not
    // provably transitive — sorting requires a strict weak ordering), any
    // one bad segment disables header-based ordering for the WHOLE
    // composition; std::stable_sort then leaves everything in the
    // filename-lexicographic order already established above, which is
    // always safe (D0051) and correct for the common/comparable case too
    // (that's the whole premise of the filename pattern being sortable).
    // SN-8629: order by the segment's DOMAIN-NORMALIZED anchored start, not by a raw header
    // timestamp. The old key was `records_.front().timestamp` -- positional, so it took whatever
    // domain the physically-first record happened to stamp. On a log whose segments interleave
    // GPS-ToW and host-uptime DIDs that is a coin flip per segment, and sorting by it placed
    // segments out of order, manufacturing forward jumps and rewinds that are not in the data.
    //
    // Two conditions must BOTH hold before the re-sort is allowed to touch the
    // filename-lexicographic order already established above:
    //
    //  1. Every segment must carry an anchor of at least `minimumOrderableTier`. A single
    //     unanchored segment disables the re-sort for the whole composition, because a missing
    //     key cannot be compared -- and the previous comparator's attempt to tolerate one
    //     (`if (aT && bT && ...) ... return false`) made a zero key tie with every segment while
    //     non-zero keys still ordered among themselves. That is not a strict weak ordering, so
    //     `std::stable_sort`'s result was formally undefined.
    //  2. The keys must be mutually comparable, which the anchor analysis guarantees by
    //     construction: every `anchoredStartMs` is expressed in the same absolute frame.
    //
    // When either fails, filename-lexicographic order stands -- always safe per D0051, and
    // correct for the comparable case too, which is the whole premise of a sortable pattern.
    // SN-8629 / SN-8704: place every segment on the absolute frame BEFORE testing orderability.
    //
    // A reader is built from one segment in isolation, so on its own it can only reach the tiers
    // a segment establishes from its own payload. The chained tiers need sibling context, and
    // this is the only layer that has it.
    //
    // This walks the log by RECORDING SESSION rather than pairwise, which is what makes the
    // propagation BIDIRECTIONAL. The (ToW - uptime) offset is a constant for a boot session:
    // uptime and GPS time advance together until the device reboots. So the moment ANY segment
    // of a session pins that constant, every other segment of the same session is anchored by
    // it -- including segments EARLIER in the log than the one that supplied it. The previous
    // implementation was a single forward pass, which could only ever push information later:
    // a log whose first five segments had no absolute time of their own, followed by a sixth
    // that acquired a GPS fix, left the first five stranded at FilenameAnchor or None even
    // though the log contained everything needed to place them.
    //
    // Session boundaries come from uptime RESETS. Uptime rises monotonically within a session,
    // so a segment whose uptime starts below its predecessor's means the device rebooted, and
    // the offset must NOT be carried across that boundary (D0069 s4 / SN-8339: the offset is
    // per-boot-session, not per-log).
    //
    // Doing it this way also dissolves the circularity the forward chain papered over --
    // "ordering needs anchors, chaining needs an order". Session grouping only needs the
    // filename order, which is timestamp-sortable by construction (D0051); the anchors then
    // either confirm that order or correct it below.
    {
        std::vector<AnchorAnalysis> analyses;
        analyses.reserve(readers.size());
        for (const auto& r : readers) analyses.push_back(r.anchorAnalysis());

        for (const auto& a : planSessionAdoptions(analyses)) {
            readers[a.segment].adoptSessionOffset(a.offsetMs, a.donorDid, a.donorIsEarlier);
        }

        // Anything still unanchored takes the per-segment fallbacks (filename, or none) plus the
        // durability-regression check against its predecessor. Segments that adopted a session
        // offset are already placed and are left alone.
        AnchorAnalysis prevAnalysis{};
        const AnchorAnalysis* prev = nullptr;
        for (auto& r : readers) {
            if (!r.anchorAnalysis().anchored()) {
                r.reanalyzeWithPrevious(prev);
            }
            prevAnalysis = r.anchorAnalysis();
            prev         = &prevAnalysis;
        }

        // Share the LOG's uptime zero across every segment. Only a segment that is sequence
        // `_0001` can establish it alone, so this is the layer that distributes it -- and it has
        // to be a separate pass, because the loop above only re-analyses UNANCHORED segments and
        // a filename-anchored one already counts as anchored.
        //
        // Without it each filename-anchored segment re-derives a per-SEGMENT anchor from its own
        // uptime minimum, which cancels out and collapses every segment of the log onto the log's
        // open instant -- leaving the `anchoredStartMs` sort below with all-equal keys. A log
        // whose early segments were culled has no segment that knows the zero; it stays 0, the
        // whole log shifts late uniformly, and its internal geometry is still correct.
        uint64_t logZeroUptimeMs = 0;
        for (const auto& r : readers) {
            if (r.anchorAnalysis().logStartUptimeMs != 0) {
                logZeroUptimeMs = r.anchorAnalysis().logStartUptimeMs;
                break;
            }
        }
        if (logZeroUptimeMs != 0) {
            for (auto& r : readers) r.applyLogStartUptime(logZeroUptimeMs);
        }
    }

    // Every segment must carry an anchor of at least `minimumOrderableTier` before the re-sort
    // is allowed to touch the filename order established above. A single unanchored segment
    // disables it for the whole composition, because a missing key cannot be compared -- and
    // the pre-SN-8629 comparator's attempt to tolerate one (`if (aT && bT && ...) return false`)
    // made a zero key tie with every segment while non-zero keys still ordered among
    // themselves. That is not a strict weak ordering, so `std::stable_sort` was undefined.
    //
    // Mutual comparability is guaranteed by construction: every `anchoredStartMs` is expressed
    // in the same absolute frame.
    const bool allSegmentsOrderable = std::all_of(readers.begin(), readers.end(),
        [](const ISLogReader& r) {
            const AnchorAnalysis& a = r.anchorAnalysis();
            return a.anchored() && a.tier >= AnchorAnalysis::minimumOrderableTier &&
                   a.anchoredStartMs != 0;
        });

    // Fallback key when no absolute anchor is available. A segment with no absolute time is still
    // orderable AGAINST ITS SIBLINGS if they all carry host-uptime records: uptime runs
    // continuously across the segments of one recording session, so its extrema order them
    // correctly even though they say nothing about where the session sits in wall-clock time.
    // Only ever compared against other uptime extrema, never against an anchored start -- mixing
    // an absolute key with a relative one is the incomparability this whole change is about.
    const bool allSegmentsHaveUptime = std::all_of(readers.begin(), readers.end(),
        [](const ISLogReader& r) {
            const AnchorAnalysis& a = r.anchorAnalysis();
            return a.uptimeRecords > 0 && a.uptimeMinMs != 0;
        });

    if (allSegmentsOrderable) {
        std::stable_sort(readers.begin(), readers.end(),
            [](const ISLogReader& a, const ISLogReader& b) {
                return a.anchorAnalysis().anchoredStartMs < b.anchorAnalysis().anchoredStartMs;
            });
    } else if (allSegmentsHaveUptime) {
        log_info(IS_LOG_ISLOG,
                 "ISDeviceLog::fromSegments: no absolute anchor on every segment -- ordering by "
                 "host-uptime extrema (session-relative)");
        std::stable_sort(readers.begin(), readers.end(),
            [](const ISLogReader& a, const ISLogReader& b) {
                return a.anchorAnalysis().uptimeMinMs < b.anchorAnalysis().uptimeMinMs;
            });
    } else {
        log_info(IS_LOG_ISLOG,
                 "ISDeviceLog::fromReaders: keeping filename order -- segments carry neither a "
                 "common time anchor nor comparable uptime extrema");
    }

    ISDeviceLog out;
    out.segments_ = std::move(readers);
    out.deviceId_ = expected;
    out.buildIndex();
    log_more_info(IS_LOG_ISLOG, "ISDeviceLog::fromReaders: device 0x%016llx, "
                  "%zu segment(s), %zu record(s)",
                  static_cast<unsigned long long>(out.deviceId_),
                  out.segments_.size(), out.total_);
    return out;
}

void ISDeviceLog::buildIndex() {
    total_ = 0;
    all_.clear();
    byDid_.clear();
    segmentBase_.assign(segments_.size(), 0);

    for (std::size_t s = 0; s < segments_.size(); ++s) {
        segmentBase_[s] = total_;   // SN-8339: global arrival base for this segment
        const std::size_t n = segments_[s].recordCount();
        all_.reserve(all_.size() + n);
        for (std::size_t r = 0; r < n; ++r) {
            const Locator loc{ s, r };
            all_.push_back(loc);
        }
        total_ += n;
    }

    for (const auto& loc : all_) {
        const ISRecordView v = segments_[loc.segment].recordAt(loc.record);
        byDid_[v.did()].push_back(loc);
    }
}

std::vector<ISDiagnostic> ISDeviceLog::diagnostics() const {
    std::vector<ISDiagnostic> out;
    for (const auto& seg : segments_) {
        auto d = seg.diagnostics();
        out.insert(out.end(), std::make_move_iterator(d.begin()), std::make_move_iterator(d.end()));
    }
    return out;
}

std::vector<ISDeviceLog::did_t> ISDeviceLog::presentDids() const {
    std::vector<did_t> out;
    out.reserve(byDid_.size());
    for (const auto& kv : byDid_) out.push_back(kv.first);
    std::sort(out.begin(), out.end());
    return out;
}

// Audit A2. Both of these used to read `seg.segmentStartTimestamp()` -- the `.idx`'s RAW,
// mixed-domain transcription -- and wrap it in `fromPayloadToW` unconditionally. That is wrong
// twice on any log without a time-of-week, as measured on the uptime-only fixture: the value
// was 5 ms (the `log_time_offset_ms` the live writer parks in the `timestamp` field of a
// timeless DID_DEV_INFO) where the earliest real record was at 10000 ms and the true anchored
// start was 1789938572000, and the tag claimed `PayloadToW` on a log containing no ToW at all.
//
// `segmentSpanStart/End` produce the value and its provenance together from the anchor cascade,
// so neither can be right while the other is wrong. Segments are stored in anchored order by
// `fromSegments`, but a segment can still be unanchored (tier `None`), so keep scanning for the
// first/last one that yields a real value rather than trusting position alone.
TimeStamp ISDeviceLog::spanStart() const noexcept {
    for (const auto& seg : segments_) {
        const TimeStamp t = seg.segmentSpanStart();
        if (t.value != 0) return t;
    }
    return TimeStamp::fromSessionOnly(0, deviceId_);
}

TimeStamp ISDeviceLog::spanEnd() const noexcept {
    for (auto it = segments_.rbegin(); it != segments_.rend(); ++it) {
        const TimeStamp t = it->segmentSpanEnd();
        if (t.value != 0) return t;
    }
    return TimeStamp::fromSessionOnly(0, deviceId_);
}

namespace {

//! SN-8105: single arrival-ordered pass folding the min and max of every
//! record's resolver-anchored timestamp. Skips zero-raw (no time field) and
//! `SessionOnly/Unknown` (no shared anchor) records. `any` is false when the
//! log has no anchorable record. Keyed on arrival index for SN-8339 multi-boot.
struct AnchoredFold {
    uint64_t minMs = UINT64_MAX;
    uint64_t maxMs = 0;
    bool     any   = false;
};

AnchoredFold foldAnchoredSpan(const ISDeviceLog& log,
                              const ISTimeResolver& resolver) {
    AnchoredFold f;
    const uint64_t deviceId = log.deviceId();
    for (ISRecordView v : log.allRecords()) {
        const uint64_t raw = v.timestamp().value;
        if (raw == 0) continue;
        const TimeStamp r = resolver.resolve(raw, deviceId, v.arrivalIndex());
        if (r.source == TimeSource::SessionOnly &&
            r.confidence == TimeConfidence::Unknown) {
            continue;
        }
        if (r.value == 0) continue;
        if (!f.any || r.value < f.minMs) f.minMs = r.value;
        if (!f.any || r.value > f.maxMs) f.maxMs = r.value;
        f.any = true;
    }
    return f;
}

}  // namespace

TimeStamp ISDeviceLog::anchoredSpanStart(const ISTimeResolver& resolver) const noexcept {
    const AnchoredFold f = foldAnchoredSpan(*this, resolver);
    if (!f.any) return spanStart();
    return TimeStamp::fromResolvedViaSync(f.minMs, deviceId_, TimeConfidence::Exact);
}

TimeStamp ISDeviceLog::anchoredSpanEnd(const ISTimeResolver& resolver) const noexcept {
    const AnchoredFold f = foldAnchoredSpan(*this, resolver);
    if (!f.any) return spanEnd();
    return TimeStamp::fromResolvedViaSync(f.maxMs, deviceId_, TimeConfidence::Exact);
}

ISDeviceLog::Range ISDeviceLog::records(did_t did) const noexcept {
    auto it = byDid_.find(did);
    if (it == byDid_.end()) {
        return Range{ this, &kEmptyLocators_, 0, 0 };
    }
    return Range{ this, &it->second, 0, it->second.size() };
}

ISDeviceLog::Range ISDeviceLog::allRecords() const noexcept {
    return Range{ this, &all_, 0, all_.size() };
}

ISDeviceLog::Range
    ISDeviceLog::Range::in_time(TimeStamp t0, TimeStamp t1) const {
    if (parent_ == nullptr || locators_ == nullptr) return *this;
    if (begin_ >= end_) return *this;

    auto tsAt = [&](std::size_t i) -> uint64_t {
        const Locator& loc = (*locators_)[i];
        return parent_->segments_[loc.segment]
                   .recordAt(loc.record).timestamp().value;
    };

    std::size_t lo = begin_;
    while (lo < end_ && tsAt(lo) < t0.value) ++lo;
    std::size_t hi = lo;
    while (hi < end_ && tsAt(hi) <= t1.value) ++hi;
    return Range{ parent_, locators_, lo, hi };
}

ISDeviceLog::RangeIterator
    ISDeviceLog::seek(TimeStamp target) const noexcept {
    // Linear scan — D-07 will replace with a monotonic binary search
    // post time-resolution.
    for (std::size_t i = 0; i < all_.size(); ++i) {
        const Locator& loc = all_[i];
        const uint64_t ts =
            segments_[loc.segment].recordAt(loc.record).timestamp().value;
        if (ts >= target.value) {
            return RangeIterator{ this, &all_, i };
        }
    }
    return RangeIterator{ this, &all_, all_.size() };
}

ISRecordView ISDeviceLog::RangeIterator::operator*() const noexcept {
    if (!parent_ || !locators_ || pos_ >= locators_->size()) return {};
    const Locator& loc = (*locators_)[pos_];
    ISRecordView v = parent_->segments_[loc.segment].recordAt(loc.record);
    // SN-8339: stamp the global arrival index so consumers can key the
    // multi-boot resolver, regardless of which range (allRecords / records(did)
    // / seek / in_time) produced this view.
    if (loc.segment < parent_->segmentBase_.size()) {
        v.setArrivalIndex(parent_->segmentBase_[loc.segment] + loc.record);
    }
    return v;
}

ISDeviceLog::RangeIterator&
    ISDeviceLog::RangeIterator::operator++() noexcept {
    if (!parent_ || !locators_ || pos_ >= locators_->size()) {
        ++pos_;
        return *this;
    }
    const std::size_t prevSeg = (*locators_)[pos_].segment;
    ++pos_;
    if (pos_ < locators_->size() &&
        parent_->onSegmentBoundary_) {
        const std::size_t newSeg = (*locators_)[pos_].segment;
        if (newSeg != prevSeg) {
            parent_->onSegmentBoundary_(prevSeg);
        }
    }
    return *this;
}

} // namespace inertial_sense
