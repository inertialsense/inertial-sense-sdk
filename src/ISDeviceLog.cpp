/**
 * @file ISDeviceLog.cpp
 * @brief See ISDeviceLog.h.
 *
 * D-05 / SN-7896.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDeviceLog.h"

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

    // Stable, deterministic order: fromSegments uses lexicographic path order as
    // the tiebreaker when segment start-times are equal/missing, so sort up front
    // (openDirectory already sorts, but this is a public entry point).
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

    // Validate same deviceId across all segments.
    const uint64_t expected = readers.front().deviceId();
    for (std::size_t i = 1; i < readers.size(); ++i) {
        const uint64_t got = readers[i].deviceId();
        if (got != expected) {
            log_error(IS_LOG_ISLOG, "ISDeviceLog::fromSegments: device-id "
                      "mismatch — first segment is SN%llu, segment %zu (%s) "
                      "is SN%llu",
                      static_cast<unsigned long long>(expected), i,
                      segmentPaths[i].filename().string().c_str(),
                      static_cast<unsigned long long>(got));
            return fail(ISErrorCode::Corrupted,
                std::string{"ISDeviceLog::fromSegments: device-id mismatch — "
                            "first segment is SN"} + std::to_string(expected)
                + ", segment " + std::to_string(i) + " is SN" + std::to_string(got));
        }
    }

    // Order by segment-start timestamp. Use header.first_timestamp_ms
    // when present (D-01 writer fills this; D-04 scan-rebuild does too)
    // and fall back to the path's lexicographic order otherwise — the
    // writer's filename pattern is timestamp-sortable per D0051.
    std::sort(readers.begin(), readers.end(),
        [](const ISLogReader& a, const ISLogReader& b) {
            const uint64_t aT = a.segmentStartTimestamp();
            const uint64_t bT = b.segmentStartTimestamp();
            if (aT && bT && aT != bT) return aT < bT;
            // Rely on filename-lex ordering as the tiebreaker.
            return false;  // stable; preserve input order on ties
        });

    ISDeviceLog out;
    out.segments_ = std::move(readers);
    out.deviceId_ = expected;
    out.buildIndex();
    log_more_info(IS_LOG_ISLOG, "ISDeviceLog::fromSegments: device 0x%016llx, "
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

std::vector<ISDeviceLog::did_t> ISDeviceLog::presentDids() const {
    std::vector<did_t> out;
    out.reserve(byDid_.size());
    for (const auto& kv : byDid_) out.push_back(kv.first);
    std::sort(out.begin(), out.end());
    return out;
}

TimeStamp ISDeviceLog::spanStart() const noexcept {
    for (const auto& seg : segments_) {
        const uint64_t v = seg.segmentStartTimestamp();
        if (v != 0) return TimeStamp::fromPayloadToW(v, deviceId_);
    }
    return TimeStamp::fromPayloadToW(0, deviceId_);
}

TimeStamp ISDeviceLog::spanEnd() const noexcept {
    for (auto it = segments_.rbegin(); it != segments_.rend(); ++it) {
        const uint64_t v = it->segmentEndTimestamp();
        if (v != 0) return TimeStamp::fromPayloadToW(v, deviceId_);
    }
    return TimeStamp::fromPayloadToW(0, deviceId_);
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
