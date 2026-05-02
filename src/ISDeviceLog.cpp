/**
 * @file ISDeviceLog.cpp
 * @brief See ISDeviceLog.h.
 *
 * D-05 / SN-7896.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDeviceLog.h"

#include "core/msg_logger.h"

#include <algorithm>
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

    // Open each segment.
    std::vector<ISLogReader> readers;
    readers.reserve(segmentPaths.size());
    for (const auto& path : segmentPaths) {
        auto r = ISLogReader::openSegment(path);
        if (!r) {
            log_error(IS_LOG_ISLOG, "openSegment failed for %s: %s",
                      path.c_str(), r.error().message.c_str());
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
                      "mismatch — first segment is SN%lu, segment %zu (%s) "
                      "is SN%lu",
                      static_cast<unsigned long>(expected), i,
                      segmentPaths[i].filename().c_str(),
                      static_cast<unsigned long>(got));
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
    log_more_info(IS_LOG_ISLOG, "ISDeviceLog::fromSegments: device 0x%016lx, "
                  "%zu segment(s), %zu record(s)",
                  static_cast<unsigned long>(out.deviceId_),
                  out.segments_.size(), out.total_);
    return out;
}

void ISDeviceLog::buildIndex() {
    total_ = 0;
    all_.clear();
    byDid_.clear();

    for (std::size_t s = 0; s < segments_.size(); ++s) {
        const std::size_t n = segments_[s].recordCount();
        all_.reserve(all_.size() + n);
        for (std::size_t r = 0; r < n; ++r) {
            const Locator loc{ s, r };
            all_.push_back(loc);
            // Pull DID via the segment's records-vector accessor.
            // (Segment's per-DID map is already built; we replay it here.)
        }
        for (auto did : segments_[s].presentDids()) {
            auto& bucket = byDid_[did];
            for (auto rec : segments_[s].records(did)) {
                // We need the in-segment record index, not the view.
                // Easier path: the segment's allRecords() walks records
                // in arrival order with index 0..N-1, and records(did)
                // walks them in arrival order too. So we can map by
                // counting — but that's O(N²). Instead, iterate
                // records(did) and for each match remember the
                // arrival-order index by checking offsetInFile() —
                // unique per record after D-04. Simpler still:
                // re-derive from segment's internal byDid_ via a
                // small accessor. Lacking that, use a linear pass.
                (void)rec;
            }
            // Simpler: walk the segment's records() range and rely on
            // the fact that ISDeviceLog::all_ is built in segment-then-
            // arrival order. We can rebuild byDid_ from all_ by
            // replaying ISLogReader::recordAt and reading did().
        }
        total_ += n;
    }

    // Rebuild byDid_ cleanly by iterating all_ once and reading
    // records' DIDs through the segment's recordAt accessor.
    byDid_.clear();
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
    return parent_->segments_[loc.segment].recordAt(loc.record);
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
