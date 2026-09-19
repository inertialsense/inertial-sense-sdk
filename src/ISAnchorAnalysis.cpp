/**
 * @file ISAnchorAnalysis.cpp
 * @brief Anchor-tier cascade — resolves a segment's absolute start time and how sure we are.
 *
 * @note SN-8629.
 */

#include "ISAnchorAnalysis.h"
#include "ISAnchorCollector.h"
#include "data_sets.h"
#include "core/msg_logger.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace inertial_sense {

const char* anchorTierName(AnchorTier t) noexcept {
    switch (t) {
        case AnchorTier::None:               return "None";
        case AnchorTier::FilenameAnchor:     return "FilenameAnchor";
        case AnchorTier::PrevSegmentChained: return "PrevSegmentChained";
        case AnchorTier::BridgedToW:         return "BridgedToW";
        case AnchorTier::PayloadToWSingle:   return "PayloadToWSingle";
        case AnchorTier::PayloadToWBridge:   return "PayloadToWBridge";
    }
    return "?";
}

namespace {

//! A ToW that is plausibly within a GPS week. Guards against a zeroed or garbage field being
//! accepted as an anchor purely because a validity bit happened to be set.
constexpr uint64_t kGpsWeekMs = 604'800'000ULL;

//! Boundary separating the two domains a record timestamp may be in. GPS time-of-week runs to
//! 604,800,000 ms; host uptime would have to be a 7-day continuous run to reach it, which no
//! log in practice does. Used only to bucket RECORD timestamps, never payload fields.
constexpr uint64_t kDomainSplitMs = 100'000'000ULL;

//! Tolerance on the (ToW - uptime) offset before two anchors in the same segment are treated as
//! disagreeing. Sized to swallow the inter-record spacing at typical output rates (a bridge
//! record and a ToW-only record sampled one output period apart legitimately differ by that
//! period) while still catching a genuinely stale or wrong clock.
constexpr int64_t kOffsetAgreementToleranceMs = 250;

/** @return  True when @p towMs sits inside a GPS week and is not the zero sentinel. */
inline bool plausibleTow(uint64_t towMs) noexcept {
    return towMs > 0 && towMs < kGpsWeekMs;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// AnchorCollector
// ---------------------------------------------------------------------------------------------

void AnchorCollector::consume(uint32_t did, uint16_t structOffset, const uint8_t* payload,
                              uint32_t payloadSize, uint64_t recordTsMs) {
    ++seen_;

    // Bucket the RECORD timestamp by domain so the segment's extrema are per-domain rather than
    // positional. This is what replaces records_.front()/back().timestamp as the header source.
    if (recordTsMs == 0) {
        ++out_.untimedRecords;
    } else if (recordTsMs >= kDomainSplitMs) {
        ++out_.towRecords;
        out_.towMinMs = out_.towMinMs ? std::min(out_.towMinMs, recordTsMs) : recordTsMs;
        out_.towMaxMs = std::max(out_.towMaxMs, recordTsMs);
    } else {
        ++out_.uptimeRecords;
        out_.uptimeMinMs = out_.uptimeMinMs ? std::min(out_.uptimeMinMs, recordTsMs) : recordTsMs;
        out_.uptimeMaxMs = std::max(out_.uptimeMaxMs, recordTsMs);
    }

    // A partial record (dataHdr.offset != 0) is one chunk of a larger struct. It carries a real
    // timestamp, so it counted toward the extrema above, but it is excluded from everything
    // below: its bytes do not hold the anchor fields at the expected positions, and several
    // chunks of one logical record share a timestamp, which would read as a stalled clock.
    if (structOffset != 0) return;

    trackStall(did, recordTsMs);

    // Timestamp-only observation. The index-driven path (`ISLogReader::analyzeFromRecords`) has a
    // DID and a timestamp for every record but only pays to re-frame the payload of the few DIDs
    // that can actually anchor, so it passes nullptr for the rest. Everything above this point —
    // per-domain extrema, stall detection, and the running uptime that a ToW-only anchor
    // correlates against — needs the whole record stream to be correct.
    if (payload == nullptr) return;

    // ---- Tier 5: dual-domain bridge records. ToW and uptime in ONE payload, so the offset
    // needs no correlation against a neighbour. DID_SYS_PARAMS is the IMX bridge and
    // DID_GPX_STATUS the GPX equivalent; they are equally trustworthy, and a GPX-sourced log
    // may contain no DID_SYS_PARAMS at all.
    if (did == DID_SYS_PARAMS && payloadSize >= sizeof(sys_params_t)) {
        sys_params_t sp{};
        std::memcpy(&sp, payload, sizeof(sp));
        const bool towValid = (sp.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
        if (towValid && plausibleTow(sp.timeOfWeekMs) && sp.upTime > 0.0) {
            const uint64_t upMs = static_cast<uint64_t>(sp.upTime * 1000.0);
            offerBridge(DID_SYS_PARAMS, sp.timeOfWeekMs, upMs);
        }
        return;
    }
    if (did == DID_GPX_STATUS && payloadSize >= sizeof(gpx_status_t)) {
        gpx_status_t gs{};
        std::memcpy(&gs, payload, sizeof(gs));
        const bool towValid = (gs.hdwStatus & (GPX_HDW_STATUS_GNSS1_TIME_OF_WEEK_VALID |
                                               GPX_HDW_STATUS_GNSS2_TIME_OF_WEEK_VALID)) != 0;
        if (towValid && plausibleTow(gs.timeOfWeekMs) && gs.upTime > 0.0) {
            const uint64_t upMs = static_cast<uint64_t>(gs.upTime * 1000.0);
            offerBridge(DID_GPX_STATUS, gs.timeOfWeekMs, upMs);
        }
        return;
    }

    // ---- Tier 4: ToW-only records. These give an absolute time but no uptime to pair it with,
    // so the offset can only be recovered by correlating against a neighbouring uptime-domain
    // record. Remember the earliest one; pairing happens in finish().
    if (isTowOnlyCandidate(did) && recordTsMs >= kDomainSplitMs && plausibleTow(recordTsMs)) {
        if (towOnlyDid_ == 0) {
            towOnlyDid_  = did;
            towOnlyTowMs = recordTsMs;
            // The nearest uptime-domain record seen so far is the correlation partner. Records
            // are written in arrival order, so the immediately-preceding uptime record is within
            // one output period of this one.
            towOnlyUpMs = lastUptimeMs_;
        }
    }
}

void AnchorCollector::offerBridge(uint32_t did, uint64_t towMs, uint64_t upMs) {
    const int64_t off = static_cast<int64_t>(towMs) - static_cast<int64_t>(upMs);

    if (bridgeDid_ == 0) {
        // Earliest bridge record in the segment wins.
        bridgeDid_   = did;
        bridgeTowMs_ = towMs;
        bridgeUpMs_  = upMs;
        bridgeOffMs_ = off;
        return;
    }

    // A second bridge from the OTHER device disagreeing with the first is an anomaly worth
    // surfacing: the IMX and GPX clocks should agree, and if they do not, neither key is
    // trustworthy without a human looking. The first (earliest) still wins so ordering stays
    // deterministic.
    if (did != bridgeDid_ && std::llabs(off - bridgeOffMs_) > kOffsetAgreementToleranceMs) {
        if (!bridgeDisagreementReported_) {
            bridgeDisagreementReported_ = true;
            out_.anomalies.push_back(
                "bridge clocks disagree: DID " + std::to_string(bridgeDid_) + " offset " +
                std::to_string(bridgeOffMs_) + " ms vs DID " + std::to_string(did) + " offset " +
                std::to_string(off) + " ms (delta " + std::to_string(off - bridgeOffMs_) +
                " ms); using the earlier");
        }
    }
}

void AnchorCollector::trackStall(uint32_t did, uint64_t recordTsMs) {
    if (recordTsMs == 0) return;
    if (recordTsMs < kDomainSplitMs) lastUptimeMs_ = recordTsMs;

    auto& st = stalls_[did];
    ++st.count;
    if (st.count == 1) { st.lastTsMs = recordTsMs; st.repeats = 0; return; }

    if (recordTsMs == st.lastTsMs) {
        ++st.repeats;
    } else {
        // A DID that was stalled and then moved again: remember the worst run so finish() can
        // report it even though the DID recovered.
        st.worstRepeats = std::max(st.worstRepeats, st.repeats);
        st.repeats      = 0;
        st.lastTsMs     = recordTsMs;
    }
}

AnchorAnalysis AnchorCollector::finish(const AnchorAnalysis* prev) {
    // ---- Resolve the tier, strongest first.
    if (bridgeDid_ != 0) {
        out_.tier           = AnchorTier::PayloadToWBridge;
        out_.anchorDid      = bridgeDid_;
        out_.anchorTowMs    = bridgeTowMs_;
        out_.anchorUptimeMs = bridgeUpMs_;
        out_.offsetMs       = bridgeOffMs_;
    } else if (towOnlyDid_ != 0 && towOnlyUpMs != 0) {
        out_.tier           = AnchorTier::PayloadToWSingle;
        out_.anchorDid      = towOnlyDid_;
        out_.anchorTowMs    = towOnlyTowMs;
        out_.anchorUptimeMs = towOnlyUpMs;
        out_.offsetMs       = static_cast<int64_t>(towOnlyTowMs) - static_cast<int64_t>(towOnlyUpMs);
    } else if (prev != nullptr && prev->anchored() && prev->offsetMs != 0 &&
               out_.uptimeMinMs != 0 && out_.uptimeMinMs >= prev->uptimeMaxMs) {
        // No absolute time of our own, but the previous segment's offset applies because our
        // uptime picks up where theirs left off.
        out_.tier     = AnchorTier::BridgedToW;
        out_.offsetMs = prev->offsetMs;
        out_.anomalies.push_back("no ToW-bearing record in this segment; carried the previous "
                                 "segment's offset forward");
    } else if (prev != nullptr && prev->anchored()) {
        // Nothing to bridge with either — chain from the predecessor's end.
        out_.tier = AnchorTier::PrevSegmentChained;
        if (out_.uptimeMinMs != 0) {
            out_.offsetMs = static_cast<int64_t>(prev->anchoredEndMs) -
                            static_cast<int64_t>(out_.uptimeMinMs);
        }
        out_.anomalies.push_back("no absolute time available; chained from the previous "
                                 "segment's end");
    } else if (filenameAnchorMs_ != 0) {
        out_.tier = AnchorTier::FilenameAnchor;
        if (out_.uptimeMinMs != 0) {
            out_.offsetMs = static_cast<int64_t>(filenameAnchorMs_) -
                            static_cast<int64_t>(out_.uptimeMinMs);
        }
        out_.anomalies.push_back("anchored to the segment filename; no in-log absolute time");
    } else {
        out_.tier = AnchorTier::None;
        out_.anomalies.push_back("no anchor of any kind; segment is session-relative only");
    }

    // ---- Project the segment's span onto the absolute frame. Prefer the uptime domain, which
    // every device stamps continuously; fall back to the ToW domain for a segment that carries
    // nothing else (a GPX-only capture, for instance).
    if (out_.tier != AnchorTier::None) {
        if (out_.uptimeRecords > 0 && out_.uptimeMinMs != 0) {
            out_.anchoredStartMs = static_cast<uint64_t>(
                static_cast<int64_t>(out_.uptimeMinMs) + out_.offsetMs);
            out_.anchoredEndMs = static_cast<uint64_t>(
                static_cast<int64_t>(out_.uptimeMaxMs) + out_.offsetMs);
        } else if (out_.towRecords > 0) {
            out_.anchoredStartMs = out_.towMinMs;
            out_.anchoredEndMs   = out_.towMaxMs;
        }
    }

    // ---- Anomalies the application layer surfaces instead of drawing through.
    for (const auto& [did, st] : stalls_) {
        const std::size_t worst = std::max(st.worstRepeats, st.repeats);
        if (worst >= kStallReportThreshold) {
            out_.anomalies.push_back(
                "DID " + std::to_string(did) + " emitted " + std::to_string(worst + 1) +
                " consecutive records with an identical timestamp (" +
                std::to_string(st.lastTsMs) + " ms) - its clock stalled while it kept reporting");
        }
    }

    if (prev != nullptr && prev->anchored() && out_.tier < prev->tier) {
        out_.anomalies.push_back(
            std::string("anchor durability dropped from ") + anchorTierName(prev->tier) +
            " to " + anchorTierName(out_.tier) + " relative to the previous segment");
    }

    log_debug(IS_LOG_ISLOG,
              "AnchorCollector: tier=%s did=%u off=%lld anchored=[%llu..%llu] "
              "(uptime=%zu tow=%zu untimed=%zu) anomalies=%zu\n",
              anchorTierName(out_.tier), out_.anchorDid, (long long)out_.offsetMs,
              (unsigned long long)out_.anchoredStartMs, (unsigned long long)out_.anchoredEndMs,
              out_.uptimeRecords, out_.towRecords, out_.untimedRecords, out_.anomalies.size());

    return out_;
}

bool AnchorCollector::needsPayload(uint32_t did) noexcept {
    // Only the dual-domain bridge records are read out of their payload. Tier-4 ToW-only
    // candidates are recognized from the record's index timestamp alone, so a caller working
    // from an index never has to re-frame them.
    return did == DID_SYS_PARAMS || did == DID_GPX_STATUS;
}

bool AnchorCollector::isTowOnlyCandidate(uint32_t did) noexcept {
    switch (did) {
        case DID_INS_1:
        case DID_INS_2:
        case DID_INS_3:
        case DID_INS_4:
        case DID_GNSS1_POS:
        case DID_GNSS2_POS:
        case DID_GNSS1_VEL:
        case DID_GNSS2_VEL:
            return true;
        default:
            return false;
    }
}

}  // namespace inertial_sense
