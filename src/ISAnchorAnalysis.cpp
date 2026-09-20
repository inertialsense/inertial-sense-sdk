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

#include "ISDataMappings.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

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

const char* anchorConsensusName(AnchorConsensus c) noexcept {
    switch (c) {
        case AnchorConsensus::NotApplicable:                return "NotApplicable";
        case AnchorConsensus::Uncorroborated:               return "Uncorroborated";
        case AnchorConsensus::Corroborated:                 return "Corroborated";
        case AnchorConsensus::DisagreedResolvedByAuthority: return "DisagreedByAuthority";
        case AnchorConsensus::DisagreedUnresolved:          return "DisagreedUNRESOLVED";
    }
    return "?";
}

namespace {

//! A ToW that is plausibly within a GPS week. Guards against a zeroed or garbage field being
//! accepted as an anchor purely because a validity bit happened to be set.
constexpr uint64_t kGpsWeekMs = 604'800'000ULL;

//! Shorthand for the shared classifier. The domain of a record's timestamp is a known fact
//! about its DID -- see cISDataMappings::TimestampDomain() for why this must never be a
//! magnitude test, and for the 451,040 records the old magnitude test got wrong.
using TsDomain = cISDataMappings::eTimestampDomain;

inline TsDomain timestampDomain(uint32_t did) noexcept {
    return cISDataMappings::TimestampDomain(did);
}

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
    // The domain comes from the DID's timestamp FIELD, never from the value's magnitude -- see
    // TsDomain above for why, and for the 451,040 records the magnitude test got wrong.
    const TsDomain domain = timestampDomain(did);
    if (recordTsMs == 0 || domain == TsDomain::TIMESTAMP_DOMAIN_NONE) {
        // No internal time, or a timestamp we cannot place in a domain -- either way it
        // contributes no extremum. (A non-zero timestamp with no declared field should be
        // impossible: the value came FROM that field.)
        ++out_.untimedRecords;
    } else if (domain == TsDomain::TIMESTAMP_DOMAIN_GPS_TOW) {
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

    // ---- Tier 4: ToW-only records. These give an absolute time but no uptime to pair it with,
    // so the offset can only be recovered by correlating against a neighbouring uptime-domain
    // record. Remember the earliest one; pairing happens in finish().
    //
    // Evaluated BEFORE the payload guard below, because it reads nothing out of the payload —
    // the record's own index timestamp IS the time-of-week. Sitting after the guard made this
    // tier unreachable from the index-driven path, which supplies a payload only for the DIDs
    // `needsPayload()` names, so a log with GNSS records but no bridge record would have
    // resolved a full tier lower there than it does from a byte scan. No log in the golden
    // corpus exercises tier 4, so nothing caught it; `TowOnlyAnchorNeedsNoPayload` does.
    if (isTowOnlyCandidate(did) && domain == TsDomain::TIMESTAMP_DOMAIN_GPS_TOW && plausibleTow(recordTsMs)) {
        if (towOnlyDid_ == 0) {
            towOnlyDid_  = did;
            towOnlyTowMs = recordTsMs;
            // The nearest uptime-domain record seen so far is the correlation partner. Records
            // are written in arrival order, so the immediately-preceding uptime record is within
            // one output period of this one.
            towOnlyUpMs = lastUptimeMs_;
        }
        // Offer it regardless of whether it was the first: consensus needs every claim, and a
        // ToW-only source whose offset wanders relative to a bridge is exactly the kind of
        // disagreement worth reporting.
        if (lastUptimeMs_ != 0) {
            offerCandidate(AnchorTier::PayloadToWSingle, did, recordTsMs, lastUptimeMs_);
        }
        return;
    }

    // Past this point a payload is required. The index-driven path has a DID and a timestamp for
    // every record but only pays to re-frame the payload of the DIDs that can actually anchor,
    // so it passes nullptr for the rest. Everything above — per-domain extrema, stall detection,
    // the running uptime, and tier 4 — must work without one.
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
            offerCandidate(AnchorTier::PayloadToWBridge, DID_SYS_PARAMS,
                           sp.timeOfWeekMs, upMs);
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
            offerCandidate(AnchorTier::PayloadToWBridge, DID_GPX_STATUS,
                           gs.timeOfWeekMs, upMs);
        }
        return;
    }
}

void AnchorCollector::offerCandidate(AnchorTier tier, uint32_t did,
                                     uint64_t towMs, uint64_t upMs) {
    const int64_t off = static_cast<int64_t>(towMs) - static_cast<int64_t>(upMs);

    // Fold a repeat of a claim we already hold. A 1 Hz bridge restating the same offset for
    // twenty minutes corroborates it; it does not add a new opinion.
    for (const auto& c : out_.candidates) {
        if (c.did == did && std::llabs(c.offsetMs - off) <= kOffsetAgreementToleranceMs) {
            return;
        }
    }
    if (out_.candidates.size() >= kMaxCandidates) {
        return;   // bounded; see kMaxCandidates
    }

    AnchorCandidate c;
    c.tier     = tier;
    c.did      = did;
    c.towMs    = towMs;
    c.uptimeMs = upMs;
    c.offsetMs = off;
    out_.candidates.push_back(c);
}

void AnchorCollector::resolveConsensus() {
    if (out_.candidates.empty()) {
        out_.consensus = AnchorConsensus::NotApplicable;
        return;
    }

    // Highest authority wins outright (Kyle's option (b)): a bridge record beats a ToW-only
    // record even if the two disagree, because the bridge needs no correlation step.
    AnchorTier best = AnchorTier::None;
    for (const auto& c : out_.candidates) best = std::max(best, c.tier);

    std::vector<std::size_t> top;
    for (std::size_t i = 0; i < out_.candidates.size(); ++i) {
        if (out_.candidates[i].tier == best) top.push_back(i);
    }

    // Among the top authority, find the largest cluster of mutually-agreeing offsets. With one
    // candidate the cluster is itself; with two that agree it is both; with three where two
    // align, the majority carries and the third is the outlier we reject and report.
    std::size_t winner = top.front();
    std::size_t winnerVotes = 0;
    for (std::size_t i : top) {
        std::size_t votes = 0;
        for (std::size_t j : top) {
            if (std::llabs(out_.candidates[i].offsetMs - out_.candidates[j].offsetMs)
                    <= kOffsetAgreementToleranceMs) {
                ++votes;
            }
        }
        // Strictly-greater keeps the FIRST of a tie, so a deadlock stays deterministic.
        if (votes > winnerVotes) { winnerVotes = votes; winner = i; }
    }

    const AnchorCandidate& w = out_.candidates[winner];
    out_.candidates[winner].accepted = true;
    winnerDid_   = w.did;
    winnerTowMs_ = w.towMs;
    winnerUpMs_  = w.uptimeMs;
    winnerOffMs_ = w.offsetMs;
    haveWinner_  = true;

    // Classify the agreement, and report every dissenter regardless of authority -- for an
    // analysis tool a lesser source claiming a different time is precisely what we want to
    // know about, even though it does not change the answer.
    const std::size_t topCount = top.size();
    std::size_t dissentSameTier = 0;
    for (std::size_t i : top) {
        if (i == winner) continue;
        if (std::llabs(out_.candidates[i].offsetMs - w.offsetMs) > kOffsetAgreementToleranceMs) {
            ++dissentSameTier;
        }
    }

    if (topCount == 1) {
        out_.consensus = (out_.candidates.size() > 1)
                             ? AnchorConsensus::DisagreedResolvedByAuthority
                             : AnchorConsensus::Uncorroborated;
    } else if (dissentSameTier == 0) {
        out_.consensus = AnchorConsensus::Corroborated;
    } else if (winnerVotes * 2 > topCount) {
        // A real majority agreed; the minority is rejected as suspect.
        out_.consensus = AnchorConsensus::DisagreedResolvedByAuthority;
    } else {
        // Equal authority, no majority -- the one case we cannot adjudicate.
        out_.consensus = AnchorConsensus::DisagreedUnresolved;
    }

    for (const auto& c : out_.candidates) {
        if (c.accepted) continue;
        const int64_t delta = c.offsetMs - w.offsetMs;
        if (std::llabs(delta) <= kOffsetAgreementToleranceMs) continue;
        const bool equalAuthority = (c.tier == best);
        out_.anomalies.push_back(
            std::string(equalAuthority ? "EQUAL-AUTHORITY clock disagreement: "
                                       : "lesser-authority clock disagrees: ") +
            "DID " + std::to_string(c.did) + " (" + anchorTierName(c.tier) + ") offset " +
            std::to_string(c.offsetMs) + " ms vs accepted DID " + std::to_string(w.did) +
            " (" + anchorTierName(w.tier) + ") offset " + std::to_string(w.offsetMs) +
            " ms; delta " + std::to_string(delta) + " ms" +
            (equalAuthority && out_.consensus == AnchorConsensus::DisagreedUnresolved
                 ? " -- NO CONSENSUS, anchor is not adjudicated"
                 : " -- rejected in favour of the more trustworthy source"));
    }
}

void AnchorCollector::trackStall(uint32_t did, uint64_t recordTsMs) {
    if (recordTsMs == 0) return;
    // Remember the most recent UPTIME-domain timestamp as the correlation partner for a
    // ToW-only anchor. Classified by the DID's timestamp field, not the value's magnitude.
    if (timestampDomain(did) == TsDomain::TIMESTAMP_DOMAIN_UPTIME) lastUptimeMs_ = recordTsMs;

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
    // ---- Adjudicate every absolute-time claim collected during the scan. This replaces
    // "first valid candidate wins": the winner is the most trustworthy claim, ties among equal
    // authority are broken by majority, and dissenters are reported whether or not they change
    // the answer. See resolveConsensus() and AnchorConsensus.
    resolveConsensus();

    // ---- Resolve the tier, strongest first.
    if (haveWinner_) {
        const auto accepted = std::find_if(out_.candidates.begin(), out_.candidates.end(),
                                           [](const AnchorCandidate& c) { return c.accepted; });
        out_.tier           = (accepted != out_.candidates.end()) ? accepted->tier
                                                                  : AnchorTier::None;
        out_.anchorDid      = winnerDid_;
        out_.anchorTowMs    = winnerTowMs_;
        out_.anchorUptimeMs = winnerUpMs_;
        out_.offsetMs       = winnerOffMs_;
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
