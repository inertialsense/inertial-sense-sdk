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
#include <cstdint>    // UINT32_MAX -- the time-offset field is 32-bit
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

TimeSource timeSourceForTier(AnchorTier t) noexcept {
    switch (t) {
        case AnchorTier::PayloadToWBridge:
        case AnchorTier::PayloadToWSingle:   return TimeSource::PayloadToW;
        case AnchorTier::BridgedToW:
        case AnchorTier::PrevSegmentChained: return TimeSource::ResolvedViaSync;
        case AnchorTier::FilenameAnchor:     return TimeSource::FileTimeAnchored;
        case AnchorTier::None:               return TimeSource::SessionOnly;
    }
    return TimeSource::SessionOnly;
}

TimeStamp anchoredTimeStamp(uint64_t ms, AnchorTier t, uint64_t deviceId) noexcept {
    switch (timeSourceForTier(t)) {
        case TimeSource::PayloadToW:
            return TimeStamp::fromPayloadToW(ms, deviceId);
        case TimeSource::ResolvedViaSync:
            // A chained/bridged anchor is an inference from a sibling segment, not a witnessed
            // value here -- `Interpolated` is what that is.
            return TimeStamp::fromResolvedViaSync(ms, deviceId, TimeConfidence::Interpolated);
        case TimeSource::FileTimeAnchored:
            return TimeStamp::fromFileTimeAnchored(ms, deviceId);
        case TimeSource::HostReceived:
        case TimeSource::SessionOnly:
            break;
    }
    return TimeStamp::fromSessionOnly(ms, deviceId);
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

/**
 * @brief Agreement tolerance on the `(ToW - uptime)` offset, PER TIER PAIR.
 *
 * A tier-5 bridge reads both halves of the offset out of ONE record, so two bridges that agree
 * should agree almost exactly -- a tight tolerance there is meaningful. A tier-4 ToW-only
 * candidate cannot: it pairs its ToW with the nearest PRECEDING uptime-domain record, which is up
 * to one output period earlier. That correlation error is systematic, not noise.
 *
 * Measured on the 542-segment golden corpus, tier-4-vs-tier-5 offset deltas cluster at
 * **-255..-370 ms (p50 -351)**. A flat 250 ms tolerance therefore flagged 278 false
 * disagreements and buried the one genuine outlier (-502,446,094 ms, ~5.8 days) among them --
 * which is the worst possible state for an anomaly channel: the real signal becomes invisible.
 * Kyle's framing was "we need to know about it... within reason"; 278 cries of wolf is not that.
 *
 * So: tight between two bridges, one output period of slack when a tier-4 claim is involved, and
 * double that when BOTH are tier 4 (each carries its own correlation error, and they can lean in
 * opposite directions).
 */
constexpr int64_t kBridgeAgreementToleranceMs = 250;    //!< tier 5 vs tier 5 -- no correlation step
constexpr int64_t kCorrelatedToleranceMs      = 600;    //!< one correlation step (>= the measured 370)

//! Tolerance for comparing two offset claims of the given tiers.
inline int64_t agreementToleranceMs(AnchorTier a, AnchorTier b) noexcept {
    const int correlated = (a == AnchorTier::PayloadToWSingle ? 1 : 0) +
                           (b == AnchorTier::PayloadToWSingle ? 1 : 0);
    if (correlated == 0) return kBridgeAgreementToleranceMs;
    return kCorrelatedToleranceMs * correlated;
}

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

    // Past this point a payload is required. The index-driven path has a DID and a timestamp for
    // every record but only pays to re-frame the payload of the DIDs that can actually anchor
    // (`needsPayload()`), so it passes nullptr for the rest. Everything above — per-domain
    // extrema, stall detection and the running uptime — must work without one.
    if (payload == nullptr) return;

    // ---- Tier 4: ToW-only records. These give an absolute time but no uptime to pair it with,
    // so the offset can only be recovered by correlating against a neighbouring uptime-domain
    // record.
    //
    // VALIDITY-GATED, like every other tier. It previously was not -- it accepted any
    // plausible-looking ToW -- and that was masked by an accident: the old magnitude domain test
    // misfiled small ToW values as uptime, so a GNSS-less log never produced a tier-4 candidate
    // at all. Fixing the domain classifier removed that accidental safety net and four AHRS
    // captures in the golden corpus (22 segments) immediately began anchoring at
    // PayloadToWSingle off `DID_INS_2.timeOfWeek` -- a field with no meaning on a device that
    // never had GNSS. Claiming tier 4 there is worse than admitting FilenameAnchor, because it
    // asserts confidence we do not have.
    //
    // `ins_1/2/3/4_t.hdwStatus` is documented as a copy of `DID_SYS_PARAMS.hdwStatus`, so INS
    // records can use the SAME gate as the tier-5 bridge. GNSS position/velocity carry a fix
    // type in `status` instead.
    if (isTowOnlyCandidate(did) && domain == TsDomain::TIMESTAMP_DOMAIN_GPS_TOW &&
        plausibleTow(recordTsMs) && towOnlyPayloadIsValid(did, payload, payloadSize)) {
        if (towOnlyDid_ == 0) {
            towOnlyDid_  = did;
            towOnlyTowMs = recordTsMs;
            // The nearest uptime-domain record seen so far is the correlation partner. Records
            // are written in arrival order, so the immediately-preceding uptime record is within
            // one output period of this one -- which is also why this tier's offset carries a
            // systematic correlation error; see agreementToleranceMs().
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
        if (c.did == did &&
            std::llabs(c.offsetMs - off) <= agreementToleranceMs(c.tier, tier)) {
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
                    <= agreementToleranceMs(out_.candidates[i].tier, out_.candidates[j].tier)) {
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
        if (std::llabs(out_.candidates[i].offsetMs - w.offsetMs) >
                agreementToleranceMs(out_.candidates[i].tier, w.tier)) {
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
        if (std::llabs(delta) <= agreementToleranceMs(c.tier, w.tier)) continue;
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

    // ---- Resolve the LOG's uptime zero first: the filename branch below needs it, and it is a
    // log-level constant rather than anything about this segment. Record uptimes form one
    // continuous axis across a log's segments, so subtracting this zero turns any record's
    // uptime into elapsed-time-into-the-log. See AnchorAnalysis::logStartUptimeMs.
    if (prev != nullptr && prev->logStartUptimeMs != 0) {
        out_.logStartUptimeMs = prev->logStartUptimeMs;      // inherited; identical log-wide
    } else if (isFirstSegmentOfLog_ && out_.uptimeMinMs != 0) {
        out_.logStartUptimeMs = out_.uptimeMinMs;            // this segment IS the log's start
    } else {
        out_.logStartUptimeMs = 0;                           // early segments culled: unknowable
    }

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
        // Re-anchor the LOG to the filename timestamp -- ONE offset for the whole log, not a
        // per-segment calculation. Kyle 2026-09-21: this tier is reached only when the log
        // carries no in-log wall clock at all, so every record is relative to device uptime and
        // the log as a whole can be safely re-anchored; the result is inaccurate in absolute
        // terms and that is accepted, because "without a durable wall-clock anchor from within
        // the log itself, any conclusion about the actual time is purely hearsay" -- and some
        // absolute time beats none.
        //
        // This previously subtracted THIS SEGMENT's `uptimeMinMs`, which then cancelled against
        // it in `anchoredStartMs = uptimeMinMs + offsetMs`, so every filename-anchored segment
        // of a log reported `filenameAnchorMs` identically. Two segments 90 s apart measured a
        // 0 ms anchored gap, and `ISDeviceLog::fromSegments` -- which orders on
        // `anchoredStartMs` -- was left sorting on all-equal keys.
        //
        // Subtracting the LOG's zero instead places the log's first record exactly on the
        // filename timestamp and every later record at its true elapsed offset from it. When the
        // log's early segments were culled the zero is unknowable and stays 0, which shifts the
        // whole log late by the device's pre-logging uptime -- uniformly, so the log's internal
        // geometry remains exact.
        out_.tier = AnchorTier::FilenameAnchor;
        out_.offsetMs = static_cast<int64_t>(filenameAnchorMs_) -
                        static_cast<int64_t>(out_.logStartUptimeMs);
        out_.anomalies.push_back(
            out_.logStartUptimeMs != 0
                ? "anchored to the log filename; no in-log absolute time"
                : "anchored to the log filename, and the log's first segment is absent, so the "
                  "whole log may be offset late by the device's uptime before logging began");
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

std::vector<SessionAdoption>
    planSessionAdoptions(const std::vector<AnchorAnalysis>& perSegment) {
    std::vector<SessionAdoption> out;
    const std::size_t n = perSegment.size();
    if (n == 0) return out;

    // --- Partition into recording sessions on uptime resets.
    std::vector<std::size_t> sessionOf(n, 0);
    std::size_t sessions = 0;
    uint64_t    prevUptimeMin = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const uint64_t up = perSegment[i].uptimeMinMs;
        if (i > 0 && up != 0 && prevUptimeMin != 0 && up < prevUptimeMin) {
            ++sessions;   // uptime went backwards: the device rebooted here
        }
        sessionOf[i] = sessions;
        if (up != 0) prevUptimeMin = up;
    }

    // --- Per session, pick the most trustworthy FIRST-HAND anchor and share it both ways.
    for (std::size_t sess = 0; sess <= sessions; ++sess) {
        std::size_t donor = n;
        for (std::size_t i = 0; i < n; ++i) {
            if (sessionOf[i] != sess) continue;
            const AnchorAnalysis& a = perSegment[i];
            if (!a.firstHand() || a.offsetMs == 0) continue;
            if (donor == n) { donor = i; continue; }
            const AnchorAnalysis& d = perSegment[donor];
            if (a.tier > d.tier || (a.tier == d.tier && a.consensus > d.consensus)) donor = i;
        }
        if (donor == n) continue;   // no absolute time anywhere in this session

        const AnchorAnalysis& d = perSegment[donor];
        for (std::size_t i = 0; i < n; ++i) {
            if (sessionOf[i] != sess || i == donor) continue;
            const AnchorAnalysis& a = perSegment[i];
            if (a.firstHand()) continue;                              // keep first-hand evidence
            if (a.uptimeRecords == 0 || a.uptimeMinMs == 0) continue;  // nothing to project onto
            SessionAdoption s;
            s.segment        = i;
            s.offsetMs       = d.offsetMs;
            s.donorDid       = d.anchorDid;
            s.donorIsEarlier = (donor < i);
            out.push_back(s);
        }
    }
    return out;
}

bool AnchorCollector::towOnlyPayloadIsValid(uint32_t did, const uint8_t* payload,
                                            uint32_t payloadSize) noexcept {
    if (payload == nullptr) return false;
    switch (did) {
        case DID_INS_1: {
            if (payloadSize < sizeof(ins_1_t)) return false;
            ins_1_t v{}; std::memcpy(&v, payload, sizeof(v));
            return v.week != 0 && (v.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
        }
        case DID_INS_2: {
            if (payloadSize < sizeof(ins_2_t)) return false;
            ins_2_t v{}; std::memcpy(&v, payload, sizeof(v));
            return v.week != 0 && (v.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
        }
        case DID_INS_3: {
            if (payloadSize < sizeof(ins_3_t)) return false;
            ins_3_t v{}; std::memcpy(&v, payload, sizeof(v));
            return v.week != 0 && (v.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
        }
        case DID_INS_4: {
            if (payloadSize < sizeof(ins_4_t)) return false;
            ins_4_t v{}; std::memcpy(&v, payload, sizeof(v));
            return v.week != 0 && (v.hdwStatus & HDW_STATUS_GNSS_TIME_OF_WEEK_VALID) != 0;
        }
        case DID_GNSS1_POS:
        case DID_GNSS2_POS: {
            if (payloadSize < sizeof(gnss_pos_t)) return false;
            gnss_pos_t v{}; std::memcpy(&v, payload, sizeof(v));
            // A receiver reporting no fix has no GPS time to offer, whatever its ToW field says.
            return v.week != 0 &&
                   (v.status & GNSS_STATUS_FIX_MASK) != GNSS_STATUS_FIX_NONE;
        }
        case DID_GNSS1_VEL:
        case DID_GNSS2_VEL: {
            if (payloadSize < sizeof(gnss_vel_t)) return false;
            gnss_vel_t v{}; std::memcpy(&v, payload, sizeof(v));
            // gnss_vel_t carries no `week`, so the fix type is the only signal available.
            return (v.status & GNSS_STATUS_FIX_MASK) != GNSS_STATUS_FIX_NONE;
        }
        default:
            return false;
    }
}

bool AnchorCollector::needsPayload(uint32_t did) noexcept {
    // Bridge records are read out of their payload for BOTH halves of the offset. Tier-4
    // ToW-only candidates take their time from the record's index timestamp but their VALIDITY
    // from the payload (see towOnlyPayloadIsValid), so they need it too -- without it the
    // index-driven path would accept an ungated candidate and reach a tier the byte-scan path
    // correctly refuses, which is exactly the route disagreement the tests assert against.
    return did == DID_SYS_PARAMS || did == DID_GPX_STATUS || isTowOnlyCandidate(did);
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
