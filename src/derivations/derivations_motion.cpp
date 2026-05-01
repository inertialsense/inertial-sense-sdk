/**
 * @file derivations_motion.cpp
 * @brief Motion-derived scalars + body→NED rotations.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "derivation_helpers.h"

namespace inertial_sense {
namespace derivations {

namespace {

inline double norm3(const float v[3]) {
    return std::sqrt(static_cast<double>(v[0]) * v[0]
                   + static_cast<double>(v[1]) * v[1]
                   + static_cast<double>(v[2]) * v[2]);
}

} // namespace

ISExpected<DerivationResult> evalVelocityMagnitude(const DerivationContext& ctx) {
    DerivationResult out;
    out.channelNames = { "speed" };
    out.outputUnit   = "m/s";
    out.outputFrame  = "";  // magnitude is frame-invariant

    bool any = false;
    detail::walkPayloads<ins_2_t>(
        ctx.reader(), DID_INS_2,
        [&](TimeStamp ts, const ins_2_t& ins) {
            any = true;
            out.samples.push_back({ ts, { norm3(ins.uvw) } });
        });
    if (!any) return detail::missingInput(DID_INS_2, "uvw");
    return out;
}

ISExpected<DerivationResult> evalAccelMagnitude(const DerivationContext& ctx) {
    DerivationResult out;
    out.channelNames = { "accel" };
    out.outputUnit   = "m/s²";
    out.outputFrame  = "";

    bool any = false;
    detail::walkPayloads<imu_t>(
        ctx.reader(), DID_IMU,
        [&](TimeStamp ts, const imu_t& imu) {
            any = true;
            out.samples.push_back({ ts, { norm3(imu.I.acc) } });
        });
    if (!any) return detail::missingInput(DID_IMU, "I.acc");
    return out;
}

ISExpected<DerivationResult> evalRateMagnitude(const DerivationContext& ctx) {
    DerivationResult out;
    out.channelNames = { "rate" };
    out.outputUnit   = "rad/s";
    out.outputFrame  = "";

    bool any = false;
    detail::walkPayloads<imu_t>(
        ctx.reader(), DID_IMU,
        [&](TimeStamp ts, const imu_t& imu) {
            any = true;
            out.samples.push_back({ ts, { norm3(imu.I.pqr) } });
        });
    if (!any) return detail::missingInput(DID_IMU, "I.pqr");
    return out;
}

ISExpected<DerivationResult> evalBodyAccelToNed(const DerivationContext& ctx) {
    auto qn2bs = detail::collectPayloads<ins_2_t>(ctx.reader(), DID_INS_2);
    if (qn2bs.empty()) return detail::missingInput(DID_INS_2, "qn2b");

    bool keepGravity = true;
    if (auto p = ctx.param("include_gravity")) {
        keepGravity = !(*p == std::string_view{"false"} || *p == std::string_view{"0"});
    }

    DerivationResult out;
    out.channelNames = { "n", "e", "d" };
    out.outputUnit   = "m/s²";
    out.outputFrame  = "NED";

    bool anyImu = false;
    detail::walkPayloads<imu_t>(
        ctx.reader(), DID_IMU,
        [&](TimeStamp ts, const imu_t& imu) {
            anyImu = true;
            const ins_2_t* ins = detail::closestByTimestamp(qn2bs, ts);
            if (!ins) return;

            ixVector3 body = { imu.I.acc[0], imu.I.acc[1], imu.I.acc[2] };
            ixVector3 nav;
            ixQuat    q = { ins->qn2b[0], ins->qn2b[1], ins->qn2b[2], ins->qn2b[3] };
            // quatRot rotates a body-frame vector by `q` (body→nav)
            // into the nav frame.
            quatRot(nav, q, body);

            double n = nav[0], e = nav[1], d = nav[2];
            if (!keepGravity) {
                d -= detail::kStandardGravity;
            }
            out.samples.push_back({ ts, { n, e, d } });
        });
    if (!anyImu) return detail::missingInput(DID_IMU, "I.acc");
    return out;
}

ISExpected<DerivationResult> evalHeadingFromVelocity(const DerivationContext& ctx) {
    const bool positive = detail::parseWrapPositive(ctx);

    DerivationResult out;
    out.channelNames = { "heading" };
    out.outputUnit   = "rad";
    out.outputFrame  = "NED";

    bool any = false;
    detail::walkPayloads<ins_2_t>(
        ctx.reader(), DID_INS_2,
        [&](TimeStamp ts, const ins_2_t& ins) {
            any = true;
            ixVector3 uvw = { ins.uvw[0], ins.uvw[1], ins.uvw[2] };
            ixVector3 vNed;
            ixQuat    q = { ins.qn2b[0], ins.qn2b[1], ins.qn2b[2], ins.qn2b[3] };
            quatRot(vNed, q, uvw);

            // Skip near-zero velocity samples — heading is undefined at
            // rest. Threshold matches the typical INS noise floor for
            // body-frame velocity (~1 cm/s).
            const double horiz = std::sqrt(
                static_cast<double>(vNed[0]) * vNed[0]
              + static_cast<double>(vNed[1]) * vNed[1]);
            if (horiz < 0.01) return;

            const double h = std::atan2(static_cast<double>(vNed[1]),
                                        static_cast<double>(vNed[0]));
            out.samples.push_back({
                ts,
                { detail::wrapHeading(h, positive) },
            });
        });
    if (!any) return detail::missingInput(DID_INS_2, "uvw");
    return out;
}

} // namespace derivations
} // namespace inertial_sense
