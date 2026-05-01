/**
 * @file derivations_pose.cpp
 * @brief Quaternion-driven derivations: quat→euler, heading-from-quat,
 *        body-rate→euler-rate.
 *
 * Quaternion convention: per `data_sets.h`, INS_2.qn2b is a Hamilton
 * quaternion (W,X,Y,Z) representing body→nav rotation. We feed it
 * directly to the SDK's `quat2euler`, which yields (roll, pitch, yaw)
 * in radians. No frame remapping happens here.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "derivation_helpers.h"

namespace inertial_sense {
namespace derivations {

using detail::missingInput;
using detail::parseWrapPositive;
using detail::wrapHeading;

ISExpected<DerivationResult> evalQuatToEuler(const DerivationContext& ctx) {
    DerivationResult out;
    out.channelNames = { "roll", "pitch", "yaw" };
    out.outputUnit   = "rad";
    out.outputFrame  = "body→NED";

    bool any = false;
    detail::walkPayloads<ins_2_t>(
        ctx.reader(), DID_INS_2,
        [&](TimeStamp ts, const ins_2_t& ins) {
            any = true;
            ixEuler theta;
            ixQuat  q = { ins.qn2b[0], ins.qn2b[1], ins.qn2b[2], ins.qn2b[3] };
            quat2euler(q, theta);
            out.samples.push_back({
                ts,
                { static_cast<double>(theta[0]),
                  static_cast<double>(theta[1]),
                  static_cast<double>(theta[2]) },
            });
        });
    if (!any) return missingInput(DID_INS_2, "qn2b");
    return out;
}

ISExpected<DerivationResult> evalHeadingFromQuat(const DerivationContext& ctx) {
    const bool positive = parseWrapPositive(ctx);

    DerivationResult out;
    out.channelNames = { "heading" };
    out.outputUnit   = "rad";
    out.outputFrame  = "NED";

    bool any = false;
    detail::walkPayloads<ins_2_t>(
        ctx.reader(), DID_INS_2,
        [&](TimeStamp ts, const ins_2_t& ins) {
            any = true;
            ixEuler theta;
            ixQuat  q = { ins.qn2b[0], ins.qn2b[1], ins.qn2b[2], ins.qn2b[3] };
            quat2euler(q, theta);
            out.samples.push_back({
                ts,
                { wrapHeading(static_cast<double>(theta[2]), positive) },
            });
        });
    if (!any) return missingInput(DID_INS_2, "qn2b");
    return out;
}

ISExpected<DerivationResult> evalBodyRateToEulerRate(const DerivationContext& ctx) {
    auto qn2bs = detail::collectPayloads<ins_2_t>(ctx.reader(), DID_INS_2);
    if (qn2bs.empty()) return missingInput(DID_INS_2, "qn2b");

    DerivationResult out;
    out.channelNames = { "roll_rate", "pitch_rate", "yaw_rate" };
    out.outputUnit   = "rad/s";
    out.outputFrame  = "Euler";

    bool anyImu = false;
    detail::walkPayloads<imu_t>(
        ctx.reader(), DID_IMU,
        [&](TimeStamp ts, const imu_t& imu) {
            anyImu = true;
            const ins_2_t* ins = detail::closestByTimestamp(qn2bs, ts);
            if (!ins) return;

            ixEuler theta;
            ixQuat  q = { ins->qn2b[0], ins->qn2b[1], ins->qn2b[2], ins->qn2b[3] };
            quat2euler(q, theta);
            const double roll  = static_cast<double>(theta[0]);
            const double pitch = static_cast<double>(theta[1]);

            const double cr = std::cos(roll),  sr = std::sin(roll);
            const double cp = std::cos(pitch);
            // Singularity guard: pitch close to ±π/2.
            const double tp = (std::abs(cp) > 1e-6) ? (std::sin(pitch) / cp)
                                                    : (std::sin(pitch) / 1e-6);

            const double p  = static_cast<double>(imu.I.pqr[0]);
            const double q_ = static_cast<double>(imu.I.pqr[1]);
            const double r  = static_cast<double>(imu.I.pqr[2]);

            // Standard 3-2-1 Euler-rate transform:
            //   φ̇ = p + sin(φ) tan(θ) q + cos(φ) tan(θ) r
            //   θ̇ =      cos(φ)        q -      sin(φ)        r
            //   ψ̇ =      sin(φ)/cos(θ) q +      cos(φ)/cos(θ) r
            const double rollRate  = p + sr * tp * q_ + cr * tp * r;
            const double pitchRate = cr * q_ - sr * r;
            const double yawRate   = (std::abs(cp) > 1e-6)
                                   ? ((sr / cp) * q_ + (cr / cp) * r)
                                   : 0.0;

            out.samples.push_back({
                ts,
                { rollRate, pitchRate, yawRate },
            });
        });
    if (!anyImu) return missingInput(DID_IMU, "I.pqr");
    return out;
}

} // namespace derivations
} // namespace inertial_sense
