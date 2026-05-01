/**
 * @file derivations_lla.cpp
 * @brief LLA → NED projection — both `@first_fix` and `@user_origin`.
 *
 * Uses the SDK's `lla2ned_d` (`ISEarth.h`) for the actual geodetic
 * math. lla[0]/lla[1] in degrees, lla[2] in metres above WGS84
 * ellipsoid — matches the documented INS_2 convention.
 *
 * The `@first_fix` variant uses the first record's lla as the origin.
 * Today we don't have a fix-quality field exposed via this surface, so
 * we always pick the first record; D-07 era can refine to "first
 * record where `eGpsStatusFix` is Fix-or-better" by re-parsing the
 * GPS records alongside.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "derivation_helpers.h"

namespace inertial_sense {
namespace derivations {

namespace {

DerivationResult makeShell() {
    DerivationResult r;
    r.channelNames = { "n", "e", "d" };
    r.outputUnit   = "m";
    r.outputFrame  = "NED";
    return r;
}

void appendNed(DerivationResult& out, TimeStamp t,
               const double origin[3], const double lla[3]) {
    double mutableOrigin[3] = { origin[0], origin[1], origin[2] };
    double mutableLla[3]    = { lla[0],    lla[1],    lla[2]    };
    ixVector3 ned;
    // INS_2.lla is documented as WGS84 lat/lon in *degrees*; use the
    // degree-input variant (the bare `lla2ned_d` expects radians).
    llaDeg2ned_d(mutableOrigin, mutableLla, ned);
    out.samples.push_back({
        t,
        { static_cast<double>(ned[0]),
          static_cast<double>(ned[1]),
          static_cast<double>(ned[2]) },
    });
}

} // namespace

ISExpected<DerivationResult> evalLlaToNedFirstFix(const DerivationContext& ctx) {
    DerivationResult out = makeShell();
    bool   haveOrigin = false;
    double origin[3]  = { 0.0, 0.0, 0.0 };

    detail::walkPayloads<ins_2_t>(
        ctx.reader(), DID_INS_2,
        [&](TimeStamp ts, const ins_2_t& ins) {
            if (!haveOrigin) {
                origin[0] = ins.lla[0];
                origin[1] = ins.lla[1];
                origin[2] = ins.lla[2];
                haveOrigin = true;
            }
            appendNed(out, ts, origin, ins.lla);
        });
    if (!haveOrigin) return detail::missingInput(DID_INS_2, "lla");
    return out;
}

ISExpected<DerivationResult> evalLlaToNedUserOrigin(const DerivationContext& ctx) {
    auto userOrigin = ctx.param("origin_lla");
    if (!userOrigin) {
        return fail(ISErrorCode::InvalidArgument,
                    "lla_to_ned@user_origin: missing required parameter "
                    "'origin_lla' (expected 'lat_deg,lon_deg,alt_m')");
    }
    auto parsed = detail::parseLlaCsv(*userOrigin);
    if (!parsed) {
        return fail(ISErrorCode::InvalidArgument,
                    "lla_to_ned@user_origin: malformed 'origin_lla' "
                    "(expected 'lat_deg,lon_deg,alt_m')");
    }
    const double origin[3] = { (*parsed)[0], (*parsed)[1], (*parsed)[2] };

    DerivationResult out = makeShell();
    bool any = false;
    detail::walkPayloads<ins_2_t>(
        ctx.reader(), DID_INS_2,
        [&](TimeStamp ts, const ins_2_t& ins) {
            any = true;
            appendNed(out, ts, origin, ins.lla);
        });
    if (!any) return detail::missingInput(DID_INS_2, "lla");
    return out;
}

} // namespace derivations
} // namespace inertial_sense
