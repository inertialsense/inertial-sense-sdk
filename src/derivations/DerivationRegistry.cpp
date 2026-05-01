/**
 * @file DerivationRegistry.cpp
 * @brief Singleton registration of the v1 built-in derivation catalog.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "DerivationRegistry.h"

#include "data_sets.h"

namespace inertial_sense {
namespace derivations {

namespace {

/// Helper that builds + registers one entry. Inserts into both the
/// keyed map and the order vector.
void addEntry(std::map<std::string, Derivation>& entries,
              std::vector<const Derivation*>&   order,
              Derivation                        d) {
    auto [it, inserted] = entries.emplace(std::string(d.name), std::move(d));
    if (inserted) {
        order.push_back(&it->second);
    }
}

} // namespace

DerivationRegistry::DerivationRegistry() {
    // ---- Pose -----------------------------------------------------------
    addEntry(entries_, order_, Derivation{
        "quat_to_euler",
        "Roll/pitch/yaw from INS_2.qn2b (Hamilton, body→nav).",
        "rad", "body→NED",
        { { DID_INS_2, "qn2b" } },
        {},
        &evalQuatToEuler,
    });
    addEntry(entries_, order_, Derivation{
        "heading_from_quat",
        "Yaw component of INS_2.qn2b, wrapped per `wrap` parameter.",
        "rad", "NED",
        { { DID_INS_2, "qn2b" } },
        { { "wrap", "signed", "signed → [-pi, pi]; positive → [0, 2pi]" } },
        &evalHeadingFromQuat,
    });

    // ---- Position -------------------------------------------------------
    addEntry(entries_, order_, Derivation{
        "lla_to_ned@first_fix",
        "INS_2.lla projected to NED with origin at the first record's "
        "lla. Falls back to first record if no fix-quality marker is "
        "available.",
        "m", "NED",
        { { DID_INS_2, "lla" } },
        {},
        &evalLlaToNedFirstFix,
    });
    addEntry(entries_, order_, Derivation{
        "lla_to_ned@user_origin",
        "INS_2.lla projected to NED with origin from `origin_lla` "
        "parameter (comma-separated `lat,lon,alt`).",
        "m", "NED",
        { { DID_INS_2, "lla" } },
        { { "origin_lla", "", "lat_deg,lon_deg,alt_m" } },
        &evalLlaToNedUserOrigin,
    });

    // ---- Magnitudes (frame-invariant scalars) ---------------------------
    addEntry(entries_, order_, Derivation{
        "velocity_magnitude",
        "‖uvw‖ from INS_2 (frame-invariant; equal to ‖vNED‖).",
        "m/s", "",
        { { DID_INS_2, "uvw" } },
        {},
        &evalVelocityMagnitude,
    });
    addEntry(entries_, order_, Derivation{
        "accel_magnitude",
        "‖I.acc‖ from IMU.",
        "m/s²", "",
        { { DID_IMU, "I.acc" } },
        {},
        &evalAccelMagnitude,
    });
    addEntry(entries_, order_, Derivation{
        "rate_magnitude",
        "‖I.pqr‖ from IMU.",
        "rad/s", "",
        { { DID_IMU, "I.pqr" } },
        {},
        &evalRateMagnitude,
    });

    // ---- Body→NED rotations --------------------------------------------
    addEntry(entries_, order_, Derivation{
        "body_accel_to_ned_accel",
        "Rotates IMU.I.acc into NED via INS_2.qn2b. `include_gravity` "
        "parameter controls whether g is removed (default = true: "
        "physics-as-measured; false subtracts +9.80665 m/s² from D).",
        "m/s²", "NED",
        { { DID_IMU, "I.acc" }, { DID_INS_2, "qn2b" } },
        { { "include_gravity", "true", "true → leave gravity in; false → subtract g from D component" } },
        &evalBodyAccelToNed,
    });
    addEntry(entries_, order_, Derivation{
        "body_rate_to_euler_rate",
        "Body angular rates (IMU.I.pqr) expressed as Euler-angle "
        "derivatives (roll/pitch/yaw rates) using the current Euler "
        "angles from INS_2.qn2b.",
        "rad/s", "Euler",
        { { DID_IMU, "I.pqr" }, { DID_INS_2, "qn2b" } },
        {},
        &evalBodyRateToEulerRate,
    });

    // ---- Heading --------------------------------------------------------
    addEntry(entries_, order_, Derivation{
        "heading_from_velocity",
        "atan2(vEast, vNorth) using NED velocity rotated from "
        "INS_2.uvw via INS_2.qn2b.",
        "rad", "NED",
        { { DID_INS_2, "uvw" }, { DID_INS_2, "qn2b" } },
        { { "wrap", "signed", "signed → [-pi, pi]; positive → [0, 2pi]" } },
        &evalHeadingFromVelocity,
    });
}

const DerivationRegistry& DerivationRegistry::instance() {
    static const DerivationRegistry r;
    return r;
}

std::optional<const Derivation*> DerivationRegistry::get(std::string_view name) const {
    auto it = entries_.find(std::string(name));
    if (it == entries_.end()) return std::nullopt;
    return std::optional<const Derivation*>{ &it->second };
}

std::vector<const Derivation*> DerivationRegistry::all() const {
    return order_;
}

} // namespace derivations
} // namespace inertial_sense
