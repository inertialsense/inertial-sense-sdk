/**
 * @file DIDTraits.h
 * @brief Compile-time DID → payload-struct mapping for the templated
 *        sugar layer over `ISLogReader` (D-03 / SN-7894).
 *
 * D0021 split the SDK reader into two layers: a type-erased core
 * (D-02 — `ISLogReader::records(did)` + `ISRecordView`) and a
 * compile-time-typed sugar on top (this header). Direct-code consumers
 * (cltool, ROS, customer integrations) reach for the sugar when the
 * DID is known at build time and they want a `const T&` rather than a
 * cast. The plot-template engine (D-40+) uses the type-erased core
 * because it picks DIDs at runtime from YAML.
 *
 * **Coverage:** v1 hand-codes specializations for the ~15 most
 * commonly-consumed DIDs. The full ~300-entry set will land via an
 * auto-generated header (CMake-time Python script parsing
 * `data_sets.c`'s mapping arrays) — tracked as a follow-up. v1's
 * partial coverage is *opt-in* — calling `records<DID>()` for an
 * unspecialized DID is a clean compile error, not silent breakage,
 * because the primary template is forward-declared without a default.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "data_sets.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace inertial_sense {

/// Compile-time integer alias used as the non-type template parameter
/// for `DIDTraits` and `ISLogReader::records<DID>()`. `eDataIDs` is the
/// underlying `data_sets.h` enum, but we accept any integer-convertible
/// value to keep call sites ergonomic.
using did_t = uint32_t;

/**
 * @brief Primary template — intentionally undefined.
 *
 * A DID with no `DIDTraits` specialization triggers an "incomplete
 * type" compile error at the call site, which is the exact UX we want
 * — typo'd or unsupported DIDs fail loudly at compile time rather
 * than at runtime via a string lookup.
 */
template <did_t DID> struct DIDTraits;

// ---------------------------------------------------------------------------
// Detection helper — `has_traits_v<DID>` is true iff a specialization
// exists. Used by SFINAE / `static_assert` in the templated sugar.
// ---------------------------------------------------------------------------
namespace detail {
    template <did_t DID, class = void>
    struct has_traits : std::false_type {};

    template <did_t DID>
    struct has_traits<DID, std::void_t<typename DIDTraits<DID>::type>>
        : std::true_type {};
} // namespace detail

template <did_t DID>
inline constexpr bool has_traits_v = detail::has_traits<DID>::value;

// ---------------------------------------------------------------------------
// Specializations — v1 hand-coded set.
//
// The DOC pattern: each block is one DID. The macro keeps boilerplate
// lean while leaving the actual specialization syntax visible (no
// magic indirection). When the auto-gen lands it'll emit the same
// shape.
// ---------------------------------------------------------------------------

#define IS_DEFINE_DID_TRAITS(DID_VAL, payload_struct, did_name) \
    template <> struct DIDTraits<static_cast<did_t>(DID_VAL)> {                 \
        using type = payload_struct;                                            \
        static constexpr const char* name = did_name;                           \
        static constexpr std::size_t expected_size = sizeof(payload_struct);    \
    }

// Core navigation outputs.
IS_DEFINE_DID_TRAITS(DID_INS_1,             ins_1_t,        "DID_INS_1");
IS_DEFINE_DID_TRAITS(DID_INS_2,             ins_2_t,        "DID_INS_2");
IS_DEFINE_DID_TRAITS(DID_INS_3,             ins_3_t,        "DID_INS_3");
IS_DEFINE_DID_TRAITS(DID_INS_4,             ins_4_t,        "DID_INS_4");

// IMU.
IS_DEFINE_DID_TRAITS(DID_IMU,               imu_t,          "DID_IMU");
IS_DEFINE_DID_TRAITS(DID_PIMU,              pimu_t,         "DID_PIMU");

// GPS position + velocity (1- and 2-receiver variants).
IS_DEFINE_DID_TRAITS(DID_GPS1_POS,          gps_pos_t,      "DID_GPS1_POS");
IS_DEFINE_DID_TRAITS(DID_GPS2_POS,          gps_pos_t,      "DID_GPS2_POS");
IS_DEFINE_DID_TRAITS(DID_GPS1_VEL,          gps_vel_t,      "DID_GPS1_VEL");
IS_DEFINE_DID_TRAITS(DID_GPS2_VEL,          gps_vel_t,      "DID_GPS2_VEL");

// Standalone sensor outputs.
IS_DEFINE_DID_TRAITS(DID_MAGNETOMETER,      magnetometer_t, "DID_MAGNETOMETER");
IS_DEFINE_DID_TRAITS(DID_BAROMETER,         barometer_t,    "DID_BAROMETER");

// System metadata + identification.
IS_DEFINE_DID_TRAITS(DID_DEV_INFO,          dev_info_t,     "DID_DEV_INFO");
IS_DEFINE_DID_TRAITS(DID_SYS_PARAMS,        sys_params_t,   "DID_SYS_PARAMS");

#undef IS_DEFINE_DID_TRAITS

} // namespace inertial_sense
