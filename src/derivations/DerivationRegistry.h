/**
 * @file DerivationRegistry.h
 * @brief Built-in derivation catalog — D-09 / SN-7899.
 *
 * D0037 / D0042: a curated catalog of named, tested derivation
 * functions that take one or more SDK record streams and produce a
 * value-semantic stream of derived samples. Templates (D-40+) and
 * the derived-series dispatch (D-44) reference these entries by
 * name; the exprtk formula path (D-43) coexists as an alternative
 * for ad-hoc math.
 *
 * **Pure C++17.** No Qt, no exceptions. Failures travel via
 * `ISExpected<T>`.
 *
 * **Stable names.** Each entry's `name` is part of the public surface
 * — once shipped, plot templates depend on it. Don't rename without
 * a deprecation cycle.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISError.h"
#include "ISLogReader.h"
#include "ISTimeStamp.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace inertial_sense {
namespace derivations {

/**
 * @brief Declares one input-data dependency a derivation needs.
 *
 * A derivation that requires `DID_INS_2.qn2b` declares
 * `InputSpec{ DID_INS_2, "qn2b" }`. The `fieldPath` is documentation
 * only — actual field access happens inside the evaluator's body via
 * the underlying payload struct.
 *
 * `optional` marks inputs that may be replaced or supplied via
 * parameters (e.g. `lla_to_ned@user_origin`'s LLA reference).
 */
struct InputSpec {
    uint32_t        did;
    std::string_view fieldPath;
    bool            optional = false;
};

/**
 * @brief A single output sample produced by a derivation.
 *
 * `values` is variable-length to support both scalar and multi-channel
 * derivations under one type:
 *   - 1 element for scalar (e.g. `velocity_magnitude`).
 *   - 3 elements for vector outputs (e.g. `quat_to_euler` produces
 *     `[roll, pitch, yaw]`).
 *
 * The `time` field carries the same `TimeStamp` provenance as the
 * underlying record(s) — `source = PayloadToW`, `confidence = Exact`
 * for raw inputs; `Resolved` / `Approximate` once D-07 lands and
 * derivations consume resolved inputs.
 */
struct DerivedSample {
    TimeStamp           time;
    std::vector<double> values;
};

/**
 * @brief A derivation's evaluation result.
 *
 * Per-channel naming + unit + frame travel with the data so the
 * UI (D-91) can label axes and the playback engine (D-60) can scrub
 * across derivations consistently.
 */
struct DerivationResult {
    /// One name per element of `DerivedSample::values`. For
    /// `quat_to_euler`: `["roll", "pitch", "yaw"]`. Single-channel
    /// outputs typically use the derivation's name itself.
    std::vector<std::string> channelNames;

    /// Output unit string — e.g. "rad", "m", "m/s". Echoes the
    /// `Derivation::outputUnit` advertised at registration time.
    std::string outputUnit;

    /// Output frame — e.g. "NED", "body", "" for frame-invariant
    /// scalars like magnitudes.
    std::string outputFrame;

    /// Samples in input arrival order (not necessarily timestamp-
    /// monotonic; D-07 will sort). Same semantics as the underlying
    /// reader's `allRecords()`.
    std::vector<DerivedSample> samples;
};

/**
 * @brief Documents a parameter the derivation accepts via
 *        `DerivationContext::setParam`.
 */
struct DerivationParameter {
    std::string_view name;
    std::string_view defaultValue;
    std::string_view description;
};

/**
 * @brief Per-evaluation inputs + parameter map.
 *
 * Holds a non-owning pointer to one source `ISLogReader`. Multi-
 * device contexts will follow when D-32 lands the series cache; for
 * v1, one segment per evaluation is sufficient.
 */
class DerivationContext {
public:
    /// Constructs a context bound to one source reader. The reader's
    /// lifetime must outlive the evaluation.
    explicit DerivationContext(const ISLogReader& reader) noexcept
        : reader_(&reader) {}

    /// @return  Bound source reader. Aliased pointer, do not store
    ///          past the evaluation.
    const ISLogReader& reader() const noexcept { return *reader_; }

    /**
     * Sets a string-valued parameter. Derivations that declare
     * parameters (`lla_to_ned@user_origin`'s `origin_lla`, etc.)
     * read the value via `param()`. Unknown names are accepted
     * (forward-compatibility for templates that pass extras).
     *
     * @param name   Parameter name (must match the derivation's
     *               declared parameter name).
     * @param value  String form; derivation parses it.
     */
    void setParam(std::string_view name, std::string value) {
        params_[std::string(name)] = std::move(value);
    }

    /// @return  Value previously set via `setParam`, or `nullopt` if
    ///          the name wasn't set.
    std::optional<std::string_view> param(std::string_view name) const {
        auto it = params_.find(std::string(name));
        if (it == params_.end()) return std::nullopt;
        return std::optional<std::string_view>{ it->second };
    }

private:
    const ISLogReader*               reader_;
    std::map<std::string, std::string> params_;
};

/// Function-pointer signature every derivation implements.
using EvaluateFn = ISExpected<DerivationResult> (*)(const DerivationContext&);

/**
 * @brief Static descriptor for one named derivation.
 *
 * Registered once at SDK startup via the registry's internal init
 * list. Consumers retrieve by `DerivationRegistry::get(name)` and
 * call `evaluate(ctx)`.
 */
struct Derivation {
    std::string_view                 name;
    std::string_view                 description;
    std::string_view                 outputUnit;
    std::string_view                 outputFrame;
    std::vector<InputSpec>           inputs;
    std::vector<DerivationParameter> parameters;
    EvaluateFn                       evaluate;
};

/**
 * @brief Registry of all built-in derivations.
 *
 * Singleton. Construction populates the catalog at first call.
 * Lookups are O(log N); enumeration is O(N).
 *
 * @code
 *   const auto& reg = derivations::DerivationRegistry::instance();
 *   if (auto d = reg.get("quat_to_euler")) {
 *       derivations::DerivationContext ctx{ reader };
 *       auto r = (*d)->evaluate(ctx);
 *       // ...
 *   }
 * @endcode
 */
class DerivationRegistry {
public:
    /// @return  Singleton instance. Thread-safe (Meyers' singleton).
    static const DerivationRegistry& instance();

    /// @return  Pointer to the entry with the given name, or
    ///          `nullopt` if absent.
    std::optional<const Derivation*> get(std::string_view name) const;

    /// @return  All registered entries, in registration order.
    std::vector<const Derivation*> all() const;

    /// @return  Count of registered entries.
    std::size_t size() const noexcept { return entries_.size(); }

    DerivationRegistry(const DerivationRegistry&)            = delete;
    DerivationRegistry& operator=(const DerivationRegistry&) = delete;

private:
    DerivationRegistry();

    /// Backing storage. `std::string` keys (not `string_view`) so
    /// the map owns its keys and string-view lookups stay safe even
    /// when callers pass through temporaries.
    std::map<std::string, Derivation> entries_;

    /// Insertion-order list for `all()` enumerations.
    std::vector<const Derivation*>    order_;
};

// ---------------------------------------------------------------------------
// Built-in derivation evaluator declarations.
//
// One per entry in the v1 catalog. Implementations live in the
// matching .cpp files; the registry knits them into the catalog at
// init.
// ---------------------------------------------------------------------------

ISExpected<DerivationResult> evalQuatToEuler(const DerivationContext&);
ISExpected<DerivationResult> evalLlaToNedFirstFix(const DerivationContext&);
ISExpected<DerivationResult> evalLlaToNedUserOrigin(const DerivationContext&);
ISExpected<DerivationResult> evalVelocityMagnitude(const DerivationContext&);
ISExpected<DerivationResult> evalAccelMagnitude(const DerivationContext&);
ISExpected<DerivationResult> evalRateMagnitude(const DerivationContext&);
ISExpected<DerivationResult> evalBodyAccelToNed(const DerivationContext&);
ISExpected<DerivationResult> evalBodyRateToEulerRate(const DerivationContext&);
ISExpected<DerivationResult> evalHeadingFromVelocity(const DerivationContext&);
ISExpected<DerivationResult> evalHeadingFromQuat(const DerivationContext&);

} // namespace derivations
} // namespace inertial_sense
