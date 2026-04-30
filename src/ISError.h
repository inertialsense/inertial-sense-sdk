/**
 * @file ISError.h
 * @brief SDK-wide error type used by `tl::expected` returning APIs.
 *
 * D-10 / SN-7881 / D0049: new SDK public API (ISLogReader, ISLogWriter,
 * ISTimeResolver, built-in derivations) returns `ISExpected<T>` so
 * failure paths are checked, composable, and self-documenting without
 * exceptions or the C++23 `<expected>` header (the SDK's C++17
 * baseline can't use it).
 *
 * Existing `cDeviceLog` / `cISLogger` APIs keep their bool /
 * out-param / exception conventions — see docs/error-handling.md.
 */

#pragma once

#include "tl/expected.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace inertial_sense {

// Initial error-code set. More entries added as downstream stories
// (D-02 ISLogReader, D-07 ISTimeResolver, D-08 ISLogWriter) discover
// genuinely distinct failure modes — don't pre-invent.
enum class ISErrorCode : uint16_t {
    Ok = 0,             // unused as an error; present for range hygiene
    InvalidArgument,
    NotFound,
    PermissionDenied,
    Io,
    Corrupted,
    Truncated,
    Unsupported,
    Internal,
};

// Carries the failure code and a short canonical message. Construct
// the message on the failure path only (the success path returns
// `tl::expected<T, ISError>::value()` and never builds the string).
struct ISError {
    ISErrorCode code;
    std::string message;
};

// Template alias — ergonomic spelling for the SDK's canonical
// fallible-return type. Header-only; no ABI considerations.
template <class T>
using ISExpected = tl::expected<T, ISError>;

// Factory helper for failure construction:
//   return inertial_sense::fail(ISErrorCode::Truncated, "tail at 0x...");
inline tl::unexpected<ISError> fail(ISErrorCode code, std::string message) {
    return tl::unexpected<ISError>{ ISError{ code, std::move(message) } };
}

} // namespace inertial_sense
