# SDK error handling

The SDK uses two error conventions, deliberately. New API uses
`ISExpected<T>`; legacy API keeps what it has. The cutover is per-API,
not a flag day.

## New public API — `ISExpected<T>`

Anything new that can fail returns
`inertial_sense::ISExpected<T>` (a `tl::expected<T, ISError>` alias):

```cpp
#include "ISError.h"

inertial_sense::ISExpected<size_t> ISLogReader::byteOffsetForSample(
    inertial_sense::ISDeviceId did, size_t sample) const;

auto r = reader.byteOffsetForSample(did, sample);
if (!r) {                       // failure path
    log.warn("offset lookup failed: {}", r.error().message);
    return inertial_sense::fail(r.error().code, "wrapping in caller context");
}
size_t offset = *r;             // success path
```

The failure type is a small POD:

```cpp
enum class ISErrorCode : uint16_t {
    Ok, InvalidArgument, NotFound, PermissionDenied,
    Io, Corrupted, Truncated, Unsupported, Internal,
};
struct ISError {
    ISErrorCode code;
    std::string message;        // short canonical text; constructed only on failure
};
```

**Conventions:**

- New stories adding API to `ISLogReader`, `ISLogWriter`,
  `ISTimeResolver`, and the built-in derivation catalog return
  `ISExpected<T>` whenever the operation can fail.
- Construct `ISError::message` on the failure path only. Keep it
  short and canonical (`"truncated tail at offset 0x..."`); UI-side
  formatting is the consumer's job, not the SDK's.
- Use the `inertial_sense::fail(...)` helper for failure
  construction — clearer at the call site than a manual
  `tl::unexpected<ISError>{...}`.
- Callers must check `.has_value()` (or use `.value_or(...)`); do
  not rely on exception-style unwrapping. `tl::expected::value()`
  throws `bad_expected_access` if the result is an error — treat
  that as a programmer bug, not a control-flow path.
- New error codes get added to `ISErrorCode` only when a downstream
  story discovers a genuinely distinct failure mode. Don't
  pre-invent codes; the enum is small on purpose.

## Why not `std::expected`?

D0020 locks the SDK at C++17 for ABI compatibility with existing
consumers. `std::expected` requires C++23. `tl::expected` (Sy Brand,
CC0, header-only) is API-compatible with the standard, so consumers
compiled at C++23 can `using std::expected;` if they prefer — the
SDK's choice doesn't propagate into their code shape.

## Why not exceptions?

The SDK has a long history of bool / out-param error handling
because it ships into embedded and bare-metal contexts (firmware,
RTOS, no-RTTI builds) where exceptions are unavailable or expensive.
`tl::expected` has zero runtime overhead on the success path and a
small POD allocation on the failure path — palatable everywhere.

## Legacy API stays as-is

`cDeviceLog`, `cISLogger`, `cInertialSenseDisplay`, the parser
families, and every other pre-D-10 class **keep their existing
signatures**: bool returns, out-params, occasional exceptions, the
`isLogger.eClose` enums, etc. Refactoring them under D-10 invites
unrelated regressions and breaks consumer codebases for no
proportionate gain.

If you're modifying a legacy method's body, you're free to use
`ISExpected<T>` *internally*, but the public signature stays. If the
legacy API genuinely needs to expose a richer failure mode, file a
separate refactor story — don't piggyback.

## Cutover ledger (forward-looking)

Stories that adopt `ISExpected<T>` as part of their delivery:

| Story | What | Status |
|---|---|---|
| D-02 / SN-7893 | `ISLogReader` core | pending |
| D-07 / SN-7897 | `ISTimeResolver` | pending |
| D-08 / SN-7898 | `ISLogWriter` | pending |
| D-09 / SN-7899 | Built-in derivation catalog | pending |

Anything not on this list keeps the legacy convention.
