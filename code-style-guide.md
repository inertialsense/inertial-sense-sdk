# Inertial Sense SDK — C++ Code Style Guide

> Scope: C/C++ source in the Inertial Sense SDK and the projects that embed it
> (EvalTool, cltool, firmware). This guide is **normative**. The key words **MUST**,
> **MUST NOT**, **SHOULD**, **SHOULD NOT**, and **MAY** are used in the RFC 2119 sense.
>
> Mechanical formatting (indent, braces, column width, include grouping) is owned by
> `.clang-format` and enforced in CI — this guide does not restate it beyond a summary.
> When this guide and `.clang-format` disagree, `.clang-format` wins for formatting.

---

## 1. Formatting (owned by `.clang-format`)

The SDK's `.clang-format` is the source of truth. For reference, it currently sets:

- `BasedOnStyle: Google`
- `IndentWidth: 4`, `UseTab: Never` — **4 spaces, never tabs**
- `ColumnLimit: 200`
- `IncludeBlocks: Regroup` — includes are grouped and sorted automatically
- `InsertNewlineAtEOF: true`

You **MUST** run `clang-format` (or the IDE equivalent) before committing. Do not
hand-format against the tool.

---

## 2. Documentation — Doxygen

Reference: <https://www.doxygen.nl/manual/docblocks.html>. Doxygen accepts several
comment styles; this project standardizes on **two** of them and forbids the rest so
that documentation is consistent and greppable.

### 2.1 The two allowed forms

| Purpose | Form | Position |
|---|---|---|
| Document the item that **follows** (file, class, struct, enum, function, standalone) | `/** … */` block | **before** the declaration |
| Document the item to its **left** (a member variable, enumerator, or parameter) | `//!< …` | **trailing**, same line |

That is the whole rule: **preceding documentation uses `/** */`; trailing
member/enumerator/parameter documentation uses `//!<`.**

### 2.2 Forbidden forms

You **MUST NOT** use:

- `///` or `///<` — the triple-slash forms (this is the most common mistake).
- `//!` as a *preceding* single-line block — use a `/** */` block instead.
- `/*! */` Qt-style blocks — use `/** */` (JavaDoc style) for consistency.

> Why `//!<` and not `///<` for trailing docs? Both are valid Doxygen, but the project
> standard pairs the JavaDoc preceding block (`/** */`) with the `//!<` trailing marker
> so the two roles are visually distinct and a simple search for `///` reliably finds
> style violations.

### 2.3 Preceding blocks — `/** */`

Every line begins with ` * `. Start with a one-line brief; a blank line separates the
brief from the detailed description (or use `@brief` explicitly).

```cpp
/**
 * Enable or disable a known relay host.
 *
 * Disabled hosts remain listed by getRelayHosts() but contribute no ports. Enabling
 * spawns the SSE worker; disabling tears it down and evicts the host's ports.
 *
 * @param url      canonical or raw relay URL (normalized internally)
 * @param enabled  true to start contributing ports, false to stop
 */
void setRelayHostEnabled(const std::string& url, bool enabled);
```

Single-line briefs are fine for simple members/functions:

```cpp
/** Returns the number of ports currently managed. */
size_t getPortCount() const;
```

### 2.4 Trailing member / enumerator / parameter docs — `//!<`

The `<` tells Doxygen the comment documents the item **to its left**. Per the Doxygen
manual these trailing blocks document **members and parameters only** — never the
struct/enum/class itself (that gets a preceding `/** */`).

```cpp
/** Active transport in use for a relay host. */
enum class RelayFeedType : uint8_t {
    Auto    = 0,  //!< negotiating; not yet connected via any transport
    SSE     = 1,  //!< push-based /api/events/devices stream is connected
    Polling = 2,  //!< polling fallback (SSE unavailable or failed)
};

struct RelayHostStatus {
    std::string  url;            //!< canonical "http://host:port" (no path)
    bool         enabled;        //!< true iff this host contributes ports
    size_t       deviceCount;    //!< number of devices reported by this host
};
```

If a trailing description needs more than one line, promote it to a preceding
`/** */` block above the member rather than wrapping a `//!<`.

Parameter docs **MAY** use the trailing form inline in a signature, but the `@param`
form in the function's preceding block (2.3) is preferred for readability.

### 2.5 File headers

Every translation unit and header **MUST** open with a file block. `@file` and
`@brief` document the file itself, so they live in a preceding `/** */` block:

```cpp
/**
 * @file RelayPortFactory.h
 * @brief Discovers IS device ports through remote HTTP-based relay hosts.
 *
 * @author <name> on <date>
 * @copyright Copyright (c) <year> Inertial Sense, Inc. All rights reserved.
 */
```

### 2.6 Common commands

Use `@`-prefixed commands (not `\`). The vocabulary in regular use:

`@file` `@brief` `@param` `@param[in]` `@param[out]` `@param[in,out]` `@return`
`@retval` `@tparam` `@note` `@warning` `@see` `@code`/`@endcode` `@deprecated` `@todo`.

### 2.7 What to document

- Public API (anything in a header that callers use): **MUST** be documented.
- Non-obvious private functions, invariants, ownership, threading/locking
  assumptions, and units: **SHOULD** be documented.
- When you change a function's signature, you **MUST** update its doc block in the
  same change (params added/removed/renamed, return semantics, ownership).
- Trivial getters/setters and self-evident code: a doc comment is optional; do not
  add noise that merely restates the signature.

---

## 3. Naming

These reflect the prevailing SDK conventions. Match the **surrounding file** first; use
the following for new files and new symbols.

| Kind | Convention | Example |
|---|---|---|
| Type (class/struct/enum) | `PascalCase` | `RelayPortFactory`, `RelayHostStatus` |
| `enum class` enumerators | `PascalCase` | `RelayFeedType::Polling` |
| Function / method | `camelCase` | `discoverPorts()`, `bindPort()` |
| Local variable / parameter | `camelCase` | `portUrl`, `relayHost` |
| Private data member | `camelCase` with trailing `_` | `relayHosts_`, `pollInterval_` |
| Plain-struct public field | `camelCase`, no underscore | `RelayHostStatus::deviceCount` |
| Constant / `constexpr` / macro | `UPPER_SNAKE_CASE` | `DEFAULT_HTTP_PORT`, `MDNS_QUERY_INTERVAL_MS` |

Notes:
- The private-member trailing `_` is used by newer code (e.g. `RelayPortFactory`) but is
  **not** universal across the SDK. Do **not** mass-rename existing members; follow the
  file you are in, and prefer the trailing `_` in new code.
- **Do not** prefix interface/abstract types with `I` (no `IFoo`). Name the abstraction
  for what it is. Defer introducing abstractions until a second concrete case exists,
  and keep the class count low — prefer a singleton concrete factory over an interface
  hierarchy when there is one implementation.

---

## 4. Language & best practices (C++17)

**Const-correctness.** Mark methods `const` when they don't mutate observable state.
Pass read-only objects by `const&`. Prefer `const`/`constexpr` for values that don't change.

**References vs pointers.** Use a reference for a required, non-null argument; use a
pointer only when null is meaningful (optional/out param). Never return a raw owning pointer.

**Ownership & RAII.** Express ownership with `std::unique_ptr` (sole owner) or
`std::shared_ptr` (shared). Raw pointers/references are non-owning observers. Acquire
locks with `std::lock_guard`/`std::unique_lock`, never bare `lock()`/`unlock()`. Every
resource (socket, thread, file, mutex) **MUST** be released on every path, including
exceptions and early returns.

**`enum class`.** Prefer scoped enums over plain `enum`. Give them an explicit underlying
type when the width matters on the wire (`enum class … : uint8_t`).

**Modern keywords.** Use `nullptr` (not `NULL`/`0`), `override` on every overriding
method, `= default` / `= delete` for special members, `auto` where it aids readability.
Delete copy/move on non-copyable types (e.g. types owning a `std::thread`).

**Includes (IWYU).** Include what you use; don't lean on transitive includes. Keep the
public header's includes minimal — prefer forward declarations and push heavy includes
(`httplib.h`, `json.hpp`) into the `.cpp`. `IncludeBlocks: Regroup` will order them.

**Anonymous namespace.** Give internal (TU-local) helpers internal linkage via an
unnamed `namespace { … }` rather than `static` on each symbol.

**Error handling.** The SDK core avoids throwing across module boundaries. Prefer status
returns (`bool`, an out-param, or an enum/`std::optional`) for expected failures; reserve
exceptions for truly exceptional conditions and never let one escape a C ABI boundary or
a callback invoked by C code.

**Threading & locking.** Document which mutex guards what, and the lock order. Two rules
that have bitten this codebase specifically:
- **Do not hold a lock across a callback or blocking I/O.** Snapshot the data you need
  under the lock, release it, then invoke the callback / do the network/DNS work. Holding
  a factory mutex across `bindPort()`/`getaddrinfo()` froze the UI (SN-8175).
- **Never do network, DNS, or port I/O on the GUI thread.** Marshal it to the worker/IO
  thread (e.g. `QMetaObject::invokeMethod(obj, "slot", Qt::QueuedConnection)`).

**Magic numbers.** Name timeouts, intervals, retry counts, and buffer sizes as
`constexpr` (e.g. `OFFLINE_EVICT_MS`) rather than embedding literals at the call site.

---

## 5. References

- Doxygen — Documenting the code: <https://www.doxygen.nl/manual/docblocks.html>
- Doxygen — Special commands: <https://www.doxygen.nl/manual/commands.html>
- `.clang-format` in this directory (mechanical formatting, CI-enforced)
