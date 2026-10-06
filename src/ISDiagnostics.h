/**
 * @file ISDiagnostics.h
 * @brief Structured diagnostics an application can act on — audit finding B3.
 *
 * Before this, everything the reader learned while opening a log dead-ended at
 * `ISLogReader`: `warnings()` is a `std::vector<std::string>` with no accessor on `ISDeviceLog`,
 * `ISLog` or Logalyzer's adapter, and `AnchorAnalysis::anomalies` is the same shape one level
 * down. So the stall detector that found a customer's frozen clock, the sidecar-rebuild reasons,
 * and the anchor-durability regressions were all unreachable from the application that needed to
 * show them.
 *
 * Strings were also the wrong shape for what Kyle asked for next: an orphaned `.idx` should be
 * reported *and offered for deletion*, which needs a machine-readable kind and a path, not prose
 * a UI would have to parse.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace inertial_sense {

/** @brief How much the user needs to care. */
enum class ISDiagSeverity : uint8_t {
    Info = 0,   ///< Worth showing if asked; nothing is wrong.
    Notice,     ///< The data was usable but something was reconstructed, adopted or degraded.
    Warning,    ///< A real problem that did not stop the open, and that changes what the data means.
};

/**
 * @brief What kind of thing happened. Stable, machine-readable; the UI keys behaviour off this.
 *
 * Deliberately coarse. A kind earns its own enumerator when an application would *do* something
 * different about it — offer a deletion, colour a timeline band, refuse to plot — not merely
 * when the wording differs.
 */
enum class ISDiagKind : uint8_t {
    Other = 0,

    /// A `.idx` with no segment beside it. It indexes nothing reachable and can never be
    /// upgraded, because an upgrade needs the segment to scan. `path` is the sidecar;
    /// `remedy` says it is safe to delete.
    OrphanedSidecar,

    /// The sidecar was missing, stale, corrupt or legacy, so the index was rebuilt from the
    /// segment. `message` carries the reason.
    SidecarRebuilt,

    /// A legacy sidecar's observed receipt times were carried into the new one, or declined.
    SidecarUpgraded,

    /// The rebuilt sidecar could not be written back (read-only media). Reading still works.
    SidecarNotPersisted,

    /// The segment ends mid-packet.
    SegmentTruncated,

    /// A DID kept emitting records while its clock stood still. The records were re-timed from
    /// their neighbours; the run is worth marking rather than drawing through.
    StalledClock,

    /// This segment's anchor is less durable than its predecessor's, or absent entirely, so its
    /// placement on the timeline is weaker than the rest of the log's.
    WeakAnchor,
};

/** @return  Stable lower-case identifier for @p k, for logs and for a UI's string table. */
const char* isDiagKindName(ISDiagKind k) noexcept;

/**
 * @brief One thing worth telling the user about a log that was just opened.
 *
 * `path` is always the file the diagnostic is *about* — the segment for most kinds, the sidecar
 * itself for @ref ISDiagKind::OrphanedSidecar — so a UI can offer an action without re-deriving
 * which file was meant.
 */
struct ISDiagnostic {
    ISDiagSeverity        severity = ISDiagSeverity::Info;
    ISDiagKind            kind     = ISDiagKind::Other;
    std::filesystem::path path;      ///< The file this is about. May be empty for log-level notes.
    std::string           message;   ///< What happened, in one sentence, already user-readable.
    std::string           remedy;    ///< What can be done about it, or empty when nothing need be.

    /** @return  True when an application should surface this unprompted. */
    bool needsAttention() const noexcept { return severity >= ISDiagSeverity::Notice; }
};

/**
 * @brief Optional sink invoked as diagnostics are produced, for progress-time reporting.
 *
 * The accessors on `ISLogReader` / `ISDeviceLog` / `ISLog` are enough when an application opens a
 * log and then asks what happened. A sink is for the case where it wants to know *during* a long
 * open — the same role `ISDeviceLog::setOnSegmentBoundary` plays for iteration.
 */
using ISDiagSink = void (*)(const ISDiagnostic&, void* userData);

} // namespace inertial_sense
