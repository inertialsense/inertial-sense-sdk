/**
 * @file ISDiagnostics.cpp
 * @brief See ISDiagnostics.h. Audit B3.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDiagnostics.h"

namespace inertial_sense {

const char* isDiagKindName(ISDiagKind k) noexcept {
    switch (k) {
        case ISDiagKind::Other:               return "other";
        case ISDiagKind::OrphanedSidecar:     return "orphaned-sidecar";
        case ISDiagKind::SidecarRebuilt:      return "sidecar-rebuilt";
        case ISDiagKind::SidecarUpgraded:     return "sidecar-upgraded";
        case ISDiagKind::SidecarNotPersisted: return "sidecar-not-persisted";
        case ISDiagKind::SegmentTruncated:    return "segment-truncated";
        case ISDiagKind::StalledClock:        return "stalled-clock";
        case ISDiagKind::WeakAnchor:          return "weak-anchor";
    }
    return "?";
}

} // namespace inertial_sense
