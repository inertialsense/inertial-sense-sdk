/**
 * @file ISLogWriter.h
 * @brief Narrow-scope writer for SDK 3.0 — emits `.raw` + v2 `.idx` pairs.
 *
 * D-08 / SN-7898 / D0035 / D0049: the output side of Logalyzer's
 * Bake-Trim (D-72). Takes records from an `ISLogReader` (or any
 * source that produces `ISRecordView`s) and writes a fresh `.raw`
 * segment whose bytes are bit-identical to the inputs, alongside a
 * matching v2 `.idx` sidecar. **Bytes are never re-packetized.**
 *
 * **No legacy formats.** RAW + sidecar only. CSV / KML / JSON / v1
 * `.idx` are out of scope per D0049 — `cISLogger` keeps owning those
 * paths for now.
 *
 * **Atomic semantics.** All writes go to `<path>.tmp` files. On
 * `finalize()` the tempfiles are renamed in place; if the process
 * dies before `finalize()`, the destructor deletes the tempfiles
 * and a `.raw` does not appear at the final path. Mirrors the D-04
 * `.idx` write path so any reader-side handling (D-04 truncation
 * detection) applies symmetrically if the rename happens but the
 * caller crashes mid-stream before finalize.
 *
 * **Thread-safety.** A single `ISLogWriter` is **not** thread-safe —
 * `append` / `finalize` calls must be serialized externally. The
 * reader-side `const`-safe-concurrent-iteration guarantee does not
 * have an analog here; writers serialize a single output stream.
 *
 * **Move-only.** Owning two write sessions to the same tempfiles
 * would corrupt them; copy ops are deleted, move ops are noexcept.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISError.h"
#include "ISLogIndex.h"
#include "ISRecordView.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace inertial_sense {

class ISLogWriter {
public:
    /**
     * @brief Settings bag for `create()`.
     *
     * Default-constructed values match the most-conservative behavior
     * (fail if output exists, host uptime ms timestamps). Callers
     * generally only override `overwrite`, `tsUnits`, and `tsSource`
     * when they have a reason to.
     */
    struct Options {
        /// Output `.raw` path. Required; the matching `.idx` is
        /// derived by replacing the `.raw` suffix with `.idx`. If the
        /// suffix isn't `.raw`, `.idx` is appended.
        std::filesystem::path rawOutPath;

        /// Required device id stamped into the header and validated
        /// against incoming records. Set to 0 to skip validation.
        uint64_t sourceDeviceId = 0;

        /// Optional caller note describing where this output came
        /// from. Held in memory for caller introspection (`lineageNote()`)
        /// but **not persisted** in the v2 format — the bytes the
        /// reader sees are bit-identical to the inputs, so any
        /// out-of-band lineage tracking belongs in a higher-level
        /// container (workspace `.ilw`, etc.).
        std::optional<std::string> lineageNote = std::nullopt;

        /// Replace an existing output. Default `false` — `create()`
        /// returns `InvalidArgument` if the target `.raw` or `.idx`
        /// already exists. With `true`, the tempfile rename overwrites.
        bool overwrite = false;

        /// Header `ts_units` (informational; doesn't transform the
        /// record timestamps written to the `.idx`).
        idx::TimestampUnits tsUnits = idx::TimestampUnits::HostUptimeMs;

        /// Header `ts_source` (informational).
        idx::HeaderTimeSource tsSource = idx::HeaderTimeSource::PayloadToW;
    };

    /**
     * @brief Open a writer.
     *
     * Creates `<rawOutPath>.tmp` and `<idxOutPath>.tmp`, writes a
     * placeholder `.idx` header (FINALIZED unset), and returns the
     * writer ready to accept `append()` calls.
     *
     * @param opts  Settings; see `Options`.
     * @return      Open writer, or an `ISError`:
     *              - `InvalidArgument` if `rawOutPath` is empty,
     *                already exists (and `!overwrite`), or its parent
     *                directory doesn't exist.
     *              - `Io` on tempfile open / initial header write.
     */
    static ISExpected<ISLogWriter> create(Options opts);

    /**
     * @brief Convenience overload taking just the path + device id.
     *
     * Equivalent to `create(Options{ rawOutPath, sourceDeviceId,
     * lineageNote })`.
     */
    static ISExpected<ISLogWriter> create(
        std::filesystem::path rawOutPath,
        uint64_t sourceDeviceId,
        std::optional<std::string> lineageNote = std::nullopt);

    ~ISLogWriter();

    ISLogWriter(const ISLogWriter&)            = delete;
    ISLogWriter& operator=(const ISLogWriter&) = delete;

    /**
     * Move-construct. Transfers the tempfile handles + accumulated
     * stats; the source is left in a "moved-from" state where every
     * method returns `InvalidArgument`.
     *
     * @param other  Source writer; safe to destroy.
     */
    ISLogWriter(ISLogWriter&&) noexcept;

    /**
     * Move-assign. If `*this` was an active session, its tempfiles
     * are deleted (same semantics as the destructor on a non-finalized
     * writer) before adopting `other`'s state.
     */
    ISLogWriter& operator=(ISLogWriter&&) noexcept;

    /**
     * @brief Append one record.
     *
     * Writes the record's bytes verbatim into the output `.raw` and
     * adds a v2 `.idx` record at the resulting offset. The record's
     * `flags` (including `HAS_TOW`) are preserved from the source
     * view; no re-derivation happens at this layer.
     *
     * @param view  Source record. Must have `bytes().first != nullptr`
     *              and `bytes().second > 0`. If `sourceDeviceId` was
     *              non-zero at `create()` time, `view.timestamp().deviceId`
     *              must match (first mismatch fails fast with
     *              `InvalidArgument`).
     * @return      `Ok` on success, otherwise:
     *              - `InvalidArgument` — empty view, device-id
     *                mismatch, writer not initialized, or already
     *                finalized.
     *              - `Io` — short write on `.raw` or `.idx`.
     */
    ISExpected<void> append(const ISRecordView& view);

    /**
     * @brief Filter a range and append the matches.
     *
     * Equivalent to a manual `for (auto v : range) if (pred(v))
     * append(v);` loop, with early-exit on the first append failure.
     *
     * @tparam RecordRange  Anything iterable yielding `ISRecordView`.
     * @tparam Predicate    Callable `bool(const ISRecordView&)`.
     * @param range         Source range.
     * @param pred          Filter predicate.
     * @return              Count of records actually written, or the
     *                      first `ISError` raised by `append()`.
     */
    template <class RecordRange, class Predicate>
    ISExpected<std::size_t> appendFiltered(RecordRange&& range, Predicate&& pred) {
        std::size_t written = 0;
        for (const ISRecordView& v : range) {
            if (!pred(v)) continue;
            auto r = append(v);
            if (!r) return tl::unexpected<ISError>{ r.error() };
            ++written;
        }
        return written;
    }

    /**
     * @brief Close the session and atomically install the output.
     *
     * Patches the `.idx` header with the final record count, first /
     * last timestamps, and sync-point count; sets `FINALIZED`; flushes
     * and closes both files; renames `.tmp` → final. Idempotent in
     * the sense that calling it twice on the same writer returns
     * `InvalidArgument` on the second call (no further side effects).
     *
     * @return  `Ok` on success.
     *          - `Io` if any flush, seek, write, or rename failed.
     *          - `InvalidArgument` if the writer was uninitialized or
     *            already finalized.
     */
    ISExpected<void> finalize();

    /**
     * @return  Number of records successfully appended so far. Survives
     *          across `finalize()`.
     */
    std::size_t recordCount() const noexcept { return recordCount_; }

    /**
     * @return  First record timestamp written, or 0 if no records
     *          have been appended.
     */
    uint64_t firstTimestamp() const noexcept { return firstTimestamp_; }

    /**
     * @return  Last record timestamp written, or 0 if no records
     *          have been appended.
     */
    uint64_t lastTimestamp() const noexcept { return lastTimestamp_; }

    /**
     * @return  Count of records whose `HAS_TOW` flag was set —
     *          potential sync points for D-07.
     */
    uint32_t syncPointCount() const noexcept { return syncPointCount_; }

    /**
     * @return  `true` once `finalize()` succeeded; otherwise `false`.
     */
    bool isFinalized() const noexcept { return finalized_; }

    /**
     * @return  Caller-supplied lineage note from `Options`, if any.
     *          Held in memory only — not written to disk.
     */
    const std::optional<std::string>& lineageNote() const noexcept {
        return lineageNote_;
    }

    /**
     * @return  Final `.raw` path (the location after `finalize()`
     *          renames the tempfile).
     */
    const std::filesystem::path& rawPath() const noexcept { return rawOutPath_; }

    /**
     * @return  Final `.idx` path.
     */
    const std::filesystem::path& idxPath() const noexcept { return idxOutPath_; }

private:
    ISLogWriter() = default;

    /**
     * Closes any open streams and removes the tempfiles. Used by the
     * destructor and by `operator=(&&)` when the moved-into writer
     * was an active session. Idempotent.
     */
    void cleanupTempfiles() noexcept;

    /**
     * Patches the on-disk `.idx` header with the final stats and
     * `FINALIZED` flag. Called from `finalize()`; assumes the idx
     * stream is open.
     */
    ISExpected<void> writeFinalHeader();

    // Final destination paths.
    std::filesystem::path rawOutPath_;
    std::filesystem::path idxOutPath_;

    // Tempfiles staged for atomic rename on finalize.
    std::filesystem::path rawTmpPath_;
    std::filesystem::path idxTmpPath_;

    std::ofstream rawStream_;
    std::ofstream idxStream_;

    // Header template — `producer_version`, `ts_units`, `ts_source`
    // are seeded at create time and stays put. `total_records`,
    // `first_timestamp_ms`, `last_timestamp_ms`, `sync_point_count`,
    // `flags` are patched at finalize().
    idx::is_log_idx_header_t header_{};

    uint64_t                   sourceDeviceId_  = 0;
    std::optional<std::string> lineageNote_;
    std::size_t                recordCount_     = 0;
    uint64_t                   firstTimestamp_  = 0;
    uint64_t                   lastTimestamp_   = 0;
    uint32_t                   syncPointCount_  = 0;
    uint64_t                   rawOffset_       = 0;
    bool                       initialized_     = false;
    bool                       finalized_       = false;
};

} // namespace inertial_sense
