/**
 * @file ISLogWriter.cpp
 * @brief Implementation of the narrow-scope SDK 3.0 writer.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISLogWriter.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <system_error>
#include <utility>

namespace inertial_sense {

namespace {

constexpr uint32_t kProducerVersion = 0x00030000u;  // SDK 3.0 marker

std::filesystem::path deriveIdxPath(const std::filesystem::path& raw) {
    if (raw.extension() == ".raw") {
        std::filesystem::path out = raw;
        out.replace_extension(".idx");
        return out;
    }
    std::filesystem::path out = raw;
    out += ".idx";
    return out;
}

std::filesystem::path withTmpSuffix(const std::filesystem::path& p) {
    std::filesystem::path out = p;
    out += ".tmp";
    return out;
}

} // namespace

// ----- Lifecycle -----------------------------------------------------------

ISExpected<ISLogWriter> ISLogWriter::create(Options opts) {
    if (opts.rawOutPath.empty()) {
        return fail(ISErrorCode::InvalidArgument,
                    "ISLogWriter::create: rawOutPath is empty");
    }

    const auto rawOut = opts.rawOutPath;
    const auto idxOut = deriveIdxPath(rawOut);

    if (!rawOut.parent_path().empty()) {
        std::error_code ec;
        if (!std::filesystem::is_directory(rawOut.parent_path(), ec)) {
            return fail(ISErrorCode::InvalidArgument,
                        "ISLogWriter::create: parent directory does not exist: "
                        + rawOut.parent_path().string());
        }
    }

    std::error_code ec;
    if (!opts.overwrite) {
        if (std::filesystem::exists(rawOut, ec)) {
            return fail(ISErrorCode::InvalidArgument,
                        "ISLogWriter::create: output already exists: "
                        + rawOut.string());
        }
        if (std::filesystem::exists(idxOut, ec)) {
            return fail(ISErrorCode::InvalidArgument,
                        "ISLogWriter::create: output already exists: "
                        + idxOut.string());
        }
    }

    const auto rawTmp = withTmpSuffix(rawOut);
    const auto idxTmp = withTmpSuffix(idxOut);

    // Best-effort cleanup of stale tempfiles from a previous crashed session
    // — we own these names by convention.
    std::filesystem::remove(rawTmp, ec);
    std::filesystem::remove(idxTmp, ec);

    ISLogWriter w;
    w.rawOutPath_     = rawOut;
    w.idxOutPath_     = idxOut;
    w.rawTmpPath_     = rawTmp;
    w.idxTmpPath_     = idxTmp;
    w.overwrite_      = opts.overwrite;
    w.sourceDeviceId_ = opts.sourceDeviceId;
    w.lineageNote_    = std::move(opts.lineageNote);
    w.header_         = idx::makeDefaultHeader(kProducerVersion,
                                               opts.tsUnits,
                                               opts.tsSource);
    // SN-8383: ISLogWriter is a derivative re-writer driven by ISRecordView,
    // which does not carry the per-record host-uptime delta. It therefore cannot
    // produce a meaningful `local_uptime_ms`, so it emits honest v2.0 (24-byte)
    // records rather than 32-byte records with a permanently-zero delta and a
    // clear HAS_LOCAL_DELTA flag. The live capture path (DeviceLog) is the v2.1
    // delta producer. (If ISRecordView ever exposes the source delta, this can
    // switch back to v2.1 and copy it through.)
    w.header_.record_size = static_cast<uint16_t>(idx::IS_LOG_IDX_RECORD_V2_SIZE);

    w.rawStream_.open(rawTmp,
                      std::ios::binary | std::ios::out | std::ios::trunc);
    if (!w.rawStream_.is_open()) {
        return fail(ISErrorCode::Io,
                    "ISLogWriter::create: cannot open " + rawTmp.string());
    }

    w.idxStream_.open(idxTmp,
                      std::ios::binary | std::ios::out | std::ios::trunc);
    if (!w.idxStream_.is_open()) {
        w.rawStream_.close();
        std::filesystem::remove(rawTmp, ec);
        return fail(ISErrorCode::Io,
                    "ISLogWriter::create: cannot open " + idxTmp.string());
    }

    // Seed the .idx with a placeholder header. `finalize()` will seek
    // back to offset 0 and rewrite it with real stats + FINALIZED.
    uint8_t buf[idx::IS_LOG_IDX_HEADER_SIZE];
    idx::serializeHeader(buf, w.header_);
    w.idxStream_.write(reinterpret_cast<const char*>(buf),
                       idx::IS_LOG_IDX_HEADER_SIZE);
    if (!w.idxStream_.good()) {
        w.cleanupTempfiles();
        return fail(ISErrorCode::Io,
                    "ISLogWriter::create: failed writing placeholder header");
    }

    w.initialized_ = true;
    return w;
}

ISExpected<ISLogWriter> ISLogWriter::create(
    std::filesystem::path rawOutPath,
    uint64_t sourceDeviceId,
    std::optional<std::string> lineageNote) {
    Options o;
    o.rawOutPath     = std::move(rawOutPath);
    o.sourceDeviceId = sourceDeviceId;
    o.lineageNote    = std::move(lineageNote);
    return create(std::move(o));
}

ISLogWriter::~ISLogWriter() {
    if (initialized_ && !finalized_) {
        std::cerr << "[ISLogWriter] destroyed without finalize() — "
                  << "removing tempfiles for " << rawOutPath_.string()
                  << std::endl;
        cleanupTempfiles();
    }
}

ISLogWriter::ISLogWriter(ISLogWriter&& other) noexcept
    : rawOutPath_(std::move(other.rawOutPath_)),
      idxOutPath_(std::move(other.idxOutPath_)),
      rawTmpPath_(std::move(other.rawTmpPath_)),
      idxTmpPath_(std::move(other.idxTmpPath_)),
      rawStream_(std::move(other.rawStream_)),
      idxStream_(std::move(other.idxStream_)),
      header_(other.header_),
      sourceDeviceId_(other.sourceDeviceId_),
      lineageNote_(std::move(other.lineageNote_)),
      recordCount_(other.recordCount_),
      firstTimestamp_(other.firstTimestamp_),
      lastTimestamp_(other.lastTimestamp_),
      syncPointCount_(other.syncPointCount_),
      rawOffset_(other.rawOffset_),
      initialized_(other.initialized_),
      finalized_(other.finalized_) {
    other.initialized_ = false;
    other.finalized_   = false;
}

ISLogWriter& ISLogWriter::operator=(ISLogWriter&& other) noexcept {
    if (this != &other) {
        if (initialized_ && !finalized_) {
            cleanupTempfiles();
        }
        rawOutPath_     = std::move(other.rawOutPath_);
        idxOutPath_     = std::move(other.idxOutPath_);
        rawTmpPath_     = std::move(other.rawTmpPath_);
        idxTmpPath_     = std::move(other.idxTmpPath_);
        rawStream_      = std::move(other.rawStream_);
        idxStream_      = std::move(other.idxStream_);
        header_         = other.header_;
        sourceDeviceId_ = other.sourceDeviceId_;
        lineageNote_    = std::move(other.lineageNote_);
        recordCount_    = other.recordCount_;
        firstTimestamp_ = other.firstTimestamp_;
        lastTimestamp_  = other.lastTimestamp_;
        syncPointCount_ = other.syncPointCount_;
        rawOffset_      = other.rawOffset_;
        initialized_    = other.initialized_;
        finalized_      = other.finalized_;
        other.initialized_ = false;
        other.finalized_   = false;
    }
    return *this;
}

// ----- Append --------------------------------------------------------------

ISExpected<void> ISLogWriter::append(const ISRecordView& view) {
    if (!initialized_) {
        return fail(ISErrorCode::InvalidArgument,
                    "ISLogWriter::append: writer is uninitialized (moved-from?)");
    }
    if (finalized_) {
        return fail(ISErrorCode::InvalidArgument,
                    "ISLogWriter::append: writer was already finalized");
    }
    auto bytes = view.bytes();
    if (bytes.first == nullptr || bytes.second == 0) {
        return fail(ISErrorCode::InvalidArgument,
                    "ISLogWriter::append: empty record view");
    }
    if (sourceDeviceId_ != 0) {
        const uint64_t recordDevice = view.timestamp().deviceId;
        if (recordDevice != sourceDeviceId_) {
            char msg[256];
            std::snprintf(msg, sizeof(msg),
                          "ISLogWriter::append: record device id %llu "
                          "does not match writer's sourceDeviceId %llu",
                          static_cast<unsigned long long>(recordDevice),
                          static_cast<unsigned long long>(sourceDeviceId_));
            return fail(ISErrorCode::InvalidArgument, msg);
        }
    }

    // Write payload bytes verbatim.
    rawStream_.write(reinterpret_cast<const char*>(bytes.first),
                     static_cast<std::streamsize>(bytes.second));
    if (!rawStream_.good()) {
        return fail(ISErrorCode::Io,
                    "ISLogWriter::append: short write on .raw");
    }

    // Build and emit the .idx record. Offset is the raw write position
    // BEFORE this record's bytes — derived from our own running counter
    // rather than tellp() so the value is well-defined even when the
    // underlying stream is unseekable (pipes in tests, etc.).
    const uint64_t recordOffset = rawOffset_;
    rawOffset_ += bytes.second;

    idx::is_log_idx_record_v2_t rec{};
    rec.timestamp = view.timestamp().value;
    rec.offset    = recordOffset;
    rec.did       = view.did();
    rec.flags     = view.flags();
    rec.reserved  = 0;

    // serializeRecord fills the full 32-byte v2.1 layout; ISLogWriter writes
    // only the 24-byte v2.0 prefix (see header.record_size above). The trailing
    // local_uptime_ms / reserved2 (both 0 here) are intentionally not emitted.
    uint8_t buf[idx::IS_LOG_IDX_RECORD_V2_1_SIZE];
    idx::serializeRecord(buf, rec);
    idxStream_.write(reinterpret_cast<const char*>(buf),
                     idx::IS_LOG_IDX_RECORD_V2_SIZE);
    if (!idxStream_.good()) {
        return fail(ISErrorCode::Io,
                    "ISLogWriter::append: short write on .idx");
    }

    // Stats.
    if (recordCount_ == 0) {
        firstTimestamp_ = rec.timestamp;
    }
    lastTimestamp_ = rec.timestamp;
    if ((rec.flags & idx::IS_LOG_IDX_REC_FLAG_HAS_TOW) != 0) {
        ++syncPointCount_;
    }
    ++recordCount_;
    return {};
}

// ----- Finalize ------------------------------------------------------------

ISExpected<void> ISLogWriter::writeFinalHeader() {
    header_.total_records      = recordCount_;
    header_.first_timestamp_ms = firstTimestamp_;
    header_.last_timestamp_ms  = lastTimestamp_;
    header_.sync_point_count   = syncPointCount_;
    header_.flags             |= idx::IS_LOG_IDX_HDR_FLAG_FINALIZED;

    idxStream_.flush();
    idxStream_.seekp(0, std::ios::beg);
    if (!idxStream_.good()) {
        return fail(ISErrorCode::Io,
                    "ISLogWriter::finalize: seek to header failed");
    }

    uint8_t buf[idx::IS_LOG_IDX_HEADER_SIZE];
    idx::serializeHeader(buf, header_);
    idxStream_.write(reinterpret_cast<const char*>(buf),
                     idx::IS_LOG_IDX_HEADER_SIZE);
    if (!idxStream_.good()) {
        return fail(ISErrorCode::Io,
                    "ISLogWriter::finalize: rewriting header failed");
    }
    idxStream_.flush();
    return {};
}

ISExpected<void> ISLogWriter::finalize() {
    if (!initialized_) {
        return fail(ISErrorCode::InvalidArgument,
                    "ISLogWriter::finalize: writer is uninitialized");
    }
    if (finalized_) {
        return fail(ISErrorCode::InvalidArgument,
                    "ISLogWriter::finalize: already finalized");
    }

    // Flush + close raw before the rename so the OS commits its
    // buffers. ofstream::close() flushes automatically but we
    // double-tap to make the failure visible if it short-writes.
    rawStream_.flush();
    if (!rawStream_.good()) {
        return fail(ISErrorCode::Io,
                    "ISLogWriter::finalize: flushing .raw failed");
    }
    rawStream_.close();

    auto hdr = writeFinalHeader();
    if (!hdr) return hdr;
    idxStream_.close();

    std::error_code ec;
    // std::filesystem::rename won't overwrite an existing destination on
    // Windows; when overwrite was requested, clear the destination first so
    // the rename succeeds on all platforms (POSIX rename overwrites anyway).
    if (overwrite_) {
        std::filesystem::remove(rawOutPath_, ec);
        std::filesystem::remove(idxOutPath_, ec);
        ec.clear();
    }
    std::filesystem::rename(rawTmpPath_, rawOutPath_, ec);
    if (ec) {
        return fail(ISErrorCode::Io,
                    "ISLogWriter::finalize: rename .raw failed: " + ec.message());
    }
    std::filesystem::rename(idxTmpPath_, idxOutPath_, ec);
    if (ec) {
        // Pretty bad — .raw is in place but .idx isn't. Leave the
        // tempfile behind so an admin can recover; the .raw is
        // independently readable (D-04 will scan-rebuild a fresh .idx).
        return fail(ISErrorCode::Io,
                    "ISLogWriter::finalize: rename .idx failed (.raw landed, "
                    ".idx still at " + idxTmpPath_.string() + "): " + ec.message());
    }

    finalized_ = true;
    return {};
}

// ----- Cleanup -------------------------------------------------------------

void ISLogWriter::cleanupTempfiles() noexcept {
    if (rawStream_.is_open()) rawStream_.close();
    if (idxStream_.is_open()) idxStream_.close();
    std::error_code ec;
    if (!rawTmpPath_.empty()) std::filesystem::remove(rawTmpPath_, ec);
    if (!idxTmpPath_.empty()) std::filesystem::remove(idxTmpPath_, ec);
    initialized_ = false;
}

} // namespace inertial_sense
