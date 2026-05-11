/**
 * @file ISLogReader.cpp
 * @brief Segment reader implementation. See ISLogReader.h.
 *
 * D-02 / SN-7893. Pure SDK code — no Qt, no exceptions, no legacy `cDeviceLog*` / `cISLogger` headers (D-02 DoD #3).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISLogReader.h"

#include "core/msg_logger.h"

// com_manager.h FIRST — short-circuits the broken extern-C wrap in ISFirmwareUpdater.h that would otherwise trip C++
// overloads. Same trick used in test_log_index.cpp.
#include "com_manager.h"

#include "ISComm.h"
#include "ISDataMappings.h"
#include "data_sets.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <system_error>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <unistd.h>
#endif

namespace inertial_sense {

namespace fs = std::filesystem;

const std::vector<std::size_t> ISLogReader::kEmptyIndices_ = {};

// ============================================================
// ISFileSource — mmap with buffered fallback.
// ============================================================

ISFileSource::ISFileSource(ISFileSource&& other) noexcept
    : data_(other.data_),
      size_(other.size_),
      mmapped_(other.mmapped_),
      mmapBase_(other.mmapBase_),
      mmapLen_(other.mmapLen_),
      fd_(other.fd_),
#if defined(_WIN32)
      fileHandle_(other.fileHandle_),
      mappingHandle_(other.mappingHandle_),
#endif
      buffer_(std::move(other.buffer_)) {
    other.data_     = nullptr;
    other.size_     = 0;
    other.mmapped_  = false;
    other.mmapBase_ = nullptr;
    other.mmapLen_  = 0;
    other.fd_       = -1;
#if defined(_WIN32)
    other.fileHandle_    = nullptr;
    other.mappingHandle_ = nullptr;
#endif
}

ISFileSource& ISFileSource::operator=(ISFileSource&& other) noexcept {
    if (this != &other) {
        releaseMapping();
        data_     = other.data_;
        size_     = other.size_;
        mmapped_  = other.mmapped_;
        mmapBase_ = other.mmapBase_;
        mmapLen_  = other.mmapLen_;
        fd_       = other.fd_;
#if defined(_WIN32)
        fileHandle_    = other.fileHandle_;
        mappingHandle_ = other.mappingHandle_;
#endif
        buffer_   = std::move(other.buffer_);

        other.data_     = nullptr;
        other.size_     = 0;
        other.mmapped_  = false;
        other.mmapBase_ = nullptr;
        other.mmapLen_  = 0;
        other.fd_       = -1;
#if defined(_WIN32)
        other.fileHandle_    = nullptr;
        other.mappingHandle_ = nullptr;
#endif
    }
    return *this;
}

ISFileSource::~ISFileSource() {
    releaseMapping();
}

void ISFileSource::releaseMapping() noexcept {
#if defined(_WIN32)
    if (mmapped_ && mmapBase_) {
        UnmapViewOfFile(mmapBase_);
    }
    if (mappingHandle_) {
        CloseHandle(static_cast<HANDLE>(mappingHandle_));
    }
    if (fileHandle_) {
        CloseHandle(static_cast<HANDLE>(fileHandle_));
    }
    mappingHandle_ = nullptr;
    fileHandle_    = nullptr;
#else
    if (mmapped_ && mmapBase_ && mmapLen_) {
        munmap(mmapBase_, mmapLen_);
    }
    if (fd_ >= 0) {
        ::close(fd_);
    }
    fd_ = -1;
#endif
    mmapBase_ = nullptr;
    mmapLen_  = 0;
    mmapped_  = false;
    data_     = nullptr;
    size_     = 0;
}

ISExpected<std::unique_ptr<ISFileSource>> ISFileSource::open(const fs::path& path) {
    std::error_code ec;
    if (!fs::exists(path, ec)) {
        return fail(ISErrorCode::NotFound, "ISFileSource::open: file not found: " + path.string());
    }
    const auto sz = fs::file_size(path, ec);
    if (ec) {
        return fail(ISErrorCode::Io, "ISFileSource::open: file_size() failed: " + ec.message());
    }

    auto out = std::unique_ptr<ISFileSource>(new ISFileSource());
    out->size_ = static_cast<std::size_t>(sz);

    if (out->size_ == 0) {
        // Empty file. No mapping needed; data_ stays null and bytes() returns (nullptr, 0). Treat as not-mmapped for
        // clarity.
        return out;
    }

#if defined(_WIN32)
    HANDLE h = CreateFileW(path.wstring().c_str(),
                           GENERIC_READ,
                           FILE_SHARE_READ,
                           nullptr,
                           OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL,
                           nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        const DWORD err = GetLastError();
        if (err == ERROR_ACCESS_DENIED) {
            return fail(ISErrorCode::PermissionDenied, "ISFileSource::open: access denied: " + path.string());
        }
        return fail(ISErrorCode::Io, "ISFileSource::open: CreateFileW failed: " + path.string());
    }
    out->fileHandle_ = h;

    HANDLE mapping = CreateFileMappingW(h, nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (mapping) {
        void* base = MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0);
        if (base) {
            out->mappingHandle_ = mapping;
            out->mmapBase_      = base;
            out->mmapLen_       = out->size_;
            out->mmapped_       = true;
            out->data_          = static_cast<const uint8_t*>(base);
            return out;
        }
        CloseHandle(mapping);
    }
    // mmap failed — fall through to buffered read.
#else
    int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        if (errno == EACCES) {
            return fail(ISErrorCode::PermissionDenied, "ISFileSource::open: access denied: " + path.string());
        }
        return fail(ISErrorCode::Io, "ISFileSource::open: open() failed: " + path.string() + " (" + std::strerror(errno) + ")");
    }
    out->fd_ = fd;

    void* base = ::mmap(nullptr, out->size_, PROT_READ, MAP_PRIVATE, fd, 0);
    if (base != MAP_FAILED) {
        out->mmapBase_ = base;
        out->mmapLen_  = out->size_;
        out->mmapped_  = true;
        out->data_     = static_cast<const uint8_t*>(base);
        return out;
    }
    // mmap failed (e.g. tmpfs-on-some-kernels, network FS) — fall through.
#endif

    // Buffered fallback: read the whole file into a heap-allocated buffer. v1 segments are < 16 MB by D-01's writer cap
    // so this is fine; if 32-bit machines hit large-segment cases later, that's a future story (per D-02 spec note on
    // 2 GB).
    auto buf = std::unique_ptr<uint8_t[]>(new uint8_t[out->size_]);
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return fail(ISErrorCode::Io, "ISFileSource::open: ifstream open failed: " + path.string());
    }
    in.read(reinterpret_cast<char*>(buf.get()), static_cast<std::streamsize>(out->size_));
    if (!in) {
        return fail(ISErrorCode::Io, "ISFileSource::open: short read: " + path.string());
    }
    out->buffer_ = std::move(buf);
    out->data_   = out->buffer_.get();
    out->mmapped_ = false;
    return out;
}

// ============================================================
// ISLogReader — public surface.
// ============================================================

ISLogReader::~ISLogReader() = default;
ISLogReader::ISLogReader(ISLogReader&&) noexcept = default;
ISLogReader& ISLogReader::operator=(ISLogReader&&) noexcept = default;

ISExpected<ISLogReader> ISLogReader::openSegment(const fs::path& raw) {
    log_debug(IS_LOG_ISLOG, "ISLogReader::openSegment: %s", raw.c_str());
    auto src = ISFileSource::open(raw);
    if (!src) {
        log_error(IS_LOG_ISLOG, "ISFileSource::open failed for %s: %s", raw.c_str(), src.error().message.c_str());
        return tl::unexpected<ISError>{ src.error() };
    }
    return construct(std::move(*src), raw);
}

ISExpected<ISLogReader> ISLogReader::construct(std::unique_ptr<ISLogSource> rawSource, const fs::path& rawPath) {
    ISLogReader r;
    r.rawSource_ = std::move(rawSource);
    r.rawPath_   = rawPath;

    // -----------------------------------------------------------------
    // Sidecar discovery: replace ".raw" with ".idx" (matches the writer convention from cDeviceLog::OpenNewSaveFile /
    // m_fileName + ".idx").
    // -----------------------------------------------------------------
    fs::path idxPath = rawPath;
    if (idxPath.extension() == ".raw") {
        idxPath.replace_extension(".idx");
    } else {
        idxPath += ".idx";   // defensive — writer always uses .raw today
    }
    r.idxPath_ = idxPath;

    // -----------------------------------------------------------------
    // Try the on-disk sidecar first. If it parses cleanly AND its counters are consistent with the .raw, use it.
    // Otherwise fall through to the scan-rebuild path (D-04).
    // -----------------------------------------------------------------
    enum class RebuildReason { None, Missing, V1, Corrupted, Stale, ReadError };
    RebuildReason rebuildReason = RebuildReason::Missing;

    std::error_code ec;
    if (fs::exists(idxPath, ec)) {
        auto idxSrc = ISFileSource::open(idxPath);
        if (!idxSrc) {
            rebuildReason = RebuildReason::ReadError;
        } else {
            const auto& s = **idxSrc;
            if (s.size() < idx::IS_LOG_IDX_HEADER_SIZE) {
                rebuildReason = RebuildReason::Corrupted;
            } else {
                uint8_t hdrBuf[idx::IS_LOG_IDX_HEADER_SIZE];
                std::memcpy(hdrBuf, s.data(), idx::IS_LOG_IDX_HEADER_SIZE);
                auto hdr = idx::parseHeader(hdrBuf);
                if (!hdr) {
                    // LegacyFormat (no magic) → v1 sidecar. Other errors (Unsupported / Corrupted) → drop and rebuild.
                    rebuildReason = (hdr.error().code == ISErrorCode::LegacyFormat)
                        ? RebuildReason::V1
                        : RebuildReason::Corrupted;
                } else {
                    const std::size_t bodyStart = hdr->header_size;
                    if (bodyStart > s.size()) {
                        rebuildReason = RebuildReason::Corrupted;
                    } else {
                        const std::size_t bodyBytes = s.size() - bodyStart;
                        const std::size_t nRecords  = bodyBytes / idx::IS_LOG_IDX_RECORD_V2_SIZE;

                        // Stale-detection: if the header advertises a total_records that disagrees with what's
                        // physically present in the file body, the sidecar was truncated, partially overwritten, or
                        // otherwise out-of-sync with the .raw. FINALIZED + matching count is the happy path.
                        const bool finalized = (hdr->flags & idx::IS_LOG_IDX_HDR_FLAG_FINALIZED) != 0;
                        const bool countConsistent = !finalized || hdr->total_records == nRecords;

                        if (!countConsistent) {
                            rebuildReason = RebuildReason::Stale;
                        } else {
                            // Pull every record.
                            std::vector<idx::is_log_idx_record_v2_t> recs;
                            recs.reserve(nRecords);
                            for (std::size_t i = 0; i < nRecords; ++i) {
                                uint8_t recBuf[idx::IS_LOG_IDX_RECORD_V2_SIZE];
                                std::memcpy(recBuf, s.data() + bodyStart + i * idx::IS_LOG_IDX_RECORD_V2_SIZE, idx::IS_LOG_IDX_RECORD_V2_SIZE);
                                recs.push_back(idx::parseRecord(recBuf));
                            }

                            // Poison sweep: pre-fix v2 .idx files baked the host's wall-clock value (~1.7e12 ms in 2026)
                            // into the timestamp field whenever a record had no internal time, courtesy of the old
                            // cISDataMappings::TimestampOrCurrentTime call in buildIndexFromScan. Those values overwhelm
                            // valid GPS-ToW (max 604_800_000 ms) downstream consumers see, producing fold-back
                            // rendering. Detect + rebuild.
                            //
                            // Threshold: 1_000_000_000_000 ms — far above any plausible PayloadToW or host-uptime, but
                            // well below 2031's wall-clock floor.
                            //
                            // SN-8004: DID_GPS*_RAW records legitimately carry Unix-epoch-seconds timestamps via gtime_t
                            // (cISDataMappings::Timestamp returns `obs.time.sec + obs.time.time` for those DIDs —
                            // that's seconds since 1970, ~1.7e9 in 2026, which becomes ~1.7e12 ms after the *1000
                            // conversion). Exclude them from the sweep — otherwise every RTK log ping-pongs between
                            // rebuild and re-flag-as-stale on each load.
                            //
                            // @note D-112 / SN-7999 (introduced) + SN-8004 (GPS_RAW exclusion).
                            auto isGpsRawDid = [](uint32_t did) noexcept {
                                return did == DID_GPS1_RAW || did == DID_GPS2_RAW || did == DID_GPS_BASE_RAW;
                            };
                            constexpr uint64_t kPoisonThresholdMs = 1'000'000'000'000ULL;
                            std::size_t poisonedCount = 0;
                            for (const auto& rec : recs) {
                                if (isGpsRawDid(rec.did)) continue;
                                if (rec.timestamp > kPoisonThresholdMs) {
                                    ++poisonedCount;
                                    if (poisonedCount > 4) break;   // early-exit; rebuild anyway
                                }
                            }
                            if (poisonedCount > 0) {
                                rebuildReason = RebuildReason::Stale;
                            } else {
                                r.header_         = *hdr;
                                r.hadOnDiskIndex_ = true;
                                r.buildIndexFromIdx(recs);
                                // Trusted index → assume the .raw is intact end-to-end. A future story can optionally
                                // tail-verify, but here we honor the FINALIZED flag.
                                r.isTruncated_      = false;
                                r.truncationOffset_ = r.rawSource_->size();
                                rebuildReason = RebuildReason::None;
                            }
                        }
                    }
                }
            }
        }
    }

    if (!r.hadOnDiskIndex_) {
        // Default header; counters get filled in by buildIndexFromScan.
        r.header_ = idx::makeDefaultHeader(0, idx::TimestampUnits::HostUptimeMs, idx::HeaderTimeSource::Mixed);
        r.buildIndexFromScan();

        // Document the rebuild for the caller.
        const char* reasonStr = "unknown";
        switch (rebuildReason) {
            case RebuildReason::Missing:    reasonStr = "missing";    break;
            case RebuildReason::V1:         reasonStr = "v1";         break;
            case RebuildReason::Stale:      reasonStr = "stale";      break;
            case RebuildReason::Corrupted:  reasonStr = "corrupted";  break;
            case RebuildReason::ReadError:  reasonStr = "read-error"; break;
            case RebuildReason::None:       reasonStr = "n/a";        break;
        }
        r.warnings_.push_back(std::string{"sidecar: rebuilt from .raw scan (reason: "} + reasonStr + ")");
        log_warn(IS_LOG_ISLOG, "%s: sidecar rebuilt from .raw scan (reason: %s)", rawPath.filename().c_str(), reasonStr);

        // Persist the rebuilt sidecar. Suppressed when the build flips IS_LOG_READER_NO_PERSIST_INDEX (e.g. tests,
        // customers who don't want surprise writes to log dirs) or when the .raw sits on read-only media.
#if !defined(IS_LOG_READER_NO_PERSIST_INDEX)
        if (!r.persistIndex()) {
            r.warnings_.push_back("sidecar: persist failed (read-only filesystem?)");
            log_warn(IS_LOG_ISLOG, "%s: sidecar persist failed (read-only filesystem?)", rawPath.filename().c_str());
        }
#endif
    }

    r.deriveDeviceId(rawPath);

    // Build allIndices_ once (0..N-1). RangeIterator over allRecords() walks this; it's the canonical "everything"
    // range.
    r.allIndices_.resize(r.records_.size());
    for (std::size_t i = 0; i < r.allIndices_.size(); ++i) {
        r.allIndices_[i] = i;
    }

    return r;
}

void ISLogReader::buildIndexFromIdx(const std::vector<idx::is_log_idx_record_v2_t>& records) {
    records_ = records;
    byDid_.clear();
    byDid_.reserve(64);
    for (std::size_t i = 0; i < records_.size(); ++i) {
        byDid_[records_[i].did].push_back(i);
    }
}

void ISLogReader::buildIndexFromScan() {
    records_.clear();
    byDid_.clear();
    isTruncated_ = false;
    truncationOffset_ = rawSource_ ? rawSource_->size() : 0;

    if (!rawSource_ || rawSource_->size() == 0) {
        return;
    }

    const uint8_t* base   = rawSource_->data();
    const std::size_t total = rawSource_->size();

    // is_comm_init wants a scratch buffer to hold the in-progress packet. PKT_BUF_SIZE is the SDK's max-packet bound.
    // Keep it on the stack — it's small (a few KB).
    is_comm_instance_t comm{};
    uint8_t commBuf[PKT_BUF_SIZE];
    is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);
    is_comm_enable_protocol(&comm, _PTYPE_NMEA);
    is_comm_enable_protocol(&comm, _PTYPE_RTCM3);
    is_comm_enable_protocol(&comm, _PTYPE_UBLOX);

    // Track the file offset of the byte immediately after the last successful packet emit. When a packet emits at byte
    // index `i`, the packet started at `lastEmitEnd` and ended at `i`, so the .idx record offset is `lastEmitEnd`.
    // After the emit we set lastEmitEnd = i + 1.
    std::size_t lastEmitEnd = 0;

    for (std::size_t i = 0; i < total; ++i) {
        protocol_type_t ptype = is_comm_parse_byte(&comm, base[i]);
        if (ptype == _PTYPE_NONE) continue;

        if (ptype == _PTYPE_INERTIAL_SENSE_DATA ||
            ptype == _PTYPE_INERTIAL_SENSE_CMD) {
            // ISB packet — record into the index.
            const auto& dataHdr = comm.rxPkt.dataHdr;
            // D-112 / SN-7999 follow-up: use Timestamp(), NOT TimestampOrCurrentTime(). The legacy "OrCurrentTime"
            // fallback returns the *host wall-clock* (1.7e9 sec since Unix epoch as of 2026) when a record has no
            // internal timestamp. That value gets baked into the .idx and then mixed with valid GPS-ToW values from
            // sibling records, producing chart fold-back rendering and broken `spanStart()`/`spanEnd()` extents.
            // `Timestamp()` returns 0 for records without an internal time field — a clean sentinel downstream
            // consumers (RawSeriesBuilder, ISDeviceLog::spanStart) can skip cleanly.
            const double tsSec = cISDataMappings::Timestamp(&dataHdr, comm.rxPkt.data.ptr);
            const uint64_t tsMs = static_cast<uint64_t>(tsSec * 1000.0);

            idx::is_log_idx_record_v2_t rec{};
            rec.timestamp = tsMs;
            rec.offset    = static_cast<uint64_t>(lastEmitEnd);
            rec.did       = dataHdr.id;
            rec.flags     = 0;
            rec.reserved  = 0;
            records_.push_back(rec);
        }
        // Whether or not we recorded this packet (NMEA/RTCM/UBX skipped), its bytes are consumed; advance the post-emit
        // cursor.
        lastEmitEnd = i + 1;
    }

    // After the scan, any bytes the parser consumed-but-didn't-emit since lastEmitEnd are an incomplete trailing packet
    // — the truncation signal.
    if (lastEmitEnd < total) {
        isTruncated_      = true;
        truncationOffset_ = static_cast<uint64_t>(lastEmitEnd);
        warnings_.push_back("truncation: stopped at offset " + std::to_string(lastEmitEnd) + " (file size " + std::to_string(total) + ")");
        log_warn(IS_LOG_ISLOG, "%s: truncated, stopped at offset %zu (file size %zu)",
                 rawPath_.filename().c_str(), static_cast<std::size_t>(lastEmitEnd), static_cast<std::size_t>(total));
    } else {
        truncationOffset_ = total;
    }

    // Populate byDid_ and finalize header counters.
    byDid_.clear();
    byDid_.reserve(64);
    for (std::size_t i = 0; i < records_.size(); ++i) {
        byDid_[records_[i].did].push_back(i);
    }
    if (!records_.empty()) {
        header_.total_records       = records_.size();
        header_.first_timestamp_ms  = records_.front().timestamp;
        header_.last_timestamp_ms   = records_.back().timestamp;
        header_.flags |= idx::IS_LOG_IDX_HDR_FLAG_FINALIZED;
    }
}

bool ISLogReader::persistIndex() const {
    if (idxPath_.empty() || records_.empty()) return false;

    // Atomic write: <raw>.idx.tmp → rename → <raw>.idx. A crash mid-write leaves the old sidecar (if any) intact.
    const fs::path tmpPath = idxPath_.string() + ".tmp";

    std::ofstream out(tmpPath, std::ios::binary | std::ios::trunc);
    if (!out) return false;

    uint8_t hdrBuf[idx::IS_LOG_IDX_HEADER_SIZE];
    idx::serializeHeader(hdrBuf, header_);
    out.write(reinterpret_cast<const char*>(hdrBuf), idx::IS_LOG_IDX_HEADER_SIZE);
    if (!out) { std::error_code ec; fs::remove(tmpPath, ec); return false; }

    for (const auto& rec : records_) {
        uint8_t recBuf[idx::IS_LOG_IDX_RECORD_V2_SIZE];
        idx::serializeRecord(recBuf, rec);
        out.write(reinterpret_cast<const char*>(recBuf), idx::IS_LOG_IDX_RECORD_V2_SIZE);
        if (!out) { std::error_code ec; fs::remove(tmpPath, ec); return false; }
    }
    out.close();
    if (!out) { std::error_code ec; fs::remove(tmpPath, ec); return false; }

    std::error_code ec;
    fs::rename(tmpPath, idxPath_, ec);
    if (ec) {
        // Cross-volume rename or permission issue — fall back to copy.
        fs::copy_file(tmpPath, idxPath_, fs::copy_options::overwrite_existing, ec);
        std::error_code rmEc;
        fs::remove(tmpPath, rmEc);
        if (ec) return false;
    }
    return true;
}

uint64_t ISLogReader::fileSize() const noexcept {
    return rawSource_ ? rawSource_->size() : 0;
}

std::pair<const uint8_t*, std::size_t> ISLogReader::rawBytes() const noexcept {
    if (!rawSource_) return { nullptr, 0 };
    return { rawSource_->data(), rawSource_->size() };
}

void ISLogReader::deriveDeviceId(const fs::path& rawPath) {
    deviceId_ = 0;
    hdwId_    = 0;

    // 1) Walk the raw bytes via `is_comm_parse_byte` looking for the first DID_DEV_INFO packet. We can't shortcut to
    //    the record's `.offset` because that is the ISB packet *start* (framing + header + payload + checksum), not
    //    the payload offset — we need the parser to hand us `comm.rxPkt.data.ptr`, which points into the dev_info_t
    //    payload proper. Bounded: terminate at the first DEV_INFO packet so this is at most a single early-exit
    //    linear pass.
    if (rawSource_ && rawSource_->size() > 0) {
        is_comm_instance_t comm{};
        uint8_t commBuf[PKT_BUF_SIZE];
        is_comm_init(&comm, commBuf, sizeof(commBuf), nullptr);
        is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);

        const uint8_t* base = rawSource_->data();
        const std::size_t total = rawSource_->size();
        for (std::size_t i = 0; i < total; ++i) {
            protocol_type_t p = is_comm_parse_byte(&comm, base[i]);
            if (p != _PTYPE_INERTIAL_SENSE_DATA && p != _PTYPE_INERTIAL_SENSE_CMD) {
                continue;
            }
            if (comm.rxPkt.dataHdr.id != DID_DEV_INFO) continue;
            if (comm.rxPkt.dataHdr.size != sizeof(dev_info_t)) continue;

            dev_info_t info{};
            std::memcpy(&info, comm.rxPkt.data.ptr, sizeof(info));
            if (info.serialNumber != 0) {
                deviceId_ = info.serialNumber;
                hdwId_    = static_cast<uint16_t>(ENCODE_DEV_INFO_TO_HDW_ID(info));
                return;
            }
            // First DEV_INFO had a zero serial — partial-update record or test fixture stub. Stop scanning; let the
            // filename fallback handle it.
            break;
        }
    }

    // 2) Filename fallback. cltool/cISLogger emits names of the form `LOG_SN<n>_<timestamp>_<seq>.raw`. Extract the
    //    digits between "SN" and the next underscore. The fallback can recover the serial number but not the hardware
    //    id, so `hdwId_` stays 0.
    const std::string stem = rawPath.stem().string();   // strips ".raw"
    const std::size_t snPos = stem.find("SN");
    if (snPos != std::string::npos) {
        std::size_t i = snPos + 2;
        uint64_t value = 0;
        bool any = false;
        while (i < stem.size() && std::isdigit(static_cast<unsigned char>(stem[i]))) {
            value = value * 10 + static_cast<uint64_t>(stem[i] - '0');
            ++i;
            any = true;
        }
        if (any) deviceId_ = value;
    }
}

uint64_t ISLogReader::segmentStartTimestamp() const noexcept {
    if (records_.empty()) return 0;
    if (hadOnDiskIndex_ && header_.first_timestamp_ms != 0) {
        return header_.first_timestamp_ms;
    }
    return records_.front().timestamp;
}

uint64_t ISLogReader::segmentEndTimestamp() const noexcept {
    if (records_.empty()) return 0;
    if (hadOnDiskIndex_ && header_.last_timestamp_ms != 0) {
        return header_.last_timestamp_ms;
    }
    return records_.back().timestamp;
}

std::vector<ISLogReader::did_t> ISLogReader::presentDids() const {
    std::vector<did_t> out;
    out.reserve(byDid_.size());
    for (const auto& kv : byDid_) {
        out.push_back(kv.first);
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::size_t ISLogReader::recordEndOffset(std::size_t recordIdx) const noexcept {
    if (recordIdx + 1 < records_.size()) {
        // Records share an arrival-order array but their .raw offsets are per-arrival-chunk (the writer increments
        // m_lastIndexOffset after each SaveData call). We use a sorted scan to find the smallest offset strictly
        // greater than the current record's, which is the natural end-of-bytes for "this record" in the contiguous
        // chunk-input case. If multiple records share an offset (one parser-loop emitted several index records), they
        // all observe the same byte range — documented in ISRecordView.
        const uint64_t cur = records_[recordIdx].offset;
        uint64_t next = rawSource_ ? rawSource_->size() : cur;
        for (std::size_t i = recordIdx + 1; i < records_.size(); ++i) {
            if (records_[i].offset > cur) {
                next = records_[i].offset;
                break;
            }
        }
        return static_cast<std::size_t>(next);
    }
    return rawSource_ ? rawSource_->size() : 0;
}

ISRecordView ISLogReader::viewAt(std::size_t recordIdx) const noexcept {
    if (recordIdx >= records_.size() || !rawSource_) {
        return {};
    }
    const auto& rec = records_[recordIdx];
    const std::size_t endOff = recordEndOffset(recordIdx);
    const std::size_t off    = static_cast<std::size_t>(rec.offset);
    const uint8_t* base      = rawSource_->data();
    const std::size_t total  = rawSource_->size();

    const uint8_t* dataPtr = nullptr;
    std::size_t    dataLen = 0;
    if (off < total) {
        dataPtr = base + off;
        dataLen = (endOff > off && endOff <= total) ? (endOff - off) : 0;
    }

    return ISRecordView{
        rec.did,
        rec.timestamp,
        deviceId_,
        rec.offset,
        dataPtr,
        dataLen,
        rec.flags,
    };
}

ISRecordView ISLogReader::RangeIterator::operator*() const noexcept {
    if (parent_ == nullptr || indices_ == nullptr || pos_ >= indices_->size()) {
        return {};
    }
    return parent_->viewAt((*indices_)[pos_]);
}

ISLogReader::Range ISLogReader::records(did_t did) const noexcept {
    auto it = byDid_.find(did);
    if (it == byDid_.end()) {
        return Range{ this, &kEmptyIndices_, 0, 0 };
    }
    return Range{ this, &it->second, 0, it->second.size() };
}

ISLogReader::Range ISLogReader::allRecords() const noexcept {
    return Range{ this, &allIndices_, 0, allIndices_.size() };
}

ISLogReader::Range ISLogReader::Range::in_time(TimeStamp t0, TimeStamp t1) const {
    if (parent_ == nullptr || indices_ == nullptr) return *this;
    if (begin_ >= end_) return *this;
    // Linear scan to find the first index with timestamp >= t0.value.
    std::size_t lo = begin_;
    while (lo < end_ && parent_->records_[(*indices_)[lo]].timestamp < t0.value) {
        ++lo;
    }
    // And last index with timestamp <= t1.value (exclusive end).
    std::size_t hi = lo;
    while (hi < end_ && parent_->records_[(*indices_)[hi]].timestamp <= t1.value) {
        ++hi;
    }
    return Range{ parent_, indices_, lo, hi };
}

ISLogReader::RangeIterator ISLogReader::seek(TimeStamp target) const noexcept {
    if (records_.empty()) {
        return RangeIterator{ this, &allIndices_, 0 };
    }

    // Detect monotonic timestamps in records_; if so, do a binary search. Otherwise fall back to linear scan. We don't
    // cache the monotonicity check because the records_ vector is immutable after construction; the cost is amortized
    // over potentially many seek() calls but recomputed each call. For typical workloads this is acceptable; if it
    // shows up in profiling, cache the boolean in a const member set in construct().
    bool monotonic = true;
    for (std::size_t i = 1; i < records_.size(); ++i) {
        if (records_[i].timestamp < records_[i - 1].timestamp) {
            monotonic = false;
            break;
        }
    }

    std::size_t pos;
    if (monotonic) {
        // std::lower_bound on the timestamps via the all-indices view.
        auto it = std::lower_bound(allIndices_.begin(), allIndices_.end(), target.value,
            [this](std::size_t i, uint64_t t) { return records_[i].timestamp < t; });
        pos = static_cast<std::size_t>(it - allIndices_.begin());
    } else {
        pos = allIndices_.size();
        for (std::size_t i = 0; i < allIndices_.size(); ++i) {
            if (records_[allIndices_[i]].timestamp >= target.value) {
                pos = i;
                break;
            }
        }
    }
    return RangeIterator{ this, &allIndices_, pos };
}

} // namespace inertial_sense
