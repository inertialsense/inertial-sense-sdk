/**
 * @file ISLogReader.cpp
 * @brief Segment reader implementation. See ISLogReader.h.
 *
 * D-02 / SN-7893. Pure SDK code — no Qt, no exceptions, no legacy
 * `cDeviceLog*` / `cISLogger` headers (D-02 DoD #3).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include "ISLogReader.h"

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
        return fail(ISErrorCode::NotFound,
                    "ISFileSource::open: file not found: " + path.string());
    }
    const auto sz = fs::file_size(path, ec);
    if (ec) {
        return fail(ISErrorCode::Io,
                    "ISFileSource::open: file_size() failed: " + ec.message());
    }

    auto out = std::unique_ptr<ISFileSource>(new ISFileSource());
    out->size_ = static_cast<std::size_t>(sz);

    if (out->size_ == 0) {
        // Empty file. No mapping needed; data_ stays null and bytes()
        // returns (nullptr, 0). Treat as not-mmapped for clarity.
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
            return fail(ISErrorCode::PermissionDenied,
                        "ISFileSource::open: access denied: " + path.string());
        }
        return fail(ISErrorCode::Io,
                    "ISFileSource::open: CreateFileW failed: " + path.string());
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
            return fail(ISErrorCode::PermissionDenied,
                        "ISFileSource::open: access denied: " + path.string());
        }
        return fail(ISErrorCode::Io,
                    "ISFileSource::open: open() failed: " + path.string()
                    + " (" + std::strerror(errno) + ")");
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

    // Buffered fallback: read the whole file into a heap-allocated
    // buffer. v1 segments are < 16 MB by D-01's writer cap so this
    // is fine; if 32-bit machines hit large-segment cases later, that's
    // a future story (per D-02 spec note on 2 GB).
    auto buf = std::unique_ptr<uint8_t[]>(new uint8_t[out->size_]);
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return fail(ISErrorCode::Io,
                    "ISFileSource::open: ifstream open failed: " + path.string());
    }
    in.read(reinterpret_cast<char*>(buf.get()), static_cast<std::streamsize>(out->size_));
    if (!in) {
        return fail(ISErrorCode::Io,
                    "ISFileSource::open: short read: " + path.string());
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
    auto src = ISFileSource::open(raw);
    if (!src) {
        return tl::unexpected<ISError>{ src.error() };
    }
    return construct(std::move(*src), raw);
}

ISExpected<ISLogReader>
    ISLogReader::construct(std::unique_ptr<ISLogSource> rawSource,
                           const fs::path& rawPath) {
    ISLogReader r;
    r.rawSource_ = std::move(rawSource);

    // -----------------------------------------------------------------
    // Sidecar: replace ".raw" with ".idx" (matches the writer
    // convention from cDeviceLog::OpenNewSaveFile / m_fileName + ".idx").
    // -----------------------------------------------------------------
    fs::path idxPath = rawPath;
    if (idxPath.extension() == ".raw") {
        idxPath.replace_extension(".idx");
    } else {
        // Fallback: append ".idx" (e.g. `.bin.idx` if a non-standard
        // naming sneaks in). Writer always uses `.raw` today; this
        // branch is defensive.
        idxPath += ".idx";
    }

    std::error_code ec;
    if (fs::exists(idxPath, ec)) {
        auto idxSrc = ISFileSource::open(idxPath);
        if (!idxSrc) {
            // Index couldn't be opened (corrupt FS, perm change between
            // exists() and open()). Treat as missing — fall through to
            // the lazy scan.
        } else {
            const auto& s = **idxSrc;
            if (s.size() >= idx::IS_LOG_IDX_HEADER_SIZE) {
                uint8_t hdrBuf[idx::IS_LOG_IDX_HEADER_SIZE];
                std::memcpy(hdrBuf, s.data(), idx::IS_LOG_IDX_HEADER_SIZE);
                auto hdr = idx::parseHeader(hdrBuf);
                if (hdr) {
                    // Pull every record. The writer uses
                    // header_size = IS_LOG_IDX_HEADER_SIZE; if a future v3
                    // grows the header, hdr.header_size lets us skip the
                    // unknown trailing fields without misreading records.
                    const std::size_t bodyStart = hdr->header_size;
                    if (bodyStart > s.size()) {
                        return fail(ISErrorCode::Corrupted,
                                    "ISLogReader: .idx header_size exceeds file size: "
                                    + idxPath.string());
                    }
                    const std::size_t bodyBytes = s.size() - bodyStart;
                    const std::size_t nRecords  = bodyBytes / idx::IS_LOG_IDX_RECORD_V2_SIZE;
                    std::vector<idx::is_log_idx_record_v2_t> recs;
                    recs.reserve(nRecords);
                    for (std::size_t i = 0; i < nRecords; ++i) {
                        uint8_t recBuf[idx::IS_LOG_IDX_RECORD_V2_SIZE];
                        std::memcpy(recBuf,
                                    s.data() + bodyStart + i * idx::IS_LOG_IDX_RECORD_V2_SIZE,
                                    idx::IS_LOG_IDX_RECORD_V2_SIZE);
                        recs.push_back(idx::parseRecord(recBuf));
                    }
                    r.header_         = *hdr;
                    r.hadOnDiskIndex_ = true;
                    r.buildIndexFromIdx(recs);
                }
                // hdr.error() is LegacyFormat / Unsupported / Corrupted —
                // same fallback path as missing sidecar.
            }
        }
    }

    if (!r.hadOnDiskIndex_) {
        // Fabricate a default header. Lazy index will populate the
        // counters from the .raw scan (D-04 will persist this back
        // to disk; for D-02 we just hold it in memory).
        r.header_ = idx::makeDefaultHeader(0, idx::TimestampUnits::HostUptimeMs,
                                           idx::HeaderTimeSource::Mixed);
        r.buildIndexFromScan();
    }

    r.deriveDeviceId(rawPath);

    // Build allIndices_ once (0..N-1). RangeIterator over allRecords()
    // walks this; it's the canonical "everything" range.
    r.allIndices_.resize(r.records_.size());
    for (std::size_t i = 0; i < r.allIndices_.size(); ++i) {
        r.allIndices_[i] = i;
    }

    return r;
}

void ISLogReader::buildIndexFromIdx(
        const std::vector<idx::is_log_idx_record_v2_t>& records) {
    records_ = records;
    byDid_.clear();
    byDid_.reserve(64);
    for (std::size_t i = 0; i < records_.size(); ++i) {
        byDid_[records_[i].did].push_back(i);
    }
}

void ISLogReader::buildIndexFromScan() {
    // D-04 is responsible for the full robust scan-and-rebuild. For
    // D-02 we land an empty fallback — the reader still answers
    // metadata queries (recordCount() == 0, allRecords() empty), and
    // the AC explicitly says missing-sidecar is not an error. The
    // .raw bytes remain accessible via the source for any downstream
    // consumer who wants to do their own parse.
    records_.clear();
    byDid_.clear();
}

void ISLogReader::deriveDeviceId(const fs::path& rawPath) {
    deviceId_ = 0;

    // 1) Try to find a DID_DEV_INFO record in the index. We need its
    //    bytes to read the serialNumber field. dev_info_t is defined
    //    in data_sets.h with serialNumber as the first field; rather
    //    than pull the full struct here (which would couple this file
    //    to a heavy header), we read the first uint32_t at the
    //    record's offset, which is the serialNumber per the layout.
    //    DID_DEV_INFO == 1 (data_sets.h:40).
    constexpr uint32_t kDevInfoDid = 1u;
    auto it = byDid_.find(kDevInfoDid);
    if (it != byDid_.end() && !it->second.empty() && rawSource_) {
        const auto& rec = records_[it->second.front()];
        if (rec.offset + sizeof(uint32_t) <= rawSource_->size()) {
            uint32_t sn = 0;
            std::memcpy(&sn, rawSource_->data() + rec.offset, sizeof(uint32_t));
            // dev_info_t::serialNumber is little-endian on disk (ARM SDK).
            // A serial of 0 is implausible for real hardware, so we fall
            // through to filename parsing if so — protects against the
            // record being a partial-update where serialNumber wasn't set.
            if (sn != 0) {
                deviceId_ = sn;
                return;
            }
        }
    }

    // 2) Filename fallback. cltool/cISLogger emits names of the form
    //    `LOG_SN<n>_<timestamp>_<seq>.raw`. Extract the digits between
    //    "SN" and the next underscore.
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
        // Records share an arrival-order array but their .raw offsets
        // are per-arrival-chunk (the writer increments m_lastIndexOffset
        // after each SaveData call). We use a sorted scan to find the
        // smallest offset strictly greater than the current record's,
        // which is the natural end-of-bytes for "this record" in the
        // contiguous chunk-input case. If multiple records share an
        // offset (one parser-loop emitted several index records), they
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
    while (lo < end_ &&
           parent_->records_[(*indices_)[lo]].timestamp < t0.value) {
        ++lo;
    }
    // And last index with timestamp <= t1.value (exclusive end).
    std::size_t hi = lo;
    while (hi < end_ &&
           parent_->records_[(*indices_)[hi]].timestamp <= t1.value) {
        ++hi;
    }
    return Range{ parent_, indices_, lo, hi };
}

ISLogReader::RangeIterator ISLogReader::seek(TimeStamp target) const noexcept {
    if (records_.empty()) {
        return RangeIterator{ this, &allIndices_, 0 };
    }

    // Detect monotonic timestamps in records_; if so, do a binary
    // search. Otherwise fall back to linear scan. We don't cache the
    // monotonicity check because the records_ vector is immutable
    // after construction; the cost is amortized over potentially many
    // seek() calls but recomputed each call. For typical workloads
    // this is acceptable; if it shows up in profiling, cache the
    // boolean in a const member set in construct().
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
        auto it = std::lower_bound(
            allIndices_.begin(), allIndices_.end(), target.value,
            [this](std::size_t i, uint64_t t) {
                return records_[i].timestamp < t;
            });
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
