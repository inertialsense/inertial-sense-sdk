/**
 * @file ISLogSource.h
 * @brief Internal source abstraction under `ISLogReader`.
 *
 * D-02 / SN-7893: a small polymorphic base sits under the reader so
 * v2 can add a port-live source (D0016 streaming) without retrofitting
 * the public reader API. v1 implements only `ISFileSource`. This
 * header is internal — not surfaced through `ISLogReader.h` — so
 * downstream consumers can't accidentally construct a reader on top
 * of an arbitrary source.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "ISError.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>

namespace inertial_sense {

/**
 * @brief Read-only source backing an `ISLogReader`.
 *
 * v1 contract: deliver the entire segment as a contiguous read-only
 * byte range (`data()` + `size()`). The file-source implementation
 * memory-maps the `.raw`. v2 (D-08-era streaming) will introduce a
 * pull-based variant; the public reader API doesn't need to change.
 */
class ISLogSource {
public:
    virtual ~ISLogSource() = default;

    /// Pointer to the first byte of the source. Stable for the source
    /// object's lifetime; never null after construction.
    virtual const uint8_t* data() const noexcept = 0;

    /// Total byte count of the source.
    virtual std::size_t size() const noexcept = 0;

    /// True if the underlying byte range is a memory-mapping. False
    /// when the buffered-I/O fallback was used (mmap unavailable on
    /// the host filesystem). Informational; doesn't change semantics.
    virtual bool isMmapped() const noexcept = 0;
};

/**
 * @brief File-backed `ISLogSource` — mmap'd if possible, buffered
 *        otherwise.
 *
 * Construct via `ISFileSource::open()`. Failures reported through
 * `ISExpected<T>` per D-10. Errors:
 *
 *  - `NotFound`         — file doesn't exist.
 *  - `PermissionDenied` — open() returned EACCES / Windows equivalent.
 *  - `Io`               — both mmap and buffered-read failed.
 *
 * Move-only.
 */
class ISFileSource : public ISLogSource {
public:
    static ISExpected<std::unique_ptr<ISFileSource>>
        open(const std::filesystem::path& path);

    ~ISFileSource() override;

    ISFileSource(const ISFileSource&)            = delete;
    ISFileSource& operator=(const ISFileSource&) = delete;
    ISFileSource(ISFileSource&&) noexcept;
    ISFileSource& operator=(ISFileSource&&) noexcept;

    const uint8_t* data() const noexcept override { return data_; }
    std::size_t    size() const noexcept override { return size_; }
    bool           isMmapped() const noexcept override { return mmapped_; }

private:
    ISFileSource() = default;
    void releaseMapping() noexcept;

    const uint8_t* data_     = nullptr;
    std::size_t    size_     = 0;
    bool           mmapped_  = false;

    // Backing storage when mmap is in use; ignored otherwise.
    void*          mmapBase_ = nullptr;
    std::size_t    mmapLen_  = 0;
    int            fd_       = -1;     // POSIX
#if defined(_WIN32)
    void*          fileHandle_   = nullptr;
    void*          mappingHandle_= nullptr;
#endif

    // Backing storage for buffered fallback.
    std::unique_ptr<uint8_t[]> buffer_;
};

} // namespace inertial_sense
