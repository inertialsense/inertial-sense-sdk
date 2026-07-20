/**
 * @file ISLogReaderSugar.h
 * @brief Templated sugar layer over `ISLogReader::records(did)` —
 *        D-03 / SN-7894.
 *
 * Yields `const T&` for compile-time-known DIDs:
 *
 * @code
 *   auto reader = ISLogReader::openSegment(path).value();
 *   for (const ins_2_t& ins : reader.records<DID_INS_2>()) {
 *       std::cout << ins.lla[0] << '\n';
 *   }
 * @endcode
 *
 * **Why a separate header.** `DIDTraits.h` pulls in `data_sets.h`,
 * which is large; consumers that only use the type-erased core
 * (`records(did)` + `ISRecordView`) shouldn't pay that compile cost.
 * Include this header explicitly to opt in.
 *
 * **Implementation note.** Eagerly walks the segment's raw bytes via
 * `is_comm_parse_byte` and materializes the typed payloads. This
 * sidesteps the chunk-shared-offset semantics of
 * `ISRecordView::bytes()` (multiple records that arrived in one
 * chunk-input observe the same byte range, which is *not* the
 * payload-only range a typed cast wants — see D0053). Eager
 * materialization costs one parse pass per `records<DID>()` call;
 * cache the range in user code if you iterate multiple times.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#pragma once

#include "DIDTraits.h"
#include "ISLogReader.h"
#include "ISTimeStamp.h"

#include <cstring>
#include <iterator>
#include <utility>
#include <vector>

namespace inertial_sense {

/**
 * @brief Eagerly-materialized range of typed payload structs +
 *        timestamps, returned by `ISLogReader::records<DID>()`.
 *
 * Iteration yields `const T&` (the payload struct). Each element
 * also carries a `TimeStamp` accessible via `iterator::timestamp()`.
 *
 * Range concept: forward-iterable, sized, copyable. Lifetime is
 * independent of the parent reader once constructed (the typed
 * payloads are owning copies, not aliases of the reader's mmap).
 */
template <class T>
class TypedRange {
public:
    using value_type = T;

    using sample_t = std::pair<TimeStamp, T>;

    explicit TypedRange(std::vector<sample_t> samples) noexcept
        : samples_(std::move(samples)) {}

    TypedRange()                                  = default;
    TypedRange(const TypedRange&)                 = default;
    TypedRange(TypedRange&&) noexcept             = default;
    TypedRange& operator=(const TypedRange&)      = default;
    TypedRange& operator=(TypedRange&&) noexcept  = default;

    class iterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type        = T;
        using difference_type   = std::ptrdiff_t;
        using reference         = const T&;
        using pointer           = const T*;

        iterator() = default;
        explicit iterator(typename std::vector<sample_t>::const_iterator it) noexcept
            : it_(it) {}

        const T&    operator*()  const noexcept { return it_->second; }
        const T*    operator->() const noexcept { return &it_->second; }
        TimeStamp   timestamp()  const noexcept { return it_->first;  }

        iterator&   operator++()    noexcept { ++it_; return *this; }
        iterator    operator++(int) noexcept { auto tmp = *this; ++it_; return tmp; }

        friend bool operator==(const iterator& a, const iterator& b) noexcept {
            return a.it_ == b.it_;
        }
        friend bool operator!=(const iterator& a, const iterator& b) noexcept {
            return !(a == b);
        }

    private:
        typename std::vector<sample_t>::const_iterator it_{};
    };

    iterator begin() const noexcept { return iterator{ samples_.cbegin() }; }
    iterator end()   const noexcept { return iterator{ samples_.cend()   }; }

    std::size_t size()  const noexcept { return samples_.size();  }
    bool        empty() const noexcept { return samples_.empty(); }

    /// Direct access to the underlying (timestamp, payload) pairs.
    /// Useful when the caller wants the timestamp alongside the value.
    const std::vector<sample_t>& samples() const noexcept { return samples_; }

private:
    std::vector<sample_t> samples_;
};

namespace detail {

/**
 * Walk the segment's raw bytes once via `is_comm_parse_byte`,
 * collecting `(TimeStamp, T)` pairs for every ISB packet whose DID
 * matches `did` and whose payload size equals `sizeof(T)`. Mismatched
 * sizes (alternate-format DIDs, partial packets) are silently
 * skipped — a size mismatch on a known DID is a structure drift
 * we'd rather log than abort on; the caller can detect zero
 * extracted samples on a non-empty segment if that ever matters.
 *
 * Defined in `ISLogReaderSugar.cpp` to keep this header light.
 */
template <class T>
TypedRange<T> extractTypedRange(const ISLogReader& reader, did_t did);

} // namespace detail

/**
 * @brief Templated sugar — returns a `TypedRange<T>` of payload structs.
 *
 * Compile error if `DID` has no `DIDTraits` specialization (the
 * `static_assert` makes the failure message clear). For DIDs that *do*
 * have a specialization but are absent from the segment, returns an
 * empty range.
 *
 * Implemented as a free function (not an `ISLogReader` member) so the
 * core reader header doesn't drag in `DIDTraits.h`'s heavy includes.
 * Spelled at the call site via ADL or qualified:
 *
 * @code
 *   for (const ins_2_t& ins : records<DID_INS_2>(reader)) { ... }
 * @endcode
 *
 * For the canonical member-style call (`reader.records<DID>()`),
 * see the inline shim below.
 */
template <did_t DID>
TypedRange<typename DIDTraits<DID>::type>
records_typed(const ISLogReader& reader) {
    static_assert(has_traits_v<DID>,
                  "records<DID>(): no DIDTraits specialization for this DID. "
                  "See SDK/src/DIDTraits.h to add one (or wait for the auto-"
                  "generated set).");
    using T = typename DIDTraits<DID>::type;
    static_assert(sizeof(T) == DIDTraits<DID>::expected_size,
                  "DIDTraits expected_size mismatches sizeof(type) — header drift?");
    return detail::extractTypedRange<T>(reader, static_cast<did_t>(DID));
}

} // namespace inertial_sense

// ---------------------------------------------------------------------------
// Member-style spelling: `reader.records<DID_INS_2>()`.
//
// Implemented as an out-of-line method template via the wrapper above.
// Keeping it in `ISLogReader.h` would force every consumer to drag in
// `DIDTraits.h` even when they only use the type-erased core, which
// the AC explicitly cautions against. The shim here lets callers who
// `#include "ISLogReaderSugar.h"` get the dot-call ergonomics without
// that cost.
// ---------------------------------------------------------------------------

namespace inertial_sense {

template <did_t DID>
TypedRange<typename DIDTraits<DID>::type> ISLogReader::records() const {
    return records_typed<DID>(*this);
}

} // namespace inertial_sense
