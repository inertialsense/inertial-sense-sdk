// D-10 / SN-7881 smoke test: confirm the vendored tl::expected header
// resolves via the SDK's PUBLIC include path, and round-trip a value
// and an ISError through the canonical `ISExpected<T>` alias.
//
// Stays narrow on purpose — the upstream library has its own
// extensive test suite. This file's job is to prove the SDK
// integration (include path, ISError type, fail() helper) works,
// not to re-test tl::expected itself.

#include <gtest/gtest.h>

#include "ISError.h"

#include <string>

using inertial_sense::ISError;
using inertial_sense::ISErrorCode;
using inertial_sense::ISExpected;
using inertial_sense::fail;

namespace {

// A trivial fallible function, like what new SDK API will look like.
ISExpected<int> safe_divide(int num, int den) {
    if (den == 0) {
        return fail(ISErrorCode::InvalidArgument, "division by zero");
    }
    return num / den;
}

} // namespace

TEST(ISExpected, RoundTripsValue) {
    auto r = safe_divide(10, 2);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(*r, 5);
    EXPECT_EQ(r.value(), 5);
}

TEST(ISExpected, RoundTripsError) {
    auto r = safe_divide(10, 0);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::InvalidArgument);
    EXPECT_EQ(r.error().message, std::string{"division by zero"});
}

TEST(ISExpected, ValueOrFallback) {
    EXPECT_EQ(safe_divide(20, 4).value_or(-1), 5);
    EXPECT_EQ(safe_divide(20, 0).value_or(-1), -1);
}

TEST(ISExpected, FailHelperConstructsUnexpectedDirectly) {
    // Spelled out: the helper produces a `tl::unexpected<ISError>` that
    // implicitly converts to ISExpected<T> with the right error.
    auto err = fail(ISErrorCode::Truncated, "tail at 0x40");
    ISExpected<int> r = err;
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ISErrorCode::Truncated);
    EXPECT_EQ(r.error().message, std::string{"tail at 0x40"});
}

TEST(ISExpected, ErrorCodesAreDistinct) {
    // Sanity: the initial error-code set we ship has no accidental
    // collisions, and Ok occupies slot 0 (so a default-constructed
    // ISErrorCode is not silently treated as a real failure code in
    // future bit-or'd semantics).
    EXPECT_EQ(static_cast<uint16_t>(ISErrorCode::Ok), 0u);
    EXPECT_NE(ISErrorCode::InvalidArgument, ISErrorCode::NotFound);
    EXPECT_NE(ISErrorCode::Io, ISErrorCode::Corrupted);
    EXPECT_NE(ISErrorCode::Truncated, ISErrorCode::Unsupported);
}
