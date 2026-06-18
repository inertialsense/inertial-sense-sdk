/**
 * @file test_uri.cpp
 * @brief Unit tests for the utils::parseUri() URI-parsing helper.
 *
 * Covers the normalization conventions the helper centralizes: IPv6-bracket stripping,
 * numeric port validation/defaulting, userinfo/path extraction, and the defaults-URI
 * overload that fills any component the primary URI omits.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>

#include "util/util.h"

using utils::UriParts;
using utils::parseUri;

// -------------------------------------------------------------------------------------------------
// Basic component extraction
// -------------------------------------------------------------------------------------------------

TEST(UriParse, FullTcpUrl) {
    const UriParts u = parseUri("tcp://192.168.1.43:7777");
    EXPECT_EQ(u.scheme, "tcp");
    EXPECT_EQ(u.host, "192.168.1.43");
    EXPECT_TRUE(u.hasPort());
    EXPECT_EQ(u.port, 7777);
}

TEST(UriParse, SchemeIsCaseSensitiveAsProvided) {
    // The helper does not lower-case the scheme; callers compare against the expected literal.
    EXPECT_EQ(parseUri("TCP://host:80").scheme, "TCP");
    EXPECT_EQ(parseUri("ntrip://host:2101").scheme, "ntrip");
}

TEST(UriParse, NtripWithUserinfoAndMountpoint) {
    const UriParts u = parseUri("ntrip://user:pass@caster.example.com:2101/MOUNT");
    EXPECT_EQ(u.scheme, "ntrip");
    EXPECT_EQ(u.host, "caster.example.com");
    EXPECT_EQ(u.port, 2101);
    EXPECT_TRUE(u.hasUserinfo());
    EXPECT_EQ(u.user, "user");
    EXPECT_EQ(u.password, "pass");
    EXPECT_EQ(u.path, "/MOUNT");
}

TEST(UriParse, NoUserinfo) {
    const UriParts u = parseUri("ntrip://caster.example.com:2101/MOUNT");
    EXPECT_FALSE(u.hasUserinfo());
    EXPECT_TRUE(u.user.empty());
    EXPECT_TRUE(u.password.empty());
}

// -------------------------------------------------------------------------------------------------
// Host handling (IPv6 bracket stripping)
// -------------------------------------------------------------------------------------------------

TEST(UriParse, Ipv6HostBracketsStripped) {
    const UriParts u = parseUri("tcp://[::1]:7777");
    EXPECT_EQ(u.host, "::1");
    EXPECT_EQ(u.port, 7777);
}

TEST(UriParse, Ipv6FullHostBracketsStripped) {
    const UriParts u = parseUri("tcp://[2001:db8::1]:1234");
    EXPECT_EQ(u.host, "2001:db8::1");
    EXPECT_EQ(u.port, 1234);
}

TEST(UriParse, HostOptionalWhenAuthorityHasPortOnly) {
    const UriParts u = parseUri("tcp://:7777");
    EXPECT_FALSE(u.hasHost());
    EXPECT_TRUE(u.host.empty());
    EXPECT_EQ(u.port, 7777);
}

// -------------------------------------------------------------------------------------------------
// Port validation
// -------------------------------------------------------------------------------------------------

TEST(UriParse, MissingPortIsAbsent) {
    const UriParts u = parseUri("tcp://host");
    EXPECT_FALSE(u.hasPort());
    EXPECT_EQ(u.port, -1);
}

TEST(UriParse, NonNumericPortIsRejectedNotThrown) {
    // FIX8::uri returns the raw token "abc"; the helper must not throw out of std::stoi.
    const UriParts u = parseUri("tcp://host:abc");
    EXPECT_FALSE(u.hasPort());
    EXPECT_EQ(u.port, -1);
}

TEST(UriParse, OutOfRangePortIsRejected) {
    EXPECT_FALSE(parseUri("tcp://host:99999999999").hasPort());  // overflows int
    EXPECT_FALSE(parseUri("tcp://host:70000").hasPort());        // > 65535
    EXPECT_FALSE(parseUri("tcp://host:0").hasPort());            // 0 is not a valid port
}

TEST(UriParse, BoundaryPortsAccepted) {
    EXPECT_EQ(parseUri("tcp://host:1").port, 1);
    EXPECT_EQ(parseUri("tcp://host:65535").port, 65535);
}

// -------------------------------------------------------------------------------------------------
// FIX8::uri requires the "//" authority introducer for host/port parsing
// -------------------------------------------------------------------------------------------------

TEST(UriParse, NoAuthorityIntroducerLeavesHostPortUnparsed) {
    // "tcp:192.168.1.43:7777" has no "//", so the remainder is treated as an opaque path -
    // this documents why the cltool -base help text uses the "tcp://..." form.
    const UriParts u = parseUri("tcp:192.168.1.43:7777");
    EXPECT_EQ(u.scheme, "tcp");
    EXPECT_FALSE(u.hasHost());
    EXPECT_FALSE(u.hasPort());
}

TEST(UriParse, EmptyInput) {
    const UriParts u = parseUri("");
    EXPECT_FALSE(u.hasScheme());
    EXPECT_FALSE(u.hasHost());
    EXPECT_FALSE(u.hasPort());
}

// -------------------------------------------------------------------------------------------------
// Defaults-URI overload
// -------------------------------------------------------------------------------------------------

TEST(UriParseDefaults, FillsMissingHostAndPort) {
    // Mirrors the Rtcm3CorrectionServer listen-URI case.
    const UriParts u = parseUri("tcp://:9999", "tcp://127.0.0.1:7777");
    EXPECT_EQ(u.host, "127.0.0.1");  // host omitted -> default
    EXPECT_EQ(u.port, 9999);         // port present -> kept
}

TEST(UriParseDefaults, AllComponentsDefaultedWhenInputEmpty) {
    const UriParts u = parseUri("", "tcp://127.0.0.1:7777");
    EXPECT_EQ(u.scheme, "tcp");
    EXPECT_EQ(u.host, "127.0.0.1");
    EXPECT_EQ(u.port, 7777);
}

TEST(UriParseDefaults, PrimaryComponentsWin) {
    const UriParts u = parseUri("tcp://10.0.0.5:1234", "tcp://127.0.0.1:7777");
    EXPECT_EQ(u.host, "10.0.0.5");
    EXPECT_EQ(u.port, 1234);
}

TEST(UriParseDefaults, NtripPortDefault) {
    // Mirrors NtripCorrectionService: caster URL without a port falls back to 2101.
    EXPECT_EQ(parseUri("ntrip://caster.example.com/MOUNT", "ntrip://:2101").port, 2101);
    EXPECT_EQ(parseUri("ntrip://caster.example.com:2102/MOUNT", "ntrip://:2101").port, 2102);
}

TEST(UriParseDefaults, InvalidPrimaryPortFallsBackToDefault) {
    // A malformed port in the primary is "absent", so the default applies.
    const UriParts u = parseUri("tcp://host:abc", "tcp://127.0.0.1:7777");
    EXPECT_EQ(u.port, 7777);
}