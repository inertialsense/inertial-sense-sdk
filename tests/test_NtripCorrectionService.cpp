/**
 * @file test_NtripCorrectionService.cpp
 * @brief Response-classification tests for the NTRIP client (SN-8493).
 *
 * The bug these cover: negotiateMountPoint() used to return true for any response it could read
 * without timing out, so a caster that rejected our credentials -- or answered an unknown mount point
 * with its source table -- was indistinguishable from a healthy stream. Every case below is a
 * response a real caster can send, in either the Ntrip 1.0 or the 2.0 dialect.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>

#include "NtripCorrectionService.h"

typedef NtripCorrectionService Ntrip;

/** No Content-Length was sent; a live correction stream never carries one. */
#define NO_LENGTH (-1)

/**
 * @brief Classifies a response head and returns the result, discarding the message.
 * @param status the status line
 * @param type the Content-Type value, empty for none
 * @param length the Content-Length value, NO_LENGTH for none
 * @return the classified result
 */
static Ntrip::eNtripResult classify(const char* status, const char* type = "", long length = NO_LENGTH) {
    std::string message;
    return Ntrip::classifyResponse(status, type, length, message);
}

// ---------------------------------------------------------------------------------------------
// Success
// ---------------------------------------------------------------------------------------------

TEST(NtripResponse, Ntrip1SuccessIcy) {
    // Ntrip 1.0's success line is Shoutcast-derived and is NOT a valid HTTP status line, so a strict
    // HTTP parser would wrongly reject a perfectly good connection.
    EXPECT_EQ(Ntrip::NTRIP_RESULT_OK, classify("ICY 200 OK"));
}

TEST(NtripResponse, Ntrip2SuccessHttp) {
    EXPECT_EQ(Ntrip::NTRIP_RESULT_OK, classify("HTTP/1.1 200 OK", "gnss/data"));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_OK, classify("HTTP/1.0 200 OK", "gnss/data"));
}

TEST(NtripResponse, SuccessMessageIsEmpty) {
    std::string message = "not cleared";
    EXPECT_EQ(Ntrip::NTRIP_RESULT_OK, Ntrip::classifyResponse("ICY 200 OK", "", NO_LENGTH, message));
    EXPECT_TRUE(message.empty());
}

TEST(NtripResponse, StatusLineCaseIsIgnored) {
    // Field names and tokens vary in case between caster implementations.
    EXPECT_EQ(Ntrip::NTRIP_RESULT_OK, classify("icy 200 ok"));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_OK, classify("http/1.1 200 OK", "gnss/data"));
}

// ---------------------------------------------------------------------------------------------
// Credentials
// ---------------------------------------------------------------------------------------------

TEST(NtripResponse, Http401IsUnauthorized) {
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNAUTHORIZED, classify("HTTP/1.1 401 Unauthorized"));
}

TEST(NtripResponse, IcyNon200IsStillParsed) {
    // Some casters answer a poorly-formed Ntrip 1.0 request with an ICY-prefixed error status.
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNAUTHORIZED, classify("ICY 401 Unauthorized"));
}

TEST(NtripResponse, Ntrip1BadPasswordIsUnauthorized) {
    // Ntrip 1.0 adds this custom status code rather than using an HTTP one.
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNAUTHORIZED, classify("ERROR - Bad Password"));
}

TEST(NtripResponse, ForbiddenAndProxyAuthAreUnauthorized) {
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNAUTHORIZED, classify("HTTP/1.1 403 Forbidden"));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNAUTHORIZED, classify("HTTP/1.1 407 Proxy Authentication Required"));
}

TEST(NtripResponse, ReasonPhraseReachesTheMessage) {
    // The caller puts this in front of a user, so the caster's own wording has to survive.
    std::string message;
    Ntrip::classifyResponse("HTTP/1.1 401 Unauthorized", "", NO_LENGTH, message);
    EXPECT_NE(std::string::npos, message.find("401"));
    EXPECT_NE(std::string::npos, message.find("Unauthorized"));
}

// ---------------------------------------------------------------------------------------------
// Mount point
// ---------------------------------------------------------------------------------------------

TEST(NtripResponse, ExplicitSourcetableIsMountpointFailure) {
    EXPECT_EQ(Ntrip::NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE, classify("SOURCETABLE 200 OK"));
}

TEST(NtripResponse, Http404IsMountpointFailure) {
    EXPECT_EQ(Ntrip::NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE, classify("HTTP/1.1 404 Not Found"));
}

TEST(NtripResponse, SourcetableContentTypeIsMountpointFailure) {
    // The Ntrip 2.0 spelling: a 200 whose body is the caster's directory, not corrections.
    EXPECT_EQ(Ntrip::NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE,
              classify("HTTP/1.1 200 OK", "gnss/sourcetable", 4096));
}

TEST(NtripResponse, ContentLengthOnA200IsMountpointFailure) {
    // This is the case that actually bit us. Ntrip 1.0 serves its source table as text/plain, which is
    // too generic to key on -- but a correction stream is endless and so never carries a length, while
    // a source table always does.
    EXPECT_EQ(Ntrip::NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE,
              classify("HTTP/1.1 200 OK", "text/plain", 243));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_MOUNTPOINT_UNAVAILABLE,
              classify("ICY 200 OK", "text/plain", 243));
}

TEST(NtripResponse, PlainTextWithoutLengthIsNotRejected) {
    // Guard against over-eager detection: text/plain alone must not condemn a stream, or a sloppy
    // caster that mislabels a working feed would break.
    EXPECT_EQ(Ntrip::NTRIP_RESULT_OK, classify("HTTP/1.1 200 OK", "text/plain", NO_LENGTH));
}

TEST(NtripResponse, ZeroContentLengthIsNotAFailure) {
    EXPECT_EQ(Ntrip::NTRIP_RESULT_OK, classify("ICY 200 OK", "", 0));
}

// ---------------------------------------------------------------------------------------------
// Other errors and unknown replies
// ---------------------------------------------------------------------------------------------

TEST(NtripResponse, ServerErrorsAreCasterErrors) {
    EXPECT_EQ(Ntrip::NTRIP_RESULT_CASTER_ERROR, classify("HTTP/1.1 500 Internal Server Error"));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_CASTER_ERROR, classify("HTTP/1.1 501 Not Implemented"));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_CASTER_ERROR, classify("HTTP/1.1 503 Service Unavailable"));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_CASTER_ERROR, classify("HTTP/1.1 409 Conflict"));
}

TEST(NtripResponse, GenericErrorStatusIsReportedVerbatim) {
    std::string message;
    EXPECT_EQ(Ntrip::NTRIP_RESULT_CASTER_ERROR,
              Ntrip::classifyResponse("ERROR - Mount Point Taken", "", NO_LENGTH, message));
    EXPECT_NE(std::string::npos, message.find("Mount Point Taken"));
}

TEST(NtripResponse, UnknownReplyIsRejectedNotAssumedGood) {
    // The whole point of the fix: an unrecognised reply must not be treated as success, because that
    // is how a caster quirk becomes a connection that silently never delivers.
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNRECOGNIZED, classify("<html><body>Gateway</body></html>"));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNRECOGNIZED, classify("garbage"));
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNRECOGNIZED, classify(""));
}

TEST(NtripResponse, UnknownReplyIsTruncatedInTheMessage) {
    // A caster that answers with a whole HTML page must not paste it all into a UI label.
    const std::string flood(500, 'x');
    std::string message;
    EXPECT_EQ(Ntrip::NTRIP_RESULT_UNRECOGNIZED,
              Ntrip::classifyResponse(flood, "", NO_LENGTH, message));
    EXPECT_LT(message.length(), 200u);
}
