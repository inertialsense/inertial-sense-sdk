/**
 * @file test_port_open_async_contract.cpp
 * @brief SN-8571 -- locks in portOpen()'s asynchronous contract (a single call may return
 *        PORT_ERROR__NONE while a non-blocking connect is still in flight, without setting
 *        PORT_FLAG__OPENED) and proves portOpenRetry() correctly waits for it.
 *
 * Scaffolded on a minimal SDK-layer fake port (a bare base_port_t with a custom portOpen
 * callback) rather than a real socket/fd -- portable, and it exercises the contract the SDK
 * itself owns (base_port.h) rather than the host OS's TCP stack timing.
 */

#include <gtest/gtest.h>

#include "core/base_port.h"

namespace {

// Minimal base_port_t-compatible fake port that simulates an asynchronous connect: the first
// `pendingCalls` portOpen() calls return PORT_ERROR__NONE without setting PORT_FLAG__OPENED --
// exactly like tcpPortOpen()'s non-blocking connect() reporting EINPROGRESS -- and only the
// call after that sets the flag.
struct fake_async_port_t {
    base_port_t base = {};
    int pendingCalls = 0;   //!< portOpen() calls remaining before the simulated handshake completes
    int openCallCount = 0;  //!< total portOpen() invocations seen, for assertions
};

int fakeAsyncPortOpen(port_handle_t port) {
    auto* p = reinterpret_cast<fake_async_port_t*>(port);
    ++p->openCallCount;
    if (p->pendingCalls > 0) {
        --p->pendingCalls;
        return PORT_ERROR__NONE;   // "connect in progress" -- NONE, but deliberately not open yet
    }
    portFlagsSet(port, PORT_FLAG__OPENED);
    return PORT_ERROR__NONE;
}

fake_async_port_t makeFakeAsyncPort(int pendingCalls) {
    fake_async_port_t p;
    p.base.pnum = 1;
    p.base.ptype = PORT_TYPE__COMM;
    p.base.pflags = PORT_FLAG__VALID;   // portIsValid() requires this in addition to the checksum
    p.base.portOpen = fakeAsyncPortOpen;
    p.pendingCalls = pendingCalls;
    portRecalcChksum(&p);
    return p;
}

} // namespace

// SN-8571 AC: "A test fails if a delayed-open port is treated as immediately open." This locks
// in the platform contract a bare portOpen() call relies on callers respecting: on an
// asynchronous transport, ONE portOpen() call can legitimately return PORT_ERROR__NONE while the
// port is still not actually open. Any caller that skips the portIsOpened() check after a single
// portOpen() call is exactly the SN-8571 bug pattern (SN-8508, SN-8561).
TEST(PortOpenAsyncContract, BareCallDoesNotImplyOpenOnAsyncTransport) {
    fake_async_port_t port = makeFakeAsyncPort(/*pendingCalls=*/5);

    const int result = portOpen(&port);
    EXPECT_EQ(result, PORT_ERROR__NONE)
        << "an in-progress async connect reports NONE, not an error";
    EXPECT_FALSE(portIsOpened(&port))
        << "PORT_ERROR__NONE must NOT be treated as \"open\" -- the handshake is still pending";
}

// The serial (synchronous) case must be unaffected: the port sets PORT_FLAG__OPENED on the very
// first call, so a bare portOpen() IS sufficient there (SN-8571 AC: "no behaviour change on
// synchronous transports").
TEST(PortOpenAsyncContract, BareCallDoesImplyOpenOnSynchronousTransport) {
    fake_async_port_t port = makeFakeAsyncPort(/*pendingCalls=*/0);  // opens on the first call

    const int result = portOpen(&port);
    EXPECT_EQ(result, PORT_ERROR__NONE);
    EXPECT_TRUE(portIsOpened(&port))
        << "a synchronous port opens on the first call and must report open immediately";
}

// portOpenRetry() is the fix: it must keep polling a delayed-open port until it actually opens,
// rather than accepting the first PORT_ERROR__NONE.
TEST(PortOpenAsyncContract, PortOpenRetryWaitsForDelayedOpen) {
    fake_async_port_t port = makeFakeAsyncPort(/*pendingCalls=*/5);

    const int result = portOpenRetry(&port, /*timeoutMs=*/1000, /*retryDelayMs=*/1);
    EXPECT_EQ(result, PORT_ERROR__NONE);
    EXPECT_TRUE(portIsOpened(&port))
        << "portOpenRetry() must not return success until the port actually reports open";
    EXPECT_GE(port.openCallCount, 6)
        << "portOpenRetry() must re-invoke portOpen() -- a sleep alone never advances the handshake";
}

// portOpenRetry() must still fail (not hang or false-positive) if the port never actually opens
// within the timeout.
TEST(PortOpenAsyncContract, PortOpenRetryTimesOutIfNeverOpens) {
    fake_async_port_t port = makeFakeAsyncPort(/*pendingCalls=*/1000000);  // never opens in time

    const int result = portOpenRetry(&port, /*timeoutMs=*/20, /*retryDelayMs=*/1);
    EXPECT_NE(result, PORT_ERROR__NONE);
    EXPECT_FALSE(portIsOpened(&port));
}
