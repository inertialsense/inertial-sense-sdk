/**
 * Unit tests for serialPortErrorDedupGate() (SN-8650): the pure decision logic that collapses a
 * burst of identical port errors (same action, same code, same port) into one immediate ERROR
 * report plus at most one trailing summary, instead of one log line per occurrence.
 *
 * The field symptom (SN-8650) was a Windows-only, multi-COM-port USB-hub-contention log flood --
 * not independently reproducible in this container (no Windows toolchain) -- but the state
 * machine itself is platform-independent and takes its clock as an explicit parameter, so it's
 * exercised directly here with synthetic timestamps, no OS or logging dependency.
 */

#include <gtest/gtest.h>
#include <cstring>

extern "C" {
#include "serialPortPlatform.h"
}

static serial_port_error_dedup_t freshState() {
    serial_port_error_dedup_t state;
    memset(&state, 0, sizeof(state));
    return state;
}

static const char* ACTION_A = "serialPortWrite():: Error writing: WriteFile() failed";
static const char* ACTION_B = "serialPortWrite():: Error fetching 'overlapped result': GetOverlappedResult() failed";

TEST(SerialPortErrorDedup, FirstEverError_IsReportedImmediately) {
    serial_port_error_dedup_t state = freshState();
    int mode = serialPortErrorDedupGate(&state, ACTION_A, 22, 1000, 500, nullptr, 0);
    EXPECT_EQ(SERIAL_PORT_DEDUP_IMMEDIATE, mode);
}

TEST(SerialPortErrorDedup, FirstDuplicate_IsSuppressed_AndOpensWindow) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 1000, 500, nullptr, 0);   // immediate report

    int mode = serialPortErrorDedupGate(&state, ACTION_A, 22, 1010, 500, nullptr, 0); // 10ms later, same error
    EXPECT_EQ(SERIAL_PORT_DEDUP_SUPPRESSED, mode);
    EXPECT_EQ(1u, state.count);
    EXPECT_EQ(1010u, state.firstMs);
    EXPECT_TRUE(state.pending);   // serialPortErrorDedupCheckStale()'s cheap-bailout flag
}

TEST(SerialPortErrorDedup, DenseBurst_WithinWindow_StaysSuppressedAndSlides) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);      // immediate report @ t=0

    // 50 more occurrences, 100ms apart -- well within the 500ms window each time, so the
    // deadline keeps sliding and nothing should ever be flushed mid-burst.
    uint64_t t = 100;
    for (int i = 0; i < 50; i++) {
        int mode = serialPortErrorDedupGate(&state, ACTION_A, 22, t, 500, nullptr, 0);
        EXPECT_EQ(SERIAL_PORT_DEDUP_SUPPRESSED, mode) << "iteration " << i;
        t += 100;
    }
    EXPECT_EQ(50u, state.count);
    EXPECT_EQ(100u, state.firstMs);   // set once, on the first suppressed duplicate
}

TEST(SerialPortErrorDedup, GapPastWindow_FoldsAndFlushes_WithCountAndDuration) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);         // immediate @ t=0
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0);       // dup #1 @ t=100 (firstMs=100, deadline=600)
    serialPortErrorDedupGate(&state, ACTION_A, 22, 200, 500, nullptr, 0);       // dup #2 @ t=200 (deadline=700)

    char summary[256] = {0};
    // t=1500 is well past the 700ms deadline -- this occurrence should fold in and flush.
    int mode = serialPortErrorDedupGate(&state, ACTION_A, 22, 1500, 500, summary, sizeof(summary));

    EXPECT_EQ(SERIAL_PORT_DEDUP_SUMMARY_FOLDED, mode);
    EXPECT_NE(nullptr, strstr(summary, "duplicated 3 time(s)"));   // dup#1, dup#2, and this folded occurrence
    EXPECT_NE(nullptr, strstr(summary, "over the last 1400ms"));  // 1500 - firstMs(100)
    EXPECT_NE(nullptr, strstr(summary, "(22)"));
    EXPECT_EQ(nullptr, state.action);   // run is closed
    EXPECT_EQ(0u, state.count);
}

TEST(SerialPortErrorDedup, AfterFold_NextIdenticalError_IsReportedFreshNotResumed) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0);
    char summary[256];
    serialPortErrorDedupGate(&state, ACTION_A, 22, 1500, 500, summary, sizeof(summary)); // folds/flushes

    // Same action+code arrives again, long after the fold -- must be a fresh immediate report,
    // not silently resumed suppression (otherwise a quiet port would never get its next error
    // logged at all).
    int mode = serialPortErrorDedupGate(&state, ACTION_A, 22, 300000, 500, nullptr, 0);
    EXPECT_EQ(SERIAL_PORT_DEDUP_IMMEDIATE, mode);
}

TEST(SerialPortErrorDedup, DifferentErrorCode_FlushesPendingRun_ThenReportsImmediately) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);     // immediate
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0);   // dup #1 (opens run)

    char summary[256] = {0};
    int mode = serialPortErrorDedupGate(&state, ACTION_A, 433, 150, 500, summary, sizeof(summary)); // different code

    EXPECT_EQ(SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE, mode);
    EXPECT_NE(nullptr, strstr(summary, "duplicated 1 time(s)"));  // only dup #1 was pending; the
                                                                   // code-433 occurrence itself is
                                                                   // reported separately, not folded
    EXPECT_NE(nullptr, strstr(summary, "(22)"));
    EXPECT_EQ(433, state.errorCode);  // now tracking the new error
    EXPECT_EQ(0u, state.count);
}

TEST(SerialPortErrorDedup, DifferentAction_SameCode_IsTreatedAsADifferentError) {
    // Action is compared by pointer identity (see the function's own doc comment) -- two
    // distinct call sites that happen to report the same numeric code are never conflated.
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0); // opens a run under ACTION_A

    int mode = serialPortErrorDedupGate(&state, ACTION_B, 22, 150, 500, nullptr, 0);
    EXPECT_EQ(SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE, mode);
    EXPECT_EQ(ACTION_B, state.action);
}

TEST(SerialPortErrorDedup, SparseIsolatedRecurrence_AfterLoneReport_IsFreshNotADuplicate) {
    // A lone immediate report that never gained a duplicate used to leave its identity latched
    // forever -- so a much later, independent recurrence of the identical action+code (e.g. the
    // real field log's two GetOverlappedResult(433) events, 24 minutes apart with nothing else
    // in between) would be silently swallowed as "duplicate #1" instead of getting its own
    // report. windowMs now also bounds how long a lone report stays eligible to gain one.
    serial_port_error_dedup_t state = freshState();
    int first = serialPortErrorDedupGate(&state, ACTION_B, 433, 0, 500, nullptr, 0);
    ASSERT_EQ(SERIAL_PORT_DEDUP_IMMEDIATE, first);

    // Same action+code, but long after the 500ms window -- must be fresh, not a suppressed dup.
    int mode = serialPortErrorDedupGate(&state, ACTION_B, 433, 1'440'000, 500, nullptr, 0); // 24 min later
    EXPECT_EQ(SERIAL_PORT_DEDUP_IMMEDIATE, mode);
    EXPECT_FALSE(state.pending);
    EXPECT_EQ(0u, state.count);
}

TEST(SerialPortErrorDedup, RecurrenceWithinWindow_AfterLoneReport_IsStillTreatedAsADuplicate) {
    // The flip side: arriving *within* the window, a second occurrence of the same error is
    // still the normal, intended duplicate-suppression case.
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);

    int mode = serialPortErrorDedupGate(&state, ACTION_A, 22, 200, 500, nullptr, 0);
    EXPECT_EQ(SERIAL_PORT_DEDUP_SUPPRESSED, mode);
    EXPECT_EQ(1u, state.count);
    EXPECT_TRUE(state.pending);
}

TEST(SerialPortErrorDedup, NoPendingRun_DifferentError_IsJustImmediate_NoSpuriousSummary) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);  // immediate; no duplicate followed

    // A different error arrives with nothing pending to flush (count==0) -- must be a plain
    // immediate report, not SUMMARY_THEN_IMMEDIATE (there's nothing to summarize).
    int mode = serialPortErrorDedupGate(&state, ACTION_B, 5, 50, 500, nullptr, 0);
    EXPECT_EQ(SERIAL_PORT_DEDUP_IMMEDIATE, mode);
}

TEST(SerialPortErrorDedup, NullSummaryBuffer_DoesNotCrash_OnFoldOrFlush) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0);
    // Fold path with summaryOut == nullptr.
    EXPECT_EQ(SERIAL_PORT_DEDUP_SUMMARY_FOLDED, serialPortErrorDedupGate(&state, ACTION_A, 22, 5000, 500, nullptr, 0));

    serial_port_error_dedup_t state2 = freshState();
    serialPortErrorDedupGate(&state2, ACTION_A, 22, 0, 500, nullptr, 0);
    serialPortErrorDedupGate(&state2, ACTION_A, 22, 100, 500, nullptr, 0);
    // Code-change flush path with summaryOut == nullptr.
    EXPECT_EQ(SERIAL_PORT_DEDUP_SUMMARY_THEN_IMMEDIATE, serialPortErrorDedupGate(&state2, ACTION_B, 5, 150, 500, nullptr, 0));
}

// --- serialPortErrorDedupCheckStale(): the activity-driven path, independent of the next error ---

TEST(SerialPortErrorDedupCheckStale, NothingPending_ReturnsZero_EvenLongAfter) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);   // immediate; no duplicate ever followed

    EXPECT_EQ(0, serialPortErrorDedupCheckStale(&state, 10'000'000, nullptr, 0));
}

TEST(SerialPortErrorDedupCheckStale, PendingButWithinWindow_ReturnsZero) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0);   // opens run, deadline=600

    EXPECT_EQ(0, serialPortErrorDedupCheckStale(&state, 400, nullptr, 0));
    EXPECT_TRUE(state.pending);   // untouched -- still within the window
}

TEST(SerialPortErrorDedupCheckStale, PendingAndExpired_FlushesWithoutAnyNewError) {
    // This is exactly the real-world gap that motivated this function: a small trailing run on
    // an otherwise-quiet port used to sit open until the *next error* (which, in the field log,
    // was 8+ minutes later) -- now any routine activity (a successful read/write/etc.) on the
    // port flushes it promptly instead.
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);       // immediate @ t=0
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0);     // dup #1 @ t=100 (deadline=600)

    char summary[256] = {0};
    // No further errors at all -- just a routine successful operation, long after the deadline.
    int flushed = serialPortErrorDedupCheckStale(&state, 60000, summary, sizeof(summary));

    EXPECT_EQ(1, flushed);
    EXPECT_NE(nullptr, strstr(summary, "duplicated 1 time(s)"));
    EXPECT_NE(nullptr, strstr(summary, "over the last 59900ms"));   // 60000 - firstMs(100)
    EXPECT_EQ(nullptr, state.action);
    EXPECT_EQ(0u, state.count);
    EXPECT_FALSE(state.pending);
}

TEST(SerialPortErrorDedupCheckStale, AfterStaleFlush_NextIdenticalError_IsReportedFresh) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0);
    serialPortErrorDedupCheckStale(&state, 60000, nullptr, 0);   // flush via activity, no new error involved

    int mode = serialPortErrorDedupGate(&state, ACTION_A, 22, 60050, 500, nullptr, 0);
    EXPECT_EQ(SERIAL_PORT_DEDUP_IMMEDIATE, mode);   // not silently resumed suppression
}

TEST(SerialPortErrorDedupCheckStale, NullSummaryBuffer_DoesNotCrash) {
    serial_port_error_dedup_t state = freshState();
    serialPortErrorDedupGate(&state, ACTION_A, 22, 0, 500, nullptr, 0);
    serialPortErrorDedupGate(&state, ACTION_A, 22, 100, 500, nullptr, 0);
    EXPECT_EQ(1, serialPortErrorDedupCheckStale(&state, 60000, nullptr, 0));
}
