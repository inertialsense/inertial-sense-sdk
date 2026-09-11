/**
 * @file test_ISFirmwareUpdater_isDone.cpp
 * @brief SN-8579 -- fwUpdate_isDone()'s in_progress term checked `session_id == 0`, which is
 *        backwards: fwUpdate_requestUpdate() assigns a fresh non-zero session_id when a session
 *        starts and fwUpdate_handleDone() clears it to 0 when one ends, so `session_id != 0` is
 *        what "session active" means everywhere else in this class.
 *
 * The one pre-existing test that drives fwUpdate_isDone() (test_ISFirmwarePackage.cpp) is
 * #ifdef-gated behind a hardcoded local firmware-package path and not part of CI -- it provides no
 * actual regression coverage. These tests exercise the corrected formula directly against the
 * session_id/session_status state space, since driving a real session end-to-end needs a live
 * port, a connected device, and a real firmware image.
 *
 * The fixture is friended by ISFirmwareUpdater (guarded by SDK_UNIT_TEST, see ISFirmwareUpdater.h)
 * so it can poke the privately-inherited session_id/session_status directly -- no test-only method
 * on the class itself, and nothing that exists outside a unit-test build.
 */

#include <gtest/gtest.h>
#include "gtest_helpers.h"
#include "test_data_utils.h"

#include "ISFileManager.h"
#include "ISFirmwareUpdater.h"

using namespace fwUpdate;

class FwUpdateIsDoneTest : public ::testing::Test {
protected:
    dev_info_t devInfo{};
    ISFwUpdateState state;
    ISFirmwareUpdater updater{static_cast<port_handle_t>(nullptr), &devInfo, state};

    void setSessionState(uint16_t sessionId, update_status_e status) {
        updater.session_id = sessionId;
        updater.session_status = status;
    }
};

TEST_F(FwUpdateIsDoneTest, NoSessionEverStarted_IsDone) {
    setSessionState(0, NOT_STARTED);
    EXPECT_TRUE(updater.fwUpdate_isDone())
        << "a freshly-constructed updater with no session and no queued commands is done";
}

TEST_F(FwUpdateIsDoneTest, ActiveSession_Ready_NotDone) {
    setSessionState(1234, READY);
    EXPECT_FALSE(updater.fwUpdate_isDone())
        << "session_id != 0 (assigned by fwUpdate_requestUpdate()) + READY must read as in-progress";
}

TEST_F(FwUpdateIsDoneTest, ActiveSession_InProgress_NotDone) {
    setSessionState(1234, IN_PROGRESS);
    EXPECT_FALSE(updater.fwUpdate_isDone())
        << "this is the exact case the pre-fix code got backwards: an active session_id with "
           "status genuinely mid-transfer must NOT report done";
}

TEST_F(FwUpdateIsDoneTest, ActiveSession_Finalizing_NotDone) {
    setSessionState(1234, FINALIZING);
    EXPECT_FALSE(updater.fwUpdate_isDone())
        << "FINALIZING is still mid-flight (waiting on the device to confirm) -- not done";
}

TEST_F(FwUpdateIsDoneTest, SessionClearedAfterSuccess_IsDone) {
    // Mirrors fwUpdate_handleDone(): session_id cleared to 0, status set to the terminal value
    // from the MSG_UPDATE_DONE payload.
    setSessionState(0, FINISHED);
    EXPECT_TRUE(updater.fwUpdate_isDone());
}

TEST_F(FwUpdateIsDoneTest, SessionClearedAfterError_IsDone) {
    setSessionState(0, ERR_FLASH_INVALID);
    EXPECT_TRUE(updater.fwUpdate_isDone());
}

// Documents why the bug was masked in the ordinary success/failure case: FINISHED and every
// ERR_* code fall OUTSIDE the (NOT_STARTED, FINISHED) open range by construction of the enum
// values, so the status term alone already excludes them regardless of which way the session_id
// check goes. A stale, never-cleared session_id (exactly what a skipped fwUpdate_sendDone() -- the
// other half of SN-8579 -- leaves behind) is harmless IF status also reached a genuine terminal
// value via a progress message. The bug only bites while status is still genuinely mid-flight
// (see the three ActiveSession_* cases above), which is what makes it hide behind
// hasPendingCommands() in the common single-target case and only surface in back-to-back updates.
TEST_F(FwUpdateIsDoneTest, StaleSessionIdWithTerminalStatus_StillIsDone) {
    setSessionState(1234, FINISHED);  // session_id never cleared, status is terminal
    EXPECT_TRUE(updater.fwUpdate_isDone());
}
