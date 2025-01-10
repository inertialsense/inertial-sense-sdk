/**
 * @file ISDevice.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDevice.h"
#include "ISFirmwareUpdater.h"

/**
 * @return true if this device is in the process of being updated, otherwise returns false.
 * False is returned regardless of whether the update was successful or not.
 */
bool ISDeviceUpdater::inProgress() { return (fwUpdater && !fwUpdater->fwUpdate_isDone()); }

/**
 * Instructs the device to continue performing its actions.  This should be called regularly to ensure that the update process
 * does not stall.
 * @return true if the update is still in progress (calls inProgress()), or false if the update is finished and no further updates are needed.
 */
bool ISDeviceUpdater::update() {
    if (fwUpdater) {
        fwUpdater->fwUpdate_step();

        if ("upload" == fwUpdater->getActiveCommand()) {
            if (fwUpdater->fwUpdate_getSessionTarget() != lastTarget) {
                hasError = false;
                lastStatus = fwUpdate::NOT_STARTED;
                lastMessage.clear();
                lastTarget = fwUpdater->fwUpdate_getSessionTarget();
            }
            lastSlot = fwUpdater->fwUpdate_getSessionImageSlot();

            if ((fwUpdater->fwUpdate_getSessionStatus() == fwUpdate::NOT_STARTED) && fwUpdater->isWaitingResponse()) {
                // We're just starting (no error yet, but no response either)
                lastStatus = fwUpdate::INITIALIZING;
                lastMessage = ISFirmwareUpdater::fwUpdate_getNiceStatusName(lastStatus);
            } else if ((fwUpdater->fwUpdate_getSessionStatus() != fwUpdate::NOT_STARTED) && (lastStatus != fwUpdater->fwUpdate_getSessionStatus())) {
                // We're got a valid status update (error or otherwise)
                lastStatus = fwUpdater->fwUpdate_getSessionStatus();
                lastMessage = ISFirmwareUpdater::fwUpdate_getNiceStatusName(lastStatus);

                // check for error
                if (!hasError && ((fwUpdater->fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED) || fwUpdater->hasErrors())) {
                    hasError = true;
                }
            }

            // update our upload progress
            if (lastStatus == fwUpdate::IN_PROGRESS) {
                percent = ((float) fwUpdater->fwUpdate_getNextChunkID() / (float) fwUpdater->fwUpdate_getTotalChunks()) * 100.f;
            } else {
                percent = lastStatus <= fwUpdate::READY ? 0.f : 100.f;
            }
        } else if ("waitfor" == fwUpdater->getActiveCommand()) {
            lastMessage = "Waiting for response from device.";
        } else if ("reset" == fwUpdater->getActiveCommand()) {
            lastMessage = "Resetting device.";
        } else if ("delay" == fwUpdater->getActiveCommand()) {
            lastMessage = "Waiting...";
        }

        if (!fwUpdater->hasPendingCommands()) {
            if (fwUpdater->hasErrors()) {
                hasError = fwUpdater->hasErrors();
                lastMessage = "Error: ";
                lastMessage += "One or more step errors occurred.";
            } else if (hasError) {
                lastMessage = "Error: ";
                lastMessage += ISFirmwareUpdater::fwUpdate_getNiceStatusName(lastStatus);
            } else {
                lastMessage = "Completed successfully.";
            }
        }

        // cleanup if we're done.
        bool is_done = fwUpdater->fwUpdate_isDone();
        if (is_done) {
            // collect errors before we close out the updater
            errors = fwUpdater->getStepErrors();
            hasError |= !errors.empty();

            delete fwUpdater;
            fwUpdater = nullptr;
        }
    } else {
        percent = 0.0;
    }

    return inProgress();
}

