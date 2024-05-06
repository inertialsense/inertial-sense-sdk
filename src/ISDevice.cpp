/**
 * @file ISDevice.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDevice.h"
#include "ISFirmwareUpdater.h"

bool ISDeviceUpdater::inProgress() { return (fwUpdater && !fwUpdater->fwUpdate_isDone()); }

void ISDeviceUpdater::update() {
    if (fwUpdater) {
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
                if (!hasError && fwUpdater && fwUpdater->fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED) {
                    hasError = true;
                }
            }

            // update our upload progress
            if ((lastStatus == fwUpdate::IN_PROGRESS)) {
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
            if (!hasError) {
                lastMessage = "Completed successfully.";
            } else {
                lastMessage = "Error: ";
                lastMessage += ISFirmwareUpdater::fwUpdate_getNiceStatusName(lastStatus);
            }
        }

        // cleanup if we're done.
        if (fwUpdater->fwUpdate_isDone()) {
            delete fwUpdater;
            fwUpdater = nullptr;
        }
    } else {
        percent = 0.0;
    }
}

