/**
 * @file ISDevice.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDevice.h"
#include "ISFirmwareUpdater.h"

#include "ISBFirmwareUpdater.h"
#include "ISDFUFirmwareUpdater.h"

bool ISDevice::fwUpdateInProgress() { return (fwUpdater && !fwUpdater->fwUpdate_isDone()); }

bool ISDevice::fwUpdate() {
    if (fwUpdater) {
        fwUpdater->fwUpdate_step();
        if ("upload" == fwUpdater->getActiveCommand()) {
            if (fwUpdater->fwUpdate_getSessionTarget() != fwState.lastTarget) {
                fwState.hasError = false;
                fwState.lastStatus = fwUpdate::NOT_STARTED;
                fwState.lastMessage.clear();
                fwState.lastTarget = fwUpdater->fwUpdate_getSessionTarget();
            }
            fwState.lastSlot = fwUpdater->fwUpdate_getSessionImageSlot();

            if ((fwUpdater->fwUpdate_getSessionStatus() == fwUpdate::NOT_STARTED) && fwUpdater->isWaitingResponse()) {
                // We're just starting (no error yet, but no response either)
                fwState.lastStatus = fwUpdate::INITIALIZING;
                fwState.lastMessage = ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwState.lastStatus);
            } else if ((fwUpdater->fwUpdate_getSessionStatus() != fwUpdate::NOT_STARTED) && (fwState.lastStatus != fwUpdater->fwUpdate_getSessionStatus())) {
                // We're got a valid status update (error or otherwise)
                fwState.lastStatus = fwUpdater->fwUpdate_getSessionStatus();
                fwState.lastMessage = ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwState.lastStatus);

                // check for error
                if (!fwState.hasError && fwUpdater && fwUpdater->fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED) {
                    fwState.hasError = true;
                }
            }

            // update our upload progress
            if ((fwState.lastStatus == fwUpdate::IN_PROGRESS)) {
                fwState.percent = fwUpdater->getPercentComplete(); // ((float) fwUpdater->fwUpdate_getNextChunkID() / (float) fwUpdater->fwUpdate_getTotalChunks()) * 100.f;
            } else {
                fwState.percent = fwState.lastStatus <= fwUpdate::READY ? 0.f : 100.f;
            }
        } else if ("waitfor" == fwUpdater->getActiveCommand()) {
            fwState.lastMessage = "Waiting for response from device.";
        } else if ("reset" == fwUpdater->getActiveCommand()) {
            fwState.lastMessage = "Resetting device.";
        } else if ("delay" == fwUpdater->getActiveCommand()) {
            fwState.lastMessage = "Waiting...";
        }

        if (!fwUpdater->hasPendingCommands()) {
            if (!fwState.hasError) {
                fwState.lastMessage = "Completed successfully.";
            } else {
                fwState.lastMessage = "Error: ";
                fwState.lastMessage += ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwState.lastStatus);
            }
        }

        // cleanup if we're done.
        if (fwUpdater->fwUpdate_isDone()) {
            delete fwUpdater;
            fwUpdater = nullptr;
        }
    } else {
        fwState.percent = 0.0;
    }
    return fwUpdateInProgress();
}

bool ISDevice::handshakeISB() {
    static const uint8_t handshakerChar = 'U';

    // Bootloader sync requires at least 6 'U' characters to be sent every 10ms.
    // write a 'U' to handshake with the boot loader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_RETRIES; i++) {
        if(serialPortWrite(&serialPort, &handshakerChar, 1) != 1) {
            return false;
        }

        if (serialPortWaitForTimeout(&serialPort, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY)) {
            return true;           // Success
        }
    }

    return false;
}

bool ISDevice::queryDeviceInfoISB() {
    uint8_t buf[64] = {};

    handshakeISB();     // We have to handshake before we can do anything... if we've already handshaked, we won't go a response, so ignore this result

    serialPortFlush(&serialPort);
    serialPortRead(&serialPort, buf, sizeof(buf));    // empty Rx buffer

    // Query device
    serialPortWrite(&serialPort, (uint8_t*)":020000041000EA", 15);

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
    int count = serialPortReadTimeout(&serialPort, buf, 14, 1000);
    if (count >= 8 && buf[0] == 0xAA && buf[1] == 0x55)
    {   // expected response
        devInfo.firmwareVer[0] = buf[2];
        devInfo.firmwareVer[1] = buf[3];
        // m_isb_props.rom_available = buf[4];

        if(buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
        {
            switch ((eProcessorType)buf[5]) {
                case IS_PROCESSOR_UNKNOWN:
                    devInfo.hardwareType = IS_HARDWARE_TYPE_UNKNOWN;
                    break;
                case IS_PROCESSOR_SAMx70:
                    devInfo.hardwareType = IS_HARDWARE_TYPE_EVB;
                    break;
                case IS_PROCESSOR_STM32L4:
                    // IMX-5.0
                    devInfo.hardwareType = IS_HARDWARE_TYPE_IMX;
                    devInfo.hardwareVer[0] = 5;
                    devInfo.hardwareVer[1] = 0;
                    break;
                case IS_PROCESSOR_STM32U5:
                    // GPX-1
                    devInfo.hardwareType = IS_HARDWARE_TYPE_GPX; // OR IMX-5.1
                    devInfo.hardwareVer[0] = 1;
                    devInfo.hardwareVer[1] = 0;
                    break;
                case IS_PROCESSOR_NUM:
                    break;
            }
            // m_isb_props.is_evb = buf[6];
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo) ;
            hdwRunState = ISDevice::HDW_STATE_BOOTLOADER;
            memcpy(&devInfo.serialNumber, &buf[7], sizeof(uint32_t));
            return true;
        }
    }

    hdwId = IS_HARDWARE_TYPE_UNKNOWN;
    hdwRunState = ISDevice::HDW_STATE_UNKNOWN;
    return false;
}

bool ISDevice::queryDeviceInfoDFU() {
    return false;
}

bool ISDevice::queryDeviceInfo() {
    if ((serialPort.errorCode != ENOENT) && serialPortIsOpen(&serialPort)){
        if (queryDeviceInfoISB() == true)
            return true;
    }
    return false;
}
