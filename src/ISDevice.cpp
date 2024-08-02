/**
 * @file ISDevice.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDevice.h"
#include "ISBootloaderBase.h"
#include "ISFirmwareUpdater.h"
#include "util/util.h"

ISDevice ISDevice::invalidRef;

bool ISDevice::fwUpdateInProgress() { return (fwUpdater && !fwUpdater->fwUpdate_isDone()); }

void ISDevice::fwUpdate() {
    if (fwUpdater) {
        if (fwUpdater->getActiveCommand() == "upload") {
            if (fwUpdater->fwUpdate_getSessionTarget() != fwLastTarget) {
                fwHasError = false;
                fwLastStatus = fwUpdate::NOT_STARTED;
                fwLastMessage.clear();
                fwLastTarget = fwUpdater->fwUpdate_getSessionTarget();
            }
            fwLastSlot = fwUpdater->fwUpdate_getSessionImageSlot();

            if ((fwUpdater->fwUpdate_getSessionStatus() == fwUpdate::NOT_STARTED) && fwUpdater->isWaitingResponse()) {
                // We're just starting (no error yet, but no response either)
                fwLastStatus = fwUpdate::INITIALIZING;
                fwLastMessage = ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwLastStatus);
            } else if ((fwUpdater->fwUpdate_getSessionStatus() != fwUpdate::NOT_STARTED) && (fwLastStatus != fwUpdater->fwUpdate_getSessionStatus())) {
                // We're got a valid status update (error or otherwise)
                fwLastStatus = fwUpdater->fwUpdate_getSessionStatus();
                fwLastMessage = ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwLastStatus);

                // check for error
                if (!fwHasError && fwUpdater && fwUpdater->fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED) {
                    fwHasError = true;
                }
            }

            // update our upload progress
            if ((fwLastStatus == fwUpdate::IN_PROGRESS)) {
                fwPercent = ((float) fwUpdater->fwUpdate_getNextChunkID() / (float) fwUpdater->fwUpdate_getTotalChunks()) * 100.f;
            } else {
                fwPercent = fwLastStatus <= fwUpdate::READY ? 0.f : 100.f;
            }
        } else if (fwUpdater->getActiveCommand() == "waitfor") {
            fwLastMessage = "Waiting for response from device.";
        } else if (fwUpdater->getActiveCommand() == "reset") {
            fwLastMessage = "Resetting device.";
        } else if (fwUpdater->getActiveCommand() == "delay") {
            fwLastMessage = "Waiting...";
        }

        if (!fwUpdater->hasPendingCommands()) {
            if (!fwHasError) {
                fwLastMessage = "Completed successfully.";
            } else {
                fwLastMessage = "Error: ";
                fwLastMessage += ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwLastStatus);
            }
        }

        // cleanup if we're done.
        if (fwUpdater->fwUpdate_isDone()) {
            delete fwUpdater;
            fwUpdater = nullptr;
        }
    } else {
        fwPercent = 0.0;
    }
}

bool ISDevice::handshakeISbl() {
    static const uint8_t handshakerChar = 'U';

    // Bootloader sync requires at least 6 'U' characters to be sent every 10ms.
    // write a 'U' to handshake with the bootloader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_RETRIES; i++) {
        if(serialPortWrite(port, &handshakerChar, 1) != 1) {
            return false;
        }

        if (serialPortWaitForTimeout(port, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY)) {
            return true;           // Success
        }
    }

    return false;
}

bool ISDevice::queryDeviceInfoISbl() {
    uint8_t buf[64] = {};

    handshakeISbl();     // We have to handshake before we can do anything... if we've already handshaked, we won't go a response, so ignore this result

    serialPortFlush(port);
    serialPortRead(port, buf, sizeof(buf));    // empty Rx buffer

    // Query device
    serialPortWrite(port, (uint8_t*)":020000041000EA", 15);

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
    int count = serialPortReadTimeout(port, buf, 14, 1000);
    if (count >= 8 && buf[0] == 0xAA && buf[1] == 0x55)
    {   // expected response
        devInfo.firmwareVer[0] = buf[2];
        devInfo.firmwareVer[1] = buf[3];
        // m_isb_props.rom_available = buf[4];

        if(buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
        {
            switch ((ISBootloader::eProcessorType)buf[5]) {
                case ISBootloader::IS_PROCESSOR_UNKNOWN:
                    devInfo.hardwareType = IS_HARDWARE_TYPE_UNKNOWN;
                    break;
                case ISBootloader::IS_PROCESSOR_SAMx70:
                    devInfo.hardwareType = IS_HARDWARE_TYPE_EVB;
                    break;
                case ISBootloader::IS_PROCESSOR_STM32L4:
                    // IMX-5.0
                    devInfo.hardwareType = IS_HARDWARE_TYPE_IMX;
                    devInfo.hardwareVer[0] = 5;
                    devInfo.hardwareVer[1] = 0;
                    break;
                case ISBootloader::IS_PROCESSOR_STM32U5:
                    // GPX-1
                    devInfo.hardwareType = IS_HARDWARE_TYPE_GPX; // OR IMX-5.1
                    devInfo.hardwareVer[0] = 1;
                    devInfo.hardwareVer[1] = 0;
                    break;
                case ISBootloader::IS_PROCESSOR_NUM:
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

/**
 * Generates an "ID" that can be used to match in the configuration of the format: [hwType-hwVer]::SN[serialNo]
 * This is used for me programatic purposes requiring a unique identifier, but still somewhat human-readable.
 * If you are looking for a "friendly" (ie, only for humans) name of the device, use getDeviceName().
 * @param dev
 * @return
 */
std::string ISDevice::getId() {
    const char *typeName = "\?\?\?";
    switch (DECODE_HDW_TYPE(hdwId)) {
        case IS_HARDWARE_TYPE_UINS: typeName = "uINS"; break;
        case IS_HARDWARE_TYPE_IMX: typeName = "IMX"; break;
        case IS_HARDWARE_TYPE_GPX: typeName = "GPX"; break;
        default: typeName = "\?\?\?"; break;
    }
    return utils::string_format("%s-%d.%d::SN%ld", typeName, DECODE_HDW_MAJOR(hdwId), DECODE_HDW_MINOR(hdwId), devInfo.serialNumber);
}

std::string ISDevice::getName() {
    std::string out;

    // device serial no
    out += utils::string_format("SN%09d (", devInfo.serialNumber);

    // hardware type & version
    const char *typeName = "\?\?\?";
    switch (devInfo.hardwareType) {
        case IS_HARDWARE_TYPE_UINS: typeName = "uINS"; break;
        case IS_HARDWARE_TYPE_IMX: typeName = "IMX"; break;
        case IS_HARDWARE_TYPE_GPX: typeName = "GPX"; break;
        default: typeName = "\?\?\?"; break;
    }
    out += utils::string_format("%s-%u.%u", typeName, devInfo.hardwareVer[0], devInfo.hardwareVer[1]);
    if ((devInfo.hardwareVer[2] != 0) || (devInfo.hardwareVer[3] != 0)) {
        out += utils::string_format(".%u", devInfo.hardwareVer[2]);
        if (devInfo.hardwareVer[3] != 0)
            out += utils::string_format(".%u", devInfo.hardwareVer[3]);
    }
    out += ")";
    // return utils::string_format("%s-%d.%d::SN%ld", typeName, dev.devInfo.hardwareVer[0], dev.devInfo.hardwareVer[1], dev.devInfo.serialNumber);
    return out;
}

/**
 * Generates a single string representing the firmware version & build information for this specified device.
 * @param dev the dev_data_s device for which to format the version info
 * @param detail an indicator for the amount of detail that should be provided in the resulting string.
 *      a value of 0 will output only the firmware version only.
 *      a value of 1 will output the firmware version and build number.
 *      a value of 2 will output the firmware version, build number, and build date/time.
 * @return the resulting string
 */
std::string ISDevice::getFirmwareInfo(int detail) {
    std::string out;

    // firmware version
    out += utils::string_format("fw%u.%u.%u", devInfo.firmwareVer[0], devInfo.firmwareVer[1], devInfo.firmwareVer[2]);
    switch(devInfo.buildType) {
        case 'a': out +="-alpha";       break;
        case 'b': out +="-beta";        break;
        case 'c': out +="-rc";          break;
        case 'd': out +="-devel";       break;
        case 's': out +="-snap";        break;
        case '*': out +="-snap";        break;
        default : out +="";             break;
    }
    if (devInfo.firmwareVer[3] != 0)
        out += utils::string_format(".%u", devInfo.firmwareVer[3]);

    if (detail > 0) {
        out += utils::string_format(" %08x", devInfo.repoRevision);
        if (devInfo.buildType == '*') {
            out += "*";
        }

        if (detail > 1) {
            // build number/type
            out += utils::string_format(" b%05x.%d", ((devInfo.buildNumber >> 12) & 0xFFFFF), (devInfo.buildNumber & 0xFFF));

            // build date/time
            out += utils::string_format(" %04u-%02u-%02u", devInfo.buildYear + 2000, devInfo.buildMonth, devInfo.buildDay);
            out += utils::string_format(" %02u:%02u:%02u", devInfo.buildHour, devInfo.buildMinute, devInfo.buildSecond);
            if (devInfo.buildMillisecond)
                out += utils::string_format(".%03u", devInfo.buildMillisecond);
        }
    }

    //return utils::string_format("fw%d.%d.%d %3d%c", devInfo.firmwareVer[0], devInfo.firmwareVer[1], devInfo.firmwareVer[2], devInfo.buildNumber, devInfo.buildType);
    return out;
}

std::string ISDevice::getDescription() {
    return utils::string_format("%-12s [ %s : %-14s ]", getName().c_str(), getFirmwareInfo(1).c_str(), portName(port));
}

