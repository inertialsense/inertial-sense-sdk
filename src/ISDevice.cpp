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
#include "imx_defaults.h"
#include "ISLogger.h"

ISDevice ISDevice::invalidRef;

/**
 * Steps the communications for this device, sending any scheduled requests and parsing any received data on the device's associated port (if connected).
 * @return
 */
bool ISDevice::step() {
    if (!port) return false;

    comManagerStep(port);
    if (fwUpdater)
        fwUpdate();
    return true;
}

is_operation_result ISDevice::updateFirmware(
        fwUpdate::target_t targetDevice,
        std::vector<std::string> cmds,
        ISBootloader::pfnBootloadProgress uploadProgress,
        ISBootloader::pfnBootloadProgress verifyProgress,
        ISBootloader::pfnBootloadStatus infoProgress,
        void (*waitAction)()
)
{
    fwUpdater = new ISFirmwareUpdater(*this);
    fwUpdater->setTarget(targetDevice);

    // TODO: Implement maybe
    fwUpdater->setUploadProgressCb(uploadProgress);
    fwUpdater->setVerifyProgressCb(verifyProgress);
    fwUpdater->setInfoProgressCb(infoProgress);

    fwUpdater->setCommands(cmds);

    return IS_OP_OK;
}

/**
 * @return true if this device is in the process of being updated, otherwise returns false.
 * False is returned regardless of whether the update was successful or not.
 */
bool ISDevice::fwUpdateInProgress() { return (fwUpdater && !fwUpdater->fwUpdate_isDone()); }

/**
 * Instructs the device to continue performing its actions.  This should be called regularly to ensure that the update process
 * does not stall.
 * @return true if the update is still in progress (calls inProgress()), or false if the update is finished and no further updates are needed.
 */
bool ISDevice::fwUpdate() {
    if (fwUpdater) {
        fwUpdater->fwUpdate_step();

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
                if (!fwHasError && fwUpdater && ((fwUpdater->fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED) || fwUpdater->hasErrors())) {
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
            if (fwUpdater->hasErrors()) {
                fwHasError = fwUpdater->hasErrors();
                fwLastMessage = "Error: ";
                fwLastMessage += "One or more step errors occurred.";
            } else if (!fwHasError) {
                fwLastMessage = "Error: ";
                fwLastMessage += ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwLastStatus);
            } else {
                fwLastMessage = "Completed successfully.";
            }
        }

        // cleanup if we're done.
        bool is_done = fwUpdater->fwUpdate_isDone();
        if (is_done) {
            // collect errors before we close out the updater
            fwErrors = fwUpdater->getStepErrors();
            fwHasError |= !fwErrors.empty();

            delete fwUpdater;
            fwUpdater = nullptr;
        }
    } else {
        fwPercent = 0.0;
    }

    return fwUpdateInProgress();
}

bool ISDevice::handshakeISbl() {
    static const uint8_t handshakerChar = 'U';

    // Bootloader sync requires at least 6 'U' characters to be sent every 10ms.
    // write a 'U' to handshake with the bootloader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_RETRIES; i++) {
        if (serialPortWrite(port, &handshakerChar, 1) != 1) {
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

        if (buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
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
std::string ISDevice::getIdAsString(const dev_info_t& devInfo) {
    const char *typeName = "\?\?\?";
    switch (devInfo.hardwareType) {
        case IS_HARDWARE_TYPE_UINS: typeName = "uINS"; break;
        case IS_HARDWARE_TYPE_IMX: typeName = "IMX"; break;
        case IS_HARDWARE_TYPE_GPX: typeName = "GPX"; break;
        default: typeName = "\?\?\?"; break;
    }
    return utils::string_format("%s-%d.%d::SN%ld", typeName, devInfo.hardwareVer[0], devInfo.hardwareVer[1], devInfo.serialNumber);
}

std::string ISDevice::getIdAsString() {
    return getIdAsString(devInfo);
}

std::string ISDevice::getName(const dev_info_t &devInfo) {
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

std::string ISDevice::getName() {
    return getName(devInfo);
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

std::string ISDevice::getFirmwareInfo(const dev_info_t& devInfo, int detail, eHdwRunStates hdwRunState) {
    std::string out;

    if (hdwRunState == eHdwRunStates::HDW_STATE_BOOTLOADER) {
        out += utils::string_format("ISbl.v%u%c **BOOTLOADER**", devInfo.firmwareVer[0], devInfo.firmwareVer[1]);
    } else {
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
    }

    return out;
}

std::string ISDevice::getFirmwareInfo(int detail) {
    return getFirmwareInfo(devInfo, detail, hdwRunState);
}

std::string ISDevice::getDescription() {
    return utils::string_format("%-12s [ %s : %-14s ]", getName().c_str(), getFirmwareInfo(1).c_str(), portName(port));
}

void ISDevice::registerWithLogger(cISLogger *logger) {
    if (logger) {
        logger->registerDevice(this);
    }
}

bool ISDevice::BroadcastBinaryData(uint32_t dataId, int periodMultiple)
{
    if (!port)
        return false;

    if (devInfo.protocolVer[0] != PROTOCOL_VERSION_CHAR0)
        return false;   // TODO: Not sure if we really need this here.  We should be doing a broader level check for protocol compatibility either at a higher level, preventing us from getting here in the first place
                        //   Or at a lower-level, like in the comMangerGetData() call that does this check for everything.

    if (periodMultiple < 0) {
        comManagerDisableData(port, dataId);
    } else {
        comManagerGetData(port, dataId, 0, 0, periodMultiple);
    }
    return true;
}

[[deprecated]]
bool ISDevice::verifyFlashConfigUpload() {
    bool success = false;
    unsigned int startTimeMs = current_timeMs();

    StopBroadcasts(false);  // only stop broadcasts to this port... FIXME: Do we REALLY want to do this??  This will stop legitimately configured broadcasts
    do {
        // Setup communications
        BroadcastBinaryData(DID_FLASH_CONFIG, 0);
        BroadcastBinaryData(DID_SYS_PARAMS, 0);
        SLEEP_MS(200);
        printf(".");
        fflush(stdout);

        comManagerStep(port);
        success = (memcmp(&(flashCfg), &(flashCfgUpload), sizeof(nvm_flash_cfg_t)) == 0); // Flash config matches
    } while (!success && ((current_timeMs() - startTimeMs) < 5000));
    return success;
}

void ISDevice::SetSysCmd(const uint32_t command) {
    sysCmd.command = command;
    sysCmd.invCommand = ~command;
    // [C COMM INSTRUCTION]  Update the entire DID_SYS_CMD data set in the IMX.
    comManagerSendData(port, &sysCmd, DID_SYS_CMD, sizeof(system_command_t), 0);
}

void ISDevice::SendNmea(const std::string& nmeaMsg)
{
    uint8_t buf[1024] = {0};
    int n = 0;
    if (nmeaMsg[0] != '$') buf[n++] = '$'; // Append header if missing
    memcpy(&buf[n], nmeaMsg.c_str(), nmeaMsg.size());
    n += nmeaMsg.size();
    nmea_sprint_footer((char*)buf, sizeof(buf), n);
    SendRaw(buf, n);
}


/**
 * Sends message to device to set devices Event Filter
 * param Target: 0 = device,
 *               1 = forward to device GNSS 1 port (ie GPX),
 *               2 = forward to device GNSS 2 port (ie GPX),
 *               else will return
 *       port: Send in target COM port.
 *                If arg is < 0 default port will be used
*/
void ISDevice::SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel)
{
    #define EVENT_MAX_SIZE (1024 + DID_EVENT_HEADER_SIZE)
    uint8_t data[EVENT_MAX_SIZE] = {0};

    did_event_t event = {
            .time = 123,
            .senderSN = 0,
            .senderHdwId = 0,
            .length = sizeof(did_event_filter_t),
    };

    did_event_filter_t filter = {
            .portMask = portMask,
    };

    filter.eventMask.priorityLevel = priorityLevel;
    filter.eventMask.msgTypeIdMask = msgTypeIdMask;

    if (target == 0)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_FILTER;
    else if (target == 1)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_GNSS1_FILTER;
    else if (target == 2)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_GNSS2_FILTER;
    else
        return;

    memcpy(data, &event, DID_EVENT_HEADER_SIZE);
    memcpy((void*)(data+DID_EVENT_HEADER_SIZE), &filter, _MIN(sizeof(did_event_filter_t), EVENT_MAX_SIZE-DID_EVENT_HEADER_SIZE));

    SendData(DID_EVENT, data, DID_EVENT_HEADER_SIZE + event.length, 0);
}

// This method uses DID_SYS_PARAMS.flashCfgChecksum to determine if the local flash config is synchronized.
void ISDevice::SyncFlashConfig(unsigned int timeMs) {

    if (timeMs - flashSyncCheckTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS) {
        return;
    }
    flashSyncCheckTimeMs = timeMs;

    if (flashCfgUploadTimeMs) {
        // Upload in progress
        if (timeMs - flashCfgUploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS) {
            // Wait for upload to process.  Pause sync.
            sysParams.flashCfgChecksum = 0;
        }
    }

    // Require valid sysParams checksum
    if (sysParams.flashCfgChecksum) {
        if (sysParams.flashCfgChecksum == flashCfg.checksum) {
            if (flashCfgUploadTimeMs) {
                // Upload complete.  Allow sync.
                flashCfgUploadTimeMs = 0;

                if (flashCfgUploadChecksum == sysParams.flashCfgChecksum) {
                    printf("DID_FLASH_CONFIG upload complete.\n");
                } else {
                    printf("DID_FLASH_CONFIG upload rejected.\n");
                }
            }
        } else {
            // Out of sync.  Request flash config.
            printf("Out of sync.  Requesting DID_FLASH_CONFIG...\n");
            comManagerGetData(port, DID_FLASH_CONFIG, 0, 0, 0);
        }
    }
}

void ISDevice::UpdateFlashConfigChecksum(nvm_flash_cfg_t &flashCfg_)
{
    bool platformCfgUpdateIoConfig = flashCfg_.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;

    // Exclude from the checksum update the following which does not get saved in the flash config
    flashCfg_.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;

    if (platformCfgUpdateIoConfig)
    {   // Update ioConfig
        imxPlatformConfigToFlashCfgIoConfig(&flashCfg_.ioConfig, flashCfg_.platformConfig);
    }

    // Update checksum
    flashCfg_.checksum = flashChecksum32(&flashCfg, sizeof(nvm_flash_cfg_t));
}

/**
 * Populates the passed reference to a nvm_flash_cfg_t struct with the contents of this device's last known flash config
 * @param flashCfg
 * @return true if flashCfg was populated, and the flash checksum matches the remote device's checksum (they are synchronized).
 *    False indicates that the resulting flash config cannot be trusted due to an inability to confirm synchronization
 */
bool ISDevice::FlashConfig(nvm_flash_cfg_t& flashCfg_)
{
    // Copy flash config
    flashCfg_ = ISDevice::flashCfg;

    // Indicate whether the port connection is valid, open, and the flash config is synchronized; otherwise false
    return (port && serialPortIsOpen(port) && (sysParams.flashCfgChecksum == flashCfg.checksum));
}

bool ISDevice::SetFlashConfig(nvm_flash_cfg_t& flashCfg_) {
    static_assert(sizeof(nvm_flash_cfg_t) % 4 == 0, "Size of nvm_flash_cfg_t must be a 4 bytes in size!!!");
    uint32_t *newCfg = (uint32_t*)&flashCfg_;
    uint32_t *curCfg = (uint32_t*)&(flashCfg);
    int iSize = sizeof(nvm_flash_cfg_t) / 4;
    bool failure = false;

    bool platformCfgUpdateIoConfig = flashCfg.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;

    // Exclude updateIoConfig bit from flash config and keep track of it separately so it does not affect whether the platform config gets uploaded
    flashCfg.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;

    flashCfg.checksum = flashChecksum32(&flashCfg, sizeof(nvm_flash_cfg_t));

    // Iterate over and upload flash config in 4 byte segments.  Upload only contiguous segments of mismatched data starting at `key` (i = 2).  Don't upload size or checksum.
    for (int i = 2; i < iSize; i++) {
        // Start with index 2 to exclude size and checksum
        if (newCfg[i] != curCfg[i]) {
            // Found start
            uint8_t *head = (uint8_t*)&(newCfg[i]);

            // Search for end
            for (; i < iSize && newCfg[i] != curCfg[i]; i++);

            // Found end
            uint8_t *tail = (uint8_t*)&(newCfg[i]);
            int size = tail-head;
            int offset = head-((uint8_t*)newCfg);

            if (platformCfgUpdateIoConfig &&
                (head <= (uint8_t*)&(flashCfg_.platformConfig)) &&
                (tail >  (uint8_t*)&(flashCfg_.platformConfig))) {
                // Re-apply updateIoConfig bit prior to upload
                flashCfg_.platformConfig |= PLATFORM_CFG_UPDATE_IO_CONFIG;
            }

            const data_info_t* fieldInfo = cISDataMappings::FieldInfoByOffset(DID_FLASH_CONFIG, offset);
            printf("%s :: Sending DID_FLASH_CONFIG.%s (offset %d, size %d)\n", getIdAsString().c_str(), (fieldInfo ? fieldInfo->name.c_str() : "<UNKNOWN>"), offset, size);
            int fail = comManagerSendData(port, head, DID_FLASH_CONFIG, size, offset);
            failure = failure || fail;
            flashCfgUploadTimeMs = current_timeMs();        // non-zero indicates upload in progress
        }
    }

    flashCfgUpload = flashCfg_;
    if (flashCfgUploadTimeMs != 0) {
        // Update checksum
        UpdateFlashConfigChecksum(flashCfg_);

        // Save checksum to ensure upload happened correctly
        if (flashCfgUploadTimeMs) {
            flashCfgUploadChecksum = flashCfg_.checksum;
        }
    } else {
        // printf("DID_FLASH_CONFIG in sync.  No upload.\n");
    }

    // Update local copy of flash config
    flashCfg = flashCfg_;

    // Success
    return !failure;
}

bool ISDevice::WaitForFlashSynced()
{
    if (!port)
        return false;   // No device, no flash-sync

    unsigned int startMs = current_timeMs();
    while (!FlashConfigSynced())
    {   // Request and wait for flash config
        comManagerStep(port);

        if ((current_timeMs() - startMs) > SYNC_FLASH_CFG_TIMEOUT_MS)
        {   // Timeout waiting for flash config
            #if PRINT_DEBUG
            printf("Timeout waiting for DID_FLASH_CONFIG failure!\n");

            ISDevice& device = m_comManagerState.devices[pHandle];
            printf("device.flashCfg.checksum:          %u\n", device.flashCfg.checksum);
            printf("device.sysParams.flashCfgChecksum: %u\n", device.sysParams.flashCfgChecksum);
            printf("device.flashCfgUploadTimeMs:       %u\n", device.flashCfgUploadTimeMs);
            printf("device.flashCfgUploadChecksum:     %u\n", device.flashCfgUploadChecksum);
            #endif
            return false;
        }
        else
        {   // Query DID_SYS_PARAMS
            GetData(DID_SYS_PARAMS);
            // printf("Waiting for flash sync...\n");
        }
        SLEEP_MS(20);  // give some time for the device to respond.
    }

    return FlashConfigSynced();
}

/**
 * @return true if there are "PENDING FLASH WRITES" waiting to clear, or no response from device.
 */
bool ISDevice::hasPendingFlashWrites(uint32_t& ageSinceLastPendingWrite) {
    if (!port || !serialPortIsOpen(port))
        return false;

    return ((sysParams.hdwStatus & HDW_STATUS_FLASH_WRITE_PENDING) || (sysParams.hdwStatus == 0));
}

bool ISDevice::reset() {
    if (current_timeMs() > nextResetTime) {
        for (int i = 0; i < 3; i++) {
            SetSysCmd(SYS_CMD_SOFTWARE_RESET);
            SLEEP_MS(10);
        }
        nextResetTime = current_timeMs() + resetRequestThreshold;
        return true;
    }
    return false;
}