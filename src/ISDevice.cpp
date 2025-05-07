/**
 * @file ISDevice.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "core/msg_logger.h"
#include "ISDevice.h"
#include "ISBootloaderBase.h"
#include "ISFirmwareUpdater.h"
#include "util/util.h"
#include "imx_defaults.h"
#include "ISLogger.h"

ISDevice ISDevice::invalidRef;

/**
 * General Purpose IS-binary protocol handler for the InertialSense class.
 * This is called anytime ISB packets are received by any of the underlying ports
 * which are managed by the InertialSense and CommManager classes.  Eventually
 * this should be moved into the ISDevice class, where devices of different types
 * can handle their data independently. There could be a hybrid approach here
 * where this function would (should?) locate the ISDevice by its port, and then
 * redirect to the ISDevice specific callback.
 * @param data The data which was parsed and is ready to be consumed
 * @param port The port which the data was received from
 * @return 0 if this data packet WILL NOT BE processed again by other handlers;
 *   any other value indicates that the packet MAY BE processed by other handlers.
 *   No guarantee is given that other handlers will process this packet if the
 *   return value is non-zero, but IS GUARANTEED that this packet WILL NOT BE
 *   further processed if a zero-value is returned.  Effectively, 0 = End-of-line.
 */
int ISDevice::processIsbMsgs(void* ctx, p_data_t* data, port_handle_t port)
{
    ISDevice* device = (ISDevice*)ctx;
    //device->stepLogger(ctx, data, port);
    return (device && device->port == port) ? device->onIsbDataHandler(data, port) : -1;
}

int ISDevice::processNmeaMsgs(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port)
{
    ISDevice* device = (ISDevice*)ctx;
    return (device && device->port == port) ? device->onNmeaHandler(msg, msgSize, port) : -1;
}

int ISDevice::processPacket(void *ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port) {
    ISDevice* device = (ISDevice*)ctx;
    return (device && device->port == port) ? device->onPacketHandler(ptype, pkt, port) : -1;
}

bool ISDevice::Update() {
    return step();
}

/**
 * Steps the communications for this device, sending any scheduled requests and parsing any received data on the device's associated port (if connected).
 * @return
 */
bool ISDevice::step() {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!port || !portIsValid(port))
        return false;

    if (portType(port) & PORT_TYPE__COMM)
        is_comm_port_parse_messages(port); // Read data directly into comm buffer and call callback functions

    if (!hasDeviceInfo()) {
        validateDeviceAlt(30000);
    } else if (fwUpdater) {
        fwUpdate();
    } else {
        if (sysParams.flashCfgChecksum == 0)
            GetData(DID_SYS_PARAMS);
        if (flashCfg.checksum == 0)
            GetData(DID_FLASH_CONFIG);

        SyncFlashConfig();
    }

    return true;
}

is_operation_result ISDevice::updateFirmware(fwUpdate::target_t targetDevice, std::vector<std::string> cmds, fwUpdate::pfnStatusCb infoProgress, void (*waitAction)()) {
    fwUpdater = new ISFirmwareUpdater(this);
    fwUpdater->setInfoProgressCb(infoProgress);
    fwUpdater->setTarget(targetDevice);
    fwUpdater->setCommands(cmds);

    return IS_OP_OK;
}

/**
 * @return true if this device is in the process of being updated, otherwise returns false.
 * False is returned regardless of whether the update was successful or not.
 */
bool ISDevice::fwUpdateInProgress() { return (fwUpdater && !fwUpdater->fwUpdate_isDone()); }

/**
 * @return as percentage (0-1.0) the completion progress for the current fwUpdate, or 0.0 if not update is in progress.
 */
float ISDevice::fwUpdatePercentCompleted() { return (fwUpdater && !fwUpdater->fwUpdate_isDone()) ? fwUpdater->fwUpdate_getProgressPercent() : 0.0f; }

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

    for (int i = 0; i < 5; i++) {
        serialPortWrite(port, (uint8_t*)"\n", 1);
        SLEEP_MS(10);
        serialPortFlush(port);
        serialPortRead(port, buf, sizeof(buf));    // empty Rx buffer
    }

    // Query device
    serialPortWrite(port, (uint8_t*)":020000041000EA", 18);

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
            devInfo.hdwRunState = HDW_STATE_BOOTLOADER;
            memcpy(&devInfo.serialNumber, &buf[7], sizeof(uint32_t));
            return true;
        }
    }

    hdwId = IS_HARDWARE_TYPE_UNKNOWN;
    devInfo = {};
    return false;
}


bool ISDevice::validateDevice(uint32_t timeout) {
    unsigned int startTime = current_timeMs();

    if (!isConnected())
        return false;

    // check for Inertial-Sense App by making an NMEA request (which it should respond to)
    do {
        if (SERIAL_PORT(port)->errorCode == ENOENT)
            return false;

        switch (previousQueryType) {
            case QUERYTYPE_NMEA:
                SendRaw((uint8_t *) NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE);
                break;
            case QUERYTYPE_ISB:
                SendRaw((uint8_t *) DID_DEV_INFO, sizeof(dev_info_t));
                break;
            case QUERYTYPE_ISbootloader:
                queryDeviceInfoISbl();
                break;
            case QUERYTYPE_mcuBoot:
                break;

        }
        // there was some other janky issue with the requested port; even though the device technically exists, its in a bad state. Let's just drop it now.
        if (SERIAL_PORT(port)->errorCode != 0)
            return false;

        SLEEP_MS(100);
        step();

        if ((current_timeMs() - startTime) > timeout)
            return false;

        previousQueryType = static_cast<queryTypes>((int)previousQueryType + 1 % (int)QUERYTYPE_MAX);
    } while ((hdwId != IS_HARDWARE_TYPE_UNKNOWN) && (devInfo.serialNumber != 0));

    if ((hdwId != IS_HARDWARE_TYPE_UNKNOWN) &&
        (devInfo.hdwRunState != HDW_STATE_UNKNOWN) &&
        (devInfo.protocolVer[0] == PROTOCOL_VERSION_CHAR0)) {
        comManagerGetData(this, DID_SYS_CMD, 0, 0, 0);
        comManagerGetData(this, DID_FLASH_CONFIG, 0, 0, 0);
        comManagerGetData(this, DID_EVB_FLASH_CFG, 0, 0, 0);
    }

    return true;
}

/**
 * Non-blocking, internal(ish) method to validate a device.  Will be called repeatedly from step() as long as "isValidating" is true.
 * @param timeout the maximum number of milliseconds that must pass without a validating response from the device, before giving up.
 * @return true if the device has been validated, otherwise false
 */
bool ISDevice::validateDeviceAlt(uint32_t timeout) {
    if (!isConnected())
        return false;

    if (hasDeviceInfo()) {
        validationStartMs = 0;
        return true;
    }

    // uint64_t nanos = current_timeUs();

    if (!validationStartMs) {
        validationStartMs = current_timeMs();
    }

    // doing the timeout check first helps during debugging (since stepping through code will likely trigger the timeout.
    if ((current_timeMs() - validationStartMs) > timeout) {
        debug_message("ISDevice::validateDeviceAlt() timed out after %dms.\n", current_timeMs() - validationStartMs);
        validationStartMs = 0;
        return false;
    }

    switch (previousQueryType) {
        case ISDevice::queryTypes::QUERYTYPE_NMEA :
            // debug_message("[DBG] Querying serial port '%s' using NMEA protocol.\n", SERIAL_PORT(port)->portName);
            // comManagerSendRaw(port, (uint8_t *) NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE);
            SendNmea(NMEA_CMD_QUERY_DEVICE_INFO);
            break;
        case ISDevice::queryTypes::QUERYTYPE_ISB :
            // debug_message("[DBG] Querying serial port '%s' using ISB protocol.\n", SERIAL_PORT(port)->portName);
            // comManagerGetData(port, DID_DEV_INFO, 0, 0, 0);
            GetData(DID_DEV_INFO);
            break;
        case ISDevice::queryTypes::QUERYTYPE_ISbootloader :
            // debug_message("[DBG] Querying serial port '%s' using ISbootloader protocol.\n", SERIAL_PORT(port)->portName);
            queryDeviceInfoISbl();
            break;
        case ISDevice::queryTypes::QUERYTYPE_mcuBoot :
            // debug_message("[DBG] Querying serial port '%s' mcuBoot/SMP protocol.\n", SERIAL_PORT(port)->portName);
            // comManagerGetData(port, DID_DEV_INFO, 0, 0, 0);
            break;
    }

//    uint64_t dt = current_timeUs() - nanos;
//    if (dt > 20000)
//        debug_message("ISDevice::validateDeviceAlt() executed for %ld nanos, for device %s.\n", current_timeUs() - nanos, getDescription().c_str());

    SLEEP_MS(5); // give just enough time for the device to receive, process and respond to the query

    previousQueryType = static_cast<queryTypes>(((int)previousQueryType + 1) % (int)QUERYTYPE_MAX);
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
    // device serial no
    std::string out = utils::string_format("SN%09d (", devInfo.serialNumber);

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

std::string ISDevice::getFirmwareInfo(const dev_info_t &devInfo, int detail) {
    std::string out;

    if (devInfo.hdwRunState == eHdwRunStates::HDW_STATE_BOOTLOADER) {
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
            case '^': out +="-snap";        break;
            default : out +="";             break;
        }
        if (devInfo.firmwareVer[3] != 0)
            out += utils::string_format(".%u", devInfo.firmwareVer[3]);

        if (detail > 0) {
            out += utils::string_format(" %08x", devInfo.repoRevision);
            if (devInfo.buildType == '^') {
                out += "^";
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
    return getFirmwareInfo(devInfo, detail);
}

std::string ISDevice::getDescription() {
    return utils::string_format("%-12s [ %-12s : %-14s ]", getName().c_str(), getFirmwareInfo(1).c_str(), getPortName().c_str());
}

void ISDevice::registerWithLogger(cISLogger *logger) {
    if (logger) {
        logger->registerDevice(this);
    }
}

/**
 * Requests that this device broadcast the requested DID are the specified period
 * @param dataId the DID to be broadcast at periodic intervals
 * @param periodMultiple the period multiple (NOT a frequency). If 0, this will request a one-shot, also effectively stopping any existing broadcasts
 * @return true if the request was successfully sent, otherwise false (ie, port invalid, invalid device, etc)
 */
bool ISDevice::BroadcastBinaryData(uint32_t dataId, int periodMultiple)
{
    if (!port)
        return false;

    if (devInfo.protocolVer[0] != PROTOCOL_VERSION_CHAR0)
        return false;   // TODO: Not sure if we really need this here.  We should be doing a broader level check for protocol compatibility either at a higher level, preventing us from getting here in the first place
                        //   Or at a lower-level, like in the comMangerGetData() call that does this check for everything.

    std::lock_guard<std::recursive_mutex> lock(portMutex);
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

    std::lock_guard<std::recursive_mutex> lock(portMutex);

    StopBroadcasts(false);  // only stop broadcasts to this port... FIXME: Do we REALLY want to do this??  This will stop legitimately configured broadcasts
    do {
        // Setup communications
        BroadcastBinaryData(DID_FLASH_CONFIG, 0);
        BroadcastBinaryData(DID_SYS_PARAMS, 0);
        SLEEP_MS(200);
        printf(".");
        fflush(stdout);

        step(); // comManagerStep(port); // FIXME:  we shouldn't be making comManager calls...
        success = (memcmp(&(flashCfg), &(flashCfgUpload), sizeof(nvm_flash_cfg_t)) == 0); // Flash config matches
    } while (!success && ((current_timeMs() - startTimeMs) < 5000));
    return success;
}

int ISDevice::SetSysCmd(const uint32_t command) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    sysCmd.command = command;
    sysCmd.invCommand = ~command;
    // [C COMM INSTRUCTION]  Update the entire DID_SYS_CMD data set in the IMX.
    debug_message("Issuing SYS_CMD %d to %s (%s)\n", command, getIdAsString().c_str(), getPortName().c_str());
    return comManagerSendData(port, &sysCmd, DID_SYS_CMD, sizeof(system_command_t), 0);
}

int ISDevice::SendNmea(const std::string& nmeaMsg)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    uint8_t buf[1024] = {0};
    int n = 0;
    if (nmeaMsg[0] != '$') buf[n++] = '$'; // Append header if missing
    memcpy(&buf[n], nmeaMsg.c_str(), nmeaMsg.size());
    n += nmeaMsg.size();
    nmea_sprint_footer((char*)buf, sizeof(buf), n);
    return SendRaw(buf, n);
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
int ISDevice::SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

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
        return 0;

    memcpy(data, &event, DID_EVENT_HEADER_SIZE);
    memcpy((void*)(data+DID_EVENT_HEADER_SIZE), &filter, _MIN(sizeof(did_event_filter_t), EVENT_MAX_SIZE-DID_EVENT_HEADER_SIZE));

    return SendData(DID_EVENT, data, DID_EVENT_HEADER_SIZE + event.length, 0);
}


/**
 * This is a non-blocking call (intended to be called repeatedly) that checks to see if we have confirmation
 * that the FlashCfg.checksum == sysParams.flashCfgChecksum, which validates that our checksum is the same
 * in local memory as it is on the device.
 * @param timeMs a timestamp provided by the caller
 * @return true if the checksum has been verified, otherwise false.  This can be misleading, as false
 *  does not indicate a lack of communications, or a timeout, only that it has not yet been able to validate.
 *  It is the callers responsibility to handle timeout checking to determine it should give up looking for
 *  a valid sync.
 */
bool ISDevice::SyncFlashConfig() {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    unsigned int timeMs = current_timeMs();
    if (flashCfgUploadTimeMs)
    {   // if flashCfgUploadTimeMs != 0, then a UPLOAD is still in progress (we have sent a new Cfg, but waiting for it to sync back via the DID_SYS_PARAMS
        if (timeMs - flashCfgUploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
        {   // if its been fewer than SYNC_FLASH_CFG_CHECK_PERIOD_MS since we started the upload, clear the sysParams.flashCfgChecksum; We want to receive a new one to ensure its changed
            sysParams.flashCfgChecksum = 0;
            return false;
        }
    }

    if (timeMs - flashSyncCheckTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
    {   // throttle checks; if it has been less than 200ms, since the last check return false (sync still in progress)
        return false;
    }

    if (!sysParams.flashCfgChecksum)
    {   // since we cleared this, above, if flashCfgChecksum == 0, we're still waiting to receive a SYS_PARAMS
        return false;
    }

    flashSyncCheckTimeMs = timeMs;

    // Require valid sysParams checksum
    if (sysParams.flashCfgChecksum != flashCfg.checksum)
    {   // we have a sysParams.flashCfgChecksum, but IT DOES NOT SYNC with our local copy of the flashCfg, so we need to request a new copy
        GetData(DID_FLASH_CONFIG);
        return false;
    }

    // at this point, we have CONFIRMED the device's flashCfg
    if (flashCfgUpload.checksum != sysParams.flashCfgChecksum)
    { // sysParams.flashCfgChecksum DOES NOT MATCH uploaded, sync failed!
        flashCfgUploadTimeMs = 0;
        flashCfgUpload = { };
        // printf("DID_FLASH_CONFIG upload rejected.\n");
        return false;
    }

    // we have a sysParams.flashCfgChecksum and it matches the received (from the device), AND the uploaded, so SUCCESS!!
    flashCfgUploadTimeMs = 0;
    flashSyncCheckTimeMs = 0;
    flashCfgUpload = { };
    // printf("DID_FLASH_CONFIG upload complete.\n");
    return true;
}

void ISDevice::UpdateFlashConfigChecksum(nvm_flash_cfg_t &flashCfg_)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    bool platformCfgUpdateIoConfig = flashCfg_.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;

    // Exclude from the checksum update the following which does not get saved in the flash config
    flashCfg_.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;

    if (platformCfgUpdateIoConfig)
    {   // Update ioConfig
        imxPlatformConfigToFlashCfgIoConfig(&flashCfg_.ioConfig, flashCfg_.platformConfig);
    }

    // Update checksum
    flashCfg_.checksum = flashChecksum32(&flashCfg_, sizeof(nvm_flash_cfg_t));
}

/**
 * Populates the passed reference to a nvm_flash_cfg_t struct with the contents of this device's last known flash config
 * @param flashCfg
 * @return true if flashCfg was populated, and the flash checksum matches the remote device's checksum (they are synchronized).
 *    False indicates that the resulting flash config cannot be trusted due to an inability to confirm synchronization
 */
bool ISDevice::FlashConfig(nvm_flash_cfg_t& flashCfg_)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    // Copy flash config
    flashCfg_ = ISDevice::flashCfg;

    // Indicate whether the port connection is valid, open, and the flash config is synchronized; otherwise false
    return (port && serialPortIsOpen(port) && (sysParams.flashCfgChecksum == flashCfg.checksum));
}

/**
 * This uploads the provided flashCfg to the remove device, but makes no checks that it was successfully synchronized.
 * This method attempt to "intelligently" upload only the portions of the flashCfg that has actually changed, reducing
 * traffic and minimizing the risk of a sync-failure due to elements which maybe programmatically changed, however it
 * may make multiple sends, if the new and previous configurations have non-contiguous modifications.
 * Use WaitForFlashSynced() or SetFlashConfigAndConfirm() to actually confirm that the new config was applied to the
 * device correctly.
 * @param flashCfg_ the new flash_config to upload
 * @return true if the ANY of the changes failed to send to the remove device.
 */
bool ISDevice::SetFlashConfig(nvm_flash_cfg_t& flashCfg_) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    static_assert(sizeof(nvm_flash_cfg_t) % 4 == 0, "Size of nvm_flash_cfg_t must be a 4 bytes in size!!!");
    uint32_t *newCfg = (uint32_t*)&flashCfg_;
    uint32_t *curCfg = (uint32_t*)&(flashCfg);
    int iSize = sizeof(nvm_flash_cfg_t) / 4;
    bool failure = false;

    bool platformCfgUpdateIoConfig = flashCfg.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;

    // Exclude updateIoConfig bit from flash config and keep track of it separately so it does not affect whether the platform config gets uploaded
    flashCfg.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;
    flashCfg.RTKCfgBits &= ~RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER;

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
            std::string fieldName = (fieldInfo ? fieldInfo->name.c_str() : "<UNKNOWN>");
            std::string fieldValue = "??"; // (fieldInfo ? cISDataMappings::DataToString(fieldInfo, )->);
            // printf("%s :: Sending DID_FLASH_CONFIG.%s = %s (offset %d, size %d)\n", getIdAsString().c_str(), fieldName.c_str(), fieldValue.c_str(), offset, size);
            int fail = SendData(DID_FLASH_CONFIG, head, size, offset);
            failure = failure || fail;
            flashCfgUploadTimeMs = current_timeMs();        // non-zero indicates upload in progress
        }
    }

    flashCfgUpload = flashCfg_;
    if (flashCfgUploadTimeMs != 0) {
        // Update checksum
        UpdateFlashConfigChecksum(flashCfg_);
        flashCfgUpload.checksum = flashCfg.checksum;    // update the upload checksum in case it changed
    } else {
        // printf("DID_FLASH_CONFIG in sync.  No upload.\n");
    }

    // Update local copy of flash config
    flashCfg = flashCfg_;

    // Success
    return !failure;
}

/**
 * A blocking function calls which waits until both a DID_FLASH_CFG and DID_SYS_PARAMS have
 * been received which have a matching flashCfg checksum; ensuring that we have a valid copy
 * of the devices' flash configuration.
 * @param timeout
 * @return true if both the flashCfg.checksum and sysParams.flashCfgChecksum match (and neither are zero)
 */
bool ISDevice::WaitForFlashSynced(uint32_t timeout)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!port)
        return false;   // No device, no flash-sync

    // If there are no upload pending, then just go ahead and check...
    unsigned int now = current_timeMs();
    unsigned int startMs = now;

    static unsigned int lastRequest = startMs;
    if ((flashCfgUploadTimeMs == 0) && (flashCfgUpload.checksum == 0))
    {   // no upload is in progress; we don't need to verify with our uploaded flashCfg
        while (flashCfg.checksum != sysParams.flashCfgChecksum)
        {
            if ((now - startMs) > timeout)
            {   // timeout reached
                return false;
            }

            if ((current_timeMs() - lastRequest) > 50)
            {   // request both SYS_PARAMS and FLASH_CONFIG every 50ms until we get a response
                lastRequest = current_timeMs();
                GetData(DID_SYS_PARAMS);
                GetData(DID_FLASH_CONFIG);
                step();
            }
            SLEEP_MS(5);
            now = current_timeMs();
        }
        return true;
    }

    // If we are here, then we've uploaded a new flash cfg, and we are waiting to see if the device accepted or rejected it.
    while (!FlashConfigSynced())
    {   // Request and wait for flash config
        step();
        SyncFlashConfig();

        if ((now - startMs) > timeout)
        {   // Timeout waiting for flash config
            #if PRINT_DEBUG
            printf("Timeout waiting for DID_FLASH_CONFIG failure!\n");

            ISDevice& device = m_comManagerState.devices[pHandle];
            printf("device.flashCfg.checksum:          %u\n", device.flashCfg.checksum);
            printf("device.sysParams.flashCfgChecksum: %u\n", device.sysParams.flashCfgChecksum);
            printf("device.flashCfgUploadTimeMs:       %u\n", device.flashCfgUploadTimeMs);
            printf("device.flashCfgUploadChecksum:     %u\n", device.flashCfgUploadChecksum);
            #endif
            break; // we'll give it one last change to succeed between now and the return at the bottom
        }
        else
        {   // Query DID_SYS_PARAMS
            GetData(DID_SYS_PARAMS);
            SLEEP_MS(5);

            if (((now - startMs) % 500) == 0)
                SetFlashConfig(flashCfgUpload); // try and upload again.

            step();
            if (SyncFlashConfig())
                return true; // we don't need to do any additional validation
        }
        SLEEP_MS(5);  // give some time for the device to respond.
        now = current_timeMs();
    }

    return FlashConfigSynced();
}

/**
 * @return true if there are "PENDING FLASH WRITES" waiting to clear, or no response from device.
 */
bool ISDevice::hasPendingFlashWrites(uint32_t& ageSinceLastPendingWrite) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!port || !serialPortIsOpen(port))
        return false;

    return ((sysParams.hdwStatus & HDW_STATUS_FLASH_WRITE_PENDING) || (sysParams.hdwStatus == 0));
}

/**
 * Sets the provided FlashCfg and then blocks for timeout, waiting for the uploaded FlashCfg to be
 * then downloaded, before finally confirming that the new values have been set.
 * @param flashCfg the flash config to upload
 * @return
 */
bool ISDevice::SetFlashConfigAndConfirm(nvm_flash_cfg_t& flashCfg, uint32_t timeout)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!SetFlashConfig(flashCfg))  // Upload and verify upload
        return false;   // we failed to even upload the new config

    // save the uploaded config, with correct checksum calculated in SetFlashConfig()
    nvm_flash_cfg_t tmpFlash = flashCfg;

    SLEEP_MS(10);
    step();

    if (!WaitForFlashSynced(timeout))
        return false;   // Re-download flash config

    if ((flashCfgUploadTimeMs != 0) && (flashSyncCheckTimeMs != 0))
        return false;   // timed-out,

    return (memcmp(&flashCfg, &tmpFlash, sizeof(nvm_flash_cfg_t)) == 0);
}


bool ISDevice::reset() {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!portIsValid(port) || (current_timeMs() > nextResetTime)) {
        for (int i = 0; i < 3; i++) {
            SetSysCmd(SYS_CMD_SOFTWARE_RESET);
            SLEEP_MS(10);
        }
        nextResetTime = current_timeMs() + resetRequestThreshold;
        return true;
    }
    return false;
}

int ISDevice::onIsbDataHandler(p_data_t* data, port_handle_t port)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (data->hdr.size==0 || data->ptr==NULL) {
        return 0;   // We didn't process, so let others try
    }

    if (devLogger) {
        // FIXME:  devLogger->SaveData(data, 0);
        // stepLogFunction(s_cm_state->inertialSenseInterface, data, port);
    }

    // printf("DID: %d\n", data->hdr.id);
    switch (data->hdr.id) {
        case DID_DEV_INFO:
            devInfo = *(dev_info_t*)data->ptr;
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
            if (devInfo.hdwRunState == HDW_STATE_UNKNOWN)
                devInfo.hdwRunState = HDW_STATE_APP;   // since this is ISB, its pretty safe to assume that we are in APP mode.
            break;
        case DID_SYS_CMD:
            sysCmd = *(system_command_t*)data->ptr;
            break;
        case DID_SYS_PARAMS:
            copyDataPToStructP(&sysParams, data, sizeof(sys_params_t));
            DEBUG_PRINT("Received DID_SYS_PARAMS\n");
            break;
        case DID_FLASH_CONFIG:
            copyDataPToStructP(&flashCfg, data, sizeof(nvm_flash_cfg_t));
            if ( dataOverlap(offsetof(nvm_flash_cfg_t, checksum), 4, data)) {
                sysParams.flashCfgChecksum = flashCfg.checksum;
            }
            DEBUG_PRINT("Received DID_FLASH_CONFIG\n");
            break;
        case DID_FIRMWARE_UPDATE:
            // we don't respond to messages if we don't already have an active Updater
            if (fwUpdater) {
                fwUpdater->fwUpdate_processMessage(data->ptr, data->hdr.size);
                fwUpdate();
            }
            break;

        // FIXME:  Not sure what the following code is doing... It probably should not be here, and should go away.
        //  this seems to be for RTK RTCM3/NTrip Correction services, to republish the device's current position as NMEA GGA
        case DID_GPS1_POS:
            static time_t lastTime;
            time_t currentTime = time(NULLPTR);
            if (abs(currentTime - lastTime) > 5) {	// Update every 5 seconds
                lastTime = currentTime;
                gps_pos_t &gps = *((gps_pos_t*)data->ptr);
                if ((gps.status&GPS_STATUS_FIX_MASK) >= GPS_STATUS_FIX_3D) {
                    // *s_cm_state->clientBytesToSend = nmea_gga(s_cm_state->clientBuffer, s_cm_state->clientBufferSize, gps);
                }
            }
            break;
    }

    devInfo.hdwRunState = HDW_STATE_APP;  // It's basically impossible for us to receive ISB protocol, and NOT be in APP state
    return 1;   // allow others to continue to process this message
}

// return 0 on success, -1 on failure
int ISDevice::onNmeaHandler(const unsigned char* msg, int msgSize, port_handle_t port) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    switch (getNmeaMsgId(msg, msgSize))
    {
        case NMEA_MSG_ID_INFO:
            nmea_parse_info(devInfo, (const char*)msg, msgSize);
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
            devInfo.hdwRunState = HDW_STATE_APP;
        break;
    }
    return 1;   // allow others to continue to process this message
}

int ISDevice::onPacketHandler(protocol_type_t ptype, packet_t *pkt, port_handle_t port) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);
    return 1;   // allow others to continue to process this message
}

void ISDevice::stepLogger(void* ctx, const p_data_t* data, port_handle_t port)
{
/*
    InertialSense* i = &InertialSense::StepLogger();
    cMutexLocker logMutexLocker(&i->m_logMutex);
    if (i->m_logger.Enabled())
    {
        p_data_buf_t d;
        d.hdr = data->hdr;
        memcpy(d.buf, data->ptr, d.hdr.size);
        i->m_logPackets[port].push_back(d);
    }
*/
}

bool ISDevice::assignPort(port_handle_t newPort) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (port) {
        // releaseSerialPort()  TODO: I'm sure there is something we probably need to do before we can just assign the new port - close, flush, delete, etc?
    }

    port = newPort;
    if (!portIsValid(port)) {
        return false;   // nothing more to do if the port is invalid
    }

    if ((portType(newPort) & PORT_TYPE__COMM)) {
        originalCbs = COMM_PORT(newPort)->comm.cb; // make a copy of the new port's original callbacks/context, which we'll restore when this device is destroyed
    }

    defaultCbs.all = processPacket;
    registerIsbDataHandler(processIsbMsgs);
    registerProtocolHandler(_PTYPE_NMEA, processNmeaMsgs);

    is_comm_callbacks_t portCbs = defaultCbs;

    // Initialize IScomm instance, for serial reads / writes
    if ((portType(port) & PORT_TYPE__COMM)) {
        comm_port_t* comm = COMM_PORT(port);

        is_comm_init(&(comm->comm), comm->buffer, sizeof(comm->buffer), portCbs.all);
        is_comm_register_port_callbacks(port, &portCbs);

#if ENABLE_PACKET_CONTINUATION
        // Packet data continuation
        memset(&(port->con), 0, MEMBERSIZE(com_manager_port_t,con));
#endif
    }
//    if ((hdwId != IS_HARDWARE_ANY) || devInfo.serialNumber)
//        debug_message("[DBG] Device %s bound to port %s.\n", getIdAsString().c_str(), getPortName().c_str());
    return true;
}


pfnIsCommIsbDataHandler ISDevice::registerIsbDataHandler(pfnIsCommIsbDataHandler cbHandler) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    pfnIsCommIsbDataHandler oldHandler = defaultCbs.isbData;
    defaultCbs.context = this;
    defaultCbs.isbData = cbHandler;
    defaultCbs.protocolMask |= ENABLE_PROTOCOL_ISB;

    if (port && (portType(port) & PORT_TYPE__COMM)) {
        COMM_PORT(port)->comm.cb.context = this;
        oldHandler = is_comm_register_isb_handler(&COMM_PORT(port)->comm, cbHandler);
    }

    return oldHandler;
}

pfnIsCommGenMsgHandler ISDevice::registerProtocolHandler(int ptype, pfnIsCommGenMsgHandler cbHandler) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if ((ptype < _PTYPE_FIRST_DATA) || (ptype > _PTYPE_LAST_DATA))
        return NULL;

    pfnIsCommGenMsgHandler oldHandler = defaultCbs.generic[ptype];

    // if port is null, set this as the default handler, and also set it for all available ports
    defaultCbs.context = this;
    defaultCbs.generic[ptype] = cbHandler;
    defaultCbs.protocolMask |= (0x01 << ptype);  // enable the protocol  TODO: if cbHandler is NULL, this should disable the protocol

    if (port && portType(port) & PORT_TYPE__COMM) {
        COMM_PORT(port)->comm.cb.context = this;
        return is_comm_register_msg_handler(&COMM_PORT(port)->comm, ptype, cbHandler);
    }

    return oldHandler;
}

void ISDevice::SaveFlashConfigFile(std::string path)
{
    nvm_flash_cfg_t* outData = &flashCfg;

    YAML::Node map = YAML::Node(YAML::NodeType::Map);

    map["size"]             = outData->size;
    map["checksum"]         = outData->checksum;
    map["key"]              = outData->key;
    map["startupImuDtMs"]   = outData->startupImuDtMs;
    map["startupNavDtMs"]   = outData->startupNavDtMs;
    map["ser0BaudRate"]     = outData->ser0BaudRate;
    map["ser1BaudRate"]     = outData->ser1BaudRate;

    YAML::Node insRotation = YAML::Node(YAML::NodeType::Sequence);
    insRotation.push_back(outData->insRotation[0]);
    insRotation.push_back(outData->insRotation[1]);
    insRotation.push_back(outData->insRotation[2]);
    map["insRotation"]                 = insRotation;

    YAML::Node insOffset = YAML::Node(YAML::NodeType::Sequence);
    insOffset.push_back(outData->insOffset[0]);
    insOffset.push_back(outData->insOffset[1]);
    insOffset.push_back(outData->insOffset[2]);
    map["insOffset"]                 = insOffset;

    YAML::Node gps1AntOffset = YAML::Node(YAML::NodeType::Sequence);
    gps1AntOffset.push_back(outData->gps1AntOffset[0]);
    gps1AntOffset.push_back(outData->gps1AntOffset[1]);
    gps1AntOffset.push_back(outData->gps1AntOffset[2]);
    map["gps1AntOffset"]    = gps1AntOffset;

    map["dynamicModel"]     = (uint16_t)outData->dynamicModel;
    map["debug"]            = (uint16_t)outData->debug;
    map["gnssSatSigConst"]  = outData->gnssSatSigConst;
    map["sysCfgBits"]       = outData->sysCfgBits;

    YAML::Node refLla = YAML::Node(YAML::NodeType::Sequence);
    refLla.push_back(outData->refLla[0]);
    refLla.push_back(outData->refLla[1]);
    refLla.push_back(outData->refLla[2]);
    map["refLla"]                     = refLla;

    YAML::Node lastLla = YAML::Node(YAML::NodeType::Sequence);
    lastLla.push_back(outData->lastLla[0]);
    lastLla.push_back(outData->lastLla[1]);
    lastLla.push_back(outData->lastLla[2]);
    map["lastLla"]                     = lastLla;

    map["lastLlaTimeOfWeekMs"]      = outData->lastLlaTimeOfWeekMs;
    map["lastLlaWeek"]              = outData->lastLlaWeek;
    map["lastLlaUpdateDistance"]    = outData->lastLlaUpdateDistance;
    map["ioConfig"]                 = outData->ioConfig;
    map["platformConfig"]           = outData->platformConfig;

    YAML::Node gps2AntOffset = YAML::Node(YAML::NodeType::Sequence);
    gps2AntOffset.push_back(outData->gps2AntOffset[0]);
    gps2AntOffset.push_back(outData->gps2AntOffset[1]);
    gps2AntOffset.push_back(outData->gps2AntOffset[2]);
    map["gps2AntOffset"]            = gps2AntOffset;

    YAML::Node zeroVelRotation = YAML::Node(YAML::NodeType::Sequence);
    zeroVelRotation.push_back(outData->zeroVelRotation[0]);
    zeroVelRotation.push_back(outData->zeroVelRotation[1]);
    zeroVelRotation.push_back(outData->zeroVelRotation[2]);
    map["zeroVelRotation"]          = zeroVelRotation;

    YAML::Node zeroVelOffset = YAML::Node(YAML::NodeType::Sequence);
    zeroVelOffset.push_back(outData->zeroVelOffset[0]);
    zeroVelOffset.push_back(outData->zeroVelOffset[1]);
    zeroVelOffset.push_back(outData->zeroVelOffset[2]);
    map["zeroVelOffset"]            = zeroVelOffset;

    map["gpsTimeUserDelay"]         = outData->gpsTimeUserDelay;
    map["magDeclination"]           = outData->magDeclination;
    map["gpsTimeSyncPeriodMs"]      = outData->gpsTimeSyncPeriodMs;
    map["startupGPSDtMs"]           = outData->startupGPSDtMs;
    map["RTKCfgBits"]               = outData->RTKCfgBits;
    map["sensorConfig"]             = outData->sensorConfig;
    map["gpsMinimumElevation"]      = outData->gpsMinimumElevation;
    map["ser2BaudRate"]             = outData->ser2BaudRate;

    YAML::Node wheelCfgTransE_b2w   = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[0]);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[1]);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[2]);
    map["wheelCfgTransE_b2w"]       = wheelCfgTransE_b2w;

    YAML::Node wheelCfgTransE_b2wsig = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[0]);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[1]);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[2]);
    map["wheelCfgTransE_b2wsig"]    = wheelCfgTransE_b2wsig;

    YAML::Node wheelCfgTransT_b2w = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[0]);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[1]);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[2]);
    map["wheelCfgTransT_b2w"]       = wheelCfgTransT_b2w;

    YAML::Node wheelCfgTransT_b2wsig = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[0]);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[1]);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[2]);
    map["wheelCfgTransT_b2wsig"]    = wheelCfgTransT_b2wsig;

    map["wheelConfigTrackWidth"]    = outData->wheelConfig.track_width;
    map["wheelConfigRadius"]        = outData->wheelConfig.radius;
    map["wheelConfigBits"]          = outData->wheelConfig.bits;

    std::ofstream fout(path);

    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << map;
    fout << emitter.c_str();
    fout.close();
}

int ISDevice::LoadFlashConfig(std::string path)
{
    try
    {
        nvm_flash_cfg_t loaded_flash;
        FlashConfig(loaded_flash);

        YAML::Node inData = YAML::LoadFile(path);
        loaded_flash.size                     = inData["size"].as<uint32_t>();
        loaded_flash.checksum                 = inData["checksum"].as<uint32_t>();
        loaded_flash.key                      = inData["key"].as<uint32_t>();
        loaded_flash.startupImuDtMs           = inData["startupImuDtMs"].as<uint32_t>();
        loaded_flash.startupNavDtMs           = inData["startupNavDtMs"].as<uint32_t>();
        loaded_flash.ser0BaudRate             = inData["ser0BaudRate"].as<uint32_t>();
        loaded_flash.ser1BaudRate             = inData["ser1BaudRate"].as<uint32_t>();

        YAML::Node insRotation                = inData["insRotation"];
        loaded_flash.insRotation[0]           = insRotation[0].as<float>();
        loaded_flash.insRotation[1]           = insRotation[1].as<float>();
        loaded_flash.insRotation[2]           = insRotation[2].as<float>();

        YAML::Node insOffset                  = inData["insOffset"];
        loaded_flash.insOffset[0]             = insOffset[0].as<float>();
        loaded_flash.insOffset[1]             = insOffset[1].as<float>();
        loaded_flash.insOffset[2]             = insOffset[2].as<float>();

        YAML::Node gps1AntOffset              = inData["gps1AntOffset"];
        loaded_flash.gps1AntOffset[0]         = gps1AntOffset[0].as<float>();
        loaded_flash.gps1AntOffset[1]         = gps1AntOffset[1].as<float>();
        loaded_flash.gps1AntOffset[2]         = gps1AntOffset[2].as<float>();

        loaded_flash.dynamicModel             = (uint8_t)inData["dynamicModel"].as<uint16_t>();
        loaded_flash.debug                    = (uint8_t)inData["debug"].as<uint16_t>();
        loaded_flash.gnssSatSigConst          = inData["gnssSatSigConst"].as<uint16_t>();
        loaded_flash.sysCfgBits               = inData["sysCfgBits"].as<uint32_t>();

        YAML::Node refLla                     = inData["refLla"];
        loaded_flash.refLla[0]                = refLla[0].as<double>();
        loaded_flash.refLla[1]                = refLla[1].as<double>();
        loaded_flash.refLla[2]                = refLla[2].as<double>();

        YAML::Node lastLla                    = inData["lastLla"];
        loaded_flash.lastLla[0]               = lastLla[0].as<double>();
        loaded_flash.lastLla[1]               = lastLla[1].as<double>();
        loaded_flash.lastLla[2]               = lastLla[2].as<double>();

        loaded_flash.lastLlaTimeOfWeekMs      = inData["lastLlaTimeOfWeekMs"].as<uint32_t>();
        loaded_flash.lastLlaWeek              = inData["lastLlaWeek"].as<uint32_t>();
        loaded_flash.lastLlaUpdateDistance    = inData["lastLlaUpdateDistance"].as<float>();
        loaded_flash.ioConfig                 = inData["ioConfig"].as<uint32_t>();
        loaded_flash.platformConfig           = inData["platformConfig"].as<uint32_t>();

        YAML::Node gps2AntOffset              = inData["gps2AntOffset"];
        loaded_flash.gps2AntOffset[0]         = gps2AntOffset[0].as<float>();
        loaded_flash.gps2AntOffset[1]         = gps2AntOffset[1].as<float>();
        loaded_flash.gps2AntOffset[2]         = gps2AntOffset[2].as<float>();

        YAML::Node zeroVelRotation            = inData["zeroVelRotation"];
        loaded_flash.zeroVelRotation[0]       = zeroVelRotation[0].as<float>();
        loaded_flash.zeroVelRotation[1]       = zeroVelRotation[1].as<float>();
        loaded_flash.zeroVelRotation[2]       = zeroVelRotation[2].as<float>();

        YAML::Node zeroVelOffset              = inData["zeroVelOffset"];
        loaded_flash.zeroVelOffset[0]         = zeroVelOffset[0].as<float>();
        loaded_flash.zeroVelOffset[1]         = zeroVelOffset[1].as<float>();
        loaded_flash.zeroVelOffset[2]         = zeroVelOffset[2].as<float>();

        loaded_flash.gpsTimeUserDelay         = inData["gpsTimeUserDelay"].as<float>();
        loaded_flash.magDeclination           = inData["magDeclination"].as<float>();
        loaded_flash.gpsTimeSyncPeriodMs      = inData["gpsTimeSyncPeriodMs"].as<uint32_t>();
        loaded_flash.startupGPSDtMs           = inData["startupGPSDtMs"].as<uint32_t>();
        loaded_flash.RTKCfgBits               = inData["RTKCfgBits"].as<uint32_t>();
        loaded_flash.sensorConfig             = inData["sensorConfig"].as<uint32_t>();
        loaded_flash.gpsMinimumElevation      = inData["gpsMinimumElevation"].as<float>();
        loaded_flash.ser2BaudRate             = inData["ser2BaudRate"].as<uint32_t>();

        loaded_flash.wheelConfig.bits         = inData["wheelConfigBits"].as<uint32_t>();
        loaded_flash.wheelConfig.radius       = inData["wheelConfigRadius"].as<float>();
        loaded_flash.wheelConfig.track_width  = inData["wheelConfigTrackWidth"].as<float>();

        YAML::Node wheelCfgTransE_b2w                       = inData["wheelCfgTransE_b2w"];
        loaded_flash.wheelConfig.transform.e_b2w[0]         = wheelCfgTransE_b2w[0].as<float>();
        loaded_flash.wheelConfig.transform.e_b2w[1]         = wheelCfgTransE_b2w[1].as<float>();
        loaded_flash.wheelConfig.transform.e_b2w[2]         = wheelCfgTransE_b2w[2].as<float>();

        YAML::Node wheelCfgTransE_b2wsig                    = inData["wheelCfgTransE_b2wsig"];
        loaded_flash.wheelConfig.transform.e_b2w_sigma[0]   = wheelCfgTransE_b2wsig[0].as<float>();
        loaded_flash.wheelConfig.transform.e_b2w_sigma[1]   = wheelCfgTransE_b2wsig[1].as<float>();
        loaded_flash.wheelConfig.transform.e_b2w_sigma[2]   = wheelCfgTransE_b2wsig[2].as<float>();

        YAML::Node wheelCfgTransT_b2w                       = inData["wheelCfgTransT_b2wsig"];
        loaded_flash.wheelConfig.transform.t_b2w[0]         = wheelCfgTransT_b2w[0].as<float>();
        loaded_flash.wheelConfig.transform.t_b2w[1]         = wheelCfgTransT_b2w[1].as<float>();
        loaded_flash.wheelConfig.transform.t_b2w[2]         = wheelCfgTransT_b2w[2].as<float>();

        YAML::Node wheelCfgTransT_b2wsig                    = inData["wheelCfgTransT_b2wsig"];
        loaded_flash.wheelConfig.transform.t_b2w_sigma[0]   = wheelCfgTransT_b2wsig[0].as<float>();
        loaded_flash.wheelConfig.transform.t_b2w_sigma[1]   = wheelCfgTransT_b2wsig[1].as<float>();
        loaded_flash.wheelConfig.transform.t_b2w_sigma[2]   = wheelCfgTransT_b2wsig[2].as<float>();

        SetFlashConfig(loaded_flash);
    }
    catch (const YAML::Exception& ex)
    {
        printf("[ERROR] --- There was an error parsing the YAML file: %s", ex.what());
        return -1;
    }

    return 0;
}
