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

int ISDevice::processIsbAck(void* ctx, p_ack_t* ack, unsigned char packetIdentifier, port_handle_t port)
{
    ISDevice* device = (ISDevice*)ctx;
    //device->stepLogger(ctx, data, port);
    return (device && device->port == port) ? device->onIsbAckHandler(ack, packetIdentifier, port) : -1;
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

    if (!isConnected())
        return false;

    if (portType(port) & PORT_TYPE__COMM)
        is_comm_port_parse_messages(port); // Read data directly into comm buffer and call callback functions

    if (!hasDeviceInfo()) {
        validateAsync(30000);
    } else if (fwUpdater) {
        fwUpdate();
    } else {
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
        if (portWrite(port, &handshakerChar, 1) != 1) {
            return false;
        }

        if (portWaitForTimeout(port, &handshakerChar, 1, 10)) {
            return true;           // Success
        }
    }

    return false;
}

bool ISDevice::queryDeviceInfoISbl(uint32_t timeout) {
    uint8_t buf[64] = {};

    handshakeISbl();     // We have to handshake before we can do anything... if we've already handshaked, we won't go a response, so ignore this result

    for (int i = 0; i < 5; i++) {
        portWrite(port, (uint8_t*)"\n", 1);
        SLEEP_MS(10);
        portFlush(port);
        portRead(port, buf, sizeof(buf));    // empty Rx buffer
    }

    // Query device
    portWrite(port, (uint8_t*)":020000041000EA", 15);
    SLEEP_MS(10);

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
    uint32_t timeoutExpires = current_timeMs() + timeout;
    do {
        int count = portReadTimeout(port, buf, 14, 10);
        if (count >= 8 && buf[0] == 0xAA && buf[1] == 0x55) {   // expected response
            devInfo.firmwareVer[0] = buf[2];
            devInfo.firmwareVer[1] = buf[3];
            // m_isb_props.rom_available = buf[4];

            if (buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n') {
                switch ((ISBootloader::eProcessorType) buf[5]) {
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
                hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
                devInfo.hdwRunState = HDW_STATE_BOOTLOADER;
                devInfo.protocolVer[0] = PROTOCOL_VERSION_CHAR0;
                devInfo.protocolVer[1] = PROTOCOL_VERSION_CHAR1;
                devInfo.protocolVer[2] = PROTOCOL_VERSION_CHAR2;
                memcpy(&devInfo.serialNumber, &buf[7], sizeof(uint32_t));
                return true;
            }
        }
    } while (current_timeMs() < timeoutExpires);

    hdwId = IS_HARDWARE_TYPE_UNKNOWN;
    devInfo = {};
    return false;
}


bool ISDevice::validate(uint32_t timeout) {
    if (!isConnected())
        return false;

    // check for Inertial-Sense App by making an NMEA request (which it should respond to)
    hdwId = IS_HARDWARE_NONE,  devInfo = {};    // force a fresh check, don't just take previous values.

    queryTypes nextQueryType = QUERYTYPE_NMEA;
    unsigned int startTime = current_timeMs();
    do {
        if ((current_timeMs() - startTime) > timeout)
            return false;

        // FIXME - Don't tolerate SERIAL_PORT specific conditions in ISDevice
        if (SERIAL_PORT(port)->errorCode == ENOENT)
            return false;

        switch (nextQueryType) {
            case QUERYTYPE_NMEA:
                SendRaw((uint8_t *) NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE);
                break;
            case QUERYTYPE_ISB:
                GetData(DID_DEV_INFO);
                break;
            case QUERYTYPE_ISbootloader:
                queryDeviceInfoISbl(250);
                break;
            case QUERYTYPE_mcuBoot:
                break;

        }

        SLEEP_MS(20);
        step();

        nextQueryType = static_cast<queryTypes>((int)nextQueryType + 1 % (int)QUERYTYPE_MAX);
    } while (!hasDeviceInfo());

    if (hasDeviceInfo()) {
        // once we have device info, turn off these other messages
        GetData(DID_DEV_INFO);
        GetData(DID_SYS_PARAMS);
        GetData(DID_FLASH_CONFIG);
    }

    previousQueryType = QUERYTYPE_NMEA;
    return true;
}

/**
 * Non-blocking, internal(ish) method to validate a device.  Will be called repeatedly from step() as long as "isValidating" is true.
 * @param timeout the maximum number of milliseconds that must pass without a validating response from the device, before giving up.
 * @return -1 if the default fails to validate, 0 if the device is still validating, 1 if the device successfully validated
 */
int ISDevice::validateAsync(uint32_t timeout) {
    if (!isConnected())
        return -1;

    if (hasDeviceInfo()) {
        // we got out Device Info, so reset our timer (stop trying) and return true
        validationStartMs = 0;
        previousQueryType = QUERYTYPE_NMEA;
        return 1;
    }

    // if this is non-zero, it means we're actively validating; this helps us know when to give up/timeout
    if (!validationStartMs) {
        validationStartMs = current_timeMs();
    }

    // doing the timeout check first helps during debugging (since stepping through code will likely trigger the timeout.
    if ((current_timeMs() - validationStartMs) > timeout) {
        // We failed to get a response before the timeout occurred, so reset the timer (stop trying) and return false
        debug_message("ISDevice::validateAsync() timed out after %dms.\n", current_timeMs() - validationStartMs);
        validationStartMs = 0;
        previousQueryType = QUERYTYPE_NMEA;
        return -1;
    }

    switch (previousQueryType) {
        case ISDevice::queryTypes::QUERYTYPE_NMEA :
            // debug_message("[DBG] Querying serial port '%s' using NMEA protocol.\n", SERIAL_PORT(port)->portName);
            SendNmea(NMEA_CMD_QUERY_DEVICE_INFO);
            break;
        case ISDevice::queryTypes::QUERYTYPE_ISB :
            // debug_message("[DBG] Querying serial port '%s' using ISB protocol.\n", SERIAL_PORT(port)->portName);
            GetData(DID_DEV_INFO);
            break;
        case ISDevice::queryTypes::QUERYTYPE_ISbootloader :
            // debug_message("[DBG] Querying serial port '%s' using ISbootloader protocol.\n", SERIAL_PORT(port)->portName);
            queryDeviceInfoISbl(250);
            break;
        case ISDevice::queryTypes::QUERYTYPE_mcuBoot :
            // debug_message("[DBG] Querying serial port '%s' mcuBoot/SMP protocol.\n", SERIAL_PORT(port)->portName);
            break;
    }

    SLEEP_MS(5); // give just enough time for the device to receive, process and respond to the query

    previousQueryType = static_cast<queryTypes>(((int)previousQueryType + 1) % (int)QUERYTYPE_MAX);
    return 0;
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

std::string ISDevice::getName(const dev_info_t &devInfo, int flags) {
    // device serial no
    std::string out = utils::string_format( !(flags & COMPACT_SERIALNO) ? "SN%09d (" : "SN%d (", devInfo.serialNumber);

    // hardware type & version
    const char *typeName = "\?\?\?";
    switch (devInfo.hardwareType) {
        case IS_HARDWARE_TYPE_UINS: typeName = "uINS"; break;
        case IS_HARDWARE_TYPE_IMX: typeName = "IMX"; break;
        case IS_HARDWARE_TYPE_GPX: typeName = "GPX"; break;
        default: typeName = "\?\?\?"; break;
    }
    out += utils::string_format("%s-%u.%u", typeName, devInfo.hardwareVer[0], devInfo.hardwareVer[1]);
    if (!(flags & COMPACT_HARDWARE_VER)) {
        if ((devInfo.hardwareVer[2] != 0) || (devInfo.hardwareVer[3] != 0)) {
            out += utils::string_format(".%u", devInfo.hardwareVer[2]);
            if (devInfo.hardwareVer[3] != 0)
                out += utils::string_format(".%u", devInfo.hardwareVer[3]);
        }
    }
    out += ")";

    return out;
}

std::string ISDevice::getName(int flags) {
    return getName(devInfo, flags);
}

/**
 * Generates a single string representing the firmware version & build information for this specified device.
 * @param dev the dev_data_s device for which to format the version info
 * @param flags an indicator for the amount of detail that should be provided in the resulting string.
 *      a value of 0 will output the firmware version only.
 *      a value of 1 will output the firmware version and build number.
 *      a value of 2 will output the firmware version, build number, and build date/time.
 * @return the resulting string
 */

std::string ISDevice::getFirmwareInfo(const dev_info_t &devInfo, int flags) {
    std::string out;

    if (devInfo.hdwRunState == eHdwRunStates::HDW_STATE_BOOTLOADER) {
        out += utils::string_format("ISbl.v%u%c **BOOTLOADER**", devInfo.firmwareVer[0], devInfo.firmwareVer[1]);
    } else {
        // firmware version
        out += utils::string_format("fw%u.%u.%u", devInfo.firmwareVer[0], devInfo.firmwareVer[1], devInfo.firmwareVer[2]);
        if (!(flags & COMPACT_BUILD_TYPE)) {
            switch (devInfo.buildType) {
                case 'a': out += "-alpha";  break;
                case 'b': out += "-beta";   break;
                case 'c': out += "-rc";     break;
                case 'd': out += "-devel";  break;
                case 's': out += "-snap";   break;
                case '^': out += "-snap";   break;
                default : out += "";        break;
            }
        } else {
            out += (char)devInfo.buildType;
        }
        if (devInfo.firmwareVer[3] != 0)
            out += utils::string_format(".%u", devInfo.firmwareVer[3]);

        if (!(flags & OMIT_COMMIT_HASH)) {
            out += utils::string_format(" %08x", devInfo.repoRevision);
            if (devInfo.buildType == '^') {
                out += "^";
            }

            if (!(flags & OMIT_BUILD_DATE)) {
                // build number/type
                out += utils::string_format(" b%05x.%d", ((devInfo.buildNumber >> 12) & 0xFFFFF), (devInfo.buildNumber & 0xFFF));

                if (!(flags & OMIT_BUILD_TIME)) {
                    // build date/time
                    out += utils::string_format(" %04u-%02u-%02u", devInfo.buildYear + 2000, devInfo.buildMonth, devInfo.buildDay);
                    out += utils::string_format(" %02u:%02u:%02u", devInfo.buildHour, devInfo.buildMinute, devInfo.buildSecond);
                    if (!(flags & OMIT_BUILD_MILLIS)) {

                        if (devInfo.buildMillisecond)
                            out += utils::string_format(".%03u", devInfo.buildMillisecond);
                    }
                }
            }
        }
    }

    return out;
}

std::string ISDevice::getFirmwareInfo(int flags) {
    return getFirmwareInfo(devInfo, flags);
}

std::string ISDevice::getDescription(int flags) {
    std::string desc = getName(flags);
    if (!(flags & OMIT_FIRMWARE_VERSION)) {
        desc += " " + getFirmwareInfo(flags);
        if (!(flags & OMIT_PORT_NAME))
            desc += ", " + getPortName() + (isConnected() ? "" : " (Closed)");
    }
    return desc;
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
    if (!portIsValid(port))
        return false;

    if (devInfo.protocolVer[0] != PROTOCOL_VERSION_CHAR0)
        return false;   // TODO: Not sure if we really need this here.  We should be doing a broader level check for protocol compatibility either at a higher level, preventing us from getting here in the first place
                        //   Or at a lower-level, like in the comMangerGetData() call that does this check for everything.

    std::lock_guard<std::recursive_mutex> lock(portMutex);
    if (periodMultiple < 0) {
        DisableData(dataId);
    } else {
        GetData(dataId, 0, 0, periodMultiple);
    }
    return true;
}

int ISDevice::SetSysCmd(const uint32_t command) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    sysCmd.command = command;
    sysCmd.invCommand = ~command;
    // [C COMM INSTRUCTION]  Update the entire DID_SYS_CMD data set in the IMX.
    debug_message("Issuing SYS_CMD %d to %s (%s)\n", command, getIdAsString().c_str(), getPortName().c_str());
    return comManagerSendData(port, &sysCmd, DID_SYS_CMD, sizeof(system_command_t), 0);
}

/**
 * Send the specified string as a NMEA sentence.  This function will insert the prefix and calculate the checksum if they
 * are not already provided.
 * @param nmeaMsg
 * @return 0 on success, -1 on failure
 */
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

void ISDevice::SyncFlashConfig()
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    unsigned int timeMs = current_timeMs();
    if (timeMs - syncCheckTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
    {
        return;
    }
    syncCheckTimeMs = timeMs;

    if (devInfo.hardwareType == IS_HARDWARE_TYPE_IMX ||
        sysParams.timeOfWeekMs || 
        imxFlashCfg.checksum)
    {   // Sync IMX flash config if a IMX present
        DeviceSyncFlashCfg(timeMs, DID_FLASH_CONFIG,  imxFlashCfgUploadTimeMs, imxFlashCfg.checksum, sysParams.flashCfgChecksum, imxFlashCfgUploadChecksum);
    }

    if (devInfo.hardwareType == IS_HARDWARE_TYPE_GPX ||
        gpxDevInfo.hardwareType == IS_HARDWARE_TYPE_GPX ||
        gpxStatus.timeOfWeekMs || 
        gpxFlashCfg.checksum)
    {   // Sync GPX flash config if a GPX present
        DeviceSyncFlashCfg(timeMs, DID_GPX_FLASH_CFG, gpxFlashCfgUploadTimeMs, gpxFlashCfg.checksum, gpxStatus.flashCfgChecksum, gpxFlashCfgUploadChecksum);
    }
}

void ISDevice::DeviceSyncFlashCfg(unsigned int timeMs, uint16_t did, unsigned int &uploadTimeMs, uint32_t &flashCfgChecksum, uint32_t &syncChecksum, uint32_t &uploadChecksum)
{
    if (uploadTimeMs)
    {	// Upload in progress
        if (timeMs - uploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
        {	// Wait for upload to process.  Pause sync.
            syncChecksum = 0;
        }
    }

    // Require valid sysParams checksum
    if (syncChecksum)  
    {   
        if (syncChecksum == flashCfgChecksum)
        {
            if (uploadTimeMs)
            {   // Upload complete.  Allow sync.
                uploadTimeMs = 0;

                if (uploadChecksum == syncChecksum)
                {
                    printf("%s upload complete.\n", cISDataMappings::DataName(did));
                }
                else
                {
                    printf("%s upload rejected.\n", cISDataMappings::DataName(did));
                }
            }
        }
        else
        {	// Out of sync.  Request flash config.
            DEBUG_PRINT("Out of sync.  Requesting %s...\n", cISDataMappings::DataName(did));
            GetData(did, 0, 0, 0);
        }
    } 
}

void ISDevice::UpdateFlashConfigChecksum(nvm_flash_cfg_t &flashCfg_)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    bool platformCfgUpdateIoConfig = flashCfg_.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;

    // Exclude from the checksum update the following which does not get saved in the flash config
    flashCfg_.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;

    if (platformCfgUpdateIoConfig)
    {   // Update ioConfig
        imxPlatformConfigToFlashCfgIoConfig(&flashCfg_.ioConfig, &flashCfg_.ioConfig2, flashCfg_.platformConfig);
    }

    // Update checksum
    flashCfg_.checksum = flashChecksum32(&flashCfg_, sizeof(nvm_flash_cfg_t));
}

/**
 * Populates the passed reference to a nvm_flash_cfg_t struct with the contents of this device's last known flash config
 * @param flashCfg a struct which will be populated with a copy of the current flash configuration for this device
 * @param timeout if > 0 will block for timeout milliseconds, attempting to synchronize the flash config. If == 0, returns
 *    the current flashConfig value is memory, and does not attempt to synchronize (though it have have been previously).
 * @return true if flashCfg was populated, and the flash checksum matches the remote device's checksum (they are synchronized).
 *    False indicates that the resulting flash config cannot be trusted due to mismatched synchronization checksum or
 */
bool ISDevice::ImxFlashConfig(nvm_flash_cfg_t& flashCfg_, uint32_t timeout)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!isConnected()) {
        return false;   // No device, no flash config
    }

    // attempt to synchronize, if requested
    if (timeout > 0) {
        WaitForImxFlashCfgSynced(false, timeout);
    }

    // Copy flash config
    flashCfg_ = imxFlashCfg;

    // Indicate whether the port connection is valid, open, and the flash config is synchronized; otherwise false
    return sysParams.flashCfgChecksum == imxFlashCfg.checksum;
}

bool ISDevice::GpxFlashConfig(gpx_flash_cfg_t& flashCfg_, uint32_t timeout)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!isConnected()) {
        return false;   // No device, no flash config
    }

    // attempt to synchronize, if requested
    if (timeout > 0) {
        WaitForGpxFlashCfgSynced(false, timeout);
    }

    // Copy flash config
    flashCfg_ = gpxFlashCfg;

    // Indicate whether the port connection is valid, open, and the flash config is synchronized; otherwise false
    return gpxStatus.flashCfgChecksum == gpxFlashCfg.checksum;
}

/**
 * This uploads the provided flashCfg to the remove device, but makes no checks that it was successfully synchronized.
 * This method attempt to "intelligently" upload only the portions of the flashCfg that has actually changed, reducing
 * traffic and minimizing the risk of a sync-failure due to elements which maybe programmatically changed, however it
 * may make multiple sends, if the new and previous configurations have non-contiguous modifications.
 * Use WaitForImxFlashCfgSynced() or SetImxFlashCfgAndConfirm() to actually confirm that the new config was applied to the
 * device correctly.
 * @param flashCfg_ the new flash_config to upload
 * @return true if the ANY of the changes failed to send to the remove device.
 */
bool ISDevice::SetImxFlashConfig(nvm_flash_cfg_t& flashCfg) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    // Temporarily clear updateIoConfig for checksum
    bool updateIo = flashCfg.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;
    flashCfg.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;
    UpdateFlashConfigChecksum(flashCfg);
    if (updateIo) flashCfg.platformConfig |= PLATFORM_CFG_UPDATE_IO_CONFIG;

    bool success = UploadFlashConfigDiff(
        reinterpret_cast<uint8_t*>(&flashCfg),
        reinterpret_cast<uint8_t*>(&imxFlashCfg),
        sizeof(nvm_flash_cfg_t),
        DID_FLASH_CONFIG,
        imxFlashCfgUploadTimeMs,
        imxFlashCfgUploadChecksum
    );

    if (!imxFlashCfgUploadTimeMs)
        printf("DID_FLASH_CONFIG in sync.  No upload.\n");
    else
        imxFlashCfgUploadChecksum = flashCfg.checksum;

    imxFlashCfg = flashCfg;
    return success;
}

bool ISDevice::SetGpxFlashConfig(gpx_flash_cfg_t& flashCfg) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    bool success = UploadFlashConfigDiff(
        reinterpret_cast<uint8_t*>(&flashCfg),
        reinterpret_cast<uint8_t*>(&gpxFlashCfg),
        sizeof(nvm_flash_cfg_t),
        DID_FLASH_CONFIG,
        gpxFlashCfgUploadTimeMs,
        gpxFlashCfgUploadChecksum
    );

    if (!gpxFlashCfgUploadTimeMs)
        printf("DID_GPX_FLASH_CONFIG in sync.  No upload.\n");
    else
        gpxFlashCfgUploadChecksum = flashCfg.checksum;

    gpxFlashCfg = flashCfg;
    return success;
}

bool ISDevice::UploadFlashConfigDiff(uint8_t* newData, uint8_t* curData, size_t sizeBytes, uint32_t did, uint32_t& uploadTimeMsOut, uint32_t& checksumOut) {
    std::vector<cISDataMappings::MemoryUsage> usageVec;
    const auto& dataSetMap = *cISDataMappings::NameToInfoMap(did);

    for (const auto& [fieldName, info] : dataSetMap)
    {
        if (info.size == 0) continue;

        // Handle arrays element by element
        size_t elemSize = (info.arraySize > 0) ? info.elementSize : info.size;
        size_t count = (info.arraySize > 0) ? info.arraySize : 1;

        for (size_t i = 0; i < count; ++i)
        {
            uint8_t* newPtr = newData + info.offset + i * elemSize;
            uint8_t* curPtr = curData + info.offset + i * elemSize;

            if (memcmp(newPtr, curPtr, elemSize) != 0)
            {
                cISDataMappings::AppendMemoryUsage(usageVec, newPtr, elemSize);
            }
        }
    }

    bool failure = false;
    for (const cISDataMappings::MemoryUsage& usage : usageVec)
    {
        int offset = static_cast<int>(usage.ptr - newData);
        std::cout << "Sending " << cISDataMappings::DataName(did) << ": size " << usage.size << ", offset " << offset << std::endl;
        failure |= SendData(did, usage.ptr, static_cast<int>(usage.size), offset);

        if (!failure)
        {
            uploadTimeMsOut = current_timeMs();
        }
    }

    return !failure;
}

bool ISDevice::ImxFlashConfigSynced() {
    return  (imxFlashCfg.checksum == sysParams.flashCfgChecksum) && 
            (imxFlashCfgUploadTimeMs == 0) && 
            !ImxFlashConfigUploadFailure(); 
}

bool ISDevice::GpxFlashConfigSynced() {
    return  (gpxFlashCfg.checksum == gpxStatus.flashCfgChecksum) && 
            (gpxFlashCfgUploadTimeMs == 0) && 
            !GpxFlashConfigUploadFailure(); 
}

bool ISDevice::ImxFlashConfigUploadFailure() {
    // a failed flash upload is considered when imxFlashCfgUploadChecksum is non-zero, and DOES NOT match sysParams.flashCfgChecksum
    return  imxFlashCfgUpload.checksum && 
            (imxFlashCfgUpload.checksum != sysParams.flashCfgChecksum);
}
bool ISDevice::GpxFlashConfigUploadFailure() {
    // a failed flash upload is considered when gpxFlashCfgUploadChecksum is non-zero, and DOES NOT match gpxStatus.flashCfgChecksum
    return  gpxFlashCfgUpload.checksum && 
            (gpxFlashCfgUpload.checksum != gpxStatus.flashCfgChecksum);
}

/**
 * A blocking function calls which waits until both a DID_FLASH_CFG and DID_SYS_PARAMS have
 * been received which have a matching flashCfg checksum; ensuring that we have a valid copy
 * of the devices' flash configuration.
 * @param timeout
 * @return true if both the flashCfg.checksum and sysParams.flashCfgChecksum match (and neither are zero)
 */
bool ISDevice::WaitForImxFlashCfgSynced(bool forceSync, uint32_t timeout)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!port)
        return false;   // No device, no flash-sync

    // If there are no upload pending, then just go ahead and check...
    unsigned int now = current_timeMs();
    unsigned int startMs = now;

    while(!ImxFlashConfigSynced())
    {   // Request and wait for IMX flash config
        step();
        SLEEP_MS(100);

        if (current_timeMs() - startMs > timeout)
        {   // Timeout waiting for IMX flash config
            printf("Timeout waiting for DID_FLASH_CONFIG failure!\n");

#if PRINT_DEBUG
            DEBUG_PRINT("device.imxFlashCfg.checksum:          %u\n", imxFlashCfg.checksum);
            DEBUG_PRINT("device.sysParams.flashCfgChecksum:    %u\n", sysParams.flashCfgChecksum); 
            DEBUG_PRINT("device.imxFlashCfgUploadTimeMs:       %u\n", imxFlashCfgUploadTimeMs);
            DEBUG_PRINT("device.imxFlashCfgUploadChecksum:     %u\n", imxFlashCfgUploadChecksum);
#endif
            return false;
        }
        else
        {   // Query DID_SYS_PARAMS
            GetData(DID_SYS_PARAMS);
            DEBUG_PRINT("Waiting for IMX flash sync...\n");
        }
    }

    return ImxFlashConfigSynced();
}

bool ISDevice::WaitForGpxFlashCfgSynced(bool forceSync, uint32_t timeout)
{
    unsigned int startMs = current_timeMs();
    while(!GpxFlashConfigSynced())
    {   // Request and wait for GPX flash config
        step();
        SLEEP_MS(100);

        if (current_timeMs() - startMs > timeout)
        {   // Timeout waiting for GPX flash config
            printf("Timeout waiting for DID_GPX_FLASH_CONFIG failure!\n");

#if PRINT_DEBUG
            DEBUG_PRINT("device.gpxFlashCfg.checksum:          %u\n", gpxFlashCfg.checksum);
            DEBUG_PRINT("device.gpxStatus.flashCfgChecksum:    %u\n", gpxStatus.flashCfgChecksum); 
            DEBUG_PRINT("device.gpxFlashCfgUploadTimeMs:       %u\n", gpxFlashCfgUploadTimeMs);
            DEBUG_PRINT("device.gpxFlashCfgUploadChecksum:     %u\n", gpxFlashCfgUploadChecksum);
#endif
            return false;
        }
        else
        {   // Query DID_GPX_STATUS
            GetData(DID_GPX_STATUS);
            DEBUG_PRINT("Waiting for GPX flash sync...\n");
        }
    }

    return GpxFlashConfigSynced();
}

/**
 * @return true if there are "PENDING FLASH WRITES" waiting to clear, or no response from device.
 */
bool ISDevice::hasPendingImxFlashWrites(uint32_t& ageSinceLastPendingWrite) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!port || !portIsOpened(port))
        return false;

    return ((sysParams.hdwStatus & HDW_STATUS_FLASH_WRITE_PENDING) || (sysParams.hdwStatus == 0));
}

/**
 * Sets the provided FlashCfg and then blocks for timeout, waiting for the uploaded FlashCfg to be
 * then downloaded, before finally confirming that the new values have been set.
 * @param flashCfg the flash config to upload
 * @return
 */
bool ISDevice::SetImxFlashCfgAndConfirm(nvm_flash_cfg_t& flashCfg, uint32_t timeout) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!SetImxFlashConfig(flashCfg))  // Upload and verify upload
        return false;               // we failed to even upload the new config

    // save the uploaded config, with correct checksum calculated in SetImxFlashConfig()
    nvm_flash_cfg_t tmpFlash = flashCfg;

    SLEEP_MS(10);
    step();

    if (!WaitForImxFlashCfgSynced(false, timeout))
        return false;   // Re-download flash config

    if ((imxFlashCfgUploadTimeMs != 0) && (imxFlashSyncCheckTimeMs != 0))
        return false;   // timed-out,

    return (memcmp(&flashCfg, &tmpFlash, sizeof(nvm_flash_cfg_t)) == 0);
}

/**
 * Sets the provided GPX FlashCfg and then blocks for timeout, waiting for the uploaded FlashCfg to be
 * then downloaded, before finally confirming that the new values have been set.
 * @param flashCfg the flash config to upload
 * @return true if the flash config was successfully uploaded and confirmed, otherwise false
 */
bool ISDevice::SetGpxFlashCfgAndConfirm(gpx_flash_cfg_t& flashCfg, uint32_t timeout) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!SetGpxFlashConfig(flashCfg))  // Upload and verify upload
        return false;               // we failed to even upload the new config

    // save the uploaded config, with correct checksum calculated in SetImxFlashConfig()
    gpx_flash_cfg_t tmpFlash = flashCfg;

    SLEEP_MS(10);
    step();

    if (!WaitForGpxFlashCfgSynced(false, timeout))
        return false;   // Re-download flash config

    if ((gpxFlashCfgUploadTimeMs != 0) && (gpxFlashSyncCheckTimeMs != 0))
        return false;   // timed-out,

    return (memcmp(&flashCfg, &tmpFlash, sizeof(gpx_flash_cfg_t)) == 0);
}

template<typename T>
bool SaveFlashConfigToFile(const std::string& path, int did, std::function<bool(T&)> getCfg)
{
    T flashCfg;
    if (!getCfg(flashCfg))
    {
        printf("[ERROR] --- Failed to get flash config\n");
        return false;
    }

    YAML::Node yaml;
    if (!cISDataMappings::DataToYaml(did, reinterpret_cast<const uint8_t*>(&flashCfg), yaml))
    {
        printf("[ERROR] --- Failed to serialize flash config to YAML\n");
        return false;
    }

    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << yaml;

    std::ofstream fout(path);
    fout << emitter.c_str();
    return true;
}

template<typename T>
bool LoadFlashConfigFromFile(const std::string& path, int did, std::function<bool(T&)> setCfg)
{
    try
    {
        YAML::Node yaml = YAML::LoadFile(path);
        T flashCfg;
        if (!cISDataMappings::YamlToData(did, yaml, reinterpret_cast<uint8_t*>(&flashCfg)))
        {
            printf("[ERROR] --- Failed to parse YAML into flash config structure\n");
            return false;
        }

        if (!setCfg(flashCfg))
        {
            printf("[ERROR] --- Failed to apply flash config\n");
            return false;
        }
    }
    catch (const YAML::Exception& ex)
    {
        printf("[ERROR] --- There was an error parsing the YAML file: %s\n", ex.what());
        return false;
    }

    return true;
}

bool ISDevice::SaveImxFlashConfigToFile(std::string path)
{
    return SaveFlashConfigToFile<nvm_flash_cfg_t>(path, DID_FLASH_CONFIG,
        [this](nvm_flash_cfg_t& cfg) { return ImxFlashConfig(cfg); });
}

bool ISDevice::SaveGpxFlashConfigToFile(std::string path)
{
    return SaveFlashConfigToFile<gpx_flash_cfg_t>(path, DID_GPX_FLASH_CFG,
        [this](gpx_flash_cfg_t& cfg) { return GpxFlashConfig(cfg); });
}

bool ISDevice::LoadImxFlashConfigFromFile(std::string path)
{
    return LoadFlashConfigFromFile<nvm_flash_cfg_t>(path, DID_FLASH_CONFIG,
        [this](nvm_flash_cfg_t& cfg) { return SetImxFlashConfig(cfg); });
}

bool ISDevice::LoadGpxFlashConfigFromFile(std::string path)
{
    return LoadFlashConfigFromFile<gpx_flash_cfg_t>(path, DID_GPX_FLASH_CFG,
        [this](gpx_flash_cfg_t& cfg) { return SetGpxFlashConfig(cfg); });
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
        return 0;   // this message is invalid, so don't let anything else try and handle it...
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
            if (devInfo.hdwRunState == HDW_STATE_UNKNOWN)   // this value should be passed from the device, but if not...
                devInfo.hdwRunState = HDW_STATE_APP;        // since this is ISB, its pretty safe to assume that we are in APP mode.
            break;
        case DID_SYS_CMD:
            sysCmd = *(system_command_t*)data->ptr;
            break;
        case DID_SYS_PARAMS:
            copyDataPToStructP(&sysParams, data, sizeof(sys_params_t));
            DEBUG_PRINT("Received DID_SYS_PARAMS\n");
            break;
        case DID_FLASH_CONFIG:
            copyDataPToStructP(&imxFlashCfg, data, sizeof(nvm_flash_cfg_t));
            if ( dataOverlap(offsetof(nvm_flash_cfg_t, checksum), 4, data)) {
                sysParams.flashCfgChecksum = imxFlashCfg.checksum;
            }
            DEBUG_PRINT("Received DID_FLASH_CONFIG\n");
            break;
        case DID_GPX_FLASH_CFG:
            copyDataPToStructP(&gpxFlashCfg, data, sizeof(gpx_flash_cfg_t));
            if ( dataOverlap( offsetof(gpx_flash_cfg_t, checksum), 4, data ) )
            {	// Checksum received
                gpxStatus.flashCfgChecksum = gpxFlashCfg.checksum;
            }
            DEBUG_PRINT("Received DID_GPX_FLASH_CFG\n");
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
            if (abs(currentTime - lastTime) > 5) 
            {   // Update every 5 seconds
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

int ISDevice::onIsbAckHandler(p_ack_t* ack, unsigned char packetIdentifier, port_handle_t port){

    return 1;   // allow others to continue to process this message
}

// return 0 on success, -1 on failure
int ISDevice::onNmeaHandler(const unsigned char* msg, int msgSize, port_handle_t port) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    switch (getNmeaMsgId(msg, msgSize))
    {
        case NMEA_MSG_ID_INFO:
        {	// IMX device Info
            dev_info_t info;
            nmea_parse_info(info, (const char*)msg, msgSize);
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
            devInfo.hdwRunState = HDW_STATE_APP;
            switch (info.hardwareType)
            {
            case IS_HARDWARE_TYPE_IMX:
                devInfo = info;
                break;

            case IS_HARDWARE_TYPE_GPX:
                if (devInfo.hardwareType == 0 ||
                    devInfo.hardwareType == IS_HARDWARE_TYPE_GPX)
                {   // Populate if device info is not set or GPX
                    devInfo = info;
                }
                gpxDevInfo = info;
                break;
            }
        }
        break;
    }
    return 1;   // allow others to continue to process this message
}

int ISDevice::onPacketHandler(protocol_type_t ptype, packet_t *pkt, port_handle_t port) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (ptype == _PTYPE_INERTIAL_SENSE_ACK) {

        eISBPacketFlags pktType = is_comm_to_isb_pkt_type(&COMM_PORT(port)->comm);
        return onIsbAckHandler((p_ack_t*)pkt->data.ptr, pktType, port);
    }

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
    // registerIsbAckDataHandler(processIsbAckMsgs);
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

