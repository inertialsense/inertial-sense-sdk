/**
 * @file ISDevice.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "core/msg_logger.h"
#include "DeviceManager.h"
#include "ISDevice.h"
#include "ISFirmwareUpdater.h"
#include "ISHttpRequest.h"
#include "ISLogger.h"
#include "util/util.h"
#include "imx_defaults.h"

const ISDevice ISDevice::invalidRef;

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

int ISDevice::processPacket(void *ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port) 
{
    ISDevice* device = (ISDevice*)ctx;
    return (device && device->port == port) ? device->onPacketHandler(ptype, pkt, port) : -1;
}

/**
 * @param ptype the type (_PTYPE_*) of the packet query. Default to _PTYPE_NONE (any packet type)
 * @return returns the number of milliseconds since a message of this type was received. If ptype == _PTYPE_NONE
 *   this will return the minimum age of all packet types.
 */
uint32_t ISDevice::millisSinceLastRx(int ptype) {
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    if (ptype != _PTYPE_NONE)
    {
        // Validate ptype is within the valid range before indexing lastRxTs.
        if (ptype < 0 || ptype >= _PTYPE_SIZE)
        {
            // Unknown or invalid packet type; define as "no packets received" by returning the max age.
            return UINT32_MAX;
        }

        auto ts = lastRxTs[ptype];
        if (ts == std::chrono::high_resolution_clock::time_point())
        {
            // No packets of this type have been received yet.
            return UINT32_MAX;
        }

        return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - ts).count();
    }
    uint32_t min = UINT32_MAX;
    for (auto ts : lastRxTs) {
        if (ts == std::chrono::high_resolution_clock::time_point())
            continue;
        min = std::min(min, (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - ts).count());
    }
    return min;
}


/**
 * Steps the communications for this device, sending any scheduled requests and parsing any received data on the device's associated port (if connected).
 * @return false if the device is inactionable, either through configuration or port status; otherwise true indicates a sufficient state to perform work, even if there was nothing to do.
 */
bool ISDevice::step() {
    FnProfiler fn("ISDevice::step() [" + getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO) + "]", 50000);    // this shouldn't really ever take longer than 50ms to execute
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (portFlagsIsSet(port, PORT_FLAG__NO_ISDEVICE))
        return false;

    if (was_connected != portIsOpened(port)) {
        was_connected = isConnected();
        DeviceManager::getInstance().notifyListeners(shared_from_this(), isConnected() ? DeviceManager::DEVICE_CONNECTED : DeviceManager::DEVICE_DISCONNECTED);
    }

    bool didStuff = false;
    if (isConnected() && (portType(port) & PORT_TYPE__COMM) && !(COMM_PORT(port)->flags & COMM_PORT_FLAG__EXPLICIT_READ)) {
        is_comm_port_parse_messages(port); // Read data directly into comm buffer and call callback functions
        fn.mark("Parsed messages.");
        if (!hasDeviceInfo()) {
            validateAsync(250);
            fn.mark("Validating device.");
        } else {
            SyncFlashConfig();
            fn.mark("Synchronizing flash.");
        }
        didStuff = true;
    }

    if (m_calibration && m_calUploadState != -1) {
        if (ISDeviceCal::uploadSensorCalStep(port, m_calUploadState, *m_calibration) != ASYNC_STATE__PENDING) {
            m_calibration = nullptr;
            m_calUploadState = -1;
        }
    }

    if (fwUpdater) {  // the fwUpdate MUST happen after is_comm_port_parse_messages
        fwUpdate();
        fn.mark("Handling fwUpdate().");
        didStuff = true;
    }

    return didStuff;
}

is_operation_result ISDevice::updateFirmware(fwUpdate::target_t targetDevice, std::vector<std::string> cmds, fwUpdate::pfnStatusCb infoProgress, void (*waitAction)()) {
    std::unique_lock<std::mutex> lock(fwUpdateMutex, std::try_to_lock);
    if (!lock.owns_lock())
        return IS_OP_ERROR;

    fwUpdateState.resetState();

    if (!fwUpdater) {
        fwUpdater = new ISFirmwareUpdater(shared_from_this(), fwUpdateState);
    }

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
float ISDevice::fwUpdatePercentCompleted() {
    return (fwUpdater && !fwUpdater->fwUpdate_isDone()) ? fwUpdater->getProgress() : 0.0f;
}

/**
 * Returns a set of messages generated during a firmware update
 * @param level the IS_LOG_LEVEL_* of messages to return from the update (defaults to IS_LOG_LEVEL_ERROR)
 * @return
 */
std::vector<ISFwUpdateState::message> ISDevice::fwUpdateMessages(eLogLevel level) {
    auto lk = fwUpdateState.lock();
    std::vector<ISFwUpdateState::message> out;
    for (const auto& msg : fwUpdateState.messages) {
        if (msg.severity < level)
            out.push_back(msg);
    }
    return out;
}

/**
 * Instructs the device to continue performing its actions.  This should be called regularly to ensure that the update process
 * does not stall.
 * @param msg a pointer to an optional p_data_t containing a DID_FIRMWARE message to be processed; if nullptr (default) then no message is parsed.
 * @return true if the update is still in progress (calls inProgress()), or false if the update is finished and no further updates are needed.
 */
bool ISDevice::fwUpdate(p_data_t* msg) {
    std::unique_lock<std::mutex> lock(fwUpdateMutex, std::try_to_lock);
    // check if our mutex is already locked, if so, we're recursing into this function, and we shouldn't...
    if (!lock.owns_lock())
        return true;

    if (fwUpdater) {
        if (portIsValid(port) && !isConnected())
            connect(true);  // especially if we're updated - this port should never really be closed (right??)

        if (msg) fwUpdater->processMessage(msg);

        // FIXME: fwUpdate->device almost isn't necessary, and its circular since device->fwUpdater->device - We should eliminate this if possible
        if (fwUpdater->device != shared_from_this())
            fwUpdater->device = shared_from_this();

        fwUpdater->step();

        // cleanup if we're done.
        if (fwUpdater->fwUpdate_isDone()) {
            // collect errors before we close out the updater
            delete fwUpdater;
            fwUpdater = nullptr;
        }
    }

    return fwUpdateInProgress();
}

bool ISDevice::handshakeISbl() {
    static const uint8_t handshakerChar = 'U';
    uint8_t readCh = 0;

    // this call shouldn't really ever take longer than 120ms to execute - Sending 10x 'U' every 10ms + 20%
    FnProfiler fn("ISDevice::handshakeISbl() [" + getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO) + "]", 120000);
    // log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::handshakeISbl() called.", getIdAsString().c_str());

    // first, flush all incoming data and ensure we have a clean buffer...
    for (int i = 0; i < 5; i++) {
        if (portAvailable(port))
            portFlush(port);

        if ((i == 4) && portAvailable(port)) {
            log_warn(IS_LOG_ISDEVICE, "[%s] ISDevice::handshakeISbl() is unable to clear the port RX buffer. Handshaking is not possible.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
            return true;   // unable to clear buffer, so we can't handshake, but return true anyway so we don't keep trying
        }
    }
    fn.mark("RX Buffer cleared. Starting handshake.");

    // Bootloader sync requires at least 6 'U' characters to be sent every 10ms.
    // write a 'U' to handshake with the bootloader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_HANDSHAKE_COUNT; i++) {
        // OLD WAY : if (portWaitForTimeout(port, &handshakerChar, 1, 10)) {
        while (portRead(port, &readCh, 1) == 1) {
            if (readCh == handshakerChar)
                return true;    // received a responding handshake char, so success
        }

        if (portWrite(port, &handshakerChar, 1) != 1) {
            return false;   // failed to write, so there is an error
        }
        SLEEP_MS(BOOTLOADER_HANDSHAKE_DELAY);
    }

    // if we got here without an error, and still haven't seen a valid handshake character
    // then we probably never will (likely because its already seen a handshake before)
    return true;    // go ahead and return true, so we don't keep trying
}

bool ISDevice::queryDeviceInfoISbl(uint32_t timeout) {
    uint8_t buf[64] = {};
    if (!portIsOpened(port))
        return false;

    FnProfiler fn("ISDevice::queryDeviceInfoISbl() [" + getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO) + "]", timeout / 2 * 1000);    // this shouldn't really ever take longer than 50ms to execute
    //log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::queryDeviceInfoISbl() called.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
    if (!hasHandshake) {
        hasHandshake = handshakeISbl();     // We have to handshake before we can do anything... if we've already handshaked, we won't go a response, so ignore this result
        fn.mark("Handshake == " + std::to_string(hasHandshake));
    }

    // clear any partial commands and flush the rx buffer
    for (int i = 0; i < 5; i++) {
        if (portWrite(port, (uint8_t*)"\n", 1) == 1) {
            SLEEP_MS(2);
            if (portAvailable(port))
                portFlush(port);
        }
    }
    fn.mark("Flushed Tx buffer.");

    // Query device
    if (portWrite(port, (uint8_t*)":020000041000EA", 15) != 15)
        return false;       // we couldn't send the entire string - so don't even bother waiting for a response. We'll have to try again later.

    fn.mark("Query sent.");

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
    uint32_t timeoutExpires = current_timeMs() + timeout;
    do {
        int count = portReadTimeout(port, buf, 14, 50);
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
                        devInfo.hardwareType = IS_HARDWARE_TYPE_GPX; // OR IMX-6.0
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
                fn.mark("Got a response.");
                return true;
            }
        }
        SLEEP_MS(5);
    } while (current_timeMs() < timeoutExpires);

    // hdwId = IS_HARDWARE_TYPE_UNKNOWN;
    // devInfo = {};
    fn.mark("Timed-out waiting.");
    // log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::queryDeviceInfoISbl() no valid response received - Either not an ISDevice, or not in ISbootloader.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
    return false;
}


bool ISDevice::validate(uint32_t timeout) {
    if (!isConnected())
        return false;

    FnProfiler fn("ISDevice::queryDeviceInfoISbl() [" + getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO) + "]", timeout / 2 * 1000);    // this shouldn't really ever take longer than 50ms to execute
    log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) called.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);

    // check for Inertial-Sense App by making an NMEA request (which it should respond to)
    is_hardware_t oldHdwId = hdwId;
    dev_info_t oldDevInfo = devInfo;
    hdwId = IS_HARDWARE_NONE,  devInfo = {};    // force a fresh check, don't just take previous values.

    bool hasDevInfo = hasDeviceInfo();
    queryType nextQueryType = QUERYTYPE_NMEA;
    unsigned int startTime = current_timeMs();
    do {
        if ((current_timeMs() - startTime) > timeout) {
            // after we've timed out - make a last ditch effort to check for a legacy (<6j) IS bootloader, otherwise fail
            hdwId = oldHdwId, devInfo = oldDevInfo;
            log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) : Device failed to validate in time.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);
            return (queryDeviceInfoISbl(250) && hasDeviceInfo());
        }

        if (!port || !portIsValid(port) || !portIsOpened(port)) {
            log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) : Port invalidated or closed while validating.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);
            hdwId = oldHdwId, devInfo = oldDevInfo;
            return false;
        }

        switch (nextQueryType) {
            case QUERYTYPE_NMEA:
                log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) Querying NMEA DEV_INFO.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);
                SendRaw((uint8_t *) NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE);
                break;
            case QUERYTYPE_ISB:
                log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) Querying ISB DEV_INFO.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);
                GetData(DID_DEV_INFO);
                break;
            case QUERYTYPE_ISbootloader:
                log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) Querying ISbl DEV_INFO.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);
                queryDeviceInfoISbl(250);
                break;
            case QUERYTYPE_mcuBoot:
                break;

        }

        SLEEP_MS(10);    // make sure we give enough time for the device to respond - otherwise we might step each others toes
        // step();   Instead of doing this...
        if (isConnected() && (portType(port) & PORT_TYPE__COMM)) {
            // log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) Parsing responses...", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);
            is_comm_port_parse_messages(port); // ...read data directly into comm buffer and call callback functions
        }

        nextQueryType = static_cast<queryType>(((int)nextQueryType + 1) % (int)QUERYTYPE_MAX);
        hasDevInfo = hasDeviceInfo();
    } while (!hasDevInfo);

    fn.mark("Finished validating.");
    log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) : Validation finished: %s", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout, hasDevInfo ? "SUCCESS" : "FAILURE");

    if (hasDevInfo) {
        // once we have device info, turn off these other messages
        GetData(DID_DEV_INFO);
        GetData(DID_SYS_PARAMS);
        GetData(DID_FLASH_CONFIG);
        // if we aren't connected to a GPX, these should be ignored -- but if a GPX is available, we want to know about it.
        GetData(DID_GPX_FLASH_CFG);
        GetData(DID_GPX_STATUS);
    }

    nextValidationType = QUERYTYPE_NMEA;
    return true;
}

/**
 * Non-blocking, internal(ish) method to validate a device.  Will be called repeatedly from step() as long as "isValidating" is true.
 * @param timeout the maximum number of milliseconds that must pass without a validating response from the device, before giving up.
 * @return ASYNC_STATE__TIMEOUT if the device fails to validate,
 *         ASYNC_STATE__PENDING if the device is still validating,
 *         ASYNC_STATE__SUCCESSif the device successfully validated.
 */
int ISDevice::validateAsync(uint32_t timeout) {
    if (!isConnected())
        return -1;

    uint32_t now = current_timeMs();
    FnProfiler fn("ISDevice::validateAsync() [" + getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO) + "]", timeout / 2 * 1000);    // this shouldn't really ever take longer than 50ms to execute
    if (hasDeviceInfo()) {
        // we got out Device Info, so reset our timer (stop trying) and return true
        // log_debug(IS_LOG_ISDEVICE, "[%s] validateAsync() finished after %dms.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), current_timeMs() - validationStartMs);
        validationStartMs = 0;
        nextValidationType = QUERYTYPE_NMEA;
        hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);

        // once we have device info, turn off these other messages
        GetData(DID_DEV_INFO);
        GetData(DID_SYS_PARAMS);
        GetData(DID_FLASH_CONFIG);

        return ASYNC_STATE__SUCCESS;
    }

    // if this is non-zero, it means we're actively validating; this helps us know when to give up/timeout
    if (!validationStartMs) {
        validationStartMs = now;
    }

    // doing the timeout check first helps during debugging (since stepping through code will likely trigger the timeout.
    if ((now - validationStartMs) > timeout) {
        /*
        // Do this when we leave scope...
        Finalizer cleanup([&] { nextValidationType = QUERYTYPE_NMEA,  validationStartMs = 0; });

        // after we've timed out - make a last ditch effort to check for a legacy (<6j) IS bootloader, otherwise fail
        if (hasDeviceInfo() || queryDeviceInfoISbl(250)) {
            log_debug(IS_LOG_ISDEVICE, "[%s] validateAsync() finished after %dms.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), current_timeMs() - validationStartMs);
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
            return ASYNC_STATE__SUCCESS;
        }

        */
        // We failed to get a response before the timeout occurred, so reset the timer (stop trying) and return false
        log_debug(IS_LOG_ISDEVICE, "[%s] validateAsync() timed out after %dms [[ %d, %d, %d ]]", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), current_timeMs() - validationStartMs, devInfo.hdwRunState, devInfo.hardwareType, devInfo.serialNumber);
        nextValidationType = QUERYTYPE_NMEA;
        validationStartMs = 0;
        return ASYNC_STATE__TIMEOUT;
    }

    if (nextValidationMs < now) {
        switch (nextValidationType) {
            case QUERYTYPE_NMEA :
                // log_debug(IS_LOG_ISDEVICE, "Querying serial port '%s' using NMEA protocol.", SERIAL_PORT(port)->portName);
                SendNmea(NMEA_CMD_QUERY_DEVICE_INFO);
                break;
            case QUERYTYPE_ISB :
                // log_debug(IS_LOG_ISDEVICE, "Querying serial port '%s' using ISB protocol.", SERIAL_PORT(port)->portName);
                GetData(DID_DEV_INFO);
                break;
            case QUERYTYPE_ISbootloader :
                queryDeviceInfoISbl(250);
                break;
            case QUERYTYPE_mcuBoot :
                // log_debug(IS_LOG_ISDEVICE, "Querying serial port '%s' mcuBoot/SMP protocol.", SERIAL_PORT(port)->portName);
                break;
        }

        // SLEEP_MS(2);    // make sure we give enough time for the device to respond - otherwise we might step each others toes
        nextValidationType = static_cast<queryType>(((int)nextValidationType + 1) % (int)QUERYTYPE_MAX);
        nextValidationMs = now + 10;  // 10 millis to respond before we try the next query method
    }
    return ASYNC_STATE__PENDING;
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

std::string ISDevice::getIdAsString() const {
    return getIdAsString(devInfo);
}

uint64_t ISDevice::parseDeviceIdString(const std::string& str) {
    uint32_t serialNo = 0;
    is_hardware_t hdwId = IS_HARDWARE_ANY;

    // Find separator — accept both "::" (canonical) and ":" (shorthand)
    std::string snPart;
    size_t sepPos = str.find("::");
    size_t sepLen = 2;
    if (sepPos == std::string::npos) {
        sepPos = str.find(':');
        sepLen = 1;
    }

    if (sepPos != std::string::npos) {
        // Has hardware type prefix, e.g. "IMX-5.0::SN129495" or "IMX-5.0:129495"
        std::string hdwStr = str.substr(0, sepPos);
        snPart = str.substr(sepPos + sepLen);

        // Parse hardware type: "IMX-5.0", "GPX-1.0", "uINS-3.2", etc.
        size_t dashPos = hdwStr.find('-');
        if (dashPos != std::string::npos) {
            std::string typeName = hdwStr.substr(0, dashPos);
            std::string verStr = hdwStr.substr(dashPos + 1);
            uint8_t hdwType = 0;
            if (strcasecmp(typeName.c_str(), "IMX") == 0)       hdwType = IS_HARDWARE_TYPE_IMX;
            else if (strcasecmp(typeName.c_str(), "GPX") == 0)   hdwType = IS_HARDWARE_TYPE_GPX;
            else if (strcasecmp(typeName.c_str(), "uINS") == 0)  hdwType = IS_HARDWARE_TYPE_UINS;
            else if (strcasecmp(typeName.c_str(), "EVB") == 0)   hdwType = IS_HARDWARE_TYPE_EVB;
            if (hdwType) {
                uint8_t major = 0, minor = 0;
                size_t dotPos = verStr.find('.');
                if (dotPos != std::string::npos) {
                    major = (uint8_t)strtoul(verStr.c_str(), nullptr, 10);
                    minor = (uint8_t)strtoul(verStr.c_str() + dotPos + 1, nullptr, 10);
                } else {
                    major = (uint8_t)strtoul(verStr.c_str(), nullptr, 10);
                }
                hdwId = ENCODE_HDW_ID(hdwType, major, minor);
            }
        }
    } else {
        // No separator — could be "SN129495" or just "129495"
        snPart = str;
    }

    // Strip optional "SN" prefix from serial number part
    if (snPart.size() > 2 && strncasecmp(snPart.c_str(), "SN", 2) == 0)
        snPart = snPart.substr(2);
    serialNo = strtoul(snPart.c_str(), nullptr, 10);

    if (serialNo == 0)
        return 0;

    return ((uint64_t)hdwId << 48) | serialNo;
}

/**
 * Renders a string which can serve as a unique hardware identifier describing the device_info provided
 *   The output string looks like "SN<number> (<hdwType>-<hdwVersion>)". Flags can be used to modify the
 *   output slightly, as desired.
 * @param devInfo the device info used to render the string
 * @param flags a bitmask of rendering options, specifically:
 *   - COMPACT_SERIALNO     renders the device serial number without leading zeros
 *   - COMPACT_HARWARE_VER  hides the 3rd and 4rth digits of the hardware version, unless they are non-zero
 * @return
 */
std::string ISDevice::getName(const dev_info_t &devInfo, int flags) {
    // device serial no
    std::string out = utils::string_format( !(flags & COMPACT_SERIALNO) ? "SN%09d (" : "SN%d (", devInfo.serialNumber);

    // hardware type & version
    const char *typeName = "\?\?\?";
    switch (devInfo.hardwareType) {
        case IS_HARDWARE_TYPE_UINS: typeName = "uINS"; break;
        case IS_HARDWARE_TYPE_IMX: typeName = "IMX"; break;
        case IS_HARDWARE_TYPE_GPX: typeName = "GPX"; break;
        case IS_HDW_GNSS_SONY: typeName = "CXD"; break;
        case IS_HDW_GNSS_UBLOX: typeName = "UBX"; break;
        case IS_HDW_GNSS_SEPTENTRIO: typeName = "SEP"; break;
        case IS_HDW_GNSS_STM_TESSIO: typeName = "STM"; break;
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

std::string ISDevice::getName(int flags) const {
    return getName(devInfo, flags);
}

/**
 * @brief Generates a string representing the firmware version & build information provided in the provided dev_info_t struct
 *
 * This function constructs a string that represents the firmware version and additional build details
 * of a device based on its current state and provided flags. It handles both bootloader and normal
 * firmware states, appending relevant versioning and build information.
 *
 * @param devInfo A reference to a `dev_info_t` structure containing device information.
 * @param flags An integer representing various flags that control the output format.
 *              - `COMPACT_BUILD_TYPE`: If set, the build type is represented compactly.
 *              - `OMIT_COMMIT_HASH`: If set, the commit hash is omitted from the output.
 *              - `OMIT_BUILD_KEY`: If set, the build key is omitted from the output.
 *              - `OMIT_BUILD_DATE`: If set, the build date is omitted from the output.
 *              - `OMIT_BUILD_TIME`: If set, the build time is omitted from the output.
 *              - `OMIT_BUILD_MILLIS`: If set, the build milliseconds are omitted from the output.
 *
 * @return A string containing the formatted firmware information.
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

        if (devInfo.repoRevision && !(flags & OMIT_COMMIT_HASH)) {
            out += utils::string_format(" %08x", devInfo.repoRevision);
            if (devInfo.buildType == '^') {
                out += "^";
            }
        }

        if (devInfo.buildNumber && !(flags & OMIT_BUILD_KEY)) {
            // build number/type
            out += utils::string_format(" b%05x.%d", ((devInfo.buildNumber >> 12) & 0xFFFFF), (devInfo.buildNumber & 0xFFF));
        }

        if (!(flags & OMIT_BUILD_DATE)) {
            // build date
            out += utils::string_format(" %04u-%02u-%02u", devInfo.buildYear + 2000, devInfo.buildMonth, devInfo.buildDay);

            if (!(flags & OMIT_BUILD_TIME)) {
                // build time
                out += utils::string_format(" %02u:%02u:%02u", devInfo.buildHour, devInfo.buildMinute, devInfo.buildSecond);
                if (devInfo.buildMillisecond && !(flags & OMIT_BUILD_MILLIS)) {
                    out += utils::string_format(".%03u", devInfo.buildMillisecond);
                }
            }
        }
    }

    return out;
}

std::string ISDevice::getFirmwareInfo(int flags) const {
    return getFirmwareInfo(devInfo, flags);
}

std::string ISDevice::getDescription(int flags) const {
    std::string desc = getName(flags);
    if (!(flags & OMIT_FIRMWARE_VERSION)) {
        desc += " " + getFirmwareInfo(flags);
    }
    if (!(flags & OMIT_PORT_NAME) && portIsValid(port))
        desc += ", " + getPortName() + (isConnected() ? "" : " (Closed)");
    return desc;
}

void ISDevice::registerWithLogger(cISLogger *logger) {
    if (logger) {
        logger->registerDevice(shared_from_this());
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

/**
 * Issues the specified SYS_CMD to the device. Note that this does not confirm or validate whether the requested command
 * was received and processed, only that the command was successfully sent.
 * @param command the command to issue
 * @return 0 on success, -1 on failure
 */
int ISDevice::SetSysCmd(const uint32_t command) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    sysCmd.command = command;
    sysCmd.invCommand = ~command;
    // [C COMM INSTRUCTION]  Update the entire DID_SYS_CMD data set in the IMX.
    log_debug(IS_LOG_ISDEVICE, "[%s] Issuing SYS_CMD %d to %s (%s)", getIdAsString().c_str(), command, getIdAsString().c_str(), getPortName().c_str());
    return SendData(DID_SYS_CMD, &sysCmd, sizeof(system_command_t), 0);
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
    n += static_cast<int>(nmeaMsg.size());
    nmea_sprint_footer(reinterpret_cast<char *>(buf), sizeof(buf), n);
    return SendRaw(buf, n);  // TODO instead?? (SendRaw(buf, n) >= PORT_ERROR__NONE ? 0 : -1);
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

    did_event_t event = {};
    event.time = 123;
    event.senderSN = 0;
    event.senderHdwId = 0;
    event.length = sizeof(did_event_filter_t);

    did_event_filter_t filter = {};
    filter.portMask = portMask,
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
 * Synchronizes the flash configuration for this device, if necessary.
 * This will check the last time the flash config was synchronized, and if it has been longer than the SYNC_FLASH_CFG_CHECK_PERIOD_MS,
 * it will request the flash config from the device and update the local copy.
 */
void ISDevice::SyncFlashConfig()
{
    if (devInfo.hdwRunState != HDW_STATE_APP)
        return;

    std::lock_guard<std::recursive_mutex> lock(portMutex);

    unsigned int timeMs = current_timeMs();
    if (timeMs - syncCheckTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
    {
        return;
    }
    syncCheckTimeMs = timeMs;

    if (devInfo.hardwareType == IS_HARDWARE_TYPE_IMX)
    {   // Sync IMX flash config if a IMX present
        DeviceSyncFlashCfg(timeMs, DID_FLASH_CONFIG,  DID_SYS_PARAMS, imxFlashCfgUploadTimeMs, imxFlashCfg.checksum, sysParams.flashCfgChecksum, imxFlashCfgUploadChecksum);
    }

    if (devInfo.hardwareType == IS_HARDWARE_TYPE_GPX ||
        gpxDevInfo.hardwareType == IS_HARDWARE_TYPE_GPX)
    {   // Sync GPX flash config if a GPX present
        DeviceSyncFlashCfg(timeMs, DID_GPX_FLASH_CFG, DID_GPX_STATUS, gpxFlashCfgUploadTimeMs, gpxFlashCfg.checksum, gpxStatus.flashCfgChecksum, gpxFlashCfgUploadChecksum);
    }
}

int ISDevice::DeviceSyncFlashCfg(unsigned int timeMs, uint16_t flashCfgDid, uint16_t syncDid, unsigned int &uploadTimeMs, uint32_t &flashCfgChecksum, uint32_t &syncChecksum, uint32_t &uploadChecksum)
{
    FnProfiler fn("ISDevice::DeviceSyncFlashCfg()");
    if (devInfo.hdwRunState != HDW_STATE_APP)
        return -1;

    if (uploadTimeMs)
    {   // Upload in progress
        if (timeMs - uploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
        {   // Wait for upload to process.  Pause sync.
            syncChecksum = 0xFFFFFFFF;      // Indicate out of sync
            return -1;
        }
    }

    // Require valid sysParams checksum
    if (ValidFlashCfgCksum(syncChecksum))
    {
        if (ValidFlashCfgCksum(flashCfgChecksum) && (syncChecksum == flashCfgChecksum))
        {   // Checksum is valid and matches
            if (uploadTimeMs)
            {   // Upload complete.  Allow sync.
                bool success = (uploadChecksum == syncChecksum);
                log_debug(IS_LOG_ISDEVICE, "[%s] %s upload %s.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), cISDataMappings::DataName(flashCfgDid), (success ? "complete" : "rejected"));
                uploadTimeMs = 0;
                return (success ? 1 : 0);
            }
        }
        else
        {   // Out of sync.  Request flash config.
            log_debug(IS_LOG_ISDEVICE, "[%s] Out of sync.  Requesting %s...", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), cISDataMappings::DataName(flashCfgDid));
            if (!BroadcastBinaryData(flashCfgDid)) {
               log_error(IS_LOG_ISDEVICE, "[%s] Failed to request %s!", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), cISDataMappings::DataName(flashCfgDid));
            }
        }
    }
    else
    {   // Out of sync.  Request sysParams or gpxStatus.
        log_debug(IS_LOG_ISDEVICE, "[%s] Out of sync.  Requesting %s...", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), cISDataMappings::DataName(syncDid));
        if (!BroadcastBinaryData(syncDid)) {
            log_error(IS_LOG_ISDEVICE, "[%s] Failed to request %s!", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), cISDataMappings::DataName(flashCfgDid));
        }
    }
    return 0;
}

void ISDevice::UpdateFlashConfigChecksum(nvm_flash_cfg_t &flashCfg_)
{
    if (devInfo.hdwRunState != HDW_STATE_APP)
        return;

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
        WaitForImxFlashCfgSynced(true, timeout);
    }

    // Copy flash config
    flashCfg_ = imxFlashCfg;

    // Indicate whether the port connection is valid, open, and the flash config is synchronized; otherwise false
    return ValidFlashCfgCksum(imxFlashCfg.checksum) && (sysParams.flashCfgChecksum == imxFlashCfg.checksum);
}

bool ISDevice::GpxFlashConfig(gpx_flash_cfg_t& flashCfg_, uint32_t timeout)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!isConnected() || (devInfo.hdwRunState != HDW_STATE_APP)) {
        return false;   // No device, no flash config
    }

    // attempt to synchronize, if requested
    if (timeout > 0) {
        WaitForGpxFlashCfgSynced(false, timeout);
    }

    // Copy flash config
    flashCfg_ = gpxFlashCfg;

    // Indicate whether the port connection is valid, open, and the flash config is synchronized; otherwise false
    return ValidFlashCfgCksum(gpxStatus.flashCfgChecksum) && (gpxStatus.flashCfgChecksum == gpxFlashCfg.checksum);
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

    if (imxFlashCfgUploadTimeMs)
        imxFlashCfgUploadChecksum = flashCfg.checksum;

    imxFlashCfg = flashCfg;
    return success;
}

bool ISDevice::SetGpxFlashConfig(gpx_flash_cfg_t& flashCfg) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    bool success = UploadFlashConfigDiff(
        reinterpret_cast<uint8_t*>(&flashCfg),
        reinterpret_cast<uint8_t*>(&gpxFlashCfg),
        sizeof(gpx_flash_cfg_t),
        DID_GPX_FLASH_CFG,
        gpxFlashCfgUploadTimeMs,
        gpxFlashCfgUploadChecksum
    );

    if (gpxFlashCfgUploadTimeMs)
        gpxFlashCfgUploadChecksum = flashCfg.checksum;

    gpxFlashCfg = flashCfg;
    return success;
}

bool ISDevice::UploadFlashConfigDiff(uint8_t* newData, uint8_t* curData, size_t sizeBytes, uint32_t flashCfgDid, uint32_t& uploadTimeMsOut, uint32_t& checksumOut) {
    std::vector<cISDataMappings::MemoryUsage> usageVec;
    const auto& dataSetMap = *cISDataMappings::NameToInfoMap(flashCfgDid);

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
        log_more_debug(IS_LOG_ISDEVICE, "[%s] Sending %s: size %lu, offset %d", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), cISDataMappings::DataName(flashCfgDid), usage.size, offset);
        failure |= (SendData(flashCfgDid, usage.ptr, static_cast<int>(usage.size), offset) != 0);   // SendData() returns 0 on success

        if (!failure)
        {
            uploadTimeMsOut = current_timeMs();
        }
    }

    return !failure;
}

bool ISDevice::ImxFlashConfigSynced() {
    bool checksumValid = ValidFlashCfgCksum(imxFlashCfg.checksum);
    bool checksumsMatch = (imxFlashCfg.checksum == sysParams.flashCfgChecksum);
    bool uploadCompleted = (imxFlashCfgUploadTimeMs == 0);

    bool uploadChecksumValid = ValidFlashCfgCksum(imxFlashCfgUpload.checksum) && imxFlashCfgUpload.checksum;
    bool uploadChecksumsMatch = (imxFlashCfgUpload.checksum == sysParams.flashCfgChecksum);
    bool uploadSuccess = (!uploadChecksumValid || uploadChecksumsMatch); // bool uploadSuccess = !ImxFlashConfigUploadFailure();

    return  checksumValid && checksumsMatch && uploadCompleted && uploadSuccess;
}

bool ISDevice::GpxFlashConfigSynced() {
    return  ValidFlashCfgCksum(gpxFlashCfg.checksum) &&
            (gpxFlashCfg.checksum == gpxStatus.flashCfgChecksum) &&
            (gpxFlashCfgUploadTimeMs == 0) &&
            !GpxFlashConfigUploadFailure();
}

bool ISDevice::ImxFlashConfigUploadFailure() {
    // a failed flash upload is considered when imxFlashCfgUploadChecksum is non-zero, and DOES NOT match sysParams.flashCfgChecksum
    return  ValidFlashCfgCksum(imxFlashCfgUpload.checksum) &&
            imxFlashCfgUpload.checksum &&
            (imxFlashCfgUpload.checksum != sysParams.flashCfgChecksum);
}
bool ISDevice::GpxFlashConfigUploadFailure() {
    // a failed flash upload is considered when gpxFlashCfgUploadChecksum is non-zero, and DOES NOT match gpxStatus.flashCfgChecksum
    return  ValidFlashCfgCksum(gpxFlashCfgUpload.checksum) &&
            gpxFlashCfgUpload.checksum &&
            (gpxFlashCfgUpload.checksum != gpxStatus.flashCfgChecksum);
}

/**
 * A blocking function calls which waits until both a DID_FLASH_CFG and DID_SYS_PARAMS have
 * been received which have a matching flashCfg checksum; ensuring that we have a valid copy
 * of the devices' flash configuration.
 * @param forceSync if true, invalidates any existing checksum ensuring the both messages must be received and validated
 * @param timeout the maximum time to wait for the synchronization to occur, before returning false
 * @return true if both the flashCfg.checksum and sysParams.flashCfgChecksum match (and neither are zero)
 */
bool ISDevice::WaitForImxFlashCfgSynced(bool forceSync, uint32_t timeoutMs)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!port)
        return false;   // No device, no flash-sync

    if (devInfo.hdwRunState != HDW_STATE_APP)
        return false;   // Device not running application firmware

    if (forceSync)
        sysParams.flashCfgChecksum = 0xFFFFFFFF;    // Invalidate to force re-sync

    // If there are no upload pending, then just go ahead and check...
    unsigned int startMs = current_timeMs();
    while(!ImxFlashConfigSynced())
    {   // Request and wait for IMX flash config
        step();
        SLEEP_MS(5);

        int elaspedMs = (int)(current_timeMs() - startMs);
        if (elaspedMs > (int)timeoutMs)
        {   // Timeout waiting for IMX flash config
            log_warn(IS_LOG_ISDEVICE, "[%s] Timeout waiting for DID_FLASH_CONFIG to sync! (%d ms elapsed, 0x%08x (FlashCfg) != 0x%08x (SysParams)", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), elaspedMs, imxFlashCfg.checksum, sysParams.flashCfgChecksum);
            return false;
        }
        else
        {   // Query DID_SYS_PARAMS
            GetData(DID_SYS_PARAMS);
            log_bombastic(IS_LOG_ISDEVICE, "[%s] Waiting for IMX flash sync...", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
        }
    }

    return ImxFlashConfigSynced();
}

bool ISDevice::WaitForGpxFlashCfgSynced(bool forceSync, uint32_t timeout)
{
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!port)
        return false;   // No device, no flash-sync

    if (devInfo.hdwRunState != HDW_STATE_APP)
        return false;   // Device not running application firmware

    if (forceSync)
        gpxStatus.flashCfgChecksum = 0xFFFFFFFF;    // Invalidate to force re-sync

    // If there are no upload pending, then just go ahead and check...
    unsigned int startMs = current_timeMs();
    while(!GpxFlashConfigSynced())
    {   // Request and wait for GPX flash config
        step();
        SLEEP_MS(10);

        if (current_timeMs() - startMs > timeout)
        {   // Timeout waiting for GPX flash config
            log_info(IS_LOG_ISDEVICE, "[%s] Timeout waiting for DID_GPX_FLASH_CONFIG to sync!", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
            return false;
        }
        else
        {   // Query DID_GPX_STATUS
            GetData(DID_GPX_STATUS);
            log_bombastic(IS_LOG_ISDEVICE, "[%s] Waiting for GPX flash sync...", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
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
 * @param timeout the maximum amount of time to wait fo the sync before returning false
 * @param waitForWrite if true, will also wait for the PENDING_WRITE flag to clear, indicating
 *   that the configuration is saves to flash (not just RAM). Note that writing to NVM Flash
 *   will occur automatically after some period of time, even if this is false. A common pattern
 *   is to change config, and then reset but if the caller resets before the config is written to
 *   NVM, even if it reports as synchronized, the subsequent reset may prevent the configuration
 *   was being persisted, and the reset will lose that change. This parameter is a convenience for
 *   operations in which the caller needs confirmation that the config is written to NVM, and
 *   safely persisted across resets.
 * @return true if Cfg was successfully sent and sync, and (optionally) written to device flash
 */
bool ISDevice::SetImxFlashCfgAndConfirm(nvm_flash_cfg_t& flashCfg, uint32_t timeout, bool waitForWrite) {
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

    if (waitForWrite && !waitForImxFlashWrite(timeout))
        return false;   // tricky - if no FLash Write was required, this will return false, even though we successfully updated the config.

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
bool SaveFlashConfigToFile(const std::string& path, int flashCfgDid, std::function<bool(T&)> getCfg)
{
    T flashCfg;
    if (!getCfg(flashCfg))
    {
        log_error(IS_LOG_ISDEVICE, "Failed to get flash config.");
        return false;
    }

    YAML::Node yaml;
    if (!cISDataMappings::DataToYaml(flashCfgDid, reinterpret_cast<const uint8_t*>(&flashCfg), yaml))
    {
        log_error(IS_LOG_ISDEVICE, "Failed to serialize flash config to YAML.");
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
bool LoadFlashConfigFromFile(const std::string& path, int flashCfgDid, std::function<bool(T&)> setCfg)
{
    try
    {
        YAML::Node yaml = YAML::LoadFile(path);
        T flashCfg;
        if (!cISDataMappings::YamlToData(flashCfgDid, yaml, reinterpret_cast<uint8_t*>(&flashCfg)))
        {
            log_error(IS_LOG_ISDEVICE, "Failed to parse YAML into flash config structure");
            return false;
        }

        if (!setCfg(flashCfg))
        {
            log_error(IS_LOG_ISDEVICE, "Failed to apply flash config");
            return false;
        }
    }
    catch (const YAML::Exception& ex)
    {
        log_error(IS_LOG_ISDEVICE, "There was an error parsing the YAML file: %s", ex.what());
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

/**
 * @brief Uploads sensor calibration data to a device in steps.
 * @param port The communication port handle.
 * @param calUploadState The current state of the upload process. This is updated by the function.
 * @param cal The sensor_cal_t structure containing the calibration data to upload.
 * @return 1 if the upload is complete, 0 if it's in progress, -1 on error.
 */
int ISDevice::UploadImxCalibrationAsync(ISDeviceCal& cal) {
    if (!isConnected())
        return ASYNC_STATE__FAILURE;   // nothing to do, if we aren't connected

    return ISDeviceCal::uploadSensorCalStep(port, m_calUploadState, cal);
}


bool ISDevice::UploadImxCalibration(ISDeviceCal& cal)
{
    if (!isConnected())
        return false;   // nothing to do, if we aren't connected

    int result = 0;
    try {
        int calUploadState = 0;
        do {
            result = ISDeviceCal::uploadSensorCalStep(port, calUploadState, cal);
            SLEEP_MS(ISDeviceCal::CAL_UPLOAD_SLEEP_MS);
        } while (result == ASYNC_STATE__PENDING);
    } catch (const std::exception& e) {
        log_error(IS_LOG_ISDEVICE, "[%s] Calibration upload failed!", getIdAsString().c_str());
        return false;
    }

    if (result == ASYNC_STATE__SUCCESS)
    {
        log_info(IS_LOG_ISDEVICE, "[%s] Calibration upload complete.", getIdAsString().c_str());
        return true;
    }
    else
    {
        log_error(IS_LOG_ISDEVICE, "[%s] Calibration upload failed!", getIdAsString().c_str());
        return false;
    }
}


bool ISDevice::UploadImxCalibrationFromFile(std::string path)
{
    ISDeviceCal cal(path);
    return UploadImxCalibration(cal);
}

bool ISDevice::UploadImxCalibrationFromJson(const nlohmann::json& calJson)
{
    ISDeviceCal cal(calJson);
    return UploadImxCalibration(cal);
}

int ISDevice::UploadIMXCalibrationFromURL(const std::string& restBaseUrl)
{

    ISDeviceCal cal{};
    ISHttpRequest::Response calResp = cal.loadFromURL(restBaseUrl, devInfo);
    switch (calResp.statusCode) {
        case -2:
            log_error(IS_LOG_ISDEVICE, "[%s] Failed to parse calibration response.", getIdAsString().c_str());
            return calResp.statusCode;
        case -1:
            log_error(IS_LOG_ISDEVICE, "[%s] Failed to connect to calibration DB at %s", getIdAsString().c_str(), restBaseUrl.c_str());
            return calResp.statusCode;
        case 404:
            log_warn(IS_LOG_ISDEVICE, "[%s] Device not found in calibration DB", getIdAsString().c_str());
            return calResp.statusCode;
        case 200: {
            int pos = calResp.statusMessage.find("] ");
            auto msg = calResp.statusMessage.substr(pos+2);
            log_info(IS_LOG_ISDEVICE, "[%s] %s", getIdAsString().c_str(), msg.c_str());
            break; // success
        }
        default:
            log_error(IS_LOG_ISDEVICE, "[%s] Calibration DB returned HTTP %d: %s", getIdAsString().c_str(), calResp.statusCode, calResp.statusMessage.c_str());
            return calResp.statusCode;
    }

    return UploadImxCalibration(cal) ? 200 : -1;
}

bool ISDevice::softwareReset() {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!isConnected() || (nextResetTime && (nextResetTime - current_timeMs() > 0)))
        return false;

    log_info(IS_LOG_ISDEVICE, "[%s] Requesting Software Reset", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
    for (int i = 0; i < 3; i++) {   // we shouldn't need to do this 3 times...
        if (SetSysCmd(SYS_CMD_SOFTWARE_RESET))
            break;
        SLEEP_MS(5)
    }
    disconnect();
    nextResetTime = current_timeMs() + resetRequestThreshold;
    return true;
}

bool ISDevice::manufacturingInfo(manufacturing_info_t& info, uint32_t timeoutMs) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    if (!isConnected()) {
        return false;   // No device, no flash config
    }

    log_info(IS_LOG_ISDEVICE, "[%s] Requesting Manufacturing Info", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());

    int startTime = current_timeMs();
    while ((int)current_timeMs() - startTime < (int)timeoutMs) {
        if (manfInfo.hardwareId) {
            info = manfInfo;
            return true;
        }

        GetData(DID_MANUFACTURING_INFO);
        SLEEP_MS(5);
        step();
    }

    return false;
}

int ISDevice::onIsbDataHandler(p_data_t* data, port_handle_t port)
{
    if ((data->hdr.size==0) || (data->ptr==NULL))
        return 0;   // this message is invalid, so don't let anything else try and handle it...

    std::lock_guard<std::recursive_mutex> lock(portMutex);
    if (devLogger) {
        // FIXME:  devLogger->SaveData(data, 0);
        // stepLogFunction(s_cm_state->inertialSenseInterface, data, port);
    }

    markRxTs();
    sampleIsbMsgStats(*data);
    // printf("DID: %d\n", data->hdr.id);
    switch (data->hdr.id) {
        case DID_DEV_INFO:
            devInfo = *(dev_info_t*)data->ptr;
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
            if (devInfo.hdwRunState == HDW_STATE_UNKNOWN)   // this value should be passed from the device, but if not...
                devInfo.hdwRunState = HDW_STATE_APP;        // since this is ISB, its pretty safe to assume that we are in APP mode.
            break;
        case DID_GPX_DEV_INFO:
            gpxDevInfo = *(dev_info_t*)data->ptr;
            log_more_debug(IS_LOG_ISDEVICE, "[%s] Received DID_GPX_DEV_INFO: hwType=%d, fw=%d.%d.%d.%d",
                getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(),
                gpxDevInfo.hardwareType, gpxDevInfo.firmwareVer[0], gpxDevInfo.firmwareVer[1],
                gpxDevInfo.firmwareVer[2], gpxDevInfo.firmwareVer[3]);
            break;
        case DID_SYS_CMD:
            sysCmd = *(system_command_t*)data->ptr;
            break;
        case DID_SYS_PARAMS:
            copyDataPToStructP(&sysParams, data, sizeof(sys_params_t));
            log_bombastic(IS_LOG_ISDEVICE, "[%s] Received DID_SYS_PARAMS", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());    // this ones bombastic, because it can come every millisecond.
            break;
        case DID_FLASH_CONFIG:
            copyDataPToStructP(&imxFlashCfg, data, sizeof(nvm_flash_cfg_t));
            if ( dataOverlap(offsetof(nvm_flash_cfg_t, checksum), 4, data)) {
                sysParams.flashCfgChecksum = imxFlashCfg.checksum;
            }
            log_more_debug(IS_LOG_ISDEVICE, "[%s] Received DID_FLASH_CONFIG", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
            break;
        case DID_GPX_STATUS:
            copyDataPToStructP(&gpxStatus, data, sizeof(gpx_status_t));
            log_more_debug(IS_LOG_ISDEVICE, "[%s] Received DID_GPX_STATUS", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
            break;
        case DID_GPX_FLASH_CFG:
            copyDataPToStructP(&gpxFlashCfg, data, sizeof(gpx_flash_cfg_t));
            if ( dataOverlap( offsetof(gpx_flash_cfg_t, checksum), 4, data ) )
            {	// Checksum received
                gpxStatus.flashCfgChecksum = gpxFlashCfg.checksum;
            }
            log_more_debug(IS_LOG_ISDEVICE, "[%s] Received DID_GPX_FLASH_CFG", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
            break;
        case DID_MANUFACTURING_INFO:
            copyDataPToStructP(&manfInfo, data, sizeof(manufacturing_info_t));
            break;

        case DID_FIRMWARE_UPDATE:
            if (fwUpdater)
                fwUpdater->processMessage(data);
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

    markRxTs(_PTYPE_NMEA);
    switch (getNmeaMsgId(msg, msgSize))
    {
        case NMEA_MSG_ID_INFO:
        {	// IMX device Info
            dev_info_t info = {};
            nmea_parse_info(info, (const char*)msg, msgSize);
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
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
        }
        break;
    }
    return 1;   // allow others to continue to process this message
}

int ISDevice::onPacketHandler(protocol_type_t ptype, packet_t *pkt, port_handle_t port) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);
    markRxTs(ptype);

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

    if ((portFlags(newPort) & PORT_FLAG__NO_ISDEVICE)) {
        return false;   // we cannot assign a NO_ISDEVICE port to an ISDevice (We would probably make the device or the other-end of the port unhappy)
    }

    port = newPort;
    if (!portIsValid(port)) {
        return false;   // nothing more to do if the port is invalid
    }

    if ((portType(newPort) & PORT_TYPE__COMM)) {
        originalCbs = COMM_PORT(newPort)->comm.cb; // make a copy of the new port's original callbacks/context, which we'll restore when this device is destroyed
    }

    registerAllHandler(processPacket);
    registerIsbDataHandler(processIsbMsgs);
    // registerIsbAckDataHandler(processIsbAckMsgs);
    registerProtocolHandler(_PTYPE_NMEA, processNmeaMsgs);

    is_comm_callbacks_t portCbs = defaultCbs;

    // Initialize IScomm instance, for serial reads / writes
    if ((portType(port) & PORT_TYPE__COMM)) {
        comm_port_t* comm = COMM_PORT(port);

        is_comm_init(&(comm->comm), comm->buffer, sizeof(comm->buffer), portCbs.all);
        is_comm_register_port_callbacks(port, &portCbs);
    }
    return true;
}

/**
 * Connects the bound port to the device, if the port is valid and of PORT_TYPE__COMM
 * Can be overridden to provide custom configuration, etc on connection - just remember
 *  to call back into ISDevice::connect() in your new method.
 * @param revalidate if true causes the device to validate after connecting (default = false)
 * @return true if the connection is made/port opened, otherwise false
 */
bool ISDevice::connect(bool revalidate) {

    if (!portIsValid(port) || !(portType(port) & PORT_TYPE__COMM))      // TODO?? Generally, device MUST use a COMM port, but ISbl is NOT a COMM protocol
        return false;   // port is invalid or incorrect type, so we can't connect it

    imxFlashCfgUpload = {};
    gpxFlashCfgUpload = {};
    imxFlashCfgUploadTimeMs = 0;
    gpxFlashCfgUploadTimeMs = 0;
    imxFlashSyncCheckTimeMs = 0;
    gpxFlashSyncCheckTimeMs = 0;
    imxFlashCfgUploadChecksum = 0;
    gpxFlashCfgUploadChecksum = 0;

    if (revalidate)
        devInfo.hdwRunState = HDW_STATE_UNKNOWN; // this will further reinforce a validation

    bool alreadyOpened = portIsOpened(port);
    if (!alreadyOpened) {
        if (nextConnectMs > current_timeMs()) {
            SLEEP_MS(15);
            log_debug(IS_LOG_ISDEVICE, "Connection throttled. You can retry this device again in %dms.", nextConnectMs - current_timeMs());
            return false;   // don't attempt to reconnect until nextConnectMs has expired
        }

        if (portOpen(port) != PORT_ERROR__NONE) {
            SLEEP_MS(15);
            nextConnectMs = current_timeMs() + 500; // if the connect fails, delay 500ms before trying again.
            log_debug(IS_LOG_ISDEVICE, "Device failed to connect... You can retry this device again in %dms.", nextConnectMs - current_timeMs());
            return false;
        }
    }

    bool success = true;            // if we've connected, we're successful -- until we try and validate - if we try and validate
    portStatsReset(port);           // always reset port stats with a reconnect (even if we were already connected)
    if (revalidate) {
        SLEEP_MS(15);
        success = validate();          // if validating, only return true if we successfully validated
    }

    // ONLY notify of DEVICE_CONNECTED, if the port was closed at the start of this function
    if (!alreadyOpened) {
        DeviceManager::getInstance().notifyListeners(shared_from_this(), DeviceManager::DEVICE_CONNECTED);  // notify that we've connected (if we are)
        log_debug(IS_LOG_ISDEVICE, "Connected to ISDevice::%s%s", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), (success ? " (revalidated)" : ""));
    }
    was_connected = success;
    return success;
}

pfnIsCommHandler ISDevice::registerAllHandler(pfnIsCommHandler cbHandler) {
    std::lock_guard<std::recursive_mutex> lock(portMutex);

    pfnIsCommHandler oldHandler = defaultCbs.all;
    defaultCbs.context = this;
    defaultCbs.all = cbHandler;

    if (port && (portType(port) & PORT_TYPE__COMM)) {
        COMM_PORT(port)->comm.cb.context = this;
        oldHandler = is_comm_register_all_handler(&COMM_PORT(port)->comm, cbHandler);
    }

    return oldHandler;
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

/**
 * blocks until the pending flashConfig changes have been successfully written to the device.
 * @return true if a pending write was detected and cleared, otherwise false.  NOTE that this
 * may return false if previous pending writes were successfully written prior to calling this
 * function. To be truly effective, this call should be made immediately after a call to
 * SetImxFlashConfig()
 */
bool ISDevice::waitForImxFlashWrite(uint32_t timeoutMs)
{
    bool pendingWrites = false;
    uint32_t writeAge = 0;

    if (!isConnected())
        return false;

    // StopBroadcasts();   // TODO: do we really want to stop broadcasts??  probably not...

    // First, let's assume that haven't received a PENDING_FLASH_WRITES, but that we will within 250ms
    for (int i = 0; i < 5 && !pendingWrites; i++) {
        BroadcastBinaryData(DID_SYS_PARAMS, 0);
        SLEEP_MS(50);   // give a millisecond or 50 for the device to respond.
        step();
        pendingWrites = hasPendingImxFlashWrites(writeAge);
    }
    if (pendingWrites == false)
        return false;   // we never got a message that writes were pending... maybe there aren't any?

    // At this point, pendingWrites must be true, so now we wait for it to clear, or timeout to occur
    unsigned int startTimeMs = current_timeMs();
    do {
        BroadcastBinaryData(DID_SYS_PARAMS, 0);
        SLEEP_MS(50);   // give a millisecond or 50 for the device to respond.
        step();

        if (!hasPendingImxFlashWrites(writeAge))
            return true;    // no more pendingWrites, so return true that's we've seen it clear
    } while ((current_timeMs() - startTimeMs) < timeoutMs);
    return false;
}

double ISDevice::sampleIsbMsgStats(const p_data_t& data) {

    auto& stat = didStats[data.hdr.id];
    stat.accrual += (data.hdr.size + ISB_MIN_PACKET_SIZE + (data.hdr.offset ? 2 : 0));
    switch (data.hdr.id)
    {
        case DID_GPX_STATUS:        stat.sample( ((gpx_status_t*)data.ptr)->upTime );       break;
        case DID_SYS_PARAMS:        stat.sample( ((sys_params_t*)data.ptr)->upTime );       break;
        case DID_INS_1:             stat.sample( ((ins_1_t*)data.ptr)->timeOfWeek );        break;
        case DID_INS_2:             stat.sample( ((ins_2_t*)data.ptr)->timeOfWeek );        break;
        case DID_INS_3:             stat.sample( ((ins_3_t*)data.ptr)->timeOfWeek );        break;
        case DID_INS_4:             stat.sample( ((ins_4_t*)data.ptr)->timeOfWeek );        break;
        case DID_INL2_STATES:       stat.sample( ((inl2_states_t*)data.ptr)->timeOfWeek );  break;
        case DID_INL2_MAG_OBS_INFO: stat.sample( ((inl2_mag_obs_info_t*)data.ptr)->timeOfWeekMs * 0.001 );  break;
        case DID_IMU:               stat.sample( ((imu_t*)data.ptr)->time );                break;
        case DID_IMU_RAW:           stat.sample( ((imu_t*)data.ptr)->time );                break;
        case DID_PIMU:              stat.sample( ((pimu_t*)data.ptr)->time );               break;
        case DID_MAGNETOMETER:      stat.sample( ((magnetometer_t*)data.ptr)->time );       break;
        case DID_BAROMETER:         stat.sample( ((barometer_t*)data.ptr)->time );          break;
        case DID_SYS_SENSORS:       stat.sample( ((sys_sensors_t*)data.ptr)->time );        break;
        case DID_GPS1_POS:
        case DID_GPS2_POS:          stat.sample( ((gps_pos_t*)data.ptr)->timeOfWeekMs * 0.001 );    break;
        case DID_GPS1_VEL:
        case DID_GPS2_VEL:          stat.sample( ((gps_vel_t*)data.ptr)->timeOfWeekMs * 0.001 );    break;
        case DID_GPS1_SAT:
        case DID_GPS2_SAT:          stat.sample( ((gps_sat_t*)data.ptr)->timeOfWeekMs * 0.001 );    break;
        case DID_GPS1_SIG:
        case DID_GPS2_SIG:          stat.sample( ((gps_sig_t*)data.ptr)->timeOfWeekMs * 0.001 );    break;
        case DID_GPS1_RTK_POS_REL:
        case DID_GPS2_RTK_CMP_REL:
            if (((gps_rtk_rel_t*)data.ptr)->timeOfWeekMs != 0)
                stat.sample( ((gps_rtk_rel_t*)data.ptr)->timeOfWeekMs * 0.001 );
            break;
        case DID_GPS1_RTK_POS_MISC:
        case DID_GPS2_RTK_CMP_MISC:
            if (((gps_rtk_rel_t*)data.ptr)->timeOfWeekMs != 0)
                stat.sample( ((gps_rtk_misc_t*)data.ptr)->timeOfWeekMs * 0.001 );
            break;
        default:                    stat.sample();  break;
    }
    return stat.lastSampleTime();
}
