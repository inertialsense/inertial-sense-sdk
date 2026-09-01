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

    if (was_connected != (bool)portIsOpened(port)) {
        was_connected = isConnected();
        if (!isConnected() && m_calibration) {
            // Port went away mid-upload. Drop the cal data and surface IS_OP_CLOSED so the
            // caller can distinguish this from a protocol-layer failure.
            log_warn(IS_LOG_ISDEVICE, "[%s] Async calibration upload aborted: port closed mid-upload at step %d.", getIdAsString().c_str(), m_calUploadState);
            m_calibration.reset();
            m_calUploadState = -1;
            m_calUploadResult = IS_OP_CLOSED;
        }
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
        const ISDeviceCal::AsyncState stepResult = ISDeviceCal::uploadSensorCalStep(port, m_calUploadState, *m_calibration, devInfo, m_calibration->uploadCtx);
         if (stepResult == ISDeviceCal::ASYNC_STATE__PENDING) {
             SLEEP_MS(ISDeviceCal::CAL_UPLOAD_SLEEP_MS);   // Give the device a break between upload steps, important for uploads over UART.
         } else {
            if (stepResult == ISDeviceCal::ASYNC_STATE__SUCCESS) {
                m_calUploadResult = IS_OP_OK;
                log_info(IS_LOG_ISDEVICE, "[%s] Async calibration upload complete.", getIdAsString().c_str());
            } else {
                m_calUploadResult = IS_OP_ERROR;
                log_error(IS_LOG_ISDEVICE, "[%s] Async calibration upload failed at step %d.", getIdAsString().c_str(), m_calUploadState);
            }
            m_calibration.reset();
            m_calUploadState = -1;
        }
        didStuff = true;
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

    // If a prior updater exists and is still running, refuse to overwrite it.
    // If it's done or cancelled, clean it up and start fresh — the IOManager tick
    // would do the same cleanup, but may not have run yet.
    if (fwUpdater) {
        if (!fwUpdater->fwUpdate_isDone())
            return IS_OP_IN_PROGRESS;
        delete fwUpdater;
        fwUpdater = nullptr;
    }

    fwUpdateState.resetState();
    fwUpdater = new ISFirmwareUpdater(shared_from_this(), fwUpdateState);

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

bool ISDevice::handshakeISbl(port_handle_t port, int burstCount) {
    static const uint8_t handshakerChar = 'U';
    uint8_t readCh = 0;

    // Costs burstCount * BOOTLOADER_HANDSHAKE_DELAY ms when the bootloader does not echo, which is the
    // normal case (see the header). Callers on a deadline size burstCount accordingly.

    // log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::handshakeISbl() called.", getIdAsString().c_str());

    // first, flush all incoming data and ensure we have a clean buffer...
    for (int i = 0; i < 5; i++) {
        if (portAvailable(port))
            portFlush(port);

        if ((i == 4) && portAvailable(port)) {
            log_warn(IS_LOG_ISDEVICE, "[%s] handshakeISbl() is unable to clear the port RX buffer. Handshaking is not possible.", portName(port));
            return false;   // unable to clear the buffer, so no handshake happened
        }
    }

    // Bootloader sync requires at least 6 'U' characters to be sent every 10ms.
    // write a 'U' to handshake with the bootloader - once we get a 'U' back we are ready to go
    for (int i = 0; i < burstCount; i++) {
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

    // No echo is the NORMAL case for an already-synchronized bootloader -- it echoes once per sync -- so
    // this is reported rather than treated as a fault. Callers must not gate on it; the version response is
    // the only evidence of synchronization.
    return false;
}

bool ISDevice::queryIsblVersionFrame(port_handle_t port, uint8_t frame[14], uint32_t budgetMs, isbl_probe_t* detail)
{
    isbl_probe_t local;
    isbl_probe_t& d = detail ? *detail : local;
    const uint32_t startMs = current_timeMs();

    // Wait for the port to be open rather than bailing on the first !portIsOpened(): on an asynchronous
    // transport "not open yet" is transient, since a non-blocking connect returns PORT_ERROR__NONE while
    // the handshake is still in flight and leaves PORT_FLAG__OPENED clear. portOpenRetry() returns
    // immediately when the port is already open, so a local serial port pays nothing for this.
    uint32_t openWaitMs = budgetMs / 4;
    if (openWaitMs > ISBL_PORT_OPEN_WAIT_MS)  openWaitMs = ISBL_PORT_OPEN_WAIT_MS;
    if (portOpenRetry(port, openWaitMs, 10) != PORT_ERROR__NONE)
        return false;       // never became usable; d.portOpened stays false so the caller can say so
    d.portOpened = true;

    // Both halves of the schedule -- how long one read may block, and how long a burst may run -- are
    // derived from the caller's budget, because the two classes of caller differ by more than an order of
    // magnitude. A dedicated probe (the ISv1 path) hands over seconds and wants the generous read a relayed
    // port needs; validate()/validateAsync() hand over ~250ms for ONE step of a round-robin that also tries
    // NMEA and ISB, and expect to be re-entered. Capping the read at a quarter of the budget guarantees the
    // first pass can still afford a recovery burst, which is then truncated to whatever remains.
    uint32_t readMs = budgetMs / 4;
    if (readMs > ISBL_VERSION_READ_TIMEOUT_MS)  readMs = ISBL_VERSION_READ_TIMEOUT_MS;
    if (readMs < ISBL_VERSION_MIN_READ_MS)      readMs = ISBL_VERSION_MIN_READ_MS;

    while (d.attempts < ISBL_VERSION_MAX_ATTEMPTS) {
        if ((current_timeMs() - startMs) >= budgetMs)
            return false;       // the previous round's re-sync consumed the budget
        d.attempts++;

        // Terminate any PARTIAL COMMAND already sitting in the bootloader's parser, then clear our own RX
        // buffer. Both halves are needed and they do different jobs: the newlines end a truncated command
        // on the DEVICE (otherwise the version query below is appended to that fragment and rejected,
        // which is what happens in the ISv2 path, where validate() has just probed the port with NMEA/ISB
        // traffic), while the flush stops a stale 0xAA55 frame from an earlier probe satisfying this
        // attempt and reporting a version it never obtained.
        for (int i = 0; i < 5; i++) {
            if (portWrite(port, (uint8_t*)"\n", 1) == 1) {
                SLEEP_MS(2);
                if (portAvailable(port))
                    portFlush(port);
            }
        }
        portFlush(port);

        memset(frame, 0, 14);
        if (portWrite(port, (uint8_t*)":020000041000EA", 15) != 15)
            return false;       // the port is not usable; retrying cannot help

        // readMs is the nominal cost of one read; what this read may actually spend is whatever is LEFT.
        // The loop-top check can pass with only a few ms remaining, and the clear sequence above spends
        // ~10ms more, so a read allowed its full nominal cost returns after the caller's deadline.
        uint32_t readElapsed = current_timeMs() - startMs;
        if (readElapsed >= budgetMs)
            return false;
        uint32_t thisReadMs = budgetMs - readElapsed;
        if (thisReadMs > readMs)  thisReadMs = readMs;

        int count = portReadTimeout(port, frame, 14, thisReadMs);
        if ((count < 0) || !portIsOpened(port))
            return false;       // hard port error, or the transport dropped under us

        if ((count >= 8) && (frame[0] == 0xAA) && (frame[1] == 0x55)) {
            // A version 6+ reply is the full 14 bytes ending ".\r\n", and the processor type, EVB flag and
            // serial number live inside that frame -- so a short or unterminated one is retried rather
            // than half-parsed. Pre-v6 bootloaders never send those fields and are accepted on the header.
            bool full = (count >= 14) && (frame[11] == '.') && (frame[12] == '\r') && (frame[13] == '\n');
            if ((frame[2] < 6) || full)
                return true;
        }

        // Unanswered, so the bootloader may not be synchronized. Send a burst and ask again. Its result is
        // recorded but never gates anything -- an already-synchronized bootloader cannot acknowledge it.
        // A burst truncated by the budget is still worth sending: the bootloader's autobaud accumulates
        // across bursts (roughly 10-15 of them at 921600), and a short-budget caller is re-entered
        // repeatedly, so each revolution contributes. A burst with no time left to ask again afterwards
        // is not -- so one minimum read is held back, and a burst that cannot fit is not sent at all.
        uint32_t elapsed = current_timeMs() - startMs;
        if (elapsed >= budgetMs)
            return false;
        uint32_t burstBudget = budgetMs - elapsed;
        burstBudget = (burstBudget > ISBL_VERSION_MIN_READ_MS) ? (burstBudget - ISBL_VERSION_MIN_READ_MS) : 0;
        int burstCount = (int)(burstBudget / BOOTLOADER_HANDSHAKE_DELAY);
        if (burstCount > BOOTLOADER_HANDSHAKE_COUNT)
            burstCount = BOOTLOADER_HANDSHAKE_COUNT;
        if (burstCount <= 0)
            return false;

        d.handshakes++;
        if (handshakeISbl(port, burstCount))
            d.handshakeAcked = true;
    }

    // The attempt cap, not the budget. Reaching it means reads and bursts are both returning far faster
    // than their nominal cost, which is a transport problem rather than an unsynchronized bootloader.
    log_warn(IS_LOG_ISDEVICE, "[%s] queryIsblVersionFrame() gave up after %d attempts in %dms (budget %dms) "
                              "-- reads are not consuming their timeout; check the transport.",
             portName(port), d.attempts, current_timeMs() - startMs, budgetMs);
    return false;
}

bool ISDevice::queryDeviceInfoISbl(uint32_t timeout) {
    uint8_t buf[64] = {};

    FnProfiler fn("ISDevice::queryDeviceInfoISbl() [" + getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO) + "]", timeout / 2 * 1000);

    // Version-first negotiation, shared with the ISv1 path -- see queryIsblVersionFrame().
    isbl_probe_t probe;
    if (queryIsblVersionFrame(port, buf, timeout, &probe)) {
        fn.mark("Got a response (" + std::to_string(probe.attempts) + " attempt(s), " +
                std::to_string(probe.handshakes) + " handshake(s), " +
                (probe.handshakeAcked ? "acknowledged" : "not acknowledged") + ").");

        // The processor type, EVB flag and serial number live in the v6+ tail, so only a full frame can
        // populate devInfo. queryIsblVersionFrame() also accepts a pre-v6 header-only reply, which
        // carries none of those -- parse nothing from it rather than reading uninitialised bytes.
        if (buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n') {
            {
                // firmwareVer is assigned HERE, inside the validity guard, and not as soon as the 0xAA55
                // prefix matches: a reply that is long enough and correctly prefixed but has a malformed
                // tail must not mutate devInfo. A failed query has no business dirtying the identity it was
                // only meant to read -- stranded bootloader version bytes in an otherwise APP-mode devInfo
                // produce a mixed identity, and ISv2 trusts hdwRunState enough to fire an APP-mode reset at
                // a device already sitting in ISbl.
                devInfo.firmwareVer[0] = buf[2];
                devInfo.firmwareVer[1] = buf[3];
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
                        // STM32U5 hardware (IMX-6 and GPX-1) uses mcuBoot, not ISbl —
                        // reaching this branch indicates an unexpected/legacy state.
                        // Don't assign a misleading default like "GPX-1.0" or "IMX-1.0":
                        // both type and version are ambiguous from the bootloader response
                        // alone, and a wrong combination can mis-route firmware images.
                        // Leave hardwareType/hardwareVer untouched so any previously-cached
                        // identity (set by an APP-state validation upstream) survives.
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
                markDevInfoConfirmed();     // a full version frame is a real answer from the bootloader
                return true;
            }
        }
    }

    fn.mark("No version response.");
    // log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::queryDeviceInfoISbl() no valid response received - Either not an ISDevice, or not in ISbootloader.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
    return false;
}


void ISDevice::markDevInfoConfirmed() {
    devInfoConfirmedMs = current_timeMs();
}

bool ISDevice::validate(uint32_t timeout) {
    // validate() is its own blocking implementation rather than a wrapper around validateAsync(), so
    // it needs the guard too -- see disableValidation(). Silent, because a caller that disabled
    // validation asked for exactly this and does not need telling each time.
    if (doNotValidate)
        return false;

    if (!isConnected()) {
        // INFO, not debug: validating a closed port is a caller mistake in the same class as querying
        // one, and not a hard failure -- but the caller should hear about it. This return also precedes
        // every other trace in this function, so a silent exit here is indistinguishable from
        // "validation ran and failed", which is a painful thing to debug.
        log_info(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) : port not valid/open (isConnected() false); cannot validate.",
                 getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);
        return false;
    }

    FnProfiler fn("ISDevice::queryDeviceInfoISbl() [" + getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO) + "]", timeout / 2 * 1000);    // this shouldn't really ever take longer than 50ms to execute
    log_more_debug(IS_LOG_ISDEVICE, "[%s] ISDevice::validate(%d) called.", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(), timeout);

    // Check the discovery hint for this port (set by RelayPortFactory or similar). The
    // hint is informative only — we still actively validate — but it lets us pick the
    // most-efficient initial query type. This matters for bootloader devices: NMEA and
    // DID queries can leave a bootloader in an unresponsive state for a window, so when
    // the hint says HDW_STATE_BOOTLOADER we start with the ISBL query to land cleanly.
    const dev_info_t* hint = DeviceManager::getInstance().getDeviceHint(port);

    // check for Inertial-Sense App by making an NMEA request (which it should respond to)
    is_hardware_t oldHdwId = hdwId;
    dev_info_t oldDevInfo = devInfo;
    hdwId = IS_HARDWARE_NONE,  devInfo = {};    // force a fresh check, don't just take previous values.
    clearDevInfoConfirmed();                    // and nothing below may report success on the old value

    bool hasDevInfo = hasDeviceInfo();
    queryType nextQueryType = (hint && hint->hdwRunState == HDW_STATE_BOOTLOADER)
                              ? QUERYTYPE_ISbootloader
                              : QUERYTYPE_NMEA;
    unsigned int startTime = current_timeMs();
    do {
        if ((current_timeMs() - startTime) > timeout) {
            // After we've timed out — make a last-ditch effort to check for a legacy (<6j)
            // IS bootloader, otherwise fail. Only restore the prior identity if it came
            // from a successful APP-state validation. A previous mis-identification (e.g.
            // a spurious "uINS-0.0" from an earlier failed validate) would otherwise persist
            // across the next bootloader bounce and corrupt subsequent ISBL identification.
            if (oldDevInfo.hdwRunState == HDW_STATE_APP) {
                hdwId = oldHdwId;
                devInfo = oldDevInfo;
            } else if ((oldDevInfo.serialNumber != 0) && (oldDevInfo.hardwareType != 0)) {
                // Not trusted enough to restore wholesale, but keep the identity keys: the unique id is
                // (hdwId << 48) | serialNumber, and it is the only way getDevice(uid) can find this device
                // again. A timed-out validation is no evidence that either field changed. hdwRunState stays
                // UNKNOWN, so hasDeviceInfo() is false and a full revalidation is still required.
                devInfo.serialNumber = oldDevInfo.serialNumber;
                devInfo.hardwareType = oldDevInfo.hardwareType;
                memcpy(devInfo.hardwareVer, oldDevInfo.hardwareVer, sizeof(devInfo.hardwareVer));
                devInfo.hdwRunState = HDW_STATE_UNKNOWN;
                hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
            }
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

    // The loop above only exits when hasDeviceInfo() became true, and inside validate() that can only
    // have come from a parsed response -- devInfo was zeroed on entry.
    markDevInfoConfirmed();

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
 * @return ASYNC_STATE__FAILURE if there is no connection to validate over -- distinct from a timeout,
 *              since nothing was asked and retrying is pointless until the port is reopened,
 *         ASYNC_STATE__TIMEOUT if the device fails to validate,
 *         ASYNC_STATE__PENDING if the device is still validating,
 *         ASYNC_STATE__SUCCESS if the device successfully validated.
 */
int ISDevice::validateAsync(uint32_t timeout) {
    // Guarded here rather than at each caller: step(), DeviceFactory::stepValidation() and the
    // bootloader updaters all reach validation through this one function, and a device told never to
    // validate must mean it for all of them. FAILURE rather than TIMEOUT for the same reason a closed
    // port returns it -- nothing was asked, and retrying will not change that.
    if (doNotValidate)
        return ASYNC_STATE__FAILURE;

    if (!isConnected())
        return ASYNC_STATE__FAILURE;

    uint32_t now = current_timeMs();
    FnProfiler fn("ISDevice::validateAsync() [" + getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO) + "]", timeout / 2 * 1000);    // this shouldn't really ever take longer than 50ms to execute
    // A complete devInfo is not evidence that the device answered: DeviceFactory::beginValidation()
    // seeds devInfo from a discovery hint, and RelayPortFactory's hint carries hdwRunState and
    // protocolVer -- every field hasDeviceInfo() inspects. Success therefore also requires a
    // confirmation from an actual response, so that an unresponsive port cannot validate and a cached
    // run state cannot stand in for a live one.
    if (hasDeviceInfo() && (devInfoConfirmedMs != 0)) {
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
        // First step of a new validation cycle — pick the initial query type based on
        // the discovery hint, if any. NMEA/DID queries to a bootloader can leave it in
        // an unresponsive state, so when the hint says HDW_STATE_BOOTLOADER we lead
        // with the ISBL query. Without a hint we fall through to NMEA (the default
        // for app firmware) and round-robin from there.
        const dev_info_t* hint = DeviceManager::getInstance().getDeviceHint(port);
        if (hint && hint->hdwRunState == HDW_STATE_BOOTLOADER) {
            nextValidationType = QUERYTYPE_ISbootloader;
        }
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
    return utils::string_format("%s-%d.%d::SN%u", typeName, devInfo.hardwareVer[0], devInfo.hardwareVer[1], devInfo.serialNumber);
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
    if ((devInfo.hardwareType == IS_HDW_GNSS_UBLOX) && (devInfo.hardwareVer[0] != 0)) {
        // u-blox receivers do not report a meaningful hardware version, so hardwareVer[] carries the
        // MODEL IDENTITY instead: [0] = product-line letter as ('X' - 'A'), [1] = model number,
        // [2] = variant letter as ('D' - 'A') or 0. Render the designation an operator recognizes --
        // "UBX-X20D" / "UBX-F9P" / "UBX-M8" -- rather than a meaningless "UBX-23.20.3".
        // The encoding is self-describing, so new models need no table here.
        out += utils::string_format("%s-%c%u", typeName, (char)('A' + devInfo.hardwareVer[0]), devInfo.hardwareVer[1]);
        if (devInfo.hardwareVer[2] != 0)
            out += utils::string_format("%c", (char)('A' + devInfo.hardwareVer[2]));
    } else {
        out += utils::string_format("%s-%u.%u", typeName, devInfo.hardwareVer[0], devInfo.hardwareVer[1]);
        if (!(flags & COMPACT_HARDWARE_VER)) {
            if ((devInfo.hardwareVer[2] != 0) || (devInfo.hardwareVer[3] != 0)) {
                out += utils::string_format(".%u", devInfo.hardwareVer[2]);
                if (devInfo.hardwareVer[3] != 0)
                    out += utils::string_format(".%u", devInfo.hardwareVer[3]);
            }
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
    // Rendering lives in utils so it is available without an ISDevice, and so there is one
    // definition of how a firmware version is spelled. This remains the name callers use.
    return utils::getFirmwareInfoAsString(devInfo, (uint16_t)flags);
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

    flashCfg.checksum = flashChecksum32(&flashCfg, sizeof(gpx_flash_cfg_t));

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
 * @brief Initiates an asynchronous calibration upload. Ownership of the supplied
 * calibration object transfers to the device on accept; subsequent step() ticks drive
 * the per-step protocol until completion. See header for full result vocabulary.
 */
is_operation_result ISDevice::UploadImxCalibrationAsync(std::unique_ptr<ISDeviceCal> cal) {
    if (!isConnected())
        return IS_OP_CLOSED;

    if (m_calUploadResult == IS_OP_IN_PROGRESS || m_calibration)
        return IS_OP_IN_PROGRESS;

    m_calibration = std::move(cal);
    m_calUploadState = 0;
    m_calUploadResult = IS_OP_IN_PROGRESS;
    log_info(IS_LOG_ISDEVICE, "[%s] Async calibration upload initiated.", getIdAsString().c_str());
    return IS_OP_OK;
}


bool ISDevice::UploadImxCalibration(ISDeviceCal& cal)
{
    if (!isConnected())
        return false;   // nothing to do, if we aren't connected

    int result = 0;
    try {
        int calUploadState = 0;
        ISDeviceCal::cal_upload_ctx_t ctx;
        do {
            result = ISDeviceCal::uploadSensorCalStep(port, calUploadState, cal, devInfo, ctx);
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

    if (!isConnected() || isResetPending())
        return false;

    log_info(IS_LOG_ISDEVICE, "[%s] Requesting Software Reset", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str());
    for (int i = 0; i < 3; i++) {   // we shouldn't need to do this 3 times...
        if (SetSysCmd(SYS_CMD_SOFTWARE_RESET))
            break;
        SLEEP_MS(5)
    }
    disconnect();
    lastResetTime = current_timeMs();
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
            markDevInfoConfirmed();
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
        case DID_GNSS1_POS:
            static time_t lastTime;
            time_t currentTime = time(NULLPTR);
            if (abs(currentTime - lastTime) > 5) 
            {   // Update every 5 seconds
                lastTime = currentTime;
                gnss_pos_t &gps = *((gnss_pos_t*)data->ptr);
                if ((gps.status&GNSS_STATUS_FIX_MASK) >= GNSS_STATUS_FIX_3D) {
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
                markDevInfoConfirmed();
                break;

            case IS_HARDWARE_TYPE_GPX:
                if (devInfo.hardwareType == 0 ||
                    devInfo.hardwareType == IS_HARDWARE_TYPE_GPX)
                {   // Populate if device info is not set or GPX
                    devInfo = info;
                    markDevInfoConfirmed();     // only when devInfo itself was populated, not gpxDevInfo alone
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

/** @copydoc ISDevice::connect */
bool ISDevice::connect(bool revalidate, uint32_t openTimeoutMs) {

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

    if (revalidate) {
        devInfo.hdwRunState = HDW_STATE_UNKNOWN; // this will further reinforce a validation
        clearDevInfoConfirmed();                 // a caller asking to revalidate is telling us the old answer is suspect
    }

    bool alreadyOpened = portIsOpened(port);
    if (!alreadyOpened) {
        if (nextConnectMs > current_timeMs()) {
            SLEEP_MS(15);
            log_debug(IS_LOG_ISDEVICE, "Connection throttled. You can retry this device again in %dms.", nextConnectMs - current_timeMs());
            return false;   // don't attempt to reconnect until nextConnectMs has expired
        }

        // portOpen() on an asynchronous transport (TCP) reports PORT_ERROR__NONE while the connect()
        // handshake is still in flight and deliberately leaves PORT_FLAG__OPENED clear -- its contract
        // is that the caller keeps polling until the port actually opens (see tcpPortOpen() in
        // core/tcpPort.c). Re-invoking portOpen() is what advances that state; sleeping alone never
        // will. Treating the first PORT_ERROR__NONE as "connected" hands back a port that
        // isConnected() still reports false for, which silently fails every subsequent query
        // (validate(), ImxFlashConfig(), ...). Serial ports set PORT_FLAG__OPENED on the first call,
        // so they break out immediately and are unaffected.
        int portError = PORT_ERROR__NONE;
        uint32_t openDeadlineMs = current_timeMs() + openTimeoutMs;
        do {
            portError = portOpen(port);
            if ((portError != PORT_ERROR__NONE) || portIsOpened(port))
                break;
            SLEEP_MS(1);
        } while (current_timeMs() < openDeadlineMs);

        if ((portError != PORT_ERROR__NONE) || !portIsOpened(port)) {
            SLEEP_MS(15);
            nextConnectMs = current_timeMs() + 500; // if the connect fails, delay 500ms before trying again.
            log_debug(IS_LOG_ISDEVICE, "Device failed to connect (%s)... You can retry this device again in %dms.",
                      (portError != PORT_ERROR__NONE) ? "port error" : "handshake did not complete in time",
                      nextConnectMs - current_timeMs());
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
        // Only claim revalidation when we actually attempted it -- `success` is initialized true, so
        // reporting on it alone printed "(revalidated)" for every connect(false) call.
        log_debug(IS_LOG_ISDEVICE, "Connected to ISDevice::%s%s", getDescription(ESSENTIAL_FIRMWARE_INFO|COMPACT_SERIALNO).c_str(),
                  (revalidate ? (success ? " (revalidated)" : " (revalidation FAILED)") : ""));
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
        case DID_GNSS1_POS:
        case DID_GNSS2_POS:          stat.sample( ((gnss_pos_t*)data.ptr)->timeOfWeekMs * 0.001 );    break;
        case DID_GNSS1_VEL:
        case DID_GNSS2_VEL:          stat.sample( ((gnss_vel_t*)data.ptr)->timeOfWeekMs * 0.001 );    break;
        case DID_GNSS1_SAT:
        case DID_GNSS2_SAT:          stat.sample( ((gnss_sat_t*)data.ptr)->timeOfWeekMs * 0.001 );    break;
        case DID_GNSS1_SIG:
        case DID_GNSS2_SIG:          stat.sample( ((gnss_sig_t*)data.ptr)->timeOfWeekMs * 0.001 );    break;
        case DID_GNSS1_RTK_POS_REL:
        case DID_GNSS2_RTK_CMP_REL:
            if (((gnss_rtk_rel_t*)data.ptr)->timeOfWeekMs != 0)
                stat.sample( ((gnss_rtk_rel_t*)data.ptr)->timeOfWeekMs * 0.001 );
            break;
        case DID_GNSS1_RTK_POS_MISC:
        case DID_GNSS2_RTK_CMP_MISC:
            if (((gnss_rtk_rel_t*)data.ptr)->timeOfWeekMs != 0)
                stat.sample( ((gnss_rtk_misc_t*)data.ptr)->timeOfWeekMs * 0.001 );
            break;
        default:                    stat.sample();  break;
    }
    return stat.lastSampleTime();
}
