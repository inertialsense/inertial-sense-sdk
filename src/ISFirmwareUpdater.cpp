//
// Created by kylemallory on 8/18/23.
//

#include "ISDevice.h"
#include "ISFirmwareUpdater.h"

#include "ISBFirmwareUpdater.h"

#include <algorithm>
#include <map>

// Forward declarations for static helpers used throughout this file
static bool icontains(const std::string& haystack, const std::string& needle);
static update_policy_e parsePolicyString(const std::string& str);

#define LOG_FWUPDATE_STATUS(log_level, ...)   { do {                                                        \
    /* log_msg(IS_LOG_FWUPDATE, log_level, __VA_ARGS__); */                                                 \
    if (pfnStatus_cb) { pfnStatus_cb(std::make_any<ISFirmwareUpdater*>(this), log_level, __VA_ARGS__); }    \
} while (0); }                                                                                              \


ISFirmwareUpdater::ISFirmwareUpdater(device_handle_t device, ISFwUpdateState& state) : FirmwareUpdateHost(), device(device), updateState(state) {
    if (device) {
        port = device->port;
        devInfo = &device->devInfo;
        uint16_t targetHdwId = ENCODE_DEV_INFO_TO_HDW_ID(device->devInfo);

        // At some point during the upgrade, we'll likely reset the device and we need to watch for the device to come back. But the EvalTool normally doesn't discovery
        // new devices (only new ports). So, let's use the PortManagers::port_listener mechanism to detect when new ports are discover, only during the Firmware Update
        // operation.  When new ports are found, we'll attempt to discover a device on only those specific ports. We MUST keep the handle to the listener, so we can
        // release it when we're done, otherwise this could get called even after the function is out of scope, which would be BAD. Don't forget to release it at the bottom!

        // NOTE: its possible that the device may enumerate its port in the OS before the device is ready to respond to queries (though not likely). As a result, it's
        // possible that if the discoverDevice()'s timeout parameter is too low, we might miss the device - but too long, and its will block other pending ports/events.
        // We might consider a mechanism that records the new ports, and then continues to check them outside of the listener event.
        portListenerHdl = PortManager::getInstance().addPortListener(
                [targetHdwId](PortManager::port_event_e event, uint16_t portType, std::string portName, port_handle_t port, PortFactory& portFactory) {
                    if (event == PortManager::PORT_ADDED) {
                        if ( DeviceManager::getInstance().discoverDevice(port, targetHdwId, 500, DeviceManager::DISCOVERY__CLOSE_PORT_ON_FAILURE | DeviceManager::DISCOVERY__FORCE_REVALIDATION) ) {
                            log_info(IS_LOG_FWUPDATE, "Discovered device [%s] while performing Firmware Updates.", DeviceManager::getInstance().getDevice(port)->getDescription().c_str())
                        } else {
                            log_info(IS_LOG_FWUPDATE, "Discovered new port [%s] while performing Firmware Updates, but couldn't associate an ISDevice.", portName.c_str())
                            portClose(port);
                        }
                    }
                }
        );
    }
}

/**
 * Specifies the target device that you wish to update. This will attempt an initial REQ_VERSION request of that device
 * to determine if the device is available, and what firmware it is currently running.  This is a non-blocking call,
 * and will return immediately.
 * @param _target
 */
void ISFirmwareUpdater::setTarget(fwUpdate::target_t _target) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (_target == fwUpdate::TARGET_IMX5)
        _target = fwUpdate::TARGET_ISB_IMX5;    // default IMX-5 targets to ISB_IMX5 (but not DFU_IMX5)

    session_target = target = _target;
    session_image_slot = 0;
    target_devInfo = NULL;

    if (deviceUpdater) {
        // remove any previously allocated deviceUpdaters, and clear the reference : TODO: this should be a smart-pointer
        delete deviceUpdater;
        deviceUpdater = nullptr;
    }

    if (session_target != fwUpdate::TARGET_UNKNOWN) {
        if (_target == fwUpdate::TARGET_DFU_FLAG) {
            deviceUpdater = new ISDFUFirmwareUpdater(_target);
        } else if (_target & fwUpdate::TARGET_ISB_FLAG) {
            // we are about to do an IMX-5 update through the IS bootloader
            deviceUpdater = new ISBFirmwareUpdater(_target, device, toHost);
        }
    }
}

/**
 * Specifies a series of commands to perform, in sequence, in order to perform a "complex" update.
 * @param cmds a vector of strings, one command (and arguments) per entry.
 * @return true
 */
bool ISFirmwareUpdater::setCommands(std::vector<std::string> cmds) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    commands.clear();
    for (std::string& c : cmds) {
        auto pos = c.find_first_of("=");
        std::string cmdName = c.substr(0, pos);
        std::string cmdArgs = c.substr(pos+1);

        commands.emplace_back("", cmdName, cmdArgs);
    }
    updateState.state = ISFwUpdateState::UPDATER_CMDS_QUEUED;
    return true;
}

/**
 * Initiates an update of a local USB-DFU device. The target
 * @param usbDevice
 * @param target
 * @param deviceId
 * @param filename
 * @param progressRate
 * @return
 */
/*
fwUpdate::update_status_e ISFirmwareUpdater::initializeDFUUpdate(libusb_device* usbDevice, fwUpdate::target_t target, uint32_t deviceId, const std::string &filename, int flags, int progressRate)
{
    srand(time(NULL)); // get *some kind* of seed/appearance of a random number.

    size_t fileSize = 0;
    srcFile = new std::ifstream(filename, std::ios::binary);
    if (md5_file_details(srcFile, fileSize, session_md5) != 0)
        return fwUpdate::ERR_INVALID_IMAGE;
    // TODO: We need to validate that this firmware file is the correct file for this target, and that its an actual update (unless 'forceUpdate' is true)

    updateStartTime = current_timeMs();
    nextStartAttempt = current_timeMs() + attemptInterval;

    if (dfuUpdater != nullptr) {
        // we probably want to kill the previous instance and start with a new one... but maybe not?
        delete dfuUpdater;

        std::vector<DFUDevice*> devices;
        ISDFUFirmwareUpdater::getAvailableDevices(devices);
        // dfuUpdater = new ISDFUFirmwareUpdater(usbDevice, (uint32_t)target, deviceId);
    }

//    if (dfuUpdater->isConnected() == false) {
//        delete dfuUpdater;
//        return fwUpdate::ERR_UNKNOWN;
//    }

    fwUpdate::update_status_e result = (fwUpdate_requestUpdate(target, 0, 0, chunkSize, fileSize, session_md5, progressRate) ? fwUpdate::NOT_STARTED : fwUpdate::ERR_UNKNOWN);
    LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Requested firmware update with Image '%s', md5: %s", fwUpdate_getSessionTargetName(), filename.c_str(), md5_to_string(session_md5).c_str());
    return result;
}
*/

fwUpdate::update_status_e ISFirmwareUpdater::initializeUpload(fwUpdate::target_t _target, const std::string &filename, int slot, int flags, bool forceUpdate, int chunkSize, int progressRate)
{
    // std::lock_guard<std::recursive_mutex> lock(mutex);
    srand(time(NULL)); // get *some kind* of seed/appearance of a random number.

    size_t fileSize = 0;
    if (zip_archive && (filename.rfind("pkg://", 0) == 0)) {
        // check into the current archive for this file
        size_t data_len = 0;
        char *data = (char *)mz_zip_reader_extract_file_to_heap(zip_archive, filename.c_str() + 6 /* "pkg://" */, &data_len, 0);
        if (data == nullptr) {
            return fwUpdate::ERR_INVALID_IMAGE;
        }
        std::string casted_memory(static_cast<char*>(data), data_len);
        srcFile = (std::istream*)new std::istringstream(casted_memory);
    } else {
        srcFile = new std::ifstream(filename, std::ios::binary);
    }

    // TODO: We need to validate that this firmware file is the correct file for this target, and that its an actual update (unless 'forceUpdate' is true)

    // let's get the file's MD5 hash
    int hashError = (flags & fwUpdate::IMG_FLAG_useAlternateMD5)
            ? altMD5_file_details(srcFile, fileSize, session_md5)
            : md5_stream_details(*srcFile, fileSize, session_md5);
    if (hashError != 0) {
        return fwUpdate::ERR_INVALID_IMAGE;
    }

    LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Initiating update with image '%s' to target slot %d (%d bytes, md5: %s)", filename.c_str(), slot, fileSize, md5_to_string(session_md5).c_str());

    updateStartTime = current_timeMs();
    nextStartAttempt = current_timeMs() + attemptInterval;
    fwUpdate::update_status_e result = (fwUpdate_requestUpdate(_target, slot, flags, chunkSize, fileSize, session_md5, progressRate) ? fwUpdate::NOT_STARTED : fwUpdate::ERR_COMMS);

    return result;
}

/**
 * This function tries to manage if/when the user attempts to cancel/interrupt an update. Some types of updates should
 * not be allowed to be cancelled, because they may leave the device in an bad/unrecoverable state. This function tries
 * to manage those instances. One approach to this is having the manifest set a flag that prevents the update from being
 * cancelled. Regardless of the mechanism, this function should attempt to
 * @param immediately if false, signals that the update should stop as soon as it is operationally safe; if true, the
 *   update will immediately stop all operations, which could leave devices in an inoperable or unrecoverable state.
 * @return the current operating status of the update
 */
fwUpdate::update_status_e ISFirmwareUpdater::cancel(bool immediately) {
    if (fwUpdate_getSessionStatus()) {
        updateState.state = ISFwUpdateState::UPDATER_WAITING_TO_CANCEL;
    }

    // Since updates can be critical in nature, making hardware inoperable if managed incorrectly, we need to make sure
    // its safe to cancel.  All steps which have NOT already been executed are deemed safe to cancel (though, we may
    // need to revisit this in the future.

    // For now, first, we'll cancel all pending commands
    if (hasPendingCommands()) {
        for (auto& cmd : commands) {
            if (cmd.status == ISFwUpdaterCmd::CMD_QUEUED)
                cmd.status = ISFwUpdaterCmd::CMD_CANCELLED;
        }
    }

    // Second, if the current command is cancelable, then mark it as canceled, otherwise, keep its state and let it finish
    if (activeCmd && (activeCmd->flags & ISFwUpdaterCmd::CMD_FLAGS__CANCELABLE))
        activeCmd->status = ISFwUpdaterCmd::CMD_CANCELLED;

    // finally, if the user has requested an IMMEDIATE cancel, then we take the nuclear option
    // WARNING - THIS interrupts all tasks IMMEDIATELY in whatever state they were in. There is no
    // attempt to allow actions/commands to "save face" and the device maybe left in an inoperable
    // (and possibly IRRECOVERABLE) state!
    if (immediately) {
        commands.clear();
        activeCmd = &nullCmd;
        target = fwUpdate::TARGET_HOST;
    }

    return fwUpdate_getSessionStatus();
}

bool ISFirmwareUpdater::isCancelable() {
    return (activeCmd && (activeCmd->flags & ISFwUpdaterCmd::CMD_FLAGS__CANCELABLE));
}

bool ISFirmwareUpdater::fwUpdate_handleVersionResponse(const fwUpdate::payload_t& msg) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);
    memset(&remoteDevInfo, 0, sizeof(dev_info_t));
    if ((msg.data.version_resp.resTarget > fwUpdate::TARGET_HOST) && (msg.data.version_resp.resTarget <= fwUpdate::TARGET_MAXNUM) && (msg.data.version_resp.resTarget != target)) {
        return false;
    }

    remoteDevInfoTargetId = msg.data.version_resp.resTarget;
    remoteDevInfo.serialNumber = msg.data.version_resp.serialNumber;
    remoteDevInfo.hardwareType = msg.data.version_resp.hardwareType;
    remoteDevInfo.hdwRunState = msg.data.version_resp.hdwRunState;
    memcpy(remoteDevInfo.hardwareVer, msg.data.version_resp.hardwareVer, 4);
    memcpy(remoteDevInfo.firmwareVer, msg.data.version_resp.firmwareVer, 4);
    remoteDevInfo.buildType = msg.data.version_resp.buildType;
    remoteDevInfo.buildNumber = msg.data.version_resp.buildNumber;
    remoteDevInfo.buildYear = msg.data.version_resp.buildYear;
    remoteDevInfo.buildMonth = msg.data.version_resp.buildMonth;
    remoteDevInfo.buildDay = msg.data.version_resp.buildDay;
    remoteDevInfo.buildHour = msg.data.version_resp.buildHour;
    remoteDevInfo.buildMinute = msg.data.version_resp.buildMinute;
    remoteDevInfo.buildSecond = msg.data.version_resp.buildSecond;
    remoteDevInfo.buildMillisecond = msg.data.version_resp.buildMillis;

    target_devInfo = &remoteDevInfo;
    LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Received device version: %s, %s", ISDevice::getName(remoteDevInfo).c_str(), ISDevice::getFirmwareInfo(remoteDevInfo).c_str());

    return true;
}

int ISFirmwareUpdater::fwUpdate_getImageChunk(uint32_t offset, uint32_t len, void **buffer) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);
    if (srcFile && (srcFile->rdstate() == 0)) {
        srcFile->seekg((std::streampos)offset);
        len = _MIN(len, session_image_size - (uint32_t)srcFile->tellg());
        srcFile->read((char *)*buffer, len);
        return srcFile->gcount();
    }
    return -1;
}

bool ISFirmwareUpdater::fwUpdate_handleUpdateResponse(const fwUpdate::payload_t &msg) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);
    if (session_id != msg.data.update_resp.session_id)
        return false; // this message isn't for us...

    if ((session_id == msg.data.update_resp.session_id) && (session_status == msg.data.update_resp.status) && (session_status != fwUpdate::INITIALIZING))
        return true; // we're receiving duplicate messages, so ignore them

    session_status = msg.data.update_resp.status;
    session_total_chunks = msg.data.update_resp.totl_chunks;

    if (session_status == fwUpdate::READY) {
        LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_MORE_DEBUG, "Received remote response: %s; Expecting %d chunks.", fwUpdate_getStatusName(session_status), session_total_chunks);
    } else {
        LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_MORE_DEBUG, "Received remote response: %s", fwUpdate_getStatusName(session_status));
    }

    switch (session_status) {
        case fwUpdate::ERR_MAX_CHUNK_SIZE:    // indicates that the maximum chunk size requested in the original upload request is too large.  The host is expected to begin a new session with a smaller chunk size.
            return fwUpdate_requestUpdate(session_target, session_image_slot, session_image_flags, session_chunk_size / 2, session_image_size, session_md5);
        case fwUpdate::ERR_INVALID_SESSION:   // indicates that the requested session ID is invalid.
        case fwUpdate::ERR_INVALID_SLOT:      // indicates that the request slot does not exist. Different targets have different number of slots which can be written to.
        case fwUpdate::ERR_NOT_ALLOWED:       // indicates that writing to the requested slot is not allowed, usually due to security constrains such as a locked firmware, Read-Only FLASH, etc.
        case fwUpdate::ERR_NOT_ENOUGH_MEMORY: // indicates that the requested firmware file size would exceed the available slot size.
        case fwUpdate::ERR_OLDER_FIRMWARE:    // indicates that the new firmware is an older (or earlier) version, and performing this would result in a downgrade.
        case fwUpdate::ERR_TIMEOUT:           // indicates that the update process timed-out waiting for data (either a request, response, or chunk data that never arrived)
        case fwUpdate::ERR_CHECKSUM_MISMATCH: // indicates that the final checksum didn't match the checksum specified at the start of the process
        case fwUpdate::ERR_COMMS:             // indicates that an error in the underlying comms system
        case fwUpdate::ERR_NOT_SUPPORTED:    // indicates that the target device doesn't support this protocol
        case fwUpdate::ERR_FLASH_WRITE_FAILURE: // indicates that writing of the chunk to flash/nvme storage failed (this can be retried)
        case fwUpdate::ERR_FLASH_OPEN_FAILURE:  // indicates that an attempt to "open" a particular flash location failed for unknown reasons.
        case fwUpdate::ERR_FLASH_INVALID:       // indicates that the image, after writing to flash failed to validate (invalid signature, couldn't decrypt, etc).
            return false;

        case fwUpdate::READY:
            next_chunk_id = 0;
            // fall through
        default:
            return true;
    }
}

bool ISFirmwareUpdater::fwUpdate_handleResendChunk(const fwUpdate::payload_t &msg) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    // TODO: LOG msg.data.req_resend.reason
    uint32_t current_ms = current_timeMs();
    if (msg.data.req_resend.chunk_id == last_resent_chunk) {
        resent_chunkid_count++;
        // If we have more than 10 consecutive write errors in more than 5 seconds, fail out
        if ((resent_chunkid_count > 10) && (current_ms - resent_chunkid_time > 5000)) {
            switch (msg.data.req_resend.reason) {
                case fwUpdate::REASON_WRITE_ERROR:
                    session_status = fwUpdate::ERR_FLASH_WRITE_FAILURE;
                    break;
                case fwUpdate::REASON_INVALID_SEQID:
                case fwUpdate::REASON_INVALID_SIZE:
                    session_status = fwUpdate::ERR_INVALID_CHUNK;
                    break;
                case fwUpdate::REASON_NONE:
                    break;
            }
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_ERROR, "To many resends of the same chunk; giving up with error %s", fwUpdate_getNiceStatusName(session_status));
            return false;
        }
    } else {
        resent_chunkid_count = 0;
        resent_chunkid_time = current_ms;
    }

    LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_DEBUG, "Remote requested resend of %d: %d", msg.data.req_resend.chunk_id, msg.data.req_resend.reason);
    nextChunkSend = current_timeMs() + resendChunkDelay;
    return fwUpdate_sendNextChunk(); // we don't have to send this right away, but sure, why not!
}

bool ISFirmwareUpdater::fwUpdate_handleUpdateProgress(const fwUpdate::payload_t &msg) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (session_status >= fwUpdate::NOT_STARTED)
        session_status = msg.data.progress.status; // don't overwrite an error status in the event of racing messages.

    progress_total = msg.data.progress.totl_chunks;
    progress_num = msg.data.progress.num_chunks;
    // percentComplete = msg.data.progress.num_chunks/(float)(msg.data.progress.totl_chunks)*100.f;
    const char* message = (msg.data.progress.msg_len > 0) ? (const char*)&msg.data.progress.message : "";
    LOG_FWUPDATE_STATUS(static_cast<eLogLevel>(msg.data.progress.msg_level), message);
    return true;
}

bool ISFirmwareUpdater::fwUpdate_handleDone(const fwUpdate::payload_t &msg) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    session_status = msg.data.resp_done.status;
    session_id = 0;
    SLEEP_MS(200);  // FIXME: there is a very weird instance in which completing an update and *immediately* starting the next, can attempt to reuse the session id - This tries to fix that.
    return true;
}

bool ISFirmwareUpdater::fwUpdate_isDone() {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    bool cmdsPending = hasPendingCommands();
    bool in_progress = ((session_id == 0) && (fwUpdate_getSessionStatus() > fwUpdate::NOT_STARTED) && (fwUpdate_getSessionStatus() < fwUpdate::FINISHED));
    bool is_done = !(cmdsPending || in_progress);
    return is_done;
}

void ISFirmwareUpdater::fwUpdate_handleLocalDevice() {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    // pull all data from the buffer there really should only be one message at a time... :fingers-crossed:
    const int toHost_size = toHost.size();

    uint8_t* toHostBuf = new uint8_t[toHost_size];
    uint8_t* p = toHostBuf;
    while (toHost.size()) {
        *p++ = toHost.front();
        toHost.pop_front();
    }

    for (p = toHostBuf; p < toHostBuf + toHost_size; ) {
        fwUpdate::payload_t *msg;
        void *aux_data = nullptr;
        int msg_len = fwUpdate_mapBufferToPayload(p, &msg, &aux_data);
        p += msg_len;   // we need to continue to consume the message, even if we can't process it below, otherwise, we'll be stuck here forever.

        fwUpdate_processMessage(*msg);
    }
    delete [] toHostBuf;
}

void ISFirmwareUpdater::refreshUpdateState() {
    auto lk = updateState.lock();
    auto activeCmd = getActiveCommand();
    if (&activeCmd != &nullCmd)
        updateState.lastMessage = activeCmd.resultMsg;

    // let's always recheck our error state for any commands (past, present, or future) which have failed
    updateState.hasErrors = false;
    for (auto& c :commands) {
        if (c.status == ISFwUpdaterCmd::CMD_ERROR)
            updateState.hasErrors = true;
    }

    if (activeCmd.cmd == "upload") {
        updateState.status = fwUpdate_getSessionStatus();
        updateState.slot = fwUpdate_getSessionImageSlot();
        // TODO?? updateState.state should keep the state of the entire update, not just the last upload... right??

        if (fwUpdate_getSessionTarget() != updateState.target) {
            updateState.target = fwUpdate_getSessionTarget();
            updateState.status = fwUpdate::NOT_STARTED;
            updateState.messages.clear();
        }

        if ((getUploadStatus() == fwUpdate::NOT_STARTED) && (activeCmd.status == ISFwUpdaterCmd::CMD_QUEUED)) {
            updateState.status = fwUpdate::INITIALIZING;    // We're just starting (no error yet, but no response either)
        }
        updateState.lastMessage = fwUpdate_getNiceStatusName(updateState.status);
    } else if (activeCmd.cmd == "waitfor") {
        updateState.lastMessage = "Waiting for response from device.";
    } else if (activeCmd.cmd == "reset") {
        updateState.lastMessage = "Resetting device.";
    } else if (activeCmd.cmd == "delay") {
        updateState.lastMessage = "Waiting...";
    }
    switch (updateState.state) {
        case ISFwUpdateState::UPDATER_CANCELED:
            updateState.lastMessage = "Cancelled by User.";
            break;
        case ISFwUpdateState::UPDATER_DONE_WITH_ERRORS:
            updateState.lastMessage = "Error: One or more step errors occurred.";
            break;
        case ISFwUpdateState::UPDATER_WAITING_TO_CANCEL:
            updateState.lastMessage = "Cancelling (waiting for non-cancelable step to complete).";
            break;
        case ISFwUpdateState::UPDATER_IDLE:
            break;
        case ISFwUpdateState::UPDATER_CMDS_QUEUED:
            updateState.lastMessage = "Starting update...";
            break;
        case ISFwUpdateState::UDPATER_SUSPENDED:
            updateState.lastMessage = "Update suspended.";
            break;
        case ISFwUpdateState::UPDATER_IN_PROGRESS:
            updateState.lastMessage = "Updating...";
            if (updateState.hasErrors) {
                updateState.lastMessage = "Error: ";
                updateState.lastMessage += getUploadStatusName();
            }
            break;
        case ISFwUpdateState::UPDATER_SUCCESSFUL:
        case ISFwUpdateState::SUCCESS_WITH_NOTIFICATIONS:
            updateState.lastMessage = "Completed successfully.";
            break;
    }

    updateState.progress = (!fwUpdate_isDone() ? getProgress() : 0.0f);
}

bool ISFirmwareUpdater::step() {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    FnProfiler fnStep("ISFirmwareUpdater::step() [" + ( device ? device->getIdAsString() : "") + "]", 30000);   // this can typically take around 25ms in Windows

    if (device && (device->port != port))
        port = device->port;

    if (!port && (nextPortCheck < current_timeMs())) {
        nextPortCheck = current_timeMs() + 1000;    // check every second.
        PortManager& portManager = PortManager::getInstance();
        DeviceManager& deviceManager = DeviceManager::getInstance();
        std::stringstream ss;

        // we are specifically only interested on doing a device discovery on ports which do not have an associated device
        portManager.discoverPorts();
        for ( auto p : portManager.locked_range()) {
            if (deviceManager.getDevice(p) == nullptr) {
                if (deviceManager.discoverDevice(p))
                    if (device->port)
                        break;
            }
        }
        fnStep.mark("Finished Port check.");
    }

    if (deviceUpdater) {
        deviceUpdater->fwUpdate_step();
        // we need to handle local data exchange through our byte stream to the device
        if (!toHost.empty())
            fwUpdate_handleLocalDevice();
        fnStep.mark("Finished deviceUpdater->fwUpdate_step().");
    }

    if (lastStatus != session_status) {
        LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_MORE_DEBUG, "Session status changed: %s", fwUpdate_getStatusName(session_status));
        lastStatus = session_status;
        fnStep.mark("Finished progress callbacks.");
    }

    if (!commands.empty()) {
        if ((activeCmd == &nullCmd) || ((activeCmd->status != ISFwUpdaterCmd::CMD_IN_PROCESS) && (activeCmd->status != ISFwUpdaterCmd::CMD_QUEUED)))
            activeCmd = &getNextQueuedCmd(activeCmd);
        activeCmd = &runCommand(*activeCmd);
    } else {
        activeCmd = &nullCmd;
    }

    bool result = fwUpdate_step();
    fnStep.mark("Finished fwUpdate_step().");

    if (fwUpdate_isDone()) {
        updateState.state = hasErrors() ? ISFwUpdateState::UPDATER_DONE_WITH_ERRORS : ISFwUpdateState::UPDATER_SUCCESSFUL;

        // be sure to release/cleanup the source file after we are finished with it.
        if (srcFile) {
            delete srcFile;
            srcFile = nullptr;
        }
    }

    refreshUpdateState();
    return result;
}

bool ISFirmwareUpdater::fwUpdate_step(fwUpdate::msg_types_e msg_type, bool processed) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);
    FnProfiler fnStep("ISFirmwareUpdater::fwUpdate_step() [" + ( device ? device->getIdAsString() : "") + "]", 5000);

    switch(session_status) {
        case fwUpdate::READY:
        case fwUpdate::IN_PROGRESS:
            if (nextChunkSend < current_timeMs()) // don't send chunks too fast
                fwUpdate_sendNextChunk();
            break;
        case fwUpdate::FINISHED:
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Firmware uploaded in %0.1f seconds", (current_timeMs() - updateStartTime) / 1000.f);
            if (hasPendingCommands()) {
                //session_id = 0;
                session_status = fwUpdate::NOT_STARTED;
                nextStartAttempt = current_timeMs() + attemptInterval;
            }
            break;
        case fwUpdate::ERR_MAX_CHUNK_SIZE:
            if (session_id != 0) {
                // we need to get a new session, and a new chunk size, but let's keep everything else the same
                // let's try and stick to factors of 2 (ie, 64, 128, 256, 512, 1024, etc) even if we started with some number in-between (384)
                double lv = log2(session_chunk_size);
                int bits = (lv == floor(lv)) ? (int)(lv-1) : (int)(lv); // round down to the nearest multiple of 2
                session_chunk_size = 1 << bits;
                session_id = (uint16_t) rand(); // since we ended on an error, we need a new session id.
                fwUpdate_requestUpdate();
            }
            break;
        case fwUpdate::ERR_TIMEOUT:
            if (session_status < fwUpdate::NOT_STARTED) {
                handleCommandError(*activeCmd, -1, "No Response from device.");
            }
            break;
        case fwUpdate::NOT_STARTED:
        case fwUpdate::INITIALIZING:
        case fwUpdate::FINALIZING:
        default:
            if (session_status < fwUpdate::NOT_STARTED) {
                handleCommandError(*activeCmd, -session_status, "Unexpected response from device : %s", fwUpdate_getSessionStatusName());
            }
            break;
    }

    uint32_t lastMsg = fwUpdate_getLastMessageAge();
    if ((lastMsg > timeout_duration) && ((session_status > fwUpdate::NOT_STARTED) && (session_status < fwUpdate::FINISHED)))
        session_status = fwUpdate::ERR_TIMEOUT;

    return (session_status != fwUpdate::NOT_STARTED);
}

bool ISFirmwareUpdater::fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t *buffer, int buff_len) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (deviceUpdater != nullptr) {
        bool result = deviceUpdater->fwUpdate_processMessage(buffer, buff_len);
        if (!toHost.empty()) // check for any responses
            fwUpdate_handleLocalDevice();
        return result;
    }

    // TODO: This can be removed after 2.1.0 release
    if ((remoteDevInfo.firmwareVer[0] == 2) && (remoteDevInfo.firmwareVer[1] == 0) && (remoteDevInfo.firmwareVer[2] == 0) && (remoteDevInfo.firmwareVer[3] <= 10)) {
        if (((fwUpdate::payload_t*)buffer)->hdr.msg_type == fwUpdate::MSG_REQ_UPDATE) {
            // these versions didn't support 'flags' to zero it out.
            ((fwUpdate::payload_t*)buffer)->data.req_update.image_flags = 0;
        }
    }
    // TODO: end

    nextChunkSend = current_timeMs() + chunkDelay; // give *at_least* enough time for the send buffer to actually transmit before we send the next message

    port_handle_t preferredPort = portIsOpened(device->port) ? device->port : (portIsOpened(port) ? port : nullptr);
    int result = comManagerSendData(preferredPort, buffer, DID_FIRMWARE_UPDATE, buff_len, 0);
    return (result == 0);
}

/**
 * Called when an error occurs while processing a command, to perform corrective actions (if possible).
 * Primarily, this checks if there is a failLabel defined and looks for the corresponding command label.
 * Otherwise it logs the message/errorcode, and clears the command stack.
 * @param cmd the current command being executed when this error occurred
 * @param errCode a numerical error code
 * @param errMsg a corresponding human-readable error message to the numerical error code
 */
void ISFirmwareUpdater::handleCommandError(ISFwUpdaterCmd& cmd, int errCode, const char *errMsg, ...) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (!commands.empty()) {
        fwUpdate_resetEngine();
    }

    char buffer[256];
    va_list args;
    va_start(args, errMsg);
    VSNPRINTF(buffer, sizeof(buffer), errMsg, args);
    va_end(args);

    cmd.status = ISFwUpdaterCmd::CMD_ERROR;
    {
        auto lk = updateState.lock();
        updateState.messages.emplace_back(activeStep, cmd, IS_LOG_LEVEL_ERROR, buffer);
    }

    LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_ERROR, buffer);

    if (failLabel.empty()) {
        // if no label has been specified, clear all commands and reset
        // commands.clear();
        activeCmd = &nullCmd;
        target = fwUpdate::TARGET_HOST;
        return;
    }

    activeCmd = &jumpToStep(failLabel.substr(1));
}

/**
 * This is the primary command processor, which allows each command a chance to run.
 * @param cmd the command to run
 * @return the command to run on the next call (this maybe different from the passed cmd)
 */
ISFwUpdaterCmd& ISFirmwareUpdater::runCommand(ISFwUpdaterCmd& cmd) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (&nullCmd == &cmd)
        return nullCmd;

    if (cmd.status == ISFwUpdaterCmd::CMD_QUEUED) {
        cmd.timeStarted = std::chrono::system_clock::now();
        if (!cmd.cmd.empty()) {
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_MORE_DEBUG, "Executing manifest command '%s\\%s'", cmd.step.c_str(), cmd.cmd.c_str());
        }
        if (updateState.state == ISFwUpdateState::UDPATER_SUSPENDED)
            return cmd; // this should prevent the next QUEUED command from executing (until updateState is changed)
    } else
        updateState.state = ISFwUpdateState::UPDATER_IN_PROGRESS;

    if (cmd.step != activeStep) {
        // new step section/target - we should reset certain states here if needed
        activeStep = std::string(cmd.step);
        session_target = target = fwUpdate::TARGET_HOST;
        session_image_slot = slotNum = 0;
        failLabel.clear();

        // Late pattern resolution: if this step has no resolved policy yet, try matching
        // policyPatterns against the target name within this step's commands.
        if (!activeStep.empty() &&
            (stepPolicies.find(activeStep) == stepPolicies.end() ||
             stepPolicies[activeStep].policy == UPDATE_POLICY_DEFAULT)) {
            for (auto& c : commands) {
                if (c.step == activeStep && c.cmd == "target" && c.hasArg("target")) {
                    std::string targetName = c.getArg("target");
                    for (auto& [pattern, policy] : policyPatterns) {
                        if (icontains(targetName, pattern)) {
                            stepPolicies[activeStep].policy = policy;
                            break;
                        }
                    }
                    break;
                }
            }
        }

        // Check if this step should be skipped entirely.
        // Policies only apply to steps that have a "target" command — because policies are
        // inherently about firmware targets. Steps without a target are control-flow (FINALIZE,
        // ERROR, etc.) and should always run. Without a target, there's no version to compare
        // for IF_NEWER, and no device to skip or force.
        bool stepHasTarget = false;
        for (auto& c : commands) {
            if (c.step == activeStep && c.cmd == "target") { stepHasTarget = true; break; }
        }

        if (!activeStep.empty() && stepHasTarget) {
            update_policy_e policy = getEffectivePolicy(activeStep);
            if (policy == UPDATE_POLICY_SKIP) {
                LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Skipping step '%s' (policy: SKIP)", activeStep.c_str());
                for (auto& c : commands) {
                    if (c.step == activeStep && c.status == ISFwUpdaterCmd::CMD_QUEUED)
                        c.status = ISFwUpdaterCmd::CMD_NOT_EXECUTED;
                }
                cmd.status = ISFwUpdaterCmd::CMD_NOT_EXECUTED;
                activeCmd = &getNextQueuedCmd(&cmd);
                return *activeCmd;
            }
        }

        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    }

    if (cmd.cmd == "package") {
        cmd_ExtractPackage(cmd);
        // cmd reference is now dangling (commands vector was repopulated) — return nullCmd
        return nullCmd;
    }
    else if (cmd.cmd == "target") cmd_SetTarget(cmd);
    else if (cmd.cmd == "waitfor") cmd_WaitFor(cmd);
    else if (cmd.cmd == "delay") cmd_Delay(cmd);
    else if (cmd.cmd == "upload") cmd_UploadImage(cmd);
    else if (cmd.cmd == "reset") cmd_resetDevice(cmd);
    else if (cmd.cmd == "finish") cmd_finish(cmd);
    else if ((cmd.cmd == "on-error")) {
        failLabel = cmd[0].c_str();
        // all labels must start with a colon (:)
        if (failLabel[0] != ':') {
            handleCommandError(cmd, -1, "Invalid label [%s]. Labels must start with a colon (:).", failLabel.c_str());
            cmd.status = ISFwUpdaterCmd::CMD_ERROR;
            failLabel.clear();
        } else {
            cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
        }
    } else if ((cmd.cmd == "echo") && (cmd.args.size() == 1)) {
        // simply outputs the argument string to the progress handler
        // this is primarily for manifest authors to output information to the user
        std::string msg;
        for (auto [k,v] : cmd.args) msg += v;
        LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, msg.c_str());
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    } else if ((cmd.cmd == "slot") && (cmd.args.size() == 1)) {
        slotNum = strtol(cmd[0].c_str(), nullptr, 10);
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    } else if ((cmd.cmd == "timeout") && (cmd.args.size() == 1)) {
        fwUpdate_setTimeoutDuration(strtol(cmd[0].c_str(), nullptr, 10));
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    } else if (cmd.cmd == "force") {
        forceUpdate = (cmd[0] == "true" ? true : false);
        defaultPolicy = forceUpdate ? UPDATE_POLICY_FORCE : UPDATE_POLICY_DEFAULT;
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    } else if (cmd.cmd == "policy") {
        std::string policyArg = cmd.getArg("policy", cmd[0]);
        std::string patternArg = cmd.getArg("target", "");
        update_policy_e p = parsePolicyString(policyArg);
        if (p == UPDATE_POLICY_DEFAULT) {
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_ERROR, "Unknown update policy: '%s' (valid: skip, if-newer, force, always)", policyArg.c_str());
        } else if (!patternArg.empty()) {
            // Explicit target pattern provided: apply as a pattern match
            setPolicyPattern(patternArg, p);
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_DEBUG, "Set update policy '%s' for targets matching '%s'", policyArg.c_str(), patternArg.c_str());
        } else if (!activeStep.empty()) {
            // No target specified, but we're inside a step scope: apply to the current step
            setStepPolicy(activeStep, p);
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_DEBUG, "Set update policy '%s' for step '%s'", policyArg.c_str(), activeStep.c_str());
        } else {
            // No target, no active step: set the global default
            setDefaultPolicy(p);
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_DEBUG, "Set default update policy: %s", policyArg.c_str());
        }
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    } else if ((cmd.cmd == "chunk") && (cmd.args.size() == 1)) {
        chunkSize = strtol(cmd[0].c_str(), nullptr, 10);
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    } else if ((cmd.cmd == "rate") && (cmd.args.size() == 1)) {
        progressRate = strtol(cmd[0].c_str(), nullptr, 10);
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    } else if (cmd.cmd.empty()) {
        // no command, nothing to do.
    } else {
        // unknown command - ignore it
        LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_ERROR, "Unknown command: '%s'", cmd.cmd.c_str());
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
    }

    if ((cmd.status != ISFwUpdaterCmd::CMD_QUEUED) && (cmd.status != ISFwUpdaterCmd::CMD_IN_PROCESS)) {
        cmd.timeFinished = std::chrono::system_clock::now();
    }

    return *activeCmd;
}

/**
 * @brief locates a provided manifest (either directly, or within a ZIP package), extracts the manifest and expands the
 * manifest into a new set of commands, loading them into the command queue. Optionally, this can suspend the updater
 * after extracting those commands..
 * @param args a set of positional arguments
 * This command has the following arguments:
 *     path [requires] :: the path to the package - if the path ends with ".yaml" it is treated as a path directly to
 *       the manifest file, otherwise its assumed the path is a .zip file which contains a manifest.yaml.
 *     suspend [optional] :: if true (default is false) the updater will be suspeneded (requiring explicit resumption)
 *       immediately after extracting the manifest and populating the command queue - this allows additional "post-processing"
 *       on the resulting command queue before performing any additional commands.
 */
void ISFirmwareUpdater::cmd_ExtractPackage(ISFwUpdaterCmd cmd) {
    // NOTE: Unlike other actions, use a copy of the ISFwUpdaterCmd, because processPackageManifest() by design
    // modifies the list of commands to process, which could possibly invalidate the underlying reference to this
    // command.

    // std::lock_guard<std::recursive_mutex> lock(mutex);
    std::string arg0 = cmd[0];  // REMEMBER cmd[0] is getting the first argument (std::string) from cmd
    bool suspendAfterExtraction = ((cmd.args.size() == 2) && (cmd[1] == "true"));
    bool isManifest = (arg0.length() >= 5) && (0 == arg0.compare (arg0.length() - 5, 5, ".yaml"));
    pkg_error_e err_result = isManifest ? processPackageManifest(arg0) : openFirmwarePackage(arg0);

    // processPackageManifest/openFirmwarePackage modifies the commands vector, which
    // invalidates any pointers into it (including the caller's reference and activeCmd).
    // Reset activeCmd so step() doesn't dereference a dangling pointer.
    activeCmd = &nullCmd;

    if (err_result != PKG_SUCCESS) {
        const char *err_msg = nullptr;
        switch (err_result) {
            case PKG_ERR_PACKAGE_FILE_ERROR:
                err_msg = "Unable to open package.";
                break;
            case PKG_ERR_INVALID_IMAGES:
                err_msg = "Manifest has no images defined, or is malformed.";
                break;
            case PKG_ERR_INVALID_STEPS:
                err_msg = "Manifest has no steps defined, or is malformed.";
                break;
            case PKG_ERR_INVALID_TARGET:
                err_msg = "Manifest references an invalid target, or no target defined for image.";
                break;
            case PKG_ERR_UNSUPPORTED_TARGET:
                err_msg = "Image references an invalid or unsupported target device.";
                break;
            case PKG_ERR_NO_ACTIONS:
                err_msg = "Manifest has no valid actions.";
                break;
            case PKG_ERR_IMAGE_INVALID_REFERENCE:
                err_msg = "Manifest references an invalid or non-existent image entry.";
                break;
            case PKG_ERR_IMAGE_UNKNOWN_PATH:
                err_msg = "Manifest image path is missing or invalid.";
                break;
            case PKG_ERR_IMAGE_FILE_NOT_FOUND:
                err_msg = "Manifest image reference is valid, but the backing datafile is missing.";
                break;
            case PKG_ERR_IMAGE_FILE_SIZE_MISMATCH:
                err_msg = "Manifest's reported image size does not match the actual data file size.";
                break;
            case PKG_ERR_IMAGE_FILE_MD5_MISMATCH:
                err_msg = "Manifest's reported image MD5 digest does not match the actual data file MD5 digest.";
                break;
            case PKG_ERR_NO_MANIFEST:
                err_msg = "Manifest is missing or corrupt.";
                break;
            case PKG_SUCCESS:
                break;
        }
        handleCommandError(cmd, err_result, "Error processing firmware package [%s] (Error code: %d) :: %s", arg0.c_str(), err_result, err_msg);
    } else if (suspendAfterExtraction) {
        // we aren't ready to process any new commands from the package...
        updateState.state = ISFwUpdateState::UDPATER_SUSPENDED;
    }

}

/**
 * @brief processes the manifest command "target" responsible for setting the target device to apply all subsequent command towards.
 * @param args a set of positional arguments
 * This command has the following arguments:
 *     target ID [required] :: One of IMX5, IMX6, GPX1, GNSS1, GNSS2
 *     timeout [optional] :: the number of milliseconds to wait for a response from the requested target (defaults to 0ms, or no wait)
 *     interval [optional] :: the number of milliseconds between re-request attempts, while waiting for a response (0ms (default) mean do not send any additional re-requests)
 *     on_timeout [optional] :: a label to jump to in the event that the timeout occurs waiting for the device
 */
void ISFirmwareUpdater::cmd_SetTarget(ISFwUpdaterCmd& cmd) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    std::string targetName = cmd["target"];
    if (cmd.status == ISFwUpdaterCmd::CMD_QUEUED) {
        if (targetName == "IMX5") setTarget(fwUpdate::TARGET_IMX5);
        else if (targetName == "IMX6") setTarget(fwUpdate::TARGET_IMX6);
        else if (targetName == "GPX1") setTarget(fwUpdate::TARGET_GPX1);
        else if (targetName == "GNSS1") setTarget(fwUpdate::TARGET_SONY_CXD5610__1);
        else if (targetName == "GNSS2") setTarget(fwUpdate::TARGET_SONY_CXD5610__2);
        else {
            handleCommandError(cmd, -1, "Invalid Target specified: %s  (Valid targets are: IMX5, GPX1, GNSS1, GNSS2)", targetName.c_str());
            cmd.status = ISFwUpdaterCmd::CMD_ERROR;
            return;
        }
        cmd.resultMsg = "Targeting device " + targetName + " for update.";
    }
    if (cmd.args.size() == 1) {
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS; // if we're only setting the target, there is nothing left to do.
    } else {
        cmd_WaitFor(cmd);       // this will set the cmd.status for us.
    }
}


/**
 * @brief processes the manifest command "waitfor" which suspends execution of all other commands until the current target responds
 * to a VersionInfo request or times-out waiting.
 * @param args a set of positional arguments
 * This command accepts the following arguments:
 *     timeout [required] :: the number of milliseconds to wait for a response from the requested target (defaults to 0ms, or no wait)
 *     interval [optional] :: the number of milliseconds between re-request attempts, while waiting for a response (0ms (default) mean do not send any additional re-requests)
 *     on_timeout [optional] :: a label to jump to in the event that the timeout occurs waiting for the device
 *     force [optional] :: clear any existing devInfo and force a new request of VersionInfo
 */
void ISFirmwareUpdater::cmd_WaitFor(ISFwUpdaterCmd& cmd) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (cmd.status == ISFwUpdaterCmd::CMD_QUEUED) {
        // force a new query, even if we previously had devInfo for this target, and use default values
        pingInterval = strtol(cmd.getArg("interval", "1000").c_str(), nullptr, 10);
        pingTimeoutMs = strtol(cmd.getArg("timeout", "5000").c_str(), nullptr, 10);
        pingTimeoutExpires = current_timeMs() + pingTimeoutMs;
        timeoutLabel = cmd.getArg("on-timeout", failLabel);
        if (timeoutLabel[0] != ':') {
            handleCommandError(cmd, -1, "Invalid label [%s]. Labels must start with a colon (:).", timeoutLabel.c_str());
            cmd.status = ISFwUpdaterCmd::CMD_ERROR;
        }

        if (cmd["force"] == "true")
            target_devInfo = NULL;

        cmd.resultMsg = "Requesting status of target device...";
    }

    cmd.status = ISFwUpdaterCmd::CMD_IN_PROCESS;
    if (target_devInfo && ((remoteDevInfoTargetId & fwUpdate::TARGET_TYPE_MASK) == (target & fwUpdate::TARGET_TYPE_MASK))) {
        // SUCCESS
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
        cmd.resultMsg = "Received response from target.";
        pingInterval = 1000;       //!< delay between attempts to communicate with a target device
        pingNextRetry = 0;         //!< time for next ping
        pingTimeoutMs = 0;         //!< time when the ping operation will timeout if no response before then
        timeoutLabel.clear();      //!< a label to jump to, when a "waitfor" times out (which is not always an error)
    } else if (pingTimeoutExpires && (current_timeMs() > pingTimeoutExpires)) {
        // TIMEOUT occurred
        cmd.status = ISFwUpdaterCmd::CMD_ERROR;
        cmd.resultMsg = "Timeout limit reached waiting for response from the target device.";
        pingTimeoutExpires= pingNextRetry = 0;
        if (!timeoutLabel.empty()) {
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, cmd.resultMsg.c_str());
            activeCmd = &jumpToStep(timeoutLabel.substr(1));
        } else {
            handleCommandError(cmd, -1, cmd.resultMsg.c_str());
        }
    } else if (pingInterval && (pingNextRetry < current_timeMs())) {
        // Still waiting and ready to send another ping
        pingNextRetry = current_timeMs() + pingInterval;
        target_devInfo = nullptr;

        LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_MORE_INFO, "Requesting version info from '%s' (upto %0.2f seconds)...", fwUpdate_getTargetName(target), (pingTimeoutExpires - current_timeMs()) / 1000.0);
        fwUpdate_requestVersionInfo(target);
        cmd.resultMsg = "Requesting status of target device...";
    } else {
        cmd.resultMsg = "Waiting for response from target device...";
    }
}

void ISFirmwareUpdater::cmd_Delay(ISFwUpdaterCmd& cmd) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (cmd.status == ISFwUpdaterCmd::CMD_QUEUED) {
        if (cmd.args.size() != 1) {
            cmd.status = ISFwUpdaterCmd::CMD_ERROR;
            cmd.resultMsg = "Missing required 'delay' argument";
            return;
        }

        cmd.resultMsg = utils::string_format("Pausing for %0.2f seconds...", strtol(cmd[0].c_str(), nullptr, 10) / 1000.0);
        LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_MORE_INFO, cmd.resultMsg.c_str());
        pauseUntil = current_timeMs() + strtol(cmd[0].c_str(), nullptr, 10);
    }

    cmd.status = ISFwUpdaterCmd::CMD_IN_PROCESS;
    if (!pauseUntil || pauseUntil < current_timeMs()) {
        cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
        cmd.resultMsg = utils::string_format("Paused for %0.2f seconds.", strtol(cmd[0].c_str(), nullptr, 10) / 1000.0);
        pauseUntil = 0;
    }
}

/**
 * @brief processes the manifest command "upload" which handles sending a firmware image to a device.
 * @param cmd the state for this command, which includes arguments, status, and result codes, etc.
 * This command accepts the following arguments:
 *     filename [required] :: the path/filename of the image to upload to the device
 *     interval [optional] :: the period interval (in milliseconds) which the remote device should send progress/status reports (default = "250")
 *     chunkSize [optional] :: the size of each chunk to send (default = "512")
 *     force [optional] :: if "true" will cause the upload to occur, bypassing version checking (default = "false");
 */
void ISFirmwareUpdater::cmd_UploadImage(ISFwUpdaterCmd& cmd) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (cmd.status == ISFwUpdaterCmd::CMD_QUEUED) {
        filename = cmd["filename"];
        if (cmd.hasArg("slot")) slotNum = std::strtol(cmd.getArg("slot", "0").c_str(), nullptr, 10);
        if (cmd.hasArg("interval")) progressRate = std::strtol(cmd.getArg("interval", "250").c_str(), nullptr, 10);
        if (cmd.hasArg("chunkSize")) progressRate = std::strtol(cmd.getArg("chunkSize", "512").c_str(), nullptr, 10);
        if (cmd.hasArg("force")) forceUpdate = (cmd.getArg("force", "false") == "true");

        // Resolve effective update policy for this upload
        update_policy_e effectivePolicy;
        if (cmd.hasArg("force") && cmd.getArg("force") == "true") {
            effectivePolicy = UPDATE_POLICY_FORCE;
        } else if (cmd.hasArg("update-policy")) {
            effectivePolicy = parsePolicyString(cmd.getArg("update-policy"));
            if (effectivePolicy == UPDATE_POLICY_DEFAULT)
                effectivePolicy = getEffectivePolicy(cmd.step);
        } else {
            effectivePolicy = getEffectivePolicy(cmd.step);
        }

        // SKIP: skip this upload entirely
        if (effectivePolicy == UPDATE_POLICY_SKIP) {
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Skipping upload of '%s' (policy: SKIP)", filename.c_str());
            cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
            cmd.resultMsg = "Upload skipped (policy: SKIP).";
            return;
        }

        // FORCE: ensure forceUpdate is set for protocol-level behavior
        if (effectivePolicy == UPDATE_POLICY_FORCE)
            forceUpdate = true;

        fwUpdate_resetEngine();

        uint8_t flags = 0;
        // check for non encrypted file CXD update slot 4 or slot 2 if .fpk
        if (((target & fwUpdate::TARGET_SONY_CXD5610) == fwUpdate::TARGET_SONY_CXD5610) && (slotNum == 4 || (slotNum == 2 && filename.substr(filename.find_last_of(".") + 1) == "fpk")))
            flags |= fwUpdate::IMG_FLAG_imageNotEncrypted;

        if (((target & fwUpdate::TARGET_IMX5) == fwUpdate::TARGET_IMX5) && (target & fwUpdate::TARGET_ISB_FLAG) && (devInfo->hardwareType == IS_HARDWARE_TYPE_IMX))
            target = fwUpdate::TARGET_ISB_IMX5;

        // Resolve target_devInfo when not already populated (e.g., no waitfor command, or explicit -uf-cmd mode).
        // This must happen for ALL policies — not just IF_NEWER — because the resolved version
        // determines the IMG_FLAG_useAlternateMD5 flag needed for correct checksum computation.
        if (!target_devInfo) {
            if (((target & fwUpdate::TARGET_IMX5) && (devInfo->hardwareType == IS_HARDWARE_TYPE_IMX)) ||
                ((target & fwUpdate::TARGET_GPX1) && (devInfo->hardwareType == IS_HARDWARE_TYPE_GPX))) {
                // Target is the same device as the host — use its dev info directly
                remoteDevInfo = *devInfo;
                target_devInfo = &remoteDevInfo;
            } else {
                // Target is a different device (e.g., GPX proxied through IMX) — request version
                // info from the target via the firmware update protocol
                if (!pingTimeoutExpires) {
                    pingTimeoutExpires = current_timeMs() + 3000;
                    LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Requesting target version info (up to 3s)...");
                }
                fwUpdate_requestVersionInfo(target);
                if (current_timeMs() < pingTimeoutExpires) {
                    return;  // stay CMD_QUEUED, re-enter on next cycle
                }
                // Timeout — proceed without target version
                LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_WARN, "Timed out waiting for target version info; proceeding with upload");
                pingTimeoutExpires = 0;
            }

            // Set useAlternateMD5 for legacy firmware compatibility (only needed for GPX 2.0.0)
            if (target_devInfo && remoteDevInfo.firmwareVer[0] == 2 && remoteDevInfo.firmwareVer[1] == 0 && remoteDevInfo.firmwareVer[2] == 0)
                flags |= fwUpdate::IMG_FLAG_useAlternateMD5;
            else if (!target_devInfo)
                flags |= fwUpdate::IMG_FLAG_useAlternateMD5;  // unknown version, use alternate MD5 for safety
        }

        // If the target is in bootloader mode, the app firmware version is unobservable
        // (firmwareVer reflects the bootloader identifier, e.g. ISbl "v6j" → [6, 'j']).
        // Any version comparison is wasted energy and tends to look "newer" because
        // 'j' (106) dwarfs typical app majors. Promote the policy to FORCE so the
        // upload proceeds and downstream protocol checks are bypassed too. SKIP is
        // honored — if the caller explicitly asked to skip, we still skip.
        if (target_devInfo && target_devInfo->hdwRunState == HDW_STATE_BOOTLOADER &&
            effectivePolicy != UPDATE_POLICY_SKIP && effectivePolicy != UPDATE_POLICY_FORCE) {
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO,
                "Target is in bootloader mode; promoting policy to FORCE (app version is unobservable).");
            effectivePolicy = UPDATE_POLICY_FORCE;
            forceUpdate = true;
        }

        // IF_NEWER: compare image version against target's current firmware version
        if (effectivePolicy == UPDATE_POLICY_IF_NEWER && target_devInfo) {
            dev_info_t imageDevInfo = {};
            uint16_t parsed = 0;
            bool hasImageVer = false;

            if (cmd.hasArg("image-version")) {
                std::string verStr = cmd.getArg("image-version");
                // devInfoFromString expects a 'v' or 'fw' prefix for version parsing
                if (!verStr.empty() && verStr[0] != 'v' && verStr.substr(0, 2) != "fw")
                    verStr.insert(0, "v");
                parsed = utils::devInfoFromString(verStr, imageDevInfo);
                if (parsed & utils::DV_BIT_FIRMWARE_VER)
                    hasImageVer = true;
            }

            // Fallback: try to parse version from the filename itself
            if (!hasImageVer && !filename.empty()) {
                parsed = utils::devInfoFromString(filename, imageDevInfo);
                if (parsed & utils::DV_BIT_FIRMWARE_VER)
                    hasImageVer = true;
            }

            if (hasImageVer) {
                std::string imageVerStr = utils::devInfoToString(imageDevInfo, utils::DV_BIT_FIRMWARE_VER);
                std::string targetVerStr = utils::devInfoToString(*target_devInfo, utils::DV_BIT_FIRMWARE_VER);

                // Only compare fields that were actually parsed from the image version string
                int64_t cmp = utils::compareFirmwareVersions(imageDevInfo, *target_devInfo, parsed);
                if (cmp <= 0) {
                    LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO,
                        "Ignoring Update. New firmware %s is older than device %s.",
                        imageVerStr.c_str(), targetVerStr.c_str());
                    cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
                    cmd.resultMsg = "Target is already up to date.";
                    return;
                }
            }
            // If no image version available, fall through to upload (can't compare)
        }

        fwUpdate::update_status_e status = initializeUpload(target, filename, slotNum, flags, forceUpdate, chunkSize, progressRate);
        if (status < fwUpdate::NOT_STARTED) {
            // there was an error -- probably should flush the command queue
            cmd.resultMsg = utils::string_format("Error initiating Firmware upload: [%s] %s", filename.c_str(), fwUpdate_getStatusName(status));
            handleCommandError(cmd, -1, cmd.resultMsg.c_str());
            cmd.status = ISFwUpdaterCmd::CMD_ERROR;
        } else {
            nextStartAttempt = current_timeMs() + attemptInterval;
            // session_status = fwUpdate::NOT_STARTED;
            cmd.status = ISFwUpdaterCmd::CMD_IN_PROCESS;
            cmd.resultMsg = "Initiating image upload.";
        }
    } else {
        if (fwUpdate_getSessionStatus() == fwUpdate::NOT_STARTED) {
            if (fwUpdate_getLastMessageAge() > 1500) {
                // if we have already made the initial request, but haven't yet received a response after 1500ms...
                // we'll resend a request periodically until we do... but we vary the time between requests

                if (startAttempts >= maxAttempts) {
                    // if we've reached our "maxAttempts", then we'll wait an additional period of time.
                    // backoff (wait attemptInternal * 3), and then try again.  Eventually, we will timeout below if there is a larger issue.
                    startAttempts = 0;
                    nextStartAttempt = current_timeMs() + (attemptInterval * 10);
                } else {
                    // otherwise, check if it's time to send the next update
                    if (nextStartAttempt < current_timeMs()) {// time has elapsed, so re-issue request to update
                        nextStartAttempt = current_timeMs() + attemptInterval;
                        if (fwUpdate_requestUpdate()) {
                            startAttempts++;
                            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_DEBUG, "[%s : %d] :: Requesting Firmware Update start (Attempt %d)", portName(port), devInfo->serialNumber, startAttempts);
                        } else {
                            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_ERROR, "Error attempting to initiate Firmware Update");
                        }
                    }
                }
            }
        }

        if (session_status == fwUpdate::FINISHED) {
            cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
            cmd.resultMsg = "Upload successful.";
            targetUploadPerformed[target] = true;
        } else if (session_status < fwUpdate::NOT_STARTED) {
            cmd.status = ISFwUpdaterCmd::CMD_ERROR;
            cmd.resultMsg = utils::string_format("Error: %s", fwUpdate_getNiceStatusName(fwUpdate_getSessionStatus()));
        } else {
            cmd.status = ISFwUpdaterCmd::CMD_IN_PROCESS;
            cmd.resultMsg = "Uploading firmware image.";
        }
    }
}

void ISFirmwareUpdater::cmd_resetDevice(ISFwUpdaterCmd& cmd) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    bool force = (cmd.getArg("force", "false") == "true");

    if (!force) {
        // Skip reset if target's policy is SKIP
        update_policy_e policy = getEffectivePolicy(cmd.step);
        if (policy == UPDATE_POLICY_SKIP) {
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Skipping reset (policy: SKIP)");
            cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
            cmd.resultMsg = "Reset skipped (policy: SKIP).";
            return;
        }

        // Skip reset if no upload was actually performed for this target (e.g. IF_NEWER with same version)
        if (targetUploadPerformed.find(target) == targetUploadPerformed.end()) {
            LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Skipping reset — no upload was performed for this target");
            cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
            cmd.resultMsg = "Reset skipped (no upload performed).";
            return;
        }
    }

    bool hard = (cmd.getArg("type", "soft") == "hard");
    if (cmd.hasArg("type") && (cmd["type"] == "tobl")) {
        fwUpdate_requestReset(target, fwUpdate::RESET_INTO_BOOTLOADER);
    } else {
        fwUpdate_requestReset(target, hard ? fwUpdate::RESET_HARD : fwUpdate::RESET_SOFT);
    }

    LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, "Requesting target reset (%s)", hard ? "hard" : "soft");

    cmd.resultMsg = utils::string_format("Performing %s Reset.", (hard ? "Hardware" : "Software" ));
    cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;
}

void ISFirmwareUpdater::cmd_finish(ISFwUpdaterCmd& cmd) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    // mark all queued commands to "NOT_EXECUTED" so that our checks for QUEUED are happy.
    for (auto& cmd : commands)
        if (cmd.status == ISFwUpdaterCmd::CMD_QUEUED)
            cmd.status = ISFwUpdaterCmd::CMD_NOT_EXECUTED;
    bool reportErrors = (cmd.args.size() == 1 && cmd[0] == "true");
    cmd.resultMsg = utils::string_format("Firmware Update completed %s", reportErrors ? "with errors. Please review update log for specifics." : "successfully.");
    cmd.status = ISFwUpdaterCmd::CMD_SUCCESS;

    if (reportErrors)
        LOG_FWUPDATE_STATUS(IS_LOG_LEVEL_INFO, cmd.resultMsg.c_str());
}

void ISFirmwareUpdater::initialize() {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    nextStartAttempt = 0;                                   //!< the number of millis (uptime?) that we will next attempt to start an upgrade
    startAttempts = 0;                                      //!< the number of attempts that have been made to request that an update be started

    maxAttempts = 5;                                        //!< the maximum number of attempts that will be made before we give up.
    attemptInterval = 350;                                  //!< the number of millis between attempts - default is to try every quarter-second, for 5 seconds

    last_resent_chunk = 0;                                  //!< the chunk id of the last/previous received req_resend  (are we getting multiple requests for the same chunk?)
    resent_chunkid_count = 0;                               //!< the number of consecutive req_resend for the same chunk, reset if the current resend request is different than last_resent_chunk
    resent_chunkid_time = 0;                                //!< time (ms uptime) of the first failed write for the given chunk id (also reset if the resend request's chunk is different)

    chunkDelay = 25;                                        //!< provides a throttling mechanism
    resendChunkDelay = 250;                                   //!< provides a throttling mechanism
    nextChunkSend = 0;                                      //!< don't send the next chunk until this time has expired.
    updateStartTime = 0;                                    //!< the system time when the firmware was started (for performance reporting)

    // pfnStatus_cb = nullptr;
    toHost.clear();                                         //!< a "data stream" that contains the raw-byte responses from the local FirmwareUpdateDevice (to the host)
    commands.clear();

    activeStep.clear();                                     //!< the name of the currently executing step name, from the manifest when available
    activeCmd = &nullCmd;                                    //!< the name (without parameters) of the currently executing command
    failLabel.clear();                                      //!< a label to jump to, when an error occurs
    // requestPending = false;                                 //!< true is an update has been requested, but we're still waiting on a response.
    slotNum = 0, chunkSize = 512, progressRate = 250;
    forceUpdate = false;
    pingInterval = 1000;                                    //!< delay between attempts to communicate with a target device
    pingNextRetry = 0;                                      //!< time for next ping
    pingTimeoutMs = 0;                                      //!< time when the ping operation will timeout if no response before then
    timeoutLabel.clear();                                   //!< a label to jump to, when a "waitfor" times out (which is not always an error)
    pauseUntil = 0;                                         //!< delays next command execution until this time (but still allows the fwUpdate to step/receive responses).
    filename.clear();
    target = fwUpdate::TARGET_HOST;

    //mz_zip_archive *zip_archive = nullptr; //!< is NOT null IF we are updating from a firmware package (zip archive).
    //fwUpdate::FirmwareUpdateDevice *deviceUpdater = nullptr;
    remoteDevInfo = {};
    targetUploadPerformed.clear();
    logLevel = IS_LOG_LEVEL_INFO;                           //!< default log level to show

    // NOTE: stepPolicies, policyPatterns, and defaultPolicy are intentionally
    // NOT reset here. They are set by policy commands which run BEFORE the
    // package command calls initialize() via processPackageManifest().

    updateState.resetState();
}


std::vector<ISFirmwareUpdater::StepInfo> ISFirmwareUpdater::getStepSummary() const {
    std::lock_guard<std::recursive_mutex> lock(mutex);
    std::vector<StepInfo> result;
    std::map<std::string, size_t> targetIndex;  // targetName -> index in result

    for (const auto& cmd : commands) {
        if (cmd.step.empty() || cmd.cmd != "target" || !cmd.args.count("target"))
            continue;

        std::string targetName = cmd.args.at("target");
        auto it = targetIndex.find(targetName);
        if (it == targetIndex.end()) {
            // First time seeing this target — create a new entry
            targetIndex[targetName] = result.size();
            result.push_back({targetName, {cmd.step}, getEffectivePolicy(cmd.step)});
        } else {
            // Additional step for the same target — just add the step label
            auto& existing = result[it->second];
            if (std::find(existing.stepLabels.begin(), existing.stepLabels.end(), cmd.step) == existing.stepLabels.end()) {
                existing.stepLabels.push_back(cmd.step);
            }
        }
    }
    return result;
}

void ISFirmwareUpdater::setStepPolicy(const std::string& stepLabel, update_policy_e policy) {
    stepPolicies[stepLabel].policy = policy;
}

void ISFirmwareUpdater::setPolicyPattern(const std::string& pattern, update_policy_e policy) {
    policyPatterns.emplace_back(pattern, policy);
}

void ISFirmwareUpdater::setDefaultPolicy(update_policy_e policy) {
    defaultPolicy = policy;
}

update_policy_e ISFirmwareUpdater::getEffectivePolicy(const std::string& stepLabel) const {
    auto it = stepPolicies.find(stepLabel);
    if (it != stepPolicies.end() && it->second.policy != UPDATE_POLICY_DEFAULT)
        return it->second.policy;

    if (defaultPolicy != UPDATE_POLICY_DEFAULT)
        return defaultPolicy;

    if (forceUpdate)
        return UPDATE_POLICY_FORCE;

    return UPDATE_POLICY_IF_NEWER;
}

/**
 * Case-insensitive substring match. Returns true if haystack contains needle (ignoring case).
 */
static bool icontains(const std::string& haystack, const std::string& needle) {
    if (needle.size() > haystack.size()) return false;
    auto it = std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end(),
        [](char a, char b) { return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b)); });
    return it != haystack.end();
}

/**
 * Parses a policy string ("skip", "if-newer", "force", "always") into an update_policy_e value.
 */
static update_policy_e parsePolicyString(const std::string& str) {
    if (str == "skip") return UPDATE_POLICY_SKIP;
    if (str == "if-newer") return UPDATE_POLICY_IF_NEWER;
    if (str == "force" || str == "always") return UPDATE_POLICY_FORCE;
    return UPDATE_POLICY_DEFAULT;
}


/**
 * Locates the next queued, command in the command stack and returns it
 * @return the next available command which is still in the CMD_QUEUED state, or returns nullCmd if none
 */
ISFwUpdaterCmd& ISFirmwareUpdater::getNextQueuedCmd(ISFwUpdaterCmd* curCmd) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    auto cmd_it = (!curCmd || curCmd == &nullCmd ? commands.begin() : std::find(commands.begin(), commands.end(), *curCmd));
    for (; cmd_it != commands.end(); cmd_it++ ) {
        if (cmd_it->status == ISFwUpdaterCmd::CMD_QUEUED) return *cmd_it;
    }
    return nullCmd;
}

/**
 * Locates the first command with the step label matching "stepLabel" and returns it. This function performs
 * no checks to see if the returned command has been previous executed, etc.
 * @return the first available command which has the specified step label
 */
ISFwUpdaterCmd& ISFirmwareUpdater::jumpToStep(const std::string& stepLabel) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    for (auto& cmd : commands) {
        if (cmd.step == stepLabel) return cmd;
    }
    return nullCmd;
}

/**
 * Injects a series of commands into the command queue, based on the contents of the manifest, which defines
 * the steps/commands necessary to perform a package update.
 * @param manifest YAML tree the defines the manifest to parse
 * @param archive a pointer to the mz_zip_archive associated with this manifest, if any (otherwise nullptr).
 * @return 0 on success, otherwise PKG_ERR_*.
 */
ISFirmwareUpdater::pkg_error_e ISFirmwareUpdater::processPackageManifest(YAML::Node& manifest, mz_zip_archive* archive = nullptr) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    initialize();
    YAML::Node images = manifest["images"];
    if (!images.IsMap())
        return PKG_ERR_INVALID_IMAGES; // images must be a map (of maps)

    YAML::Node steps = manifest["steps"];

    if (!steps.IsSequence())
        return PKG_ERR_INVALID_STEPS; // steps must be a sequence (of maps)

    for (auto steps_iv : steps) {
        // each step in the list of steps defines a label, which is then also a sequence of commands
        for (auto step : steps_iv) {
            YAML::Node label = step.first;
            YAML::Node cmds = step.second;

            if (!label.IsScalar())
                return PKG_ERR_INVALID_TARGET; // key must identify a target name

            if (!cmds.IsSequence())
                return PKG_ERR_NO_ACTIONS; // actions must be a sequence (of maps)

            auto labelName = label.as<std::string>();
            commands.emplace_back(labelName, "");

            for (auto cmds_iv : cmds) {
                for (auto cmd : cmds_iv) {
                    auto cmd_name = cmd.first.as<std::string>();
                    auto cmd_arg = cmd.second.as<std::string>();

                    if (cmd_name == "image") {
                        uint32_t image_size = 0;
                        md5hash_t image_hash = {};
                        // uint8_t image_version[4] = {};

                        size_t file_size = 0;
                        md5hash_t file_hash = {};

                        // this is an "image" command, so lookup the key in the images map
                        YAML::Node image = images[cmd_arg];
                        if (!image.IsDefined() || !image.IsMap())
                            return PKG_ERR_IMAGE_INVALID_REFERENCE; // step references invalid or non-existent image in manifest

                        if (!image["filename"].IsDefined() && !image["filename"].IsScalar())
                            return PKG_ERR_IMAGE_UNKNOWN_PATH; // an image must define AT LEAST a filename

                        std::string filename = image["filename"].as<std::string>();
                        if (archive) {
                            uint32_t f_index = 0;
                            mz_zip_archive_file_stat file_stat;
                            if (mz_zip_reader_locate_file_v2(archive, filename.c_str(), nullptr, 0, &f_index)) {
                                mz_zip_reader_file_stat(archive, f_index, &file_stat);
                                file_size = file_stat.m_uncomp_size;
                                filename.insert(0, "pkg://");
                            } else
                                return PKG_ERR_IMAGE_FILE_NOT_FOUND;
                        } else {
                            std::ifstream stream(filename, std::ios_base::binary);
                            if (md5_stream_details(stream, file_size, file_hash) != 0) {
                                // If you're here, and you don't know why.. check your current working directory.
                                // std::string workDir = ISFileManager::CurrentWorkingDirectory();
                                // printf("%s", workDir.c_str());
                                return PKG_ERR_IMAGE_FILE_NOT_FOUND; // file is invalid or non-existent
                            }
                        }

                        // the following are optional parameters; including them in the manifest image forces us to validate that parameter BEFORE allowing the image to be send to the device
                        if (image["size"].IsDefined() && image["size"].IsScalar()) {
                            image_size = image["size"].as<int>();

                            if (image_size != file_size)
                                return PKG_ERR_IMAGE_FILE_SIZE_MISMATCH; // file size doesn't match the manifest image size
                        }

                        // We only do MD5 validation on non-zipped files...
                        // TODO: we should do this eventually, but its costly, since we end up extracting/decoding multiple times
                        if (!archive) {
                            if (image["md5sum"].IsDefined() && image["md5sum"].IsScalar()) {
                                std::string hash_str = image["md5sum"].as<std::string>();
                                image_hash = md5_from_string(hash_str);

                                if (memcmp(&image_hash, &file_hash, sizeof(md5hash_t)) != 0)
                                    return PKG_ERR_IMAGE_FILE_MD5_MISMATCH; // file hash doesn't make the manifest image hash
                            }
                        }

                        std::string args = "filename="+filename;
                        if (image["slot"].IsDefined() && image["slot"].IsScalar())
                            args += ",slot="+image["slot"].as<std::string>();

                        // Extract image version from manifest (for IF_NEWER comparison)
                        if (image["version"].IsDefined() && image["version"].IsScalar()) {
                            std::string verStr = image["version"].as<std::string>();
                            args += ",image-version=" + verStr;
                        }

                        // Resolve update policy for this step:
                        //   1. Exact stepPolicies entry (from CLI setStepPolicy) — highest priority
                        //   2. Pattern match from policyPatterns (from CLI setPolicyPattern)
                        //   3. Manifest's update: field (only if no CLI default was set)
                        if (stepPolicies.find(labelName) == stepPolicies.end() ||
                            stepPolicies[labelName].policy == UPDATE_POLICY_DEFAULT) {
                            // Try deferred pattern match against step label
                            for (auto& [pattern, policy] : policyPatterns) {
                                if (icontains(labelName, pattern)) {
                                    stepPolicies[labelName].policy = policy;
                                    break;
                                }
                            }
                        }
                        // Only apply manifest's update: field if no CLI-level policy
                        // (neither per-step, pattern, nor defaultPolicy) would override it.
                        if (defaultPolicy == UPDATE_POLICY_DEFAULT &&
                            (stepPolicies.find(labelName) == stepPolicies.end() ||
                             stepPolicies[labelName].policy == UPDATE_POLICY_DEFAULT)) {
                            if (image["update"].IsDefined() && image["update"].IsScalar()) {
                                stepPolicies[labelName].policy = parsePolicyString(image["update"].as<std::string>());
                            }
                        }

                        commands.emplace_back(labelName, "upload", args);
                    } else {
                        // anything that isn't "image" is treated like a normal command
                        commands.emplace_back(labelName, cmd_name, cmd_arg);
                    }
                }
            }
        }
    }

    return PKG_SUCCESS;
}

ISFirmwareUpdater::pkg_error_e ISFirmwareUpdater::processPackageManifest(const std::string& manifest_file) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    try {
        YAML::Node manifest = YAML::LoadFile(manifest_file);

        if (manifest.IsNull())
            return PKG_ERR_PACKAGE_FILE_ERROR;

        // always process the manifest from the manifest's path
        std::string parentDir;
        if (ISFileManager::getParentDirectory(manifest_file, parentDir)) {
            if (chdir(parentDir.c_str()))
            {   // Handle error
            }
        }

        return processPackageManifest(manifest);
    } catch (YAML::BadFile& e) {
        return PKG_ERR_PACKAGE_FILE_ERROR;
    } catch (YAML::ParserException& e) {
        return PKG_ERR_PACKAGE_FILE_ERROR;
    } catch (YAML::BadConversion& e) {
        return PKG_ERR_PACKAGE_FILE_ERROR;;
    } catch (YAML::RepresentationException& e) {
        return PKG_ERR_PACKAGE_FILE_ERROR;
    }
}

ISFirmwareUpdater::pkg_error_e ISFirmwareUpdater::openFirmwarePackage(const std::string& pkg_file) {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    mz_bool status;
    size_t file_size;
    void *p;
    pkg_error_e result = PKG_SUCCESS;

    if (!zip_archive)
        zip_archive = (mz_zip_archive *)malloc(sizeof(mz_zip_archive));

    // initialize the archive struct and open the file
    mz_zip_zero_struct(zip_archive);
    status = mz_zip_reader_init_file(zip_archive, pkg_file.c_str(), 0);
    if (!status)
        return PKG_ERR_PACKAGE_FILE_ERROR;

    p = mz_zip_reader_extract_file_to_heap(zip_archive, "manifest.yaml", &file_size, 0);
    if (!p || (file_size == 0))
        return PKG_ERR_NO_MANIFEST;

    std::string casted_memory(static_cast<char*>(p), file_size);
    std::istringstream stream(casted_memory);
    YAML::Node manifest = YAML::Load(stream);
    if (manifest)
        result = processPackageManifest(manifest, zip_archive);
    mz_free(p);

    // TODO: I can't make up my mind... to keep the zip-reader available for possible future file extractions, or close it and reopen it each time.  We're only talking about a dozen files at max, and most time 2-5 files on average.
    // mz_zip_reader_end(zip_archive);
    return result;
}

ISFirmwareUpdater::pkg_error_e ISFirmwareUpdater::cleanupFirmwarePackage() {
    // std::lock_guard<std::recursive_mutex> lock(mutex);

    if (srcFile) {
        delete srcFile;
        srcFile = nullptr;
    }
    if (srcFileBytes) {
        mz_free(srcFileBytes);
        srcFileBytes = nullptr;
    }
    if (zip_archive) {
        mz_zip_reader_end(zip_archive);
        mz_free(zip_archive);
        zip_archive = nullptr;
    }
    return PKG_SUCCESS;
}
