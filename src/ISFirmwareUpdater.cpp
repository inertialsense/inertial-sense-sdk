//
// Created by kylemallory on 8/18/23.
//

#include "ISFirmwareUpdater.h"

/**
 * Specifies the target device that you wish to update. This will attempt an initial REQ_VERSION request of that device
 * to determine if the device is available, and what firmware it is currently running.  This is a non-blocking call,
 * and will return immediately.
 * @param _target
 */
void ISFirmwareUpdater::setTarget(fwUpdate::target_t _target) {
    session_target = target = _target;
    session_image_slot = 0;
    
    // request version info from the target
    target_devInfo = nullptr;
    pauseUntil = current_timeMs() + 2000; // wait for 2 seconds for a response from the target (should be more than enough)
}

/**
 * Specifies a series of commands to perform, in sequence, in order to perform a "complex" update.
 * @param cmds a vector of strings, one command (and arguments) per entry.
 * @return true
 */
bool ISFirmwareUpdater::setCommands(std::vector<std::string> cmds) {
    commands = cmds;
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

        std::vector<dfu::DFUDevice*> devices;
        dfu::ISDFUFirmwareUpdater::getAvailableDevices(devices);
        // dfuUpdater = new ISDFUFirmwareUpdater(usbDevice, (uint32_t)target, deviceId);
    }

//    if (dfuUpdater->isConnected() == false) {
//        delete dfuUpdater;
//        return fwUpdate::ERR_UNKNOWN;
//    }

    fwUpdate::update_status_e result = (fwUpdate_requestUpdate(target, 0, 0, chunkSize, fileSize, session_md5, progressRate) ? fwUpdate::NOT_STARTED : fwUpdate::ERR_UNKNOWN);
    if (pfnInfoProgress_cb != nullptr)
        pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, "Requested firmware update with Image '%s', md5: %s", fwUpdate_getSessionTargetName(), filename.c_str(), md5_to_string(session_md5).c_str());
    return result;
}


fwUpdate::update_status_e ISFirmwareUpdater::initializeUpdate(fwUpdate::target_t _target, const std::string &filename, int slot, int flags, bool forceUpdate, int chunkSize, int progressRate)
{
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
            : md5_file_details(srcFile, fileSize, session_md5);
    if ( hashError != 0 )
        return fwUpdate::ERR_INVALID_IMAGE;

    updateStartTime = current_timeMs();
    nextStartAttempt = current_timeMs() + attemptInterval;
    fwUpdate::update_status_e result = (fwUpdate_requestUpdate(_target, slot, flags, chunkSize, fileSize, session_md5, progressRate) ? fwUpdate::NOT_STARTED : fwUpdate::ERR_UNKNOWN);
    if (pfnInfoProgress_cb != nullptr)
        pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, "Initiating update with image '%s' to target slot %d (%d bytes, md5: %s)", filename.c_str(), slot, fileSize, md5_to_string(session_md5).c_str());
    return result;
}

bool ISFirmwareUpdater::fwUpdate_handleVersionResponse(const fwUpdate::payload_t& msg) {
    memset(&remoteDevInfo, 0, sizeof(dev_info_t));
    if ((msg.data.version_resp.resTarget > fwUpdate::TARGET_HOST) && (msg.data.version_resp.resTarget <= fwUpdate::TARGET_MAXNUM) && (msg.data.version_resp.resTarget != target)) {
        return false;
    }

    remoteDevInfo.serialNumber = msg.data.version_resp.serialNumber;
    remoteDevInfo.hardwareType = msg.data.version_resp.hardwareType;
    memcpy(remoteDevInfo.hardwareVer, msg.data.version_resp.hardwareVer, 4);
    memcpy(remoteDevInfo.firmwareVer, msg.data.version_resp.firmwareVer, 4);
    remoteDevInfo.buildYear = msg.data.version_resp.buildYear;
    remoteDevInfo.buildMonth = msg.data.version_resp.buildMonth;
    remoteDevInfo.buildDay = msg.data.version_resp.buildDay;
    remoteDevInfo.buildHour = msg.data.version_resp.buildHour;
    remoteDevInfo.buildMinute = msg.data.version_resp.buildMinute;
    remoteDevInfo.buildSecond = msg.data.version_resp.buildSecond;
    remoteDevInfo.buildMillisecond = msg.data.version_resp.buildMillis;
    remoteDevInfo.buildType = msg.data.version_resp.buildType;
    target_devInfo = &remoteDevInfo;

    if (pfnInfoProgress_cb != nullptr) {
        if ((remoteDevInfo.hardwareType >= IS_HARDWARE_TYPE_UINS) && (remoteDevInfo.hardwareType <= IS_HARDWARE_TYPE_GPX)) {
            pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, "Received version info: %s-%d.%d.%d:SN-%05d, Fw %d.%d.%d.%d (%d)", g_isHardwareTypeNames[remoteDevInfo.hardwareType],
                               remoteDevInfo.hardwareVer[0], remoteDevInfo.hardwareVer[1], remoteDevInfo.hardwareVer[2], (remoteDevInfo.serialNumber != 0xFFFFFFFF ? remoteDevInfo.serialNumber : 0),
                               remoteDevInfo.firmwareVer[0], remoteDevInfo.firmwareVer[1], remoteDevInfo.firmwareVer[2], remoteDevInfo.firmwareVer[3],
                               remoteDevInfo.buildNumber);
        } else {
            pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, "Received version info: %s, Fw %d.%d.%d.%d", fwUpdate_getTargetName(msg.data.version_resp.resTarget),
                               remoteDevInfo.firmwareVer[0], remoteDevInfo.firmwareVer[1], remoteDevInfo.firmwareVer[2], remoteDevInfo.firmwareVer[3]);
        }
    }

    return true;
}

int ISFirmwareUpdater::fwUpdate_getImageChunk(uint32_t offset, uint32_t len, void **buffer) {
    if (srcFile && (srcFile->rdstate() == 0)) {
        srcFile->seekg((std::streampos)offset);
        len = _MIN(len, session_image_size - (uint32_t)srcFile->tellg());
        srcFile->read((char *)*buffer, len);
        return srcFile->gcount();
    }
    return -1;
}

bool ISFirmwareUpdater::fwUpdate_handleUpdateResponse(const fwUpdate::payload_t &msg) {
    if (session_id != msg.data.update_resp.session_id)
        return false; // this message isn't for us...

    if ((session_id == msg.data.update_resp.session_id) && (session_status == msg.data.update_resp.status) && (session_status != fwUpdate::INITIALIZING))
        return true; // we're receiving duplicate messages, so ignore them

    session_status = msg.data.update_resp.status;
    session_total_chunks = msg.data.update_resp.totl_chunks;

    if (pfnInfoProgress_cb != nullptr) {
        if (session_status == fwUpdate::READY)
            pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_MORE_DEBUG, "Received remote response: %s; Expecting %d chunks.", fwUpdate_getStatusName(session_status), session_total_chunks);
        else
            pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_MORE_DEBUG, "Received remote response: %s", fwUpdate_getStatusName(session_status));
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
            if (pfnInfoProgress_cb != nullptr)
                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_ERROR, "To many resends of the same chunk; giving up with error %s", fwUpdate_getNiceStatusName(session_status));
            return false;
        }
    } else {
        resent_chunkid_count = 0;
        resent_chunkid_time = current_ms;
    }

    if (pfnInfoProgress_cb != nullptr)
        pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_DEBUG, "Remote requested resend of %d: %d", msg.data.req_resend.chunk_id, msg.data.req_resend.reason);
    nextChunkSend = current_timeMs() + nextChunkDelay;
    return fwUpdate_sendNextChunk(); // we don't have to send this right away, but sure, why not!
}

std::mutex progress_mutex;
bool ISFirmwareUpdater::fwUpdate_handleUpdateProgress(const fwUpdate::payload_t &msg) {
    progress_mutex.lock();

    if (session_status >= fwUpdate::NOT_STARTED)
        session_status = msg.data.progress.status; // don't overwrite an error status in the event of racing messages.

    float percent = msg.data.progress.num_chunks/(float)(msg.data.progress.totl_chunks)*100.f;
    const char* message = msg.data.progress.msg_len ? (const char*)&msg.data.progress.message : "";

    if (pfnUploadProgress_cb != nullptr)
        pfnUploadProgress_cb(this, percent);

    if (pfnVerifyProgress_cb != nullptr)
        pfnVerifyProgress_cb(this, percent);

    if (pfnInfoProgress_cb != nullptr)
        pfnInfoProgress_cb(this, static_cast<ISBootloader::eLogLevel>(msg.data.progress.msg_level), message);

    progress_mutex.unlock();
    return true;
}

bool ISFirmwareUpdater::fwUpdate_handleDone(const fwUpdate::payload_t &msg) {
    session_status = msg.data.resp_done.status;
    return true;
}

bool ISFirmwareUpdater::fwUpdate_isDone()
{
    bool cmdsPending = hasPendingCommands();
    bool in_progress = ((fwUpdate_getSessionStatus() > fwUpdate::NOT_STARTED) && (fwUpdate_getSessionStatus() < fwUpdate::FINISHED));
    bool is_done = !(cmdsPending || requestPending || in_progress);
    return is_done;
}

bool ISFirmwareUpdater::fwUpdate_step(fwUpdate::msg_types_e msg_type, bool processed)
{
    uint32_t lastMsgAge = 0;
    static fwUpdate::update_status_e lastStatus = fwUpdate::NOT_STARTED;

    if ((pfnInfoProgress_cb != nullptr) && (lastStatus != session_status)) {
        pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_MORE_DEBUG, "Session status changed: %s", fwUpdate_getStatusName(session_status));
        lastStatus = session_status;
    }

    switch(session_status) {
        case fwUpdate::NOT_STARTED:
            if (!requestPending) {
                if (!commands.empty()) {
                    if (pauseUntil && pauseUntil > current_timeMs())
                        return fwUpdate::MSG_UNKNOWN; // means to delay execution of next command for some period of time..
                    pauseUntil = 0;
                    runCommand(commands[0]);
                }
            } else {
                lastMsgAge = fwUpdate_getLastMessageAge();
                if (lastMsgAge > 1500) {
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
                                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_DEBUG, "[%s : %d] :: Requesting Firmware Update start (Attempt %d)", portName, devInfo->serialNumber, startAttempts);
                            } else {
                                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_ERROR, "Error attempting to initiate Firmware Update");
                            }
                        }
                    }
                }
            }

            // nothing to do..
            break;
        case fwUpdate::INITIALIZING:
            requestPending = false;
            break;
        case fwUpdate::READY:
        case fwUpdate::IN_PROGRESS:
            requestPending = false;
            if (nextChunkSend < current_timeMs()) // don't send chunks too fast
                fwUpdate_sendNextChunk();
            break;
        case fwUpdate::FINALIZING:
            requestPending = false;
            break; // do nothing, just wait
        case fwUpdate::FINISHED:
            if (pfnInfoProgress_cb != nullptr)
                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, "Firmware uploaded in %0.1f seconds", (current_timeMs() - updateStartTime) / 1000.f);
            if (hasPendingCommands()) {
                requestPending = false;
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
                handleCommandError("upload", -1, "No Response from device.");
            }
            break;
        default:
            if (session_status < fwUpdate::NOT_STARTED) {
                handleCommandError("upload", -session_status, "Unexpected response from device : %s", fwUpdate_getSessionStatusName());
            }
            break;
    }

    uint32_t lastMsg = fwUpdate_getLastMessageAge();
    if ((lastMsg > timeout_duration) && (requestPending || ((session_status > fwUpdate::NOT_STARTED) && (session_status < fwUpdate::FINISHED))))
        session_status = fwUpdate::ERR_TIMEOUT;

    if (fwUpdate_isDone()) {
        // be sure to release/cleanup the source file after we are finished with it.
        if (srcFile) {
            delete srcFile;
            srcFile = nullptr;
        }
    }

    return (session_status != fwUpdate::NOT_STARTED);
}

bool ISFirmwareUpdater::fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t *buffer, int buff_len) {
    if ((dfuUpdater != nullptr) && ((target & fwUpdate::TARGET_DFU_FLAG) == fwUpdate::TARGET_DFU_FLAG)) {
        return dfuUpdater->fwUpdate_processMessage(buffer, buff_len);
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
    int result = comManagerSendData(pHandle, buffer, DID_FIRMWARE_UPDATE, buff_len, 0);
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
void ISFirmwareUpdater::handleCommandError(const std::string& cmd, int errCode, const char *errMsg, ...) {

    if (!commands.empty()) {
        requestPending = false;
        fwUpdate_resetEngine();
    }

    char buffer[256];
    va_list args;
    va_start(args, errMsg);
    VSNPRINTF(buffer, sizeof(buffer), errMsg, args);
    va_end(args);

    stepErrors.emplace_back(activeStep, cmd, buffer);

    if (pfnInfoProgress_cb != nullptr)
        pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_ERROR, buffer);

    if (failLabel.empty()) {
        // if no label has been specified, clear all commands and reset
        commands.clear();
        target = fwUpdate::TARGET_HOST;
        return;
    }

    // else, we have a failLabel defined.. let's skip all messages until its found
    while (!commands.empty() && (commands[0] != failLabel)) {
        commands.erase(commands.begin());
    }
}

void ISFirmwareUpdater::runCommand(std::string cmd) {
    if (pfnInfoProgress_cb != nullptr) {
        static std::string priorCmd;
        if (priorCmd != cmd)
            pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_MORE_DEBUG, "Executing manifest command \"%s\"", cmd.c_str());
        priorCmd = cmd;
    }

    std::vector<std::string> args;
    splitString(cmd, '=', args);
    if (!args.empty()) {
        activeCommand = args[0];
        if (args.size() > 1) {
            std::string remainder = args[1];
            splitString(args[1], ',', args);
        }

        if ((activeCommand == "package") && (args.size() == 1)) {
            bool isManifest = (args[0].length() >= 5) && (0 == args[0].compare (args[0].length() - 5, 5, ".yaml"));
            pkg_error_e err_result = isManifest ? processPackageManifest(args[0]) : openFirmwarePackage(args[0]);
            if (err_result != PKG_SUCCESS) {
                const char *err_msg = nullptr;
                switch (err_result) {
                    case PKG_SUCCESS:
                        break;
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
                }
                handleCommandError(activeCommand, err_result, "Error processing firmware package [%s] (Error code: %d) :: %s", args[0].c_str(), err_result, err_msg);

            }
        } else if (activeCommand[0] == ':') {
            // new step section/target - we should reset certain states here if needed
            activeStep = std::string(activeCommand.c_str() + 1);
            if (activeCommand == ":IMX5") setTarget(fwUpdate::TARGET_IMX5);
            else if (activeCommand == ":GPX1") setTarget(fwUpdate::TARGET_GPX1);
            else if (activeCommand == ":GNSS1") setTarget(fwUpdate::TARGET_SONY_CXD5610__1);
            else if (activeCommand == ":GNSS2") setTarget(fwUpdate::TARGET_SONY_CXD5610__2);
            else session_target = target = fwUpdate::TARGET_HOST;
            session_image_slot = slotNum = 0;
            failLabel.clear();
        } else if ((activeCommand == "target") && (args.size() == 1)) {
            if (args[0] == "IMX5") setTarget(fwUpdate::TARGET_IMX5);
            else if (args[0] == "GPX1") setTarget(fwUpdate::TARGET_GPX1);
            else if (args[0] == "GNSS1") setTarget(fwUpdate::TARGET_SONY_CXD5610__1);
            else if (args[0] == "GNSS2") setTarget(fwUpdate::TARGET_SONY_CXD5610__2);
            else {
                handleCommandError(activeCommand, -1, "Invalid Target specified: %s  (Valid targets are: IMX5, GPX1, GNSS1, GNSS2)", args[0].c_str());
            }
        } else if ((activeCommand == "on-error") && (args.size() == 1)) {
            failLabel = args[0].c_str();
            // all labels must start with a colon (:)
            if (failLabel[0] != ':') {
                failLabel.clear();
                handleCommandError(activeCommand, -1, "Invalid label. Labels must start with a colon (:).");
            }
        } else if ((activeCommand == "echo") && (args.size() == 1)) {
            // simply outputs the argument string to the progress handler
            // this is primarily for manifest authors to output information to the user
            std::string msg;
            joinStrings(args, ',', msg);
            if (pfnInfoProgress_cb != nullptr)
                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, msg.c_str());
        } else if ((activeCommand == "slot") && (args.size() == 1)) {
            slotNum = strtol(args[0].c_str(), nullptr, 10);
        } else if ((activeCommand == "timeout") && (args.size() == 1)) {
            fwUpdate_setTimeoutDuration(strtol(args[0].c_str(), nullptr, 10));
        } else if (activeCommand == "force") {
            forceUpdate = (args[0] == "true" ? true : false);
        } else if ((activeCommand == "chunk") && (args.size() == 1)) {
            chunkSize = strtol(args[0].c_str(), nullptr, 10);
        } else if ((activeCommand == "rate") && (args.size() == 1)) {
            progressRate = strtol(args[0].c_str(), nullptr, 10);
        } else if ((activeCommand == "delay") && (args.size() == 1)) {
            if (pfnInfoProgress_cb != nullptr)
                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_MORE_INFO, "Pausing for %0.2f seconds...", strtol(args[0].c_str(), nullptr, 10) / 1000.0);
            pauseUntil = current_timeMs() + strtol(args[0].c_str(), nullptr, 10);
        } else if ((activeCommand == "waitfor") && (args.size() >= 1) && (args.size() <= 3)) {
            if ((args.size() == 3) && (args[2] == "true")) {
                // we need to remove the "force" from this command, and then replace the existing command back on the queue.
                commands.erase(commands.begin()); // pop this command off the front
                commands.insert(commands.begin(), "" + activeCommand + "=" + args[0] + "," + args[1]); // and then re-insert it at the front again, but without the force argument

                // force a new query, even if we previously had devInfo for this target, and use default values
                pingTimeout = 0;
                pingInterval = 1000;
                target_devInfo = NULL;
            }
            if (!target_devInfo) {
                if (!pingTimeout) { // if pingTimeout == 0, then we haven't started yet, so parse args and setup schedule
                    pingTimeout = current_timeMs() + strtol(args[0].c_str(), nullptr, 10);
                    if (args.size() >= 2) {
                        pingInterval = strtol(args[1].c_str(), nullptr, 10);
                    }
                    if (pfnInfoProgress_cb != nullptr)
                        pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_MORE_INFO, "Requesting version info from target device (upto %0.2f seconds)...", strtol(args[0].c_str(), nullptr, 10) / 1000.0);
                    return; // returning now will force this command to be re-executed (until the timeout expires).
                }
                if (pingTimeout < current_timeMs()) {
                    pingTimeout = pingNextRetry = 0;
                    handleCommandError(activeCommand, -1, "Timeout waiting for response from target device.");
                } else if (pingNextRetry < current_timeMs()) {
                    pingNextRetry = current_timeMs() + pingInterval;
                    target_devInfo = nullptr;
                    fwUpdate_requestVersionInfo(target);

                    if (pfnInfoProgress_cb != nullptr)
                        pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_MORE_DEBUG, "Re-requesting version info from target device...");
                }
                return; // keep trying...
            }
        } else if ((activeCommand == "upload") && (args.size() == 1)) {
            filename = args[0];
            fwUpdate_resetEngine();

            // TODO move this to it's own function before we expand this any father

            uint8_t flags = 0;
            // check for non encrypted file CXD update slot 4 or slot 2 if .fpk
            if (((target & fwUpdate::TARGET_SONY_CXD5610) == fwUpdate::TARGET_SONY_CXD5610) && (slotNum == 4 || (slotNum == 2 && filename.substr(filename.find_last_of(".") + 1) == "fpk")))
                flags |= fwUpdate::IMG_FLAG_imageNotEncrypted;

            // any target which doesn't report version info will also expect the old MD5 digest
            if (!target_devInfo) {
                // TODO: We should be able to remove most of this after 2.1.0 has been released
                if ( ((target & fwUpdate::TARGET_IMX5) && (devInfo->hardwareType == IS_HARDWARE_TYPE_IMX)) ||
                     ((target & fwUpdate::TARGET_GPX1) && (devInfo->hardwareType == IS_HARDWARE_TYPE_GPX)) ) {
                    // just copy in the current "main" device's dev info, since they are the same device as the target
                    remoteDevInfo = *devInfo;
                    target_devInfo = &remoteDevInfo;
                } else if ((target & fwUpdate::TARGET_GPX1) && (devInfo->hardwareType == IS_HARDWARE_TYPE_IMX)) {
                    // let's see if we can get the GPX version from the IMX dev info (it should be in addInfo)
                    const char *gpxVInfo = strstr(devInfo->addInfo, "G2.");
                    if (gpxVInfo) {
                        int v1 = 0, v2 = 0, v3 = 0, v4 = 0, bn = 0;
                        if ((sscanf(gpxVInfo, "G%d.%d.%d.%d-%d", &v1, &v2, &v3, &v4, &bn) == 5) ||
                            (sscanf(gpxVInfo, "G%d.%d.%d-%d", &v1, &v2, &v3, &bn) == 4))
                        {
                            remoteDevInfo.hardwareType = IS_HARDWARE_TYPE_GPX;
                            remoteDevInfo.hardwareVer[0] = 1, remoteDevInfo.hardwareVer[1] = 0, remoteDevInfo.hardwareVer[2] = 3, remoteDevInfo.hardwareVer[3] = 0;
                            remoteDevInfo.firmwareVer[0] = v1, remoteDevInfo.firmwareVer[1] = v2, remoteDevInfo.firmwareVer[2] = v3, remoteDevInfo.firmwareVer[3] = v4;
                            target_devInfo = &remoteDevInfo;
                            if ((v1 == 2) && (v2 == 0) && (v3 == 0))
                                flags |= fwUpdate::IMG_FLAG_useAlternateMD5;
                        }
                    }
                } else
                    flags |= fwUpdate::IMG_FLAG_useAlternateMD5;
            }

            fwUpdate::update_status_e status = initializeUpdate(target, filename, slotNum, flags, forceUpdate, chunkSize, progressRate);

            if (status < fwUpdate::NOT_STARTED) {
                // there was an error -- probably should flush the command queue
                handleCommandError(activeCommand, -1, "Error initiating Firmware upload: [%s] %s", filename.c_str(), fwUpdate_getStatusName(status));
            } else {
                requestPending = true;
                nextStartAttempt = current_timeMs() + attemptInterval;
                // session_status = fwUpdate::NOT_STARTED;
            }
        } else if (activeCommand == "reset") {
            bool hard = (args.size() == 1 && args[0] == "hard");
            fwUpdate_requestReset(target, hard ? fwUpdate::RESET_HARD : fwUpdate::RESET_SOFT);
            if (pfnInfoProgress_cb != nullptr)
                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, "Requesting target reset (%s)", hard ? "hard" : "soft");
        } else if (activeCommand == "finish") {
            commands.clear();
            bool reportErrors = (args.size() == 1 && args[0] == "true");
            if (reportErrors && (pfnInfoProgress_cb != nullptr))
                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, "Firmware Update completed %s", reportErrors ? "with errors. Please review update log for specifics." : "successfully.");
        } else {
            // unknown command - ignore it
            if (pfnInfoProgress_cb != nullptr)
                pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_ERROR, "Unknown command: '%s'", activeCommand.c_str());
        }
    }

    // If we are here, we've successfully executed our command, and it can be removed from the command queue.
    if (!commands.empty())
        commands.erase(commands.begin()); // pop the command off the front
    else
        activeCommand.clear();
}

/**
 * Injects a series of commands into the command queue, based on the contents of the manifest, which defines
 * the steps/commands necessary to perform a package update.
 * @param manifest YAML tree the defines the manifest to parse
 * @param archive a pointer to the mz_zip_archive associated with this manifest, if any (otherwise nullptr).
 * @return 0 on success, otherwise PKG_ERR_*.
 */
ISFirmwareUpdater::pkg_error_e ISFirmwareUpdater::processPackageManifest(YAML::Node& manifest, mz_zip_archive* archive = nullptr) {
    YAML::Node images = manifest["images"];
    if (!images.IsMap())
        return PKG_ERR_INVALID_IMAGES; // images must be a map (of maps)

    YAML::Node steps = manifest["steps"];

    if (!steps.IsSequence())
        return PKG_ERR_INVALID_STEPS; // steps must be a sequence (of maps)

    for (auto steps_iv : steps) {
        // each step in the list of steps defines a target, which is then also a sequence of commands
        for (auto target : steps_iv) {
            YAML::Node key = target.first;
            YAML::Node actions = target.second;

            if (!key.IsScalar()) {
                return PKG_ERR_INVALID_TARGET; // key must identify a target name
            }

            if (!actions.IsSequence())
                return PKG_ERR_NO_ACTIONS; // actions must be a sequence (of maps)

            std::string step_name = key.as<std::string>();
            commands.push_back(":" + step_name);

            for (auto actions_iv : actions) {
                for (auto cmd : actions_iv) {
                    auto cmd_name = cmd.first.as<std::string>();
                    auto cmd_arg = cmd.second.as<std::string>();

                    if (cmd_name == "target") {
                        std::string target_name = cmd_arg;
                        if ((target_name != "IMX5") &&
                            (target_name != "GPX1") &&
                            (target_name != "GNSS1") &&
                            (target_name != "GNSS2") )
                            return PKG_ERR_UNSUPPORTED_TARGET; // target name is invalid/unsupported
                        commands.push_back("target=" + target_name);
                    } else if (cmd_name == "image") {
                        int image_slot = 0;
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
                            std::ifstream stream(filename);
                            if (md5_file_details((std::istream*)&stream, file_size, file_hash) != 0)
                                return PKG_ERR_IMAGE_FILE_NOT_FOUND; // file is invalid or non-existent
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

                        if (image["slot"].IsDefined() && image["slot"].IsScalar())
                            image_slot = image["slot"].as<int>();

                        commands.push_back("slot=" + std::to_string(image_slot));
                        commands.push_back("upload=" + filename);
                    } else {
                        // anything that isn't "image" is treated like a normal command
                        commands.push_back(cmd_name + "=" + cmd_arg);
                    }
                }
            }
        }
    }

    return PKG_SUCCESS;
}

ISFirmwareUpdater::pkg_error_e ISFirmwareUpdater::processPackageManifest(const std::string& manifest_file) {
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
    mz_bool status;
    size_t file_size;
    void *p;
    pkg_error_e result = PKG_SUCCESS;

    if (!zip_archive) {
        zip_archive = (mz_zip_archive *)malloc(sizeof(mz_zip_archive));
    }

    // initialize the archive struct and open the file
    mz_zip_zero_struct(zip_archive);
    status = mz_zip_reader_init_file(zip_archive, pkg_file.c_str(), 0);
    if (!status) {
        return PKG_ERR_PACKAGE_FILE_ERROR;
    }

    p = mz_zip_reader_extract_file_to_heap(zip_archive, "manifest.yaml", &file_size, 0);
    if (p && (file_size > 0)) {
        std::string casted_memory(static_cast<char*>(p), file_size);
        std::istringstream stream(casted_memory);
        YAML::Node manifest = YAML::Load(stream);
        if (manifest)
            result = processPackageManifest(manifest, zip_archive);
        mz_free(p);
    }

    // TODO: I can't make up my mind... to keep the zip-reader available for possible future file extractions, or close it and reopen it each time.  We're only talking about a dozen files at max, and most time 2-5 files on average.
    // mz_zip_reader_end(zip_archive);
    return result;
}

ISFirmwareUpdater::pkg_error_e ISFirmwareUpdater::cleanupFirmwarePackage() {
    if (srcFile) {
        delete srcFile;
        srcFile = nullptr;
    }
    if (zip_archive) {
        mz_zip_reader_end(zip_archive);
        free(zip_archive);
        zip_archive = nullptr;
    }
    return PKG_SUCCESS;
}
