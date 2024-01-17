//
// Created by kylemallory on 8/18/23.
//

#include "ISFirmwareUpdater.h"
#include "ISUtilities.h"
#include "util/md5.h"

bool ISFirmwareUpdater::setCommands(std::vector<std::string> cmds) {
    commands = cmds;
    return true;
}

fwUpdate::update_status_e ISFirmwareUpdater::initializeDFUUpdate(libusb_device* usbDevice, fwUpdate::target_t target, uint32_t deviceId, const std::string &filename, int progressRate)
{
    srand(time(NULL)); // get *some kind* of seed/appearance of a random number.

    size_t fileSize = 0;
    srcFile = new std::ifstream(filename, std::ios::binary);
    if (md5_file_details(*srcFile, fileSize, session_md5) != 0)
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
    printf("Requested Firmware Update to device '%s' with Image '%s', md5: %08x-%08x-%08x-%08x\n", fwUpdate_getSessionTargetName(), filename.c_str(), session_md5[0], session_md5[1], session_md5[2], session_md5[3]);
    return result;
}


fwUpdate::update_status_e ISFirmwareUpdater::initializeUpdate(fwUpdate::target_t _target, const std::string &filename, int slot, int flags, bool forceUpdate, int chunkSize, int progressRate)
{
    srand(time(NULL)); // get *some kind* of seed/appearance of a random number.

    size_t fileSize = 0;
    srcFile = new std::ifstream(filename, std::ios::binary);
    if (md5_file_details(*srcFile, fileSize, session_md5) != 0)
        return fwUpdate::ERR_INVALID_IMAGE;
    // TODO: We need to validate that this firmware file is the correct file for this target, and that its an actual update (unless 'forceUpdate' is true)

    updateStartTime = current_timeMs();
    nextStartAttempt = current_timeMs() + attemptInterval;
    fwUpdate::update_status_e result = (fwUpdate_requestUpdate(_target, slot, flags, chunkSize, fileSize, session_md5, progressRate) ? fwUpdate::NOT_STARTED : fwUpdate::ERR_UNKNOWN);
    printf("Requested Firmware Update to device '%s' with Image '%s', md5: %08x-%08x-%08x-%08x\n", fwUpdate_getSessionTargetName(), filename.c_str(), session_md5[0], session_md5[1], session_md5[2], session_md5[3]);
    return result;
}

int ISFirmwareUpdater::fwUpdate_getImageChunk(uint32_t offset, uint32_t len, void **buffer) {
    if (srcFile && srcFile->is_open()) {
        srcFile->seekg((std::streampos)offset, srcFile->beg);
        len = _MIN(len, session_image_size - (uint32_t)srcFile->tellg());
        srcFile->read((char *)*buffer, len);
    }
    return len;
}

bool ISFirmwareUpdater::fwUpdate_handleUpdateResponse(const fwUpdate::payload_t &msg) {
    if ((session_id == msg.data.update_resp.session_id) && (session_status == msg.data.update_resp.status) && (session_status != fwUpdate::INITIALIZING))
        return true; // we're receiving duplicate messages, so ignore them

    session_status = msg.data.update_resp.status;
    session_total_chunks = msg.data.update_resp.totl_chunks;

    switch (session_status) {
        case fwUpdate::ERR_MAX_CHUNK_SIZE:    // indicates that the maximum chunk size requested in the original upload request is too large.  The host is expected to begin a new session with a smaller chunk size.
            return fwUpdate_requestUpdate(session_target, session_image_slot, session_image_flags, session_chunk_size / 2, session_image_size, md5hash);
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
            return false;
        }
    } else {
        resent_chunkid_count = 0;
        resent_chunkid_time = current_ms;
    }

    printf("Requesting resend of %d: %d\n", msg.data.req_resend.chunk_id, msg.data.req_resend.reason);
    nextChunkSend = current_timeMs() + nextChunkDelay;
    return fwUpdate_sendNextChunk(); // we don't have to send this right away, but sure, why not!
}

bool ISFirmwareUpdater::fwUpdate_handleUpdateProgress(const fwUpdate::payload_t &msg) {
    session_status = msg.data.progress.status;
    int num = msg.data.progress.num_chunks;
    int tot = msg.data.progress.totl_chunks;
    float percent = msg.data.progress.num_chunks/(float)(msg.data.progress.totl_chunks)*100.f;
    const char *message = (const char *)&msg.data.progress.message;

    if(pfnUploadProgress_cb != nullptr)
        pfnUploadProgress_cb(this, percent);

    if(pfnVerifyProgress_cb != nullptr)
        pfnVerifyProgress_cb(this, percent);

    if(pfnInfoProgress_cb != nullptr)
        pfnInfoProgress_cb(this, ISBootloader::IS_LOG_LEVEL_INFO, message);

    // FIXME: We really want this to call back into the InertialSense class, with some kind of a status callback mechanism; or it should be a callback provided by the original caller
    printf("[%5.2f] [%s:%d > %s] :: Progress %d/%d (%0.1f%%) [%s] :: [%d] %s\n", current_timeMs() / 1000.0f, portName, devInfo->serialNumber, fwUpdate_getSessionTargetName(), num, tot, percent, fwUpdate_getSessionStatusName(), msg.data.progress.msg_level, message);
    return true;
}

bool ISFirmwareUpdater::fwUpdate_handleDone(const fwUpdate::payload_t &msg) {
    session_status = msg.data.resp_done.status;
    printf("[%5.2f] [%s:%d > %s] :: Update Finished:%s\n", current_timeMs() / 1000.0f, portName, devInfo->serialNumber, fwUpdate_getSessionTargetName(), fwUpdate_getSessionStatusName());
    return true;
}

bool ISFirmwareUpdater::fwUpdate_isDone()
{
    return !(hasPendingCommands() || requestPending || ((fwUpdate_getSessionStatus() > fwUpdate::NOT_STARTED) && (fwUpdate_getSessionStatus() < fwUpdate::FINISHED)));
}

fwUpdate::msg_types_e ISFirmwareUpdater::fwUpdate_step() 
{
    uint32_t lastMsgAge = 0;

    switch(session_status) {
        case fwUpdate::NOT_STARTED:
            if (!requestPending) {
                if (!commands.empty()) {
                    auto cmd = commands[0];
                    commands.erase(commands.begin()); // pop the command off the front
                    runCommand(cmd);
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
                        // otherwise, check if its time to send the next update
                        if (nextStartAttempt < current_timeMs()) {// time has elapsed, so re-issue request to update
                            nextStartAttempt = current_timeMs() + attemptInterval;
                            if (fwUpdate_requestUpdate()) {
                                startAttempts++;
                                printf("[%s : %d] :: Requesting Firmware Update Start (Attempt %d)\n", portName, devInfo->serialNumber, startAttempts);
                            } else {
                                printf("Error attempting to initiate Firmware Update\n");
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
            printf("Firmware uploaded in %0.1f seconds: %s\n", (current_timeMs() - updateStartTime) / 1000.f, fwUpdate_getSessionStatusName());
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
            printf("Firmware Update Error: No Response from device [%s : %d].\n", portName, devInfo->serialNumber);
            clearAllCommands();
            break;
        default:
            if (session_status < fwUpdate::NOT_STARTED)
                clearAllCommands();
            printf("Unexpected Response: %s\n", fwUpdate_getSessionStatusName());
            break;
    }

    if ((session_status > fwUpdate::NOT_STARTED) && (session_status < fwUpdate::FINISHED) && (fwUpdate_getLastMessageAge() > timeout_duration))
        session_status = fwUpdate::ERR_TIMEOUT;

    return fwUpdate::MSG_UNKNOWN;
}

bool ISFirmwareUpdater::fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t *buffer, int buff_len) {
    if ((dfuUpdater != nullptr) && ((target & fwUpdate::TARGET_DFU_FLAG) == fwUpdate::TARGET_DFU_FLAG)) {
        return dfuUpdater->fwUpdate_processMessage(buffer, buff_len);
    }

    nextChunkSend = current_timeMs() + chunkDelay; // give *at_least* enough time for the send buffer to actually transmit before we send the next message
    int result = comManagerSendData(pHandle, buffer, DID_FIRMWARE_UPDATE, buff_len, 0);
    return (result == 0);
}

void ISFirmwareUpdater::runCommand(std::string cmd) {
    std::vector<std::string> args;
    splitString(cmd, '=', args);
    if (!args.empty()) {
        if ((args[0] == "package") && (args.size() == 2)) {
            bool isManifest = (args[1].length() >= 5) && (0 == args[1].compare (args[1].length() - 5, 5, ".yaml"));
            if (isManifest) {
                int err_result = 0;
                if ((err_result = processPackageManifest(args[1])) != 0) {
                    printf("Error processing Firmware Package :: %d\n", err_result);
                    commands.clear();
                    target = fwUpdate::TARGET_HOST;
                }
            } else
                openFirmwarePackage(args[1]);
        } else if ((args[0] == "target") && (args.size() == 2)) {
            if (args[1] == "IMX5") target = fwUpdate::TARGET_IMX5;
            else if (args[1] == "GPX1") target = fwUpdate::TARGET_GPX1;
            else if (args[1] == "GNSS1") target = fwUpdate::TARGET_SONY_CXD5610__1;
            else if (args[1] == "GNSS2") target = fwUpdate::TARGET_SONY_CXD5610__2;
            else {
                printf("Firmware Update Error :: Invalid Target: %s    (Valid targets are: IMX5, GPX1, GNSS1, GNSS2)\n", args[1].c_str());
                commands.clear();
                target = fwUpdate::TARGET_HOST;
            }
        } else if ((args[0] == "slot") && (args.size() == 2)) {
            slotNum = strtol(args[1].c_str(), nullptr, 10);
        } else if ((args[0] == "timeout") && (args.size() == 2)) {
            fwUpdate_setTimeoutDuration(strtol(args[1].c_str(), nullptr, 10));
        } else if (args[0] == "force") {
            forceUpdate = (args[1] == "true" ? true : false);
        } else if (args[0] == "reset") {
            bool hard = (args.size() == 2 && args[1] == "hard");
            session_target = target; // kinda janky (we should pass the target into this call)
            fwUpdate_requestReset(hard ? fwUpdate::RESET_HARD : fwUpdate::RESET_SOFT);
        } else if ((args[0] == "chunk") && (args.size() == 2)) {
            chunkSize = strtol(args[1].c_str(), nullptr, 10);
        } else if ((args[0] == "rate") && (args.size() == 2)) {
            progressRate = strtol(args[1].c_str(), nullptr, 10);
        } else if ((args[0] == "upload") && (args.size() == 2)) {
            filename = args[1];

            // check if any flags should be set
            uint8_t flags = 0;

            // TODO move this to it's own function before we expand this any father
            // check for non encrypted file CXD update slot 4 or slot 2 if .fpk
            if((target == fwUpdate::TARGET_SONY_CXD5610__1 || target == fwUpdate::TARGET_SONY_CXD5610__2) && (slotNum == 4 || (slotNum == 2 && filename.substr(filename.find_last_of(".") + 1) == "fpk")))
                flags |= fwUpdate::image_flags_imageNotEncrypted;

            fwUpdate::update_status_e status = initializeUpdate(target, filename, slotNum, flags, forceUpdate, chunkSize, progressRate);

            if (status < fwUpdate::NOT_STARTED) {
                // there was an error -- probably should flush the command queue
                printf("Error initiating Firmware upload: [%s] %s\n", filename.c_str(), fwUpdate_getStatusName(status));
                commands.clear();
            } else {
                requestPending = true;
                nextStartAttempt = current_timeMs() + attemptInterval;
                // session_status = fwUpdate::NOT_STARTED;
            }
        }
    }
}

int ISFirmwareUpdater::processPackageManifest(YAML::Node& manifest, mz_zip_archive* archive = nullptr) {
    YAML::Node images = manifest["images"];
    if (!images.IsMap())
        return -1; // images must be a map (of maps)

    YAML::Node steps = manifest["steps"];

    if (!steps.IsSequence())
        return -2; // steps must be a sequence (of maps)

    for (auto steps_iv : steps) {
        // each step in the list of steps defines a target, which is then also a sequence of commands
        for (auto target : steps_iv) {
            YAML::Node key = target.first;
            YAML::Node actions = target.second;

            if (!key.IsScalar())
                return -3; // key must identify a target name
            std::string target_name = key.as<std::string>();
            if ((target_name != "IMX5") &&
                (target_name != "GPX1") &&
                (target_name != "GNSS1") &&
                (target_name != "GNSS2") )
                return -4; // target name is invalid/unsupported

            if (!actions.IsSequence())
                return -5; // actions must be a sequence (of maps)

            commands.push_back("target=" + target_name);
            for (auto actions_iv : actions) {
                for (auto cmd : actions_iv) {
                    auto cmd_name = cmd.first.as<std::string>();
                    auto cmd_arg = cmd.second.as<std::string>();

                    if (cmd_name == "image") {
                        int image_slot = 0;
                        uint32_t image_size = 0;
                        uint32_t image_hash[4] = {};
                        // uint8_t image_version[4] = {};

                        size_t file_size = 0;
                        uint32_t file_hash[4] = {};

                        // this is an "image" command, so lookup the key in the images map
                        YAML::Node image = images[cmd_arg];
                        if (!image.IsDefined() || !image.IsMap())
                            return -6; // step references invalid or non-existent image in manifest

                        if (!image["filename"].IsDefined() && !image["filename"].IsScalar())
                            return -7; // an image must define AT LEAST a filename

                        std::string filename = image["filename"].as<std::string>();
                        if (archive) {
                            uint32_t f_index = 0;
                            mz_zip_archive_file_stat file_stat;
                            if (mz_zip_reader_locate_file_v2(archive, filename.c_str(), nullptr, 0, &f_index)) {
                                mz_zip_reader_file_stat(archive, f_index, &file_stat);
                                file_size = file_stat.m_uncomp_size;
                            } else
                                return -8;
                        } else {
                            std::ifstream stream(filename);
                            if (md5_file_details(stream, file_size, file_hash) != 0)
                                return -8; // file is invalid or non-existent
                        }

                        // the following are optional parameters; including them in the manifest image forces us to validate that parameter BEFORE allowing the image to be send to the device
                        if (image["size"].IsDefined() && image["size"].IsScalar()) {
                            image_size = image["size"].as<int>();

                            if (image_size != file_size)
                                return -9; // file size doesn't match the manifest image size
                        }

                        // We only do MD5 validation on non-zipped files...
                        // TODO: we should do this eventually, but its costly, since we end up decoding multiple times
                        if (!archive) {
                            if (image["md5sum"].IsDefined() && image["md5sum"].IsScalar()) {
                                std::string hash_str = image["md5sum"].as<std::string>();
                                sscanf(hash_str.c_str(), "%08x%08x%08x%08x", &image_hash[0], &image_hash[1], &image_hash[2], &image_hash[3]);

                                if (memcmp(image_hash, file_hash, sizeof(image_hash)) != 0)
                                    return -10; // file hash doesn't make the manifest image hash
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

    return 0;
}

int ISFirmwareUpdater::processPackageManifest(const std::string& manifest_file) {
    YAML::Node manifest = YAML::LoadFile("manifest.yaml");
    return processPackageManifest(manifest);
}

int ISFirmwareUpdater::openFirmwarePackage(const std::string& pkg_file) {
    mz_bool status;
    mz_zip_archive zip_archive;
    size_t file_size;
    void *p;

    // initialize the archive struct and open the file
    mz_zip_zero_struct(&zip_archive);
    status = mz_zip_reader_init_file(&zip_archive, pkg_file.c_str(), 0);
    if (status) {
        p = mz_zip_reader_extract_file_to_heap(&zip_archive, "manifest.yaml", &file_size, 0);
        if (p && (file_size > 0)) {
            std::string casted_memory(static_cast<char*>(p), file_size);
            std::istringstream stream(casted_memory);
            YAML::Node manifest = YAML::Load(stream);
            if (manifest.IsMap())
                processPackageManifest(manifest, &zip_archive);
            mz_free(p);
        }
        mz_zip_reader_end(&zip_archive);
    }
    return 0;
}

int ISFirmwareUpdater::cleanupFirmwarePackage() { return 0; }
