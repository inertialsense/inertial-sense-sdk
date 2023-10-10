//
// Created by kylemallory on 8/18/23.
//

#include "ISFirmwareUpdater.h"
#include "ISUtilities.h"

bool ISFirmwareUpdater::initializeUpdate(fwUpdate::target_t _target, const std::string &filename, int slot, bool forceUpdate, int chunkSize, int progressRate) 
{
    srand(time(NULL)); // get *some kind* of seed.

    srcFile = new std::ifstream(filename, std::ios::binary);

    // get the file size, and checksum for the file
    srcFile->seekg(0, srcFile->end); // move to the end
    size_t fileSize = srcFile->tellg(); // get the position
    srcFile->seekg(0, srcFile->beg); // move back to the start

    // calculate the md5 checksum
    resetMd5();
    
    uint8_t buff[512];
    size_t curPos = srcFile->tellg();

    while ( curPos < fileSize) 
    {
        int len = _MIN(fileSize - curPos, sizeof(buff)); // are we doing a full block, or partial block
        srcFile->seekg(curPos);
        srcFile->read((char *)buff, len);

        hashMd5(len, buff);
        curPos = srcFile->tellg();
    }

    getCurrentMd5(session_md5);
    printf("Image '%s', md5: %8x%8x%8x%8x\n", filename.c_str(), session_md5[0], session_md5[1], session_md5[2], session_md5[3]);


    // TODO: We need to validate that this firmware file is the correct file for this target, and that its an actual update (unless 'forceUpdate' is true)

    setTimeoutDuration(15000);
    updateStartTime = current_timeMs();
    nextStartAttempt = current_timeMs() + attemptInterval;
    return requestUpdate(_target, slot, chunkSize, fileSize, session_md5, progressRate);
}

int ISFirmwareUpdater::getImageChunk(uint32_t offset, uint32_t len, void **buffer) {
    if (srcFile && srcFile->is_open()) {
        srcFile->seekg((std::streampos)offset, srcFile->beg);
        len = _MIN(len, session_image_size - (uint32_t)srcFile->tellg());
        srcFile->read((char *)*buffer, len);
    }
    return len;
}

bool ISFirmwareUpdater::handleUpdateResponse(const fwUpdate::payload_t &msg) {
    if ((session_id == msg.data.update_resp.session_id) && (session_status == msg.data.update_resp.status))
        return true; // we're receiving duplicate messages, so ignore them

    session_status = msg.data.update_resp.status;
    session_total_chunks = msg.data.update_resp.totl_chunks;

    switch (session_status) {
        case fwUpdate::ERR_MAX_CHUNK_SIZE:    // indicates that the maximum chunk size requested in the original upload request is too large.  The host is expected to begin a new session with a smaller chunk size.
            return requestUpdate(target_id, session_image_slot, session_chunk_size / 2, session_image_size, md5hash);
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

bool ISFirmwareUpdater::handleResendChunk(const fwUpdate::payload_t &msg) {
    // TODO: LOG msg.data.req_resend.reason
    nextChunkSend = current_timeMs() + nextChunkDelay;
    return sendNextChunk(); // we don't have to send this right away, but sure, why not!
}

bool ISFirmwareUpdater::handleUpdateProgress(const fwUpdate::payload_t &msg) {
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
        pfnInfoProgress_cb(this, message, ISBootloader::IS_LOG_LEVEL_INFO);
    

    // FIXME: We really want this to call back into the InertialSense class, with some kind of a status callback mechanism; or it should be a callback provided by the original caller
    printf("[%s : %d] :: Progress %d/%d (%0.1f%%) [%s] :: [%d] %s\n", portName, devInfo->serialNumber, num, tot, percent, getSessionStatusName(), msg.data.progress.msg_level, message);
    return true;
}

fwUpdate::msg_types_e ISFirmwareUpdater::step() {
    switch(session_status) {
        case fwUpdate::NOT_STARTED:
            // nothing to do..
            break;
        case fwUpdate::INITIALIZING:
            if (startAttempts < maxAttempts) {
                if (nextStartAttempt < current_timeMs()) {// time has elapsed, so re-issue request to update
                    nextStartAttempt = current_timeMs() + attemptInterval;
                    if (requestUpdate()) {
                        startAttempts++;
                        printf("[%s : %d] :: Requesting Firmware Start (Attempt %d)\n", portName, devInfo->serialNumber, startAttempts);
                    } else {
                        session_status = fwUpdate::ERR_COMMS; // error sending the request
                    }
                }
            } else {
                // backoff (wait attemptInternal * 3), and then try again.  Eventually, we will timeout below if there is a larger issue.
                startAttempts = 0;
                nextStartAttempt = current_timeMs() + (attemptInterval * 3);
            }
            break;
        case fwUpdate::READY:
        case fwUpdate::IN_PROGRESS:
            if (nextChunkSend < current_timeMs()) // don't send chunks too fast
                sendNextChunk();
            break;
        case fwUpdate::FINISHED:
            printf("Firmware uploaded in %0.1f seconds: %s\n", (current_timeMs() - updateStartTime) / 1000.f, getSessionStatusName());
            break;
        case fwUpdate::ERR_MAX_CHUNK_SIZE:
            if (session_id != 0) {
                // we need to get a new session, and a new chunk size, but let's keep everything else the same
                // let's try and stick to factors of 2 (ie, 64, 128, 256, 512, 1024, etc) even if we started with some number in-between (384)
                double lv = log2(session_chunk_size);
                int bits = (lv == floor(lv)) ? (int)(lv-1) : (int)(lv); // round down to the nearest multiple of 2
                session_chunk_size = 1 << bits;
                session_id = (uint16_t) rand(); // since we ended on an error, we need a new session id.
                requestUpdate();
            }
            break;
        default:
            printf("Firmware uploaded in %0.1f seconds: %s\n", (current_timeMs() - updateStartTime) / 1000.f, getSessionStatusName());
            break;
    }

    if ((session_status > fwUpdate::NOT_STARTED) && (session_status < fwUpdate::FINISHED) && (getLastMessageAge() > timeout_duration))
        session_status = fwUpdate::ERR_TIMEOUT;

    return fwUpdate::MSG_UNKNOWN;
}

bool ISFirmwareUpdater::writeToWire(fwUpdate::target_t target, uint8_t *buffer, int buff_len) {
    nextChunkSend = current_timeMs() + 15; // give *at_least* enough time for the send buffer to actually transmit before we send the next message
    int result = comManagerSendData(pHandle, buffer, DID_FIRMWARE_UPDATE, buff_len, 0);
    return (result == 0);
}

