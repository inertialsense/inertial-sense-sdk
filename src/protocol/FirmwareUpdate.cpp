//
// Created by kylemallory on 7/26/23.
//

#include "FirmwareUpdate.h"

#ifdef __ZEPHYR__
#include <zephyr/random/rand32.h>
LOG_MODULE_REGISTER(FirmwareUpdater, LOG_LEVEL_DBG);
#endif

namespace fwUpdate {

#ifdef DEBUG_LOGGING
    static const char* type_names[] = { "UNKNOWN", "REQ_RESET", "RESET_RESP", "REQ_UPDATE", "UPDATE_RESP", "UPDATE_CHUNK", "UPDATE_PROGRESS", "REQ_RESEND_CHUNK", "UPDATE_DONE", "REQ_VERSION", "VERSION_RESP"};
#endif
    struct status_strings_t {
        const char* name;
        const char* nice;
    };

    static const status_strings_t status_names [] = {
            { .name = "ERR_UNKNOWN", .nice = "Unknown Error" },
            { .name = "ERR_INVALID_TARGET", .nice = "Invalid Target" },
            { .name = "ERR_INVALID_CHUNK", .nice = "Invalid or Out of Sequence Chunk" },
            { .name = "ERR_INVALID_IMAGE", .nice = "Invalid Image" },
            { .name = "ERR_UPDATER_CLOSED", .nice = "Updater Closed" },
            { .name = "ERR_FLASH_INVALID", .nice = "Flash Invalid" },
            { .name = "ERR_FLASH_OPEN_FAILURE", .nice = "Flash Open Failure" },
            { .name = "ERR_FLASH_WRITE_FAILURE", .nice = "Flash Write Failure" },
            { .name = "ERR_NOT_SUPPORTED", .nice = "Request or Feature Not Supported" },
            { .name = "ERR_COMMS", .nice = "Communications Error" },
            { .name = "ERR_CHECKSUM_MISMATCH", .nice = "Checksum Mismatch" },
            { .name = "ERR_TIMEOUT", .nice = "Communications Timeout" },
            { .name = "ERR_MAX_CHUNK_SIZE", .nice = "Max Chunk Size Exceeded" },
            { .name = "ERR_OLDER_FIRMWARE", .nice = "Older Firmware - Downgrade not allowed" },
            { .name = "ERR_NOT_ENOUGH_MEMORY", .nice = "Not Enough Memory" },
            { .name = "ERR_NOT_ALLOWED", .nice = "Operation Not Allowed" },
            { .name = "ERR_INVALID_SLOT", .nice = "Invalid Device Slot" },
            { .name = "ERR_INVALID_SESSION", .nice = "Invalid Session" },
            { .name = "NOT_STARTED", .nice = "Not Started" },
            { .name = "INITIALIZING", .nice = "Initializing" },
            { .name = "READY", .nice = "Ready" },
            { .name = "IN_PROGRESS", .nice = "In Progress" },
            { .name = "FINALIZING", .nice = "Finalizing" },
            { .name = "FINISHED", .nice = "Finished" },
    };

#ifdef DEBUG_LOGGING
    static const char* reason_names[] = { "NONE", "INVALID_SEQID", "WRITE_ERROR", "INVALID_SIZE" };
#endif

    /*==================================================================================*
     * Firmware Base Implementation goes here                                           *
     *==================================================================================*/

    FirmwareUpdateBase::FirmwareUpdateBase() { };

    size_t FirmwareUpdateBase::fwUpdate_getPayloadSize(const payload_t* payload, bool include_aux) {
        switch (payload->hdr.msg_type) {
            case MSG_REQ_RESET:
                return sizeof(payload->hdr) + sizeof(payload->data.req_reset);
            case MSG_RESET_RESP:
                return sizeof(payload->hdr) + sizeof(payload->data.rpl_reset);
            case MSG_REQ_UPDATE:
                return sizeof(payload->hdr) + sizeof(payload->data.req_update);
            case MSG_UPDATE_RESP:
                return sizeof(payload->hdr) + sizeof(payload->data.update_resp);
            case MSG_UPDATE_CHUNK:
                return sizeof(payload->hdr) + sizeof(payload->data.chunk) + (include_aux ? payload->data.chunk.data_len - 1 : -1);
            case MSG_REQ_RESEND_CHUNK:
                return sizeof(payload->hdr) + sizeof(payload->data.req_resend);
            case MSG_UPDATE_PROGRESS:
                return sizeof(payload->hdr) + sizeof(payload->data.progress) + (include_aux ? payload->data.progress.msg_len - 1 : -1);
            case MSG_UPDATE_DONE:
                return sizeof(payload->hdr) + sizeof(payload->data.resp_done);
            case MSG_REQ_VERSION_INFO:
                return sizeof(payload->hdr) + sizeof(payload->data.req_version);
            case MSG_VERSION_INFO_RESP:
                return sizeof(payload->hdr) + sizeof(payload->data.version_resp);
            default:
                break;
        }
        return 0;
    }


    /**
     * Packs a byte buffer that can be sent out onto the wire, using data from a passed msg_payload_t.
     * Note that this results in at least one copy, and possibly multiple assignments. Where possible, you
     * should opt to cast the payload directly into a uint8_t pointer and use directly. This is not always
     * possible (particularly with strings/chunk data).
     * @param msg_payload
     * @param buffer
     * @param max_len
     * @return the number of bytes of the resulting buffer, after packing, or -1 if buffer is not large enough
     */
    int FirmwareUpdateBase::fwUpdate_packPayload(uint8_t* buffer, int max_len, const payload_t& payload, const void *aux_data) {
        int payload_size = (int) fwUpdate_getPayloadSize(&payload, false);
        if (payload_size > max_len) return -1; // Not enough buffer space
        if (payload_size == 0) return -2; // Unknown/invalid message

        memcpy(buffer, (void *) &payload, payload_size);

        int aux_len = 0;
        if (payload.hdr.msg_type == MSG_UPDATE_CHUNK)
            aux_len = payload.data.chunk.data_len;
        else if (payload.hdr.msg_type == MSG_UPDATE_PROGRESS)
            aux_len = payload.data.progress.msg_len;

        if ((aux_len > 0) && (aux_data != nullptr)) {
            memcpy( ((uint8_t*)buffer) + payload_size, (uint8_t*)aux_data, aux_len);
        }

        return payload_size + aux_len;
    }

    /**
     * Unpacks a DID payload byte buffer (from the comms system) into a firmware_update msg_payload_t struct
     * Note that this results in at least one copy, and possibly multiple assignments. Where possible, you
     * should opt to cast the pointer into a msg_payload_t, and use directly, but that isn't always possible.
     * @param buffer a pointer to the start of the byte buffer containing the raw data
     * @param buf_len the number of bytes the unpack from the byte buffer
     * @param msg_payload the payload_t struct that the data will be unpacked into.
     * @return true on success, otherwise false
     */
    int FirmwareUpdateBase::fwUpdate_unpackPayload(const uint8_t* buffer, int buf_len, payload_t& payload, void *aux_data, uint16_t max_aux) {
        int payload_size = fwUpdate_getPayloadSize((payload_t *) buffer);
        int aux_len = 0;
        void* aux_ptr = nullptr;

        if (((payload_t*)buffer)->hdr.msg_type == MSG_UPDATE_CHUNK) {
            aux_len = ((payload_t*)buffer)->data.chunk.data_len;
            aux_ptr = &(((payload_t*)buffer)->data.chunk.data);
        }
        else if (((payload_t*)buffer)->hdr.msg_type == MSG_UPDATE_PROGRESS) {
            aux_len = ((payload_t*)buffer)->data.progress.msg_len;
            aux_ptr = &(((payload_t*)buffer)->data.progress.message);
        }

        if (buf_len < payload_size + aux_len) return -1; // the passed buffer is incomplete (smaller than the unpacked message size)

        memcpy((void *)(&payload), buffer, payload_size);
        if ((aux_len <= max_aux) && (aux_data != nullptr)) {
            memcpy( aux_data, aux_ptr, aux_len);
        }

        return payload_size + aux_len;
    }

    /**
     * Maps a DID payload byte buffer (from the comms system) into a fwUpdate::payload_t struct, and extracts aux_data if any.
     * @param buffer a pointer to the raw byte buffer
     * @param msg_payload a double-pointer which on return will point to the start of the buffer (this is a simple cast)
     * @param aux_data a double-pointer which on return will point to any auxilary data in the payload, or nullptr if there is none
     * @return returns the total number of bytes in the packet, including aux data if any
     */
    int FirmwareUpdateBase::fwUpdate_mapBufferToPayload(const uint8_t *buffer, payload_t** payload, void** aux_data) {
        // TODO: Remove after 2.1.0 release
        // >=2.0.0.11  versions, the type was changes from a uint16_t to a uint32_t (but had the same packing)
        // this would result in later version not interpreting the msg_type correctly (having non-zero high-order bits).
        uint32_t msg_type = (int)(((payload_t*)buffer)->hdr.msg_type) & 0xFFFF;
        ((payload_t*)buffer)->hdr.msg_type =  (msg_types_e)msg_type;
        // TODO: End

        int payload_size = fwUpdate_getPayloadSize((payload_t *) buffer);
        int aux_len = 0;

        if (((payload_t*)buffer)->hdr.msg_type == MSG_UPDATE_CHUNK) {
            aux_len = ((payload_t*)buffer)->data.chunk.data_len;
            *aux_data = &(((payload_t*)buffer)->data.chunk.data);
        }
        else if (((payload_t*)buffer)->hdr.msg_type == MSG_UPDATE_PROGRESS) {
            aux_len = ((payload_t*)buffer)->data.progress.msg_len;
            *aux_data = &(((payload_t*)buffer)->data.progress.message);
        }

        *payload = (payload_t*)buffer;
        return payload_size + aux_len;
    }

    /**
     * packages and sends the specified payload, including any auxillary data.
     * Note that the payload must already specify the amount of aux data the be included.
     * @param payload
     * @param aux_data the auxillary data to include, or nullptr if none.
     * @return
     */
    bool FirmwareUpdateBase::fwUpdate_sendPayload(fwUpdate::payload_t& payload, void *aux_data) {
        int payload_len = fwUpdate_packPayload(build_buffer, FWUPDATE__MAX_PAYLOAD_SIZE, payload, aux_data);
        #ifdef DEBUG_LOGGING
        LOG_DBG("Sending to %s", fwUpdate_payloadToString(&payload));
        #endif

        return fwUpdate_writeToWire((fwUpdate::target_t) payload.hdr.target_device, build_buffer, payload_len);
    }

    char* FirmwareUpdateBase::fwUpdate_payloadToString(const payload_t* payload) {
        #ifdef DEBUG_LOGGING
        static char tmp[256];
        int cur_len = 0;

        cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, "%s : %s ", fwUpdate_getTargetName(payload->hdr.target_device), type_names[payload->hdr.msg_type]);
        switch (payload->hdr.msg_type) {
            case MSG_REQ_UPDATE:
                cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, "[session=%d, slot=%d, image_size=%u, chunk_size=%u, flags=%04X, progress_rate=%ums, md5=%s]",
                                    payload->data.req_update.session_id,
                                    payload->data.req_update.image_slot,
                                    (unsigned int)payload->data.req_update.file_size,
                                    payload->data.req_update.chunk_size,
                                    payload->data.req_update.image_flags,
                                    payload->data.req_update.progress_rate,
                                    md5_to_string(payload->data.req_update.md5_hash).c_str());
                break;
            case MSG_UPDATE_RESP:
                cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, "[session=%d, status='%s', chunks=%d]",
                                    payload->data.update_resp.session_id, fwUpdate_getStatusName(payload->data.update_resp.status), payload->data.update_resp.totl_chunks);
                break;
            case MSG_UPDATE_CHUNK:
                cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, "[session=%d, chunk=%d, len=%d]",
                                    payload->data.chunk.session_id, payload->data.chunk.chunk_id, payload->data.chunk.data_len);
                break;
            case MSG_REQ_RESEND_CHUNK:
                cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, "[session=%d, chunk=%d, reason='%s']",
                                    payload->data.req_resend.session_id, payload->data.req_resend.chunk_id, reason_names[payload->data.req_resend.reason]);
                break;
            case MSG_UPDATE_PROGRESS:
                cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, "[session=%d, status='%s', total=%d, chunks=%d]",
                                    payload->data.progress.session_id, fwUpdate_getStatusName(payload->data.progress.status), payload->data.progress.totl_chunks, payload->data.progress.num_chunks);
                if (payload->data.progress.msg_len > 0)
                    cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, " %s", &payload->data.progress.message);
                break;
            case MSG_UPDATE_DONE:
                cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, "[session=%d, status='%s']",
                                    payload->data.resp_done.session_id, fwUpdate_getStatusName(payload->data.resp_done.status));
                break;
            case MSG_VERSION_INFO_RESP:
                cur_len += snprintf(tmp + cur_len, sizeof(tmp) - cur_len, "[%s : %d.%d.%d.%d (%d), %04d-%02d-%02d]",
                                    fwUpdate_getTargetName(payload->data.version_resp.resTarget), payload->data.version_resp.firmwareVer[0],
                                    payload->data.version_resp.firmwareVer[1], payload->data.version_resp.firmwareVer[2], payload->data.version_resp.firmwareVer[3], payload->data.version_resp.buildNumber,
                                    2000 + payload->data.version_resp.buildYear, payload->data.version_resp.buildMonth, payload->data.version_resp.buildDay);
                break;
            default:
                break;
        }
        if (cur_len) { }
        return tmp;
        #else
        return nullptr;
        #endif
    }

    /**
     * @return a human-readable status name for the current session status
     */
    const char *FirmwareUpdateBase::fwUpdate_getStatusName(update_status_e status) {
        if (status >= 0)
            return fwUpdate::status_names[status + abs(fwUpdate::ERR_UNKNOWN)].name;
        else
            return fwUpdate::status_names[status - (fwUpdate::ERR_UNKNOWN)].name;
    }

    /**
     * Returns a human-friendly string describing the update status, used for UIs
     * @param status
     * @return a constant char * to a string representing the specified status
     */
    const char *FirmwareUpdateBase::fwUpdate_getNiceStatusName(update_status_e status) {
        if (status >= 0)
            return fwUpdate::status_names[status + abs(fwUpdate::ERR_UNKNOWN)].nice;
        else
            return fwUpdate::status_names[status - (fwUpdate::ERR_UNKNOWN)].nice;
    }

    /**
     * @return a human-readable status name for the current session target
     */
    const char *FirmwareUpdateBase::fwUpdate_getTargetName(target_t target) {
        target_t target_masked = (target_t)((uint32_t)target & 0xFFFF0);
        switch (target_masked) {
            case TARGET_HOST: return "HOST";
            case TARGET_IMX5: return "IMX-5";
            case TARGET_GPX1: return "GPX-1";
            case TARGET_VPX: return "VPX-1";
            case TARGET_UBLOX_F9P: return "uBlox F9P";
            case TARGET_SONY_CXD5610:
                switch (target & 0x0F) {
                    case 1: return "CXD5610.1";
                    case 2: return "CXD5610.2";
                    default: return "CXD5610";
                }
                break;
            default: return "[UNKNOWN]";
        }
    }

    /*==================================================================================*
     * Firmware Device base implementation goes here                                           *
     *==================================================================================*/

    /**
     * Creates a FirmwareUpdateDevice instance
     * @param target_id the target_id which this message will respond to.
     * @param startUpdate
     * @param writeChunk
     * @param resetMcu
     * @param progress_millis the rate at which progress updates will be sent out, in milli-seconds (default is every 100ms, 0 = no updates are sent).
     */
    // FirmwareUpdateDevice::FirmwareUpdateDevice(uint16_t tid, start_update_fn startUpdate, write_chunk_fn writeChunk, reset_mcu_fn resetMcu, uint16_t progress_millis) {
    FirmwareUpdateDevice::FirmwareUpdateDevice(target_t tid) {
        session_target = tid;
        fwUpdate_resetEngine();
    }


    /**
     * Called by the communications system anytime a DID_FIRMWARE_UPDATE is received.
     * @param msg_payload the contents of the DID_FIRMWARE_UPDATE payload
     * @return true if this message was consumed by this interface, or false if the message was not intended for us, and should be passed along to other ports/interfaces.
     *
     * Note: Internally, this method calls fwUpdate_step(), so even if you don't call fwUpdate_step(), it can still operate with just inbound messages, but interval updates/etc won't run.
     */
    bool FirmwareUpdateDevice::fwUpdate_processMessage(const payload_t& payload) {
        bool result = false;

        //target_t target_masked = (target_t)((uint32_t)msg_payload.hdr.target_device & 0xFFFF0);
        target_t target_masked = payload.hdr.target_device;
        if (target_masked != session_target)
            return false; // if this message isn't for us, then we return false which will forward this message on to other connected devices

        #ifdef DEBUG_LOGGING
        LOG_DBG("Received by %s", fwUpdate_payloadToString(&payload));
        #endif

        fwUpdate_resetTimeout();
        switch (payload.hdr.msg_type) {
            case MSG_REQ_UPDATE:
                result = fwUpdate_handleInitialize(payload);
                break;
            case MSG_UPDATE_CHUNK:
                result = fwUpdate_handleChunk(payload);
                break;
            case MSG_REQ_RESET:
                result = fwUpdate_handleReset(payload);
                break;
            case MSG_REQ_VERSION_INFO:
                result = fwUpdate_handleVersionInfo(payload);
                break;
            default:
                result = false;
        }

        if ((progress_interval > 0) && (nextProgressReport < current_timeMs())) {
            nextProgressReport = current_timeMs() + progress_interval;
            fwUpdate_sendProgress();
        }

        fwUpdate_step(payload.hdr.msg_type, result); // TODO: we should probably do something with the step() result, but not sure what just yet
        return result;
    }
    bool FirmwareUpdateDevice::fwUpdate_processMessage(const uint8_t* buffer, int buf_len) {
        fwUpdate::payload_t *payload = nullptr;
        void *aux_data = nullptr;

        int payload_len = fwUpdate_mapBufferToPayload(buffer, &payload, &aux_data);
        if (payload_len <= 0)
            return false;

        return fwUpdate_processMessage(*payload);
    }

    /**
     * This is an internal method used to send an update message to the host system regarding the status of the update process
     * This message only include the number of chunks sent, and the total expected (sufficient for a percentage) and the
     * @return true if the message was sent, false if there was an error
     */
    bool FirmwareUpdateDevice::fwUpdate_sendProgress() {
        nextProgressReport = current_timeMs() + progress_interval;
        return fwUpdate_sendProgressFormatted(99, nullptr);
    }

    /**
     * This is an internal method used to send an update message to the host system regarding the status of the update process
     * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
     * @param message the actual message to be sent to the host
     * @return true if the message was sent, false if there was an error
     */
    bool FirmwareUpdateDevice::fwUpdate_sendProgress(int level, const std::string message) {
        return fwUpdate_sendProgressFormatted(level, message.c_str());
    }

    /**
     * This is an internal method used to send an update message to the host system regarding the status of the update process
     * This variation allows for printf-based string formatting
     * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
     * @param message the actual message to be sent to the host
     * @
     * @return true if the message was sent, false if there was an error
     */
    bool FirmwareUpdateDevice::fwUpdate_sendProgressFormatted(int level, const char* message, ...) {
        static char buffer[256];
        size_t msg_len = 0;

        if ((session_id == 0) || (session_status == NOT_STARTED))
            return false;

        if (message) {
            va_list ap;
            va_start(ap, message);
            msg_len = vsnprintf(buffer, sizeof(buffer) - 1, message, ap);
            va_end(ap);
        } else
            memset(buffer, 0, sizeof(buffer));

        if (msg_len >= sizeof(buffer)-1)
            return false;

        payload_t msg;
        msg.hdr.target_device = TARGET_HOST; // progress messages always go back to the host.
        msg.hdr.msg_type = MSG_UPDATE_PROGRESS;
        msg.data.progress.session_id = session_id;
        msg.data.progress.status = session_status;
        msg.data.progress.totl_chunks = session_total_chunks;
        msg.data.progress.num_chunks = _MAX(0, _MIN(session_total_chunks, last_chunk_id+1));
        msg.data.progress.msg_level = level;
        msg.data.progress.msg_len = msg_len;
        msg.data.progress.message = 0;

        msg_len = fwUpdate_packPayload(build_buffer, sizeof(build_buffer), msg, buffer);
        return fwUpdate_writeToWire((fwUpdate::target_t) msg.hdr.target_device, build_buffer, msg_len);
    }

    /**
     * @return true if we have an active session and are updating.
     */
    bool FirmwareUpdateDevice::fwUpdate_isUpdating() {
        return (session_id != 0) && (session_status > NOT_STARTED);
    }

    /**
     * @brief Notifies the host that the firmware update is complete for any reason, including an error.
     * This call does not generate a response or acknowledgement from the host.
     *
     * @param reason the specific reason the update was finished.
     * @param clear_session if true, causes the current session to be invalidated
     * @param reset_device if true, will call fwUpdate_performReset() after sending the message.
     * @return true if successfully sent, otherwise false.
     */
    bool FirmwareUpdateDevice::fwUpdate_sendDone(update_status_e reason, bool clear_session, bool reset_device) {
        payload_t response;
        response.hdr.target_device = TARGET_HOST;
        response.hdr.msg_type = MSG_UPDATE_DONE;
        response.data.resp_done.session_id = session_id;
        response.data.resp_done.status = session_status = reason;
        fwUpdate_sendPayload(response);

        bool result = true;
        if (clear_session)
            result = fwUpdate_resetEngine();
        if (reset_device)
            result = fwUpdate_performReset(session_target, RESET_SOFT);
        return result;
    }

    /**
     * Internal method use to reinitialize the update engine.  This should clear the the current session_id, existing image data, running md5 sums, etc.
     * After calling this function, the subsystem must receive a REQ_UPDATE message to start to new session, etc.
     * This probably should be called after an update is finished, but is probably safest to call as the first step in a REQ_UPDATE.
     * @return true if the system was able to properly initialize, false if there was an error of something (you have a REAL problem in this case).
     */
    bool FirmwareUpdateDevice::fwUpdate_resetEngine() {
        resend_count = 0;
        session_id = 0;       // the current session id - all received messages with a session_id must match this value.  O == no session set (invalid)
        last_chunk_id = -1;   // the last received chunk id from a CHUNK message.  -1 == no chunk yet received; the next received chunk must be 0.
        session_status = NOT_STARTED;
        md5_init(md5Context);
        return true;
    }

    /**
     * Internally called by fwUpdate_processMessage() when a REQ_UPDATE message is received.
     * @param payload the DID message
     * @return true if the message was received and parsed without error, false otherwise.
     *
     * NOTE, this function should call out and send an error status in the event of a failure.
     */
    bool FirmwareUpdateDevice::fwUpdate_handleInitialize(const payload_t& payload) {
        if (payload.hdr.msg_type != MSG_REQ_UPDATE)
            return false;

        // make sure our session_id is good (TODO: Maybe we should retain a history of recent session ids to make sure we aren't reusing an old one?)
        if (payload.data.req_update.session_id == 0)
            session_status = ERR_INVALID_SESSION;

        fwUpdate_resetEngine();
        session_status = fwUpdate_startUpdate(payload);
        session_id = payload.data.req_update.session_id;
        session_image_size = payload.data.req_update.file_size;
        session_chunk_size = payload.data.req_update.chunk_size;
        // progress_interval = payload.data.req_update.progress_rate;

        if (session_status > NOT_STARTED) {
            session_image_slot = payload.data.req_update.image_slot;
            session_image_flags = payload.data.req_update.image_flags;
            session_total_chunks = (uint16_t) ceil((float)session_image_size / (float)session_chunk_size);
            session_md5 = payload.data.req_update.md5_hash;
        }

        // prepare the response
        payload_t response;
        response.hdr.target_device = TARGET_HOST;
        response.hdr.msg_type = MSG_UPDATE_RESP;
        response.data.update_resp.session_id = session_id;
        response.data.update_resp.totl_chunks = session_total_chunks;
        response.data.update_resp.status = session_status;

        return fwUpdate_sendPayload(response);
    }

    /**
     * Internally called by fwUpdate_processMessage() when a UPDATE_CHUNK message is received.
     * @param payload the DID payload
     * @return
     */
    bool FirmwareUpdateDevice::fwUpdate_handleChunk(const payload_t& payload) {
        if (payload.hdr.msg_type != MSG_UPDATE_CHUNK)
            return false;

        // if the session id doesn't match our current session id, then ignore this message.
        if (payload.data.chunk.session_id != session_id)
            return false;

        // if the chunk id does match the next expected chunk id, then send an resend for the correct/missing chunk
        if (payload.data.chunk.chunk_id != (uint16_t)(last_chunk_id + 1)) {
            fwUpdate_sendRetry(REASON_INVALID_SEQID);
            return false;
        }

        // ensure data_len is the same as session_chunk_size, unless its the very last chunk
        uint16_t mod_size = (session_image_size % session_chunk_size);
        uint16_t expected_size = ((payload.data.chunk.chunk_id == session_total_chunks-1) && (mod_size != 0)) ? mod_size : session_chunk_size;
        if (payload.data.chunk.data_len != expected_size) {
            fwUpdate_sendRetry(REASON_INVALID_SIZE);
            return false;
        }

        uint32_t chnk_offset = payload.data.chunk.chunk_id * session_chunk_size;
        if (fwUpdate_writeImageChunk((fwUpdate::target_t) payload.hdr.target_device, session_image_slot, chnk_offset, payload.data.chunk.data_len, (uint8_t *) &payload.data.chunk.data) < NOT_STARTED) {
            fwUpdate_sendRetry(REASON_WRITE_ERROR);
            return false;
        }

        #ifdef __ZEPHYR__
        //printk("[FwUpdate::%d] Received valid chunk %d of %d\n", session_id, last_chunk_id, session_total_chunks);
        #endif

        // if we're here, all our validations have passed, and we've successfully written our data to flash...
        session_status = IN_PROGRESS;
        last_chunk_id = payload.data.chunk.chunk_id;
        // run the chunk data through the md5 hasher
        md5_update(md5Context, (uint8_t *)&payload.data.chunk.data, payload.data.chunk.data_len);

        // if we've received the last message, confirm the checksum and then send a final status to notify the host that we've received everything error-free.
        if (last_chunk_id >= (session_total_chunks-1)) { // remember, chunk_ids are 0-based
            fwUpdate_sendProgress(); // force sending of a final progress message (which should report 100% complete)

            md5hash_t md5;
            md5_final(md5Context, md5);
            if (memcmp(&session_md5, &md5, sizeof(md5hash_t)) != 0) {
                #ifdef __ZEPHYR__
                printk("Expecting %s md5, but received %s\n", md5_to_string(session_md5).c_str(), md5_to_string(md5).c_str());
                #endif
                session_status = ERR_CHECKSUM_MISMATCH;
            } else {
                // This is the final step for validation; note that fwUpdate_finishUpdate() doesn't implicitly send a response/status to the other end(but the implementing class could).
                session_status = fwUpdate_finishUpdate(session_target, session_image_slot, session_image_flags);
            }

            if (session_status != fwUpdate::FINALIZING) {
                fwUpdate_sendDone(session_status, false, false);
            }
        }

        return true;
    }

    /**
     * Internally called by fwUpdate_processMessage() when a REQ_RESET message is received, to reset the target MCU.
     * @param payload the DID message
     * @return true if the message was received and parsed without error, false otherwise.
     */
    bool FirmwareUpdateDevice::fwUpdate_handleReset(const payload_t& payload) {
        if (payload.hdr.msg_type != MSG_REQ_RESET)
            return false;

        payload_t response;
        response.hdr.target_device = TARGET_HOST;
        response.hdr.msg_type = MSG_RESET_RESP;
        fwUpdate_sendPayload(response); // make sure this goes out before the reset happens. We might need to schedule the reset, so the send can happen, if the underlying call doesn't block.

        return fwUpdate_performReset(session_target, (reset_flags_e)payload.data.req_reset.reset_flags);
    }


    /**
     * Internally called by fwUpdate_processMessage() when a REQ_VERSION_INFO message is received, to request version info for the target device.
     * @param payload the DID message
     * @return true if the message was received and parsed without error, false otherwise.
     */
    bool FirmwareUpdateDevice::fwUpdate_handleVersionInfo(const payload_t& payload) {
        if (payload.hdr.msg_type != MSG_REQ_VERSION_INFO)
            return false;

        dev_info_t devInfo = { };
        if (!fwUpdate_queryVersionInfo(payload.hdr.target_device, devInfo))
            memset(&devInfo, 0xFF, sizeof(dev_info_t));

        payload_t response;
        response.hdr.target_device = TARGET_HOST;
        response.hdr.msg_type = MSG_VERSION_INFO_RESP;
        response.data.version_resp.resTarget = payload.hdr.target_device;
        response.data.version_resp.serialNumber = devInfo.serialNumber;
        response.data.version_resp.hardwareType = devInfo.hardwareType;
        memcpy(&response.data.version_resp.hardwareVer[0], &devInfo.hardwareVer[0], 4);
        memcpy(&response.data.version_resp.firmwareVer[0], &devInfo.firmwareVer[0], 4);
        response.data.version_resp.buildYear = devInfo.buildYear;
        response.data.version_resp.buildMonth = devInfo.buildMonth;
        response.data.version_resp.buildDay = devInfo.buildDay;
        response.data.version_resp.buildHour = devInfo.buildHour;
        response.data.version_resp.buildMinute = devInfo.buildMinute;
        response.data.version_resp.buildSecond = devInfo.buildSecond;
        response.data.version_resp.buildMillis = devInfo.buildMillisecond;
        response.data.version_resp.buildType = devInfo.buildType;
        response.data.version_resp.buildNumber = devInfo.buildNumber;

        return fwUpdate_sendPayload(response);
    }


    /**
     * Sends a REQ_RESEND_CHUNK message in response to receiving an invalid CHUNK message.
     * @param payload the originally received invalid chunk message that cause this RETRY to be sent
     * @return return true is a retry was sent, or false if a retry was not sent.  NOTE this is not an error, as a valid message will not send a retry.
     */
    bool FirmwareUpdateDevice::fwUpdate_sendRetry(resend_reason_e reason) {
        payload_t response;
        response.hdr.target_device = TARGET_HOST;
        response.hdr.msg_type = MSG_REQ_RESEND_CHUNK;
        response.data.req_resend.session_id = session_id;
        response.data.req_resend.chunk_id = last_chunk_id + 1;
        response.data.req_resend.reason = reason;
        //resend_count++;
        return fwUpdate_sendPayload(response);
    }

    /*==================================================================================*
     * HOST-API goes here                                                                *
     *==================================================================================*/

    /**
     * Creates a FirmwareUpdateSDK instance
     */
    FirmwareUpdateHost::FirmwareUpdateHost() { };

    /**
     * Call this any time a DID_FIRMWARE_UPDATE is received by the comms system, to parse and process the message.
     * @param msg_payload the contents of the DID_FIRMWARE_UPDATE payload
     * @return true if this message was consumed by this interface, or false if the message was not intended for us, and should be passed along to other ports/interfaces.
     */
    bool FirmwareUpdateHost::fwUpdate_processMessage(const payload_t& payload) {
        #ifdef DEBUG_LOGGING
            LOG_DBG("Received by %s", fwUpdate_payloadToString(&payload));
        #endif

        if (payload.hdr.target_device != TARGET_HOST)
            return false;

        bool result = false;
        fwUpdate_resetTimeout();
        switch (payload.hdr.msg_type) {
            case MSG_UPDATE_RESP:
                if (payload.data.update_resp.session_id == session_id)
                    result = fwUpdate_handleUpdateResponse(payload);
                break;
            case MSG_UPDATE_PROGRESS:
                if (payload.data.progress.session_id == session_id)
                    result = fwUpdate_handleUpdateProgress(payload);
                break;
            case MSG_REQ_RESEND_CHUNK:
                if (payload.data.req_resend.session_id == session_id) {
                    resend_count += next_chunk_id - payload.data.req_resend.chunk_id;
                    next_chunk_id = payload.data.req_resend.chunk_id;
                    result = fwUpdate_handleResendChunk(payload);
                }
                break;
            case MSG_UPDATE_DONE:
                if (payload.data.resp_done.session_id == session_id) {
                    session_status = payload.data.resp_done.status;
                    result = fwUpdate_handleDone(payload);
                }
                break;
            case MSG_RESET_RESP:
                break;
            case MSG_VERSION_INFO_RESP:
                result = fwUpdate_handleVersionResponse(payload);
                break;
            default:
                break;
        }
        return result;
    }

    bool FirmwareUpdateHost::fwUpdate_processMessage(const uint8_t* buffer, int buf_len) {
        fwUpdate::payload_t *msg = nullptr;
        void *aux_data = nullptr;

        int msg_len = fwUpdate_mapBufferToPayload(buffer, &msg, &aux_data);
        if (msg_len <= 0)
            return false;

        return fwUpdate_processMessage(*msg);
    }

    /**
     * Called by the host application to initiate a request by the SDK to update a target device.
     * @param target_id
     * @param image_id
     * @param image
     * @param chunk_size
     * @return
     */
    bool FirmwareUpdateHost::fwUpdate_requestUpdate(target_t target_id, int image_slot, int image_flags, uint16_t chunk_size, uint32_t image_size, md5hash_t image_md5, int32_t progress_rate) {

        // FIXME:  Should probably check to see if an update is already in progress, and attempt to finish it first?
        fwUpdate_resetEngine();
        fwUpdate_resetTimeout();
        fwUpdate::payload_t request;
#ifdef __ZEPHYR__
        session_id = (uint16_t)sys_rand32_get();
#else
        session_id = (uint16_t)rand();
#endif
        session_status = fwUpdate::NOT_STARTED;

        request.hdr.target_device = session_target = target_id;
        request.hdr.msg_type = fwUpdate::MSG_REQ_UPDATE;
        request.data.req_update.session_id = session_id;
        request.data.req_update.image_slot = session_image_slot = image_slot;
        request.data.req_update.image_flags = session_image_flags = image_flags;
        request.data.req_update.chunk_size = session_chunk_size = chunk_size;
        request.data.req_update.file_size = session_image_size = image_size;
        request.data.req_update.progress_rate = session_progress_rate = progress_rate;
        request.data.req_update.md5_hash = session_md5 = image_md5;

        return fwUpdate_sendPayload(request);
    }

    bool FirmwareUpdateHost::fwUpdate_requestUpdate() {

        if ((session_status >= READY) || (session_id == 0))
            return false;

        fwUpdate::payload_t request;
        request.hdr.target_device = session_target;
        request.hdr.msg_type = fwUpdate::MSG_REQ_UPDATE;
        request.data.req_update.session_id = session_id;
        request.data.req_update.image_slot = session_image_slot;
        request.data.req_update.image_flags = session_image_flags;
        request.data.req_update.chunk_size = session_chunk_size;
        request.data.req_update.file_size = session_image_size;
        request.data.req_update.progress_rate = session_progress_rate;
        request.data.req_update.md5_hash = session_md5;

        return fwUpdate_sendPayload(request);
    }

    bool FirmwareUpdateHost::fwUpdate_requestReset(target_t target, uint16_t reset_flags) {
        // We don't care about having a session in order to request a reset (it's session independent)
        // if ((session_status != NOT_STARTED) || (session_id == 0))
        //    return false;

        fwUpdate::payload_t request;
        request.hdr.target_device = target;
        request.hdr.msg_type = fwUpdate::MSG_REQ_RESET;
        request.data.req_reset.reset_flags = reset_flags;

        return fwUpdate_sendPayload(request);
    }

    /**
     * Requests hardware/firmware version information from the specified device(s)
     * Note that you may get multiple responses for matching targets, if more than one device of that target exists,
     * such as querying the GNSS receivers, without explicitly indicated GNSS1 or GNSS2.
     * @param target
     * @return
     */
    bool FirmwareUpdateHost::fwUpdate_requestVersionInfo(target_t target) {
        // We don't care about having a session in order to request version info (it's session independent)
        // if ((session_status != NOT_STARTED) || (session_id == 0))
        //    return false;

        fwUpdate::payload_t request;
        request.hdr.target_device = target;
        request.hdr.msg_type = fwUpdate::MSG_REQ_VERSION_INFO;

        return fwUpdate_sendPayload(request);
    }


    int FirmwareUpdateHost::fwUpdate_sendNextChunk() {
        payload_t* msg = (payload_t*)&build_buffer;

        if (next_chunk_id >= session_total_chunks)
            return 0; // don't keep sending chunks... but also, don't "finish" the update (that's the remote's job).

        // I'm exploiting the pack/unpack + build_buffer to allow building a payload in place, including room for the chunk data.
        msg->hdr.target_device = session_target;
        msg->hdr.msg_type = fwUpdate::MSG_UPDATE_CHUNK;
        msg->data.chunk.session_id = session_id;
        msg->data.chunk.chunk_id = next_chunk_id;
        msg->data.chunk.data_len = ((msg->data.chunk.chunk_id == session_total_chunks-1) && (session_image_size % session_chunk_size)) ? (session_image_size % session_chunk_size) : session_chunk_size;

        // by calling unpackPayloadNoCopy(...) we basically just re-map the msg point back onto itself, but
        // we also get the *aux_data pointer, which we can then pass onto getNextChunk(), which it will write
        // directly into our build buffer.
        void *chunk_data = nullptr;
        int msg_len = fwUpdate_mapBufferToPayload(build_buffer, &msg, &chunk_data);

        uint32_t offset = msg->data.chunk.chunk_id * session_chunk_size;
        int chunk_len = fwUpdate_getImageChunk(offset, msg->data.chunk.data_len, &chunk_data);
        if (chunk_len == msg->data.chunk.data_len) {
            chunks_sent++; // we track the total number of chunks that we've tried to send, regardless of whether we sent it successfully or not

            #ifdef DEBUG_LOGGING
            // we don't call sendPayload from here (we just send our build_buffer direct to the writer.
            LOG_DBG("Sending to %s", fwUpdate_payloadToString(msg));
            #endif

            if (fwUpdate_writeToWire((fwUpdate::target_t) msg->hdr.target_device, build_buffer, msg_len))
                next_chunk_id = msg->data.chunk.chunk_id + 1; // increment to the next chuck, if we're successful
        }

        return (session_total_chunks - next_chunk_id);
    }

    /**
     * @return true if we have an active session and are updating.
     */
    bool FirmwareUpdateHost::fwUpdate_isUpdating() {
        return ((session_status >= READY) && (session_id != 0));
    }

    /**
     * Internal method use to reinitialize the update engine.  This should clear the the current session_id, existing image data, running md5 sums, etc.
     * After calling this function, the subsystem must receive a REQ_UPDATE message to start to new session, etc.
     * This probably should be called after an update is finished, but is probably safest to call as the first step in a REQ_UPDATE.
     * @return true if the system was able to properly initialize, false if there was an error of something (you have a REAL problem in this case).
     */
    bool FirmwareUpdateHost::fwUpdate_resetEngine() {
        resend_count = 0;
        session_status = NOT_STARTED;
        session_id = 0;
        session_chunk_size = 512;
        session_image_size = 0;
        session_image_slot = 0;
        next_chunk_id = 0;
        md5_init(md5Context);
        return true;
    }

    /**
     * @return a human-readable status name for the current session status
     */
    const char *FirmwareUpdateHost::fwUpdate_getSessionStatusName() {
        return FirmwareUpdateBase::fwUpdate_getStatusName(session_status);
    }

    /**
     * @return a human-readable status name for the current session status
     */
    const char *FirmwareUpdateHost::fwUpdate_getSessionTargetName() {
        return FirmwareUpdateBase::fwUpdate_getTargetName(session_target);
    }

} // fwUpdate
