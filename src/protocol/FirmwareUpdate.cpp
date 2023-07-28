//
// Created by kylemallory on 7/26/23.
//

#include "FirmwareUpdate.h"

namespace fwUpdate {

    /**
     * Creates a FirmwareUpdateDevice instance
     * @param target_id the target_id which this message will respond to.
     * @param startUpdate
     * @param writeChunk
     * @param resetMcu
     * @param progress_millis the rate at which progress updates will be sent out, in milli-seconds (default is every 100ms, 0 = no updates are sent).
     */
    // FirmwareUpdateDevice::FirmwareUpdateDevice(uint16_t tid, start_update_fn startUpdate, write_chunk_fn writeChunk, reset_mcu_fn resetMcu, uint16_t progress_millis) {
    FirmwareUpdateDevice::FirmwareUpdateDevice(uint16_t tid, uint16_t progress_millis) {
        target_id = tid;
//        cb_startUpdate = startUpdate;
//        cb_writeChunk = writeChunk;
//        cb_resetMcu = resetMcu;
        progress_interval = progress_millis;

        resetEngine();
    }

    /**
     * called at each step interval; if you put this behind a Scheduled Task, call this method at each interval.
     * This method is primarily used to perform routine maintenance, like checking if the init process is complete, or to give out status update, etc.
     * If you don't call step() things should still generally work, but it probably won't seem very responsive.
     * @return the number of times step() has been called since the last request to start an update. This isn't immediately useful, but its nice to know that its actually doing something.
     */
    int FirmwareUpdateDevice::step() {

    }

    /**
     * Called by the communications system anytime a DID_FIRMWARE_UPDATE is received.
     * @param msg_payload the contents of the DID_FIRMWARE_UPDATE payload
     * @return true if this message was consumed by this interface, or false if the message was not intended for us, and should be passed along to other ports/interfaces.
     *
     * Note: Internally, this method calls step(), so even if you don't call step(), but it can still operate with just inbound messages, but interval updates/etc won't run.
     */
    int FirmwareUpdateDevice::processMessage(const payload_t& msg_payload) {
        switch (msg_payload.msg_type) {
            case MSG_REQ_UPDATE:
                return handleInitialize(msg_payload);
            case MSG_UPDATE_CHUNK:
                return handleChunk(msg_payload);
            case MSG_REQ_RESET:
                return handleMcuReset(msg_payload);
        }
        return false;
    }

    /**
     * This is an internal method used to send an update message to the host system regarding the status of the update process
     * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
     * @param message the actual message to be sent to the host
     * @return true if the message was sent, false if there was an error
     */
    bool FirmwareUpdateDevice::sendProgress(int level, const std::string message) {
        payload_t did;

        did.msg_type = MSG_UPDATE_PROGRESS;
        did.msg_data.progress.session_id = cur_session_id;
        did.msg_data.progress.totl_chunks = total_chunks;
        did.msg_data.progress.num_chunks = last_chunk_id;
        did.msg_data.progress.msg_level = level;
        did.msg_data.progress.msg_len = message.length();

        // FIXME: How to write the *actual* message into the payload.  I suspect its more like writing everything into a buffer, and then

        // send the packet out onto the wire.
        return true;
    }

    /**
     * This is an internal method used to send an update message to the host system regarding the status of the update process
     * This variation allows for printf-based string formatting
     * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
     * @param message the actual message to be sent to the host
     * @
     * @return true if the message was sent, false if there was an error
     */
    bool FirmwareUpdateDevice::sendProgress(int level, const std::string message, ...) {
        // format into a single string, and then call sendProgress(level, msg);
    }

    /**
     * @return true if we have an active session and are updating.
     */
    bool FirmwareUpdateDevice::isUpdating() {
        return cur_session_id != 0;
    }

    /**
     * Internal method use to reinitialize the update engine.  This should clear the the current session_id, existing image data, running md5 sums, etc.
     * After calling this function, the subsystem must receive a REQ_UPDATE message to start to new session, etc.
     * This probably should be called after an update is finished, but is probably safest to call as the first step in a REQ_UPDATE.
     * @return true if the system was able to properly initialize, false if there was an error of something (you have a REAL problem in this case).
     */
    bool FirmwareUpdateDevice::resetEngine() {
        cur_session_id = 0;       //! the current session id - all received messages with a session_id must match this value.  O == no session set (invalid)
        last_chunk_id = 0xFFFF;    //! the last received chunk id from a CHUNK message.  0xFFFF == no chunk yet received; the next received chunk must be 0.
        resetMd5();
    }

    /**
     * Internally called by processMessage() when a REQ_UPDATE message is received.
     * @param payload the DID message
     * @return true if the message was received and parsed without error, false otherwise.
     *
     * NOTE, this function should call out and send an error status in the event of a failure.
     */
    bool FirmwareUpdateDevice::handleInitialize(const payload_t& payload) {
        if (payload.msg_type != MSG_REQ_UPDATE)
            return false;

        update_status_e status = NOT_STARTED;

        // is the session id a valid one (anything other than 0 is okay)
        // TODO: setup proper validation
        if ((status == NOT_STARTED) && (payload.msg_data.req_update.session_id == 0))
            status = ERR_INVALID_SESSION;

        // query information from the system about the requested firmware slot
        // TODO: setup proper validation
        if ((status == NOT_STARTED) && (payload.msg_data.req_update.image_slot != 0))
            status = ERR_INVALID_SLOT;

        // is the requested slot writable?
        // TODO: setup proper validation
        if ((status == NOT_STARTED) && (payload.msg_data.req_update.image_slot != 0))
            status = ERR_NOT_ALLOWED;

        // does the slot have enough space to accomodate the image
        // TODO: setup proper validation
        uint32_t slot_size = 0x8000000;
        if ((status == NOT_STARTED) && (payload.msg_data.req_update.file_size < slot_size))
            status = ERR_NOT_ENOUGH_MEMORY;

        // validate that the system can handle the requested chunk size (it should not exceed the RX buffer, and it should be multiples of 16 bytes (because I said so).
        // TODO: setup proper validation
        if ((status == NOT_STARTED) && (payload.msg_data.req_update.chunk_size > 2048))
            status = ERR_MAX_CHUNK_SIZE;

        if (status == NOT_STARTED) {
            resetEngine();
            cur_session_id = payload.msg_data.req_update.session_id;
            image_slot = payload.msg_data.req_update.image_slot;
            image_size = payload.msg_data.req_update.file_size;
            chunk_size = payload.msg_data.req_update.chunk_size;

            if (startFirmwareUpdate(target_id, image_slot, image_size)) {
                status = INITIALIZING;
            }
        }

        // prepare the response
        payload_t response;
        response.target_device = TARGET_NONE;
        response.msg_type = MSG_UPDATE_RESP;
        response.msg_data.update_resp.session_id = cur_session_id;
        response.msg_data.update_resp.totl_chunks = (uint16_t) ceil((float)image_size / (float)chunk_size);
        response.msg_data.update_resp.status = status;

        // TODO: send the response back out onto the wire
    }

    /**
     * Internally called by processMessage() when a UPDATE_CHUNK message is received.
     * @param payload the DID payload
     * @return
     */
    bool FirmwareUpdateDevice::handleChunk(const payload_t& payload) {
        if (payload.msg_type != MSG_UPDATE_CHUNK)
            return false;

        // if the session id doesn't match our current session id, then ignore this message.
        if (payload.msg_data.chunk.session_id != cur_session_id)
            return false;

        // if the chunk id does match the next expected chunk id, then send an resend for the correct/missing chunk
        if (payload.msg_data.chunk.session_id != last_chunk_id + 1) {
            sendRetry(REASON_INVALID_SEQID);
            return false;
        }

        // ensure data_len is the same as chunk_size, unless its the very last chunk
        if (payload.msg_data.chunk.data_len != chunk_size) {
            // if this is the very last chunk, and it doesn't equal the remaining number of bytes in the file, its invalid.
            if ((payload.msg_data.chunk.chunk_id == total_chunks) && (payload.msg_data.chunk.data_len == (image_size % chunk_size))) {
                // do nothing... this is exactly what we wanted!
            } else {
                sendRetry(REASON_INVALID_SIZE);
            }
            return false;
        }

        // if we're here, all our validations have passed...

        // run the chunk data through the md5 hasher
        hashMd5(payload.msg_data.chunk.data_len, (uint8_t *)&payload.msg_data.chunk.data);

        // TODO: write the buffer into FLASH, or write the data out to the GNSS receiver, etc..
        // Zephyr: OTA write to image slot

        // note that we successfully wrote this chunk, so we can start to handle the next.
        last_chunk_id = payload.msg_data.chunk.chunk_id;
        return true;
    }

    /**
     * Internally called by processMessage() when a REQ_RESET message is received, to reset the target MCU.
     * @param payload the DID message
     * @return true if the message was received and parsed without error, false otherwise.
     */
    bool FirmwareUpdateDevice::handleMcuReset(const payload_t& payload) {
        if (payload.msg_type != MSG_REQ_RESET)
            return false;

        payload_t response;
        response.target_device = TARGET_NONE;
        response.msg_type = MSG_RESET_RESP;
        // TODO: send the MSG_RESET_RESP

        // actually perform the reset
        performSoftReset(target_id);
    }

    /**
     * Sends a REQ_RESEND_CHUNK message in response to receiving an invalid CHUNK message.
     * @param payload the originally received invalid chunk message that cause this RETRY to be sent
     * @return return true is a retry was sent, or false if a retry was not sent.  NOTE this is not an error, as a valid message will not send a retry.
     */
    bool FirmwareUpdateDevice::sendRetry(resend_reason_e reason) {
        payload_t response;
        response.target_device = TARGET_NONE;
        response.msg_type = MSG_REQ_RESEND_CHUNK;
        response.msg_data.req_resend.session_id = cur_session_id;
        response.msg_data.req_resend.chunk_id = last_chunk_id + 1;
        response.msg_data.req_resend.reason = reason;

        // TODO: Actually put the payload on the wire, and return true if successful
        return false;
    }

    /**
     * Initializes the MD5 hash. Don't forget to call hashMd5() afterwards to actually get your hash
     */
    void FirmwareUpdateDevice::resetMd5() {

    }

    /**
     * Adds the specified data into the running MD5 hash
     * @param len the number of bytes to consume into the hash
     * @param data the bytes to consume into the hash
     * @return a static buffer of 16 unsigned bytes which represent the 128 total bits of the MD5 hash
     */
    uint8_t* FirmwareUpdateDevice::hashMd5(uint16_t len, uint8_t* data) {

    }
} // fwUpdate