//
// Created by kylemallory on 7/26/23.
//

#include <istream>
#include <stdarg.h>
#include "FirmwareUpdate.h"

namespace fwUpdate {

    static const char* status_names[] = { "ERR_NOT_SUPPORTED", "ERR_COMMS", "ERR_CHECKSUM_MISMATCH", "ERR_TIMEOUT", "ERR_MAX_CHUNK_SIZE", "ERR_OLDER_FIRMWARE", "ERR_NOT_ENOUGH_MEMORY", "ERR_NOT_ALLOWED", "ERR_INVALID_SLOT", "ERR_INVALID_SESSION",
    "NOT_STARTED", "INITIALIZING", "GOOD_TO_GO", "WAITING_FOR_DATA", "FINISHED"};

    /*==================================================================================*
     * Firmware Base Implementation goes here                                           *
     *==================================================================================*/

    FirmwareUpdateBase::FirmwareUpdateBase() { };

    size_t FirmwareUpdateBase::getMsgSize(const payload_t* msg, bool include_aux) {
        switch (msg->hdr.msg_type) {
            case MSG_REQ_RESET:
                return sizeof(msg->hdr) + sizeof(msg->data.req_reset);
            case MSG_RESET_RESP:
                return sizeof(msg->hdr) + sizeof(msg->data.rpl_reset);
            case MSG_REQ_UPDATE:
                return sizeof(msg->hdr) + sizeof(msg->data.req_update);
            case MSG_UPDATE_RESP:
                return sizeof(msg->hdr) + sizeof(msg->data.update_resp);
            case MSG_UPDATE_CHUNK:
                return sizeof(msg->hdr) + sizeof(msg->data.chunk) + (include_aux ? msg->data.chunk.data_len - 1 : -1);
            case MSG_REQ_RESEND_CHUNK:
                return sizeof(msg->hdr) + sizeof(msg->data.req_resend);
            case MSG_UPDATE_PROGRESS:
                return sizeof(msg->hdr) + sizeof(msg->data.progress) + (include_aux ? msg->data.progress.msg_len - 1 : -1);
            case MSG_UPDATE_FINISHED:
                return sizeof(msg->hdr) + sizeof(msg->data.resp_done);
            case MSG_REQ_VERSION_INFO:
                return sizeof(msg->hdr) + sizeof(msg->data.req_version);
            case MSG_VERSION_INFO_RESP:
                return sizeof(msg->hdr) + sizeof(msg->data.version_resp);
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
    int FirmwareUpdateBase::packPayload(uint8_t* buffer, int max_len, const payload_t& msg_payload, const void *aux_data) {
        int msg_size = getMsgSize(&msg_payload, false);
        if (msg_size > max_len) return -1; // Not enough buffer space
        if (msg_size == 0) return -2; // Unknown/invalid message

        memcpy(buffer, (void *) &msg_payload, msg_size);

        int aux_len = 0;
        if (msg_payload.hdr.msg_type == MSG_UPDATE_CHUNK)
            aux_len = msg_payload.data.chunk.data_len;
        else if (msg_payload.hdr.msg_type == MSG_UPDATE_PROGRESS)
            aux_len = msg_payload.data.progress.msg_len;

        if ((aux_len > 0) && (aux_data != nullptr)) {
            memcpy( ((uint8_t*)buffer) + msg_size - 1, (uint8_t*)aux_data, aux_len);
        }

        return msg_size + aux_len;
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
    int FirmwareUpdateBase::unpackPayload(const uint8_t* buffer, int buf_len, payload_t& msg_payload, void *aux_data, uint16_t max_aux) {
        int msg_size = getMsgSize((payload_t *)buffer);
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

        if (buf_len < msg_size + aux_len) return -1; // the passed buffer is incomplete (smaller than the unpacked message size)

        memcpy((void *)(&msg_payload), buffer, msg_size);
        if ((aux_len <= max_aux) && (aux_data != nullptr)) {
            memcpy( aux_data, aux_ptr, aux_len);
        }

        return msg_size + aux_len;
    }

    /**
     * Unpacks a DID payload byte buffer (from the comms system) into a fwUpdate::payload_t struct, but avoids making copies of the data.
     * @param buffer a pointer to the raw byte buffer
     * @param buf_len the number of bytes in the raw byte buffer
     * @param msg_payload a double-pointer which on return will point to the start of the buffer (this is a simple cast)
     * @param aux_data a double-pointer which on return will point to any auxilary data in the payload, or nullptr if there is none
     * @return returns the total number of bytes in the packet, including aux data if any
     */
    int FirmwareUpdateBase::unpackPayloadNoCopy(const uint8_t *buffer, int buf_len, payload_t** msg_payload, void** aux_data) {
        int msg_size = getMsgSize((payload_t *)buffer);
        int aux_len = 0;

        if (((payload_t*)buffer)->hdr.msg_type == MSG_UPDATE_CHUNK) {
            aux_len = ((payload_t*)buffer)->data.chunk.data_len;
            *aux_data = &(((payload_t*)buffer)->data.chunk.data);
        }
        else if (((payload_t*)buffer)->hdr.msg_type == MSG_UPDATE_PROGRESS) {
            aux_len = ((payload_t*)buffer)->data.progress.msg_len;
            *aux_data = &(((payload_t*)buffer)->data.progress.message);
        }

        *msg_payload = (payload_t*)buffer;
        return msg_size + aux_len;
    }

    /**
     * packages and sends the specified payload, including any auxillary data.
     * Note that the payload must already specify the amount of aux data the be included.
     * @param payload
     * @param aux_data the auxillary data to include, or nullptr if none.
     * @return
     */
    bool FirmwareUpdateBase::sendPayload(fwUpdate::payload_t& payload, void *aux_data) {
        int payload_len = packPayload(build_buffer, FWUPDATE__MAX_PAYLOAD_SIZE, payload, aux_data);
        return writeToWire((fwUpdate::target_t)payload.hdr.target_device, build_buffer, payload_len);
    }

    //
    // MD5 implementation from here: https://gist.github.com/creationix/4710780
    //

    /**
     * Initializes the MD5 hash. Don't forget to call hashMd5() afterwards to actually get your hash
     */
    void FirmwareUpdateBase::resetMd5() {
        // seed the hash
        md5hash[0] = 0x67452301;
        md5hash[1] = 0xefcdab89;
        md5hash[2] = 0x98badcfe;
        md5hash[3] = 0x10325476;
    }

    /**
     * Adds the specified data into the running MD5 hash
     * @param len the number of bytes to consume into the hash
     * @param data the bytes to consume into the hash
     * @return a static buffer of 16 unsigned bytes which represent the 128 total bits of the MD5 hash
     *
     * TODO: This function uses dynamic memory to allocate memory for the data buffer. Since our implementation
     * will generally be using the fixed size of the session_chunk_size, we can probably do this allocation once and
     * reuse the buffer, instead of allocating and then freeing with each call.  Likewise, we maybe able to
     * define a static buffer of MAX_CHUNK_SIZE and go that route as well.
     */
    #define MD5_LEFTROTATE(x, c) (((x) << (c)) | ((x) >> (32 - (c))))
    uint8_t* FirmwareUpdateBase::hashMd5(size_t data_len, uint8_t* data) {
        // Message (to prepare)
        uint8_t *msg = NULL;

        // Note: All variables are unsigned 32 bit and wrap modulo 2^32 when calculating

        // r specifies the per-round shift amounts

        static uint32_t r[] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                        5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
                        4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                        6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};

        // Use binary integer part of the sines of integers (in radians) as constants// Initialize variables:
        static uint32_t k[] = {
                0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
                0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
                0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
                0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
                0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
                0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
                0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
                0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
                0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
                0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
                0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
                0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
                0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
                0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
                0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
                0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};


        // Pre-processing: adding a single 1 bit
        //append "1" bit to message
        /* Notice: the input bytes are considered as bits strings,
           where the first bit is the most significant bit of the byte.[37] */

        // Pre-processing: padding with zeros
        //append "0" bit until message length in bit ≡ 448 (mod 512)
        //append length mod (2 pow 64) to message

        int new_len = ((((data_len + 8) / 64) + 1) * 64) - 8;

        msg = static_cast<uint8_t *>(calloc(new_len + 64, 1)); // also appends "0" bits
        // (we alloc also 64 extra bytes...)
        memcpy(msg, data, data_len);
        msg[data_len] = 128; // write the "1" bit

        uint32_t bits_len = 8*data_len; // note, we append the len
        memcpy(msg + new_len, &bits_len, 4);           // in bits at the end of the buffer

        // Process the message in successive 512-bit chunks:
        //for each 512-bit chunk of message:
        int offset;
        for(offset=0; offset<new_len; offset += (512/8)) {

            // break chunk into sixteen 32-bit words w[j], 0 ≤ j ≤ 15
            uint32_t *w = (uint32_t *) (msg + offset);

            // Initialize hash value for this chunk:
            uint32_t a = md5hash[0];
            uint32_t b = md5hash[1];
            uint32_t c = md5hash[2];
            uint32_t d = md5hash[3];

            // Main loop:
            uint32_t i;
            for(i = 0; i<64; i++) {
                uint32_t f, g;

                if (i < 16) {
                    f = (b & c) | ((~b) & d);
                    g = i;
                } else if (i < 32) {
                    f = (d & b) | ((~d) & c);
                    g = (5*i + 1) % 16;
                } else if (i < 48) {
                    f = b ^ c ^ d;
                    g = (3*i + 5) % 16;
                } else {
                    f = c ^ (b | (~d));
                    g = (7*i) % 16;
                }

                uint32_t temp = d;
                d = c;
                c = b;
                b = b + MD5_LEFTROTATE((a + f + k[i] + w[g]), r[i]);
                a = temp;
            }

            // Add this chunk's hash to result so far:

            md5hash[0] += a;
            md5hash[1] += b;
            md5hash[2] += c;
            md5hash[3] += d;
        }

        // cleanup
        free(msg);

        return (uint8_t *)&md5hash[0];
    }

    /**
     * updates the passed reference to an array, the current running md5 sum.
     * @param md5sum the reference to an array of uint32_t[4] where the md5 sum will be stored
     */
    void FirmwareUpdateBase::getCurrentMd5(uint32_t(&md5sum)[4]) {
        md5sum[0] = md5hash[0];
        md5sum[1] = md5hash[1];
        md5sum[2] = md5hash[2];
        md5sum[3] = md5hash[3];
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
    FirmwareUpdateDevice::FirmwareUpdateDevice(target_t tid, uint16_t progress_millis) {
        target_id = tid;
        progress_interval = progress_millis;

        resetEngine();
    }


    /**
     * Called by the communications system anytime a DID_FIRMWARE_UPDATE is received.
     * @param msg_payload the contents of the DID_FIRMWARE_UPDATE payload
     * @return true if this message was consumed by this interface, or false if the message was not intended for us, and should be passed along to other ports/interfaces.
     *
     * Note: Internally, this method calls step(), so even if you don't call step(), it can still operate with just inbound messages, but interval updates/etc won't run.
     */
    bool FirmwareUpdateDevice::processMessage(const payload_t& msg_payload) {
        bool result = false;

        if (msg_payload.hdr.target_device != target_id)
            return false; // if this message isn't for us, then we return false which will forward this message on to other connected devices

        switch (msg_payload.hdr.msg_type) {
            case MSG_REQ_UPDATE:
                result = handleInitialize(msg_payload);
                break;
            case MSG_UPDATE_CHUNK:
                result = handleChunk(msg_payload);
                break;
            case MSG_REQ_RESET:
                result = handleMcuReset(msg_payload);
                break;
            default:
                result = false;
        }
        
        step(); // TODO: we should probably do something with the step() result, but not sure what just yet

        return result;
    }
    bool FirmwareUpdateDevice::processMessage(const uint8_t* buffer, int buf_len) {
        fwUpdate::payload_t *msg = nullptr;
        void *aux_data = nullptr;

        int msg_len = unpackPayloadNoCopy(buffer, buf_len, &msg, &aux_data);
        if (msg_len <= 0)
            return false;

        return processMessage(*msg);
    }

    /**
     * This is an internal method used to send an update message to the host system regarding the status of the update process
     * This message only include the number of chunks sent, and the total expected (sufficient for a percentage) and the
     * @return true if the message was sent, false if there was an error
     */
    bool FirmwareUpdateDevice::sendProgress() {
        return sendProgressFormatted(99, nullptr);
    }


    /**
     * This is an internal method used to send an update message to the host system regarding the status of the update process
     * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
     * @param message the actual message to be sent to the host
     * @return true if the message was sent, false if there was an error
     */
    bool FirmwareUpdateDevice::sendProgress(int level, const std::string message) {
        return sendProgressFormatted(level, message.c_str());
    }

    /**
     * This is an internal method used to send an update message to the host system regarding the status of the update process
     * This variation allows for printf-based string formatting
     * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
     * @param message the actual message to be sent to the host
     * @
     * @return true if the message was sent, false if there was an error
     */
    bool FirmwareUpdateDevice::sendProgressFormatted(int level, const char* message, ...) {
        static char buffer[256];
        size_t msg_len = 0;

        if (message) {
            va_list ap;
            va_start(ap, message);
            msg_len = vsnprintf(buffer, sizeof(buffer) - 1, message, ap);
            va_end(ap);
        }

        if (msg_len >= sizeof(buffer)-1)
            return false;

        payload_t msg;
        msg.hdr.target_device = target_id;
        msg.hdr.msg_type = MSG_UPDATE_PROGRESS;
        msg.data.progress.session_id = cur_session_id;
        msg.data.progress.status = session_status;
        msg.data.progress.totl_chunks = total_chunks;
        msg.data.progress.num_chunks = last_chunk_id;
        msg.data.progress.msg_level = level;
        msg.data.progress.msg_len = msg_len + 1; // don't forget the null-terminator

        msg_len = packPayload(build_buffer, sizeof(build_buffer), msg, buffer);
        return writeToWire((fwUpdate::target_t)msg.hdr.target_device, build_buffer, msg_len);
    }

    /**
     * @return true if we have an active session and are updating.
     */
    bool FirmwareUpdateDevice::isUpdating() {
        return (cur_session_id != 0) && (session_status > NOT_STARTED);
    }

    /**
     * Internal method use to reinitialize the update engine.  This should clear the the current session_id, existing image data, running md5 sums, etc.
     * After calling this function, the subsystem must receive a REQ_UPDATE message to start to new session, etc.
     * This probably should be called after an update is finished, but is probably safest to call as the first step in a REQ_UPDATE.
     * @return true if the system was able to properly initialize, false if there was an error of something (you have a REAL problem in this case).
     */
    bool FirmwareUpdateDevice::resetEngine() {
        cur_session_id = 0;       //! the current session id - all received messages with a session_id must match this value.  O == no session set (invalid)
        last_chunk_id = 0xFFFF;   //! the last received chunk id from a CHUNK message.  0xFFFF == no chunk yet received; the next received chunk must be 0.
        session_status = NOT_STARTED;
        resetMd5();
        return true;
    }

    /**
     * Internally called by processMessage() when a REQ_UPDATE message is received.
     * @param payload the DID message
     * @return true if the message was received and parsed without error, false otherwise.
     *
     * NOTE, this function should call out and send an error status in the event of a failure.
     */
    bool FirmwareUpdateDevice::handleInitialize(const payload_t& payload) {
        if (payload.hdr.target_device != target_id)
            return false;

        if (payload.hdr.msg_type != MSG_REQ_UPDATE)
            return false;

        // make sure our session_id is good (TODO: Maybe we should retain a history of recent session ids to make sure we aren't reusing an old one?)
        if (payload.data.req_update.session_id == 0)
            session_status = ERR_INVALID_SESSION;

        resetEngine();
        session_status = startFirmwareUpdate(payload);
        cur_session_id = payload.data.req_update.session_id;
        image_size = payload.data.req_update.file_size;
        chunk_size = payload.data.req_update.chunk_size;

        if (session_status > NOT_STARTED) {
            image_slot = payload.data.req_update.image_slot;
            total_chunks = (uint16_t) ceil((float)image_size / (float)chunk_size);
            session_md5[0] = payload.data.req_update.md5_hash[0];
            session_md5[1] = payload.data.req_update.md5_hash[1];
            session_md5[2] = payload.data.req_update.md5_hash[2];
            session_md5[3] = payload.data.req_update.md5_hash[3];
        }

        // prepare the response
        payload_t response;
        response.hdr.target_device = TARGET_NONE;
        response.hdr.msg_type = MSG_UPDATE_RESP;
        response.data.update_resp.session_id = cur_session_id;
        response.data.update_resp.totl_chunks = total_chunks;
        response.data.update_resp.status = session_status;

        return sendPayload(response);
    }

    /**
     * Internally called by processMessage() when a UPDATE_CHUNK message is received.
     * @param payload the DID payload
     * @return
     */
    bool FirmwareUpdateDevice::handleChunk(const payload_t& payload) {
        if (payload.hdr.msg_type != MSG_UPDATE_CHUNK)
            return false;

        // if the session id doesn't match our current session id, then ignore this message.
        if (payload.data.chunk.session_id != cur_session_id)
            return false;

        // if the chunk id does match the next expected chunk id, then send an resend for the correct/missing chunk
        if (payload.data.chunk.chunk_id != (uint16_t)(last_chunk_id + 1)) {
            sendRetry(REASON_INVALID_SEQID);
            return false;
        }

        // ensure data_len is the same as session_chunk_size, unless its the very last chunk
        if (payload.data.chunk.data_len != chunk_size) {
            // if this is the very last chunk, and it doesn't equal the remaining number of bytes in the file, its invalid.
            if (!((payload.data.chunk.chunk_id == total_chunks) && (payload.data.chunk.data_len == (image_size % chunk_size)))) {
                sendRetry(REASON_INVALID_SIZE);
            }
            return false;
        }

        // if we're here, all our validations have passed...
        session_status = WAITING_FOR_DATA;

        // run the chunk data through the md5 hasher
        hashMd5(payload.data.chunk.data_len, (uint8_t *)&payload.data.chunk.data);

        uint32_t chnk_offset = payload.data.chunk.chunk_id * chunk_size;
        writeImageChunk((fwUpdate::target_t)payload.hdr.target_device, image_slot, chnk_offset, payload.data.chunk.data_len, (uint8_t *)&payload.data.chunk.data);

        // note that we successfully wrote this chunk, so we can start to handle the next.
        last_chunk_id = payload.data.chunk.chunk_id;

        // if we've received the last message, confirm the checksum and then send a final status to notify the host that we've received everything error-free.
        if (last_chunk_id >= (total_chunks - 1)) {
            payload_t response;
            response.hdr.target_device = TARGET_NONE;
            response.hdr.msg_type = MSG_UPDATE_FINISHED;
            response.data.resp_done.session_id = cur_session_id;
            if (finishFirmwareUpgrade(target_id, image_slot)) {
                response.data.resp_done.status = FINISHED;
            } else {
                response.data.resp_done.status = ERR_CHECKSUM_MISMATCH;
            }
            sendPayload(response);
        }

        return true;
    }

    /**
     * Internally called by processMessage() when a REQ_RESET message is received, to reset the target MCU.
     * @param payload the DID message
     * @return true if the message was received and parsed without error, false otherwise.
     */
    bool FirmwareUpdateDevice::handleMcuReset(const payload_t& payload) {
        if (payload.hdr.msg_type != MSG_REQ_RESET)
            return false;

        payload_t response;
        response.hdr.target_device = TARGET_NONE;
        response.hdr.msg_type = MSG_RESET_RESP;
        sendPayload(response); // make sure this goes out before the reset happens. We might need to schedule the reset, so the send can happen, if the underlying call doesn't block.

        return performSoftReset(target_id); // TODO: add option to support both hard and soft resets?
    }

    /**
     * Sends a REQ_RESEND_CHUNK message in response to receiving an invalid CHUNK message.
     * @param payload the originally received invalid chunk message that cause this RETRY to be sent
     * @return return true is a retry was sent, or false if a retry was not sent.  NOTE this is not an error, as a valid message will not send a retry.
     */
    bool FirmwareUpdateDevice::sendRetry(resend_reason_e reason) {
        payload_t response;
        response.hdr.target_device = TARGET_NONE;
        response.hdr.msg_type = MSG_REQ_RESEND_CHUNK;
        response.data.req_resend.session_id = cur_session_id;
        response.data.req_resend.chunk_id = last_chunk_id + 1;
        response.data.req_resend.reason = reason;

        sendPayload(response);
        return false;
    }

    /*==================================================================================*
     * SDK-API goes here                                                                *
     *==================================================================================*/

    /**
     * Creates a FirmwareUpdateSDK instance
     */
    FirmwareUpdateSDK::FirmwareUpdateSDK() { };

    /**
     * Call this any time a DID_FIRMWARE_UPDATE is received by the comms system, to parse and process the message.
     * @param msg_payload the contents of the DID_FIRMWARE_UPDATE payload
     * @return true if this message was consumed by this interface, or false if the message was not intended for us, and should be passed along to other ports/interfaces.
     */
    bool FirmwareUpdateSDK::processMessage(const payload_t& msg_payload) {
        if ((msg_payload.hdr.msg_type != MSG_REQ_UPDATE) && (msg_payload.data.update_resp.session_id != cur_session_id))
            return false; // ignore this message, its not for us

        switch (msg_payload.hdr.msg_type) {
            case MSG_UPDATE_RESP:
                return handleUpdateResponse(msg_payload);
            case MSG_REQ_RESEND_CHUNK:
                return handleResendChunk(msg_payload);
            case MSG_UPDATE_PROGRESS:
                return handleUpdateProgress(msg_payload);
            case MSG_UPDATE_FINISHED:
                session_status = msg_payload.data.resp_done.status;
                return true;
            case MSG_VERSION_INFO_RESP:
                // FIXME: we want to do something with this data...
                return false;
            default:
                return false;
        }
        return false;
    }
    bool FirmwareUpdateSDK::processMessage(const uint8_t* buffer, int buf_len) {
        fwUpdate::payload_t *msg = nullptr;
        void *aux_data = nullptr;

        int msg_len = unpackPayloadNoCopy(buffer, buf_len, &msg, &aux_data);
        if (msg_len <= 0)
            return false;

        return processMessage(*msg);
    }

    /**
     * Called by the host application to initiate a request by the SDK to update a target device.
     * @param target_id
     * @param image_id
     * @param image
     * @param chunk_size
     * @return
     */
    bool FirmwareUpdateSDK::requestUpdate(target_t target, int image_slot, uint16_t chunk_size, uint32_t image_size, uint32_t image_md5[4]) {

        // FIXME:  Should probably check to see if an update is already in progress, and attempt to finish it first?
        fwUpdate::payload_t request;
#ifdef __ZEPHYR__
        cur_session_id = (uint16_t)sys_rand32_get();
#else
        cur_session_id = (uint16_t)random();
#endif

        request.hdr.target_device = target_id = target;
        request.hdr.msg_type = fwUpdate::MSG_REQ_UPDATE;
        request.data.req_update.session_id = cur_session_id;
        request.data.req_update.image_slot = session_image_slot = image_slot;
        request.data.req_update.chunk_size = session_chunk_size = chunk_size;
        request.data.req_update.file_size = session_image_size = image_size;
        request.data.req_update.md5_hash[0] = md5hash[0] = image_md5[0];
        request.data.req_update.md5_hash[1] = md5hash[1] = image_md5[1];
        request.data.req_update.md5_hash[2] = md5hash[2] = image_md5[2];
        request.data.req_update.md5_hash[3] = md5hash[3] = image_md5[3];

        return sendPayload(request);
    }

    bool FirmwareUpdateSDK::requestUpdate() {

        if ((session_status != INITIALIZING) || (cur_session_id == 0))
            return false;

        fwUpdate::payload_t request;
        request.hdr.target_device = target_id;
        request.hdr.msg_type = fwUpdate::MSG_REQ_UPDATE;
        request.data.req_update.session_id = cur_session_id;
        request.data.req_update.image_slot = session_image_slot;
        request.data.req_update.chunk_size = session_chunk_size;
        request.data.req_update.file_size = session_image_size;
        request.data.req_update.md5_hash[0] = md5hash[0];
        request.data.req_update.md5_hash[1] = md5hash[1];
        request.data.req_update.md5_hash[2] = md5hash[2];
        request.data.req_update.md5_hash[3] = md5hash[3];

        return sendPayload(request);
    }

    int FirmwareUpdateSDK::sendNextChunk() {
        payload_t* msg = (payload_t*)&build_buffer;

        // I'm exploiting the pack/unpack + build_buffer to allow building a payload in place, including room for the chunk data.
        msg->hdr.target_device = target_id;
        msg->hdr.msg_type = fwUpdate::MSG_UPDATE_CHUNK;
        msg->data.chunk.session_id = cur_session_id;
        msg->data.chunk.chunk_id = next_chunk_id;
        msg->data.chunk.data_len = (msg->data.chunk.chunk_id < session_total_chunks ? session_chunk_size : session_image_size % session_chunk_size);

        // by calling unpackPayloadNoCopy(...) we basically just re-map the msg point back onto itself, but
        // we also get the *aux_data pointer, which we can then pass onto getNextChunk(), which it will write
        // directly into our build buffer.
        void *chunk_data = nullptr;
        int msg_len = unpackPayloadNoCopy(build_buffer, sizeof(build_buffer), &msg, &chunk_data);

        uint32_t offset = msg->data.chunk.chunk_id * session_chunk_size;
        int chunk_len = getImageChunk(offset, msg->data.chunk.data_len, &chunk_data);
        if (chunk_len == msg->data.chunk.data_len)
            if (writeToWire((fwUpdate::target_t)msg->hdr.target_device, build_buffer, msg_len))
                next_chunk_id = msg->data.chunk.chunk_id + 1; // increment to the next chuck, if we're successful

        return (session_total_chunks - next_chunk_id);
    }

    /**
     * @return true if we have an active session and are updating.
     */
    bool FirmwareUpdateSDK::isUpdating() {
        return ((session_status >= GOOD_TO_GO) && (cur_session_id != 0));
    }

    /**
     * Internal method use to reinitialize the update engine.  This should clear the the current session_id, existing image data, running md5 sums, etc.
     * After calling this function, the subsystem must receive a REQ_UPDATE message to start to new session, etc.
     * This probably should be called after an update is finished, but is probably safest to call as the first step in a REQ_UPDATE.
     * @return true if the system was able to properly initialize, false if there was an error of something (you have a REAL problem in this case).
     */
    bool FirmwareUpdateSDK::resetEngine() {
        return false;
    }

    /**
     * @return a human-readable status name for the current session status
     */
    const char *FirmwareUpdateSDK::getSessionStatusName() {
        if (session_status >= 0)
            return fwUpdate::status_names[session_status + abs(fwUpdate::ERR_NOT_SUPPORTED)];
        else
            return fwUpdate::status_names[session_status - fwUpdate::ERR_NOT_SUPPORTED];
    }




} // fwUpdate