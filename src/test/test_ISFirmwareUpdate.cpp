//
// Created by kylemallory on 7/31/23.
//

#include <gtest/gtest.h>
#include "gtest_helpers.h"

#include "../protocol/FirmwareUpdate.h"

/**
 * This is a really basic FIFO buffer implementation.  It is NOT a ring buffer.
 * Put data into it upto the max buffer size, and you can pull data out upto the amount of data in the buffer.
 * This is intended to be a simple Exchange Buffer - a place where one thing can write data, so another thing can
 * then immediately read that data. There is not effort to maintain data integrity, etc.
 */

#define DEBUG_INFO

static const char* MSG_TYPES[] = { "UNKNOWN", "REQ_RESET", "RESET_RESP", "REQ_UPDATE", "UPDATE_RESP", "UPDATE_CHUNK", "UPDATE_PROGRESS", "REQ_RESEND", "UPDATE_FINISHED" };
static const char* STATUS_TYPES[] = { "CHECKSUM_MISMATCH", "TIMEOUT", "MAX_CHUNK_SIZE", "OLDER_FIRMWARE", "NOT_ENOUGH_MEMORY", "NOT_ALLOWED", "INVALID_SLOT", "INVALID_SESSION", "NOT_STARTED", "INITIALIZING", "GOOD_TO_GO", "WAITING_FOR_DATA", "FINISHED" };

static uint32_t fake_md5[4] = { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F };
static uint32_t real_md5[4] = { 0xff3a9932, 0xb52b501d, 0x802f05e4, 0x5ed33d04 };

class ExchangeBuffer {
public:
    ExchangeBuffer(int size) {
        buff_size = size;
        buffer = new uint8_t[size];
    }

    ~ExchangeBuffer() { delete[] buffer; }

    int dataAvailable() { return data_available; }

    int readData(uint8_t *data, int max_data) {
        int data_len = (max_data > data_available ? data_available : max_data);

        memcpy(data, buffer, data_len);
        data_available -= data_len;

        memmove(buffer, buffer + data_len, (buff_size - data_len));
        return data_len;
    }

    bool writeData(uint8_t* data, int data_len) {
        if (data_available + data_len > buff_size)
            return false;

        memcpy(buffer + data_available, data, data_len);
        data_available += data_len;
        return true;
    }

    void flush() {
        memset(buffer, 0, buff_size);
        data_available = 0;
    }

private:
    uint8_t *buffer = nullptr;
    int32_t buff_size;
    int32_t data_available = 0;
};


/**
 * These  next few tests attempts to verify that a generic (base-derived) implementation for a device and the SDK
 * properly respond to the correct messages.  Such as, a REQ_UPDATE initiates the update process and we should see
 * an UPDATE_RESP response generated.  We don't actually test the interaction with the comms system; we have a basic
 * comms buffer where we will pack message into, and then unpack from that same buffer location.
 *
 * To do this, we first implement a "Test" device/SDK classes.
 */

class ISFirmwareUpdateTestDev : public fwUpdate::FirmwareUpdateDevice {
public:
    ExchangeBuffer& exchangeBuffer;

    ISFirmwareUpdateTestDev(ExchangeBuffer& eb) : FirmwareUpdateDevice(fwUpdate::TARGET_IMX5, 100), exchangeBuffer(eb) {
    }

    int performSoftReset(fwUpdate::target_t target_id) { return 0; };
    int performHardReset(fwUpdate::target_t target_id) { return 0; };

    // this initializes the system to begin receiving firmware image chunks for the target device, image slot and image size
    fwUpdate::update_status_e startFirmwareUpdate(fwUpdate::payload_t msg) {

        // query information from the system about the requested firmware slot
        if (msg.data.req_update.image_slot != 0)
            return fwUpdate::ERR_INVALID_SLOT;

        uint32_t slot_size = 0x800000;
        if (msg.data.req_update.file_size > slot_size)
            return fwUpdate::ERR_NOT_ENOUGH_MEMORY;

        // validate that the system can handle the requested chunk size (it should not exceed the RX buffer, and it should be multiples of 16 bytes (because I said so).
        if (msg.data.req_update.chunk_size > 2048)
            return fwUpdate::ERR_MAX_CHUNK_SIZE;

        return fwUpdate::GOOD_TO_GO;
    }

    // writes the indicated block of data (of len bytes) to the target and device-specific image slot, and with the specified offset
    int writeImageChunk(fwUpdate::target_t target_id, int slot_id, int offset, int len, uint8_t *data) {
        return 0;
    }

    // this marks the finish of the upgrade, that all image bytes have been received, the md5 sum passed, and the device can complete the requested upgrade, and perform any device-specific finalization
    int finishFirmwareUpgrade(fwUpdate::target_t target_id, int slot_id) {
        uint32_t hash[4];
        // check that our md5 matches.  Return >0 is we're error free.
        getCurrentMd5(hash);
        if ( (hash[0] == session_md5[0]) && (hash[1] == session_md5[1]) && (hash[2] == session_md5[2]) && (hash[3] == session_md5[3]) )
            return 1;

        return 0;
    }

    // called internally to process a packed payload, ready to be put on the wire.
    bool writeToWire(uint8_t* buffer, int buff_len) {
        return exchangeBuffer.writeData(buffer, buff_len);
    }

    /**
     * called at each step interval; if you put this behind a Scheduled Task, call this method at each interval.
     * This method is primarily used to perform routine maintenance, like checking if the init process is complete, or to give out status update, etc.
     * If you don't call step() things should still generally work, but it probably won't seem very responsive.
     * @return the number of times step() has been called since the last request to start an update. This isn't immediately useful, but its nice to know that its actually doing something.
     */
    virtual int step() {
        static uint8_t buffer[2048];
        static uint32_t step_num = 0;
        fwUpdate::payload_t *msg = nullptr;
        void *aux_data = nullptr;
        bool success = false;

        // check if a packet is waiting in the exchange buffer.
        if (exchangeBuffer.dataAvailable() > 0) {
            int buf_len = exchangeBuffer.readData(buffer, sizeof(buffer));
            int msg_len = unpackPayloadNoCopy(buffer, buf_len, &msg, &aux_data);
            if (msg_len > 0) {
#ifdef DEBUG_INFO
                if (msg->hdr.msg_type == fwUpdate::MSG_UPDATE_CHUNK)
                    PRINTF("DEV :: Received MSG %s (Chunk %d)...\n", MSG_TYPES[msg->hdr.msg_type], msg->data.chunk.chunk_id);
                else
                    PRINTF("DEV :: Received MSG %s...\n", MSG_TYPES[msg->hdr.msg_type]);
#endif
                if (processMessage(*msg))
                    step_num++;
            }
        }
        return step_num;
    }
};

class ISFirmwareUpdateTestSDK : public fwUpdate::FirmwareUpdateSDK {
public:
    ExchangeBuffer& exchangeBuffer;

    explicit ISFirmwareUpdateTestSDK(ExchangeBuffer& eb) : FirmwareUpdateSDK(), exchangeBuffer(eb) {
    }

    bool writeToWire(uint8_t* buffer, int buff_len) override {
        return exchangeBuffer.writeData(buffer, buff_len);
    }

    /**
     * For out tests, all our chunks are generated with a repeating byte pattern of 0x00 - 0x0F
     * @param offset the offset into the image file to pull data from
     * @param len the number of bytes to pull from the image file
     * @param buffer a provided buffer to store the data into.
     * @return
     */
    int getImageChunk(uint32_t offset, uint32_t len, void **buffer) override {
        for (int i = 0; i < len; i++)
            ((uint8_t*)*buffer)[i] = (i + offset) % 0x10;
        return len;
    }

    bool handleUpdateResponse(const fwUpdate::payload_t& msg) {
        if (msg.data.update_resp.session_id != cur_session_id)
            return false; // ignore this message, its not for us

        session_status = msg.data.update_resp.status;
        session_total_chunks = msg.data.update_resp.totl_chunks;
        if (session_status == fwUpdate::GOOD_TO_GO) {
            next_chunk_id = 0;
        }

        return true;
    }

    bool handleResendChunk(const fwUpdate::payload_t& msg) {
        if (msg.data.update_resp.session_id != cur_session_id)
            return false; // ignore this message, it's not for us

        next_chunk_id = msg.data.req_resend.chunk_id;
        // the reason doesn't really matter, but we might want to write it to a log or something?
        // TODO: LOG msg.data.req_resend.reason

        sendNextChunk();
        return true;
    }

    bool handleUpdateProgress(const fwUpdate::payload_t& msg) {
        if (msg.data.update_resp.session_id != cur_session_id)
            return false; // ignore this message, it's not for us

        // these messages are purely for UI purposes, they should be handled accordingly.
        // otherwise, we just ignore them

        return true;
    }

    /**
     * called at each step interval; if you put this behind a Scheduled Task, call this method at each interval.
     * This method is primarily used to drive the update process. Unlike the device interface, on the SDK-side, you must call Step,
     * in order to advance the update engine, and transfer image data. Failure to call Step at a regular interval could lead to the
     * device triggering a timeout and aborting the upgrade process.
     * @return the number of times step() has been called since the last request to start an update. This isn't immediately useful, but its nice to know that its actually doing something.
     */
    virtual int step() {
        static uint8_t buffer[2048];
        static uint32_t step_num = 0;
        fwUpdate::payload_t *msg = nullptr;
        void *aux_data = nullptr;
        bool success = false;

        // check if a packet is waiting in the exchange buffer.
        if (exchangeBuffer.dataAvailable() > 0) {
            int buf_len = exchangeBuffer.readData(buffer, sizeof(buffer));
            int msg_len = unpackPayloadNoCopy(buffer, buf_len, &msg, &aux_data);
            if (msg_len > 0) {
#ifdef DEBUG_INFO
                if (msg->hdr.msg_type == fwUpdate::MSG_REQ_RESEND_CHUNK)
                    PRINTF("SDK :: Received MSG %s (Chunk %d)...\n", MSG_TYPES[msg->hdr.msg_type], msg->data.req_resend.chunk_id);
                else if (msg->hdr.msg_type == fwUpdate::MSG_UPDATE_RESP)
                    PRINTF("SDK :: Received MSG %s (%s)...\n", MSG_TYPES[msg->hdr.msg_type], STATUS_TYPES[msg->data.update_resp.status - fwUpdate::ERR_CHECKSUM_MISMATCH]);
                else if (msg->hdr.msg_type == fwUpdate::MSG_UPDATE_FINISHED)
                    PRINTF("SDK :: Received MSG %s (%s)...\n", MSG_TYPES[msg->hdr.msg_type], STATUS_TYPES[msg->data.update_resp.status - fwUpdate::ERR_CHECKSUM_MISMATCH]);
                else
                    PRINTF("SDK :: Received MSG %s...\n", MSG_TYPES[msg->hdr.msg_type]);
#endif
                if (processMessage(*msg))
                    step_num++;
            }

        }
        return step_num;
    }


};

ExchangeBuffer eb(2048); // the exchange buffer used in these tests to simulate back-and-forth data exchanges

TEST(ISFirmwareUpdate, md5_hashing)
{
    const char* data = "The quick brown fox jumped over the lazy dogs.\n";
    uint32_t md5Hash[4];
    ISFirmwareUpdateTestSDK fuSDK(eb);

    fuSDK.resetMd5();
    fuSDK.hashMd5(strlen(data), (uint8_t *) data);
    fuSDK.getCurrentMd5(md5Hash);

    EXPECT_EQ(md5Hash[0], 0x096161b2);
    EXPECT_EQ(md5Hash[1], 0x052c1802);
    EXPECT_EQ(md5Hash[2], 0xac91c781);
    EXPECT_EQ(md5Hash[3], 0x8c584f87);
}

TEST(ISFirmwareUpdate, pack_unpack__req_update)
{
    uint8_t buffer[48];
    ISFirmwareUpdateTestSDK fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_REQ_UPDATE;
    fuMsg.data.req_update.session_id = session_id;
    fuMsg.data.req_update.image_slot = 1;
    fuMsg.data.req_update.chunk_size = 1024;
    fuMsg.data.req_update.file_size = 1234567;
    fuMsg.data.req_update.md5_hash[0] = 0x00010203;
    fuMsg.data.req_update.md5_hash[1] = 0x04050607;
    fuMsg.data.req_update.md5_hash[2] = 0x08090A0B;
    fuMsg.data.req_update.md5_hash[3] = 0x0C0D0E0F;

    int packed_size = fuSDK.packPayload(buffer, sizeof(buffer), fuMsg);
    EXPECT_EQ(packed_size, 36);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_REQ_UPDATE);
    EXPECT_EQ(outMsg->data.req_update.session_id, session_id);
    EXPECT_EQ(outMsg->data.req_update.image_slot, 0x01);
    EXPECT_EQ(outMsg->data.req_update.chunk_size, 1024);
    EXPECT_EQ(outMsg->data.req_update.file_size, 1234567);
    EXPECT_EQ(outMsg->data.req_update.md5_hash[0], 0x00010203);
    EXPECT_EQ(outMsg->data.req_update.md5_hash[1], 0x04050607);
    EXPECT_EQ(outMsg->data.req_update.md5_hash[2], 0x08090A0B);
    EXPECT_EQ(outMsg->data.req_update.md5_hash[3], 0x0C0D0E0F);

    int unpack_len = fuSDK.unpackPayload(buffer, packed_size, fuMsg);
    EXPECT_EQ(unpack_len, 36);

    EXPECT_EQ(outMsg->hdr.target_device, fuMsg.hdr.target_device);
    EXPECT_EQ(outMsg->hdr.msg_type, fuMsg.hdr.msg_type);
    EXPECT_EQ(outMsg->data.req_update.session_id, fuMsg.data.req_update.session_id);
    EXPECT_EQ(outMsg->data.req_update.image_slot, fuMsg.data.req_update.image_slot);
    EXPECT_EQ(outMsg->data.req_update.chunk_size, fuMsg.data.req_update.chunk_size);
    EXPECT_EQ(outMsg->data.req_update.file_size, fuMsg.data.req_update.file_size);
    EXPECT_EQ(outMsg->data.req_update.md5_hash[0], fuMsg.data.req_update.md5_hash[0]);
    EXPECT_EQ(outMsg->data.req_update.md5_hash[1], fuMsg.data.req_update.md5_hash[1]);
    EXPECT_EQ(outMsg->data.req_update.md5_hash[2], fuMsg.data.req_update.md5_hash[2]);
    EXPECT_EQ(outMsg->data.req_update.md5_hash[3], fuMsg.data.req_update.md5_hash[3]);
}

TEST(ISFirmwareUpdate, pack_unpack__update_resp)
{
    uint8_t buffer[48];
    ISFirmwareUpdateTestSDK fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_UPDATE_RESP;
    fuMsg.data.update_resp.session_id = session_id;
    fuMsg.data.update_resp.totl_chunks = 1234;
    fuMsg.data.update_resp.status = fwUpdate::WAITING_FOR_DATA;

    int packed_size = fuSDK.packPayload(buffer, sizeof(buffer), fuMsg);
    EXPECT_EQ(packed_size, 16);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_UPDATE_RESP);
    EXPECT_EQ(outMsg->data.update_resp.session_id, session_id);
    EXPECT_EQ(outMsg->data.update_resp.totl_chunks, 1234);
    EXPECT_EQ(outMsg->data.update_resp.status, fwUpdate::WAITING_FOR_DATA);

    int unpack_len = fuSDK.unpackPayload(buffer, packed_size, fuMsg);
    EXPECT_EQ(unpack_len, 16);

    EXPECT_EQ(outMsg->hdr.target_device, fuMsg.hdr.target_device);
    EXPECT_EQ(outMsg->hdr.msg_type, fuMsg.hdr.msg_type);
    EXPECT_EQ(outMsg->data.update_resp.session_id, fuMsg.data.update_resp.session_id);
    EXPECT_EQ(outMsg->data.update_resp.totl_chunks, fuMsg.data.update_resp.totl_chunks);
    EXPECT_EQ(outMsg->data.update_resp.status, fuMsg.data.update_resp.status);
}

TEST(ISFirmwareUpdate, pack_unpack__chunk)
{
    uint8_t buffer[1024];
    uint8_t chnk_data[512];
    ISFirmwareUpdateTestSDK fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    // initialize chnk_data
    for (int i = 0; i < sizeof(chnk_data); i++)
        chnk_data[i] = (uint8_t)(i % 0x10);

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_UPDATE_CHUNK;
    fuMsg.data.chunk.session_id = session_id;
    fuMsg.data.chunk.chunk_id = 1234;
    fuMsg.data.chunk.data_len = 512;
    fuMsg.data.chunk.data = chnk_data[0];

    int packed_size = fuSDK.packPayload(buffer, sizeof(buffer), fuMsg, chnk_data);
    EXPECT_EQ(packed_size, 527);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_UPDATE_CHUNK);
    EXPECT_EQ(outMsg->data.chunk.session_id, session_id);
    EXPECT_EQ(outMsg->data.chunk.chunk_id, 1234);
    EXPECT_EQ(outMsg->data.chunk.data_len, 512);
    EXPECT_EQ( memcmp((void *)(&outMsg->data.chunk.data), (void *)chnk_data, sizeof(chnk_data)), 0);

    uint8_t aux_data[1024];
    int unpack_len = fuSDK.unpackPayload(buffer, packed_size, fuMsg, aux_data, sizeof(aux_data));
    EXPECT_EQ(unpack_len, 527);

    EXPECT_EQ(outMsg->hdr.target_device, fuMsg.hdr.target_device);
    EXPECT_EQ(outMsg->hdr.msg_type, fuMsg.hdr.msg_type);
    EXPECT_EQ(outMsg->data.chunk.session_id, fuMsg.data.chunk.session_id);
    EXPECT_EQ(outMsg->data.chunk.chunk_id, fuMsg.data.chunk.chunk_id);
    EXPECT_EQ(outMsg->data.chunk.data_len, fuMsg.data.chunk.data_len);
    EXPECT_EQ( memcmp((void *)(&outMsg->data.chunk.data), (void *)chnk_data, sizeof(chnk_data)), 0);
}

TEST(ISFirmwareUpdate, pack_unpack__req_resend)
{
    uint8_t buffer[1024];
    ISFirmwareUpdateTestSDK fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_REQ_RESEND_CHUNK;
    fuMsg.data.req_resend.session_id = session_id;
    fuMsg.data.req_resend.chunk_id = 1234;
    fuMsg.data.req_resend.reason = fwUpdate::REASON_WRITE_ERROR;

    int packed_size = fuSDK.packPayload(buffer, sizeof(buffer), fuMsg);
    EXPECT_EQ(packed_size, 16);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_REQ_RESEND_CHUNK);
    EXPECT_EQ(outMsg->data.req_resend.session_id, session_id);
    EXPECT_EQ(outMsg->data.req_resend.chunk_id, 1234);
    EXPECT_EQ(outMsg->data.req_resend.reason, fwUpdate::REASON_WRITE_ERROR);

    uint8_t aux_data[1024];
    int unpack_len = fuSDK.unpackPayload(buffer, packed_size, fuMsg);
    EXPECT_EQ(unpack_len, 16);

    EXPECT_EQ(outMsg->hdr.target_device, fuMsg.hdr.target_device);
    EXPECT_EQ(outMsg->hdr.msg_type, fuMsg.hdr.msg_type);
    EXPECT_EQ(outMsg->data.req_resend.session_id, fuMsg.data.req_resend.session_id);
    EXPECT_EQ(outMsg->data.req_resend.chunk_id, fuMsg.data.req_resend.chunk_id);
    EXPECT_EQ(outMsg->data.req_resend.reason, fuMsg.data.req_resend.reason);
}

TEST(ISFirmwareUpdate, pack_unpack__progress)
{
    uint8_t buffer[1024];
    uint8_t *progress_msg = (uint8_t *) "Things are definitely progressing!";
    ISFirmwareUpdateTestSDK fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_UPDATE_PROGRESS;
    fuMsg.data.progress.session_id = session_id;
    fuMsg.data.progress.num_chunks = 1234;
    fuMsg.data.progress.totl_chunks = 512;
    fuMsg.data.progress.msg_level = 2;
    fuMsg.data.progress.msg_len = strlen((const char *)progress_msg);

    int packed_size = fuSDK.packPayload(buffer, sizeof(buffer), fuMsg, progress_msg);
    EXPECT_EQ(packed_size, 51);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_UPDATE_PROGRESS);
    EXPECT_EQ(outMsg->data.progress.session_id, session_id);
    EXPECT_EQ(outMsg->data.progress.num_chunks, 1234);
    EXPECT_EQ(outMsg->data.progress.totl_chunks, 512);
    EXPECT_EQ(outMsg->data.progress.msg_level, 2);
    EXPECT_EQ(outMsg->data.progress.msg_len, strlen((const char *)progress_msg));
    EXPECT_EQ( memcmp((void *)(&outMsg->data.progress.message), (void *)progress_msg, strlen((const char *)progress_msg)), 0);

    // Now let's unpack the packed buffer, and verify that all the same data is there.

    uint8_t aux_data[1024];
    int unpack_len = fuSDK.unpackPayload(buffer, packed_size, fuMsg, aux_data, sizeof(aux_data));
    EXPECT_EQ(unpack_len, 51);

    EXPECT_EQ(outMsg->hdr.target_device, fuMsg.hdr.target_device);
    EXPECT_EQ(outMsg->hdr.msg_type, fuMsg.hdr.msg_type);
    EXPECT_EQ(outMsg->data.progress.session_id, fuMsg.data.progress.session_id);
    EXPECT_EQ(outMsg->data.progress.num_chunks, fuMsg.data.progress.num_chunks);
    EXPECT_EQ(outMsg->data.progress.totl_chunks, fuMsg.data.progress.totl_chunks);
    EXPECT_EQ(outMsg->data.progress.msg_level, fuMsg.data.progress.msg_level);
    EXPECT_EQ(outMsg->data.progress.msg_len, fuMsg.data.progress.msg_len);
    EXPECT_EQ( memcmp((void *)(aux_data), (void *)progress_msg, fuMsg.data.progress.msg_len), 0);
}

TEST(ISFirmwareUpdate, exchange__req_update_repl) {
    static uint8_t buffer[2048];
    fwUpdate::payload_t *msg = nullptr;
    void *aux_data = nullptr;

    ISFirmwareUpdateTestSDK fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // Make the request to the device
    fuSDK.requestUpdate(fwUpdate::TARGET_IMX5, 0, 1024,1234567, fake_md5);

    // Force the device-side to pull the message, and respond.
    fuDev.step();

    // for this test, we need to manually pull the data from the exchange buffer, and unpack the response.
    // NOTE: Don't use the following as an example of how to use the API
    if (eb.dataAvailable() > 0) {
        // Data is waiting in the exchange buffer; pull and unpack it for analysis.
        int buf_len = eb.readData(buffer, sizeof(buffer));
        int msg_len = fuSDK.unpackPayloadNoCopy(buffer, buf_len, &msg, &aux_data);
        if (msg_len > 0) {
            // FIXME: Currently, we expect an error -- we need to implement the device-side checks for initialization
            EXPECT_EQ(msg->hdr.target_device, fwUpdate::TARGET_NONE);
            EXPECT_EQ(msg->hdr.msg_type, fwUpdate::MSG_UPDATE_RESP);
            EXPECT_EQ(msg->data.update_resp.session_id, 17767);
            EXPECT_EQ(msg->data.update_resp.totl_chunks, 1206);
            EXPECT_EQ(msg->data.update_resp.status, fwUpdate::GOOD_TO_GO); // any negative value is an error
        }
    }
}

TEST(ISFirmwareUpdate, exchange__success)
{
    static uint8_t buffer[2048];
    fwUpdate::payload_t *msg = nullptr;
    void *aux_data = nullptr;

    ISFirmwareUpdateTestSDK fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // Make the request to the device; the device should expect 8 chunks total
    PRINTF("Requesting firmware update of remote device (should send 8 chunks total (Ids 0-7)...\n");
    fuSDK.requestUpdate(fwUpdate::TARGET_IMX5, 0, 1024,8192, real_md5);

    fuDev.step(); // advance Device, to process the request and send the response
    fuSDK.step(); // advance Host, to process the Device response
    EXPECT_EQ(fuSDK.getSessionStatus(), fwUpdate::GOOD_TO_GO);

    // from here out, this should be normal.
    int i = 0;
    while(fuSDK.getSessionStatus() < fwUpdate::FINISHED) {
        i++;
        if (fuSDK.getSessionStatus() >= fwUpdate::GOOD_TO_GO) {
            fuSDK.sendNextChunk();
        }
        fuDev.step(); // make sure the device-side processes it...
        fuSDK.step();
        EXPECT_EQ(fuSDK.getNextChunkID(), i);

        // check if any errors
        if (fuSDK.getSessionStatus() < fwUpdate::NOT_STARTED)
            break;
    }

    // finally, we should have a status FINISHED
    EXPECT_EQ(fuSDK.getSessionStatus(), fwUpdate::FINISHED);
}

TEST(ISFirmwareUpdate, exchange__req_resend)
{
    static uint8_t buffer[2048];
    fwUpdate::payload_t *msg = nullptr;
    void *aux_data = nullptr;

    ISFirmwareUpdateTestSDK fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // Make the request to the device; the device should expect 8 chunks total
#ifdef DEBUG_INFO
    PRINTF("Requesting firmware update of remote device (should send 8 chunks total (Ids 0-7)...\n");
#endif
    fuSDK.requestUpdate(fwUpdate::TARGET_IMX5, 0, 1024,8192, real_md5);

    fuDev.step(); // advance Device, to process the request and send the response
    fuSDK.step(); // advance Host, to process the Device response
    EXPECT_EQ(fuSDK.getSessionStatus(), fwUpdate::GOOD_TO_GO);
    // but we'll only provide 4...
    for (int i = 0 ; i < 4; i++) {
        // Force the device-side to pull the message, and respond.
        if (fuSDK.getSessionStatus() >= fwUpdate::GOOD_TO_GO) {
            fuSDK.sendNextChunk();
        } else {
            EXPECT_EQ(fuSDK.getSessionStatus(), fwUpdate::GOOD_TO_GO);
        }
        fuDev.step();
        fuSDK.step();
    }

    // then we'll skip the 5th, and send the 6th.  This should cause the device to request that we resend #5.
    // to do this, we'll have to get inside and muck-up the works...  Alternatively, we could have injected a req_resend
    // of any particular chunk, and just ensured that it resent that requested, but this actually tests that the device-
    // side handles the missing chunk as well.
    fuSDK.sendNextChunk();
    // We "muck with it" by twiddling the Exchange Buffer contents, before we let the device-side read it.
    if (eb.dataAvailable() > 0) {
        // Data is waiting in the exchange buffer; pull and unpack it for analysis.
        int buf_len = eb.readData(buffer, sizeof(buffer));
        int msg_len = fuSDK.unpackPayloadNoCopy(buffer, buf_len, &msg, &aux_data);
        if (msg_len > 0) {
#ifdef DEBUG_INFO
            PRINTF("Modifying chunk 4 to report as chunk 6 (skipping 4 and 5).\n");
#endif
            EXPECT_EQ(msg->data.chunk.chunk_id, 4); // if we did it correctly, this should be 4...
            msg->data.chunk.chunk_id = 6; // now let's change it to 6
            eb.writeData(buffer, buf_len); // and then put it back into the exchange buffer
        }
    }
    fuDev.step(); // don't forget to step through each (to process the bad message)

    fuSDK.step(); // and to processing the req_chunk response to the bad message, which should resend the requested chunk 4

    // at this point, our NextChunkID should be 5 (since we resent 4)
    int i = 5;
    EXPECT_EQ(fuSDK.getNextChunkID(), i);

    fuDev.step(); // We need one more device-side step to actually pull the resent chunk from the exchange buffer.

    // from here out, this should be normal.
    while(fuSDK.getSessionStatus() < fwUpdate::FINISHED) {
        i++;
        if (fuSDK.getSessionStatus() >= fwUpdate::GOOD_TO_GO) {
            fuSDK.sendNextChunk();
        }
        fuDev.step(); // make sure the device-side processes it...
        fuSDK.step();
        EXPECT_EQ(fuSDK.getNextChunkID(), i);

        // check if any errors
        if (fuSDK.getSessionStatus() < fwUpdate::NOT_STARTED)
            break;
    }

    // finally, we should have a status FINISHED
    EXPECT_EQ(fuSDK.getSessionStatus(), fwUpdate::FINISHED);
}

TEST(ISFirmwareUpdate, exchange__invalid_checksum)
{
    static uint8_t buffer[2048];
    fwUpdate::payload_t *msg = nullptr;
    void *aux_data = nullptr;

    ISFirmwareUpdateTestSDK fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // Make the request to the device; the device should expect 8 chunks total
    PRINTF("Requesting firmware update of remote device (should send 8 chunks total (Ids 0-7)...\n");
    fuSDK.requestUpdate(fwUpdate::TARGET_IMX5, 0, 1024,8192, fake_md5);

    fuDev.step(); // advance Device, to process the request and send the response
    fuSDK.step(); // advance Host, to process the Device response
    EXPECT_EQ(fuSDK.getSessionStatus(), fwUpdate::GOOD_TO_GO);

    // from here out, this should be normal.
    int i = 0;
    while(fuSDK.getSessionStatus() < fwUpdate::FINISHED) {
        i++;
        if (fuSDK.getSessionStatus() >= fwUpdate::GOOD_TO_GO) {
            fuSDK.sendNextChunk();
        }
        fuDev.step(); // make sure the device-side processes it...
        fuSDK.step();
        EXPECT_EQ(fuSDK.getNextChunkID(), i);

        // check if any errors
        if (fuSDK.getSessionStatus() < fwUpdate::NOT_STARTED)
            break;
    }

    // finally, we should have a status FINISHED
    EXPECT_EQ(fuSDK.getSessionStatus(), fwUpdate::ERR_CHECKSUM_MISMATCH);
}

