//
// Created by kylemallory on 7/31/23.
//

#include <gtest/gtest.h>
#include "gtest_helpers.h"

#include <stdio.h>
#include "../protocol/FirmwareUpdate.h"
#include "miniz.h"
#include "md5.h"

/**
 * This is a really basic FIFO buffer implementation.  It is NOT a ring buffer.
 * Put data into it upto the max buffer size, and you can pull data out upto the amount of data in the buffer.
 * This is intended to be a simple Exchange Buffer - a place where one thing can write data, so another thing can
 * then immediately read that data. There is not effort to maintain data integrity, etc.
 */

#define DEBUG_INFO

static const char* MSG_TYPES[] = { "UNKNOWN", "REQ_RESET", "RESET_RESP", "REQ_UPDATE", "UPDATE_RESP", "UPDATE_CHUNK", "UPDATE_PROGRESS", "REQ_RESEND", "UPDATE_DONE" };

static md5hash_t fake_md5;
static md5hash_t real_md5;

void initialize_md5() {
    fake_md5.dwords[0] = 0x00010203;
    fake_md5.dwords[1] = 0x04050607;
    fake_md5.dwords[2] = 0x08090A0B;
    fake_md5.dwords[3] = 0x0C0D0E0F;
    
    real_md5.dwords[0] = 0x13b16c00;
    real_md5.dwords[1] = 0x427089d8;
    real_md5.dwords[2] = 0x821f472b;
    real_md5.dwords[3] = 0xcb102f3c;
}

class ExchangeBuffer {
public:
    ExchangeBuffer(int size) {
        buff_size = size;
        buffer = new uint8_t[size];
        flush();
    }

    ~ExchangeBuffer() { delete[] buffer; }

    int dataAvailable() { return data_available; }

    int removeData(int start, int len) {
        int trailing =  data_available - (start + len); // (start + len > data_available ? data_available : data_available - (start + len));
        if (trailing > 0) {
            memmove(buffer + start, buffer + start + len, trailing);
            start += trailing;
        }
        memset(buffer + start, 0, len); // zero out the void that we just left (keep our buffer clean)
        data_available -= len;
        return data_available;
    }

    int readData(uint8_t *data, int max_data, int offset = 0) {
        fwUpdate::payload_t *payload;
        uint8_t *aux_data, *payload_start = buffer + offset;
        int payload_len = fwUpdate::FirmwareUpdateBase::fwUpdate_mapBufferToPayload(payload_start, &payload, (void **)&aux_data);

        memcpy(data, payload_start, payload_len);
        removeData(buffer - payload_start, payload_len);
        return payload_len;
    }

    bool isNextDataForTarget(fwUpdate::target_t target) {
        fwUpdate::payload_t *payload = (fwUpdate::payload_t *)buffer;
        return (payload->hdr.target_device == target);
    }

    int getNextDataOffsetForTarget(fwUpdate::target_t target) {
        fwUpdate::payload_t *payload;
        uint8_t *aux_data, *payload_start = buffer;
        int offset = 0, payload_len = 0;
        while (offset < data_available) {
            payload_len = fwUpdate::FirmwareUpdateBase::fwUpdate_mapBufferToPayload(payload_start, &payload, (void **)&aux_data);
            if (payload->hdr.msg_type == fwUpdate::MSG_UNKNOWN)
                return -1; // FIXME: You should investigate why this is happening... yes, YOU!!
            if (payload->hdr.target_device == target)
                return offset;
            offset += payload_len;
        }
        return -1;
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
 * This class can probably be removed.  Its not used, but there are foreseeable unit-test cases which may benefit from it.
 *   ** It has NOT been tested, and probably will need some additional work to get it functional **
 */
class Fifo {
private:
    int capacity;        // Total capacity of the FIFO
    int size;            // Current number of items in the FIFO
    int frontIndex;      // Index of the front item in the FIFO
    int rearIndex;       // Index of the rear item in the FIFO
    unsigned char** buffer; // Array of dynamically allocated byte arrays

public:
    Fifo(int capacity) : capacity(capacity), size(0), frontIndex(0), rearIndex(0) {
        buffer = new unsigned char*[capacity];
        for (int i = 0; i < capacity; ++i) {
            buffer[i] = new unsigned char[256]; // Each array can hold up to 256 bytes
        }
    }

    ~Fifo() {
        for (int i = 0; i < capacity; ++i) {
            delete[] buffer[i];
        }
        delete[] buffer;
    }

    bool isEmpty() const {
        return size == 0;
    }

    bool isFull() const {
        return size == capacity;
    }

    void enqueue(const unsigned char* data, int length) {
        if (isFull()) {
            std::cerr << "FIFO is full. Cannot enqueue more data." << std::endl;
            return;
        }

        memcpy(buffer[rearIndex], data, length);
        rearIndex = (rearIndex + 1) % capacity;
        ++size;
    }

    bool dequeue(unsigned char* data, int& length) {
        if (isEmpty()) {
            std::cerr << "FIFO is empty. Cannot dequeue data." << std::endl;
            return false;
        }

        length = strlen(reinterpret_cast<const char*>(buffer[frontIndex]));
        memcpy(data, buffer[frontIndex], length);

        frontIndex = (frontIndex + 1) % capacity;
        --size;
        return true;
    }
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

    ISFirmwareUpdateTestDev(ExchangeBuffer& eb) : FirmwareUpdateDevice(fwUpdate::TARGET_IMX5), exchangeBuffer(eb) {
    }

    bool sendProgressUpdates = true;

    int fwUpdate_performReset(fwUpdate::target_t target_id, fwUpdate::reset_flags_e reset_flags) { return 0; };

    bool fwUpdate_queryVersionInfo(fwUpdate::target_t target_id, dev_info_t& devInfo) { return false; };

    // this initializes the system to begin receiving firmware image chunks for the target device, image slot and image size
    fwUpdate::update_status_e fwUpdate_startUpdate(const fwUpdate::payload_t& msg) {

        // query information from the system about the requested firmware slot
        if (msg.data.req_update.image_slot != 0)
            return fwUpdate::ERR_INVALID_SLOT;

        uint32_t slot_size = 0x800000;
        if (msg.data.req_update.file_size > slot_size)
            return fwUpdate::ERR_NOT_ENOUGH_MEMORY;

        // validate that the system can handle the requested chunk size (it should not exceed the RX buffer, and it should be multiples of 16 bytes (because I said so).
        if (msg.data.req_update.chunk_size > MaxChunkSize)
            return fwUpdate::ERR_MAX_CHUNK_SIZE;

        return fwUpdate::READY;
    }

    // writes the indicated block of data (of len bytes) to the target and device-specific image slot, and with the specified offset
    fwUpdate::update_status_e fwUpdate_writeImageChunk(fwUpdate::target_t target_id, int slot_id, int offset, int len, uint8_t *data) {
        return fwUpdate::IN_PROGRESS;
    }

    // this marks the finish of the upgrade, that all image bytes have been received, the md5 sum passed, and the device can complete the requested upgrade, and perform any device-specific finalization
    fwUpdate::update_status_e  fwUpdate_finishUpdate(fwUpdate::target_t target_id, int slot_id, int flags) {
        // check that our md5 matches.  Return >0 is we're error free.
        if (memcmp(&md5Context.state, &session_md5, sizeof(md5hash_t)) == 0)
            return fwUpdate::FINISHED;

        return fwUpdate::ERR_CHECKSUM_MISMATCH;
    }

    // called internally to process a packed payload, ready to be put on the wire.
    bool fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t* buffer, int buff_len) {
        return exchangeBuffer.writeData(buffer, buff_len);
    }

    /**
     * called at each step interval; if you put this behind a Scheduled Task/Thread, call this method at each interval.
     * This method is primarily used to perform routine maintenance, like checking if the init process is complete, or to give out status update, etc.
     * If you don't call step() things should still generally work, but it probably won't seem very responsive.
     * @return the number of times step() has been called since the last request to start an update. This isn't immediately useful, but its nice to know that its actually doing something.
     */
    virtual bool fwUpdate_step(fwUpdate::msg_types_e msg_type = fwUpdate::MSG_UNKNOWN, bool processed = false) {
        return true;
    }

    void pullAndProcessNextMessage() {
        uint8_t buffer[4096];
        int offset = 0;

        if (exchangeBuffer.dataAvailable() > 0) {
            do {
                offset = exchangeBuffer.getNextDataOffsetForTarget(fwUpdate_getCurrentTarget());
                if (offset >= 0) {
                    // Data is waiting in the exchange buffer for us; let's pull and unpack it for analysis.
                    int buf_len = exchangeBuffer.readData(buffer, sizeof(buffer), offset);
                    fwUpdate_processMessage(buffer, buf_len);
                }
            } while ((offset >= 0) && (exchangeBuffer.dataAvailable() > 0)); // keep pulling all data that is meant for us
        }
    }

    int GetNextExpectedChunk() { return last_chunk_id + 1; }
};

class ISFirmwareUpdateTestHost : public fwUpdate::FirmwareUpdateHost {
public:
    ExchangeBuffer& exchangeBuffer;

    explicit ISFirmwareUpdateTestHost(ExchangeBuffer& eb) : FirmwareUpdateHost(), exchangeBuffer(eb) {
    }

    bool fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t* buffer, int buff_len) override {
        return exchangeBuffer.writeData(buffer, buff_len);
    }

    /**
     * For out tests, all our chunks are generated with a repeating byte pattern of 0x00 - 0x0F
     * @param offset the offset into the image file to pull data from
     * @param len the number of bytes to pull from the image file
     * @param buffer a provided buffer to store the data into.
     * @return
     */
    int fwUpdate_getImageChunk(uint32_t offset, uint32_t len, void **buffer) override {
        for (uint32_t i = 0; i < len; i++)
            ((uint8_t*)*buffer)[i] = (i + offset) % 0x10;
        return len;
    }

    /**
     * Just used for unit-tests, calculates the actual checksum for the number and size of chunks specified.
     * @param chunk_size
     * @param num_chunks
     * @param hashOut
     * @return
     */
    void calcChecksumForTest(int image_size, int chunk_size, md5hash_t& hashOut) {
        md5_init(md5Context);
        void* buf = malloc(chunk_size);

        if (buf) {
            int num_chunks = (uint16_t) ceil((float)image_size / (float)chunk_size);
            for (int i = 0; i < num_chunks; i++) {
                uint16_t mod_size = (image_size % chunk_size);
                uint16_t expected_size = ((i == num_chunks-1) && (mod_size != 0)) ? mod_size : chunk_size;

                fwUpdate_getImageChunk(i * chunk_size, expected_size, &buf);
                md5_update(md5Context, (uint8_t *)buf, expected_size);
            }
            // md5_final()
            hashOut = md5Context.state;
        }
        md5_final(md5Context, hashOut);
    }

    bool fwUpdate_handleVersionResponse(const fwUpdate::payload_t& msg) {
        return true;
    }


    bool fwUpdate_handleUpdateResponse(const fwUpdate::payload_t& msg) {
        if (msg.data.update_resp.session_id != session_id)
            return false; // ignore this message, its not for us

        session_status = msg.data.update_resp.status;
        session_total_chunks = msg.data.update_resp.totl_chunks;
        if (session_status == fwUpdate::READY) {
            next_chunk_id = 0;
        }

        return true;
    }

    bool fwUpdate_handleResendChunk(const fwUpdate::payload_t& msg) {
        if (msg.data.req_resend.session_id != session_id)
            return false; // ignore this message, it's not for us

        next_chunk_id = msg.data.req_resend.chunk_id;
        // the reason doesn't really matter, but we might want to write it to a log or something?
        // TODO: LOG msg.data.req_resend.reason

        fwUpdate_sendNextChunk();
        return true;
    }

    bool fwUpdate_handleUpdateProgress(const fwUpdate::payload_t& msg) {
        if (msg.data.progress.session_id != session_id)
            return false; // ignore this message, it's not for us

        int percent = (int)(((msg.data.progress.num_chunks)/(float)(msg.data.progress.totl_chunks)*100) + 0.5f);
        PRINTF("SDK :: Progress %d/%d (%d%%) :: [%d,%d] %s\n", msg.data.progress.num_chunks, msg.data.progress.totl_chunks, percent, msg.data.progress.msg_level, msg.data.progress.msg_len, (const char *)&msg.data.progress.message);
        return true;
    }

    bool fwUpdate_handleDone(const fwUpdate::payload_t& msg) {
        if (msg.data.update_resp.session_id != session_id)
            return false; // ignore this message, it's not for us
        return true;
    }


    /**
     * called at each step interval; if you put this behind a Scheduled Task, call this method at each interval.
     * This method is primarily used to drive the update process. Unlike the device interface, on the SDK-side, you must call Step,
     * in order to advance the update engine, and transfer image data. Failure to call Step at a regular interval could lead to the
     * device triggering a timeout and aborting the upgrade process.
     * @return the message type, if any that was most recently processed.
     */
    virtual bool fwUpdate_step(fwUpdate::msg_types_e msg_type = fwUpdate::MSG_UNKNOWN, bool processed = false) {
        static uint8_t buffer[4096];
        fwUpdate::payload_t *msg = nullptr;
        void *aux_data = nullptr;
        fwUpdate::msg_types_e out = fwUpdate::MSG_UNKNOWN;

        // check if a packet is waiting in the exchange buffer.
        if (exchangeBuffer.dataAvailable() > 0) {
            int offset = 0;
            do {
                offset = exchangeBuffer.getNextDataOffsetForTarget(fwUpdate::TARGET_HOST);
                if (offset >= 0) {
                    // Data is waiting in the exchange buffer for us; let's pull and unpack it for analysis.
                    int buf_len = exchangeBuffer.readData(buffer, sizeof(buffer), offset);
                    int msg_len = fwUpdate_mapBufferToPayload(buffer, &msg, &aux_data);
                    if (msg_len > 0) {
                        if (fwUpdate_processMessage(buffer, buf_len))
                            out = msg->hdr.msg_type;
                    }
                }
            } while ((offset >= 0) && (exchangeBuffer.dataAvailable() > 0)); // keep pulling all data that is meant for us

#ifdef DEBUG_INFO
            if (msg->hdr.msg_type == fwUpdate::MSG_REQ_RESEND_CHUNK) {
                PRINTF("SDK :: Received MSG %s (Chunk %d)...\n", MSG_TYPES[msg->hdr.msg_type], msg->data.req_resend.chunk_id);
            } else {
                PRINTF("SDK :: Received MSG %s (%s)...\n", MSG_TYPES[msg->hdr.msg_type], getSessionStatusName());
            }
#endif
        }
        return true;
    }
};

ExchangeBuffer eb(2048); // the exchange buffer used in these tests to simulate back-and-forth data exchanges

TEST(ISFirmwareUpdate, pack_unpack__req_update)
{
    uint8_t buffer[48];
    ISFirmwareUpdateTestHost fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_REQ_UPDATE;
    fuMsg.data.req_update.session_id = session_id;
    fuMsg.data.req_update.image_slot = 1;
    fuMsg.data.req_update.image_flags = 5;
    fuMsg.data.req_update.file_size = 1234567;
    fuMsg.data.req_update.chunk_size = 1024;
    fuMsg.data.req_update.progress_rate = 789;
    fuMsg.data.req_update.md5_hash.dwords[0] = 0x00010203;
    fuMsg.data.req_update.md5_hash.dwords[1] = 0x04050607;
    fuMsg.data.req_update.md5_hash.dwords[2] = 0x08090A0B;
    fuMsg.data.req_update.md5_hash.dwords[3] = 0x0C0D0E0F;

    int packed_size = fuSDK.fwUpdate_packPayload(buffer, sizeof(buffer), fuMsg);
    EXPECT_EQ(packed_size, 36);

    EXPECT_EQ(*(uint32_t*)(buffer+0), fwUpdate::TARGET_GPX1);       // uint32_t (0 + 4)
    EXPECT_EQ(*(uint32_t*)(buffer+4), fwUpdate::MSG_REQ_UPDATE);    // uint32_t (4 + 4)
    EXPECT_EQ(*(uint16_t*)(buffer+8), session_id);                  // uint16_t (8 + 2)
    EXPECT_EQ(*(uint8_t*)(buffer+10), 1);                          // uint16_t (10 + 1)
    EXPECT_EQ(*(uint8_t*)(buffer+11), 5);                          // uint16_t (11 + 1)
    EXPECT_EQ(*(uint32_t*)(buffer+12), 1234567);                    // uint32_t (12 + 4)
    EXPECT_EQ(*(uint16_t*)(buffer+16), 1024);                       // uint16_t (16 + 2)
    EXPECT_EQ(*(uint16_t*)(buffer+18), 789);                        // uint16_t (18 + 2)
    EXPECT_EQ(*(uint32_t*)(buffer+20), 0x00010203);                 // uint32_t (20 + 4)
    EXPECT_EQ(*(uint32_t*)(buffer+24), 0x04050607);                 // uint32_t (24 + 4)
    EXPECT_EQ(*(uint32_t*)(buffer+28), 0x08090A0B);                 // uint32_t (28 + 4)
    EXPECT_EQ(*(uint32_t*)(buffer+32), 0x0C0D0E0F);                 // uint32_t (32 + 4) = total length is 36

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_REQ_UPDATE);
    EXPECT_EQ(outMsg->data.req_update.session_id, session_id);
    EXPECT_EQ(outMsg->data.req_update.image_slot, 0x01);
    EXPECT_EQ(outMsg->data.req_update.image_flags, 0x05);
    EXPECT_EQ(outMsg->data.req_update.file_size, 1234567);
    EXPECT_EQ(outMsg->data.req_update.chunk_size, 1024);
    EXPECT_EQ(outMsg->data.req_update.md5_hash.dwords[0], 0x00010203);
    EXPECT_EQ(outMsg->data.req_update.md5_hash.dwords[1], 0x04050607);
    EXPECT_EQ(outMsg->data.req_update.md5_hash.dwords[2], 0x08090A0B);
    EXPECT_EQ(outMsg->data.req_update.md5_hash.dwords[3], 0x0C0D0E0F);

    int unpack_len = fuSDK.fwUpdate_unpackPayload(buffer, packed_size, fuMsg);
    EXPECT_EQ(unpack_len, 36);

    EXPECT_EQ(outMsg->hdr.target_device, fuMsg.hdr.target_device);
    EXPECT_EQ(outMsg->hdr.msg_type, fuMsg.hdr.msg_type);
    EXPECT_EQ(outMsg->data.req_update.session_id, fuMsg.data.req_update.session_id);
    EXPECT_EQ(outMsg->data.req_update.image_slot, fuMsg.data.req_update.image_slot);
    EXPECT_EQ(outMsg->data.req_update.chunk_size, fuMsg.data.req_update.chunk_size);
    EXPECT_EQ(outMsg->data.req_update.file_size, fuMsg.data.req_update.file_size);
    EXPECT_EQ(outMsg->data.req_update.md5_hash.dwords[0], fuMsg.data.req_update.md5_hash.dwords[0]);
    EXPECT_EQ(outMsg->data.req_update.md5_hash.dwords[1], fuMsg.data.req_update.md5_hash.dwords[1]);
    EXPECT_EQ(outMsg->data.req_update.md5_hash.dwords[2], fuMsg.data.req_update.md5_hash.dwords[2]);
    EXPECT_EQ(outMsg->data.req_update.md5_hash.dwords[3], fuMsg.data.req_update.md5_hash.dwords[3]);
}

TEST(ISFirmwareUpdate, pack_unpack__update_resp)
{
    uint8_t buffer[48];
    ISFirmwareUpdateTestHost fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_UPDATE_RESP;
    fuMsg.data.update_resp.session_id = session_id;
    fuMsg.data.update_resp.totl_chunks = 1234;
    fuMsg.data.update_resp.status = fwUpdate::IN_PROGRESS;

    int packed_size = fuSDK.fwUpdate_packPayload(buffer, sizeof(buffer), fuMsg);
    EXPECT_EQ(packed_size, 14);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_UPDATE_RESP);
    EXPECT_EQ(outMsg->data.update_resp.session_id, session_id);
    EXPECT_EQ(outMsg->data.update_resp.totl_chunks, 1234);
    EXPECT_EQ(outMsg->data.update_resp.status, fwUpdate::IN_PROGRESS);

    int unpack_len = fuSDK.fwUpdate_unpackPayload(buffer, packed_size, fuMsg);
    EXPECT_EQ(unpack_len, 14);

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
    ISFirmwareUpdateTestHost fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    // initialize chnk_data
    for (size_t i = 0; i < sizeof(chnk_data); i++)
        chnk_data[i] = (uint8_t)(i % 0x10);

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_UPDATE_CHUNK;
    fuMsg.data.chunk.session_id = session_id;
    fuMsg.data.chunk.chunk_id = 1234;
    fuMsg.data.chunk.data_len = 512;
    fuMsg.data.chunk.data = chnk_data[0];

    int packed_size = fuSDK.fwUpdate_packPayload(buffer, sizeof(buffer), fuMsg, chnk_data);
    EXPECT_EQ(packed_size, 526);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_UPDATE_CHUNK);
    EXPECT_EQ(outMsg->data.chunk.session_id, session_id);
    EXPECT_EQ(outMsg->data.chunk.chunk_id, 1234);
    EXPECT_EQ(outMsg->data.chunk.data_len, 512);
    EXPECT_EQ( memcmp((void *)(&outMsg->data.chunk.data), (void *)chnk_data, sizeof(chnk_data)), 0);

    uint8_t aux_data[1024];
    int unpack_len = fuSDK.fwUpdate_unpackPayload(buffer, packed_size, fuMsg, aux_data, sizeof(aux_data));
    EXPECT_EQ(unpack_len, 526);

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
    ISFirmwareUpdateTestHost fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_REQ_RESEND_CHUNK;
    fuMsg.data.req_resend.session_id = session_id;
    fuMsg.data.req_resend.chunk_id = 1234;
    fuMsg.data.req_resend.reason = fwUpdate::REASON_WRITE_ERROR;

    int packed_size = fuSDK.fwUpdate_packPayload(buffer, sizeof(buffer), fuMsg);
    EXPECT_EQ(packed_size, 14);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_REQ_RESEND_CHUNK);
    EXPECT_EQ(outMsg->data.req_resend.session_id, session_id);
    EXPECT_EQ(outMsg->data.req_resend.chunk_id, 1234);
    EXPECT_EQ(outMsg->data.req_resend.reason, fwUpdate::REASON_WRITE_ERROR);

    //uint8_t aux_data[1024];
    int unpack_len = fuSDK.fwUpdate_unpackPayload(buffer, packed_size, fuMsg);
    EXPECT_EQ(unpack_len, 14);

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
    ISFirmwareUpdateTestHost fuSDK(eb);
    fwUpdate::payload_t fuMsg;

    uint16_t session_id = 0x7F7F; // (uint16_t)random();
    memset(buffer, 0, sizeof(buffer));

    fuMsg.hdr.target_device = fwUpdate::TARGET_GPX1;
    fuMsg.hdr.msg_type = fwUpdate::MSG_UPDATE_PROGRESS;
    fuMsg.data.progress.session_id = session_id;
    fuMsg.data.progress.status = fwUpdate::NOT_STARTED;
    fuMsg.data.progress.num_chunks = 1234;
    fuMsg.data.progress.totl_chunks = 512;
    fuMsg.data.progress.msg_level = 2;
    fuMsg.data.progress.msg_len = strlen((const char *)progress_msg);

    int packed_size = fuSDK.fwUpdate_packPayload(buffer, sizeof(buffer), fuMsg, progress_msg);
    EXPECT_EQ(packed_size, 52);

    // If we've done our jobs right, we should be able to cast the payload buffer, back to a payload_t*, and access all the same data

    fwUpdate::payload_t *outMsg = (fwUpdate::payload_t *)&buffer;

    EXPECT_EQ(outMsg->hdr.target_device, fwUpdate::TARGET_GPX1);
    EXPECT_EQ(outMsg->hdr.msg_type, fwUpdate::MSG_UPDATE_PROGRESS);
    EXPECT_EQ(outMsg->data.progress.session_id, session_id);
    EXPECT_EQ(outMsg->data.progress.status, fwUpdate::NOT_STARTED);
    EXPECT_EQ(outMsg->data.progress.num_chunks, 1234);
    EXPECT_EQ(outMsg->data.progress.totl_chunks, 512);
    EXPECT_EQ(outMsg->data.progress.msg_level, 2);
    EXPECT_EQ(outMsg->data.progress.msg_len, strlen((const char *)progress_msg));
    EXPECT_EQ( memcmp((void *)(&outMsg->data.progress.message), (void *)progress_msg, strlen((const char *)progress_msg)), 0);

    // Now let's unpack the packed buffer, and verify that all the same data is there.

    uint8_t aux_data[1024];
    int unpack_len = fuSDK.fwUpdate_unpackPayload(buffer, packed_size, fuMsg, aux_data, sizeof(aux_data));
    EXPECT_EQ(unpack_len, 52);

    EXPECT_EQ(outMsg->hdr.target_device, fuMsg.hdr.target_device);
    EXPECT_EQ(outMsg->hdr.msg_type, fuMsg.hdr.msg_type);
    EXPECT_EQ(outMsg->data.progress.session_id, fuMsg.data.progress.session_id);
    EXPECT_EQ(outMsg->data.progress.num_chunks, fuMsg.data.progress.num_chunks);
    EXPECT_EQ(outMsg->data.progress.totl_chunks, fuMsg.data.progress.totl_chunks);
    EXPECT_EQ(outMsg->data.progress.msg_level, fuMsg.data.progress.msg_level);
    EXPECT_EQ(outMsg->data.progress.msg_len, fuMsg.data.progress.msg_len);
    EXPECT_EQ( memcmp((void *)(aux_data), (void *)progress_msg, fuMsg.data.progress.msg_len), 0);
}

TEST(ISFirmwareUpdate, exchange__req_update_repl) 
{
    initialize_md5();
    static uint8_t buffer[5000];
    fwUpdate::payload_t *msg = nullptr;
    void *aux_data = nullptr;

    eb.flush();
    ISFirmwareUpdateTestHost fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // don't send progress updates for this test.
    fuDev.sendProgressUpdates = false;

    // Make the request to the device
    fuSDK.fwUpdate_requestUpdate(fwUpdate::TARGET_IMX5, 0, 0, 512,1234567, fake_md5);

    fuDev.pullAndProcessNextMessage();

    // for this test, we need to manually pull the data from the exchange buffer, and unpack the response.
    // NOTE: Don't use the following as an example of how to use the API
    EXPECT_GT(eb.dataAvailable(), 0);
    if (eb.dataAvailable() > 0) {
        // Data is waiting in the exchange buffer; pull and unpack it for analysis.
        int buf_len = eb.readData(buffer, sizeof(buffer));
        EXPECT_EQ(buf_len, 14);
        int msg_len = fuSDK.fwUpdate_mapBufferToPayload(buffer, &msg, &aux_data);
        if (msg_len > 0) {
            if (msg->hdr.msg_type == fwUpdate::MSG_UPDATE_RESP) {
                // FIXME: Currently, we expect an error -- we need to implement the device-side checks for initialization
                EXPECT_EQ(msg->hdr.target_device, fwUpdate::TARGET_HOST);
                EXPECT_EQ(msg->hdr.msg_type, fwUpdate::MSG_UPDATE_RESP);
                EXPECT_EQ(msg->data.update_resp.session_id, fuSDK.fwUpdate_getSessionID());
                uint16_t numChunks = ceil(fuSDK.fwUpdate_getImageSize() / (float)fuSDK.fwUpdate_getChunkSize());
                EXPECT_EQ(msg->data.update_resp.totl_chunks, numChunks);
                EXPECT_EQ(msg->data.update_resp.status, fwUpdate::READY); // any negative value is an error
            }
        }
    }
}

TEST(ISFirmwareUpdate, exchange__req_resend)
{
    initialize_md5();
    static uint8_t buffer[5000];
    fwUpdate::payload_t *msg = nullptr;
    void *aux_data = nullptr;

    eb.flush();
    ISFirmwareUpdateTestHost fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // don't send progress updates for this test.
    fuDev.sendProgressUpdates = false;

    // Make the request to the device; the device should expect 8 chunks total
#ifdef DEBUG_INFO
    PRINTF("Requesting firmware update of remote device (should send 8 chunks total (Ids 0-7)...\n");
#endif
    int imageSize = fuSDK.MaxChunkSize * 8;
    fuSDK.calcChecksumForTest(imageSize, fuSDK.MaxChunkSize, real_md5);
    fuSDK.fwUpdate_requestUpdate(fwUpdate::TARGET_IMX5, 0, 0, 512, imageSize, real_md5);

    fuDev.pullAndProcessNextMessage(); // make sure the device-side processes it...
    fuSDK.fwUpdate_step(); // advance Host, to process the Device response

    EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::READY);
    // but we'll only provide 4...
    for (int i = 0 ; i < 4; i++) {
        // Force the device-side to pull the message, and respond.
        if ((fuSDK.fwUpdate_getSessionStatus() == fwUpdate::READY) || (fuSDK.fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS)) {
            fuSDK.fwUpdate_sendNextChunk();
        } else {
            EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::READY);
        }
        fuDev.pullAndProcessNextMessage(); // make sure the device-side processes it...
        fuSDK.fwUpdate_step();
    }

    // then we'll skip the 5th, and send the 6th.  This should cause the device to request that we resend #5.
    // to do this, we'll have to get inside and muck-up the works...  Alternatively, we could have injected a req_resend
    // of any particular chunk, and just ensured that it resent that requested, but this actually tests that the device-
    // side handles the missing chunk as well.
    fuSDK.fwUpdate_sendNextChunk();
    // We "muck with it" by twiddling the Exchange Buffer contents, before we let the device-side read it.
    if (eb.dataAvailable() > 0) {
        // Data is waiting in the exchange buffer; pull and unpack it for analysis.
        int buf_len = eb.readData(buffer, sizeof(buffer));
        int msg_len = fuSDK.fwUpdate_mapBufferToPayload(buffer, &msg, &aux_data);
        if (msg_len > 0) {
#ifdef DEBUG_INFO
            PRINTF("Modifying chunk 4 to report as chunk 6 (skipping 4 and 5).\n");
#endif
            EXPECT_EQ(msg->data.chunk.chunk_id, 4); // if we did it correctly, this should be 4...
            msg->data.chunk.chunk_id = 6; // now let's change it to 6
            eb.writeData(buffer, buf_len); // and then put it back into the exchange buffer
        }
    }

    // BE CAREFUL:  This following steps are time-critical!!  If you are debugging, you may get a failure here because fuDev.pullAndProcessNextMessage() may send a PROGRESS message if
    // you hold too long in a breakpoint.  If it does, and it shows up after the RESEND message, this line will fail.  Under normal circumstances, it should work fine, but YOU HAVE NOW
    // BEEN WARNED to avoid this rabbit hole!  If it becomes an issue, one option would be to implement a fuSDK.step(1) which would force step() to only process n-number of messages.
    // Alternatively, we could update the FirmwareUpdate API to allow disabling of sending PROGRESS messages entirely (set progressInterval to -1, in the request, for example).

    fuDev.pullAndProcessNextMessage(); // will cause the Device to pull the bad chunk, which should respond with a RESEND_CHUNK
    EXPECT_EQ(fuSDK.fwUpdate_step(), true); // and step() to process the RESEND_CHUNK response to the bad message, which should resend the requested chunk 4
    EXPECT_EQ(fuSDK.fwUpdate_getNextChunkID(), 5); // at this point, our NextChunkID should be 5 (since we resent 4)

    // and now we resume with the remaining chunks
    while(fuSDK.fwUpdate_getSessionStatus() < fwUpdate::FINISHED) {
        if ((fuSDK.fwUpdate_getSessionStatus() == fwUpdate::READY) || (fuSDK.fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS)) {
            fuSDK.fwUpdate_sendNextChunk();
        }
        fuDev.pullAndProcessNextMessage(); // make sure the device-side processes it...
        fuSDK.fwUpdate_step(); // process all incoming messages
        EXPECT_EQ(fuSDK.fwUpdate_getNextChunkID(), fuDev.GetNextExpectedChunk());

        // check if any errors
        if (fuSDK.fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED)
            break;
    }

    // finally, we should have a status FINISHED
    EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::FINISHED) << "Actual result was: " << fuSDK.fwUpdate_getSessionStatusName() << std::endl;
}

TEST(ISFirmwareUpdate, exchange__invalid_checksum)
{
    initialize_md5();
    static uint8_t buffer[2048];

    eb.flush();
    ISFirmwareUpdateTestHost fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // Make the request to the device; the device should expect 8 chunks total
    PRINTF("Requesting firmware update of remote device (should send 8 chunks total (Ids 0-7)...\n");
    int imageSize = fuSDK.MaxChunkSize * 8;
    fuSDK.calcChecksumForTest(imageSize, fuSDK.MaxChunkSize, real_md5);
    fuSDK.fwUpdate_requestUpdate(fwUpdate::TARGET_IMX5, 0, 0, 512, imageSize, fake_md5);

    fuDev.pullAndProcessNextMessage(); // advance Device, to process the request and send the response
    fuSDK.fwUpdate_step(); // advance Host, to process the Device response
    EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::READY);

    // from here out, this should be normal.
    int i = 0;
    while(fuSDK.fwUpdate_getSessionStatus() < fwUpdate::FINISHED) {
        i++;
        if (fuSDK.fwUpdate_getSessionStatus() >= fwUpdate::READY) {
            fuSDK.fwUpdate_sendNextChunk();
        }

        if (i == 4) {
            // SETUP THE TEST :::
            // On the 4rth chunk, let's modify the exchange buffer content..
            ASSERT_NE(eb.dataAvailable(), 0); // fail the test if the buffer is empty
            int buf_len = eb.readData(buffer, sizeof(buffer));
            buffer[20] = 129; // just some random data at some random offset - only one byte should do!
            eb.writeData(buffer, buf_len); // and then put it back into the exchange buffer
            fuDev.pullAndProcessNextMessage(); // will cause the Device to pull the bad chunk, which should respond with a RESEND_CHUNK
        } else {
            fuDev.pullAndProcessNextMessage(); // make sure the device-side processes it...
        }
        fuSDK.fwUpdate_step();
        EXPECT_EQ(fuSDK.fwUpdate_getNextChunkID(), i);

        // check if any errors
        if (fuSDK.fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED)
            break;
    }

    // finally, we should have a status FINISHED
    EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::ERR_CHECKSUM_MISMATCH);
}

/**
 * This tests checks to make sure we have a complete, successful firmware update exchange from beginning to end.
 */
TEST(ISFirmwareUpdate, exchange__success)
{
    initialize_md5();
    eb.flush();
    ISFirmwareUpdateTestHost fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // don't send progress updates for this test.
    fuDev.sendProgressUpdates = false;

    // Make the request to the device; the device should expect 8 chunks total
    PRINTF("Requesting firmware update of remote device (should send 8 chunks total (Ids 0-7)...\n");
    int imageSize = fuSDK.MaxChunkSize * 8;
    fuSDK.calcChecksumForTest(imageSize, fuSDK.MaxChunkSize, real_md5);
    fuSDK.fwUpdate_requestUpdate(fwUpdate::TARGET_IMX5, 0, 0, 512, imageSize, real_md5);

    fuDev.pullAndProcessNextMessage();
    fuSDK.fwUpdate_step(); // advance Host, to process the Device response

    EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::READY);

    // from here out, this should be normal.
    while((fuSDK.fwUpdate_getSessionStatus() < fwUpdate::FINISHED) && (fuDev.GetNextExpectedChunk() != -1)) {
        if ((fuSDK.fwUpdate_getSessionStatus() == fwUpdate::READY) || (fuSDK.fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS)) {
            fuSDK.fwUpdate_sendNextChunk();
        }

        fuDev.pullAndProcessNextMessage();
        EXPECT_EQ(fuSDK.fwUpdate_getNextChunkID(), fuDev.GetNextExpectedChunk());

        fuSDK.fwUpdate_step();

        // check if any errors
        if (fuSDK.fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED)
            break;
    }

    // finally, we should have a status FINISHED
    EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::FINISHED);
}

/**
 * This tests checks to make sure we have a complete, successful firmware update exchange from beginning to end.
 */
TEST(ISFirmwareUpdate, exchange__success_non_chunk_boundary)
{
    initialize_md5();
    ISFirmwareUpdateTestHost fuSDK(eb);
    ISFirmwareUpdateTestDev fuDev(eb);

    // don't send progress updates for this test.
    fuDev.sendProgressUpdates = false;

    // Make the request to the device; the device should expect 8 chunks total
    PRINTF("Requesting firmware update of remote device (should send 8 chunks total (Ids 0-7)...\n");
    int imageSize = fuSDK.MaxChunkSize * 8.5;
    fuSDK.calcChecksumForTest(imageSize, fuSDK.MaxChunkSize, real_md5);
    fuSDK.fwUpdate_requestUpdate(fwUpdate::TARGET_IMX5, 0, 0, 512, imageSize, real_md5);

    fuDev.pullAndProcessNextMessage();
    fuSDK.fwUpdate_step(); // advance Host, to process the Device response

    EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::READY);

    // from here out, this should be normal.
    while((fuSDK.fwUpdate_getSessionStatus() < fwUpdate::FINISHED) && (fuDev.GetNextExpectedChunk() != 65536)) {
        if ((fuSDK.fwUpdate_getSessionStatus() == fwUpdate::READY) || (fuSDK.fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS)) {
            fuSDK.fwUpdate_sendNextChunk();
        }

        fuDev.pullAndProcessNextMessage();
        EXPECT_EQ(fuSDK.fwUpdate_getNextChunkID(), fuDev.GetNextExpectedChunk());

        fuSDK.fwUpdate_step();

        // check if any errors
        if (fuSDK.fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED)
            break;
    }

    // finally, we should have a status FINISHED
    EXPECT_EQ(fuSDK.fwUpdate_getSessionStatus(), fwUpdate::FINISHED);
}
