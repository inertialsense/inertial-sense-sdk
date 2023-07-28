//
// Created by kylemallory on 7/26/23.
//

#ifndef IS_FIRMWAREUPDATE_H
#define IS_FIRMWAREUPDATE_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "ISConstants.h"

#ifdef __cplusplus
#include <string>
extern "C" {
#endif

namespace fwUpdate {

    /** (DID_FIRMWARE_UPDATE) */

/*
 * The firmware_update DID type is an exchange of state and data that can be used to perform an update of a remote device's firmware
 *
 * Each packet defines a target.  Targets are hard-coded but follow a pattern, < 0xFF is Inertial-Sense, > 0xFF is 3rd-party.
 * High-nibble of LSB specifies the product, while the Low-nibble specifies a mask for which (1-4) devices to update.  Using a mask for the device allows
 * simultaneous updates of multiple, similar devices (ie, TARGET_SONY_CXD5610__1 | TARGET_SONY_CXD5610__2 can update both receivers from the same payload).
 *
 * It may become adventageous for a device to track the path (of ports) from which a request to update was received, thereby allowing for a more direct flow of
 * data between the target device and the host, without having to blindly forward data to all unknown ports.  This protocol is defined such that is should be
 * resilient to necessary changed to support this functionality, without adverse impacts on the performance or complexity of this subsystem.
 *
 * In addition to sending the actual firmware image over the wire, there are a few mechanisms to support efficient (and hopefully fast) updates to multiple devices
 * at the same time. However, care must be taken not to perform a reset of a device which is an intermediary relay (an upstream relay) to another device, since this
 * could impact communications to those downstream devices.  For example, don't reset the IMX if you're also in the process of updating the GPX and GNSS receivers.
 * The protocol is robust enough that it should be able to recover but it is strongly discouraged. Ideally, if updating multiple targets, wait until all targets have
 * finished, and received confirmation of the update being written, and then perform resets of all devices.
 *
 * The general flow of messages, when performing an update should look similar to the following (note that some devices may differ slightly):
 *
 *   Prior to the host PC sending the REQ_UPDATE message, it generates a random session-id, and specified the target slot for the GPX (1). It also populates the file
 *   size of the firmware image, specifies the chunk_size (the size of each transmitted chunk), and finally an MD5 hash for the entire transmitted firmware.
 *
 *   A REQ_UPDATE is sent from the host to a specific target device (ie, GPX on IG-2). Any device which receives the message is expected to forward the message to all
 *   connected ports on the device (ie, IMX receives on USB, and will resend on SER0, SER1, SER2, and on SPI (if configured)). It will NOT resend the same message BACK
 *   to the USB port.  In the IG-2, the GPX is connected on the SER0, so it will receive it on this port.  The GPX will accept both GPX and Sony GNSS messages. Since
 *   this message is for the GPX directly, it will be processed internally, and WILL NOT be forward onto any of its other ports (the GPX is the target).
 *
 *   On reception by the GPX, the GPX will respond with a RPL_UPDATE message, with the name session_id in the original message, the number of chunks that it will expect
 *   from the host (firmware size / chunk_size). If there are no issues with the request, the status will be set to GOOD_TO_GO.
 *
 *   Once the host receives the GOOD_TO_GO response, it will begin sending, without interruption, multiple UPDATE_CHUNK messages, in sequential, contiguous order, using
 *   the same target_device and session_id, and with incrementing chunk_id's for each message. It is assumed that as long as there is NO RESPONSE to an UPDATE_CHUNK
 *   message, that reception of that message was good, and the host will continue to send the next chunk until all chunks have been sent.
 *
 *   On the target device, when an UPDATE_CHUNK message is received, if the session id matches the session made in the original REQ_UPDATE, and the chunk_id matches the
 *   next EXPECTED chunk_id, then the chunk is written into NVME/Flash space, and an internal MD5 checksum is fed the data from the chunk. Note that the IS protocol has a
 *   higher-level mechanism to calculate/test a packet-level checksum to ensure data integrity of each message/packet.  If the session id is different, the device wil
 *   ignore the message entirely. Optionally, the device may respond with a REQ_RESEND_CHUNK to the host (it MUST make the request with the original, expected session id).
 *
 *   In the event that there is a failure to receive an UPDATE_CHUNK by the device, either due to a timeout or receiving a chunk out of sequence, the device should
 *   issue back to the host, a REQ_RESEND_CHUNK message, indicating the chunk id and the reason for the resend. Upon receiving a REQ_RESEND_CHUNK, a transmitting host will
 *   finish sending any chunk message in progress, and still stop sending any further chunks from the current sequence.  At this point, the host should resume sending a
 *   new continuous stream of UPDATE_CHUNK messages, starting with the requested id and continuing sequentially and contiguously from that id.
 *
 *   At ANY time during the update process, the device may send an UPDATE_PROGRESS message, which provides information about the status of the Update process. The message
 *   should include a chunk/total_chunk indicating, at the time the message was sent, the progress received, as well as a string message that can be displayed by the host
 *   PC to communicate status.
 *
 * From a functional standpoint, this module in comprised of 2 systems, an SDK interface, and a device interface. The SDK interface is used by the Host PC and associated
 * tool (EvalTool, cltool, etc) to initiate a request for and perform a firmware update. The device interface is instantiated on the device, and it called anytime a
 * DID_FIRMWARE_UPDATE message is received by the device, and is responsible for processing that message if it is the intended recipient, or to notify the calling interface
 * if it is not the intended target. NOTE: it is the role of the calling class to forward the incoming message to other ports, in the event that this device is not the
 * target. This is because the Device interface has no knowledge of the available ports on the specific device, as it is intended to be as device agnostic as possible.
 *
 * Note that this class/interface does not concern itself with device-specific functionality, such as managing the internal state of the device specific update mechanism,
 * nor does it deal with the actual mechanism for writing the firmware to flash.  This interface does offer a mechanism to ensure that the entirety of the firmware image
 * is transmitted correctly (hence MD5 checksum and a retry mechanism), but it does not attempt to validate the viability of the firmware image, or perform security checks
 * such as signing verification or decryption, etc. These are generally reserved for the specific device's implementation (typically OS specific) and its bootloader.
 *
 * HOW TO USE (Device):
 *   Instantiate a new FirmwareUpdateDevice() object, by passing the target ID for this device, and a pair of callback methods.  The first is the callback used to start
 *   the state machine to initiate the device for a firmware update.  The second callback is used when a valid CHUNK is received, which needs to be written to NVME/Flash.
 *
 */

    typedef enum {
        TARGET_NONE = 0x00,
        TARGET_IMX5 = 0x10,
        TARGET_GPX1 = 0x20,
        TARGET_VPX = 0x30, // RESERVED FOR VPX
        TARGET_UBLOX_F9P__1 = 0x111,
        TARGET_UBLOX_F9P__2 = 0x112,
        TARGET_UBLOX_F9P__ALL = 0x11F,
        TARGET_SONY_CXD5610__1 = 0x121,
        TARGET_SONY_CXD5610__2 = 0x122,
        TARGET_SONY_CXD5610__ALL = 0x12F,
        TARGET_MAXNUM
    } target_t;

    typedef enum {
        WAITING_FOR_DATA = 3,       // indicates that the update status has started, and at least 1 chunk has been sent, but more chunks are still expected
        GOOD_TO_GO = 2,             // indicates that the update status has finished initializing and is waiting for the first chunk of firmware data
        INITIALIZING = 1,           // indicates that an update has been requested, but the subsystem is waiting on completion of the bootloader or other back-end mechanism to initialize before data transfer can begin.
        NOT_STARTED = 0,            // indicates that the update process has not been initiated (it will fall back to this after an error, and short timeout).
        ERR_INVALID_SESSION = -1,   // indicates that the requested session ID is invalid.
        ERR_INVALID_SLOT = -2,      // indicates that the request slot does not exist. Different targets have different number of slots which can be written to.
        ERR_NOT_ALLOWED = -3,       // indicates that writing to the requested slot is not allowed, usually due to security constrains such as a locked firmware, Read-Only FLASH, etc.
        ERR_NOT_ENOUGH_MEMORY = -4, // indicates that the requested firmware file size would exceed the available slot size.
        ERR_OLDER_FIRMWARE = -5,    // indicates that the new firmware is an older (or earlier) version, and performing this would result in a downgrade.
        ERR_MAX_CHUNK_SIZE = -6,    // indicates that the maximum chunk size requested in the original upload request is too large.  The host is expected to begin a new session with a smaller chunk size.
        ERR_TIMEOUT = -7,           // indicates that the update process timed-out waiting for data (either a request, response, or chunk data that never arrived)
    } update_status_e;

    typedef enum {
        REASON_NONE = 0,
        REASON_INVALID_SEQID = 1,
        REASON_WRITE_ERROR = 2,     // there was an error writing the data to FLASH (perhaps it took too long?)
        REASON_INVALID_SIZE = 3,    // unless the chunk id is the last chunk, the size of the chunk should always be the negotiated chunk_size;
    } resend_reason_e;

    typedef enum {
        MSG_REQ_RESET = 1,          // a host is requesting that the device perform a reset.
        MSG_RESET_RESP = 2,         // response to the requesting host, that a reset was performed (but not guarantee that it was successful).
        MSG_REQ_UPDATE = 3,         // a host is requesting that the device enter update mode - in essense, initiate the update state-machine that is responsible for getting the target device into a state where it can receive an update.
                                    // The payload is a total of 12 bytes, the first 8 bytes, representing in little-endian, the overall size of the payload. The last 4 are the size of each payload chunk.
                                    // The payload max_chnks should be the total number of chunks, of overall payload size / the size of the payload chunk (rounded up).
        MSG_UPDATE_RESP = 4,        // communicates back to the host that the device is ready to update (in bootloader mode, etc) (the state machine has finished setup and is ready for data).
        MSG_UPDATE_CHUNK = 6,       // this message contains data which is a portion of the new firmware.  The chnk_id, and num_chunks represent the location of the payload within the overall firmware image
        MSG_UPDATE_PROGRESS = 7,    // this is a message sent back to the host at regular intervals to communicate to the user the status of the update process.  This message can be sent at any time
        MSG_REQ_RESEND_CHUNK = 8,   // this is a message send by the device back to the host, requesting that a particular chunk be resent.  The device should send this when there is an issue with the last received "UPDATE_PAYLOAD",
                                    // either in a checksum error, invalid/missing chunk id, etc.  When this message is received by the host, the host MUST resend the requested chunk, and all subsequent chunks that follow it, regardless
                                    // if they were previously sent.  Likewise, on the device, as soon as a received chunk is deemed invalid, forcing this message to be sent back to the host, all subsequent payload chunks received which
                                    // are NOT this requested chunk MUST BE ingored.
        MSG_UPDATE_FINISHED = 9,    // this message is sent when the device-side has completed receiving file chunks, regardless of the status of those chunks, or the reception of all available chunks.  In essense, this is a notice
                                    // to the host that no more chunks of data will be accepted, regardless of state. Included in this message is a status indicating whether the image transfer was successful, of not. When this message
                                    // is sent, the associated session_id is invalidated ensuring that no further messages can be processed. If there is an error, a new session will need to be started.
    } msg_types_e;

    typedef union {
        struct {

        } req_reset;

        struct {

        } rpl_reset;

        struct PACKED {
            uint8_t image_slot;     //! a device-specific "slot" which is used to target specific files/regions of FLASH to update, ie, the Sony GNSS receiver has 4 different firmware files, each needs to be applied in turn. If the 8th (MSB) bit is raised, this is treated as a "FORCE"
            uint16_t session_id;    //! random 16-bit identifier used to validate the data stream. This should be regenerated for each REQUEST_UPDATE
            uint32_t file_size;     //! the total size of the entire firmware file
            uint16_t chunk_size;    //! the maximum size of each chunk
            uint32_t md5_hash;      //! the md5 hash for the original firmware file.  If the delivered MD5 hash doesn't match this, after receiving the final chunk, the firmware file will be discarded.
        } req_update;

        struct PACKED {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            uint16_t totl_chunks;   //! the total number of chunks that are necessary to transmit the entire firmware file
            update_status_e status; //! a status code (OK, ERROR, etc). Any error reported invalidates the session_id, and a new request with a new session_id must be made
        } update_resp;

        struct PACKED {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            uint16_t chunk_id;      //! the chunk number identifying this portion of the firmware
            uint16_t data_len;      //! the number of bytes of accompanying data
            uint8_t data;           //! the first byte of data (cast to a uint8_t * to access the rest...)
        } chunk;

        struct PACKED {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            uint16_t chunk_id;      //! the chunk number identifying this portion of the firmware which should be resent
            resend_reason_e reason; //! an indicator of why this chunk was requested. This is optional, but is useful for debugging purposes. Regardless of the reason, the requested chunk, and all subsequent chunks must be resent.
        } req_resend;

        struct PACKED {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            uint16_t num_chunks;    //! the number of chunks which have so far been received by the device, in sequential, contiguous order.  Ie, this is only the number of received VALID and processed chunks.
            uint16_t totl_chunks;   //! the total number of chunks which the device is expecting from the host
            uint8_t msg_level;      //! a numerical indication of the criticality of this message, 0 being the highest. Best practive is to associate syslog type levels here (CRITICAL, ERROR, WARN, INFO, DEBUG, etc).
            uint8_t msg_len;        //! the length of the following string (in bytes)
            uint8_t message;        //! an arbitrary human-readable string, that is intended to be consumed by the user to give status about the update process
        } progress;
    } msg_data_t;

    typedef struct PACKED {
        target_t target_device;     //! the target type and instance which this message is intended for
        msg_types_e msg_type;       //! msg_type enum used to indicate how to parse the subsequent data in this message
        msg_data_t msg_data;        //! the actual message data
    } payload_t;

//    typedef int (*start_update_fn)(uint16_t); // a call-back function that is used to notify the application that an update has been requested.
//    typedef int (*write_chunk_fn)(uint16_t, uint16_t len, uint8_t* data); // a call-back function that is used to perform the writing of a chunk of data to NVME/Flash.
//    typedef int (*finish_update_fn)(); // a call-back function that is used to notify the application that the update is finished writing.
//    typedef int (*reset_mcu_fn)();  // a call-back function that is used to notify the application to perform a reset of the target device.

    /**
     * FirmwareUpdateDevice is a base abstract-class implementation of a device/target specific implementation, and provides the majority of common
     * functionality used by the API.
     *
     * Implementing classes should extend FirmwareUpdateDevice into a device-specific class, such as FirmwareUpdateGPX, which provides target-specific
     * mechanisms for persisting data, reading/writing to/from comm ports/interfaces, etc.
     */
    class FirmwareUpdateDevice {
    public:
        /**
         * Creates a FirmwareUpdateDevice instance
         * @param target_id the target_id which this message will respond to.
         * @param startUpdate
         * @param writeChunk
         * @param resetMcu
         * @param progress_millis the rate at which progress updates will be sent out, in milli-seconds (default is every 100ms, 0 = no updates are sent).
         */
        FirmwareUpdateDevice(uint16_t target_id, uint16_t progress_millis = 100);

        /**
         * called at each step interval; if you put this behind a Scheduled Task, call this method at each interval.
         * This method is primarily used to perform routine maintenance, like checking if the init process is complete, or to give out status update, etc.
         * If you don't call step() things should still generally work, but it probably won't seem very responsive.
         * @return the number of times step() has been called since the last request to start an update. This isn't immediately useful, but its nice to know that its actually doing something.
         */
        int step();

        /**
         * Called by the communications system anytime a DID_FIRMWARE_UPDATE is received.
         * @param msg_payload the contents of the DID_FIRMWARE_UPDATE payload
         * @return true if this message was consumed by this interface, or false if the message was not intended for us, and should be passed along to other ports/interfaces.
         *
         * Note: Internally, this method calls step(), so even if you don't call step(), but it can still operate with just inbound messages, but interval updates/etc won't run.
         */
        int processMessage(const payload_t& msg_payload);

        /**
         * Unpacks a DID payload byte buffer (from the comms system) into a firmware_update msg_payload_t struct
         * Note that this results in at least one copy, and possibly multiple assignments. Where possible, you
         * should opt to cast the pointer into a msg_payload_t, and use directly, but that isn't always possible.
         * @param buffer a pointer to the start of the byte buffer containing the raw data
         * @param buf_len the number of bytes the unpack from the byte buffer
         * @param msg_payload the payload_t struct that the data will be unpacked into.
         * @return true on success, otherwise false
         */
        bool unpackPayload(const uint8_t* buffer, int buf_len, payload_t& msg_payload);

        /**
         * Packs a byte buffer that can be sent out onto the wire, using data from a passed msg_payload_t.
         * Note that this results in at least one copy, and possibly multiple assignments. Where possible, you
         * should opt to cast the payload directly into a uint8_t pointer and use directly. This is not always
         * possible (particularly with strings/chunk data).
         * @param msg_payload
         * @param buffer
         * @param max_len
         * @return
         */
        int packPayload(const payload_t& msg_payload, uint8_t* buffer, int max_len);

        //===========  Functions which MUST be implemented ===========//
        virtual int performSoftReset(int target_id) = 0; // this is a software managed reset, by such as my informing the OS/MCU to restart the system
        virtual int performHardReset(int target_id) = 0; // this is a hardware, force reset usually by pulling interfacing pins into the mcu either HIGH or LOW to force a reset state on the hardware

        virtual int startFirmwareUpdate(int target_id, int slot_id, int image_size) = 0; // this initializes the system to begin receiving firmware image chunks for the target device, image slot and image size
        virtual int writeImageChunk(int target_id, int slot_id, int offset, int len, uint8_t *data) = 0; // writes the indicated block of data (of len bytes) to the target and device-specific image slot, and with the specified offset
        virtual int finishFirmwareUpgrade(int target_id, int slot_id) = 0; // this marks the finish of the upgrade, that all image bytes have been received, the md5 sum passed, and the device can complete the requested upgrade, and perform any device-specific finalization


    protected:
        /**
         * This is an internal method used to send an update message to the host system regarding the status of the update process
         * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
         * @param message the actual message to be sent to the host
         * @return true if the message was sent, false if there was an error
         */
        bool sendProgress(int level, const std::string message);

        /**
         * This is an internal method used to send an update message to the host system regarding the status of the update process
         * This variation allows for printf-based string formatting
         * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
         * @param message the actual message to be sent to the host
         * @
         * @return true if the message was sent, false if there was an error
         */
        bool sendProgress(int level, const std::string message, ...);

        /**
         * @return true if we have an active session and are updating.
         */
        bool isUpdating();

    private:
        uint16_t target_id = TARGET_NONE;
//        start_update_fn cb_startUpdate = nullptr;
//        write_chunk_fn cb_writeChunk = nullptr;
//        finish_update_fn cb_finishUpdate = nullptr;
//        reset_mcu_fn cb_resetMcu = nullptr;
        uint16_t progress_interval = 100;

        uint16_t cur_session_id = 0;        //! the current session id - all received messages with a session_id must match this value.  O == no session set (invalid)
        uint16_t last_chunk_id = 0xFFFF;    //! the last received chunk id from a CHUNK message.  0xFFFF == no chunk yet received; the next received chunk must be 0.
        uint16_t chunk_size = 0;            //! the negotiated maximum size for each chunk.
        uint16_t total_chunks = 0;          //! the total number of chunks for the given image size
        uint32_t image_size = 0;            //! the total size of the image to be sent
        uint8_t image_slot = 0;             //! the "slot" to which this image will be written in the flash

        uint32_t[4] md5hash { 0x0, 0x0, 0x0, 0x0 };

        /**
         * Internal method use to reinitialize the update engine.  This should clear the the current session_id, existing image data, running md5 sums, etc.
         * After calling this function, the subsystem must receive a REQ_UPDATE message to start to new session, etc.
         * This probably should be called after an update is finished, but is probably safest to call as the first step in a REQ_UPDATE.
         * @return true if the system was able to properly initialize, false if there was an error of something (you have a REAL problem in this case).
         */
        bool resetEngine();

        /**
         * Internally called by processMessage() when a REQ_UPDATE message is received.
         * @param payload the DID message
         * @return true if the message was received and parsed without error, false otherwise.
         *
         * NOTE, this function should call out and send an error status in the event of a failure.
         */
        bool handleInitialize(const payload_t& payload);

        /**
         * Internally called by processMessage() when a UPDATE_CHUNK message is received.
         * @param payload the DID payload
         * @return
         */
        bool handleChunk(const payload_t& payload);

        /**
         * Internally called by processMessage() when a REQ_RESET message is received, to reset the target MCU.
         * @param payload the DID message
         * @return true if the message was received and parsed without error, false otherwise.
         */
        bool handleMcuReset(const payload_t& payload);

        /**
         * Sends a REQ_RESEND_CHUNK message in response to receiving an invalid CHUNK message. This will ALWAYS send with the current session_id, and the last received chunk_id + 1;
         * @param reason for the resend
         * @return return true is a retry was sent, or false if a retry was not sent.  NOTE this is not an error, as a valid message will not send a retry.
         */
        bool sendRetry(resend_reason_e reason);

        /**
         * Initializes the MD5 hash. Don't forget to call hashMd5() afterwards to actually get your hash
         */
        void resetMd5();

        /**
         * Adds the specified data into the running MD5 hash
         * @param len the number of bytes to consume into the hash
         * @param data the bytes to consume into the hash
         * @return a static buffer of 16 unsigned bytes which represent the 128 total bits of the MD5 hash
         */
        uint8_t* hashMd5(uint16_t len, uint8_t* data);

    };

    class FirmwareUpdateSDK {

    };

} // fwUpdate

#ifdef __cplusplus
    }
#endif

#endif //IS_FIRMWAREUPDATE_H
