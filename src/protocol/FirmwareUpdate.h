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
#include "ISUtilities.h"

#ifdef __cplusplus
#include <string>

#include "util/md5.h"

// #define DEBUG_LOGGING
#ifdef __ZEPHYR__
    #include <zephyr/logging/log.h>
#else
    #ifdef DEBUG_LOGGING
        #define LOG_DBG(...)    printf(__VA_ARGS__); printf("\n");
    #else
        #define LOG_DBG(...)
    #endif
#endif

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
 *   size of the firmware image, specifies the session_chunk_size (the size of each transmitted chunk), and finally an MD5 hash for the entire transmitted firmware.
 *
 *   A REQ_UPDATE is sent from the host to a specific target device (ie, GPX on IG-2). Any device which receives the message is expected to forward the message to all
 *   connected ports on the device (ie, IMX receives on USB, and will resend on SER0, SER1, SER2, and on SPI (if configured)). It will NOT resend the same message BACK
 *   to the USB port.  In the IG-2, the GPX is connected on the SER0, so it will receive it on this port.  The GPX will accept both GPX and Sony GNSS messages. Since
 *   this message is for the GPX directly, it will be processed internally, and WILL NOT be forward onto any of its other ports (the GPX is the target).
 *
 *   On reception by the GPX, the GPX will respond with a RPL_UPDATE message, with the name session_id in the original message, the number of chunks that it will expect
 *   from the host (firmware size / session_chunk_size). If there are no issues with the request, the status will be set to GOOD_TO_GO.
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

#define FWUPDATE__MAX_CHUNK_SIZE   512
#define FWUPDATE__MAX_PAYLOAD_SIZE (FWUPDATE__MAX_CHUNK_SIZE + 92)

    static constexpr uint32_t TARGET_TYPE_MASK = 0xFFF0;
    static constexpr uint32_t TARGET_DFU_FLAG = 0x80000000;
    static constexpr uint32_t TARGET_ISB_FLAG = 0x40000000;

    enum target_t : uint32_t {
        TARGET_HOST = 0x00,
        TARGET_IMX5 = 0x10,
        TARGET_DFU_IMX5 = (TARGET_DFU_FLAG | TARGET_IMX5),
        TARGET_ISB_IMX5 = (TARGET_ISB_FLAG | TARGET_IMX5), // note that the IMX5 ONLY support ISB mode (it doesn't directly support ISv2)
        TARGET_GPX1 = 0x20,
        TARGET_DFU_GPX1 = (TARGET_DFU_FLAG | TARGET_GPX1),
        TARGET_VPX = 0x30, // RESERVED FOR VPX
        TARGET_UBLOX_F9P = 0x110,
        TARGET_UBLOX_F9P__1 = 0x111,
        TARGET_UBLOX_F9P__2 = 0x112,
        TARGET_UBLOX_F9P__ALL = 0x11F,
        TARGET_SONY_CXD5610 = 0x120,
        TARGET_SONY_CXD5610__1 = 0x121,
        TARGET_SONY_CXD5610__2 = 0x122,
        TARGET_SONY_CXD5610__ALL = 0x12F,
        TARGET_MAXNUM,
        TARGET_UNKNOWN = 0xFFFFFFFF,
    };

    enum msg_types_e : uint32_t {
        MSG_UNKNOWN = 0,            // an unknown or undefined message type.
        MSG_REQ_RESET = 1,          // a host is requesting that the device perform a reset.
        MSG_RESET_RESP = 2,         // response to the requesting host, that a reset was performed (but not guarantee that it was successful).
        MSG_REQ_UPDATE = 3,         // a host is requesting that the device enter update mode - in essense, initiate the update state-machine that is responsible for getting the target device into a state where it can receive an update.
        // The payload is a total of 12 bytes, the first 8 bytes, representing in little-endian, the overall size of the payload. The last 4 are the size of each payload chunk.
        // The payload max_chnks should be the total number of chunks, of overall payload size / the size of the payload chunk (rounded up).
        MSG_UPDATE_RESP = 4,        // communicates back to the host that the device is ready to update (in bootloader mode, etc) (the state machine has finished setup and is ready for data).
        MSG_UPDATE_CHUNK = 5,       // this message contains data which is a portion of the new firmware.  The chnk_id, and num_chunks represent the location of the payload within the overall firmware image
        MSG_UPDATE_PROGRESS = 6,    // this is a message sent back to the host at regular intervals to communicate to the user the status of the update process.  This message can be sent at any time
        MSG_REQ_RESEND_CHUNK = 7,   // this is a message send by the device back to the host, requesting that a particular chunk be resent.  The device should send this when there is an issue with the last received "UPDATE_PAYLOAD",
        // either in a checksum error, invalid/missing chunk id, etc.  When this message is received by the host, the host MUST resend the requested chunk, and all subsequent chunks that follow it, regardless
        // if they were previously sent.  Likewise, on the device, as soon as a received chunk is deemed invalid, forcing this message to be sent back to the host, all subsequent payload chunks received which
        // are NOT this requested chunk MUST BE ignored.
        MSG_UPDATE_DONE = 8,        // this message is sent when the device-side has completed receiving file chunks, regardless of the status of those chunks, or the reception of all available chunks.  In essense, this is a notice
        // to the host that no more chunks of data will be accepted, regardless of state. Included in this message is a status indicating whether the image transfer was successful, of not. When this message
        // is sent, the associated session_id is invalidated ensuring that no further messages can be processed. If there is an error, a new session will need to be started.
        MSG_REQ_VERSION_INFO = 9,   // this message is sent by the host to request information about the current target's firmware
        MSG_VERSION_INFO_RESP = 10, // this message is the response from a device, which details the target devices hardware and firmware version and also firmware build info.
    };

    enum update_status_e : int16_t {
        FINISHED = 5,               // indicates that all operations are completed and device reports success (generally ready for a reset)
        FINALIZING = 4,             // indicates that all chunks have been received, and checksum is valid, but still waiting on internal operations to complete
        IN_PROGRESS = 3,            // indicates that the update status has started, and at least 1 chunk has been sent, but more chunks are still expected
        READY = 2,                  // indicates that the update status has finished initializing and is waiting for the first chunk of firmware data
        INITIALIZING = 1,           // indicates that an update has been received, but the subsystem is waiting on completion of the bootloader or other back-end mechanism to initialize before data transfer can begin.
        NOT_STARTED = 0,            // indicates that the update process has not been initiated (it will fall back to this after an error, and a short timeout).
        ERR_INVALID_SESSION = -1,   // indicates that the requested session ID is invalid.
        ERR_INVALID_SLOT = -2,      // indicates that the request slot does not exist. Different targets have different number of slots which can be written to.
        ERR_NOT_ALLOWED = -3,       // indicates that writing to the requested slot is not allowed, usually due to security constrains such as a locked firmware, Read-Only FLASH, etc.
        ERR_NOT_ENOUGH_MEMORY = -4, // indicates that the requested firmware file size would exceed the available slot size.
        ERR_OLDER_FIRMWARE = -5,    // indicates that the new firmware is an older (or earlier) version, and performing this would result in a downgrade.
        ERR_MAX_CHUNK_SIZE = -6,    // indicates that the maximum chunk size requested in the original upload request is too large.  The host is expected to begin a new session with a smaller chunk size.
        ERR_TIMEOUT = -7,           // indicates that the update process timed-out waiting for data (either a request, response, or chunk data that never arrived)
        ERR_CHECKSUM_MISMATCH = -8, // indicates that the final checksum didn't match the checksum specified at the start of the process
        ERR_COMMS = -9,             // indicates that an error in the underlying comms system
        ERR_NOT_SUPPORTED = -10,    // indicates that the target device doesn't support this protocol
        ERR_FLASH_WRITE_FAILURE = -11,    // indicates that writing of the chunk to flash/nvme storage failed (this can be retried)
        ERR_FLASH_OPEN_FAILURE = -12,   // indicates that an attempt to "open" a particular flash location failed for unknown reasons.
        ERR_FLASH_INVALID = -13,    // indicates that the image, after writing to flash failed to validate.
        ERR_UPDATER_CLOSED = -14,   //
        ERR_INVALID_IMAGE = -15,    // indicates that the specified image file is invalid; this can also be reported directly by the host if the image file is not found.
        ERR_INVALID_CHUNK = -16,    // indicates a repeated failure to deliver the correct chunk id or size
        ERR_INVALID_TARGET = -17,   // indicates that the target is invalid - this could mean that the target 'index' doesn't exist, or that the target + target_flags are unsupported, etc.
        ERR_UNKNOWN = -18,          // indicates an unknown error, this should *always* be the last (lower) value
        // TODO: IF YOU ADD NEW ERROR MESSAGES, don't forget to update fwUpdate::status_names, and getSessionStatusName()
    };

    enum resend_reason_e : int16_t {
        REASON_NONE = 0,
        REASON_INVALID_SEQID = 1,   // the last received chunk was out of order, so we're requesting the correct chunk id.
        REASON_WRITE_ERROR = 2,     // there was an error writing the data to FLASH (perhaps it took too long?)
        REASON_INVALID_SIZE = 3,    // unless the chunk id is the last chunk, the size of the chunk should always be the negotiated session_chunk_size;
    };

    enum reset_flags_e : uint16_t {
        RESET_SOFT = 0,             // typically, a software reset (start the program over, but don't remove power or clear RAM)
        RESET_HARD = 1,             // a hard reset, in which the device is power-cycled; this may not always be possible since generally software on a device can't remove its own power
        RESET_INTO_BOOTLOADER = 2,  // indicates that the device should reset into the bootloader (this may not always be possible)
        RESET_CONFIG = 4,           // indicates that the device should clear its configuration before performing the reset (Ie, factory restart?)
        RESET_UPSTREAM = 8,         // indicates that this device should reset all of its upstream devices, in addition to itself
    };

    enum image_flagsPos_e : uint8_t {
        IMG_FLAG_POS__imageNotEncrypted = 0,  // position of bit that informs firmware that sony image is not encrypted
        IMG_FLAG_POS__useAlternateMD5 = 7,    // position of bit that informs firmware to use the alternate (obsolete) MD5 algorithm
    };

    enum image_flagsMask_e : uint8_t {
        IMG_FLAG_imageNotEncrypted = 0x01 << IMG_FLAG_POS__imageNotEncrypted,   // bit mask that informs firmware that sony image is not encrypted
        IMG_FLAG_useAlternateMD5 = 1 << IMG_FLAG_POS__useAlternateMD5           // bit mask that informs firmware to use the alternate (obsolete) MD5 algo
    };

    PUSH_PACK_1

    typedef union {
        struct {
            uint16_t reset_flags;
        } req_reset;

        struct {             //! a response to a reset request (usually this isn't sent, but sometimes, like asking a GNSS receiver to reset, it could send back a reply, since that doesn't originate on the GNSS receiver).
            target_t target;        //! responding target
            uint16_t status;        //! response status (0 == success)
        } rpl_reset;

        struct {
        } req_version;     //! requests the version info for the target device

        struct {
            uint16_t session_id;    //! random 16-bit identifier used to validate the data stream. This should be regenerated for each REQUEST_UPDATE
            uint8_t image_slot;     //! a device-specific "slot" which is used to target specific files/regions of FLASH to update, ie, the Sony GNSS receiver has 4 different firmware files, each needs to be applied in turn. If the 8th (MSB) bit is raised, this is treated as a "FORCE"
            uint8_t image_flags;    //! flags for update (see image_flagsMask_e)
            uint32_t file_size;     //! the total size of the entire firmware file
            uint16_t chunk_size;    //! the maximum size of each chunk
            uint16_t progress_rate; //! the rate (millis) at which the device should publish progress reports back to the host.
            md5hash_t md5_hash;     //! the md5 hash for the original firmware file.  If the delivered MD5 hash doesn't match this, after receiving the final chunk, the firmware file will be discarded.
        } req_update;

        struct {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            uint16_t totl_chunks;   //! the total number of chunks that are necessary to transmit the entire firmware file
            update_status_e status; //! a status code (OK, ERROR, etc). Any error reported invalidates the session_id, and a new request with a new session_id must be made
        } update_resp;

        struct {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            uint16_t chunk_id;      //! the chunk number identifying this portion of the firmware
            uint16_t data_len;      //! the number of bytes of accompanying data
            uint8_t data;           //! the first byte of data (cast to a uint8_t * to access the rest...)
        } chunk;

        struct {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            uint16_t chunk_id;      //! the chunk number identifying this portion of the firmware which should be resent
            resend_reason_e reason; //! an indicator of why this chunk was requested. This is optional, but is useful for debugging purposes. Regardless of the reason, the requested chunk, and all subsequent chunks must be resent.
        } req_resend;

        struct {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            update_status_e status; //! the current status of the session, from the device standpoint (only devices send the progress message)
            uint16_t num_chunks;    //! the number of chunks which have so far been received by the device, in sequential, contiguous order.  Ie, this is only the number of received VALID and processed chunks.
            uint16_t totl_chunks;   //! the total number of chunks which the device is expecting from the host
            uint8_t msg_level;      //! a numerical indication of the criticality of this message, 0 being the highest. Best practive is to associate syslog type levels here (CRITICAL, ERROR, WARN, INFO, DEBUG, etc).
            uint8_t msg_len;        //! the length of the following string (in bytes)
            uint8_t message;        //! an arbitrary human-readable string, that is intended to be consumed by the user to give status about the update process
        } progress;

        struct {
            uint16_t session_id;    //! random 16-bit identifier used to validate/associate the data stream.
            update_status_e status; //! a status code (OK, ERROR, etc). Any error reported invalidates the session_id, and a new request with a new session_id must be made
        } resp_done;

        struct {
            target_t resTarget;     //! the target identifier of the responding device (for which this data represents)
            uint32_t serialNumber;  //! the serial number of the host, or controlling device (return the IMX SN if querying the IMX's Accelerometer, for example)
            uint8_t reserved;       //! unused
            uint8_t hardwareType;   //! hardware identifier
            uint8_t hardwareVer[4]; //! Hardware version
            uint8_t firmwareVer[4]; //! Firmware (software) version

            uint8_t buildHash[4];   //! Git hash
            uint32_t buildNumber;   //! Build number

            uint8_t buildType;      //! Build type (Release: 'a'=ALPHA, 'b'=BETA, 'c'=RELEASE CANDIDATE, 'r'=PRODUCTION RELEASE, 'd'=debug)
            uint8_t buildYear;      //! Build date year - 2000
            uint8_t buildMonth;     //! Build date month
            uint8_t buildDay;       //! Build date day

            uint8_t buildHour;      //! Build time hour
            uint8_t buildMinute;    //! Build time minute
            uint8_t buildSecond;    //! Build time second
            uint8_t buildMillis;    //! Build time millisecond
        } version_resp;

    } msg_data_t;

    typedef struct {
        target_t target_device;     //! the target type and instance which this message is intended for
        msg_types_e msg_type;       //! msg_type enum used to indicate how to parse the subsequent data in this message
    } msg_header_t;

    typedef struct {
        msg_header_t hdr;
        msg_data_t data;        //! the actual message data
    } payload_t;

    POP_PACK

//    typedef int (*start_update_fn)(uint16_t); // a call-back function that is used to notify the application that an update has been requested.
//    typedef int (*write_chunk_fn)(uint16_t, uint16_t len, uint8_t* data); // a call-back function that is used to perform the writing of a chunk of data to NVME/Flash.
//    typedef int (*finish_update_fn)(); // a call-back function that is used to notify the application that the update is finished writing.
//    typedef int (*reset_mcu_fn)();  // a call-back function that is used to notify the application to perform a reset of the target device.

    /**
     * FirmwareUpdateBase provides base abstract-class functionality for all FirmwareUpdate functionality, such as parsing of packets, MD5 checksum
     * calculations, etc.
     */
    class FirmwareUpdateBase {
    public:

        static const size_t MaxChunkSize = FWUPDATE__MAX_CHUNK_SIZE;
        static const size_t MaxPayloadSize = FWUPDATE__MAX_PAYLOAD_SIZE;

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
        static int fwUpdate_packPayload(uint8_t* buffer, int max_len, const payload_t& payload, const void *aux_data= nullptr);

        /**
         * Unpacks a DID payload byte buffer (from the comms system) into a firmware_update msg_payload_t struct
         * Note that this results in at least one copy, and possibly multiple assignments. Where possible, you
         * should opt to cast the pointer into a msg_payload_t, and use directly, but that isn't always possible.
         * @param buffer a pointer to the start of the byte buffer containing the raw data
         * @param buf_len the number of bytes the unpack from the byte buffer
         * @param msg_payload the payload_t struct that the data will be unpacked into.
         * @return true on success, otherwise false
         */
        static int fwUpdate_unpackPayload(const uint8_t* buffer, int buf_len, payload_t& payload, void *aux_data= nullptr, uint16_t max_aux= 0);

        /**
         * maps a DID payload byte buffer (from the comms system) into a fwUpdate::payload_t struct, and extracts aux_data if any.
         * @param buffer a pointer to the raw byte buffer
         * @param msg_payload a double-pointer which on return will point to the start of the buffer (this is a simple cast)
         * @param aux_data a double-pointer which on return will point to any auxilary data in the payload, or nullptr if there is none
         * @return returns the total number of bytes in the packet, including aux data if any
         */
        static int fwUpdate_mapBufferToPayload(const uint8_t *buffer, payload_t** payload, void** aux_data);

        /**
         * Returns the string representation of the passed status
         * @param status
         * @return a constant char * to a string representing the specified status
         */
        static const char *fwUpdate_getStatusName(update_status_e status);

        /**
         * Returns a human-friendly string describing the update status, used for UIs
         * @param status
         * @return a constant char * to a string representing the specified status
         */
        static const char *fwUpdate_getNiceStatusName(update_status_e status);

        /**
         * Returns the string representation of the passed target
         * @param target
         * @return a constant char * to a string representing the specified target
         */
        static const char *fwUpdate_getTargetName(target_t target);

        /**
         * Returns the "Type" portion of the specified target_t, stripping out target flags, and index identifiers.
         * ie, passing TARGET_DFU_IMX5 would return TARGET_IMX5; passing TARGET_SONY_CXD5610__ALL returns TARGET_SONY_CXD5610.
         * @param target the original target type
         * @return the base target type
         */
        target_t fwUpdate_getTargetType(target_t target) { return (target_t)((uint32_t)target & TARGET_TYPE_MASK); };

    protected:
        uint8_t build_buffer[FWUPDATE__MAX_PAYLOAD_SIZE];       //! workspace for packing/unpacking payload messages
        uint32_t last_message = 0;                              //! the time (millis) since we last received a payload targeted for us.
        uint32_t timeout_duration = 20000;                      //! the number of millis without any messages, by which we determine a timeout has occurred.  TODO: Should we prod the device (with a required response) at regular multiples of this to effect a keep-alive?
        uint32_t resend_count = 0;                              //! the number of times a request was sent/received to resend a chunk. This provides an error rate mechanism; Ideal is < 1% of total packets.

        target_t session_target = TARGET_HOST;
        update_status_e session_status = NOT_STARTED;           //! last known state of this session
        uint16_t session_id = 0;                                //! the current session id - all received messages with a session_id must match this value.  O == no session set (invalid)
        uint16_t session_chunk_size = 0;                        //! the negotiated maximum size for each chunk.
        uint16_t session_total_chunks = 0;                      //! the total number of chunks for the given image size
        uint32_t session_image_size = 0;                        //! the total size of the image to be sent
        uint16_t session_progress_rate = 250;                   //! the number of milliseconds between progress updates
        uint8_t session_image_slot = 0;                         //! the "slot" to which this image will be written in the flash
        uint8_t session_image_flags = 0;                        //! additional flags to be communicated to the device for special processing
        md5hash_t session_md5 = { };                            //! the md5 of the firmware image
        md5Context_t md5Context = { };                          //! context used internally for building MD5 hashes

        FirmwareUpdateBase();
        virtual ~FirmwareUpdateBase() {};

        /**
         * packages and sends the specified payload, including any auxillary data.
         * Note that the payload must already specify the amount of aux data the be included.
         * @param payload
         * @param aux_data the auxillary data to include, or nullptr if none.
         * @return
         */
        bool fwUpdate_sendPayload(fwUpdate::payload_t& payload, void *aux_data= nullptr);

        /**
         * This method is primarily used to perform routine maintenance, like checking if the init process is complete, or to give out status update, etc.
         * Called with each messages that is received/processed.  Can optionally be called manually, typically at a regular interval. If you don't call
         * fwUpdate_step() things should still generally work, but state advancement may stall if there is no received message (for example, after all
         * messages have been sent by the remote).
         * @param msg_type the last received msg_type that was received.  If you are calling this method manually (from a timer, etc) pass in MSG_UNKNOWN.
         * @param processed indicates whether the received message was successfully processed after being received (this is usually the return value from
         *  fwUpdate_handleXXXX() functions).
         * @return false _suggests_ that the step couldn't complete in some way or another, perhaps because there was no processed messages, or that the
         *  is in an invalid state, etc.  Otherwise, return true. This result is primarily informational, and should not effect any downstream functionality
         *  or prevent (or indicate) a failure of the firmware update engine.
         */
        virtual bool fwUpdate_step(msg_types_e msg_type = MSG_UNKNOWN, bool processed = false) = 0;

        /**
         * Virtual function that must be implemented in the concrete implementations, responsible for writing buffer out to the wire (serial, or otherwise).
         * @param target a reference to the target for which this data is intended
         * @param buffer
         * @param buff_len
         * @return
         */
        virtual bool fwUpdate_writeToWire(target_t target, uint8_t* buffer, int buff_len) = 0;

        /**
         * Sets the duration (in milliseconds) which will trigger a Timeout status if a session has been started, but no further communications has been received for this target (host or device).
         * @param timeout
         */
        void fwUpdate_setTimeoutDuration(uint32_t timeout) { timeout_duration = timeout; }

        /**
         * Returns the elapsed time since the last message was received by this target, meant for this target.  Use this value > timeoutDuration to detect a timeout condition.
         * @return
         */
        uint32_t fwUpdate_getLastMessageAge() { return current_timeMs() - last_message; }

        /**
         * Forces a reset of the last message time; this is useful when first starting a new session.
         */
        void fwUpdate_resetTimeout() { last_message = current_timeMs(); }

        static char* fwUpdate_payloadToString(const payload_t* payload);

    private:
        /**
         * returns the total size of the passed payload msg, including any variable length data included in the message.
         * @param msg
         * @return the number of bytes that this entire message contains, including headers, etc.
         */
        static size_t fwUpdate_getPayloadSize(const payload_t* payload, bool include_aux= false);
    };

    /**
     * FirmwareUpdateDevice is a base abstract-class implementation of a device/target specific implementation, and provides the majority of common
     * functionality used by the API.
     *
     * Implementing classes should extend FirmwareUpdateDevice into a device-specific class, such as FirmwareUpdateGPX, which provides target-specific
     * mechanisms for persisting data, reading/writing to/from comm ports/interfaces, etc.
     */
    class FirmwareUpdateDevice : public FirmwareUpdateBase {
    public:
        /**
         * Creates a FirmwareUpdateDevice instance
         * @param target_id informs this instance which messages it should respond to.
         */
        FirmwareUpdateDevice(target_t target_id);
        virtual ~FirmwareUpdateDevice() { };

        /**
         * Called by the communications system anytime a DID_FIRMWARE_UPDATE is received.
         * @param msg_payload the contents of the DID_FIRMWARE_UPDATE payload
         * @return true if this message was consumed by this interface, or false if the message was not intended for us, and should be passed along to other ports/interfaces.
         *
         * Note: Internally, this method calls fwUpdate_step(), so even if you don't call fwUpdate_step(), but it can still operate with just inbound messages, but interval updates/etc won't run.
         */
        bool fwUpdate_processMessage(const payload_t& msg_payload);
        bool fwUpdate_processMessage(const uint8_t* buffer, int buf_len);

        /**
         * @return the target type for this instance.
         */
        target_t fwUpdate_getCurrentTarget() { return session_target; }
        update_status_e fwUpdate_getSessionStatus() { return session_status; }
        uint16_t fwUpdate_getSessionID() { return session_id; }
        float fwUpdate_getResendRate() { return ((float)resend_count / (float)last_chunk_id); }
        uint16_t fwUpdate_getLastChunkID() { return last_chunk_id; }
        uint16_t fwUpdate_getChunkSize() { return session_chunk_size; }
        uint16_t fwUpdate_getTotalChunks() { return session_total_chunks; }
        uint16_t fwUpdate_getImageSize() { return session_image_size; }
        uint16_t fwUpdate_getImageSlot() { return session_image_slot; }



        //===========  Functions which MUST be implemented ===========//

        /**
         * Writes the requested data (usually a packed payload_t) out to the specified device
         * Note that the implementation between a target and an actual interface is device-specific. In most cases,
         * for a Device-implementation, this will typically specify TARGET_HOST, which will direct back to the
         * controlling host.
         * @param target
         * @param buffer
         * @param buff_len
         * @return true if the data was successfully sent to the underlying communication system, otherwise false
         */
        virtual bool fwUpdate_writeToWire(target_t target, uint8_t* buffer, int buff_len) = 0;

        /**
         * Performs a system reset of various severity per reset_flags, (ie, RESET_SOFT by informing the OS/MCU to restart the system,
         * vs RESET_HARD, usually by pulling interfacing pins into the MCU either HIGH or LOW to force a reset state on the hardware).
         * Note that some systems may not always be able to respond with a success before the system is reset.
         * If a system is NOT able to perform a reset (ie UNSUPPORTED, etc), this MUST return false.
         * @param target_id the device to reset
         * @return true if successful, otherwise false
         */
        virtual int fwUpdate_performReset(target_t target_id, reset_flags_e reset_flags) = 0;

        /**
         * Internally called by fwUpdate_processMessage() when a REQ_VERSION_INFO message is received, to request version info for the target device.
         * This is to be implemented by the concrete class.  If the target/requested device can not provide version info, this should return false.
         * If this call returns false, the API will respond with a MSG_VERSION_INFO_RESP, with the message filled with 0xFF, indicating not-supported.
         * NOTE that this call is passed a reference to a const dev_info_t; the base-class provides the instance which is referenced. As the implementer
         * of this class, it is your responsibility to fill it with the appropriate data.
         * @param a reference to a dev_info_t struct that contains the necessary version information to be returned back to the querying host.
         * @return true if the message was received and parsed without error, false otherwise.
         */
        virtual bool fwUpdate_queryVersionInfo(target_t target_id, dev_info_t& dev_info) = 0;

        /**
         * Initializes the system to begin receiving firmware image chunks for the target device, image slot and image size.
         * @param msg the message which contains the request data, such as slot, file size, chunk size, md5 checksum, etc.
         * @return an update_status_e indicating the continued state of the update process, or an error. For fwUpdate_startUpdate
         * this should return "GOOD_TO_GO" on success.
         */
        virtual update_status_e fwUpdate_startUpdate(const payload_t& msg) = 0;

        /**
         * Writes data (of len bytes) as a chunk of a larger firmware image to the target and device-specific image slot, and with the specified offset
         * @param target_id the target id
         * @param slot_id the image slot, if applicable (otherwise 0).
         * @param offset the offset into the slot to write this chunk
         * @param len the number of bytes in this chunk
         * @param data the chunk data
         * @return an update_status_e indicating the continued state of the update process, or an error. For fwUpdate_writeImageChunk
         * this should return "WAITING_FOR_DATA" if more chunks are expected, or an error.
         */
        virtual update_status_e fwUpdate_writeImageChunk(target_t target_id, int slot_id, int offset, int len, uint8_t *data) = 0;

        /**
         * Validated and finishes writing of the firmware image; that all image bytes have been received, the md5 sum passed, and the device can complete the requested upgrade, and perform any device-specific finalization.
         * @param target_id the target_id
         * @param slot_id the image slot, if applicable (otherwise 0)
         * @return
         */
        virtual update_status_e fwUpdate_finishUpdate(target_t target_id, int slot_id, int flags) = 0;


    protected:
        /**
         * This is an internal method used to send an update message to the host system regarding the status of the update process
         * This message only include the number of chunks sent, and the total expected (sufficient for a percentage) and the
         * @return true if the message was sent, false if there was an error
         */
        bool fwUpdate_sendProgress();

        /**
         * This is an internal method used to send an update message to the host system regarding the status of the update process
         * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
         * @param message the actual message to be sent to the host
         * @return true if the message was sent, false if there was an error
         */
        bool fwUpdate_sendProgress(int level, const std::string message);

        /**
         * This is an internal method used to send an update message to the host system regarding the status of the update process
         * This variation allows for printf-based string formatting
         * @param level the criticality/severity of this message (0 = Critical, 1 = Error, 2 = Warning, 3 = Info, 4 = Debug, etc)
         * @param message the actual message to be sent to the host
         * @
         * @return true if the message was sent, false if there was an error
         */
        bool fwUpdate_sendProgressFormatted(int level, const char *message, ...);

        /**
         * @return true if we have an active session and are updating.
         */
        bool fwUpdate_isUpdating();

        /**
         * @brief Notifies the host that the firmware update is complete for any reason, including an error.
         * This call does not generate a response or acknowledgement from the host.
         *
         * @param reason the specific reason the update was finished.
         * @param clear_session if true, causes the current session to be invalidated
         * @param reset_device if true, will call fwUpdate_performReset() after sending the message.
         * @return true if successfully sent, otherwise false.
         */
        bool fwUpdate_sendDone(update_status_e reason, bool clear_session, bool reset_device);

        /**
         * Internal method use to reinitialize the update engine.  This should clear the the current session_id, existing image data, running md5 sums, etc.
         * After calling this function, the subsystem must receive a REQ_UPDATE message to start to new session, etc.
         * This probably should be called after an update is finished, but is probably safest to call as the first step in a REQ_UPDATE.
         * @return true if the system was able to properly initialize, false if there was an error of something (you have a REAL problem in this case).
         */
        bool fwUpdate_resetEngine();

        /**
         * Internally called by fwUpdate_processMessage() when a REQ_UPDATE message is received.
         * @param payload the DID message
         * @return true if the message was received and parsed without error, false otherwise.
         *
         * NOTE, this function should call out and send an error status in the event of a failure.
         */
        bool fwUpdate_handleInitialize(const payload_t& payload);

        /**
         * Internally called by fwUpdate_processMessage() when a UPDATE_CHUNK message is received.
         * @param payload the DID payload
         * @return
         */
        bool fwUpdate_handleChunk(const payload_t& payload);

        /**
         * Internally called by fwUpdate_processMessage() when a REQ_RESET message is received, to reset the target MCU.
         * @param payload the DID message
         * @return true if the message was received and parsed without error, false otherwise.
         */
        bool fwUpdate_handleReset(const payload_t& payload);

        /**
         * Internally called by fwUpdate_processMessage() when a REQ_VERSION_INFO message is received, to request version info for the target device.
         * @param payload the DID message
         * @return true if the message was received and parsed without error, false otherwise.
         */
        bool fwUpdate_handleVersionInfo(const payload_t& payload);

        /**
         * Sends a REQ_RESEND_CHUNK message in response to receiving an invalid CHUNK message. This will ALWAYS send with the current session_id, and the last received chunk_id + 1;
         * @param reason for the resend
         * @return return true is a retry was sent, or false if a retry was not sent.  NOTE this is not an error, as a valid message will not send a retry.
         */
        bool fwUpdate_sendRetry(resend_reason_e reason);

        uint32_t progress_interval = 500;               // we'll send progress updates at 2hz.
        uint32_t nextProgressReport = 0;                // the next system
        int32_t last_chunk_id = -1;                     // the last received chunk id from a CHUNK message. -1 = no chunk yet received; the next received chunk must be 0.
    };

    class FirmwareUpdateHost : public FirmwareUpdateBase {
    public:
        /**
         * Creates a FirmwareUpdateSDK instance
         */
        FirmwareUpdateHost();
        virtual ~FirmwareUpdateHost() {};

        /**
         * Call this any time a DID_FIRMWARE_UPDATE is received by the comms system, to parse and process the message.
         * @param msg_payload the contents of the DID_FIRMWARE_UPDATE payload
         * @return true if this message was consumed by this interface, or false if the message was not intended for us, and should be passed along to other ports/interfaces.
         */
        bool fwUpdate_processMessage(const payload_t& msg_payload);
        bool fwUpdate_processMessage(const uint8_t* buffer, int buf_len);

        /**
         * Called by the host application to initiate a request by the SDK to update a target device.
         * @param target_id the target device to update
         * @param image_slot the "slot" on the target device which this image should be written to (device specific, if supported, otherwise 0)
         * @param chunk_size the size of each chunk used to transmit the image (smaller sizes take longer, larger sizes consume more memory and risk buffer overflows)
         * @param image_size the total number of bytes of the firmware image
         * @param image_md5 the md5 checksum of the firmware image
         * @param progress_rate the rate (in millis) which the device should send out progress updates
         * @return
         */
        bool fwUpdate_requestUpdate(target_t target_id, int image_slot, int image_flags, uint16_t chunk_size, uint32_t image_size, md5hash_t image_md5, int32_t progress_rate = 500);

        /**
         * Called by the hsot application to resend a previous "fwUpdate_requestUpdate" with a full parameter set.
         * @return
         */
        bool fwUpdate_requestUpdate();

        /**
         * Requests that the remote device perform a reset. Note that this request does not need a session, or any other pre-negotiated state.
         * @param target
         * @param reset_flags
         * @return true if the request was successfully sent
         */
        bool fwUpdate_requestReset(target_t target, uint16_t reset_flags);

        /**
         * Requests that the remote device respond with the devices current firmware and hardware version information.
         * @param target
         * @param dev_info
         * @return true if the request was successfully sent
         */
        bool fwUpdate_requestVersionInfo(target_t target);

        /**
         * Sends the next chunk of the firmware image to the remote side.  Internally, this call handles fetching the requested
         * data from the image file, through an implemented fwUpdate_getImageChunk(), which actually handles the fileIO, etc.
         * @return the number of remaining chunks that still need sending (returning 0 = all chunks sent).
         */
        int fwUpdate_sendNextChunk(void);

        /**
         * @return true if we have an active session and are updating.
         */
        bool fwUpdate_isUpdating();

        /**
         * @return a human-readable status name for the current session status
         */
        const char *fwUpdate_getSessionStatusName();

        /**
         * @return a human-readable name for the current session target device
         */
        const char *fwUpdate_getSessionTargetName();

        /**
         * @return the current session target device
         */
        target_t fwUpdate_getSessionTarget() { return session_target; }

        /**
         * @return the current session status
         */
        update_status_e fwUpdate_getSessionStatus() { return session_status; }

        /**
         * @return the current session id, or 0 if no session has been started
         */
        uint16_t fwUpdate_getSessionID() { return session_id; }

        /**
         * @return the currently target slot number for this session, or -1 if no session has been started
         */
        uint16_t fwUpdate_getSessionImageSlot() { return session_id != 0 ? session_image_slot : -1; }

        /**
         * @return the ID of the next chunk to be sent
         */
        uint16_t fwUpdate_getNextChunkID() { return next_chunk_id; }

        /**
         * @returns total number of complete chunks
        */
        uint16_t fwUpdate_getChunksSent() { return chunks_sent; }

        /**
         * @return the negotiated chunk size for this session
         */
        uint16_t fwUpdate_getChunkSize() { return session_chunk_size; }

        /**
         * @return the total number of chunks negotiated for this session, determined by the image size
         */
        uint16_t fwUpdate_getTotalChunks() { return session_total_chunks; }

        /**
         * @return the size of the firmware image for this session
         */
        uint32_t fwUpdate_getImageSize() { return session_image_size; }

        /**
         * @return the number of chunks that the remote device has specifically requested be resent
         */
        uint16_t fwUpdate_getResendCount() { return resend_count; }

        /**
         * @return a percentage of the number of resent chunks vs total chunks sent (indicates an error rate)
         */
        float fwUpdate_getResendRate() { return (chunks_sent > 0) ? ((float)resend_count / (float)chunks_sent) : 0.f; }


    protected:
        //===========  Functions which MUST be implemented ===========//

        /**
         * To be implemented by the concrete class, this function is called by the underlying API into the concrete implementation,
         * to transmit data onto "the wire".
         * @param target the device which this data is targeting; useful if the implementation has specific ports for specific targets
         * @param buffer the data to be send
         * @param buff_len the length of the data to be sent
         * @return returns true if the data was successfully sent, otherwise false
         */
        virtual bool fwUpdate_writeToWire(target_t target, uint8_t* buffer, int buff_len) = 0;

        /**
         * To be implemented by the concrete class, this method loads the next image chunk from disk (or whereever) and
         * places it into a buffer, so the API can package it into a payload, and send it over the wire.
         * This method is not concerned with anything than "get bytes n-n+x, and load them into *buffer".
         * @param offset the offset in the file from which next chunk of bytes should begin (use this to fseek, etc)
         * @param len the number of bytes that should be loaded (this should be session_chunk_size, unless its the final chunk)
         * @param buffer a pointer to a block of memory that is allocated by the caller, to which the data should be placed.
         *   - Do not retain this pointer, as it is not guaranteed to be in scope after this function returns.
         * @return the actual number of bytes loaded into the buffer. Any other value other than the requested len, will likely
         * result in an error.  If an error is encountered reading the data, you should return a negative value here.
         */
        virtual int fwUpdate_getImageChunk(uint32_t offset, uint32_t len, void **buffer) = 0;

        /**
         * To be implemented by the concrete class, this method provides the response to a VERSION_INFO requests for a target
         * device. Note that this is not session dependent, and can be received at any time (though usually before an UPDATE_REQ
         * is made).
         */
        virtual bool fwUpdate_handleVersionResponse(const payload_t& msg) = 0;

        /**
         * To be implemented by the concrete class, this method is called when the API receives a response to an update request.
         * This message primarily contains error information, or session details if the request is valid.
         * @param msg the payload message for the update response
         * @return true if this message was properly handled (regardless of error, etc), otherwise false
         */
        virtual bool fwUpdate_handleUpdateResponse(const payload_t& msg) = 0;

        /**
         * To be implemented by the concrete class, this method is called when the API receives a request from the remote
         * to resend a specific chunk. Normally, this message is handled internally, but can be used to report errors on
         * the remote side, which could drive specific behaviors (such as a flash write error, vs a dropped chunk on the wire)
         * @param msg the payload message for the resend request
         * @return true if this message was properly handled (regardless of error, etc), otherwise false
         */
        virtual bool fwUpdate_handleResendChunk(const payload_t& msg) = 0;

        /**
         * To be implemented by the concrete class, this method is called when the API receives a progress message from
         * the remote device. These messages are informational only, and can generally be ignored, but maybe useful for
         * communicating status messages and progress back to the user.
         * @param msg the progress payload message
         * @return true if this message was properly handled (regardless of error, etc), otherwise false
         */
        virtual bool fwUpdate_handleUpdateProgress(const payload_t& msg) = 0;

        /**
         * To be implemented by the concrete class, this method is called when the API receives DONE message from the
         * remote device indicating the completion (success or error) of the Update process. This effectively denotes
         * that the session is closed, and a new session will have to be initiated. This message is informational only,
         * and can generally be ignored, but maybe useful for communicating status messages and progress back to the user.
         * @param msg the progress payload message
         * @return true if this message was properly handled (regardless of error, etc), otherwise false
         */
        virtual bool fwUpdate_handleDone(const payload_t& msg) = 0;

        /**
         * Internal method use to reinitialize the update engine.  This should clear the the current session_id, existing image data, running md5 sums, etc.
         * After calling this function, the subsystem must receive a REQ_UPDATE message to start to new session, etc.
         * This probably should be called after an update is finished, but is probably safest to call as the first step in a REQ_UPDATE.
         * @return true if the system was able to properly initialize, false if there was an error of something (you have a REAL problem in this case).
         */
        bool fwUpdate_resetEngine();

        uint16_t next_chunk_id = 0;                     //! the next chuck id to send, at the next send.
        uint16_t chunks_sent = 0;                       //! the total number of chunks that have been sent, including resends
    };

} // fwUpdate

#ifdef __cplusplus
}
#endif

#endif //IS_FIRMWAREUPDATE_H
