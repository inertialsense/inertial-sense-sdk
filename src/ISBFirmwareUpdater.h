/**
 * @file ISBFirmwareUpdater.h
 * @brief Device-side fwUpdate::FirmwareUpdateDevice implementation for the ISB (Inertial Sense
 *        Bootloader) legacy update protocol -- page-oriented erase/write/verify of an Intel-HEX
 *        image over the ISB's ASCII command/ack wire protocol.
 *
 * @author Kyle Mallory on 5/29/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_ISB_FIRMWAREUPDATER_H
#define IS_ISB_FIRMWAREUPDATER_H

#include <mutex>
#include <deque>

#include "PortManager.h"
#include "DeviceManager.h"
#include "ISFirmwareUpdater.h"
#include "protocol/FirmwareUpdate.h"
#include "util/util.h"

//#define BOOTLOADER_RETRIES                  12
//#define BOOTLOADER_RESPONSE_DELAY           10
#define BOOTLOADER_REFRESH_DELAY            500    //!< minimum interval (ms) between successive ISB step refreshes
#define MAX_VERIFY_CHUNK_SIZE               1024    //!< maximum number of bytes read back per verify-chunk request
#define BOOTLOADER_TIMEOUT_DEFAULT          1000    //!< default timeout (ms) for an ISB command/ack exchange
#define MAX_SEND_COUNT                      510     //!< maximum number of bytes sent per ISB write command
/*
 * ISB REPLY ALPHABET, and a TODO to decode it.
 *
 * The bootloader answers every command with a single character followed by CRLF, from one place --
 * send_answer_frame(), cpp/hdw-src/bootloader/common/embeddedBootloader.c:644. Verified emitted:
 *
 *   ".\r\n"   OK.
 *   "X\r\n"   Checksum error. By far the most reachable failure: every command group validates a
 *              checksum (embeddedBootloader.c:841, 947, 1014, 1058, 1158, 1185). The link is alive and
 *              one command was corrupted, so this is the ONE reply for which retrying the page is the
 *              right response rather than failing the session.
 *   "P\r\n"   Write error or bad address (:856, :878 errWRITE; :711, :1196 errADDRESS). Genuine flash
 *              or addressing failure; not retryable.
 *   "K\r\n"   Unknown OR UNEXPECTED command (:959, :1036, :1201, :1207, dispatcher default :1239;
 *              errUNEXPECTED :939, :975). Unexpected means a state-machine violation -- e.g.
 *              CMD_PROGRAM_START while not IDLE -- i.e. the host and device have desynchronised. Also
 *              send_answer_frame()'s fallback when no status bit is set at all, which includes
 *              DFU_CMD_STATUS_OK_NORETCHAR (0x0), since testing that against DFU_CMD_STATUS_OK is false.
 *
 * Enumerated in the firmware but NOT emitted, so not worth handling until that changes:
 *   "L\r\n"   Security bit set. Dead code: both call sites are `if (security_active)` and
 *              embeddedBootloader.c:498 assigns security_active = 0 with the real check commented out.
 *   "NNNN\r\n" Blank-check failure, 4 hex digits of the failing page offset -- the only variable-length
 *              reply. Reachable solely via CMD_BLANK_CHECK (CMD_GRP_UPLOAD), which no SDK path sends.
 *
 * TODO: the SDK recognises ONLY ".\r\n". waitForAck() slides past any non-matching byte hunting for it,
 * and portWaitForTimeout() does the same, so X/P/K are silently discarded and every failure degrades into
 * the same generic BOOTLOADER_TIMEOUT_DEFAULT timeout with no cause reported. Read the CRLF-terminated
 * frame and classify the character instead. 11 call sites across ISBFirmwareUpdater.cpp (x6) and
 * ISBootloaderISB.cpp (x5, two of which hand-roll memcmp(buf, ".\r\n", 3)), so it wants one shared
 * helper rather than two parallel fixes. Order of value: X (report + retry the page), K (report the
 * desync -- it is what a pipelining bug looks like from the wire), P (report; not retryable).
 */

/** 1 to collect a page write's ack at the start of the NEXT step instead of blocking for it, so the
 *  round-trip overlaps whatever the step loop does in between. 0 restores the blocking write/ack pairing
 *  (useful for A/B measurement). Worth roughly a 2.5x reduction in total time on a multi-device bench. */
#define ISB_PIPELINED_PAGE_WRITES           1

// logical page size, offsets for pages are 0x0000 to 0xFFFF - flash page size on devices will vary and is not relevant to the bootloader client
#define FLASH_PAGE_SIZE                     65536   //!< logical page size used for ISB page offsets (0x0000-0xFFFF); independent of the device's actual physical flash page size

/**
 * Device-side fwUpdate::FirmwareUpdateDevice implementation for the ISB (Inertial Sense
 * Bootloader) legacy update protocol: erases, writes, and verifies an Intel-HEX image page by
 * page over the ISB's ASCII command/ack wire protocol.
 */
class ISBFirmwareUpdater : public fwUpdate::FirmwareUpdateDevice {

public:
    /**
     * Constructor to manage an ISB-protocol firmware update of the specified target on the given device.
     * @param target the fwUpdate target device this updater instance is responsible for
     * @param device the connected device the target resides on
     * @param toHost a shared byte-stream queue that outgoing responses/acks are written to, to be sent back to the host
     */
    ISBFirmwareUpdater(fwUpdate::target_t target, const device_handle_t& device, std::deque<uint8_t>& toHost) : FirmwareUpdateDevice(target), device(device), target_devInfo(), toHost(toHost) { }


    ~ISBFirmwareUpdater() override {
        if (imgBuffer) delete imgBuffer;
        if (imgStream) delete imgStream;
    };

    /**
     * Formats and reports a progress/status message. Overridden here because ISB reports progress
     * per erase/write/verify page step rather than per fixed-size chunk, so the base class's
     * chunk-count formatting doesn't apply.
     * @param level the severity of the message (one of IS_LOG_LEVEL_*)
     * @param message a printf-style format string
     * @param ... format arguments for message
     * @return true if the message was successfully formatted and reported, otherwise false
     */
    bool fwUpdate_sendProgressFormatted(int level, const char* message, ...) override;

    /**
     * Formats and reports a progress/status message that additionally carries an explicit
     * chunk-count pair, for steps (e.g. page verify) that report progress against a page/chunk
     * count that differs from the base class's own bookkeeping.
     * @param level the severity of the message (one of IS_LOG_LEVEL_*)
     * @param total_chunks the total number of chunks/pages for the current step
     * @param num_chunks the number of chunks/pages completed so far
     * @param message a printf-style format string
     * @param ... format arguments for message
     * @return true if the message was successfully formatted and reported, otherwise false
     */
    bool fwUpdate_sendProgressFormatted(int level, int total_chunks, int num_chunks, const char* message, ...);

    /**
     * Drives the ISB update state machine forward by one step; called internally by
     * processMessage() when a message is received, and should also be called periodically
     * (independent of message reception) so timeouts and page erase/write/verify progress
     * continue to advance.
     * @param msg_type the type of message that was last processed, or MSG_UNKNOWN
     * @param processed true if msg_type was already handled by the caller (additional/optional
     *        processing may still occur here), false if it was not yet handled
     * @return true if some action was taken as a result of this step, otherwise false
     */
    bool fwUpdate_step(fwUpdate::msg_types_e msg_type, bool processed) override;

    /**
     * Performs a system reset of various severity per reset_flags, (ie, RESET_SOFT by informing the OS/MCU to restart the system,
     * vs RESET_HARD, usually by pulling interfacing pins into the MCU either HIGH or LOW to force a reset state on the hardware).
     * Note that some systems may not always be able to respond with a success before the system is reset.
     * If a system is NOT able to perform a reset (ie UNSUPPORTED, etc), this MUST return false.
     * @param target_id the device to reset
     * @param reset_flags the severity/style of reset to perform (e.g. RESET_SOFT, RESET_HARD)
     * @return true if successful, otherwise false
     */
    bool fwUpdate_performReset(fwUpdate::target_t target_id, fwUpdate::reset_flags_e reset_flags) override;

    /**
     * Internally called by fwUpdate_processMessage() when a REQ_VERSION_INFO message is received, to request version info for the target device.
     * This is to be implemented by the concrete class.  If the target/requested device can not provide version info, this should return false.
     * If this call returns false, the API will respond with a MSG_VERSION_INFO_RESP, with the message filled with 0xFF, indicating not-supported.
     * NOTE that this call is passed a reference to a const dev_info_t; the base-class provides the instance which is referenced. As the implementer
     * of this class, it is your responsibility to fill it with the appropriate data.
     * @param target_id the device whose version info is being requested
     * @param dev_info reference to a dev_info_t struct to fill with the version information to be returned back to the querying host
     * @return true if the message was received and parsed without error, false otherwise.
     */
    bool fwUpdate_queryVersionInfo(fwUpdate::target_t target_id, dev_info_t& dev_info) override;

    /**
     * Initializes the system to begin receiving firmware image chunks for the target device, image slot and image size.
     * @param msg the message which contains the request data, such as slot, file size, chunk size, md5 checksum, etc.
     * @return an update_status_e indicating the continued state of the update process, or an error. For fwUpdate_startUpdate
     * this should return "GOOD_TO_GO" on success.
     */
    fwUpdate::update_status_e fwUpdate_startUpdate(const fwUpdate::payload_t& msg) override;

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
    fwUpdate::update_status_e fwUpdate_writeImageChunk(fwUpdate::target_t target_id, int slot_id, int offset, int len, uint8_t *data) override;

    /**
     * Validates and finishes writing of the firmware image; that all image bytes have been received, the md5 sum passed, and the device can complete the requested upgrade, and perform any device-specific finalization.
     * @param target_id the target_id
     * @param slot_id the image slot, if applicable (otherwise 0)
     * @param flags additional flags controlling finalization behavior
     * @return an update_status_e indicating the continued state of the update process, or an error
     */
    fwUpdate::update_status_e fwUpdate_finishUpdate(fwUpdate::target_t target_id, int slot_id, int flags) override;

    /**
     * Writes the requested data (usually a packed payload_t) out to the specified device.
     * Note that the implementation between a target and an actual interface is device-specific. In most cases,
     * for a Device-implementation, this will typically specify TARGET_HOST, which will direct back to the
     * controlling host.
     * @param target the target this message is directed to
     * @param buffer the encoded buffer to send
     * @param buff_len the number of bytes in the encoded buffer to send
     * @return true if the data was successfully sent to the underlying communication system, otherwise false
     */
    bool fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t* buffer, int buff_len) override;

private:
    typedef enum {
        IS_DEV_TYPE_NONE = 0,
        IS_DEV_TYPE_SAMBA,
        IS_DEV_TYPE_ISB,
        IS_DEV_TYPE_APP,
        IS_DEV_TYPE_DFU,
    } eDeviceType;

    typedef enum {
        IS_PROCESSOR_UNKNOWN = -1,
        IS_PROCESSOR_SAMx70 = 0,        // uINS-3/4, EVB-2
        IS_PROCESSOR_STM32L4,           // IMX-5
        IS_PROCESSOR_STM32U5,           // GPX-1, IMX-6.0

        IS_PROCESSOR_NUM,               // Must be last
    } eProcessorType;

    typedef enum {
        // Bootloaders must be first because bootloaders may contain app signatures
        IS_IMAGE_SIGN_ISB_STM32L4 = 0x00000001,
        IS_IMAGE_SIGN_ISB_SAMx70_16K = 0x00000002,
        IS_IMAGE_SIGN_ISB_SAMx70_24K = 0x00000004,

        IS_IMAGE_SIGN_UINS_3_16K = 0x00000008,
        IS_IMAGE_SIGN_UINS_3_24K = 0x00000010,
        IS_IMAGE_SIGN_EVB_2_16K = 0x00000020,
        IS_IMAGE_SIGN_EVB_2_24K = 0x00000040,
        IS_IMAGE_SIGN_IMX_5p0 = 0x00000080,

        IS_IMAGE_SIGN_NUM_BITS_USED = 8,

        IS_IMAGE_SIGN_APP = IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K | IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_IMX_5p0 | IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4,
        IS_IMAGE_SIGN_ISB = IS_IMAGE_SIGN_UINS_3_16K | IS_IMAGE_SIGN_UINS_3_24K | IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K | IS_IMAGE_SIGN_IMX_5p0 | IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K | IS_IMAGE_SIGN_ISB_STM32L4,

        IS_IMAGE_SIGN_SAMBA = IS_IMAGE_SIGN_ISB_SAMx70_16K | IS_IMAGE_SIGN_ISB_SAMx70_24K,
        IS_IMAGE_SIGN_DFU = IS_IMAGE_SIGN_ISB_STM32L4,

        IS_IMAGE_SIGN_EVB = IS_IMAGE_SIGN_EVB_2_16K | IS_IMAGE_SIGN_EVB_2_24K,

        IS_IMAGE_SIGN_NONE = 0,
        IS_IMAGE_SIGN_ERROR = 0x80000000,
    } eImageSignature;

    static const int HEX_BUFFER_SIZE = 1024;

    inline static PortManager& portManager = PortManager::getInstance();
    inline static DeviceManager& deviceManager = DeviceManager::getInstance();

    device_handle_t device;                 //!< an ISDevice instance to which are are communicating/updating
    dev_info_t target_devInfo;              //!< the original devInfo of the ISDevice above, used in future validations between reboots, etc.
    uint64_t lastMissingTargetId = 0;       //!< last target unique-id that fwUpdate_step() failed to locate; throttles the (verbose) lookup-failure diagnostic to one report per distinct id
    uint32_t last_reboot = 0;               //!< time when the last reboot to the device was issued
    uint32_t isblPhaseStartMs = 0;          //!< when the wait for ISbl FIRST began; unlike last_reboot this survives
                                            //!< the phase's own retries, so it can bound the whole phase
    uint32_t nextStepMs = 0;                //!< if this time exceeds the current clock (current_timeMs()) fwUpdate_step() will return immediately until this elapses

    struct {
        bool isValid = false;               //!< true if the data in this struct is valid
        bool is_evb = false;                //!< Available on version 6+, otherwise false
        int processor = 0;                  //!< Differentiates between uINS-3 and IMX-5
        bool rom_available = 0;             //!< ROM bootloader is available on this port

        uint32_t app_offset = 0;            //!< Helps in loading bin files
        uint32_t verify_size = 0;           //!< Chunk size, limited on Windows
    } m_isb_props = {};                     //!< Essential properties for the ISbootloader to properly install an APP firmware image


    static std::vector<uint32_t> serial_list;
    static std::vector<uint32_t> rst_serial_list;

    fwUpdate::target_t getTargetType();

    bool rebootToRomDfu();
    bool rebootToISB();
    bool rebootToAPP(bool keepPortOpen = false);

    bool sendCmd(const std::string& cmd, int chksumPos = -1);
    bool waitForAck(const std::string& ackStr, const std::string& progressMsg, uint32_t maxTimeout, uint32_t& elapsed, float& progress);

    // No local sync()/handshake here on purpose. The ISbl autobaud burst lives in
    // ISDevice::handshakeISbl(), and the negotiation that decides whether a burst is even needed lives in
    // ISDevice::queryIsblVersionFrame() -- which this class reaches through device->queryDeviceInfoISbl().
    uint32_t get_device_info();
    eImageSignature check_is_compatible();
    is_operation_result fetch_device_info_and_signature(eImageSignature* out_signature = nullptr);
    int checksum(int checkSum, uint8_t* ptr, int start, int end, int checkSumPosition, int finalCheckSum);
    is_operation_result select_page(int page);
    is_operation_result begin_program_for_current_page(int startOffset, int endOffset);
    int is_isb_read_line(ByteBufferStream& byteStream, char line[HEX_BUFFER_SIZE]);
    /**
     * Writes one page of hex payload to the bootloader.
     * @param hexData the ASCII-hex payload for this page
     * @param byteCount the number of BYTES (half the hex characters) this page carries
     * @param pipelined true to return as soon as the write is issued, leaving its acknowledgement to be
     *        collected by reapPendingPageWrite(); false to block for the acknowledgement and commit
     *        inline. Callers that emit several pages in one pass must use false, since each page's
     *        header is built from the committed currentOffset.
     */
    is_operation_result upload_hex_page(unsigned char* hexData, uint8_t byteCount, bool pipelined = false);
    is_operation_result upload_hex(unsigned char* hexData, uint16_t charCount, bool& dataSent);
    is_operation_result fill_current_page();
    is_operation_result process_hex_stream(ByteBufferStream& byteStream);

    /**
     * Scans the buffered Intel-HEX image for the highest absolute flash address that will be
     * written. The .hex text size is unrelated to the resulting flash extent (records are ASCII,
     * and process_hex_stream() coalesces records and gap-fills pages), so the records must be
     * parsed: the current Extended-Linear-Address (type 0x04) supplies the page base and each
     * data (type 0x00) record contributes abs = (ela<<16)|offset, end = abs + byteCount.
     * @return the exclusive top absolute flash address written by the image (0 if unavailable).
     */
    uint32_t calcFinalImageTopAddress();

    /**
     * @return the exclusive top absolute address of writable APP flash for the currently-targeted
     * device, based on its bootloader version. Used to reject an oversized image before erasing.
     */
    uint32_t getWritableFlashTopAddress();

    is_operation_result download_data(int startOffset, int endOffset);

    ByteBuffer* imgBuffer = nullptr;
    ByteBufferStream* imgStream = nullptr;

    uint8_t rxWorkBuf[128]{};
    uint8_t *rxWorkBufPtr =rxWorkBuf;

    bool doVerify = false;
    typedef enum : uint8_t {
        UPLOADING = 0,
        ERASING = 1,
        WRITING = 2,
        VERIFYING = 3,
        REBOOT_TO_APP = 4,
        UPDATE_DONE = 5
    } updateState_t;
    updateState_t updateState = UPLOADING;
    float transferProgress = 0.f;       // the percentage complete of the data transfer step
    uint32_t transferTimeout = 0;

    typedef enum : int8_t {
        ERASE_TIMEOUT = -2,
        ERASE_ERROR = -1,
        ERASE_INITIALIZE = 0,
        ERASE = 1,
        ERASE_FINALIZE = 2,
        ERASE_DONE = 3,
    } eraseState_t;
    eraseState_t eraseState = ERASE_INITIALIZE;
    float eraseProgress = 0.f;          // the percentage complete of the erase step
    uint32_t eraseStartedMs = 0;        // the timestamp (ms) when the erase step was started
    uint32_t eraseElapsed = 0;          // milliseconds elapsed since the start of the erase step

    typedef enum : int8_t {
        WRITE_TIMEOUT = -2,
        WRITE_ERROR = -1,
        WRITE_INITIALIZE = 0,
        WRITE = 1,
        WRITE_FINALIZE = 2,
        WRITE_DONE = 3,
    } writeState_t;
    writeState_t writeState = WRITE_INITIALIZE;
    float writeProgress = 0.f;          // the percentage complete of the write step
    uint32_t writeTimeout = 0;

    typedef enum : int8_t {
        VERIFY_IMAGE_MISMATCH = -3,
        VERIFY_TIMEOUT = -2,
        VERIFY_ERROR = -1,
        VERIFY_INITIALIZE = 0,
        VERIFY_VERIFY = 1,
        VERIFY_FINALIZE = 2,
        VERIFY_FINISHED = 3,
    } verifyState_t;
    verifyState_t verifyState = VERIFY_INITIALIZE;
    float verifyProgress = 0.f;          // the percentage complete of the verify step
    uint32_t verifyTimeout = 0;

    int currentPage = -1;
    int currentOffset = 0;
    int totalBytes = 0;
    int verifyCheckSum = 0;

    /**
     * A page write that has been sent but whose ".\r\n" acknowledgement has not yet been collected.
     *
     * Retains everything needed to (a) commit the write once it is acknowledged and (b) name it exactly
     * if it is NAKed or times out -- the write is no longer on the stack when its outcome is known, so
     * without this a failure could only be reported against whatever the stream had moved on to.
     */
    struct pendingPageWrite_t {
        bool     active = false;        //!< true while an issued page write awaits acknowledgement
        uint32_t seq = 0;               //!< monotonic page-write number, for traceability in messages
        int      page = 0;              //!< currentPage the write was issued against
        int      offset = 0;            //!< currentOffset the write was issued at
        uint8_t  byteCount = 0;         //!< payload byte count of the write
        int      verifyCheckSum = 0;    //!< running verify checksum INCLUDING this write; committed on ACK
        uint32_t deadline = 0;          //!< current_timeMs() by which the ACK must have arrived
        uint32_t elapsed = 0;           //!< waitForAck() progress accumulator
    } pendingWrite;
    uint32_t pageWriteSeq = 0;          //!< number of page writes issued this session

    /**
     * Collects the acknowledgement for an outstanding page write, committing it on success.
     * @return IS_OP_OK acknowledged and committed; IS_OP_RETRY not yet arrived, try again next step;
     *         IS_OP_ERROR the write's deadline expired
     */
    is_operation_result reapPendingPageWrite();

    /**
     * Blocks until an outstanding page write is acknowledged, so a caller may then perform a blocking
     * ISB exchange of its own.
     *
     * Any blocking exchange issued while a pipelined write is outstanding consumes THAT write's
     * acknowledgement instead of its own, and every subsequent exchange stays one ack ahead of itself
     * until the last one has none left and times out. Only page-boundary work (select/begin-program/fill)
     * takes this path, a handful of times per image, so blocking here costs nothing measurable.
     *
     * @return IS_OP_OK once nothing is outstanding, or IS_OP_ERROR if the write's deadline expired
     */
    is_operation_result drainPendingPageWrite();

    unsigned char output[HEX_BUFFER_SIZE * 2]{}; // big enough to store an entire extra line of buffer if needed
    const unsigned char* outputPtrEnd = output + (HEX_BUFFER_SIZE * 2);
    unsigned char* outputPtr = output;

    PortManager::port_listener_handle_t portListenerHandle;
    bool portsChanged = false;

    int lastSubOffset = -1;

    fwUpdate::update_status_e last_session_status = fwUpdate::NOT_STARTED;

    std::deque<uint8_t>& toHost;
    uint32_t nextPortCheck = 0;

    eraseState_t eraseFlash_step(uint32_t timeout = 20000);
    writeState_t writeFlash_step(uint32_t timeout = 20000);
};

#endif //IS_ISB_FIRMWAREUPDATER_H
