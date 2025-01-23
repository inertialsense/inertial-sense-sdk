/**
 * @file ISBFirmwareUpdater.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 5/29/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_ISB_FIRMWAREUPDATER_H
#define IS_ISB_FIRMWAREUPDATER_H

#include <mutex>
#include <deque>

#include "serialPort.h"
#include "ISFirmwareUpdater.h"
#include "protocol/FirmwareUpdate.h"
#include "util/util.h"


// Delete this and assocated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
#define SUPPORT_BOOTLOADER_V5A

/** uINS bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER 921600

// #define BOOTLOADER_RETRIES          100
// #define BOOTLOADER_RESPONSE_DELAY   10
#define BOOTLOADER_REFRESH_DELAY            500
#define MAX_VERIFY_CHUNK_SIZE               1024
#define BOOTLOADER_TIMEOUT_DEFAULT          1000
#define MAX_SEND_COUNT                      510

// logical page size, offsets for pages are 0x0000 to 0xFFFF - flash page size on devices will vary and is not relevant to the bootloader client
#define FLASH_PAGE_SIZE                     65536

class ISBFirmwareUpdater : public fwUpdate::FirmwareUpdateDevice {

public:
    /**
     * Constructor to establish a connected to the specified device, and optionally validating against hdwId and serialNo
     * @param device the libusb device which identifies the connected device to update. This is NOT a libusb_device_handle! If null, this function will use to first detected DFU device which matches the hdwId and/or serialNo
     * @param hdwId the hardware id (from manufacturing info) used to identify which specific hdwType + hdwVer we should be targeting (used in validation)
     * @param serialNo the device-specific unique Id (or serial number) that is used to uniquely identify a particular device (used in validation)
     */
    ISBFirmwareUpdater(fwUpdate::target_t target, const ISDevice* device, std::deque<uint8_t>& toHost) : FirmwareUpdateDevice(target), device((ISDevice*)device), toHost(toHost) {
        // uint16_t hdwId = (target & fwUpdate::TARGET_IMX5 ? ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0)  : ENCODE_HDW_ID(IS_HARDWARE_TYPE_UINS, 3, 2));
        // as soon as this is instantiated, we should attempt to target and boot the device into ISB mode.
        // rebootToISB(5, 0, false);
    }
    ~ISBFirmwareUpdater() override = default;

    /**
     * We override this in this class, because we have to do some better handling for erase/write, rather than chunks.
     * @param level
     * @param message
     * @param ...
     * @return
     */
    bool fwUpdate_sendProgressFormatted(int level, const char* message, ...) override;

    bool fwUpdate_sendProgressFormatted(int level, int total_chunks, int num_chunks, const char* message, ...);

    // this is called internally by processMessage() to do the things; it should also be called periodically to send status updated, etc.
    bool fwUpdate_step(fwUpdate::msg_types_e msg_type, bool processed) override;

    // called internally to perform a system reset of various severity per reset_flags (HARD, SOFT, etc)
    /**
     * Performs a system reset of various severity per reset_flags, (ie, RESET_SOFT by informing the OS/MCU to restart the system,
     * vs RESET_HARD, usually by pulling interfacing pins into the MCU either HIGH or LOW to force a reset state on the hardware).
     * Note that some systems may not always be able to respond with a success before the system is reset.
     * If a system is NOT able to perform a reset (ie UNSUPPORTED, etc), this MUST return false.
     * @param target_id the device to reset
     * @return true if successful, otherwise false
     */
    int fwUpdate_performReset(fwUpdate::target_t target_id, fwUpdate::reset_flags_e reset_flags) override;

    // called internally (by the receiving device) to populate the dev_info_t struct for the requested device
    /**
     * Internally called by fwUpdate_processMessage() when a REQ_VERSION_INFO message is received, to request version info for the target device.
     * This is to be implemented by the concrete class.  If the target/requested device can not provide version info, this should return false.
     * If this call returns false, the API will respond with a MSG_VERSION_INFO_RESP, with the message filled with 0xFF, indicating not-supported.
     * NOTE that this call is passed a reference to a const dev_info_t; the base-class provides the instance which is referenced. As the implementer
     * of this class, it is your responsibility to fill it with the appropriate data.
     * @param a reference to a dev_info_t struct that contains the necessary version information to be returned back to the querying host.
     * @return true if the message was received and parsed without error, false otherwise.
     */
    bool fwUpdate_queryVersionInfo(fwUpdate::target_t target_id, dev_info_t& dev_info) override;

    /**
     * Initializes the system to begin receiving firmware image chunks for the target device, image slot and image size.
     * @param msg the message which contains the request data, such as slot, file size, chunk size, md5 checksum, etc.
     * @return an update_status_e indicating the continued state of the update process, or an error. For fwUpdate_startUpdate
     * this should return "GOOD_TO_GO" on success.
     */
    // this initializes the system to begin receiving firmware image chunks for the target device, image slot and image size
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
    // writes the indicated block of data (of len bytes) to the target and device-specific image slot, and with the specified offset
    fwUpdate::update_status_e fwUpdate_writeImageChunk(fwUpdate::target_t target_id, int slot_id, int offset, int len, uint8_t *data) override;

    /**
     * Validated and finishes writing of the firmware image; that all image bytes have been received, the md5 sum passed, and the device can complete the requested upgrade, and perform any device-specific finalization.
     * @param target_id the target_id
     * @param slot_id the image slot, if applicable (otherwise 0)
     * @return
     */
    // this marks the finish of the upgrade, that all image bytes have been received, the md5 sum passed, and the device can complete the requested upgrade, and perform any device-specific finalization
    fwUpdate::update_status_e fwUpdate_finishUpdate(fwUpdate::target_t target_id, int slot_id, int flags) override;

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
        IS_PROCESSOR_STM32U5,           // GPX-1, IMX-5.1

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

    ISDevice* device;

    uint32_t m_sn = 0;                      //!< Inertial Sense serial number, i.e. SN60000
    uint16_t hardwareId = 0;                //!< Inertial Sense Hardware Type (IMX, GPX, etc)
    uint8_t m_isb_major = 0;                //!< ISB Major revision on device
    char m_isb_minor = 0;                   //!< ISB Minor revision on device
    bool isb_mightUpdate = false;           //!< true if device will be updated if bootloader continues

    fwUpdate::payload_t lastPayload{};      //!< the last receive payload (uses during initialization, if having to switch to bootloader mode, etc)

    struct {
        bool is_evb = false;                //!< Available on version 6+, otherwise false
        int processor = 0;                  //!< Differentiates between uINS-3 and IMX-5
        bool rom_available = 0;             //!< ROM bootloader is available on this port

        uint32_t app_offset = 0;            //!< Helps in loading bin files
        uint32_t verify_size = 0;           //!< Chunk size, limited on Windows
    } m_isb_props = {};

    static std::vector<uint32_t> serial_list;
    static std::mutex serial_list_mutex;

    static std::vector<uint32_t> rst_serial_list;
    static std::mutex rst_serial_list_mutex;

    fwUpdate::target_t getTargetType();

    bool rebootToRomDfu();
    bool rebootToISB();
    bool rebootToAPP(bool keepPortOpen = false);

    bool sendCmd(const std::string& cmd, int chksumPos = -1);
    bool waitForAck(const std::string& ackStr, const std::string& progressMsg, uint32_t maxTimeout, uint32_t& elapsed, float& progress);

    is_operation_result sync();
    uint32_t get_device_info();
    eImageSignature check_is_compatible();
    int checksum(int checkSum, uint8_t* ptr, int start, int end, int checkSumPosition, int finalCheckSum);
    is_operation_result select_page(int page);
    is_operation_result begin_program_for_current_page(int startOffset, int endOffset);
    int is_isb_read_line(ByteBufferStream& byteStream, char line[HEX_BUFFER_SIZE]);
    is_operation_result upload_hex_page(unsigned char* hexData, uint8_t byteCount);
    is_operation_result upload_hex(unsigned char* hexData, uint16_t charCount);
    is_operation_result fill_current_page();
    is_operation_result process_hex_stream(ByteBufferStream& byteStream);

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
        UPDATE_DONE = 4
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

    unsigned char output[HEX_BUFFER_SIZE * 2]{}; // big enough to store an entire extra line of buffer if needed
    const unsigned char* outputPtrEnd = output + (HEX_BUFFER_SIZE * 2);
    unsigned char* outputPtr = output;

    int lastSubOffset = -1;
    int subOffset = 0;


    std::deque<uint8_t>& toHost;

    eraseState_t eraseFlash_step(uint32_t timeout = 60000);
    writeState_t writeFlash_step(uint32_t timeout = 60000);
};

#endif //IS_ISB_FIRMWAREUPDATER_H
