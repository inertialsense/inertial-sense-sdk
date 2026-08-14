/**
 * @file ISDFUFirmwareUpdater.h
 * @brief USB DFU (Device Firmware Update) support: DFUDevice wraps a single libusb DFU device
 *        (open/erase/write/verify/finalize over STM32's DfuSe extensions), and
 *        ISDFUFirmwareUpdater adapts that to the fwUpdate::FirmwareUpdateDevice interface used
 *        by the rest of the firmware update stack, plus static device enumeration/discovery.
 *
 * @author Kyle Mallory on 11/28/23.
 * @copyright Copyright (c) 2023 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_DFU_FIRMWAREUPDATER_H
#define IS_DFU_FIRMWAREUPDATER_H

#include "protocol/FirmwareUpdate.h"

#include <functional>
#include <mutex>
#include <queue>

#include "ihex.h"
#include "ISUtilities.h"
#include "util/md5.h"
#include "util/util.h"

#include "libusb.h"


#ifdef _MSC_VER
# pragma pack(push)
# pragma pack(1)
#endif /* _MSC_VER */
#define USB_DFU_CAN_DOWNLOAD    (1 << 0)
#define USB_DFU_CAN_UPLOAD    (1 << 1)
#define USB_DFU_MANIFEST_TOL    (1 << 2)
#define USB_DFU_WILL_DETACH    (1 << 3)
struct usb_dfu_func_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bmAttributes;
    uint16_t wDetachTimeOut;
    uint16_t wTransferSize;
    uint16_t bcdDFUVersion;
#ifdef _MSC_VER
    };
# pragma pack(pop)
#elif defined __GNUC__
# if defined __MINGW32__
    } __attribute__ ((__packed__, __gcc_struct__));
# else
} __attribute__ ((__packed__));
# endif
#else
    #warning "No way to pack struct on this compiler? This will break!"
#endif /* _MSC_VER */

#define USB_DT_DFU_SIZE            9

#define USB_TYPE_DFU        (LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE)

static constexpr uint32_t OTP_SECTION_SIZE = 64;        // 64 bytes. DO NOT CHANGE.
static constexpr uint32_t OTP_NUM_SECTIONS = 8;         // 8 attempts. DO NOT CHANGE.
static constexpr uint64_t OTP_KEY = 0xBAADBEEFB0BABABE;    // DO NOT CHANGE

static constexpr uint16_t USB_DESCRIPTOR_DFU = 0x21;
static constexpr int UID_MAX_SIZE = 20;
static constexpr int MAX_DESC_STR_LEN = 253;

static constexpr uint32_t STM32_PAGE_SIZE = 0x800;
static constexpr uint32_t STM32_PAGE_ERROR_MASK = 0x7FF;

static constexpr uint16_t STM32_DESCRIPTOR_VENDOR_ID = 0x0483;
static constexpr uint16_t STM32_DESCRIPTOR_PRODUCT_ID = 0xdf11;

const md5hash_t DFU_FINGERPRINT_STM32L4    = { { 0xFA, 0x45, 0x85, 0x0B, 0xE6, 0x92, 0x56, 0x3A, 0xD6, 0x5C, 0x40, 0x05, 0xDE, 0xBC, 0xB3, 0xF9 } };
const md5hash_t DFU_FINGERPRINT_STM32U5_1M = { { 0x82, 0x03, 0x64, 0x70, 0x21, 0x65, 0x55, 0x2A, 0xA2, 0x8B, 0xE7, 0x9D, 0x69, 0xBB, 0xA6, 0x2F } };
const md5hash_t DFU_FINGERPRINT_STM32U5_2M = { { 0xB5, 0xCE, 0xEA, 0xEB, 0xEE, 0xA7, 0x53, 0x3C, 0x4D, 0xCC, 0xBF, 0x30, 0x71, 0x9B, 0xE2, 0xAB } };

/** USB DFU class bStatus values (DFU spec §6.1.2), reported by DFU_GETSTATUS. Values are fixed by the spec -- do not change. */
typedef enum
{
    DFU_STATUS_OK = 0,             //!< no error condition is present
    DFU_STATUS_ERR_TARGET,         //!< file is not targeted for use by this device
    DFU_STATUS_ERR_FILE,           //!< file failed a device-specific verification test
    DFU_STATUS_ERR_WRITE,          //!< device is unable to write memory
    DFU_STATUS_ERR_ERASED,         //!< memory erase function failed
    DFU_STATUS_ERR_CHECK_ERASED,   //!< memory erase check failed
    DFU_STATUS_ERR_PROG,           //!< program memory function failed
    DFU_STATUS_ERR_VERIFY,         //!< programmed memory failed verification
    DFU_STATUS_ERR_ADDRESS,        //!< cannot program memory due to received address that is out of range
    DFU_STATUS_ERR_NOTDONE,        //!< received DFU_DNLOAD with wLength = 0, but device does not think it has all of the data yet
    DFU_STATUS_ERR_FIRMWARE,       //!< device's firmware is corrupt and cannot return to run-time (non-DFU) operations
    DFU_STATUS_ERR_VENDOR,         //!< iString indicates a vendor-specific error
    DFU_STATUS_ERR_USBR,           //!< device detected unexpected USB reset signaling
    DFU_STATUS_ERR_POR,            //!< device detected unexpected power on reset
    DFU_STATUS_ERR_UNKNOWN,        //!< something went wrong, but the device does not know what it was
    DFU_STATUS_ERR_STALLEDPKT,     //!< device stalled an unexpected request

    DFU_STATUS_NUM,                //!< number of defined status values; must be last
} dfu_status;

/** USB DFU class state machine states (DFU spec §6.1.2), reported by DFU_GETSTATE. Values are fixed by the spec -- do not change. */
typedef enum
{
    DFU_STATE_APP_IDLE = 0,         //!< device is running its normal (non-DFU) application
    DFU_STATE_APP_DETACH,           //!< device has received DFU_DETACH and is waiting for a USB reset
    DFU_STATE_IDLE,                 //!< device is operating in the DFU mode and is waiting for requests
    DFU_STATE_DNLOAD_SYNC,          //!< device has received a block and is waiting for the host to solicit its status
    DFU_STATE_DNBUSY,               //!< device is programming a control-write block into its nonvolatile memory
    DFU_STATE_DNLOAD_IDLE,          //!< device is processing a download operation and is expecting more data
    DFU_STATE_MANIFEST_SYNC,        //!< device has received the final block and is waiting for the host to solicit its status
    DFU_STATE_MANIFEST,             //!< device is in the manifestation phase, programming the received image
    DFU_STATE_MANIFEST_WAIT_RESET,  //!< device has programmed its memory and is waiting for a USB reset or power-on reset
    DFU_STATE_UPLOAD_IDLE,          //!< device is processing an upload operation and is expecting more requests
    DFU_STATE_ERROR,                //!< an error has occurred; device is waiting for DFU_CLRSTATUS

    DFU_STATE_NUM,                  //!< number of defined states; must be last
} dfu_state;

/**

    Reference:: UM0424 STM32 USB-FS-Device Development Kit, Pages 71 & 72
                https://www.st.com/content/ccc/resource/technical/document/user_manual/01/c6/32/df/79/ad/48/32/CD00158241.pdf/files/CD00158241.pdf/jcr:content/translations/en.CD00158241.pdf

    Alternate settings have to be used to access additional memory segments and other
    memories (Flash memory, RAM, EEPROM) which may or may not be physically
    implemented in the CPU memory mapping, such as external serial SPI Flash memory or
    external NOR/NAND Flash memory.

    The name of the alternate setting string descriptor respects the description of [4] chapter 10.
        @Target Memory Name/Start Address/Sector(1)_Count*Sector(1)_Size Sector(1)_Type,
                                          Sector(2)_Count*Sector(2)_Size Sector(2)_Type,
                                          ...
                                          Sector(n)_Count*Sector(n)_Size Sector(n)_Type

    // STM32L4 / IMX-5
    path="3-11.3", alt=3, name="@Device Feature/0xFFFF0000/01*004 e"
    path="3-11.3", alt=2, name="@OTP Memory /0x1FFF7000/01*0001Ke"
    path="3-11.3", alt=1, name="@Option Bytes  /0x1FFF7800/01*040 e"
    path="3-11.3", alt=0, name="@Internal Flash  /0x08000000/0256*0002Kg"

    // STM32U5 / GPX-1
    path="3-11.4", alt=2, name="@OTP Memory   /0x0BFA0000/01*512 e"
    path="3-11.4", alt=1, name="@Option Bytes   /0x40022040/01*64 e"
    path="3-11.4", alt=0, name="@Internal Flash   /0x08000000/128*08Kg"
**/

/** One accessible memory segment/region reported by a DfuSe alternate-setting descriptor. */
typedef struct {
    uint64_t address;                       //!< the base address of the accessible memory, reported by the DFU descriptor
    uint16_t pages;                          //!< the number of pages for this descriptor, reported by the DFU descriptor
    uint32_t pageSize;                       //!< the size of each page related to this descriptor, reported by the DFU descriptor
    uint8_t pageType;                        //!< the page type (readable/writable/erasable, etc)
} dfu_memory_t;

/** STM32 DfuSe alternate-setting indices, identifying which memory region an interface's altsetting addresses. */
typedef enum : uint16_t {
    STM32_DFU_INTERFACE_FLASH = 0,    //!< "@Internal Flash" -- main program flash
    STM32_DFU_INTERFACE_OPTIONS = 1,  //!< "@Option Bytes" -- STM32 option byte region (RDP, boot config, etc.)
    STM32_DFU_INTERFACE_OTP = 2,      //!< "@OTP Memory" -- one-time-programmable region (Inertial Sense serial/HW id)
    STM32_DFU_INTERFACE_FEATURES = 3  //!< "@Device Feature" -- vendor-specific DfuSe command interface
} dfu_interface_alternatives;

/**
 * Result/error code for DFUDevice operations. Internal to this header (not part of the wire
 * protocol), so values can change as needed.
 */
typedef enum
{
    DFU_ERROR_NONE = 0,                    //!< operation succeeded
    DFU_ERROR_DEVICE_NOTFOUND = -1,        //!< no matching DFU device was found
    DFU_ERROR_DEVICE_BUSY = -2,            //!< device is already open/in-use
    DFU_ERROR_TIMEOUT = -3,                //!< a USB transfer or state wait timed out
    DFU_ERROR_LIBUSB = -4,                 //!< a libusb call failed; see getLastLibusbError()/getLastLibusbErrorName()
    DFU_ERROR_STATUS = -5,                 //!< the device reported a DFU error status (dfu_status != DFU_STATUS_OK)
    DFU_ERROR_INVALID_ARG = -6,            //!< an invalid argument was passed (e.g. bad address/length)
    DFU_ERROR_FILE_NOTFOUND = -7,          //!< the firmware image file could not be opened
    DFU_ERROR_FILE_INVALID = -8,           //!< the firmware image file failed validation
    DFU_ERROR_RDP_LOCKED = -9,             //!< SN-8043: chip at RDP Level 1 (RDP > 0xAA); flash writes are silently dropped. Recoverable via erase/RDP-regress.
    DFU_ERROR_RDP_PERMANENT_LOCKED = -10,  //!< SN-8043: chip at RDP Level 2 (RDP == 0xCC); permanently locked, cannot be recovered.
    DFU_ERROR_WRITE_VERIFY_FAILED = -11,   //!< SN-8043: post-write readback did not match the source image; the write did not land.
} dfu_error;

/** Processor family detected from a DFU device's USB fingerprint (see DFUDevice::getProcessorType). */
typedef enum {
    IS_PROCESSOR_UNKNOWN = -1,      //!< not yet identified / unrecognized fingerprint
    IS_PROCESSOR_SAMx70 = 0,        //!< uINS-3/4, EVB-2
    IS_PROCESSOR_STM32L4,           //!< IMX-5
    IS_PROCESSOR_STM32U5,           //!< GPX-1, IMX-6

    IS_PROCESSOR_NUM,               //!< number of known processor types; must be last
} eProcessorType;

/**
 * Wraps a single USB DFU device (identified by a libusb_device) and drives an STM32 DfuSe-style
 * firmware update over it: open/claim -> erase -> write -> verify -> finalize, plus descriptor-based
 * device identification (processor type, flash size, fingerprint) used to match a physical module
 * against a target and firmware image.
 */
class DFUDevice {
public:

    /**
     * Wraps the given (already-referenced) libusb device; does not open it. Reads USB descriptors
     * to populate identification fields (see fetchDeviceInfo()).
     * @param device the libusb device to wrap; this object takes ownership (unref'd in the destructor)
     * @param cbProgress optional callback invoked to report upload/erase/verify progress
     * @param cbStatus optional callback invoked to report status/log messages
     */
    DFUDevice(libusb_device *device, fwUpdate::pfnProgressCb cbProgress = nullptr, fwUpdate::pfnStatusCb  cbStatus = nullptr) {
        usbDevice = device;
        progressCb = cbProgress;
        statusCb = cbStatus;
        usbHandle = nullptr;
        fetchDeviceInfo();
    }

    /** Closes the USB handle (if open) and releases the wrapped libusb device. */
    ~DFUDevice() {
        if (usbHandle) {
            libusb_release_interface(usbHandle, 0);
            libusb_close(usbHandle);
            usbHandle = nullptr;
        }
        if (usbDevice) {
            libusb_unref_device(usbDevice);
            usbDevice = nullptr;
        }
    }

    /** @return true if the device is currently open and still present on the bus. */
    bool isConnected() { return (usbHandle != nullptr) && (libusb_get_device(usbHandle) != nullptr); }

    /**
     * Connect and establish USB DFU status is IDLE.
     * @param resetDevice when true (default) issue a libusb_reset_device() after opening, forcing a clean
     *                    USB re-enumeration before the DFU session -- wanted before programming. Pass false
     *                    for read-only identification (descriptor/fingerprint/OTP reads during discovery):
     *                    abort()+waitForState(IDLE) still establish a known DFU state, and skipping the
     *                    reset avoids a ~100-200ms per-device USB re-enumeration on every scan.
     * @return DFU_ERROR_NONE on success, or a dfu_error describing the failure (open/claim/state)
     */
    dfu_error open(bool resetDevice = true);

    /**
     * Erases, writes, and verifies the given Intel-HEX firmware file to this device, then finalizes.
     * @param filename path to an Intel-HEX (.hex) firmware image
     * @param baseAddress base flash address to write the image at (0 = the image's own addressing)
     * @return DFU_ERROR_NONE on success, or a dfu_error describing the failure
     */
    dfu_error updateFirmware(std::string filename, uint64_t baseAddress = 0);

    /**
     * Erases, writes, and verifies an Intel-HEX firmware image read from the given stream, then finalizes.
     * @param stream an input stream over Intel-HEX (.hex) firmware image text
     * @param baseAddress base flash address to write the image at (0 = the image's own addressing)
     * @return DFU_ERROR_NONE on success, or a dfu_error describing the failure
     */
    dfu_error updateFirmware(std::istream& stream, uint64_t baseAddress = 0);
    // dfu_error updateFirmware(std::queue<uint8_t>, uint32_t imgSize, uint64_t baseAddress = 0);

    /**
     * Finalizes a completed firmware write (e.g. writing option bytes / leaving DFU mode so the
     * device reboots into the new application). A resulting USB disconnect is generally the
     * expected, successful outcome -- see isExpectedOptionByteResetError().
     * @return DFU_ERROR_NONE on success (including the expected reset-disconnect), or a dfu_error otherwise
     */
    dfu_error finalizeFirmware();

    /** Closes the USB handle, if open. @return DFU_ERROR_NONE on success, or a dfu_error otherwise */
    dfu_error close();

    /** Issues a USB port reset on the device. @return 0 on success, or a libusb error code */
    int reset();

    /**
     * Issues a single-byte DfuSe class-specific command to the device, sent as a DFU_DNLOAD with
     * block number 0 (the mechanism ST's DfuSe bootloader uses for its special commands, e.g. the
     * same transport the internal erase path uses). The caller supplies the command code; this method
     * performs no interpretation of it. Some commands initiate a long internal operation and/or an
     * immediate device reset, so a resulting USB disconnect (see isExpectedOptionByteResetError()) is
     * treated as the expected, successful outcome rather than a failure.
     * @param cmd the DfuSe command code to send
     * @return DFU_ERROR_NONE on success (including the expected reset-disconnect); a libusb-tagged error otherwise
     */
    dfu_error sendDfuCommand(int cmd);

    /** @return a human-readable description of the device (manufacturer/product strings), or "" if unavailable */
    const char *getDescription();

    /** @return the MD5 fingerprint computed from this device's descriptors, used to match it against a known firmware image */
    md5hash_t getFingerprint() { return fingerprint.state; }

    /** @return the fwUpdate target type this device corresponds to (derived from its fingerprint/processor type), or TARGET_UNKNOWN */
    fwUpdate::target_t getTargetType();

    /** @return the Inertial Sense hardware id (type/version) read from OTP data, or 0xFFFF if unavailable */
    uint16_t getHardwareId() { return hardwareId; }

    /** @return the Inertial Sense serial number read from OTP data, or UINT32_MAX if unavailable */
    uint32_t getSerialNo() { return sn; }

    /**
     * The USB DFU serial-number string from the device's iSerialNumber descriptor (the STM32
     * factory unique-ID-derived serial). Unlike getSerialNo() (the Inertial Sense OTP serial), this
     * is read from a plain USB string descriptor, so it is available even when the module's
     * flash/OTP is read-protected, and it is stable across a flash erase/reprogram cycle. That makes
     * it the reliable key for re-matching a specific physical module after it resets and re-enumerates.
     * @return the DFU serial string (empty if the descriptor was unavailable)
     */
    const char *getUsbSerial() const { return dfuSerial.c_str(); }

    /**
     * SN-8193: the raw libusb error code captured the last time this device produced a
     * DFU_ERROR_LIBUSB. The dfu_error return value only carries the DFU_ERROR_LIBUSB tag, so the
     * specific libusb code is preserved here for diagnostics.
     * @return the most recent libusb error code for this device, or LIBUSB_SUCCESS (0) if none has occurred
     */
    int getLastLibusbError() const { return lastLibusbError; }

    /**
     * SN-8193: human-readable name of the most recent libusb error for this device (see getLastLibusbError()).
     * @return the libusb error name from libusb_error_name() (e.g. "LIBUSB_ERROR_NO_DEVICE")
     */
    const char* getLastLibusbErrorName() const { return libusb_error_name(lastLibusbError); }

    /** @return the processor family detected from this device's USB fingerprint */
    eProcessorType getProcessorType() const { return processorType; }

    /** @return the total flash size (bytes) reported by this device's memory segment descriptors */
    uint32_t getTotalFlashSize() const;

    /**
     * @param procType the processor family (see getProcessorType())
     * @param totalFlashSize the total flash size (bytes, see getTotalFlashSize())
     * @return a human-readable device type name (e.g. "IMX-5"), derived from procType and totalFlashSize
     */
    static const char* getDeviceTypeName(eProcessorType procType, uint32_t totalFlashSize);

    /** @param cbProgress callback invoked to report upload/erase/verify progress */
    void setProgressCb(fwUpdate::pfnProgressCb cbProgress){ progressCb = cbProgress;}

    /** @param cbStatus callback invoked to report status/log messages */
    void setStatusCb(fwUpdate::pfnStatusCb cbStatus) { statusCb = cbStatus;}

    /** @param errNo a dfu_error value @return the enumerator name of errNo (e.g. "DFU_ERROR_TIMEOUT") */
    static const char* getErrorName(int errNo);

    /**
     * Maps a raw STM32 FLASH_OPTR RDP (read-protection) byte to a dfu_error verdict.
     * Pure decision logic, separated so it can be unit-tested without USB hardware (SN-8043).
     *   RDP == 0xAA -> DFU_ERROR_NONE (Level 0, unprotected; flash programming allowed)
     *   RDP == 0xCC -> DFU_ERROR_RDP_PERMANENT_LOCKED (Level 2, permanent)
     *   otherwise   -> DFU_ERROR_RDP_LOCKED (Level 1; flash writes silently dropped by the ROM bootloader)
     * @param rdpByte the raw STM32 FLASH_OPTR RDP byte read from the device
     * @return a dfu_error verdict for the RDP level (see mapping above)
     */
    static dfu_error rdpVerdict(uint8_t rdpByte);

    /**
     * SN-8193: classifies a libusb error returned by the Option-Bytes download() in finalizeFirmware().
     * Writing the STM32 FLASH option bytes triggers a mandatory immediate device reset, so the USB
     * device disconnects mid-transfer and libusb reports a disconnect-class error. That error is the
     * EXPECTED successful outcome of finalize, not a failure. Pure decision logic, separated so it can
     * be unit-tested without USB hardware (same pattern as rdpVerdict()).
     * @param libusbError a libusb return/error code (e.g. LIBUSB_ERROR_NO_DEVICE)
     * @return true for the disconnect-class codes (LIBUSB_ERROR_NO_DEVICE / _IO / _PIPE) that are the
     *         expected result of the option-byte reset; false for success and all other errors
     *         (e.g. TIMEOUT, ACCESS), which remain real finalize failures
     */
    static bool isExpectedOptionByteResetError(int libusbError);

    /**
     * SN-8193: human-readable name for a libusb error code (thin wrapper over libusb's own
     * libusb_error_name()). Pure, so it is unit-testable without USB hardware.
     * @param libusbCode a libusb return/error code
     * @return the libusb error name (e.g. "LIBUSB_ERROR_NO_DEVICE"); never null
     */
    static const char* libusbErrorName(int libusbCode);

    /**
     * Populates devInfo from this device's identification data (hardware id, serial number,
     * processor type/flash size derived device name, etc).
     * @param devInfo the dev_info_t struct to fill
     * @return 0 on success, or a negative error code if identification data was unavailable
     */
    int fillDeviceInfo(dev_info_t &devInfo);

protected:
    /**
     * SN-8193: records `libusbCode` as this device's last libusb error and returns the
     * DFU_ERROR_LIBUSB tag. Replaces the old `(dfu_error)(DFU_ERROR_LIBUSB | (code << 16))` packing,
     * which silently discarded the libusb code: DFU_ERROR_LIBUSB (-4) is sign-extended to all-ones in
     * the high bits, so OR-ing the shifted code never changed any bit and the result was always -4.
     * The code is now retrievable via getLastLibusbError() / getLastLibusbErrorName().
     * @param libusbCode the libusb error code to record for this device
     * @return DFU_ERROR_LIBUSB
     */
    dfu_error libusbError(int libusbCode);

    /**
     * Opens the wrapped device (if not already open), reads its USB descriptors (vendor/product
     * strings, DFU functional descriptor, alternate-setting memory segments), computes its
     * fingerprint, and reads OTP data (serial number, hardware id) if accessible.
     * @return DFU_ERROR_NONE on success, or a dfu_error describing the failure
     */
    dfu_error fetchDeviceInfo();

    /**
     * Reads and decodes a USB string descriptor to plain ASCII.
     * @param desc_index the string descriptor index (from another descriptor's iXxx field)
     * @param data buffer to receive the decoded ASCII string
     * @param length size of data, in bytes
     * @return the number of bytes written to data, or a negative libusb error code
     */
    int get_string_descriptor_ascii(uint8_t desc_index, char *data, int length);

    /**
     * Selects the memory segment containing [address, address+data_len) and validates the write is
     * in-bounds, before a download (erase/write) operation proceeds.
     * @param address the target flash address
     * @param data_len the number of bytes to be written starting at address
     * @return DFU_ERROR_NONE if the range is valid and selected, or a dfu_error otherwise
     */
    dfu_error prepAndValidateBeforeDownload(uint32_t address, uint32_t data_len);

    /**
     * Erases the page(s) of mem covering [address, address+data_len).
     * @param mem the memory segment descriptor covering address
     * @param address the flash address to begin erasing from
     * @param data_len the number of bytes that will subsequently be written (determines page count)
     * @return DFU_ERROR_NONE on success, or a dfu_error describing the failure
     */
    dfu_error eraseFlash(const dfu_memory_t& mem, uint32_t& address, uint32_t data_len);

    /**
     * Writes data_len bytes of data to mem starting at address.
     * @param mem the memory segment descriptor covering address
     * @param address the flash address to begin writing at
     * @param data_len the number of bytes to write
     * @param data the bytes to write
     * @return DFU_ERROR_NONE on success, or a dfu_error describing the failure
     */
    dfu_error writeFlash(const dfu_memory_t& mem, uint32_t& address, uint32_t data_len, uint8_t *data);

    /**
     * SN-8043: Pre-flight read-protection (RDP) check. STM32U5 only (IMX-6 / GPX-1); a no-op (returns
     * DFU_ERROR_NONE) for other processors. Reads FLASH_OPTR via the DFU OPTIONS segment (which works
     * regardless of RDP level) and returns rdpVerdict() of the RDP byte. At RDP Level 1 the ROM
     * bootloader ACKs every DNLOAD but silently drops the underlying flash programming, so we must
     * refuse to proceed rather than report a false success.
     * @return DFU_ERROR_NONE if unprotected (or not applicable to this processor), or a dfu_error otherwise
     */
    dfu_error checkReadProtection();

    /**
     * SN-8043: Post-write readback verification. Uploads the first verifyLen bytes of the just-written
     * region and compares them against the source image. Returns DFU_ERROR_WRITE_VERIFY_FAILED on
     * mismatch (the classic RDP-silent-drop signature is an all-0xFF / all-0x00 readback). A readback
     * transport failure is logged but NOT treated as fatal, to avoid false negatives on good devices.
     * @param address the flash address the just-completed write started at
     * @param expected the source image bytes that should now be present at address
     * @param verifyLen the number of bytes to read back and compare
     * @return DFU_ERROR_NONE if the readback matches (or could not be performed), or DFU_ERROR_WRITE_VERIFY_FAILED on mismatch
     */
    dfu_error verifyFlashWrite(uint32_t address, const uint8_t *expected, uint32_t verifyLen);

private:
    libusb_device *usbDevice = nullptr;
    libusb_device_handle *usbHandle = nullptr;  // if this is not null, then this should be a valid, open handle.
    int lastLibusbError = LIBUSB_SUCCESS;       //!< SN-8193: raw libusb code from this device's most recent DFU_ERROR_LIBUSB

    uint16_t vid = 0;                           // the vendor id for this device (for filtering/selection)
    uint16_t pid = 0;                           // the product id for this device (for filtering/selection)
    usb_dfu_func_descriptor funcDescriptor {};  // a copy of the DFU functional descriptor
    std::vector<std::string> dfuDescriptors {}; // an array containing the contents of each of the available Alt Identifier strings (used to generate the fingerprint)

    std::string dfuManufacturer;                // the extracted manufacturer id/name (as a string) from the iManufacturer descriptor
    std::string dfuProduct;                     // the extracted product id/name (as a string) from the iProduct descriptor
    std::string dfuSerial;                      // the extracted DFU device serial number, from descriptors (see iSerialNumber above)

    uint32_t sn = -1;                           // Inertial Sense serial number (from OTP data)
    uint16_t hardwareId = -1;                   // Inertial Sense Hardware ID (from OTP data)
    eProcessorType processorType = IS_PROCESSOR_UNKNOWN;          // detected processor type/family
    dfu_memory_t segments[4] {};                // memory segment detail, corresponding with the alternate descriptor ID

    md5Context_t fingerprint {};                // an MD5 hash of various data/parameters used to uniquely identify this device

    uint16_t dlBlockNum = 0;                    // download block count; should be reset for each separate transfer
    uint16_t ulBlockNum = 0;                    // upload block count; should be reset for each separate transfer

    fwUpdate::pfnProgressCb progressCb = nullptr;
    fwUpdate::pfnStatusCb statusCb = nullptr;

    /**
     * @brief OTP section
     */
    typedef struct {
        uint32_t serialNumber;   //!< Inertial Sense serial number
        uint16_t lotNumber;      //!< Inertial Sense lot number
        uint16_t hardwareId;     //!< Inertial Sense Hardware Id (type/version)
        char date[16];           //!< Inertial Sense manufacturing date (YYYYMMDDHHMMSS)
    } otp_info_t;

    int detach(uint8_t timeout);

    int download(uint16_t& wValue, uint8_t *buf, uint16_t len, uint32_t timeout_ms = 5000);

    int upload(uint16_t& wValue, uint8_t *buf, uint16_t len, uint32_t timeout_ms = 5000);

    int getStatus(dfu_status *status, uint32_t *delay, dfu_state *state, uint8_t *i_string);

    int clearStatus();

    int getState(dfu_state *buf);

    int abort();

    int waitForState(dfu_state required_state, dfu_state* actual_state = nullptr, uint32_t timeout_ms = 5000);

    int setAddress(uint16_t& wValue, uint32_t address, uint32_t timeout_ms = 5000);

    int readMemory(uint32_t memloc, uint8_t *rxBuf, size_t rxLen, uint32_t timeout_ms = 5000);

    static DFUDevice::otp_info_t *decodeOTPData(uint8_t *raw, int len);

    static int findDescriptor(const uint8_t *desc_list, int list_len, uint8_t desc_type, void *res_buf, int res_size);

    static int decodeMemoryPageDescriptor(const std::string& altSetting, dfu_memory_t& segment);

    static const char *dfuDeviceErrors[];
};

/**
 * Adapts a DFUDevice to the fwUpdate::FirmwareUpdateDevice interface, so a USB-DFU-mode target
 * (e.g. an STM32-based IMX/GPX module) can be driven through the same firmware update state
 * machine used for in-application (wire-protocol) updates. Also provides static libusb
 * initialization and DFU device enumeration/discovery helpers used before a device is claimed.
 */
class ISDFUFirmwareUpdater : public fwUpdate::FirmwareUpdateDevice {
public:
    /**
     * Constructor to establish a connection to the specified device, and optionally validate against serialNo.
     * @param target the fwUpdate target device this updater instance is responsible for
     * @param device the libusb device which identifies the connected device to update. This is NOT a libusb_device_handle! If null, this function will use the first detected DFU device which matches serialNo
     * @param serialNo the device-specific unique Id (or serial number) that is used to uniquely identify a particular device (used in validation); UINT32_MAX to skip validation
     */
    ISDFUFirmwareUpdater(fwUpdate::target_t target, libusb_device *device = nullptr, uint32_t serialNo = UINT32_MAX);

    /** Trivial destructor; owned DFUDevice/buffers are released elsewhere (see cleanup in the .cpp). */
    ~ISDFUFirmwareUpdater() { };

    /** Initialize the libusb context. Call once at application startup before any DFU operations. */
    static void initLibUSB();
    /** Tear down the libusb context. Call once at application shutdown. */
    static void exitLibUSB();

    /**
     * Lightweight, value-type identity for a discovered DFU device -- safe to hand across threads
     * (no libusb handles, no ownership). Built from a DFUDevice during enumeration.
     */
    struct DfuDeviceInfo {
        std::string    description;                           //!< human-readable (DFUDevice::getDescription)
        std::string    typeName;                              //!< "IMX-6"/"GPX-1"/"IMX-5"/"Unknown"
        std::string    usbSerial;                             //!< USB DFU serial descriptor
        uint32_t       serialNo = 0xFFFFFFFF;                 //!< IS OTP serial (UINT32_MAX if unreadable)
        uint16_t       hardwareId = 0xFFFF;                   //!< encoded hardware id (0xFFFF if unknown)
        eProcessorType processorType = IS_PROCESSOR_UNKNOWN;  //!< processor family from the USB fingerprint
        uint32_t       totalFlashSize = 0;                    //!< total flash size in bytes (0 if unknown)
    };

    /** Invoked once per successfully-enumerated device as it is identified (on the calling thread). */
    typedef std::function<void(const DfuDeviceInfo &info)> pfnDfuDeviceFoundCb;

    /**
     * Enumerate connected DFU devices.
     * @param devices       receives one DFUDevice* per enumerated device (caller takes ownership)
     * @param vid           USB vendor id filter (0 = any)
     * @param pid           USB product id filter (0 = any)
     * @param onDeviceFound if provided, called for each device as it is identified, enabling a
     *                      progressive/streaming UI when this runs on a worker thread
     * @return the number of devices enumerated into @p devices
     */
    static size_t getAvailableDevices(std::vector<DFUDevice *> &devices, uint16_t vid = 0x0000, uint16_t pid = 0x0000,
                                      pfnDfuDeviceFoundCb onDeviceFound = nullptr);

    /**
     * @param vid USB vendor id filter (0 = any)
     * @param pid USB product id filter (0 = any)
     * @return the number of currently-attached DFU devices matching vid/pid
     */
    static int getNumDevices(uint16_t vid = 0x0000, uint16_t pid = 0x0000);

    /**
     * Removes (and deletes) every device in devices whose fingerprint does not match fingerprint.
     * @param devices the device list to filter in place
     * @param fingerprint the fingerprint to match against (see DFUDevice::getFingerprint())
     * @return the number of devices remaining in devices after filtering
     */
    static size_t filterDevicesByFingerprint(std::vector<DFUDevice *> &devices, md5hash_t fingerprint);

    /**
     * Removes (and deletes) every device in devices whose fwUpdate target type does not match target.
     * @param devices the device list to filter in place
     * @param target the fwUpdate target type to match against (see DFUDevice::getTargetType())
     * @return the number of devices remaining in devices after filtering
     */
    static size_t filterDevicesByTargetType(std::vector<DFUDevice *> &devices, fwUpdate::target_t target);

    /**
     * @param usbDevice the libusb device to check
     * @param vid USB vendor id filter (0 = any)
     * @param pid USB product id filter (0 = any)
     * @return true if usbDevice matches vid/pid and exposes a USB DFU functional descriptor
     */
    static bool isDFUDevice(libusb_device *usbDevice, uint16_t vid, uint16_t pid);

    // ---- DFU discovery state machine (step-driven; no internal thread) ----------------------------

    /**
     * State of the step-driven DFU discovery scan. Reports when the set of attached DFU devices has
     * SETTLED, so a UI can wait for discovery to complete instead of reacting to the first device that
     * appears (which yields a partial list with not-yet-enumerated "unknown" entries). Settle detection
     * is COUNT-based (not identity-based): an unidentifiable DFU device still contributes to the count
     * being waited on, so it can never deadlock completion. The SDK keeps NO thread -- the caller drives
     * discoveryStep() from its own loop/worker at whatever cadence it likes (single-thread-friendly).
     */
    enum eDfuDiscoveryState {
        DFU_DISCOVERY_IDLE = 0,    //!< no DFU devices present
        DFU_DISCOVERY_STARTED,     //!< one or more devices appeared; count not yet stable
        DFU_DISCOVERY_COMPLETED,   //!< count held steady for >= the settle timeout
    };

    /**
     * Caller-owned discovery state. Set vid/pid/settleTimeoutMs once, then call discoveryStep() each
     * tick and read state/count after. No clock or thread is used internally -- elapsedMs (passed to
     * discoveryStep) supplies timing, so the cadence is entirely the caller's.
     */
    struct DfuDiscoveryContext {
        // configuration (set by caller before first step):
        uint16_t           vid = 0x0000;                 //!< count filter (0 = any)
        uint16_t           pid = 0x0000;                 //!< count filter (0 = any)
        uint32_t           settleTimeoutMs = 1000;       //!< count must hold steady this long for COMPLETED
        // observable result (read by caller):
        eDfuDiscoveryState state = DFU_DISCOVERY_IDLE;   //!< current discovery state
        int                count = 0;                    //!< device count from the most recent step
        // internal:
        int                lastCount = 0;                //!< the count currently being timed for stability
        uint32_t           stableElapsedMs = 0;          //!< accumulated time the count has held steady
    };

    /**
     * Advances the discovery state machine by one poll: counts DFU devices (getNumDevices(vid,pid)) and
     * updates ctx.state/ctx.count. initLibUSB() must have been called first.
     * @param ctx       caller-owned discovery context (configuration in, state/count out)
     * @param elapsedMs time since the previous call (e.g. the caller's poll interval)
     * @return true iff the state changed this step (so the caller can emit/act on the transition)
     */
    static bool discoveryStep(DfuDiscoveryContext &ctx, uint32_t elapsedMs);


    /**
     * Drives the DFU update state machine forward by one step; called internally by
     * processMessage() when a message is received, and should also be called periodically
     * (independent of message reception) so timeouts and erase/write/verify progress continue
     * to advance.
     * @param msg_type the type of message that was last processed, or MSG_UNKNOWN
     * @param processed true if msg_type was already handled by the caller (additional/optional
     *        processing may still occur here), false if it was not yet handled
     * @return true if some action was taken as a result of this step, otherwise false
     */
    bool fwUpdate_step(fwUpdate::msg_types_e msg_type = fwUpdate::MSG_UNKNOWN, bool processed = false) override;

    /**
     * Called by the port receiver when a DID_FIRMWARE_UPDATE message has been received, to parse
     * the internal fwUpdate payload out of the raw port buffer.
     * @param rxPort the port the message was received on
     * @param buffer the raw received message buffer
     * @param buf_len the number of bytes in buffer
     * @return true if the message was successfully parsed and processed, otherwise false
     */
    bool fwUpdate_processMessage(int rxPort, const uint8_t* buffer, int buf_len);

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

    static std::mutex dfuMutex;
    /** Set by initLibUSB() from libusb_init()'s return value; getNumDevices() checks this before
     *  calling any other libusb_* function on the (possibly never-initialized) default context. */
    static bool libUsbAvailable;
    DFUDevice *curDevice;

    struct membuf: std::streambuf {
        membuf(char const* base, size_t size) {
            char* p(const_cast<char*>(base));
            this->setg(p, p, p + size);
        }
    };
    struct imemstream: virtual membuf, std::istream {
        imemstream(char const* base, size_t size)
                : membuf(base, size)
                , std::istream(static_cast<std::streambuf*>(this)) {
        }
    };

    ByteBuffer* imgBuffer = nullptr;
    ByteBufferStream* imgStream = nullptr;

    std::deque<uint8_t> toDevice;         //!< a data stream that is input from the host (host tx) and output to the device (device rx)
    std::deque<uint8_t> toHost;           //!< a data stream that is input from the device (device tx) and output to the host (host rx)

};

#endif //IS_DFU_FIRMWAREUPDATER_H
