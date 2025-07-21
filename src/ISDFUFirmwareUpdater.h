/**
 * @file ISDFUFirmwareUpdater.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 11/28/23.
 * @copyright Copyright (c) 2023 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_DFU_FIRMWAREUPDATER_H
#define IS_DFU_FIRMWAREUPDATER_H

#include "protocol/FirmwareUpdate.h"

#include "ihex.h"
#include "ISUtilities.h"
// #include "protocol/usb_dfu.h"
#include "util/md5.h"

#include "libusb.h"

#include <mutex>

namespace dfu {

#ifdef _MSC_VER
# pragma pack(push)
# pragma pack(1)
#endif /* _MSC_VER */
struct usb_dfu_func_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bmAttributes;
#define USB_DFU_CAN_DOWNLOAD    (1 << 0)
#define USB_DFU_CAN_UPLOAD    (1 << 1)
#define USB_DFU_MANIFEST_TOL    (1 << 2)
#define USB_DFU_WILL_DETACH    (1 << 3)
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

const md5hash_t DFU_FINGERPRINT_STM32L4 = { 0xFA, 0x45, 0x85, 0x0B, 0xE6, 0x92, 0x56, 0x3A, 0xD6, 0x5C, 0x40, 0x05, 0xDE, 0xBC, 0xB3, 0xF9 };
const md5hash_t DFU_FINGERPRINT_STM32U5 = { 0x82, 0x03, 0x64, 0x70, 0x21, 0x65, 0x55, 0x2A, 0xA2, 0x8B, 0xE7, 0x9D, 0x69, 0xBB, 0xA6, 0x2F };

typedef enum    // From DFU manual, do not change
{
    DFU_STATUS_OK = 0,
    DFU_STATUS_ERR_TARGET,
    DFU_STATUS_ERR_FILE,
    DFU_STATUS_ERR_WRITE,
    DFU_STATUS_ERR_ERASED,
    DFU_STATUS_ERR_CHECK_ERASED,
    DFU_STATUS_ERR_PROG,
    DFU_STATUS_ERR_VERIFY,
    DFU_STATUS_ERR_ADDRESS,
    DFU_STATUS_ERR_NOTDONE,
    DFU_STATUS_ERR_FIRMWARE,
    DFU_STATUS_ERR_VENDOR,
    DFU_STATUS_ERR_USBR,
    DFU_STATUS_ERR_POR,
    DFU_STATUS_ERR_UNKNOWN,
    DFU_STATUS_ERR_STALLEDPKT,

    DFU_STATUS_NUM,
} dfu_status;

typedef enum    // From DFU manual, do not change
{
    DFU_STATE_APP_IDLE = 0,
    DFU_STATE_APP_DETACH,
    DFU_STATE_IDLE,
    DFU_STATE_DNLOAD_SYNC,
    DFU_STATE_DNBUSY,
    DFU_STATE_DNLOAD_IDLE,
    DFU_STATE_MANIFEST_SYNC,
    DFU_STATE_MANIFEST,
    DFU_STATE_MANIFEST_WAIT_RESET,
    DFU_STATE_UPLOAD_IDLE,
    DFU_STATE_ERROR,

    DFU_STATE_NUM,
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

typedef struct {
    uint64_t address;                       // the base address of the accessible memory, reported by the DFU descriptor
    uint16_t pages;                         // the number of pages for this descriptor, reported by the DFU descriptor
    uint32_t pageSize;                      // the size of each page related to this descriptor, reported by the DFU descriptor
    uint8_t pageType;                       // the page type (readable/writable/erasable, etc)
} dfu_memory_t;

typedef enum : uint16_t {
    STM32_DFU_INTERFACE_FLASH = 0, // @Internal Flash
    STM32_DFU_INTERFACE_OPTIONS = 1, // @Option Bytes
    STM32_DFU_INTERFACE_OTP = 2, // @OTP Memory
    STM32_DFU_INTERFACE_FEATURES = 3  // @Device Feature
} dfu_interface_alternatives;

typedef enum    // Internal only, can change as needed
{
    DFU_ERROR_NONE = 0,
    DFU_ERROR_DEVICE_NOTFOUND = -1,
    DFU_ERROR_DEVICE_BUSY = -2,
    DFU_ERROR_TIMEOUT = -3,
    DFU_ERROR_LIBUSB = -4,
    DFU_ERROR_STATUS = -5,
    DFU_ERROR_INVALID_ARG = -6,
    DFU_ERROR_FILE_NOTFOUND = -7,
    DFU_ERROR_FILE_INVALID = -8,
} dfu_error;

typedef enum {
    IS_PROCESSOR_UNKNOWN = -1,
    IS_PROCESSOR_SAMx70 = 0,        // uINS-3/4, EVB-2
    IS_PROCESSOR_STM32L4,           // IMX-5
    IS_PROCESSOR_STM32U5,           // GPX-1, IMX-6

    IS_PROCESSOR_NUM,               // Must be last
} eProcessorType;

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_DEBUG = 4,
    IS_LOG_LEVEL_SILLY = 5
} eLogLevel;

typedef void (*pfnFwUpdateStatus)(void* obj, int logLevel, const char* msg, ...);
typedef void (*pfnFwUpdateProgress)(void* obj, const std::string stepName, int stepNo, int totalSteps, float percent);

class DFUDevice {
public:

    DFUDevice(libusb_device *device, pfnFwUpdateProgress cbProgress = nullptr, pfnFwUpdateStatus cbStatus = nullptr) {
        usbDevice = device;
        progressFn = cbProgress;
        statusFn = cbStatus;
        usbHandle = nullptr;
        fetchDeviceInfo();
    }

    bool isConnected() { return (usbHandle != nullptr) && (libusb_get_device(usbHandle) != nullptr); }

    /**
     * Connect and establish USB DFU status is IDLE
     * @return
     */
    dfu_error open();
    dfu_error updateFirmware(std::string filename, uint64_t baseAddress = 0);
    dfu_error updateFirmware(std::istream& stream, uint64_t baseAddress = 0);
    dfu_error finalizeFirmware();
    dfu_error close();

    const char *getDescription();

    md5hash_t getFingerprint() { return fingerprint.state; }

    fwUpdate::target_t getTargetType();

    void setProgressCb(pfnFwUpdateProgress cbProgress){progressFn = cbProgress;}
    void setStatusCb(pfnFwUpdateStatus cbStatus) {statusFn = cbStatus;}

protected:
    dfu_error fetchDeviceInfo();

    int get_string_descriptor_ascii(uint8_t desc_index, char *data, int length);

    dfu_error prepAndValidateBeforeDownload(uint32_t address, uint32_t data_len);

    dfu_error eraseFlash(const dfu_memory_t& mem, uint32_t& address, uint32_t data_len);

    dfu_error writeFlash(const dfu_memory_t& mem, uint32_t& address, uint32_t data_len, uint8_t *data);

private:
    libusb_device *usbDevice;
    libusb_device_handle *usbHandle;            // if this is not null, then this should be a valid, open handle.

    uint16_t vid;                               // the vendor id for this device (for filtering/selection)
    uint16_t pid;                               // the product id for this device (for filtering/selection)
    usb_dfu_func_descriptor funcDescriptor;     // a copy of the DFU functional descriptor
    std::vector<std::string> dfuDescriptors;    // an array containing the contents of each of the available Alt Identifier strings (used to generate the fingerprint)

    std::string dfuManufacturer;                // the extracted manufacturer id/name (as a string) from the iManufacturer descriptor
    std::string dfuProduct;                     // the extracted product id/name (as a string) from the iProduct descriptor
    std::string dfuSerial;                      // the extracted DFU device serial number, from descriptors (see iSerialNumber above)

    uint32_t sn;                                // Inertial Sense serial number (from OTP data)
    uint16_t hardwareId;                        // Inertial Sense Hardware ID (from OTP data)
    eProcessorType processorType;               // detected processor type/family
    dfu_memory_t segments[4];                   // memory segment detail, corresponding with the alternate descriptor ID

    md5Context_t fingerprint;                      // an MD5 hash of various data/parameters used to uniquely identify this device

    uint16_t dlBlockNum = 0;                    // download block count; should be reset for each separate transfer
    uint16_t ulBlockNum = 0;                    // upload block count; should be reset for each separate transfer

    pfnFwUpdateProgress progressFn;
    pfnFwUpdateStatus statusFn;

    /**
     * @brief OTP section
     */
    typedef struct {
        uint32_t serialNumber;   //! Inertial Sense serial number
        uint16_t lotNumber;      //! Inertial Sense lot number
        uint16_t hardwareId;     //! Inertial Sense Hardware Id (type/version)
        char date[16];           //! Inertial Sense manufacturing date (YYYYMMDDHHMMSS)
    } otp_info_t;

    int detach(uint8_t timeout);

    int download(uint16_t& wValue, uint8_t *buf, uint16_t len);

    int upload(uint16_t& wValue, uint8_t *buf, uint16_t len);

    int getStatus(dfu_status *status, uint32_t *delay, dfu_state *state, uint8_t *i_string);

    int clearStatus();

    int getState(dfu_state *buf);

    int abort();

    int reset();

    int waitForState(dfu_state required_state, dfu_state* actual_state = nullptr );

    int setAddress(uint16_t& wValue, uint32_t address);

    int readMemory(uint32_t memloc, uint8_t *rxBuf, size_t rxLen);


    static DFUDevice::otp_info_t *decodeOTPData(uint8_t *raw, int len);

    static int findDescriptor(const uint8_t *desc_list, int list_len, uint8_t desc_type, void *res_buf, int res_size);

    static int decodeMemoryPageDescriptor(const std::string& altSetting, dfu_memory_t& segment);
};

class ISDFUFirmwareUpdater : public fwUpdate::FirmwareUpdateDevice {

public:

    /**
     * Constructor to establish a connected to the specified device, and optionally validating against hdwId and serialNo
     * @param device the libusb device which identifies the connected device to update. This is NOT a libusb_device_handle! If null, this function will use to first detected DFU device which matches the hdwId and/or serialNo
     * @param hdwId the hardware id (from manufacturing info) used to identify which specific hdwType + hdwVer we should be targeting (used in validation)
     * @param serialNo the device-specific unique Id (or serial number) that is used to uniquely identify a particular device (used in validation)
     */
    ISDFUFirmwareUpdater(libusb_device *device, uint32_t hdwId, uint32_t serialNo);

    static size_t getAvailableDevices(std::vector<DFUDevice *> &devices, uint16_t vid = 0x0000, uint16_t pid = 0x0000);

    static size_t filterDevicesByFingerprint(std::vector<DFUDevice *> &devices, md5hash_t fingerprint);

    static size_t filterDevicesByTargetType(std::vector<DFUDevice *> &devices, fwUpdate::target_t target);

    static bool isDFUDevice(libusb_device *usbDevice, uint16_t vid, uint16_t pid);


private:
    static std::mutex dfuMutex;
    DFUDevice *curDevice;
    typedef void (*pfnFwUpdateStatus)(void* obj, int logLevel, const char* msg, ...);
    typedef void (*pfnFwUpdateProgress)(void* obj, int stepNo, int totalSteps, float percent);

};

} // namespace dfu
#endif //IS_DFU_FIRMWAREUPDATER_H
