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

const md5hash_t DFU_FINGERPRINT_STM32L4 = {.dwords = {0x428a299c, 0x0af5e729, 0x86dd575c, 0x7bd1b7ae}};
const md5hash_t DFU_FINGERPRINT_STM32U5 = {.dwords = {0x076e3e2e, 0xdce8b63b, 0x067292ec, 0xc4c985b9}};

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

typedef enum {
    STM32_DFU_INTERFACE_FLASH = 0, // @Internal Flash
    STM32_DFU_INTERFACE_OPTIONS = 1, // @Option Bytes
    STM32_DFU_INTERFACE_OTP = 2, // @OTP Memory
    STM32_DFU_INTERFACE_FEATURES = 3  // @Device Feature
} dfu_interface_alternatives;

/**
    path="3-11.3", alt=3, name="@Device Feature/0xFFFF0000/01*004 e"
    path="3-11.3", alt=2, name="@OTP Memory /0x1FFF7000/01*0001Ke"
    path="3-11.3", alt=1, name="@Option Bytes  /0x1FFF7800/01*040 e"
    path="3-11.3", alt=0, name="@Internal Flash  /0x08000000/0256*0002Kg"

    path="3-11.4", alt=2, name="@OTP Memory   /0x0BFA0000/01*512 e"
    path="3-11.4", alt=1, name="@Option Bytes   /0x40022040/01*64 e"
    path="3-11.4", alt=0, name="@Internal Flash   /0x08000000/128*08Kg"
**/

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
    IS_PROCESSOR_STM32U5,           // GPX-1, IMX-5.1

    IS_PROCESSOR_NUM,               // Must be last
} eProcessorType;


class DFUDevice {
public:

    DFUDevice(libusb_device *device) : usbDevice(device), usbHandle(nullptr) {
        fetchDeviceInfo();
    }

    bool isConnected() { return (usbHandle != nullptr) && (libusb_get_device(usbHandle) != nullptr); }

    /**
     * Connect and establish USB DFU status is IDLE
     * @return
     */
    dfu_error open();

    dfu_error writeFirmware(std::string filename, uint64_t baseAddress = 0x08000000);

    /**
     * Erases the flash memory on the device only where the image will live
     * @param image_sections
     * @param image an array of image sections which need to be erased.
     * @return
     */
    dfu_error erase(int image_sections, ihex_image_section_t *image);

    dfu_error close();

    md5hash_t getFingerprint() { return fingerprint; }

protected:
    dfu_error fetchDeviceInfo();

    int get_string_descriptor_ascii(uint8_t desc_index, char *data, int length);

    // static is_operation_result get_serial_number_libusb(libusb_device_handle** handle, uint32_t& sn, std::string& uid, uint8_t sn_idx);
    // static is_operation_result get_serial_number_libusb(libusb_device_handle** handle, uint16_t& hdw, uint32_t& sn, std::string& uid, uint8_t sn_idx);

private:
    // Recipe for DFU UID number:
    // sprintf(ctx->match_props.uid, "%X%X", manufacturing_info->uid[0] + manufacturing_info->uid[2], (uint16_t)(manufacturing_info->uid[1] >> 16));
    libusb_device *usbDevice = nullptr;
    libusb_device_handle *usbHandle = nullptr; // if this is not null, then this should be a valid, open handle.

    uint16_t vid;                               // the vendor id for this device (for filtering/selection)
    uint16_t pid;                               // the product id for this device (for filtering/selection)
    uint8_t iSerialNumber = 0;                  // the index of the USB/DFU descriptor which contains the device serial number
    std::string dfuSerial;                      // the extracted DFU device serial number, from descriptors (see iSerialNumber above)
    std::vector<std::string> dfuDescriptors;    // a array containing the contents of each of the available Alt Identifier strings (used to generate the fingerprint)

    uint32_t sn;                                // Inertial Sense serial number (from OTP data)
    uint16_t hardwareId;                        // Inertial Sense Hardware ID (from OTP data)
    eProcessorType processorType;               // detected processor type/family

    md5hash_t fingerprint;                      // an MD5 hash of various data/parameters used to uniquely identify this device

    /**
     * @brief OTP section
     */
    typedef struct {
        uint32_t serialNumber;   //! Inertial Sense serial number
        uint16_t lotNumber;      //! Inertial Sense lot number
        uint16_t hardwareId;     //! Inertial Sense Hardware Id (type/version)
        char date[16];       //! Inertial Sense manufacturing date (YYYYMMDDHHMMSS)
    } otp_info_t;

    static int detach(libusb_device_handle *handle, uint8_t timeout);

    static int download(libusb_device_handle *handle, uint16_t wValue, uint8_t *buf, uint16_t len);

    static int upload(libusb_device_handle *dev_handle, uint16_t wValue, uint8_t *buf, uint16_t len);

    static int getStatus(libusb_device_handle *handle, dfu_status *status, uint32_t *delay, dfu_state *state, uint8_t *i_string);

    static int clearStatus(libusb_device_handle *handle);

    static int getState(libusb_device_handle *handle, dfu_state *buf);

    static int abort(libusb_device_handle *handle);

    static int reset(libusb_device_handle *dev_handle);

    static int waitForState(libusb_device_handle *dev_handle, dfu_state required_state);

    static int setAddress(libusb_device_handle *dev_handle, uint32_t address);

    static int readMemory(libusb_device_handle *handle, uint32_t memloc, uint8_t *rxBuf, size_t rxLen);


    static DFUDevice::otp_info_t *decodeOTPData(uint8_t *raw, int len);

    static int findDescriptor(const uint8_t *desc_list, int list_len, uint8_t desc_type, void *res_buf, int res_size);
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

    /**
     * @return Returns true if there is an active USB DFU device connection which matches the filter/validation conditions specified in the constructor.
     */

    static int getAvailableDevices(std::vector<DFUDevice *> &devices, uint16_t vid = 0x0000, uint16_t pid = 0x0000);

    static int filterDevicesByFingerprint(std::vector<DFUDevice *> &devices, md5hash_t fingerprint);

    static bool isDFUDevice(libusb_device *usbDevice, uint16_t vid, uint16_t pid);


private:
    DFUDevice *curDevice;

};

}
#endif //IS_DFU_FIRMWAREUPDATER_H
