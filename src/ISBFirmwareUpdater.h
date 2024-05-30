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
#include "serialPort.h"
#include "ISFirmwareUpdater.h"
#include "protocol/FirmwareUpdate.h"


// Delete this and assocated code in Q4 2022 after bootloader v5a is out of circulation. WHJ
#define SUPPORT_BOOTLOADER_V5A

/** uINS bootloader baud rate */
#define IS_BAUD_RATE_BOOTLOADER 921600

#define BOOTLOADER_RETRIES          100
#define BOOTLOADER_RESPONSE_DELAY   10
#define BOOTLOADER_REFRESH_DELAY    500
#define MAX_VERIFY_CHUNK_SIZE       1024
#define BOOTLOADER_TIMEOUT_DEFAULT  1000
#define MAX_SEND_COUNT              510

// logical page size, offsets for pages are 0x0000 to 0xFFFF - flash page size on devices will vary and is not relevant to the bootloader client
#define FLASH_PAGE_SIZE 65536



class ISBFirmwareUpdater : public fwUpdate::FirmwareUpdateDevice {

public:
    /**
     * Constructor to establish a connected to the specified device, and optionally validating against hdwId and serialNo
     * @param device the libusb device which identifies the connected device to update. This is NOT a libusb_device_handle! If null, this function will use to first detected DFU device which matches the hdwId and/or serialNo
     * @param hdwId the hardware id (from manufacturing info) used to identify which specific hdwType + hdwVer we should be targeting (used in validation)
     * @param serialNo the device-specific unique Id (or serial number) that is used to uniquely identify a particular device (used in validation)
     */
    ISBFirmwareUpdater(serial_port_t* port, uint32_t hdwId, uint32_t serialNo);

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

    serial_port_t* m_port;
    int m_baud;

    uint32_t m_sn;                      // Inertial Sense serial number, i.e. SN60000
    uint16_t m_hdw;                     // Inertial Sense Hardware Type (IMX, GPX, etc)
    uint8_t m_isb_major;                // ISB Major revision on device
    char m_isb_minor;                   // ISB Minor revision on device
    bool isb_mightUpdate;               // true if device will be updated if bootloader continues

    struct {
        bool is_evb;                    // Available on version 6+, otherwise false
        int processor;       // Differentiates between uINS-3 and IMX-5
        bool rom_available;             // ROM bootloader is available on this port

        uint32_t app_offset;            // Helps in loading bin files
        uint32_t verify_size;           // Chunk size, limited on Windows
    } m_isb_props;

    static std::vector<uint32_t> serial_list;
    static std::mutex serial_list_mutex;

    static std::vector<uint32_t> rst_serial_list;
    static std::mutex rst_serial_list_mutex;

    // typedef void (*pfnFwUpdateStatus)(void *obj, int logLevel, const char *msg, ...);
    // typedef void (*pfnFwUpdateProgress)(void *obj, int stepNo, int totalSteps, float percent);

    fwUpdate::pfnProgressCb progressCb;
    fwUpdate::pfnStatusCb statusCb;

    eImageSignature check_is_compatible();
    is_operation_result sync();
    is_operation_result sync(serial_port_t *s);
    int checksum(int checkSum, uint8_t* ptr, int start, int end, int checkSumPosition, int finalCheckSum);
    is_operation_result erase_flash();

};

#endif //IS_ISB_FIRMWAREUPDATER_H
