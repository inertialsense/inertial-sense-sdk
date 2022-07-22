/**
 * @file ISBootloaderTypes.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense definitions for bootloading devices
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_TYPES_H
#define __IS_BOOTLOADER_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ISConstants.h"

#include <stdint.h>
#include <stdbool.h>

#if PLATFORM_IS_WINDOWS
#pragma warning( push )
#pragma warning( disable : 4200 )
#endif
#include "libusb/libusb.h"
#if PLATFORM_IS_WINDOWS
#pragma warning( pop )
#endif

#include "serialPort.h"


#define IS_DEVICE_LIST_LEN          256
#define IS_FIRMWARE_PATH_LENGTH     256

extern const char* is_uins_5_firmware_needle;
extern const char* is_uins_3_firmware_needle;
extern const char* is_evb_2_firmware_needle;

typedef enum {
    IS_VERIFY_ON  = 1,
    IS_VERIFY_OFF = 2
} is_verification_style;

typedef enum {
    IS_PROCESSOR_SAMx70 = 0,        // uINS-5
    IS_PROCESSOR_STM32L4,           // uINS-3/4, EVB-2

    IS_PROCESSOR_NUM,               // Must be last
} is_processor_type;

typedef enum {
    // BOOTLOADERS must be before APPS because bootloaders may contain app signatures
    IS_IMAGE_SIGN_ISB_STM32L4 = 0x00000001,
    IS_IMAGE_SIGN_ISB_SAMx70_16K = 0x00000002,
    IS_IMAGE_SIGN_ISB_SAMx70_24K = 0x00000004,

    IS_IMAGE_SIGN_UINS_3_16K = 0x00000008,
    IS_IMAGE_SIGN_UINS_3_24K = 0x00000010,
    IS_IMAGE_SIGN_EVB_2_16K = 0x00000020,
    IS_IMAGE_SIGN_EVB_2_24K = 0x00000040,
    IS_IMAGE_SIGN_UINS_5 = 0x00000080,
    
    IS_IMAGE_SIGN_NUM_BITS_USED = 8,
} is_image_signature;

typedef struct
{
    uint8_t major;                  // Bootloader major revision, 1, 2, 3, etc. 
    char minor;                     // Bootloader minor revision, a, b, c, etc.
    bool is_evb;                    // Available on version 6+, otherwise false
    is_processor_type processor;    // Differentiates between uINS-3 and uINS-5
    bool rom_available;             // ROM bootloader is available on this port
    
    uint32_t app_offset;            // Helps in loading bin files
    uint32_t verify_size;           // Chink size, limited on Windows

    char enable_command[5];         // "EBLE" (EVB) or "BLEN" (uINS)
} is_bootloader_properties;

typedef struct
{
    uint8_t uins_version[4];        // Bootloader major revision, 1, 2, 3, etc. 
    uint8_t evb_version[4];         // Bootloader minor revision, a, b, c, etc.
} is_app_properties;

typedef struct
{
    uint32_t serial;                // Inertial Sense serial number, i.e. SN60000
    is_bootloader_properties isb;
    is_app_properties app;
} is_device_properties;

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_DEBUG = 4,
    IS_LOG_LEVEL_SILLY = 5
} is_log_level;

typedef enum {
    IS_HANDLE_TYPE_LIBUSB,
    IS_HANDLE_TYPE_SERIAL
} is_handle_type;

#define IS_DFU_UID_MAX_SIZE     20
#define IS_DFU_LIST_LEN         256

typedef enum
{
    STM32_DESCRIPTOR_VENDOR_ID = 0x0483,
    STM32_DESCRIPTOR_PRODUCT_ID = 0xdf11
} is_dfu_descriptor;

// Recipe for DFU serial number:
// sprintf(ctx->match_props.uid, "%X%X", manufacturing_info->uid[0] + manufacturing_info->uid[2], (uint16_t)(manufacturing_info->uid[1] >> 16));

typedef struct 
{
    char uid[IS_DFU_UID_MAX_SIZE];      // DFU device serial number, from descriptors
    uint32_t sn;                        // Inertial Sense serial number
    uint16_t vid;
    uint16_t pid;
    libusb_device_handle* handle_libusb;
} is_dfu_id;

typedef struct 
{
    is_dfu_id id[IS_DFU_LIST_LEN];
    size_t present;
} is_dfu_list;

typedef struct
{
    is_handle_type status;     
    serial_port_t port;
    char port_name[100];
    int baud;
    is_dfu_id dfu;
} is_device_handle;

/** Bootloader callback function prototype, return 1 to stay running, return 0 to cancel */
typedef is_operation_result(*pfnBootloadProgress)(void* obj, float percent);
/** Bootloader information string function prototype. */
typedef void(*pfnBootloadStatus)(void* obj, const char* infoString, is_log_level level);

typedef struct
{
    // Firmware
    char firmware_path[IS_FIRMWARE_PATH_LENGTH];
    char verify_path[IS_FIRMWARE_PATH_LENGTH];

    // Device
    is_device_properties props;                   
    is_verification_style verify;
    is_device_handle handle;

    // Thread and class info
    void* user_data;
    void* thread;

    // Callbacks
    pfnBootloadProgress update_callback;
    pfnBootloadProgress verify_callback;
    pfnBootloadStatus info_callback;

    // Status
    bool update_in_progress;
    int retries_left;
    float update_progress;
    float verify_progress;
    bool success;
    int device_type;
    bool use_progress;  // Use percentages to compute total progress for all devices
} is_device_context;

typedef enum
{
    IS_DEV_TYPE_NONE = 0,
    IS_DEV_TYPE_SAMBA,
    IS_DEV_TYPE_ISB,
    IS_DEV_TYPE_APP,
    IS_DEV_TYPE_DFU,
} is_device_type;

#ifdef __cplusplus
}
#endif

#endif // __IS_BOOTLOADER_TYPES_H
