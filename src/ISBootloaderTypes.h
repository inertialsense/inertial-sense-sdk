/**
 * @file ISBootloaderTypes.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense definitions for bootloading devices
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_TYPES_H
#define __IS_BOOTLOADER_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    IS_DEVICE_INTERFACE_FLAG_SAMBA      = 0b00000000,  // default
    IS_DEVICE_INTERFACE_FLAG_DFU        = 0b00000001,
    IS_DEVICE_INTERFACE_FLAG_STM32UART  = 0b00000010,

    IS_DEVICE_INTERFACE_FLAG_RSVD1      = 0b00000100,
    IS_DEVICE_INTERFACE_FLAG_RSVD2      = 0b00001000,
    IS_DEVICE_INTERFACE_FLAG_RSVD3      = 0b00010000,
    IS_DEVICE_INTERFACE_FLAG_RSVD4      = 0b00100000,
    IS_DEVICE_INTERFACE_FLAG_RSVD5      = 0b01000000,

    IS_DEVICE_INTERFACE_FLAG_DEBUG      = 0b10000000,
} is_device_interface_flags;

typedef enum {
    IS_UPDATE_BOOTLOADER        = 1,
    IS_UPDATE_FIRMWARE          = 2,
} is_update_flash_style;

typedef enum {
    IS_VERIFY_ON  = 1,
    IS_VERIFY_OFF = 2
} is_verification_style;

typedef uint8_t communications_flags;   // 1111 1111

typedef enum {
    IS_OP_ERROR     = 0,
    IS_OP_OK        = 1
} is_operation_result;

typedef enum {
    IS_SCHEME_UNKNOWN = 0,
    IS_SCHEME_SAMBA,
    IS_SCHEME_DFU,
    IS_SCHEME_STM32UART
} is_device_scheme;

typedef enum {
    // IS_SN_MAX_SIZE_V3 = 13
    // IS_SN_MAX_SIZE_V4 = 13
    IS_SN_MAX_SIZE = 20
} is_serial_number_max_size;

typedef enum {
    IS_COMPORT_MAX_SIZE = 16
} is_comport_port_max_length;

typedef enum {
    IS_DEVICE_MATCH_FLAG_VID        = 0b00000001,
    IS_DEVICE_MATCH_FLAG_PID        = 0b00000010,
    IS_DEVICE_MATCH_FLAG_SN         = 0b00000100,
    IS_DEVICE_MATCH_FLAG_RSVD1      = 0b00001000,
    IS_DEVICE_MATCH_FLAG_RSVD2      = 0b00010000,
    IS_DEVICE_MATCH_FLAG_RSVD3      = 0b00100000,
    IS_DEVICE_MATCH_FLAG_RSVD4      = 0b01000000,
    IS_DEVICE_MATCH_FLAG_RSVD5      = 0b10000000,
} is_device_match_flags;

typedef uint8_t match_flags;   // 1111 1111

typedef struct
{
    is_device_scheme scheme;
    match_flags match;
    char serial_number[IS_SN_MAX_SIZE];
    uint16_t vid;
    uint16_t pid;
    char filename[64];    // String that must exist in firmware file name. '\0' disables check  
} is_device_match_properties;

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_DEBUG = 4,
    IS_LOG_LEVEL_SILLY = 5
} is_device_interface_log_level;

typedef struct
{
    is_device_match_properties match_props;
    is_device_interface_log_level log_level;
    void * instance_data;
    void * handle;
} is_device_interface;

/** Bootloader callback function prototype, return 1 to stay running, return 0 to cancel */
typedef int(*pfnBootloadProgress)(const void* obj, float percent);
/** Bootloader information string function prototype. */
typedef void(*pfnBootloadStatus)(const void* obj, const char* infoString);

typedef struct
{
    is_device_interface* interface;
    char* firmware_file_path;
    char* bootloader_file_path;
    int baud_rate;   // Does not apply in DFU
    bool force_bootloader_update; // Does not apply in DFU
    is_update_flash_style firmware_type;
    is_verification_style verification_style;
    pfnBootloadProgress update_progress_callback;
    pfnBootloadProgress verify_progress_callback;
    pfnBootloadStatus info_callback;
    const void* user_data;
    void* port_handle;  // Cast to serial_port_t or libusb_dev_handle
    void* port_id;  // Can be cast to char* for COM port
    void* thread;
} is_device_context;

#ifdef __cplusplus
}
#endif

#endif // __IS_BOOTLOADER_TYPES_H
