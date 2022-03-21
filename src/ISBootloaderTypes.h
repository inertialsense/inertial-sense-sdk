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

#include "libusb/libusb.h"

#include "serialPort.h"

#ifndef BOOTLOADER_ERROR_LENGTH
#define BOOTLOADER_ERROR_LENGTH	512		// Set to zero to disable
#endif

extern const char* is_uins_5_firmware_needle;
extern const char* is_uins_3_firmware_needle;
extern const char* is_evb_2_firmware_needle;

typedef enum {
    IS_DEVICE_UNKNOWN = 0,
    IS_DEVICE_UINS = 1,
    IS_DEVICE_EVB = 2,
} is_device_type;

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
    IS_DEVICE_MATCH_FLAG_TYPE       = 0b00001000,
    IS_DEVICE_MATCH_FLAG_MAJOR      = 0b00010000,
    IS_DEVICE_MATCH_FLAG_MINOR      = 0b00100000,
    IS_DEVICE_MATCH_FLAG_RSVD1      = 0b01000000,
    IS_DEVICE_MATCH_FLAG_RSVD2      = 0b10000000,
} is_device_match_flags;

typedef uint8_t match_flags;   // 1111 1111

typedef struct
{
    match_flags match;
    is_device_type type;
    uint8_t major;
    uint8_t minor; 
    char serial_number[IS_SN_MAX_SIZE];
    uint16_t vid;
    uint16_t pid;
} is_device_match_properties;

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_DEBUG = 4,
    IS_LOG_LEVEL_SILLY = 5
} is_device_interface_log_level;

/** Bootloader callback function prototype, return 1 to stay running, return 0 to cancel */
typedef int(*pfnBootloadProgress)(const void* obj, float percent);
/** Bootloader information string function prototype. */
typedef void(*pfnBootloadStatus)(const void* obj, const char* infoString);

typedef enum {
    IS_HANDLE_TYPE_LIBUSB,
    IS_HANDLE_TYPE_SERIAL
} is_handle_type;

typedef struct
{
    is_handle_type status;
    const char* port_name;          // COM port name. Invalid in DFU mode.
    char serial_num[IS_SN_MAX_SIZE];            
    serial_port_t* port;            // Invalid once device enters DFU mode
    libusb_device_handle* libusb;   // DFU only. Invalid until DFU mode reached.
} is_device_handle;


typedef struct
{
    is_device_match_properties match_props;
    is_device_scheme scheme;
    const char* firmware_file_path;
    const char* bootloader_file_path;               // Does not apply in DFU
    bool force_bootloader_update;                   // Does not apply in DFU
    int baud_rate;                                  
    is_verification_style verification_style;
    pfnBootloadProgress update_progress_callback;
    pfnBootloadProgress verify_progress_callback;
    pfnBootloadStatus info_callback;
    const void* user_data;
    is_device_handle handle;
    void* thread;
    bool success;
    char error[BOOTLOADER_ERROR_LENGTH];
} is_device_context;


#ifdef __cplusplus
}
#endif

#endif // __IS_BOOTLOADER_TYPES_H
