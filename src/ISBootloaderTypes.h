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
    IS_UPDATE_UINS_FIRMWARE     = 2,
    IS_UPDATE_EVB_FIRMARE       = 3,    
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
    IS_UINS         = 0,
    IS_EVB          = 1,
} is_device_type;

typedef struct
{
    is_device_type type; 
    int version_major;
    int version_minor;
    communications_flags bootloader_flash_support;
} is_device;

typedef unsigned char * is_device_uri;

typedef enum {
    IS_SCHEME_UNKNOWN = 0,
    IS_SCHEME_SAMBA,
    IS_SCHEME_DFU,
    IS_SCHEME_STM32UART
} is_device_scheme;

typedef enum {
    // IS_SN_MAX_SIZE_V3 = 13
    // IS_SN_MAX_SIZE_V4 = 13
    IS_SN_MAX_SIZE_V5 = 13
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
} is_device_interface_flags;

typedef uint8_t match_flags;   // 1111 1111

typedef struct
{
    is_device_scheme scheme;
    match_flags match;
    char serial_number[IS_SN_MAX_SIZE_V5];
    uint16_t vid;
    uint16_t pid;
} is_device_uri_properties;

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
    is_device device;
    is_device_uri_properties uri_properties;
    void* dev_handle;   // usually cast to libusb_device_handle
    is_device_interface_log_level log_level;
    void * instance_data;
} is_device_interface;

typedef void(*pfnIsDeviceInterfaceError)(const is_device_interface const * interface, const void* user_data, int error_code, const char * error_message);
typedef int(*pfnIsDeviceInterfaceTaskProgress)(const is_device_interface const * interface, const void* user_data, float percent);

typedef const unsigned char * const uins_data_buffer;

typedef struct
{
    const is_device_interface const * interface;
    const void* user_data;
    pfnIsDeviceInterfaceTaskProgress progress_callback;
    pfnIsDeviceInterfaceError error_callback;
} is_device_context;

typedef void (*is_list_devices_callback_fn)(is_device_uri);

typedef struct
{
    int size;
    is_device_uri devices[256];
} is_device_uri_list;

#ifdef __cplusplus
}
#endif

#endif // __IS_BOOTLOADER_TYPES_H
