#ifndef __IS_UINS_TYPES_H
#define __IS_UINS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IS_DEVICE_INTERFACE_FLAG_SAM   = 0b00000000,  // default
    IS_DEVICE_INTERFACE_FLAG_DFU   = 0b00000001,
    IS_DEVICE_INTERFACE_FLAG_UART  = 0b00000010,

    IS_DEVICE_INTERFACE_FLAG_RSVD1 = 0b00000100,
    IS_DEVICE_INTERFACE_FLAG_RSVD2 = 0b00001000,
    IS_DEVICE_INTERFACE_FLAG_RSVD3 = 0b00010000,
    IS_DEVICE_INTERFACE_FLAG_RSVD4 = 0b00100000,
    IS_DEVICE_INTERFACE_FLAG_RSVD5 = 0b01000000,

    IS_DEVICE_INTERFACE_FLAG_DEBUG = 0b10000000,
} uins_device_interface_flags;

typedef enum {
    IS_UPDATE_BOOTLOADER           = 1,
    IS_UPDATE_APPLICATION_FIRMWARE = 2
} uins_update_flash_style;

typedef enum {
    IS_VERIFY_ON  = 1,
    IS_VERIFY_OFF = 2
} uins_verification_style;

typedef unsigned char communications_flags;   // 1111 1111

typedef enum {
    IS_OP_ERROR     = 0,
    IS_OP_OK        = 1
} uins_operation_result;

typedef struct
{
    int version_major;
    int version_minor;
    communications_flags bootloader_flash_support;
} uins_device;

typedef unsigned char * uins_device_uri;

typedef enum {
    IS_SCHEME_UNKNOWN = 0,
    IS_SCHEME_SAM,
    IS_SCHEME_DFU,
    IS_SCHEME_UART
} uins_device_scheme;

typedef enum uins_serial_number_max_size
{
    // IS_SN_MAX_SIZE_V3 = 13
    // IS_SN_MAX_SIZE_V4 = 13
    IS_SN_MAX_SIZE_V5 = 13
} uins_serial_number_max_size;

typedef struct uins_device_uri_properties
{
    uins_device_scheme scheme;
    char serial_number[IS_SN_MAX_SIZE_V5];
} uins_device_uri_properties;

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_DEBUG = 4,
    IS_LOG_LEVEL_SILLY = 5
} uins_device_interface_log_level;

/** a unique id for a device interface
 * 
 * Examples:
 *  file://dev/ttyACM0
 *  sam://vendorid/productid
 *  dfu://vendorid/productid/altid/index_number/f .... 
 *  uart://vendorid/productid/115200
 */
typedef struct uins_device_interface
{
    uins_device device;
    uins_device_uri_properties uri_properties;
    int read_timeout_ms;
    int write_timeout_ms;
    uins_device_interface_log_level log_level;
    void * instance_data;
} uins_device_interface;

typedef void(*pfnUinsDeviceInterfaceError)(const uins_device_interface const * interface, const void* user_data, int error_code, const char * error_message);
typedef int(*pfnUinsDeviceInterfaceTaskProgress)(const uins_device_interface const * interface, const void* user_data, float percent);

typedef const unsigned char * const uins_data_buffer;

typedef struct uins_device_context
{
    const uins_device_interface const * interface;
    const void* user_data;
    pfnUinsDeviceInterfaceTaskProgress progress_callback;
    pfnUinsDeviceInterfaceError error_callback;
} uins_device_context;

typedef void (*uins_list_devices_callback_fn)(uins_device_uri);

typedef struct uins_device_uri_list
{
    int size;
    uins_device_uri devices[256];
} uins_device_uri_list;

#ifdef __cplusplus
}
#endif

#endif // __IS_UINS_TYPES_H
