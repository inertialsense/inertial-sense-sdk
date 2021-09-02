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
    IS_DEVICE_INTERFACE_FLAG_RSVD6 = 0b10000000,
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
    IS_OP_ERROR = 0,
    IS_OP_OK    = 1
} uins_operation_result;

typedef struct
{
    int version_major;
    int version_minor;
    communications_flags bootloader_flash_support;
} uins_device;

typedef const unsigned char * uins_device_uri;

typedef enum {
    IS_SCHEME_SAM = 0,
    IS_SCHEME_DFU,
    IS_SCHEME_UART
} uins_upload_scheme;

typedef struct
{
    uins_upload_scheme scheme;
    unsigned int vid;
    unsigned int pid;
    unsigned int alt;
} uins_device_uri_properties;


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
    // uins_device_uri uri;
    uins_device_uri_properties uri_properties;
    int read_timeout_ms;
    int write_timeout_ms;
    void * instance_data;
} uins_device_interface;

typedef void(*pfnUinsDeviceInterfaceError)(uins_device_interface* interface, const void* user_data, int error_code, const char * error_message);
typedef int(*pfnUinsDeviceInterfaceTaskProgress)(uins_device_interface* interface, const void* user_data, float percent);

typedef const unsigned char * const uins_data_buffer;


#ifdef __cplusplus
}
#endif

#endif // __IS_UINS_TYPES_H
