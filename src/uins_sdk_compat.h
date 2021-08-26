#ifndef __IS_UINS_SDK_COMPAT_H
#define __IS_UINS_SDK_COMPAT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum uins_device_interface_flags {
    IS_DEVICE_INTERFACE_FLAG_SAM   = 0b00000000,  // default
    IS_DEVICE_INTERFACE_FLAG_DFU   = 0b00000001,
    IS_DEVICE_INTERFACE_FLAG_UART  = 0b00000010,

    IS_DEVICE_INTERFACE_FLAG_RSVD1 = 0b00000100,
    IS_DEVICE_INTERFACE_FLAG_RSVD2 = 0b00001000,
    IS_DEVICE_INTERFACE_FLAG_RSVD3 = 0b00010000,
    IS_DEVICE_INTERFACE_FLAG_RSVD4 = 0b00100000,
    IS_DEVICE_INTERFACE_FLAG_RSVD5 = 0b01000000,
    IS_DEVICE_INTERFACE_FLAG_RSVD6 = 0b10000000,
} uins_device_interface_flags_t;

typedef enum uins_bootloader_verification_style {
    IS_VERIFY_BOOTLOADER_ON  = 1,
    IS_VERIFY_BOOTLOADER_OFF = 2
} uins_bootloader_verification_style_t;

typedef unsigned char communications_flags_t;   // 1111 1111

typedef enum uins_operation_result {
    IS_OP_ERROR = 0,
    IS_OP_OK    = 1
} uins_operation_result_t;

typedef struct uins_device
{
    int version_major;
    int version_minor;
    communications_flags_t bootloader_flash_support;
} uins_device_t;

typedef const unsigned char * uins_device_uri;

/** a unique id for a device interface
 * 
 * Examples:
 *  file://dev/ttyACM0
 *  sam://vendorid/productid
 *  dfu://vendorid/productid/altid
 */
typedef struct uins_device_interface
{
    uins_device_t device;
    uins_device_uri uri;
    int read_timeout_ms;
    int write_timeout_ms;
    void * instance_data;
} uins_device_interface_t;

typedef void(*pfnUinsDeviceInterfaceError)(uins_device_interface_t* interface, const void* user_data, int error_code, const char * error_message);
typedef int(*pfnUinsDeviceInterfaceTaskProgress)(uins_device_interface_t* interface, const void* user_data, float percent);

typedef const unsigned char * const uins_firmware_file_buffer;

uins_device_t uins_31();
uins_device_t uins_40();
uins_device_t uins_50();

/**
 * @brief Create a device interface object
 * 
 * @param device The device type to search for
 * @param interface The device interface to initialize and populate with values.  NULL if not found.
 * @param unique_identifier A unique uri to the device interface
 * @param optional_callback_handler If not NULL, the callback handler to signal when the interface is used for subsequent operations
 * @return a newly allocated device interface on the heap
 * @see uins_destroy_device_interface 
 */
uins_device_interface_t* uins_create_device_interface(
    uins_device_t device,
    const uins_device_uri unique_identifier
);

/** performs any necessary flush or clean up operations, releases instance data resources and frees heap memory from create */
uins_operation_result_t uins_destroy_device_interface(uins_device_interface_t* interface);

/** open the device interface */
uins_operation_result_t uins_open(uins_device_interface_t* interface);

/** close the device interface */
uins_operation_result_t uins_close(uins_device_interface_t* interface);

/** read the specified number of bytes from the device interface into the buffer */
uins_operation_result_t uins_read(uins_device_interface_t* interface, int read_count, uins_firmware_file_buffer buffer);

/** write the specified number of bytes from the buffer onto the device interface */
uins_operation_result_t uins_write(uins_device_interface_t* interface, int write_count, uins_firmware_file_buffer buffer);

/** copy hex file from this machine to the device interface */
uins_operation_result_t uins_update_bootloader(
    const uins_device_interface_t* interface,
    const char* firmware_file_path,
    uins_bootloader_verification_style_t verification_style,
    pfnUinsDeviceInterfaceError error_callback,
    pfnUinsDeviceInterfaceTaskProgress upload_progress_callback,
    pfnUinsDeviceInterfaceTaskProgress verify_progress_callback,
    const void* user_data
);

#ifdef __cplusplus
}
#endif

#endif // __IS_UINS_SDK_COMPAT_H
