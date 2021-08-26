#ifndef __IS_UINS_SDK_COMPAT_H
#define __IS_UINS_SDK_COMPAT_H

#include "uins_types.h"

#ifdef __cplusplus
extern "C" {
#endif

uins_device uins_31();
uins_device uins_40();
uins_device uins_50();

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
uins_device_interface* uins_create_device_interface(
    uins_device device,
    const uins_device_uri unique_identifier
);

/** performs any necessary flush or clean up operations, releases instance data resources and frees heap memory from create */
uins_operation_result uins_destroy_device_interface(uins_device_interface* interface);

/** open the device interface */
uins_operation_result uins_open(uins_device_interface* interface);

/** close the device interface */
uins_operation_result uins_close(uins_device_interface* interface);

/** read the specified number of bytes from the device interface into the buffer */
uins_operation_result uins_read(uins_device_interface* interface, int read_count, uins_data_buffer buffer);

/** write the specified number of bytes from the buffer onto the device interface */
uins_operation_result uins_write(uins_device_interface* interface, int write_count, uins_data_buffer buffer);

/** copy hex file from this machine to the device interface */
uins_operation_result uins_update_flash(
    const uins_device_interface* interface,
    const char* firmware_file_path,
    uins_update_flash_style firmware_type,
    uins_verification_style verification_style,
    pfnUinsDeviceInterfaceError error_callback,
    pfnUinsDeviceInterfaceTaskProgress upload_progress_callback,
    pfnUinsDeviceInterfaceTaskProgress verify_progress_callback,
    const void* user_data
);

#ifdef __cplusplus
}
#endif

#endif // __IS_UINS_SDK_COMPAT_H
