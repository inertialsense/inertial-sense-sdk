#ifndef __IS_UINS_SDK_COMPAT_H
#define __IS_UINS_SDK_COMPAT_H

#include "uins_types.h"

#ifdef __cplusplus
extern "C" {

// TODO: namespace
// TODO: api version constant
// TODO: winapi macros

#endif

uins_device uins_31();
uins_device uins_40();
uins_device uins_50();

/**
 * @brief probes the USB bus, builds device uris and adds them to the list
 * 
 * @param list list of device uris
 * @param callback_fn called for each device found
 */
void uins_list_devices(uins_device_uri_list* list, uins_list_devices_callback_fn callback_fn);

/**
 * @brief copies a device uri to the list
 * 
 * @param list list of device uris
 * @param uri new uri to add to the list
 */
void uins_add_device(uins_device_uri_list* list, uins_device_uri uri);

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

/** changes the log level of the device interface
 * 0: nothing
 * 1: error only (default)
 * 2: warning
 * 3: info
 * 4: debug
 * 5: silly
*/
uins_operation_result uins_change_log_level(uins_device_interface* interface, uins_device_interface_log_level log_level);

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
