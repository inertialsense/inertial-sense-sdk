/**
 * @file ISBootloaderCommon.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating embedded systems
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __IS_BOOTLOADER_COMMON_H
#define __IS_BOOTLOADER_COMMON_H

#include "ISBootloaderTypes.h"

#ifdef __cplusplus
extern "C" {

// TODO: namespace
// TODO: api version constant
// TODO: winapi macros

#endif

/*
    Steps:
        - Create a device with uins_3, uins_4, uins_5, evb_2, etc.
        - 
*/

is_device uins_3(uint8_t minor);
is_device uins_4(uint8_t minor);
is_device uins_5(uint8_t minor);
is_device evb_2(uint8_t minor);

/**
 * @brief probes the USB bus, builds device uris and adds them to the list
 * 
 * @param list list of device uris
 * @param callback_fn called for each device found
 */
void is_probe_device_list(is_device_uri_list* list, is_list_devices_callback_fn callback_fn);

/**
 * @brief frees the memory from the probed devices
 * 
 * @param list list of device uris
 * @param callback_fn 
 */
void is_free_device_list(is_device_uri_list* list);


/**
 * @brief copies a device uri to the list
 * 
 * @param list list of device uris
 * @param uri new uri to add to the list
 */
void is_add_device(is_device_uri_list* list, is_device_uri uri);

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
is_device_interface* is_create_device_interface(
    is_device device,
    const is_device_uri unique_identifier
);

/** performs any necessary flush or clean up operations, releases instance data resources and frees heap memory from create */
is_operation_result is_destroy_device_interface(is_device_interface* interface);

/** changes the log level of the device interface
 * 0: nothing
 * 1: error only (default)
 * 2: warning
 * 3: info
 * 4: debug
 * 5: silly
*/
is_operation_result is_device_change_log_level(is_device_interface* interface, is_device_interface_log_level log_level);

/**
 * @brief Get a list of libusb handles that match the criteria in `interf`
 * 
 * @param interf defines the device parameters to match with
 * @param device_list libusb-generated device list
 * @param device_count number of devices in `device_list`
 * @param match_list pass in an array of `libusb_device_handle*`, function will fill with list of device handles
 * @param match_count number of device handles at end of function execution
 * @return is_operation_result 
 */
is_operation_result is_get_libusb_handles(
    const is_device_interface* const interf, 
    libusb_device** device_list, 
    size_t device_count,
    libusb_device_handle** match_list,
    size_t* match_count
);

/** copy hex file from this machine to the device interface */
is_operation_result is_update_flash(
    const is_device_interface* interface,
    const char* firmware_file_path,
    is_update_flash_style firmware_type,
    is_verification_style verification_style,
    pfnIsDeviceInterfaceError error_callback,
    pfnIsDeviceInterfaceTaskProgress upload_progress_callback,
    pfnIsDeviceInterfaceTaskProgress verify_progress_callback,
    const void* user_data
);

void is_print_device_info(
	libusb_context* ctx,
	struct libusb_device *dev,
	struct libusb_device_descriptor* desc,
	struct libusb_config_descriptor* cfg,
	libusb_device_handle* devh
);

#ifdef __cplusplus
}
#endif

#endif // __IS_BOOTLOADER_COMMON_H
