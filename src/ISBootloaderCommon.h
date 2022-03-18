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
#include "ISBootloaderDfu.h"

#ifdef __cplusplus
extern "C" {

// TODO: namespace
// TODO: api version constant
// TODO: winapi macros

#endif

/*
    Steps:
        - Create an object of type `is_device_uri_list`
        - Probe device list with `is_probe_device_list()` to fill the uri list 
            with identifiers for each attached device
        - Get a device interface by passing the `is_device` struct to `is_create_device_interface()`
        - (DFU/UART) `is_get_libusb_handles()` gives a list of handles matching the interface
        - Pass one of the handles to `is_update_flash()`. Threading should start here.
        - Call `is_release_libusb_handles()` to release libusb handles and exit libusb
        - Free device interface with `is_destroy_device_interface`
        - Free device list with `is_free_device_list`
*/

is_device uins_3(uint8_t minor);
is_device uins_4(uint8_t minor);
is_device uins_5(uint8_t minor);
is_device evb_2(uint8_t minor);

/**
 * @brief Create a device interface object
 * 
 * @param device The device type to search for
 * @return a newly allocated device interface on the heap
 * @see is_destroy_device_interface 
 */
is_device_interface* is_create_device_interface(
    is_device device
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
 * @param device_list libusb-generated device list, will be allocated and filled in this function
 * @param device_count number of devices in `device_list`
 * @param match_list pass in an array of `libusb_device_handle*`, function will fill with list of device handles
 * @param match_count number of device handles at end of function execution
 * @return is_operation_result 
 */
is_operation_result is_get_libusb_handles(
    const is_device_interface const * interf, 
    libusb_context* ctx,
    libusb_device** device_list, 
    size_t* device_count,
    libusb_device_handle** match_list,
    size_t* match_count
);

/**
 * @brief Release (close) libusb devices and libusb_exit()  
 * 
 * @param device_list list of all devices found by libusb
 * @param match_list matching device list created by `is_get_libusb_handles`
 * @param match_count numer of devices matched by `is_get_libusb_handles`
 * @return is_operation_result 
 */
is_operation_result is_release_libusb_handles(
    libusb_device** device_list, 
    libusb_device_handle** match_list,
    size_t match_count
);

/**
 * @brief Write flash to device
 * 
 * @param context setup struct of type is_device_context
 */
void is_update_flash(void* context);

void is_print_device_info(
	libusb_context* ctx,
	struct libusb_device *dev,
	struct libusb_device_descriptor* desc,
	struct libusb_config_descriptor* cfg,
	libusb_device_handle* dev_handle
);

#ifdef __cplusplus
}
#endif

#endif // __IS_BOOTLOADER_COMMON_H
