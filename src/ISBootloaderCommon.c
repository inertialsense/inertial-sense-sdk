/**
 * @file ISBootloaderCommon.c
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

#include "ISBootloaderCommon.h"

#include "serialPort.h"
#include "serialPortPlatform.h"
#include "inertialSenseBootLoader.h"
#include "ISBootloaderDfu.h"

static void is_add_device(
    is_device_uri_list* list, 
    struct libusb_device_descriptor* desc, 
    struct libusb_config_descriptor* conf_desc,
    libusb_device_handle* handle, 
    is_list_devices_callback_fn callback
);

is_device uins_3(uint8_t minor)
{
    is_device d;
    d.type = IS_UINS;
    d.version_major = 3;
    d.version_minor = minor;
    d.bootloader_flash_support = IS_DEVICE_INTERFACE_FLAG_SAMBA;
    return d;
}

is_device uins_4(uint8_t minor)
{
    is_device d;
    d.type = IS_UINS;
    d.version_major = 4;
    d.version_minor = minor;
    d.bootloader_flash_support = IS_DEVICE_INTERFACE_FLAG_SAMBA;
    return d;
}

is_device uins_5(uint8_t minor)
{
    is_device d;
    d.type = IS_UINS;
    d.version_major = 5;
    d.version_minor = minor;
    d.bootloader_flash_support = IS_DEVICE_INTERFACE_FLAG_DFU | IS_DEVICE_INTERFACE_FLAG_STM32UART;
    return d;
}

is_device evb_2(uint8_t minor)
{
    is_device d;
    d.type = IS_EVB;
    d.version_major = 2;
    d.version_minor = minor;
    d.bootloader_flash_support = IS_DEVICE_INTERFACE_FLAG_SAMBA;
    return d;
}

void is_probe_device_list(
    is_device_uri_list* uri_list,
    is_list_devices_callback_fn callback
)
{
    int ret_libusb;
    libusb_context* ctx;
    libusb_device** device_list;
    size_t device_count;
    struct libusb_device *dev;
    struct libusb_device_descriptor desc;
    libusb_device_handle* dev_handle;
    struct libusb_config_descriptor* cfg;

    ret_libusb = libusb_init(&ctx);
    if (ret_libusb < LIBUSB_SUCCESS) 
    {
        libusb_exit(ctx);
        return;
    } 

    device_count = libusb_get_device_list(ctx, &device_list);
    for (size_t i = 0; i < device_count; ++i) 
    {
        dev = device_list[i];

        ret_libusb = libusb_open(dev, &dev_handle);
        if (ret_libusb < LIBUSB_SUCCESS) 
        {
            libusb_close(dev_handle); 
            continue;
        }

        ret_libusb = libusb_get_device_descriptor(dev, &desc);
        if (ret_libusb < LIBUSB_SUCCESS) 
        {
            libusb_close(dev_handle); 
            continue;
        }

        for (size_t cfg_idx = 0; cfg_idx < desc.bNumConfigurations; cfg_idx++)
        {
            ret_libusb = libusb_get_config_descriptor(dev, cfg_idx, &cfg);
            if (ret_libusb < LIBUSB_SUCCESS) 
            {
                libusb_free_config_descriptor(cfg);
                libusb_close(dev_handle); 
                continue;
            }

            is_add_device(uri_list, &desc, cfg, dev_handle, callback);

            libusb_free_config_descriptor(cfg);
        }

        libusb_close(dev_handle);
    }

    libusb_free_device_list(device_list, 1);
    libusb_exit(ctx);
}

void is_free_device_list(is_device_uri_list* list)
{
    for (int i=0; i != list->size; ++i)
    {
        free(list->devices[i]);
    }
    list->size = 0;
}

static void is_add_device(
    is_device_uri_list* list, 
    struct libusb_device_descriptor* desc, 
    struct libusb_config_descriptor* conf_desc,
    libusb_device_handle* handle, 
    is_list_devices_callback_fn callback
)
{
    int ret_libusb;
    
    unsigned char serial_number[IS_SN_MAX_SIZE];
    char uri[256];

    // Get the string containing the serial number from the device
    ret_libusb = libusb_get_string_descriptor_ascii(handle, desc->iSerialNumber, serial_number, sizeof(serial_number));
    if (ret_libusb < LIBUSB_SUCCESS) 
    {
        // Set the serial number as 0
        serial_number[0] = '0';
        serial_number[1] = '\0';
    }

    if(conf_desc->interface->altsetting[0].bInterfaceClass == 0xFE && 
        conf_desc->interface->altsetting[0].bInterfaceSubClass == 0x01 && 
        conf_desc->interface->altsetting[0].bInterfaceProtocol == 0x02
    )
    {
        sprintf(uri, "dfu://%s/%04X/%04X", serial_number, desc->idVendor, desc->idProduct);
    }
    else if(desc->bDeviceClass == 0x02 &&
        conf_desc->interface->altsetting[0].bInterfaceClass == 0x02
    )
    {
        sprintf(uri, "samba://%s/%04X/%04X", serial_number, desc->idVendor, desc->idProduct);

        // TODO: Add stm32uart support
    }
    else
    {
        return;
    }
    
    if(callback) callback(uri);

	list->devices[list->size] = malloc(strlen(uri) + 1);
	strcpy(list->devices[list->size], uri);

	list->size += 1;
}

is_device_interface* is_create_device_interface(
    is_device device,
    const is_device_uri uri
)
{
    is_device_interface* interface = malloc(sizeof(is_device_interface));

    interface->log_level = IS_LOG_LEVEL_ERROR;

    // dfu://serialnum

    char uri_scheme[9];
    char serial_number[IS_SN_MAX_SIZE];
    char vid[5];
    char pid[5];
    long val;

    int uri_scan_status = sscanf(uri,
        "%9[^:]%*[:/]%99[^/]/%99[^/]/%s",
        uri_scheme,
        serial_number,
        vid,
        pid);

    val = strtoul(vid, NULL, 16);
    interface->uri_properties.vid = (uint16_t)val;
    val = strtoul(pid, NULL, 16);
    interface->uri_properties.pid = (uint16_t)val;

    strncpy(interface->uri_properties.serial_number, serial_number, IS_SN_MAX_SIZE);

    if(strncmp(uri_scheme, "samba", 5) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_SAMBA;
        interface->uri_properties.match = IS_DEVICE_MATCH_FLAG_VID | IS_DEVICE_MATCH_FLAG_PID;
    }
    else if(strncmp(uri_scheme, "dfu", 3) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_DFU;
        interface->uri_properties.match = IS_DEVICE_MATCH_FLAG_VID | IS_DEVICE_MATCH_FLAG_PID | IS_DEVICE_MATCH_FLAG_SN;
    }
    else if(strncmp(uri_scheme, "stm32uart", 9) == 0)
    {
        interface->uri_properties.scheme = IS_SCHEME_STM32UART;
        interface->uri_properties.match = IS_DEVICE_MATCH_FLAG_VID | IS_DEVICE_MATCH_FLAG_PID | IS_DEVICE_MATCH_FLAG_SN;
        
    }
    else
    {
        interface->uri_properties.scheme = IS_SCHEME_UNKNOWN;
    }

    return interface;
}

is_operation_result is_destroy_device_interface(is_device_interface* interface)
{
    free(interface);

    return IS_OP_OK;
}

is_operation_result is_device_change_log_level(is_device_interface* interface, is_device_interface_log_level log_level)
{
    if (log_level < 0 || log_level > 5)
    {
        return IS_OP_ERROR;
    }

    interface->log_level = log_level;
    return IS_OP_OK;
}

is_operation_result is_get_libusb_handles(
    const is_device_interface const * interf, 
    libusb_context* ctx,
    libusb_device** device_list, 
    size_t* device_count,
    libusb_device_handle** match_list,
    size_t* match_count
)
{
    libusb_device* dev;
    libusb_device_handle* dev_handle;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor* cfg;
    int libusb_result;
    *match_count = 0;

    libusb_result = libusb_init(&ctx);
    *device_count = libusb_get_device_list(ctx, &device_list);

    for (size_t i = 0; i < *device_count; ++i) {
        dev = device_list[i];

        libusb_result = libusb_get_device_descriptor(dev, &desc);

        if ((desc.idVendor != interf->uri_properties.vid) && (interf->uri_properties.match & IS_DEVICE_MATCH_FLAG_VID))
            continue;   // must be some other usb device

        if ((desc.idProduct != interf->uri_properties.pid) && (interf->uri_properties.match & IS_DEVICE_MATCH_FLAG_PID))
            continue;   // must be some other usb device

        libusb_result = libusb_open(dev, &dev_handle);
        if (libusb_result == 0)
        {
            if(!(interf->uri_properties.match & IS_DEVICE_MATCH_FLAG_SN))
            {
                // Don't check for serial number (SAM-BA bootloaders, etc.)
                // Device found
                match_list[(*match_count)++] = dev_handle;
                continue;
            }

            int libusb_result;
            const size_t sn_size = IS_SN_MAX_SIZE;
            const size_t url_buffer_size = sn_size + 6;
            char url_buffer[url_buffer_size];
            unsigned char serial_number[sn_size];

            // Get the string containing the serial number from the device
            libusb_result = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, serial_number, sizeof(serial_number));
        
            if(strncmp(serial_number, interf->uri_properties.serial_number, IS_SN_MAX_SIZE) == 0)
            {
                // Device found
                match_list[(*match_count)++] = dev_handle;
                continue;
            }
        }

        libusb_close(dev_handle);
    }

    return IS_OP_OK;
}

is_operation_result is_release_libusb_handles(
    libusb_device** device_list, 
    libusb_device_handle** match_list,
    size_t match_count
)
{
    for (size_t i = 0; i < match_count; ++i) {
        libusb_close(match_list[i]);
    }

    libusb_free_device_list(device_list, 1);
    libusb_exit(NULL);

    return IS_OP_OK;
}

is_operation_result is_update_flash(
    const is_device_interface* interface,
    const char* firmware_file_path,
    const char* com_port,
    is_update_flash_style firmware_type,
    is_verification_style verification_style,
    pfnIsDeviceInterfaceError error_callback,
    pfnIsDeviceInterfaceTaskProgress update_progress_callback,
    pfnIsDeviceInterfaceTaskProgress verify_progress_callback,
    const void* user_data,
    void* dev_handle
)
{
    int ret = IS_OP_ERROR;
    
    if(!firmware_file_path) return ret;

    if(interface->uri_properties.scheme == IS_SCHEME_DFU)
    {
        int ihex_ret;

        // Load the firmware into memory. Even if we don't use it now (SAM-BA), this is a good check
        ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
        ihex_ret = ihex_load_sections(firmware_file_path, image, MAX_NUM_IHEX_SECTIONS);
        if(ihex_ret <= 0) return ret;

        is_device_context context;
        context.interface = interface;
        context.user_data = user_data;
        context.progress_callback = update_progress_callback;
        context.error_callback = error_callback;

        ret = is_dfu_flash(&context, image, ihex_ret, (libusb_device_handle*)dev_handle);

        // Free the memory associated with the firmware image
        ihex_unload_sections(image, ihex_ret);
    }
    else if(interface->uri_properties.scheme == IS_SCHEME_SAMBA)
    {
        // TODO: SAM-BA support
    }
    else if(interface->uri_properties.scheme == IS_SCHEME_STM32UART)
    {
        // TODO: UART support
    }

    

    return ret;
}

void is_print_device_info(
	libusb_context* ctx,
	struct libusb_device *dev,
	struct libusb_device_descriptor* desc,
	struct libusb_config_descriptor* cfg,
	libusb_device_handle* devh
)
{
	int libusb_result;
	const struct libusb_interface *uif;
	int intf_idx;
	const struct libusb_interface_descriptor *intf;
	unsigned char interface_name[255];

	printf("---\n");
	printf("vendor id: 0x%x\n", desc->idVendor);
	printf("product id: 0x%x\n", desc->idProduct);

	uint8_t address = libusb_get_device_address(dev);
	printf("address: 0x%x\n", address);

	uint8_t bus_number = libusb_get_bus_number(dev);
	printf("bus number: 0x%x\n", bus_number);

	unsigned char serial_number[255];
	libusb_result = libusb_get_string_descriptor_ascii(devh, desc->iSerialNumber, serial_number, sizeof(serial_number));
	printf("serial number: %s\n", serial_number);

	unsigned char manufacturer_name[255];
	libusb_result = libusb_get_string_descriptor_ascii(devh, desc->iManufacturer, manufacturer_name, sizeof(manufacturer_name));
	printf("manufacturer: %s\n", manufacturer_name);

	for (intf_idx = 0; intf_idx < cfg->bNumInterfaces; intf_idx++)
	{
		uif = &cfg->interface[intf_idx];
		if (!uif)
			break;

		for (int alt_idx = 0; alt_idx < cfg->interface[intf_idx].num_altsetting; alt_idx++)
		{
			intf = &uif->altsetting[alt_idx];
			if (intf->bInterfaceClass != 0xfe || intf->bInterfaceSubClass != 1)
				continue;

			printf("alt index: %d\n", alt_idx);

			libusb_result = libusb_get_string_descriptor_ascii(devh, intf->iInterface, (void *)interface_name, sizeof(interface_name));
			printf("interface name: %s\n", interface_name);

			/*
			libusb_result = libusb_get_descriptor(devh, USB_DT_DFU, 0, (void *)&dfu_descriptor, sizeof(dfu_descriptor));

			printf("bcdDFUVersion: %d\n", dfu_descriptor.bcdDFUVersion);
			printf("bDescriptorType: %d\n", dfu_descriptor.bDescriptorType);
			printf("bLength: %d\n", dfu_descriptor.bLength);
			printf("bmAttributes: %d\n", dfu_descriptor.bmAttributes);
			printf("wDetachTimeOut: %d\n", dfu_descriptor.wDetachTimeOut);
			printf("wTransferSize: %d\n", dfu_descriptor.wTransferSize);
			*/

			printf("\n");
		}
	}

}