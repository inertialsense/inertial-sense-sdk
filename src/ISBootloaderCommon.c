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

is_device_interface* is_create_device_interface(
    is_device device
)
{
    is_device_interface* interface = malloc(sizeof(is_device_interface));

    interface->log_level = IS_LOG_LEVEL_ERROR;

    interface->device = device;
    interface->handle = NULL;
    interface->instance_data = NULL;

    if(interface->device.type == IS_UINS && interface->device.version_major == 5)
    {
        interface->match_props.scheme = IS_SCHEME_DFU;
        interface->match_props.match = IS_DEVICE_MATCH_FLAG_VID | IS_DEVICE_MATCH_FLAG_PID | IS_DEVICE_MATCH_FLAG_SN;
        interface->match_props.vid = STM32_DESCRIPTOR_VENDOR_ID;
        interface->match_props.pid = STM32_DESCRIPTOR_PRODUCT_ID;
        interface->match_props.serial_number[0] = '\0';
    }
    else
    {
        free(interface);
        return NULL;
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
    struct libusb_device_descriptor* desc;
    struct libusb_config_descriptor* cfg;
    int ret_libusb;
    *match_count = 0;

    ret_libusb = libusb_init(&ctx);
    if(ret_libusb < 0) return;

    *device_count = libusb_get_device_list(ctx, &device_list);

    for (size_t i = 0; i < *device_count; ++i) {
        dev = device_list[i];

        ret_libusb = libusb_get_device_descriptor(dev, &desc);
        if(ret_libusb < 0) continue;

        if ((desc->idVendor != interf->match_props.vid) && (interf->match_props.match & IS_DEVICE_MATCH_FLAG_VID))
            continue;   // must be some other usb device

        if ((desc->idProduct != interf->match_props.pid) && (interf->match_props.match & IS_DEVICE_MATCH_FLAG_PID))
            continue;   // must be some other usb device

        ret_libusb = libusb_open(dev, &dev_handle);
        if (ret_libusb < 0)
        {
            libusb_close(dev_handle);
            continue;
        }

        unsigned char serial_number[IS_SN_MAX_SIZE];

        // Get the string containing the serial number from the device
        ret_libusb = libusb_get_string_descriptor_ascii(dev_handle, desc->iSerialNumber, serial_number, sizeof(serial_number));
        if (ret_libusb < LIBUSB_SUCCESS) 
        {   // Set the serial number as none
            serial_number[0] = '\0';
        }

        bool sn_matches = !(interf->match_props.match & IS_DEVICE_MATCH_FLAG_SN) || 
            (strncmp(serial_number, interf->match_props.serial_number, IS_SN_MAX_SIZE) == 0) ||
            interf->match_props.serial_number[0] == '\0';

        if(cfg->interface->altsetting[0].bInterfaceClass == 0xFE && 
            cfg->interface->altsetting[0].bInterfaceSubClass == 0x01 && 
            cfg->interface->altsetting[0].bInterfaceProtocol == 0x02 &&
            sn_matches 
        )
        {   // Add 
            match_list[(*match_count)++] = dev_handle;
        }
        else
        {   // Not a DFU device, or serial number doesn't match
            libusb_close(dev_handle);
            continue;
        }
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

void is_update_flash(void* context)
{
    is_device_context* ctx = (is_device_context*)context;
    int ret = IS_OP_ERROR;
    
    if(!ctx->firmware_file_path) return ret;

    if(strstr(ctx->firmware_file_path, ctx->interface->match_props.filename) != NULL) return ret;

    if(ctx->interface->match_props.scheme == IS_SCHEME_DFU)
    {
        int ihex_ret;

        // Load the firmware into memory. Even if we don't use it now (SAM-BA), this is a good check
        ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
        ihex_ret = ihex_load_sections(ctx->firmware_file_path, image, MAX_NUM_IHEX_SECTIONS);
        if(ihex_ret <= 0) return ret;

        ret = is_dfu_flash(ctx, image, ihex_ret, (libusb_device_handle*)ctx->port_handle);

        // Free the memory associated with the firmware image
        ihex_unload_sections(image, ihex_ret);
    }
    else if(ctx->interface->match_props.scheme == IS_SCHEME_SAMBA)
    {
        bootload_params_t params;

        memset(params.error, 0, BOOTLOADER_ERROR_LENGTH);
        serialPortPlatformInit((serial_port_t*)ctx->port_handle);
        serialPortSetPort((serial_port_t*)ctx->port_handle, (const char*)ctx->port_id);
        params.uploadProgress = ctx->update_progress_callback;
        params.verifyProgress = ctx->verify_progress_callback;
        params.statusText = ctx->info_callback;
        params.fileName = ctx->firmware_file_path;
        params.bootName = ctx->bootloader_file_path;
        params.forceBootloaderUpdate = ctx->force_bootloader_update;
        params.port = (serial_port_t*)ctx->port_handle;
        params.verifyFileName = NULL;   // TODO: Add verify
        params.flags.bitFields.enableVerify = (ctx->verification_style == IS_VERIFY_ON);
        params.numberOfDevices = 1; // Unused, set to 1
        params.baudRate = ctx->baud_rate;		

        if(ctx->interface->device.type == IS_EVB)
        {
            strncpy(params.bootloadEnableCmd, "EBLE", 4);
        }
        else
        {
            strncpy(params.bootloadEnableCmd, "BLEN", 4);
        }

        bootloadFileEx(&params);
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