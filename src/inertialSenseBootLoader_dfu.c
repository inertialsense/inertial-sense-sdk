/**
 * @file inertialSenseBootLoader_dfu.h
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief DFU firmware update routines for Inertial Sense products
 * @version 0.1
 * @date 2022-03-14
 * 
 * @copyright Copyright (c) 2022 Inertial Sense, Inc.
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file inertialSenseBootLoader_dfu.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @author Steven Fusco ()
 * @brief Inertial Sense bootloader for DFU-capable STM32 defices
 * @version 0.1
 * @date 2022-03-14
 * 
 */

#include "inertialSenseBootLoader_dfu.h"

int uins_add_device(uins_device_uri_list* list, uins_device_uri uri)
{
	list->devices[list->size] = malloc(strlen(uri) + 1);
	strcpy(list->devices[list->size], uri);

	list->size += 1;
}

void buildUri(uins_device_uri_list* list, struct libusb_device_descriptor* desc, libusb_device_handle* devh, uins_list_devices_callback_fn callback)
{
	int libusb_result;
	const size_t sn_size = 13;
	const size_t url_buffer_size = sn_size + 6;
	char url_buffer[url_buffer_size];
	unsigned char serial_number[sn_size];

	libusb_result = libusb_get_string_descriptor_ascii(devh, desc->iSerialNumber, serial_number, sizeof(serial_number));
	
	snprintf(url_buffer, url_buffer_size, "dfu://%s", serial_number);
	callback(url_buffer);

	uins_add_device(list, url_buffer);
}

void uinsPrintDeviceInfo(
	libusb_context* ctx,
	struct libusb_device *dev,
	struct libusb_device_descriptor* desc,
	struct libusb_config_descriptor* cfg,
	libusb_device_handle* devh
)
{
	int libusb_result;
	struct usb_dfu_func_descriptor dfu_descriptor;
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

void uinsProbeDfuDevices(uins_device_uri_list* uri_list, uins_list_devices_callback_fn callback)
{
	// TODO: libusb_result asserts
	// TODO: integrate this probe function with uinsBootloadFileExDfu and remove old match logic

	libusb_context* ctx;
	libusb_device** device_list;
	ssize_t device_count;
	ssize_t i;
	int cfg_idx;
	int libusb_result;
	struct libusb_device *dev;
	struct libusb_device_descriptor desc;
	libusb_device_handle* dev_handle;
	struct libusb_config_descriptor* cfg;

	libusb_result = libusb_init(&ctx);

	device_count = libusb_get_device_list(ctx, &device_list);
	for (i = 0; i < device_count; ++i) {
		dev = device_list[i];

		libusb_result = libusb_get_device_descriptor(dev, &desc);

		if (desc.idVendor != UINS5_DESCRIPTOR_VENDOR_ID || desc.idProduct != UINS5_DESCRIPTOR_PRODUCT_ID)
		{
			// bail early, must be some other usb device
			continue;
		}

		for (cfg_idx = 0; cfg_idx != desc.bNumConfigurations; cfg_idx++)
		{
			libusb_result = libusb_open(dev, &dev_handle);
			if (0 == libusb_result)
			{
				libusb_result = libusb_get_config_descriptor(dev, cfg_idx, &cfg);

				// uinsPrintDeviceInfo(ctx, dev, &desc, cfg, dev_handle);

				buildUri(uri_list, &desc, dev_handle, callback);

				libusb_free_config_descriptor(cfg);
				libusb_close(dev_handle);
			}
			else
			{
				fprintf(stderr, "failed to open device: %s", libusb_error_name(libusb_result));
			}
		}
	}

	libusb_free_device_list(device_list, 1);
	libusb_exit(ctx);
}

int uinsBootloadFileExDfu(const uins_device_context const * context, uins_dfu_config* config)
{
	int expected_size = 0;
	unsigned int transfer_size = 0;
	libusb_context *ctx;
	int num_sections = 0;
	
	ihex_image_section_t image[NUM_IHEX_SECTIONS];
	num_sections = ihex_load_sections(config->filename, image, NUM_IHEX_SECTIONS);

	ret = libusb_init(&ctx);
	if (ret)
	{
		uinsLogError(context, ret, "unable to initialize libusb");
		return EX_IOERR;
	}

	if (context->interface->log_level > IS_LOG_LEVEL_DEBUG) {
#if defined(LIBUSB_API_VERSION) && LIBUSB_API_VERSION >= 0x01000106
		libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);
#else
		libusb_set_debug(ctx, 255);
#endif
	}
probe:
	probe_devices(ctx, &config);

	return ret;
}

eDfuError dfu_GETSTATUS(libusb_device_handle** dev_handle, eDfuStatus* status, uint32_t* delay, eDfuState* state, uint8_t* i_string)
{
	uint8_t buf[6] = { 0 };

	if (libusb_control_transfer(*dev_handle, 0b10100001, 0x03, 0, 0, buf, 6, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	*status = (eDfuStatus)buf[0];
	*delay = (buf[3] << 16) | (buf[2] << 8) | buf[1];
	*state = (eDfuState)buf[4];
	*i_string = buf[5];				// Index of the description string. 0 = doesn't exist

	return DFU_ERROR_NONE;
}

eDfuError dfu_CLRSTATUS(libusb_device_handle** dev_handle)
{
	if (libusb_control_transfer(*dev_handle, 0b00100001, 0x04, 0, 0, NULL, 0, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

eDfuError dfu_GETSTATE(libusb_device_handle** dev_handle, uint8_t* buf)
{
	if (libusb_control_transfer(*dev_handle, 0b10100001, 0x05, 0, 0, buf, 1, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

eDfuError dfu_ABORT(libusb_device_handle** dev_handle)
{
	if (libusb_control_transfer(*dev_handle, 0b00100001, 0x06, 0, 0, NULL, 0, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

eDfuError dfu_UPLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len)
{
	if (libusb_control_transfer(*dev_handle, 0b10100001, 0x02, wValue, 0, buf, len, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

eDfuError dfu_DNLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len)
{
	if (libusb_control_transfer(*dev_handle, 0b00100001, 0x01, wValue, 0, buf, len, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

eDfuError dfu_DETACH(libusb_device_handle** dev_handle, uint8_t timeout)
{
	if (libusb_control_transfer(*dev_handle, 0b00100001, 0x00, timeout, 0, NULL, 0, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

eDfuError dfu_wait(libusb_device* dev, libusb_device_handle** dev_handle, eDfuState required_state)
{
	eDfuStatus status;
	uint32_t waitTime = 0;
	eDfuState state;
	uint8_t stringIndex;

	uint8_t tryCounter = 0;

	eDfuError ret = dfu_GETSTATUS(dev_handle, &status, &waitTime, &state, &stringIndex);
	if (ret != DFU_ERROR_NONE) return ret;

	while (status != DFU_STATUS_OK || state != required_state)
	{
		if (status != DFU_STATUS_OK) dfu_CLRSTATUS(dev_handle);

		Sleep(waitTime);	// TODO: Make pause work with multi-threading

		ret = dfu_GETSTATUS(dev_handle, &status, &waitTime, &state, &stringIndex);
		if (ret != DFU_ERROR_NONE) return ret;

		tryCounter++;
		if (tryCounter > 4) return DFU_ERROR_TIMEOUT;
	}

	return DFU_ERROR_NONE;
}
