/**
 * @file ISBootloaderDfu.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating DFU capable devices (STM32)
 * @version 0.1
 * @date 2022-03-15
 * 
 * @copyright Copyright (c) 2022 Inertial Sense, Inc
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderDfu.h"

typedef enum
{
	DFU_ERROR_NONE = 0,
	DFU_ERROR_NO_DEVICE = -1,
	DFU_ERROR_LIBUSB = -2,
	DFU_ERROR_STATUS = -3,
	DFU_ERROR_INVALID_ARG = -4,
	DFU_ERROR_NO_FILE = -5,
	DFU_ERROR_TIMEOUT = -6,
} dfu_error;

typedef enum
{
	DFU_STATUS_OK = 0,
	DFU_STATUS_ERR_TARGET,
	DFU_STATUS_ERR_FILE,
	DFU_STATUS_ERR_WRITE,
	DFU_STATUS_ERR_ERASED,
	DFU_STATUS_ERR_CHECK_ERASED,
	DFU_STATUS_ERR_PROG,
	DFU_STATUS_ERR_VERIFY,
	DFU_STATUS_ERR_ADDRESS,
	DFU_STATUS_ERR_NOTDONE,
	DFU_STATUS_ERR_FIRMWARE,
	DFU_STATUS_ERR_VENDOR,
	DFU_STATUS_ERR_USBR,
	DFU_STATUS_ERR_POR,
	DFU_STATUS_ERR_UNKNOWN,
	DFU_STATUS_ERR_STALLEDPKT,
	
	DFU_STATUS_NUM,
} dfu_status;

typedef enum
{
	DFU_STATE_APP_IDLE = 0,
	DFU_STATE_APP_DETACH,
	DFU_STATE_IDLE,
	DFU_STATE_DNLOAD_SYNC,
	DFU_STATE_DNBUSY, 
	DFU_STATE_DNLOAD_IDLE,
	DFU_STATE_MANIFEST_SYNC,
	DFU_STATE_MANIFEST,
	DFU_STATE_MANIFEST_WAIT_RESET,
	DFU_STATE_UPLOAD_IDLE,
	DFU_STATE_ERROR,

	DFU_STATE_NUM,
} dfu_state;

static void is_dfu_build_uri(is_device_uri_list* list, struct libusb_device_descriptor* desc, libusb_device_handle* handle, is_list_devices_callback_fn callback);

static dfu_error dfu_DETACH(libusb_device_handle** dev_handle, uint8_t timeout);
static dfu_error dfu_DNLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len);
static dfu_error dfu_UPLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len);
static dfu_error dfu_GETSTATUS(libusb_device_handle** dev_handle, dfu_status* status, uint32_t *delay, dfu_state* state, uint8_t *i_string);
static dfu_error dfu_CLRSTATUS(libusb_device_handle** dev_handle);
static dfu_error dfu_GETSTATE(libusb_device_handle** dev_handle, uint8_t* buf);
static dfu_error dfu_ABORT(libusb_device_handle** dev_handle);

static dfu_error dfu_wait_for_state(libusb_device* dev, libusb_device_handle** dev_handle, dfu_state required_state);

void is_dfu_probe(is_device_uri_list* uri_list, is_list_devices_callback_fn callback)
{
	// TODO: libusb_result asserts

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

				is_dfu_build_uri(uri_list, &desc, dev_handle, callback);

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

int is_dfu_flash(const is_device_context const * context, is_dfu_config* config)
{
	int expected_size = 0;
	unsigned int transfer_size = 0;
	libusb_context *ctx;
	int num_sections = 0;
	
	ihex_image_section_t image[NUM_IHEX_SECTIONS];
	num_sections = ihex_load_sections(config->filename, image, NUM_IHEX_SECTIONS);

	int ret = libusb_init(&ctx);
	if (ret)
	{
		uinsLogError(context, ret, "unable to initialize libusb");
		return -1;
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

static void is_dfu_build_uri(is_device_uri_list* list, struct libusb_device_descriptor* desc, libusb_device_handle* handle, is_list_devices_callback_fn callback)
{
	int libusb_result;
	const size_t sn_size = IS_SN_MAX_SIZE_V5;
	const size_t url_buffer_size = sn_size + 6;
	char url_buffer[url_buffer_size];
	unsigned char serial_number[sn_size];

	libusb_result = libusb_get_string_descriptor_ascii(handle, desc->iSerialNumber, serial_number, sizeof(serial_number));
	
	snprintf(url_buffer, url_buffer_size, "dfu://%s", serial_number);
	if(callback != NULL)
	{
		callback(url_buffer);
	}

	uins_add_device(list, url_buffer);
}

static dfu_error dfu_GETSTATUS(libusb_device_handle** dev_handle, dfu_status* status, uint32_t* delay, dfu_state* state, uint8_t* i_string)
{
	uint8_t buf[6] = { 0 };

	if (libusb_control_transfer(*dev_handle, 0b10100001, 0x03, 0, 0, buf, 6, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	*status = (dfu_status)buf[0];
	*delay = (buf[3] << 16) | (buf[2] << 8) | buf[1];
	*state = (dfu_state)buf[4];
	*i_string = buf[5];				// Index of the description string. 0 = doesn't exist

	return DFU_ERROR_NONE;
}

static dfu_error dfu_CLRSTATUS(libusb_device_handle** dev_handle)
{
	if (libusb_control_transfer(*dev_handle, 0b00100001, 0x04, 0, 0, NULL, 0, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

static dfu_error dfu_GETSTATE(libusb_device_handle** dev_handle, uint8_t* buf)
{
	if (libusb_control_transfer(*dev_handle, 0b10100001, 0x05, 0, 0, buf, 1, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

static dfu_error dfu_ABORT(libusb_device_handle** dev_handle)
{
	if (libusb_control_transfer(*dev_handle, 0b00100001, 0x06, 0, 0, NULL, 0, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

static dfu_error dfu_UPLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len)
{
	if (libusb_control_transfer(*dev_handle, 0b10100001, 0x02, wValue, 0, buf, len, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

static dfu_error dfu_DNLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len)
{
	if (libusb_control_transfer(*dev_handle, 0b00100001, 0x01, wValue, 0, buf, len, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

static dfu_error dfu_DETACH(libusb_device_handle** dev_handle, uint8_t timeout)
{
	if (libusb_control_transfer(*dev_handle, 0b00100001, 0x00, timeout, 0, NULL, 0, 100) < LIBUSB_SUCCESS)
	{
		return DFU_ERROR_LIBUSB;
	}

	return DFU_ERROR_NONE;
}

static dfu_error dfu_wait_for_state(libusb_device* dev, libusb_device_handle** dev_handle, dfu_state required_state)
{
	dfu_status status;
	uint32_t waitTime = 0;
	dfu_state state;
	uint8_t stringIndex;

	uint8_t tryCounter = 0;

	dfu_error ret = dfu_GETSTATUS(dev_handle, &status, &waitTime, &state, &stringIndex);
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
