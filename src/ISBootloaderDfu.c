/**
 * @file ISBootloaderDfu.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating DFU capable devices (uINS-5)
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
    https://www.usb.org/sites/default/files/DFU_1.1.pdf
    https://www.st.com/resource/en/application_note/cd00264379-usb-dfu-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf
*/

#include "ISBootloaderDfu.h"

#include <time.h>

#define STM32_PAGE_SIZE 0x800
#define STM32_PAGE_ERROR_MASK 0x7FF

typedef enum	// Internal only, can change as needed
{
    DFU_ERROR_NONE = 0,
    DFU_ERROR_NO_DEVICE = -1,
    DFU_ERROR_LIBUSB = -2,
    DFU_ERROR_STATUS = -3,
    DFU_ERROR_INVALID_ARG = -4,
    DFU_ERROR_NO_FILE = -5,
    DFU_ERROR_TIMEOUT = -6,
} dfu_error;

typedef enum	// From DFU manual, do not change
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

typedef enum	// From DFU manual, do not change
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

static void is_dfu_add_uri(is_device_uri_list* list, struct libusb_device_descriptor* desc, libusb_device_handle* handle, is_list_devices_callback_fn callback);

static dfu_error dfu_DETACH(libusb_device_handle** dev_handle, uint8_t timeout);
static dfu_error dfu_DNLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len);
static dfu_error dfu_UPLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len);
static dfu_error dfu_GETSTATUS(libusb_device_handle** dev_handle, dfu_status* status, uint32_t *delay, dfu_state* state, uint8_t *i_string);
static dfu_error dfu_CLRSTATUS(libusb_device_handle** dev_handle);
static dfu_error dfu_GETSTATE(libusb_device_handle** dev_handle, uint8_t* buf);
static dfu_error dfu_ABORT(libusb_device_handle** dev_handle);

/**
 * @brief Program option bytes and reset device
 * 
 * @param bytes data to program
 * @param size must be equal to full length of option byte section
 * @param dev_handle handle to device
 * @return is_operation_result 
 */
is_operation_result is_dfu_write_option_bytes(
    uint8_t* bytes,
    int size, 
    libusb_device_handle* dev_handle
);

/**
 * @brief Leave DFU mode
 * @note Only works if the option bytes are set to *not* enter DFU mode after reset. 
 *  If option bytes are not set, you can leave DFU mode with `is_dfu_write_option_bytes`,
 *  which will exit DFU mode automatically.
 * @see is_dfu_write_option_bytes
 * 
 * @param dev_handle 
 * @return is_operation_result 
 */
is_operation_result is_dfu_leave(libusb_device_handle* dev_handle);

static dfu_error dfu_set_address_pointer(libusb_device_handle** dev_handle, uint32_t address);
static dfu_error dfu_wait_for_state(libusb_device_handle** dev_handle, dfu_state required_state);

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

                is_dfu_add_uri(uri_list, &desc, dev_handle, callback);

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

is_operation_result is_dfu_flash(
    const is_device_context const * context, 
    ihex_image_section_t* image,
    int image_sections,
    libusb_device_handle* dev_handle
)
{
    libusb_claim_interface(dev_handle, 0);

    // Cancel any existing operations and reset status to good
    dfu_ABORT(&dev_handle);
    dfu_wait_for_state(&dev_handle, DFU_STATE_IDLE);

    if(image[0].address == 0) {
        for(size_t i = 0; i < image_sections; i++)
        {
            image[i].address += 0x08000000;
        }
    }

    // Erase memory (only erase pages where firmware lives)
    for(size_t i = 0; i < image_sections; i++)
    {
        if(image[i].address & STM32_PAGE_ERROR_MASK)
        {
            continue;	// Page is not aligned with write location
        }

        if(image[i].image == NULL || image[i].len == 0)
        {
            continue;	// Null image
        }

        uint32_t byteInSection = 0;

        do {
            uint32_t pageAddress = byteInSection + image[i].address;
            uint8_t eraseCommand[5];

            eraseCommand[0] = 0x41;
            memcpy(&eraseCommand[1], &pageAddress, 4);

            int ret = dfu_DNLOAD(&dev_handle, 0, eraseCommand, 5);
            dfu_wait_for_state(&dev_handle, DFU_STATE_DNLOAD_IDLE);

            byteInSection += STM32_PAGE_SIZE;
        } while(byteInSection < image[i].len - 1);
    }

    // Write memory
    for(size_t i = 0; i < image_sections; i++)
    {
        dfu_set_address_pointer(&dev_handle, image[i].address);

        uint32_t byteInSection = 0;

        do {
            uint8_t payload[STM32_PAGE_SIZE] = { 0 };
            uint32_t payloadLen = STM32_PAGE_SIZE;
            uint32_t bytesRemaining = image[i].len - byteInSection;
            if (payloadLen > bytesRemaining)
            {
                payloadLen = bytesRemaining;
            }

            // Copy image into buffer for transmission
            // TODO: Should this actually be 0xFF so we don't write to unused sections?
            memset(payload, 0xFF, STM32_PAGE_SIZE);
            memcpy(payload, &image[i].image[byteInSection], payloadLen);

            uint8_t blockNum = byteInSection / STM32_PAGE_SIZE;
    
            int ret = dfu_DNLOAD(&dev_handle, blockNum + 2, payload, STM32_PAGE_SIZE);		
            dfu_wait_for_state(&dev_handle, DFU_STATE_DNLOAD_IDLE);

            byteInSection += payloadLen;
        } while (byteInSection < image[i].len - 1);
    }

    dfu_ABORT(&dev_handle);
    dfu_wait_for_state(&dev_handle, DFU_STATE_IDLE);

    uint8_t options[] = {
        0xaa,0xf8,0xff,0xfb, 0x55,0x07,0x00,0x04,
        0xff,0xff,0xff,0xff, 0x00,0x00,0x00,0x00,
        0x00,0x00,0xff,0xff, 0xff,0xff,0x00,0x00,
        0xff,0xff,0x00,0xff, 0x00,0x00,0xff,0x00,
        0xff,0xff,0x00,0xff, 0x00,0x00,0xff,0x00
    };

    is_dfu_write_option_bytes(options, sizeof(options), (libusb_device_handle*)dev_handle);

    libusb_release_interface(dev_handle, 0);

    return IS_OP_OK;
}

static void is_dfu_add_uri(is_device_uri_list* list, struct libusb_device_descriptor* desc, libusb_device_handle* handle, is_list_devices_callback_fn callback)
{
    int libusb_result;
    const size_t sn_size = IS_SN_MAX_SIZE_V5;
    const size_t url_buffer_size = sn_size + 16;
    char url_buffer[url_buffer_size];
    unsigned char serial_number[sn_size];

    // Get the string containing the serial number from the device
    libusb_result = libusb_get_string_descriptor_ascii(handle, desc->iSerialNumber, serial_number, sizeof(serial_number));
    
    snprintf(url_buffer, url_buffer_size, "dfu://%s/%04X/%04X", serial_number, desc->idVendor, desc->idProduct);
    if(callback != NULL)
    {
        callback(url_buffer);
    }

    is_add_device(list, url_buffer);
}

static is_operation_result is_dfu_write_option_bytes(
    uint8_t* bytes,
    int size, 
    libusb_device_handle* dev_handle
)
{
    // Cancel any existing operations and reset status to good
    dfu_ABORT(&dev_handle);
    dfu_wait_for_state(&dev_handle, DFU_STATE_IDLE);

    int ret_alt = libusb_set_interface_alt_setting(dev_handle, 0, UINS5_DFU_INTERFACE_OPTIONS);

    dfu_error set_ret = dfu_set_address_pointer(&dev_handle, 0x1FFF7800);

    int ret = dfu_DNLOAD(&dev_handle, 2, bytes, size);
    dfu_wait_for_state(&dev_handle, DFU_STATE_DNLOAD_IDLE);	

    // Device will reset automatically here

    return IS_OP_OK;
}

static is_operation_result is_dfu_leave(libusb_device_handle* dev_handle)
{
    // Cancel any existing operations and reset status to good
    dfu_ABORT(&dev_handle);
    dfu_wait_for_state(&dev_handle, DFU_STATE_IDLE);

    dfu_error set_ret = dfu_set_address_pointer(&dev_handle, 0x08000000);

    int ret = dfu_DNLOAD(&dev_handle, 0, NULL, 0);
    if(dfu_wait_for_state(&dev_handle, DFU_STATE_MANIFEST) != DFU_ERROR_NONE) return IS_OP_ERROR;

    ret = libusb_reset_device(dev_handle);

    return IS_OP_OK;
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

static dfu_error dfu_set_address_pointer(libusb_device_handle** dev_handle, uint32_t address)
{
    unsigned char data[5] = { 0 };
	data[0] = 0x21;
	memcpy(&data[1], &address, 4);

	dfu_DNLOAD(dev_handle, 0, data, 5);
    return dfu_wait_for_state(dev_handle, DFU_STATE_DNLOAD_IDLE);
}

static dfu_error dfu_wait_for_state(libusb_device_handle** dev_handle, dfu_state required_state)
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

        const struct timespec delay_ts = { 0L, waitTime * 1000L * 1000L };
		nanosleep(&delay_ts, NULL); // TODO: Windows

        ret = dfu_GETSTATUS(dev_handle, &status, &waitTime, &state, &stringIndex);
        if (ret != DFU_ERROR_NONE) return ret;

        tryCounter++;
        if (tryCounter > 4) return DFU_ERROR_TIMEOUT;
    }

    return DFU_ERROR_NONE;
}
