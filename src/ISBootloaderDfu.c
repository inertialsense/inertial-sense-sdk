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

#include "ISUtilities.h"
#include "ISBootloaderDfu.h"
#include "inertialSenseBootLoader.h"
#include "serialPortPlatform.h"

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

static is_operation_result is_dfu_write_option_bytes(
    uint8_t* bytes,
    int size, 
    libusb_device_handle* dev_handle
);

static dfu_error dfu_DETACH(libusb_device_handle** dev_handle, uint8_t timeout);
static dfu_error dfu_DNLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len);
static dfu_error dfu_UPLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len);
static dfu_error dfu_GETSTATUS(libusb_device_handle** dev_handle, dfu_status* status, uint32_t *delay, dfu_state* state, uint8_t *i_string);
static dfu_error dfu_CLRSTATUS(libusb_device_handle** dev_handle);
static dfu_error dfu_GETSTATE(libusb_device_handle** dev_handle, uint8_t* buf);
static dfu_error dfu_ABORT(libusb_device_handle** dev_handle);

is_operation_result is_list_dfu(
    is_dfu_list* list
)
{
    list->present = 0;

    libusb_device** device_list;
    libusb_device* dev;
    libusb_device_handle* dev_handle;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor* cfg;
    int ret_libusb;

    ret_libusb = libusb_init(NULL);
    if(ret_libusb < 0) return IS_OP_ERROR;

    size_t device_count = libusb_get_device_list(NULL, &device_list);

    for (size_t i = 0; i < device_count; ++i) {
        dev = device_list[i];

        ret_libusb = libusb_get_device_descriptor(dev, &desc);
        if(ret_libusb < 0) continue;

        // Check vendor and product IDs against list
        size_t j;
        size_t match_list_len = sizeof(dfu_matches)/sizeof(is_device_vid_pid);
        for(j = 0; j < match_list_len; j++)
        {
            if (desc.idVendor != dfu_matches[j].vid) continue;      // must be some other usb device
            if (desc.idProduct != dfu_matches[j].pid) continue;     // must be some other usb device
            break;  // We found a device
        }
        if(j >= match_list_len) continue;    // We didn't find a match

        ret_libusb = libusb_get_config_descriptor(device_list[i], 0, &cfg);

        // USB-IF DFU interface class numbers
        if(cfg->interface->altsetting[0].bInterfaceClass != 0xFE || 
            cfg->interface->altsetting[0].bInterfaceSubClass != 0x01 || 
            cfg->interface->altsetting[0].bInterfaceProtocol != 0x02
        ) continue;

        // Open the device
        ret_libusb = libusb_open(dev, &dev_handle);
        if (ret_libusb < LIBUSB_SUCCESS) continue;

        // Reset the device
        ret_libusb = libusb_reset_device(dev_handle);
        if(ret_libusb < LIBUSB_SUCCESS) 
        {
            libusb_close(dev_handle);
            continue; 
        }

        // Get the string containing the serial number from the device
        unsigned char serial_number[IS_SN_MAX_SIZE];
        ret_libusb = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, serial_number, sizeof(serial_number));
        if(ret_libusb < LIBUSB_SUCCESS) serial_number[0] = '\0'; // Set the serial number as none
        
        // Add to list
        strncpy(list->id[list->present].sn, (char*)serial_number, IS_SN_MAX_SIZE);
        list->id[list->present].usb.vid = desc.idVendor;
        list->id[list->present].usb.pid = desc.idProduct;

        list->present++;
        if(list->present >= IS_DFU_LIST_LEN)
        {
            libusb_close(dev_handle);
            libusb_free_device_list(device_list, 1);
            libusb_exit(NULL);
            return IS_OP_OK;
        }

        libusb_close(dev_handle);
    }

    libusb_free_device_list(device_list, 1);
    libusb_exit(NULL);

    return IS_OP_OK;
}

is_operation_result is_init_dfu_context(is_device_context* ctx)
{
    ctx->scheme = IS_SCHEME_DFU;
    ctx->match_props.match = 
        IS_DEVICE_MATCH_FLAG_VID | 
        IS_DEVICE_MATCH_FLAG_PID | 
        IS_DEVICE_MATCH_FLAG_TYPE | 
        IS_DEVICE_MATCH_FLAG_MAJOR;

    strncpy(ctx->bl_enable_command, "BLEN", 5);

    if(strlen(ctx->match_props.serial_number) != 0)
    {
        ctx->match_props.match |= IS_DEVICE_MATCH_FLAG_SN;
    }

    return IS_OP_OK; // Didn't find a device
}

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
static is_operation_result is_dfu_leave(libusb_device_handle* dev_handle);

static dfu_error dfu_set_address_pointer(libusb_device_handle** dev_handle, uint32_t address);
static dfu_error dfu_wait_for_state(libusb_device_handle** dev_handle, dfu_state required_state);

is_operation_result is_dfu_flash(is_device_context* context)
{
    int ret_libusb;
    dfu_error ret_dfu;
    ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
    size_t image_sections;
    is_operation_result ret_is;
    libusb_device** device_list;
    libusb_device_handle* dev_handle;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor* cfg;

    size_t device_count = libusb_get_device_list(NULL, &device_list);

    // Obtain a device handle
    bool dev_found = false;
    for(size_t i = 0; i < device_count; ++i) 
    {
        ret_libusb = libusb_get_device_descriptor(device_list[i], &desc);
        if (ret_libusb < 0) continue;

        if(desc.idVendor != context->match_props.vid) continue;
        if(desc.idProduct != context->match_props.pid) continue;

        ret_libusb = libusb_get_config_descriptor(device_list[i], 0, &cfg);
        if (ret_libusb < 0) continue;

        // USB-IF DFU interface class numbers
        if(cfg->interface->altsetting[0].bInterfaceClass != 0xFE || 
            cfg->interface->altsetting[0].bInterfaceSubClass != 0x01 || 
            cfg->interface->altsetting[0].bInterfaceProtocol != 0x02
        ) continue;

        // Open the device
        ret_libusb = libusb_open(device_list[i], &dev_handle);
        if (ret_libusb < 0) continue;

        // Reset the device
        ret_libusb = libusb_reset_device(dev_handle);
        if(ret_libusb < LIBUSB_SUCCESS) 
        {
            libusb_close(dev_handle);
            continue; 
        }

        // Get the string containing the serial number from the device
        unsigned char serial_number[IS_SN_MAX_SIZE];
        ret_libusb = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, serial_number, sizeof(serial_number));
        if(ret_libusb < LIBUSB_SUCCESS) serial_number[0] = '\0'; // Set the serial number as none

        // Check the serial number
        if((context->match_props.match & IS_DEVICE_MATCH_FLAG_SN) && (strcmp(context->match_props.serial_number, (char*)serial_number) != 0)) 
        {
            libusb_close(dev_handle);
            continue;
        }
        
        context->handle.libusb = dev_handle;
        context->handle.status = IS_HANDLE_TYPE_LIBUSB;
        dev_found = true;
    }

    libusb_free_device_list(device_list, 1);
    if(!dev_found) return IS_OP_ERROR;

    ret_libusb = libusb_claim_interface(context->handle.libusb, 0);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR; 

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&context->handle.libusb);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;    
    
    // Reset status to good
    ret_dfu = dfu_wait_for_state(&context->handle.libusb, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) return IS_OP_ERROR;

    // Load the firmware image
    image_sections = ihex_load_sections(context->firmware.firmware_path, image, MAX_NUM_IHEX_SECTIONS);
    if(image_sections <= 0) return IS_OP_ERROR;

    int image_total_len = 0;
    for(size_t i = 0; i < image_sections; i++)
    {
        image_total_len += image[i].len;
    }

    // If starting address is 0, set it to 0x08000000 (start of flash memory)
    if(image[0].address == 0x00000000) 
    {
        for(size_t i = 0; i < image_sections; i++)
        {
            image[i].address += 0x08000000;
        }
    }

    uint32_t bytes_written_total = 0;

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

            ret_libusb = dfu_DNLOAD(&context->handle.libusb, 0, eraseCommand, 5);
            // if (ret_libusb < LIBUSB_SUCCESS) 
            // {
            //     ihex_unload_sections(image, image_sections);
            //     return IS_OP_ERROR;  
            // }

            ret_dfu = dfu_wait_for_state(&context->handle.libusb, DFU_STATE_DNLOAD_IDLE);
            // if (ret_dfu < DFU_ERROR_NONE) 
            // {
            //     ihex_unload_sections(image, image_sections);
            //     return IS_OP_ERROR;  
            // }

            byteInSection += STM32_PAGE_SIZE;
            bytes_written_total += STM32_PAGE_SIZE;

            context->update_progress_callback(context, 0.25f * ((float)bytes_written_total / (float)image_total_len));
        } while(byteInSection < image[i].len - 1);
    }

    bytes_written_total = 0;

    // Write memory
    for(size_t i = 0; i < image_sections; i++)
    {
        ret_dfu = dfu_set_address_pointer(&context->handle.libusb, image[i].address);
        if (ret_dfu < DFU_ERROR_NONE) 
        {
            ihex_unload_sections(image, image_sections);
            return IS_OP_ERROR;  
        }

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
            memset(payload, 0xFF, STM32_PAGE_SIZE);
            memcpy(payload, &image[i].image[byteInSection], payloadLen);

            uint8_t blockNum = byteInSection / STM32_PAGE_SIZE;
    
            ret_libusb = dfu_DNLOAD(&context->handle.libusb, blockNum + 2, payload, STM32_PAGE_SIZE);
            if (ret_libusb < LIBUSB_SUCCESS) 
            {
                ihex_unload_sections(image, image_sections);
                return IS_OP_ERROR;  
            }

            ret_dfu = dfu_wait_for_state(&context->handle.libusb, DFU_STATE_DNLOAD_IDLE);
            if (ret_dfu < DFU_ERROR_NONE) 
            {
                ihex_unload_sections(image, image_sections);
                return IS_OP_ERROR;  
            }

            byteInSection += payloadLen;
            bytes_written_total += payloadLen;

            context->update_progress_callback(context, 0.25f + 0.75f * ((float)bytes_written_total / (float)image_total_len));
        } while (byteInSection < image[i].len - 1);
    }

    // Unload the firmware image
    ihex_unload_sections(image, image_sections);

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&context->handle.libusb);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;    
    
    // Reset status to good
    ret_dfu = dfu_wait_for_state(&context->handle.libusb, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) return IS_OP_ERROR;

    // Option bytes
    // This hard-coded array sets mostly defaults, but without PH3 enabled and
    // with DFU mode disabled. Application will enable DFU mode if needed.
    uint8_t options[] = {
        0xaa,0xf8,0xff,0xfb, 0x55,0x07,0x00,0x04,
        0xff,0xff,0xff,0xff, 0x00,0x00,0x00,0x00,
        0x00,0x00,0xff,0xff, 0xff,0xff,0x00,0x00,
        0xff,0xff,0x00,0xff, 0x00,0x00,0xff,0x00,
        0xff,0xff,0x00,0xff, 0x00,0x00,0xff,0x00
    };

    ret_is = is_dfu_write_option_bytes(options, sizeof(options), context->handle.libusb);

    ret_libusb = libusb_release_interface(context->handle.libusb, 0);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;    

    return ret_is;
}

static is_operation_result is_dfu_write_option_bytes(
    uint8_t* bytes,
    int size, 
    libusb_device_handle* dev_handle
)
{
    int ret_libusb;
    dfu_error ret_dfu;

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&dev_handle);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;   

    // Reset status to good
    ret_dfu = dfu_wait_for_state(&dev_handle, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) return IS_OP_ERROR;

    // Select the alt setting for option bytes
    ret_libusb = libusb_set_interface_alt_setting(dev_handle, 0, STM32_DFU_INTERFACE_OPTIONS);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;  

    ret_dfu = dfu_set_address_pointer(&dev_handle, 0x1FFF7800);
    if (ret_dfu < DFU_ERROR_NONE) return IS_OP_ERROR;

    ret_libusb = dfu_DNLOAD(&dev_handle, 2, bytes, size);
    dfu_wait_for_state(&dev_handle, DFU_STATE_DNLOAD_IDLE);	

    // ret_libusb = libusb_reset_device(dev_handle);
    // if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR; 

    return IS_OP_OK;
}

static is_operation_result is_dfu_leave(libusb_device_handle* dev_handle)
{
    int ret_libusb;
    dfu_error ret_dfu;

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&dev_handle);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;    
    
    // Reset status to good
    ret_dfu = dfu_wait_for_state(&dev_handle, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) return IS_OP_ERROR;

    // Set address pointer to flash
    ret_dfu = dfu_set_address_pointer(&dev_handle, 0x08000000);
    if (ret_dfu < DFU_ERROR_NONE) return IS_OP_ERROR;

    // Request DFU leave
    ret_libusb = dfu_DNLOAD(&dev_handle, 0, NULL, 0);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;    

    // Execute DFU leave
    ret_dfu = dfu_wait_for_state(&dev_handle, DFU_STATE_MANIFEST);
    if (ret_dfu < DFU_ERROR_NONE) return IS_OP_ERROR; 

    // Reset USB device
    ret_libusb = libusb_reset_device(dev_handle);
    if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR; 

    return IS_OP_OK;
}

static int dfu_GETSTATUS(libusb_device_handle** dev_handle, dfu_status* status, uint32_t* delay, dfu_state* state, uint8_t* i_string)
{
    int ret_libusb;
    uint8_t buf[6] = { 0 };

    ret_libusb = libusb_control_transfer(*dev_handle, 0b10100001, 0x03, 0, 0, buf, 6, 100);

    *status = (dfu_status)buf[0];
    *delay = (buf[3] << 16) | (buf[2] << 8) | buf[1];
    *state = (dfu_state)buf[4];
    *i_string = buf[5];

    return ret_libusb;
}

static int dfu_CLRSTATUS(libusb_device_handle** dev_handle)
{
    return libusb_control_transfer(*dev_handle, 0b00100001, 0x04, 0, 0, NULL, 0, 100);
}

static int dfu_GETSTATE(libusb_device_handle** dev_handle, uint8_t* buf)
{
    return libusb_control_transfer(*dev_handle, 0b10100001, 0x05, 0, 0, buf, 1, 100);
}

static int dfu_ABORT(libusb_device_handle** dev_handle)
{
    return libusb_control_transfer(*dev_handle, 0b00100001, 0x06, 0, 0, NULL, 0, 100);
}

static int dfu_UPLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len)
{
    return libusb_control_transfer(*dev_handle, 0b10100001, 0x02, wValue, 0, buf, len, 100);
}

static int dfu_DNLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len)
{
    return libusb_control_transfer(*dev_handle, 0b00100001, 0x01, wValue, 0, buf, len, 100);
}

static int dfu_DETACH(libusb_device_handle** dev_handle, uint8_t timeout)
{
    return libusb_control_transfer(*dev_handle, 0b00100001, 0x00, timeout, 0, NULL, 0, 100);
}

static dfu_error dfu_set_address_pointer(libusb_device_handle** dev_handle, uint32_t address)
{
    int ret_libusb;
    unsigned char data[5] = { 0 };
	data[0] = 0x21;
	memcpy(&data[1], &address, 4);

    ret_libusb = dfu_DNLOAD(dev_handle, 0, data, 5);
	if(ret_libusb < LIBUSB_SUCCESS) return DFU_ERROR_LIBUSB;

    return dfu_wait_for_state(dev_handle, DFU_STATE_DNLOAD_IDLE);
}

static dfu_error dfu_wait_for_state(libusb_device_handle** dev_handle, dfu_state required_state)
{
    int ret_libusb;

    dfu_status status;
    uint32_t waitTime = 0;
    dfu_state state;
    uint8_t stringIndex;

    uint8_t tryCounter = 0;

    ret_libusb = dfu_GETSTATUS(dev_handle, &status, &waitTime, &state, &stringIndex);
    if (ret_libusb < LIBUSB_SUCCESS) 
    {
        return DFU_ERROR_LIBUSB;
    }

    while (status != DFU_STATUS_OK || state != required_state)
    {
        if (status != DFU_STATUS_OK)
        { 
            ret_libusb = dfu_CLRSTATUS(dev_handle);
            if (ret_libusb < LIBUSB_SUCCESS) return DFU_ERROR_LIBUSB;
        }

		SLEEP_MS(_MAX(waitTime, 10)); // TODO: Windows

        ret_libusb = dfu_GETSTATUS(dev_handle, &status, &waitTime, &state, &stringIndex);
        if (ret_libusb < LIBUSB_SUCCESS) 
        {
            return DFU_ERROR_LIBUSB;
        }

        tryCounter++;
        if (tryCounter > 4) return DFU_ERROR_TIMEOUT;
    }

    return DFU_ERROR_NONE;
}
