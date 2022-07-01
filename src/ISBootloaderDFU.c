/**
 * @file ISBootloaderDFU.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense routines for updating ISB (Inertial Sense Bootloader)
 *  images using the DFU protocol.
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// Resources for DFU protocol:
//  - https://www.usb.org/sites/default/files/DFU_1.1.pdf
//  - https://www.st.com/resource/en/application_note/cd00264379-usb-dfu-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf


#include "ISUtilities.h"
#include "ISBootloaderDFU.h"
#include "serialPortPlatform.h"
#include "ISBootloaderTypes.h"

#include <time.h>

#define DFU_STATUS(x, level) ctx->info_callback(ctx, x, level)

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

static is_operation_result is_dfu_get_sn(libusb_device_handle** handle, uint32_t* sn);

typedef enum
{
    STM32_DFU_INTERFACE_FLASH    = 0, // @Internal Flash  /0x08000000/0256*0002Kg
    STM32_DFU_INTERFACE_OPTIONS  = 1, // @Option Bytes  /0x1FFF7800/01*040 e
    STM32_DFU_INTERFACE_OTP      = 2, // @OTP Memory /0x1FFF7000/01*0001Ke
    STM32_DFU_INTERFACE_FEATURES = 3  // @Device Feature/0xFFFF0000/01*004 e
} is_stm32l4_dfu_interface_alternatives;

is_operation_result is_dfu_list_devices(is_dfu_list* list)
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

        // Check vendor and product ID
        if (desc.idVendor != STM32_DESCRIPTOR_VENDOR_ID) continue;      // must be some other usb device
        if (desc.idProduct != STM32_DESCRIPTOR_PRODUCT_ID) continue;     // must be some other usb device

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
        unsigned char uid[IS_DFU_UID_MAX_SIZE];
        ret_libusb = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, uid, sizeof(uid));
        if(ret_libusb < LIBUSB_SUCCESS) uid[0] = '\0'; // Set the serial number as none
        
        // Add to list
        is_dfu_get_sn(&dev_handle, &list->id[list->present].sn);
        strncpy(list->id[list->present].uid, (char*)uid, IS_DFU_UID_MAX_SIZE);
        list->id[list->present].vid = desc.idVendor;
        list->id[list->present].pid = desc.idProduct;

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

typedef struct
{
	/** Inertial Sense serial number */
	uint32_t		serialNumber;

	/** Inertial Sense lot number */
	uint32_t		lotNumber;

	/** Inertial Sense manufacturing date (YYYYMMDDHHMMSS) */
    char			date[16];
} is_dfu_otp_id_t;

#define OTP_SECTION_SIZE	64		// 64 bytes. DO NOT CHANGE.
#define OTP_NUM_SECTIONS    16      // 16 attempts. DO NOT CHANGE.
#define OTP_KEY				0xBAADBEEFB0BABABE		// DO NOT CHANGE

/**
 * @brief Get the Inertial Sense serial number from flash (if not present, get DFU serial number)
 * 
 * @param ctx device context
 * @return is_operation_result 
 */
static is_operation_result is_dfu_get_sn(libusb_device_handle** handle, uint32_t* sn)
{
    dfu_status status;
    uint32_t waitTime = 0;
    dfu_state state;
    uint8_t stringIdx;

    int ret_libusb;

    uint8_t rxBuf[1024] = {0};

    // Get the 1K OTP section from the chip
    // 0x1FFF7000 is the address. Little endian.
    {
        // Clear status back to good
        dfu_CLRSTATUS(handle);
        ret_libusb = dfu_GETSTATUS(handle, &status, &waitTime, &state, &stringIdx);

        // Set the address pointer (command is 0x21)
        uint8_t txBuf[] = { 0x21, 0x00, 0x70, 0xFF, 0x1F };
        ret_libusb = dfu_DNLOAD(handle, 0, txBuf, sizeof(txBuf));
        if(ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;
        
        // Address pointer takes effect after GETSTATUS command
        ret_libusb = dfu_GETSTATUS(handle, &status, &waitTime, &state, &stringIdx);
        if(ret_libusb < LIBUSB_SUCCESS || status != DFU_STATUS_OK || state != DFU_STATE_DNBUSY) return IS_OP_ERROR;
        ret_libusb = dfu_GETSTATUS(handle, &status, &waitTime, &state, &stringIdx);
        if(ret_libusb < LIBUSB_SUCCESS || status != DFU_STATUS_OK) return IS_OP_ERROR;

        // Get out of download mode
        dfu_ABORT(handle);

        // Read the full OTP page
        ret_libusb = dfu_UPLOAD(handle, 2, rxBuf, 1024);
        if(ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR;
    }

    int index = 0;
	uint8_t* otp_mem = (uint8_t*)rxBuf; 
	
    // Look for the first section of zeroes
	uint8_t cmp[OTP_SECTION_SIZE];
	memset(cmp, 0xFF, OTP_SECTION_SIZE);
    bool foundSn = true;
	while(memcmp(cmp, otp_mem, OTP_SECTION_SIZE) != 0)
	{
		otp_mem += OTP_SECTION_SIZE; index++;
		if(index >= OTP_NUM_SECTIONS)
		{
            foundSn = false; break;	// No more room in OTP
		}
	}

    // Go back one, to the last filled section
	index--;
    is_dfu_otp_id_t* id = (is_dfu_otp_id_t*)((index * OTP_SECTION_SIZE) + rxBuf);

	uint64_t key = OTP_KEY;
	if(memcmp(otp_mem - 8, &key, 8) == 0 && foundSn)
	{
        *sn = id->serialNumber;
        return IS_OP_OK;
    }

    return IS_OP_ERROR;
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

is_operation_result is_dfu_flash(is_device_context* ctx)
{
    int ret_libusb;
    dfu_error ret_dfu;
    ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
    size_t image_sections;
    is_operation_result ret_is;
    libusb_device** device_list;
    libusb_device_handle* dev_handle = NULL;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor* cfg;

    size_t device_count = libusb_get_device_list(NULL, &device_list);

    // Obtain a device handle
    bool dev_found = false;
    for(size_t i = 0; i < device_count; ++i) 
    {
        ret_libusb = libusb_get_device_descriptor(device_list[i], &desc);
        if (ret_libusb < 0) continue;

        if(desc.idVendor != ctx->handle.dfu.vid) continue;
        if(desc.idProduct != ctx->handle.dfu.pid) continue;

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

        // Get the string containing the uid from the device
        unsigned char uid[IS_DFU_UID_MAX_SIZE];
        ret_libusb = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, uid, sizeof(uid));
        if(ret_libusb < LIBUSB_SUCCESS) uid[0] = '\0'; // Set the serial number as none
        if(strcmp(ctx->handle.dfu.uid, (char*)uid) != 0 && strlen(ctx->handle.dfu.uid) != 0) 
        {
            libusb_close(dev_handle);
            continue;
        }

        // Check the serial number 
        uint32_t sn = 0;
        is_dfu_get_sn(&dev_handle, &sn);
        if(sn != ctx->handle.dfu.sn && ctx->handle.dfu.sn != 0)
        {
            libusb_close(dev_handle);
            continue;
        }
        
        ctx->handle.libusb = dev_handle;
        ctx->handle.status = IS_HANDLE_TYPE_LIBUSB;
        dev_found = true;
        break;
    }

    libusb_free_device_list(device_list, 1);
    if(!dev_found || !dev_handle) { return IS_OP_ERROR; } 

    DFU_STATUS("Found DFU device, starting firmware update", IS_LOG_LEVEL_INFO);

    ret_libusb = libusb_claim_interface(ctx->handle.libusb, 0);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_close(dev_handle); return IS_OP_ERROR; } 

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&ctx->handle.libusb);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_close(dev_handle); return IS_OP_ERROR; } 
    
    // Reset status to good
    ret_dfu = dfu_wait_for_state(&ctx->handle.libusb, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_close(dev_handle); return IS_OP_ERROR; } 

    // Load the firmware image
    image_sections = ihex_load_sections(ctx->firmware_path, image, MAX_NUM_IHEX_SECTIONS);
    if(image_sections <= 0) { libusb_close(dev_handle); return IS_OP_ERROR; } 

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

    DFU_STATUS("Erasing flash...", IS_LOG_LEVEL_INFO);

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

            ret_libusb = dfu_DNLOAD(&ctx->handle.libusb, 0, eraseCommand, 5);
            // if (ret_libusb < LIBUSB_SUCCESS) 
            // {
            //     ihex_unload_sections(image, image_sections);
            //     return IS_OP_ERROR;  
            // }

            ret_dfu = dfu_wait_for_state(&ctx->handle.libusb, DFU_STATE_DNLOAD_IDLE);
            // if (ret_dfu < DFU_ERROR_NONE) 
            // {
            //     ihex_unload_sections(image, image_sections);
            //     return IS_OP_ERROR;  
            // }

            byteInSection += STM32_PAGE_SIZE;
            bytes_written_total += STM32_PAGE_SIZE;

            ctx->update_progress = 0.25f * ((float)bytes_written_total / (float)image_total_len);
            ctx->update_callback(ctx, ctx->update_progress);
        } while(byteInSection < image[i].len - 1);
    }

    bytes_written_total = 0;

    DFU_STATUS("Programming flash...", IS_LOG_LEVEL_INFO);

    // Write memory
    for(size_t i = 0; i < image_sections; i++)
    {
        ret_dfu = dfu_set_address_pointer(&ctx->handle.libusb, image[i].address);
        if (ret_dfu < DFU_ERROR_NONE) 
        {
            ihex_unload_sections(image, image_sections);
            libusb_close(dev_handle); 
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
    
            ret_libusb = dfu_DNLOAD(&ctx->handle.libusb, blockNum + 2, payload, STM32_PAGE_SIZE);
            if (ret_libusb < LIBUSB_SUCCESS) 
            {
                ihex_unload_sections(image, image_sections);
                libusb_close(dev_handle); 
                return IS_OP_ERROR;  
            }

            ret_dfu = dfu_wait_for_state(&ctx->handle.libusb, DFU_STATE_DNLOAD_IDLE);
            if (ret_dfu < DFU_ERROR_NONE) 
            {
                ihex_unload_sections(image, image_sections);
                libusb_close(dev_handle); 
                return IS_OP_ERROR;  
            }

            byteInSection += payloadLen;
            bytes_written_total += payloadLen;

            ctx->update_progress = 0.25f + 0.75f * ((float)bytes_written_total / (float)image_total_len);
            ctx->update_callback(ctx, ctx->update_progress);
        } while (byteInSection < image[i].len - 1);
    }

    // Unload the firmware image
    ihex_unload_sections(image, image_sections);

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&ctx->handle.libusb);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_close(dev_handle); return IS_OP_ERROR; }    
    
    // Reset status to good
    ret_dfu = dfu_wait_for_state(&ctx->handle.libusb, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_close(dev_handle); return IS_OP_ERROR; }

    DFU_STATUS("Restarting device...", IS_LOG_LEVEL_INFO);

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

    ret_is = is_dfu_write_option_bytes(options, sizeof(options), ctx->handle.libusb);

    libusb_release_interface(ctx->handle.libusb, 0);
    libusb_close(dev_handle);

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
