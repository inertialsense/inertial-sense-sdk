/**
 * @file ISBootloaderDFU.cpp
 * @author Dave Cutting
 * @brief Inertial Sense bootloader routines for DFU devices
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderDFU.h"
#include "ihex.h"
#include "ISUtilities.h"

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdbool.h> 

using namespace ISBootloader;

std::mutex cISBootloaderDFU::m_DFUmutex;

static constexpr uint32_t STM32_PAGE_SIZE = 0x800;
static constexpr uint32_t STM32_PAGE_ERROR_MASK = 0x7FF;

/**
 * @brief OTP section 
 * 
 */
typedef struct
{
	/** Inertial Sense serial number */
	uint32_t		serialNumber;

	/** Inertial Sense lot number */
	uint16_t		lotNumber;

    /** Inertial Sense Hardware/Product ID (UINS, IMX, GPX, VPX, etc) */
    uint16_t        hardwareId;

	/** Inertial Sense manufacturing date (YYYYMMDDHHMMSS) */
    char			date[16];

    /** Platform / carrier board (ePlatformCfg::PLATFORM_CFG_TYPE_MASK).  Only valid if greater than zero. */
    int32_t		    platformType;

    /** Disabled Feature Bits - a bit mask which (when set to 1) will disable certain product-specific features. */
    // NOTE: feature bits are cummulative, in that once a bit is set, IT CAN NOT EVER BE UNSET
    int32_t         reservedBits;
} is_dfu_otp_id_t;

static constexpr uint32_t OTP_SECTION_SIZE = 64;		// 64 bytes. DO NOT CHANGE.
static constexpr uint32_t OTP_NUM_SECTIONS = 16;        // 16 attempts. DO NOT CHANGE.
static constexpr uint64_t OTP_KEY = 0xBAADBEEFB0BABABE;	// DO NOT CHANGE

is_operation_result cISBootloaderDFU::match_test(void* param)
{
    const char* uid = (const char*)param;

    if(strnlen(uid, 20) != 0 && strnlen(m_dfu.uid, 20) != 0 && strncmp(uid, m_dfu.uid, 20) == 0)
    {
        return IS_OP_OK;
    }

    return IS_OP_ERROR;
}

int cISBootloaderDFU::get_num_devices()
{
    int present = 0;

    libusb_device** device_list;
    libusb_device* dev;
    //libusb_device_handle* dev_handle;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor* cfg;
    int ret_libusb;

    if(m_DFUmutex.try_lock())
    {
        libusb_init(NULL);

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

            present++;
        }

        libusb_free_device_list(device_list, 1);

        libusb_exit(NULL);

        m_DFUmutex.unlock();
    }

    return present;
}

is_operation_result cISBootloaderDFU::list_devices(is_dfu_list* list)
{
    list->present = 0;

    libusb_device** device_list;
    libusb_device* dev;
    libusb_device_handle* dev_handle;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor* cfg;
    int ret_libusb;

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

        // Add to list
        std::string uidstr;
        get_serial_number_libusb(&dev_handle, list->id[list->present].sn, uidstr, desc.iSerialNumber);
        strncpy(list->id[list->present].uid, (char*)uidstr.c_str(), IS_DFU_UID_MAX_SIZE);
        list->id[list->present].vid = desc.idVendor;
        list->id[list->present].pid = desc.idProduct;
        list->id[list->present].handle_libusb = dev_handle;
        list->id[list->present].iSerialNumber = desc.iSerialNumber;

        list->present++;
        if(list->present >= IS_DFU_LIST_LEN)
        {
            libusb_free_device_list(device_list, 1);
            return IS_OP_OK;
        }
    }

    libusb_free_device_list(device_list, 1);

    return IS_OP_OK;
}

eImageSignature cISBootloaderDFU::check_is_compatible()
{
    return IS_IMAGE_SIGN_DFU;
}

is_operation_result cISBootloaderDFU::get_serial_number_libusb(libusb_device_handle** handle, uint32_t& sn, std::string& uidstr, uint8_t sn_idx)
{
    dfu_status status;
    uint32_t waitTime = 0;
    dfu_state state;
    uint8_t stringIdx;
    dfu_error ret_dfu;

    int ret_libusb;

    uint8_t rxBuf[1024] = {0};

    SLEEP_MS(100);

    ret_libusb = libusb_claim_interface(*handle, 0);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(*handle, 0); return IS_OP_ERROR; }

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(handle);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(*handle, 0); return IS_OP_ERROR; }

    // Reset status to good
    ret_dfu = dfu_wait_for_state(handle, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(*handle, 0); return IS_OP_ERROR; }

    // Get the string containing the serial number from the device
    unsigned char uid[IS_DFU_UID_MAX_SIZE];
    ret_libusb = libusb_get_string_descriptor_ascii(*handle, sn_idx, uid, sizeof(uid));
    if(ret_libusb < LIBUSB_SUCCESS) uid[0] = '\0'; // Set the serial number as none

    uidstr = std::string((const char*)uid);

    // TODO make this IMX/GPX aware (OTP location maybe processor and/or product/device dependent)

    // Get the 1K OTP section from the chip
    // 0x1FFF7000 is the address. Little endian.
    {
        // Set the address pointer (command is 0x21)
        uint8_t txBuf[] = { 0x21, 0x00, 0x70, 0xFF, 0x1F };
        ret_libusb = dfu_DNLOAD(handle, 0, txBuf, sizeof(txBuf));
        if(ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(*handle, 0); return IS_OP_ERROR; }
        
        // Address pointer takes effect after GETSTATUS command
        ret_libusb = dfu_GETSTATUS(handle, &status, &waitTime, &state, &stringIdx);
        if(ret_libusb < LIBUSB_SUCCESS || status != DFU_STATUS_OK || state != DFU_STATE_DNBUSY) { libusb_release_interface(*handle, 0); return IS_OP_ERROR; }
        ret_libusb = dfu_GETSTATUS(handle, &status, &waitTime, &state, &stringIdx);
        if(ret_libusb < LIBUSB_SUCCESS || status != DFU_STATUS_OK) { libusb_release_interface(*handle, 0); return IS_OP_ERROR; }

        // Get out of download mode
        dfu_ABORT(handle);

        // Read the full OTP page
        ret_libusb = dfu_UPLOAD(handle, 2, rxBuf, 1024);
        if(ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(*handle, 0); return IS_OP_ERROR; }
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
		if(index >= (int)OTP_NUM_SECTIONS)
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
        sn = id->serialNumber;

        libusb_release_interface(*handle, 0);
        return IS_OP_OK;
    }

    libusb_release_interface(*handle, 0);
    return IS_OP_ERROR;
}

uint32_t cISBootloaderDFU::get_device_info()
{
    get_serial_number_libusb(&m_dfu.handle_libusb, m_sn, m_port_name, m_dfu.iSerialNumber);

    return m_sn;
}


is_operation_result cISBootloaderDFU::download_image(std::string filename)
{
    int ret_libusb;
    dfu_error ret_dfu;
    ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
    size_t image_sections;

    SLEEP_MS(100);

    ret_libusb = libusb_claim_interface(m_dfu.handle_libusb, 0);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&m_dfu.handle_libusb);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }
    
    // Reset status to good
    ret_dfu = dfu_wait_for_state(&m_dfu.handle_libusb, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    // Load the firmware image
    image_sections = ihex_load_sections(filename.c_str(), image, MAX_NUM_IHEX_SECTIONS);
    if(image_sections <= 0) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

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

    status_update("(DFU) Erasing flash...", IS_LOG_LEVEL_INFO);

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

            ret_libusb = dfu_DNLOAD(&m_dfu.handle_libusb, 0, eraseCommand, 5);
            if (ret_libusb < LIBUSB_SUCCESS)
            { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

            ret_dfu = dfu_wait_for_state(&m_dfu.handle_libusb, DFU_STATE_DNLOAD_IDLE);
            if (ret_dfu < DFU_ERROR_NONE)
             { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

            byteInSection += STM32_PAGE_SIZE;
            bytes_written_total += STM32_PAGE_SIZE;

            m_update_progress = 0.25f * ((float)bytes_written_total / (float)image_total_len);
            m_update_callback(this, m_update_progress);
        } while(byteInSection < image[i].len - 1);
    }

    bytes_written_total = 0;

    status_update("(DFU) Programming flash...", IS_LOG_LEVEL_INFO);

    // Write memory
    for(size_t i = 0; i < image_sections; i++)
    {
        ret_dfu = dfu_wait_for_state(&m_dfu.handle_libusb, DFU_STATE_DNLOAD_IDLE);

        ret_dfu = dfu_set_address_pointer(&m_dfu.handle_libusb, image[i].address);
        if (ret_dfu < DFU_ERROR_NONE)
        {
            ihex_unload_sections(image, image_sections);
            libusb_release_interface(m_dfu.handle_libusb, 0);
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

            uint8_t blockNum = (uint8_t)(byteInSection / STM32_PAGE_SIZE);
    
            ret_libusb = dfu_DNLOAD(&m_dfu.handle_libusb, blockNum + 2, payload, STM32_PAGE_SIZE);
            if (ret_libusb < LIBUSB_SUCCESS) 
            {
                ihex_unload_sections(image, image_sections);
                libusb_release_interface(m_dfu.handle_libusb, 0);
                return IS_OP_ERROR;
            }

            ret_dfu = dfu_wait_for_state(&m_dfu.handle_libusb, DFU_STATE_DNLOAD_IDLE);
            if (ret_dfu < DFU_ERROR_NONE) 
            {
                ihex_unload_sections(image, image_sections);
                libusb_release_interface(m_dfu.handle_libusb, 0);
                return IS_OP_ERROR;
            }

            byteInSection += payloadLen;
            bytes_written_total += payloadLen;

            m_update_progress = 0.25f + 0.75f * ((float)bytes_written_total / (float)image_total_len);
            m_update_callback(this, m_update_progress);
        } while (byteInSection < image[i].len - 1);
    }

    // Unload the firmware image
    ihex_unload_sections(image, image_sections);

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&m_dfu.handle_libusb);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }
    
    // Reset status to good
    ret_dfu = dfu_wait_for_state(&m_dfu.handle_libusb, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    libusb_release_interface(m_dfu.handle_libusb, 0);

    return IS_OP_OK;
}

is_operation_result cISBootloaderDFU::reboot_up()
{
    int ret_libusb;
    dfu_error ret_dfu;

    m_info_callback(this, IS_LOG_LEVEL_INFO, "(DFU) Rebooting to IS-bootloader mode...");

    // Option bytes
    // This hard-coded array sets mostly defaults, but without PH3 enabled and
    // with DFU mode disabled. Application will enable DFU mode if needed.
    uint8_t bytes[] = {
        0xaa,0xf8,0xff,0xfb, 0x55,0x07,0x00,0x04,
        0xff,0xff,0xff,0xff, 0x00,0x00,0x00,0x00,
        0x00,0x00,0xff,0xff, 0xff,0xff,0x00,0x00,
        0xff,0xff,0x00,0xff, 0x00,0x00,0xff,0x00,
        0xff,0xff,0x00,0xff, 0x00,0x00,0xff,0x00
    };

    ret_libusb = libusb_claim_interface(m_dfu.handle_libusb, 0);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&m_dfu.handle_libusb);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    // Reset status to good
    ret_dfu = dfu_wait_for_state(&m_dfu.handle_libusb, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    ret_dfu = dfu_set_address_pointer(&m_dfu.handle_libusb, 0x1FFF7800);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    ret_libusb = dfu_DNLOAD(&m_dfu.handle_libusb, 2, bytes, sizeof(bytes));
    dfu_wait_for_state(&m_dfu.handle_libusb, DFU_STATE_DNLOAD_IDLE);	

    // ret_libusb = libusb_reset_device(dev_handle);
    // if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR; 

    libusb_release_interface(m_dfu.handle_libusb, 0);

    return IS_OP_OK;
}

is_operation_result cISBootloaderDFU::reboot()
{
    int ret_libusb;
    dfu_error ret_dfu;

    ret_libusb = libusb_claim_interface(m_dfu.handle_libusb, 0);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&m_dfu.handle_libusb);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }
    
    // Reset status to good
    ret_dfu = dfu_wait_for_state(&m_dfu.handle_libusb, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    // Reset USB device
    ret_libusb = libusb_reset_device(m_dfu.handle_libusb);
    if (ret_libusb < LIBUSB_SUCCESS)  { libusb_release_interface(m_dfu.handle_libusb, 0); return IS_OP_ERROR; }

    libusb_release_interface(m_dfu.handle_libusb, 0);

    return IS_OP_OK;
}

int cISBootloaderDFU::dfu_GETSTATUS(libusb_device_handle** dev_handle, dfu_status* status, uint32_t* delay, dfu_state* state, uint8_t* i_string)
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

int cISBootloaderDFU::dfu_CLRSTATUS(libusb_device_handle** dev_handle)
{
    return libusb_control_transfer(*dev_handle, 0b00100001, 0x04, 0, 0, NULL, 0, 100);
}

int cISBootloaderDFU::dfu_GETSTATE(libusb_device_handle** dev_handle, uint8_t* buf)
{
    return libusb_control_transfer(*dev_handle, 0b10100001, 0x05, 0, 0, buf, 1, 100);
}

int cISBootloaderDFU::dfu_ABORT(libusb_device_handle** dev_handle)
{
    return libusb_control_transfer(*dev_handle, 0b00100001, 0x06, 0, 0, NULL, 0, 100);
}

int cISBootloaderDFU::dfu_UPLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len)
{
    return libusb_control_transfer(*dev_handle, 0b10100001, 0x02, wValue, 0, buf, len, 100);
}

int cISBootloaderDFU::dfu_DNLOAD(libusb_device_handle** dev_handle, uint8_t wValue, uint8_t* buf, uint16_t len)
{
    return libusb_control_transfer(*dev_handle, 0b00100001, 0x01, wValue, 0, buf, len, 100);
}

int cISBootloaderDFU::dfu_DETACH(libusb_device_handle** dev_handle, uint8_t timeout)
{
    return libusb_control_transfer(*dev_handle, 0b00100001, 0x00, timeout, 0, NULL, 0, 100);
}

cISBootloaderDFU::dfu_error cISBootloaderDFU::dfu_set_address_pointer(libusb_device_handle** dev_handle, uint32_t address)
{
    int ret_libusb;
    unsigned char data[5] = { 0 };
	data[0] = 0x21;
	memcpy(&data[1], &address, 4);

    ret_libusb = dfu_DNLOAD(dev_handle, 0, data, 5);
	if(ret_libusb < LIBUSB_SUCCESS) return DFU_ERROR_LIBUSB;

    return dfu_wait_for_state(dev_handle, DFU_STATE_DNLOAD_IDLE);
}

cISBootloaderDFU::dfu_error cISBootloaderDFU::dfu_wait_for_state(libusb_device_handle** dev_handle, dfu_state required_state)
{
    dfu_status status = DFU_STATUS_ERR_UNKNOWN;
    uint32_t waitTime = 0;
    dfu_state state;
    uint8_t stringIndex;

    uint8_t tryCounter = 0;

    dfu_GETSTATUS(dev_handle, &status, &waitTime, &state, &stringIndex);

    while (status != DFU_STATUS_OK || state != required_state)
    {
        if (status != DFU_STATUS_OK)
        { 
            dfu_CLRSTATUS(dev_handle);
        }

        SLEEP_MS(_MAX(waitTime, 10));

        dfu_GETSTATUS(dev_handle, &status, &waitTime, &state, &stringIndex);

        if (++tryCounter > 5) return DFU_ERROR_TIMEOUT;
    }

    return DFU_ERROR_NONE;
}


