/**
 * @file ISBootloaderDFU.cpp
 * @author Dave Cutting
 * @brief Inertial Sense bootloader routines for DFU devices
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderDFU.h"
#include "ihex.h"
#include "ISUtilities.h"
#include "protocol/usb_dfu.h"
#include "util/md5.h"

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
static constexpr uint16_t USB_DESCRIPTOR_DFU = 0x21;

/*
 * Similar to libusb_get_string_descriptor_ascii but will allow1
 * truncated descriptors (descriptor length mismatch) seen on
 * e.g. the STM32F427 ROM bootloader.
 */
static int get_string_descriptor_ascii(libusb_device_handle *devh, uint8_t desc_index, char *data, int length)
{
    unsigned char tbuf[255];
    uint16_t langid;
    int r, di, si;

    /* get the language IDs and pick the first one */
    r = libusb_get_string_descriptor(devh, 0, 0, tbuf, sizeof(tbuf));
    if (r < 0) {
        // warnx("Failed to retrieve language identifiers");
        return r;
    }
    if (r < 4 || tbuf[0] < 4 || tbuf[1] != LIBUSB_DT_STRING) {		/* must have at least one ID */
        // warnx("Broken LANGID string descriptor");
        return -1;
    }
    langid = tbuf[2] | (tbuf[3] << 8);

    r = libusb_get_string_descriptor(devh, desc_index, langid, tbuf, sizeof(tbuf));
    if (r < 0) {
        // warnx("Failed to retrieve string descriptor %d", desc_index);
        return r;
    }
    if (tbuf[1] != LIBUSB_DT_STRING) {	/* sanity check */
        // warnx("Malformed string descriptor %d, type = 0x%02x", desc_index, tbuf[1]);
        return -1;
    }
    if (tbuf[0] > r) {	/* if short read,           */
        // warnx("Patching string descriptor %d length (was %d, received %d)", desc_index, tbuf[0], r);
        tbuf[0] = r;	/* fix up descriptor length */
    }

    /* convert from 16-bit unicode to ascii string */
    for (di = 0, si = 2; si + 1 < tbuf[0] && di < length; si += 2) {
        if (tbuf[si + 1])	/* high byte of unicode char */
            data[di++] = '?';
        else
            data[di++] = tbuf[si];
    }
    data[di] = 0;
    return di;
}

/*
 * Look for a descriptor in a concatenated descriptor list. Will
 * return upon the first match of the given descriptor type. Returns length of
 * found descriptor, limited to res_size
 */
static int find_descriptor(const uint8_t *desc_list, int list_len, uint8_t desc_type, void *res_buf, int res_size)
{
    int p = 0;

    if (list_len < 2)
        return (-1);

    while (p + 1 < list_len) {
        int desclen;

        desclen = (int) desc_list[p];
        if (desclen == 0) {
            printf("Invalid descriptor list"); // FIXME: WARN/DEBUG log?
            return -1;
        }
        if (desc_list[p + 1] == desc_type) {
            if (desclen > res_size)
                desclen = res_size;
            if (p + desclen > list_len)
                desclen = list_len - p;
            memcpy(res_buf, &desc_list[p], desclen);
            return desclen;
        }
        p += (int) desc_list[p];
    }
    return -1;
}

is_operation_result cISBootloaderDFU::match_test(void* param)
{
    const char* uid = (const char*)param;

    if(strnlen(uid, 20) != 0 && strnlen(m_dfu.dfuSerial.c_str(), 20) != 0 && strncmp(uid, m_dfu.dfuSerial.c_str(), 20) == 0)
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

        for (size_t i = 0; i < (device_count - 1); ++i) {
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

is_operation_result cISBootloaderDFU::list_devices(is_dfu_list& list)
{
    libusb_device** device_list;
    libusb_device* dev;
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

        // Add to list
        is_dfu_info devInfo {};
        libusb_device_handle *handle;
        if (libusb_open(dev, &handle) == LIBUSB_SUCCESS) {
            get_dfu_device_info(handle, devInfo);
            list.push_back(devInfo);
            libusb_close(handle);
        }
    }

    libusb_free_device_list(device_list, 1);
    return IS_OP_OK;
}

eImageSignature cISBootloaderDFU::check_is_compatible()
{
    return IS_IMAGE_SIGN_DFU;
}

is_operation_result cISBootloaderDFU::read_memory(libusb_device_handle** handle, uint8_t sn_idx, std::string& uidstr, uint32_t memloc, uint8_t* rxBuf, size_t rxLen) {
    // Get the 1K OTP section from the chip
    int ret_libusb;
    uint8_t stringIdx;
    dfu_error ret_dfu;

    uint32_t waitTime = 0;
    dfu_status status;
    dfu_state state;

    ret_libusb = libusb_claim_interface(*handle, 0);
    if (ret_libusb < LIBUSB_SUCCESS) {
        libusb_release_interface(*handle, 0);
        return IS_OP_ERROR;
    }

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(handle);
    if (ret_libusb < LIBUSB_SUCCESS) {
        libusb_release_interface(*handle, 0);
        return IS_OP_ERROR;
    }

    // Reset status to good
    ret_dfu = dfu_wait_for_state(handle, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) {
        libusb_release_interface(*handle, 0);
        return IS_OP_ERROR;
    }

    uint8_t txBuf[5] = { 0x21 }; // Set the address pointer (command is 0x21)
    txBuf[1] = memloc & 0xFF;
    txBuf[2] = memloc>>8 & 0xFF;
    txBuf[3] = memloc>>16 & 0xFF;
    txBuf[4] = memloc>>24 & 0xFF;


    ret_libusb = dfu_DNLOAD(handle, 0, txBuf, sizeof(txBuf));
    if(ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(*handle, 0); return IS_OP_ERROR; }

    // Address pointer takes effect after GETSTATUS command
    ret_libusb = dfu_GETSTATUS(handle, &status, &waitTime, &state, &stringIdx);
    if(ret_libusb < LIBUSB_SUCCESS || status != DFU_STATUS_OK || state != DFU_STATE_DNBUSY) {
        libusb_release_interface(*handle, 0);
        return IS_OP_ERROR;
    }

    ret_libusb = dfu_GETSTATUS(handle, &status, &waitTime, &state, &stringIdx);
    if(ret_libusb < LIBUSB_SUCCESS || status != DFU_STATUS_OK) {
        libusb_release_interface(*handle, 0);
        return IS_OP_ERROR;
    }

    // Get out of download mode
    dfu_ABORT(handle);

    // Read the full requested memory
    ret_libusb = dfu_UPLOAD(handle, 2, rxBuf, rxLen);

    // error or not, we're done here
    libusb_release_interface(*handle, 0);
    return (ret_libusb < LIBUSB_SUCCESS) ? IS_OP_ERROR : IS_OP_OK;
}

//is_operation_result cISBootloaderDFU::get_serial_number_libusb(libusb_device_handle** handle, uint32_t& sn, std::string& uidstr, uint8_t sn_idx)
is_operation_result cISBootloaderDFU::get_dfu_device_info(libusb_device_handle* handle, is_dfu_info& info)
{
    uint8_t rxBuf[2048] = {0};
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor *cfg;
    struct usb_dfu_func_descriptor func_dfu;

    if (libusb_claim_interface(handle, 0) < LIBUSB_SUCCESS)
        return IS_OP_ERROR;

    SLEEP_MS(100);
    // TODO make this IMX/GPX aware (OTP location maybe processor and/or product/device dependent)
    // Its important to note that if this is a BRAND NEW, NEVER BEEN PROGRAMMED STM32, it will have no OTP data. Even if we know the MCU type, we still don't know the application its in (ie, IMX-5.1 vs GPX-1).

    libusb_device *usb_device = libusb_get_device(handle);
    if (libusb_get_device_descriptor(usb_device, &desc) != LIBUSB_SUCCESS) {
        libusb_release_interface(handle, 0);
        return IS_OP_ERROR;
    }

    //info.dfuSerial;
    info.vid = desc.idVendor;
    info.pid = desc.idProduct;
    info.usbDevice = usb_device;
    // info.usbHandle = handle;
    info.iSerialNumber = desc.iSerialNumber; // device descriptor to retrieve the DFU device's serial number (uid)??
    md5_reset(info.fingerprint);

    // Get the string containing the serial number from the device
    unsigned char uid[IS_DFU_UID_MAX_SIZE];
    if (libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, uid, sizeof(uid)) < LIBUSB_SUCCESS)
        uid[0] = '\0'; // Set the serial number as none
    info.dfuSerial = std::string((const char*)uid);

    // iterate configurations
    for (int cfg_idx = 0; cfg_idx < desc.bNumConfigurations; cfg_idx++) {
        if (libusb_get_config_descriptor(usb_device, cfg_idx, &cfg) == 0) {

            // iterate interfaces
            const struct libusb_interface_descriptor *intf;
            const struct libusb_interface *uif;
            for (int intf_idx = 0; intf_idx < cfg->bNumInterfaces; intf_idx++) {
                uif = &cfg->interface[intf_idx];
                if (!uif)
                    break;

                for (int alt_idx = 0; alt_idx < cfg->interface[intf_idx].num_altsetting; alt_idx++) {
                    intf = &uif->altsetting[alt_idx];
                    if ((intf == nullptr) || (intf->bInterfaceClass != 0xfe) || (intf->bInterfaceSubClass != 1))
                        continue;

                    memset(&func_dfu, 0, sizeof(func_dfu));
                    if (find_descriptor(intf->extra, intf->extra_length, USB_DT_DFU, &func_dfu, sizeof(func_dfu)) < 0) {
                        if (libusb_get_descriptor(handle, USB_DT_DFU, 0, (unsigned char *) &func_dfu, sizeof(func_dfu)) < 0) {
                            /* fake version 1.0 */
                            func_dfu.bLength = 7;
                            func_dfu.bcdDFUVersion = libusb_cpu_to_le16(0x0100);
                            // continue; // no valid functional descriptor
                        }
                    }

                    if (func_dfu.bLength == 7) {
                        func_dfu.bcdDFUVersion = libusb_cpu_to_le16(0x0100);
                    } else if (func_dfu.bLength < 9) {
                        // printf("Error obtaining DFU functional descriptor. Warning: Transfer size can not be detected\n");
                        func_dfu.bcdDFUVersion = libusb_cpu_to_le16(0x0100);
                        func_dfu.wTransferSize = 0;
                    }

                    int dfu_mode = (intf->bInterfaceProtocol == 2);

                    /* e.g. DSO Nano has bInterfaceProtocol 0 instead of 2 */
                    if (func_dfu.bcdDFUVersion == 0x011a && intf->bInterfaceProtocol == 0)
                        dfu_mode = 1;

                    /* LPC DFU bootloader has bInterfaceProtocol 1 (Runtime) instead of 2 */
                    if (desc.idVendor == 0x1fc9 && desc.idProduct == 0x000c && intf->bInterfaceProtocol == 1)
                        dfu_mode = 1;

                    if (intf->iInterface != 0) {
                        char alt_name[MAX_DESC_STR_LEN];
                        if (get_string_descriptor_ascii(handle, intf->iInterface, alt_name, MAX_DESC_STR_LEN) > 0)
                            info.dfuDescriptors.push_back(alt_name);
                        else
                            info.dfuDescriptors.push_back("");
                    }
                }
            }

            libusb_free_config_descriptor(cfg);
        }
    }

    // Calculate a fingerprint from the device info/descriptors (Don't use anything that isn't guaranteed unique per device-type!!)
    // TODO: IF YOU CHANGE THE DATA USED IN THE HASHING BELOW, YOU WILL ALSO HAVE TO CHANGE THE RESPECTIVE FINGERPRINTS in ISBootloaderDFU.h
    // DO NOT MODIFY THESE IF YOU DON'T KNOW WHAT YOU'RE DOING AND WHY
    md5_hash(info.fingerprint, sizeof(info.vid), (uint8_t *)&info.vid);
    md5_hash(info.fingerprint, sizeof(info.pid), (uint8_t *)&info.pid);
    for (auto dfuDesc : info.dfuDescriptors) {
        md5_hash(info.fingerprint, dfuDesc.size(), (uint8_t*)dfuDesc.c_str());
    }

    // TODO: make this work for both GPX-1 and IMX-5.1
    uint16_t hardwareType = 0;
    eProcessorType processor_hint = IS_PROCESSOR_UNKNOWN;
    if (MD5HASH_MATCHES(info.fingerprint, DFU_FINGERPRINT_STM32L4)) processor_hint = IS_PROCESSOR_STM32L4; // possible IMX
    else if (MD5HASH_MATCHES(info.fingerprint, DFU_FINGERPRINT_STM32U5)) processor_hint = IS_PROCESSOR_STM32U5; // possible GPX

    // find OTP Memory region from dfuDescriptors
    uint32_t otpLocation = 0x00000000;
    for (auto dfuDesc : info.dfuDescriptors) {
        if (dfuDesc.compare(0, 8, "@OTP Mem") == 0) {
            std::vector<std::string> params;
            splitString(dfuDesc,'/',params);
            otpLocation = strtol(params[1].c_str(), nullptr, 16);
            break;
        }
    }

    // try and read the OTP memory
    if ((otpLocation != 0) && (read_memory(&handle, desc.iSerialNumber, info.dfuSerial, otpLocation, rxBuf, 1024) == IS_OP_OK)) {
        info.sn = -1; // 0xFFFFFFFF
        info.hdwType = -1; // 0xFFFF
        is_dfu_otp_id_t* id = decode_otp_data(rxBuf, sizeof(rxBuf));
        if(id != nullptr) {
            info.sn = id->serialNumber;
            hardwareType = id->hardwareId;
        }
    } else {
        libusb_release_interface(handle, 0);
        return IS_OP_ERROR;
    }

    if ((info.hdwType == 0xFFFF) && (processor_hint != IS_PROCESSOR_UNKNOWN)) {
        if (processor_hint == IS_PROCESSOR_STM32L4) hardwareType = DEV_INFO_HARDWARE_IMX; // only the IMX-5 uses the STM32L4
        else if (processor_hint == IS_PROCESSOR_STM32U5) hardwareType = DEV_INFO_HARDWARE_GPX; // TODO: Both the GPX-1 and IMX-5.1 will use the STM32U5 (at the moment)
    }

    // based on what we know so far, let's try and figure out a hardware type
    if ((hardwareType & 0xFFF0) == 0) {
        // if this is true, then we don't *really* know the hardware type or version (just the type)
        switch (hardwareType) {
            case DEV_INFO_HARDWARE_UINS:
                hardwareType = ENCODE_HDW_INFO(DEV_INFO_HARDWARE_UINS, 3, 2);
                break;
            case DEV_INFO_HARDWARE_EVB:
                hardwareType = ENCODE_HDW_INFO(DEV_INFO_HARDWARE_EVB, 2, 0);
                break;
            case DEV_INFO_HARDWARE_IMX:
                hardwareType = ENCODE_HDW_INFO(DEV_INFO_HARDWARE_IMX, 5, 0);
                break;
            case DEV_INFO_HARDWARE_GPX:
                hardwareType = ENCODE_HDW_INFO(DEV_INFO_HARDWARE_GPX, 1, 0);
                break;
        }
    }
    info.hdwType = hardwareType;

    libusb_release_interface(handle, 0);
    return IS_OP_OK;
}

uint32_t cISBootloaderDFU::get_device_info()
{
    // get_serial_number_libusb(&m_dfu.handle_libusb, m_hdw, m_sn, m_port_name, m_dfu.iSerialNumber);
    libusb_device_handle *handle;
    if (libusb_open(m_dfu.usbDevice, &handle) == LIBUSB_SUCCESS) {
        get_dfu_device_info(handle, m_dfu);
        libusb_close(handle);
        handle = nullptr;
    }

    return m_dfu.sn;
}

is_operation_result cISBootloaderDFU::download_image(std::string filename)
{
    int ret_libusb;
    dfu_error ret_dfu;
    ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
    size_t image_sections;

    SLEEP_MS(100);

    if (libusb_open(m_dfu.usbDevice, &m_dfu.usbHandle) < LIBUSB_SUCCESS)
        return IS_OP_ERROR;

    ret_libusb = libusb_claim_interface(m_dfu.usbHandle, 0);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&m_dfu.usbHandle);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Reset status to good
    ret_dfu = dfu_wait_for_state(&m_dfu.usbHandle, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Load the firmware image
    image_sections = ihex_load_sections(filename.c_str(), image, MAX_NUM_IHEX_SECTIONS);
    if(image_sections <= 0) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

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

            ret_libusb = dfu_DNLOAD(&m_dfu.usbHandle, 0, eraseCommand, 5);
            if (ret_libusb < LIBUSB_SUCCESS)
            { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

            ret_dfu = dfu_wait_for_state(&m_dfu.usbHandle, DFU_STATE_DNLOAD_IDLE);
            if (ret_dfu < DFU_ERROR_NONE)
            { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

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
        ret_dfu = dfu_wait_for_state(&m_dfu.usbHandle, DFU_STATE_DNLOAD_IDLE);

        ret_dfu = dfu_set_address_pointer(&m_dfu.usbHandle, image[i].address);
        if (ret_dfu < DFU_ERROR_NONE)
        {
            ihex_unload_sections(image, image_sections);
            libusb_release_interface(m_dfu.usbHandle, 0);
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

            ret_libusb = dfu_DNLOAD(&m_dfu.usbHandle, blockNum + 2, payload, STM32_PAGE_SIZE);
            if (ret_libusb < LIBUSB_SUCCESS)
            {
                ihex_unload_sections(image, image_sections);
                libusb_release_interface(m_dfu.usbHandle, 0);
                return IS_OP_ERROR;
            }

            ret_dfu = dfu_wait_for_state(&m_dfu.usbHandle, DFU_STATE_DNLOAD_IDLE);
            if (ret_dfu < DFU_ERROR_NONE)
            {
                ihex_unload_sections(image, image_sections);
                libusb_release_interface(m_dfu.usbHandle, 0);
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
    ret_libusb = dfu_ABORT(&m_dfu.usbHandle);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Reset status to good
    ret_dfu = dfu_wait_for_state(&m_dfu.usbHandle, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    libusb_release_interface(m_dfu.usbHandle, 0);

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

    ret_libusb = libusb_claim_interface(m_dfu.usbHandle, 0);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&m_dfu.usbHandle);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Reset status to good
    ret_dfu = dfu_wait_for_state(&m_dfu.usbHandle, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    ret_dfu = dfu_set_address_pointer(&m_dfu.usbHandle, 0x1FFF7800);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    ret_libusb = dfu_DNLOAD(&m_dfu.usbHandle, 2, bytes, sizeof(bytes));
    dfu_wait_for_state(&m_dfu.usbHandle, DFU_STATE_DNLOAD_IDLE);

    // ret_libusb = libusb_reset_device(dev_handle);
    // if (ret_libusb < LIBUSB_SUCCESS) return IS_OP_ERROR; 

    libusb_release_interface(m_dfu.usbHandle, 0);

    return IS_OP_OK;
}

is_operation_result cISBootloaderDFU::reboot()
{
    int ret_libusb;
    dfu_error ret_dfu;

    ret_libusb = libusb_claim_interface(m_dfu.usbHandle, 0);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Cancel any existing operations
    ret_libusb = dfu_ABORT(&m_dfu.usbHandle);
    if (ret_libusb < LIBUSB_SUCCESS) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Reset status to good
    ret_dfu = dfu_wait_for_state(&m_dfu.usbHandle, DFU_STATE_IDLE);
    if (ret_dfu < DFU_ERROR_NONE) { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    // Reset USB device
    ret_libusb = libusb_reset_device(m_dfu.usbHandle);
    if (ret_libusb < LIBUSB_SUCCESS)  { libusb_release_interface(m_dfu.usbHandle, 0); return IS_OP_ERROR; }

    libusb_release_interface(m_dfu.usbHandle, 0);

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

is_dfu_otp_id_t* cISBootloaderDFU::decode_otp_data(uint8_t* raw, int len) {
    int index = 0;
    uint8_t* otp_mem = (uint8_t*)raw;

    // Look for the first section of zeroes
    uint8_t cmp[OTP_SECTION_SIZE];
    memset(cmp, 0xFF, OTP_SECTION_SIZE);
    bool foundSn = false;
    while(memcmp(cmp, otp_mem, OTP_SECTION_SIZE) != 0)
    {
        otp_mem += OTP_SECTION_SIZE; index++;
        if(index >= (int)OTP_NUM_SECTIONS)
        {
            foundSn = false;
            break;  // No more room in OTP
        }
        foundSn = true;
    }

    // Go back one, to the last filled section
    index--;
    if (index < 0) return nullptr;

    is_dfu_otp_id_t* otp = (is_dfu_otp_id_t*)((index * OTP_SECTION_SIZE) + raw);

    uint64_t key = OTP_KEY;
    if(memcmp(otp_mem - 8, &key, 8) == 0 && foundSn) {
        return otp;
    }

    return nullptr;
}

