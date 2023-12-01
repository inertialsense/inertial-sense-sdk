/**
 * @file ISDFUFirmwareUpdater.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 11/28/23.
 * @copyright Copyright (c) 2023 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDFUFirmwareUpdater.h"
#include <algorithm>

namespace dfu {

    int ISDFUFirmwareUpdater::getAvailableDevices(std::vector<DFUDevice *> &devices, uint16_t vid, uint16_t pid) {
        libusb_device **device_list;
        libusb_device *dev;

        size_t device_count = libusb_get_device_list(NULL, &device_list);
        for (size_t i = 0; i < device_count; ++i) {
            dev = device_list[i];

            // let's instantiate a DFUDevice object and add it to our list, if everything is cool
            if (isDFUDevice(dev, vid, pid)) {
                DFUDevice *dfuDevice = new DFUDevice(dev);
                devices.push_back(dfuDevice);
            }
        }

        libusb_free_device_list(device_list, 1);
        return devices.size();
    }

    int ISDFUFirmwareUpdater::filterDevicesByFingerprint(std::vector<DFUDevice *> &devices, md5hash_t fingerprint) {

        auto removed = std::remove_if(std::begin(devices), std::end(devices), [&fingerprint](DFUDevice *d) { return !md5_matches(d->getFingerprint(), fingerprint); });
        devices.erase(removed, devices.end());
        return devices.size();
    }

/**
 * Checks if the specified device is a valid DFU device
 * @param usbDevice
 * @param vid
 * @param pid
 * @return
 */
    bool dfu::ISDFUFirmwareUpdater::isDFUDevice(libusb_device *usbDevice, uint16_t vid, uint16_t pid) {
        struct libusb_device_descriptor desc;
        struct libusb_config_descriptor *cfg;

        if (libusb_get_device_descriptor(usbDevice, &desc) < 0)
            return false;

        // Check vendor and product ID
        if (((vid != 0x0000) && (desc.idVendor != vid)) || ((pid != 0x0000) && (desc.idProduct != pid)))
            return false;      // must be some other usb device

        if (libusb_get_config_descriptor(usbDevice, 0, &cfg) < 0)
            return false;

        // USB-IF DFU interface class numbers
        if ((cfg->interface->altsetting[0].bInterfaceClass != 0xFE) ||
            (cfg->interface->altsetting[0].bInterfaceSubClass != 0x01) ||
            (cfg->interface->altsetting[0].bInterfaceProtocol != 0x02))
            return false;

        return true;
    }

    dfu_error DFUDevice::fetchDeviceInfo() {
        struct libusb_device_descriptor desc;
        struct libusb_config_descriptor *cfg;
        struct usb_dfu_func_descriptor func_dfu;

        if (!isConnected())
            open();

        if (libusb_claim_interface(usbHandle, 0) < LIBUSB_SUCCESS)
            return DFU_ERROR_DEVICE_BUSY;

        SLEEP_MS(100);
        // TODO make this IMX/GPX aware (OTP location maybe processor and/or product/device dependent)
        // Its important to note that if this is a BRAND NEW, NEVER BEEN PROGRAMMED STM32, it will have no OTP data. Even if we know the MCU type, we still don't know the application its in (ie, IMX-5.1 vs GPX-1).

        libusb_device *usb_device = libusb_get_device(usbHandle);
        if (libusb_get_device_descriptor(usb_device, &desc) != LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_LIBUSB;
        }

        //info.dfuSerial;
        vid = desc.idVendor;
        pid = desc.idProduct;
        usbDevice = usb_device;
        // info.usbHandle = handle;
        iSerialNumber = desc.iSerialNumber; // device descriptor to retrieve the DFU device's serial number (uid)??
        md5_reset(fingerprint);

        // Get the string containing the serial number from the device
        unsigned char uid[UID_MAX_SIZE];
        if (libusb_get_string_descriptor_ascii(usbHandle, desc.iSerialNumber, uid, sizeof(uid)) < LIBUSB_SUCCESS)
            uid[0] = '\0'; // Set the serial number as none
        dfuSerial = std::string((const char *) uid);

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
                        if (findDescriptor(intf->extra, intf->extra_length, USB_DESCRIPTOR_DFU, &func_dfu, sizeof(func_dfu)) < 0) {
                            if (libusb_get_descriptor(usbHandle, USB_DESCRIPTOR_DFU, 0, (unsigned char *) &func_dfu, sizeof(func_dfu)) < 0) {
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

                        bool dfu_mode = (intf->bInterfaceProtocol == 2);

                        /* e.g. DSO Nano has bInterfaceProtocol 0 instead of 2 */
                        if (func_dfu.bcdDFUVersion == 0x011a && intf->bInterfaceProtocol == 0)
                            dfu_mode = true;

                        /* LPC DFU bootloader has bInterfaceProtocol 1 (Runtime) instead of 2 */
                        if (desc.idVendor == 0x1fc9 && desc.idProduct == 0x000c && intf->bInterfaceProtocol == 1)
                            dfu_mode = true;

                        if (dfu_mode && (intf->iInterface != 0)) {
                            char alt_name[MAX_DESC_STR_LEN];
                            if (get_string_descriptor_ascii(intf->iInterface, alt_name, MAX_DESC_STR_LEN) > 0)
                                dfuDescriptors.push_back(alt_name);
                            else
                                dfuDescriptors.push_back("");
                        }
                    }
                }

                libusb_free_config_descriptor(cfg);
            }
        }

        // Calculate a fingerprint from the device info/descriptors (Don't use anything that isn't guaranteed unique per device-type!!)
        // TODO: IF YOU CHANGE THE DATA USED IN THE HASHING BELOW, YOU WILL ALSO HAVE TO CHANGE THE RESPECTIVE FINGERPRINTS in ISBootloaderDFU.h
        // DO NOT MODIFY THESE IF YOU DON'T KNOW WHAT YOU'RE DOING AND WHY
        md5_hash(fingerprint, sizeof(vid), (uint8_t *) &vid);
        md5_hash(fingerprint, sizeof(pid), (uint8_t *) &pid);
        for (auto dfuDesc: dfuDescriptors) {
            md5_hash(fingerprint, dfuDesc.size(), (uint8_t *) dfuDesc.c_str());
        }

        // TODO: make this work for both GPX-1 and IMX-5.1
        uint16_t hardwareType = 0;
        processorType = IS_PROCESSOR_UNKNOWN;

        if (md5_matches(fingerprint, DFU_FINGERPRINT_STM32L4)) processorType = IS_PROCESSOR_STM32L4; // possible IMX
        else if (md5_matches(fingerprint, DFU_FINGERPRINT_STM32U5)) processorType = IS_PROCESSOR_STM32U5; // possible GPX

        // find OTP Memory region from dfuDescriptors
        uint32_t otpLocation = 0x00000000;
        for (auto dfuDesc: dfuDescriptors) {
            if (dfuDesc.compare(0, 8, "@OTP Mem") == 0) {
                std::vector<std::string> params;
                splitString(dfuDesc, '/', params);
                otpLocation = strtol(params[1].c_str(), nullptr, 16);
                break;
            }
        }

        // try and read the OTP memory
        if (otpLocation != 0) {
            sn = -1; // 0xFFFFFFFF
            hardwareId = -1; // 0xFFFF
            uint8_t rxBuf[OTP_SECTION_SIZE * OTP_NUM_SECTIONS] = {0};

            int len = readMemory(usbHandle, otpLocation, rxBuf, sizeof(rxBuf));
            if (len > 0) {
                otp_info_t *id = decodeOTPData(rxBuf, sizeof(rxBuf));
                if (id != nullptr) {
                    sn = id->serialNumber;
                    hardwareType = id->hardwareId;
                }
            }
        } else {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_LIBUSB;
        }

        if ((hardwareId == 0xFFFF) && (processorType != IS_PROCESSOR_UNKNOWN)) {
            if (processorType == IS_PROCESSOR_STM32L4) hardwareType = DEV_INFO_HARDWARE_IMX; // only the IMX-5 uses the STM32L4
            else if (processorType == IS_PROCESSOR_STM32U5) hardwareType = DEV_INFO_HARDWARE_GPX; // TODO: Both the GPX-1 and IMX-5.1 will use the STM32U5 (at the moment)
        }

        // based on what we know so far, let's try and figure out a hardware type
        if ((hardwareType & 0xFFF0) == 0) {
            // if this is true, then we don't *really* know the hardware type or version (just the type)
            switch (hardwareType) {
                case DEV_INFO_HARDWARE_UINS:
                    hardwareId = ENCODE_HDW_INFO(DEV_INFO_HARDWARE_UINS, 3, 2);
                    break;
                case DEV_INFO_HARDWARE_EVB:
                    hardwareId = ENCODE_HDW_INFO(DEV_INFO_HARDWARE_EVB, 2, 0);
                    break;
                case DEV_INFO_HARDWARE_IMX:
                    hardwareId = ENCODE_HDW_INFO(DEV_INFO_HARDWARE_IMX, 5, 0);
                    break;
                case DEV_INFO_HARDWARE_GPX:
                    hardwareId = ENCODE_HDW_INFO(DEV_INFO_HARDWARE_GPX, 1, 0);
                    break;
            }
        }

        libusb_release_interface(usbHandle, 0);
        return DFU_ERROR_NONE;
    }


    DFUDevice::otp_info_t *DFUDevice::decodeOTPData(uint8_t *raw, int len) {
        int index = 0;
        uint8_t *otp_mem = (uint8_t *) raw;

        // Look for the first section of zeroes
        uint8_t cmp[OTP_SECTION_SIZE];
        memset(cmp, 0xFF, OTP_SECTION_SIZE);
        bool foundSn = false;
        while (memcmp(cmp, otp_mem, OTP_SECTION_SIZE) != 0) {
            otp_mem += OTP_SECTION_SIZE;
            index++;
            if (index >= (int) OTP_NUM_SECTIONS) {
                foundSn = false;
                break;  // No more room in OTP
            }
            foundSn = true;
        }

        // Go back one, to the last filled section
        index--;
        if (index < 0) return nullptr;

        otp_info_t *otp = (otp_info_t * )((index * OTP_SECTION_SIZE) + raw);

        uint64_t key = OTP_KEY;
        if (memcmp(otp_mem - 8, &key, 8) == 0 && foundSn) {
            return otp;
        }

        return nullptr;
    }


/**
 * Open a connection to the associated USB device and establish USB DFU status is IDLE
 * @return
 */
    dfu_error DFUDevice::open() {
        int ret_libusb;
        dfu_error ret_dfu;

        if (libusb_open(usbDevice, &usbHandle) < LIBUSB_SUCCESS)
            return DFU_ERROR_DEVICE_NOTFOUND;

        ret_libusb = libusb_claim_interface(usbHandle, 0);
        if (ret_libusb < LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_DEVICE_BUSY;
        }

        // Cancel any existing operations
        ret_libusb = abort(usbHandle);
        if (ret_libusb < LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_STATUS;
        }

        // Reset status to good
        ret_libusb = waitForState(usbHandle, DFU_STATE_IDLE);
        if (ret_libusb < LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_STATUS;
        }

        return DFU_ERROR_NONE;
    }

    dfu_error DFUDevice::writeFirmware(std::string filename, uint64_t baseAddress) {
        ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
        size_t image_sections;
        int ret_libusb;
        dfu_error ret_dfu;

        SLEEP_MS(100);

        /**
         * Load the firmware's .hex file, parse its sections, and apply a base offset, if necessary
         */
        // Load the firmware image
        image_sections = ihex_load_sections(filename.c_str(), image, MAX_NUM_IHEX_SECTIONS);
        if (image_sections <= 0) {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_INVALID_ARG;
        }

        int image_total_len = 0;
        for (size_t i = 0; i < image_sections; image_total_len += image[i].len, i++);

        // If starting address is 0, set it to 0x08000000 (start of flash memory)
        if (image[0].address == 0x00000000) {
            for (size_t i = 0; i < image_sections; image[i].address += baseAddress, i++);
        }

        erase(image_sections, image);

        // status_update("(DFU) Programming flash...", IS_LOG_LEVEL_INFO);

        // Write memory
        uint32_t bytes_written_total = 0;
        for (size_t i = 0; i < image_sections; i++) {
            ret_libusb = waitForState(usbHandle, DFU_STATE_DNLOAD_IDLE);

            ret_libusb = setAddress(usbHandle, image[i].address);
            if (ret_libusb < LIBUSB_SUCCESS) {
                ihex_unload_sections(image, image_sections);
                libusb_release_interface(usbHandle, 0);
                return DFU_ERROR_INVALID_ARG;
            }

            uint32_t byteInSection = 0;
            do {
                uint8_t payload[STM32_PAGE_SIZE] = {0};
                uint32_t payloadLen = STM32_PAGE_SIZE;
                uint32_t bytesRemaining = image[i].len - byteInSection;
                if (payloadLen > bytesRemaining)
                    payloadLen = bytesRemaining;

                // Copy image into buffer for transmission
                memset(payload, 0xFF, STM32_PAGE_SIZE);
                memcpy(payload, &image[i].image[byteInSection], payloadLen);

                uint8_t blockNum = (uint8_t) (byteInSection / STM32_PAGE_SIZE);

                ret_libusb = download(usbHandle, blockNum + 2, payload, STM32_PAGE_SIZE);
                if (ret_libusb < LIBUSB_SUCCESS) {
                    ihex_unload_sections(image, image_sections);
                    libusb_release_interface(usbHandle, 0);
                    return DFU_ERROR_LIBUSB;
                }

                ret_libusb = waitForState(usbHandle, DFU_STATE_DNLOAD_IDLE);
                if (ret_libusb < LIBUSB_SUCCESS) {
                    ihex_unload_sections(image, image_sections);
                    libusb_release_interface(usbHandle, 0);
                    return DFU_ERROR_LIBUSB;
                }

                byteInSection += payloadLen;
                bytes_written_total += payloadLen;

                // TODO implement progress callbacks
                // m_update_progress = 0.25f + 0.75f * ((float)bytes_written_total / (float)image_total_len);
                // m_update_callback(this, m_update_progress);
            } while (byteInSection < image[i].len - 1);
        }

        // Unload the firmware image
        ihex_unload_sections(image, image_sections);

        return DFU_ERROR_NONE;
    }

/**
 * Erases the flash memory on the device only where the image will live
 * @param image_sections
 * @param image an array of image sections which need to be erased.
 * @return
 */
    dfu_error DFUDevice::erase(int image_sections, ihex_image_section_t *image) {
        /**
         * Prepare the device's FLASH by first erasing the flash contents
         * FIXME: This should probably be in its own method!
         */
        // status_update("(DFU) Erasing flash...", IS_LOG_LEVEL_INFO);
        int ret_libusb;
        dfu_error ret_dfu;

        uint32_t image_total_len = 0;
        uint32_t bytes_total_erased = 0;
        float progress = 0.0f;

        for (int i = 0; i < image_sections; image_total_len += image[i].len, i++);

        // Erase memory (only erase pages where firmware lives)
        for (int i = 0; i < image_sections; i++) {
            if (image[i].address & STM32_PAGE_ERROR_MASK)
                continue;   // Page is not aligned with write location

            if ((image[i].image == NULL) || (image[i].len == 0))
                continue;   // Null image

            uint32_t byteInSection = 0;
            do {
                uint32_t pageAddress = byteInSection + image[i].address;
                uint8_t eraseCommand[5];

                eraseCommand[0] = 0x41;
                memcpy(&eraseCommand[1], &pageAddress, 4);

                ret_libusb = download(usbHandle, 0, eraseCommand, 5);
                if (ret_libusb < LIBUSB_SUCCESS) {
                    libusb_release_interface(usbHandle, 0);
                    return DFU_ERROR_STATUS;
                }

                ret_libusb = waitForState(usbHandle, DFU_STATE_DNLOAD_IDLE);
                if (ret_libusb < LIBUSB_SUCCESS) {
                    libusb_release_interface(usbHandle, 0);
                    return DFU_ERROR_STATUS;
                }

                byteInSection += STM32_PAGE_SIZE;
                bytes_total_erased += STM32_PAGE_SIZE;

                // TODO implement progress callbacks
                // progress = 0.25f * ((float)bytes_total_erased / (float)image_total_len);
                // m_update_callback(this, progress);
            } while (byteInSection < image[i].len - 1);
        }

        return DFU_ERROR_NONE;
    }


    dfu_error DFUDevice::close() {
        int ret_libusb;
        dfu_error ret_dfu;

        // Cancel any existing operations
        ret_libusb = abort(usbHandle);
        if (ret_libusb < LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_LIBUSB;
        }

        // Reset status to good
        ret_libusb = waitForState(usbHandle, DFU_STATE_IDLE);
        if (ret_libusb < LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_LIBUSB;
        }

        libusb_release_interface(usbHandle, 0);

        return DFU_ERROR_NONE;
    }

/*
 * Similar to libusb_get_string_descriptor_ascii but will allow1
 * truncated descriptors (descriptor length mismatch) seen on
 * e.g. the STM32F427 ROM bootloader.
 */
    int DFUDevice::get_string_descriptor_ascii(uint8_t desc_index, char *data, int length) {
        unsigned char tbuf[255];
        uint16_t langid;
        int r, di, si;

        /* get the language IDs and pick the first one */
        r = libusb_get_string_descriptor(usbHandle, 0, 0, tbuf, sizeof(tbuf));
        if (r < 0) {
            // warnx("Failed to retrieve language identifiers");
            return r;
        }
        if (r < 4 || tbuf[0] < 4 || tbuf[1] != LIBUSB_DT_STRING) {        /* must have at least one ID */
            // warnx("Broken LANGID string descriptor");
            return -1;
        }
        langid = tbuf[2] | (tbuf[3] << 8);

        r = libusb_get_string_descriptor(usbHandle, desc_index, langid, tbuf, sizeof(tbuf));
        if (r < 0) {
            // warnx("Failed to retrieve string descriptor %d", desc_index);
            return r;
        }
        if (tbuf[1] != LIBUSB_DT_STRING) {    /* sanity check */
            // warnx("Malformed string descriptor %d, type = 0x%02x", desc_index, tbuf[1]);
            return -1;
        }
        if (tbuf[0] > r) {    /* if short read,           */
            // warnx("Patching string descriptor %d length (was %d, received %d)", desc_index, tbuf[0], r);
            tbuf[0] = r;    /* fix up descriptor length */
        }

        /* convert from 16-bit unicode to ascii string */
        for (di = 0, si = 2; si + 1 < tbuf[0] && di < length; si += 2) {
            if (tbuf[si + 1])    /* high byte of unicode char */
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
    int DFUDevice::findDescriptor(const uint8_t *desc_list, int list_len, uint8_t desc_type, void *res_buf, int res_size) {
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

    int DFUDevice::readMemory(libusb_device_handle *handle, uint32_t memloc, uint8_t *rxBuf, size_t rxLen) {
        // Get the 1K OTP section from the chip
        int ret_libusb;
        uint8_t stringIdx;
        dfu_error ret_dfu;

        uint32_t waitTime = 0;
        dfu_status status;
        dfu_state state;
        int bytes_read;

        // Cancel any existing operations
        ret_libusb = abort(handle);
        if (ret_libusb >= LIBUSB_SUCCESS) {
            ret_libusb = waitForState(handle, DFU_STATE_IDLE);
            if (ret_libusb >= LIBUSB_SUCCESS) {
                ret_libusb = setAddress(handle, memloc);
                if (ret_libusb >= LIBUSB_SUCCESS) {
                    // Read the full requested memory
                    ret_libusb = upload(handle, 2, rxBuf, rxLen);
                    if (ret_libusb >= LIBUSB_SUCCESS) {
                        bytes_read = ret_libusb;
                        ret_libusb = getStatus(handle, &status, &waitTime, &state, &stringIdx);
                        if (ret_libusb >= LIBUSB_SUCCESS)
                            return bytes_read;
                    }
                }
            }
        }

        return ret_libusb;
    }


/**
 * Enters a blocking state while waiting for the USB DFU device to enter a particular state.
 * This implementation has a fixed timeout(10ms)/retry(5x) cycle and should never block for more than ~50ms.
 * @param dev_handle
 * @param required_state
 * @return DFU_ERROR_NONE if state watches the required_state, otherwise will return DFU_ERROR_TIMEOUT if the timeout condition occurs first.
 */
    int DFUDevice::waitForState(libusb_device_handle *dev_handle, dfu_state required_state) {
        dfu_status status = DFU_STATUS_ERR_UNKNOWN;
        uint32_t waitTime = 0;
        dfu_state state;
        uint8_t stringIndex;
        int ret_libusb = 0;
        uint8_t tryCounter = 0;

        ret_libusb = getStatus(dev_handle, &status, &waitTime, &state, &stringIndex);
        while (status != DFU_STATUS_OK || state != required_state) {
            if (status != DFU_STATUS_OK) {
                clearStatus(dev_handle);
            }

            SLEEP_MS(_MAX(waitTime, 10));
            ret_libusb = getStatus(dev_handle, &status, &waitTime, &state, &stringIndex);

            if (++tryCounter > 5)
                return ret_libusb;
            break;
        }

        return LIBUSB_SUCCESS;
    }

/**
 * Directs the DFU device to read/write memory from the specified address location (this is not a pointer!)
 * @param dev_handle
 * @param address
 * @return
 */
    int DFUDevice::setAddress(libusb_device_handle *dev_handle, uint32_t address) {
        dfu_state state;
        dfu_status status;
        uint32_t waitTime = 0;
        uint8_t stringIdx = 0;

        int ret_libusb;
        unsigned char data[5] = {0x21};
        memcpy(&data[1], &address, 4);

        ret_libusb = download(dev_handle, 0, data, 5);
        if (ret_libusb >= LIBUSB_SUCCESS) {
            // Address pointer takes effect after GETSTATUS command
            ret_libusb = getStatus(dev_handle, &status, &waitTime, &state, &stringIdx);
            if ((ret_libusb >= LIBUSB_SUCCESS) && (status == DFU_STATUS_OK) && (state == DFU_STATE_DNBUSY)) {
                ret_libusb = getStatus(dev_handle, &status, &waitTime, &state, &stringIdx);
                if ((ret_libusb >= LIBUSB_SUCCESS) && (status == DFU_STATUS_OK)) {
                    ret_libusb = abort(dev_handle);
                    if (ret_libusb >= LIBUSB_SUCCESS)
                        ret_libusb = waitForState(dev_handle, DFU_STATE_DNLOAD_IDLE);
                }
            }
        }

        return ret_libusb;
    }

/**
 * returns the status, state and additional supporting dat of the DFU device. It is important to note that the Status and State are not synonymous and have distinct meanings.
 * @param dev_handle
 * @param status the status of the DFU device typically reflective of an error condition which likely will need to be resolved or cleared before further action can be taken.
 * @param delay a polling period (ms) reflective of an implied delay that should elapse before a subsequent, identical (of DFU_GETSTATUS) request is made.
 * @param state the state of the DFU device, usually reflective of a transition of type of in-progress event (such as initializing, uploading, etc).
 * @param i_string index of the status description in the descriptor string table
 * @return
 */
    int DFUDevice::getStatus(libusb_device_handle *dev_handle, dfu_status *status, uint32_t *delay, dfu_state *state, uint8_t *i_string) {
        int ret_libusb;
        uint8_t buf[6] = {0};

        ret_libusb = libusb_control_transfer(dev_handle, 0b10100001, 0x03, 0, 0, buf, 6, 100);

        *status = (dfu_status) buf[0];
        *delay = (buf[3] << 16) | (buf[2] << 8) | buf[1];
        *state = (dfu_state) buf[4];
        *i_string = buf[5];

        return ret_libusb;
    }

/**
 * Clears any current DFU_STATE_ERROR and corresponding DFU_STATUS, effectively resetting the state of the device, and returning it an IDLE state.
 * This method is only applicable/valid if the DFU state is currently in an ERROR state.  In order to reset back to an IDLE state under non-erorr
 * conditions, see DFUDevice::abort().
 * @param dev_handle
 * @return
 */
    int DFUDevice::clearStatus(libusb_device_handle *dev_handle) {
        return libusb_control_transfer(dev_handle, 0b00100001, 0x04, 0, 0, NULL, 0, 100);
    }

/**
 * Requests the current state of the DFU device, identical to the state reported by DFUDevice::getStatus().
 * @param dev_handle
 * @param buf
 * @return
 */
    int DFUDevice::getState(libusb_device_handle *dev_handle, dfu_state *state) {
        return libusb_control_transfer(dev_handle, 0b10100001, 0x05, 0, 0, (uint8_t *) state, 1, 100);
    }

/**
 * Aborts any current condition/sequence and returns to device to the DFU IDLE state.
 * @param dev_handle
 * @return
 */
    int DFUDevice::abort(libusb_device_handle *dev_handle) {
        return libusb_control_transfer(dev_handle, 0b00100001, 0x06, 0, 0, NULL, 0, 100);
    }

/**
 * Initiates a request to read len bytes of firmware image data from the device and store it into the memory address indicated by buf.
 * @param dev_handle
 * @param wValue a poin
 * @param buf
 * @param len
 * @return
 */
    int DFUDevice::upload(libusb_device_handle *dev_handle, uint16_t wValue, uint8_t *buf, uint16_t len) {
        return libusb_control_transfer(dev_handle, 0b10100001, 0x02, wValue, 0, buf, len, 100);
    }

/**
 * Initiates or signals the transfer of len bytes of data (*buf) to the DFU device, at the location specified by the last call to setAddress().
 * A len == 0 indicates that the transfer is complete, and represents the final payload packet of the download operation.
 * @param dev_handle
 * @param wValue
 * @param buf
 * @param len
 * @return
 */
    int DFUDevice::download(libusb_device_handle *dev_handle, uint16_t wValue, uint8_t *buf, uint16_t len) {
        return libusb_control_transfer(dev_handle, 0b00100001, 0x01, wValue, 0, buf, len, 100);
    }

/**
 * Initiates a "detach/attach sequence", effectively triggering a release from APP mode (when currently in APP mode) and enabling DFU mode, if a USB reset is
 * issues before the specified timeout period (ms) expires.
 * @param dev_handle
 * @param timeout
 * @return
 */
    int DFUDevice::detach(libusb_device_handle *dev_handle, uint8_t timeout) {
        return libusb_control_transfer(dev_handle, 0b00100001, 0x00, timeout, 0, NULL, 0, 100);
    }

/**
 * Issues a USB reset (not a hardware/power-on reset, but effectively a software reset) to the connected device causing the device to be re-enumerated in the host.
 * @param dev_handle
 * @return
 */
    int DFUDevice::reset(libusb_device_handle *dev_handle) {
        return libusb_reset_device(dev_handle);
    }

}