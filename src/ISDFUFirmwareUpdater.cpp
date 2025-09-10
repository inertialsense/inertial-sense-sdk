/**
 * @file ISDFUFirmwareUpdater.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 11/28/23.
 * @copyright Copyright (c) 2023 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDFUFirmwareUpdater.h"
#include <fstream>
#include <algorithm>

namespace dfu {

//    static const char* state_names[] = {
//        "APP_IDLE",
//        "APP_DETACH",
//        "IDLE",
//        "DNLOAD_SYNC",
//        "DNBUSY",
//        "DNLOAD_IDLE",
//        "MANIFEST_SYNC",
//        "MANIFEST",
//        "MANIFEST_WAIT_RESET",
//        "UPLOAD_IDLE",
//        "ERROR",
//    };

    std::mutex ISDFUFirmwareUpdater::dfuMutex;

    /**
     * Adds all discovered DFU devices, which match the specified VID/PID (if != 0) to the referenced devices vector.
     * @param devices a vectorof DFUDevice which contains all available/matching DFU devices
     * @param vid
     * @param pid
     * @return the number of dfu devices discovered (devices.size())
     */
    size_t ISDFUFirmwareUpdater::getAvailableDevices(std::vector<DFUDevice *> &devices, uint16_t vid, uint16_t pid) {
        libusb_device **device_list;
        libusb_device *dev;

        if(dfuMutex.try_lock())
        {
            libusb_init(NULL);

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
            libusb_exit(NULL);
            dfuMutex.unlock();
        }
        return devices.size();
    }

    /**
     * Removes any DFUDevice from devices which does not match the specified md5 fingerprint
     * @param devices vector of known devices
     * @param fingerprint the md5 digest "fingerprint" to match against (matches will be retained, all others will be removed)
     * @return the number of devices remaining in the vector (devices.size())
     */
    size_t ISDFUFirmwareUpdater::filterDevicesByFingerprint(std::vector<DFUDevice *> &devices, md5hash_t fingerprint) {
        auto removed = std::remove_if(std::begin(devices), std::end(devices), [&fingerprint](DFUDevice *d) { return !md5_matches(d->getFingerprint(), fingerprint); });
        devices.erase(removed, devices.end());
        return devices.size();
    }

    /**
     * Removes any DFUDevice from devices which does not match the specified fwUpdate::target_t type
     * @param devices vector of known devices
     * @param target the target type retain (all others will be removed)
     * @return the number of devices remaining in the vector (devices.size())
     */
    size_t ISDFUFirmwareUpdater::filterDevicesByTargetType(std::vector<DFUDevice *> &devices, fwUpdate::target_t target) {
        auto removed = std::remove_if(std::begin(devices), std::end(devices), [&target](DFUDevice *d) { return d->getTargetType() != target; });
        devices.erase(removed, devices.end());
        return devices.size();
    }

    /**
     * Checks if the specified device is a valid DFU device. Note that this method does not require an
     * active connection/interface to the device.
     * @param usbDevice libusb_device reference (not a handle) with which to perform the check
     * @param vid if not 0, will only return true if the DFU device also matches this Vender ID
     * @param pid if not 0, will only return true if the DFU device also matches this Product ID
     * @return
     */
    bool dfu::ISDFUFirmwareUpdater::isDFUDevice(libusb_device *usbDevice, uint16_t vid, uint16_t pid) {
        struct libusb_device_descriptor desc;
        struct libusb_config_descriptor *cfg;
        bool success = true;

        if (libusb_get_device_descriptor(usbDevice, &desc) < 0) {
            success = false;
        }

        // Check vendor and product ID
        else if (((vid != 0x0000) && (desc.idVendor != vid)) || ((pid != 0x0000) && (desc.idProduct != pid))) {
            success = false;      // must be some other usb device
        }

        else if (libusb_get_config_descriptor(usbDevice, 0, &cfg) < 0) {
            success = false;
        }

        // USB-IF DFU interface class numbers
        else if ((cfg->interface->altsetting[0].bInterfaceClass != 0xFE) ||
            (cfg->interface->altsetting[0].bInterfaceSubClass != 0x01) ||
            (cfg->interface->altsetting[0].bInterfaceProtocol != 0x02)) {
            success = false;
        }

        return success;
    }

    /**
     * Returns a fwUpdate-compatible target type (fwUpdate::target_t) appropriate for this DFU device,
     * given the parsed hardware id, where available.
     * @return determined fwUpdate::target_t is detectable, otherwise TARGET_UNKNOWN
     */
    fwUpdate::target_t DFUDevice::getTargetType() {
        switch (DECODE_HDW_TYPE(hardwareId)) {
            case IS_HARDWARE_TYPE_IMX:
                if ((DECODE_HDW_MAJOR(hardwareId) == 5) && (DECODE_HDW_MINOR(hardwareId) == 0)) return fwUpdate::TARGET_IMX5;
                // else if ((DECODE_HDW_MAJOR(hardwareId) == 5) && (DECODE_HDW_MINOR(hardwareId) == 1)) return fwUpdate::TARGET_IMX51;
                break;
            case IS_HARDWARE_TYPE_GPX:
                if ((DECODE_HDW_MAJOR(hardwareId) == 1) && (DECODE_HDW_MINOR(hardwareId) == 0)) return fwUpdate::TARGET_GPX1;
                break;
        }

        return fwUpdate::TARGET_UNKNOWN;
    }


    /**
     * high-level method to query and parse as much identifying information (USB, DFU, OTP, etc) as possible from the
     * associated USB device, to further populate this DFUDevice instance with details necessary for further operations.
     * @return
     */
    dfu_error DFUDevice::fetchDeviceInfo() {
        struct libusb_device_descriptor desc;
        struct libusb_config_descriptor *cfg;
        unsigned char str_buff[MAX_DESC_STR_LEN];

        if (!isConnected())
            open();

        if (libusb_claim_interface(usbHandle, 0) < LIBUSB_SUCCESS)
            return DFU_ERROR_DEVICE_BUSY;

        SLEEP_MS(100);
        // TODO make this IMX/GPX aware (OTP location maybe processor and/or product/device dependent)
        // Its important to note that if this is a BRAND NEW, NEVER BEEN PROGRAMMED STM32, it will have no OTP data. Even if we know the MCU type, we still don't know the application its in (ie, IMX-6 vs GPX-1).

        libusb_device *usb_device = libusb_get_device(usbHandle);
        if (libusb_get_device_descriptor(usb_device, &desc) != LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_LIBUSB;
        }

        //info.dfuSerial;
        vid = desc.idVendor;
        pid = desc.idProduct;
        usbDevice = usb_device;
        md5_init(fingerprint);

        // Get the serial number
        if (libusb_get_string_descriptor_ascii(usbHandle, desc.iSerialNumber, str_buff, sizeof(str_buff)) > LIBUSB_SUCCESS)
            dfuSerial = std::string((const char *) str_buff);

        // Get the product description
        if (libusb_get_string_descriptor_ascii(usbHandle, desc.iProduct, str_buff, sizeof(str_buff)) > LIBUSB_SUCCESS)
            dfuProduct = std::string((const char *) str_buff);

        // Get the manufacturer description
        if (libusb_get_string_descriptor_ascii(usbHandle, desc.iManufacturer, str_buff, sizeof(str_buff)) > LIBUSB_SUCCESS)
            dfuManufacturer = std::string((const char *) str_buff);

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

                        memset(&funcDescriptor, 0, sizeof(funcDescriptor));
                        if (findDescriptor(intf->extra, intf->extra_length, USB_DESCRIPTOR_DFU, &funcDescriptor, sizeof(funcDescriptor)) < 0) {
                            if (libusb_get_descriptor(usbHandle, USB_DESCRIPTOR_DFU, 0, (unsigned char *) &funcDescriptor, sizeof(funcDescriptor)) < 0) {
                                /* fake version 1.0 */
                                funcDescriptor.bLength = 7;
                                funcDescriptor.bcdDFUVersion = libusb_cpu_to_le16(0x0100);
                                // continue; // no valid functional descriptor
                            }
                        }

                        if (funcDescriptor.bLength == 7) {
                            funcDescriptor.bcdDFUVersion = libusb_cpu_to_le16(0x0100);
                        } else if (funcDescriptor.bLength < 9) {
                            // printf("Error obtaining DFU functional descriptor. Warning: Transfer size can not be detected\n");
                            funcDescriptor.bcdDFUVersion = libusb_cpu_to_le16(0x0100);
                            funcDescriptor.wTransferSize = 0;
                        }

                        bool dfu_mode = (intf->bInterfaceProtocol == 2);

                        /* e.g. DSO Nano has bInterfaceProtocol 0 instead of 2 */
                        if (funcDescriptor.bcdDFUVersion == 0x011a && intf->bInterfaceProtocol == 0)
                            dfu_mode = true;

                        /* LPC DFU bootloader has bInterfaceProtocol 1 (Runtime) instead of 2 */
                        if (desc.idVendor == 0x1fc9 && desc.idProduct == 0x000c && intf->bInterfaceProtocol == 1)
                            dfu_mode = true;

                        if (dfu_mode && (intf->iInterface != 0)) {
                            char alt_name[MAX_DESC_STR_LEN];
                            if (get_string_descriptor_ascii(intf->iInterface, alt_name, MAX_DESC_STR_LEN) > 0) {
                                dfuDescriptors.push_back(alt_name);
                                size_t lastPos = dfuDescriptors.size()-1;
                                decodeMemoryPageDescriptor(dfuDescriptors[lastPos], segments[lastPos]);
                            } else
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
        md5_update(fingerprint, (uint8_t *) &vid, sizeof(vid));
        md5_update(fingerprint, (uint8_t *) &pid, sizeof(pid));
        for (auto dfuDesc: dfuDescriptors) {
            md5_update(fingerprint, (uint8_t *) dfuDesc.c_str(), dfuDesc.size());
        }

        // TODO: make this work for both GPX-1 and IMX-6
        uint16_t hardwareType = 0;
        processorType = IS_PROCESSOR_UNKNOWN;

        if (md5_matches(fingerprint.state, DFU_FINGERPRINT_STM32L4)) processorType = IS_PROCESSOR_STM32L4; // possible IMX
        else if (md5_matches(fingerprint.state, DFU_FINGERPRINT_STM32U5)) processorType = IS_PROCESSOR_STM32U5; // possible GPX

        // try and read the OTP memory
        dfu_memory_t otp = segments[STM32_DFU_INTERFACE_OTP];
        if (otp.address != 0) {
            sn = -1; // 0xFFFFFFFF
            hardwareId = -1; // 0xFFFF

            uint8_t* rxBuf = new uint8_t[otp.pageSize] {0};
            int len = readMemory(otp.address, rxBuf, otp.pageSize); // otp.pageSize);
            if (len > 0) {
                otp_info_t *id = decodeOTPData(rxBuf, len);
                if (id != nullptr) {
                    sn = id->serialNumber;
                    hardwareType = id->hardwareId;
                }
            }
            delete [] rxBuf;
        } else {
            libusb_release_interface(usbHandle, 0);
            return DFU_ERROR_LIBUSB;
        }

        if ((hardwareId == 0xFFFF) && (processorType != IS_PROCESSOR_UNKNOWN)) {
            if (processorType == IS_PROCESSOR_STM32L4) hardwareType = IS_HARDWARE_TYPE_IMX; // only the IMX-5 uses the STM32L4
            else if (processorType == IS_PROCESSOR_STM32U5) hardwareType = IS_HARDWARE_TYPE_GPX; // TODO: Both the GPX-1 and IMX-6 will use the STM32U5 (at the moment)
        }

        // based on what we know so far, let's try and figure out a hardware type
        if ((hardwareType & 0xFFF0) == 0) {
            // if this is true, then we don't *really* know the hardware type or version (just the type)
            switch (hardwareType) {
                case IS_HARDWARE_TYPE_UINS:
                    hardwareId = ENCODE_HDW_ID(IS_HARDWARE_TYPE_UINS, 3, 2);
                    break;
                case IS_HARDWARE_TYPE_EVB:
                    hardwareId = ENCODE_HDW_ID(IS_HARDWARE_TYPE_EVB, 2, 0);
                    break;
                case IS_HARDWARE_TYPE_IMX:
                    hardwareId = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
                    break;
                case IS_HARDWARE_TYPE_GPX:
                    hardwareId = ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, 1, 0);
                    break;
            }
        }

        libusb_release_interface(usbHandle, 0);
        return DFU_ERROR_NONE;
    }

    /**
     * A utility function to parse additional IS-specific device data from the OTP memory.
     * This function doesn't read the OTP data, but simply parses the raw buffer, and returns
     * the last/most recent OTP data.
     * @param raw
     * @param len
     * @return a otp_info_t pointer into the raw pointer, of the last valid entry, or nullptr if the raw data is invalid.
     */
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
        dfu_error ret_dfu = DFU_ERROR_NONE;

        if (libusb_open(usbDevice, &usbHandle) < LIBUSB_SUCCESS)
            return DFU_ERROR_DEVICE_NOTFOUND;

        ret_libusb = libusb_claim_interface(usbHandle, 0);
        if (ret_libusb < LIBUSB_SUCCESS) {
            ret_dfu = DFU_ERROR_DEVICE_BUSY;
        } else {
            // Cancel any existing operations and return to IDLE state
            ret_libusb = abort();
            if (ret_libusb == LIBUSB_SUCCESS) {
                ret_libusb = waitForState(DFU_STATE_IDLE);
            }
            if (ret_libusb < LIBUSB_SUCCESS) {
                ret_dfu = DFU_ERROR_STATUS;
            }
        }

        if (ret_dfu != DFU_ERROR_NONE)
            libusb_release_interface(usbHandle, 0);

        return ret_dfu;
    }

    /**
     * High-level function used to program flash memory from a specified file. This function will ONLY
     * target the devices FLASH memory segment. The baseAddress must point to a location within the
     * flash memory, and that the size of the firmware image must not exceed the size of, or allow writing
     * outside of the FLASH memory segment. This function will erase existing flash memory before writing
     * the new firmware image.  Using the baseAddress allows writing of partitions or portions of data
     * into the firmware, allowing for example, to target one of multiple mcuBOOT slots, without overwriting
     * data in adjacent partitions.
     * @param filename the .hex or .bin file that should be programmed to the device
     * @param baseAddress this is the actual memory location which the firmware should be written to.
     * @return
     */
    dfu_error DFUDevice::updateFirmware(std::string filename, uint64_t baseAddress) {
        ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
        size_t image_sections;
        int ret_libusb;
        dfu_error ret_dfu;

        SLEEP_MS(100);

        if (!isConnected()) {
            ret_dfu = open();
            if (ret_dfu != DFU_ERROR_NONE)
                return ret_dfu;
        }

        std::string ext = ".hex";
        if (filename.compare(filename.length() - ext.length(), ext.length(), ext) == 0) {
            /**
             * Load the firmware's .hex file, parse its sections, and apply a base offset, if necessary
             */
            // Load the firmware image
            image_sections = ihex_load_sections(filename.c_str(), image, MAX_NUM_IHEX_SECTIONS);
            if (image_sections <= 0) {
                return DFU_ERROR_FILE_NOTFOUND;
            }

            // If baseAddress is not zero, then we will try and align the firmware to the specified base, otherwise we'll take it like it is (for better or for worse).
            if (baseAddress) {
                // If starting address is not the same as the baseAddress, calculate a new offset, and shift all the image sections accordingly
                uint64_t baseOffset = baseAddress - image[0].address;
                if (baseOffset) {
                    for (size_t i = 0; i < image_sections; image[i].address += baseOffset, i++);
                }
            }
        } else {
            std::ifstream file(filename, std::ios::binary);
            if (!file.is_open())
                return DFU_ERROR_FILE_NOTFOUND;

            image[0].address = (baseAddress ? baseAddress : segments[STM32_DFU_INTERFACE_FLASH].address);
            auto fsize = file.tellg();
            file.seekg( 0, std::ios::end );
            image[0].len = file.tellg() - fsize;
            file.seekg( 0, std::ios::beg );

            image[0].image = static_cast<uint8_t *>(malloc(image[0].len));
            file.read((char *)image[0].image, image[0].len);

            file.close();
            image_sections = 1;
        }

        uint32_t offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
        if (statusFn) {
            int lastSep = filename.find_last_of('/')+1;
            std::string fname(filename.substr(lastSep, filename.length() - lastSep));
            statusFn(this, IS_LOG_LEVEL_INFO, "(%s) Updating flash with firmware \"%s\" (@ 0x%08X)", getDescription(), fname.c_str(), segments[STM32_DFU_INTERFACE_FLASH].address + offset);
        }

        ret_libusb = abort();
        if (ret_libusb == LIBUSB_SUCCESS) {
            if (statusFn) {
                statusFn(this, IS_LOG_LEVEL_INFO, "(%s) Erasing flash memory...", getDescription());
            }

            offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
            for (size_t i = 0; i < image_sections; i++) {
                //offset = image[i].address + offset;
                ret_dfu = eraseFlash(segments[STM32_DFU_INTERFACE_FLASH], offset, image[i].len);
                if (ret_dfu != DFU_ERROR_NONE) {
                    statusFn(this, IS_LOG_LEVEL_ERROR, "(%s) Error erasing flash: %04x", getDescription(), -ret_dfu);
                    return ret_dfu;
                }
            }

            if (statusFn) {
                statusFn(this, IS_LOG_LEVEL_INFO, "(%s) Programming flash memory...", getDescription());
            }

            offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
            for (size_t i = 0; i < image_sections; i++) {
                ret_dfu = writeFlash(segments[STM32_DFU_INTERFACE_FLASH], offset, image[i].len, image[i].image);
                if (ret_dfu != DFU_ERROR_NONE) {
                    statusFn(this, IS_LOG_LEVEL_ERROR, "(%s) Error writing flash: %04x", getDescription(), -ret_dfu);
                    return ret_dfu;
                }
            }
        }

        // Unload the firmware image
        ihex_unload_sections(image, image_sections);
        return ret_dfu;
    }

    /**
     * High-level function used to program flash memory using the provided stream. This function will ONLY
     * target the devices FLASH memory segment. The baseAddress must point to a location within the
     * flash memory, and that the size of the firmware image must not exceed the size of, or allow writing
     * outside of the FLASH memory segment. This function will erase existing flash memory before writing
     * the new firmware image.  Using the baseAddress allows writing of partitions or portions of data
     * into the firmware, allowing for example, to target one of multiple mcuBOOT slots, without overwriting
     * data in adjacent partitions.  Note that this call does not handle any file format parsing/conversion;
     * the passed stream should backed by a byte-for-byte copy of the firmware image.
     * @param stream a binary stream of "raw" firmware image data that should be written to flash
     * @param baseAddress this is the actual memory location which the firmware should be written to.
     * @return
     */
    dfu_error DFUDevice::updateFirmware(std::istream& stream, uint64_t baseAddress) {
        ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
        size_t image_sections;
        int ret_libusb;
        dfu_error ret_dfu;

        SLEEP_MS(100);

        if (!isConnected()) {
            ret_dfu = open();
            if (ret_dfu != DFU_ERROR_NONE)
                return ret_dfu;
        }

        {
            if (!stream)
                return DFU_ERROR_FILE_NOTFOUND;


            image[0].address = (baseAddress ? baseAddress : segments[STM32_DFU_INTERFACE_FLASH].address);
            auto fsize = stream.tellg();
            stream.seekg( 0, std::ios::end );
            image[0].len = stream.tellg() - fsize;
            stream.seekg( 0, std::ios::beg );

            image[0].image = static_cast<uint8_t *>(malloc(image[0].len));
            stream.read((char *)image[0].image, image[0].len);

            //stream.close();
            image_sections = 1;
        }

        uint32_t offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
        if (statusFn) {
            statusFn(this, IS_LOG_LEVEL_INFO, "(%s) Updating flash with firmware \"%s\" (@ 0x%08X)", getDescription(), segments[STM32_DFU_INTERFACE_FLASH].address + offset);
        }

        ret_libusb = abort();
        if (ret_libusb == LIBUSB_SUCCESS) {
            if (statusFn) {
                statusFn(this, IS_LOG_LEVEL_INFO, "(%s) Erasing flash memory...", getDescription());
            }

            offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
            for (size_t i = 0; i < image_sections; i++) {
                //offset = image[i].address + offset;
                ret_dfu = eraseFlash(segments[STM32_DFU_INTERFACE_FLASH], offset, image[i].len);
                if (ret_dfu != DFU_ERROR_NONE) {
                    statusFn(this, IS_LOG_LEVEL_ERROR, "(%s) Error erasing flash: %04x", getDescription(), -ret_dfu);
                    return ret_dfu;
                }
            }

            if (statusFn) {
                statusFn(this, IS_LOG_LEVEL_INFO, "(%s) Programming flash memory...", getDescription());
            }

            offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
            for (size_t i = 0; i < image_sections; i++) {
                ret_dfu = writeFlash(segments[STM32_DFU_INTERFACE_FLASH], offset, image[i].len, image[i].image);
                if (ret_dfu != DFU_ERROR_NONE) {
                    statusFn(this, IS_LOG_LEVEL_ERROR, "(%s) Error writing flash: %04x", getDescription(), -ret_dfu);
                    return ret_dfu;
                }
            }
        }

        // Unload the firmware image
        ihex_unload_sections(image, image_sections);
        return ret_dfu;
    }

    /**
     * Erases one or more pages of flash memory on the device.
     * @param address the address of the flash page (must be aligned to the page boundary)
     * @param data_len the number of bytes to erase.  This can extend beyond a single page. Note that you cannot erase a partial page. If data_len exceeds the bounds of a page, the entire next page is also erased.
     * @return
     */
    dfu_error DFUDevice::eraseFlash(const dfu_memory_t& mem, uint32_t& offset, uint32_t data_len)
    {
        /**
         * Prepare the device's FLASH by first erasing the flash contents
         */
        int ret_libusb;
        dfu_error ret_dfu;
        dfu_state state;

        if ((ret_dfu = prepAndValidateBeforeDownload(mem.address + offset, data_len)) < DFU_ERROR_NONE)
            return ret_dfu;

        if (data_len == 0)
            return DFU_ERROR_NONE; // nothing to do

        // Erase memory
        uint32_t bytes_erased = 0;
        uint32_t byteInSection = 0;
        do {
            uint32_t pageAddress = mem.address + offset;
            uint8_t eraseCommand[5];

            eraseCommand[0] = 0x41;
            memcpy(&eraseCommand[1], &pageAddress, 4);

            dlBlockNum = 0; // Erase Flash commands are ALWAYS sent with a 0 wValue/wBlockNum
            ret_libusb = download(dlBlockNum, eraseCommand, 5);
            if (ret_libusb < LIBUSB_SUCCESS)
                return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

            ret_libusb = waitForState(DFU_STATE_DNLOAD_IDLE, &state);
            if (ret_libusb < LIBUSB_SUCCESS)
                return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

            byteInSection += mem.pageSize;
            bytes_erased += mem.pageSize;
            offset += mem.pageSize;

            if (progressFn) {
                float progress = (float) bytes_erased / (float) data_len;
                progressFn(this, "ERASING", 1, 2, progress);
            }
        } while (byteInSection < data_len - 1);

        return DFU_ERROR_NONE;
    }

    /**
     * Writes an arbitrary amount of data into flash memory on the DFU device.
     * @param mem the memory segment to which the flash should be written
     * @param offset the offset into the memory segment where this data should be written
     * @param data_len the number of bytes to write
     * @param data the data to be written
     * @return
     */
    dfu_error DFUDevice::writeFlash(const dfu_memory_t& mem, uint32_t& offset, uint32_t data_len, uint8_t *data) {
        dfu_error ret_dfu = DFU_ERROR_NONE;
        // uint16_t blockNum = 0;
        int ret_libusb = LIBUSB_SUCCESS;

        if ((ret_dfu = prepAndValidateBeforeDownload(mem.address + offset, data_len)) < DFU_ERROR_NONE)
            return ret_dfu;

        if (data_len == 0)
            return DFU_ERROR_NONE; // nothing to do

        // Write memory
        uint32_t bytes_written = 0;
        uint32_t byteInSection = 0;
        uint8_t* payload = new uint8_t[mem.pageSize];

        if (progressFn) {
            float progress = (float) bytes_written / (float) data_len;
            progressFn(this, "WRITING", 2, 2, progress);
        }

        do {
            uint32_t payloadLen = mem.pageSize;
            uint32_t bytesRemaining = data_len - byteInSection;
            if (payloadLen > bytesRemaining)
                payloadLen = bytesRemaining;

            // Set write address
            ret_libusb = setAddress(dlBlockNum, mem.address + offset);
            if (ret_libusb < LIBUSB_SUCCESS) {
                return (dfu_error) (DFU_ERROR_LIBUSB | (ret_libusb << 16));
            }

            // Copy image into buffer for transmission
            memset(payload, 0xFF, mem.pageSize);
            memcpy(payload, &data[byteInSection], payloadLen);

            //blockNum = (uint16_t) (byteInSection / mem.pageSize) + wValue;

            ret_libusb = download(dlBlockNum, payload, payloadLen);
            if (ret_libusb < LIBUSB_SUCCESS) {
                ret_dfu = (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));
                break;
            }

            ret_libusb = waitForState(DFU_STATE_DNLOAD_IDLE);
            if (ret_libusb < LIBUSB_SUCCESS) {
                ret_dfu = (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));
                break;
            }

            byteInSection += payloadLen;
            bytes_written += payloadLen;
            offset += mem.pageSize;

            if (progressFn) {
                float progress = (float) bytes_written / (float) data_len;
                progressFn(this, "WRITING", 2, 2, progress);
            }
        } while (byteInSection < data_len - 1);

        delete [] payload;
        return ret_dfu;
    }

    /**
     * Performs any finalization procedures necessary for DFU device to successfully complete its update. This
     * typically results in a reset of the device, but is also generally device specific and therefor is not
     * guaranteed to initiate a reset.
     * @return
     */
    dfu_error DFUDevice::finalizeFirmware() {
        int ret_libusb;
        dfu_error ret_dfu;
        dfu_state state;

        if (!isConnected()) {
            if ((ret_dfu = open()) < DFU_ERROR_NONE)
                return ret_dfu;
        }

        if (statusFn) {
            statusFn(this, IS_LOG_LEVEL_INFO, "(%s) Finalizing DFU programming...", getDescription());
        }

        if (processorType == IS_PROCESSOR_STM32L4) {
            // FIXME: This should be handled as a call into a virtual function in a "device/processor-specific" derived class

            // Option bytes
            // This hard-coded array sets mostly defaults, but without PH3 enabled and
            // with DFU mode disabled. Application will enable DFU mode if needed.
            uint8_t bytes[] = {
                    0xaa, 0xf8, 0xff, 0xfb,  0x55, 0x07, 0x00, 0x04,  0xff, 0xff, 0xff, 0xff,  0x00, 0x00, 0x00, 0x00,  
                    0x00, 0x00, 0xff, 0xff,  0xff, 0xff, 0x00, 0x00,  0xff, 0xff, 0x00, 0xff,  0x00, 0x00, 0xff, 0x00,  
                    0xff, 0xff, 0x00, 0xff,  0x00, 0x00, 0xff, 0x00
            };

            // return writeFlash(segments[STM32_DFU_INTERFACE_OPTIONS], 0, bytes, sizeof(bytes));

            if ((getState(&state) == LIBUSB_SUCCESS) && (state != DFU_STATE_DNLOAD_IDLE) && (state != DFU_STATE_IDLE)) {
                abort(); // We were doing something else, but not any more... Cancel any existing operations, and return to a good, known state
                ret_libusb = waitForState(DFU_STATE_IDLE);
                if (ret_libusb < LIBUSB_SUCCESS)
                    return (dfu_error) (DFU_ERROR_LIBUSB | (ret_libusb << 16));
            }

            ret_libusb = setAddress(dlBlockNum, segments[STM32_DFU_INTERFACE_OPTIONS].address);
            if (ret_libusb < LIBUSB_SUCCESS)
                return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

            // STM32 DFU specs will reset the device immediately after writing to the Option Bytes
            ret_libusb = download(dlBlockNum, bytes, sizeof(bytes));
            if (ret_libusb < LIBUSB_SUCCESS)
                return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

            // if there wasn't an error, the device just restarted, and we have no indication of an error, so it must be OK!
            return DFU_ERROR_NONE;

        } else if (processorType == IS_PROCESSOR_STM32U5) {

            // Option bytes - Address: 0x40022040
            // This hard-coded array sets mostly defaults, but without PH3 enabled and
            // with DFU mode disabled. Application will enable DFU mode if needed.
            uint8_t bytes[] = {
                    0xaa, 0xf8, 0xef, 0x1B,  0x7f, 0x00, 0x00, 0x08,  0x7f, 0x00, 0xf9, 0x0B,  0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  0xff, 0xff, 0x80, 0xff,  0xff, 0xff, 0x80, 0xff,
                    0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00
            };            

            if ((getState(&state) == LIBUSB_SUCCESS) && (state != DFU_STATE_DNLOAD_IDLE)) {            // Cancel any existing operations
                ret_libusb = abort();
                if (ret_libusb < LIBUSB_SUCCESS)
                    return (dfu_error) (DFU_ERROR_LIBUSB | (ret_libusb << 16));

                // Reset status to good
                ret_libusb = waitForState(DFU_STATE_IDLE);
                if (ret_libusb < LIBUSB_SUCCESS)
                    return (dfu_error) (DFU_ERROR_LIBUSB | (ret_libusb << 16));
            }

            ret_libusb = setAddress(dlBlockNum, segments[STM32_DFU_INTERFACE_OPTIONS].address);
            if (ret_libusb < LIBUSB_SUCCESS)
                return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

            // STM32 DFU specs will reset the device immediately after writing to the Option Bytes
            ret_libusb = download(dlBlockNum, bytes, sizeof(bytes));
            if (ret_libusb < LIBUSB_SUCCESS)
                return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

            // if there wasn't an error, the device just restarted, and we have no indication of an error, so it must be OK!
            return DFU_ERROR_NONE;
        }

        // Wait for the drop to the MANIFEST-SYNC state
        ret_libusb = waitForState(DFU_STATE_MANIFEST_SYNC, &state);
        if (state == DFU_STATE_APP_IDLE) {
            // if we immediately fall back to APP-IDLE, then we're done.  Just return success;
            reset();
            return DFU_ERROR_NONE;
        }

        if (ret_libusb < LIBUSB_SUCCESS)
            return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

        // Wait for the drop to the MANIFEST state
        ret_libusb = waitForState(DFU_STATE_MANIFEST, &state);
        if (ret_libusb < LIBUSB_SUCCESS)
            return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

        ret_libusb = waitForState(DFU_STATE_MANIFEST_WAIT_RESET, &state);
        if (ret_libusb < LIBUSB_SUCCESS)
            return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));

        // At this point, there is nothing left to due but reset
        detach(100);
        reset();

        return DFU_ERROR_NONE;
    }

    /**
     * Closes the DFU/USB device, by cancelling any active DFU operations, and waiting for the DFU IDLE state
     * before releasing the USB interface.
     * @return
     */
    dfu_error DFUDevice::close() {
        int ret_libusb;

        // Cancel any existing operations
        ret_libusb = abort();
        if (ret_libusb < LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));
        }

        // Reset status to good
        ret_libusb = waitForState(DFU_STATE_IDLE);
        if (ret_libusb < LIBUSB_SUCCESS) {
            libusb_release_interface(usbHandle, 0);
            return (dfu_error)(DFU_ERROR_LIBUSB | (ret_libusb << 16));
        }

        libusb_release_interface(usbHandle, 0);

        return DFU_ERROR_NONE;
    }

    /**
     * Produces a human-readable, unique identifier for this device
     * @return
     */
    const char *DFUDevice::getDescription() {
        static char buff[64];
        if (sn != 0xFFFFFFFF)
            sprintf(buff, "%s-%d.%d:SN-%05d", g_isHardwareTypeNames[DECODE_HDW_TYPE(hardwareId)], DECODE_HDW_MAJOR(hardwareId), DECODE_HDW_MINOR(hardwareId), (sn != 0xFFFFFFFF ? sn : 0));
        else
            sprintf(buff, "%s-%d.%d:DFU-%s", g_isHardwareTypeNames[DECODE_HDW_TYPE(hardwareId)], DECODE_HDW_MAJOR(hardwareId), DECODE_HDW_MINOR(hardwareId), dfuSerial.c_str());
        return buff;
    }

    /**
     * Ensures the specified address and data length are valid, and that the LIBUSB DFU state is suitable to
     * begin a DOWNLOAD operation (transfer to device). This does NOT initiate the download, but will cancel
     * operations which could prevent a download from starting correctly.
     * @param address
     * @param data_len
     * @return
     */
    dfu_error DFUDevice::prepAndValidateBeforeDownload(uint32_t address, uint32_t data_len) {
        int ret_libusb = LIBUSB_SUCCESS;
        dfu_state state = DFU_STATE_IDLE;
        dfu_error ret_dfu = DFU_ERROR_NONE;

        if (!isConnected()) {
            if ((ret_dfu = open()) < DFU_ERROR_NONE)
                return ret_dfu;
        }

        if (address & STM32_PAGE_ERROR_MASK)
            return DFU_ERROR_INVALID_ARG;

        // make sure we're in a good state
        if ((getState(&state) == LIBUSB_SUCCESS) && (state != DFU_STATE_IDLE) && (state != DFU_STATE_DNLOAD_IDLE)) {
            ret_libusb = abort();
            ret_libusb = waitForState(DFU_STATE_IDLE);
            if (ret_libusb < LIBUSB_SUCCESS) {
                return (dfu_error) (DFU_ERROR_LIBUSB | (ret_libusb << 16));
            }
        }

        return DFU_ERROR_NONE;
    }

    /**
     * Similar to libusb_get_string_descriptor_ascii but will allow truncated descriptors (descriptor length mismatch) seen
     * on e.g. the STM32F427 ROM bootloader.
     * @param desc_index descriptor index to return
     * @param data memory buffer to store the descriptor in
     * @param length the maximum length of buffer; if the string is larger than length, only length bytes will be copied
     * @return if >= 0, indicated success and returns the copied length of the string (<= length), otherwise a negative value indicates a LIBUSB_ error
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

    /**
     * Look for a descriptor in a concatenated descriptor list. Will return upon the first match of the given descriptor type.
     * @param desc_list
     * @param list_len
     * @param desc_type
     * @param res_buf
     * @param res_size
     * @return length of found descriptor, limited to res_size
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

    /**
     * Decodes the STM32 alternate ID descriptor, which defines accessible memory regions, their size and type
     * @param altSetting
     * @param segment
     * @return
     */
    int DFUDevice::decodeMemoryPageDescriptor(const std::string& altSetting, dfu_memory_t& segment) {

        if (altSetting[0] != '@')
            return -1;  // all STM32 alternate descriptors start with an '@'; if this doesn't have that, let's stop now

        std::vector<std::string> params;
        splitString(altSetting, '/', params);

        // pop the first entry (descriptor name), everything afterwards are pairs of "address/page" which are parsed together
        std::string descriptor = params[0];
        params.erase(params.begin());

        for (auto it = begin(params); it != end(params); it++) {
            if (it == end(params)) {
                // awkward; there isn't a matching address/page info pair describing the memory segment - just give up now.
                break;
            }
            segment.address = strtoll(it->c_str(), nullptr, 16);

            it++;
            std::vector<std::string> segmentParams;
            splitString(it->c_str(), '*', segmentParams);

            segment.pages = strtol(segmentParams[0].c_str(), nullptr, 10);
            char *pageFlags = nullptr;
            segment.pageSize = strtol(segmentParams[1].c_str(), &pageFlags, 10);
            switch (pageFlags[0]) {
                case 'M':
                    segment.pageSize *= (1024 * 1024);
                    break;
                case 'K':
                    segment.pageSize *= 1024;
                    break;
                case 'B':
                case ' ':
                    break; // no multiplier
            }
            segment.pageType = pageFlags[1];

            // TODO: implements support to parse multiple pairs - right now, we just parse the first pair and then bail out
            break;
        }
        return 0;
    }

    /**
     * Reads n-bytes of data from the DFU device, starting at memloc address, and stores in the references buffer
     * @param memloc
     * @param rxBuf
     * @param rxLen
     * @return
     */
    int DFUDevice::readMemory(uint32_t memloc, uint8_t *rxBuf, size_t rxLen) {
        int ret_libusb;
        uint8_t stringIdx;

        uint32_t waitTime = 0;
        dfu_status status;
        dfu_state state;
        int bytes_read;

        // Cancel any existing operations
        ret_libusb = abort();
        if (ret_libusb >= LIBUSB_SUCCESS) {
            ret_libusb = waitForState(DFU_STATE_IDLE, &state);
            if (ret_libusb >= LIBUSB_SUCCESS) {
                ret_libusb = setAddress(ulBlockNum, memloc);
                if (ret_libusb >= LIBUSB_SUCCESS) {
                    // drop out of DFU_STATE_DNLOAD_IDLE, and get back to DFU_STATE_IDLE before we 'upload'
                    ret_libusb = abort();
                    ret_libusb = waitForState(DFU_STATE_IDLE, &state);
                    // Read the full requested memory
                    if (ret_libusb >= LIBUSB_SUCCESS) {
                        ret_libusb = upload(ulBlockNum, rxBuf, rxLen);
                        if (ret_libusb >= LIBUSB_SUCCESS) {
                            bytes_read = ret_libusb;
                            ret_libusb = getStatus(&status, &waitTime, &state, &stringIdx);
                            if (ret_libusb >= LIBUSB_SUCCESS)
                                return bytes_read;
                        }
                    }
                }
            }
        }

        return ret_libusb;
    }


/**
 * Enters a blocking state while waiting for the USB DFU device to enter a particular state.
 * This implementation has a fixed timeout(10ms)/retry(5x) cycle and should never block for more than ~50ms.
 * @param required_state
 * @param actual_state if not null, the final state will be returned (useful in the event of a timeout).
 * @return DFU_ERROR_NONE if state watches the required_state, otherwise will return DFU_ERROR_TIMEOUT if the timeout condition occurs first.
 */
    int DFUDevice::waitForState(dfu_state required_state, dfu_state* actual_state) {
        dfu_status status = DFU_STATUS_ERR_UNKNOWN;
        uint32_t waitTime = 0;
        dfu_state state;
        uint8_t stringIndex;
        int ret_libusb = 0;
        uint8_t tryCounter = 0;

        if (actual_state == nullptr)
            actual_state = &state;

        do {
            ret_libusb = getStatus(&status, &waitTime, actual_state, &stringIndex);
            if (status != DFU_STATUS_OK) {
                clearStatus();
            }
            if (*actual_state != required_state) {
                if (tryCounter++ > 5)
                    return ret_libusb;

                SLEEP_MS(_MAX(waitTime, 25));
            }
        } while ((status != DFU_STATUS_OK) || (*actual_state != required_state));

        return LIBUSB_SUCCESS;
    }

/**
 * Directs the DFU device to read/write memory from the specified address location (this is not a pointer!)
 * @param address
 * @return
 */
    int DFUDevice::setAddress(uint16_t& wValue, uint32_t address) {
        dfu_state state;
        dfu_status status;
        uint32_t waitTime = 0;
        uint8_t stringIdx = 0;

        int ret_libusb;
        unsigned char data[5] = {0x21};
        memcpy(&data[1], &address, 4);

        wValue = 0; // a "Set Address" must always have a wValue/wBlockNum of 0
        ret_libusb = download(wValue, data, 5);
        if (ret_libusb >= LIBUSB_SUCCESS) {
            // Address pointer takes effect after GETSTATUS command
            ret_libusb = getStatus(&status, &waitTime, &state, &stringIdx);
            if ((ret_libusb >= LIBUSB_SUCCESS) && (status == DFU_STATUS_OK) && (state == DFU_STATE_DNBUSY)) {
                ret_libusb = getStatus(&status, &waitTime, &state, &stringIdx);
                if ((ret_libusb >= LIBUSB_SUCCESS) && (status == DFU_STATUS_OK)) {
                    ret_libusb = abort();
                    if (ret_libusb >= LIBUSB_SUCCESS)
                        ret_libusb = waitForState(DFU_STATE_DNLOAD_IDLE);
                }
            }
            if (wValue == 1) wValue++; // STM32 reserves wValue/wBlockNum = 1 for DFU_UPLOAD and DFU_DNLOAD for future use (so increment past it)
        }

        return ret_libusb;
    }

/**
 * returns the status, state and additional supporting dat of the DFU device. It is important to note that the Status and State are not synonymous and have distinct meanings.
 * @param status the status of the DFU device typically reflective of an error condition which likely will need to be resolved or cleared before further action can be taken.
 * @param delay a polling period (ms) reflective of an implied delay that should elapse before a subsequent, identical (of DFU_GETSTATUS) request is made.
 * @param state the state of the DFU device, usually reflective of a transition of type of in-progress event (such as initializing, uploading, etc).
 * @param i_string index of the status description in the descriptor string table
 * @return
 */
    int DFUDevice::getStatus(dfu_status *status, uint32_t *delay, dfu_state *state, uint8_t *i_string) {
        int ret_libusb;
        uint8_t buf[6] = {0};

        ret_libusb = libusb_control_transfer(usbHandle, 0b10100001, 0x03, 0, 0, buf, 6, 100);

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
    int DFUDevice::clearStatus() {
        return libusb_control_transfer(usbHandle, 0b00100001, 0x04, 0, 0, NULL, 0, 100);
    }

/**
 * Requests the current state of the DFU device, identical to the state reported by DFUDevice::getStatus().
 * @param buf
 * @return
 */
    int DFUDevice::getState(dfu_state *state) {
        return libusb_control_transfer(usbHandle, 0b10100001, 0x05, 0, 0, (uint8_t *) state, 1, 100);
    }

/**
 * Aborts any current condition/sequence and returns to device to the DFU IDLE state.
 * @return
 */
    int DFUDevice::abort() {
        return libusb_control_transfer(usbHandle, 0b00100001, 0x06, 0, 0, NULL, 0, 100);
    }

/**
 * Initiates a request to read len bytes of firmware image data from the device and store it into the memory address indicated by buf.
 * @param wValue a poin
 * @param buf
 * @param len
 * @return
 */
    int DFUDevice::upload(uint16_t& wValue, uint8_t *buf, uint16_t len) {

        int ret_libusb = LIBUSB_SUCCESS;
        int bytesRemain = len, bytesReceived = 0;
        do {
            int bytesToReceive = (bytesRemain > funcDescriptor.wTransferSize) ? funcDescriptor.wTransferSize : bytesRemain;
            ret_libusb = libusb_control_transfer(usbHandle, 0b10100001, 0x02, wValue, 0, buf + bytesReceived, bytesToReceive, 100);
            // ret_libusb = libusb_control_transfer(usbHandle, 0b00100001, 0x01, wValue, 0, buf + bytesSent, bytesToSend, 100);
            if (ret_libusb >= LIBUSB_SUCCESS) {
                bytesReceived += ret_libusb; // setup for next block transfer
                bytesRemain -= ret_libusb;
                wValue++;

                if (len != 0) // if len == 0, we expect to fall into the MANIFEST cycle
                    ret_libusb = waitForState(DFU_STATE_UPLOAD_IDLE);
            }
        } while ((bytesRemain > 0) && (ret_libusb >= LIBUSB_SUCCESS));
        return (ret_libusb >= LIBUSB_SUCCESS) ? bytesReceived : ret_libusb;
    }

/**
 * Initiates or signals the transfer of len bytes of data (*buf) to the DFU device, at the location specified by the last call to setAddress().
 * A len == 0 indicates that the transfer is complete, and represents the final payload packet of the download operation.
 * @param wValue
 * @param buf
 * @param len
 * @return
 */
    int DFUDevice::download(uint16_t& wValue, uint8_t *buf, uint16_t len) {
        dfu_state state = DFU_STATE_IDLE;
        int ret_libusb = LIBUSB_SUCCESS;
        int bytesRemain = len, bytesSent = 0;
        do {
            int bytesToSend = (bytesRemain > funcDescriptor.wTransferSize) ? funcDescriptor.wTransferSize : bytesRemain;
            ret_libusb = libusb_control_transfer(usbHandle, 0b00100001, 0x01, wValue, 0, buf + bytesSent, bytesToSend, 100);
            if (ret_libusb >= LIBUSB_SUCCESS) {
                bytesSent += ret_libusb; // setup for next block transfer
                bytesRemain -= ret_libusb;
                wValue++;

                if (len != 0) { // if len == 0, we expect to fall into the MANIFEST cycle
                    ret_libusb = waitForState(DFU_STATE_DNLOAD_IDLE, &state);
                    if (state == DFU_STATE_APP_IDLE)
                        ret_libusb = LIBUSB_SUCCESS;
                }
            }
        } while ((bytesRemain > 0) && (ret_libusb >= LIBUSB_SUCCESS));
        return (ret_libusb >= LIBUSB_SUCCESS) ? bytesSent : ret_libusb;
    }

/**
 * Initiates a "detach/attach sequence", effectively triggering a release from APP mode (when currently in APP mode) and enabling DFU mode, if a USB reset is
 * issues before the specified timeout period (ms) expires.
 * @param timeout
 * @return
 */
    int DFUDevice::detach(uint8_t timeout) {
        return libusb_control_transfer(usbHandle, 0b00100001, 0x00, timeout, 0, NULL, 0, 100);
    }

/**
 * Issues a USB reset (not a hardware/power-on reset, but effectively a software reset) to the connected device causing the device to be re-enumerated in the host.
 * @return
 */
    int DFUDevice::reset() {
        return libusb_reset_device(usbHandle);
    }

}
