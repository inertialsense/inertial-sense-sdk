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
#include <chrono>

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

void ISDFUFirmwareUpdater::initLibUSB() {
    libusb_init(NULL);
    log_info(IS_LOG_FWUPDATE, "DFU: libusb initialized");
}

void ISDFUFirmwareUpdater::exitLibUSB() {
    libusb_exit(NULL);
    log_info(IS_LOG_FWUPDATE, "DFU: libusb shutdown");
}

const char *DFUDevice::dfuDeviceErrors[] = {
        "SUCCESS",
        "DEVICE_NOT_FOUND",
        "DEVICE_BUSY",
        "DEVICE_TIMEOUT",
        "LIBUSB_ERROR",
        "INVALID_STATUS",
        "INVALID_ARGUMENT",
        "FILE_NOT_FOUND",
        "INVALID_IMAGE",
        "RDP_LOCKED",               // -DFU_ERROR_RDP_LOCKED (9)
        "RDP_PERMANENT_LOCKED",     // -DFU_ERROR_RDP_PERMANENT_LOCKED (10)
        "WRITE_VERIFY_FAILED",      // -DFU_ERROR_WRITE_VERIFY_FAILED (11)
};

/**
 * Returns the human-readable name for an error index. The expected argument is the positive index
 * (i.e. -dfu_error); callers conventionally pass getErrorName(-result). Out-of-range values (including
 * packed libusb errors that don't map to a base code) return "UNKNOWN" rather than reading out of bounds.
 */
const char *DFUDevice::getErrorName(int errNo) {
    static const int errCount = (int)(sizeof(dfuDeviceErrors) / sizeof(dfuDeviceErrors[0]));
    if (errNo < 0 || errNo >= errCount)
        return "UNKNOWN";
    return dfuDeviceErrors[errNo];
}

/**
 * SN-8043: pure RDP-byte -> verdict mapping. See header for semantics. Kept free of any USB/device
 * state so it can be exercised directly by unit tests (tests/test_ISDFUFirmwareUpdater.cpp).
 */
dfu_error DFUDevice::rdpVerdict(uint8_t rdpByte) {
    if (rdpByte == 0xAA)
        return DFU_ERROR_NONE;                  // Level 0 - unprotected, programming allowed
    if (rdpByte == 0xCC)
        return DFU_ERROR_RDP_PERMANENT_LOCKED;  // Level 2 - permanent, unrecoverable
    return DFU_ERROR_RDP_LOCKED;                // Level 1 - flash writes silently dropped
}

/**
 * SN-8193: see header. Writing the FLASH Option Bytes makes the STM32 reset immediately, so the
 * USB device drops off the bus mid-transfer; the resulting disconnect-class libusb error is the
 * expected, successful end of finalize. Kept free of any USB/device state so it can be exercised
 * directly by unit tests (tests/test_ISDFUFirmwareUpdater.cpp).
 */
bool DFUDevice::isExpectedOptionByteResetError(int libusbError) {
    switch (libusbError) {
        case LIBUSB_ERROR_NO_DEVICE:    // device left the bus (most common)
        case LIBUSB_ERROR_IO:           // transfer torn down by the reset
        case LIBUSB_ERROR_PIPE:         // endpoint stalled as the device went away
            return true;
        default:
            return false;
    }
}

/**
 * SN-8193: see header. Thin wrapper over libusb's own name lookup; pure so it can be unit-tested.
 */
const char *DFUDevice::libusbErrorName(int libusbCode) {
    return libusb_error_name(libusbCode);
}

/**
 * SN-8193: see header. Records the libusb code for diagnostics and returns the DFU_ERROR_LIBUSB tag.
 * Replaces the old `DFU_ERROR_LIBUSB | (code << 16)` packing, which discarded the code entirely.
 */
dfu_error DFUDevice::libusbError(int libusbCode) {
    lastLibusbError = libusbCode;
    return DFU_ERROR_LIBUSB;
}


/**
 * Adds all discovered DFU devices, which match the specified VID/PID (if != 0), to the referenced devices vector.
 * @param devices       a vector of DFUDevice which receives all available/matching DFU devices
 * @param vid           USB vendor id filter (0 = any)
 * @param pid           USB product id filter (0 = any)
 * @param onDeviceFound optional callback invoked per device as it is identified (see header)
 * @return the number of dfu devices discovered (devices.size())
 */
size_t ISDFUFirmwareUpdater::getAvailableDevices(std::vector<DFUDevice *> &devices, uint16_t vid, uint16_t pid,
                                                 pfnDfuDeviceFoundCb onDeviceFound) {
    libusb_device **device_list;
    libusb_device *dev;

    // Value-type identities collected under dfuMutex and dispatched to onDeviceFound *after* the lock is
    // released (see below). One per enumerated DFU device, in enumeration order.
    std::vector<DfuDeviceInfo> foundInfos;

    {
        std::lock_guard<std::mutex> lock(dfuMutex);

        size_t device_count = libusb_get_device_list(NULL, &device_list);
        log_info(IS_LOG_FWUPDATE, "DFU getAvailableDevices: libusb found %zu USB device(s), filtering for VID=%04X PID=%04X",
                 device_count, vid, pid);

        int dfuCount = 0;
        for (size_t i = 0; i < device_count; ++i) {
            dev = device_list[i];

            if (isDFUDevice(dev, vid, pid)) {
                dfuCount++;
                libusb_ref_device(dev);  // prevent dangling pointer after free_device_list
                DFUDevice *dfuDevice = new DFUDevice(dev);
                devices.push_back(dfuDevice);

                if (onDeviceFound) {
                    // Snapshot a value-type identity (no handle/ownership) for each device as it's
                    // identified -- lets a UI populate progressively when this runs on a worker thread.
                    // Deferred to the dispatch loop below so the callback never runs under dfuMutex.
                    DfuDeviceInfo info;
                    const char *desc = dfuDevice->getDescription();
                    info.description    = desc ? desc : "";
                    info.typeName       = DFUDevice::getDeviceTypeName(dfuDevice->getProcessorType(), dfuDevice->getTotalFlashSize());
                    info.usbSerial      = dfuDevice->getUsbSerial();
                    info.serialNo       = dfuDevice->getSerialNo();
                    info.hardwareId     = dfuDevice->getHardwareId();
                    info.processorType  = dfuDevice->getProcessorType();
                    info.totalFlashSize = dfuDevice->getTotalFlashSize();
                    foundInfos.push_back(info);
                }
            }
        }

        libusb_free_device_list(device_list, 1);

        log_info(IS_LOG_FWUPDATE, "DFU getAvailableDevices: found %d DFU device(s), %zu enumerated successfully",
                 dfuCount, devices.size());
    }

    // Invoke onDeviceFound OUTSIDE dfuMutex. The callback runs arbitrary caller code and, on a UI worker
    // thread, may re-enter DFU APIs that take this same (non-recursive) mutex -- holding the lock across
    // it would risk deadlock and tie the mutex hold time to callback latency. The identities are value
    // snapshots, safe to deliver after the libusb device list has been freed.
    for (const DfuDeviceInfo &info : foundInfos)
        onDeviceFound(info);

    return devices.size();
}

/**
 * Returns the number of DFU devices currently connected, without opening or classifying them.
 * Uses the same mutex as getAvailableDevices() for consistent access.
 */
int ISDFUFirmwareUpdater::getNumDevices(uint16_t vid, uint16_t pid) {
    int count = 0;
    // No mutex needed — libusb enumeration is thread-safe, and we only read descriptors here.
    libusb_device **device_list;
    size_t device_count = libusb_get_device_list(NULL, &device_list);
    for (size_t i = 0; i < device_count; ++i) {
        if (isDFUDevice(device_list[i], vid, pid))
            count++;
    }
    libusb_free_device_list(device_list, 1);
    return count;
}

// ---- DFU discovery state machine (step-driven; no internal thread) --------------------------------

/**
 * One poll of the count-based settle state machine. The caller owns ctx and drives the cadence
 * (passing elapsedMs since the previous call), so the SDK keeps no thread and no clock. See the header
 * for the full contract.
 * @param ctx       caller-owned discovery context (configuration in, state/count out)
 * @param elapsedMs time since the previous call
 * @return true iff the state changed this step
 */
bool ISDFUFirmwareUpdater::discoveryStep(DfuDiscoveryContext &ctx, uint32_t elapsedMs)
{
    const int n = getNumDevices(ctx.vid, ctx.pid);
    const eDfuDiscoveryState prev = ctx.state;
    ctx.count = n;

    switch (ctx.state) {
    case DFU_DISCOVERY_IDLE:
        if (n > 0) {                            // first device(s) appeared -> start settling
            ctx.state = DFU_DISCOVERY_STARTED;
            ctx.lastCount = n;
            ctx.stableElapsedMs = 0;
            log_info(IS_LOG_FWUPDATE, "DFU discovery: STARTED (%d device(s) appeared)", n);
        }
        break;

    case DFU_DISCOVERY_STARTED:
        if (n == 0) {                           // all went away -> back to idle
            ctx.state = DFU_DISCOVERY_IDLE;
            log_info(IS_LOG_FWUPDATE, "DFU discovery: IDLE (all devices removed before settling)");
        } else if (n != ctx.lastCount) {        // still arriving/leaving -> restart the settle timer
            log_debug(IS_LOG_FWUPDATE, "DFU discovery: count %d -> %d, settle timer reset", ctx.lastCount, n);
            ctx.lastCount = n;
            ctx.stableElapsedMs = 0;
        } else {                                // count holding steady -> accumulate settle time
            ctx.stableElapsedMs += elapsedMs;
            if (ctx.stableElapsedMs >= ctx.settleTimeoutMs) {
                ctx.state = DFU_DISCOVERY_COMPLETED;
                log_info(IS_LOG_FWUPDATE, "DFU discovery: COMPLETED (%d device(s) settled after %ums)", n, ctx.stableElapsedMs);
            }
        }
        break;

    case DFU_DISCOVERY_COMPLETED:
        if (n == 0) {                           // unplugged -> idle
            ctx.state = DFU_DISCOVERY_IDLE;
            log_info(IS_LOG_FWUPDATE, "DFU discovery: IDLE (all devices removed)");
        } else if (n != ctx.lastCount) {        // set changed after completion -> re-settle
            ctx.state = DFU_DISCOVERY_STARTED;
            log_info(IS_LOG_FWUPDATE, "DFU discovery: re-STARTED (count %d -> %d after completion)", ctx.lastCount, n);
            ctx.lastCount = n;
            ctx.stableElapsedMs = 0;
        }
        break;
    }

    return ctx.state != prev;
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
bool ISDFUFirmwareUpdater::isDFUDevice(libusb_device *usbDevice, uint16_t vid, uint16_t pid) {
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor *cfg;

    if (libusb_get_device_descriptor(usbDevice, &desc) < 0)
        return false;

    // Check vendor and product ID
    if (((vid != 0x0000) && (desc.idVendor != vid)) || ((pid != 0x0000) && (desc.idProduct != pid)))
        return false;

    if (libusb_get_config_descriptor(usbDevice, 0, &cfg) < 0)
        return false;

    // USB-IF DFU interface class numbers
    bool isDfu = (cfg->interface->altsetting[0].bInterfaceClass == 0xFE) &&
                 (cfg->interface->altsetting[0].bInterfaceSubClass == 0x01) &&
                 (cfg->interface->altsetting[0].bInterfaceProtocol == 0x02);

    libusb_free_config_descriptor(cfg);
    return isDfu;
}

/**
 * Creates the device updater, targeting a specific device based on the specified parameters.
 * This updater will only target a single device.  If the target parameters match more than one
 * available device, only the first device is used.
 * TODO: We could consider throwing some kind of "ambiquity" exception if more that one device matches.
 * @param target
 * @param device
 * @param serialNo
 */
ISDFUFirmwareUpdater::ISDFUFirmwareUpdater(fwUpdate::target_t target, libusb_device *device, uint32_t serialNo) : FirmwareUpdateDevice(target) {
    // uint16_t hdwId = (target & fwUpdate::TARGET_IMX5 ? ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0)  : ENCODE_HDW_ID(IS_HARDWARE_TYPE_UINS, 3, 2));
    std::vector<DFUDevice*> devices;
    size_t count = ISDFUFirmwareUpdater::getAvailableDevices(devices);
    count = ISDFUFirmwareUpdater::filterDevicesByTargetType(devices, session_target);
    fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_INFO, "Found %d DFU devices suitable for update.", count);

    if (serialNo == UINT32_MAX)
        curDevice = devices[0];
    else {
        for (auto d : devices) {
            if (d->getSerialNo() == serialNo) {
                curDevice = d;
                break;
            }
        }
    }

    fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_INFO, "Found matching DFU device: %s", curDevice->getDescription());
}

// this is called internally by processMessage() to do the things to do, it should also be called periodically to send status updated, etc.
bool ISDFUFirmwareUpdater::fwUpdate_step(fwUpdate::msg_types_e msg_type, bool processed) {
    static int nextStep = 0;

    if (fwUpdate_getSessionStatus() == fwUpdate::NOT_STARTED)
        return false;

#ifdef DEBUG_INFO
    char prog_msg[256];
        memset(prog_msg, 0, sizeof(prog_msg)); // clear any messages...
        if (msg->hdr.msg_type == fwUpdate::MSG_UPDATE_CHUNK)
            snprintf(prog_msg, sizeof(prog_msg), "DEV :: Received MSG %s (Chunk %d)...\n", MSG_TYPES[msg->hdr.msg_type], msg->data.chunk.chunk_id);
        else
            snprintf(prog_msg, sizeof(prog_msg), "DEV :: Received MSG %s...\n", MSG_TYPES[msg->hdr.msg_type]);
        PRINTF("%s", prog_msg);
    if (result)ISFirmware
        sendProgress(3, (const char *)prog_msg);
#endif // DEBUG_INFO

    nextStep++;

    if (nextStep > 1000 ) {
        nextStep = 0;
        return true;
    }

    return true;
}

/**
 * Not quite sure what to do here just yet, since this manages its out buffer space... we don't really
 * pass anything into here.  Basically, I can either ignore all the parameters, or I can parse the toDevice queue
 * prior to calling this function, and then just call this function...
 * @param rxPort
 * @param buffer
 * @param buf_len
 * @return
 */
bool ISDFUFirmwareUpdater::fwUpdate_processMessage(int rxPort, const uint8_t* buffer, int buf_len) {

    // pull all data from the buffer there really should only be one message at a time... :fingers-crossed:
    const size_t tmpBuf_size = toDevice.size();
    const auto tmpBuf = new uint8_t[tmpBuf_size];
    uint8_t* p = tmpBuf;
    while (!toDevice.empty()) {
        *p++ = toDevice.front();
        toDevice.pop_front();
    }

    bool result = FirmwareUpdateDevice::fwUpdate_processMessage(p, static_cast<int>(tmpBuf_size));
    delete [] tmpBuf;
    return result;
}

// called internally to perform a system reset of various severity per reset_flags (HARD, SOFT, etc)
bool ISDFUFirmwareUpdater::fwUpdate_performReset(fwUpdate::target_t target_id, fwUpdate::reset_flags_e reset_flags) {
/*
    RESET_SOFT = 0,             // typically, a software reset (start the program over, but don't remove power or clear RAM)
    RESET_HARD = 1,             // a hard reset, in which the device is power-cycled; this may not always be possible since generally software on a device can't remove its own power
    RESET_INTO_BOOTLOADER = 2,  // indicates that the device should reset into the bootloader (this may not always be possible)
    RESET_CONFIG = 4,           // indicates that the device should clear its configuration before performing the reset (Ie, factory restart?)
    RESET_UPSTREAM = 8,         // indicates that this device should reset all of its upstream devices, in addition to itself
*/

    // TODO: DFU is pretty limited in what it can do.. we really only can manage a reset by forcing a particular DFU state and/or re-initializing USB
    //  Anything else should probably return not-supported
    if (curDevice->getTargetType() == target_id) {
        curDevice->reset();
        return true;
    }
    return false;
}

// called internally (by the receiving device) to populate the dev_info_t struct for the requested device
bool ISDFUFirmwareUpdater::fwUpdate_queryVersionInfo(fwUpdate::target_t target_id, dev_info_t& dev_info) {
    if (curDevice->getTargetType() == target_id) {
        return curDevice->fillDeviceInfo(dev_info);
    }
    return false;
}

/**
 * Initializes the system to begin receiving firmware image chunks for the target device, image slot and image size.
 * Specifically, this allocates a stream that incoming chunks will be buffered into, and also initializes that stream
 * into the DFUDevice::updateFirmware().  DFUDevice::updateFirmware should operate on the stream, as data becomes available.
 * @param msg
 * @return
 */
fwUpdate::update_status_e ISDFUFirmwareUpdater::fwUpdate_startUpdate(const fwUpdate::payload_t& msg) {

    if (curDevice)
        if (curDevice->open() == DFU_ERROR_NONE) {
            if ((session_target & fwUpdate::TARGET_TYPE_MASK) == fwUpdate::TARGET_IMX5) {
                // hopefully, we've already validated that the target device is already an IMX and there is nothing more to do.
            }
            session_image_size = msg.data.req_update.file_size;
            imgBuffer = new ByteBuffer(session_image_size);
            imgStream = new ByteBufferStream(*imgBuffer);
            return fwUpdate::READY;
        }
    return fwUpdate::ERR_NOT_ALLOWED;
}

// writes the indicated block of data (of len bytes) to the target and device-specific image slot, and with the specified offset
/**
 *
 * @param target_id
 * @param slot_id
 * @param offset
 * @param len
 * @param data
 * @return
 */
fwUpdate::update_status_e  ISDFUFirmwareUpdater::fwUpdate_writeImageChunk(fwUpdate::target_t target_id, int slot_id, int offset, int len, uint8_t *data) {
    int fw_result = DFU_ERROR_NONE;
    if ((session_target & fwUpdate::TARGET_TYPE_MASK) == fwUpdate::TARGET_IMX5) {
        if ((session_image_slot == 0) && imgBuffer && imgStream) {
            imgBuffer->insert(offset, data, len);

            if (fw_result != DFU_ERROR_NONE)
                fwUpdate_sendProgressFormatted(IS_LOG_LEVEL_ERROR, "(%s) ERROR: Firmware update finished with status: %s", curDevice->getDescription(), curDevice->getErrorName(-fw_result));

            //finalization_needed = true;
            curDevice->finalizeFirmware();
        }
    }

    return fwUpdate::ERR_NOT_SUPPORTED;
}

// this marks the finish of the upgrade, that all image bytes have been received, the md5 sum passed, the device can complete the requested upgrade, and perform any device-specific finalization
fwUpdate::update_status_e  ISDFUFirmwareUpdater::fwUpdate_finishUpdate(fwUpdate::target_t target_id, int slot_id, int flags) {
    if (curDevice) {
        curDevice->updateFirmware(reinterpret_cast<std::istream &>(*imgStream));
        curDevice->finalizeFirmware();
        curDevice->close();
        return fwUpdate::FINISHED;
    }
    return fwUpdate::ERR_INVALID_SESSION;
}


// called internally to transmit data to back to the host
bool ISDFUFirmwareUpdater::fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t* buffer, int buff_len) {

    while (buff_len--)
        toHost.push_back(*buffer++);

    return true;
}

/**
 * Returns the total flash size in bytes, calculated from the flash segment geometry.
 */
uint32_t DFUDevice::getTotalFlashSize() const {
    return (uint32_t)segments[STM32_DFU_INTERFACE_FLASH].pages * segments[STM32_DFU_INTERFACE_FLASH].pageSize;
}

/**
 * Returns a human-readable device type name based on processor type and flash size.
 * STM32L4 -> "IMX-5", STM32U5 <=1MB -> "GPX-1", STM32U5 >1MB -> "IMX-6", else -> "Unknown"
 */
const char* DFUDevice::getDeviceTypeName(eProcessorType procType, uint32_t totalFlashSize) {
    if (procType == IS_PROCESSOR_STM32L4)
        return "IMX-5";
    if (procType == IS_PROCESSOR_STM32U5) {
        if (totalFlashSize > 1024 * 1024)
            return "IMX-6";
        return "GPX-1";
    }
    return "Unknown";
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
            if (DECODE_HDW_MAJOR(hardwareId) == 6) return fwUpdate::TARGET_IMX6;
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

    uint8_t busNum = libusb_get_bus_number(usbDevice);
    uint8_t devAddr = libusb_get_device_address(usbDevice);

    if (!isConnected()) {
        // Identification is read-only; open WITHOUT the USB reset to avoid a ~100-200ms per-device
        // re-enumeration on every scan. The subsequent programming open() (default reset=true) still
        // gets a clean slate.
        auto result = open(/*resetDevice*/ false);
        if (result != DFU_ERROR_NONE) {
            log_error(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: open() failed (dfu_error=%d)",
                      busNum, devAddr, result);
            return result;
        }
    }

    int ret = libusb_claim_interface(usbHandle, 0);
    if (ret < LIBUSB_SUCCESS) {
        log_error(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: claim_interface failed (%s)",
                  busNum, devAddr, libusb_error_name(ret));
        return DFU_ERROR_DEVICE_BUSY;
    }

    // NOTE: A brief settle time was previously needed here (SLEEP_MS(100)).
    // If devices fail to enumerate reliably, consider restoring a short delay.

    libusb_device *usb_device = libusb_get_device(usbHandle);
    if (libusb_get_device_descriptor(usb_device, &desc) != LIBUSB_SUCCESS) {
        log_error(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: get_device_descriptor failed", busNum, devAddr);
        libusb_release_interface(usbHandle, 0);
        return DFU_ERROR_LIBUSB;
    }

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

    log_debug(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: VID=%04X PID=%04X mfg=\"%s\" prod=\"%s\" serial=\"%s\"",
              busNum, devAddr, vid, pid, dfuManufacturer.c_str(), dfuProduct.c_str(), dfuSerial.c_str());

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
                        }
                    }

                    if (funcDescriptor.bLength == 7) {
                        funcDescriptor.bcdDFUVersion = libusb_cpu_to_le16(0x0100);
                    } else if (funcDescriptor.bLength < 9) {
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
                            log_debug(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: alt[%zu]=\"%s\" addr=0x%08llX pages=%u pageSize=%u",
                                      busNum, devAddr, lastPos, alt_name,
                                      (unsigned long long)segments[lastPos].address, segments[lastPos].pages, segments[lastPos].pageSize);
                        } else {
                            log_warn(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: failed to read alt descriptor string (alt_idx=%d)",
                                     busNum, devAddr, alt_idx);
                            dfuDescriptors.push_back("");
                        }
                    }
                }
            }

            libusb_free_config_descriptor(cfg);
        }
    }

    // Calculate a fingerprint from the device info/descriptors
    // NOTE: IF YOU CHANGE THE DATA USED IN THE HASHING BELOW, YOU WILL ALSO HAVE TO CHANGE THE RESPECTIVE FINGERPRINTS in ISBootloaderDFU.h
    md5_update(fingerprint, reinterpret_cast<const unsigned char *>(&vid), sizeof(vid));
    md5_update(fingerprint, reinterpret_cast<const unsigned char *>(&pid), sizeof(pid));
    for (auto& dfuDesc: dfuDescriptors) {
        md5_update(fingerprint, reinterpret_cast<const unsigned char *>(dfuDesc.c_str()), (unsigned int)dfuDesc.size());
    }

    uint16_t hardwareType = 0;
    processorType = IS_PROCESSOR_UNKNOWN;

    // SN-8165: also note whether this is a known IMX-6 / GPX-1 signature specifically (the two STM32U5
    // fingerprints: _2M = IMX-6, _1M = GPX-1). Keyed on the exact fingerprint -- not the coarser
    // processorType -- so a future STM32U5-based product with a different signature is NOT silently
    // OTP-skipped below. Those two parts lock OTP behind RDP after their first boot, making the OTP read
    // a guaranteed ~200ms-timeout failure on every later scan.
    bool fingerprintIsImx6OrGpx1 = false;
    if (md5_matches(fingerprint.state, DFU_FINGERPRINT_STM32L4)) processorType = IS_PROCESSOR_STM32L4;
    else if (md5_matches(fingerprint.state, DFU_FINGERPRINT_STM32U5_1M)) { processorType = IS_PROCESSOR_STM32U5; fingerprintIsImx6OrGpx1 = true; }
    else if (md5_matches(fingerprint.state, DFU_FINGERPRINT_STM32U5_2M)) { processorType = IS_PROCESSOR_STM32U5; fingerprintIsImx6OrGpx1 = true; }

    if (processorType == IS_PROCESSOR_UNKNOWN) {
        log_warn(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: unknown fingerprint: %s  descriptors(%zu):",
                 busNum, devAddr, md5_to_string(fingerprint.state).c_str(), dfuDescriptors.size());
        for (size_t i = 0; i < dfuDescriptors.size(); i++)
            log_warn(IS_LOG_FWUPDATE, "  [%zu] \"%s\"", i, dfuDescriptors[i].c_str());
    } else {
        log_debug(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: fingerprint=%s processor=%d descriptors=%zu",
                  busNum, devAddr, md5_to_string(fingerprint.state).c_str(), processorType, dfuDescriptors.size());
    }

    // try and read the OTP memory
    dfu_memory_t otp = segments[STM32_DFU_INTERFACE_OTP];
    if (otp.address != 0) {
        sn = -1; // 0xFFFFFFFF
        hardwareId = -1; // 0xFFFF

        // Decide per-device, by exact signature, whether reading OTP is worth attempting. On an
        // IMX-6 / GPX-1 (the two STM32U5 fingerprints) the OTP is readable only during the single power
        // cycle between the factory OTP write and the next reset (which latches RDP); on every subsequent
        // enumeration the read is guaranteed to fail and costs the full ~200ms timeout per device. The
        // module type (IMX-6 vs GPX-1) is still resolved below from the descriptor flash size, so the DFU
        // listing loses nothing by skipping it. The STM32L4 (IMX-5) -- and any unrecognized signature --
        // keeps reading OTP, where the SN is genuinely available and useful across runs.
        if (fingerprintIsImx6OrGpx1) {
            log_debug(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: skipping OTP read (IMX-6/GPX-1 OTP is RDP-locked after first boot; type resolved from flash size)",
                      busNum, devAddr);
        } else {
            auto rxBuf = new uint8_t[otp.pageSize] {0};
            // OTP lives in protected flash, so reading it on a read-protected (RDP-locked) module fails --
            // either an immediate STALL (LIBUSB_ERROR_PIPE) or, if the device just NAKs, a wait timeout.
            // Use a short timeout here so a locked module is detected in ~200ms rather than stalling the
            // whole enumeration for the full default (5s) per device. The RDP/option-byte register at
            // 0x40022040 is NOT flash and stays readable, so checkReadProtection() still detects the lock.
            const uint32_t OTP_READ_TIMEOUT_MS = 200;
            auto len = readMemory(static_cast<uint32_t>(otp.address), rxBuf, otp.pageSize, OTP_READ_TIMEOUT_MS);
            if (len > 0) {
                otp_info_t *id = decodeOTPData(rxBuf, len);
                if (id != nullptr) {
                    sn = id->serialNumber;
                    hardwareType = id->hardwareId;
                    log_debug(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: OTP SN=%u hwId=0x%04X",
                              busNum, devAddr, sn, hardwareType);
                } else {
                    log_info(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: OTP read OK (%d bytes) but no valid OTP data (unprogrammed?)",
                             busNum, devAddr, len);
                }
            } else {
                // Failed/short read here most likely means the module is read-protected (RDP-locked);
                // SN/hwId stay at their unknown defaults and the device is still listed by type. The USB
                // DFU serial (dfuSerial) remains valid for identifying the module.
                log_warn(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: OTP readMemory failed (ret=%d, addr=0x%08X, size=%u); likely read-protected (RDP-locked), though a transient USB error could also cause this -- SN/hwId left unknown",
                         busNum, devAddr, len, (uint32_t)otp.address, otp.pageSize);
            }
            delete [] rxBuf;
        }
    } else {
        log_warn(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: no OTP segment found (descriptors=%zu)",
                 busNum, devAddr, dfuDescriptors.size());
        libusb_release_interface(usbHandle, 0);
        return DFU_ERROR_LIBUSB;
    }

    if ((hardwareId == 0xFFFF) && (processorType != IS_PROCESSOR_UNKNOWN)) {
        if (processorType == IS_PROCESSOR_STM32L4) hardwareType = IS_HARDWARE_TYPE_IMX; // only the IMX-5 uses the STM32L4
        else if (processorType == IS_PROCESSOR_STM32U5) {
            uint32_t totalFlash = getTotalFlashSize();
            hardwareType = (totalFlash > 1024 * 1024) ? IS_HARDWARE_TYPE_IMX : IS_HARDWARE_TYPE_GPX;
        }
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
                if (processorType == IS_PROCESSOR_STM32L4)
                    hardwareId = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
                else if (processorType == IS_PROCESSOR_STM32U5)
                    hardwareId = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 6, 0);
                break;
            case IS_HARDWARE_TYPE_GPX:
                hardwareId = ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, 1, 0);
                break;
        }
    }

    log_info(IS_LOG_FWUPDATE, "DFU fetchDeviceInfo [%d:%d]: complete — %s (hwId=0x%04X, SN=%u, proc=%d, flash=%uKB)",
             busNum, devAddr, getDeviceTypeName(processorType, getTotalFlashSize()),
             hardwareId, sn, processorType, getTotalFlashSize() / 1024);

    libusb_release_interface(usbHandle, 0);
    libusb_close(usbHandle);
    usbHandle = nullptr;
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
dfu_error DFUDevice::open(bool resetDevice) {
    int ret_libusb;
    dfu_error ret_dfu = DFU_ERROR_NONE;

    if ((ret_libusb = libusb_open(usbDevice, &usbHandle)) < LIBUSB_SUCCESS) {
        log_error(IS_LOG_FWUPDATE, "DFU open: libusb_open failed (%s)", libusb_error_name(ret_libusb));
        return DFU_ERROR_DEVICE_NOTFOUND;
    }

    // Detach kernel driver if active (primarily needed on Linux)
    int kernelActive = libusb_kernel_driver_active(usbHandle, 0);
    if (kernelActive == 1) {
        ret_libusb = libusb_detach_kernel_driver(usbHandle, 0);
        if (ret_libusb < LIBUSB_SUCCESS)
            log_warn(IS_LOG_FWUPDATE, "DFU open: detach_kernel_driver failed (%s)", libusb_error_name(ret_libusb));
    }
    // A USB port reset forces a full re-enumeration (~100-200ms on Linux). It's wanted before programming
    // (clean slate) but is pure overhead for a read-only identification pass, where abort()+waitForState
    // below already establish a known DFU state. Discovery opens with resetDevice=false to avoid paying
    // that cost on every device, every scan.
    if (resetDevice)
        libusb_reset_device(usbHandle);

    // Claim with a short bounded retry: a claim can transiently fail with LIBUSB_ERROR_BUSY when a prior
    // handle on the same device (a just-finished discovery open, a sibling worker, or the kernel driver
    // re-attaching) hasn't fully released yet. Rather than drop the device from a provisioning run on a
    // momentary collision, wait briefly and retry a few times before giving up.
    const int   CLAIM_MAX_ATTEMPTS = 3;
    const int   CLAIM_RETRY_MS     = 50;
    for (int attempt = 1; ; attempt++) {
        ret_libusb = libusb_claim_interface(usbHandle, 0);
        // Only LIBUSB_ERROR_BUSY is the transient case worth retrying (a prior handle or the kernel
        // driver hasn't finished releasing). Any other failure (NO_DEVICE, ACCESS, ...) is terminal --
        // retrying only adds latency and emits a misleading "busy" log, so bail immediately on those.
        if (ret_libusb >= LIBUSB_SUCCESS || ret_libusb != LIBUSB_ERROR_BUSY || attempt >= CLAIM_MAX_ATTEMPTS)
            break;
        log_warn(IS_LOG_FWUPDATE, "DFU open: claim_interface busy (%s), retry %d/%d after %dms",
                 libusb_error_name(ret_libusb), attempt, CLAIM_MAX_ATTEMPTS - 1, CLAIM_RETRY_MS);
        SLEEP_MS(CLAIM_RETRY_MS);
    }
    if (ret_libusb < LIBUSB_SUCCESS) {
        log_error(IS_LOG_FWUPDATE, "DFU open: claim_interface failed (%s)", libusb_error_name(ret_libusb));
        ret_dfu = DFU_ERROR_DEVICE_BUSY;
    } else {
        // Cancel any existing operations and return to IDLE state
        ret_libusb = abort();
        if (ret_libusb == LIBUSB_SUCCESS) {
            ret_libusb = waitForState(DFU_STATE_IDLE);
        }
        if (ret_libusb < LIBUSB_SUCCESS) {
            log_warn(IS_LOG_FWUPDATE, "DFU open: failed to reach IDLE state (%s)", libusb_error_name(ret_libusb));
            ret_dfu = DFU_ERROR_STATUS;
        }
    }

    if (ret_dfu != DFU_ERROR_NONE) {
        // Don't leak the handle on a failed open. Release the interface (a no-op/harmless if the claim
        // itself failed), then close and null usbHandle so isConnected() correctly reports the device as
        // not open rather than appearing connected on a half-open handle.
        libusb_release_interface(usbHandle, 0);
        libusb_close(usbHandle);
        usbHandle = nullptr;
    }

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
/**
 * SN-8043: Pre-flight read-protection check. See header. STM32U5 only; no-op otherwise.
 */
dfu_error DFUDevice::checkReadProtection() {
    if (processorType != IS_PROCESSOR_STM32U5)
        return DFU_ERROR_NONE;

    if (!isConnected()) {
        dfu_error ret = open();
        if (ret != DFU_ERROR_NONE)
            return ret;
    }

    // FLASH_OPTR is exposed through the DFU OPTIONS segment, which the ROM bootloader allows reading
    // regardless of RDP level. The option-byte block mirrors the FLASH option registers in order:
    // [0]=FLASH_OPTR, [1]=FLASH_NSBOOTADD0R, [2]=FLASH_NSBOOTADD1R. The RDP byte is OPTR[7:0].
    uint8_t optBytes[12] = { 0 };
    const dfu_memory_t& options = segments[STM32_DFU_INTERFACE_OPTIONS];
    // Short timeout: on a read-protected module this read is dropped/stalls, and the pre-flight must
    // not block for the full default per device (a healthy OPTR read completes in well under this).
    const uint32_t OPTR_READ_TIMEOUT_MS = 200;
    int read = readMemory(static_cast<uint32_t>(options.address), optBytes, sizeof(optBytes), OPTR_READ_TIMEOUT_MS);
    if (read < (int)sizeof(uint32_t)) {
        // On the STM32U5 the option-byte read is dropped/stalls exactly when the module is read-
        // protected, so treat an unreadable FLASH_OPTR as RDP-locked rather than skipping the gate.
        // This reports a clear RDP_LOCKED (instead of a later opaque erase timeout) so the caller can
        // handle the condition explicitly. A false positive on a genuinely unlocked module is harmless
        // -- the caller's RDP_LOCKED handling is a no-op when the device is not actually protected.
        // INFO, not WARN: this is an expected, handled condition (the caller turns RDP_LOCKED into
        // either a clear failure or the opt-in recovery), so it should show as transient status rather
        // than a sticky per-device warning in the UI.
        if (statusCb)
            statusCb(this, IS_LOG_LEVEL_INFO, "(%s) RDP pre-flight: could not read FLASH_OPTR (ret=%d); assuming read-protected (RDP-locked)", getDescription(), read);
        return DFU_ERROR_RDP_LOCKED;
    }

    // Decode and log the committed boot configuration. On a re-provisioning attempt these are the
    // values the PREVIOUS finalize actually committed, so this is the ground truth for diagnosing a
    // device that reboots back into DFU: it shows whether the option-byte write took effect, and
    // which boot address the ROM will select for each BOOT0 pin level (nSWBOOT0=1 => pin selects).
    if (read >= 12) {
        uint32_t optr        = optBytes[0] | (optBytes[1] << 8) | (optBytes[2] << 16) | ((uint32_t)optBytes[3] << 24);
        uint32_t nsBootAdd0R = optBytes[4] | (optBytes[5] << 8) | (optBytes[6] << 16) | ((uint32_t)optBytes[7] << 24);
        uint32_t nsBootAdd1R = optBytes[8] | (optBytes[9] << 8) | (optBytes[10] << 16) | ((uint32_t)optBytes[11] << 24);
        uint32_t bootAddr0   = nsBootAdd0R & 0xFFFFFF80u;   // NSBOOTADD0 is bits[31:7]; address = field << 7
        uint32_t bootAddr1   = nsBootAdd1R & 0xFFFFFF80u;   // NSBOOTADD1 (BOOT0=1 / fallback) target
        uint8_t  nSwBoot0    = (optr >> 26) & 1u;           // 1 => BOOT0 pin selects; 0 => nBOOT0 bit selects
        uint8_t  nBoot0      = (optr >> 27) & 1u;
        uint8_t  tzen        = (optr >> 31) & 1u;
        log_info(IS_LOG_FWUPDATE,
                 "DFU bootcfg: OPTR=0x%08X (RDP=0x%02X nSWBOOT0=%u nBOOT0=%u TZEN=%u) NSBOOTADD0=0x%08X NSBOOTADD1=0x%08X => BOOT0-low boots 0x%08X, BOOT0-high boots 0x%08X",
                 optr, optr & 0xFF, nSwBoot0, nBoot0, tzen, bootAddr0, bootAddr1, bootAddr0, bootAddr1);
    }

    uint8_t rdp = optBytes[0];
    dfu_error verdict = rdpVerdict(rdp);
    if (verdict != DFU_ERROR_NONE && statusCb) {
        statusCb(this, IS_LOG_LEVEL_ERROR,
                 "(%s) Read protection active (RDP=0x%02X): flash programming will be silently dropped. Erase/recover the module before provisioning.",
                 getDescription(), rdp);
    }
    return verdict;
}

/**
 * SN-8043: Post-write readback verification. See header. STM32U5 only; no-op otherwise.
 */
dfu_error DFUDevice::verifyFlashWrite(uint32_t address, const uint8_t *expected, uint32_t verifyLen) {
    if (processorType != IS_PROCESSOR_STM32U5)
        return DFU_ERROR_NONE;
    if (expected == nullptr || verifyLen == 0)
        return DFU_ERROR_NONE;

    if (verifyLen > 64)
        verifyLen = 64;   // first 64 bytes (vector table: initial SP + reset vector) are never all-0xFF on a real image

    uint8_t readback[64] = { 0 };
    int read = readMemory(address, readback, verifyLen);
    if (read < (int)verifyLen) {
        // Could not read back; log but don't fail (avoid false negatives on otherwise-good devices).
        if (statusCb)
            statusCb(this, IS_LOG_LEVEL_WARN, "(%s) Write verify: readback failed (ret=%d) @ 0x%08X; skipping verify", getDescription(), read, address);
        return DFU_ERROR_NONE;
    }

    if (memcmp(readback, expected, verifyLen) != 0) {
        if (statusCb)
            statusCb(this, IS_LOG_LEVEL_ERROR, "(%s) Write verify FAILED @ 0x%08X: flash readback does not match source (writes were dropped).", getDescription(), address);
        return DFU_ERROR_WRITE_VERIFY_FAILED;
    }
    return DFU_ERROR_NONE;
}

dfu_error DFUDevice::updateFirmware(std::string filename, uint64_t baseAddress) {
    ihex_image_section_t image[MAX_NUM_IHEX_SECTIONS];
    size_t image_sections;
    int ret_libusb;
    dfu_error ret_dfu = DFU_ERROR_NONE;

    SLEEP_MS(100);

    if (!isConnected()) {
        ret_dfu = open();
        if (ret_dfu != DFU_ERROR_NONE)
            return ret_dfu;
    }

    // SN-8043: refuse to proceed when the chip is read-protected (RDP > 0xAA). At RDP Level 1 the ROM
    // bootloader ACKs every flash DNLOAD but silently drops the programming, which previously produced
    // a false DFU_ERROR_NONE and a bricked, empty-flash unit. STM32U5 only; no-op for other processors.
    if ((ret_dfu = checkReadProtection()) != DFU_ERROR_NONE)
        return ret_dfu;

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
            auto baseOffset = static_cast<uint32_t>(baseAddress - image[0].address);
            if (baseOffset) {
                for (size_t i = 0; i < image_sections; image[i].address += baseOffset, i++);
            }
        }
    } else {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open())
            return DFU_ERROR_FILE_NOTFOUND;

        image[0].address = static_cast<uint32_t>(baseAddress ? baseAddress : segments[STM32_DFU_INTERFACE_FLASH].address);
        auto fsize = file.tellg();
        file.seekg( 0, std::ios::end );
        image[0].len = static_cast<uint32_t>(file.tellg() - fsize);
        file.seekg( 0, std::ios::beg );

        image[0].image = static_cast<uint8_t *>(malloc(image[0].len));
        file.read((char *)image[0].image, image[0].len);

        file.close();
        image_sections = 1;
    }

    uint32_t offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
    if (statusCb) {
        size_t lastSep = filename.find_last_of('/')+1;
        std::string fname(filename.substr(lastSep, filename.length() - lastSep));
        statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Updating flash with firmware \"%s\" (@ 0x%08X)", getDescription(), fname.c_str(), segments[STM32_DFU_INTERFACE_FLASH].address + offset);
    }

    ret_libusb = abort();
    if (ret_libusb == LIBUSB_SUCCESS) {
        if (statusCb) {
            statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Erasing flash memory...", getDescription());
        }

        offset = static_cast<uint32_t>(image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address);
        for (size_t i = 0; i < image_sections; i++) {
            //offset = image[i].address + offset;
            ret_dfu = eraseFlash(segments[STM32_DFU_INTERFACE_FLASH], offset, image[i].len);
            if (ret_dfu != DFU_ERROR_NONE) {
                statusCb(this, IS_LOG_LEVEL_ERROR, "(%s) Error erasing flash: %04x", getDescription(), -ret_dfu);
                return ret_dfu;
            }
        }

        if (statusCb) {
            statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Programming flash memory...", getDescription());
        }

        offset = static_cast<uint32_t>(image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address);
        for (size_t i = 0; i < image_sections; i++) {
            ret_dfu = writeFlash(segments[STM32_DFU_INTERFACE_FLASH], offset, image[i].len, image[i].image);
            if (ret_dfu != DFU_ERROR_NONE) {
                statusCb(this, IS_LOG_LEVEL_ERROR, "(%s) Error writing flash: %04x", getDescription(), -ret_dfu);
                return ret_dfu;
            }
        }

        // SN-8043: verify the write actually landed. Catches the RDP-silent-drop (all-0xFF readback)
        // and any other "bootloader ACK'd but flash didn't take" failure. verifyFlashWrite() logs.
        if ((ret_dfu = verifyFlashWrite(image[0].address, image[0].image, image[0].len)) != DFU_ERROR_NONE)
            return ret_dfu;
    }

    // Unload the firmware image
    ihex_unload_sections(image, image_sections);
    return ret_dfu;
}

/**
 * High-level function used to program flash memory using the provided stream. This function will ONLY
 * target the device's FLASH memory segment. The baseAddress must point to a location within the
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

    // SN-8043: refuse to proceed when the chip is read-protected (RDP > 0xAA). At RDP Level 1 the ROM
    // bootloader ACKs every flash DNLOAD but silently drops the programming, which previously produced
    // a false DFU_ERROR_NONE and a bricked, empty-flash unit. STM32U5 only; no-op for other processors.
    if ((ret_dfu = checkReadProtection()) != DFU_ERROR_NONE)
        return ret_dfu;

    {
        if (!stream)
            return DFU_ERROR_FILE_NOTFOUND;


        image[0].address = static_cast<uint32_t>(baseAddress ? baseAddress : segments[STM32_DFU_INTERFACE_FLASH].address);
        auto fsize = stream.tellg();
        stream.seekg( 0, std::ios::end );
        image[0].len = static_cast<uint32_t>(stream.tellg() - fsize);
        stream.seekg( 0, std::ios::beg );

        image[0].image = static_cast<uint8_t *>(malloc(image[0].len));
        stream.read((char *)image[0].image, image[0].len);

        //stream.close();
        image_sections = 1;
    }

    uint32_t offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
    if (statusCb) {
        statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Updating flash with firmware \"%s\" (@ 0x%08X)", getDescription(), segments[STM32_DFU_INTERFACE_FLASH].address + offset);
    }

    ret_libusb = abort();
    if (ret_libusb == LIBUSB_SUCCESS) {
        if (statusCb) {
            dfu_memory_t& flash = segments[STM32_DFU_INTERFACE_FLASH];
            statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Flash segment: addr=0x%08X, pageSize=%u, pages=%u",
                     getDescription(), flash.address, flash.pageSize, flash.pages);
            statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Erasing flash memory...", getDescription());
        }

        offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
        for (size_t i = 0; i < image_sections; i++) {
            //offset = image[i].address + offset;
            ret_dfu = eraseFlash(segments[STM32_DFU_INTERFACE_FLASH], offset, image[i].len);
            if (ret_dfu != DFU_ERROR_NONE) {
                statusCb(this, IS_LOG_LEVEL_ERROR, "(%s) Error erasing flash: %04x", getDescription(), -ret_dfu);
                return ret_dfu;
            }
        }

        if (statusCb) {
            statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Programming flash memory...", getDescription());
        }

        offset = image[0].address - segments[STM32_DFU_INTERFACE_FLASH].address;
        for (size_t i = 0; i < image_sections; i++) {
            ret_dfu = writeFlash(segments[STM32_DFU_INTERFACE_FLASH], offset, image[i].len, image[i].image);
            if (ret_dfu != DFU_ERROR_NONE) {
                statusCb(this, IS_LOG_LEVEL_ERROR, "(%s) Error writing flash: %04x", getDescription(), -ret_dfu);
                return ret_dfu;
            }
        }

        // SN-8043: verify the write actually landed. Catches the RDP-silent-drop (all-0xFF readback)
        // and any other "bootloader ACK'd but flash didn't take" failure. verifyFlashWrite() logs.
        if ((ret_dfu = verifyFlashWrite(image[0].address, image[0].image, image[0].len)) != DFU_ERROR_NONE)
            return ret_dfu;
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
        if (ret_libusb < LIBUSB_SUCCESS) {
            if (statusCb) statusCb(this, IS_LOG_LEVEL_ERROR, "(%s) Erase download failed at 0x%08X: libusb=%d", getDescription(), pageAddress, ret_libusb);
            return libusbError(ret_libusb);
        }

        ret_libusb = waitForState(DFU_STATE_DNLOAD_IDLE, &state);
        if (ret_libusb < LIBUSB_SUCCESS) {
            if (statusCb) statusCb(this, IS_LOG_LEVEL_ERROR, "(%s) Erase waitForState failed at 0x%08X: libusb=%d, state=%d", getDescription(), pageAddress, ret_libusb, state);
            return libusbError(ret_libusb);
        }

        byteInSection += mem.pageSize;
        bytes_erased += mem.pageSize;
        offset += mem.pageSize;

        if (progressCb) {
            float progress = (float) bytes_erased / (float) data_len;
            progressCb(this, progress, "Erasing Flash", 1, 2);
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

    log_info(IS_LOG_FWUPDATE, "DFU writeFlash: data_len=%u, pageSize=%u, wTransferSize=%u, chunksPerPage=%u",
             data_len, mem.pageSize, funcDescriptor.wTransferSize,
             funcDescriptor.wTransferSize ? (mem.pageSize + funcDescriptor.wTransferSize - 1) / funcDescriptor.wTransferSize : 0);

    // Write memory
    uint32_t bytes_written = 0;
    uint32_t byteInSection = 0;
    uint8_t* payload = new uint8_t[mem.pageSize];

    if (progressCb) {
        float progress = (float) bytes_written / (float) data_len;
        progressCb(this, progress, "Writing Flash", 2, 2);
    }

    do {
        uint32_t payloadLen = mem.pageSize;
        uint32_t bytesRemaining = data_len - byteInSection;
        if (payloadLen > bytesRemaining)
            payloadLen = bytesRemaining;

        // Set write address
        ret_libusb = setAddress(dlBlockNum, mem.address + offset);
        if (ret_libusb < LIBUSB_SUCCESS) {
            return libusbError(ret_libusb);
        }

        // Copy image into buffer for transmission
        memset(payload, 0xFF, mem.pageSize);
        memcpy(payload, &data[byteInSection], payloadLen);

        //blockNum = (uint16_t) (byteInSection / mem.pageSize) + wValue;

        ret_libusb = download(dlBlockNum, payload, payloadLen);
        if (ret_libusb < LIBUSB_SUCCESS) {
            ret_dfu = libusbError(ret_libusb);
            break;
        }

        ret_libusb = waitForState(DFU_STATE_DNLOAD_IDLE);
        if (ret_libusb < LIBUSB_SUCCESS) {
            ret_dfu = libusbError(ret_libusb);
            break;
        }

        byteInSection += payloadLen;
        bytes_written += payloadLen;
        offset += mem.pageSize;

        if (progressCb) {
            float progress = (float) bytes_written / (float) data_len;
            progressCb(this, progress, "Writing Flash", 2, 2);
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

    if (statusCb) {
        statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Finalizing DFU programming...", getDescription());
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

        // See the STM32U5 branch below: the verifyFlashWrite() readback can leave the bootloader in
        // dfuERROR, where DFU_ABORT STALLs (LIBUSB_ERROR_PIPE). Recover with DFU_CLRSTATUS instead.
        state = DFU_STATE_ERROR;
        getState(&state);
        log_debug(IS_LOG_FWUPDATE, "(%s) DFU finalize: entry state=%d", getDescription(), state);
        if (state == DFU_STATE_ERROR) {
            clearStatus();                              // valid recovery from dfuERROR (abort would STALL)
            waitForState(DFU_STATE_IDLE);
        } else if ((state != DFU_STATE_DNLOAD_IDLE) && (state != DFU_STATE_IDLE)) {
            abort(); // We were doing something else, but not any more... Cancel any existing operations, and return to a good, known state
            ret_libusb = waitForState(DFU_STATE_IDLE);
            if (ret_libusb < LIBUSB_SUCCESS)
                return libusbError(ret_libusb);
        }

        ret_libusb = setAddress(dlBlockNum, segments[STM32_DFU_INTERFACE_OPTIONS].address);
        if (ret_libusb < LIBUSB_SUCCESS)
            return libusbError(ret_libusb);

        // STM32 DFU specs will reset the device immediately after writing to the Option Bytes. That
        // reset drops the USB device off the bus mid-transfer, so a disconnect-class libusb error here
        // is the EXPECTED, successful outcome (SN-8193); only a non-disconnect error is a real failure.
        ret_libusb = download(dlBlockNum, bytes, sizeof(bytes));
        if (ret_libusb < LIBUSB_SUCCESS && !isExpectedOptionByteResetError(ret_libusb))
            return libusbError(ret_libusb);

        if ((ret_libusb < LIBUSB_SUCCESS) && statusCb)
            statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Option bytes written; device reset as expected (libusb=%d)", getDescription(), ret_libusb);

        // The device just restarted; we have no further indication of an error, so it is OK.
        return DFU_ERROR_NONE;

    } else if (processorType == IS_PROCESSOR_STM32U5) {

        // Option bytes - Address: 0x40022040
        // This hard-coded array sets mostly defaults, but without PH3 enabled and
        // with DFU mode disabled. Application will enable DFU mode if needed.
        // SN-8140: OPTR byte 3 was 0x1B (nSWBOOT0=0). The IMX firmware's
        // flash_init()->flash_set_optr(ST_OPTR_PROD_DEFAULT=0x1FEFF8AA) forces
        // nSWBOOT0=1 on boot via an OBL_LAUNCH+reset; that resumed boot faults
        // for layout-unlucky builds (e.g. the 3.0.0 release), bricking the unit.
        // Provision OPTR=0x1FEFF8AA to match PROD_DEFAULT so no OBL_LAUNCH fires.
        uint8_t bytes[] = {
                0xaa, 0xf8, 0xef, 0x1F,  0x7f, 0x00, 0x00, 0x08,  0x7f, 0x00, 0xf9, 0x0B,  0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  0xff, 0xff, 0x80, 0xff,  0xff, 0xff, 0x80, 0xff,
                0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00
        };

        // Establish a clean DFU state before writing the Option Bytes. SN-8043's post-write
        // verifyFlashWrite() readback (a DFU UPLOAD) can leave the STM32U5 ROM bootloader in
        // dfuERROR. DFU_ABORT is INVALID from dfuERROR and the device STALLs it (LIBUSB_ERROR_PIPE),
        // which previously aborted finalize before the Option Bytes were ever written -- and SN-8193
        // then masked that PIPE as a benign "finalize warning", so provisioning reported success
        // while the boot-config option bytes silently never changed. The only valid recovery from
        // dfuERROR is DFU_CLRSTATUS; DFU_ABORT is only valid from the idle/upload-idle states.
        state = DFU_STATE_ERROR;
        getState(&state);
        log_debug(IS_LOG_FWUPDATE, "(%s) DFU finalize: entry state=%d", getDescription(), state);
        if (state == DFU_STATE_ERROR) {
            clearStatus();                              // valid recovery from dfuERROR (abort would STALL)
            waitForState(DFU_STATE_IDLE);
        } else if (state != DFU_STATE_DNLOAD_IDLE) {    // Cancel any existing operations
            ret_libusb = abort();
            if (ret_libusb < LIBUSB_SUCCESS)
                return libusbError(ret_libusb);

            // Reset status to good
            ret_libusb = waitForState(DFU_STATE_IDLE);
            if (ret_libusb < LIBUSB_SUCCESS)
                return libusbError(ret_libusb);
        }

        ret_libusb = setAddress(dlBlockNum, segments[STM32_DFU_INTERFACE_OPTIONS].address);
        if (ret_libusb < LIBUSB_SUCCESS)
            return libusbError(ret_libusb);

        // STM32 DFU specs will reset the device immediately after writing to the Option Bytes. That
        // reset drops the USB device off the bus mid-transfer, so a disconnect-class libusb error here
        // is the EXPECTED, successful outcome (SN-8193); only a non-disconnect error is a real failure.
        ret_libusb = download(dlBlockNum, bytes, sizeof(bytes));
        if (ret_libusb < LIBUSB_SUCCESS && !isExpectedOptionByteResetError(ret_libusb))
            return libusbError(ret_libusb);

        if ((ret_libusb < LIBUSB_SUCCESS) && statusCb)
            statusCb(this, IS_LOG_LEVEL_INFO, "(%s) Option bytes written; device reset as expected (libusb=%d)", getDescription(), ret_libusb);

        // The device just restarted; we have no further indication of an error, so it is OK.
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
        return libusbError(ret_libusb);

    // Wait for the drop to the MANIFEST state
    ret_libusb = waitForState(DFU_STATE_MANIFEST, &state);
    if (ret_libusb < LIBUSB_SUCCESS)
        return libusbError(ret_libusb);

    ret_libusb = waitForState(DFU_STATE_MANIFEST_WAIT_RESET, &state);
    if (ret_libusb < LIBUSB_SUCCESS)
        return libusbError(ret_libusb);

    // At this point, there is nothing left to due but reset
    detach(100);
    reset();

    return DFU_ERROR_NONE;
}

/**
 * Issues a single-byte DfuSe class-specific command (DFU_DNLOAD with block number 0). Generic: the
 * command code is passed through verbatim, with no interpretation here. See the header for the full
 * contract. Modeled on the DfuSe special-command transport used by the internal erase path.
 */
dfu_error DFUDevice::sendDfuCommand(int cmd) {
    int ret_libusb;
    dfu_error ret_dfu;
    dfu_state state;

    if (!isConnected()) {
        if ((ret_dfu = open()) < DFU_ERROR_NONE)
            return ret_dfu;
    }

    // Establish a clean DFU state before issuing the command. A prior UPLOAD/readback can leave the
    // ROM bootloader in dfuERROR, where DFU_ABORT STALLs (LIBUSB_ERROR_PIPE); the only valid recovery
    // from dfuERROR is DFU_CLRSTATUS.
    state = DFU_STATE_ERROR;
    getState(&state);
    log_debug(IS_LOG_FWUPDATE, "(%s) DFU sendDfuCommand(0x%02X): entry state=%d", getDescription(), (unsigned)(cmd & 0xFF), state);
    if (state == DFU_STATE_ERROR) {
        clearStatus();
        waitForState(DFU_STATE_IDLE);
    } else if ((state != DFU_STATE_DNLOAD_IDLE) && (state != DFU_STATE_IDLE)) {
        ret_libusb = abort();
        if (ret_libusb < LIBUSB_SUCCESS)
            return libusbError(ret_libusb);
        ret_libusb = waitForState(DFU_STATE_IDLE);
        if (ret_libusb < LIBUSB_SUCCESS)
            return libusbError(ret_libusb);
    }

    // DfuSe special commands are sent as a DFU_DNLOAD with wValue/wBlockNum = 0 (see eraseFlash()).
    // A command may kick off a long internal operation and/or an immediate reset; the device then
    // stops responding, which surfaces EITHER as a disconnect-class error OR as a transfer timeout
    // (the GETSTATUS poll gets no answer). Once the DNLOAD has been accepted, both outcomes mean
    // "command accepted, device is resetting" -- the expected, successful result here.
    uint8_t cmdByte = (uint8_t)(cmd & 0xFF);
    dlBlockNum = 0;
    ret_libusb = download(dlBlockNum, &cmdByte, 1, 15000);
    if (ret_libusb < LIBUSB_SUCCESS && !isExpectedOptionByteResetError(ret_libusb) && (ret_libusb != LIBUSB_ERROR_TIMEOUT))
        return libusbError(ret_libusb);

    if ((ret_libusb < LIBUSB_SUCCESS) && statusCb)
        statusCb(this, IS_LOG_LEVEL_INFO, "(%s) DFU command 0x%02X issued; device reset as expected (libusb=%d)", getDescription(), (unsigned)cmdByte, ret_libusb);

    return DFU_ERROR_NONE;
}

/**
 * Closes the DFU/USB device, by cancelling any active DFU operations, and waiting for the DFU IDLE state
 * before releasing the USB interface.
 * @return
 */
dfu_error DFUDevice::close() {
    if (!usbHandle)
        return DFU_ERROR_NONE;

    int ret_libusb;
    dfu_error ret_dfu = DFU_ERROR_NONE;

    // Cancel any existing operations
    ret_libusb = abort();
    if (ret_libusb < LIBUSB_SUCCESS) {
        ret_dfu = libusbError(ret_libusb);
    } else {
        // Reset status to good
        ret_libusb = waitForState(DFU_STATE_IDLE);
        if (ret_libusb < LIBUSB_SUCCESS)
            ret_dfu = libusbError(ret_libusb);
    }

    libusb_release_interface(usbHandle, 0);
    libusb_close(usbHandle);
    usbHandle = nullptr;

    return ret_dfu;
}

/**
 * Produces a human-readable, unique identifier for this device
 * @return
 */
const char *DFUDevice::getDescription() {
    thread_local char buff[64];
    if (sn != 0xFFFFFFFF)
        snprintf(buff, sizeof(buff), "%s-%d.%d:SN-%05d", g_isHardwareTypeNames[DECODE_HDW_TYPE(hardwareId)], DECODE_HDW_MAJOR(hardwareId), DECODE_HDW_MINOR(hardwareId), sn);
    else
        snprintf(buff, sizeof(buff), "%s-%d.%d:DFU-%s", g_isHardwareTypeNames[DECODE_HDW_TYPE(hardwareId)], DECODE_HDW_MAJOR(hardwareId), DECODE_HDW_MINOR(hardwareId), dfuSerial.c_str());
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
            return libusbError(ret_libusb);
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
int DFUDevice::readMemory(uint32_t memloc, uint8_t *rxBuf, size_t rxLen, uint32_t timeout_ms) {
    int ret_libusb;
    uint8_t stringIdx;

    uint32_t waitTime = 0;
    dfu_status status;
    dfu_state state;
    int bytes_read;

    // Cancel any existing operations
    ret_libusb = abort();
    if (ret_libusb >= LIBUSB_SUCCESS) {
        ret_libusb = waitForState(DFU_STATE_IDLE, &state, timeout_ms);
        if (ret_libusb >= LIBUSB_SUCCESS) {
            ret_libusb = setAddress(ulBlockNum, memloc, timeout_ms);
            if (ret_libusb >= LIBUSB_SUCCESS) {
                // drop out of DFU_STATE_DNLOAD_IDLE, and get back to DFU_STATE_IDLE before we 'upload'
                ret_libusb = abort();
                ret_libusb = waitForState(DFU_STATE_IDLE, &state, timeout_ms);
                // Read the full requested memory
                if (ret_libusb >= LIBUSB_SUCCESS) {
                    ret_libusb = upload(ulBlockNum, rxBuf, rxLen, timeout_ms);
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
int DFUDevice::waitForState(dfu_state required_state, dfu_state* actual_state, uint32_t timeout_ms) {
    dfu_status status = DFU_STATUS_ERR_UNKNOWN;
    uint32_t waitTime = 0;
    dfu_state state;
    uint8_t stringIndex;
    int ret_libusb = 0;

    if (actual_state == nullptr)
        actual_state = &state;

    auto start = std::chrono::steady_clock::now();

    int polls = 0;

    do {
        ret_libusb = getStatus(&status, &waitTime, actual_state, &stringIndex);
        if (ret_libusb < LIBUSB_SUCCESS)
            return ret_libusb;

        polls++;

        if (status != DFU_STATUS_OK) {
            clearStatus();
        }
        if (*actual_state != required_state) {
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= timeout_ms)
                return LIBUSB_ERROR_TIMEOUT;

            SLEEP_MS(1);  // Poll aggressively; device bwPollTimeout is overly conservative
        }
    } while ((status != DFU_STATUS_OK) || (*actual_state != required_state));

    // BOMBASTIC only: even a "settled normally" wait is two USB round-trips (~8ms) because the device
    // reports a transient non-target state on the first GETSTATUS, so a low-threshold MORE_DEBUG log here
    // floods once per flash block / per device with no real signal. Keep it at the most verbose tier so
    // it's available when deep-debugging a genuine stall but invisible at normal DEBUG+ levels.
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
    log_bombastic(IS_LOG_FWUPDATE, "DFU waitForState: state=%d reached after %d polls (%lldms, bwPollTimeout=%ums)", required_state, polls, (long long)elapsedMs, waitTime);

    return LIBUSB_SUCCESS;
}

/**
 * Directs the DFU device to read/write memory from the specified address location (this is not a pointer!)
 * @param address
 * @return
 */
int DFUDevice::setAddress(uint16_t& wValue, uint32_t address, uint32_t timeout_ms) {
    dfu_state state;
    dfu_status status;
    uint32_t waitTime = 0;
    uint8_t stringIdx = 0;

    int ret_libusb;
    unsigned char data[5] = {0x21};
    memcpy(&data[1], &address, 4);

    wValue = 0; // a "Set Address" must always have a wValue/wBlockNum of 0
    ret_libusb = download(wValue, data, 5, timeout_ms);
    if (ret_libusb >= LIBUSB_SUCCESS) {
        // Address pointer takes effect after GETSTATUS command
        ret_libusb = getStatus(&status, &waitTime, &state, &stringIdx);
        if ((ret_libusb >= LIBUSB_SUCCESS) && (status == DFU_STATUS_OK) && (state == DFU_STATE_DNBUSY)) {
            ret_libusb = getStatus(&status, &waitTime, &state, &stringIdx);
            if ((ret_libusb >= LIBUSB_SUCCESS) && (status == DFU_STATUS_OK)) {
                ret_libusb = abort();
                if (ret_libusb >= LIBUSB_SUCCESS)
                    ret_libusb = waitForState(DFU_STATE_DNLOAD_IDLE, nullptr, timeout_ms);
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
int DFUDevice::upload(uint16_t& wValue, uint8_t *buf, uint16_t len, uint32_t timeout_ms) {

    int ret_libusb = LIBUSB_SUCCESS;
    int bytesRemain = len, bytesReceived = 0;
    do {
        int bytesToReceive = (bytesRemain > funcDescriptor.wTransferSize) ? funcDescriptor.wTransferSize : bytesRemain;
        // Honor timeout_ms on the transfer itself (not just the waitForState below) so the parameter
        // bounds the whole per-block operation; a slow device producing UPLOAD data no longer trips a
        // fixed 100ms transfer timeout.
        ret_libusb = libusb_control_transfer(usbHandle, 0b10100001, 0x02, wValue, 0, buf + bytesReceived, bytesToReceive, timeout_ms);
        if (ret_libusb >= LIBUSB_SUCCESS) {
            bytesReceived += ret_libusb; // setup for next block transfer
            bytesRemain -= ret_libusb;
            wValue++;

            if (len != 0) // if len == 0, we expect to fall into the MANIFEST cycle
                ret_libusb = waitForState(DFU_STATE_UPLOAD_IDLE, nullptr, timeout_ms);
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
int DFUDevice::download(uint16_t& wValue, uint8_t *buf, uint16_t len, uint32_t timeout_ms) {
    dfu_state state = DFU_STATE_IDLE;
    int ret_libusb = LIBUSB_SUCCESS;
    int bytesRemain = len, bytesSent = 0;
    do {
        int bytesToSend = (bytesRemain > funcDescriptor.wTransferSize) ? funcDescriptor.wTransferSize : bytesRemain;
        // Honor timeout_ms on the transfer itself (not just the waitForState below) so the parameter
        // bounds the whole per-block operation rather than a fixed 100ms transfer timeout.
        ret_libusb = libusb_control_transfer(usbHandle, 0b00100001, 0x01, wValue, 0, buf + bytesSent, bytesToSend, timeout_ms);
        if (ret_libusb >= LIBUSB_SUCCESS) {
            bytesSent += ret_libusb; // setup for next block transfer
            bytesRemain -= ret_libusb;
            wValue++;

            if (len != 0) { // if len == 0, we expect to fall into the MANIFEST cycle
                ret_libusb = waitForState(DFU_STATE_DNLOAD_IDLE, &state, timeout_ms);
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

/**
 * Attempts to populate a dev_info_t struct using information that can be determined through the DFU interface.
 * This should include the hardware id (type, version), serial number, etc. In some instances, we maybe able
 * to read some or most of this information by reading from certain areas of flash.  However, note that
 * with Read-out Protection enabled on the GPX (and eventually on the IMX) we will likely be limited in what
 * we can continue to read.
 * @param devInfo
 * @return true if devInfo was at least populated with hardware type, version, and serial number information, otherwise false
 */
int DFUDevice::fillDeviceInfo(dev_info_t &devInfo) {
    devInfo.hardwareType = DECODE_HDW_TYPE(hardwareId);
    devInfo.hardwareVer[0] = DECODE_HDW_MAJOR(hardwareId);
    devInfo.hardwareVer[1] = DECODE_HDW_MINOR(hardwareId);
    devInfo.serialNumber = sn;

    return 0;
}

