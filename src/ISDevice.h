/**
 * @file ISDevice.h
 * @brief Represents a single, physical Inertial Sense device (IMX or GPX) bound to a port: owns
 * its identity (dev_info_t), synchronized flash/system-parameter state, and the comm-protocol
 * handlers that parse data received from it. DeviceManager owns a collection of these; a caller
 * that only ever talks to one device can use a standalone ISDevice directly without DeviceManager.
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef INERTIALSENSESDK_ISDEVICE_H
#define INERTIALSENSESDK_ISDEVICE_H

#include <chrono>
#include <functional>
#include <memory>

#include "json.hpp"
#include "core/msg_logger.h"
#include "ChronoStat.h"
#include "DeviceLog.h"
#include "ISDeviceCal.h"
#include "ISFirmwareUpdater.h"
#include "protocol/FirmwareUpdate.h"
#include "protocol_nmea.h"

extern "C"
{
    // [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
    #include "data_sets.h"
    #include "com_manager.h"
    #include "core/base_port.h"
}

#define BOOTLOADER_HANDSHAKE_COUNT  10
#define BOOTLOADER_HANDSHAKE_DELAY  10

#define PRINT_DEBUG 0
#if PRINT_DEBUG
#define DEBUG_PRINT(...)    printf("L%d: ", __LINE__); printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

class ISFirmwareUpdater;
class cISLogger;

#if !defined(cISLogger)
    class cDeviceLog;
    typedef std::shared_ptr<cDeviceLog> logger_handle_t;
#endif

/** @brief Represents a single physical Inertial Sense device (IMX or GPX) and its bound port, identity, and synchronized state. */
class ISDevice : public std::enable_shared_from_this<ISDevice> {
public:

    /** @brief Generic result states for non-blocking/async operations (e.g. validateAsync()). */
    enum AsyncState {
        ASYNC_STATE__TIMEOUT     = -2,          //!< specific failure state indicating a lack of response within a timeperiod
        ASYNC_STATE__FAILURE     = -1,          //!< general failure state indicating an error condition, but otherwise a completed async cycle
        ASYNC_STATE__PENDING     = 0,           //!< indicates that the async operation is still in progress, and should be called again soon
        ASYNC_STATE__SUCCESS     = 1,           //!< indicates that the async operation was successful, and no further actions are necessary
    };

    /** @brief Bitmask flags controlling the verbosity/format of getName()/getDescription()/getFirmwareInfo() output. */
    enum DevInfoFormatFlags : uint16_t {
        // Description Options
        OMIT_FIRMWARE_VERSION    = 0x0001,      //!< suppresses output of the firmware version
        OMIT_PORT_NAME           = 0x0002,      //!< suppresses output of the device port
        COMPACT_HARDWARE_VER     = 0x0004,      //!< forces hiding digits 3 & 4 of the hardware version number, digits 1 & 2 are always shown
        COMPACT_SERIALNO         = 0x0008,      //!< disables zero-padding of the serial number
        COMPACT_BUILD_TYPE       = 0x0010,      //!< formats the build-type (when the firmware version is show) as a single character

        // Version Options
        OMIT_COMMIT_HASH         = 0x0100,      //!< suppresses the output of the commit hash/dirty status
        OMIT_BUILD_KEY           = 0x0200,      //!< suppresses the output of the build host key and number
        OMIT_BUILD_DATE          = 0x0400,      //!< suppresses the output of the build date
        OMIT_BUILD_TIME          = 0x0800,      //!< suppresses the output of the build time
        OMIT_BUILD_MILLIS        = 0x1000,      //!< suppresses the output of the build milliseconds when not zero

        ESSENTIAL_FIRMWARE_INFO  = (ISDevice::OMIT_COMMIT_HASH | ISDevice::OMIT_BUILD_KEY | ISDevice::OMIT_BUILD_MILLIS | ISDevice::OMIT_BUILD_DATE | ISDevice::OMIT_BUILD_TIME),
    };

    const static ISDevice invalidRef;   //!< a shared invalid/placeholder ISDevice instance, usable where a non-null reference is required but no real device is bound

    /** @return the formatted unique-identifier string (see getIdAsString()) for the given devInfo, without requiring an ISDevice instance. */
    static std::string getIdAsString(const dev_info_t& devInfo);
    /** @brief Formats the device-name string (see getName()) for the given devInfo, without requiring an ISDevice instance. @param devInfo the device info to format @param flags a DevInfoFormatFlags bitmask @return the formatted name string */
    static std::string getName(const dev_info_t& devInfo, int flags = (COMPACT_SERIALNO | COMPACT_HARDWARE_VER));
    /** @brief Formats the device-description string (see getDescription()) for the given devInfo, without requiring an ISDevice instance. @param devInfo the device info to format @param flags a DevInfoFormatFlags bitmask @return the formatted description string */
    static std::string getDescription(const dev_info_t& devInfo, int flags = (COMPACT_SERIALNO | COMPACT_HARDWARE_VER | ESSENTIAL_FIRMWARE_INFO));
    /** @brief Formats the firmware-info string (see getFirmwareInfo()) for the given devInfo, without requiring an ISDevice instance. @param devInfo the device info to format @param flags a DevInfoFormatFlags bitmask @return the formatted firmware-info string */
    static std::string getFirmwareInfo(const dev_info_t &devInfo, int flags = 0);

    /** @brief Constructs an unidentified device bound to the given port (if any); devInfo remains empty until validate()/step() populates it. */
    explicit ISDevice(is_hardware_t _hdwId = IS_HARDWARE_TYPE_UNKNOWN, port_handle_t _port = nullptr) {
        // std::cout << "Creating ISDevice for port " << portName(_port) << " " << this << std::endl;
        hdwId = _hdwId;
        devInfo = { 0 };
        imxFlashCfg.checksum = 0xFFFFFFFF;
        gpxFlashCfg.checksum = 0xFFFFFFFF;
        sysParams.flashCfgChecksum = 0xFFFFFFFF;        // Set invalid checksum to trigger synchronization
        gpxStatus.flashCfgChecksum = 0xFFFFFFFF;        // Set invalid checksum to trigger synchronization
        assignPort(_port);
    }

    /** @brief Constructs a device pre-seeded with a known devInfo (e.g. from discovery), bound to the given port (if any). */
    explicit ISDevice(const dev_info_t& _devInfo, port_handle_t _port = nullptr) {
        // std::cout << "Creating ISDevice for port " << portName(_port) << " " << this << std::endl;
        hdwId = ENCODE_DEV_INFO_TO_HDW_ID(_devInfo);
        devInfo = _devInfo;
        imxFlashCfg.checksum = 0xFFFFFFFF;
        gpxFlashCfg.checksum = 0xFFFFFFFF;
        sysParams.flashCfgChecksum = 0xFFFFFFFF;        // Set invalid checksum to trigger synchronization
        gpxStatus.flashCfgChecksum = 0xFFFFFFFF;        // Set invalid checksum to trigger synchronization
        assignPort(_port);
    }

    /** @brief Copy constructor; rebinds the copy's port callbacks to reference the new instance rather than src. */
    ISDevice(const ISDevice& src) : std::enable_shared_from_this<ISDevice>(src), devLogger(src.devLogger) {
        // std::cout << "Creating ISDevice copy from " << ISDevice::getIdAsString(src.devInfo)  << " " << this << std::endl;

        hdwId = src.hdwId;
        devInfo = src.devInfo;
        imxFlashCfg = src.imxFlashCfg;
        gpxFlashCfg = src.gpxFlashCfg;
        sysParams = src.sysParams;
        gpxStatus = src.gpxStatus;
        sysCmd = src.sysCmd;
        // devLogger = src.devLogger.get();
        closeStatus = src.closeStatus;

        defaultCbs = src.defaultCbs;
        defaultCbs.context = this;

        port = src.port;
        if (portType(port) & PORT_TYPE__COMM) {      // this is pretty much always true, because you can't really have an ISDevice that isn't a COMM port, but just in case..
            COMM_PORT(port)->comm.cb.context = this; // we need to update the port's callback to reference the copy's instance, not the original
        }
        // NOTE: Don't reconfigure any other callbacks; since this originated from an ISDevice, function pointers should still be valid. We just need the newer context.
    }

    /** @brief Destructor; restores the port's original callbacks (if still bound to this instance) and frees the firmware updater, but does NOT close/invalidate the port itself. */
    virtual ~ISDevice() {
        if (m_calibration) {
            // Log silent failures on shutdown; unique_ptr frees the cal data.
            log_warn(IS_LOG_ISDEVICE, "[%s] ISDevice destroyed with async calibration upload still in flight at step %d.", getIdAsString().c_str(), m_calUploadState);
            m_calUploadResult = IS_OP_CLOSED;
        }

        devInfo = {};

        if (port) {
            // NOTE: DO NOT CLOSE or otherwise modify the associated port.  The only thing we should be doing with the port,
            // is making sure its callbacks don't call back into this instance.
            if (portType(port) & PORT_TYPE__COMM) {
                if (COMM_PORT(port)->comm.cb.context == this) {
                    COMM_PORT(port)->comm.cb = originalCbs; // return the original callbacks/contexts, but only if the context matches us
                }
            }
            port = nullptr;
        }

        if (fwUpdater) {
            delete fwUpdater;
            fwUpdater = nullptr;
        }
    }

    /** @brief Copy-assignment; copies identity/config/state from src (but not port callback bindings -- see the copy constructor). */
    ISDevice& operator=(const ISDevice& src) {
        port = src.port;
        hdwId = src.hdwId;
        devInfo = src.devInfo;
        imxFlashCfg = src.imxFlashCfg;
        gpxFlashCfg = src.gpxFlashCfg;
        sysParams = src.sysParams;
        gpxStatus = src.gpxStatus;
        imxFlashCfgUploadTimeMs = src.imxFlashCfgUploadTimeMs;
        gpxFlashCfgUploadTimeMs = src.gpxFlashCfgUploadTimeMs;
        imxFlashCfgUpload = src.imxFlashCfgUpload;
        gpxFlashCfgUpload = src.gpxFlashCfgUpload;
        sysParams = src.sysParams;
        sysCmd = src.sysCmd;
        // devLogger = src.devLogger.get();
        closeStatus = src.closeStatus;
        return *this;
    }

    /** @return a shared_ptr<T> aliasing this device, dynamic-cast to subclass T; null if this instance is not actually a T. */
    template<typename T>
    std::shared_ptr<T> as() { return std::dynamic_pointer_cast<T>(shared_from_this()); }

    /** @return a reference to this device dynamic-cast to subclass T; undefined behavior (null dereference) if this instance is not actually a T. */
    template<typename T>
    T& asRef() { return *(std::dynamic_pointer_cast<T>(shared_from_this())); }

    /**
     * Generates a uint64_t which encodes the hardware type, hardware version, and hardware serial number, representing a unique device
     * @param devInfo the device info to encode
     * @return a uint64_t uniquely identifying the device (encoded hdwId in bits 48-63, serial number in bits 0-31)
     */
    static uint64_t getUniqueId(const dev_info_t& devInfo) { return ((uint64_t)ENCODE_DEV_INFO_TO_HDW_ID(devInfo) << 48) | devInfo.serialNumber; }

    /** @return this device's unique Id; see getUniqueId(const dev_info_t&). */
    uint64_t getUniqueId() { return getUniqueId(this->devInfo); }

    /**
     * Parses a device identifier string into a unique ID (hdwId << 48 | serialNumber).
     * Accepted formats: "IMX-5.0::SN129495", "IMX-5.0:129495", "GPX-1.0:SN42", "SN129495", "129495"
     * If no hardware type is specified, IS_HARDWARE_ANY is used.
     * @param str the device identifier string to parse
     * @return unique ID, or 0 on parse failure
     */
    static uint64_t parseDeviceIdString(const std::string& str);

    /**
     * Binds the specified port to this device. Reconfigures the port handler to call back
     * into this device instance, and reinitializes the underlying ISComm instance and
     * buffers.
     * @param port the port to bind; pass an invalid/PORT_FLAG__NO_ISDEVICE port to fail the assignment
     * @return true if the port was successfully assigned, false if newPort is flagged PORT_FLAG__NO_ISDEVICE
     */
    bool assignPort(port_handle_t port);

    /**
     * @return true is this ISDevice has a valid, and open port
     */
    bool isConnected() const {
        bool valid = portIsValid(port);
        bool comPort = portType(port) & PORT_TYPE__COMM;    // Not sure that this is entirely required; but it doesn't really hurt currently
        bool open = portIsOpened(port);
        return (valid && comPort && open);
    }

    /**
     * Connects the bound port to the device, if the port is valid and of PORT_TYPE__COMM
     * Can be overridden to provide custom configuration, etc on connection - just remember
     *  to call back into ISDevice::connect() in your new method.
     *
     * Asynchronous transports (TCP) do not finish connecting within a single portOpen() call:
     * tcpPortOpen() returns PORT_ERROR__NONE while the handshake is still in flight and leaves
     * PORT_FLAG__OPENED clear, expecting the caller to keep polling. This function therefore
     * re-invokes portOpen() until the port actually reports open, or @p openTimeoutMs elapses.
     * Serial ports set PORT_FLAG__OPENED on the first call and are unaffected.
     *
     * @param revalidate    if true causes the device to validate after connecting (default = false)
     * @param openTimeoutMs how long to keep polling portOpen() for the port to actually report
     *                      open. Loopback/LAN handshakes complete in one or two polls; the default
     *                      only bounds the pathological (unreachable host) case. Pass a smaller
     *                      value from step()-driven loops that retry on their own.
     * @return true if the port is genuinely open (and, when @p revalidate is set, validated),
     *         otherwise false
     */
    virtual bool connect(bool revalidate = false, uint32_t openTimeoutMs = 500);

    /**
     * Disconnects/closes the bound port to the device, if the port is VALID
     * Can be overridden to provide custom tear-down, etc on disconnect - just remember
     *  to call back into ISDevice::connect() in your new method.
     * @param invalidate if true, the associated port will be invalidated after closing.
     *   Invalidating the port will require rediscovering/revalidating the port again before
     *   it can be used to communicate with the device again. This is most often done when
     *   issuing a device reset on a transient port type, such as a TCP or USB port type.
     *   In most circumstances in which a port is closed, you DO NOT want to invalidate it,
     *   thus false is the default and likely correct option.
     * @return true if successful, otherwise false
     */
    virtual bool disconnect(bool invalidate = false) {
        bool closed = (portClose(port) == PORT_ERROR__NONE);
        if (invalidate)
            portInvalidate(port);
        return closed;
    }

    /**
     * Simple utility test to confirm if this device matches the specified HdwId and optional Hdw serial number
     * @param hdwId_ the Hardware ID to match against
     * @param serialNo the serial number to further match against, if not zero (default is zero)
     * @return true if this device matches the criteria, otherwise false;
     *
     * Note that the HdwId is a bitwise check - meaning that a IS_HARDWARE_IMX and a IS_HARDWARE_IMX_5_0 will
     * both match a device which is reporting as IS_HARDWARE_IMX_5_0
     */
    inline bool matchesHdwId(uint16_t hdwId_, uint32_t serialNo = 0) const {
        return ((hdwId == IS_HARDWARE_ANY) || ((hdwId & hdwId_) == hdwId)) &&
                    ((serialNo == 0) || (serialNo == devInfo.serialNumber));
    }

    /**
     * @return true if the device has valid, minimal required devInfo values sufficient to indicate that it genuinely
     * identifies an Inertial Sense device.
     */
    bool hasDeviceInfo() const {
        return (hdwId != IS_HARDWARE_TYPE_UNKNOWN) && (hdwId != IS_HARDWARE_ANY) && (devInfo.hdwRunState != HDW_STATE_UNKNOWN) && (devInfo.serialNumber != 0) && (devInfo.hardwareType != 0) && (devInfo.protocolVer[0] == PROTOCOL_VERSION_CHAR0);
    }

    /**
     * Specifies a handler for protocol messages, which will be called when any message is successfully parsed. This 
     * function will return the previously registered handler. It is the callers responsibility to restore the previous 
     * handler, when this handler is no longer required.
     * @param cbHandler a function pointer or lambda function which will be called when any Data packet is received
     * @return the previously registered handler, if any.
     */
    pfnIsCommHandler registerAllHandler(pfnIsCommHandler cbHandler);

    /**
     * Specifies an alternate handler for Inertial Sense "Data" binary protocol messages, which will be called when
     * any DID message is successfully parsed. This function will return the previously registered handler. It is the
     * callers responsibility to restore the previous handler, when this handler is no longer required.
     * @param cbHandler a function pointer or lambda function which will be called when an ISB Data packet is received
     * @return the previously registered handler, if any.
     */
    pfnIsCommIsbDataHandler registerIsbDataHandler(pfnIsCommIsbDataHandler cbHandler);

    /**
     * @note declared but not currently defined anywhere in the SDK; registerIsbDataHandler() is the implemented equivalent for ISB Data messages.
     * @param cbHandler a function pointer or lambda function intended to be called when an ISB Ack message is received
     * @returns the previously registered handler, if any.
     */
    pfnIsCommIsbDataHandler registerIsbAckHandler(pfnIsCommIsbDataHandler cbHandler);

    /**
     * Specifies an alternate handler for non-Inertial Sense protocol messages, which will be called when
     * any DID message is successfully parsed. This function will return the previously registered handler. It is the
     * callers responsibility to restore the previous handler, when this handler is no longer required.
     * @param ptype the PTYPE_* protocol type indicating which protocol will trigger a callback to this handler
     * @param cbHandler a function pointer or lambda function which will be called when an ISB Data packet is received
     * @returns the previously registered handler, if any.
     */
    pfnIsCommGenMsgHandler registerProtocolHandler(int ptype, pfnIsCommGenMsgHandler cbHandler);

    /**
     * Called to process any pending, received data on the bound port, and call any registered handlers for any valid
     * packets which are parsed from that data. Additionally, this call will manage other comm-related tasks such as
     * data/config synchronization to the device, as well as progressing firmware updates, etc.  This function should
     * be called a regular interval fast enough to prevent received data from overflowing the port's RX buffer
     * (typically a 1ms interval or faster, for a 921600 Serial Baud rate)).
     * @return false if the port is invalid or closed, otherwise true. Note that 'true' does NOT provide any indication
     *  of data parsed, etc. Only that the port was valid, and that the maintenance functions were called.
     */
    virtual bool step();

    /**
     * @param ptype the type (_PTYPE_*) of the packet query. Default to _PTYPE_NONE (any packet type)
     * @return returns the number of milliseconds since a message of this type was received. If ptype == _PTYPE_NONE
     *   this will return the minimum age of all packet types.
     */
    uint32_t millisSinceLastRx(int ptype = _PTYPE_NONE);

    /**
     * @returns the name of the currently bound port, or an empty string if none.
     */
    std::string getPortName() const { return (port && portIsValid(port) && portName(port) ? portName(port) : ""); }

    /**
     * @returns a formatted string which can be used to uniquely identify the hardware associated with this device. The
     * formatted string appears as "<HdwType>-<HdwVer.Maj>.<HdrVer.Min>::SN<SerialNo>". This is sufficient to be used
     * in hashing or other comparison functions to identify a specific device.
     */
    std::string getIdAsString() const;

    /**
     * @returns a formatted string similar to getIdAsString(), but slightly more human-friendly.  The formatted string
     * appears as "SN<SerialNo> (<HdwType>-<HdwVer[0]>.<HdrVer[1]>[.<HdrVer[2]>.<HdrVer[3]>])"
     */
    std::string getName(int flags = (COMPACT_SERIALNO | COMPACT_HARDWARE_VER)) const;

    /**
     * Returns a string representing the device firmware, as reported by its devInfo struct, with varying levels of
     * detail, depending on the format flags specified.
     * @param flags an integer bitmask derived from DevInfoFormatFlags which alters the output format
     * @return the formatted Firmware Information string
     */
    std::string getFirmwareInfo(int flags = 0) const;

    /**
     * @returns a formatted string that completely describes the device as a concatenation of the following calls:
     *   getName() + getFirmwareInfo(1) + portName()
     */
    std::string getDescription(int flags = 0) const;

    /**
     * Registers this device with the specified ISLogger instance, allowing the logger instance to capture and
     * log the data received from this device.  The format, rules and options for data logging are managed by
     * the ISLogger instance.
     * @param logger the cISLogger instance to register this device with
     */
    void registerWithLogger(cISLogger* logger);

    /**
     * @returns true is the device is indicated that a reset is required; this state SHOULD be acted on by resetting the device to ensure that it is operating as expected
     */
    bool isResetRequired() { return ((devInfo.hardwareType == IS_HARDWARE_TYPE_IMX) && (sysParams.hdwStatus & HDW_STATUS_SYSTEM_RESET_REQUIRED)) ||
                                    ((devInfo.hardwareType == IS_HARDWARE_TYPE_GPX) && (gpxStatus.hdwStatus & GPX_HDW_STATUS_SYSTEM_RESET_REQUIRED)); }

    /**
     * Immediately issues s SysCmd to instruct the device to perform a software reset as soon as reasonably possible.
     * Note that there is no acknowledgement or other indication that the device received the reset command before the
     * device is reset. In order to confirm that the device was successfully reset, you should compare the upTime of
     * the device before and after the reset is issued.
     * @return true if the request was successfully sent, false if the action was not able to be performed.
     */
    bool softwareReset();

    /**
     * @returns true if reset() was called recently, and we are waiting for the device to return.
     */
    bool isResetPending() { return (current_timeMs() - lastResetTime) < resetRequestThreshold; }

    /**
     * Fetches (if not previously fetched), the devices manufacturing info and populates it into the passed reference.
     * @param manfInfo a reference to the manufacturing_info_t struct to be populated
     * @param timeoutMs the maximum amount of time to wait for the manufacturing info response from the device
     * @return true if the manfInfo was successfully populated, otherwise false (ie, port invalid, invalid device, timeout, etc);
     */
    bool manufacturingInfo(manufacturing_info_t& manfInfo, uint32_t timeoutMs = 100);

    // Core Interface Functions - these should be the only calls which call into the ComManager functions directly,
    //     these are essentially the basis of all comms to the device, with few exceptions.

    /** @brief Sends a raw ISB packet (pktInfo/did/data) to the device. @return bytes sent, or -1 if not connected or the device is in the bootloader. */
    int Send(uint8_t pktInfo, void *data=NULL, uint16_t did=0, uint16_t size=0, uint16_t offset=0) { std::lock_guard<std::recursive_mutex> lock(portMutex); return (isConnected() && devInfo.hdwRunState != HDW_STATE_BOOTLOADER) ? comManagerSend(port, pktInfo, data, did, size, offset) : -1; }
    /** @brief Sends a raw byte buffer directly to the device's port, bypassing ISB packet framing. @return bytes sent, or -1 if not connected or the device is in the bootloader. */
    int SendRaw(const void* data, uint32_t length) { std::lock_guard<std::recursive_mutex> lock(portMutex); return (isConnected() && devInfo.hdwRunState != HDW_STATE_BOOTLOADER) ? comManagerSendRaw(port, data, length) : -1; }
    /** @brief Sends a DID_* data set to the device (a "set data" request). @return bytes sent, or -1 if not connected or the device is in the bootloader. */
    int SendData(eDataIDs dataId, const void* data, uint32_t length, uint32_t offset = 0) { std::lock_guard<std::recursive_mutex> lock(portMutex); return (isConnected() && devInfo.hdwRunState != HDW_STATE_BOOTLOADER) ? comManagerSendData(port, data, dataId, length, offset) : -1; }
    /** @brief Requests the device broadcast (or fetch once, if period is 0) the given DID. No-op if not connected or the device is in the bootloader. */
    void GetData(eDataIDs dataId, uint16_t length=0, uint16_t offset=0, uint16_t period=0) { std::lock_guard<std::recursive_mutex> lock(portMutex); if ((isConnected() && devInfo.hdwRunState != HDW_STATE_BOOTLOADER)) comManagerGetData(port, dataId, length, offset, period); }

    /** @brief Requests the device broadcast a preset RMC (real-time message controller) bundle of messages. No-op if not connected or the device is in the bootloader. */
    void BroadcastBinaryDataRmcPreset(uint64_t rmcPreset, uint32_t rmcOptions) { std::lock_guard<std::recursive_mutex> lock(portMutex); if ((isConnected() && devInfo.hdwRunState != HDW_STATE_BOOTLOADER)) comManagerGetDataRmc(port, rmcPreset, rmcOptions); }
    /** @brief Stops the device from broadcasting the given DID. No-op if not connected or the device is in the bootloader. */
    void DisableData(eDataIDs dataId) { std::lock_guard<std::recursive_mutex> lock(portMutex); if ((isConnected() && devInfo.hdwRunState != HDW_STATE_BOOTLOADER)) comManagerDisableData(port, dataId); }

    // TODO?? Replace the above with these? (and probably move to ISDevice.cpp)  -- these attempt to reduce dependency on comManager*() functions.
    /*
    int Send(uint8_t pktInfo, void *data=NULL, uint16_t did=0, uint16_t size=0, uint16_t offset=0) {
        std::lock_guard<std::recursive_mutex> lock(portMutex);
        if (!isConnected()) return PORT_ERROR__NOT_CONNECTED;
        if (devInfo.hdwRunState == HDW_STATE_BOOTLOADER) return PORT_ERROR__NOT_SUPPORTED;
        return comManagerSend(port, pktInfo, data, did, size, offset)  < 0 ? -1 : 0;
    }

    **
     * Send a raw binary buffer, byte-for-byte, to this device
     * @param data a pointer to a block of memory to be sent
     * @param length the number of bytes to be sent
     * @return >=0 indicates the number of bytes actually sent, <0 is one of PORT_ERROR__*. Note that PORT_ERROR__NONE == 0 is not an error, but indicates no bytes where sent
     *
    int SendRaw(const void* data, uint32_t length) {
        std::lock_guard<std::recursive_mutex> lock(portMutex);
        if (!isConnected()) return PORT_ERROR__NOT_CONNECTED;
        return portWrite(port, static_cast<const uint8_t *>(data), length);
    }

    int SendData(eDataIDs dataId, const void* data, uint32_t length, uint32_t offset = 0) {
        std::lock_guard<std::recursive_mutex> lock(portMutex);
        if (!isConnected()) return PORT_ERROR__NOT_CONNECTED;
        if (devInfo.hdwRunState == HDW_STATE_BOOTLOADER) return PORT_ERROR__NOT_SUPPORTED;
        return is_comm_write(port, PKT_TYPE_SET_DATA, dataId, length, offset, data) < 0 ? -1 : 0;
    }

    void GetData(eDataIDs dataId, uint16_t length=0, uint16_t offset=0, uint16_t period=0) {
        std::lock_guard<std::recursive_mutex> lock(portMutex);
        if (isConnected() && (devInfo.hdwRunState != HDW_STATE_BOOTLOADER))
            comManagerGetData(port, dataId, length, offset, period);
    }

    void BroadcastBinaryDataRmcPreset(uint64_t rmcPreset, uint32_t rmcOptions) {
        std::lock_guard<std::recursive_mutex> lock(portMutex);
        if ((isConnected() && devInfo.hdwRunState != HDW_STATE_BOOTLOADER))
            comManagerGetDataRmc(port, rmcPreset, rmcOptions);
    }
    void DisableData(eDataIDs dataId) {
        std::lock_guard<std::recursive_mutex> lock(portMutex);
        if ((isConnected() && devInfo.hdwRunState != HDW_STATE_BOOTLOADER))
            comManagerDisableData(port, dataId);
    }
    */
    // Convenience Functions

    /**
     * Requests that this device broadcast the requested DID are the specified period
     * @param dataId the DID to be broadcast at periodic intervals
     * @param periodMultiple the period multiple (NOT a frequency). If 0 (default), this will request a one-shot, also effectively stopping any existing broadcasts
     * @return true if the request was successfully sent, otherwise false (ie, port invalid, invalid device, etc)
     */
    bool BroadcastBinaryData(uint32_t dataId, int periodMultiple = 0);

    /** @brief Sends a raw NMEA sentence to the device. @return bytes sent, or a negative PORT_ERROR__* on failure. */
    int SendNmea(const std::string& nmeaMsg);
    /** @brief Sends the NMEA "query device info" command. @return bytes sent, or a negative PORT_ERROR__* on failure. */
    int QueryDeviceInfo() { return SendRaw(NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE); }
    /** @brief Sends the NMEA "save persistent messages to flash" command. @return bytes sent, or a negative PORT_ERROR__* on failure. */
    int SavePersistent() { return SendRaw(NMEA_CMD_SAVE_PERSISTENT_MESSAGES_TO_FLASH, NMEA_CMD_SIZE); }

    [[deprecated("Use ISDevice::softwareReset() instead")]]
    int SoftwareReset() { return (int) softwareReset(); }

    /** @brief Configures which message types are logged/reported for a given target and port mask, above the given priority level. @return bytes sent, or a negative PORT_ERROR__* on failure. */
    int SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel);
    /** @brief Sends a system command (system_command_t) to the device. @return bytes sent, or a negative PORT_ERROR__* on failure. */
    int SetSysCmd(const uint32_t command);
    /** @brief Sends the NMEA "stop all broadcasts" command, either for the current port or all ports. @return bytes sent, or a negative PORT_ERROR__* on failure. */
    int StopBroadcasts(bool allPorts = false) { return SendRaw((allPorts ? NMEA_CMD_STOP_ALL_BROADCASTS_ALL_PORTS : NMEA_CMD_STOP_ALL_BROADCASTS_CUR_PORT), NMEA_CMD_SIZE); }

    /** @brief Checks whether this device has one or more IMX flash-config writes still pending completion. @param ageSinceLastPendingWrite populated with the age (ms) of the oldest pending write. @return true if a write is pending. */
    bool hasPendingImxFlashWrites(uint32_t& ageSinceLastPendingWrite);
    /** @brief Blocks (calling step() internally) until all pending IMX flash writes complete or timeoutMs elapses. @return true if all pending writes completed before the timeout. */
    bool waitForImxFlashWrite(uint32_t timeoutMs);

    /** @brief Attempts to acquire portMutex without blocking. @return true if the lock was acquired. */
    bool lockPort() { return portMutex.try_lock(); }
    /** @brief Releases portMutex previously acquired via lockPort(). */
    void unlockPort() { return portMutex.unlock(); }

    /** @return a const reference to this device's dev_info_t. */
    const dev_info_t& DeviceInfo() { return devInfo; }
    /** @return a const reference to this device's sys_params_t (IMX system parameters, updated as DID_SYS_PARAMS messages arrive). */
    const sys_params_t& SysParams() { return sysParams; }

    /**
     * @brief IMX/GPX Flash Configuration Synchronization
     *
     * The InertialSense class maintains synchronization between the host's local flash
     * configuration and the configuration stored on the connected IMX or GPX device.
     *
     * Key Concepts:
     * - Each device maintains local copies of flash configuration:
     *      - IMX: device.imxFlashCfg
     *      - GPX: device.gpxFlashCfg
     * - The device also reports its flash configuration checksum via:
     *      - IMX: device.sysParams.flashCfgChecksum
     *      - GPX: device.gpxStatus.flashCfgChecksum
     *
     * Synchronization Mechanism:
     * - Periodically (every SYNC_FLASH_CFG_CHECK_PERIOD_MS), SyncFlashConfig() compares
     *   the local checksum with the device-reported checksum.
     * - If mismatched, the full flash configuration is requested from the device.
     * - Uploads are initiated via SetImxFlashConfig() / SetGpxFlashConfig(), which:
     *      - Compute and send only the changed regions of the configuration.
     *      - Update tracking variables (upload time, expected checksum).
     * - The Update() function drives both synchronization and upload completion.
     *
     * Validation:
     * - ImxFlashConfigSynced() / GpxFlashConfigSynced() return true if:
     *      - Local and device checksums match.
     *      - No upload is pending.
     *      - No upload failure is detected.
     * - ImxFlashConfigUploadFailure() / GpxFlashConfigUploadFailure() detect if a
     *   configuration update was rejected or not yet received by the device.
     * - WaitForImxFlashCfgSynced() / WaitForGpxFlashCfgSynced() can be used to block
     *   until synchronization is complete.
     */

    /**
     * Populates the passed reference flashCfg with the locally synchronized copy of the remote device's config.
     * @param flashCfg_ a reference to a nvm_flash_cfg_t (IMX) or gpx_flash_cfg_t (GPX) struct to be populated
     * @param timeout the maximum time (ms) to wait for the flash config to be received/synchronized, if not already available
     * @returns true if the flashCfg has been synchronized with the device (and can thus be trusted), otherwise false.
     */
    bool ImxFlashConfig(nvm_flash_cfg_t& flashCfg_, uint32_t timeout = 2500);
    /** @copydoc ImxFlashConfig */
    bool GpxFlashConfig(gpx_flash_cfg_t& flashCfg_, uint32_t timeout = 2500);

    /**
     * Uploads the provided flashCfg to the remove device, but makes NO checks that it was successfully synchronized.
     * This method attempt to "intelligently" upload only the portions of the flashCfg that has actually changed, reducing
     * traffic and minimizing the risk of a sync-failure due to elements which maybe programmatically changed, however it
     * may make multiple sends, if the new and previous configurations have non-contiguous modifications.
     * Use WaitForImxFlashCfgSynced() or SetImxFlashCfgAndConfirm() to actually confirm that the new config was applied to the
     * device correctly.
     * @param flashCfg_ the new flash_config to upload
     * @return true if the ANY of the changes failed to send to the remove device.
     */
    bool SetImxFlashConfig(nvm_flash_cfg_t& flashCfg_);
    /** @copydoc SetImxFlashConfig */
    bool SetGpxFlashConfig(gpx_flash_cfg_t& flashCfg_);

    /**
     * Indicates whether the current IMX flash config has been downloaded, is available via ImxFlashConfig(),
     * and matches the device's most recently reported sysParams.flashCfgChecksum with no upload in progress or failed.
     * @return true if the flash config is valid, currently synchronized, otherwise false.
     */
    bool ImxFlashConfigSynced();
    /** @copydoc ImxFlashConfigSynced */
    bool GpxFlashConfigSynced();

    /**
     * @returns true if the local flashConfig upload was either not received or rejected.
     * TODO: this REALLY only does a checksum comparison of the sysParams and the uploaded flashCfg to confirm they match.
     *  Maybe this is enough, but this function name maybe
     */
    bool ImxFlashConfigUploadFailure();
    /** @copydoc ImxFlashConfigUploadFailure */
    bool GpxFlashConfigUploadFailure();

    /**
     * A blocking function call which waits until both a DID_FLASH_CFG and DID_SYS_PARAMS have
     * been received with a matching flashCfg checksum, ensuring a valid copy of the device's flash
     * configuration has been synchronized locally.
     * @param forceSync if true, invalidates any existing checksum, ensuring both messages must be received and validated again
     * @param timeout the maximum time (ms) to wait for the synchronization to occur, before returning false
     * @return true if both the flashCfg.checksum and sysParams.flashCfgChecksum match (and neither are zero)
     */
    bool WaitForImxFlashCfgSynced(bool forceSync = false, uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS);
    /** @copydoc WaitForImxFlashCfgSynced */
    bool WaitForGpxFlashCfgSynced(bool forceSync = false, uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS);

    /**
     * A blocking call which uploads and then waits for synchronization confirmation that the new configuration was applied.
     * As part of the validation/synchronization, it downloads the newest FlashCfg from the device and performs a byte-for-byte
     * comparison* to ensure it was uploaded/downloaded correctly.  This could fail where the WaitForImxFlashCfgSynced() might pass,
     * because some parts of the flashCfg are programmatically set to reflect state. For example, sending a rtkConfig = 0x08,
     * may return a rtkConfig of 0x00400008 because the 0x4 reflects that it was persisted (or something like that).
     * @param flashCfg the config to upload (and later match against the downloaded firmware)
     * @param timeout a timeout value for how long to wait for the new flash to sync/download before failing
     * @param waitForWrite if true (default is false) will wait for confirmation that the flash was written to flash before returning.
     * @return true if the new config was uploaded, synced, downloaded and matched with the original flashCfg, otherwise false
     */
    bool SetImxFlashCfgAndConfirm(nvm_flash_cfg_t& flashCfg, uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS, bool waitForWrite = false);
    /**
     * @brief GPX counterpart to SetImxFlashCfgAndConfirm(); see its doc. Note this overload has no waitForWrite parameter.
     * @param flashCfg the config to upload (and later match against the downloaded firmware)
     * @param timeout a timeout value for how long to wait for the new flash to sync/download before failing
     * @return true if the new config was uploaded, synced, downloaded and matched with the original flashCfg, otherwise false
     */
    bool SetGpxFlashCfgAndConfirm(gpx_flash_cfg_t& flashCfg, uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS);

    /**
     * @brief Serializes the locally synchronized IMX flash config to a YAML file.
     * @param path Path to the YAML flash config file to write
     * @return true on success, false on failure (e.g. could not read the local flash config, or YAML serialization failed).
     */
    bool SaveImxFlashConfigToFile(std::string path);
    /** @copydoc SaveImxFlashConfigToFile */
    bool SaveGpxFlashConfigToFile(std::string path);

    /**
     * @brief Reads a YAML flash config file and uploads it to the device via SetImxFlashConfig().
     * @param path Path to the YAML flash config file to read
     * @return true on success, false on failure (e.g. file/YAML parse error, or the upload failed).
     */
    bool LoadImxFlashConfigFromFile(std::string path);
    /** @copydoc LoadImxFlashConfigFromFile */
    bool LoadGpxFlashConfigFromFile(std::string path);

    /**
     * Initiates an asynchronous calibration upload. The device takes ownership of the
     * supplied calibration object; subsequent step() ticks drive the per-step protocol
     * until completion. Poll getCalibrationUploadResult() (or isCalibrationUploadInProgress())
     * for status.
     * @param cal heap-allocated calibration data; ownership transfers to the device on IS_OP_OK.
     *            On any other return value the caller retains ownership.
     * @return IS_OP_OK on accept (upload kicked off);
     *         IS_OP_IN_PROGRESS if a prior upload is still in flight on this device (caller retains ownership);
     *         IS_OP_CLOSED if the device is not connected (caller retains ownership).
     */
    is_operation_result UploadImxCalibrationAsync(std::unique_ptr<ISDeviceCal> cal);

    /**
     * @return result of the most recent (or currently in-flight) async calibration upload.
     *         IS_OP_NONE if no upload has ever been initiated on this device.
     *         IS_OP_IN_PROGRESS if an upload is in flight.
     *         IS_OP_OK if the last upload completed successfully.
     *         IS_OP_ERROR if the last upload failed at the protocol layer.
     *         IS_OP_CLOSED if the last upload was aborted because the port closed mid-upload.
     */
    is_operation_result getCalibrationUploadResult() const { return m_calUploadResult; }

    /** @return true if an async calibration upload is currently in flight on this device. */
    bool isCalibrationUploadInProgress() const { return m_calUploadResult == IS_OP_IN_PROGRESS; }

    /** @brief Synchronous calibration upload: blocks in a loop directly driving ISDeviceCal::uploadSensorCalStep() (independent of step()/UploadImxCalibrationAsync()) until the upload completes or fails. @return true if the upload completed successfully. */
    bool UploadImxCalibration(ISDeviceCal& cal);

    /**
     * @brief UploadImxCalibrationFromFile
     * @param path - Path to JSON calibration file
     * @return true for success, false for failure.
     */
    bool UploadImxCalibrationFromFile(std::string path);

    /**
     * @brief Upload calibration from a pre-parsed JSON object
     * @param calJson - Parsed JSON object containing calibration data
     * @return true for success, false for failure.
     */
    bool UploadImxCalibrationFromJson(const nlohmann::json& calJson);

    /**
     * @brief Fetch calibration from REST API and upload to device
     * @param restBaseUrl - Base URL of the calibration database (e.g., "http://caldb.local:8080")
     * @return HTTP status code (200 = success), -1 for connection/parse errors
     */
    int UploadIMXCalibrationFromURL(const std::string& restBaseUrl);

    /** @brief Declared but not currently defined anywhere in the SDK; SaveImxFlashConfigToFile()/SaveGpxFlashConfigToFile() are the implemented equivalents. */
    void SaveFlashConfigFile(std::string path);

    /** @return the current firmware-update status/progress for this device. */
    fwUpdate::update_status_e getUpdateStatus() { return fwUpdateState.status; };

    std::recursive_mutex        portMutex;                           //!< used to guard against concurrent use of the port in multi-threaded environments - only one read/write at a time
    port_handle_t               port = 0;                            //!< the current port (if any) through which we communicate with the physical device
    uint32_t                    nextConnectMs = 0;                   //!< time in millis when the connect connection attempt will be made (0 = don't wait)
    // libusb_device* usbDevice = nullptr; // reference to the USB device (if using a USB connection), otherwise should be nullptr.

    is_hardware_t               hdwId = IS_HARDWARE_TYPE_UNKNOWN;    //!< hardware type and version (ie, IMX-5.0)

    std::map<int, ChronoStat>   didStats;                            //!< A collection of performance statistics for ISB messages (per DID)

    dev_info_t                  devInfo = { };                       //!< Populated with IMX info if present, otherwise GPX info if present
    dev_info_t                  gpxDevInfo = { };                    //!< Only populated if a GPX device is present
    sys_params_t                sysParams = { };                     //!< IMX system parameters/status, updated as DID_SYS_PARAMS messages are received
    gpx_status_t                gpxStatus = { };                     //!< GPX status, updated as DID_GPX_STATUS messages are received
    nvm_flash_cfg_t             imxFlashCfg = { };                    //!< Locally synchronized copy of the IMX flash configuration (see ImxFlashConfig())
    gpx_flash_cfg_t             gpxFlashCfg = { };                    //!< Locally synchronized copy of the GPX flash configuration (see GpxFlashConfig())
    nvm_flash_cfg_t             imxFlashCfgUpload = { };             //!< This is the flashConfig that was most recently sent to the device
    gpx_flash_cfg_t             gpxFlashCfgUpload = { };             //!< This is the flashConfig that was most recently sent to the device
    unsigned int                imxFlashCfgUploadTimeMs = 0;         //!< (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    unsigned int                gpxFlashCfgUploadTimeMs = 0;         //!< (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    unsigned int                imxFlashSyncCheckTimeMs = 0;         //!< (ms) indicates that last time when the host confirmed synchronization of the remote and local flashCfg
    unsigned int                gpxFlashSyncCheckTimeMs = 0;         //!< (ms) indicates that last time when the host confirmed synchronization of the remote and local flashCfg
    uint32_t                    imxFlashCfgUploadChecksum = 0;        //!< checksum of the most recently uploaded IMX flash config, used to detect upload success/failure
    uint32_t                    gpxFlashCfgUploadChecksum = 0;        //!< checksum of the most recently uploaded GPX flash config, used to detect upload success/failure
    system_command_t            sysCmd = { };                        //!< most recent system_command_t sent via SetSysCmd()
    manufacturing_info_t        manfInfo = {};                       //!< manufacturing info populated by manufacturingInfo()

    logger_handle_t             devLogger = { };                     //!< logger this device is registered with (see registerWithLogger()), if any
    fwUpdate::update_status_e   closeStatus = { };                   //!< firmware-update status captured at the time this device/port was closed


    // TODO: make these private or protected
    std::mutex                  fwUpdateMutex;                       //!< used to guard against re-entrant calls into fwUpdate functions
    ISFirmwareUpdater*          fwUpdater = NULLPTR;                 //!< a pointer to the active updater
    ISFwUpdateState             fwUpdateState;                       //!< stores the state of the fwUpdater (current or previous)

    uint32_t                    lastResetRequest = 0;                //!< system time when the last reset requests was sent
    uint32_t                    resetRequestThreshold = 5000;        //!< Don't allow to send reset requests more frequently than this...
    uint32_t                    lastResetTime = 0;                   //!< used to throttle reset requests

    /**
     * Starts a firmware update for this device, constructing and configuring an ISFirmwareUpdater.
     * Subsequent fwUpdate() calls (driven by step()) advance the update.
     * @param targetDevice which sub-target (main app, bootloader, etc.) to update
     * @param cmds the firmware-update command sequence (files/actions) to perform
     * @param infoProgress callback invoked with progress/status updates
     * @param waitAction optional callback invoked while blocking/waiting during the update
     * @return IS_OP_OK if the update was started; IS_OP_IN_PROGRESS if an update is already running on this device; IS_OP_ERROR if fwUpdateMutex could not be acquired (re-entrant call)
     */
    is_operation_result updateFirmware(fwUpdate::target_t targetDevice, std::vector<std::string> cmds, fwUpdate::pfnStatusCb infoProgress, void (*waitAction)());
    /** @return true if this device is in the process of being updated, otherwise returns false. False is returned regardless of whether the update was successful or not. */
    bool fwUpdateInProgress();
    /** @return as percentage (0-1.0) the completion progress for the current fwUpdate, or 0.0 if no update is in progress. */
    float fwUpdatePercentCompleted();
    /**
     * Returns a set of messages generated during a firmware update
     * @param level the IS_LOG_LEVEL_* of messages to return from the update (defaults to IS_LOG_LEVEL_ERROR)
     * @return the filtered list of update messages
     */
    std::vector<ISFwUpdateState::message> fwUpdateMessages(eLogLevel level = IS_LOG_LEVEL_ERROR);
    /**
     * Instructs the device to continue performing its actions.  This should be called regularly to ensure that the update process
     * does not stall.
     * @param msg a pointer to an optional p_data_t containing a DID_FIRMWARE message to be processed; if nullptr (default) then no message is parsed.
     * @return true if the update is still in progress (calls inProgress()), or false if the update is finished and no further updates are needed.
     */
    bool fwUpdate(p_data_t* msg = nullptr);

    /**
     * @brief Queries device info while the device is believed to be in the ISbootloader, handshaking
     * first via handshakeISbl() if not already done.
     *
     * Public because a bootloader-mode devInfo can legitimately arrive from a discovery hint rather
     * than from the device (RelayPortFactory seeds one), and a hint carries no parsable ISbl version.
     * Callers that depend on the real bootloader version -- ISBFirmwareUpdater picks its flash offset
     * from it -- must be able to ask the bootloader directly. Unlike validate(), this does not clear
     * devInfo, so a failed probe leaves existing identity intact.
     *
     * @param timeout how long to wait for a response
     * @return true if a valid ISbootloader device-info response was received before timeout.
     */
    bool queryDeviceInfoISbl(uint32_t timeout = 3000);

    /** @return true if a and this device share the same serial number and hardware type. */
    bool operator==(const ISDevice& a) const { return (a.devInfo.serialNumber == devInfo.serialNumber) && (a.devInfo.hardwareType == devInfo.hardwareType); };

    /** @brief Blocking device-identification query: cycles NMEA/ISB/bootloader/MCUboot probes until devInfo is populated or timeout elapses. @return true if the device was successfully identified. */
    bool validate(uint32_t timeout = 1000);
    /** @brief Non-blocking counterpart to validate(): advances one step of the query cycle per call. @return an AsyncState value indicating progress/result. */
    int validateAsync(uint32_t timeout = 1000);

    /** @brief ISComm packet-received callback; currently only forwards _PTYPE_INERTIAL_SENSE_ACK packets to onIsbAckHandler(). @return 1 to allow other registered handlers to continue processing this message. */
    virtual int onPacketHandler(protocol_type_t ptype, packet_t *pkt, port_handle_t port);
    /** @brief ISComm ISB "Data" message callback; updates devInfo/sysParams/etc from the parsed DID payload. @return 1 to allow other registered handlers to continue processing this message. */
    virtual int onIsbDataHandler(p_data_t* data, port_handle_t port);
    /** @brief ISComm ISB "Ack" message callback. */
    virtual int onIsbAckHandler(p_ack_t* ack, unsigned char packetIdentifier, port_handle_t port);
    /** @brief ISComm NMEA message callback; on an INFO sentence, updates devInfo (and gpxDevInfo for GPX-reported info) and hdwId. @return 1 to allow other registered handlers to continue processing this message. */
    virtual int onNmeaHandler(const unsigned char* msg, int msgSize, port_handle_t port);

    static const int SYNC_FLASH_CFG_CHECK_PERIOD_MS =    200;     //!< (ms) interval between automatic flash-config synchronization checks
    static const int SYNC_FLASH_CFG_TIMEOUT_MS =        3000;     //!< (ms) default timeout used by WaitForImxFlashCfgSynced()/WaitForGpxFlashCfgSynced()

    /** @brief Ordered set of device-identification probe types cycled through by validate()/validateAsync(). */
    enum queryType : uint8_t {
        QUERYTYPE_NMEA = 0,         //!< probing via an NMEA device-info query
        QUERYTYPE_ISB,              //!< probing via the ISB binary protocol device-info query
        QUERYTYPE_ISbootloader,     //!< probing via the ISbootloader handshake/protocol
        QUERYTYPE_mcuBoot,          //!< probing via the MCUboot/SMP protocol
        QUERYTYPE_MAX = QUERYTYPE_mcuBoot,     //!< sentinel: highest valid queryType value
    };

private:
    bool                        was_connected = false;               //!< true, if this device's port was opened during the previous call to step()

    uint32_t                    validationStartMs = 0;               //!< If non-zero, the time in Epoch Ms at which validation was started; if zero, validation has finished (use hasDeviceInfo() to determine device status)
    uint32_t                    nextValidationMs = 0;                //!< if current_timeMs() > than this time, we'll perform the next validation query, otherwise we wait to see if the previous responds.
    queryType                   nextValidationType = QUERYTYPE_NMEA; //!< we cycle through different types of device queries looking for the first response (0 = NMEA, 1 = ISbinary, 2 = ISbootloader, 3 = MCUboot/SMP)
    unsigned int                syncCheckTimeMs = 0;

    bool                        hasHandshake = false;                //!< indicator that this device has already negotiated a handshake, and shouldn't keep trying
    std::array<std::chrono::high_resolution_clock::time_point, _PTYPE_SIZE> lastRxTs; //!< An array of timestamps of when last data was received of a particular protocol type (ISB, NMEA, RTCM3, etc)

    std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS> bcastMsgBuffers = {}; //!< pending/active BroadcastBinaryData() request slots
    is_comm_callbacks_t         originalCbs = {};                    //!< a copy of the port's original CBs before it was bound to this ISDevice; will be restored if this device is destroyed
    is_comm_callbacks_t         defaultCbs = {};                     //!< local copy of any callbacks passed at init
    std::map<int, pfnIsCommIsbDataHandler> didHandlers;              //!< DID-specific handlers
    pfnIsCommHandler            packetHandler = nullptr;              //!< previously registered "all packets" handler, restored when this device is unbound/destroyed
    pfnIsCommHandler            externalAllHandler = nullptr;        //!< A user-implemented handler for all packet/message types
    pfnIsCommIsbDataHandler     defaultISBHandler = nullptr;          //!< previously registered ISB-data handler, restored when this device is unbound/destroyed

    int                         m_calUploadState = -1;               //!< step indicator for calibration uploads (-1 = idle, 0..7 = active step)
    std::unique_ptr<ISDeviceCal> m_calibration;                       //!< calibration object owned by the device while an async upload is in flight (nullptr otherwise)
    is_operation_result         m_calUploadResult = IS_OP_NONE;      //!< result of the most recent (or currently in-flight) async calibration upload

    /** @brief Static ISComm callback trampoline; forwards to ctx's onPacketHandler() if ctx is this device and port matches. @return the forwarded result, or -1 if ctx/port don't match. */
    static int processPacket(void* ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port);
    /** @brief Static ISComm callback trampoline; forwards to ctx's onIsbDataHandler() if ctx is this device and port matches. @return the forwarded result, or -1 if ctx/port don't match. */
    static int processIsbMsgs(void* ctx, p_data_t* data, port_handle_t port);
    /** @brief Static ISComm callback trampoline; forwards to ctx's onIsbAckHandler() if ctx is this device and port matches. @return the forwarded result, or -1 if ctx/port don't match. */
    static int processIsbAck(void* ctx, p_ack_t* ack, unsigned char packetIdentifier, port_handle_t port);
    /** @brief Static ISComm callback trampoline; forwards to ctx's onNmeaHandler() if ctx is this device and port matches. @return the forwarded result, or -1 if ctx/port don't match. */
    static int processNmeaMsgs(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port);

    /** @brief Currently a no-op stub (body is commented out); intended to forward received data to an owning cISLogger. */
    void stepLogger(void* ctx, const p_data_t* data, port_handle_t port);

    /** @brief Performs the ISbootloader handshake (sends repeated handshake chars, waits for the device's response) required before queryDeviceInfoISbl() can succeed. @return true once handshaking completes (or the RX buffer could not be cleared, to avoid retrying indefinitely). */
    bool handshakeISbl();
    // NOTE: queryDeviceInfoISbl() is declared in the public section above -- ISBFirmwareUpdater needs it.

    /** @brief Periodic (SYNC_FLASH_CFG_CHECK_PERIOD_MS) IMX/GPX flash-config synchronization tick; no-op unless the device is running application firmware. */
    void SyncFlashConfig();
    /** @brief Shared IMX/GPX implementation backing SyncFlashConfig(): compares local vs. device-reported flash-config checksums and requests the full config on mismatch. @return see the .cpp for the specific negative/positive result codes. */
    int DeviceSyncFlashCfg(unsigned int timeMs, uint16_t flashCfgDid, uint16_t syncDid, unsigned int &uploadTimeMs, uint32_t &flashCfgChecksum, uint32_t &syncChecksum, uint32_t &uploadChecksum);
    /** @brief Sends only the changed regions between curData and newData for the given flash-config DID, minimizing upload traffic. @return true if any region failed to send. */
    bool UploadFlashConfigDiff(uint8_t* newData, uint8_t* curData, size_t sizeBytes, uint32_t did, uint32_t& uploadTimeMsOut, uint32_t& checksumOut);
    /** @brief Recomputes and updates flashCfg_'s checksum field after a local modification. */
    void UpdateFlashConfigChecksum(nvm_flash_cfg_t& flashCfg_);
    /** @return true if checksum is not the "invalid/unset" sentinel value (0xFFFFFFFF). */
    inline bool ValidFlashCfgCksum(uint32_t checksum) { return (checksum != 0xFFFFFFFF); }

    /**
     * To be called internally when a valid packet of ptype is received...
     * @param ptype the type (_PTYPE_*) of the received packet (defaults to _PTYPE_INERTIAL_SENSE_DATA)
     */
    void markRxTs(int ptype = _PTYPE_INERTIAL_SENSE_DATA) { lastRxTs[ptype] = std::chrono::high_resolution_clock::now(); }

    /** @brief Accrues data-received size into the per-DID ChronoStat entry in didStats. @return the accrued value added for this message. */
    double sampleIsbMsgStats(const p_data_t& data);

};

typedef std::shared_ptr<ISDevice> device_handle_t;

#endif //INERTIALSENSESDK_ISDEVICE_H
