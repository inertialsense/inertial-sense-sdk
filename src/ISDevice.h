/**
 * @file ISDevice.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef INERTIALSENSESDK_ISDEVICE_H
#define INERTIALSENSESDK_ISDEVICE_H

#include <memory>
#include <functional>

#include "DeviceLog.h"
#include "ISBootloaderBase.h"
#include "protocol/FirmwareUpdate.h"
#include "protocol_nmea.h"
#include "ISFirmwareUpdater.h"

extern "C"
{
    // [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
    #include "data_sets.h"
    #include "com_manager.h"
    #include "serialPortPlatform.h"
}

#define BOOTLOADER_RETRIES          10
#define BOOTLOADER_RESPONSE_DELAY   100

#define PRINT_DEBUG 0
#if PRINT_DEBUG
#define DEBUG_PRINT(...)    printf("L%d: ", __LINE__); printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

class ISFirmwareUpdater;
class cISLogger;

class ISDevice {
public:

    static ISDevice invalidRef;
    static std::string getIdAsString(const dev_info_t& devInfo);
    static std::string getName(const dev_info_t& devInfo);
    static std::string getFirmwareInfo(const dev_info_t &devInfo, int detail = 1);

    ISDevice() {
        //std::cout << "Creating empty ISDevice: " << this << std::endl;
        flashCfg.checksum = (uint32_t)-1;
    }

    explicit ISDevice(port_handle_t _port) {
        // std::cout << "Creating ISDevice for port " << portName(_port) << " " << this << std::endl;
        flashCfg.checksum = (uint32_t)-1;
        assignPort(_port);
    }

    explicit ISDevice(port_handle_t _port, const dev_info_t& _devInfo) {
        std::cout << "Creating ISDevice for port " << portName(_port) << " " << this << std::endl;
        hdwId = ENCODE_DEV_INFO_TO_HDW_ID(_devInfo);
        devInfo = _devInfo;
        flashCfg.checksum = (uint32_t)-1;
        assignPort(_port);
    }

    explicit ISDevice(const ISDevice& src) : devLogger(src.devLogger) {
        // std::cout << "Creating ISDevice copy from " << ISDevice::getIdAsString(src.devInfo)  << " " << this << std::endl;

        hdwId = src.hdwId;
        devInfo = src.devInfo;
        flashCfg = src.flashCfg;
        flashCfgUploadTimeMs = src.flashCfgUploadTimeMs;
        flashCfgUpload = src.flashCfgUpload;
        evbFlashCfg = src.evbFlashCfg;
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

    virtual ~ISDevice() {
//        if (((hdwId != IS_HARDWARE_TYPE_UNKNOWN) && (hdwId != IS_HARDWARE_ANY)) || (devInfo.serialNumber != 0))
//            std::cout << "Destroying ISDevice " << getDescription() << ". " << this << std::endl;

        //if (port && serialPortIsOpen(port))
        //    serialPortClose(port);
        devInfo = {};
        if (port) {
            if (portType(port) & PORT_TYPE__COMM) {
                if (COMM_PORT(port)->comm.cb.context == this) {
                    COMM_PORT(port)->comm.cb = originalCbs; // return the original callbacks/contexts, but only if the context matches us
                }
            }
        }
        port = 0;
    }

    ISDevice& operator=(const ISDevice& src) {
        port = src.port;
        hdwId = src.hdwId;
        devInfo = src.devInfo;
        flashCfg = src.flashCfg;
        flashCfgUploadTimeMs = src.flashCfgUploadTimeMs;
        flashCfgUpload = src.flashCfgUpload;
        evbFlashCfg = src.evbFlashCfg;
        sysCmd = src.sysCmd;
        // devLogger = src.devLogger.get();
        closeStatus = src.closeStatus;
        return *this;
    }

    /**
     * Binds the specified port to this device. Reconfigures the port handler to call back
     * into this device instance, and reinitializes the underlying ISComm instance and
     * buffers.
     * @param port
     * @return
     */
    bool assignPort(port_handle_t port);

    /**
     * @return true is this ISDevice has a valid, and open port
     */
    bool isConnected() {
        if (port && (portType(port) & PORT_TYPE__COMM))
            return serialPortIsOpen(port);

        return false;
    }

    /**
     * @return true if the device has valid, minimal required devInfo values sufficient to indicate
     * that it genuinely identifies an Inertial Sense device.
     */
    bool hasDeviceInfo() { return (hdwId != IS_HARDWARE_TYPE_UNKNOWN) && (hdwId != IS_HARDWARE_ANY) && (devInfo.hdwRunState != HDW_STATE_UNKNOWN) && (devInfo.serialNumber != 0) && (devInfo.hardwareType != 0); }

    /**
     * Specifies an alternate handler for Inertial Sense "Data" binary protocol messages, which will be called when
     * any DID message is successfully parsed. This function will return the previously registered handler. It is the
     * callers responsibility to restore the previous handler, when this handler is no longer required.
     * @param cbHandler a function pointer or lambda function which will be called when an ISB Data packet is received
     * @return the previously registered handler, if any.
     */
    pfnIsCommIsbDataHandler registerIsbDataHandler(pfnIsCommIsbDataHandler cbHandler);

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
    bool step();
    /**
     * An alias function for step(); it literally calls step(), and returns its result.
     */
     [[deprecated("Use step() instead.")]]
    bool Update();

    /**
     * @returns the name of the currently bound port, or an empty string if none.
     */
    std::string getPortName() {  std::lock_guard<std::recursive_mutex> lock(portMutex); return (port ? portName(port) : ""); }

    /**
     * @returns a formatted string which can be used to uniquely identify the hardware associated with this device. The
     * formatted string appears as "<HdwType>-<HdwVer.Maj>.<HdrVer.Min>::SN<SerialNo>". This is sufficient to be used
     * in hashing or other comparison functions to identify a specific device.
     */
    std::string getIdAsString();

    /**
     * @returns a formatted string similar to getIdAsString(), but slightly more human-friendly.  The formatted string
     * appears as "SN<SerialNo> (<HdwType>-<HdwVer[0]>.<HdrVer[1]>[.<HdrVer[2]>.<HdrVer[3]>])"
     */
    std::string getName();

    /**
     * Returns a string representing the device firmware, as reported by its devInfo struct, with varying levels of
     * detail. The 'detail' parameter includes additional information into the resulting string:
     *      detail = 0 returns "fw#.#.#-<relType>.<relNum>"
     *      detail = 1 appends to the above " <git commit><dirtyFlag>"
     *      detail = 2 appends to the above " <buildKey>.<buildNum> <buildDate> <buildTime>"
     * @param detail an integer indicating the level of detail to include in the resulting string (default is 1)
     * @return the formatted Firmware Information string
     */
    std::string getFirmwareInfo(int detail = 1);

    /**
     * @returns a formatted string that completely describes the device as a concatenation of the following calls:
     *   getName() + getFirmwareInfo(1) + portName()
     */
    std::string getDescription();

    /**
     * Registers this device with the specified ISLogger instance, allowing the logger instance to capture and
     * log the data received from this device.  The format, rules and options for data logging are managed by
     * the ISLogger instance.
     * @param logger
     */
    void registerWithLogger(cISLogger* logger);

    /**
     * @returns true is the device is indicated that a reset is required; this state SHOULD be acted on by resetting the device to ensure that it is operating as expected
     */
    bool isResetRequired() { return ((devInfo.hardwareType == IS_HARDWARE_TYPE_IMX) && (sysParams.hdwStatus & HDW_STATUS_SYSTEM_RESET_REQUIRED)) ||
                                    ((devInfo.hardwareType == IS_HARDWARE_TYPE_GPX) && (gpxStatus.hdwStatus & GPX_HDW_STATUS_SYSTEM_RESET_REQUIRED)); }

    /**
     * Immediately issues s SysCmd to instruct the device to reset immediately. Note that there is no
     * acknowledgement or other indication that the device received the reset command, before the device
     * is reset. In order to confirm that the device was successfully reset, you could compare the upTime
     * of the device before and after the reset is issued.
     * @return true if the request was successfully sent, false if the action was not able to be performed.
     */
    bool reset();

    /**
     * @returns true if reset() was called recently, and we are waiting for the device to return.
     */
    bool isResetPending() { return current_timeMs() < nextResetTime; }

    // Convenience Functions
    bool BroadcastBinaryData(uint32_t dataId, int periodMultiple);
    void BroadcastBinaryDataRmcPreset(uint64_t rmcPreset, uint32_t rmcOptions) { std::lock_guard<std::recursive_mutex> lock(portMutex); comManagerGetDataRmc(port, rmcPreset, rmcOptions); }
    void GetData(eDataIDs dataId, uint16_t length=0, uint16_t offset=0, uint16_t period=0) { std::lock_guard<std::recursive_mutex> lock(portMutex); comManagerGetData(port, dataId, length, offset, period); }
    int SendData(eDataIDs dataId, const uint8_t* data, uint32_t length, uint32_t offset = 0) { std::lock_guard<std::recursive_mutex> lock(portMutex); return comManagerSendData(port, data, dataId, length, offset); }
    int SendRaw(const uint8_t* data, uint32_t length) {  std::lock_guard<std::recursive_mutex> lock(portMutex); return comManagerSendRaw(port, data, length); }

    int SendNmea(const std::string& nmeaMsg);
    int QueryDeviceInfo() { return SendRaw((uint8_t*)NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE); }
    int SavePersistent() { return SendRaw((uint8_t*)NMEA_CMD_SAVE_PERSISTENT_MESSAGES_TO_FLASH, NMEA_CMD_SIZE); }
    int SoftwareReset() { return SendRaw((uint8_t*)NMEA_CMD_SOFTWARE_RESET, NMEA_CMD_SIZE); }

    int SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel);
    int SetSysCmd(const uint32_t command);
    int StopBroadcasts(bool allPorts = false) { return SendRaw((uint8_t*)(allPorts ? NMEA_CMD_STOP_ALL_BROADCASTS_ALL_PORTS : NMEA_CMD_STOP_ALL_BROADCASTS_CUR_PORT), NMEA_CMD_SIZE); }

    bool hasPendingFlashWrites(uint32_t& ageSinceLastPendingWrite);

    bool lockPort() { return portMutex.try_lock(); }
    void unlockPort() { return portMutex.unlock(); }

    const dev_info_t& DeviceInfo() { return devInfo; }
    const sys_params_t& SysParams() { return sysParams; }

    // OH, ALL THE FLASHY-SYNCY STUFF

    /**
     * Populates the passed reference flashCfg with the locally synchronized copy of the remove device's config.
     * @param flashCfg_ a reference to a nvm_flash_cfg_t struct to be populated
     * @returns true if the flashCfg has been synchronized with the device (and can thus be trusted), otherwise false.
     */
    bool FlashConfig(nvm_flash_cfg_t& flashCfg_);

    /**
     * Uploads the provided flashCfg to the remove device, but makes NO checks that it was successfully synchronized.
     * This method attempt to "intelligently" upload only the portions of the flashCfg that has actually changed, reducing
     * traffic and minimizing the risk of a sync-failure due to elements which maybe programmatically changed, however it
     * may make multiple sends, if the new and previous configurations have non-contiguous modifications.
     * Use WaitForFlashSynced() or SetFlashConfigAndConfirm() to actually confirm that the new config was applied to the
     * device correctly.
     * @param flashCfg_ the new flash_config to upload
     * @return true if the ANY of the changes failed to send to the remove device.
     */
    bool SetFlashConfig(nvm_flash_cfg_t& flashCfg_);

    /**
     * A fancy function that attempts to synchronize flashCfg between host and device - Honestly, I'm not sure its use case
     * This function DOES NOT BLOCK, it is a (not-so-)simple state check as to whether the flash is currently synced or not.
     * This is actually called internally by step(), and its result is ignored; as such, if step() is being called regularly
     * the local flashCfg should be regularly synced with the remote device. Use FlashConfigSynced() to test whether the
     * local flashCfg is actually synchronized.  TODO this function should probably be made "protected"
     * @param timeMs the current time...
     * @return true if the config is synchronized, otherwise false.
     */
    bool SyncFlashConfig();

    /**
     * Indicates whether the current IMX flash config has been downloaded and available via FlashConfig().
     * @param port the port to get flash config for
     * @return true if the flash config is valid, currently synchronized, otherwise false.
     */
    bool FlashConfigSynced() {
        step();   // let's give it a very, very brief chance to processing any pendind data before we check our status...
        if (flashCfgUpload.checksum && (flashCfgUpload.checksum == sysParams.flashCfgChecksum) && (flashCfg.checksum == sysParams.flashCfgChecksum)) {
            flashCfgUpload = {};
            flashCfgUploadTimeMs = 0;
            return true;    // a 3-way match between upload, device, and sysParams
        }
        if (flashCfg.checksum == sysParams.flashCfgChecksum) {
            return true;    // a 2-way check between just the device and the sysParams
        }

        return false;
    }

    /**
     * Another fancy function that blocks until a flash sync has actually occurred.
     * @return true if successful or otherwise false if it couldn't (timeout? validation? bad connection?  -- who knows?)
     */
    bool WaitForFlashSynced(uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS);

    /**
     * A blocking call which uploads and then waits for synchronization confirmation that the new configuration was applied.
     * As part of the validation/synchronization, it downloads the newest FlashCfg from the device and performs a byte-for-byte
     * comparison* to ensure it was uploaded/downloaded correctly.  This could fail where the WaitForFlashSynced() might pass,
     * because some parts of the flashCfg are programmatically set to reflect state. For example, sending a rtkConfig = 0x08,
     * may return a rtkConfig of 0x00400008 because the 0x4 reflects that it was persisted (or something like that).
     * @param flashCfg the config to upload (and later match against the downloaded firmware)
     * @param timeout a timeout value for how long to wait for the new flash to sync/download before failing
     * @return true if the new config was uploaded, synced, downloaded and matched with the original flashCfg, otherwise false
     */
    bool SetFlashConfigAndConfirm(nvm_flash_cfg_t& flashCfg, uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS);

    /**
     * A kind-of-redundant function?? I'm not sure how this is exactly different (or better?) than WaitForFlashSynced() or SetFlashConfigAndConfirm()?
     * @return true if the local flashConfig was successfully uploaded and synchronization is confirmed, otherwise false
     */
    bool verifyFlashConfigUpload();

    /**
     * @returns true if the local flashConfig upload was either not received or rejected.
     * TODO: this REALLY only does a checksum comparision of the sysParams and the uploaded flashCfg to confirm they match.
     *  Maybe this is enough, but this function name maybe
     */
    bool FlashConfigUploadFailure() {
        // a failed flash upload is considered when flashCfgUploadChecksum is non-zero, and DOES NOT match sysParams.flashCfgChecksum
        return flashCfgUpload.checksum && (flashCfgUpload.checksum != sysParams.flashCfgChecksum);
    }

    void UpdateFlashConfigChecksum(nvm_flash_cfg_t& flashCfg_);

    /**
    * Gets current update status for selected device index
    * @param deviceIndex
    */
    fwUpdate::update_status_e getUpdateStatus() { return fwLastStatus; };

    std::recursive_mutex  portMutex;                                           //! used to guard against concurrent use of the port in multi-threaded environments - only one read/write at a time
    port_handle_t port = 0;
    // libusb_device* usbDevice = nullptr; // reference to the USB device (if using a USB connection), otherwise should be nullptr.

    is_hardware_t               hdwId = IS_HARDWARE_TYPE_UNKNOWN;    //! hardware type and version (ie, IMX-5.0)

    dev_info_t                  devInfo = { };
    sys_params_t                sysParams = { };
    gpx_status_t                gpxStatus = { };
    nvm_flash_cfg_t             flashCfg = { };
    nvm_flash_cfg_t             flashCfgUpload = { };                //!< This is the flashConfig that was most recently sent to the device
    unsigned int                flashCfgUploadTimeMs = 0;            //!< (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    unsigned int                flashSyncCheckTimeMs = 0;            //!< (ms) indicates that last time when the host confirmed synchronization of the remote and local flashCfg
    evb_flash_cfg_t             evbFlashCfg = { };
    system_command_t            sysCmd = { };
    manufacturing_info_t        manfInfo = {};


    std::shared_ptr<cDeviceLog> devLogger = { };
    fwUpdate::update_status_e closeStatus = { };


    // TODO: make these private or protected
    ISFirmwareUpdater* fwUpdater = NULLPTR;
    float fwPercent = 0;
    bool fwHasError = false;
    std::vector<std::tuple<std::string, std::string, std::string>> fwErrors;
    uint16_t fwLastSlot = 0;
    fwUpdate::target_t fwLastTarget = fwUpdate::TARGET_UNKNOWN;
    fwUpdate::update_status_e fwLastStatus = fwUpdate::NOT_STARTED;
    std::string fwLastMessage;

    uint32_t lastResetRequest = 0;              //! system time when the last reset requests was sent
    uint32_t resetRequestThreshold = 5000;      //! Don't allow to send reset requests more frequently than this...
    uint32_t nextResetTime = 0;                 //! used to throttle reset requests

    is_operation_result updateFirmware(fwUpdate::target_t targetDevice, std::vector<std::string> cmds, fwUpdate::pfnStatusCb infoProgress, void (*waitAction)());
    bool fwUpdateInProgress();
    bool fwUpdate();

    bool operator==(const ISDevice& a) const { return (a.devInfo.serialNumber == devInfo.serialNumber) && (a.devInfo.hardwareType == devInfo.hardwareType); };

    bool handshakeISbl();
    bool queryDeviceInfoISbl();
    bool validateDevice(uint32_t timeout);

    virtual int onPacketHandler(protocol_type_t ptype, packet_t *pkt, port_handle_t port);
    virtual int onIsbDataHandler(p_data_t* data, port_handle_t port);
    virtual int onNmeaHandler(const unsigned char* msg, int msgSize, port_handle_t port);

    static const int SYNC_FLASH_CFG_CHECK_PERIOD_MS =    200;
    static const int SYNC_FLASH_CFG_TIMEOUT_MS =        3000;

    enum queryTypes {
        QUERYTYPE_NMEA = 0,
        QUERYTYPE_ISB,
        QUERYTYPE_ISbootloader,
        QUERYTYPE_mcuBoot,
        QUERYTYPE_MAX = QUERYTYPE_mcuBoot,
    };

private:
    pfnIsCommHandler packetHandler = nullptr;
    pfnIsCommIsbDataHandler defaultISBHandler = nullptr;
    std::map<int, pfnIsCommIsbDataHandler> didHandlers;
    std::array<broadcast_msg_t, MAX_NUM_BCAST_MSGS> bcastMsgBuffers = {}; // [MAX_NUM_BCAST_MSGS];
    is_comm_callbacks_t originalCbs = {}; // a copy of the port's original CBs before it was bound to this ISDevice; will be restored if this device is destroyed
    is_comm_callbacks_t defaultCbs = {}; // local copy of any callbacks passed at init

    static int processPacket(void* ctx, protocol_type_t ptype, packet_t *pkt, port_handle_t port);
    static int processIsbMsgs(void* ctx, p_data_t* data, port_handle_t port);
    static int processNmeaMsgs(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port);

    void stepLogger(void* ctx, const p_data_t* data, port_handle_t port);
};



#endif //INERTIALSENSESDK_ISDEVICE_H
