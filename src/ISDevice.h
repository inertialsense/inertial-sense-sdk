/**
 * @file ISDevice.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef INERTIALSENSESDK_ISDEVICE_H
#define INERTIALSENSESDK_ISDEVICE_H

#include <memory>

#include "DeviceLog.h"
#include "ISBootloaderBase.h"
#include "protocol/FirmwareUpdate.h"
#include "protocol_nmea.h"

extern "C"
{
    // [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
    #include "data_sets.h"
    #include "com_manager.h"
    #include "serialPortPlatform.h"
}

#define BOOTLOADER_RETRIES          10
#define BOOTLOADER_RESPONSE_DELAY   100

class ISFirmwareUpdater;
class cISLogger;

class ISDevice {
public:
    enum eHdwRunStates:uint8_t {
        HDW_STATE_UNKNOWN,
        HDW_STATE_BOOTLOADER,
        HDW_STATE_APP,
    };

    static ISDevice invalidRef;
    static std::string getIdAsString(const dev_info_t& devInfo);
    static std::string getName(const dev_info_t& devInfo);
    static std::string getFirmwareInfo(const dev_info_t& devInfo, int detail = 1, eHdwRunStates hdwRunState = eHdwRunStates::HDW_STATE_APP);

    ISDevice() {
        flashCfg.checksum = (uint32_t)-1;
    }

    ISDevice(port_handle_t _port) {
        flashCfg.checksum = (uint32_t)-1;
        port = _port;
    }

    ISDevice(const ISDevice& src) : devLogger(src.devLogger) {
        port = src.port;
        hdwId = src.hdwId;
        hdwRunState = src.hdwRunState;
        devInfo = src.devInfo;
        flashCfg = src.flashCfg;
        flashCfgUploadTimeMs = src.flashCfgUploadTimeMs;
        flashCfgUpload = src.flashCfgUpload;
        evbFlashCfg = src.evbFlashCfg;
        sysCmd = src.sysCmd;
        // devLogger = src.devLogger.get();
        closeStatus = src.closeStatus;
    }

    ~ISDevice() {
        //if (port && serialPortIsOpen(port))
        //    serialPortClose(port);
        port = 0;
        hdwId = IS_HARDWARE_TYPE_UNKNOWN;
        hdwRunState = HDW_STATE_UNKNOWN;
    }

    /**
     * @return true is this ISDevice has a valid, and open port
     */
    bool isConnected() { return (port && serialPortIsOpen(port)); }

    bool Update();
    bool step();

    std::string getIdAsString();
    std::string getName();
    std::string getFirmwareInfo(int detail = 1);
    std::string getDescription();

    void registerWithLogger(cISLogger* logger);

    // Convenience Functions
    bool BroadcastBinaryData(uint32_t dataId, int periodMultiple);
    void BroadcastBinaryDataRmcPreset(uint64_t rmcPreset, uint32_t rmcOptions) { comManagerGetDataRmc(port, rmcPreset, rmcOptions); }
    void GetData(eDataIDs dataId, uint16_t length=0, uint16_t offset=0, uint16_t period=0) { comManagerGetData(port, dataId, length, offset, period); }
    int SendData(eDataIDs dataId, const uint8_t* data, uint32_t length, uint32_t offset = 0) { return comManagerSendData(port, data, dataId, length, offset); }
    int SendRaw(const uint8_t* data, uint32_t length) { return comManagerSendRaw(port, data, length); }

    int SendNmea(const std::string& nmeaMsg);
    int QueryDeviceInfo() { return SendRaw((uint8_t*)NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE); }
    int SavePersistent() { return SendRaw((uint8_t*)NMEA_CMD_SAVE_PERSISTENT_MESSAGES_TO_FLASH, NMEA_CMD_SIZE); }
    int SoftwareReset() { return SendRaw((uint8_t*)NMEA_CMD_SOFTWARE_RESET, NMEA_CMD_SIZE); }

    int SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel);
    int SetSysCmd(const uint32_t command);
    int StopBroadcasts(bool allPorts = false) { return SendRaw((uint8_t*)(allPorts ? NMEA_CMD_STOP_ALL_BROADCASTS_ALL_PORTS : NMEA_CMD_STOP_ALL_BROADCASTS_CUR_PORT), NMEA_CMD_SIZE); }

    /**
     * @returns true is the device is indicated that a reset is required; this state SHOULD be acted on by resetting the device to ensure that it is operating as expected
     */
    bool isResetRequired() { return ((devInfo.hardwareType == IS_HARDWARE_TYPE_IMX) && (sysParams.hdwStatus & HDW_STATUS_SYSTEM_RESET_REQUIRED)) ||
                             ((devInfo.hardwareType == IS_HARDWARE_TYPE_GPX) && (gpxStatus.hdwStatus & GPX_HDW_STATUS_SYSTEM_RESET_REQUIRED)); }

    /**
     * Immediately issues s SysCmd to instruct the device to reset immediately
     * @return true if the request was successfully sent, false if the action was not able to be performed.
     */
    bool reset();

    /**
     * @returns true if reset() was called recently, and we are waiting for the device to return.
     */
    bool isResetPending() { return current_timeMs() < nextResetTime; }

    bool hasPendingFlashWrites(uint32_t& ageSinceLastPendingWrite);

    const dev_info_t& DeviceInfo() { return devInfo; }
    const sys_params_t& SysParams() { return sysParams; }

    // OH, ALL THE FLASHY-SYNCY STUFF
    bool FlashConfig(nvm_flash_cfg_t& flashCfg_);
    bool SetFlashConfig(nvm_flash_cfg_t& flashCfg_);

    /**
     * A fancy function that attempts to synchronize flashcfg between host and device - Honestly, I'm not sure its use case
     * This function DOES NOT BLOCK, it is a (not-so-)simple state check as to whether the flash is currently synced or not.
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
     * Another fancy function that blocks until a flash sync has actually occurred, returning true if successful or false if it couldn't (timeout?  validation?  bad connection?  -- who knows?)
     * @return
     */
    bool WaitForFlashSynced(uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS);

    /**
     * This is kind of like the previous one, but it actually downloads the newest FlashCfg from the device and does a byte-for-byte comparison
     * to ensure it was uploaded/downlaoded correctly.  This could fail where the previous might pass, because some parts of the flashCfg are programatically set to reflect state.
     * For example, if you send a rtkConfig = 0x08, it may return a rtkConfig of 0x00400008 because the 0x4 reflects that its persisted (or something like that).
     * @param flashCfg the config to upload (and later match against the downloaded firmware)
     * @param timeout a timeout value for how long to wait for the new flash to sync/download before failing
     * @return true if the new config was uploaded, synced, downloaded and matched with the original flashCfg, otherwise false
     */
    bool SetFlashConfigAndConfirm(nvm_flash_cfg_t& flashCfg, uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS);


    bool waitForFlashWrite();

    /**
     * A kind-of-redundant function?? I'm not sure how this is exactly different (or better?) than WaitForFlashSynced()?
     * @return
     */
    bool verifyFlashConfigUpload();

    /**
     * Failed to upload flash configuration for any reason.
     * @param port the port to get flash config for
     * @return true Flash config upload was either not received or rejected.
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


    port_handle_t port = 0;
    // libusb_device* usbDevice = nullptr; // reference to the USB device (if using a USB connection), otherwise should be nullptr.

    is_hardware_t               hdwId = IS_HARDWARE_ANY;             //! hardware type and version (ie, IMX-5.0)
    eHdwRunStates               hdwRunState = HDW_STATE_UNKNOWN;     //! state of hardware (running, bootloader, etc).

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

    std::vector<std::string> target_idents;
    std::vector<std::string> target_messages;

    uint32_t lastResetRequest = 0;              //! system time when the last reset requests was sent
    uint32_t resetRequestThreshold = 5000;      //! Don't allow to send reset requests more frequently than this...
    uint32_t nextResetTime = 0;                 //! used to throttle reset requests

    is_operation_result updateFirmware(fwUpdate::target_t targetDevice, std::vector<std::string> cmds,
            ISBootloader::pfnBootloadProgress uploadProgress, ISBootloader::pfnBootloadProgress verifyProgress, ISBootloader::pfnBootloadStatus infoProgress, void (*waitAction)()
);

    bool fwUpdateInProgress();
    bool fwUpdate();

    bool operator==(const ISDevice& a) const { return (a.devInfo.serialNumber == devInfo.serialNumber) && (a.devInfo.hardwareType == devInfo.hardwareType); };

    bool handshakeISbl();
    bool queryDeviceInfoISbl();

    static const int SYNC_FLASH_CFG_CHECK_PERIOD_MS =    200;
    static const int SYNC_FLASH_CFG_TIMEOUT_MS =        3000;
};



#endif //INERTIALSENSESDK_ISDEVICE_H
