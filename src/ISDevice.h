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
#include "protocol/FirmwareUpdate.h"

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

class ISDevice {
public:
    static ISDevice invalidRef;
    enum eHdwRunStates:uint8_t {
        HDW_STATE_UNKNOWN,
        HDW_STATE_BOOTLOADER,
        HDW_STATE_APP,
    };

    ISDevice() {
        flashCfg.checksum = (uint32_t)-1;
    }

    ISDevice(port_handle_t _port) {
        flashCfg.checksum = (uint32_t)-1;
        port = _port;
    }

    ISDevice(const ISDevice& src) {
        port = src.port;
        hdwId = src.hdwId;
        hdwRunState = src.hdwRunState;
        devInfo = src.devInfo;
        flashCfg = src.flashCfg;
        flashCfgUploadTimeMs = src.flashCfgUploadTimeMs;
        flashCfgUploadChecksum = src.flashCfgUploadChecksum;
        evbFlashCfg = src.evbFlashCfg;
        sysCmd = src.sysCmd;
        devLogger = src.devLogger;
        closeStatus = src.closeStatus;
    }

    ~ISDevice() {
        //if (port && serialPortIsOpen(port))
        //    serialPortClose(port);
        port = 0;
        hdwId = IS_HARDWARE_TYPE_UNKNOWN;
        hdwRunState = HDW_STATE_UNKNOWN;
    }

    std::string getId();
    std::string getName();
    std::string getFirmwareInfo(int detail);
    std::string getDescription();

        port_handle_t port = 0;
    // libusb_device* usbDevice = nullptr; // reference to the USB device (if using a USB connection), otherwise should be nullptr.

    uint16_t hdwId = IS_HARDWARE_TYPE_UNKNOWN;                       //! hardware type and version (ie, IMX-5.0)
    eHdwRunStates hdwRunState = HDW_STATE_UNKNOWN;                   //! state of hardware (running, bootloader, etc).

    dev_info_t devInfo = { };
    sys_params_t sysParams = { };
    nvm_flash_cfg_t flashCfg = { };
    unsigned int flashCfgUploadTimeMs = 0;		// (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    uint32_t flashCfgUploadChecksum = 0;
    evb_flash_cfg_t evbFlashCfg = { };
    system_command_t sysCmd = { };

    std::shared_ptr<cDeviceLog> devLogger;
    fwUpdate::update_status_e closeStatus = { };


    // TODO: make these private or protected
    ISFirmwareUpdater* fwUpdater = NULLPTR;
    float fwPercent = 0;
    bool fwHasError = false;
    uint16_t fwLastSlot = 0;
    fwUpdate::target_t fwLastTarget = fwUpdate::TARGET_UNKNOWN;
    fwUpdate::update_status_e fwLastStatus = fwUpdate::NOT_STARTED;
    std::string fwLastMessage;

    std::vector<std::string> target_idents;
    std::vector<std::string> target_messages;

    bool fwUpdateInProgress();
    void fwUpdate();

    bool operator==(const ISDevice& a) const { return (a.devInfo.serialNumber == devInfo.serialNumber) && (a.devInfo.hardwareType == devInfo.hardwareType); };


    bool handshakeISbl();
    bool queryDeviceInfoISbl();
};



#endif //INERTIALSENSESDK_ISDEVICE_H
