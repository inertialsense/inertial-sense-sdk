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

#include "DeviceLog.h"
#include "protocol/FirmwareUpdate.h"
// #include "ISFirmwareUpdater.h"

extern "C"
{
    // [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
    #include "data_sets.h"
    #include "com_manager.h"
    #include "serialPortPlatform.h"
}

class ISFirmwareUpdater;

class ISDeviceUpdater {
public:
    ISFirmwareUpdater* fwUpdater;
    float percent;
    bool hasError;
    std::vector<std::tuple<std::string, std::string, std::string>> errors;
    uint16_t lastSlot;
    fwUpdate::target_t lastTarget;
    fwUpdate::update_status_e lastStatus;
    std::string lastMessage;

    std::vector<std::string> target_idents;
    std::vector<std::string> target_messages;

    bool inProgress();
    bool update();
    // void getErrors() { return (fwUpdater != NULL ? fwUpdater->hasErrors() : hasError); }
};

class ISDevice {
public:
    int portHandle = 0;
    serial_port_t serialPort = { };
    // libusb_device* usbDevice = nullptr; // reference to the USB device (if using a USB connection), otherwise should be nullptr.

    dev_info_t devInfo = { };                   // Populated with IMX info if present, otherwise GPX info if present
    dev_info_t gpxDevInfo = { };                // Only populated if a GPX device is present
    sys_params_t sysParams = { };
    gpx_status_t gpxStatus = { };
    nvm_flash_cfg_t imxFlashCfg = { };
    gpx_flash_cfg_t gpxFlashCfg = { };
    unsigned int imxFlashCfgUploadTimeMs = 0;		// (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    unsigned int gpxFlashCfgUploadTimeMs = 0;
    uint32_t imxFlashCfgUploadChecksum = 0;
    uint32_t gpxFlashCfgUploadChecksum = 0;
    evb_flash_cfg_t evbFlashCfg = { };
    system_command_t sysCmd = { };

    std::shared_ptr<cDeviceLog> devLogger;
    fwUpdate::update_status_e closeStatus = { };
    ISDeviceUpdater fwUpdate = { };

    static ISDevice invalidRef;

    ISDevice() 
    { 
        sysParams.flashCfgChecksum = 0xFFFFFFFF;		// Set invalid checksum to trigger synchronization
        gpxStatus.flashCfgChecksum = 0xFFFFFFFF;		// Set invalid checksum to trigger synchronization
        imxFlashCfg.checksum = 0xFFFFFFFF;			    // Set invalid checksum to trigger synchronization
        gpxFlashCfg.checksum = 0xFFFFFFFF;			    // Set invalid checksum to trigger synchronization
    };

};



#endif //INERTIALSENSESDK_ISDEVICE_H
