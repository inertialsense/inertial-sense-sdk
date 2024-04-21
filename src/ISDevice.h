/**
 * @file ISDevice.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef INERTIALSENSESDK_ISDEVICE_H
#define INERTIALSENSESDK_ISDEVICE_H

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
    uint16_t lastSlot;
    fwUpdate::target_t lastTarget;
    fwUpdate::update_status_e lastStatus;
    std::string lastMessage;

    std::vector<std::string> target_idents;
    std::vector<std::string> target_messages;

    bool inProgress();
    void update();
};

class ISDevice {
public:
    int portHandle;
    serial_port_t serialPort;
    // libusb_device* usbDevice; // reference to the USB device (if using a USB connection), otherwise should be nullptr.
    dev_info_t devInfo;
    system_command_t sysCmd;
    nvm_flash_cfg_t flashCfg;
    unsigned int flashCfgUploadTimeMs;		// (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    uint32_t flashCfgUploadChecksum;
    evb_flash_cfg_t evbFlashCfg;
    sys_params_t sysParams;
    fwUpdate::update_status_e closeStatus;
    ISDeviceUpdater fwUpdate;

    static ISDevice invalidRef;

    ISDevice() { };


};



#endif //INERTIALSENSESDK_ISDEVICE_H
