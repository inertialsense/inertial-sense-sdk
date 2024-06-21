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
#include "ISFirmwareUpdater.h"

extern "C"
{
    // [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
    #include "data_sets.h"
    #include "com_manager.h"
    #include "serialPortPlatform.h"
}

class ISFirmwareUpdater;

class ISDevice {
public:
    enum eHdwRunStates:uint8_t {
        HDW_STATE_UNKNOWN,
        HDW_STATE_BOOTLOADER,
        HDW_STATE_APP,
    };

    int portHandle = 0;
    serial_port_t serialPort = { };
    // libusb_device* usbDevice = nullptr; // reference to the USB device (if using a USB connection), otherwise should be nullptr.

    uint16_t hdwId;                         //! hardware type and version (ie, IMX-5.0)
    eHdwRunStates hdwRunState;                   //! state of hardware (running, bootloader, etc).

    dev_info_t devInfo = { };
    sys_params_t sysParams = { };
    nvm_flash_cfg_t flashCfg = { };
    unsigned int flashCfgUploadTimeMs = 0;		// (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    uint32_t flashCfgUploadChecksum = 0;
    evb_flash_cfg_t evbFlashCfg = { };
    system_command_t sysCmd = { };

    std::shared_ptr<cDeviceLog> devLogger;
    fwUpdate::update_status_e closeStatus = { };

    struct {
        float percent = 0.f;
        bool hasError = false;
        uint16_t lastSlot = 0;
        fwUpdate::target_t lastTarget = fwUpdate::TARGET_UNKNOWN;
        fwUpdate::update_status_e lastStatus = fwUpdate::NOT_STARTED;
        std::string lastMessage;

        std::vector<std::string> target_idents;
        std::vector<std::string> target_messages;
    } fwState = {};
    ISFirmwareUpdater *fwUpdater = nullptr;

    bool fwUpdateInProgress();
    bool fwUpdate();

    static ISDevice invalidRef;

    ISDevice() {
        hdwId = 0;
        hdwRunState = HDW_STATE_UNKNOWN;
        portHandle = -1;
        serialPort = {};
        sysParams.flashCfgChecksum = 0xFFFFFFFF;		// Invalidate flash config checksum to trigger sync event
    };

    ISDevice(int ph, const serial_port_t & sp) {
        hdwId = 0;
        hdwRunState = HDW_STATE_UNKNOWN;
        portHandle = ph;
        serialPort = sp;
        sysParams.flashCfgChecksum = 0xFFFFFFFF;		// Invalidate flash config checksum to trigger sync event
    }

    bool queryDeviceInfo();

protected:
    bool handshakeISB();
    bool queryDeviceInfoISB();

    bool queryDeviceInfoDFU();

};



#endif //INERTIALSENSESDK_ISDEVICE_H
