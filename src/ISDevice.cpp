/**
 * @file ISDevice.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "ISDevice.h"
#include "ISFirmwareUpdater.h"

#include "ISBFirmwareUpdater.h"
#include "ISDFUFirmwareUpdater.h"

#include "protocol_nmea.h"

bool ISDevice::fwUpdateInProgress() { return (fwUpdater && !fwUpdater->fwUpdate_isDone()); }

bool ISDevice::fwUpdate() {
    if (fwUpdater) {
        fwUpdater->fwUpdate_step();
        if ("upload" == fwUpdater->getActiveCommand()) {
            if (fwUpdater->fwUpdate_getSessionTarget() != fwState.lastTarget) {
                fwState.hasError = false;
                fwState.lastStatus = fwUpdate::NOT_STARTED;
                fwState.lastMessage.clear();
                fwState.lastTarget = fwUpdater->fwUpdate_getSessionTarget();
            }
            fwState.lastSlot = fwUpdater->fwUpdate_getSessionImageSlot();

            if ((fwUpdater->fwUpdate_getSessionStatus() == fwUpdate::NOT_STARTED) && fwUpdater->isWaitingResponse()) {
                // We're just starting (no error yet, but no response either)
                fwState.lastStatus = fwUpdate::INITIALIZING;
                fwState.lastMessage = ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwState.lastStatus);
            } else if ((fwUpdater->fwUpdate_getSessionStatus() != fwUpdate::NOT_STARTED) && (fwState.lastStatus != fwUpdater->fwUpdate_getSessionStatus())) {
                // We're got a valid status update (error or otherwise)
                fwState.lastStatus = fwUpdater->fwUpdate_getSessionStatus();
                fwState.lastMessage = ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwState.lastStatus);

                // check for error
                if (!fwState.hasError && fwUpdater && fwUpdater->fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED) {
                    fwState.hasError = true;
                }
            }

            // update our upload progress
            if ((fwState.lastStatus == fwUpdate::IN_PROGRESS)) {
                fwState.percent = fwUpdater->getPercentComplete(); // ((float) fwUpdater->fwUpdate_getNextChunkID() / (float) fwUpdater->fwUpdate_getTotalChunks()) * 100.f;
            } else {
                fwState.percent = fwState.lastStatus <= fwUpdate::READY ? 0.f : 100.f;
            }
        } else if ("waitfor" == fwUpdater->getActiveCommand()) {
            fwState.lastMessage = "Waiting for response from device.";
        } else if ("reset" == fwUpdater->getActiveCommand()) {
            fwState.lastMessage = "Resetting device.";
        } else if ("delay" == fwUpdater->getActiveCommand()) {
            fwState.lastMessage = "Waiting...";
        }

        if (!fwUpdater->hasPendingCommands()) {
            if (!fwState.hasError) {
                fwState.lastMessage = "Completed successfully.";
            } else {
                fwState.lastMessage = "Error: ";
                fwState.lastMessage += ISFirmwareUpdater::fwUpdate_getNiceStatusName(fwState.lastStatus);
            }
        }

        // cleanup if we're done.
        if (fwUpdater->fwUpdate_isDone()) {
            delete fwUpdater;
            fwUpdater = nullptr;
        }
    } else {
        fwState.percent = 0.0;
    }
    return fwUpdateInProgress();
}

bool ISDevice::handshakeISB() {
    static const uint8_t handshakerChar = 'U';

    // Bootloader sync requires at least 6 'U' characters to be sent every 10ms.
    // write a 'U' to handshake with the boot loader - once we get a 'U' back we are ready to go
    for (int i = 0; i < BOOTLOADER_RETRIES; i++) {
        if(serialPortWrite(&serialPort, &handshakerChar, 1) != 1) {
            return false;
        }

        if (serialPortWaitForTimeout(&serialPort, &handshakerChar, 1, BOOTLOADER_RESPONSE_DELAY)) {
            return true;           // Success
        }
    }

    return false;
}

bool ISDevice::queryDeviceInfoISB() {
    uint8_t buf[64] = {};

    handshakeISB();     // We have to handshake before we can do anything... if we've already handshaked, we won't go a response, so ignore this result

    serialPortFlush(&serialPort);
    serialPortRead(&serialPort, buf, sizeof(buf));    // empty Rx buffer

    // Query device
    serialPortWrite(&serialPort, (uint8_t*)":020000041000EA", 15);

    // Read Version, SAM-BA Available, serial number (in version 6+) and ok (.\r\n) response
    int count = serialPortReadTimeout(&serialPort, buf, 14, 1000);
    if (count >= 8 && buf[0] == 0xAA && buf[1] == 0x55)
    {   // expected response
        devInfo.firmwareVer[0] = buf[2];
        devInfo.firmwareVer[1] = buf[3];
        // m_isb_props.rom_available = buf[4];

        if(buf[11] == '.' && buf[12] == '\r' && buf[13] == '\n')
        {
            switch ((eProcessorType)buf[5]) {
                case IS_PROCESSOR_UNKNOWN:
                    devInfo.hardwareType = IS_HARDWARE_TYPE_UNKNOWN;
                    break;
                case IS_PROCESSOR_SAMx70:
                    devInfo.hardwareType = IS_HARDWARE_TYPE_EVB;
                    break;
                case IS_PROCESSOR_STM32L4:
                    // IMX-5.0
                    devInfo.hardwareType = IS_HARDWARE_TYPE_IMX;
                    devInfo.hardwareVer[0] = 5;
                    devInfo.hardwareVer[1] = 0;
                    break;
                case IS_PROCESSOR_STM32U5:
                    // GPX-1
                    devInfo.hardwareType = IS_HARDWARE_TYPE_GPX; // OR IMX-5.1
                    devInfo.hardwareVer[0] = 1;
                    devInfo.hardwareVer[1] = 0;
                    break;
                case IS_PROCESSOR_NUM:
                    break;
            }
            // m_isb_props.is_evb = buf[6];
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo) ;
            hdwRunState = ISDevice::HDW_STATE_BOOTLOADER;
            memcpy(&devInfo.serialNumber, &buf[7], sizeof(uint32_t));
            return true;
        }
    }

    hdwId = IS_HARDWARE_TYPE_UNKNOWN;
    hdwRunState = ISDevice::HDW_STATE_UNKNOWN;
    return false;
}

bool ISDevice::queryDeviceInfoDFU() {
    return false;
}

bool ISDevice::queryDeviceInfo() {
    if ((serialPort.errorCode != ENOENT) && serialPortIsOpen(&serialPort)){
        if (queryDeviceInfoISB() == true)
            return true;
    }
    return false;
}

bool ISDevice::open(int baudRate, bool validate) {

    if (serialPortIsOpen(&serialPort))
        return true;

    int ret = serialPortOpen(&serialPort, serialPort.port, baudRate, false);
    if (ret != 0)
        return false;

    if (!validate)
        return true;

    return validateDevice(5000); // 5 seconds to successfully validate
}

bool ISDevice::stopBroadcasts(bool allPorts) {
    comManagerSendRawInstance(&cmInstance, portHandle, (uint8_t*)(allPorts ? NMEA_CMD_STOP_ALL_BROADCASTS_ALL_PORTS : NMEA_CMD_STOP_ALL_BROADCASTS_CUR_PORT), NMEA_CMD_SIZE);
    return false;
}

bool ISDevice::validateDevice(uint32_t timeout) {
    unsigned int startTime = current_timeMs();
    int queryType = 0;  // we cycle through different types of queries looking for the first response (0 = NMEA, 1 = ISbinary, 2 = ISbootloader, 3 = MCUboot/SMP)

    if (!isOpen())
        return false;

    // check for Inertial-Sense App by making an NMEA request (which it should respond to)
    do {
        if (serialPort.errorCode == ENOENT)
            return false;

        switch (queryType) {
            case QUERYTYPE_NMEA:
                comManagerSendRawInstance(cmInstance, portHandle, (uint8_t *) NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE);
                break;
            case QUERYTYPE_ISB:
                comManagerSendRawInstance(cmInstance, portHandle, (uint8_t *) DID_DEV_INFO, sizeof(dev_info_t));
                break;
            case QUERYTYPE_IBbootloader:
                queryDeviceInfo();
                break;
            case QUERYTYPE_mcuBoot:
                break;

        }
        // there was some other janky issue with the requested port; even though the device technically exists, its in a bad state. Let's just drop it now.
        if (serialPort.errorCode != 0)
            return false;

        SLEEP_MS(100);
        stepComms();

        if ((current_timeMs() - startTime) > timeout)
            return false;

        queryType = static_cast<queryTypes>((int)queryType + 1 % (int)QUERYTYPE_MAX);
    } while ((hdwId != IS_HARDWARE_TYPE_UNKNOWN) && (devInfo.serialNumber != 0));

    if ((hdwId != IS_HARDWARE_TYPE_UNKNOWN) &&
        (hdwRunState != ISDevice::HDW_STATE_UNKNOWN) &&
        (devInfo.protocolVer[0] == PROTOCOL_VERSION_CHAR0)) {
        comManagerGetData(this, DID_SYS_CMD, 0, 0, 0);
        comManagerGetData(this, DID_FLASH_CONFIG, 0, 0, 0);
        comManagerGetData(this, DID_EVB_FLASH_CFG, 0, 0, 0);
    }

    return true;
}

bool ISDevice::isOpen() {
    return serialPortIsOpen(&serialPort);
}

int ISDevice::writeTo(const unsigned char* buf, int len) {
    return serialPortWrite(&serialPort, buf, len);
}

int ISDevice::readFrom(unsigned char* buf, int len) {
    int bytesRead = serialPortReadTimeout(&serialPort, buf, len, 1);

    if (devLogger) {     // Save raw data to ISlogger
        // s_is->LogRawData(&s_cm_state->devices[port], bytesRead, buf);
    }
    return bytesRead;
}

void ISDevice::stepComms() {
    // TODO: This should be moved into the connection manager...
    //   the device shouldn't have to do any stepping; it should already be done
    comManagerStepRxInstance(&cmInstance, 0);
    comManagerStepTxInstance(&cmInstance);

}

void ISDevice::processRxData(p_data_t* data)
{
    if (data->hdr.size==0 || data->ptr==NULL) {
        return;
    }

    if (devLogger) {
        // FIXME:  devLogger->SaveData(data, 0);
        // stepLogFunction(s_cm_state->inertialSenseInterface, data, port);
    }

    hdwRunState = ISDevice::HDW_STATE_APP;  // It's basically impossible for us to receive ISB protocol, and NOT be in APP state
    switch (data->hdr.id) {
        case DID_DEV_INFO:
            devInfo = *(dev_info_t*)data->ptr;
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
            break;
        case DID_SYS_CMD:
            sysCmd = *(system_command_t*)data->ptr;
            break;
        case DID_SYS_PARAMS:
            copyDataPToStructP(&sysParams, data, sizeof(sys_params_t));
            DEBUG_PRINT("Received DID_SYS_PARAMS\n");
            break;
        case DID_FLASH_CONFIG:
            copyDataPToStructP(&flashCfg, data, sizeof(nvm_flash_cfg_t));
            if ( dataOverlap(offsetof(nvm_flash_cfg_t, checksum), 4, data)) {
                sysParams.flashCfgChecksum = flashCfg.checksum;
            }
            DEBUG_PRINT("Received DID_FLASH_CONFIG\n");
            break;
        case DID_FIRMWARE_UPDATE:
            // we don't respond to messages if we don't already have an active Updater
            if (fwUpdater) {
                fwUpdater->fwUpdate_processMessage(data->ptr, data->hdr.size);
                fwUpdate();
            }
            break;

    // FIXME:  Not sure what the following code is doing... It probably should not be here, and should go away.
        case DID_GPS1_POS:
            static time_t lastTime;
            time_t currentTime = time(NULLPTR);
            if (abs(currentTime - lastTime) > 5) {	// Update every 5 seconds
                lastTime = currentTime;
                gps_pos_t &gps = *((gps_pos_t*)data->ptr);
                if ((gps.status&GPS_STATUS_FIX_MASK) >= GPS_STATUS_FIX_3D) {
                    // *s_cm_state->clientBytesToSend = nmea_gga(s_cm_state->clientBuffer, s_cm_state->clientBufferSize, gps);
                }
            }
            break;
    }

    // Now that we've done our own processing, we can call others...
    if (binaryCallbacks[data->hdr.id].callback != NULLPTR) {
        binaryCallbacks[data->hdr.id].callback(this, data);
    }

    if (binaryCallbackGlobal != NULLPTR) {
        // Called for all DID's
        binaryCallbackGlobal(this, data);
    }
}

// return 0 on success, -1 on failure
void ISDevice::processRxNmea(const uint8_t* msg, int msgSize) {
    if (m_handlerNmea) {
        m_handlerNmea(this, msg, msgSize);
    }

    switch (getNmeaMsgId(msg, msgSize))
    {
        case NMEA_MSG_ID_INFO:
            nmea_parse_info(devInfo, (const char*)msg, msgSize);
            hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
            hdwRunState = ISDevice::HDW_STATE_APP;
        break;
    }
}

void ISDevice::getData(uint16_t did, uint16_t size, uint16_t offset, uint16_t period) {
    // Create and Send request packet
    p_data_get_t get;
    get.id = did;
    get.offset = offset;
    get.size = size;
    get.period = period;

    comManagerSendInstance(cmInstance, serialPort, PKT_TYPE_GET_DATA, &get, 0, sizeof(get), 0);
}