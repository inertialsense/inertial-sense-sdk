//
// Created by kylemallory on 8/18/23.
//

#ifndef SDK_ISFIRMWAREUPDATER_H
#define SDK_ISFIRMWAREUPDATER_H

#include <fstream>
#include <algorithm>

#include "ISFirmwareUpdater.h"
#include <protocol/FirmwareUpdate.h>

extern "C"
{
// [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
#include "data_sets.h"
#include "com_manager.h"

#include "serialPortPlatform.h"
}


class ISFirmwareUpdater final : public fwUpdate::FirmwareUpdateSDK {
private:
    int pHandle = 0;                    // a handle to the comm port which we use to talk to the device
    std::ifstream* srcFile;   // the file that we are currently sending to a remote device, or nullptr if none
    uint32_t nextStartAttempt = 0;      // the number of millis (uptime?) that we will next attempt to start an upgrade
    int8_t startAttempts = 0;           // the number of attempts that have been made to request that an update be started

    int8_t maxAttempts = 5;             // the maximum number of attempts that will be made before we give up.
    uint16_t attemptInterval = 250;     // the number of millis between attempts - default is to try every quarter-second, for 5 seconds

public:
    ISFirmwareUpdater(int portHandle) : FirmwareUpdateSDK(), pHandle(portHandle) { };

    ~ISFirmwareUpdater() override {};

    bool initializeUpdate(fwUpdate::target_t _target, const std::string& filename, int slot = 0, bool forceUpdate = false, int chunkSize = 2048);

    /**
     * @param offset the offset into the image file to pull data from
     * @param len the number of bytes to pull from the image file
     * @param buffer a provided buffer to store the data into.
     * @return
     */
    int getImageChunk(uint32_t offset, uint32_t len, void **buffer) override;

    /**
     * @param msg
     * @return
     */
    bool handleUpdateResponse(const fwUpdate::payload_t& msg);

    bool handleResendChunk(const fwUpdate::payload_t& msg);

    bool handleUpdateProgress(const fwUpdate::payload_t& msg);

    /**
     * called at each step interval; if you put this behind a Scheduled Task, call this method at each interval.
     * This method is primarily used to drive the update process. Unlike the device interface, on the SDK-side, you must call Step,
     * in order to advance the update engine, and transfer image data. Failure to call Step at a regular interval could lead to the
     * device triggering a timeout and aborting the upgrade process.
     * @return the message type, if any that was most recently processed.
     */
    virtual fwUpdate::msg_types_e step() override;

    bool writeToWire(uint8_t* buffer, int buff_len) override;

};

#endif //SDK_ISFIRMWAREUPDATER_H
