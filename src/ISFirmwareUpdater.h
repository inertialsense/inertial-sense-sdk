//
// Created by kylemallory on 8/18/23.
//

#ifndef SDK_ISFIRMWAREUPDATER_H
#define SDK_ISFIRMWAREUPDATER_H

#include <fstream>
#include <algorithm>

#include "ISFirmwareUpdater.h"
#include <protocol/FirmwareUpdate.h>

#include "ISBootloaderBase.h"

extern "C"
{
// [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
#include "data_sets.h"
#include "com_manager.h"

#include "serialPortPlatform.h"
}


class ISFirmwareUpdater final : public fwUpdate::FirmwareUpdateHost {
private:
    int pHandle = 0;                    //! a handle to the comm port which we use to talk to the device
    const char *portName = nullptr;     //! the name of the port referenced by pHandle
    const dev_info_t *devInfo;          //! a reference to the root device connected on this port
    std::ifstream* srcFile;             //! the file that we are currently sending to a remote device, or nullptr if none
    uint32_t nextStartAttempt = 0;      //! the number of millis (uptime?) that we will next attempt to start an upgrade
    int8_t startAttempts = 0;           //! the number of attempts that have been made to request that an update be started

    int8_t maxAttempts = 5;             //! the maximum number of attempts that will be made before we give up.
    uint16_t attemptInterval = 350;     //! the number of millis between attempts - default is to try every quarter-second, for 5 seconds

    uint16_t nextChunkDelay = 250;      //! provides a throttling mechanism
    uint32_t nextChunkSend = 0;         //! don't send the next chunk until this time has expired.
    uint32_t updateStartTime = 0;       //! the system time when the firmware was started (for performance reporting)

    ISBootloader::pfnBootloadProgress pfnUploadProgress_cb = nullptr;
    ISBootloader::pfnBootloadProgress pfnVerifyProgress_cb = nullptr;
    ISBootloader::pfnBootloadStatus pfnInfoProgress_cb = nullptr;

public:

    /**
     * Constructor to initiate and manage updating a firmware image of a device connected on the specified port
     * @param portHandle handle to the port (typically serial) to which the device is connected
     * @param portName a named reference to the connected port handle (ie, COM1 or /dev/ttyACM0)
     */
    ISFirmwareUpdater(int portHandle, const char *portName, const dev_info_t *devInfo) : FirmwareUpdateHost(), pHandle(portHandle), portName(portName), devInfo(devInfo) { };

    ~ISFirmwareUpdater() override {};

    bool initializeUpdate(fwUpdate::target_t _target, const std::string& filename, int slot = 0, bool forceUpdate = false, int chunkSize = 2048, int progressRate = 500);

    /**
     * @param offset the offset into the image file to pull data from
     * @param len the number of bytes to pull from the image file
     * @param buffer a provided buffer to store the data into.
     * @return
     */
    int fwUpdate_getImageChunk(uint32_t offset, uint32_t len, void **buffer) override;

    /**
     * @param msg
     * @return
     */
    bool fwUpdate_handleUpdateResponse(const fwUpdate::payload_t &msg);

    bool fwUpdate_handleResendChunk(const fwUpdate::payload_t &msg);

    bool fwUpdate_handleUpdateProgress(const fwUpdate::payload_t &msg);

    void setUploadProgressCb(ISBootloader::pfnBootloadProgress pfnUploadProgress){pfnUploadProgress_cb = pfnUploadProgress;}
    void setVerifyProgressCb(ISBootloader::pfnBootloadProgress pfnVerifyProgress){pfnVerifyProgress_cb = pfnVerifyProgress;}
    void setInfoProgressCb(ISBootloader::pfnBootloadStatus pfnInfoProgress) {pfnInfoProgress_cb = pfnInfoProgress;}

    /**
     * called at each step interval; if you put this behind a Scheduled Task, call this method at each interval.
     * This method is primarily used to drive the update process. Unlike the device interface, on the SDK-side, you must call Step,
     * in order to advance the update engine, and transfer image data. Failure to call Step at a regular interval could lead to the
     * device triggering a timeout and aborting the upgrade process.
     * @return the message type, if any that was most recently processed.
     */
    virtual fwUpdate::msg_types_e fwUpdate_step() override;

    bool fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t* buffer, int buff_len) override;

};

#endif //SDK_ISFIRMWAREUPDATER_H
