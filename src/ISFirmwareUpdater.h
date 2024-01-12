//
// Created by kylemallory on 8/18/23.
//

#ifndef SDK_ISFIRMWAREUPDATER_H
#define SDK_ISFIRMWAREUPDATER_H

#include <fstream>
#include <algorithm>

#include "ISFirmwareUpdater.h"
#include <protocol/FirmwareUpdate.h>

#include "ISDFUFirmwareUpdater.h"
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

    uint16_t last_resent_chunk = 0;     //! the chunk id of the last/previous received req_resend  (are we getting multiple requests for the same chunk?)
    uint16_t resent_chunkid_count = 0;  //! the number of consecutive req_resend for the same chunk, reset if the current resend request is different than last_resent_chunk
    uint32_t resent_chunkid_time = 0;   //! time (ms uptime) of the first failed write for the given chunk id (also reset if the resend request's chunk is different)

    uint16_t chunkDelay = 15;           //! provides a throttling mechanism
    uint16_t nextChunkDelay = 250;      //! provides a throttling mechanism
    uint32_t nextChunkSend = 0;         //! don't send the next chunk until this time has expired.
    uint32_t updateStartTime = 0;       //! the system time when the firmware was started (for performance reporting)

    ISBootloader::pfnBootloadProgress pfnUploadProgress_cb = nullptr;
    ISBootloader::pfnBootloadProgress pfnVerifyProgress_cb = nullptr;
    ISBootloader::pfnBootloadStatus pfnInfoProgress_cb = nullptr;

    std::vector<std::string> commands;
    bool requestPending = false; // true is an update has been requested, but we're still waiting on a response.
    int slotNum = 0, chunkSize = 512, progressRate = 250;
    bool forceUpdate = false;
    std::string filename;
    fwUpdate::target_t target;

    dfu::ISDFUFirmwareUpdater* dfuUpdater = nullptr;

    void runCommand(std::string cmd);

public:

    /**
     * Constructor to initiate and manage updating a firmware image of a device connected on the specified port
     * @param portHandle handle to the port (typically serial) to which the device is connected
     * @param portName a named reference to the connected port handle (ie, COM1 or /dev/ttyACM0)
     */
    ISFirmwareUpdater(int portHandle, const char *portName, const dev_info_t *devInfo) : FirmwareUpdateHost(), pHandle(portHandle), portName(portName), devInfo(devInfo) { };
    ~ISFirmwareUpdater() override {};

    void setDefaultTarget(fwUpdate::target_t _target) { target = _target; }

    bool setCommands(std::vector<std::string> cmds);
    bool addCommands(std::vector<std::string> cmds);
    bool hasPendingCommands() { return !commands.empty(); }
    void clearAllCommands() { commands.clear(); }

    /**
     * Initializes a DFU-based firmware update targeting the specified USB device.
     * This method instantiates an ISDFUFirmwareUpdater, which subsequent messages from this class are sent directly to the DFUFirmwareUpdater for processing.
     * This method is necessary to pass additional information into the DFUFirmwareUpdater instance necessary for the DFU session (such as the USB device id, etc)
     * which isn't otherwise supported over the standard wire protocol.
     * @param usbDevice a libusb_device identifier which the DFU device is connected (it should already be in DFU mode)
     * @param _target the fwUpdate target device (passed for device/firmware validation and protocol compatibility)
     * @param deviceId the device's unique id/serial number from the manufacturing/dev info used to ensure we are targeting a specific device - if 0, this is ignored
     * @param filename  the filename of the firmware image to upload.  This MUST be a .hex file
     * @param progressRate the period (ms) in which progress updates are sent back to this class to report back to the UI, etc.
     * @return return true if the ISDFUFirmwareUpdater was instantiated and validations passed indicating that the device is ready to start receiving image data.
     */
    fwUpdate::update_status_e initializeDFUUpdate(libusb_device* usbDevice, fwUpdate::target_t _target, uint32_t deviceId, const std::string &filename, int progressRate);

    fwUpdate::update_status_e initializeUpdate(fwUpdate::target_t _target, const std::string& filename, int slot = 0, int flags = 0, bool forceUpdate = false, int chunkSize = 2048, int progressRate = 500);

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

    bool fwUpdate_handleDone(const fwUpdate::payload_t &msg);
    bool fwUpdate_isDone();

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

    /**
     * Firmware Package Support -- Firmware packages are zip files with a YAML-based manifest, which describes the devices, images, and related metadata necessary to update multiple devices
     * together.  This allows for a user to update a number of connected devices (IMX, GPX, and GNSS Receiver Firmware) using a single file, and without any coordination of which files are
     * necessary for which devices, etc.
     */

    int openFirmwarePackage(const std::string& pkg_file);

    int processPackageManifest(const std::string& manifest_file);

    int parsePackageManifestToCommands();

    int cleanupFirmwarePackage();

};

#endif //SDK_ISFIRMWAREUPDATER_H
