//
// Created by kylemallory on 8/18/23.
//

#ifndef SDK_ISFIRMWAREUPDATER_H
#define SDK_ISFIRMWAREUPDATER_H

#include <fstream>
#include <algorithm>

#include <protocol/FirmwareUpdate.h>

#include "ISDevice.h"
// #include "InertialSense.h"
#include "ISFileManager.h"
#include "ISUtilities.h"
#include "util/md5.h"
#include "ISDFUFirmwareUpdater.h"
#include "ISBootloaderBase.h"
#include "miniz.h"

#ifndef __EMBEDDED__
    #include "yaml-cpp/yaml.h"
#endif


extern "C"
{
    // [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
    #include "data_sets.h"
    #include "com_manager.h"
    #include "serialPortPlatform.h"
}


class ISFirmwareUpdater : public fwUpdate::FirmwareUpdateHost {
private:
    std::istream *srcFile = nullptr;    //! the file that we are currently sending to a remote device, or nullptr if none
    uint32_t nextStartAttempt = 0;      //! the number of millis (uptime?) that we will next attempt to start an upgrade
    int8_t startAttempts = 0;           //! the number of attempts that have been made to request that an update be started

    int8_t maxAttempts = 5;             //! the maximum number of attempts that will be made before we give up.
    uint16_t attemptInterval = 350;     //! the number of millis between attempts - default is to try every quarter-second, for 5 seconds

    uint16_t last_resent_chunk = 0;     //! the chunk id of the last/previous received req_resend  (are we getting multiple requests for the same chunk?)
    uint16_t resent_chunkid_count = 0;  //! the number of consecutive req_resend for the same chunk, reset if the current resend request is different than last_resent_chunk
    uint32_t resent_chunkid_time = 0;   //! time (ms uptime) of the first failed write for the given chunk id (also reset if the resend request's chunk is different)

    uint16_t chunkDelay = 25;           //! provides a throttling mechanism
    uint16_t nextChunkDelay = 250;      //! provides a throttling mechanism
    uint32_t nextChunkSend = 0;         //! don't send the next chunk until this time has expired.
    uint32_t updateStartTime = 0;       //! the system time when the firmware was started (for performance reporting)

    ISBootloader::pfnBootloadProgress pfnUploadProgress_cb = nullptr;
    ISBootloader::pfnBootloadProgress pfnVerifyProgress_cb = nullptr;
    ISBootloader::pfnBootloadStatus pfnInfoProgress_cb = nullptr;

    std::vector<std::string> commands;
    std::string activeStep;             //! the name of the currently executing step name, from the manifest when available
    std::string activeCommand;          //! the name (without parameters) of the currently executing command
    std::string failLabel;              //! a label to jump to, when an error occurs
    bool requestPending = false;        //! true is an update has been requested, but we're still waiting on a response.
    int slotNum = 0, chunkSize = 512, progressRate = 250;
    bool forceUpdate = false;
    uint32_t pingInterval = 1000;       //! delay between attempts to communicate with a target device
    uint32_t pingNextRetry = 0;         //! time for next ping
    uint32_t pingTimeout = 0;           //! time when the ping operation will timeout if no response before then
    uint32_t pauseUntil = 0;            //! delays next command execution until this time (but still allows the fwUpdate to step/receive responses).
    std::string filename;
    fwUpdate::target_t target;

    mz_zip_archive *zip_archive = nullptr; // is NOT null IF we are updating from a firmware package (zip archive).
    dfu::ISDFUFirmwareUpdater *dfuUpdater = nullptr;
    dev_info_t remoteDevInfo = {};

    std::vector<std::tuple<std::string, std::string, std::string>> stepErrors;

    void runCommand(std::string cmd);

public:

    enum pkg_error_e {
        PKG_SUCCESS = 0,
        PKG_ERR_PACKAGE_FILE_ERROR = -1,            // the package file couldn't be opened/accessed (invalid, or not found)
        PKG_ERR_INVALID_IMAGES = -2,                // the manifest doesn't define any images, or the images are incorrectly formatted
        PKG_ERR_INVALID_STEPS = -3,                 // the manifest doesn't define any steps, or the steps are incorrectly formatted
        PKG_ERR_INVALID_TARGET = -4,                // the active step target is invalid (yaml schema/syntax)
        PKG_ERR_UNSUPPORTED_TARGET = -5,            // the step target specified is valid, but not supported
        PKG_ERR_NO_ACTIONS = -6,                    // the step doesn't describe any actions to perform
        PKG_ERR_IMAGE_INVALID_REFERENCE = -7,       // the step action 'image' references an image which doesn't exist in the manifest
        PKG_ERR_IMAGE_UNKNOWN_PATH = -8,            // the referenced image doesn't include a filename
        PKG_ERR_IMAGE_FILE_NOT_FOUND = -9,          // the file for the referenced image doesn't exist
        PKG_ERR_IMAGE_FILE_SIZE_MISMATCH = -10,      // the image file's actual size doesn't match the manifest's reported size
        PKG_ERR_IMAGE_FILE_MD5_MISMATCH = -11,      // the image file's actual md5sum doesn't match the manifest's reported md5sum
    };

    // const ISDevice& device;
    int pHandle = 0;                        //! a handle to the comm port which we use to talk to the device
    const char *portName = nullptr;         //! the name of the port referenced by pHandle
    const dev_info_t *devInfo = nullptr;    //! the root device info connected on this port
    dev_info_t *target_devInfo = nullptr;   //! the target's device info, if any

    /**
     * Constructor to initiate and manage updating a firmware image of a device connected on the specified port
     * @param portHandle handle to the port (typically serial) to which the device is connected
     * @param portName a named reference to the connected port handle (ie, COM1 or /dev/ttyACM0)
     */
    ISFirmwareUpdater(int portHandle, const char *portName, const dev_info_t *devInfo) : FirmwareUpdateHost(), pHandle(portHandle), portName(portName), devInfo(devInfo) { };

    ISFirmwareUpdater(ISDevice device) : FirmwareUpdateHost(), pHandle(device.portHandle), portName(device.serialPort.port), devInfo(&device.devInfo) { };

    ~ISFirmwareUpdater() override {};

    void setTarget(fwUpdate::target_t _target);

    bool setCommands(std::vector<std::string> cmds);

    bool addCommands(std::vector<std::string> cmds);

    bool isWaitingResponse() { return requestPending; }

    bool hasPendingCommands() { return !commands.empty(); }

    bool hasErrors() { return !stepErrors.empty(); }

    std::vector<std::tuple<std::string, std::string, std::string>> getStepErrors() { return stepErrors; }

    int getPendingCommands() { return commands.size(); }
    int getPendingUploads() {
        int count = 0;
        for (auto cmd: commands) {
            if (cmd.find_first_of("upload") == 0)
                count++;
        }
        return count;
    }

    std::string getActiveCommand() { return activeCommand; };

    void clearAllCommands() { commands.clear(); }

    /**
     * Called when an error occurs while processing a command, to perform corrective actions (if possible).
     * Primarily, this checks if there is a failLabel defined and looks for the corresponding command label.
     * Otherwise it logs the message/errorcode, and clears the command stack.
     * @param errCode
     * @param errMsg
     */
    void handleCommandError(const std::string &cmd, int errCode, const char *errMmsg, ...);

    /**
     * Initializes a DFU-based firmware update targeting the specified USB device.
     * This method instantiates an ISDFUFirmwareUpdater, which subsequent messages from this class are sent directly to the DFUFirmwareUpdater for processing.
     * This method is necessary to pass additional information into the DFUFirmwareUpdater instance necessary for the DFU session (such as the USB device id, etc)
     * which isn't otherwise supported over the standard wire protocol.
     * @param usbDevice a libusb_device identifier which the DFU device is connected (it should already be in DFU mode)
     * @param _target the fwUpdate target device (passed for device/firmware validation and protocol compatibility)
     * @param deviceId the device's unique id/serial number from the manufacturing/dev info used to ensure we are targeting a specific device - if 0, this is ignored
     * @param filename  the filename of the firmware image to upload.  This MUST be a .hex file
     * @param flags additional flags used to configure options for the firmware update process
     * @param progressRate the period (ms) in which progress updates are sent back to this class to report back to the UI, etc.
     * @return return true if the ISDFUFirmwareUpdater was instantiated and validations passed indicating that the device is ready to start receiving image data.
     */
    fwUpdate::update_status_e initializeDFUUpdate(libusb_device *usbDevice, fwUpdate::target_t target, uint32_t deviceId, const std::string &filename, int flags = 0, int progressRate = 500);

    fwUpdate::update_status_e initializeUpdate(fwUpdate::target_t _target, const std::string &filename, int slot = 0, int flags = 0, bool forceUpdate = false, int chunkSize = 2048, int progressRate = 500);

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
    bool fwUpdate_handleVersionResponse(const fwUpdate::payload_t &msg);

    bool fwUpdate_handleUpdateResponse(const fwUpdate::payload_t &msg);

    bool fwUpdate_handleResendChunk(const fwUpdate::payload_t &msg);

    bool fwUpdate_handleUpdateProgress(const fwUpdate::payload_t &msg);

    bool fwUpdate_handleDone(const fwUpdate::payload_t &msg);

    bool fwUpdate_isDone();

    void setUploadProgressCb(ISBootloader::pfnBootloadProgress pfnUploadProgress) { pfnUploadProgress_cb = pfnUploadProgress; }

    void setVerifyProgressCb(ISBootloader::pfnBootloadProgress pfnVerifyProgress) { pfnVerifyProgress_cb = pfnVerifyProgress; }

    void setInfoProgressCb(ISBootloader::pfnBootloadStatus pfnInfoProgress) { pfnInfoProgress_cb = pfnInfoProgress; }

    /**
     * this is called internally by processMessage() to do the things; it should also be called periodically to send status updated, etc.
     * This method is primarily used to drive the update process.
     * @param msg_type the type of message that was last processed, or MSG_UNKNOWN
     * @param processed true, if the message was already processed (in this case, you can do additional or optional processing in needed), or false if it was not.
     * @return true if some action was handled by the reception of this message, otherwise false.
     */
    bool fwUpdate_step(fwUpdate::msg_types_e msg_type = fwUpdate::MSG_UNKNOWN, bool processed = false) override;

    /**
     * Overridden implementation that is responsible for writing the fwUpdate packets to the wire/port.
     * @param target the target that this message is directed to - note that this information is already encoded in the buffer, and maybe different. This
     *               parameter is primarily for decision making in determining which port should be selected.  Ie, an implementation might keep track of
     *               which ports packets from specific targets were received on, and the direct data back to that specific port based on the target that
     *               is being replied to. Use of this parameter is at the implementers discretion.
     * @param buffer the encoded buffer to be send
     * @param buff_len the number of bytes in the encoded buffer to send
     * @return true on success, otherwise false
     */
    bool fwUpdate_writeToWire(fwUpdate::target_t target, uint8_t *buffer, int buff_len) override;

    /**
     * Firmware Package Support -- Firmware packages are zip files with a YAML-based manifest, which describes the devices, images, and related metadata necessary to update multiple devices
     * together.  This allows for a user to update a number of connected devices (IMX, GPX, and GNSS Receiver Firmware) using a single file, and without any coordination of which files are
     * necessary for which devices, etc.
     */
    pkg_error_e openFirmwarePackage(const std::string &pkg_file);

    /**
     * Parses a YAML tree containing the manifest of the firmware package
     * @param manifest YAML::Node of the manifests parsed YAML file/text
     * @param archive a pointer to the archive which contains this manifest (or null-ptr if parsed from a file).
     * @return
     */
    pkg_error_e processPackageManifest(YAML::Node &manifest, mz_zip_archive *archive);

    pkg_error_e processPackageManifest(const std::string &manifest_file);

    /**
     * Performs any necessary cleanup of memory, file handles, or temporary files after all tasks associated with a firmware package have finished (or from an unrecoverable error).
     * @return
     */
    pkg_error_e cleanupFirmwarePackage();

    int cmd_processPackage(std::vector<std::string> &args);

    int cmd_setTarget(std::vector<std::string> &args);

    int cmd_setMethod(std::vector<std::string>& args);

    int cmd_WaitFor(std::vector<std::string> &args);

    int cmd_Upload(std::vector<std::string> &args);

    int cmd_Reset(std::vector<std::string> &args);
};
#endif //SDK_ISFIRMWAREUPDATER_H
