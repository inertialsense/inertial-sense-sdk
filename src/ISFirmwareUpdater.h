//
// Created by kylemallory on 8/18/23.
//

#ifndef SDK_ISFIRMWAREUPDATER_H
#define SDK_ISFIRMWAREUPDATER_H

#include <fstream>
#include <algorithm>
#include <deque>
#include <map>

#include "util/md5.h"
#include <protocol/FirmwareUpdate.h>

#include "ISDevice.h"
#include "ISFileManager.h"
#include "ISUtilities.h"
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

#if !defined(ISDevice)
    class ISDevice;
    typedef std::shared_ptr<ISDevice> device_handle_t;
#endif



class ISFwUpdaterCmd {
public:
    enum cmd_status_e : int8_t {
        CMD_NOT_EXECUTED = -2,                              //!< command was queued, but ultimately never executed (was skipped due to jumps, etc)
        CMD_ERROR = -1,                                     //!< command failed to execute successfully
        CMD_QUEUED = 0,                                     //!< command is queued, and waiting to be executed
        CMD_IN_PROCESS = 1,                                 //!< command has start execution, but has not completed
        CMD_SUCCESS = 2,                                    //!< command had successfully completed
    };

    std::string step;                                   //!< the step label that this command is executed under
    std::string cmd;                                    //!< the name of the command
    cmd_status_e status = CMD_QUEUED;                   //!< a code indicating the state of the command: PENDING, IN_PROCESS, SUCCESS, ERROR, etc.
    std::map<std::string, std::string> args;            //!< a set of parameters (key-value pairs) to be used by the command

    std::string resultMsg;                              //!< an optional message to be reported/displayed reflecting the completion state of the command
    std::vector<std::tuple<uint8_t, std::string>> msgs; //!< a list of messages that occurred doing the execution of this cmd
    std::chrono::system_clock::time_point timeQueued;   //!< wall-clock time when this command was queued
    std::chrono::system_clock::time_point timeStarted;  //!< wall-clock time when this command started execution
    std::chrono::system_clock::time_point timeFinished; //!< wall-clock time when this command finished execution (error or success)

    explicit ISFwUpdaterCmd() { }

    ISFwUpdaterCmd(const std::string& _step, const std::string& _cmd) : step(_step), cmd(_cmd) {
        status = CMD_QUEUED;
        timeQueued = std::chrono::system_clock::now();
    }

    ISFwUpdaterCmd(const std::string& _step, const std::string& _cmd, const std::string& _args, std::deque<std::string> _keyNames = {}) : ISFwUpdaterCmd(_step, _cmd) {
        static std::map<std::string, std::vector<std::string>> defaultKeys = {
                {"target", {"target","timeout", "interval", "on-timeout"}},
                {"waitfor", {"timeout", "interval", "force", "on-timeout"}},
                {"upload", {"filename", "slot", "force", "interval"}},
                {"reset", {"type"}},
        };

        auto tmpArgs = utils::split_string(_args, ",");
        for ( std::string& arg : tmpArgs ) {
            auto kvpair = utils::split_string(arg, "=");
            if (_keyNames.empty()) {
                if (kvpair.size() == 2) {
                    args[kvpair[0]] = kvpair[1];
                } else if (defaultKeys.find(_cmd) != defaultKeys.end()) {
                    auto key = defaultKeys[_cmd][args.size()];
                    args[key] = kvpair[0];
                } else
                    args[std::to_string(args.size())] = kvpair[0];
            } else {
                if (kvpair.size() == 1) {
                    auto kn = _keyNames[0];
                    _keyNames.pop_front();
                    args[kn] = kvpair[0];
                } else if (kvpair.size() == 2) {
                    args[kvpair[0]] = kvpair[1];
                }
            }
        }
    }

    bool operator==(const ISFwUpdaterCmd& other) const {
        return (cmd == other.cmd) && (step == other.step);
    }
    inline std::string operator[](const std::string& k) {
        return args[k];
    }

    inline std::string operator[](int i) {
        for (auto& [k, v] : args) {
            if (i-- <= 0)
                return v;
        }
        return "";
    }

    inline bool hasArg(const std::string& k) {
        return (args.find(k) != args.end());
    }

    inline std::string getArg(const std::string& k, const std::string& def = "") {
        return (hasArg(k) ? args[k] : def);
    }

};

static ISFwUpdaterCmd nullCmd = ISFwUpdaterCmd();

class ISFirmwareUpdater : private fwUpdate::FirmwareUpdateHost {
public:

    struct update_msgs {
        std::string target;                                 //!< the target (if any) which was active, if any
        ISFwUpdaterCmd cmd;                                 //!< the command that generated this message
        int severity;                                       //!< the severity level of the message - use one of IS_LOG_LEVEL_*
        std::string msg;                                    //!< the fully-formatted message
    };

    port_handle_t port = nullptr;                           //!< a handle to the comm port which we use to talk to the device - if possible, we should be using the device->port
    device_handle_t device;                                 //!< a handle to the device which is being updated; maybe null in some cases
    const dev_info_t *devInfo = nullptr;                    //!< the root device info connected on this port
    dev_info_t *target_devInfo = nullptr;                   //!< the target's device info, if any

    /**
     * Constructor to initiate and manage updating a firmware image of a device connected on the specified port
     * @param portHandle handle to the port (typically serial) to which the device is connected
     * @param portName a named reference to the connected port handle (ie, COM1 or /dev/ttyACM0)
     */
    ISFirmwareUpdater(port_handle_t port, const dev_info_t *devInfo) : FirmwareUpdateHost(), port(port), devInfo(devInfo), activeCmd(&nullCmd) { }

    explicit ISFirmwareUpdater(device_handle_t device);

    void setInfoProgressCb(fwUpdate::pfnStatusCb cb) { pfnStatus_cb = cb; }

    ~ISFirmwareUpdater() override {
        cleanupFirmwarePackage();
        if (deviceUpdater) {
            delete deviceUpdater;
            deviceUpdater = nullptr;
        }
        devInfo = nullptr;
    };

    void setTarget(fwUpdate::target_t _target);

    bool setCommands(std::vector<std::string> cmds);

    // bool addCommands(std::vector<std::string> cmds);

    bool step();

    // bool isWaitingResponse() { return requestPending; }

    bool hasPendingCommands() { std::lock_guard lock(mutex); for (auto& c :commands) { if ((c.status == ISFwUpdaterCmd::CMD_QUEUED) || (c.status == ISFwUpdaterCmd::CMD_IN_PROCESS)) return true; } return false; }

    // bool hasErrors() { return !stepErrors.empty(); }
    bool hasErrors() { std::lock_guard lock(mutex); for (auto& c :commands) { if (c.status == ISFwUpdaterCmd::CMD_ERROR) return true; } return false; }

    void setLogLevel(eLogLevel level) { logLevel = level; }

    eLogLevel getLogLevel() { return logLevel; }

    std::vector<update_msgs> getStepErrors() { return stepErrors; }

    ISFwUpdaterCmd& getActiveCommand() { return *activeCmd; };

    float getProgress(int* steps = nullptr, int* total=nullptr) {
        if (steps) *steps = fwUpdate_getProgressNum();
        if (total) *total = fwUpdate_getProgressTotal();
        return fwUpdate_getProgressPercent();
    }

    fwUpdate::target_t getActiveTarget() { return fwUpdate_getSessionTarget(); }

    const char* getActiveTargetName() { return fwUpdate_getSessionTargetName(); }

    int getActiveSlot() { return fwUpdate_getSessionImageSlot(); }

    fwUpdate::update_status_e getUploadStatus() { return fwUpdate_getSessionStatus(); }

    const char* getUploadStatusName() { return fwUpdate_getNiceStatusName(getUploadStatus()); }

    bool processMessage(p_data_t* msg) { return fwUpdate_processMessage(msg->ptr, msg->hdr.size); }

    void clearAllCommands() { commands.clear(); }

    /**
     * Called when an error occurs while processing a command, to perform corrective actions (if possible).
     * Primarily, this checks if there is a failLabel defined and looks for the corresponding command label.
     * Otherwise it logs the message/errorcode, and clears the command stack.
     * @param errCode
     * @param errMsg
     */
    void handleCommandError(ISFwUpdaterCmd& cmd, int errCode, const char *errMmsg, ...);

    /**
     * Signals the updater that is should complete any pending commands, and stop processing any further commands. Optionally (if immediately == true), it will
     * no wait for pending commands to complete; this should be avoided do to possibly leaving some devices in an non-bootable state.
     * @param immediately if true (default is false), will not wait for existing actions to complete
     * @return the final update state; this should be fwUpdate::ERR_INTERRUPTED, but maybe any valid state.
     */
    fwUpdate::update_status_e cancel(bool immediately = false);

    bool isCancelable();

    bool fwUpdate_isDone();

    /**
     * this is called internally by processMessage() to do the things; it should also be called periodically to send status updated, etc.
     * This method is primarily used to drive the update process.
     * @param msg_type the type of message that was last processed, or MSG_UNKNOWN
     * @param processed true, if the message was already processed (in this case, you can do additional or optional processing in needed), or false if it was not.
     * @return true if some action was handled by the reception of this message, otherwise false.
     */
    bool fwUpdate_step(fwUpdate::msg_types_e msg_type = fwUpdate::MSG_UNKNOWN, bool processed = false) override;


private:

    enum pkg_error_e {
        PKG_SUCCESS = 0,
        PKG_ERR_PACKAGE_FILE_ERROR = -1,                    //!< the package file couldn't be opened/accessed (invalid, or not found)
        PKG_ERR_INVALID_IMAGES = -2,                        //!< the manifest doesn't define any images, or the images are incorrectly formatted
        PKG_ERR_INVALID_STEPS = -3,                         //!< the manifest doesn't define any steps, or the steps are incorrectly formatted
        PKG_ERR_INVALID_TARGET = -4,                        //!< the active step target is invalid (yaml schema/syntax)
        PKG_ERR_UNSUPPORTED_TARGET = -5,                    //!< the step target specified is valid, but not supported
        PKG_ERR_NO_ACTIONS = -6,                            //!< the step doesn't describe any actions to perform
        PKG_ERR_IMAGE_INVALID_REFERENCE = -7,               //!< the step action 'image' references an image which doesn't exist in the manifest
        PKG_ERR_IMAGE_UNKNOWN_PATH = -8,                    //!< the referenced image doesn't include a filename
        PKG_ERR_IMAGE_FILE_NOT_FOUND = -9,                  //!< the file for the referenced image doesn't exist
        PKG_ERR_IMAGE_FILE_SIZE_MISMATCH = -10,             //!< the image file's actual size doesn't match the manifest's reported size
        PKG_ERR_IMAGE_FILE_MD5_MISMATCH = -11,              //!< the image file's actual md5sum doesn't match the manifest's reported md5sum
        PKG_ERR_NO_MANIFEST = -12,                          //!< the package does not contain a manifest, or the manifest was invalid.
    };

    std::recursive_mutex mutex;                             //!< make things thread-safe??
    fwUpdate::pfnStatusCb pfnStatus_cb = nullptr;

    /** These are member variables that are indicate the state of this updater (not a specific upload, etc) **/

    enum updater_state_e : int8_t {
        UPDATER_DONE_WITH_ERRORS = -2,                      //!< no longer running, errors occurred during the update
        UPDATER_WAITING_TO_CANCEL = -1,                     //!< user requested a cancel, but operations are still pending
        UPDATER_IDLE = 0,                                   //!< the Updater is idle. Nothing started, nothing to do, etc.
        UPDATER_CMDS_QUEUED = 1,                            //!< commands are queued, but no commands are in process
        UPDATER_IN_PROGRESS,                                //!< commands are queued, and one or more are actively being ran
        UPDATER_SUCCESSFUL,                                 //!< no more queued commands, and no errors reported
        SUCCESS_WITH_NOTIFICATIONS,                         //!< no more queued commands, but there were notifications/messages reported (but not errors)
    };

    updater_state_e updateState = UPDATER_IDLE;             //!< true if this update has been cancelled (but may still be waiting for a step to complete)
    std::vector<ISFwUpdaterCmd> commands;                   //!< the stack of commands to execute for this update
    std::string activeStep;                                 //!< the name of the currently executing step name, from the manifest when available
    std::string failLabel;                                  //!< a label to jump to, when an error occurs
    ISFwUpdaterCmd* activeCmd = &nullCmd;                   //!< a reference to the currently executing command.

    eLogLevel logLevel = IS_LOG_LEVEL_INFO;                 //!< default log level to show
    std::vector<update_msgs> stepErrors;                    //!< a list of error messages messages that occurred during the update
    //bool requestPending = false;                            //!< true if a fwUpdate request has been made, and we're still waiting for a response.


    /** =========================================================== **/
    /** the rest of these are generally command specific variables  **/

    // ----  These are for uploads (using fwUpdater)

    std::istream *srcFile = nullptr;                        //!< the file that we are currently sending to a remote device, or nullptr if none
    uint32_t nextStartAttempt = 0;                          //!< the number of millis (uptime?) that we will next attempt to start an upgrade
    int8_t startAttempts = 0;                               //!< the number of attempts that have been made to request that an update be started

    int8_t maxAttempts = 5;                                 //!< the maximum number of attempts that will be made before we give up.
    uint16_t attemptInterval = 350;                         //!< the number of millis between attempts - default is to try every quarter-second, for 5 seconds

    uint16_t last_resent_chunk = 0;                         //!< the chunk id of the last/previous received req_resend  (are we getting multiple requests for the same chunk?)
    uint16_t resent_chunkid_count = 0;                      //!< the number of consecutive req_resend for the same chunk, reset if the current resend request is different than last_resent_chunk
    uint32_t resent_chunkid_time = 0;                       //!< time (ms uptime) of the first failed write for the given chunk id (also reset if the resend request's chunk is different)

    uint16_t chunkDelay = 5;                               //!< provides a throttling mechanism
    uint16_t resendChunkDelay = 250;                        //!< provides a throttling mechanism when resending chunks
    uint32_t nextChunkSend = 0;                             //!< don't send the next chunk until this time has expired.
    uint32_t updateStartTime = 0;                           //!< the system time when the firmware was started (for performance reporting)

    std::deque<uint8_t> toHost;                             //!< a "data stream" that contains the raw-byte responses from the local FirmwareUpdateDevice (to the host)

    fwUpdate::update_status_e lastStatus = fwUpdate::NOT_STARTED;
    int slotNum = 0, chunkSize = 512, progressRate = 250;
    bool forceUpdate = false;

    uint32_t pingInterval = 1000;                           //!< delay between attempts to communicate with a target device
    uint32_t pingNextRetry = 0;                             //!< time for next ping
    uint32_t pingTimeoutMs = 0;                             //!< time when the ping operation will timeout if no response before then
    uint32_t pingTimeoutExpires = 0;                        //!< time when the ping request will expire unless a response is received
    std::string timeoutLabel;                               //!< a label to jump to, when a "waitfor" times out (which is not always an error)
    uint32_t pauseUntil = 0;                                //!< delays next command execution until this time (but still allows the fwUpdate to step/receive responses).
    std::string filename;
    fwUpdate::target_t target = fwUpdate::TARGET_UNKNOWN;   //!< TODO: I believe this is the EXPECTED target ID (which is essentially the same as the session_target)

    mz_zip_archive *zip_archive = nullptr;                  //!< is NOT null IF we are updating from a firmware package (zip archive).
    fwUpdate::FirmwareUpdateDevice *deviceUpdater = nullptr;
    dev_info_t remoteDevInfo = {};
    fwUpdate::target_t remoteDevInfoTargetId = fwUpdate::TARGET_UNKNOWN;   //!< this is the target id of the responding device's version Info


    void initialize();
    ISFwUpdaterCmd& getNextQueuedCmd(ISFwUpdaterCmd* curCmd = nullptr);
    ISFwUpdaterCmd& jumpToStep(const std::string& stepLabel);
    ISFwUpdaterCmd& runCommand(ISFwUpdaterCmd& cmd);
    void cmd_ExtractPackage(ISFwUpdaterCmd& cmd);
    void cmd_SetTarget(ISFwUpdaterCmd& cmd);
    void cmd_WaitFor(ISFwUpdaterCmd& cmd);
    void cmd_Delay(ISFwUpdaterCmd& cmd);
    void cmd_UploadImage(ISFwUpdaterCmd& cmd);
    void cmd_resetDevice(ISFwUpdaterCmd& cmd);
    void cmd_finish(ISFwUpdaterCmd& cmd);
    // int cmd_setMethod(cmd_state& cmd);
    // int cmd_Reset(cmd_state& cmd);


    fwUpdate::update_status_e initializeUpload(fwUpdate::target_t _target, const std::string &filename, int slot = 0, int flags = 0, bool forceUpdate = false, int chunkSize = 2048, int progressRate = 200);

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
    // fwUpdate::update_status_e initializeDFUUpdate(libusb_device *usbDevice, fwUpdate::target_t target, uint32_t deviceId, const std::string &filename, int flags = 0, int progressRate = 500);


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
    bool fwUpdate_handleVersionResponse(const fwUpdate::payload_t &msg) override;

    bool fwUpdate_handleUpdateResponse(const fwUpdate::payload_t &msg) override;

    bool fwUpdate_handleResendChunk(const fwUpdate::payload_t &msg) override;

    bool fwUpdate_handleUpdateProgress(const fwUpdate::payload_t &msg) override;

    bool fwUpdate_handleDone(const fwUpdate::payload_t &msg) override;

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

    void fwUpdate_handleLocalDevice();

};
#endif //SDK_ISFIRMWAREUPDATER_H
