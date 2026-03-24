//
// Created by kylemallory on 8/18/23.
//

#ifndef SDK_ISFIRMWAREUPDATER_H
#define SDK_ISFIRMWAREUPDATER_H

#include <fstream>
#include <algorithm>
#include <deque>
#include <map>
#include <mutex>

#include "ISConstants.h"

#include "ISDevice.h"
#include "ISFileManager.h"
#include "ISUtilities.h"
#include "PortManager.h"
#include "util/md5.h"
#include "protocol/FirmwareUpdate.h"

#include "miniz.h"

#if PLATFORM_IS_EMBEDDED == 0
    #include "yaml-cpp/yaml.h"
    #include "ISDFUFirmwareUpdater.h"
    #include "ISBootloaderBase.h"
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


/**
 * Defines the update policy for a firmware update step/target.
 */
enum update_policy_e : int8_t {
    UPDATE_POLICY_DEFAULT   = 0,   //!< inherit from updater-level default (backward compat)
    UPDATE_POLICY_SKIP      = 1,   //!< skip this target's step entirely
    UPDATE_POLICY_IF_NEWER  = 2,   //!< upload only if image version > target's current version
    UPDATE_POLICY_FORCE     = 3,   //!< always upload (current behavior)
};

/**
 * Stores the update policy and image version metadata for a single step/target.
 */
struct step_policy_t {
    update_policy_e policy = UPDATE_POLICY_DEFAULT;
    uint8_t imageVersion[4] = {};  //!< parsed from manifest "version: x.y.z.w"
    bool hasImageVersion = false;  //!< true if imageVersion was explicitly set
};


class ISFwUpdaterCmd {
public:
    enum cmd_status_e : int8_t {
        CMD_CANCELLED = -3,                                 //!< command was canceled (by the user) before it could complete
        CMD_NOT_EXECUTED = -2,                              //!< command was queued, but ultimately never executed (was skipped due to jumps, etc)
        CMD_ERROR = -1,                                     //!< command failed to execute successfully
        CMD_QUEUED = 0,                                     //!< command is queued, and waiting to be executed
        CMD_IN_PROCESS = 1,                                 //!< command has start execution, but has not completed
        CMD_SUCCESS = 2,                                    //!< command had successfully completed
        CMD_SUSPENDED = 3,                                  //!< command has been suspended (by the user) - must be moved back into IN_PROCESS to resume
    };
    enum cmd_flags_e : int16_t {
        CMD_FLAGS__PAUSABLE = 1 << 0,                       //!< command can be paused/suspended
        CMD_FLAGS__CANCELABLE = 1 << 1,                     //!< command can be canceled (by the user)
    };

    std::string step;                                           //!< the step label that this command is executed under
    std::string cmd;                                        //!< the name of the command
    cmd_status_e status = CMD_QUEUED;                       //!< a code indicating the state of the command: PENDING, IN_PROCESS, SUCCESS, ERROR, etc.
    std::map<std::string, std::string> args;                //!< a set of parameters (key-value pairs) to be used by the command
    cmd_flags_e flags;                                      //!< a bitmask of flags that apply to this command (which are common to all commands)

    std::string resultMsg;                                  //!< an optional message to be reported/displayed reflecting the active/last-known state of the command (ie, can still be used when the cmd is in progress).
    std::vector<std::tuple<uint8_t, std::string>> msgs;     //!< a list of messages that occurred doing the execution of this cmd
    std::chrono::system_clock::time_point timeQueued;       //!< wall-clock time when this command was queued
    std::chrono::system_clock::time_point timeStarted;      //!< wall-clock time when this command started execution
    std::chrono::system_clock::time_point timeFinished;     //!< wall-clock time when this command finished execution (error or success)

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
                {"policy", {"policy", "target"}},
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

class ISFwUpdateState {
public:
    enum updater_state_e : int8_t {
        UPDATER_CANCELED = -3,                              //!< no longer running, user cancelled, and not all steps were completed.
        UPDATER_DONE_WITH_ERRORS = -2,                      //!< no longer running, errors occurred during the update
        UPDATER_WAITING_TO_CANCEL = -1,                     //!< user requested a cancel, but operations are still pending
        UPDATER_IDLE = 0,                                   //!< the Updater is idle. Nothing started, nothing to do, etc.
        UPDATER_CMDS_QUEUED = 1,                            //!< commands are queued, but no commands are in process
        UPDATER_IN_PROGRESS = 2,                            //!< commands are queued, and one or more are actively being ran
        UPDATER_SUCCESSFUL = 3,                             //!< no more queued commands, and no errors reported
        SUCCESS_WITH_NOTIFICATIONS = 4,                     //!< no more queued commands, but there were notifications/messages reported (but not errors)
    };

    struct message {
        std::string target;                                 //!< the target (if any) which was active, if any
        ISFwUpdaterCmd cmd;                                 //!< the command that generated this message
        eLogLevel severity;                                 //!< the severity level of the message - use one of IS_LOG_LEVEL_*
        std::string msg;                                    //!< the fully-formatted message

        message(const std::string& _target, const ISFwUpdaterCmd& _cmd, eLogLevel _severity, const std::string& _msg) : target(_target), cmd(_cmd), severity(_severity), msg(_msg) { };
    };

    ISFwUpdateState() = default;

    // Copy constructor — copies all data fields but creates a fresh mutex (mutexes are non-copyable).
    // The source is NOT locked here; callers should use getSnapshot() for thread-safe copies.
    ISFwUpdateState(const ISFwUpdateState& other)
        : lastMessage(other.lastMessage), state(other.state), status(other.status),
          target(other.target), slot(other.slot), progress(other.progress),
          messages(other.messages), hasErrors(other.hasErrors) { }

    ISFwUpdateState& operator=(const ISFwUpdateState& other) {
        if (this != &other) {
            lastMessage = other.lastMessage;
            state = other.state;
            status = other.status;
            target = other.target;
            slot = other.slot;
            progress = other.progress;
            messages = other.messages;
            hasErrors = other.hasErrors;
        }
        return *this;
    }

    /**
     * Acquire a lock on this state object. All reads and writes to this state should be done while holding this lock,
     * to prevent cross-thread data races (e.g., GUI thread reading while IOManager thread writes).
     */
    std::unique_lock<std::recursive_mutex> lock() const { return std::unique_lock<std::recursive_mutex>(mtx); }

    /**
     * Returns a thread-safe snapshot (copy) of the current state. The caller can safely read from the copy
     * without holding a lock. Prefer this over direct field access from non-owner threads.
     */
    ISFwUpdateState getSnapshot() const {
        auto lk = lock();
        ISFwUpdateState snapshot(*this);
        return snapshot;
    }

    void resetState() {
        auto lk = lock();
        lastMessage.clear();
        messages.clear();
        target = fwUpdate::TARGET_HOST;
        status = fwUpdate::NOT_STARTED;
        slot = 0;
        progress = 0.f;
        hasErrors = false;
    }

    std::string                 lastMessage;                        //!< the current/last/most recent message which should be shown to the user
    updater_state_e             state = UPDATER_IDLE;               //!< the current/last state of the updater (overall, across all commands)
    fwUpdate::update_status_e   status = fwUpdate::NOT_STARTED;     //!< the current/last status of the updater (typically more for the current upload/command);
    fwUpdate::target_t          target = fwUpdate::TARGET_UNKNOWN;  //!< the current/last target device that is/was being updated
    uint16_t                    slot = 0;                           //!< the current/last target slot that is/was being uploaded to
    float                       progress = 0.f;                     //!< the current/last progress of the target upload
    std::vector<message>        messages;                           //!< the collection of all messages that have occurred during the update
    bool                        hasErrors = false;                  //!< an easy indicator to track errors while still in progress, generally true if msgs contains one more IS_LOG_LEVEL_ERROR messages

private:
    mutable std::recursive_mutex mtx;                               //!< protects all fields from concurrent read/write across threads
};



static ISFwUpdaterCmd nullCmd = ISFwUpdaterCmd();
static ISFwUpdateState nullState = ISFwUpdateState();

class ISFirmwareUpdater : private fwUpdate::FirmwareUpdateHost {
public:

    port_handle_t port = nullptr;                           //!< a handle to the comm port which we use to talk to the device - if possible, we should be using the device->port
    device_handle_t device;                                 //!< a handle to the device which is being updated; maybe null in some cases
    const dev_info_t *devInfo = nullptr;                    //!< the root device info connected on this port
    dev_info_t *target_devInfo = nullptr;                   //!< the target's device info, if any

    /**
     * Constructor to initiate and manage updating a firmware image of a device connected on the specified port
     * @param portHandle handle to the port (typically serial) to which the device is connected
     * @param portName a named reference to the connected port handle (ie, COM1 or /dev/ttyACM0)
     */
    ISFirmwareUpdater(port_handle_t port, const dev_info_t *devInfo, ISFwUpdateState& state) : FirmwareUpdateHost(), port(port), devInfo(devInfo), updateState(state), activeCmd(&nullCmd) { }

    explicit ISFirmwareUpdater(device_handle_t device, ISFwUpdateState& state);

    void setInfoProgressCb(fwUpdate::pfnStatusCb cb) { pfnStatus_cb = cb; }

    ~ISFirmwareUpdater() override {
        cleanupFirmwarePackage();

        if (portListenerHdl) {
            PortManager::getInstance().removePortListener(portListenerHdl);
        }

        if (deviceUpdater) {
            delete deviceUpdater;
            deviceUpdater = nullptr;
        }

        devInfo = nullptr;
    };

    void refreshUpdateState();

    void setTarget(fwUpdate::target_t _target);

    bool setCommands(std::vector<std::string> cmds);

    bool step();

    bool hasPendingCommands() { for (auto& c :commands) { if ((c.status == ISFwUpdaterCmd::CMD_QUEUED) || (c.status == ISFwUpdaterCmd::CMD_IN_PROCESS)) return true; } return false; }

    bool hasErrors() { for (auto& c :commands) { if (c.status == ISFwUpdaterCmd::CMD_ERROR) return true; } return false; }

    void setLogLevel(eLogLevel level) { logLevel = level; }

    eLogLevel getLogLevel() { return logLevel; }

    std::vector<ISFwUpdateState::message> getMessages() { return updateState.messages; }

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
     * Sets an explicit update policy for a specific step (by exact label name).
     */
    void setStepPolicy(const std::string& stepLabel, update_policy_e policy);

    /**
     * Stores a deferred pattern-based policy override. The pattern is matched (case-insensitive
     * substring) against step labels during manifest parsing, and against target names at step
     * transition time. First matching pattern wins.
     */
    void setPolicyPattern(const std::string& pattern, update_policy_e policy);

    /**
     * Sets the updater-wide default policy, used when no step-specific or pattern-matched policy applies.
     */
    void setDefaultPolicy(update_policy_e policy);

    /**
     * Resolves the effective update policy for a given step label, using the following priority:
     *   1. Exact stepPolicies entry
     *   2. defaultPolicy (if not DEFAULT)
     *   3. forceUpdate bool (legacy)
     *   4. IF_NEWER fallback
     */
    update_policy_e getEffectivePolicy(const std::string& stepLabel) const;

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
    fwUpdate::pfnStatusCb pfnStatus_cb = nullptr;           //!< callback for status updates
    PortManager::port_listener_handle_t portListenerHdl {}; //!< handle to a port listener so we can watch for devices that reboot

    /** These are member variables that are indicate the state of this updater (not a specific upload, etc) **/

    ISFwUpdateState& updateState;                           //!< a reference to an ISFwUpdateState object which will hold the state of this updater
    std::vector<ISFwUpdaterCmd> commands;                   //!< the stack of commands to execute for this update
    std::string activeStep;                                 //!< the name of the currently executing step name, from the manifest when available
    std::string failLabel;                                  //!< a label to jump to, when an error occurs
    ISFwUpdaterCmd* activeCmd = &nullCmd;                   //!< a reference to the currently executing command.
    std::string statusMsg;                                  //!< a string the reflects the current state of the updater - this should be "Human Readable" (it generally gets reported directly to the user in the UI, etc).

    eLogLevel logLevel = IS_LOG_LEVEL_INFO;                 //!< default log level to show

    /** ---- Per-target update policy state ---- **/
    std::map<std::string, step_policy_t> stepPolicies;      //!< resolved per-step policies, keyed by exact step label
    std::vector<std::pair<std::string, update_policy_e>> policyPatterns; //!< deferred patterns to match against step labels / target names
    update_policy_e defaultPolicy = UPDATE_POLICY_DEFAULT;  //!< updater-wide default policy
    std::map<fwUpdate::target_t, bool> targetUploadPerformed; //!< tracks whether any upload actually occurred for each target


    /** =========================================================== **/
    /** the rest of these are generally command specific variables  **/

    // ----  These are for uploads (using fwUpdater)

    char* srcFileBytes = nullptr;                           //!< the pointer to the allocated raw bytes that are wrapped by srcFile - if not null, this should be freed when finished with the stream below is closed.
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

    uint32_t nextPortCheck = 0;                             //!< time when the next port-check should be made, if this device has no bound port.
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
    void cmd_ExtractPackage(ISFwUpdaterCmd cmd);
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
