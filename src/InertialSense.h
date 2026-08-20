/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file InertialSense.h
 * @brief Legacy all-devices façade over PortManager/DeviceManager/ISDevice: opens port(s), discovers
 * devices, and broadcasts commands/data-requests to every connected device at once. Most of the
 * per-device functionality here has since moved onto ISDevice directly (see the individual method
 * docs) -- new code that only needs to address a single device should prefer ISDevice.
 */

#ifndef __INERTIALSENSE_H
#define __INERTIALSENSE_H

#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <cstring>
#include <string>
#include <functional>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <vector>

#include "core/msg_logger.h"
#include "message_stats.h"
#include "ISConstants.h"
#include "ISLogger.h"
#include "ISDisplay.h"
#include "ISUtilities.h"
#include "ISDataMappings.h"
#include "ISStream.h"
#include "ISDevice.h"
#include "message_stats.h"
#include "ISFirmwareUpdater.h"

#include "PortManager.h"
#include "DeviceManager.h"

extern "C"
{
// [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
#include "data_sets.h"
#include "com_manager.h"

#include "serialPortPlatform.h"
}

class InertialSense;

/** Custom-allocator callback signature: given a port and its discovered dev_info_t, return a newly allocated device (or subclass). */
typedef device_handle_t(*pfnOnNewDeviceHandler)(port_handle_t port, const dev_info_t& devInfo);
/** Custom-clone callback signature: return a newly allocated copy of orig. */
typedef device_handle_t(*pfnOnCloneDeviceHandler)(const ISDevice& orig);
/** Callback signature for per-step data logging hooks. */
typedef void(*pfnStepLogFunction)(void* ctx, const p_data_t* data, port_handle_t port);
/** Callback signature for per-DID or global binary-data received callbacks. */
typedef std::function<void(void* ctx, p_data_t* data, port_handle_t port)> pfnHandleBinaryData;
/** Callback signature for ISB ack/set-data-response received callbacks. */
typedef std::function<void(void* ctx, p_ack_t* ack, unsigned char packetIdentifier, port_handle_t port)> pfnHandleAckData;

/**
 * @brief Inertial Sense C++ interface.
 * Note only one instance of this class per process is supported.
 */
class InertialSense
{
public:
    PortManager& portManager = PortManager::getInstance();     //!< reference to the PortManager singleton
    DeviceManager& deviceManager = DeviceManager::getInstance(); //!< reference to the DeviceManager singleton

    /** @brief Legacy ComManager-era global callback/buffer state, retained for the deprecated single-callback broadcast path and testing (see ComManagerState()). */
    struct com_manager_cpp_state_t
    {
        // common vars
        pfnHandleBinaryData binaryCallbackGlobal;      //!< callback invoked for any DID without a more specific entry in binaryCallback
        pfnHandleAckData binaryAckCallback;    //!< acknowledgment command and set data callback
#define SIZE_BINARY_CALLBACK    256
        pfnHandleBinaryData binaryCallback[SIZE_BINARY_CALLBACK] = {};     //!< per-DID binary-data callbacks, indexed by DID (for DIDs < SIZE_BINARY_CALLBACK)
        pfnStepLogFunction stepLogFunction = nullptr;                      //!< optional per-step data logging hook
        InertialSense* inertialSenseInterface = nullptr;                  //!< back-reference to the owning InertialSense instance
        char* clientBuffer = nullptr;                                      //!< unused legacy client buffer pointer
        int clientBufferSize = 0;                                          //!< unused legacy client buffer size
        int* clientBytesToSend = 0;                                        //!< unused legacy client bytes-to-send pointer
        int16_t discoveryTimeout = DeviceManager::DISCOVERY__DEFAULT_TIMEOUT;  //!< per-device discovery timeout (ms) used by this instance
    };


    /**
    * Constructor
    */
    InertialSense();

    /** @brief Constructs with a custom set of port/device factories, in place of the default set registered by the no-arg constructor. */
    InertialSense(std::vector<PortFactory*> pFactories, std::vector<DeviceFactory*> dFactories);

    /**
    * Constructor
    * @param callbackIsb InertialSense binary received data callback (optional). If specified, ALL BroadcastBinaryData requests will callback to this function.
    * @param callbackHandlerAck acknowledgment/set-data-response received callback (optional).
    * @param callbackRmc Real-time message controller received data callback (optional).
    * @param callbackNmea NMEA received received data callback (optional).
    * @param callbackUblox Ublox binary received data callback (optional).
    * @param callbackRtcm3 RTCM3 received data callback (optional).
    * @param callbackSpartn Spartn received data callback (optional).
    * @param callbackSeptSbf Septentrio SBF received data callback (optional).
    * @param callbackSeptReply Septentrio reply received data callback (optional).
    * @param callbackNewDevice custom new-device allocator callback (optional); see registerNewDeviceHandler().
    */
    explicit InertialSense(
            pfnHandleBinaryData     callbackIsb,
            pfnHandleAckData        callbackHandlerAck = NULL,
            pfnComManagerRmcHandler callbackRmc = NULL,
            pfnIsCommGenMsgHandler  callbackNmea = NULL,
            pfnIsCommGenMsgHandler  callbackUblox = NULL,
            pfnIsCommGenMsgHandler  callbackRtcm3 = NULL,
            pfnIsCommGenMsgHandler  callbackSpartn = NULL,
            pfnIsCommGenMsgHandler  callbackSeptSbf = NULL,
            pfnIsCommGenMsgHandler  callbackSeptReply = NULL,
            pfnOnNewDeviceHandler   callbackNewDevice = NULL);

    /**
    * Destructor
    */
    virtual ~InertialSense();

    /** @return the most recently constructed InertialSense instance, or nullptr if none exists. Only one instance per process is supported. */
    static InertialSense* getLastInstance();

    /**
    * Closes any open connection and then opens the device
    * @param port the port to open
    * @param baudRate the baud rate to connect with - supported rates are 115200, 230400, 460800, 921600, 2000000, 3000000
    * @param disableBroadcastsOnClose whether to send a stop broadcasts command to all units on Close
    * @param filterHdwType a IS_HARDWARE_* type used to restrict discovery to only matching device types
    * @return true if opened, false if failure (i.e. baud rate is bad or port fails to open)
    */
    bool Open(const char* port, int baudRate=IS_BAUDRATE_DEFAULT, bool disableBroadcastsOnClose=false, uint16_t filterHdwType=IS_HARDWARE_ANY);

    /**
    * Check if the connection is open
    * @return true if at least one managed device is currently connected
    */
    bool IsOpen();

    /**
    * Close the device connection, stop logger if running, and free resources.
    */
    void Close();

    /**
    * Get all open serial port names
    * @return currently always returns an empty vector; not yet implemented against DeviceManager/PortManager.
    */
    std::vector<std::string> GetPortNames() { return {}; }

    /**
     * @return a vector of available ports
     * NOTE that this may return ports which do not have a corresponding ISDevice
     */
    std::set<port_handle_t> getPorts() { return portManager; }

    /** @return the number of devices currently managed by DeviceManager. */
    int DeviceCount() { return (int)deviceManager.DeviceCount(); }

    /** @return a reference to DeviceManager's backing list of managed devices. */
    std::list<device_handle_t>& getDevices() { return deviceManager; };

    /** @return the device bound to port, or nullptr if none is known. */
    device_handle_t getDevice(port_handle_t port) { return deviceManager.getDevice(port); }

    /** @return the device with the given unique Id (see ISDevice::getUniqueId()), or nullptr if none is known. */
    device_handle_t getDevice(uint64_t uid) { return deviceManager.getDevice(uid); }

    /**
    * Call in a loop to send and receive data.  Call at regular intervals as frequently as want to receive data.
    * @return true if updating should continue, false if the process should be shutdown
    */
    bool Update();

    /**
     * Register a callback handler for data stream errors.
     * @param errorHandler function to be called when a data-stream parse error occurs
     */
    void setErrorHandler(pfnComManagerParseErrorHandler errorHandler) { m_handlerError = errorHandler; }

    /**
    * Enable or disable logging - logging is disabled by default
    * @param logEnable enable or disable the logger - disabling the logger after enabling it will close it and flush all data to disk
    * @param logPath the path to write the log files to
    * @param logOptions the cISLogger save options (log type, filters, etc.) to use
    * @param rmcPreset RMC preset for data streaming
    * @param rmcOptions RMC options for data streaming
    * @return true if success, false if failure
    */
    bool EnableLogger(
        bool logEnable = true,
        const std::string& logPath = cISLogger::g_emptyString,
        const cISLogger::sSaveOptions &logOptions = cISLogger::sSaveOptions(),
        uint64_t rmcPreset = RMC_PRESET_IMX_PPD,
        uint32_t rmcOptions = RMC_OPTIONS_PRESERVE_CTRL);

    /**
    * (deprecated) Not recommended for future development.
    * Enable or disable logging - logging is disabled by default
    * @param logEnable enable or disable the logger - disabling the logger after enabling it will close it and flush all data to disk
    * @param logPath the path to write the log files to
    * @param logType the type of log to write
    * @param rmcPreset RMC preset for data streaming
    * @param rmcOptions RMC options for data streaming
    * @param driveUsageLimitPercent the maximum usable disk space in percent of total drive size (0.0 to 1.0). Oldest files are deleted to maintain this limit. Zero to disable this limit.
    * @param maxFileSize the max file size for each log file in bytes
    * @param subFolder timestamp sub folder or empty for none
    * @return true if success, false if failure
    */
    [[deprecated("Not recommended for future development. Use EnableLogger() instead.")]]
    bool SetLoggerEnabled(
        bool logEnable,
        const std::string& logPath = cISLogger::g_emptyString,
        cISLogger::eLogType logType = cISLogger::eLogType::LOGTYPE_DAT,
        uint64_t rmcPreset = RMC_PRESET_IMX_PPD,
        uint32_t rmcOptions = RMC_OPTIONS_PRESERVE_CTRL,
        float driveUsageLimitPercent = 0.5f,
        uint32_t maxFileSize = 1024 * 1024 * 5,
        const std::string& subFolder = cISLogger::g_emptyString);

    /**
    * Gets whether logging is enabled
    * @return whether logging is enabled
    */
    bool LoggerEnabled() { return m_logger.Enabled(); }

    /**
     * @brief Get pointer to ISLogger
     * 
     * @return cISLogger* ISLogger pointer
     */
    cISLogger* Logger() { return &m_logger; }

    /**
     * @brief Log raw data directly to ISLogger
     * 
     * @param device device associated with this raw data.
     * @param dataSize Number of bytes of raw data.
     * @param data Pointer to raw data.
     */
    void LogRawData(device_handle_t device, int dataSize, const uint8_t* data);

    /**
     * Locates the device associated with the specified port
     * @param port the port to look up; defaults to the first/only device's port if 0
     * @return device_handle_t which is connected to port, otherwise NULL
     */
    device_handle_t DeviceByPort(port_handle_t port = 0);

    /**
     * Locates the device associated with the specified port name
     * @param port_name the port name to look up (see portName())
     * @return device_handle_t which is connected to port, otherwise NULL
     * @note declared but not currently defined anywhere in the SDK.
     */
    device_handle_t DeviceByPortName(const std::string& port_name);

    /**
     * @param oldPorts the previously known set of port names, used as a baseline to detect new ones
     * @return a list of discovered ports which are not currently associated with a open device
     */
    std::vector<std::string> checkForNewPorts(std::vector<std::string>& oldPorts);

    /**
     * @brief Process received data from a port
     *
     * @param port the port the data was received on
     * @param data the parsed ISB data message
     */
    void ProcessRxData(port_handle_t port, p_data_t* data);
    /** @brief Process a received NMEA sentence from a port. @param port the port the message was received on @param msg the raw NMEA sentence bytes @param msgSize length of msg in bytes */
    void ProcessRxNmea(port_handle_t port, const uint8_t* msg, int msgSize);

    /**
    * Flush all data from receive port
    */
    void FlushRx()
    {
        for (auto device : deviceManager)
        {
            if (device->isConnected())
                portFlush(device->port);
        }
    }

    /**
    * Get the timeout flush logger parameter in seconds
    * @return the timeout flush logger parameter in seconds
    */
    time_t TimeoutFlushLoggerSeconds() { return m_logger.TimeoutFlushSeconds(); }

    /**
    * Set the timeout flush logger parameter in seconds
    * @param timeoutFlushLoggerSeconds the timeout flush logger parameter in seconds
    */
    void SetTimeoutFlushLoggerSeconds(time_t timeoutFlushLoggerSeconds) { m_logger.SetTimeoutFlushSeconds(timeoutFlushLoggerSeconds); }

    /**
    * Enable the device validate used to verify device response when Open() is called.
    * @param enable device validation
    */
    void EnableDeviceValidation(bool enable) { m_enableDeviceValidation = enable; }

#if !PLATFORM_IS_EMBEDDED
#endif

    /**
     * V2 firmware update mechanism. Calling this function will attempt to initiate a firmware update with the targeted device(s), with callbacks to provide information about the status
     * of the update process.
     * @param targetDevice the device which all commands should be directed to
     * @param cmds a vector of strings to be interpreted as commands, performed in sequence.  ie ["slot=0","upload=myfirmware.bin","slot=1","upload=configuration.conf","softReset"]
     * @param fwUpdateStatus a callback method which provides progress information about the update
     * @param waitAction a callback which is checked periodically to see if the update should be cancelled
     * @return always IS_OP_OK; this fires ISDevice::updateFirmware() on every managed device without aggregating their individual results (poll isFirmwareUpdateFinished()/isFirmwareUpdateSuccessful() for outcome)
     */
    is_operation_result updateFirmware(fwUpdate::target_t targetDevice, std::vector<std::string> cmds, fwUpdate::pfnStatusCb fwUpdateStatus, void (*waitAction)() = nullptr);

    /**
     * @return true if all devices have finished all firmware update steps
     */
    bool isFirmwareUpdateFinished();

    /**
     * @return true if all devices has completed with no reported errors
     */
    bool isFirmwareUpdateSuccessful();

    /**
     * @return returns a percentage (0-100) indicating the average percent complete of all devices performing a firmware update
     * Note: Percent Complete is only reported when uploading a file. Since this is an average, as devices progress through
     * different files (IMX, GPX, GNSS, etc) the percent will start over (as their individual progress will restart for each new file).
     */
    int getFirmwareUpdatePercent();

    /**
     * Step through all known devices; performing validation, data processing, and firmware upgrades.
     * This should be called at periodic intervals in order to allow all devices to process.
     * TODO: Ideally this should not be called, and each device will have its own thread to process
     *   its own data, etc.  But, that doesn't exists yet.
     */
    void step() { for (auto device : deviceManager) { device->step(); } }

    /**
    * Request device(s) version information (dev_info_t) for all connected devices. This does not wait for, or
    * validate the response.
    */
    static void QueryDeviceInfo();

    /**
    * Turn off broadcasting of all messages on all connected devices.
     * @param allPorts if true (default), will instruct each device to stop all message broadcasts on all device
     *   ports (Ser0, Ser1, Ser2, etc). Otherwise (false), only the device port which is directly connected to
     *   this host will stop broadcasting; the device will continue to broadcast on its other ports.
    */
    static void StopBroadcasts(bool allPorts=true);

    /**
     * Persists the currently streaming/broadcasting messages to flash memory, and enables broastcasts on boot.
     * This will cause the device to automatically resume streaming of its current message sets on all configured
     * ports each time the device reboots.
     */
    static void SavePersistent();

    /**
     * Instructs all connected devices to perform a software reset.
     */
    static void SoftwareReset();

    /**
     * @brief Request a specific data set by DID.
     *
     * @param dataId Data set ID
     * @param length Byte length of data requested.  Zero means entire data set.
     * @param offset Byte offset into data
     * @param period Broadcast period multiple
     */
    static void GetData(eDataIDs dataId, uint16_t length=0, uint16_t offset=0, uint16_t period=0);

    /**
    * Send packet payload data to all devices; the payload data is wrapped according to the pktInfo parameter
    * and the appropriate checksum is calculated and appended.  This function can be used to send non-standard packets
    * and data sets, such as RTCM, UBLOX, etc.
    * @param pktInfo a field indication the type of, and flags for, the packet to be sent
    * @param data the data to send
    * @param did the data id of the data to send
    * @param size length of data to send
    * @param offset offset into data to send at
     */
    static void Send(uint8_t pktInfo, void *data=NULL, uint16_t did=0, uint16_t size=0, uint16_t offset=0);

    /**
     * Send IS packet payload data to all devices; the payload data is wrapped in an ISB packet with the specified dataId
     * and the appropriate checksum is calculated and appended.  This function can be used to send a subset of a data set.
     * For example, to set only a portion of DID_FLASH_CONFIG, you could use SendData like this:
     *   SendData(DID_FLASH_CONFIG, &cfg.refLla[0], sizeof(double)*3, offsetof(nvm_flash_cfg_t, refLla));
     * @param dataId the data id of the data to send
     * @param data the data to send
     * @param length length of data to send
     * @param offset offset into data to send at
     */
    static void SendData(eDataIDs dataId, void* data, uint32_t length, uint32_t offset = 0);

    /**
    * Send raw (bare) data directly to serial port
    * @param data the data to send
    * @param length length of data to send
    */
    static void SendRaw(void* data, uint32_t length);

    /**
     * Send the specified string as a NMEA sentence.  This function will insert the prefix and calculate the checksum if they
     * are not already provided.
     * @param nmeaMsg the sentence to send
     */
    static void SendNmea(const std::string& nmeaMsg);

    /**
     * Requests every currently managed device broadcast (or fetch once, if periodMultiple is 0) the given DID.
     * @param dataId the data id (DID_* - see data_sets.h) to broadcast
     * @param periodMultiple a scalar that the source period is multiplied by to give the output period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
     */
    static void BroadcastBinaryData(uint32_t dataId, int periodMultiple);

    /*
    * Broadcast binary data
    * @param dataId the data id (DID_* - see data_sets.h) to broadcast
    * @param periodMultiple a scalar that the source period is multiplied by to give the output period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
    * @param callback optional callback for this dataId
    * @return true if success, false if error - if callback is NULL and no global callback was passed to the constructor, this will return false
    */
    // bool BroadcastBinaryData(uint32_t dataId, int periodMultiple, pfnHandleBinaryData callback = NULL);

    /**
    * Enable streaming of predefined set of messages.  The default preset, RMC_PRESET_INS, stream data necessary for post processing.
    * @param rmcPreset realtimeMessageController preset
    * @param rmcOptions realtimeMessageController options bitmask
    */
    static void BroadcastBinaryDataRmcPreset(uint64_t rmcPreset=RMC_PRESET_INS, uint32_t rmcOptions=0);

    /**
    * Get current device system command
    * @param port the port to get sysCmd for
    * @return current device system command
    */
    system_command_t GetSysCmd(port_handle_t port = 0);

    /**
    * Set device configuration
    * @param port the port to set sysCmd for
    * @param command system command value (see eSystemCommand)
    */
    void SetSysCmd(const uint32_t command, port_handle_t port = 0);

    /**
     * Sends message to device to set devices Event Filter
     * @param target 0 = device, 1 = forward to device GNSS 1 port (ie GPX), 2 = forward to device GNSS 2 port (ie GPX); any other value is rejected without sending
     * @param msgTypeIdMask bitmask of message type Ids to filter on
     * @param portMask bitmask of device ports the filter applies to
     * @param priorityLevel minimum priority level required for a message to pass the filter
     * @param port the device to send to; if 0 (default), sends to the first/only managed device
    */
    void SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel, port_handle_t port = 0);

    // TODO - These have (generally) all been moved into ISDevice and are no longer needed here.
    //  these DO NOT operate a all devices, but on a single device, which is redudant at best
    //  and ambiguous at worst.
    //  Kyle Mallory - Remove by 7/23/2025

    /**
    * Get the flash config, returns the latest flash config read from the IMX flash memory
    * @param flashCfg the flash config value
    * @param port the port to get flash config for
    * @return bool whether the flash config is valid, currently synchronized
    */
    bool ImxFlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port = 0);
    bool GpxFlashConfig(gpx_flash_cfg_t &flashCfg, port_handle_t port = 0);

    /**
    * Set the flash config and update flash config on the IMX flash memory
    * @param flashCfg the flash config
    * @param port the port to set flash config for
    * @return true if success
    */
    bool SetImxFlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port = 0);
    bool SetGpxFlashConfig(gpx_flash_cfg_t &flashCfg, port_handle_t port = 0);

    /**
    * Indicates whether the current IMX flash config has been downloaded and available via FlashConfig().
    * @param port the port to get flash config for
    * @return true if the flash config is valid, currently synchronized, otherwise false.
    */
    bool ImxFlashConfigSynced(port_handle_t port = 0);
    bool GpxFlashConfigSynced(port_handle_t port = 0);

    /**
     * @brief Failed to upload flash configuration for any reason.
     *
     * @param port the port to get flash config for
     * @return true Flash config upload was either not received or rejected.
     */
    bool ImxFlashConfigUploadFailure(port_handle_t port = 0);
    bool GpxFlashConfigUploadFailure(port_handle_t port = 0);

    /**
     * @brief Blocking wait calling Update() and SLEEP(10ms) until the flash config has been synchronized.
     *
     * @param port the port
     * @return false When failed to synchronize
     */
    bool WaitForImxFlashCfgSynced(port_handle_t port = 0);
    bool WaitForGpxFlashCfgSynced(port_handle_t port = 0);

    /**
     * @brief Serializes the given device's locally synchronized IMX flash config to a YAML file.
     * @param path Path to the YAML flash config file to write
     * @param port the device to save from; if 0 (default), uses the first/only managed device
     * @return true on success, false on failure.
     */
    bool SaveImxFlashConfigToFile(std::string path, port_handle_t port = 0);
    /** @copydoc SaveImxFlashConfigToFile */
    bool SaveGpxFlashConfigToFile(std::string path, port_handle_t port = 0);

    /**
     * @brief Reads a YAML flash config file and uploads it to the given device.
     * @param path Path to the YAML flash config file to read
     * @param port the device to upload to; if 0 (default), uses the first/only managed device
     * @return true on success, false on failure.
     */
    bool LoadImxFlashConfigFromFile(std::string path, port_handle_t port = 0);
    /** @copydoc LoadImxFlashConfigFromFile */
    bool LoadGpxFlashConfigFromFile(std::string path, port_handle_t port = 0);

    /**
     * @brief Uploads IMX Calibration from file to device
     * @param path File path to calibration JSON file
     * @param port Port handle to device. If NULL, first device found will be used.
     * @return true on success
     * @return false on failure
     */
    bool UploadImxCalibrationFromFile(std::string path, port_handle_t port = 0);

    /**
     * @brief Enable or disable automatic discovery on network ports.
     *
     * When enabled, network port discovery (including mDNS-based discovery) is performed
     * on all supported network ports. When disabled, network ports are not automatically
     * discovered.
     *
     * Calling this function will clear existing network port factories and close any
     * associated ports managed by them, effectively resetting network discovery state.
     * It is safe to call while ports are open, but any open network ports may be closed
     * and will need to be re-established if still required.
     *
     * @param enable Set to true to enable discovery on all network ports, or false to
     *               disable discovery and clear existing network discovery state.
     */
    void SetNetworkPortDiscovery(bool enable = false);

    /**
     * Enable or disable HTTP relay-based port discovery (RelayPortFactory) alongside the
     * existing serial/TCP/mDNS factories. Call in tandem with RelayPortFactory::addRelayHost()
     * + setRelayHostEnabled() to select which relay hosts contribute ports.
     *
     * @param enable Set to true to register RelayPortFactory with PortManager, false to
     *               remove it. Toggling this will also clear PortManager's existing ports,
     *               matching the SetNetworkPortDiscovery() contract.
     */
    void SetRelayPortDiscovery(bool enable = false);

    /**
     * Enable or disable local-serial port discovery (SerialPortFactory) alongside the
     * other registered factories. Defaults to enabled — most consumers want host-attached
     * USB/UART devices visible. Disable when the host is also running a service that
     * holds USB serial ports exclusively (e.g. the bridgeboard relay simulator on the
     * same machine), to avoid the SDK racing the local-OS enumeration against the
     * relay-mediated discovery for the same physical device.
     *
     * @param enable Set to true (default) to register SerialPortFactory, false to remove
     *               it. Toggling clears PortManager's existing ports, matching the
     *               SetNetworkPortDiscovery() / SetRelayPortDiscovery() contract.
     */
    void SetSerialPortDiscovery(bool enable = true);

    /** @brief Used for testing; exposes the legacy ComManager-era global callback/buffer state. @return pointer to this instance's com_manager_cpp_state_t. */
    InertialSense::com_manager_cpp_state_t* ComManagerState() { return &m_comManagerState; }

    /**
     * Registers a custom handler to instantiate discovered devices. Default behavior is to
     * create new ISDevice instances for each new device discovered. Setting a NewDeviceHandler
     * to a custom function allows for instancing a custom ISDevice subclass and/or doing any
     * additional initialization of that device at creation. The handler is provided the port
     * and the device info for the newly discovered device.
     * @param handler a function pointer to be called when a new device is discovered
     * @return the previously registered handler, if any
     */
    pfnOnNewDeviceHandler registerNewDeviceHandler(pfnOnNewDeviceHandler handler) {
        pfnOnNewDeviceHandler oldHandler = m_newDeviceHandler;
        m_newDeviceHandler = handler;
        return oldHandler;
    }

    /** @brief Resolves port to a device (or the first managed device if port is null) and invokes func on it. @return func's result, or false if no matching device is found. */
    template<typename Func>
    bool WithDevice(port_handle_t port, Func&& func)
    {
        device_handle_t device = (port == NULL) ? deviceManager.front() : deviceManager.getDevice(port);
        return (device ? func(device) : false);
    }

    static const int SYNC_FLASH_CFG_CHECK_PERIOD_MS =    200;     //!< (ms) interval between automatic flash-config synchronization checks
    static const int SYNC_FLASH_CFG_TIMEOUT_MS =        3000;     //!< (ms) default timeout for flash-config synchronization waits

protected:
    /** @brief Registered as the ComManager port-error callback; prints errMsg to stdout. @return 0 (always handled). */
    static int OnPortError(port_handle_t port, int errCode, const char *errMsg);

private:
    uint32_t m_timeMs;                                          //!< last time (ms) Update() was called
    InertialSense::com_manager_cpp_state_t m_comManagerState;   //!< legacy ComManager-era global callback/buffer state (see ComManagerState())
    pfnOnNewDeviceHandler m_newDeviceHandler = NULLPTR;         //!< custom new-device allocator, if registered via registerNewDeviceHandler()
    pfnOnCloneDeviceHandler m_cloneDeviceHandler = NULLPTR;     //!< custom device-clone handler, if registered
    pfnIsCommGenMsgHandler  m_handlerNmea = NULLPTR;            //!< user-registered NMEA message callback
    pfnIsCommGenMsgHandler  m_handlerUblox = NULLPTR;           //!< user-registered u-blox message callback
    pfnIsCommGenMsgHandler  m_handlerRtcm3 = NULLPTR;           //!< user-registered RTCM3 message callback
    pfnIsCommGenMsgHandler  m_handlerSpartn = NULLPTR;          //!< user-registered SPARTN message callback
    pfnIsCommGenMsgHandler  m_handlerSeptSbf = NULLPTR;         //!< user-registered Septentrio SBF message callback
    pfnIsCommGenMsgHandler  m_handlerSeptReply = NULLPTR;       //!< user-registered Septentrio reply message callback
    pfnComManagerRmcHandler m_handlerRmc = NULLPTR;             //!< user-registered RMC (real-time message controller) callback
    pfnComManagerParseErrorHandler m_handlerError = NULLPTR;    //!< user-registered data-stream parse error callback (see setErrorHandler())

    cISLogger m_logger;                                          //!< the logger instance used by EnableLogger()/SetLoggerEnabled()
    void* m_logThread;                                           //!< handle to the background logger thread, if logging is enabled
    cMutex m_logMutex;                                           //!< guards m_logPackets / logger state against concurrent access from LoggerThread()
    std::map<port_handle_t, std::vector<p_data_buf_t>> m_logPackets;  //!< per-port queue of packets pending write by LoggerThread()
    time_t m_lastLogReInit;                                     //!< last time the logger was (re)initialized

    char m_clientBuffer[512];                                   //!< unused legacy client buffer
    int m_clientBufferBytesToSend;                              //!< unused legacy client buffer byte count
    bool m_forwardGpgga;                                        //!< unused legacy GPGGA-forwarding flag

    int m_baudRate = IS_BAUDRATE_DEFAULT;                       //!< baud rate used by the most recent Open() call
    bool m_enableDeviceValidation = true;                       //!< whether Open()'d devices are validated (see EnableDeviceValidation())
    bool m_disableBroadcastsOnClose;                            //!< whether Close() sends a stop-broadcasts command to all devices first
    bool m_serialPortDiscoveryEnabled  = true;   //!< last value passed to SetSerialPortDiscovery (default on)
    bool m_networkPortDiscoveryEnabled = false;  //!< last value passed to SetNetworkPortDiscovery
    bool m_relayPortDiscoveryEnabled   = false;  //!< last value passed to SetRelayPortDiscovery

    /** Rebuild PortManager's factory list according to the current m_*PortDiscoveryEnabled flags and clear its existing ports. Shared by all three Set*PortDiscovery setters. */
    void rebuildPortFactories();

    std::vector<std::string> m_ignoredPorts;    //!< port names which should be ignored (known bad, etc).

    std::set<port_handle_t> portsToValidate;    //!< ports which were discovered but have not been validated as an ISDevice

    device_listener_handle_t                m_deviceListenerHandle;  //!< handle for the deviceManagerHandler listener registered on the singleton DeviceManager; removed in ~InertialSense() so a destroyed instance never leaves a dangling listener
    PortManager::port_listener_handle_t     m_portListenerHandle;    //!< handle for the portManagerHandler listener registered on the singleton PortManager; removed in ~InertialSense()
    /**
     * Handle for the port listener updateFirmware() registers to catch devices re-enumerating mid-update.
     *
     * It is a MEMBER rather than a local because the lifetimes do not line up: updateFirmware() only
     * starts the sessions and returns immediately, while the stepping (and therefore the reboots this
     * listener exists to observe) happens later in the caller's loop. Releasing it before returning would
     * defeat it entirely -- which is presumably why the release ended up commented out, leaving a
     * listener registered on a SINGLETON, capturing by reference, for the rest of the process. Same
     * defect as the two handles above, and the same fix: own it here, release it in ~InertialSense().
     */
    PortManager::port_listener_handle_t     m_fwUpdateListenerHandle;


    /** @brief Called each Update() to service the logger thread/state. @return false if the logger failed to open. */
    bool UpdateServer();
    /** @brief Opens/initializes the logger at path with the given save options and starts LoggerThread(). @return false if the logger failed to open. */
    bool EnableLogging(const std::string& path, const cISLogger::sSaveOptions& options = cISLogger::sSaveOptions());
    /** @brief Stops and flushes the logger, if enabled. */
    void DisableLogging();
    /** @return true once every currently managed device has reported valid dev_info_t (see ISDevice::hasDeviceInfo()). */
    bool HasReceivedDeviceInfoFromAllDevices();
    /** @brief Opens the given port pattern (or discovers matching ports) at baudRate, optionally filtered by hardware type. @return true if at least one port was opened. */
    bool OpenPorts(const char* port, int baudRate, uint16_t filterHdwType=IS_HARDWARE_ANY);
    /** @brief Closes all currently open device ports. @param drainBeforeClose if true, waits for pending TX/RX to drain before closing. */
    void ClosePorts(bool drainBeforeClose = false);
    /** @brief Background thread entry point that drains m_logPackets to the logger. @param info the owning InertialSense instance, cast from void*. */
    static void LoggerThread(void* info);
    /** @brief Per-step data callback registered on devices while logging is enabled; queues data into m_logPackets for LoggerThread(). */
    static void StepLogger(void* ctx, const p_data_t* data, port_handle_t port);

    /** @brief PortManager port-event listener; used to detect and react to newly discovered/removed ports. */
    void portManagerHandler(uint8_t event, uint16_t portType, std::string portName, port_handle_t port, PortFactory& portFactory);
    /** @brief DeviceManager device-event listener; used to detect and react to newly discovered/removed devices. */
    void deviceManagerHandler(uint8_t event, device_handle_t device);
};

#endif
