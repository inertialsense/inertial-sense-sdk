/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
#include <vector>

#include "ISConstants.h"
#include "ISTcpClient.h"
#include "ISTcpServer.h"
#include "ISLogger.h"
#include "ISDisplay.h"
#include "ISUtilities.h"
#include "ISSerialPort.h"
#include "ISDataMappings.h"
#include "ISStream.h"
#include "ISDevice.h"
#include "ISClient.h"
#include "message_stats.h"
#include "ISBootloaderThread.h"
#include "ISFirmwareUpdater.h"

extern "C"
{
// [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
#include "data_sets.h"
#include "com_manager.h"

#include "serialPortPlatform.h"
}

class InertialSense;

typedef std::function<void(InertialSense* i, p_data_t* data, int pHandle)> pfnHandleBinaryData;
typedef std::function<void(InertialSense* i, p_ack_t* ack, unsigned char packetIdentifier, int pHandle)> pfnHandleAckData;
typedef void(*pfnStepLogFunction)(InertialSense* i, const p_data_t* data, int pHandle);

/**
* Inertial Sense C++ interface
* Note only one instance of this class per process is supported
*/
class InertialSense : public iISTcpServerDelegate
{
public:
    struct com_manager_cpp_state_t
    {
        // per device vars
        std::vector<ISDevice> devices;

        // common vars
        pfnHandleBinaryData binaryCallbackGlobal;
        pfnHandleAckData binaryAckCallback;     // acknowledgment command and set data callback
#define SIZE_BINARY_CALLBACK	256
        pfnHandleBinaryData binaryCallback[SIZE_BINARY_CALLBACK];
        pfnStepLogFunction stepLogFunction;
        InertialSense* inertialSenseInterface;
        char* clientBuffer;
        int clientBufferSize;
        int* clientBytesToSend;
        int16_t discoveryTimeout = 5000;
    };

    typedef struct
    {
        std::string port;
        std::string error;
    } bootload_result_t;

    /**
    * Constructor
    * @param callbackIsb InertialSense binary received data callback (optional). If specified, ALL BroadcastBinaryData requests will callback to this function.
    * @param callbackRmc Real-time message controller received data callback (optional).
    * @param callbackNmea NMEA received received data callback (optional).
    * @param callbackUblox Ublox binary received data callback (optional).
    * @param callbackRtcm3 RTCM3 received data callback (optional).
    * @param callbackSpartn Spartn received data callback (optional).
    */
    InertialSense(
            pfnHandleBinaryData    callbackIsb = NULL,
            pfnHandleAckData       handlerAck = NULL,
            pfnIsCommAsapMsg       callbackRmc = NULL,
            pfnIsCommGenMsgHandler callbackNmea = NULL,
            pfnIsCommGenMsgHandler callbackUblox = NULL,
            pfnIsCommGenMsgHandler callbackRtcm3 = NULL,
            pfnIsCommGenMsgHandler callbackSpartn = NULL );

    /**
    * Destructor
    */
    virtual ~InertialSense();

    /**
    * Closes any open connection and then opens the device
    * @param port the port to open
    * @param baudRate the baud rate to connect with - supported rates are 115200, 230400, 460800, 921600, 2000000, 3000000
    * @param disableBroadcastsOnClose whether to send a stop broadcasts command to all units on Close
    * @return true if opened, false if failure (i.e. baud rate is bad or port fails to open)
    */
    bool Open(const char* port, int baudRate=IS_BAUDRATE_DEFAULT, bool disableBroadcastsOnClose=false);

    /**
    * Check if the connection is open
    */
    bool IsOpen();

    /**
    * Close the device connection, stop logger if running, and free resources.
    */
    void Close();

    /**
    * Get all open serial port names
    */
    std::vector<std::string> GetPorts();

    /**
    * Get the number of open devices
    * @return the number of open devices
    */
    size_t DeviceCount();

    /**
     * Returns a vector of available, connected devices
     * @return
     */
    std::vector<ISDevice>& getDevices();


    /**
     * Returns a reference to an is_device_t struct that contains information about the specified device
     * @return
     */
    ISDevice& getDevice(uint32_t index);

    /**
    * Call in a loop to send and receive data.  Call at regular intervals as frequently as want to receive data.
    * @return true if updating should continue, false if the process should be shutdown
    */
    bool Update();

    /**
     * Register a callback handler for data stream errors.
     */
    void setErrorHandler(pfnIsCommParseErrorHandler errorHandler) { m_handlerError = errorHandler; }

    /**
    * Enable or disable logging - logging is disabled by default
    * @param logEnable enable or disable the logger - disabling the logger after enabling it will close it and flush all data to disk
    * @param logPath the path to write the log files to
    * @param logType the type of log to write
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
	 * @param device Index of device (pHandle) for raw data.
	 * @param dataSize Number of bytes of raw data.
	 * @param data Pointer to raw data.
	 */
	void LogRawData(ISDevice* device, int dataSize, const uint8_t* data);

	/**
	* Connect to a server and send the data from that server to the IMX. Open must be called first to connect to the IMX unit.
	* @param connectionString the server to connect, this is the data type (RTCM3,IS,UBLOX) followed by a colon followed by connection info (ip:port or serial:baud). This can also be followed by an optional url, user and password, i.e. RTCM3:192.168.1.100:7777:RTCM3_Mount:user:password
	* @return true if connection opened, false if failure
	*/
	bool OpenConnectionToServer(const std::string& connectionString);

    /**
    * Create a server that will stream data from the IMX to connected clients. Open must be called first to connect to the IMX unit.
    * @param connectionString ip address followed by colon followed by port. Ip address is optional and can be blank to auto-detect.
    * @return true if success, false if error
    */
    bool CreateHost(const std::string& connectionString);

    /**
    * Close any open connection to a server
    */
    void CloseServerConnection();

    /**
    * Request device(s) version information (dev_info_t).
    */
    void QueryDeviceInfo();

    /**
    * Turn off all messages.  Current port only if allPorts = false.
    */
    void StopBroadcasts(bool allPorts=true);

    /**
     * Current data streaming will continue streaming at boot. 
     */
    void SavePersistent();

    /**
     * Software reset device(s) with open serial port.
     */
    void SoftwareReset();

    /**
     * @brief Request a specific data set by DID.
     * 
     * @param dataId Data set ID  
     * @param length Byte length of data requested.  Zero means entire data set.
     * @param offset Byte offset into data
     * @param period Broadcast period multiple
     */
    void GetData(eDataIDs dataId, uint16_t length=0, uint16_t offset=0, uint16_t period=0);

    /**
    * Send data to the IMX - this is usually only used for advanced or special cases, normally you won't use this method
    * @param dataId the data id of the data to send
    * @param data the data to send
    * @param length length of data to send
    * @param offset offset into data to send at
    */
    void SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset);

    /**
    * Send raw data to the IMX - (byte swapping disabled)
    * @param dataId the data id of the data to send
    * @param data the data to send
    * @param length length of data to send
    * @param offset offset into data to send at
    */
    void SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length = 0, uint32_t offset = 0);

    /**
    * Send raw (bare) data directly to serial port
    * @param data the data to send
    * @param length length of data to send
    */
    void SendRaw(uint8_t* data, uint32_t length);

    /**
    * Get the device info
    * @param pHandle the pHandle to get device info for
    * @return the device info
    */
    const dev_info_t DeviceInfo(int pHandle = 0);

    /**
    * Get current device system command
    * @param pHandle the pHandle to get sysCmd for
    * @return current device system command
    */
    system_command_t GetSysCmd(int pHandle = 0);

    /**
    * Set device configuration
    * @param pHandle the pHandle to set sysCmd for
    * @param command system command value (see eSystemCommand)
    */
    void SetSysCmd(const uint32_t command, int pHandle = -1);

    /**
     * Sends message to device to set devices Event Filter
     * param Target: 0 = device, 
     *               1 = forward to device GNSS 1 port (ie GPX), 
     *               2 = forward to device GNSS 2 port (ie GPX),
     *               else will return
     *       pHandle: Send in target COM port. 
     *                If arg is < 0 default port will be used 
    */
    void SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel, int pHandle = -1);

    /**
     * @brief IMX/GPX Flash Configuration Synchronization
     *
     * The InertialSense class maintains synchronization between the host's local flash
     * configuration and the configuration stored on the connected IMX or GPX device.
     * 
     * Key Concepts:
     * - Each device maintains local copies of flash configuration:
     *      - IMX: device.imxFlashCfg
     *      - GPX: device.gpxFlashCfg
     * - The device also reports its flash configuration checksum via:
     *      - IMX: device.sysParams.flashCfgChecksum
     *      - GPX: device.gpxStatus.flashCfgChecksum
     * 
     * Synchronization Mechanism:
     * - Periodically (every SYNC_FLASH_CFG_CHECK_PERIOD_MS), SyncFlashConfig() compares
     *   the local checksum with the device-reported checksum.
     * - If mismatched, the full flash configuration is requested from the device.
     * - Uploads are initiated via SetImxFlashConfig() / SetGpxFlashConfig(), which:
     *      - Compute and send only the changed regions of the configuration.
     *      - Update tracking variables (upload time, expected checksum).
     * - The Update() function drives both synchronization and upload completion.
     * 
     * Validation:
     * - ImxFlashConfigSynced() / GpxFlashConfigSynced() return true if:
     *      - Local and device checksums match.
     *      - No upload is pending.
     *      - No upload failure is detected.
     * - ImxFlashConfigUploadFailure() / GpxFlashConfigUploadFailure() detect if a
     *   configuration update was rejected or not yet received by the device.
     * - WaitForImxFlashCfgSynced() / WaitForGpxFlashCfgSynced() can be used to block
     *   until synchronization is complete.
     */

    /**
    * Get the flash config, returns the latest flash config read from the IMX flash memory
    * @param flashCfg the flash config value
    * @param pHandle the port pHandle to get flash config
    * @return bool whether the flash config is valid and currently synchronized
    */
    bool ImxFlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle = 0);
    bool GpxFlashConfig(gpx_flash_cfg_t &flashCfg, int pHandle = 0);

    /**
    * Indicates whether the current flash config has been downloaded and available via ImxFlashConfig()
    * @param pHandle the port pHandle to get flash config
    * @return bool whether the flash config is valid and currently synchronized
    */
    bool ImxFlashConfigSynced(int pHandle = 0);
    bool GpxFlashConfigSynced(int pHandle = 0);

    /**
     * @brief Failed to upload flash configuration for any reason.   
     * 
     * @param pHandle the port pHandle to get flash config for
     * @return true Flash config upload was either not received or rejected.
     */
    bool ImxFlashConfigUploadFailure(int pHandle = 0);
    bool GpxFlashConfigUploadFailure(int pHandle = 0);

    /**
    * Set the flash config and update flash config on the IMX flash memory
    * @param flashCfg the flash config
    * @param pHandle the pHandle to set flash config for
    * @return true if success
    */
    bool SetImxFlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle = 0);
    bool SetGpxFlashConfig(gpx_flash_cfg_t &flashCfg, int pHandle = 0);

    /**
     * @brief Blocking wait calling Update() and SLEEP(10ms) until the flash config has been synchronized. 
     * 
     * @param pHandle the port pHandle
     * @return false When failed to synchronize
     */
    bool WaitForImxFlashCfgSynced(bool forceSync = false, uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS, int pHandle = 0);
    bool WaitForGpxFlashCfgSynced(bool forceSync = false, uint32_t timeout = SYNC_FLASH_CFG_TIMEOUT_MS, int pHandle = 0);

    void ProcessRxData(int pHandle, p_data_t* data);
    void ProcessRxNmea(int pHandle, const uint8_t* msg, int msgSize);

    /**
     * Request a specific device broadcast binary data
     * @param pHandle the device's pHandle to request data from
     * @param dataId the data id (DID_* - see data_sets.h) to broadcast
     * @param periodMultiple a scalar that the source period is multiplied by to give the output period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
     * @return true if success, false if error - if callback is NULL and no global callback was passed to the constructor, this will return false
     */
    bool BroadcastBinaryData(int pHandle, uint32_t dataId, int periodMultiple);

    /**
    * Broadcast binary data
    * @param dataId the data id (DID_* - see data_sets.h) to broadcast
    * @param periodMultiple a scalar that the source period is multiplied by to give the output period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
    * @param callback optional callback for this dataId
    * @return true if success, false if error - if callback is NULL and no global callback was passed to the constructor, this will return false
    */
    bool BroadcastBinaryData(uint32_t dataId, int periodMultiple, pfnHandleBinaryData callback = NULL);

    /**
    * Enable streaming of predefined set of messages.  The default preset, RMC_PRESET_INS, stream data necessary for post processing.
    * @param rmcPreset realtimeMessageController preset
    */
    void BroadcastBinaryDataRmcPreset(uint64_t rmcPreset=RMC_PRESET_INS, uint32_t rmcOptions=0);

    /**
    * Get the number of bytes read or written to/from client or server connections
    * @return byte count
    */
    uint64_t ClientServerByteCount() { return m_clientServerByteCount; }

    /**
    * Get the current number of client connections
    * @return int number of current client connected
    */
    int ClientConnectionCurrent() { return m_clientConnectionsCurrent; }

    /**
    * Get the total number of client connections
    * @return int number of total client that have connected
    */
    int ClientConnectionTotal() { return m_clientConnectionsTotal; }

    /**
    * Get TCP server IP address and port (i.e. "127.0.0.1:7777")
    * @return string IP address and port
    */
    std::string TcpServerIpAddressPort() { return (m_tcpServer.IpAddress().empty() ? "127.0.0.1" : m_tcpServer.IpAddress()) + ":" + std::to_string(m_tcpServer.Port()); }

    /**
    * Get Client connection info string (i.e. "127.0.0.1:7777")
    * @return string IP address and port
    */
    std::string ClientConnectionInfo() { return m_clientStream->ConnectionInfo(); }

    /**
    * Flush all data from receive port
    */
    void FlushRx()
    {
        uint8_t buf[10];
        for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
        {
            if (!serialPortIsOpen(&m_comManagerState.devices[i].serialPort))
            {
                while (serialPortReadTimeout(&(m_comManagerState.devices[i].serialPort), buf, sizeof(buf), 0));
            }
        }
    }

    /**
    * Get access to the underlying serial port
    * @param pHandle the pHandle to get the serial port for
    * @return the serial port
    */
    serial_port_t* SerialPort(int pHandle = 0)
    {
        if ((size_t)pHandle >= m_comManagerState.devices.size())
        {
            return NULL;
        }
        return &(m_comManagerState.devices[pHandle].serialPort);
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

    /**
    * Bootload a file - if the bootloader fails, the device stays in bootloader mode and you must call BootloadFile again until it succeeds. If the bootloader gets stuck or has any issues, power cycle the device.
    * Please ensure that all other connections to the com port are closed before calling this function.
    *
    TODO: Param documentation
    */
    static is_operation_result BootloadFile(
            const std::string& comPort,
            const uint32_t serialNum,
            const std::string& fileName,
            const std::string& blFileName,
            bool forceBootloaderUpdate,
            int baudRate = IS_BAUDRATE_921600,
            ISBootloader::pfnBootloadProgress uploadProgress = NULLPTR,
            ISBootloader::pfnBootloadProgress verifyProgress = NULLPTR,
            ISBootloader::pfnBootloadStatus infoProgress = NULLPTR,
            void (*waitAction)() = NULLPTR
    );

    /**
     * V2 firmware update mechanism. Calling this function will attempt to inititate a firmware update with the targeted device(s) on the connected port(s), with callbacks to provide information about the status
     * of the update process.
     * @param comPort
     * @param baudRate
     * @param targetDevice the device which all commands should be directed to
     * @param cmds a vector of strings to be interpreted as commands, performed in sequence.  ie ["slot=0","upload=myfirmware.bin","slot=1","upload=configuration.conf","softReset"]
     * @param uploadProgress
     * @param verifyProgress
     * @param infoProgress
     * @param waitAction
     * @param LoadFlashConfig
     * @return
     */
    is_operation_result updateFirmware(
            const std::string& comPort,
            int baudRate,
            fwUpdate::target_t targetDevice,
            std::vector<std::string> cmds,
            ISBootloader::pfnBootloadProgress uploadProgress,
            ISBootloader::pfnBootloadProgress verifyProgress,
            ISBootloader::pfnBootloadStatus infoProgress,
            void (*waitAction)()
    );

    is_operation_result updateFirmware(
            ISDevice& device,
            fwUpdate::target_t targetDevice,
            std::vector<std::string> cmds,
            ISBootloader::pfnBootloadProgress uploadProgress,
            ISBootloader::pfnBootloadProgress verifyProgress,
            ISBootloader::pfnBootloadStatus infoProgress,
            void (*waitAction)()
    );

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
    * Gets current update status for selected device index
    * @param deviceIndex
    */
    fwUpdate::update_status_e getUpdateStatus(uint32_t deviceIndex);

    /**
    * Gets device index from COM port
    * @param COM port
    */
    int getUpdateDeviceIndex(const char* com);

    /**
    * Gets current devInfo using device index
    * @param dev_info_t devI
    * @param uint32_t deviceIndex
    */
    bool getUpdateDevInfo(dev_info_t* devI, uint32_t deviceIndex);

    /**
     * @brief SaveImxFlashConfigToFile
     * @param path - Path to YAML flash config file
     * @param pHandle - Handle of current device
     */
    bool SaveImxFlashConfigToFile(std::string path, int pHandle = 0);
    bool SaveGpxFlashConfigToFile(std::string path, int pHandle = 0);

    /**
     * @brief LoadFlashConfig
     * @param path - Path to YAML flash config file
     * @param pHandle - Handle of current device
     * @return -1 for failure to upload file, 0 for success.
     */
    bool LoadImxFlashConfigFromFile(std::string path, int pHandle = 0);
    bool LoadGpxFlashConfigFromFile(std::string path, int pHandle = 0);

    std::string ServerMessageStatsSummary() { return messageStatsSummary(m_serverMessageStats); }
    std::string ClientMessageStatsSummary() { return messageStatsSummary(m_clientMessageStats); }

    // Used for testing
    InertialSense::com_manager_cpp_state_t* ComManagerState() { return &m_comManagerState; }
    ISDevice* ComManagerDevice(int pHandle=0) { if (pHandle >= (int)m_comManagerState.devices.size()) return NULLPTR; return &(m_comManagerState.devices[pHandle]); }

    static const int SYNC_FLASH_CFG_CHECK_PERIOD_MS =    200;
    static const int SYNC_FLASH_CFG_TIMEOUT_MS =        3000;

protected:
    bool OnClientPacketReceived(const uint8_t* data, uint32_t dataLength);
    void OnClientConnecting(cISTcpServer* server) OVERRIDE;
    void OnClientConnected(cISTcpServer* server, is_socket_t socket) OVERRIDE;
    void OnClientConnectFailed(cISTcpServer* server) OVERRIDE;
    void OnClientDisconnected(cISTcpServer* server, is_socket_t socket) OVERRIDE;

private:
    uint32_t m_timeMs;
    InertialSense::com_manager_cpp_state_t m_comManagerState;
    pfnIsCommAsapMsg       m_handlerRmc = NULLPTR;
    pfnIsCommGenMsgHandler m_handlerNmea = NULLPTR;
    pfnIsCommGenMsgHandler m_handlerUblox = NULLPTR;
    pfnIsCommGenMsgHandler m_handlerRtcm3 = NULLPTR;
    pfnIsCommGenMsgHandler m_handlerSpartn = NULLPTR;
    pfnIsCommParseErrorHandler m_handlerError = NULLPTR;
    cISLogger m_logger;
    void* m_logThread;
    cMutex m_logMutex;
    std::map<int, std::vector<p_data_buf_t>> m_logPackets;
    time_t m_lastLogReInit;

    char m_clientBuffer[512];
    int m_clientBufferBytesToSend;
    bool m_forwardGpgga;

    cISTcpServer m_tcpServer;
    cISSerialPort m_serialServer;
    cISStream* m_clientStream;				// Our client connection to a server
    uint64_t m_clientServerByteCount;
    int m_clientConnectionsCurrent = 0;
    int m_clientConnectionsTotal = 0;
    mul_msg_stats_t m_clientMessageStats = {};

    bool m_enableDeviceValidation = true;
    bool m_disableBroadcastsOnClose;
    com_manager_init_t m_cmInit;
    com_manager_port_t *m_cmPorts;
    is_comm_instance_t m_gpComm;
    uint8_t m_gpCommBuffer[PKT_BUF_SIZE];
    mul_msg_stats_t m_serverMessageStats = {};
    unsigned int m_syncCheckTimeMs = 0;

    // returns false if logger failed to open
    bool UpdateServer();
    bool UpdateClient();
    bool EnableLogging(const std::string& path, const cISLogger::sSaveOptions& options = cISLogger::sSaveOptions());
    void DisableLogging();
    bool HasReceivedDeviceInfo(size_t index);
    bool HasReceivedDeviceInfoFromAllDevices();
    void RemoveDevice(size_t index);
    bool OpenSerialPorts(const char* port, int baudRate);
    void CloseSerialPorts(bool drainBeforeClose = false);
    static void LoggerThread(void* info);
    static void StepLogger(InertialSense* i, const p_data_t* data, int pHandle);
    static void BootloadStatusUpdate(void* obj, const char* str);

    void CheckRequestFlashConfig(unsigned int timeMs, unsigned int &uploadTimeMs, bool synced, int port, uint16_t did);
    void SyncFlashConfig(unsigned int timeMs);
    void DeviceSyncFlashCfg(int devIndex, unsigned int timeMs, uint16_t flashCfgDid, uint16_t syncDid, unsigned int &uploadTimeMs, uint32_t &flashCfgChecksum, uint32_t &syncChecksum, uint32_t &uploadChecksum);
    bool UploadFlashConfigDiff(int pHandle, uint8_t* newData, uint8_t* curData, size_t sizeBytes, uint32_t did, uint32_t& uploadTimeMsOut, uint32_t& checksumOut);
    void UpdateFlashConfigChecksum(nvm_flash_cfg_t &flashCfg);
    bool ValidFlashCfgCksum(uint32_t checksum) { return (checksum != 0xFFFFFFFF); }
};

#endif
