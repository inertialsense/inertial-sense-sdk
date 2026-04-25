/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <algorithm>
#include <vector>

#include "InertialSense.h"
#include "ISmDnsPortFactory.h"
#include "TcpPortFactory.h"
#include "RelayPortFactory.h"
#include "protocol_nmea.h"
#include "protocol/FirmwareUpdate.h"

#include "imx_defaults.h"
#include "uri.hpp"

#if !PLATFORM_IS_EMBEDDED
#include "ISBootloaderThread.h"
#endif

using namespace std;

#define PRINT_DEBUG 0
#if PRINT_DEBUG
#define DEBUG_PRINT(...)    printf("L%d: ", __LINE__); printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) 
#endif

static InertialSense *s_is = NULL;
static InertialSense::com_manager_cpp_state_t *s_cm_state = NULL;

/**
 * General Purpose IS-binary protocol handler for the InertialSense class.
 * This is called anytime ISB packets are received by any of the underlying ports
 * which are managed by the InertialSense and CommManager classes.  Eventually
 * this should be moved into the ISDevice class, where devices of different types
 * can handle their data independently. There could be a hybrid approach here
 * where this function would (should?) locate the ISDevice by its port, and then
 * redirect to the ISDevice specific callback.
 * @param data The data which was parsed and is ready to be consumed
 * @param port The port which the data was received from
 * @return 0 if this data packet WILL NOT BE processed again by other handlers
 *   any other value indicates that the packet MAY BE processed by other handlers.
 *   No guarantee is given that other handlers will process this packet if the
 *   return value is non-zero, but is guaranteed that it will no reprocess this
 *   packet if a zero-value is returned.
 */
static int staticProcessRxData(void *ctx, p_data_t* data, port_handle_t port)
{
    if ((port == NULLPTR) || (data->hdr.id >= (sizeof(s_cm_state->binaryCallback)/sizeof(pfnHandleBinaryData))))
    {
        return -1;
    }

    pfnHandleBinaryData handler = s_cm_state->binaryCallback[data->hdr.id];
    s_cm_state->stepLogFunction(ctx, data, port);

    if (handler)
    {
        handler(s_cm_state->inertialSenseInterface, data, port);
    }

    pfnHandleBinaryData handlerGlobal = s_cm_state->binaryCallbackGlobal;
    if (handlerGlobal)
    {
        // Called for all DID's
        handlerGlobal(s_cm_state->inertialSenseInterface, data, port);
    }

    s_cm_state->inertialSenseInterface->ProcessRxData(port, data);

    switch (data->hdr.id)
    {
        case DID_GPS1_POS:
            static time_t lastTime;
            time_t currentTime = time(NULLPTR);
            if (abs(currentTime - lastTime) > 5)
            {   // Update every 5 seconds
                lastTime = currentTime;
                gps_pos_t &gps = *((gps_pos_t*)data->ptr);
                if ((gps.status&GPS_STATUS_FIX_MASK) >= GPS_STATUS_FIX_3D)
                {
                    *s_cm_state->clientBytesToSend = nmea_gga(s_cm_state->clientBuffer, s_cm_state->clientBufferSize, gps);
                }
            }
            break;
    }
    return 0;
}

static int staticProcessAck(void* ctx, p_ack_t* ack, unsigned char packetIdentifier, port_handle_t port)
{
    pfnHandleAckData handler = s_cm_state->binaryAckCallback;
    if (handler != NULLPTR)
    {
        handler(s_cm_state->inertialSenseInterface, ack, packetIdentifier, port);
    }
    return 0;
}

static int staticProcessRxNmea(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port)
{
    if (port)
    {
        s_cm_state->inertialSenseInterface->ProcessRxNmea(port, msg, msgSize);
    }
    return 0;
}

InertialSense::InertialSense(std::vector<PortFactory*> pFactories, std::vector<DeviceFactory*> dFactories)
{
    s_is = this;
    s_cm_state = &m_comManagerState;
    m_logThread = NULLPTR;
    m_lastLogReInit = time(0);
    m_clientBufferBytesToSend = 0;
    m_disableBroadcastsOnClose = false;  // For Intel.

    // register device factories before we do port factories, so if (for some strange reason) ports get discovered early, there is a device factory to handle it
    // addDeviceFactory(&ImxDeviceFactory::getInstance());
    if (dFactories.empty()) {
        deviceManager.addDeviceFactory((DeviceFactory *) &ImxDeviceFactory::getInstance());
    } else {
        for (auto f: dFactories) deviceManager.addDeviceFactory(f);
    }
    deviceManager.addDeviceListener([this](auto && PH1, auto && PH2) { deviceManagerHandler(PH1, PH2); });

    if (pFactories.empty()) {
        SerialPortFactory& spf = SerialPortFactory::getInstance();
        spf.portOptions.defaultBaudRate = BAUDRATE_921600;
        spf.portOptions.defaultBlocking = false;
        portManager.addPortFactory(&spf);
        TcpPortFactory& tpf = TcpPortFactory::getInstance();
        tpf.portOptions.defaultBlocking = false;
        portManager.addPortFactory(&tpf);
        ISmDnsPortFactory& mdtf = ISmDnsPortFactory::getInstance();
        mdtf.portOptions.defaultBlocking = false;
        portManager.addPortFactory(&mdtf);
    } else {
        for (auto f : pFactories) portManager.addPortFactory(f);
    }
    portManager.addPortListener([this](auto && PH1, auto && PH2, auto && PH3, auto && PH4, auto && PH5) { portManagerHandler(PH1, PH2, PH3, PH4, PH5); });


    for (int i=0; i<int(sizeof(m_comManagerState.binaryCallback)/sizeof(pfnHandleBinaryData)); i++)
    {
        m_comManagerState.binaryCallback[i] = {};
    }
    m_comManagerState.binaryCallbackGlobal = nullptr; // handlerIsb;
    m_comManagerState.stepLogFunction = &InertialSense::StepLogger;
    m_comManagerState.inertialSenseInterface = this;
    m_comManagerState.clientBuffer = m_clientBuffer;
    m_comManagerState.clientBufferSize = sizeof(m_clientBuffer);
    m_comManagerState.clientBytesToSend = &m_clientBufferBytesToSend;
    comManagerGetGlobal()->assignUserPointer(&m_comManagerState);
    // memset(&m_cmInit, 0, sizeof(m_cmInit));
    // m_cmPorts = NULLPTR;

    m_newDeviceHandler = nullptr; // handlerNewDevice;
    // m_cloneDeviceHandler = handleClonedDevices;

    // Rx data callback functions
    m_handlerRmc    = nullptr;
    m_handlerNmea   = nullptr;
    m_handlerUblox  = nullptr;
    m_handlerRtcm3  = nullptr;
    m_handlerSpartn = nullptr;
    m_handlerSeptSbf = nullptr;
    m_handlerSeptReply = nullptr;
}

InertialSense::InertialSense() : InertialSense(std::vector<PortFactory*>(), std::vector<DeviceFactory*>()) {
    s_is = this;
}

InertialSense::InertialSense(
        pfnHandleBinaryData     handlerIsb,
        pfnHandleAckData        handlerIsbAck,
        pfnComManagerRmcHandler handlerRmc,
        pfnIsCommGenMsgHandler  handlerNmea,
        pfnIsCommGenMsgHandler  handlerUblox,
        pfnIsCommGenMsgHandler  handlerRtcm3,
        pfnIsCommGenMsgHandler  handlerSpartn,
        pfnIsCommGenMsgHandler  handlerSeptSbf,
        pfnIsCommGenMsgHandler  handlerSeptReply,
        pfnOnNewDeviceHandler handlerNewDevice)
{
    s_is = this;
    s_cm_state = &m_comManagerState;
    m_logThread = NULLPTR;
    m_lastLogReInit = time(0);
    m_clientBufferBytesToSend = 0;
    m_disableBroadcastsOnClose = false;  // For Intel.

    deviceManager.addDeviceFactory((DeviceFactory*)&ImxDeviceFactory::getInstance());
    deviceManager.addDeviceListener([this](auto && PH1, auto && PH2) { deviceManagerHandler(PH1, PH2); });

    SetNetworkPortDiscovery();
    portManager.addPortListener([this](auto && PH1, auto && PH2, auto && PH3, auto && PH4, auto && PH5) { portManagerHandler(PH1, PH2, PH3, PH4, PH5); });

    for (int i=0; i<int(sizeof(m_comManagerState.binaryCallback)/sizeof(pfnHandleBinaryData)); i++)
    {
        m_comManagerState.binaryCallback[i] = {};
    }
    m_comManagerState.binaryCallbackGlobal = handlerIsb;
    m_comManagerState.binaryAckCallback = handlerIsbAck;
    m_comManagerState.stepLogFunction = &InertialSense::StepLogger;
    m_comManagerState.inertialSenseInterface = this;
    m_comManagerState.clientBuffer = m_clientBuffer;
    m_comManagerState.clientBufferSize = sizeof(m_clientBuffer);
    m_comManagerState.clientBytesToSend = &m_clientBufferBytesToSend;
    comManagerGetGlobal()->assignUserPointer(&m_comManagerState);
    // memset(&m_cmInit, 0, sizeof(m_cmInit));
    // m_cmPorts = NULLPTR;

    m_newDeviceHandler = handlerNewDevice;
    // m_cloneDeviceHandler = handleClonedDevices;

    // Rx data callback functions
    m_handlerRmc    = handlerRmc;
    m_handlerNmea   = handlerNmea;
    m_handlerUblox  = handlerUblox;
    m_handlerRtcm3  = handlerRtcm3;
    m_handlerSpartn = handlerSpartn;
    m_handlerSeptSbf = handlerSeptSbf;
    m_handlerSeptReply = handlerSeptReply;
}

InertialSense::~InertialSense()
{
    Close();
    DisableLogging();
    s_is = nullptr;
}

InertialSense* InertialSense::getLastInstance() {
    return s_is;
}

bool InertialSense::EnableLogging(const string& path, const cISLogger::sSaveOptions& options)
{
    cMutexLocker logMutexLocker(&m_logMutex);

    if (!m_logger.InitSave(path, options))
    {
        return false;
    }
    m_logger.EnableLogging(true);
    for (auto d : deviceManager)
    {
        m_logger.registerDevice(d);
    }
    if (m_logThread == NULLPTR)
    {
        m_logThread = threadCreateAndStart(&InertialSense::LoggerThread, this, "LoggingThread");
    }
    return true;
}

void InertialSense::DisableLogging()
{
    if (m_logger.Enabled() || m_logThread != NULLPTR)
    {
        m_logger.EnableLogging(false);
        printf("Disabling logger...");
        fflush(stdout);

        // just sets a bool no need to lock
        threadJoinAndFree(m_logThread);
        m_logThread = NULLPTR;
        m_logger.CloseAllFiles();
    }
}


bool InertialSense::HasReceivedDeviceInfoFromAllDevices()
{
    if (deviceManager.empty())
        return false;

    for (auto device : deviceManager)
    {
        if (!device->hasDeviceInfo())
            return false;
    }

    return true;
}

void InertialSense::LoggerThread(void* info)
{
    bool running = true;
    InertialSense* inertialSense = (InertialSense*)info;

    // gather packets in memory
    map<port_handle_t, vector<p_data_buf_t>> packets;

    while (running)
    {
        SLEEP_MS(20);
        {
            // lock so we can read and clear m_logPackets
            cMutexLocker logMutexLocker(&inertialSense->m_logMutex);
            for (auto [port, pkts] : inertialSense->m_logPackets)
            {
                packets[port] = pkts;
            }

            // clear shared memory
            inertialSense->m_logPackets.clear();

            // update running state
            running = inertialSense->m_logger.Enabled();
        }

        if (running)
        {
            // log the packets
            for (auto [port, pkts] : packets)
            {
                if (inertialSense->m_logger.Type() != cISLogger::LOGTYPE_RAW) {
                    for (auto& pkt : pkts) {
                        auto device = inertialSense->getDevice(port);
                        if (!inertialSense->m_logger.LogData(device->devLogger, &pkt.hdr, pkt.buf)) {
                            // Failed to write to log
                            SLEEP_MS(20); // FIXME:  This maybe problematic, as it may unnecessarily delay the thread, leading run-away memory usage.
                        }
                    }
                }

                // clear all log data for this port
                pkts.clear();   // TODO: Make sure this clears the backing vector, not just the local instance
            }
        }

        inertialSense->m_logger.Update();
    }

    printf("...logger thread terminated.\n");
}

void InertialSense::StepLogger(void* ctx, const p_data_t* data, port_handle_t port)
{
    if (!ctx) return;

    InertialSense* i = getLastInstance(); // (InertialSense*)ctx;
    cMutexLocker logMutexLocker(&i->m_logMutex);
    if (i->m_logger.Enabled())
    {
        p_data_buf_t d;
        d.hdr = data->hdr;
        memcpy(d.buf, data->ptr, d.hdr.size);
        i->m_logPackets[port].push_back(d);
    }
}

bool InertialSense::EnableLogger(
    bool logEnable,
    const string& logPath,
    const cISLogger::sSaveOptions &logOptions,
    uint64_t rmcPreset,
    uint32_t rmcOptions)
{
    if (logEnable)
    {
        if (m_logThread != NULLPTR)
        {
            // already logging
            return true;
        }

        if (rmcPreset)
        {
            BroadcastBinaryDataRmcPreset(rmcPreset, rmcOptions);
        }
        return EnableLogging(logPath, logOptions);
    }

    // !enable, shutdown logger gracefully
    DisableLogging();
    return true;
}

[[deprecated("Not recommended for future development. Use EnableLogger() instead.")]]
bool InertialSense::SetLoggerEnabled(
    bool logEnable,
    const string& logPath,
    cISLogger::eLogType logType,
    uint64_t rmcPreset,
    uint32_t rmcOptions,
    float driveUsageLimitPercent,
    uint32_t maxFileSize,
    const string& subFolder)
{
    cISLogger::sSaveOptions logOptions;
    logOptions.driveUsageLimitPercent = driveUsageLimitPercent;
    logOptions.maxFileSize = maxFileSize;
    logOptions.subDirectory = subFolder;
    return EnableLogger(logEnable, logPath, logOptions, rmcPreset, rmcOptions);
}



bool InertialSense::Update()
{
    m_timeMs = current_timeMs();

//    m_correctionsServer.step();
//    if (m_correctionService.step() <= 0) {
//        // usually an error because the correction service (base) port is closed or invalid.
//        portOpen(m_correctionService.getSourcePort());
//    }

    bool anyOpen = false;   // if all serial ports have closed, shutdown
    for (auto device : deviceManager) {
        if (device) {
            device->step();
            if (device->fwUpdateInProgress() || device->isConnected())
                anyOpen = true;
        }
    }

    return anyOpen;
}

bool InertialSense::Open(const char* port, int baudRate, bool disableBroadcastsOnClose)
{
    // null com port, just use other features of the interface like ntrip
    if (port[0] == '0' && port[1] == '\0')
    {
        return true;
    }

    m_disableBroadcastsOnClose = false;
    m_baudRate = baudRate;
    if (OpenPorts(port, baudRate))
    {
        m_disableBroadcastsOnClose = disableBroadcastsOnClose;
        return true;
    }
    return false;
}

bool InertialSense::IsOpen()
{
    return (DeviceCount() != 0 && portIsOpened(deviceManager.front()->port));
}

void InertialSense::Close()
{
    EnableLogger(false);
    if (m_disableBroadcastsOnClose)
    {
        StopBroadcasts(false);
        SLEEP_MS(100);
    }

    // Note the distinction here; we are closing ports, not devices...  Maybe we should do this differently though?
    for (auto device : deviceManager) {
        if (device->isConnected()) {
            portFlush(device->port);
        }
        device->disconnect();
    }

}

#if 0
vector<string> InertialSense::GetPortNames()
{
    vector<string> ports;
    for (auto device : m_comManagerState.devices) { ports.push_back(portName(device->port)); }
    return ports;
}

void InertialSense::QueryDeviceInfo()
{
    for (auto device : m_comManagerState.devices) { device->QueryDeviceInfo(); }
}

void InertialSense::StopBroadcasts(bool allPorts)
{
    for (auto device : m_comManagerState.devices) { device->StopBroadcasts(allPorts); }
}

void InertialSense::SavePersistent()
{
    for (auto device : m_comManagerState.devices) { device->SavePersistent(); }
}

void InertialSense::SoftwareReset()
{
    for (auto device : m_comManagerState.devices) { device->SoftwareReset(); }
}

void InertialSense::GetData(eDataIDs dataId, uint16_t length, uint16_t offset, uint16_t period)
{
    for (auto device : m_comManagerState.devices) { device->GetData(dataId, length, offset, period); }
}

void InertialSense::SendData(eDataIDs dataId, void* data, uint32_t length, uint32_t offset)
{
    for (auto device : m_comManagerState.devices) { device->SendData(dataId, data, length, offset); }
}

void InertialSense::SendRaw(void* data, uint32_t length)
{
    for (auto device : m_comManagerState.devices) { device->SendRaw(data, length); }
}

void InertialSense::Send(uint8_t pktInfo, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    for (auto device : m_comManagerState.devices) { device->Send(pktInfo, data, did, size, offset); }
}

void InertialSense::SetSysCmd(const uint32_t command, port_handle_t port)
{
    if (port == nullptr)
    {   // Send to all
        for (auto device : m_comManagerState.devices) { device->SetSysCmd(command); }
    }
    else
    {   // Specific port
        device_handle_t device = DeviceByPort(port);
        if (device) device->SetSysCmd(command);
    }
}

/**
 * Sends message to device to set devices Event Filter
 * param Target: 0 = device, 
 *               1 = forward to device GNSS 1 port (ie GPX), 
 *               2 = forward to device GNSS 2 port (ie GPX),
 *               else will return
 *       port: Send in target COM port.
 *                If arg is < 0 default port will be used 
*/
void InertialSense::SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel, port_handle_t port)
{
    #define EVENT_MAX_SIZE (1024 + DID_EVENT_HEADER_SIZE)
    uint8_t data[EVENT_MAX_SIZE] = {0};

    did_event_t event = {
        .time = 123,
        .senderSN = 0,
        .senderHdwId = 0,
        .length = sizeof(did_event_filter_t),
    };

    did_event_filter_t filter = {
        .portMask = portMask,
    };

    filter.eventMask.priorityLevel = priorityLevel;
    filter.eventMask.msgTypeIdMask = msgTypeIdMask;

    if (target == 0)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_FILTER;
    else if (target == 1)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_GNSS1_FILTER;
    else if (target == 2)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_GNSS2_FILTER;
    else 
        return;

    memcpy(data, &event, DID_EVENT_HEADER_SIZE);
    memcpy((void*)(data+DID_EVENT_HEADER_SIZE), &filter, _MIN(sizeof(did_event_filter_t), EVENT_MAX_SIZE-DID_EVENT_HEADER_SIZE));

    if (!port)
        SendData(DID_EVENT, data, DID_EVENT_HEADER_SIZE + event.length, 0);
    else    
        comManagerSendData(port, data, DID_EVENT, DID_EVENT_HEADER_SIZE + event.length, 0);
}

#endif

bool InertialSense::ImxFlashConfig(nvm_flash_cfg_t& flashCfg, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) { return dev->ImxFlashConfig(flashCfg); });
}

bool InertialSense::GpxFlashConfig(gpx_flash_cfg_t& flashCfg, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) { return dev->GpxFlashConfig(flashCfg); });
}

bool InertialSense::SetImxFlashConfig(nvm_flash_cfg_t& flashCfg, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) { return dev->SetImxFlashConfig(flashCfg); });
}

bool InertialSense::SetGpxFlashConfig(gpx_flash_cfg_t& flashCfg, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) { return dev->SetGpxFlashConfig(flashCfg); });
}

bool InertialSense::ImxFlashConfigSynced(port_handle_t port)
{
    return WithDevice(port, [](device_handle_t dev) { return dev->ImxFlashConfigSynced(); });
}

bool InertialSense::GpxFlashConfigSynced(port_handle_t port)
{
    return WithDevice(port, [](device_handle_t dev) { return dev->GpxFlashConfigSynced(); });
}

bool InertialSense::ImxFlashConfigUploadFailure(port_handle_t port)
{
    return WithDevice(port, [](device_handle_t dev) { return dev->ImxFlashConfigUploadFailure(); });
}

bool InertialSense::GpxFlashConfigUploadFailure(port_handle_t port)
{
    return WithDevice(port, [](device_handle_t dev) { return dev->GpxFlashConfigUploadFailure(); });
}

bool InertialSense::WaitForImxFlashCfgSynced(port_handle_t port)
{
    return WithDevice(port, [](device_handle_t dev) { return dev->WaitForImxFlashCfgSynced(); });
}

bool InertialSense::WaitForGpxFlashCfgSynced(port_handle_t port)
{
    return WithDevice(port, [](device_handle_t dev) { return dev->WaitForGpxFlashCfgSynced(); });
}

bool InertialSense::SaveImxFlashConfigToFile(std::string path, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) { return dev->SaveImxFlashConfigToFile(path); });
}

bool InertialSense::SaveGpxFlashConfigToFile(std::string path, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) { return dev->SaveGpxFlashConfigToFile(path); });
}

bool InertialSense::LoadImxFlashConfigFromFile(std::string path, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) { return dev->LoadImxFlashConfigFromFile(path); });
}

bool InertialSense::LoadGpxFlashConfigFromFile(std::string path, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) { return dev->LoadGpxFlashConfigFromFile(path); });
}

bool InertialSense::UploadImxCalibrationFromFile(std::string path, port_handle_t port)
{
    return WithDevice(port, [&](device_handle_t dev) {
        FIX8::basic_uri uri(path);
        if (uri.parse()) {
            return (dev->UploadIMXCalibrationFromURL(path) == 200);
        }

        return dev->UploadImxCalibrationFromFile(path);
    });
}

// Rebuild the PortManager's factory list according to the current enable flags.
// SerialPortFactory is included by default (m_serialPortDiscoveryEnabled defaults to true);
// the network and relay factories are opt-in. TcpPortFactory is always present — direct
// tcp:// URLs are a host-side capability, not a discovery surface.
void InertialSense::rebuildPortFactories()
{
    portManager.clearPortFactories();
    if (m_serialPortDiscoveryEnabled) {
        portManager.addPortFactory((PortFactory*)&(SerialPortFactory::getInstance()));
    }
    portManager.addPortFactory((PortFactory*)&(TcpPortFactory::getInstance()));
    if (m_networkPortDiscoveryEnabled) {
        portManager.addPortFactory((PortFactory*)&(ISmDnsPortFactory::getInstance()));
    }
    if (m_relayPortDiscoveryEnabled) {
        portManager.addPortFactory((PortFactory*)&(RelayPortFactory::getInstance()));
    }
    // Removes all ports from the PortManager.
    portManager.clear();
}

void InertialSense::SetSerialPortDiscovery(bool enable)
{
    m_serialPortDiscoveryEnabled = enable;
    rebuildPortFactories();
}

void InertialSense::SetNetworkPortDiscovery(bool enable)
{
    m_networkPortDiscoveryEnabled = enable;
    rebuildPortFactories();
}

void InertialSense::SetRelayPortDiscovery(bool enable)
{
    m_relayPortDiscoveryEnabled = enable;
    rebuildPortFactories();
}

void InertialSense::ProcessRxData(port_handle_t port, p_data_t* data)
{
    if (data->hdr.size==0 || data->ptr==NULL)
    {
        return;
    }

    // THIS IS MOVED TO ISDevice::ProcessRxData()
}

// return 0 on success, -1 on failure
void InertialSense::ProcessRxNmea(port_handle_t port, const uint8_t* msg, int msgSize)
{
    switch (getNmeaMsgId(msg, msgSize)) {
        case NMEA_MSG_ID_INFO: {
            // Device Info
            auto device = getDevice(port);
            if (!device) {
                dev_info_t devInfo = {};
                if (!nmea_parse_info(devInfo, (const char *) msg, msgSize)) {
                    device = deviceManager.registerNewDevice(port, devInfo);
                    device->devInfo.hdwRunState = HDW_STATE_APP; // We know this for a fact
                } else {
                    // FIXME: This is a weird state, sometimes the device will return an empty $INFO response.  What should we do with it?
                    //   We know the device is alive, because we got A response, but it wasn't sufficient to actually identify the device.
                }
            }
        }
        break;
    }

    if (m_handlerNmea) {
        m_handlerNmea(this, msg, msgSize, port);
    }
}

#if 0

bool InertialSense::BroadcastBinaryData(port_handle_t port, uint32_t dataId, int periodMultiple)
{
    if (!port)
    {
        return false;
    }

    if (periodMultiple < 0) {
        comManagerDisableData(port, dataId);
    } else {
        auto device = DeviceByPort(port);
        if (device && device->devInfo.protocolVer[0] == PROTOCOL_VERSION_CHAR0) {
            comManagerGetData(port, dataId, 0, 0, periodMultiple);
        }
    }
    return true;
}

bool InertialSense::BroadcastBinaryData(uint32_t dataId, int periodMultiple, pfnHandleBinaryData callback)
{
    if (m_comManagerState.devices.size() == 0)
    {
        return false;
    }

    if (dataId < (sizeof(m_comManagerState.binaryCallback)/sizeof(pfnHandleBinaryData)))
    {
        m_comManagerState.binaryCallback[dataId] = callback;
    }

    for (auto device : m_comManagerState.devices) { device->BroadcastBinaryData(dataId, periodMultiple); }
    return true;
}

void InertialSense::BroadcastBinaryDataRmcPreset(uint64_t rmcPreset, uint32_t rmcOptions)
{
    for (auto device : m_comManagerState.devices) { device->BroadcastBinaryDataRmcPreset(rmcPreset, rmcOptions); }
}

#endif

is_operation_result InertialSense::updateFirmware(fwUpdate::target_t targetDevice, std::vector<std::string> cmds, fwUpdate::pfnStatusCb fwUpdateStatus, void (*waitAction)())
{
    // At some point during an upgrade, we'll likely reset the device and will need to watch for the device to come back. But the cltool normally doesn't discovery
    // new devices (only new ports). So, let's use the PortManagers::port_listener mechanism, during Firmware Updates, to detect when new ports are discovered. When
    // new ports are found, we'll attempt to discover a device on only those specific ports. We MUST keep the handle to the listener, so we can release it when we're
    // done, otherwise this could get called even after the function is out of scope, which would be BAD. Don't forget to release it at the bottom!

    // NOTE: its possible that the device may enumerate its port in the OS before the device is ready to respond to queries (though not likely). As a result, it's
    // possible that if the discoverDevice()'s timeout parameter is too low, we might miss the device - but too long, and its will block other pending ports/events.
    // We might consider a mechanism that records the new ports, and then continues to check them outside of the listener event.
    auto plHandle = portManager.addPortListener(
            [&](PortManager::port_event_e event, uint16_t portType, std::string portName, port_handle_t port, PortFactory& portFactory) {
                log_info(IS_LOG_PORT_MANAGER, "Detected port change (%s) during Firmware Update: %s", event == PortManager::PORT_ADDED ? "Add" : "Remove", portName.c_str());
                if (event == PortManager::PORT_ADDED) {
                    deviceManager.discoverDevice(port, IS_HARDWARE_ANY, 1500, DeviceManager::DISCOVERY__CLOSE_PORT_ON_FAILURE | DeviceManager::DISCOVERY__FORCE_REVALIDATION);
                }
            }
    );

    for (auto device : deviceManager) {
        device->updateFirmware(targetDevice, cmds, fwUpdateStatus, waitAction);
    }

    // portManager.removePortListener(plHandle);


#if !PLATFORM_IS_WINDOWS
    fputs("\e[?25h", stdout);    // Turn cursor back on
#endif

    return IS_OP_OK;
}

/**
 * @return true if ALL connected devices have finished ALL firmware updates (V2) (no pending commands)
 */
bool InertialSense::isFirmwareUpdateFinished() {
    for (auto device : deviceManager) {
        if (device->fwUpdateInProgress())
            return false;
    }
    return true;
}

/**
 * @return false if ANY connected devices returned an error from ANY firmware update; but you should first call isFirmwareUpdateFinished()
 */
bool InertialSense::isFirmwareUpdateSuccessful() {
    for (auto device : deviceManager) {
        ISFirmwareUpdater *fwUpdater = device->fwUpdater;
        if (fwUpdater && fwUpdater->fwUpdate_isDone() && ((fwUpdater->getUploadStatus() < fwUpdate::NOT_STARTED) || fwUpdater->hasErrors()))
            return false;
    }
    return true;
}


int InertialSense::getFirmwareUpdatePercent() {
    float total_percent = 0.0;
    int total_devices = 0;

    for (auto device : deviceManager) {
        if (device->fwUpdateInProgress()) {
            total_percent += device->fwUpdatePercentCompleted();
            total_devices++;
        }
    }

    if (total_devices)
        return (int)((float)total_percent / (float)total_devices);

    return 100;
}

#if !PLATFORM_IS_EMBEDDED
[[ deprecated("This function is deprecated. It will be removed for the 3.0 SDK release.") ]]
is_operation_result InertialSense::BootloadFile(
        const string& comPort,
        const uint32_t serialNum,
        const string& fileName,
        const string& blFileName,
        bool forceBootloaderUpdate,
        int baudRate,
        fwUpdate::pfnProgressCb uploadProgress,
        fwUpdate::pfnProgressCb verifyProgress,
        fwUpdate::pfnStatusCb infoProgress,
        void (*waitAction)()
)
{
    vector<string> comPorts;

    if (comPort == "*")
    {
        cISSerialPort::GetComPorts(comPorts);
    }
    else
    {
        splitString(comPort, ',', comPorts);
    }
    std::sort(comPorts.begin(), comPorts.end());

    vector<string> all_ports;                   // List of ports connected
    vector<string> update_ports;
    vector<string> ports_user_ignore;           // List of ports that were connected at startup but not selected. Will ignore in update.

    cISSerialPort::GetComPorts(all_ports);

    // On non-Windows systems, try to interpret each user-specified port as a symlink and find what it is pointing to
    // TODO: This only works for "/dev/" ports
#if !PLATFORM_IS_WINDOWS
    for (unsigned int k = 0; k < comPorts.size(); k++)
    {
        char buf[PATH_MAX];
        int newsize = readlink(comPorts[k].c_str(), buf, sizeof(buf)-1);
        if (newsize < 0)
        {
            continue;
        }

        buf[newsize] = '\0';
        comPorts[k] = "/dev/" + string(buf);
    }
#endif

    // Get the list of ports to ignore during the bootloading process
    std::sort(all_ports.begin(), all_ports.end());
    std::sort(comPorts.begin(), comPorts.end());
    set_difference(
            all_ports.begin(), all_ports.end(),
            comPorts.begin(), comPorts.end(),
            back_inserter(ports_user_ignore));

    // file exists?
    ifstream tmpStream(fileName);
    if (!tmpStream.good())
    {
        printf("File does not exist: %s\n", fileName.c_str());
        return IS_OP_ERROR;
    }

    ISBootloader::firmwares_t files;
    files.fw_uINS_3.path = fileName;
    files.bl_uINS_3.path = blFileName;
    files.fw_IMX_5.path = fileName;
    files.bl_IMX_5.path = blFileName;
    files.fw_EVB_2.path = fileName;
    files.bl_EVB_2.path = blFileName;

    std::vector<cISBootloaderThread::confirm_bootload_t> confirm_device_list;
    if (!cISBootloaderThread::set_mode_and_check_devices(comPorts, baudRate, files, uploadProgress, verifyProgress, infoProgress, waitAction, &confirm_device_list))
		return IS_OP_ERROR;   // Error or no devices found

    cISSerialPort::GetComPorts(all_ports);

    // Get the list of ports to ignore during the bootloading process
    std::sort(all_ports.begin(), all_ports.end());
    std::sort(ports_user_ignore.begin(), ports_user_ignore.end());
    set_difference(
            all_ports.begin(), all_ports.end(),
            ports_user_ignore.begin(), ports_user_ignore.end(),
            back_inserter(update_ports));

    cISBootloaderThread::update(update_ports, forceBootloaderUpdate, baudRate, files, uploadProgress, verifyProgress, infoProgress, waitAction);

    printf("\n\r");

    #if !PLATFORM_IS_WINDOWS
    fputs("\e[?25h", stdout);    // Turn cursor back on
    #endif

    return IS_OP_OK;
}
#endif

int InertialSense::OnPortError(port_handle_t port, int errCode, const char *errMsg) {
    printf("%s\n", errMsg);
    return 0;
}

bool InertialSense::OpenPorts(const char* portPattern, int baudRate)
{
    m_baudRate = baudRate;

    ClosePorts();

    SerialPortFactory::getInstance().setBaudRate(m_baudRate);

    if (portPattern == NULLPTR || validateBaudRate(baudRate) != 0)
    {
        return false;
    }

    // split port on comma in case we need to open multiple serial ports
    vector<string> portNames;
    size_t maxCount = UINT32_MAX;

    // Note that the following callbacks/handlers will be updated to be ISDevice specific, once the port is deemed to be an ISDevice
    log_info(IS_LOG_FACILITY_NONE, "Initializing comManager...");
    comManagerInit((std::set<port_handle_t>*)&portManager, 10, staticProcessRxData, 0, 0, 0, 0);
    comManagerRegisterProtocolHandler(_PTYPE_NMEA, staticProcessRxNmea);
    comManagerRegisterProtocolHandler(_PTYPE_UBLOX, m_handlerUblox);
    comManagerRegisterProtocolHandler(_PTYPE_RTCM3, m_handlerRtcm3);
    comManagerRegisterProtocolHandler(_PTYPE_SPARTN, m_handlerSpartn);
    comManagerRegisterProtocolHandler(_PTYPE_SEPTENTRIO_SBF, m_handlerSeptSbf);
    comManagerRegisterProtocolHandler(_PTYPE_SEPTENTRIO_REPLY, m_handlerSeptReply);

    // TODO: This should all be handled by the PortManager & PortFactory
    // handle wildcard, auto-detect serial ports
    if (portPattern[0] == '*')
    {
        // m_enableDeviceValidation = true; // always use device-validation when given the 'all ports' wildcard.    (WHJ) I commented this out.  We don't want to force device verification with the loopback tests.
        log_info(IS_LOG_FACILITY_NONE, "Querying OS for available serial ports.");
        portManager.discoverPorts();
        if (portPattern[1] != '\0')
        {
            maxCount = atoi(portPattern + 1);
            maxCount = (maxCount == 0 ? UINT32_MAX : maxCount);
        }
    }
    else
    {
        // comma separated list of serial ports
        if (splitString(portPattern, ',', portNames) > 0) {
            for (auto p: portNames) {
                portManager.discoverPorts(p);
            }
        }
    }

    if (m_enableDeviceValidation) {
        log_info(IS_LOG_FACILITY_NONE, "Starting device validation on %lu registered ports.", portManager.size());

        // we'll make a copy of all the port handles (into a set); as we validate each, we'll remove it from this new set until they are all gone
        for (auto port : portManager.locked_range()) portsToValidate.insert(port);

        // attempt to discover devices on all known ports
        deviceManager.discoverDevices(IS_HARDWARE_ANY, m_comManagerState.discoveryTimeout, DeviceManager::DISCOVERY__CLOSE_PORT_ON_FAILURE);  // In this case, We ABSOLUTELY want to open any closes ports (because they are all closed currently)

        // remove all ports from portToValidate if a device has bound to that port
        for ( auto d : deviceManager ) portsToValidate.erase(d->port);

        log_info(IS_LOG_FACILITY_NONE, "Completed device validation for %lu devices, on %lu ports.", deviceManager.size(), portManager.size());
        if (!portsToValidate.empty()) {
            std::string names;
            for (auto port : portsToValidate) {
                if (!names.empty()) names += ", ";
                names += std::string(portName(port));
            }
            // auto names = utils::join_to_string<std::set<std::string>>(portsToValidate, ", ");
            log_error(IS_LOG_FACILITY_NONE, "Timeout waiting to validate %lu ports: %s.", portsToValidate.size(), names.c_str());
        }
    }

    // request extended device info for remaining connected devices...
    for (auto device : deviceManager) {
        // but only if they are of a compatible protocol version
        if (device->hasDeviceInfo()) {
            device->GetData(DID_SYS_PARAMS);
            device->GetData(DID_FLASH_CONFIG);
            device->GetData(DID_GPX_FLASH_CFG);
            device->GetData(DID_GPX_STATUS);
        }
        device->WaitForImxFlashCfgSynced();
    }


    return (m_enableDeviceValidation ? !deviceManager.empty() : !portManager.empty());
}

void InertialSense::ClosePorts(bool drainBeforeClose)
{
    // TODO: we should find the associated port in the m_comManagerState.devices, and remove the port reference
    // TODO: we need to provide a notification mechanism to inform consumers (ie, test_common framework) to clean up as well.
    // TODO: we probably need to make sure all other references to the port are clear and then destroy the underlying port instance

    // Note the distinction here; we are closing ports, not devices...  Maybe we should do this differently though?
    for (auto port : portManager.locked_range()) {
        if (port) {
            if (drainBeforeClose) {
                portDrain(port, 0);
            }
            portClose(port);
        }
    }
}

/**
 * Handles port management for the InertialSense class. Specifically, defaults to opening newly discovered ports. This function can be
 * overridden to provide additional functionality is extending classes.
 * @param event the type of event for the specified port, typically either PORT_ADDED or PORT_REMOVED
 * @param pType the type of port that this event is associated with - together with the pName, this *should* uniquely identify the port
 * @param pName the name of the port this event is associated with - together with the pType, this *should* uniquely identify the port
 * @param port the port handle that is associated with this event - this maybe null if the port is being removed since this handler is called
 *   after the port has already been identified as having been removed.
 */
void InertialSense::portManagerHandler(uint8_t event, uint16_t pType, std::string pName, port_handle_t port, PortFactory& portFactory) {
    switch ((PortManager::port_event_e)event) {
        case PortManager::PORT_ADDED:
            log_debug(IS_LOG_PORT_MANAGER, "PortManager::PORT_ADDED '%s'", pName.c_str());
            if (!portIsOpened(port)) {
                // debug_message(IS_LOG_FACILITY_NONE, "Opening serial port '%s'", curPortName.c_str());

                // FIXME: the portManagerHandler shouldn't ever open/close the port - it can, but it shouldn't.
                //  This is because the portManager should not ever "do anything" with the port, other than
                //  manage it for others.  So, for device discovery, etc. those services should operate on the
                //  port, but only when they need to do so.  Yes, its "nice" that this could validate that a
                //  port can be opened, if a port is busy elsewhere, that doesn't make it invalid.  Besides,
                //  its the PortFactory's job to validate the port, not the listener... usually.
/*
                if (portOpen(port) == PORT_ERROR__OPEN_FAILURE) {
                    log_debug(IS_LOG_FACILITY_NONE, "Error opening serial port '%s'.  Ignoring.  Error was: %s", pName.c_str(), SERIAL_PORT(port)->error);
                    portClose(port);           // failed to open
                    m_ignoredPorts.push_back(pName);     // record this port name as bad, so we don't try and reopen it again
                }
*/
            }
            break;
        case PortManager::PORT_REMOVED:
            log_debug(IS_LOG_FACILITY_NONE, "PortManager::PORT_REMOVED '%s'.", pName.c_str());
            break;
    }
}

/**
 * Handles device management for the InertialSense class. Specifically, this is called when the device is successfully
 * validated and the port is assigned.  This is used with the portManagerHandler() to complete the device validation phase
 * @param event the type of event for the associated device, typically either DEVICE_ADDED or DEVICE_REMOVED
 * @param device an ISDevice pointer to the associated device.
 */
void InertialSense::deviceManagerHandler(uint8_t event, device_handle_t device) {
    switch ((DeviceManager::device_event_e)event) {
        case DeviceManager::DEVICE_ADDED: {
            log_debug(IS_LOG_FACILITY_NONE, "Device %s added on port %s", device->getIdAsString().c_str(), portIsValid(device->port) ? portName(device->port) : "(None)");

            // since we've validated, we can remove this from the "portsToValidate" set
            auto removeMe = portsToValidate.find(device->port);
            if (removeMe != portsToValidate.end()) {
                log_debug(IS_LOG_FACILITY_NONE, "Removed %s from portsToValidate", portIsValid(device->port) ? portName(device->port) : "(None)");
                portsToValidate.erase(removeMe);
            }
        }
            break;
        case DeviceManager::DEVICE_PORT_BOUND:
            break;
        case DeviceManager::DEVICE_CONNECTED:
            break;
        case DeviceManager::DEVICE_INFO_CHANGED:
            break;
        case DeviceManager::DEVICE_DISCONNECTED:
            break;
        case DeviceManager::DEVICE_PORT_LOST:
            break;
        case DeviceManager::DEVICE_REMOVED:
            log_debug(IS_LOG_FACILITY_NONE, "Device %s removed", device->getIdAsString().c_str());
            break;
    }
}

void InertialSense::QueryDeviceInfo()
{
    for (auto device : DeviceManager::getInstance()) { device->QueryDeviceInfo(); }
}

void InertialSense::StopBroadcasts(bool allPorts)
{
    for (auto device : DeviceManager::getInstance()) { device->StopBroadcasts(allPorts); }
}

void InertialSense::SavePersistent()
{
    for (auto device : DeviceManager::getInstance()) { device->SavePersistent(); }
}

void InertialSense::SoftwareReset()
{
    for (auto device : DeviceManager::getInstance()) { device->softwareReset(); }
}

void InertialSense::GetData(eDataIDs dataId, uint16_t length, uint16_t offset, uint16_t period)
{
    for (auto device : DeviceManager::getInstance()) { device->GetData(dataId, length, offset, period); }
}

void InertialSense::SendData(eDataIDs dataId, void* data, uint32_t length, uint32_t offset)
{
    for (auto device : DeviceManager::getInstance()) { device->SendData(dataId, data, length, offset); }
}

void InertialSense::SendRaw(void* data, uint32_t length)
{
    for (auto device : DeviceManager::getInstance()) { device->SendRaw(data, length); }
}

void InertialSense::SendNmea(const std::string& nmeaMsg)
{
    for (auto device : DeviceManager::getInstance()) { device->SendNmea(nmeaMsg); }
}

void InertialSense::Send(uint8_t pktInfo, void *data, uint16_t did, uint16_t size, uint16_t offset)
{
    for (auto device : DeviceManager::getInstance()) { device->Send(pktInfo, data, did, size, offset); }
}

void InertialSense::BroadcastBinaryData(uint32_t dataId, int periodMultiple)
{
    for (auto device : DeviceManager::getInstance()) { device->BroadcastBinaryData(dataId, periodMultiple); }
}

void InertialSense::BroadcastBinaryDataRmcPreset(uint64_t rmcPreset, uint32_t rmcOptions)
{
    for (auto device : DeviceManager::getInstance()) { device->BroadcastBinaryDataRmcPreset(rmcPreset, rmcOptions); }
}

void InertialSense::SetSysCmd(const uint32_t command, port_handle_t port)
{
    if (port == nullptr) {   // Send to all
        for (auto device : deviceManager) { device->SetSysCmd(command); }
    } else {                 // Send to specific port
        auto device = getDevice(port);
        if (device) device->SetSysCmd(command);
    }
}

/**
* Get current device system command
* @param port the port to get sysCmd for
* @return current device system command
*/
system_command_t InertialSense::GetSysCmd(port_handle_t port)
{
    auto device = getDevice(port);
    if (device)
        return device->sysCmd;

    return { 0, 0 };    // nothing...
}

/**
 * Sends message to device to set devices Event Filter
 * param Target: 0 = device,
 *               1 = forward to device GNSS 1 port (ie GPX),
 *               2 = forward to device GNSS 2 port (ie GPX),
 *               else will return
 *       port: Send in target COM port.
 *                If arg is < 0 default port will be used
*/
void InertialSense::SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel, port_handle_t port)
{
    #define EVENT_MAX_SIZE (1024 + DID_EVENT_HEADER_SIZE)
    uint8_t data[EVENT_MAX_SIZE] = {0};

    did_event_t event = {};
    event.time = 123,
    event.senderSN = 0,
    event.senderHdwId = 0,
    event.length = sizeof(did_event_filter_t);

    did_event_filter_t filter = {};
    filter.portMask = portMask;
    filter.eventMask.priorityLevel = priorityLevel;
    filter.eventMask.msgTypeIdMask = msgTypeIdMask;

    if (target == 0)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_FILTER;
    else if (target == 1)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_GNSS1_FILTER;
    else if (target == 2)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_GNSS2_FILTER;
    else
        return;

    memcpy(data, &event, DID_EVENT_HEADER_SIZE);
    memcpy((void*)(data+DID_EVENT_HEADER_SIZE), &filter, _MIN(sizeof(did_event_filter_t), EVENT_MAX_SIZE-DID_EVENT_HEADER_SIZE));

    if (!port) {
        SendData(DID_EVENT, data, DID_EVENT_HEADER_SIZE + event.length, 0);
    } else {
        auto device = deviceManager.getDevice(port);
        if (device) device->SendData(DID_EVENT, data, DID_EVENT_HEADER_SIZE + event.length, 0);
    }
}
