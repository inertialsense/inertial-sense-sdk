/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <algorithm>
#include <vector>

#include "protocol_nmea.h"
#include "yaml-cpp/yaml.h"
#include "protocol_nmea.h"
#include "InertialSense.h"
#include "ISDevice.h"
#include "ISBootloaderThread.h"
#include "ISBootloaderDFU.h"
#include "protocol/FirmwareUpdate.h"
#include "imx_defaults.h"

using namespace std;

#define PRINT_DEBUG 0
#if PRINT_DEBUG
#define DEBUG_PRINT(...)    printf("L%d: ", __LINE__); printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) 
#endif

static InertialSense *s_is = NULL;
static InertialSense::com_manager_cpp_state_t *s_cm_state = NULL;

static int staticSendData(port_handle_t port, const uint8_t* buf, int len)
{
    return serialPortWrite(port, buf, len);
}

static int staticReadData(port_handle_t port, uint8_t* buf, int len)
{
    int bytesRead = serialPortReadTimeout(port, buf, len, 1);

    if (s_is) {   // Save raw data to ISlogger
        s_is->LogRawData(s_is->DeviceByPort(port), bytesRead, buf);
    }
    return bytesRead;
}

static int staticProcessRxData(p_data_t* data, port_handle_t port)
{
    if ((port == NULLPTR) || (data->hdr.id >= (sizeof(s_cm_state->binaryCallback)/sizeof(pfnHandleBinaryData))))
    {
        return -1;
    }

    pfnHandleBinaryData handler = s_cm_state->binaryCallback[data->hdr.id];
    s_cm_state->stepLogFunction(s_cm_state->inertialSenseInterface, data, port);

    if (handler != NULLPTR)
    {
        handler(s_cm_state->inertialSenseInterface, data, port);
    }

    pfnHandleBinaryData handlerGlobal = s_cm_state->binaryCallbackGlobal;
    if (handlerGlobal != NULLPTR)
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

static int staticProcessRxNmea(const unsigned char* msg, int msgSize, port_handle_t port)
{
    if (port)
    {
        s_cm_state->inertialSenseInterface->ProcessRxNmea(port, msg, msgSize);
    }
    return 0;
}

InertialSense::InertialSense(
        pfnHandleBinaryData     handlerIsb,
        pfnComManagerRmcHandler handlerRmc,
        pfnIsCommGenMsgHandler  handlerNmea,
        pfnIsCommGenMsgHandler  handlerUblox,
        pfnIsCommGenMsgHandler  handlerRtcm3,
        pfnIsCommGenMsgHandler  handlerSpartn,
        pfnOnNewDeviceHandler handlerNewDevice) : m_tcpServer(this)
{
    s_is = this;
    s_cm_state = &m_comManagerState;
    m_logThread = NULLPTR;
    m_lastLogReInit = time(0);
    m_clientStream = NULLPTR;
    m_clientBufferBytesToSend = 0;
    m_clientServerByteCount = 0;
    m_disableBroadcastsOnClose = false;  // For Intellian

    for (int i=0; i<int(sizeof(m_comManagerState.binaryCallback)/sizeof(pfnHandleBinaryData)); i++)
    {
        m_comManagerState.binaryCallback[i] = {};
    }
    m_comManagerState.binaryCallbackGlobal = handlerIsb;
    m_comManagerState.stepLogFunction = &InertialSense::StepLogger;
    m_comManagerState.inertialSenseInterface = this;
    m_comManagerState.clientBuffer = m_clientBuffer;
    m_comManagerState.clientBufferSize = sizeof(m_clientBuffer);
    m_comManagerState.clientBytesToSend = &m_clientBufferBytesToSend;
    comManagerGetGlobal()->assignUserPointer(&m_comManagerState);
    // memset(&m_cmInit, 0, sizeof(m_cmInit));
    // m_cmPorts = NULLPTR;

    is_comm_init(&m_gpComm, m_gpCommBuffer, sizeof(m_gpCommBuffer), NULL);

    m_newDeviceHandler = handlerNewDevice;

    // Rx data callback functions
    m_handlerRmc    = handlerRmc;
    m_handlerNmea   = handlerNmea;
    m_handlerUblox  = handlerUblox;
    m_handlerRtcm3  = handlerRtcm3;
    m_handlerSpartn = handlerSpartn;
}

InertialSense::~InertialSense()
{
    Close();
    CloseServerConnection();
    DisableLogging();
}

bool InertialSense::EnableLogging(const string& path, const cISLogger::sSaveOptions& options)
{
    cMutexLocker logMutexLocker(&m_logMutex);

    if (!m_logger.InitSave(path, options))
    {
        return false;
    }
    m_logger.EnableLogging(true);
    for (auto d : m_comManagerState.devices)
    {
        m_logger.registerDevice(d);
    }
    if (m_logThread == NULLPTR)
    {
        m_logThread = threadCreateAndStart(&InertialSense::LoggerThread, this);
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

void InertialSense::LogRawData(ISDevice* device, int dataSize, const uint8_t* data)
{
    if (device && device->devLogger)
        m_logger.LogData(device->devLogger, dataSize, data);
}

bool InertialSense::registerDevice(ISDevice* device) {
    if (!device)
        return NULL;

    // first, ensure there isn't a matching device already
    for (auto d : m_comManagerState.devices) {
        if (d == device)
            return true;
        if ((d->hdwId == ENCODE_DEV_INFO_TO_HDW_ID(device->devInfo)) && (d->devInfo.serialNumber == device->devInfo.serialNumber))
            return true;
    }

    m_comManagerState.devices.push_back(device);
    return true;
}

ISDevice* InertialSense::registerNewDevice(port_handle_t port, dev_info_t devInfo) {
    if (!port)
        return NULL;

    // first, ensure there isn't a matching device already
    for (auto d : m_comManagerState.devices) {
        if ((d->hdwId == ENCODE_DEV_INFO_TO_HDW_ID(devInfo)) && (d->devInfo.serialNumber == devInfo.serialNumber)) {
            d->port = port;
            return d;
        }
    }

    // If we're here, we didn't find the device above
    ISDevice* newDevice = (m_newDeviceHandler ? m_newDeviceHandler(port) : new ISDevice(port));
    newDevice->port = port;
    newDevice->devInfo = devInfo;
    newDevice->hdwId = ENCODE_DEV_INFO_TO_HDW_ID(devInfo);
    newDevice->hdwRunState = ISDevice::HDW_STATE_APP; // this is probably a safe assumption, assuming we have dev good info
    newDevice->flashCfgUploadChecksum = 0;
    newDevice->sysParams.flashCfgChecksum = 0;
    m_comManagerState.devices.push_back(newDevice);
    return m_comManagerState.devices.empty() ? NULL : (ISDevice*)m_comManagerState.devices.back();
}

bool InertialSense::releaseDevice(ISDevice* device, bool closePort)
{
    auto deviceIter = std::find(m_comManagerState.devices.begin(), m_comManagerState.devices.end(), device);
    if (deviceIter == m_comManagerState.devices.end())
        return false;

    if (closePort && device->port) {
        serialPortClose(device->port);
        freeSerialPort(device->port, false);
    }

    m_comManagerState.devices.erase(deviceIter); // erase only remove the ISDevice* from the list, but doesn't release/free the instance itself
    device->port = NULL;
    delete device;

    return true;
}

bool InertialSense::HasReceivedDeviceInfo(ISDevice* device)
{
    return (device && (device->hdwId != 0) && (device->hdwRunState != ISDevice::HDW_STATE_UNKNOWN) && (device->devInfo.serialNumber != 0) && (device->devInfo.hardwareType != 0));
}

bool InertialSense::HasReceivedDeviceInfoFromAllDevices()
{
    if (m_comManagerState.devices.empty())
        return false;

    for (auto device : m_comManagerState.devices)
    {
        if (!HasReceivedDeviceInfo(device))
        {
            return false;
        }
    }

    return true;
}

void InertialSense::RemoveDevice(ISDevice* device)
{
    for (auto cmsDevice : m_comManagerState.devices) {
        if (cmsDevice == device) {
            // m_serialPorts.erase(m_serialPorts.begin() + i);
            if (device->port) {
                serialPortClose(device->port);
                comManagerRemovePort(device->port);
                //delete (serial_port_t *) device->port;
                //device->port = NULL;
            }
        }
    }
    std::remove_if (m_comManagerState.devices.begin(), m_comManagerState.devices.end(), [&](const ISDevice* d){
        return d == device;
    });
    // TODO: remove the device from m_comManagerState
    //   -- we don't really need to remove it, but we should
    //   -- we could end up with the same device listed more than once, with different ports if we don't, though only the most recent should have an active/open port
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
                        ISDevice* device = inertialSense->DeviceByPort(port);
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

void InertialSense::StepLogger(InertialSense* i, const p_data_t* data, port_handle_t port)
{
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

// [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
bool InertialSense::OpenConnectionToServer(const string& connectionString)
{
    CloseServerConnection();

    // calls new cISTcpClient or new cISSerialPort
    m_clientStream = cISClient::OpenConnectionToServer(connectionString, &m_forwardGpgga);

    return m_clientStream!=NULLPTR;
}

void InertialSense::CloseServerConnection()
{
    m_tcpServer.Close();
    // m_serialServer.Close();

    if (m_clientStream != NULLPTR)
    {
        delete m_clientStream;
        m_clientStream = NULLPTR;
    }
}

// [type]:[ip/url]:[port]
bool InertialSense::CreateHost(const string& connectionString)
{
    // if no serial connection, fail
    if (!IsOpen())
    {
        return false;
    }

    CloseServerConnection();

    vector<string> pieces;
    splitString(connectionString, ':', pieces);
    if (pieces.size() < 3)
    {
        return false;
    }

    string type     = pieces[0];    // TCP, SERIAL
    string host     = pieces[1];    // IP / URL
    string port     = pieces[2];

    if (type != "TCP")
    {
        return false;
    }

    StopBroadcasts();

    return (m_tcpServer.Open(host, atoi(port.c_str())) == 0);
}

size_t InertialSense::DeviceCount()
{
    return m_comManagerState.devices.size();
}

/**
 * @return a vector of available ports
 * NOTE that this may return ports which do not have a corresponding ISDevice
 */
std::vector<port_handle_t> InertialSense::getPorts() {
    std::vector<port_handle_t> ports;
    for (auto port : m_serialPorts) {
        ports.push_back((port_handle_t)port);
    }
    return ports;
}

/**
 * Returns a vector of available, connected devices
 * @return
 */
std::list<ISDevice*>& InertialSense::getDevices() {
    return m_comManagerState.devices;
}

/**
 * Returns a vector of available, connected devices
 * @return
 */
std::vector<ISDevice*> InertialSense::getDevicesAsVector() {
    std::vector<ISDevice*> vecOut;
    for (auto device : m_comManagerState.devices) {
        vecOut.push_back(device);
    }
    return vecOut;
}

/**
 * Returns the ISDevice instance associated with the specified port, or NULL if there is no associated device
 * @param port
 * @return
 */
ISDevice* InertialSense::getDevice(port_handle_t port) {
    for (auto device : m_comManagerState.devices) {
        if (device->port == port)
            return device;
    }

    return NULL;
}

/**
 * Returns the ISDevice instance associated with the specified port, or NULL if there is no associated device
 * @param port
 * @return
 */
ISDevice* InertialSense::getDevice(uint32_t serialNum, is_hardware_t hdwId) {
    for (auto device : m_comManagerState.devices) {
        if ((device->hdwId == hdwId) && (device->devInfo.serialNumber == serialNum))
            return device;
    }

    return NULL;
}


bool InertialSense::Update()
{
    m_timeMs = current_timeMs();

    if (m_tcpServer.IsOpen() && m_comManagerState.devices.size() > 0)
    {
        UpdateServer();
    }
    else
    {
        UpdateClient();

        // [C COMM INSTRUCTION]  2.) Update Com Manager at regular interval to send and receive data.
        // Normally called within a while loop.  Include a thread "sleep" if running on a multi-thread/
        // task system with serial port read function that does NOT incorporate a timeout.
        if (m_comManagerState.devices.size() > 0)
        {
            comManagerStep();
            SyncFlashConfig(m_timeMs);

            // check if we have an valid instance of the FirmareUpdate class, and if so, call it's Step() function
            for (auto device : m_comManagerState.devices) {
                if (serialPortIsOpen(device->port) && device->fwUpdater != nullptr) {
                    if (!device->fwUpdater->fwUpdate_step()) { // device.fwUpdate.update();
                        if (device->fwLastStatus < fwUpdate::NOT_STARTED) {
                            // TODO: Report a REAL error
                            // printf("Error starting firmware update: %s\n", fwUpdater->getSessionStatusName());
                        }

#ifdef DEBUG_CONSOLELOGGING
                    } else if ((fwUpdater->getNextChunkID() != lastChunk) || (status != lastStatus)) {
                        int serialNo = m_comManagerState.devices[devIdx].devInfo.serialNumber;
                        float pcnt = fwUpdater->getTotalChunks() == 0 ? 0.f : ((float)fwUpdater->getNextChunkID() / (float)fwUpdater->getTotalChunks() * 100.f);
                        float errRt = fwUpdater->getResendRate() * 100.f;
                        const char *status = fwUpdater->getSessionStatusName();
                        printf("SN%d :: %s : [%d of %d] %0.1f%% complete (%u, %0.1f%% resend)\n", serialNo, status, fwUpdater->getNextChunkID(), fwUpdater->getTotalChunks(), pcnt, fwUpdater->getResendCount(), errRt);
#endif
                    }
                }
            }
        }
    }

    // if any serial ports have closed, shutdown
    bool anyOpen = false;
    for (auto device : m_comManagerState.devices)
    {
        if (!serialPortIsOpen(device->port))
        {
            // Make sure its closed..
            serialPortClose(device->port);
        } else
            anyOpen = true;
    }

    return anyOpen;
}

bool InertialSense::UpdateServer()
{
    // as a tcp server, only the first serial port is read from
    port_handle_t port = m_comManagerState.devices.front()->port;
    is_comm_instance_t *comm = &(COMM_PORT(port)->comm);
    protocol_type_t ptype = _PTYPE_NONE;

    // Get available size of comm buffer
    int n = is_comm_free(comm);         // TODO:  This is a little janky; as a Serial/COMM port, this should already know how to do these things...

    // Read data directly into comm buffer
    if ((n = serialPortReadTimeout(port, comm->rxBuf.tail, n, 0)))
    {
        // Update comm buffer tail pointer
        comm->rxBuf.tail += n;

        // Search comm buffer for valid packets
        while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
        {
            string str;

            switch (ptype)
            {
                case _PTYPE_RTCM3:
                case _PTYPE_UBLOX:
                    // forward data on to connected clients
                    m_clientServerByteCount += comm->rxPkt.data.size;
                    if (m_tcpServer.Write(comm->rxPkt.data.ptr, comm->rxPkt.data.size) != (int)comm->rxPkt.data.size)
                    {
                        cout << endl << "Failed to write bytes to tcp server!" << endl;
                    }
                    if (ptype == _PTYPE_RTCM3)
                    {
                        if ((comm->rxPkt.id == 1029) && (comm->rxPkt.data.size < 1024))
                        {
                            str = string().assign(reinterpret_cast<char*>(comm->rxPkt.data.ptr + 12), comm->rxPkt.data.size - 12);
                        }
                    }
                    break;

                default:
                    break;
            }

            if (ptype != _PTYPE_NONE)
            {   // Record message info
                messageStatsAppend(str, m_serverMessageStats, ptype, comm->rxPkt.id, m_timeMs);
            }
        }
    }
    m_tcpServer.Update();

    return true;
}

bool InertialSense::UpdateClient()
{
    if (m_clientStream == NULLPTR)
    {
        return false;
    }

    // Forward only valid uBlox and RTCM3 packets
    is_comm_instance_t *comm = &(m_gpComm);
    protocol_type_t ptype = _PTYPE_NONE;
    static int error = 0;

    // Get available size of comm buffer.  is_comm_free() modifies comm->rxBuf pointers, call it before using comm->rxBuf.tail.
    int n = is_comm_free(comm);

    // Read data directly into comm buffer
    if ((n = m_clientStream->Read(comm->rxBuf.tail, n)))
    {
        // Update comm buffer tail pointer
        comm->rxBuf.tail += n;

        // Search comm buffer for valid packets
        while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
        {
            string str;

            switch (ptype)
            {
                case _PTYPE_UBLOX:
                case _PTYPE_RTCM3:
                    m_clientServerByteCount += comm->rxPkt.data.size;
                    OnClientPacketReceived(comm->rxPkt.data.ptr, comm->rxPkt.data.size);

                    if (ptype == _PTYPE_RTCM3)
                    {
                        if ((comm->rxPkt.id == 1029) && (comm->rxPkt.data.size < 1024))
                        {
                            str = string().assign(reinterpret_cast<char*>(comm->rxPkt.data.ptr + 12), comm->rxPkt.data.size - 12);
                        }
                    }
                    break;

                case _PTYPE_PARSE_ERROR:
                    if (error)
                    {   // Don't print first error.  Likely due to port having been closed.
                        printf("InertialSense::UpdateClient() PARSE ERROR count: %d\n", error);
                    }
                    error++;
                    break;

                default:
                    break;
            }

            if (ptype != _PTYPE_NONE)
            {   // Record message info
                messageStatsAppend(str, m_clientMessageStats, ptype, comm->rxPkt.id, m_timeMs);
            }
        }
    }

    // Send data to client if available, i.e. nmea gga pos
    if (m_clientBufferBytesToSend > 0 && m_forwardGpgga)
    {
        m_clientStream->Write(m_clientBuffer, m_clientBufferBytesToSend);
    }
    m_clientBufferBytesToSend = 0;

    return true;
}

bool InertialSense::Open(const char* port, int baudRate, bool disableBroadcastsOnClose)
{
    // null com port, just use other features of the interface like ntrip
    if (port[0] == '0' && port[1] == '\0')
    {
        return true;
    }

    m_disableBroadcastsOnClose = false;
    if (OpenSerialPorts(port, baudRate))
    {
        m_disableBroadcastsOnClose = disableBroadcastsOnClose;
        return true;
    }
    return false;
}

bool InertialSense::IsOpen()
{
    return (m_comManagerState.devices.size() != 0 && serialPortIsOpen(m_comManagerState.devices.front()->port));
}

void InertialSense::Close()
{
    EnableLogger(false);
    if (m_disableBroadcastsOnClose)
    {
        StopBroadcasts();
        SLEEP_MS(100);
    }
//    for (auto& device : m_comManagerState.devices)
//    {
//        serialPortClose(device.port);
//    }
//    m_comManagerState.devices.clear();
//    m_serialPorts.clear();
    CloseSerialPorts(true); // allow all opened ports to transmit all buffered data
}

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

void InertialSense::SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
    for (auto device : m_comManagerState.devices) { device->SendData(dataId, data, length, offset); }
}

void InertialSense::SendRaw(uint8_t* data, uint32_t length)
{
    for (auto device : m_comManagerState.devices) { device->SendRaw(data, length); }
}

void InertialSense::SetSysCmd(const uint32_t command, port_handle_t port)
{
    if (port == nullptr)
    {   // Send to all
        for (auto device : m_comManagerState.devices) { device->SetSysCmd(command); }
    }
    else
    {   // Specific port
        ISDevice* device = DeviceByPort(port);
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

// This method uses DID_SYS_PARAMS.flashCfgChecksum to determine if the local flash config is synchronized.
void InertialSense::SyncFlashConfig(unsigned int timeMs)
{
    if (timeMs - m_syncCheckTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
    {
        return;
    }
    m_syncCheckTimeMs = timeMs;

    for (auto device : m_comManagerState.devices)
    {
        if (device->flashCfgUploadTimeMs)
        {   // Upload in progress
            if (timeMs - device->flashCfgUploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
            {   // Wait for upload to process.  Pause sync.
                device->sysParams.flashCfgChecksum = 0;
            }
        }

        // Require valid sysParams checksum
        if (device->sysParams.flashCfgChecksum)
        {   
            if (device->sysParams.flashCfgChecksum == device->flashCfg.checksum)
            {
                if (device->flashCfgUploadTimeMs)
                {   // Upload complete.  Allow sync.
                    device->flashCfgUploadTimeMs = 0;

                    if (device->flashCfgUploadChecksum == device->sysParams.flashCfgChecksum)
                    {
                        printf("DID_FLASH_CONFIG upload complete.\n");
                    }
                    else
                    {
                        printf("DID_FLASH_CONFIG upload rejected.\n");
                    }
                }
            }
            else
            {   // Out of sync.  Request flash config.
                DEBUG_PRINT("Out of sync.  Requesting DID_FLASH_CONFIG...\n");
                comManagerGetData(device->port, DID_FLASH_CONFIG, 0, 0, 0);
            }
        } 
    }
}

void InertialSense::UpdateFlashConfigChecksum(nvm_flash_cfg_t &flashCfg)
{
    bool platformCfgUpdateIoConfig = flashCfg.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;

    // Exclude from the checksum update the following which does not get saved in the flash config
    flashCfg.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;

    if (platformCfgUpdateIoConfig)
    {   // Update ioConfig
        imxPlatformConfigToFlashCfgIoConfig(&flashCfg.ioConfig, flashCfg.platformConfig);
    }

    // Update checksum
    flashCfg.checksum = flashChecksum32(&flashCfg, sizeof(nvm_flash_cfg_t));
}

bool InertialSense::FlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port)
{
    ISDevice* device = NULL;
    if (!port) {
        device = m_comManagerState.devices.front();
    } else {
        device = DeviceByPort(port);
    }

    if (!device)
        return false;

    // Copy flash config
    flashCfg = device->flashCfg;

    // Indicate whether flash config is synchronized
    return device->sysParams.flashCfgChecksum == device->flashCfg.checksum;
}

bool InertialSense::SetFlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port)
{
    ISDevice* device = NULL;
    if (!port) {
        device = m_comManagerState.devices.front();
    } else {
        device = DeviceByPort(port);
    }

    if (device == NULL)
        return false;   // no device, no setting of flash-config

    device->flashCfg.checksum = flashChecksum32(&device->flashCfg, sizeof(nvm_flash_cfg_t));

    // Iterate over and upload flash config in 4 byte segments.  Upload only contiguous segments of mismatched data starting at `key` (i = 2).  Don't upload size or checksum.
    static_assert(sizeof(nvm_flash_cfg_t) % 4 == 0, "Size of nvm_flash_cfg_t must be a 4 bytes in size!!!");
    uint32_t *newCfg = (uint32_t*)&flashCfg;
    uint32_t *curCfg = (uint32_t*)&(device->flashCfg);
    int iSize = sizeof(nvm_flash_cfg_t) / 4;
    bool failure = false;
    // Exclude updateIoConfig bit from flash config and keep track of it separately so it does not affect whether the platform config gets uploaded
    bool platformCfgUpdateIoConfig = flashCfg.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;
    flashCfg.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;

    for (int i = 2; i < iSize; i++)     // Start with index 2 to exclude size and checksum
    {
        if (newCfg[i] != curCfg[i])
        {   // Found start
            uint8_t *head = (uint8_t*)&(newCfg[i]);

            // Search for end
            for (; i < iSize && newCfg[i] != curCfg[i]; i++);

            // Found end
            uint8_t *tail = (uint8_t*)&(newCfg[i]);
            int size = tail-head;
            int offset = head-((uint8_t*)newCfg);

            if (platformCfgUpdateIoConfig &&
                head <= (uint8_t*)&(flashCfg.platformConfig) &&
                tail >  (uint8_t*)&(flashCfg.platformConfig))
            {   // Re-apply updateIoConfig bit prior to upload
                flashCfg.platformConfig |= PLATFORM_CFG_UPDATE_IO_CONFIG;
            }

            const data_info_t* fieldInfo = cISDataMappings::FieldInfoByOffset(DID_FLASH_CONFIG, offset);
            DEBUG_PRINT("Sending DID_FLASH_CONFIG: size %d, offset %d (%s)\n", size, offset, (fieldInfo ? fieldInfo->name.c_str() : "<UNKNOWN>"));
            int fail = comManagerSendData(device->port, head, DID_FLASH_CONFIG, size, offset);
            failure = failure || fail;
            device->flashCfgUploadTimeMs = current_timeMs();                // non-zero indicates upload in progress
        }
    }

    if (device->flashCfgUploadTimeMs == 0)
    {
        printf("DID_FLASH_CONFIG in sync.  No upload.\n");
    }

    // Update checksum
    UpdateFlashConfigChecksum(flashCfg);

    // Save checksum to ensure upload happened correctly
    if (device->flashCfgUploadTimeMs)
    {
        device->flashCfgUploadChecksum = flashCfg.checksum;
    }

    // Update local copy of flash config
    device->flashCfg = flashCfg;

    // Success
    return !failure;
}

bool InertialSense::WaitForFlashSynced(port_handle_t port)
{
    ISDevice* device = NULL;
    if (!port) {
        device = m_comManagerState.devices.front();
    } else {
        device = DeviceByPort(port);
    }

    if (!device)
        return false;   // No device, no flash-sync

    unsigned int startMs = current_timeMs();
    while (!FlashConfigSynced(device->port))
    {   // Request and wait for flash config
        Update();

        if ((current_timeMs() - startMs) > SYNC_FLASH_CFG_TIMEOUT_MS)
        {   // Timeout waiting for flash config
            printf("Timeout waiting for DID_FLASH_CONFIG failure!\n");

            #if PRINT_DEBUG
            ISDevice& device = m_comManagerState.devices[pHandle];
            DEBUG_PRINT("device.flashCfg.checksum:          %u\n", device.flashCfg.checksum);
            DEBUG_PRINT("device.sysParams.flashCfgChecksum: %u\n", device.sysParams.flashCfgChecksum); 
            DEBUG_PRINT("device.flashCfgUploadTimeMs:       %u\n", device.flashCfgUploadTimeMs);
            DEBUG_PRINT("device.flashCfgUploadChecksum:     %u\n", device.flashCfgUploadChecksum);
            #endif
            return false;
        }
        else
        {   // Query DID_SYS_PARAMS
            GetData(DID_SYS_PARAMS);
            DEBUG_PRINT("Waiting for flash sync...\n");
        }
        SLEEP_MS(20);  // give some time for the device to respond.
    }

    return FlashConfigSynced(device->port);
}

void InertialSense::ProcessRxData(port_handle_t port, p_data_t* data)
{
    if (data->hdr.size==0 || data->ptr==NULL)
    {
        return;
    }

    ISDevice* device = DeviceByPort(port);
    if (!device) {
        if (data->hdr.id != DID_DEV_INFO)
            return;
    }

    switch (data->hdr.id)
    {
        case DID_DEV_INFO:
            if (!device) {
                device = registerNewDevice(port, *(dev_info_t*)data->ptr);
                device->hdwRunState = ISDevice::HDW_STATE_APP; // we know this for a fact
            } else {
                device->devInfo = *(dev_info_t*)data->ptr;
            }
            break;
        case DID_SYS_CMD:
            device->sysCmd = *(system_command_t*)data->ptr;
            break;
        case DID_SYS_PARAMS:
            copyDataPToStructP(&(device->sysParams), data, sizeof(sys_params_t));
            DEBUG_PRINT("Received DID_SYS_PARAMS\n");
            break;
        case DID_FLASH_CONFIG:
            copyDataPToStructP(&(device->flashCfg), data, sizeof(nvm_flash_cfg_t));
            if (dataOverlap(offsetof(nvm_flash_cfg_t, checksum), 4, data))
            {   // Checksum received
                device->sysParams.flashCfgChecksum = device->flashCfg.checksum;
            }
            DEBUG_PRINT("Received DID_FLASH_CONFIG\n");
            break;
        case DID_FIRMWARE_UPDATE:
            // we don't respond to messages if we don't already have an active Updater
            if (device->fwUpdater) {
                device->fwUpdater->fwUpdate_processMessage(data->ptr, data->hdr.size);
                device->fwUpdate();
            }
            break;
    }
}

// return 0 on success, -1 on failure
void InertialSense::ProcessRxNmea(port_handle_t port, const uint8_t* msg, int msgSize)
{
    switch (getNmeaMsgId(msg, msgSize)) {
        case NMEA_MSG_ID_INFO: {
            // Device Info
            ISDevice* device = DeviceByPort(port);
            if (!device) {
                dev_info_t devInfo = {};
                if (!nmea_parse_info(devInfo, (const char *) msg, msgSize)) {
                    device = registerNewDevice(port, devInfo);
                    device->hdwRunState = ISDevice::HDW_STATE_APP; // We know this for a fact
                } else {
                    // FIXME: This is a weird state, sometimes the device will return an empty $INFO response.  What should we do with it?
                    //   We know the device is alive, because we got A response, but it wasn't sufficient to actually identify the device.
                }
            }
        }
        break;
    }

    if (m_handlerNmea) {
        m_handlerNmea(msg, msgSize, port);
    }
}

bool InertialSense::BroadcastBinaryData(port_handle_t port, uint32_t dataId, int periodMultiple)
{
    if (!port)
    {
        return false;
    }

    if (periodMultiple < 0) {
        comManagerDisableData(port, dataId);
    } else {
        ISDevice *device = DeviceByPort(port);
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

is_operation_result InertialSense::updateFirmware(
        const string& comPort,
        int baudRate,
        fwUpdate::target_t targetDevice,
        std::vector<std::string> cmds,
        ISBootloader::pfnBootloadProgress uploadProgress,
        ISBootloader::pfnBootloadProgress verifyProgress,
        ISBootloader::pfnBootloadStatus infoProgress,
        void (*waitAction)()
)
{
    EnableDeviceValidation(true);
    if (OpenSerialPorts(comPort.c_str(), baudRate)) {
        for (auto device : m_comManagerState.devices) {
            device->fwUpdater = new ISFirmwareUpdater(device->port, &device->devInfo);
            device->fwUpdater->setTarget(targetDevice);

            // TODO: Implement maybe
            device->fwUpdater->setUploadProgressCb(uploadProgress);
            device->fwUpdater->setVerifyProgressCb(verifyProgress);
            device->fwUpdater->setInfoProgressCb(infoProgress);

            device->fwUpdater->setCommands(cmds);
        }
    }

    printf("\n\r");

#if !PLATFORM_IS_WINDOWS
    fputs("\e[?25h", stdout);    // Turn cursor back on
#endif

    return IS_OP_OK;
}

is_operation_result InertialSense::updateFirmware(
        ISDevice* device,
        fwUpdate::target_t targetDevice,
        std::vector<std::string> cmds,
        ISBootloader::pfnBootloadProgress uploadProgress,
        ISBootloader::pfnBootloadProgress verifyProgress,
        ISBootloader::pfnBootloadStatus infoProgress,
        void (*waitAction)()
)
{
    EnableDeviceValidation(true);
    device->fwUpdater = new ISFirmwareUpdater(*device);
    device->fwUpdater->setTarget(targetDevice);

    // TODO: Implement maybe
    device->fwUpdater->setUploadProgressCb(uploadProgress);
    device->fwUpdater->setVerifyProgressCb(verifyProgress);
    device->fwUpdater->setInfoProgressCb(infoProgress);

    device->fwUpdater->setCommands(cmds);

    printf("\n\r");

#if !PLATFORM_IS_WINDOWS
    fputs("\e[?25h", stdout);    // Turn cursor back on
#endif

    return IS_OP_OK;
}

/**
 * @return true if ALL connected devices have finished ALL firmware updates (V2) (no pending commands)
 */
bool InertialSense::isFirmwareUpdateFinished() {
    for (auto device : m_comManagerState.devices) {
        if (device->fwUpdateInProgress())
            return false;
    }
    return true;
}

/**
 * @return false if ANY connected devices returned an error from ANY firmware update; but you should first call isFirmwareUpdateFinished()
 */
bool InertialSense::isFirmwareUpdateSuccessful() {
    for (auto device : m_comManagerState.devices) {
        ISFirmwareUpdater *fwUpdater = device->fwUpdater;
        if (fwUpdater->hasErrors() ||
            ((fwUpdater != nullptr) &&
                fwUpdater->fwUpdate_isDone() &&
                (
                        (fwUpdater->fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED) ||
                        fwUpdater->hasErrors()
              )
          ))
        return false;
    }
    return true;
}


int InertialSense::getFirmwareUpdatePercent() {
    float total_percent = 0.0;
    int total_devices = 0;

    for (auto device : m_comManagerState.devices) {
        if (device->fwUpdateInProgress()) {
            total_percent += device->fwPercent;
            total_devices++;
        }
    }

    if (total_devices)
        return (int)((float)total_percent / (float)total_devices);

    return 100;
}

/**
* Gets device index from COM port
* @param COM port
*/
[[ deprecated ]]
int InertialSense::getUpdateDeviceIndex(const char* com)
{
    for (auto device : m_comManagerState.devices)
    {
        if (!strcmp(((serial_port_t*)device->port)->portName, com))
            if (device->port)
                return portId(device->port);
    }
    return -1;
}

is_operation_result InertialSense::BootloadFile(
        const string& comPort,
        const uint32_t serialNum,
        const string& fileName,
        const string& blFileName,
        bool forceBootloaderUpdate,
        int baudRate,
        ISBootloader::pfnBootloadProgress uploadProgress,
        ISBootloader::pfnBootloadProgress verifyProgress,
        ISBootloader::pfnBootloadStatus infoProgress,
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
    sort(comPorts.begin(), comPorts.end());

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
    sort(all_ports.begin(), all_ports.end());
    sort(comPorts.begin(), comPorts.end());
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

    #if !PLATFORM_IS_WINDOWS
    fputs("\e[?25l", stdout);    // Turn off cursor during firmware update
    #endif

    printf("\n\r");

    ISBootloader::firmwares_t files;
    files.fw_uINS_3.path = fileName;
    files.bl_uINS_3.path = blFileName;
    files.fw_IMX_5.path = fileName;
    files.bl_IMX_5.path = blFileName;
    files.fw_EVB_2.path = fileName;
    files.bl_EVB_2.path = blFileName;

    cISBootloaderThread::set_mode_and_check_devices(comPorts, baudRate, files, uploadProgress, verifyProgress, infoProgress, waitAction);

    cISSerialPort::GetComPorts(all_ports);

    // Get the list of ports to ignore during the bootloading process
    sort(all_ports.begin(), all_ports.end());
    sort(ports_user_ignore.begin(), ports_user_ignore.end());
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

bool InertialSense::OnClientPacketReceived(const uint8_t* data, uint32_t dataLength)
{
    for (auto device : m_comManagerState.devices) { device->SendRaw(data, dataLength); }
    return false; // do not parse, since we are just forwarding it on
}

void InertialSense::OnClientConnecting(cISTcpServer* server)
{
    (void)server;
    // cout << endl << "Client connecting..." << endl;
}

void InertialSense::OnClientConnected(cISTcpServer* server, socket_t socket)
{
    // cout << endl << "Client connected: " << (int)socket << endl;
    m_clientConnectionsCurrent++;
    m_clientConnectionsTotal++;
}

void InertialSense::OnClientConnectFailed(cISTcpServer* server)
{
    // cout << endl << "Client connection failed!" << endl;
}

void InertialSense::OnClientDisconnected(cISTcpServer* server, socket_t socket)
{
    // cout << endl << "Client disconnected: " << (int)socket << endl;
    m_clientConnectionsCurrent--;
    if (m_clientConnectionsCurrent<0)
    {
        m_clientConnectionsCurrent = 0;
    }
}

int InertialSense::OnSerialPortError(port_handle_t port, int errCode, const char *errMsg) {
    printf("%s\n", errMsg);
    return 0;
}

port_handle_t InertialSense::allocateSerialPort(int ptype) {
//    vector<base_port_t *> serPorts;
//    serPorts.push_back(0);
//    base_port_t* tmpPort = serPorts[0];
//    int pId = portId(tmpPort);
//    serPorts.clear();

    // TODO :: TRYING SOMETHING DIFFERENT
    //   -- rather than using new, m_serialPorts will allocate a new port using a copy constructor.
    //   We'll dereference the internal copy in the vector to a port_handle_t -- this avoids us using new/delete or malloc/free
    serial_port_t* serialPort = new serial_port_t;
    serialPort->base.pnum = (uint16_t)m_serialPorts.size(); // m_comManagerState.devices.size();
    serialPort->base.ptype = (ptype | PORT_TYPE__UART);

    serialPort->pfnError = OnSerialPortError;

    port_handle_t port = (port_handle_t)serialPort;
    serialPortPlatformInit(port);

    m_serialPorts.insert(port);
    comManagerRegisterPort(port);    // don't forget to register this port with the comManager and wire callbacks

    return port;
}

bool InertialSense::freeSerialPort(port_handle_t port, bool releaseDevice) {
    if (!port)
        return false;

    // its annoying that its not "easy" to remove a vector element by value, especially when the value is complex
    for (auto p : m_serialPorts) {
        if (p == port) {
            ISDevice* device = getDevice(port);

            // TODO: there should be some kind of a callback/notify system where a port can directly notify listeners that its being closed/destroyed
            serialPortClose(port);
            comManagerRemovePort(port);     // remove port from ComManager so it won't try to read/write the port when stepping();

            if (device) {
                if (device && releaseDevice) {
                    device->port = NULL;
                    m_comManagerState.devices.remove(device);
                }

            }

            m_serialPorts.erase(m_serialPorts.find(port));
            // TODO: Don't need this, if we stick with the vector/copy allocation/initialization
            // memset(port, 0, sizeof(serial_port_t));
            // delete (serial_port_t*)port;
            return true;
        }
    }

    return false;
}

bool InertialSense::OpenSerialPorts(const char* portPattern, int baudRate)
{
    CloseSerialPorts();

    if (portPattern == NULLPTR || validateBaudRate(baudRate) != 0)
    {
        return false;
    }

    // split port on comma in case we need to open multiple serial ports
    vector<string> portNames;
    size_t maxCount = UINT32_MAX;

    debug_message("[DBG] Initializing comManager...\n");
    comManagerInit(&m_serialPorts, 10, staticProcessRxData, 0, 0, 0, &m_cmBufBcastMsg);
    comManagerRegisterProtocolHandler(_PTYPE_NMEA, staticProcessRxNmea);
    comManagerRegisterProtocolHandler(_PTYPE_UBLOX, m_handlerUblox);
    comManagerRegisterProtocolHandler(_PTYPE_RTCM3, m_handlerRtcm3);
    comManagerRegisterProtocolHandler(_PTYPE_SPARTN, m_handlerSpartn);

    // handle wildcard, auto-detect serial ports
    if (portPattern[0] == '*')
    {
        // m_enableDeviceValidation = true; // always use device-validation when given the 'all ports' wildcard.    (WHJ) I commented this out.  We don't want to force device verification with the loopback tests.
        debug_message("[DBG] Querying OS for available serial ports.\n");
        cISSerialPort::GetComPorts(portNames);
        if (portPattern[1] != '\0')
        {
            maxCount = atoi(portPattern + 1);
            maxCount = (maxCount == 0 ? UINT32_MAX : maxCount);
        }
    }
    else
    {
        // comma separated list of serial ports
        splitString(portPattern, ',', portNames);
    }

    // allocate, register and open serial ports, but don't if its already allocated, registered, and opened.
    for (std::string curPortName : portNames) {
        // check to see if this port should be ignored
        bool skipPort = false;
        for (auto ignored : m_ignoredPorts) {
            if (ignored == curPortName) {
                debug_message("[DBG] Ignoring port %s\n", curPortName.c_str());
                skipPort = true;
            }
        }
        if (skipPort)
            continue;   // skip this one...

        // check is this port already exists, and is open...
        port_handle_t newPort = NULL;
        for (auto port : m_serialPorts) {
            if (portName(port) && (portName(port) == curPortName)) {
                debug_message("[DBG] Serial port '%s' has already been allocated.\n", curPortName.c_str());
                newPort = port;
                break;
            }
        }

        if (!newPort) {
            debug_message("[DBG] Allocating serial port instance for %s\n", curPortName.c_str());
            newPort = allocateSerialPort(PORT_TYPE__COMM);
        }

        if (!serialPortIsOpen(newPort)) {
            debug_message("[DBG] Opening serial port '%s'\n", curPortName.c_str());
            if (serialPortOpen(newPort, curPortName.c_str(), baudRate, 0) == 0) {
                debug_message("[DBG] Error opening serial port '%s'.  Ignoring.  Error was: %s\n", curPortName.c_str(), SERIAL_PORT(newPort)->error);
                serialPortClose(newPort);           // failed to open
                m_ignoredPorts.push_back(curPortName);     // record this port name as bad, so we don't try and reopen it again
            }
        } else {
          // nothing to do, it's already registered and open.
        }
    }

    bool timeoutOccurred = false;
    if (m_enableDeviceValidation) {
        unsigned int startTime = current_timeMs();
        uint8_t checkType = 0; // 0 == NMEA, 1 = mcuBoot, 2 = IS binary, 3 = IS bootloader

        do {
            // doing the timeout check first help during debugging (since stepping through code will likely trigger the timeout.
            if ((current_timeMs() - startTime) > (uint32_t)m_comManagerState.discoveryTimeout) {
                timeoutOccurred = true;
                break;
            }

            for (auto port : m_serialPorts) {
                ISDevice device(port);
                switch (checkType) {
                    case 0 :
                        comManagerSendRaw(port, (uint8_t *) NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE);
                        break;
                    case 1 :
                        comManagerGetData(port, DID_DEV_INFO, 0, 0, 0);
                        break;
                    case 2 :
                        // comManagerGetData(port, DID_DEV_INFO, 0, 0, 0);
                        break;
                    case 3 :
                        device.queryDeviceInfoISbl();
                        //comManagerGetData(port, DID_DEV_INFO, 0, 0, 0);
                        break;
                }

                for (int i = 0; i < 3; i++) {
                    SLEEP_MS(5);
                    comManagerStep();
                }

                if ((device.hdwId != 0) && (device.hdwRunState != 0)) {
                    debug_message("[DBG] Received response from serial port '%s'. Registering device.\n", SERIAL_PORT(port)->portName);
                    registerDevice(&device);
                } else if (SERIAL_PORT(port)->errorCode) {
                    // there was some other janky issue with the requested port; even though the device technically exists, its in a bad state. Let's just drop it now.
                    debug_message("[DBG] There was an error accessing serial port '%s': %s\n", SERIAL_PORT(port)->portName, SERIAL_PORT(port)->error);
                    comManagerRemovePort(port);
                }
            }

            checkType = (checkType + 1) % 4;
        } while (!HasReceivedDeviceInfoFromAllDevices());

        // remove each failed device where communications were not received
        std::vector<std::string> deadPorts;
        std::vector<ISDevice*> deadDevices;
        for (auto device : m_comManagerState.devices) {
            if (!HasReceivedDeviceInfo(device)) {
                debug_message("[DBG] Failed to receive response on serial port '%s'\n", portName(device->port));
                deadPorts.push_back(portName(device->port));
                deadDevices.push_back(device);
            }
        }

        if (timeoutOccurred && !deadPorts.empty()) {
            fprintf(stderr, "Timeout waiting for response from ports: [");
            for (auto portItr = deadPorts.begin(); portItr != deadPorts.end(); portItr++) {
                fprintf(stderr, "%s%s", (portItr == deadPorts.begin() ? "" : ", "), portItr->c_str());
            }
            fprintf(stderr, "]\n");
            fflush(stderr);
        }

        // We don't remove the device above while still iterating over the list of devices.
        for (auto deadDevice : deadDevices) {
            if (deadDevice) {
                debug_message("[DBG] Deallocating device associated with port '%s'\n", portName(deadDevice->port));
                RemoveDevice(deadDevice);
            }
        }
        deadDevices.clear();
    }

    // request extended device info for remaining connected devices...
    for (auto device : m_comManagerState.devices) {
        // but only if they are of a compatible protocol version
        if ((device->devInfo.protocolVer[0] == PROTOCOL_VERSION_CHAR0) && (device->hdwRunState == ISDevice::HDW_STATE_APP)) {
            device->GetData(DID_SYS_CMD, 0, 0, 0);
            device->GetData(DID_SYS_CMD, 0, 0, 0);
            device->GetData(DID_FLASH_CONFIG, 0, 0, 0);
            device->GetData(DID_EVB_FLASH_CFG, 0, 0, 0);
        }
    }

    return (m_enableDeviceValidation ? !m_comManagerState.devices.empty() : !m_serialPorts.empty());
}

void InertialSense::CloseSerialPorts(bool drainBeforeClose)
{
    debug_message("Closing all serial ports.\n");

    // Note the distinction here; we are closing ports, not devices...  Maybe we should do this differently though?
    for (auto port : m_serialPorts) {
        if (port) {
            if (drainBeforeClose) {
                serialPortDrain(port);
            }
            serialPortClose(port);
        }
    }

    // TODO: we should find the associated port in the m_comManagerState.devices, and remove the port reference
    // TODO: we need to provide a notification mechanism to inform consumers (ie, test_common framework) to clean up as well.
    // TODO: we probably need to make sure all other references to the port are clear and then destroy the underlying port instance
}

void InertialSense::SaveFlashConfigFile(std::string path, port_handle_t port)
{
    ISDevice* device = DeviceByPort(port);
    if (!device)
        return;

    nvm_flash_cfg_t* outData = &device->flashCfg;

    YAML::Node map = YAML::Node(YAML::NodeType::Map);

    map["size"]             = outData->size;
    map["checksum"]         = outData->checksum;
    map["key"]              = outData->key;
    map["startupImuDtMs"]   = outData->startupImuDtMs;
    map["startupNavDtMs"]   = outData->startupNavDtMs;
    map["ser0BaudRate"]     = outData->ser0BaudRate;
    map["ser1BaudRate"]     = outData->ser1BaudRate;

    YAML::Node insRotation = YAML::Node(YAML::NodeType::Sequence);
    insRotation.push_back(outData->insRotation[0]);
    insRotation.push_back(outData->insRotation[1]);
    insRotation.push_back(outData->insRotation[2]);
    map["insRotation"]                 = insRotation;

    YAML::Node insOffset = YAML::Node(YAML::NodeType::Sequence);
    insOffset.push_back(outData->insOffset[0]);
    insOffset.push_back(outData->insOffset[1]);
    insOffset.push_back(outData->insOffset[2]);
    map["insOffset"]                 = insOffset;

    YAML::Node gps1AntOffset = YAML::Node(YAML::NodeType::Sequence);
    gps1AntOffset.push_back(outData->gps1AntOffset[0]);
    gps1AntOffset.push_back(outData->gps1AntOffset[1]);
    gps1AntOffset.push_back(outData->gps1AntOffset[2]);
    map["gps1AntOffset"]    = gps1AntOffset;

    map["dynamicModel"]     = (uint16_t)outData->dynamicModel;
    map["debug"]            = (uint16_t)outData->debug;
    map["gnssSatSigConst"]  = outData->gnssSatSigConst;
    map["sysCfgBits"]       = outData->sysCfgBits;

    YAML::Node refLla = YAML::Node(YAML::NodeType::Sequence);
    refLla.push_back(outData->refLla[0]);
    refLla.push_back(outData->refLla[1]);
    refLla.push_back(outData->refLla[2]);
    map["refLla"]                     = refLla;

    YAML::Node lastLla = YAML::Node(YAML::NodeType::Sequence);
    lastLla.push_back(outData->lastLla[0]);
    lastLla.push_back(outData->lastLla[1]);
    lastLla.push_back(outData->lastLla[2]);
    map["lastLla"]                     = lastLla;

    map["lastLlaTimeOfWeekMs"]      = outData->lastLlaTimeOfWeekMs;
    map["lastLlaWeek"]              = outData->lastLlaWeek;
    map["lastLlaUpdateDistance"]    = outData->lastLlaUpdateDistance;
    map["ioConfig"]                 = outData->ioConfig;
    map["platformConfig"]           = outData->platformConfig;

    YAML::Node gps2AntOffset = YAML::Node(YAML::NodeType::Sequence);
    gps2AntOffset.push_back(outData->gps2AntOffset[0]);
    gps2AntOffset.push_back(outData->gps2AntOffset[1]);
    gps2AntOffset.push_back(outData->gps2AntOffset[2]);
    map["gps2AntOffset"]            = gps2AntOffset;

    YAML::Node zeroVelRotation = YAML::Node(YAML::NodeType::Sequence);
    zeroVelRotation.push_back(outData->zeroVelRotation[0]);
    zeroVelRotation.push_back(outData->zeroVelRotation[1]);
    zeroVelRotation.push_back(outData->zeroVelRotation[2]);
    map["zeroVelRotation"]          = zeroVelRotation;

    YAML::Node zeroVelOffset = YAML::Node(YAML::NodeType::Sequence);
    zeroVelOffset.push_back(outData->zeroVelOffset[0]);
    zeroVelOffset.push_back(outData->zeroVelOffset[1]);
    zeroVelOffset.push_back(outData->zeroVelOffset[2]);
    map["zeroVelOffset"]            = zeroVelOffset;

    map["gpsTimeUserDelay"]         = outData->gpsTimeUserDelay;
    map["magDeclination"]           = outData->magDeclination;
    map["gpsTimeSyncPeriodMs"]      = outData->gpsTimeSyncPeriodMs;
    map["startupGPSDtMs"]           = outData->startupGPSDtMs;
    map["RTKCfgBits"]               = outData->RTKCfgBits;
    map["sensorConfig"]             = outData->sensorConfig;
    map["gpsMinimumElevation"]      = outData->gpsMinimumElevation;
    map["ser2BaudRate"]             = outData->ser2BaudRate;

    YAML::Node wheelCfgTransE_b2w   = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[0]);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[1]);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[2]);
    map["wheelCfgTransE_b2w"]       = wheelCfgTransE_b2w;

    YAML::Node wheelCfgTransE_b2wsig = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[0]);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[1]);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[2]);
    map["wheelCfgTransE_b2wsig"]    = wheelCfgTransE_b2wsig;

    YAML::Node wheelCfgTransT_b2w = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[0]);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[1]);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[2]);
    map["wheelCfgTransT_b2w"]       = wheelCfgTransT_b2w;

    YAML::Node wheelCfgTransT_b2wsig = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[0]);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[1]);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[2]);
    map["wheelCfgTransT_b2wsig"]    = wheelCfgTransT_b2wsig;

    map["wheelConfigTrackWidth"]    = outData->wheelConfig.track_width;
    map["wheelConfigRadius"]        = outData->wheelConfig.radius;
    map["wheelConfigBits"]          = outData->wheelConfig.bits;

    std::ofstream fout(path);

    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << map;
    fout << emitter.c_str();
    fout.close();
}

int InertialSense::LoadFlashConfig(std::string path, port_handle_t port)
{
    try
    {
        nvm_flash_cfg_t loaded_flash;
        FlashConfig(loaded_flash);

        YAML::Node inData = YAML::LoadFile(path);
        loaded_flash.size                     = inData["size"].as<uint32_t>();
        loaded_flash.checksum                 = inData["checksum"].as<uint32_t>();
        loaded_flash.key                      = inData["key"].as<uint32_t>();
        loaded_flash.startupImuDtMs           = inData["startupImuDtMs"].as<uint32_t>();
        loaded_flash.startupNavDtMs           = inData["startupNavDtMs"].as<uint32_t>();
        loaded_flash.ser0BaudRate             = inData["ser0BaudRate"].as<uint32_t>();
        loaded_flash.ser1BaudRate             = inData["ser1BaudRate"].as<uint32_t>();

        YAML::Node insRotation                = inData["insRotation"];
        loaded_flash.insRotation[0]           = insRotation[0].as<float>();
        loaded_flash.insRotation[1]           = insRotation[1].as<float>();
        loaded_flash.insRotation[2]           = insRotation[2].as<float>();

        YAML::Node insOffset                  = inData["insOffset"];
        loaded_flash.insOffset[0]             = insOffset[0].as<float>();
        loaded_flash.insOffset[1]             = insOffset[1].as<float>();
        loaded_flash.insOffset[2]             = insOffset[2].as<float>();

        YAML::Node gps1AntOffset              = inData["gps1AntOffset"];
        loaded_flash.gps1AntOffset[0]         = gps1AntOffset[0].as<float>();
        loaded_flash.gps1AntOffset[1]         = gps1AntOffset[1].as<float>();
        loaded_flash.gps1AntOffset[2]         = gps1AntOffset[2].as<float>();

        loaded_flash.dynamicModel             = (uint8_t)inData["dynamicModel"].as<uint16_t>();
        loaded_flash.debug                    = (uint8_t)inData["debug"].as<uint16_t>();
        loaded_flash.gnssSatSigConst          = inData["gnssSatSigConst"].as<uint16_t>();
        loaded_flash.sysCfgBits               = inData["sysCfgBits"].as<uint32_t>();

        YAML::Node refLla                     = inData["refLla"];
        loaded_flash.refLla[0]                = refLla[0].as<double>();
        loaded_flash.refLla[1]                = refLla[1].as<double>();
        loaded_flash.refLla[2]                = refLla[2].as<double>();

        YAML::Node lastLla                    = inData["lastLla"];
        loaded_flash.lastLla[0]               = lastLla[0].as<double>();
        loaded_flash.lastLla[1]               = lastLla[1].as<double>();
        loaded_flash.lastLla[2]               = lastLla[2].as<double>();

        loaded_flash.lastLlaTimeOfWeekMs      = inData["lastLlaTimeOfWeekMs"].as<uint32_t>();
        loaded_flash.lastLlaWeek              = inData["lastLlaWeek"].as<uint32_t>();
        loaded_flash.lastLlaUpdateDistance    = inData["lastLlaUpdateDistance"].as<float>();
        loaded_flash.ioConfig                 = inData["ioConfig"].as<uint32_t>();
        loaded_flash.platformConfig           = inData["platformConfig"].as<uint32_t>();

        YAML::Node gps2AntOffset              = inData["gps2AntOffset"];
        loaded_flash.gps2AntOffset[0]         = gps2AntOffset[0].as<float>();
        loaded_flash.gps2AntOffset[1]         = gps2AntOffset[1].as<float>();
        loaded_flash.gps2AntOffset[2]         = gps2AntOffset[2].as<float>();

        YAML::Node zeroVelRotation            = inData["zeroVelRotation"];
        loaded_flash.zeroVelRotation[0]       = zeroVelRotation[0].as<float>();
        loaded_flash.zeroVelRotation[1]       = zeroVelRotation[1].as<float>();
        loaded_flash.zeroVelRotation[2]       = zeroVelRotation[2].as<float>();

        YAML::Node zeroVelOffset              = inData["zeroVelOffset"];
        loaded_flash.zeroVelOffset[0]         = zeroVelOffset[0].as<float>();
        loaded_flash.zeroVelOffset[1]         = zeroVelOffset[1].as<float>();
        loaded_flash.zeroVelOffset[2]         = zeroVelOffset[2].as<float>();

        loaded_flash.gpsTimeUserDelay         = inData["gpsTimeUserDelay"].as<float>();
        loaded_flash.magDeclination           = inData["magDeclination"].as<float>();
        loaded_flash.gpsTimeSyncPeriodMs      = inData["gpsTimeSyncPeriodMs"].as<uint32_t>();
        loaded_flash.startupGPSDtMs           = inData["startupGPSDtMs"].as<uint32_t>();
        loaded_flash.RTKCfgBits               = inData["RTKCfgBits"].as<uint32_t>();
        loaded_flash.sensorConfig             = inData["sensorConfig"].as<uint32_t>();
        loaded_flash.gpsMinimumElevation      = inData["gpsMinimumElevation"].as<float>();
        loaded_flash.ser2BaudRate             = inData["ser2BaudRate"].as<uint32_t>();

        loaded_flash.wheelConfig.bits         = inData["wheelConfigBits"].as<uint32_t>();
        loaded_flash.wheelConfig.radius       = inData["wheelConfigRadius"].as<float>();
        loaded_flash.wheelConfig.track_width  = inData["wheelConfigTrackWidth"].as<float>();

        YAML::Node wheelCfgTransE_b2w                       = inData["wheelCfgTransE_b2w"];
        loaded_flash.wheelConfig.transform.e_b2w[0]         = wheelCfgTransE_b2w[0].as<float>();
        loaded_flash.wheelConfig.transform.e_b2w[1]         = wheelCfgTransE_b2w[1].as<float>();
        loaded_flash.wheelConfig.transform.e_b2w[2]         = wheelCfgTransE_b2w[2].as<float>();

        YAML::Node wheelCfgTransE_b2wsig                    = inData["wheelCfgTransE_b2wsig"];
        loaded_flash.wheelConfig.transform.e_b2w_sigma[0]   = wheelCfgTransE_b2wsig[0].as<float>();
        loaded_flash.wheelConfig.transform.e_b2w_sigma[1]   = wheelCfgTransE_b2wsig[1].as<float>();
        loaded_flash.wheelConfig.transform.e_b2w_sigma[2]   = wheelCfgTransE_b2wsig[2].as<float>();

        YAML::Node wheelCfgTransT_b2w                       = inData["wheelCfgTransT_b2wsig"];
        loaded_flash.wheelConfig.transform.t_b2w[0]         = wheelCfgTransT_b2w[0].as<float>();
        loaded_flash.wheelConfig.transform.t_b2w[1]         = wheelCfgTransT_b2w[1].as<float>();
        loaded_flash.wheelConfig.transform.t_b2w[2]         = wheelCfgTransT_b2w[2].as<float>();

        YAML::Node wheelCfgTransT_b2wsig                    = inData["wheelCfgTransT_b2wsig"];
        loaded_flash.wheelConfig.transform.t_b2w_sigma[0]   = wheelCfgTransT_b2wsig[0].as<float>();
        loaded_flash.wheelConfig.transform.t_b2w_sigma[1]   = wheelCfgTransT_b2wsig[1].as<float>();
        loaded_flash.wheelConfig.transform.t_b2w_sigma[2]   = wheelCfgTransT_b2wsig[2].as<float>();

        SetFlashConfig(loaded_flash);
    }
    catch (const YAML::Exception& ex)
    {
        printf("[ERROR] --- There was an error parsing the YAML file: %s", ex.what());
        return -1;
    }

    return 0;
}

/**
* Get the device info
* @param port the port to get device info for
* @return the device info
*/
const dev_info_t InertialSense::DeviceInfo(port_handle_t port)
{
    ISDevice* device = DeviceByPort(port);
    if (device)
        return device->devInfo;

    return {};
}

/**
* Get current device system command
* @param port the port to get sysCmd for
* @return current device system command
*/
system_command_t InertialSense::GetSysCmd(port_handle_t port)
{
    ISDevice* device = DeviceByPort(port);
    if (device)
        return device->sysCmd;

    return { 0, 0 };    // nothing...
}

/**
 * Returns the device associated with the specified port
 * @param port
 * @return ISDevice* which is connected to port, otherwise NULL
 */
ISDevice* InertialSense::DeviceByPort(port_handle_t port) {
    if (port == nullptr)
        return (!m_comManagerState.devices.empty() ? m_comManagerState.devices.front() : nullptr);

    for (auto device: m_comManagerState.devices) {
        if (device->port == port)
            return device;
    }
    return nullptr;
}

/**
 * Resturns the device associated with the specified port name
 * @param port
 * @return ISDevice* which is connected to port, otherwise NULL
 */
ISDevice* InertialSense::DeviceByPortName(const std::string& port_name) {
    for (auto device : m_comManagerState.devices) {
        if (device->port) {
            const char* devPortName = portName(device->port);
            if (devPortName && (std::string(devPortName) == port_name))
                return device;
        }
    }
    return nullptr;
}

/**
 * @return a list of discovered ports which are not currently associated with a open device
 */
std::vector<std::string> InertialSense::checkForNewPorts(std::vector<std::string>& oldPorts) {
    std::vector<std::string> new_ports, all_ports;
    cISSerialPort::GetComPorts(all_ports);

    // remove from "ports", all the ports which we currently have an open connection for.
    for (auto& portName : all_ports) {
        bool ignored = false;
        for (auto& ignoredPort : m_ignoredPorts) {
            if (ignoredPort == portName)
                ignored = true;
        }
        for (auto& ignoredPort : oldPorts) {
            if (ignoredPort == portName)
                ignored = true;
        }
        if (ignored)
            continue;

        if (DeviceByPortName(portName) == nullptr)
            new_ports.push_back(portName);
    }

    return new_ports;
}



/**
 * Compared two dev_info_t structs, and returns an bitmap indicating which fields match
 * @param info1
 * @param info2
 * @return a uint32_t with each bit indicating a match of a specific field in the struct
 */
uint32_t InertialSense::compareDevInfo(const dev_info_t& info1, const dev_info_t& info2) {
    uint32_t match = 0;

    match |= (((info1.reserved          == info2.reserved)          & 1) << 0);

    match |= (((info1.hardwareType      == info2.hardwareType)      & 1) << 1);
    match |= (((info1.reserved2         == info2.reserved2)         & 1) << 2);

    match |= (((info1.serialNumber      == info2.serialNumber)      & 1) << 3);

    match |= (((info1.hardwareVer[0]    == info2.hardwareVer[0])    & 1) << 4);
    match |= (((info1.hardwareVer[1]    == info2.hardwareVer[1])    & 1) << 5);
    match |= (((info1.hardwareVer[2]    == info2.hardwareVer[2])    & 1) << 6);
    match |= (((info1.hardwareVer[3]    == info2.hardwareVer[3])    & 1) << 7);

    match |= (((info1.firmwareVer[0]    == info2.firmwareVer[0])    & 1) << 8);
    match |= (((info1.firmwareVer[1]    == info2.firmwareVer[1])    & 1) << 9);
    match |= (((info1.firmwareVer[2]    == info2.firmwareVer[2])    & 1) << 10);
    match |= (((info1.firmwareVer[3]    == info2.firmwareVer[3])    & 1) << 11);

    match |= (((info1.buildNumber       == info2.buildNumber)       & 1) << 12);

    match |= (((info1.protocolVer[0]    == info2.protocolVer[0])    & 1) << 13);
    match |= (((info1.protocolVer[1]    == info2.protocolVer[1])    & 1) << 14);
    match |= (((info1.protocolVer[2]    == info2.protocolVer[2])    & 1) << 15);
    match |= (((info1.protocolVer[3]    == info2.protocolVer[3])    & 1) << 16);

    match |= (((!strncmp(info1.manufacturer, info2.manufacturer, DEVINFO_MANUFACTURER_STRLEN))    & 1) << 17);

    match |= (((info1.buildType         == info2.buildType)         & 1) << 18);

    match |= (((info1.buildYear         == info2.buildYear)         & 1) << 19);
    match |= (((info1.buildMonth        == info2.buildMonth)        & 1) << 20);
    match |= (((info1.buildDay          == info2.buildDay)          & 1) << 21);

    match |= (((info1.buildHour         == info2.buildHour)         & 1) << 22);
    match |= (((info1.buildMinute       == info2.buildMinute)       & 1) << 23);
    match |= (((info1.buildSecond       == info2.buildSecond)       & 1) << 24);
    match |= (((info1.buildMillisecond  == info2.buildMillisecond)  & 1) << 25);

    match |= (((!strncmp(info1.addInfo, info2.addInfo, DEVINFO_ADDINFO_STRLEN))    & 1) << 26);

    return match;
}

/**
 * Returns a subset of connected devices filtered by the passed devInfo and filterFlags.
 * filterFlags is a bitmask the matches the returned bitmap from compareDevInfo, in which
 * each bit corresponds to a field in devInfo, which must be matched in order to be
 * selected. All bits which are set in filterFlags must also be set in the result from
 * compareDevInfo in order to selected.  Passing 0x0000 for filterFlags will return all available
 * devices (any device matches), while passing 0xFFFF will only match an exact match, including
 * the serial number.
 * @param devInfo
 * @param filterFlags
 * @return a vector of ISDevice which match the filter criteria
 */
std::vector<ISDevice*> InertialSense::selectByDevInfo(const dev_info_t& devInfo, uint32_t filterFlags) {
    std::vector<ISDevice*> selected;

    for (auto device : m_comManagerState.devices) {
        uint32_t matchy = compareDevInfo(devInfo, device->devInfo) & filterFlags;
        if (matchy == filterFlags)
            selected.push_back(device);
    }
    return selected;
}

/**
 * Returns a subset of connected devices filtered by the passed hardware id.
 * Note that any HdwId component (TYPE, MAJOR, MINOR) which bit mask is all ones, will
 * be ignored in the filter criteria.  Ie, to filter on ALL IMX devices, regardless of
 * version, pass hdwId = ENCODE_HDW_ID(HDW_TYPE__IMX, 0xFF, 0xFF), or to filter on any
 * IMX-5.x devices, pass hdwId = ENCODE_HDW_ID(HDW_TYPE__IMX, 5, 0xFF)
 * @param hdwId
 * @return a vector of ISDevice* which match the filter criteria (hdwId)
 */
std::vector<ISDevice*> InertialSense::selectByHdwId(const uint16_t hdwId) {
    dev_info_t devInfo = { };
    uint32_t filterFlags = 0;

    // filter hdw type
    devInfo.hardwareType = DECODE_HDW_TYPE(hdwId);
    if (devInfo.hardwareType != (HDW_TYPE__MASK >> HDW_TYPE__SHIFT)) {
        filterFlags |= (1 << 1);
    }

    // filter major hdw version
    devInfo.hardwareVer[0] = DECODE_HDW_MAJOR(hdwId);
    if (devInfo.hardwareVer[0] != (HDW_MAJOR__MASK >> HDW_MAJOR__SHIFT)) {
        filterFlags |= (1 << 4);
    }

    // filter minor hdw version
    devInfo.hardwareVer[1] = DECODE_HDW_MINOR(hdwId);
    if (devInfo.hardwareVer[1] != (HDW_MINOR__MASK >> HDW_MINOR__SHIFT)) {
        filterFlags |= (1 << 5);
    }

    return selectByDevInfo(devInfo, filterFlags);
}
