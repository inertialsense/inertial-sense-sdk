/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

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
#include "TCPPortFactory.h"
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

static int staticProcessRxNmea(void* ctx, const unsigned char* msg, int msgSize, port_handle_t port)
{
    if (port)
    {
        s_cm_state->inertialSenseInterface->ProcessRxNmea(port, msg, msgSize);
    }
    return 0;
}

InertialSense::InertialSense(std::vector<PortFactory*> pFactories, std::vector<DeviceFactory*> dFactories) : m_tcpServer(this)
{
    s_is = this;
    s_cm_state = &m_comManagerState;
    m_logThread = NULLPTR;
    m_lastLogReInit = time(0);
    m_clientStream = NULLPTR;
    m_clientBufferBytesToSend = 0;
    m_clientServerByteCount = 0;
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
        TCPPortFactory& tpf = TCPPortFactory::getInstance();
        tpf.portOptions.defaultBlocking = false;
        portManager.addPortFactory(&tpf);
    } else {
        for (auto f : pFactories) portManager.addPortFactory(f);
    }
    portManager.addPortListener([this](auto && PH1, auto && PH2, auto && PH3, auto && PH4) { portManagerHandler(PH1, PH2, PH3, PH4); });


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

    is_comm_init(&m_gpComm, m_gpCommBuffer, sizeof(m_gpCommBuffer), NULL);

    m_newDeviceHandler = nullptr; // handlerNewDevice;
    // m_cloneDeviceHandler = handleClonedDevices;

    // Rx data callback functions
    m_handlerRmc    = nullptr;
    m_handlerNmea   = nullptr;
    m_handlerUblox  = nullptr;
    m_handlerRtcm3  = nullptr;
    m_handlerSpartn = nullptr;
}

InertialSense::InertialSense() : InertialSense( { (PortFactory*)&(SerialPortFactory::getInstance()) }, { (DeviceFactory*)&ImxDeviceFactory::getInstance() }) {
    s_is = this;
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
    m_disableBroadcastsOnClose = false;  // For Intel.

    deviceManager.addDeviceFactory((DeviceFactory*)&ImxDeviceFactory::getInstance());
    deviceManager.addDeviceListener([this](auto && PH1, auto && PH2) { deviceManagerHandler(PH1, PH2); });

    portManager.addPortFactory((PortFactory*)&(SerialPortFactory::getInstance()));
    portManager.addPortListener([this](auto && PH1, auto && PH2, auto && PH3, auto && PH4) { portManagerHandler(PH1, PH2, PH3, PH4); });

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
    // m_cloneDeviceHandler = handleClonedDevices;

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
                        ISDevice* device = inertialSense->getDevice(port);
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

#if 0
size_t InertialSense::DeviceCount()
{
    //return m_comManagerState.devices.size();
    return size();
}

/**
 * Returns a vector of available, connected devices
 * @return
 */
std::list<ISDevice*>& InertialSense::getDevices() {
    // return m_comManagerState.devices;
    return *this;
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

#endif


bool InertialSense::Update()
{
    m_timeMs = current_timeMs();

    if (m_tcpServer.IsOpen() && DeviceCount() > 0)
    {
        UpdateServer();
    }
    else
    {
        UpdateClient();

        // [C COMM INSTRUCTION]  2.) Update each device at regular interval to send and receive data.
        // Normally called within a while loop.  Include a thread "sleep" if running on a multi-thread/
        // task system with serial port read function that does NOT incorporate a timeout.
        for (auto device : deviceManager)
            if (device)
                device->step();
    }

    // if any serial ports have closed, shutdown
    bool anyOpen = false;
    for (auto device : deviceManager)
    {
        if (!portIsOpened(device->port))
        {
            // Make sure its closed..
            portClose(device->port);
        } else
            anyOpen = true;
    }

    return anyOpen;
}

/**
 * TCP Server primary handler - parses data received via the first connected device, looking for RTCM3/UBLOX protocol
 * and sends that same data out to the underlying ISTCPServer's connected clients
 * @return always returns true, though should probably return false if the m_tcpServer has no active clients (or something)
 */
bool InertialSense::UpdateServer()
{
    // As I understand it, this function is responsible for reading RTCM3, and other useful data sets from connected IMX,
    // and publishing it to connected clients (because it is the server).

    // This is a little different, kind-of, because we don't actually let the ISDevice parse any data (but maybe we should).
    // Rather, we parse data directly from the COMM buffer, so we can determine what type of data it is (though we should
    // already know this). then, based on the packet type (RTCM3/UBLOX, etc) we'll send that data out to the socket.
    //
    // Ideally, the TCP socket would also be a port_handle_t, and we'd essentially plumb up a passthrough:  Let the ISDevice
    // parse data FROM the device, call a custom callback for the data types we're interested in, and then when those are
    // received, we'd send them right back out the TCP port_handle_t.  Perhaps one day; not today.

    // as a tcp server, only the first serial port is read from
    port_handle_t port = deviceManager.front()->port;
    is_comm_instance_t *comm = &(COMM_PORT(port)->comm);
    protocol_type_t ptype = _PTYPE_NONE;

    // Get available size of comm buffer
    int n = is_comm_free(comm);         // TODO:  This is a little janky; as a Serial/COMM port, this should already know how to do these things...

    // Read data directly into comm buffer
    if ((n = portReadTimeout(port, comm->rxBuf.tail, n, 0)))
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
                messageStatsAppend(str, m_serverMessageStats, ptype, comm->rxPkt.id, comm->rxPkt.size, m_timeMs);
            }
        }
    }
    m_tcpServer.Update();

    return true;
}

/**
 *
 * @return
 */
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
                messageStatsAppend(str, m_clientMessageStats, ptype, comm->rxPkt.id, comm->rxPkt.size, m_timeMs);
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
    m_baudRate = baudRate;
    if (OpenSerialPorts(port, baudRate))
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
    for (ISDevice* d : deviceManager) {
        if (d->isConnected()) {
            portFlush(d->port);
        }
        d->disconnect();
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

                    if (device->flashCfgUpload.checksum == device->sysParams.flashCfgChecksum)
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
        imxPlatformConfigToFlashCfgIoConfig(&flashCfg.ioConfig, &flashCfg.ioConfig2, flashCfg.platformConfig);
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

    return (device ? device->FlashConfig(flashCfg) : false);
}

bool InertialSense::SetFlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port)
{
    ISDevice* device = NULL;
    if (!port) {
        device = m_comManagerState.devices.front();
    } else {
        device = DeviceByPort(port);
    }

    return (device ? device->SetFlashConfig(flashCfg) : false);
}

bool InertialSense::WaitForFlashSynced(port_handle_t port)
{
    ISDevice* device = NULL;
    if (!port) {
        device = m_comManagerState.devices.front();
    } else {
        device = DeviceByPort(port);
    }

    return (device ? device->WaitForFlashSynced() : false);
}
#endif

void InertialSense::ProcessRxData(port_handle_t port, p_data_t* data)
{
    if (data->hdr.size==0 || data->ptr==NULL)
    {
        return;
    }

    ISDevice* device = getDevice(port);
    if (!device) {
        if (data->hdr.id != DID_DEV_INFO)
            return;
    }

    switch (data->hdr.id)
    {
        case DID_DEV_INFO:
            if (!device) {
                device = deviceManager.registerNewDevice(port, *(dev_info_t*)data->ptr);
                device->devInfo.hdwRunState = HDW_STATE_APP; // we know this for a fact
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
            ISDevice* device = getDevice(port);
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

#endif

is_operation_result InertialSense::updateFirmware(fwUpdate::target_t targetDevice, std::vector<std::string> cmds, fwUpdate::pfnStatusCb fwUpdateStatus, void (*waitAction)())
{
    for (auto device : deviceManager) {
        device->updateFirmware(targetDevice, cmds, fwUpdateStatus, waitAction);
    }

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
        if (fwUpdater && fwUpdater->fwUpdate_isDone() && ((fwUpdater->fwUpdate_getSessionStatus() < fwUpdate::NOT_STARTED) || fwUpdater->hasErrors()))
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

bool InertialSense::OnClientPacketReceived(const uint8_t* data, uint32_t dataLength)
{
    for (auto device : deviceManager) { device->SendRaw(data, dataLength); }
    return false; // do not parse, since we are just forwarding it on
}

void InertialSense::OnClientConnecting(cISTcpServer* server)
{
    (void)server;
    // cout << endl << "Client connecting..." << endl;
}

void InertialSense::OnClientConnected(cISTcpServer* server, is_socket_t socket)
{
    // cout << endl << "Client connected: " << (int)socket << endl;
    m_clientConnectionsCurrent++;
    m_clientConnectionsTotal++;
}

void InertialSense::OnClientConnectFailed(cISTcpServer* server)
{
    // cout << endl << "Client connection failed!" << endl;
}

void InertialSense::OnClientDisconnected(cISTcpServer* server, is_socket_t socket)
{
    // cout << endl << "Client disconnected: " << (int)socket << endl;
    m_clientConnectionsCurrent--;
    if (m_clientConnectionsCurrent<0)
    {
        m_clientConnectionsCurrent = 0;
    }
}

int InertialSense::OnPortError(port_handle_t port, int errCode, const char *errMsg) {
    printf("%s\n", errMsg);
    return 0;
}

bool InertialSense::OpenSerialPorts(const char* portPattern, int baudRate)
{
    m_baudRate = baudRate;

    CloseSerialPorts();

    if (portPattern == NULLPTR || validateBaudRate(baudRate) != 0)
    {
        return false;
    }

    // split port on comma in case we need to open multiple serial ports
    vector<string> portNames;
    size_t maxCount = UINT32_MAX;

    // Note that the following callbacks/handlers will be updated to be ISDevice specific, once the port is deemed to be an ISDevice
    debug_message("[DBG] Initializing comManager...\n");
    comManagerInit((std::unordered_set<port_handle_t>*)&portManager, 10, staticProcessRxData, 0, 0, 0, 0);
    comManagerRegisterProtocolHandler(_PTYPE_NMEA, staticProcessRxNmea);
    comManagerRegisterProtocolHandler(_PTYPE_UBLOX, m_handlerUblox);
    comManagerRegisterProtocolHandler(_PTYPE_RTCM3, m_handlerRtcm3);
    comManagerRegisterProtocolHandler(_PTYPE_SPARTN, m_handlerSpartn);

    // TODO: This should all be handled by the PortManager & PortFactory
    // handle wildcard, auto-detect serial ports
    if (portPattern[0] == '*')
    {
        // m_enableDeviceValidation = true; // always use device-validation when given the 'all ports' wildcard.    (WHJ) I commented this out.  We don't want to force device verification with the loopback tests.
        debug_message("[DBG] Querying OS for available serial ports.\n");
        portManager.discoverPorts();
        // cISSerialPort::GetComPorts(portNames);
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
        debug_message("[DBG] Starting device validation on %lu registered ports.\n", portManager.size());

        // we'll make a copy of all the port handles (into a set); as we validate each, we'll remove it from this new set until they are all gone
        for (auto port : portManager) portsToValidate.insert(port);

        // since we can re-validating some devices, there may be previous devInfo that would allow the validation to pass incorrectly, so clear it
        for (auto device : deviceManager) device->devInfo.hdwRunState = HDW_STATE_UNKNOWN;  // FIXME: (REMOVE) This doesn't actually do anything because we haven't discovered any devices yet

        // check for Inertial-Sense devices by making a series of protocol requests (which it should respond to at least one of)
        unsigned int startTime = current_timeMs();
        do {
            // doing the timeout check first helps during debugging (since stepping through code will likely trigger the timeout.
            if ((current_timeMs() - startTime) > (uint32_t)m_comManagerState.discoveryTimeout)
                break;

            deviceManager.discoverDevices(IS_HARDWARE_ANY, 5000, DeviceManager::DISCOVERY__CLOSE_PORT_ON_FAILURE);  // In this case, We ABSOLUTELY want to open any closes ports (because they are all closed currently)
        } while (!portsToValidate.empty());
        debug_message("[DBG] Completed device validation for %lu devices, on %lu ports.\n", deviceManager.size(), portManager.size());

        if (!portsToValidate.empty()) {
            std::string names;
            for (auto port : portsToValidate) {
                if (!names.empty()) names += ", ";
                names += std::string(portName(port));
            }
            // auto names = utils::join_to_string<std::set<std::string>>(portsToValidate, ", ");
            fprintf(stderr, "Timeout waiting to validate %lu ports: %s.\n", portsToValidate.size(), names.c_str());
            fflush(stderr);
        }

        // Now we need to look for other devices (old?) devices which are effectively dead, and clean them up
        std::vector<ISDevice*> deadDevices;
        for (auto dev : deviceManager) {
            if (!dev->port || !dev->isConnected()) {
                deadDevices.push_back(dev);
            }
        }

        // We don't remove the device above while still iterating over the list of devices.
        if (!deadDevices.empty()) {
            for (auto deadDevice : deadDevices) {
                if (deadDevice) {
                    fprintf(stderr, "Found device %s on invalid/closed port %s.\n", deadDevice->getIdAsString().c_str(), deadDevice->getPortName().c_str());
                    deviceManager.releaseDevice(deadDevice);
                }
            }
            deadDevices.clear();
        }
    }

    // request extended device info for remaining connected devices...
    for (auto device : deviceManager) {
        // but only if they are of a compatible protocol version
        if (device->hasDeviceInfo()) {
            device->GetData(DID_SYS_PARAMS);
            device->GetData(DID_FLASH_CONFIG);
            // device->GetData(DID_EVB_FLASH_CFG);
        }
    }

    return (m_enableDeviceValidation ? !deviceManager.empty() : !portManager.empty());
}

void InertialSense::CloseSerialPorts(bool drainBeforeClose)
{
    // TODO: we should find the associated port in the m_comManagerState.devices, and remove the port reference
    // TODO: we need to provide a notification mechanism to inform consumers (ie, test_common framework) to clean up as well.
    // TODO: we probably need to make sure all other references to the port are clear and then destroy the underlying port instance

    // Note the distinction here; we are closing ports, not devices...  Maybe we should do this differently though?
    for (auto port : portManager) {
        if (port) {
            if (drainBeforeClose) {
                serialPortDrain(port, 0);
            }
            serialPortClose(port);
        }
    }
}

#if 0
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

#endif

/**
 * Resturns the device associated with the specified port name
 * @param port
 * @return ISDevice* which is connected to port, otherwise NULL
 */
ISDevice* InertialSense::DeviceByPortName(const std::string& port_name) {
    for (auto device : deviceManager) {
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
[[deprecated("Use InertialSense.portManager.discoverPorts() instead.")]]
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

        if (DeviceByPortName(portName) == nullptr) {
            new_ports.push_back(portName);
        }
    }

    return new_ports;
}


#if 0
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
        uint32_t matchy = util::compareDevInfo(devInfo, device->devInfo) & filterFlags;
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
#endif

/**
 * Handles port management for the InertialSense class. Specifically, defaults to opening newly discovered ports. This function can be
 * overridden to provide additional functionality is extending classes.
 * @param event the type of event for the specified port, typically either PORT_ADDED or PORT_REMOVED
 * @param pType the type of port that this event is associated with - together with the pName, this *should* uniquely identify the port
 * @param pName the name of the port this event is associated with - together with the pType, this *should* uniquely identify the port
 * @param port the port handle that is associated with this event - this maybe null if the port is being removed since this handler is called
 *   after the port has already been identified as having been removed.
 */
void InertialSense::portManagerHandler(uint8_t event, uint16_t pType, std::string pName, port_handle_t port) {
    switch ((PortManager::port_event_e)event) {
        case PortManager::PORT_ADDED:
            debug_message("[DBG] PortManager::PORT_ADDED '%s'\n", pName.c_str());
            if (!portIsOpened(port)) {
                // debug_message("[DBG] Opening serial port '%s'\n", curPortName.c_str());

                // FIXME: the portManagerHandler shouldn't ever open/close the port - it can, but it shouldn't.
                //  This is because the portManager should not ever "do anything" with the port, other than
                //  manage it for others.  So, for device discovery, etc. those services should operate on the
                //  port, but only when they need to do so.  Yes, its "nice" that this could validate that a
                //  port can be opened, if a port is busy elsewhere, that doesn't make it invalid.  Besides,
                //  its the PortFactory's job to validate the port, not the listener... usually.
/*
                if (portOpen(port) == PORT_ERROR__OPEN_FAILURE) {
                    debug_message("[DBG] Error opening serial port '%s'.  Ignoring.  Error was: %s\n", pName.c_str(), SERIAL_PORT(port)->error);
                    serialPortClose(port);           // failed to open
                    m_ignoredPorts.push_back(pName);     // record this port name as bad, so we don't try and reopen it again
                }
*/
            }
            break;
        case PortManager::PORT_REMOVED:
            debug_message("[DBG] PortManager::PORT_REMOVED '%s'.\n", pName.c_str());
            break;
    }
}

/**
 * Handles device management for the InertialSense class. Specifically, this is called when the device is successfully
 * validated and the port is assigned.  This is used with the portManagerHandler() to complete the device validation phase
 * @param event the type of event for the associated device, typically either DEVICE_ADDED or DEVICE_REMOVED
 * @param device an ISDevice pointer to the associated device.
 */
void InertialSense::deviceManagerHandler(uint8_t event, ISDevice* device) {
    switch ((DeviceManager::device_event_e)event) {
        case DeviceManager::DEVICE_ADDED: {
            debug_message("[DBG] Device %s added on port %s\n", device->getIdAsString().c_str(), portIsValid(device->port) ? portName(device->port) : "(None)");

            // since we've validated, we can remove this from the "portsToValidate" set
            auto removeMe = portsToValidate.find(device->port);
            if (removeMe != portsToValidate.end()) {
                debug_message("[DBG] Removed %s from portsToValidate\n", portIsValid(device->port) ? portName(device->port) : "(None)");
                portsToValidate.erase(removeMe);
            }
        }
            break;
        case DeviceManager::DEVICE_REMOVED:
            debug_message("[DBG] Device %s removed\n", device->getIdAsString().c_str());
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
    for (auto device : DeviceManager::getInstance()) { device->SoftwareReset(); }
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
        ISDevice* device = getDevice(port);
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
    ISDevice* device = getDevice(port);
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

    if (!port) {
        SendData(DID_EVENT, data, DID_EVENT_HEADER_SIZE + event.length, 0);
    } else {
        ISDevice *device = DeviceManager::getInstance().getDevice(port);
        if (device) device->SendData(DID_EVENT, data, DID_EVENT_HEADER_SIZE + event.length, 0);
    }
}

/*
bool InertialSense::FlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port) {
    ISDevice* device = port ? getDevice(port) : deviceManager.front();
    return (device ? device->FlashConfig(flashCfg) : false);
}

bool InertialSense::SetFlashConfig(nvm_flash_cfg_t &flashCfg, port_handle_t port) {
    ISDevice* device = port ? getDevice(port) : deviceManager.front();
    return (device ? device->SetFlashConfig(flashCfg) : false);}

bool InertialSense::WaitForFlashSynced(port_handle_t port) {
    ISDevice* device = port ? getDevice(port) : deviceManager.front();
    return (device ? device->WaitForFlashSynced() : false);
}
*/
