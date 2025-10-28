/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "protocol_nmea.h"
#include "yaml-cpp/yaml.h"
#include "protocol_nmea.h"
#include "InertialSense.h"
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

static int staticSendData(unsigned int port, const uint8_t* buf, int len)
{
    if ((size_t)port >= s_cm_state->devices.size())
    {
        return 0;
    }
    return serialPortWrite(&(s_cm_state->devices[port].serialPort), buf, len);
}

static int staticReadData(unsigned int port, uint8_t* buf, int len)
{
    if ((size_t)port >= s_cm_state->devices.size())
    {
        return 0;
    }
    int bytesRead = serialPortReadTimeout(&s_cm_state->devices[port].serialPort, buf, len, 1);

    if (s_is)
    {   // Save raw data to ISlogger
        s_is->LogRawData(&s_cm_state->devices[port], bytesRead, buf);
    }

    return bytesRead;
}

static int staticProcessRxData(unsigned int port, p_data_t* data)
{
    if (data->hdr.id >= (sizeof(s_cm_state->binaryCallback)/sizeof(pfnHandleBinaryData)))
    {
        return -1;
    }

    pfnHandleBinaryData handler = s_cm_state->binaryCallback[data->hdr.id];
    s_cm_state->stepLogFunction(s_cm_state->inertialSenseInterface, data, port);

    if ((size_t)port > s_cm_state->devices.size())
    {
        return -1;
    }

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
            {	// Update every 5 seconds
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

static int staticProcessAck(unsigned int port, p_ack_t* ack, unsigned char packetIdentifier)
{
    pfnHandleAckData handler = s_cm_state->binaryAckCallback;
    if (handler != NULLPTR)
    {
        handler(s_cm_state->inertialSenseInterface, ack, packetIdentifier, port);
    }
    return 0;
}

static int staticProcessRxNmea(unsigned int port, const unsigned char* msg, int msgSize)
{
    if ((size_t)port > s_cm_state->devices.size())
    {
        return 0;
    }

    s_cm_state->inertialSenseInterface->ProcessRxNmea(port, msg, msgSize);

    return 0;
}


InertialSense::InertialSense(
        pfnHandleBinaryData    handlerIsb,
        pfnHandleAckData       handlerAck,
        pfnIsCommAsapMsg       handlerRmc,
        pfnIsCommGenMsgHandler handlerNmea,
        pfnIsCommGenMsgHandler handlerUblox,
        pfnIsCommGenMsgHandler handlerRtcm3,
        pfnIsCommGenMsgHandler handlerSpartn ) : m_tcpServer(this)
{
    s_is = this;
    s_cm_state = &m_comManagerState;
    m_logThread = NULLPTR;
    m_lastLogReInit = time(0);
    m_clientStream = NULLPTR;
    m_clientBufferBytesToSend = 0;
    m_clientServerByteCount = 0;
    m_disableBroadcastsOnClose = false;  // For Intel.

    for(int i=0; i<int(sizeof(m_comManagerState.binaryCallback)/sizeof(pfnHandleBinaryData)); i++)
    {
        m_comManagerState.binaryCallback[i] = {};
    }
    m_comManagerState.binaryCallbackGlobal = handlerIsb;
    m_comManagerState.binaryAckCallback = handlerAck;
    m_comManagerState.stepLogFunction = &InertialSense::StepLogger;
    m_comManagerState.inertialSenseInterface = this;
    m_comManagerState.clientBuffer = m_clientBuffer;
    m_comManagerState.clientBufferSize = sizeof(m_clientBuffer);
    m_comManagerState.clientBytesToSend = &m_clientBufferBytesToSend;
    comManagerAssignUserPointer(comManagerGetGlobal(), &m_comManagerState);
    memset(&m_cmInit, 0, sizeof(m_cmInit));
    m_cmPorts = NULLPTR;
    is_comm_init(&m_gpComm, m_gpCommBuffer, sizeof(m_gpCommBuffer));

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
    for (auto& d : m_comManagerState.devices)
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
    m_logger.LogData(device->devLogger, dataSize, data);
}

bool InertialSense::HasReceivedDeviceInfo(size_t index)
{
    if (index >= m_comManagerState.devices.size())
    {
        return false;
    }

    return (
            m_comManagerState.devices[index].devInfo.serialNumber != 0 &&
            m_comManagerState.devices[index].devInfo.manufacturer[0] != 0);
}

bool InertialSense::HasReceivedDeviceInfoFromAllDevices()
{
    if (m_comManagerState.devices.size() == 0)
    {
        return false;
    }

    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        if (!HasReceivedDeviceInfo(i))
        {
            return false;
        }
    }

    return true;
}

void InertialSense::RemoveDevice(size_t index)
{
    if (index >= m_comManagerState.devices.size())
    {
        return;
    }

    serialPortClose(&m_comManagerState.devices[index].serialPort);
}

void InertialSense::LoggerThread(void* info)
{
    bool running = true;
    InertialSense* inertialSense = (InertialSense*)info;

    // gather up packets in memory
    map<int, vector<p_data_buf_t>> packets;

    while (running)
    {
        SLEEP_MS(20);
        {
            // lock so we can read and clear m_logPackets
            cMutexLocker logMutexLocker(&inertialSense->m_logMutex);
            for (map<int, vector<p_data_buf_t>>::iterator i = inertialSense->m_logPackets.begin(); i != inertialSense->m_logPackets.end(); i++)
            {
                packets[i->first] = i->second;
            }

            // clear shared memory
            inertialSense->m_logPackets.clear();

            // update running state
            running = inertialSense->m_logger.Enabled();
        }

        if (running)
        {
            // log the packets
            for (map<int, vector<p_data_buf_t>>::iterator i = packets.begin(); i != packets.end(); i++)
            {
                if (inertialSense->m_logger.Type() != cISLogger::LOGTYPE_RAW) {
                    size_t numPackets = i->second.size();
                    for (size_t j = 0; j < numPackets; j++) {
                        auto device = inertialSense->m_comManagerState.devices[i->first];
                        if (!inertialSense->m_logger.LogData(device.devLogger, &i->second[j].hdr, i->second[j].buf)) {
                            // Failed to write to log
                            SLEEP_MS(20); // FIXME:  This maybe problematic, as it may unnecessarily delay the thread, leading run-away memory usage.
                        }
                    }
                }

                // clear all log data for this pHandle
                i->second.clear();
            }
        }

        inertialSense->m_logger.Update();
    }

	printf("...logger thread terminated.\n");
}

void InertialSense::StepLogger(InertialSense* i, const p_data_t* data, int pHandle)
{
    cMutexLocker logMutexLocker(&i->m_logMutex);
    if (i->m_logger.Enabled())
    {
        p_data_buf_t d;
        d.hdr = data->hdr;
        memcpy(d.buf, data->ptr, d.hdr.size);
        vector<p_data_buf_t>& vec = i->m_logPackets[pHandle];
        vec.push_back(d);
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

        if(rmcPreset)
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
    m_serialServer.Close();

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
 * Returns a vector of available, connected devices
 * @return
 */
std::vector<ISDevice>& InertialSense::getDevices() {
    return m_comManagerState.devices;
}

ISDevice& InertialSense::getDevice(uint32_t deviceIndex) {
    return m_comManagerState.devices[deviceIndex];
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
            for (auto& device : m_comManagerState.devices) {
                if (serialPortIsOpen(&(device.serialPort)) && device.fwUpdate.fwUpdater != nullptr) {
                    if (!device.fwUpdate.update()) {
                        if (device.fwUpdate.lastStatus < fwUpdate::NOT_STARTED) {
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
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        if (!serialPortIsOpen(&m_comManagerState.devices[i].serialPort))
        {
            // Make sure its closed..
            serialPortClose(&m_comManagerState.devices[i].serialPort);
        } else
            anyOpen = true;
    }

    return anyOpen;
}

bool InertialSense::UpdateServer()
{
    // as a tcp server, only the first serial port is read from
    is_comm_instance_t *comm = &(m_gpComm);
    protocol_type_t ptype = _PTYPE_NONE;

    // Get available size of comm buffer.  is_comm_free() modifies comm->rxBuf pointers, call it before using comm->rxBuf.tail.
    int n = is_comm_free(comm);

    // Read data directly into comm buffer
    if ((n = serialPortReadTimeout(&m_comManagerState.devices[0].serialPort, comm->rxBuf.tail, n, 0)))
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
            {	// Record message info
                messageStatsAppend(str, m_serverMessageStats, ptype, comm->rxPkt.id, comm->rxPkt.size, m_timeMs);
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
                    {	// Don't print first error.  Likely due to port having been closed.
                        printf("InertialSense::UpdateClient() PARSE ERROR count: %d\n", error);
                    }
                    error++;
                    break;

                default:
                    break;
            }

            if (ptype != _PTYPE_NONE)
            {	// Record message info
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
    if (OpenSerialPorts(port, baudRate))
    {
        m_disableBroadcastsOnClose = disableBroadcastsOnClose;
        return true;
    }
    return false;
}

bool InertialSense::IsOpen()
{
    return (m_comManagerState.devices.size() != 0 && serialPortIsOpen(&m_comManagerState.devices[0].serialPort));
}

void InertialSense::Close()
{
    EnableLogger(false);
    if (m_disableBroadcastsOnClose)
    {
        StopBroadcasts(false);
        SLEEP_MS(100);
    }
    CloseSerialPorts(true); // allow all opened ports to transmit all buffered data
}

vector<string> InertialSense::GetPorts()
{
    vector<string> ports;
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        ports.push_back(m_comManagerState.devices[i].serialPort.port);
    }
    return ports;
}

void InertialSense::QueryDeviceInfo()
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        comManagerSendRaw((int)i, (uint8_t*)NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE);
    }
}

void InertialSense::StopBroadcasts(bool allPorts)
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        comManagerSendRaw((int)i, (uint8_t*)(allPorts ? NMEA_CMD_STOP_ALL_BROADCASTS_ALL_PORTS : NMEA_CMD_STOP_ALL_BROADCASTS_CUR_PORT), NMEA_CMD_SIZE);
    }
}

void InertialSense::SavePersistent()
{
    // Save persistent messages to flash
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        comManagerSendRaw((int)i, (uint8_t*)NMEA_CMD_SAVE_PERSISTENT_MESSAGES_TO_FLASH, NMEA_CMD_SIZE);
    }
}

void InertialSense::SoftwareReset()
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        comManagerSendRaw((int)i, (uint8_t*)NMEA_CMD_SOFTWARE_RESET, NMEA_CMD_SIZE);
    }
}

void InertialSense::GetData(eDataIDs dataId, uint16_t length, uint16_t offset, uint16_t period)
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        comManagerGetData((int)i, dataId, length, offset, 0);
    }
}

void InertialSense::SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        comManagerSendData((int)i, data, dataId, length, offset);
    }
}

void InertialSense::SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        comManagerSendRawData((int)i, data, dataId, length, offset);
    }
}

void InertialSense::SendRaw(uint8_t* data, uint32_t length)
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        comManagerSendRaw((int)i, data, length);
    }
}

void InertialSense::SetSysCmd(const uint32_t command, int pHandle)
{
    if (pHandle == -1)
    {	// Send to all
        for (size_t port = 0; port < m_comManagerState.devices.size(); port++)
        {
            SetSysCmd(command, (int)port);
        }
    }
    else
    {	// Specific port
        if ((size_t)pHandle >= m_comManagerState.devices.size())
        {
            return;
        }

        m_comManagerState.devices[pHandle].sysCmd.command = command;
        m_comManagerState.devices[pHandle].sysCmd.invCommand = ~command;
        // [C COMM INSTRUCTION]  Update the entire DID_SYS_CMD data set in the IMX.
        comManagerSendData(pHandle, &m_comManagerState.devices[pHandle].sysCmd, DID_SYS_CMD, sizeof(system_command_t), 0);
    }
}

/**
 * Sends message to device to set devices Event Filter
 * param Target: 0 = device, 
 *               1 = forward to device GNSS 1 port (ie GPX), 
 *               2 = forward to device GNSS 2 port (ie GPX),
 *               else will return
 *       pHandle: Send in target COM port. 
 *                If arg is < 0 default port will be used 
*/
void InertialSense::SetEventFilter(int target, uint32_t msgTypeIdMask, uint8_t portMask, int8_t priorityLevel, int pHandle)
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

    if(target == 0)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_FILTER;
    else if(target == 1)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_GNSS1_FILTER;
    else if(target == 2)
        event.msgTypeID = EVENT_MSG_TYPE_ID_ENA_GNSS2_FILTER;
    else 
        return;

    memcpy(data, &event, DID_EVENT_HEADER_SIZE);
    memcpy((void*)(data+DID_EVENT_HEADER_SIZE), &filter, _MIN(sizeof(did_event_filter_t), EVENT_MAX_SIZE-DID_EVENT_HEADER_SIZE));

    if(pHandle < 0)
        SendData(DID_EVENT, data, DID_EVENT_HEADER_SIZE + event.length, 0);
    else    
        comManagerSendData(pHandle, data, DID_EVENT, DID_EVENT_HEADER_SIZE + event.length, 0);
}

void InertialSense::CheckRequestFlashConfig(unsigned int timeMs, unsigned int &uploadTimeMs, bool synced, int port, uint16_t flashCfgDid)
{
    if (uploadTimeMs)
    {	// Upload in progress
        if (timeMs - uploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
        {	// Wait for upload to process.  Pause sync.
            return;
        }
        else
        {	// Upload complete.  Allow sync.
            uploadTimeMs = 0;
        }
    }

    if (!synced)
    {	// Out of sync.  Request flash config.
        comManagerGetData(port, flashCfgDid, 0, 0, 0);
    }
}

// Check if flash config is synchronized and if not request it from the device.
void InertialSense::DeviceSyncFlashCfg(int devIndex, unsigned int timeMs, uint16_t flashCfgDid, uint16_t syncDid, unsigned int &uploadTimeMs, uint32_t &flashCfgChecksum, uint32_t &syncChecksum, uint32_t &uploadChecksum)
{
    if (uploadTimeMs)
    {	// Upload in progress
        if (timeMs - uploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
        {	// Wait for upload to process.  Pause sync.
            syncChecksum = 0xFFFFFFFF;      // invalidate checksum
            return;
        }
    }

    // Require valid sysParams checksum
    if (ValidFlashCfgCksum(syncChecksum))
    {
        if (ValidFlashCfgCksum(flashCfgChecksum) && syncChecksum == flashCfgChecksum)
        {   // Checksum is valid and matches
            if (uploadTimeMs)
            {   // Upload complete.  Allow sync.
                uploadTimeMs = 0;

                if (uploadChecksum == syncChecksum)
                {
                    printf("%s upload complete.\n", cISDataMappings::DataName(flashCfgDid));
                }
                else
                {
                    printf("%s upload rejected.\n", cISDataMappings::DataName(flashCfgDid));
                }
            }
        }
        else
        {	// Out of sync.  Request flash config.
            DEBUG_PRINT("Out of sync.  Requesting %s...\n", cISDataMappings::DataName(flashCfgDid));
            comManagerGetData(devIndex, flashCfgDid, 0, 0, 0);
        }
    } 
    else
    {	// Out of sync.  Request sysParams or gpxStatus.
        DEBUG_PRINT("Out of sync.  Requesting %s...\n", cISDataMappings::DataName(syncDid));
        comManagerGetData(devIndex, syncDid, 0, 0, 0);
    }
}

// This method uses DID_SYS_PARAMS.flashCfgChecksum to determine if the local flash config is synchronized.
void InertialSense::SyncFlashConfig(unsigned int timeMs)
{
    if (timeMs - m_syncCheckTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
    {
        return;
    }
    m_syncCheckTimeMs = timeMs;

    for (int i=0; i<int(m_comManagerState.devices.size()); i++)
    {
        ISDevice& device = m_comManagerState.devices[i];

        if (device.devInfo.hardwareType == IS_HARDWARE_TYPE_IMX)
        {   // Sync IMX flash config if a IMX present
            DeviceSyncFlashCfg(i, timeMs, DID_FLASH_CONFIG,  DID_SYS_PARAMS, device.imxFlashCfgUploadTimeMs, device.imxFlashCfg.checksum, device.sysParams.flashCfgChecksum, device.imxFlashCfgUploadChecksum);
        }

        if (device.devInfo.hardwareType == IS_HARDWARE_TYPE_GPX ||
            device.gpxDevInfo.hardwareType == IS_HARDWARE_TYPE_GPX)
        {   // Sync GPX flash config if a GPX present
            DeviceSyncFlashCfg(i, timeMs, DID_GPX_FLASH_CFG, DID_GPX_STATUS, device.gpxFlashCfgUploadTimeMs, device.gpxFlashCfg.checksum, device.gpxStatus.flashCfgChecksum, device.gpxFlashCfgUploadChecksum);
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

bool InertialSense::ImxFlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle)
{
    if ((size_t)pHandle >= m_comManagerState.devices.size())
    {
        return false;
    }

    ISDevice& device = m_comManagerState.devices[pHandle];

    // Copy flash config
    flashCfg = device.imxFlashCfg;

    // Indicate whether flash config is synchronized
    return ValidFlashCfgCksum(device.imxFlashCfg.checksum) && device.sysParams.flashCfgChecksum == device.imxFlashCfg.checksum;
}

bool InertialSense::GpxFlashConfig(gpx_flash_cfg_t &flashCfg, int pHandle)
{
    if ((size_t)pHandle >= m_comManagerState.devices.size())
    {
        return false;
    }
    ISDevice& device = m_comManagerState.devices[pHandle];

    // Copy flash config
    flashCfg = device.gpxFlashCfg;

    // Indicate whether flash config is synchronized
    return ValidFlashCfgCksum(device.gpxFlashCfg.checksum) && device.gpxStatus.flashCfgChecksum == device.gpxFlashCfg.checksum;
}

bool InertialSense::ImxFlashConfigSynced(int pHandle) 
{ 
    if (m_comManagerState.devices.size() == 0)
    {   // No devices
        return false;
    }

    ISDevice& device = m_comManagerState.devices[pHandle];
    return  ValidFlashCfgCksum(device.imxFlashCfg.checksum) &&
            (device.imxFlashCfg.checksum == device.sysParams.flashCfgChecksum) && 
            (device.imxFlashCfgUploadTimeMs==0) && !ImxFlashConfigUploadFailure(pHandle); 
}

bool InertialSense::GpxFlashConfigSynced(int pHandle) 
{ 
    if (m_comManagerState.devices.size() == 0)
    {   // No devices
        return false;
    }

    ISDevice& device = m_comManagerState.devices[pHandle];
    return  ValidFlashCfgCksum(device.gpxFlashCfg.checksum) &&
            (device.gpxFlashCfg.checksum == device.gpxStatus.flashCfgChecksum) && 
            (device.gpxFlashCfgUploadTimeMs==0) && !GpxFlashConfigUploadFailure(pHandle); 
}

bool InertialSense::ImxFlashConfigUploadFailure(int pHandle)
{ 
    if (m_comManagerState.devices.size() == 0)
    {   // No devices
        return true;
    }

    ISDevice& device = m_comManagerState.devices[pHandle];
    return device.imxFlashCfgUploadChecksum && (device.imxFlashCfgUploadChecksum != device.sysParams.flashCfgChecksum);
} 

bool InertialSense::GpxFlashConfigUploadFailure(int pHandle)
{ 
    if (m_comManagerState.devices.size() == 0)
    {   // No devices
        return true;
    }

    ISDevice& device = m_comManagerState.devices[pHandle];
    return device.gpxFlashCfgUploadChecksum && (device.gpxFlashCfgUploadChecksum != device.gpxStatus.flashCfgChecksum);
}

bool InertialSense::UploadFlashConfigDiff(int pHandle, uint8_t* newData, uint8_t* curData, size_t sizeBytes, uint32_t flashCfgDid, uint32_t& uploadTimeMsOut, uint32_t& checksumOut)
{
    if ((size_t)pHandle >= m_comManagerState.devices.size()) {
        return false;
    }

    std::vector<cISDataMappings::MemoryUsage> usageVec;
    const auto& dataSetMap = *cISDataMappings::NameToInfoMap(flashCfgDid);

    for (const auto& [fieldName, info] : dataSetMap)
    {
        if (info.size == 0) continue;

        // Handle arrays element by element
        size_t elemSize = (info.arraySize > 0) ? info.elementSize : info.size;
        size_t count = (info.arraySize > 0) ? info.arraySize : 1;

        for (size_t i = 0; i < count; ++i)
        {
            uint8_t* newPtr = newData + info.offset + i * elemSize;
            uint8_t* curPtr = curData + info.offset + i * elemSize;

            if (memcmp(newPtr, curPtr, elemSize) != 0)
            {
                cISDataMappings::AppendMemoryUsage(usageVec, newPtr, elemSize);
            }
        }
    }

    bool failure = false;
    for (const cISDataMappings::MemoryUsage& usage : usageVec)
    {
        int offset = static_cast<int>(usage.ptr - newData);
        cout << "Sending " << cISDataMappings::DataName(flashCfgDid) << ": size " << usage.size << ", offset " << offset << endl;
        failure |= comManagerSendData(pHandle, usage.ptr, flashCfgDid, static_cast<int>(usage.size), offset);
        if (!failure)
        {
            uploadTimeMsOut = current_timeMs();
        }
    }

    return !failure;
}

bool InertialSense::SetImxFlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle)
{
    ISDevice& device = m_comManagerState.devices[pHandle];

    // Temporarily clear updateIoConfig for checksum
    bool updateIo = flashCfg.platformConfig & PLATFORM_CFG_UPDATE_IO_CONFIG;
    flashCfg.platformConfig &= ~PLATFORM_CFG_UPDATE_IO_CONFIG;
    UpdateFlashConfigChecksum(flashCfg);
    if (updateIo) flashCfg.platformConfig |= PLATFORM_CFG_UPDATE_IO_CONFIG;

    bool success = UploadFlashConfigDiff(
        pHandle,
        reinterpret_cast<uint8_t*>(&flashCfg),
        reinterpret_cast<uint8_t*>(&device.imxFlashCfg),
        sizeof(nvm_flash_cfg_t),
        DID_FLASH_CONFIG,
        device.imxFlashCfgUploadTimeMs,
        device.imxFlashCfgUploadChecksum
    );

    if (!device.imxFlashCfgUploadTimeMs)
        printf("DID_FLASH_CONFIG in sync.  No upload.\n");
    else
        device.imxFlashCfgUploadChecksum = flashCfg.checksum;

    device.imxFlashCfg = flashCfg;
    return success;
}

bool InertialSense::SetGpxFlashConfig(gpx_flash_cfg_t &flashCfg, int pHandle)
{
    ISDevice& device = m_comManagerState.devices[pHandle];

    bool success = UploadFlashConfigDiff(
        pHandle,
        reinterpret_cast<uint8_t*>(&flashCfg),
        reinterpret_cast<uint8_t*>(&device.gpxFlashCfg),
        sizeof(gpx_flash_cfg_t),
        DID_GPX_FLASH_CFG,
        device.gpxFlashCfgUploadTimeMs,
        device.gpxFlashCfgUploadChecksum
    );

    if (!device.gpxFlashCfgUploadTimeMs)
        printf("DID_GPX_FLASH_CONFIG in sync.  No upload.\n");
    else
        device.gpxFlashCfgUploadChecksum = flashCfg.checksum;

    device.gpxFlashCfg = flashCfg;
    return success;
}

bool InertialSense::WaitForImxFlashCfgSynced(bool forceSync, uint32_t timeout, int pHandle)
{
    if (m_comManagerState.devices.size() == 0)
    {   // No devices
        return false;
    }
    ISDevice& device = m_comManagerState.devices[pHandle];

    if (forceSync)
        device.sysParams.flashCfgChecksum = 0xFFFFFFFF;    // Invalidate to force re-sync

    unsigned int startMs = current_timeMs();
    while(!ImxFlashConfigSynced(pHandle))
    {   // Request and wait for IMX flash config
        Update();
        SLEEP_MS(100);

        if (current_timeMs() - startMs > timeout)
        {   // Timeout waiting for IMX flash config
            printf("Timeout waiting for DID_FLASH_CONFIG failure!\n");

#if PRINT_DEBUG
            ISDevice& device = m_comManagerState.devices[pHandle];
            DEBUG_PRINT("device.imxFlashCfg.checksum:          %u\n", device.imxFlashCfg.checksum);
            DEBUG_PRINT("device.sysParams.flashCfgChecksum:    %u\n", device.sysParams.flashCfgChecksum); 
            DEBUG_PRINT("device.imxFlashCfgUploadTimeMs:       %u\n", device.imxFlashCfgUploadTimeMs);
            DEBUG_PRINT("device.imxFlashCfgUploadChecksum:     %u\n", device.imxFlashCfgUploadChecksum);
#endif
            return false;
        }
        else
        {   // Query DID_SYS_PARAMS
            GetData(DID_SYS_PARAMS);
            DEBUG_PRINT("Waiting for IMX flash sync...\n");
        }
    }

    return ImxFlashConfigSynced(pHandle);
}

bool InertialSense::WaitForGpxFlashCfgSynced(bool forceSync, uint32_t timeout, int pHandle)
{
    if (m_comManagerState.devices.size() == 0)
    {   // No devices
        return false;
    }
    ISDevice& device = m_comManagerState.devices[pHandle];

    if (forceSync)
        device.gpxStatus.flashCfgChecksum = 0xFFFFFFFF;    // Invalidate to force re-sync

    unsigned int startMs = current_timeMs();
    while(!GpxFlashConfigSynced(pHandle))
    {   // Request and wait for GPX flash config
        Update();
        SLEEP_MS(100);

        if (current_timeMs() - startMs > timeout)
        {   // Timeout waiting for GPX flash config
            printf("Timeout waiting for DID_GPX_FLASH_CONFIG failure!\n");

#if PRINT_DEBUG
            ISDevice& device = m_comManagerState.devices[pHandle];
            DEBUG_PRINT("device.gpxFlashCfg.checksum:          %u\n", device.gpxFlashCfg.checksum);
            DEBUG_PRINT("device.gpxStatus.flashCfgChecksum:    %u\n", device.gpxStatus.flashCfgChecksum); 
            DEBUG_PRINT("device.gpxFlashCfgUploadTimeMs:       %u\n", device.gpxFlashCfgUploadTimeMs);
            DEBUG_PRINT("device.gpxFlashCfgUploadChecksum:     %u\n", device.gpxFlashCfgUploadChecksum);
#endif
            return false;
        }
        else
        {   // Query DID_GPX_STATUS
            GetData(DID_GPX_STATUS);
            DEBUG_PRINT("Waiting for GPX flash sync...\n");
        }
    }

    return GpxFlashConfigSynced(pHandle);
}

void InertialSense::ProcessRxData(int pHandle, p_data_t* data)
{
    if (data->hdr.size==0 || data->ptr==NULL)
    {
        return;
    }

    ISDevice& device = m_comManagerState.devices[pHandle];

    switch (data->hdr.id)
    {
        case DID_DEV_INFO:
            device.devInfo = *(dev_info_t*)data->ptr;
            break;
        case DID_GPX_DEV_INFO:
            device.gpxDevInfo = *(dev_info_t*)data->ptr;
            break;
        case DID_SYS_CMD:           device.sysCmd = *(system_command_t*)data->ptr;                          break;
        case DID_SYS_PARAMS:        
            copyDataPToStructP(&device.sysParams, data, sizeof(sys_params_t));      
            DEBUG_PRINT("Received DID_SYS_PARAMS\n");
            break;
        case DID_GPX_STATUS:
            copyDataPToStructP(&device.gpxStatus, data, sizeof(gpx_status_t));
            DEBUG_PRINT("Received DID_GPX_STATUS\n");
            break;
        case DID_FLASH_CONFIG:
            copyDataPToStructP(&device.imxFlashCfg, data, sizeof(nvm_flash_cfg_t));
            if ( dataOverlap( offsetof(nvm_flash_cfg_t, checksum), 4, data ) )
            {	// Checksum received
                device.sysParams.flashCfgChecksum = device.imxFlashCfg.checksum;
            }
            DEBUG_PRINT("Received DID_FLASH_CONFIG\n");
            break;
        case DID_GPX_FLASH_CFG:
            copyDataPToStructP(&device.gpxFlashCfg, data, sizeof(gpx_flash_cfg_t));
            if ( dataOverlap( offsetof(gpx_flash_cfg_t, checksum), 4, data ) )
            {	// Checksum received
                device.gpxStatus.flashCfgChecksum = device.gpxFlashCfg.checksum;
            }
            DEBUG_PRINT("Received DID_GPX_FLASH_CFG\n");
            break;
        case DID_FIRMWARE_UPDATE:
            // we don't respond to messages if we don't already have an active Updater
            if (m_comManagerState.devices[pHandle].fwUpdate.fwUpdater) {
                m_comManagerState.devices[pHandle].fwUpdate.fwUpdater->fwUpdate_processMessage(data->ptr, data->hdr.size);
            }
            break;
    }
}

// return 0 on success, -1 on failure
void InertialSense::ProcessRxNmea(int pHandle, const uint8_t* msg, int msgSize)
{
    if (m_handlerNmea)
    {
        m_handlerNmea(pHandle, msg, msgSize);
    }

    ISDevice& device = m_comManagerState.devices[pHandle];

    switch (getNmeaMsgId(msg, msgSize))
    {
	case NMEA_MSG_ID_INFO:
        {	// IMX device Info
            dev_info_t info;
			nmea_parse_info(info, (const char*)msg, msgSize);
            switch (info.hardwareType)
            {
            case IS_HARDWARE_TYPE_IMX:
                device.devInfo = info;
                break;

            case IS_HARDWARE_TYPE_GPX:
                if (device.devInfo.hardwareType == 0 ||
                    device.devInfo.hardwareType == IS_HARDWARE_TYPE_GPX)
                {   // Populate if device info is not set or GPX
                    device.devInfo = info;
                }
                device.gpxDevInfo = info;
                break;
            }
		}
		break;
	}
}

bool InertialSense::BroadcastBinaryData(int pHandle, uint32_t dataId, int periodMultiple)
{
    if (pHandle >= (int)m_comManagerState.devices.size())
    {
        return false;
    }

    if (periodMultiple < 0) {
        comManagerDisableData(pHandle, dataId);
    } else if (m_comManagerState.devices[pHandle].devInfo.protocolVer[0] == PROTOCOL_VERSION_CHAR0) {
        comManagerGetData(pHandle, dataId, 0, 0, periodMultiple);
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

    if (periodMultiple < 0)
    {
        for (int i = 0; i < (int)m_comManagerState.devices.size(); i++)
        {
            // [C COMM INSTRUCTION]  Stop broadcasting of one specific DID message from the IMX.
            comManagerDisableData(i, dataId);
        }
    }
    else
    {
        for (int i = 0; i < (int)m_comManagerState.devices.size(); i++)
        {
            // [C COMM INSTRUCTION]  3.) Request a specific data set from the IMX.  "periodMultiple" specifies the interval
            // between broadcasts and "periodMultiple=0" will disable broadcasts and transmit one single message.
            if (m_comManagerState.devices[i].devInfo.protocolVer[0] == PROTOCOL_VERSION_CHAR0 || !m_enableDeviceValidation) 
            {
                comManagerGetData(i, dataId, 0, 0, periodMultiple);
            }
        }
    }
    return true;
}

void InertialSense::BroadcastBinaryDataRmcPreset(uint64_t rmcPreset, uint32_t rmcOptions)
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        // [C COMM INSTRUCTION]  Use a preset to enable a predefined set of messages.  R
        comManagerGetDataRmc((int)i, rmcPreset, rmcOptions);
    }
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
        for (int i = 0; i < (int) m_comManagerState.devices.size(); i++) {
            ISDevice& device = m_comManagerState.devices[i];
            device.fwUpdate.fwUpdater = new ISFirmwareUpdater(i, m_comManagerState.devices[i].serialPort.port, &m_comManagerState.devices[i].devInfo);
            device.fwUpdate.fwUpdater->setTarget(targetDevice);

            // TODO: Implement maybe
            device.fwUpdate.fwUpdater->setUploadProgressCb(uploadProgress);
            device.fwUpdate.fwUpdater->setVerifyProgressCb(verifyProgress);
            device.fwUpdate.fwUpdater->setInfoProgressCb(infoProgress);

            device.fwUpdate.fwUpdater->setCommands(cmds);
        }
    }

    printf("\n\r");

#if !PLATFORM_IS_WINDOWS
    fputs("\e[?25h", stdout);	// Turn cursor back on
#endif

    return IS_OP_OK;
}

is_operation_result InertialSense::updateFirmware(
        ISDevice& device,
        fwUpdate::target_t targetDevice,
        std::vector<std::string> cmds,
        ISBootloader::pfnBootloadProgress uploadProgress,
        ISBootloader::pfnBootloadProgress verifyProgress,
        ISBootloader::pfnBootloadStatus infoProgress,
        void (*waitAction)()
)
{
    EnableDeviceValidation(true);
    device.fwUpdate.fwUpdater = new ISFirmwareUpdater(device);
    device.fwUpdate.fwUpdater->setTarget(targetDevice);

    // TODO: Implement maybe
    device.fwUpdate.fwUpdater->setUploadProgressCb(uploadProgress);
    device.fwUpdate.fwUpdater->setVerifyProgressCb(verifyProgress);
    device.fwUpdate.fwUpdater->setInfoProgressCb(infoProgress);

    device.fwUpdate.fwUpdater->setCommands(cmds);

    printf("\n\r");

#if !PLATFORM_IS_WINDOWS
    fputs("\e[?25h", stdout);	// Turn cursor back on
#endif

    return IS_OP_OK;
}

/**
 * @return true if ALL connected devices have finished ALL firmware updates (V2) (no pending commands)
 */
bool InertialSense::isFirmwareUpdateFinished() {
    for (auto& device : m_comManagerState.devices) {
        if (device.fwUpdate.inProgress())
            return false;
    }
    return true;
}

/**
 * @return false if ANY connected devices returned an error from ANY firmware update; but you should first call isFirmwareUpdateFinished()
 */
bool InertialSense::isFirmwareUpdateSuccessful() {
    for (auto device : m_comManagerState.devices) {
        ISFirmwareUpdater *fwUpdater = device.fwUpdate.fwUpdater;
        if (device.fwUpdate.hasError ||
            (   (fwUpdater != nullptr) &&
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
        if (device.fwUpdate.inProgress()) {
            total_percent += device.fwUpdate.percent;
            total_devices++;
        }
    }

    if (total_devices)
        return (int)((float)total_percent / (float)total_devices);

    return 100;
}

/**
 * Gets current update status for selected device index
 * @param deviceIndex
 */
fwUpdate::update_status_e InertialSense::getUpdateStatus(uint32_t deviceIndex)
{
    if (m_comManagerState.devices[deviceIndex].fwUpdate.fwUpdater != NULL)
        return m_comManagerState.devices[deviceIndex].fwUpdate.fwUpdater->fwUpdate_getSessionStatus();
    else
        return fwUpdate::ERR_UPDATER_CLOSED;
}

/**
 * Gets device index from COM port
 * @param COM port
 */
int InertialSense::getUpdateDeviceIndex(const char* com)
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        if (!strcmp(m_comManagerState.devices[i].serialPort.port, com))
            return (int)i;
    }
    return -1;
}

/**
 * Gets current devInfo using device index
 * @param dev_info_t devInfo
 * @param uint32_t deviceIndex
 */
bool InertialSense::getUpdateDevInfo(dev_info_t* devInfo, uint32_t deviceIndex)
{
    if (m_comManagerState.devices[deviceIndex].fwUpdate.fwUpdater != NULL || 1)
    {
        *devInfo = m_comManagerState.devices[deviceIndex].devInfo;
        return true;
    }
    else
    {
        return false; //TODO: This need to be smarter!
    }
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
    for(unsigned int k = 0; k < comPorts.size(); k++)
    {
        char buf[PATH_MAX];
        int newsize = readlink(comPorts[k].c_str(), buf, sizeof(buf)-1);
        if(newsize < 0)
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
        printf("File does not exist");
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
    sort(all_ports.begin(), all_ports.end());
    sort(ports_user_ignore.begin(), ports_user_ignore.end());
    set_difference(
            all_ports.begin(), all_ports.end(),
            ports_user_ignore.begin(), ports_user_ignore.end(),
            back_inserter(update_ports));

    cISBootloaderThread::update(update_ports, forceBootloaderUpdate, baudRate, files, uploadProgress, verifyProgress, infoProgress, waitAction);

    printf("\n\r");

    #if !PLATFORM_IS_WINDOWS
    fputs("\e[?25h", stdout);	// Turn cursor back on
    #endif

    return IS_OP_OK;
}

bool InertialSense::OnClientPacketReceived(const uint8_t* data, uint32_t dataLength)
{
    for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
    {
        // sleep in between to allow test bed to send the serial data
        // TODO: This was 10ms, but that was to long for the CI test.
// 		SLEEP_MS(1);	// This is commented out because it causes problems when using testbad with cltool on single board computer.
        serialPortWrite(&m_comManagerState.devices[i].serialPort, data, dataLength);
    }
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

bool InertialSense::OpenSerialPorts(const char* port, int baudRate)
{
    CloseSerialPorts();

    if (port == NULLPTR || comManagerValidateBaudRate(baudRate) != 0)
    {
        return false;
    }

    // split port on comma in case we need to open multiple serial ports
    vector<string> ports;
    size_t maxCount = UINT32_MAX;

    // handle wildcard, auto-detect serial ports
    if (port[0] == '*')
    {
        // m_enableDeviceValidation = true; // always use device-validation when given the 'all ports' wildcard.    (WHJ) I commented this out.  We don't want to force device verification with the loopback tests.
        cISSerialPort::GetComPorts(ports);
        if (port[1] != '\0')
        {
            maxCount = atoi(port + 1);
            maxCount = (maxCount == 0 ? UINT32_MAX : maxCount);
        }
    }
    else
    {
        // comma separated list of serial ports
        splitString(port, ',', ports);
    }

    // open serial ports
    for (size_t i = 0; i < ports.size(); i++)
    {
        serial_port_t serial;
        serialPortPlatformInit(&serial);
        if (serialPortOpen(&serial, ports[i].c_str(), baudRate, 0) == 0)
        {
            // failed to open
            serialPortClose(&serial);
        }
        else
        {
            ISDevice device;
            device.portHandle = i;
            device.serialPort = serial;
            m_comManagerState.devices.push_back(device);
        }
    }

    // [C COMM INSTRUCTION]  1.) Setup com manager.  Specify number of serial ports and register callback functions for
    // serial port read and write and for successfully parsed data.  Ensure appropriate buffer memory allocation.
    if (m_cmPorts) { delete[] m_cmPorts; }
    m_cmPorts = new com_manager_port_t[m_comManagerState.devices.size()];

    if (m_cmInit.broadcastMsg) { delete[] m_cmInit.broadcastMsg; }
    m_cmInit.broadcastMsgSize = COM_MANAGER_BUF_SIZE_BCAST_MSG(MAX_NUM_BCAST_MSGS);
    m_cmInit.broadcastMsg = new broadcast_msg_t[MAX_NUM_BCAST_MSGS];

    // Register message hander callback functions: RealtimeMessageController (RMC) handler, NMEA, ublox, and RTCM3.
    is_comm_callbacks_t callbacks = {};
    callbacks.rmc   = m_handlerRmc;
    callbacks.nmea  = staticProcessRxNmea;
    callbacks.ublox = m_handlerUblox;
    callbacks.rtcm3 = m_handlerRtcm3;
    callbacks.sprtn = m_handlerSpartn;
    callbacks.error = m_handlerError;
    
    if (comManagerInit((int) m_comManagerState.devices.size(), 10, staticReadData, staticSendData, 0, staticProcessRxData, staticProcessAck, 0, &m_cmInit, m_cmPorts, &callbacks) == -1) {    // Error
        return false;
    }

    bool timeoutOccurred = false;
    if (m_enableDeviceValidation) {
        unsigned int startTime = current_timeMs();
        bool removedSerials = false;

        do {
            for (size_t i = 0; i < m_comManagerState.devices.size(); i++) {
                if ((m_comManagerState.devices[i].serialPort.errorCode == ENOENT) ||
                    (comManagerSendRaw((int) i, (uint8_t *) NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE) != 0)) {
                    // there was some other janky issue with the requested port; even though the device technically exists, its in a bad state. Let's just drop it now.
                    RemoveDevice(i);
                    removedSerials = true;
                }
            }

            SLEEP_MS(100);
            comManagerStep();

            if ((current_timeMs() - startTime) > (uint32_t)m_comManagerState.discoveryTimeout) {
                timeoutOccurred = true;
                break;
            }
        } while (!HasReceivedDeviceInfoFromAllDevices());

        // remove each failed device where communications were not received
        std::vector<std::string> deadPorts;
        for (int i = ((int) m_comManagerState.devices.size() - 1); i >= 0; i--) {
            if (!HasReceivedDeviceInfo(i)) {
                deadPorts.push_back(m_comManagerState.devices[i].serialPort.port);
                RemoveDevice(i);
                removedSerials = true;
            }
        }

        if (timeoutOccurred) {
            fprintf(stderr, "Timeout waiting for response from ports: [");
            for (auto portItr = deadPorts.begin(); portItr != deadPorts.end(); portItr++) {
                fprintf(stderr, "%s%s", (portItr == deadPorts.begin() ? "" : ", "), portItr->c_str());
            }
            fprintf(stderr, "]\n");
            fflush(stderr);
        }

        // if no devices left, all failed, we return failure
        if (m_comManagerState.devices.size() == 0) {
            CloseSerialPorts();
            return false;
        }

        // remove ports if we are over max count
        while (m_comManagerState.devices.size() > maxCount) {
            RemoveDevice(m_comManagerState.devices.size() - 1);
            removedSerials = true;
        }

        // setup com manager again if serial ports dropped out with new count of serial ports
        if (removedSerials) {
            is_comm_callbacks_t callbacks = {};
            callbacks.rmc   = m_handlerRmc;
            callbacks.nmea  = staticProcessRxNmea;
            callbacks.ublox = m_handlerUblox;
            callbacks.rtcm3 = m_handlerRtcm3;
            callbacks.sprtn = m_handlerSpartn;
            callbacks.error = m_handlerError;
            comManagerInit((int) m_comManagerState.devices.size(), 10, staticReadData, staticSendData, 0, staticProcessRxData, 0, 0, &m_cmInit, m_cmPorts, &callbacks);
        }
    }

    // request extended device info for remaining connected devices...
    for (int i = ((int) m_comManagerState.devices.size() - 1); i >= 0; i--) {
        // but only if they are of a compatible protocol version
        if (m_comManagerState.devices[i].devInfo.protocolVer[0] == PROTOCOL_VERSION_CHAR0) {
            comManagerGetData((int) i, DID_SYS_CMD, 0, 0, 0);
            comManagerGetData((int) i, DID_FLASH_CONFIG, 0, 0, 0);
            comManagerGetData((int) i, DID_GPX_FLASH_CFG, 0, 0, 0);
        }
    }

    return m_comManagerState.devices.size() != 0;
}

void InertialSense::CloseSerialPorts(bool drainBeforeClose)
{
    for (auto& device : m_comManagerState.devices)
    {
        if (drainBeforeClose)
            serialPortDrain(&device.serialPort);

        serialPortClose(&device.serialPort);
    }
    m_comManagerState.devices.clear();
}

template<typename T>
bool SaveFlashConfigToFile(const std::string& path, int did, std::function<bool(T&, int)> getCfg, int pHandle)
{
    T flashCfg;
    if (!getCfg(flashCfg, pHandle))
    {
        printf("[ERROR] --- Failed to get flash config\n");
        return false;
    }

    YAML::Node yaml;
    if (!cISDataMappings::DataToYaml(did, reinterpret_cast<const uint8_t*>(&flashCfg), yaml))
    {
        printf("[ERROR] --- Failed to serialize flash config to YAML\n");
        return false;
    }

    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << yaml;

    std::ofstream fout(path);
    fout << emitter.c_str();
    return true;
}

template<typename T>
int LoadFlashConfigFromFile(const std::string& path, int did, std::function<bool(T&, int)> setCfg, int pHandle)
{
    try
    {
        YAML::Node yaml = YAML::LoadFile(path);
        T flashCfg;
        if (!cISDataMappings::YamlToData(did, yaml, reinterpret_cast<uint8_t*>(&flashCfg)))
        {
            printf("[ERROR] --- Failed to parse YAML into flash config structure\n");
            return false;
        }

        setCfg(flashCfg, pHandle);
    }
    catch (const YAML::Exception& ex)
    {
        printf("[ERROR] --- There was an error parsing the YAML file: %s\n", ex.what());
        return false;
    }

    return true;
}

bool InertialSense::SaveImxFlashConfigToFile(std::string path, int pHandle)
{
    return SaveFlashConfigToFile<nvm_flash_cfg_t>(path, DID_FLASH_CONFIG,
        [this](nvm_flash_cfg_t& cfg, int handle) { return ImxFlashConfig(cfg, handle); },
        pHandle);
}

bool InertialSense::SaveGpxFlashConfigToFile(std::string path, int pHandle)
{
    return SaveFlashConfigToFile<gpx_flash_cfg_t>(path, DID_GPX_FLASH_CFG,
        [this](gpx_flash_cfg_t& cfg, int handle) { return GpxFlashConfig(cfg, handle); },
        pHandle);
}

bool InertialSense::LoadImxFlashConfigFromFile(std::string path, int pHandle)
{
    return LoadFlashConfigFromFile<nvm_flash_cfg_t>(path, DID_FLASH_CONFIG,
        [this](nvm_flash_cfg_t& cfg, int handle) { return SetImxFlashConfig(cfg, handle); },
        pHandle);
}

bool InertialSense::LoadGpxFlashConfigFromFile(std::string path, int pHandle)
{
    return LoadFlashConfigFromFile<gpx_flash_cfg_t>(path, DID_GPX_FLASH_CFG,
        [this](gpx_flash_cfg_t& cfg, int handle) { return SetGpxFlashConfig(cfg, handle); },
        pHandle);
}



/**
* Get the device info
* @param pHandle the pHandle to get device info for
* @return the device info
*/
const dev_info_t InertialSense::DeviceInfo(int pHandle)
{
    if ((size_t)pHandle >= m_comManagerState.devices.size())
    {
        pHandle = 0;
    }
    return m_comManagerState.devices[pHandle].devInfo;
}

/**
* Get current device system command
* @param pHandle the pHandle to get sysCmd for
* @return current device system command
*/
system_command_t InertialSense::GetSysCmd(int pHandle)
{
    if ((size_t)pHandle >= m_comManagerState.devices.size())
    {
        pHandle = 0;
    }
    return m_comManagerState.devices[pHandle].sysCmd;
}
