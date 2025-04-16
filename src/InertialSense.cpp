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

// This method uses DID_SYS_PARAMS.flashCfgChecksum to determine if the local flash config is synchronized.
void InertialSense::SyncFlashConfig(unsigned int timeMs)
{
    if (timeMs - m_syncCheckTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
    {
        return;
    }
    m_syncCheckTimeMs = timeMs;

    for (size_t i=0; i<m_comManagerState.devices.size(); i++)
    {
        ISDevice& device = m_comManagerState.devices[i];

        if (device.flashCfgUploadTimeMs)
        {	// Upload in progress
            if (timeMs - device.flashCfgUploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
            {	// Wait for upload to process.  Pause sync.
                device.sysParams.flashCfgChecksum = 0;
            }
        }

        // Require valid sysParams checksum
        if (device.sysParams.flashCfgChecksum)  
        {   
            if (device.sysParams.flashCfgChecksum == device.flashCfg.checksum)
            {
                if (device.flashCfgUploadTimeMs)
                {   // Upload complete.  Allow sync.
                    device.flashCfgUploadTimeMs = 0;

                    if (device.flashCfgUploadChecksum == device.sysParams.flashCfgChecksum)
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
            {	// Out of sync.  Request flash config.
                DEBUG_PRINT("Out of sync.  Requesting DID_FLASH_CONFIG...\n");
                comManagerGetData((int)i, DID_FLASH_CONFIG, 0, 0, 0);
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

bool InertialSense::FlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle)
{
    if ((size_t)pHandle >= m_comManagerState.devices.size())
    {
        pHandle = 0;
    }

    ISDevice& device = m_comManagerState.devices[pHandle];

    // Copy flash config
    flashCfg = device.flashCfg;

    // Indicate whether flash config is synchronized
    return device.sysParams.flashCfgChecksum == device.flashCfg.checksum;
}

bool InertialSense::SetFlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle)
{
    if ((size_t)pHandle >= m_comManagerState.devices.size())
    {
        return 0;
    }
    ISDevice& device = m_comManagerState.devices[pHandle];

    device.flashCfg.checksum = flashChecksum32(&device.flashCfg, sizeof(nvm_flash_cfg_t));

    // Iterate over and upload flash config in 4 byte segments.  Upload only contiguous segments of mismatched data starting at `key` (i = 2).  Don't upload size or checksum.
    static_assert(sizeof(nvm_flash_cfg_t) % 4 == 0, "Size of nvm_flash_cfg_t must be a 4 bytes in size!!!");
    uint32_t *newCfg = (uint32_t*)&flashCfg;
    uint32_t *curCfg = (uint32_t*)&device.flashCfg; 
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
            
            DEBUG_PRINT("Sending DID_FLASH_CONFIG: size %d, offset %d\n", size, offset);
            int fail = comManagerSendData(pHandle, head, DID_FLASH_CONFIG, size, offset);            
            failure = failure || fail;
            device.flashCfgUploadTimeMs = current_timeMs();						// non-zero indicates upload in progress
        }
    }

    if (device.flashCfgUploadTimeMs == 0)
    {
        printf("DID_FLASH_CONFIG in sync.  No upload.\n");
    }

    // Update checksum
    UpdateFlashConfigChecksum(flashCfg);

    // Save checksum to ensure upload happened correctly
    if (device.flashCfgUploadTimeMs)
    {
        device.flashCfgUploadChecksum = flashCfg.checksum;
    }

    // Update local copy of flash config
    device.flashCfg = flashCfg;

    // Success
    return !failure;
}

bool InertialSense::WaitForFlashSynced(int pHandle)
{
    unsigned int startMs = current_timeMs();
    while(!FlashConfigSynced(pHandle))
    {   // Request and wait for flash config
        Update();
        SLEEP_MS(100);

        if (current_timeMs() - startMs > 3000)
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
    }

    return FlashConfigSynced(pHandle);
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
        case DID_DEV_INFO:          device.devInfo = *(dev_info_t*)data->ptr;                               break;
        case DID_SYS_CMD:           device.sysCmd = *(system_command_t*)data->ptr;                          break;
        case DID_SYS_PARAMS:        
            copyDataPToStructP(&device.sysParams, data, sizeof(sys_params_t));      
            DEBUG_PRINT("Received DID_SYS_PARAMS\n");
            break;
        case DID_FLASH_CONFIG:
            copyDataPToStructP(&device.flashCfg, data, sizeof(nvm_flash_cfg_t));
            if ( dataOverlap( offsetof(nvm_flash_cfg_t, checksum), 4, data ) )
            {	// Checksum received
                device.sysParams.flashCfgChecksum = device.flashCfg.checksum;
            }
            DEBUG_PRINT("Received DID_FLASH_CONFIG\n");
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
			nmea_parse_info(device.devInfo, (const char*)msg, msgSize);			
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
* @param dev_info_t devI
* @param uint32_t deviceIndex
*/
bool InertialSense::getUpdateDevInfo(dev_info_t* devI, uint32_t deviceIndex)
{
    if (m_comManagerState.devices[deviceIndex].fwUpdate.fwUpdater != NULL || 1)
    {
        memcpy(devI, &m_comManagerState.devices[deviceIndex].devInfo, sizeof(dev_info_t));
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

    #if !PLATFORM_IS_WINDOWS
    fputs("\e[?25l", stdout);	// Turn off cursor during firmware update
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
            device.sysParams.flashCfgChecksum = 0xFFFFFFFF;		// Invalidate flash config checksum to trigger sync event
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
    
    if (comManagerInit((int) m_comManagerState.devices.size(), 10, staticReadData, staticSendData, 0, staticProcessRxData, 0, 0, &m_cmInit, m_cmPorts, &callbacks) == -1) {    // Error
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
            comManagerGetData((int) i, DID_EVB_FLASH_CFG, 0, 0, 0);
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

void InertialSense::SaveFlashConfigFile(std::string path, int pHandle)
{
    nvm_flash_cfg_t* outData = &m_comManagerState.devices[pHandle].flashCfg;

    YAML::Node map = YAML::Node(YAML::NodeType::Map);

    map["size"] 					= outData->size;
    map["checksum"] 				= outData->checksum;
    map["key"] 						= outData->key;
    map["startupImuDtMs"] 			= outData->startupImuDtMs;
    map["startupNavDtMs"] 			= outData->startupNavDtMs;
    map["ser0BaudRate"] 			= outData->ser0BaudRate;
    map["ser1BaudRate"] 			= outData->ser1BaudRate;

    YAML::Node insRotation = YAML::Node(YAML::NodeType::Sequence);
    insRotation.push_back(outData->insRotation[0]);
    insRotation.push_back(outData->insRotation[1]);
    insRotation.push_back(outData->insRotation[2]);
    map["insRotation"] 				= insRotation;

    YAML::Node insOffset = YAML::Node(YAML::NodeType::Sequence);
    insOffset.push_back(outData->insOffset[0]);
    insOffset.push_back(outData->insOffset[1]);
    insOffset.push_back(outData->insOffset[2]);
    map["insOffset"] 				= insOffset;

    YAML::Node gps1AntOffset = YAML::Node(YAML::NodeType::Sequence);
    gps1AntOffset.push_back(outData->gps1AntOffset[0]);
    gps1AntOffset.push_back(outData->gps1AntOffset[1]);
    gps1AntOffset.push_back(outData->gps1AntOffset[2]);
    map["gps1AntOffset"] 			= gps1AntOffset;

    map["dynamicModel"] 				= (uint16_t)outData->dynamicModel;
    map["debug"] 					= (uint16_t)outData->debug;
    map["gnssSatSigConst"] 			= outData->gnssSatSigConst;
    map["sysCfgBits"] 				= outData->sysCfgBits;

    YAML::Node refLla = YAML::Node(YAML::NodeType::Sequence);
    refLla.push_back(outData->refLla[0]);
    refLla.push_back(outData->refLla[1]);
    refLla.push_back(outData->refLla[2]);
    map["refLla"] 					= refLla;

    YAML::Node lastLla = YAML::Node(YAML::NodeType::Sequence);
    lastLla.push_back(outData->lastLla[0]);
    lastLla.push_back(outData->lastLla[1]);
    lastLla.push_back(outData->lastLla[2]);
    map["lastLla"] 					= lastLla;

    map["lastLlaTimeOfWeekMs"] 		= outData->lastLlaTimeOfWeekMs;
    map["lastLlaWeek"] 				= outData->lastLlaWeek;
    map["lastLlaUpdateDistance"] 	= outData->lastLlaUpdateDistance;
    map["ioConfig"] 				= outData->ioConfig;
    map["platformConfig"] 			= outData->platformConfig;

    YAML::Node gps2AntOffset = YAML::Node(YAML::NodeType::Sequence);
    gps2AntOffset.push_back(outData->gps2AntOffset[0]);
    gps2AntOffset.push_back(outData->gps2AntOffset[1]);
    gps2AntOffset.push_back(outData->gps2AntOffset[2]);
    map["gps2AntOffset"] 			= gps2AntOffset;

    YAML::Node zeroVelRotation = YAML::Node(YAML::NodeType::Sequence);
    zeroVelRotation.push_back(outData->zeroVelRotation[0]);
    zeroVelRotation.push_back(outData->zeroVelRotation[1]);
    zeroVelRotation.push_back(outData->zeroVelRotation[2]);
    map["zeroVelRotation"] 			= zeroVelRotation;

    YAML::Node zeroVelOffset = YAML::Node(YAML::NodeType::Sequence);
    zeroVelOffset.push_back(outData->zeroVelOffset[0]);
    zeroVelOffset.push_back(outData->zeroVelOffset[1]);
    zeroVelOffset.push_back(outData->zeroVelOffset[2]);
    map["zeroVelOffset"] 			= zeroVelOffset;

    map["gpsTimeUserDelay"] 		= outData->gpsTimeUserDelay;
    map["magDeclination"] 			= outData->magDeclination;
    map["gpsTimeSyncPeriodMs"] 		= outData->gpsTimeSyncPeriodMs;
    map["startupGPSDtMs"] 			= outData->startupGPSDtMs;
    map["RTKCfgBits"] 				= outData->RTKCfgBits;
    map["sensorConfig"] 			= outData->sensorConfig;
    map["gpsMinimumElevation"] 		= outData->gpsMinimumElevation;
    map["ser2BaudRate"] 			= outData->ser2BaudRate;

    YAML::Node wheelCfgTransE_b2w 	= YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[0]);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[1]);
    wheelCfgTransE_b2w.push_back(outData->wheelConfig.transform.e_b2w[2]);
    map["wheelCfgTransE_b2w"] 		= wheelCfgTransE_b2w;

    YAML::Node wheelCfgTransE_b2wsig = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[0]);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[1]);
    wheelCfgTransE_b2wsig.push_back(outData->wheelConfig.transform.e_b2w_sigma[2]);
    map["wheelCfgTransE_b2wsig"] 	= wheelCfgTransE_b2wsig;

    YAML::Node wheelCfgTransT_b2w = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[0]);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[1]);
    wheelCfgTransT_b2w.push_back(outData->wheelConfig.transform.t_b2w[2]);
    map["wheelCfgTransT_b2w"] 	= wheelCfgTransT_b2w;

    YAML::Node wheelCfgTransT_b2wsig = YAML::Node(YAML::NodeType::Sequence);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[0]);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[1]);
    wheelCfgTransT_b2wsig.push_back(outData->wheelConfig.transform.t_b2w_sigma[2]);
    map["wheelCfgTransT_b2wsig"] 	= wheelCfgTransT_b2wsig;

    map["wheelConfigTrackWidth"] 	= outData->wheelConfig.track_width;
    map["wheelConfigRadius"] 		= outData->wheelConfig.radius;
    map["wheelConfigBits"] 			= outData->wheelConfig.bits;

    std::ofstream fout(path);

    YAML::Emitter emitter;
    emitter.SetSeqFormat(YAML::Flow);
    emitter << map;
    fout << emitter.c_str();
    fout.close();
}

int InertialSense::LoadFlashConfig(std::string path, int pHandle)
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

        loaded_flash.dynamicModel              = (uint8_t)inData["dynamicModel"].as<uint16_t>();
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
