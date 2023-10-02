/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "protocol_nmea.h"
#include <yaml-cpp/yaml.h>
#include "InertialSense.h"
#include "ISBootloaderThread.h"
#include "ISBootloaderDFU.h"
#include "protocol/FirmwareUpdate.h"

using namespace std;

static InertialSense *g_is;
static InertialSense::com_manager_cpp_state_t *s_cm_state;

static int staticSendData(int port, const unsigned char* buf, int len)
{
	if ((size_t)port >= s_cm_state->devices.size())
	{
		return 0;
	}
	return serialPortWrite(&(s_cm_state->devices[port].serialPort), buf, len);
}

static int staticReadData(int port, unsigned char* buf, int len)
{
	if ((size_t)port >= s_cm_state->devices.size())
	{
		return 0;
	}
	return serialPortReadTimeout(&s_cm_state->devices[port].serialPort, buf, len, 1);
}

static void staticProcessRxData(int port, p_data_t* data)
{
	if (data->hdr.id >= (sizeof(s_cm_state->binaryCallback)/sizeof(pfnHandleBinaryData)))
	{
		return;
	}

	pfnHandleBinaryData handler = s_cm_state->binaryCallback[data->hdr.id];
	s_cm_state->stepLogFunction(s_cm_state->inertialSenseInterface, data, port);

	if ((size_t)port > s_cm_state->devices.size())
	{
		return;
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
	}
}

static int staticProcessRxNmea(int port, const unsigned char* msg, int msgSize)
{
	if ((size_t)port > s_cm_state->devices.size())
	{
		return 0;
	}

	s_cm_state->inertialSenseInterface->ProcessRxNmea(port, msg, msgSize);
	
	return 0;
}


InertialSense::InertialSense(
	pfnHandleBinaryData        handlerIsb,
	pfnComManagerAsapMsg       handlerRmc,
	pfnComManagerGenMsgHandler handlerNmea,
	pfnComManagerGenMsgHandler handlerUblox, 
	pfnComManagerGenMsgHandler handlerRtcm3,
	pfnComManagerGenMsgHandler handlerSpartn ) : m_tcpServer(this)
{
	g_is = this;
	s_cm_state = &m_comManagerState;
	m_logThread = NULLPTR;
	m_lastLogReInit = time(0);
	m_clientStream = NULLPTR;
	m_clientBufferBytesToSend = 0;
	m_clientServerByteCount = 0;
    m_disableBroadcastsOnClose = false;
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
}

bool InertialSense::EnableLogging(const string& path, cISLogger::eLogType logType, float maxDiskSpacePercent, uint32_t maxFileSize, const string& subFolder)
{
	cMutexLocker logMutexLocker(&m_logMutex);
	if (!m_logger.InitSaveTimestamp(subFolder, path, cISLogger::g_emptyString, (int)m_comManagerState.devices.size(), logType, maxDiskSpacePercent, maxFileSize, subFolder.length() != 0))
	{
		return false;
	}
	m_logger.EnableLogging(true);
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		m_logger.SetDeviceInfo(&m_comManagerState.devices[i].devInfo);
	}
	if (m_logThread == NULLPTR)
	{
		m_logThread = threadCreateAndStart(&InertialSense::LoggerThread, this);
	}
	return true;
}

void InertialSense::DisableLogging()
{
	// just sets a bool no need to lock
	m_logger.EnableLogging(false);
	threadJoinAndFree(m_logThread);
	m_logThread = NULLPTR;
	m_logger.CloseAllFiles();
}

bool InertialSense::HasReceivedResponseFromDevice(size_t index)
{
	if (index >= m_comManagerState.devices.size())
	{
		return false;
	}

	return (
		m_comManagerState.devices[index].flashCfg.size != 0 &&
		m_comManagerState.devices[index].devInfo.serialNumber != 0 &&
		m_comManagerState.devices[index].devInfo.manufacturer[0] != 0);
}

bool InertialSense::HasReceivedResponseFromAllDevices()
{
	if (m_comManagerState.devices.size() == 0)
	{
		return false;
	}

	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		if (!HasReceivedResponseFromDevice(i))
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
	m_comManagerState.devices.erase(m_comManagerState.devices.begin() + index);
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
				for (size_t j = 0; j < i->second.size(); j++)
				{
					if (!inertialSense->m_logger.LogData(i->first, &i->second[j].hdr, i->second[j].buf))
					{
						// Failed to write to log
						SLEEP_MS(20);
					}
				}

				// clear all log data for this pHandle
				i->second.clear();
			}
		}

		inertialSense->m_logger.Update();
	}

	printf("\n...Logger thread terminated...\n");
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

bool InertialSense::SetLoggerEnabled(
    bool enable, 
    const string& path, 
    cISLogger::eLogType logType, 
    uint64_t rmcPreset, 
    uint32_t rmcOptions,
    float maxDiskSpacePercent, 
    uint32_t maxFileSize, 
    const string& subFolder)
{
	if (enable)
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
		return EnableLogging(path, logType, maxDiskSpacePercent, maxFileSize, subFolder);
	}

	// !enable, shutdown logger gracefully
	DisableLogging();
	return true;
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
            for (size_t devIdx = 0; devIdx < m_comManagerState.devices.size(); devIdx++) {
                if (serialPortIsOpen(&(m_comManagerState.devices[devIdx].serialPort)) && m_comManagerState.devices[devIdx].fwUpdater != nullptr) {
                    ISFirmwareUpdater* fwUpdater = m_comManagerState.devices[devIdx].fwUpdater;
                    fwUpdater->step();

                    fwUpdate::update_status_e status = fwUpdater->getSessionStatus();
                    if ((status == fwUpdate::FINISHED) || ( status < fwUpdate::NOT_STARTED)) {
                        if (status < fwUpdate::NOT_STARTED) {
                            // TODO: Report a REAL error
                            // printf("Error starting firmware update: %s\n", fwUpdater->getSessionStatusName());
                        }

                        // release the FirmwareUpdater
                        delete m_comManagerState.devices[devIdx].fwUpdater;
                        m_comManagerState.devices[devIdx].fwUpdater = nullptr;
						m_comManagerState.devices[devIdx].closeStatus = status;
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
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		if (!serialPortIsOpen(&m_comManagerState.devices[i].serialPort))
		{
			CloseSerialPorts();
			return false;
		}
	}

	return true;
}

bool InertialSense::UpdateServer()
{
	// as a tcp server, only the first serial port is read from
	is_comm_instance_t *comm = &(m_gpComm);
	protocol_type_t ptype = _PTYPE_NONE;

	// Get available size of comm buffer
	int n = is_comm_free(comm);

	// Read data directly into comm buffer
	if ((n = serialPortReadTimeout(&m_comManagerState.devices[0].serialPort, comm->rxBuf.tail, n, 0)))
	{
		// Update comm buffer tail pointer
		comm->rxBuf.tail += n;

		// Search comm buffer for valid packets
		while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
		{
			int id = 0;	// len = 0;
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
					// len = messageStatsGetbitu(comm->rxPkt.data.ptr, 14, 10);
					id = messageStatsGetbitu(comm->rxPkt.data.ptr, 24, 12);
					if ((id == 1029) && (comm->rxPkt.data.size < 1024))
					{
						str = string().assign(reinterpret_cast<char*>(comm->rxPkt.data.ptr + 12), comm->rxPkt.data.size - 12);
					}
				}
				else if (ptype == _PTYPE_UBLOX)
				{
					id = *((uint16_t*)(&comm->rxPkt.data.ptr[2]));
				}
				break;

			case _PTYPE_PARSE_ERROR:
				break;

			case _PTYPE_INERTIAL_SENSE_DATA:
			case _PTYPE_INERTIAL_SENSE_CMD:
				id = comm->rxPkt.hdr.id;
				break;

			case _PTYPE_NMEA:
				{	// Use first four characters before comma (e.g. PGGA in $GPGGA,...)   
					uint8_t *pStart = comm->rxPkt.data.ptr + 2;
					uint8_t *pEnd = std::find(pStart, pStart + 8, ',');
					pStart = _MAX(pStart, pEnd - 8);
					memcpy(&id, pStart, (pEnd - pStart));
				}
				break;

			default:
				break;
			}

			if (ptype != _PTYPE_NONE)
			{	// Record message info
				messageStatsAppend(str, m_serverMessageStats, ptype, id, m_timeMs);
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

	// Get available size of comm buffer
	int n = is_comm_free(comm);

	// Read data directly into comm buffer
	if ((n = m_clientStream->Read(comm->rxBuf.tail, n)))
	{
		// Update comm buffer tail pointer
		comm->rxBuf.tail += n;

		// Search comm buffer for valid packets
		while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
		{
			int id = 0;
			string str;

			switch (ptype)
			{
			case _PTYPE_UBLOX:
			case _PTYPE_RTCM3:
				m_clientServerByteCount += comm->rxPkt.data.size;
				OnClientPacketReceived(comm->rxPkt.data.ptr, comm->rxPkt.data.size);

				if (ptype == _PTYPE_RTCM3)
				{
					id = messageStatsGetbitu(comm->rxPkt.data.ptr, 24, 12);
					if ((id == 1029) && (comm->rxPkt.data.size < 1024))
					{
						str = string().assign(reinterpret_cast<char*>(comm->rxPkt.data.ptr + 12), comm->rxPkt.data.size - 12);
					}
				}
				else if (ptype == _PTYPE_UBLOX)
				{
					id = *((uint16_t*)(&comm->rxPkt.data.ptr[2]));
				}
				break;

			case _PTYPE_PARSE_ERROR:
				if (error)
				{	// Don't print first error.  Likely due to port having been closed.
					printf("InertialSense::UpdateClient() PARSE ERROR count: %d\n", error);
				}
				error++;
				break;

			case _PTYPE_INERTIAL_SENSE_DATA:
			case _PTYPE_INERTIAL_SENSE_CMD:
				id = comm->rxPkt.hdr.id;
				break;

			case _PTYPE_NMEA:
				{	// Use first four characters before comma (e.g. PGGA in $GPGGA,...)   
					uint8_t *pStart = comm->rxPkt.data.ptr + 2;
					uint8_t *pEnd = std::find(pStart, pStart + 8, ',');
					pStart = _MAX(pStart, pEnd - 8);
					memcpy(&id, pStart, (pEnd - pStart));
				}
				break;

			default:
				break;
			}

			if (ptype != _PTYPE_NONE)
			{	// Record message info
				messageStatsAppend(str, m_clientMessageStats, ptype, id, m_timeMs);
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
	SetLoggerEnabled(false);
	if (m_disableBroadcastsOnClose)
	{
		StopBroadcasts();
		SLEEP_MS(100);
	}
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		serialPortClose(&m_comManagerState.devices[i].serialPort);
	}
	m_comManagerState.devices.clear();
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
	uint8_t cmd[11] = NMEA_STR_QUERY_DEVICE_INFO;
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		comManagerSendRaw((int)i, (uint8_t*)&cmd, sizeof(cmd));
	}
}

void InertialSense::StopBroadcasts(bool allPorts)
{
	uint8_t cmdAll[11] = NMEA_STR_STOP_ALL_BROADCASTS_ALL_PORTS; 
	uint8_t cmdCur[11] = NMEA_STR_STOP_ALL_BROADCASTS_CUR_PORT;
	uint8_t *cmd = (allPorts ? cmdAll : cmdCur);
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		comManagerSendRaw((int)i, (uint8_t*)&cmd, sizeof(cmd));
	}
}

void InertialSense::SavePersistent()
{
    // Save persistent messages to flash
	uint8_t cmd[11] = NMEA_STR_SAVE_PERSISTENT_MESSAGES_TO_FLASH;
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		comManagerSendRaw((int)i, (uint8_t*)&cmd, sizeof(cmd));
	}
}

void InertialSense::SoftwareReset()
{
	uint8_t cmd[11] = NMEA_STR_SOFTWARE_RESET;
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		comManagerSendRaw((int)i, (uint8_t*)&cmd, sizeof(cmd));
	}
}

void InertialSense::SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		// [C COMM INSTRUCTION]  4.) Send data to the uINS.  
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
		// [C COMM INSTRUCTION]  Update the entire DID_SYS_CMD data set in the uINS.  
		comManagerSendData(pHandle, &m_comManagerState.devices[pHandle].sysCmd, DID_SYS_CMD, sizeof(system_command_t), 0);
	}
}

// This method uses DID_SYS_PARAMS.flashCfgChecksum to determine if the local flash config is synchronized.
void InertialSense::SyncFlashConfig(unsigned int timeMs)
{
	if (timeMs - m_syncCheckTimeMs > SYNC_FLASH_CFG_CHECK_PERIOD_MS)
	{
		m_syncCheckTimeMs = timeMs;

		for (size_t i=0; i<m_comManagerState.devices.size(); i++)
		{
			is_device_t &device = m_comManagerState.devices[i];

			if (device.flashCfgUploadTimeMs)
			{	// Upload in progress
				if (timeMs - device.flashCfgUploadTimeMs < SYNC_FLASH_CFG_CHECK_PERIOD_MS)
				{	// Wait for upload to process.  Pause sync.
					continue;
				}
				else
				{	// Upload complete.  Allow sync.
					device.flashCfgUploadTimeMs = 0;
				}
			}

			if (device.flashCfg.checksum != device.sysParams.flashCfgChecksum)
			{	// Out of sync.  Request flash config.
				comManagerGetData((int)i, DID_FLASH_CONFIG, 0, 0, 0);
			}
		}
	}
}

bool InertialSense::FlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle)
{
	if ((size_t)pHandle >= m_comManagerState.devices.size())
	{
		pHandle = 0;
	}

	is_device_t &device = m_comManagerState.devices[pHandle];

	// Copy flash config
	flashCfg = device.flashCfg;

	// Indicate whether flash config is sychronized
	return device.sysParams.flashCfgChecksum == device.flashCfg.checksum;
}

int InertialSense::SetFlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle)
{
	if ((size_t)pHandle >= m_comManagerState.devices.size())
	{
		return 0;
	}
	is_device_t &device = m_comManagerState.devices[pHandle];

	// Update checksum 
	flashCfg.checksum = flashChecksum32(&flashCfg, sizeof(nvm_flash_cfg_t));

	device.flashCfg = flashCfg;
	device.flashCfgUploadTimeMs = current_timeMs();						// non-zero indicates upload in progress
	device.sysParams.flashCfgChecksum = device.flashCfg.checksum;

	// [C COMM INSTRUCTION]  Update the entire DID_FLASH_CONFIG data set in the uINS.
	return comManagerSendData(pHandle, &device.flashCfg, DID_FLASH_CONFIG, sizeof(nvm_flash_cfg_t), 0);
}

bool InertialSense::EvbFlashConfig(evb_flash_cfg_t &evbFlashCfg, int pHandle)
{
	if ((size_t)pHandle >= m_comManagerState.devices.size())
	{
		pHandle = 0;
	}

	evbFlashCfg = m_comManagerState.devices[pHandle].evbFlashCfg;

	return true;
}

int InertialSense::SetEvbFlashConfig(evb_flash_cfg_t &evbFlashCfg, int pHandle)
{
	if ((size_t)pHandle >= m_comManagerState.devices.size())
	{
		return 0;
	}
	is_device_t &device = m_comManagerState.devices[pHandle];

	// Update checksum
	evbFlashCfg.checksum = flashChecksum32(&evbFlashCfg, sizeof(evb_flash_cfg_t));

	device.evbFlashCfg = evbFlashCfg;

	// [C COMM INSTRUCTION]  Update the entire DID_FLASH_CONFIG data set in the uINS.  
	return comManagerSendData(pHandle, &device.evbFlashCfg, DID_EVB_FLASH_CFG, sizeof(evb_flash_cfg_t), 0);
}

void InertialSense::ProcessRxData(int pHandle, p_data_t* data)
{
	if (data->hdr.size==0 || data->ptr==NULL)
	{
		return;
	}

	is_device_t &device = m_comManagerState.devices[pHandle];

	switch (data->hdr.id)
	{
	case DID_DEV_INFO:          device.devInfo = *(dev_info_t*)data->ptr;                               break;
	case DID_SYS_CMD:           device.sysCmd = *(system_command_t*)data->ptr;                          break;
	case DID_EVB_FLASH_CFG:     device.evbFlashCfg = *(evb_flash_cfg_t*)data->ptr;                      break;
	case DID_SYS_PARAMS:        copyDataPToStructP(&device.sysParams, data, sizeof(sys_params_t));      break;
	case DID_FLASH_CONFIG:
		copyDataPToStructP(&device.flashCfg, data, sizeof(nvm_flash_cfg_t));
		if ( dataOverlap( offsetof(nvm_flash_cfg_t, checksum), 4, data ) )
		{	// Checksum received
			device.sysParams.flashCfgChecksum = device.flashCfg.checksum;
		}
		break;
    case DID_FIRMWARE_UPDATE:
        // we don't respond to messages if we don't already have an active Updater
        if (m_comManagerState.devices[pHandle].fwUpdater)
            m_comManagerState.devices[pHandle].fwUpdater->processMessage(data->ptr, data->hdr.size);
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

	is_device_t &device = m_comManagerState.devices[pHandle];

	int messageIdUInt = NMEA_MESSAGEID_TO_UINT(msg+1);
	switch (messageIdUInt)
	{
	case NMEA_MSG_UINT_INFO:
		if( memcmp(msg, "$INFO,", 6) == 0)
		{	// IMX device Info
			nmea_parse_info(device.devInfo, (const char*)msg, msgSize);			
		}
		break;
	}
}

bool InertialSense::BroadcastBinaryData(uint32_t dataId, int periodMultiple, pfnHandleBinaryData callback)
{
	if (m_comManagerState.devices.size() == 0 || dataId >= (sizeof(m_comManagerState.binaryCallback)/sizeof(pfnHandleBinaryData)))
	{
		return false;
	}
	else
	{
		m_comManagerState.binaryCallback[dataId] = callback;
	}
	if (periodMultiple < 0)
	{
		for (int i = 0; i < (int)m_comManagerState.devices.size(); i++)
		{
			// [C COMM INSTRUCTION]  Stop broadcasting of one specific DID message from the uINS.
			comManagerDisableData(i, dataId);
		}
	}
	else
	{
		for (int i = 0; i < (int)m_comManagerState.devices.size(); i++)
		{
			// [C COMM INSTRUCTION]  3.) Request a specific data set from the uINS.  "periodMultiple" specifies the interval
			// between broadcasts and "periodMultiple=0" will disable broadcasts and transmit one single message. 
			comManagerGetData(i, dataId, 0, 0, periodMultiple);
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
        int slotNum,
        const string& fileName,
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
        char buff[128];
        printf("File does not exist: [%s] %s", getcwd(buff, sizeof(buff)-1), fileName.c_str());
        return IS_OP_ERROR;
    }

#if !PLATFORM_IS_WINDOWS
    fputs("\e[?25l", stdout);	// Turn off cursor during firmare update
#endif

    printf("\n\r");


    cISSerialPort::GetComPorts(all_ports);

    // Get the list of ports to ignore during the bootloading process
    sort(all_ports.begin(), all_ports.end());
    sort(ports_user_ignore.begin(), ports_user_ignore.end());
    set_difference(
            all_ports.begin(), all_ports.end(),
            ports_user_ignore.begin(), ports_user_ignore.end(),
            back_inserter(update_ports));

    for (int i = 0; i < (int)m_comManagerState.devices.size(); i++) {
        m_comManagerState.devices[i].fwUpdater = new ISFirmwareUpdater(i, m_comManagerState.devices[i].serialPort.port, &m_comManagerState.devices[i].devInfo);
        
		// TODO: Impliment maybe
		// m_comManagerState.devices[i].fwUpdater->setUploadProgressCb(uploadProgress);
		// m_comManagerState.devices[i].fwUpdater->setVerifyProgressCb(verifyProgress);
		// m_comManagerState.devices[i].fwUpdater->setInfoProgressCb(infoProgress);
		
		m_comManagerState.devices[i].fwUpdater->initializeUpdate(targetDevice, fileName, slotNum, false, 512, 250);
    }

    printf("\n\r");

#if !PLATFORM_IS_WINDOWS
    fputs("\e[?25h", stdout);	// Turn cursor back on
#endif

    return IS_OP_OK;
}

/**
* Gets current update status for selected device index
* @param deviceIndex
*/
fwUpdate::update_status_e InertialSense::getUpdateStatus(uint32_t deviceIndex)
{
	try
	{
		if (m_comManagerState.devices[deviceIndex].fwUpdater != NULL)
			return m_comManagerState.devices[deviceIndex].fwUpdater->getSessionStatus();
		else
			return fwUpdate::ERR_UPDATER_CLOSED;

	}
	catch(...)
	{
		return fwUpdate::ERR_INVALID_SLOT;
	}

	return fwUpdate::ERR_UNKOWN;
}

/**
* Gets reason device was closed for selected device index
* @param deviceIndex
*/
fwUpdate::update_status_e InertialSense::getCloseStatus(uint32_t deviceIndex)
{
	try
	{
		return m_comManagerState.devices[deviceIndex].closeStatus;
	}
	catch (...)
	{
		return fwUpdate::ERR_INVALID_SLOT;
	}

	return fwUpdate::ERR_UNKOWN;
}

/**
* Gets current update percent for selected device index
* @param deviceIndex
*/
float InertialSense::getUploadPercent(uint32_t deviceIndex)
{
	float totalChunks;
	if (m_comManagerState.devices[deviceIndex].fwUpdater != NULL)
	{
		totalChunks = m_comManagerState.devices[deviceIndex].fwUpdater->getTotalChunks();

		if (totalChunks > 0)
			return (m_comManagerState.devices[deviceIndex].fwUpdater->getNextChunkID() / totalChunks)*100;
		else
				return 100.0;
	}
	else
		return 100.0; //TODO: This need to be smarter!
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
	if (m_comManagerState.devices[deviceIndex].fwUpdater != NULL || 1)
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
	fputs("\e[?25l", stdout);	// Turn off cursor during firmare update
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
			is_device_t device = {};
			device.serialPort = serial;
			device.sysParams.flashCfgChecksum = 0xFFFFFFFF;		// Invalidate flash config checksum to trigger sync event
			m_comManagerState.devices.push_back(device);
		}
	}

	// [C COMM INSTRUCTION]  1.) Setup com manager.  Specify number of serial ports and register callback functions for
	// serial port read and write and for successfully parsed data.  Ensure appropriate buffer memory allocation.
	if (m_cmPorts) { delete [] m_cmPorts; }
	m_cmPorts = new com_manager_port_t[m_comManagerState.devices.size()];

	if (m_cmInit.broadcastMsg) { delete [] m_cmInit.broadcastMsg; }
	m_cmInit.broadcastMsgSize = COM_MANAGER_BUF_SIZE_BCAST_MSG(MAX_NUM_BCAST_MSGS);
	m_cmInit.broadcastMsg = new broadcast_msg_t[MAX_NUM_BCAST_MSGS];
	if (comManagerInit((int)m_comManagerState.devices.size(), 10, staticReadData, staticSendData, 0, staticProcessRxData, 0, 0, &m_cmInit, m_cmPorts) == -1)
	{	// Error
		return false;
	}
	comManagerSetCallbacks(m_handlerRmc, staticProcessRxNmea, m_handlerUblox, m_handlerRtcm3, m_handlerSpartn);

	if (m_enableDeviceValidation)
	{
		time_t startTime = time(0);

		// Query devices with 10 second timeout
		uint8_t getNmeaInfoBuf[11] = NMEA_STR_QUERY_DEVICE_INFO;
		while (!HasReceivedResponseFromAllDevices() && (time(0) - startTime < 10))
		{
			QueryDeviceInfo();
			for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
			{
				comManagerSendRaw((int)i, (uint8_t*)&getNmeaInfoBuf, sizeof(getNmeaInfoBuf));
				// comManagerGetData((int)i, DID_DEV_INFO,         0, 0, 0);
				comManagerGetData((int)i, DID_SYS_CMD,          0, 0, 0);
				comManagerGetData((int)i, DID_FLASH_CONFIG,     0, 0, 0);
				comManagerGetData((int)i, DID_EVB_FLASH_CFG,    0, 0, 0);
			}

			SLEEP_MS(100);
			comManagerStep();
		}

		bool removedSerials = false;

		// remove each failed device where communications were not received
		for (int i = ((int)m_comManagerState.devices.size() - 1); i >= 0; i--)
		{
			if (!HasReceivedResponseFromDevice(i))
			{
				RemoveDevice(i);
				removedSerials = true;
			}
		}

		// if no devices left, all failed, we return failure
		if (m_comManagerState.devices.size() == 0)
		{
			CloseSerialPorts();
			return false;
		}

		// remove ports if we are over max count
		while (m_comManagerState.devices.size() > maxCount)
		{
			RemoveDevice(m_comManagerState.devices.size()-1);
			removedSerials = true;
		}

		// setup com manager again if serial ports dropped out with new count of serial ports
		if (removedSerials)
		{
			comManagerInit((int)m_comManagerState.devices.size(), 10, staticReadData, staticSendData, 0, staticProcessRxData, 0, 0, &m_cmInit, m_cmPorts);
			comManagerSetCallbacks(m_handlerRmc, staticProcessRxNmea, m_handlerUblox, m_handlerRtcm3, m_handlerSpartn);
		}
	}

    return m_comManagerState.devices.size() != 0;
}

void InertialSense::CloseSerialPorts()
{
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		serialPortClose(&m_comManagerState.devices[i].serialPort);
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
