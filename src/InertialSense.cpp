/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "protocol_nmea.h"
#include "yaml-cpp/yaml.h"
#include "protocol_nmea.h"
#include "InertialSense.h"
#ifndef EXCLUDE_BOOTLOADER
#include "ISBootloaderThread.h"
#include "ISBootloaderDFU.h"
#endif

using namespace std;

static int staticSendData(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len)
{
	InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);
	if ((size_t)pHandle >= s->devices.size())
	{
		return 0;
	}
	return serialPortWrite(&s->devices[pHandle].serialPort, buf, len);
}

static int staticReadData(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len)
{
	InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);
	if ((size_t)pHandle >= s->devices.size())
	{
		return 0;
	}
	return serialPortReadTimeout(&s->devices[pHandle].serialPort, buf, len, 1);
}

static void staticProcessRxData(CMHANDLE cmHandle, int pHandle, p_data_t* data)
{
	InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);

	if (data->hdr.id >= (sizeof(s->binaryCallback)/sizeof(pfnHandleBinaryData)))
	{
		return;
	}

	pfnHandleBinaryData handler = s->binaryCallback[data->hdr.id];
	s->stepLogFunction(s->inertialSenseInterface, data, pHandle);

	if ((size_t)pHandle > s->devices.size())
	{
		return;
	}

	if (handler != NULLPTR)
	{
		handler(s->inertialSenseInterface, data, pHandle);
	}

	pfnHandleBinaryData handlerGlobal = s->binaryCallbackGlobal;
	if (handlerGlobal != NULLPTR)
	{
		// Called for all DID's
		handlerGlobal(s->inertialSenseInterface, data, pHandle);
	}

	s->inertialSenseInterface->ProcessRxData(pHandle, data);

	switch (data->hdr.id)
	{
	case DID_GPS1_POS:
		static time_t lastTime;
		time_t currentTime = time(NULLPTR);
		if (abs(currentTime - lastTime) > 5)
		{	// Update every 5 seconds
			lastTime = currentTime;
			gps_pos_t &gps = *((gps_pos_t*)data->buf);
			if ((gps.status&GPS_STATUS_FIX_MASK) >= GPS_STATUS_FIX_3D)
			{
				*s->clientBytesToSend = nmea_gga(s->clientBuffer, s->clientBufferSize, gps);
			}
		}
	}
}

static int staticProcessRxNmea(CMHANDLE cmHandle, int pHandle, const unsigned char* msg, int msgSize)
{
	InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);

	if ((size_t)pHandle > s->devices.size())
	{
		return 0;
	}

	s->inertialSenseInterface->ProcessRxNmea(pHandle, msg, msgSize);
	
	return 0;
}


InertialSense::InertialSense(
	pfnHandleBinaryData        handlerIsb,
	pfnComManagerAsapMsg       handlerRmc,
	pfnComManagerGenMsgHandler handlerNmea,
	pfnComManagerGenMsgHandler handlerUblox, 
	pfnComManagerGenMsgHandler handlerRtcm3 ) : m_tcpServer(this)
{
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
	m_handlerNmea = handlerNmea;
	comManagerSetCallbacks(handlerRmc, staticProcessRxNmea, handlerUblox, handlerRtcm3);
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
	map<int, vector<p_data_t>> packets;

	while (running)
	{
		SLEEP_MS(20);
		{
			// lock so we can read and clear m_logPackets
			cMutexLocker logMutexLocker(&inertialSense->m_logMutex);
			for (map<int, vector<p_data_t>>::iterator i = inertialSense->m_logPackets.begin(); i != inertialSense->m_logPackets.end(); i++)
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
			for (map<int, vector<p_data_t>>::iterator i = packets.begin(); i != packets.end(); i++)
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
		vector<p_data_t>& vec = i->m_logPackets[pHandle];
		vec.push_back(*data);
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

size_t InertialSense::GetDeviceCount()
{
	return m_comManagerState.devices.size();
}

bool InertialSense::Update()
{
	unsigned int timeMs = current_timeMs();

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

			SyncFlashConfig(timeMs);
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
	if ((n = serialPortReadTimeout(&m_comManagerState.devices[0].serialPort, comm->buf.tail, n, 0)))
	{
		// Update comm buffer tail pointer
		comm->buf.tail += n;

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
				m_clientServerByteCount += comm->dataHdr.size;
				if (m_tcpServer.Write(comm->dataPtr, comm->dataHdr.size) != (int)comm->dataHdr.size)
				{
					cout << endl << "Failed to write bytes to tcp server!" << endl;
				}
				if (ptype == _PTYPE_RTCM3)
				{
					// len = messageStatsGetbitu(comm->dataPtr, 14, 10);
					id = messageStatsGetbitu(comm->dataPtr, 24, 12);
					if ((id == 1029) && (comm->dataHdr.size < 1024))
					{
						str = string().assign(reinterpret_cast<char*>(comm->dataPtr + 12), comm->dataHdr.size - 12);
					}
				}
				else if (ptype == _PTYPE_UBLOX)
				{
					id = *((uint16_t*)(&comm->dataPtr[2]));
				}
				break;

			case _PTYPE_PARSE_ERROR:
				break;

			case _PTYPE_INERTIAL_SENSE_DATA:
			case _PTYPE_INERTIAL_SENSE_CMD:
				id = comm->dataHdr.id;
				break;

			case _PTYPE_NMEA:
				{	// Use first four characters before comma (e.g. PGGA in $GPGGA,...)   
					uint8_t *pStart = comm->dataPtr + 2;
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
				messageStatsAppend(str, m_serverMessageStats, ptype, id, current_timeMs());
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
	if ((n = m_clientStream->Read(comm->buf.tail, n)))
	{
		// Update comm buffer tail pointer
		comm->buf.tail += n;

		// Search comm buffer for valid packets
		while ((ptype = is_comm_parse(comm)) != _PTYPE_NONE)
		{
			int id = 0;
			string str;

			switch (ptype)
			{
			case _PTYPE_UBLOX:
			case _PTYPE_RTCM3:
				m_clientServerByteCount += comm->dataHdr.size;
				OnClientPacketReceived(comm->dataPtr, comm->dataHdr.size);

				if (ptype == _PTYPE_RTCM3)
				{
					id = messageStatsGetbitu(comm->dataPtr, 24, 12);
					if ((id == 1029) && (comm->dataHdr.size < 1024))
					{
						str = string().assign(reinterpret_cast<char*>(comm->dataPtr + 12), comm->dataHdr.size - 12);
					}
				}
				else if (ptype == _PTYPE_UBLOX)
				{
					id = *((uint16_t*)(&comm->dataPtr[2]));
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
				id = comm->dataHdr.id;
				break;

			case _PTYPE_NMEA:
				{	// Use first four characters before comma (e.g. PGGA in $GPGGA,...)   
					uint8_t *pStart = comm->dataPtr + 2;
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
				messageStatsAppend(str, m_clientMessageStats, ptype, id, current_timeMs());
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

void InertialSense::SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		// [C COMM INSTRUCTION]  4.) Send data to the uINS.  
		comManagerSendData((int)i, dataId, data, length, offset);
	}
}

void InertialSense::SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
	{
		comManagerSendRawData((int)i, dataId, data, length, offset);
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
		comManagerSendData(pHandle, DID_SYS_CMD, &m_comManagerState.devices[pHandle].sysCmd, sizeof(system_command_t), 0);
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

bool InertialSense::GetFlashConfig(nvm_flash_cfg_t &flashCfg, int pHandle)
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
	return comManagerSendData(pHandle, DID_FLASH_CONFIG, &device.flashCfg, sizeof(nvm_flash_cfg_t), 0);
}

bool InertialSense::GetEvbFlashConfig(evb_flash_cfg_t &evbFlashCfg, int pHandle)
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
	return comManagerSendData(pHandle, DID_EVB_FLASH_CFG, &device.evbFlashCfg, sizeof(evb_flash_cfg_t), 0);
}

void InertialSense::ProcessRxData(int pHandle, p_data_t* data)
{
	is_device_t &device = m_comManagerState.devices[pHandle];

	switch (data->hdr.id)
	{
	case DID_DEV_INFO:          device.devInfo = *(dev_info_t*)data->buf;                               break;
	case DID_SYS_CMD:           device.sysCmd = *(system_command_t*)data->buf;                          break;
	case DID_EVB_FLASH_CFG:     device.evbFlashCfg = *(evb_flash_cfg_t*)data->buf;                      break;	
	case DID_SYS_PARAMS:        copyDataPToStructP(&device.sysParams, data, sizeof(sys_params_t));      break;
	case DID_FLASH_CONFIG:      
		copyDataPToStructP(&device.flashCfg, data, sizeof(nvm_flash_cfg_t));    
		if ( dataOverlap( offsetof(nvm_flash_cfg_t, checksum), 4, data ) )
		{	// Checksum received
			device.sysParams.flashCfgChecksum = device.flashCfg.checksum;
		}
		break;
	}
}

// return 0 on success, -1 on failure
void InertialSense::ProcessRxNmea(int pHandle, const uint8_t* msg, int msgSize)
{
	if (m_handlerNmea)
	{
		m_handlerNmea(comManagerGetGlobal(), pHandle, msg, msgSize);	
	}

	is_device_t &device = m_comManagerState.devices[pHandle];

    switch (getNmeaMsgId(msg, msgSize))
    {
	case NMEA_MSG_ID_INFO:
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
#ifndef EXCLUDE_BOOTLOADER
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

#endif // EXCLUDE_BOOTLOADER
	
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
#define NUM_ENSURED_PKTS 10
	if (m_cmInit.ensuredPackets) { delete [] m_cmInit.ensuredPackets; }
	m_cmInit.ensuredPacketsSize = COM_MANAGER_BUF_SIZE_ENSURED_PKTS(NUM_ENSURED_PKTS);
	m_cmInit.ensuredPackets = new ensured_pkt_t[NUM_ENSURED_PKTS];
	if (comManagerInit((int)m_comManagerState.devices.size(), NUM_ENSURED_PKTS, 10, 10, staticReadData, staticSendData, 0, staticProcessRxData, 0, 0, &m_cmInit, m_cmPorts) == -1)
	{	// Error
		return false;
	}

	// Register message hander callback functions: RealtimeMessageController (RMC) handler, NMEA, ublox, and RTCM3.
	comManagerSetCallbacks(NULL, staticProcessRxNmea, NULL, NULL);

	if (m_enableDeviceValidation)
	{
		time_t startTime = time(0);
        bool removedSerials = false;

		// Query devices with 10 second timeout
		while (!HasReceivedResponseFromAllDevices() && (time(0) - startTime < 10))
		{
			for (size_t i = 0; i < m_comManagerState.devices.size(); i++)
			{
                if ((m_comManagerState.devices[i].serialPort.errorCode == ENOENT) ||
                    (comManagerSendRaw((int)i, (uint8_t*)NMEA_CMD_QUERY_DEVICE_INFO, NMEA_CMD_SIZE) != 0))
                {
                    // there was some other janky issue with the requested port; even though the device technically exists, its in a bad state. Let's just drop it now.
                    RemoveDevice(i);
                    removedSerials = true, i--;
                }
                else
                {
                    // comManagerGetData((int)i, DID_DEV_INFO,         0, 0, 0);
                    comManagerGetData((int) i, DID_SYS_CMD, 0, 0, 0);
                    comManagerGetData((int) i, DID_FLASH_CONFIG, 0, 0, 0);
                    comManagerGetData((int) i, DID_EVB_FLASH_CFG, 0, 0, 0);
                }
			}

			SLEEP_MS(100);
			comManagerStep();
		}

		// remove each failed device where communications were not received
		for (int i = ((int)m_comManagerState.devices.size() - 1); i >= 0; i--)
		{
			if (!HasReceivedResponseFromDevice(i))
			{
				RemoveDevice(i);
				removedSerials = true, i--;
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
			comManagerInit((int)m_comManagerState.devices.size(), 10, 10, 10, staticReadData, staticSendData, 0, staticProcessRxData, 0, 0, &m_cmInit, m_cmPorts);
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

    map["insDynModel"] 				= (uint16_t)outData->insDynModel;
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
        GetFlashConfig(loaded_flash);

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

        loaded_flash.insDynModel              = (uint8_t)inData["insDynModel"].as<uint16_t>();
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
