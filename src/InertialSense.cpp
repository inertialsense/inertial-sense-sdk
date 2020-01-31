/*
MIT LICENSE

Copyright (c) 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "InertialSense.h"

using namespace std;

typedef struct
{
	bootload_params_t param;
	bool result;
	serial_port_t serial;
	void* thread;
} bootloader_state_t;

static void bootloaderThread(void* state)
{
	bootloader_state_t* s = (bootloader_state_t*)state;
	serialPortOpen(&s->serial, s->serial.port, s->param.baudRate, 1);
	if (!enableBootloader(&s->serial, s->param.baudRate, s->param.error, BOOTLOADER_ERROR_LENGTH, s->param.bootloadEnableCmd))
	{
		serialPortClose(&s->serial);
		s->result = false;
		return;
	}
	s->result = (bootloadFileEx(&s->param) != 0);
	serialPortClose(&s->serial);
}

static void bootloaderUpdateBootloaderThread(void* state)
{
    bootloader_state_t* s = (bootloader_state_t*)state;
    serialPortOpen(&s->serial, s->serial.port, s->param.baudRate, 1);
    s->result = (bootloadUpdateBootloaderEx(&s->param) != 0);
    serialPortClose(&s->serial);
}

static int staticSendPacket(CMHANDLE cmHandle, int pHandle, buffer_t *packet)
{
	// Suppress compiler warnings
	(void)pHandle;
	(void)cmHandle;

	InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);
	if ((size_t)pHandle >= s->serialPorts.size())
	{
		return 0;
	}
	return serialPortWrite(&s->serialPorts[pHandle], packet->buf, packet->size);
}

static int staticReadPacket(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len)
{
	// Suppress compiler warnings
	(void)pHandle;
	(void)cmHandle;

	InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);
	if ((size_t)pHandle >= s->serialPorts.size())
	{
		return 0;
	}
	return serialPortReadTimeout(&s->serialPorts[pHandle], buf, len, 1);
}

static void staticProcessRxData(CMHANDLE cmHandle, int pHandle, p_data_t* data)
{
	(void)cmHandle;

	if (data->hdr.id < DID_COUNT)
	{
		InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);
		pfnHandleBinaryData handler = s->binaryCallback[data->hdr.id];
		s->stepLogFunction(s->inertialSenseInterface, data, pHandle);
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

		// if we got dev info, sysCmd or flash config, set it
		if (data->hdr.id == DID_DEV_INFO)
		{
			s->devInfo[pHandle] = *(dev_info_t*)data->buf;
		}
		else if (data->hdr.id == DID_SYS_CMD)
		{
			s->sysCmd[pHandle] = *(system_command_t*)data->buf;
		}
		else if (data->hdr.id == DID_FLASH_CONFIG)
		{
			s->flashConfig[pHandle] = *(nvm_flash_cfg_t*)data->buf;
		}
		else if (data->hdr.id == DID_GPS1_POS)
		{
			// every 5 seconds, put in a new gps position message
			static time_t nextGpsMessageTime;
			time_t currentTime = time(NULLPTR);
			if (currentTime > nextGpsMessageTime)
			{
				nextGpsMessageTime = currentTime + 5;
				*s->clientBytesToSend = gpsToNmeaGGA((gps_pos_t*)data->buf, s->clientBuffer, s->clientBufferSize);
			}
		}
	}
}

InertialSense::InertialSense(pfnHandleBinaryData callback) : m_tcpServer(this)
{
	m_logThread = NULLPTR;
	m_lastLogReInit = time(0);
	m_clientStream = NULLPTR;
	m_clientBufferBytesToSend = 0;
	m_clientServerByteCount = 0;
    m_disableBroadcastsOnClose = false;
	memset(m_comManagerState.binaryCallback, 0, sizeof(m_comManagerState.binaryCallback));
	m_comManagerState.binaryCallbackGlobal = callback;
	m_comManagerState.stepLogFunction = &InertialSense::StepLogger;
	m_comManagerState.inertialSenseInterface = this;
	m_comManagerState.clientBuffer = m_clientBuffer;
	m_comManagerState.clientBufferSize = sizeof(m_clientBuffer);
	m_comManagerState.clientBytesToSend = &m_clientBufferBytesToSend;
	comManagerAssignUserPointer(comManagerGetGlobal(), &m_comManagerState);
	memset(&m_cmBuffers, 0, sizeof(com_manager_buffers_t));
}

InertialSense::~InertialSense()
{
	CloseServerConnection();
	Close();
}

bool InertialSense::EnableLogging(const string& path, cISLogger::eLogType logType, float maxDiskSpacePercent, uint32_t maxFileSize, const string& subFolder)
{
	cMutexLocker logMutexLocker(&m_logMutex);
	if (!m_logger.InitSaveTimestamp(subFolder, path, cISLogger::g_emptyString, (int)m_comManagerState.serialPorts.size(), logType, maxDiskSpacePercent, maxFileSize, subFolder.length() != 0))
	{
		return false;
	}
	m_logger.EnableLogging(true);
	for (size_t i = 0; i < m_comManagerState.devInfo.size(); i++)
	{
		m_logger.SetDeviceInfo(&m_comManagerState.devInfo[i]);
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
	return (m_comManagerState.flashConfig[index].size != 0 && m_comManagerState.devInfo[index].serialNumber != 0 && m_comManagerState.devInfo[index].manufacturer[0] != 0);
}

bool InertialSense::HasReceivedResponseFromAllDevices()
{
	if (m_comManagerState.devInfo.size() != m_comManagerState.sysCmd.size() || m_comManagerState.sysCmd.size() != m_comManagerState.flashConfig.size())
	{
		return false;
	}

	for (size_t i = 0; i < m_comManagerState.devInfo.size(); i++)
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
	serialPortClose(&m_comManagerState.serialPorts[index]);
	m_comManagerState.serialPorts.erase(m_comManagerState.serialPorts.begin() + index);
	m_comManagerState.devInfo.erase(m_comManagerState.devInfo.begin() + index);
	m_comManagerState.sysCmd.erase(m_comManagerState.sysCmd.begin() + index);
	m_comManagerState.flashConfig.erase(m_comManagerState.flashConfig.begin() + index);
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

bool InertialSense::SetLoggerEnabled(
    bool enable, 
    const string& path, 
    cISLogger::eLogType logType, 
    uint64_t rmcPreset, 
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
			BroadcastBinaryDataRmcPreset(rmcPreset);
		}
		return EnableLogging(path, logType, maxDiskSpacePercent, maxFileSize, subFolder);
	}

	// !enable, shutdown logger gracefully
	DisableLogging();
	return true;
}

bool InertialSense::OpenServerConnection(const string& connectionString)
{
	bool opened = false;

	CloseServerConnection();
	vector<string> pieces;
	splitString(connectionString, ":", pieces);
	if (pieces.size() < 3)
	{
		return opened;
	}

	if (pieces[1] == "SERIAL")
	{
		if (pieces.size() < 4)
		{
			return opened;
		}
		else if ((opened = (m_serialServer.Open(pieces[2].c_str(), atoi(pieces[3].c_str())))))
		{
			m_clientStream = &m_serialServer;
		}
	}
	else
	{
		opened = (m_tcpClient.Open(pieces[1], atoi(pieces[2].c_str())) == 0);
		string url = (pieces.size() > 3 ? pieces[3] : "");
		string user = (pieces.size() > 4 ? pieces[4] : "");
		string pwd = (pieces.size() > 5 ? pieces[5] : "");
		if (url.size() != 0)
		{
			m_tcpClient.HttpGet(url, "Inertial Sense", user, pwd);
		}
	}
	if (opened)
	{
		// configure as RTK rover
		uint32_t cfgBits = RTK_CFG_BITS_GPS1_RTK_ROVER;
		for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
		{
			comManagerSendData((int)i, DID_FLASH_CONFIG, &cfgBits, sizeof(cfgBits), offsetof(nvm_flash_cfg_t, RTKCfgBits));
		}
		if (m_clientStream == NULLPTR)
		{
			m_clientStream = &m_tcpClient;
		}
	}

	return opened;
}

void InertialSense::CloseServerConnection()
{
	m_tcpClient.Close();
	m_tcpServer.Close();
	m_serialServer.Close();
	m_clientStream = NULLPTR;
}

bool InertialSense::CreateHost(const string& ipAndPort)
{
	// if no serial connection, fail
	if (!IsOpen())
	{
		return false;
	}

	CloseServerConnection();
	size_t colon = ipAndPort.find(':', 0);
	if (colon == string::npos)
	{
		return false;
	}
	StopBroadcasts();
	string host = ipAndPort.substr(0, colon);
	string portString = ipAndPort.substr(colon + 1);
	int port = (int)strtol(portString.c_str(), NULLPTR, 10);
	return (m_tcpServer.Open(host, port) == 0);
}

bool InertialSense::IsOpen()
{
	return (m_comManagerState.serialPorts.size() != 0 && serialPortIsOpen(&m_comManagerState.serialPorts[0]));
}

size_t InertialSense::GetDeviceCount()
{
	return m_comManagerState.serialPorts.size();
}

bool InertialSense::Update()
{
	uint8_t buffer[4096];
	if (m_tcpServer.IsOpen() && m_comManagerState.serialPorts.size() != 0)
	{
		// as a tcp server, only the first serial port is read from
		int count = serialPortReadTimeout(&m_comManagerState.serialPorts[0], buffer, sizeof(buffer), 0);
		if (count > 0)
		{
			// forward data on to connected clients
			m_clientServerByteCount += count;
			if (m_tcpServer.Write(buffer, count) != count)
			{
				cout << endl << "Failed to write bytes to tcp server!" << endl;
			}
		}
		m_tcpServer.Update();
	}
	else
	{
		if (m_clientStream != NULLPTR)
		{
			int count = m_clientStream->Read(buffer, sizeof(buffer));
			if (count > 0)
			{
				m_clientServerByteCount += count;
                OnPacketReceived(buffer, count);
            }

			// send data to client if available, i.e. nmea gga pos
			if (m_clientServerByteCount > 0)
			{
				int bytes = m_clientBufferBytesToSend;
				if (bytes != 0)
				{
					m_clientStream->Write(m_clientBuffer, bytes);
					m_clientBufferBytesToSend = 0;
				}
			}
		}
		
		// [C COMM INSTRUCTION]  2.) Update Com Manager at regular interval to send and receive data.  
		// Normally called within a while loop.  Include a thread "sleep" if running on a multi-thread/
		// task system with serial port read function that does NOT incorporate a timeout.   
		if (m_comManagerState.serialPorts.size() != 0)
		{
			comManagerStep();
		}
	}

	// if any serial ports have closed, shutdown
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		if (!serialPortIsOpen(&m_comManagerState.serialPorts[i]))
		{
			Close();
			return false;
		}
	}

	return true;
}

void InertialSense::Close()
{
	SetLoggerEnabled(false);
	if (m_disableBroadcastsOnClose)
	{
		StopBroadcasts();
		SLEEP_MS(100);
	}
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		serialPortClose(&m_comManagerState.serialPorts[i]);
	}
	m_comManagerState.serialPorts.clear();
	m_comManagerState.sysCmd.clear();
	m_comManagerState.devInfo.clear();
	m_comManagerState.flashConfig.clear();
	CloseServerConnection();
}

vector<string> InertialSense::GetPorts()
{
	vector<string> ports;
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		ports.push_back(m_comManagerState.serialPorts[i].port);
	}
	return ports;
}

void InertialSense::StopBroadcasts(bool allPorts)
{
    uint8_t pid = (allPorts ? PID_STOP_BROADCASTS_ALL_PORTS : PID_STOP_BROADCASTS_CURRENT_PORT);

	// Stop all broadcasts
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		// [C COMM INSTRUCTION]  Turns off (disable) all broadcasting and streaming on all ports from the uINS.
		comManagerSend((int)i, pid, 0, 0, 0);
	}
}

void InertialSense::SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		// [C COMM INSTRUCTION]  4.) Send data to the uINS.  
		comManagerSendData((int)i, dataId, data, length, offset);
	}
}

void InertialSense::SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		comManagerSendRawData((int)i, dataId, data, length, offset);
	}
}

void InertialSense::SetSysCmd(const system_command_t& sysCmd, int pHandle)
{
	m_comManagerState.sysCmd[pHandle] = sysCmd;
	// [C COMM INSTRUCTION]  Update the entire DID_SYS_CMD data set in the uINS.  
	comManagerSendData(pHandle, DID_SYS_CMD, &m_comManagerState.sysCmd[pHandle], sizeof(sysCmd), 0);
}

void InertialSense::SetFlashConfig(const nvm_flash_cfg_t& flashConfig, int pHandle)
{
	m_comManagerState.flashConfig[pHandle] = flashConfig;
	// [C COMM INSTRUCTION]  Update the entire DID_FLASH_CONFIG data set in the uINS.  
	comManagerSendData(pHandle, DID_FLASH_CONFIG, &m_comManagerState.flashConfig[pHandle], sizeof(flashConfig), 0);
	Update();
}

bool InertialSense::BroadcastBinaryData(uint32_t dataId, int periodMultiple, pfnHandleBinaryData callback)
{
	if (m_comManagerState.serialPorts.size() == 0 || dataId >= DID_COUNT)
	{
		return false;
	}
	else
	{
		m_comManagerState.binaryCallback[dataId] = callback;
	}
	if (periodMultiple < 0)
	{
		for (int i = 0; i < (int)m_comManagerState.serialPorts.size(); i++)
		{
			// [C COMM INSTRUCTION]  Stop broadcasting of one specific DID message from the uINS.
			comManagerDisableData(i, dataId);
		}
	}
	else
	{
		for (int i = 0; i < (int)m_comManagerState.serialPorts.size(); i++)
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
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		// [C COMM INSTRUCTION]  Use a preset to enable a predefined set of messages.  R 
		comManagerGetDataRmc((int)i, rmcPreset, rmcOptions);
	}
}

vector<InertialSense::bootloader_result_t> InertialSense::BootloadFile(const string& comPort, const string& fileName, int baudRate, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, bool updateBootloader)
{
	return BootloadFile(comPort, fileName, "", baudRate, uploadProgress, verifyProgress, NULLPTR, updateBootloader);
}

vector<InertialSense::bootloader_result_t> InertialSense::BootloadFile(const string& comPort, const string& fileName, const string& bootloaderFileName, int baudRate, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, pfnBootloadStatus infoProgress, bool updateBootloader)
{
	vector<bootloader_result_t> results;
	vector<string> portStrings;
	vector<bootloader_state_t> state;

	if (comPort == "*")
	{
		cISSerialPort::GetComPorts(portStrings);
	}
	else
	{
		splitString(comPort, ",", portStrings);
	}
	sort(portStrings.begin(), portStrings.end());
	state.resize(portStrings.size());

	// test file exists
	{
		ifstream tmpStream(fileName);
		if (!tmpStream.good())
		{
			for (size_t i = 0; i < state.size(); i++)
			{
				results.push_back({ state[i].serial.port, "File does not exist" });
			}
		}
	}

	if (results.size() == 0)
	{
		// for each port requested, setup a thread to do the bootloader for that port
		for (size_t i = 0; i < state.size(); i++)
		{
            memset(state[i].param.error, 0, BOOTLOADER_ERROR_LENGTH);
			serialPortPlatformInit(&state[i].serial);
			serialPortSetPort(&state[i].serial, portStrings[i].c_str());
			state[i].param.uploadProgress = uploadProgress;
			state[i].param.verifyProgress = verifyProgress;
			state[i].param.statusText = infoProgress;
			state[i].param.fileName = fileName.c_str();
			state[i].param.bootName = bootloaderFileName.c_str();
			state[i].param.port = &state[i].serial;
			state[i].param.verifyFileName = NULLPTR;
			state[i].param.flags.bitFields.enableVerify = (verifyProgress != NULLPTR);
            state[i].param.numberOfDevices = (int)state.size();
            state[i].param.baudRate = baudRate;
            strncpy(state[i].param.bootloadEnableCmd, "BLEN", 4);
            if (updateBootloader)
            {   // Update bootloader firmware
                state[i].thread = threadCreateAndStart(bootloaderUpdateBootloaderThread, &state[i]);
            }
            else
            {   // Update application firmware
                state[i].thread = threadCreateAndStart(bootloaderThread, &state[i]);
            }
		}

		// wait for all threads to finish
		for (size_t i = 0; i < state.size(); i++)
		{
			threadJoinAndFree(state[i].thread);
		}

		// if any thread failed, we return failure
		for (size_t i = 0; i < state.size(); i++)
		{
			results.push_back({ state[i].serial.port, state[i].param.error });
		}
	}

	return results;
}

bool InertialSense::OnPacketReceived(const uint8_t* data, uint32_t dataLength)
{
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		// sleep in between to allow test bed to send the serial data
		// TODO: This was 10ms, but that was to long for thr CI test.
		SLEEP_MS(1);
		serialPortWrite(&m_comManagerState.serialPorts[i], data, dataLength);
	}
	return false; // do not parse, since we are just forwarding it on
}

void InertialSense::OnClientConnecting(cISTcpServer* server)
{
	(void)server;
	cout << endl << "Client connecting..." << endl;
}

void InertialSense::OnClientConnected(cISTcpServer* server, socket_t socket)
{
	cout << endl << "Client connected: " << (int)socket << endl;
}

void InertialSense::OnClientConnectFailed(cISTcpServer* server)
{
	cout << endl << "Client connection failed!" << endl;
}

void InertialSense::OnClientDisconnected(cISTcpServer* server, socket_t socket)
{
	cout << endl << "Client disconnected: " << (int)socket << endl;
}

bool InertialSense::OpenSerialPorts(const char* port, int baudRate)
{
	Close();

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
		splitString(port, ",", ports);
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
			m_comManagerState.serialPorts.push_back(serial);
		}
	}

	// [C COMM INSTRUCTION]  1.) Setup com manager.  Specify number of serial ports and register callback functions for
	// serial port read and write and for successfully parsed data.  Ensure appropriate buffer memory allocation.
	if (m_cmBuffers.ringBuffer != NULLPTR){	delete m_cmBuffers.ringBuffer; }
	if (m_cmBuffers.status != NULLPTR){	delete m_cmBuffers.status; }
	if (m_cmBuffers.broadcastMsg != NULLPTR){ delete m_cmBuffers.broadcastMsg; }
	m_cmBuffers.ringBufferSize = (uint32_t)COM_MANAGER_BUF_SIZE_RING_BUFFER(m_comManagerState.serialPorts.size());
	m_cmBuffers.ringBuffer = new ring_buffer_t[m_comManagerState.serialPorts.size()];
	m_cmBuffers.statusSize = (uint32_t)COM_MANAGER_BUF_SIZE_STATUS(m_comManagerState.serialPorts.size());
	m_cmBuffers.status = new com_manager_status_t[m_comManagerState.serialPorts.size()];
	m_cmBuffers.broadcastMsgSize = COM_MANAGER_BUF_SIZE_BCAST_MSG(MAX_NUM_BCAST_MSGS);
	m_cmBuffers.broadcastMsg = new broadcast_msg_t[MAX_NUM_BCAST_MSGS];
#define NUM_ENSURED_PKTS 10
	m_cmBuffers.ensuredPacketsSize = COM_MANAGER_BUF_SIZE_ENSURED_PKTS(NUM_ENSURED_PKTS);
	m_cmBuffers.ensuredPackets = new ensured_pkt_t[NUM_ENSURED_PKTS];
	if (comManagerInit((int)m_comManagerState.serialPorts.size(), NUM_ENSURED_PKTS, 10, 10, staticReadPacket, staticSendPacket, 0, staticProcessRxData, 0, 0, &m_cmBuffers) == -1)
	{	// Error
		return false;
	}

	// re-initialize data sets
	system_command_t configTemplate;
	memset(&configTemplate, 0, sizeof(configTemplate));
	m_comManagerState.sysCmd.resize(m_comManagerState.serialPorts.size(), configTemplate);
	dev_info_t devInfoTemplate;
	memset(&devInfoTemplate, 0, sizeof(devInfoTemplate));
	m_comManagerState.devInfo.resize(m_comManagerState.serialPorts.size(), devInfoTemplate);
	nvm_flash_cfg_t flashTemplate;
	memset(&flashTemplate, 0, sizeof(flashTemplate));
	m_comManagerState.flashConfig.resize(m_comManagerState.serialPorts.size(), flashTemplate);

	// negotiate baud rate by querying device info - don't return out until it negotiates or times out
	// if the baud rate is already correct, the request for the message should succeed very quickly
	time_t startTime = time(0);

	// try to auto-baud for up to 10 seconds, then abort if we didn't get a valid packet
	// we wait until we get a valid serial number and manufacturer
	while (!HasReceivedResponseFromAllDevices() && time(0) - startTime < 10)
	{
		for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
		{
			comManagerGetData((int)i, DID_SYS_CMD, 0, 0, 0);
			comManagerGetData((int)i, DID_DEV_INFO, 0, 0, 0);
			comManagerGetData((int)i, DID_FLASH_CONFIG, 0, 0, 0);
		}

		SLEEP_MS(13);
		comManagerStep();
	}

	bool removedSerials = false;

	// remove each failed device where communications were not received
	for (int i = (int)(m_comManagerState.devInfo.size() - 1); i >= 0; i--)
	{
		if (!HasReceivedResponseFromDevice(i))
		{
			RemoveDevice(i);
			removedSerials = true;
		}
	}

	// if no devices left, all failed, we return failure
	if (m_comManagerState.serialPorts.size() == 0)
	{
		Close();
		return false;
	}

	// remove ports if we are over max count
	while (m_comManagerState.serialPorts.size() > maxCount)
	{
		RemoveDevice(m_comManagerState.serialPorts.size() - 1);
		removedSerials = true;
	}

	// setup com manager again if serial ports dropped out with new count of serial ports
	if (removedSerials)
	{
		comManagerInit((int)m_comManagerState.serialPorts.size(), 10, 10, 10, staticReadPacket, staticSendPacket, 0, staticProcessRxData, 0, 0, &m_cmBuffers);
	}

    return m_comManagerState.serialPorts.size() != 0;
}
