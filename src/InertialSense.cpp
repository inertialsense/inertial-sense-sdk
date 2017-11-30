/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

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
	serialPortOpen(&s->serial, s->serial.port, IS_BAUD_RATE_BOOTLOADER, 1);
	if (!enableBootloader(&s->serial, s->param.error, s->param.errorLength))
	{
		serialPortClose(&s->serial);
		s->result = false;
		return;
	}
	s->result = (bootloadFileEx(&s->param) != 0);
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
	return serialPortRead(&s->serialPorts[pHandle], buf, len);
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

		// if we got dev info, config or flash config, set it
		if (data->hdr.id == DID_DEV_INFO)
		{
			s->devInfo[pHandle] = *(dev_info_t*)data->buf;
		}
		else if (data->hdr.id == DID_CONFIG)
		{
			s->config[pHandle] = *(config_t*)data->buf;
		}
		else if (data->hdr.id == DID_FLASH_CONFIG)
		{
			s->flashConfig[pHandle] = *(nvm_flash_cfg_t*)data->buf;
		}
	}
}

InertialSense::InertialSense(pfnHandleBinaryData callback) : m_serialServer(NULL, true, 0, false)
{
	m_logThread = NULLPTR;
	m_lastLogReInit = time(0);
	m_parser = NULLPTR;
	m_clientStream = NULLPTR;
	m_clientServerByteCount = 0;
	m_disableBroadcastsOnClose = false;
	memset(m_comManagerState.binaryCallback, 0, sizeof(m_comManagerState.binaryCallback));
	m_comManagerState.stepLogFunction = &InertialSense::StepLogger;
	m_comManagerState.inertialSenseInterface = this;
	comManagerAssignUserPointer(getGlobalComManager(), &m_comManagerState);

#if defined(ENABLE_IS_PYTHON_WRAPPER)

	m_comManagerState.binaryCallbackGlobal = (callback == NULLPTR ? NULLPTR : (pfnHandleBinaryData)pyClt_dataCallback);

#else

	m_comManagerState.binaryCallbackGlobal = callback;

#endif

}

InertialSense::~InertialSense()
{
	CloseServerConnection();
	Close();
}

bool InertialSense::EnableLogging(const string& path, float maxDiskSpacePercent, uint32_t maxFileSize, uint32_t chunkSize, const string& subFolder)
{
	cMutexLocker logMutexLocker(&m_logMutex);
	if (!m_logger.InitSaveTimestamp(subFolder, path, cISLogger::g_emptyString, (int)m_comManagerState.serialPorts.size(), cISLogger::LOGTYPE_DAT, maxDiskSpacePercent, maxFileSize, chunkSize, subFolder.length() != 0))
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
	if (m_comManagerState.devInfo.size() != m_comManagerState.config.size() || m_comManagerState.config.size() != m_comManagerState.flashConfig.size())
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
	m_comManagerState.config.erase(m_comManagerState.config.begin() + index);
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
		SLEEP_MS(2);
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
	}
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
	m_disableBroadcastsOnClose = false;
	if (OpenSerialPorts(port, baudRate))
	{
		m_disableBroadcastsOnClose = disableBroadcastsOnClose;
		return true;
	}
	return false;
}

bool InertialSense::SetLoggerEnabled(bool enable, const string& path, uint32_t logSolution, float maxDiskSpacePercent, uint32_t maxFileSize, uint32_t chunkSize, const string& subFolder)
{
	if (enable)
	{
		if (m_logThread != NULLPTR)
		{
			// already logging
			return true;
		}
		SetSolutionStream(logSolution);
		return EnableLogging(path, maxDiskSpacePercent, maxFileSize, chunkSize, subFolder);
	}

	// !enable, shutdown logger gracefully
	DisableLogging();
	return true;
}

void InertialSense::SetSolutionStream(uint32_t logSolution)
{
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		m_comManagerState.config[i].solStreamCtrl = logSolution;
		sendDataComManager((int)i, DID_CONFIG, &m_comManagerState.config[i].solStreamCtrl, sizeof(m_comManagerState.config[i].solStreamCtrl), OFFSETOF(config_t, solStreamCtrl));
	}
}

bool InertialSense::OpenServerConnection(const string& connectionString)
{
	bool opened = false;

	// if no serial connection, fail
	if (!IsOpen())
	{
		return opened;
	}

	CloseServerConnection();
	vector<string> pieces;
	SplitString(connectionString, ":", pieces);
	if (pieces.size() < 3)
	{
		return opened;
	}
	else if (pieces[0] == "RTCM3")
	{
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
			m_tcpClient.HttpGet(url, "InertialSense", user, pwd);
		}
		m_parser = cGpsParser::CreateParser(GpsParserTypeRtcm3, this);
	}
	else if (pieces[0] == "IS")
	{
		opened = (m_tcpClient.Open(pieces[1], atoi(pieces[2].c_str())) == 0);
		m_parser = cGpsParser::CreateParser(GpsParserTypeInertialSense, this);
	}
	else if (pieces[0] == "UBLOX")
	{
		opened = (m_tcpClient.Open(pieces[1], atoi(pieces[2].c_str())) == 0);
		m_parser = cGpsParser::CreateParser(GpsParserTypeUblox, this);
	}

	if (opened)
	{
		// configure as RTK rover
		uint32_t cfgBits = SYS_CFG_BITS_RTK_ROVER;
		for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
		{
			sendDataComManager((int)i, DID_FLASH_CONFIG, &cfgBits, sizeof(cfgBits), OFFSETOF(nvm_flash_cfg_t, sysCfgBits));
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
	if (m_parser != NULLPTR)
	{
		delete m_parser;
		m_parser = NULLPTR;
	}
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
	bool opened = (m_tcpServer.Open(host, port) == 0);
	if (opened)
	{
		// configure as RTK base station
		uint32_t cfgBits = SYS_CFG_BITS_RTK_BASE_STATION;
		sendDataComManager(0, DID_FLASH_CONFIG, &cfgBits, sizeof(cfgBits), OFFSETOF(nvm_flash_cfg_t, sysCfgBits));
		cfgBits = 1; // raw messages
		sendDataComManager(0, DID_CONFIG, &cfgBits, sizeof(cfgBits), OFFSETOF(config_t, msgCfgBits));
	}
	return opened;
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
	if (m_clientStream != NULLPTR)
	{
		int count = m_clientStream->Read(buffer, sizeof(buffer));
		if (count > 0)
		{
			m_clientServerByteCount += count;
			m_parser->Write(buffer, count);
		}
	}
	else if (m_tcpServer.IsOpen() && m_comManagerState.serialPorts.size() != 0)
	{
		// as a tcp server, only the first serial port is read from
		int count = serialPortRead(&m_comManagerState.serialPorts[0], buffer, sizeof(buffer));
		if (count > 0)
		{
			// forward data on to connected clients
			m_tcpServer.Write(buffer, count);
			m_clientServerByteCount += count;
		}
		m_tcpServer.Update();
	}
	stepComManager();

	// if any serial ports have closed, shut everything down
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
		SLEEP_MS(1000);
	}
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		serialPortClose(&m_comManagerState.serialPorts[i]);
	}
	m_comManagerState.serialPorts.clear();
	m_comManagerState.config.clear();
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

void InertialSense::StopBroadcasts()
{
	// Stop all broadcasts

	// // Binary command
	// unsigned char pkt[9] = { 0xff,0x06,0x00,0x01,0x00,0xfd,0x00,0x07,0xfe };

	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		// Turn off any existing broadcast messages, i.e. INS, IMU, GPS, etc.
		sendComManager((int)i, PID_STOP_ALL_BROADCASTS, 0, 0, 0);
	}
}

void InertialSense::SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		sendDataComManager((int)i, dataId, data, length, offset);
	}
}

void InertialSense::SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		sendRawDataComManager((int)i, dataId, data, length, offset);
	}
}

void InertialSense::SetConfig(const config_t& config, int pHandle)
{
	m_comManagerState.config[pHandle] = config;
	sendDataComManager(pHandle, DID_CONFIG, &m_comManagerState.config[pHandle], sizeof(config), 0);
}

void InertialSense::SetFlashConfig(const nvm_flash_cfg_t& flashConfig, int pHandle)
{
	m_comManagerState.flashConfig[pHandle] = flashConfig;
	sendDataComManager(pHandle, DID_FLASH_CONFIG, &m_comManagerState.flashConfig[pHandle], sizeof(flashConfig), 0);
}

bool InertialSense::BroadcastBinaryData(uint32_t dataId, int periodMS, pfnHandleBinaryData callback)
{
	if (m_comManagerState.serialPorts.size() == 0 || dataId >= DID_COUNT)
	{
		return false;
	}
	else
	{
		m_comManagerState.binaryCallback[dataId] = callback;
	}
	if (periodMS < 0)
	{
		for (int i = 0; i < (int)m_comManagerState.serialPorts.size(); i++)
		{
			disableDataComManager(i, dataId);
		}
	}
	else
	{
		for (int i = 0; i < (int)m_comManagerState.serialPorts.size(); i++)
		{
			// we pass an offset and size of 0. The uINS will handle a size and offset of 0 and make it the full struct size.
			getDataComManager(i, dataId, 0, 0, periodMS);
		}
	}
	return true;
}

bool InertialSense::BootloadFile(const string& comPort, const string& fileName, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, char* errorBuffer, int errorBufferLength)
{
	// test file exists
	{
		ifstream tmpStream(fileName);
		if (!tmpStream.good())
		{
			if (errorBuffer != NULLPTR)
			{
				SNPRINTF(errorBuffer, errorBufferLength, "Bootloader file does not exist");
			}
			return false;
		}
	}

	// for debug
	// SERIAL_PORT_DEFAULT_TIMEOUT = 9999999;

	vector<string> portStrings;
	vector<bootloader_state_t> state;
	if (comPort == "*")
	{
		cISSerialPort::GetComPorts(portStrings);
	}
	else
	{
		SplitString(comPort, ",", portStrings);
	}
	state.resize(portStrings.size());

	// for each port requested, setup a thread to do the bootloader for that port
	for (size_t i = 0; i < portStrings.size(); i++)
	{
		memset(&state[i].serial, 0, sizeof(state[i].serial));
		serialPortSetPort(&state[i].serial, portStrings[i].c_str());
		serialPortPlatformInit(&state[i].serial);
		state[i].param.uploadProgress = uploadProgress;
		state[i].param.verifyProgress = verifyProgress;
		state[i].param.fileName = fileName.c_str();
		state[i].param.error = errorBuffer;
		state[i].param.errorLength = errorBufferLength;
		state[i].param.port = &state[i].serial;
		state[i].param.verifyFileName = NULLPTR;
		state[i].param.flags.bitFields.enableVerify = (verifyProgress != NULLPTR);
		state[i].thread = threadCreateAndStart(bootloaderThread, &state[i]);
	}

	// wait for all threads to finish
	for (size_t i = 0; i < state.size(); i++)
	{
		threadJoinAndFree(state[i].thread);
	}

	// if any thread failed, we return failure
	for (size_t i = 0; i < state.size(); i++)
	{
		if (!state[i].result)
		{
			return false;
		}
	}

	// all threads succeeded
	return true;
}

size_t InertialSense::SplitString(const string& s, const string& delimiter, vector<string>& result)
{
	result.clear();
	size_t pos = 0;
	size_t pos2;
	while (true)
	{
		pos2 = s.find(delimiter, pos);
		if (pos2 == string::npos)
		{
			result.push_back(s.substr(pos));
			break;
		}
		else
		{
			result.push_back(s.substr(pos, pos2 - pos));
			pos = pos2 + delimiter.length();
		}
	}
	return result.size();
}

bool InertialSense::OnPacketReceived(const cGpsParser* parser, const uint8_t* data, uint32_t dataLength)
{
	(void)parser;
	for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
	{
		// sleep in between to allow test bed to send the serial data
		SLEEP_MS(10);
		serialPortWrite(&m_comManagerState.serialPorts[i], data, dataLength);
	}
	return false; // do not parse, since we are just forwarding it on
}

bool InertialSense::OpenSerialPorts(const char* port, int baudRate)
{
	Close();

	if (port == NULLPTR || validateBaudRate(baudRate) != 0)
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
		SplitString(port, ",", ports);
	}

	// open serial ports
	for (size_t i = 0; i < ports.size(); i++)
	{
		serial_port_t serial;
		memset(&serial, 0, sizeof(serial));
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

	// setup com manager
	initComManager((int)m_comManagerState.serialPorts.size(), 10, 10, 10, staticReadPacket, staticSendPacket, 0, staticProcessRxData, 0, 0);

	// re-initialize data sets
	config_t configTemplate;
	memset(&configTemplate, 0, sizeof(configTemplate));
	m_comManagerState.config.resize(m_comManagerState.serialPorts.size(), configTemplate);
	dev_info_t devInfoTemplate;
	memset(&devInfoTemplate, 0, sizeof(devInfoTemplate));
	m_comManagerState.devInfo.resize(m_comManagerState.serialPorts.size(), devInfoTemplate);
	nvm_flash_cfg_t flashTemplate;
	memset(&flashTemplate, 0, sizeof(flashTemplate));
	m_comManagerState.flashConfig.resize(m_comManagerState.serialPorts.size(), flashTemplate);

	// negotiate baud rate by querying device info - don't return out until it negotiates or times out
	// if the baud rate is already correct, the request for the message should succeed very quickly
	time_t startTime = time(0);

	// try to auto-baud for up to 3 seconds, then abort if we didn't get a valid packet
	// we wait until we get a valid serial number and manufacturer
	while (!HasReceivedResponseFromAllDevices() && time(0) - startTime < 3)
	{
		for (size_t i = 0; i < m_comManagerState.serialPorts.size(); i++)
		{
			getDataComManager((int)i, DID_CONFIG, 0, 0, 0);
			getDataComManager((int)i, DID_DEV_INFO, 0, 0, 0);
			getDataComManager((int)i, DID_FLASH_CONFIG, 0, 0, 0);
		}

		SLEEP_MS(12);
		stepComManager();
		SLEEP_MS(12);
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
		initComManager((int)m_comManagerState.serialPorts.size(), 10, 10, 10, staticReadPacket, staticSendPacket, 0, staticProcessRxData, 0, 0);
	}

    return true;
}

#if defined(ENABLE_IS_PYTHON_WRAPPER)

namespace py = pybind11;

bool InertialSense::PyBootloadFile(const string& comPort, const string& fileName)
{
	pfnBootloadProgress uploadProgress = bootloadUploadProgress;
	pfnBootloadProgress verifyProgress = bootloadVerifyProgress;
	char bootloaderError[1024];
	int errorBufferLength = 1024;

	// for debug
	// SERIAL_PORT_DEFAULT_TIMEOUT = 9999999;
	InertialSense bootloader;
	serialPortSetPort(&bootloader.m_serialPort, comPort.c_str());
	if (!enableBootloader(&bootloader.m_serialPort, &bootloaderError[0], errorBufferLength, IS_BAUD_RATE_BOOTLOADER_COM))
	{
		serialPortClose(&bootloader.m_serialPort);
		return false;
	}
	bootload_params_t params;
	params.fileName = fileName.c_str();
	params.port = &bootloader.m_serialPort;
	params.error = &bootloaderError[0];
	params.errorLength = errorBufferLength;
	params.obj = &bootloader;
	params.uploadProgress = uploadProgress;
	params.verifyProgress = verifyProgress;
	params.verifyFileName = NULLPTR;
	if (!bootloadFileEx(&params))
	{
		serialPortClose(&bootloader.m_serialPort);
		return false;
	}
	serialPortClose(&bootloader.m_serialPort);
	return true;
}

void InertialSense::SetPyCallback(py::object func)
{
	InertialSense::m_pyCallback = func;
}

py::object InertialSense::GetPyCallback()
{
	return m_pyCallback;
}

void InertialSense::SetPyDisplay(cInertialSenseDisplay display)
{
	m_pyDisplay = display;
}

cInertialSenseDisplay InertialSense::GetPyDisplay()
{
	return m_pyDisplay;
}

py::dict py_dataHandling(p_data_t* data)
{
	// uDatasets is a union of all datasets that we can receive.  See data_sets.h for a full list of all available datasets. 
	uDatasets d = {};
	copyDataPToStructP(&d, data, sizeof(uDatasets));

	// Put the data into a python usable type (This works)
	py::dict dOut;
	py::dict header;
	py::dict msgData;
	header[py::str("id")] = data->hdr.id;
	header[py::str("size")] = data->hdr.size;
	header[py::str("offset")] = data->hdr.offset;
	dOut[py::str("header")] = header;

	// Example of how to access dataset fields.
	switch (data->hdr.id)
	{
	case DID_INS_1:
		msgData["theta"] = convertArray(d.ins1.theta, 3);		// euler attitude
		msgData["uvw"] = convertArray(d.ins1.uvw, 3);			// body velocities
		msgData["lla"] = convertArray(d.ins1.lla, 3);			// latitude, longitude, altitude
		break;
	case DID_INS_2:
		msgData["qn2b"] = convertArray(d.ins2.qn2b, 4);			// quaternion attitude 
		msgData["uvw"] = convertArray(d.ins2.uvw, 3);			// body velocities
		msgData["lla"] = convertArray(d.ins2.lla, 3);			// latitude, longitude, altitude
		break;
	case DID_INS_3:
		msgData["qn2b"] = convertArray(d.ins3.qn2b, 4);			// quaternion attitude 
		msgData["uvw"] = convertArray(d.ins3.uvw, 3);			// body velocities
		msgData["lla"] = convertArray(d.ins3.lla, 3);			// latitude, longitude, altitude
		break;
	case DID_INS_4:
		msgData["qe2b"] = convertArray(d.ins4.qe2b, 4);			// quaternion attitude 
		msgData["ve"] = convertArray(d.ins4.ve, 3);				// earth centered earth fixed  velocities
		msgData["ecef"] = convertArray(d.ins4.ecef, 3);			// earth centered earth fixed position
		break;
	case DID_DUAL_IMU:
		msgData["imu1acc"] = convertArray(d.dualImu.I[0].acc, 3);
		msgData["imu1pqr"] = convertArray(d.dualImu.I[0].pqr, 3);
		msgData["imu2acc"] = convertArray(d.dualImu.I[1].acc, 3);
		msgData["imu2pqr"] = convertArray(d.dualImu.I[1].pqr, 3);
		msgData["time"] = d.dualImu.time;
		break;
	case DID_DELTA_THETA_VEL:
		msgData["time"] = d.dThetaVel.time;
		msgData["theta"] = convertArray(d.dThetaVel.theta, 3);
		msgData["uvw"] = convertArray(d.dThetaVel.uvw, 3);
		msgData["dt"] = d.dThetaVel.dt;
		break;
	case DID_GPS:
	{
		msgData["rxps"] = d.gps.rxps;
		msgData["towOffset"] = d.gps.towOffset;

		py::dict gpsPos;
		gpsPos["week"] = d.gps.pos.week;
		gpsPos["timeOfWeekMs"] = d.gps.pos.timeOfWeekMs;
		gpsPos["status"] = d.gps.pos.status;
		gpsPos["cno"] = d.gps.pos.cno;
		gpsPos["lla"] = convertArray(d.gps.pos.lla, 3);
		gpsPos["hMSL"] = d.gps.pos.hMSL;
		gpsPos["hAcc"] = d.gps.pos.hAcc;
		gpsPos["vAcc"] = d.gps.pos.vAcc;
		gpsPos["pDop"] = d.gps.pos.pDop;
		msgData["gpsPos"] = gpsPos;

		py::dict gpsVel;
		gpsVel["timeOfWeekMs"] = d.gps.vel.timeOfWeekMs;
		gpsVel["ned"] = convertArray(d.gps.vel.ned, 3);
		gpsVel["s2D"] = d.gps.vel.s2D;
		gpsVel["s3D"] = d.gps.vel.s3D;
		gpsVel["sAcc"] = d.gps.vel.sAcc;
		gpsVel["course"] = d.gps.vel.course;
		gpsVel["cAcc"] = d.gps.vel.cAcc;
		msgData["gpsVel"] = gpsVel;
		break;
	}
	case DID_MAGNETOMETER_1:
		msgData["mag"] = convertArray(d.mag.mag, 3);
		msgData["time"] = d.mag.time;
		break;
	case DID_MAGNETOMETER_2:
		msgData["mag"] = convertArray(d.mag.mag, 3);
		msgData["time"] = d.mag.time;
		break;
	case DID_BAROMETER:
		msgData["time"] = d.baro.time;
		msgData["bar"] = d.baro.bar;
		msgData["barTemp"] = d.baro.barTemp;
		msgData["humidity"] = d.baro.humidity;
		msgData["mslBar"] = d.baro.mslBar;
		break;
	case DID_RAW_GPS_DATA:
		//printf("Received the Raw Data Message!");
		//py::list buffer;
		msgData["receiverIndex"] = d.gpsRaw.receiverIndex;
		msgData["type"] = d.gpsRaw.type;
		msgData["count"] = d.gpsRaw.count;
		msgData["reserved"] = d.gpsRaw.reserved;
		/*for (int i = 0; i < 1020; i++) {
		buffer.append(d.gpsRaw.buf)
		}
		msgData["buf"] = buffer; */
		msgData["buf"] = d.gpsRaw.buf;
		break;
	}

	dOut[py::str("data")] = msgData;

	return dOut;
}

static void pyClt_dataCallback(InertialSense* i, p_data_t* data)
{
	i->GetPyDisplay().ProcessData(data);

	py::dict dOut = py_dataHandling(data);

	py::object pyFunc = i->GetPyCallback();
	if (pyFunc != NULLPTR)
	{
		pyFunc(dOut);
	}
}

py::list convertArray(float const data[], int length)
{
	py::list dOut;
	for (int i = 0; i < length; i++)
	{
		dOut.append(data[i]);
	}

	return dOut;
}

py::list convertArray(double const data[], int length)
{
	py::list dOut;
	for (int i = 0; i < length; i++)
	{
		dOut.append(data[i]);
	}

	return dOut;
}

bool pyUpdateFlashConfig(InertialSense& inertialSenseInterface, string flashConfigString)
{
	const nvm_flash_cfg_t& flashConfig = inertialSenseInterface.GetFlashConfig();
	const map_lookup_name_t& globalMap = cISDataMappings::GetMap();
	const map_name_to_info_t& flashMap = globalMap.at(DID_FLASH_CONFIG);

	if (flashConfigString.length() < 2)
	{
		// read flash config and display
		data_mapping_string_t stringBuffer;
		cout << "Current flash config" << endl;
		for (map_name_to_info_t::const_iterator i = flashMap.begin(); i != flashMap.end(); i++)
		{
			if (cISDataMappings::DataToString(i->second, NULLPTR, (const uint8_t*)&flashConfig, stringBuffer))
			{
				cout << i->second.name << " = " << stringBuffer << endl;
			}
		}
		return false;
	}
	else
	{
		nvm_flash_cfg_t flashConfig = inertialSenseInterface.GetFlashConfig();
		vector<string> keyValues;
		InertialSense::SplitString(flashConfigString, "|", keyValues);
		for (size_t i = 0; i < keyValues.size(); i++)
		{
			vector<string> keyAndValue;
			InertialSense::SplitString(splitString(keyValues[i], "=", keyAndValue);
			if (keyAndValue.size() == 2)
			{
				if (flashMap.find(keyAndValue[0]) == flashMap.end())
				{
					cout << "Unrecognized flash config key '" << keyAndValue[0] << "' specified, ignoring." << endl;
				}
				else
				{
					const data_info_t& info = flashMap.at(keyAndValue[0]);
					cISDataMappings::StringToData(keyAndValue[1].c_str(), NULLPTR, (uint8_t*)&flashConfig, info);
					cout << "Updated flash config key '" << keyAndValue[0] << "' to '" << keyAndValue[1].c_str() << "'" << endl;
				}
			}
		}
		inertialSenseInterface.SetFlashConfig(flashConfig);
		SLEEP_MS(1000);
		inertialSenseInterface.GetPyDisplay().Clear();
		return true;
	}
}

test_initializer inertialsensemodule([](py::module &m) {
	py::module m2 = m.def_submodule("inertialsensemodule");

	py::class_<iGpsParserDelegate> iGpsPD(m2, "GPS_Parser_Delegate");
	iGpsPD.def("OnPacketReceived", &iGpsParserDelegate::OnPacketReceived);

	py::class_<InertialSense> inertialsense(m2, "InertialSense", iGpsPD);
	inertialsense.def(py::init<>());
	inertialsense.def("Open", &InertialSense::Open);
	inertialsense.def("Update", &InertialSense::Update);
	inertialsense.def("GetTcpByteCount", &InertialSense::GetTcpByteCount);
	inertialsense.def("Close", &InertialSense::Close);
	inertialsense.def("StopBroadcasts", &InertialSense::StopBroadcasts);
	inertialsense.def("BroadcastBinaryData", &InertialSense::BroadcastBinaryData);
	inertialsense.def("OpenServerConnectionRTCM3", &InertialSense::OpenServerConnectionRTCM3);
	inertialsense.def("OpenServerConnectionInertialSense", &InertialSense::OpenServerConnectionInertialSense);
	inertialsense.def("OpenServerConnectionUblox", &InertialSense::OpenServerConnectionUblox);
	inertialsense.def_static("BootloadFile", &InertialSense::BootloadFile);
	inertialsense.def_static("PyBootloadFile", &InertialSense::PyBootloadFile);
	inertialsense.def("SetLoggerEnabled", &InertialSense::SetLoggerEnabled);
	inertialsense.def("SetPyCallback", &InertialSense::SetPyCallback);
	inertialsense.def("PyBroadcastBinaryData", &InertialSense::PyBroadcastBinaryData);
	inertialsense.def("SetPyDisplay", &InertialSense::SetPyDisplay);
	inertialsense.def("CreateHost", &InertialSense::CreateHost);

	m2.def("pyUpdateFlashConfig", pyUpdateFlashConfig);
});

bool InertialSense::PyBroadcastBinaryData(uint32_t dataId, int periodMS)
{
	if (m_comManagerState.binarySerialPort == NULLPTR || dataId >= DID_COUNT)
	{
		return false;
	}

	m_comManagerState.binaryCallback[dataId] = (pfnHandleBinaryData)pyClt_dataCallback;

	if (dataId == DID_RAW_GPS_DATA)
	{
		config_t cfg = GetConfig();
		cfg.msgCfgBits = 1;
		SetConfig(cfg);
	}

	if (periodMS < 0)
	{
		disableDataComManager(0, dataId);
	}
	else
	{
		// we pass an offset and size of 0. The uINS will handle a size and offset of 0 and make it the full struct size.
		getDataComManager(0, dataId, 0, 0, periodMS);
	}
	return true;
}

#endif
