/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "InertialSense.h"

using namespace std;

static int staticSendPacket(CMHANDLE cmHandle, int pHandle, buffer_t *packet)
{
	// Suppress compiler warnings
	(void)pHandle;
    (void)cmHandle;

	InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);
	return serialPortWrite(s->binarySerialPort, packet->buf, packet->size);
}

static int staticReadPacket(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len)
{
	// Suppress compiler warnings
	(void)pHandle;
    (void)cmHandle;

	InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);
	return serialPortRead(s->binarySerialPort, buf, len);
}

static void staticProcessRxData(CMHANDLE cmHandle, int pHandle, p_data_t* data)
{
    (void)pHandle;
    (void)cmHandle;

	if (data->hdr.id < DID_COUNT)
	{
		InertialSense::com_manager_cpp_state_t* s = (InertialSense::com_manager_cpp_state_t*)comManagerGetUserPointer(cmHandle);
		pfnHandleBinaryData handler = s->binaryCallback[data->hdr.id];
		s->stepLogFunction(s->inertialSenseInterface, data);
		if (handler != NULL)
		{
			handler(s->inertialSenseInterface, data);
		}

		pfnHandleBinaryData handlerGlobal = s->binaryCallbackGlobal;
		if (handlerGlobal != NULL)
		{	
			// Called for all DID's
			handlerGlobal(s->inertialSenseInterface, data);
		}

		// if we got the device info, populate it
		if (data->hdr.id == DID_DEV_INFO)
		{
			memcpy(s->devInfo, data->buf, sizeof(dev_info_t));
		}
	}
}

InertialSense::InertialSense(pfnHandleBinaryData callback, serial_port_t* serialPort) : m_rtcm3Reader(this)
{
	memset(&m_comManagerState, 0, sizeof(m_comManagerState));
	memset(&m_deviceInfo, 0, sizeof(m_deviceInfo));
	m_logSolution = SLOG_DISABLED;
	m_cmHandle = getGlobalComManager();
	m_pHandle = 0;
	m_logThread = nullptr;
	m_lastLogReInit = time(0);

	if (serialPort == NULL)
	{
		memset(&m_serialPort, 0, sizeof(m_serialPort));
		serialPortPlatformInit(&m_serialPort);
	}
	else
	{
		// the serial port should have already been initialized by the caller
		memcpy(&m_serialPort, serialPort, sizeof(serial_port_t));
	}

	m_comManagerState.binarySerialPort = &m_serialPort;
	m_comManagerState.stepLogFunction = &InertialSense::StepLogger;
	m_comManagerState.inertialSenseInterface = this;
	m_comManagerState.devInfo = &m_deviceInfo;
	comManagerAssignUserPointer(m_cmHandle, &m_comManagerState);
	initComManager(1, 10, 10, 10, staticReadPacket, staticSendPacket, 0, staticProcessRxData, 0, 0);
	m_comManagerState.binaryCallbackGlobal = callback;
}

InertialSense::~InertialSense()
{
	Close();
}

void InertialSense::DisableLogging()
{
	m_logger.EnableLogging(false);
	if (m_logThread != nullptr)
	{
		m_logThread->join();
		delete m_logThread;
		m_logThread = nullptr;
	}
	m_logger.CloseAllFiles();
}

void InertialSense::LoggerThread()
{
	bool running = true;
	while (running)
	{
		SLEEP_MS(20);

		// gather up packets in memory
		vector<p_data_t> packets;
		{
			// lock so we can read and clear m_logPackets
			lock_guard<mutex> l(m_logMutex);
			for (list<p_data_t>::iterator i = m_logPackets.begin(); i != m_logPackets.end(); i++)
			{
				packets.push_back(*i);
			}

			// clear shared memory
			m_logPackets.clear();

			// update running state
			running = m_logger.Enabled();
		}

		// log the packets
		for (size_t i = 0; i < packets.size(); i++)
		{
			m_logger.LogData(0, &packets[i].hdr, packets[i].buf);
		}
	}
}

void InertialSense::StepLogger(InertialSense* i, const p_data_t* data)
{
	// single threaded implementation would just do this:
	// i->m_logger.LogData(0, (p_data_hdr_t*)&data->hdr, (uint8_t*)data->buf); return;
	const time_t reinitSeconds = 2;
	{
		lock_guard<mutex> l(i->m_logMutex);
		if (i->m_logger.Enabled())
		{
			i->m_logPackets.push_back(*data);
		}
	}

	// if time has passed, re-send the appropriate log solution message
	if (time(0) - i->m_lastLogReInit > reinitSeconds)
	{
		sendDataComManagerInstance(i->m_cmHandle, 0, DID_CONFIG, &i->m_logSolution, sizeof(i->m_logSolution), offsetof(config_t, sLogCtrl));
		i->m_lastLogReInit = time(0);
	}
}

bool InertialSense::Open(const char* port, int baudRate)
{
	// clear the device in preparation for auto-baud
	memset(&m_deviceInfo, 0, sizeof(m_deviceInfo));

	if (validateBaudRate(baudRate) != 0 || serialPortOpen(&m_serialPort, port, baudRate, 0) != 1)
	{
		return false;
	}

	// negotiate baud rate by querying device info - don't return out until it negotiates or times out
	// if the baud rate is already correct, the request for the message should succeed very quickly
	time_t startTime = time(0);

	// try to auto-baud for up to 3 seconds, then abort if we didn't get a valid packet
	// we wait until we get a valid serial number and manufacturer
	while (m_deviceInfo.serialNumber == 0 && m_deviceInfo.manufacturer[0] == '\0' && time(0) - startTime < 3)
	{
		getDataComManagerInstance(m_cmHandle, 0, DID_DEV_INFO, 0, 0, 0);
		SLEEP_MS(2); // 2 milliseconds wait for reply
		stepComManagerInstance(m_cmHandle);
	}

	if (m_deviceInfo.serialNumber == 0 && m_deviceInfo.manufacturer[0] == '\0')
	{
		Close();
		return false;
	}

	return true;
}

void InertialSense::SetLoggerEnabled(bool enable, const string& path, uint32_t logSolution, float maxDiskSpaceMB, uint32_t maxFileSize, uint32_t chunkSize, bool useTimestampSubFolder)
{
	m_logger.CloseAllFiles();
	m_logSolution = logSolution;
	sendDataComManagerInstance(m_cmHandle, 0, DID_CONFIG, &m_logSolution, sizeof(uint32_t), offsetof(config_t, sLogCtrl));

	if (enable)
	{
		getDataComManagerInstance(m_cmHandle, 0, DID_DEV_INFO, 0, 0, 0);
		lock_guard<mutex> lk(m_logMutex);
		m_logger.InitSave(cISLogger::LOGTYPE_DAT, path, 1, maxDiskSpaceMB, maxFileSize, chunkSize, useTimestampSubFolder);
		m_logger.EnableLogging(true);
		if (m_logThread == nullptr)
		{
			m_logThread = new thread(&InertialSense::LoggerThread, this);
		}
	}
	else
	{
		DisableLogging();
	}
}

bool InertialSense::OpenServerConnectionRTCM3(const string& hostAndPort)
{
	size_t colon = hostAndPort.find(':', 0);
	if (colon == string::npos)
	{
		return false;
	}
	size_t colon2 = hostAndPort.find(':', colon + 1);
	if (colon2 == string::npos)
	{
		return false;
	}
	string host = hostAndPort.substr(0, colon);
	string portString = hostAndPort.substr(colon + 1, colon2 - colon - 1);
	int port = (int)strtol(portString.c_str(), NULL, 10);
	bool opened = (m_tcpClient.Open(host, port) == 0);
	if (opened)
	{
		// try to find url:user:pwd on the string
		colon = hostAndPort.find(':', colon2 + 1);
		if (colon != string::npos)
		{
			string url = hostAndPort.substr(colon2 + 1, colon - colon2 - 1);
			colon2 = hostAndPort.find(':', colon + 1);
			if (colon2 != string::npos)
			{
				string user = hostAndPort.substr(colon + 1, colon2 - colon - 1);
				string pwd = hostAndPort.substr(colon2 + 1);
				if (user.size() != 0 && pwd.size() != 0)
				{
					m_tcpClient.HttpGet(url, "InertialSense", user, pwd);
				}
			}
		}
	}
	return opened;
}

void InertialSense::CloseServerConnection()
{
	m_tcpClient.Close();
}

bool InertialSense::IsOpen()
{
	return (serialPortIsOpen(&m_serialPort) != 0);
}

void InertialSense::Update()
{
	if (m_tcpClient.IsOpen())
	{
		uint8_t buffer[4096];
		int count = m_tcpClient.Read(buffer, sizeof(buffer), false);
		for (int i = 0; i < count; i++)
		{
			m_rtcm3Reader.WriteByte(buffer[i]);
		}
	}
	stepComManager();
}

void InertialSense::Close()
{
	SetLoggerEnabled(false);
	serialPortClose(&m_serialPort);
	m_tcpClient.Close();
}

const char* InertialSense::GetPort()
{
	return m_serialPort.port;
}

void InertialSense::StopBroadcasts()
{	
	// Stop all broadcasts

#if 0	// Binary command

	unsigned char pkt[9] = { 0xff,0x06,0x00,0x01,0x00,0xfd,0x00,0x07,0xfe };
	serialPortWrite(&m_serialPort, pkt, 9);

#elif 0	// ASCII command

	serialPortWriteAscii(&m_serialPort, "STPB", 4);

#else

	// Turn off any existing broadcast messages, i.e. INS, IMU, GPS, etc.
	sendComManagerInstance(m_cmHandle, 0, PID_STOP_ALL_BROADCASTS, 0, 0, 0);

#endif

}

void InertialSense::SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	sendRawDataComManagerInstance(m_cmHandle, m_pHandle, dataId, data, length, offset);
}

bool InertialSense::BroadcastBinaryData(uint32_t dataId, int periodMS, pfnHandleBinaryData callback)
{
	if (m_comManagerState.binarySerialPort == NULL || dataId >= DID_COUNT || (m_comManagerState.binaryCallbackGlobal == NULL && m_comManagerState.binaryCallback[dataId] == NULL))
	{
		return false;
	}
	m_comManagerState.binaryCallback[dataId] = callback;
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

void InertialSense::SetBroadcastSolutionEnabled(bool enable)
{
	m_logSolution = (enable ? SLOG_W_INS2 : SLOG_DISABLED);
}

bool InertialSense::BootloadFile(const string& comPort, const string& fileName, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, char* errorBuffer, int errorBufferLength)
{
	// for debug
	// SERIAL_PORT_DEFAULT_TIMEOUT = 9999999;
	Close();
	serialPortSetPort(&m_serialPort, comPort.c_str());
	if (!enableBootloader(&m_serialPort, errorBuffer, errorBufferLength))
	{
		serialPortClose(&m_serialPort);
		return false;
	}
	bootload_params_t params;
	params.fileName = fileName.c_str();
	params.port = &m_serialPort;
	params.error = errorBuffer;
	params.errorLength = errorBufferLength;
	params.obj = this;
	params.uploadProgress = uploadProgress;
	params.verifyProgress = verifyProgress;
	params.verifyFileName = NULL;
	if (!bootloadFileEx(&params))
	{
		serialPortClose(&m_serialPort);
		return false;
	}
	serialPortClose(&m_serialPort);
	Open(m_serialPort.port);
	return true;
}

bool InertialSense::OnPacketReceived(const cRtcm3Reader* reader, const uint8_t* data, uint32_t dataLength)
{
	(void)reader;

	serialPortWrite(&m_serialPort, data, dataLength);
	return false; // do not parse, since we are just forwarding it on
}

