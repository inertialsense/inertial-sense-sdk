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

InertialSense::InertialSense(pfnHandleBinaryData callback, serial_port_t* serialPort)
{
	memset(&m_comManagerState, 0, sizeof(m_comManagerState));
	memset(&m_deviceInfo, 0, sizeof(m_deviceInfo));
	m_logSolution = SLOG_DISABLED;
	m_cmHandle = getGlobalComManager();
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
	lock_guard<mutex> lk(m_logMutex);
	m_logger.EnableLogging(false);
	m_logCondition.notify_one();
	if (m_logThread != nullptr)
	{
		m_logThread->join();
		delete m_logThread;
		m_logThread = nullptr;
	}
}

void InertialSense::LoggerThread()
{
	while (m_logger.Enabled())
	{
		unique_lock<mutex> lk(m_logMutex);
		m_logCondition.wait(lk, [this] { return this->m_logPackets.size() != 0 || !m_logger.Enabled(); });
		if (m_logPackets.size() != 0 && m_logger.Enabled())
		{
			m_logger.LogData(0, &m_logPackets.begin()->hdr, m_logPackets.begin()->buf);
			m_logPackets.pop_front();
		}
	}
}

void InertialSense::StepLogger(InertialSense* i, const p_data_t* data)
{
	// single threaded implementation would just do this:
	// i->m_logger.LogData(0, (p_data_hdr_t*)&data->hdr, (uint8_t*)data->buf); return;
	const time_t reinitSeconds = 2;

	if (i->m_logger.Enabled())
	{
		lock_guard<mutex> lk(i->m_logMutex);
		i->m_logPackets.push_back(*data);
		i->m_logCondition.notify_one();
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

	// TODO: Validate baud rate, return false if invalid

	if (!(serialPortOpen(&m_serialPort, port, baudRate, 0) != 0))
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

bool InertialSense::IsOpen()
{
	return (serialPortIsOpen(&m_serialPort) != 0);
}

void InertialSense::Update()
{
	stepComManager();
}

void InertialSense::Close()
{
	SetLoggerEnabled(false);
	serialPortClose(&m_serialPort);
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

bool InertialSense::BootloadFile(const string& fileName, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, char* errorBuffer, int errorBufferLength)
{
	// for debug
	// SERIAL_PORT_DEFAULT_TIMEOUT = 9999999;
	Close();
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



