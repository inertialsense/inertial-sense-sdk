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
		else if (data->hdr.id == DID_CONFIG)
		{
			memcpy(s->config, data->buf, sizeof(config_t));
		}
		else if (data->hdr.id == DID_FLASH_CONFIG)
		{
			memcpy(s->flashConfig, data->buf, sizeof(nvm_flash_cfg_t));
		}
	}
}

InertialSense::InertialSense(pfnHandleBinaryData callback, serial_port_t* serialPort)
{
	memset(&m_comManagerState, 0, sizeof(m_comManagerState));
	memset(&m_deviceInfo, 0, sizeof(m_deviceInfo));
	memset(&m_config, 0, sizeof(m_config));
	memset(&m_flashConfig, 0, sizeof(m_flashConfig));
	m_cmHandle = getGlobalComManager();
	m_pHandle = 0;
	m_logThread = NULLPTR;
	m_lastLogReInit = time(0);
	m_parser = NULL;
	m_tcpByteCount = 0;

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
	m_comManagerState.config = &m_config;
	m_comManagerState.flashConfig = &m_flashConfig;
	comManagerAssignUserPointer(m_cmHandle, &m_comManagerState);
	initComManager(1, 10, 10, 10, staticReadPacket, staticSendPacket, 0, staticProcessRxData, 0, 0);
	
#if defined(ENABLE_IS_PYTHON_WRAPPER)
		
	m_comManagerState.binaryCallbackGlobal = (callback == NULL ? NULL : (pfnHandleBinaryData)pyClt_dataCallback);

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
	if (!m_logger.InitSaveTimestamp(subFolder, path, cISLogger::g_emptyString, 1, cISLogger::LOGTYPE_DAT, maxDiskSpacePercent, maxFileSize, chunkSize, subFolder.length() != 0))
	{
		return false;
	}
	m_logger.EnableLogging(true);
	m_logger.SetDeviceInfo(&m_deviceInfo);
	if (m_logThread == NULLPTR)
	{
		m_logThread = threadCreateAndStart(&InertialSense::LoggerThread, this);
	}
	return true;
}

void InertialSense::DisableLogging()
{
	{
		cMutexLocker logMutexLocker(&m_logMutex);
		m_logger.EnableLogging(false);
	}
	threadJoinAndFree(m_logThread);
	m_logThread = NULLPTR;
	{
		cMutexLocker logMutexLocker(&m_logMutex);
		m_logger.CloseAllFiles();
	}
}

void InertialSense::LoggerThread(void* info)
{
	bool running = true;
	InertialSense* inertialSense = (InertialSense*)info;

	// gather up packets in memory
	vector<p_data_t> packets;

	while (running)
	{
		SLEEP_MS(2);
		{
			// lock so we can read and clear m_logPackets
			cMutexLocker logMutexLocker(&inertialSense->m_logMutex);
			for (list<p_data_t>::iterator i = inertialSense->m_logPackets.begin(); i != inertialSense->m_logPackets.end(); i++)
			{
				packets.push_back(*i);
			}

			// clear shared memory
			inertialSense->m_logPackets.clear();

			// update running state
			running = inertialSense->m_logger.Enabled();
		}

		// log the packets
		for (size_t i = 0; i < packets.size(); i++)
		{
            if (!inertialSense->m_logger.LogData(0, &packets[i].hdr, packets[i].buf))
            {
                // Failed to write to log
				return;
			}
		}
		packets.clear();
	}
}

void InertialSense::StepLogger(InertialSense* i, const p_data_t* data)
{
	// single threaded implementation would just do this:
	// i->m_logger.LogData(0, (p_data_hdr_t*)&data->hdr, (uint8_t*)data->buf); return;
	cMutexLocker logMutexLocker(&i->m_logMutex);
	if (i->m_logger.Enabled())
	{
		i->m_logPackets.push_back(*data);
	}
}

bool InertialSense::Open(const char* port, int baudRate)
{
	// clear the device in preparation for auto-baud
	memset(&m_deviceInfo, 0, sizeof(m_deviceInfo));
	memset(&m_config, 0, sizeof(m_config));
	memset(&m_flashConfig, 0, sizeof(m_flashConfig));

	if (validateBaudRate(baudRate) != 0 || serialPortOpen(&m_serialPort, port, baudRate, 0) != 1)
	{
		return false;
	}

	// negotiate baud rate by querying device info - don't return out until it negotiates or times out
	// if the baud rate is already correct, the request for the message should succeed very quickly
	time_t startTime = time(0);

	// try to auto-baud for up to 10 seconds, then abort if we didn't get a valid packet
	// we wait until we get a valid serial number and manufacturer
	while ((m_flashConfig.size == 0 || m_deviceInfo.serialNumber == 0 || m_deviceInfo.manufacturer[0] == '\0') && time(0) - startTime < 10)
	{
		getDataComManagerInstance(m_cmHandle, 0, DID_CONFIG, 0, 0, 0);
		getDataComManagerInstance(m_cmHandle, 0, DID_DEV_INFO, 0, 0, 0);
		getDataComManagerInstance(m_cmHandle, 0, DID_FLASH_CONFIG, 0, 0, 0);
		SLEEP_MS(12);
		stepComManagerInstance(m_cmHandle);
		SLEEP_MS(12);
	}

	if (m_flashConfig.size == 0 || m_deviceInfo.serialNumber == 0 || m_deviceInfo.manufacturer[0] == '\0')
	{
		Close();
		return false;
	}

	return true;
}

bool InertialSense::SetLoggerEnabled(bool enable, const string& path, uint32_t logSolution, float maxDiskSpacePercent, uint32_t maxFileSize, uint32_t chunkSize, const string& subFolder)
{
	{
		cMutexLocker logMutexLocker(&m_logMutex);
		m_logger.CloseAllFiles();
		m_config.msgCfgBits = 1;
		m_config.sLogCtrl = logSolution;
		sendDataComManagerInstance(m_cmHandle, 0, DID_CONFIG, &m_config, 0, 0);
		getDataComManagerInstance(m_cmHandle, 0, DID_RAW_GPS_DATA, 0, 0, 200);
		getDataComManagerInstance(m_cmHandle, 0, DID_RTK_SOL, 0, 0, 200);
	}
	if (enable)
	{
		return EnableLogging(path, maxDiskSpacePercent, maxFileSize, chunkSize, subFolder);
	}
	DisableLogging();
	return true;
}

bool InertialSense::OpenServerConnectionRTCM3(const string& hostAndPort)
{
	size_t colon, colon2;
	if (!OpenServerConnection(hostAndPort, colon2))
	{
		return false;
	}

	m_parser = cGpsParser::CreateParser(GpsParserTypeRtcm3, this);

	// try to find url:user:pwd on the string
	colon = hostAndPort.find(':', colon2);
	if (colon != string::npos)
	{
		string url = hostAndPort.substr(colon2, colon - colon2);
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

	return true;
}

bool InertialSense::OpenServerConnectionInertialSense(const string& hostAndPort)
{
	if (OpenServerConnection(hostAndPort))
	{
		m_parser = cGpsParser::CreateParser(GpsParserTypeInertialSense, this);
		return true;
	}
	return false;
}

bool InertialSense::OpenServerConnectionUblox(const string& hostAndPort)
{
	if (OpenServerConnection(hostAndPort))
	{
		m_parser = cGpsParser::CreateParser(GpsParserTypeUblox, this);
		return true;
	}
	return false;
}

void InertialSense::CloseServerConnection()
{
	if (m_parser != NULL)
	{
		delete m_parser;
		m_parser = NULL;
	}
	m_tcpClient.Close();
	m_tcpServer.Close();
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
	int port = (int)strtol(portString.c_str(), NULL, 10);
    bool opened = (m_tcpServer.Open(host, port) == 0);
	if (opened)
	{
		// configure as RTK base station
		uint32_t cfgBits = SYS_CFG_BITS_RTK_BASE_STATION;
		sendDataComManagerInstance(m_cmHandle, m_pHandle, DID_FLASH_CONFIG, &cfgBits, sizeof(cfgBits), OFFSETOF(nvm_flash_cfg_t, sysCfgBits));
		cfgBits = 1; // raw messages
		sendDataComManagerInstance(m_cmHandle, m_pHandle, DID_CONFIG, &cfgBits, sizeof(cfgBits), OFFSETOF(config_t, msgCfgBits));
	}
	return opened;
}

bool InertialSense::IsOpen()
{
	return (serialPortIsOpen(&m_serialPort) != 0);
}

void InertialSense::Update()
{
	uint8_t buffer[4096];
	if (m_tcpClient.IsOpen())
	{
		int count = m_tcpClient.Read(buffer, sizeof(buffer));
		if (count > 0)
		{
			m_tcpByteCount += count;
			m_parser->Write(buffer, count);
		}
	}
	else if (m_tcpServer.IsOpen())
	{
		int count = serialPortRead(&m_serialPort, buffer, sizeof(buffer));
		if (count > 0)
		{
			m_tcpServer.Write(buffer, count);
			m_tcpByteCount += count;
		}
		m_tcpServer.Update();
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

void InertialSense::SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	sendDataComManagerInstance(m_cmHandle, m_pHandle, dataId, data, length, offset);
}

void InertialSense::SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset)
{
	sendRawDataComManagerInstance(m_cmHandle, m_pHandle, dataId, data, length, offset);
}

void InertialSense::SetConfig(const config_t& config)
{
	m_config = config;
	sendDataComManagerInstance(m_cmHandle, m_pHandle, DID_CONFIG, &m_config, sizeof(config), 0);
}

void InertialSense::SetFlashConfig(const nvm_flash_cfg_t& flashConfig)
{
	m_flashConfig = flashConfig;
	sendDataComManagerInstance(m_cmHandle, m_pHandle, DID_FLASH_CONFIG, &m_flashConfig, sizeof(flashConfig), 0);
}

bool InertialSense::BroadcastBinaryData(uint32_t dataId, int periodMS, pfnHandleBinaryData callback)
{
	if (m_comManagerState.binarySerialPort == NULL || dataId >= DID_COUNT)
	{
		return false;
	}

	if (callback == NULL) 
	{
		return false;
	}
	else
	{
		m_comManagerState.binaryCallback[dataId] = callback;
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

bool InertialSense::BootloadFile(const string& comPort, const string& fileName, pfnBootloadProgress uploadProgress, pfnBootloadProgress verifyProgress, char* errorBuffer, int errorBufferLength)
{
	// for debug
	// SERIAL_PORT_DEFAULT_TIMEOUT = 9999999;
	InertialSense bootloader;
	serialPortSetPort(&bootloader.m_serialPort, comPort.c_str());
	if (!enableBootloader(&bootloader.m_serialPort, errorBuffer, errorBufferLength))
	{
		serialPortClose(&bootloader.m_serialPort);
		return false;
	}
	bootload_params_t params;
	params.fileName = fileName.c_str();
	params.port = &bootloader.m_serialPort;
	params.error = errorBuffer;
	params.errorLength = errorBufferLength;
	params.obj = &bootloader;
	params.uploadProgress = uploadProgress;
	params.verifyProgress = verifyProgress;
	params.verifyFileName = NULL;
	params.flags.bitFields.enableVerify = (verifyProgress != NULL);
	if (!bootloadFileEx(&params))
	{
		serialPortClose(&bootloader.m_serialPort);
		return false;
	}
	serialPortClose(&bootloader.m_serialPort);
	return true;
}

bool InertialSense::OnPacketReceived(const cGpsParser* parser, const uint8_t* data, uint32_t dataLength)
{
    (void)parser;
	serialPortWrite(&m_serialPort, data, dataLength);
	return false; // do not parse, since we are just forwarding it on
}

bool InertialSense::OpenServerConnection(const string& hostAndPort, size_t& portEndIndex)
{
	// if no serial connection, fail
	if (!IsOpen())
	{
		return false;
	}

	CloseServerConnection();
	size_t colon = hostAndPort.find(':', 0);
	if (colon == string::npos)
	{
		return false;
	}
	size_t colon2 = hostAndPort.find(':', colon + 1);
	string host = hostAndPort.substr(0, colon);
	if (colon2 == string::npos)
	{
		colon2 = portEndIndex = hostAndPort.length();
	}
	else
	{
		portEndIndex = colon2 + 1;
	}
	string portString = hostAndPort.substr(colon + 1, colon2 - colon - 1);
	int port = (int)strtol(portString.c_str(), NULL, 10);
	bool opened = (m_tcpClient.Open(host, port) == 0);
	if (opened)
	{
		// configure as RTK rover
		uint32_t cfgBits = SYS_CFG_BITS_RTK_ROVER;
		sendDataComManagerInstance(m_cmHandle, m_pHandle, DID_FLASH_CONFIG, &cfgBits, sizeof(cfgBits), OFFSETOF(nvm_flash_cfg_t, sysCfgBits));
	}
	return opened;
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
	params.verifyFileName = NULL;
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
	case DID_INS_2:
		msgData["qn2b"] = convertArray(d.ins2.qn2b, 4);	// quaternion attitude 
		msgData["uvw"] = convertArray(d.ins2.uvw, 3);			// body velocities
		msgData["lla"] = convertArray(d.ins2.lla, 3);			// latitude, longitude, altitude
		break;
	case DID_INS_1:
		msgData["theta"] = convertArray(d.ins1.theta, 3);		// euler attitude
		msgData["lla"] = convertArray(d.ins1.lla, 3);			// latitude, longitude, altitude
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
	if (pyFunc != NULL)
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

template<typename Out>
void splitStringIterator(const string &s, char delim, Out result)
{
	stringstream ss;
	ss.str(s);
	string item;
	while (getline(ss, item, delim))
	{
		*(result++) = item;
	}
}


vector<string> splitString(const string &s, char delim)
{
	vector<string> elems;
	splitStringIterator(s, delim, back_inserter(elems));
	return elems;
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
			if (cISDataMappings::DataToString(i->second, NULL, (const uint8_t*)&flashConfig, stringBuffer))
			{
				cout << i->second.name << " = " << stringBuffer << endl;
			}
		}
		return false;
	}
	else
	{
		nvm_flash_cfg_t flashConfig = inertialSenseInterface.GetFlashConfig();
		vector<string> keyValues = splitString(flashConfigString, '|');
		for (size_t i = 0; i < keyValues.size(); i++)
		{
			vector<string> keyAndValue = splitString(keyValues[i], '=');
			if (keyAndValue.size() == 2)
			{
				if (flashMap.find(keyAndValue[0]) == flashMap.end())
				{
					cout << "Unrecognized flash config key '" << keyAndValue[0] << "' specified, ignoring." << endl;
				}
				else
				{
					const data_info_t& info = flashMap.at(keyAndValue[0]);
					cISDataMappings::StringToData(keyAndValue[1].c_str(), NULL, (uint8_t*)&flashConfig, info);
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
	if (m_comManagerState.binarySerialPort == NULL || dataId >= DID_COUNT)
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
