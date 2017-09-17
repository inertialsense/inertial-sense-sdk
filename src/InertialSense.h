/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __INTERTIALSENSE_H
#define __INTERTIALSENSE_H

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include <iostream>
#include <sstream>

#include "ISConstants.h"
#include "ISTcpClient.h"
#include "ISTcpServer.h"
#include "ISLogger.h"
#include "ISDisplay.h"
#include "ISUtilities.h"
#include "ISConstants.h"
#include "GpsParser.h"
#include "ISDataMappings.h"

extern "C"
{
	#include "data_sets.h"
	#include "com_manager.h"
	#include "serialPortPlatform.h"
	#include "inertialSenseBootLoader.h"
}

#if defined(ENABLE_IS_PYTHON_WRAPPER)

#include "../pySDK/pySDK.h"

#endif

class InertialSense;

typedef void(*pfnHandleBinaryData)(InertialSense* i, p_data_t* data);
typedef void(*pfnStepLogFunction)(InertialSense* i, const p_data_t* data);

using namespace std;

/*!
* Inertial Sense C++ interface
* Note only one instance of this class per process is supported
*/
class InertialSense : public iGpsParserDelegate
{
public:
	struct com_manager_cpp_state_t
	{
		serial_port_t* binarySerialPort;
		pfnHandleBinaryData binaryCallbackGlobal;
		pfnHandleBinaryData binaryCallback[DID_COUNT];
		pfnStepLogFunction stepLogFunction;
		dev_info_t* devInfo;
		config_t* config;
		nvm_flash_cfg_t* flashConfig;
		InertialSense* inertialSenseInterface;
	};

	/*!
	* Constructor
	* @param callback binary data callback, optional. If specified, ALL BroadcastBinaryData requests will callback
	* @param serial port, optional. If not NULL, it will be copied to an internal serial port and should have already been initialized
	*/
	InertialSense(pfnHandleBinaryData callback = NULL, serial_port_t* serialPort = NULL);

	/*!
	* Destructor
	*/
	virtual ~InertialSense();

	/*
	* Broadcast binary data
	* @param dataId the data id (DID_* - see data_sets.h) to broadcast
	* @param periodMS the period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
	* @param callback optional callback for this dataId
	* @return true if success, false if error - if callback is NULL and no global callback was passed to the constructor, this will return false
	*/ 
	bool BroadcastBinaryData(uint32_t dataId, int periodMS, pfnHandleBinaryData callback = NULL);

	/*!
	* Close the connection, logger and free all resources
	*/
	void Close();

	/*!
	* Get the current port name
	*/
	const char* GetPort();

	/*!
	* Check if the connection is open
	*/
	bool IsOpen();

	/*!
	* Closes any open connection and then opens the device
	* @param port the port to open
	* @param baudRate the baud rate to connect with - supported rates are 115200, 230400, 460800, 921600, 2000000, 3000000
	* @return true if opened, false if failure (i.e. baud rate is bad or port fails to open)
	*/
	bool Open(const char* port, int baudRate = IS_COM_BAUDRATE_DEFAULT);

	/*!
	*  Call in a loop to send and receive data.  Call at regular intervals as frequently as want to receive data.
	*/
	void Update();

	/*!
	* Enable or disable logging - logging is disabled by default
	* @param enable enable or disable the logger - disabling the logger after enabling it will close it and flush all data to disk
	* @param path the path to write the log files to
	* @param logSolution SLOG_DISABLED, SLOG_W_INS1 or SLOG_W_INS2. SLOG_W_INS1 and SLOG_W_INS2 write about 4-5 megabytes of data per second.
	* @param maxDiskSpacePercent the max disk space to use in percent of free space (0.0 to 1.0)
	* @param maxFileSize the max file size for each log file in bytes
	* @param chunkSize the max data to keep in RAM before flushing to disk in bytes
	* @param subFolder timestamp sub folder or empty for none
	* @return true if success, false if failure
	*/
	bool SetLoggerEnabled(bool enable, const string& path = cISLogger::g_emptyString, uint32_t logSolution = SLOG_W_INS2, float maxDiskSpacePercent = 0.5f, uint32_t maxFileSize = 1024 * 1024 * 5, uint32_t chunkSize = 131072, const string& subFolder = cISLogger::g_emptyString);

	/*!
	* Gets whether logging is enabled
	* @return whether logging is enabled
	*/
	bool LoggerEnabled() { return m_logger.Enabled(); }

	/*!
	* Connect to an RTCM3 server and send the data from that server to the uINS. Open must be called first to connect to the uINS unit.
	* @param hostAndPort the server to connect to with the host followed by colon, followed by the port, followed by colon, then optional url, user and password, i.e. 192.168.1.100:7777:RTCM3_Mount:user:password
	* @return true if connection opened, false if failure
	*/
	bool OpenServerConnectionRTCM3(const string& hostAndPort);

	/*!
	* Connect to an InertialSense server and send the data from that server to the uINS. Open must be called first to connect to the uINS unit.
	* @param hostAndPort the server to connect to with the host, then a colon, then the port, i.e. 192.168.1.100:7777
	*/
	bool OpenServerConnectionInertialSense(const string& hostAndPort);

	/*!
	* Connect to a ublox server and send the data from that server to the uINS. Open must be called first to connect to the uINS unit.
	* @param hostAndPort the server to connect to with the host, then a colon, then the port, i.e. 192.168.1.100:7777
	*/
	bool OpenServerConnectionUblox(const string& hostAndPort);

	/*!
	* Create a host that will stream data from the uINS to connected clients. Open must be called first to connect to the uINS unit.
	* @param ipAndPort ip address followed by colon followed by port. Ip address is optional and can be blank to auto-detect.
	* @return true if success, false if error
	*/
	bool CreateHost(const string& ipAndPort);

	/*!
	* Close any open connection to a server
	*/
	void CloseServerConnection();

	/*!
	* Turn off all messages
	*/
	void StopBroadcasts();

	/*!
	* Send data to the uINS - this is usually only used for advanced or special cases, normally you won't use this method
	* @param dataId the data id of the data to send
	* @param data the data to send
	* @param length length of data to send
	* @param offset offset into data to send at
	*/
	void SendData(eDataIDs dataId, uint8_t* data, uint32_t length, uint32_t offset);

	/*!
	* Send raw data to the uINS - this is usually only used for advanced or special cases, normally you won't use this method
	* @param dataId the data id of the data to send
	* @param data the data to send
	* @param length length of data to send
	* @param offset offset into data to send at
	*/
	void SendRawData(eDataIDs dataId, uint8_t* data, uint32_t length = 0, uint32_t offset = 0);

	/*!
	* Get the device info
	* @return the device info
	*/
	const dev_info_t& GetDeviceInfo() { return m_deviceInfo; }

	/*!
	* Get current device conviguration
	* @return current device configuration
	*/
	config_t GetConfig() { return m_config; }

	/*!
	* Set device configuration
	* @param config new device configuration
	*/
	void SetConfig(const config_t& config);

	/*!
	* Get the flash config, returns the latest flash config read from the uINS flash memory
	* @return the flash config
	*/
	nvm_flash_cfg_t GetFlashConfig() { return m_flashConfig; }

	/*!
	* Set the flash config and update flash config on the uINS flash memory
	* @param flashConfig the flash config
	*/
	void SetFlashConfig(const nvm_flash_cfg_t& flashConfig);

	/*!
	* Get the number of bytes read or written over tcp connection
	* @return byte count of tcp bytes read or written
	*/
	uint64_t GetTcpByteCount() { return m_tcpByteCount; }

	/*!
	* Get access to the underlying serial port
	* @return the serial port
	*/
	serial_port_t* GetSerialPort() { return &m_serialPort; }

	/*!
	* Bootload a file - if the bootloader fails, the device stays in bootloader mode and you must call BootloadFile again until it succeeds. If the bootloader gets stuck or has any issues, power cycle the device.
	* Please ensure that all other connections to the com port are closed before calling this function.
	* @param the com port to bootload
	* @param fileName the path of the file to bootload
	* @param uploadProgress optional callback for upload progress
	* @param verifyProgress optional callback for verify progress
	* @param errorBuffer receives any error messages
	* @param errorBufferLength the number of bytes available in errorBuffer
	*/
	static bool BootloadFile(const string& comPort, const string& fileName, pfnBootloadProgress uploadProgress = NULL, pfnBootloadProgress verifyProgress = NULL, char* errorBuffer = NULL, int errorBufferLength = 0);

protected:
	bool OnPacketReceived(const cGpsParser* parser, const uint8_t* data, uint32_t dataLength) OVERRIDE;

private:
	bool OpenServerConnection(const string& hostAndPort) { size_t tmp; return OpenServerConnection(hostAndPort, tmp); }
	bool OpenServerConnection(const string& hostAndPort, size_t& portEndIndex);

	InertialSense::com_manager_cpp_state_t m_comManagerState;
	serial_port_t m_serialPort;
	CMHANDLE m_cmHandle;
	int m_pHandle;
	cISLogger m_logger;
	void* m_logThread;
	cMutex m_logMutex;
	list<p_data_t> m_logPackets;
	time_t m_lastLogReInit;
	dev_info_t m_deviceInfo;
	config_t m_config;
	nvm_flash_cfg_t m_flashConfig;
	cISTcpClient m_tcpClient;
	cISTcpServer m_tcpServer;
	cGpsParser* m_parser; // parser so we can forward messages from a server to the uINS all at once - that way they aren't broken up in between other data set to the uINS
	uint64_t m_tcpByteCount;

	// returns false if logger failed to open
	bool EnableLogging(const string& path, float maxDiskSpacePercent, uint32_t maxFileSize, uint32_t chunkSize, const string& subFolder);
	void DisableLogging();
	static void LoggerThread(void* info);
	static void StepLogger(InertialSense* i, const p_data_t* data);

#if defined(ENABLE_IS_PYTHON_WRAPPER)

	pybind11::function m_pyCallback;
	cInertialSenseDisplay m_pyDisplay;

public:

	/*
	* Broadcast binary data
	* @param dataId the data id (DID_* - see data_sets.h) to broadcast
	* @param periodMS the period in milliseconds, 0 for one time message, less than 0 to disable broadcast of the specified dataId
	* @return true if success, false if error - if callback is NULL and no global callback was passed to the constructor, this will return false
	*/
	bool PyBroadcastBinaryData(uint32_t dataId, int periodMS);

	/*!
	* Bootload a file - if the bootloader fails, the device stays in bootloader mode and you must call BootloadFile again until it succeeds. If the bootloader gets stuck or has any issues, power cycle the device.
	* Please ensure that all other connections to the com port are closed before calling this function.
	* @param the com port to bootload
	* @param fileName the path of the file to bootload
	*/
	static bool InertialSense::PyBootloadFile(const string& comPort, const string& fileName);

	/*!
	* Set the pyCallback
	* @param python callback function
	*/
	void SetPyCallback(pybind11::object func);

	/*!
	* Set the Display variable for the pyCallback
	* @param python callback function
	*/
	void SetPyDisplay(cInertialSenseDisplay display);

	/*!
	* Get the pyCallback
	* @param python callback function
	*/
	pybind11::object GetPyCallback();

	/*!
	* Get the pyDisplay
	* @param python callback function
	*/
	cInertialSenseDisplay GetPyDisplay();

#endif

};

#if defined(ENABLE_IS_PYTHON_WRAPPER)

static void pyClt_dataCallback(InertialSense* i, p_data_t* data);
pybind11::dict py_dataHandling(p_data_t* data);
pybind11::list convertArray(float const data[], int length);
pybind11::list convertArray(double const data[], int length);
bool pyUpdateFlashConfig(InertialSense& inertialSenseInterface, string flashConfig);

#endif

#endif