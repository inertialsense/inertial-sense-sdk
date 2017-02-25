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
#include <thread>
#include <mutex>
#include <list>
#include <condition_variable>

#include "ISTcpClient.h"
#include "ISLogger.h"
#include "ISDisplay.h"
#include "ISUtilities.h"
#include "ISConstants.h"
#include "Rtcm3Reader.h"

extern "C"
{
	#include "data_sets.h"
	#include "com_manager.h"
	#include "serialPortPlatform.h"
	#include "inertialSenseBootLoader.h"
}

class InertialSense;

typedef void(*pfnHandleBinaryData)(InertialSense* i, p_data_t* data);
typedef void(*pfnStepLogFunction)(InertialSense* i, const p_data_t* data);

using namespace std;

/*!
* Inertial Sense C++ interface
* Note only one instance of this class per process is supported
*/
class InertialSense : public iRtcm3ReaderDelegate
{
public:
	struct com_manager_cpp_state_t
	{
		serial_port_t* binarySerialPort;
		pfnHandleBinaryData binaryCallbackGlobal;
		pfnHandleBinaryData binaryCallback[DID_COUNT];
		pfnStepLogFunction stepLogFunction;
		dev_info_t* devInfo;
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
	* Sets whether the device broadcasts standard solution messages (DID_INS2, DID_SYS_PARAMS, DID_EKF_STATES, DID_OBS_PARAMS, DID_GPS_POS)
	* @param enable whether to enable or disable standard solution messages
	*/
	void SetBroadcastSolutionEnabled(bool enabled);

	/*!
	* Close the connection, logger and free all resources
	*/
	void Close();

	/*!
	* Bootload a file and re-open the device - if the bootloader fails, the device stays in bootloader mode and you
	*  must call BootloadFile again until it succeeds. If the bootloader gets stuck or has any issues, power cycle the device.
	*  You must manually re-enable logging if you had it enabled before calling BootloadFile.
	* @param fileName the path of the file to bootload
	* @param uploadProgress optional callback for upload progress
	* @param verifyProgress optional callback for verify progress
	* @param errorBuffer receives any error messages
	* @param errorBufferLength the number of bytes available in errorBuffer
	*/ 
	bool BootloadFile(const string& fileName, pfnBootloadProgress uploadProgress = NULL, pfnBootloadProgress verifyProgress = NULL, char* errorBuffer = NULL, int errorBufferLength = 0);

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
	* @param maxDiskSpaceMB the max disk space to use in megabytes
	* @param maxFileSize the max file size for each log file in bytes
	* @param chunkSize the max data to keep in RAM before flushing to disk in bytes
	* @param useTimestampSubFolder whether to put the log in a sub-folder with the timestamp
	*/
	void SetLoggerEnabled(bool enable, const string& path = cISLogger::g_emptyString, uint32_t logSolution = SLOG_W_INS2, float maxDiskSpaceMB = 1024.0f, uint32_t maxFileSize = 1024 * 1024 * 5, uint32_t chunkSize = 131072, bool useTimestampSubFolder = false);

	/*!
	* Gets whether logging is enabled
	* @return whether logging is enabled
	*/
	bool LoggerEnabled() { return m_logger.Enabled(); }

	/*!
	* Connect to an RTCM3 server and send the data from that server to the uINS
	* @param hostAndPort the server to connect to with the host, then port information after a colon, followed by colon then optional url, user and password, i.e. 192.168.1.100:7777:RTCM3_Mount:user:password
	* @return true if connection opened, false if failure
	*/
	bool OpenServerConnectionRTCM3(const string& hostAndPort);

	/*!
	* Close any open connection to a server
	*/
	void CloseServerConnection();

	/*!
	* Turn off all messages
	*/
	void StopBroadcasts();

	/*!
	* Get the device info - if it is not yet known, the serial number will be 0
	*/
	const dev_info_t& GetDeviceInfo() { return m_deviceInfo; }

protected:
	bool OnPacketReceived(const cRtcm3Reader* reader, const uint8_t* data, uint32_t dataLength) override;

private:
	InertialSense::com_manager_cpp_state_t m_comManagerState;
	string m_asciiLine;
	serial_port_t m_serialPort;
	CMHANDLE m_cmHandle;
	cISLogger m_logger;
	thread *m_logThread;
	mutex m_logMutex;
	condition_variable m_logCondition;
	list<p_data_t> m_logPackets;
	uint32_t m_logSolution; // SLOG_DISABLED if none
	time_t m_lastLogReInit;
	dev_info_t m_deviceInfo;
	cISTcpClient m_tcpClient;
	cRtcm3Reader m_rtcm3Reader;

	void LoggerThread();
	void DisableLogging();
	static void StepLogger(InertialSense* i, const p_data_t* data);
};

#endif