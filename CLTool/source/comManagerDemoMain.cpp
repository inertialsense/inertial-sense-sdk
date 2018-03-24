/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <iostream>
#include "comManagerDemoMain.h"
#include "../../src/InertialSenseSDK.h"

#if 0

/*
*  main.cpp
*
*  (c) 2015-2016 Inertial Sense, LLC
*
*  This project demonstrations implementation of the Inertial Sense SDK, which
*  includes the communications manager (Com Manager).  Certain keywords
*  found within this file identify step-by-step instructions that demonstrate
*  the SDK implementation.  Use your search tool to find each of these steps
*  and its corresponding description in this file.  The keywords are:
*
*     KEYWORD					- SDK IMPLEMENTATION
*     "// [STEP-COM MANAGER]"	- Com Manager**
*     "// [STEP-SERIAL PORT]"	- Serial port communications**.  If not running in Linux or Windows,
*								  these steps can be replaced by your own serial port driver calls.
*     "// [STEP-INS LOGGER]"	- INS logger, used for data analysis and post-processing
*     "// [STEP-LOGGER REPLAY]"	- Data log replay
*     "// [STEP-BOOTLOADER]"	- Firmware bootloader
*
*     **Typical for binary communications
*/

// [STEP-COM MANAGER] 1.) Include com_manager.h and data_sets.h header files.
#include "../../src/data_sets.h"
#include "../../src/com_manager.h"
// [STEP-LOGGER REPLAY] .)
// [STEP-INS LOGGER] 1.) Include ISLogger.h header file.
#include "../../src/ISLogger.h"
// [STEP-BOOTLOADER] 1.) Include InertialSense.h header file.
// [STEP-SERIAL PORT] 1.) Include InertialSense.h header file.
#include "../../src/InertialSense.h"
#include "../../src/ISUtilities.h"
#include <fstream>
using namespace std;

enum DemoMode
{
	DMODE_INS1 = 1,
	DMODE_INS2,
	DMODE_INS2_W_LOGGER,
	DMODE_INS_SOLUTION_W_LOGGER,
	DMODE_DATA_LOG_REPLAY,
	DMODE_BOOTLOADER,
};

#define PROTOCOL_BINARY			"B"
#define PROTOCOL_ASCII			"A"

#define SETTINGS_FILENAME		"settings.txt"
#define DEFAULT_DEMO_MODE		DMODE_INS1
#define DEFAULT_PROTOCOL		PROTOCOL_BINARY
#define DEFAULT_LOG_TYPE		0
#define DEMO_BAUD_RATE			BAUD_RATE_STANDARD

// global state variables
static InertialSense g_interface;	// for higher level access to protocol
									// [STEP-SERIAL PORT] 2.) Declare the serial port variable 
static serial_port_t g_serialPort;	// for binary protocol
static bool g_running = true;
static bool g_threadRunning = false;
static int g_demoMode = DEFAULT_DEMO_MODE;
static double g_replaySpeed = 1.0;

static cInertialSenseDisplay g_ISDisplay;


// [STEP-LOGGER REPLAY]
// [STEP-INS LOGGER] 2.) Instantiate solution logger. 
static cISLogger		g_ISlogger;


// callback functions - these must be implemented by you if you are reading BINARY data
int staticSendPacket(CMHANDLE cmHandle, int pHandle, buffer_t *packet);
int staticReadPacket(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len);
void staticProcessRxData(CMHANDLE cmHandle, int pHandle, p_data_t *data);


// reads from the serial port
static void communicationsThread()
{
	try
	{
		while (g_running)
		{
			// check if we are using the interface class
			if (g_interface.IsOpen())
			{
				g_interface.Update();
			}
			else
			{	// [STEP-COM MANAGER] 6.) Update the Com Manger interface by calling stepComManager() at regular intervals.  This will
				// process recieved data and call staticProcessRxData() through the Com Manger callback function.
				stepComManager();
			}

			// [STEP-SERIAL PORT] 6.) Sleep for period less than or equal to half data rate (i.e. 500ms for 1KHz data rates).  This is
			// necessary to prevent excessive processor loading.
			SLEEP_US(500);	// Necessary for 1KHz data rates
		}
	}
	catch (...)
	{
	}

	g_threadRunning = false;
}


// Reads from data log 
static void replayThread()
{
	try
	{
		// [STEP-LOGGER REPLAY] .) Iterate through devices (>1 if data sets from more than 1 uINS unit are in data log directory)
		for (int dev = 0; g_running && dev < (int)g_ISlogger.GetDeviceCount(); dev++)
		{
			p_data_t* data;

			// [STEP-LOGGER REPLAY] .) Read one data structure at a time
			while ((data = g_ISlogger.ReadData(dev)) && g_running)
			{
				if (data->hdr.size == 0)
					continue;

				staticProcessRxData(getGlobalComManager(), dev, data);
			}
		}
	}
	catch (...)
	{
	}

	// 	printf("\nThread terminated\n");
	g_threadRunning = false;
}


void broadcastSolutionData(uint32_t sLogCtrl)
{
	int handle = 0;

	// [STEP-INS LOGGER] 5.) Enable broadcast solution log if desired by setting sLogCtrl to SLOG_INIT_W_INS1 
	// or SLOG_INIT_W_INS2.  This will automatically broadcast the following messages which are used
	// for post-processing.  As such, these messages should NOT be individually enabled when the 
	// solution log is enabled.
	//		Message:			Broadcast Period (ms):
	//		DID_INS_MISC		0
	//		DID_SYS_PARAMS		0
	//		DID_FLASH_CONFIG	0
	//		DID_DEV_INFO		0
	//		DID_IMU				(solution period)
	//		DID_GPS_POS			200
	//		DID_INS_(1 or 2)	10
	//		DID_INS_PARAMS		10
	//		DID_SYS_PARAMS		10
	sendDataComManager(handle, DID_CONFIG, &sLogCtrl, sizeof(uint32_t), offsetof(config_t, sLogCtrl));
}


void stopCommunications()
{
	int handle = 0;

	// Turn off any existing broadcast messages, i.e. INS, IMU, GPS, etc.
	sendComManager(handle, PID_STOP_ALL_BROADCASTS, 0, 0, 0);

	// Disable Solution Data Broadcast
	broadcastSolutionData(SLOG_DISABLED);
}


// starts communicating with the device in either ASCII or binary mode
static void setupCommunications()
{
	// In binary mode, the SDK provides a communications manager that handles all the binary protocol for you
	// this is a little more low level but useful if you want a slightly more efficient transfer mechanism or can't use C++
	// in this example, we have only one global serial port, and that handle is 0
	int handle = 0;

	// [STEP-COM MANAGER] 2.) Initialize the Com Manager with 1 handle and register callback functions.  These callback functions
	// get called from stepComManager() and provide the serial port read and write interface.
	initComManager(1, 20, 10, 25, staticReadPacket, staticSendPacket, 0, staticProcessRxData, 0, 0);

	// [STEP-COM MANAGER] 3.) Disable any broadcast messages currently enabled.
	stopCommunications();

	// Wait for any data in serial pipeline, then flush serial buffer
	SLEEP_MS(200);
	serialPortFlush(&g_serialPort);

	// [STEP-COM MANAGER] 4.) Enable any broadcasting messages if desired.
	switch (g_demoMode)
	{
	case DMODE_INS1:
	case DMODE_INS2:
	case DMODE_INS2_W_LOGGER:
		// Enable more data broadcasting
		if (g_demoMode == DMODE_INS1)
			getDataComManager(handle, DID_INS_1, 0, 0, 50);			// INS1 data (w/ euler attitude) every 50 ms
		else
			getDataComManager(handle, DID_INS_2, 0, 0, 50);			// INS2 data (w/ quaternion attitude) every 50 ms
		getDataComManager(handle, DID_IMU_1, 0, 0, 50);				// IMU data every 50 ms
		getDataComManager(handle, DID_MAGNETOMETER_1, 0, 0, 100);	// Magnetometer data every 100 ms
		getDataComManager(handle, DID_BAROMETER, 0, 0, 100);			// Barometer data every 100 ms
		getDataComManager(handle, DID_GPS_POS, 0, 0, 250);	// GPS data every 250 ms

															// [STEP-COM MANAGER] 5.) Poll/request one-time messages if desired.
															// Poll/request entire message
		getDataComManager(handle, DID_DEV_INFO, 0, 0, 0);			// Device info (serial number, firmware version, etc.)

		// Poll/request portion of a message
		getDataComManager(handle, DID_SYS_SENSORS, offsetof(sys_sensors_t, temp), sizeof(float), 0);  // System temperature in Celsius

		break;

	case DMODE_INS_SOLUTION_W_LOGGER:
		// Enable broadcasting of solution data
		broadcastSolutionData(SLOG_W_INS2);
		break;
	}

	// Clear the display
	fflush(stdout);
	g_ISDisplay.Clear();
	g_ISDisplay.Home();

	g_threadRunning = true;
	thread t = thread(communicationsThread);
	t.detach();
}


// starts replay thread
static void setupReplay()
{
	g_ISDisplay.Clear();
	g_ISDisplay.Home();
	fflush(stdout);

	g_threadRunning = true;
	thread t = thread(replayThread);
	t.detach();
}


// for the binary protocol this writes to the serial port
int staticSendPacket(CMHANDLE cmHandle, int pHandle, buffer_t *packet)
{
	// Suppress compiler warnings
	(void)cmHandle;
	(void)pHandle;

	// [STEP-SERIAL PORT] 7.) Write to serial port.
	return serialPortWrite(&g_serialPort, packet->buf, packet->size);
}


// for the binary protocol this reads data from the serial port
int staticReadPacket(CMHANDLE cmHandle, int pHandle, unsigned char* buf, int len)
{
	// Suppress compiler warnings
	(void)cmHandle;
	(void)pHandle;

	// [STEP-SERIAL PORT] 8.) Read from serial port.
	return serialPortRead(&g_serialPort, buf, len);
}


// for the binary protocol, this processes a packet of data
void staticProcessRxData(CMHANDLE cmHandle, int pHandle, p_data_t *data)
{
	// Suppress compiler warnings
	(void)cmHandle;
	(void)pHandle;

	static int msgTimeMsOffset = 0;
	int msgTimeMs = 0;
	int curTimeMs = current_weekMs();

	// [STEP-INS LOGGER] 5.) Log all data to file from within the Com Manager processRxData callback function.
	g_ISlogger.LogData(pHandle, &(data->hdr), data->buf);

	// Ignore more than one device
	if (pHandle != 0)
	{
		return;
	}

	g_ISDisplay.ProcessData(data, g_demoMode == DMODE_DATA_LOG_REPLAY, g_replaySpeed);


	// [STEP-INS LOGGER] 6.) Monitor log size if desired.
	if (g_ISlogger.Enabled() && g_ISlogger.GetDeviceCount() > 0)
	{
		printf("Log:\t%.1f of %.1f MB,  %d file(s)  \n", g_ISlogger.FileSizeMB(), g_ISlogger.LogSizeMB(), g_ISlogger.FileCount());
	}
}


// main
int comManagerDemoMain(int argc, char* argv[])
{
	// Suppress compiler warnings
	(void)argc;
	(void)argv;

	// Welcome Screen
	g_ISDisplay.Clear();
	g_ISDisplay.Home();
	cout << endl << "================================================" << endl;
	cout << endl << "     $ Inertial Sense - DEMO" << endl;
	cout << endl << "     Micro Navigation Systems" << endl;
	cout << endl << "================================================" << endl;

	// Open Settings File
	ifstream ifile(SETTINGS_FILENAME);
	string serialPort, protocol = DEFAULT_PROTOCOL, firmwarePath, datalogPath, str;
	int logType = DEFAULT_LOG_TYPE;
	string backupPath;
	if (ifile.is_open())
	{	// Read from settings file
		if (!getline(ifile, serialPort))
			serialPort = DEFAULT_COM_PORT;
		if (getline(ifile, str) && str.size())
			g_demoMode = atoi(str.c_str());
		if (!getline(ifile, protocol))
			protocol = DEFAULT_PROTOCOL;
		if (!getline(ifile, firmwarePath))
			firmwarePath = "";
		if (!getline(ifile, datalogPath))
			datalogPath = "";
		if (getline(ifile, str) && str.size())
			g_replaySpeed = atof(str.c_str());
		if (getline(ifile, str) && str.size())
			logType = atoi(str.c_str());

		ifile.close();
	}
	else
	{	// File doesn't exist - Use Defaults
		serialPort = DEFAULT_COM_PORT;
		g_demoMode = DEFAULT_DEMO_MODE;
		protocol = DEFAULT_PROTOCOL;
		g_replaySpeed = 1.0;
	}
	if (g_demoMode <= 0 || g_demoMode == DMODE_BOOTLOADER)
		g_demoMode = DEFAULT_DEMO_MODE;

	// Demo Mode?
	cout << endl << "Enter demo mode: ";
	cout << endl << " " << DMODE_INS1 << " - INS1 data";
	cout << endl << " " << DMODE_INS2 << " - INS2 data";
	cout << endl << " " << DMODE_INS2_W_LOGGER << " - INS2 data w/ logger";
	cout << endl << " " << DMODE_INS_SOLUTION_W_LOGGER << " - Solution data (INS2, IMU, GPS, etc.) w/ logger";
	cout << endl << " " << DMODE_DATA_LOG_REPLAY << " - Data log replay";
	cout << endl << " " << DMODE_BOOTLOADER << " - Bootloader";
	cout << endl << "[" << g_demoMode << "]: ";
	cin.clear();
	getline(cin, str);
	if (str.size())
		g_demoMode = atoi(str.c_str());
	if (g_demoMode <= 0)
		g_demoMode = DEFAULT_DEMO_MODE;

	// Initialize serial port
	char port[50] = { 0 };
	switch (g_demoMode)
	{
	case DMODE_DATA_LOG_REPLAY:
		// Don't use serial port
		break;

	default:
		cout << endl << "Enter port [" << serialPort << "]: ";
		cin.clear();
		cin.getline(port, sizeof(port) / sizeof(char));

		if (strnlen(port, 50) > 0)
			serialPort = port;

		// [STEP-SERIAL PORT] 3.) Initialize the serial port callback functions and port name. 
		serialPortPlatformInit(&g_serialPort);
		serialPortSetPort(&g_serialPort, serialPort.c_str());

		if (!serialPortOpen(&g_serialPort, g_serialPort.port, DEMO_BAUD_RATE, 0))
		{
			cout << endl << "Failed to open serial port: " << serialPort << endl;
			return 1;
		}
		serialPortClose(&g_serialPort);
		break;
	}


	switch (g_demoMode)
	{
		//////////////////////////////////////////////////////////////////////////
	default:
	case DMODE_INS1:
	case DMODE_INS2:
	case DMODE_INS2_W_LOGGER:
	case DMODE_INS_SOLUTION_W_LOGGER:
		cout << endl << "Would you like ASCII (A) or Binary (B) communications protocol? [" << protocol << "]: ";
		cin.clear();
		getline(cin, str);
		if (str.size())
		{
			if (str[0] == 'a' || str[0] == 'A')
				protocol = PROTOCOL_ASCII;
			else
				protocol = PROTOCOL_BINARY;
		}

		switch (g_demoMode)
		{
		case DMODE_INS2_W_LOGGER:
		case DMODE_INS_SOLUTION_W_LOGGER:
			cout << endl << "Would you like .dat (0), .sdat (1), or .csv (2) log files? [" << logType << "]: ";
			cin.clear();
			getline(cin, str);
			if (str.size())
				logType = atoi(str.c_str());
			break;
		}

		if (str.size())
		{
			if (str[0] == 'a' || str[0] == 'A')
				protocol = PROTOCOL_ASCII;
			else
				protocol = PROTOCOL_BINARY;
		}


		if (protocol == PROTOCOL_ASCII)
		{
			g_interface.Open(g_serialPort.port);
		}
		else
		{
			if (g_demoMode == DMODE_INS2_W_LOGGER ||
				g_demoMode == DMODE_INS_SOLUTION_W_LOGGER)
			{
				// [STEP-INS LOGGER] 3.) Initialize logger for saving data.  Default file location is DEFAULT_LOGS_DIRECTORY.
				g_ISlogger.InitSave((cISLogger::eLogType)logType);

				// [STEP-INS LOGGER] 4.) Enable logging.  This can be toggled anytime to stop/start logging.
				g_ISlogger.EnableLogging(true);

				datalogPath = g_ISlogger.LogDirectory();
			}

			// [STEP-SERIAL PORT] 4.) Open serial port.
			serialPortOpen(&g_serialPort, g_serialPort.port, DEMO_BAUD_RATE, 0);	// Non-Blocking
																					// 			serialPortOpen( &g_serialPort, g_serialPort.port, DEMO_BAUD_RATE, 1 );
		}

		// [STEP-SERIAL PORT] 5.) Verify serial port opened.
		if (!g_interface.IsOpen() && !serialPortIsOpen(&g_serialPort))
			cout << endl << "Failed to open serial port: " << port << endl;
		else
			setupCommunications();

		// block until ENTER is pressed
		while (getchar() != '\n') { SLEEP_MS(250); }

		if (g_demoMode == DMODE_INS2_W_LOGGER ||
			g_demoMode == DMODE_INS_SOLUTION_W_LOGGER)
		{
			// [STEP-INS LOGGER] 7.) Close all log files before exiting program.
			g_ISlogger.CloseAllFiles();
		}

		g_running = false;

		while (g_threadRunning) { SLEEP_MS(100); }
		break;


	case DMODE_DATA_LOG_REPLAY:
		// Query for directory
		cout << "\nDirectory containing data log file(s) [" << datalogPath << "]: ";
		cin.clear();
		backupPath = datalogPath;
		getline(cin, datalogPath);
		if (!datalogPath.size())
			datalogPath = backupPath;

		// Query replay speed
		cout << "\nReplay speed [" << g_replaySpeed << "]: ";
		cin.clear();
		getline(cin, str);
		if (str.size())
			g_replaySpeed = atof(str.c_str());
		if (g_replaySpeed < 0.0f || g_replaySpeed > 1000)
			g_replaySpeed = 1.0;

		// [STEP-LOGGER REPLAY] .) Initialize data logger to read logs from given directory
		if (datalogPath.size() &&
			g_ISlogger.LoadFromDirectory(datalogPath))
			setupReplay();

		// block until ENTER is pressed
		while (getchar() != '\n') { SLEEP_MS(100); }
		g_running = false;
		while (g_threadRunning) { SLEEP_MS(100); }
		break;


		//////////////////////////////////////////////////////////////////////////
	case DMODE_BOOTLOADER:
		char error[128];

		cout << endl;
		cout << "Firmware hex file path [" << firmwarePath << "]: ";
		cin.clear();
		backupPath = firmwarePath;
		getline(cin, firmwarePath);
		if (!firmwarePath.size())
			firmwarePath = backupPath;

		if (firmwarePath.size())
		{
			// Run Bootloader
			cout << endl << "Please wait...";
			fflush(stdout);
			// [STEP-BOOTLOADER] 1.) - Put the device into bootloader mode.
			enableBootloader(&g_serialPort, error, sizeof(error) / sizeof(error[0]));
			if (error[0] != '\0')
			{
				printf("Error enabling boot loader: %s", error);
				firmwarePath = backupPath;
			}
			else
			{
				// [STEP-BOOTLOADER] 2.) - Upload new firmware.
				bootloadFile(firmwarePath.c_str(), &g_serialPort, error, sizeof(error) / sizeof(error[0]), g_serialPort.port, bootloadUploadProgress, bootloadVerifyProgress);
				if (error[0] != '\0')
				{
					printf("Bootload error: %s\n", error);
					firmwarePath = backupPath;
				}
			}
		}
		else
		{
			cout << endl << "Invalid bootloader file path!" << endl;
			firmwarePath = backupPath;
		}

		// block until ENTER is pressed
		cout << "Press ENTER to quit\n";
		while (getchar() != '\n') { SLEEP_MS(100); }

		// Disable any broadcast messages
		stopCommunications();
		break;
	}

	g_ISDisplay.Goodbye();

	// [STEP-SERIAL PORT] 9.) Close serial port.
	serialPortClose(&g_serialPort);

	// Save Settings File
	ofstream ofile;
	ofile.open("settings.txt");
	ofile << serialPort << endl;
	ofile << g_demoMode << endl;
	ofile << protocol << endl;
	ofile << firmwarePath << endl;
	ofile << datalogPath << endl;
	ofile << g_replaySpeed << endl;
	ofile << logType << endl;
	ofile.close();

	return 0;
}

#endif