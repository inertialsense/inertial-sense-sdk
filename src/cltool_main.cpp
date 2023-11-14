/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
*  main.cpp
*
*  (c) 2014 Inertial Sense, LLC
*
*  The Inertial Sense Command Line Tool (cltool) shows how easy it is to communicate with the uINS, log data, update firmware and more.
*
*  The following keywords are found within this file to identify instructions
*  for SDK implementation.  
*
*    KEYWORD:                  SDK Implementation
*    [C++ COMM INSTRUCTION]    - C++ binding API - InertialSense class with binary communication 
*                                protocol and serial port support for Linux and Windows.
*    [C COMM INSTRUCTION]      - C binding API - Com Manager with binary communication protocol.
*    [LOGGER INSTRUCTION]      - Data logger.
*    [BOOTLOADER INSTRUCTION]  - Firmware update feature.
*
*  This app is designed to be compiled in Linux and Windows.  When using MS
*  Visual Studio IDE, command line arguments can be supplied by right clicking 
*  the project in solution explorer and then selecting properties -> debugging -> command line arguments
*/

// Contains command line parsing and utility functions.  Include this in your project to use these utility functions.
#include "cltool.h"
#include "protocol_nmea.h"

#include <signal.h>

using namespace std;

static bool g_killThreadsNow = false;


static void display_server_client_status(InertialSense* i, bool server=false, bool showMessageSummary=false, bool refreshDisplay=false)
{
	if (g_inertialSenseDisplay.GetDisplayMode() == cInertialSenseDisplay::DMODE_QUIET ||
		g_inertialSenseDisplay.GetDisplayMode() == cInertialSenseDisplay::DMODE_SCROLL)
	{
		return;
	}

	static float serverKBps = 0;
	static uint64_t serverByteCount = 0;
	static uint64_t serverByteRateTimeMsLast = 0;
	static uint64_t serverByteCountLast = 0;
	static stringstream outstream;

	uint64_t newServerByteCount = i->GetClientServerByteCount();
	if (serverByteCount != newServerByteCount)
	{
		serverByteCount = newServerByteCount;

		// Data rate of server bytes
		uint64_t timeMs = getTickCount();
		uint64_t dtMs = timeMs - serverByteRateTimeMsLast;
		if (dtMs >= 1000)
		{
			uint64_t serverBytesDelta = serverByteCount - serverByteCountLast;
			serverKBps = ((float)serverBytesDelta / (float)dtMs);

			// Update history
			serverByteCountLast = serverByteCount;
			serverByteRateTimeMsLast = timeMs;
		}

		outstream.str("");	// clear
		outstream << "\n";
		if (server)
		{
			outstream << "Server: " << i->GetTcpServerIpAddressPort()   << "     Tx: ";
		}
		else
		{
			outstream << "Client: " << i->GetClientConnectionInfo()     << "     Rx: ";
		}
		outstream << fixed << setw(3) << setprecision(1) << serverKBps << " KB/s, " << (long long)i->GetClientServerByteCount() << " bytes    \n";

		if (server)
		{	// Server
			outstream << "Connections: " << i->GetClientConnectionCurrent() << " current, " << i->GetClientConnectionTotal() << " total    \n";
			if (showMessageSummary)
			{
 				outstream << i->getServerMessageStatsSummary();
			}
			refreshDisplay = true;
		}
		else
		{	// Client
			com_manager_status_t* status = comManagerGetStatus(0);
			if (status != NULLPTR && status->communicationErrorCount>2)
			{
				outstream << "Com errors: " << status->communicationErrorCount << "     \n";
			}
			if (showMessageSummary)
			{
				outstream << i->getClientMessageStatsSummary();
			}
		}
	}

	if (refreshDisplay)
	{
		cout << outstream.str();
	}
}


// [C++ COMM INSTRUCTION] STEP 5: Handle received data 
static void cltool_dataCallback(InertialSense* i, p_data_t* data, int pHandle)
{
	if (data->hdr.id != g_commandLineOptions.outputOnceDid && g_commandLineOptions.outputOnceDid)
	{
		return;
	}
    (void)i;
    (void)pHandle;

	// Print data to terminal
	g_inertialSenseDisplay.ProcessData(data);

#if 0

	// uDatasets is a union of all datasets that we can receive.  See data_sets.h for a full list of all available datasets. 
	uDatasets d = {};
	copyDataPToStructP(&d, data, sizeof(uDatasets));
    
	// Example of how to access dataset fields.
	switch (data->hdr.id)
	{
	case DID_INS_2:
		d.ins2.qn2b;		// quaternion attitude 
		d.ins2.uvw;			// body velocities
		d.ins2.lla;			// latitude, longitude, altitude
		break;
	case DID_INS_1:
		d.ins1.theta;		// euler attitude
		d.ins1.lla;			// latitude, longitude, altitude
		break;
	case DID_IMU:				
		d.imu3;      
		break;
	case DID_PIMU:		
		d.pImu;    
		break;
	case DID_GPS_NAV:				
		d.gpsNav;       
		break;
	case DID_MAGNETOMETER:		
		d.mag;          
		break;
	case DID_BAROMETER:				
		d.baro;         
		break;
	case DID_SYS_SENSORS:			
		d.sysSensors;   
		break;
	}
    
#endif
    
}

// Where we tell the uINS what data to send and at what rate.  
// "cltool_dataCallback()" is registered as the callback functions for all received data.
// All DID messages are found in data_sets.h
static bool cltool_setupCommunications(InertialSense& inertialSenseInterface)
{
	inertialSenseInterface.StopBroadcasts();	// Stop streaming any prior messages

	if (g_commandLineOptions.asciiMessages.size() != 0)
	{
		serialPortWriteAscii(inertialSenseInterface.GetSerialPort(), g_commandLineOptions.asciiMessages.c_str(), (int)g_commandLineOptions.asciiMessages.size());
		return true;
	}

	// ask for device info every 2 seconds
	inertialSenseInterface.BroadcastBinaryData(DID_DEV_INFO, 2000);

	// depending on command line options. stream various data sets
	if (g_commandLineOptions.datasetEdit.did)
	{	// Dataset to edit
		g_inertialSenseDisplay.SelectEditDataset(g_commandLineOptions.datasetEdit.did);
		inertialSenseInterface.BroadcastBinaryData(g_commandLineOptions.datasetEdit.did, g_commandLineOptions.datasetEdit.periodMultiple);
	}
	else while (g_commandLineOptions.datasets.size())
	{	// Datasets to stream
		inertialSenseInterface.BroadcastBinaryData(g_commandLineOptions.datasets.back().did, g_commandLineOptions.datasets.back().periodMultiple);
		switch (g_commandLineOptions.datasets.back().did)
		{
		case DID_RTOS_INFO:
			system_command_t cfg;
			cfg.command = SYS_CMD_ENABLE_RTOS_STATS;
			cfg.invCommand = ~cfg.command;
			inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
			break;
		}
		g_commandLineOptions.datasets.pop_back();
	}
	if (g_commandLineOptions.timeoutFlushLoggerSeconds > 0)
	{
		inertialSenseInterface.SetTimeoutFlushLoggerSeconds(g_commandLineOptions.timeoutFlushLoggerSeconds);
	}
	if (g_commandLineOptions.magRecal)
	{	
		// Enable broadcase of DID_MAG_CAL so we can observe progress and tell when the calibration is done (i.e. DID_MAG_CAL.state: 200=in progress, 201=done).
		inertialSenseInterface.BroadcastBinaryData(DID_MAG_CAL, 100);
		// Enable mag recal
		inertialSenseInterface.SendRawData(DID_MAG_CAL, (uint8_t*)&g_commandLineOptions.magRecalMode, sizeof(g_commandLineOptions.magRecalMode), offsetof(mag_cal_t, state));
	}
    if (g_commandLineOptions.surveyIn.state)
    {   // Enable mult-axis 
        inertialSenseInterface.SendRawData(DID_SURVEY_IN, (uint8_t*)&g_commandLineOptions.surveyIn, sizeof(survey_in_t), 0);
    }
	if (g_commandLineOptions.rmcPreset)
	{
		inertialSenseInterface.BroadcastBinaryDataRmcPreset(g_commandLineOptions.rmcPreset, RMC_OPTIONS_PRESERVE_CTRL);
	}
    if (g_commandLineOptions.persistentMessages)
    {   // Save persistent messages to flash
		cout << "Sending save persistent messages." << endl;
        inertialSenseInterface.SendRaw((uint8_t*)NMEA_CMD_SAVE_PERSISTENT_MESSAGES_TO_FLASH, NMEA_CMD_SIZE);
    }
    if (g_commandLineOptions.softwareResetImx)
    {   // Issue software reset
		cout << "Sending software reset." << endl;
        inertialSenseInterface.SendRaw((uint8_t*)NMEA_CMD_SOFTWARE_RESET, NMEA_CMD_SIZE);
    }
    if (g_commandLineOptions.softwareResetEvb)
    {   // Issue software reset to EVB
		cout << "Sending EVB software reset." << endl;
        uint32_t sysCommand = SYS_CMD_SOFTWARE_RESET;
        inertialSenseInterface.SendRawData(DID_EVB_STATUS, (uint8_t*)&sysCommand, sizeof(uint32_t), offsetof(evb_status_t, sysCommand));
    }
    if (g_commandLineOptions.chipEraseEvb2)
    {   // Chip erase EVB
		cout << "Sending EVB chip erase." << endl;
        uint32_t sysCommand;		
		sysCommand = SYS_CMD_MANF_UNLOCK;
        inertialSenseInterface.SendRawData(DID_EVB_STATUS, (uint8_t*)&sysCommand, sizeof(uint32_t), offsetof(evb_status_t, sysCommand));
        sysCommand = SYS_CMD_MANF_CHIP_ERASE;
        inertialSenseInterface.SendRawData(DID_EVB_STATUS, (uint8_t*)&sysCommand, sizeof(uint32_t), offsetof(evb_status_t, sysCommand));
    }
    if (g_commandLineOptions.sysCommand != 0)
    {   // Send system command to IMX
		cout << "Sending system command: " << g_commandLineOptions.sysCommand;
		switch(g_commandLineOptions.sysCommand)
		{
		case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS1:
		case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS2:
		case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER0:
		case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER1:
		case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER2:
		case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_TO_GPS1:
			cout << " Enable serial bridge"; break;
		case SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE:
			cout << "Disable serial bridge"; break;
		case SYS_CMD_MANF_FACTORY_RESET:            cout << " Factory Reset";           break;
		case SYS_CMD_MANF_CHIP_ERASE:               cout << " Chip Erase";              break;
		case SYS_CMD_MANF_DOWNGRADE_CALIBRATION:    cout << " Downgrade Calibration";   break;
		}
		cout << endl;
		system_command_t cfg;

		cfg.command = SYS_CMD_MANF_UNLOCK;
		cfg.invCommand = ~cfg.command;
		inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);

		cfg.command = g_commandLineOptions.sysCommand;
		cfg.invCommand = ~cfg.command;
		inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
		return false;
    }
	if (g_commandLineOptions.platformType >= 0 && g_commandLineOptions.platformType < PLATFORM_CFG_TYPE_COUNT)
	{	
		// Confirm 
		cout << "CAUTION!!!\n\nSetting the device(s) platform type in OTP memory.  This can only be done a limited number of times.\n\nPlatform: " << g_commandLineOptions.platformType << "\n\n";

		// Set platform type in OTP memory
		manufacturing_info_t manfInfo = {};
		manfInfo.key = 72720;
		manfInfo.platformType = g_commandLineOptions.platformType;
		// Write key (uint32_t) and platformType (int32_t), 8 bytes
		inertialSenseInterface.SendRawData(DID_MANUFACTURING_INFO, (uint8_t*)&manfInfo.key, sizeof(uint32_t)*2, offsetof(manufacturing_info_t, key));
		return false;
	}

	if (g_commandLineOptions.roverConnection.length() != 0)
	{
		vector<string> pieces;
		splitString(g_commandLineOptions.roverConnection, ':', pieces);
		if (pieces[0] != "TCP" &&
			pieces[0] != "SERIAL")
		{
			cout << "Invalid base connection, 1st field must be: TCP or SERIAL\n  -rover=" << g_commandLineOptions.roverConnection << endl;
			return false;
		}
		if (pieces[1] != "RTCM3" &&
			pieces[1] != "IS" &&
			pieces[1] != "UBLOX")
		{
			cout << "Invalid base connection, 2nd field must be: RTCM3, UBLOX, or IS\n  -rover=" << g_commandLineOptions.roverConnection << endl;
			return false;
		}

		if (!inertialSenseInterface.OpenConnectionToServer(g_commandLineOptions.roverConnection))
		{
			cout << "Failed to connect to server (base)." << endl;
		}
	}
	if (g_commandLineOptions.flashCfg.length() != 0)
	{
		return cltool_updateFlashCfg(inertialSenseInterface, g_commandLineOptions.flashCfg);
	}
	if (g_commandLineOptions.evbFlashCfg.length() != 0)
	{
		return cltool_updateEvbFlashCfg(inertialSenseInterface, g_commandLineOptions.evbFlashCfg);
	}
	return true;
}

std::vector<ISBootloader::cISBootloaderBase*> firmwareProgressContexts;

is_operation_result bootloadUpdateCallback(void* obj, float percent);
is_operation_result bootloadVerifyCallback(void* obj, float percent);

static int cltool_updateFirmware()
{
	// [BOOTLOADER INSTRUCTION] Update firmware
	if (g_commandLineOptions.updateBootloaderFilename.size() > 0)
	{
		cout << "Checking bootloader firmware: " << g_commandLineOptions.updateBootloaderFilename << endl;
	}
	cout << "Updating application firmware: " << g_commandLineOptions.updateAppFirmwareFilename << endl;
	
	firmwareProgressContexts.clear();

	if(InertialSense::BootloadFile(
		g_commandLineOptions.comPort,
		0,
        g_commandLineOptions.updateAppFirmwareFilename,
		g_commandLineOptions.updateBootloaderFilename,
		g_commandLineOptions.forceBootloaderUpdate,
        g_commandLineOptions.baudRate, 
		bootloadUpdateCallback,
		(g_commandLineOptions.bootloaderVerify ? bootloadVerifyCallback : 0),
		cltool_bootloadUpdateInfo,
		cltool_firmwareUpdateWaiter
	) == IS_OP_OK) return 0;
	
	return -1;
}

std::mutex print_mutex;

void printProgress()
{
	print_mutex.lock();

	int divisor = 0;
	float total = 0.0f;

	cISBootloaderThread::m_ctx_mutex.lock();

	for (size_t i = 0; i < cISBootloaderThread::ctx.size(); i++)
	{
		if (cISBootloaderThread::ctx[i] && cISBootloaderThread::ctx[i]->m_use_progress)
		{
			divisor++;

			if (!cISBootloaderThread::ctx[i]->m_verify)
			{
				total += cISBootloaderThread::ctx[i]->m_update_progress;
			}
			else
			{
				total += cISBootloaderThread::ctx[i]->m_update_progress * 0.5f;
				total += cISBootloaderThread::ctx[i]->m_verify_progress * 0.5f;
			}
		}
	}

	cISBootloaderThread::m_ctx_mutex.unlock();

	if (divisor) 
	{
		total /= divisor;
		int display = (int)(total * 100);
#if 0
		// Print progress in one spot using \r.  In some terminals it causes scolling of new lines.   
		printf("Progress: %d%%\r", display);	
		fflush(stdout);
#else
		// Print progress in condensed format.
		static int displayLast = 0;
#define DISPLAY_RES		5
		if (display == displayLast && display!=0)
		{
			printf("%d%% ", display);
		}
		fflush(stdout);

		while (display < displayLast)
		{	// Decrement
			displayLast -= DISPLAY_RES;
		}
		while (display >= displayLast)
		{	// Increment
			displayLast += DISPLAY_RES;
		}
#endif
	}

	print_mutex.unlock();
}

is_operation_result bootloadUpdateCallback(void* obj, float percent)
{
	if(obj)
	{
		ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase*)obj;
		ctx->m_update_progress = percent;
	}
	return g_killThreadsNow ? IS_OP_CANCELLED : IS_OP_OK;
}

is_operation_result bootloadVerifyCallback(void* obj, float percent)
{
	if(obj)
	{
		ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase*)obj;
		ctx->m_verify_progress = percent;
	}

	return g_killThreadsNow ? IS_OP_CANCELLED : IS_OP_OK;
}

void cltool_bootloadUpdateInfo(void* obj, const char* str, ISBootloader::eLogLevel level)
{
	print_mutex.lock();

	if(obj == NULL)
    {
		cout << str << endl;
		print_mutex.unlock();
        return;
    }

	ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase *)obj;

	if (ctx->m_sn != 0 && ctx->m_port_name.size() != 0)
	{
		printf("    | %s (SN%d):\r", ctx->m_port_name.c_str(), ctx->m_sn);
	}
	else if(ctx->m_sn != 0)
	{
		printf("    | (SN%d):\r", ctx->m_sn);
	}
	else if (ctx->m_port_name.size() != 0)
	{
		printf("    | %s:\r", ctx->m_port_name.c_str());
	}
	else
	{
		printf("    | SN?:\r");
	}
	
	printf("\t\t\t\t%s\r\n", str);
	print_mutex.unlock();
}

void cltool_firmwareUpdateWaiter()
{
	printProgress();
}

static int cltool_createHost()
{
	InertialSense inertialSenseInterface;
	if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate))
	{
		cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
		return -1;
	}
	else if (g_commandLineOptions.flashCfg.length() != 0 && !cltool_updateFlashCfg(inertialSenseInterface, g_commandLineOptions.flashCfg))
	{
		cout << "Failed to update flash config" << endl;
		return -1;
	}
	else if (g_commandLineOptions.evbFlashCfg.length() != 0 && !cltool_updateFlashCfg(inertialSenseInterface, g_commandLineOptions.evbFlashCfg))
	{
		cout << "Failed to update EVB flash config" << endl;
		return -1;
	}
	else if (!inertialSenseInterface.CreateHost(g_commandLineOptions.baseConnection))
	{
		cout << "Failed to create host at " << g_commandLineOptions.baseConnection << endl;
		return -1;
	}

	inertialSenseInterface.StopBroadcasts();

	unsigned int timeSinceClearMs = 0, curTimeMs;
	while (!g_inertialSenseDisplay.ExitProgram())
	{
		inertialSenseInterface.Update();
		curTimeMs = current_timeMs();
		bool refresh = false;
		if (curTimeMs - timeSinceClearMs > 2000 || curTimeMs < timeSinceClearMs)
		{	// Clear terminal
			g_inertialSenseDisplay.Clear();
			timeSinceClearMs = curTimeMs;
			refresh = true;
		}
		g_inertialSenseDisplay.Home();
		cout << g_inertialSenseDisplay.Hello();
		display_server_client_status(&inertialSenseInterface, true, true, refresh);
	}
	cout << "Shutting down..." << endl;

	// close the interface cleanly, this ensures serial port and any logging are shutdown properly
	inertialSenseInterface.Close();
	inertialSenseInterface.CloseServerConnection();
	
	return 0;
}

static void sigint_cb(int sig)
{
	g_killThreadsNow = true;
	cltool_bootloadUpdateInfo(NULL, "Update cancelled, killing threads and exiting...", ISBootloader::eLogLevel::IS_LOG_LEVEL_ERROR);
	signal(SIGINT, SIG_DFL);
}

static int inertialSenseMain()
{	
	// clear display
	g_inertialSenseDisplay.SetDisplayMode((cInertialSenseDisplay::eDisplayMode)g_commandLineOptions.displayMode);
	g_inertialSenseDisplay.SetKeyboardNonBlocking();

	// if replay data log specified on command line, do that now and return
	if (g_commandLineOptions.replayDataLog)
	{	
		// [REPLAY INSTRUCTION] 1.) Replay data log
		return !cltool_replayDataLog();
	}
	// if app firmware was specified on the command line, do that now and return
	else if (g_commandLineOptions.updateAppFirmwareFilename.length() != 0)
	{
		signal(SIGINT, sigint_cb);
		return cltool_updateFirmware();
	}
	else if (g_commandLineOptions.updateBootloaderFilename.length() != 0)
	{
		cout << "option -uf [FILENAME] must be used with option -ub [FILENAME] " << endl;
		return -1;
	}
	// if host was specified on the command line, create a tcp server
	else if (g_commandLineOptions.baseConnection.length() != 0)
	{
		return cltool_createHost();
	}
	else if (g_commandLineOptions.asciiMessages.size() != 0)
	{
		serial_port_t serialForAscii;
		serialPortPlatformInit(&serialForAscii);
		serialPortOpen(&serialForAscii, g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, 0);
		serialPortWriteAscii(&serialForAscii, "STPB", 4);
		serialPortWriteAscii(&serialForAscii, ("ASCB," + g_commandLineOptions.asciiMessages).c_str(), (int)(5 + g_commandLineOptions.asciiMessages.size()));
		unsigned char line[512];
		unsigned char* asciiData;
		while (!g_inertialSenseDisplay.ExitProgram())
		{
			int count = serialPortReadAsciiTimeout(&serialForAscii, line, sizeof(line), 100, &asciiData);
			if (count > 0)
			{
				printf("%s", (char*)asciiData);
				printf("\r\n");
			}
		}
	}
	// open the device, start streaming data and logging if needed
	else
	{
		// [C++ COMM INSTRUCTION] STEP 1: Instantiate InertialSense Class  
		// Create InertialSense object, passing in data callback function pointer.
		InertialSense inertialSenseInterface(cltool_dataCallback);

		// [C++ COMM INSTRUCTION] STEP 2: Open serial port
		if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, g_commandLineOptions.disableBroadcastsOnClose))
		{
			cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
			return -1;	// Failed to open serial port
		}

		// [C++ COMM INSTRUCTION] STEP 3: Enable data broadcasting
		if (cltool_setupCommunications(inertialSenseInterface))
		{
			// [LOGGER INSTRUCTION] Setup and start data logger
			if (g_commandLineOptions.asciiMessages.size() == 0 && !cltool_setupLogger(inertialSenseInterface))
			{
				cout << "Failed to setup logger!" << endl;
				inertialSenseInterface.Close();
				inertialSenseInterface.CloseServerConnection();
				return -1;
			}
			try
			{
				// Main loop. Could be in separate thread if desired.
				while (!g_inertialSenseDisplay.ExitProgram())
				{
					g_inertialSenseDisplay.GetKeyboardInput();

					if (g_inertialSenseDisplay.UploadNeeded())
					{
						cInertialSenseDisplay::edit_data_t *edata = g_inertialSenseDisplay.EditData();
						inertialSenseInterface.SendData(edata->did, edata->data, edata->info.dataSize, edata->info.dataOffset);
					}

					// [C++ COMM INSTRUCTION] STEP 4: Read data
					if (!inertialSenseInterface.Update())
					{	// device disconnected, exit
						break;
					}

					// Print to standard output
					bool refreshDisplay = g_inertialSenseDisplay.PrintData();

					// Collect and print summary list of client messages received
					display_server_client_status(&inertialSenseInterface, false, false, refreshDisplay);
				}
			}
			catch (...)
			{
				cout << "Unknown exception...";
			}
		}

		// [C++ COMM INSTRUCTION] STEP 6: Close interface
		// Close cleanly to ensure serial port and logging are shutdown properly.  (optional)
		inertialSenseInterface.Close();
		inertialSenseInterface.CloseServerConnection();
	}

	return 0;
}


int cltool_main(int argc, char* argv[])
{
	// Parse command line options
	if (!cltool_parseCommandLine(argc, argv))
	{
		// parsing failed
		return -1;
	}

	g_inertialSenseDisplay.setOutputOnceDid(g_commandLineOptions.outputOnceDid);

	// InertialSense class example using command line options
	int result = inertialSenseMain();
	if (result == -1)
	{
		cltool_outputHelp();

        // Pause so user can read error
        SLEEP_MS(2000);
	}

	g_inertialSenseDisplay.ShutDown();

	return result;
}
