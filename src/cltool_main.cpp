/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

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
*    KEYWORD:                
*    [COMM INSTRUCTION]        - InertialSense class implementation with binary communication protocol 
*                                and serial port support for Linux and Windows.
*    [LOGGER INSTRUCTION]      - Data logger
*    [BOOTLOADER INSTRUCTIONS] - Firmware update feature
*
*  This app is designed to be compiled in Linux and Windows.  When using MS
*  Visual Studio IDE, command line arguments can be supplied by right clicking 
*  the project in solution explorer and then selecting properties -> debugging -> command line arguments
*/

// Contains command line parsing and utility functions.  Include this in your project to use these utility functions.
#include "cltool.h"

// [COMM INSTRUCTION] 4.) This function is called every time there is new data.
static void cltool_dataCallback(InertialSense* i, p_data_t* data, int pHandle)
{
	// Print data to terminal
	g_inertialSenseDisplay.ProcessData(data);
}

// Where we tell the uINS what data to send and at what rate.  
// "cltool_dataCallback()" is registered as the callback functions for all received data.
// All DID messages are found in data_sets.h
static bool cltool_setupCommunications(InertialSense& inertialSenseInterface)
{
	int periodMs = 50;
	int streamingMessageCount = 0;
	inertialSenseInterface.StopBroadcasts();	// Stop streaming any prior messages

	// ask for device info every 2 seconds
	inertialSenseInterface.BroadcastBinaryData(DID_DEV_INFO, 2000);

	// depending on command line options. stream various data sets
	if (g_commandLineOptions.streamINS1)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_1, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamINS2)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_2, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamINS3)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_3, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamINS4)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_4, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamSysSensors)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_SYS_SENSORS, 100);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamDualIMU)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_DUAL_IMU, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamIMU1)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_IMU_1, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamIMU2)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_IMU_2, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamDThetaVel)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_DELTA_THETA_VEL, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamGPS)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_GPS, 200);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamMag1)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_MAGNETOMETER_1, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamMag2)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_MAGNETOMETER_2, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamBaro)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_BAROMETER, periodMs);
		streamingMessageCount++;
	}
	if (g_commandLineOptions.streamRTOS)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_RTOS_INFO, 250);
		uint32_t enRTOSStats = 1;
		inertialSenseInterface.SendRawData(DID_CONFIG, (uint8_t*)&enRTOSStats, sizeof(enRTOSStats), OFFSETOF(config_t, enRTOSStats));
		streamingMessageCount++;
	}

	// stream default post processing messages if no other message were specified and no solution streaming option was specified
	if (g_commandLineOptions.solStreamCtrl == 0xFFFFFFFF)
	{
		if (streamingMessageCount == 0)
		{
			g_commandLineOptions.solStreamCtrl = SOL_STREAM_PPD1_INS2;
		}
		else
		{
			g_commandLineOptions.solStreamCtrl = 0;
		}
	}
	if (g_commandLineOptions.solStreamCtrl != 0)
	{
		// turn on solution stream for each device
		inertialSenseInterface.SetSolutionStream(g_commandLineOptions.solStreamCtrl);
	}

	if (g_commandLineOptions.serverConnection.length() != 0)
	{
		if (g_commandLineOptions.serverConnection.find("RTCM3:") == 0 ||
			g_commandLineOptions.serverConnection.find("IS:") == 0 ||
			g_commandLineOptions.serverConnection.find("UBLOX:") == 0)
		{
			if (!inertialSenseInterface.OpenServerConnection(g_commandLineOptions.serverConnection))
			{
				cout << "Failed to connect to server." << endl;
			}
		}
		else
		{
			cout << "Invalid server connection, must prefix with RTCM3:, IS: or UBLOX:, " << g_commandLineOptions.serverConnection << endl;
			return false;
		}
	}
	if (g_commandLineOptions.flashConfig.length() != 0)
	{
		return cltool_updateFlashConfig(inertialSenseInterface, g_commandLineOptions.flashConfig);
	}
	return true;
}

static int cltool_runBootloader()
{
	// [BOOTLOADER INSTRUCTION] Update firmware
	char bootloaderError[1024];
	cout << "Bootloading file at " << g_commandLineOptions.bootloaderFileName << endl;
	bool success = InertialSense::BootloadFile(g_commandLineOptions.comPort, g_commandLineOptions.bootloaderFileName, bootloadUploadProgress,
		(g_commandLineOptions.bootloaderVerify ? bootloadVerifyProgress : 0), bootloaderError, sizeof(bootloaderError));
	if (!success)
	{
		cout << "Error bootloading file " << g_commandLineOptions.bootloaderFileName << ", error: " << bootloaderError << endl;
	}
	return (success ? 0 : -1);
}

static int cltool_createHost()
{
	InertialSense inertialSenseInterface;
	if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate))
	{
		cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
		return -1;	// Failed to open serial port
	}
	else if (g_commandLineOptions.flashConfig.length() != 0 && !cltool_updateFlashConfig(inertialSenseInterface, g_commandLineOptions.flashConfig))
	{
		return -1;
	}
	else if (!inertialSenseInterface.CreateHost(g_commandLineOptions.host))
	{
		cout << "Failed to create host at " << g_commandLineOptions.host << endl;
		return -1; // Failed to open host
	}
	while (!g_inertialSenseDisplay.ControlCWasPressed())
	{
		inertialSenseInterface.Update();
		g_inertialSenseDisplay.Home();
		cout << "Server bytes: " << inertialSenseInterface.GetClientServerByteCount() << "   ";
		SLEEP_MS(1);
	}
	cout << "Shutting down..." << endl;

	// close the interface cleanly, this ensures serial port and any logging are shutdown properly
	inertialSenseInterface.Close();
	
	return 0;
}

static int inertialSenseMain()
{	
	// clear display
	g_inertialSenseDisplay.SetDisplayMode(g_commandLineOptions.displayMode);
	g_inertialSenseDisplay.Clear();

	// if replay data log specified on command line, do that now and return
	if (g_commandLineOptions.replayDataLog)
	{	
		// [REPLAY INSTRUCTION] 1.) Replay data log
		return !cltool_replayDataLog();
	}

	// if bootloader was specified on the command line, do that now and return out
	else if (g_commandLineOptions.bootloaderFileName.length() != 0)
	{
		// [BOOTLOADER INSTRUCTION] 1.) Run bootloader
		return cltool_runBootloader();
	}

	// if host was specified on the command line, create a tcp server
	else if (g_commandLineOptions.host.length() != 0)
	{
		return cltool_createHost();
	}

	// open the device, start streaming data and logging
	else
	{
		// [COMM INSTRUCTION] 1.) Create InertialSense object and open serial port
		InertialSense inertialSenseInterface(cltool_dataCallback);
		if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, g_commandLineOptions.disableBroadcastsOnClose))
		{
			cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
			return -1;	// Failed to open serial port
		}

		// [COMM INSTRUCTION] 2.) Enable data broadcasting from uINS
		if (cltool_setupCommunications(inertialSenseInterface))
		{
			// [LOGGER INSTRUCTION] Setup and start data logger
			if (!cltool_setupLogger(inertialSenseInterface))
			{
				cout << "Failed to setup logger!" << endl;
				inertialSenseInterface.Close();
				return -1;
			}
			try
			{
				// Main loop. Could be in separate thread if desired.
				while (!g_inertialSenseDisplay.ControlCWasPressed())
				{
					// [COMM INSTRUCTION] 3.) Process data and messages
					if (!inertialSenseInterface.Update())
					{
						// device disconnected, exit
						break;
					}
					else if (inertialSenseInterface.GetClientServerByteCount() != 0)
					{
						g_inertialSenseDisplay.GoToRow(1);
						cout << "Client bytes: " << inertialSenseInterface.GetClientServerByteCount() << "   ";
					}

					// Specify the minimum time between read/write updates.
					SLEEP_MS(1);
				}
			}
			catch (...)
			{
				cout << "Unknown exception, ";
			}
		}

		cout << "Shutting down..." << endl;

		// close the interface cleanly, this ensures serial port and any logging are shutdown properly
		inertialSenseInterface.Close();
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

	// InertialSense class example using command line options
	if (inertialSenseMain() == -1)
	{
		cltool_outputHelp();
		return -1;
	}
	
	return 0;
}
