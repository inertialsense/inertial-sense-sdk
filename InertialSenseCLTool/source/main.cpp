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
*  The Inertial Sense Command Line Tool (iscltool) shows how easy it is to 
*  communicate with the uINS, data log, or bootload firmware.  
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
void cltool_dataCallback(InertialSense* i, p_data_t* data)
{
	// Print data to terminal
	g_inertialSenseDisplay.ProcessData(data);

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
	case DID_DUAL_IMU:          d.dualImu;      break;
	case DID_DELTA_THETA_VEL:   d.dThetaVel;    break;
	case DID_IMU_1:             d.imu;          break;
	case DID_IMU_2:             d.imu;          break;
	case DID_GPS:               d.gps;          break;
	case DID_MAGNETOMETER_1:    d.mag;          break;
	case DID_MAGNETOMETER_2:    d.mag;          break;
	case DID_BAROMETER:         d.baro;         break;
	case DID_SYS_SENSORS:       d.sysSensors;   break;
	}
}

// Where we tell the uINS what data to send and at what rate.  
// "cltool_dataCallback()" is registered as the callback functions for all received data.
// All DID messages are found in data_sets.h
void cltool_setupCommunications(InertialSense& inertialSenseInterface)
{
	int periodMs = 50;
	inertialSenseInterface.StopBroadcasts();	// Stop streaming any prior messages

	// ask for device info every 2 seconds
	inertialSenseInterface.BroadcastBinaryData(DID_DEV_INFO, 2000, cltool_dataCallback);

	// depending on command line options. stream various data sets
	if (g_commandLineOptions.streamSol)
	{
		inertialSenseInterface.SetBroadcastSolutionEnabled(true);
	}
	if (g_commandLineOptions.streamINS1)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_1, periodMs, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamINS2)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_INS_2, periodMs, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamSysSensors)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_SYS_SENSORS, 100, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamDualIMU)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_DUAL_IMU, periodMs, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamIMU1)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_IMU_1, periodMs, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamIMU2)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_IMU_2, periodMs, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamDThetaVel)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_DELTA_THETA_VEL, periodMs, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamGPS)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_GPS, 200, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamMag1)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_MAGNETOMETER_1, periodMs, cltool_dataCallback);
	}
	if (g_commandLineOptions.streamBaro)
	{
		inertialSenseInterface.BroadcastBinaryData(DID_BAROMETER, periodMs, cltool_dataCallback);
	}
	if (g_commandLineOptions.serverHostAndPort.size() != 0)
	{
		inertialSenseInterface.OpenServerConnectionRTCM3(g_commandLineOptions.serverHostAndPort);
	}
}


int inertialSenseMain()
{	
	g_inertialSenseDisplay.SetDisplayMode(g_commandLineOptions.displayMode);

	// if replay data log specified on command line, do that now and return
	if (g_commandLineOptions.replayDataLog)
	{	
		return !cltool_replayDataLog();
	}

	// if bootloader was specified on the command line, do that now and return out
	if (g_commandLineOptions.bootloaderFileName.length() != 0)
	{
		// [BOOTLOADER INSTRUCTIONS] Update firmware
		char bootloaderError[1024];
		cout << "Bootloading file at " << g_commandLineOptions.bootloaderFileName << endl;
		InertialSense bootloader;
		bool success = bootloader.BootloadFile(g_commandLineOptions.comPort, g_commandLineOptions.bootloaderFileName, bootloadUploadProgress, bootloadVerifyProgress, bootloaderError, sizeof(bootloaderError));
		if (!success)
		{
			cout << "Error bootloading file " << g_commandLineOptions.bootloaderFileName << ", error: " << bootloaderError;
		}
		return (success ? 0 : -1);
	}

	// [COMM INSTRUCTION] 1.) Create InertialSense object and open serial port. 
	InertialSense inertialSenseInterface(cltool_dataCallback);
	if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate))
	{	
		cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
		return -1;	// Failed to open serial port
	}

	// [COMM INSTRUCTION] 2.) Enable data broadcasting from uINS
	cltool_setupCommunications(inertialSenseInterface);

	// [LOGGER INSTRUCTION] Setup and start data logger
	cltool_setupLogger(inertialSenseInterface);

	// catch ctrl-c, break, etc.
	cltool_setupCtrlCHandler();

	// clear display
	g_inertialSenseDisplay.Clear();

	try
	{
		// Main loop. Could be in separate thread if desired.
		while (!g_ctrlCPressed)
		{
			// [COMM INSTRUCTION] 3.) Process data and messages
			inertialSenseInterface.Update();

			// Specify the minimum time between read/write updates.
			SLEEP_MS(1);
		}
	}
	catch (...)
	{
		cout << "Unknown exception, ";
	}

	cout << "Shutting down..." << endl;

	// close the interface cleanly, this ensures serial port and any logging are shutdown properly
	inertialSenseInterface.Close();

	return 0;
}


int main(int argc, char* argv[])
{
	// Parse command line options
	if (!cltool_parseCommandLine(argc, argv))
	{
		// parsing failed
		return -1;
	}

	// InertialSense class example using command line options
	return inertialSenseMain();
}
