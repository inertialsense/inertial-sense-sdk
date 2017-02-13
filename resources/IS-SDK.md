
**InertialSenseLibs** - are the Inertial Sense source code libraries that provide quick integration to communicate with the Inertial Sense product line, including the uINS, uAHRS, and uINS.  It includes data logger, math libraries, and serial port interface for Linux, Mac, and Windows environments.   

**EvalTool** - is a graphical Windows-based desktop program that allows you to explore and test functionality of the Inertial Sense products in real-time.  It has scrolling plots, 3D model representation, table views of all data, data logger, and firmware updating interface for the uINS, uAHRS, or uIMU. The EvalTool can simultaneously interface with multiple Inertial Sense devices.

**InertialSenseCLTool** - is a command line utility that can be used to communicate, log data, and update firmware for Inertial Sense products.  Additionally, InertialSenseCLTool serves as example source code to demonstrate how to integrate the Inertial Sense SDK into your own source code.  The InertialSenseCLTool can be compiled in Linux, Mac, and Windows. 

### Documents

 * [User Manual](https://inertialsense.com/download/uins-datasheet/)
 * [Datasheet - uINS, uAHRS, and uIMU](https://inertialsense.com/download/uins-datasheet/)
 * [Release Notes](https://inertialsense.com/download/release-notes) - version information and software feature change log.

### Source Code, SDK and Tools

 * [EvalTool](https://inertialsense.com/download/eval-tool-installer/) - windows installer.
 * [SDK with InertialSenseCLTool](https://inertialsense.com/download/eval-tool-installer/) - libraries, command line tool, and example source code in C/C++
 * [Firmware](https://inertialsense.com/download/eval-tool-installer/) - for uINS, uAHRS, and uIMU hardware v2 and v3.  See [release-notes](https://inertialsense.com/download/release-notes) for details.

### Design Resources

 * [uINS, uAHRS, uIMU v2 CAD Model](https://inertialsense.com/download/eval-tool-installer/) - 3D step model of uINS 2 used for CAD and circuit board designs.

### Support

 * Email - support@inertialsense.com

************************************************
## Instructions

### InertialSenseCLTool Use

#### Compiling in Linux / Mac
1. Create build directory...
~~~~~~~~~~~~~{.c}
$ cd InertialSenseCLTool
$ mkdir build
~~~~~~~~~~~~~
2. Run cmake from within build directory.
~~~~~~~~~~~~~{.c}
$ cd build
$ cmake ..
~~~~~~~~~~~~~
3. Compile using make.
~~~~~~~~~~~~~{.c}
$ make
~~~~~~~~~~~~~
4. Add current user to the "dialout" group in order to read and write to the USB serial communication ports:
~~~~~~~~~~~~~{.c}
$ sudousermod -a -G dialout $USER
$ sudousermod -a -G plugdev $USER
~~~~~~~~~~~~~
(reboot computer)
5. Run executable
~~~~~~~~~~~~~{.c}
$ ./bin/iscltool
~~~~~~~~~~~~~

#### Compiling in Windows (MS Visual Studio)
1. Open Visual Studio solution file (InertialSenseCLTool.sln).
2. Build (F7).


************************************************
## SDK Instructions

### InertialSense Class
The InertialSense class provides a simple, powerful, and convenient method to communicate, data log, and bootload firmware with the uINS, uAHRS, and uIMU.  It is designed primarily around the Inertial Sense binary protocol.  Please refer to the **InertialSenseCLTool** project files **main.cpp** and **cltool.cpp** with the follow instructions.

#### [COMM INSTRUCTIONS]
1. Create InertialSense object and open serial port.
~~~~~~~~~~~~~{.c}
	// [COMM INSTRUCTION] 1.) Create InertialSense object and open serial port. 
	InertialSense inertialSenseInterface(getGlobalComManager(), cltool_dataCallback);
~~~~~~~~~~~~~
2. Enable data broadcasting from uINS.
~~~~~~~~~~~~~{.c}
	// [COMM INSTRUCTION] 2.) Enable data broadcasting from uINS
	cltool_setupCommunications(inertialSenseInterface);
~~~~~~~~~~~~~
3. The Update() function must be called at regular intervals to send and receive data. 
~~~~~~~~~~~~~{.c}
	// Main loop.  Could be in separate thread if desired.
	while (!g_ctrlCPressed)
	{
		// [COMM INSTRUCTION] 3.) Process data and messages
		inertialSenseInterface.Update();

		// Specify the minimum time between read/write updates.
		SLEEP_MS(1);
	}
~~~~~~~~~~~~~
4. New data is available in the data callback function.
~~~~~~~~~~~~~{.c}
// [COMM INSTRUCTION] 4.) This function is called every time there is new data.
void cltool_dataCallback(InertialSense* i, p_data_t* data)
{
	// Print data to terminal
	g_inertialSenseDisplay.ProcessData(data);

	// Access data received here
	uDatasets d = {};
	copyDataPToStructP(&d, data, sizeof(uDatasets));
	switch (data->hdr.id)
	{
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
~~~~~~~~~~~~~

#### [LOGGER INSTRUCTIONS]
The steps described in the COMM INSTRUCTIONS section are needed in addition to those in this section.
1. To configure and start the data logger, call cltool_setupLogger().  This function may be called directly or duplicated for your purposes.
~~~~~~~~~~~~~{.c}
	// [LOGGER INSTRUCTION] Setup data logger
	cltool_setupLogger(inertialSenseInterface);
~~~~~~~~~~~~~

#### [BOOTLOADER INSTRUCTIONS]
The steps described in the COMM INSTRUCTIONS section are needed in addition to those in this section.
1. To execute the bootloader, call cltool_runBootloader().  This function may be called directly or duplicated for your purposes.
~~~~~~~~~~~~~{.c}
	// [BOOTLOADER INSTRUCTIONS] Update firmware
	return cltool_runBootloader(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.bootloaderFileName.c_str(), NULL);
~~~~~~~~~~~~~


