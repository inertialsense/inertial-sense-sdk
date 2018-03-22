**SDK** - The Inertial Sense open source software development kit provides quick integration for communication with the Inertial Sense product line, including the uINS, uAHRS, and uINS.  It includes data logger, math libraries, and serial port interface for Linux and Windows environments.   

**EvalTool** - Graphical Windows-based desktop program that allows you to explore and test functionality of the Inertial Sense products in real-time.  It has scrolling plots, 3D model representation, table views of all data, data logger, and firmware updating interface for the uINS, uAHRS, or uIMU. The EvalTool can simultaneously interface with multiple Inertial Sense devices.

**CLTool** - Command line utility that can be used to communicate, log data, and update firmware for Inertial Sense products.  Additionally, InertialSenseCLTool serves as example source code to demonstrate how to integrate the Inertial Sense SDK into your own source code.  The InertialSenseCLTool can be compiled in Linux and Windows. 

### Documents

 * [SDK Documentation](http://docs.inertialsense.com/)
 * [User Manual, Datasheet, and Release Notes](https://github.com/inertialsense/InertialSenseSDK/releases)

### Downloads

 * [SDK & CLTool Binaries](https://github.com/inertialsense/InertialSenseSDK/releases) - pre-compiled release static libraries and executables for Linux and Windows.
 * [SDK & CLTool Source Code](https://github.com/inertialsense/InertialSenseSDK) - open source SDK repository with command line tool and example C/C++ source code.
 * [Firmware and EvalTool](https://github.com/inertialsense/InertialSenseSDK/releases) - uINS, uAHRS, and uIMU firmware and Windows application installer.

### Design Resources

 * [uINS PCB Design Libraries](https://inertialsense.com/resources) - Schematic and layout files for printed circuit board designs.
 * [uINS, uAHRS, uIMU CAD Model](https://inertialsense.com/resources) - 3D step model of uINS and EVB used for CAD and circuit board designs.

### Support

 * Email - support@inertialsense.com

************************************************
## CLTool Instructions

#### Compiling in Linux

1. Create build directory inside SDK/InertialSenseCLTool/.

        $ cd InertialSenseCLTool
        $ mkdir build

2. Run cmake from within build directory.

        $ cd build
        $ cmake ..

 to cross-compile to 32 or 64 bit:

        sudo apt-get install libc6-dev-i386
        sudo apt-get install gcc-multilib g++-multilib

 32 bit

        cmake .. -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_C_FLAGS=-m32

 64 bit

        cmake .. -DCMAKE_CXX_FLAGS=-m64 -DCMAKE_C_FLAGS=-m64

3. Compile using make.

        $ make

4. Add current user to the "dialout" group in order to read and write to the USB serial communication ports:

        $ sudo usermod -a -G dialout $USER
        $ sudo usermod -a -G plugdev $USER

 (**Reboot computer**)

5. Run executable

        $ ./bin/cltool

#### Compiling in Windows (MS Visual Studio)

1. Open Visual Studio solution file (InertialSenseCLTool.sln).

2. Build (F5 or F7).


************************************************
# SDK API

The InertialSense Software Development Kit (SDK) is used to communicate, data log, and update firmware with the Inertial Sense products.  The InertialSenseCLTool command line tool, included in the SDK, is provided as an example for both C++ and C implementation.  The following keywords are in the cltool_main.cpp and InertialSense.cpp files to identify the necessary steps for implementation.

| Keyword | SDK Implementation |
| ------------------- | :---------- |
| [C++ COMM INSTRUCTION] | C++ binding API - InertialSense class with binary communication protocol and serial port support for Linux and Windows. |
| [C COMM INSTRUCTION] | C binding API - Com Manager with binary communication protocol. |
| [LOGGER INSTRUCTION] | Data logger. |
| [BOOTLOADER INSTRUCTION] | Firmware update. |

### C++ Binding API

#### InertialSense C++ Class

The InertialSense C++ class, defined in InertialSense.h/.cpp, is provided to simplify the process of communicating with the binary protocol, logging, and updating firmware with the InertialSense products.  It is designed primarily around the Inertial Sense binary protocol.  

* Include the InertialSense header file.

```c
#include "InertialSense.h"
```

* Create InertialSense object and open serial port.

```c
// [C++ COMM INSTRUCTION] 1.) Create InertialSense object, passing in data callback function pointer.
InertialSense inertialSenseInterface(cltool_dataCallback);

// [C++ COMM INSTRUCTION] 2.) Open serial port.
if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str()))
{	
	cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
	return -1;	// Failed to open serial port
}
```

* Enable data broadcasting from the uINS.

```c
// [C++ COMM INSTRUCTION] 3.) Enable data broadcasting from uINS.
cltool_setupCommunications(inertialSenseInterface);
```

* Call the Update() method at regular intervals to send and receive data.

```c
// Main loop. Could be in separate thread if desired.
while (!g_inertialSenseDisplay.ControlCWasPressed())
{
	// [C++ COMM INSTRUCTION] 4.) Process data and messages.
	if (!inertialSenseInterface.Update())
	{
		// device disconnected, exit
		break;
	}
}
```

* New data is available in the data callback function.

```c
// [C++ COMM INSTRUCTION] 5.) This function is called every time there is new data.
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
```

* Close interface when done.

```c
// [C++ COMM INSTRUCTION] 6.) Close cleanly to ensure serial port and logging are shutdown properly. (optional)
inertialSenseInterface.Close();
```

#### Data Logging

* To configure and start the data logger.

```c
// [LOGGER INSTRUCTION] Setup and start data logger
if (!cltool_setupLogger(inertialSenseInterface))
{
	cout << "Failed to setup logger!" << endl;
	return -1;
}
```

### C Binding API

For pure C solutions such as embedded systems or situations where memory is limited, the C binding API is provided. The C binding contains an interface for communicating with the device using the binary protocol, as well as updating firmware (discussed later).

#### ComManager C Library

The ComManager pure C library provides a method to communicate, data log, and update firmware with the uINS, uAHRS, and uIMU.  It is designed primarily around the Inertial Sense binary protocol

* Include the Com Manager and data sets header files.

```c
// [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h  
#include "data_sets.h"
#include "com_manager.h"
```

* Initialized the Com Manager.

```c
// [C COMM INSTRUCTION]  1.) Setup com manager.  Specify number of serial ports and register callback functions for
// serial port read and write and for successfully parsed data.
initComManager((int)m_comManagerState.serialPorts.size(), 10, 10, 10, staticReadPacket, staticSendPacket, 0, staticProcessRxData, 0, 0);
```

* The stepComManager() function must be called at regular intervals to send and receive data.

```c
// Main loop.  Could be in separate thread if desired.
while (!g_ctrlCPressed)
{
	// [C COMM INSTRUCTION]  2.) Update Com Manager at regular interval to send and receive data.  
	// Normally called within a while loop.  Include a thread "sleep" if running on a multi-thread/
	// task system with serial port read function that does NOT incorporate a timeout.   
	stepComManager();
	// SLEEP_MS(1);
}
```

* Get data from the uINS.

```c
// [C COMM INSTRUCTION]  3.) Request a specific data set from the uINS.  "periodMs" specifies the interval
// between broadcasts and "periodMs=0" will disable broadcasts and transmit one single message. 
getDataComManager(i, dataId, 0, 0, periodMS);
```

* Send data to the uINS.

```c
// [C COMM INSTRUCTION]  4.) Send data to the uINS.  
sendDataComManager((int)i, dataId, data, length, offset);
```

* Disable all broadcasting and streaming from the uINS.

```c
// [C COMM INSTRUCTION]  Turns off (disable) all broadcasting and streaming on all ports from the uINS.
sendComManager((int)i, PID_STOP_ALL_BROADCASTS, 0, 0, 0);
```

* Turn off broadcasting of one DID message from the uINS.

```c
// [C COMM INSTRUCTION]  Stop broadcasting of one specific DID message from the uINS.
disableDataComManager(i, dataId);
```

* Turn off broadcasting of one DID message from the uINS.

```c
// [C COMM INSTRUCTION]  Stop broadcasting of one specific DID message from the uINS.
disableDataComManager(i, dataId);
```

* Enable solution data streaming for data logging.

```c
// [C COMM INSTRUCTION]  Enable solution streaming used for data logging.
// Recommended default is solStreamCtrl = SOL_STREAM_PPD1_INS2.
sendDataComManager((int)i, DID_CONFIG, &m_comManagerState.config[i].solStreamCtrl, sizeof(m_comManagerState.config[i].solStreamCtrl), OFFSETOF(config_t, solStreamCtrl));
```

#### Updating Firmware (Bootloader)

* To execute the bootloader, call cltool_runBootloader().  This function may be called directly or duplicated for your purposes.

```c
// [BOOTLOADER INSTRUCTIONS] Update firmware
return cltool_runBootloader(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.bootloaderFileName.c_str(), NULL);
```


************************************************
(c) 2014 Inertial Sense, LLC