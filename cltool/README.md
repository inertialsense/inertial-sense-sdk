# C++ API - Inertial Sense Class and CLTool Example Project

The <a href="https://github.com/inertialsense/inertial-sense-sdk/blob/release/src/InertialSense.cpp">InertialSense C++ class</a>, defined in InertialSense.h/.cpp, provides all SDK capabilities including serial communications, data logging to file, and embedded firmware update for <a href="https://inertialsense.com">InertialSense</a> products.

## CLTool Example

The <a href="https://github.com/inertialsense/inertial-sense-sdk/tree/master/cltool">Command Line Tool (cltool)</a> is an open source project designed to illustrate InertialSense C++ class implementation.  The cltool project can be compiled on most operating systems using cmake and gcc and can be used to communicate, log data, and update firmware for Inertial Sense products.  A Visual Studio project for Windows is also included.  See [Using cltool](../App_Usage/cltool.md) for details on compiling and running the cltool.

### Implementation Keywords
The following keywords are found in the cltool soure code identify the steps for InertialSense class implementation.

```C++
/* SDK Implementation Keywords:
 * [C++ COMM INSTRUCTION] - C++ binding API, InertialSense class with binary
 * communication protocol and serial port support for Linux and Windows.
 * [LOGGER INSTRUCTION] - Data logger.
 * [BOOTLOADER INSTRUCTION] - Firmware update feature.
 */
```

## Serial Communications

### Step 1: Instantiate InertialSense class

Include the InertialSense header file. Create InertialSense object.

```C++
#include "InertialSense.h"

// [C++ COMM INSTRUCTION] 1.) Create InertialSense object, passing in data callback function pointer.
InertialSense inertialSenseInterface(cltool_dataCallback);
```

### Step 2: Open serial port

Open the serial by specifying the com port number, buadrate, and  and The serial port used for communications

```C++
if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(),
   g_commandLineOptions.baudRate,
   g_commandLineOptions.disableBroadcastsOnClose))
{
	cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
	return -1; // Failed to open serial port
}
```

### Step 3: Enable data broadcasting

The following enables data broadcasting from the uINS at a specified data rate or period in milliseconds.

``` C++
cltool_setupCommunications(inertialSenseInterface)
```

### Step 4: Read data

Call the Update() method at regular intervals to send and receive data.

``` C++
// Main loop. Could be in separate thread if desired.
while (!g_inertialSenseDisplay.ControlCWasPressed())
{
	if (!inertialSenseInterface.Update())
	{
		// device disconnected, exit
		break;
	}
}
```

### Step 5: Handle received data

New data is available in the data callback function.

``` C++
static void cltool_dataCallback(InertialSense* i, p_data_t* data, int pHandle)
{
	// Print data to terminal
	g_inertialSenseDisplay.ProcessData(data);

	// uDatasets is a union of all datasets that we can receive. See data_sets.h for a full list of all available datasets.
	uDatasets d = {};
	copyDataPToStructP(&d, data, sizeof(uDatasets));

	// Example of how to access dataset fields.
	switch (data->hdr.id)
	{
	case DID_INS_2:
		d.ins2.qn2b; // quaternion attitude
		d.ins2.uvw; // body velocities
		d.ins2.lla; // latitude, longitude, altitude
		break;
	case DID_INS_1:
		d.ins1.theta; // euler attitude
		d.ins1.lla; // latitude, longitude, altitude
	break;
	case DID_IMU: d.imu; break;
	case DID_PIMU: d.pimu; break;
	case DID_IMU_1: d.imu; break;
	case DID_IMU_2: d.imu; break;
	case DID_GPS: d.gps; break;
	case DID_MAGNETOMETER: d.mag; break;
	case DID_BAROMETER: d.baro; break;
	case DID_SYS_SENSORS: d.sysSensors; break;
	}
}
```

### Step 6: Close interface

Close the interface when your application finishes.

```C
// Close cleanly to ensure serial port and logging are shutdown properly. (optional)
inertialSenseInterface.Close();
```

## Data Logging

### Step 1: Configure and Start Logging

```C
// [LOGGER INSTRUCTION] Setup and start data logger
if (!cltool_setupLogger(inertialSenseInterface))
{
	cout << "Failed to setup logger!" << endl;
	return -1;
}
```

### Run ()

```c
// [LOGGER INSTRUCTION] Setup and start data logger
if (!cltool_setupLogger(inertialSenseInterface))
{
	cout << "Failed to setup logger!" << endl;
	return -1;
}
```

## Compile & Run (Linux/Mac)

1. Create build directory
   ``` bash
   cd cltool
   mkdir build
   ```
2. Run cmake from within build directory
   ``` bash
   cd build
   cmake ..
   ```
3. Compile using make
    ``` bash
    make
    ```
4. If necessary, add current user to the "dialout" group to read and write to the USB serial communication ports.  In some cases the Modem Manager must be disabled to prevent interference with serial communication. 
   ```bash
   sudo usermod -a -G dialout $USER
   sudo usermod -a -G plugdev $USER
   sudo systemctl disable ModemManager.service && sudo systemctl stop ModemManager.service
   (reboot computer)
   ```
5. Run executable
   ``` bash
   ./cltool
   ```

### Install udev rules for IMX-5 bootloader

If you need to update the bootloader on the IMX-5 (contact Inertial Sense to get this file), you will need to run the cltool with elevated (sudo) permissions, or run the following commands prior to running the cltool to give permissions to all users. This is required because the first-stage bootloader of the IMX-5 is a DFU device, rather than a CDC (virtual COM port) device.
```
sudo cp ../../../90-inertialsense.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

## Compile & Run (Windows MS Visual Studio)
1. [Install and Configure Visual Studio](../getting-started/#installing-and-configuring-visual-studio)
2. Open the project folder in Visual Studio: Open a local folder -> inertial-sense-sdk/cltool/
3. Build All
4. Run executable
   ``` bash
   C:\inertial-sense-sdk\cltool\out\...\cltool.exe
   ```


## Summary

This section has covered the basic functionality you need to set up and communicate with <a href="https://inertialsense.com">Inertial Sense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/inertial-sense-sdk">Inertial Sense SDK</a> GitHub repository, and we will be happy to help.
