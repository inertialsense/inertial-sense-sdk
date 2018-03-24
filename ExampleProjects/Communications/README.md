# SDK: Binary Communications Example Project

This example project demonstrates binary communications with the <a href="https://inertialsense.com">InertialSense</a> products (uINS, uAHRS, and uIMU) using the Inertial Sense SDK.

## Files

#### Project Files

* [ISCommunicationsExample.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/ExampleProjects/Communications/ISCommunicationsExample.c)

#### SDK Files
 
* [data_sets.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/data_sets.c)
* [data_sets.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/data_sets.h)
* [ISComm.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/ISComm.c)
* [ISComm.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/ISComm.h)
* [serialPort.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/serialPort.c)
* [serialPort.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/serialPort.h)
* [serialPortPlatform.c](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/serialPortPlatform.c)
* [serialPortPlatform.h](https://github.com/inertialsense/InertialSenseSDK/tree/master/src/serialPortPlatform.h)


## Implementation

### Step 1: Add Includes

```C++
// Change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
```

### Step 2: Init comm instance

```C++
	is_comm_instance_t comm;
	uint8_t buffer[2048];

	// Make sure to assign a valid buffer and buffer size to the comm instance
	comm.buffer = buffer;
	comm.bufferSize = sizeof(buffer);

	// Initialize the comm instance, sets up state tracking, packet parsing, etc.
	is_comm_init(&comm);
```

### Step 3: Initialize and open serial port

```C++
	serial_port_t serialPort;

	// Initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
	//  you will need to handle the serial port creation, open and reads yourself. In this
	//  case, you do not need to include serialPort.h/.c and serialPortPlatform.h/.c in your project.
	serialPortPlatformInit(&serialPort);

	// Open serial, last parameter is a 1 which means a blocking read, you can set as 0 for non-blocking
	// you can change the baudrate to a supported baud rate (IS_BAUDRATE_*), make sure to reboot the uINS
	//  if you are changing baud rates, you only need to do this when you are changing baud rates.
	if (!serialPortOpen(&serialPort, argv[1], IS_BAUDRATE_3000000, 1))
	{
		printf("Failed to open serial port on com port %s\r\n", argv[1]);
		return -2;
	}
```

### Step 4: Enable message broadcasting

```C++
	int messageSize;

	// Stop all broadcasts on the device
	messageSize = is_comm_stop_broadcasts(&comm);
	if (messageSize < 1)
	{
		printf("Failed to encode stop broadcasts message\r\n");
		return -3;
	}
	serialPortWrite(&serialPort, buffer, messageSize);

	// Ask for INS message 20 times a second (period of 50 milliseconds).  Max rate is 500 times a second (2ms period).
	messageSize = is_comm_get_data(&comm, _DID_INS_LLA_EULER_NED, 0, 0, 50);
	if (messageSize < 1)
	{
		printf("Failed to encode get INS message\r\n");
		return -4;
	}
	serialPortWrite(&serialPort, buffer, messageSize);

	// Ask for gps message 5 times a second (period of 200 milliseconds) - offset and size can be left at 0 unless you want to just pull a specific field from a data set
	messageSize = is_comm_get_data(&comm, _DID_GPS_NAV, 0, 0, 200);
	if (messageSize < 1)
	{
		printf("Failed to encode get GPS message\r\n");
		return -4;
	}
	serialPortWrite(&serialPort, buffer, messageSize);
```

### Step 5: Handle recieved data 

```C++
	int count;
	uint8_t inByte;

	// You can set running to false with some other piece of code to break out of the loop and end the program
	while (running)
	{
		// Read one byte with a 20 millisecond timeout
		while ((count = serialPortReadCharTimeout(&serialPort, &inByte, 20)) > 0)
		{
			switch (is_comm_parse(&comm, inByte))
			{
			case _DID_INS_LLA_EULER_NED:
				handleInsMessage((ins_1_t*)buffer);
				break;

			case _DID_GPS_NAV:
				handleGpsMessage((gps_nav_t*)buffer);
				break;

			case _DID_IMU_DUAL:
				handleImuMessage((dual_imu_t*)buffer);
				break;

				// TODO: add other cases for other data ids that you care about
			}
		}
	}
```

## Compile & Run (Linux/Mac)

1. Create build directory
``` bash
$ cd InertialSenseSDK/ExampleProjects/Communications
$ mkdir build
```
2. Run cmake from within build directory
``` bash
$ cd build
$ cmake ..
```
3. Compile using make
 ``` bash
 $ make
 ```
4. If necessary, add current user to the "dialout" group in order to read and write to the USB serial communication ports:
```bash
$ sudousermod -a -G dialout $USER
$ sudousermod -a -G plugdev $USER
(reboot computer)
```
5. Run executable
``` bash
$ ./bin/ISCommunicationsExample /dev/ttyUSB0
```
## Compile & Run (Windows MS Visual Studio)

1. Open Visual Studio solution file (InertialSenseSDK\ExampleProjects\Communications\VS_project\ISCommunicationsExample.sln)
2. Build (F7)
3. Run executable
``` bash
C:\InertialSenseSDK\ExampleProjects\Communications\VS_project\Release\ISCommunicationsExample.exe COM3
```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/InertialSenseSDK">InertialSenseSDK</a> github repository, and we will be happy to help.
