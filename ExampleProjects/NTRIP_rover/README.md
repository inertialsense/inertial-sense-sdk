# SDK: NTRIP Rover Example Project

This [ISNtripRoverExample](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/NTRIP_rover) project demonstrates how to implement a TCP NTRIP connection to and receive corrections from an RTK base station.  This example supplies RTK corrections to the <a href="https://inertialsense.com">InertialSense</a> products (uINS and Rugged) using the Inertial Sense SDK.

## Implementation

### Step 1: Add Includes

```C++
// Change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
#include "../../src/ISStream.h"
#include "../../src/ISClient.h"
#include "../../src/protocol_nmea.h"
```

### Step 2: Init comm instance

```C++
	is_comm_instance_t comm;
	uint8_t buffer[2048];

	// Initialize the comm instance, sets up state tracking, packet parsing, etc.
	is_comm_init(&comm, buffer, sizeof(buffer));
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
	if (!serialPortOpen(&serialPort, argv[1], IS_BAUDRATE_921600, 1))
	{
		printf("Failed to open serial port on com port %s\r\n", argv[1]);
		return -2;
	}
```

### STEP 4: Connect to the RTK base (sever)

```c++
	// Connection string follows the following format:
	// [type]:[IP or URL]:[port]:[mountpoint]:[username]:[password]
	// i.e. TCP:RTCM3:192.168.1.100:7777:mount:user:password
	if ((s_clientStream = cISClient::OpenConnectionToServer(argv[2])) == NULLPTR)
	{
		printf("Failed to open RTK base connection %s\r\n", argv[2]);
		return -2;
	}
```

### Step 5: Stop any message broadcasting

```c++
	// Stop all broadcasts on the device
	int messageSize = is_comm_stop_broadcasts(comm);
	if (messageSize != serialPortWrite(serialPort, comm->buffer, messageSize))
	{
		printf("Failed to encode and write stop broadcasts message\r\n");
	}
```

### Step 6: Enable message broadcasting

To use NTRIP, we must enable the DID_GPS1_POS message which will be rebroadcast as NMEA GGA every 5 seconds to the RTK NTRIP base station.  

```C++
int enable_message_broadcasting(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	int n = is_comm_get_data(comm, _DID_GPS1_POS, 0, 0, 1);
	if (n != serialPortWrite(serialPort, comm->buf.start, n))
	{
		printf("Failed to encode and write get GPS message\r\n");
		return -5;
	}
	n = is_comm_get_data(comm, DID_GPS1_RTK_POS_REL, 0, 0, 1);
	if (n != serialPortWrite(serialPort, comm->buf.start, n))
	{
		printf("Failed to encode and write get GPS message\r\n");
		return -5;
	}
	return 0;
}
```

### Step 7: Handle received data 

See the ISNtripRoverExample.cpp for details.

## Compile & Run (Linux/Mac)

1. Install necessary dependencies
   ``` bash
   # For Debian/Ubuntu linux, install libusb-1.0-0-dev from packages
   sudo apt update && sudo apt install libusb-1.0-0-dev
   # For MacOS, install libusb using brew
   brew install libusb
   ```
2. Create build directory
   ``` bash
   cd inertial-sense-sdk/ExampleProjects/NTRIP_rover
   mkdir build
   ```
3. Run cmake from within build directory
   ``` bash
   cd build
   cmake ..
   ```
4. Compile using make
   ``` bash
   make
   ```
5. If necessary, add current user to the "dialout" group to read and write to the USB serial communication ports.  In some cases the Modem Manager must be disabled to prevent interference with serial communication. 
   ```bash
   sudo usermod -a -G dialout $USER
   sudo usermod -a -G plugdev $USER
   sudo systemctl disable ModemManager.service && sudo systemctl stop ModemManager.service
   (reboot computer)
   ```
6. Run executable
   ``` bash
   ./bin/ISNtripRoverExample /dev/ttyUSB0 TCP:RTCM3:192.168.1.100:7777:mount:user:password
   ```
## Compile & Run (Windows MS Visual Studio)

1. Open Visual Studio solution file (inertial-sense-sdk\ExampleProjects\Communications\VS_project\ISCommunicationsExample.sln)
2. Build (F7)
3. Run executable
   ``` bash
   C:\inertial-sense-sdk\ExampleProjects\ISNtripRoverExample\VS_project\Release\ISNtripRoverExample.exe COM3
   ```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/inertial-sense-sdk">inertial-sense-sdk</a> GitHub repository, and we will be happy to help.
