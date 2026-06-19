# SDK: Port Factory Custom Virtual Communications Port Example Project

This [CustomVirtualPortExample](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/CustomPort/CustomVirtual) project demonstrates the creation of a custom Port Factory child class built upon an SDK virtual test port as the base_port implementation, using the Inertial Sense SDK.

## Files

#### Project Files

* [CustomVirtualExample.cpp](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/CustomPort/CustomVirtual/CustomVirtualExample.cpp)
* [CustomVirtualPortFactory.cpp](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/CustomPort/CustomVirtual/CustomVirtualPortFactory.cpp)
* [CustomVirtualPortFactory.h](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/CustomPort/CustomVirtual/CustomVirtualPortFactory.h)

#### SDK Files

* [core/base_port.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/core/base_port.h)
* [core/msg_logger.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/core/msg_logger.h)
* [ISUtilities.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/ISUtilities.h)
* [PortFactory.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/PortFactory.h)
* [test_serial_utils.cpp](https://github.com/inertialsense/inertial-sense-sdk/tree/main/tests/test_serial_utils.cpp)
* [test_serial_utils.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/tests/test_serial_utils.h)

## Documentation
Doxygen style comments are ubiquitous throughout the three example Project Files.  You may build the Doxygen HTML documentation by creating a Doxyfile and following standard Doxygen build instructions.

The following implementation instructions identify some examples of similar code to that found under corresponding "STEP X" markings in the source files.  Please refer to the source file code directly and treat the code in this README as orientation only.

## Implementation

### Step 1: Choose Port Channel Implementation
Identify and source or build the underlying transport interface.  The Port Factory is designed to provide a base class for building a port discoverer, upon any lower level channel type.  Your channel implementation extends the SDK base_port C object, and base_port then provides an API for channel access using a set of function hooks for methods implemented in your channel code.  The base_port comes with definitions for all kinds of different port types.  See the SDK [core/base_port.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/core/base_port.h).


In this example we use the SDK virtual test port defined in [test_serial_utils.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/tests/test_serial_utils.h)
, which has both loopback and passthrough ports, so that the example can be demonstrated without specialized hardware.

```C
typedef struct test_port_s {
    union {
        base_port_t base;
        comm_port_t comm;
    };

    rmci_t          rmci;
    uint8_t         rmciUPMcnt[DID_COUNT];
    uint8_t         rmciNMEAcnt[NMEA_MSG_ID_COUNT];

    // Used to simulate serial ports
    ring_buf_t      portRingBuf;
    uint8_t         portBuffer[PORT_BUFFER_SIZE];
    uint8_t         name[6];
} test_port_t;

```



### Step 1: Create New Project Files and Include Headers
Create two new files, named something like YOURNAMEPortFactory.h and YOURNAMEPortFactory.cpp.  This example uses CustomVirtualPortFactory.*.  

Example headers for .h file:
```C++
/**
 * Include any of your own custom application port definition headers, the lower-level
 * code that defines the interface used by this custom port factory; in this case the SDK
 * virtual test ports
 */
#include "../tests/test_serial_utils.h"

/** Include the header file for the abstract class PortFactory.h
 */
#include "PortFactory.h"

// etc
```

Example headers for .cpp file:
```C++
/** Include C++ libraries for use by your custom port class member functions defined here
 */
#include <vector>
#include <regex>

/** Include utility functions for use by your custom port class member functions defined here
 */
#include "ISUtilities.h"

//etc
```


### Step 2: Extend/Define Port Factory for Custom Port

See the CustomVirtualPortFactory.h file for configuration example of singleton port factory, optional class members and functions, etc.  The header file defines a child class that inherits from PortFactory, as in:
```C++
class CustomVirtualPortFactory : public PortFactory
```

The .cpp file body will define at a minimum the following virtual PortFactory functions: bindPort, locatePorts, releasePort, validatePort:
```C++
port_handle_t CustomVirtualPortFactory::bindPort(const std::string& pName, uint16_t pType);

bool CustomVirtualPortFactory::releasePort(port_handle_t port);

bool CustomVirtualPortFactory::validatePort(const std::string& pName, uint16_t pType);

void CustomVirtualPortFactory::locatePorts(std::function<void(PortFactory*, uint16_t, std::string)> portCallback, const std::string& pattern, uint16_t pType)
```

Add in additional support functions as needed for your application.
```C++
/**
 * Populates a vector of string identifiers for all available virtual ports from test_serial_utils.
 * For this example, it will be a number of virtual ports defined in the data_sets.h header used by the 
 * test_serial_utils definitions.
 * @param portNames a reference to a vector of strings, which will be populated with names identifiers of available ports
 * @return the number of ports found on the host
 */
int CustomVirtualPortFactory::getComPorts(std::vector<std::string>& portNames)
```

### Step 3: Create Application
Create a new file named something like YOURNAMEExample.cpp, like CustomVirtualExample.cpp in this example.

Include headers for any desired Inertial Sense SDK utilities, user IO capabilities, etc.  Reference your new Port Factory class:
```C++
/** The port factory child class the user creates, inheriting from PortFactory.h definition */
#include "CustomVirtualPortFactory.h"
```

Add forward declarations for custom application functions, and then create main.

### Step 4: Create and Init the New Port Factory
Initialize the port with bindPort, identifying the port type (from base_port.h definitions) which is virtual loopback comms in this case
```C++
    CustomVirtualPortFactory& vpf =  CustomVirtualPortFactory::getInstance();
    port_handle_t port = vpf.bindPort(argv[1], PORT_TYPE__COMM | PORT_TYPE__LOOPBACK);
```

This will also validate the port, per the validation method you specify in the CustomVirtualPortFactory.cpp validatePort definition.

### Step 5: Exercise the Loopback Port
In a loop executed a fixed number of iterations, send to and receive a hard coded message on the loopback port.  Use the API of the SDK core/base_port.h C object, with hooked functions (such as portWrite) implemented by the underlying wrapped virtual test port.

```C++
while (portIsOpened(port) && run_cnt > 0) {
//...
        if (portFree(port) >= wlen) {
            wbytes = portWrite(port, wbuf, wlen);
//...   
        if (portAvailable(port) > 0) {
            rbytes = portRead(port, rbuf, PORT_BUFFER_SIZE);
//...			
```   




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

	// Initialize the comm instance, sets up state tracking, packet parsing, etc.
	is_comm_init(&comm, buffer, sizeof(buffer), NULL);
    is_comm_enable_protocol(&comm, _PTYPE_INERTIAL_SENSE_DATA);
    is_comm_enable_protocol(&comm, _PTYPE_NMEA);

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

### Step 4: Stop any message broadcasting

```c++
	// Stop all broadcasts on the device
	int messageSize = is_comm_stop_broadcasts(comm);
	if (messageSize != serialPortWrite(serialPort, comm->buffer, messageSize))
	{
		printf("Failed to encode and write stop broadcasts message\r\n");
	}
```

### Step 5: Set configuration (optional)

```C++
	// Set INS output Euler rotation in radians to 90 degrees roll for mounting
	float rotation[3] = { 90.0f*C_DEG2RAD_F, 0.0f, 0.0f };
	int messageSize = is_comm_set_data_to_buf(comm, DID_FLASH_CONFIG, sizeof(float) * 3, offsetof(nvm_flash_cfg_t, insRotation), rotation);
	if (messageSize != serialPortWrite(serialPort, comm->buffer, messageSize))
	{
		printf("Failed to encode and write set INS rotation\r\n");
	}
```

### Step 6: Enable message broadcasting

This can be done either using the Realtime Message Controller (RMC) or the get data command.

#### Realtime Message Controller (RMC)

```c++
// Enable broadcasts using RMC: DID_INS_1 @ 20Hz and DID_GPS_NAV @ 5Hz
rmc_t rmc;
rmc.bits = RMC_BITS_INS1 | RMC_BITS_GPS_NAV;
rmc.insPeriodMs = 50;	// INS @ 20Hz
rmc.options = 0;		// current port

int messageSize = is_comm_set_data_to_buf(comm, DID_RMC, 0, 0, &rmc);
if (messageSize != serialPortWrite(serialPort, comm->buffer, messageSize))
{
	printf("Failed to encode and write RMC message\r\n");
}
```
#### Get Data Command

```C++
	// Ask for INS message 20 times a second (period of 50 milliseconds).  Max rate is 500 times a second (2ms period).
	int messageSize = is_comm_get_data_to_buf(buffer, bufferSize, comm, DID_INS_1, 0, 0, 50);
	if (messageSize != serialPortWrite(serialPort, comm->buffer, messageSize))
	{
		printf("Failed to encode and write get INS message\r\n");
	}

#if 1
	// Ask for gps message 5 times a second (period of 200 milliseconds) - size and offset can be left at 0 unless you want to just pull a specific field from a data set
	messageSize = is_comm_get_data_to_buf(buffer, bufferSize, comm, _DID_GPS_NAV, 0, 0, 200);
	if (messageSize != serialPortWrite(serialPort, comm->buffer, messageSize))
	{
		printf("Failed to encode and write get GPS message\r\n");
	}
#endif
```

### Step 7: Handle received data 

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
			case DID_INS_1:
				handleInsMessage((ins_1_t*)buffer);
				break;

			case _DID_GPS_NAV:
				handleGpsMessage((gps_nav_t*)buffer);
				break;

			case DID_IMU:
				handleImuMessage((imu_t*)buffer);
				break;

				// TODO: add other cases for other data ids that you care about
			}
		}
	}
```

## Compile & Run (Linux)

1. Install necessary dependencies
   ``` bash
   # For Debian/Ubuntu linux, install libusb-1.0-0-dev from packages
   sudo apt update && sudo apt install libusb-1.0-0-dev   
   ```
2. Create build directory
   ``` bash
   cd inertial-sense-sdk/ExampleProjects/CustomPort/CustomVirtual
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
5. Run executable, with one argument identifying which virtual port to use
   ``` bash
   ./CustomVirtual TEST0
   ```
## Compile & Run (Windows MS Visual Studio)

1. Open Visual Studio solution file (inertial-sense-sdk\ExampleProjects\Communications\VS_project\ISCommExample.sln)
2. Build (F7)
3. Run executable
   ``` bash
   C:\inertial-sense-sdk\ExampleProjects\Communications\VS_project\Release\ISCommExample.exe COM3
   ```

## Summary

If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/inertial-sense-sdk">inertial-sense-sdk</a> GitHub repository, and we will be happy to help.
