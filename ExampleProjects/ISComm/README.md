# SDK: Binary Communications Example Project

## Introduction
This [ISCommExample](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/ISComm) project demonstrates binary communications with the <a href="https://inertialsense.com">InertialSense</a> products (IMX and GPX) using the Inertial Sense SDK.


## Purpose and Design
The ISComm module, also known as the Inertial Sense simple communications interface, defines the packet structures, protocol type enumeration, and the stateful is_comm_instance_t parser used by all IS SDK layers.  The parser supports simultaneous detection of ISB, NMEA, u-blox, RTCM3, SPARTN, Sony, and Septentrio protocols from a single byte stream.

The simple comm interface does not require any of the com manager APIs and is designed for simple or lightweight scenarios, tiny embedded platforms, etc.  It provides an API of `is_comm_` methods, some of which are demonstrated here.  

Our example binds to a serial port connected to an Inertial Sense device and then shows the use of ISComm in communicating with it.  We provide a callback function to pass to ISComm as a data handler, open communcations, and finally process and display what is received for the user to observe.

All information is displayed to standard output by default in this application.

## Documentation and Project Navigation


## Files

#### Project Files

* [ISCommExample.cpp](./ISCommExample.cpp)

Files in this folder.

#### SDK Files

* [data_sets.c](../../src/data_sets.c)
* [data_sets.h](../../src/data_sets.h)
* [ISComm.c](../../src/ISComm.c)
* [ISComm.h](../../src/ISComm.h)
* [ISPose.h](../../src/ISPose.h)
* [ISUtilities.h](../../src/ISUtilities.h)
* [PortFactory.h](../../src/PortFactory.h)

Paths relative to this folder.

## Implementation

### Step 1: Add Includes
We create only one new code file for this example project.  We are demonstrating the ISComm module, and also use the Port Factory to bind a serial port to use for communication to the Inertial Sense device.  A couple of other modules/utilities included as needed for this example.

```C++
#include "ISComm.h"
#include "ISPose.h"
#include "ISUtilities.h"
#include "PortFactory.h"
```

### Step 2: Application main, initialize and open serial port
The entry point for the application confirms a single command-line argument specifying the port to be used, which we then set up.  This is done using the `SerialPortFactory` extension of the `PortFactory` class.  For our application, the only setting needed is the baud rate.
```C++
SerialPortFactory& spf = SerialPortFactory::getInstance();
spf.setBaudRate(921600);
port_handle_t port = spf.bindPort(argv[1]);
```
The `port_handle_t` will be passed to the ISComm module as the port it will use to send and receive messages.

### Step 3: Stop any message broadcasting
This command writes a broadcast stop message to all ports:
```c++
is_comm_stop_broadcasts_all_ports(port);
```

### Step 4: Set data configuration (optional)
This would be an optional step that encodes and writes a `SET_DATA` packet to alter the way the device data is presented; we include it here in our example.  Note that the values displayed on execution will look different depending upon your usage of this.
```C++
// Set INS output Euler rotation in radians to 90 degrees roll for mounting
float rotation[3] = { 90.0f*C_DEG2RAD_F, 0.0f, 0.0f };
is_comm_set_data(port, DID_FLASH_CONFIG, sizeof(float) * 3, offsetof(nvm_flash_cfg_t, insRotation), rotation);
```

### Step 5: Register callback for data handling
We send the name of the function we create (detailed in Step 8) in our application that handles ISB data in the way we want:
```C++
// Any ISB protocol messages will call into this handler
is_comm_register_port_isb_handler(port, isbDataHandler);
```

### Step 6: Enable message broadcasting
Writes a `GET_DATA` request packet with DID type, period, etc.  This example can demonstrate data of four different types:  `DID_INS_1`, `DID_INS_2`, `DID_GNSS1_POS`, `DID_IMU`.
```C++
//Request INS1_1 message at 100x startupNavDtd (this should be about 100 x 7ms = 700ms)
is_comm_get_data(port, DID_INS_1, 0, 0, 100);
```

### Step 7: Save persistent messages (optional)
This would be an optional step that encodes and writes a `SET_DATA` packet to cause the device to "Save current persistent messages".  *Note that this is removed by default as it causes a write to the device's flash.*
```c++
#if 0
	system_command_t cfg;
    cfg.command = SYS_CMD_SAVE_PERSISTENT_MESSAGES;
    cfg.invCommand = ~cfg.command;
	is_comm_set_data(port, DID_SYS_CMD, 0, 0, &cfg);
#endif
```

### Step 8: Process received messages
As long as the port remains open, this loop will ask ISComm to parse the messages received on the port:
```C++
while (portIsOpened(port)) {
	is_comm_port_parse_messages(port);
	SLEEP_MS(1);
}
```
The data handling callback we provided allows us to specify what we do with those messages in our application.

### Step 9: Handle received data 
We create a callback handler from the ISComm parser, called for each InertialSense binary message that is successfully parsed.  ISComm gives it various arguments:  a context pointer that can be associated with the port/ISCOMM instance, a pointer to the packet/message structure of the parsed message, and the port the packet/message was received by.

```C++
int isbDataHandler(void* ctx, p_data_t* data, port_handle_t port) {
	switch (data->hdr.id)
	{
	case DID_INS_1:
		handleIns1Message((ins_1_t*)data->ptr);
		break;

	case DID_INS_2:
		handleIns2Message((ins_2_t*)data->ptr);
		break;
	//etc...
```

This handler calls a custom function for each of the four demonstrated message types, which are also defined in our ISCommExample.cpp file.  By default, all these functions do is nicely display the received values from the device.

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
   cd inertial-sense-sdk/ExampleProjects/ISComm
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
   ./ISCommExample /dev/ttyACM0
   ```
7. You should observe data from the device, as in:
	```
	INS TimeOfWeek: 11.793s, LLA: 0.0000000,0.0000000, 0.00, Euler:  -0.5,  0.6,115.3
	INS TimeOfWeek: 12.493s, LLA: 0.0000000,0.0000000, 0.00, Euler:  -0.5,  0.6,115.3
	INS TimeOfWeek: 13.193s, LLA: 0.0000000,0.0000000, 0.00, Euler:  -0.5,  0.6,115.3
	```
	Exit the application with Ctrl+C.


## Compile & Run (Windows MS Visual Studio)

1. Open Visual Studio solution file (inertial-sense-sdk\ExampleProjects\Communications\VS_project\ISCommExample.sln)
2. Build (F7)
3. Run executable
   ``` bash
   C:\inertial-sense-sdk\ExampleProjects\Communications\VS_project\Release\ISCommExample.exe COM3
   ```

## Support

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/inertial-sense-sdk">inertial-sense-sdk</a> GitHub repository, and we will be happy to help.
