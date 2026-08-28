# SDK: Custom Robot Device Example Project

## Introduction
This [Custom Robot Device Example](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/CustomRobotDevice) project demonstrates the creation of custom `ISDevice` and custom `DeviceFactory` child classes, using the Inertial Sense SDK.  It also shows how to use `PortManager` and `DeviceManager` to maintain the set of discovered ports and discovered devices that come out of factory operations.  A simple application can be built from the provided code out of the box.  **This example requires an IMX device to be connected to your workstation.**

## Purpose and Design
This project is constructed as both a walk-through and a template for users wishing to learn how to make use of three modules of the SDK that would be important pieces for device management.    

One is the `ISDevice` which is a C++ class that represents a single, physical Inertial Sense device (IMX or GPX) bound to a port.  It owns its identity (dev_info_t), synchronized flash/system-parameter state, and the comm-protocol handlers that parse data received from it. `DeviceManager` owns a collection of these; a caller that only ever talks to one device can use a standalone `ISDevice` directly without `DeviceManager`, but we will demonstrate the manager here.

The second is the `DeviceFactory` which is an abstract C++ class that is responsible for discovery of connected devices of a particular type.  There should be one implementation for each type of discoverable device.  It handles device-type identification and allocation: given a raw port, determines what kind of Inertial Sense device (if any) is attached and constructs the matching `ISDevice` subclass.  The factory allocates a device object, and this operation returns a `device_handle_t`.  As an abstract class, this allows for third-party locators to be implemented for custom device types.

Finally, we demonstrate the `DeviceManager`, which uses the custom device factory to create and keep a known list of available devices, optionally filtered to a limited set, or all.  It will provide access to a factory's devices as requested.

In this example we use a Linux platform USB serial port for our device connection to an Inertial Sense IMX-5 unit.  Conceptually, we are presenting the idea of a robot device we have built with an IMX on board, and we will be reading data from the IMX.  Our custom device we are calling `CustomRobotDevice` and it is discovered via our `CustomDeviceFactory`.  We make use of the SDK's provided `SerialPortFactory` extension of `PortFactory` to bind to the port, and `PortManager` as well.  See this dependency (dashed) and process (solid) diagram for an overview:


```mermaid
graph TD
   A[[main]] --> B[[device step loop]]
   A -.-> C(CustomRobotDevice)
   A -.-> D(CustomDeviceFactory)
   A -.-> E(DeviceManager)     
   A -.-> F(SerialPortFactory)
   A -.-> G(PortManager)
   C -.-> H(ISDevice)
   D -.-> I(DeviceFactory)
   F -.-> J(PortFactory)
   H -.-> K(dev_info_t)
   B ~~~ H
   B <--> C
   %%comment
```

## Documentation Usage and Project Navigation
In the [Implementation](#implementation) section of this README, you will find a series of sub-sections called Steps, which you may follow in order.  These Steps accomplish two purposes:  one is to provide a guided tour of the project files by reference and sample, and the second is to outline a process for how a user might go about creating their own versions of a custom implementation for their purposes.  This is done by explaining what has been created for you already by way of demonstration, and why each piece is critical. Thus, as you proceed through the Step sub-sections you will find that you are not being instructed to generate any new code, as it is a fully functional example already, but rather being introduced to how something similar could be created.

Each Step is part of a project-level process, will reference one or more of the code files, and describe actions the user can take in the custom implementation creation.  Each code file in the local project folder will contain one or more commented sections each associated with a README Step section, and the Step sections in a given file may start at any Step number depending on when in the project-level process that file's contents come into play.

Here is a visual representation of the STEP concept to help navigate:
```mermaid
flowchart LR
    A[Implementation Section <br> Step 1 <br> Step 2 <br> Step 3 <br> ... ] --> B[Code File X <br> ... <br> /** STEP 2 <br> ... <br> /** STEP 3 <br> ... ]
    A --> C[Code File Y <br> ... <br> /** STEP 3 <br> ...]
    A --> D[Code File Z <br> .. <br> /** STEP 1 <br>  ...]
```

The parts of the code associated with a given README Step are tagged like this:
```C++
/** STEP 6: 
```

Doxygen style comments are ubiquitous throughout the three example Project Files.  You may build the Doxygen HTML documentation by creating a Doxyfile and following standard Doxygen build instructions if that suits you, but all information is contained in this document plus the files listed above.  

```C
/** 
 * @note These double-asterisk comments are Doxygen style, included in documentation 
 * generated by that tool if run, and found throughout the code in this example to guide 
 * users through the process of creating and using SDK class custom implementations
 */
```

The following implementation instructions identify some examples of similar code to that found under corresponding "STEP X" markings in the source files.  Please refer to the source file code directly and **treat the incomplete snippets of code in this README as orientation only**.

## Files

#### Project Files

* [main.cpp](./main.cpp)
* [CustomDeviceFactory.h](./CustomDeviceFactory.h)
* [CustomRobotDevice.cpp](./CustomRobotDevice.cpp)
* [CustomRobotDevice.h](./CustomRobotDevice.h)

Note these are local to this folder.

#### SDK Files

* [com_manager.h](../../../src/com_manager.h)
* [core/base_port.h](../../../src/core/base_port.h)
* [core/msg_logger.h](../../../src/core/msg_logger.h)
* [DeviceFactory.h](../../../src/DeviceFactory.h)
* [DeviceManager.h](../../../src/DeviceManager.h)
* [ISDevice.h](../../../src/ISDevice.h)
* [ISUtilities.h](../../../src/ISUtilities.h)
* [PortFactory.h](../../../src/PortFactory.h)
* [PortManager.h](../../../src/PortManager.h)


Note file name list entries relative to SDK `src/` folder, with local relative path linked.

## Implementation

### Step 1: Create Custom Device Header
The `DeviceFactory` is designed to provide a base class for building a device discoverer, upon any `ISDevice` type, which populates a `dev_info_t`.  Our example device implementation is called `CustomRobotDevice` and extends the `ISDevice` class.  We communicate with the device via the port's `port_handle_t` bound by the `PortFactory`.  We implement data handling methods unique to our device in the code. 

```c++
#include "PortFactory.h"
#include "ISDevice.h"
#include "ISDisplay.h"
```

Inherit from ISDevice, and add anything custom to make this device unique.  In this example we show a data_sets.h type for the DID data we ask for in this application by default, `DID_INS_1`: 
```c++
class CustomRobotDevice : public ISDevice {

public:
   /** structures to hold the data we are interested in for this device, from data_sets.h */
   ins_1_t insData = {};
```

Function declarations for our new class include an optional configuration function to set up our device and select the data we want to see, a virtual `step()` function implementation used by every `ISDevice` to "run" and process messages, and at least one message handler callback also implementing a virtual `ISDevice` function.  Implemented in CustomRobotDevice.cpp.
```c++
bool configure();

bool step() override;

int onIsbDataHandler(p_data_t* data, port_handle_t port) override;
```

Note that for demonstration purposes we have added an optional `cInertialSenseDisplay` member to this class, for data display formatting to the terminal:
```c++
cInertialSenseDisplay isDisplay = cInertialSenseDisplay(cInertialSenseDisplay::DMODE_PRETTY);
```

### Step 2: Extend ISDevice With Custom Device Implementation

In CustomRobotDevice.cpp, we complete the new device implementation, starting with important headers:
```c++
#include "CustomRobotDevice.h"
#include "ISUtilities.h"
#include "core/msg_logger.h"
```

We use our custom device class to configure the connected device to send us data and process the data.  After verifying connection and stopping all messages, we identify which messages we would like to receive from the device, starting with `DID_INS_1` type by default.  Any desired DID type can be added one per line:
```c++
bool CustomRobotDevice::configure() {
   //...
   std::vector<std::function<bool()>> bcast_calls = {
      [this]() { return BroadcastBinaryData(DID_INS_1, 25); } //,
      //[this]() { return BroadcastBinaryData(DID_SYS_PARAMS, 100); }  
    };
   //...   
```

The `ISDevice::step()` function is called to process any pending, received data on the bound port, and call any registered handlers for any valid packets which are parsed from that data. Additionally, this call will manage other comm-related tasks such as data/config synchronization to the device, as well as progressing firmware updates, etc.  This function should be called a regular interval fast enough to prevent received data from overflowing the port's RX buffer (typically a 1ms interval or faster, for a 921600 Serial Baud rate).  Returns false if the port is invalid or closed, otherwise true. Note that 'true' does NOT provide any indication of data parsed, etc. Only that the port was valid, and that the maintenance functions were called.  We implement the new `step()` function, but do not add any logic to it for our current purposes.

```c++
bool CustomRobotDevice::step() {
   /** Custom step operations here if desired */
    
   return ISDevice::step(); // call the parent step() function to do all the usual ISDevice functions
}
```

Finally, we need at least one function to handle the DID messages we receive.  `onIsbDataHandler()` is a callback data handler for the `ISDevice`, and which will be called every time data arrives from the physical device.   `p_data_t` struct pointer represents the buffer of data received from the device, including the data ID, associated flags, and the actual data payload.  `port_handle_t` represents the port that this data was received from.  We could for example copy the data to our `ins_1_t` struct for custom processing one field at a time if desired, though in this example demonstrate using `ISDisplay` to nicely print it to standard out.

```c++
 int CustomRobotDevice::onIsbDataHandler(p_data_t* data, port_handle_t port) {

   if ( ISDevice::onIsbDataHandler(data, port) ) {
      if (data->hdr.id == DID_INS_1) {                     
         /** optionally do something with this data here */
         std::cout << isDisplay.DataToString((const p_data_t*)data);
   //...   
 ```

 Note that we first call the ISDevice's own handler as it does a validation on the data.


### Step 3: Extend DeviceFactory with New Class
This example creates the `CustomDeviceFactory` derived class specified by the file CustomDeviceFactory.h.  We will derive it from the SDK `DeviceFactory` class.  The factory acts in concert with the `DeviceManager` to discover a physical device connected via the provided `port_handle_t`.   For only specific types of devices that we indicate, it then allocates our device object, giving  us a `device_handle_t` we then use to allow our custom device object to interface with the physical device.

We need some headers included, and create a derived class:
```c++
#include "DeviceFactory.h"
#include "CustomRobotDevice.h"

class CustomDeviceFactory : public DeviceFactory {
```

At a minimum to serve our custom device, we must implement the allocateDevice function.
```C++
/**
 * A function to be implemented in the factory responsible for allocating the underlying device type and returning a pointer to it
 * This function should NOT manipulate the underlying port, such as opening, etc.
 * @param devInfo the device information uniquely identifying the specific device
 * @param port an associated port (optional) that this device should be bound to.
 * @return a ISDevice pointer to the newly allocated ISDevice or null of not allocated
 */
virtual device_handle_t allocateDevice(const dev_info_t &devInfo, port_handle_t port = nullptr) { return std::make_shared<ISDevice>(devInfo, port); };
```

In this example we imagine our custom device was built with an IMX-5 unit so we look for that HW idenfication specifically.  This is identified with `IS_HARDWARE_IMX_5_0`, from data_sets.h.  Here are some other type examples from that file:

```c++
static const is_hardware_t IS_HARDWARE_EVB_2_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_EVB, 2, 0);
//...
static const is_hardware_t IS_HARDWARE_IMX_6_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 6, 0);
static const is_hardware_t IS_HARDWARE_GPX      = ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, -1, -1);
static const is_hardware_t IS_HARDWARE_GPX_1_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, 1, 0);
//etc
```

If the ident matches, we return a shared pointer to a new `CustomRobotDevice`.  We are only implementing this one function that is a few lines long for this new device factory, so we will leave it here in the header rather than create a separate .cpp file. 
```C++
device_handle_t allocateDevice(const dev_info_t &devInfo, port_handle_t port) override {

   /** When we find IMX-5 dev info, we know we want to allocate a new custom device */
   if (ENCODE_DEV_INFO_TO_HDW_ID(devInfo) == IS_HARDWARE_IMX_5_0)
      return std::make_shared<CustomRobotDevice>(devInfo, port);

   return nullptr;
}
```


### Step 4: Create Example Application
Create a new .cpp file for the application, which for us is [main.cpp](./main.cpp) in this example.  Include headers for any desired Inertial Sense SDK utilities, user IO capabilities, etc.  Reference the new `CustomRobotDevice` and `CustomDeviceFactory` classes, and don't forget the `DeviceManager` and `PortManager`:
```C++
#include <stdio.h>
#include "DeviceManager.h"
#include "CustomRobotDevice.h"
#include "CustomDeviceFactory.h"
#include "PortManager.h"
```

We create a `main_discovery()` that operates on our devices and managers, followed by a `main()` entry point that processes the command line.  This will be the entry point for the application.  Our `main()` uses the command line arg for identifying a port (if given) to be used to search for a device.  If no port is given, we search all available ports in a discovery process.  

```C
/**
 * Uses portPattern for discovering a device connected to a port in an example of setting up a custom device
 * connection.  Use PortManager and PortFactory to bind a port_handle_t to the named port. With the handle,
 * the port is opened, and DeviceManager and DeviceFactory search for a device of a certain hardware ID.
 * We then receive data from the found device.
 */
int main_discovery(const char* portPattern)
{
   //...
```

```C
int main(int argc, const char** argv) {

    const char* portPattern = "(.+)";   // NOTE: this is a MATCHING REGEX pattern (this one matches everything)

    if (argc > 2)
    {
        printf("Usage: No argument allows automatic port discovery based on universal pattern.  Or if desired, a single argument selects the port (i.e. /dev/ttyACM0)\r\n");
//...
```


### Step 5: Incorporate Logging
The application code in main.cpp uses standard output status messages to inform the user of what is happening at a high level.  However, the SDK provides it's own logging system, and we demonstrate its use in our `ISDevice` extension in CustomRobotDevice.cpp.  The [msg_logger.h](../../src/core/msg_logger.h) API provides multi-platform message logging with level control, and printf-style format strings support.  It writes to the file local to the executable called `inertial_sense.log`.  Log commands can be added like so, from a line in CustomVirtualPortFactory.cpp:

```C++
log_msg(IS_LOG_ISDEVICE, IS_LOG_LEVEL_ERROR, "Failed to send \"Stop Broadcasts\" request." );
```

In our main appliaction code we can set the log level to determine which messages will be logged and which ignored:
```C++
IS_SET_LOG_LEVEL(IS_LOG_LEVEL_INFO);
```

Facility definitions like `IS_LOG_DEVICE_MANAGER` or `IS_LOG_PORT_FACTORY` identify which module is logging.


### Step 6: Instantiate the Managers and Factories
In `main()`, we started by first doing a nominal check on the command line argument with some usage statement upon invoke error.  Then in `main_discovery()` we create our communications management by instantiating one `PortManager` with a `SerialVirtualPortFactory`, followed by one `DeviceManager` with a `CustomDeviceFactory`, like so:
```C++
PortManager& pm = PortManager::getInstance();
pm.addPortFactory(&SerialPortFactory::getInstance());   // tell the PortManager that we are interested in Serial Ports

DeviceManager& dm = DeviceManager::getInstance();
dm.addDeviceFactory(&CustomDeviceFactory::getInstance());  // tell the DeviceManager that we are interested in Custom Devices
```

We are interested in the Port Manager finding all available ports per the search parameter, then with viable ports we try to find a device that matches our description on the other end:
```C++    
std::shared_ptr<CustomRobotDevice> device = nullptr;    

pm.discoverPorts(portPattern);  // first let's attempt to discover ports 

if (!pm.empty()) {              
   dm.discoverDevices(IS_HARDWARE_IMX, 1500, DeviceManager::DISCOVERY__CLOSE_PORT_ON_FAILURE);
   device = dm.getDevices().front()->as<CustomRobotDevice>();
```

Once we find that device, we'll use it for the remaining operations we do.


### Step 7: Connect and Configure the Device
Once we have found the IMX device, we call the `ISDevice::connect()` method.  We have decoupled the specific configuration operations of the IMX device from the application by creating a new  (optional) `configure()` method in our CustomRobotDevice class, which we call here from the application.  That way, we can manage the data requests and data processing all within our new device class and leave the application code more agnostic to the specific things we do with this device.

```C++
if ( !device->connect() ) {
//...

if ( !device->configure() ) {
//...
```

### Step 8: Read and Process Device Data
Our read loop then spins as long as the port is open, to process incoming data from the connected IMX device, and we periodically sleep for the CPU cycles to catch up.  The `ISDevice::step())` function causes the passing of messages between our custom device object and the hardware.
```c++
while ( portIsOpened(device->port) ) {                
   device->step();                          
   SLEEP_MS(10);                            
}
```

## Compile & Run (Linux)

1. Install necessary dependencies
   ```bash
   # For Debian/Ubuntu linux, install libusb-1.0-0-dev from packages
   sudo apt update && sudo apt install libusb-1.0-0-dev   
   ```
2. Create build directory
   ```bash
   cd inertial-sense-sdk/ExampleProjects/CustomPort/CustomVirtual
   mkdir build
   ```
3. Run cmake from within build directory
   ```bash
   cd build
   cmake ..
   ```
4. Compile using make
   ```bash
   make
   ```
5. Run executable, with optionally one argument identifying which port to use
   ```bash
   ./CustomRobotDevice
   ```
   OR
   ```bash
   ./CustomRobotDevice /dev/ttyACM0
   ```

6. View output from the application
   ```
   CustomRobotDevice example application started (ctrl+\ to quit), attempting to use port /dev/ttyACM0
   Found and connected 1 IMX device(s) on port(s): 
   /dev/ttyACM0

   (4) DID_INS_1: 12344.856s
	   Euler	    89.70,    0.95,  138.93
	   UVW	     0.0,     0.0,     0.0
	   LLA	    0.0000000,    0.0000000,    0.0 ellipsoid
	   STATUS
   		Satellite Rx 0     Aiding: Mag 1, GNSS (Hdg 0, Pos 0)
		   Mode: AHRS         Solution: VRS
		   Errors    Rx parse 1, temperature 0, self-test 0
		   hdwStatus (0x02140002)
   (4) DID_INS_1: 12345.031s
   	Euler	    89.70,    0.95,  138.93
   	UVW	     0.0,     0.0,     0.0
   	LLA	    0.0000000,    0.0000000,    0.0 ellipsoid
   	STATUS
		   Satellite Rx 0     Aiding: Mag 1, GNSS (Hdg 0, Pos 0)
		   Mode: AHRS         Solution: VRS
		   Errors    Rx parse 1, temperature 0, self-test 0
		   hdwStatus (0x02140000)
   ```
   ... and so on, repeating until commanded to exit using Ctrl+\\.

6. View logged results in the file called `inertial_sense.log` that is written local to the app executable.  If you attempt port discovery, you should see something like this logging inaccessible ports that are encountered in the search process:
   ```
   [12:35:44.386481] ERROR  (IS_LOG_PORT) :: config_serial_port():: termios confirmation failed to match expected values:
   [12:35:44.386552] ERROR  (IS_LOG_PORT) :: config_serial_port():: setting c_cflag mismatch: expected: 1cb7, actual: cbd
   [12:35:44.386561] ERROR  (IS_LOG_PORT) :: [] serialPortOpenPlatform():: Error configuring port: Operation not permitted (1)
   ```

## Support

If this doesn't cover everything you need, feel free to reach out to us via <a href="support@inertialsense.com">email</a>, and we will be happy to help.
