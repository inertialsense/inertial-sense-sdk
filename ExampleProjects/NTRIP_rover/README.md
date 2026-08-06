# SDK: NTRIP Rover Example Project

This [ISNtripRoverExample](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/NTRIP_rover) project demonstrates how to implement a TCP NTRIP connection to and receive corrections from an RTK base station.  This example supplies RTK corrections to the <a href="https://inertialsense.com">InertialSense</a> products (uINS and Rugged) using the Inertial Sense SDK.

## Implementation

### Step 1: Add Includes

```C++
// Change these include paths to the correct paths for your project
#include "../../src/PortManager.h"
#include "../../src/DeviceManager.h"
#include "../../src/ISDevice.h"
#include "../../src/NtripCorrectionService.h"
```

### Step 2: Extend ISDevice

Extend `ISDevice` to bind to the physical serial port and wire up an `NtripCorrectionService` that forwards received corrections to the device's port. Overriding `onIsbDataHandler()` lets you parse the messages you care about (position, RTK status, etc.) as they arrive.

```C++
class NtripRover : public ISDevice {
public:
    NtripCorrectionService ntrip;

    NtripRover(const std::string& serPort, const std::string& ntrip_url) : ISDevice(), ntripUrl(ntrip_url) {
        // bind to the physical serial port (hardware) and assign to the device
        assignPort(SerialPortFactory::getInstance().bindPort(serPort, PORT_TYPE__UNKNOWN));

        // tell the NtripCorrectionService to forward the received corrections to this device's port
        ntrip.addPort(port);
    }
    ...
};
```

### Step 3: Connect and configure the device

```C++
NtripRover myRover(serialPort, ntripUrl);

if (!myRover.connect()) {
    printf("Unable to connect to the specified port.\r\n");
    return -2;
}

myRover.configure();    // Stops existing broadcasts and enables the messages we need (SYS_PARAMS, GPX_STATUS, GNSS1_POS, GNSS1_RTK_POS_REL)
```

### Step 4: Connect to the RTK base (NTRIP caster)

The `NtripCorrectionService` handles the HTTP GET / basic-auth handshake and RTCM3 forwarding once a 3D fix is available:

```C++
// Connection string follows the standard NTRIP URL format:
// ntrip://<username>:<password>@<host>:<port>/<mountpoint>
if (!ntrip.isConnected())
    ntrip.connect(ntripUrl);
else
    ntrip.step();   // process and forward received corrections
```

### Step 5: Main loop

```C++
while (1) {
    myRover.step();   // drives both the device and, once connected, the NTRIP correction service
    SLEEP_MS(1);
}
```

See [ISNtripRoverExampleV3.cpp](ISNtripRoverExampleV3.cpp) for the full, working example, including forwarding the rover's GGA position back to the base every 5 seconds and printing fix status/RTK stats.

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
   ./bin/ISNtripRoverExample /dev/ttyUSB0 ntrip://user:password@192.168.1.100:7777/mount
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
