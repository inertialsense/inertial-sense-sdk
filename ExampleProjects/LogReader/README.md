# SDK: Data Log Reader Example Project

This [ISLogReaderExample](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/LogReader) project demonstrates data logging with the <a href="https://inertialsense.com">InertialSense</a> products (uINS, uAHRS, and uIMU) using the Inertial Sense SDK.

## Files

#### Project Files

* [ISLogReaderExample.cpp](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/LogReader/ISLogReaderExample.cpp)

#### SDK Files

* [SDK](https://github.com/inertialsense/inertial-sense-sdk/tree/master/src)


## Implementation

### Step 1: Add Includes

```C++
// Change these include paths to the correct paths for your project
#include "../../src/InertialSense.h"
```

### Step 2: Instantiate InertialSense class

```C++
	// InertialSense class wraps communications and logging in a convenient, easy to use class
	InertialSense inertialSense(dataCallback);
	if (!inertialSense.Open(argv[1]))
	{
		std::cout << "Failed to open com port at " << argv[1] << std::endl;
	}
```

### Step 3: Enable data logger

```C++
	// get log type from command line
	cISLogger::eLogType logType = (argc < 3 ? cISLogger::eLogType::LOGTYPE_DAT : cISLogger::ParseLogType(argv[2]));
	inertialSense.SetLoggerEnabled(true, "", logType);
```

### Step 4: Enable data broadcasting

```C++
	// broadcast the standard set of post processing messages (ins, imu, etc.)
	inertialSense.BroadcastBinaryDataRmcPreset();

	// instead of the rmc preset (real-time message controller) you can request individual messages...
	// inertialSense.BroadcastBinaryData(DID_IMU, 10); // imu every 10 milliseconds (100 hz)
```

By default, data logs will be stored in the "IS_logs" directory in the current directory.

``` bash
build/IS_logs/LOG_SN30664_20180323_112822_0001.dat
```

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
   cd inertial-sense-sdk/ExampleProjects/Logger
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
   ./bin/ISLoggerExample /dev/ttyUSB0
   ```
## Compile & Run (Windows MS Visual Studio)

1. Open Visual Studio solution file (inertial-sense-sdk\ExampleProjects\Logger\VS_project\ISLoggerExample.sln)
2. Build (F7)
3. Run executable
   ``` bash
   C:\inertial-sense-sdk\ExampleProjects\Logger\VS_project\Release\ISLoggerExample.exe COM3
   ```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/inertial-sense-sdk">inertial-sense-sdk</a> GitHub repository, and we will be happy to help.
