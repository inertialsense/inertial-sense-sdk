# SDK: Simple Data Logging Example

This [ISLoggerWrite](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/ISLogger_write) project demonstrates how to use the **cISLogger** c++ class in the [Inertial Sense SDK](https://github.com/inertialsense/inertial-sense-sdk) to log data from [InertialSense](https://inertialsense.com) products (IMX and GPX).

## Files

#### Project Files

* [ISLoggerWriteExample.cpp](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/ISLogger_write/ISLoggerWriteExample.cpp)

#### SDK Files

* [SDK](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src)


## Implementation

### Step: Setup and Enable Logger

```C++
   cISLogger logger;
   logger.InitSave(cISLogger::eLogType::LOGTYPE_RAW, logPath);
   logger.EnableLogging(true);
```

### Step: Log serial port data to file

```C++
   uint8_t buf[512];
   if (int len = serialPortRead(&s_serialPort, buf, sizeof(buf)))
   {
      // Log serial port data to file
      logger.LogData(0, len, buf);
   }
```

If reading one byte at a time from the serial port, the following applies. 

```C++
   uint8_t byte;
   if (serialPortReadChar(&s_serialPort, &byte))
   {
      // Log serial port data to file
      logger.LogData(0, 1, &byte);
   }
```

By default, data logs will be stored in the "IS_logs" directory in the current directory.

``` bash
build/IS_logs/LOG_SN30664_20180323_112822_0001.dat
```

### Enabling a PPD Stream

There must be data streaming out of the IMX in order to log data.  The Post Process Data (PPD) message set is a predefined message set that is recommended for post-processing data analysis.  The following code demonstrates how to enable a PPD stream from the IMX.    

```C++
   // Enable PPD data stream without disabling other messages
   stream_configure_rmc_preset(RMC_PRESET_IMX_PPD, RMC_OPTIONS_PRESERVE_CTRL);

   void stream_configure_rmc_preset(uint64_t bits = 0, uint32_t options = 0) 
   {
      is_comm_instance_t comm = {};
      uint8_t buf[64];
      is_comm_init(&comm, buf, sizeof(buf));

      rmc_t rmc;
      rmc.bits = bits;
      rmc.options = options;

      int len = is_comm_data_to_buf(buf, sizeof(buf), &comm, DID_RMC, sizeof(rmc_t), 0, (void*)&rmc);

      // Write command to serial port
      serialPortWrite(&s_serialPort, buf, len);
   }
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
   cd inertial-sense-sdk/ExampleProjects/ISLogger_write
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
   ./build/ISLoggerWrite /dev/ttyACM0
   ```

## Compile & Run (Windows MS Visual Studio)

1. **Open Visual Studio:** Start Microsoft Visual Stdio.

2. **Open the CMake Project:** 

   - Go to **File** > **Open** > **CMake...**.
   - Navigate to the folder containing your CMake project (where the **CMakeLists.txt** file is located).
   - Select the **CMakeLists.txt** file and click **Open**.

3. **Build the Project:** (F7)

4. **Run the Executable:** 

   ``` bash
   C:\inertial-sense-sdk\ExampleProjects\ISLogger_write\VS_project\Release\ISLoggerWrite.exe COM3
   ```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/inertial-sense-sdk">inertial-sense-sdk</a> GitHub repository, and we will be happy to help.
