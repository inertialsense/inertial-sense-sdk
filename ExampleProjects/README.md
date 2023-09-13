# SDK Example Projects

The following example projects are provide with the SDK to demonstrate various capabilities of the <a href="https://inertialsense.com">InertialSense</a> products (uINS, uAHRS, and uIMU) and the Inertial Sense SDK.

## Example Projects

* [Ascii Communications](Ascii/README) - How to communicate using NMEA protocol using SDK.
* [Binary Communications](Communications/README) - How to communicate using InertialSense binary protocol using SDK. 
* [Bootloader](bootloader/README) - How to update firmware on the InertialSense products using SDK.
* [Logger ](Logger/README) - How to data log using using SDK.
* [CLTool](..cltool/README) - A multipurpose command line tool capable most common functionalities, including those of all example projects.

## Compile & Run (Linux/Mac)

The following steps will build executables for all of the example projects.

1. Install necessary dependencies
   ``` bash
   sudo apt update && sudo apt install libusb-1.0-0-dev
   ```
2. Create build directory
   ``` bash
   cd inertial-sense-sdk/ExampleProjects
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
   ./bin/[EXECUTABLE] /dev/ttyUSB0
   ```
## Compile & Run (Windows MS Visual Studio)

The following steps will build executables for all of the example projects.

1. [Install and Configure Visual Studio](../getting-started/#installing-and-configuring-visual-studio)
2. Open the project folder in Visual Studio: Open a local folder -> inertial-sense-sdk/ExampleProjects/
3. Build All
4. Run executable
   ``` bash
   C:\inertial-sense-sdk\ExampleProjects\[PROJECT]\...\[EXECUTABLE.EXE] COM3
   ```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/inertial-sense-sdk">inertial-sense-sdk</a> github repository, and we will be happy to help.
