**SDK** - The Inertial Sense open source software development kit provides quick integration for communication with the Inertial Sense product line, including the uINS, uAHRS, and uINS.  It includes data logger, math libraries, and serial port interface for Linux and Windows environments.   

**EvalTool** - Graphical Windows-based desktop program that allows you to explore and test functionality of the Inertial Sense products in real-time.  It has scrolling plots, 3D model representation, table views of all data, data logger, and firmware updating interface for the uINS, uAHRS, or uIMU. The EvalTool can simultaneously interface with multiple Inertial Sense devices.

**CLTool** - Command line utility that can be used to communicate, log data, and update firmware for Inertial Sense products.  Additionally, InertialSenseCLTool serves as example source code to demonstrate how to integrate the Inertial Sense SDK into your own source code.  The InertialSenseCLTool can be compiled in Linux and Windows. 

### Documents

 * [SDK Documentation](http://docs.inertialsense.com/)
 * [User Manual](https://github.com/inertialsense/InertialSenseSDK/files/1308832/IS_User_Manual.pdf)
 * [Datasheet - uINS, uAHRS, and uIMU](https://github.com/inertialsense/InertialSenseSDK/files/1308833/uIMU_uAHRS_uINS_Datasheet.pdf)
 * [Release Notes](https://github.com/inertialsense/InertialSenseSDK/files/1308836/release-notes.txt) - version information and software change log.

### Downloads

 * [SDK & CLTool Binaries](https://github.com/inertialsense/InertialSenseSDK/releases) - pre-compiled release static libraries and executables for Linux and Windows.
 * [SDK & CLTool Source Code](https://github.com/inertialsense/InertialSenseSDK) - open source SDK repository with command line tool and example C/C++ source code.
 * [Firmware and EvalTool](https://github.com/inertialsense/InertialSenseSDK/releases) - uINS, uAHRS, and uIMU firmware and Windows application installer.

### Design Resources

 * [uINS PCB Design Libraries](https://inertialsense.com/resources) - Schematic and layout files for printed circuit board designs.
 * [uINS, uAHRS, uIMU CAD Model](https://inertialsense.com/resources) - 3D step model of uINS and EVB used for CAD and circuit board designs.

### Support

 * Email - support@inertialsense.com

************************************************
## CLTool Instructions

#### Compiling in Linux

1. Create build directory inside SDK/InertialSenseCLTool/.

        $ cd InertialSenseCLTool
        $ mkdir build

2. Run cmake from within build directory.

        $ cd build
        $ cmake ..

 to cross-compile to 32 or 64 bit:

        sudo apt-get install libc6-dev-i386
        sudo apt-get install gcc-multilib g++-multilib

 32 bit

        cmake .. -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_C_FLAGS=-m32

 64 bit

        cmake .. -DCMAKE_CXX_FLAGS=-m64 -DCMAKE_C_FLAGS=-m64

3. Compile using make.

        $ make

4. Add current user to the "dialout" group in order to read and write to the USB serial communication ports:

        $ sudo usermod -a -G dialout $USER
        $ sudo usermod -a -G plugdev $USER

 (**Reboot computer**)

5. Run executable

        $ ./bin/cltool

#### Compiling in Windows (MS Visual Studio)

1. Open Visual Studio solution file (InertialSenseCLTool.sln).

2. Build (F5 or F7).


************************************************
# SDK Instructions

### InertialSense Class

The InertialSense class provides a simple, powerful, and convenient method to communicate, data log, and bootload firmware with the uINS, uAHRS, and uIMU.  It is designed primarily around the Inertial Sense binary protocol.  Please refer to the **InertialSenseCLTool** project files **main.cpp** and **cltool.cpp** with the follow instructions.

#### [COMM INSTRUCTIONS]

* Create InertialSense object and open serial port.

```c
        // [COMM INSTRUCTION] 1.) Create InertialSense object and open serial port. 
        InertialSense inertialSenseInterface(cltool_dataCallback);
        if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str()))
        {	
            cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
            return -1;	// Failed to open serial port
        }
```

* Enable data broadcasting from uINS.

```c
        // [COMM INSTRUCTION] 2.) Enable data broadcasting from uINS
        cltool_setupCommunications(inertialSenseInterface);
```

* The Update() function must be called at regular intervals to send and receive data.

```c
        // Main loop.  Could be in separate thread if desired.
        while (!g_ctrlCPressed)
        {
            // [COMM INSTRUCTION] 3.) Process data and messages
            inertialSenseInterface.Update();

            // Specify the minimum time between read/write updates.
            SLEEP_MS(1);
        }
```

* New data is available in the data callback function.

```c
    // [COMM INSTRUCTION] 4.) This function is called every time there is new data.
    void cltool_dataCallback(InertialSense* i, p_data_t* data)
    {
        // Print data to terminal
        g_inertialSenseDisplay.ProcessData(data);

        // uDatasets is a union of all datasets that we can receive.  See data_sets.h for a full list of all available datasets. 
        uDatasets d = {};
        copyDataPToStructP(&d, data, sizeof(uDatasets));

        // Example of how to access dataset fields.
        switch (data->hdr.id)
        {
        case DID_INS_2:		   
            d.ins2.qn2b;		// quaternion attitude 
            d.ins2.uvw;			// body velocities
            d.ins2.lla;			// latitude, longitude, altitude
            break;
        case DID_INS_1:             
            d.ins1.theta;		// euler attitude
            d.ins1.lla;			// latitude, longitude, altitude
            break;
        case DID_DUAL_IMU:          d.dualImu;      break;
        case DID_DELTA_THETA_VEL:   d.dThetaVel;    break;
        case DID_IMU_1:             d.imu;          break;
        case DID_IMU_2:             d.imu;          break;
        case DID_GPS:               d.gps;          break;
        case DID_MAGNETOMETER_1:    d.mag;          break;
        case DID_MAGNETOMETER_2:    d.mag;          break;
        case DID_BAROMETER:         d.baro;         break;
        case DID_SYS_SENSORS:       d.sysSensors;   break;
        }
    }
```

#### [LOGGER INSTRUCTIONS]
The steps described in the COMM INSTRUCTIONS section are needed in addition to those in this section.

* To configure and start the data logger, call cltool_setupLogger().  This function may be called directly or duplicated for your purposes.

```c
        // [LOGGER INSTRUCTION] Setup data logger
        cltool_setupLogger(inertialSenseInterface);
```

#### [BOOTLOADER INSTRUCTIONS]
The steps described in the COMM INSTRUCTIONS section are needed in addition to those in this section.

* To execute the bootloader, call cltool_runBootloader().  This function may be called directly or duplicated for your purposes.

```c
        // [BOOTLOADER INSTRUCTIONS] Update firmware
        return cltool_runBootloader(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.bootloaderFileName.c_str(), NULL);
```


************************************************
(c) 2014 Inertial Sense, LLC