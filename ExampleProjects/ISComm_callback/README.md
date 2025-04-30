# SDK: Communications Example Project

This [ISCommCallbackExample](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/ISComm_callback) project demonstrates binary communications with the <a href="https://inertialsense.com">InertialSense</a> products using the Inertial Sense SDK.

## Files

#### Project Files

* [ISCommCallbackExample.cpp](https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects/ISComm_callback/ISCommCallbackExample.cpp)

#### SDK Files

* [data_sets.c](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/data_sets.c)
* [data_sets.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/data_sets.h)
* [ISComm.c](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/ISComm.c)
* [ISComm.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/ISComm.h)
* [serialPort.c](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/serialPort.c)
* [serialPort.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/serialPort.h)
* [serialPortPlatform.c](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/serialPortPlatform.c)
* [serialPortPlatform.h](https://github.com/inertialsense/inertial-sense-sdk/tree/main/src/serialPortPlatform.h)


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
	uint8_t comm_buf[2048];

	// Initialize the comm instance, sets up state tracking, packet parsing, etc.
	is_comm_init(&comm, comm_buf, sizeof(comm_buf));
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

### Step 4: Stop any message broadcasting

```c++
// Stop all broadcasts on the device
uint8_t buf[200];
int len = is_comm_write_to_buf(buf, sizeof(buf), comm, PKT_TYPE_STOP_BROADCASTS_ALL_PORTS, 0, 0, 0, NULL);
if (len > 0) serialPortWrite(&s_serialPort, buf, len);
```

### Step 5: Set configuration (optional)

```C++
// Set INS output Euler rotation in radians to 90 degrees roll for mounting
float rotation[3] = { 90.0f*C_DEG2RAD_F, 0.0f, 0.0f };
uint8_t buf[200];
int len = is_comm_set_data_to_buf(buf, sizeof(buf), comm, DID_FLASH_CONFIG, sizeof(float) * 3, offsetof(nvm_flash_cfg_t, insRotation), rotation);
if (len > 0) serialPortWrite(&s_serialPort, buf, len);
```

### Step 6: Enable message broadcasting

This can be done using the get data command.

#### Get Data Command

```C++
uint8_t buf[200];
int len;

// Ask for INS message (startupNavDtMs x 10).  Set data rate to zero to disable broadcast and pull a single packet.
len = is_comm_get_data_to_buf(buf, sizeof(buf), comm, DID_INS_1, 0, 0, 10);
if (len > 0) serialPortWrite(&s_serialPort, buf, len);

// Ask for GPS message (startupGpsDtMs x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
len = is_comm_get_data_to_buf(buf, sizeof(buf), comm, DID_GPS1_POS, 0, 0, 1);
if (len > 0) serialPortWrite(&s_serialPort, buf, len);

// Ask for IMU message (startupNavDtMs x 10).  This could be as high as 1000 times a second (period multiple of 1)
len = is_comm_get_data_to_buf(buf, sizeof(buf), comm, DID_IMU, 0, 0, 10);
if (len > 0) serialPortWrite(&s_serialPort, buf, len);
```

### Step 7: Handle received data 

```C++
// Handle InertialSense binary (ISB) messages
int parse_isb(unsigned int port, p_data_t* data)
{
	uDatasets *d = (uDatasets *)(data->ptr);

	switch (data->hdr.id)
    {
        case DID_INS_1:
			printf("INS TimeOfWeek: %.3fs, LLA: %3.7f,%3.7f,%5.2f, Euler: %5.1f,%5.1f,%5.1f\r\n",
				d->ins1.timeOfWeek,
				d->ins1.lla[0], d->ins1.lla[1], d->ins1.lla[2],
				d->ins1.theta[0] * C_RAD2DEG_F, d->ins1.theta[1] * C_RAD2DEG_F, d->ins1.theta[2] * C_RAD2DEG_F);
            break;

		case DID_INS_2:
			ixVector3 theta;
			quat2euler(d->ins2.qn2b, theta);		
			printf("INS TimeOfWeek: %.3fs, LLA: %3.7f,%3.7f,%5.2f, Euler: %5.1f,%5.1f,%5.1f\r\n",
				d->ins2.timeOfWeek,
				d->ins2.lla[0], d->ins2.lla[1], d->ins2.lla[2],
				theta[0] * C_RAD2DEG_F, theta[1] * C_RAD2DEG_F, theta[2] * C_RAD2DEG_F);
			break;

		case DID_IMU:
			printf("IMU Time: %.3fs, PQR: %5.1f,%5.1f,%5.1f, ACC: %5.1f,%5.1f,%5.1f,\r\n",
				d->imu.time,
				d->imu.I.pqr[0], d->imu.I.pqr[1], d->imu.I.pqr[2],
				d->imu.I.acc[0], d->imu.I.acc[1], d->imu.I.acc[2]);
            break;

		case DID_GPS1_POS:
			printf("GPS TimeOfWeek: %dms, LLA: %3.7f,%3.7f,%5.2f\r\n", d->gpsPos.timeOfWeekMs, d->gpsPos.lla[0], d->gpsPos.lla[1], d->gpsPos.lla[2]);
			break;
	}

	return 0;
}

// Handle NMEA messages
int parse_nmea(unsigned int port, const unsigned char* msg, int msgSize)
{
	return 0;
}

int main(int argc, char* argv[])
{
    ...
	// Setup callback functions
	is_comm_callbacks_t s_callbacks = {};
	s_callbacks.isbData = parse_isb;
	s_callbacks.nmea    = parse_nmea;

	while (running)
	{
		// Read from serial port
		uint8_t buf[200];
		int bytes_read = serialPortRead(&s_serialPort, buf, sizeof(buf));

		// Parse buffer which will call isb_parse_data() callback
		is_comm_buffer_parse_messages(buf, bytes_read, &comm, &s_callbacks);

		// Sleep to reduce CPU usage
		serialPortSleep(&s_serialPort, 1);
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
   cd inertial-sense-sdk/ExampleProjects/Communications
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
   ./bin/ISCommunicationsExample /dev/ttyUSB0
   ```
## Compile & Run (Windows MS Visual Studio)

1. Open cmake project in Visual Studio
2. Build (F7)
3. Run executable
   ``` bash
   C:\inertial-sense-sdk\ExampleProjects\Communications\VS_project\Release\ISCommCallbackExample.exe COM3
   ```

## Summary

That covers all the basic functionality you need to set up and talk to <a href="https://inertialsense.com">InertialSense</a> products.  If this doesn't cover everything you need, feel free to reach out to us on the <a href="https://github.com/inertialsense/inertial-sense-sdk">inertial-sense-sdk</a> GitHub repository, and we will be happy to help.
