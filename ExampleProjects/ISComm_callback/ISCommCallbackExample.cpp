/*
MIT LICENSE

Copyright 2014-2018 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// Change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
#include "../../src/ISPose.h"
#include "../../src/ISUtilities.h"
#include "../../src/protocol_nmea.h"

static serial_port_t s_serialPort;
static int running = 1;

void set_configuration(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	// Set INS output Euler rotation in radians to 90 degrees roll for mounting
	float rotation[3] = { 90.0f*C_DEG2RAD_F, 0.0f, 0.0f };
	uint8_t buf[200];
	int len = is_comm_set_data_to_buf(buf, sizeof(buf), comm, DID_FLASH_CONFIG, sizeof(float) * 3, offsetof(nvm_flash_cfg_t, insRotation), rotation);
	if (len > 0) serialPortWrite(&s_serialPort, buf, len);
}

void stop_message_broadcasting(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	// Stop all broadcasts on the device
	uint8_t buf[200];
    int len = is_comm_write_to_buf(buf, sizeof(buf), comm, PKT_TYPE_STOP_BROADCASTS_ALL_PORTS, 0, 0, 0, NULL);
	if (len > 0) serialPortWrite(&s_serialPort, buf, len);
}

void save_persistent_messages(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	system_command_t cfg;
	cfg.command = SYS_CMD_SAVE_PERSISTENT_MESSAGES;
	cfg.invCommand = ~cfg.command;

	uint8_t buf[200];
	int len = is_comm_set_data_to_buf(buf, sizeof(buf), comm, DID_SYS_CMD, 0, 0, &cfg);
	if (len > 0) serialPortWrite(&s_serialPort, buf, len);
}

// Enable message streaming
void enable_message_streaming(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	uint8_t buf[200];
	int len;

	// Ask for INS message (startupNavDtMs x 10).  Set data rate to zero to disable broadcast and pull a single packet.
	len = is_comm_get_data_to_buf(buf, sizeof(buf), comm, DID_INS_1, 0, 0, 10);
	if (len > 0) serialPortWrite(&s_serialPort, buf, len);

#if 1
	// Ask for GPS message (startupGpsDtMs x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
	len = is_comm_get_data_to_buf(buf, sizeof(buf), comm, DID_GPS1_POS, 0, 0, 1);
	if (len > 0) serialPortWrite(&s_serialPort, buf, len);
#endif

#if 1
	// Ask for IMU message (startupNavDtMs x 10).  This could be as high as 1000 times a second (period multiple of 1)
	len = is_comm_get_data_to_buf(buf, sizeof(buf), comm, DID_IMU, 0, 0, 10);
	if (len > 0) serialPortWrite(&s_serialPort, buf, len);
#endif
}

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
	if (argc < 2)
	{
		printf("Please pass the com port as the only argument (i.e. /dev/ttyACM0 or COM5)\r\n");
		// In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3" 
		return -1;
	}

	// Init comm instance
	is_comm_instance_t comm;
	uint8_t comm_buf[2048];

	// Initialize the comm instance, sets up state tracking, packet parsing, etc.
	is_comm_init(&comm, comm_buf, sizeof(comm_buf));

	// Initialize and open serial port

	// Initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
	//  you will need to handle the serial port creation, open and reads yourself. In this
	//  case, you do not need to include serialPort.h/.c and serialPortPlatform.h/.c in your project.
	serialPortPlatformInit(&s_serialPort);

	// Open serial, last parameter is a 1 which means a blocking read, you can set as 0 for non-blocking
	// you can change the baudrate to a supported baud rate (IS_BAUDRATE_*), make sure to reboot the uINS
	//  if you are changing baud rates, you only need to do this when you are changing baud rates.
	if (!serialPortOpen(&s_serialPort, argv[1], IS_BAUDRATE_921600, 1))
	{
		printf("Failed to open serial port on com port %s\r\n", argv[1]);
		return -2;
	}

	int error;

	// Stop any message broadcasting
	stop_message_broadcasting(&s_serialPort, &comm);

#if 0	// Set configuration
	set_configuration(&serialPort, &comm);
#endif

	// Enable message streaming
	enable_message_streaming(&s_serialPort, &comm);

#if 0   // (Optional) Save currently enabled streams as persistent messages enabled after reboot
	save_persistent_messages(&serialPort, &comm);
#endif

	// Setup callback functions
	is_comm_callbacks_t s_callbacks = {};
	s_callbacks.isbData = parse_isb;
	s_callbacks.nmea = parse_nmea;

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
}

