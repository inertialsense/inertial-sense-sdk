/*
MIT LICENSE

Copyright 2014-2018 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/ISComm.h"
#include "../../src/serialPortPlatform.h"
#include "../../src/ISPose.h"
#include "../../src/ISUtilities.h"
#include "../../src/protocol_nmea.h"

static serial_port_t s_serialPort;
static int running = 1;

static void handleIns1Message(ins_1_t* ins)
{
	printf("INS TimeOfWeek: %.3fs, LLA: %3.7f,%3.7f,%5.2f, Euler: %5.1f,%5.1f,%5.1f\r\n",
		ins->timeOfWeek,
		ins->lla[0], ins->lla[1], ins->lla[2],
		ins->theta[0] * C_RAD2DEG_F, ins->theta[1] * C_RAD2DEG_F, ins->theta[2] * C_RAD2DEG_F);
}

static void handleIns2Message(ins_2_t* ins)
{
	ixVector3 theta;
	quat2euler(ins->qn2b, theta);

	printf("INS TimeOfWeek: %.3fs, LLA: %3.7f,%3.7f,%5.2f, Euler: %5.1f,%5.1f,%5.1f\r\n",
		ins->timeOfWeek,
		ins->lla[0], ins->lla[1], ins->lla[2],
		theta[0] * C_RAD2DEG_F, theta[1] * C_RAD2DEG_F, theta[2] * C_RAD2DEG_F);
}

static void handleGpsMessage(gps_pos_t* pos)
{
	printf("GPS TimeOfWeek: %dms, LLA: %3.7f,%3.7f,%5.2f\r\n", pos->timeOfWeekMs, pos->lla[0], pos->lla[1], pos->lla[2]);
}

static void handleImuMessage(imu_t* imu)
{
	printf("IMU Time: %.3fs, PQR: %5.1f,%5.1f,%5.1f, ACC: %5.1f,%5.1f,%5.1f,\r\n",
		imu->time,
		imu->I.pqr[0], imu->I.pqr[1], imu->I.pqr[2],
		imu->I.acc[0], imu->I.acc[1], imu->I.acc[2]);
}

static int portWrite(unsigned int port, const unsigned char* buf, int len)
{
	return serialPortWrite(&s_serialPort, buf, len);
}

static int portRead(int port, unsigned char* buf, int len)
{
	return serialPortRead(&s_serialPort, buf, len);
}

int set_configuration(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	// Set INS output Euler rotation in radians to 90 degrees roll for mounting
	float rotation[3] = { 90.0f*C_DEG2RAD_F, 0.0f, 0.0f };
	if (is_comm_set_data(portWrite, 0, comm, DID_FLASH_CONFIG, sizeof(float) * 3, offsetof(nvm_flash_cfg_t, insRotation), rotation) < 0)
	{
		printf("Failed to encode and write set INS rotation\r\n");
		return -3;
	}

	return 0;
}


int stop_message_broadcasting(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	// Stop all broadcasts on the device
	if (is_comm_stop_broadcasts_all_ports(portWrite, 0, comm) < 0)
	{
		printf("Failed to encode and write stop broadcasts message\r\n");
		return -3;
	}
	return 0;
}


int save_persistent_messages(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	system_command_t cfg;
	cfg.command = SYS_CMD_SAVE_PERSISTENT_MESSAGES;
	cfg.invCommand = ~cfg.command;

	if (is_comm_set_data(portWrite, 0, comm, DID_SYS_CMD, 0, 0, &cfg) < 0)
	{
		printf("Failed to write save persistent message\r\n");
		return -3;
	}
	return 0;
}


int enable_message_broadcasting(serial_port_t *serialPort, is_comm_instance_t *comm)
{
	// Ask for INS message w/ update 40ms period (4ms source period x 10).  Set data rate to zero to disable broadcast and pull a single packet.
	if (is_comm_get_data(portWrite, 0, comm, DID_INS_1, 0, 0, 10) < 0)
	{
		printf("Failed to encode and write get INS message\r\n");
		return -4;
	}

#if 1
	// Ask for GPS message at period of 200ms (200ms source period x 1).  Offset and size can be left at 0 unless you want to just pull a specific field from a data set.
	if (is_comm_get_data(portWrite, 0, comm, DID_GPS1_POS, 0, 0, 1) < 0)
	{
		printf("Failed to encode and write get GPS message\r\n");
		return -5;
	}
#endif

#if 0
	// Ask for IMU message at period of 100ms (1ms source period x 100).  This could be as high as 1000 times a second (period multiple of 1)
	if (is_comm_get_data(portWrite, 0, comm, DID_IMU, 0, 0, 100) < 0)
	{
		printf("Failed to encode and write get IMU message\r\n");
		return -6;
	}
#endif
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

	// STEP 2: Init comm instance
	is_comm_instance_t comm;
	uint8_t buffer[2048];

	// Initialize the comm instance, sets up state tracking, packet parsing, etc.
	is_comm_init(&comm, buffer, sizeof(buffer));

	// STEP 3: Initialize and open serial port

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

	// STEP 4: Stop any message broadcasting
	if ((error = stop_message_broadcasting(&s_serialPort, &comm)))
	{
		return error;
	}


#if 0	// STEP 5: Set configuration
	if ((error = set_configuration(&serialPort, &comm)))
	{
		return error;
	}
#endif

	// STEP 6: Enable message broadcasting
	if ((error = enable_message_broadcasting(&s_serialPort, &comm)))
	{
		return error;
	}

#if 0   // STEP 7: (Optional) Save currently enabled streams as persistent messages enabled after reboot
	save_persistent_messages(&serialPort, &comm);
#endif

	// STEP 8: Handle received data
	uint8_t inByte;

	while (running)
	{
		// Read one byte with a 20 millisecond timeout
		while (serialPortReadCharTimeout(&s_serialPort, &inByte, 20) > 0)
		{
			// timeMs = current_timeMs();
			switch (is_comm_parse_byte(&comm, inByte))
			{
			case _PTYPE_INERTIAL_SENSE_DATA:
				switch (comm.rxPkt.hdr.id)
				{
				case DID_INS_1:
					handleIns1Message((ins_1_t*)comm.rxPkt.data.ptr);
					break;

				case DID_INS_2:
					handleIns2Message((ins_2_t*)comm.rxPkt.data.ptr);
					break;

				case DID_GPS1_POS:
					handleGpsMessage((gps_pos_t*)comm.rxPkt.data.ptr);
					break;

				case DID_IMU:
					handleImuMessage((imu_t*)comm.rxPkt.data.ptr);
					break;

					// TODO: add other cases for other data ids that you care about
				}
				break;

			case _PTYPE_NMEA:
				switch (getNmeaMsgId(comm.rxPkt.data.ptr, comm.rxPkt.dataHdr.size))
				{
                case NMEA_MSG_ID_GNGGA:
					// Access NMEA message here:
					// comm.dataPtr 
					// comm.dataHdr.size
                    break;
                }
				break;

			default:
				break;
			}
		}
	}
}

