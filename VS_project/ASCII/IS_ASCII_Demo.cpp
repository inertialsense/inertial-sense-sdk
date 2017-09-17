#include "../../src/serialPortPlatform.h"

#include <conio.h>

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		printf("Please specify the com port as the only argument\r\n");
		return -1;
	}
	const char* comPort = argv[1];
	serial_port_t serialPort = { 0 };
	serialPortPlatformInit(&serialPort);
	if (serialPortOpen(&serialPort, comPort, BAUDRATE_3000000, 0) == 0)
	{
		printf("Failed to open serial port at com port %s\r\n", comPort);
		return -2;
	}

	char msg[512];

	// send the stop all broadcasts message
	serialPortWriteAscii(&serialPort, "STPB", 4);

	// periods are in milliseconds
	// a blank period means do not modify the existing broadcast period for that message
	sprintf(msg, "ASCB,%s,%s,%s,%s,%s,%s,%s,%s,%s",
		"50", // IMU - Dual IMU sensor data (two sets of 3-axis gyros and accelerometers) in the body frame.
		"50", // INS1 - INS output with Euler angles and NED offset from the reference LLA.
		"", // INS2 - INS output with quaternion attitude.
		"", // GPS POS - GPS position data.
		"", // GPS VEL - GPS velocity data.
		"200", // GGA - NMEA global positioning system fix.
		"", // GLL - NMEA geographic position, latitude / longitude and time.
		"", // GSA - NMEA GPS DOP and active satellites.
		""  // DTV - Integrated IMU, delta theta velocity (conning and sculling integrals) in the body frame.
	);

	serialPortWriteAscii(&serialPort, msg, strlen(msg));

	// read in lines until key press
	while (!_kbhit())
	{
		int count = serialPortRead(&serialPort, (unsigned char*)msg, sizeof(msg) - 1);
		msg[count] = '\0';
		printf(msg);
	}

	serialPortClose(&serialPort);
}

