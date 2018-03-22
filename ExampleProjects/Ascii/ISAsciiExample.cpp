/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// change these include paths to the correct paths for your project
#include "../../src/serialPortPlatform.h"
#include "../../src/ISComm.h"

static bool running = true;

int main(int argc, char* argv[])
{
	unsigned char* asciiData;
	unsigned char asciiLine[512];
	serial_port_t serialPort;

	// very important - the serial port must be initialized to zeros
	memset(&serialPort, 0, sizeof(serialPort));

	if (argc < 2)
	{
		printf("Please pass the com port as the only argument\r\n");
		// In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3" 
		return -1;
	}

	// initialize the serial port (Windows, MAC or Linux) - if using an embedded system like Arduino,
	//  you will need to handle the serial port creation, open and reads yourself. In this
	//  case, you do not need to include serialPort.h/.c and serialPortPlatform.h/.c in your project.
	serialPortPlatformInit(&serialPort);

	// open serial, last parameter is a 1 which means a blocking read, you can set as 0 for non-blocking
	// you can change the baudrate to a supported baud rate (IS_BAUDRATE_*), make sure to reboot the uINS
	//  if you are changing baud rates, you only need to do this when you are changing baud rates.
	if (!serialPortOpen(&serialPort, argv[1], IS_BAUDRATE_3000000, 1))
	{
		printf("Failed to open serial port on com port %s\r\n", argv[1]);
		return -2;
	}

	// stop all broadcasts on the device, we don't want binary message coming through while we are doing ASCII
	if (!serialPortWriteAscii(&serialPort, "STPB", 4))
	{
		printf("Failed to encode stop broadcasts message\r\n");
		return -3;
	}

	// ASCII protocol is based on NMEA protocol https://en.wikipedia.org/wiki/NMEA_0183
	// turn on the INS message at a period of 100 milliseconds (10 hz)
	// serialPortWriteAscii takes care of the leading $ character, checksum and ending \r\n newline
	// ASCB message enables ASCII broadcasts
	// ASCB fields: 1:options, 2:PIMU, 3:PPIMU, 4:PINS1, 5:PINS2, 6:PGPSP, 7:reserved, 8:GPGGA, 9:GPGLL, 10:GPGSA, 11:GPRMC
	// options can be 0 for current serial port, 1 for serial 0, 2 for serial 1 or 3 for both serial ports
	// Instead of a 0 for a message, it can be left blank (,,) to not modify the period for that message
	// please see the user manual for additional updates and notes

	// Get PINS1 @ 10Hz on the connected serial port, leave all other broadcasts the same
	const char* asciiMessage = "ASCB,0,,,100,,,,,,,";

	// Get PIMU @ 50Hz, GPGGA @ 5Hz, both serial ports, set all other periods to 0
	// const char* asciiMessage = "ASCB,3,20,0,0,0,0,0,100,0,0,0";

	// Stop all messages / broadcasts
	// const char* asciiMessage = "STPB";
																				
	if (!serialPortWriteAscii(&serialPort, asciiMessage, (int)strnlen(asciiMessage, 128)))
	{
		printf("Failed to encode ASCII get INS message\r\n");
		return -4;
	}

	/*
	// you can also enable ASCII messages using the binary protocol
	is_comm_instance_t comm;
	uint8_t commBuffer[2048];
	comm.buffer = commBuffer;
	comm.bufferSize = sizeof(commBuffer);
	is_comm_init(&comm);
	ascii_msgs_t ascii;
	memset(&ascii, 0, sizeof(ascii));
	ascii.gprmc = 100;
	int messageSize = is_comm_set_data(&comm, DID_ASCII_BCAST_PERIOD, 0, sizeof(ascii), &ascii);
	if (messageSize > 0)
	{
		serialPortWrite(&serialPort, comm.buffer, messageSize);
	}
	*/

	// you can set running to false with some other piece of code to break out of the loop and end the program
	while (running)
	{
		if (serialPortReadAscii(&serialPort, asciiLine, sizeof(asciiLine), &asciiData) > 0)
		{
			printf("%s\n", asciiData);
		}
	}
}

