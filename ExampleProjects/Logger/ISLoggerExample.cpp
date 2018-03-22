/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>

// change these include paths to the correct paths for your project
#include "../../src/data_sets.h"
#include "../../src/serialPortPlatform.h"
#include "../../src/InertialSense.h"

static void dataCallback(InertialSense* i, p_data_t* data, int pHandle)
{
	static uint64_t dataCount;
	printf("Data count: %" PRIu64 "          \r", ++dataCount);
}

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("Please pass the com port and then optionally the log type as the only arguments (dat,sdat,csv,kml).\r\n");
		// In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3 kml" 
		return -1;
	}

	serial_port_t serialPort;

	// very important - the serial port must be initialized to zeros
	memset(&serialPort, 0, sizeof(serialPort));

	// utility class for display and ctrl-c handling
	cInertialSenseDisplay display;

	// InertialSense class wraps communications and logging in a convenient, easy to use class
	InertialSense inertialSense(dataCallback);
	if (!inertialSense.Open(argv[1]))
	{
		std::cout << "Failed to open com port at " << argv[1] << std::endl;
	}
	
	// get log type from command line
	cISLogger::eLogType logType = (argc < 3 ? cISLogger::eLogType::LOGTYPE_DAT : cISLogger::ParseLogType(argv[2]));
	inertialSense.SetLoggerEnabled(true, "", logType);

	// broadcast the standard set of post processing messages (ins, imu, etc.)
	inertialSense.BroadcastBinaryDataRmcPreset();

	// instead of the rmc preset (real-time message controller) you can request individual messages...
	// inertialSense.BroadcastBinaryData(DID_DUAL_IMU, 10); // imu every 10 milliseconds (100 hz)

	std::cout << "Started logger..." << std::endl;

	while (!display.ControlCWasPressed())
	{
		inertialSense.Update();
	}

	inertialSense.Close();
}

