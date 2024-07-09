/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include "ISDisplay.h"
#include "ISLogger.h"
#include "ISUtilities.h"
#include "serialPortPlatform.h"

using namespace std;

static comm_port_t s_serialPort = {};


/**
 * Send the DID_RMC with the specified bits and options.
 * This is used to start streaming of requested DIDs to be logged.
 * Sending 0x0 and 0x0 (param defaults) will turn off streaming.
 */
void stream_configure_rmc_preset(uint64_t bits = 0, uint32_t options = 0) 
{
//	is_comm_instance_t comm = {};
	uint8_t buf[64];
	is_comm_init(&s_serialPort.comm, buf, sizeof(buf));

	rmc_t rmc;
	rmc.bits = bits;
	rmc.options = options;

	int len = is_comm_data_to_buf(buf, sizeof(buf), &s_serialPort.comm, DID_RMC, sizeof(rmc_t), 0, (void*)&rmc);

	// Write command to serial port
	serialPortWrite((port_handle_t)&s_serialPort, buf, len);
}


int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("Provide the port as an argument: $ ./ISLoggerExample /dev/ttyACM0\n");
		return -1;
	}

	string portName = string(argv[1]);
	int baudrate = 921600;
	string logPath = "test_log";

    // Setup and enable logger.  Select the LOGTYPE (i.e. dat, raw, csv)
    cISLogger logger;
    auto devLog = logger.registerDevice(IS_HARDWARE_TYPE_IMX, 12345);   // if you know this information, you can pass it, but it's not important that it match your actual hardware.
    logger.InitSave(cISLogger::eLogType::LOGTYPE_RAW, logPath);
    logger.registerDevice(IS_HARDWARE_TYPE_IMX, 12345);
    logger.EnableLogging(true);

    // Open serial port
	serialPortPlatformInit((port_handle_t)&s_serialPort);
	if (serialPortOpen((port_handle_t)&s_serialPort, portName.c_str(), baudrate, 0) == 0)
	{
		cout << "Failed to open port: " << portName;
		return -1;
	}

    // Enable PPD data stream without disabling other messages
	stream_configure_rmc_preset(RMC_PRESET_PPD_BITS, RMC_OPTIONS_PRESERVE_CTRL);

	cout << "Started logger.  Press ctrl-c to quit." << endl;

	// Utility class for ctrl-c handling
	cInertialSenseDisplay display;
	while (!display.ExitProgram())
    {
		// Prevent CPU overload
    	SLEEP_MS(1);

		uint8_t buf[512];
		if (int len = serialPortRead((port_handle_t)&s_serialPort, buf, sizeof(buf)))
		{
			// Log serial port data to file
			logger.LogData(devLog, len, buf);

			printf("Log file size: %.3f MB \r", logger.LogSizeAllMB());
		}
    }

	// Write remaining data and close log file(s)
	logger.CloseAllFiles();
}

