/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

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


int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("Provide the log directory as an argument: $ ./ISLoggerReadExample sandbox/20241101_123142\n");
		return -1;
	}

	string logPath = argv[1];

    cISLogger logger;
    logger.LoadFromDirectory(logPath);

	cout << "Started CISLogger read.  Press ctrl-c to quit." << endl;

	// Utility class for ctrl-c handling
	cInertialSenseDisplay display;
	// display.SetDisplayMode(cInertialSenseDisplay::DMODE_SCROLL);
	// display.SetDisplayMode(cInertialSenseDisplay::DMODE_PRETTY);
	display.SetDisplayMode(cInertialSenseDisplay::DMODE_STATS);	
    display.SetKeyboardNonBlocking();

	std::vector<shared_ptr<cDeviceLog>> devices = logger.DeviceLogs();
    for (auto deviceLog : devices)
	{
		if (display.ExitProgram())
		{
			return 0;
		}

        p_data_buf_t* data = NULL;
        while ((data = deviceLog->ReadData()))
		{
#if 0
			double replaySpeedX = 0;	// Set to zero for non-realtime, fast as possible
			display.ProcessData(data, true, replaySpeedX);
			display.PrintData();
#endif

#if 0
			printf("ID: %3d\n", data->hdr.id);
#endif

#if 0
			logger.PrintStatistics();
#endif
		}

		// Scan for "q" press to exit program
		display.GetKeyboardInput();
	}
	// Revert non-blocking keyboard
	display.ResetTerminalMode();

	logger.CloseAllFiles();
}

