/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __CLTOOL_H__
#define __CLTOOL_H__

#include <stdio.h>
#include <iostream>
#include <iomanip>      // std::setw
#include <algorithm>
#include <string>

// change these includes to be the correct path for your system
#include "InertialSense.h" // best to include this file first
#include "ISDisplay.h"
#include "ISUtilities.h"
#include "ISBootloaderBase.h"

#define APP_NAME                "cltool"
#if PLATFORM_IS_WINDOWS
#define APP_EXT                 ".exe"
#define EXAMPLE_PORT            "COM5"
#define EXAMPLE_LOG_DIR         "c:\\logs\\20170117_222549       "
#define EXAMPLE_FIRMWARE_FILE   "c:\\fw\\IS_IMX-5.hex"
#define EXAMPLE_BOOTLOADER_FILE "c:\\fw\\IS_bootloader-STM32L4.hex"
#define EXAMPLE_SPACE_1         "    "
#define EXAMPLE_SPACE_2         "   "
#else
#define APP_EXT	                ""
#define EXAMPLE_PORT            "/dev/ttyS2"
#define EXAMPLE_LOG_DIR         "logs/20170117_222549                "
#define EXAMPLE_FIRMWARE_FILE   "fw/IS_IMX-5.hex"
#define EXAMPLE_BOOTLOADER_FILE "fw/IS_bootloader-STM32L4.hex"
#define EXAMPLE_SPACE_1         "    "
#define EXAMPLE_SPACE_2			"         "
#endif

typedef struct
{
	eDataIDs	did;
	int			periodMultiple;
} stream_did_t;

typedef struct
{
	std::string comPort; 					// -c com_port
	std::string updateAppFirmwareFilename; 	// -uf file_name
	std::string updateBootloaderFilename; 	// -ub file_name
	bool forceBootloaderUpdate;				// -fb
	bool bootloaderVerify; 					// -bv
	bool replayDataLog;
	bool softwareResetImx;
	bool softwareResetEvb;
	bool magRecal;
	uint32_t magRecalMode;
	survey_in_t surveyIn;
	std::string asciiMessages;
	double replaySpeed;
	int displayMode;	
	
	uint64_t rmcPreset;
	bool persistentMessages;
	stream_did_t datasetEdit;				// -edit DID#=periodMultiple
	std::vector<stream_did_t> datasets;		// -did DID#=periodMultiple	
	
	bool enableLogging;
	std::string logType; 					// -lt=dat
	std::string logPath; 					// -lp path
	float maxLogSpacePercent; 				// -lms=max_space_mb
	uint32_t maxLogFileSize; 				// -lmf=max_file_size
	std::string logSubFolder; 				// -lts=1
	int baudRate; 							// -baud=3000000
	bool disableBroadcastsOnClose;	
	
	std::string roverConnection; 			// -rover=type:IP/URL:port:mountpoint:user:password   (server)
	std::string baseConnection; 			// -base=IP:port    (client)	
	
	std::string flashCfg;
	std::string evbFlashCfg;	
	uint32_t timeoutFlushLoggerSeconds;
	uint32_t outputOnceDid;	
	
	uint32_t sysCommand;
	int32_t platformType;
	bool chipEraseEvb2;
} cmd_options_t;

extern cmd_options_t g_commandLineOptions;
extern serial_port_t g_serialPort;
extern cInertialSenseDisplay g_inertialSenseDisplay;
extern bool g_ctrlCPressed;

int cltool_main(int argc, char* argv[]);
int cltool_serialPortSendComManager(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend);

// returns false if failure
bool cltool_setupLogger(InertialSense& inertialSenseInterface);
bool cltool_parseCommandLine(int argc, char* argv[]);
bool cltool_replayDataLog();
void cltool_outputUsage();
void cltool_outputHelp();
void cltool_firmwareUpdateWaiter();
void cltool_bootloadUpdateInfo(void* obj, const char* str, ISBootloader::eLogLevel level);
bool cltool_updateFlashCfg(InertialSense& inertialSenseInterface, std::string flashCfg); // true if should continue
bool cltool_updateEvbFlashCfg(InertialSense& inertialSenseInterface, std::string evbFlashCfg); // true if should continue

#endif // __CLTOOL_H__

