/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

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
#include "util/util.h"

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

enum eExitCodes
{
    EXIT_CODE_SUCCESS 	                            =  0,
    EXIT_CODE_INVALID_COMMAND_LINE                  = -1,
    EXIT_CODE_PARSE_COMMAND_LINE_FAILED             = -2,
    EXIT_CODE_NO_DEVICES_FOUND                      = -3,
    EXIT_CODE_DEVICE_DISCONNECTED                   = -4,
    EXIT_CODE_FIRMWARE_UPDATE_FAILED                = -5,
    EXIT_CODE_FAILED_TO_SETUP_COMMUNICATIONS        = -6,
};

typedef struct
{
    eDataIDs	did;
    int			periodMultiple;
    struct {
        uint64_t    lastRxTime;
        double      rxCount;
    }           rxStats;
} stream_did_t;

typedef struct
{
    did_event_filter_t evFilter;
    uint16_t dest;
    bool sendEVF;
} EVFContainer_t;

typedef struct
{
    uint32_t Addrs[10];
    std::string outDir;
    bool sendEVM;
    bool IMX;
    bool hex;
    uint8_t addrCnt;
} EVMContainer_t;

typedef struct
{
    std::string inFile;
    std::string outFile;
    std::string logType;
    bool extractEv;
} EVOContainer_t;

typedef struct cmd_options_s // we need to name this to make MSVC happy, since we make default assignments in the struct below (updateFirmwareTarget, etc)
{
    std::string comPort; 					// -c com_port
    std::string updateAppFirmwareFilename; 	// -uf file_name
    std::string updateBootloaderFilename; 	// -ub file_name
    std::vector<std::string> fwUpdateCmds;  // commands for firmware updates
    bool forceBootloaderUpdate;				// -fb
    bool bootloaderVerify; 					// -bv
    bool replayDataLog;
    bool softwareReset;
    bool magRecal;
    uint32_t magRecalMode;
    survey_in_t surveyIn;
    bool nmeaRx;
    std::string nmeaMessage;				// A full NMEA message with checksum terminator will be automatically added and then nmeaMessage sent 
    double replaySpeed;
    int displayMode;
    int verboseLevel = ISBootloader::eLogLevel::IS_LOG_LEVEL_INFO;
    
    uint64_t rmcPreset;
    bool persistentMessages;
    stream_did_t datasetEdit;				// -edit DID#=periodMultiple
    std::vector<stream_did_t> datasets;		// -did DID#=periodMultiple	
    
    bool enableLogging;
    std::string logType; 					// -lt=dat
    std::string logPath; 					// -lp path
    float logDriveUsageLimitPercent; 		// -lms=max_drive_percent, 0 for disabled
    float logDriveUsageLimitMb;				// -lmb=max_drive_limit_mb, 0 for disabled
    uint32_t maxLogFileSize; 				// -lmf=max_file_size
    std::string logSubFolder; 				// -lts=1
    int baudRate; 							// -baud=3000000
    bool disableBroadcastsOnClose;	
    
    std::string roverConnection; 			// -rover=type:IP/URL:port:mountpoint:user:password   (server)
    std::string baseConnection; 			// -base=IP:port    (client)	
    
    bool imxflashCfgSet;
    bool gpxflashCfgSet;
    std::string imxFlashCfg;
    std::string gpxFlashCfg;
    uint32_t timeoutFlushLoggerSeconds;
    std::vector<uint32_t> outputOnceDid;	
    std::vector<uint32_t> setAckDid;

    YAML::Node getNode;
    YAML::Node setNode;
    
    uint32_t sysCommand;
    int32_t platformType;
    fwUpdate::target_t updateFirmwareTarget = fwUpdate::TARGET_HOST;
    uint32_t updateFirmwareSlot = 0;
    uint32_t runDurationMs = 0;				// Run for this many millis before exiting (0 = indefinitely)
    bool list_devices = false;				// if true, dumps results of findDevices() including port name.
    EVFContainer_t evFCont = {0};
    EVMContainer_t evMCont = {0};
    EVOContainer_t evOCont;

    bool disableDeviceValidation = false;	// Keep port(s) open even if no devices response is received.
    bool listenMode = false;				// Disable device verification and don't send stop-broadcast command on start.
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
bool cltool_extractEventData();
void cltool_outputUsage();
void cltool_outputHelp();
void cltool_firmwareUpdateWaiter();
void cltool_bootloadUpdateInfo(void* obj, ISBootloader::eLogLevel level, const char* str, ...);
void cltool_firmwareUpdateInfo(void* obj, ISBootloader::eLogLevel level, const char* str, ...);
bool cltool_updateImxFlashCfg(InertialSense& inertialSenseInterface, std::string flashCfgString);
bool cltool_updateGpxFlashCfg(InertialSense& inertialSenseInterface, std::string flashCfgString);

#endif // __CLTOOL_H__

