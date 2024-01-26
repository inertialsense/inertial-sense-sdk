/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "cltool.h"
#include <string.h>
#include "ISDataMappings.h"

using namespace std;


cmd_options_t g_commandLineOptions = {};
serial_port_t g_serialPort;
cInertialSenseDisplay g_inertialSenseDisplay;
static bool g_internal = false;

int cltool_serialPortSendComManager(CMHANDLE cmHandle, int pHandle, buffer_t* bufferToSend)
{
	(void)cmHandle;
	(void)pHandle;
	return serialPortWrite(&g_serialPort, bufferToSend->buf, bufferToSend->size);
}

bool cltool_setupLogger(InertialSense& inertialSenseInterface)
{
	// Enable logging in continuous background mode
	return inertialSenseInterface.SetLoggerEnabled
	(
		g_commandLineOptions.enableLogging, // enable logger
		g_commandLineOptions.logPath, // path to log to, if empty defaults to DEFAULT_LOGS_DIRECTORY
		cISLogger::ParseLogType(g_commandLineOptions.logType), // log type
		g_commandLineOptions.rmcPreset, // Stream rmc preset
        RMC_OPTIONS_PRESERVE_CTRL,
		g_commandLineOptions.maxLogSpacePercent, // max space in percentage of free space to use, 0 for unlimited
		g_commandLineOptions.maxLogFileSize, // each log file will be no larger than this in bytes
		g_commandLineOptions.logSubFolder // log sub folder name
	);

	// Call these elsewhere as needed
// 	inertialSenseInterface.EnableLogger(false);	// Enable/disable during runtime
// 	inertialSenseInterface.CloseLogger();		// Stop logging and save remaining data to file
}

static bool startsWith(const char* str, const char* pre)
{
	size_t lenpre = strlen(pre), lenstr = strlen(str);
	return lenstr < lenpre ? false : strncasecmp(pre, str, lenpre) == 0;
}

static bool matches(const char* str, const char* pre)
{
	size_t lenpre = strlen(pre), lenstr = strlen(str);
	return lenstr != lenpre ? false : strncasecmp(pre, str, lenpre) == 0;
}

#define CL_DEFAULT_BAUD_RATE				IS_BAUDRATE_DEFAULT 
#define CL_DEFAULT_COM_PORT					"*"
#define CL_DEFAULT_DISPLAY_MODE				cInertialSenseDisplay::DMODE_SCROLL
#define CL_DEFAULT_LOG_TYPE					"dat"
#define CL_DEFAULT_LOGS_DIRECTORY			DEFAULT_LOGS_DIRECTORY
#define CL_DEFAULT_ENABLE_LOGGING			false 
#define CL_DEFAULT_MAX_LOG_FILE_SIZE		1024 * 1024 * 5
#define CL_DEFAULT_MAX_LOG_SPACE_PERCENT	0.5f 
#define CL_DEFAULT_REPLAY_SPEED				1.0
#define CL_DEFAULT_BOOTLOAD_VERIFY			false

bool read_did_argument(stream_did_t *dataset, string s)
{
	// Try to use DID number
	eDataIDs did = strtol(s.c_str(), NULL, 10);

	std::string::size_type pos = s.find('=');

	if (did <= DID_NULL || did >= DID_COUNT)
	{	// Number is invalid.  Use DID name.
		string name = s;
		if (pos != std::string::npos)
		{	// Remove equal sign
			name = s.substr(0, pos);
		}

		did = cISDataMappings::GetDataSetId(name);
	}

	if (did > DID_NULL && did < DID_COUNT)
	{	// DID is valid
		dataset->did = did;
		dataset->periodMultiple = 1;

		if (pos != std::string::npos)
		{	// Contains '='
			string m = s.substr(pos + 1);

			if (m.find(" ") == std::string::npos)
			{	// Found period multiple following equal sign (i.e. DID=periodMultiple: 4=4 or DID_INS_1=4)
				dataset->periodMultiple = strtol(m.c_str(), NULL, 10);
			}
		}

		return true;
	}

	return false;
}

string cltool_version()
{
	string info;
#if defined(IS_SDK_DESCRIBE_TAG)
	info += string("") + IS_SDK_DESCRIBE_TAG;
#endif
#if defined(IS_SDK_BUILD_DATE) && defined(IS_SDK_BUILD_TIME)
	info += string(" ") + IS_SDK_BUILD_DATE + " " + IS_SDK_BUILD_TIME;
#endif
	return info;
}

void print_dids()
{
#if defined(INCLUDE_LUNA_DATA_SETS)
	for (eDataIDs id = 0; id < DID_COUNT; id++)
#else
	for (eDataIDs id = 0; id < DID_COUNT_UINS; id++)
#endif
	{
		printf("(%d) %s\n", id, cISDataMappings::GetDataSetName(id));
	}
	cltool_outputHelp();
}

void enable_display_mode(int mode = cInertialSenseDisplay::DMODE_PRETTY)
{   
    g_commandLineOptions.displayMode = mode;
}

bool cltool_parseCommandLine(int argc, char* argv[])
{
	// defaults
	g_commandLineOptions.baudRate = CL_DEFAULT_BAUD_RATE;
	g_commandLineOptions.comPort = CL_DEFAULT_COM_PORT;
	g_commandLineOptions.displayMode = CL_DEFAULT_DISPLAY_MODE;
	g_commandLineOptions.rmcPreset = 0;
	g_commandLineOptions.enableLogging = CL_DEFAULT_ENABLE_LOGGING;
	g_commandLineOptions.logType = CL_DEFAULT_LOG_TYPE;
	g_commandLineOptions.logPath = CL_DEFAULT_LOGS_DIRECTORY;
	g_commandLineOptions.logSubFolder = cISLogger::CreateCurrentTimestamp();
	g_commandLineOptions.maxLogFileSize = CL_DEFAULT_MAX_LOG_FILE_SIZE;
	g_commandLineOptions.maxLogSpacePercent = CL_DEFAULT_MAX_LOG_SPACE_PERCENT;
	g_commandLineOptions.replaySpeed = CL_DEFAULT_REPLAY_SPEED;
	g_commandLineOptions.bootloaderVerify = CL_DEFAULT_BOOTLOAD_VERIFY;
	g_commandLineOptions.timeoutFlushLoggerSeconds = 3;
	g_commandLineOptions.asciiMessages = "";
	g_commandLineOptions.updateBootloaderFilename = "";
	g_commandLineOptions.forceBootloaderUpdate = false;

    g_commandLineOptions.surveyIn.state = 0;
    g_commandLineOptions.surveyIn.maxDurationSec = 15 * 60; // default survey of 15 minutes
    g_commandLineOptions.surveyIn.minAccuracy = 0;
	g_commandLineOptions.outputOnceDid = 0;
	g_commandLineOptions.platformType = -1;

	if(argc <= 1)
	{	// Display usage menu if no options are provided 
		cltool_outputUsage();
		return false;
	}

	// printf("Arguments: %d\n\n%s\n\n", argc, argv[0]);

	// parse command line.  Keep these options in alphabetic order!
	for (int i = 1; i < argc; i++)
	{
		const char* a = argv[i];
        if (startsWith(a, "--"))
        {	// Treat "--" same as "-"
			a++;
        }
        
		if (startsWith(a, "-asciiMessages="))
		{
			g_commandLineOptions.asciiMessages = &a[15];
            enable_display_mode();
		}
		else if (startsWith(a, "-base="))
		{
			g_commandLineOptions.baseConnection = &a[6];
            enable_display_mode();
		}
		else if (startsWith(a, "-baud="))
		{
			g_commandLineOptions.baudRate = strtol(&a[6], NULL, 10);
		}
		else if (startsWith(a, "-chipEraseEvb"))
		{
			g_commandLineOptions.chipEraseEvb2 = true;
		}
		else if (startsWith(a, "-chipEraseIMX"))
		{
			g_commandLineOptions.sysCommand = SYS_CMD_MANF_CHIP_ERASE;
		}
		else if (matches(a, "-c") && (i + 1) < argc)
		{
			g_commandLineOptions.comPort = argv[++i];	// use next argument
		}
		else if (startsWith(a, "-dboc"))
		{
			g_commandLineOptions.disableBroadcastsOnClose = true;
		}
		else if (startsWith(a, "-dids"))
		{
			print_dids();
			return false;
		}
		else if (startsWith(a, "-did") && (i + 1) < argc)
		{
			while ((i + 1) < argc && !startsWith(argv[i + 1], "-"))	// next argument doesn't start with "-"
			{
				if (g_commandLineOptions.outputOnceDid)
				{
					i++;
				}
				else
				{
					stream_did_t dataset = {};
					if (read_did_argument(&dataset, argv[++i]))		// use next argument
					{
						if (dataset.periodMultiple == 0)
						{
							g_commandLineOptions.outputOnceDid = dataset.did;
							g_commandLineOptions.datasets.clear();
						}
						g_commandLineOptions.datasets.push_back(dataset);
					}
				}
			}			
            enable_display_mode();
		}
		else if (startsWith(a, "-edit"))
		{
			stream_did_t dataset = {};
			if (((i + 1) < argc) && read_did_argument(&dataset, argv[++i]))	// use next argument
			{
				g_commandLineOptions.datasetEdit = dataset;
			}
			else
			{	// Invalid argument
				print_dids();
				return false;
			}
		}
		else if (startsWith(a, "-evbFlashCfg="))
		{
			g_commandLineOptions.evbFlashCfg = &a[13];
		}
		else if (startsWith(a, "-evbFlashCfg"))
		{
			g_commandLineOptions.evbFlashCfg = ".";
		}
        else if (startsWith(a, "-evbReset"))
        {
            g_commandLineOptions.softwareResetEvb = true;
        }
		else if (startsWith(a, "-factoryReset"))
		{
			g_commandLineOptions.sysCommand = SYS_CMD_MANF_FACTORY_RESET;
		}		
		else if (startsWith(a, "-fb"))
		{
			g_commandLineOptions.forceBootloaderUpdate = true;
		}
		else if (startsWith(a, "-flashCfg="))
		{
			g_commandLineOptions.flashCfg = &a[10];
		}
		else if (startsWith(a, "-flashCfg"))
		{
			g_commandLineOptions.flashCfg = ".";
		}
		else if (startsWith(a, "-hi") || startsWith(a, "--hi"))
		{
			g_internal = true;
			cltool_outputUsage();
			return false;
		}
		else if (startsWith(a, "-h") || startsWith(a, "--h") || startsWith(a, "-help") || startsWith(a, "--help"))
		{
			cltool_outputUsage();
			return false;
		}
		else if (startsWith(a, "-lms="))
		{
			g_commandLineOptions.maxLogSpacePercent = (float)atof(&a[5]);
            enable_display_mode();
		}
		else if (startsWith(a, "-lmf="))
		{
			g_commandLineOptions.maxLogFileSize = (uint32_t)strtoul(&a[5], NULL, 10);
            enable_display_mode();
		}
        else if (startsWith(a, "-log-flush-timeout="))
        {
            g_commandLineOptions.timeoutFlushLoggerSeconds = strtoul(&a[19], NULLPTR, 10);
            enable_display_mode();
        }
        else if (startsWith(a, "-lts="))
		{
			const char* subFolder = &a[5];
			if (*subFolder == '1' || startsWith(subFolder, "true"))
			{
				g_commandLineOptions.logSubFolder = cISLogger::CreateCurrentTimestamp();
			}
			else if (*subFolder == '\0' || *subFolder == '0' || startsWith(subFolder, "false"))
			{
				g_commandLineOptions.logSubFolder = cISLogger::g_emptyString;
			}
			else
			{
				g_commandLineOptions.logSubFolder = subFolder;
			}
		}
		else if (startsWith(a, "-lp") && (i + 1) < argc)
		{
			g_commandLineOptions.logPath = argv[++i];	// use next argument;
		}
		else if (startsWith(a, "-lt="))
		{
			g_commandLineOptions.logType = &a[4];
		}
		else if (startsWith(a, "-lon"))
		{
			g_commandLineOptions.enableLogging = true;
		}
		else if (startsWith(a, "-magRecal"))
		{
			g_commandLineOptions.rmcPreset = 0;
			g_commandLineOptions.magRecal = true;
			g_commandLineOptions.magRecalMode = strtol(a + 9, NULL, 10);
            enable_display_mode();
		}
		else if (startsWith(a, "-platform"))
		{
			#define PLATFORM_TYPE_TAG_LEN	10
			if(strlen(a) <= PLATFORM_TYPE_TAG_LEN || !isdigit(a[PLATFORM_TYPE_TAG_LEN]))
			{
				cout << "Platform type not specified.\n\n";
				return false;
			}

			int platformType = (uint32_t)strtoul(&a[PLATFORM_TYPE_TAG_LEN], NULL, 10);
			if (platformType < 0 || platformType >= PLATFORM_CFG_TYPE_COUNT)
			{
				cout << "Invalid platform type: " << platformType << "\n\n";
				return false;
			}
			else
			{	// Valid platform
				g_commandLineOptions.platformType = platformType;
			}
		}
		else if (startsWith(a, "-presetPPD"))
		{
			g_commandLineOptions.rmcPreset = RMC_PRESET_PPD_GROUND_VEHICLE;
            enable_display_mode();
		}
		else if (startsWith(a, "-presetINS2"))
		{
			g_commandLineOptions.rmcPreset = RMC_PRESET_INS_BITS;
            enable_display_mode();
		}
        else if (startsWith(a, "-persistent"))
        {
            g_commandLineOptions.persistentMessages = true;
        }
        else if (startsWith(a, "-q"))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_QUIET;
		}
		else if (startsWith(a, "-rp") && (i + 1) < argc)
		{
			g_commandLineOptions.replayDataLog = true;
			g_commandLineOptions.logPath = argv[++i];	// use next argument
            enable_display_mode();
		}
		else if (startsWith(a, "-rs="))
		{
			g_commandLineOptions.replayDataLog = true;
			g_commandLineOptions.replaySpeed = (float)atof(&a[4]);
            enable_display_mode();
		}
        else if (startsWith(a, "-resetEvb"))
        {
            g_commandLineOptions.softwareResetEvb = true;
        }
        else if (startsWith(a, "-reset"))
        {
            g_commandLineOptions.softwareResetImx = true;
        }		
		else if (startsWith(a, "-romBootloader"))
		{
			g_commandLineOptions.sysCommand = SYS_CMD_MANF_ENABLE_ROM_BOOTLOADER;
		}
		else if (startsWith(a, "-rover="))
		{
			g_commandLineOptions.roverConnection = &a[7];

			// DID_GPS1_POS must be enabled for NTRIP VRS to supply rover position.
			stream_did_t dataset = {};
			read_did_argument(&dataset, "DID_GPS1_POS");
			g_commandLineOptions.datasets.push_back(dataset);
            enable_display_mode();
		}
		else if (startsWith(a, "-r"))
		{
			g_commandLineOptions.replayDataLog = true;
            enable_display_mode();
		}
        else if (startsWith(a, "-stats"))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_STATS;
            enable_display_mode();
		}
        else if (startsWith(a, "-survey="))
        {
            g_commandLineOptions.rmcPreset = 0;
            g_commandLineOptions.surveyIn.state = strtol(a + 8, NULL, 10);
            int maxDurationSec = strtol(a + 10, NULL, 10);
            if (maxDurationSec > 5)
            {
                g_commandLineOptions.surveyIn.maxDurationSec = maxDurationSec;
            }
            enable_display_mode();
        }
		else if (startsWith(a, "-sysCmd="))
		{
			g_commandLineOptions.sysCommand = (uint32_t)strtoul(&a[8], NULL, 10);
		}		
		else if (startsWith(a, "-s"))
		{
            enable_display_mode(cInertialSenseDisplay::DMODE_SCROLL);
		}
		else if (startsWith(a, "-ub") && (i + 1) < argc)
		{
			g_commandLineOptions.updateBootloaderFilename = argv[++i];	// use next argument
		}
        else if (startsWith(a, "-uf") && (i + 1) < argc)
        {
            g_commandLineOptions.updateAppFirmwareFilename = argv[++i];	// use next argument
        }
		else if (startsWith(a, "-uv"))
		{
			g_commandLineOptions.bootloaderVerify = true;
		}
		else if (startsWith(a, "-v") || startsWith(a, "--version"))
		{
			cout << cltool_version() << endl;
			return false;
		}
		else
		{
			cout << "Unrecognized command line option: " << a << endl;
			cltool_outputHelp();
			return false;
		}
	}

	// We are either using a serial port or replaying data
	if ((g_commandLineOptions.comPort.length() == 0) && !g_commandLineOptions.replayDataLog)
	{
		cltool_outputUsage();
		return false;
	}
	else if (g_commandLineOptions.updateAppFirmwareFilename.length() != 0 && g_commandLineOptions.comPort.length() == 0)
	{
		cout << "Use COM_PORT option \"-c \" with bootloader" << endl;
		return false;
	}
    else if (g_commandLineOptions.updateBootloaderFilename.length() != 0 && g_commandLineOptions.comPort.length() == 0)
    {
        cout << "Use COM_PORT option \"-c \" with bootloader" << endl;
        return false;
    }
        
	return true;
}

bool cltool_replayDataLog()
{
	if (g_commandLineOptions.logPath.length() == 0)
	{
		cout << "Please specify the replay log path!" << endl;
		return false;
	}

	cISLogger logger;
	if (!logger.LoadFromDirectory(g_commandLineOptions.logPath, cISLogger::ParseLogType(g_commandLineOptions.logType), { "ALL" }))
	{
		cout << "Failed to load log files: " << g_commandLineOptions.logPath << endl;
		return false;
	}

	cout << "Replaying log files: " << g_commandLineOptions.logPath << endl;
	p_data_t *data;
	while ((data = logger.ReadData()) != NULL)
	{
		g_inertialSenseDisplay.ProcessData(data, g_commandLineOptions.replayDataLog, g_commandLineOptions.replaySpeed);

// 		if (data->hdr.id == DID_GPS1_RAW)
// 		{
// 			// Insert your code for processing data here
// 		}
	}

	cout << "Done replaying log files: " << g_commandLineOptions.logPath << endl;
	g_inertialSenseDisplay.Goodbye();
	return true;
}

void cltool_outputUsage()
{
	cout << boldOff;
	cout << "-----------------------------------------------------------------" << endlbOn;
	cout << "CLTool - " << boldOff << cltool_version() << endl;
	cout << endl;
	cout << "    Command line utility for communicating, logging, and updating firmware with Inertial Sense product line." << endl;
	cout << endlbOn;
	cout << "EXAMPLES" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -did DID_INS_1 DID_GPS1_POS DID_PIMU " << EXAMPLE_SPACE_1 << boldOff << " # stream DID messages" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -did 4 13 3           " << EXAMPLE_SPACE_1 << boldOff << " # stream same as line above" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -did 3=5              " << EXAMPLE_SPACE_1 << boldOff << " # stream DID_PIMU at startupNavDtMs x 5" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -presetPPD            " << EXAMPLE_SPACE_1 << boldOff << " # stream post processing data (PPD) with INS2" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -presetPPD -lon -lts=1" << EXAMPLE_SPACE_1 << boldOff << " # stream PPD + INS2 data, logging, dir timestamp" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -edit DID_FLASH_CFG   " << EXAMPLE_SPACE_1 << boldOff << " # edit DID_FLASH_CONFIG message" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -baud=115200 -did 5 13=10 " << boldOff << " # stream at 115200 bps, GPS streamed at 10x startupGPSDtMs" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -rover=RTCM3:192.168.1.100:7777:mount:user:password" << boldOff << "    # Connect to RTK NTRIP base" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -rp " <<     EXAMPLE_LOG_DIR                                              << boldOff << " # replay log files from a folder" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c "  <<     EXAMPLE_PORT << " -uf " << EXAMPLE_FIRMWARE_FILE << " -ub " << EXAMPLE_BOOTLOADER_FILE << " -uv" << boldOff << endlbOff;
	cout << "                                                   " << boldOff << " # update application firmware and bootloader" << endlbOff;
	cout << "    " << APP_NAME << APP_EXT << " -c * -baud=921600              "                    << EXAMPLE_SPACE_2 << boldOff << " # 921600 bps baudrate on all serial ports" << endlbOff;
	cout << endlbOn;
	cout << "OPTIONS (General)" << endl;
	cout << "    -h --help" << boldOff << "       Display this help menu." << endlbOn;
	cout << "    -c " << boldOff << "COM_PORT     Select the serial port. Set COM_PORT to \"*\" for all ports and \"*4\" to use" << endlbOn;
	cout << "       " << boldOff << "             only the first four ports. " <<  endlbOn;
	cout << "    -baud=" << boldOff << "BAUDRATE  Set serial port baudrate.  Options: " << IS_BAUDRATE_115200 << ", " << IS_BAUDRATE_230400 << ", " << IS_BAUDRATE_460800 << ", " << IS_BAUDRATE_921600 << " (default)" << endlbOn;
	cout << "    -magRecal[n]" << boldOff << "    Recalibrate magnetometers: 0=multi-axis, 1=single-axis" << endlbOn;
	cout << "    -q" << boldOff << "              Quiet mode, no display." << endlbOn;
	cout << "    -reset         " << boldOff << " Issue software reset." << endlbOn;
	cout << "    -resetEvb      " << boldOff << " Issue software reset on EVB." << endlbOn;
	cout << "    -s" << boldOff << "              Scroll displayed messages to show history." << endlbOn;
	cout << "    -stats" << boldOff << "          Display statistics of data received." << endlbOn;
	cout << "    -survey=[s],[d]" << boldOff << " Survey-in and store base position to refLla: s=[" << SURVEY_IN_STATE_START_3D << "=3D, " << SURVEY_IN_STATE_START_FLOAT << "=float, " << SURVEY_IN_STATE_START_FIX << "=fix], d=durationSec" << endlbOn;
	cout << "    -uf " << boldOff << "FILEPATH    Update application firmware using .hex file FILEPATH.  Add -baud=115200 for systems w/ baud rate limits." << endlbOn;
	cout << "    -ub " << boldOff << "FILEPATH    Update bootloader using .bin file FILEPATH if version is old. Must be used along with option -uf." << endlbOn;
	cout << "    -fb " << boldOff << "            Force bootloader update regardless of the version." << endlbOn;
	cout << "    -uv " << boldOff << "            Run verification after application firmware update." << endlbOn;
	cout << "    -sysCmd=[c]" << boldOff << "     Send DID_SYS_CMD c (see eSystemCommand) preceeded by unlock command then exit the program." << endlbOn;
	cout << "    -factoryReset " << boldOff << "  Reset IMX flash config to factory defaults." << endlbOn;
	cout << "    -romBootloader " << boldOff << " Reboot into ROM bootloader mode.  Requires power cycle and reloading bootloader and firmware." << endlbOn;
	if (g_internal)
	{
	cout << "    -chipEraseIMX " << boldOff << "  CAUTION!!! Erase everything on IMX (firmware, config, calibration, etc.)" << endlbOn;
	cout << "    -chipEraseEvb2 " << boldOff << " CAUTION!!! Erase everything on EVB2 (firmware, config, etc.)" << endlbOn;
	cout << "    -platform=[t]" << boldOff << "   CAUTION!!! Sets the manufacturing platform type in OTP memory (only get 15 writes)." << endlbOn;
	}
	cout << "    -v" << boldOff << "              Print version information." << endlbOn;

	cout << endlbOn;
	cout << "OPTIONS (Message Streaming)" << endl;
	cout << "    -did [DID#<=PERIODMULT> DID#<=PERIODMULT> ...]" << boldOff << "  Stream 1 or more datasets and display w/ compact view." << endlbOn;
	cout << "    -edit [DID#<=PERIODMULT>]                     " << boldOff << "  Stream and edit 1 dataset." << endlbOff;
	cout << "          Each DID# can be the DID number or name and appended with <=PERIODMULT> to decrease message frequency. " << endlbOff;
	cout << "          Message period = source period x PERIODMULT. PERIODMULT is 1 if not specified." << endlbOff;
	cout << "          Common DIDs: DID_INS_1, DID_INS_2, DID_INS_4, DID_PIMU, DID_IMU, DID_GPS1_POS," << endlbOff;
	cout << "          DID_GPS2_RTK_CMP_REL, DID_BAROMETER, DID_MAGNETOMETER, DID_FLASH_CONFIG (see data_sets.h for complete list)" << endlbOn;
	cout << "    -dids          " << boldOff << " Print list of all DID datasets" << endlbOn;
	cout << "    -persistent    " << boldOff << " Save current streams as persistent messages enabled on startup" << endlbOn;
	cout << "    -presetPPD     " << boldOff << " Stream preset post processing datasets (PPD)" << endlbOn;
	cout << "    -presetINS2    " << boldOff << " Stream preset INS2 datasets" << endlbOn;
	cout << endlbOn;
	cout << "OPTIONS (Logging to file, disabled by default)" << endl;
	cout << "    -lon" << boldOff << "            Enable logging" << endlbOn;
	cout << "    -lt=" << boldOff << "TYPE        Log type: dat (default), sdat, kml or csv" << endlbOn;
	cout << "    -lp " << boldOff << "PATH        Log data to path (default: ./" << CL_DEFAULT_LOGS_DIRECTORY << ")" << endlbOn;
	cout << "    -lms=" << boldOff << "PERCENT    Log max space in percent of free space (default: " << CL_DEFAULT_MAX_LOG_SPACE_PERCENT << ")" << endlbOn;
	cout << "    -lmf=" << boldOff << "BYTES      Log max file size in bytes (default: " << CL_DEFAULT_MAX_LOG_FILE_SIZE << ")" << endlbOn;
	cout << "    -lts=" << boldOff << "0          Log sub folder, 0 or blank for none, 1 for timestamp, else use as is" << endlbOn;
	cout << "    -r" << boldOff << "              Replay data log from default path" << endlbOn;
	cout << "    -rp " << boldOff << "PATH        Replay data log from PATH" << endlbOn;
	cout << "    -rs=" << boldOff << "SPEED       Replay data log at x SPEED. SPEED=0 runs as fast as possible." << endlbOn;
	cout << endlbOn;
	cout << "OPTIONS (Read or write flash configuration from command line)" << endl;
	cout << "    -flashCfg" << boldOff << "       List all IMX \"keys\" and \"values\"" << endlbOn;
	cout << "   \"-flashCfg=[key]=[value]|[key]=[value]\" " << boldOff <<  endlbOn;
	cout << "    -evbFlashCfg" << boldOff << "    List all EVB \"keys\" and \"values\"" << endlbOn;
	cout << "   \"-evbFlashCfg=[key]=[value]|[key]=[value]\" " << boldOff <<  endlbOn;
	cout << "        " << boldOff << "            Set key / value pairs in flash config. Surround with \"quotes\" when using pipe operator." << endlbOn;
	cout << "EXAMPLES" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c " << EXAMPLE_PORT << " -flashCfg  " << boldOff << "# Read from device and print all keys and values" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c " << EXAMPLE_PORT << " -flashCfg=insRotation[0]=1.5708|insOffset[1]=1.2  " << boldOff << endlbOn;
	cout << "     " << boldOff << "                             # Set multiple flashCfg values" << endlbOn;
	cout << "OPTIONS (RTK Rover / Base)" << endlbOn;
	cout << "    -rover=" << boldOff << "[type]:[IP or URL]:[port]:[mountpoint]:[username]:[password]" << endl;
	cout << "        As a rover (client), receive RTK corrections.  Examples:" << endl;
	cout << "            -rover=TCP:RTCM3:192.168.1.100:7777:mountpoint:username:password   (NTRIP)" << endl;
	cout << "            -rover=TCP:RTCM3:192.168.1.100:7777" << endl;
	cout << "            -rover=TCP:UBLOX:192.168.1.100:7777" << endl;
	cout << "            -rover=SERIAL:RTCM3:" << EXAMPLE_PORT << ":57600             (port, baud rate)" << endlbOn;
	cout << "    -base=" << boldOff << "[IP]:[port]   As a Base (sever), send RTK corrections.  Examples:" << endl;
	cout << "            -base=TCP::7777                            (IP is optional)" << endl;
	cout << "            -base=TCP:192.168.1.43:7777" << endl;
	cout << "            -base=SERIAL:" << EXAMPLE_PORT << ":921600" << endl;
	cout << endlbOn;	
	cout << "CLTool - " << boldOff << cltool_version() << endl;

	cout << boldOff;   // Last line.  Leave bold text off on exit.
}

void cltool_outputHelp()
{
	cout << endlbOff << "Run \"" << boldOn << "cltool -h" << boldOff << "\" to display the help menu." << endl;
}

bool cltool_updateFlashCfg(InertialSense& inertialSenseInterface, string flashCfgString)
{
	nvm_flash_cfg_t flashCfg;
	inertialSenseInterface.GetFlashConfig(flashCfg);
	const map_name_to_info_t& flashMap = *cISDataMappings::GetMapInfo(DID_FLASH_CONFIG);

	if (flashCfgString.length() < 2)
	{
		// read flash config and display
		data_mapping_string_t stringBuffer;
		cout << "Current flash config" << endl;
		for (map_name_to_info_t::const_iterator i = flashMap.begin(); i != flashMap.end(); i++)
		{
			if (cISDataMappings::DataToString(i->second, NULL, (const uint8_t*)&flashCfg, stringBuffer))
			{
				cout << i->second.name << " = " << stringBuffer << endl;
			}
		}
	}
	else
	{
		vector<string> keyValues;
		splitString(flashCfgString, '|', keyValues);
		for (size_t i = 0; i < keyValues.size(); i++)
		{
			vector<string> keyAndValue;
			splitString(keyValues[i], '=', keyAndValue);
			if (keyAndValue.size() == 2)
			{
				if (flashMap.find(keyAndValue[0]) == flashMap.end())
				{
					cout << "Unrecognized flash config key '" << keyAndValue[0] << "' specified, ignoring." << endl;
				}
				else
				{
					const data_info_t& info = flashMap.at(keyAndValue[0]);
					int radix = (keyAndValue[1].compare(0, 2, "0x") == 0 ? 16 : 10);
					int substrIndex = 2 * (radix == 16); // skip 0x for hex
					const string& str = keyAndValue[1].substr(substrIndex);
					cISDataMappings::StringToData(str.c_str(), (int)str.length(), NULL, (uint8_t*)&flashCfg, info, radix);
					cout << "Updated flash config key '" << keyAndValue[0] << "' to '" << keyAndValue[1].c_str() << "'" << endl;
				}
			}
		}
		inertialSenseInterface.SetFlashConfig(flashCfg);
	}

    return false;
}

bool cltool_updateEvbFlashCfg(InertialSense& inertialSenseInterface, string flashCfgString)
{
	evb_flash_cfg_t evbFlashCfg;
	inertialSenseInterface.GetEvbFlashConfig(evbFlashCfg);
	const map_name_to_info_t& flashMap = *cISDataMappings::GetMapInfo(DID_EVB_FLASH_CFG);

	if (flashCfgString.length() < 2)
	{
		// read flash config and display
		data_mapping_string_t stringBuffer;
		cout << "Current EVB flash config" << endl;
		for (map_name_to_info_t::const_iterator i = flashMap.begin(); i != flashMap.end(); i++)
		{
			if (cISDataMappings::DataToString(i->second, NULL, (const uint8_t*)&evbFlashCfg, stringBuffer))
			{
				cout << i->second.name << " = " << stringBuffer << endl;
			}
		}
	}
	else
	{
		vector<string> keyValues;
		splitString(flashCfgString, '|', keyValues);
		for (size_t i = 0; i < keyValues.size(); i++)
		{
			vector<string> keyAndValue;
			splitString(keyValues[i], '=', keyAndValue);
			if (keyAndValue.size() == 2)
			{
				if (flashMap.find(keyAndValue[0]) == flashMap.end())
				{
					cout << "Unrecognized EVB flash config key '" << keyAndValue[0] << "' specified, ignoring." << endl;
				}
				else
				{
					const data_info_t& info = flashMap.at(keyAndValue[0]);
					int radix = (keyAndValue[1].compare(0, 2, "0x") == 0 ? 16 : 10);
					int substrIndex = 2 * (radix == 16); // skip 0x for hex
					const string& str = keyAndValue[1].substr(substrIndex);
					cISDataMappings::StringToData(str.c_str(), (int)str.length(), NULL, (uint8_t*)&evbFlashCfg, info, radix);
					cout << "Updated EVB flash config key '" << keyAndValue[0] << "' to '" << keyAndValue[1].c_str() << "'" << endl;
				}
			}
		}
		inertialSenseInterface.SetEvbFlashConfig(evbFlashCfg);
	}

    return false;
}
