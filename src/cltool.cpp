/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "cltool.h"
#include "ISDataMappings.h"

cmd_options_t g_commandLineOptions;
serial_port_t g_serialPort;
cInertialSenseDisplay g_inertialSenseDisplay;

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
		g_commandLineOptions.logSolution, // solution logging options
		g_commandLineOptions.maxLogSpacePercent, // max space in percentage of free space to use, 0 for unlimited
		g_commandLineOptions.maxLogFileSize, // each log file will be no larger than this in bytes
		g_commandLineOptions.maxLogMemory, // logger will try and keep under this amount of memory
		g_commandLineOptions.logSubFolder // log sub folder name
	);

	// Call these elsewhere as needed
// 	inertialSenseInterface.EnableLogger(false);	// Enable/disable during runtime
// 	inertialSenseInterface.CloseLogger();		// Stop logging and save remaining data to file
}

template<typename Out>
void splitStringIterator(const string &s, char delim, Out result)
{
	stringstream ss;
	ss.str(s);
	string item;
	while (getline(ss, item, delim))
	{
		*(result++) = item;
	}
}


vector<string> splitString(const string &s, char delim)
{
	vector<string> elems;
	splitStringIterator(s, delim, back_inserter(elems));
	return elems;
}

static bool startsWith(const char* str, const char* pre)
{
	size_t lenpre = strlen(pre), lenstr = strlen(str);
	return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}

bool cltool_parseCommandLine(int argc, char* argv[])
{
	// set defaults
	g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_PRETTY;
	g_commandLineOptions.logPath = DEFAULT_LOGS_DIRECTORY;
	g_commandLineOptions.logSolution = SLOG_W_INS2;
	g_commandLineOptions.maxLogFileSize = 1024 * 1024 * 5;
	g_commandLineOptions.maxLogSpacePercent = 0.5f;
	g_commandLineOptions.maxLogMemory = 131072;
	g_commandLineOptions.replaySpeed = 1.0;
	g_commandLineOptions.enableLogging = true;
	g_commandLineOptions.baudRate = IS_COM_BAUDRATE_DEFAULT;
	g_commandLineOptions.bootloaderVerify = true;

	// parse command line
	for (int i = 1; i < argc; i++)
	{
		const char* a = argv[i];
		if (!strncmp(a, "-h", 16))
		{
			cltool_outputUsage();
			return false;
		}
		else if (startsWith(a, "-baud="))
		{
			g_commandLineOptions.baudRate = strtol(&a[6], NULL, 10);
		}
		else if (startsWith(a, "-b="))
		{
			g_commandLineOptions.bootloaderFileName = &a[3];
		}
		else if (startsWith(a, "-bv="))
		{
			g_commandLineOptions.bootloaderVerify = a[4] == '1';
		}
		else if (startsWith(a, "-c="))
		{
			g_commandLineOptions.comPort = &a[3];
		}
		else if (startsWith(a, "-lms="))
		{
			g_commandLineOptions.maxLogSpacePercent = (float)atof(&a[5]);
		}
		else if (startsWith(a, "-lmf="))
		{
			g_commandLineOptions.maxLogFileSize = (uint32_t)strtoul(&a[5], NULL, 10);
		}
		else if (startsWith(a, "-lmm="))
		{
			g_commandLineOptions.maxLogMemory = (uint32_t)strtoul(&a[5], NULL, 10);
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
		else if (startsWith(a, "-lp="))
		{
			g_commandLineOptions.logPath = &a[4];
		}
		else if (!strncmp(a, "-loff", 5))
		{
			g_commandLineOptions.enableLogging = false;
		}
		else if (!strncmp(a, "-q", 2))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_QUIET;
		}
		else if (startsWith(a, "-rp="))
		{
			g_commandLineOptions.replayDataLog = true;
			g_commandLineOptions.logPath = &a[4];
		}
		else if (startsWith(a, "-rs="))
		{
			g_commandLineOptions.replayDataLog = true;
			g_commandLineOptions.replaySpeed = (float)atof(&a[4]);
		}
		else if (!strncmp(a, "-r", 2))
		{
			g_commandLineOptions.replayDataLog = true;
		}
		else if (!strncmp(a, "-sINS1", 6))
		{
			g_commandLineOptions.streamINS1 = true;
		}
		else if (!strncmp(a, "-sINS2", 6))
		{
			g_commandLineOptions.streamINS2 = true;
		}
		else if (!strncmp(a, "-sDualIMU", 9))
		{
			g_commandLineOptions.streamDualIMU = true;
		}
		else if (!strncmp(a, "-sIMU1", 6))
		{
			g_commandLineOptions.streamIMU1 = true;
		}
		else if (!strncmp(a, "-sIMU2", 6))
		{
			g_commandLineOptions.streamIMU2 = true;
		}
		else if (!strncmp(a, "-sGPS", 5))
		{
			g_commandLineOptions.streamGPS = true;
		}
		else if (!strncmp(a, "-sMag1", 6))
		{
			g_commandLineOptions.streamMag1 = true;
		}
		else if (!strncmp(a, "-sBaro", 6))
		{
			g_commandLineOptions.streamBaro = true;
		}
		else if (!strncmp(a, "-sSol=", 6))
		{
			g_commandLineOptions.logSolution = strtol(a + 6, NULL, 10);
		}
		else if (!strncmp(a, "-sSensors", 9))
		{
			g_commandLineOptions.streamSysSensors = true;
		}
		else if (!strncmp(a, "-sDThetaVel", 11))
		{
			g_commandLineOptions.streamDThetaVel = true;
		}
		else if (!strncmp(a, "-scroll", 7))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_SCROLL;
		}
		else if (!strncmp(a, "-stats", 6))
		{
			g_commandLineOptions.displayMode = cInertialSenseDisplay::DMODE_STATS;
		}
		else if (startsWith(a, "-svr="))
		{
			g_commandLineOptions.serverConnection = &a[5];
		}
		else if (startsWith(a, "-host="))
		{
			g_commandLineOptions.host = &a[6];
		}
		else if (startsWith(a, "-flashConfig="))
		{
			g_commandLineOptions.flashConfig = &a[13];
		}
		else
		{
			cout << "Unrecognized command line option: " << a << endl;
			cltool_outputUsage();
			return false;
		}
	}

	// We are either using a serial port or replaying data
	if ((g_commandLineOptions.comPort.length() == 0) && !g_commandLineOptions.replayDataLog)
	{
		cltool_outputUsage();
		return false;
	}

	if (g_commandLineOptions.bootloaderFileName.length() && g_commandLineOptions.comPort.length() == 0)
		cout << "Use COM_PORT option \"-c=\" with bootloader" << endl;

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
	if (!logger.LoadFromDirectory(g_commandLineOptions.logPath))
	{
		cout << "Failed to load log files: " << g_commandLineOptions.logPath << endl;
		return false;
	}

	cout << "Replaying log files: " << g_commandLineOptions.logPath << endl;
	p_data_t *data;
	while ((data = logger.ReadData()) != NULL)
	{
		g_inertialSenseDisplay.ProcessData(data, g_commandLineOptions.replayDataLog, g_commandLineOptions.replaySpeed);
	}

	cout << "Done replaying log files: " << g_commandLineOptions.logPath << endl;
	g_inertialSenseDisplay.Goodbye();
	return true;
}

void cltool_outputUsage()
{
	cout << boldOff;
	cout << "-----------------------------------------------------------------" << endlbOn;
	cout << "                    $ Inertial Sense CL Tool" << endl;
	cout << boldOn << "USAGE" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << boldOff << " [OPTION]" << endlbOn;
	cout << endlbOn;
	cout << "EXAMPLE USAGE" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" <<     EXAMPLE_PORT << " -sINS1             " << EXAMPLE_SPACE_1 << boldOff << " # stream one data set" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" <<     EXAMPLE_PORT << " -sINS2 -sGPS -sBaro" << EXAMPLE_SPACE_1 << boldOff << " # stream multiple sets" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" <<     EXAMPLE_PORT << " -sINS2 -l -lts=0   " << EXAMPLE_SPACE_1 << boldOff << " # stream and log data" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -rp=" << EXAMPLE_LOG_DIR                                              << boldOff << " # replay data file" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" <<     EXAMPLE_PORT << " -b=" << EXAMPLE_FIRMWARE_FILE           << boldOff << " # bootload firmware" << endlbOn;
	cout << "    " << APP_NAME << APP_EXT << " -c=" << EXAMPLE_PORT << " -baud=921600           " << EXAMPLE_SPACE_1 << boldOff << " # set the serial port baudrate" << endlbOn;
	cout << endlbOn;
	cout << "DESCRIPTION" << endlbOff;
	cout << "    Command line utility for communicating, logging, and updating" << endl;
	cout << "    firmware with Inertial Sense product line." << endl;
	cout << endlbOn;
	cout << "OPTIONS" << endlbOn;
	cout << "    -c=" << boldOff << "COM_PORT    select the serial port" << endlbOn;
	cout << endlbOn;
	cout << "    -b=" << boldOff << "FILEPATH    bootload firmware using .hex file FILEPATH" << endlbOn;
	cout << "    -baud=" << boldOff << "BAUDRATE set the serial port baudrate" << endlbOn;
	cout << endlbOn;
	cout << "    -h" << boldOff << "             display this help menu" << endlbOn;
	cout << "    -q" << boldOff << "             quite mode, no display" << endlbOn;
	cout << "    -scroll" << boldOff << "        scroll displayed messages to show history" << endlbOn;
	cout << endlbOn;
	cout << "    -sINS1" << boldOff << "         stream message DID_INS_1" << endlbOn;
	cout << "    -sINS2" << boldOff << "         stream message DID_INS_2" << endlbOn;
	cout << "    -sDualIMU" << boldOff << "      stream message DID_DUAL_IMU" << endlbOn;
	cout << "    -sIMU1" << boldOff << "         stream message DID_IMU_1" << endlbOn;
	cout << "    -sIMU2" << boldOff << "         stream message DID_IMU_2" << endlbOn;
	cout << "    -sGPS " << boldOff << "         stream message DID_GPS" << endlbOn;
	cout << "    -sMag1" << boldOff << "         stream message DID_MAGNETOMETER_1" << endlbOn;
	cout << "    -sBaro" << boldOff << "         stream message DID_BAROMETER" << endlbOn;
	cout << "    -sSol= " << boldOff << "        stream solution messages (IMU, GPS, INS2, etc.)" << endlbOn;
	cout << "    -sSensors" << boldOff << "      stream message DID_SYS_SENSORS" << endlbOn;
	cout << "    -sDThetaVel" << boldOff << "    stream message DID_DELTA_THETA_VEL" << endlbOn;

	cout << endl << "    Logging is enabled by default." << endlbOn;
	cout << "    -loff" << boldOff << "          disable logging" << endlbOn;
	cout << "    -lp=" << boldOff << "PATH       log data to path" << endlbOn;
	cout << "    -r" << boldOff << "             replay data log from default path" << endlbOn;
	cout << "    -rp=" << boldOff << "PATH       replay data log from PATH" << endlbOn;
	cout << "    -rs=" << boldOff << "SPEED      replay data log at x SPEED" << endlbOn;
	cout << "    -lms=" << boldOff << "PERCENT   log max space in percent of free space (default: 0.5)" << endlbOn;
	cout << "    -lmf=" << boldOff << "BYTES     log max file size in bytes (default: 5242880)" << endlbOn;
	cout << "    -lmm=" << boldOff << "BYTES     log max memory in bytes (default: 131072)" << endlbOn;
	cout << "    -lts=" << boldOff << "0         log sub folder, 0 or blank for none, 1 for timestamp, else use as is" << endlbOn;

	cout << endl << "Flash config. Read or write flash configuration." << endl;
	cout << "    -flashConfig=." << boldOff << " - read flash config and display." << endlbOn;
	cout << "    -flashConfig=key=value|key=value " << boldOff << " - set key / value pairs in flash config." << endlbOn;

	cout << endl << "Server connection. Use the uINS as RTK rover." << endl;
	cout << "    -svr=" << boldOff << "connection info, used to retreive external data and send to the uINS. Examples:" << endl;
	cout << "    For retrieving RTCM3 data: RTCM3:192.168.1.100:7777:url:user:password - url, user and password are optional." << endl;
	cout << "    For retrieving InertialSense data: IS:192.168.1.100:7777 - no url, user or password for InertialSense data." << endlbOn;
	cout << "    For retrieving UBLOX data: UBLOX:192.168.1.100:7777 - no url, user or password." << endl;

	cout << endl << "Server host. Use the uINS as RTK base station." << endl;
	cout << "    -host=" << boldOff << "ipAndPort, host an InertialSense server, i.e. :7777 or 192.168.1.43:7777. The ip address part is optional." << endlbOn;

	cout << boldOff;   // Last line.  Leave bold text off on exit.
}

bool cltool_updateFlashConfig(InertialSense& inertialSenseInterface, string flashConfigString)
{
	const nvm_flash_cfg_t& flashConfig = inertialSenseInterface.GetFlashConfig();
	const map_lookup_name_t& globalMap = cISDataMappings::GetMap();
	const map_name_to_info_t& flashMap = globalMap.at(DID_FLASH_CONFIG);

	if (flashConfigString.length() < 2)
	{
		// read flash config and display
		data_mapping_string_t stringBuffer;
		cout << "Current flash config" << endl;
		for (map_name_to_info_t::const_iterator i = flashMap.begin(); i != flashMap.end(); i++)
		{
			if (cISDataMappings::DataToString(i->second, NULL, (const uint8_t*)&flashConfig, stringBuffer))
			{
				cout << i->second.name << " = " << stringBuffer << endl;
			}
		}
		return false;
	}
	else
	{
		nvm_flash_cfg_t flashConfig = inertialSenseInterface.GetFlashConfig();
		vector<string> keyValues = splitString(flashConfigString, '|');
		for (size_t i = 0; i < keyValues.size(); i++)
		{
			vector<string> keyAndValue = splitString(keyValues[i], '=');
			if (keyAndValue.size() == 2)
			{
				if (flashMap.find(keyAndValue[0]) == flashMap.end())
				{
					cout << "Unrecognized flash config key '" << keyAndValue[0] << "' specified, ignoring." << endl;
				}
				else
				{
					const data_info_t& info = flashMap.at(keyAndValue[0]);
					cISDataMappings::StringToData(keyAndValue[1].c_str(), keyAndValue[1].length(), NULL, (uint8_t*)&flashConfig, info);
					cout << "Updated flash config key '" << keyAndValue[0] << "' to '" << keyAndValue[1].c_str() << "'" << endl;
				}
			}
		}
		inertialSenseInterface.SetFlashConfig(flashConfig);
		SLEEP_MS(1000);
		g_inertialSenseDisplay.Clear();
		return true;
	}
}
