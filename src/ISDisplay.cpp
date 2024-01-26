/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <math.h>

#include "ISConstants.h"
#include "ISUtilities.h"
#include "ISDisplay.h"
#include "ISPose.h"
#include "ISEarth.h"

#if PLATFORM_IS_WINDOWS

#include <conio.h>

#endif

#if PLATFORM_IS_LINUX || PLATFORM_IS_APPLE

#include <unistd.h>
#include <sys/time.h>
#include <termios.h>

#endif

using namespace std;

#define PRINTV3_P1		"%8.1f,%8.1f,%8.1f"
#define PRINTV3_P2		" %8.2f,%8.2f,%8.2f"
#define PRINTV3_P3		"  %8.3f,%8.3f,%8.3f"
#define PRINTV4_P1		"%8.1f,%8.1f,%8.1f,%8.1f"
#define PRINTV4_P2		" %8.2f,%8.2f,%8.2f,%8.2f"
#define PRINTV4_P3		"  %8.3f,%8.3f,%8.3f,%8.3f"
#define PRINTV3_LLA		"%13.7f,%13.7f,%7.1f ellipsoid"
#define PRINTV3_LLA_MSL	"%13.7f,%13.7f,%7.1f MSL"
#define BUF_SIZE 8192

#define DATASET_VIEW_NUM_ROWS 25

#define DISPLAY_DELTA_TIME	0	// show delta time instead of time

static bool s_exitProgram;

#if PLATFORM_IS_WINDOWS

static bool ctrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
	case CTRL_BREAK_EVENT:
	case CTRL_LOGOFF_EVENT:
	case CTRL_SHUTDOWN_EVENT:
		s_exitProgram = true;
		return true;
	default:
		return false;
	}
}

#else

#include <signal.h>

static void signalFunction(int sig)
{
    (void)sig;
	s_exitProgram = true;
}

#endif


cInertialSenseDisplay::cInertialSenseDisplay(eDisplayMode displayMode)
{
	m_displayMode = displayMode;

#if PLATFORM_IS_WINDOWS

	// Hide cursor
	ShowCursor(false);

	if (!SetConsoleCtrlHandler((PHANDLER_ROUTINE)ctrlHandler, true))
	{
		std::cout << "Failed to set console ctrl handler!" << std::endl;
	}

#else

	signal(SIGINT, signalFunction);

#endif

}

cInertialSenseDisplay::~cInertialSenseDisplay()
{

#if !PLATFORM_IS_WINDOWS

	if (m_nonblockingkeyboard)
	{	// Revert terminal changes from KeyboardNonBlock();
		ResetTerminalMode();
	}

#endif

}


void cInertialSenseDisplay::ShowCursor(bool visible)
{

#if PLATFORM_IS_WINDOWS

// 	m_windowsConsoleIn = GetStdHandle(STD_INPUT_HANDLE);
	m_windowsConsoleOut = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_CURSOR_INFO cursorInfo;
	GetConsoleCursorInfo(m_windowsConsoleOut, &cursorInfo);
	cursorInfo.bVisible = visible;
	SetConsoleCursorInfo(m_windowsConsoleOut, &cursorInfo);

#endif

}


void cInertialSenseDisplay::ShutDown()
{
	ShowCursor(true);
// 	cout << "Shutting down..." << endl;
}


void cInertialSenseDisplay::Clear(void)
{
	if (!m_interactiveMode)
	{
		return;
	}

#if PLATFORM_IS_WINDOWS

	COORD topLeft = { 0, 0 };
	HANDLE console = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_SCREEN_BUFFER_INFO screen;
	DWORD written;
	GetConsoleScreenBufferInfo(console, &screen);
	FillConsoleOutputCharacterA(console, ' ', screen.dwSize.X * screen.dwSize.Y, topLeft, &written);
	FillConsoleOutputAttribute(console, FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_BLUE, screen.dwSize.X * screen.dwSize.Y, topLeft, &written);
	SetConsoleCursorPosition(console, topLeft);

#else

	printf( "\x1B[2J" ); // VT100 terminal command

#endif
}

void cInertialSenseDisplay::Home(void)
{
	if (!m_interactiveMode)
	{
		return;
	}

#if PLATFORM_IS_WINDOWS

	COORD topLeft = { 0, 0 };
	SetConsoleCursorPosition(m_windowsConsoleOut, topLeft);

#else

	printf( "\x1B[H" ); // VT100 terminal command

#endif

}

void cInertialSenseDisplay::GoToRow(int y)
{

#if PLATFORM_IS_WINDOWS

	COORD pos = { 0, (int16_t)y };
	SetConsoleCursorPosition(m_windowsConsoleOut, pos);

#else

	printf("\x1B[%dH", y); // VT100 terminal command

#endif

}

void cInertialSenseDisplay::GoToColumnAndRow(int x, int y)
{

#if PLATFORM_IS_WINDOWS

	COORD pos = { (int16_t)x, (int16_t)y };
	SetConsoleCursorPosition(m_windowsConsoleOut, pos);

#else

	printf("\x1B[%d;%df", y, x); // VT100 terminal command

#endif

}

string cInertialSenseDisplay::Hello()
{
	return "$ Inertial Sense.  Press CTRL-C to terminate.\n";
}

string cInertialSenseDisplay::Connected()
{
	return string("$ Inertial Sense.  Connected.  Press CTRL-C to terminate.  Rx ") + std::to_string(m_rxCount) + "\n";
}

string cInertialSenseDisplay::Replay(double speed)
{
	char buf[BUF_SIZE];

	SNPRINTF(buf, BUF_SIZE, "$ Inertial Sense.  Replay mode at %.1lfx speed.  Press CTRL-C to terminate.\n", speed);

	return buf;
}

string cInertialSenseDisplay::Goodbye()
{
	return "\nThanks for using Inertial Sense!\n";
}



void cInertialSenseDisplay::SetKeyboardNonBlocking()
{
	m_nonblockingkeyboard = true;

#if !PLATFORM_IS_WINDOWS

    struct termios new_settings;
    tcgetattr(0, &orig_termios_);
    new_settings = orig_termios_;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    // new_settings.c_lflag &= ~ISIG;	// disable ctrl-c signal
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);

#endif

}


void cInertialSenseDisplay::ResetTerminalMode()
{

#if !PLATFORM_IS_WINDOWS

    tcsetattr(0, TCSANOW, &orig_termios_);

#endif

}


int cInertialSenseDisplay::KeyboardHit()
{

#if PLATFORM_IS_WINDOWS

	return _kbhit();

#else

    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);

#endif

}

int cInertialSenseDisplay::GetChar()
{

#if PLATFORM_IS_WINDOWS

	return _getch();

#else

	struct termios oldt, newt;
	tcgetattr(STDIN_FILENO, &oldt); 			/* store old settings*/
	newt = oldt; 								/* copy old settings to new settings */
	newt.c_lflag &= ~(ICANON);					/* change settings */
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);	/* apply the new settings immediatly */
	int ch = getchar(); 						/* standard getchar call */
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 	/* reapply the old settings */
	return ch;

#endif

}


bool cInertialSenseDisplay::ExitProgram()
{
	return s_exitProgram;
}


void cInertialSenseDisplay::SetExitProgram()
{
	s_exitProgram = true;
}

// Return true on refresh
void cInertialSenseDisplay::ProcessData(p_data_t* data, bool enableReplay, double replaySpeedX)
{
	if (m_displayMode == DMODE_QUIET)
	{
		return;
	}

	unsigned int curTimeMs = current_timeMs();
	m_rxCount++;

	m_enableReplay = enableReplay;
	m_replaySpeedX = replaySpeedX;

	if (enableReplay)
	{
		static bool isTowMode = false;
		static unsigned int gpsTowMsOffset = 0;
		static unsigned int msgTimeMsOffset = 0;
		unsigned int msgTimeMs = 0;

		// Copy only new data
		uDatasets d = {};
		copyDataPToStructP(&d, data, sizeof(uDatasets));

		// Record message time.  In either ToW or time since boot.
		switch (data->hdr.id)
		{
			// Time of week - double
		case DID_INS_1:
		case DID_INS_2:
			msgTimeMs = (unsigned int)(1000.0 * d.ins1.timeOfWeek);
			isTowMode = true;
			break;

			// Time of week - uint32 ms
		case DID_SYS_PARAMS:
			msgTimeMs = d.gpsPos.timeOfWeekMs;
			isTowMode = true;
			break;

		case DID_GPS1_POS:
		case DID_GPS1_RTK_POS:
			msgTimeMs = d.gpsPos.timeOfWeekMs;
			gpsTowMsOffset = (unsigned int)(1000.0 * d.gpsPos.towOffset);
			isTowMode = true;
			break;

		case DID_GPS1_RTK_POS_REL:
			msgTimeMs = d.gpsRtkRel.timeOfWeekMs;
			isTowMode = true;
			break;

		case DID_GPS1_RTK_POS_MISC:
			msgTimeMs = d.gpsPos.timeOfWeekMs;
			gpsTowMsOffset = (unsigned int)(1000.0 * d.gpsPos.towOffset);
			isTowMode = false;
			break;

			// Time since boot - double
		case DID_MAGNETOMETER:
		case DID_BAROMETER:
		case DID_SYS_SENSORS:
		case DID_PIMU:
		case DID_IMU:
		case DID_INL2_STATES:
		case DID_GPS_BASE_RAW:
			if (isTowMode)
				msgTimeMs = (unsigned int)(1000.0 * d.imu.time) + gpsTowMsOffset;
			else
				msgTimeMs = (unsigned int)(1000.0 * d.imu.time);
			break;

			// Unidentified data type
// 		default: printf("Unknown DID %d\t", data->hdr.id);	return;			
		}


		// Control replay rate
		if (msgTimeMs != 0 && replaySpeedX > 0.0)
		{
			for (;;)
			{
				curTimeMs = current_timeMs();

				// Replay speed
				unsigned int replayTimeMs = (unsigned int)(long)(((double)curTimeMs) * replaySpeedX);

				// Reinitialize message offset
				if ((msgTimeMs + msgTimeMsOffset - replayTimeMs) > 1500)
					msgTimeMsOffset = replayTimeMs - msgTimeMs;

				// Proceed if we're caught up
				if (replayTimeMs >= msgTimeMs + msgTimeMsOffset)
					break;

				// Add delay
// 				SLEEP_US(1000);
				SLEEP_MS(10);
			}
		}
	}


	static unsigned int timeSinceClearMs = 0;
	static char idHist[DID_COUNT] = { 0 };
	if (m_displayMode != DMODE_SCROLL)
	{
		// Clear display every 2 seconds or if we start seeing new messages.
		if (curTimeMs - timeSinceClearMs > 2000 || curTimeMs < timeSinceClearMs || idHist[data->hdr.id] == 0)
		{
			Clear();
			idHist[data->hdr.id] = 1;
			timeSinceClearMs = curTimeMs;
		}
	}

	// Save data to be displayed from PrintData()
	switch (m_displayMode)
	{
	default:
		m_displayMode = DMODE_PRETTY;
		// fall through
	case DMODE_PRETTY:
		// Data stays at fixed location (no scroll history)
		DataToVector(data);
		if (m_outputOnceDid == data->hdr.id)
		{	// Exit as soon as we display DID
			Home();
			cout << VectortoString();
			exit(1);
		}
		break;

	case DMODE_EDIT:
		if (m_editData.did == data->hdr.id)
		{
			m_editData.pData = *data;
		}
		break;

	case DMODE_STATS:
		DataToStats(data);
		break;

	case DMODE_SCROLL:	// Scroll display 
		cout << DataToString(data) << endl;
		break;
	}
}

// Print data to standard out at the following refresh rate.  Return true to refresh display.
bool cInertialSenseDisplay::PrintData(unsigned int refreshPeriodMs)
{
	unsigned int curTimeMs = current_timeMs();
	static unsigned int timeSinceRefreshMs = 0;

	// Limit display refresh rate
	if (curTimeMs - timeSinceRefreshMs < refreshPeriodMs)
	{
		return false;
	}
	timeSinceRefreshMs = curTimeMs;

	// Display Data
	switch (m_displayMode)
	{
	default:	// Do not display
		break;

	case DMODE_PRETTY:
		Home();
		if (m_enableReplay)
			cout << Replay(m_replaySpeedX) << endl;
		else
			cout << Connected() << endl;

		cout << VectortoString();
		return true;

	case DMODE_EDIT:
		Home();
		if (m_enableReplay)
			cout << Replay(m_replaySpeedX) << endl;
		else
			cout << Connected() << endl;

		// Generic column format
		cout << DatasetToString(&m_editData.pData);
		return true;

	case DMODE_STATS:
		Home();
		cout << Connected() << endl;
		PrintStats();
		return true;

	case DMODE_SCROLL:	// Scroll display 
		break;
	}

	return false;
}

string cInertialSenseDisplay::VectortoString()
{
	stringstream ss;

	for (size_t i = 0; i < m_didMsgs.size(); i++)
	{
		if (m_didMsgs[i].size())
		{
			ss << m_didMsgs[i];
		}
	}

	return ss.str();
}

void cInertialSenseDisplay::DataToVector(const p_data_t* data)
{
	size_t id = data->hdr.id;
	if (m_didMsgs.size() <= id)
	{	// Resize vector if necessary
		m_didMsgs.resize(id + 1);
	}
	// Add string to vector
	m_didMsgs[id] = DataToString(data);
}

void cInertialSenseDisplay::DataToStats(const p_data_t* data)
{
	size_t id = data->hdr.id;
	if (m_didStats.size() <= id)
	{	// Resize vector if necessary
		m_didStats.resize(id + 1);
	}

	// Update stats
	int curTimeMs = current_timeMs();
	sDidStats& s = m_didStats[id];
	s.count++;
	if (s.lastTimeMs)
		s.dtMs = curTimeMs - s.lastTimeMs;
	s.lastTimeMs = curTimeMs;
}

void cInertialSenseDisplay::PrintStats()
{
	// Display stats
	printf("    Count      dt  DID  Name \n");
	for (int i = 0; i < (int)m_didStats.size(); i++)
	{
		sDidStats& s = m_didStats[i];
		if (s.count)
		{
			printf("%9d %7.3lf %4d  %s\n", s.count, s.dtMs*0.001, i, cISDataMappings::GetDataSetName(i));
		}
	}
}

string cInertialSenseDisplay::DataToString(const p_data_t* data)
{
	uDatasets d = {};

	// Copy only new data
	copyDataPToStructP(&d, data, sizeof(uDatasets));

	string str;
	switch (data->hdr.id)
	{
	case DID_EVB_DEV_INFO:
	case DID_DEV_INFO:          str = DataToStringDevInfo(d.devInfo, data->hdr);        break;
	case DID_IMU:               str = DataToStringIMU(d.imu, data->hdr);                break;
	case DID_PIMU:              str = DataToStringPreintegratedImu(d.pImu, data->hdr);  break;
	case DID_INS_1:             str = DataToStringINS1(d.ins1, data->hdr);              break;
	case DID_INS_2:             str = DataToStringINS2(d.ins2, data->hdr);              break;
	case DID_INS_3:             str = DataToStringINS3(d.ins3, data->hdr);              break;
	case DID_INS_4:             str = DataToStringINS4(d.ins4, data->hdr);              break;
	case DID_BAROMETER:         str = DataToStringBarometer(d.baro, data->hdr);         break;
	case DID_MAGNETOMETER:      str = DataToStringMagnetometer(d.mag, data->hdr);       break;
	case DID_MAG_CAL:           str = DataToStringMagCal(d.magCal, data->hdr);          break;
	case DID_GPS1_POS:          str = DataToStringGpsPos(d.gpsPos, data->hdr);			break;
	case DID_GPS2_POS:          str = DataToStringGpsPos(d.gpsPos, data->hdr);			break;
	case DID_GPS1_RTK_POS:      str = DataToStringGpsPos(d.gpsPos, data->hdr);			break;
	case DID_GPS1_RTK_POS_REL:  str = DataToStringRtkRel(d.gpsRtkRel, data->hdr);		break;
	case DID_GPS1_RTK_POS_MISC: str = DataToStringRtkMisc(d.gpsRtkMisc, data->hdr);		break;
	case DID_GPS2_RTK_CMP_REL:  str = DataToStringRtkRel(d.gpsRtkRel, data->hdr);		break;
	case DID_GPS2_RTK_CMP_MISC: str = DataToStringRtkMisc(d.gpsRtkMisc, data->hdr);		break;
	case DID_GPS1_RAW:
	case DID_GPS2_RAW:
	case DID_GPS_BASE_RAW:      str = DataToStringRawGPS(d.gpsRaw, data->hdr);          break;
    case DID_SURVEY_IN:         str = DataToStringSurveyIn(d.surveyIn, data->hdr);      break;
	case DID_SYS_PARAMS:        str = DataToStringSysParams(d.sysParams, data->hdr);    break;
	case DID_SYS_SENSORS:       str = DataToStringSysSensors(d.sysSensors, data->hdr);  break;
	case DID_RTOS_INFO:         str = DataToStringRTOS(d.rtosInfo, data->hdr);          break;
	case DID_SENSORS_ADC:       str = DataToStringSensorsADC(d.sensorsAdc, data->hdr);  break;
	case DID_WHEEL_ENCODER:     str = DataToStringWheelEncoder(d.wheelEncoder, data->hdr); break;
	default:
#if 0	// List all DIDs 
		char buf[128];
		SNPRINTF(buf, sizeof(buf), "(%d) %s \n", data->hdr.id, cISDataMappings::GetDataSetName(data->hdr.id));
		str = buf;
#elif 0
		str = DataToStringGeneric(data);
#endif
		break;
	}

	return str;
}

char* cInertialSenseDisplay::StatusToString(char* ptr, char* ptrEnd, const uint32_t insStatus, const uint32_t hdwStatus)
{
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tSTATUS\n");
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t\tSatellite Rx %d     Aiding: Mag %d, GPS (Hdg %d, Pos %d)\n",
		(hdwStatus & HDW_STATUS_GPS_SATELLITE_RX) != 0,
        (insStatus & INS_STATUS_MAG_AIDING_HEADING) != 0,
        (insStatus & INS_STATUS_GPS_AIDING_HEADING) != 0,
        (insStatus & INS_STATUS_GPS_AIDING_POS) != 0);
	if (insStatus & INS_STATUS_NAV_MODE)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t\tMode: NAV ");
	}
	else
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t\tMode: AHRS");
	}
	switch (INS_STATUS_SOLUTION(insStatus))
	{
	default:
	case INS_STATUS_SOLUTION_OFF:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: OFF\n");                 break;
	case INS_STATUS_SOLUTION_ALIGNING:              ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: ALIGNING\n");            break;
	case INS_STATUS_SOLUTION_NAV:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: NAV\n");                 break;
	case INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE:     ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: NAV HIGH VARIANCE\n");   break;
    case INS_STATUS_SOLUTION_AHRS:                  ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: AHRS\n");                break;
    case INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE:    ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: AHRS HIGH VARIANCE\n");  break;
    case INS_STATUS_SOLUTION_VRS:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: VRS\n");                 break;
    case INS_STATUS_SOLUTION_VRS_HIGH_VARIANCE:     ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Solution: VRS HIGH VARIANCE\n");   break;
    }
// 	ptr += SNPRINTF(ptr, ptrEnd - ptr, "         Align Good: Att %d, Vel %d, Pos %d\n",
// 		(insStatus & INS_STATUS_ATT_ALIGN_GOOD) != 0,
// 		(insStatus & INS_STATUS_VEL_ALIGN_GOOD) != 0,
// 		(insStatus & INS_STATUS_POS_ALIGN_GOOD) != 0);
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t\tErrors    Rx parse %d, temperature %d, self-test %d\n",
		HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus),
		(hdwStatus & HDW_STATUS_ERR_TEMPERATURE) != 0,
		(hdwStatus & HDW_STATUS_BIT_FAULT) != 0);

	return ptr;
}

char* cInertialSenseDisplay::InsStatusToSolStatusString(char* ptr, char* ptrEnd, const uint32_t insStatus)
{
	switch (INS_STATUS_SOLUTION(insStatus))
	{
	default:
	case INS_STATUS_SOLUTION_OFF:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, ", OFF      ");	break;
	case INS_STATUS_SOLUTION_ALIGNING:              ptr += SNPRINTF(ptr, ptrEnd - ptr, ", ALIGNING ");	break;
	case INS_STATUS_SOLUTION_NAV:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, ", NAV      ");	break;
	case INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE:		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", NAV VARIA");	break;
    case INS_STATUS_SOLUTION_AHRS:                  ptr += SNPRINTF(ptr, ptrEnd - ptr, ", AHRS     ");	break;
    case INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE:    ptr += SNPRINTF(ptr, ptrEnd - ptr, ", AHRS VARI");	break;
    case INS_STATUS_SOLUTION_VRS:                   ptr += SNPRINTF(ptr, ptrEnd - ptr, ", VRS      ");	break;
    case INS_STATUS_SOLUTION_VRS_HIGH_VARIANCE:     ptr += SNPRINTF(ptr, ptrEnd - ptr, ", VRS VARI ");	break;
    }

	return ptr;
}

string cInertialSenseDisplay::DataToStringINS1(const ins_1_t &ins1, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(ins1.timeOfWeek - lastTime);
	lastTime = ins1.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", ins1.timeOfWeek);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr = InsStatusToSolStatusString(ptr, ptrEnd, ins1.insStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " theta[%6.2f,%6.2f,%7.2f], uvw[%6.2f,%6.2f,%6.2f], lla[%12.7f,%12.7f,%7.1f], ned[%6.3f,%6.3f,%6.3f]",
			ins1.theta[0] * C_RAD2DEG_F,
			ins1.theta[1] * C_RAD2DEG_F,
			ins1.theta[2] * C_RAD2DEG_F,
			ins1.uvw[0], ins1.uvw[1], ins1.uvw[2],
			ins1.lla[0], ins1.lla[1], ins1.lla[2],
			ins1.ned[0], ins1.ned[1], ins1.ned[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tEuler\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2 "\n",
			ins1.theta[0] * C_RAD2DEG_F,	// Roll
			ins1.theta[1] * C_RAD2DEG_F,	// Pitch
			ins1.theta[2] * C_RAD2DEG_F);	// Yaw
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tUWV\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1 "\n",
			ins1.uvw[0],					// U body velocity
			ins1.uvw[1],					// V body velocity
			ins1.uvw[2]);					// W body velocity
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tLLA\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA "\n",
			ins1.lla[0],					// INS Latitude
			ins1.lla[1],					// INS Longitude
			ins1.lla[2]);					// INS Ellipsoid altitude (meters)
		ptr = StatusToString(ptr, ptrEnd, ins1.insStatus, ins1.hdwStatus);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringINS2(const ins_2_t &ins2, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(ins2.timeOfWeek - lastTime);
	lastTime = ins2.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", ins2.timeOfWeek);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr = InsStatusToSolStatusString(ptr, ptrEnd, ins2.insStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " qn2b[%6.3f,%6.3f,%6.3f,%6.3f], uvw[%6.2f,%6.2f,%6.2f], lla[%12.7f,%12.7f,%7.1f]",
			ins2.qn2b[0], ins2.qn2b[1], ins2.qn2b[2], ins2.qn2b[3],
			ins2.uvw[0], ins2.uvw[1], ins2.uvw[2],
			ins2.lla[0], ins2.lla[1], ins2.lla[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tQn2b\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV4_P3 "\n",					// Quaternion attitude rotation
			ins2.qn2b[0],					// W
			ins2.qn2b[1],					// X
			ins2.qn2b[2],					// Y
			ins2.qn2b[3]);					// Z
		float theta[3];
		quat2euler(ins2.qn2b, theta);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t(Euler)\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2 "\n",					// Convert quaternion to euler rotation
			theta[0] * C_RAD2DEG_F,			// Roll
			theta[1] * C_RAD2DEG_F,			// Pitch
			theta[2] * C_RAD2DEG_F);		// Yaw
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tUWV\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1 "\n",
			ins2.uvw[0],					// U body velocity
			ins2.uvw[1],					// V body velocity
			ins2.uvw[2]);					// W body velocity
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tLLA\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA "\n",
			ins2.lla[0],					// INS Latitude
			ins2.lla[1],					// INS Longitude
			ins2.lla[2]);					// INS Ellipsoid altitude (meters)
		ptr = StatusToString(ptr, ptrEnd, ins2.insStatus, ins2.hdwStatus);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringINS3(const ins_3_t &ins3, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(ins3.timeOfWeek - lastTime);
	lastTime = ins3.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", ins3.timeOfWeek);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr = InsStatusToSolStatusString(ptr, ptrEnd, ins3.insStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " qn2b[%6.3f,%6.3f,%6.3f,%6.3f], uvw[%6.2f,%6.2f,%6.2f], lla[%12.7f,%12.7f,%7.1f]",
			ins3.qn2b[0], ins3.qn2b[1], ins3.qn2b[2], ins3.qn2b[3],
			ins3.uvw[0], ins3.uvw[1], ins3.uvw[2],
			ins3.lla[0], ins3.lla[1], ins3.lla[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tQn2b\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV4_P3 "\n",					// Quaternion attitude rotation
			ins3.qn2b[0],					// W
			ins3.qn2b[1],					// X
			ins3.qn2b[2],					// Y
			ins3.qn2b[3]);					// Z
		float theta[3];
		quat2euler(ins3.qn2b, theta);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t(Euler)\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2 "\n",					// Convert quaternion to euler rotation
			theta[0] * C_RAD2DEG_F,			// Roll
			theta[1] * C_RAD2DEG_F,			// Pitch
			theta[2] * C_RAD2DEG_F);		// Yaw
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tUWV\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1 "\n",
			ins3.uvw[0],					// U body velocity
			ins3.uvw[1],					// V body velocity
			ins3.uvw[2]);					// W body velocity
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tLLA\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA_MSL "\n",
			ins3.lla[0],					// INS Latitude
			ins3.lla[1],					// INS Longitude
			ins3.lla[2]);					// INS Ellipsoid altitude (meters)
		ptr = StatusToString(ptr, ptrEnd, ins3.insStatus, ins3.hdwStatus);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringINS4(const ins_4_t &ins4, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(ins4.timeOfWeek - lastTime);
	lastTime = ins4.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", ins4.timeOfWeek);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr = InsStatusToSolStatusString(ptr, ptrEnd, ins4.insStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " qe2b[%6.3f,%6.3f,%6.3f,%6.3f], ve[%6.2f,%6.2f,%6.2f], ecef[%12.7f,%12.7f,%7.1f]",
			ins4.qe2b[0], ins4.qe2b[1], ins4.qe2b[2], ins4.qe2b[3],
			ins4.ve[0], ins4.ve[1], ins4.ve[2],
			ins4.ecef[0], ins4.ecef[1], ins4.ecef[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tQe2b\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV4_P3 "\n",					// Quaternion attitude rotation
			ins4.qe2b[0],					// W
			ins4.qe2b[1],					// X
			ins4.qe2b[2],					// Y
			ins4.qe2b[3]);					// Z
		float theta[3];
		qe2b2EulerNedEcef(theta, (float*)ins4.qe2b, (double*)ins4.ecef);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t(Euler)\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2 "\n",					// Convert quaternion to euler rotation
			theta[0] * C_RAD2DEG_F,			// Roll
			theta[1] * C_RAD2DEG_F,			// Pitch
			theta[2] * C_RAD2DEG_F);		// Yaw
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tVE\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3 "\n",
			ins4.ve[0],						// X ECEF velocity
			ins4.ve[1],						// Y ECEF velocity
			ins4.ve[2]);					// Z ECEF velocity
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tECEF\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3 "\n",
			ins4.ecef[0],					// X ECEF position
			ins4.ecef[1],					// Y ECEF position
			ins4.ecef[2]);					// Z ECEF position
		ptr = StatusToString(ptr, ptrEnd, ins4.insStatus, ins4.hdwStatus);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringIMU(const imu_t &imu, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

	return string(buf) + DataToStringIMU(imu, m_displayMode != DMODE_SCROLL);
}

string cInertialSenseDisplay::DataToStringIMU(const imu_t &imu, bool full)
{
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(imu.time - lastTime);
	lastTime = imu.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", imu.time);
#endif

	if (!full)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", pqr[%5.1f,%5.1f,%5.1f]",
			imu.I.pqr[0] * C_RAD2DEG_F,
			imu.I.pqr[1] * C_RAD2DEG_F,
			imu.I.pqr[2] * C_RAD2DEG_F);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", acc[%5.1f,%5.1f,%5.1f]",
			imu.I.acc[0], imu.I.acc[1], imu.I.acc[2]);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tPQR\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1 "\n",
			imu.I.pqr[0] * C_RAD2DEG_F,		// P angular rate
			imu.I.pqr[1] * C_RAD2DEG_F,		// Q angular rate
			imu.I.pqr[2] * C_RAD2DEG_F);		// R angular rate
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tAcc\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P1 "\n",
			imu.I.acc[0],					// X acceleration
			imu.I.acc[1],					// Y acceleration
			imu.I.acc[2]);					// Z acceleration
	}

	return buf;
}


string cInertialSenseDisplay::DataToStringPreintegratedImu(const pimu_t &imu, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(imu.time - lastTime);
	lastTime = imu.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs, dt:%6.3f", imu.time, imu.dt);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", theta[%6.3f,%6.3f,%6.3f], vel[%6.3f,%6.3f,%6.3f]",
			imu.theta[0] * C_RAD2DEG_F,
			imu.theta[1] * C_RAD2DEG_F,
			imu.theta[2] * C_RAD2DEG_F,
			imu.vel[0], imu.vel[1], imu.vel[2]);
	}
	else
	{	// Spacious format
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tIMU1 theta\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3 "\n",
			imu.theta[0] * C_RAD2DEG_F,		// IMU1 P angular rate
			imu.theta[1] * C_RAD2DEG_F,		// IMU1 Q angular rate
			imu.theta[2] * C_RAD2DEG_F);		// IMU1 R angular rate
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tIMU1 vel\t");
        ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3 "\n",
            imu.vel[0],						// IMU1 X acceleration
            imu.vel[1],						// IMU1 Y acceleration
            imu.vel[2]);						// IMU1 Z acceleration
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringBarometer(const barometer_t &baro, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(baro.time - lastTime);
	lastTime = baro.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", baro.time);
#endif

	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", %.2fkPa", baro.bar);
	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", %.1fm", baro.mslBar);
	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", %.2fC", baro.barTemp);
	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", Humid. %.1f%%", baro.humidity);

	if (m_displayMode == DMODE_PRETTY)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringMagnetometer(const magnetometer_t &mag, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime[2] = { 0 };
	double dtMs = 1000.0*(mag.time - lastTime[i]);
	lastTime[i] = mag.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", mag.time);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", mag[%6.2f,%6.2f,%6.2f]",
			mag.mag[0],					// X magnetometer
			mag.mag[1],					// Y magnetometer
			mag.mag[2]);				// Z magnetometer
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tmag\t");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P2 "\n",
			mag.mag[0],					// X magnetometer
			mag.mag[1],					// Y magnetometer
			mag.mag[2]);				// Z magnetometer
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringMagCal(const mag_cal_t &mag, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

	switch (mag.state)
	{
	default:							ptr += SNPRINTF(ptr, ptrEnd - ptr, "  state %3d,               ", mag.state);	break;
	case MAG_CAL_STATE_MULTI_AXIS:		ptr += SNPRINTF(ptr, ptrEnd - ptr, "  state %3d (MULTI-AXIS ), ", mag.state);	break;
	case MAG_CAL_STATE_SINGLE_AXIS:		ptr += SNPRINTF(ptr, ptrEnd - ptr, "  state %3d (SINGLE-AXIS), ", mag.state);	break;
	case MAG_CAL_STATE_ABORT:			ptr += SNPRINTF(ptr, ptrEnd - ptr, "  state %3d (ABORT      ), ", mag.state);	break;
	case MAG_CAL_STATE_RECAL_RUNNING:	ptr += SNPRINTF(ptr, ptrEnd - ptr, "  state %3d (Recal-ing  ), ", mag.state);	break;
	case MAG_CAL_STATE_RECAL_COMPLETE:	ptr += SNPRINTF(ptr, ptrEnd - ptr, "  state %3d (Recal done ), ", mag.state);	break;
	}

	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "   progress: %3.0f %%,   declination: %4.1f",
			mag.progress,
			mag.declination * C_RAD2DEG_F);
	}

	if (m_displayMode == DMODE_PRETTY)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringGpsPos(const gps_pos_t &gps, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;

	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

	return string(buf) + DataToStringGpsPos(gps, m_displayMode != DMODE_SCROLL);
}

string cInertialSenseDisplay::DataToStringGpsPos(const gps_pos_t &gps, bool full)
{
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;

#if DISPLAY_DELTA_TIME==1
	static int lastTimeMs = 0;
	int dtMs = gps.timeOfWeekMs - lastTimeMs;
	lastTimeMs = gps.timeOfWeekMs;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %3dms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %dms", gps.timeOfWeekMs);
#endif

	if (!full)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", LLA[%12.7f,%12.7f,%7.1f], %d sats, %4.1f cno, %4.3f hAcc, %4.3f vAcc, %4.3f pDop",
			gps.lla[0], gps.lla[1], gps.lla[2],
			gps.status&GPS_STATUS_NUM_SATS_USED_MASK, gps.cnoMean,
			gps.hAcc, gps.vAcc, gps.pDop);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\tSats: %2d,  ",
			gps.status&GPS_STATUS_NUM_SATS_USED_MASK);	// Satellites used in solution
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "Status: 0x%08x (", gps.status);
		switch (gps.status&GPS_STATUS_FIX_MASK)
		{
		default: 
		case GPS_STATUS_FIX_NONE:               ptr += SNPRINTF(ptr, ptrEnd - ptr, "%d", (gps.status&GPS_STATUS_FIX_MASK)>>GPS_STATUS_FIX_BIT_OFFSET);	break;
		case GPS_STATUS_FIX_2D:                 ptr += SNPRINTF(ptr, ptrEnd - ptr, "2D");           break;
		case GPS_STATUS_FIX_3D:                 ptr += SNPRINTF(ptr, ptrEnd - ptr, "3D");           break;
		case GPS_STATUS_FIX_RTK_SINGLE:         ptr += SNPRINTF(ptr, ptrEnd - ptr, "RTK Single");   break;
		case GPS_STATUS_FIX_RTK_FLOAT:          ptr += SNPRINTF(ptr, ptrEnd - ptr, "RTK Float");    break;
        case GPS_STATUS_FIX_RTK_FIX:            ptr += SNPRINTF(ptr, ptrEnd - ptr, "RTK FIX");      break;
        }
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ") \thAcc: %.3f m     cno: %3.1f dBHz\n", gps.hAcc, gps.cnoMean);	// Position accuracy
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tLLA: ");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA "    ",
			gps.lla[0],					// GPS Latitude
			gps.lla[1],					// GPS Longitude
			gps.lla[2]);				// GPS Ellipsoid altitude (meters)
		bool comma = false;
		if (gps.status&GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED)
		{	
			if (gps.status&GPS_STATUS_FLAGS_GPS1_RTK_RAW_GPS_DATA_ERROR)	{ AddCommaToString(comma, ptr, ptrEnd); ptr += SNPRINTF(ptr, ptrEnd - ptr, "Raw error"); }
			switch (gps.status&GPS_STATUS_FLAGS_ERROR_MASK)
			{
			case GPS_STATUS_FLAGS_GPS1_RTK_BASE_DATA_MISSING:				{ AddCommaToString(comma, ptr, ptrEnd); ptr += SNPRINTF(ptr, ptrEnd - ptr, "Base missing");	} break;
			case GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MOVING:			{ AddCommaToString(comma, ptr, ptrEnd); ptr += SNPRINTF(ptr, ptrEnd - ptr, "Moving base");	} break;
			case GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_INVALID:			{ AddCommaToString(comma, ptr, ptrEnd); ptr += SNPRINTF(ptr, ptrEnd - ptr, "Moving invalid, ");	} break;
			}
		}
		if (gps.status&GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED)
		{
			if (gps.status&GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED) 		{ AddCommaToString(comma, ptr, ptrEnd); ptr += SNPRINTF(ptr, ptrEnd - ptr, "Compassing"); }
		}

		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n"); 
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringRtkRel(const gps_rtk_rel_t &rel, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;

	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static int lastTimeMs = 0;
	int dtMs = rel.timeOfWeekMs - lastTimeMs;
	lastTimeMs = rel.timeOfWeekMs;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %3dms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %dms", rel.timeOfWeekMs);
#endif

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, ", V2B[%10.3f,%10.3f,%9.2f], %4.1f age, %4.1f arRatio, %4.3f dist, %4.2f bear",
			rel.baseToRoverVector[0], rel.baseToRoverVector[1], rel.baseToRoverVector[2],
			rel.differentialAge, rel.arRatio, rel.baseToRoverDistance, rel.baseToRoverHeading);
	}
	else
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tbaseToRover: ");
		ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_P3 "\n",
			rel.baseToRoverVector[0],				// Vector to base in ECEF
			rel.baseToRoverVector[1],				// Vector to base in ECEF
			rel.baseToRoverVector[2]);				// Vector to base in ECEF
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tRTK:\tdiffAge:%5.1fs  arRatio: %4.1f  dist:%7.2fm  bear:%6.1f\n", 
			rel.differentialAge, rel.arRatio, rel.baseToRoverDistance, rel.baseToRoverHeading*C_RAD2DEG_F);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringRtkMisc(const gps_rtk_misc_t& rtk, const p_data_hdr_t& hdr)
{
	(void)hdr;
	string didName = cISDataMappings::GetDataSetName(hdr.id);
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	const char* terminator = (m_displayMode != DMODE_SCROLL ? "\n" : "");
	ptr += SNPRINTF(buf, ptrEnd - ptr, "%s: T=%d, lla[%4.7f,%4.7f,%7.3f], A[%3.3f,%3.3f,%3.3f], AR:%3.3f, dop(g,h,v)[%3.3f,%3.3f,%3.3f] %s",
		didName.c_str(),
		rtk.timeOfWeekMs, rtk.baseLla[0], rtk.baseLla[1], rtk.baseLla[2],
		rtk.accuracyPos[0], rtk.accuracyPos[1], rtk.accuracyPos[2],
		rtk.arThreshold,
		rtk.gDop, rtk.hDop, rtk.vDop,
		terminator);

	if (m_displayMode != DMODE_SCROLL)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringRawGPS(const gps_raw_t& raw, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	const char* terminator = (m_displayMode != DMODE_SCROLL ? "\n" : "");
	ptr += SNPRINTF(buf, ptrEnd - ptr, "RAW GPS: receiverIndex=%d, type=%d, count=%d   %s",
		raw.receiverIndex, raw.dataType, raw.obsCount, terminator);

	if (m_displayMode != DMODE_SCROLL)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}
	return buf;
}

string cInertialSenseDisplay::DataToStringSurveyIn(const survey_in_t &survey, const p_data_hdr_t& hdr)
{
    (void)hdr;
    char buf[BUF_SIZE];
    char* ptr = buf;
    char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));
    ptr += SNPRINTF(ptr, ptrEnd - ptr, " state: %d ", survey.state);
    switch (survey.state)
    {
    case SURVEY_IN_STATE_OFF:           ptr += SNPRINTF(ptr, ptrEnd - ptr, "(off)");           break;
    case SURVEY_IN_STATE_RUNNING_3D:    ptr += SNPRINTF(ptr, ptrEnd - ptr, "(running 3D)");    break;
    case SURVEY_IN_STATE_RUNNING_FLOAT: ptr += SNPRINTF(ptr, ptrEnd - ptr, "(running Float)"); break;
    case SURVEY_IN_STATE_RUNNING_FIX:   ptr += SNPRINTF(ptr, ptrEnd - ptr, "(running Fix)");   break;
    case SURVEY_IN_STATE_SAVE_POS:      ptr += SNPRINTF(ptr, ptrEnd - ptr, "(saving pos)");    break;
    case SURVEY_IN_STATE_DONE:          ptr += SNPRINTF(ptr, ptrEnd - ptr, "(done)");          break;
    }

    int elapsedTimeMin = survey.elapsedTimeSec / 60;
    int elapsedTimeSec = survey.elapsedTimeSec - (elapsedTimeMin * 60);
    int maxDurationMin = survey.maxDurationSec / 60;
    int maxDurationSec = survey.maxDurationSec - (maxDurationMin * 60);

    ptr += SNPRINTF(ptr, ptrEnd - ptr, ", elapsed: %d:%02d of %2d:%02d", 
        elapsedTimeMin, elapsedTimeSec, maxDurationMin, maxDurationSec );
    if (m_displayMode != DMODE_SCROLL)
    {
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n\thAcc: %4.3f\tlla:", survey.hAccuracy);
        ptr += SNPRINTF(ptr, ptrEnd - ptr, PRINTV3_LLA "\n",
            survey.lla[0],					// latitude
            survey.lla[1],					// longitude
            survey.lla[2]);					// altitude
    }
    else
    {   // Single line format
        ptr += SNPRINTF(ptr, ptrEnd - ptr, ", hAcc: %4.3f ", survey.hAccuracy);
        ptr += SNPRINTF(ptr, ptrEnd - ptr, " lla[%12.7f,%12.7f,%7.1f]",
            survey.lla[0],					// latitude
            survey.lla[1],					// longitude
            survey.lla[2]);					// altitude
    }
    return buf;
}

string cInertialSenseDisplay::DataToStringSysParams(const sys_params_t& sys, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static int lastTimeMs = 0;
	int dtMs = sys.timeOfWeekMs - lastTimeMs;
	lastTimeMs = sys.timeOfWeekMs;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %3dms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %dms", sys.timeOfWeekMs);
#endif

	ptr += SNPRINTF(ptr, ptrEnd - ptr, ",%d,%d,%d\n", sys.imuSamplePeriodMs, sys.navOutputPeriodMs, sys.genFaultCode);

	if (m_displayMode == DMODE_PRETTY)
	{
		ptr = StatusToString(ptr, ptrEnd, sys.insStatus, sys.hdwStatus);
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\tTemp:  IMU %4.1f C\tBaro %4.1f C\tMCU %4.1f C\n", sys.imuTemp, sys.baroTemp, sys.mcuTemp);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringSysSensors(const sys_sensors_t& sensors, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime = 0;
	double dtMs = 1000.0*(sensors.time - lastTime);
	lastTime = sensors.time;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", sensors.time);
#endif

	// Single line format
	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", %4.1fC, pqr[%5.1f,%5.1f,%5.1f], acc[%5.1f,%5.1f,%5.1f], mag[%6.2f,%6.2f,%6.2f]",
		sensors.temp,
		sensors.pqr[0] * C_RAD2DEG_F,
		sensors.pqr[1] * C_RAD2DEG_F,
		sensors.pqr[2] * C_RAD2DEG_F,
		sensors.acc[0], sensors.acc[1], sensors.acc[2],
		sensors.mag[0], sensors.mag[1], sensors.mag[2]
	);

	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", baro[%5.2fkPa, %4.1fC, %7.2fm, %3.1f%% humidity], adc[%3.1fV, %3.1fV, %3.1fV, %3.1fV]",
		sensors.bar, sensors.barTemp, sensors.mslBar, sensors.humidity,
		sensors.vin, sensors.ana1, sensors.ana3, sensors.ana4
	);

	if (m_displayMode != DMODE_SCROLL)
	{
		ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringRTOS(const rtos_info_t& info, const p_data_hdr_t& hdr)
{
	cDataCSV csv;
	string csvString;
	csv.DataToStringCSV(hdr, (const uint8_t*)&info, csvString);
	const char* terminator = (m_displayMode != DMODE_SCROLL ? "\n" : "");
	return string("RTOS: ") + csvString + terminator;
}

string cInertialSenseDisplay::DataToStringDevInfo(const dev_info_t &info, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

	return string(buf) + DataToStringDevInfo(info, m_displayMode!=DMODE_SCROLL) + (m_displayMode!=DMODE_SCROLL ? "\n" : "");
}

string cInertialSenseDisplay::DataToStringDevInfo(const dev_info_t &info, bool full)
{
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;

	// Single line format
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " SN%d, Fw %d.%d.%d.%d %d%c, %04d-%02d-%02d",
		info.serialNumber,
		info.firmwareVer[0],
		info.firmwareVer[1],
		info.firmwareVer[2],
		info.firmwareVer[3],
		info.buildNumber,
		(info.buildDate[0] ? info.buildDate[0] : ' '),
		info.buildDate[1] + 2000,
		info.buildDate[2],
		info.buildDate[3]
	);

	if (full)
	{	// Spacious format
		ptr += SNPRINTF(ptr, ptrEnd - ptr, " %02d:%02d:%02d, Proto %d.%d.%d.%d",
			info.buildTime[0],
			info.buildTime[1],
			info.buildTime[2],
			info.protocolVer[0],
			info.protocolVer[1],
			info.protocolVer[2],
			info.protocolVer[3]
		);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringSensorsADC(const sys_sensors_adc_t &sensorsADC, const p_data_hdr_t &hdr) {
    (void) hdr; // hdr is not used

	stringstream ss;
	ss << "DID_SENSORS_ADC:";
	ss << fixed;
    ss << "time " << setprecision(3) << sensorsADC.time << ", ";
    ss << "bar " << setprecision(2) << sensorsADC.bar << ", ";
    ss << "barTemp " << setprecision(2) << sensorsADC.barTemp << ", ";
    ss << "humidity " << setprecision(2) << sensorsADC.humidity << ", ";

//     ss << " ana[" << setprecision(2);
//     for (size_t i = 0; i < NUM_ANA_CHANNELS; ++i)
//     {
//         if (i != 0) { ss << ", "; }
//         ss << sensorsADC.ana[i];
//     }
//     ss << "]";

	if (m_displayMode != DMODE_SCROLL)
	{    // Spacious format
		ss << "\n";
#define SADC_WIDTH	5
		for (size_t i = 0; i < NUM_IMU_DEVICES; ++i)
		{
			auto &imu = sensorsADC.imu[i];
			ss << "\timu[" << i << "]: " << setprecision(0);
			ss << "pqr[" << setw(SADC_WIDTH) << imu.pqr[0] << "," << setw(SADC_WIDTH) << imu.pqr[1] << "," << setw(SADC_WIDTH) << imu.pqr[2] << "], ";
			ss << "acc[" << setw(SADC_WIDTH) << imu.acc[0] << "," << setw(SADC_WIDTH) << imu.acc[1] << "," << setw(SADC_WIDTH) << imu.acc[2] << "], ";
			ss << "temp " << setprecision(3) << imu.temp << ",";
		}
		for (size_t i = 0; i < NUM_MAG_DEVICES; ++i)
		{
			auto &mag = sensorsADC.mag[i];
			ss << "mag[" << setw(SADC_WIDTH) << mag.mag[0] << "," << setw(SADC_WIDTH) << mag.mag[1] << "," << setw(SADC_WIDTH) << mag.mag[2] << "], ";
			ss << "\n";
		}
	}
	else
	{
		for (size_t i = 0; i < NUM_IMU_DEVICES; ++i)
		{
			auto &imu = sensorsADC.imu[i];
			ss << "mpu[" << i << "]: " << setprecision(0);
			ss << "pqr[" << imu.pqr[0] << "," << imu.pqr[1] << "," << imu.pqr[2] << "], ";
			ss << "acc[" << imu.acc[0] << "," << imu.acc[1] << "," << imu.acc[2] << "], ";
			ss << "temp " << setprecision(3) << imu.temp << ",";
		}
		for (size_t i = 0; i < NUM_MAG_DEVICES; ++i)
		{
			auto &mag = sensorsADC.mag[i];
			ss << "mag[" << mag.mag[0] << "," << mag.mag[1] << "," << mag.mag[2] << "], ";
		}
	}

	return ss.str();
}

string cInertialSenseDisplay::DataToStringWheelEncoder(const wheel_encoder_t &wheel, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s:", hdr.id, cISDataMappings::GetDataSetName(hdr.id));

#if DISPLAY_DELTA_TIME==1
	static double lastTime[2] = { 0 };
	double dtMs = 1000.0*(wheel.timeOfWeek - lastTime[i]);
	lastTime[i] = wheel.timeOfWeek;
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
	ptr += SNPRINTF(ptr, ptrEnd - ptr, " %.3lfs", wheel.timeOfWeek);
#endif

	ptr += SNPRINTF(ptr, ptrEnd - ptr, ", [left,right] (rad) theta[%6.2f,%6.2f]  omega[%5.2f,%5.2f]  wrap[%d,%d]\n",
		wheel.theta_l,			// Left wheel angle
		wheel.theta_r,			// Right wheel angle
		wheel.omega_l,			// Left wheel angular velocity
		wheel.omega_r,			// Right wheel angular velocity
		wheel.wrap_count_l,		// Left wheel angle wrap
		wheel.wrap_count_r		// Right wheel angle wrap
	);
	
	return buf;
}


#define DISPLAY_SNPRINTF(f_, ...)	{ptr += SNPRINTF(ptr, ptrEnd - ptr, (f_), ##__VA_ARGS__);}
#define DTS_VALUE_FORMAT	"%22s "

string cInertialSenseDisplay::DataToStringGeneric(const p_data_t* data)
{
	const map_name_to_info_t *mapInfo = cISDataMappings::GetMapInfo(data->hdr.id);

	uDatasets d = {};
	copyDataPToStructP(&d, data, sizeof(d));

	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	DISPLAY_SNPRINTF("(%d) %s:      W up, S down\n", data->hdr.id, cISDataMappings::GetDataSetName(data->hdr.id));

	data_mapping_string_t tmp;
	for (map_name_to_info_t::const_iterator it = mapInfo->begin(); it != mapInfo->end(); it++)
	{
		// Print value
		cISDataMappings::DataToString(it->second, &(data->hdr), (uint8_t*)&d, tmp);
		DISPLAY_SNPRINTF(DTS_VALUE_FORMAT, tmp);

		// Print value name
		DISPLAY_SNPRINTF("  %s\n", it->first.c_str());
	}

	return buf;
}

string cInertialSenseDisplay::DatasetToString(const p_data_t* data)
{
	if (m_editData.mapInfo == NULL)
	{
		return "";
	}

	uDatasets d = {};
	copyDataPToStructP(&d, data, sizeof(d));

	char buf[BUF_SIZE];
	char* ptr = buf;
	char* ptrEnd = buf + BUF_SIZE;
	DISPLAY_SNPRINTF("(%d) %s:      W up, S down\n", data->hdr.id, cISDataMappings::GetDataSetName(data->hdr.id));

	data_mapping_string_t tmp;
	for (map_name_to_info_t::const_iterator it = m_editData.mapInfoBegin; it != m_editData.mapInfoEnd; it++)
	{
		// Print value
		if (it == m_editData.mapInfoSelection && m_editData.editEnabled)
		{	// Show keyboard value
			DISPLAY_SNPRINTF(DTS_VALUE_FORMAT, m_editData.field.c_str());
		}
		else
		{	// Show received value
			cISDataMappings::DataToString(it->second, &(data->hdr), (uint8_t*)&d, tmp);
			DISPLAY_SNPRINTF(DTS_VALUE_FORMAT, tmp);
		}

		// Print selection marker
		if (it == m_editData.mapInfoSelection)
		{
			if (m_editData.editEnabled) { DISPLAY_SNPRINTF("X"); }
			else                        { DISPLAY_SNPRINTF("*"); }
		}
		else
		{
			DISPLAY_SNPRINTF(" ");
		}

		// Print value name
		DISPLAY_SNPRINTF(" %s\n", it->first.c_str());
	}

	return buf;
}


void cInertialSenseDisplay::GetKeyboardInput()
{
	int c = 0;

	if (KeyboardHit())
	{	// Keyboard was pressed
		c = GetChar();
	}

	if (c == 0)
	{	// No keyboard inputs
		return;
	}

	// Keyboard was pressed
	c = tolower(c);

	// printf("Keyboard input: '%c' %d\n", c, c);    // print key value for debug.  Comment out cltool_dataCallback() for this to print correctly.
	// return;

	if ((c >= '0' && c <= '9') || 
		(c >= 'a' && c <= 'f') || c == '.' || c == '-' )
	{	// Number
		m_editData.field += (char)c;
		m_editData.editEnabled = true;
	}
	else switch (c)
	{
	case 8:		// Backspace
	case 127:	// Delete
		m_editData.field.pop_back();
		break;
	case 10:
	case 13:	// Enter	// Convert string to number
		if (m_editData.editEnabled)
		{
			// val = std::stof(m_editData.field);
			m_editData.info = m_editData.mapInfoSelection->second;
			int radix = (m_editData.info.dataFlags == DataFlagsDisplayHex ? 16 : 10);
			cISDataMappings::StringToVariable(m_editData.field.c_str(), (int)m_editData.field.length(), m_editData.data, m_editData.info.dataType, m_editData.info.dataSize, radix);
			m_editData.uploadNeeded = true;

			// Copy data into local Rx buffer
			if (m_editData.pData.hdr.id == m_editData.did &&
				m_editData.pData.hdr.size+ m_editData.pData.hdr.offset >= m_editData.info.dataSize+m_editData.info.dataOffset)
			{
				memcpy(m_editData.pData.buf + m_editData.info.dataOffset, m_editData.data, m_editData.info.dataSize);
			}
		}
		StopEditing();
		break;
	case 27:	// Escape
		StopEditing();
		break;

	case 'w': VarSelectDecrement(); m_editData.field.clear(); break;	// up
	case 's': VarSelectIncrement(); m_editData.field.clear(); break;	// down
		
	case 3:		// Ctrl-C
	case 'q':
		SetExitProgram();
		break;
	}
}


void cInertialSenseDisplay::SelectEditDataset(int did)
{
	m_editData.did = did;
	m_editData.mapInfo = cISDataMappings::GetMapInfo(did);
	m_editData.mapInfoSelection = m_editData.mapInfo->begin();
	m_editData.mapInfoBegin = m_editData.mapInfo->begin();

	// Set m_editData.mapInfoBegin to end or DATASET_VIEW_NUM_ROWSth element, whichever is smaller.
	int i=0;
	for (map_name_to_info_t::const_iterator it = m_editData.mapInfo->begin(); it != m_editData.mapInfo->end(); )
	{
		if (++i>DATASET_VIEW_NUM_ROWS)
		{
			break;
		}

		m_editData.mapInfoEnd = ++it;
	}

	SetDisplayMode(cInertialSenseDisplay::DMODE_EDIT);

	// Stuff zeros in so that write-only datasets will appear
	p_data_t data = {};
	data.hdr.id = did;
	data.hdr.size = cISDataMappings::GetSize(did);
	ProcessData(&data);
}


void cInertialSenseDisplay::VarSelectIncrement() 
{
	if (m_editData.mapInfo == NULL)
	{
		return;
	}

	map_name_to_info_t::const_iterator mapInfoEnd = m_editData.mapInfoEnd;
	if (m_editData.mapInfoSelection != --(mapInfoEnd))
	{
		m_editData.mapInfoSelection++;
	}
	else if (m_editData.mapInfoEnd != m_editData.mapInfo->end())
	{
		m_editData.mapInfoBegin++;
		m_editData.mapInfoEnd++;
		m_editData.mapInfoSelection++;
		Clear();
	}

	m_editData.editEnabled = false; 
}


void cInertialSenseDisplay::VarSelectDecrement() 
{ 
	if (m_editData.mapInfo == NULL)
	{
		return;
	}

	if (m_editData.mapInfoSelection != m_editData.mapInfoBegin)
	{
		m_editData.mapInfoSelection--;
	}
	else if (m_editData.mapInfoBegin != m_editData.mapInfo->begin())
	{
		m_editData.mapInfoBegin--;
		m_editData.mapInfoEnd--;
		m_editData.mapInfoSelection--;
		Clear();
	}
	
	m_editData.editEnabled = false; 
}


void cInertialSenseDisplay::StopEditing()
{
	m_editData.editEnabled = false;
	m_editData.field.clear();
}


ostream& boldOn(ostream& os)
{

#if PLATFORM_IS_WINDOWS

	return os;

#else

	return os << "\033[1m";

#endif

}

ostream& boldOff(ostream& os)
{

#if PLATFORM_IS_WINDOWS

	return os;

#else

	return os << "\033[0m";

#endif

}

// Bold on with newline
ostream& endlbOn(ostream& os)
{

#if PLATFORM_IS_WINDOWS

	return os << endl;

#else

	return os << endl << boldOn;

#endif

}

// Bold off with newline
ostream& endlbOff(ostream& os)
{

#if PLATFORM_IS_WINDOWS

	return os << endl;

#else

	return os << endl << boldOff;

#endif

}
