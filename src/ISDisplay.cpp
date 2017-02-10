/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <math.h>

#include "ISUtilities.h"
#include "ISDisplay.h"
#include "ISConstants.h"

using namespace std;

#if defined(_WIN32)		//   =========  Windows  =========

#include <windows.h>

#else					//   ==========  Linux  ==========

#include <unistd.h>
#include <sys/time.h>

#endif

#define PRINTV3_P1	"%7.1f,%7.1f,%7.1f\n"
#define PRINTV3_P2	" %7.2f,%7.2f,%7.2f\n"
#define PRINTV3_P3	"  %7.3f,%7.3f,%7.3f\n"
#define PRINTV4_P1	"%7.1f,%7.1f,%7.1f,%7.1f\n"
#define PRINTV4_P2	" %7.2f,%7.2f,%7.2f,%7.2f\n"
#define PRINTV4_P3	"  %7.3f,%7.3f,%7.3f,%7.3f\n"
#define PRINTV3_LLA	"%13.7f,%13.7f,%7.1f\n"
#define BUF_SIZE 1024


cInertialSenseDisplay::cInertialSenseDisplay()
{
	cout << endl << Hello() << endl;

	m_displayMode = false;
}


void cInertialSenseDisplay::Clear(void)
{
#if defined(_WIN32)
	system( "cls" );
#else
	printf( "\x1B[2J" );		// VT100 terminal command
#endif
}

void cInertialSenseDisplay::Home(void)
{
#if defined(_WIN32)
	COORD pos = { 0, 0 };
	HANDLE output = GetStdHandle( STD_OUTPUT_HANDLE );
	SetConsoleCursorPosition( output, pos );
#else
	printf( "\x1B[H" );			// VT100 terminal command
#endif
}

void cInertialSenseDisplay::GoToRow(unsigned int row)
{
#if defined(_WIN32)
	(void)row;
#else
	printf( "\x1B[%dH", row );	// VT100 terminal command
#endif
}

string cInertialSenseDisplay::Hello()
{
	return "$ Inertial Sense.  Press CTRL-C to terminate.\n";
}

string cInertialSenseDisplay::Connected()
{
	return "$ Inertial Sense.  Connected.  Press CTRL-C to terminate.\n";
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


void cInertialSenseDisplay::ProcessData(p_data_t *data, bool enableReplay, double replaySpeedX)
{
	int curTimeMs = current_weekMs();

	if (enableReplay)
	{
		static bool isTowMode = false;
		static int gpsTowMsOffset = 0;
		static int msgTimeMsOffset = 0;
		int msgTimeMs = 0;

		// Copy only new data
		uDatasets d = {};
		copyDataPToStructP(&d, data, sizeof(uDatasets));

		// Record message time.  In either ToW or time since boot.
		switch (data->hdr.id)
		{
			// Time of week - double
		case DID_INS_MISC:
		case DID_INS_1:
		case DID_INS_2:				
			msgTimeMs = (int)(1000.0*d.ins1.timeOfWeek); 
			isTowMode = true;
			break;

			// Time of week - uint32 ms
		case DID_SYS_PARAMS:
		case DID_GPS_POS:			
			msgTimeMs = d.gps.pos.timeOfWeekMs; 
			isTowMode = true;
			break;

		case DID_GPS:				
			msgTimeMs = d.gps.pos.timeOfWeekMs; 
			gpsTowMsOffset = (int)(1000.0*d.gps.towOffset); 
			isTowMode = true; 
			break;

			// Time since boot - double
		case DID_MAGNETOMETER_1:
		case DID_BAROMETER:
		case DID_SYS_SENSORS:
		case DID_DELTA_THETA_VEL:
		case DID_DUAL_IMU:
		case DID_EKF_STATES:
		case DID_IMU_2:
		case DID_IMU_1:
			if( isTowMode )
				msgTimeMs = (int)(1000.0*d.imu.time) + gpsTowMsOffset; 
			else
				msgTimeMs = (int)(1000.0*d.imu.time);
			break;

			// Unidentified data type
// 		default: printf("Unknown DID %d\t", data->hdr.id);	return;			
		}


		// Control replay rate
		if (msgTimeMs != 0 && replaySpeedX>0.0)
		{
			for (;;)
			{
				curTimeMs = current_weekMs();

				// Replay speed
				int replayTimeMs = (int)(long)(((double)curTimeMs)*replaySpeedX);

				// Reinitialize message offset
				if (abs(msgTimeMs + msgTimeMsOffset - replayTimeMs) > 1500)
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


	static int timeSinceRefreshMs = 0;
	static int timeSinceClearMs = 0;
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


	// Display Data


	switch (m_displayMode)
	{
	default:
		m_displayMode = DMODE_PRETTY;
	case DMODE_PRETTY:

		// Data stays at fixed location (no scroll history)
		DataToVector(data);

		// Limit printData display updates to 20Hz (50 ms)
		if (curTimeMs - timeSinceRefreshMs > 50 || curTimeMs < timeSinceRefreshMs)
		{
			Home();
			if (enableReplay)
				cout << Replay(replaySpeedX) << endl;
			else
				cout << Connected() << endl;

			cout << VectortoString();

			timeSinceRefreshMs = curTimeMs;
		}
		break;

	case DMODE_STATS:
		// Limit printData display updates to 20Hz (50 ms)
		if (curTimeMs - timeSinceRefreshMs > 50 || curTimeMs < timeSinceRefreshMs)
		{
			Home();
			cout << Connected() << endl;
			DataToStats(data);

			timeSinceRefreshMs = curTimeMs;
		}
		break;


	case DMODE_SCROLL:	// Scroll display 
		cout << DataToString(data) << endl;
		break;
	}



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
	int curTimeMs = current_weekMs();
	sDidStats &s = m_didStats[id];
	s.count++;
	if(s.lastTimeMs)
		s.dtMs = curTimeMs - s.lastTimeMs;
	s.lastTimeMs = curTimeMs;

	// Display stats
	printf("Data ID    Count         dt\n");
	for (int i = 0; i < (int)m_didStats.size(); i++)
	{
		sDidStats &s = m_didStats[i];
		if (s.count)
		{
			printf(" %6d %8d %10.3lf\n", i, s.count, s.dtMs*0.001);
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
	case DID_DEV_INFO:			str = DataToStringDevInfo(d.devInfo, data->hdr);			break;
	case DID_DUAL_IMU:			str = DataToStringDualIMU(d.dualImu, data->hdr);			break;
	case DID_IMU_1:
	case DID_IMU_2:				str = DataToStringIMU(d.imu, data->hdr);					break;
	case DID_DELTA_THETA_VEL:	str = DataToStringDThetaVel(d.dThetaVel, data->hdr);		break;
	case DID_INS_1:				str = DataToStringINS1(d.ins1, data->hdr);				break;
	case DID_INS_2:				str = DataToStringINS2(d.ins2, data->hdr);				break;
	case DID_MAGNETOMETER_1:
	case DID_MAGNETOMETER_2:		str = DataToStringMag(d.mag, data->hdr);					break;
	case DID_BAROMETER:			str = DataToStringBaro(d.baro, data->hdr);				break;
	case DID_GPS:				str = DataToStringGPS(d.gps, data->hdr);					break;
	case DID_SYS_PARAMS:			str = DataToStringSysParams(d.sysParams, data->hdr);		break;
	case DID_SYS_SENSORS:		str = DataToStringSysSensors(d.sysSensors, data->hdr);	break;

	default:
		char buf[128];
		SNPRINTF(buf, 128, "DID: %d - No Print Setup\n", data->hdr.id);
		str = buf;
		break;
	}

	return str;
}


string cInertialSenseDisplay::DataToStringINS1(const ins_1_t &ins1, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_INS_1: %.3lfs %dwk", ins1.timeOfWeek, ins1.week);

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, ", %s theta[%6.2f,%6.2f,%7.2f], uvw[%6.2f,%6.2f,%6.2f], lla[%12.7f,%12.7f,%7.1f], ned[%6.3f,%6.3f,%6.3f]",
			((ins1.iStatus&INS_STATUS_ATT_ALIGNED) ? "aligned" : "_______"),
			ins1.theta[0] * C_RAD2DEG_F,
			ins1.theta[1] * C_RAD2DEG_F,
			ins1.theta[2] * C_RAD2DEG_F,
			ins1.uvw[0], ins1.uvw[1], ins1.uvw[2],
			ins1.lla[0], ins1.lla[1], ins1.lla[2],
			ins1.ned[0], ins1.ned[1], ins1.ned[2]);
	}
	else
	{	// Spacious format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n\tEuler\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P2,
			ins1.theta[0] * C_RAD2DEG_F,	// Roll
			ins1.theta[1] * C_RAD2DEG_F,	// Pitch
			ins1.theta[2] * C_RAD2DEG_F);	// Yaw
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tUWV\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P1,
			ins1.uvw[0],					// U body velocity
			ins1.uvw[1],					// V body velocity
			ins1.uvw[2]);				// W body velocity
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tLLA\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_LLA,
			ins1.lla[0],					// INS Latitude
			ins1.lla[1],					// INS Longitude
			ins1.lla[2]);				// INS Ellipsoid altitude (meters)
											// 		SNPRINTF(&buf[n],BUF_SIZE-n, "\tiStatus    0x%08x\n", g_msg.ins1.iStatus);
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tAligned    Att %d   Vel %d   Pos %d\n",
			(ins1.iStatus & INS_STATUS_ATT_ALIGNED) != 0,
			(ins1.iStatus & INS_STATUS_VEL_ALIGNED) != 0,
			(ins1.iStatus & INS_STATUS_POS_ALIGNED) != 0
		);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringINS2(const ins_2_t &ins2, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_INS_2: %.3lfs %dwk", ins2.timeOfWeek, ins2.week);	

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, ", %s qn2b[%6.3f,%6.3f,%6.3f,%6.3f], uvw[%6.2f,%6.2f,%6.2f], lla[%12.7f,%12.7f,%7.1f]",
			((ins2.iStatus&INS_STATUS_ATT_ALIGNED) ? "aligned" : "_______"),
			ins2.qn2b[0], ins2.qn2b[1], ins2.qn2b[2], ins2.qn2b[3],
			ins2.uvw[0], ins2.uvw[1], ins2.uvw[2],
			ins2.lla[0], ins2.lla[1], ins2.lla[2]);
	}
	else
	{	// Spacious format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n\tQuat\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV4_P3,					// Quaternion attitude rotation
			ins2.qn2b[0],					// W
			ins2.qn2b[1],					// X
			ins2.qn2b[2],					// Y
			ins2.qn2b[3]);					// Z
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tUWV\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P1,
			ins2.uvw[0],					// U body velocity
			ins2.uvw[1],					// V body velocity
			ins2.uvw[2]);				// W body velocity
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tLLA\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_LLA,
			ins2.lla[0],					// INS Latitude
			ins2.lla[1],					// INS Longitude
			ins2.lla[2]);				// INS Ellipsoid altitude (meters)
											// 		n += SNPRINTF(&buf[n],BUF_SIZE-n, "\tiStatus    0x%08x\n", g_msg.ins2.iStatus);
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tAligned    Att %d   Vel %d   Pos %d\n",
			(ins2.iStatus & INS_STATUS_ATT_ALIGNED) != 0,
			(ins2.iStatus & INS_STATUS_VEL_ALIGNED) != 0,
			(ins2.iStatus & INS_STATUS_POS_ALIGNED) != 0);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringDualIMU(const dual_imu_t &imu, const p_data_hdr_t& hdr)
{
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_DUAL_IMU: %.3lfs", imu.time);

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		for (int i = 0; i < 2; i++)
		{
			n += SNPRINTF(&buf[n], BUF_SIZE - n, ", pqr[%5.1f,%5.1f,%5.1f], acc[%5.1f,%5.1f,%5.1f]",
				imu.I[i].pqr[0] * C_RAD2DEG_F,
				imu.I[i].pqr[1] * C_RAD2DEG_F,
				imu.I[i].pqr[2] * C_RAD2DEG_F,
				imu.I[i].acc[0], imu.I[i].acc[1], imu.I[i].acc[2]);
		}
	}
	else
	{	// Spacious format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n");
		for (int i = 0; i < 2; i++)
		{
			n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tPQR\t");
			n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P1,
				imu.I[i].pqr[0] * C_RAD2DEG_F,		// P angular rate
				imu.I[i].pqr[1] * C_RAD2DEG_F,		// Q angular rate
				imu.I[i].pqr[2] * C_RAD2DEG_F);		// R angular rate
			n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tAcc\t");
			n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P1,
				imu.I[i].acc[0],					// X acceleration
				imu.I[i].acc[1],					// Y acceleration
				imu.I[i].acc[2]);					// Z acceleration
		}
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringIMU(const imu_t &imu, const p_data_hdr_t& hdr)
{
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_IMU_%d: %.3lfs", (hdr.id == DID_IMU_1 ? 1 : 2), imu.time);

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, ", pqr[%5.1f,%5.1f,%5.1f], acc[%5.1f,%5.1f,%5.1f]",
			imu.I.pqr[0] * C_RAD2DEG_F,
			imu.I.pqr[1] * C_RAD2DEG_F,
			imu.I.pqr[2] * C_RAD2DEG_F,
			imu.I.acc[0], imu.I.acc[1], imu.I.acc[2]);
	}
	else
	{	// Spacious format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n\tPQR\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P1,
			imu.I.pqr[0] * C_RAD2DEG_F,		// P angular rate
			imu.I.pqr[1] * C_RAD2DEG_F,		// Q angular rate
			imu.I.pqr[2] * C_RAD2DEG_F);		// R angular rate
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tAcc\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P1,
			imu.I.acc[0],					// X acceleration
			imu.I.acc[1],					// Y acceleration
			imu.I.acc[2]);					// Z acceleration
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringDThetaVel(const delta_theta_vel_t &imu, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_DELTA_THETA_VEL: %.3lfs", imu.time);

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, ", dtheta[%6.3f,%6.3f,%6.3f], duvw[%6.3f,%6.3f,%6.3f]",
			imu.theta[0] * C_RAD2DEG_F,
			imu.theta[1] * C_RAD2DEG_F,
			imu.theta[2] * C_RAD2DEG_F,
			imu.uvw[0], imu.uvw[1], imu.uvw[2]);
	}
	else
	{	// Spacious format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n\tdtheta\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P3,
			imu.theta[0] * C_RAD2DEG_F,		// P angular rate
			imu.theta[1] * C_RAD2DEG_F,		// Q angular rate
			imu.theta[2] * C_RAD2DEG_F);		// R angular rate
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tduvw\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P3,
			imu.uvw[0],					// X acceleration
			imu.uvw[1],					// Y acceleration
			imu.uvw[2]);					// Z acceleration
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringMag(const magnetometer_t &mag, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_MAGNETOMETER_%d: %.3lfs", (hdr.id == DID_MAGNETOMETER_1 ? 1 : 2), mag.time);

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, ", mag[%6.2f,%6.2f,%6.2f]",
			mag.mag[0],					// X magnetometer
			mag.mag[1],					// Y magnetometer
			mag.mag[2]);					// Z magnetometer
	}
	else
	{	// Spacious format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n\tmag\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_P2,
			mag.mag[0],					// X magnetometer
			mag.mag[1],					// Y magnetometer
			mag.mag[2]);					// Z magnetometer
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringBaro(const barometer_t &baro, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_BAROMETER: %.3lfs", baro.time);
	n += SNPRINTF(&buf[n], BUF_SIZE - n, ", %.2fkPa", baro.bar);
	n += SNPRINTF(&buf[n], BUF_SIZE - n, ", %.1fm", baro.mslBar);
	n += SNPRINTF(&buf[n], BUF_SIZE - n, ", %.2fC", baro.barTemp);
	n += SNPRINTF(&buf[n], BUF_SIZE - n, ", Humid. %.1f%%", baro.humidity);

	if (m_displayMode == DMODE_PRETTY)
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n");

	return buf;
}

string cInertialSenseDisplay::DataToStringGPS(const gps_t &gps, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_GPS: %dms %dwk", gps.pos.timeOfWeekMs, gps.pos.week);

	if (m_displayMode == DMODE_SCROLL)
	{	// Single line format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, ", LLA[%12.7f,%12.7f,%7.1f], %d sats, %d cno, %4.2f hAcc, %4.2f vAcc, %4.2f pDop",
			gps.pos.lla[0], gps.pos.lla[1], gps.pos.lla[2],
			gps.pos.status&GPS_STATUS_NUM_SATS_USED_MASK, gps.pos.cno,
			gps.pos.hAcc, gps.pos.vAcc, gps.pos.pDop);
	}
	else
	{	// Spacious format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n\tSats:  %2d    ",
			gps.pos.status&GPS_STATUS_NUM_SATS_USED_MASK);	// Satellites used in solution
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "Status:  0x%08x", gps.pos.status);
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tAccuracy:  %.1f m   \n",
			gps.pos.hAcc);					// Position accuracy
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\tLLA\t");
		n += SNPRINTF(&buf[n], BUF_SIZE - n, PRINTV3_LLA,
			gps.pos.lla[0],					// GPS Latitude
			gps.pos.lla[1],					// GPS Longitude
			gps.pos.lla[2]);					// GPS Ellipsoid altitude (meters)
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringDevInfo(const dev_info_t &info, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_DEV_INFO:");

	// Single line format
	n += SNPRINTF(&buf[n], BUF_SIZE - n, " SN%d, Fw %d.%d.%d.%d %c%d, %04d-%02d-%02d",
		info.serialNumber,
		info.firmwareVer[3],
		info.firmwareVer[2],
		info.firmwareVer[1],
		info.firmwareVer[0],
		info.buildDate[3],
		info.buildNumber,
		info.buildDate[2] + 2000,
		info.buildDate[1],
		info.buildDate[0]
	);

	if (m_displayMode != DMODE_SCROLL)
	{	// Spacious format
		n += SNPRINTF(&buf[n], BUF_SIZE - n, " %02d:%02d:%02d, Proto %d.%d.%d.%d\n",
			info.buildTime[3],
			info.buildTime[2],
			info.buildTime[1],
			info.protocolVer[3],
			info.protocolVer[2],
			info.protocolVer[1],
			info.protocolVer[0]
		);
	}

	return buf;
}

string cInertialSenseDisplay::DataToStringSysParams(const sys_params_t& sys, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_SYS_PARAMS: %dms", sys.timeOfWeekMs);

	n += SNPRINTF(&buf[n], BUF_SIZE - n, ",%d,%d,%f,%f,%f,%f,%d,%d,%f,%f,%f,%f,%d",
		sys.iStatus, sys.hStatus, sys.alignAttDetect, sys.alignAttError, sys.alignVelError, sys.alignPosError,
		sys.sampleDtMs, sys.navDtMs, sys.ftf0, sys.magInclination, sys.magDeclination, sys.magMagnitude, sys.genFaultCode);

	if (m_displayMode != DMODE_SCROLL)
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n");

	return buf;
}

string cInertialSenseDisplay::DataToStringSysSensors(const sys_sensors_t& sensors, const p_data_hdr_t& hdr)
{
	(void)hdr;
	char buf[BUF_SIZE];
	int n = SNPRINTF(buf, BUF_SIZE, "DID_SYS_SENSORS: %.3lfs", sensors.time);

	// Single line format
	n += SNPRINTF(&buf[n], BUF_SIZE - n, ", %4.1fC, pqr[%5.1f,%5.1f,%5.1f], acc[%5.1f,%5.1f,%5.1f], mag[%6.2f,%6.2f,%6.2f], baro[%5.2fkPa, %4.1fC, %7.2fm, %3.1f%% humidity], adc[%3.1fV, %3.1fV, %3.1fV, %3.1fV]",
		sensors.temp,
		sensors.pqr[0] * C_RAD2DEG_F,
		sensors.pqr[1] * C_RAD2DEG_F,
		sensors.pqr[2] * C_RAD2DEG_F,
		sensors.acc[0], sensors.acc[1], sensors.acc[2],
		sensors.mag[0], sensors.mag[1], sensors.mag[2],
		sensors.bar, sensors.barTemp, sensors.mslBar, sensors.humidity,
		sensors.vin, sensors.ana1, sensors.ana3, sensors.ana4
	);

	if (m_displayMode != DMODE_SCROLL)
		n += SNPRINTF(&buf[n], BUF_SIZE - n, "\n");

	return buf;
}



ostream& boldOn(ostream& os)
{
#if defined(_WIN32)
	return os;
#else // Linux
	return os << "\033[1m";
#endif
}

ostream& boldOff(ostream& os)
{
#if defined(_WIN32)
	return os;
#else // Linux
	return os << "\033[0m";
#endif
}

// Bold on with newline
ostream& endlbOn(ostream& os)
{
#if defined(_WIN32)
	return os << endl;
#else // Linux
	return os << endl << boldOn;
#endif
}

// Bold off with newline
ostream& endlbOff(ostream& os)
{
#if defined(_WIN32)
	return os << endl;
#else // Linux
	return os << endl << boldOff;
#endif
}


