/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IS_DISPLAY_H
#define IS_DISPLAY_H

#include <stdlib.h>
#include <inttypes.h>
#include <vector>
#include <string>

#include "com_manager.h"
#include "data_sets.h"

/*!
* Utility functions for displaying data
*/
class cInertialSenseDisplay
{
public:
	enum eDisplayMode
	{
		DMODE_PRETTY,
		DMODE_SCROLL,
		DMODE_STATS,
		DMODE_QUITE
	};

	cInertialSenseDisplay();

	void SetDisplayMode(int mode) { m_displayMode = mode; };
	void Clear(void);
	void Home(void);
	void GoToRow(unsigned int row);
	std::string Hello();
	std::string Connected();
	std::string Replay(double speed=1.0);
	std::string Goodbye();

	// for the binary protocol, this processes a packet of data
	void ProcessData(p_data_t *data, bool enableReplay = false, double replaySpeedX = 1.0);
	void DataToStats(const p_data_t* data);
	std::string DataToString(const p_data_t* data);
	std::string DataToStringINS1(const ins_1_t &ins1, const p_data_hdr_t& hdr);
	std::string DataToStringINS2(const ins_2_t &ins2, const p_data_hdr_t& hdr);
	std::string DataToStringDualIMU(const dual_imu_t &imu, const p_data_hdr_t& hdr);
	std::string DataToStringIMU(const imu_t &imu, const p_data_hdr_t& hdr);
	std::string DataToStringDThetaVel(const delta_theta_vel_t &imu, const p_data_hdr_t& hdr);
	std::string DataToStringMag(const magnetometer_t &mag, const p_data_hdr_t& hdr);
	std::string DataToStringBaro(const barometer_t &baro, const p_data_hdr_t& hdr);
	std::string DataToStringGPS(const gps_t &gps, const p_data_hdr_t& hdr);
	std::string DataToStringDevInfo(const dev_info_t &info, const p_data_hdr_t& hdr);
	std::string DataToStringSysParams(const sys_params_t& sys, const p_data_hdr_t& hdr);
	std::string DataToStringSysSensors(const sys_sensors_t& sensors, const p_data_hdr_t& hdr);

private:
	std::string VectortoString();
	void DataToVector(const p_data_t* data);

	std::vector<std::string>	m_didMsgs;
	int							m_displayMode;


	struct sDidStats
	{
		int lastTimeMs;
		int dtMs;
		int count;
	};

	std::vector<sDidStats>	m_didStats;

};


std::ostream& boldOn(std::ostream& os);
std::ostream& boldOff(std::ostream& os);
std::ostream& endlbOn(std::ostream& os);  // Bold on with newline
std::ostream& endlbOff(std::ostream& os); // Bold off with newline




#endif // IS_DISPLAY_H
