/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

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
#include "ISConstants.h"
#include "ISDataMappings.h"

#if !PLATFORM_IS_WINDOWS

#include <termios.h>

#endif

using namespace std;

/**
* Utility functions for displaying data
*/
class cInertialSenseDisplay
{
public:
	typedef struct
	{
		const map_name_to_info_t 			*mapInfo;
		map_name_to_info_t::const_iterator 	mapInfoSelection;
		map_name_to_info_t::const_iterator 	mapInfoBegin;
		map_name_to_info_t::const_iterator 	mapInfoEnd;

		bool            editEnabled;
		std::string     field;
		uint32_t        did;
		bool            uploadNeeded;
		uint8_t 		data[MAX_DATASET_SIZE];
		data_info_t 	info;
		p_data_t		pData;
	} edit_data_t;

	enum eDisplayMode
	{
		DMODE_PRETTY,
		DMODE_SCROLL,
		DMODE_EDIT,
		DMODE_STATS,
		DMODE_QUIET,
	};

	cInertialSenseDisplay();
	~cInertialSenseDisplay();

	void SetDisplayMode(eDisplayMode mode) { m_displayMode = mode; };
	eDisplayMode GetDisplayMode() { return m_displayMode; }
	void ShowCursor(bool visible);
	void ShutDown();
	void Clear(void);
	void Home(void);
	void GoToRow(int y);
	void GoToColumnAndRow(int x, int y);
	string Hello();
	string Connected();
	string Replay(double speed=1.0);
	string Goodbye();

	void SetKeyboardNonBlock();
	void ResetTerminalMode();
	int KeyboardHit();
	int GetChar();
	bool ExitProgram();
	void SetExitProgram();

	// for the binary protocol, this processes a packet of data
	void ProcessData(p_data_t *data, bool enableReplay = false, double replaySpeedX = 1.0);
	bool PrintData(unsigned int refreshPeriodMs = 100);		// 100ms = 10Hz
	void DataToStats(const p_data_t* data);
	void PrintStats();
	string DataToString(const p_data_t* data);
	char* StatusToString(char* ptr, char* ptrEnd, const uint32_t insStatus, const uint32_t hdwStatus);
	char* InsStatusToSolStatusString(char* ptr, char* ptrEnd, const uint32_t insStatus);
	string DataToStringINS1(const ins_1_t &ins1, const p_data_hdr_t& hdr);
	string DataToStringINS2(const ins_2_t &ins2, const p_data_hdr_t& hdr);
 	string DataToStringINS3(const ins_3_t &ins3, const p_data_hdr_t& hdr);
	string DataToStringINS4(const ins_4_t &ins4, const p_data_hdr_t& hdr);
	string DataToStringIMU(const imu_t &imu, const p_data_hdr_t& hdr);
	string DataToStringPreintegratedImu(const preintegrated_imu_t &imu, const p_data_hdr_t& hdr);
	string DataToStringBarometer(const barometer_t& baro, const p_data_hdr_t& hdr);
	string DataToStringMagnetometer(const magnetometer_t &mag, const p_data_hdr_t& hdr);
	string DataToStringMagCal(const mag_cal_t &mag, const p_data_hdr_t& hdr);
	string DataToStringGpsPos(const gps_pos_t &gps, const p_data_hdr_t& hdr, const string didName);
	string DataToStringRtkRel(const gps_rtk_rel_t &gps, const p_data_hdr_t& hdr, const string didName);
	string DataToStringRtkMisc(const gps_rtk_misc_t& sol, const p_data_hdr_t& hdr, const string didName);
	string DataToStringRawGPS(const gps_raw_t& raw, const p_data_hdr_t& hdr);
    string DataToStringSurveyIn(const survey_in_t &survey, const p_data_hdr_t& hdr);
	string DataToStringSysParams(const sys_params_t& sys, const p_data_hdr_t& hdr);
	string DataToStringSysSensors(const sys_sensors_t& sensors, const p_data_hdr_t& hdr);
	string DataToStringRTOS(const rtos_info_t& info, const p_data_hdr_t& hdr);
	string DataToStringDevInfo(const dev_info_t &info, const p_data_hdr_t& hdr);
	string DataToStringSensorsADC(const sys_sensors_adc_t &sensorsADC, const p_data_hdr_t& hdr);
	string DataToStringWheelEncoder(const wheel_encoder_t &enc, const p_data_hdr_t& hdr);
	string DataToStringGeneric(const p_data_t* data);

	string DatasetToString(const p_data_t* data);

	void GetKeyboardInput();
	void SelectEditDataset(int did);
	void VarSelectIncrement();
	void VarSelectDecrement();
	void StopEditing();
	bool UploadNeeded() { bool uploadNeeded = m_editData.uploadNeeded; m_editData.uploadNeeded = false; return uploadNeeded; };
	edit_data_t *EditData() { return &m_editData; }
	void setOutputOnceDid(int did) { m_outputOnceDid = did; m_interactiveMode = m_outputOnceDid == 0; }

private:
	string VectortoString();
	void DataToVector(const p_data_t* data);

	bool m_nonblockingkeyboard;
	vector<string> m_didMsgs;
	eDisplayMode m_displayMode = DMODE_PRETTY;
	uint16_t m_rxCount = 0;

	bool m_enableReplay = false;
	double m_replaySpeedX = 1.0;

	edit_data_t m_editData = {};
	uint32_t m_outputOnceDid = 0;			// Set to DID to display then exit cltool.  0 = disabled
	bool m_interactiveMode = true;

	struct sDidStats
	{
		int lastTimeMs;
		int dtMs;
		int count;
	};

	vector<sDidStats> m_didStats;

#if PLATFORM_IS_WINDOWS

	HANDLE m_windowsConsoleIn;
	HANDLE m_windowsConsoleOut;

#else

    struct termios orig_termios_;

#endif

};

ostream& boldOn(ostream& os);
ostream& boldOff(ostream& os);
ostream& endlbOn(ostream& os);  // Bold on with newline
ostream& endlbOff(ostream& os); // Bold off with newline

#endif // IS_DISPLAY_H
