/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

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
#include "serialPortPlatform.h"

#if !PLATFORM_IS_WINDOWS

#include <termios.h>

#endif


/**
* Utility functions for displaying data
*/
class cInertialSenseDisplay
{
public:
	typedef struct edit_data_s // we need to name this to make MSVC happy, since we make default assignments in the struct below (pData)
	{
		const map_name_to_info_t 			*mapInfo;
		map_name_to_info_t::const_iterator 	mapInfoSelection;
		map_name_to_info_t::const_iterator 	mapInfoBegin;
		map_name_to_info_t::const_iterator 	mapInfoEnd;
		uint32_t		selectionArrayIdx;

		bool            readOnlyMode;
		bool            editEnabled;
		std::string     field;
		uint32_t        did;
		bool            uploadNeeded;
		uint8_t 		data[MAX_DATASET_SIZE];
		data_info_t 	info;
		uint8_t			pDataBuf[MAX_DATASET_SIZE];
		p_data_t		pData = {{}, pDataBuf};
	} edit_data_t;

	enum eDisplayMode
	{
		DMODE_QUIET = 0,
		DMODE_PRETTY,
		DMODE_SCROLL,
		DMODE_EDIT,
		DMODE_STATS,
        DMODE_RAW_PARSE,
	};

	cInertialSenseDisplay(eDisplayMode displayMode = DMODE_QUIET);
	~cInertialSenseDisplay();

	void SetDisplayMode(eDisplayMode mode) { m_displayMode = mode; };
	eDisplayMode GetDisplayMode() { return m_displayMode; };
    void showRawData(bool enable) { m_showRawHex = enable; };
	void ShowCursor(bool visible);
	void ShutDown();
	void Clear(void);
	void Home(void);
	void GoToRow(int y);
	void GoToColumnAndRow(int x, int y);
	std::string Header();
	std::string Hello();
	std::string Connected();
	std::string Replay(double speed=1.0);
	std::string Goodbye();

	void SetKeyboardNonBlocking();
	void ResetTerminalMode();
	int KeyboardHit();
	int GetChar();
	bool ExitProgram();
	void SetExitProgram();

	// for the binary protocol, this processes a packet of data
	void ProcessData(p_data_buf_t* data, bool enableReplay = false, double replaySpeedX = 1.0);
	void ProcessData(p_data_t *data, bool enableReplay = false, double replaySpeedX = 1.0);
	bool PrintData(unsigned int refreshPeriodMs = 100);		// 100ms = 10Hz
	static std::string PrintIsCommStatus(is_comm_instance_t *comm);
	void DataToStats(const p_data_t* data);
	void PrintStats();
	std::string DataToString(const p_data_t* data);
	char* StatusToString(char* ptr, char* ptrEnd, const uint32_t insStatus, const uint32_t hdwStatus);
	char* InsStatusToSolStatusString(char* ptr, char* ptrEnd, const uint32_t insStatus);
	std::string DataToStringINS1(const ins_1_t &ins1, const p_data_hdr_t& hdr);
	std::string DataToStringINS2(const ins_2_t &ins2, const p_data_hdr_t& hdr);
 	std::string DataToStringINS3(const ins_3_t &ins3, const p_data_hdr_t& hdr);
	std::string DataToStringINS4(const ins_4_t &ins4, const p_data_hdr_t& hdr);
	std::string DataToStringIMU(const imu_t &imu, const p_data_hdr_t& hdr);
	static std::string DataToStringIMU(const imu_t &imu, bool full=false);
	std::string DataToStringPreintegratedImu(const pimu_t &imu, const p_data_hdr_t& hdr);
	std::string DataToStringBarometer(const barometer_t& baro, const p_data_hdr_t& hdr);
	std::string DataToStringMagnetometer(const magnetometer_t &mag, const p_data_hdr_t& hdr);
	std::string DataToStringMagCal(const mag_cal_t &mag, const p_data_hdr_t& hdr);
	std::string DataToStringGpsVersion(const gps_version_t &ver, const p_data_hdr_t& hdr);
	std::string DataToStringGpsPos(const gps_pos_t &gps, const p_data_hdr_t& hdr);
	static std::string DataToStringGpsPos(const gps_pos_t &gps, bool full=false);
	std::string DataToStringRtkRel(const gps_rtk_rel_t &gps, const p_data_hdr_t& hdr);
	std::string DataToStringRtkMisc(const gps_rtk_misc_t& sol, const p_data_hdr_t& hdr);
	std::string DataToStringRawGPS(const gps_raw_t& raw, const p_data_hdr_t& hdr);
    std::string DataToStringSurveyIn(const survey_in_t &survey, const p_data_hdr_t& hdr);
	std::string DataToStringSysParams(const sys_params_t& sys, const p_data_hdr_t& hdr);
	std::string DataToStringSysSensors(const sys_sensors_t& sensors, const p_data_hdr_t& hdr);
	std::string DataToStringRTOS(const rtos_info_t& info, const p_data_hdr_t& hdr);
	std::string DataToStringGRTOS(const gpx_rtos_info_t& info, const p_data_hdr_t& hdr);
	std::string DataToStringDevInfo(const dev_info_t &info, const p_data_hdr_t& hdr);
	static std::string DataToStringDevInfo(const dev_info_t &info, bool full=false);
	std::string DataToStringSensorsADC(const sys_sensors_adc_t &sensorsADC, const p_data_hdr_t& hdr);
	std::string DataToStringWheelEncoder(const wheel_encoder_t &enc, const p_data_hdr_t& hdr);
    std::string DataToStringGPXStatus(const gpx_status_t &gpxStatus, const p_data_hdr_t& hdr);
    std::string DataToStringDebugArray(const debug_array_t &debug, const p_data_hdr_t& hdr);
    std::string DataToStringPortMonitor(const port_monitor_t &portMon, const p_data_hdr_t& hdr);
	std::string DataToStringEvent(const did_event_t &event, const p_data_hdr_t& hdr);
    std::string DataToStringRawHex(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine);
    std::string DataToStringPacket(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine, bool colorize);
	std::string DataToStringGeneric(const p_data_t* data);
	static void AddCommaToString(bool &comma, char* &ptr, char* &ptrEnd){ if (comma) { ptr += SNPRINTF(ptr, ptrEnd - ptr, ", "); } comma = true; };

	std::string DatasetToString(const p_data_t* data);

	void GetKeyboardInput();
	void SelectEditDataset(int did, bool readOnlyMode=false);
	void VarSelectIncrement();
	void VarSelectDecrement();
	void StopEditing();
	bool UploadNeeded() { bool uploadNeeded = m_editData.uploadNeeded; m_editData.uploadNeeded = false; return uploadNeeded; };
	edit_data_t *EditData() { return &m_editData; }
	void setOutputOnceDid(int did) { m_outputOnceDid = did; m_interactiveMode = m_outputOnceDid == 0; }
	void SetSerialPort(serial_port_t* port) { m_port = port; }
	void SetCommInstance(is_comm_instance_t* comm) { m_comm = comm; }

private:
	std::string VectorToString();
	void DataToVector(const p_data_t* data);

	bool m_nonblockingkeyboard = false;
	std::vector<std::string> m_didMsgs;
	eDisplayMode m_displayMode = DMODE_QUIET;
	uint32_t m_startMs = 0;
	serial_port_t* m_port = NULL;
	is_comm_instance_t* m_comm = NULL;

	bool m_enableReplay = false;
	double m_replaySpeedX = 1.0;

	edit_data_t m_editData = {};
	uint32_t m_outputOnceDid = 0;			// Set to DID to display then exit cltool.  0 = disabled
	bool m_interactiveMode = true;
    bool m_showRawHex = false;

	struct sDidStats
	{
		int lastTimeMs;
		int dtMs;
		int count;
	};

	std::vector<sDidStats> m_didStats;

#if PLATFORM_IS_WINDOWS

	HANDLE m_windowsConsoleIn;
	HANDLE m_windowsConsoleOut;

#else

    struct termios orig_termios_;

#endif

};

std::ostream& boldOn(std::ostream& os);
std::ostream& boldOff(std::ostream& os);
std::ostream& endlbOn(std::ostream& os);  // Bold on with newline
std::ostream& endlbOff(std::ostream& os); // Bold off with newline

#endif // IS_DISPLAY_H
