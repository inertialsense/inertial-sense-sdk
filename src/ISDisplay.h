/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file ISDisplay.h
 * @brief Console/terminal display and formatting helper for cltool-style host applications:
 *        renders received device data sets as human-readable text, tracks per-DID receive
 *        statistics, and drives an interactive terminal UI for viewing/editing a data set.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_DISPLAY_H
#define IS_DISPLAY_H

#include <cstdlib>
#include <cinttypes>
#include <vector>
#include <string>

#include "com_manager.h"
#include "data_sets.h"

#include "ISConstants.h"
#include "ISDataMappings.h"
#include "ISDevice.h"

#include "serialPortPlatform.h"

#if !PLATFORM_IS_WINDOWS && !PLATFORM_IS_EMBEDDED
#include <termios.h>
#endif


/**
 * Console/terminal front-end used by cltool (and similar host tools) to render incoming device
 * data. Owns the terminal mode (raw keyboard input, cursor positioning), formats every DID's
 * data set into human-readable strings (the DataToString*() family), tracks per-DID receive
 * statistics for the "stats" display mode, and drives an interactive field-editing UI for
 * uploading modified values back to the device.
 */
class cInertialSenseDisplay
{
public:
    /** State for the interactive single-field editor (see SelectEditDataset()/GetKeyboardInput()). Named (rather than anonymous) to keep MSVC happy with the default member initializer on pData. */
    typedef struct edit_data_s // we need to name this to make MSVC happy, since we make default assignments in the struct below (pData)
    {
        const map_name_to_info_t            *mapInfo;             //!< data mapping table for the dataset currently being edited
        map_name_to_info_t::const_iterator  mapInfoSelection;      //!< currently selected field within mapInfo
        map_name_to_info_t::const_iterator  mapInfoBegin;          //!< begin iterator of mapInfo, cached for wraparound navigation
        map_name_to_info_t::const_iterator  mapInfoEnd;            //!< end iterator of mapInfo, cached for wraparound navigation
        uint32_t                            selectionArrayIdx;    //!< array index selected, for fields that are arrays

        bool                                readOnlyMode;          //!< true if editing is disabled and the dataset is only being displayed
        bool                                editEnabled;           //!< true while the user is actively editing the selected field
        std::string                         field;                 //!< text currently being typed for the selected field's new value
        uint32_t                            did;                   //!< data set ID being edited
        bool                                uploadNeeded;           //!< true once an edited value is ready to be uploaded to the device
        uint8_t                             data[MAX_DATASET_SIZE]; //!< last-received copy of the dataset being edited
        data_info_t                         info;                   //!< data mapping info for the currently selected field
        uint8_t                             pDataBuf[MAX_DATASET_SIZE]; //!< backing buffer for pData
        p_data_t                            pData = {{}, pDataBuf};     //!< packet data descriptor wrapping pDataBuf, used to stage an upload
    } edit_data_t;

    /** Top-level rendering mode for PrintData()/PrintStats(). */
    enum eDisplayMode
    {
        DMODE_QUIET = 0,    //!< no console output
        DMODE_PRETTY,       //!< human-readable, formatted multi-line output
        DMODE_SCROLL,       //!< one line of output per received message, scrolling
        DMODE_EDIT,         //!< interactive field-editing UI
        DMODE_STATS,        //!< per-DID receive statistics summary
        DMODE_RAW_PARSE,    //!< raw hex dump of received packets
    };

    /**
     * Constructs the display and initializes terminal state.
     * @param displayMode the initial display mode
     */
    cInertialSenseDisplay(eDisplayMode displayMode = DMODE_QUIET);

    /** Restores the terminal to its original mode. */
    ~cInertialSenseDisplay();

    /** Sets the current display mode. @param mode the new display mode */
    void SetDisplayMode(eDisplayMode mode) { m_displayMode = mode; };

    /** @return the current display mode */
    eDisplayMode GetDisplayMode() { return m_displayMode; };

    /** Enables or disables printing of received data as raw hex. @param enable true to show raw hex, false to show formatted data */
    void showRawData(bool enable) { m_showRawHex = enable; };

    /** Shows or hides the terminal cursor. @param visible true to show the cursor, false to hide it */
    void ShowCursor(bool visible);

    /** Restores the terminal to its normal (non-raw, cursor-visible) mode. */
    void ShutDown();

    /** Clears the terminal screen. */
    void Clear(void);

    /** Moves the cursor to the terminal's home position (top-left). */
    void Home(void);

    /**
     * Moves the cursor to a given row, column 0.
     * @param y the target row
     */
    void GoToRow(int y);

    /**
     * Moves the cursor to a given column and row.
     * @param x the target column
     * @param y the target row
     */
    void GoToColumnAndRow(int x, int y);

    /** @return the column header string describing the fields printed in scroll mode */
    std::string Header();

    /** @return a greeting banner string printed at startup */
    std::string Hello();

    /** @return a string describing the connected device */
    std::string Connected();

    /**
     * @param speed replay speed multiplier being used
     * @return a string describing the active log-replay session
     */
    std::string Replay(double speed=1.0);

    /** @return a farewell banner string printed at shutdown */
    std::string Goodbye();

    /** Puts the keyboard into non-blocking, unbuffered raw mode so KeyboardHit()/GetChar() work without waiting for Enter. */
    void SetKeyboardNonBlocking();

    /** Restores the keyboard/terminal to its original (buffered, echoing) mode. */
    void ResetTerminalMode();

    /** @return non-zero if a key is currently available to be read, 0 otherwise */
    int KeyboardHit();

    /** @return the next character from the keyboard (blocking) */
    int GetChar();

    /** @return true if the user has requested the program exit (e.g. pressed 'q' or Ctrl-C) */
    bool ExitProgram();

    /** Flags that the program should exit, causing subsequent ExitProgram() calls to return true. */
    void SetExitProgram();

    /**
     * Updates per-DID statistics and buffers a formatted string for the given packet, for the binary protocol.
     * @param data the received packet data (raw buffer form)
     * @param enableReplay true if this data originates from log replay
     * @param replaySpeedX replay speed multiplier, used when enableReplay is true
     */
    void ProcessData(p_data_buf_t* data, bool enableReplay = false, double replaySpeedX = 1.0);

    /**
     * Updates per-DID statistics and buffers a formatted string for the given packet.
     * @param data the received packet data
     * @param enableReplay true if this data originates from log replay
     * @param replaySpeedX replay speed multiplier, used when enableReplay is true
     */
    void ProcessData(p_data_t *data, bool enableReplay = false, double replaySpeedX = 1.0);

    /**
     * Renders the current display mode's output to the console, throttled to refreshPeriodMs.
     * @param refreshPeriodMs minimum time, in milliseconds, between successive screen redraws (100ms = 10Hz)
     * @return true if the screen was actually redrawn, false if throttled/skipped
     */
    bool PrintData(unsigned int refreshPeriodMs = 100); // 100ms = 10Hz

    /**
     * @param comm the comm instance whose parse status/error counters are to be reported
     * @return a string summarizing the is_comm_instance_t's parse status
     */
    static std::string PrintIsCommStatus(is_comm_instance_t *comm);

    /**
     * Updates the per-DID receive statistics (count, time delta) used by DMODE_STATS.
     * @param data the received packet data
     */
    void DataToStats(const p_data_t* data);

    /** Prints the accumulated per-DID receive statistics to the console (DMODE_STATS). */
    void PrintStats();

    /**
     * @param data the received packet data
     * @return a formatted, human-readable string describing the data set, dispatched by DID
     */
    std::string DataToString(const p_data_t* data);

    /**
     * Appends a human-readable decode of insStatus/hdwStatus flag bits to a buffer.
     * @param ptr current write position in the destination buffer
     * @param ptrEnd end of the destination buffer (exclusive), used to bound writes
     * @param insStatus INS status flags to decode
     * @param hdwStatus hardware status flags to decode
     * @return the updated write position (ptr advanced past the appended text)
     */
    char* StatusToString(char* ptr, char* ptrEnd, const uint32_t insStatus, const uint32_t hdwStatus);

    /**
     * Appends a human-readable decode of the INS solution status bits of insStatus to a buffer.
     * @param ptr current write position in the destination buffer
     * @param ptrEnd end of the destination buffer (exclusive), used to bound writes
     * @param insStatus INS status flags to decode
     * @return the updated write position (ptr advanced past the appended text)
     */
    char* InsStatusToSolStatusString(char* ptr, char* ptrEnd, const uint32_t insStatus);

    /**
     * @param ins1 the INS1 data set to format
     * @param hdr the packet header associated with ins1 (used for timestamp)
     * @return a formatted, human-readable string describing ins1
     */
    std::string DataToStringINS1(const ins_1_t &ins1, const p_data_hdr_t& hdr);

    /**
     * @param ins2 the INS2 data set to format
     * @param hdr the packet header associated with ins2 (used for timestamp)
     * @return a formatted, human-readable string describing ins2
     */
    std::string DataToStringINS2(const ins_2_t &ins2, const p_data_hdr_t& hdr);

    /**
     * @param ins3 the INS3 data set to format
     * @param hdr the packet header associated with ins3 (used for timestamp)
     * @return a formatted, human-readable string describing ins3
     */
     std::string DataToStringINS3(const ins_3_t &ins3, const p_data_hdr_t& hdr);

    /**
     * @param ins4 the INS4 data set to format
     * @param hdr the packet header associated with ins4 (used for timestamp)
     * @return a formatted, human-readable string describing ins4
     */
    std::string DataToStringINS4(const ins_4_t &ins4, const p_data_hdr_t& hdr);

    /**
     * @param imus the multi-IMU data set to format
     * @param hdr the packet header associated with imus (used for timestamp)
     * @return a formatted, human-readable string describing imus
     */
    std::string DataToStringIMUs(const imus_t &imus, const p_data_hdr_t& hdr);

    /**
     * @param imus the multi-IMU data set to format
     * @param numDevices the number of IMU devices present in imus to format
     * @param full if true, includes additional detail fields
     * @return a formatted, human-readable string describing imus
     */
    static std::string DataToStringIMUs(const imus_t &imus, int numDevices, bool full=false);

    /**
     * @param imu the IMU data set to format
     * @param hdr the packet header associated with imu (used for timestamp)
     * @return a formatted, human-readable string describing imu
     */
    std::string DataToStringIMU(const imu_t &imu, const p_data_hdr_t& hdr);

    /**
     * @param imu the IMU data set to format
     * @param full if true, includes additional detail fields
     * @return a formatted, human-readable string describing imu
     */
    static std::string DataToStringIMU(const imu_t &imu, bool full=false);

    /**
     * @param imu the preintegrated IMU data set to format
     * @param hdr the packet header associated with imu (used for timestamp)
     * @return a formatted, human-readable string describing imu
     */
    std::string DataToStringPreintegratedImu(const pimu_t &imu, const p_data_hdr_t& hdr);

    /**
     * @param baro the barometer data set to format
     * @param hdr the packet header associated with baro (used for timestamp)
     * @return a formatted, human-readable string describing baro
     */
    std::string DataToStringBarometer(const barometer_t& baro, const p_data_hdr_t& hdr);

    /**
     * @param mag the magnetometer data set to format
     * @param hdr the packet header associated with mag (used for timestamp)
     * @return a formatted, human-readable string describing mag
     */
    std::string DataToStringMagnetometer(const magnetometer_t &mag, const p_data_hdr_t& hdr);

    /**
     * @param mag the magnetometer calibration data set to format
     * @param hdr the packet header associated with mag (used for timestamp)
     * @return a formatted, human-readable string describing mag
     */
    std::string DataToStringMagCal(const mag_cal_t &mag, const p_data_hdr_t& hdr);

    /**
     * @param ver the GNSS receiver version data set to format
     * @param hdr the packet header associated with ver (used for timestamp)
     * @return a formatted, human-readable string describing ver
     */
    std::string DataToStringGnssVersion(const gnss_version_t &ver, const p_data_hdr_t& hdr);

    /**
     * @param gnss the GNSS position data set to format
     * @param hdr the packet header associated with gnss (used for timestamp)
     * @return a formatted, human-readable string describing gnss
     */
    std::string DataToStringGnssPos(const gnss_pos_t &gnss, const p_data_hdr_t& hdr);

    /**
     * @param gnss the GNSS position data set to format
     * @param full if true, includes additional detail fields
     * @return a formatted, human-readable string describing gnss
     */
    static std::string DataToStringGnssPos(const gnss_pos_t &gnss, bool full=false);

    /**
     * @param gnss the GNSS RTK relative-positioning data set to format
     * @param hdr the packet header associated with gnss (used for timestamp)
     * @return a formatted, human-readable string describing gnss
     */
    std::string DataToStringRtkRel(const gnss_rtk_rel_t &gnss, const p_data_hdr_t& hdr);

    /**
     * @param sol the GNSS RTK miscellaneous data set to format
     * @param hdr the packet header associated with sol (used for timestamp)
     * @return a formatted, human-readable string describing sol
     */
    std::string DataToStringRtkMisc(const gnss_rtk_misc_t& sol, const p_data_hdr_t& hdr);

    /**
     * @param raw the raw GNSS observation/ephemeris data set to format
     * @param hdr the packet header associated with raw (used for timestamp)
     * @return a formatted, human-readable string describing raw
     */
    std::string DataToStringRawGNSS(const gnss_raw_t& raw, const p_data_hdr_t& hdr);

    /**
     * @param gnss the GNSS satellite-info data set to format
     * @param hdr the packet header associated with gnss (used for timestamp)
     * @return a formatted, human-readable string describing gnss
     */
    std::string DataToStringGnssSat(const gnss_sat_t &gnss, const p_data_hdr_t& hdr);

    /**
     * @param gnss the GNSS satellite-info data set to format
     * @param full if true, includes additional detail fields
     * @return a formatted, human-readable string describing gnss
     */
    static std::string DataToStringGnssSat(const gnss_sat_t &gnss, bool full=false);

    /**
     * @param survey the survey-in data set to format
     * @param hdr the packet header associated with survey (used for timestamp)
     * @return a formatted, human-readable string describing survey
     */
    std::string DataToStringSurveyIn(const survey_in_t &survey, const p_data_hdr_t& hdr);

    /**
     * @param sys the system parameters data set to format
     * @param hdr the packet header associated with sys (used for timestamp)
     * @return a formatted, human-readable string describing sys
     */
    std::string DataToStringSysParams(const sys_params_t& sys, const p_data_hdr_t& hdr);

    /**
     * @param sensors the system sensors data set to format
     * @param hdr the packet header associated with sensors (used for timestamp)
     * @return a formatted, human-readable string describing sensors
     */
    std::string DataToStringSysSensors(const sys_sensors_t& sensors, const p_data_hdr_t& hdr);

    /**
     * @param info the IMX RTOS task-info data set to format
     * @param hdr the packet header associated with info (used for timestamp)
     * @return a formatted, human-readable string describing info
     */
    std::string DataToStringRTOS(const rtos_info_t& info, const p_data_hdr_t& hdr);

    /**
     * @param info the GPX RTOS task-info data set to format
     * @param hdr the packet header associated with info (used for timestamp)
     * @return a formatted, human-readable string describing info
     */
    std::string DataToStringGRTOS(const gpx_rtos_info_t& info, const p_data_hdr_t& hdr);

    /**
     * @param info the device info data set to format
     * @param hdr the packet header associated with info (used for timestamp)
     * @return a formatted, human-readable string describing info
     */
    std::string DataToStringDevInfo(const dev_info_t &info, const p_data_hdr_t& hdr);

    /**
     * @param info the device info data set to format
     * @param flags formatting flags controlling which fields are included
     * @return a formatted, human-readable string describing info
     */
    static std::string DataToStringDevInfo(const dev_info_t &info, int flags=0);

    /**
     * @param sensorsADC the raw ADC sensor sample data set to format
     * @param hdr the packet header associated with sensorsADC (used for timestamp)
     * @return a formatted, human-readable string describing sensorsADC
     */
    std::string DataToStringSensorsADC(const sys_sensors_adc_t &sensorsADC, const p_data_hdr_t& hdr);

    /**
     * @param enc the wheel encoder data set to format
     * @param hdr the packet header associated with enc (used for timestamp)
     * @return a formatted, human-readable string describing enc
     */
    std::string DataToStringWheelEncoder(const wheel_encoder_t &enc, const p_data_hdr_t& hdr);

    /**
     * @param gpxStatus the GPX status data set to format
     * @param hdr the packet header associated with gpxStatus (used for timestamp)
     * @return a formatted, human-readable string describing gpxStatus
     */
    std::string DataToStringGPXStatus(const gpx_status_t &gpxStatus, const p_data_hdr_t& hdr);

    /**
     * @param debug the debug array data set to format
     * @param hdr the packet header associated with debug (used for timestamp)
     * @return a formatted, human-readable string describing debug
     */
    std::string DataToStringDebugArray(const debug_array_t &debug, const p_data_hdr_t& hdr);

    /**
     * @param portMon the port monitor data set to format
     * @param hdr the packet header associated with portMon (used for timestamp)
     * @return a formatted, human-readable string describing portMon
     */
    std::string DataToStringPortMonitor(const port_monitor_t &portMon, const p_data_hdr_t& hdr);

    /**
     * @param event the device event data set to format
     * @param hdr the packet header associated with event (used for timestamp)
     * @return a formatted, human-readable string describing event
     */
    std::string DataToStringEvent(const did_event_t &event, const p_data_hdr_t& hdr);

    /**
     * @param raw_data pointer to the raw packet bytes to dump
     * @param hdr the packet header associated with raw_data (used for timestamp)
     * @param bytesPerLine number of bytes to print per output line
     * @return a hex-dump string of raw_data
     */
    std::string DataToStringRawHex(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine);

    /**
     * @param raw_data pointer to the raw packet bytes to dump
     * @param hdr the packet header associated with raw_data (used for timestamp)
     * @param bytesPerLine number of bytes to print per output line
     * @param colorize if true, colorizes the hex dump output using ANSI escape codes
     * @return a hex-dump string of raw_data
     */
    std::string DataToStringPacket(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine, bool colorize);

    /**
     * @param data the received packet data
     * @return a generic, mapping-driven, human-readable string describing data (used as a fallback for DIDs without a dedicated formatter)
     */
    std::string DataToStringGeneric(const p_data_t* data);

    /**
     * Appends ", " to the buffer if comma is already true, then sets comma to true. Used to separate list items when building comma-separated output.
     * @param[in,out] comma tracks whether a preceding item has already been written; updated to true
     * @param[in,out] ptr current write position in the destination buffer; advanced past the appended text
     * @param ptrEnd end of the destination buffer (exclusive), used to bound writes
     */
    static void AddCommaToString(bool &comma, char* &ptr, char* &ptrEnd){ if (comma) { ptr += SNPRINTF(ptr, ptrEnd - ptr, ", "); } comma = true; };

    /**
     * @param data the received packet data
     * @return a formatted, human-readable string describing the data set (top-level entry point dispatched by display mode)
     */
    std::string DatasetToString(const p_data_t* data);

    /** Reads and processes any pending keyboard input, driving the interactive editor's navigation/typing. */
    void GetKeyboardInput();

    /**
     * Begins interactively editing the dataset for the given DID.
     * @param did the data set ID to edit
     * @param readOnlyMode if true, the dataset is displayed but cannot be modified
     */
    void SelectEditDataset(int did, bool readOnlyMode=false);

    /** Advances the field-editor selection to the next field. */
    void VarSelectIncrement();

    /** Moves the field-editor selection to the previous field. */
    void VarSelectDecrement();

    /** Ends the current field-editing session, discarding an in-progress edit. */
    void StopEditing();

    /** @return true if an edited value is ready to be uploaded; clears the pending flag as a side effect */
    bool UploadNeeded() { bool uploadNeeded = m_editData.uploadNeeded; m_editData.uploadNeeded = false; return uploadNeeded; };

    /** @return a pointer to the current editor state, for staging an upload */
    edit_data_t *EditData() { return &m_editData; }

    /**
     * Restricts output to a specific set of DIDs, printed once and then exits (used by cltool's non-interactive "print and quit" mode).
     * @param did the list of DIDs to print; if empty, interactive mode is re-enabled
     */
    void setOutputOnceDid(std::vector<uint32_t> did)
    {
        m_outputOnceDid = did; m_interactiveMode = did.empty();
    }
    // void SetSerialPort(serial_port_t* port) { m_port = port; }
    // void SetCommInstance(is_comm_instance_t* comm) { m_comm = comm; }

    /** Sets the device associated with this display, used for device-specific formatting. @param activeDevice handle of the active device */
    void setDevice(device_handle_t activeDevice) { m_device = activeDevice; }

private:
    std::string VectorToString();
    void DataToVector(const p_data_t* data);

    bool m_nonblockingkeyboard = false;
    std::vector<std::string> m_didMsgs;
    eDisplayMode m_displayMode = DMODE_QUIET;
    uint32_t m_startMs = 0;
    device_handle_t m_device = NULL;
    // serial_port_t* m_port = NULL;
    // is_comm_instance_t* m_comm = NULL;

    bool m_enableReplay = false;
    double m_replaySpeedX = 1.0;

    edit_data_t m_editData = {};
    std::vector<uint32_t> m_outputOnceDid = {};            // Set to DID to display then exit cltool.  0 = disabled
    bool m_interactiveMode = true;
    bool m_showRawHex = false;

    struct sDidStats
    {
        int lastTimeMs;    //!< the timestamp (ms) of the previous sample used to compute dtMs
        int dtMs;          //!< the time delta (ms) between the two most recent samples
        int count;         //!< the number of samples received for this DID
    };

    std::vector<sDidStats> m_didStats;

#if PLATFORM_IS_WINDOWS

    HANDLE m_windowsConsoleIn;
    HANDLE m_windowsConsoleOut;

#else

    struct termios orig_termios_;

#endif

};

/**
 * Turns on bold text for subsequent output to the stream, using an ANSI escape code.
 * @param os the output stream to modify
 * @return the same stream, for chaining with operator<<
 */
std::ostream& boldOn(std::ostream& os);

/**
 * Turns off bold text for subsequent output to the stream, using an ANSI escape code.
 * @param os the output stream to modify
 * @return the same stream, for chaining with operator<<
 */
std::ostream& boldOff(std::ostream& os);

/**
 * Turns on bold text and appends a newline (bold on with newline).
 * @param os the output stream to modify
 * @return the same stream, for chaining with operator<<
 */
std::ostream& endlbOn(std::ostream& os);

/**
 * Turns off bold text and appends a newline (bold off with newline).
 * @param os the output stream to modify
 * @return the same stream, for chaining with operator<<
 */
std::ostream& endlbOff(std::ostream& os);

#endif // IS_DISPLAY_H
