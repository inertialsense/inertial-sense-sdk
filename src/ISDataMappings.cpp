/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <map>
#include <vector>
#include <set>
#include <algorithm>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "ISDataMappings.h"
#include "DataJSON.h"
#include "ISUtilities.h"
#include "ISConstants.h"
#include "data_sets.h"

// #define USE_IS_INTERNAL

#ifdef USE_IS_INTERNAL
#include "../../cpp/libs/families/imx/IS_internal.h"
#include "../../cpp/libs/families/imx/ISDataMappingsInternal.h"
#endif

using namespace std;

#define SYM_DEG             "°"
#define SYM_DEG_C           "°C"
#define SYM_DEG_DEG_M       "°,°,m"
#define SYM_DEG_PER_S       "°/s"
#define SYM_M_PER_S         "m/s"
#define SYM_M_PER_S_2       "m/s²"
#define SYM_DEG_C_PER_S     "°C/s"

const char s_insStatusDescription[] = "INS Status flags [0,0,MagStatus,SolStatus,     NavMode,GpsMagUsed,Variance,VarianceCoarse]";
const char s_hdwStatusDescription[] = "Hdw Status flags [Fault,BIT,RxErrCount,ComErr, SenSatHist,SensorSat,GpsSatRx,Motion]";
const char s_imuStatusDescription[] = "IMU Status flags [Sensor saturation]";

// Stringify the macro value
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#if PLATFORM_IS_EMBEDDED
cISDataMappings* cISDataMappings::s_map;
#else
cISDataMappings cISDataMappings::s_map;
#endif

const unsigned char g_asciiToLowerMap[256] =
{
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
    41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q',
    'r','s','t','u','v','w','x','y','z',91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,
    119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,
    154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,
    189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,
    224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255
};


static void PopulateSizeMappings(uint32_t sizeMap[DID_COUNT])
{
    //memset(sizeMap, 0, sizeof(sizeMap));
    // NOTE: If you use the line above, the compiler issues this warning:
    //       "‘sizeof’ on array function parameter ‘sizeMap’ will return size of ‘uint32_t* {aka unsigned int*}’"
    //       So, we use `sizeof(uint32_t) * DID_COUNT` instead to get the correct size.
    memset(sizeMap, 0, sizeof(uint32_t) * DID_COUNT);

    sizeMap[DID_DEV_INFO] = sizeof(dev_info_t);
    sizeMap[DID_BIT] = sizeof(bit_t);
    sizeMap[DID_SYS_FAULT] = sizeof(system_fault_t);
    sizeMap[DID_MAGNETOMETER] = sizeof(magnetometer_t);
    sizeMap[DID_BAROMETER] = sizeof(barometer_t);
    sizeMap[DID_IMU3_UNCAL] = sizeof(imu3_t);
    sizeMap[DID_IMU3_RAW] = sizeof(imu3_t);
    sizeMap[DID_IMU_RAW] = sizeof(imu_t);
    sizeMap[DID_IMU] = sizeof(imu_t);
    sizeMap[DID_PIMU] = sizeof(pimu_t);
    sizeMap[DID_IMU_MAG] = sizeof(imu_mag_t);
    sizeMap[DID_PIMU_MAG] = sizeof(pimu_mag_t);
    sizeMap[DID_WHEEL_ENCODER] = sizeof(wheel_encoder_t);
    sizeMap[DID_GROUND_VEHICLE] = sizeof(ground_vehicle_t);
    sizeMap[DID_SYS_CMD] = sizeof(system_command_t);
    sizeMap[DID_RMC] = sizeof(rmc_t);
    sizeMap[DID_INS_1] = sizeof(ins_1_t);
    sizeMap[DID_INS_2] = sizeof(ins_2_t);
    sizeMap[DID_INS_3] = sizeof(ins_3_t);
    sizeMap[DID_INS_4] = sizeof(ins_4_t);
    sizeMap[DID_GPS1_POS] = sizeof(gps_pos_t);
    sizeMap[DID_GPS1_RCVR_POS] = sizeof(gps_pos_t);
    sizeMap[DID_GPS1_VEL] = sizeof(gps_vel_t);
    sizeMap[DID_GPS2_POS] = sizeof(gps_pos_t);
    sizeMap[DID_GPS2_VEL] = sizeof(gps_vel_t);
    sizeMap[DID_GPS1_SAT] = sizeof(gps_sat_t);
    sizeMap[DID_GPS2_SAT] = sizeof(gps_sat_t);
    sizeMap[DID_GPS1_SIG] = sizeof(gps_sig_t);
    sizeMap[DID_GPS2_SIG] = sizeof(gps_sig_t);
    sizeMap[DID_GPS1_VERSION] = sizeof(gps_version_t);
    sizeMap[DID_GPS2_VERSION] = sizeof(gps_version_t);
    sizeMap[DID_GPS1_RTK_POS] = sizeof(gps_pos_t);
    sizeMap[DID_GPS1_RTK_POS_REL] = sizeof(gps_rtk_rel_t);
    sizeMap[DID_GPS1_RTK_POS_MISC] = sizeof(gps_rtk_misc_t);
    sizeMap[DID_GPS2_RTK_CMP_REL] = sizeof(gps_rtk_rel_t);
    sizeMap[DID_GPS2_RTK_CMP_MISC] = sizeof(gps_rtk_misc_t);
    sizeMap[DID_SYS_PARAMS] = sizeof(sys_params_t);
    sizeMap[DID_SYS_SENSORS] = sizeof(sys_sensors_t);
    sizeMap[DID_FLASH_CONFIG] = sizeof(nvm_flash_cfg_t);
    sizeMap[DID_GPS_BASE_RAW] = sizeof(gps_raw_t);
    sizeMap[DID_STROBE_IN_TIME] = sizeof(strobe_in_time_t);
    sizeMap[DID_RTOS_INFO] = sizeof(rtos_info_t);
    sizeMap[DID_CAN_CONFIG] = sizeof(can_config_t);
    sizeMap[DID_DEBUG_ARRAY] = sizeof(debug_array_t);
    sizeMap[DID_DEBUG_STRING] = sizeof(debug_string_t);
    sizeMap[DID_IO] = sizeof(io_t);
    sizeMap[DID_INFIELD_CAL] = sizeof(infield_cal_t);
    sizeMap[DID_REFERENCE_IMU] = sizeof(imu_t);
    sizeMap[DID_REFERENCE_PIMU] = sizeof(pimu_t);
    sizeMap[DID_REFERENCE_MAGNETOMETER] = sizeof(magnetometer_t);

    sizeMap[DID_INL2_MAG_OBS_INFO] = sizeof(inl2_mag_obs_info_t);
    sizeMap[DID_INL2_STATES] = sizeof(inl2_states_t);

    sizeMap[DID_SENSORS_ADC] = sizeof(sys_sensors_adc_t);
    sizeMap[DID_SENSORS_UCAL] = sizeof(sensors_w_temp_t);
    sizeMap[DID_SENSORS_TCAL] = sizeof(sensors_w_temp_t);
    sizeMap[DID_SENSORS_MCAL] = sizeof(sensors_w_temp_t);
    sizeMap[DID_SENSORS_TC_BIAS] = sizeof(sensors_t);
    sizeMap[DID_SCOMP] = sizeof(sensor_compensation_t);

    sizeMap[DID_EVB_STATUS] = sizeof(evb_status_t);
    sizeMap[DID_EVB_FLASH_CFG] = sizeof(evb_flash_cfg_t);
    sizeMap[DID_EVB_DEBUG_ARRAY] = sizeof(debug_array_t);
    sizeMap[DID_EVB_RTOS_INFO] = sizeof(evb_rtos_info_t);
    sizeMap[DID_EVB_DEV_INFO] = sizeof(dev_info_t);

    sizeMap[DID_EVENT] = sizeof(did_event_t);

    sizeMap[DID_GPX_DEV_INFO] = sizeof(dev_info_t);
    sizeMap[DID_GPX_STATUS] = sizeof(gpx_status_t);
    sizeMap[DID_GPX_FLASH_CFG] = sizeof(gpx_flash_cfg_t);
    sizeMap[DID_GPX_RTOS_INFO] = sizeof(gpx_rtos_info_t);
    sizeMap[DID_GPX_DEBUG_ARRAY] = sizeof(debug_array_t);
    sizeMap[DID_GPX_BIT] = sizeof(gpx_bit_t);
    sizeMap[DID_GPX_RMC] = sizeof(rmc_t);
    sizeMap[DID_GPX_PORT_MONITOR] = sizeof(port_monitor_t);
    
#ifdef USE_IS_INTERNAL
    sizeMap[DID_RTK_DEBUG] = sizeof(rtk_debug_t);
//     sizeMap[DID_RTK_STATE] = sizeof(rtk_state_t);
    sizeMap[DID_RTK_CODE_RESIDUAL] = sizeof(rtk_residual_t);
    sizeMap[DID_RTK_PHASE_RESIDUAL] = sizeof(rtk_residual_t);
    sizeMap[DID_NVR_USERPAGE_G0] = sizeof(nvm_group_0_t);
    sizeMap[DID_NVR_USERPAGE_G1] = sizeof(nvm_group_1_t);
    sizeMap[DID_INL2_STATUS] = sizeof(inl2_status_t);
    sizeMap[DID_INL2_MISC] = sizeof(inl2_misc_t);
    sizeMap[DID_RTK_DEBUG_2] = sizeof(rtk_debug_2_t);
#endif

#if defined(INCLUDE_LUNA_DATA_SETS)
    sizeMap[DID_EVB_LUNA_FLASH_CFG] = sizeof(evb_luna_flash_cfg_t);
    sizeMap[DID_EVB_LUNA_STATUS] = sizeof(evb_luna_status_t);
    sizeMap[DID_EVB_LUNA_SENSORS] = sizeof(evb_luna_sensors_t);
    sizeMap[DID_EVB_LUNA_REMOTE_KILL] = sizeof(evb_luna_remote_kill_t);
    sizeMap[DID_EVB_LUNA_VELOCITY_CONTROL] = sizeof(evb_luna_velocity_control_t);
    sizeMap[DID_EVB_LUNA_VELOCITY_COMMAND] = sizeof(evb_luna_velocity_command_t);
    sizeMap[DID_EVB_LUNA_AUX_COMMAND] = sizeof(evb_luna_aux_command_t);
#endif

}

static void PopulateTimestampField(uint32_t id, const data_info_t** timestamps, map_name_to_info_t mappings[DID_COUNT])
{
    static const string timestampFields[] = { "time", "timeOfWeek", "timeOfWeekMs", "seconds" };
    const map_name_to_info_t& offsetMap = mappings[id];

    if (offsetMap.size() != 0)
    {
        for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(timestampFields); i++)
        {
            map_name_to_info_t::const_iterator timestampField = offsetMap.find(timestampFields[i]);
            if (timestampField != offsetMap.end())
            {
                timestamps[id] = (const data_info_t*)&timestampField->second;
                return;
            }
        }
    }

    timestamps[id] = NULLPTR; // ensure value is not garbage
}

static void PopulateDeviceInfoMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(dev_info_t, id);
    ADD_MAP_4("reserved", reserved, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("reserved2", reserved2, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_7("hardwareType", hardwareType, DATA_TYPE_UINT8,  uint8_t,  "", "Hardware type: 1=uINS, 2=EVB, 3=IMX, 4=GPX", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("serialNumber", serialNumber, DATA_TYPE_UINT32, uint32_t, "", "Serial number", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("hardwareVer[0]", hardwareVer[0], DATA_TYPE_UINT8, uint8_t&, "", "Hardware version", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("hardwareVer[1]", hardwareVer[1], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("hardwareVer[2]", hardwareVer[2], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("hardwareVer[3]", hardwareVer[3], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("firmwareVer[0]", firmwareVer[0], DATA_TYPE_UINT8, uint8_t&, "", "Firmware version", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("firmwareVer[1]", firmwareVer[1], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("firmwareVer[2]", firmwareVer[2], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("firmwareVer[3]", firmwareVer[3], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildNumber", buildNumber, DATA_TYPE_UINT32, uint32_t, "", "Build number (0xFFFFF000 = Host key, 0x00000FFF = Build #)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("protocolVer[0]", protocolVer[0], DATA_TYPE_UINT8, uint8_t&, "", "Communications protocol version", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("protocolVer[1]", protocolVer[1], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("protocolVer[2]", protocolVer[2], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("protocolVer[3]", protocolVer[3], DATA_TYPE_UINT8, uint8_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("repoRevision", repoRevision, DATA_TYPE_UINT32, uint32_t, "", "Repo revision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("manufacturer", manufacturer, DATA_TYPE_STRING, char[DEVINFO_MANUFACTURER_STRLEN], "", "manufacturer", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildType", buildType, DATA_TYPE_UINT8, uint8_t, "", "'a'(97)=ALPHA, 'b'(98)=BETA, 'c'(99)=CANDIDATE, 'r'(114)=PRODUCTION, 'd'(100)=develop, 's'(115)=snapshot, '*'(42)=dirty", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildYear", buildYear, DATA_TYPE_UINT8, uint8_t, "", "Build year-2000", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildMonth", buildMonth, DATA_TYPE_UINT8, uint8_t, "", "Build month", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildDay", buildDay, DATA_TYPE_UINT8, uint8_t, "", "Build day", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildHour", buildHour, DATA_TYPE_UINT8, uint8_t, "", "Build hour", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildMinute", buildMinute, DATA_TYPE_UINT8, uint8_t, "", "Build minute", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildSecond", buildSecond, DATA_TYPE_UINT8, uint8_t, "", "Build second", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("buildMillisecond", buildMillisecond, DATA_TYPE_UINT8, uint8_t, "", "Build millisecond", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("addInfo", addInfo, DATA_TYPE_STRING, char[DEVINFO_ADDINFO_STRLEN], "", "Additional info", DATA_FLAGS_READ_ONLY);
    // TODO: dev_info_t.firmwareMD5Hash support
    // ADD_MAP_4("firmwareMD5Hash[0]", firmwareMD5Hash[0], DATA_TYPE_UINT32, uint32_t&);
    // ADD_MAP_4("firmwareMD5Hash[1]", firmwareMD5Hash[1], DATA_TYPE_UINT32, uint32_t&);
    // ADD_MAP_4("firmwareMD5Hash[2]", firmwareMD5Hash[2], DATA_TYPE_UINT32, uint32_t&);
    // ADD_MAP_4("firmwareMD5Hash[3]", firmwareMD5Hash[3], DATA_TYPE_UINT32, uint32_t&);
    ASSERT_SIZE(totalSize);
}

static void PopulateManufacturingInfoMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(manufacturing_info_t, DID_MANUFACTURING_INFO);
    ADD_MAP_7("serialNumber", serialNumber, DATA_TYPE_UINT32, uint32_t, "", "Serial number", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("hardwareId", hardwareId, DATA_TYPE_UINT16, uint16_t, "", "Hardware Id", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("lotNumber", lotNumber, DATA_TYPE_UINT16, uint16_t, "", "Lot number", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("date", date, DATA_TYPE_STRING, char[16], "", "Manufacturing date (YYYYMMDDHHMMSS)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("key", key, DATA_TYPE_UINT32, uint32_t, "", "key (times OTP area was set, 15 max)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("platformType", platformType, DATA_TYPE_INT32, int32_t, "", "Platform type (carrier board)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("reserved", reserved, DATA_TYPE_INT32, int32_t, "", "Reserved", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("uid[0]", uid[0], DATA_TYPE_UINT32, uint32_t&, "", "Unique microcontroller identifier", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("uid[1]", uid[1], DATA_TYPE_UINT32, uint32_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("uid[2]", uid[2], DATA_TYPE_UINT32, uint32_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("uid[3]", uid[3], DATA_TYPE_UINT32, uint32_t&, "", "\"", DATA_FLAGS_READ_ONLY);
    ASSERT_SIZE(totalSize);
}

static void PopulateIOMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(io_t, DID_IO);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("gpioStatus", gpioStatus, DATA_TYPE_UINT32, uint32_t, "", "Use to read and control GPIO input and output.", DATA_FLAGS_DISPLAY_HEX);
    ASSERT_SIZE(totalSize);
}

static void PopulateBitMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(bit_t, DID_BIT);
    ADD_MAP_6("command", command, DATA_TYPE_UINT8, uint8_t, "", "[cmd: " TOSTRING(BIT_CMD_FULL_STATIONARY) "=start full, " TOSTRING(BIT_CMD_BASIC_MOVING) "=start basic, " TOSTRING(BIT_CMD_FULL_STATIONARY_HIGH_ACCURACY) "=start full HA, " TOSTRING(BIT_CMD_OFF) "=off]");
    ADD_MAP_7("lastCommand", lastCommand, DATA_TYPE_UINT8, uint8_t, "", "Last input command", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("state", state, DATA_TYPE_UINT8, uint8_t, "", "[state: " TOSTRING(BIT_STATE_RUNNING) "=running " TOSTRING(BIT_STATE_DONE) "=done]", DATA_FLAGS_READ_ONLY);
    ADD_MAP_4("reserved", reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_7("hdwBitStatus", hdwBitStatus, DATA_TYPE_UINT32, uint32_t, "", "Hardware built-in test status. See eHdwBitStatusFlags for info.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("calBitStatus", calBitStatus, DATA_TYPE_UINT32, uint32_t, "", "Calibration built-in test status. See eCalBitStatusFlags for info.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_8("tcPqrBias", tcPqrBias, DATA_TYPE_F32, float, SYM_DEG_PER_S, "Gyro temp cal bias", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("tcAccBias", tcAccBias, DATA_TYPE_F32, float, SYM_DEG_PER_S "/C", "Gyro temp cal slope", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("tcPqrSlope", tcPqrSlope, DATA_TYPE_F32, float, SYM_DEG_PER_S "/C", "Gyro temp cal linearity", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_7("tcAccSlope", tcAccSlope, DATA_TYPE_F32, float, SYM_M_PER_S, "Accel temp cal bias", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("tcPqrLinearity", tcPqrLinearity, DATA_TYPE_F32, float, SYM_M_PER_S "/C", "Accel temp cal slope", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("tcAccLinearity", tcAccLinearity, DATA_TYPE_F32, float, SYM_M_PER_S "/C", "Accel temp cal linearity", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("pqr", pqr, DATA_TYPE_F32, float, SYM_DEG_PER_S, "Angular rate error", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_7("acc", acc, DATA_TYPE_F32, float, SYM_M_PER_S, "Acceleration error", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("pqrSigma", pqrSigma, DATA_TYPE_F32, float, SYM_DEG_PER_S, "Angular rate standard deviation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_7("accSigma", accSigma, DATA_TYPE_F32, float, SYM_M_PER_S, "Acceleration standard deviation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_6("testMode", testMode, DATA_TYPE_UINT8, uint8_t, "", "Test Mode: " TOSTRING(BIT_TEST_MODE_SIM_GPS_NOISE) "=GPS noise, " TOSTRING(BIT_TEST_MODE_SERIAL_DRIVER_RX_OVERFLOW) "=Rx overflow, " TOSTRING(BIT_TEST_MODE_SERIAL_DRIVER_TX_OVERFLOW) "=Tx overflow");
    ADD_MAP_7("testVar", testVar, DATA_TYPE_UINT8, uint8_t, "", "Test Mode variable (port number)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("detectedHardwareId", detectedHardwareId, DATA_TYPE_UINT16, uint16_t, "", "Hardware ID detected (see eIsHardwareType) used to validate correct firmware use.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ASSERT_SIZE(totalSize);
}

static void PopulateGpxBitMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(gpx_bit_t, DID_GPX_BIT);
    ADD_MAP_7("results", results, DATA_TYPE_UINT32, uint32_t, "", "GPX BIT test status (see eGPXBit_results)", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("command", command, DATA_TYPE_UINT8, uint8_t, "", "Command (see eGPXBit_CMD)");
    ADD_MAP_6("port", port, DATA_TYPE_UINT8, uint8_t, "", "Port used with the test");
    ADD_MAP_6("testMode", testMode, DATA_TYPE_UINT8, uint8_t, "", "Self-test mode: 102=TxOverflow, 103=RxOverflow (see eGPXBit_test_mode)");
    ADD_MAP_6("state", state, DATA_TYPE_UINT8, uint8_t, "", "Built-in self-test state (see eGPXBit_state)");
    ADD_MAP_7("detectedHardwareId", detectedHardwareId, DATA_TYPE_UINT16, uint16_t, "", "Hardware ID detected (see eIsHardwareType) used to validate correct firmware use.", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("reserved[0]", reserved[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved[1]", reserved[1], DATA_TYPE_UINT8, uint8_t&);
    ASSERT_SIZE(totalSize);
}

static void PopulateSysFaultMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(system_fault_t, DID_SYS_FAULT);
    int flags = DATA_FLAGS_DISPLAY_HEX;
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "Bits: 23:20[flashMigMrk, code, stkOverflow, malloc] 19:16[busFlt, memMng, usageFlt, hardFlt] 7:4[flashMig, softRst] 3:0[bootldrRst, userRst]", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("g1Task", g1Task, DATA_TYPE_UINT32, uint32_t, "", "Active task at fault");
    ADD_MAP_6("g2FileNum", g2FileNum, DATA_TYPE_UINT32, uint32_t, "", "File number at fault");
    ADD_MAP_6("g3LineNum", g3LineNum, DATA_TYPE_UINT32, uint32_t, "", "Line number at fault");
    ADD_MAP_7("g4", g4, DATA_TYPE_UINT32, uint32_t, "", "value at fault", flags);
    ADD_MAP_7("g5Lr", g5Lr, DATA_TYPE_UINT32, uint32_t, "", "Load register at fault", flags);
    ADD_MAP_7("pc", pc, DATA_TYPE_UINT32, uint32_t, "", "program counter at fault", flags);
    ADD_MAP_7("psr", psr, DATA_TYPE_UINT32, uint32_t, "", "program status register at fault", flags);
    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t dataId, string description)
{
    INIT_MAP(imu_t, dataId);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("pqr[0]", I.pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("pqr[1]", I.pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("pqr[2]", I.pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("acc[0]", I.acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc[1]", I.acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc[2]", I.acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ASSERT_SIZE(totalSize);
}

static void PopulateIMU3Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t dataId, string description)
{
    INIT_MAP(imu3_t, dataId);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_8("I0.pqr[0]", I[0].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 1 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I0.pqr[1]", I[0].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 1 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I0.pqr[2]", I[0].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 1 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("I0.acc[0]", I[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 1 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I0.acc[1]", I[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 1 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I0.acc[2]", I[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 1 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_8("I1.pqr[0]", I[1].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 2 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I1.pqr[1]", I[1].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 2 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I1.pqr[2]", I[1].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 2 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("I1.acc[0]", I[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 2 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I1.acc[1]", I[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 2 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I1.acc[2]", I[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 2 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_8("I2.pqr[0]", I[2].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 3 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I2.pqr[1]", I[2].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 3 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("I2.pqr[2]", I[2].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU 3 angular rate.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("I2.acc[0]", I[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 3 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I2.acc[1]", I[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 3 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("I2.acc[2]", I[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU 3 linear acceleration.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateSysParamsMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sys_params_t, DID_SYS_PARAMS);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t, "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t, "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("imuTemp", imuTemp, DATA_TYPE_F32, float, "", "Sys status flags", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("baroTemp", baroTemp, DATA_TYPE_F32, float,  SYM_DEG_C, "IMU temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    ADD_MAP_7("mcuTemp", mcuTemp, DATA_TYPE_F32, float,  SYM_DEG_C, "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    ADD_MAP_7("sysStatus", sysStatus, DATA_TYPE_UINT32, uint32_t,  SYM_DEG_C, "MCU temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    ADD_MAP_6("imuSamplePeriodMs", imuSamplePeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "IMU sample period. Zero disables sensor sampling.");
    ADD_MAP_6("navOutputPeriodMs", navOutputPeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Nav/AHRS filter ouput period.");
    ADD_MAP_7("sensorTruePeriod", sensorTruePeriod, DATA_TYPE_F64, double, "ms", "Nav/AHRS filter update period.", DATA_FLAGS_READ_ONLY);
    ADD_MAP_8("flashCfgChecksum", flashCfgChecksum, DATA_TYPE_UINT32, uint32_t, "us", "Actual sample period relative to GPS PPS.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, 1.0e6);
    ADD_MAP_7("navUpdatePeriodMs", navUpdatePeriodMs, DATA_TYPE_UINT32, uint32_t,  "", "Flash config checksum used for synchronization", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("genFaultCode", genFaultCode, DATA_TYPE_UINT32, uint32_t, "", "General fault code descriptor", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("upTime", upTime, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    ASSERT_SIZE(totalSize);
}

static void PopulateSysSensorsMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sys_sensors_t, DID_SYS_SENSORS);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("temp", temp, DATA_TYPE_F32, float, SYM_DEG_C, "System temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_8("pqr[0]", pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("pqr[1]", pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("pqr[2]", pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_7("acc[0]", acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S, "Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("acc[1]", acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S, "Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("acc[2]", acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S, "Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("mag[0]", mag[0], DATA_TYPE_F32, float&, "", "Magnetometer normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("mag[1]", mag[1], DATA_TYPE_F32, float&, "", "Magnetometer normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("mag[2]", mag[2], DATA_TYPE_F32, float&, "", "Magnetometer normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("bar", bar, DATA_TYPE_F32, float, "kPa", "Barometric pressure", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3 );
    ADD_MAP_7("barTemp", barTemp, DATA_TYPE_F32, float, "m", "Barometer MSL altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2 );
    ADD_MAP_7("mslBar", mslBar, DATA_TYPE_F32, float, "C", "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("humidity", humidity, DATA_TYPE_F32, float, "%rH", "Relative humidity, 0%-100%", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("vin", vin, DATA_TYPE_F32, float, "V", "System input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2 );
    ADD_MAP_7("ana1", ana1, DATA_TYPE_F32, float, "V", "ADC analog 1 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3 );
    ADD_MAP_7("ana3", ana1, DATA_TYPE_F32, float, "V", "ADC analog 3 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3 );
    ADD_MAP_7("ana4", ana1, DATA_TYPE_F32, float, "V", "ADC analog 4 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3 );
    ASSERT_SIZE(totalSize);
}

static void PopulateRMCMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rmc_t, DID_RMC);
    ADD_MAP_5("bits", bits, DATA_TYPE_UINT64, uint64_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("options", options, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ASSERT_SIZE(totalSize);
}

static void PopulateINS1Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ins_1_t, DID_INS_1);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    ADD_MAP_7("week", week, DATA_TYPE_UINT32, uint32_t, "week", "Weeks since Jan 6, 1980", flags );
    ADD_MAP_7("timeOfWeek", timeOfWeek, DATA_TYPE_F64, double, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_8("theta[0]", theta[0], DATA_TYPE_F32, float&, SYM_DEG, "Euler angle - roll",   flags | DATA_FLAGS_ANGLE, C_RAD2DEG); // ERROR_THRESH_ROLLPITCH);
    ADD_MAP_8("theta[1]", theta[1], DATA_TYPE_F32, float&, SYM_DEG, "Euler angle - pitch",  flags | DATA_FLAGS_ANGLE, C_RAD2DEG); // ERROR_THRESH_ROLLPITCH);
    ADD_MAP_8("theta[2]", theta[2], DATA_TYPE_F32, float&, SYM_DEG, "Euler angle - yaw",    flags | DATA_FLAGS_ANGLE, C_RAD2DEG); // ERROR_THRESH_HEADING);
    ADD_MAP_7("uvw[0]", uvw[0], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("uvw[1]", uvw[1], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("uvw[2]", uvw[2], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("ned[0]", ned[0], DATA_TYPE_F32, float&, "m", "North offset from reference LLA", flags);
    ADD_MAP_7("ned[1]", ned[1], DATA_TYPE_F32, float&, "m", "East offset from reference LLA", flags);
    ADD_MAP_7("ned[2]", ned[2], DATA_TYPE_F32, float&, "m", "Down offset from reference LLA", flags);
    ADD_MAP_7("lla[0]", lla[0], DATA_TYPE_F64, double&, SYM_DEG, "WGS84 coordinate - latitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
    ADD_MAP_7("lla[1]", lla[1], DATA_TYPE_F64, double&, SYM_DEG, "WGS84 coordinate - longitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
    ADD_MAP_7("lla[2]", lla[2], DATA_TYPE_F64, double&, "m", "WGS84 coordinate - ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ASSERT_SIZE(totalSize);
}

static void PopulateINS2Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ins_2_t, DID_INS_2);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    ADD_MAP_7("week", week, DATA_TYPE_UINT32, uint32_t, "week", "Weeks since Jan 6, 1980", flags );
    ADD_MAP_7("timeOfWeek", timeOfWeek, DATA_TYPE_F64, double, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("qn2b[0]", qn2b[0], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qn2b[1]", qn2b[1], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qn2b[2]", qn2b[2], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qn2b[3]", qn2b[3], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("uvw[0]", uvw[0], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("uvw[1]", uvw[1], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("uvw[2]", uvw[2], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("lla[0]", lla[0], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "WGS84 latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
    ADD_MAP_7("lla[1]", lla[1], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "WGS84 latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
    ADD_MAP_7("lla[2]", lla[2], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "WGS84 latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
    ASSERT_SIZE(totalSize);
}

static void PopulateINS3Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ins_3_t, DID_INS_3);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    ADD_MAP_7("week", week, DATA_TYPE_UINT32, uint32_t, "week", "Weeks since Jan 6, 1980", flags );
    ADD_MAP_7("timeOfWeek", timeOfWeek, DATA_TYPE_F64, double, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("qn2b[0]", qn2b[0], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qn2b[1]", qn2b[1], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qn2b[2]", qn2b[2], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qn2b[3]", qn2b[3], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to NED: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("uvw[0]", uvw[0], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("uvw[1]", uvw[1], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("uvw[2]", uvw[2], DATA_TYPE_F32, float&, "m/s", "Velocity in body frame", flags);
    ADD_MAP_7("lla[0]", lla[0], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "WGS84 latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
    ADD_MAP_7("lla[1]", lla[1], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "WGS84 latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
    ADD_MAP_7("lla[2]", lla[2], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "WGS84 latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_8);
    ADD_MAP_7("msl", msl, DATA_TYPE_F32, float, "m", "Height above mean sea level (MSL)", flags);
    ASSERT_SIZE(totalSize);
}

static void PopulateINS4Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ins_4_t, DID_INS_4);
    uint32_t flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    ADD_MAP_7("week", week, DATA_TYPE_UINT32, uint32_t, "week", "Weeks since Jan 6, 1980", flags );
    ADD_MAP_7("timeOfWeek", timeOfWeek, DATA_TYPE_F64, double, "s", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4 );
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t,  "", s_insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t,  "", s_hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("qe2b[0]", qe2b[0], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to ECEF: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qe2b[1]", qe2b[1], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to ECEF: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qe2b[2]", qe2b[2], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to ECEF: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("qe2b[3]", qe2b[3], DATA_TYPE_F32, float&, "", "Quaternion body rotation with respect to ECEF: W, X, Y, Z", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("ve[0]", ve[0], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF (earth-centered earth-fixed) frame", flags);
    ADD_MAP_7("ve[1]", ve[1], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF (earth-centered earth-fixed) frame", flags);
    ADD_MAP_7("ve[2]", ve[2], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF (earth-centered earth-fixed) frame", flags);
    ADD_MAP_7("ecef[0]", ecef[0], DATA_TYPE_F64, double&, "m", "Position in ECEF (earth-centered earth-fixed) frame", flags);
    ADD_MAP_7("ecef[1]", ecef[1], DATA_TYPE_F64, double&, "m", "Position in ECEF (earth-centered earth-fixed) frame", flags);
    ADD_MAP_7("ecef[2]", ecef[2], DATA_TYPE_F64, double&, "m", "Position in ECEF (earth-centered earth-fixed) frame", flags);
    ASSERT_SIZE(totalSize);
}

static void PopulateGpsPosMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_pos_t, id);
    ADD_MAP_7("week", week, DATA_TYPE_UINT32, uint32_t, "week", "Weeks since Jan 6, 1980", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("ecef[0]", ecef[0], DATA_TYPE_F64, double&, "m", "Position in ECEF {x,y,z}", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ecef[1]", ecef[1], DATA_TYPE_F64, double&, "m", "Position in ECEF {x,y,z}", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ecef[2]", ecef[2], DATA_TYPE_F64, double&, "m", "Position in ECEF {x,y,z}", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("lla[0]", lla[0], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_7);
    ADD_MAP_7("lla[1]", lla[1], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_7);
    ADD_MAP_7("lla[2]", lla[2], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Latitude, longitude, ellipsoid altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_7);
    ADD_MAP_7("hMSL", hMSL, DATA_TYPE_F32, float, "m", "Meters above sea level", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("hAcc", hAcc, DATA_TYPE_F32, float, "m", "Position horizontal accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vAcc", vAcc, DATA_TYPE_F32, float, "m", "Position vertical accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pDop", pDop, DATA_TYPE_F32, float, "m", "Position dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("cnoMean", cnoMean, DATA_TYPE_F32, float, "dBHz", "Average of non-zero satellite carrier to noise ratios (signal strengths)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    ADD_MAP_7("towOffset", towOffset, DATA_TYPE_F64, double, "sec", "Time sync offset from local clock", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("leapS", leapS, DATA_TYPE_UINT8, uint8_t, "", "GPS leap seconds (GPS-UTC). Receiver's best knowledge of the leap seconds offset from UTC to GPS time.", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("satsUsed", satsUsed, DATA_TYPE_UINT8, uint8_t, "", "Number of satellites used in the solution", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("cnoMeanSigma", cnoMeanSigma, DATA_TYPE_UINT8, uint8_t, "10dBHz", "10x standard deviation of CNO mean over past 5 seconds", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("reserved", reserved, DATA_TYPE_UINT8, uint8_t, "", "", DATA_FLAGS_READ_ONLY);
    ASSERT_SIZE(totalSize);
}

static void PopulateGpsVelMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_vel_t, id);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vel[0]", vel[0], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF {vx,vy,vz} or NED {vN, vE, 0} if status GPS_STATUS_FLAGS_GPS_NMEA_DATA = 0 or 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("vel[1]", vel[1], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF {vx,vy,vz} or NED {vN, vE, 0} if status GPS_STATUS_FLAGS_GPS_NMEA_DATA = 0 or 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("vel[2]", vel[2], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF {vx,vy,vz} or NED {vN, vE, 0} if status GPS_STATUS_FLAGS_GPS_NMEA_DATA = 0 or 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("sAcc", sAcc, DATA_TYPE_F32, float, "m/s", "Speed accuracy", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "GPS status: NMEA input if status flag GPS_STATUS_FLAGS_GPS_NMEA_DATA", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ASSERT_SIZE(totalSize);
}

static void PopulateGpsSatMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_sat_t, id);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("numSats", numSats, DATA_TYPE_UINT32, uint32_t, "", "Number of satellites in sky", DATA_FLAGS_READ_ONLY);

#define ADD_MAP_SAT_INFO(n) \
    ADD_MAP_4("sat" #n ".gnssId",    sat[n].gnssId,    DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sat" #n ".svId",      sat[n].svId,      DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sat" #n ".elev",      sat[n].elev,      DATA_TYPE_INT8,  int8_t); \
    ADD_MAP_4("sat" #n ".azim",      sat[n].azim,      DATA_TYPE_INT16, int16_t); \
    ADD_MAP_4("sat" #n ".cno",       sat[n].cno,       DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sat" #n ".status",    sat[n].status,    DATA_TYPE_UINT16, uint16_t);

    ADD_MAP_SAT_INFO(0);
    ADD_MAP_SAT_INFO(1);
    ADD_MAP_SAT_INFO(2);
    ADD_MAP_SAT_INFO(3);
    ADD_MAP_SAT_INFO(4);
    ADD_MAP_SAT_INFO(5);
    ADD_MAP_SAT_INFO(6);
    ADD_MAP_SAT_INFO(7);
    ADD_MAP_SAT_INFO(8);
    ADD_MAP_SAT_INFO(9);

    ADD_MAP_SAT_INFO(10);
    ADD_MAP_SAT_INFO(11);
    ADD_MAP_SAT_INFO(12);
    ADD_MAP_SAT_INFO(13);
    ADD_MAP_SAT_INFO(14);
    ADD_MAP_SAT_INFO(15);
    ADD_MAP_SAT_INFO(16);
    ADD_MAP_SAT_INFO(17);
    ADD_MAP_SAT_INFO(18);
    ADD_MAP_SAT_INFO(19);

    ADD_MAP_SAT_INFO(20);
    ADD_MAP_SAT_INFO(21);
    ADD_MAP_SAT_INFO(22);
    ADD_MAP_SAT_INFO(23);
    ADD_MAP_SAT_INFO(24);
    ADD_MAP_SAT_INFO(25);
    ADD_MAP_SAT_INFO(26);
    ADD_MAP_SAT_INFO(27);
    ADD_MAP_SAT_INFO(28);
    ADD_MAP_SAT_INFO(29);

    ADD_MAP_SAT_INFO(30);
    ADD_MAP_SAT_INFO(31);
    ADD_MAP_SAT_INFO(32);
    ADD_MAP_SAT_INFO(33);
    ADD_MAP_SAT_INFO(34);
    ADD_MAP_SAT_INFO(35);
    ADD_MAP_SAT_INFO(36);
    ADD_MAP_SAT_INFO(37);
    ADD_MAP_SAT_INFO(38);
    ADD_MAP_SAT_INFO(39);

    ADD_MAP_SAT_INFO(40);
    ADD_MAP_SAT_INFO(41);
    ADD_MAP_SAT_INFO(42);
    ADD_MAP_SAT_INFO(43);
    ADD_MAP_SAT_INFO(44);
    ADD_MAP_SAT_INFO(45);
    ADD_MAP_SAT_INFO(46);
    ADD_MAP_SAT_INFO(47);
    ADD_MAP_SAT_INFO(48);
    ADD_MAP_SAT_INFO(49);
    ASSERT_SIZE(totalSize);
}

static void PopulateGpsSigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_sig_t, id);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("numSigs", numSigs, DATA_TYPE_UINT32, uint32_t, "", "Number of signals in sky", DATA_FLAGS_READ_ONLY);

#define ADD_MAP_SAT_SIG(n) \
    ADD_MAP_4("sig" #n ".gnssId",    sig[n].gnssId,    DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".svId",      sig[n].svId,      DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".sigId",     sig[n].sigId,     DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".cno",       sig[n].cno,       DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".quality",   sig[n].quality,   DATA_TYPE_UINT8, uint8_t); \
    ADD_MAP_4("sig" #n ".status",    sig[n].status,    DATA_TYPE_UINT16, uint16_t);

    ADD_MAP_SAT_SIG(0);
    ADD_MAP_SAT_SIG(1);
    ADD_MAP_SAT_SIG(2);
    ADD_MAP_SAT_SIG(3);
    ADD_MAP_SAT_SIG(4);
    ADD_MAP_SAT_SIG(5);
    ADD_MAP_SAT_SIG(6);
    ADD_MAP_SAT_SIG(7);
    ADD_MAP_SAT_SIG(8);
    ADD_MAP_SAT_SIG(9);

    ADD_MAP_SAT_SIG(10);
    ADD_MAP_SAT_SIG(11);
    ADD_MAP_SAT_SIG(12);
    ADD_MAP_SAT_SIG(13);
    ADD_MAP_SAT_SIG(14);
    ADD_MAP_SAT_SIG(15);
    ADD_MAP_SAT_SIG(16);
    ADD_MAP_SAT_SIG(17);
    ADD_MAP_SAT_SIG(18);
    ADD_MAP_SAT_SIG(19);

    ADD_MAP_SAT_SIG(20);
    ADD_MAP_SAT_SIG(21);
    ADD_MAP_SAT_SIG(22);
    ADD_MAP_SAT_SIG(23);
    ADD_MAP_SAT_SIG(24);
    ADD_MAP_SAT_SIG(25);
    ADD_MAP_SAT_SIG(26);
    ADD_MAP_SAT_SIG(27);
    ADD_MAP_SAT_SIG(28);
    ADD_MAP_SAT_SIG(29);

    ADD_MAP_SAT_SIG(30);
    ADD_MAP_SAT_SIG(31);
    ADD_MAP_SAT_SIG(32);
    ADD_MAP_SAT_SIG(33);
    ADD_MAP_SAT_SIG(34);
    ADD_MAP_SAT_SIG(35);
    ADD_MAP_SAT_SIG(36);
    ADD_MAP_SAT_SIG(37);
    ADD_MAP_SAT_SIG(38);
    ADD_MAP_SAT_SIG(39);

    ADD_MAP_SAT_SIG(40);
    ADD_MAP_SAT_SIG(41);
    ADD_MAP_SAT_SIG(42);
    ADD_MAP_SAT_SIG(43);
    ADD_MAP_SAT_SIG(44);
    ADD_MAP_SAT_SIG(45);
    ADD_MAP_SAT_SIG(46);
    ADD_MAP_SAT_SIG(47);
    ADD_MAP_SAT_SIG(48);
    ADD_MAP_SAT_SIG(49);
    ASSERT_SIZE(totalSize);
}

static void PopulateMagnetometerMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t did)
{
    INIT_MAP(magnetometer_t, did);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("mag[0]", mag[0], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[1]", mag[1], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[2]", mag[2], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateBarometerMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(barometer_t, DID_BAROMETER);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("bar", bar, DATA_TYPE_F32, float, "kPa", "Barometric pressure", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mslBar", mslBar, DATA_TYPE_F32, float, "m", "MSL altitude from barometric pressure sensor", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("barTemp", barTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("humidity", humidity, DATA_TYPE_F32, float, "%rH", "Relative humidity, 0%-100%", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t did, string description)
{
    INIT_MAP(pimu_t, did);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("dt", dt, DATA_TYPE_F32, float, "s", "Integration period.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_8("theta[0]", theta[0], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("theta[1]", theta[1], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("theta[2]", theta[2], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_7("vel[0]", vel[0], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("vel[1]", vel[1], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("vel[2]", vel[2], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.  " + description, DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMagMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(pimu_mag_t, DID_PIMU_MAG);
    ADD_MAP_7("imutime", pimu.time, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("theta[0]", pimu.theta[0], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("theta[1]", pimu.theta[1], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("theta[2]", pimu.theta[2], DATA_TYPE_F32, float&, SYM_DEG, "IMU delta theta coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_7("vel[0]", pimu.vel[0], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("vel[1]", pimu.vel[1], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("vel[2]", pimu.vel[2], DATA_TYPE_F32, float&, "m/s", "IMU delta velocity coning and sculling integrals in body/IMU frame.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("dt", pimu.dt, DATA_TYPE_F32, float, "s", "Integration period.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imustatus", pimu.status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("magtime", mag.time, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("mag[0]", mag.mag[0], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[1]", mag.mag[1], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[2]", mag.mag[2], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMagnetometerMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(imu_mag_t, DID_IMU_MAG);
    ADD_MAP_7("imutime", imu.time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("pqr[0]", imu.I.pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("pqr[1]", imu.I.pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("pqr[2]", imu.I.pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "IMU Angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_7("acc[0]", imu.I.acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc[1]", imu.I.acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc[2]", imu.I.acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "IMU Linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("imustatus", imu.status, DATA_TYPE_UINT32, uint32_t, "", s_imuStatusDescription, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("magtime", mag.time, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("mag[0]", mag.mag[0], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[1]", mag.mag[1], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag[2]", mag.mag[2], DATA_TYPE_F32, float&, "", "Normalized gauss", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateInfieldCalMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(infield_cal_t, DID_INFIELD_CAL);
    ADD_MAP_6("state", state, DATA_TYPE_UINT32, uint32_t, "", "0=off, init[1=IMU, 2=gyro, 3=accel], init align INS[4, 5=+IMU, 6=+gyro, 7=+accel], 8=sample, 9=finish (see eInfieldCalState)");
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "Infield cal status (see eInfieldCalStatus)", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("sampleTimeMs", sampleTimeMs, DATA_TYPE_UINT32, uint32_t, "ms", "Duration of IMU sample averaging. sampleTimeMs = 0 means \"imu\" member contains the IMU bias from flash.");

    ADD_MAP_8("imu[0].pqr[0]", imu[0].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[0].pqr[1]", imu[0].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[0].pqr[2]", imu[0].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("imu[0].acc[0]", imu[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[0].acc[1]", imu[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[0].acc[2]", imu[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("imu[1].pqr[0]", imu[1].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[1].pqr[1]", imu[1].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[1].pqr[2]", imu[1].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("imu[1].acc[0]", imu[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[1].acc[1]", imu[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[1].acc[2]", imu[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("imu[2].pqr[0]", imu[2].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[2].pqr[1]", imu[2].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("imu[2].pqr[2]", imu[2].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Sampled angular rate.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("imu[2].acc[0]", imu[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[2].acc[1]", imu[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("imu[2].acc[2]", imu[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Sampled linear acceleration.  IMU bias when state=INFIELD_CAL_STATE_SAVED_AND_FINISHED", DATA_FLAGS_FIXED_DECIMAL_4);

    ADD_MAP_7("calData[0].down.dev[0].acc[0]", calData[0].down.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[0].acc[1]", calData[0].down.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[0].acc[2]", calData[0].down.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[1].acc[0]", calData[0].down.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[1].acc[1]", calData[0].down.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[1].acc[2]", calData[0].down.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[2].acc[0]", calData[0].down.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[2].acc[1]", calData[0].down.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].down.dev[2].acc[2]", calData[0].down.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[0].down.yaw", calData[0].down.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP_7("calData[0].up.dev[0].acc[0]", calData[0].up.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[0].acc[1]", calData[0].up.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[0].acc[2]", calData[0].up.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[1].acc[0]", calData[0].up.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[1].acc[1]", calData[0].up.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[1].acc[2]", calData[0].up.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[2].acc[0]", calData[0].up.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[2].acc[1]", calData[0].up.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[0].up.dev[2].acc[2]", calData[0].up.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[0].up.yaw", calData[0].up.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);

    ADD_MAP_7("calData[1].down.dev[0].acc[0]", calData[1].down.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[0].acc[1]", calData[1].down.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[0].acc[2]", calData[1].down.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[1].acc[0]", calData[1].down.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[1].acc[1]", calData[1].down.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[1].acc[2]", calData[1].down.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[2].acc[0]", calData[1].down.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[2].acc[1]", calData[1].down.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].down.dev[2].acc[2]", calData[1].down.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[1].down.yaw", calData[1].down.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP_7("calData[1].up.dev[0].acc[0]", calData[1].up.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[0].acc[1]", calData[1].up.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[0].acc[2]", calData[1].up.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[1].acc[0]", calData[1].up.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[1].acc[1]", calData[1].up.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[1].acc[2]", calData[1].up.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[2].acc[0]", calData[1].up.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[2].acc[1]", calData[1].up.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[1].up.dev[2].acc[2]", calData[1].up.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[1].up.yaw", calData[1].up.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);

    ADD_MAP_7("calData[2].down.dev[0].acc[0]", calData[2].down.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[0].acc[1]", calData[2].down.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[0].acc[2]", calData[2].down.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[1].acc[0]", calData[2].down.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[1].acc[1]", calData[2].down.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[1].acc[2]", calData[2].down.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[2].acc[0]", calData[2].down.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[2].acc[1]", calData[2].down.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].down.dev[2].acc[2]", calData[2].down.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[2].down.yaw", calData[2].down.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP_7("calData[2].up.dev[0].acc[0]", calData[2].up.dev[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[0].acc[1]", calData[2].up.dev[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[0].acc[2]", calData[2].up.dev[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[1].acc[0]", calData[2].up.dev[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[1].acc[1]", calData[2].up.dev[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[1].acc[2]", calData[2].up.dev[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[2].acc[0]", calData[2].up.dev[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[2].acc[1]", calData[2].up.dev[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("calData[2].up.dev[2].acc[2]", calData[2].up.dev[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Linear acceleration", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_8("calData[2].up.yaw", calData[2].up.yaw, DATA_TYPE_F32, float, SYM_DEG, "Yaw angle. >=999 means two samples have been averaged.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ASSERT_SIZE(totalSize);
}

static void PopulateWheelEncoderMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(wheel_encoder_t, DID_WHEEL_ENCODER);
    ADD_MAP_7("timeOfWeek", timeOfWeek, DATA_TYPE_F64, double, "s", "Time of measurement wrt current week", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_8("theta_l", theta_l, DATA_TYPE_F32, float, SYM_DEG, "Left wheel angle", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("omega_l", omega_l, DATA_TYPE_F32, float, SYM_DEG_PER_S, "Left wheel angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_8("theta_r", theta_r, DATA_TYPE_F32, float, SYM_DEG, "Right wheel angle", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("omega_r", omega_r, DATA_TYPE_F32, float, SYM_DEG_PER_S, "Right wheel angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
    ADD_MAP_6("wrap_count_l", wrap_count_l, DATA_TYPE_UINT32, uint32_t, "", "");
    ADD_MAP_6("wrap_count_r", wrap_count_r, DATA_TYPE_UINT32, uint32_t, "", "");
    ASSERT_SIZE(totalSize);
}

static void PopulateGroundVehicleMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ground_vehicle_t, DID_GROUND_VEHICLE);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("mode", mode, DATA_TYPE_UINT32, uint32_t, "", "1=learning; Commands[2=start, 3=resume, 4=clear&start, 5=stop&save, 6=cancel]");
    ADD_MAP_5("wheelConfig.bits", wheelConfig.bits, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], DATA_TYPE_F32, float&, "rad", "Euler angle rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], DATA_TYPE_F32, float&, "rad", "Euler angle rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], DATA_TYPE_F32, float&, "rad", "Euler angle rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], DATA_TYPE_F32, float&, "rad", "Standard deviation of Euler angles describing rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], DATA_TYPE_F32, float&, "rad", "Standard deviation of Euler angles describing rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], DATA_TYPE_F32, float&, "rad", "Standard deviation of Euler angles describing rotation from imu (body) to wheel frame (center of non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], DATA_TYPE_F32, float&, "m", "Translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], DATA_TYPE_F32, float&, "m", "Translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], DATA_TYPE_F32, float&, "m", "Translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], DATA_TYPE_F32, float&, "m", "Standard deviation of translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], DATA_TYPE_F32, float&, "m", "Standard deviation of translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], DATA_TYPE_F32, float&, "m", "Standard deviation of translation from imu (body) to wheel frame origin (center of non-steering axle), expressed in imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("wheelConfig.track_width", wheelConfig.track_width, DATA_TYPE_F32, float, "m", "Distance between left and right wheels");
    ADD_MAP_6("wheelConfig.radius", wheelConfig.radius, DATA_TYPE_F32, float, "m", "Wheel radius");
    ASSERT_SIZE(totalSize);
}

static void PopulateConfigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(system_command_t, DID_SYS_CMD);
    ADD_MAP_6("command", command, DATA_TYPE_UINT32, uint32_t, "", "99=software reset, 5=zero sensors");
    ADD_MAP_6("invCommand", invCommand, DATA_TYPE_UINT32, uint32_t, "", "Bitwise inverse of command (-command - 1) required to process command.");
    ASSERT_SIZE(totalSize);
}

static void PopulateFlashConfigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(nvm_flash_cfg_t, DID_FLASH_CONFIG);
    string str;
    ADD_MAP_6("startupImuDtMs", startupImuDtMs, DATA_TYPE_UINT32, uint32_t, "ms", "IMU sample (system input data) period set on startup. Cannot be larger than startupInsDtMs. Zero disables sensor/IMU sampling.");
    ADD_MAP_6("startupNavDtMs", startupNavDtMs, DATA_TYPE_UINT32, uint32_t, "ms", "GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max).");
    ADD_MAP_6("startupGPSDtMs", startupGPSDtMs, DATA_TYPE_UINT32, uint32_t, "ms", "Nav filter (system output data) update period set on startup. 1ms min (1KHz max).");
    ADD_MAP_6("ser0BaudRate", ser0BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 0 baud rate");
    ADD_MAP_6("ser1BaudRate", ser1BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 1 baud rate");
    ADD_MAP_6("ser2BaudRate", ser2BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 2 baud rate");
    str = "rotation from INS Sensor Frame to Intermediate Output Frame.  Order applied: yaw, pitch, roll.";
    ADD_MAP_8("insRotation[0]", insRotation[0], DATA_TYPE_F32, float&, SYM_DEG, "Roll "  + str, 0, C_RAD2DEG);
    ADD_MAP_8("insRotation[1]", insRotation[1], DATA_TYPE_F32, float&, SYM_DEG, "Pitch " + str, 0, C_RAD2DEG);
    ADD_MAP_8("insRotation[2]", insRotation[2], DATA_TYPE_F32, float&, SYM_DEG, "Yaw "   + str, 0, C_RAD2DEG);
    str = "offset from Intermediate Output Frame to INS Output Frame.  INS rotation is applied before this.";
    ADD_MAP_6("insOffset[0]", insOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("insOffset[1]", insOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("insOffset[2]", insOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);
    str = "offset from Sensor Frame origin to GPS1 antenna.";
    ADD_MAP_6("gps1AntOffset[0]", gps1AntOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("gps1AntOffset[1]", gps1AntOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("gps1AntOffset[2]", gps1AntOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);
    str = "offset from Sensor Frame origin to GPS2 antenna.";
    ADD_MAP_6("gps2AntOffset[0]", gps2AntOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("gps2AntOffset[1]", gps2AntOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("gps2AntOffset[2]", gps2AntOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);

    ADD_MAP_8("gpsTimeSyncPeriodMs", gpsTimeSyncPeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "GPS time synchronization pulse period.", 0, 1.0);
    ADD_MAP_8("gpsTimeUserDelay", gpsTimeUserDelay, DATA_TYPE_F32, float, "s", "User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.", DATA_FLAGS_FIXED_DECIMAL_3, 1.0);
    ADD_MAP_8("gpsMinimumElevation", gpsMinimumElevation, DATA_TYPE_F32, float, SYM_DEG, "GPS minimum elevation of a satellite above the horizon to be used in the solution.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP_8("gnssSatSigConst", gnssSatSigConst, DATA_TYPE_UINT16, uint16_t, "", "GNSS constellations used. 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS (see eGnssSatSigConst)", DATA_FLAGS_DISPLAY_HEX, 1.0);

    ADD_MAP_8("dynamicModel", dynamicModel, DATA_TYPE_UINT8, uint8_t, "", "0:port, 2:stationary, 3:walk, 4:ground vehicle, 5:sea, 6:air<1g, 7:air<2g, 8:air<4g, 9:wrist", 0, 1.0);
    str = "AutobaudOff [0x1=Ser0, 0x2=Ser1], 0x4=AutoMagRecal, 0x8=DisableMagDecEst, ";
    str += "0x10=DisableLeds, ";
    str += "0x100=1AxisMagRecal, ";
    str += "FusionOff [0x1000=Mag, 0x2000=Baro, 0x4000=GPS], ";
    str += "0x10000=enZeroVel, 0x100000=enNavStrobeOutput";
    ADD_MAP_7("sysCfgBits", sysCfgBits, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_DISPLAY_HEX);
    str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    ADD_MAP_7("RTKCfgBits", RTKCfgBits, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("ioConfig", ioConfig, DATA_TYPE_UINT32, uint32_t, "", "(see enum eIoConfig) IMU disable: 0x1000000,0x20000000,0x4000000", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("platformConfig", platformConfig, DATA_TYPE_UINT32, uint32_t, "", "Hardware platform (IMX carrier board, i.e. RUG, EVB, IG) configuration bits (see ePlatformConfig)", DATA_FLAGS_DISPLAY_HEX);
    str =  "Gyr FS (deg/s) 0x7:[0=250, 1=500, 2=1000, 3=2000, 4=4000], ";
    str += "Acc FS 0x30:[0=2g, 1=4g, 2=8g, 3=16g], ";
    str += "Gyr DLPF (Hz) 0x0F00:[0=250, 1=184, 2=92, 3=41, 4=20, 5=10, 6=5], ";
    str += "Acc DLPF (Hz) 0xF000:[0=218, 1=218, 2=99, 3=45, 4=21, 5=10, 6=5], ";
    ADD_MAP_7("sensorConfig", sensorConfig, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_DISPLAY_HEX);

    ADD_MAP_7("refLla[0]", refLla[0], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Reference latitude, longitude, ellipsoid altitude for north east down (NED) calculations", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("refLla[1]", refLla[1], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Reference latitude, longitude, ellipsoid altitude for north east down (NED) calculations", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("refLla[2]", refLla[2], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Reference latitude, longitude, ellipsoid altitude for north east down (NED) calculations", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("lastLla[0]", lastLla[0], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Last latitude, longitude, ellipsoid altitude. (Aids GPS startup)", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("lastLla[1]", lastLla[1], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Last latitude, longitude, ellipsoid altitude. (Aids GPS startup)", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("lastLla[2]", lastLla[2], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Last latitude, longitude, ellipsoid altitude. (Aids GPS startup)", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_6("lastLlaTimeOfWeekMs", lastLlaTimeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Last LLA update time since Sunday morning");
    ADD_MAP_6("lastLlaWeek", lastLlaWeek, DATA_TYPE_UINT32, uint32_t, "week", "Last LLA update Weeks since Jan 6, 1980");
    ADD_MAP_6("lastLlaUpdateDistance", lastLlaUpdateDistance, DATA_TYPE_F32, float, "m", "Distance between current and last LLA that triggers an update of lastLLA)");
    ADD_MAP_8("magDeclination", magDeclination, DATA_TYPE_F32, float,  SYM_DEG, "Magnetic north declination (heading offset from true north)", DATA_FLAGS_FIXED_DECIMAL_2, C_RAD2DEG);
	ADD_MAP_6("magInterferenceThreshold", magInterferenceThreshold, DATA_TYPE_F32, float, "", "Magnetometer interference sensitivity threshold. Typical range is 2-10 (3 default) and 1000 to disable mag interference detection.");
	ADD_MAP_6("magCalibrationQualityThreshold", magCalibrationQualityThreshold, DATA_TYPE_F32, float, "", "Magnetometer calibration quality sensitivity threshold. Typical range is 10-20 (10 default) and 1000 to disable mag calibration quality check, forcing it to be always good.");
    str = "rotation from INS Sensor Frame to Intermediate Output Frame.  Order applied: heading, pitch, roll.";
    ADD_MAP_8("zeroVelRotation[0]", zeroVelRotation[0], DATA_TYPE_F32, float&, SYM_DEG, "Roll " + str, 0, C_RAD2DEG);
    ADD_MAP_8("zeroVelRotation[1]", zeroVelRotation[1], DATA_TYPE_F32, float&, SYM_DEG, "Pitch " + str, 0, C_RAD2DEG);
    ADD_MAP_8("zeroVelRotation[2]", zeroVelRotation[2], DATA_TYPE_F32, float&, SYM_DEG, "Yaw " + str, 0, C_RAD2DEG);
    str = "offset from Intermediate Output Frame to INS Output Frame.  INS rotation is applied before this.";
    ADD_MAP_6("zeroVelOffset[0]", zeroVelOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("zeroVelOffset[1]", zeroVelOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("zeroVelOffset[2]", zeroVelOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);
    ADD_MAP_7("wheelConfig.bits", wheelConfig.bits, DATA_TYPE_UINT32, uint32_t, "", "Wheel encoder config bits: 0x1 enable encoders, 0x2 kinematic constraints", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], DATA_TYPE_F32, float&, "rad", "Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], DATA_TYPE_F32, float&, "rad", "Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], DATA_TYPE_F32, float&, "rad", "Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], DATA_TYPE_F32, float&, "m", "Standard deviation of Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], DATA_TYPE_F32, float&, "m", "Standard deviation of Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], DATA_TYPE_F32, float&, "m", "Standard deviation of Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], DATA_TYPE_F32, float&, "m", "Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], DATA_TYPE_F32, float&, "m", "Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], DATA_TYPE_F32, float&, "m", "Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], DATA_TYPE_F32, float&, "rad", "Standard deviation of translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], DATA_TYPE_F32, float&, "rad", "Standard deviation of translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], DATA_TYPE_F32, float&, "rad", "Standard deviation of translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("wheelConfig.track_width", wheelConfig.track_width, DATA_TYPE_F32, float, "m", "Distance between the left and right wheels");
    ADD_MAP_6("wheelConfig.radius", wheelConfig.radius, DATA_TYPE_F32, float, "m", "Wheel radius");
    ADD_MAP_4("debug", debug, DATA_TYPE_UINT8, uint8_t);

    // Keep at end
    ADD_MAP_6("size", size, DATA_TYPE_UINT32, uint32_t, "", "Flash group size. Set to 1 to reset this flash group.");
    ADD_MAP_6("checksum", checksum, DATA_TYPE_UINT32, uint32_t, "", "Flash checksum");
    ADD_MAP_6("key", key, DATA_TYPE_UINT32, uint32_t, "", "Flash key");
    ASSERT_SIZE(totalSize);
}

/**
 * Maps DID_EVENT for SDK
*/
static void PopulateISEventMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(did_event_t, DID_EVENT);
    ADD_MAP_7("Time stamp of message (System Up seconds)", time, DATA_TYPE_F64, double, "sec", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    ADD_MAP_6("Senders serial number", senderSN, DATA_TYPE_UINT32, uint32_t, "", "");
    ADD_MAP_6("Sender hardware type", senderHdwId, DATA_TYPE_UINT16, uint16_t, "", "Hardware: 0=Host, 1=uINS, 2=EVB, 3=IMX, 4=GPX (see eIsHardwareType)");
    ADD_MAP_6("Message ID", msgTypeID, DATA_TYPE_UINT16, uint16_t, "", "(see eEventMsgTypeID)");
    ADD_MAP_6("Priority", priority, DATA_TYPE_UINT8, uint8_t, "", "see eEventPriority");
    ADD_MAP_6("Length", length, DATA_TYPE_UINT16, uint16_t, "bytes", "");
    ADD_MAP_7("data", data, DATA_TYPE_STRING, uint8_t[MEMBERSIZE(MAP_TYPE, data)], "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("Reserved 8 bit", res8, DATA_TYPE_UINT8, uint8_t, "", "");
    ASSERT_SIZE(totalSize);
}

static void PopulateGpxFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(gpx_flash_cfg_t, DID_GPX_FLASH_CFG);
    string str;
    ADD_MAP_6("ser0BaudRate", ser0BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 0 baud rate");
    ADD_MAP_6("ser1BaudRate", ser1BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 1 baud rate");
    ADD_MAP_6("ser2BaudRate", ser2BaudRate, DATA_TYPE_UINT32, uint32_t, "bps", "Serial port 2 baud rate");
    ADD_MAP_6("startupGPSDtMs", startupGPSDtMs, DATA_TYPE_UINT32, uint32_t, "ms", "GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max).");
    str = "offset from Sensor Frame origin to GPS1 antenna.";
    ADD_MAP_6("gps1AntOffset[0]", gps1AntOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("gps1AntOffset[1]", gps1AntOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("gps1AntOffset[2]", gps1AntOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);
    str = "offset from Sensor Frame origin to GPS2 antenna.";
    ADD_MAP_6("gps2AntOffset[0]", gps2AntOffset[0], DATA_TYPE_F32, float&, "m", "X " + str);
    ADD_MAP_6("gps2AntOffset[1]", gps2AntOffset[1], DATA_TYPE_F32, float&, "m", "Y " + str);
    ADD_MAP_6("gps2AntOffset[2]", gps2AntOffset[2], DATA_TYPE_F32, float&, "m", "Z " + str);
    ADD_MAP_7("gnssSatSigConst", gnssSatSigConst, DATA_TYPE_UINT16, uint16_t, "", "GNSS constellations used. 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS (see eGnssSatSigConst)", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("dynamicModel", dynamicModel, DATA_TYPE_UINT8, uint8_t, "", "0:port, 2:stationary, 3:walk, 4:ground vehicle, 5:sea, 6:air<1g, 7:air<2g, 8:air<4g, 9:wrist");
    ADD_MAP_6("debug", debug, DATA_TYPE_UINT8, uint8_t, "", "Reserved");
    ADD_MAP_6("gpsTimeSyncPeriodMs", gpsTimeSyncPeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "GPS time synchronization pulse period.");
    ADD_MAP_7("gpsTimeUserDelay", gpsTimeUserDelay, DATA_TYPE_F32, float, "s", "User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_8("gpsMinimumElevation", gpsMinimumElevation, DATA_TYPE_F32, float, SYM_DEG, "GPS minimum elevation of a satellite above the horizon to be used in the solution.", DATA_FLAGS_FIXED_DECIMAL_1, C_RAD2DEG);
    str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    ADD_MAP_7("RTKCfgBits", RTKCfgBits, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_DISPLAY_HEX);

    // Keep at end
    ADD_MAP_6("size", size, DATA_TYPE_UINT32, uint32_t, "", "Flash group size. Set to 1 to reset this flash group.");
    ADD_MAP_6("checksum", checksum, DATA_TYPE_UINT32, uint32_t, "", "Flash checksum");
    ADD_MAP_6("key", key, DATA_TYPE_UINT32, uint32_t, "", "Flash key");
    ASSERT_SIZE(totalSize);
}

static void PopulateGpxStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(gpx_status_t, DID_GPX_STATUS);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "(see eGpxStatus)", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcBitsSer0", grmcBitsSer0, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC bit Serial 0", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcBitsSer1", grmcBitsSer1, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC bit Serial 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcBitsSer2", grmcBitsSer2, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC bit Serial 2", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcBitsUSB", grmcBitsUSB, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC bit USB.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
 
    ADD_MAP_7("grmcNMEABitsSer0", grmcNMEABitsSer0, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC NMEA bit Serial 0", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcNMEABitsSer1", grmcNMEABitsSer1, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC NMEA bit Serial 1", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcNMEABitsSer2", grmcNMEABitsSer2, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC NMEA bit Serial 2", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("grmcNMEABitsUSB", grmcNMEABitsUSB, DATA_TYPE_UINT64, uint64_t, "", "GPX RMC NMEA bit USB.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
 
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t, "", "Hardware status eHdwStatusFlags", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("mcuTemp", mcuTemp, DATA_TYPE_F32, float, SYM_DEG_C, "MCU temperature", DATA_FLAGS_FIXED_DECIMAL_3 | DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("navOutputPeriodMs", navOutputPeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Nav output period (ms)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("flashCfgChecksum", flashCfgChecksum, DATA_TYPE_UINT32, uint32_t, "", "Flash config validation", DATA_FLAGS_READ_ONLY);
 
    string str = "Rover [0x1=G1, 0x2=G2], 0x8=GCompass, ";
    str += "BaseOutG1 [0x10=UbxS0, 0x20=UbxS1, 0x40=RtcmS0, 0x80=RtcmS1], ";
    str += "BaseOutG2 [0x100=UbxS0, 0x200=UbxS1, 0x400=RtcmS0, 0x800=RtcmS1], ";
    str += "0x1000=MovingBasePos, 0x4000=SameHdwRvrBase";
    ADD_MAP_7("rtkMode", rtkMode, DATA_TYPE_UINT32, uint32_t, "", str, DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("gnss1RunState", gnss1RunState, DATA_TYPE_UINT32, uint32_t, "", "GNSS1 status (see RunState)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("gnss2RunState", gnss2RunState, DATA_TYPE_UINT32, uint32_t, "", "GNSS2 status (see RunState)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("SourcePort", gpxSourcePort, DATA_TYPE_UINT8, uint8_t, "", "Port", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("upTime", upTime, DATA_TYPE_F64, double, "s", "Local time since startup.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    ASSERT_SIZE(totalSize);
}

static void PopulateEvbStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_status_t, DID_EVB_STATUS);
    ADD_MAP_7("week", week, DATA_TYPE_UINT32, uint32_t, "week", "Weeks since Jan 6, 1980", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("firmwareVer[0]", firmwareVer[0], DATA_TYPE_UINT8, uint8_t&, "", "Firmware version", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("firmwareVer[1]", firmwareVer[1], DATA_TYPE_UINT8, uint8_t&, "", "Firmware version", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("firmwareVer[2]", firmwareVer[2], DATA_TYPE_UINT8, uint8_t&, "", "Firmware version", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("firmwareVer[3]", firmwareVer[3], DATA_TYPE_UINT8, uint8_t&, "", "Firmware version", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("evbStatus", evbStatus, DATA_TYPE_UINT32, uint32_t, "", "EVB status bits", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("loggerMode", loggerMode, DATA_TYPE_UINT32, uint32_t, "", TOSTRING(EVB2_LOG_CMD_START) "=start, " TOSTRING(EVB2_LOG_CMD_STOP) "=stop");
    ADD_MAP_6("loggerElapsedTimeMs", loggerElapsedTimeMs, DATA_TYPE_UINT32, uint32_t, "ms", "Elapsed time of the current data log.");
    ADD_MAP_7("wifiIpAddr", wifiIpAddr, DATA_TYPE_UINT32, uint32_t, "", "WiFi IP address", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("sysCommand", sysCommand, DATA_TYPE_UINT32, uint32_t, "", "99=software reset, 1122334455=unlock, 1357924681=chip erase");
    ADD_MAP_7("towOffset", towOffset, DATA_TYPE_F64, double, "sec", "Time sync offset from local clock", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ASSERT_SIZE(totalSize);
}

static void PopulateEvbFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_flash_cfg_t, DID_EVB_FLASH_CFG);
    ADD_MAP_6("cbPreset", cbPreset, DATA_TYPE_UINT8, uint8_t, "", 
        TOSTRING(EVB2_CB_PRESET_RS232) "=Wireless Off, " 
        TOSTRING(EVB2_CB_PRESET_RS232_XBEE) "=XBee On, " 
        TOSTRING(EVB2_CB_PRESET_RS422_WIFI) "=WiFi On & RS422, " 
        TOSTRING(EVB2_CB_PRESET_USB_HUB_RS232) "=USB hub, " 
        TOSTRING(EVB2_CB_PRESET_USB_HUB_RS422) "=USB hub w/ RS422");
    ADD_MAP_4("reserved1[0]", reserved1[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved1[1]", reserved1[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved1[2]", reserved1[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_7("cbf[0]", cbf[0], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[1]", cbf[1], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[2]", cbf[2], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[3]", cbf[3], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[4]", cbf[4], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[5]", cbf[5], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[6]", cbf[6], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[7]", cbf[7], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[8]", cbf[8], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbf[9]", cbf[9], DATA_TYPE_UINT32, uint32_t&, "", "Communications bridge forwarding", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("cbOptions", cbOptions, DATA_TYPE_UINT32, uint32_t, "", "Communications bridge options (see eEvb2ComBridgeOptions)", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("uinsComPort", uinsComPort, DATA_TYPE_UINT8, uint8_t, "", "EVB port for uINS communications and SD card logging. 0=uINS0 (default), 1=uINS1, SP330=5, 6=GPIO_H8 (use eEvb2CommPorts)");
    ADD_MAP_6("uinsAuxPort", uinsAuxPort, DATA_TYPE_UINT8, uint8_t, "", "EVB port for uINS aux com and RTK corrections. 0=uINS0, 1=uINS1 (default), 5=SP330, 6=GPIO_H8 (use eEvb2CommPorts)");
    ADD_MAP_7("portOptions", portOptions, DATA_TYPE_UINT32, uint32_t, "", "EVB port options:  0x1=radio RTK filter ", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("bits", bits, DATA_TYPE_UINT32, uint32_t, "", "Configuration bits (see eEvb2ConfigBits). 0x10=stream PPD on log button", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("radioPID", radioPID, DATA_TYPE_UINT32, uint32_t, "", "Radio Preamble ID in hexadecimal. 0x0 to 0x9", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("radioNID", radioNID, DATA_TYPE_UINT32, uint32_t, "", "Radio Network ID in hexadecimal. 0x0 to 0x7FFF", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("radioPowerLevel", radioPowerLevel, DATA_TYPE_UINT32, uint32_t, "", "Radio transmitter output power level. (XBee PRO SX 0=20dBm, 1=27dBm, 2=30dBm)");

    ADD_MAP_6("wifi[0].ssid", wifi[0].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Service Set Identifier (SSID) or network name.");
    ADD_MAP_6("wifi[0].psk",  wifi[0].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Pre-Shared Key (PSK) authentication or network password.");
    ADD_MAP_6("wifi[1].ssid", wifi[1].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Service Set Identifier (SSID) or network name.");
    ADD_MAP_6("wifi[1].psk",  wifi[1].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Pre-Shared Key (PSK) authentication or network password.");
    ADD_MAP_6("wifi[2].ssid", wifi[2].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Service Set Identifier (SSID) or network name.");
    ADD_MAP_6("wifi[2].psk",  wifi[2].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], "", "WiFi Pre-Shared Key (PSK) authentication or network password.");
    ADD_MAP_6("server[0].ipAddr[0]", server[0].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[0].ipAddr[1]", server[0].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[0].ipAddr[2]", server[0].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[0].ipAddr[3]", server[0].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[0].port", server[0].port, DATA_TYPE_UINT32, uint32_t, "", "Sever port");
    ADD_MAP_6("server[1].ipAddr[0]", server[1].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[1].ipAddr[1]", server[1].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[1].ipAddr[2]", server[1].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[1].ipAddr[3]", server[1].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[1].port", server[1].port, DATA_TYPE_UINT32, uint32_t, "", "Sever port");
    ADD_MAP_6("server[2].ipAddr[0]", server[2].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[2].ipAddr[1]", server[2].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[2].ipAddr[2]", server[2].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[2].ipAddr[3]", server[2].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&, "", "Server IP address");
    ADD_MAP_6("server[2].port", server[2].port, DATA_TYPE_UINT32, uint32_t, "", "Sever port");

    ADD_MAP_6("encoderTickToWheelRad", encoderTickToWheelRad, DATA_TYPE_F32, float, "rad/tick", "Wheel encoder tick to wheel rotation scalar");
    ADD_MAP_6("CANbaud_kbps", CANbaud_kbps, DATA_TYPE_UINT32, uint32_t, "kbps", "CAN baud rate");
    ADD_MAP_7("can_receive_address", can_receive_address, DATA_TYPE_UINT32, uint32_t, "", "CAN Receive Address", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("reserved2[0]", reserved2[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved2[1]", reserved2[1], DATA_TYPE_UINT8, uint8_t&);

    ADD_MAP_6("h3sp330BaudRate", h3sp330BaudRate, DATA_TYPE_UINT32, uint32_t, "", "Baud rate for EVB serial port on H3 (SP330 RS233 and RS485/422).");
    ADD_MAP_6("h4xRadioBaudRate", h4xRadioBaudRate, DATA_TYPE_UINT32, uint32_t, "", "Baud rate for EVB serial port H4 (TLL to external radio).");
    ADD_MAP_6("h8gpioBaudRate", h8gpioBaudRate, DATA_TYPE_UINT32, uint32_t, "", "Baud rate for EVB serial port H8 (TLL).");
    ADD_MAP_7("wheelCfgBits", wheelCfgBits, DATA_TYPE_UINT32, uint32_t, "", "(eWheelCfgBits). Reverse encoder [0x100 left, 0x200 right, 0x300 both], enable [0x2 encoder, 0x4 wheel]", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("velocityControlPeriodMs", velocityControlPeriodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Wheel encoder and control update period");

    ADD_MAP_6("size", size, DATA_TYPE_UINT32, uint32_t, "", "Flash group size. Set to 1 to reset this flash group.");
    ADD_MAP_6("checksum", checksum, DATA_TYPE_UINT32, uint32_t, "", "Flash checksum");
    ADD_MAP_6("key", key, DATA_TYPE_UINT32, uint32_t, "", "Flash key");
    ASSERT_SIZE(totalSize);
}

static void PopulateDebugArrayMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(debug_array_t, id);
    ADD_MAP_4("i[0]", i[0], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_4("i[1]", i[1], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_4("i[2]", i[2], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_4("i[3]", i[3], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_4("i[4]", i[4], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_4("i[5]", i[5], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_4("i[6]", i[6], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_4("i[7]", i[7], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_4("i[8]", i[8], DATA_TYPE_INT32, int32_t&);
    ADD_MAP_7("f[0]", f[0], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("f[1]", f[1], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("f[2]", f[2], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("f[3]", f[3], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("f[4]", f[4], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("f[5]", f[5], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("f[6]", f[6], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("f[7]", f[7], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("f[8]", f[8], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("lf[0]", lf[0], DATA_TYPE_F64, double&, "", "", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("lf[1]", lf[1], DATA_TYPE_F64, double&, "", "", DATA_FLAGS_FIXED_DECIMAL_9);
    ADD_MAP_7("lf[2]", lf[2], DATA_TYPE_F64, double&, "", "", DATA_FLAGS_FIXED_DECIMAL_9);
    ASSERT_SIZE(totalSize);
}

static void PopulateDebugStringMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(debug_string_t, id);    
    ADD_MAP_6("s", s, DATA_TYPE_STRING, uint8_t[DEBUG_STRING_SIZE], "", "");
    ASSERT_SIZE(totalSize);
}

#if defined(INCLUDE_LUNA_DATA_SETS)

static void PopulateEvbLunaFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_flash_cfg_t, DID_EVB_LUNA_FLASH_CFG); 
    ADD_MAP_7("bits", bits, DATA_TYPE_UINT32, uint32_t, "", "0x1 geof, 0x2 bump, 0x4 prox, 0x100 rkill, 0x200 rkclient, 0x400 rkclient2", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("minLatGeofence", minLatGeofence, DATA_TYPE_F64, double, "deg", "Geofence Min Latitude");
    ADD_MAP_6("maxLatGeofence", maxLatGeofence, DATA_TYPE_F64, double, "deg", "Geofence Max Latitude");
    ADD_MAP_6("minLonGeofence", minLonGeofence, DATA_TYPE_F64, double, "deg", "Geofence Min Longitude");
    ADD_MAP_6("maxLonGeofence", maxLonGeofence, DATA_TYPE_F64, double, "deg", "Geofence Max Longitude");
    ADD_MAP_6("remoteKillTimeoutMs", remoteKillTimeoutMs, DATA_TYPE_UINT32, uint32_t, "", "");
    ADD_MAP_6("bumpSensitivity", bumpSensitivity, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("minProxDistance", minProxDistance, DATA_TYPE_F32, float, "", "");
    ADD_MAP_7("velControl.config", velControl.config, DATA_TYPE_UINT32, uint32_t, "", "0=hoverbot, 1=ZTM, 2=PWM", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("velControl.cmdTimeoutMs", velControl.cmdTimeoutMs, DATA_TYPE_UINT32, uint32_t, "", "");
    ADD_MAP_6("velControl.wheelRadius", velControl.wheelRadius, DATA_TYPE_F32, float, "rad", "Wheel radius");
    ADD_MAP_6("velControl.wheelBaseline", velControl.wheelBaseline, DATA_TYPE_F32, float, "m", "Wheel baseline, distance between wheels");
    ADD_MAP_6("velControl.engine_rpm", velControl.engine_rpm, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.vehicle.u_min", velControl.vehicle.u_min, DATA_TYPE_F32, float, "m/s", "");
    ADD_MAP_6("velControl.vehicle.u_cruise", velControl.vehicle.u_cruise, DATA_TYPE_F32, float, "m/s", "");
    ADD_MAP_6("velControl.vehicle.u_max", velControl.vehicle.u_max, DATA_TYPE_F32, float, "m/s", "");
    ADD_MAP_6("velControl.vehicle.u_slewLimit", velControl.vehicle.u_slewLimit, DATA_TYPE_F32, float, "m/s", "");
    ADD_MAP_6("velControl.vehicle.w_max_autonomous", velControl.vehicle.w_max_autonomous, DATA_TYPE_F32, float, "rad/s", "");
    ADD_MAP_6("velControl.vehicle.w_max", velControl.vehicle.w_max, DATA_TYPE_F32, float, "rad/s", "");
    ADD_MAP_6("velControl.vehicle.w_slewLimit", velControl.vehicle.w_slewLimit, DATA_TYPE_F32, float, "rad/s", "");
    ADD_MAP_6("velControl.vehicle.u_FB_Kp", velControl.vehicle.u_FB_Kp, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.vehicle.w_FB_Kp", velControl.vehicle.w_FB_Kp, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.vehicle.w_FB_Ki", velControl.vehicle.w_FB_Ki, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.vehicle.w_FF_c0", velControl.vehicle.w_FF_c0, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.vehicle.w_FF_c1", velControl.vehicle.w_FF_c1, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.vehicle.w_FF_deadband", velControl.vehicle.w_FF_deadband, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.vehicle.testSweepRate", velControl.vehicle.testSweepRate, DATA_TYPE_F32, float, "m/s/s", "");
    ADD_MAP_6("velControl.wheel.slewRate", velControl.wheel.slewRate, DATA_TYPE_F32, float, "rad/s/s", "");
    ADD_MAP_6("velControl.wheel.velMax", velControl.wheel.velMax, DATA_TYPE_F32, float, "rad/s", "");
    ADD_MAP_6("velControl.wheel.FF_vel_deadband", velControl.wheel.FF_vel_deadband, DATA_TYPE_F32, float, "rad/s", "");
    ADD_MAP_6("velControl.wheel.FF_c_est_Ki[0]", velControl.wheel.FF_c_est_Ki[0], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.FF_c_est_Ki[1]", velControl.wheel.FF_c_est_Ki[1], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.FF_c_est_max[0]", velControl.wheel.FF_c_est_max[0], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.FF_c_est_max[1]", velControl.wheel.FF_c_est_max[1], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.FF_c_l[0]", velControl.wheel.FF_c_l[0], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.FF_c_l[1]", velControl.wheel.FF_c_l[1], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.FF_c_r[0]", velControl.wheel.FF_c_r[0], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.FF_c_r[1]", velControl.wheel.FF_c_r[1], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.FF_FB_engine_rpm", velControl.wheel.FF_FB_engine_rpm, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.FB_Kp", velControl.wheel.FB_Kp, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.FB_Ki", velControl.wheel.FB_Ki, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.FB_Kd", velControl.wheel.FB_Kd, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.FB_gain_deadband", velControl.wheel.FB_gain_deadband, DATA_TYPE_F32, float, "rad/s", "");
    ADD_MAP_6("velControl.wheel.FB_gain_deadband_reduction", velControl.wheel.FB_gain_deadband_reduction, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_l[0]", velControl.wheel.InversePlant_l[0], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_l[1]", velControl.wheel.InversePlant_l[1], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_l[2]", velControl.wheel.InversePlant_l[2], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_l[3]", velControl.wheel.InversePlant_l[3], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_l[4]", velControl.wheel.InversePlant_l[4], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_r[0]", velControl.wheel.InversePlant_r[0], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_r[1]", velControl.wheel.InversePlant_r[1], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_r[2]", velControl.wheel.InversePlant_r[2], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_r[3]", velControl.wheel.InversePlant_r[3], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.InversePlant_r[4]", velControl.wheel.InversePlant_r[4], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.actuatorTrim_l", velControl.wheel.actuatorTrim_l, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.actuatorTrim_r", velControl.wheel.actuatorTrim_r, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.actuatorLimits_l[0]", velControl.wheel.actuatorLimits_l[0], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.actuatorLimits_l[1]", velControl.wheel.actuatorLimits_l[1], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.actuatorLimits_r[0]", velControl.wheel.actuatorLimits_r[0], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.actuatorLimits_r[1]", velControl.wheel.actuatorLimits_r[1], DATA_TYPE_F32, float&, "", "");
    ADD_MAP_6("velControl.wheel.actuatorDeadbandDuty_l", velControl.wheel.actuatorDeadbandDuty_l, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.actuatorDeadbandDuty_r", velControl.wheel.actuatorDeadbandDuty_r, DATA_TYPE_F32, float, "", "");
    ADD_MAP_6("velControl.wheel.actuatorDeadbandVel", velControl.wheel.actuatorDeadbandVel, DATA_TYPE_F32, float, "", "");

    ADD_MAP_6("size", size, DATA_TYPE_UINT32, uint32_t, "", "Flash group size. Set to 1 to reset this flash group.");
    ADD_MAP_6("checksum", checksum, DATA_TYPE_UINT32, uint32_t, "", "Flash checksum");
    ADD_MAP_6("key", key, DATA_TYPE_UINT32, uint32_t, "", "Flash key");
    ASSERT_SIZE(totalSize);
}

static void PopulateCoyoteStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_status_t, DID_EVB_LUNA_STATUS);
    ADD_MAP_6("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "GPS time of week (since Sunday morning).");
    ADD_MAP_7("evbLunaStatus", evbLunaStatus, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("motorState", motorState, DATA_TYPE_UINT32, uint32_t, "", "");
    ADD_MAP_6("remoteKillMode", remoteKillMode, DATA_TYPE_UINT32, uint32_t, "", "Motor state (eLunaMotorState)");
    ADD_MAP_7("supplyVoltage", supplyVoltage, DATA_TYPE_F32, float, "V", "", DATA_FLAGS_FIXED_DECIMAL_1);
    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaSensorsMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_sensors_t, DID_EVB_LUNA_SENSORS);
    ADD_MAP_6("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "s", "GPS time of week (since Sunday morning).");
    ADD_MAP_7("proxSensorOutput[0]", proxSensorOutput[0], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("proxSensorOutput[1]", proxSensorOutput[1], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("proxSensorOutput[2]", proxSensorOutput[2], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("proxSensorOutput[3]", proxSensorOutput[3], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("proxSensorOutput[4]", proxSensorOutput[4], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("proxSensorOutput[5]", proxSensorOutput[5], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("proxSensorOutput[6]", proxSensorOutput[6], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("proxSensorOutput[7]", proxSensorOutput[7], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("proxSensorOutput[8]", proxSensorOutput[8], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_4("bumpEvent", bumpEvent, DATA_TYPE_INT32, int32_t);
    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaVelocityControlMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_velocity_control_t, DID_EVB_LUNA_VELOCITY_CONTROL);
    ADD_MAP_6("timeMs", timeMs, DATA_TYPE_UINT32, uint32_t, "ms", "");
    ADD_MAP_7("dt", dt, DATA_TYPE_F32, float, "s", "", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("current_mode", current_mode, DATA_TYPE_UINT32, uint32_t, "", "0 disable, 1 stop, 2 enable, test vel[3 dual, 4 single, 5 sweep], 6 test eff, test duty [7 single, 8 sweep] (eLunaWheelControllerMode)");
    ADD_MAP_7("vehicle.velCmd_f", vehicle.velCmd_f, DATA_TYPE_F32, float, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.velCmd_w", vehicle.velCmd_w, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.velCmdMnl_f", vehicle.velCmdMnl_f, DATA_TYPE_F32, float, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.velCmdMnl_w", vehicle.velCmdMnl_w, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.velCmdSlew_f", vehicle.velCmdSlew_f, DATA_TYPE_F32, float, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.velCmdSlew_w", vehicle.velCmdSlew_w, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.vel_f", vehicle.vel_f, DATA_TYPE_F32, float, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.vel_w", vehicle.vel_w, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.err_f", vehicle.err_f, DATA_TYPE_F32, float, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.err_w", vehicle.err_w, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.eff_f", vehicle.eff_f, DATA_TYPE_F32, float, "m/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vehicle.eff_w", vehicle.eff_w, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.velCmd", wheel_l.velCmd, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.velCmdSlew", wheel_l.velCmdSlew, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.vel", wheel_l.vel, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.err", wheel_l.err, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.ff_eff", wheel_l.ff_eff, DATA_TYPE_F32, float, "rad", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.fb_eff", wheel_l.fb_eff, DATA_TYPE_F32, float, "rad", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.eff", wheel_l.eff, DATA_TYPE_F32, float, "rad", "Control effort = ff_eff_l + fb_eff", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.effInt", wheel_l.effInt, DATA_TYPE_F32, float, "rad", "Control effort intermediate", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_l.effDuty", wheel_l.effDuty, DATA_TYPE_F32, float, "%", "Duty cycle 0-100", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("wheel_r.velCmd", wheel_r.velCmd, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_r.velCmdSlew", wheel_r.velCmdSlew, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_r.vel", wheel_r.vel, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_r.err", wheel_r.err, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_r.ff_eff", wheel_r.ff_eff, DATA_TYPE_F32, float, "rad", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_r.fb_eff", wheel_r.fb_eff, DATA_TYPE_F32, float, "rad", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_r.eff", wheel_r.eff, DATA_TYPE_F32, float, "rad", "Control effort = ff_eff_l + fb_eff", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_r.effInt", wheel_r.effInt, DATA_TYPE_F32, float, "rad", "Control effort intermediate", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("wheel_r.effDuty", wheel_r.effDuty, DATA_TYPE_F32, float, "%", "Duty cycle 0-100", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("potV_l", potV_l, DATA_TYPE_F32, float, "V", "Left potentiometer input", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("potV_r", potV_r, DATA_TYPE_F32, float, "V", "Right potentiometer input", DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaVelocityCommandMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_velocity_command_t, DID_EVB_LUNA_VELOCITY_COMMAND);
    ADD_MAP_6("timeMs", timeMs, DATA_TYPE_UINT32, uint32_t, "ms", "");
    ADD_MAP_6("modeCmd", modeCmd, DATA_TYPE_UINT32, uint32_t, "", "0 disable, 1 stop, 2 enable, 3 enable w/o watchdog (eLunaVelocityControlMode)");
    ADD_MAP_6("fwd_vel", fwd_vel, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_6("turn_rate", turn_rate, DATA_TYPE_F32, float, "rad/s", "", DATA_FLAGS_FIXED_DECIMAL_4);
    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaAuxCmdMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_aux_command_t, DID_EVB_LUNA_AUX_COMMAND);
    
    ADD_MAP_4("command", command, DATA_TYPE_UINT32, uint32_t, "", "0 blade off, 1 blade on, 2 ebrake on, 3 ebrake off, 4 beep (eLunaAuxCommands)");

    ASSERT_SIZE(totalSize);
}

#endif

static void PopulateGpsRtkRelMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_rtk_rel_t, id);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t,  "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("baseToRoverVector[0]", baseToRoverVector[0], DATA_TYPE_F32, float&, "m", "Vector from base to rover in ECEF.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("baseToRoverVector[1]", baseToRoverVector[1], DATA_TYPE_F32, float&, "m", "Vector from base to rover in ECEF.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("baseToRoverVector[2]", baseToRoverVector[2], DATA_TYPE_F32, float&, "m", "Vector from base to rover in ECEF.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("differentialAge", differentialAge, DATA_TYPE_F32, float, "s", "Age of differential signal received.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("arRatio", arRatio, DATA_TYPE_F32, float, "", "Ambiguity resolution ratio factor for validation.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_1);
    ADD_MAP_7("baseToRoverDistance", baseToRoverDistance, DATA_TYPE_F32, float, "", "baseToRoverDistance (m)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_8("baseToRoverHeading", baseToRoverHeading, DATA_TYPE_F32, float, SYM_DEG, "Angle from north to baseToRoverVector in local tangent plane.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_ANGLE | DATA_FLAGS_FIXED_DECIMAL_4, C_RAD2DEG);
    ADD_MAP_8("baseToRoverHeadingAcc", baseToRoverHeadingAcc, DATA_TYPE_F32, float, SYM_DEG, "Accuracy of baseToRoverHeading.", DATA_FLAGS_READ_ONLY | DATA_FLAGS_ANGLE | DATA_FLAGS_FIXED_DECIMAL_6, C_RAD2DEG);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRtkMiscMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_rtk_misc_t, id);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("accuracyPos[0]", accuracyPos[0], DATA_TYPE_F32, float&, "m", "Accuracy in meters north, east, up (standard deviation)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("accuracyPos[1]", accuracyPos[1], DATA_TYPE_F32, float&, "m", "Accuracy in meters north, east, up (standard deviation)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("accuracyPos[2]", accuracyPos[2], DATA_TYPE_F32, float&, "m", "Accuracy in meters north, east, up (standard deviation)", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("accuracyCov[0]", accuracyCov[0], DATA_TYPE_F32, float&, "m", "Absolute value of means square root of estimated covariance NE, EU, UN", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("accuracyCov[1]", accuracyCov[1], DATA_TYPE_F32, float&, "m", "Absolute value of means square root of estimated covariance NE, EU, UN", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("accuracyCov[2]", accuracyCov[2], DATA_TYPE_F32, float&, "m", "Absolute value of means square root of estimated covariance NE, EU, UN", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("arThreshold", arThreshold, DATA_TYPE_F32, float, "", "Ambiguity resolution threshold for validation", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("gDop", gDop, DATA_TYPE_F32, float, "m", "Geomatic dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("hDop", hDop, DATA_TYPE_F32, float, "m", "Horizontal dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("vDop", vDop, DATA_TYPE_F32, float, "m", "Vertical dilution of precision", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("baseLla[0]", baseLla[0], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Base position in latitude, longitude, altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_7);
    ADD_MAP_7("baseLla[1]", baseLla[1], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Base position in latitude, longitude, altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_7);
    ADD_MAP_7("baseLla[2]", baseLla[2], DATA_TYPE_F64, double&, SYM_DEG_DEG_M, "Base position in latitude, longitude, altitude", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_7);
    ADD_MAP_7("cycleSlipCount", cycleSlipCount, DATA_TYPE_UINT32, uint32_t, "int", "Cycle slip counter", DATA_FLAGS_READ_ONLY);

    ADD_MAP_7("roverGpsObservationCount", roverGpsObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover gps observation element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseGpsObservationCount", baseGpsObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Base gps observation element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("roverGlonassObservationCount", roverGlonassObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover glonass observation element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseGlonassObservationCount", baseGlonassObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Base glonass observation element counter", DATA_FLAGS_READ_ONLY);

    ADD_MAP_7("roverGalileoObservationCount", roverGalileoObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover galileo observation element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseGalileoObservationCount", baseGalileoObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Base galileo observation element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("roverBeidouObservationCount", roverBeidouObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover beidou observation element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseBeidouObservationCount", baseBeidouObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Base beidou observation element counter", DATA_FLAGS_READ_ONLY);

    ADD_MAP_7("roverQzsObservationCount", roverQzsObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover qzs observation element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseQzsObservationCount", baseQzsObservationCount, DATA_TYPE_UINT32, uint32_t, "int", "Base qzs observation element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("roverGpsEphemerisCount", roverGpsEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover gps ephemeris element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseGpsEphemerisCount", baseGpsEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Base gps ephemeris element counter", DATA_FLAGS_READ_ONLY);

    ADD_MAP_7("roverGlonassEphemerisCount", roverGlonassEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover glonass ephemeris element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseGlonassEphemerisCount", baseGlonassEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Base glonass ephemeris element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("roverGalileoEphemerisCount", roverGalileoEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover galileo ephemeris element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseGalileoEphemerisCount", baseGalileoEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Base galileo ephemeris element counter", DATA_FLAGS_READ_ONLY);

    ADD_MAP_7("roverBeidouEphemerisCount", roverBeidouEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover beidou ephemeris element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseBeidouEphemerisCount", baseBeidouEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Base beidou ephemeris element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("roverQzsEphemerisCount", roverQzsEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover qzs ephemeris element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseQzsEphemerisCount", baseQzsEphemerisCount, DATA_TYPE_UINT32, uint32_t, "int", "Base qzs ephemeris element counter", DATA_FLAGS_READ_ONLY);

    ADD_MAP_7("roverSbasCount", roverSbasCount, DATA_TYPE_UINT32, uint32_t, "int", "Rover sbas element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseSbasCount", baseSbasCount, DATA_TYPE_UINT32, uint32_t, "int", "Base sbas element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("baseAntennaCount", baseAntennaCount, DATA_TYPE_UINT32, uint32_t, "int", "Base antenna position element counter", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("ionUtcAlmCount", ionUtcAlmCount, DATA_TYPE_UINT32, uint32_t, "int", "Ion model / utc / alm element counter", DATA_FLAGS_READ_ONLY);

    ADD_MAP_7("correctionChecksumFailures", correctionChecksumFailures, DATA_TYPE_UINT32, uint32_t, "int", "Correction input checksum failures", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("timeToFirstFixMs", timeToFirstFixMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time to first RTK fix", DATA_FLAGS_READ_ONLY);
    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRawMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_raw_t, id);
    ADD_MAP_7("receiveIndex", receiverIndex, DATA_TYPE_UINT8, uint8_t, "", "Receiver index (1=Rover, 2=Base). RTK positioning or RTK compassing must be enabled to stream raw GPS data.", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("dataType", dataType, DATA_TYPE_UINT8, uint8_t, "", "Type of data (eRawDataType: 1=observations, 2=ephemeris, 3=glonassEphemeris, 4=SBAS, 5=baseAntenna, 6=IonosphereModel)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("obsCount", obsCount, DATA_TYPE_UINT8, uint8_t, "", "Number of observations in array (obsd_t) when dataType==1", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("reserved", reserved, DATA_TYPE_UINT8, uint8_t, "", "Reserved", DATA_FLAGS_READ_ONLY);
    ADD_MAP_4("dataBuf", data.buf, DATA_TYPE_BINARY, uint8_t[MEMBERSIZE(MAP_TYPE, data.buf)]);
    ASSERT_SIZE(totalSize);
}

static void PopulateStrobeInTimeMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(strobe_in_time_t, DID_STROBE_IN_TIME);
    ADD_MAP_7("week", week, DATA_TYPE_UINT32, uint32_t, "week", "Weeks since Jan 6, 1980", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pin", pin, DATA_TYPE_UINT16, uint16_t, "", "STROBE input pin number", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("count", count, DATA_TYPE_UINT16, uint16_t, "", "STROBE input serial index number", DATA_FLAGS_READ_ONLY);
    ASSERT_SIZE(totalSize);
}

static void PopulateRtosInfoMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rtos_info_t, DID_RTOS_INFO);
    ADD_MAP_7("freeHeapSize", freeHeapSize, DATA_TYPE_UINT32, uint32_t, "", "Heap unused bytes (high-water mark)", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mallocSize", mallocSize, DATA_TYPE_UINT32, uint32_t, "", "Total memory allocated using malloc()", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("freeSize", freeSize, DATA_TYPE_UINT32, uint32_t, "", "Total memory freed using free()", DATA_FLAGS_READ_ONLY);

    ADD_MAP_6("T0_name", task[0].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T0_cpuUsage", task[0].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T0_stackUnused", task[0].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T0_priority", task[0].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T0_periodMs", task[0].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T0_runtimeUs", task[0].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T0_avgRuntimeUs", task[0].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T0_avgLowerRuntimeUs", task[0].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T0_avgUpperRuntimeUs", task[0].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T0_maxRuntimeUs", task[0].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T0_startTimeUs", task[0].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T0_gapCount", task[0].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T0_doubleGapCount", task[0].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T0_reserved", task[0].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T0_handle", task[0].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T1_name", task[1].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T1_cpuUsage", task[1].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T1_stackUnused", task[1].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T1_priority", task[1].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T1_periodMs", task[1].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T1_runtimeUs", task[1].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T1_avgRuntimeUs", task[1].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T1_avgLowerRuntimeUs", task[1].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T1_avgUpperRuntimeUs", task[1].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T1_maxRuntimeUs", task[1].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T1_startTimeUs", task[1].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T1_gapCount", task[1].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T1_doubleGapCount", task[1].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T1_reserved", task[1].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T1_handle", task[1].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T2_name", task[2].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T2_cpuUsage", task[2].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T2_stackUnused", task[2].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T2_priority", task[2].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T2_periodMs", task[2].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T2_runtimeUs", task[2].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T2_avgRuntimeUs", task[2].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T2_avgLowerRuntimeUs", task[2].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T2_avgUpperRuntimeUs", task[2].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T2_maxRuntimeUs", task[2].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T2_startTimeUs", task[2].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T2_gapCount", task[2].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T2_doubleGapCount", task[2].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T2_reserved", task[2].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T2_handle", task[2].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T3_name", task[3].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T3_cpuUsage", task[3].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T3_stackUnused", task[3].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T3_priority", task[3].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T3_periodMs", task[3].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T3_runtimeUs", task[3].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T3_avgRuntimeUs", task[3].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T3_avgLowerRuntimeUs", task[3].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T3_avgUpperRuntimeUs", task[3].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T3_maxRuntimeUs", task[3].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T3_startTimeUs", task[3].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T3_gapCount", task[3].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T3_doubleGapCount", task[3].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T3_reserved", task[3].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T3_handle", task[3].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T4_name", task[4].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T4_cpuUsage", task[4].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T4_stackUnused", task[4].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T4_priority", task[4].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T4_periodMs", task[4].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T4_runtimeUs", task[4].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T4_avgRuntimeUs", task[4].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T4_avgLowerRuntimeUs", task[4].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T4_avgUpperRuntimeUs", task[4].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T4_maxRuntimeUs", task[4].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T4_startTimeUs", task[4].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T4_gapCount", task[4].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T4_doubleGapCount", task[4].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T4_reserved", task[4].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T4_handle", task[4].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_6("T5_name", task[5].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], "", "Task name");
    ADD_MAP_7("T5_cpuUsage", task[5].cpuUsage, DATA_TYPE_F32, f_t, "%", "CPU usage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("T5_stackUnused", task[5].stackUnused, DATA_TYPE_UINT32, uint32_t, "", "Task stack unused bytes (high-water mark)");
    ADD_MAP_6("T5_priority", task[5].priority, DATA_TYPE_UINT32, uint32_t, "", "Task priority");
    ADD_MAP_7("T5_periodMs", task[5].periodMs, DATA_TYPE_UINT32, uint32_t, "ms", "Task period", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("T5_runtimeUs", task[5].runtimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task execution time");
    ADD_MAP_7("T5_avgRuntimeUs", task[5].avgRuntimeUs, DATA_TYPE_F32, float, "us", "Average runtime", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T5_avgLowerRuntimeUs", task[5].lowerRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes less than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("T5_avgUpperRuntimeUs", task[5].upperRuntimeUs, DATA_TYPE_F32, float, "us", "Average of runtimes greater than avgRuntimeUs", DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_6("T5_maxRuntimeUs", task[5].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t, "us", "Task max execution time");
    ADD_MAP_6("T5_startTimeUs", task[5].startTimeUs, DATA_TYPE_UINT32, uint32_t, "us", "");
    ADD_MAP_6("T5_gapCount", task[5].gapCount, DATA_TYPE_UINT16, uint16_t, "", "Number of times task took too long");
    ADD_MAP_6("T5_doubleGapCount", task[5].doubleGapCount, DATA_TYPE_UINT8, uint8_t, "", "Number of times task took too long twice in a row");
    ADD_MAP_4("T5_reserved", task[5].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T5_handle", task[5].handle, DATA_TYPE_UINT32, uint32_t);
    ASSERT_SIZE(totalSize);
}
static void PopulateCanConfigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(can_config_t, DID_CAN_CONFIG);
    ADD_MAP_6("can_period_mult[CID_INS_TIME]", can_period_mult[CID_INS_TIME], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_TIME Messages");
    ADD_MAP_6("can_period_mult[CID_INS_STATUS]", can_period_mult[CID_INS_STATUS], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_STATUS Messages");
    ADD_MAP_6("can_period_mult[CID_INS_EULER]", can_period_mult[CID_INS_EULER], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_EULER Messages");
    ADD_MAP_6("can_period_mult[CID_INS_QUATN2B]", can_period_mult[CID_INS_QUATN2B], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_QUATN2B Messages");
    ADD_MAP_6("can_period_mult[CID_INS_QUATE2B]", can_period_mult[CID_INS_QUATE2B], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_QUATE2B Messages");
    ADD_MAP_6("can_period_mult[CID_INS_UVW]", can_period_mult[CID_INS_UVW], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_UVW Messages");
    ADD_MAP_6("can_period_mult[CID_INS_VE]", can_period_mult[CID_INS_VE], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_VE Messages");
    ADD_MAP_6("can_period_mult[CID_INS_LAT]", can_period_mult[CID_INS_LAT], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_LAT Messages");
    ADD_MAP_6("can_period_mult[CID_INS_LON]", can_period_mult[CID_INS_LON], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_LON Messages");
    ADD_MAP_6("can_period_mult[CID_INS_ALT]", can_period_mult[CID_INS_ALT], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_ALT Messages");
    ADD_MAP_6("can_period_mult[CID_INS_NORTH_EAST]", can_period_mult[CID_INS_NORTH_EAST], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_NORTH_EAST Messages");
    ADD_MAP_6("can_period_mult[CID_INS_DOWN]", can_period_mult[CID_INS_DOWN], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_DOWN Messages");
    ADD_MAP_6("can_period_mult[CID_INS_ECEF_X]", can_period_mult[CID_INS_ECEF_X], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_ECEF_X Messages");
    ADD_MAP_6("can_period_mult[CID_INS_ECEF_Y]", can_period_mult[CID_INS_ECEF_Y], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_ECEF_Y Messages");
    ADD_MAP_6("can_period_mult[CID_INS_ECEF_Z]", can_period_mult[CID_INS_ECEF_Z], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_ECEF_Z Messages");
    ADD_MAP_6("can_period_mult[CID_INS_MSL]", can_period_mult[CID_INS_MSL], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_INS_MSL Messages");
    ADD_MAP_6("can_period_mult[CID_PREINT_PX]", can_period_mult[CID_PREINT_PX], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_PREINT_PX Messages");
    ADD_MAP_6("can_period_mult[CID_PREINT_QY]", can_period_mult[CID_PREINT_QY], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_PREINT_QY Messages");
    ADD_MAP_6("can_period_mult[CID_PREINT_RZ]", can_period_mult[CID_PREINT_RZ], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_PREINT_RZ Messages");
    ADD_MAP_6("can_period_mult[CID_DUAL_PX]", can_period_mult[CID_DUAL_PX], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_DUAL_PX Messages");
    ADD_MAP_6("can_period_mult[CID_DUAL_QY]", can_period_mult[CID_DUAL_QY], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_DUAL_QY Messages");
    ADD_MAP_6("can_period_mult[CID_DUAL_RZ]", can_period_mult[CID_DUAL_RZ], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_DUAL_RZ Messages");
    ADD_MAP_6("can_period_mult[CID_GPS1_POS]", can_period_mult[CID_GPS1_POS], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_GPS1_POS Messages");
    ADD_MAP_6("can_period_mult[CID_GPS2_POS]", can_period_mult[CID_GPS2_POS], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_GPS2_POS Messages");
    ADD_MAP_6("can_period_mult[CID_GPS1_RTK_POS_REL]", can_period_mult[CID_GPS1_RTK_POS_REL], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_GPS1_RTK_POS_REL Messages");
    ADD_MAP_6("can_period_mult[CID_GPS2_RTK_CMP_REL]", can_period_mult[CID_GPS2_RTK_CMP_REL], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_GPS2_RTK_CMP_REL Messages");
    ADD_MAP_6("can_period_mult[CID_ROLL_ROLLRATE]", can_period_mult[CID_ROLL_ROLLRATE], DATA_TYPE_UINT16, uint16_t&, " ", "Broadcast Period Multiple for CID_ROLL_ROLLRATE Messages");

    ADD_MAP_7("can_transmit_address[CID_INS_TIME]", can_transmit_address[CID_INS_TIME], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_TIME Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_STATUS]", can_transmit_address[CID_INS_STATUS], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_STATUS Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_EULER]", can_transmit_address[CID_INS_EULER], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_EULER Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_QUATN2B]", can_transmit_address[CID_INS_QUATN2B], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_QUATN2B Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_QUATE2B]", can_transmit_address[CID_INS_QUATE2B], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_QUATE2B Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_UVW]", can_transmit_address[CID_INS_UVW], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_UVW Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_VE]", can_transmit_address[CID_INS_VE], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_VE Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_LAT]", can_transmit_address[CID_INS_LAT], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_LAT Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_LON]", can_transmit_address[CID_INS_LON], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_LON Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_ALT]", can_transmit_address[CID_INS_ALT], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_ALT Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_NORTH_EAST]", can_transmit_address[CID_INS_NORTH_EAST], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_NORTH_EAST Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_DOWN]", can_transmit_address[CID_INS_DOWN], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_DOWN Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_ECEF_X]", can_transmit_address[CID_INS_ECEF_X], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_ECEF_X Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_ECEF_Y]", can_transmit_address[CID_INS_ECEF_Y], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_ECEF_Y Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_ECEF_Z]", can_transmit_address[CID_INS_ECEF_Z], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_ECEF_Z Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_INS_MSL]", can_transmit_address[CID_INS_MSL], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_INS_MSL Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_PREINT_PX]", can_transmit_address[CID_PREINT_PX], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_PREINT_PX Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_PREINT_QY]", can_transmit_address[CID_PREINT_QY], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_PREINT_QY Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_PREINT_RZ]", can_transmit_address[CID_PREINT_RZ], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_PREINT_RZ Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_DUAL_PX]", can_transmit_address[CID_DUAL_PX], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_DUAL_PX Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_DUAL_QY]", can_transmit_address[CID_DUAL_QY], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_DUAL_QY Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_DUAL_RZ]", can_transmit_address[CID_DUAL_RZ], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_DUAL_RZ Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_GPS1_POS]", can_transmit_address[CID_GPS1_POS], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_GPS1_POS Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_GPS2_POS]", can_transmit_address[CID_GPS2_POS], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_GPS2_POS Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_GPS1_RTK_POS_REL]", can_transmit_address[CID_GPS1_RTK_POS_REL], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_GPS1_RTK_POS_REL Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_GPS2_RTK_CMP_REL]", can_transmit_address[CID_GPS2_RTK_CMP_REL], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_GPS2_RTK_CMP_REL Messages", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_7("can_transmit_address[CID_ROLL_ROLLRATE]", can_transmit_address[CID_ROLL_ROLLRATE], DATA_TYPE_UINT32, uint32_t&, "", "CAN Address CID_ROLL_ROLLRATE Messages", DATA_FLAGS_DISPLAY_HEX);

    ADD_MAP_6("can_baudrate_kbps", can_baudrate_kbps, DATA_TYPE_UINT16, uint16_t, "kbps", "CAN baud rate");
    ADD_MAP_7("can_receive_address", can_receive_address, DATA_TYPE_UINT32, uint32_t, "", "CAN Receive Address", DATA_FLAGS_DISPLAY_HEX);
    ASSERT_SIZE(totalSize);
}

static void PopulateDiagMsgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(diag_msg_t, DID_DIAGNOSTIC_MESSAGE);
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("messageLength", messageLength, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("message", message, DATA_TYPE_STRING, char[MEMBERSIZE(diag_msg_t, message)]);
    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsADCMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sys_sensors_adc_t, DID_SENSORS_ADC);
    ADD_MAP_7("time", time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr1[0]", imu[0].pqr[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr1[1]", imu[0].pqr[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr1[2]", imu[0].pqr[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc1[0]", imu[0].acc[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc1[1]", imu[0].acc[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc1[2]", imu[0].acc[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("temp1",   imu[0].temp, DATA_TYPE_F32, float, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr2[0]", imu[1].pqr[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr2[1]", imu[1].pqr[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr2[2]", imu[1].pqr[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc2[0]", imu[1].acc[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc2[1]", imu[1].acc[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc2[2]", imu[1].acc[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("temp2", imu[1].temp, DATA_TYPE_F32, float, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr3[0]", imu[2].pqr[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr3[1]", imu[2].pqr[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr3[2]", imu[2].pqr[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc3[0]", imu[2].acc[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc3[1]", imu[2].acc[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc3[2]", imu[2].acc[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("temp3", imu[2].temp, DATA_TYPE_F32, float, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag1[0]", mag[0].mag[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag1[1]", mag[0].mag[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag1[2]", mag[0].mag[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag2[0]", mag[1].mag[0], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag2[1]", mag[1].mag[1], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag2[2]", mag[1].mag[2], DATA_TYPE_F32, float&, "LSB", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("bar", bar, DATA_TYPE_F32, float, "kPa", "Barometric pressure", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("barTemp", barTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Barometer temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("humidity", humidity, DATA_TYPE_F32, float, "%rH", "Relative humidity, 0%-100%", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ana[0]", ana[0], DATA_TYPE_F32, float&, "V", "ADC analog 0 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ana[1]", ana[1], DATA_TYPE_F32, float&, "V", "ADC analog 1 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ana[2]", ana[2], DATA_TYPE_F32, float&, "V", "ADC analog 3 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ana[3]", ana[3], DATA_TYPE_F32, float&, "V", "ADC analog 4 input voltage", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsISMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(sensors_w_temp_t, id);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2;
    ADD_MAP_7("imu3.time", imu3.time, DATA_TYPE_F64, double, "s", "Time since boot up", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("imu3.status", imu3.status, DATA_TYPE_UINT32, uint32_t, "", "Status", DATA_FLAGS_READ_ONLY | DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_8("pqr0[0]", imu3.I[0].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0[1]", imu3.I[0].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0[2]", imu3.I[0].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_7("acc0[0]", imu3.I[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc0[1]", imu3.I[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc0[2]", imu3.I[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_8("pqr1[0]", imu3.I[1].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1[1]", imu3.I[1].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1[2]", imu3.I[1].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_7("acc1[0]", imu3.I[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc1[1]", imu3.I[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc1[2]", imu3.I[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_8("pqr2[0]", imu3.I[2].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2[1]", imu3.I[2].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2[2]", imu3.I[2].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "", flags, C_RAD2DEG);
    ADD_MAP_7("acc2[0]", imu3.I[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc2[1]", imu3.I[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("acc2[2]", imu3.I[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "", flags);
    ADD_MAP_7("temp0", temp[0], DATA_TYPE_F32, float&, SYM_DEG_C, "Uncalibrated sensor output.", flags);
    ADD_MAP_7("temp1", temp[1], DATA_TYPE_F32, float&, SYM_DEG_C, "Uncalibrated sensor output.", flags);
    ADD_MAP_7("temp2", temp[2], DATA_TYPE_F32, float&, SYM_DEG_C, "Uncalibrated sensor output.", flags);
    ADD_MAP_7("mag0[0]", mag[0].xyz[0], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag0[1]", mag[0].xyz[1], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag0[2]", mag[0].xyz[2], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag1[0]", mag[1].xyz[0], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag1[1]", mag[1].xyz[1], DATA_TYPE_F32, float&, "", "", flags);
    ADD_MAP_7("mag1[2]", mag[1].xyz[2], DATA_TYPE_F32, float&, "", "", flags);
    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsTCMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sensors_t, DID_SENSORS_TC_BIAS);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3;
    ADD_MAP_6("time", time, DATA_TYPE_F64, double, "s", "GPS time of week (since Sunday morning).");
    ADD_MAP_7("pqr0[0]", mpu[0].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr0[1]", mpu[0].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr0[2]", mpu[0].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("acc0[0]", mpu[0].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc0[1]", mpu[0].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc0[2]", mpu[0].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("mag0[0]", mpu[0].mag[0], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag0[1]", mpu[0].mag[1], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag0[2]", mpu[0].mag[2], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("pqr1[0]", mpu[1].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr1[1]", mpu[1].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr1[2]", mpu[1].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("acc1[0]", mpu[1].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc1[1]", mpu[1].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc1[2]", mpu[1].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("mag1[0]", mpu[1].mag[0], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag1[1]", mpu[1].mag[1], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag1[2]", mpu[1].mag[2], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("pqr2[0]", mpu[2].pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr2[1]", mpu[2].pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("pqr2[2]", mpu[2].pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Temperature compensation bias", flags);
    ADD_MAP_7("acc2[0]", mpu[2].acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc2[1]", mpu[2].acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("acc2[2]", mpu[2].acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Temperature compensation bias", flags);
    ADD_MAP_7("mag2[0]", mpu[2].mag[0], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag2[1]", mpu[2].mag[1], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ADD_MAP_7("mag2[2]", mpu[2].mag[2], DATA_TYPE_F32, float&, "", "Temperature compensation bias", flags);
    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsCompMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sensor_compensation_t, DID_SCOMP);
    int flags = DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4;
    string str = 
        TOSTRING(SC_RUNTIME) "=Runtime, Tcal["
        TOSTRING(SC_TCAL_INIT) "=Init, "
        TOSTRING(SC_TCAL_RUNNING) "=Running, "
        TOSTRING(SC_TCAL_STOP) "=Stop, "
        TOSTRING(SC_TCAL_DONE) "=Done], MCAL["
        TOSTRING(SC_MCAL_SAMPLE_INIT) "=Init, "
        TOSTRING(SC_MCAL_SAMPLE_MEAN_UCAL) "=UCAL, "
        TOSTRING(SC_MCAL_SAMPLE_MEAN_TCAL) "=TCAL, "
        TOSTRING(SC_MCAL_SAMPLE_MEAN_MCAL) "=MCAL, "
        TOSTRING(SC_LPF_SAMPLE) "=LPF 0.01Hz, "
        TOSTRING(SC_LPF_SAMPLE_FAST) "=LPF 1Hz ";

    ADD_MAP_7("timeMs", timeMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time since boot up", DATA_FLAGS_READ_ONLY);
    ADD_MAP_6("calState", calState, DATA_TYPE_UINT32, uint32_t, "", str);
    ADD_MAP_7("status", status, DATA_TYPE_UINT32, uint32_t, "", "", DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_6("sampleCount", sampleCount, DATA_TYPE_UINT32, uint32_t, "", "Used in averaging");
    ADD_MAP_7("alignAccel[0]", alignAccel[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Alignment acceleration", flags);
    ADD_MAP_7("alignAccel[1]", alignAccel[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Alignment acceleration", flags);
    ADD_MAP_7("alignAccel[2]", alignAccel[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Alignment acceleration", flags);

    // Gyros
    ADD_MAP_8("pqr0.lpfLsb[0]", pqr[0].lpfLsb[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0.lpfLsb[1]", pqr[0].lpfLsb[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0.lpfLsb[2]", pqr[0].lpfLsb[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_7("pqr0.lpfTemp", pqr[0].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_8("pqr0.k[0]", pqr[0].k[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0.k[1]", pqr[0].k[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr0.k[2]", pqr[0].k[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_7("pqr0.temp", pqr[0].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr0.tempRampRate", pqr[0].tempRampRate, DATA_TYPE_F32, float, SYM_DEG_C_PER_S, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr0.tci", pqr[0].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr0.numTcPts", pqr[0].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr0.dtTemp", pqr[0].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_8("pqr1.lpfLsb[0]", pqr[1].lpfLsb[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1.lpfLsb[1]", pqr[1].lpfLsb[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1.lpfLsb[2]", pqr[1].lpfLsb[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_7("pqr1.lpfTemp", pqr[1].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_8("pqr1.k[0]", pqr[1].k[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1.k[1]", pqr[1].k[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr1.k[2]", pqr[1].k[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_7("pqr1.temp", pqr[1].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr1.tempRampRate", pqr[1].tempRampRate, DATA_TYPE_F32, float, SYM_DEG_C_PER_S, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr1.tci", pqr[1].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr1.numTcPts", pqr[1].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr1.dtTemp", pqr[1].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_8("pqr2.lpfLsb[0]", pqr[2].lpfLsb[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2.lpfLsb[1]", pqr[2].lpfLsb[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2.lpfLsb[2]", pqr[2].lpfLsb[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Low-pass filtered LSB", flags, C_RAD2DEG);
    ADD_MAP_7("pqr2.lpfTemp", pqr[2].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_8("pqr2.k[0]", pqr[2].k[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2.k[1]", pqr[2].k[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_8("pqr2.k[2]", pqr[2].k[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Slope", flags, C_RAD2DEG);
    ADD_MAP_7("pqr2.temp", pqr[2].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("pqr2.tempRampRate", pqr[2].tempRampRate, DATA_TYPE_F32, float, SYM_DEG_C_PER_S, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("pqr2.tci", pqr[2].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr2.numTcPts", pqr[2].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("pqr2.dtTemp", pqr[2].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    // Accels
    ADD_MAP_7("acc0.lpfLsb[0]", acc[0].lpfLsb[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc0.lpfLsb[1]", acc[0].lpfLsb[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc0.lpfLsb[2]", acc[0].lpfLsb[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc0.lpfTemp", acc[0].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("acc0.k[0]", acc[0].k[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc0.k[1]", acc[0].k[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc0.k[2]", acc[0].k[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc0.temp", acc[0].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc0.tempRampRate", acc[0].tempRampRate, DATA_TYPE_F32, float, SYM_M_PER_S_2, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc0.tci", acc[0].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc0.numTcPts", acc[0].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc0.dtTemp", acc[0].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_7("acc1.lpfLsb[0]", acc[1].lpfLsb[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc1.lpfLsb[1]", acc[1].lpfLsb[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc1.lpfLsb[2]", acc[1].lpfLsb[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc1.lpfTemp", acc[1].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("acc1.k[0]", acc[1].k[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc1.k[1]", acc[1].k[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc1.k[2]", acc[1].k[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc1.temp", acc[1].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc1.tempRampRate", acc[1].tempRampRate, DATA_TYPE_F32, float, SYM_M_PER_S_2, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc1.tci", acc[1].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc1.numTcPts", acc[1].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc1.dtTemp", acc[1].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_7("acc2.lpfLsb[0]", acc[2].lpfLsb[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc2.lpfLsb[1]", acc[2].lpfLsb[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc2.lpfLsb[2]", acc[2].lpfLsb[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Low-pass filtered LSB", flags);
    ADD_MAP_7("acc2.lpfTemp", acc[2].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("acc2.k[0]", acc[2].k[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc2.k[1]", acc[2].k[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc2.k[2]", acc[2].k[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Slope", flags);
    ADD_MAP_7("acc2.temp", acc[2].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("acc2.tempRampRate", acc[2].tempRampRate, DATA_TYPE_F32, float, SYM_M_PER_S_2, "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("acc2.tci", acc[2].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc2.numTcPts", acc[2].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("acc2.dtTemp", acc[2].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    // Magnetometers
    ADD_MAP_7("mag0.lpfLsb[0]", mag[0].lpfLsb[0], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag0.lpfLsb[1]", mag[0].lpfLsb[1], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag0.lpfLsb[2]", mag[0].lpfLsb[2], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag0.lpfTemp", mag[0].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("mag0.k[0]", mag[0].k[0], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag0.k[1]", mag[0].k[1], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag0.k[2]", mag[0].k[2], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag0.temp", mag[0].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag0.tempRampRate", mag[0].tempRampRate, DATA_TYPE_F32, float, "", "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag0.tci", mag[0].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mag0.numTcPts", mag[0].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mag0.dtTemp", mag[0].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    ADD_MAP_7("mag1.lpfLsb[0]", mag[1].lpfLsb[0], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag1.lpfLsb[1]", mag[1].lpfLsb[1], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag1.lpfLsb[2]", mag[1].lpfLsb[2], DATA_TYPE_F32, float&, "", "Low-pass filtered LSB", flags);
    ADD_MAP_7("mag1.lpfTemp", mag[1].lpfTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Low-pass filtered temperature", flags);
    ADD_MAP_7("mag1.k[0]", mag[1].k[0], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag1.k[1]", mag[1].k[1], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag1.k[2]", mag[1].k[2], DATA_TYPE_F32, float&, "", "Slope", flags);
    ADD_MAP_7("mag1.temp", mag[1].temp, DATA_TYPE_F32, float, SYM_DEG_C, "Temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_2);
    ADD_MAP_7("mag1.tempRampRate", mag[1].tempRampRate, DATA_TYPE_F32, float, "", "Temperature ramp rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("mag1.tci", mag[1].tci, DATA_TYPE_UINT32, uint32_t, "", "Temp comp index", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mag1.numTcPts", mag[1].numTcPts, DATA_TYPE_UINT32, uint32_t, "", "Number of temp comp points", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("mag1.dtTemp", mag[1].dtTemp, DATA_TYPE_F32, float, SYM_DEG_C, "Delta from last tc point to current temperature", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);

    // Reference IMU
    ADD_MAP_8("referenceImu.pqr[0]", referenceImu.pqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Reference IMU angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("referenceImu.pqr[1]", referenceImu.pqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Reference IMU angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("referenceImu.pqr[2]", referenceImu.pqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Reference IMU angular rate", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("referenceImu.acc[0]", referenceImu.acc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Reference IMU linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("referenceImu.acc[1]", referenceImu.acc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Reference IMU linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_7("referenceImu.acc[2]", referenceImu.acc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Reference IMU linear acceleration", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    // Reference Mag
    ADD_MAP_7("referenceMag[0]", referenceMag[0], DATA_TYPE_F32, float&, "", "Reference magnetometer", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("referenceMag[1]", referenceMag[1], DATA_TYPE_F32, float&, "", "Reference magnetometer", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("referenceMag[2]", referenceMag[2], DATA_TYPE_F32, float&, "", "Reference magnetometer", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ASSERT_SIZE(totalSize);
}

static void PopulateInl2MagObsInfo(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(inl2_mag_obs_info_t, DID_INL2_MAG_OBS_INFO);
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_4);
    ADD_MAP_6("Ncal_samples", Ncal_samples, DATA_TYPE_UINT32, uint32_t, "", "");
    ADD_MAP_6("ready", ready, DATA_TYPE_UINT32, uint32_t, "", "Data ready to be processed");
    ADD_MAP_6("calibrated", calibrated, DATA_TYPE_UINT32, uint32_t, "", "Calibration data present");
    ADD_MAP_6("auto_recal", auto_recal, DATA_TYPE_UINT32, uint32_t, "", "Allow mag to auto-recalibrate");
    ADD_MAP_6("outlier", outlier, DATA_TYPE_UINT32, uint32_t, "", "Bad sample data");
    ADD_MAP_7("magHdg", magHdg, DATA_TYPE_F32, float, "deg", "Heading from magnetometer", DATA_FLAGS_FIXED_DECIMAL_3 | DATA_FLAGS_ANGLE);
    ADD_MAP_7("insHdg", insHdg, DATA_TYPE_F32, float, "deg", "Heading from INS", DATA_FLAGS_FIXED_DECIMAL_3 | DATA_FLAGS_ANGLE);
    ADD_MAP_7("magInsHdgDelta", magInsHdgDelta, DATA_TYPE_F32, float, "deg", "Difference between magHdg and insHdg", DATA_FLAGS_FIXED_DECIMAL_3 | DATA_FLAGS_ANGLE);
    ADD_MAP_7("nis", nis, DATA_TYPE_F32, float, "", "", DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("nis_threshold", nis_threshold, DATA_TYPE_F32, float, "", "", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[0]", Wcal[0], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[1]", Wcal[1], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[2]", Wcal[2], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[3]", Wcal[3], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[4]", Wcal[4], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[5]", Wcal[5], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[6]", Wcal[6], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[7]", Wcal[7], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Wcal[8]", Wcal[8], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_6("activeCalSet", activeCalSet, DATA_TYPE_UINT32, uint32_t, "", "Active calibration set (0 or 1)");
    ADD_MAP_7("magHdgOffset", magHdgOffset, DATA_TYPE_F32, float, "deg", "Offset from mag heading to ins heading estimate", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("Tcal", Tcal, DATA_TYPE_F32, float, "", "Scaled computed variance of calibrated magnetometer samples. Above 5 is bad.", DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("bias_cal[0]", bias_cal[0], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("bias_cal[1]", bias_cal[1], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ADD_MAP_7("bias_cal[2]", bias_cal[2], DATA_TYPE_F32, float&, "", "", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_5);
    ASSERT_SIZE(totalSize);
}

static void PopulateInl2StatesMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(inl2_states_t, DID_INL2_STATES);
    int flags = DATA_FLAGS_FIXED_DECIMAL_4;
    ADD_MAP_7("timeOfWeek", timeOfWeek, DATA_TYPE_F64, double, "s", "Time of week since Sunday morning, GMT", flags);
    ADD_MAP_7("qe2b[0]", qe2b[0], DATA_TYPE_F32, float&, "", "Quaternion rotation from ECEF to body frame", flags);
    ADD_MAP_7("qe2b[1]", qe2b[1], DATA_TYPE_F32, float&, "", "Quaternion rotation from ECEF to body frame", flags);
    ADD_MAP_7("qe2b[2]", qe2b[2], DATA_TYPE_F32, float&, "", "Quaternion rotation from ECEF to body frame", flags);
    ADD_MAP_7("qe2b[3]", qe2b[3], DATA_TYPE_F32, float&, "", "Quaternion rotation from ECEF to body frame", flags);
    ADD_MAP_7("ve[0]", ve[0], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF frame", flags);
    ADD_MAP_7("ve[1]", ve[1], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF frame", flags);
    ADD_MAP_7("ve[2]", ve[2], DATA_TYPE_F32, float&, "m/s", "Velocity in ECEF frame", flags);
    ADD_MAP_7("ecef[0]", ecef[0], DATA_TYPE_F64, double&, "m", "Position in ECEF frame", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ecef[1]", ecef[1], DATA_TYPE_F64, double&, "m", "Position in ECEF frame", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_7("ecef[2]", ecef[2], DATA_TYPE_F64, double&, "m", "Position in ECEF frame", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3);
    ADD_MAP_8("biasPqr[0]", biasPqr[0], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Gyro bias", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("biasPqr[1]", biasPqr[1], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Gyro bias", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_8("biasPqr[2]", biasPqr[2], DATA_TYPE_F32, float&, SYM_DEG_PER_S, "Gyro bias", DATA_FLAGS_READ_ONLY | DATA_FLAGS_FIXED_DECIMAL_3, C_RAD2DEG);
    ADD_MAP_7("biasAcc[0]", biasAcc[0], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Accelerometer bias", flags);
    ADD_MAP_7("biasAcc[1]", biasAcc[1], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Accelerometer bias", flags);
    ADD_MAP_7("biasAcc[2]", biasAcc[2], DATA_TYPE_F32, float&, SYM_M_PER_S_2, "Accelerometer bias", flags);
    ADD_MAP_7("biasBaro", biasBaro, DATA_TYPE_F32, float, "m", "Barometer bias", flags);
    ADD_MAP_8("magDec", magDec, DATA_TYPE_F32, float, SYM_DEG, "Magnetic declination", flags, C_RAD2DEG);
    ADD_MAP_8("magInc", magInc, DATA_TYPE_F32, float, SYM_DEG, "Magnetic inclination", flags, C_RAD2DEG);
    ASSERT_SIZE(totalSize);
}



const char* const cISDataMappings::m_dataIdNames[] =
{    // Matches data identifier list (eDataIDs) in data_sets.h
    "DID_NULL",                         // 0
    "DID_DEV_INFO",                     // 1
    "DID_SYS_FAULT",                    // 2
    "DID_PIMU",                         // 3
    "DID_INS_1",                        // 4
    "DID_INS_2",                        // 5
    "DID_GPS1_RCVR_POS",                // 6
    "DID_SYS_CMD",                      // 7
    "DID_NMEA_BCAST_PERIOD",            // 8
    "DID_RMC",                          // 9
    "DID_SYS_PARAMS",                   // 10
    "DID_SYS_SENSORS",                  // 11
    "DID_FLASH_CONFIG",                 // 12
    "DID_GPS1_POS",                     // 13
    "DID_GPS2_POS",                     // 14
    "DID_GPS1_SAT",                     // 15
    "DID_GPS2_SAT",                     // 16
    "DID_GPS1_VERSION",                 // 17
    "DID_GPS2_VERSION",                 // 18
    "DID_MAG_CAL",                      // 19
    "DID_UNUSED_20",                    // 20
    "DID_GPS1_RTK_POS_REL",             // 21
    "DID_GPS1_RTK_POS_MISC",            // 22
    "DID_FEATURE_BITS",                 // 23
    "DID_SENSORS_UCAL",                 // 24
    "DID_SENSORS_TCAL",                 // 25
    "DID_SENSORS_TC_BIAS",              // 26
    "DID_IO",                           // 27
    "DID_SENSORS_ADC",                  // 28
    "DID_SCOMP",                        // 29
    "DID_GPS1_VEL",                     // 30
    "DID_GPS2_VEL",                     // 31
    "DID_HDW_PARAMS",                   // 32
    "DID_NVR_MANAGE_USERPAGE",          // 33
    "DID_NVR_USERPAGE_SN",              // 34
    "DID_NVR_USERPAGE_G0",              // 35
    "DID_NVR_USERPAGE_G1",              // 36
    "DID_DEBUG_STRING",                 // 37
    "DID_RTOS_INFO",                    // 38
    "DID_DEBUG_ARRAY",                  // 39
    "DID_SENSORS_MCAL",                 // 40
    "DID_GPS1_TIMEPULSE",               // 41
    "DID_CAL_SC",                       // 42
    "DID_CAL_SC1",                      // 43
    "DID_CAL_SC2",                      // 44
    "DID_GPS1_SIG",                     // 45
    "DID_SENSORS_ADC_SIGMA",            // 46
    "DID_REFERENCE_MAGNETOMETER",       // 47
    "DID_INL2_STATES",                  // 48
    "DID_INL2_COVARIANCE_LD",           // 49
    "DID_INL2_STATUS",                  // 50
    "DID_INL2_MISC",                    // 51
    "DID_MAGNETOMETER",                 // 52
    "DID_BAROMETER",                    // 53
    "DID_GPS1_RTK_POS",                 // 54
    "DID_ROS_COVARIANCE_POSE_TWIST",    // 55
    "DID_COMMUNICATIONS_LOOPBACK",      // 56
    "DID_IMU3_UNCAL",                   // 57
    "DID_IMU",                          // 58
    "DID_INL2_MAG_OBS_INFO",            // 59
    "DID_GPS_BASE_RAW",                 // 60
    "DID_GPS_RTK_OPT",                  // 61
    "DID_REFERENCE_PIMU",               // 62
    "DID_MANUFACTURING_INFO",           // 63
    "DID_BIT",                          // 64
    "DID_INS_3",                        // 65
    "DID_INS_4",                        // 66
    "DID_INL2_NED_SIGMA",               // 67
    "DID_STROBE_IN_TIME",               // 68
    "DID_GPS1_RAW",                     // 69
    "DID_GPS2_RAW",                     // 70
    "DID_WHEEL_ENCODER",                // 71
    "DID_DIAGNOSTIC_MESSAGE",           // 72
    "DID_SURVEY_IN",                    // 73
    "DID_CAL_SC_INFO",                  // 74
    "DID_PORT_MONITOR",                 // 75
    "DID_RTK_STATE",                    // 76
    "DID_RTK_PHASE_RESIDUAL",           // 77
    "DID_RTK_CODE_RESIDUAL",            // 78
    "DID_RTK_DEBUG",                    // 79
    "DID_EVB_STATUS",                   // 80
    "DID_EVB_FLASH_CFG",                // 81
    "DID_EVB_DEBUG_ARRAY",              // 82
    "DID_EVB_RTOS_INFO",                // 83
    "DID_GPS2_SIG",                     // 84
    "DID_IMU_MAG",                      // 85
    "DID_PIMU_MAG",                     // 86
    "DID_GROUND_VEHICLE",               // 87
    "DID_POSITION_MEASUREMENT",         // 88
    "DID_RTK_DEBUG_2",                  // 89
    "DID_CAN_CONFIG",                   // 90
    "DID_GPS2_RTK_CMP_REL",             // 91
    "DID_GPS2_RTK_CMP_MISC",            // 92
    "DID_EVB_DEV_INFO",                 // 93
    "DID_INFIELD_CAL",                  // 94 
    "DID_REFERENCE_IMU",                // 95 
    "DID_IMU3_RAW",                     // 96 
    "DID_IMU_RAW",                      // 97 
    "DID_FIRMWARE_UPDATE",              // 98 
    "DID_RUNTIME_PROFILER",             // 99 
    "UNUSED_100",                       // 100
    "UNUSED_101",                       // 101
    "UNUSED_102",                       // 102
    "UNUSED_103",                       // 103
    "UNUSED_104",                       // 104
    "UNUSED_105",                       // 105
    "UNUSED_106",                       // 106
    "UNUSED_107",                       // 107
    "UNUSED_108",                       // 108
    "UNUSED_109",                       // 109
    "DID_EVB_LUNA_FLASH_CFG",           // 110
    "DID_EVB_LUNA_STATUS",              // 111
    "DID_EVB_LUNA_SENSORS",             // 112
    "DID_EVB_LUNA_REMOTE_KILL",         // 113
    "DID_EVB_LUNA_VELOCITY_CONTROL",    // 114
    "DID_EVB_LUNA_VELOCITY_COMMAND",    // 115
    "DID_EVB_LUNA_AUX_COMMAND",         // 116
    "",                                 // 117
    "",                                 // 118
    "DID_EVENT",                        // 119
    "DID_GPX_DEV_INFO",                 // 120
    "DID_GPX_FLASH_CFG",                // 121
    "DID_GPX_RTOS_INFO",                // 122
    "DID_GPX_STATUS",                   // 123
    "DID_GPX_DEBUG_ARRAY",              // 124
    "DID_GPX_BIT",                      // 125
    "DID_GPX_RMC",                      // 126
    "DID_GPX_PORT_MONITOR",             // 127
    "",                                 // 128
    "",                                 // 129
    "",                                 // 130
    ""                                  // 131
};


cISDataMappings::cISDataMappings()
{
    PopulateSizeMappings(m_lookupSize);
    PopulateDeviceInfoMappings(m_lookupInfo, m_indexInfo, DID_DEV_INFO);
    PopulateManufacturingInfoMappings(m_lookupInfo, m_indexInfo);
    PopulateBitMappings(m_lookupInfo, m_indexInfo);
    PopulateGpxBitMappings(m_lookupInfo, m_indexInfo);
    PopulateSysFaultMappings(m_lookupInfo, m_indexInfo);
    PopulateIMU3Mappings(m_lookupInfo, m_indexInfo, DID_IMU3_UNCAL, "Triple IMU data directly from sensor (uncalibrated).");
    PopulateIMU3Mappings(m_lookupInfo, m_indexInfo, DID_IMU3_RAW, "Triple IMU data calibrated from DID_IMU3_UNCAL.");
    PopulateIMUMappings(m_lookupInfo, m_indexInfo, DID_IMU_RAW, "IMU data averaged from DID_IMU3_RAW.");
    PopulateIMUMappings(m_lookupInfo, m_indexInfo, DID_IMU, "IMU data down-sampled from IMU rate to navigation rate.");
    PopulateIMUDeltaThetaVelocityMappings(m_lookupInfo, m_indexInfo, DID_PIMU, "Preintegrated IMU.");
    PopulateIMUMagnetometerMappings(m_lookupInfo, m_indexInfo);
    PopulateIMUDeltaThetaVelocityMagMappings(m_lookupInfo, m_indexInfo);
    PopulateMagnetometerMappings(m_lookupInfo, m_indexInfo, DID_MAGNETOMETER);
    PopulateMagnetometerMappings(m_lookupInfo, m_indexInfo, DID_REFERENCE_MAGNETOMETER);
    PopulateBarometerMappings(m_lookupInfo, m_indexInfo);
    PopulateWheelEncoderMappings(m_lookupInfo, m_indexInfo);
    PopulateSysParamsMappings(m_lookupInfo, m_indexInfo);
    PopulateSysSensorsMappings(m_lookupInfo, m_indexInfo);
    PopulateRMCMappings(m_lookupInfo, m_indexInfo);
    PopulateINS1Mappings(m_lookupInfo, m_indexInfo);
    PopulateINS2Mappings(m_lookupInfo, m_indexInfo);
    PopulateINS3Mappings(m_lookupInfo, m_indexInfo);
    PopulateINS4Mappings(m_lookupInfo, m_indexInfo);
    PopulateGpsPosMappings(m_lookupInfo, m_indexInfo, DID_GPS1_POS);
    PopulateGpsPosMappings(m_lookupInfo, m_indexInfo, DID_GPS1_RCVR_POS);
    PopulateGpsPosMappings(m_lookupInfo, m_indexInfo, DID_GPS2_POS);
    PopulateGpsPosMappings(m_lookupInfo, m_indexInfo, DID_GPS1_RTK_POS);
    PopulateGpsVelMappings(m_lookupInfo, m_indexInfo, DID_GPS1_VEL);
    PopulateGpsVelMappings(m_lookupInfo, m_indexInfo, DID_GPS2_VEL);
#if 0	// Too much data, we don't want to log this. WHJ
    PopulateGpsSatMappings(m_lookupInfo, m_indexInfo, DID_GPS1_SAT);
    PopulateGpsSatMappings(m_lookupInfo, m_indexInfo, DID_GPS2_SAT);
    PopulateGpsSigMappings(m_lookupInfo, m_indexInfo, DID_GPS1_SIG);
    PopulateGpsSigMappings(m_lookupInfo, m_indexInfo, DID_GPS2_SIG);
#endif
    PopulateGpsRtkRelMappings(m_lookupInfo, m_indexInfo, DID_GPS1_RTK_POS_REL);
    PopulateGpsRtkRelMappings(m_lookupInfo, m_indexInfo, DID_GPS2_RTK_CMP_REL);
    PopulateGpsRtkMiscMappings(m_lookupInfo, m_indexInfo, DID_GPS1_RTK_POS_MISC);
    PopulateGpsRtkMiscMappings(m_lookupInfo, m_indexInfo, DID_GPS2_RTK_CMP_MISC);
    PopulateGpsRawMappings(m_lookupInfo, m_indexInfo, DID_GPS1_RAW);
    PopulateGpsRawMappings(m_lookupInfo, m_indexInfo, DID_GPS2_RAW);
    PopulateGpsRawMappings(m_lookupInfo, m_indexInfo, DID_GPS_BASE_RAW);
    PopulateGroundVehicleMappings(m_lookupInfo, m_indexInfo);
    PopulateConfigMappings(m_lookupInfo, m_indexInfo);
    PopulateFlashConfigMappings(m_lookupInfo, m_indexInfo);
    PopulateDebugArrayMappings(m_lookupInfo, m_indexInfo, DID_DEBUG_ARRAY);
    PopulateDebugStringMappings(m_lookupInfo, m_indexInfo, DID_DEBUG_STRING);
    PopulateEvbStatusMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbFlashCfgMappings(m_lookupInfo, m_indexInfo);
    PopulateDebugArrayMappings(m_lookupInfo, m_indexInfo, DID_EVB_DEBUG_ARRAY);
    PopulateDeviceInfoMappings(m_lookupInfo, m_indexInfo, DID_EVB_DEV_INFO);
    PopulateIOMappings(m_lookupInfo, m_indexInfo);
    PopulateIMUMappings(m_lookupInfo, m_indexInfo, DID_REFERENCE_IMU, "Reference IMU.");
    PopulateIMUDeltaThetaVelocityMappings(m_lookupInfo, m_indexInfo, DID_REFERENCE_PIMU, "Reference PIMU.");
    PopulateInfieldCalMappings(m_lookupInfo, m_indexInfo);
    PopulateInl2MagObsInfo(m_lookupInfo, m_indexInfo);
    PopulateInl2StatesMappings(m_lookupInfo, m_indexInfo);

    PopulateSensorsADCMappings(m_lookupInfo, m_indexInfo);
    PopulateSensorsISMappings(m_lookupInfo, m_indexInfo, DID_SENSORS_UCAL);
    PopulateSensorsISMappings(m_lookupInfo, m_indexInfo, DID_SENSORS_TCAL);
    PopulateSensorsISMappings(m_lookupInfo, m_indexInfo, DID_SENSORS_MCAL);
    PopulateSensorsTCMappings(m_lookupInfo, m_indexInfo);
    PopulateSensorsCompMappings(m_lookupInfo, m_indexInfo);

    PopulateISEventMappings(m_lookupInfo, m_indexInfo);

    PopulateDeviceInfoMappings(m_lookupInfo, m_indexInfo, DID_GPX_DEV_INFO);
    PopulateGpxFlashCfgMappings(m_lookupInfo, m_indexInfo);
    // DID_GPX_RTOS_INFO
    PopulateGpxStatusMappings(m_lookupInfo, m_indexInfo);
    PopulateDebugArrayMappings(m_lookupInfo, m_indexInfo, DID_GPX_DEBUG_ARRAY);
    PopulateStrobeInTimeMappings(m_lookupInfo, m_indexInfo);
//    PopulateRtosInfoMappings(m_lookupInfo, m_indexInfo);
    PopulateDiagMsgMappings(m_lookupInfo, m_indexInfo);
    PopulateCanConfigMappings(m_lookupInfo, m_indexInfo);

#ifdef USE_IS_INTERNAL
    PopulateUserPage0Mappings(m_lookupInfo, m_indexInfo);
    PopulateUserPage1Mappings(m_lookupInfo, m_indexInfo);
//     PopulateRtkStateMappings(m_lookupInfo, m_indexInfo);
//     PopulateRtkResidualMappings(m_lookupInfo, m_indexInfo, DID_RTK_CODE_RESIDUAL);
//     PopulateRtkResidualMappings(m_lookupInfo, m_indexInfo, DID_RTK_PHASE_RESIDUAL);
    PopulateRtkDebugMappings(m_lookupInfo, m_indexInfo);
    // PopulateRtkDebug2Mappings(m_lookupInfo, m_indexInfo);
#endif

#if defined(INCLUDE_LUNA_DATA_SETS)
    PopulateEvbLunaFlashCfgMappings(m_lookupInfo, m_indexInfo);
    PopulateCoyoteStatusMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbLunaSensorsMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbLunaVelocityControlMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbLunaVelocityCommandMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbLunaAuxCmdMappings(m_lookupInfo, m_indexInfo);
#endif

    // this must come last
    for (uint32_t id = 0; id < DID_COUNT; id++)
    {
        PopulateTimestampField(id, m_timestampFields, m_lookupInfo);
    }
}


cISDataMappings::~cISDataMappings()
{
}


const char* cISDataMappings::GetName(uint32_t dataId)
{
    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(m_dataIdNames) == DID_COUNT);

    if (dataId >= DID_COUNT)
    {
        return "unknown";
    }

    return m_dataIdNames[dataId];
}


uint32_t cISDataMappings::GetId(string name)
{
//     transform(name.begin(), name.end(), name.begin(), ::toupper);

    for (eDataIDs id = 0; id < DID_COUNT; id++)
    {
        if (strcmp(name.c_str(), m_dataIdNames[id]) == 0)
        {    // Found match
            return id;
        }
    }

    return 0;
}


const map_name_to_info_t* cISDataMappings::GetMapInfo(uint32_t dataId)
{
    if (dataId >= DID_COUNT)
    {
        return NULLPTR;
    }

#if PLATFORM_IS_EMBEDDED

    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

    return &s_map->m_lookupInfo[dataId];

#else

    return &s_map.m_lookupInfo[dataId];

#endif
}


const map_index_to_info_t* cISDataMappings::GetIndexMapInfo(uint32_t dataId)
{
    if (dataId >= DID_COUNT)
    {
        return NULLPTR;
    }

#if PLATFORM_IS_EMBEDDED

    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

    return &s_map->m_indexInfo[dataId];

#else

    return &s_map.m_indexInfo[dataId];

#endif
}


// const data_info_t* cISDataMappings::GetFieldDataInfo(uint32_t dataId, uint32_t field)
// {
//     if (dataId >= DID_COUNT)
//     {
//         return NULLPTR;
//     }

// #if PLATFORM_IS_EMBEDDED

//     if (s_map == NULLPTR)
//     {
//         s_map = new cISDataMappings();
//     }

//     return s_map->m_indexInfo[dataId][field];

// #else

//     return s_map.m_indexInfo[dataId][field];

// #endif
// }


uint32_t cISDataMappings::GetSize(uint32_t dataId)
{
    if (dataId >= DID_COUNT)
    {
        return 0;
    }

#if PLATFORM_IS_EMBEDDED
    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

    return s_map->m_lookupSize[dataId];
#else
    return s_map.m_lookupSize[dataId];
#endif
}


bool cISDataMappings::StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* datasetBuffer, const data_info_t& info, int radix, bool json)
{
    const uint8_t* ptr;
    if (!CanGetFieldData(info, hdr, datasetBuffer, ptr))
    {
        return false;
    }

    return StringToVariable(stringBuffer, stringLength, ptr, info.dataType, info.dataSize, radix, json);
}

bool cISDataMappings::StringToVariable(const char* stringBuffer, int stringLength, const uint8_t* dataBuffer, eDataType dataType, uint32_t dataSize, int radix, bool json)
{
    switch (dataType)
    {
    case DATA_TYPE_INT8:
        protectUnalignedAssign<int8_t>((void*)dataBuffer, (int8_t)strtol(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_INT16:
        protectUnalignedAssign<int16_t>((void*)dataBuffer, (int16_t)strtol(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_INT32:
        protectUnalignedAssign<int32_t>((void*)dataBuffer, (int32_t)strtol(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_UINT8:
        protectUnalignedAssign<uint8_t>((void*)dataBuffer, (uint8_t)strtoul(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_UINT16:
        protectUnalignedAssign<uint16_t>((void*)dataBuffer, (uint16_t)strtoul(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_UINT32:
        protectUnalignedAssign<uint32_t>((void*)dataBuffer, (uint32_t)strtoul(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_INT64:
        protectUnalignedAssign<int64_t>((void*)dataBuffer, (int64_t)strtoll(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_UINT64:
        protectUnalignedAssign<uint64_t>((void*)dataBuffer, (uint64_t)strtoull(stringBuffer, NULL, radix));
        break;

    case DATA_TYPE_F32:
        protectUnalignedAssign<float>((void*)dataBuffer, (float)strtod(stringBuffer, NULL));
        break;

    case DATA_TYPE_F64:
        protectUnalignedAssign<double>((void*)dataBuffer, (double)strtod(stringBuffer, NULL));
        break;

    case DATA_TYPE_STRING:
    {
        string s2(stringBuffer);
        if (json)
        {
            char c;
            bool escaped = false;
            for (size_t i = 0; i < s2.size(); i++)
            {
                c = s2[i];
                if (c == '\\')
                {
                    if (!escaped)
                    {
                        escaped = true;
                        s2.erase(i);
                        continue;
                    }
                }
                escaped = false;
            }
            s2 = stringBuffer;
        }
        else
        {
            s2.erase(std::remove(s2.begin(), s2.end(), '"'), s2.end());
        }
        // ensure string fits with null terminator
        s2.resize(dataSize - 1);
        memcpy((void*)dataBuffer, s2.data(), s2.length());
        memset((uint8_t*)dataBuffer + s2.length(), 0, dataSize - s2.length());
    } break;

    case DATA_TYPE_BINARY:
    {
        // convert hex data back to binary
        size_t len = _MIN(1020, stringLength);
        len -= (len % 2);
        uint8_t* ptr2 = (uint8_t*)dataBuffer;
        for (size_t i = 0; i != len; i += 2)
        {
            *ptr2 = (getHexValue(stringBuffer[i + 1]) << 4) | getHexValue(stringBuffer[i + 2]);
        }
    } break;

    default:
        return false;
    }

    return true;
}


bool cISDataMappings::DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* datasetBuffer, data_mapping_string_t stringBuffer, bool json)
{
    const uint8_t* ptr;
    if (!CanGetFieldData(info, hdr, datasetBuffer, ptr))
    {
        // pick a default string
        if (info.dataType == DATA_TYPE_STRING)
        {
            stringBuffer[0] = '"';
            stringBuffer[1] = '"';
            stringBuffer[2] = '\0';
        }
        else if (info.dataType == DATA_TYPE_BINARY)
        {
            if (json)
            {
                stringBuffer[0] = '"';
                stringBuffer[1] = '"';
                stringBuffer[2] = '\0';
            }
            else
            {
                stringBuffer[0] = '\0';
            }
        }
        else
        {
            stringBuffer[0] = '0';
            stringBuffer[1] = '\0';
        }
        return false;
    }

    return VariableToString(info.dataType, info.dataFlags, ptr, datasetBuffer, info.dataSize, stringBuffer, json);
}


bool cISDataMappings::VariableToString(eDataType dataType, eDataFlags dataFlags, const uint8_t* ptr, const uint8_t* dataBuffer, uint32_t dataSize, data_mapping_string_t stringBuffer, bool json)
{
    int precision;
    switch (dataType)
    {
    case DATA_TYPE_INT8:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(int8_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int8_t*)ptr);
        break;
    case DATA_TYPE_UINT8:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(uint8_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint8_t*)ptr);
        break;
    case DATA_TYPE_INT16:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(int16_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int16_t*)ptr);
        break;
    case DATA_TYPE_UINT16:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(uint16_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint16_t*)ptr);
        break;
    case DATA_TYPE_INT32:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(int32_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int32_t*)ptr);
        break;
    case DATA_TYPE_UINT32:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(uint32_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint32_t*)ptr);        
        break;
    case DATA_TYPE_INT64:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (long long)*(uint64_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%lld", (long long)*(int64_t*)ptr);
        break;
    case DATA_TYPE_UINT64:
        if (dataFlags & DATA_FLAGS_DISPLAY_HEX) SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (unsigned long long)*(uint64_t*)ptr);
        else                                    SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%llu", (unsigned long long)*(uint64_t*)ptr);
        break;
    case DATA_TYPE_F32:
        precision = (dataFlags&DATA_FLAGS_FIXED_DECIMAL_MASK);
        if (precision == 0) { precision = 9;  }     // Default to 9 digits
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.*f", precision, *(float*)ptr);
        break;
    case DATA_TYPE_F64:                             
        precision = (dataFlags&DATA_FLAGS_FIXED_DECIMAL_MASK);
        if (precision == 0) { precision = 17; }     // Default to 17 digits
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.*f", precision, *(double*)ptr);
        break;

    case DATA_TYPE_STRING:
    {
        stringBuffer[0] = '"';
        int tempIndex = 1;
        char* bufPtr2 = (char*)(ptr);
        char* bufPtrEnd = bufPtr2 + _MIN(IS_DATA_MAPPING_MAX_STRING_LENGTH, dataSize) - 3;
        for (; bufPtr2 < bufPtrEnd && *bufPtr2 != '\0'; bufPtr2++)
        {
            if (json)
            {
                if (IS_JSON_ESCAPE_CHAR(*bufPtr2))
                {
                    if (bufPtr2 < bufPtrEnd - 1)
                    {
                        stringBuffer[tempIndex++] = '\\';
                    }
                    else
                    {
                        break;
                    }
                }
            }
            stringBuffer[tempIndex++] = *bufPtr2;
        }
        stringBuffer[tempIndex++] = '"';
        stringBuffer[tempIndex] = '\0';
    } break;

    case DATA_TYPE_BINARY:
    {
        size_t hexIndex = 1;
        if (json)
        {
            stringBuffer[0] = '"';
        }
        // convert to hex
        const unsigned char* hexTable = getHexLookupTable();
        for (size_t i = 0; i < dataSize; i++)
        {
            stringBuffer[hexIndex++] = hexTable[0x0F & (dataBuffer[i] >> 4)];
            stringBuffer[hexIndex++] = hexTable[0x0F & dataBuffer[i]];
        }
        if (json)
        {
            stringBuffer[hexIndex++] = '"';
        }
        stringBuffer[hexIndex] = '\0';
    } break;

    default:
        stringBuffer[0] = '\0';
        return false;
    }
    return true;
}

double cISDataMappings::GetTimestamp(const p_data_hdr_t* hdr, const uint8_t* buf)
{
    if (hdr == NULL || buf == NULL || hdr->id == 0 || hdr->id >= DID_COUNT || hdr->size == 0)
    {
        return 0.0;
    }
    
    // raw data types with observation use a custom timestamp function
    if (hdr->id == DID_GPS1_RAW || hdr->id == DID_GPS2_RAW || hdr->id == DID_GPS_BASE_RAW)
    {
        gps_raw_t* raw = (gps_raw_t*)buf;
        if (raw->dataType == eRawDataType::raw_data_type_observation && raw->obsCount>0)
        {
            const obsd_t& obs = raw->data.obs[0];
            return obs.time.sec + (double)obs.time.time;
        }
        return 0.0;
    }

#if PLATFORM_IS_EMBEDDED

    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

#endif

    const data_info_t* timeStampField =
    
#if PLATFORM_IS_EMBEDDED

        s_map->m_timestampFields[hdr->id];

#else

        s_map.m_timestampFields[hdr->id];

#endif

    if (timeStampField != NULLPTR)
    {
        const uint8_t* ptr;
        if (CanGetFieldData(*timeStampField, hdr, (uint8_t*)buf, ptr))
        {
            if (timeStampField->dataType == DATA_TYPE_F64)
            {
                // field is seconds, use as is
                return protectUnalignedAssign<double>((void *)ptr);
            }
            else if (timeStampField->dataType == DATA_TYPE_UINT32)
            {
                // field is milliseconds, convert to seconds
                return 0.001 * (*(uint32_t*)ptr);
            }
        }
    }
    return 0.0;
}

bool cISDataMappings::CanGetFieldData(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* buf, const uint8_t*& ptr)
{
    if (buf == NULL)
    {
        return false;
    }
    else if (hdr == NULL)
    {
        // assume buf is large enough for the full data structure
        ptr = buf + info.dataOffset;
        return true;
    }
    int32_t fullSize = (hdr->size == 0 ? GetSize(hdr->id) : hdr->size);
    int32_t offset = (int32_t)info.dataOffset - (int32_t)hdr->offset;
    if (offset >= 0 && 
        offset <= (fullSize - (int32_t)info.dataSize))
    {
        ptr = buf + offset;
        return true;
    }
    ptr = NULL;
    return false;
}
