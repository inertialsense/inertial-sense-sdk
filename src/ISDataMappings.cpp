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

#ifdef USE_IS_INTERNAL
#include "../../cpp/libs/families/imx/IS_internal.h"
#endif

using namespace std;

#define SYM_DEG             "°"
#define SYM_DEG_C           "°C"
#define SYM_DEG_DEG_M       "°,°,m"
#define SYM_DEG_PER_S       "°/s"
#define SYM_M_PER_S         "m/s"

const char insStatusDescription[] = "INS Status flags [0,0,MagStatus,SolStatus,     NavMode,GpsMagUsed,Variance,VarianceCoarse]";
const char hdwStatusDescription[] = "Hdw Status flags [Fault,BIT,RxErrCount,ComErr, SenSatHist,SensorSat,GpsSatRx,Motion]";


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

CONST_EXPRESSION uint32_t s_eDataTypeSizes[DATA_TYPE_COUNT] =
{
    (uint32_t)sizeof(int8_t),
    (uint32_t)sizeof(uint8_t),
    (uint32_t)sizeof(int16_t),
    (uint32_t)sizeof(uint16_t),
    (uint32_t)sizeof(int32_t),
    (uint32_t)sizeof(uint32_t),
    (uint32_t)sizeof(int64_t),
    (uint32_t)sizeof(uint64_t),
    (uint32_t)sizeof(float),
    (uint32_t)sizeof(double),
    (uint32_t)0, // string, must be set to actual size by caller
    (uint32_t)0  // binary, must be set to actual size by caller
};

#define INIT_MAP(dtype, id) \
    typedef dtype MAP_TYPE; \
    map_name_to_info_t& map = mappings[(id)]; \
    map_index_to_info_t& idx = indices[(id)]; \
    uint32_t totalSize = 0; \
    uint32_t fieldCount = 0;

#if CPP11_IS_ENABLED

// dataSize can be 0 for default size, must be set for string type
#define ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, units, description, flags, conversion)  map[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)sizeof(fieldType), (dataType), (eDataFlags)(flags), (name), (units), (description), (conversion) }; idx[fieldCount++] = &(map[std::string(name)]); totalSize += sizeof(fieldType);

// note when passing member type for arrays, it must be a reference, i.e. float&
#define ADD_MAP_4(name, member, dataType, fieldType) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, "", "", 0, 1.0); \
    static_assert(is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP_5(name, member, dataType, fieldType, dataFlags) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, "", "", dataFlags, 1.0); \
    static_assert(is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP_6(name, member, dataType, fieldType, units, description) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, units, description, 0, 1.0); \
    static_assert(is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP_7(name, member, dataType, fieldType, units, description, dataFlags) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, units, description, dataFlags, 1.0); \
    static_assert(is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP_8(name, member, dataType, fieldType, units, description, dataFlags, conversion) \
    ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, units, description, dataFlags, conversion); \
    static_assert(is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)sizeof(fieldType) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)sizeof(fieldType) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ASSERT_SIZE(s) assert(s == sizeof(MAP_TYPE))

#else

#define ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, dataFlags) map[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)sizeof(fieldType), dataType, (eDataFlags)dataFlags, name }; totalSize += sizeof(fieldType);
#define ADD_MAP_4(name, member, dataType, fieldType, dataFlags) ADD_MAP_NO_VALIDATION(name, member, dataType, fieldType, dataFlags)
#define ASSERT_SIZE(s) // not supported on VS < 2015

#endif


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
    sizeMap[DID_IO] = sizeof(io_t);
    sizeMap[DID_INFIELD_CAL] = sizeof(infield_cal_t);
    sizeMap[DID_REFERENCE_IMU] = sizeof(imu_t);
    sizeMap[DID_REFERENCE_PIMU] = sizeof(pimu_t);
    sizeMap[DID_REFERENCE_MAGNETOMETER] = sizeof(magnetometer_t);

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

    sizeMap[DID_SENSORS_UCAL] = sizeof(sensors_w_temp_t);
    sizeMap[DID_SENSORS_TCAL] = sizeof(sensors_w_temp_t);
    sizeMap[DID_SENSORS_MCAL] = sizeof(sensors_w_temp_t);
    sizeMap[DID_SENSORS_TC_BIAS] = sizeof(sensors_t);
    sizeMap[DID_SCOMP] = sizeof(sensor_compensation_t);
    sizeMap[DID_RTK_DEBUG] = sizeof(rtk_debug_t);
//     sizeMap[DID_RTK_STATE] = sizeof(rtk_state_t);
    sizeMap[DID_RTK_CODE_RESIDUAL] = sizeof(rtk_residual_t);
    sizeMap[DID_RTK_PHASE_RESIDUAL] = sizeof(rtk_residual_t);
    sizeMap[DID_NVR_USERPAGE_G0] = sizeof(nvm_group_0_t);
    sizeMap[DID_NVR_USERPAGE_G1] = sizeof(nvm_group_1_t);
    sizeMap[DID_INL2_STATES] = sizeof(inl2_states_t);
    sizeMap[DID_INL2_STATUS] = sizeof(inl2_status_t);
    sizeMap[DID_INL2_MISC] = sizeof(inl2_misc_t);
    sizeMap[DID_INL2_MAG_OBS_INFO] = sizeof(inl2_mag_obs_info_t);
    sizeMap[DID_IMU_MAG] = sizeof(imu_mag_t);
    sizeMap[DID_PIMU_MAG] = sizeof(pimu_mag_t);
    sizeMap[DID_SENSORS_ADC] = sizeof(sys_sensors_adc_t);
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

// Stringify the macro value
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

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

    ADD_MAP_4("status", status, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("g1Task", g1Task, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("g2FileNum", g2FileNum, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("g3LineNum", g3LineNum, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("g4", g4, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("g5Lr", g5Lr, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pc", pc, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("psr", psr, DATA_TYPE_UINT32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t dataId)
{
    INIT_MAP(imu_t, dataId);
    
    ADD_MAP_4("time", time, DATA_TYPE_F64, double);
    ADD_MAP_4("pqr[0]", I.pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr[1]", I.pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr[2]", I.pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc[0]", I.acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc[1]", I.acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc[2]", I.acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMU3Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t dataId)
{
    INIT_MAP(imu3_t, dataId);

    ADD_MAP_4("time", time, DATA_TYPE_F64, double);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("I0.pqr[0]", I[0].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("I0.pqr[1]", I[0].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("I0.pqr[2]", I[0].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("I0.acc[0]", I[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("I0.acc[1]", I[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("I0.acc[2]", I[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("I1.pqr[0]", I[1].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("I1.pqr[1]", I[1].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("I1.pqr[2]", I[1].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("I1.acc[0]", I[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("I1.acc[1]", I[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("I1.acc[2]", I[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("I2.pqr[0]", I[2].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("I2.pqr[1]", I[2].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("I2.pqr[2]", I[2].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("I2.acc[0]", I[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("I2.acc[1]", I[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("I2.acc[2]", I[2].acc[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysParamsMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sys_params_t, DID_SYS_PARAMS);
    
    ADD_MAP_7("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t, "ms", "Time of week since Sunday morning, GMT", DATA_FLAGS_READ_ONLY);
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t, "", insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t, "", hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
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
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t,  "", insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t,  "", hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
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
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t,  "", insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t,  "", hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
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
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t,  "", insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t,  "", hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
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
    ADD_MAP_7("insStatus", insStatus, DATA_TYPE_UINT32, uint32_t,  "", insStatusDescription, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_INS_STATUS);
    ADD_MAP_7("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t,  "", hdwStatusDescription, DATA_FLAGS_DISPLAY_HEX);
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
    
    ADD_MAP_4("week", week, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_GPS_STATUS);
    ADD_MAP_4("ecef[0]", ecef[0], DATA_TYPE_F64, double&);
    ADD_MAP_4("ecef[1]", ecef[1], DATA_TYPE_F64, double&);
    ADD_MAP_4("ecef[2]", ecef[2], DATA_TYPE_F64, double&);
    ADD_MAP_4("lla[0]", lla[0], DATA_TYPE_F64, double&);
    ADD_MAP_4("lla[1]", lla[1], DATA_TYPE_F64, double&);
    ADD_MAP_4("lla[2]", lla[2], DATA_TYPE_F64, double&);
    ADD_MAP_4("hMSL", hMSL, DATA_TYPE_F32, float);
    ADD_MAP_4("hAcc", hAcc, DATA_TYPE_F32, float);
    ADD_MAP_4("vAcc", vAcc, DATA_TYPE_F32, float);
    ADD_MAP_4("pDop", pDop, DATA_TYPE_F32, float);
    ADD_MAP_4("cnoMean", cnoMean, DATA_TYPE_F32, float);
    ADD_MAP_4("towOffset", towOffset, DATA_TYPE_F64, double);
    ADD_MAP_4("leapS", leapS, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("satsUsed", satsUsed, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("cnoMeanSigma", cnoMeanSigma, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("reserved", reserved, DATA_TYPE_UINT8, uint8_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsVelMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_vel_t, id);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("vel[0]", vel[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("vel[1]", vel[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("vel[2]", vel[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("sAcc", sAcc, DATA_TYPE_F32, float);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsSatMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_sat_t, id);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("numSats", numSats, DATA_TYPE_UINT32, uint32_t);

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
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("numSigs", numSigs, DATA_TYPE_UINT32, uint32_t);

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
    
    ADD_MAP_4("time", time, DATA_TYPE_F64, double);
    ADD_MAP_4("mag[0]", mag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag[1]", mag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag[2]", mag[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateBarometerMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(barometer_t, DID_BAROMETER);
    
    ADD_MAP_4("time", time, DATA_TYPE_F64, double);
    ADD_MAP_4("bar", bar, DATA_TYPE_F32, float);
    ADD_MAP_4("mslBar", mslBar, DATA_TYPE_F32, float);
    ADD_MAP_4("barTemp", barTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("humidity", humidity, DATA_TYPE_F32, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t did)
{
    INIT_MAP(pimu_t, did);
    
    ADD_MAP_4("time", time, DATA_TYPE_F64, double);
    ADD_MAP_4("theta[0]", theta[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("theta[1]", theta[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("theta[2]", theta[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("vel[0]", vel[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("vel[1]", vel[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("vel[2]", vel[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("dt", dt, DATA_TYPE_F32, float);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMagMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(pimu_mag_t, DID_PIMU_MAG);
    
    ADD_MAP_4("imutime", pimu.time, DATA_TYPE_F64, double);
    ADD_MAP_4("theta[0]", pimu.theta[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("theta[1]", pimu.theta[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("theta[2]", pimu.theta[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("vel[0]", pimu.vel[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("vel[1]", pimu.vel[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("vel[2]", pimu.vel[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("dt", pimu.dt, DATA_TYPE_F32, float);
    ADD_MAP_4("imustatus", pimu.status, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("magtime", mag.time, DATA_TYPE_F64, double);
    ADD_MAP_4("mag[0]", mag.mag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag[1]", mag.mag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag[2]", mag.mag[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMagnetometerMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(imu_mag_t, DID_IMU_MAG);
    
    ADD_MAP_4("imutime", imu.time, DATA_TYPE_F64, double);
    ADD_MAP_4("pqr[0]", imu.I.pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr[1]", imu.I.pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr[2]", imu.I.pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc[0]", imu.I.acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc[1]", imu.I.acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc[2]", imu.I.acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("imustatus", imu.status, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("magtime", mag.time, DATA_TYPE_F64, double);
    ADD_MAP_4("mag[0]", mag.mag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag[1]", mag.mag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag[2]", mag.mag[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);

}


static void PopulateInfieldCalMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(infield_cal_t, DID_INFIELD_CAL);
    
    ADD_MAP_4("state", state, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("sampleTimeMs", sampleTimeMs, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("imu[0].pqr[0]", imu[0].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[0].pqr[1]", imu[0].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[0].pqr[2]", imu[0].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[0].acc[0]", imu[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[0].acc[1]", imu[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[0].acc[2]", imu[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[1].pqr[0]", imu[1].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[1].pqr[1]", imu[1].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[1].pqr[2]", imu[1].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[1].acc[0]", imu[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[1].acc[1]", imu[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[1].acc[2]", imu[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[2].pqr[0]", imu[2].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[2].pqr[1]", imu[2].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[2].pqr[2]", imu[2].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[2].acc[0]", imu[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[2].acc[1]", imu[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("imu[2].acc[2]", imu[2].acc[2], DATA_TYPE_F32, float&);

    ADD_MAP_4("calData[0].down.dev[0].acc[0]", calData[0].down.dev[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.dev[0].acc[1]", calData[0].down.dev[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.dev[0].acc[2]", calData[0].down.dev[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.dev[1].acc[0]", calData[0].down.dev[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.dev[1].acc[1]", calData[0].down.dev[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.dev[1].acc[2]", calData[0].down.dev[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.dev[2].acc[0]", calData[0].down.dev[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.dev[2].acc[1]", calData[0].down.dev[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.dev[2].acc[2]", calData[0].down.dev[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].down.yaw", calData[0].down.yaw, DATA_TYPE_F32, float);
    ADD_MAP_4("calData[0].up.dev[0].acc[0]", calData[0].up.dev[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.dev[0].acc[1]", calData[0].up.dev[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.dev[0].acc[2]", calData[0].up.dev[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.dev[1].acc[0]", calData[0].up.dev[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.dev[1].acc[1]", calData[0].up.dev[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.dev[1].acc[2]", calData[0].up.dev[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.dev[2].acc[0]", calData[0].up.dev[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.dev[2].acc[1]", calData[0].up.dev[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.dev[2].acc[2]", calData[0].up.dev[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[0].up.yaw", calData[0].up.yaw, DATA_TYPE_F32, float);

    ADD_MAP_4("calData[1].down.dev[0].acc[0]", calData[1].down.dev[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.dev[0].acc[1]", calData[1].down.dev[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.dev[0].acc[2]", calData[1].down.dev[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.dev[1].acc[0]", calData[1].down.dev[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.dev[1].acc[1]", calData[1].down.dev[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.dev[1].acc[2]", calData[1].down.dev[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.dev[2].acc[0]", calData[1].down.dev[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.dev[2].acc[1]", calData[1].down.dev[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.dev[2].acc[2]", calData[1].down.dev[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].down.yaw", calData[1].down.yaw, DATA_TYPE_F32, float);
    ADD_MAP_4("calData[1].up.dev[0].acc[0]", calData[1].up.dev[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.dev[0].acc[1]", calData[1].up.dev[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.dev[0].acc[2]", calData[1].up.dev[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.dev[1].acc[0]", calData[1].up.dev[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.dev[1].acc[1]", calData[1].up.dev[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.dev[1].acc[2]", calData[1].up.dev[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.dev[2].acc[0]", calData[1].up.dev[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.dev[2].acc[1]", calData[1].up.dev[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.dev[2].acc[2]", calData[1].up.dev[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[1].up.yaw", calData[1].up.yaw, DATA_TYPE_F32, float);

    ADD_MAP_4("calData[2].down.dev[0].acc[0]", calData[2].down.dev[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.dev[0].acc[1]", calData[2].down.dev[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.dev[0].acc[2]", calData[2].down.dev[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.dev[1].acc[0]", calData[2].down.dev[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.dev[1].acc[1]", calData[2].down.dev[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.dev[1].acc[2]", calData[2].down.dev[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.dev[2].acc[0]", calData[2].down.dev[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.dev[2].acc[1]", calData[2].down.dev[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.dev[2].acc[2]", calData[2].down.dev[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].down.yaw", calData[2].down.yaw, DATA_TYPE_F32, float);
    ADD_MAP_4("calData[2].up.dev[0].acc[0]", calData[2].up.dev[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.dev[0].acc[1]", calData[2].up.dev[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.dev[0].acc[2]", calData[2].up.dev[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.dev[1].acc[0]", calData[2].up.dev[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.dev[1].acc[1]", calData[2].up.dev[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.dev[1].acc[2]", calData[2].up.dev[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.dev[2].acc[0]", calData[2].up.dev[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.dev[2].acc[1]", calData[2].up.dev[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.dev[2].acc[2]", calData[2].up.dev[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("calData[2].up.yaw", calData[2].up.yaw, DATA_TYPE_F32, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateReferenceIMUMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(imu_t, DID_REFERENCE_IMU);
    
    ADD_MAP_4("time", time, DATA_TYPE_F64, double);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("I.pqr[0]", I.pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("I.pqr[1]", I.pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("I.pqr[2]", I.pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("I.acc[0]", I.acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("I.acc[1]", I.acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("I.acc[2]", I.acc[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateWheelEncoderMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(wheel_encoder_t, DID_WHEEL_ENCODER);
    
    ADD_MAP_4("timeOfWeek", timeOfWeek, DATA_TYPE_F64, double);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("theta_l", theta_l, DATA_TYPE_F32, float);
    ADD_MAP_4("omega_l", omega_l, DATA_TYPE_F32, float);
    ADD_MAP_4("theta_r", theta_r, DATA_TYPE_F32, float);
    ADD_MAP_4("omega_r", omega_r, DATA_TYPE_F32, float);
    ADD_MAP_4("wrap_count_l", wrap_count_l, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("wrap_count_r", wrap_count_r, DATA_TYPE_UINT32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateGroundVehicleMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ground_vehicle_t, DID_GROUND_VEHICLE);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("status", status, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("mode", mode, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("wheelConfig.bits", wheelConfig.bits, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("wheelConfig.track_width", wheelConfig.track_width, DATA_TYPE_F32, float);
    ADD_MAP_4("wheelConfig.radius", wheelConfig.radius, DATA_TYPE_F32, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateConfigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(system_command_t, DID_SYS_CMD);
    
    ADD_MAP_4("command", command, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("invCommand", invCommand, DATA_TYPE_UINT32, uint32_t);

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

    ADD_MAP_4("Time stamp of message (System Up seconds)", time, DATA_TYPE_F64, double);
    ADD_MAP_4("Senders serial number", senderSN, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("Sender hardware type", senderHdwId, DATA_TYPE_UINT16, uint16_t);

    ADD_MAP_4("Message ID", msgTypeID, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("Priority", priority, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("Length", length, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("data", data, DATA_TYPE_STRING, uint8_t[MEMBERSIZE(MAP_TYPE, data)]);

    ADD_MAP_4("Reserved 8 bit", res8, DATA_TYPE_UINT8, uint8_t);
}

static void PopulateGpxFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(gpx_flash_cfg_t, DID_GPX_FLASH_CFG);
    
    ADD_MAP_4("size", size, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("checksum", checksum, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("key", key, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("ser0BaudRate", ser0BaudRate, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("ser1BaudRate", ser1BaudRate, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("ser2BaudRate", ser2BaudRate, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("startupGPSDtMs", startupGPSDtMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("gps1AntOffset[0]", gps1AntOffset[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("gps1AntOffset[1]", gps1AntOffset[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("gps1AntOffset[2]", gps1AntOffset[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("gps2AntOffset[0]", gps2AntOffset[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("gps2AntOffset[1]", gps2AntOffset[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("gps2AntOffset[2]", gps2AntOffset[2], DATA_TYPE_F32, float&);
    ADD_MAP_5("gnssSatSigConst", gnssSatSigConst, DATA_TYPE_UINT16, uint16_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("dynamicModel", dynamicModel, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("debug", debug, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("gpsTimeSyncPeriodMs", gpsTimeSyncPeriodMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("gpsTimeUserDelay", gpsTimeUserDelay, DATA_TYPE_F32, float);
    ADD_MAP_4("gpsMinimumElevation", gpsMinimumElevation, DATA_TYPE_F32, float);
    ADD_MAP_5("RTKCfgBits", RTKCfgBits, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpxStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(gpx_status_t, DID_GPX_STATUS);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("status", status, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("grmcBitsSer0", grmcBitsSer0, DATA_TYPE_UINT64, uint64_t);
    ADD_MAP_4("grmcBitsSer1", grmcBitsSer1, DATA_TYPE_UINT64, uint64_t);
    ADD_MAP_4("grmcBitsSer2", grmcBitsSer2, DATA_TYPE_UINT64, uint64_t);
    ADD_MAP_4("grmcBitsUSB", grmcBitsUSB, DATA_TYPE_UINT64, uint64_t);
    ADD_MAP_4("grmcNMEABitsSer0", grmcNMEABitsSer0, DATA_TYPE_UINT64, uint64_t);
    ADD_MAP_4("grmcNMEABitsSer1", grmcNMEABitsSer1, DATA_TYPE_UINT64, uint64_t);
    ADD_MAP_4("grmcNMEABitsSer2", grmcNMEABitsSer2, DATA_TYPE_UINT64, uint64_t);
    ADD_MAP_4("grmcNMEABitsUSB", grmcNMEABitsUSB, DATA_TYPE_UINT64, uint64_t);
    ADD_MAP_4("hdwStatus", hdwStatus, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("mcuTemp", mcuTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("navOutputPeriodMs", navOutputPeriodMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("flashCfgChecksum", flashCfgChecksum, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("rtkMode", rtkMode, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("gnss1RunState", gnss1RunState, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("gnss2RunState", gnss2RunState, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("SourcePort", gpxSourcePort, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("upTime", upTime, DATA_TYPE_F64, double);
}

static void PopulateEvbStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_status_t, DID_EVB_STATUS);
    
    ADD_MAP_4("week", week, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("firmwareVer[0]", firmwareVer[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("firmwareVer[1]", firmwareVer[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("firmwareVer[2]", firmwareVer[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("firmwareVer[3]", firmwareVer[3], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_5("evbStatus", evbStatus, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("loggerMode", loggerMode, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("loggerElapsedTimeMs", loggerElapsedTimeMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("wifiIpAddr", wifiIpAddr, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("sysCommand", sysCommand, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("towOffset", towOffset, DATA_TYPE_F64, double);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_flash_cfg_t, DID_EVB_FLASH_CFG);
    
    ADD_MAP_4("size", size, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("checksum", checksum, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("key", key, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("cbPreset", cbPreset, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("reserved1[0]", reserved1[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved1[1]", reserved1[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved1[2]", reserved1[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_5("cbf[0]", cbf[0], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[1]", cbf[1], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[2]", cbf[2], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[3]", cbf[3], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[4]", cbf[4], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[5]", cbf[5], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[6]", cbf[6], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[7]", cbf[7], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[8]", cbf[8], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbf[9]", cbf[9], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("cbOptions", cbOptions, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("bits", bits, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("radioPID", radioPID, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("radioNID", radioNID, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("radioPowerLevel", radioPowerLevel, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("wifi[0].ssid", wifi[0].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE]);
    ADD_MAP_4("wifi[0].psk",  wifi[0].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE]);
    ADD_MAP_4("wifi[1].ssid", wifi[1].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE]);
    ADD_MAP_4("wifi[1].psk",  wifi[1].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE]);
    ADD_MAP_4("wifi[2].ssid", wifi[2].ssid, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE]);
    ADD_MAP_4("wifi[2].psk",  wifi[2].psk,  DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE]);
    ADD_MAP_4("server[0].ipAddr[0]", server[0].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[0].ipAddr[1]", server[0].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[0].ipAddr[2]", server[0].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[0].ipAddr[3]", server[0].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[0].port", server[0].port, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("server[1].ipAddr[0]", server[1].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[1].ipAddr[1]", server[1].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[1].ipAddr[2]", server[1].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[1].ipAddr[3]", server[1].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[1].port", server[1].port, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("server[2].ipAddr[0]", server[2].ipAddr.u8[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[2].ipAddr[1]", server[2].ipAddr.u8[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[2].ipAddr[2]", server[2].ipAddr.u8[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[2].ipAddr[3]", server[2].ipAddr.u8[3], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("server[2].port", server[2].port, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("encoderTickToWheelRad", encoderTickToWheelRad, DATA_TYPE_F32, float);
    ADD_MAP_4("CANbaud_kbps", CANbaud_kbps, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("can_receive_address", can_receive_address, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("uinsComPort", uinsComPort, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("uinsAuxPort", uinsAuxPort, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("reserved2[0]", reserved2[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved2[1]", reserved2[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("portOptions", portOptions, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("h3sp330BaudRate", h3sp330BaudRate, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("h4xRadioBaudRate", h4xRadioBaudRate, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("h8gpioBaudRate", h8gpioBaudRate, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("wheelCfgBits", wheelCfgBits, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("velocityControlPeriodMs", velocityControlPeriodMs, DATA_TYPE_UINT32, uint32_t);

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
    ADD_MAP_4("f[0]", f[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("f[1]", f[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("f[2]", f[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("f[3]", f[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("f[4]", f[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("f[5]", f[5], DATA_TYPE_F32, float&);
    ADD_MAP_4("f[6]", f[6], DATA_TYPE_F32, float&);
    ADD_MAP_4("f[7]", f[7], DATA_TYPE_F32, float&);
    ADD_MAP_4("f[8]", f[8], DATA_TYPE_F32, float&);
    ADD_MAP_4("lf[0]", lf[0], DATA_TYPE_F64, double&);
    ADD_MAP_4("lf[1]", lf[1], DATA_TYPE_F64, double&);
    ADD_MAP_4("lf[2]", lf[2], DATA_TYPE_F64, double&);

    ASSERT_SIZE(totalSize);
}

#if defined(INCLUDE_LUNA_DATA_SETS)

static void PopulateEvbLunaFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_flash_cfg_t, DID_EVB_LUNA_FLASH_CFG);
    
    ADD_MAP_4("size", size, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("checksum", checksum, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("key", key, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("bits", bits, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("minLatGeofence", minLatGeofence, DATA_TYPE_F64, double);
    ADD_MAP_4("maxLatGeofence", maxLatGeofence, DATA_TYPE_F64, double);
    ADD_MAP_4("minLonGeofence", minLonGeofence, DATA_TYPE_F64, double);
    ADD_MAP_4("maxLonGeofence", maxLonGeofence, DATA_TYPE_F64, double);
    ADD_MAP_4("remoteKillTimeoutMs", remoteKillTimeoutMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("bumpSensitivity", bumpSensitivity, DATA_TYPE_F32, float);
    ADD_MAP_4("minProxDistance", minProxDistance, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.config",                  velControl.config, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("velControl.cmdTimeoutMs",            velControl.cmdTimeoutMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("velControl.wheelRadius",             velControl.wheelRadius, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheelBaseline",           velControl.wheelBaseline, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.engine_rpm",              velControl.engine_rpm, DATA_TYPE_F32, float);

    ADD_MAP_4("velControl.vehicle.u_min",           velControl.vehicle.u_min, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.u_cruise",        velControl.vehicle.u_cruise, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.u_max",           velControl.vehicle.u_max, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.u_slewLimit",     velControl.vehicle.u_slewLimit, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.w_max_autonomous",velControl.vehicle.w_max_autonomous, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.w_max",           velControl.vehicle.w_max, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.w_slewLimit",     velControl.vehicle.w_slewLimit, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.u_FB_Kp",         velControl.vehicle.u_FB_Kp, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.w_FB_Kp",         velControl.vehicle.w_FB_Kp, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.w_FB_Ki",         velControl.vehicle.w_FB_Ki, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.w_FF_c0",         velControl.vehicle.w_FF_c0, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.w_FF_c1",         velControl.vehicle.w_FF_c1, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.w_FF_deadband",   velControl.vehicle.w_FF_deadband, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.vehicle.testSweepRate",   velControl.vehicle.testSweepRate, DATA_TYPE_F32, float);

    ADD_MAP_4("velControl.wheel.slewRate",          velControl.wheel.slewRate, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.velMax",            velControl.wheel.velMax, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.FF_vel_deadband",   velControl.wheel.FF_vel_deadband, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.FF_c_est_Ki[0]",    velControl.wheel.FF_c_est_Ki[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.FF_c_est_Ki[1]",    velControl.wheel.FF_c_est_Ki[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.FF_c_est_max[0]",   velControl.wheel.FF_c_est_max[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.FF_c_est_max[1]",   velControl.wheel.FF_c_est_max[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.FF_c_l[0]",         velControl.wheel.FF_c_l[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.FF_c_l[1]",         velControl.wheel.FF_c_l[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.FF_c_r[0]",         velControl.wheel.FF_c_r[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.FF_c_r[1]",         velControl.wheel.FF_c_r[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.FF_FB_engine_rpm",  velControl.wheel.FF_FB_engine_rpm, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.FB_Kp",             velControl.wheel.FB_Kp, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.FB_Ki",             velControl.wheel.FB_Ki, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.FB_Kd",             velControl.wheel.FB_Kd, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.FB_gain_deadband",  velControl.wheel.FB_gain_deadband, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.FB_gain_deadband_reduction",  velControl.wheel.FB_gain_deadband_reduction, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.InversePlant_l[0]", velControl.wheel.InversePlant_l[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_l[1]", velControl.wheel.InversePlant_l[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_l[2]", velControl.wheel.InversePlant_l[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_l[3]", velControl.wheel.InversePlant_l[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_l[4]", velControl.wheel.InversePlant_l[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_r[0]", velControl.wheel.InversePlant_r[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_r[1]", velControl.wheel.InversePlant_r[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_r[2]", velControl.wheel.InversePlant_r[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_r[3]", velControl.wheel.InversePlant_r[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.InversePlant_r[4]", velControl.wheel.InversePlant_r[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.actuatorTrim_l",    velControl.wheel.actuatorTrim_l, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.actuatorTrim_r",    velControl.wheel.actuatorTrim_r, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.actuatorLimits_l[0]", velControl.wheel.actuatorLimits_l[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.actuatorLimits_l[1]", velControl.wheel.actuatorLimits_l[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.actuatorLimits_r[0]", velControl.wheel.actuatorLimits_r[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.actuatorLimits_r[1]", velControl.wheel.actuatorLimits_r[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("velControl.wheel.actuatorDeadbandDuty_l", velControl.wheel.actuatorDeadbandDuty_l, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.actuatorDeadbandDuty_r", velControl.wheel.actuatorDeadbandDuty_r, DATA_TYPE_F32, float);
    ADD_MAP_4("velControl.wheel.actuatorDeadbandVel", velControl.wheel.actuatorDeadbandVel, DATA_TYPE_F32, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateCoyoteStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_status_t, DID_EVB_LUNA_STATUS);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("evbLunaStatus", evbLunaStatus, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("motorState", motorState, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("remoteKillMode", remoteKillMode, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("supplyVoltage", supplyVoltage, DATA_TYPE_F32, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaSensorsMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_sensors_t, DID_EVB_LUNA_SENSORS);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("proxSensorOutput[0]", proxSensorOutput[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("proxSensorOutput[1]", proxSensorOutput[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("proxSensorOutput[2]", proxSensorOutput[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("proxSensorOutput[3]", proxSensorOutput[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("proxSensorOutput[4]", proxSensorOutput[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("proxSensorOutput[5]", proxSensorOutput[5], DATA_TYPE_F32, float&);
    ADD_MAP_4("proxSensorOutput[6]", proxSensorOutput[6], DATA_TYPE_F32, float&);
    ADD_MAP_4("proxSensorOutput[7]", proxSensorOutput[7], DATA_TYPE_F32, float&);
    ADD_MAP_4("proxSensorOutput[8]", proxSensorOutput[8], DATA_TYPE_F32, float&);
    ADD_MAP_4("bumpEvent", bumpEvent, DATA_TYPE_INT32, int32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaVelocityControlMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_velocity_control_t, DID_EVB_LUNA_VELOCITY_CONTROL);
    
    ADD_MAP_4("timeMs", timeMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("dt", dt, DATA_TYPE_F32, float);
    ADD_MAP_4("current_mode", current_mode, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("vehicle.velCmd_f", vehicle.velCmd_f, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.velCmd_w", vehicle.velCmd_w, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.velCmdMnl_f", vehicle.velCmdMnl_f, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.velCmdMnl_w", vehicle.velCmdMnl_w, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.velCmdSlew_f", vehicle.velCmdSlew_f, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.velCmdSlew_w", vehicle.velCmdSlew_w, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.vel_f", vehicle.vel_f, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.vel_w", vehicle.vel_w, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.err_f", vehicle.err_f, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.err_w", vehicle.err_w, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.eff_f", vehicle.eff_f, DATA_TYPE_F32, float);
    ADD_MAP_4("vehicle.eff_w", vehicle.eff_w, DATA_TYPE_F32, float);

    ADD_MAP_4("wheel_l.velCmd",             wheel_l.velCmd, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.velCmdSlew",         wheel_l.velCmdSlew, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.vel",                wheel_l.vel, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.err",                wheel_l.err, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.ff_eff",             wheel_l.ff_eff, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.fb_eff",             wheel_l.fb_eff, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.fb_eff_integral",    wheel_l.fb_eff_integral, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.eff",                wheel_l.eff, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.effInt",             wheel_l.effInt, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_l.effDuty",            wheel_l.effDuty, DATA_TYPE_F32, float);

    ADD_MAP_4("wheel_r.velCmd",             wheel_r.velCmd, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.velCmdSlew",         wheel_r.velCmdSlew, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.vel",                wheel_r.vel, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.err",                wheel_r.err, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.ff_eff",             wheel_r.ff_eff, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.fb_eff",             wheel_r.fb_eff, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.fb_eff_integral",    wheel_r.fb_eff_integral, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.eff",                wheel_r.eff, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.effInt",             wheel_r.effInt, DATA_TYPE_F32, float);
    ADD_MAP_4("wheel_r.effDuty",            wheel_r.effDuty, DATA_TYPE_F32, float);

    ADD_MAP_4("potV_l", potV_l, DATA_TYPE_F32, float);
    ADD_MAP_4("potV_r", potV_r, DATA_TYPE_F32, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaVelocityCommandMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_velocity_command_t, DID_EVB_LUNA_VELOCITY_COMMAND);
    
    ADD_MAP_4("timeMs", timeMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("modeCmd", modeCmd, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("fwd_vel", fwd_vel, DATA_TYPE_F32, float);
    ADD_MAP_4("turn_rate", turn_rate, DATA_TYPE_F32, float);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaAuxCmdMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_aux_command_t, DID_EVB_LUNA_AUX_COMMAND);
    
    ADD_MAP_4("command", command, DATA_TYPE_UINT32, uint32_t);

    ASSERT_SIZE(totalSize);
}

#endif

static void PopulateGpsRtkRelMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_rtk_rel_t, id);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("differentialAge", differentialAge, DATA_TYPE_F32, float);
    ADD_MAP_4("arRatio", arRatio, DATA_TYPE_F32, float);
    ADD_MAP_4("baseToRoverVector[0]", baseToRoverVector[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("baseToRoverVector[1]", baseToRoverVector[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("baseToRoverVector[2]", baseToRoverVector[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("baseToRoverDistance", baseToRoverDistance, DATA_TYPE_F32, float);
    ADD_MAP_4("baseToRoverHeading", baseToRoverHeading, DATA_TYPE_F32, float);
    ADD_MAP_4("baseToRoverHeadingAcc", baseToRoverHeadingAcc, DATA_TYPE_F32, float);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX | DATA_FLAGS_GPS_STATUS);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRtkMiscMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_rtk_misc_t, id);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("accuracyPos[0]", accuracyPos[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("accuracyPos[1]", accuracyPos[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("accuracyPos[2]", accuracyPos[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("accuracyCov[0]", accuracyCov[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("accuracyCov[1]", accuracyCov[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("accuracyCov[2]", accuracyCov[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("arThreshold", arThreshold, DATA_TYPE_F32, float);
    ADD_MAP_4("gDop", gDop, DATA_TYPE_F32, float);
    ADD_MAP_4("hDop", hDop, DATA_TYPE_F32, float);
    ADD_MAP_4("vDop", vDop, DATA_TYPE_F32, float);
    ADD_MAP_4("baseLla[0]", baseLla[0], DATA_TYPE_F64, double&);
    ADD_MAP_4("baseLla[1]", baseLla[1], DATA_TYPE_F64, double&);
    ADD_MAP_4("baseLla[2]", baseLla[2], DATA_TYPE_F64, double&);
    ADD_MAP_4("cycleSlipCount", cycleSlipCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverGpsObservationCount", roverGpsObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseGpsObservationCount", baseGpsObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverGlonassObservationCount", roverGlonassObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseGlonassObservationCount", baseGlonassObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverGalileoObservationCount", roverGalileoObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseGalileoObservationCount", baseGalileoObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverBeidouObservationCount", roverBeidouObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseBeidouObservationCount", baseBeidouObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverQzsObservationCount", roverQzsObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseQzsObservationCount", baseQzsObservationCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverGpsEphemerisCount", roverGpsEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseGpsEphemerisCount", baseGpsEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverGlonassEphemerisCount", roverGlonassEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseGlonassEphemerisCount", baseGlonassEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverGalileoEphemerisCount", roverGalileoEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseGalileoEphemerisCount", baseGalileoEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverBeidouEphemerisCount", roverBeidouEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseBeidouEphemerisCount", baseBeidouEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverQzsEphemerisCount", roverQzsEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseQzsEphemerisCount", baseQzsEphemerisCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("roverSbasCount", roverSbasCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseSbasCount", baseSbasCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("baseAntennaCount", baseAntennaCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("ionUtcAlmCount", ionUtcAlmCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("correctionChecksumFailures", correctionChecksumFailures, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("timeToFirstFixMs", timeToFirstFixMs, DATA_TYPE_UINT32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRawMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_raw_t, id);
    

    ADD_MAP_4("receiveIndex", receiverIndex, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("dataType", dataType, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obsCount", obsCount, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("reserved", reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("dataBuf", data.buf, DATA_TYPE_BINARY, uint8_t[MEMBERSIZE(MAP_TYPE, data.buf)]);

    ASSERT_SIZE(totalSize);
}

static void PopulateStrobeInTimeMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(strobe_in_time_t, DID_STROBE_IN_TIME);
    

    ADD_MAP_4("week", week, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pin", pin, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("count", count, DATA_TYPE_UINT16, uint16_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateRtosInfoMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rtos_info_t, DID_RTOS_INFO);
    

    ADD_MAP_4("freeHeapSize", freeHeapSize, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("mallocSize", mallocSize, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("freeSize", freeSize, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("T0_name", task[0].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN]);
    ADD_MAP_4("T0_priority", task[0].priority, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T0_stackUnused", task[0].stackUnused, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T0_periodMs", task[0].periodMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T0_runtimeUs", task[0].runtimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T0_avgRuntimeUs", task[0].avgRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T0_avgLowerRuntimeUs", task[0].lowerRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T0_avgUpperRuntimeUs", task[0].upperRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T0_maxRuntimeUs", task[0].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T0_startTimeUs", task[0].startTimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T0_gapCount", task[0].gapCount, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("T0_doubleGapCount", task[0].doubleGapCount, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T0_reserved", task[0].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T0_cpuUsage", task[0].cpuUsage, DATA_TYPE_F32, f_t);
    ADD_MAP_4("T0_handle", task[0].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("T1_name", task[1].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN]);
    ADD_MAP_4("T1_priority", task[1].priority, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T1_stackUnused", task[1].stackUnused, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T1_periodMs", task[1].periodMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T1_runtimeUs", task[1].runtimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T1_avgRuntimeUs", task[1].avgRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T1_avgLowerRuntimeUs", task[1].lowerRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T1_avgUpperRuntimeUs", task[1].upperRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T1_maxRuntimeUs", task[1].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T1_startTimeUs", task[1].startTimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T1_gapCount", task[1].gapCount, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("T1_doubleGapCount", task[1].doubleGapCount, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T1_reserved", task[1].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T1_cpuUsage", task[1].cpuUsage, DATA_TYPE_F32, f_t);
    ADD_MAP_4("T1_handle", task[1].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("T2_name", task[2].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN]);
    ADD_MAP_4("T2_priority", task[2].priority, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T2_stackUnused", task[2].stackUnused, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T2_periodMs", task[2].periodMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T2_runtimeUs", task[2].runtimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T2_avgRuntimeUs", task[2].avgRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T2_avgLowerRuntimeUs", task[2].lowerRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T2_avgUpperRuntimeUs", task[2].upperRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T2_maxRuntimeUs", task[2].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T2_startTimeUs", task[2].startTimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T2_gapCount", task[2].gapCount, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("T2_doubleGapCount", task[2].doubleGapCount, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T2_reserved", task[2].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T2_cpuUsage", task[2].cpuUsage, DATA_TYPE_F32, f_t);
    ADD_MAP_4("T2_handle", task[2].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("T3_name", task[3].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN]);
    ADD_MAP_4("T3_priority", task[3].priority, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T3_stackUnused", task[3].stackUnused, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T3_periodMs", task[3].periodMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T3_runtimeUs", task[3].runtimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T3_avgRuntimeUs", task[3].avgRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T3_avgLowerRuntimeUs", task[3].lowerRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T3_avgUpperRuntimeUs", task[3].upperRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T3_maxRuntimeUs", task[3].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T3_startTimeUs", task[3].startTimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T3_gapCount", task[3].gapCount, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("T3_doubleGapCount", task[3].doubleGapCount, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T3_reserved", task[3].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T3_cpuUsage", task[3].cpuUsage, DATA_TYPE_F32, f_t);
    ADD_MAP_4("T3_handle", task[3].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("T4_name", task[4].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN]);
    ADD_MAP_4("T4_priority", task[4].priority, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T4_stackUnused", task[4].stackUnused, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T4_periodMs", task[4].periodMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T4_runtimeUs", task[4].runtimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T4_avgRuntimeUs", task[4].avgRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T4_avgLowerRuntimeUs", task[4].lowerRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T4_avgUpperRuntimeUs", task[4].upperRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T4_maxRuntimeUs", task[4].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T4_startTimeUs", task[4].startTimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T4_gapCount", task[4].gapCount, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("T4_doubleGapCount", task[4].doubleGapCount, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T4_reserved", task[4].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T4_cpuUsage", task[4].cpuUsage, DATA_TYPE_F32, f_t);
    ADD_MAP_4("T4_handle", task[4].handle, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("T5_name", task[5].name, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN]);
    ADD_MAP_4("T5_priority", task[5].priority, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T5_stackUnused", task[5].stackUnused, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T5_periodMs", task[5].periodMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T5_runtimeUs", task[5].runtimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T5_avgRuntimeUs", task[5].avgRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T5_avgLowerRuntimeUs", task[5].lowerRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T5_avgUpperRuntimeUs", task[5].upperRuntimeUs, DATA_TYPE_F32, float);
    ADD_MAP_4("T5_maxRuntimeUs", task[5].maxRuntimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T5_startTimeUs", task[5].startTimeUs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("T5_gapCount", task[5].gapCount, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_4("T5_doubleGapCount", task[5].doubleGapCount, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T5_reserved", task[5].reserved, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("T5_cpuUsage", task[5].cpuUsage, DATA_TYPE_F32, f_t);
    ADD_MAP_4("T5_handle", task[5].handle, DATA_TYPE_UINT32, uint32_t);

    ASSERT_SIZE(totalSize);
}
static void PopulateCanConfigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(can_config_t, DID_CAN_CONFIG);
    
    ADD_MAP_4("can_period_mult[CID_INS_TIME]", can_period_mult[CID_INS_TIME], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_STATUS]", can_period_mult[CID_INS_STATUS], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_EULER]", can_period_mult[CID_INS_EULER], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_QUATN2B]", can_period_mult[CID_INS_QUATN2B], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_QUATE2B]", can_period_mult[CID_INS_QUATE2B], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_UVW]", can_period_mult[CID_INS_UVW], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_VE]", can_period_mult[CID_INS_VE], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_LAT]", can_period_mult[CID_INS_LAT], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_LON]", can_period_mult[CID_INS_LON], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_ALT]", can_period_mult[CID_INS_ALT], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_NORTH_EAST]", can_period_mult[CID_INS_NORTH_EAST], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_DOWN]", can_period_mult[CID_INS_DOWN], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_ECEF_X]", can_period_mult[CID_INS_ECEF_X], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_ECEF_Y]", can_period_mult[CID_INS_ECEF_Y], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_ECEF_Z]", can_period_mult[CID_INS_ECEF_Z], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_INS_MSL]", can_period_mult[CID_INS_MSL], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_PREINT_PX]", can_period_mult[CID_PREINT_PX], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_PREINT_QY]", can_period_mult[CID_PREINT_QY], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_PREINT_RZ]", can_period_mult[CID_PREINT_RZ], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_DUAL_PX]", can_period_mult[CID_DUAL_PX], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_DUAL_QY]", can_period_mult[CID_DUAL_QY], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_DUAL_RZ]", can_period_mult[CID_DUAL_RZ], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_GPS1_POS]", can_period_mult[CID_GPS1_POS], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_GPS2_POS]", can_period_mult[CID_GPS2_POS], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_GPS1_RTK_POS_REL]", can_period_mult[CID_GPS1_RTK_POS_REL], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_GPS2_RTK_CMP_REL]", can_period_mult[CID_GPS2_RTK_CMP_REL], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_4("can_period_mult[CID_ROLL_ROLLRATE]", can_period_mult[CID_ROLL_ROLLRATE], DATA_TYPE_UINT16, uint16_t&);
    ADD_MAP_5("can_transmit_address[CID_INS_TIME]", can_transmit_address[CID_INS_TIME], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_STATUS]", can_transmit_address[CID_INS_STATUS], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_EULER]", can_transmit_address[CID_INS_EULER], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_QUATN2B]", can_transmit_address[CID_INS_QUATN2B], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_QUATE2B]", can_transmit_address[CID_INS_QUATE2B], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_UVW]", can_transmit_address[CID_INS_UVW], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_VE]", can_transmit_address[CID_INS_VE], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_LAT]", can_transmit_address[CID_INS_LAT], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_LON]", can_transmit_address[CID_INS_LON], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_ALT]", can_transmit_address[CID_INS_ALT], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_NORTH_EAST]", can_transmit_address[CID_INS_NORTH_EAST], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_DOWN]", can_transmit_address[CID_INS_DOWN], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_ECEF_X]", can_transmit_address[CID_INS_ECEF_X], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_ECEF_Y]", can_transmit_address[CID_INS_ECEF_Y], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_ECEF_Z]", can_transmit_address[CID_INS_ECEF_Z], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_INS_MSL]", can_transmit_address[CID_INS_MSL], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_PREINT_PX]", can_transmit_address[CID_PREINT_PX], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_PREINT_QY]", can_transmit_address[CID_PREINT_QY], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_PREINT_RZ]", can_transmit_address[CID_PREINT_RZ], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_DUAL_PX]", can_transmit_address[CID_DUAL_PX], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_DUAL_QY]", can_transmit_address[CID_DUAL_QY], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_DUAL_RZ]", can_transmit_address[CID_DUAL_RZ], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_GPS1_POS]", can_transmit_address[CID_GPS1_POS], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_GPS2_POS]", can_transmit_address[CID_GPS2_POS], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_GPS1_RTK_POS_REL]", can_transmit_address[CID_GPS1_RTK_POS_REL], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_GPS2_RTK_CMP_REL]", can_transmit_address[CID_GPS2_RTK_CMP_REL], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_5("can_transmit_address[CID_ROLL_ROLLRATE]", can_transmit_address[CID_ROLL_ROLLRATE], DATA_TYPE_UINT32, uint32_t&, DATA_FLAGS_DISPLAY_HEX);

    ADD_MAP_4("can_baudrate_kbps", can_baudrate_kbps, DATA_TYPE_UINT16, uint16_t);
    ADD_MAP_5("can_receive_address", can_receive_address, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);

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

#ifdef USE_IS_INTERNAL

static void PopulateSensorsADCMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sys_sensors_adc_t, DID_SENSORS_ADC);
    
    ADD_MAP_4("time", time, DATA_TYPE_F64, double);
    ADD_MAP_4("pqr1[0]", imu[0].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1[1]", imu[0].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1[2]", imu[0].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[0]", imu[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[1]", imu[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[2]", imu[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("temp1",   imu[0].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr2[0]", imu[1].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2[1]", imu[1].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2[2]", imu[1].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[0]", imu[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[1]", imu[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[2]", imu[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("temp2", imu[1].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr3[0]", imu[2].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr3[1]", imu[2].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr3[2]", imu[2].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc3[0]", imu[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc3[1]", imu[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc3[2]", imu[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("temp3", imu[2].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("mag1[0]", mag[0].mag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1[1]", mag[0].mag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1[2]", mag[0].mag[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag2[0]", mag[1].mag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag2[1]", mag[1].mag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag2[2]", mag[1].mag[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("bar", bar, DATA_TYPE_F32, float);
    ADD_MAP_4("barTemp", barTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("humidity", humidity, DATA_TYPE_F32, float);
    ADD_MAP_4("ana[0]", ana[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("ana[1]", ana[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("ana[2]", ana[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("ana[3]", ana[3], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsISMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(sensors_w_temp_t, id);
    
    ADD_MAP_4("imu3.time", imu3.time, DATA_TYPE_F64, double);
    ADD_MAP_4("imu3.status", imu3.status, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pqr0[0]", imu3.I[0].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0[1]", imu3.I[0].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0[2]", imu3.I[0].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0[0]", imu3.I[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0[1]", imu3.I[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0[2]", imu3.I[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1[0]", imu3.I[1].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1[1]", imu3.I[1].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1[2]", imu3.I[1].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[0]", imu3.I[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[1]", imu3.I[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[2]", imu3.I[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2[0]", imu3.I[2].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2[1]", imu3.I[2].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2[2]", imu3.I[2].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[0]", imu3.I[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[1]", imu3.I[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[2]", imu3.I[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("temp0", temp[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("temp1", temp[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("temp2", temp[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0[0]", mag[0].xyz[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0[1]", mag[0].xyz[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0[2]", mag[0].xyz[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1[0]", mag[1].xyz[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1[1]", mag[1].xyz[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1[2]", mag[1].xyz[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsTCMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sensors_t, DID_SENSORS_TC_BIAS);
    
    ADD_MAP_4("time", time, DATA_TYPE_F64, double);
    ADD_MAP_4("pqr0[0]", mpu[0].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0[1]", mpu[0].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0[2]", mpu[0].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0[0]", mpu[0].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0[1]", mpu[0].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0[2]", mpu[0].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0[0]", mpu[0].mag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0[1]", mpu[0].mag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0[2]", mpu[0].mag[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1[0]", mpu[1].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1[1]", mpu[1].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1[2]", mpu[1].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[0]", mpu[1].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[1]", mpu[1].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1[2]", mpu[1].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1[0]", mpu[1].mag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1[1]", mpu[1].mag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1[2]", mpu[1].mag[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2[0]", mpu[2].pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2[1]", mpu[2].pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2[2]", mpu[2].pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[0]", mpu[2].acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[1]", mpu[2].acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2[2]", mpu[2].acc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag2[0]", mpu[2].mag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag2[1]", mpu[2].mag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag2[2]", mpu[2].mag[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsCompMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sensor_compensation_t, DID_SCOMP);
    
    ADD_MAP_4("timeMs", timeMs, DATA_TYPE_UINT32, uint32_t);

    // Gyros
    ADD_MAP_4("pqr0.lpfLsb[0]", pqr[0].lpfLsb[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0.lpfLsb[1]", pqr[0].lpfLsb[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0.lpfLsb[2]", pqr[0].lpfLsb[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0.lpfTemp", pqr[0].lpfTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr0.k[0]", pqr[0].k[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0.k[1]", pqr[0].k[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0.k[2]", pqr[0].k[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr0.temp", pqr[0].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr0.tempRampRate", pqr[0].tempRampRate, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr0.tci", pqr[0].tci, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pqr0.numTcPts", pqr[0].numTcPts, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pqr0.dtTemp", pqr[0].dtTemp, DATA_TYPE_F32, float);

    ADD_MAP_4("pqr1.lpfLsb[0]", pqr[1].lpfLsb[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1.lpfLsb[1]", pqr[1].lpfLsb[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1.lpfLsb[2]", pqr[1].lpfLsb[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1.lpfTemp", pqr[1].lpfTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr1.k[0]", pqr[1].k[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1.k[1]", pqr[1].k[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1.k[2]", pqr[1].k[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr1.temp", pqr[1].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr1.tempRampRate", pqr[1].tempRampRate, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr1.tci", pqr[1].tci, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pqr1.numTcPts", pqr[1].numTcPts, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pqr1.dtTemp", pqr[1].dtTemp, DATA_TYPE_F32, float);

    ADD_MAP_4("pqr2.lpfLsb[0]", pqr[2].lpfLsb[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2.lpfLsb[1]", pqr[2].lpfLsb[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2.lpfLsb[2]", pqr[2].lpfLsb[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2.lpfTemp", pqr[2].lpfTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr2.k[0]", pqr[2].k[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2.k[1]", pqr[2].k[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2.k[2]", pqr[2].k[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("pqr2.temp", pqr[2].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr2.tempRampRate", pqr[2].tempRampRate, DATA_TYPE_F32, float);
    ADD_MAP_4("pqr2.tci", pqr[2].tci, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pqr2.numTcPts", pqr[2].numTcPts, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("pqr2.dtTemp", pqr[2].dtTemp, DATA_TYPE_F32, float);

    // Accels
    ADD_MAP_4("acc0.lpfLsb[0]", acc[0].lpfLsb[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0.lpfLsb[1]", acc[0].lpfLsb[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0.lpfLsb[2]", acc[0].lpfLsb[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0.lpfTemp", acc[0].lpfTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("acc0.k[0]", acc[0].k[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0.k[1]", acc[0].k[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0.k[2]", acc[0].k[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc0.temp", acc[0].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("acc0.tempRampRate", acc[0].tempRampRate, DATA_TYPE_F32, float);
    ADD_MAP_4("acc0.tci", acc[0].tci, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("acc0.numTcPts", acc[0].numTcPts, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("acc0.dtTemp", acc[0].dtTemp, DATA_TYPE_F32, float);

    ADD_MAP_4("acc1.lpfLsb[0]", acc[1].lpfLsb[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1.lpfLsb[1]", acc[1].lpfLsb[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1.lpfLsb[2]", acc[1].lpfLsb[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1.lpfTemp", acc[1].lpfTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("acc1.k[0]", acc[1].k[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1.k[1]", acc[1].k[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1.k[2]", acc[1].k[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc1.temp", acc[1].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("acc1.tempRampRate", acc[1].tempRampRate, DATA_TYPE_F32, float);
    ADD_MAP_4("acc1.tci", acc[1].tci, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("acc1.numTcPts", acc[1].numTcPts, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("acc1.dtTemp", acc[1].dtTemp, DATA_TYPE_F32, float);

    ADD_MAP_4("acc2.lpfLsb[0]", acc[2].lpfLsb[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2.lpfLsb[1]", acc[2].lpfLsb[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2.lpfLsb[2]", acc[2].lpfLsb[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2.lpfTemp", acc[2].lpfTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("acc2.k[0]", acc[2].k[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2.k[1]", acc[2].k[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2.k[2]", acc[2].k[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("acc2.temp", acc[2].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("acc2.tempRampRate", acc[2].tempRampRate, DATA_TYPE_F32, float);
    ADD_MAP_4("acc2.tci", acc[2].tci, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("acc2.numTcPts", acc[2].numTcPts, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("acc2.dtTemp", acc[2].dtTemp, DATA_TYPE_F32, float);

    // Magnetometers
    ADD_MAP_4("mag0.lpfLsb[0]", mag[0].lpfLsb[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0.lpfLsb[1]", mag[0].lpfLsb[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0.lpfLsb[2]", mag[0].lpfLsb[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0.lpfTemp", mag[0].lpfTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("mag0.k[0]", mag[0].k[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0.k[1]", mag[0].k[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0.k[2]", mag[0].k[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag0.temp", mag[0].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("mag0.tempRampRate", mag[0].tempRampRate, DATA_TYPE_F32, float);
    ADD_MAP_4("mag0.tci", mag[0].tci, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("mag0.numTcPts", mag[0].numTcPts, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("mag0.dtTemp", mag[0].dtTemp, DATA_TYPE_F32, float);

    ADD_MAP_4("mag1.lpfLsb[0]", mag[1].lpfLsb[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1.lpfLsb[1]", mag[1].lpfLsb[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1.lpfLsb[2]", mag[1].lpfLsb[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1.lpfTemp", mag[1].lpfTemp, DATA_TYPE_F32, float);
    ADD_MAP_4("mag1.k[0]", mag[1].k[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1.k[1]", mag[1].k[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1.k[2]", mag[1].k[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag1.temp", mag[1].temp, DATA_TYPE_F32, float);
    ADD_MAP_4("mag1.tempRampRate", mag[1].tempRampRate, DATA_TYPE_F32, float);
    ADD_MAP_4("mag1.tci", mag[1].tci, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("mag1.numTcPts", mag[1].numTcPts, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("mag1.dtTemp", mag[1].dtTemp, DATA_TYPE_F32, float);

    // Reference IMU
    ADD_MAP_4("referenceImu.pqr[0]", referenceImu.pqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("referenceImu.pqr[1]", referenceImu.pqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("referenceImu.pqr[2]", referenceImu.pqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("referenceImu.acc[0]", referenceImu.acc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("referenceImu.acc[1]", referenceImu.acc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("referenceImu.acc[2]", referenceImu.acc[2], DATA_TYPE_F32, float&);
    // Reference Mag
    ADD_MAP_4("referenceMag[0]", referenceMag[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("referenceMag[1]", referenceMag[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("referenceMag[2]", referenceMag[2], DATA_TYPE_F32, float&);

    ADD_MAP_4("sampleCount", sampleCount, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("calState", calState, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_5("status", status, DATA_TYPE_UINT32, uint32_t, DATA_FLAGS_DISPLAY_HEX);
    ADD_MAP_4("alignAccel[0]", alignAccel[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("alignAccel[1]", alignAccel[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("alignAccel[2]", alignAccel[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}

static void PopulateUserPage0Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(nvm_group_0_t, DID_NVR_USERPAGE_G0);
    
    ADD_MAP_4("size", size, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("checksum", checksum, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("key", key, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("lockBits", lockBits, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("featureBits", featureBits, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("featureHash1", featureHash1, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("featureHash2", featureHash2, DATA_TYPE_UINT32, uint32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateUserPage1Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(nvm_group_1_t, DID_NVR_USERPAGE_G1);
    
    ADD_MAP_4("size", size, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("checksum", checksum, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("key", key, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("bKpqr", cf.bKpqr, DATA_TYPE_F32, float);
    ADD_MAP_4("bKuvw", cf.bKuvw, DATA_TYPE_F32, float);
    ADD_MAP_4("oKat1", cf.oKat1, DATA_TYPE_F32, float);
    ADD_MAP_4("oKat2", cf.oKat2, DATA_TYPE_F32, float);
    ADD_MAP_4("oKuvw", cf.oKuvw, DATA_TYPE_F32, float);
    ADD_MAP_4("oKlla", cf.oKlla, DATA_TYPE_F32, float);
    ADD_MAP_4("mag.bias_cal[0]", mag.bias_cal[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.bias_cal[1]", mag.bias_cal[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.bias_cal[2]", mag.bias_cal[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[0]", mag.Wcal[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[1]", mag.Wcal[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[2]", mag.Wcal[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[3]", mag.Wcal[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[4]", mag.Wcal[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[5]", mag.Wcal[5], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[6]", mag.Wcal[6], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[7]", mag.Wcal[7], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.Wcal[8]", mag.Wcal[8], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[0]", mag.DtD[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[1]", mag.DtD[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[2]", mag.DtD[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[3]", mag.DtD[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[4]", mag.DtD[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[5]", mag.DtD[5], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[6]", mag.DtD[6], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[7]", mag.DtD[7], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[8]", mag.DtD[8], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[9]", mag.DtD[9], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[10]", mag.DtD[10], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[11]", mag.DtD[11], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[12]", mag.DtD[12], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[13]", mag.DtD[13], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[14]", mag.DtD[14], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[15]", mag.DtD[15], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[16]", mag.DtD[16], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[17]", mag.DtD[17], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[18]", mag.DtD[18], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[19]", mag.DtD[19], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[20]", mag.DtD[20], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[21]", mag.DtD[21], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[22]", mag.DtD[22], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[23]", mag.DtD[23], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[24]", mag.DtD[24], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[25]", mag.DtD[25], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[26]", mag.DtD[26], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[27]", mag.DtD[27], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[28]", mag.DtD[28], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[29]", mag.DtD[29], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[30]", mag.DtD[30], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[31]", mag.DtD[31], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[32]", mag.DtD[32], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[33]", mag.DtD[33], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[34]", mag.DtD[34], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[35]", mag.DtD[35], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[36]", mag.DtD[36], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[37]", mag.DtD[37], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[38]", mag.DtD[38], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[39]", mag.DtD[39], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[40]", mag.DtD[40], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[41]", mag.DtD[41], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[42]", mag.DtD[42], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[43]", mag.DtD[43], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[44]", mag.DtD[44], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[45]", mag.DtD[45], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[46]", mag.DtD[46], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[47]", mag.DtD[47], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[48]", mag.DtD[48], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[49]", mag.DtD[49], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[50]", mag.DtD[50], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[51]", mag.DtD[51], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[52]", mag.DtD[52], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[53]", mag.DtD[53], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[54]", mag.DtD[54], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[55]", mag.DtD[55], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[56]", mag.DtD[56], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[57]", mag.DtD[57], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[58]", mag.DtD[58], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[59]", mag.DtD[59], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[60]", mag.DtD[60], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[61]", mag.DtD[61], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[62]", mag.DtD[62], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[63]", mag.DtD[63], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[64]", mag.DtD[64], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[65]", mag.DtD[65], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[66]", mag.DtD[66], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[67]", mag.DtD[67], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[68]", mag.DtD[68], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[69]", mag.DtD[69], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[70]", mag.DtD[70], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[71]", mag.DtD[71], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[72]", mag.DtD[72], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[73]", mag.DtD[73], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[74]", mag.DtD[74], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[75]", mag.DtD[75], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[76]", mag.DtD[76], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[77]", mag.DtD[77], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[78]", mag.DtD[78], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[79]", mag.DtD[79], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[80]", mag.DtD[80], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[81]", mag.DtD[81], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[82]", mag.DtD[82], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[83]", mag.DtD[83], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[84]", mag.DtD[84], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[85]", mag.DtD[85], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[86]", mag.DtD[86], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[87]", mag.DtD[87], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[88]", mag.DtD[88], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[89]", mag.DtD[89], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[90]", mag.DtD[90], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[91]", mag.DtD[91], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[92]", mag.DtD[92], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[93]", mag.DtD[93], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[94]", mag.DtD[94], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[95]", mag.DtD[95], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[96]", mag.DtD[96], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[97]", mag.DtD[97], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[98]", mag.DtD[98], DATA_TYPE_F32, float&);
    ADD_MAP_4("mag.DtD[99]", mag.DtD[99], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}
static void PopulateInl2MagObsInfo(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(inl2_mag_obs_info_t, DID_INL2_MAG_OBS_INFO);
    
    ADD_MAP_4("timeOfWeekMs", timeOfWeekMs, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("Ncal_samples", Ncal_samples, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("ready", ready, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("calibrated", calibrated, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("auto_recal", auto_recal, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("outlier", outlier, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("magHdg", magHdg, DATA_TYPE_F32, float);
    ADD_MAP_4("insHdg", insHdg, DATA_TYPE_F32, float);
    ADD_MAP_4("magInsHdgDelta", magInsHdgDelta, DATA_TYPE_F32, float);
    ADD_MAP_4("nis", nis, DATA_TYPE_F32, float);
    ADD_MAP_4("nis_threshold", nis_threshold, DATA_TYPE_F32, float);
    ADD_MAP_4("Wcal[0]", Wcal[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("Wcal[1]", Wcal[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("Wcal[2]", Wcal[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("Wcal[3]", Wcal[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("Wcal[4]", Wcal[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("Wcal[5]", Wcal[5], DATA_TYPE_F32, float&);
    ADD_MAP_4("Wcal[6]", Wcal[6], DATA_TYPE_F32, float&);
    ADD_MAP_4("Wcal[7]", Wcal[7], DATA_TYPE_F32, float&);
    ADD_MAP_4("Wcal[8]", Wcal[8], DATA_TYPE_F32, float&);
    ADD_MAP_4("activeCalSet", activeCalSet, DATA_TYPE_UINT32, uint32_t);
    ADD_MAP_4("magHdgOffset", magHdgOffset, DATA_TYPE_F32, float);
    ADD_MAP_4("Tcal", Tcal, DATA_TYPE_F32, float);
    ADD_MAP_4("bias_cal[0]", bias_cal[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("bias_cal[1]", bias_cal[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("bias_cal[2]", bias_cal[2], DATA_TYPE_F32, float&);

    ASSERT_SIZE(totalSize);
}
static void PopulateInl2StatesMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(inl2_states_t, DID_INL2_STATES);
    
    ADD_MAP_4("timeOfWeek", timeOfWeek, DATA_TYPE_F64, double);
    ADD_MAP_4("qe2b[0]", qe2b[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("qe2b[1]", qe2b[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("qe2b[2]", qe2b[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("qe2b[3]", qe2b[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("ve[0]", ve[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("ve[1]", ve[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("ve[2]", ve[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("ecef[0]", ecef[0], DATA_TYPE_F64, double&);
    ADD_MAP_4("ecef[1]", ecef[1], DATA_TYPE_F64, double&);
    ADD_MAP_4("ecef[2]", ecef[2], DATA_TYPE_F64, double&);
    ADD_MAP_4("biasPqr[0]", biasPqr[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("biasPqr[1]", biasPqr[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("biasPqr[2]", biasPqr[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("biasAcc[0]", biasAcc[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("biasAcc[1]", biasAcc[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("biasAcc[2]", biasAcc[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("biasBaro", biasBaro, DATA_TYPE_F32, float);
    ADD_MAP_4("magDec", magDec, DATA_TYPE_F32, float);
    ADD_MAP_4("magInc", magInc, DATA_TYPE_F32, float);

    ASSERT_SIZE(totalSize);
}

// static void PopulateRtkStateMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
// {
//     INIT_MAP(rtk_state_t MAP_TYPE;
//     map_name_to_info_t& m = mappings[DID_RTK_STATE];
//     uint32_t totalSize = 0;
//     ADD_MAP_4("time.time", time.time, DATA_TYPE_INT64, int64_t);
//     ADD_MAP_4("time.sec", time.sec, DATA_TYPE_F64, double);
//     ADD_MAP_4("rp[0]", rp_ecef[0], DATA_TYPE_F64, double&);
//     ADD_MAP_4("rp[1]", rp_ecef[1], DATA_TYPE_F64, double&);
//     ADD_MAP_4("rp[2]", rp_ecef[2], DATA_TYPE_F64, double&);
//     ADD_MAP_4("rv[0]", rv_ecef[0], DATA_TYPE_F64, double&);
//     ADD_MAP_4("rv[1]", rv_ecef[1], DATA_TYPE_F64, double&);
//     ADD_MAP_4("rv[2]", rv_ecef[2], DATA_TYPE_F64, double&);
//     ADD_MAP_4("ra[0]", ra_ecef[0], DATA_TYPE_F64, double&);
//     ADD_MAP_4("ra[1]", ra_ecef[1], DATA_TYPE_F64, double&);
//     ADD_MAP_4("ra[2]", ra_ecef[2], DATA_TYPE_F64, double&);
// 
//     ADD_MAP_4("bp[0]", bp_ecef[0], DATA_TYPE_F64, double&);
//     ADD_MAP_4("bp[1]", bp_ecef[1], DATA_TYPE_F64, double&);
//     ADD_MAP_4("bp[2]", bp_ecef[2], DATA_TYPE_F64, double&);
//     ADD_MAP_4("bv[0]", bv_ecef[0], DATA_TYPE_F64, double&);
//     ADD_MAP_4("bv[1]", bv_ecef[1], DATA_TYPE_F64, double&);
//     ADD_MAP_4("bv[2]", bv_ecef[2], DATA_TYPE_F64, double&);
// }

static void PopulateRtkResidualMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], int DID)
{
    INIT_MAP(rtk_residual_t, DID);
    
    ADD_MAP_4("time.time", time.time, DATA_TYPE_INT64, int64_t);
    ADD_MAP_4("time.sec", time.sec, DATA_TYPE_F64, double);
    ADD_MAP_4("nv", nv, DATA_TYPE_INT32, int32_t);

    ASSERT_SIZE(totalSize);
}

static void PopulateRtkDebugMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rtk_debug_t, DID_RTK_DEBUG);    

    ADD_MAP_4("time.time", time.time, DATA_TYPE_INT64, int64_t);
    ADD_MAP_4("time.sec", time.sec, DATA_TYPE_F64, double);

    ADD_MAP_4("rej_ovfl", rej_ovfl, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("code_outlier", code_outlier, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("phase_outlier", phase_outlier, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("code_large_residual", code_large_residual, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("phase_large_residual", phase_large_residual, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("invalid_base_position", invalid_base_position, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("bad_baseline_holdamb", bad_baseline_holdamb, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("base_position_error", base_position_error, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("outc_ovfl", outc_ovfl, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("reset_timer", reset_timer, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("use_ubx_position", use_ubx_position, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("large_v2b", large_v2b, DATA_TYPE_UINT8, uint8_t);
    
    ADD_MAP_4("base_position_update", base_position_update, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("rover_position_error", rover_position_error, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("reset_bias", reset_bias, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("start_relpos", start_relpos, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("end_relpos", end_relpos, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("start_rtkpos", start_rtkpos, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("pnt_pos_error", pnt_pos_error, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("no_base_obs_data", no_base_obs_data, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("diff_age_error", diff_age_error, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("moveb_time_sync_error", moveb_time_sync_error, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("waiting_for_rover_packet", waiting_for_rover_packet, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("waiting_for_base_packet", waiting_for_base_packet, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("lsq_error", lsq_error, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("lack_of_valid_sats", lack_of_valid_sats, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("divergent_pnt_pos_iteration", divergent_pnt_pos_iteration, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("chi_square_error", chi_square_error, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("cycle_slips", cycle_slips, DATA_TYPE_UINT32, uint32_t);

    ADD_MAP_4("ubx_error", ubx_error, DATA_TYPE_F32, float);

    ADD_MAP_4("solStatus", solStatus, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("rescode_err_marker", rescode_err_marker, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("error_count", error_count, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("error_code", error_code, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("dist2base", dist2base, DATA_TYPE_F32, float);

    ADD_MAP_4("reserved1", reserved1, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("gdop_error", gdop_error, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("warning_code", warning_code, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("warning_count", warning_count, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("double_debug[0]", double_debug[0], DATA_TYPE_F64, double&);
    ADD_MAP_4("double_debug[1]", double_debug[1], DATA_TYPE_F64, double&);
    ADD_MAP_4("double_debug[2]", double_debug[2], DATA_TYPE_F64, double&);
    ADD_MAP_4("double_debug[3]", double_debug[3], DATA_TYPE_F64, double&);

    ADD_MAP_4("debug[0]", debug[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("debug[1]", debug[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("obs_count_bas", obs_count_bas, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_count_rov", obs_count_rov, DATA_TYPE_UINT8, uint8_t);

    //ADD_MAP_4("obs_pairs_filtered", obs_pairs_filtered, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("reserved2", reserved2, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("raw_ptr_queue_overrun", raw_ptr_queue_overrun, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("raw_dat_queue_overrun", raw_dat_queue_overrun, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_unhealthy", obs_unhealthy, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("obs_rover_avail", obs_rover_avail, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_base_avail", obs_base_avail, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_pairs_used_float", obs_pairs_used_float, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_pairs_used_ar", obs_pairs_used_ar, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("obs_eph_avail", obs_eph_avail, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_low_snr_rover", obs_low_snr_rover, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_low_snr_base", obs_low_snr_base, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_high_snr_parity", obs_high_snr_parity, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("obs_zero_L1_rover", obs_zero_L1_rover, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_zero_L1_base", obs_zero_L1_base, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_low_elev_rover", obs_low_elev_rover, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("obs_low_elev_base", obs_low_elev_base, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("eph1RxCnt", eph1RxCnt, DATA_TYPE_UINT8, uint8_t);
    ADD_MAP_4("eph2RxCnt", eph2RxCnt, DATA_TYPE_UINT8, uint8_t);

    ADD_MAP_4("reserved[0]", reserved[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("reserved[1]", reserved[1], DATA_TYPE_UINT8, uint8_t&);

    ASSERT_SIZE(totalSize);
}

#if 0
static void PopulateRtkDebug2Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rtk_debug_2_t, DID_RTK_DEBUG_2);

    ADD_MAP_4("time.time", time.time, DATA_TYPE_INT64, int64_t);
    ADD_MAP_4("time.sec", time.sec, DATA_TYPE_F64, double);

#if 0    // This doesn't work in Linux

    char str[50];
    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasFloat[%d]", i);
        ADD_MAP_4(str, satBiasFloat[i], DATA_TYPE_F32, float&);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasFix[%d]", i);
        ADD_MAP_4(str, satBiasFix[i], DATA_TYPE_F32, float&);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "qualL[%d]", i);
        ADD_MAP_4(str, qualL[i], DATA_TYPE_UINT8, uint8_t&);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "sat[%d]", i);
        ADD_MAP_4(str, sat[i], DATA_TYPE_UINT8, uint8_t&);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasCov[%d]", i);
        ADD_MAP_4(str, satBiasStd[i], DATA_TYPE_F32, float&);
    }

#else

    ADD_MAP_4("satBiasFloat[0]", satBiasFloat[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[1]", satBiasFloat[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[2]", satBiasFloat[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[3]", satBiasFloat[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[4]", satBiasFloat[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[5]", satBiasFloat[5], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[6]", satBiasFloat[6], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[7]", satBiasFloat[7], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[8]", satBiasFloat[8], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[9]", satBiasFloat[9], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[10]", satBiasFloat[10], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[11]", satBiasFloat[11], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[12]", satBiasFloat[12], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[13]", satBiasFloat[13], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[14]", satBiasFloat[14], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[15]", satBiasFloat[15], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[16]", satBiasFloat[16], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[17]", satBiasFloat[17], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[18]", satBiasFloat[18], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[19]", satBiasFloat[19], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[20]", satBiasFloat[20], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFloat[21]", satBiasFloat[21], DATA_TYPE_F32, float&);

    ADD_MAP_4("satBiasFix[0]", satBiasFix[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[1]", satBiasFix[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[2]", satBiasFix[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[3]", satBiasFix[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[4]", satBiasFix[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[5]", satBiasFix[5], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[6]", satBiasFix[6], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[7]", satBiasFix[7], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[8]", satBiasFix[8], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[9]", satBiasFix[9], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[10]", satBiasFix[10], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[11]", satBiasFix[11], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[12]", satBiasFix[12], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[13]", satBiasFix[13], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[14]", satBiasFix[14], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[15]", satBiasFix[15], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[16]", satBiasFix[16], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[17]", satBiasFix[17], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[18]", satBiasFix[18], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[19]", satBiasFix[19], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[20]", satBiasFix[20], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasFix[21]", satBiasFix[21], DATA_TYPE_F32, float&);

    ADD_MAP_4("qualL[0]", qualL[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[1]", qualL[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[2]", qualL[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[3]", qualL[3], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[4]", qualL[4], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[5]", qualL[5], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[6]", qualL[6], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[7]", qualL[7], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[8]", qualL[8], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[9]", qualL[9], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[10]", qualL[10], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[11]", qualL[11], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[12]", qualL[12], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[13]", qualL[13], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[14]", qualL[14], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[15]", qualL[15], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[16]", qualL[16], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[17]", qualL[17], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[18]", qualL[18], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[19]", qualL[19], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[20]", qualL[20], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("qualL[21]", qualL[21], DATA_TYPE_UINT8, uint8_t&);

    ADD_MAP_4("sat[0]", sat[0], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[1]", sat[1], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[2]", sat[2], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[3]", sat[3], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[4]", sat[4], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[5]", sat[5], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[6]", sat[6], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[7]", sat[7], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[8]", sat[8], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[9]", sat[9], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[10]", sat[10], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[11]", sat[11], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[12]", sat[12], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[13]", sat[13], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[14]", sat[14], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[15]", sat[15], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[16]", sat[16], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[17]", sat[17], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[18]", sat[18], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[19]", sat[19], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[20]", sat[20], DATA_TYPE_UINT8, uint8_t&);
    ADD_MAP_4("sat[21]", sat[21], DATA_TYPE_UINT8, uint8_t&);

    ADD_MAP_4("satBiasStd[0]", satBiasStd[0], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[1]", satBiasStd[1], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[2]", satBiasStd[2], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[3]", satBiasStd[3], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[4]", satBiasStd[4], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[5]", satBiasStd[5], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[6]", satBiasStd[6], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[7]", satBiasStd[7], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[8]", satBiasStd[8], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[9]", satBiasStd[9], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[10]", satBiasStd[10], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[11]", satBiasStd[11], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[12]", satBiasStd[12], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[13]", satBiasStd[13], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[14]", satBiasStd[14], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[15]", satBiasStd[15], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[16]", satBiasStd[16], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[17]", satBiasStd[17], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[18]", satBiasStd[18], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[19]", satBiasStd[19], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[20]", satBiasStd[20], DATA_TYPE_F32, float&);
    ADD_MAP_4("satBiasStd[21]", satBiasStd[21], DATA_TYPE_F32, float&);

    ADD_MAP_4("satLockCnt[0]", satLockCnt[0], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[1]", satLockCnt[1], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[2]", satLockCnt[2], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[3]", satLockCnt[3], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[4]", satLockCnt[4], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[5]", satLockCnt[5], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[6]", satLockCnt[6], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[7]", satLockCnt[7], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[8]", satLockCnt[8], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[9]", satLockCnt[9], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[10]", satLockCnt[10], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[11]", satLockCnt[11], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[12]", satLockCnt[12], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[13]", satLockCnt[13], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[14]", satLockCnt[14], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[15]", satLockCnt[15], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[16]", satLockCnt[16], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[17]", satLockCnt[17], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[18]", satLockCnt[18], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[19]", satLockCnt[19], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[20]", satLockCnt[20], DATA_TYPE_INT8, int8_t&);
    ADD_MAP_4("satLockCnt[21]", satLockCnt[21], DATA_TYPE_INT8, int8_t&);

#endif

    ADD_MAP_4("num_biases", num_biases, DATA_TYPE_UINT8, uint8_t);

    ASSERT_SIZE(totalSize);
}
#endif

#endif // USE_IS_INTERNAL


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
    PopulateIMU3Mappings(m_lookupInfo, m_indexInfo, DID_IMU3_UNCAL);
    PopulateIMU3Mappings(m_lookupInfo, m_indexInfo, DID_IMU3_RAW);
    PopulateIMUMappings(m_lookupInfo, m_indexInfo, DID_IMU_RAW);
    PopulateIMUMappings(m_lookupInfo, m_indexInfo, DID_IMU);
    PopulateIMUDeltaThetaVelocityMappings(m_lookupInfo, m_indexInfo, DID_PIMU);
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
    PopulateEvbStatusMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbFlashCfgMappings(m_lookupInfo, m_indexInfo);
    PopulateDebugArrayMappings(m_lookupInfo, m_indexInfo, DID_EVB_DEBUG_ARRAY);
    PopulateDeviceInfoMappings(m_lookupInfo, m_indexInfo, DID_EVB_DEV_INFO);
    PopulateIOMappings(m_lookupInfo, m_indexInfo);
    PopulateReferenceIMUMappings(m_lookupInfo, m_indexInfo);
    PopulateIMUDeltaThetaVelocityMappings(m_lookupInfo, m_indexInfo, DID_REFERENCE_PIMU);
    PopulateInfieldCalMappings(m_lookupInfo, m_indexInfo);

    PopulateISEventMappings(m_lookupInfo, m_indexInfo);

    PopulateDeviceInfoMappings(m_lookupInfo, m_indexInfo, DID_GPX_DEV_INFO);
    PopulateGpxFlashCfgMappings(m_lookupInfo, m_indexInfo);
    // DID_GPX_RTOS_INFO
    PopulateGpxStatusMappings(m_lookupInfo, m_indexInfo);
    PopulateDebugArrayMappings(m_lookupInfo, m_indexInfo, DID_GPX_DEBUG_ARRAY);

#if defined(INCLUDE_LUNA_DATA_SETS)
    PopulateEvbLunaFlashCfgMappings(m_lookupInfo, m_indexInfo);
    PopulateCoyoteStatusMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbLunaSensorsMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbLunaVelocityControlMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbLunaVelocityCommandMappings(m_lookupInfo, m_indexInfo);
    PopulateEvbLunaAuxCmdMappings(m_lookupInfo, m_indexInfo);
#endif

    PopulateStrobeInTimeMappings(m_lookupInfo, m_indexInfo);
//    PopulateRtosInfoMappings(m_lookupInfo, m_indexInfo);
    PopulateDiagMsgMappings(m_lookupInfo, m_indexInfo);
    PopulateCanConfigMappings(m_lookupInfo, m_indexInfo);

#ifdef USE_IS_INTERNAL

    PopulateSensorsADCMappings(m_lookupInfo, m_indexInfo);
    PopulateSensorsISMappings(m_lookupInfo, m_indexInfo, DID_SENSORS_UCAL);
    PopulateSensorsISMappings(m_lookupInfo, m_indexInfo, DID_SENSORS_TCAL);
    PopulateSensorsISMappings(m_lookupInfo, m_indexInfo, DID_SENSORS_MCAL);
    PopulateSensorsTCMappings(m_lookupInfo, m_indexInfo);
    PopulateSensorsCompMappings(m_lookupInfo, m_indexInfo);
    PopulateUserPage0Mappings(m_lookupInfo, m_indexInfo);
    PopulateUserPage1Mappings(m_lookupInfo, m_indexInfo);
    PopulateInl2MagObsInfo(m_lookupInfo, m_indexInfo);
    PopulateInl2StatesMappings(m_lookupInfo, m_indexInfo);
//     PopulateRtkStateMappings(m_lookupInfo, m_indexInfo);
//     PopulateRtkResidualMappings(m_lookupInfo, m_indexInfo, DID_RTK_CODE_RESIDUAL);
//     PopulateRtkResidualMappings(m_lookupInfo, m_indexInfo, DID_RTK_PHASE_RESIDUAL);
    PopulateRtkDebugMappings(m_lookupInfo, m_indexInfo);
    // PopulateRtkDebug2Mappings(m_lookupInfo, m_indexInfo);
    PopulateIMUDeltaThetaVelocityMagMappings(m_lookupInfo, m_indexInfo);
    PopulateIMUMagnetometerMappings(m_lookupInfo, m_indexInfo);

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
