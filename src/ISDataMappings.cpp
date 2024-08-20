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

#define SYMBOL_DEGREES     "°"


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
#define ADD_MAP_NO_VALIDATION1(name, member, dataSize, dataType, fieldType, flags)                                  map[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize), dataType, (eDataFlags)flags, name, "", "", 0 };                      idx[fieldCount++] = &(map[std::string(name)]); totalSize += sizeof(fieldType);
#define ADD_MAP_NO_VALIDATION2(name, member, dataSize, dataType, fieldType, flags, units, description)              map[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize), dataType, (eDataFlags)flags, name, units, description, 0 };          idx[fieldCount++] = &(map[std::string(name)]); totalSize += sizeof(fieldType);
#define ADD_MAP_NO_VALIDATION3(name, member, dataSize, dataType, fieldType, units, description, flags, conversion)  map[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize), dataType, (eDataFlags)flags, name, units, description, conversion }; idx[fieldCount++] = &(map[std::string(name)]); totalSize += sizeof(fieldType);

// note when passing member type for arrays, it must be a reference, i.e. float&
#define ADD_MAP(name, member, dataSize, dataType, fieldType, dataFlags) \
    ADD_MAP_NO_VALIDATION1(name, member, dataSize, dataType, fieldType, dataFlags); \
    static_assert(is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP2(name, member, dataSize, dataType, fieldType, dataFlags, units, description) \
    ADD_MAP_NO_VALIDATION2(name, member, dataSize, dataType, fieldType, dataFlags, units, description); \
    static_assert(is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ADD_MAP3(name, member, dataSize, dataType, fieldType, units, description, dataFlags, conversion) \
    ADD_MAP_NO_VALIDATION3(name, member, dataSize, dataType, fieldType, units, description, dataFlags, conversion); \
    static_assert(is_same<decltype(MAP_TYPE::member), fieldType>::value, "Field type is an unexpected type"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == sizeof(fieldType), "Field type is an unexpected size, sizeof(fieldType)"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == sizeof(MAP_TYPE::member), "Field type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ASSERT_SIZE(s) assert(s == sizeof(MAP_TYPE))

#else

#define ADD_MAP_NO_VALIDATION(name, member, dataSize, dataType, fieldType, dataFlags) map[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)(dataSize == 0 ? sizeof(fieldType) : dataSize), dataType, (eDataFlags)dataFlags, name }; totalSize += sizeof(fieldType);
#define ADD_MAP(name, member, dataSize, dataType, fieldType, dataFlags) ADD_MAP_NO_VALIDATION(name, member, dataSize, dataType, fieldType, dataFlags)
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
    
    ADD_MAP("reserved", reserved, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("reserved2", reserved2, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("hardwareType", hardwareType, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("serialNumber", serialNumber, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("hardwareVer[0]", hardwareVer[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("hardwareVer[1]", hardwareVer[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("hardwareVer[2]", hardwareVer[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("hardwareVer[3]", hardwareVer[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("firmwareVer[0]", firmwareVer[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("firmwareVer[1]", firmwareVer[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("firmwareVer[2]", firmwareVer[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("firmwareVer[3]", firmwareVer[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("buildNumber", buildNumber, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("protocolVer[0]", protocolVer[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("protocolVer[1]", protocolVer[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("protocolVer[2]", protocolVer[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("protocolVer[3]", protocolVer[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("repoRevision", repoRevision, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("manufacturer", manufacturer, DEVINFO_MANUFACTURER_STRLEN, DATA_TYPE_STRING, char[DEVINFO_MANUFACTURER_STRLEN], 0);
    ADD_MAP("buildType", buildType, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("buildYear", buildYear, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("buildMonth", buildMonth, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("buildDay", buildDay, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("buildHour", buildHour, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("buildMinute", buildMinute, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("buildSecond", buildSecond, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("buildMillisecond", buildMillisecond, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("addInfo", addInfo, DEVINFO_ADDINFO_STRLEN, DATA_TYPE_STRING, char[DEVINFO_ADDINFO_STRLEN], 0);
    // TODO: dev_info_t.firmwareMD5Hash support
    // ADD_MAP("firmwareMD5Hash[0]", firmwareMD5Hash[0], 0, DATA_TYPE_UINT32, uint32_t&, 0);
    // ADD_MAP("firmwareMD5Hash[1]", firmwareMD5Hash[1], 0, DATA_TYPE_UINT32, uint32_t&, 0);
    // ADD_MAP("firmwareMD5Hash[2]", firmwareMD5Hash[2], 0, DATA_TYPE_UINT32, uint32_t&, 0);
    // ADD_MAP("firmwareMD5Hash[3]", firmwareMD5Hash[3], 0, DATA_TYPE_UINT32, uint32_t&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateManufacturingInfoMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(manufacturing_info_t, DID_MANUFACTURING_INFO);

    ADD_MAP("serialNumber", serialNumber, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("hardwareId", hardwareId, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("lotNumber", lotNumber, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("date", date, 16, DATA_TYPE_STRING, char[16], 0);
    ADD_MAP("key", key, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("platformType", platformType, 0, DATA_TYPE_INT32, int32_t, 0);
    ADD_MAP("reserved", reserved, 0, DATA_TYPE_INT32, int32_t, 0);
    ADD_MAP("uid[0]", uid[0], 0, DATA_TYPE_UINT32, uint32_t&, 0);
    ADD_MAP("uid[1]", uid[1], 0, DATA_TYPE_UINT32, uint32_t&, 0);
    ADD_MAP("uid[2]", uid[2], 0, DATA_TYPE_UINT32, uint32_t&, 0);
    ADD_MAP("uid[3]", uid[3], 0, DATA_TYPE_UINT32, uint32_t&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateIOMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(io_t, DID_IO);

    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("gpioStatus", gpioStatus, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateBitMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(bit_t, DID_BIT);

    ADD_MAP("command", command, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("lastCommand", lastCommand, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("state", state, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("reserved", reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("hdwBitStatus", hdwBitStatus, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("calBitStatus", calBitStatus, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("tcPqrBias", tcPqrBias, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("tcAccBias", tcAccBias, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("tcPqrSlope", tcPqrSlope, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("tcAccSlope", tcAccSlope, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("tcPqrLinearity", tcPqrLinearity, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("tcAccLinearity", tcAccLinearity, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("pqr", pqr, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc", acc, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqrSigma", pqrSigma, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("accSigma", accSigma, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("testMode", testMode, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("testVar", testVar, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("detectedHardwareId", detectedHardwareId, 0, DATA_TYPE_UINT16, uint16_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpxBitMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(gpx_bit_t, DID_GPX_BIT);

    ADD_MAP("results", results, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("command", command, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("port", port, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("testMode", testMode, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("state", state, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("detectedHardwareId", detectedHardwareId, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("reserved[0]", reserved[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("reserved[1]", reserved[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysFaultMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(system_fault_t, DID_SYS_FAULT);

    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("g1Task", g1Task, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("g2FileNum", g2FileNum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("g3LineNum", g3LineNum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("g4", g4, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("g5Lr", g5Lr, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pc", pc, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("psr", psr, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t dataId)
{
    INIT_MAP(imu_t, dataId);
    
    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("pqr[0]", I.pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr[1]", I.pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr[2]", I.pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[0]", I.acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[1]", I.acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[2]", I.acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMU3Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t dataId)
{
    INIT_MAP(imu3_t, dataId);

    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("I0.pqr[0]", I[0].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I0.pqr[1]", I[0].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I0.pqr[2]", I[0].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I0.acc[0]", I[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I0.acc[1]", I[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I0.acc[2]", I[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I1.pqr[0]", I[1].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I1.pqr[1]", I[1].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I1.pqr[2]", I[1].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I1.acc[0]", I[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I1.acc[1]", I[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I1.acc[2]", I[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I2.pqr[0]", I[2].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I2.pqr[1]", I[2].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I2.pqr[2]", I[2].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I2.acc[0]", I[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I2.acc[1]", I[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I2.acc[2]", I[2].acc[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysParamsMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sys_params_t, DID_SYS_PARAMS);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("insStatus", insStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("hdwStatus", hdwStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("imuTemp", imuTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("baroTemp", baroTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mcuTemp", mcuTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("sysStatus", sysStatus, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("imuSamplePeriodMs", imuSamplePeriodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("navOutputPeriodMs", navOutputPeriodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("sensorTruePeriod", sensorTruePeriod, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("flashCfgChecksum", flashCfgChecksum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("navUpdatePeriodMs", navUpdatePeriodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("genFaultCode", genFaultCode, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("upTime", upTime, 0, DATA_TYPE_F64, double, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysSensorsMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sys_sensors_t, DID_SYS_SENSORS);
    
    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("temp", temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr[0]", pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr[1]", pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr[2]", pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[0]", acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[1]", acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[2]", acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[0]", mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[1]", mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[2]", mag[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("bar", bar, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("barTemp", barTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mslBar", mslBar, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("humidity", humidity, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vin", vin, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("ana1", ana1, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("ana3", ana1, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("ana4", ana1, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateRMCMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rmc_t, DID_RMC);
    
    ADD_MAP("bits", bits, 0, DATA_TYPE_UINT64, uint64_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("options", options, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS1Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ins_1_t, DID_INS_1);
    
    ADD_MAP("week", week, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("timeOfWeek", timeOfWeek, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("insStatus", insStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("hdwStatus", hdwStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("theta[0]", theta[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("theta[1]", theta[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("theta[2]", theta[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[0]", uvw[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[1]", uvw[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[2]", uvw[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("lla[0]", lla[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[1]", lla[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[2]", lla[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("ned[0]", ned[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ned[1]", ned[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ned[2]", ned[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS2Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ins_2_t, DID_INS_2);
    
    ADD_MAP("week", week, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("timeOfWeek", timeOfWeek, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("insStatus", insStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("hdwStatus", hdwStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("qn2b[0]", qn2b[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qn2b[1]", qn2b[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qn2b[2]", qn2b[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qn2b[3]", qn2b[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[0]", uvw[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[1]", uvw[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[2]", uvw[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("lla[0]", lla[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[1]", lla[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[2]", lla[2], 0, DATA_TYPE_F64, double&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS3Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ins_3_t, DID_INS_3);
    
    ADD_MAP("week", week, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("timeOfWeek", timeOfWeek, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("insStatus", insStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("hdwStatus", hdwStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("qn2b[0]", qn2b[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qn2b[1]", qn2b[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qn2b[2]", qn2b[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qn2b[3]", qn2b[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[0]", uvw[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[1]", uvw[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("uvw[2]", uvw[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("lla[0]", lla[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[1]", lla[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[2]", lla[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("msl", msl, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS4Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ins_4_t, DID_INS_4);
    
    ADD_MAP("week", week, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("timeOfWeek", timeOfWeek, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("insStatus", insStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("hdwStatus", hdwStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("qe2b[0]", qe2b[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qe2b[1]", qe2b[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qe2b[2]", qe2b[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qe2b[3]", qe2b[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ve[0]", ve[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ve[1]", ve[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ve[2]", ve[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ecef[0]", ecef[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("ecef[1]", ecef[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("ecef[2]", ecef[2], 0, DATA_TYPE_F64, double&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsPosMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_pos_t, id);
    
    ADD_MAP("week", week, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("ecef[0]", ecef[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("ecef[1]", ecef[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("ecef[2]", ecef[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[0]", lla[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[1]", lla[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lla[2]", lla[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("hMSL", hMSL, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("hAcc", hAcc, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vAcc", vAcc, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pDop", pDop, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("cnoMean", cnoMean, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("towOffset", towOffset, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("leapS", leapS, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("satsUsed", satsUsed, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("cnoMeanSigma", cnoMeanSigma, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("reserved", reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsVelMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_vel_t, id);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("vel[0]", vel[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("vel[1]", vel[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("vel[2]", vel[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("sAcc", sAcc, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsSatMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_sat_t, id);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("numSats", numSats, 0, DATA_TYPE_UINT32, uint32_t, 0);

#define ADD_MAP_SAT_INFO(n) \
    ADD_MAP("sat" #n ".gnssId",    sat[n].gnssId,    0, DATA_TYPE_UINT8, uint8_t, 0); \
    ADD_MAP("sat" #n ".svId",      sat[n].svId,      0, DATA_TYPE_UINT8, uint8_t, 0); \
    ADD_MAP("sat" #n ".elev",      sat[n].elev,      0, DATA_TYPE_INT8,  int8_t,  0); \
    ADD_MAP("sat" #n ".azim",      sat[n].azim,      0, DATA_TYPE_INT16, int16_t, 0); \
    ADD_MAP("sat" #n ".cno",       sat[n].cno,       0, DATA_TYPE_UINT8, uint8_t, 0); \
    ADD_MAP("sat" #n ".status",    sat[n].status,    0, DATA_TYPE_UINT16, uint16_t, 0);

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
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("numSigs", numSigs, 0, DATA_TYPE_UINT32, uint32_t, 0);

#define ADD_MAP_SAT_SIG(n) \
    ADD_MAP("sig" #n ".gnssId",    sig[n].gnssId,    0, DATA_TYPE_UINT8, uint8_t, 0); \
    ADD_MAP("sig" #n ".svId",      sig[n].svId,      0, DATA_TYPE_UINT8, uint8_t, 0); \
    ADD_MAP("sig" #n ".sigId",     sig[n].sigId,     0, DATA_TYPE_UINT8, uint8_t, 0); \
    ADD_MAP("sig" #n ".cno",       sig[n].cno,       0, DATA_TYPE_UINT8, uint8_t, 0); \
    ADD_MAP("sig" #n ".quality",   sig[n].quality,   0, DATA_TYPE_UINT8, uint8_t, 0); \
    ADD_MAP("sig" #n ".status",    sig[n].status,    0, DATA_TYPE_UINT16, uint16_t, 0);

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
    
    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("mag[0]", mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[1]", mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[2]", mag[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateBarometerMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(barometer_t, DID_BAROMETER);
    
    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("bar", bar, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mslBar", mslBar, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("barTemp", barTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("humidity", humidity, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t did)
{
    INIT_MAP(pimu_t, did);
    
    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("theta[0]", theta[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("theta[1]", theta[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("theta[2]", theta[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("vel[0]", vel[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("vel[1]", vel[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("vel[2]", vel[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("dt", dt, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMagMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(pimu_mag_t, DID_PIMU_MAG);
    
    ADD_MAP("imutime", pimu.time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("theta[0]", pimu.theta[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("theta[1]", pimu.theta[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("theta[2]", pimu.theta[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("vel[0]", pimu.vel[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("vel[1]", pimu.vel[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("vel[2]", pimu.vel[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("dt", pimu.dt, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("imustatus", pimu.status, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("magtime", mag.time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("mag[0]", mag.mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[1]", mag.mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[2]", mag.mag[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMagnetometerMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(imu_mag_t, DID_IMU_MAG);
    
    ADD_MAP("imutime", imu.time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("pqr[0]", imu.I.pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr[1]", imu.I.pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr[2]", imu.I.pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[0]", imu.I.acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[1]", imu.I.acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc[2]", imu.I.acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imustatus", imu.status, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("magtime", mag.time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("mag[0]", mag.mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[1]", mag.mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag[2]", mag.mag[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);

}


static void PopulateInfieldCalMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(infield_cal_t, DID_INFIELD_CAL);
    
    ADD_MAP("state", state, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("sampleTimeMs", sampleTimeMs, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("imu[0].pqr[0]", imu[0].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[0].pqr[1]", imu[0].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[0].pqr[2]", imu[0].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[0].acc[0]", imu[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[0].acc[1]", imu[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[0].acc[2]", imu[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[1].pqr[0]", imu[1].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[1].pqr[1]", imu[1].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[1].pqr[2]", imu[1].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[1].acc[0]", imu[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[1].acc[1]", imu[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[1].acc[2]", imu[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[2].pqr[0]", imu[2].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[2].pqr[1]", imu[2].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[2].pqr[2]", imu[2].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[2].acc[0]", imu[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[2].acc[1]", imu[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("imu[2].acc[2]", imu[2].acc[2], 0, DATA_TYPE_F32, float&, 0);

    ADD_MAP("calData[0].down.dev[0].acc[0]", calData[0].down.dev[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.dev[0].acc[1]", calData[0].down.dev[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.dev[0].acc[2]", calData[0].down.dev[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.dev[1].acc[0]", calData[0].down.dev[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.dev[1].acc[1]", calData[0].down.dev[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.dev[1].acc[2]", calData[0].down.dev[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.dev[2].acc[0]", calData[0].down.dev[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.dev[2].acc[1]", calData[0].down.dev[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.dev[2].acc[2]", calData[0].down.dev[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].down.yaw", calData[0].down.yaw, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("calData[0].up.dev[0].acc[0]", calData[0].up.dev[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.dev[0].acc[1]", calData[0].up.dev[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.dev[0].acc[2]", calData[0].up.dev[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.dev[1].acc[0]", calData[0].up.dev[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.dev[1].acc[1]", calData[0].up.dev[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.dev[1].acc[2]", calData[0].up.dev[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.dev[2].acc[0]", calData[0].up.dev[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.dev[2].acc[1]", calData[0].up.dev[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.dev[2].acc[2]", calData[0].up.dev[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[0].up.yaw", calData[0].up.yaw, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("calData[1].down.dev[0].acc[0]", calData[1].down.dev[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.dev[0].acc[1]", calData[1].down.dev[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.dev[0].acc[2]", calData[1].down.dev[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.dev[1].acc[0]", calData[1].down.dev[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.dev[1].acc[1]", calData[1].down.dev[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.dev[1].acc[2]", calData[1].down.dev[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.dev[2].acc[0]", calData[1].down.dev[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.dev[2].acc[1]", calData[1].down.dev[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.dev[2].acc[2]", calData[1].down.dev[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].down.yaw", calData[1].down.yaw, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("calData[1].up.dev[0].acc[0]", calData[1].up.dev[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.dev[0].acc[1]", calData[1].up.dev[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.dev[0].acc[2]", calData[1].up.dev[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.dev[1].acc[0]", calData[1].up.dev[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.dev[1].acc[1]", calData[1].up.dev[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.dev[1].acc[2]", calData[1].up.dev[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.dev[2].acc[0]", calData[1].up.dev[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.dev[2].acc[1]", calData[1].up.dev[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.dev[2].acc[2]", calData[1].up.dev[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[1].up.yaw", calData[1].up.yaw, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("calData[2].down.dev[0].acc[0]", calData[2].down.dev[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.dev[0].acc[1]", calData[2].down.dev[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.dev[0].acc[2]", calData[2].down.dev[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.dev[1].acc[0]", calData[2].down.dev[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.dev[1].acc[1]", calData[2].down.dev[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.dev[1].acc[2]", calData[2].down.dev[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.dev[2].acc[0]", calData[2].down.dev[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.dev[2].acc[1]", calData[2].down.dev[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.dev[2].acc[2]", calData[2].down.dev[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].down.yaw", calData[2].down.yaw, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("calData[2].up.dev[0].acc[0]", calData[2].up.dev[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.dev[0].acc[1]", calData[2].up.dev[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.dev[0].acc[2]", calData[2].up.dev[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.dev[1].acc[0]", calData[2].up.dev[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.dev[1].acc[1]", calData[2].up.dev[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.dev[1].acc[2]", calData[2].up.dev[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.dev[2].acc[0]", calData[2].up.dev[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.dev[2].acc[1]", calData[2].up.dev[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.dev[2].acc[2]", calData[2].up.dev[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("calData[2].up.yaw", calData[2].up.yaw, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateReferenceIMUMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(imu_t, DID_REFERENCE_IMU);
    
    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("I.pqr[0]", I.pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I.pqr[1]", I.pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I.pqr[2]", I.pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I.acc[0]", I.acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I.acc[1]", I.acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("I.acc[2]", I.acc[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateWheelEncoderMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(wheel_encoder_t, DID_WHEEL_ENCODER);
    
    ADD_MAP("timeOfWeek", timeOfWeek, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("theta_l", theta_l, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("omega_l", omega_l, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("theta_r", theta_r, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("omega_r", omega_r, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wrap_count_l", wrap_count_l, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("wrap_count_r", wrap_count_r, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGroundVehicleMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(ground_vehicle_t, DID_GROUND_VEHICLE);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("mode", mode, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("wheelConfig.bits", wheelConfig.bits, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.track_width", wheelConfig.track_width, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheelConfig.radius", wheelConfig.radius, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateConfigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(system_command_t, DID_SYS_CMD);
    
    ADD_MAP("command", command, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("invCommand", invCommand, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateFlashConfigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(nvm_flash_cfg_t, DID_FLASH_CONFIG);
    
    string degrees = SYMBOL_DEGREES;
    string str;
    ADD_MAP2("startupImuDtMs", startupImuDtMs, 0, DATA_TYPE_UINT32, uint32_t, 0, "ms", "IMU sample (system input data) period set on startup. Cannot be larger than startupInsDtMs. Zero disables sensor/IMU sampling.");
    ADD_MAP2("startupNavDtMs", startupNavDtMs, 0, DATA_TYPE_UINT32, uint32_t, 0, "ms", "GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max).");
    ADD_MAP2("startupGPSDtMs", startupGPSDtMs, 0, DATA_TYPE_UINT32, uint32_t, 0, "ms", "Nav filter (system output data) update period set on startup. 1ms min (1KHz max).");
    ADD_MAP2("ser0BaudRate", ser0BaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0, "bps", "Serial port 0 baud rate");
    ADD_MAP2("ser1BaudRate", ser1BaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0, "bps", "Serial port 1 baud rate");
    ADD_MAP2("ser2BaudRate", ser2BaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0, "bps", "Serial port 2 baud rate");
    str = "rotation from INS Sensor Frame to Intermediate Output Frame.  Order applied: yaw, pitch, roll.";
    ADD_MAP2("insRotation[0]", insRotation[0], 0, DATA_TYPE_F32, float&, 0, degrees, "Roll "  + str); // , 0, C_RAD2DEG);
    ADD_MAP2("insRotation[1]", insRotation[1], 0, DATA_TYPE_F32, float&, 0, degrees, "Pitch " + str); // , 0, C_RAD2DEG);
    ADD_MAP2("insRotation[2]", insRotation[2], 0, DATA_TYPE_F32, float&, 0, degrees, "Yaw "   + str); // , 0, C_RAD2DEG);
    str = "offset from Intermediate Output Frame to INS Output Frame.  INS rotation is applied before this.";
    ADD_MAP2("insOffset[0]", insOffset[0], 0, DATA_TYPE_F32, float&, 0, "m", "X " + str);
    ADD_MAP2("insOffset[1]", insOffset[1], 0, DATA_TYPE_F32, float&, 0, "m", "Y " + str);
    ADD_MAP2("insOffset[2]", insOffset[2], 0, DATA_TYPE_F32, float&, 0, "m", "Z " + str);
    str = "offset from Sensor Frame origin to GPS1 antenna.";
    ADD_MAP2("gps1AntOffset[0]", gps1AntOffset[0], 0, DATA_TYPE_F32, float&, 0, "m", "X " + str);
    ADD_MAP2("gps1AntOffset[1]", gps1AntOffset[1], 0, DATA_TYPE_F32, float&, 0, "m", "Y " + str);
    ADD_MAP2("gps1AntOffset[2]", gps1AntOffset[2], 0, DATA_TYPE_F32, float&, 0, "m", "Z " + str);
    str = "offset from Sensor Frame origin to GPS2 antenna.";
    ADD_MAP2("gps2AntOffset[0]", gps2AntOffset[0], 0, DATA_TYPE_F32, float&, 0, "m", "X " + str);
    ADD_MAP2("gps2AntOffset[1]", gps2AntOffset[1], 0, DATA_TYPE_F32, float&, 0, "m", "Y " + str);
    ADD_MAP2("gps2AntOffset[2]", gps2AntOffset[2], 0, DATA_TYPE_F32, float&, 0, "m", "Z " + str);

    ADD_MAP3("gpsTimeSyncPeriodMs", gpsTimeSyncPeriodMs, 0, DATA_TYPE_UINT32, uint32_t, "ms", "GPS time synchronization pulse period.", 0, 1.0);
    ADD_MAP3("gpsTimeUserDelay", gpsTimeUserDelay, 0, DATA_TYPE_F32, float, "s", "User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.", DATA_FLAG_FIXED_DECIMAL_3, 1.0);
    ADD_MAP3("gpsMinimumElevation", gpsMinimumElevation, 0, DATA_TYPE_F32, float, degrees, "GPS minimum elevation of a satellite above the horizon to be used in the solution.", DATA_FLAG_FIXED_DECIMAL_1, C_RAD2DEG);
    ADD_MAP3("gnssSatSigConst", gnssSatSigConst, 0, DATA_TYPE_UINT16, uint16_t, "", "GNSS constellations used. 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS (see eGnssSatSigConst)", DATA_FLAG_DISPLAY_HEX, 1.0);

    ADD_MAP("dynamicModel", dynamicModel, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("debug", debug, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("sysCfgBits", sysCfgBits, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("refLla[0]", refLla[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("refLla[1]", refLla[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("refLla[2]", refLla[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lastLla[0]", lastLla[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lastLla[1]", lastLla[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lastLla[2]", lastLla[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lastLlaTimeOfWeekMs", lastLlaTimeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("lastLlaWeek", lastLlaWeek, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("lastLlaUpdateDistance", lastLlaUpdateDistance, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("ioConfig", ioConfig, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("platformConfig", platformConfig, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("magDeclination", magDeclination, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("zeroVelRotation[0]", zeroVelRotation[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("zeroVelRotation[1]", zeroVelRotation[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("zeroVelRotation[2]", zeroVelRotation[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("zeroVelOffset[0]", zeroVelOffset[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("zeroVelOffset[1]", zeroVelOffset[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("zeroVelOffset[2]", zeroVelOffset[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("RTKCfgBits", RTKCfgBits, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("sensorConfig", sensorConfig, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("wheelConfig.bits", wheelConfig.bits, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("wheelConfig.track_width", wheelConfig.track_width, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheelConfig.radius", wheelConfig.radius, 0, DATA_TYPE_F32, float, 0);
	ADD_MAP("magInterferenceThreshold", magInterferenceThreshold, 0, DATA_TYPE_F32, float, 0);
	ADD_MAP("magCalibrationQualityThreshold", magCalibrationQualityThreshold, 0, DATA_TYPE_F32, float, 0);

    // Keep at end
    ADD_MAP("size", size, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("checksum", checksum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("key", key, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

/**
 * Maps DID_EVENT for SDK
*/
static void PopulateISEventMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(did_event_t, DID_EVENT);

    ADD_MAP("Time stamp of message (System Up seconds)", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("Senders serial number", senderSN, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("Sender hardware type", senderHdwId, 0, DATA_TYPE_UINT16, uint16_t, 0);

    ADD_MAP("Message ID", msgTypeID, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("Priority", priority, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("Length", length, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("data", data, 0, DATA_TYPE_STRING, uint8_t[MEMBERSIZE(MAP_TYPE, data)], 0);

    ADD_MAP("Reserved 8 bit", res8, 0, DATA_TYPE_UINT8, uint8_t, 0);
}

static void PopulateGpxFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(gpx_flash_cfg_t, DID_GPX_FLASH_CFG);
    
    ADD_MAP("size", size, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("checksum", checksum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("key", key, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("ser0BaudRate", ser0BaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("ser1BaudRate", ser1BaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("ser2BaudRate", ser2BaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("startupGPSDtMs", startupGPSDtMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("gps1AntOffset[0]", gps1AntOffset[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("gps1AntOffset[1]", gps1AntOffset[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("gps1AntOffset[2]", gps1AntOffset[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("gps2AntOffset[0]", gps2AntOffset[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("gps2AntOffset[1]", gps2AntOffset[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("gps2AntOffset[2]", gps2AntOffset[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("gnssSatSigConst", gnssSatSigConst, 0, DATA_TYPE_UINT16, uint16_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("dynamicModel", dynamicModel, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("debug", debug, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("gpsTimeSyncPeriodMs", gpsTimeSyncPeriodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("gpsTimeUserDelay", gpsTimeUserDelay, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("gpsMinimumElevation", gpsMinimumElevation, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("RTKCfgBits", RTKCfgBits, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpxStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(gpx_status_t, DID_GPX_STATUS);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("grmcBitsSer0", grmcBitsSer0, 0, DATA_TYPE_UINT64, uint64_t, 0);
    ADD_MAP("grmcBitsSer1", grmcBitsSer1, 0, DATA_TYPE_UINT64, uint64_t, 0);
    ADD_MAP("grmcBitsSer2", grmcBitsSer2, 0, DATA_TYPE_UINT64, uint64_t, 0);
    ADD_MAP("grmcBitsUSB", grmcBitsUSB, 0, DATA_TYPE_UINT64, uint64_t, 0);
    ADD_MAP("grmcNMEABitsSer0", grmcNMEABitsSer0, 0, DATA_TYPE_UINT64, uint64_t, 0);
    ADD_MAP("grmcNMEABitsSer1", grmcNMEABitsSer1, 0, DATA_TYPE_UINT64, uint64_t, 0);
    ADD_MAP("grmcNMEABitsSer2", grmcNMEABitsSer2, 0, DATA_TYPE_UINT64, uint64_t, 0);
    ADD_MAP("grmcNMEABitsUSB", grmcNMEABitsUSB, 0, DATA_TYPE_UINT64, uint64_t, 0);
    ADD_MAP("hdwStatus", hdwStatus, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("mcuTemp", mcuTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("navOutputPeriodMs", navOutputPeriodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("flashCfgChecksum", flashCfgChecksum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("rtkMode", rtkMode, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("gnss1RunState", gnss1RunState, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("gnss2RunState", gnss2RunState, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("SourcePort", gpxSourcePort, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("upTime", upTime, 0, DATA_TYPE_F64, double, 0);
}

static void PopulateEvbStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_status_t, DID_EVB_STATUS);
    
    ADD_MAP("week", week, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("firmwareVer[0]", firmwareVer[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("firmwareVer[1]", firmwareVer[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("firmwareVer[2]", firmwareVer[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("firmwareVer[3]", firmwareVer[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("evbStatus", evbStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("loggerMode", loggerMode, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("loggerElapsedTimeMs", loggerElapsedTimeMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("wifiIpAddr", wifiIpAddr, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("sysCommand", sysCommand, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("towOffset", towOffset, 0, DATA_TYPE_F64, double, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_flash_cfg_t, DID_EVB_FLASH_CFG);
    
    ADD_MAP("size", size, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("checksum", checksum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("key", key, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("cbPreset", cbPreset, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("reserved1[0]", reserved1[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("reserved1[1]", reserved1[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("reserved1[2]", reserved1[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("cbf[0]", cbf[0], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[1]", cbf[1], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[2]", cbf[2], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[3]", cbf[3], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[4]", cbf[4], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[5]", cbf[5], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[6]", cbf[6], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[7]", cbf[7], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[8]", cbf[8], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbf[9]", cbf[9], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("cbOptions", cbOptions, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("bits", bits, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("radioPID", radioPID, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("radioNID", radioNID, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("radioPowerLevel", radioPowerLevel, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("wifi[0].ssid", wifi[0].ssid, WIFI_SSID_PSK_SIZE, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP("wifi[0].psk", wifi[0].psk, WIFI_SSID_PSK_SIZE, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP("wifi[1].ssid", wifi[1].ssid, WIFI_SSID_PSK_SIZE, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP("wifi[1].psk", wifi[1].psk, WIFI_SSID_PSK_SIZE, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP("wifi[2].ssid", wifi[2].ssid, WIFI_SSID_PSK_SIZE, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP("wifi[2].psk", wifi[2].psk, WIFI_SSID_PSK_SIZE, DATA_TYPE_STRING, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP("server[0].ipAddr[0]", server[0].ipAddr.u8[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[0].ipAddr[1]", server[0].ipAddr.u8[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[0].ipAddr[2]", server[0].ipAddr.u8[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[0].ipAddr[3]", server[0].ipAddr.u8[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[0].port", server[0].port, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("server[1].ipAddr[0]", server[1].ipAddr.u8[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[1].ipAddr[1]", server[1].ipAddr.u8[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[1].ipAddr[2]", server[1].ipAddr.u8[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[1].ipAddr[3]", server[1].ipAddr.u8[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[1].port", server[1].port, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("server[2].ipAddr[0]", server[2].ipAddr.u8[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[2].ipAddr[1]", server[2].ipAddr.u8[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[2].ipAddr[2]", server[2].ipAddr.u8[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[2].ipAddr[3]", server[2].ipAddr.u8[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("server[2].port", server[2].port, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("encoderTickToWheelRad", encoderTickToWheelRad, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("CANbaud_kbps", CANbaud_kbps, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("can_receive_address", can_receive_address, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("uinsComPort", uinsComPort, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("uinsAuxPort", uinsAuxPort, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("reserved2[0]", reserved2[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("reserved2[1]", reserved2[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("portOptions", portOptions, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("h3sp330BaudRate", h3sp330BaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("h4xRadioBaudRate", h4xRadioBaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("h8gpioBaudRate", h8gpioBaudRate, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("wheelCfgBits", wheelCfgBits, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("velocityControlPeriodMs", velocityControlPeriodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateDebugArrayMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(debug_array_t, id);
    
    ADD_MAP("i[0]", i[0], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("i[1]", i[1], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("i[2]", i[2], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("i[3]", i[3], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("i[4]", i[4], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("i[5]", i[5], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("i[6]", i[6], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("i[7]", i[7], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("i[8]", i[8], 0, DATA_TYPE_INT32, int32_t&, 0);
    ADD_MAP("f[0]", f[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("f[1]", f[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("f[2]", f[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("f[3]", f[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("f[4]", f[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("f[5]", f[5], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("f[6]", f[6], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("f[7]", f[7], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("f[8]", f[8], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("lf[0]", lf[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lf[1]", lf[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("lf[2]", lf[2], 0, DATA_TYPE_F64, double&, 0);

    ASSERT_SIZE(totalSize);
}

#if defined(INCLUDE_LUNA_DATA_SETS)

static void PopulateEvbLunaFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_flash_cfg_t, DID_EVB_LUNA_FLASH_CFG);
    
    ADD_MAP("size", size, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("checksum", checksum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("key", key, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("bits", bits, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("minLatGeofence", minLatGeofence, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("maxLatGeofence", maxLatGeofence, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("minLonGeofence", minLonGeofence, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("maxLonGeofence", maxLonGeofence, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("remoteKillTimeoutMs", remoteKillTimeoutMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("bumpSensitivity", bumpSensitivity, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("minProxDistance", minProxDistance, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.config",                  velControl.config, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("velControl.cmdTimeoutMs",            velControl.cmdTimeoutMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("velControl.wheelRadius",             velControl.wheelRadius, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheelBaseline",           velControl.wheelBaseline, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.engine_rpm",              velControl.engine_rpm, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("velControl.vehicle.u_min",           velControl.vehicle.u_min, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.u_cruise",        velControl.vehicle.u_cruise, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.u_max",           velControl.vehicle.u_max, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.u_slewLimit",     velControl.vehicle.u_slewLimit, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.w_max_autonomous",velControl.vehicle.w_max_autonomous, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.w_max",           velControl.vehicle.w_max, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.w_slewLimit",     velControl.vehicle.w_slewLimit, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.u_FB_Kp",         velControl.vehicle.u_FB_Kp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.w_FB_Kp",         velControl.vehicle.w_FB_Kp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.w_FB_Ki",         velControl.vehicle.w_FB_Ki, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.w_FF_c0",         velControl.vehicle.w_FF_c0, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.w_FF_c1",         velControl.vehicle.w_FF_c1, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.w_FF_deadband",   velControl.vehicle.w_FF_deadband, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.vehicle.testSweepRate",   velControl.vehicle.testSweepRate, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("velControl.wheel.slewRate",          velControl.wheel.slewRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.velMax",            velControl.wheel.velMax, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.FF_vel_deadband",   velControl.wheel.FF_vel_deadband, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.FF_c_est_Ki[0]",    velControl.wheel.FF_c_est_Ki[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.FF_c_est_Ki[1]",    velControl.wheel.FF_c_est_Ki[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.FF_c_est_max[0]",   velControl.wheel.FF_c_est_max[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.FF_c_est_max[1]",   velControl.wheel.FF_c_est_max[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.FF_c_l[0]",         velControl.wheel.FF_c_l[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.FF_c_l[1]",         velControl.wheel.FF_c_l[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.FF_c_r[0]",         velControl.wheel.FF_c_r[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.FF_c_r[1]",         velControl.wheel.FF_c_r[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.FF_FB_engine_rpm",  velControl.wheel.FF_FB_engine_rpm, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.FB_Kp",             velControl.wheel.FB_Kp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.FB_Ki",             velControl.wheel.FB_Ki, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.FB_Kd",             velControl.wheel.FB_Kd, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.FB_gain_deadband",  velControl.wheel.FB_gain_deadband, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.FB_gain_deadband_reduction",  velControl.wheel.FB_gain_deadband_reduction, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.InversePlant_l[0]", velControl.wheel.InversePlant_l[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_l[1]", velControl.wheel.InversePlant_l[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_l[2]", velControl.wheel.InversePlant_l[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_l[3]", velControl.wheel.InversePlant_l[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_l[4]", velControl.wheel.InversePlant_l[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_r[0]", velControl.wheel.InversePlant_r[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_r[1]", velControl.wheel.InversePlant_r[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_r[2]", velControl.wheel.InversePlant_r[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_r[3]", velControl.wheel.InversePlant_r[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.InversePlant_r[4]", velControl.wheel.InversePlant_r[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.actuatorTrim_l",    velControl.wheel.actuatorTrim_l, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.actuatorTrim_r",    velControl.wheel.actuatorTrim_r, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.actuatorLimits_l[0]", velControl.wheel.actuatorLimits_l[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.actuatorLimits_l[1]", velControl.wheel.actuatorLimits_l[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.actuatorLimits_r[0]", velControl.wheel.actuatorLimits_r[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.actuatorLimits_r[1]", velControl.wheel.actuatorLimits_r[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("velControl.wheel.actuatorDeadbandDuty_l", velControl.wheel.actuatorDeadbandDuty_l, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.actuatorDeadbandDuty_r", velControl.wheel.actuatorDeadbandDuty_r, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("velControl.wheel.actuatorDeadbandVel", velControl.wheel.actuatorDeadbandVel, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateCoyoteStatusMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_status_t, DID_EVB_LUNA_STATUS);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("evbLunaStatus", evbLunaStatus, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("motorState", motorState, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("remoteKillMode", remoteKillMode, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("supplyVoltage", supplyVoltage, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaSensorsMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_sensors_t, DID_EVB_LUNA_SENSORS);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("proxSensorOutput[0]", proxSensorOutput[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("proxSensorOutput[1]", proxSensorOutput[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("proxSensorOutput[2]", proxSensorOutput[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("proxSensorOutput[3]", proxSensorOutput[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("proxSensorOutput[4]", proxSensorOutput[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("proxSensorOutput[5]", proxSensorOutput[5], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("proxSensorOutput[6]", proxSensorOutput[6], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("proxSensorOutput[7]", proxSensorOutput[7], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("proxSensorOutput[8]", proxSensorOutput[8], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("bumpEvent", bumpEvent, 0, DATA_TYPE_INT32, int32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaVelocityControlMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_velocity_control_t, DID_EVB_LUNA_VELOCITY_CONTROL);
    
    ADD_MAP("timeMs", timeMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("dt", dt, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("current_mode", current_mode, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("vehicle.velCmd_f", vehicle.velCmd_f, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.velCmd_w", vehicle.velCmd_w, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.velCmdMnl_f", vehicle.velCmdMnl_f, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.velCmdMnl_w", vehicle.velCmdMnl_w, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.velCmdSlew_f", vehicle.velCmdSlew_f, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.velCmdSlew_w", vehicle.velCmdSlew_w, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.vel_f", vehicle.vel_f, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.vel_w", vehicle.vel_w, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.err_f", vehicle.err_f, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.err_w", vehicle.err_w, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.eff_f", vehicle.eff_f, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vehicle.eff_w", vehicle.eff_w, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("wheel_l.velCmd",             wheel_l.velCmd, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.velCmdSlew",         wheel_l.velCmdSlew, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.vel",                wheel_l.vel, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.err",                wheel_l.err, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.ff_eff",             wheel_l.ff_eff, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.fb_eff",             wheel_l.fb_eff, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.fb_eff_integral",    wheel_l.fb_eff_integral, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.eff",                wheel_l.eff, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.effInt",             wheel_l.effInt, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_l.effDuty",            wheel_l.effDuty, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("wheel_r.velCmd",             wheel_r.velCmd, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.velCmdSlew",         wheel_r.velCmdSlew, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.vel",                wheel_r.vel, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.err",                wheel_r.err, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.ff_eff",             wheel_r.ff_eff, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.fb_eff",             wheel_r.fb_eff, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.fb_eff_integral",    wheel_r.fb_eff_integral, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.eff",                wheel_r.eff, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.effInt",             wheel_r.effInt, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("wheel_r.effDuty",            wheel_r.effDuty, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("potV_l", potV_l, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("potV_r", potV_r, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaVelocityCommandMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_velocity_command_t, DID_EVB_LUNA_VELOCITY_COMMAND);
    
    ADD_MAP("timeMs", timeMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("modeCmd", modeCmd, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("fwd_vel", fwd_vel, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("turn_rate", turn_rate, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaAuxCmdMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(evb_luna_aux_command_t, DID_EVB_LUNA_AUX_COMMAND);
    
    ADD_MAP("command", command, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

#endif

static void PopulateGpsRtkRelMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_rtk_rel_t, id);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("differentialAge", differentialAge, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("arRatio", arRatio, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("baseToRoverVector[0]", baseToRoverVector[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("baseToRoverVector[1]", baseToRoverVector[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("baseToRoverVector[2]", baseToRoverVector[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("baseToRoverDistance", baseToRoverDistance, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("baseToRoverHeading", baseToRoverHeading, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("baseToRoverHeadingAcc", baseToRoverHeadingAcc, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRtkMiscMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_rtk_misc_t, id);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("accuracyPos[0]", accuracyPos[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("accuracyPos[1]", accuracyPos[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("accuracyPos[2]", accuracyPos[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("accuracyCov[0]", accuracyCov[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("accuracyCov[1]", accuracyCov[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("accuracyCov[2]", accuracyCov[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("arThreshold", arThreshold, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("gDop", gDop, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("hDop", hDop, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("vDop", vDop, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("baseLla[0]", baseLla[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("baseLla[1]", baseLla[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("baseLla[2]", baseLla[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("cycleSlipCount", cycleSlipCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverGpsObservationCount", roverGpsObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseGpsObservationCount", baseGpsObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverGlonassObservationCount", roverGlonassObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseGlonassObservationCount", baseGlonassObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverGalileoObservationCount", roverGalileoObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseGalileoObservationCount", baseGalileoObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverBeidouObservationCount", roverBeidouObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseBeidouObservationCount", baseBeidouObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverQzsObservationCount", roverQzsObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseQzsObservationCount", baseQzsObservationCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverGpsEphemerisCount", roverGpsEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseGpsEphemerisCount", baseGpsEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverGlonassEphemerisCount", roverGlonassEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseGlonassEphemerisCount", baseGlonassEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverGalileoEphemerisCount", roverGalileoEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseGalileoEphemerisCount", baseGalileoEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverBeidouEphemerisCount", roverBeidouEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseBeidouEphemerisCount", baseBeidouEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverQzsEphemerisCount", roverQzsEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseQzsEphemerisCount", baseQzsEphemerisCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("roverSbasCount", roverSbasCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseSbasCount", baseSbasCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("baseAntennaCount", baseAntennaCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("ionUtcAlmCount", ionUtcAlmCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("correctionChecksumFailures", correctionChecksumFailures, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("timeToFirstFixMs", timeToFirstFixMs, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRawMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(gps_raw_t, id);
    

    ADD_MAP("receiveIndex", receiverIndex, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("dataType", dataType, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obsCount", obsCount, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("reserved", reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("dataBuf", data.buf, 0, DATA_TYPE_BINARY, uint8_t[MEMBERSIZE(MAP_TYPE, data.buf)], 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateStrobeInTimeMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(strobe_in_time_t, DID_STROBE_IN_TIME);
    

    ADD_MAP("week", week, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pin", pin, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("count", count, 0, DATA_TYPE_UINT16, uint16_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateRtosInfoMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rtos_info_t, DID_RTOS_INFO);
    

    ADD_MAP("freeHeapSize", freeHeapSize, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("mallocSize", mallocSize, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("freeSize", freeSize, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("T0_name", task[0].name, MAX_TASK_NAME_LEN, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP("T0_priority", task[0].priority, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T0_stackUnused", task[0].stackUnused, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T0_periodMs", task[0].periodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T0_runtimeUs", task[0].runtimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T0_avgRuntimeUs", task[0].avgRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T0_avgLowerRuntimeUs", task[0].lowerRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T0_avgUpperRuntimeUs", task[0].upperRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T0_maxRuntimeUs", task[0].maxRuntimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T0_startTimeUs", task[0].startTimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T0_gapCount", task[0].gapCount, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("T0_doubleGapCount", task[0].doubleGapCount, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T0_reserved", task[0].reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T0_cpuUsage", task[0].cpuUsage, 0, DATA_TYPE_F32, f_t, 0);
    ADD_MAP("T0_handle", task[0].handle, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("T1_name", task[1].name, MAX_TASK_NAME_LEN, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP("T1_priority", task[1].priority, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T1_stackUnused", task[1].stackUnused, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T1_periodMs", task[1].periodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T1_runtimeUs", task[1].runtimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T1_avgRuntimeUs", task[1].avgRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T1_avgLowerRuntimeUs", task[1].lowerRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T1_avgUpperRuntimeUs", task[1].upperRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T1_maxRuntimeUs", task[1].maxRuntimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T1_startTimeUs", task[1].startTimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T1_gapCount", task[1].gapCount, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("T1_doubleGapCount", task[1].doubleGapCount, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T1_reserved", task[1].reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T1_cpuUsage", task[1].cpuUsage, 0, DATA_TYPE_F32, f_t, 0);
    ADD_MAP("T1_handle", task[1].handle, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("T2_name", task[2].name, MAX_TASK_NAME_LEN, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP("T2_priority", task[2].priority, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T2_stackUnused", task[2].stackUnused, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T2_periodMs", task[2].periodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T2_runtimeUs", task[2].runtimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T2_avgRuntimeUs", task[2].avgRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T2_avgLowerRuntimeUs", task[2].lowerRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T2_avgUpperRuntimeUs", task[2].upperRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T2_maxRuntimeUs", task[2].maxRuntimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T2_startTimeUs", task[2].startTimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T2_gapCount", task[2].gapCount, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("T2_doubleGapCount", task[2].doubleGapCount, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T2_reserved", task[2].reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T2_cpuUsage", task[2].cpuUsage, 0, DATA_TYPE_F32, f_t, 0);
    ADD_MAP("T2_handle", task[2].handle, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("T3_name", task[3].name, MAX_TASK_NAME_LEN, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP("T3_priority", task[3].priority, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T3_stackUnused", task[3].stackUnused, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T3_periodMs", task[3].periodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T3_runtimeUs", task[3].runtimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T3_avgRuntimeUs", task[3].avgRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T3_avgLowerRuntimeUs", task[3].lowerRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T3_avgUpperRuntimeUs", task[3].upperRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T3_maxRuntimeUs", task[3].maxRuntimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T3_startTimeUs", task[3].startTimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T3_gapCount", task[3].gapCount, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("T3_doubleGapCount", task[3].doubleGapCount, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T3_reserved", task[3].reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T3_cpuUsage", task[3].cpuUsage, 0, DATA_TYPE_F32, f_t, 0);
    ADD_MAP("T3_handle", task[3].handle, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("T4_name", task[4].name, MAX_TASK_NAME_LEN, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP("T4_priority", task[4].priority, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T4_stackUnused", task[4].stackUnused, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T4_periodMs", task[4].periodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T4_runtimeUs", task[4].runtimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T4_avgRuntimeUs", task[4].avgRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T4_avgLowerRuntimeUs", task[4].lowerRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T4_avgUpperRuntimeUs", task[4].upperRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T4_maxRuntimeUs", task[4].maxRuntimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T4_startTimeUs", task[4].startTimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T4_gapCount", task[4].gapCount, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("T4_doubleGapCount", task[4].doubleGapCount, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T4_reserved", task[4].reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T4_cpuUsage", task[4].cpuUsage, 0, DATA_TYPE_F32, f_t, 0);
    ADD_MAP("T4_handle", task[4].handle, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("T5_name", task[5].name, MAX_TASK_NAME_LEN, DATA_TYPE_STRING, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP("T5_priority", task[5].priority, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T5_stackUnused", task[5].stackUnused, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T5_periodMs", task[5].periodMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T5_runtimeUs", task[5].runtimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T5_avgRuntimeUs", task[5].avgRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T5_avgLowerRuntimeUs", task[5].lowerRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T5_avgUpperRuntimeUs", task[5].upperRuntimeUs, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("T5_maxRuntimeUs", task[5].maxRuntimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T5_startTimeUs", task[5].startTimeUs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("T5_gapCount", task[5].gapCount, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("T5_doubleGapCount", task[5].doubleGapCount, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T5_reserved", task[5].reserved, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("T5_cpuUsage", task[5].cpuUsage, 0, DATA_TYPE_F32, f_t, 0);
    ADD_MAP("T5_handle", task[5].handle, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}
static void PopulateCanConfigMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(can_config_t, DID_CAN_CONFIG);
    
    ADD_MAP("can_period_mult[CID_INS_TIME]", can_period_mult[CID_INS_TIME], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_STATUS]", can_period_mult[CID_INS_STATUS], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_EULER]", can_period_mult[CID_INS_EULER], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_QUATN2B]", can_period_mult[CID_INS_QUATN2B], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_QUATE2B]", can_period_mult[CID_INS_QUATE2B], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_UVW]", can_period_mult[CID_INS_UVW], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_VE]", can_period_mult[CID_INS_VE], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_LAT]", can_period_mult[CID_INS_LAT], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_LON]", can_period_mult[CID_INS_LON], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_ALT]", can_period_mult[CID_INS_ALT], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_NORTH_EAST]", can_period_mult[CID_INS_NORTH_EAST], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_DOWN]", can_period_mult[CID_INS_DOWN], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_ECEF_X]", can_period_mult[CID_INS_ECEF_X], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_ECEF_Y]", can_period_mult[CID_INS_ECEF_Y], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_ECEF_Z]", can_period_mult[CID_INS_ECEF_Z], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_INS_MSL]", can_period_mult[CID_INS_MSL], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_PREINT_PX]", can_period_mult[CID_PREINT_PX], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_PREINT_QY]", can_period_mult[CID_PREINT_QY], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_PREINT_RZ]", can_period_mult[CID_PREINT_RZ], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_DUAL_PX]", can_period_mult[CID_DUAL_PX], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_DUAL_QY]", can_period_mult[CID_DUAL_QY], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_DUAL_RZ]", can_period_mult[CID_DUAL_RZ], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_GPS1_POS]", can_period_mult[CID_GPS1_POS], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_GPS2_POS]", can_period_mult[CID_GPS2_POS], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_GPS1_RTK_POS_REL]", can_period_mult[CID_GPS1_RTK_POS_REL], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_GPS2_RTK_CMP_REL]", can_period_mult[CID_GPS2_RTK_CMP_REL], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_period_mult[CID_ROLL_ROLLRATE]", can_period_mult[CID_ROLL_ROLLRATE], 0, DATA_TYPE_UINT16, uint16_t&, 0);
    ADD_MAP("can_transmit_address[CID_INS_TIME]", can_transmit_address[CID_INS_TIME], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_STATUS]", can_transmit_address[CID_INS_STATUS], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_EULER]", can_transmit_address[CID_INS_EULER], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_QUATN2B]", can_transmit_address[CID_INS_QUATN2B], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_QUATE2B]", can_transmit_address[CID_INS_QUATE2B], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_UVW]", can_transmit_address[CID_INS_UVW], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_VE]", can_transmit_address[CID_INS_VE], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_LAT]", can_transmit_address[CID_INS_LAT], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_LON]", can_transmit_address[CID_INS_LON], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_ALT]", can_transmit_address[CID_INS_ALT], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_NORTH_EAST]", can_transmit_address[CID_INS_NORTH_EAST], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_DOWN]", can_transmit_address[CID_INS_DOWN], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_ECEF_X]", can_transmit_address[CID_INS_ECEF_X], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_ECEF_Y]", can_transmit_address[CID_INS_ECEF_Y], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_ECEF_Z]", can_transmit_address[CID_INS_ECEF_Z], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_INS_MSL]", can_transmit_address[CID_INS_MSL], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_PREINT_PX]", can_transmit_address[CID_PREINT_PX], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_PREINT_QY]", can_transmit_address[CID_PREINT_QY], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_PREINT_RZ]", can_transmit_address[CID_PREINT_RZ], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_DUAL_PX]", can_transmit_address[CID_DUAL_PX], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_DUAL_QY]", can_transmit_address[CID_DUAL_QY], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_DUAL_RZ]", can_transmit_address[CID_DUAL_RZ], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_GPS1_POS]", can_transmit_address[CID_GPS1_POS], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_GPS2_POS]", can_transmit_address[CID_GPS2_POS], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_GPS1_RTK_POS_REL]", can_transmit_address[CID_GPS1_RTK_POS_REL], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_GPS2_RTK_CMP_REL]", can_transmit_address[CID_GPS2_RTK_CMP_REL], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("can_transmit_address[CID_ROLL_ROLLRATE]", can_transmit_address[CID_ROLL_ROLLRATE], 0, DATA_TYPE_UINT32, uint32_t&, DATA_FLAG_DISPLAY_HEX);

    ADD_MAP("can_baudrate_kbps", can_baudrate_kbps, 0, DATA_TYPE_UINT16, uint16_t, 0);
    ADD_MAP("can_receive_address", can_receive_address, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);

    ASSERT_SIZE(totalSize);
}

static void PopulateDiagMsgMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(diag_msg_t, DID_DIAGNOSTIC_MESSAGE);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("messageLength", messageLength, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("message", message, MEMBERSIZE(diag_msg_t, message), DATA_TYPE_STRING, char[MEMBERSIZE(diag_msg_t, message)], 0);

    ASSERT_SIZE(totalSize);
}

#ifdef USE_IS_INTERNAL

static void PopulateSensorsADCMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sys_sensors_adc_t, DID_SENSORS_ADC);
    
    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("pqr1[0]", imu[0].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1[1]", imu[0].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1[2]", imu[0].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[0]", imu[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[1]", imu[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[2]", imu[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("temp1",   imu[0].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr2[0]", imu[1].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2[1]", imu[1].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2[2]", imu[1].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[0]", imu[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[1]", imu[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[2]", imu[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("temp2", imu[1].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr3[0]", imu[2].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr3[1]", imu[2].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr3[2]", imu[2].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc3[0]", imu[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc3[1]", imu[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc3[2]", imu[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("temp3", imu[2].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mag1[0]", mag[0].mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1[1]", mag[0].mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1[2]", mag[0].mag[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag2[0]", mag[1].mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag2[1]", mag[1].mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag2[2]", mag[1].mag[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("bar", bar, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("barTemp", barTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("humidity", humidity, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("ana[0]", ana[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ana[1]", ana[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ana[2]", ana[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ana[3]", ana[3], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsISMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], uint32_t id)
{
    INIT_MAP(sensors_w_temp_t, id);
    
    ADD_MAP("imu3.time", imu3.time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("imu3.status", imu3.status, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pqr0[0]", imu3.I[0].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0[1]", imu3.I[0].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0[2]", imu3.I[0].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0[0]", imu3.I[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0[1]", imu3.I[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0[2]", imu3.I[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1[0]", imu3.I[1].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1[1]", imu3.I[1].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1[2]", imu3.I[1].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[0]", imu3.I[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[1]", imu3.I[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[2]", imu3.I[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2[0]", imu3.I[2].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2[1]", imu3.I[2].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2[2]", imu3.I[2].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[0]", imu3.I[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[1]", imu3.I[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[2]", imu3.I[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("temp0", temp[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("temp1", temp[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("temp2", temp[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0[0]", mag[0].xyz[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0[1]", mag[0].xyz[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0[2]", mag[0].xyz[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1[0]", mag[1].xyz[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1[1]", mag[1].xyz[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1[2]", mag[1].xyz[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsTCMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sensors_t, DID_SENSORS_TC_BIAS);
    
    ADD_MAP("time", time, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("pqr0[0]", mpu[0].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0[1]", mpu[0].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0[2]", mpu[0].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0[0]", mpu[0].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0[1]", mpu[0].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0[2]", mpu[0].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0[0]", mpu[0].mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0[1]", mpu[0].mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0[2]", mpu[0].mag[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1[0]", mpu[1].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1[1]", mpu[1].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1[2]", mpu[1].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[0]", mpu[1].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[1]", mpu[1].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1[2]", mpu[1].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1[0]", mpu[1].mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1[1]", mpu[1].mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1[2]", mpu[1].mag[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2[0]", mpu[2].pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2[1]", mpu[2].pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2[2]", mpu[2].pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[0]", mpu[2].acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[1]", mpu[2].acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2[2]", mpu[2].acc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag2[0]", mpu[2].mag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag2[1]", mpu[2].mag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag2[2]", mpu[2].mag[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsCompMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(sensor_compensation_t, DID_SCOMP);
    
    ADD_MAP("timeMs", timeMs, 0, DATA_TYPE_UINT32, uint32_t, 0);

    // Gyros
    ADD_MAP("pqr0.lpfLsb[0]", pqr[0].lpfLsb[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0.lpfLsb[1]", pqr[0].lpfLsb[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0.lpfLsb[2]", pqr[0].lpfLsb[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0.lpfTemp", pqr[0].lpfTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr0.k[0]", pqr[0].k[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0.k[1]", pqr[0].k[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0.k[2]", pqr[0].k[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr0.temp", pqr[0].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr0.tempRampRate", pqr[0].tempRampRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr0.tci", pqr[0].tci, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pqr0.numTcPts", pqr[0].numTcPts, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pqr0.dtTemp", pqr[0].dtTemp, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("pqr1.lpfLsb[0]", pqr[1].lpfLsb[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1.lpfLsb[1]", pqr[1].lpfLsb[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1.lpfLsb[2]", pqr[1].lpfLsb[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1.lpfTemp", pqr[1].lpfTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr1.k[0]", pqr[1].k[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1.k[1]", pqr[1].k[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1.k[2]", pqr[1].k[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr1.temp", pqr[1].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr1.tempRampRate", pqr[1].tempRampRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr1.tci", pqr[1].tci, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pqr1.numTcPts", pqr[1].numTcPts, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pqr1.dtTemp", pqr[1].dtTemp, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("pqr2.lpfLsb[0]", pqr[2].lpfLsb[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2.lpfLsb[1]", pqr[2].lpfLsb[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2.lpfLsb[2]", pqr[2].lpfLsb[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2.lpfTemp", pqr[2].lpfTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr2.k[0]", pqr[2].k[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2.k[1]", pqr[2].k[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2.k[2]", pqr[2].k[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("pqr2.temp", pqr[2].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr2.tempRampRate", pqr[2].tempRampRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("pqr2.tci", pqr[2].tci, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pqr2.numTcPts", pqr[2].numTcPts, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("pqr2.dtTemp", pqr[2].dtTemp, 0, DATA_TYPE_F32, float, 0);

    // Accels
    ADD_MAP("acc0.lpfLsb[0]", acc[0].lpfLsb[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0.lpfLsb[1]", acc[0].lpfLsb[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0.lpfLsb[2]", acc[0].lpfLsb[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0.lpfTemp", acc[0].lpfTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc0.k[0]", acc[0].k[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0.k[1]", acc[0].k[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0.k[2]", acc[0].k[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc0.temp", acc[0].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc0.tempRampRate", acc[0].tempRampRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc0.tci", acc[0].tci, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("acc0.numTcPts", acc[0].numTcPts, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("acc0.dtTemp", acc[0].dtTemp, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("acc1.lpfLsb[0]", acc[1].lpfLsb[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1.lpfLsb[1]", acc[1].lpfLsb[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1.lpfLsb[2]", acc[1].lpfLsb[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1.lpfTemp", acc[1].lpfTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc1.k[0]", acc[1].k[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1.k[1]", acc[1].k[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1.k[2]", acc[1].k[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc1.temp", acc[1].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc1.tempRampRate", acc[1].tempRampRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc1.tci", acc[1].tci, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("acc1.numTcPts", acc[1].numTcPts, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("acc1.dtTemp", acc[1].dtTemp, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("acc2.lpfLsb[0]", acc[2].lpfLsb[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2.lpfLsb[1]", acc[2].lpfLsb[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2.lpfLsb[2]", acc[2].lpfLsb[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2.lpfTemp", acc[2].lpfTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc2.k[0]", acc[2].k[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2.k[1]", acc[2].k[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2.k[2]", acc[2].k[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("acc2.temp", acc[2].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc2.tempRampRate", acc[2].tempRampRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("acc2.tci", acc[2].tci, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("acc2.numTcPts", acc[2].numTcPts, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("acc2.dtTemp", acc[2].dtTemp, 0, DATA_TYPE_F32, float, 0);

    // Magnetometers
    ADD_MAP("mag0.lpfLsb[0]", mag[0].lpfLsb[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0.lpfLsb[1]", mag[0].lpfLsb[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0.lpfLsb[2]", mag[0].lpfLsb[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0.lpfTemp", mag[0].lpfTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mag0.k[0]", mag[0].k[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0.k[1]", mag[0].k[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0.k[2]", mag[0].k[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag0.temp", mag[0].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mag0.tempRampRate", mag[0].tempRampRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mag0.tci", mag[0].tci, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("mag0.numTcPts", mag[0].numTcPts, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("mag0.dtTemp", mag[0].dtTemp, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("mag1.lpfLsb[0]", mag[1].lpfLsb[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1.lpfLsb[1]", mag[1].lpfLsb[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1.lpfLsb[2]", mag[1].lpfLsb[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1.lpfTemp", mag[1].lpfTemp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mag1.k[0]", mag[1].k[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1.k[1]", mag[1].k[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1.k[2]", mag[1].k[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag1.temp", mag[1].temp, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mag1.tempRampRate", mag[1].tempRampRate, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mag1.tci", mag[1].tci, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("mag1.numTcPts", mag[1].numTcPts, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("mag1.dtTemp", mag[1].dtTemp, 0, DATA_TYPE_F32, float, 0);

    // Reference IMU
    ADD_MAP("referenceImu.pqr[0]", referenceImu.pqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("referenceImu.pqr[1]", referenceImu.pqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("referenceImu.pqr[2]", referenceImu.pqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("referenceImu.acc[0]", referenceImu.acc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("referenceImu.acc[1]", referenceImu.acc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("referenceImu.acc[2]", referenceImu.acc[2], 0, DATA_TYPE_F32, float&, 0);
    // Reference Mag
    ADD_MAP("referenceMag[0]", referenceMag[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("referenceMag[1]", referenceMag[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("referenceMag[2]", referenceMag[2], 0, DATA_TYPE_F32, float&, 0);

    ADD_MAP("sampleCount", sampleCount, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("calState", calState, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("status", status, 0, DATA_TYPE_UINT32, uint32_t, DATA_FLAG_DISPLAY_HEX);
    ADD_MAP("alignAccel[0]", alignAccel[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("alignAccel[1]", alignAccel[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("alignAccel[2]", alignAccel[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateUserPage0Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(nvm_group_0_t, DID_NVR_USERPAGE_G0);
    
    ADD_MAP("size", size, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("checksum", checksum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("key", key, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("lockBits", lockBits, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("featureBits", featureBits, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("featureHash1", featureHash1, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("featureHash2", featureHash2, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateUserPage1Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(nvm_group_1_t, DID_NVR_USERPAGE_G1);
    
    ADD_MAP("size", size, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("checksum", checksum, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("key", key, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("bKpqr", cf.bKpqr, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("bKuvw", cf.bKuvw, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("oKat1", cf.oKat1, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("oKat2", cf.oKat2, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("oKuvw", cf.oKuvw, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("oKlla", cf.oKlla, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("mag.bias_cal[0]", mag.bias_cal[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.bias_cal[1]", mag.bias_cal[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.bias_cal[2]", mag.bias_cal[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[0]", mag.Wcal[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[1]", mag.Wcal[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[2]", mag.Wcal[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[3]", mag.Wcal[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[4]", mag.Wcal[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[5]", mag.Wcal[5], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[6]", mag.Wcal[6], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[7]", mag.Wcal[7], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.Wcal[8]", mag.Wcal[8], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[0]", mag.DtD[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[1]", mag.DtD[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[2]", mag.DtD[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[3]", mag.DtD[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[4]", mag.DtD[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[5]", mag.DtD[5], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[6]", mag.DtD[6], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[7]", mag.DtD[7], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[8]", mag.DtD[8], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[9]", mag.DtD[9], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[10]", mag.DtD[10], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[11]", mag.DtD[11], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[12]", mag.DtD[12], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[13]", mag.DtD[13], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[14]", mag.DtD[14], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[15]", mag.DtD[15], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[16]", mag.DtD[16], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[17]", mag.DtD[17], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[18]", mag.DtD[18], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[19]", mag.DtD[19], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[20]", mag.DtD[20], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[21]", mag.DtD[21], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[22]", mag.DtD[22], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[23]", mag.DtD[23], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[24]", mag.DtD[24], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[25]", mag.DtD[25], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[26]", mag.DtD[26], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[27]", mag.DtD[27], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[28]", mag.DtD[28], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[29]", mag.DtD[29], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[30]", mag.DtD[30], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[31]", mag.DtD[31], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[32]", mag.DtD[32], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[33]", mag.DtD[33], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[34]", mag.DtD[34], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[35]", mag.DtD[35], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[36]", mag.DtD[36], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[37]", mag.DtD[37], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[38]", mag.DtD[38], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[39]", mag.DtD[39], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[40]", mag.DtD[40], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[41]", mag.DtD[41], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[42]", mag.DtD[42], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[43]", mag.DtD[43], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[44]", mag.DtD[44], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[45]", mag.DtD[45], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[46]", mag.DtD[46], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[47]", mag.DtD[47], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[48]", mag.DtD[48], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[49]", mag.DtD[49], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[50]", mag.DtD[50], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[51]", mag.DtD[51], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[52]", mag.DtD[52], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[53]", mag.DtD[53], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[54]", mag.DtD[54], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[55]", mag.DtD[55], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[56]", mag.DtD[56], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[57]", mag.DtD[57], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[58]", mag.DtD[58], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[59]", mag.DtD[59], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[60]", mag.DtD[60], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[61]", mag.DtD[61], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[62]", mag.DtD[62], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[63]", mag.DtD[63], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[64]", mag.DtD[64], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[65]", mag.DtD[65], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[66]", mag.DtD[66], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[67]", mag.DtD[67], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[68]", mag.DtD[68], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[69]", mag.DtD[69], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[70]", mag.DtD[70], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[71]", mag.DtD[71], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[72]", mag.DtD[72], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[73]", mag.DtD[73], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[74]", mag.DtD[74], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[75]", mag.DtD[75], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[76]", mag.DtD[76], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[77]", mag.DtD[77], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[78]", mag.DtD[78], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[79]", mag.DtD[79], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[80]", mag.DtD[80], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[81]", mag.DtD[81], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[82]", mag.DtD[82], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[83]", mag.DtD[83], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[84]", mag.DtD[84], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[85]", mag.DtD[85], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[86]", mag.DtD[86], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[87]", mag.DtD[87], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[88]", mag.DtD[88], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[89]", mag.DtD[89], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[90]", mag.DtD[90], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[91]", mag.DtD[91], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[92]", mag.DtD[92], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[93]", mag.DtD[93], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[94]", mag.DtD[94], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[95]", mag.DtD[95], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[96]", mag.DtD[96], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[97]", mag.DtD[97], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[98]", mag.DtD[98], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("mag.DtD[99]", mag.DtD[99], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}
static void PopulateInl2MagObsInfo(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(inl2_mag_obs_info_t, DID_INL2_MAG_OBS_INFO);
    
    ADD_MAP("timeOfWeekMs", timeOfWeekMs, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("Ncal_samples", Ncal_samples, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("ready", ready, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("calibrated", calibrated, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("auto_recal", auto_recal, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("outlier", outlier, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("magHdg", magHdg, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("insHdg", insHdg, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("magInsHdgDelta", magInsHdgDelta, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("nis", nis, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("nis_threshold", nis_threshold, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("Wcal[0]", Wcal[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("Wcal[1]", Wcal[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("Wcal[2]", Wcal[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("Wcal[3]", Wcal[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("Wcal[4]", Wcal[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("Wcal[5]", Wcal[5], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("Wcal[6]", Wcal[6], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("Wcal[7]", Wcal[7], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("Wcal[8]", Wcal[8], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("activeCalSet", activeCalSet, 0, DATA_TYPE_UINT32, uint32_t, 0);
    ADD_MAP("magHdgOffset", magHdgOffset, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("Tcal", Tcal, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("bias_cal[0]", bias_cal[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("bias_cal[1]", bias_cal[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("bias_cal[2]", bias_cal[2], 0, DATA_TYPE_F32, float&, 0);

    ASSERT_SIZE(totalSize);
}
static void PopulateInl2StatesMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(inl2_states_t, DID_INL2_STATES);
    
    ADD_MAP("timeOfWeek", timeOfWeek, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("qe2b[0]", qe2b[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qe2b[1]", qe2b[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qe2b[2]", qe2b[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("qe2b[3]", qe2b[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ve[0]", ve[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ve[1]", ve[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ve[2]", ve[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("ecef[0]", ecef[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("ecef[1]", ecef[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("ecef[2]", ecef[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("biasPqr[0]", biasPqr[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("biasPqr[1]", biasPqr[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("biasPqr[2]", biasPqr[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("biasAcc[0]", biasAcc[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("biasAcc[1]", biasAcc[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("biasAcc[2]", biasAcc[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("biasBaro", biasBaro, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("magDec", magDec, 0, DATA_TYPE_F32, float, 0);
    ADD_MAP("magInc", magInc, 0, DATA_TYPE_F32, float, 0);

    ASSERT_SIZE(totalSize);
}

// static void PopulateRtkStateMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
// {
//     INIT_MAP(rtk_state_t MAP_TYPE;
//     map_name_to_info_t& m = mappings[DID_RTK_STATE];
//     uint32_t totalSize = 0;
//     ADD_MAP("time.time", time.time, 0, DATA_TYPE_INT64, int64_t, 0);
//     ADD_MAP("time.sec", time.sec, 0, DATA_TYPE_F64, double, 0);
//     ADD_MAP("rp[0]", rp_ecef[0], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("rp[1]", rp_ecef[1], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("rp[2]", rp_ecef[2], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("rv[0]", rv_ecef[0], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("rv[1]", rv_ecef[1], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("rv[2]", rv_ecef[2], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("ra[0]", ra_ecef[0], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("ra[1]", ra_ecef[1], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("ra[2]", ra_ecef[2], 0, DATA_TYPE_F64, double&, 0);
// 
//     ADD_MAP("bp[0]", bp_ecef[0], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("bp[1]", bp_ecef[1], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("bp[2]", bp_ecef[2], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("bv[0]", bv_ecef[0], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("bv[1]", bv_ecef[1], 0, DATA_TYPE_F64, double&, 0);
//     ADD_MAP("bv[2]", bv_ecef[2], 0, DATA_TYPE_F64, double&, 0);
// }

static void PopulateRtkResidualMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT], int DID)
{
    INIT_MAP(rtk_residual_t, DID);
    
    ADD_MAP("time.time", time.time, 0, DATA_TYPE_INT64, int64_t, 0);
    ADD_MAP("time.sec", time.sec, 0, DATA_TYPE_F64, double, 0);
    ADD_MAP("nv", nv, 0, DATA_TYPE_INT32, int32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateRtkDebugMappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rtk_debug_t, DID_RTK_DEBUG);    

    ADD_MAP("time.time", time.time, 0, DATA_TYPE_INT64, int64_t, 0);
    ADD_MAP("time.sec", time.sec, 0, DATA_TYPE_F64, double, 0);

    ADD_MAP("rej_ovfl", rej_ovfl, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("code_outlier", code_outlier, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("phase_outlier", phase_outlier, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("code_large_residual", code_large_residual, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("phase_large_residual", phase_large_residual, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("invalid_base_position", invalid_base_position, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("bad_baseline_holdamb", bad_baseline_holdamb, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("base_position_error", base_position_error, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("outc_ovfl", outc_ovfl, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("reset_timer", reset_timer, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("use_ubx_position", use_ubx_position, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("large_v2b", large_v2b, 0, DATA_TYPE_UINT8, uint8_t, 0);
    
    ADD_MAP("base_position_update", base_position_update, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("rover_position_error", rover_position_error, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("reset_bias", reset_bias, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("start_relpos", start_relpos, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("end_relpos", end_relpos, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("start_rtkpos", start_rtkpos, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("pnt_pos_error", pnt_pos_error, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("no_base_obs_data", no_base_obs_data, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("diff_age_error", diff_age_error, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("moveb_time_sync_error", moveb_time_sync_error, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("waiting_for_rover_packet", waiting_for_rover_packet, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("waiting_for_base_packet", waiting_for_base_packet, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("lsq_error", lsq_error, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("lack_of_valid_sats", lack_of_valid_sats, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("divergent_pnt_pos_iteration", divergent_pnt_pos_iteration, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("chi_square_error", chi_square_error, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("cycle_slips", cycle_slips, 0, DATA_TYPE_UINT32, uint32_t, 0);

    ADD_MAP("ubx_error", ubx_error, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("solStatus", solStatus, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("rescode_err_marker", rescode_err_marker, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("error_count", error_count, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("error_code", error_code, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("dist2base", dist2base, 0, DATA_TYPE_F32, float, 0);

    ADD_MAP("reserved1", reserved1, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("gdop_error", gdop_error, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("warning_code", warning_code, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("warning_count", warning_count, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("double_debug[0]", double_debug[0], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("double_debug[1]", double_debug[1], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("double_debug[2]", double_debug[2], 0, DATA_TYPE_F64, double&, 0);
    ADD_MAP("double_debug[3]", double_debug[3], 0, DATA_TYPE_F64, double&, 0);

    ADD_MAP("debug[0]", debug[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("debug[1]", debug[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("obs_count_bas", obs_count_bas, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_count_rov", obs_count_rov, 0, DATA_TYPE_UINT8, uint8_t, 0);

    //ADD_MAP("obs_pairs_filtered", obs_pairs_filtered, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("reserved2", reserved2, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("raw_ptr_queue_overrun", raw_ptr_queue_overrun, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("raw_dat_queue_overrun", raw_dat_queue_overrun, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_unhealthy", obs_unhealthy, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("obs_rover_avail", obs_rover_avail, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_base_avail", obs_base_avail, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_pairs_used_float", obs_pairs_used_float, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_pairs_used_ar", obs_pairs_used_ar, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("obs_eph_avail", obs_eph_avail, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_low_snr_rover", obs_low_snr_rover, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_low_snr_base", obs_low_snr_base, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_high_snr_parity", obs_high_snr_parity, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("obs_zero_L1_rover", obs_zero_L1_rover, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_zero_L1_base", obs_zero_L1_base, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_low_elev_rover", obs_low_elev_rover, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("obs_low_elev_base", obs_low_elev_base, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("eph1RxCnt", eph1RxCnt, 0, DATA_TYPE_UINT8, uint8_t, 0);
    ADD_MAP("eph2RxCnt", eph2RxCnt, 0, DATA_TYPE_UINT8, uint8_t, 0);

    ADD_MAP("reserved[0]", reserved[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("reserved[1]", reserved[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);

    ASSERT_SIZE(totalSize);
}

#if 0
static void PopulateRtkDebug2Mappings(map_name_to_info_t mappings[DID_COUNT], map_index_to_info_t indices[DID_COUNT])
{
    INIT_MAP(rtk_debug_2_t, DID_RTK_DEBUG_2);

    ADD_MAP("time.time", time.time, 0, DATA_TYPE_INT64, int64_t, 0);
    ADD_MAP("time.sec", time.sec, 0, DATA_TYPE_F64, double, 0);

#if 0    // This doesn't work in Linux

    char str[50];
    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasFloat[%d]", i, 0);
        ADD_MAP(str, satBiasFloat[i], 0, DATA_TYPE_F32, float&, 0);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasFix[%d]", i, 0);
        ADD_MAP(str, satBiasFix[i], 0, DATA_TYPE_F32, float&, 0);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "qualL[%d]", i, 0);
        ADD_MAP(str, qualL[i], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "sat[%d]", i, 0);
        ADD_MAP(str, sat[i], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasCov[%d]", i, 0);
        ADD_MAP(str, satBiasStd[i], 0, DATA_TYPE_F32, float&, 0);
    }

#else

    ADD_MAP("satBiasFloat[0]", satBiasFloat[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[1]", satBiasFloat[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[2]", satBiasFloat[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[3]", satBiasFloat[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[4]", satBiasFloat[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[5]", satBiasFloat[5], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[6]", satBiasFloat[6], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[7]", satBiasFloat[7], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[8]", satBiasFloat[8], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[9]", satBiasFloat[9], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[10]", satBiasFloat[10], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[11]", satBiasFloat[11], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[12]", satBiasFloat[12], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[13]", satBiasFloat[13], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[14]", satBiasFloat[14], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[15]", satBiasFloat[15], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[16]", satBiasFloat[16], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[17]", satBiasFloat[17], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[18]", satBiasFloat[18], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[19]", satBiasFloat[19], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[20]", satBiasFloat[20], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFloat[21]", satBiasFloat[21], 0, DATA_TYPE_F32, float&, 0);

    ADD_MAP("satBiasFix[0]", satBiasFix[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[1]", satBiasFix[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[2]", satBiasFix[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[3]", satBiasFix[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[4]", satBiasFix[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[5]", satBiasFix[5], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[6]", satBiasFix[6], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[7]", satBiasFix[7], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[8]", satBiasFix[8], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[9]", satBiasFix[9], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[10]", satBiasFix[10], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[11]", satBiasFix[11], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[12]", satBiasFix[12], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[13]", satBiasFix[13], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[14]", satBiasFix[14], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[15]", satBiasFix[15], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[16]", satBiasFix[16], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[17]", satBiasFix[17], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[18]", satBiasFix[18], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[19]", satBiasFix[19], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[20]", satBiasFix[20], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasFix[21]", satBiasFix[21], 0, DATA_TYPE_F32, float&, 0);

    ADD_MAP("qualL[0]", qualL[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[1]", qualL[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[2]", qualL[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[3]", qualL[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[4]", qualL[4], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[5]", qualL[5], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[6]", qualL[6], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[7]", qualL[7], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[8]", qualL[8], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[9]", qualL[9], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[10]", qualL[10], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[11]", qualL[11], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[12]", qualL[12], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[13]", qualL[13], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[14]", qualL[14], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[15]", qualL[15], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[16]", qualL[16], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[17]", qualL[17], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[18]", qualL[18], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[19]", qualL[19], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[20]", qualL[20], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("qualL[21]", qualL[21], 0, DATA_TYPE_UINT8, uint8_t&, 0);

    ADD_MAP("sat[0]", sat[0], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[1]", sat[1], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[2]", sat[2], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[3]", sat[3], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[4]", sat[4], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[5]", sat[5], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[6]", sat[6], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[7]", sat[7], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[8]", sat[8], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[9]", sat[9], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[10]", sat[10], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[11]", sat[11], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[12]", sat[12], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[13]", sat[13], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[14]", sat[14], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[15]", sat[15], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[16]", sat[16], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[17]", sat[17], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[18]", sat[18], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[19]", sat[19], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[20]", sat[20], 0, DATA_TYPE_UINT8, uint8_t&, 0);
    ADD_MAP("sat[21]", sat[21], 0, DATA_TYPE_UINT8, uint8_t&, 0);

    ADD_MAP("satBiasStd[0]", satBiasStd[0], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[1]", satBiasStd[1], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[2]", satBiasStd[2], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[3]", satBiasStd[3], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[4]", satBiasStd[4], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[5]", satBiasStd[5], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[6]", satBiasStd[6], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[7]", satBiasStd[7], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[8]", satBiasStd[8], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[9]", satBiasStd[9], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[10]", satBiasStd[10], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[11]", satBiasStd[11], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[12]", satBiasStd[12], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[13]", satBiasStd[13], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[14]", satBiasStd[14], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[15]", satBiasStd[15], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[16]", satBiasStd[16], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[17]", satBiasStd[17], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[18]", satBiasStd[18], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[19]", satBiasStd[19], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[20]", satBiasStd[20], 0, DATA_TYPE_F32, float&, 0);
    ADD_MAP("satBiasStd[21]", satBiasStd[21], 0, DATA_TYPE_F32, float&, 0);

    ADD_MAP("satLockCnt[0]", satLockCnt[0], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[1]", satLockCnt[1], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[2]", satLockCnt[2], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[3]", satLockCnt[3], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[4]", satLockCnt[4], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[5]", satLockCnt[5], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[6]", satLockCnt[6], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[7]", satLockCnt[7], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[8]", satLockCnt[8], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[9]", satLockCnt[9], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[10]", satLockCnt[10], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[11]", satLockCnt[11], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[12]", satLockCnt[12], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[13]", satLockCnt[13], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[14]", satLockCnt[14], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[15]", satLockCnt[15], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[16]", satLockCnt[16], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[17]", satLockCnt[17], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[18]", satLockCnt[18], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[19]", satLockCnt[19], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[20]", satLockCnt[20], 0, DATA_TYPE_INT8, int8_t&, 0);
    ADD_MAP("satLockCnt[21]", satLockCnt[21], 0, DATA_TYPE_INT8, int8_t&, 0);

#endif

    ADD_MAP("num_biases", num_biases, 0, DATA_TYPE_UINT8, uint8_t, 0);

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
    "UNUSED_98",                        // 98 
    "UNUSED_99",                        // 99 
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
    switch (dataType)
    {
    case DATA_TYPE_INT8:
        if (dataFlags == DATA_FLAG_DISPLAY_HEX)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(int8_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int8_t*)ptr);
        break;

    case DATA_TYPE_UINT8:
        if (dataFlags == DATA_FLAG_DISPLAY_HEX)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(uint8_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint8_t*)ptr);
        break;

    case DATA_TYPE_INT16:
        if (dataFlags == DATA_FLAG_DISPLAY_HEX)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(int16_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int16_t*)ptr);
        break;

    case DATA_TYPE_UINT16:
        if (dataFlags == DATA_FLAG_DISPLAY_HEX)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(uint16_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint16_t*)ptr);
        break;

    case DATA_TYPE_INT32:
        if (dataFlags == DATA_FLAG_DISPLAY_HEX)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(int32_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int32_t*)ptr);
        break;

    case DATA_TYPE_UINT32:
        if (dataFlags == DATA_FLAG_DISPLAY_HEX)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(uint32_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint32_t*)ptr);        
        break;

    case DATA_TYPE_INT64:
        if (dataFlags == DATA_FLAG_DISPLAY_HEX)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (long long)*(uint64_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%lld", (long long)*(int64_t*)ptr);
        break;

    case DATA_TYPE_UINT64:
        if (dataFlags == DATA_FLAG_DISPLAY_HEX)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (unsigned long long)*(uint64_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%llu", (unsigned long long)*(uint64_t*)ptr);
        break;

    case DATA_TYPE_F32:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.9g", *(float*)ptr);
        break;

    case DATA_TYPE_F64:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.17g", *(double*)ptr);
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
