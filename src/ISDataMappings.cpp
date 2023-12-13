/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

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
#include "../../cpp/libs/IS_internal.h"
#endif

using namespace std;

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

CONST_EXPRESSION uint32_t s_eDataTypeSizes[DataTypeCount] =
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

inline uint32_t GetDataTypeSize(eDataType dataType)
{
    if (dataType >= 0 && dataType < DataTypeCount)
    {
        return s_eDataTypeSizes[dataType];
    }
    return 0;
}

#if CPP11_IS_ENABLED

// dataSize can be 0 for default size, must be set for string type
#define ADD_MAP_NO_VALIDATION(map, totalSize, name, member, dataSize, dataType, memberType, dataFlags) (map)[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize), dataType, (eDataFlags)dataFlags, name }; totalSize += sizeof(memberType);

// note when passing member type for arrays, it must be a reference, i.e. float&
#define ADD_MAP(map, totalSize, name, member, dataSize, dataType, memberType, dataFlags) \
    ADD_MAP_NO_VALIDATION(map, totalSize, name, member, dataSize, dataType, memberType, dataFlags); \
    static_assert(is_same<decltype(MAP_TYPE::member), memberType>::value, "Member type is an unexpected type"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == sizeof(memberType), "Member type is an unexpected size, sizeof(memberType)"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == sizeof(MAP_TYPE::member), "Member type is an unexpected size, sizeof(MAP_TYPE::member)"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ASSERT_SIZE(s) assert(s == sizeof(MAP_TYPE))

#else

#define ADD_MAP_NO_VALIDATION(map, totalSize, name, member, dataSize, dataType, memberType, dataFlags) (map)[std::string(name)] = { (uint32_t)offsetof(MAP_TYPE, member), (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize), dataType, (eDataFlags)dataFlags, name }; totalSize += sizeof(memberType);
#define ADD_MAP(map, totalSize, name, member, dataSize, dataType, memberType, dataFlags) ADD_MAP_NO_VALIDATION(map, totalSize, name, member, dataSize, dataType, memberType, dataFlags)
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
    sizeMap[DID_MANUFACTURING_INFO] = sizeof(manufacturing_info_t);
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
    sizeMap[DID_GPS1_UBX_POS] = sizeof(gps_pos_t);
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

static void PopulateDeviceInfoMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef dev_info_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "reserved", reserved, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "hardware", hardware, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "serialNumber", serialNumber, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "hardwareVer[0]", hardwareVer[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "hardwareVer[1]", hardwareVer[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "hardwareVer[2]", hardwareVer[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "hardwareVer[3]", hardwareVer[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "firmwareVer[0]", firmwareVer[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "firmwareVer[1]", firmwareVer[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "firmwareVer[2]", firmwareVer[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "firmwareVer[3]", firmwareVer[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "buildNumber", buildNumber, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "protocolVer[0]", protocolVer[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "protocolVer[1]", protocolVer[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "protocolVer[2]", protocolVer[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "protocolVer[3]", protocolVer[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "repoRevision", repoRevision, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "manufacturer", manufacturer, DEVINFO_MANUFACTURER_STRLEN, DataTypeString, char[DEVINFO_MANUFACTURER_STRLEN], 0);
    ADD_MAP(m, totalSize, "buildDate[0]", buildDate[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "buildDate[1]", buildDate[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "buildDate[2]", buildDate[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "buildDate[3]", buildDate[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "buildTime[0]", buildTime[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "buildTime[1]", buildTime[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "buildTime[2]", buildTime[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "buildTime[3]", buildTime[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "addInfo", addInfo, DEVINFO_ADDINFO_STRLEN, DataTypeString, char[DEVINFO_ADDINFO_STRLEN], 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateManufacturingInfoMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef manufacturing_info_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_MANUFACTURING_INFO];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "serialNumber", serialNumber, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "lotNumber", lotNumber, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "date", date, 16, DataTypeString, char[16], 0);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "platformType", platformType, 0, DataTypeInt32, int32_t, 0);
    ADD_MAP(m, totalSize, "uid[0]", uid[0], 0, DataTypeUInt32, uint32_t&, 0);
    ADD_MAP(m, totalSize, "uid[1]", uid[1], 0, DataTypeUInt32, uint32_t&, 0);
    ADD_MAP(m, totalSize, "uid[2]", uid[2], 0, DataTypeUInt32, uint32_t&, 0);
    ADD_MAP(m, totalSize, "uid[3]", uid[3], 0, DataTypeUInt32, uint32_t&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateIOMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef io_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_IO];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "gpioStatus", gpioStatus, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateBitMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef bit_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_BIT];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "state", state, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "hdwBitStatus", hdwBitStatus, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "calBitStatus", calBitStatus, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "tcPqrBias", tcPqrBias, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "tcAccBias", tcAccBias, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "tcPqrSlope", tcPqrSlope, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "tcAccSlope", tcAccSlope, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "tcPqrLinearity", tcPqrLinearity, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "tcAccLinearity", tcAccLinearity, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "pqr", pqr, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc", acc, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqrSigma", pqrSigma, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "accSigma", accSigma, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "testMode", testMode, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysFaultMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef system_fault_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_SYS_FAULT];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "g1Task", g1Task, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "g2FileNum", g2FileNum, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "g3LineNum", g3LineNum, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "g4", g4, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "g5Lr", g5Lr, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pc", pc, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "psr", psr, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t dataId)
{
    typedef imu_t MAP_TYPE;
    map_name_to_info_t& m = mappings[dataId];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "pqr[0]", I.pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr[1]", I.pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr[2]", I.pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[0]", I.acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[1]", I.acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[2]", I.acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMU3Mappings(map_name_to_info_t mappings[DID_COUNT], uint32_t dataId)
{
    typedef imu3_t MAP_TYPE;
    map_name_to_info_t& m = mappings[dataId];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "I0.pqr[0]", I[0].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I0.pqr[1]", I[0].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I0.pqr[2]", I[0].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I0.acc[0]", I[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I0.acc[1]", I[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I0.acc[2]", I[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I1.pqr[0]", I[1].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I1.pqr[1]", I[1].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I1.pqr[2]", I[1].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I1.acc[0]", I[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I1.acc[1]", I[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I1.acc[2]", I[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I2.pqr[0]", I[2].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I2.pqr[1]", I[2].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I2.pqr[2]", I[2].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I2.acc[0]", I[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I2.acc[1]", I[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I2.acc[2]", I[2].acc[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysParamsMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef sys_params_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_SYS_PARAMS];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "imuTemp", imuTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "baroTemp", baroTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mcuTemp", mcuTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "sysStatus", sysStatus, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "imuSamplePeriodMs", imuSamplePeriodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "navOutputPeriodMs", navOutputPeriodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "sensorTruePeriod", sensorTruePeriod, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "flashCfgChecksum", flashCfgChecksum, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "navUpdatePeriodMs", navUpdatePeriodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "genFaultCode", genFaultCode, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);

    ASSERT_SIZE(totalSize);
}

static void PopulateSysSensorsMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef sys_sensors_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_SYS_SENSORS];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "temp", temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr[0]", pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr[1]", pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr[2]", pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[0]", acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[1]", acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[2]", acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[0]", mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[1]", mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[2]", mag[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "bar", bar, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "barTemp", barTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mslBar", mslBar, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "humidity", humidity, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vin", vin, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "ana1", ana1, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "ana3", ana1, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "ana4", ana1, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateRMCMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef rmc_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_RMC];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "bits", bits, 0, DataTypeUInt64, uint64_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "options", options, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS1Mappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef ins_1_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_INS_1];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "theta[0]", theta[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "theta[1]", theta[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "theta[2]", theta[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[0]", uvw[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[1]", uvw[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[2]", uvw[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "lla[0]", lla[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[1]", lla[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[2]", lla[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "ned[0]", ned[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ned[1]", ned[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ned[2]", ned[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS2Mappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef ins_2_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_INS_2];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "qn2b[0]", qn2b[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qn2b[1]", qn2b[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qn2b[2]", qn2b[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qn2b[3]", qn2b[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[0]", uvw[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[1]", uvw[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[2]", uvw[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "lla[0]", lla[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[1]", lla[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[2]", lla[2], 0, DataTypeDouble, double&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS3Mappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef ins_3_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_INS_3];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "qn2b[0]", qn2b[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qn2b[1]", qn2b[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qn2b[2]", qn2b[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qn2b[3]", qn2b[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[0]", uvw[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[1]", uvw[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "uvw[2]", uvw[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "lla[0]", lla[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[1]", lla[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[2]", lla[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "msl", msl, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateINS4Mappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef ins_4_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_INS_4];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "insStatus", insStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "hdwStatus", hdwStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "qe2b[0]", qe2b[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qe2b[1]", qe2b[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qe2b[2]", qe2b[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qe2b[3]", qe2b[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ve[0]", ve[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ve[1]", ve[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ve[2]", ve[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ecef[0]", ecef[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "ecef[1]", ecef[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "ecef[2]", ecef[2], 0, DataTypeDouble, double&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsPosMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef gps_pos_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "ecef[0]", ecef[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "ecef[1]", ecef[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "ecef[2]", ecef[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[0]", lla[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[1]", lla[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lla[2]", lla[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "hMSL", hMSL, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "hAcc", hAcc, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vAcc", vAcc, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pDop", pDop, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "cnoMean", cnoMean, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "towOffset", towOffset, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "leapS", leapS, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "satsUsed", satsUsed, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "cnoMeanSigma", cnoMeanSigma, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "reserved", reserved, 0, DataTypeUInt8, uint8_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsVelMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef gps_vel_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "vel[0]", vel[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "vel[1]", vel[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "vel[2]", vel[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "sAcc", sAcc, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsSatMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef gps_sat_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "numSats", numSats, 0, DataTypeUInt32, uint32_t, 0);

#define ADD_MAP_SAT_INFO(n) \
    ADD_MAP(m, totalSize, "sat" #n ".gnssId",    sat[n].gnssId,    0, DataTypeUInt8, uint8_t, 0); \
    ADD_MAP(m, totalSize, "sat" #n ".svId",      sat[n].svId,      0, DataTypeUInt8, uint8_t, 0); \
    ADD_MAP(m, totalSize, "sat" #n ".elev",      sat[n].elev,      0, DataTypeInt8,  int8_t,  0); \
    ADD_MAP(m, totalSize, "sat" #n ".azim",      sat[n].azim,      0, DataTypeInt16, int16_t, 0); \
    ADD_MAP(m, totalSize, "sat" #n ".cno",       sat[n].cno,       0, DataTypeUInt8, uint8_t, 0); \
    ADD_MAP(m, totalSize, "sat" #n ".status",    sat[n].status,    0, DataTypeUInt16, uint16_t, 0);

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

static void PopulateGpsSigMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef gps_sig_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "numSigs", numSigs, 0, DataTypeUInt32, uint32_t, 0);

#define ADD_MAP_SAT_SIG(n) \
    ADD_MAP(m, totalSize, "sig" #n ".gnssId",    sig[n].gnssId,    0, DataTypeUInt8, uint8_t, 0); \
    ADD_MAP(m, totalSize, "sig" #n ".svId",      sig[n].svId,      0, DataTypeUInt8, uint8_t, 0); \
    ADD_MAP(m, totalSize, "sig" #n ".sigId",     sig[n].sigId,     0, DataTypeUInt8, uint8_t, 0); \
    ADD_MAP(m, totalSize, "sig" #n ".cno",       sig[n].cno,       0, DataTypeUInt8, uint8_t, 0); \
    ADD_MAP(m, totalSize, "sig" #n ".quality",   sig[n].quality,   0, DataTypeUInt8, uint8_t, 0); \
    ADD_MAP(m, totalSize, "sig" #n ".status",    sig[n].status,    0, DataTypeUInt16, uint16_t, 0);

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

static void PopulateMagnetometerMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t did)
{
    typedef magnetometer_t MAP_TYPE;
    map_name_to_info_t& m = mappings[did];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "mag[0]", mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[1]", mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[2]", mag[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateBarometerMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef barometer_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_BAROMETER];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "bar", bar, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mslBar", mslBar, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "barTemp", barTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "humidity", humidity, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t did)
{
    typedef pimu_t MAP_TYPE;
    map_name_to_info_t& m = mappings[did];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "theta[0]", theta[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "theta[1]", theta[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "theta[2]", theta[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "vel[0]", vel[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "vel[1]", vel[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "vel[2]", vel[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "dt", dt, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUDeltaThetaVelocityMagMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef pimu_mag_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_PIMU_MAG];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "imutime", pimu.time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "theta[0]", pimu.theta[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "theta[1]", pimu.theta[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "theta[2]", pimu.theta[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "vel[0]", pimu.vel[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "vel[1]", pimu.vel[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "vel[2]", pimu.vel[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "dt", pimu.dt, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "imustatus", pimu.status, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "magtime", mag.time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "mag[0]", mag.mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[1]", mag.mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[2]", mag.mag[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateIMUMagnetometerMappings(map_name_to_info_t mappings [DID_COUNT])
{

    typedef imu_mag_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_IMU_MAG];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "imutime", imu.time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "pqr[0]", imu.I.pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr[1]", imu.I.pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr[2]", imu.I.pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[0]", imu.I.acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[1]", imu.I.acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc[2]", imu.I.acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imustatus", imu.status, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "magtime", mag.time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "mag[0]", mag.mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[1]", mag.mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag[2]", mag.mag[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);

}


static void PopulateInfieldCalMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef infield_cal_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_INFIELD_CAL];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "state", state, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "sampleTimeMs", sampleTimeMs, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "imu[0].pqr[0]", imu[0].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[0].pqr[1]", imu[0].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[0].pqr[2]", imu[0].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[0].acc[0]", imu[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[0].acc[1]", imu[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[0].acc[2]", imu[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[1].pqr[0]", imu[1].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[1].pqr[1]", imu[1].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[1].pqr[2]", imu[1].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[1].acc[0]", imu[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[1].acc[1]", imu[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[1].acc[2]", imu[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[2].pqr[0]", imu[2].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[2].pqr[1]", imu[2].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[2].pqr[2]", imu[2].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[2].acc[0]", imu[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[2].acc[1]", imu[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "imu[2].acc[2]", imu[2].acc[2], 0, DataTypeFloat, float&, 0);

    ADD_MAP(m, totalSize, "calData[0].down.dev[0].acc[0]", calData[0].down.dev[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.dev[0].acc[1]", calData[0].down.dev[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.dev[0].acc[2]", calData[0].down.dev[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.dev[1].acc[0]", calData[0].down.dev[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.dev[1].acc[1]", calData[0].down.dev[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.dev[1].acc[2]", calData[0].down.dev[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.dev[2].acc[0]", calData[0].down.dev[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.dev[2].acc[1]", calData[0].down.dev[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.dev[2].acc[2]", calData[0].down.dev[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].down.yaw", calData[0].down.yaw, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[0].acc[0]", calData[0].up.dev[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[0].acc[1]", calData[0].up.dev[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[0].acc[2]", calData[0].up.dev[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[1].acc[0]", calData[0].up.dev[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[1].acc[1]", calData[0].up.dev[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[1].acc[2]", calData[0].up.dev[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[2].acc[0]", calData[0].up.dev[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[2].acc[1]", calData[0].up.dev[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.dev[2].acc[2]", calData[0].up.dev[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[0].up.yaw", calData[0].up.yaw, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "calData[1].down.dev[0].acc[0]", calData[1].down.dev[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.dev[0].acc[1]", calData[1].down.dev[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.dev[0].acc[2]", calData[1].down.dev[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.dev[1].acc[0]", calData[1].down.dev[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.dev[1].acc[1]", calData[1].down.dev[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.dev[1].acc[2]", calData[1].down.dev[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.dev[2].acc[0]", calData[1].down.dev[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.dev[2].acc[1]", calData[1].down.dev[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.dev[2].acc[2]", calData[1].down.dev[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].down.yaw", calData[1].down.yaw, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[0].acc[0]", calData[1].up.dev[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[0].acc[1]", calData[1].up.dev[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[0].acc[2]", calData[1].up.dev[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[1].acc[0]", calData[1].up.dev[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[1].acc[1]", calData[1].up.dev[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[1].acc[2]", calData[1].up.dev[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[2].acc[0]", calData[1].up.dev[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[2].acc[1]", calData[1].up.dev[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.dev[2].acc[2]", calData[1].up.dev[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[1].up.yaw", calData[1].up.yaw, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "calData[2].down.dev[0].acc[0]", calData[2].down.dev[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.dev[0].acc[1]", calData[2].down.dev[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.dev[0].acc[2]", calData[2].down.dev[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.dev[1].acc[0]", calData[2].down.dev[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.dev[1].acc[1]", calData[2].down.dev[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.dev[1].acc[2]", calData[2].down.dev[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.dev[2].acc[0]", calData[2].down.dev[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.dev[2].acc[1]", calData[2].down.dev[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.dev[2].acc[2]", calData[2].down.dev[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].down.yaw", calData[2].down.yaw, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[0].acc[0]", calData[2].up.dev[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[0].acc[1]", calData[2].up.dev[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[0].acc[2]", calData[2].up.dev[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[1].acc[0]", calData[2].up.dev[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[1].acc[1]", calData[2].up.dev[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[1].acc[2]", calData[2].up.dev[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[2].acc[0]", calData[2].up.dev[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[2].acc[1]", calData[2].up.dev[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.dev[2].acc[2]", calData[2].up.dev[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "calData[2].up.yaw", calData[2].up.yaw, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateReferenceIMUMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef imu_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_REFERENCE_IMU];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "I.pqr[0]", I.pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I.pqr[1]", I.pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I.pqr[2]", I.pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I.acc[0]", I.acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I.acc[1]", I.acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "I.acc[2]", I.acc[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateWheelEncoderMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef wheel_encoder_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_WHEEL_ENCODER];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "theta_l", theta_l, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "omega_l", omega_l, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "theta_r", theta_r, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "omega_r", omega_r, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wrap_count_l", wrap_count_l, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "wrap_count_r", wrap_count_r, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGroundVehicleMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef ground_vehicle_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_GROUND_VEHICLE];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "mode", mode, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "wheelConfig.bits", wheelConfig.bits, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.track_width", wheelConfig.track_width, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheelConfig.radius", wheelConfig.radius, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateConfigMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef system_command_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_SYS_CMD];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "command", command, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "invCommand", invCommand, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateFlashConfigMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef nvm_flash_cfg_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_FLASH_CONFIG];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", size, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "checksum", checksum, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "startupImuDtMs", startupImuDtMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "startupNavDtMs", startupNavDtMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "ser0BaudRate", ser0BaudRate, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "ser1BaudRate", ser1BaudRate, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "insRotation[0]", insRotation[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "insRotation[1]", insRotation[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "insRotation[2]", insRotation[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "insOffset[0]", insOffset[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "insOffset[1]", insOffset[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "insOffset[2]", insOffset[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "gps1AntOffset[0]", gps1AntOffset[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "gps1AntOffset[1]", gps1AntOffset[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "gps1AntOffset[2]", gps1AntOffset[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "insDynModel", insDynModel, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "debug", debug, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "gnssSatSigConst", gnssSatSigConst, 0, DataTypeUInt16, uint16_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "sysCfgBits", sysCfgBits, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "refLla[0]", refLla[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "refLla[1]", refLla[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "refLla[2]", refLla[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lastLla[0]", lastLla[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lastLla[1]", lastLla[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lastLla[2]", lastLla[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lastLlaTimeOfWeekMs", lastLlaTimeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "lastLlaWeek", lastLlaWeek, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "lastLlaUpdateDistance", lastLlaUpdateDistance, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "ioConfig", ioConfig, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "platformConfig", platformConfig, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "gpsTimeUserDelay", gpsTimeUserDelay, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "magDeclination", magDeclination, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "gps2AntOffset[0]", gps2AntOffset[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "gps2AntOffset[1]", gps2AntOffset[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "gps2AntOffset[2]", gps2AntOffset[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "zeroVelRotation[0]", zeroVelRotation[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "zeroVelRotation[1]", zeroVelRotation[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "zeroVelRotation[2]", zeroVelRotation[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "zeroVelOffset[0]", zeroVelOffset[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "zeroVelOffset[1]", zeroVelOffset[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "zeroVelOffset[2]", zeroVelOffset[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "gpsTimeSyncPeriodMs", gpsTimeSyncPeriodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "startupGPSDtMs", startupGPSDtMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "RTKCfgBits", RTKCfgBits, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "sensorConfig", sensorConfig, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "gpsMinimumElevation", gpsMinimumElevation, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "ser2BaudRate", ser2BaudRate, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "wheelConfig.bits", wheelConfig.bits, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w[0]", wheelConfig.transform.e_b2w[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w[1]", wheelConfig.transform.e_b2w[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w[2]", wheelConfig.transform.e_b2w[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w_sigma[0]", wheelConfig.transform.e_b2w_sigma[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w_sigma[1]", wheelConfig.transform.e_b2w_sigma[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.e_b2w_sigma[2]", wheelConfig.transform.e_b2w_sigma[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w[0]", wheelConfig.transform.t_b2w[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w[1]", wheelConfig.transform.t_b2w[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w[2]", wheelConfig.transform.t_b2w[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w_sigma[0]", wheelConfig.transform.t_b2w_sigma[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w_sigma[1]", wheelConfig.transform.t_b2w_sigma[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.transform.t_b2w_sigma[2]", wheelConfig.transform.t_b2w_sigma[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "wheelConfig.track_width", wheelConfig.track_width, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheelConfig.radius", wheelConfig.radius, 0, DataTypeFloat, float, 0);
	ADD_MAP(m, totalSize, "magInterferenceThreshold", magInterferenceThreshold, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbStatusMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef evb_status_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_EVB_STATUS];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "firmwareVer[0]", firmwareVer[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "firmwareVer[1]", firmwareVer[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "firmwareVer[2]", firmwareVer[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "firmwareVer[3]", firmwareVer[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "evbStatus", evbStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "loggerMode", loggerMode, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "loggerElapsedTimeMs", loggerElapsedTimeMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "wifiIpAddr", wifiIpAddr, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "sysCommand", sysCommand, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "towOffset", towOffset, 0, DataTypeDouble, double, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef evb_flash_cfg_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_EVB_FLASH_CFG];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", size, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "checksum", checksum, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "cbPreset", cbPreset, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "reserved1[0]", reserved1[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "reserved1[1]", reserved1[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "reserved1[2]", reserved1[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "cbf[0]", cbf[0], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[1]", cbf[1], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[2]", cbf[2], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[3]", cbf[3], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[4]", cbf[4], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[5]", cbf[5], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[6]", cbf[6], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[7]", cbf[7], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[8]", cbf[8], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbf[9]", cbf[9], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "cbOptions", cbOptions, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "bits", bits, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "radioPID", radioPID, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "radioNID", radioNID, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "radioPowerLevel", radioPowerLevel, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "wifi[0].ssid", wifi[0].ssid, WIFI_SSID_PSK_SIZE, DataTypeString, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP(m, totalSize, "wifi[0].psk", wifi[0].psk, WIFI_SSID_PSK_SIZE, DataTypeString, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP(m, totalSize, "wifi[1].ssid", wifi[1].ssid, WIFI_SSID_PSK_SIZE, DataTypeString, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP(m, totalSize, "wifi[1].psk", wifi[1].psk, WIFI_SSID_PSK_SIZE, DataTypeString, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP(m, totalSize, "wifi[2].ssid", wifi[2].ssid, WIFI_SSID_PSK_SIZE, DataTypeString, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP(m, totalSize, "wifi[2].psk", wifi[2].psk, WIFI_SSID_PSK_SIZE, DataTypeString, char[WIFI_SSID_PSK_SIZE], 0);
    ADD_MAP(m, totalSize, "server[0].ipAddr[0]", server[0].ipAddr.u8[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[0].ipAddr[1]", server[0].ipAddr.u8[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[0].ipAddr[2]", server[0].ipAddr.u8[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[0].ipAddr[3]", server[0].ipAddr.u8[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[0].port", server[0].port, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "server[1].ipAddr[0]", server[1].ipAddr.u8[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[1].ipAddr[1]", server[1].ipAddr.u8[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[1].ipAddr[2]", server[1].ipAddr.u8[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[1].ipAddr[3]", server[1].ipAddr.u8[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[1].port", server[1].port, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "server[2].ipAddr[0]", server[2].ipAddr.u8[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[2].ipAddr[1]", server[2].ipAddr.u8[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[2].ipAddr[2]", server[2].ipAddr.u8[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[2].ipAddr[3]", server[2].ipAddr.u8[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "server[2].port", server[2].port, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "encoderTickToWheelRad", encoderTickToWheelRad, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "CANbaud_kbps", CANbaud_kbps, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "can_receive_address", can_receive_address, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "uinsComPort", uinsComPort, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "uinsAuxPort", uinsAuxPort, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "reserved2[0]", reserved2[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "reserved2[1]", reserved2[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "portOptions", portOptions, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "h3sp330BaudRate", h3sp330BaudRate, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "h4xRadioBaudRate", h4xRadioBaudRate, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "h8gpioBaudRate", h8gpioBaudRate, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "wheelCfgBits", wheelCfgBits, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "velocityControlPeriodMs", velocityControlPeriodMs, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateDebugArrayMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef debug_array_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "i[0]", i[0], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "i[1]", i[1], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "i[2]", i[2], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "i[3]", i[3], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "i[4]", i[4], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "i[5]", i[5], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "i[6]", i[6], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "i[7]", i[7], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "i[8]", i[8], 0, DataTypeInt32, int32_t&, 0);
    ADD_MAP(m, totalSize, "f[0]", f[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "f[1]", f[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "f[2]", f[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "f[3]", f[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "f[4]", f[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "f[5]", f[5], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "f[6]", f[6], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "f[7]", f[7], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "f[8]", f[8], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "lf[0]", lf[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lf[1]", lf[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "lf[2]", lf[2], 0, DataTypeDouble, double&, 0);

    ASSERT_SIZE(totalSize);
}

#if defined(INCLUDE_LUNA_DATA_SETS)

static void PopulateEvbLunaFlashCfgMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef evb_luna_flash_cfg_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_EVB_LUNA_FLASH_CFG];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", size, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "checksum", checksum, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "bits", bits, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "minLatGeofence", minLatGeofence, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "maxLatGeofence", maxLatGeofence, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "minLonGeofence", minLonGeofence, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "maxLonGeofence", maxLonGeofence, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "remoteKillTimeoutMs", remoteKillTimeoutMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "bumpSensitivity", bumpSensitivity, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "minProxDistance", minProxDistance, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.config",                  velControl.config, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "velControl.cmdTimeoutMs",            velControl.cmdTimeoutMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "velControl.wheelRadius",             velControl.wheelRadius, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheelBaseline",           velControl.wheelBaseline, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.engine_rpm",              velControl.engine_rpm, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "velControl.vehicle.u_min",           velControl.vehicle.u_min, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.u_cruise",        velControl.vehicle.u_cruise, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.u_max",           velControl.vehicle.u_max, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.u_slewLimit",     velControl.vehicle.u_slewLimit, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.w_max_autonomous",velControl.vehicle.w_max_autonomous, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.w_max",           velControl.vehicle.w_max, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.w_slewLimit",     velControl.vehicle.w_slewLimit, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.u_FB_Kp",         velControl.vehicle.u_FB_Kp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.w_FB_Kp",         velControl.vehicle.w_FB_Kp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.w_FB_Ki",         velControl.vehicle.w_FB_Ki, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.w_FF_c0",         velControl.vehicle.w_FF_c0, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.w_FF_c1",         velControl.vehicle.w_FF_c1, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.w_FF_deadband",   velControl.vehicle.w_FF_deadband, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.vehicle.testSweepRate",   velControl.vehicle.testSweepRate, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "velControl.wheel.slewRate",          velControl.wheel.slewRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.velMax",            velControl.wheel.velMax, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_vel_deadband",   velControl.wheel.FF_vel_deadband, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_c_est_Ki[0]",    velControl.wheel.FF_c_est_Ki[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_c_est_Ki[1]",    velControl.wheel.FF_c_est_Ki[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_c_est_max[0]",   velControl.wheel.FF_c_est_max[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_c_est_max[1]",   velControl.wheel.FF_c_est_max[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_c_l[0]",         velControl.wheel.FF_c_l[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_c_l[1]",         velControl.wheel.FF_c_l[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_c_r[0]",         velControl.wheel.FF_c_r[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_c_r[1]",         velControl.wheel.FF_c_r[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FF_FB_engine_rpm",  velControl.wheel.FF_FB_engine_rpm, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FB_Kp",             velControl.wheel.FB_Kp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FB_Ki",             velControl.wheel.FB_Ki, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FB_Kd",             velControl.wheel.FB_Kd, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FB_gain_deadband",  velControl.wheel.FB_gain_deadband, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.FB_gain_deadband_reduction",  velControl.wheel.FB_gain_deadband_reduction, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_l[0]", velControl.wheel.InversePlant_l[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_l[1]", velControl.wheel.InversePlant_l[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_l[2]", velControl.wheel.InversePlant_l[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_l[3]", velControl.wheel.InversePlant_l[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_l[4]", velControl.wheel.InversePlant_l[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_r[0]", velControl.wheel.InversePlant_r[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_r[1]", velControl.wheel.InversePlant_r[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_r[2]", velControl.wheel.InversePlant_r[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_r[3]", velControl.wheel.InversePlant_r[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.InversePlant_r[4]", velControl.wheel.InversePlant_r[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorTrim_l",    velControl.wheel.actuatorTrim_l, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorTrim_r",    velControl.wheel.actuatorTrim_r, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorLimits_l[0]", velControl.wheel.actuatorLimits_l[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorLimits_l[1]", velControl.wheel.actuatorLimits_l[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorLimits_r[0]", velControl.wheel.actuatorLimits_r[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorLimits_r[1]", velControl.wheel.actuatorLimits_r[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorDeadbandDuty_l", velControl.wheel.actuatorDeadbandDuty_l, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorDeadbandDuty_r", velControl.wheel.actuatorDeadbandDuty_r, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "velControl.wheel.actuatorDeadbandVel", velControl.wheel.actuatorDeadbandVel, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateCoyoteStatusMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef evb_luna_status_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_EVB_LUNA_STATUS];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "evbLunaStatus", evbLunaStatus, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "motorState", motorState, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "remoteKillMode", remoteKillMode, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "supplyVoltage", supplyVoltage, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaSensorsMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef evb_luna_sensors_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_EVB_LUNA_SENSORS];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[0]", proxSensorOutput[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[1]", proxSensorOutput[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[2]", proxSensorOutput[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[3]", proxSensorOutput[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[4]", proxSensorOutput[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[5]", proxSensorOutput[5], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[6]", proxSensorOutput[6], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[7]", proxSensorOutput[7], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "proxSensorOutput[8]", proxSensorOutput[8], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "bumpEvent", bumpEvent, 0, DataTypeInt32, int32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaVelocityControlMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef evb_luna_velocity_control_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_EVB_LUNA_VELOCITY_CONTROL];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeMs", timeMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "dt", dt, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "current_mode", current_mode, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "vehicle.velCmd_f", vehicle.velCmd_f, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.velCmd_w", vehicle.velCmd_w, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.velCmdMnl_f", vehicle.velCmdMnl_f, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.velCmdMnl_w", vehicle.velCmdMnl_w, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.velCmdSlew_f", vehicle.velCmdSlew_f, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.velCmdSlew_w", vehicle.velCmdSlew_w, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.vel_f", vehicle.vel_f, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.vel_w", vehicle.vel_w, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.err_f", vehicle.err_f, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.err_w", vehicle.err_w, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.eff_f", vehicle.eff_f, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vehicle.eff_w", vehicle.eff_w, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "wheel_l.velCmd",             wheel_l.velCmd, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.velCmdSlew",         wheel_l.velCmdSlew, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.vel",                wheel_l.vel, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.err",                wheel_l.err, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.ff_eff",             wheel_l.ff_eff, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.fb_eff",             wheel_l.fb_eff, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.fb_eff_integral",    wheel_l.fb_eff_integral, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.eff",                wheel_l.eff, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.effInt",             wheel_l.effInt, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_l.effDuty",            wheel_l.effDuty, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "wheel_r.velCmd",             wheel_r.velCmd, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.velCmdSlew",         wheel_r.velCmdSlew, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.vel",                wheel_r.vel, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.err",                wheel_r.err, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.ff_eff",             wheel_r.ff_eff, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.fb_eff",             wheel_r.fb_eff, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.fb_eff_integral",    wheel_r.fb_eff_integral, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.eff",                wheel_r.eff, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.effInt",             wheel_r.effInt, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "wheel_r.effDuty",            wheel_r.effDuty, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "potV_l", potV_l, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "potV_r", potV_r, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaVelocityCommandMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef evb_luna_velocity_command_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_EVB_LUNA_VELOCITY_COMMAND];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeMs", timeMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "modeCmd", modeCmd, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "fwd_vel", fwd_vel, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "turn_rate", turn_rate, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateEvbLunaAuxCmdMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef evb_luna_aux_command_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_EVB_LUNA_AUX_COMMAND];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "command", command, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

#endif

static void PopulateGpsRtkRelMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef gps_rtk_rel_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "differentialAge", differentialAge, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "arRatio", arRatio, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "baseToRoverVector[0]", baseToRoverVector[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "baseToRoverVector[1]", baseToRoverVector[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "baseToRoverVector[2]", baseToRoverVector[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "baseToRoverDistance", baseToRoverDistance, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "baseToRoverHeading", baseToRoverHeading, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "baseToRoverHeadingAcc", baseToRoverHeadingAcc, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRtkMiscMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef gps_rtk_misc_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "accuracyPos[0]", accuracyPos[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "accuracyPos[1]", accuracyPos[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "accuracyPos[2]", accuracyPos[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "accuracyCov[0]", accuracyCov[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "accuracyCov[1]", accuracyCov[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "accuracyCov[2]", accuracyCov[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "arThreshold", arThreshold, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "gDop", gDop, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "hDop", hDop, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "vDop", vDop, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "baseLla[0]", baseLla[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "baseLla[1]", baseLla[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "baseLla[2]", baseLla[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "cycleSlipCount", cycleSlipCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverGpsObservationCount", roverGpsObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseGpsObservationCount", baseGpsObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverGlonassObservationCount", roverGlonassObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseGlonassObservationCount", baseGlonassObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverGalileoObservationCount", roverGalileoObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseGalileoObservationCount", baseGalileoObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverBeidouObservationCount", roverBeidouObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseBeidouObservationCount", baseBeidouObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverQzsObservationCount", roverQzsObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseQzsObservationCount", baseQzsObservationCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverGpsEphemerisCount", roverGpsEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseGpsEphemerisCount", baseGpsEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverGlonassEphemerisCount", roverGlonassEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseGlonassEphemerisCount", baseGlonassEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverGalileoEphemerisCount", roverGalileoEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseGalileoEphemerisCount", baseGalileoEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverBeidouEphemerisCount", roverBeidouEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseBeidouEphemerisCount", baseBeidouEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverQzsEphemerisCount", roverQzsEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseQzsEphemerisCount", baseQzsEphemerisCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "roverSbasCount", roverSbasCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseSbasCount", baseSbasCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "baseAntennaCount", baseAntennaCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "ionUtcAlmCount", ionUtcAlmCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "correctionChecksumFailures", correctionChecksumFailures, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "timeToFirstFixMs", timeToFirstFixMs, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateGpsRawMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef gps_raw_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "receiveIndex", receiverIndex, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "dataType", dataType, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "obsCount", obsCount, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "reserved", reserved, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "dataBuf", data.buf, 0, DataTypeBinary, uint8_t[MEMBERSIZE(MAP_TYPE, data.buf)], 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateStrobeInTimeMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef strobe_in_time_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_STROBE_IN_TIME];
    uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "week", week, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pin", pin, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "count", count, 0, DataTypeUInt16, uint16_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateRtosInfoMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef rtos_info_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_RTOS_INFO];
    uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "freeHeapSize", freeHeapSize, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "mallocSize", mallocSize, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "freeSize", freeSize, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "T0_name", task[0].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP(m, totalSize, "T0_priority", task[0].priority, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T0_stackUnused", task[0].stackUnused, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T0_periodMs", task[0].periodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T0_runtimeUs", task[0].runtimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T0_avgRuntimeUs", task[0].avgRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T0_avgLowerRuntimeUs", task[0].lowerRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T0_avgUpperRuntimeUs", task[0].upperRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T0_maxRuntimeUs", task[0].maxRuntimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T0_startTimeUs", task[0].startTimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T0_gapCount", task[0].gapCount, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "T0_doubleGapCount", task[0].doubleGapCount, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T0_reserved", task[0].reserved, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T0_cpuUsage", task[0].cpuUsage, 0, DataTypeFloat, f_t, 0);
    ADD_MAP(m, totalSize, "T0_handle", task[0].handle, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "T1_name", task[1].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP(m, totalSize, "T1_priority", task[1].priority, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T1_stackUnused", task[1].stackUnused, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T1_periodMs", task[1].periodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T1_runtimeUs", task[1].runtimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T1_avgRuntimeUs", task[1].avgRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T1_avgLowerRuntimeUs", task[1].lowerRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T1_avgUpperRuntimeUs", task[1].upperRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T1_maxRuntimeUs", task[1].maxRuntimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T1_startTimeUs", task[1].startTimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T1_gapCount", task[1].gapCount, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "T1_doubleGapCount", task[1].doubleGapCount, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T1_reserved", task[1].reserved, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T1_cpuUsage", task[1].cpuUsage, 0, DataTypeFloat, f_t, 0);
    ADD_MAP(m, totalSize, "T1_handle", task[1].handle, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "T2_name", task[2].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP(m, totalSize, "T2_priority", task[2].priority, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T2_stackUnused", task[2].stackUnused, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T2_periodMs", task[2].periodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T2_runtimeUs", task[2].runtimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T2_avgRuntimeUs", task[2].avgRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T2_avgLowerRuntimeUs", task[2].lowerRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T2_avgUpperRuntimeUs", task[2].upperRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T2_maxRuntimeUs", task[2].maxRuntimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T2_startTimeUs", task[2].startTimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T2_gapCount", task[2].gapCount, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "T2_doubleGapCount", task[2].doubleGapCount, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T2_reserved", task[2].reserved, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T2_cpuUsage", task[2].cpuUsage, 0, DataTypeFloat, f_t, 0);
    ADD_MAP(m, totalSize, "T2_handle", task[2].handle, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "T3_name", task[3].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP(m, totalSize, "T3_priority", task[3].priority, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T3_stackUnused", task[3].stackUnused, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T3_periodMs", task[3].periodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T3_runtimeUs", task[3].runtimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T3_avgRuntimeUs", task[3].avgRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T3_avgLowerRuntimeUs", task[3].lowerRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T3_avgUpperRuntimeUs", task[3].upperRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T3_maxRuntimeUs", task[3].maxRuntimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T3_startTimeUs", task[3].startTimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T3_gapCount", task[3].gapCount, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "T3_doubleGapCount", task[3].doubleGapCount, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T3_reserved", task[3].reserved, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T3_cpuUsage", task[3].cpuUsage, 0, DataTypeFloat, f_t, 0);
    ADD_MAP(m, totalSize, "T3_handle", task[3].handle, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "T4_name", task[4].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP(m, totalSize, "T4_priority", task[4].priority, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T4_stackUnused", task[4].stackUnused, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T4_periodMs", task[4].periodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T4_runtimeUs", task[4].runtimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T4_avgRuntimeUs", task[4].avgRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T4_avgLowerRuntimeUs", task[4].lowerRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T4_avgUpperRuntimeUs", task[4].upperRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T4_maxRuntimeUs", task[4].maxRuntimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T4_startTimeUs", task[4].startTimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T4_gapCount", task[4].gapCount, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "T4_doubleGapCount", task[4].doubleGapCount, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T4_reserved", task[4].reserved, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T4_cpuUsage", task[4].cpuUsage, 0, DataTypeFloat, f_t, 0);
    ADD_MAP(m, totalSize, "T4_handle", task[4].handle, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "T5_name", task[5].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN], 0);
    ADD_MAP(m, totalSize, "T5_priority", task[5].priority, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T5_stackUnused", task[5].stackUnused, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T5_periodMs", task[5].periodMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T5_runtimeUs", task[5].runtimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T5_avgRuntimeUs", task[5].avgRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T5_avgLowerRuntimeUs", task[5].lowerRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T5_avgUpperRuntimeUs", task[5].upperRuntimeUs, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "T5_maxRuntimeUs", task[5].maxRuntimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T5_startTimeUs", task[5].startTimeUs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "T5_gapCount", task[5].gapCount, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "T5_doubleGapCount", task[5].doubleGapCount, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T5_reserved", task[5].reserved, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "T5_cpuUsage", task[5].cpuUsage, 0, DataTypeFloat, f_t, 0);
    ADD_MAP(m, totalSize, "T5_handle", task[5].handle, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}
static void PopulateCanConfigMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef can_config_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_CAN_CONFIG];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_TIME]", can_period_mult[CID_INS_TIME], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_STATUS]", can_period_mult[CID_INS_STATUS], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_EULER]", can_period_mult[CID_INS_EULER], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_QUATN2B]", can_period_mult[CID_INS_QUATN2B], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_QUATE2B]", can_period_mult[CID_INS_QUATE2B], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_UVW]", can_period_mult[CID_INS_UVW], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_VE]", can_period_mult[CID_INS_VE], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_LAT]", can_period_mult[CID_INS_LAT], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_LON]", can_period_mult[CID_INS_LON], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_ALT]", can_period_mult[CID_INS_ALT], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_NORTH_EAST]", can_period_mult[CID_INS_NORTH_EAST], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_DOWN]", can_period_mult[CID_INS_DOWN], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_ECEF_X]", can_period_mult[CID_INS_ECEF_X], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_ECEF_Y]", can_period_mult[CID_INS_ECEF_Y], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_ECEF_Z]", can_period_mult[CID_INS_ECEF_Z], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_INS_MSL]", can_period_mult[CID_INS_MSL], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_PREINT_PX]", can_period_mult[CID_PREINT_PX], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_PREINT_QY]", can_period_mult[CID_PREINT_QY], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_PREINT_RZ]", can_period_mult[CID_PREINT_RZ], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_DUAL_PX]", can_period_mult[CID_DUAL_PX], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_DUAL_QY]", can_period_mult[CID_DUAL_QY], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_DUAL_RZ]", can_period_mult[CID_DUAL_RZ], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_GPS1_POS]", can_period_mult[CID_GPS1_POS], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_GPS2_POS]", can_period_mult[CID_GPS2_POS], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_GPS1_RTK_POS_REL]", can_period_mult[CID_GPS1_RTK_POS_REL], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_GPS2_RTK_CMP_REL]", can_period_mult[CID_GPS2_RTK_CMP_REL], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_period_mult[CID_ROLL_ROLLRATE]", can_period_mult[CID_ROLL_ROLLRATE], 0, DataTypeUInt16, uint16_t&, 0);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_TIME]", can_transmit_address[CID_INS_TIME], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_STATUS]", can_transmit_address[CID_INS_STATUS], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_EULER]", can_transmit_address[CID_INS_EULER], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_QUATN2B]", can_transmit_address[CID_INS_QUATN2B], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_QUATE2B]", can_transmit_address[CID_INS_QUATE2B], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_UVW]", can_transmit_address[CID_INS_UVW], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_VE]", can_transmit_address[CID_INS_VE], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_LAT]", can_transmit_address[CID_INS_LAT], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_LON]", can_transmit_address[CID_INS_LON], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_ALT]", can_transmit_address[CID_INS_ALT], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_NORTH_EAST]", can_transmit_address[CID_INS_NORTH_EAST], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_DOWN]", can_transmit_address[CID_INS_DOWN], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_ECEF_X]", can_transmit_address[CID_INS_ECEF_X], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_ECEF_Y]", can_transmit_address[CID_INS_ECEF_Y], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_ECEF_Z]", can_transmit_address[CID_INS_ECEF_Z], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_INS_MSL]", can_transmit_address[CID_INS_MSL], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_PREINT_PX]", can_transmit_address[CID_PREINT_PX], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_PREINT_QY]", can_transmit_address[CID_PREINT_QY], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_PREINT_RZ]", can_transmit_address[CID_PREINT_RZ], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_DUAL_PX]", can_transmit_address[CID_DUAL_PX], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_DUAL_QY]", can_transmit_address[CID_DUAL_QY], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_DUAL_RZ]", can_transmit_address[CID_DUAL_RZ], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_GPS1_POS]", can_transmit_address[CID_GPS1_POS], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_GPS2_POS]", can_transmit_address[CID_GPS2_POS], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_GPS1_RTK_POS_REL]", can_transmit_address[CID_GPS1_RTK_POS_REL], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_GPS2_RTK_CMP_REL]", can_transmit_address[CID_GPS2_RTK_CMP_REL], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "can_transmit_address[CID_ROLL_ROLLRATE]", can_transmit_address[CID_ROLL_ROLLRATE], 0, DataTypeUInt32, uint32_t&, DataFlagsDisplayHex);

    ADD_MAP(m, totalSize, "can_baudrate_kbps", can_baudrate_kbps, 0, DataTypeUInt16, uint16_t, 0);
    ADD_MAP(m, totalSize, "can_receive_address", can_receive_address, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);

    ASSERT_SIZE(totalSize);
}

static void PopulateDiagMsgMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef diag_msg_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_DIAGNOSTIC_MESSAGE];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "messageLength", messageLength, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "message", message, MEMBERSIZE(diag_msg_t, message), DataTypeString, char[MEMBERSIZE(diag_msg_t, message)], 0);

    ASSERT_SIZE(totalSize);
}

#ifdef USE_IS_INTERNAL

static void PopulateSensorsADCMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef sys_sensors_adc_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_SENSORS_ADC];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "pqr1[0]", imu[0].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1[1]", imu[0].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1[2]", imu[0].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[0]", imu[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[1]", imu[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[2]", imu[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "temp1",   imu[0].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr2[0]", imu[1].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2[1]", imu[1].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2[2]", imu[1].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[0]", imu[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[1]", imu[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[2]", imu[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "temp2", imu[1].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr3[0]", imu[2].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr3[1]", imu[2].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr3[2]", imu[2].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc3[0]", imu[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc3[1]", imu[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc3[2]", imu[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "temp3", imu[2].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mag1[0]", mag[0].mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1[1]", mag[0].mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1[2]", mag[0].mag[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag2[0]", mag[1].mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag2[1]", mag[1].mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag2[2]", mag[1].mag[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "bar", bar, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "barTemp", barTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "humidity", humidity, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "ana[0]", ana[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ana[1]", ana[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ana[2]", ana[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ana[3]", ana[3], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsISMappings(map_name_to_info_t mappings[DID_COUNT], uint32_t id)
{
    typedef sensors_w_temp_t MAP_TYPE;
    map_name_to_info_t& m = mappings[id];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "imu3.time", imu3.time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "imu3.status", imu3.status, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pqr0[0]", imu3.I[0].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0[1]", imu3.I[0].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0[2]", imu3.I[0].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0[0]", imu3.I[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0[1]", imu3.I[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0[2]", imu3.I[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1[0]", imu3.I[1].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1[1]", imu3.I[1].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1[2]", imu3.I[1].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[0]", imu3.I[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[1]", imu3.I[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[2]", imu3.I[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2[0]", imu3.I[2].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2[1]", imu3.I[2].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2[2]", imu3.I[2].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[0]", imu3.I[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[1]", imu3.I[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[2]", imu3.I[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "temp0", temp[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "temp1", temp[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "temp2", temp[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0[0]", mag[0].xyz[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0[1]", mag[0].xyz[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0[2]", mag[0].xyz[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1[0]", mag[1].xyz[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1[1]", mag[1].xyz[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1[2]", mag[1].xyz[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsTCMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef sensors_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_SENSORS_TC_BIAS];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", time, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "pqr0[0]", mpu[0].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0[1]", mpu[0].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0[2]", mpu[0].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0[0]", mpu[0].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0[1]", mpu[0].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0[2]", mpu[0].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0[0]", mpu[0].mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0[1]", mpu[0].mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0[2]", mpu[0].mag[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1[0]", mpu[1].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1[1]", mpu[1].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1[2]", mpu[1].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[0]", mpu[1].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[1]", mpu[1].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1[2]", mpu[1].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1[0]", mpu[1].mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1[1]", mpu[1].mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1[2]", mpu[1].mag[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2[0]", mpu[2].pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2[1]", mpu[2].pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2[2]", mpu[2].pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[0]", mpu[2].acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[1]", mpu[2].acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2[2]", mpu[2].acc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag2[0]", mpu[2].mag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag2[1]", mpu[2].mag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag2[2]", mpu[2].mag[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateSensorsCompMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef sensor_compensation_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_SCOMP];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeMs", timeMs, 0, DataTypeUInt32, uint32_t, 0);

    // Gyros
    ADD_MAP(m, totalSize, "pqr0.lpfLsb[0]", pqr[0].lpfLsb[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0.lpfLsb[1]", pqr[0].lpfLsb[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0.lpfLsb[2]", pqr[0].lpfLsb[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0.lpfTemp", pqr[0].lpfTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr0.k[0]", pqr[0].k[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0.k[1]", pqr[0].k[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0.k[2]", pqr[0].k[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr0.temp", pqr[0].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr0.tempRampRate", pqr[0].tempRampRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr0.tci", pqr[0].tci, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pqr0.numTcPts", pqr[0].numTcPts, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pqr0.dtTemp", pqr[0].dtTemp, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "pqr1.lpfLsb[0]", pqr[1].lpfLsb[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1.lpfLsb[1]", pqr[1].lpfLsb[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1.lpfLsb[2]", pqr[1].lpfLsb[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1.lpfTemp", pqr[1].lpfTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr1.k[0]", pqr[1].k[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1.k[1]", pqr[1].k[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1.k[2]", pqr[1].k[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr1.temp", pqr[1].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr1.tempRampRate", pqr[1].tempRampRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr1.tci", pqr[1].tci, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pqr1.numTcPts", pqr[1].numTcPts, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pqr1.dtTemp", pqr[1].dtTemp, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "pqr2.lpfLsb[0]", pqr[2].lpfLsb[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2.lpfLsb[1]", pqr[2].lpfLsb[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2.lpfLsb[2]", pqr[2].lpfLsb[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2.lpfTemp", pqr[2].lpfTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr2.k[0]", pqr[2].k[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2.k[1]", pqr[2].k[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2.k[2]", pqr[2].k[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "pqr2.temp", pqr[2].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr2.tempRampRate", pqr[2].tempRampRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "pqr2.tci", pqr[2].tci, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pqr2.numTcPts", pqr[2].numTcPts, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "pqr2.dtTemp", pqr[2].dtTemp, 0, DataTypeFloat, float, 0);

    // Accels
    ADD_MAP(m, totalSize, "acc0.lpfLsb[0]", acc[0].lpfLsb[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0.lpfLsb[1]", acc[0].lpfLsb[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0.lpfLsb[2]", acc[0].lpfLsb[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0.lpfTemp", acc[0].lpfTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc0.k[0]", acc[0].k[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0.k[1]", acc[0].k[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0.k[2]", acc[0].k[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc0.temp", acc[0].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc0.tempRampRate", acc[0].tempRampRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc0.tci", acc[0].tci, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "acc0.numTcPts", acc[0].numTcPts, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "acc0.dtTemp", acc[0].dtTemp, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "acc1.lpfLsb[0]", acc[1].lpfLsb[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1.lpfLsb[1]", acc[1].lpfLsb[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1.lpfLsb[2]", acc[1].lpfLsb[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1.lpfTemp", acc[1].lpfTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc1.k[0]", acc[1].k[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1.k[1]", acc[1].k[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1.k[2]", acc[1].k[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc1.temp", acc[1].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc1.tempRampRate", acc[1].tempRampRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc1.tci", acc[1].tci, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "acc1.numTcPts", acc[1].numTcPts, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "acc1.dtTemp", acc[1].dtTemp, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "acc2.lpfLsb[0]", acc[2].lpfLsb[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2.lpfLsb[1]", acc[2].lpfLsb[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2.lpfLsb[2]", acc[2].lpfLsb[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2.lpfTemp", acc[2].lpfTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc2.k[0]", acc[2].k[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2.k[1]", acc[2].k[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2.k[2]", acc[2].k[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "acc2.temp", acc[2].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc2.tempRampRate", acc[2].tempRampRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "acc2.tci", acc[2].tci, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "acc2.numTcPts", acc[2].numTcPts, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "acc2.dtTemp", acc[2].dtTemp, 0, DataTypeFloat, float, 0);

    // Magnetometers
    ADD_MAP(m, totalSize, "mag0.lpfLsb[0]", mag[0].lpfLsb[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0.lpfLsb[1]", mag[0].lpfLsb[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0.lpfLsb[2]", mag[0].lpfLsb[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0.lpfTemp", mag[0].lpfTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mag0.k[0]", mag[0].k[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0.k[1]", mag[0].k[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0.k[2]", mag[0].k[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag0.temp", mag[0].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mag0.tempRampRate", mag[0].tempRampRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mag0.tci", mag[0].tci, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "mag0.numTcPts", mag[0].numTcPts, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "mag0.dtTemp", mag[0].dtTemp, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "mag1.lpfLsb[0]", mag[1].lpfLsb[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1.lpfLsb[1]", mag[1].lpfLsb[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1.lpfLsb[2]", mag[1].lpfLsb[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1.lpfTemp", mag[1].lpfTemp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mag1.k[0]", mag[1].k[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1.k[1]", mag[1].k[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1.k[2]", mag[1].k[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag1.temp", mag[1].temp, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mag1.tempRampRate", mag[1].tempRampRate, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mag1.tci", mag[1].tci, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "mag1.numTcPts", mag[1].numTcPts, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "mag1.dtTemp", mag[1].dtTemp, 0, DataTypeFloat, float, 0);

    // Reference IMU
    ADD_MAP(m, totalSize, "referenceImu.pqr[0]", referenceImu.pqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "referenceImu.pqr[1]", referenceImu.pqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "referenceImu.pqr[2]", referenceImu.pqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "referenceImu.acc[0]", referenceImu.acc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "referenceImu.acc[1]", referenceImu.acc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "referenceImu.acc[2]", referenceImu.acc[2], 0, DataTypeFloat, float&, 0);
    // Reference Mag
    ADD_MAP(m, totalSize, "referenceMag[0]", referenceMag[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "referenceMag[1]", referenceMag[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "referenceMag[2]", referenceMag[2], 0, DataTypeFloat, float&, 0);

    ADD_MAP(m, totalSize, "sampleCount", sampleCount, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "calState", calState, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "status", status, 0, DataTypeUInt32, uint32_t, DataFlagsDisplayHex);
    ADD_MAP(m, totalSize, "alignAccel[0]", alignAccel[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "alignAccel[1]", alignAccel[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "alignAccel[2]", alignAccel[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateUserPage0Mappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef nvm_group_0_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_NVR_USERPAGE_G0];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", size, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "checksum", checksum, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "lockBits", lockBits, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "featureBits", featureBits, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "featureHash1", featureHash1, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "featureHash2", featureHash2, 0, DataTypeUInt32, uint32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateUserPage1Mappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef nvm_group_1_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_NVR_USERPAGE_G1];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", size, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "checksum", checksum, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "key", key, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "bKpqr", cf.bKpqr, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "bKuvw", cf.bKuvw, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "oKat1", cf.oKat1, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "oKat2", cf.oKat2, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "oKuvw", cf.oKuvw, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "oKlla", cf.oKlla, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "mag.bias_cal[0]", mag.bias_cal[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.bias_cal[1]", mag.bias_cal[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.bias_cal[2]", mag.bias_cal[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[0]", mag.Wcal[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[1]", mag.Wcal[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[2]", mag.Wcal[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[3]", mag.Wcal[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[4]", mag.Wcal[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[5]", mag.Wcal[5], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[6]", mag.Wcal[6], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[7]", mag.Wcal[7], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.Wcal[8]", mag.Wcal[8], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[0]", mag.DtD[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[1]", mag.DtD[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[2]", mag.DtD[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[3]", mag.DtD[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[4]", mag.DtD[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[5]", mag.DtD[5], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[6]", mag.DtD[6], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[7]", mag.DtD[7], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[8]", mag.DtD[8], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[9]", mag.DtD[9], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[10]", mag.DtD[10], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[11]", mag.DtD[11], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[12]", mag.DtD[12], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[13]", mag.DtD[13], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[14]", mag.DtD[14], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[15]", mag.DtD[15], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[16]", mag.DtD[16], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[17]", mag.DtD[17], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[18]", mag.DtD[18], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[19]", mag.DtD[19], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[20]", mag.DtD[20], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[21]", mag.DtD[21], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[22]", mag.DtD[22], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[23]", mag.DtD[23], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[24]", mag.DtD[24], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[25]", mag.DtD[25], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[26]", mag.DtD[26], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[27]", mag.DtD[27], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[28]", mag.DtD[28], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[29]", mag.DtD[29], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[30]", mag.DtD[30], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[31]", mag.DtD[31], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[32]", mag.DtD[32], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[33]", mag.DtD[33], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[34]", mag.DtD[34], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[35]", mag.DtD[35], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[36]", mag.DtD[36], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[37]", mag.DtD[37], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[38]", mag.DtD[38], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[39]", mag.DtD[39], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[40]", mag.DtD[40], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[41]", mag.DtD[41], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[42]", mag.DtD[42], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[43]", mag.DtD[43], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[44]", mag.DtD[44], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[45]", mag.DtD[45], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[46]", mag.DtD[46], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[47]", mag.DtD[47], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[48]", mag.DtD[48], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[49]", mag.DtD[49], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[50]", mag.DtD[50], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[51]", mag.DtD[51], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[52]", mag.DtD[52], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[53]", mag.DtD[53], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[54]", mag.DtD[54], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[55]", mag.DtD[55], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[56]", mag.DtD[56], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[57]", mag.DtD[57], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[58]", mag.DtD[58], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[59]", mag.DtD[59], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[60]", mag.DtD[60], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[61]", mag.DtD[61], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[62]", mag.DtD[62], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[63]", mag.DtD[63], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[64]", mag.DtD[64], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[65]", mag.DtD[65], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[66]", mag.DtD[66], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[67]", mag.DtD[67], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[68]", mag.DtD[68], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[69]", mag.DtD[69], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[70]", mag.DtD[70], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[71]", mag.DtD[71], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[72]", mag.DtD[72], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[73]", mag.DtD[73], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[74]", mag.DtD[74], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[75]", mag.DtD[75], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[76]", mag.DtD[76], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[77]", mag.DtD[77], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[78]", mag.DtD[78], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[79]", mag.DtD[79], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[80]", mag.DtD[80], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[81]", mag.DtD[81], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[82]", mag.DtD[82], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[83]", mag.DtD[83], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[84]", mag.DtD[84], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[85]", mag.DtD[85], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[86]", mag.DtD[86], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[87]", mag.DtD[87], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[88]", mag.DtD[88], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[89]", mag.DtD[89], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[90]", mag.DtD[90], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[91]", mag.DtD[91], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[92]", mag.DtD[92], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[93]", mag.DtD[93], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[94]", mag.DtD[94], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[95]", mag.DtD[95], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[96]", mag.DtD[96], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[97]", mag.DtD[97], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[98]", mag.DtD[98], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "mag.DtD[99]", mag.DtD[99], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}
static void PopulateInl2MagObsInfo(map_name_to_info_t mappings[DID_COUNT])
{
    typedef inl2_mag_obs_info_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_INL2_MAG_OBS_INFO];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", timeOfWeekMs, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "Ncal_samples", Ncal_samples, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "ready", ready, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "calibrated", calibrated, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "auto_recal", auto_recal, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "outlier", outlier, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "magHdg", magHdg, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "insHdg", insHdg, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "magInsHdgDelta", magInsHdgDelta, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "nis", nis, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "nis_threshold", nis_threshold, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "Wcal[0]", Wcal[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "Wcal[1]", Wcal[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "Wcal[2]", Wcal[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "Wcal[3]", Wcal[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "Wcal[4]", Wcal[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "Wcal[5]", Wcal[5], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "Wcal[6]", Wcal[6], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "Wcal[7]", Wcal[7], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "Wcal[8]", Wcal[8], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "activeCalSet", activeCalSet, 0, DataTypeUInt32, uint32_t, 0);
    ADD_MAP(m, totalSize, "magHdgOffset", magHdgOffset, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "Tcal", Tcal, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "bias_cal[0]", bias_cal[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "bias_cal[1]", bias_cal[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "bias_cal[2]", bias_cal[2], 0, DataTypeFloat, float&, 0);

    ASSERT_SIZE(totalSize);
}
static void PopulateInl2StatesMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef inl2_states_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_INL2_STATES];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeek", timeOfWeek, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "qe2b[0]", qe2b[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qe2b[1]", qe2b[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qe2b[2]", qe2b[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "qe2b[3]", qe2b[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ve[0]", ve[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ve[1]", ve[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ve[2]", ve[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "ecef[0]", ecef[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "ecef[1]", ecef[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "ecef[2]", ecef[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "biasPqr[0]", biasPqr[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "biasPqr[1]", biasPqr[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "biasPqr[2]", biasPqr[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "biasAcc[0]", biasAcc[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "biasAcc[1]", biasAcc[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "biasAcc[2]", biasAcc[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "biasBaro", biasBaro, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "magDec", magDec, 0, DataTypeFloat, float, 0);
    ADD_MAP(m, totalSize, "magInc", magInc, 0, DataTypeFloat, float, 0);

    ASSERT_SIZE(totalSize);
}

// static void PopulateRtkStateMappings(map_name_to_info_t mappings[DID_COUNT])
// {
//     typedef rtk_state_t MAP_TYPE;
//     map_name_to_info_t& m = mappings[DID_RTK_STATE];
//     uint32_t totalSize = 0;
//     ADD_MAP(m, totalSize, "time.time", time.time, 0, DataTypeInt64, int64_t, 0);
//     ADD_MAP(m, totalSize, "time.sec", time.sec, 0, DataTypeDouble, double, 0);
//     ADD_MAP(m, totalSize, "rp[0]", rp_ecef[0], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "rp[1]", rp_ecef[1], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "rp[2]", rp_ecef[2], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "rv[0]", rv_ecef[0], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "rv[1]", rv_ecef[1], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "rv[2]", rv_ecef[2], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "ra[0]", ra_ecef[0], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "ra[1]", ra_ecef[1], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "ra[2]", ra_ecef[2], 0, DataTypeDouble, double&, 0);
// 
//     ADD_MAP(m, totalSize, "bp[0]", bp_ecef[0], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "bp[1]", bp_ecef[1], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "bp[2]", bp_ecef[2], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "bv[0]", bv_ecef[0], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "bv[1]", bv_ecef[1], 0, DataTypeDouble, double&, 0);
//     ADD_MAP(m, totalSize, "bv[2]", bv_ecef[2], 0, DataTypeDouble, double&, 0);
// }

static void PopulateRtkResidualMappings(map_name_to_info_t mappings[DID_COUNT], int DID)
{
    typedef rtk_residual_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID];
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time.time", time.time, 0, DataTypeInt64, int64_t, 0);
    ADD_MAP(m, totalSize, "time.sec", time.sec, 0, DataTypeDouble, double, 0);
    ADD_MAP(m, totalSize, "nv", nv, 0, DataTypeInt32, int32_t, 0);

    ASSERT_SIZE(totalSize);
}

static void PopulateRtkDebugMappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef rtk_debug_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_RTK_DEBUG];
    uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "time.time", time.time, 0, DataTypeInt64, int64_t, 0);
    ADD_MAP(m, totalSize, "time.sec", time.sec, 0, DataTypeDouble, double, 0);

    ADD_MAP(m, totalSize, "rej_ovfl", rej_ovfl, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "code_outlier", code_outlier, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "phase_outlier", phase_outlier, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "code_large_residual", code_large_residual, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "phase_large_residual", phase_large_residual, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "invalid_base_position", invalid_base_position, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "bad_baseline_holdamb", bad_baseline_holdamb, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "base_position_error", base_position_error, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "outc_ovfl", outc_ovfl, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "reset_timer", reset_timer, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "use_ubx_position", use_ubx_position, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "large_v2b", large_v2b, 0, DataTypeUInt8, uint8_t, 0);
    
    ADD_MAP(m, totalSize, "base_position_update", base_position_update, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "rover_position_error", rover_position_error, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "reset_bias", reset_bias, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "start_relpos", start_relpos, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "end_relpos", end_relpos, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "start_rtkpos", start_rtkpos, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "pnt_pos_error", pnt_pos_error, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "no_base_obs_data", no_base_obs_data, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "diff_age_error", diff_age_error, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "moveb_time_sync_error", moveb_time_sync_error, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "waiting_for_rover_packet", waiting_for_rover_packet, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "waiting_for_base_packet", waiting_for_base_packet, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "lsq_error", lsq_error, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "lack_of_valid_sats", lack_of_valid_sats, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "divergent_pnt_pos_iteration", divergent_pnt_pos_iteration, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "chi_square_error", chi_square_error, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "cycle_slips", cycle_slips, 0, DataTypeUInt32, uint32_t, 0);

    ADD_MAP(m, totalSize, "ubx_error", ubx_error, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "solStatus", solStatus, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "rescode_err_marker", rescode_err_marker, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "error_count", error_count, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "error_code", error_code, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "dist2base", dist2base, 0, DataTypeFloat, float, 0);

    ADD_MAP(m, totalSize, "reserved1", reserved1, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "gdop_error", gdop_error, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "warning_code", warning_code, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "warning_count", warning_count, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "double_debug[0]", double_debug[0], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "double_debug[1]", double_debug[1], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "double_debug[2]", double_debug[2], 0, DataTypeDouble, double&, 0);
    ADD_MAP(m, totalSize, "double_debug[3]", double_debug[3], 0, DataTypeDouble, double&, 0);

    ADD_MAP(m, totalSize, "debug[0]", debug[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "debug[1]", debug[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "obs_count_bas", obs_count_bas, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "obs_count_rov", obs_count_rov, 0, DataTypeUInt8, uint8_t, 0);

    ADD_MAP(m, totalSize, "obs_pairs_filtered", obs_pairs_filtered, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "obs_pairs_used", obs_pairs_used, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "raw_ptr_queue_overrun", raw_ptr_queue_overrun, 0, DataTypeUInt8, uint8_t, 0);
    ADD_MAP(m, totalSize, "raw_dat_queue_overrun", raw_dat_queue_overrun, 0, DataTypeUInt8, uint8_t, 0);

    ASSERT_SIZE(totalSize);
}

#if 0
static void PopulateRtkDebug2Mappings(map_name_to_info_t mappings[DID_COUNT])
{
    typedef rtk_debug_2_t MAP_TYPE;
    map_name_to_info_t& m = mappings[DID_RTK_DEBUG_2];
    uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "time.time", time.time, 0, DataTypeInt64, int64_t, 0);
    ADD_MAP(m, totalSize, "time.sec", time.sec, 0, DataTypeDouble, double, 0);

#if 0    // This doesn't work in Linux

    char str[50];
    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasFloat[%d]", i, 0);
        ADD_MAP(m, totalSize, str, satBiasFloat[i], 0, DataTypeFloat, float&, 0);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasFix[%d]", i, 0);
        ADD_MAP(m, totalSize, str, satBiasFix[i], 0, DataTypeFloat, float&, 0);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "qualL[%d]", i, 0);
        ADD_MAP(m, totalSize, str, qualL[i], 0, DataTypeUInt8, uint8_t&, 0);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "sat[%d]", i, 0);
        ADD_MAP(m, totalSize, str, sat[i], 0, DataTypeUInt8, uint8_t&, 0);
    }

    for (int i = 0; i < NUMSATSOL; i++)
    {
        SNPRINTF(str, sizeof(str), "satBiasCov[%d]", i, 0);
        ADD_MAP(m, totalSize, str, satBiasStd[i], 0, DataTypeFloat, float&, 0);
    }

#else

    ADD_MAP(m, totalSize, "satBiasFloat[0]", satBiasFloat[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[1]", satBiasFloat[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[2]", satBiasFloat[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[3]", satBiasFloat[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[4]", satBiasFloat[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[5]", satBiasFloat[5], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[6]", satBiasFloat[6], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[7]", satBiasFloat[7], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[8]", satBiasFloat[8], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[9]", satBiasFloat[9], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[10]", satBiasFloat[10], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[11]", satBiasFloat[11], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[12]", satBiasFloat[12], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[13]", satBiasFloat[13], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[14]", satBiasFloat[14], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[15]", satBiasFloat[15], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[16]", satBiasFloat[16], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[17]", satBiasFloat[17], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[18]", satBiasFloat[18], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[19]", satBiasFloat[19], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[20]", satBiasFloat[20], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFloat[21]", satBiasFloat[21], 0, DataTypeFloat, float&, 0);

    ADD_MAP(m, totalSize, "satBiasFix[0]", satBiasFix[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[1]", satBiasFix[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[2]", satBiasFix[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[3]", satBiasFix[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[4]", satBiasFix[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[5]", satBiasFix[5], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[6]", satBiasFix[6], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[7]", satBiasFix[7], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[8]", satBiasFix[8], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[9]", satBiasFix[9], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[10]", satBiasFix[10], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[11]", satBiasFix[11], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[12]", satBiasFix[12], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[13]", satBiasFix[13], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[14]", satBiasFix[14], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[15]", satBiasFix[15], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[16]", satBiasFix[16], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[17]", satBiasFix[17], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[18]", satBiasFix[18], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[19]", satBiasFix[19], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[20]", satBiasFix[20], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasFix[21]", satBiasFix[21], 0, DataTypeFloat, float&, 0);

    ADD_MAP(m, totalSize, "qualL[0]", qualL[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[1]", qualL[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[2]", qualL[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[3]", qualL[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[4]", qualL[4], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[5]", qualL[5], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[6]", qualL[6], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[7]", qualL[7], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[8]", qualL[8], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[9]", qualL[9], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[10]", qualL[10], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[11]", qualL[11], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[12]", qualL[12], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[13]", qualL[13], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[14]", qualL[14], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[15]", qualL[15], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[16]", qualL[16], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[17]", qualL[17], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[18]", qualL[18], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[19]", qualL[19], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[20]", qualL[20], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "qualL[21]", qualL[21], 0, DataTypeUInt8, uint8_t&, 0);

    ADD_MAP(m, totalSize, "sat[0]", sat[0], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[1]", sat[1], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[2]", sat[2], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[3]", sat[3], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[4]", sat[4], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[5]", sat[5], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[6]", sat[6], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[7]", sat[7], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[8]", sat[8], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[9]", sat[9], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[10]", sat[10], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[11]", sat[11], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[12]", sat[12], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[13]", sat[13], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[14]", sat[14], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[15]", sat[15], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[16]", sat[16], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[17]", sat[17], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[18]", sat[18], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[19]", sat[19], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[20]", sat[20], 0, DataTypeUInt8, uint8_t&, 0);
    ADD_MAP(m, totalSize, "sat[21]", sat[21], 0, DataTypeUInt8, uint8_t&, 0);

    ADD_MAP(m, totalSize, "satBiasStd[0]", satBiasStd[0], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[1]", satBiasStd[1], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[2]", satBiasStd[2], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[3]", satBiasStd[3], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[4]", satBiasStd[4], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[5]", satBiasStd[5], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[6]", satBiasStd[6], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[7]", satBiasStd[7], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[8]", satBiasStd[8], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[9]", satBiasStd[9], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[10]", satBiasStd[10], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[11]", satBiasStd[11], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[12]", satBiasStd[12], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[13]", satBiasStd[13], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[14]", satBiasStd[14], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[15]", satBiasStd[15], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[16]", satBiasStd[16], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[17]", satBiasStd[17], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[18]", satBiasStd[18], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[19]", satBiasStd[19], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[20]", satBiasStd[20], 0, DataTypeFloat, float&, 0);
    ADD_MAP(m, totalSize, "satBiasStd[21]", satBiasStd[21], 0, DataTypeFloat, float&, 0);

    ADD_MAP(m, totalSize, "satLockCnt[0]", satLockCnt[0], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[1]", satLockCnt[1], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[2]", satLockCnt[2], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[3]", satLockCnt[3], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[4]", satLockCnt[4], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[5]", satLockCnt[5], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[6]", satLockCnt[6], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[7]", satLockCnt[7], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[8]", satLockCnt[8], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[9]", satLockCnt[9], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[10]", satLockCnt[10], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[11]", satLockCnt[11], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[12]", satLockCnt[12], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[13]", satLockCnt[13], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[14]", satLockCnt[14], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[15]", satLockCnt[15], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[16]", satLockCnt[16], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[17]", satLockCnt[17], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[18]", satLockCnt[18], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[19]", satLockCnt[19], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[20]", satLockCnt[20], 0, DataTypeInt8, int8_t&, 0);
    ADD_MAP(m, totalSize, "satLockCnt[21]", satLockCnt[21], 0, DataTypeInt8, int8_t&, 0);

#endif

    ADD_MAP(m, totalSize, "num_biases", num_biases, 0, DataTypeUInt8, uint8_t, 0);

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
    "DID_GPS1_UBX_POS",                 // 6
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
    "DID_INTERNAL_DIAGNOSTIC",          // 20
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
    ""                                  // 119
};


cISDataMappings::cISDataMappings()
{
    PopulateSizeMappings(m_lookupSize);
    PopulateDeviceInfoMappings(m_lookupInfo, DID_DEV_INFO);
    PopulateManufacturingInfoMappings(m_lookupInfo);
    PopulateBitMappings(m_lookupInfo);
    PopulateSysFaultMappings(m_lookupInfo);
    PopulateIMU3Mappings(m_lookupInfo, DID_IMU3_UNCAL);
    PopulateIMU3Mappings(m_lookupInfo, DID_IMU3_RAW);
    PopulateIMUMappings(m_lookupInfo, DID_IMU_RAW);
    PopulateIMUMappings(m_lookupInfo, DID_IMU);
    PopulateIMUDeltaThetaVelocityMappings(m_lookupInfo, DID_PIMU);
    PopulateMagnetometerMappings(m_lookupInfo, DID_MAGNETOMETER);
    PopulateMagnetometerMappings(m_lookupInfo, DID_REFERENCE_MAGNETOMETER);
    PopulateBarometerMappings(m_lookupInfo);
    PopulateWheelEncoderMappings(m_lookupInfo);
    PopulateSysParamsMappings(m_lookupInfo);
    PopulateSysSensorsMappings(m_lookupInfo);
    PopulateRMCMappings(m_lookupInfo);
    PopulateINS1Mappings(m_lookupInfo);
    PopulateINS2Mappings(m_lookupInfo);
    PopulateINS3Mappings(m_lookupInfo);
    PopulateINS4Mappings(m_lookupInfo);
    PopulateGpsPosMappings(m_lookupInfo, DID_GPS1_POS);
    PopulateGpsPosMappings(m_lookupInfo, DID_GPS1_UBX_POS);
    PopulateGpsPosMappings(m_lookupInfo, DID_GPS2_POS);
    PopulateGpsPosMappings(m_lookupInfo, DID_GPS1_RTK_POS);
    PopulateGpsVelMappings(m_lookupInfo, DID_GPS1_VEL);
    PopulateGpsVelMappings(m_lookupInfo, DID_GPS2_VEL);
#if 0    // Too much data, we don't want to log this. WHJ
    PopulateGpsSatMappings(m_lookupInfo, DID_GPS1_SAT);
    PopulateGpsSatMappings(m_lookupInfo, DID_GPS2_SAT);
    PopulateGpsSigMappings(m_lookupInfo, DID_GPS1_SIG);
    PopulateGpsSigMappings(m_lookupInfo, DID_GPS2_SIG);
#endif
    PopulateGpsRtkRelMappings(m_lookupInfo, DID_GPS1_RTK_POS_REL);
    PopulateGpsRtkRelMappings(m_lookupInfo, DID_GPS2_RTK_CMP_REL);
    PopulateGpsRtkMiscMappings(m_lookupInfo, DID_GPS1_RTK_POS_MISC);
    PopulateGpsRtkMiscMappings(m_lookupInfo, DID_GPS2_RTK_CMP_MISC);
    PopulateGpsRawMappings(m_lookupInfo, DID_GPS1_RAW);
    PopulateGpsRawMappings(m_lookupInfo, DID_GPS2_RAW);
    PopulateGpsRawMappings(m_lookupInfo, DID_GPS_BASE_RAW);
    PopulateGroundVehicleMappings(m_lookupInfo);
    PopulateConfigMappings(m_lookupInfo);
    PopulateFlashConfigMappings(m_lookupInfo);
    PopulateDebugArrayMappings(m_lookupInfo, DID_DEBUG_ARRAY);
    PopulateEvbStatusMappings(m_lookupInfo);
    PopulateEvbFlashCfgMappings(m_lookupInfo);
    PopulateDebugArrayMappings(m_lookupInfo, DID_EVB_DEBUG_ARRAY);
    PopulateDeviceInfoMappings(m_lookupInfo, DID_EVB_DEV_INFO);
    PopulateIOMappings(m_lookupInfo);
    PopulateReferenceIMUMappings(m_lookupInfo);
    PopulateIMUDeltaThetaVelocityMappings(m_lookupInfo, DID_REFERENCE_PIMU);
    PopulateInfieldCalMappings(m_lookupInfo);

#if defined(INCLUDE_LUNA_DATA_SETS)
    PopulateEvbLunaFlashCfgMappings(m_lookupInfo);
    PopulateCoyoteStatusMappings(m_lookupInfo);
    PopulateEvbLunaSensorsMappings(m_lookupInfo);
    PopulateEvbLunaVelocityControlMappings(m_lookupInfo);
    PopulateEvbLunaVelocityCommandMappings(m_lookupInfo);
    PopulateEvbLunaAuxCmdMappings(m_lookupInfo);
#endif

    PopulateStrobeInTimeMappings(m_lookupInfo);
//    PopulateRtosInfoMappings(m_lookupInfo);
    PopulateDiagMsgMappings(m_lookupInfo);
    PopulateCanConfigMappings(m_lookupInfo);

#ifdef USE_IS_INTERNAL

    PopulateSensorsADCMappings(m_lookupInfo);
    PopulateSensorsISMappings(m_lookupInfo, DID_SENSORS_UCAL);
    PopulateSensorsISMappings(m_lookupInfo, DID_SENSORS_TCAL);
    PopulateSensorsISMappings(m_lookupInfo, DID_SENSORS_MCAL);
    PopulateSensorsTCMappings(m_lookupInfo);
    PopulateSensorsCompMappings(m_lookupInfo);
    PopulateUserPage0Mappings(m_lookupInfo);
    PopulateUserPage1Mappings(m_lookupInfo);
    PopulateInl2MagObsInfo(m_lookupInfo);
    PopulateInl2StatesMappings(m_lookupInfo);
//     PopulateRtkStateMappings(m_lookupInfo);
//     PopulateRtkResidualMappings(m_lookupInfo, DID_RTK_CODE_RESIDUAL);
//     PopulateRtkResidualMappings(m_lookupInfo, DID_RTK_PHASE_RESIDUAL);
    PopulateRtkDebugMappings(m_lookupInfo);
    // PopulateRtkDebug2Mappings(m_lookupInfo);
    PopulateIMUDeltaThetaVelocityMagMappings(m_lookupInfo);
    PopulateIMUMagnetometerMappings(m_lookupInfo);

#endif

    // this mustcome last
    for (uint32_t id = 0; id < DID_COUNT; id++)
    {
        PopulateTimestampField(id, m_timestampFields, m_lookupInfo);
    }
}


cISDataMappings::~cISDataMappings()
{
}


const char* cISDataMappings::GetDataSetName(uint32_t dataId)
{
    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(m_dataIdNames) == DID_COUNT);//

    if (dataId < DID_COUNT)
    {
        return m_dataIdNames[dataId];
    }
    return "unknown";
}


uint32_t cISDataMappings::GetDataSetId(string name)
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

#if PLATFORM_IS_EMBEDDED

    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

#endif

    if (dataId < DID_COUNT)
    {

#if PLATFORM_IS_EMBEDDED

        return &s_map->m_lookupInfo[dataId];

#else

        return &s_map.m_lookupInfo[dataId];

#endif

    }
    return NULLPTR;
}


uint32_t cISDataMappings::GetSize(uint32_t dataId)
{

#if PLATFORM_IS_EMBEDDED

    if (s_map == NULLPTR)
    {
        s_map = new cISDataMappings();
    }

#endif

#if PLATFORM_IS_EMBEDDED

    return (dataId < DID_MAX_COUNT ? s_map->m_lookupSize[dataId] : 0);

#else

    return (dataId < DID_COUNT ? s_map.m_lookupSize[dataId] : 0);

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
    case DataTypeInt8:
        *(int8_t*)dataBuffer = (int8_t)strtol(stringBuffer, NULL, radix);
        break;

    case DataTypeUInt8:
        *(uint8_t*)dataBuffer = (uint8_t)strtoul(stringBuffer, NULL, radix);
        break;

    case DataTypeInt16:
        *(int16_t*)dataBuffer = (int16_t)strtol(stringBuffer, NULL, radix);
        break;

    case DataTypeUInt16:
        *(uint16_t*)dataBuffer = (uint16_t)strtoul(stringBuffer, NULL, radix);
        break;

    case DataTypeInt32:
        *(int32_t*)dataBuffer = (int32_t)strtol(stringBuffer, NULL, radix);
        break;

    case DataTypeUInt32:
        *(uint32_t*)dataBuffer = (uint32_t)strtoul(stringBuffer, NULL, radix);
        break;

    case DataTypeInt64:
        *(int64_t*)dataBuffer = (int64_t)strtoll(stringBuffer, NULL, radix);
        break;

    case DataTypeUInt64:
        *(uint64_t*)dataBuffer = (uint64_t)strtoull(stringBuffer, NULL, radix);
        break;

    case DataTypeFloat:
        *(float*)dataBuffer = strtof(stringBuffer, NULL);
        break;

    case DataTypeDouble:
        *(double*)dataBuffer = strtod(stringBuffer, NULL);
        break;

    case DataTypeString:
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

    case DataTypeBinary:
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
        if (info.dataType == DataTypeString)
        {
            stringBuffer[0] = '"';
            stringBuffer[1] = '"';
            stringBuffer[2] = '\0';
        }
        else if (info.dataType == DataTypeBinary)
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
    case DataTypeInt8:
        if (dataFlags == DataFlagsDisplayHex)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(int8_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int8_t*)ptr);
        break;

    case DataTypeUInt8:
        if (dataFlags == DataFlagsDisplayHex)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%02X", *(uint8_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint8_t*)ptr);
        break;

    case DataTypeInt16:
        if (dataFlags == DataFlagsDisplayHex)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(int16_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int16_t*)ptr);
        break;

    case DataTypeUInt16:
        if (dataFlags == DataFlagsDisplayHex)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%04X", *(uint16_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint16_t*)ptr);
        break;

    case DataTypeInt32:
        if (dataFlags == DataFlagsDisplayHex)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(int32_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int32_t*)ptr);
        break;

    case DataTypeUInt32:
        if (dataFlags == DataFlagsDisplayHex)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%08X", *(uint32_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint32_t*)ptr);        
        break;

    case DataTypeInt64:
        if (dataFlags == DataFlagsDisplayHex)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (long long)*(uint64_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%lld", (long long)*(int64_t*)ptr);
        break;

    case DataTypeUInt64:
        if (dataFlags == DataFlagsDisplayHex)
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "0x%016llX", (unsigned long long)*(uint64_t*)ptr);
        else
            SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%llu", (unsigned long long)*(uint64_t*)ptr);
        break;

    case DataTypeFloat:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.9g", *(float*)ptr);
        break;

    case DataTypeDouble:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.17g", *(double*)ptr);
        break;

    case DataTypeString:
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

    case DataTypeBinary:
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
            if (timeStampField->dataType == DataTypeDouble)
            {
                // field is seconds, use as is
                return *(double*)ptr;
            }
            else if (timeStampField->dataType == DataTypeUInt32)
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
    if (offset >= 0 && offset <= fullSize - (int32_t)info.dataSize)
    {
        ptr = (buf + offset);
        return true;
    }
    ptr = NULL;
    return false;
}
