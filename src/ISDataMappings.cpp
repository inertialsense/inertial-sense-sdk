/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISDataMappings.h"
#include "ISUtilities.h"
#include "ISConstants.h"
#include "data_sets.h"

#ifdef USE_IS_INTERNAL
#include "../../libs/IS_internal.h"
#endif

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

cISDataMappings cISDataMappings::s_map;

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
    (uint32_t)0,
    (uint32_t)0
};

inline uint32_t GetDataTypeSize(eDataType dataType)
{
    if (dataType >= 0 && dataType < DataTypeCount)
    {
        return s_eDataTypeSizes[dataType];
    }
    return 0;
}

#if C11_IS_ENABLED

#define ADD_MAP_NO_VALIDATION(map, totalSize, name, type, member, dataSize, dataType, memberType) map[name] = { (uint32_t)OFFSETOF(type, member), (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize), dataType, name }; totalSize += sizeof(memberType);

// note when passing member type for arrays, it must be a reference, i.e. float&
#define ADD_MAP(map, totalSize, name, type, member, dataSize, dataType, memberType) \
    ADD_MAP_NO_VALIDATION(map, totalSize, name, type, member, dataSize, dataType, memberType); \
    static_assert(is_same<decltype(type::member), memberType>::value, "Member type is an unexpected type"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == sizeof(memberType), "Member type is an unexpected size"); \
    static_assert((uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == sizeof(type::member), "Member type is an unexpected size"); \
    static_assert(s_eDataTypeSizes[dataType] == 0 || (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == s_eDataTypeSizes[dataType], "Data type size does not match member size");
#define ASSERT_SIZE(s, t) assert(s == sizeof(t))

#else

#define ADD_MAP_NO_VALIDATION(map, totalSize, name, type, member, dataSize, dataType, memberType) map[name] = { (uint32_t)OFFSETOF(type, member), (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize), dataType, name }; totalSize += sizeof(memberType);
#define ADD_MAP(map, totalSize, name, type, member, dataSize, dataType, memberType) ADD_MAP_NO_VALIDATION(map, totalSize, name, type, member, dataSize, dataType, memberType)
#define ASSERT_SIZE(s, t) // not supported on VS < 2015

#endif

static void PopulateSizeMappings(map<uint32_t, uint32_t>& sizeMap)
{
    sizeMap[DID_DEV_INFO] = sizeof(dev_info_t);
    sizeMap[DID_IMU_1] = sizeof(imu_t);
    sizeMap[DID_IMU_2] = sizeof(imu_t);
    sizeMap[DID_MAGNETOMETER_1] = sizeof(magnetometer_t);
    sizeMap[DID_MAGNETOMETER_2] = sizeof(magnetometer_t);
    sizeMap[DID_BAROMETER] = sizeof(barometer_t);
    sizeMap[DID_DELTA_THETA_VEL] = sizeof(delta_theta_vel_t);
    sizeMap[DID_INS_1] = sizeof(ins_1_t);
    sizeMap[DID_INS_2] = sizeof(ins_2_t);
    sizeMap[DID_INS_3] = sizeof(ins_3_t);
    sizeMap[DID_INS_4] = sizeof(ins_4_t);
    sizeMap[DID_GPS] = sizeof(gps_t);
    sizeMap[DID_CONFIG] = sizeof(config_t);
    sizeMap[DID_GPS_POS] = sizeof(gps_nav_poslla_t);
    sizeMap[DID_GPS_VEL] = sizeof(gps_nav_velned_t);
    sizeMap[DID_GPS_CNO] = sizeof(gps_cno_t);
    sizeMap[DID_INS_MISC] = sizeof(ins_misc_t);
    sizeMap[DID_SYS_PARAMS] = sizeof(sys_params_t);
    sizeMap[DID_SYS_SENSORS] = sizeof(sys_sensors_t);
    sizeMap[DID_FLASH_CONFIG] = sizeof(nvm_flash_cfg_t);
    sizeMap[DID_INS_RESOURCES] = sizeof(ins_res_t);
    sizeMap[DID_RTK_SOL] = sizeof(rtk_sol_t);
    sizeMap[DID_DUAL_IMU] = sizeof(dual_imu_t);
    sizeMap[DID_RAW_GPS_DATA] = sizeof(raw_gps_msg_t);
    sizeMap[DID_RTOS_INFO] = sizeof(rtos_info_t);

#ifdef USE_IS_INTERNAL

    sizeMap[DID_SENSORS_ADC] = sizeof(sys_sensors_adc_t);
    sizeMap[DID_SENSORS_IS1] = sizeof(sensors_w_temp_t);
    sizeMap[DID_SENSORS_IS2] = sizeof(sensors_w_temp_t);
    sizeMap[DID_SENSORS_TC_BIAS] = sizeof(sensors_t);
    sizeMap[DID_SENSORS_CF_BIAS] = sizeof(sensor_bias_t);
    sizeMap[DID_SCOMP] = sizeof(sensor_compensation_t);
    sizeMap[DID_DEBUG_ARRAY] = sizeof(debug_array_t);
    sizeMap[DID_NVR_USERPAGE_G0] = sizeof(nvm_group_0_t);
    sizeMap[DID_NVR_USERPAGE_G1] = sizeof(nvm_group_1_t);
    sizeMap[DID_INL2_STATES] = sizeof(inl2_states_t);
    sizeMap[DID_INL2_STATUS] = sizeof(inl2_status_t);
    sizeMap[DID_INL2_MISC] = sizeof(inl2_misc_t);

#endif

}

static void PopulateDeviceInfoMappings(map_lookup_name_t& mappings)
{
    typedef dev_info_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "reserved", MAP_TYPE, reserved, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "serialNumber", MAP_TYPE, serialNumber, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hardwareVer[0]", MAP_TYPE, hardwareVer[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "hardwareVer[1]", MAP_TYPE, hardwareVer[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "hardwareVer[2]", MAP_TYPE, hardwareVer[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "hardwareVer[3]", MAP_TYPE, hardwareVer[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "firmwareVer[0]", MAP_TYPE, firmwareVer[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "firmwareVer[1]", MAP_TYPE, firmwareVer[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "firmwareVer[2]", MAP_TYPE, firmwareVer[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "firmwareVer[3]", MAP_TYPE, firmwareVer[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildNumber", MAP_TYPE, buildNumber, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "protocolVer[0]", MAP_TYPE, protocolVer[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "protocolVer[1]", MAP_TYPE, protocolVer[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "protocolVer[2]", MAP_TYPE, protocolVer[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "protocolVer[3]", MAP_TYPE, protocolVer[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "repoRevision", MAP_TYPE, repoRevision, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "manufacturer", MAP_TYPE, manufacturer, DEVINFO_MANUFACTURER_STRLEN, DataTypeString, char[DEVINFO_MANUFACTURER_STRLEN]);
    ADD_MAP(m, totalSize, "buildDate[0]", MAP_TYPE, buildDate[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildDate[1]", MAP_TYPE, buildDate[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildDate[2]", MAP_TYPE, buildDate[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildDate[3]", MAP_TYPE, buildDate[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildTime[0]", MAP_TYPE, buildTime[0], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildTime[1]", MAP_TYPE, buildTime[1], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildTime[2]", MAP_TYPE, buildTime[2], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "buildTime[3]", MAP_TYPE, buildTime[3], 0, DataTypeUInt8, uint8_t&);
    ADD_MAP(m, totalSize, "addInfo", MAP_TYPE, addInfo, DEVINFO_ADDINFO_STRLEN, DataTypeString, char[DEVINFO_ADDINFO_STRLEN]);
    mappings[DID_DEV_INFO] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateIMUMappings(map_lookup_name_t& mappings)
{
    typedef imu_t MAP_TYPE;
    typedef dual_imu_t MAP_TYPE2;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", MAP_TYPE, time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "pqr[0]", MAP_TYPE, I.pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[1]", MAP_TYPE, I.pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[2]", MAP_TYPE, I.pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[0]", MAP_TYPE, I.acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[1]", MAP_TYPE, I.acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[2]", MAP_TYPE, I.acc[2], 0, DataTypeFloat, float&);
    mappings[DID_IMU_1] = m;
    mappings[DID_IMU_2] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);

    map_name_to_info_t m2;
    totalSize = 0;
    ADD_MAP(m2, totalSize, "time", MAP_TYPE2, time, 0, DataTypeDouble, double);
    ADD_MAP(m2, totalSize, "pqr1[0]", MAP_TYPE2, I[0].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "pqr1[1]", MAP_TYPE2, I[0].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "pqr1[2]", MAP_TYPE2, I[0].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "acc1[0]", MAP_TYPE2, I[0].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "acc1[1]", MAP_TYPE2, I[0].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "acc1[2]", MAP_TYPE2, I[0].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "pqr2[0]", MAP_TYPE2, I[1].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "pqr2[1]", MAP_TYPE2, I[1].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "pqr2[2]", MAP_TYPE2, I[1].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "acc2[0]", MAP_TYPE2, I[1].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "acc2[1]", MAP_TYPE2, I[1].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m2, totalSize, "acc2[2]", MAP_TYPE2, I[1].acc[2], 0, DataTypeFloat, float&);
    mappings[DID_DUAL_IMU] = m2;

    ASSERT_SIZE(totalSize, MAP_TYPE2);
}

static void PopulateSysParamsMappings(map_lookup_name_t& mappings)
{
    typedef sys_params_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", MAP_TYPE, timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "insStatus", MAP_TYPE, insStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hdwStatus", MAP_TYPE, hdwStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "alignAttDetect", MAP_TYPE, alignAttDetect, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "alignAttError", MAP_TYPE, alignAttError, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "alignVelError", MAP_TYPE, alignVelError, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "alignPosError", MAP_TYPE, alignPosError, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "sampleDtMs", MAP_TYPE, sampleDtMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "navDtMs", MAP_TYPE, navDtMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "ftf0", MAP_TYPE, ftf0, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magInclination", MAP_TYPE, magInclination, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magDeclination", MAP_TYPE, magDeclination, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magMagnitude", MAP_TYPE, magMagnitude, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "genFaultCode", MAP_TYPE, genFaultCode, 0, DataTypeUInt32, uint32_t);
    mappings[DID_SYS_PARAMS] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateSysSensorsMappings(map_lookup_name_t& mappings)
{
    typedef sys_sensors_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", MAP_TYPE, time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "temp", MAP_TYPE, temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pqr[0]", MAP_TYPE, pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[1]", MAP_TYPE, pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[2]", MAP_TYPE, pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[0]", MAP_TYPE, acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[1]", MAP_TYPE, acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[2]", MAP_TYPE, acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[0]", MAP_TYPE, mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[1]", MAP_TYPE, mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[2]", MAP_TYPE, mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "bar", MAP_TYPE, bar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "barTemp", MAP_TYPE, barTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "mslBar", MAP_TYPE, mslBar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "humidity", MAP_TYPE, humidity, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "vin", MAP_TYPE, vin, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ana1", MAP_TYPE, ana1, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ana3", MAP_TYPE, ana1, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ana4", MAP_TYPE, ana1, 0, DataTypeFloat, float);
    mappings[DID_SYS_SENSORS] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateINS1Mappings(map_lookup_name_t& mappings)
{
    typedef ins_1_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", MAP_TYPE, week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeek", MAP_TYPE, timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "insStatus", MAP_TYPE, insStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hdwStatus", MAP_TYPE, hdwStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "theta[0]", MAP_TYPE, theta[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[1]", MAP_TYPE, theta[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[2]", MAP_TYPE, theta[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[0]", MAP_TYPE, uvw[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[1]", MAP_TYPE, uvw[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[2]", MAP_TYPE, uvw[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "lla[0]", MAP_TYPE, lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", MAP_TYPE, lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", MAP_TYPE, lla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ned[0]", MAP_TYPE, ned[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[1]", MAP_TYPE, ned[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[2]", MAP_TYPE, ned[2], 0, DataTypeFloat, float&);
    mappings[DID_INS_1] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateINS2Mappings(map_lookup_name_t& mappings, uint32_t did)
{
    typedef ins_2_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", MAP_TYPE, week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeek", MAP_TYPE, timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "insStatus", MAP_TYPE, insStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hdwStatus", MAP_TYPE, hdwStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "qn2b[0]", MAP_TYPE, qn2b[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[1]", MAP_TYPE, qn2b[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[2]", MAP_TYPE, qn2b[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[3]", MAP_TYPE, qn2b[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[0]", MAP_TYPE, uvw[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[1]", MAP_TYPE, uvw[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[2]", MAP_TYPE, uvw[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "lla[0]", MAP_TYPE, lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", MAP_TYPE, lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", MAP_TYPE, lla[2], 0, DataTypeDouble, double&);
    mappings[did] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateINS4Mappings(map_lookup_name_t& mappings)
{
    typedef ins_4_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", MAP_TYPE, week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeek", MAP_TYPE, timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "insStatus", MAP_TYPE, insStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "hdwStatus", MAP_TYPE, hdwStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "qe2b[0]", MAP_TYPE, qe2b[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[1]", MAP_TYPE, qe2b[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[2]", MAP_TYPE, qe2b[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[3]", MAP_TYPE, qe2b[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[0]", MAP_TYPE, ve[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[1]", MAP_TYPE, ve[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[2]", MAP_TYPE, ve[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ecef[0]", MAP_TYPE, ecef[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ecef[1]", MAP_TYPE, ecef[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ecef[2]", MAP_TYPE, ecef[2], 0, DataTypeDouble, double&);
    mappings[DID_INS_4] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateGPSMappings(map_lookup_name_t& mappings)
{
    typedef gps_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", MAP_TYPE, pos.week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeekMs", MAP_TYPE, pos.timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "status", MAP_TYPE, pos.status, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno", MAP_TYPE, pos.cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lla[0]", MAP_TYPE, pos.lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", MAP_TYPE, pos.lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", MAP_TYPE, pos.lla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "hMSL", MAP_TYPE, pos.hMSL, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "hAcc", MAP_TYPE, pos.hAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "vAcc", MAP_TYPE, pos.vAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pDop", MAP_TYPE, pos.pDop, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "timeOfWeekMs2", MAP_TYPE, vel.timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "ned[0]", MAP_TYPE, vel.ned[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[1]", MAP_TYPE, vel.ned[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[2]", MAP_TYPE, vel.ned[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "s2D", MAP_TYPE, vel.s2D, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "s3D", MAP_TYPE, vel.s3D, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "sAcc", MAP_TYPE, vel.sAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "course", MAP_TYPE, vel.course, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "cAcc", MAP_TYPE, vel.cAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "rxps", MAP_TYPE, rxps, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "towOffset", MAP_TYPE, towOffset, 0, DataTypeDouble, double);
    mappings[DID_GPS] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateGPSPosMappings(map_lookup_name_t& mappings)
{
    typedef gps_nav_poslla_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "week", MAP_TYPE, week, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "timeOfWeekMs", MAP_TYPE, timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "status", MAP_TYPE, status, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno", MAP_TYPE, cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lla[0]", MAP_TYPE, lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", MAP_TYPE, lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", MAP_TYPE, lla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "hMSL", MAP_TYPE, hMSL, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "hAcc", MAP_TYPE, hAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "vAcc", MAP_TYPE, vAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pDop", MAP_TYPE, pDop, 0, DataTypeFloat, float);
    mappings[DID_GPS_POS] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateGPSVelocityMappings(map_lookup_name_t& mappings)
{
    typedef gps_nav_velned_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", MAP_TYPE, timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "ned[0]", MAP_TYPE, ned[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[1]", MAP_TYPE, ned[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[2]", MAP_TYPE, ned[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "s2D", MAP_TYPE, s2D, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "s3D", MAP_TYPE, s3D, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "sAcc", MAP_TYPE, sAcc, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "course", MAP_TYPE, course, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "cAcc", MAP_TYPE, cAcc, 0, DataTypeFloat, float);
    mappings[DID_GPS_VEL] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateGPSCNOMappings(map_lookup_name_t& mappings)
{
    typedef gps_cno_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", MAP_TYPE, timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "numSats", MAP_TYPE, numSats, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[0]", MAP_TYPE, info[0].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[1]", MAP_TYPE, info[1].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[2]", MAP_TYPE, info[2].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[3]", MAP_TYPE, info[3].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[4]", MAP_TYPE, info[4].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[5]", MAP_TYPE, info[5].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[6]", MAP_TYPE, info[6].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[7]", MAP_TYPE, info[7].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[8]", MAP_TYPE, info[8].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[9]", MAP_TYPE, info[9].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[10]", MAP_TYPE, info[10].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[11]", MAP_TYPE, info[11].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[12]", MAP_TYPE, info[12].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[13]", MAP_TYPE, info[13].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[14]", MAP_TYPE, info[14].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[15]", MAP_TYPE, info[15].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[16]", MAP_TYPE, info[16].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[17]", MAP_TYPE, info[17].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[18]", MAP_TYPE, info[18].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[19]", MAP_TYPE, info[19].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[20]", MAP_TYPE, info[20].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[21]", MAP_TYPE, info[21].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[22]", MAP_TYPE, info[22].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[23]", MAP_TYPE, info[23].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[24]", MAP_TYPE, info[24].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[25]", MAP_TYPE, info[25].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[26]", MAP_TYPE, info[26].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[27]", MAP_TYPE, info[27].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[28]", MAP_TYPE, info[28].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[29]", MAP_TYPE, info[29].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[30]", MAP_TYPE, info[30].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[31]", MAP_TYPE, info[31].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[32]", MAP_TYPE, info[32].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[33]", MAP_TYPE, info[33].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[34]", MAP_TYPE, info[34].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[35]", MAP_TYPE, info[35].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[36]", MAP_TYPE, info[36].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[37]", MAP_TYPE, info[37].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[38]", MAP_TYPE, info[38].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[39]", MAP_TYPE, info[39].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[40]", MAP_TYPE, info[40].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[41]", MAP_TYPE, info[41].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[42]", MAP_TYPE, info[42].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[43]", MAP_TYPE, info[43].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[44]", MAP_TYPE, info[44].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[45]", MAP_TYPE, info[45].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[46]", MAP_TYPE, info[46].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[47]", MAP_TYPE, info[47].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[48]", MAP_TYPE, info[48].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "svId[49]", MAP_TYPE, info[49].svId, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[0]", MAP_TYPE, info[0].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[1]", MAP_TYPE, info[1].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[2]", MAP_TYPE, info[2].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[3]", MAP_TYPE, info[3].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[4]", MAP_TYPE, info[4].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[5]", MAP_TYPE, info[5].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[6]", MAP_TYPE, info[6].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[7]", MAP_TYPE, info[7].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[8]", MAP_TYPE, info[8].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[9]", MAP_TYPE, info[9].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[10]", MAP_TYPE, info[10].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[11]", MAP_TYPE, info[11].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[12]", MAP_TYPE, info[12].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[13]", MAP_TYPE, info[13].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[14]", MAP_TYPE, info[14].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[15]", MAP_TYPE, info[15].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[16]", MAP_TYPE, info[16].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[17]", MAP_TYPE, info[17].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[18]", MAP_TYPE, info[18].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[19]", MAP_TYPE, info[19].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[20]", MAP_TYPE, info[20].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[21]", MAP_TYPE, info[21].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[22]", MAP_TYPE, info[22].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[23]", MAP_TYPE, info[23].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[24]", MAP_TYPE, info[24].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[25]", MAP_TYPE, info[25].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[26]", MAP_TYPE, info[26].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[27]", MAP_TYPE, info[27].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[28]", MAP_TYPE, info[28].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[29]", MAP_TYPE, info[29].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[30]", MAP_TYPE, info[30].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[31]", MAP_TYPE, info[31].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[32]", MAP_TYPE, info[32].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[33]", MAP_TYPE, info[33].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[34]", MAP_TYPE, info[34].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[35]", MAP_TYPE, info[35].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[36]", MAP_TYPE, info[36].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[37]", MAP_TYPE, info[37].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[38]", MAP_TYPE, info[38].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[39]", MAP_TYPE, info[39].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[40]", MAP_TYPE, info[40].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[41]", MAP_TYPE, info[41].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[42]", MAP_TYPE, info[42].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[43]", MAP_TYPE, info[43].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[44]", MAP_TYPE, info[44].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[45]", MAP_TYPE, info[45].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[46]", MAP_TYPE, info[46].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[47]", MAP_TYPE, info[47].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[48]", MAP_TYPE, info[48].cno, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cno[49]", MAP_TYPE, info[49].cno, 0, DataTypeUInt32, uint32_t);
    mappings[DID_GPS_CNO] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateMagnetometerMappings(map_lookup_name_t& mappings)
{
    typedef magnetometer_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", MAP_TYPE, time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "mag[0]", MAP_TYPE, mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[1]", MAP_TYPE, mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[2]", MAP_TYPE, mag[2], 0, DataTypeFloat, float&);
    mappings[DID_MAGNETOMETER_1] = m;
    mappings[DID_MAGNETOMETER_2] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateBarometerMappings(map_lookup_name_t& mappings)
{
    typedef barometer_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", MAP_TYPE, time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "bar", MAP_TYPE, bar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "mslBar", MAP_TYPE, mslBar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "barTemp", MAP_TYPE, barTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "humidity", MAP_TYPE, humidity, 0, DataTypeFloat, float);
    mappings[DID_BAROMETER] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateDeltaThetaVelocityMappings(map_lookup_name_t& mappings)
{
    typedef delta_theta_vel_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", MAP_TYPE, time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "theta[0]", MAP_TYPE, theta[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[1]", MAP_TYPE, theta[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[2]", MAP_TYPE, theta[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[0]", MAP_TYPE, uvw[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[1]", MAP_TYPE, uvw[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[2]", MAP_TYPE, uvw[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dt", MAP_TYPE, dt, 0, DataTypeFloat, float);
    mappings[DID_DELTA_THETA_VEL] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateINSMiscMappings(map_lookup_name_t& mappings)
{
    typedef ins_misc_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeek", MAP_TYPE, timeOfWeek, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "timeOfWeekMs", MAP_TYPE, timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lla[0]", MAP_TYPE, x.lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", MAP_TYPE, x.lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", MAP_TYPE, x.lla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "uvw[0]", MAP_TYPE, x.uvw[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[1]", MAP_TYPE, x.uvw[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[2]", MAP_TYPE, x.uvw[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[0]", MAP_TYPE, x.qn2b[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[1]", MAP_TYPE, x.qn2b[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[2]", MAP_TYPE, x.qn2b[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[3]", MAP_TYPE, x.qn2b[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[0]", MAP_TYPE, theta[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[1]", MAP_TYPE, theta[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "theta[2]", MAP_TYPE, theta[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[0]", MAP_TYPE, ned[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[1]", MAP_TYPE, ned[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ned[2]", MAP_TYPE, ned[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[0]", MAP_TYPE, dcm[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[1]", MAP_TYPE, dcm[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[2]", MAP_TYPE, dcm[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[3]", MAP_TYPE, dcm[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[4]", MAP_TYPE, dcm[4], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[5]", MAP_TYPE, dcm[5], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[6]", MAP_TYPE, dcm[6], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[7]", MAP_TYPE, dcm[7], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "dcm[8]", MAP_TYPE, dcm[8], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[0]", MAP_TYPE, pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[1]", MAP_TYPE, pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr[2]", MAP_TYPE, pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[0]", MAP_TYPE, acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[1]", MAP_TYPE, acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc[2]", MAP_TYPE, acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[0]", MAP_TYPE, mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[1]", MAP_TYPE, mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag[2]", MAP_TYPE, mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mslBar", MAP_TYPE, mslBar, 0, DataTypeFloat, float);
    mappings[DID_INS_MISC] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateConfigMappings(map_lookup_name_t& mappings)
{
    typedef config_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "enBootloader", MAP_TYPE, enBootloader, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "solStreamCtrl", MAP_TYPE, solStreamCtrl, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "enSensorStats", MAP_TYPE, enSensorStats, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "enRTOSStats", MAP_TYPE, enBootloader, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "gpsStatus", MAP_TYPE, gpsStatus, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "system", MAP_TYPE, system, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "msgCfgBits", MAP_TYPE, msgCfgBits, 0, DataTypeUInt32, uint32_t);
    mappings[DID_CONFIG] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateFlashConfigMappings(map_lookup_name_t& mappings)
{
    typedef nvm_flash_cfg_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", MAP_TYPE, size, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "checksum", MAP_TYPE, checksum, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "key", MAP_TYPE, key, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "startupSampleDtMs", MAP_TYPE, startupSampleDtMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "startupNavDtMs", MAP_TYPE, startupNavDtMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "ser0BaudRate", MAP_TYPE, ser0BaudRate, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "ser1BaudRate", MAP_TYPE, ser1BaudRate, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "insRotation[0]", MAP_TYPE, insRotation[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insRotation[1]", MAP_TYPE, insRotation[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insRotation[2]", MAP_TYPE, insRotation[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insOffset[0]", MAP_TYPE, insOffset[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insOffset[1]", MAP_TYPE, insOffset[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insOffset[2]", MAP_TYPE, insOffset[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "gpsAntOffset[0]", MAP_TYPE, gpsAntOffset[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "gpsAntOffset[1]", MAP_TYPE, gpsAntOffset[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "gpsAntOffset[2]", MAP_TYPE, gpsAntOffset[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "insDynModel", MAP_TYPE, insDynModel, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "sysCfgBits", MAP_TYPE, sysCfgBits, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "refLla[0]", MAP_TYPE, refLla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "refLla[1]", MAP_TYPE, refLla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "refLla[2]", MAP_TYPE, refLla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lastLla[0]", MAP_TYPE, lastLla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lastLla[1]", MAP_TYPE, lastLla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lastLla[2]", MAP_TYPE, lastLla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lastLlaTimeOfWeekMs", MAP_TYPE, lastLlaTimeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lastLlaWeek", MAP_TYPE, lastLlaWeek, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lastLlaUpdateDistance", MAP_TYPE, lastLlaUpdateDistance, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ioConfig", MAP_TYPE, ioConfig, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cBrdConfig", MAP_TYPE, cBrdConfig, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "servoFailsafeTriggerUs", MAP_TYPE, servoFailsafeTriggerUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "servoFailsafePulseUs[0]", MAP_TYPE, servoFailsafePulseUs[0], 0, DataTypeUInt32, uint32_t&);
    ADD_MAP(m, totalSize, "servoFailsafePulseUs[1]", MAP_TYPE, servoFailsafePulseUs[1], 0, DataTypeUInt32, uint32_t&);
    ADD_MAP(m, totalSize, "servoFailsafePulseUs[2]", MAP_TYPE, servoFailsafePulseUs[2], 0, DataTypeUInt32, uint32_t&);
    ADD_MAP(m, totalSize, "servoFailsafePulseUs[3]", MAP_TYPE, servoFailsafePulseUs[3], 0, DataTypeUInt32, uint32_t&);
    ADD_MAP(m, totalSize, "servoFailsafePulseUs[4]", MAP_TYPE, servoFailsafePulseUs[4], 0, DataTypeUInt32, uint32_t&);
    ADD_MAP(m, totalSize, "servoFailsafePulseUs[5]", MAP_TYPE, servoFailsafePulseUs[5], 0, DataTypeUInt32, uint32_t&);
    ADD_MAP(m, totalSize, "servoFailsafePulseUs[6]", MAP_TYPE, servoFailsafePulseUs[6], 0, DataTypeUInt32, uint32_t&);
    ADD_MAP(m, totalSize, "servoFailsafePulseUs[7]", MAP_TYPE, servoFailsafePulseUs[7], 0, DataTypeUInt32, uint32_t&);
    ADD_MAP(m, totalSize, "magInclination", MAP_TYPE, magInclination, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magDeclination", MAP_TYPE, magDeclination, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magMagnitude", MAP_TYPE, magMagnitude, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magB[0]", MAP_TYPE, magB[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "magB[1]", MAP_TYPE, magB[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "magB[2]", MAP_TYPE, magB[2], 0, DataTypeFloat, float&);
    mappings[DID_FLASH_CONFIG] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateINSResourcesMappings(map_lookup_name_t& mappings)
{
    typedef ins_res_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", MAP_TYPE, timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lla[0]", MAP_TYPE, x_dot.lla[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[1]", MAP_TYPE, x_dot.lla[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lla[2]", MAP_TYPE, x_dot.lla[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "uvw[0]", MAP_TYPE, x_dot.uvw[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[1]", MAP_TYPE, x_dot.uvw[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "uvw[2]", MAP_TYPE, x_dot.uvw[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[0]", MAP_TYPE, x_dot.qn2b[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[1]", MAP_TYPE, x_dot.qn2b[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[2]", MAP_TYPE, x_dot.qn2b[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qn2b[3]", MAP_TYPE, x_dot.qn2b[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "magYawOffset", MAP_TYPE, magYawOffset, 0, DataTypeFloat, float);
    mappings[DID_INS_RESOURCES] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateRtkSolMappings(map_lookup_name_t& mappings)
{
    typedef rtk_sol_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "status", MAP_TYPE, status, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "seconds", MAP_TYPE, seconds, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "pos[0]", MAP_TYPE, pos[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "pos[1]", MAP_TYPE, pos[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "pos[2]", MAP_TYPE, pos[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "vel[0]", MAP_TYPE, vel[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "vel[1]", MAP_TYPE, vel[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "vel[2]", MAP_TYPE, vel[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "accuracy[0]", MAP_TYPE, accuracy[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracy[1]", MAP_TYPE, accuracy[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracy[2]", MAP_TYPE, accuracy[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracy[3]", MAP_TYPE, accuracy[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracy[4]", MAP_TYPE, accuracy[4], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "accuracy[5]", MAP_TYPE, accuracy[5], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "numberOfSatellites", MAP_TYPE, numberOfSatellites, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "age", MAP_TYPE, age, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ratio", MAP_TYPE, ratio, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "threshold", MAP_TYPE, threshold, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "gdop", MAP_TYPE, gdop, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "pdop", MAP_TYPE, pdop, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "hdop", MAP_TYPE, hdop, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "vdop", MAP_TYPE, vdop, 0, DataTypeDouble, double);
    mappings[DID_RTK_SOL] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateRtosInfoMappings(map_lookup_name_t& mappings)
{
    typedef rtos_info_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;

    ADD_MAP(m, totalSize, "name[0]", MAP_TYPE, task[0].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[0]", MAP_TYPE, task[0].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[0]", MAP_TYPE, task[0].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[0]", MAP_TYPE, task[0].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[0]", MAP_TYPE, task[0].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[0]", MAP_TYPE, task[0].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[0]", MAP_TYPE, task[0].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[0]", MAP_TYPE, task[0].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[1]", MAP_TYPE, task[1].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[1]", MAP_TYPE, task[1].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[1]", MAP_TYPE, task[1].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[1]", MAP_TYPE, task[1].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[1]", MAP_TYPE, task[1].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[1]", MAP_TYPE, task[1].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[1]", MAP_TYPE, task[1].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[1]", MAP_TYPE, task[1].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[2]", MAP_TYPE, task[2].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[2]", MAP_TYPE, task[2].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[2]", MAP_TYPE, task[2].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[2]", MAP_TYPE, task[2].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[2]", MAP_TYPE, task[2].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[2]", MAP_TYPE, task[2].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[2]", MAP_TYPE, task[2].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[2]", MAP_TYPE, task[2].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[3]", MAP_TYPE, task[3].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[3]", MAP_TYPE, task[3].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[3]", MAP_TYPE, task[3].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[3]", MAP_TYPE, task[3].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[3]", MAP_TYPE, task[3].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[3]", MAP_TYPE, task[3].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[3]", MAP_TYPE, task[3].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[3]", MAP_TYPE, task[3].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[4]", MAP_TYPE, task[4].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[4]", MAP_TYPE, task[4].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[4]", MAP_TYPE, task[4].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[4]", MAP_TYPE, task[4].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[4]", MAP_TYPE, task[4].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[4]", MAP_TYPE, task[4].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[4]", MAP_TYPE, task[4].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[4]", MAP_TYPE, task[4].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "name[5]", MAP_TYPE, task[5].name, MAX_TASK_NAME_LEN, DataTypeString, char[MAX_TASK_NAME_LEN]);
    ADD_MAP(m, totalSize, "priority[5]", MAP_TYPE, task[5].priority, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "stackUnused[5]", MAP_TYPE, task[5].stackUnused, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "periodMs[5]", MAP_TYPE, task[5].periodMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "runTimeUs[5]", MAP_TYPE, task[5].runTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "maxRunTimeUs[5]", MAP_TYPE, task[5].maxRunTimeUs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "cpuUsage[5]", MAP_TYPE, task[5].cpuUsage, 0, DataTypeFloat, f_t);
    ADD_MAP(m, totalSize, "handle[5]", MAP_TYPE, task[5].handle, 0, DataTypeUInt32, uint32_t);

    ADD_MAP(m, totalSize, "freeHeapSize", MAP_TYPE, freeHeapSize, 0, DataTypeUInt32, uint32_t);

    mappings[DID_RTOS_INFO] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

#ifdef USE_IS_INTERNAL

static void PopulateSensorsADCMappings(map_lookup_name_t& mappings)
{
    typedef sys_sensors_adc_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", MAP_TYPE, time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "pqr1[0]", MAP_TYPE, mpu[0].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", MAP_TYPE, mpu[0].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", MAP_TYPE, mpu[0].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", MAP_TYPE, mpu[0].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", MAP_TYPE, mpu[0].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", MAP_TYPE, mpu[0].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[0]", MAP_TYPE, mpu[0].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[1]", MAP_TYPE, mpu[0].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[2]", MAP_TYPE, mpu[0].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp1", MAP_TYPE, mpu[0].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pqr2[0]", MAP_TYPE, mpu[1].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", MAP_TYPE, mpu[1].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", MAP_TYPE, mpu[1].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", MAP_TYPE, mpu[1].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", MAP_TYPE, mpu[1].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", MAP_TYPE, mpu[1].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[0]", MAP_TYPE, mpu[1].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[1]", MAP_TYPE, mpu[1].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[2]", MAP_TYPE, mpu[1].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp2", MAP_TYPE, mpu[1].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "bar", MAP_TYPE, bar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "barTemp", MAP_TYPE, barTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "humidity", MAP_TYPE, humidity, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "ana[0]", MAP_TYPE, ana[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ana[1]", MAP_TYPE, ana[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ana[2]", MAP_TYPE, ana[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ana[3]", MAP_TYPE, ana[3], 0, DataTypeFloat, float&);
    mappings[DID_SENSORS_ADC] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateSensorsISMappings(map_lookup_name_t& mappings)
{
    typedef sensors_w_temp_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "pqr1[0]", MAP_TYPE, mpu[0].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", MAP_TYPE, mpu[0].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", MAP_TYPE, mpu[0].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", MAP_TYPE, mpu[0].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", MAP_TYPE, mpu[0].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", MAP_TYPE, mpu[0].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[0]", MAP_TYPE, mpu[0].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[1]", MAP_TYPE, mpu[0].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[2]", MAP_TYPE, mpu[0].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp1", MAP_TYPE, mpu[0].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pqr2[0]", MAP_TYPE, mpu[1].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", MAP_TYPE, mpu[1].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", MAP_TYPE, mpu[1].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", MAP_TYPE, mpu[1].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", MAP_TYPE, mpu[1].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", MAP_TYPE, mpu[1].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[0]", MAP_TYPE, mpu[1].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[1]", MAP_TYPE, mpu[1].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[2]", MAP_TYPE, mpu[1].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp2", MAP_TYPE, mpu[1].temp, 0, DataTypeFloat, float);
    mappings[DID_SENSORS_IS1] = m;
    mappings[DID_SENSORS_IS2] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);

}

static void PopulateSensorsTCMappings(map_lookup_name_t& mappings)
{
    typedef sensors_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "pqr1[0]", MAP_TYPE, mpu[0].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", MAP_TYPE, mpu[0].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", MAP_TYPE, mpu[0].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", MAP_TYPE, mpu[0].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", MAP_TYPE, mpu[0].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", MAP_TYPE, mpu[0].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[0]", MAP_TYPE, mpu[0].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[1]", MAP_TYPE, mpu[0].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[2]", MAP_TYPE, mpu[0].mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[0]", MAP_TYPE, mpu[1].pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", MAP_TYPE, mpu[1].pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", MAP_TYPE, mpu[1].pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", MAP_TYPE, mpu[1].acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", MAP_TYPE, mpu[1].acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", MAP_TYPE, mpu[1].acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[0]", MAP_TYPE, mpu[1].mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[1]", MAP_TYPE, mpu[1].mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[2]", MAP_TYPE, mpu[1].mag[2], 0, DataTypeFloat, float&);
    mappings[DID_SENSORS_TC_BIAS] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateSensorsCFMappings(map_lookup_name_t& mappings)
{
    typedef sensor_bias_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "timeOfWeekMs", MAP_TYPE, timeOfWeekMs, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "pqr1[0]", MAP_TYPE, pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", MAP_TYPE, pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", MAP_TYPE, pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", MAP_TYPE, acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", MAP_TYPE, acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", MAP_TYPE, acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mslBar", MAP_TYPE, mslBar, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magI[0]", MAP_TYPE, magI[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "magI[1]", MAP_TYPE, magI[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "magI[2]", MAP_TYPE, magI[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "magB[0]", MAP_TYPE, magB[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "magB[1]", MAP_TYPE, magB[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "magB[2]", MAP_TYPE, magB[2], 0, DataTypeFloat, float&);
    mappings[DID_SENSORS_CF_BIAS] = m;
    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateSensorsCompMappings(map_lookup_name_t& mappings)
{
    typedef sensor_compensation_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "pqr1[0]", MAP_TYPE, mpu[0].lpfLsb.pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[1]", MAP_TYPE, mpu[0].lpfLsb.pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr1[2]", MAP_TYPE, mpu[0].lpfLsb.pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[0]", MAP_TYPE, mpu[0].lpfLsb.acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[1]", MAP_TYPE, mpu[0].lpfLsb.acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc1[2]", MAP_TYPE, mpu[0].lpfLsb.acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[0]", MAP_TYPE, mpu[0].lpfLsb.mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[1]", MAP_TYPE, mpu[0].lpfLsb.mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag1[2]", MAP_TYPE, mpu[0].lpfLsb.mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp1", MAP_TYPE, mpu[0].lpfLsb.temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "temp2", MAP_TYPE, mpu[0].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "tempRampRate1", MAP_TYPE, mpu[0].tempRampRate, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "tci1", MAP_TYPE, mpu[0].tci, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "numTcPts1", MAP_TYPE, mpu[0].numTcPts, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "dtTemp1", MAP_TYPE, mpu[0].dtTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "pqr2[0]", MAP_TYPE, mpu[1].lpfLsb.pqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[1]", MAP_TYPE, mpu[1].lpfLsb.pqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "pqr2[2]", MAP_TYPE, mpu[1].lpfLsb.pqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[0]", MAP_TYPE, mpu[1].lpfLsb.acc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[1]", MAP_TYPE, mpu[1].lpfLsb.acc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "acc2[2]", MAP_TYPE, mpu[1].lpfLsb.acc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[0]", MAP_TYPE, mpu[1].lpfLsb.mag[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[1]", MAP_TYPE, mpu[1].lpfLsb.mag[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag2[2]", MAP_TYPE, mpu[1].lpfLsb.mag[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "temp3", MAP_TYPE, mpu[1].lpfLsb.temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "temp4", MAP_TYPE, mpu[1].temp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "tempRampRate2", MAP_TYPE, mpu[1].tempRampRate, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "tci2", MAP_TYPE, mpu[1].tci, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "numTcPts2", MAP_TYPE, mpu[1].numTcPts, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "dtTemp2", MAP_TYPE, mpu[1].dtTemp, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "sampleCount", MAP_TYPE, sampleCount, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "calState", MAP_TYPE, calState, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "alignAccel[0]", MAP_TYPE, alignAccel[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "alignAccel[1]", MAP_TYPE, alignAccel[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "alignAccel[2]", MAP_TYPE, alignAccel[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "status", MAP_TYPE, status, 0, DataTypeUInt32, uint32_t);
    mappings[DID_SCOMP] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateDebugArrayMappings(map_lookup_name_t& mappings)
{
    typedef debug_array_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "i[0]", MAP_TYPE, i[0], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[1]", MAP_TYPE, i[1], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[2]", MAP_TYPE, i[2], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[3]", MAP_TYPE, i[3], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[4]", MAP_TYPE, i[4], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[5]", MAP_TYPE, i[5], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[6]", MAP_TYPE, i[6], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[7]", MAP_TYPE, i[7], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "i[8]", MAP_TYPE, i[8], 0, DataTypeInt32, int32_t&);
    ADD_MAP(m, totalSize, "f[0]", MAP_TYPE, f[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[1]", MAP_TYPE, f[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[2]", MAP_TYPE, f[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[3]", MAP_TYPE, f[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[4]", MAP_TYPE, f[4], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[5]", MAP_TYPE, f[5], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[6]", MAP_TYPE, f[6], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[7]", MAP_TYPE, f[7], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "f[8]", MAP_TYPE, f[8], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "lf[0]", MAP_TYPE, lf[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lf[1]", MAP_TYPE, lf[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "lf[2]", MAP_TYPE, lf[2], 0, DataTypeDouble, double&);
    mappings[DID_DEBUG_ARRAY] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateUserPage0Mappings(map_lookup_name_t& mappings)
{
    typedef nvm_group_0_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", MAP_TYPE, size, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "checksum", MAP_TYPE, checksum, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "key", MAP_TYPE, key, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "lockBits", MAP_TYPE, lockBits, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "featureBits", MAP_TYPE, featureBits, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "featureHash1", MAP_TYPE, featureHash1, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "featureHash2", MAP_TYPE, featureHash2, 0, DataTypeUInt32, uint32_t);
    mappings[DID_NVR_USERPAGE_G0] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateUserPage1Mappings(map_lookup_name_t& mappings)
{
    typedef nvm_group_1_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "size", MAP_TYPE, size, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "checksum", MAP_TYPE, checksum, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "key", MAP_TYPE, key, 0, DataTypeUInt32, uint32_t);
    ADD_MAP(m, totalSize, "bKpqr", MAP_TYPE, cf.bKpqr, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "bKuvw", MAP_TYPE, cf.bKuvw, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "oKat1", MAP_TYPE, cf.oKat1, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "oKat2", MAP_TYPE, cf.oKat2, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "oKuvw", MAP_TYPE, cf.oKuvw, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "oKlla", MAP_TYPE, cf.oKlla, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "mag.bias_cal[0]", MAP_TYPE, mag.bias_cal[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.bias_cal[1]", MAP_TYPE, mag.bias_cal[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.bias_cal[2]", MAP_TYPE, mag.bias_cal[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[0]", MAP_TYPE, mag.Wcal[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[1]", MAP_TYPE, mag.Wcal[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[2]", MAP_TYPE, mag.Wcal[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[3]", MAP_TYPE, mag.Wcal[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[4]", MAP_TYPE, mag.Wcal[4], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[5]", MAP_TYPE, mag.Wcal[5], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[6]", MAP_TYPE, mag.Wcal[6], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[7]", MAP_TYPE, mag.Wcal[7], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.Wcal[8]", MAP_TYPE, mag.Wcal[8], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[0]", MAP_TYPE, mag.DtD[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[1]", MAP_TYPE, mag.DtD[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[2]", MAP_TYPE, mag.DtD[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[3]", MAP_TYPE, mag.DtD[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[4]", MAP_TYPE, mag.DtD[4], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[5]", MAP_TYPE, mag.DtD[5], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[6]", MAP_TYPE, mag.DtD[6], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[7]", MAP_TYPE, mag.DtD[7], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[8]", MAP_TYPE, mag.DtD[8], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[9]", MAP_TYPE, mag.DtD[9], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[10]", MAP_TYPE, mag.DtD[10], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[11]", MAP_TYPE, mag.DtD[11], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[12]", MAP_TYPE, mag.DtD[12], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[13]", MAP_TYPE, mag.DtD[13], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[14]", MAP_TYPE, mag.DtD[14], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[15]", MAP_TYPE, mag.DtD[15], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[16]", MAP_TYPE, mag.DtD[16], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[17]", MAP_TYPE, mag.DtD[17], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[18]", MAP_TYPE, mag.DtD[18], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[19]", MAP_TYPE, mag.DtD[19], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[20]", MAP_TYPE, mag.DtD[20], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[21]", MAP_TYPE, mag.DtD[21], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[22]", MAP_TYPE, mag.DtD[22], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[23]", MAP_TYPE, mag.DtD[23], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[24]", MAP_TYPE, mag.DtD[24], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[25]", MAP_TYPE, mag.DtD[25], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[26]", MAP_TYPE, mag.DtD[26], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[27]", MAP_TYPE, mag.DtD[27], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[28]", MAP_TYPE, mag.DtD[28], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[29]", MAP_TYPE, mag.DtD[29], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[30]", MAP_TYPE, mag.DtD[30], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[31]", MAP_TYPE, mag.DtD[31], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[32]", MAP_TYPE, mag.DtD[32], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[33]", MAP_TYPE, mag.DtD[33], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[34]", MAP_TYPE, mag.DtD[34], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[35]", MAP_TYPE, mag.DtD[35], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[36]", MAP_TYPE, mag.DtD[36], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[37]", MAP_TYPE, mag.DtD[37], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[38]", MAP_TYPE, mag.DtD[38], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[39]", MAP_TYPE, mag.DtD[39], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[40]", MAP_TYPE, mag.DtD[40], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[41]", MAP_TYPE, mag.DtD[41], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[42]", MAP_TYPE, mag.DtD[42], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[43]", MAP_TYPE, mag.DtD[43], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[44]", MAP_TYPE, mag.DtD[44], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[45]", MAP_TYPE, mag.DtD[45], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[46]", MAP_TYPE, mag.DtD[46], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[47]", MAP_TYPE, mag.DtD[47], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[48]", MAP_TYPE, mag.DtD[48], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[49]", MAP_TYPE, mag.DtD[49], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[50]", MAP_TYPE, mag.DtD[50], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[51]", MAP_TYPE, mag.DtD[51], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[52]", MAP_TYPE, mag.DtD[52], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[53]", MAP_TYPE, mag.DtD[53], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[54]", MAP_TYPE, mag.DtD[54], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[55]", MAP_TYPE, mag.DtD[55], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[56]", MAP_TYPE, mag.DtD[56], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[57]", MAP_TYPE, mag.DtD[57], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[58]", MAP_TYPE, mag.DtD[58], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[59]", MAP_TYPE, mag.DtD[59], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[60]", MAP_TYPE, mag.DtD[60], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[61]", MAP_TYPE, mag.DtD[61], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[62]", MAP_TYPE, mag.DtD[62], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[63]", MAP_TYPE, mag.DtD[63], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[64]", MAP_TYPE, mag.DtD[64], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[65]", MAP_TYPE, mag.DtD[65], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[66]", MAP_TYPE, mag.DtD[66], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[67]", MAP_TYPE, mag.DtD[67], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[68]", MAP_TYPE, mag.DtD[68], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[69]", MAP_TYPE, mag.DtD[69], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[70]", MAP_TYPE, mag.DtD[70], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[71]", MAP_TYPE, mag.DtD[71], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[72]", MAP_TYPE, mag.DtD[72], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[73]", MAP_TYPE, mag.DtD[73], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[74]", MAP_TYPE, mag.DtD[74], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[75]", MAP_TYPE, mag.DtD[75], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[76]", MAP_TYPE, mag.DtD[76], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[77]", MAP_TYPE, mag.DtD[77], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[78]", MAP_TYPE, mag.DtD[78], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[79]", MAP_TYPE, mag.DtD[79], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[80]", MAP_TYPE, mag.DtD[80], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[81]", MAP_TYPE, mag.DtD[81], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[82]", MAP_TYPE, mag.DtD[82], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[83]", MAP_TYPE, mag.DtD[83], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[84]", MAP_TYPE, mag.DtD[84], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[85]", MAP_TYPE, mag.DtD[85], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[86]", MAP_TYPE, mag.DtD[86], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[87]", MAP_TYPE, mag.DtD[87], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[88]", MAP_TYPE, mag.DtD[88], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[89]", MAP_TYPE, mag.DtD[89], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[90]", MAP_TYPE, mag.DtD[90], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[91]", MAP_TYPE, mag.DtD[91], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[92]", MAP_TYPE, mag.DtD[92], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[93]", MAP_TYPE, mag.DtD[93], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[94]", MAP_TYPE, mag.DtD[94], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[95]", MAP_TYPE, mag.DtD[95], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[96]", MAP_TYPE, mag.DtD[96], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[97]", MAP_TYPE, mag.DtD[97], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[98]", MAP_TYPE, mag.DtD[98], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "mag.DtD[99]", MAP_TYPE, mag.DtD[99], 0, DataTypeFloat, float&);

    mappings[DID_NVR_USERPAGE_G1] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateInl2StatesMappings(map_lookup_name_t& mappings)
{
    typedef inl2_states_t MAP_TYPE;
    map_name_to_info_t m;
    uint32_t totalSize = 0;
    ADD_MAP(m, totalSize, "time", MAP_TYPE, time, 0, DataTypeDouble, double);
    ADD_MAP(m, totalSize, "qe2b[0]", MAP_TYPE, qe2b[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[1]", MAP_TYPE, qe2b[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[2]", MAP_TYPE, qe2b[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "qe2b[3]", MAP_TYPE, qe2b[3], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[0]", MAP_TYPE, ve[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[1]", MAP_TYPE, ve[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ve[2]", MAP_TYPE, ve[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "ecef[0]", MAP_TYPE, ecef[0], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ecef[1]", MAP_TYPE, ecef[1], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "ecef[2]", MAP_TYPE, ecef[2], 0, DataTypeDouble, double&);
    ADD_MAP(m, totalSize, "biasPqr[0]", MAP_TYPE, biasPqr[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasPqr[1]", MAP_TYPE, biasPqr[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasPqr[2]", MAP_TYPE, biasPqr[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasAcc[0]", MAP_TYPE, biasAcc[0], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasAcc[1]", MAP_TYPE, biasAcc[1], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasAcc[2]", MAP_TYPE, biasAcc[2], 0, DataTypeFloat, float&);
    ADD_MAP(m, totalSize, "biasBaro", MAP_TYPE, biasBaro, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magDec", MAP_TYPE, magDec, 0, DataTypeFloat, float);
    ADD_MAP(m, totalSize, "magInc", MAP_TYPE, magInc, 0, DataTypeFloat, float);
    mappings[DID_INL2_STATES] = m;

    ASSERT_SIZE(totalSize, MAP_TYPE);
}

#endif

cISDataMappings::cISDataMappings()
{
    PopulateSizeMappings(m_lookupSize);
    PopulateDeviceInfoMappings(m_columnMappings);
    PopulateIMUMappings(m_columnMappings);
    PopulateSysParamsMappings(m_columnMappings);
    PopulateSysSensorsMappings(m_columnMappings);
    PopulateINS1Mappings(m_columnMappings);
    PopulateINS2Mappings(m_columnMappings, DID_INS_2);
    PopulateINS2Mappings(m_columnMappings, DID_INS_3);
    PopulateINS4Mappings(m_columnMappings);
    PopulateGPSMappings(m_columnMappings);
    PopulateGPSPosMappings(m_columnMappings);
    PopulateGPSVelocityMappings(m_columnMappings);
    PopulateGPSCNOMappings(m_columnMappings);
    PopulateMagnetometerMappings(m_columnMappings);
    PopulateBarometerMappings(m_columnMappings);
    PopulateDeltaThetaVelocityMappings(m_columnMappings);
    PopulateINSMiscMappings(m_columnMappings);
    PopulateConfigMappings(m_columnMappings);
    PopulateFlashConfigMappings(m_columnMappings);
    PopulateINSResourcesMappings(m_columnMappings);
    PopulateRtkSolMappings(m_columnMappings);
    PopulateRtosInfoMappings(m_columnMappings);

#ifdef USE_IS_INTERNAL

    PopulateSensorsADCMappings(m_columnMappings);
    PopulateSensorsISMappings(m_columnMappings);
    PopulateSensorsTCMappings(m_columnMappings);
    PopulateSensorsCFMappings(m_columnMappings);
    PopulateSensorsCompMappings(m_columnMappings);
    PopulateDebugArrayMappings(m_columnMappings);
    PopulateUserPage0Mappings(m_columnMappings);
    PopulateUserPage1Mappings(m_columnMappings);
    PopulateInl2StatesMappings(m_columnMappings);

#endif

}

const char* cISDataMappings::GetDataSetName(uint32_t dataId)
{
    static const char* s_dataIdNames[] =
    {
        "null", // 0: DID_NULL
        "devInfo", // 1: DID_DEV_INFO,
        "imu1", // 2: DID_IMU_1
        "dThetaVel", // 3: DID_DELTA_THETA_VEL
        "ins1", // 4: DID_INS_1
        "ins2", // 5: DID_INS_2
        "gps", // 6: DID_GPS
        "config", // 7: DID_CONFIG
        "ascii_msg", // 8: DID_ASCII_BCAST_PERIOD
        "misc", // 9: DID_INS_MISC
        "sysParams", // 10: DID_SYS_PARAMS
        "sysSensors", // 11: DID_SYS_SENSORS
        "flashCfg", // 12: DID_FLASH_CONFIG
        "gpsCNO", // 13: DID_GPS_CNO
        "gpsPos", // 14: DID_GPS_POS
        "gpsVel", // 15: DID_GPS_VEL,
        "ioServos", // 16: DID_IO
        "ioServosPwm", // 17: DID_IO_SERVOS_PWM
        "ioServosPpm", // 18: DID_IO_SERVOS_PPM
        "magCal", // 19: DID_MAG_CAL
        "insRes", // 20: DID_INS_RESOURCES
        "dgpsCorr", // 21: DID_DGPS_CORRECTION
        "rtkSol", // 22: DID_RTK_SOL
        "featureBits", // 23: DID_FEATURE_BITS
        "sensorIS1", // 24: DID_SENSORS_IS1
        "sensorIS2", // 25: DID_SENSORS_IS2
        "sensorTCbias", // 26: DID_SENSORS_TC_BIAS
        "sensorCFbias", // 27: DID_SENSORS_CF_BIAS
        "sensorLSB", // 28: DID_SENSORS_ADC
        "scomp", // 29: DID_SCOMP
        "insParams", // 30: DID_INS_PARAMS
        "obsParams", // 31: DID_OBS_PARAMS
        "hdwParams", // 32: DID_HDW_PARAMS
        "userPageNvr", // 33: DID_NVR_MANAGE_USERPAGE
        "userPageSn", // 34: DID_NVR_USERPAGE_SN
        "userpage0", // 35: DID_NVR_USERPAGE_G0
        "userpage1", // 36: DID_NVR_USERPAGE_G1
        "debugString", // 37: DID_DEBUG_STRING
        "rtosInfo", // 38: DID_RTOS_INFO
        "debugArray", // 39: DID_DEBUG_ARRAY
        "sensorCal1", // 40: DID_SENSORS_CAL1
        "sensorCal2", // 41: DID_SENSORS_CAL2
        "sensorCalSC", // 42: DID_CAL_SC
        "sensorCalSC1", // 43: DID_CAL_SC1
        "sensorCalSC2", // 44: DID_CAL_SC2
        "sensorSigma", // 45: DID_SYS_SENSORS_SIGMA
        "sensorAdcSigma", // 46: DID_SENSORS_ADC_SIGMA
        "insDev1", // 47: DID_INS_DEV_1
        "inl2States", // 48: DID_INL2_STATES
        "inl2CovarianceUD", // 49: DID_INL2_COVARIANCE_UD
        "inl2Status", // 50: DID_INL2_STATUS
        "inl2Misc", // 51: DID_INL2_MISC
        "magnetometer1", // 52: DID_MAGNETOMETER_1
        "barometer", // 53: DID_BAROMETER
        "imu2", // 54: DID_IMU_2
        "magnetometer2", // 55: DID_MAGNETOMETER_2
        "gpsVersion", // 56: DID_GPS_VERSION
        "commLoopback", // 57: DID_COMMUNICATIONS_LOOPBACK
        "imuDual", // 58: DID_DUAL_IMU
        "inl2MagObs", // 59: DID_INL2_MAG_OBS_INFO
        "rawGpsData", // 60: DID_RAW_GPS_DATA
        "rtkOptions", // 61: DID_RTK_OPT
        "userPageInternal", // 62: DID_NVR_USERPAGE_INTERNAL
        "manufacturingInfo", // 63: DID_MANUFACTURING_INFO
        "bit", // 64: DID_BIT
        "ins3", // 65: DID_INS_3
        "ins4", // 66: DID_INS_4
        "inl2Variance", // 67: DID_INL2_VARIANCE
    };

    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(s_dataIdNames) == DID_COUNT);

    if (dataId < DID_COUNT)
    {
        return s_dataIdNames[dataId];
    }
    return "unknown";
}


const map_lookup_name_t& cISDataMappings::GetMap()
{
    return s_map.m_columnMappings;
}


uint32_t cISDataMappings::GetSize(uint32_t dataId)
{
    return s_map.m_lookupSize[dataId];
}


bool cISDataMappings::StringToData(const char* stringBuffer, int stringLength, const p_data_hdr_t* hdr, uint8_t* dataBuffer, const data_info_t& info)
{
    const uint8_t* ptr;
    if (!CanGetFieldData(info, hdr, dataBuffer, ptr))
    {
        return false;
    }

    switch (info.dataType)
    {
    case DataTypeInt8:
        *(int8_t*)ptr = (int8_t)strtol(stringBuffer, NULL, 10);
        break;

    case DataTypeUInt8:
        *(uint8_t*)ptr = (uint8_t)strtoul(stringBuffer, NULL, 10);
        break;

    case DataTypeInt16:
        *(int16_t*)ptr = (int16_t)strtol(stringBuffer, NULL, 10);
        break;

    case DataTypeUInt16:
        *(uint16_t*)ptr = (uint16_t)strtoul(stringBuffer, NULL, 10);
        break;

    case DataTypeInt32:
        *(int32_t*)ptr = (int32_t)strtol(stringBuffer, NULL, 10);
        break;

    case DataTypeUInt32:
        *(uint32_t*)ptr = (uint32_t)strtoul(stringBuffer, NULL, 10);
        break;

    case DataTypeInt64:
        *(int64_t*)ptr = (int64_t)strtoll(stringBuffer, NULL, 10);
        break;

    case DataTypeUInt64:
        *(uint64_t*)ptr = (uint64_t)strtoull(stringBuffer, NULL, 10);
        break;

    case DataTypeFloat:
        *(float*)ptr = strtof(stringBuffer, NULL);
        break;

    case DataTypeDouble:
        *(double*)ptr = strtod(stringBuffer, NULL);
        break;

    case DataTypeString:
    {
        string s2(stringBuffer);
        s2.erase(std::remove(s2.begin(), s2.end(), '"'), s2.end());
        // ensure string fits with null terminator
        s2.resize(info.dataSize - 1);
        memcpy((void*)ptr, s2.data(), s2.length());
        memset((uint8_t*)ptr + s2.length(), 0, info.dataSize - s2.length());
    } break;

    case DataTypeBinary:
    {
        // convert hex data back to binary
        size_t len = _MIN(1020, stringLength);
        len -= (len % 2);
        uint8_t* ptr2 = (uint8_t*)ptr;
        for (size_t i = 0; i != len; )
        {
            *ptr2 = (getHexValue(stringBuffer[i++]) << 4) | getHexValue(stringBuffer[i++]);
        }
    } break;

    default:
        return false;
    }

    return true;
}


bool cISDataMappings::DataToString(const data_info_t& info, const p_data_hdr_t* hdr, const uint8_t* dataBuffer, data_mapping_string_t stringBuffer)
{
    const uint8_t* ptr;
    if (!CanGetFieldData(info, hdr, dataBuffer, ptr))
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
            stringBuffer[0] = '\0';
        }
        else
        {
            stringBuffer[0] = '0';
            stringBuffer[1] = '\0';
        }
        return false;
    }

    switch (info.dataType)
    {
    case DataTypeInt8:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int8_t*)ptr);
        break;

    case DataTypeUInt8:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint8_t*)ptr);
        break;

    case DataTypeInt16:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int16_t*)ptr);
        break;

    case DataTypeUInt16:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint16_t*)ptr);
        break;

    case DataTypeInt32:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int32_t*)ptr);
        break;

    case DataTypeUInt32:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint32_t*)ptr);
        break;

    case DataTypeInt64:
        SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%lld", (long long)*(int64_t*)ptr);
        break;

    case DataTypeUInt64:
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
        char* bufPtr2 = (char*)(dataBuffer + info.dataOffset);
        char* bufPtrEnd = bufPtr2 + _MIN(IS_DATA_MAPPING_MAX_STRING_LENGTH, info.dataSize) - 3;
        for (; bufPtr2 < bufPtrEnd && *bufPtr2 != '\0'; bufPtr2++)
        {
            stringBuffer[tempIndex++] = *bufPtr2;
        }
        stringBuffer[tempIndex++] = '"';
        stringBuffer[tempIndex] = '\0';
    } break;

    case DataTypeBinary:
    {
        // convert to hex
        const unsigned char* hexTable = getHexLookupTable();
        size_t hexIndex = 0;
        for (size_t i = 0; i < info.dataSize; i++)
        {
            stringBuffer[hexIndex++] = hexTable[0x0F & (dataBuffer[i] >> 4)];
            stringBuffer[hexIndex++] = hexTable[0x0F & dataBuffer[i]];
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

    static const string timestampFields[] = { "time", "timeOfWeek", "timeOfWeekMs", "seconds" };

    map_lookup_name_t::const_iterator offsetMap = cISDataMappings::GetMap().find(hdr->id);
    if (offsetMap == cISDataMappings::GetMap().end())
    {
        return 0.0;
    }
    const map_name_to_info_t& dataMap = offsetMap->second;
    for (size_t i = 0; i < _ARRAY_ELEMENT_COUNT(timestampFields); i++)
    {
        map_name_to_info_t::const_iterator timeStampField = dataMap.find(timestampFields[i]);
        if (timeStampField != dataMap.end())
        {
            const uint8_t* ptr;
            if (CanGetFieldData(timeStampField->second, hdr, (uint8_t*)buf, ptr))
            {
                if (timeStampField->second.dataType == DataTypeDouble)
                {
                    // field is seconds, use as is
                    return *(double*)ptr;
                }
                else if (timeStampField->second.dataType == DataTypeUInt32)
                {
                    // field is milliseconds, convert to seconds
                    return 0.001 * (*(uint32_t*)ptr);
                }
            }
            break;
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
