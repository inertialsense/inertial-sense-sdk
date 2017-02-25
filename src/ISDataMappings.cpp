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

#if !_MSC_VER || _MSC_VER >= 1900

#define ADD_MAP_NO_VALIDATION(map, totalSize, name, type, member, dataSize, dataType, memberType) map[name] = { (uint32_t)OFFSETOF(type, member), (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize), dataType }; totalSize += sizeof(type::member);

constexpr uint32_t eDataTypeSizes[11] =
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
	(uint32_t)0
};

// note when passing member type for arrays, it must be a reference, i.e. float&
#define ADD_MAP(map, totalSize, re, type, member, dataSize, dataType, memberType) \
	ADD_MAP_NO_VALIDATION(map, totalSize, re, type, member, dataSize, dataType, memberType); \
	static_assert(is_same<decltype(type::member), memberType>::value, "Member type is an unexpected type"); \
	static_assert((uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == sizeof(memberType), "Member type is an unexpected size"); \
	static_assert((uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == sizeof(type::member), "Member type is an unexpected size"); \
	static_assert(eDataTypeSizes[dataType] == 0 || (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize) == eDataTypeSizes[dataType], "Data type size does not match member size");
#define ASSERT_SIZE(s, t) assert(s == sizeof(t))

#else

#define ADD_MAP_NO_VALIDATION(map, totalSize, name, type, member, dataSize, dataType, memberType) map[name] = { (uint32_t)OFFSETOF(type, member), (uint32_t)(dataSize == 0 ? sizeof(memberType) : dataSize), dataType };
#define ADD_MAP(map, totalSize, re, type, member, dataSize, dataType, memberType) ADD_MAP_NO_VALIDATION(map, totalSize, re, type, member, dataSize, dataType, memberType)
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
	sizeMap[DID_GPS] = sizeof(gps_t);
	sizeMap[DID_CONFIG] = sizeof(config_t);
	sizeMap[DID_GPS_POS] = sizeof(gps_nav_poslla_t);
	sizeMap[DID_GPS_VEL] = sizeof(gps_nav_velned_t);
	sizeMap[DID_GPS_RSSI] = sizeof(gps_rssi_t);
	sizeMap[DID_INS_MISC] = sizeof(ins_misc_t);
	sizeMap[DID_SYS_PARAMS] = sizeof(sys_params_t);
	sizeMap[DID_SYS_SENSORS] = sizeof(sys_sensors_t);
	sizeMap[DID_FLASH_CONFIG] = sizeof(nvm_flash_cfg_t);
	sizeMap[DID_INS_RESOURCES] = sizeof(ins_res_t);

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
	sizeMap[DID_EKF_STATES] = sizeof(ekf_states_t);

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
	ADD_MAP(m, totalSize, "iStatus", MAP_TYPE, iStatus, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "hStatus", MAP_TYPE, hStatus, 0, DataTypeUInt32, uint32_t);
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
	ADD_MAP(m, totalSize, "iStatus", MAP_TYPE, iStatus, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "hStatus", MAP_TYPE, hStatus, 0, DataTypeUInt32, uint32_t);
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

static void PopulateINS2Mappings(map_lookup_name_t& mappings)
{
	typedef ins_2_t MAP_TYPE;
	map_name_to_info_t m;
	uint32_t totalSize = 0;
	ADD_MAP(m, totalSize, "week", MAP_TYPE, week, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "timeOfWeek", MAP_TYPE, timeOfWeek, 0, DataTypeDouble, double);
	ADD_MAP(m, totalSize, "iStatus", MAP_TYPE, iStatus, 0, DataTypeUInt32, uint32_t);
	ADD_MAP(m, totalSize, "hStatus", MAP_TYPE, hStatus, 0, DataTypeUInt32, uint32_t);
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
	mappings[DID_INS_2] = m;

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

static void PopulateGPSRSSIMappings(map_lookup_name_t& mappings)
{
	typedef gps_rssi_t MAP_TYPE;
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
	mappings[DID_GPS_RSSI] = m;

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

#if 0

	ADD_MAP(m, totalSize, "bias_cal[0]", MAP_TYPE, bias_cal[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "bias_cal[1]", MAP_TYPE, bias_cal[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "bias_cal[2]", MAP_TYPE, bias_cal[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[0]", MAP_TYPE, Wcal[0], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[1]", MAP_TYPE, Wcal[1], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[2]", MAP_TYPE, Wcal[2], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[3]", MAP_TYPE, Wcal[3], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[4]", MAP_TYPE, Wcal[4], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[5]", MAP_TYPE, Wcal[5], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[6]", MAP_TYPE, Wcal[6], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[7]", MAP_TYPE, Wcal[7], 0, DataTypeFloat, float&);
	ADD_MAP(m, totalSize, "Wcal[8]", MAP_TYPE, Wcal[8], 0, DataTypeFloat, float&);

#endif

	mappings[DID_NVR_USERPAGE_G1] = m;

	ASSERT_SIZE(totalSize, MAP_TYPE);
}

static void PopulateEKFStatesMappings(map_lookup_name_t& mappings)
{
	typedef ekf_states_t MAP_TYPE;
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
	mappings[DID_EKF_STATES] = m;

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
	PopulateINS2Mappings(m_columnMappings);
	PopulateGPSMappings(m_columnMappings);
	PopulateGPSPosMappings(m_columnMappings);
	PopulateGPSVelocityMappings(m_columnMappings);
	PopulateGPSRSSIMappings(m_columnMappings);
	PopulateMagnetometerMappings(m_columnMappings);
	PopulateBarometerMappings(m_columnMappings);
	PopulateDeltaThetaVelocityMappings(m_columnMappings);
	PopulateINSMiscMappings(m_columnMappings);
	PopulateFlashConfigMappings(m_columnMappings);
	PopulateINSResourcesMappings(m_columnMappings);

#ifdef USE_IS_INTERNAL

	PopulateSensorsADCMappings(m_columnMappings);
	PopulateSensorsISMappings(m_columnMappings);
	PopulateSensorsTCMappings(m_columnMappings);
	PopulateSensorsCompMappings(m_columnMappings);
	PopulateDebugArrayMappings(m_columnMappings);
	PopulateUserPage0Mappings(m_columnMappings);
	PopulateUserPage1Mappings(m_columnMappings);
	PopulateEKFStatesMappings(m_columnMappings);

#endif

}

const char* cISDataMappings::GetDataSetName(uint32_t dataId)
{
	switch (dataId)
	{
	case DID_DEV_INFO:			return "devInfo";
	case DID_IMU_1:				return "imu1";
	case DID_IMU_2:				return "imu2";
	case DID_MAGNETOMETER_1:	return "magnetometer1";
	case DID_MAGNETOMETER_2:	return "magnetometer2";
	case DID_BAROMETER:			return "barometer";
	case DID_DELTA_THETA_VEL:	return "dThetaVel";
	case DID_INS_1:				return "ins1";
	case DID_INS_2:				return "ins2";
	case DID_GPS:				return "gps";
	case DID_CONFIG:			return "config";
	case DID_GPS_POS:			return "gpsPos";
	case DID_GPS_VEL:			return "gpsVel";
	case DID_GPS_RSSI:			return "gpsRSSI";
	case DID_INS_MISC:			return "misc";
	case DID_SYS_PARAMS:		return "sysParams";
	case DID_SYS_SENSORS:		return "sysSensors";
	case DID_FLASH_CONFIG:		return "flashCfg";
	case DID_INS_RESOURCES:		return "insRes";

#ifdef USE_IS_INTERNAL

	case DID_SENSORS_ADC:		return "sensorLSB";
	case DID_SENSORS_IS1:		return "sensorIS1";
	case DID_SENSORS_IS2:		return "sensorIS2";
	case DID_SENSORS_TC_BIAS:	return "sensorTCbias";
	case DID_SENSORS_CF_BIAS:	return "sensorCFbias";
	case DID_SCOMP:				return "scomp";
	case DID_DEBUG_ARRAY:		return "debugArray";
	case DID_NVR_USERPAGE_G0:    return "userpage0";
	case DID_NVR_USERPAGE_G1:    return "userpage1";
	case DID_EKF_STATES:		return "ekfstates";

#endif

	default:					return NULL;
	}
}


const map_lookup_name_t& cISDataMappings::GetMap()
{
	return s_map.m_columnMappings;
}


uint32_t cISDataMappings::GetSize(uint32_t dataId)
{
	return s_map.m_lookupSize[dataId];
}


bool cISDataMappings::StringToData(const char* stringBuffer, uint8_t* buf, const data_info_t& info)
{
	switch (info.dataType)
	{
	case DataTypeInt8:
		*(int8_t*)(buf + info.dataOffset) = (int8_t)strtol(stringBuffer, NULL, 10);
		break;

	case DataTypeUInt8:
		*(uint8_t*)(buf + info.dataOffset) = (uint8_t)strtoul(stringBuffer, NULL, 10);
		break;

	case DataTypeInt16:
		*(int16_t*)(buf + info.dataOffset) = (int16_t)strtol(stringBuffer, NULL, 10);
		break;

	case DataTypeUInt16:
		*(uint16_t*)(buf + info.dataOffset) = (uint16_t)strtoul(stringBuffer, NULL, 10);
		break;

	case DataTypeInt32:
		*(int32_t*)(buf + info.dataOffset) = (int32_t)strtol(stringBuffer, NULL, 10);
		break;

	case DataTypeUInt32:
		*(uint32_t*)(buf + info.dataOffset) = (uint32_t)strtoul(stringBuffer, NULL, 10);
		break;

	case DataTypeInt64:
		*(int64_t*)(buf + info.dataOffset) = (int64_t)strtoll(stringBuffer, NULL, 10);
		break;

	case DataTypeUInt64:
		*(uint64_t*)(buf + info.dataOffset) = (uint64_t)strtoull(stringBuffer, NULL, 10);
		break;

	case DataTypeFloat:
		*(float*)(buf + info.dataOffset) = strtof(stringBuffer, NULL);
		break;

	case DataTypeDouble:
		*(double*)(buf + info.dataOffset) = strtod(stringBuffer, NULL);
		break;

	case DataTypeString:
	{
		string s2(stringBuffer);
		s2.erase(std::remove(s2.begin(), s2.end(), '"'), s2.end());
		// ensure string fits with null terminator
		s2.resize(info.dataSize - 1);
		uint8_t* baseOffset = buf + info.dataOffset;
		memcpy(baseOffset, s2.data(), s2.length());
		memset(baseOffset + s2.length(), 0, info.dataSize - s2.length());
	} break;

	default:
		return false;
	}

	return true;
}


bool cISDataMappings::DataToString(const data_info_t& info, const uint8_t* dataBuffer, char stringBuffer[IS_DATA_MAPPING_MAX_STRING_LENGTH])
{
	switch (info.dataType)
	{
	case DataTypeInt8:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int8_t*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeUInt8:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint8_t*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeInt16:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int16_t*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeUInt16:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint16_t*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeInt32:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%d", (int)*(int32_t*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeUInt32:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%u", (unsigned int)*(uint32_t*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeInt64:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%lld", (long long)*(int64_t*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeUInt64:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%llu", (unsigned long long)*(uint64_t*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeFloat:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.9g", *(float*)(dataBuffer + info.dataOffset));
		break;

	case DataTypeDouble:
		SNPRINTF(stringBuffer, IS_DATA_MAPPING_MAX_STRING_LENGTH, "%.17g", *(double*)(dataBuffer + info.dataOffset));
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
	default:
		return false;
	}
	return true;
}
