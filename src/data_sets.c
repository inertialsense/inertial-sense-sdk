/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "data_sets.h"
#include <stddef.h>
#include <math.h>

// prototype for checmsun32 function
uint32_t checksum32(const void* data, int count);

// Reversed bytes in a float.
// compiler will likely inline this as it's a tiny function
void flipFloat(uint8_t* ptr)
{
	uint8_t tmp1 = *ptr++;
	uint8_t tmp2 = *ptr++;
	uint8_t tmp3 = *ptr++;
	uint8_t tmp4 = *ptr;
	*ptr-- = tmp1;
	*ptr-- = tmp2;
	*ptr-- = tmp3;
	*ptr = tmp4;
}

float flipFloatCopy(float val)
{
	float flippedFloat;
	uint8_t* ptr = (uint8_t*)&val;
	uint8_t* ptr2 = (uint8_t*)&flippedFloat;
	uint8_t tmp1 = *ptr++;
	uint8_t tmp2 = *ptr++;
	uint8_t tmp3 = *ptr++;
	uint8_t tmp4 = *ptr;
	*ptr2++ = tmp4;
	*ptr2++ = tmp3;
	*ptr2++ = tmp2;
	*ptr2 = tmp1;
	return flippedFloat;
}

void flipDouble(uint8_t* ptr)
{
	const uint32_t* w = (const uint32_t*)(ptr);
	union
	{
		double v;
		uint32_t w[2];
	} u;
	u.w[0] = w[1];
	u.w[1] = w[0];
	*(double*)ptr = u.v;
}

double flipDoubleCopy(double val)
{
	union
	{
		double v;
		uint32_t w[2];
	} u;
	u.w[1] = SWAP32(*(uint32_t*)&val);
	u.w[0] = SWAP32(*((uint32_t*)&val + 1));
	return u.v;
}

void flipEndianess32(uint8_t* data, int dataLength)
{
	// data must be 4 byte aligned to swap endian-ness
	if (dataLength & 0x00000003)
	{
		return;
	}
	
	uint32_t* dataPtr = (uint32_t*)data;
	uint32_t* dataPtrEnd = (uint32_t*)(data + dataLength);
	uint32_t tmp;
	while (dataPtr < dataPtrEnd)
	{
		tmp = *dataPtr;
		*dataPtr++ = SWAP32(tmp);
	}
}

void flipDoubles(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength)
{
	uint16_t* doubleOffsetsEnd = offsets + offsetsLength;
	int offsetToDouble;
	int maxDoubleOffset = dataLength - 8;

	while (offsets < doubleOffsetsEnd)
	{
		offsetToDouble = (*offsets++) - offset;
		if (offsetToDouble >= 0 && offsetToDouble <= maxDoubleOffset)
		{
			flipDouble(data + offsetToDouble);
		}
	}
}

void flipStrings(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength)
{
	uint16_t* stringOffsetsEnd = offsets + offsetsLength;
	int offsetToString;
	int lengthOfString;
	int maxStringOffset;

	while (offsets < stringOffsetsEnd)
	{
		offsetToString = (*offsets++) - offset;
		lengthOfString = (*offsets++);
		maxStringOffset = dataLength - lengthOfString;
		if (offsetToString >= 0 && offsetToString <= maxStringOffset)
		{
			flipEndianess32(data + offsetToString, lengthOfString);
		}
	}
}

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable: 4267)

#endif

uint16_t* getDoubleOffsets(eDataIDs dataId, uint16_t* offsetsLength)
{
	/* Offset arrays contain:
	{
	array size,
	byte offset,
	byte offset,
	...
	} */

	// first offset is the number of offsets
	static uint16_t offsetsIns1[] =
	{
		4,
		OFFSETOF(ins_1_t, timeOfWeek),
		OFFSETOF(ins_1_t, lla[0]),
		OFFSETOF(ins_1_t, lla[1]),
		OFFSETOF(ins_1_t, lla[2])
	};

	static uint16_t offsetsIns2[] =
	{
		4,
		OFFSETOF(ins_2_t, timeOfWeek),
		OFFSETOF(ins_2_t, lla[0]),
		OFFSETOF(ins_2_t, lla[1]),
		OFFSETOF(ins_2_t, lla[2])
	};

	static uint16_t offsetsIns3[] =
	{
		4,
		OFFSETOF(ins_3_t, timeOfWeek),
		OFFSETOF(ins_3_t, lla[0]),
		OFFSETOF(ins_3_t, lla[1]),
		OFFSETOF(ins_3_t, lla[2])
	};

	static uint16_t offsetsIns4[] =
	{
		4,
		OFFSETOF(ins_4_t, timeOfWeek),
		OFFSETOF(ins_4_t, ecef[0]),
		OFFSETOF(ins_4_t, ecef[1]),
		OFFSETOF(ins_4_t, ecef[2])
	};

	static uint16_t offsetsGps[] =
	{
		4,
		OFFSETOF(gps_t, pos.lla[0]),
		OFFSETOF(gps_t, pos.lla[1]),
		OFFSETOF(gps_t, pos.lla[2]),
		OFFSETOF(gps_t, towOffset)
	};

	static uint16_t offsetsGpsPos[] =
	{
		3,
		OFFSETOF(gps_nav_poslla_t, lla[0]),
		OFFSETOF(gps_nav_poslla_t, lla[1]),
		OFFSETOF(gps_nav_poslla_t, lla[2])
	};

	static uint16_t offsetsInl2Status[] =
	{
		1, 24
	};

	static uint16_t offsetsInl2States[] =
	{
		4, 0, 36, 44, 52
	};

	static uint16_t offsetsInsMisc[] =
	{
		4,
		OFFSETOF(ins_misc_t, timeOfWeek),
		OFFSETOF(ins_misc_t, x.lla[0]),
		OFFSETOF(ins_misc_t, x.lla[1]),
		OFFSETOF(ins_misc_t, x.lla[2]),
	};

	static uint16_t offsetsInsRes[] =
	{
		3,
		OFFSETOF( ins_res_t, x_dot.lla[0] ),
		OFFSETOF( ins_res_t, x_dot.lla[1] ),
		OFFSETOF( ins_res_t, x_dot.lla[2] ),
	};

	static uint16_t offsetsRtkSol[] =
	{
		11,
		OFFSETOF(rtk_sol_t, seconds),
		OFFSETOF(rtk_sol_t, pos[0]),
		OFFSETOF(rtk_sol_t, pos[1]),
		OFFSETOF(rtk_sol_t, pos[2]),
		OFFSETOF(rtk_sol_t, vel[0]),
		OFFSETOF(rtk_sol_t, vel[1]),
		OFFSETOF(rtk_sol_t, vel[2]),
		OFFSETOF(rtk_sol_t, gdop),
		OFFSETOF(rtk_sol_t, pdop),
		OFFSETOF(rtk_sol_t, hdop),
		OFFSETOF(rtk_sol_t, vdop)
	};

	static uint16_t offsetsFlashConfig[] =
	{
		6,
		OFFSETOF( nvm_flash_cfg_t, refLla[0] ),
		OFFSETOF( nvm_flash_cfg_t, refLla[1] ),
		OFFSETOF( nvm_flash_cfg_t, refLla[2] ),
		OFFSETOF( nvm_flash_cfg_t, lastLla[0] ),
		OFFSETOF( nvm_flash_cfg_t, lastLla[1] ),
		OFFSETOF( nvm_flash_cfg_t, lastLla[2] )
	};

	// INTERNAL USE ONLY
	static uint16_t offsetsOnlyTimeFirst[] = { 1, 0 };
	static uint16_t offsetsInsParams[] = { 3, 156, 164, 172 };
	static uint16_t offsetsObsParams[] = { 6, 72, 80, 88, 100, 108, 116 };
	static uint16_t offsetsDebugArray[] = { 3, 72, 80, 88 };

    static uint16_t* s_doubleOffsets[] =
	{
		0,						// DID_NULL
		0,						// DID_DEV_INFO
		offsetsOnlyTimeFirst,	// DID_IMU_1
		offsetsOnlyTimeFirst,	// DID_CON_SCUL_INT
		offsetsIns1,			// DID_INS_1
		offsetsIns2,			// DID_INS_2
		offsetsGps,				// DID_GPS
		0,						// DID_CONFIG
		0,						// DID_ASCII_BCAST_PERIOD
		offsetsInsMisc,			// DID_INS_MISC
		0,						// DID_SYS_PARAMS
		offsetsOnlyTimeFirst,	// DID_SYS_SENSORS
		offsetsFlashConfig,		// DID_FLASH_CONFIG
		0,						// DID_GPS_CNO
		offsetsGpsPos,			// DID_GPS_POS
		0,						// DID_GPS_VEL
		0,						// DID_IO
		0,						// DID_IO_SERVOS_PWM
		0,						// DID_IO_SERVOS_PPM
		0,						// DID_MAG_CAL
		offsetsInsRes,			// DID_INS_RESOURCES
        0,                      // DID_DGPS_CORRECTION
        offsetsRtkSol,          // DID_RTK_SOL,
		0,						// DID_FEATURE_BITS
		0,						// DID_SENSORS_IS1
		0,						// DID_SENSORS_IS2
		0,						// DID_SENSORS_TC_BIAS
		0,						// DID_SENSORS_CF_BIAS
		offsetsOnlyTimeFirst,	// DID_SENSORS_ADC
		0,						// DID_SCOMP
		offsetsInsParams,		// DID_INS_PARAMS
		offsetsObsParams,		// DID_OBS_PARAMS
		0,						// DID_HDW_PARAMS
		0,						// DID_NVR_MANAGE_USERPAGE
		0,						// DID_NVR_USERPAGE_SN
		0,						// DID_NVR_USERPAGE_G0
		0,						// DID_NVR_USERPAGE_G1
		0,						// DID_NVR_MANAGE_PROTECTED
		0,						// DID_RTOS_INFO
		offsetsDebugArray,		// DID_DEBUG_ARRAY
		0,						// DID_SENSORS_CAL1
		0,						// DID_SENSORS_CAL2
		0,						// DID_CAL_SC
		0,						// DID_CAL_SC1
		0,						// DID_CAL_SC2
		offsetsOnlyTimeFirst,	// DID_SYS_SENSORS_SIGMA
		offsetsOnlyTimeFirst,	// DID_SENSORS_ADC_SIGMA
		0,                      // DID_INS_DEV_1
		offsetsInl2States,      // DID_INL2_STATES
		0,                      // DID_INL2_COVARIANCE_UD
		0,                      // DID_INL2_MISC
		0,                      // DID_INL2_STATUS,
		offsetsOnlyTimeFirst,	// DID_MAGNETOMETER_1
		offsetsOnlyTimeFirst,	// DID_BAROMETER
		offsetsOnlyTimeFirst,	// DID_IMU_2
		offsetsOnlyTimeFirst,	// DID_MAGNETOMETER_2
		0,                      // DID_GPS_VERSION
		0,						// DID_COMMUNICATIONS_LOOPBACK
		offsetsOnlyTimeFirst,	// DID_DUAL_IMU
		0,						// DID_INL2_MAG_OBS_INFO
        0,						// DID_RAW_GPS_DATA
        0,                      // DID_RTK_OPT
        0,                      // DID_NVR_USERPAGE_INTERNAL
		0,						// DID_MANUFACTURING_INFO
		0,                      // DID_BIT
		offsetsIns3,			// DID_INS_3
		offsetsIns4,			// DID_INS_4
		0,						// DID_INL2_VARIANCE
	};

    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(s_doubleOffsets) == DID_COUNT);

	if (dataId < DID_COUNT)
	{
        uint16_t* offsets;
        if ((offsets = s_doubleOffsets[dataId]))
        {
            *offsetsLength = (*offsets++);
            return offsets;
        }
    }
	return 0;
}

#ifdef _MSC_VER

#pragma warning(pop)

#endif

uint16_t* getStringOffsetsLengths(eDataIDs dataId, uint16_t* offsetsLength)
{
	/* Offset arrays contain:
	{
		array size, // number of pairs
		byte offset, byte size,	// 1st pair
		byte offset, byte size,	// 2nd pair
		...
	} */

	static uint16_t debugStringOffsets[] = { 2, 0, 80 };

	static uint16_t rtosTaskOffsets[] =
	{
		12,
		0, MAX_TASK_NAME_LEN,
		32, MAX_TASK_NAME_LEN,
		64, MAX_TASK_NAME_LEN,
		96, MAX_TASK_NAME_LEN,
		128, MAX_TASK_NAME_LEN,
		160, MAX_TASK_NAME_LEN
	};

	static uint16_t manufInfoOffsets[] =
	{
		2,
		OFFSETOF(manufacturing_info_t, date), _MEMBER_ARRAY_ELEMENT_COUNT(manufacturing_info_t, date)
	};

    static uint16_t* s_stringOffsets[] =
	{
		0,						// DID_NULL
        0,						// DID_DEV_INFO
		0,						// DID_IMU_1
		0,						// DID_CON_SCUL_INT
		0,						// DID_INS_1
		0,						// DID_INS_2
		0,						// DID_GPS
		0,						// DID_CONFIG
		0,						// DID_ASCII_BCAST_PERIOD
		0,						// DID_INS_MISC
		0,						// DID_SYS_PARAMS
		0,						// DID_SYS_SENSORS
		0,						// DID_FLASH_CONFIG
		0,						// DID_GPS_CNO
		0,						// DID_GPS_POS
		0,						// DID_GPS_VEL
		0,						// DID_IO
		0,						// DID_IO_SERVOS_PWM
		0,						// DID_IO_SERVOS_PPM
		0,						// DID_MAG_CAL
		0,						// DID_INS_RESOURCES
        0,                      // DID_DGPS_CORRECTION
        0,                      // DID_RTK_SOL,
		0,						// DID_FEATURE_BITS
		0,						// DID_SENSORS_IS1
		0,						// DID_SENSORS_IS2
		0,						// DID_SENSORS_TC_BIAS
		0,						// DID_SENSORS_CF_BIAS
		0,						// DID_SENSORS_ADC
		0,						// DID_SCOMP
		0,						// DID_INS_PARAMS,
		0,						// DID_OBS_PARAMS,
		0,						// DID_HDW_PARAMS,
		0,						// DID_NVR_MANAGE_USERPAGE
		0,						// DID_NVR_USERPAGE_SN
		0,						// DID_NVR_USERPAGE_G0
		0,						// DID_NVR_USERPAGE_G1
		debugStringOffsets,		// DID_DEBUG_STRING
		rtosTaskOffsets,		// DID_RTOS_INFO
		0,						// DID_DEBUG_ARRAY
		0,						// DID_SENSORS_CAL1
		0,						// DID_SENSORS_CAL2
		0,						// DID_CAL_SC
		0,						// DID_CAL_SC1
		0,						// DID_CAL_SC2
		0,						// DID_SYS_SENSORS_SIGMA
		0,						// DID_SENSORS_ADC_SIGMA
		0,                      // DID_INS_DEV_1
		0,                      // DID_INL2_STATES
		0,                      // DID_INL2_COVARIANCE_UD
		0,                      // DID_INL2_MISC
		0,                      // DID_INL2_STATUS
		0,						// DID_MAGNETOMETER_1
		0,						// DID_BAROMETER
		0,						// DID_IMU_2
		0,						// DID_MAGNETOMETER_2
		0,						// DID_GPS_VERSION
		0,						// DID_COMMUNICATIONS_LOOPBACK
		0,						// DID_DUAL_IMU
		0,						// DID_INL2_MAG_OBS_INFO
        0,						// DID_RAW_GPS_DATA
        0,                      // DID_RTK_OPT
        0,                      // DID_NVR_USERPAGE_INTERNAL
		manufInfoOffsets,		// DID_MANUFACTURING_INFO
		0,                      // DID_BIT
		0,                      // DID_INS_3
		0,                      // DID_INS_4
		0,						// DID_INL2_VARIANCE
	};

    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(s_stringOffsets) == DID_COUNT);

    if (dataId < DID_COUNT)
	{
        uint16_t* offsets;
        if ((offsets = s_stringOffsets[dataId]))
        {
            *offsetsLength = (*offsets++);
            return offsets;
        }
    }
	return 0;
}

uint32_t checksum32(const void* data, int count)
{
	if (count % 4 != 0 || count < 0)
	{
		return 0;
	}
	
	uint32_t checksum = 0;
	uint32_t* dataPtr = (uint32_t*)data;
	uint32_t* dataEnd = dataPtr + (count / 4);
	
	while (dataPtr < dataEnd)
	{
		checksum ^= *dataPtr++;
	}
	
	return checksum;
}

// This function skips the first 4 bytes (one 4 byte word), which are assumed to be the checksum in the serial number flash memory data structure.
uint32_t serialNumChecksum32(const void* data, int size)
{
	return checksum32((const uint8_t*)data + 4, size - 4);
}

// This function skips the first 8 bytes (two 4 byte words), which are assumed to be the size and checksum in flash memory data structures.
uint32_t flashChecksum32(const void* data, int size)
{
	return checksum32((const uint8_t*)data + 8, size - 8);
}

int32_t convertDateToMjd(int32_t year, int32_t month, int32_t day)
{
	return
		367 * year
		- 7 * (year + (month + 9) / 12) / 4
		- 3 * ((year + (month - 9) / 7) / 100 + 1) / 4
		+ 275 * month / 9
		+ day
		+ 1721028
		- 2400000;
}

int32_t convertGpsToMjd(int32_t gpsCycle, int32_t gpsWeek, int32_t gpsSeconds)
{
	uint32_t gpsDays = ((gpsCycle * 1024) + gpsWeek) * 7 + (gpsSeconds / 86400);
	return convertDateToMjd(1980, 1, 6) + gpsDays;
}

void convertMjdToDate(int32_t mjd, int32_t* year, int32_t* month, int32_t* day)
{
	int32_t j, c, y, m;

	j = mjd + 2400001 + 68569;
	c = 4 * j / 146097;
	j = j - (146097 * c + 3) / 4;
	y = 4000 * (j + 1) / 1461001;
	j = j - 1461 * y / 4 + 31;
	m = 80 * j / 2447;
	*day = j - 2447 * m / 80;
	j = m / 11;
	*month = m + 2 - (12 * j);
	*year = 100 * (c - 49) + y + j;
}

void convertGpsToHMS(int32_t gpsSeconds, int32_t* hour, int32_t* minutes, int32_t* seconds)
{
	// shave off days
	gpsSeconds = gpsSeconds % 86400;

	// compute hours, minutes, seconds
	*hour = gpsSeconds / 3600;
	*minutes = (gpsSeconds / 60) % 60;
	*seconds = gpsSeconds % 60;
}

gen_1axis_sensor_t gen1AxisSensorData(double time, const float val)
{
    gen_1axis_sensor_t data;
    data.time = time;
    data.val = val;
    return data;
}

gen_3axis_sensor_t gen3AxisSensorData(double time, const float val[3])
{
    gen_3axis_sensor_t data;
    data.time = time;
    data.val[0] = val[0];
    data.val[1] = val[1];
    data.val[2] = val[2];
    return data;
}

gen_dual_3axis_sensor_t genDual3AxisSensorData(double time, const float val1[3], const float val2[3])
{
	gen_dual_3axis_sensor_t data;
	data.time = time;
	data.val1[0] = val1[0];
	data.val1[1] = val1[1];
	data.val1[2] = val1[2];
	data.val2[0] = val2[0];
	data.val2[1] = val2[1];
	data.val2[2] = val2[2];
	return data;
}

gen_3axis_sensord_t gen3AxisSensorDataD(double time, const double val[3])
{
    gen_3axis_sensord_t data;
    data.time = time;
    data.val[0] = val[0];
    data.val[1] = val[1];
    data.val[2] = val[2];
    return data;
}
