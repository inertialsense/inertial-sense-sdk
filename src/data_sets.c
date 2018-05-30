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
    int isDouble;
	while (offsets < doubleOffsetsEnd)
	{
        offsetToDouble = (*offsets++);
        isDouble = ((offsetToDouble & 0x8000) == 0);
        offsetToDouble = (offsetToDouble & 0x7FFF) - offset;
		if (offsetToDouble >= 0 && offsetToDouble <= maxDoubleOffset)
		{
            if (isDouble)
            {
                flipDouble(data + offsetToDouble);
            }
            else
            {
                uint64_t* ptr = (uint64_t*)(data + offsetToDouble);
                *ptr = SWAP64(*ptr);
            }
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
		offsetof(ins_1_t, timeOfWeek),
		offsetof(ins_1_t, lla[0]),
		offsetof(ins_1_t, lla[1]),
		offsetof(ins_1_t, lla[2])
	};

	static uint16_t offsetsIns2[] =
	{
		4,
		offsetof(ins_2_t, timeOfWeek),
		offsetof(ins_2_t, lla[0]),
		offsetof(ins_2_t, lla[1]),
		offsetof(ins_2_t, lla[2])
	};

	static uint16_t offsetsIns3[] =
	{
		4,
		offsetof(ins_3_t, timeOfWeek),
		offsetof(ins_3_t, lla[0]),
		offsetof(ins_3_t, lla[1]),
		offsetof(ins_3_t, lla[2])
	};

	static uint16_t offsetsIns4[] =
	{
		4,
		offsetof(ins_4_t, timeOfWeek),
		offsetof(ins_4_t, ecef[0]),
		offsetof(ins_4_t, ecef[1]),
		offsetof(ins_4_t, ecef[2])
	};

	static uint16_t offsetsGps[] =
	{
		7,
		offsetof(gps_nav_t, lla[0]),
		offsetof(gps_nav_t, lla[1]),
		offsetof(gps_nav_t, lla[2]),
		offsetof(gps_nav_t, towOffset),
		offsetof(gps_nav_t, ecef[0]),
		offsetof(gps_nav_t, ecef[1]),
		offsetof(gps_nav_t, ecef[2])
	};

    static uint16_t offsetsRmc[] =
    {
        1, 
        // 0x8000 denotes a 64 bit int vs a double
		offsetof(rmc_t, bits) | 0x8000
    };

	static uint16_t offsetsInl2Status[] =
	{
		1, 24
	};

	static uint16_t offsetsInl2States[] =
	{
		4, 0, 36, 44, 52
	};

	static uint16_t offsetsRtkNav[] =
	{
		3,
		offsetof(gps_rtk_misc_t, baseLla[0]),
		offsetof(gps_rtk_misc_t, baseLla[1]),
		offsetof(gps_rtk_misc_t, baseLla[2]),
	};

	static uint16_t offsetsFlashConfig[] =
	{
		6,
		offsetof( nvm_flash_cfg_t, refLla[0] ),
		offsetof( nvm_flash_cfg_t, refLla[1] ),
		offsetof( nvm_flash_cfg_t, refLla[2] ),
		offsetof( nvm_flash_cfg_t, lastLla[0] ),
		offsetof( nvm_flash_cfg_t, lastLla[1] ),
		offsetof( nvm_flash_cfg_t, lastLla[2] )
	};

	static uint16_t offsetsOnlyTimeFirst[] = { 1, 0 };
	static uint16_t offsetsDebugArray[] = { 3, 72, 80, 88 };

    static uint16_t* s_doubleOffsets[] =
	{
		0,						//  0: DID_NULL
		0,						//  1: DID_DEV_INFO
        0,						//  2: DID_CRASH_INFO
		offsetsOnlyTimeFirst,	//  3: DID_PREINTEGRATED_IMU
		offsetsIns1,			//  4: DID_INS_1
		offsetsIns2,			//  5: DID_INS_2
		offsetsGps,				//  6: DID_GPS_NAV
        0,  					//  7: DID_CONFIG
		0,						//  8: DID_ASCII_BCAST_PERIOD
		offsetsRmc,				//  9: DID_RMC
		0,						// 10: DID_SYS_PARAMS
		offsetsOnlyTimeFirst,	// 11: DID_SYS_SENSORS
		offsetsFlashConfig,		// 12: DID_FLASH_CONFIG
		offsetsGps,				// 13: DID_GPS1_NAV
		offsetsGps,				// 14: DID_GPS2_NAV
		0,						// 15: DID_GPS1_SAT
		0,						// 16: DID_GPS2_SAT
		0,                      // 17: DID_GPS1_VERSION
		0,						// 18: DID_GPS2_VERSION
		0,						// 19: DID_MAG_CAL
		0,						// 20: 
        0,                      // 21: DID_GPS_RTK_NAV
        offsetsRtkNav,          // 22: DID_GPS_RTK_MISC,
		0,						// 23: DID_FEATURE_BITS
		0,						// 24: DID_SENSORS_IS1
		0,						// 25: DID_SENSORS_IS2
		0,						// 26: DID_SENSORS_TC_BIAS
		0,						// 27: DID_IO
		offsetsOnlyTimeFirst,	// 28: DID_SENSORS_ADC
		0,						// 29: DID_SCOMP
		0,						// 30: 
		0,						// 31: 
		0,						// 32: DID_HDW_PARAMS
		0,						// 33: DID_NVR_MANAGE_USERPAGE
		0,						// 34: DID_NVR_USERPAGE_SN
		0,						// 35: DID_NVR_USERPAGE_G0
		0,						// 36: DID_NVR_USERPAGE_G1
		0,						// 37: DID_NVR_MANAGE_PROTECTED
		0,						// 38: DID_RTOS_INFO
		offsetsDebugArray,		// 39: DID_DEBUG_ARRAY
		0,						// 40: DID_SENSORS_CAL1
		0,						// 41: DID_SENSORS_CAL2
		0,						// 42: DID_CAL_SC
		0,						// 43: DID_CAL_SC1
		0,						// 44: DID_CAL_SC2
		offsetsOnlyTimeFirst,	// 45: DID_SYS_SENSORS_SIGMA
		offsetsOnlyTimeFirst,	// 46: DID_SENSORS_ADC_SIGMA
		0,                      // 47: DID_INS_DEV_1
		offsetsInl2States,      // 48: DID_INL2_STATES
		0,                      // 49: DID_INL2_COVARIANCE_LD
		0,                      // 50: DID_INL2_MISC
		0,                      // 51: DID_INL2_STATUS,
		offsetsOnlyTimeFirst,	// 52: DID_MAGNETOMETER_1
		offsetsOnlyTimeFirst,	// 53: DID_BAROMETER
		0,						// 54: 
		offsetsOnlyTimeFirst,	// 55: DID_MAGNETOMETER_2
		0,						// 56: DID_COMMUNICATIONS_LOOPBACK
		offsetsOnlyTimeFirst,	// 57: DID_DUAL_IMU_RAW
		offsetsOnlyTimeFirst,	// 58: DID_DUAL_IMU
		0,						// 59: DID_INL2_MAG_OBS_INFO
        0,						// 60: DID_GPS_BASE_RAW
        0,                      // 61: DID_GPS_RTK_OPT
        0,                      // 62: DID_NVR_USERPAGE_INTERNAL
		0,						// 63: DID_MANUFACTURING_INFO
		0,                      // 64: DID_BIT
		offsetsIns3,			// 65: DID_INS_3
		offsetsIns4,			// 66: DID_INS_4
		0,						// 67: DID_INL2_VARIANCE
		0,						// 68: DID_STROBE_IN_TIME
		0,						// 69: DID_GPS1_RAW
		0,						// 70: DID_GPS2_RAW
		0,						// 71: DID_VELOCITY_SENSOR
		0,						// 72: DID_DIAGNOSTIC_MESSAGE
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
		offsetof(manufacturing_info_t, date), _MEMBER_ARRAY_ELEMENT_COUNT(manufacturing_info_t, date)
	};
	
	static uint16_t diagMsgOffsets[] =
	{
		2,
		offsetof(diag_msg_t, message), _MEMBER_ARRAY_ELEMENT_COUNT(diag_msg_t, message)
	};

    static uint16_t* s_stringOffsets[] =
	{
		0,						//  0: DID_NULL
        0,						//  1: DID_DEV_INFO
        0,						//  2: DID_CRASH_INFO
		0,						//  3: DID_PREINTEGRATED_IMU
		0,						//  4: DID_INS_1
		0,						//  5: DID_INS_2
		0,						//  6: DID_GPS_NAV
		0,						//  7: DID_CONFIG
		0,						//  8: DID_ASCII_BCAST_PERIOD
		0,						//  9: DID_RMC
		0,						// 10: DID_SYS_PARAMS
		0,						// 11: DID_SYS_SENSORS
		0,						// 12: DID_FLASH_CONFIG
		0,						// 13: DID_GPS1_NAV
		0,						// 14: DID_GPS2_NAV
		0,						// 15: DID_GPS1_SAT
		0,						// 16: DID_GPS2_SAT
		0,						// 17: DID_GPS1_VERSION
		0,						// 18: DID_GPS2_VERSION
		0,						// 19: DID_MAG_CAL
		0,						// 20: 
        0,                      // 21: DID_GPS_RTK_NAV
        0,                      // 22: DID_GPS_RTK_MISC,
		0,						// 23: DID_FEATURE_BITS
		0,						// 24: DID_SENSORS_IS1
		0,						// 25: DID_SENSORS_IS2
		0,						// 26: DID_SENSORS_TC_BIAS
		0,						// 27: DID_IO
		0,						// 28: DID_SENSORS_ADC
		0,						// 29: DID_SCOMP
		0,						// 30: 
		0,						// 31: 
		0,						// 32: DID_HDW_PARAMS,
		0,						// 33: DID_NVR_MANAGE_USERPAGE
		0,						// 34: DID_NVR_USERPAGE_SN
		0,						// 35: DID_NVR_USERPAGE_G0
		0,						// 36: DID_NVR_USERPAGE_G1
		debugStringOffsets,		// 37: DID_DEBUG_STRING
		rtosTaskOffsets,		// 38: DID_RTOS_INFO
		0,						// 39: DID_DEBUG_ARRAY
		0,						// 40: DID_SENSORS_CAL1
		0,						// 41: DID_SENSORS_CAL2
		0,						// 42: DID_CAL_SC
		0,						// 43: DID_CAL_SC1
		0,						// 44: DID_CAL_SC2
		0,						// 45: DID_SYS_SENSORS_SIGMA
		0,						// 46: DID_SENSORS_ADC_SIGMA
		0,                      // 47: DID_INS_DEV_1
		0,                      // 48: DID_INL2_STATES
		0,                      // 49: DID_INL2_COVARIANCE_LD
		0,                      // 50: DID_INL2_MISC
		0,                      // 51: DID_INL2_STATUS
		0,						// 52: DID_MAGNETOMETER_1
		0,						// 53: DID_BAROMETER
		0,						// 54: 
		0,						// 55: DID_MAGNETOMETER_2
		0,						// 56: DID_COMMUNICATIONS_LOOPBACK
		0,						// 57: DID_DUAL_IMU_RAW
		0,						// 58: DID_DUAL_IMU
		0,						// 59: DID_INL2_MAG_OBS_INFO
        0,						// 60: DID_GPS_BASE_RAW
        0,                      // 61: DID_GPS_RTK_OPT
        0,                      // 62: DID_NVR_USERPAGE_INTERNAL
		manufInfoOffsets,		// 63: DID_MANUFACTURING_INFO
		0,                      // 64: DID_BIT
		0,                      // 65: DID_INS_3
		0,                      // 66: DID_INS_4
		0,						// 67: DID_INL2_VARIANCE
		0,						// 68: DID_STROBE_IN_TIME
		0,						// 69: DID_GPS1_RAW
		0,						// 70: DID_GPS2_RAW
		0,						// 71: DID_VELOCITY_SENSOR
		diagMsgOffsets, 		// 72: DID_DIAGNOSTIC_MESSAGE
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
	if (count < 1 || count % 4 != 0)
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

// Convert DID to message out control mask
uint64_t didToRmcBits(uint32_t dataId, uint64_t defaultRmcBits)
{
	switch (dataId)
	{
		case DID_INS_1:					return RMC_BITS_INS1;
		case DID_INS_2:					return RMC_BITS_INS2;
		case DID_INS_3:					return RMC_BITS_INS3;
		case DID_INS_4:					return RMC_BITS_INS4;
		case DID_DUAL_IMU_RAW:			return RMC_BITS_DUAL_IMU_RAW;
		case DID_DUAL_IMU:				return RMC_BITS_DUAL_IMU;
		case DID_PREINTEGRATED_IMU:		return RMC_BITS_PREINTEGRATED_IMU;
		case DID_BAROMETER:				return RMC_BITS_BAROMETER;
		case DID_MAGNETOMETER_1:		return RMC_BITS_MAGNETOMETER1;
		case DID_MAGNETOMETER_2:		return RMC_BITS_MAGNETOMETER2;
		case DID_GPS_NAV:				return RMC_BITS_GPS_NAV;
		case DID_GPS1_NAV:				return RMC_BITS_GPS1_NAV;
		case DID_GPS2_NAV:				return RMC_BITS_GPS2_NAV;
		case DID_GPS1_SAT:				return RMC_BITS_GPS1_SAT;
		case DID_GPS2_SAT:				return RMC_BITS_GPS2_SAT;
		case DID_GPS1_RAW:				return RMC_BITS_GPS1_RAW;
		case DID_GPS2_RAW:				return RMC_BITS_GPS2_RAW;
		case DID_GPS_BASE_RAW:			return RMC_BITS_GPS_BASE_RAW;
		case DID_GPS_RTK_NAV:			return RMC_BITS_GPS_RTK_NAV;
		case DID_GPS_RTK_MISC:			return RMC_BITS_GPS_RTK_MISC;
		case DID_STROBE_IN_TIME:		return RMC_BITS_STROBE_IN_TIME;
		case DID_DIAGNOSTIC_MESSAGE:	return RMC_BITS_DIAGNOSTIC_MESSAGE;		
		default:						return defaultRmcBits;
	}
}

void julianToDate(double julian, int32_t* year, int32_t* month, int32_t* day, int32_t* hours, int32_t* minutes, int32_t* seconds, int32_t* milliseconds)
{
	double j1, j2, j3, j4, j5;
	double intgr = floor(julian);
	double frac = julian - intgr;
	double gregjd = 2299161.0;
	if (intgr >= gregjd)
	{
		//Gregorian calendar correction
		double tmp = floor(((intgr - 1867216.0) - 0.25) / 36524.25);
		j1 = intgr + 1.0 + tmp - floor(0.25 * tmp);
	}
	else
	{
		j1 = intgr;
	}

	//correction for half day offset
	double dayfrac = frac + 0.5;
	if (dayfrac >= 1.0)
	{
		dayfrac -= 1.0;
		++j1;
	}

	j2 = j1 + 1524.0;
	j3 = floor(6680.0 + ((j2 - 2439870.0) - 122.1) / 365.25);
	j4 = floor(j3 * 365.25);
	j5 = floor((j2 - j4) / 30.6001);

	double d = floor(j2 - j4 - floor(j5 * 30.6001));
	double m = floor(j5 - 1);
	if (m > 12)
	{
		m -= 12;
	}
	double y = floor(j3 - 4715.0);
	if (m > 2)
	{
		--y;
	}
	if (y <= 0)
	{
		--y;
	}

	//
	// get time of day from day fraction
	//
	double hr = floor(dayfrac * 24.0);
	double mn = floor((dayfrac * 24.0 - hr) * 60.0);
	double f = ((dayfrac * 24.0 - hr) * 60.0 - mn) * 60.0;
	double sc = f;
	if (f - sc > 0.5)
	{
		++sc;
	}

	if (y < 0)
	{
		y = -y;
	}
	if (year)
	{
		*year = (int32_t)y;
	}
	if (month)
	{
		*month = (int32_t)m;
	}
	if (day)
	{
		*day = (int32_t)d;
	}
	if (hours)
	{
		*hours = (int32_t)hr;
	}
	if (minutes)
	{
		*minutes = (int32_t)mn;
	}
	if (seconds)
	{
		*seconds = (int32_t)sc;
	}
	if (milliseconds)
	{
		*milliseconds = (int32_t)((sc - floor(sc)) * 1000.0);
	}
}

double gpsToJulian(int32_t gpsWeek, int32_t gpsMilliseconds)
{
	double gpsDays = (double)gpsWeek * 7.0;
	gpsDays += ((((double)gpsMilliseconds / 1000.0) - (double)CURRENT_LEAP_SECONDS) / 86400.0);
	return (2444244.500000) + gpsDays; // 2444244.500000 Julian date for Jan 6, 1980 midnight - start of gps time
}
