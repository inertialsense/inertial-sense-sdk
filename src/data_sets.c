/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "data_sets.h"
#include <stddef.h>
#include <math.h>

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

void flipDouble(void* ptr)
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
	} u1, u2;
	u1.v = val;
	u2.w[1] = SWAP32(u1.w[0]);
	u2.w[0] = SWAP32(u1.w[1]);
	return u2.v;
}

void flipEndianess32(uint8_t* data, int dataLength)
{
	// data must be 4 byte aligned to swap endian-ness
	if (dataLength & 0x00000003)
	{
		return;
	}
	
	uint32_t* dataPtr = (void*)data;
	uint32_t* dataPtrEnd = (void*)(data + dataLength);
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
                uint64_t* ptr = (void*)(data + offsetToDouble);
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

	static uint16_t offsetsGpsTimepulse[] =
	{
		3,
		offsetof(gps_timepulse_t, towOffset),
		offsetof(gps_timepulse_t, towGps),
		offsetof(gps_timepulse_t, timeMcu)
	};

	static uint16_t offsetsSysParams[] =
	{
		1,
		offsetof(sys_params_t, sensorTruePeriod),
	};

    static uint16_t offsetsPreImuMag[] =
    {
        2,
        offsetof(pimu_mag_t, pimu.time),
        offsetof(pimu_mag_t, mag.time)
    };

    static uint16_t offsetsImuMag[] =
    {
        2,
        offsetof(imu_mag_t, imu.time),
        offsetof(imu_mag_t, mag.time)
    };

	static uint16_t offsetsGps[] =
	{
		7,
		offsetof(gps_pos_t, lla[0]),
		offsetof(gps_pos_t, lla[1]),
		offsetof(gps_pos_t, lla[2]),
		offsetof(gps_pos_t, towOffset),
		offsetof(gps_pos_t, ecef[0]),
		offsetof(gps_pos_t, ecef[1]),
		offsetof(gps_pos_t, ecef[2])
	};

    static uint16_t offsetsRmc[] =
    {
        1, 
        // 0x8000 denotes a 64 bit int vs a double
		offsetof(rmc_t, bits) | 0x8000
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
	static uint16_t offsetsSurveyIn[] =
	{
		3,
		offsetof(survey_in_t, lla[0]),
		offsetof(survey_in_t, lla[1]),
		offsetof(survey_in_t, lla[2])
	};

    static uint16_t* s_doubleOffsets[] =
	{
		0,						//  0: DID_NULL
		0,						//  1: DID_DEV_INFO
        0,						//  2: DID_SYS_FAULT
		offsetsOnlyTimeFirst,	//  3: DID_PIMU
		offsetsIns1,			//  4: DID_INS_1
		offsetsIns2,			//  5: DID_INS_2
		offsetsGps,				//  6: DID_GPS1_POS
        0,  					//  7: DID_SYS_CMD
		0,						//  8: DID_NMEA_BCAST_PERIOD
		offsetsRmc,				//  9: DID_RMC
		offsetsSysParams,		// 10: DID_SYS_PARAMS
		offsetsOnlyTimeFirst,	// 11: DID_SYS_SENSORS
		offsetsFlashConfig,		// 12: DID_FLASH_CONFIG
		offsetsGps,				// 13: DID_GPS1_RCVR_POS
		offsetsGps,				// 14: DID_GPS2_POS
		0,						// 15: DID_GPS1_SAT
		0,						// 16: DID_GPS2_SAT
		0,                      // 17: DID_GPS1_VERSION
		0,						// 18: DID_GPS2_VERSION
		0,						// 19: DID_MAG_CAL
		0,						// 20: DID_INTERNAL_DIAGNOSTIC
        0,                      // 21: DID_GPS1_RTK_POS_REL
        offsetsRtkNav,          // 22: DID_GPS1_RTK_POS_MISC
		0,						// 23: DID_FEATURE_BITS
		0,						// 24: DID_SENSORS_UCAL
		0,						// 25: DID_SENSORS_TCAL
		0,						// 26: DID_SENSORS_TC_BIAS
		0,						// 27: DID_IO
		offsetsOnlyTimeFirst,	// 28: DID_SENSORS_ADC
		0,						// 29: DID_SCOMP
		0,						// 30: DID_GPS1_VEL
		0,						// 31: DID_GPS2_VEL
		0,						// 32: DID_HDW_PARAMS
		0,						// 33: DID_NVR_MANAGE_USERPAGE
		0,						// 34: DID_NVR_USERPAGE_SN
		0,						// 35: DID_NVR_USERPAGE_G0
		0,						// 36: DID_NVR_USERPAGE_G1
		0,						// 37: DID_NVR_MANAGE_PROTECTED
		0,						// 38: DID_RTOS_INFO
		offsetsDebugArray,		// 39: DID_DEBUG_ARRAY
		0,						// 40: DID_SENSORS_MCAL
		offsetsGpsTimepulse,	// 41: DID_GPS1_TIMEPULSE
		0,						// 42: DID_CAL_SC
		0,						// 43: DID_CAL_SC1
		0,						// 44: DID_CAL_SC2
		0,						// 45: 
		offsetsOnlyTimeFirst,	// 46: DID_SENSORS_ADC_SIGMA
		offsetsOnlyTimeFirst,   // 47: DID_REFERENCE_MAGNETOMETER
		offsetsInl2States,      // 48: DID_INL2_STATES
		0,                      // 49: DID_INL2_COVARIANCE_LD
		0,                      // 50: DID_INL2_MISC
		0,                      // 51: DID_INL2_STATUS,
		offsetsOnlyTimeFirst,	// 52: DID_MAGNETOMETER
		offsetsOnlyTimeFirst,	// 53: DID_BAROMETER
		0,						// 54: DID_GPS1_RTK_POS
		offsetsOnlyTimeFirst,	// 55: DID_ROS_COVARIANCE_POSE_TWIST
		0,						// 56: DID_COMMUNICATIONS_LOOPBACK
		offsetsOnlyTimeFirst,	// 57: DID_IMU3_UNCAL
		offsetsOnlyTimeFirst,	// 58: DID_IMU
		0,						// 59: DID_INL2_MAG_OBS_INFO
        0,						// 60: DID_GPS_BASE_RAW
        0,                      // 61: DID_GPS_RTK_OPT
        offsetsOnlyTimeFirst,   // 62: DID_REFERENCE_PIMU
		0,						// 63: DID_MANUFACTURING_INFO
		0,                      // 64: DID_BIT
		offsetsIns3,			// 65: DID_INS_3
		offsetsIns4,			// 66: DID_INS_4
		0,						// 67: DID_INL2_NED_SIGMA
        0,						// 68: DID_STROBE_IN_TIME
        0,						// 69: DID_GPS1_RAW
        0,						// 70: DID_GPS2_RAW
        offsetsOnlyTimeFirst,	// 71: DID_WHEEL_ENCODER
        0,						// 72: DID_DIAGNOSTIC_MESSAGE
        offsetsSurveyIn, 		// 73: DID_SURVEY_IN
        0,                      // 74: EMPTY
        0,                      // 75: DID_PORT_MONITOR
        0,                      // 76: DID_RTK_STATE
        0,                      // 77: DID_RTK_RESIDUAL
        0,                      // 78: DID_RTK_PHASE_RESIDUAL
        0,                      // 79: DID_RTK_CODE_RESIDUAL
        0,                      // 80: DID_EVB_STATUS
        0,                      // 81: DID_EVB_FLASH_CFG
        offsetsDebugArray,      // 82: DID_EVB_DEBUG_ARRAY
        0,                      // 83: DID_EVB_RTOS_INFO
        0,                      // 84: 
        offsetsImuMag,          // 85: DID_IMU_MAG
        offsetsPreImuMag,		// 86: DID_PIMU_MAG
		0,                      // 87: DID_GROUND_VEHICLE
		offsetsOnlyTimeFirst,   // 88: DID_POSITION_MEASUREMENT
		0,                      // 89: DID_RTK_DEBUG_2
		0,                      // 90: DID_CAN_CONFIG
		0,                      // 91: DID_GPS2_RTK_CMP_REL
		offsetsRtkNav,          // 92: DID_GPS2_RTK_CMP_MISC
		0,                      // 93: DID_EVB_DEV_INFO
		0,                      // 94: DID_INFIELD_CAL
		offsetsOnlyTimeFirst,   // 95: DID_REFERENCE_IMU
		offsetsOnlyTimeFirst,   // 96: DID_IMU3_RAW
		offsetsOnlyTimeFirst,   // 97: DID_IMU_RAW
		0,                      // 98:
		0,                      // 99:
		0,                      // 100:
		0,                      // 101:
		0,                      // 102:
		0,                      // 103:
		0,                      // 104:
		0,                      // 105:
		0,                      // 106:
		0,                      // 107:
		0,                      // 108:
		0,                      // 109:
		0,                      // 110:
		0,                      // 111:
		0,                      // 112:
		0,                      // 113:
		0,                      // 114:
		0,                      // 115:
		0,                      // 116:
		0,                      // 117:
		0,                      // 118:
		0,                      // 119:
		0,                      // 120: DID_GPX_DEV_INFO
		0,                      // 121: DID_GPX_FLASH_CFG
		0,                      // 122: DID_GPX_RTOS_INFO
		0,                      // 123: DID_GPX_STATUS
		offsetsDebugArray,      // 124: DID_GPX_DEBUG_ARRAY
		0,                      // 125:
		0,                      // 126:
		0,                      // 127:
		0,                      // 128:
		0,                      // 129:
		0,                      // 130:
		0                       // 131:
	};

    STATIC_ASSERT(_ARRAY_ELEMENT_COUNT(s_doubleOffsets) == DID_COUNT);
    STATIC_ASSERT((DID_COUNT%4) == 0);

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
        0,						//  2: DID_SYS_FAULT
		0,						//  3: DID_PIMU
		0,						//  4: DID_INS_1
		0,						//  5: DID_INS_2
		0,						//  6: DID_GPS1_POS
		0,						//  7: DID_SYS_CMD
		0,						//  8: DID_NMEA_BCAST_PERIOD
		0,						//  9: DID_RMC
		0,						// 10: DID_SYS_PARAMS
		0,						// 11: DID_SYS_SENSORS
		0,						// 12: DID_FLASH_CONFIG
		0,						// 13: DID_GPS1_RCVR_POS
		0,						// 14: DID_GPS2_POS
		0,						// 15: DID_GPS1_SAT
		0,						// 16: DID_GPS2_SAT
		0,						// 17: DID_GPS1_VERSION
		0,						// 18: DID_GPS2_VERSION
		0,						// 19: DID_MAG_CAL
		0,						// 20: DID_INTERNAL_DIAGNOSTIC
        0,                      // 21: DID_GPS1_RTK_POS_REL
        0,                      // 22: DID_GPS1_RTK_POS_MISC,
		0,						// 23: DID_FEATURE_BITS
		0,						// 24: DID_SENSORS_UCAL
		0,						// 25: DID_SENSORS_TCAL
		0,						// 26: DID_SENSORS_TC_BIAS
		0,						// 27: DID_IO
		0,						// 28: DID_SENSORS_ADC
		0,						// 29: DID_SCOMP
		0,						// 30: DID_GPS1_VEL
		0,						// 31: DID_GPS2_VEL
		0,						// 32: DID_HDW_PARAMS,
		0,						// 33: DID_NVR_MANAGE_USERPAGE
		0,						// 34: DID_NVR_USERPAGE_SN
		0,						// 35: DID_NVR_USERPAGE_G0
		0,						// 36: DID_NVR_USERPAGE_G1
		debugStringOffsets,		// 37: DID_DEBUG_STRING
		rtosTaskOffsets,		// 38: DID_RTOS_INFO
		0,						// 39: DID_DEBUG_ARRAY
		0,						// 40: DID_SENSORS_MCAL
		0,						// 41: 
		0,						// 42: DID_CAL_SC
		0,						// 43: DID_CAL_SC1
		0,						// 44: DID_CAL_SC2
		0,						// 45:
		0,						// 46: DID_SENSORS_ADC_SIGMA
		0,                      // 47: DID_REFERENCE_MAGNETOMETER
		0,                      // 48: DID_INL2_STATES
		0,                      // 49: DID_INL2_COVARIANCE_LD
		0,                      // 50: DID_INL2_MISC
		0,                      // 51: DID_INL2_STATUS
		0,						// 52: DID_MAGNETOMETER
		0,						// 53: DID_BAROMETER
		0,						// 54: DID_GPS1_RTK_POS
		0,						// 55: DID_ROS_COVARIANCE_POSE_TWIST
		0,						// 56: DID_COMMUNICATIONS_LOOPBACK
		0,						// 57: DID_IMU3_UNCAL
		0,						// 58: DID_IMU
		0,						// 59: DID_INL2_MAG_OBS_INFO
        0,						// 60: DID_GPS_BASE_RAW
        0,                      // 61: DID_GPS_RTK_OPT
        0,                      // 62: DID_REFERENCE_PIMU
		manufInfoOffsets,		// 63: DID_MANUFACTURING_INFO
		0,                      // 64: DID_BIT
		0,                      // 65: DID_INS_3
		0,                      // 66: DID_INS_4
		0,						// 67: DID_INL2_NED_SIGMA
		0,						// 68: DID_STROBE_IN_TIME
		0,						// 69: DID_GPS1_RAW
		0,						// 70: DID_GPS2_RAW
		0,						// 71: DID_WHEEL_ENCODER
		diagMsgOffsets, 		// 72: DID_DIAGNOSTIC_MESSAGE
		0,                      // 73: DID_SURVEY_IN
        0,                      // 74: DID_CAL_SC_INFO
        0,                      // 75: DID_PORT_MONITOR
        0,                      // 76: DID_RTK_STATE
        0,                      // 77: DID_RTK_RESIDUAL
        0,                      // 78: DID_RTK_PHASE_RESIDUAL
        0,                      // 79: DID_RTK_CODE_RESIDUAL
        0,                      // 80: DID_EVB_STATUS
        0,                      // 81: DID_EVB_FLASH_CFG
        0,                      // 82: DID_EVB_DEBUG_ARRAY
        0,                      // 83: DID_EVB_RTOS_INFO
		0,						// 84: 
		0,						// 85: DID_IMU_MAG
		0,						// 86: DID_PIMU_MAG
		0,						// 87: DID_GROUND_VEHICLE
		0,						// 88: DID_POSITION_MEASUREMENT
		0,						// 89: DID_RTK_DEBUG_2
		0,						// 90: DID_CAN_CONFIG
		0,                      // 91: DID_GPS2_RTK_CMP_REL
		0,                      // 92: DID_GPS2_RTK_CMP_MISC
		0,                      // 93: DID_EVB_DEV_INFO
		0,                      // 94: DID_INFIELD_CAL
		0,                      // 95: DID_REFERENCE_IMU
		0,                      // 96: DID_IMU3_RAW
		0,                      // 97: DID_IMU_RAW
		0,                      // 98:
		0,                      // 99:
		0,                      // 100:
		0,                      // 101:
		0,                      // 102:
		0,                      // 103:
		0,                      // 104:
		0,                      // 105:
		0,                      // 106:
		0,                      // 107:
		0,                      // 108:
		0,                      // 109:
		0,                      // 110:
		0,                      // 111:
		0,                      // 112:
		0,                      // 113:
		0,                      // 114:
		0,                      // 115:
		0,                      // 116:
		0,                      // 117:
		0,                      // 118:
		0,                      // 119:
		0,                      // 120: DID_GPX_DEV_INFO
		0,                      // 121: DID_GPX_FLASH_CFG
		0,                      // 122: DID_GPX_RTOS_INFO
		0,                      // 123: DID_GPX_STATUS
		0,                      // 124: DID_GPX_DEBUG_ARRAY
		0,                      // 125:
		0,                      // 126:
		0,                      // 127:
		0,                      // 128:
		0,                      // 129:
		0,                      // 130:
		0                       // 131:
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

// DID to RMC bit look-up table
const uint64_t g_didToRmcBit[DID_COUNT] = 
{
	[DID_INS_1]               = RMC_BITS_INS1,
	[DID_INS_2]               = RMC_BITS_INS2,
	[DID_INS_3]               = RMC_BITS_INS3,
	[DID_INS_4]               = RMC_BITS_INS4,
	[DID_IMU3_UNCAL]          = RMC_BITS_IMU3_UNCAL,
	[DID_IMU3_RAW]            = RMC_BITS_IMU3_RAW,
	[DID_IMU_RAW]             = RMC_BITS_IMU_RAW,
	[DID_IMU]                 = RMC_BITS_IMU,
	[DID_PIMU]                = RMC_BITS_PIMU,
	[DID_REFERENCE_IMU]       = RMC_BITS_REFERENCE_IMU,
	[DID_REFERENCE_PIMU]      = RMC_BITS_REFERENCE_PIMU,
	[DID_BAROMETER]           = RMC_BITS_BAROMETER,
	[DID_MAGNETOMETER]        = RMC_BITS_MAGNETOMETER,
	[DID_GPS1_POS]            = RMC_BITS_GPS1_POS,
	[DID_GPS2_POS]            = RMC_BITS_GPS2_POS,
	[DID_GPS1_VEL]            = RMC_BITS_GPS1_VEL,
	[DID_GPS2_VEL]            = RMC_BITS_GPS2_VEL,
	[DID_GPS1_SAT]            = RMC_BITS_GPS1_SAT,
	[DID_GPS2_SAT]            = RMC_BITS_GPS2_SAT,
	[DID_GPS1_SIG]            = RMC_BITS_GPS1_SIG,
	[DID_GPS2_SIG]            = RMC_BITS_GPS2_SIG,
	[DID_GPS1_RAW]            = RMC_BITS_GPS1_RAW,
	[DID_GPS2_RAW]            = RMC_BITS_GPS2_RAW,
	[DID_GPS_BASE_RAW]        = RMC_BITS_GPS_BASE_RAW,
	[DID_GPS1_RCVR_POS]       = RMC_BITS_GPS1_UBX_POS,
	[DID_GPS1_RTK_POS]        = RMC_BITS_GPS1_RTK_POS,
	[DID_GPS1_RTK_POS_REL]    = RMC_BITS_GPS1_RTK_POS_REL,
	[DID_GPS1_RTK_POS_MISC]   = RMC_BITS_GPS1_RTK_POS_MISC,
	[DID_GPS2_RTK_CMP_REL]    = RMC_BITS_GPS1_RTK_HDG_REL,
	[DID_GPS2_RTK_CMP_MISC]   = RMC_BITS_GPS1_RTK_HDG_MISC,
	[DID_STROBE_IN_TIME]      = RMC_BITS_STROBE_IN_TIME,
	[DID_DIAGNOSTIC_MESSAGE]  = RMC_BITS_DIAGNOSTIC_MESSAGE,
	[DID_INL2_NED_SIGMA]      = RMC_BITS_INL2_NED_SIGMA,
	[DID_RTK_STATE]           = RMC_BITS_RTK_STATE,
	[DID_RTK_CODE_RESIDUAL]   = RMC_BITS_RTK_CODE_RESIDUAL,
	[DID_RTK_PHASE_RESIDUAL]  = RMC_BITS_RTK_PHASE_RESIDUAL,
	[DID_WHEEL_ENCODER]       = RMC_BITS_WHEEL_ENCODER,
	[DID_GROUND_VEHICLE]      = RMC_BITS_GROUND_VEHICLE,
	[DID_IMU_MAG]             = RMC_BITS_IMU_MAG,
	[DID_PIMU_MAG]            = RMC_BITS_PIMU_MAG,
	[DID_GPX_RTOS_INFO]       = RMC_BITS_GPX_RTOS_INFO,
	[DID_GPX_DEBUG_ARRAY]     = RMC_BITS_GPX_DEBUG,
};

uint64_t didToRmcBit(uint32_t dataId, uint64_t defaultRmcBits, uint64_t devInfoRmcBits)
{
	if (dataId == DID_DEV_INFO)     { return devInfoRmcBits; }		// This allows the dev info to respond instantly when first connected.
	else if (g_didToRmcBit[dataId]) { return g_didToRmcBit[dataId]; }
	else                            { return defaultRmcBits; }
}

// DID to NMEA RMC bit look-up table
const uint64_t g_didToNmeaRmcBit[DID_COUNT] = 
{
	[DID_IMU]                   = NMEA_RMC_BITS_PIMU,
	[DID_PIMU]                  = NMEA_RMC_BITS_PPIMU,
	[DID_IMU_RAW]               = NMEA_RMC_BITS_PRIMU,
	[DID_INS_1]                 = NMEA_RMC_BITS_PINS1,
	[DID_INS_2]                 = NMEA_RMC_BITS_PINS2,
	[DID_GPS1_SAT]              = NMEA_RMC_BITS_GSV,
	[DID_GPS1_POS]				=
		NMEA_RMC_BITS_PGPSP |
		NMEA_RMC_BITS_GGA |
		NMEA_RMC_BITS_GLL |
		NMEA_RMC_BITS_GSA |
		NMEA_RMC_BITS_RMC |
		NMEA_RMC_BITS_ZDA |
		NMEA_RMC_BITS_PASHR,
	[DID_DEV_INFO]              = NMEA_RMC_BITS_INFO,
};

// DID to GPX RMC bit look-up table
const uint64_t g_gpxDidToGrmcBit[DID_COUNT] = 
{
	[DID_GPX_DEV_INFO]           = GRMC_BITS_DEV_INFO,
	[DID_GPX_FLASH_CFG]          = GRMC_BITS_FLASH_CFG,
	[DID_GPX_RTOS_INFO]          = GRMC_BITS_RTOS_INFO,
	[DID_GPX_STATUS]             = GRMC_BITS_STATUS,
	[DID_GPX_DEBUG_ARRAY]        = GRMC_BITS_DEBUG_ARRAY,
	[DID_GPS1_POS]               = GRMC_BITS_GPS1_POS,
	[DID_GPS1_VEL]               = GRMC_BITS_GPS1_VEL,
	[DID_GPS1_RAW]               = GRMC_BITS_GPS1_RAW,
	[DID_GPS1_SAT]               = GRMC_BITS_GPS1_SAT,
	[DID_GPS1_SIG]               = GRMC_BITS_GPS1_SIG,
	[DID_GPS1_VERSION]           = GRMC_BITS_GPS1_VERSION,
	[DID_GPS2_POS]               = GRMC_BITS_GPS2_POS,
	[DID_GPS2_VEL]               = GRMC_BITS_GPS2_VEL,
	[DID_GPS2_SAT]               = GRMC_BITS_GPS2_SAT,
	[DID_GPS2_SIG]               = GRMC_BITS_GPS2_SIG,
	[DID_GPS2_RAW]               = GRMC_BITS_GPS2_RAW,
	[DID_GPS2_VERSION]           = GRMC_BITS_GPS2_VERSION,
	[DID_GPS1_RTK_POS_MISC]      = GMRC_BITS_GPS1_RTK_POS_MISC,
	[DID_GPS1_RTK_POS_REL]       = GMRC_BITS_GPS1_RTK_POS_REL,
	[DID_GPS2_RTK_CMP_MISC]      = GMRC_BITS_GPS2_RTK_CMP_MISC,
	[DID_GPS2_RTK_CMP_REL]       = GMRC_BITS_GPS2_RTK_CMP_REL,
};

void julianToDate(double julian, int32_t* year, int32_t* month, int32_t* day, int32_t* hour, int32_t* minute, int32_t* second, int32_t* millisecond)
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
	if (hour)
	{
		*hour = (int32_t)hr;
	}
	if (minute)
	{
		*minute = (int32_t)mn;
	}
	if (second)
	{
		*second = (int32_t)sc;
	}
	if (millisecond)
	{
		*millisecond = (int32_t)((sc - floor(sc)) * 1000.0);
	}
}

double gpsToUnix(uint32_t gpsWeek, uint32_t gpsTimeofWeekMS, uint8_t leapSeconds)
{
	double gpsSeconds = gpsWeek * SECONDS_PER_WEEK;
	gpsSeconds += (gpsTimeofWeekMS / 1000);
	double unixSeconds = gpsSeconds + GPS_TO_UNIX_OFFSET - leapSeconds;

	return unixSeconds;
}

double gpsToJulian(int32_t gpsWeek, int32_t gpsMilliseconds, int32_t leapSeconds)
{
	double gpsDays = (double)gpsWeek * 7.0;
	gpsDays += ((((double)gpsMilliseconds / 1000.0) - (double)leapSeconds) / 86400.0);
	return (2444244.500000) + gpsDays; // 2444244.500000 Julian date for Jan 6, 1980 midnight - start of gps time
}

static void appendGPSTimeOfLastFix(const gps_pos_t* gps, char** buffer, int* bufferLength)
{
    unsigned int millisecondsToday = gps->timeOfWeekMs % 86400000;
    unsigned int hours = millisecondsToday / 1000 / 60 / 60;
    unsigned int minutes = (millisecondsToday / (1000 * 60)) % 60;
    unsigned int seconds = (millisecondsToday / 1000) % 60;
    int written = SNPRINTF(*buffer, *bufferLength, ",%02u%02u%02u", hours, minutes, seconds);
    *bufferLength -= written;
    *buffer += written;
}

static void appendGPSCoord(const gps_pos_t* gps, char** buffer, int* bufferLength, double v, const char* degreesFormat, char posC, char negC)
{
	(void)gps;
    int degrees = (int)(v);
    double minutes = (v - ((double)degrees)) * 60.0;

    int written = SNPRINTF(*buffer, *bufferLength, degreesFormat, abs(degrees));
    *bufferLength -= written;
    *buffer += written;

    written = SNPRINTF(*buffer, *bufferLength, "%07.4f,", fabs(minutes));
    *bufferLength -= written;
    *buffer += written;

    written = SNPRINTF(*buffer, *bufferLength, "%c", (degrees >= 0 ? posC : negC));
    *bufferLength -= written;
    *buffer += written;
}
#ifndef GPX_1

/* ubx gnss indicator (ref [2] 25) -------------------------------------------*/
int ubxSys(int gnssID)
{
	switch (gnssID) {
	case 0: return SYS_GPS;
	case 1: return SYS_SBS;
	case 2: return SYS_GAL;
	case 3: return SYS_CMP;
	case 5: return SYS_QZS;
	case 6: return SYS_GLO;
	}
	return 0;
}

/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
int satNo(int sys, int prn)
{
	if (prn <= 0) return 0;
	switch (sys) {
	case SYS_GPS:
		if (prn < MINPRNGPS || MAXPRNGPS < prn) return 0;
		return prn - MINPRNGPS + 1;
	case SYS_GLO:
		if (prn < MINPRNGLO || MAXPRNGLO < prn) return 0;
		return NSATGPS + prn - MINPRNGLO + 1;
	case SYS_GAL:
		if (prn < MINPRNGAL || MAXPRNGAL < prn) return 0;
		return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
	case SYS_QZS:
		if (prn < MINPRNQZS || MAXPRNQZS < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1;
	case SYS_CMP:
		if (prn < MINPRNCMP || MAXPRNCMP < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1;
	case SYS_IRN:
		if (prn < MINPRNIRN || MAXPRNIRN < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + prn - MINPRNIRN + 1;
	case SYS_LEO:
		if (prn < MINPRNLEO || MAXPRNLEO < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN +
			prn - MINPRNLEO + 1;
	case SYS_SBS:
		if (prn < MINPRNSBS || MAXPRNSBS < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATIRN + NSATLEO +
			prn - MINPRNSBS + 1;
	}
	return 0;
}

/* satellite gnssID+svID to satellite number ------------------------
* convert satellite gnssID + svID to satellite number
* args   : int    gnssID     I   satellite system
*          int    svID       I   satellite vehicle ID within system
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
int satNumCalc(int gnssID, int svID) {
	int sys = ubxSys(gnssID);
	int prn = svID + (sys == SYS_QZS ? 192 : 0);
	return satNo(sys, prn);
}

#endif
