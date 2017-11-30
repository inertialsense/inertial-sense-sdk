/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef DATA_SETS_H
#define DATA_SETS_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "ISConstants.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! Maximum number of satellite channels */
#define MAX_NUM_SAT_CHANNELS 50

/*! Maximum length of device info manufacturer string (must be a multiple of 4) */
#define DEVINFO_MANUFACTURER_STRLEN 24
#define DEVINFO_ADDINFO_STRLEN 24

// *****************************************************************************
// ****** InertialSense binary message Data Identification Numbers (DIDs) ****** 
// ******                                                                 ******
// ****** NEVER REORDER THESE VALUES!                                     ******
// *****************************************************************************
/*! Data identifiers - these are unsigned int and #define because enum are signed according to C standard */
typedef uint32_t eDataIDs;

/*! 0  : NULL (INVALID) */
#define DID_NULL (eDataIDs)0

/*! 1  : (see dev_info_t) Device information */
#define DID_DEV_INFO (eDataIDs)1
	
/*! 2  : (see imu_t) Inertial measurement unit data: calibrated gyro, accelerometer, magnetometer, and barometric pressure. */
#define DID_IMU_1 (eDataIDs)2

/*! 3  : (see delta_theta_vel_t) Conning and sculling integral in body/IMU frame. Updated at IMU rate. */
#define DID_DELTA_THETA_VEL (eDataIDs)3

/*! 4  : (see ins_1_t) INS output: euler rotation w/ respect to NED, NED position from reference LLA */
#define DID_INS_1 (eDataIDs)4

/*! 5  : (see ins_2_t) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
#define DID_INS_2 (eDataIDs)5

/*! 6  : (see gps_t) GPS data */
#define DID_GPS (eDataIDs)6

/*! 7  : (see config_t) Configuration data */
#define DID_CONFIG (eDataIDs)7

/*! 8  : (see ascii_msgs_t) Broadcast period for ASCII messages */
#define DID_ASCII_BCAST_PERIOD (eDataIDs)8

/*! 9  : (see ins_misc_t) Other INS data */
#define DID_INS_MISC (eDataIDs)9

/*! 10 : (see sys_params_t) System parameters */
#define DID_SYS_PARAMS (eDataIDs)10

/*! 11 : (see sys_sensors_t) System sensor information (10) */
#define DID_SYS_SENSORS (eDataIDs)11

/*! 12 : (see nvm_flash_cfg_t) Flash memory configuration */
#define DID_FLASH_CONFIG (eDataIDs)12

/*! 13 : (see gps_cno_t) GPS carrier to noise ration (signal strength) */
#define DID_GPS_CNO (eDataIDs)13

/*! 14 : (see gps_nav_poslla_t) GPS velocity data */
#define DID_GPS_POS (eDataIDs)14

/*! 15 : (see gps_nav_velned_t) GPS velocity data */
#define DID_GPS_VEL (eDataIDs)15

/*! 16 : (see io_t) I/O: Servos */
#define DID_IO (eDataIDs)16

/*! 17 : (see io_servos_t) I/O: Servos Pulse Width Modulation (PWM) */
#define DID_IO_SERVOS_PWM (eDataIDs)17

/*! 18 : (see io_servos_t) I/O: Servos Pulse Position Modulation (PPM), single wire servo pulse train */
#define DID_IO_SERVOS_PPM (eDataIDs)18

/*! 19 : (see mag_cal_t) INL1 Magnetometer calibration */
#define DID_MAG_CAL (eDataIDs)19

/*! 20 : (see ins_res_t) Resources */
#define DID_INS_RESOURCES (eDataIDs)20

/*! 21 : Differential GPS Correction */
#define DID_DGPS_CORRECTION (eDataIDs)21

/*! 22 : (see rtk_sol_t) RTK Solution */
#define DID_RTK_SOL (eDataIDs)22

/*! 23 : INTERNAL USE ONLY (feature_bits_t) */
#define DID_FEATURE_BITS (eDataIDs)23

/*! 24 : INTERNAL USE ONLY (sensors_w_temp_t) */
#define DID_SENSORS_IS1 (eDataIDs)24

/*! 25 : INTERNAL USE ONLY (sensors_w_temp_t) */
#define DID_SENSORS_IS2 (eDataIDs)25

/*! 26 : INTERNAL USE ONLY (sensors_t) */
#define DID_SENSORS_TC_BIAS (eDataIDs)26

/*! 27 : INTERNAL USE ONLY (sensor_bias_t) */
#define DID_SENSORS_CF_BIAS (eDataIDs)27

/*! 28 : INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_SENSORS_ADC (eDataIDs)28

/*! 29 : INTERNAL USE ONLY (sensor_compensation_t) */
#define DID_SCOMP (eDataIDs)29

/*! 30 : INTERNAL USE ONLY (ins_params_t) */
#define DID_INS_PARAMS (eDataIDs)30

/*! 31 : INTERNAL USE ONLY (obs_params_t) */
#define DID_OBS_PARAMS (eDataIDs)31

/*! 32 : INTERNAL USE ONLY (hdw_params_t) */
#define DID_HDW_PARAMS (eDataIDs)32

/*! 33 : INTERNAL USE ONLY (nvr_manage_t) */
#define DID_NVR_MANAGE_USERPAGE (eDataIDs)33

/*! 34 : INTERNAL USE ONLY (nvm_group_sn_t) */
#define DID_NVR_USERPAGE_SN (eDataIDs)34

/*! 35 : INTERNAL USE ONLY (nvm_group_0_t) */
#define DID_NVR_USERPAGE_G0 (eDataIDs)35

/*! 36 : INTERNAL USE ONLY (nvm_group_1_t) */
#define DID_NVR_USERPAGE_G1 (eDataIDs)36

/*! 37 : INTERNAL USE ONLY (debug_string_t) */
#define DID_DEBUG_STRING (eDataIDs)37

/*! 38 : INTERNAL USE ONLY (rtos_info_t) */
#define DID_RTOS_INFO (eDataIDs)38

/*! 39 : INTERNAL USE ONLY (debug_array_t) */
#define DID_DEBUG_ARRAY (eDataIDs)39

/*! 40 : INTERNAL USE ONLY (sensors_mpu_w_temp_t) */
#define DID_SENSORS_CAL1 (eDataIDs)40

/*! 41 : INTERNAL USE ONLY (sensors_mpu_w_temp_t) */
#define DID_SENSORS_CAL2 (eDataIDs)41

/*! 42 : INTERNAL USE ONLY (sensor_cal_t) */
#define DID_CAL_SC (eDataIDs)42

/*! 43 : INTERNAL USE ONLY (sensor_cal_mpu_t) */
#define DID_CAL_SC1 (eDataIDs)43

/*! 44 : INTERNAL USE ONLY (sensor_cal_mpu_t) */
#define DID_CAL_SC2 (eDataIDs)44

/*! 45 : INTERNAL USE ONLY (sys_sensors_t) */
#define DID_SYS_SENSORS_SIGMA (eDataIDs)45

/*! 46 : INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_SENSORS_ADC_SIGMA (eDataIDs)46

/*! 47 : INTERNAL USE ONLY (ins_dev_1_t) */
#define DID_INS_DEV_1 (eDataIDs)47

/*! 48 : (see inl2_states_t) */
#define DID_INL2_STATES (eDataIDs)48

/*! 49 : (see INL2_COVARIANCE_LD_ARRAY_SIZE) */
#define DID_INL2_COVARIANCE_LD (eDataIDs)49

/*! 50 : (see inl2_status_t) */
#define DID_INL2_STATUS (eDataIDs)50

/*! 51 : (see inl2_misc_t) */
#define DID_INL2_MISC (eDataIDs)51

/*! 52 : (see magnetometer_t) Magnetometer sensor data */
#define DID_MAGNETOMETER_1 (eDataIDs)52

/*! 53 : (see barometer_t) Barometric pressure sensor data */
#define DID_BAROMETER (eDataIDs)53

/*! 54 : (see imu_t) 2nd inertial measurement unit data: calibrated gyroscope and accelerometer. */
#define DID_IMU_2 (eDataIDs)54

/*! 55 : (see magnetometer_t) 2nd magnetometer sensor data */
#define DID_MAGNETOMETER_2 (eDataIDs)55

/*! 56 : (see gps_version_t) GPS version info */
#define DID_GPS_VERSION (eDataIDs)56

/*! 57 : INTERNAL USE ONLY - Unit test for communications manager  */
#define DID_COMMUNICATIONS_LOOPBACK (eDataIDs)57

/*! 58 : (see dual_imu_t) dual inertial measurement units data: calibrated gyroscope and accelerometer. */
#define DID_DUAL_IMU (eDataIDs)58

/*! 59 : (see inl2_mag_obs_info_t) INL2 magnetometer calibration information. */
#define DID_INL2_MAG_OBS_INFO (eDataIDs)59

/*! 60 : (see raw_gps_msg_t) Raw data (observation, ephemeris, etc.) - requires little endian CPU. 4 byte header of receiver index, type, count and reserved, then n data elements based on type.  */
#define DID_RAW_GPS_DATA (eDataIDs)60

/*! 61 : (see rtk_opt_t) RTK options - requires little endian CPU */
#define DID_RTK_OPT (eDataIDs)61

/*! 62 : (internal) Internal user page data */
#define DID_NVR_USERPAGE_INTERNAL (eDataIDs)62

/*! 63 : INTERNAL USE ONLY (manufacturing_info_t) Manufacturing info */
#define DID_MANUFACTURING_INFO (eDataIDs)63

/*! 64 : (see bit_t) System built-in self-test */
#define DID_BIT (eDataIDs)64

/*! 65 : (see ins_3_t) Inertial navigation data with quaternion NED to body rotation and ECEF position */
#define DID_INS_3 (eDataIDs)65

/*! 66 : (see ins_4_t) INS output: quaternion rotation w/ respect to ECEF, ECEF position */
#define DID_INS_4 (eDataIDs)66

/*! 67 : (see inl2_variance_t) INL2 variance */
#define DID_INL2_VARIANCE (eDataIDs)67

// Adding a new data id?
// 1] Add it above and increment the previous number, include the matching data structure type in the comments
// 2] Add flip doubles and flip strings entries in data_sets.c
// 3] Add data id to ISDataMappings.cpp
// 4] Increment DID_COUNT
// 5] Test!

/*! Count of data ids - make sure to increment if you add a new data id! */
#define DID_COUNT (eDataIDs)68

/*! Maximum number of data ids */
#define DID_MAX_COUNT 256

// END DATA IDENTIFIERS --------------------------------------------------------------------------

/*! Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */
// #define PROTOCOL_VERSION_CHAR0 1        // Major (in com_manager.h)
// #define PROTOCOL_VERSION_CHAR1 0
#define PROTOCOL_VERSION_CHAR2 (0x000000FF&DID_COUNT)
#define PROTOCOL_VERSION_CHAR3 6         // Minor (in data_sets.h)

/*! INS status flags */
enum eInsStatusFlags
{
	/*! Attitude estimate is COARSE (usable but outside spec) */
	INS_STATUS_ATT_ALIGN_COARSE				= (int)0x00000001,
	/*! Velocity estimate is COARSE (usable but outside spec) */
	INS_STATUS_VEL_ALIGN_COARSE				= (int)0x00000002,
	/*! Position estimate is COARSE (usable but outside spec) */
	INS_STATUS_POS_ALIGN_COARSE				= (int)0x00000004,
	/*! Estimate is COARSE mask (usable but outside spec) */
	INS_STATUS_ALIGN_COARSE_MASK			= (int)0x00000007,

	/*! Attitude estimate is GOOD */
	INS_STATUS_ATT_ALIGN_GOOD				= (int)0x00000010,
	/*! Velocity estimate is GOOD */
	INS_STATUS_VEL_ALIGN_GOOD				= (int)0x00000020,
	/*! Position estimate is GOOD */
	INS_STATUS_POS_ALIGN_GOOD				= (int)0x00000040,
	/*! Estimate is GOOD mask */
	INS_STATUS_ALIGN_GOOD_MASK				= (int)0x00000070,

	/*! Attitude estimate is FINE.  (For internal use. Do not use.) */
	INS_STATUS_ATT_ALIGN_FINE				= (int)0x00000080,
	/*! Estimate is FINE mask.  (For internal use. Do not use.) */
	INS_STATUS_FINE_ALIGNED_MASK			= (int)0x000000FF,

	/*! GPS is being fused into solution */
	INS_STATUS_USING_GPS_IN_SOLUTION		= (int)0x00000100,
	/*! Mag is being fused into solution */
	INS_STATUS_USING_MAG_IN_SOLUTION		= (int)0x00000800,

	/*! Set = Nav mode (w/ GPS). Cleared = AHRS mode (w/o GPS) */
	INS_STATUS_NAV_MODE						= (int)0x00001000,

	/*! INS/AHRS Solution Status */
	INS_STATUS_SOLUTION_MASK				= (int)0x000F0000,
	INS_STATUS_SOLUTION_OFFSET				= 16,
#define INS_STATUS_SOLUTION(insStatus) ((insStatus&INS_STATUS_SOLUTION_MASK)>>INS_STATUS_SOLUTION_OFFSET)

	INS_STATUS_SOLUTION_OFF					= 0,	// System is off 
	INS_STATUS_SOLUTION_ALIGNING			= 1,	// System is in alignment mode
	INS_STATUS_SOLUTION_ALIGNMENT_COMPLETE	= 2,	// System is aligned but not enough dynamics have been experienced to be with specifications.
	INS_STATUS_SOLUTION_NAV_GOOD			= 3,	// System is in navigation mode and solution is good.
	INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE	= 4,	// System is in navigation mode but the attitude uncertainty has exceeded the threshold.
	INS_STATUS_SOLUTION_AHRS_GOOD			= 5,	// System is in AHRS mode and solution is good.
	INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE	= 6,	// System is in AHRS mode but the attitude uncertainty has exceeded the threshold.
	
	/*! Reserved */
	INS_STATUS_RESERVED_1					= (int)0x00100000,
	INS_STATUS_RESERVED_2					= (int)0x00200000,
	
	/*! Magnetometer is being recalibrated.  Device requires rotation. */
	INS_STATUS_MAG_RECALIBRATING			= (int)0x00400000,
	/*! Magnetometer is experiencing interference.  Attention may be required to remove interference (i.e. move device). */
	INS_STATUS_MAG_NOT_GOOD					= (int)0x00800000,

	/*! RTOS task ran longer than allotted period */
	INS_STATUS_RTOS_TASK_PERIOD_OVERRUN		= (int)0x20000000,
	/*! General fault */
	INS_STATUS_GENERAL_FAULT				= (int)0x80000000,
};

/*! Hardware status flags */
enum eHdwStatusFlags
{
	/*! Gyro motion detected sigma */
	HDW_STATUS_MOTION_GYR_SIG				= (int)0x00000001,
	/*! Accelerometer motion detected sigma */
	HDW_STATUS_MOTION_ACC_SIG				= (int)0x00000002,
	/*! Unit is moving and NOT stationary */
	HDW_STATUS_MOTION_SIG_MASK				= (int)0x00000003,
	/*! Gyro motion detected deviation */
	HDW_STATUS_MOTION_GYR_DEV				= (int)0x00000004,
	/*! Accelerometer motion detected deviation */
	HDW_STATUS_MOTION_ACC_DEV				= (int)0x00000008,
	/*! Motion mask */
	HDW_STATUS_MOTION_MASK					= (int)0x0000000F,

	/*! GPS satellite signals are being received (antenna and cable are good) */
	HDW_STATUS_GPS_SATELLITE_RX				= (int)0x00000010,

	/*! Sensor saturation on gyro */
	HDW_STATUS_SATURATION_GYR				= (int)0x00000100,
	/*! Sensor saturation on accelerometer */
	HDW_STATUS_SATURATION_ACC				= (int)0x00000200,
	/*! Sensor saturation on magnetometer */
	HDW_STATUS_SATURATION_MAG				= (int)0x00000400,
	/*! Sensor saturation on barometric pressure */
	HDW_STATUS_SATURATION_BARO				= (int)0x00000800,

	/*! Sensor saturation happened in past */
	HDW_STATUS_SATURATION_HISTORY			= (int)0x00001000,
	/*! Sensor saturation mask */
	HDW_STATUS_SATURATION_MASK				= (int)0x00000F00,
	/*! Sensor saturation offset */
	HDW_STATUS_SATURATION_OFFSET			= 8,

	/*! Communications Tx buffer limited */
	HDW_STATUS_ERR_COM_TX_LIMITED			= (int)0x00010000,
	/*! Communications Rx buffer overrun */
	HDW_STATUS_ERR_COM_RX_OVERRUN			= (int)0x00020000,
	/*! GPS Tx buffer limited */
	HDW_STATUS_ERR_GPS_TX_LIMITED			= (int)0x00040000,
	/*! GPS Rx buffer overrun */
	HDW_STATUS_ERR_GPS_RX_OVERRUN			= (int)0x00080000,

	/*! Communications parse error count */
	HDW_STATUS_COM_PARSE_ERR_COUNT_MASK		= (int)0x00F00000,
	HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET	= 20,
#define HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus) ((hdwStatus&HDW_STATUS_COM_PARSE_ERR_COUNT_MASK)>>HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET)

	/*! Built-in self-test failure */
	HDW_STATUS_BIT_FAULT				= (int)0x01000000,
	/*! Temperature outside spec'd operating range */
	HDW_STATUS_ERR_TEMPERATURE				= (int)0x02000000,
	/*! Vibrations effecting accuracy */
// 	HDW_STATUS_ERR_VIBRATION				= (int)0x04000000,

	/*! Watchdog reset */
	HDW_STATUS_FAULT_WATCHDOG_RESET			= (int)0x10000000,
	/*! Brownout (low system voltage) detection reset */
	HDW_STATUS_FAULT_BOD_RESET				= (int)0x20000000,
	/*! Power-on reset (from reset pin or software) */
	HDW_STATUS_FAULT_POR_RESET				= (int)0x40000000,
	/*! CPU error reset */
	HDW_STATUS_FAULT_CPU_ERR_RESET			= (int)0x80000000,
};


/*! GPS Status */
enum eGpsStatus
{
	GPS_STATUS_NUM_SATS_USED_MASK            = (int)0x000000FF,

	/*! Fix type  */
	GPS_STATUS_FIX_TYPE_NO_FIX               = (int)0x00000000,
	GPS_STATUS_FIX_TYPE_DEAD_RECKONING_ONLY  = (int)0x00000100,
	GPS_STATUS_FIX_TYPE_2D_FIX               = (int)0x00000200,
	GPS_STATUS_FIX_TYPE_3D_FIX               = (int)0x00000300,
	GPS_STATUS_FIX_TYPE_GPS_PLUS_DEAD_RECK   = (int)0x00000400,
	GPS_STATUS_FIX_TYPE_TIME_ONLY_FIX        = (int)0x00000500,
	GPS_STATUS_FIX_TYPE_RESERVED1            = (int)0x00000600,
	GPS_STATUS_FIX_TYPE_RESERVED2            = (int)0x00000700,
	GPS_STATUS_FIX_TYPE_MASK                 = (int)0x0000FF00,
	GPS_STATUS_FIX_TYPE_BIT_OFFSET           = (int)8,

	/*! Fix within limits (e.g. DOP & accuracy)  */
	GPS_STATUS_FIX_STATUS_FIX_OK             = (int)0x00010000,
	/*! Differential GPS (DGPS) used  */
	GPS_STATUS_FIX_STATUS_DGPS_USED          = (int)0x00020000,
	GPS_STATUS_FIX_STATUS_WEEK_VALID         = (int)0x00040000,
	GPS_STATUS_FIX_STATUS_TOW_VALID          = (int)0x00080000,
	GPS_STATUS_FIX_STATUS_MASK               = (int)0x00FF0000,
	GPS_STATUS_FIX_STATUS_BIT_OFFSET         = (int)16,

	/*! Init status  */
	GPS_STATUS_INIT_STATUS_PROGRAM_OSC       = (int)0x01000000,
	GPS_STATUS_INIT_STATUS_REINIT            = (int)0x02000000,
	GPS_STATUS_INIT_STATUS_READING           = (int)0x04000000,
	GPS_STATUS_INIT_STATUS_MASK              = (int)0xFF000000,
	GPS_STATUS_INIT_STATUS_BIT_OFFSET        = (int)24,
};

typedef enum
{	// Solution Stream Messages
	SOL_STREAM_DISABLED		= 0,	// Disable solution streaming
	// Post Process Data (PPD) = DID_GPS + DID_MAGNETOMETER1 + DID_MAGNETOMETER2 + DID_BAROMETER + DID_FLASH_CONFIG
	// PPD1 = PPD + DID_DUAL_IMU
	SOL_STREAM_PPD1_INS1	= 1,	// PPD1 + DID_INS1
	SOL_STREAM_PPD1_INS2	= 2,	// PPD1 + DID_INS2 (recommended default)
	SOL_STREAM_PPD1_INS3	= 3,	// PPD1 + DID_INS3
	SOL_STREAM_PPD1_INS4	= 4,	// PPD1 + DID_INS4
	// PPD2 = PPD + DID_DELTA_THETA_VEL (Conning and Sculling IMU)
	SOL_STREAM_PPD2_INS1	= 11,	// PPD2 + DID_INS1 
	SOL_STREAM_PPD2_INS2	= 12,	// PPD2 + DID_INS2
	SOL_STREAM_PPD2_INS3	= 13,	// PPD2 + DID_INS3
	SOL_STREAM_PPD2_INS4	= 14,	// PPD2 + DID_INS4
	// No post process data.  Only INS output @ NAV update rate (DID_FLASH_CONFIG.startupNavDtMs)
	SOL_STREAM_INS1			= 21,	// DID_INS1
	SOL_STREAM_INS2			= 22,	// DID_INS2
	SOL_STREAM_INS3			= 23,	// DID_INS3
	SOL_STREAM_INS4			= 24,	// DID_INS4
} eSolStreamControl;

enum eSysConfigBits
{
	/*! Disable automatic baudrate detection on startup */
	SYS_CFG_BITS_DISABLE_AUTOBAUD						= (int)0x00000001,

	/*! Enable automatic mag recalibration */
	SYS_CFG_BITS_AUTO_MAG_RECAL							= (int)0x00000002,

	/*! Disable LEDs */
	SYS_CFG_BITS_DISABLE_LEDS							= (int)0x00000004,

	/*! RESERVED */
	SYS_CFG_BITS_RESERVED1								= (int)0x00000008,

	/*! Enable com manager pass through of ublox data */
	SYS_CFG_BITS_ENABLE_COM_MANAGER_PASS_THROUGH_UBLOX	= (int)0x00000010,

	/*! Enable RTK rover if available */
	SYS_CFG_BITS_RTK_ROVER								= (int)0x00000020,

	/*! Enable RTK base station if available */
	SYS_CFG_BITS_RTK_BASE_STATION						= (int)0x00000040,

	/*! RTK bit mask */
	SYS_CFG_BITS_RTK_MASK								= (SYS_CFG_BITS_RTK_ROVER | SYS_CFG_BITS_RTK_BASE_STATION),

	/*! Magnetometer recalibration.  0 = multi-axis, 1 = single-axis */
	SYS_CFG_BITS_MAG_RECAL_MODE_MASK					= (int)0x00000700,
	SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET					= 8,
#define SYS_CFG_BITS_MAG_RECAL_MODE(sysCfgBits) ((sysCfgBits&SYS_CFG_BITS_MAG_RECAL_MODE_MASK)>>SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET)

	/*! Disable magnetometer fusion */
	SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION			= (int)0x00000800,
};


PUSH_PACK_1

/*! (DID_DEV_INFO) Device information */
typedef struct PACKED
{
	/*! Reserved bits */
	uint32_t        reserved;

	/*! Serial number */
	uint32_t        serialNumber;

	/*! Hardware version */
	uint8_t         hardwareVer[4];

	/*! Firmware (software) version */
	uint8_t         firmwareVer[4];

	/*! Build number */
	uint32_t        buildNumber;

	/*! Communications protocol version */
	uint8_t         protocolVer[4];

	/*! Repository revision number */
	uint32_t        repoRevision;

	/*! Manufacturer name */
	char            manufacturer[DEVINFO_MANUFACTURER_STRLEN];

    /*! Build date, little endian order: [0] = status ('r'=release, 'd'=debug), [1] = year-2000, [2] = month, [3] = day.  Reversed byte order for big endian systems */
	uint8_t         buildDate[4];

    /*! Build date, little endian order: [0] = hour, [1] = minute, [2] = second, [3] = millisecond.  Reversed byte order for big endian systems */
	uint8_t         buildTime[4];

	/*! Additional info */
	char            addInfo[DEVINFO_ADDINFO_STRLEN];
} dev_info_t;

/*! (DID_MANUFACTURING_INFO) Manufacturing info */
typedef struct PACKED
{
	/*! Serial number */
	uint32_t		serialNumber;

	/*! Lot number */
	uint32_t		lotNumber;

	/*! Manufacturing date (YYYYMMDDHHMMSS) */
    char			date[16];

	/*! Key */
	uint32_t		key;
} manufacturing_info_t;


/*! (DID_INS_1) INS output: euler rotation w/ respect to NED, NED position from reference LLA */
typedef struct PACKED
{
	/*! Weeks since January 6th, 1980 */
	uint32_t				week;
	
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/*! INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
	uint32_t				insStatus;

	/*! Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
	uint32_t				hdwStatus;

	/*! Euler angles: roll, pitch, yaw in radians with respect to NED */
	float					theta[3];

	/*! Velocity U, V, W in meters per second.  Convert to NED velocity using "vectorBodyToReference( uvw, theta, vel_ned )". */
	float					uvw[3];

	/*! WGS84 latitude, longitude, height above ellipsoid (degrees,degrees,meters) */
	double					lla[3];

	/*! North, east and down offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
	float					ned[3];
} ins_1_t;


/*! (DID_INS_2) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
typedef struct PACKED
{
	/*! Weeks since January 6th, 1980 */
	uint32_t				week;
	
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/*! INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
	uint32_t				insStatus;

	/*! Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
	uint32_t				hdwStatus;

	/*! Quaternion body rotation with respect to NED: W, X, Y, Z */
	float					qn2b[4];

	/*! Velocity U, V, W in meters per second.  Convert to NED velocity using "quatRot(vel_ned, qn2b, uvw)". */
	float					uvw[3];

	/*! WGS84 latitude, longitude, height above ellipsoid in meters (not MSL) */
	double					lla[3];
} ins_2_t;


/*! (DID_INS_3) INS output: quaternion rotation w/ respect to NED, msl altitude */
typedef struct PACKED
{
	/*! Weeks since January 6th, 1980 */
	uint32_t				week;
	
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/*! INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
	uint32_t				insStatus;

	/*! Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
	uint32_t				hdwStatus;

	/*! Quaternion body rotation with respect to NED: W, X, Y, Z */
	float					qn2b[4];

	/*! Velocity U, V, W in meters per second.  Convert to NED velocity using "quatRot(vel_ned, qn2b, uvw)". */
	float					uvw[3];

	/*! WGS84 latitude, longitude, height above mean sea level (MSL) in meters */
	double					lla[3];
} ins_3_t;


/*! (DID_INS_4) INS output: quaternion rotation w/ respect to ECEF, ECEF position */
typedef struct PACKED
{
	/*! Weeks since January 6th, 1980 */
	uint32_t				week;
	
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/*! INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
	uint32_t				insStatus;

	/*! Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
	uint32_t				hdwStatus;

	/*! Quaternion body rotation with respect to ECEF: W, X, Y, Z */
	float					qe2b[4];

	/*! Velocity in ECEF (earth-centered earth-fixed) frame in meters per second */
	float					ve[3];

	/*! Position in ECEF (earth-centered earth-fixed) frame in meters */
	double					ecef[3];
} ins_4_t;


typedef struct PACKED
{
	/*! Gyroscope P, Q, R in radians / second */
	float                   pqr[3];

	/*! Acceleration X, Y, Z in meters / second squared */
	float                   acc[3];
} imus_t;


/*! (DID_IMU_1, DID_IMU_2) Inertial Measurement Unit (IMU) data */
typedef struct PACKED
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/*! Inertial Measurement Unit (IMU) */
	imus_t					I;
} imu_t;


/*! (DID_DUAL_IMU) Dual Inertial Measurement Units (IMUs) data */
typedef struct PACKED
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/*! Inertial Measurement Units (IMUs) */
	imus_t                  I[2];
} dual_imu_t;


/*! (DID_MAGNETOMETER_1, DID_MAGNETOMETER_2) Magnetometer sensor data */
typedef struct PACKED
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;
	
	/*! Magnetometers in Gauss */
	float                   mag[3];
} magnetometer_t;


/*! (DID_BAROMETER) Barometric pressure sensor data */
typedef struct PACKED
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;
	
	/*! Barometric pressure in kilopascals */
	float                   bar;

	/*! MSL altitude from barometric pressure sensor in meters */
	float                   mslBar;

	/*! Temperature of barometric pressure sensor in Celsius */
	float                   barTemp;

	/*! Relative humidity as a percent (%rH). Range is 0% - 100% */
	float                   humidity;
} barometer_t;


/*! (DID_DELTA_THETA_VEL) Coning and sculling integral in body/IMU frame.  Updated at IMU rate. */
typedef struct PACKED
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/*! Delta theta body frame (gyroscope P, Q, R integral) in radians */
	float                   theta[3];

	/*! Delta velocity body frame (acceleration X, Y, Z integral) in meters / second */
	float                   uvw[3];

	/*! Delta time for delta theta and delta velocity in seconds */
	float					dt;
} delta_theta_vel_t;


/*! (DID_GPS_POS) GPS position */
typedef struct PACKED
{
	/*! Number of weeks since January 6th, 1980 */
	uint32_t                week;
	
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! (eGpsStatus) GPS status: [7:0] number of satellites used in solution, [15:8] status flags, [23:16] fix type */
	uint32_t                status;

	/*! Carrier to noise ratio (signal strength) of strongest satellite in dBHz */
	uint32_t                cno;

	/*! WGS84 Latitude, longitude, height above ellipsoid (not MSL) in degrees, degrees, meters */
	double					lla[3];

	/*! Height above mean sea level (MSL) in meters */
	float					hMSL;

	/*! Horizontal accuracy in meters */
	float					hAcc;

	/*! Vertical accuracy in meters */
	float					vAcc;

	/*! Position dilution of precision in meters */
	float                   pDop;
} gps_nav_poslla_t;

	
/*! (DID_GPS_VEL) GPS velocity */
typedef struct PACKED
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t				timeOfWeekMs;
		
	/*! North, east and down velocity in meters / second */
	float					ned[3];

	/*! Ground speed magnitude in meters / second (always positive) */
	float					s2D;

	/*! 3D speed magnitude in meters / second */
	float					s3D;

	/*! Speed accuracy in meters / second */
	float					sAcc;

	/*! Velocity ground course (heading) in radians */
	float					course;

	/*! Velocity ground course accuracy in radians */
	float					cAcc;
} gps_nav_velned_t;


/*! (DID_GPS) GPS Data */
typedef struct PACKED
{
	/*! GPS position */
	gps_nav_poslla_t		pos;
	
	/*! GPS velocity */
	gps_nav_velned_t		vel;

	/*! Number of GPS messages received per second */
	uint32_t				rxps;

	/*! Time sync offset between local time since boot up to time of week in seconds */
	double                  towOffset;
} gps_t;


/*! GPS satellite information */
typedef struct PACKED
{
	/*!
	Satellite identifier
	- GPS:     0, 0-32
	- SBAS:    1, 20-58
	- Galileo: 2, 0-36
	- BeiDou:  3, 0-37
	- IMES:    4, 0-10
	- QZSS:    5, 0-5
	- GLONASS: 6, 0-32
	- svId % 100 = satellite id (0 = unknown)
	- svId / 100 = constellation (0 = GPS, 1 = SBS, 2 = GAL, 3 = BEI, 4 = IMES, 5 = QZS, 6 = GLO)
	*/
	uint32_t				svId;

	/*! Carrier to noise ratio (signal strength) in  dBHz */
	uint32_t				cno;
} gps_sat_info_t;


/*! (DID_GPS_CNO) GPS satellite signal strength */
typedef struct PACKED
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! Number of satellites in the sky */
	uint32_t				numSats;

	/*! Satellite information list */
	gps_sat_info_t			info[MAX_NUM_SAT_CHANNELS];
} gps_cno_t;


/*! (DID_GPS_VERSION) GPS version strings */
typedef struct PACKED
{
	uint8_t                 swVersion[30];
	uint8_t                 hwVersion[10];
	uint8_t                 extension[30];
	uint8_t					reserved[2];		// ensure 32 bit aligned in memory
} gps_version_t;


/*! (DID_ASCII_BCAST_PERIOD) ASCII broadcast periods. This data structure (when it is included in the sCommData struct) is zeroed out on stop_all_broadcasts */ 
typedef struct PACKED
{
	/*! Broadcast period for ASCII IMU data in milliseconds. 0 for none */
	uint32_t                 imu;

	/*! Broadcast period for ASCII INS 1 data in milliseconds. 0 for none */
	uint32_t                 ins1;

	/*! Broadcast period for ASCII INS 2 data in milliseconds. 0 for none */
	uint32_t                 ins2;

	/*! Broadcast period for ASCII GPS position data in milliseconds. 0 for none */
	uint32_t                 gpsPos;

	/*! Broadcast period for ASCII GPS velocity data in milliseconds. 0 for none */
	uint32_t                 gpsVel;

	/*! Broadcast period for GGA (NMEA) data in milliseconds. 0 for none */
	uint32_t				 gga;

	/*! Broadcast period for GLL (NMEA) data in milliseconds. 0 for none */
	uint32_t				 gll;

	/*! Broadcast period for GSA (NMEA) data in milliseconds. 0 for none */
	uint32_t				 gsa;

	/*! Broadcast period for ASCII delta theta velocity data in milliseconds. 0 for none */
	uint32_t                 dtv;
} ascii_msgs_t;

/*! Generic 1 axis sensor */
typedef struct PACKED
{
	/*! Time in seconds */
	double                  time;

	/*! Three axis sensor */
	float                   val;
} gen_1axis_sensor_t;

/*! Generic 3 axis sensor */
typedef struct PACKED
{
	/*! Time in seconds */
	double                  time;

	/*! Three axis sensor */
	float                   val[3];
} gen_3axis_sensor_t;

/*! Generic dual 3 axis sensor */
typedef struct PACKED
{
	/*! Time in seconds */
	double                  time;

	/*! First three axis sensor */
	float                   val1[3];

	/*! Second three axis sensor */
	float                   val2[3];
} gen_dual_3axis_sensor_t;

/*! Generic 3 axis sensor */
typedef struct PACKED
{
	/*! Time in seconds */
	double                  time;

	/*! Three axis sensor */
	double                  val[3];
} gen_3axis_sensord_t;

/*! (DID_SYS_SENSORS) Output from system sensors */
typedef struct PACKED
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double					time;

	/*! Temperature in Celsius */
	float                   temp;

	/*! Gyros in radians / second */
	float                   pqr[3];

	/*! Accelerometers in meters / second squared */
	float                   acc[3];

	/*! Magnetometers in Gauss */
	float                   mag[3];

	/*! Barometric pressure in kilopascals */
	float                   bar;

	/*! Temperature of barometric pressure sensor in Celsius */
	float                   barTemp;

	/*! MSL altitude from barometric pressure sensor in meters */
	float                   mslBar;
	
	/*! Relative humidity as a percent (%rH). Range is 0% - 100% */
	float                   humidity;

	/*! EVB system input voltage in volts. uINS pin 5 (G2/AN2).  Use 10K/1K resistor divider between Vin and GND.  */
	float                   vin;

	/*! ADC analog input in volts. uINS pin 4, (G1/AN1). */
	float                   ana1;

	/*! ADC analog input in volts. uINS pin 19 (G3/AN3). */
	float                   ana3;

	/*! ADC analog input in volts. uINS pin 20 (G4/AN4). */
	float                   ana4;
} sys_sensors_t;

/*! Sensor state variables */
typedef struct PACKED
{
	/*! Latitude, longitude and height above ellipsoid in radians, radians and meters */
	double                  lla[3];

	/*! Velocities in body frames of X, Y and Z in meters per second */
	float                   uvw[3];

	/*! Quaternion body rotation with respect to NED: W, X, Y, Z */
	float					qn2b[4];
} state_vars_t;

/*! (DID_INS_MISC) INS Misc data */
typedef struct PACKED
{
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double                  timeOfWeek;

	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;
	
	/*! State variables */
	state_vars_t            x;
	
	/*! Euler angles: roll, pitch, yaw (radians) */
	float                   theta[3];

	/*! North, east and down offset between reference and current latitude, longitude and altitude in meters */
	float                   ned[3];

	/*! Inertial to body frame DCM (Direct cosine matrix) */
	float                   dcm[9];

	/*! Body rates (INS bias estimates removed) in radians per second */
	float                   pqr[3];

	/*! Body accelerations (INS bias estimates removed) in meters per second squared */
	float                   acc[3];

	/*! Body magnetic in Gauss */
	float                   mag[3];

	/*! MSL altitude in meters from barometric pressure */
	float					mslBar;
} ins_misc_t;

/*! INS output */
typedef struct PACKED
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! State variables */
	state_vars_t            x;

	/*! Euler angles (roll, pitch, yaw) in radians */
	float                   theta[3];

	/*! North, east and down offset between reference and current latitude, longitude and altitude in meters */
	float                   ned[3];

	/*! Inertial to body frame DCM (Direction cosine matrix) */
	float                   dcm[9];
} ins_output_t;

/*! (DID_SYS_PARAMS) System parameters */
typedef struct PACKED
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! System status 1 flags (eInsStatusFlags) */
	uint32_t                insStatus;

	/*! System status 2 flags (eHdwStatusFlags) */
	uint32_t                hdwStatus;

	/*! Attitude alignment detection */
	float				    alignAttDetect;

	/*! Attitude alignment error (approximation) in radians */
	float				    alignAttError;

	/*! Velocity alignment error in meters per second */
	float				    alignVelError;

	/*! Position alignment error in meters per second */
	float				    alignPosError;

	/*! Sample period in milliseconds. Zero disables sampling. */
	uint32_t				sampleDtMs;

	/*! Nav filter update period in milliseconds. Zero disables nav filter. */
	uint32_t				navDtMs;

	/*! Ratio of system tuned clock to actual clock frequencies */
	float					ftf0; 

	/*! Magnetic north inclination (negative pitch offset) in radians */
	float                   magInclination;

	/*! Magnetic north declination (heading offset from true north) in radians */
	float                   magDeclination;

	/*! Earth magnetic field (magnetic north) magnitude (nominally 1) */
	float                   magMagnitude;
	
	/*! General fault code descriptor */
	uint32_t                genFaultCode;
} sys_params_t;

/*! On demand messages - these messages are sent whenever available, rather than sent or received at a specific rate */
typedef enum
{
	/*! Raw observation and ephemeris data */
	MSG_CFG_BIT_RTK = 1 << 0
} eMsgCfgBits;

/*! (DID_CONFIG) Configuration functions */
typedef struct PACKED
{
	/*! Set to 1 to reset processor into bootloader mode */
	uint32_t                enBootloader;

	/*! Solution logging options (eSolStreamControl) */
	uint32_t                solStreamCtrl;

	/*! Set to 1 to enable sensor stats */
	uint32_t                enSensorStats;

	/*! Set to 1 to enable RTOS stats */
	uint32_t                enRTOSStats;

	/*! Set to 1 to enable GPS low-level configuration */
	uint32_t                gpsStatus;

	/*! System: 99=software reset */
	uint32_t                system;

	/*! Message configuration bits, on demand messages (eMsgCfgBits) */
	uint32_t				msgCfgBits;
} config_t;

#define NUM_SERVOS			8
#define SERVO_PULSE_US_MIN	700
#define SERVO_PULSE_US_MAX	2300

/*! (DID_IO) Input/Output */
typedef struct PACKED
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! General purpose I/O status */
	uint32_t				gpioStatus;
} io_t;

/*! (DID_IO_SERVOS_PWM & DID_IO_SERVOS_PPM) I/O: PWM and PPM Servos */
typedef struct PACKED
{
	/*! Servo pulse time (us) */
	uint32_t				ch[NUM_SERVOS];
} io_servos_t;

enum eMagRecalMode
{
	MAG_RECAL_MODE_MULTI_AXIS		= (int)0,
	MAG_RECAL_MODE_SINGLE_AXIS		= (int)1,
	MAG_RECAL_MODE_COMPLETE			= (int)100,
};

/*! (DID_MAG_CAL) Magnetometer Calibration */
typedef struct PACKED
{
	/*! Set mode and start recalibration.  0 = multi-axis, 1 = single-axis, 100 = done. */
	uint32_t                enMagRecal;
	
	/*! Mag recalibration progress indicator: 0-100 % */
	float					progress;

	/*! Magnetic declination estimate */
	float					declination;
} mag_cal_t;


/*! Built-in test state */
enum eBitState
{
	BIT_STATE_OFF					= (int)0,
	BIT_STATE_DONE					= (int)1,
	BIT_STATE_START					= (int)2,
	BIT_STATE_ONE_CHECK				= (int)(BIT_STATE_START+1),
};

/*! Hardware built-in test flags */
enum eHdwBitStatusFlags
{
	HDW_BIT_PASSED_MASK				= (int)0x0000000F,
	HDW_BIT_PASSED_ALL				= (int)0x00000001,
	HDW_BIT_PASSED_AHRS				= (int)0x00000002,	// Passed w/o valid GPS signal
	HDW_BIT_FAILED_MASK				= (int)0xFFFFFFF0,
	HDW_BIT_FAULT_NOISE_PQR			= (int)0x00000010,
	HDW_BIT_FAULT_NOISE_ACC			= (int)0x00000020,
	HDW_BIT_FAULT_GPS_NO_COM		= (int)0x00000100,	// No GPS serial communications
	HDW_BIT_FAULT_GPS_POOR_CNO		= (int)0x00000200,	// Poor GPS signal strength.  Check antenna
	HDW_BIT_FAULT_GPS_ACCURACY		= (int)0x00000200,	// Low number of satellites, or bad accuracy 
	HDW_BIT_FAULT_GPS_NOISE			= (int)0x00000400,	// (Not implemented)
};

/*! Calibration built-in test flags */
enum eCalBitStatusFlags
{
	CAL_BIT_PASSED_MASK				= (int)0x0000000F,
	CAL_BIT_PASSED_ALL				= (int)0x00000001,
	CAL_BIT_FAILED_MASK				= (int)0xFFFFFFF0,
	CAL_BIT_FAULT_TCAL_EMPTY		= (int)0x00000010,	// Temperature calibration not present
	CAL_BIT_FAULT_TCAL_TSPAN		= (int)0x00000020,	// Temperature calibration temperature range is inadequate
	CAL_BIT_FAULT_TCAL_INCONSISTENT	= (int)0x00000040,	// Temperature calibration number of points or slopes are not consistent
	CAL_BIT_FAULT_TCAL_CORRUPT		= (int)0x00000080,	// Temperature calibration memory corruption
	CAL_BIT_FAULT_TCAL_PQR_BIAS		= (int)0x00000100,	// Temperature calibration gyro bias
	CAL_BIT_FAULT_TCAL_PQR_SLOPE	= (int)0x00000200,	// Temperature calibration gyro slope
	CAL_BIT_FAULT_TCAL_PQR_LIN		= (int)0x00000400,	// Temperature calibration gyro linearity
	CAL_BIT_FAULT_TCAL_ACC_BIAS		= (int)0x00000800,	// Temperature calibration accelerometer bias
	CAL_BIT_FAULT_TCAL_ACC_SLOPE	= (int)0x00001000,	// Temperature calibration accelerometer slope
	CAL_BIT_FAULT_TCAL_ACC_LIN		= (int)0x00002000,	// Temperature calibration accelerometer linearity
	CAL_BIT_FAULT_ORTO_EMPTY		= (int)0x00010000,	// Cross-axis alignment is not calibrated
	CAL_BIT_FAULT_ORTO_INVALID		= (int)0x00020000,	// Cross-axis alignment is poorly formed
	CAL_BIT_FAULT_MOTION_PQR		= (int)0x00040000,	// Motion on gyros
	CAL_BIT_FAULT_MOTION_ACC		= (int)0x00080000,	// Motion on accelerometers
};

/*! (DID_BIT) Built-in self-test parameters */
typedef struct PACKED
{
	/*! Built-in self-test state */
	uint32_t                state;

	/*! Hardware BIT status (see eHdwBitStatusFlags) */
	uint32_t                hdwBitStatus;

	/*! Calibration BIT status (see eCalBitStatusFlags) */
	uint32_t                calBitStatus;

	/*! Temperature calibration bias */
	float                   tcPqrBias;
	float                   tcAccBias;

	/*! Temperature calibration slope */
	float                   tcPqrSlope;
	float                   tcAccSlope;

	/*! Temperature calibration linearity */
	float                   tcPqrLinearity;
	float                   tcAccLinearity;

	/*! PQR motion (angular rate) */
	float                   pqr;

	/*! ACC motion w/ gravity removed (linear acceleration) */
	float                   acc;

	/*! Angular rate standard deviation */
	float                   pqrSigma;

	/*! Acceleration standard deviation */
	float                   accSigma;
	
} bit_t;

/*! (DID_FLASH_CONFIG) Configuration data */
typedef struct PACKED
{
	/*! Size of group or union, which is nvm_group_x_t + padding */
	uint32_t				size;

	/*! Checksum, excluding size and checksum */
	uint32_t                checksum;

	/*! Manufacturer method for restoring flash defaults */
	uint32_t                key;

	/*! IMU sample (system input data) period in milliseconds set on startup. Cannot be larger than startupNavDtMs. Zero disables sensor/IMU sampling. */
	uint32_t				startupSampleDtMs;

	/*! Nav filter update (system output data) period in milliseconds set on startup. 2ms minimum (500Hz max). Zero disables nav filter updates. */
	uint32_t				startupNavDtMs;

	/*! Serial port 0 baud rate in bits per second */
	uint32_t				ser0BaudRate;
	
	/*! Serial port 1 baud rate in bits per second */
	uint32_t				ser1BaudRate;

	/*! Roll, pitch, yaw euler angle rotation in radians from INS Sensor Frame to Intermediate Output Frame.  Order applied: heading, pitch, roll. */
	float					insRotation[3];

	/*! X,Y,Z offset in meters from Intermediate Output Frame to INS Output Frame. */
	float					insOffset[3];

	/*! X,Y,Z offset in meters from Sensor Frame origin to GPS antenna. */
	float					gpsAntOffset[3];

	/* INS dynamic platform model.  Determines performance characteristics of system. 0=PORTABLE, 2=STATIONARY, 3=PEDESTRIAN, 4=AUTOMOTIVE, 5=SEA, 6=AIRBORNE_1G, 7=AIRBORNE_2G, 8=AIRBORNE_4G, 9=WRIST */
	uint32_t				insDynModel;
	
	/*! System configuration bits (see eSysConfigBits) */
	uint32_t				sysCfgBits;

	/*! Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations (deg, deg, m) */
	double                  refLla[3];

	/*! Last latitude, longitude, HAE (height above ellipsoid) used to aid GPS startup (deg, deg, m) */
	double					lastLla[3];

	/*! Last LLA time since week start (Sunday morning) in milliseconds */
	uint32_t				lastLlaTimeOfWeekMs;

	/*! Last LLA number of weeks since January 6th, 1980 */
	uint32_t				lastLlaWeek;
	
	/*! Distance between current and last LLA that triggers an update of lastLla  */
	float					lastLlaUpdateDistance;

	/*! Hardware interface configuration bits */
	uint32_t				ioConfig;

	/*! Carrier board (i.e. eval board) configuration bits */
	uint32_t				cBrdConfig;

	/*! Servo failsafe trigger time in microseconds. Set to zero to disable all failsafes */
	uint32_t				servoFailsafeTriggerUs;

	/*! Servo failsafe pulse time in microseconds. Set to zero to disable failsafe. */
	uint32_t				servoFailsafePulseUs[NUM_SERVOS];

	/*! Earth magnetic field (magnetic north) inclination (negative pitch offset) in radians */
	float                   magInclination;

	/*! Earth magnetic field (magnetic north) declination (heading offset from true north) in radians */
	float                   magDeclination;

	/*! Earth magnetic field (magnetic north) magnitude (nominally 1) */
	float                   magMagnitude;

	/*! Magnetometer bias estimate in body frame (normalized gauss) */
	float					magB[3];
} nvm_flash_cfg_t;

/*! (DID_INS_RESOURCES) */
typedef struct PACKED
{	
	uint32_t                timeOfWeekMs;		//			Time of week (since Sunday morning) in milliseconds, GMT
	state_vars_t            x_dot;				//			State variables derivative
	float					magYawOffset;		// (rad)	Temporary offset in mag heading used to remove discontinuities when transitioning from moving to stationary (from GPS to mag heading)
} ins_res_t;

POP_PACK

PUSH_PACK_8

/*! time struct */
typedef struct
{
	/*! time (s) expressed by standard time_t */
	time_t time;

	/*! fraction of second under 1 s */
	double sec;         
} gtime_t;

POP_PACK

PUSH_PACK_1

/*! RTK processing options */
typedef struct
{
	/*! positioning mode (PMODE_???) */
	int32_t mode;           

	/*! solution type (0:forward,1:backward,2:combined) */
	int32_t soltype;

	/*! number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
	int32_t nf;

	/*! navigation systems */
	int32_t navsys;

	/*! elevation mask angle (rad) */
	double elmin;

	/*! AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
	int32_t modear;

	/*! GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
	int32_t glomodear;

	/*! GPS AR mode (0:off,1:on) */
	int32_t gpsmodear;

	/*! BeiDou AR mode (0:off,1:on) */
	int32_t bdsmodear;

	/*! AR filtering to reject bad sats (0:off,1:on) */
	int32_t arfilter;

	/*! obs outage count to reset bias */
	int32_t maxout;

	/*! min lock count to fix ambiguity */
	int32_t minlock;

	/*! min sats to fix integer ambiguities */
	int32_t minfixsats;

	/*! min sats to hold integer ambiguities */
	int32_t minholdsats;

	/*! min sats to drop sats in AR */
	int32_t mindropsats;

	/*! use stdev estimates from receiver to adjust measurement variances */
	int32_t rcvstds;

	/*! min fix count to hold ambiguity */
	int32_t minfix;

	/*! max iteration to resolve ambiguity */
	int32_t armaxiter;

	/*! dynamics model (0:none,1:velociy,2:accel) */
	int32_t dynamics;

	/*! number of filter iteration */
	int32_t niter;

	/*! interpolate reference obs (for post mission) */
	int32_t intpref;

	/*! rover position for fixed mode */
	int32_t rovpos;

	/*! base position for relative mode */
	int32_t refpos;

	/*! code/phase error ratio */
	double eratio[1];

	/*! measurement error factor */
	double err[5];

	/*! initial-state std [0]bias,[1]iono [2]trop */
	double std[3];

	/*! process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
	double prn[6];

	/*! satellite clock stability (sec/sec) */
	double sclkstab;

	/*! AR validation threshold */
	double thresar[8];

	/*! elevation mask of AR for rising satellite (rad) */
	double elmaskar;

	/*! elevation mask to hold ambiguity (rad) */
	double elmaskhold;

	/*! slip threshold of geometry-free phase (m) */
	double thresslip;

	/*! variance for fix-and-hold psuedo measurements (cycle^2) */
	double varholdamb;

	/*! gain used for GLO and SBAS sats to adjust ambiguity */
	double gainholdamb;

	/*! max difference of time (sec) */
	double maxtdiff;

	/*! reject threshold of innovation (m) */
	double maxinno;

	/*! reject threshold of gdop */
	double maxgdop;

	/*! baseline length constraint {const,sigma} (m) */
	double baseline[2];

	/*! rover position for fixed mode {x,y,z} (ecef) (m) */
	double ru[3];

	/*! base position for relative mode {x,y,z} (ecef) (m) */
	double rb[3];

	/*! max averaging epoches */
	int32_t maxaveep;

	/*! output single by dgps/float/fix/ppp outage */
	int32_t outsingle;
} prcopt_t;
typedef prcopt_t rtk_opt_t;

/*! Raw satellite observation data */
typedef struct PACKED
{
	/*! receiver sampling time (GPST) */
	gtime_t time;

	/*! satellite number */
	uint8_t sat;

	/*! receiver number */
	uint8_t rcv;

	/*! signal strength (0.25 dBHz) */
	uint8_t SNR[1];

	/*! loss of lock indicator */
	uint8_t LLI[1];

	/*! code indicator (CODE_???) */
	uint8_t code[1];

	/*! quality of carrier phase measurement */
	uint8_t qualL[1];

	/*! quality of pseudorange measurement */
	uint8_t qualP[1];

	/*! reserved, for alignment */
	uint8_t reserved1;

	/*! observation data carrier-phase (cycle) */
	double L[1];

	/*! observation data pseudorange (m) */
	double P[1]; 

	/*! observation data doppler frequency (Hz) */
	float D[1];

	/*! reserved, for alignment */
	uint32_t reserved2;
} obsd_t;

/*! observation data */
typedef struct
{
	/*! number of obervation slots used */
	int32_t n;

	/*! number of obervation slots allocated */
	int32_t nmax;

	/*! observation data buffer */
	obsd_t* data;
} obs_t;

/*! GPS / GAL / QZS ephemeris data */
typedef struct
{
	/* satellite number */
	int32_t sat;

	/*! IODE */
	int32_t iode;
	
	/*! IODC */
	int32_t iodc;

	/*! SV accuracy (URA index) */
	int32_t sva;            

	/*! SV health (0:ok) */
	int32_t svh;            

	/* GPS/QZS: gps week, GAL: galileo week */
	int32_t week;

	/*! GPS/QZS: code on L2, GAL/CMP: data sources */
	int32_t code;

	/*! GPS/QZS: L2 P data flag, CMP: nav type */
	int32_t flag;

	/*! Toe */
	gtime_t toe;
	
	/*! Toc */
	gtime_t toc;
	
	/*! T_trans */
	gtime_t ttr;

	/*! SV orbit parameters - A */
	double A;

	/*! SV orbit parameters - e */
	double e;

	/*! SV orbit parameters - i0 */
	double i0;

	/*! SV orbit parameters - OMG0 */
	double OMG0;

	/*! SV orbit parameters - omg */
	double omg;

	/*! SV orbit parameters - M0 */
	double M0;

	/*! SV orbit parameters - deln */
	double deln;

	/*! SV orbit parameters - OMGd */
	double OMGd;

	/*! SV orbit parameters - idot */
	double idot;

	/*! SV orbit parameters - crc */
	double crc;

	/*! SV orbit parameters - crs */
	double crs;

	/*! SV orbit parameters - cuc */
	double cuc;

	/*! SV orbit parameters - cus */
	double cus;

	/*! SV orbit parameters - cic */
	double cic;

	/*! SV orbit parameters - cis */
	double cis;

	/*! Toe (s) in week */
	double toes;

	/*! fit interval (h) */
	double fit;

	/*! SV clock parameters - af0 */
	double f0;
	
	/*! SV clock parameters - af1 */
	double f1;
	
	/*! SV clock parameters - af2 */
	double f2;

	/*! group delay parameters
	* GPS/QZS:tgd[0]=TGD
	* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1
	* CMP    :tgd[0]=BGD1,tgd[1]=BGD2
	*/
	double tgd[4];

	/*! Adot for CNAV */
	double Adot;
	
	/*! ndot for CNAV */
	double ndot;
} eph_t;

/*! Glonass ephemeris data */
typedef struct
{        
	/*! satellite number */
	int32_t sat;

	/*! IODE (0-6 bit of tb field) */
	int32_t iode;

	/*! satellite frequency number */
	int32_t frq;

	/*! satellite health */
	int32_t svh;
	
	/*! satellite accuracy */
	int32_t sva;
	
	/*! satellite age of operation */
	int32_t age;

	/*! epoch of epherides (gpst) */
	gtime_t toe;

	/*! message frame time (gpst) */
	gtime_t tof;

	/*! satellite position (ecef) (m) */
	double pos[3];

	/*! satellite velocity (ecef) (m/s) */
	double vel[3];

	/*! satellite acceleration (ecef) (m/s^2) */
	double acc[3];

	/*! SV clock bias (s) */
	double taun;

	/*! relative freq bias */
	double gamn;

	/*! delay between L1 and L2 (s) */
	double dtaun;
} geph_t;

/*! SBAS message type */
typedef struct
{
	/* receiption time - week */
	int32_t week;
	
	/*! reception time - tow */
	int32_t tow;

	/*! SBAS satellite PRN number */
	int32_t prn;

	/*! SBAS message (226bit) padded by 0 */
	uint8_t msg[29];

	/*! reserved for alighment */
	uint8_t reserved[3];
} sbsmsg_t;

/*! station parameter type */
typedef struct
{
	/*! antenna delta type (0:enu,1:xyz) */
	int32_t deltype;    

	/*! station position (ecef) (m) */
	double pos[3];

	/*! antenna position delta (e/n/u or x/y/z) (m) */
	double del[3];

	/*! antenna height (m) */
	double hgt;
} sta_t;

/*! RTK solution status */
typedef enum
{
	/*! No status */
	rtk_solution_status_none = 0,

	/*! RTK fix */
	rtk_solution_status_fix = 1,

	/*! RTK float */
	rtk_solution_status_float = 2,

	/*! RTK SBAS */
	rtk_solution_status_sbas = 3,

	/*! RTK DGPS */
	rtk_solution_status_dgps = 4,

	/*! RTK SINGLE */
	rtk_solution_status_single = 5
} eRtkSolStatus;

/*! DID_RTK_SOL - requires little endian CPU */
typedef struct PACKED
{
	/*! Solution status - eRtkSolStatus */
	uint32_t status;

	/*! Seconds in GPST */
	double seconds;

	/*! Position - latitude (degrees), longitude (degrees), height (meters) */
	double pos[3];

	/*! Velocity (xyz, meters per second) */
	double vel[3];

	/*! Accuracy - sdn, sde, sdu, sdne, sdeu, sdun (meters) */
	float accuracy[6];

	/*! Number of satellites in solution */
	uint32_t numberOfSatellites;

	/*! Age of differential, seconds */
	float age;

	/*! Ambiguity resolution ratio factor for valiation */
	float ratio;

	/*! Ambiguity resolution threshold for valiation */
	float threshold;

	/*! Geometric dilution of precision in meters */
	double gdop;
	
	/*! Position dilution of precision in meters */
	double pdop;
	
	/*! Horizontal dilution of precision in meters */
	double hdop;
	
	/*! Vertical dilution of precision in meters */
	double vdop;
} rtk_sol_t;

/*! RAW data types for DID_RAW_GPS_DATA */
typedef enum
{
	/*! obsd_t */
	raw_data_type_observation = 1,

	/*! eph_t */
	raw_data_type_ephemeris = 2,

	/*! geph_t */
	raw_data_type_glonass_ephemeris = 3,

	/*! sbsmsg_t */
	raw_data_type_sbas = 4,

	/*! sta_t */
	raw_data_type_base_station_antenna_position = 5
} eRawDataType;

/*! Message wrapper for DID_RAW_GPS_DATA - simply cast the data buffer to this struct */
typedef struct PACKED
{
	/*! Receiver index, 0 or 1 */
	uint8_t receiverIndex;

	/*! Type of message - eRawDataType */
	uint8_t type; 

	/*! number of messages of type */
	uint8_t count;

	/*! Reserved */
	uint8_t reserved;

	/*! Data buffer */
	uint8_t buf[1020];
} raw_gps_msg_t;

/*!
* Rtos task types
*/
typedef enum
{
    /*!
    * Sample task
    */
    TASK_SAMPLE = 0,

    /*!
    * Nav task
    */
    TASK_NAV,

    /*!
    * Communications task
    */
    TASK_COMMUNICATIONS,

    /*!
    * Maintenance task
    */
    TASK_MAINTENANCE,

    /*!
    * Idle task
    */
    TASK_IDLE,

    /*!
    * Timer task
    */
    TASK_TIMER,

    /*!
    * Number of rtos tasks
    */
    RTOS_NUM_TASKS                       // Keep last
} eRtosTask;

/*!
* Max task name length - do not change
*/
#define MAX_TASK_NAME_LEN 12

/*!
* Rtos task info
*/
typedef struct PACKED
{
    /*!
    * Task name
    */
    char                    name[MAX_TASK_NAME_LEN];

    /*!
    * Task priority (0 - 8)
    */
    uint32_t                priority;

    /*!
    * Stack high water mark 32 bit words
    */
    uint32_t                stackUnused;

    /*!
    * Task period ms
    */
    uint32_t                periodMs;

    /*!
    * Last run time microseconds
    */
    uint32_t                 runTimeUs;

    /*!
    * Max run time microseconds
    */
    uint32_t                 maxRunTimeUs;

    /*!
    * Cpu usage percent
    */
    float					cpuUsage;

    /*!
    * Handle
    */
    uint32_t                handle;
} rtos_task_t;

/*! (DID_RTOS_INFO) */
typedef struct PACKED
{
    /*! Tasks */
    rtos_task_t             task[RTOS_NUM_TASKS];

    /*! Heap high water mark */
    uint32_t                freeHeapSize;
} rtos_info_t;

/*! Union of datasets */
typedef union PACKED
{
    dev_info_t			devInfo;
    ins_1_t				ins1;
    ins_2_t				ins2;
    ins_3_t				ins3;
    ins_4_t				ins4;
    imu_t				imu;
    dual_imu_t          dualImu;
    magnetometer_t		mag;
    barometer_t			baro;
    delta_theta_vel_t	dThetaVel;
    gps_t				gps;
    gps_nav_poslla_t	gpsPos;
    gps_nav_velned_t	gpsVel;
    gps_cno_t			gpsCNO;
    nvm_flash_cfg_t		flashCfg;
    ins_misc_t			insMisc;
    sys_params_t		sysParams;
    sys_sensors_t		sysSensors;
    io_t				io;
    ins_res_t			insRes;
    rtos_info_t			rtosInfo;
    rtk_sol_t			rtkSol;
    raw_gps_msg_t       gpsRaw;
} uDatasets;

/*! Union of INS output datasets */
typedef union PACKED
{
	ins_1_t				ins1;
	ins_2_t				ins2;
	ins_3_t				ins3;
	ins_4_t				ins4;
} uInsOutDatasets;

POP_PACK

/*!
Creates a 32 bit checksum from data

@param data the data to create a checksum for
@param count the number of bytes in data

@return the 32 bit checksum for data
*/
uint32_t serialNumChecksum32(const void* data, int size);
uint32_t flashChecksum32(const void* data, int size);

/*!
Flip the endianess of 32 bit values in data

@param data the data to flip 32 bit values in
@param dataLength the number of bytes in data
*/
void flipEndianess32(uint8_t* data, int dataLength);

/*!
Flip the bytes of a float in place (4 bytes) - ptr is assumed to be at least 4 bytes

@param ptr the float to flip
*/
void flipFloat(uint8_t* ptr);

/*!
Flip the bytes of a float (4 bytes) - ptr is assumed to be at least 4 bytes

@param val the float to flip
@return the flipped float
*/
float flipFloatCopy(float val);

/*!
Flip the bytes of a double in place (8 bytes) - ptr is assumed to be at least 8 bytes
Only flips each 4 byte pair, does not flip the individual bytes within the pair

@param ptr the double to flip
*/
void flipDouble(uint8_t* ptr);

/*!
Flip the bytes of a double in place (8 bytes)
Unlike flipDouble, this also flips the individual bytes in each 4 byte pair

@param val the double to flip
@return the flipped double
*/
double flipDoubleCopy(double val);

/*!
Flip double (64 bit) floating point values in data

@param data the data to flip doubles in
@param dataLength the number of bytes in data
@param offset offset into data to start flipping at
@param offsets a list of offsets of all doubles in data, starting at position 0
@param offsetsLength the number of items in offsets
*/
void flipDoubles(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength);

/*!
Flip string values in data - this compensates for the fact that flipEndianess32 is called on all the data

@param data the data to flip string values in
@param dataLength the number of bytes in data
@param offset the offset into data to start flipping strings at
@param offsets a list of offsets and byte lengths into data where strings start at
@param offsetsLength the number of items in offsets, should be 2 times the string count
*/
void flipStrings(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength);

// BE_SWAP: if big endian then swap, else no-op
// LE_SWAP: if little endian then swap, else no-op
#if CPU_IS_BIG_ENDIAN
#define BE_SWAP64F(_i) flipDoubleCopy(_i)
#define BE_SWAP32F(_i) flipFloatCopy(_i)
#define BE_SWAP32(_i) (SWAP32(_i))
#define BE_SWAP16(_i) (SWAP16(_i))
#define LE_SWAP64F(_i) (_i)
#define LE_SWAP32F(_i) (_i)
#define LE_SWAP32(_i) (_i)
#define LE_SWAP16(_i) (_i)
#else // little endian
#define BE_SWAP64F(_i) (_i)
#define BE_SWAP32F(_i) (_i)
#define BE_SWAP32(_i) (_i)
#define BE_SWAP16(_i) (_i)
#define LE_SWAP64F(_i) flipDoubleCopy(_i)
#define LE_SWAP32F(_i) flipFloatCopy(_i)
#define LE_SWAP32(_i) (SWAP32(_i))
#define LE_SWAP16(_i) (SWAP16(_i))
#endif

/*!
Get the offsets of double (64 bit) floating point values given a data id

@param dataId the data id to get double offsets for
@param offsetsLength receives the number of double offsets

@return a list of offets of doubles or 0 if none
*/
uint16_t* getDoubleOffsets(eDataIDs dataId, uint16_t* offsetsLength);

/*!
Gets the offsets and lengths of strings given a data id

@param dataId the data id to get string offsets and lengths for
@param offsetsLength receives the number of items in the return value

@return a list of offsets and lengths of strings for the data id or 0 if none
*/
uint16_t* getStringOffsetsLengths(eDataIDs dataId, uint16_t* offsetsLength);

// taken from http://www.leapsecond.com/tools/gpsdate.c, uses UTC time
int32_t convertDateToMjd(int32_t year, int32_t month, int32_t day);
int32_t convertGpsToMjd(int32_t gpsCycle, int32_t gpsWeek, int32_t gpsSeconds);
void convertMjdToDate(int32_t mjd, int32_t* year, int32_t* month, int32_t* day);
void convertGpsToHMS(int32_t gpsSeconds, int32_t* hour, int32_t* minutes, int32_t* seconds);

gen_1axis_sensor_t gen1AxisSensorData(double time, const float val);
gen_3axis_sensor_t gen3AxisSensorData(double time, const float val[3]);
gen_dual_3axis_sensor_t genDual3AxisSensorData(double time, const float val1[3], const float val2[3]);
gen_3axis_sensord_t gen3AxisSensorDataD(double time, const double val[3]);

#ifdef __cplusplus
}
#endif

#endif // DATA_SETS_H
