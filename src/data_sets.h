/*
MIT LICENSE

Copyright 2014-2018 Inertial Sense, Inc. - http://inertialsense.com

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

// *****************************************************************************
// ****** InertialSense binary message Data Identification Numbers (DIDs) ****** 
// ******                                                                 ******
// ****** NEVER REORDER THESE VALUES!                                     ******
// *****************************************************************************
/** Data identifiers - these are unsigned int and #define because enum are signed according to C standard */
typedef uint32_t eDataIDs;

#define DID_NULL                        (eDataIDs)0  /** NULL (INVALID) */
#define DID_DEV_INFO                    (eDataIDs)1  /** (dev_info_t) Device information */
#define DID_CRASH_INFO                  (eDataIDs)2  /** (crash_info_t) Crash information */
#define DID_PREINTEGRATED_IMU           (eDataIDs)3  /** (preintegrated_imu_t) Coning and sculling integral in body/IMU frame.  Updated at IMU rate. Also know as Delta Theta Delta Velocity, or Integrated IMU. For clarification, we will use the name "Preintegrated IMU" through the User Manual. They are integrated by the IMU at IMU update rates (1KHz). These integrals are reset each time they are output. Preintegrated IMU data acts as a form of compression, adding the benefit of higher integration rates for slower output data rates, preserving the IMU data without adding filter delay. It is most effective for systems that have higher dynamics and lower communications data rates. */
#define DID_INS_1                       (eDataIDs)4  /** (ins_1_t) INS output: euler rotation w/ respect to NED, NED position from reference LLA. */
#define DID_INS_2                       (eDataIDs)5  /** (ins_2_t) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
#define DID_GPS1_UBX_POS                (eDataIDs)6  /** (gps_pos_t) GPS 1 position data from ublox receiver. */
#define DID_CONFIG                      (eDataIDs)7  /** (config_t) Configuration data */
#define DID_ASCII_BCAST_PERIOD          (eDataIDs)8  /** (ascii_msgs_t) Broadcast period for ASCII messages */
#define DID_RMC                         (eDataIDs)9  /** (rmc_t) Realtime Message Controller (RMC). The data sets available through RMC are driven by the availability of the data. The RMC provides updates from various data sources (i.e. sensors) as soon as possible with minimal latency. Several of the data sources (sensors) output data at different data rates that do not all correspond. The RMC is provided so that broadcast of sensor data is done as soon as it becomes available. All RMC messages can be enabled using the standard Get Data packet format. */
#define DID_SYS_PARAMS                  (eDataIDs)10 /** (sys_params_t) System parameters / info */
#define DID_SYS_SENSORS                 (eDataIDs)11 /** (sys_sensors_t) System sensor information */
#define DID_FLASH_CONFIG                (eDataIDs)12 /** (nvm_flash_cfg_t) Flash memory configuration */
#define DID_GPS1_POS                    (eDataIDs)13 /** (gps_pos_t) GPS 1 position data.  This comes from DID_GPS1_UBX_POS or DID_GPS1_RTK_POS, depending on whichever is more accurate. */
#define DID_GPS2_POS                    (eDataIDs)14 /** (gps_pos_t) GPS 2 position data */
#define DID_GPS1_SAT                    (eDataIDs)15 /** (gps_sat_t) GPS 1 GNSS and sat identifiers, carrier to noise ratio (signal strength), elevation and azimuth angles, pseudo range residual. */
#define DID_GPS2_SAT                    (eDataIDs)16 /** (gps_sat_t) GPS 2 GNSS and sat identifiers, carrier to noise ratio (signal strength), elevation and azimuth angles, pseudo range residual. */
#define DID_GPS1_VERSION                (eDataIDs)17 /** (gps_version_t) GPS 1 version info */
#define DID_GPS2_VERSION                (eDataIDs)18 /** (gps_version_t) GPS 2 version info */
#define DID_MAG_CAL                     (eDataIDs)19 /** (mag_cal_t) Magnetometer calibration */
#define DID_INTERNAL_DIAGNOSTIC         (eDataIDs)20 /** INTERNAL USE ONLY (internal_diagnostic_t) Internal diagnostic info */
#define DID_GPS1_RTK_REL                (eDataIDs)21 /** (gps_rtk_rel_t) RTK navigation data */
#define DID_GPS1_RTK_MISC               (eDataIDs)22 /** (gps_rtk_misc_t) RTK related data */
#define DID_FEATURE_BITS                (eDataIDs)23 /** INTERNAL USE ONLY (feature_bits_t) */
#define DID_SENSORS_IS1                 (eDataIDs)24 /** INTERNAL USE ONLY (sensors_w_temp_t) Cross-axis aligned w/ scale factor */
#define DID_SENSORS_IS2                 (eDataIDs)25 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated */
#define DID_SENSORS_TC_BIAS             (eDataIDs)26 /** INTERNAL USE ONLY (sensors_t) */
#define DID_IO                          (eDataIDs)27 /** (io_t) I/O */
#define DID_SENSORS_ADC                 (eDataIDs)28 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_SCOMP                       (eDataIDs)29 /** INTERNAL USE ONLY (sensor_compensation_t) */
#define DID_GPS1_VEL                    (eDataIDs)30 /** (gps_vel_t) GPS 1 velocity data */
#define DID_GPS2_VEL                    (eDataIDs)31 /** (gps_vel_t) GPS 2 velocity data */
#define DID_HDW_PARAMS                  (eDataIDs)32 /** INTERNAL USE ONLY (hdw_params_t) */
#define DID_NVR_MANAGE_USERPAGE         (eDataIDs)33 /** INTERNAL USE ONLY (nvr_manage_t) */
#define DID_NVR_USERPAGE_SN             (eDataIDs)34 /** INTERNAL USE ONLY (nvm_group_sn_t) */
#define DID_NVR_USERPAGE_G0             (eDataIDs)35 /** INTERNAL USE ONLY (nvm_group_0_t) */
#define DID_NVR_USERPAGE_G1             (eDataIDs)36 /** INTERNAL USE ONLY (nvm_group_1_t) */
#define DID_DEBUG_STRING                (eDataIDs)37 /** INTERNAL USE ONLY (debug_string_t) */
#define DID_RTOS_INFO                   (eDataIDs)38 /** (rtos_info_t) RTOS information. */
#define DID_DEBUG_ARRAY                 (eDataIDs)39 /** INTERNAL USE ONLY (debug_array_t) */
#define DID_SENSORS_CAL1                (eDataIDs)40 /** INTERNAL USE ONLY (sensors_mpu_w_temp_t) */
#define DID_SENSORS_CAL2                (eDataIDs)41 /** INTERNAL USE ONLY (sensors_mpu_w_temp_t) */
#define DID_CAL_SC                      (eDataIDs)42 /** INTERNAL USE ONLY (sensor_cal_t) */
#define DID_CAL_SC1                     (eDataIDs)43 /** INTERNAL USE ONLY (sensor_cal_mpu_t) */
#define DID_CAL_SC2                     (eDataIDs)44 /** INTERNAL USE ONLY (sensor_cal_mpu_t) */
#define DID_SYS_SENSORS_SIGMA           (eDataIDs)45 /** INTERNAL USE ONLY (sys_sensors_t) */
#define DID_SENSORS_ADC_SIGMA           (eDataIDs)46 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_INS_DEV_1                   (eDataIDs)47 /** INTERNAL USE ONLY (ins_dev_1_t) */
#define DID_INL2_STATES                 (eDataIDs)48 /** (inl2_states_t) */
#define DID_INL2_COVARIANCE_LD          (eDataIDs)49 /** (INL2_COVARIANCE_LD_ARRAY_SIZE) */
#define DID_INL2_STATUS                 (eDataIDs)50 /** (inl2_status_t) */
#define DID_INL2_MISC                   (eDataIDs)51 /** (inl2_misc_t) */
#define DID_MAGNETOMETER_1              (eDataIDs)52 /** (magnetometer_t) Magnetometer sensor 1 output */
#define DID_BAROMETER                   (eDataIDs)53 /** (barometer_t) Barometric pressure sensor data */
#define DID_GPS1_RTK_POS                (eDataIDs)54 /** (gps_pos_t) GPS RTK position data */
#define DID_MAGNETOMETER_2              (eDataIDs)55 /** (magnetometer_t) 2nd magnetometer sensor data */
#define DID_COMMUNICATIONS_LOOPBACK     (eDataIDs)56 /** INTERNAL USE ONLY - Unit test for communications manager  */
#define DID_DUAL_IMU_RAW                (eDataIDs)57 /** (dual_imu_t) Dual inertial measurement unit data directly from IMU.  We recommend use of DID_DUAL_IMU or DID_PREINTEGRATED_IMU. */
#define DID_DUAL_IMU                    (eDataIDs)58 /** (dual_imu_t) Dual inertial measurement unit data down-sampled from 1KHz to navigation update rate as an anti-aliasing filter to reduce noise and preserve accuracy. */
#define DID_INL2_MAG_OBS_INFO           (eDataIDs)59 /** (inl2_mag_obs_info_t) INL2 magnetometer calibration information. */
#define DID_GPS_BASE_RAW                (eDataIDs)60 /** (gps_raw_t) GPS raw data for base station (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. */
#define DID_GPS_RTK_OPT                 (eDataIDs)61 /** (gps_rtk_opt_t) RTK options - requires little endian CPU. */
#define DID_NVR_USERPAGE_INTERNAL       (eDataIDs)62 /** (internal) Internal user page data */
#define DID_MANUFACTURING_INFO          (eDataIDs)63 /** INTERNAL USE ONLY (manufacturing_info_t) Manufacturing info */
#define DID_BIT                         (eDataIDs)64 /** (bit_t) System built-in self-test */
#define DID_INS_3                       (eDataIDs)65 /** (ins_3_t) Inertial navigation data with quaternion NED to body rotation and ECEF position. */
#define DID_INS_4                       (eDataIDs)66 /** (ins_4_t) INS output: quaternion rotation w/ respect to ECEF, ECEF position. */
#define DID_INL2_NED_SIGMA              (eDataIDs)67 /** (inl2_ned_sigma_t) INL2 standard deviation in the NED frame */
#define DID_STROBE_IN_TIME              (eDataIDs)68 /** (strobe_in_time_t) Timestamp for input strobe. */
#define DID_GPS1_RAW                    (eDataIDs)69 /** (gps_raw_t) GPS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. */
#define DID_GPS2_RAW                    (eDataIDs)70 /** (gps_raw_t) GPS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. */
#define DID_VELOCITY_MEASUREMENT        (eDataIDs)71 /** (velocity_sensor_t) External generic velocity sensor to be fused with GPS-INS measurements. */
#define DID_DIAGNOSTIC_MESSAGE          (eDataIDs)72 /** (diag_msg_t) Diagnostic message */
#define DID_SURVEY_IN                   (eDataIDs)73 /** (survey_in_t) Survey in, used to determine position for RTK base station. */
#define DID_EVB2                        (eDataIDs)74 /** (evb2_t) EVB monitor, configuration and log control interface. */
#define DID_PORT_MONITOR                (eDataIDs)75 /** (port_monitor_t) Data rate and status monitoring for each communications port. */

// Adding a new data id?
// 1] Add it above and increment the previous number, include the matching data structure type in the comments
// 2] Add flip doubles and flip strings entries in data_sets.c
// 3] Add data id to ISDataMappings.cpp
// 4] Increment DID_COUNT
// 5] Test!

/** Count of data ids (including null data id 0) - MUST BE MULTPLE OF 4 and larger than last DID number! */
#define DID_COUNT (eDataIDs)76

/** Maximum number of data ids */
#define DID_MAX_COUNT 256

// END DATA IDENTIFIERS --------------------------------------------------------------------------

/** Maximum number of satellite channels */
#define MAX_NUM_SAT_CHANNELS 50

/** Maximum length of device info manufacturer string (must be a multiple of 4) */
#define DEVINFO_MANUFACTURER_STRLEN 24
#define DEVINFO_ADDINFO_STRLEN 24


/** Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */
// #define PROTOCOL_VERSION_CHAR0 1        // Major (in com_manager.h)
// #define PROTOCOL_VERSION_CHAR1 0
#define PROTOCOL_VERSION_CHAR2 (0x000000FF&DID_COUNT)
#define PROTOCOL_VERSION_CHAR3 7         // Minor (in data_sets.h)

/** Latest known number of leap seconds */
#define CURRENT_LEAP_SECONDS 18

/** Rtk rover receiver index */
#define RECEIVER_INDEX_GPS1 1 // DO NOT CHANGE
#define RECEIVER_INDEX_EXTERNAL_BASE 2 // DO NOT CHANGE
#define RECEIVER_INDEX_GPS2 3 // DO NOT CHANGE

#define NUM_MPU_DEVICES     2

/** INS status flags */
enum eInsStatusFlags
{
	/** Attitude estimate is usable but outside spec (COARSE) */
	INS_STATUS_ATT_ALIGN_COARSE                 = (int)0x00000001,
	/** Velocity estimate is usable but outside spec (COARSE) */
	INS_STATUS_VEL_ALIGN_COARSE                 = (int)0x00000002,
	/** Position estimate is usable but outside spec (COARSE) */
	INS_STATUS_POS_ALIGN_COARSE                 = (int)0x00000004,
	/** Estimate is COARSE mask (usable but outside spec) */
	INS_STATUS_ALIGN_COARSE_MASK                = (int)0x00000007,

	INS_STATUS_UNUSED_1                         = (int)0x00000008,

	/** Attitude estimate is within spec (FINE) */
	INS_STATUS_ATT_ALIGN_FINE                   = (int)0x00000010,
	/** Velocity estimate is within spec (FINE) */
	INS_STATUS_VEL_ALIGN_FINE                   = (int)0x00000020,
	/** Position estimate is within spec (FINE) */
	INS_STATUS_POS_ALIGN_FINE                   = (int)0x00000040,
	/** Estimate is FINE mask */
	INS_STATUS_ALIGN_FINE_MASK                  = (int)0x00000070,

	/** Heading aided by GPS */
	INS_STATUS_GPS_AIDING_HEADING               = (int)0x00000080,

	/** Position and velocity aided by GPS */
	INS_STATUS_GPS_AIDING_POS_VEL               = (int)0x00000100,
	/** GPS update event occurred in solution, potentially causing discontinuity in position path */
	INS_STATUS_GPS_UPDATE_IN_SOLUTION           = (int)0x00000200,
	/** Reserved for internal purpose */
	INS_STATUS_RESERVED_1                       = (int)0x00000400,																	
	/** Heading aided by magnetic heading */
	INS_STATUS_MAG_AIDING_HEADING               = (int)0x00000800,

	/** Nav mode (set) = estimating velocity and position. AHRS mode (cleared) = NOT estimating velocity and position */
	INS_STATUS_NAV_MODE							= (int)0x00001000,

	INS_STATUS_UNUSED_2				            = (int)0x00002000,
	INS_STATUS_UNUSED_3				            = (int)0x00004000,
	INS_STATUS_UNUSED_4				            = (int)0x00008000,

	/** INS/AHRS Solution Status */
	INS_STATUS_SOLUTION_MASK					= (int)0x000F0000,
	INS_STATUS_SOLUTION_OFFSET					= 16,
#define INS_STATUS_SOLUTION(insStatus)          ((insStatus&INS_STATUS_SOLUTION_MASK)>>INS_STATUS_SOLUTION_OFFSET)

	INS_STATUS_SOLUTION_OFF                     = 0,	// System is off 
	INS_STATUS_SOLUTION_ALIGNING                = 1,	// System is in alignment mode
	INS_STATUS_SOLUTION_ALIGNMENT_COMPLETE      = 2,	// System is aligned but not enough dynamics have been experienced to be with specifications.
	INS_STATUS_SOLUTION_NAV                     = 3,	// System is in navigation mode and solution is good.
	INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE       = 4,	// System is in navigation mode but the attitude uncertainty has exceeded the threshold.
	INS_STATUS_SOLUTION_AHRS                    = 5,	// System is in AHRS mode and solution is good.
	INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE      = 6,	// System is in AHRS mode but the attitude uncertainty has exceeded the threshold.
	
    /** Trying to perform Compassing with no baseline set in flashCfg. */
    INS_STATUS_RTK_COMPASSING_ANT_POS_UNSET     = (int)0x00100000,
    /** Specified Baseline Length is significantly different from precision solution. */
    INS_STATUS_RTK_COMPASSING_WRONG_BASELINE    = (int)0x00200000,
    INS_STATUS_RTK_COMPASSING_MASK              = (INS_STATUS_RTK_COMPASSING_ANT_POS_UNSET|INS_STATUS_RTK_COMPASSING_WRONG_BASELINE),
    
	/** Magnetometer is being recalibrated.  Device requires rotation to complete the calibration process. */
	INS_STATUS_MAG_RECALIBRATING				= (int)0x00400000,
	/** Magnetometer is experiencing interference or calibration is bad.  Attention may be required to remove interference (move the device) or recalibrate the magnetometer. */
	INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL		= (int)0x00800000,

    /** GPS navigation fix type (see eNavFixStatus) */
    INS_STATUS_NAV_FIX_STATUS_MASK				= (int)0x07000000,
    INS_STATUS_NAV_FIX_STATUS_OFFSET			= 24,
#define INS_STATUS_NAV_FIX_STATUS(insStatus)    ((insStatus&INS_STATUS_NAV_FIX_STATUS_MASK)>>INS_STATUS_NAV_FIX_STATUS_OFFSET)

	/** GPS base NO observations received (i.e. RTK differential corrections) */
    INS_STATUS_RTK_BASE_ERR_NO_OBSERV			= (int)0x08000000,
    /** GPS base NO position received */
    INS_STATUS_RTK_BASE_ERR_NO_POSITION			= (int)0x10000000,
    /** GPS base position is moving */
    INS_STATUS_RTK_BASE_POSITION_MOVING			= (int)0x20000000,
	/** GPS base mask */
	INS_STATUS_RTK_BASE_MASK					= (INS_STATUS_RTK_BASE_ERR_NO_OBSERV|INS_STATUS_RTK_BASE_ERR_NO_POSITION|INS_STATUS_RTK_BASE_POSITION_MOVING),
	
	/** RTOS task ran longer than allotted period */
	INS_STATUS_RTOS_TASK_PERIOD_OVERRUN			= (int)0x40000000,
	/** General fault (eGenFaultCodes) */
	INS_STATUS_GENERAL_FAULT					= (int)0x80000000,
};

/** GPS navigation fix type */
enum eNavFixStatus
{
	NAV_FIX_STATUS_NONE							= (int)0x00000000,
	NAV_FIX_STATUS_STANDARD						= (int)0x00000001,
	NAV_FIX_STATUS_RTK_SINGLE					= (int)0x00000002,
	NAV_FIX_STATUS_RTK_FLOAT					= (int)0x00000003,
	NAV_FIX_STATUS_RTK_FIX						= (int)0x00000004,
	NAV_FIX_STATUS_STANDARD_W_GPS_COMPASSING    = (int)0x00000005,
};

/** Hardware status flags */
enum eHdwStatusFlags
{
	/** Gyro motion detected sigma */
	HDW_STATUS_MOTION_GYR_SIG					= (int)0x00000001,
	/** Accelerometer motion detected sigma */
	HDW_STATUS_MOTION_ACC_SIG					= (int)0x00000002,
	/** Unit is moving and NOT stationary */
	HDW_STATUS_MOTION_SIG_MASK					= (int)0x00000003,
	/** Gyro motion detected deviation */
	HDW_STATUS_MOTION_GYR_DEV					= (int)0x00000004,
	/** Accelerometer motion detected deviation */
	HDW_STATUS_MOTION_ACC_DEV					= (int)0x00000008,
	/** Motion mask */
	HDW_STATUS_MOTION_MASK						= (int)0x0000000F,

	/** GPS satellite signals are being received (antenna and cable are good) */
	HDW_STATUS_GPS_SATELLITE_RX					= (int)0x00000010,
	/** Event occurred on strobe input pin */
	HDW_STATUS_STROBE_IN_EVENT					= (int)0x00000020,
	/** GPS time of week is valid and reported.  Otherwise the timeOfWeek is local system time. */
	HDW_STATUS_GPS_TIME_OF_WEEK_VALID			= (int)0x00000040,

	HDW_STATUS_UNUSED_1				            = (int)0x00000080,

	/** Sensor saturation on gyro */
	HDW_STATUS_SATURATION_GYR					= (int)0x00000100,
	/** Sensor saturation on accelerometer */
	HDW_STATUS_SATURATION_ACC					= (int)0x00000200,
	/** Sensor saturation on magnetometer */
	HDW_STATUS_SATURATION_MAG					= (int)0x00000400,
	/** Sensor saturation on barometric pressure */
	HDW_STATUS_SATURATION_BARO					= (int)0x00000800,

	/** Sensor saturation mask */
	HDW_STATUS_SATURATION_MASK					= (int)0x00000F00,
	/** Sensor saturation offset */
	HDW_STATUS_SATURATION_OFFSET				= 8,

	/** Sensor saturation happened within in past 10 seconds */
	HDW_STATUS_SATURATION_HISTORY				= (int)0x00001000,

	HDW_STATUS_UNUSED_2				            = (int)0x00002000,
	HDW_STATUS_UNUSED_3				            = (int)0x00004000,
	HDW_STATUS_UNUSED_4				            = (int)0x00008000,

	/** Communications Tx buffer limited */
	HDW_STATUS_ERR_COM_TX_LIMITED				= (int)0x00010000,
	/** Communications Rx buffer overrun */
	HDW_STATUS_ERR_COM_RX_OVERRUN				= (int)0x00020000,
	/** GPS Tx buffer limited */
	HDW_STATUS_ERR_GPS_TX_LIMITED				= (int)0x00040000,
	/** GPS Rx buffer overrun */
	HDW_STATUS_ERR_GPS_RX_OVERRUN				= (int)0x00080000,

	/** Communications parse error count */
	HDW_STATUS_COM_PARSE_ERR_COUNT_MASK			= (int)0x00F00000,
	HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET		= 20,
#define HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus) ((hdwStatus&HDW_STATUS_COM_PARSE_ERR_COUNT_MASK)>>HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET)

	/** Built-in self-test failure */
	HDW_STATUS_BIT_FAULT						= (int)0x01000000,
	/** Temperature outside spec'd operating range */
	HDW_STATUS_ERR_TEMPERATURE					= (int)0x02000000,
	/** Vibrations effecting accuracy */
// 	HDW_STATUS_ERR_VIBRATION					= (int)0x04000000,

	HDW_STATUS_UNUSED_5				            = (int)0x08000000,

	/** Fault reset cause */
	HDW_STATUS_FAULT_RESET_MASK					= (int)0x70000000,	
	/** Reset from Backup mode (low-power state w/ CPU off) */
	HDW_STATUS_FAULT_RESET_BACKUP_MODE			= (int)0x10000000,
	/** Reset from Watchdog */
	HDW_STATUS_FAULT_RESET_WATCHDOG				= (int)0x20000000,
	/** Reset from Software */
	HDW_STATUS_FAULT_RESET_SOFT					= (int)0x30000000,
	/** Reset from Hardware (NRST pin low) */
	HDW_STATUS_FAULT_RESET_HDW					= (int)0x40000000,

	/** CPU error */
	HDW_STATUS_FAULT_CPU_ERR					= (int)0x80000000,
};

/** GPS Status */
enum eGpsStatus
{
	GPS_STATUS_NUM_SATS_USED_MASK				= (int)0x000000FF,

	/** Fix */
	GPS_STATUS_FIX_NONE							= (int)0x00000000,
	GPS_STATUS_FIX_DEAD_RECKONING_ONLY			= (int)0x00000100,
	GPS_STATUS_FIX_2D							= (int)0x00000200,
	GPS_STATUS_FIX_3D							= (int)0x00000300,
	GPS_STATUS_FIX_GPS_PLUS_DEAD_RECK			= (int)0x00000400,
	GPS_STATUS_FIX_TIME_ONLY					= (int)0x00000500,
	GPS_STATUS_FIX_UNUSED1                      = (int)0x00000600,
	GPS_STATUS_FIX_UNUSED2                      = (int)0x00000700,
	GPS_STATUS_FIX_DGPS							= (int)0x00000800,
	GPS_STATUS_FIX_SBAS							= (int)0x00000900,
	GPS_STATUS_FIX_RTK_SINGLE					= (int)0x00000A00,
	GPS_STATUS_FIX_RTK_FLOAT					= (int)0x00000B00,
	GPS_STATUS_FIX_RTK_FIX						= (int)0x00000C00,	
	GPS_STATUS_FIX_MASK							= (int)0x00001F00,
	GPS_STATUS_FIX_BIT_OFFSET					= (int)8,

	/** Flags  */
	GPS_STATUS_FLAGS_FIX_OK						= (int)0x00010000,		// within limits (e.g. DOP & accuracy)
	GPS_STATUS_FLAGS_DGPS_USED					= (int)0x00020000,		// Differential GPS (DGPS) used.
	GPS_STATUS_FLAGS_WEEK_VALID					= (int)0x00040000,
	GPS_STATUS_FLAGS_TOW_VALID					= (int)0x00080000,
	GPS_STATUS_FLAGS_RTK_ENABLED				= (int)0x00100000,
	GPS_STATUS_FLAGS_STATIC_MODE				= (int)0x00200000,		// Static mode
	GPS_STATUS_FLAGS_GPS_COMPASSING_ENABLED		= (int)0x00400000,
    GPS_STATUS_FLAGS_BASE_NO_OBSERV_EPHEM       = (int)0x00800000,		// GPS base observations and ephemeris received (i.e. RTK differential corrections)
    GPS_STATUS_FLAGS_BASE_NO_POSITION           = (int)0x01000000,		// GPS base position received
    GPS_STATUS_FLAGS_BASE_POSITION_MOVING       = (int)0x02000000,		// GPS Base position moving
    GPS_STATUS_FLAGS_BASE_MASK                  = (GPS_STATUS_FLAGS_BASE_NO_OBSERV_EPHEM|
                                                   GPS_STATUS_FLAGS_BASE_NO_POSITION|
                                                   GPS_STATUS_FLAGS_BASE_POSITION_MOVING),
	GPS_STATUS_FLAGS_HIGH_ACCURACY_POSITION	    = (int)0x04000000,      // 1= using RTK position, 0= using ublox position
	GPS_STATUS_FLAGS_GPS_COMPASSING_VALID	    = (int)0x08000000,
    GPS_STATUS_FLAGS_GPS_COMPASS_BAD_BASELINE   = (int)0x00002000,
    GPS_STATUS_FLAGS_GPS_COMPASS_NO_BASELINE    = (int)0x00004000,
    GPS_STATUS_FLAGS_GPS_COMPASS_MASK           = (GPS_STATUS_FLAGS_GPS_COMPASSING_VALID|
                                                   GPS_STATUS_FLAGS_GPS_COMPASS_BAD_BASELINE|
                                                   GPS_STATUS_FLAGS_GPS_COMPASS_BAD_BASELINE),
	GPS_STATUS_FLAGS_UNUSED_3                   = (int)0x00008000,
	GPS_STATUS_FLAGS_MASK						= (int)0x0FFFE000,    
	GPS_STATUS_FLAGS_BIT_OFFSET					= (int)16,

	/** Init */
	GPS_STATUS_INIT_STATUS_PROGRAM_OSC			= (int)0x10000000,
	GPS_STATUS_INIT_STATUS_REINIT				= (int)0x20000000,
	GPS_STATUS_INIT_STATUS_READING				= (int)0x40000000,
	GPS_STATUS_INIT_STATUS_MASK					= (int)0xF0000000,
	GPS_STATUS_INIT_STATUS_BIT_OFFSET			= (int)28,
};

PUSH_PACK_1

/** (DID_DEV_INFO) Device information */
typedef struct PACKED
{
	/** Reserved bits */
	uint32_t        reserved;

	/** Serial number */
	uint32_t        serialNumber;

	/** Hardware version */
	uint8_t         hardwareVer[4];

	/** Firmware (software) version */
	uint8_t         firmwareVer[4];

	/** Build number */
	uint32_t        buildNumber;

	/** Communications protocol version */
	uint8_t         protocolVer[4];

	/** Repository revision number */
	uint32_t        repoRevision;

	/** Manufacturer name */
	char            manufacturer[DEVINFO_MANUFACTURER_STRLEN];

    /** Build date, little endian order: [0] = status ('r'=release, 'd'=debug), [1] = year-2000, [2] = month, [3] = day.  Reversed byte order for big endian systems */
	uint8_t         buildDate[4];

    /** Build date, little endian order: [0] = hour, [1] = minute, [2] = second, [3] = millisecond.  Reversed byte order for big endian systems */
	uint8_t         buildTime[4];

	/** Additional info */
	char            addInfo[DEVINFO_ADDINFO_STRLEN];
} dev_info_t;

/** (DID_MANUFACTURING_INFO) Manufacturing info */
typedef struct PACKED
{
	/** Serial number */
	uint32_t		serialNumber;

	/** Lot number */
	uint32_t		lotNumber;

	/** Manufacturing date (YYYYMMDDHHMMSS) */
    char			date[16];

	/** Key */
	uint32_t		key;
} manufacturing_info_t;


/** (DID_INS_1) INS output: euler rotation w/ respect to NED, NED position from reference LLA */
typedef struct PACKED
{
	/** Weeks since January 6th, 1980 */
	uint32_t				week;
	
	/** Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
	uint32_t				insStatus;

	/** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
	uint32_t				hdwStatus;

	/** Euler angles: roll, pitch, yaw in radians with respect to NED */
	float					theta[3];

	/** Velocity U, V, W in meters per second.  Convert to NED velocity using "vectorBodyToReference( uvw, theta, vel_ned )". */
	float					uvw[3];

	/** WGS84 latitude, longitude, height above ellipsoid (degrees,degrees,meters) */
	double					lla[3];

	/** North, east and down offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
	float					ned[3];
} ins_1_t;


/** (DID_INS_2) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
typedef struct PACKED
{
	/** Weeks since January 6th, 1980 */
	uint32_t				week;
	
	/** Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
	uint32_t				insStatus;

	/** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
	uint32_t				hdwStatus;

	/** Quaternion body rotation with respect to NED: W, X, Y, Z */
	float					qn2b[4];

	/** Velocity U, V, W in meters per second.  Convert to NED velocity using "quatRot(vel_ned, qn2b, uvw)". */
	float					uvw[3];

	/** WGS84 latitude, longitude, height above ellipsoid in meters (not MSL) */
	double					lla[3];
} ins_2_t;


/** (DID_INS_3) INS output: quaternion rotation w/ respect to NED, msl altitude */
typedef struct PACKED
{
	/** Weeks since January 6th, 1980 */
	uint32_t				week;
	
	/** Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
	uint32_t				insStatus;

	/** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
	uint32_t				hdwStatus;

	/** Quaternion body rotation with respect to NED: W, X, Y, Z */
	float					qn2b[4];

	/** Velocity U, V, W in meters per second.  Convert to NED velocity using "quatRot(vel_ned, qn2b, uvw)". */
	float					uvw[3];

	/** WGS84 latitude, longitude, height above mean sea level (MSL) in meters */
	double					lla[3];

	/** height above mean sea level (MSL) in meters */
	float					msl;
} ins_3_t;


/** (DID_INS_4) INS output: quaternion rotation w/ respect to ECEF, ECEF position */
typedef struct PACKED
{
	/** Weeks since January 6th, 1980 */
	uint32_t				week;
	
	/** Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/** INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus */
	uint32_t				insStatus;

	/** Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus */
	uint32_t				hdwStatus;

	/** Quaternion body rotation with respect to ECEF: W, X, Y, Z */
	float					qe2b[4];

	/** Velocity in ECEF (earth-centered earth-fixed) frame in meters per second */
	float					ve[3];

	/** Position in ECEF (earth-centered earth-fixed) frame in meters */
	double					ecef[3];
} ins_4_t;


/** Inertial Measurement Unit (IMU) data */
typedef struct PACKED
{
	/** Gyroscope P, Q, R in radians / second */
	float                   pqr[3];

	/** Acceleration X, Y, Z in meters / second squared */
	float                   acc[3];
} imus_t;


/** Inertial Measurement Unit (IMU) data */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/** Inertial Measurement Unit (IMU) */
	imus_t					I;
} imu_t;


/** (DID_DUAL_IMU) Dual Inertial Measurement Units (IMUs) data */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/** Inertial Measurement Units (IMUs) */
	imus_t                  I[2];
} dual_imu_t;


/** (DID_MAGNETOMETER_1, DID_MAGNETOMETER_2) Magnetometer sensor data */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;
	
	/** Magnetometers in Gauss */
	float                   mag[3];
} magnetometer_t;


/** (DID_BAROMETER) Barometric pressure sensor data */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;
	
	/** Barometric pressure in kilopascals */
	float                   bar;

	/** MSL altitude from barometric pressure sensor in meters */
	float                   mslBar;

	/** Temperature of barometric pressure sensor in Celsius */
	float                   barTemp;

	/** Relative humidity as a percent (%rH). Range is 0% - 100% */
	float                   humidity;
} barometer_t;


/** (DID_PREINTEGRATED_IMU) Coning and sculling integral in body/IMU frame. */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/** IMU 1 delta theta (gyroscope {p,q,r} integral) in radians in sensor frame */
	float                   theta1[3];

	/** IMU 2 delta theta (gyroscope {p,q,r} integral) in radians in sensor frame */
	float                   theta2[3];

	/** IMU 1 delta velocity (accelerometer {x,y,z} integral) in m/s in sensor frame */
	float                   vel1[3];

	/** IMU 2 delta velocity (accelerometer {x,y,z} integral) in m/s in sensor frame */
	float                   vel2[3];

	/** Integral period in seconds for delta theta and delta velocity.  This is configured using DID_FLASH_CONFIG.startupNavDtMs. */
	float					dt;
} preintegrated_imu_t;


/** (DID_GPS1_POS, DID_GPS1_UBX_POS, DID_GPS2_POS) GPS position data */
typedef struct PACKED
{
	/** Number of weeks since January 6th, 1980 */
	uint32_t                week;

	/** Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/** (see eGpsStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags */
	uint32_t                status;

	/** Position in ECEF {x,y,z} (m) */
	double					ecef[3];
    
	/** Position - WGS84 latitude, longitude, height above ellipsoid (not MSL) (degrees, m) */
	double					lla[3];

	/** Height above mean sea level (MSL) in meters */
	float					hMSL;

	/** Horizontal accuracy in meters */
	float					hAcc;

	/** Vertical accuracy in meters */
	float					vAcc;

	/** Position dilution of precision in meters */
	float                   pDop;

	/** Average of all satellite carrier to noise ratios (signal strengths) that non-zero in dBHz */
	float                   cnoMean;

	/** Time sync offset between local time since boot up to time of week in seconds */
	double                  towOffset;
} gps_pos_t;


/** (DID_GPS1_VEL, DID_GPS2_VEL) GPS velocity data */
typedef struct PACKED
{
    /** Time of week (since Sunday morning) in milliseconds, GMT */
    uint32_t                timeOfWeekMs;

	/** Position in ECEF {vx,vy,vz} (m/s) */
	float					velEcef[3];	

	/** Speed accuracy in meters / second */
	float					sAcc;
} gps_vel_t;


/** GPS Satellite information */
typedef struct PACKED
{
	/** GNSS identifier: 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS */
	uint8_t					gnssId;			

	/** Satellite identifier */
	uint8_t					svId;			

	/** (dBHz) Carrier to noise ratio (signal strength) */
	uint8_t					cno;			

	/** (deg) Elevation (range: +/-90) */
	int8_t					elev;			

	/** (deg) Azimuth (range: +/-180) */
	int16_t					azim;			

	/** (m) Pseudo range residual */
	int16_t					prRes;			

	/** (see eSatSvFlags) */
	uint32_t				flags;			
} gps_sat_sv_t;


/** GPS Status */
enum eSatSvFlags
{
	SAT_SV_FLAGS_QUALITYIND_MASK	= 0x00000007,
	SAT_SV_FLAGS_SV_USED			= 0x00000008,
	SAT_SV_FLAGS_HEALTH_MASK		= 0x00000030,
	NAV_SAT_FLAGS_HEALTH_OFFSET		= 4,
	SAT_SV_FLAGS_DIFFCORR			= 0x00000040,
	SAT_SV_FLAGS_SMOOTHED			= 0x00000080,
	SAT_SV_FLAGS_ORBITSOURCE_MASK	= 0x00000700,
	SAT_SV_FLAGS_ORBITSOURCE_OFFSET	= 8,
	SAT_SV_FLAGS_EPHAVAIL			= 0x00000800,
	SAT_SV_FLAGS_ALMAVAIL			= 0x00001000,
	SAT_SV_FLAGS_ANOAVAIL			= 0x00002000,
	SAT_SV_FLAGS_AOPAVAIL			= 0x00004000,
};

/** (DID_GPS1_SAT) GPS satellite signal strength */
typedef struct PACKED
{
    /** Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;				
    /** Number of satellites in the sky */
	uint32_t				numSats;					
    /** Satellite information list */
	gps_sat_sv_t			sat[MAX_NUM_SAT_CHANNELS];	
} gps_sat_t;


/** (DID_GPS1_VERSION) GPS version strings */
typedef struct PACKED
{
    /** Software version */
	uint8_t                 swVersion[30];
    /** Hardware version */
	uint8_t                 hwVersion[10];		
    /** Extension */
	uint8_t                 extension[30];		
    /** ensure 32 bit aligned in memory */
	uint8_t					reserved[2];		
} gps_version_t;

// (DID_INL2_STATES) INL2 - INS Extended Kalman Filter (EKF) states
typedef struct PACKED
{
    /** Time of week (since Sunday morning) in seconds, GMT */
	double                  timeOfWeek;					

    /** Quaternion body rotation with respect to ECEF */
	float					qe2b[4];                    

    /** (m/s) Velocity in ECEF frame */
	float					ve[3];						

    /** (m)     Position in ECEF frame */
    double					ecef[3];				

	/** (rad/s) Gyro bias */
    float					biasPqr[3];	           
	
    /** (m/s^2) Accelerometer bias */
    float					biasAcc[3];	            
	
    /** (m)     Barometer bias */
    float					biasBaro;               
	
    /** (rad)   Magnetic declination */
    float					magDec;                 
	
    /** (rad)   Magnetic inclination */
    float					magInc;                 
} inl2_states_t;

/** Generic 1 axis sensor */
typedef struct PACKED
{
	/** Time in seconds */
	double                  time;

	/** Three axis sensor */
	float                   val;
} gen_1axis_sensor_t;

/** Generic 3 axis sensor */
typedef struct PACKED
{
	/** Time in seconds */
	double                  time;

	/** Three axis sensor */
	float                   val[3];
} gen_3axis_sensor_t;

/** Generic dual 3 axis sensor */
typedef struct PACKED
{
	/** Time in seconds */
	double                  time;

	/** First three axis sensor */
	float                   val1[3];

	/** Second three axis sensor */
	float                   val2[3];
} gen_dual_3axis_sensor_t;

/** Generic 3 axis sensor */
typedef struct PACKED
{
	/** Time in seconds */
	double                  time;

	/** Three axis sensor */
	double                  val[3];
} gen_3axis_sensord_t;

/** (DID_SYS_SENSORS) Output from system sensors */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double					time;

	/** Temperature in Celsius */
	float                   temp;

	/** Gyros in radians / second */
	float                   pqr[3];

	/** Accelerometers in meters / second squared */
	float                   acc[3];

	/** Magnetometers in Gauss */
	float                   mag[3];

	/** Barometric pressure in kilopascals */
	float                   bar;

	/** Temperature of barometric pressure sensor in Celsius */
	float                   barTemp;

	/** MSL altitude from barometric pressure sensor in meters */
	float                   mslBar;
	
	/** Relative humidity as a percent (%rH). Range is 0% - 100% */
	float                   humidity;

	/** EVB system input voltage in volts. uINS pin 5 (G2/AN2).  Use 10K/1K resistor divider between Vin and GND.  */
	float                   vin;

	/** ADC analog input in volts. uINS pin 4, (G1/AN1). */
	float                   ana1;

	/** ADC analog input in volts. uINS pin 19 (G3/AN3). */
	float                   ana3;

	/** ADC analog input in volts. uINS pin 20 (G4/AN4). */
	float                   ana4;
} sys_sensors_t;

/** INS output */
typedef struct PACKED
{
	/** Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/** Latitude, longitude and height above ellipsoid (rad, rad, m) */
	double                  lla[3];

	/** Velocities in body frames of X, Y and Z (m/s) */
	float                   uvw[3];

	/** Quaternion body rotation with respect to NED: W, X, Y, Z */
	float					qn2b[4];
} ins_output_t;

/** (DID_SYS_PARAMS) System parameters */
typedef struct PACKED
{
	/** Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/** System status 1 flags (eInsStatusFlags) */
	uint32_t                insStatus;

	/** System status 2 flags (eHdwStatusFlags) */
	uint32_t                hdwStatus;

	/** IMU temperature */
	float					imuTemp;

	/** Baro temperature */
	float					baroTemp;

	/** MCU temperature (not available yet) */
	float					mcuTemp;

	/** Reserved */
	float					reserved1;

	/** IMU sample period in milliseconds. Zero disables sampling. */
	uint32_t				imuPeriodMs;

	/** Nav filter update period in milliseconds. Zero disables nav filter. */
	uint32_t				navPeriodMs;
	
	/** Reserved */
	float					reserved2[4];

	/** General fault code descriptor (eGenFaultCodes) */
	uint32_t                genFaultCode;
} sys_params_t;


/** (DID_CONFIG) Configuration functions */
typedef struct PACKED
{
	/** System commands: 1=save current persistent messages, 99=software reset (eConfigSystem).  "invSystem" (following variable) must be set to bitwise inverse of this value.  */
	uint32_t                system;

    /** Bitwise inverse of system variable for error checking.  */
    uint32_t                invSystem;

} config_t;

enum eConfigSystem {
    CFG_SYS_CMD_SAVE_PERSISTENT_MESSAGES            = 1,
    CFG_SYS_CMD_ENABLE_BOOTLOADER_AND_RESET         = 2,
    CFG_SYS_CMD_ENABLE_SENSOR_STATS                 = 3,
    CFG_SYS_CMD_ENABLE_RTOS_STATS                   = 4,
    CFG_SYS_CMD_ENABLE_GPS_LOW_LEVEL_CONFIG         = 10,
    CFG_SYS_CMD_SAVE_GPS_ASSIST_TO_FLASH_RESET      = 98,
    CFG_SYS_CMD_SOFTWARE_RESET                      = 99,
};



/** (DID_ASCII_BCAST_PERIOD) ASCII broadcast periods. This data structure (when it is included in the sCommData struct) is zeroed out on stop_all_broadcasts */
typedef struct PACKED
{
	/** Options: Port selection[0x0=current, 0xFF=all, 0x1=ser0, 0x2=ser1, 0x4=USB] (see RMC_OPTIONS_...) */
	uint32_t				options;

	/** Broadcast period (ms) - ASCII dual IMU data. 0 to disable. */
	uint32_t				pimu;

	/** Broadcast period (ms) - ASCII preintegrated dual IMU: delta theta (rad) and delta velocity (m/s). 0 to disable. */
	uint32_t				ppimu;
	
	/** Broadcast period (ms) - ASCII INS output: euler rotation w/ respect to NED, NED position from reference LLA. 0 to disable. */
	uint32_t				pins1;

	/** Broadcast period (ms) - ASCII INS output: quaternion rotation w/ respect to NED, ellipsoid altitude. 0 to disable. */
	uint32_t				pins2;
	
	/** Broadcast period (ms) - ASCII GPS position data. 0 to disable. */
	uint32_t				pgpsp;

	/** Broadcast period (ms) - Reserved.  Leave zero. */
	uint32_t				reserved;

	/** Broadcast period (ms) - ASCII NMEA GPGGA GPS 3D location, fix, and accuracy. 0 to disable. */
	uint32_t				gpgga;

	/** Broadcast period (ms) - ASCII NMEA GPGLL GPS 2D location and time. 0 to disable. */
	uint32_t				gpgll;

	/** Broadcast period (ms) - ASCII NMEA GSA GPS DOP and active satellites. 0 to disable. */
	uint32_t				gpgsa;

	/** Broadcast period (ms) - ASCII NMEA recommended minimum specific GPS/Transit data. 0 to disable. */
	uint32_t				gprmc;

} ascii_msgs_t;

/* (DID_SENSORS_CAL1, DID_SENSORS_CAL2) */
typedef struct PACKED
{                                       // Units only apply for calibrated data
	f_t						pqr[3];         // (rad/s)	Angular rate
	f_t						acc[3];         // (m/s^2)	Linear acceleration
	f_t						mag[3];         // (uT)		Magnetometers
	f_t						temp;			// (°C)		Temperature of MPU
} sensors_mpu_w_temp_t;

#define NUM_ANA_CHANNELS	4
typedef struct PACKED
{                                       // LSB units for all except temperature, which is Celsius.
	double					time;
	sensors_mpu_w_temp_t	mpu[NUM_MPU_DEVICES];
	f_t						bar;            // Barometric pressure
	f_t						barTemp;		// Temperature of barometric pressure sensor
	f_t                     humidity;	// Relative humidity as a percent (%rH).  Range is 0% - 100%
	f_t						ana[NUM_ANA_CHANNELS]; // ADC analog input
} sys_sensors_adc_t;

#define RMC_NUM_PORTS	2	// COM0_PORT_NUM and COM1_PORT_NUM.  No USB yet.

/** Realtime Message Controller (used in rmc_t). 
	The data sets available through RMC are broadcast at the availability of the data.  A goal of RMC is 
	to provide updates from each onboard sensor as fast as possible with minimal latency.  The RMC is 
	provided so that broadcast of sensor data is done as soon as it becomes available.   The exception to
	this rule is the INS output data, which has a configurable output data rate according to DID_RMC.insPeriodMs.
*/
#define RMC_OPTIONS_PORT_MASK           0x000000FF
#define RMC_OPTIONS_PORT_ALL            RMC_OPTIONS_PORT_MASK
#define RMC_OPTIONS_PORT_CURRENT        0x00000000
#define RMC_OPTIONS_PORT_SER0           0x00000001
#define RMC_OPTIONS_PORT_SER1           0x00000002	// also SPI
#define RMC_OPTIONS_PORT_USB            0x00000004
#define RMC_OPTIONS_PRESERVE_CTRL       0x00000100	// Prevent any messages from getting turned off by bitwise OR'ing new message bits with current message bits.
#define RMC_OPTIONS_PERSISTENT          0x00000200	// Save current port RMC to flash memory for use following reboot, eliminating need to re-enable RMC to start data streaming.  

																// RMC message data rates:
#define RMC_BITS_INS1                   0x0000000000000001      // rmc.insPeriodMs (4ms default)
#define RMC_BITS_INS2                   0x0000000000000002      // "
#define RMC_BITS_INS3                   0x0000000000000004      // "
#define RMC_BITS_INS4                   0x0000000000000008      // "
#define RMC_BITS_DUAL_IMU               0x0000000000000010      // DID_FLASH_CONFIG.startupNavDtMs (4ms default)
#define RMC_BITS_PREINTEGRATED_IMU      0x0000000000000020      // "
#define RMC_BITS_BAROMETER              0x0000000000000040      // ~8ms
#define RMC_BITS_MAGNETOMETER1          0x0000000000000080      // ~10ms
#define RMC_BITS_MAGNETOMETER2          0x0000000000000100      // "

#define RMC_BITS_GPS1_POS               0x0000000000000400      // DID_FLASH_CONFIG.startupGpsDtMs (200ms default)
#define RMC_BITS_GPS2_POS               0x0000000000000800      // "
#define RMC_BITS_GPS1_RAW               0x0000000000001000      // "
#define RMC_BITS_GPS2_RAW               0x0000000000002000      // "
#define RMC_BITS_GPS1_SAT               0x0000000000004000      // 1s
#define RMC_BITS_GPS2_SAT               0x0000000000008000      // "
#define RMC_BITS_GPS_BASE_RAW           0x0000000000010000      // 
#define RMC_BITS_STROBE_IN_TIME         0x0000000000020000      // On strobe input event
#define RMC_BITS_DIAGNOSTIC_MESSAGE     0x0000000000040000
#define RMC_BITS_DUAL_IMU_RAW           0x0000000000080000      // DID_FLASH_CONFIG.startupImuDtMs (1ms default)
#define RMC_BITS_GPS1_VEL               0x0000000000100000      // DID_FLASH_CONFIG.startupGpsDtMs (200ms default)
#define RMC_BITS_GPS2_VEL               0x0000000000200000      // "
#define RMC_BITS_GPS1_UBX_POS           0x0000000000400000      // "
#define RMC_BITS_GPS1_RTK_POS           0x0000000000800000      // "
#define RMC_BITS_GPS1_RTK_REL           0x0000000001000000      // "
#define RMC_BITS_GPS1_RTK_MISC          0x0000000004000000      // "
#define RMC_BITS_MASK                   0x0FFFFFFFFFFFFFFF

#define RMC_BITS_INTERNAL_PPD           0x4000000000000000      // 
#define RMC_BITS_PRESET                 0x8000000000000000      // Indicate BITS is a preset

// Preset: Post Processing Data
#define RMC_PRESET_PPD_BITS_NO_IMU     (RMC_BITS_PRESET | \
                                        RMC_BITS_INS2 | \
                                        RMC_BITS_BAROMETER | \
                                        RMC_BITS_MAGNETOMETER1 | \
                                        RMC_BITS_MAGNETOMETER2 | \
                                        RMC_BITS_GPS1_POS | \
                                        RMC_BITS_GPS2_POS | \
                                        RMC_BITS_GPS1_VEL | \
                                        RMC_BITS_GPS2_VEL | \
                                        RMC_BITS_GPS1_RAW | \
                                        RMC_BITS_GPS2_RAW | \
                                        RMC_BITS_GPS_BASE_RAW | \
                                        RMC_BITS_GPS1_RTK_REL | \
                                        RMC_BITS_INTERNAL_PPD | \
                                        RMC_BITS_DIAGNOSTIC_MESSAGE )
#define RMC_PRESET_PPD_BITS            (RMC_PRESET_PPD_BITS_NO_IMU | RMC_BITS_PREINTEGRATED_IMU )
#define RMC_PRESET_PPD_BITS_RAW_IMU    (RMC_PRESET_PPD_BITS_NO_IMU | RMC_BITS_DUAL_IMU_RAW )
#define RMC_PRESET_PPD_NAV_PERIOD_MULT  25
#define RMC_PRESET_INS_BITS            (RMC_BITS_INS2 | \
                                        RMC_BITS_GPS1_POS | \
                                        RMC_BITS_PRESET )
#define RMC_PRESET_INS_NAV_PERIOD_MULT  1   // fastest rate (nav filter update rate)

/** (DID_RMC) Realtime message controller (RMC). */
typedef struct PACKED
{
	/** Data stream enable bits for the specified ports.  (see RMC_BITS_...) */
	uint64_t                bits;

	/** Options to select alternate ports to output data, etc.  (see RMC_OPTIONS_...) */
	uint32_t				options;
	
	/** IMU and Integrated IMU data transmit period is set using DID_SYS_PARAMS.navPeriodMs */
} rmc_t;


/** (DID_IO) Input/Output */
typedef struct PACKED
{
	/** Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/** General purpose I/O status */
	uint32_t				gpioStatus;
} io_t;

enum eMagRecalMode
{
	MAG_RECAL_MODE_MULTI_AXIS       = (int)0,		// Recalibrate magnetometers using multiple axis
	MAG_RECAL_MODE_SINGLE_AXIS      = (int)1,		// Recalibrate magnetometers using only one axis
	MAG_RECAL_MODE_COMPLETE         = (int)100,		// Recalibration is finished
	MAG_RECAL_MODE_ABORT            = (int)101,		// Recalibration is finished
};

/** (DID_MAG_CAL) Magnetometer Calibration */
typedef struct PACKED
{
	/** Set mode and start recalibration.  0 = multi-axis, 1 = single-axis, 100 = done. */
	uint32_t                enMagRecal;
	
	/** Mag recalibration progress indicator: 0-100 % */
	float					progress;

	/** Magnetic declination estimate */
	float					declination;
} mag_cal_t;


/** Built-in test state */
enum eBitState
{
	BIT_STATE_OFF					= (int)0,
	BIT_STATE_DONE					= (int)1,
	BIT_STATE_CMD_FULL_STATIONARY	= (int)2,	// (FULL) More comprehensive test.  Requires system be completely stationary without vibrations. 
	BIT_STATE_CMD_BASIC_MOVING		= (int)3,	// (BASIC) Ignores sensor output.  Can be run while moving.  This mode is automatically run after bootup.
	BIT_STATE_RESERVED_1			= (int)4,
	BIT_STATE_RESERVED_2			= (int)5,
	BIT_STATE_RUNNING				= (int)6,
	BIT_STATE_FINISH				= (int)7,
	BIT_STATE_COMMAND_OFF			= (int)8
};

/** Hardware built-in test flags */
enum eHdwBitStatusFlags
{
	HDW_BIT_PASSED_MASK				= (int)0x0000000F,
	HDW_BIT_PASSED_ALL				= (int)0x00000001,
	HDW_BIT_PASSED_NO_GPS			= (int)0x00000002,	// Passed w/o valid GPS signal
	HDW_BIT_MODE_MASK				= (int)0x000000F0,	// BIT mode run
	HDW_BIT_MODE_OFFSET				= (int)4,
#define HDW_BIT_MODE(hdwBitStatus) ((hdwBitStatus&HDW_BIT_MODE_MASK)>>HDW_BIT_MODE_OFFSET)
	HDW_BIT_FAILED_MASK				= (int)0xFFFFFF00,
	HDW_BIT_FAILED_AHRS_MASK		= (int)0xFFFF0F00,
	HDW_BIT_FAULT_NOISE_PQR			= (int)0x00000100,
	HDW_BIT_FAULT_NOISE_ACC			= (int)0x00000200,
	HDW_BIT_FAULT_GPS_NO_COM		= (int)0x00001000,	// No GPS serial communications
	HDW_BIT_FAULT_GPS_POOR_CNO		= (int)0x00002000,	// Poor GPS signal strength.  Check antenna
	HDW_BIT_FAULT_GPS_POOR_ACCURACY	= (int)0x00002000,	// Low number of satellites, or bad accuracy 
	HDW_BIT_FAULT_GPS_NOISE			= (int)0x00004000,	// (Not implemented)
};

/** Calibration built-in test flags */
enum eCalBitStatusFlags
{
	CAL_BIT_PASSED_MASK				= (int)0x0000000F,
	CAL_BIT_PASSED_ALL				= (int)0x00000001,
	CAL_BIT_MODE_MASK				= (int)0x000000F0,	// BIT mode run
	CAL_BIT_MODE_OFFSET				= (int)4,
#define CAL_BIT_MODE(calBitStatus) ((calBitStatus&CAL_BIT_MODE_MASK)>>CAL_BIT_MODE_OFFSET)
	CAL_BIT_FAILED_MASK				= (int)0xFFFFFF00,
	CAL_BIT_FAULT_TCAL_EMPTY		= (int)0x00000100,	// Temperature calibration not present
	CAL_BIT_FAULT_TCAL_TSPAN		= (int)0x00000200,	// Temperature calibration temperature range is inadequate
	CAL_BIT_FAULT_TCAL_INCONSISTENT	= (int)0x00000400,	// Temperature calibration number of points or slopes are not consistent
	CAL_BIT_FAULT_TCAL_CORRUPT		= (int)0x00000800,	// Temperature calibration memory corruption
	CAL_BIT_FAULT_TCAL_PQR_BIAS		= (int)0x00001000,	// Temperature calibration gyro bias
	CAL_BIT_FAULT_TCAL_PQR_SLOPE	= (int)0x00002000,	// Temperature calibration gyro slope
	CAL_BIT_FAULT_TCAL_PQR_LIN		= (int)0x00004000,	// Temperature calibration gyro linearity
	CAL_BIT_FAULT_TCAL_ACC_BIAS		= (int)0x00008000,	// Temperature calibration accelerometer bias
	CAL_BIT_FAULT_TCAL_ACC_SLOPE	= (int)0x00010000,	// Temperature calibration accelerometer slope
	CAL_BIT_FAULT_TCAL_ACC_LIN		= (int)0x00020000,	// Temperature calibration accelerometer linearity
	CAL_BIT_FAULT_ORTO_EMPTY		= (int)0x00100000,	// Cross-axis alignment is not calibrated
	CAL_BIT_FAULT_ORTO_INVALID		= (int)0x00200000,	// Cross-axis alignment is poorly formed
	CAL_BIT_FAULT_MOTION_PQR		= (int)0x00400000,	// Motion on gyros
	CAL_BIT_FAULT_MOTION_ACC		= (int)0x00800000,	// Motion on accelerometers
};


/** (DID_BIT) Built-in self-test parameters */
typedef struct PACKED
{
	/** Built-in self-test state (see eBitState) */
	uint32_t                state;

	/** Hardware BIT status (see eHdwBitStatusFlags) */
	uint32_t                hdwBitStatus;

	/** Calibration BIT status (see eCalBitStatusFlags) */
	uint32_t                calBitStatus;

	/** Temperature calibration bias */
	float                   tcPqrBias;
	float                   tcAccBias;

	/** Temperature calibration slope */
	float                   tcPqrSlope;
	float                   tcAccSlope;

	/** Temperature calibration linearity */
	float                   tcPqrLinearity;
	float                   tcAccLinearity;

	/** PQR motion (angular rate) */
	float                   pqr;

	/** ACC motion w/ gravity removed (linear acceleration) */
	float                   acc;

	/** Angular rate standard deviation */
	float                   pqrSigma;

	/** Acceleration standard deviation */
	float                   accSigma;
	
} bit_t;


/** System Configuration */
enum eSysConfigBits
{
	/*! Disable automatic baudrate detection on startup on serial port 0 and 1 */
	SYS_CFG_BITS_DISABLE_AUTOBAUD_SER0                  = (int)0x00000001,
	SYS_CFG_BITS_DISABLE_AUTOBAUD_SER1                  = (int)0x00000002,
	/*! Enable automatic mag recalibration */
	SYS_CFG_BITS_AUTO_MAG_RECAL                         = (int)0x00000004,
	/*! Disable mag declination estimation */
	SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION            = (int)0x00000008,

	/*! Disable LEDs */
	SYS_CFG_BITS_DISABLE_LEDS                           = (int)0x00000010,

	/** Magnetometer recalibration.  0 = multi-axis, 1 = single-axis */
	SYS_CFG_BITS_MAG_RECAL_MODE_MASK					= (int)0x00000700,
	SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET					= 8,
#define SYS_CFG_BITS_MAG_RECAL_MODE(sysCfgBits) ((sysCfgBits&SYS_CFG_BITS_MAG_RECAL_MODE_MASK)>>SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET)

	/** Disable magnetometer fusion */
	SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION			= (int)0x00001000,
	/** Disable barometer fusion */
	SYS_CFG_BITS_DISABLE_BAROMETER_FUSION				= (int)0x00002000,
	/** Disable GPS fusion */
	SYS_CFG_BITS_DISABLE_GPS_FUSION						= (int)0x00004000,

	/** Enable Zero Velocity Updates */
	SYS_CFG_BITS_ENABLED_ZERO_VELOCITY_UPDATES			= (int)0x00010000,

	/** Enable Nav update strobe output pulse on GPIO 9 (uINS pin 10) indicating preintegrated IMU and nav updates */
	SYS_CFG_BITS_ENABLE_NAV_STROBE_OUT_GPIO_9			= (int)0x00100000
};

/** RTK Configuration */
enum eRTKConfigBits
{
	/** Enable RTK rover on GPS1 if available */
	RTK_CFG_BITS_GPS1_RTK_ROVER							= (int)0x00000001,
	
	/** Enable RTK rover on GPS2 if available NOT SUPPORTED */
	RTK_CFG_BITS_GPS2_RTK_ROVER							= (int)0x00000002,
	
	/** Perform Compassing with dual GPS unit */
	RTK_CFG_BITS_COMPASSING								= (int)0x00000008,	
	
	/** Enable RTK base and output ublox data from GPS 1 on serial port 0 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0			= (int)0x00000010,

	/** Enable RTK base and output ublox data from GPS 1 on serial port 1 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1			= (int)0x00000020,
	
	/** Enable RTK base and output RTCM3 data from GPS 1 on serial port 0 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0			= (int)0x00000040,
	
	/** Enable RTK base and output RTCM3 data from GPS 1 on serial port 1 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1			= (int)0x00000080,
	
	/** Enable RTK base and output ublox data from GPS 2 on serial port 0 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0			= (int)0x00000100,

	/** Enable RTK base and output ublox data from GPS 2 on serial port 1 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1			= (int)0x00000200,
	
	/** Enable RTK base and output RTCM3 data from GPS 2 on serial port 0 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0			= (int)0x00000400,
	
	/** Enable RTK base and output RTCM3 data from GPS 2 on serial port 1 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1			= (int)0x00000800,	
	
	/** Enable base mode moving position. (For future use. Not implemented. This bit should always be 0 for now.) TODO: Implement moving base. */
	RTK_CFG_BITS_BASE_POS_MOVING						= (int)0x00001000,
	
	/** Reserved for future use */
	RTK_CFG_BITS_RESERVED1								= (int)0x00002000,	
	
	/** When using RTK, specifies whether the base station is identical hardware to this rover. If so, there are optimizations enabled to get fix faster. */
	RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER			= (int)0x00004000,
	
	/** All base station bits */
	RTK_CFG_BITS_RTK_BASE = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1),
									
	/** All base station bits for serial 0 */
	RTK_CFG_BITS_RTK_BASE_SER0 = (RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0),
	
	/** All base station bits for serial 1 */
	RTK_CFG_BITS_RTK_BASE_SER1 = (RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1
		| RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1),
							
	/** All rover bits */
	RTK_CFG_BITS_RTK_ROVER = (RTK_CFG_BITS_GPS1_RTK_ROVER | RTK_CFG_BITS_GPS2_RTK_ROVER),

	/** Mask of RTK modes */
	RTK_CFG_BITS_RTK_MODE_MASK = (RTK_CFG_BITS_RTK_ROVER | RTK_CFG_BITS_COMPASSING),
	
	/** RTK Enabled */
	RTK_CFG_BITS_RTK = (RTK_CFG_BITS_RTK_ROVER | RTK_CFG_BITS_RTK_BASE)
	
};


/** Sensor Configuration */
enum eSensorConfig
{
	/** Gyro full-scale sensing range selection: +- 250, 500, 1000, 2000 deg/s */	
	SENSOR_CFG_GYR_FS_250				= (int)0x00000000,
	SENSOR_CFG_GYR_FS_500				= (int)0x00000001,
	SENSOR_CFG_GYR_FS_1000				= (int)0x00000002,
	SENSOR_CFG_GYR_FS_2000				= (int)0x00000003,
	SENSOR_CFG_GYR_FS_MASK				= (int)0x00000003,
	SENSOR_CFG_GYR_FS_OFFSET			= (int)0,
	
	/** Accelerometer full-scale sensing range selection: +- 2, 4, 8, 16 m/s^2 */
	SENSOR_CFG_ACC_FS_2G				= (int)0x00000000,
	SENSOR_CFG_ACC_FS_4G				= (int)0x00000004,
	SENSOR_CFG_ACC_FS_8G				= (int)0x00000008,
	SENSOR_CFG_ACC_FS_16G				= (int)0x0000000C,
	SENSOR_CFG_ACC_FS_MASK				= (int)0x0000000C,
	SENSOR_CFG_ACC_FS_OFFSET			= (int)2,
	
	/** Gyro digital low-pass filter (DLPF) is set automatically based on the IMU sample rate.  The following 
	bit values can be used to override the bandwidth (frequency) to: 250, 184, 92, 41, 20, 10, 5 Hz */
	SENSOR_CFG_GYR_DLPF_250HZ			= (int)0x00000000,
	SENSOR_CFG_GYR_DLPF_184HZ			= (int)0x00000100,
	SENSOR_CFG_GYR_DLPF_92HZ			= (int)0x00000200,
	SENSOR_CFG_GYR_DLPF_41HZ			= (int)0x00000300,
	SENSOR_CFG_GYR_DLPF_20HZ			= (int)0x00000400,
	SENSOR_CFG_GYR_DLPF_10HZ			= (int)0x00000500,
	SENSOR_CFG_GYR_DLPF_5HZ				= (int)0x00000600,
	SENSOR_CFG_GYR_DLPF_MASK			= (int)0x00000F00,
	SENSOR_CFG_GYR_DLPF_OFFSET			= (int)8,

	/** Accelerometer digital low-pass filter (DLPF) is set automatically based on the IMU sample rate.  The 
	following bit values can be used to override the bandwidth (frequency) to: 218, 218, 99, 45, 21, 10, 5 Hz */
	SENSOR_CFG_ACC_DLPF_218HZ			= (int)0x00000000,
	SENSOR_CFG_ACC_DLPF_218HZb			= (int)0x00001000,
	SENSOR_CFG_ACC_DLPF_99HZ			= (int)0x00002000,
	SENSOR_CFG_ACC_DLPF_45HZ			= (int)0x00003000,
	SENSOR_CFG_ACC_DLPF_21HZ			= (int)0x00004000,
	SENSOR_CFG_ACC_DLPF_10HZ			= (int)0x00005000,
	SENSOR_CFG_ACC_DLPF_5HZ				= (int)0x00006000,
	SENSOR_CFG_ACC_DLPF_MASK			= (int)0x0000F000,
	SENSOR_CFG_ACC_DLPF_OFFSET			= (int)12,
	
};


/** IO configuration */
enum eIoConfig
{
	IO_CFG_INPUT_STROBE_TRIGGER_HIGH	= (int)0x00000001,
};


/** (DID_FLASH_CONFIG) Configuration data
 * IMPORTANT! These fields should not be deleted, they can be deprecated and marked as reserved,
 * or new fields added to the end.
*/
typedef struct PACKED
{
    /** Size of group or union, which is nvm_group_x_t + padding */
    uint32_t				size;

    /** Checksum, excluding size and checksum */
    uint32_t                checksum;

    /** Manufacturer method for restoring flash defaults */
    uint32_t                key;

    /** IMU sample (system input data) period in milliseconds set on startup. Cannot be larger than startupNavDtMs. Zero disables sensor/IMU sampling. */
    uint32_t				startupImuDtMs;

    /** Nav filter (system output data) update period in milliseconds set on startup. 2ms minimum (500Hz max). Zero disables nav filter updates. */
    uint32_t				startupNavDtMs;

    /** Serial port 0 baud rate in bits per second */
    uint32_t				ser0BaudRate;

    /** Serial port 1 baud rate in bits per second */
    uint32_t				ser1BaudRate;

    /** Roll, pitch, yaw euler angle rotation in radians from INS Sensor Frame to Intermediate Output Frame.  Order applied: heading, pitch, roll. */
    float					insRotation[3];

    /** X,Y,Z offset in meters from Intermediate Output Frame to INS Output Frame. */
    float					insOffset[3];

    /** X,Y,Z offset in meters from Sensor Frame origin to GPS 1 antenna. */
    float					gps1AntOffset[3];
 
    /** INS dynamic platform model.  Options are: 0=PORTABLE, 2=STATIONARY, 3=PEDESTRIAN, 4=AUTOMOTIVE, 5=SEA, 6=AIRBORNE_1G, 7=AIRBORNE_2G, 8=AIRBORNE_4G, 9=WRIST.  Used to balance noise and performance characteristics of the system.  The dynamics selected here must be at least as fast as your system or you experience accuracy error.  This is tied to the GPS position estimation model and intend in the future to be incorporated into the INS position model. */
    uint32_t				insDynModel;

    /** System configuration bits (see eSysConfigBits). */
    uint32_t				sysCfgBits;

    /** Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations (deg, deg, m) */
    double                  refLla[3];

    /** Last latitude, longitude, HAE (height above ellipsoid) used to aid GPS startup (deg, deg, m) */
    double					lastLla[3];

    /** Last LLA time since week start (Sunday morning) in milliseconds */
    uint32_t				lastLlaTimeOfWeekMs;

    /** Last LLA number of weeks since January 6th, 1980 */
    uint32_t				lastLlaWeek;

    /** Distance between current and last LLA that triggers an update of lastLla  */
    float					lastLlaUpdateDistance;

    /** Hardware interface configuration bits */
    uint32_t				ioConfig;

    /** Carrier board (i.e. eval board) configuration bits */
    uint32_t				cBrdConfig;

    /** X,Y,Z offset in meters from DOD_ Frame origin to GPS 2 antenna. */
    float					gps2AntOffset[3];

    /** Euler (roll, pitch, yaw) rotation in radians from INS Sensor Frame to Intermediate ZeroVelocity Frame.  Order applied: heading, pitch, roll. */
    float					zeroVelRotation[3];

    /** X,Y,Z offset in meters from Intermediate ZeroVelocity Frame to Zero Velocity Frame. */
    float					zeroVelOffset[3];

    /** Earth magnetic field (magnetic north) inclination (negative pitch offset) in radians */
    float                   magInclination;

    /** Earth magnetic field (magnetic north) declination (heading offset from true north) in radians */
    float                   magDeclination;

    /** Time between GPS time synchronization pulses in milliseconds.  Requires reboot to take effect. */
    uint32_t				gpsTimeSyncPeriodMs;
	
	/** GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max). */
    uint32_t				startupGPSDtMs;
	
	/** RTK configuration bits (see eRTKConfigBits). */
    uint32_t				RTKCfgBits;

    /** Sensor config (see eSensorConfig) */
    uint32_t                sensorConfig;

} nvm_flash_cfg_t;

/** INL2 - Estimate error variances */
typedef struct PACKED
{											
    /** Timestamp in milliseconds */
	unsigned int			timeOfWeekMs;	
    /** NED position error sigma */
	float					PxyzNED[3];		
    /** NED velocity error sigma */
	float					PvelNED[3];		
    /** NED attitude error sigma */
	float					PattNED[3];		
    /** Acceleration bias error sigma */
	float					PABias[3];		
    /** Angular rate bias error sigma */
	float					PWBias[3];		
    /** Barometric altitude bias error sigma */
	float					PBaroBias;		
    /** Mag declination error sigma */
	float					PDeclination;	
} inl2_ned_sigma_t;

/** (DID_STROBE_IN_TIME) Timestamp for input strobe. */
typedef struct PACKED
{
	/** Weeks since January 6th, 1980 */
	uint32_t				week;

	/** Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t				timeOfWeekMs;

	/** Strobe input pin */
	uint32_t				pin;

	/** Strobe serial index number */
	uint32_t				count;
} strobe_in_time_t;

POP_PACK

PUSH_PACK_8

/** time struct */
typedef struct
{
	/** time (s) expressed by standard time_t */
	int64_t time;

	/** fraction of second under 1 s */
	double sec;         
} gtime_t;

POP_PACK

PUSH_PACK_1

/** (DID_GPS_RTK_OPT) RTK processing options */
typedef struct
{
	/** positioning mode (PMODE_???) */
	int32_t mode;           

	/** solution type (0:forward,1:backward,2:combined) */
	int32_t soltype;

	/** number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
	int32_t nf;

	/** navigation systems */
	int32_t navsys;

	/** elevation mask angle (rad) */
	double elmin;

	/** Min snr to consider satellite for rtk */
	int32_t snrmin;

	/** AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
	int32_t modear;

	/** GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
	int32_t glomodear;

	/** GPS AR mode (0:off,1:on) */
	int32_t gpsmodear;

	/** BeiDou AR mode (0:off,1:on) */
	int32_t bdsmodear;

	/** AR filtering to reject bad sats (0:off,1:on) */
	int32_t arfilter;

	/** obs outage count to reset bias */
	int32_t maxout;

    /** reject count to reset bias */
    int32_t maxrej;

	/** min lock count to fix ambiguity */
	int32_t minlock;

	/** min sats to fix integer ambiguities */
	int32_t minfixsats;

	/** min sats to hold integer ambiguities */
	int32_t minholdsats;

	/** min sats to drop sats in AR */
	int32_t mindropsats;

	/** use stdev estimates from receiver to adjust measurement variances */
	int32_t rcvstds;

	/** min fix count to hold ambiguity */
	int32_t minfix;

	/** max iteration to resolve ambiguity */
	int32_t armaxiter;

	/** dynamics model (0:none,1:velociy,2:accel) */
	int32_t dynamics;

	/** number of filter iteration */
	int32_t niter;

	/** interpolate reference obs (for post mission) */
	int32_t intpref;

	/** rover position for fixed mode */
	int32_t rovpos;

	/** base position for relative mode */
	int32_t refpos;

	/** code/phase error ratio */
	double eratio[1];

	/** measurement error factor */
	double err[5];

	/** initial-state std [0]bias,[1]iono [2]trop */
	double std[3];

	/** process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
	double prn[6];

	/** satellite clock stability (sec/sec) */
	double sclkstab;

	/** AR validation threshold */
	double thresar[8];

	/** elevation mask of AR for rising satellite (rad) */
	double elmaskar;

	/** elevation mask to hold ambiguity (rad) */
	double elmaskhold;

	/** slip threshold of geometry-free phase (m) */
	double thresslip;

	/** variance for fix-and-hold pseudo measurements (cycle^2) */
	double varholdamb;

	/** gain used for GLO and SBAS sats to adjust ambiguity */
	double gainholdamb;

	/** max difference of time (sec) */
	double maxtdiff;

    /** reject threshold of NIS */
    double maxinnocode;
    double maxinnophase;

    /** max number of measurement rejections before bias reset */
    double maxrejc;

	/** reject threshold of gdop */
	double maxgdop;

	/** baseline length constraint {const,sigma} (m) */
	double baseline[2];
    double max_baseline_error;

	/** rover position for fixed mode {x,y,z} (ecef) (m) */
	double ru[3];

	/** base position for relative mode {x,y,z} (ecef) (m) */
	double rb[3];

	/** max averaging epochs */
	int32_t maxaveep;

	/** output single by dgps/float/fix/ppp outage */
	int32_t outsingle;
} prcopt_t;
typedef prcopt_t gps_rtk_opt_t;

/** Raw satellite observation data */
typedef struct PACKED
{
	/** receiver sampling time (GPST) */
	gtime_t time;

	/** satellite number */
	uint8_t sat;

	/** receiver number */
	uint8_t rcv;

	/** signal strength (0.25 dBHz) */
	uint8_t SNR[1];

	/** loss of lock indicator bit1=loss-of-lock, bit2=half-cycle-invalid */
	uint8_t LLI[1];

	/** code indicator (BeiDou: CODE_L1I, Other: CODE_L1C ) */
	uint8_t code[1];

	/** Estimated carrier phase measurement standard deviation (0.004 cycles) */
	uint8_t qualL[1];

	/** Estimated pseudorange measurement standard deviation (0.01 m) */
	uint8_t qualP[1];

	/** reserved, for alignment */
	uint8_t reserved;

	/** observation data carrier-phase (cycle) */
	double L[1];

	/** observation data pseudorange (m) */
	double P[1]; 

	/** observation data doppler frequency (0.002 Hz) */
	float D[1];
} obsd_t;

#define GPS_RAW_MESSAGE_BUF_SIZE    1000
#define MAX_OBSERVATION_COUNT_IN_RTK_MESSAGE (GPS_RAW_MESSAGE_BUF_SIZE / sizeof(obsd_t))

/** observation data */
typedef struct
{
	/** number of observation slots used */
	int32_t n;

	/** number of observation slots allocated */
	int32_t nmax;

	/** observation data buffer */
	obsd_t* data;
} obs_t;

/** non-Glonass ephemeris data */
typedef struct
{
	/** satellite number */
	int32_t sat;

	/** IODE Issue of Data, Ephemeris (ephemeris version) */
	int32_t iode;
	
	/** IODC Issue of Data, Clock (clock version) */
	int32_t iodc;

	/** SV accuracy (URA index) IRN-IS-200H p.97 */
	int32_t sva;            

	/** SV health GPS/QZS (0:ok) */
	int32_t svh;            

	/** GPS/QZS: gps week, GAL: galileo week */
	int32_t week;

	/** GPS/QZS: code on L2
	 * (00=Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid)
	 * GAL/CMP: data sources */
	int32_t code;

	/** GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel)
	 *
	 *  CMP: nav type */
	int32_t flag;

	/** Toe */
	gtime_t toe;
	
	/** clock data reference time (s) (20.3.4.5) */
	gtime_t toc;
	
	/** T_trans */
	gtime_t ttr;

	/** Semi-Major Axis m */
	double A;

	/** Eccentricity (no units)  */
	double e;

	/** Inclination Angle at Reference Time (rad) */
	double i0;

	/** Longitude of Ascending Node of Orbit Plane at Weekly Epoch (rad) */
	double OMG0;

	/** Argument of Perigee (rad) */
	double omg;

	/** Mean Anomaly at Reference Time (rad) */
	double M0;

	/** Mean Motion Difference From Computed Value (rad) */
	double deln;

	/** Rate of Right Ascension (rad/s) */
	double OMGd;

	/** Rate of Inclination Angle (rad/s) */
	double idot;

	/** Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius */
	double crc;

	/** Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m) */
	double crs;

	/** Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad)  */
	double cuc;

	/** Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad) */
	double cus;

	/** Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad) */
	double cic;

	/** Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad) */
	double cis;

	/** Reference Time Ephemeris in week (s) */
	double toes;

	/** fit interval (h) (0: 4 hours, 1:greater than 4 hours) */
	double fit;

	/** SV clock parameters - af0 */
	double f0;
	
	/** SV clock parameters - af1 */
	double f1;
	
	/** SV clock parameters - af2 */
	double f2;

	/** group delay parameters
	* GPS/QZS:tgd[0]=TGD (IRN-IS-200H p.103)
	* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1
	* CMP    :tgd[0]=BGD1,tgd[1]=BGD2
	*/
	double tgd[4];

	/** Adot for CNAV */
	double Adot;
	
	/** ndot for CNAV */
	double ndot;
} eph_t;

/** Glonass ephemeris data */
typedef struct
{        
	/** satellite number */
	int32_t sat;

	/** IODE (0-6 bit of tb field) */
	int32_t iode;

	/** satellite frequency number */
	int32_t frq;

	/** satellite health */
	int32_t svh;
	
	/** satellite accuracy */
	int32_t sva;
	
	/** satellite age of operation */
	int32_t age;

	/** epoch of epherides (gpst) */
	gtime_t toe;

	/** message frame time (gpst) */
	gtime_t tof;

	/** satellite position (ecef) (m) */
	double pos[3];

	/** satellite velocity (ecef) (m/s) */
	double vel[3];

	/** satellite acceleration (ecef) (m/s^2) */
	double acc[3];

	/** SV clock bias (s) */
	double taun;

	/** relative freq bias */
	double gamn;

	/** delay between L1 and L2 (s) */
	double dtaun;
} geph_t;

/** SBAS message type */
typedef struct
{
	/** receiption time - week */
	int32_t week;
	
	/** reception time - tow */
	int32_t tow;

	/** SBAS satellite PRN number */
	int32_t prn;

	/** SBAS message (226bit) padded by 0 */
	uint8_t msg[29];

	/** reserved for alighment */
	uint8_t reserved[3];
} sbsmsg_t;

/** station parameter type */
typedef struct
{
	/** antenna delta type (0:enu,1:xyz) */
	int32_t deltype;
	
	/** station position (ecef) (m) */
	double pos[3];

	/** antenna position delta (e/n/u or x/y/z) (m) */
	double del[3];

	/** antenna height (m) */
	double hgt;
	
	/** station id */
	int32_t stationId;
} sta_t;

/** almanac type */
typedef struct
{
	/** satellite number */
	int32_t sat;

	/** sv health (0:ok) */
	int32_t svh;

	/** as and sv config */
	int32_t svconf;

	/* GPS/QZS: gps week, GAL: galileo week */
	int32_t week;

	/* Toa */
	gtime_t toa;        
						
	/** SV orbit parameters - A */
	double A;

	/** SV orbit parameters - e */
	double e;

	/** SV orbit parameters - i0 */
	double i0;

	/** SV orbit parameters - OMG0 */
	double OMG0;
	
	/** SV orbit parameters - omg */
	double omg;
	
	/** SV orbit parameters - M0 */
	double M0;
	
	/** SV orbit parameters - OMGd */
	double OMGd;

	/** Toa (s) in week - toas */
	double toas;

	/** SV clock parameters - af0 */
	double f0;
	
	/** SV clock parameters - af1 */
	double f1;
} alm_t;

/** ionosphere model and utc parameters */
typedef struct
{
	double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
	double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
	double ion_irn[8];  /* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */

	double utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
	double utc_glo[4];  /* GLONASS UTC GPS time parameters */
	double utc_gal[4];  /* Galileo UTC GPS time parameters */
	double utc_qzs[4];  /* QZS UTC GPS time parameters */
	double utc_cmp[4];  /* BeiDou UTC parameters */
	double utc_irn[4];  /* IRNSS UTC parameters */
	double utc_sbs[4];  /* SBAS UTC parameters */

	int32_t leaps;      /* leap seconds (s) */
	
	alm_t alm;			/* almanac */
} ion_model_utc_alm_t;

/** RTK solution status */
typedef enum
{
	/** No status */
	rtk_solution_status_none = 0,

	/** RTK fix */
	rtk_solution_status_fix = 1,

	/** RTK float */
	rtk_solution_status_float = 2,

	/** RTK SBAS */
	rtk_solution_status_sbas = 3,

	/** RTK DGPS */
	rtk_solution_status_dgps = 4,

	/** RTK SINGLE */
	rtk_solution_status_single = 5
} eRtkSolStatus;

/** (DID_GPS1_RTK_REL) */
typedef struct PACKED
{
    /** Time of week (since Sunday morning) in milliseconds, GMT */
    uint32_t                timeOfWeekMs;

    /** Age of differential (seconds) */
    float					differentialAge;

    /** Ambiguity resolution ratio factor for validation */
    float					arRatio;

	/** Vector to base (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1 */
	float					vectorToBase[3];

    /** Distance to Base (m) */
    float                   distanceToBase;
    
    /** Angle from north to vectorToBase in local tangent plane. (rad) */
    float                   headingToBase;
} gps_rtk_rel_t;

/** (DID_GPS1_RTK_MISC) - requires little endian CPU */
typedef struct PACKED
{
	/** Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/** Accuracy - estimated standard deviations of the solution assuming a priori error model and error parameters by the positioning options.
	[]: standard deviations {ECEF - x,y,z} or {north, east, down} (meters) */
	float					accuracyPos[3];

	/** Accuracy - estimated standard deviations of the solution assuming a priori error model and error parameters by the positioning options.
	[]: Absolute value of means square root of estimated covariance NE, EU, UN */
	float					accuracyCov[3];

	/** Ambiguity resolution threshold for validation */
	float					arThreshold;

	/** Geometric dilution of precision (meters) */
	float					gDop;
	
	/** Horizontal dilution of precision (meters) */
	float					hDop;
	
	/** Vertical dilution of precision (meters) */
	float					vDop;

	/** Base Position - latitude, longitude, height (degrees, meters) */
 	double					baseLla[3];

    /** Cycle slip counter */
    uint32_t                cycleSlipCount;
	
    /** Rover gps observation element counter */
    uint32_t				roverGpsObservationCount;

    /** Base station gps observation element counter */
    uint32_t				baseGpsObservationCount;

    /** Rover glonass observation element counter */
    uint32_t				roverGlonassObservationCount;

    /** Base station glonass observation element counter */
    uint32_t				baseGlonassObservationCount;

    /** Rover galileo observation element counter */
    uint32_t				roverGalileoObservationCount;

    /** Base station galileo observation element counter */
    uint32_t				baseGalileoObservationCount;

    /** Rover beidou observation element counter */
    uint32_t				roverBeidouObservationCount;

    /** Base station beidou observation element counter */
    uint32_t				baseBeidouObservationCount;

    /** Rover qzs observation element counter */
    uint32_t				roverQzsObservationCount;

    /** Base station qzs observation element counter */
    uint32_t				baseQzsObservationCount;

	/** Rover gps ephemeris element counter */
	uint32_t				roverGpsEphemerisCount;

	/** Base station gps ephemeris element counter */
    uint32_t				baseGpsEphemerisCount;

	/** Rover glonass ephemeris element counter */
	uint32_t				roverGlonassEphemerisCount;

	/** Base station glonass ephemeris element counter */
    uint32_t				baseGlonassEphemerisCount;
	
	/** Rover galileo ephemeris element counter */
	uint32_t				roverGalileoEphemerisCount;

	/** Base station galileo ephemeris element counter */
    uint32_t				baseGalileoEphemerisCount;

    /** Rover beidou ephemeris element counter */
    uint32_t				roverBeidouEphemerisCount;

    /** Base station beidou ephemeris element counter */
    uint32_t				baseBeidouEphemerisCount;

    /** Rover qzs ephemeris element counter */
    uint32_t				roverQzsEphemerisCount;

    /** Base station qzs ephemeris element counter */
    uint32_t				baseQzsEphemerisCount;

	/** Rover sbas element counter */
	uint32_t				roverSbasCount;

	/** Base station sbas element counter */
    uint32_t				baseSbasCount;

	/** Base station antenna position element counter */
    uint32_t				baseAntennaCount;

    /** Ionosphere model, utc and almanac count */
    uint32_t				ionUtcAlmCount;
} gps_rtk_misc_t;

/** RAW data types for DID_GPS_BASE_RAW and DID_GPS2_RAW */
typedef enum
{
	/** obsd_t */
	raw_data_type_observation = 1,

	/** eph_t */
	raw_data_type_ephemeris = 2,

	/** geph_t */
	raw_data_type_glonass_ephemeris = 3,

	/** sbsmsg_t */
	raw_data_type_sbas = 4,

	/** sta_t */
	raw_data_type_base_station_antenna_position = 5,

	/** ion_model_utc_alm_t */
	raw_data_type_ionosphere_model_utc_alm = 6,
	
	/** gps_rtk_misc_t */
	raw_data_type_rtk_solution = 123
} eRawDataType;

typedef union PACKED
{   
    /** Satellite observation data */
    obsd_t              obs[MAX_OBSERVATION_COUNT_IN_RTK_MESSAGE];
    
    /** Satellite non-GLONASS ephemeris data (GPS, Galileo, Beidou, QZSS) */
    eph_t               eph;
    
    /** Satellite GLONASS ephemeris data */
    geph_t              gloEph;
    
    /** Satellite-Based Augmentation Systems (SBAS) data */
    sbsmsg_t            sbas;
        
    /** Base station information (base position, antenna position, antenna height, etc.) */
    sta_t               sta;

    /** Ionosphere model and UTC parameters */
    ion_model_utc_alm_t ion;

    /** Byte buffer */
    uint8_t             buf[GPS_RAW_MESSAGE_BUF_SIZE];
} uGpsRawData;

/** Message wrapper for DID_GPS1_RAW, DID_GPS2_RAW, and DID_GPS_BASE_RAW.  The contents of data can vary for this message and are determined by `dataType` field. */
typedef struct PACKED
{
	/** Receiver index (1=RECEIVER_INDEX_GPS1, 2=RECEIVER_INDEX_EXTERNAL_BASE, or 3=RECEIVER_INDEX_GPS2 ) */
	uint8_t receiverIndex;

	/** Type of data (eRawDataType: 1=observations, 2=ephemeris, 3=glonassEphemeris, 4=SBAS, 5=baseAntenna, 6=IonosphereModel) */
	uint8_t dataType;

	/** Number of observations in data (obsd_t) when dataType==1 (raw_data_type_observation). */
	uint8_t obsCount;

	/** Reserved */
	uint8_t reserved;

    /** Interpret based on dataType (see eRawDataType) */    
  uGpsRawData data;
} gps_raw_t;

typedef struct PACKED
{
	/** Time measure was taken */
	uint32_t time_ms;

	/** ID of sensor **/
	uint32_t id;

	/** Velocity Measurement (xyz) */
	float vel[3];

	/** Covariance Matrix Diagonal (E[xx], E[yy], E[zz]) */
	float cov[3];

	/** Quaternion rotation from IMU frame to sensor frame {w, x, y, z} **/
	float q[4];

	/** Sensor origin position in IMU frame { x, y, z} **/
	float p[3];

	/** Valid velocity vector terms {x, y, z} **/
	uint8_t valid[3];

	/** Used to keep 32-bit alignment **/
	uint8_t reserved;

} velocity_sensor_t;

/**
* Diagnostic message
*/
typedef struct 
{
	/** Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t timeOfWeekMs;
	
	/** Message length, including null terminator */
	uint32_t messageLength;
	
	/** Message data, max size of message is 256 */
	char message[256];
} diag_msg_t;

typedef enum
{
	// default state
	SURVEY_IN_STATE_OFF                     = 0,

	// commands
	SURVEY_IN_STATE_CANCEL                  = 1,
	SURVEY_IN_STATE_START_3D                = 2,
	SURVEY_IN_STATE_START_FLOAT             = 3,
	SURVEY_IN_STATE_START_FIX               = 4,

	// status
	SURVEY_IN_STATE_RUNNING_3D              = 8,
	SURVEY_IN_STATE_RUNNING_FLOAT           = 9,
	SURVEY_IN_STATE_RUNNING_FIX             = 10,
	SURVEY_IN_STATE_SAVE_POS                = 19,
	SURVEY_IN_STATE_DONE                    = 20
} eSurveyInStatus;

/**
* Survey in status
*/
typedef struct
{
	/** State of current survey, eSurveyInStatus */
	uint32_t state;

	/** Maximum time (milliseconds) survey will run if minAccuracy is not first achieved. (ignored if 0). */
	uint32_t maxDurationSec;

	/** Required horizontal accuracy (m) for survey to complete before maxDuration. (ignored if 0) */
	float minAccuracy;

	/** Elapsed time (seconds) of the survey. */
	uint32_t elapsedTimeSec;

	/** Approximate horizontal accuracy of the survey (m). */
	float hAccuracy;

	/** The current surveyed latitude, longitude, altitude (deg, deg, m) */
	double lla[3];
} survey_in_t;

/**
* (DID_EVB2) EVB-2 monitor, config, and logger control interface
*/
typedef struct
{
	/** Firmware (software) version */
	uint8_t         firmwareVer[4];
    
    /** Communications bridge configuration. (see eEvb2ComBridgeCfg) */
    uint32_t        comBridgeCfg;

    /** Data logger control state. (see eEvb2LoggerState) */
    uint32_t        loggerState;

    /** logger */
    uint32_t        loggerElapsedTimeMs;

	/** IP address */
	uint8_t         ipAddr[4];
    
} evb2_t;


/** EVB-2 communications bridge configuration. */
typedef enum
{
    /** Do not change.  Sending this value causes no effect. */
    EVB2_CBC_NA                         = 0,

    /** USB-SER0 */
    EVB2_CBD_USBxSER0                   = 1,

    /** USB-SPI */
    EVB2_CBC_USBxSPI                    = 2,
    
} eEvb2ComBridgeCfg;


/** Data logger control state.  Values labeled CMD  */
typedef enum
{
    /** Do not change.  Sending this value causes no effect. */
    EVB2_LOG_STATE_NA                   = 0,

    /** Start new log */
    EVB2_LOG_STATE_CMD_START            = 1,

    /** Logger running */
    EVB2_LOG_STATE_RUNNING              = 2,

    /** Pause current log */
    EVB2_LOG_STATE_CMD_PAUSE            = 3,

    /** Stop current log */
    EVB2_LOG_STATE_CMD_STOP             = 4,

    /** Purge all data logs from drive */
    EVB2_LOG_STATE_CMD_PURGE            = 1002,
        
} eEvb2LoggerState;


/** 
* (DID_PORT_MONITOR) Data rate and status monitoring for each communications port. 
*/
typedef struct
{
    /** Com port number  */
    uint32_t        portNumber;

//     /** Tx time ms */
//     uint32_t        txTimeMs;

    /** Tx rate (bytes/s) */
    uint32_t        txBytesPerS;

//     /** Rx time ms */
//     uint32_t        rxTimeMs;

    /** Rx rate (bytes/s) */
    uint32_t        rxBytesPerS;

    /** Status */
    uint32_t        status;
    
} port_monitor_t;


#define CRASH_INFO_NONE 0x00000000
#define CRASH_INFO_USER_RESET 0xFFFFFFFA
#define CRASH_INFO_ENABLE_BOOTLOADER 0xFFFFFFFB
#define CRASH_INFO_INVALID_CODE_OPERATION 0xFFFFFFFC
#define CRASH_INFO_MALLOC_FAILED 0xFFFFFFD
#define CRASH_INFO_SOFT_RESET 0xFFFFFFE
#define CRASH_INFO_STACK_OVERFLOW 0xFFFFFFFF

/**
* Crash Info message
* Special values:
* 0xFFFFFFFA: User reset
* 0xFFFFFFFB: Enable bootloader
* 0xFFFFFFFC: Invalid code operation
* 0xFFFFFFFD: soft reset was issued
* 0xFFFFFFFE: malloc failed
* 0xFFFFFFFF: stack overflow
* Others: Non-zero is Hard fault
*/
typedef struct 
{
	/** r0 value at time of HardFault */
	uint32_t r0;
	
    /** r1 value at time of HardFault */
	uint32_t r1;
    
    /** r2 value at time of HardFault */
	uint32_t r2;
    
    /** r3 value at time of HardFault */
	uint32_t r3;
    
    /** r12 value at time of HardFault */
	uint32_t r12;
    
    /** link register value at time of HardFault */
	uint32_t lr;
    
    /** Program Counter value at time of HardFault */
	uint32_t pc;
    
    /** Program Status Register value at time of HardFault */
	uint32_t psr;	
} crash_info_t;

/** Diagnostic information for internal use */
typedef struct
{
	/** Count of gap of more than 0.5 seconds receiving serial data, driver level, one entry for each com port */
	uint32_t gapCountSerialDriver[5];

	/** Count of gap of more than 0.5 seconds receiving serial data, class / parser level, one entry for each com port */
	uint32_t gapCountSerialParser[5];

	/** Count of rx overflow, one entry for each com port */
	uint32_t rxOverflowCount[5];

	/** Count of tx overflow, one entry for each com port */
	uint32_t txOverflowCount[5];
	
	/** Count of checksum failures, one entry for each com port */
	uint32_t checksumFailCount[5];
} internal_diagnostic_t;

/**
* Rtos task types
*/
typedef enum
{
	/**
	* Sample task
	*/
	TASK_SAMPLE = 0,

	/**
	* Nav task
	*/
	TASK_NAV,

	/**
	* Communications task
	*/
	TASK_COMMUNICATIONS,

	/**
	* Maintenance task
	*/
	TASK_MAINTENANCE,

	/**
	* Idle task
	*/
	TASK_IDLE,

	/**
	* Timer task
	*/
	TASK_TIMER,

	/**
	* Number of rtos tasks
	*/
	RTOS_NUM_TASKS                       // Keep last
} eRtosTask;

/**
* Max task name length - do not change
*/
#define MAX_TASK_NAME_LEN 12

/**
* Rtos task info
*/
typedef struct PACKED
{
	/**
	* Task name
	*/
	char                    name[MAX_TASK_NAME_LEN];

	/**
	* Task priority (0 - 8)
	*/
	uint32_t                priority;

	/**
	* Stack high water mark bytes
	*/
	uint32_t                stackUnused;

	/**
	* Task period ms
	*/
	uint32_t                periodMs;

	/**
	* Last run time microseconds
	*/
	uint32_t                runTimeUs;

	/**
	* Max run time microseconds
	*/
	uint32_t                maxRunTimeUs;
	
	/**
	* Rolling average over last 1000 executions
	*/
	float					averageRunTimeUs;
	
	/**
	* Counter of times task took too long to run
	*/
	uint32_t				gapCount;

	/**
	* Cpu usage percent
	*/
    float					cpuUsage;

	/**
	* Handle
	*/
	uint32_t                handle;
} rtos_task_t;

/** (DID_RTOS_INFO) */
typedef struct PACKED
{
	/** Tasks */
	rtos_task_t             task[RTOS_NUM_TASKS];

	/** Heap high water mark bytes */
	uint32_t                freeHeapSize;

	/** Malloc - free counter */
	uint32_t				mallocMinusFree;
} rtos_info_t;

/** Union of datasets */
typedef union PACKED
{
	dev_info_t				devInfo;
	ins_1_t					ins1;
	ins_2_t					ins2;
 	ins_3_t					ins3;
	ins_4_t					ins4;
	imu_t					imu;
	dual_imu_t				dualImu;
	magnetometer_t			mag;
	mag_cal_t				magCal;
	barometer_t				baro;
	preintegrated_imu_t		pImu;
	gps_pos_t				gpsPos;
	gps_vel_t				gpsVel;
	gps_sat_t				gpsSat;
	gps_rtk_rel_t			gpsRtkRel;
	gps_rtk_misc_t			gpsRtkMisc;
	inl2_states_t			inl2States;
	inl2_ned_sigma_t        inl2NedSigma;
	nvm_flash_cfg_t			flashCfg;
    survey_in_t             surveyIn;
	sys_params_t			sysParams;
	sys_sensors_t			sysSensors;
	rtos_info_t				rtosInfo;
	gps_raw_t				gpsRaw;
	sys_sensors_adc_t       sensorsAdc;
} uDatasets;

/** Union of INS output datasets */
typedef union PACKED
{
	ins_1_t					ins1;
	ins_2_t					ins2;
	ins_3_t					ins3;
	ins_4_t					ins4;
} uInsOutDatasets;

POP_PACK

/**
Creates a 32 bit checksum from data

@param data the data to create a checksum for
@param count the number of bytes in data

@return the 32 bit checksum for data
*/
uint32_t serialNumChecksum32(const void* data, int size);
uint32_t flashChecksum32(const void* data, int size);

/**
Flip the endianess of 32 bit values in data

@param data the data to flip 32 bit values in
@param dataLength the number of bytes in data
*/
void flipEndianess32(uint8_t* data, int dataLength);

/**
Flip the bytes of a float in place (4 bytes) - ptr is assumed to be at least 4 bytes

@param ptr the float to flip
*/
void flipFloat(uint8_t* ptr);

/**
Flip the bytes of a float (4 bytes) - ptr is assumed to be at least 4 bytes

@param val the float to flip
@return the flipped float
*/
float flipFloatCopy(float val);

/**
Flip the bytes of a double in place (8 bytes) - ptr is assumed to be at least 8 bytes
Only flips each 4 byte pair, does not flip the individual bytes within the pair

@param ptr the double to flip
*/
void flipDouble(uint8_t* ptr);

/**
Flip the bytes of a double in place (8 bytes)
Unlike flipDouble, this also flips the individual bytes in each 4 byte pair

@param val the double to flip
@return the flipped double
*/
double flipDoubleCopy(double val);

/**
Flip double (64 bit) floating point values in data

@param data the data to flip doubles in
@param dataLength the number of bytes in data
@param offset offset into data to start flipping at
@param offsets a list of offsets of all doubles in data, starting at position 0
@param offsetsLength the number of items in offsets
*/
void flipDoubles(uint8_t* data, int dataLength, int offset, uint16_t* offsets, uint16_t offsetsLength);

/**
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

/**
Get the offsets of double / int64 (64 bit) values given a data id

@param dataId the data id to get double offsets for
@param offsetsLength receives the number of double offsets

@return a list of offets of doubles or 0 if none, offset will have high bit set if it is an int64 instead of a double
*/
uint16_t* getDoubleOffsets(eDataIDs dataId, uint16_t* offsetsLength);

/**
Gets the offsets and lengths of strings given a data id

@param dataId the data id to get string offsets and lengths for
@param offsetsLength receives the number of items in the return value

@return a list of offsets and lengths of strings for the data id or 0 if none
*/
uint16_t* getStringOffsetsLengths(eDataIDs dataId, uint16_t* offsetsLength);

/** Convert DID to realtime message bits */
uint64_t didToRmcBit(uint32_t dataId, uint64_t defaultRmcBits);

/** Convert Julian Date to calendar date. */
void julianToDate(double julian, int32_t* year, int32_t* month, int32_t* day, int32_t* hours, int32_t* minutes, int32_t* seconds, int32_t* milliseconds);

/** Convert GPS Week and Seconds to Julian Date. */
double gpsToJulian(int32_t gpsWeek, int32_t gpsMilliseconds);

/*
Convert gps pos to nmea gga

@param gps gps position
@param buffer buffer to fill with nmea gga
@param bufferLength number of chars available in buffer, should be at least 128
@return number of chars written to buffer, not including the null terminator
*/
int gpsToNmeaGGA(const gps_pos_t* gps, char* buffer, int bufferLength);

#ifdef __cplusplus
}
#endif

#endif // DATA_SETS_H
