/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

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
#define DID_SYS_FAULT                   (eDataIDs)2  /** (system_fault_t) System fault information */
#define DID_PIMU                        (eDataIDs)3  /** (pimu_t) Preintegrated IMU (a.k.a. Coning and Sculling integral) in body/IMU frame.  Updated at IMU rate. Also know as delta theta delta velocity, or preintegrated IMU (PIMU). For clarification, the name "Preintegrated IMU" or "PIMU" throughout our User Manual. This data is integrated from the IMU data at the IMU update rate (startupImuDtMs, default 1ms).  The integration period (dt) and output data rate are the same as the NAV rate (startupNavDtMs) and cannot be output at any other rate. If a faster output data rate is desired, DID_IMU_RAW can be used instead. PIMU data acts as a form of compression, adding the benefit of higher integration rates for slower output data rates, preserving the IMU data without adding filter delay and addresses antialiasing. It is most effective for systems that have higher dynamics and lower communications data rates.  The minimum data period is DID_FLASH_CONFIG.startupImuDtMs or 4, whichever is larger (250Hz max). The PIMU value can be converted to IMU by dividing PIMU by dt (i.e. IMU = PIMU / dt)  */
#define DID_INS_1                       (eDataIDs)4  /** (ins_1_t) INS output: euler rotation w/ respect to NED, NED position from reference LLA. */
#define DID_INS_2                       (eDataIDs)5  /** (ins_2_t) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
#define DID_GNSS1_UBX_POS               (eDataIDs)6  /** (gnss_pos_t) GNSS 1 position data from ublox receiver. */
#define DID_SYS_CMD                     (eDataIDs)7  /** (system_command_t) System commands. Both the command and invCommand fields must be set at the same time for a command to take effect. */
#define DID_ASCII_BCAST_PERIOD          (eDataIDs)8  /** (ascii_msgs_t) Broadcast period for ASCII messages */
#define DID_RMC                         (eDataIDs)9  /** (rmc_t) Realtime Message Controller (RMC). The data sets available through RMC are driven by the availability of the data. The RMC provides updates from various data sources (i.e. sensors) as soon as possible with minimal latency. Several of the data sources (sensors) output data at different data rates that do not all correspond. The RMC is provided so that broadcast of sensor data is done as soon as it becomes available. All RMC messages can be enabled using the standard Get Data packet format. */
#define DID_SYS_PARAMS                  (eDataIDs)10 /** (sys_params_t) System parameters / info */
#define DID_SYS_SENSORS                 (eDataIDs)11 /** (sys_sensors_t) System sensor information */
#define DID_FLASH_CONFIG                (eDataIDs)12 /** (nvm_flash_cfg_t) Flash memory configuration */
#define DID_GNSS1_POS                   (eDataIDs)13 /** (gnss_pos_t) GNSS 1 position data.  This comes from IMX_DID_GNSS1_UBX_POS or IMX_DID_GNSS1_RTK_POS, depending on whichever is more accurate. */
#define DID_GNSS2_POS                   (eDataIDs)14 /** (gnss_pos_t) GNSS 2 position data */
#define DID_GNSS1_SAT                   (eDataIDs)15 /** (gnss_sat_t) GNSS 1 GNSS and sat identifiers, carrier to noise ratio (signal strength), elevation and azimuth angles, pseudo range residual. */
#define DID_GNSS2_SAT                   (eDataIDs)16 /** (gnss_sat_t) GNSS 2 GNSS and sat identifiers, carrier to noise ratio (signal strength), elevation and azimuth angles, pseudo range residual. */
#define DID_GNSS1_VERSION               (eDataIDs)17 /** (gnss_version_t) GNSS 1 version info */
#define DID_GNSS2_VERSION               (eDataIDs)18 /** (gnss_version_t) GNSS 2 version info */
#define DID_MAG_CAL                     (eDataIDs)19 /** (mag_cal_t) Magnetometer calibration */
#define DID_INTERNAL_DIAGNOSTIC         (eDataIDs)20 /** INTERNAL USE ONLY (internal_diagnostic_t) Internal diagnostic info */
#define DID_GNSS1_RTK_POS_REL           (eDataIDs)21 /** (gnss_rtk_rel_t) RTK precision position base to rover relative info. */
#define DID_GNSS1_RTK_POS_MISC          (eDataIDs)22 /** (gnss_rtk_misc_t) RTK precision position related data. */
#define DID_FEATURE_BITS                (eDataIDs)23 /** INTERNAL USE ONLY (feature_bits_t) */
#define DID_SENSORS_UCAL                (eDataIDs)24 /** INTERNAL USE ONLY (sensors_w_temp_t) Uncalibrated IMU output. */
#define DID_SENSORS_TCAL                (eDataIDs)25 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated IMU output. */
#define DID_SENSORS_TC_BIAS             (eDataIDs)26 /** INTERNAL USE ONLY (sensors_t) */
#define DID_IO                          (eDataIDs)27 /** (io_t) I/O */
#define DID_SENSORS_ADC                 (eDataIDs)28 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_SCOMP                       (eDataIDs)29 /** INTERNAL USE ONLY (sensor_compensation_t) */
#define DID_GNSS1_VEL                   (eDataIDs)30 /** (gnss_vel_t) GNSS 1 velocity data */
#define DID_GNSS2_VEL                   (eDataIDs)31 /** (gnss_vel_t) GNSS 2 velocity data */
#define DID_HDW_PARAMS                  (eDataIDs)32 /** INTERNAL USE ONLY (hdw_params_t) */
#define DID_NVR_MANAGE_USERPAGE         (eDataIDs)33 /** INTERNAL USE ONLY (nvr_manage_t) */
#define DID_NVR_USERPAGE_SN             (eDataIDs)34 /** INTERNAL USE ONLY (nvm_group_sn_t) */
#define DID_NVR_USERPAGE_G0             (eDataIDs)35 /** INTERNAL USE ONLY (nvm_group_0_t) */
#define DID_NVR_USERPAGE_G1             (eDataIDs)36 /** INTERNAL USE ONLY (nvm_group_1_t) */
#define DID_DEBUG_STRING                (eDataIDs)37 /** INTERNAL USE ONLY (debug_string_t) */
#define DID_RTOS_INFO                   (eDataIDs)38 /** (rtos_info_t) RTOS information. */
#define DID_DEBUG_ARRAY                 (eDataIDs)39 /** INTERNAL USE ONLY (debug_array_t) */
#define DID_SENSORS_MCAL                (eDataIDs)40 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated and motion calibrated IMU output. */
#define DID_GNSS1_TIMEPULSE             (eDataIDs)41 /** INTERNAL USE ONLY (gnss_timepulse_t) */
#define DID_CAL_SC                      (eDataIDs)42 /** INTERNAL USE ONLY (sensor_cal_t) */
#define DID_CAL_TEMP_COMP               (eDataIDs)43 /** INTERNAL USE ONLY (sensor_tcal_group_t) */
#define DID_CAL_MOTION                  (eDataIDs)44 /** INTERNAL USE ONLY (sensor_mcal_group_t) */
// #define DID_UNUSED_45           		(eDataIDs)45 /** used to be internal DID_SYS_SENSORS_SIGMA */
#define DID_SENSORS_ADC_SIGMA           (eDataIDs)46 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_REFERENCE_MAGNETOMETER      (eDataIDs)47 /** (magnetometer_t) Reference or truth magnetometer used for manufacturing calibration and testing */
#define DID_INL2_STATES                 (eDataIDs)48 /** (inl2_states_t) */
#define DID_INL2_COVARIANCE_LD          (eDataIDs)49 /** (INL2_COVARIANCE_LD_ARRAY_SIZE) */
#define DID_INL2_STATUS                 (eDataIDs)50 /** (inl2_status_t) */
#define DID_INL2_MISC                   (eDataIDs)51 /** (inl2_misc_t) */
#define DID_MAGNETOMETER                (eDataIDs)52 /** (magnetometer_t) Magnetometer sensor output */
#define DID_BAROMETER                   (eDataIDs)53 /** (barometer_t) Barometric pressure sensor data */
#define DID_GNSS1_RTK_POS               (eDataIDs)54 /** (gnss_pos_t) GNSS RTK position data */
#define DID_ROS_COVARIANCE_POSE_TWIST   (eDataIDs)55 /** (ros_covariance_pose_twist_t) INL2 EKF covariances matrix lower diagonals */
#define DID_COMMUNICATIONS_LOOPBACK     (eDataIDs)56 /** INTERNAL USE ONLY - Unit test for communications manager  */
#define DID_IMU3_UNCAL                  (eDataIDs)57 /** INTERNAL USE ONLY (imu3_t) Uncalibrated triple IMU data.  We recommend use of DID_IMU or DID_PIMU as they are calibrated and oversampled and contain less noise.  Minimum data period is DID_FLASH_CONFIG.startupImuDtMs or 4, whichever is larger (250Hz max). */
#define DID_IMU                         (eDataIDs)58 /** (imu_t) Inertial measurement unit data down-sampled from IMU rate (DID_FLASH_CONFIG.startupImuDtMs (1KHz)) to navigation update rate (DID_FLASH_CONFIG.startupNavDtMs) as an anti-aliasing filter to reduce noise and preserve accuracy.  Minimum data period is DID_FLASH_CONFIG.startupNavDtMs (1KHz max).  */
#define DID_INL2_MAG_OBS_INFO           (eDataIDs)59 /** (inl2_mag_obs_info_t) INL2 magnetometer calibration information. */
#define DID_GNSS_BASE_RAW               (eDataIDs)60 /** (gnss_raw_t) GNSS raw data for base station (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_GNSS_RTK_OPT                (eDataIDs)61 /** (gnss_rtk_opt_t) RTK options - requires little endian CPU. */
#define DID_REFERENCE_PIMU              (eDataIDs)62 /** (pimu_t) Reference or truth IMU used for manufacturing calibration and testing */
#define DID_MANUFACTURING_INFO          (eDataIDs)63 /** INTERNAL USE ONLY (manufacturing_info_t) Manufacturing info */
#define DID_BIT                         (eDataIDs)64 /** (bit_t) System built-in self-test */
#define DID_INS_3                       (eDataIDs)65 /** (ins_3_t) Inertial navigation data with quaternion NED to body rotation and ECEF position. */
#define DID_INS_4                       (eDataIDs)66 /** (ins_4_t) INS output: quaternion rotation w/ respect to ECEF, ECEF position. */
#define DID_INL2_NED_SIGMA              (eDataIDs)67 /** (inl2_ned_sigma_t) Standard deviation of INL2 EKF estimates in the NED frame. */
#define DID_STROBE_IN_TIME              (eDataIDs)68 /** (strobe_in_time_t) Timestamp for input strobe. */
#define DID_GNSS1_RAW                   (eDataIDs)69 /** (gnss_raw_t) GNSS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_GNSS2_RAW                   (eDataIDs)70 /** (gnss_raw_t) GNSS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_WHEEL_ENCODER               (eDataIDs)71 /** (wheel_encoder_t) Wheel encoder data to be fused with GNSS-INS measurements, set DID_GROUND_VEHICLE for configuration before sending this message */
#define DID_DIAGNOSTIC_MESSAGE          (eDataIDs)72 /** (diag_msg_t) Diagnostic message */
#define DID_SURVEY_IN                   (eDataIDs)73 /** (survey_in_t) Survey in, used to determine position for RTK base station. Base correction output cannot run during a survey and will be automatically disabled if a survey is started. */
#define DID_CAL_SC_INFO                 (eDataIDs)74 /** INTERNAL USE ONLY (sensor_cal_info_t) */
#define DID_PORT_MONITOR                (eDataIDs)75 /** (port_monitor_t) Data rate and status monitoring for each communications port. */
#define DID_RTK_STATE                   (eDataIDs)76 /** INTERNAL USE ONLY (rtk_state_t) */
#define DID_RTK_PHASE_RESIDUAL          (eDataIDs)77 /** INTERNAL USE ONLY (rtk_residual_t) */
#define DID_RTK_CODE_RESIDUAL           (eDataIDs)78 /** INTERNAL USE ONLY (rtk_residual_t) */
#define DID_RTK_DEBUG                   (eDataIDs)79 /** INTERNAL USE ONLY (rtk_debug_t) */
#define DID_EVB_STATUS                  (eDataIDs)80 /** (evb_status_t) EVB monitor and log control interface. */
#define DID_EVB_FLASH_CFG               (eDataIDs)81 /** (evb_flash_cfg_t) EVB configuration. */
#define DID_EVB_DEBUG_ARRAY             (eDataIDs)82 /** INTERNAL USE ONLY (debug_array_t) */
#define DID_EVB_RTOS_INFO               (eDataIDs)83 /** (evb_rtos_info_t) EVB-2 RTOS information. */
// #define DID_UNUSED_84                (eDataIDs)84 /** Unused */
#define DID_IMU_MAG                     (eDataIDs)85 /** (imu_mag_t) DID_IMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define DID_PIMU_MAG                    (eDataIDs)86 /** (pimu_mag_t) DID_PIMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define DID_GROUND_VEHICLE				(eDataIDs)87 /** (ground_vehicle_t) Static configuration for wheel transform measurements. */
// #define DID_UNUSED_88				(eDataIDs)88 /** Unused */
#define DID_RTK_DEBUG_2                 (eDataIDs)89 /** INTERNAL USE ONLY (rtk_debug_2_t) */
#define DID_CAN_CONFIG					(eDataIDs)90 /** (can_config_t) Addresses for CAN messages*/
#define DID_GNSS2_RTK_CMP_REL           (eDataIDs)91 /** (gnss_rtk_rel_t) Dual GNSS RTK compassing / moving base to rover (GNSS 1 to GNSS 2) relative info. */
#define DID_GNSS2_RTK_CMP_MISC          (eDataIDs)92 /** (gnss_rtk_misc_t) RTK Dual GNSS RTK compassing related data. */
#define DID_EVB_DEV_INFO                (eDataIDs)93 /** (dev_info_t) EVB device information */
#define DID_INFIELD_CAL                 (eDataIDs)94 /** (infield_cal_t) Measure and correct IMU calibration error.  Estimate INS rotation to align INS with vehicle. */
#define DID_REFERENCE_IMU               (eDataIDs)95 /** (imu_t) Raw reference or truth IMU used for manufacturing calibration and testing. Input from testbed. */
#define DID_IMU3_RAW                    (eDataIDs)96 /** (imu3_t) Triple IMU data calibrated from DID_IMU3_UNCAL.  We recommend use of DID_IMU or DID_PIMU as they are oversampled and contain less noise. */
#define DID_IMU_RAW                     (eDataIDs)97 /** (imu_t) IMU data averaged from DID_IMU3_RAW.  Use this IMU data for output data rates faster than DID_FLASH_CONFIG.startupNavDtMs.  Otherwise we recommend use of DID_IMU or DID_PIMU as they are oversampled and contain less noise. */

// Adding a new data id?
// 1] Add it above and increment the previous number, include the matching data structure type in the comments
// 2] Add flip doubles and flip strings entries in data_sets.c
// 3] Add data id to ISDataMappings.cpp
// 4] Increment DID_COUNT
// 5) Update the DIDs in IS-src/python/src/ci_hdw/data_sets.py
// 6] Test!

/** Maximum number of data ids */
#define DID_MAX_COUNT 256U

/** Maximum number of satellite channels */
#define MAX_NUM_SAT_CHANNELS 64U
#define MAX_NUM_RTK_FREQ 3U

/** Maximum length of device info manufacturer string (must be a multiple of 4) */
#define DEVINFO_MANUFACTURER_STRLEN 24U
#define DEVINFO_ADDINFO_STRLEN 24U

/** Rtk rover receiver index */
#define RECEIVER_INDEX_GPS1 1U // DO NOT CHANGE
#define RECEIVER_INDEX_EXTERNAL_BASE 2U // DO NOT CHANGE
#define RECEIVER_INDEX_GPS2 3U // DO NOT CHANGE

// Max number of devices across all hardware types (uINS-3 and IMX-5)
#define NUM_IMU_DEVICES     3		// g_numImuDevices defines the actual number of hardware specific devices
#define NUM_MAG_DEVICES     2		// g_numMagDevices defines the actual number of hardware specific devices

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

	/** Velocity aided by wheel sensor */
	INS_STATUS_WHEEL_AIDING_VEL                 = (int)0x00000008,

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

	/** Position aided by GPS position */
	INS_STATUS_GPS_AIDING_POS                   = (int)0x00000100,
	/** GPS update event occurred in solution, potentially causing discontinuity in position path */
	INS_STATUS_GPS_UPDATE_IN_SOLUTION           = (int)0x00000200,
	/** Reserved for internal purpose */
	INS_STATUS_RESERVED_1                       = (int)0x00000400,																	
	/** Heading aided by magnetic heading */
	INS_STATUS_MAG_AIDING_HEADING               = (int)0x00000800,

	/** Nav mode (set) = estimating velocity and position. AHRS mode (cleared) = NOT estimating velocity and position */
	INS_STATUS_NAV_MODE							= (int)0x00001000,

	/** User should not move (keep system motionless) to assist on-board processing. */
	INS_STATUS_DO_NOT_MOVE						= (int)0x00002000,	
	/** Velocity aided by GPS velocity */
	INS_STATUS_GPS_AIDING_VEL                   = (int)0x00004000,
	/** Vehicle kinematic calibration is good */
	INS_STATUS_KINEMATIC_CAL_GOOD	            = (int)0x00008000,

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

	/** GPS compassing antenna offsets are not set in flashCfg. */
    INS_STATUS_RTK_COMPASSING_BASELINE_UNSET    = (int)0x00100000,
    /** GPS antenna baseline specified in flashCfg and measured by GPS do not match. */
    INS_STATUS_RTK_COMPASSING_BASELINE_BAD      = (int)0x00200000,
    INS_STATUS_RTK_COMPASSING_MASK              = (INS_STATUS_RTK_COMPASSING_BASELINE_UNSET|INS_STATUS_RTK_COMPASSING_BASELINE_BAD),
    
	/** Magnetometer is being recalibrated.  Device requires rotation to complete the calibration process. HDW_STATUS_MAG_RECAL_COMPLETE is set when complete. */
	INS_STATUS_MAG_RECALIBRATING				= (int)0x00400000,
	/** Magnetometer is experiencing interference or calibration is bad.  Attention may be required to remove interference (move the device) or recalibrate the magnetometer. */
	INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL		= (int)0x00800000,

    /** GPS navigation fix type (see eGpsNavFixStatus) */
    INS_STATUS_GPS_NAV_FIX_MASK					= (int)0x03000000,
    INS_STATUS_GPS_NAV_FIX_OFFSET				= 24,
#define INS_STATUS_NAV_FIX_STATUS(insStatus)    ((insStatus&INS_STATUS_GPS_NAV_FIX_MASK)>>INS_STATUS_GPS_NAV_FIX_OFFSET)

	/** RTK compassing heading is accurate.  (RTK fix and hold status) */
	INS_STATUS_RTK_COMPASSING_VALID				= (int)0x04000000,

	/* NOTE: If you add or modify these INS_STATUS_RTK_ values, please update eInsStatusRtkBase in IS-src/python/src/ci_hdw/data_sets.py */
	/** RTK error: Observations invalid or not received  (i.e. RTK differential corrections) */
    INS_STATUS_RTK_RAW_GPS_DATA_ERROR           = (int)0x08000000,
    /** RTK error: Either base observations or antenna position have not been received */
    INS_STATUS_RTK_ERR_BASE_DATA_MISSING        = (int)0x10000000,
    /** RTK error: base position moved when it should be stationary */
    INS_STATUS_RTK_ERR_BASE_POSITION_MOVING     = (int)0x20000000,
    /** RTK error: base position invalid or not surveyed */
    INS_STATUS_RTK_ERR_BASE_POSITION_INVALID    = (int)0x30000000,
    /** RTK error: NO base position received */
    INS_STATUS_RTK_ERR_BASE_MASK                = (int)0x30000000,
	/** GPS base mask */
	INS_STATUS_RTK_ERROR_MASK					= (INS_STATUS_RTK_RAW_GPS_DATA_ERROR|INS_STATUS_RTK_ERR_BASE_MASK),
	
	/** RTOS task ran longer than allotted period */
	INS_STATUS_RTOS_TASK_PERIOD_OVERRUN			= (int)0x40000000,
	/** General fault (eGenFaultCodes) */
	INS_STATUS_GENERAL_FAULT					= (int)0x80000000,
};

/** GNSS navigation fix type */
/* NOTE: If you modify this enum, please also modify the eGpsNavFixStatus enum
 *       in imx/python/src/ci_hdw/data_sets.py */
enum eGpsNavFixStatus
{
	GPS_NAV_FIX_NONE							= (int)0x00000000,
	GPS_NAV_FIX_POSITIONING_3D					= (int)0x00000001,
	GPS_NAV_FIX_POSITIONING_RTK_FLOAT			= (int)0x00000002,
	GPS_NAV_FIX_POSITIONING_RTK_FIX				= (int)0x00000003,		// Includes fix & hold
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
	/** Reference IMU data being received */
	HDW_STATUS_REFERENCE_IMU_RX	                = (int)0x00000080,

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

	/** System Reset is Required for proper function */
	HDW_STATUS_SYSTEM_RESET_REQUIRED			= (int)0x00001000,
	/** Reference IMU used in EKF */
	HDW_STATUS_EKF_USING_REFERENCE_IMU		    = (int)0x00002000,
	/** Magnetometer recalibration has finished (when INS_STATUS_MAG_RECALIBRATING is unset).  */
	HDW_STATUS_MAG_RECAL_COMPLETE	            = (int)0x00004000,
	/** System flash write staging or occuring now.  Processor will pause and not respond during a flash write, typicaly 150-250 ms. */
	HDW_STATUS_FLASH_WRITE_PENDING              = (int)0x00008000,

	/** Communications Tx buffer limited */
	HDW_STATUS_ERR_COM_TX_LIMITED				= (int)0x00010000,
	/** Communications Rx buffer overrun */
	HDW_STATUS_ERR_COM_RX_OVERRUN				= (int)0x00020000,

	/** GPS PPS timepulse signal has not been received or is in error */
	HDW_STATUS_ERR_NO_GPS_PPS					= (int)0x00040000,
	/** Time synchronized by GPS PPS */
	HDW_STATUS_GPS_PPS_TIMESYNC					= (int)0x00080000,

	/** Communications parse error count */
	HDW_STATUS_COM_PARSE_ERR_COUNT_MASK			= (int)0x00F00000,
	HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET		= 20,
#define HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus) ((hdwStatus&HDW_STATUS_COM_PARSE_ERR_COUNT_MASK)>>HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET)

	/** (BIT) Built-in self-test running */
	HDW_STATUS_BIT_RUNNING						= (int)0x01000000,
	/** (BIT) Built-in self-test passed */
	HDW_STATUS_BIT_PASSED						= (int)0x02000000,
	/** (BIT) Built-in self-test failure */
	HDW_STATUS_BIT_FAULT						= (int)0x03000000,
	/** (BIT) Built-in self-test mask */
	HDW_STATUS_BIT_MASK							= (int)0x03000000,

	/** Temperature outside spec'd operating range */
	HDW_STATUS_ERR_TEMPERATURE					= (int)0x04000000,
	
	/** IMX pins G5-G8 are configure for SPI use */
	HDW_STATUS_SPI_INTERFACE_ENABLED			= (int)0x08000000,

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

	/** Critical System Fault - CPU error */
	HDW_STATUS_FAULT_SYS_CRITICAL				= (int)0x80000000,
};

// Used to validate GPS position (and velocity)
#define GPS_THRESH_SATS_USED			5
#define GPS_THRESH_P_DOP				3.0f
#define GPS_THRESH_H_ACC				10.0f
#define GPS_THRESH_V_ACC				20.0f
#define GPS_THRESH_S_ACC				2.0f

/** GPS Status */
enum eGpsStatus
{
	GPS_STATUS_NUM_SATS_USED_MASK                   = (int)0x000000FF,

	/** Fix */
	GPS_STATUS_FIX_NONE                             = (int)0x00000000,
	GPS_STATUS_FIX_DEAD_RECKONING_ONLY              = (int)0x00000100,
	GPS_STATUS_FIX_2D                               = (int)0x00000200,
	GPS_STATUS_FIX_3D                               = (int)0x00000300,
	GPS_STATUS_FIX_GPS_PLUS_DEAD_RECK               = (int)0x00000400,
	GPS_STATUS_FIX_TIME_ONLY                        = (int)0x00000500,
	GPS_STATUS_FIX_UNUSED1                          = (int)0x00000600,
	GPS_STATUS_FIX_UNUSED2                          = (int)0x00000700,
	GPS_STATUS_FIX_DGPS                             = (int)0x00000800,
	GPS_STATUS_FIX_SBAS                             = (int)0x00000900,
	GPS_STATUS_FIX_RTK_SINGLE                       = (int)0x00000A00,
	GPS_STATUS_FIX_RTK_FLOAT                        = (int)0x00000B00,
	GPS_STATUS_FIX_RTK_FIX                          = (int)0x00000C00,
	GPS_STATUS_FIX_MASK                             = (int)0x00001F00,
	GPS_STATUS_FIX_BIT_OFFSET                       = (int)8,

	/** Flags  */
	GPS_STATUS_FLAGS_FIX_OK                         = (int)0x00010000,      // within limits (e.g. DOP & accuracy)
	GPS_STATUS_FLAGS_DGPS_USED                      = (int)0x00020000,      // Differential GPS (DGPS) used.
 	GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD               = (int)0x00040000,      // RTK feedback on the integer solutions to drive the float biases towards the resolved integers
// 	GPS_STATUS_FLAGS_WEEK_VALID                     = (int)0x00040000,
// 	GPS_STATUS_FLAGS_TOW_VALID                      = (int)0x00080000,
	GPS_STATUS_FLAGS_RTK_POSITION_ENABLED           = (int)0x00100000,      // RTK precision positioning mode enabled
	GPS_STATUS_FLAGS_STATIC_MODE                    = (int)0x00200000,      // Static mode
	GPS_STATUS_FLAGS_RTK_COMPASSING_ENABLED         = (int)0x00400000,      // RTK moving base mode enabled
    GPS_STATUS_FLAGS_RTK_RAW_GPS_DATA_ERROR         = (int)0x00800000,      // RTK error: observations or ephemeris are invalid or not received (i.e. RTK differential corrections)
    GPS_STATUS_FLAGS_RTK_BASE_DATA_MISSING          = (int)0x01000000,      // RTK error: Either base observations or antenna position have not been received.
    GPS_STATUS_FLAGS_RTK_BASE_POSITION_MOVING       = (int)0x02000000,      // RTK error: base position moved when it should be stationary
    GPS_STATUS_FLAGS_RTK_BASE_POSITION_INVALID      = (int)0x03000000,      // RTK error: base position is invalid or not surveyed well
    GPS_STATUS_FLAGS_RTK_BASE_POSITION_MASK         = (int)0x03000000,      // RTK error: base position error bitmask
    GPS_STATUS_FLAGS_ERROR_MASK                     = (GPS_STATUS_FLAGS_RTK_RAW_GPS_DATA_ERROR|
                                                    GPS_STATUS_FLAGS_RTK_BASE_POSITION_MASK),
	GPS_STATUS_FLAGS_RTK_POSITION_VALID             = (int)0x04000000,      // GPS1 RTK precision position and carrier phase range solution with fixed ambiguities (i.e. < 6cm horizontal accuracy).  The carrier phase range solution with floating ambiguities occurs if GPS_STATUS_FIX_RTK_FIX is set and GPS_STATUS_FLAGS_RTK_POSITION_VALID is not set (i.e. > 6cm horizontal accuracy).
	GPS_STATUS_FLAGS_RTK_COMPASSING_VALID           = (int)0x08000000,      // GPS2 RTK moving base heading.  Indicates RTK fix and hold with single band RTK compassing.
    GPS_STATUS_FLAGS_RTK_COMPASSING_BASELINE_BAD    = (int)0x00002000,
    GPS_STATUS_FLAGS_RTK_COMPASSING_BASELINE_UNSET  = (int)0x00004000,
    GPS_STATUS_FLAGS_RTK_COMPASSING_MASK            = (GPS_STATUS_FLAGS_RTK_COMPASSING_ENABLED|
                                                    GPS_STATUS_FLAGS_RTK_COMPASSING_VALID|
                                                    GPS_STATUS_FLAGS_RTK_COMPASSING_BASELINE_BAD|
                                                    GPS_STATUS_FLAGS_RTK_COMPASSING_BASELINE_UNSET),
	GPS_STATUS_FLAGS_GPS_NMEA_DATA                  = (int)0x00008000,      // 1 = Data from NMEA message
	GPS_STATUS_FLAGS_GPS_PPS_TIMESYNC               = (int)0x10000000,      // Time is synchronized by GPS PPS. 

	GPS_STATUS_FLAGS_MASK                           = (int)0xFFFFE000,    
	GPS_STATUS_FLAGS_BIT_OFFSET                     = (int)16,
	
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
	/** Inertial Sense serial number */
	uint32_t		serialNumber;

	/** Inertial Sense lot number */
	uint32_t		lotNumber;

	/** Inertial Sense manufacturing date (YYYYMMDDHHMMSS) */
    char			date[16];

	/** Key */
	uint32_t		key;

	/** Microcontroller unique identifier, 128 bits for SAM / 96 for STM32 */
	uint32_t 		uid[4];
} manufacturing_info_t;

/** (DID_INS_1) INS output: euler rotation w/ respect to NED, NED position from reference LLA */
typedef struct PACKED
{
	/** GPS number of weeks since January 6th, 1980 */
	uint32_t				week;
	
	/** GPS time of week (since Sunday morning) in seconds */
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

	/** North, east and down (meters) offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
	float					ned[3];
} ins_1_t;


/** (DID_INS_2) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
typedef struct PACKED
{
	/** GPS number of weeks since January 6th, 1980 */
	uint32_t				week;
	
	/** GPS time of week (since Sunday morning) in seconds */
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
	/** GPS number of weeks since January 6th, 1980 */
	uint32_t				week;
	
	/** GPS time of week (since Sunday morning) in seconds */
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

	/** height above mean sea level (MSL) in meters */
	float					msl;
} ins_3_t;


/** (DID_INS_4) INS output: quaternion rotation w/ respect to ECEF, ECEF position */
typedef struct PACKED
{
	/** GPS number of weeks since January 6th, 1980 */
	uint32_t				week;
	
	/** GPS time of week (since Sunday morning) in seconds */
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


/** (DID_IMU, DID_REFERENCE_IMU) Inertial Measurement Unit (IMU) data */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/** IMU Status (eImuStatus) */
	uint32_t                status;

	/** Inertial Measurement Unit (IMU) */
	imus_t					I;
} imu_t;


/** (DID_IMU3_UNCAL) Dual Inertial Measurement Units (IMUs) data */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/** IMU Status (eImuStatus) */
	uint32_t                status;

	/** Inertial Measurement Units (IMUs) */
	imus_t                  I[3];

} imu3_t;


/** (DID_MAGNETOMETER) Magnetometer sensor data */
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


/** (DID_PIMU, DID_REFERENCE_PIMU) Preintegraed IMU (a.k.a. Coning and Sculling integral) in body/IMU frame. */
typedef struct PACKED
{
	/** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/** Integral period in seconds for delta theta and delta velocity.  This is configured using DID_FLASH_CONFIG.startupNavDtMs. */
	float					dt;

	/** IMU Status (eImuStatus) */
	uint32_t                status;

	/** IMU delta theta (gyroscope {p,q,r} integral) in radians in sensor frame */
	float                   theta[3];

	/** IMU delta velocity (accelerometer {x,y,z} integral) in m/s in sensor frame */
	float                   vel[3];

} pimu_t;


/** (DID_IMU_MAG) imu + mag */
typedef struct PACKED
{
	/** imu - raw or pre-integrated depending on data id */
	imu_t imu;
	
	/** mag */
	magnetometer_t mag;
} imu_mag_t;


/** (DID_PIMU_MAG) preintegrated imu + mag */
typedef struct PACKED
{
	/** Preintegrated IMU */
	pimu_t pimu;
	
	/** Magnetometer */
	magnetometer_t mag;
} pimu_mag_t;


/** IMU Status */
enum eImuStatus
{
	/** Sensor saturation on IMU1 gyro */
	IMU_STATUS_SATURATION_IMU1_GYR              = (int)0x00000001,
	/** Sensor saturation on IMU2 gyro */
	IMU_STATUS_SATURATION_IMU2_GYR              = (int)0x00000002,
	/** Sensor saturation on IMU3 gyro */
	IMU_STATUS_SATURATION_IMU3_GYR              = (int)0x00000004,
	/** Sensor saturation on IMU1 accelerometer */
	IMU_STATUS_SATURATION_IMU1_ACC              = (int)0x00000008,
	/** Sensor saturation on IMU2 accelerometer */
	IMU_STATUS_SATURATION_IMU2_ACC              = (int)0x00000010,
	/** Sensor saturation on IMU3 accelerometer */
	IMU_STATUS_SATURATION_IMU3_ACC              = (int)0x00000020,
	/** Sensor saturation mask */
	IMU_STATUS_SATURATION_MASK                  = (int)0x0000003F,

	/** Magnetometer sample occured */
	IMU_STATUS_MAG_UPDATE						= (int)0x00000100,

	/** IMU1 gyros available */
	IMU_STATUS_GYR1_OK                          = (int)0x00010000,
	/** IMU2 gyros and accelerometers available */
	IMU_STATUS_GYR2_OK                          = (int)0x00020000,
	/** IMU3 gyros available */
	IMU_STATUS_GYR3_OK                          = (int)0x00040000,
	/** IMU1 accelerometers available */
	IMU_STATUS_ACC1_OK                          = (int)0x00080000,
	/** IMU2 accelerometers available */
	IMU_STATUS_ACC2_OK                          = (int)0x00100000,
	/** IMU3 accelerometers available */
	IMU_STATUS_ACC3_OK                          = (int)0x00200000,
	/** IMU1 available */
	IMU_STATUS_IMU1_OK                          = (int)(IMU_STATUS_GYR1_OK | IMU_STATUS_ACC1_OK),
	/** IMU2 available */
	IMU_STATUS_IMU2_OK                          = (int)(IMU_STATUS_GYR2_OK | IMU_STATUS_ACC2_OK),
	/** IMU3 available */
	IMU_STATUS_IMU3_OK                          = (int)(IMU_STATUS_GYR3_OK | IMU_STATUS_ACC3_OK),
	/** IMU gyros and accelerometers available */
	IMU_STATUS_IMU_OK_MASK                      = (int)0x003F0000,
};

/** (DID_GNSS1_POS, DID_GNSS1_UBX_POS, DID_GNSS2_POS) GNSS position data */
typedef struct PACKED
{
	/** GPS number of weeks since January 6th, 1980 */
	uint32_t                week;

	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;

	/** (see eGpsStatus) GNSS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag */
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

	/** Position dilution of precision (unitless) */
	float                   pDop;

	/** Average of all non-zero satellite carrier to noise ratios (signal strengths) in dBHz */
	float                   cnoMean;

	/** Time sync offset between local time since boot up to GPS time of week in seconds.  Add this to IMU and sensor time to get GPS time of week in seconds. */
	double                  towOffset;
	
	/** GPS leap second (GPS-UTC) offset. Receiver's best knowledge of the leap seconds offset from UTC to GPS time. Subtract from GPS time of week to get UTC time of week. (18 seconds as of December 31, 2016) */
	uint8_t					leapS;

	/** Number of satellites used */
	uint8_t					satsUsed;

	/** Standard deviation of cnoMean over past 5 seconds (dBHz x10) */
	uint8_t					cnoMeanSigma;

	/** Reserved for future use */
	uint8_t					reserved;
} gnss_pos_t;


/** (DID_GNSS1_VEL, DID_GNSS2_VEL) GNSS velocity data */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

	/** GPS Velocity.  Velocity is in ECEF {vx,vy,vz} (m/s) if status bit GPS_STATUS_FLAGS_GPS_NMEA_DATA (0x00008000) is NOT set.  Velocity is in local tangent plane with no vertical velocity {vNorth, vEast, 0} (m/s) if status bit GPS_STATUS_FLAGS_GPS_NMEA_DATA (0x00008000) is set. */
	float					vel[3];	

	/** Speed accuracy in meters / second */
	float					sAcc;
	
	/** (see eGpsStatus) GNSS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag */
	uint32_t                status;
} gnss_vel_t;


/** GNSS Satellite information */
typedef struct PACKED
{
	/** GNSS constellation ID (see eSatSysId) */
	uint8_t					gnssId;			

	/** Satellite identifier */
	uint8_t					svId;			

	/** (dBHz) Carrier to noise ratio (signal strength) */
	uint8_t					cno[MAX_NUM_RTK_FREQ];			

	/** (deg) Elevation (range: +/-90) */
	int8_t					elev;			

	/** (deg) Azimuth (range: +/-180) */
	int16_t					azim;		

	/** (see eSatSvFlags) */
	uint32_t				flags;			
} gnss_sat_sv_t;

/** GNSS constellation IDs */
enum eSatSysId
{
	SAT_SYS_GPS = 0,	// GPS (G)
	SAT_SYS_GAL,		// Galileo (E)
	SAT_SYS_GLO,		// GLONASS (R)
	SAT_SYS_IRNSS,		// IRNSS (I)
	SAT_SYS_QZSS,		// QZSS (J)
	SAT_SYS_BDS,		// BeiDou (B)
	SAT_SYS_SBAS,		// SBAS (S)
};

/** GNSS Status */
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
	
	SAT_SV_FLAGS_RTK_SOL_FIX_STATUS_MASK	= 0x03000000,	// 1=float, 2=fix, 3=hold
	SAT_SV_FLAGS_RTK_SOL_FIX_STATUS_OFFSET	= 24,
	SAT_SV_FLAGS_RTK_SOL_FIX_STATUS_FLOAT	= 1,	
	SAT_SV_FLAGS_RTK_SOL_FIX_STATUS_FIX		= 2,	
	SAT_SV_FLAGS_RTK_SOL_FIX_STATUS_HOLD	= 3,	
};

/** (DID_GNSS1_SAT, DID_GNSS1_SAT) GNSS satellite signal strength */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;				
    /** Number of satellites in the sky */
	uint32_t				numSats;					
    /** Satellite information list */
	gnss_sat_sv_t			sat[MAX_NUM_SAT_CHANNELS];	
} gnss_sat_t;


/** (DID_GNSS1_VERSION, DID_GNSS1_VERSION) GPS version strings */
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
} gnss_ver_t;

// (DID_INL2_STATES) INL2 - INS Extended Kalman Filter (EKF) states
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in seconds */
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

// (DID_ROS_COVARIANCE_POSE_TWIST) INL2 - INS Extended Kalman Filter (EKF) state covariance
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in seconds */
	double                  timeOfWeek;

    /** (rad^2, m^2)  EKF attitude and position error covariance matrix lower diagonal in body (attitude) and ECEF (position) frames */
	float					covPoseLD[21];

    /** ((m/s)^2, (rad/s)^2)   EKF velocity and angular rate error covariance matrix lower diagonal in ECEF (velocity) and body (attitude) frames */
	float					covTwistLD[21];

} ros_covariance_pose_twist_t;

// (DID_INL2_STATUS)
typedef struct PACKED
{
	int						ahrs;
	int						zero_accel;
	int						zero_angrate;
	int						accel_motion;
	int						rot_motion;
	int						zero_vel;
	int						ahrs_gps_cnt;			// Counter of sequential valid GPS data (for switching from AHRS to navigation)
	float					att_err;
	int						att_coarse;				// Flag whether initial attitude error converged
	int						att_aligned;			// Flag whether initial attitude error converged
	int						att_aligning;
	int						start_proc_done;		// Cold/hot start procedure completed
	int						mag_cal_good;
	int						mag_cal_done;
	int						stat_magfield;
} inl2_status_t;

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
	/** GPS time of week (since Sunday morning) in milliseconds */
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
	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;

	/** INS status flags (eInsStatusFlags) */
	uint32_t                insStatus;

	/** Hardware status flags (eHdwStatusFlags) */
	uint32_t                hdwStatus;

	/** IMU temperature */
	float					imuTemp;

	/** Baro temperature */
	float					baroTemp;

	/** MCU temperature (not available yet) */
	float					mcuTemp;

	/** System status flags (eSysStatusFlags) */
	uint32_t				sysStatus;

	/** IMU sample period in milliseconds. Zero disables sampling. */
	uint32_t				imuPeriodMs;

	/** Preintegrated IMU (PIMU) integration period and navigation filter update period (ms). */
	uint32_t				navPeriodMs;
	
	/** Actual sample period relative to GPS PPS */
	double					sensorTruePeriod;

	/** Reserved */
	float					reserved2;
	/** Reserved */
	float					reserved3;

	/** General fault code descriptor (eGenFaultCodes).  Set to zero to reset fault code. */
	uint32_t                genFaultCode;
} sys_params_t;

/*! General Fault Code descriptor */
enum eGenFaultCodes
{
	/*! INS state limit overrun - UVW */
	GFC_INS_STATE_ORUN_UVW				= 0x00000001,
	/*! INS state limit overrun - Latitude */
	GFC_INS_STATE_ORUN_LAT				= 0x00000002,
	/*! INS state limit overrun - Altitude */
	GFC_INS_STATE_ORUN_ALT				= 0x00000004,
	/*! Unhandled interrupt */
	GFC_UNHANDLED_INTERRUPT				= 0x00000010,
	/*! Fault: sensor initialization  */
	GFC_INIT_SENSORS					= 0x00000100,
	/*! Fault: SPI bus initialization  */
	GFC_INIT_SPI						= 0x00000200,
	/*! Fault: SPI configuration  */
	GFC_CONFIG_SPI						= 0x00000400,
	/*! Fault: GPS1 init  */
	GFC_INIT_GNSS1						= 0x00000800,
	/*! Fault: GPS2 init  */
	GFC_INIT_GNSS2                      = 0x00001000,
	/*! Flash failed to load valid values */
	GFC_FLASH_INVALID_VALUES			= 0x00002000,
	/*! Flash checksum failure */
	GFC_FLASH_CHECKSUM_FAILURE			= 0x00004000,
	/*! Flash write failure */
	GFC_FLASH_WRITE_FAILURE				= 0x00008000,
	/*! System Fault: general */
	GFC_SYS_FAULT_GENERAL				= 0x00010000,
	/*! System Fault: CRITICAL system fault (see DID_SYS_FAULT) */
	GFC_SYS_FAULT_CRITICAL			    = 0x00020000,
	/*! Sensor(s) saturated */
	GFC_SENSOR_SATURATION 				= 0x00040000,
	/*! Fault: IMU initialization */
	GFC_INIT_IMU						= 0x00100000,
	/*! Fault: Magnetometer initialization */
	GFC_INIT_MAGNETOMETER				= 0x00400000,
	/*! Fault: Barometer initialization */
	GFC_INIT_BAROMETER					= 0x00200000,
	/*! Fault: I2C initialization */
	GFC_INIT_I2C						= 0x00800000,
	/*! Fault: Chip erase line toggled but did not meet required hold time.  This is caused by noise/transient on chip erase pin.  */
	GFC_CHIP_ERASE_INVALID				= 0x01000000,
};


/** (DID_SYS_CMD) System Commands */
typedef struct PACKED
{
	/** System commands (see eSystemCommand) 1=save current persistent messages, 5=zero motion, 97=save flash, 99=software reset.  "invCommand" (following variable) must be set to bitwise inverse of this value for this command to be processed.  */
	uint32_t                command;

    /** Error checking field that must be set to bitwise inverse of command field for the command to take effect.  */
    uint32_t                invCommand;

} system_command_t;

enum eSystemCommand 
{
    SYS_CMD_SAVE_PERSISTENT_MESSAGES                = 1,
    SYS_CMD_ENABLE_BOOTLOADER_AND_RESET             = 2,
    SYS_CMD_ENABLE_SENSOR_STATS                     = 3,
    SYS_CMD_ENABLE_RTOS_STATS                       = 4,
    SYS_CMD_ZERO_MOTION                             = 5,

    SYS_CMD_ENABLE_GPS_LOW_LEVEL_CONFIG             = 10,
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GNSS1  = 11,
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GNSS2  = 12,
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER0   = 13,
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER1   = 14,
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER2   = 15,

    SYS_CMD_SAVE_FLASH                              = 97,
    SYS_CMD_SAVE_GPS_ASSIST_TO_FLASH_RESET          = 98,
    SYS_CMD_SOFTWARE_RESET                          = 99,
    SYS_CMD_MANF_UNLOCK                             = 1122334455,
    SYS_CMD_MANF_FACTORY_RESET                      = 1357924680,	// SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.
    SYS_CMD_MANF_CHIP_ERASE                         = 1357924681,	// SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.
    SYS_CMD_MANF_DOWNGRADE_CALIBRATION              = 1357924682,	// SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.
};

enum eSerialPortBridge
{
	SERIAL_PORT_BRIDGE_DISABLED         = 0,
	SERIAL_PORT_BRIDGE_USB_TO_GNSS1     = 1,
	SERIAL_PORT_BRIDGE_USB_TO_GNSS2     = 2,
	SERIAL_PORT_BRIDGE_USB_TO_SER0      = 3,
	SERIAL_PORT_BRIDGE_USB_TO_SER1      = 4,
	SERIAL_PORT_BRIDGE_USB_TO_SER2      = 5,
};

/** (DID_ASCII_BCAST_PERIOD) ASCII broadcast periods. This data structure is zeroed out on stop_all_broadcasts */
typedef struct PACKED
{
	/** Options: Port selection[0x0=current, 0xFF=all, 0x1=ser0, 0x2=ser1, 0x4=ser2, 0x8=USB] (see RMC_OPTIONS_...) */
	uint32_t				options;

	/** Broadcast period (ms) - ASCII dual IMU data. 0 to disable. */
	uint16_t				pimu;

	/** Broadcast period (ms) - ASCII preintegrated dual IMU: delta theta (rad) and delta velocity (m/s). 0 to disable. */
	uint16_t				ppimu;
	
	/** Broadcast period (ms) - ASCII INS output: euler rotation w/ respect to NED, NED position from reference LLA. 0 to disable. */
	uint16_t				pins1;

	/** Broadcast period (ms) - ASCII INS output: quaternion rotation w/ respect to NED, ellipsoid altitude. 0 to disable. */
	uint16_t				pins2;
	
	/** Broadcast period (ms) - ASCII GPS position data. 0 to disable. */
	uint16_t				pgpsp;

	/** Broadcast period (ms) - Reserved.  Leave zero. */
	uint16_t				reserved;

	/** Broadcast period (ms) - ASCII NMEA GPGGA GPS 3D location, fix, and accuracy. 0 to disable. */
	uint16_t				gpgga;

	/** Broadcast period (ms) - ASCII NMEA GPGLL GPS 2D location and time. 0 to disable. */
	uint16_t				gpgll;

	/** Broadcast period (ms) - ASCII NMEA GSA GPS DOP and active satellites. 0 to disable. */
	uint16_t				gpgsa;

	/** Broadcast period (ms) - ASCII NMEA recommended minimum specific GPS/Transit data. 0 to disable. */
	uint16_t				gprmc;
	
	/** Broadcast period (ms) - ASCII NMEA Data and Time. 0 to disable. */
	uint16_t				gpzda;

	/** Broadcast period (ms) - ASCII NMEA Inertial Attitude Data. 0 to disable. */
	uint16_t				pashr;
	
} ascii_msgs_t;

/** (DID_ASCII_BCAST_PERIOD) ASCII broadcast periods. This data structure is zeroed out on stop_all_broadcasts */
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
	
	/** Broadcast period (ms) - ASCII NMEA Data and Time. 0 to disable. */
	uint32_t				gpzda;

	/** Broadcast period (ms) - ASCII NMEA Inertial Attitude Data. 0 to disable. */
	uint32_t				pashr;
	
} ascii_msgs_u32_t;

typedef struct PACKED
{
	/** (rad/s) Gyros.  Units only apply for calibrated data. */
	f_t						pqr[3];

	/** (m/s^2) Accelerometers.  Units only apply for calibrated data. */
	f_t						acc[3];

	/** (°C) Temperature of IMU.  Units only apply for calibrated data. */
	f_t						temp;
} sensors_imu_w_temp_t;

typedef struct PACKED
{                                       // Units only apply for calibrated data
	f_t						mag[3];         // (uT)		Magnetometers
} sensors_mag_t;

typedef struct PACKED
{
	/** (rad/s) Gyros.  Units only apply for calibrated data. */
	f_t						pqr[3];

	/** (m/s^2) Accelerometers.  Units only apply for calibrated data. */
	f_t						acc[3];

	/** (uT) Magnetometers.  Units only apply for calibrated data. */
	f_t						mag[3];
} sensors_mpu_t;

// (DID_SENSORS_TC_BIAS)
typedef struct PACKED
{
    /** Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;                                       // Units only apply for calibrated data

	sensors_mpu_t			mpu[NUM_IMU_DEVICES];
} sensors_t;

typedef struct PACKED
{
	f_t						xyz[3];
} mag_xyz_t;

// (DID_SENSORS_UCAL, DID_SENSORS_TCAL, DID_SENSORS_MCAL)
typedef struct PACKED
{
	imu3_t					imu3;

	/** (°C) Temperature of IMU.  Units only apply for calibrated data. */
	f_t						temp[NUM_IMU_DEVICES];

	/** (uT) Magnetometers.  Units only apply for calibrated data. */
	mag_xyz_t				mag[NUM_MAG_DEVICES];
} sensors_w_temp_t;

typedef struct PACKED
{
	f_t						lpfLsb[3];      // Low-pass filtered of g_sensors.lsb
	f_t						lpfTemp;		// (°C) Low-pass filtered sensor temperature
	f_t						k[3];			// Slope (moved from flash to here)
	f_t						temp;			// (°C)	Temperature of sensor
	f_t                     tempRampRate;   // (°C/s) Temperature ramp rate
	uint32_t                tci;            // Index of current temperature compensation point
	uint32_t                numTcPts;       // Total number of tc points
	f_t                     dtTemp;			// (°C) Temperature from last calibration point
} sensor_comp_unit_t;

typedef struct PACKED
{                                       // Sensor temperature compensation
	uint32_t                timeMs;         // (ms) Time since boot up.
	sensor_comp_unit_t		pqr[NUM_IMU_DEVICES];
	sensor_comp_unit_t		acc[NUM_IMU_DEVICES];
	sensor_comp_unit_t		mag[NUM_MAG_DEVICES];
	imus_t 					referenceImu;	// External reference IMU
	float                   referenceMag[3];// External reference magnetometer (heading reference)
	uint32_t                sampleCount;    // Number of samples collected
	uint32_t                calState;       // state machine (see eScompCalState)
	uint32_t				status;         // Status used to control LED and indicate valid sensor samples (see eScompStatus)
	f_t						alignAccel[3];  // Alignment acceleration
} sensor_compensation_t;

#define NUM_ANA_CHANNELS	4
typedef struct PACKED
{                                       // LSB units for all except temperature, which is Celsius.
	double					time;
	sensors_imu_w_temp_t	imu[NUM_IMU_DEVICES];
	sensors_mag_t			mag[NUM_MAG_DEVICES];   // Magnetometers
	f_t						bar;            		// Barometric pressure
	f_t						barTemp;				// Temperature of barometric pressure sensor
	f_t                     humidity;				// Relative humidity as a percent (%rH).  Range is 0% - 100%
	f_t						ana[NUM_ANA_CHANNELS]; // ADC analog input
} sys_sensors_adc_t;

#define NUM_COM_PORTS       4	// Number of communication ports.  (Ser0, Ser1, Ser2, and USB).
#ifndef NUM_SERIAL_PORTS
#define NUM_SERIAL_PORTS	6
#endif

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
	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;

	/** General purpose I/O status */
	uint32_t				gpioStatus;
} io_t;

enum eMagCalState
{
	MAG_CAL_STATE_DO_NOTHING		= (int)0, 

	/** COMMAND: Recalibrate magnetometers using multiple axis */
	MAG_CAL_STATE_MULTI_AXIS		= (int)1,

	/** COMMAND: Recalibrate magnetometers using only one axis */
	MAG_CAL_STATE_SINGLE_AXIS		= (int)2,

	/** COMMAND: Stop mag recalibration and do not save results */
	MAG_CAL_STATE_ABORT				= (int)101,

	/** STATUS: Mag recalibration is in progress */
	MAG_CAL_STATE_RECAL_RUNNING		= (int)200,

	/** STATUS: Mag recalibration has completed */
	MAG_CAL_STATE_RECAL_COMPLETE	= (int)201,
};

/** (DID_MAG_CAL) Magnetometer Calibration */
typedef struct PACKED
{
	/** Mag recalibration state (see eMagCalState) */
	uint32_t                state;
	
	/** Mag recalibration progress indicator: 0-100 % */
	float					progress;

	/** Magnetic declination estimate */
	float					declination;
} mag_cal_t;

// (DID_INL2_MAG_OBS_INFO)
typedef struct PACKED
{											// INL2 - Magnetometer observer info 
	/** Timestamp in milliseconds */
	uint32_t				timeOfWeekMs;	

	/** Number of calibration samples */
	uint32_t				Ncal_samples;

	/** Data ready to be processed */
	uint32_t				ready;

	/** Calibration data present.  Set to -1 to force mag recalibration. */	
	uint32_t				calibrated;

	/** Allow mag to auto-recalibrate */
	uint32_t				auto_recal;

	/** Bad sample data */		
	uint32_t				outlier;

	/** Heading from magnetometer */
	float					magHdg;

	/** Heading from INS */			
	float					insHdg;

	/** Difference between mag heading and (INS heading plus mag declination) */
	float					magInsHdgDelta;

	/** Normalized innovation squared (likelihood metric) */
	float					nis;

	/** Threshold for maximum NIS */
	float					nis_threshold;

	/** Magnetometer calibration matrix. Must be initialized with a unit matrix, not zeros! */
	float					Wcal[9];

	/** Active calibration set (0 or 1) */
	uint32_t				activeCalSet;

	/** Offset between magnetometer heading and estimate heading */
	float					magHdgOffset;

	/** Scaled computed variance between calibrated magnetometer samples.  */
    float                   Tcal;

	/** Calibrated magnetometer output can be produced using: Bcal = Wcal * (Braw - bias_cal) */
	float                   bias_cal[3];
} inl2_mag_obs_info_t;

/** Built-in Test: State */
enum eBitState
{
	BIT_STATE_OFF					                    = (int)0,
	BIT_STATE_DONE				                        = (int)1,   // Test is finished
    BIT_STATE_CMD_FULL_STATIONARY                       = (int)2,   // (FULL) Comprehensive test.  Requires system be completely stationary without vibrations. 
    BIT_STATE_CMD_BASIC_MOVING                          = (int)3,   // (BASIC) Ignores sensor output.  Can be run while moving.  This mode is automatically run after bootup.
    BIT_STATE_CMD_FULL_STATIONARY_HIGH_ACCURACY         = (int)4,   // Same as BIT_STATE_CMD_FULL_STATIONARY but with higher requirements for accuracy.  In order to pass, this test may require the Infield Calibration (DID_INFIELD_CAL) to be run. 
    BIT_STATE_RESERVED_2                                = (int)5,   
    BIT_STATE_RUNNING                                   = (int)6,   
    BIT_STATE_FINISHING                                 = (int)7,	// Computing results
    BIT_STATE_CMD_OFF                                   = (int)8,   // Stop built-in test
};

/** Built-in Test: Test Mode */
enum eBitTestMode
{
    BIT_TEST_MODE_SIM_GPS_NOISE                         = (int)100, // Simulate CNO noise
};

/** Hardware built-in test (BIT) flags */
enum eHdwBitStatusFlags
{
    HDW_BIT_PASSED_MASK             = (int)0x0000000F,
    HDW_BIT_PASSED_ALL              = (int)0x00000001,
    HDW_BIT_PASSED_NO_GPS           = (int)0x00000002,    // Passed w/o valid GPS signal
    HDW_BIT_MODE_MASK               = (int)0x000000F0,    // BIT mode run
    HDW_BIT_MODE_OFFSET             = (int)4,
#define HDW_BIT_MODE(hdwBitStatus) ((hdwBitStatus&HDW_BIT_MODE_MASK)>>HDW_BIT_MODE_OFFSET)
    HDW_BIT_FAILED_MASK             = (int)0xFFFFFF00,
    HDW_BIT_FAILED_AHRS_MASK        = (int)0xFFFF0F00,
    HDW_BIT_FAULT_NOISE_PQR         = (int)0x00000100,
    HDW_BIT_FAULT_NOISE_ACC         = (int)0x00000200,
    HDW_BIT_FAULT_MAGNETOMETER      = (int)0x00000400,
    HDW_BIT_FAULT_BAROMETER         = (int)0x00000800,
    HDW_BIT_FAULT_GPS_NO_COM        = (int)0x00001000,    // No GPS serial communications
    HDW_BIT_FAULT_GPS_POOR_CNO      = (int)0x00002000,    // Poor GPS signal strength.  Check antenna
    HDW_BIT_FAULT_GPS_POOR_ACCURACY = (int)0x00002000,    // Low number of satellites, or bad accuracy 
    HDW_BIT_FAULT_GPS_NOISE         = (int)0x00004000,    // (Not implemented)
};

/** Calibration built-in test flags */
enum eCalBitStatusFlags
{
    CAL_BIT_PASSED_MASK             = (int)0x0000000F,
    CAL_BIT_PASSED_ALL              = (int)0x00000001,
    CAL_BIT_MODE_MASK               = (int)0x000000F0,    // BIT mode run
    CAL_BIT_MODE_OFFSET             = (int)4,
#define CAL_BIT_MODE(calBitStatus) ((calBitStatus&CAL_BIT_MODE_MASK)>>CAL_BIT_MODE_OFFSET)
    CAL_BIT_FAILED_MASK             = (int)0x00FFFF00,
    CAL_BIT_FAULT_TCAL_EMPTY        = (int)0x00000100,    // Temperature calibration not present
    CAL_BIT_FAULT_TCAL_TSPAN        = (int)0x00000200,    // Temperature calibration temperature range is inadequate
    CAL_BIT_FAULT_TCAL_INCONSISTENT = (int)0x00000400,    // Temperature calibration number of points or slopes are not consistent
    CAL_BIT_FAULT_TCAL_CORRUPT      = (int)0x00000800,    // Temperature calibration memory corruption
    CAL_BIT_FAULT_TCAL_PQR_BIAS     = (int)0x00001000,    // Temperature calibration gyro bias
    CAL_BIT_FAULT_TCAL_PQR_SLOPE    = (int)0x00002000,    // Temperature calibration gyro slope
    CAL_BIT_FAULT_TCAL_PQR_LIN      = (int)0x00004000,    // Temperature calibration gyro linearity
    CAL_BIT_FAULT_TCAL_ACC_BIAS     = (int)0x00008000,    // Temperature calibration accelerometer bias
    CAL_BIT_FAULT_TCAL_ACC_SLOPE    = (int)0x00010000,    // Temperature calibration accelerometer slope
    CAL_BIT_FAULT_TCAL_ACC_LIN      = (int)0x00020000,    // Temperature calibration accelerometer linearity
    CAL_BIT_FAULT_CAL_SERIAL_NUM    = (int)0x00040000,    // Calibration info: wrong device serial number
    CAL_BIT_FAULT_MCAL_EMPTY        = (int)0x00100000,    // Motion calibration Cross-axis alignment is not calibrated
    CAL_BIT_FAULT_MCAL_INVALID      = (int)0x00200000,    // Motion calibration Cross-axis alignment is poorly formed
    CAL_BIT_FAULT_MOTION_PQR        = (int)0x00400000,    // Motion on gyros
    CAL_BIT_FAULT_MOTION_ACC        = (int)0x00800000,    // Motion on accelerometers
    CAL_BIT_NOTICE_IMU1_PQR_BIAS    = (int)0x01000000,    // IMU 1 gyro bias offset detected.  If stationary, zero gyros command may be used.
    CAL_BIT_NOTICE_IMU2_PQR_BIAS    = (int)0x02000000,    // IMU 2 gyro bias offset detected.  If stationary, zero gyros command may be used.
    CAL_BIT_NOTICE_IMU1_ACC_BIAS    = (int)0x10000000,    // IMU 1 accelerometer bias offset detected.  If stationary, zero accelerometer command may be used only on the vertical access.
    CAL_BIT_NOTICE_IMU2_ACC_BIAS    = (int)0x20000000,    // IMU 2 accelerometer bias offset detected.  If stationary, zero accelerometer command may be used only on the vertical access.
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

	/** Gyro error (rad/s) */
	float                   pqr;

	/** Accelerometer error (m/s^2) */
	float                   acc;

	/** Angular rate standard deviation */
	float                   pqrSigma;

	/** Acceleration standard deviation */
	float                   accSigma;

	/** Self-test mode (see eBitTestMode) */
	uint32_t                testMode;

} bit_t;


enum eInfieldCalState
{
    /** User Commands: */
    INFIELD_CAL_STATE_CMD_OFF                           = 0,

    /** Initialization Commands.  Select one of the following to clear prior samples and set the mode.  Zero accels requires vertical alignment.  No motion is required for all unless disabled.  */
    INFIELD_CAL_STATE_CMD_INIT_ZERO_IMU                     = 1,    // Zero accel and gyro biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_GYRO                    = 2,    // Zero only gyro  biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ACCEL                   = 3,    // Zero only accel biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE                = 4,    // Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_IMU            = 5,    // Zero gyro and accel biases.  Zero (level) INS attitude by adjusting INS rotation. 
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_GYRO           = 6,    // Zero only gyro  biases.  Zero (level) INS attitude by adjusting INS rotation. 
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_ACCEL          = 7,    // Zero only accel biases.  Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATE_CMD_INIT_OPTION_DISABLE_MOTION_DETECT     = 0x00010000,	// Bitwise AND this with the above init commands to disable motion detection during sampling (allow for more tolerant sampling).
    INFIELD_CAL_STATE_CMD_INIT_OPTION_DISABLE_REQUIRE_VERTIAL   = 0x00020000,	// Bitwise AND this with the above init commands to disable vertical alignment requirement for accelerometer bias calibration (allow for more tolerant sampling).

    /** Sample and End Commands: */
    INFIELD_CAL_STATE_CMD_START_SAMPLE                  = 8,	// Initiate 5 second sensor sampling and averaging.  Run for each orientation and 180 degree yaw rotation.
    INFIELD_CAL_STATE_CMD_SAVE_AND_FINISH               = 9,    // Run this command to compute and save results.  Must be run following INFIELD_CAL_STATE_CMD_START_SAMPLE.
    
    /** Status: (read only) */
    INFIELD_CAL_STATE_READY_FOR_SAMPLING                = 50,   // System has been initialized and is waiting for user to intiate sampling.  User must send a command to exit this state.
    INFIELD_CAL_STATE_SAMPLING                          = 51,   // System is averaging the IMU data.  Minimize all motion and vibration.
    INFIELD_CAL_STATE_RUN_BIT_AND_FINISH                = 52,   // Follow up calibration zero with BIT and copy out IMU biases.
    INFIELD_CAL_STATE_SAVED_AND_FINISHED                = 53,   // Calculations are complete and DID_INFIELD_CAL.imu holds the update IMU biases.  Updates are saved to flash. 

    /** Error Status: (read only) */
    INFIELD_CAL_STATE_ERROR_NOT_INITIALIZED             = 100,  // Init command (INFIELD_CAL_STATE_CMD_INIT_...) not set. 
    INFIELD_CAL_STATE_ERROR_SAMPLE_ABORT_MOTION_DETECTED= 101,  // Error: Motion detected. Sampling aborted. 
    INFIELD_CAL_STATE_ERROR_SAMPLE_ABORT_NOT_VERTICAL   = 102,  // Error: System not vertical. Sampling aborted. 
    INFIELD_CAL_STATE_ERROR_NO_SAMPLES_COLLECTED        = 103,  // Error: No samples have been collected
    INFIELD_CAL_STATE_ERROR_POOR_CAL_FIT                = 104,  // Error: Calibration zero is not 

    /** Internal Use Only */
	INFIELD_CAL_STATE_CMD_MASK                          = 0x0000FFFF,
    INFIELD_CAL_STATE_CMD_START_SAMPLE_BIT              = 11,	// Initiate 5 second sensor sample and averaging.  Does not save sample into cal data.
};

enum eInfieldCalStatus
{
	INFIELD_CAL_STATUS_AXIS_DN_GRAVITY                  = 0x00000001,	// Axis points in direction of gravity more than any other axis.
	INFIELD_CAL_STATUS_AXIS_DN_SAMPLED                  = 0x00000002,	// Sampled
    INFIELD_CAL_STATUS_AXIS_DN_SAMPLED_180              = 0x00000004,	// Sampled based on average of two orientations with 180 degree delta yaw. 
	INFIELD_CAL_STATUS_AXIS_UP_GRAVITY                  = 0x00000008,	// Axis points in direction of gravity more than any other axis.
	INFIELD_CAL_STATUS_AXIS_UP_SAMPLED                  = 0x00000010,	// Sampled
    INFIELD_CAL_STATUS_AXIS_UP_SAMPLED_180              = 0x00000020,	// Sampled based on average of two orientations with 180 degree delta yaw.

    INFIELD_CAL_STATUS_SAMPLE_X_OFFSET                  = 0,
    INFIELD_CAL_STATUS_SAMPLE_Y_OFFSET                  = 6,
    INFIELD_CAL_STATUS_SAMPLE_Z_OFFSET                  = 12,
	
    INFIELD_CAL_STATUS_AXIS_MASK                        = 0x0000003F,
	INFIELD_CAL_STATUS_AXES_GRAVITY_MASK                = ( \
		((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_X_OFFSET) | \
		((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_Y_OFFSET) | \
		((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_Z_OFFSET) ),

	INFIELD_CAL_STATUS_ENABLED_ZERO_ACCEL               = 0x00100000,	// Zero accel bias.  Require vertical alignment for sampling. 
	INFIELD_CAL_STATUS_ENABLED_ZERO_GYRO                = 0x00200000,	// Zero gyro bias.
	INFIELD_CAL_STATUS_ENABLED_ZERO_ATTITUDE            = 0x00400000,	// Zero (level) INS attitude by adjusting INS rotation.
	INFIELD_CAL_STATUS_ENABLED_MOTION_DETECT            = 0x00800000,	// Require no motion during sampling. 
	INFIELD_CAL_STATUS_ENABLED_NORMAL_MASK              = 0x00F00000,
	INFIELD_CAL_STATUS_ENABLED_BIT                      = 0x01000000,	// Used for BIT 
	INFIELD_CAL_STATUS_DISABLED_REQUIRE_VERTICAL        = 0x02000000,	// Do not require vertical alignment for accelerometer calibration. 

	INFIELD_CAL_STATUS_AXIS_NOT_VERTICAL                = 0x10000000,	// Axis is not aligned vertically and cannot be used for zero accel sampling.  
	INFIELD_CAL_STATUS_MOTION_DETECTED                  = 0x20000000,	// System is not stationary and cannot be used for infield calibration.
};

/** Inertial Measurement Unit (IMU) data */
typedef struct PACKED
{
	/** Vertical axis acceleration (m/s^2) */
	float                   acc[3];
} imus_acc_t;

typedef struct PACKED
{
	imus_acc_t              dev[NUM_IMU_DEVICES];

	float					yaw;		// (rad) Heading of IMU sample.  Used to determine how to average additional samples.  0 = invalid, 999 = averaged
} infield_cal_direction_t;

typedef struct PACKED
{
	infield_cal_direction_t down;		// Pointed toward earth
	infield_cal_direction_t up;			// Pointed toward sky
} infield_cal_vaxis_t;

// (DID_INFIELD_CAL)
typedef struct PACKED
{
	/** Used to set and monitor the state of the infield calibration system. (see eInfieldCalState) */
	uint32_t                state;

	/** Infield calibration status. (see eInfieldCalStatus) */
	uint32_t                status;

	/** Number of samples used in IMU average. sampleTimeMs = 0 means "imu" member contains the IMU bias from flash.  */
	uint32_t                sampleTimeMs;

	/** Dual purpose variable.  1.) This is the averaged IMU sample when sampleTimeMs != 0.  2.) This is a mirror of the motion calibration IMU bias from flash when sampleTimeMs = 0. */ 
	imus_t                  imu[NUM_IMU_DEVICES];

	/** Collected data used to solve for the bias error and INS rotation.  Vertical axis: 0 = X, 1 = Y, 2 = Z  */
	infield_cal_vaxis_t		calData[3];

} infield_cal_t;

/** (DID_WHEEL_ENCODER) Message to communicate wheel encoder measurements to GPS-INS */
typedef struct PACKED
{
    /** Time of measurement wrt current week */
    double timeOfWeek;

    /** Status Word */
    uint32_t status;

    /** Left wheel angle (rad) */
    float theta_l;

    /** Right wheel angle (rad) */
    float theta_r;
    
    /** Left wheel angular rate (rad/s) */
    float omega_l;

    /** Right wheel angular rate (rad/s) */
    float omega_r;

    /** Left wheel revolution count */
    uint32_t wrap_count_l;

    /** Right wheel revolution count */
    uint32_t wrap_count_r;

} wheel_encoder_t;

enum eWheelCfgBits
{
    WHEEL_CFG_BITS_ENABLE_ENCODER           = (int)0x00000002,
    WHEEL_CFG_BITS_ENABLE_CONTROL           = (int)0x00000004,
    WHEEL_CFG_BITS_ENABLE_MASK              = (int)0x0000000F,
    WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT   = (int)0x00000100,
    WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT  = (int)0x00000200,
	WHEEL_CFG_BITS_ENCODER_SOURCE			= (int)0x00000400,	// 0 = uINS, 1 = EVB
};

typedef enum
{
    GV_MODE_STANDBY                         = 0,
	GV_MODE_LEARNING                        = 1,
    GV_CMD_LEARNING_START                   = 2,    // Use provided transform and sigma
    GV_CMD_LEARNING_RESUME                  = 3,    // Reset sigma values
    GV_CMD_LEARNING_CLEAR_AND_START         = 4,    // Zero transform and reset sigma values
    GV_CMD_LEARNING_STOP_AND_SAVE           = 5,
    GV_CMD_LEARNING_CANCEL                  = 6,
 } eGroundVehicleMode;

typedef struct PACKED
{
	/** Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle) in radians */
	float                   e_b2w[3];

	/** Euler angle standard deviation of measurements describing the rotation from imu (body) to the wheel frame (center of the non-steering axle) in radians */
	float                   e_b2w_sigma[3];

	/** Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame in meters */
	float                   t_b2w[3];

	/** Translation standard deviation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame in meters */
	float                   t_b2w_sigma[3];

} wheel_transform_t;

typedef struct PACKED
{
	/** Config bits (see eWheelCfgBits) */
	uint32_t                bits;

	/** Euler angles and offset describing the rotation and tranlation from imu (body) to the wheel frame (center of the non-steering axle) */
	wheel_transform_t       transform;

	/** Distance between the left and right wheels */
	float                   track_width;

	/** Estimate of wheel radius */
	float                   radius;

} wheel_config_t;

typedef enum
{
	/** Kinematic learing is solving for the translation from IMU to wheel (wheel_config). */ 
	GV_STATUS_LEARNING_ENABLED		= 0x00000001,
	
	/** Navigation is running without GPS input. */ 
	GV_STATUS_DEAD_RECKONING		= 0x01000000,

	/** Vehicle kinematic parameters agree with GPS. */ 
	GV_STATUS_KINEMATIC_CAL_GOOD	= 0x02000000,

	/** Vehicle kinematic learning has converged and is complete. */ 
	GV_STATUS_LEARNING_CONVERGED    = 0x04000000,

	/** Vehicle kinematic learning data (wheel_config_t) is missing. */ 
	GV_STATUS_LEARNING_NEEDED       = 0x08000000,

} eGroundVehicleStatus;

/** (DID_GROUND_VEHICLE) Configuration of ground vehicle kinematic constraints. */
typedef struct PACKED
{
	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t				timeOfWeekMs;

	/** Ground vehicle status flags (eGroundVehicleStatus) */
	uint32_t                status;

	/** Current mode of the ground vehicle.  Use this field to apply commands. (see eGroundVehicleMode) */
	uint32_t                mode;

	/** Wheel transform, track width, and wheel radius. */
	wheel_config_t       	wheelConfig;

} ground_vehicle_t;

typedef enum
{
    INS_DYN_MODEL_PORTABLE       	= 0,
    INS_DYN_MODEL_STATIONARY        = 2,
    INS_DYN_MODEL_PEDESTRIAN        = 3,
    INS_DYN_MODEL_GROUND_VEHICLE    = 4,
    INS_DYN_MODEL_MARINE            = 5,
    INS_DYN_MODEL_AIRBORNE_1G       = 6,
    INS_DYN_MODEL_AIRBORNE_2G       = 7,
    INS_DYN_MODEL_AIRBORNE_4G       = 8,
    INS_DYN_MODEL_WRIST             = 9
} eInsDynModel;

/** (DID_INL2_NED_SIGMA) Standard deviation of INL2 EKF estimates in the NED frame. */
typedef struct PACKED
{											
    /** Timestamp in milliseconds */
	unsigned int			timeOfWeekMs;	
    /** NED position error sigma */
	float					StdPosNed[3];		
    /** NED velocity error sigma */
	float					StdVelNed[3];		
    /** NED attitude error sigma */
	float					StdAttNed[3];		
    /** Acceleration bias error sigma */
	float					StdAccBias[3];		
    /** Angular rate bias error sigma */
	float					StdGyrBias[3];		
    /** Barometric altitude bias error sigma */
	float					StdBarBias;		
    /** Mag declination error sigma */
	float					StdMagDeclination;	
} inl2_ned_sigma_t;

/** (DID_STROBE_IN_TIME) Timestamp for input strobe. */
typedef struct PACKED
{
	/** GPS number of weeks since January 6th, 1980 */
	uint32_t				week;

	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t				timeOfWeekMs;

	/** Strobe input pin */
	uint32_t				pin;

	/** Strobe serial index number */
	uint32_t				count;
} strobe_in_time_t;

#define DEBUG_I_ARRAY_SIZE		9
#define DEBUG_F_ARRAY_SIZE		9
#define DEBUG_LF_ARRAY_SIZE		3

/* (DID_DEBUG_ARRAY) */
typedef struct PACKED
{
	int32_t					i[DEBUG_I_ARRAY_SIZE];
	f_t						f[DEBUG_F_ARRAY_SIZE];
	double                  lf[DEBUG_LF_ARRAY_SIZE];
} debug_array_t;

#define DEBUG_STRING_SIZE		80

/* (DID_DEBUG_STRING) */
typedef struct PACKED
{
	uint8_t					s[DEBUG_STRING_SIZE];
} debug_string_t;

POP_PACK

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

/** (DID_GPS1_RTK_POS_REL, DID_GPS2_RTK_CMP_REL) - RTK and Dual GNSS heading base to rover relative info. */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

    /** Age of differential (seconds) */
    float					differentialAge;

    /** Ambiguity resolution ratio factor for validation */
    float					arRatio;

	/** Vector from base to rover (m) in ECEF - If Compassing enabled, this is the 3-vector from antenna 2 to antenna 1 */
	float					baseToRoverVector[3];

    /** Distance from base to rover (m) */
    float                   baseToRoverDistance;
    
    /** Angle from north to baseToRoverVector in local tangent plane. (rad) */
    float                   baseToRoverHeading;

    /** Accuracy of baseToRoverHeading. (rad) */
    float                   baseToRoverHeadingAcc;

	/** (see eGpsStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag */
	uint32_t                status;
	
} gps_rtk_rel_t;

/** (DID_GPS1_RTK_POS_MISC, DID_GPS2_RTK_CMP_MISC) - requires little endian CPU */
typedef struct PACKED
{
	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;

	/** Accuracy - estimated standard deviations of the solution assuming a priori error model and error parameters by the positioning options. []: standard deviations {ECEF - x,y,z} or {north, east, down} (meters) */
	float					accuracyPos[3];

	/** Accuracy - estimated standard deviations of the solution assuming a priori error model and error parameters by the positioning options. []: Absolute value of means square root of estimated covariance NE, EU, UN */
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
	
	/** Number of checksum failures from received corrections */
	uint32_t				correctionChecksumFailures;

	/** Time to first RTK fix. */
	uint32_t				timeToFirstFixMs;
    
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
} gnss_raw_t;

// (DID_GPS1_TIMEPULSE)
typedef struct
{
	/*! (s)	Week seconds offset from MCU to GPS time. */
	double		towOffset;			

	/*! (s)	Week seconds for next timepulse (from start of GPS week) */
	double		towGps;				

	/*! (s)	Local MCU week seconds */
	double		timeMcu;			

	/*! (ms) Local timestamp of TIM-TP message used to validate timepulse. */
	uint32_t	msgTimeMs;			

	/*! (ms) Local timestamp of time sync pulse external interrupt used to validate timepulse. */
	uint32_t	plsTimeMs;			

	/*! Counter for successful timesync events. */
	uint8_t		syncCount;			

	/*! Counter for failed timesync events. */
	uint8_t		badPulseAgeCount;			

	/*! Counter for GPS PPS interrupt re-initalization. */
	uint8_t		ppsInterruptReinitCount;

	/*! */
	uint8_t		unused;			

	/*! (ms) Local timestamp of last valid PPS sync. */
	uint32_t	lastSyncTimeMs;		

	/*! (ms) Time since last valid PPS sync. */
	uint32_t 	sinceLastSyncTimeMs;

} gps_timepulse_t;

/**
* Diagnostic message
*/
typedef struct 
{
	/** GPS time of week (since Sunday morning) in milliseconds */
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


typedef enum
{
    /** SD card logger: card ready */
    EVB_STATUS_SD_CARD_READY                = 0x00000001,

    /** SD card Logger: running */
    EVB_STATUS_SD_LOG_ENABLED               = 0x00000002,

    /** SD card error: card file system */
    EVB_STATUS_SD_ERR_CARD_FAULT            = 0x00000010,

    /** SD card error: card full */
    EVB_STATUS_SD_ERR_CARD_FULL             = 0x00000020,

    /** SD card error: mask */
    EVB_STATUS_SD_ERR_CARD_MASK             = 0x000000F0,

    /** WiFi: enabled */
    EVB_STATUS_WIFI_ENABLED                 = 0x00010000,

    /** WiFi: connected to access point (hot spot) or another device */
    EVB_STATUS_WIFI_CONNECTED               = 0x00020000,

    /** XBee: enabled */
    EVB_STATUS_XBEE_ENABLED                 = 0x00100000,

    /** XBee: connected */
    EVB_STATUS_XBEE_CONNECTED               = 0x00200000,

    /** XBee: configured */
    EVB_STATUS_XBEE_CONFIGURED              = 0x00400000,

    /** XBee: failed to configure */
    EVB_STATUS_XBEE_CONFIG_FAILURE          = 0x00800000,

	/** System flash write staging or occuring now.  Processor will pause and not respond during a flash write, typicaly 150-250 ms. */
    EVB_STATUS_FLASH_WRITE_IN_PROGRESS      = 0x01000000,

} eEvbStatus;

/** EVB-2 communications ports. */
enum eEvb2CommPorts
{
    EVB2_PORT_UINS0     = 0,
    EVB2_PORT_UINS1     = 1,
    EVB2_PORT_XBEE      = 2,
    EVB2_PORT_XRADIO    = 3,		// H4-8 (orange) Tx, H4-7 (brown) Rx 
    EVB2_PORT_BLE       = 4,		
    EVB2_PORT_SP330     = 5,		// H3-2 (brown) Tx, H3-5 (green)  Rx
    EVB2_PORT_GPIO_H8   = 6,		// H8-5 (brown) Tx, H8-6 (orange) Rx
    EVB2_PORT_USB       = 7,
    EVB2_PORT_WIFI      = 8,		
	EVB2_PORT_CAN		= 9,		// H2-3 CANL (brown), H2-4 CANH (orange)
    EVB2_PORT_COUNT
};

/** EVB-2 Communications Bridge Options */
enum eEvb2ComBridgeOptions
{
    EVB2_CB_OPTIONS_TRISTATE_UINS_IO  = 0x00000001,
    EVB2_CB_OPTIONS_SP330_RS422       = 0x00000002,
    EVB2_CB_OPTIONS_XBEE_ENABLE       = 0x00000010,
    EVB2_CB_OPTIONS_WIFI_ENABLE       = 0x00000020,
    EVB2_CB_OPTIONS_BLE_ENABLE        = 0x00000040,
    EVB2_CB_OPTIONS_SPI_ENABLE        = 0x00000080,
	EVB2_CB_OPTIONS_CAN_ENABLE	      = 0x00000100,
	EVB2_CB_OPTIONS_I2C_ENABLE	      = 0x00000200,		// Tied to uINS G1,G2
};

enum eEvb2PortOptions
{
	EVB2_PORT_OPTIONS_RADIO_RTK_FILTER		= 0x00000001,	// Allow RTCM3, NMEA, and RTCM3.  Reject IS binary.
	EVB2_PORT_OPTIONS_DEFAULT				= EVB2_PORT_OPTIONS_RADIO_RTK_FILTER,
};

/**
* (DID_EVB_STATUS) EVB-2 status and logger control interface
*/
typedef struct
{
	/** GPS number of weeks since January 6th, 1980 */
	uint32_t                week;

	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;

	/** Firmware (software) version */
	uint8_t                 firmwareVer[4];

    /** Status (eEvbStatus) */
    uint32_t                evbStatus;

    /** Data logger control state. (see eEvb2LoggerMode) */
    uint32_t                loggerMode;

    /** logger */
    uint32_t                loggerElapsedTimeMs;

    /** WiFi IP address */
    uint32_t                wifiIpAddr;

    /** System command (see eSystemCommand).  99 = software reset */
    uint32_t                sysCommand;

	/** Time sync offset between local time since boot up to GPS time of week in seconds.  Add this to IMU and sensor time to get GPS time of week in seconds. */
	double                  towOffset;

} evb_status_t;

#define WIFI_SSID_PSK_SIZE      40

typedef struct
{
    /** WiFi SSID */
    char                    ssid[WIFI_SSID_PSK_SIZE];

    /** WiFi PSK */
    char                    psk[WIFI_SSID_PSK_SIZE];

} evb_wifi_t;

typedef struct
{  
    /** Server IP address */
    union {
		uint32_t	u32;
		uint8_t		u8[4];
	} ipAddr;

    /** Server port */
    uint32_t                port;

} evb_server_t;

typedef enum
{
    EVB_CFG_BITS_WIFI_SELECT_MASK               = 0x00000003,
    EVB_CFG_BITS_WIFI_SELECT_OFFSET             = 0,
    EVB_CFG_BITS_SERVER_SELECT_MASK             = 0x0000000C,
    EVB_CFG_BITS_SERVER_SELECT_OFFSET           = 2,
    EVB_CFG_BITS_NO_STREAM_PPD_ON_LOG_BUTTON    = 0x00000010,		// Don't enable PPD stream when log button is pressed
    EVB_CFG_BITS_ENABLE_ADC4                    = 0x00000200,
	EVB_CFG_BITS_ENABLE_ADC10					= 0x00000400,
} eEvbFlashCfgBits;

#define NUM_WIFI_PRESETS     3
#define EVB_CFG_BITS_SET_IDX_WIFI(bits,idx)     {bits&=EVB_CFG_BITS_WIFI_SELECT_MASK; bits|=((idx<<EVB_CFG_BITS_WIFI_SELECT_OFFSET)&EVB_CFG_BITS_WIFI_SELECT_MASK);}
#define EVB_CFG_BITS_SET_IDX_SERVER(bits,idx)   {bits&=EVB_CFG_BITS_SERVER_SELECT_MASK; bits|=((idx<<EVB_CFG_BITS_SERVER_SELECT_OFFSET)&EVB_CFG_BITS_SERVER_SELECT_MASK);}
#define EVB_CFG_BITS_IDX_WIFI(bits)             ((bits&EVB_CFG_BITS_WIFI_SELECT_MASK)>>EVB_CFG_BITS_WIFI_SELECT_OFFSET)
#define EVB_CFG_BITS_IDX_SERVER(bits)           ((bits&EVB_CFG_BITS_SERVER_SELECT_MASK)>>EVB_CFG_BITS_SERVER_SELECT_OFFSET)

/**
* (DID_EVB_FLASH_CFG) EVB-2 flash config for monitor, config, and logger control interface
*/
typedef struct
{  
    /** Size of this struct */
    uint32_t				size;

    /** Checksum, excluding size and checksum */
    uint32_t                checksum;

    /** Manufacturer method for restoring flash defaults */
    uint32_t                key;

    /** Communications bridge preset. (see eEvb2ComBridgePreset) */
    uint8_t                 cbPreset;

	// 32-bit alignment
	uint8_t                 reserved1[3];

    /** Communications bridge forwarding */
    uint32_t                cbf[EVB2_PORT_COUNT];

    /** Communications bridge options (see eEvb2ComBridgeOptions) */
    uint32_t                cbOptions;

    /** Config bits (see eEvbFlashCfgBits) */
    uint32_t                bits;

    /** Radio preamble ID (PID) - 0x0 to 0x9. Only radios with matching PIDs can communicate together. Different PIDs minimize interference between multiple sets of networks. Checked before the network ID. */
    uint32_t                radioPID;

    /** Radio network ID (NID) - 0x0 to 0x7FFF. Only radios with matching NID can communicate together. Checked after the preamble ID. */
    uint32_t                radioNID;

    /** Radio power level - Transmitter output power level. (XBee PRO SX 0=20dBm, 1=27dBm, 2=30dBm)  */
    uint32_t                radioPowerLevel;

    /** WiFi SSID and PSK */
    evb_wifi_t              wifi[NUM_WIFI_PRESETS];

    /** Server IP and port */
    evb_server_t            server[NUM_WIFI_PRESETS];

    /** Encoder tick to wheel rotation conversion factor (in radians).  Encoder tick count per revolution on 1 channel x gear ratio x 2pi. */
    float                   encoderTickToWheelRad;

	/** CAN baudrate */
	uint32_t				CANbaud_kbps;

	/** CAN receive address */
	uint32_t				can_receive_address;

	/** EVB port for uINS communications and SD card logging. 0=uINS-Ser0 (default), 1=uINS-Ser1, SP330=5, 6=GPIO_H8 (use eEvb2CommPorts) */
	uint8_t                 uinsComPort;

	/** EVB port for uINS aux com and RTK corrections. 0=uINS-Ser0, 1=uINS-Ser1 (default), 5=SP330, 6=GPIO_H8 (use eEvb2CommPorts) */
	uint8_t                 uinsAuxPort;

	// Ensure 32-bit alignment
	uint8_t                	reserved2[2];

	/** Enable radio RTK filtering, etc. (see eEvb2PortOptions) */
	uint32_t                portOptions;

	/** Baud rate for EVB serial port H3 (SP330 RS233 and RS485/422). */
	uint32_t                h3sp330BaudRate;

	/** Baud rate for EVB serial port H4 (TLL to external radio). */
	uint32_t                h4xRadioBaudRate;

	/** Baud rate for EVB serial port H8 (TLL). */
	uint32_t                h8gpioBaudRate;

	/** Wheel encoder configuration (see eWheelCfgBits) */
	uint32_t                wheelCfgBits;

	/** Wheel update period.  Sets the wheel encoder and control update period. (ms) */
	uint32_t				velocityControlPeriodMs;

} evb_flash_cfg_t;


/** EVB-2 communications bridge configuration. */
enum eEvb2ComBridgePreset
{
    /** No change.  Sending this value causes no effect. */
    EVB2_CB_PRESET_NA = 0,

    /** No connections.  Off: XBee, WiFi */
	EVB2_CB_PRESET_ALL_OFF = 1,

    /** [uINS Hub] LED-GRN (uINS-COM): USB, RS232, H8.  (uINS-AUX): XRadio.  Off: XBee, WiFi */
    EVB2_CB_PRESET_RS232 = 2,

    /** [uINS Hub] LED-BLU (uINS-COM): USB, RS232, H8.  (uINS-AUX): XBee, XRadio.  Off: WiFi */
    EVB2_CB_PRESET_RS232_XBEE = 3,

    /** [uINS Hub] LED-PUR (uINS-COM): USB, RS422, H8.  (uINS-AUX): WiFi, XRadio.  Off: XBee */
    EVB2_CB_PRESET_RS422_WIFI = 4,

    /** [uINS Hub] LED-CYA (uINS-SER1 SPI): USB, RS423, H8.  Off: WiFi, XBee.  A reset is required following selection of this CBPreset to enable SPI on the uINS, in order to assert uINS pin 10 (G9/nSPI_EN) during bootup. */
    EVB2_CB_PRESET_SPI_RS232 = 5,

    /** [USB Hub]  LED-YEL (USB): RS232, H8, XBee, XRadio. */
    EVB2_CB_PRESET_USB_HUB_RS232 = 6,

    /** [USB Hub]  LED-WHT (USB): RS485/RS422, H8, XRadio. */
    EVB2_CB_PRESET_USB_HUB_RS422 = 7,
	
    /** Number of bridge configuration presets */
	EVB2_CB_PRESET_COUNT = 8,
    
};

#define EVB2_CB_PRESET_DEFAULT      EVB2_CB_PRESET_RS232

/** Data logger control.  Values labeled CMD  */
enum eEvb2LoggerMode
{
    /** Do not change.  Sending this value causes no effect. */
    EVB2_LOG_NA                         = 0,

    /** Start new log */
    EVB2_LOG_CMD_START                  = 2,

    /** Stop logging */
    EVB2_LOG_CMD_STOP                   = 4,

    /** Purge all data logs from drive */
    EVB2_LOG_CMD_PURGE                  = 1002,
        
};


/** 
* (DID_PORT_MONITOR) Data rate and status monitoring for each communications port. 
*/
typedef struct
{
    /** Tx rate (bytes/s) */
    uint32_t        txBytesPerS;

    /** Rx rate (bytes/s) */
    uint32_t        rxBytesPerS;

    /** Status */
    uint32_t        status;
    
} port_monitor_set_t;

typedef struct
{
	/** Port monitor set */
	port_monitor_set_t port[NUM_SERIAL_PORTS];
		
} port_monitor_t;


/**
* (DID_SYS_FAULT) System Fault Information 
* NOTE: If you modify these, please update crash_info_special_values in IS-src/python/src/ci_hdw/data_sets.py */
#define SYS_FAULT_STATUS_HARDWARE_RESET                 0x00000000
#define SYS_FAULT_STATUS_USER_RESET                     0x00000001
#define SYS_FAULT_STATUS_ENABLE_BOOTLOADER              0x00000002
// General:
#define SYS_FAULT_STATUS_SOFT_RESET                     0x00000010
#define SYS_FAULT_STATUS_FLASH_MIGRATION_EVENT          0x00000020
#define SYS_FAULT_STATUS_FLASH_MIGRATION_COMPLETED      0x00000040
#define SYS_FAULT_STATUS_RTK_MISC_ERROR                 0x00000080
#define SYS_FAULT_STATUS_MASK_GENERAL_ERROR             0xFFFFFFF0
// Critical: (usually associated with system reset)
#define SYS_FAULT_STATUS_HARD_FAULT                     0x00010000
#define SYS_FAULT_STATUS_USAGE_FAULT                    0x00020000
#define SYS_FAULT_STATUS_MEM_MANGE                      0x00040000
#define SYS_FAULT_STATUS_BUS_FAULT                      0x00080000
#define SYS_FAULT_STATUS_MALLOC_FAILED                  0x00100000
#define SYS_FAULT_STATUS_STACK_OVERFLOW                 0x00200000
#define SYS_FAULT_STATUS_INVALID_CODE_OPERATION         0x00400000
#define SYS_FAULT_STATUS_FLASH_MIGRATION_MARKER_UPDATED 0x00800000
#define SYS_FAULT_STATUS_WATCHDOG_RESET                 0x01000000
#define SYS_FAULT_STATUS_RTK_BUFFER_LIMIT               0x02000000
#define SYS_FAULT_STATUS_SENSOR_CALIBRATION             0x04000000
#define SYS_FAULT_STATUS_HARDWARE_DETECTION             0x08000000
#define SYS_FAULT_STATUS_MASK_CRITICAL_ERROR            0xFFFF0000

typedef struct 
{
    /** System fault status */
    uint32_t status;

    /** Fault Type at HardFault */
    uint32_t g1Task;

    /** Multipurpose register - Line number of fault */
    uint32_t g2FileNum;
    
    /** Multipurpose register - File number at fault */
    uint32_t g3LineNum;
        
    /** Multipurpose register - at time of fault.  */
	uint32_t g4;

    /** Multipurpose register - link register value at time of fault.  */
    uint32_t g5Lr;
    
    /** Program Counter value at time of fault */
	uint32_t pc;
    
    /** Program Status Register value at time of fault */
	uint32_t psr;
    	
} system_fault_t;

/** Diagnostic information for internal use */
typedef struct
{
	/** Count of gap of more than 0.5 seconds receiving serial data, driver level, one entry for each com port */
	uint32_t gapCountSerialDriver[NUM_SERIAL_PORTS];

	/** Count of gap of more than 0.5 seconds receiving serial data, class / parser level, one entry for each com port */
	uint32_t gapCountSerialParser[NUM_SERIAL_PORTS];

	/** Count of rx overflow, one entry for each com port */
	uint32_t rxOverflowCount[NUM_SERIAL_PORTS];

	/** Count of tx overflow, one entry for each com port */
	uint32_t txOverflowCount[NUM_SERIAL_PORTS];
	
	/** Count of checksum failures, one entry for each com port */
	uint32_t checksumFailCount[NUM_SERIAL_PORTS];
} internal_diagnostic_t;

/** RTOS tasks */
typedef enum
{
	/** Task 0: Sample	*/
	TASK_SAMPLE = 0,

	/** Task 1: Nav */
	TASK_NAV,

	/** Task 2: Communications */
	TASK_COMMUNICATIONS,

	/** Task 3: Maintenance */
	TASK_MAINTENANCE,

	/** Task 4: Idle */
	TASK_IDLE,

	/** Task 5: Timer */
	TASK_TIMER,

	/** Number of RTOS tasks */
	UINS_RTOS_NUM_TASKS                 // Keep last
} eRtosTask;

/** EVB RTOS tasks */
typedef enum
{
    /** Task 0: Communications */
    EVB_TASK_COMMUNICATIONS,

    /** Task 1: Logger */
    EVB_TASK_LOGGER,

    /** Task 2: WiFi */
    EVB_TASK_WIFI,

    /** Task 3: Maintenance */
    EVB_TASK_MAINTENANCE,

    /** Task 4: Idle */
    EVB_TASK_IDLE,

    /** Task 5: Timer */
    EVB_TASK_TIMER,

    /** Task 6: SPI to uINS */
    EVB_TASK_SPI_UINS_COM,

    /** Number of RTOS tasks */
    EVB_RTOS_NUM_TASKS                  // Keep last
} eEvbRtosTask;

/** Max task name length - do not change */
#define MAX_TASK_NAME_LEN 12

/** RTOS task info */
typedef struct PACKED
{
	/** Task name */
	char                    name[MAX_TASK_NAME_LEN];

	/** Task priority (0 - 8) */
	uint32_t                priority;

	/** Stack high water mark bytes */
	uint32_t                stackUnused;

	/** Task period ms */
	uint32_t                periodMs;

	/** Last run time microseconds */
	uint32_t                runTimeUs;

	/** Max run time microseconds */
	uint32_t                maxRunTimeUs;
	
	/** Rolling average over last 1000 executions */
	float					averageRunTimeUs;
	
	/** Counter of times task took too long to run */
	uint32_t				gapCount;

	/** Cpu usage percent */
    float					cpuUsage;

	/** Handle */
	uint32_t                handle;

	/** Local time when task loop started (following delay) */
	uint32_t                profileStartTimeUs;

    /** Task info for static memory allocation */
    StaticTask_t            taskBuffer;
	
} rtos_task_t;

/** (DID_RTOS_INFO) */
typedef struct PACKED
{
    /** Heap high water mark bytes */
    uint32_t                freeHeapSize;

    /** Total memory allocated using RTOS pvPortMalloc() */
    uint32_t				mallocSize;
    
	/** Total memory freed using RTOS vPortFree() */
	uint32_t				freeSize;

	/** Tasks */
	rtos_task_t             task[UINS_RTOS_NUM_TASKS];

} rtos_info_t;

/** (DID_EVB_RTOS_INFO) */
typedef struct PACKED
{
    /** Heap high water mark bytes */
    uint32_t                freeHeapSize;

	/** Total memory allocated using RTOS pvPortMalloc() */
	uint32_t				mallocSize;

	/** Total memory freed using RTOS vPortFree() */
	uint32_t				freeSize;

    /** Tasks */
    rtos_task_t             task[EVB_RTOS_NUM_TASKS];

} evb_rtos_info_t;

enum
{
	CID_INS_TIME,
	CID_INS_STATUS,
	CID_INS_EULER,
	CID_INS_QUATN2B,
	CID_INS_QUATE2B,
	CID_INS_UVW,
	CID_INS_VE,
	CID_INS_LAT,
	CID_INS_LON,
	CID_INS_ALT,
	CID_INS_NORTH_EAST,
	CID_INS_DOWN,
	CID_INS_ECEF_X,
	CID_INS_ECEF_Y,
	CID_INS_ECEF_Z,
	CID_INS_MSL,
	CID_PREINT_PX,
	CID_PREINT_QY,
	CID_PREINT_RZ,
	CID_DUAL_PX,
	CID_DUAL_QY,
	CID_DUAL_RZ,
	CID_GPS1_POS,
	CID_GPS1_RTK_REL,
	CID_ROLL_ROLLRATE,
	NUM_CIDS
};

/** Valid baud rates for Inertial Sense hardware */
typedef enum
{
	CAN_BAUDRATE_20_KBPS   =   20,
	CAN_BAUDRATE_33_KBPS   =   33,
	CAN_BAUDRATE_50_KBPS   =   50,
	CAN_BAUDRATE_83_KBPS   =   83,
	CAN_BAUDRATE_100_KBPS  =  100,
	CAN_BAUDRATE_125_KBPS  =  125,
	CAN_BAUDRATE_200_KBPS  =  200,
	CAN_BAUDRATE_250_KBPS  =  250,
	CAN_BAUDRATE_500_KBPS  =  500,
	CAN_BAUDRATE_1000_KBPS = 1000,

	CAN_BAUDRATE_COUNT = 10
} can_baudrate_t;

/** (DID_CAN_BCAST_PERIOD) Broadcast period of CAN messages */
typedef struct PACKED
{
	/** Broadcast period (ms) - CAN time message. 0 to disable. */
	uint16_t				can_period_mult[NUM_CIDS];
	
	/** Transmit address. */
	uint32_t				can_transmit_address[NUM_CIDS];
	
	/** Baud rate (kbps)  (See can_baudrate_t for valid baud rates)  */
	uint16_t				can_baudrate_kbps;

	/** Receive address. */
	uint32_t				can_receive_address;

} can_config_t;

#if defined(INCLUDE_LUNA_DATA_SETS)
#include "luna_data_sets.h"
#endif

/** Union of datasets */
typedef union PACKED
{
	dev_info_t				devInfo;
	ins_1_t					ins1;
	ins_2_t					ins2;
 	ins_3_t					ins3;
	ins_4_t					ins4;
	imu_t					imu;
	imu3_t					imu3;
	magnetometer_t			mag;
	mag_cal_t				magCal;
	barometer_t				baro;
    wheel_encoder_t			wheelEncoder;
	ground_vehicle_t		groundVehicle;
	pimu_t					pImu;
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
	rmc_t					rmc;
	evb_status_t			evbStatus;
	infield_cal_t			infieldCal;

#if defined(INCLUDE_LUNA_DATA_SETS)
	evb_luna_velocity_control_t     wheelController;
#endif
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
uint32_t checksum32(const void* data, int count);
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
void flipDouble(void* ptr);

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
uint64_t didToRmcBit(uint32_t dataId, uint64_t defaultRmcBits, uint64_t devInfoRmcBits);

//Time conversion constants
#define SECONDS_PER_WEEK        604800
#define SECONDS_PER_DAY         86400
#define GPS_TO_UNIX_OFFSET      315964800
/** Convert GPS Week and Ms and leapSeconds to Unix seconds**/
double gpsToUnix(uint32_t gpsWeek, uint32_t gpsTimeofWeekMS, uint8_t leapSeconds);

/** Convert Julian Date to calendar date. */
void julianToDate(double julian, int32_t* year, int32_t* month, int32_t* day, int32_t* hour, int32_t* minute, int32_t* second, int32_t* millisecond);

/** Convert GPS Week and Seconds to Julian Date.  Leap seconds are the GPS-UTC offset (18 seconds as of December 31, 2016). */
double gpsToJulian(int32_t gpsWeek, int32_t gpsMilliseconds, int32_t leapSeconds);


#ifndef RTKLIB_H
#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_IRN     0x40                /* navigation system: IRNS */
#define SYS_LEO     0x80                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */
#endif

/*
Convert gnssID to ubx gnss indicator (ref [2] 25)

@param gnssID gnssID of satellite
@return ubx gnss indicator
*/
int ubxSys(int gnssID);

/*
Convert satellite constelation and prn/slot number to satellite number

@param sys satellite system (SYS_GPS,SYS_GLO,...)
@param prn satellite prn/slot number
@return satellite number (0:error)
*/
int satNo(int sys, int prn);

/*
convert satellite gnssID + svID to satellite number

@param gnssID satellite system 
@param svID satellite prn/slot number
@return satellite number (0:error)
*/
int satNumCalc(int gnssID, int svID);

typedef gnss_pos_t gps_pos_t;
typedef gnss_vel_t gps_vel_t;
typedef gnss_sat_t gps_sat_t;
typedef gnss_raw_t gps_raw_t;
typedef gnss_sat_sv_t gps_sat_sv_t;

#ifdef __cplusplus
}
#endif

#endif // DATA_SETS_H
