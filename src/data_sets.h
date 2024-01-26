/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

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
#define DID_GPS1_UBX_POS                (eDataIDs)6  /** (gps_pos_t) GPS 1 position data from ublox receiver. */
#define DID_SYS_CMD                     (eDataIDs)7  /** (system_command_t) System commands. Both the command and invCommand fields must be set at the same time for a command to take effect. */
#define DID_NMEA_BCAST_PERIOD           (eDataIDs)8  /** (nmea_msgs_t) Set broadcast periods for NMEA messages */
#define DID_RMC                         (eDataIDs)9  /** (rmc_t) Realtime Message Controller (RMC). The data sets available through RMC are driven by the availability of the data. The RMC provides updates from various data sources (i.e. sensors) as soon as possible with minimal latency. Several of the data sources (sensors) output data at different data rates that do not all correspond. The RMC is provided so that broadcast of sensor data is done as soon as it becomes available. All RMC messages can be enabled using the standard Get Data packet format. */
#define DID_SYS_PARAMS                  (eDataIDs)10 /** (sys_params_t) System parameters / info */
#define DID_SYS_SENSORS                 (eDataIDs)11 /** (sys_sensors_t) System sensor information */
#define DID_FLASH_CONFIG                (eDataIDs)12 /** (nvm_flash_cfg_t) Flash memory configuration */
#define DID_GPS1_POS                    (eDataIDs)13 /** (gps_pos_t) GPS 1 position data.  This comes from DID_GPS1_UBX_POS or DID_GPS1_RTK_POS, depending on whichever is more accurate. */
#define DID_GPS2_POS                    (eDataIDs)14 /** (gps_pos_t) GPS 2 position data */
#define DID_GPS1_SAT                    (eDataIDs)15 /** (gps_sat_t) GPS 1 GNSS satellite information: sat identifiers, carrier to noise ratio, elevation and azimuth angles, pseudo range residual. */
#define DID_GPS2_SAT                    (eDataIDs)16 /** (gps_sat_t) GPS 2 GNSS satellite information: sat identifiers, carrier to noise ratio, elevation and azimuth angles, pseudo range residual. */
#define DID_GPS1_VERSION                (eDataIDs)17 /** (gps_version_t) GPS 1 version info */
#define DID_GPS2_VERSION                (eDataIDs)18 /** (gps_version_t) GPS 2 version info */
#define DID_MAG_CAL                     (eDataIDs)19 /** (mag_cal_t) Magnetometer calibration */
#define DID_INTERNAL_DIAGNOSTIC         (eDataIDs)20 /** INTERNAL USE ONLY (internal_diagnostic_t) Internal diagnostic info */
#define DID_GPS1_RTK_POS_REL            (eDataIDs)21 /** (gps_rtk_rel_t) RTK precision position base to rover relative info. */
#define DID_GPS1_RTK_POS_MISC           (eDataIDs)22 /** (gps_rtk_misc_t) RTK precision position related data. */
#define DID_FEATURE_BITS                (eDataIDs)23 /** INTERNAL USE ONLY (feature_bits_t) */
#define DID_SENSORS_UCAL                (eDataIDs)24 /** INTERNAL USE ONLY (sensors_w_temp_t) Uncalibrated IMU output. */
#define DID_SENSORS_TCAL                (eDataIDs)25 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated IMU output. */
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
#define DID_SENSORS_MCAL                (eDataIDs)40 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated and motion calibrated IMU output. */
#define DID_GPS1_TIMEPULSE              (eDataIDs)41 /** INTERNAL USE ONLY (gps_timepulse_t) */
#define DID_CAL_SC                      (eDataIDs)42 /** INTERNAL USE ONLY (sensor_cal_t) */
#define DID_CAL_TEMP_COMP               (eDataIDs)43 /** INTERNAL USE ONLY (sensor_tcal_group_t) */
#define DID_CAL_MOTION                  (eDataIDs)44 /** INTERNAL USE ONLY (sensor_mcal_group_t) */
#define DID_GPS1_SIG                    (eDataIDs)45 /** (gps_sig_t) GPS 1 GNSS signal information. */
#define DID_SENSORS_ADC_SIGMA           (eDataIDs)46 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_REFERENCE_MAGNETOMETER      (eDataIDs)47 /** (magnetometer_t) Reference or truth magnetometer used for manufacturing calibration and testing */
#define DID_INL2_STATES                 (eDataIDs)48 /** (inl2_states_t) INS Extended Kalman Filter (EKF) states */
#define DID_INL2_COVARIANCE_LD          (eDataIDs)49 /** (INL2_COVARIANCE_LD_ARRAY_SIZE) */
#define DID_INL2_STATUS                 (eDataIDs)50 /** (inl2_status_t) */
#define DID_INL2_MISC                   (eDataIDs)51 /** (inl2_misc_t) */
#define DID_MAGNETOMETER                (eDataIDs)52 /** (magnetometer_t) Magnetometer sensor output */
#define DID_BAROMETER                   (eDataIDs)53 /** (barometer_t) Barometric pressure sensor data */
#define DID_GPS1_RTK_POS                (eDataIDs)54 /** (gps_pos_t) GPS RTK position data */
#define DID_ROS_COVARIANCE_POSE_TWIST   (eDataIDs)55 /** (ros_covariance_pose_twist_t) INL2 EKF covariances matrix lower diagonals */
#define DID_COMMUNICATIONS_LOOPBACK     (eDataIDs)56 /** INTERNAL USE ONLY - Unit test for communications manager  */
#define DID_IMU3_UNCAL                  (eDataIDs)57 /** INTERNAL USE ONLY (imu3_t) Uncalibrated triple IMU data.  We recommend use of DID_IMU or DID_PIMU as they are calibrated and oversampled and contain less noise.  Minimum data period is DID_FLASH_CONFIG.startupImuDtMs or 4, whichever is larger (250Hz max). */
#define DID_IMU                         (eDataIDs)58 /** (imu_t) Inertial measurement unit data down-sampled from IMU rate (DID_FLASH_CONFIG.startupImuDtMs (1KHz)) to navigation update rate (DID_FLASH_CONFIG.startupNavDtMs) as an anti-aliasing filter to reduce noise and preserve accuracy.  Minimum data period is DID_FLASH_CONFIG.startupNavDtMs (1KHz max).  */
#define DID_INL2_MAG_OBS_INFO           (eDataIDs)59 /** (inl2_mag_obs_info_t) INL2 magnetometer calibration information. */
#define DID_GPS_BASE_RAW                (eDataIDs)60 /** (gps_raw_t) GPS raw data for base station (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_GPS_RTK_OPT                 (eDataIDs)61 /** (gps_rtk_opt_t) RTK options - requires little endian CPU. */
#define DID_REFERENCE_PIMU              (eDataIDs)62 /** (pimu_t) Reference or truth IMU used for manufacturing calibration and testing */
#define DID_MANUFACTURING_INFO          (eDataIDs)63 /** INTERNAL USE ONLY (manufacturing_info_t) Manufacturing info */
#define DID_BIT                         (eDataIDs)64 /** (bit_t) System built-in self-test */
#define DID_INS_3                       (eDataIDs)65 /** (ins_3_t) Inertial navigation data with quaternion NED to body rotation and ECEF position. */
#define DID_INS_4                       (eDataIDs)66 /** (ins_4_t) INS output: quaternion rotation w/ respect to ECEF, ECEF position. */
#define DID_INL2_NED_SIGMA              (eDataIDs)67 /** (inl2_ned_sigma_t) Standard deviation of INL2 EKF estimates in the NED frame. */
#define DID_STROBE_IN_TIME              (eDataIDs)68 /** (strobe_in_time_t) Timestamp for input strobe. */
#define DID_GPS1_RAW                    (eDataIDs)69 /** (gps_raw_t) GPS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_GPS2_RAW                    (eDataIDs)70 /** (gps_raw_t) GPS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_WHEEL_ENCODER               (eDataIDs)71 /** (wheel_encoder_t) Wheel encoder data to be fused with GPS-INS measurements, set DID_GROUND_VEHICLE for configuration before sending this message */
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
#define DID_GPS2_SIG                    (eDataIDs)84 /** (gps_sig_t) GPS 2 GNSS signal information. */
#define DID_IMU_MAG                     (eDataIDs)85 /** (imu_mag_t) DID_IMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define DID_PIMU_MAG                    (eDataIDs)86 /** (pimu_mag_t) DID_PIMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define DID_GROUND_VEHICLE				(eDataIDs)87 /** (ground_vehicle_t) Static configuration for wheel transform measurements. */
#define DID_POSITION_MEASUREMENT		(eDataIDs)88 /** (pos_measurement_t) External position estimate */
#define DID_RTK_DEBUG_2                 (eDataIDs)89 /** INTERNAL USE ONLY (rtk_debug_2_t) */
#define DID_CAN_CONFIG					(eDataIDs)90 /** (can_config_t) Addresses for CAN messages*/
#define DID_GPS2_RTK_CMP_REL            (eDataIDs)91 /** (gps_rtk_rel_t) Dual GNSS RTK compassing / moving base to rover (GPS 1 to GPS 2) relative info. */
#define DID_GPS2_RTK_CMP_MISC           (eDataIDs)92 /** (gps_rtk_misc_t) RTK Dual GNSS RTK compassing related data. */
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

/** Count of data ids (including null data id 0) - MUST BE MULTPLE OF 4 and larger than last DID number! */
#define DID_COUNT		(eDataIDs)120	// Used in SDK
#define DID_COUNT_UINS	(eDataIDs)100	// Used in uINS

/** Maximum number of data ids */
#define DID_MAX_COUNT 256

// END DATA IDENTIFIERS --------------------------------------------------------------------------

/** Maximum number of satellite channels */
#define MAX_NUM_SATELLITES 50

/** Maximum number of satellite signals */
#define MAX_NUM_SAT_SIGNALS 100

/** Maximum length of device info manufacturer string (must be a multiple of 4) */
#define DEVINFO_MANUFACTURER_STRLEN 24
#define DEVINFO_ADDINFO_STRLEN 24


/** Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */
// #define PROTOCOL_VERSION_CHAR0 1        // Major (in ISComm.h)
// #define PROTOCOL_VERSION_CHAR1 0
#define PROTOCOL_VERSION_CHAR2 (0x000000FF&DID_COUNT_UINS)
#define PROTOCOL_VERSION_CHAR3 9         // Minor (in data_sets.h)

/** Rtk rover receiver index */
#define RECEIVER_INDEX_GPS1 1 // DO NOT CHANGE
#define RECEIVER_INDEX_EXTERNAL_BASE 2 // DO NOT CHANGE
#define RECEIVER_INDEX_GPS2 3 // DO NOT CHANGE

// Max number of devices across all hardware types: uINS-3, uINS-4, and IMX-5
#define NUM_IMU_DEVICES     3        // g_numImuDevices defines the actual number of hardware specific devices
#define NUM_MAG_DEVICES     2        // g_numMagDevices defines the actual number of hardware specific devices

/** INS status flags */
enum eInsStatusFlags
{
    /** Attitude estimate is usable but outside spec (COARSE) */
    INS_STATUS_HDG_ALIGN_COARSE                 = (int)0x00000001,
    /** Velocity estimate is usable but outside spec (COARSE) */
    INS_STATUS_VEL_ALIGN_COARSE                 = (int)0x00000002,
    /** Position estimate is usable but outside spec (COARSE) */
    INS_STATUS_POS_ALIGN_COARSE                 = (int)0x00000004,
    /** Estimate is COARSE mask (usable but outside spec) */
    INS_STATUS_ALIGN_COARSE_MASK                = (int)0x00000007,

    /** Velocity aided by wheel sensor */
    INS_STATUS_WHEEL_AIDING_VEL                 = (int)0x00000008,

    /** Attitude estimate is within spec (FINE) */
    INS_STATUS_HDG_ALIGN_FINE                   = (int)0x00000010,
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
    INS_STATUS_NAV_MODE                         = (int)0x00001000,

    /** INS in stationary mode.  If initiated by zero velocity command, user should not move (keep system motionless) to assist on-board processing. */
    INS_STATUS_STATIONARY_MODE                  = (int)0x00002000,    
    /** Velocity aided by GPS velocity */
    INS_STATUS_GPS_AIDING_VEL                   = (int)0x00004000,
    /** Vehicle kinematic calibration is good */
    INS_STATUS_KINEMATIC_CAL_GOOD               = (int)0x00008000,

    /** INS/AHRS Solution Status */
    INS_STATUS_SOLUTION_MASK                    = (int)0x000F0000,
    INS_STATUS_SOLUTION_OFFSET                  = 16,
#define INS_STATUS_SOLUTION(insStatus)          (((insStatus)&INS_STATUS_SOLUTION_MASK)>>INS_STATUS_SOLUTION_OFFSET)

    INS_STATUS_SOLUTION_OFF                     = 0,    // System is off 
    INS_STATUS_SOLUTION_ALIGNING                = 1,    // System is in alignment mode
    INS_STATUS_SOLUTION_NAV                     = 3,    // System is in navigation mode and solution is good.
    INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE       = 4,    // System is in navigation mode but the attitude uncertainty has exceeded the threshold.
    INS_STATUS_SOLUTION_AHRS                    = 5,    // System is in AHRS mode and solution is good.
    INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE      = 6,    // System is in AHRS mode but the attitude uncertainty has exceeded the threshold.
    INS_STATUS_SOLUTION_VRS                     = 7,    // System is in VRS mode (no earth relative heading) and roll and pitch are good.
    INS_STATUS_SOLUTION_VRS_HIGH_VARIANCE       = 8,    // System is in VRS mode (no earth relative heading) but roll and pitch uncertainty has exceeded the threshold.

    /** GPS compassing antenna offsets are not set in flashCfg. */
    INS_STATUS_RTK_COMPASSING_BASELINE_UNSET    = (int)0x00100000,
    /** GPS antenna baseline specified in flashCfg and measured by GPS do not match. */
    INS_STATUS_RTK_COMPASSING_BASELINE_BAD      = (int)0x00200000,
    INS_STATUS_RTK_COMPASSING_MASK              = (INS_STATUS_RTK_COMPASSING_BASELINE_UNSET|INS_STATUS_RTK_COMPASSING_BASELINE_BAD),

    /** Magnetometer is being recalibrated.  Device requires rotation to complete the calibration process. HDW_STATUS_MAG_RECAL_COMPLETE is set when complete. */
    INS_STATUS_MAG_RECALIBRATING                = (int)0x00400000,
    /** Magnetometer is experiencing interference or calibration is bad.  Attention may be required to remove interference (move the device) or recalibrate the magnetometer. */
    INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL      = (int)0x00800000,

    /** GPS navigation fix type (see eGpsNavFixStatus) */
    INS_STATUS_GPS_NAV_FIX_MASK                 = (int)0x03000000,
    INS_STATUS_GPS_NAV_FIX_OFFSET               = 24,
#define INS_STATUS_NAV_FIX_STATUS(insStatus)    (((insStatus)&INS_STATUS_GPS_NAV_FIX_MASK)>>INS_STATUS_GPS_NAV_FIX_OFFSET)

    /** RTK compassing heading is accurate.  (RTK fix and hold status) */
    INS_STATUS_RTK_COMPASSING_VALID             = (int)0x04000000,

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
    INS_STATUS_RTK_ERROR_MASK                   = (INS_STATUS_RTK_RAW_GPS_DATA_ERROR|INS_STATUS_RTK_ERR_BASE_MASK),
    
    /** RTOS task ran longer than allotted period */
    INS_STATUS_RTOS_TASK_PERIOD_OVERRUN         = (int)0x40000000,
    /** General fault (see sys_params_t.genFaultCode) */
    INS_STATUS_GENERAL_FAULT                    = (int)0x80000000,
};

/** GPS navigation fix type */
/* NOTE: If you modify this enum, please also modify the eGpsNavFixStatus enum
 *       in IS-src/python/src/ci_hdw/data_sets.py */
enum eGpsNavFixStatus
{
    GPS_NAV_FIX_NONE                            = (int)0x00000000,
    GPS_NAV_FIX_POSITIONING_3D                  = (int)0x00000001,
    GPS_NAV_FIX_POSITIONING_RTK_FLOAT           = (int)0x00000002,
    GPS_NAV_FIX_POSITIONING_RTK_FIX             = (int)0x00000003,        // Includes fix & hold
};

/** Hardware status flags */
enum eHdwStatusFlags
{
    /** Gyro motion detected sigma */
    HDW_STATUS_MOTION_GYR_SIG                   = (int)0x00000001,
    /** Accelerometer motion detected sigma */
    HDW_STATUS_MOTION_ACC_SIG                   = (int)0x00000002,
    /** Unit is moving and NOT stationary */
    HDW_STATUS_MOTION_SIG_MASK                  = (int)0x00000003,
    /** Gyro motion detected deviation */
    HDW_STATUS_MOTION_GYR_DEV                   = (int)0x00000004,
    /** Accelerometer motion detected deviation */
    HDW_STATUS_MOTION_ACC_DEV                   = (int)0x00000008,
    /** Motion mask */
    HDW_STATUS_MOTION_MASK                      = (int)0x0000000F,

    /** GPS satellite signals are being received (antenna and cable are good) */
    HDW_STATUS_GPS_SATELLITE_RX                 = (int)0x00000010,
    /** Event occurred on strobe input pin */
    HDW_STATUS_STROBE_IN_EVENT                  = (int)0x00000020,
    /** GPS time of week is valid and reported.  Otherwise the timeOfWeek is local system time. */
    HDW_STATUS_GPS_TIME_OF_WEEK_VALID           = (int)0x00000040,
    /** Reference IMU data being received */
    HDW_STATUS_REFERENCE_IMU_RX                 = (int)0x00000080,

    /** Sensor saturation on gyro */
    HDW_STATUS_SATURATION_GYR                   = (int)0x00000100,
    /** Sensor saturation on accelerometer */
    HDW_STATUS_SATURATION_ACC                   = (int)0x00000200,
    /** Sensor saturation on magnetometer */
    HDW_STATUS_SATURATION_MAG                   = (int)0x00000400,
    /** Sensor saturation on barometric pressure */
    HDW_STATUS_SATURATION_BARO                  = (int)0x00000800,

    /** Sensor saturation mask */
    HDW_STATUS_SATURATION_MASK                  = (int)0x00000F00,
    /** Sensor saturation offset */
    HDW_STATUS_SATURATION_OFFSET                = 8,

    /** System Reset is Required for proper function */
    HDW_STATUS_SYSTEM_RESET_REQUIRED            = (int)0x00001000,
    /** Reference IMU used in EKF */
    HDW_STATUS_EKF_USING_REFERENCE_IMU          = (int)0x00002000,
    /** Magnetometer recalibration has finished (when INS_STATUS_MAG_RECALIBRATING is unset).  */
    HDW_STATUS_MAG_RECAL_COMPLETE               = (int)0x00004000,
    /** System flash write staging or occuring now.  Processor will pause and not respond during a flash write, typicaly 150-250 ms. */
    HDW_STATUS_FLASH_WRITE_PENDING              = (int)0x00008000,

    /** Communications Tx buffer limited */
    HDW_STATUS_ERR_COM_TX_LIMITED               = (int)0x00010000,
    /** Communications Rx buffer overrun */
    HDW_STATUS_ERR_COM_RX_OVERRUN               = (int)0x00020000,

    /** GPS PPS timepulse signal has not been received or is in error */
    HDW_STATUS_ERR_NO_GPS_PPS                   = (int)0x00040000,
    /** Time synchronized by GPS PPS */
    HDW_STATUS_GPS_PPS_TIMESYNC                 = (int)0x00080000,

    /** Communications parse error count */
    HDW_STATUS_COM_PARSE_ERR_COUNT_MASK         = (int)0x00F00000,
    HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET       = 20,
#define HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus) ((hdwStatus&HDW_STATUS_COM_PARSE_ERR_COUNT_MASK)>>HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET)

    /** (BIT) Built-in self-test running */
    HDW_STATUS_BIT_RUNNING                      = (int)0x01000000,
    /** (BIT) Built-in self-test passed */
    HDW_STATUS_BIT_PASSED                       = (int)0x02000000,
    /** (BIT) Built-in self-test failure */
    HDW_STATUS_BIT_FAULT                        = (int)0x03000000,
    /** (BIT) Built-in self-test mask */
    HDW_STATUS_BIT_MASK                         = (int)0x03000000,

    /** Temperature outside spec'd operating range */
    HDW_STATUS_ERR_TEMPERATURE                  = (int)0x04000000,
    
    /** IMX pins G5-G8 are configure for SPI use */
    HDW_STATUS_SPI_INTERFACE_ENABLED            = (int)0x08000000,

    /** Fault reset cause */
    HDW_STATUS_FAULT_RESET_MASK                 = (int)0x70000000,    
    /** Reset from Backup mode (low-power state w/ CPU off) */
    HDW_STATUS_FAULT_RESET_BACKUP_MODE          = (int)0x10000000,
    /** Reset from Watchdog */
    HDW_STATUS_FAULT_RESET_WATCHDOG             = (int)0x20000000,
    /** Reset from Software */
    HDW_STATUS_FAULT_RESET_SOFT                 = (int)0x30000000,
    /** Reset from Hardware (NRST pin low) */
    HDW_STATUS_FAULT_RESET_HDW                  = (int)0x40000000,

    /** Critical System Fault - CPU error */
    HDW_STATUS_FAULT_SYS_CRITICAL               = (int)0x80000000,
};

/** System status flags */
enum eSysStatusFlags
{
    /** Allow IMX to drive Testbed-3 status LEDs */
    SYS_STATUS_TBED3_LEDS_ENABLED				= (int)0x00000001,
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
	GPS_STATUS_FLAGS_GPS1_RTK_POSITION_ENABLED      = (int)0x00100000,      // GPS1 RTK precision positioning mode enabled
	GPS_STATUS_FLAGS_STATIC_MODE                    = (int)0x00200000,      // Static mode
	GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED       = (int)0x00400000,      // GPS2 RTK moving base mode enabled
    GPS_STATUS_FLAGS_GPS1_RTK_RAW_GPS_DATA_ERROR    = (int)0x00800000,      // GPS1 RTK error: observations or ephemeris are invalid or not received (i.e. RTK differential corrections)
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_DATA_MISSING     = (int)0x01000000,      // GPS1 RTK error: Either base observations or antenna position have not been received.
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MOVING  = (int)0x02000000,      // GPS1 RTK error: base position moved when it should be stationary
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_INVALID = (int)0x03000000,      // GPS1 RTK error: base position is invalid or not surveyed well
    GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MASK    = (int)0x03000000,      // GPS1 RTK error: base position error bitmask
    GPS_STATUS_FLAGS_ERROR_MASK                     = (GPS_STATUS_FLAGS_GPS1_RTK_RAW_GPS_DATA_ERROR|
                                                       GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MASK),
	GPS_STATUS_FLAGS_GPS1_RTK_POSITION_VALID        = (int)0x04000000,      // GPS1 RTK precision position and carrier phase range solution with fixed ambiguities (i.e. < 6cm horizontal accuracy).  The carrier phase range solution with floating ambiguities occurs if GPS_STATUS_FIX_RTK_FIX is set and GPS_STATUS_FLAGS_GPS1_RTK_POSITION_VALID is not set (i.e. > 6cm horizontal accuracy).
	GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_VALID         = (int)0x08000000,      // GPS2 RTK moving base heading.  Indicates RTK fix and hold with single band RTK compassing.
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_BAD  = (int)0x00002000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_UNSET= (int)0x00004000,
    GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_MASK          = (GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_ENABLED|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_VALID|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_BAD|
                                                       GPS_STATUS_FLAGS_GPS2_RTK_COMPASS_BASELINE_UNSET),
	GPS_STATUS_FLAGS_GPS_NMEA_DATA                  = (int)0x00008000,      // 1 = Data from NMEA message. GPS velocity is NED (not ECEF).
	GPS_STATUS_FLAGS_GPS_PPS_TIMESYNC               = (int)0x10000000,      // Time is synchronized by GPS PPS. 

	GPS_STATUS_FLAGS_MASK                           = (int)0xFFFFE000,    
	GPS_STATUS_FLAGS_BIT_OFFSET                     = (int)16,
	
};

PUSH_PACK_1

/** (DID_POSITION_MEASUREMENT) External position estimate*/
typedef struct PACKED
{
	/** GPS time of week (since Sunday morning) in seconds */
	double					timeOfWeek;

	/** Position in ECEF (earth-centered earth-fixed) frame in meters */
	double					ecef[3];
	
	/** Heading with respect to NED frame (rad)*/
	float 					psi;
	
	/** The Upper Diagonal of accuracy covariance matrix*/
	float					accuracyCovUD[6]; // Matrix accuracyCovUD Described below
	// 0 1 2
	// _ 3 4
	// _ _ 5

}pos_measurement_t;

enum eDevInfoHardware
{
	DEV_INFO_HARDWARE_UNSPECIFIED   = 0,
	DEV_INFO_HARDWARE_UINS          = 1,
	DEV_INFO_HARDWARE_EVB           = 2,
	DEV_INFO_HARDWARE_IMX           = 3,
	DEV_INFO_HARDWARE_GPX           = 4,
};

/** (DID_DEV_INFO) Device information */
typedef struct PACKED
{
	/** Reserved bits */
	uint16_t        reserved;

	/** Hardware: 1=uINS, 2=EVB, 3=IMX, 4=GPX (see eDevInfoHardware) */
	uint16_t        hardware;

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

/** Add missing hardware descriptor to dev_info_t. */
void devInfoPopulateMissingHardware(dev_info_t *devInfo);

/** (DID_MANUFACTURING_INFO) Manufacturing info */
typedef struct PACKED
{
	/** Inertial Sense serial number */
	uint32_t		serialNumber;

	/** Inertial Sense lot number */
	uint32_t		lotNumber;

	/** Inertial Sense manufacturing date (YYYYMMDDHHMMSS) */
    char			date[16];

	/** Key - write: unlock manufacturing info, read: number of times OTP has been set, 15 max */
	uint32_t		key;

	/** Platform / carrier board (ePlatformConfig::PLATFORM_CFG_TYPE_MASK).  Only valid if greater than zero. */
	int32_t			platformType;

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
	
	/** Reserved */
	// IMU_STATUS_RESERVED2						= (int)0x00000400,

//     /** Sensor saturation happened within past 10 seconds */
//     IMU_STATUS_SATURATION_HISTORY               = (int)0x00000100,
//     /** Sample rate fault happened within past 10 seconds */
//     IMU_STATUS_SAMPLE_RATE_FAULT_HISTORY        = (int)0x00000200,

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

/** (DID_GPS1_POS, DID_GPS1_UBX_POS, DID_GPS2_POS) GPS position data */
typedef struct PACKED
{
	/** GPS number of weeks since January 6th, 1980 */
	uint32_t                week;

	/** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;

	/** (see eGpsStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag */
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

} gps_pos_t;


/** (DID_GPS1_VEL, DID_GPS2_VEL) GPS velocity data */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
    uint32_t                timeOfWeekMs;

	/** GPS Velocity.  Velocity is in ECEF {vx,vy,vz} (m/s) if status bit GPS_STATUS_FLAGS_GPS_NMEA_DATA (0x00008000) is NOT set.  Velocity is in local tangent plane with no vertical velocity {vNorth, vEast, 0} (m/s) if status bit GPS_STATUS_FLAGS_GPS_NMEA_DATA (0x00008000) is set. */
	float					vel[3];	

	/** Speed accuracy in meters / second */
	float					sAcc;
	
	/** (see eGpsStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag */
	uint32_t                status;
} gps_vel_t;


/** GPS Satellite information */
typedef struct PACKED
{
    /** GNSS identifier (see eSatSvGnssId) */
    uint8_t					gnssId;

    /** Satellite identifier */
    uint8_t					svId;

    /** (deg) Elevation (range: +/-90) */
    int8_t					elev;

    /** (deg) Azimuth (range: +/-180) */
    int16_t					azim;

    /** (dBHz) Carrier to noise ratio (signal strength) */
    uint8_t					cno;

    /** (see eSatSvStatus) */
    uint16_t				status;

} gps_sat_sv_t;

/** Sat SV - GNSS System ID */
enum eSatSvGnssId
{
    SAT_SV_GNSS_ID_UNKNOWN  = 0,
    SAT_SV_GNSS_ID_GNSS     = 0, 	// (multi-constellation)
    SAT_SV_GNSS_ID_GPS      = 1,	// GPS (USA)
    SAT_SV_GNSS_ID_SBS      = 2,	// SBAS (multiple regional systems, see flash config for selection)
    SAT_SV_GNSS_ID_GAL      = 3,	// Galileo (European Union)	
    SAT_SV_GNSS_ID_BEI      = 4,	// BeiDou (China)
    SAT_SV_GNSS_ID_QZS      = 5,	// QZSS (Japan)
    SAT_SV_GNSS_ID_GLO      = 6,	// GLONASS (Russia)	
    SAT_SV_GNSS_ID_IRN      = 7,	// IRNSS / NavIC (India)	
    SAT_SV_GNSS_ID_IME      = 8,	// IMES (Japan's Indoor Messaging System)
};

/** GPS Sat Status */
enum eSatSvStatus
{
    SAT_SV_STATUS_SIGNAL_QUALITY_MASK               = 0x0007,   // see eSatSigQuality
    SAT_SV_STATUS_USED_IN_SOLUTION                  = 0x0008,	// Used in the solution
    SAT_SV_STATUS_USED_IN_SOLUTION_OFFSET           = 3,
    SAT_SV_STATUS_HEALTH_UNKNOWN                    = 0x0000,	// 0 = unknown
    SAT_SV_STATUS_HEALTH_GOOD                       = 0x0010,	// 1 = healthy
    SAT_SV_STATUS_HEALTH_BAD                        = 0x0020,	// 2 = unhealthy
    SAT_SV_STATUS_HEALTH_MASK                       = 0x0030,
    SAT_SV_STATUS_HEALTH_OFFSET                     = 4,

    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_MASK           = 0x0300,	// 1=float, 2=fix
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_OFFSET         = 8,
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_FLOAT          = 1,	
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_FIX            = 2,	

    // SAT_SV_STATUS_HEALTH_MASK                       = 0x00000030,
    // NAV_SAT_FLAGS_HEALTH_OFFSET                     = 4,
    // SAT_SV_STATUS_DIFFCORR                          = 0x00000040,
    // SAT_SV_STATUS_SMOOTHED                          = 0x00000080,
    // SAT_SV_STATUS_ORBITSOURCE_MASK                  = 0x00000700,
    // SAT_SV_STATUS_ORBITSOURCE_OFFSET                = 8,
    // SAT_SV_STATUS_EPHAVAIL                          = 0x00000800,
    // SAT_SV_STATUS_ALMAVAIL                          = 0x00001000,
    // SAT_SV_STATUS_ANOAVAIL                          = 0x00002000,
    // SAT_SV_STATUS_AOPAVAIL                          = 0x00004000,	
};

/** (DID_GPS1_SAT, DID_GPS2_SAT) GPS satellite information */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;				
    /** Number of satellites in the sky */
	uint32_t				numSats;					
    /** Satellite information list */
	gps_sat_sv_t			sat[MAX_NUM_SATELLITES];	
} gps_sat_t;

enum eSatSvSigId
{
    SAT_SV_SIG_ID_GPS_L1CA			= 0,
    SAT_SV_SIG_ID_GPS_L2CL			= 3,
    SAT_SV_SIG_ID_GPS_L2CM			= 4,
    SAT_SV_SIG_ID_GPS_L5I			= 6,
    SAT_SV_SIG_ID_GPS_L5Q			= 7,
    SAT_SV_SIG_ID_GPS_L5            = SAT_SV_SIG_ID_GPS_L5Q,

    SAT_SV_SIG_ID_SBAS_L1CA			= 0,
    SAT_SV_SIG_ID_SBAS_L2           = 1,
    SAT_SV_SIG_ID_SBAS_L5           = 2,

    SAT_SV_SIG_ID_Galileo_E1C2		= 0,
    SAT_SV_SIG_ID_Galileo_E1B2		= 1,
    SAT_SV_SIG_ID_Galileo_E1BC      = SAT_SV_SIG_ID_Galileo_E1B2,
    SAT_SV_SIG_ID_Galileo_E5aI		= 3,
    SAT_SV_SIG_ID_Galileo_E5aQ		= 4,
    SAT_SV_SIG_ID_Galileo_E5a       = SAT_SV_SIG_ID_Galileo_E5aQ,
    SAT_SV_SIG_ID_Galileo_E5bI		= 5,
    SAT_SV_SIG_ID_Galileo_E5bQ		= 6,
    SAT_SV_SIG_ID_Galileo_E5        = SAT_SV_SIG_ID_Galileo_E5bQ,

    SAT_SV_SIG_ID_BeiDou_B1D1		= 0,
    SAT_SV_SIG_ID_BeiDou_B1D2		= 1,
    SAT_SV_SIG_ID_BeiDou_B2D1		= 2,
    SAT_SV_SIG_ID_BeiDou_B2D2		= 3,
    SAT_SV_SIG_ID_BeiDou_B2         = SAT_SV_SIG_ID_BeiDou_B2D1,
    SAT_SV_SIG_ID_BeiDou_B1C		= 5,
    SAT_SV_SIG_ID_BeiDou_B2a		= 7,

    SAT_SV_SIG_ID_QZSS_L1CA			= 0,
    SAT_SV_SIG_ID_QZSS_L1S			= 1,
    SAT_SV_SIG_ID_QZSS_L2CM			= 4,
    SAT_SV_SIG_ID_QZSS_L2CL 		= 5,
    SAT_SV_SIG_ID_QZSS_L2           = SAT_SV_SIG_ID_QZSS_L2CL,
    SAT_SV_SIG_ID_QZSS_L5I 			= 8,
    SAT_SV_SIG_ID_QZSS_L5Q 			= 9,
    SAT_SV_SIG_ID_QZSS_L5           = SAT_SV_SIG_ID_QZSS_L5Q,

    SAT_SV_SIG_ID_GLONASS_L1OF		= 0,
    SAT_SV_SIG_ID_GLONASS_L2OF		= 2,

    SAT_SV_SIG_ID_NAVIC_L5A			= 0, 
};

enum eSatSigQuality
{
    SAT_SIG_QUALITY_NO_SIGNAL                   = 0, 	// no signal
    SAT_SIG_QUALITY_SEARCHING                   = 1, 	// searching signal
    SAT_SIG_QUALITY_ACQUIRED                    = 2, 	// signal acquired
    SAT_SIG_QUALITY_DETECTED                    = 3, 	// signal detected but unusable
    SAT_SIG_QUALITY_CODE_LOCK_TIME_SYNC         = 4, 	// code locked and time synchronized
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_1    = 5, 	// code and carrier locked and time synchronized
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_2    = 6, 	// "
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_3    = 7, 	// "
};

enum eSatSigStatus
{
    SAT_SIG_STATUS_HEALTH_UNKNOWN                    = 0x0000,	// 0 = unknown
    SAT_SIG_STATUS_HEALTH_GOOD                       = 0x0001,	// 1 = healthy
    SAT_SIG_STATUS_HEALTH_BAD                        = 0x0002,	// 2 = unhealthy
    SAT_SIG_STATUS_HEALTH_MASK                       = 0x0003,
    SAT_SIG_STATUS_USED_IN_SOLUTION                  = 0x0004,  // Signal is used in the solution
    SAT_SIG_STATUS_USED_IN_SOLUTION_OFFSET           = 2,
};


/** GPS satellite signal information */
typedef struct PACKED
{
    /** GNSS identifier (see eSatSvGnssId) */
    uint8_t					gnssId;

    /** Satellite identifier */
    uint8_t					svId;

    /** Signal identifier, frequency description (eSatSvSigId) */
    uint8_t					sigId;

    /** (dBHz) Carrier to noise ratio (signal strength) */
    uint8_t					cno;

    /** Quality indicator (see eSatSigQuality) */
    uint8_t					quality;

    /** Status flags (see eSatSigStatus) */
    uint16_t				status;

} gps_sig_sv_t;

/** (DID_GPS1_SIG, DID_GPS2_SIG) GPS satellite signal information */
typedef struct PACKED
{
    /** GPS time of week (since Sunday morning) in milliseconds */
	uint32_t                timeOfWeekMs;				
    /** Number of satellite signals in the following satelliate signal list */
	uint32_t				numSigs;					
    /** Satellite signal list */
	gps_sig_sv_t			sig[MAX_NUM_SAT_SIGNALS];	
} gps_sig_t;

typedef uint8_t         gps_extension_ver_t[30];
#define GPS_VER_NUM_EXTENSIONS	6
/** (DID_GPS1_VERSION) GPS version strings */
typedef struct PACKED
{
    /** Software version */
	uint8_t                 swVersion[30];
    /** Hardware version */
    uint8_t                 hwVersion[10];		
    /** Extension 30 bytes array description  */
	gps_extension_ver_t     extension[GPS_VER_NUM_EXTENSIONS];		
} gps_version_t;

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
	int						ahrs_gps_cnt;		// Counter of sequential valid GPS data (for switching from AHRS to navigation)
	float					hdg_err;
	int						hdg_coarse;			// Flag whether initial attitude error converged
	int						hdg_aligned;		// Flag whether initial attitude error converged
	int						hdg_aligning;
	int						ekf_init_done;	    // Hot EKF initialization completed
	int						mag_cal_good;
	int						mag_cal_done;
	int						stat_magfield;
} inl2_status_t;

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

	/** IMU sample period (ms). Zero disables sampling. */
	uint32_t				imuSamplePeriodMs;

	/** Preintegrated IMU (PIMU) integration period and navigation/AHRS filter output period (ms). */
	uint32_t				navOutputPeriodMs;
	
    /** Actual sample period relative to GPS PPS (sec) */
	double					sensorTruePeriod;

	/** Flash config checksum used with host SDK synchronization */
	uint32_t				flashCfgChecksum;

	/** Navigation/AHRS filter update period (ms) */
	uint32_t				navUpdatePeriodMs;

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
	GFC_INIT_GPS1						= 0x00000800,
	/*! Fault: GPS2 init  */
	GFC_INIT_GPS2                       = 0x00001000,
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
    SYS_CMD_NONE                                        = 0,            // (uint32 inv: 4294967295)
    SYS_CMD_SAVE_PERSISTENT_MESSAGES                    = 1,            // (uint32 inv: 4294967294)
    SYS_CMD_ENABLE_BOOTLOADER_AND_RESET                 = 2,            // (uint32 inv: 4294967293)
    SYS_CMD_ENABLE_SENSOR_STATS                         = 3,            // (uint32 inv: 4294967292)
    SYS_CMD_ENABLE_RTOS_STATS                           = 4,            // (uint32 inv: 4294967291)
    SYS_CMD_ZERO_MOTION                                 = 5,            // (uint32 inv: 4294967290)
    SYS_CMD_REF_POINT_STATIONARY                        = 6,            // (uint32 inv: 4294967289)
    SYS_CMD_REF_POINT_MOVING                            = 7,            // (uint32 inv: 4294967288)
    SYS_CMD_RESET_RTOS_STATS                            = 8,            // (uint32 inv: 4294967287)

    SYS_CMD_ENABLE_GPS_LOW_LEVEL_CONFIG                 = 10,           // (uint32 inv: 4294967285)
    SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE                  = 11,           // (uint32 inv: 4294967284)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS1       = 12,           // (uint32 inv: 4294967283)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS2       = 13,           // (uint32 inv: 4294967282)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER0       = 14,           // (uint32 inv: 4294967281)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER1       = 15,           // (uint32 inv: 4294967280)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER2       = 16,           // (uint32 inv: 4294967279)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_TO_GPS1      = 17,           // (uint32 inv: 4294967278)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GPS1  = 18,           // (uint32 inv: 4294967277)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GPS2  = 19,           // (uint32 inv: 4294967276)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_USB   = 20,           // (uint32 inv: 4294967275)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER0  = 21,           // (uint32 inv: 4294967274)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER1  = 22,           // (uint32 inv: 4294967273)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER2  = 23,           // (uint32 inv: 4294967272)

    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_LOOPBACK      = 24,           // (uint32 inv: 4294967271)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_LOOPBACK     = 25,           // (uint32 inv: 4294967270)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER1_LOOPBACK     = 26,           // (uint32 inv: 4294967269)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER2_LOOPBACK     = 27,           // (uint32 inv: 4294967268)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK = 28,           // (uint32 inv: 4294967267)

    SYS_CMD_SAVE_FLASH                                  = 97,           // (uint32 inv: 4294967198)
    SYS_CMD_SAVE_GPS_ASSIST_TO_FLASH_RESET              = 98,           // (uint32 inv: 4294967197)
    SYS_CMD_SOFTWARE_RESET                              = 99,           // (uint32 inv: 4294967196)
    SYS_CMD_MANF_UNLOCK                                 = 1122334455,   // (uint32 inv: 3172632840)
    SYS_CMD_MANF_FACTORY_RESET                          = 1357924680,   // (uint32 inv: 2937042615) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.
    SYS_CMD_MANF_CHIP_ERASE                             = 1357924681,   // (uint32 inv: 2937042614) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.  A device power cycle may be necessary to complete this command.
    SYS_CMD_MANF_DOWNGRADE_CALIBRATION                  = 1357924682,   // (uint32 inv: 2937042613) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.
    SYS_CMD_MANF_ENABLE_ROM_BOOTLOADER                  = 1357924683,   // (uint32 inv: 2937042612) SYS_CMD_MANF_RESET_UNLOCK must be sent prior to this command.  A device power cycle may be necessary to complete this command.
};

enum eSerialPortBridge
{
	SERIAL_PORT_BRIDGE_DISABLED         = 0,

    SERIAL_PORT_BRIDGE_GPS1_TO_USB      = 1,
    SERIAL_PORT_BRIDGE_GPS1_TO_SER0     = 2,
    SERIAL_PORT_BRIDGE_GPS1_TO_SER1     = 3,
    SERIAL_PORT_BRIDGE_GPS1_TO_SER2     = 4,

    SERIAL_PORT_BRIDGE_GPS2_TO_USB      = 5,
    SERIAL_PORT_BRIDGE_GPS2_TO_SER0     = 6,
    SERIAL_PORT_BRIDGE_GPS2_TO_SER1     = 7,
    SERIAL_PORT_BRIDGE_GPS2_TO_SER2     = 8,

    SERIAL_PORT_BRIDGE_USB_TO_SER0      = 9,
    SERIAL_PORT_BRIDGE_USB_TO_SER1      = 10,
    SERIAL_PORT_BRIDGE_USB_TO_SER2      = 11,
    SERIAL_PORT_BRIDGE_SER0_TO_SER1     = 12,
    SERIAL_PORT_BRIDGE_SER0_TO_SER2     = 13,
    SERIAL_PORT_BRIDGE_SER1_TO_SER2     = 14,

    SERIAL_PORT_BRIDGE_USB_TO_USB       = 15,   // loopback
    SERIAL_PORT_BRIDGE_SER0_TO_SER0     = 16,   // loopback
    SERIAL_PORT_BRIDGE_SER1_TO_SER1     = 17,   // loopback
    SERIAL_PORT_BRIDGE_SER2_TO_SER2     = 18,   // loopback
};

#define NMEA_BUFFER_SIZE 256

/** (DID_NMEA_BCAST_PERIOD) Set NMEA message broadcast periods. This data structure is zeroed out on stop_all_broadcasts */
typedef struct PACKED
{
	/** Options: Port selection[0x0=current, 0xFF=all, 0x1=ser0, 0x2=ser1, 0x4=ser2, 0x8=USB] (see RMC_OPTIONS_...) */
	uint32_t				options;

	/** Broadcast period multiple - NMEA IMU data. 0 to disable. */
	uint16_t				pimu;

	/** Broadcast period multiple - NMEA preintegrated IMU: delta theta (rad) and delta velocity (m/s). 0 to disable. */
	uint16_t				ppimu;
	
	/** Broadcast period multiple - NMEA INS output: euler rotation w/ respect to NED, NED position from reference LLA. 0 to disable. */
	uint16_t				pins1;

	/** Broadcast period multiple - NMEA INS output: quaternion rotation w/ respect to NED, ellipsoid altitude. 0 to disable. */
	uint16_t				pins2;
	
	/** Broadcast period multiple - NMEA GPS position data. 0 to disable. */
	uint16_t				pgpsp;

	/** Broadcast period multiple - NMEA Raw IMU data (up to 1KHz).  Use this IMU data for output data rates faster than DID_FLASH_CONFIG.startupNavDtMs.  Otherwise we recommend use of pimu or ppimu as they are oversampled and contain less noise. 0 to disable. */
	uint16_t				primu;

	/** Broadcast period multiple - NMEA standard GGA GNSS 3D location, fix, and accuracy. 0 to disable. */
	uint16_t				gga;

	/** Broadcast period multiple - NMEA standard GLL GNSS 2D location and time. 0 to disable. */
	uint16_t				gll;

	/** Broadcast period multiple - NMEA standard GSA GNSS DOP and active satellites. 0 to disable. */
	uint16_t				gsa;

	/** Broadcast period multiple - NMEA standard recommended minimum specific GPS/Transit data. 0 to disable. */
	uint16_t				rmc;
	
	/** Broadcast period multiple - NMEA standard Data and Time. 0 to disable. */
	uint16_t				zda;

	/** Broadcast period multiple - NMEA standard Inertial Attitude Data. 0 to disable. */
	uint16_t				pashr;

	/** Broadcast period multiple - NMEA standard satelliate information. */
	uint16_t				gsv;

	/** Broadcast period multiple - NMEA track made good and speed over ground. */
	uint16_t				vtg;

} nmea_msgs_t;

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

/** Realtime Message Controller (used in rmc_t). 
	The data sets available through RMC are broadcast at the availability of the data.  A goal of RMC is 
	to provide updates from each onboard sensor as fast as possible with minimal latency.  The RMC is 
	provided so that broadcast of sensor data is done as soon as it becomes available.   The exception to
	this rule is the INS output data, which has a configurable output data rate according to DID_RMC.insPeriodMs.
*/

#define RMC_OPTIONS_PORT_MASK           0x000000FF
#define RMC_OPTIONS_PORT_ALL            (RMC_OPTIONS_PORT_MASK)
#define RMC_OPTIONS_PORT_CURRENT        0x00000000
#define RMC_OPTIONS_PORT_SER0           0x00000001
#define RMC_OPTIONS_PORT_SER1           0x00000002	// also SPI
#define RMC_OPTIONS_PORT_SER2           0x00000004
#define RMC_OPTIONS_PORT_USB            0x00000008
#define RMC_OPTIONS_PRESERVE_CTRL       0x00000100	// Prevent any messages from getting turned off by bitwise OR'ing new message bits with current message bits.
#define RMC_OPTIONS_PERSISTENT          0x00000200	// Save current port RMC to flash memory for use following reboot, eliminating need to re-enable RMC to start data streaming.  

// RMC message data rates:
#define RMC_BITS_INS1                   0x0000000000000001      // rmc.insPeriodMs (4ms default)
#define RMC_BITS_INS2                   0x0000000000000002      // "
#define RMC_BITS_INS3                   0x0000000000000004      // "
#define RMC_BITS_INS4                   0x0000000000000008      // "
#define RMC_BITS_IMU                    0x0000000000000010      // DID_FLASH_CONFIG.startupNavDtMs (4ms default)
#define RMC_BITS_PIMU                   0x0000000000000020      // "
#define RMC_BITS_BAROMETER              0x0000000000000040      // ~8ms
#define RMC_BITS_MAGNETOMETER           0x0000000000000080      // ~10ms
// #define RMC_BITS_UNUSED              0x0000000000000100
// #define RMC_BITS_UNUSED              0x0000000000000200 
#define RMC_BITS_GPS1_POS               0x0000000000000400      // DID_FLASH_CONFIG.startupGpsDtMs (200ms default)
#define RMC_BITS_GPS2_POS               0x0000000000000800      // "
#define RMC_BITS_GPS1_RAW               0x0000000000001000      // "
#define RMC_BITS_GPS2_RAW               0x0000000000002000      // "
#define RMC_BITS_GPS1_SAT               0x0000000000004000      // 1s
#define RMC_BITS_GPS2_SAT               0x0000000000008000      // "
#define RMC_BITS_GPS_BASE_RAW           0x0000000000010000      // 
#define RMC_BITS_STROBE_IN_TIME         0x0000000000020000      // On strobe input event
#define RMC_BITS_DIAGNOSTIC_MESSAGE     0x0000000000040000
#define RMC_BITS_IMU3_UNCAL             0x0000000000080000      // DID_FLASH_CONFIG.startupImuDtMs (1ms default)
#define RMC_BITS_GPS1_VEL               0x0000000000100000      // DID_FLASH_CONFIG.startupGpsDtMs (200ms default)
#define RMC_BITS_GPS2_VEL               0x0000000000200000      // "
#define RMC_BITS_GPS1_UBX_POS           0x0000000000400000      // "
#define RMC_BITS_GPS1_RTK_POS           0x0000000000800000      // "
#define RMC_BITS_GPS1_RTK_POS_REL       0x0000000001000000      // "
#define RMC_BITS_GPS1_RTK_POS_MISC      0x0000000004000000      // "
#define RMC_BITS_INL2_NED_SIGMA         0x0000000008000000
#define RMC_BITS_RTK_STATE              0x0000000010000000
#define RMC_BITS_RTK_CODE_RESIDUAL      0x0000000020000000
#define RMC_BITS_RTK_PHASE_RESIDUAL     0x0000000040000000
#define RMC_BITS_WHEEL_ENCODER          0x0000000080000000
#define RMC_BITS_GROUND_VEHICLE         0x0000000100000000
// #define RMC_BITS_UNUSED              0x0000000200000000
#define RMC_BITS_IMU_MAG                0x0000000400000000
#define RMC_BITS_PIMU_MAG               0x0000000800000000
#define RMC_BITS_GPS1_RTK_HDG_REL       0x0000001000000000      // DID_FLASH_CONFIG.startupGpsDtMs (200ms default)
#define RMC_BITS_GPS1_RTK_HDG_MISC      0x0000002000000000      // "
#define RMC_BITS_REFERENCE_IMU          0x0000004000000000		// DID_FLASH_CONFIG.startupNavDtMs
#define RMC_BITS_REFERENCE_PIMU         0x0000008000000000		// "
#define RMC_BITS_IMU3_RAW               0x0000010000000000
#define RMC_BITS_IMU_RAW                0x0000020000000000
#define RMC_BITS_GPS1_SIG               0x0000040000000000      // 1s
#define RMC_BITS_GPS2_SIG               0x0000080000000000      // "

#define RMC_BITS_MASK                   0x0FFFFFFFFFFFFFFF
#define RMC_BITS_INTERNAL_PPD           0x4000000000000000      // 
#define RMC_BITS_PRESET                 0x8000000000000000		// Indicate BITS is a preset.  This sets the rmc period multiple and enables broadcasting.

#define RMC_PRESET_PPD_NAV_PERIOD_MULT_MS	100

// Preset: Post Processing Data
#define RMC_PRESET_PPD_BITS_NO_IMU		(RMC_BITS_PRESET \
										| RMC_BITS_INS2 \
										| RMC_BITS_BAROMETER \
										| RMC_BITS_MAGNETOMETER \
										| RMC_BITS_GPS1_POS \
										| RMC_BITS_GPS2_POS \
										| RMC_BITS_GPS1_VEL \
										| RMC_BITS_GPS2_VEL \
										| RMC_BITS_GPS1_RAW \
										| RMC_BITS_GPS2_RAW \
										| RMC_BITS_GPS_BASE_RAW \
										| RMC_BITS_GPS1_RTK_POS_REL \
										| RMC_BITS_GPS1_RTK_HDG_REL \
										| RMC_BITS_INTERNAL_PPD \
										| RMC_BITS_DIAGNOSTIC_MESSAGE)
#define RMC_PRESET_PPD_BITS				(RMC_PRESET_PPD_BITS_NO_IMU \
										| RMC_BITS_PIMU \
										| RMC_BITS_REFERENCE_PIMU)
#define RMC_PRESET_INS_BITS				(RMC_BITS_INS2 \
										| RMC_BITS_GPS1_POS \
										| RMC_BITS_PRESET)
#define RMC_PRESET_PPD_BITS_IMU3		(RMC_PRESET_PPD_BITS_NO_IMU \
										| RMC_BITS_IMU3_UNCAL)
#define RMC_PRESET_PPD_BITS_RTK_DBG		(RMC_PRESET_PPD_BITS \
										| RMC_BITS_RTK_STATE \
										| RMC_BITS_RTK_CODE_RESIDUAL \
										| RMC_BITS_RTK_PHASE_RESIDUAL)
#define RMC_PRESET_PPD_GROUND_VEHICLE	(RMC_PRESET_PPD_BITS \
										| RMC_BITS_WHEEL_ENCODER \
										| RMC_BITS_GROUND_VEHICLE)
#define RMC_PRESET_ALLAN_VARIANCE		(RMC_BITS_PRESET \
										| RMC_BITS_IMU)

/** (DID_RMC) Realtime message controller (RMC). */
typedef struct PACKED
{
	/** Data stream enable bits for the specified ports.  (see RMC_BITS_...) */
	uint64_t                bits;

	/** Options to select alternate ports to output data, etc.  (see RMC_OPTIONS_...) */
	uint32_t				options;
	
	/** IMU and Integrated IMU data transmit period is set using DID_SYS_PARAMS.navOutputPeriodMs */
} rmc_t;

enum eNmeaAsciiMsgId
{
    NMEA_MSG_ID_PIMU      = 0,
    NMEA_MSG_ID_PPIMU     = 1,
    NMEA_MSG_ID_PRIMU     = 2,
    NMEA_MSG_ID_PINS1     = 3,
    NMEA_MSG_ID_PINS2     = 4,
    NMEA_MSG_ID_PGPSP     = 5,
    NMEA_MSG_ID_GxGGA     = 6,
    NMEA_MSG_ID_GxGLL     = 7,
    NMEA_MSG_ID_GxGSA     = 8,
    NMEA_MSG_ID_GxRMC     = 9,
    NMEA_MSG_ID_GxZDA     = 10,
    NMEA_MSG_ID_PASHR     = 11, 
    NMEA_MSG_ID_PSTRB     = 12,
    NMEA_MSG_ID_INFO      = 13,
    NMEA_MSG_ID_GxGSV     = 14,
    NMEA_MSG_ID_GxVTG     = 15,
    NMEA_MSG_ID_INTEL     = 16,
    NMEA_MSG_ID_COUNT,

	// IMX/GPX Input Commands
    NMEA_MSG_ID_ASCB,         // "ASCB" - NMEA messages broadcast periods
    NMEA_MSG_ID_ASCE,         // "ASCE" - NMEA messages broadcast enable
    NMEA_MSG_ID_BLEN,         // "BLEN" - Enable bootloader on IMX (app firmware update)	
    NMEA_MSG_ID_EBLE,         // "EBLE" - Enable bootloader on EVB
    NMEA_MSG_ID_NELB,         // "NELB" - Enable SAM-BA mode	
    NMEA_MSG_ID_PERS,         // "PERS" - Save perstent messages
    NMEA_MSG_ID_SRST,         // "SRTS" - Software reset
    NMEA_MSG_ID_STPB,         // "STPB" - Stop broadcasts on all ports
    NMEA_MSG_ID_STPC,         // "STPC" - Stop broadcasts on current port
}; 

#define NMEA_RMC_BITS_PIMU          (1<<NMEA_MSG_ID_PIMU)
#define NMEA_RMC_BITS_PPIMU         (1<<NMEA_MSG_ID_PPIMU)
#define NMEA_RMC_BITS_PRIMU         (1<<NMEA_MSG_ID_PRIMU)
#define NMEA_RMC_BITS_PINS1         (1<<NMEA_MSG_ID_PINS1)
#define NMEA_RMC_BITS_PINS2         (1<<NMEA_MSG_ID_PINS2)
#define NMEA_RMC_BITS_PGPSP         (1<<NMEA_MSG_ID_PGPSP)
#define NMEA_RMC_BITS_GxGGA         (1<<NMEA_MSG_ID_GxGGA)
#define NMEA_RMC_BITS_GxGLL         (1<<NMEA_MSG_ID_GxGLL)
#define NMEA_RMC_BITS_GxGSA         (1<<NMEA_MSG_ID_GxGSA)
#define NMEA_RMC_BITS_GxRMC         (1<<NMEA_MSG_ID_GxRMC)
#define NMEA_RMC_BITS_GxZDA         (1<<NMEA_MSG_ID_GxZDA)
#define NMEA_RMC_BITS_PASHR         (1<<NMEA_MSG_ID_PASHR)
#define NMEA_RMC_BITS_PSTRB         (1<<NMEA_MSG_ID_PSTRB)
#define NMEA_RMC_BITS_INFO          (1<<NMEA_MSG_ID_INFO)
#define NMEA_RMC_BITS_GxGSV         (1<<NMEA_MSG_ID_GxGSV)
#define NMEA_RMC_BITS_GxVTG         (1<<NMEA_MSG_ID_GxVTG)
#define NMEA_RMC_BITS_INTEL         (1<<NMEA_MSG_ID_INTEL)

typedef struct PACKED
{
     /** Data stream enable bits for the specified ports.  (see RMC_BITS_...) */
    uint32_t                nmeaBits;

    /** NMEA period multiple of above ISB period multiple indexed by NMEA_MSG_ID... */
    uint8_t                 nmeaPeriod[NMEA_MSG_ID_COUNT];
}rmcNmea_t;

/** Realtime message controller internal (RMCI). */
typedef struct PACKED
{
     /** Data stream enable bits and options for the specified ports.  (see RMC_BITS_...) */
    rmc_t                   rmc;
    
    /** Used for both the DID binary and NMEA messages.  */
    uint8_t                 periodMultiple[DID_COUNT_UINS];

    rmcNmea_t               rmcNmea;

} rmci_t;


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
	/** Mag recalibration state.  COMMANDS: 1=multi-axis, 2=single-axis, 101=abort, STATUS: 200=running, 201=done (see eMagCalState) */
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
#define HDW_BIT_MODE(hdwBitStatus) (((hdwBitStatus)&HDW_BIT_MODE_MASK)>>HDW_BIT_MODE_OFFSET)
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
#define CAL_BIT_MODE(calBitStatus) (((calBitStatus)&CAL_BIT_MODE_MASK)>>CAL_BIT_MODE_OFFSET)
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


/** System Configuration (used with DID_FLASH_CONFIG.sysCfgBits) */
enum eSysConfigBits
{
	UNUSED1                                             = (int)0x00000001,
	/*! Enable mag continuous calibration.  Allow slow background magnetometer calibration in the EKF. */
	SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL              = (int)0x00000002,
	/*! Enable automatic mag recalibration */
	SYS_CFG_BITS_AUTO_MAG_RECAL                         = (int)0x00000004,
	/*! Disable mag declination estimation */
	SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION            = (int)0x00000008,

	/*! Disable LEDs */
	SYS_CFG_BITS_DISABLE_LEDS                           = (int)0x00000010,

	/** Magnetometer recalibration.  (see eMagCalState) 1 = multi-axis, 2 = single-axis */
	SYS_CFG_BITS_MAG_RECAL_MODE_MASK					= (int)0x00000700,
	SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET					= 8,
#define SYS_CFG_BITS_MAG_RECAL_MODE(sysCfgBits) ((sysCfgBits&SYS_CFG_BITS_MAG_RECAL_MODE_MASK)>>SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET)

	// When set WMM will be used to set declanation
	SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION				= (int)0x00000800,

	/** Disable magnetometer fusion */
	SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION			= (int)0x00001000,
	/** Disable barometer fusion */
	SYS_CFG_BITS_DISABLE_BAROMETER_FUSION				= (int)0x00002000,
	/** Disable GPS 1 fusion */
	SYS_CFG_BITS_DISABLE_GPS1_FUSION					= (int)0x00004000,
	/** Disable GPS 2 fusion */
	SYS_CFG_BITS_DISABLE_GPS2_FUSION					= (int)0x00008000,

	/** Disable automatic Zero Velocity Updates (ZUPT).  Disabling automatic ZUPT is useful for degraded GPS environments or applications with very slow velocities. */
	SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES		= (int)0x00010000,
	/** Disable automatic Zero Angular Rate Updates (ZARU).  Disabling automatic ZARU is useful for applications with small/slow angular rates. */
	SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES	= (int)0x00020000,
	/** Disable INS EKF updates */
	SYS_CFG_BITS_DISABLE_INS_EKF						= (int)0x00040000,
	/** Prevent built-in test (BIT) from running automatically on startup */
	SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP			= (int)0x00080000,

	/** Disable wheel encoder fusion */
	SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION			= (int)0x00100000,
	/** Disable packet encoding, binary data will have all bytes as is */
	SYS_CFG_BITS_DISABLE_PACKET_ENCODING				= (int)0x00400000,

	/** Use reference IMU in EKF instead of onboard IMU */
	SYS_CFG_USE_REFERENCE_IMU_IN_EKF					= (int)0x01000000,
	/** Reference point stationary on strobe input */
	SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT	= (int)0x02000000,

};

/** GNSS satellite system signal constellation (used with nvm_flash_cfg_t.gnssSatSigConst) */
enum eGnssSatSigConst
{
	/*! GPS  */
	GNSS_SAT_SIG_CONST_GPS                              = (uint16_t)0x0003,
	/*! QZSS  */
	GNSS_SAT_SIG_CONST_QZS                              = (uint16_t)0x000C,
	/*! Galileo  */
	GNSS_SAT_SIG_CONST_GAL                              = (uint16_t)0x0030,
	/*! BeiDou  */
	GNSS_SAT_SIG_CONST_BDS                              = (uint16_t)0x00C0,
	/*! GLONASS  */
	GNSS_SAT_SIG_CONST_GLO                              = (uint16_t)0x0300,
	/*! SBAS  */
	GNSS_SAT_SIG_CONST_SBS                              = (uint16_t)0x1000,
	/*! IRNSS / NavIC  */
	GNSS_SAT_SIG_CONST_IRN                              = (uint16_t)0x2000,
	/*! IMES  */
	GNSS_SAT_SIG_CONST_IME                              = (uint16_t)0x4000,

	/*! GNSS default */
	GNSS_SAT_SIG_CONST_DEFAULT = \
		GNSS_SAT_SIG_CONST_GPS | \
		GNSS_SAT_SIG_CONST_SBS | \
		GNSS_SAT_SIG_CONST_QZS | \
		GNSS_SAT_SIG_CONST_GAL | \
		GNSS_SAT_SIG_CONST_GLO | \
		GNSS_SAT_SIG_CONST_BDS
};

/** RTK Configuration (used with nvm_flash_cfg_t.RTKCfgBits) */
enum eRTKConfigBits
{
	/** Enable onboard RTK GNSS precision positioning (GPS1) */
	RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING				= (int)0x00000001,

	/** Enable external RTK GNSS positioning (GPS1) */
	RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL	= (int)0x00000002,

	/** Enable external RTK GNSS compassing on uBlox F9P (GPS2) */
	RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_F9P			= (int)0x00000004,

	/** Enable dual GNSS RTK compassing (GPS2 to GPS1) */
	RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING				= (int)0x00000008,	

	/** Mask of RTK GNSS positioning types */
	RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_MASK		= (RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING|RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_EXTERNAL),

	/** Mask of dual GNSS RTK compassing types */
	RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_MASK			= (RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING|RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_F9P),

    /** Mask of RTK position, heading, and base modes */
	RTK_CFG_BITS_ROVER_MODE_MASK						= (int)0x0000000F,
	
	/** Enable RTK base and output ublox data from GPS 1 on serial port 0 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0			= (int)0x00000010,

	/** Enable RTK base and output ublox data from GPS 1 on serial port 1 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1			= (int)0x00000020,

	/** Enable RTK base and output ublox data from GPS 1 on serial port 2 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER2			= (int)0x00000040,

	/** Enable RTK base and output ublox data from GPS 1 on USB port */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB				= (int)0x00000080,

	/** Enable RTK base and output RTCM3 data from GPS 1 on serial port 0 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0			= (int)0x00000100,
	
	/** Enable RTK base and output RTCM3 data from GPS 1 on serial port 1 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1			= (int)0x00000200,

	/** Enable RTK base and output RTCM3 data from GPS 1 on serial port 2 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2			= (int)0x00000400,

	/** Enable RTK base and output RTCM3 data from GPS 1 on USB port */
	RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB				= (int)0x00000800,

	/** Enable RTK base and output ublox data from GPS 2 on serial port 0 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0			= (int)0x00001000,

	/** Enable RTK base and output ublox data from GPS 2 on serial port 1 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1			= (int)0x00002000,

	/** Enable RTK base and output ublox data from GPS 2 on serial port 2 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER2			= (int)0x00004000,

	/** Enable RTK base and output ublox data from GPS 2 on USB port */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_USB				= (int)0x00008000,

	/** Enable RTK base and output RTCM3 data from GPS 2 on serial port 0 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0			= (int)0x00010000,
	
	/** Enable RTK base and output RTCM3 data from GPS 2 on serial port 1 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1			= (int)0x00020000,

	/** Enable RTK base and output RTCM3 data from GPS 2 on serial port 2 */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER2			= (int)0x00040000,

	/** Enable RTK base and output RTCM3 data from GPS 2 on USB port */
	RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_USB				= (int)0x00080000,

	/** Enable base mode moving position. (For future use. Not implemented. This bit should always be 0 for now.) TODO: Implement moving base. */
	RTK_CFG_BITS_BASE_POS_MOVING						= (int)0x00100000,
	
	/** Reserved for future use */
	RTK_CFG_BITS_RESERVED1								= (int)0x00200000,	
	
	/** When using RTK, specifies whether the base station is identical hardware to this rover. If so, there are optimizations enabled to get fix faster. */
	RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER			= (int)0x00400000,

	/** Forward all messages between the selected GPS and serial port.  Disable for RTK base use (to forward only GPS raw messages and use the surveyed location refLLA instead of current GPS position).  */
	RTK_CFG_BITS_GPS_PORT_PASS_THROUGH					= (int)0x00800000,

	/** All base station bits */
	RTK_CFG_BITS_BASE_MODE = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER2 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB  | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB  |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER2 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER2 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_USB  | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_USB ),

	/** Base station bits enabled on Ser0 */
	RTK_CFG_BITS_RTK_BASE_SER0 = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0 ),

	/** Base station bits enabled on Ser1 */
	RTK_CFG_BITS_RTK_BASE_SER1 = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1 ),

	/** Base station bits enabled on Ser2 */
	RTK_CFG_BITS_RTK_BASE_SER2 = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER2 | RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2 |
		RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER2 | RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER2 ),

    /** Base station bits for GPS1 Ublox */
    RTK_CFG_BITS_RTK_BASE_OUTPUT_GPS1_UBLOX = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_UBLOX_USB ),

    /** Base station bits for GPS2 Ublox */
    RTK_CFG_BITS_RTK_BASE_OUTPUT_GPS2_UBLOX = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_UBLOX_USB ),

    /** Base station bits for GPS1 RTCM */
	RTK_CFG_BITS_RTK_BASE_OUTPUT_GPS1_RTCM = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER1 | 
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_SER2 | 
        RTK_CFG_BITS_BASE_OUTPUT_GPS1_RTCM3_USB ),

    /** Base station bits for GPS2 RTCM */
    RTK_CFG_BITS_RTK_BASE_OUTPUT_GPS2_RTCM = (
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GPS2_RTCM3_USB),

	/** Rover on-board RTK engine used */
	RTK_CFG_BITS_ROVER_MODE_ONBOARD_MASK = (RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING | RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING),

	/** Mask of Rover, Compassing, and Base modes */
	RTK_CFG_BITS_ALL_MODES_MASK = (RTK_CFG_BITS_ROVER_MODE_MASK | RTK_CFG_BITS_BASE_MODE),	
};

/** Sensor Configuration (used with nvm_flash_cfg_t.sensorConfig) */
enum eSensorConfig
{
	/** Gyro full-scale sensing range selection: +- 250, 500, 1000, 2000, 4000 deg/s */	
	SENSOR_CFG_GYR_FS_250				= (int)0x00000000,
	SENSOR_CFG_GYR_FS_500				= (int)0x00000001,
	SENSOR_CFG_GYR_FS_1000				= (int)0x00000002,
	SENSOR_CFG_GYR_FS_2000				= (int)0x00000003,
	SENSOR_CFG_GYR_FS_4000				= (int)0x00000004,
	SENSOR_CFG_GYR_FS_MASK				= (int)0x00000007,
	SENSOR_CFG_GYR_FS_OFFSET			= (int)0,
	
	/** Accelerometer full-scale sensing range selection: +- 2, 4, 8, 16 m/s^2 */
	SENSOR_CFG_ACC_FS_2G				= (int)0x00000000,
	SENSOR_CFG_ACC_FS_4G				= (int)0x00000001,
	SENSOR_CFG_ACC_FS_8G				= (int)0x00000002,
	SENSOR_CFG_ACC_FS_16G				= (int)0x00000003,
	SENSOR_CFG_ACC_FS_MASK				= (int)0x00000030,
	SENSOR_CFG_ACC_FS_OFFSET			= (int)4,
	
	/** Gyro digital low-pass filter (DLPF) is set automatically based on the IMU sample rate.  The following 
	bit values can be used to override the bandwidth (frequency) to: 250, 184, 92, 41, 20, 10, 5 Hz */
	SENSOR_CFG_GYR_DLPF_250HZ			= (int)0x00000000,
	SENSOR_CFG_GYR_DLPF_184HZ			= (int)0x00000001,
	SENSOR_CFG_GYR_DLPF_92HZ			= (int)0x00000002,
	SENSOR_CFG_GYR_DLPF_41HZ			= (int)0x00000003,
	SENSOR_CFG_GYR_DLPF_20HZ			= (int)0x00000004,
	SENSOR_CFG_GYR_DLPF_10HZ			= (int)0x00000005,
	SENSOR_CFG_GYR_DLPF_5HZ				= (int)0x00000006,
 	SENSOR_CFG_GYR_DLPF_MASK			= (int)0x00000F00,
	SENSOR_CFG_GYR_DLPF_OFFSET			= (int)8,

	/** Accelerometer digital low-pass filter (DLPF) is set automatically based on the IMU sample rate.  The 
	following bit values can be used to override the bandwidth (frequency) to: 218, 218, 99, 45, 21, 10, 5 Hz */
	SENSOR_CFG_ACC_DLPF_218HZ			= (int)0x00000000,
	SENSOR_CFG_ACC_DLPF_218HZb			= (int)0x00000001,
	SENSOR_CFG_ACC_DLPF_99HZ			= (int)0x00000002,
	SENSOR_CFG_ACC_DLPF_45HZ			= (int)0x00000003,
	SENSOR_CFG_ACC_DLPF_21HZ			= (int)0x00000004,
	SENSOR_CFG_ACC_DLPF_10HZ			= (int)0x00000005,
	SENSOR_CFG_ACC_DLPF_5HZ				= (int)0x00000006,
	SENSOR_CFG_ACC_DLPF_MASK			= (int)0x0000F000,
	SENSOR_CFG_ACC_DLPF_OFFSET			= (int)12,

	/** Euler rotation of IMU and magnetometer from Hardware Frame to Sensor Frame.  Rotation applied in the order of yaw, pitch, roll from the sensor frame (labeled on uINS). */
	SENSOR_CFG_SENSOR_ROTATION_MASK        = (int)0x00FF0000,
	SENSOR_CFG_SENSOR_ROTATION_OFFSET      = (int)16,
	SENSOR_CFG_SENSOR_ROTATION_0_0_0       = (int)0,	// roll, pitch, yaw rotation (deg).
	SENSOR_CFG_SENSOR_ROTATION_0_0_90      = (int)1,
	SENSOR_CFG_SENSOR_ROTATION_0_0_180     = (int)2,
	SENSOR_CFG_SENSOR_ROTATION_0_0_N90     = (int)3,
	SENSOR_CFG_SENSOR_ROTATION_90_0_0      = (int)4,
	SENSOR_CFG_SENSOR_ROTATION_90_0_90     = (int)5,
	SENSOR_CFG_SENSOR_ROTATION_90_0_180    = (int)6,
	SENSOR_CFG_SENSOR_ROTATION_90_0_N90    = (int)7,
	SENSOR_CFG_SENSOR_ROTATION_180_0_0     = (int)8,
	SENSOR_CFG_SENSOR_ROTATION_180_0_90    = (int)9,
	SENSOR_CFG_SENSOR_ROTATION_180_0_180   = (int)10,
	SENSOR_CFG_SENSOR_ROTATION_180_0_N90   = (int)11,
	SENSOR_CFG_SENSOR_ROTATION_N90_0_0     = (int)12,
	SENSOR_CFG_SENSOR_ROTATION_N90_0_90    = (int)13,
	SENSOR_CFG_SENSOR_ROTATION_N90_0_180   = (int)14,
	SENSOR_CFG_SENSOR_ROTATION_N90_0_N90   = (int)15,
	SENSOR_CFG_SENSOR_ROTATION_0_90_0      = (int)16,
	SENSOR_CFG_SENSOR_ROTATION_0_90_90     = (int)17,
	SENSOR_CFG_SENSOR_ROTATION_0_90_180    = (int)18,
	SENSOR_CFG_SENSOR_ROTATION_0_90_N90    = (int)19,
	SENSOR_CFG_SENSOR_ROTATION_0_N90_0     = (int)20,
	SENSOR_CFG_SENSOR_ROTATION_0_N90_90    = (int)21,
	SENSOR_CFG_SENSOR_ROTATION_0_N90_180   = (int)22,
	SENSOR_CFG_SENSOR_ROTATION_0_N90_N90   = (int)23,

	/** Triple IMU fault detection level. Higher levels add new features to previous levels */
	SENSOR_CFG_IMU_FAULT_DETECT_MASK	   	= (int)0x0F000000,
	SENSOR_CFG_IMU_FAULT_DETECT_OFFSET		= (int)24,
	SENSOR_CFG_IMU_FAULT_DETECT_NONE		= (int)0,	// Simple averaging
	SENSOR_CFG_IMU_FAULT_DETECT_OFFLINE		= (int)1,	// One or more IMUs is offline or stuck
	SENSOR_CFG_IMU_FAULT_DETECT_LARGE_BIAS	= (int)2,
	SENSOR_CFG_IMU_FAULT_DETECT_BIAS_JUMPS	= (int)3,
	SENSOR_CFG_IMU_FAULT_DETECT_SENSOR_NOISE = (int)4,
};

/** IO configuration (used with nvm_flash_cfg_t.ioConfig) */
enum eIoConfig
{
	/** Strobe (input and output) trigger on rising edge (0 = falling edge) */
	IO_CONFIG_STROBE_TRIGGER_HIGH               = (int)0x00000001,

	// G1,G2 - STROBE, CAN, Ser2, I2C (future)
	/** G1,G2 - STROBE input on G2 */
	IO_CONFIG_G1G2_STROBE_INPUT_G2              = (int)0x00000002,
	/** G1,G2 - CAN Bus */
	IO_CONFIG_G1G2_CAN_BUS                      = (int)0x00000004,
	/** G1,G2 - General Communications on Ser2. Excludes GPS communications. */
	IO_CONFIG_G1G2_COM2                         = (int)0x00000006,
	/** G1,G2 - I2C */
	IO_CONFIG_G1G2_I2C							= (int)0x00000008,
	/** G1,G2 - MASK.  Note: This G1,G2 setting is overriden when GPS1 or GPS2 is configured to use Ser2. */
	IO_CONFIG_G1G2_MASK                         = (int)0x0000000E,
	/** G1,G2 - Default */
	IO_CONFIG_G1G2_DEFAULT                      = IO_CONFIG_G1G2_CAN_BUS,

	// G9 - STROBE, QDEC0 (future)
	/** G9 - Strobe input */
	IO_CONFIG_G9_STROBE_INPUT                   = (int)0x00000010,
	/** G9 - Enable Nav update strobe output pulse on G9 (uINS pin 10) indicating preintegrated IMU and navigation updates */
	IO_CONFIG_G9_STROBE_OUTPUT_NAV              = (int)0x00000020,
	/** G9 - SPI DRDY */
	IO_CONFIG_G9_SPI_DRDY                    	= (int)0x00000030,
	/** G9 - Bit mask */
	IO_CONFIG_G9_MASK                           = (int)0x00000030,
	/** G9 - Default */
	IO_CONFIG_G9_DEFAULT                        = (int)0,	

	// G6,G7 - Ser1, QDEC0 (future)
	/** G6,G7 - General Communications on Ser1. Excludes GPS communications.  Overriden when SPI is enabled (G9 held low on bootup/config). */
	IO_CONFIG_G6G7_COM1                         = (int)0x00000040,
	/** G6,G7 - Quadrature wheel encoder input (G6 QDEC0-A).  Overriden when SPI is enabled (G9 held low on bootup/config). */
//  IO_CONFIG_G6G7_QDEC0_INPUT_G6               = (int)0x00000080,
	/** G6,G7 - Bit mask */
	IO_CONFIG_G6G7_MASK                         = (int)0x000000C0,
	/** G6,G7 - Default */
	IO_CONFIG_G6G7_DEFAULT                      = IO_CONFIG_G6G7_COM1,	

	// G5,G8 - STROBE, QDEC1 (future), SPI (enabled when G9 is held low on bootup/config)
	/** G5,G8 - Strobe input on G5 */
	IO_CONFIG_G5G8_STROBE_INPUT_G5              = (int)0x00000100,
	/** G5,G8 - Strobe input on G8 */
	IO_CONFIG_G5G8_STROBE_INPUT_G8              = (int)0x00000200,
	/** G5,G8 - Strobe input on both G5 and G8 */
	IO_CONFIG_G5G8_STROBE_INPUT_G5_G8           = (int)0x00000300,
	/** G5,G8 - Strobe input on both G5 and G8 */
	IO_CONFIG_G5G8_G6G7_SPI_ENABLE              = (int)0x00000400,
	/** G5,G8 - Quadrature wheel encoder input (G5 QDEC1-B, G8 QDEC1-A) */
	IO_CONFIG_G5G8_QDEC_INPUT                   = (int)0x00000500,
	/** G5,G8 - Bit mask */
	IO_CONFIG_G5G8_MASK                         = (int)0x00000700,
	/** G5,G8 - Default */
	IO_CONFIG_G5G8_DEFAULT                      = (int)0,	

	/** G15 (GPS PPS) - STROBE */
	IO_CONFIG_G15_STROBE_INPUT                  = (int)0x00000800,
	// IO_CONFIG_                               = (int)0x00001000,

	/** GPS TIMEPULSE source */
	IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET			= (int)13,
	IO_CFG_GPS_TIMEPUSE_SOURCE_MASK				= (int)0x00000007,
	IO_CFG_GPS_TIMEPUSE_SOURCE_BITMASK			= (int)(IO_CFG_GPS_TIMEPUSE_SOURCE_MASK<<IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET),	
	IO_CFG_GPS_TIMEPUSE_SOURCE_DISABLED			= (int)0,
	IO_CFG_GPS_TIMEPUSE_SOURCE_GPS1_PPS_PIN20	= (int)1,
	IO_CFG_GPS_TIMEPUSE_SOURCE_GPS2_PPS			= (int)2,
	IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G2_PIN6	= (int)3,
	IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G5_PIN9	= (int)4,
	IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G8_PIN12	= (int)5,
	IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G9_PIN13	= (int)6,
#define SET_STATUS_OFFSET_MASK(result,val,offset,mask)	{ (result) &= ~((mask)<<(offset)); (result) |= ((val)<<(offset)); }	
#define IO_CFG_GPS_TIMEPUSE_SOURCE(ioConfig) (((ioConfig)>>IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET)&IO_CFG_GPS_TIMEPUSE_SOURCE_MASK)
	
	/** GPS 1 source OFFSET */
	IO_CONFIG_GPS1_SOURCE_OFFSET				= (int)16,
	/** GPS 2 source OFFSET */
	IO_CONFIG_GPS2_SOURCE_OFFSET				= (int)19,
	/** GPS 1 type OFFSET */
	IO_CONFIG_GPS1_TYPE_OFFSET					= (int)22,
	/** GPS 2 type OFFSET */
	IO_CONFIG_GPS2_TYPE_OFFSET					= (int)25,

	/** GPS 1 skip initialization */
	IO_CONFIG_GPS1_NO_INIT 						= (int)0x01000000,		// bit 24 of 0-31
	/** GPS 2 skip initialization */
	IO_CONFIG_GPS2_NO_INIT 						= (int)0x08000000,		// bit 27 of 0-31

	/** GPS source MASK */
	IO_CONFIG_GPS_SOURCE_MASK					= (int)0x00000007,
	/** GPS source - Disable */
	IO_CONFIG_GPS_SOURCE_DISABLE				= (int)0,
	/** GPS source - GNSS receiver 1 onboard uINS */
	IO_CONFIG_GPS_SOURCE_ONBOARD_1				= (int)1,
	/** GPS source - GNSS receiver 2 onboard uINS */
	IO_CONFIG_GPS_SOURCE_ONBOARD_2				= (int)2,
	/** GPS source - Serial 0 */
	IO_CONFIG_GPS_SOURCE_SER0					= (int)3,
	/** GPS source - Serial 1 */
	IO_CONFIG_GPS_SOURCE_SER1					= (int)4,
	/** GPS source - Serial 2 */
	IO_CONFIG_GPS_SOURCE_SER2					= (int)5,
	/** GPS source - last type */
	IO_CONFIG_GPS_SOURCE_LAST					= IO_CONFIG_GPS_SOURCE_SER2,	// set to last source

	/** GPS type MASK */
	IO_CONFIG_GPS_TYPE_MASK						= (int)0x00000003,
	/** GPS type - ublox M8 */
	IO_CONFIG_GPS_TYPE_UBX_M8					= (int)0,
	/** GPS type - ublox ZED-F9P w/ RTK */
	IO_CONFIG_GPS_TYPE_UBX_F9P					= (int)1,
	/** GPS type - NMEA */
	IO_CONFIG_GPS_TYPE_NMEA						= (int)2,
	/** GPS type - last type */
    IO_CONFIG_GPS_TYPE_LAST						= IO_CONFIG_GPS_TYPE_NMEA,		// Set to last type

#define IO_CONFIG_GPS1_SOURCE(ioConfig)     (((ioConfig)>>IO_CONFIG_GPS1_SOURCE_OFFSET)&IO_CONFIG_GPS_SOURCE_MASK)
#define IO_CONFIG_GPS2_SOURCE(ioConfig)     (((ioConfig)>>IO_CONFIG_GPS2_SOURCE_OFFSET)&IO_CONFIG_GPS_SOURCE_MASK)
#define IO_CONFIG_GPS1_TYPE(ioConfig)       (((ioConfig)>>IO_CONFIG_GPS1_TYPE_OFFSET)&IO_CONFIG_GPS_TYPE_MASK)
#define IO_CONFIG_GPS2_TYPE(ioConfig)       (((ioConfig)>>IO_CONFIG_GPS2_TYPE_OFFSET)&IO_CONFIG_GPS_TYPE_MASK)

#define SET_IO_CFG_GPS1_SOURCE(result,val)  SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GPS1_SOURCE_OFFSET, IO_CONFIG_GPS_SOURCE_MASK)
#define SET_IO_CFG_GPS2_SOURCE(result,val)  SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GPS2_SOURCE_OFFSET, IO_CONFIG_GPS_SOURCE_MASK)
#define SET_IO_CFG_GPS1_TYPE(result,val)    SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GPS1_TYPE_OFFSET, IO_CONFIG_GPS_TYPE_MASK)
#define SET_IO_CFG_GPS2_TYPE(result,val)    SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GPS2_TYPE_OFFSET, IO_CONFIG_GPS_TYPE_MASK)

	/** IMU 1 disable */	
	IO_CONFIG_IMU_1_DISABLE						= (int)0x10000000,
	/** IMU 2 disable */
	IO_CONFIG_IMU_2_DISABLE						= (int)0x20000000,
	/** IMU 3 disable */
	IO_CONFIG_IMU_3_DISABLE						= (int)0x40000000,

	/** Unused bits */
	// IO_CONFIG_                               = (int)0x80000000,
};

#define IO_CONFIG_DEFAULT 	(IO_CONFIG_G1G2_DEFAULT | IO_CONFIG_G5G8_DEFAULT | IO_CONFIG_G6G7_DEFAULT | IO_CONFIG_G9_DEFAULT)

enum ePlatformConfig
{
    // IMX Carrier Board
    PLATFORM_CFG_TYPE_MASK                      = (int)0x0000003F,
    PLATFORM_CFG_TYPE_FROM_MANF_OTP             = (int)0x00000080,  // Type is overwritten from manufacturing OTP memory.  Write protection, prevents direct change of platformType in flashConfig.
    PLATFORM_CFG_TYPE_NONE                      = (int)0,           // IMX-5 default
    PLATFORM_CFG_TYPE_NONE_ONBOARD_G2           = (int)1,           // uINS-3 default
    PLATFORM_CFG_TYPE_RUG1                      = (int)2,
    PLATFORM_CFG_TYPE_RUG2_0_G1                 = (int)3,
    PLATFORM_CFG_TYPE_RUG2_0_G2                 = (int)4,
    PLATFORM_CFG_TYPE_RUG2_1_G0                 = (int)5,           // PCB RUG-2.1, Case RUG-3.  GPS1 timepulse on G9
    PLATFORM_CFG_TYPE_RUG2_1_G1                 = (int)6,           // "
    PLATFORM_CFG_TYPE_RUG2_1_G2                 = (int)7,           // "
    PLATFORM_CFG_TYPE_RUG3_G0                   = (int)8,           // PCB RUG-3.x.  GPS1 timepulse on GPS1_PPS TIMESYNC (pin 20)
    PLATFORM_CFG_TYPE_RUG3_G1                   = (int)9,           // "
    PLATFORM_CFG_TYPE_RUG3_G2                   = (int)10,          // "
    PLATFORM_CFG_TYPE_EVB2_G2                   = (int)11,
    PLATFORM_CFG_TYPE_TBED3                     = (int)12,          // Testbed-3
    PLATFORM_CFG_TYPE_IG1_0_G2                  = (int)13,          // PCB IG-1.0.  GPS1 timepulse on G8
    PLATFORM_CFG_TYPE_IG1_G1                    = (int)14,          // PCB IG-1.1 and later.  GPS1 timepulse on GPS1_PPS TIMESYNC (pin 20)
    PLATFORM_CFG_TYPE_IG1_G2                    = (int)15,  
    PLATFORM_CFG_TYPE_IG2                       = (int)16,		    // IG-2 w/ IMX-5 and GPX-1
    PLATFORM_CFG_TYPE_LAMBDA_G1                 = (int)17,          // Enable UBX output on Lambda for testbed
    PLATFORM_CFG_TYPE_LAMBDA_G2              	= (int)18,          // "
    PLATFORM_CFG_TYPE_TBED2_G1_W_LAMBDA         = (int)19,          // Enable UBX input from Lambda
    PLATFORM_CFG_TYPE_TBED2_G2_W_LAMBDA         = (int)20,          // "
    PLATFORM_CFG_TYPE_COUNT                     = (int)21,

    // Presets
    PLATFORM_CFG_PRESET_MASK                    = (int)0x0000FF00,
    PLATFORM_CFG_PRESET_OFFSET                  = (int)8,

    // RUG-3 - Presets
    PLATFORM_CFG_RUG3_PRESET__0__PRESETS_DISABLED                               = 0,    // Don't use presets.  IOEXP_BITS can be set directly.
    PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GPS1          = 1,    // RUG-3-G0 default
    PLATFORM_CFG_RUG3_PRESET__2__S0_TTL_7_9_____CAN_11_12______S1_GPS1          = 2,
    PLATFORM_CFG_RUG3_PRESET__3__S0_TTL_7_9_____S2_TTL_8_10____S1_GPS1          = 3,
    PLATFORM_CFG_RUG3_PRESET__4__S0_RS232_7_9___S1_RS232_8_10__S2_GPS1          = 4,
    PLATFORM_CFG_RUG3_PRESET__5__S1_RS485_7_8_9_10_____________S2_GPS1__S0_GPS2 = 5,
    PLATFORM_CFG_RUG3_PRESET__6__SPI_7_8_9_10__________________S2_GPS1__S0_GPS2 = 6,
    PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GPS1__S0_GPS2 = 7,    // RUG-3-G2 default
    PLATFORM_CFG_RUG3_PRESET__8_________________CAN_11_12______S1_GPS1__S0_GPS2 = 8,
    PLATFORM_CFG_RUG3_PRESET__9__S2_TTL_8_10___________________S1_GPS1__S0_GPS2 = 9,
    PLATFORM_CFG_RUG3_PRESET__COUNT                                             = 10,

    PLATFORM_CFG_RUG3_PRESET__G0_DEFAULT        = PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GPS1,
    PLATFORM_CFG_RUG3_PRESET__G2_DEFAULT        = PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GPS1__S0_GPS2,

    // RUG-3 - I/O Expander disabled if platform type is != PLATFORM_CFG_TYPE_RUG3_x.
    PLATFORM_CFG_RUG3_IOEXP_BIT_MASK            = (int)0x00FF0000,
    PLATFORM_CFG_RUG3_IOEXP_BIT_OFFSET          = (int)16,

    RUG3_IOEXP_BIT_OFFSET_n232_485              = (int)0,
    RUG3_IOEXP_BIT_OFFSET_n232_TTL              = (int)1,
    RUG3_IOEXP_BIT_OFFSET_nRS_CAN               = (int)2,
    RUG3_IOEXP_BIT_OFFSET_nGPS2_RS              = (int)3,
    RUG3_IOEXP_BIT_OFFSET_nSPIEN                = (int)4,
    RUG3_IOEXP_BIT_OFFSET_nSPI_SER              = (int)5,
    RUG3_IOEXP_BIT_OFFSET_nGPSRST               = (int)6,

    PLATFORM_CFG_UPDATE_IO_CONFIG               = (int)0x01000000,    // Generate ioConfig based on platform config
};

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
    INS_DYN_MODEL_WRIST             = 9,
    INS_DYN_MODEL_INDOOR            = 10
} eInsDynModel;

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

    /** IMU sample (system input) period in milliseconds set on startup. Cannot be larger than startupNavDtMs. Zero disables sensor/IMU sampling. */
    uint32_t				startupImuDtMs;

    /** Navigation filter (system output) output period in milliseconds set on startup.  Used to initialize sysParams.navOutputPeriodMs. */
    uint32_t				startupNavDtMs;

    /** Serial port 0 baud rate in bits per second */
    uint32_t				ser0BaudRate;

    /** Serial port 1 baud rate in bits per second */
    uint32_t				ser1BaudRate;

    /** Rotation in radians about the X,Y,Z axes from Sensor Frame to Intermediate Output Frame.  Order applied: Z,Y,X. */
    float					insRotation[3];

    /** X,Y,Z offset in meters from Intermediate Output Frame to INS Output Frame. */
    float					insOffset[3];

    /** X,Y,Z offset in meters in Sensor Frame to GPS 1 antenna. */
    float					gps1AntOffset[3];
 
    /** INS dynamic platform model (see eInsDynModel).  Options are: 0=PORTABLE, 2=STATIONARY, 3=PEDESTRIAN, 4=GROUND VEHICLE, 5=SEA, 6=AIRBORNE_1G, 7=AIRBORNE_2G, 8=AIRBORNE_4G, 9=WRIST.  Used to balance noise and performance characteristics of the system.  The dynamics selected here must be at least as fast as your system or you experience accuracy error.  This is tied to the GPS position estimation model and intend in the future to be incorporated into the INS position model. */
    uint8_t					insDynModel;

	/** Debug */
	uint8_t					debug;

    /** Satellite system constellation used in GNSS solution.  (see eGnssSatSigConst) 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS */
    uint16_t				gnssSatSigConst;

    /** System configuration bits (see eSysConfigBits). */
    uint32_t				sysCfgBits;

    /** Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations (deg, deg, m) */
    double                  refLla[3];

    /** Last latitude, longitude, HAE (height above ellipsoid) used to aid GPS startup (deg, deg, m).  Updated when the distance between current LLA and lastLla exceeds lastLlaUpdateDistance. */
    double					lastLla[3];

    /** Last LLA GPS time since week start (Sunday morning) in milliseconds */
    uint32_t				lastLlaTimeOfWeekMs;

    /** Last LLA GPS number of weeks since January 6th, 1980 */
    uint32_t				lastLlaWeek;

    /** Distance between current and last LLA that triggers an update of lastLla  */
    float					lastLlaUpdateDistance;

    /** Hardware interface configuration bits (see eIoConfig). */
    uint32_t				ioConfig;

    /** Hardware platform specifying the IMX carrier board type (i.e. RUG, EVB, IG) and configuration bits (see ePlatformConfig).  The platform type is used to simplify the GPS and I/O configuration process.  */
    uint32_t				platformConfig;

    /** X,Y,Z offset in meters from DOD_ Frame origin to GPS 2 antenna. */
    float					gps2AntOffset[3];

    /** Euler (roll, pitch, yaw) rotation in radians from INS Sensor Frame to Intermediate ZeroVelocity Frame.  Order applied: heading, pitch, roll. */
    float					zeroVelRotation[3];

    /** X,Y,Z offset in meters from Intermediate ZeroVelocity Frame to Zero Velocity Frame. */
    float					zeroVelOffset[3];

    /** (sec) User defined delay for GPS time.  This parameter can be used to account for GPS antenna cable delay.  */
    float                   gpsTimeUserDelay;

    /** Earth magnetic field (magnetic north) declination (heading offset from true north) in radians */
    float                   magDeclination;

    /** Time between GPS time synchronization pulses in milliseconds.  Requires reboot to take effect. */
    uint32_t				gpsTimeSyncPeriodMs;
	
	/** GPS measurement (system input) update period in milliseconds set on startup. 200ms minimum (5Hz max). */
    uint32_t				startupGPSDtMs;
	
	/** RTK configuration bits (see eRTKConfigBits). */
    uint32_t				RTKCfgBits;

    /** Sensor config to specify the full-scale sensing ranges and output rotation for the IMU and magnetometer (see eSensorConfig in data_sets.h) */
    uint32_t                sensorConfig;

	/** Minimum elevation of a satellite above the horizon to be used in the solution (radians). Low elevation satellites may provide degraded accuracy, due to the long signal path through the atmosphere. */
	float                   gpsMinimumElevation;

    /** Serial port 2 baud rate in bits per second */
    uint32_t				ser2BaudRate;

	/** Wheel encoder: euler angles describing the rotation from imu to left wheel */
    wheel_config_t          wheelConfig;

	/** Magnetometer interference sensitivity threshold. Typical range is 2-10 (3 default) and 1000 to disable mag interference detection. */
	float                   magInterferenceThreshold;

} nvm_flash_cfg_t;

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

	/** Strobe input pin (i.e. G1, G2, G5, or G9) */
	uint16_t				pin;

	/** Strobe serial index number */
	uint16_t				count;
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

PUSH_PACK_8

/** time struct */
typedef struct
{
	/** time (s) expressed by standard time_t */
	int64_t time;

	/** fraction of second under 1 s */
	double sec;         
} gtime_t;

typedef struct PACKED
{
	gtime_t time;
	double rp_ecef[3]; // Rover position
	double rv_ecef[3]; // Rover velocity
	double ra_ecef[3]; // Rover acceleration
	double bp_ecef[3]; // Base position
	double bv_ecef[3]; // Base velocity
	double qr[6]; // rover position and velocity covariance main diagonal
	double b[24]; // satellite bias
	double qb[24]; // main diagonal of sat bias covariances
	uint8_t sat_id[24]; // satellite id of b[]
} rtk_state_t;

typedef struct PACKED
{
	gtime_t time;
	int32_t nv; // number of measurements
	uint8_t sat_id_i[24]; // sat id of measurements (reference sat)
	uint8_t sat_id_j[24]; // sat id of measurements
	uint8_t type[24]; // type (0 = dd-range, 1 = dd-phase, 2 = baseline)
	double v[24]; // residual
} rtk_residual_t;

typedef struct PACKED
{
    gtime_t time;

    uint8_t rej_ovfl;
    uint8_t code_outlier;
    uint8_t phase_outlier;
    uint8_t code_large_residual;

    uint8_t phase_large_residual;
    uint8_t invalid_base_position;
    uint8_t bad_baseline_holdamb;
    uint8_t base_position_error;

    uint8_t outc_ovfl;
    uint8_t reset_timer;
    uint8_t use_ubx_position;
    uint8_t large_v2b;

    uint8_t base_position_update;
    uint8_t rover_position_error;
    uint8_t reset_bias;
    uint8_t start_relpos;

    uint8_t end_relpos;
    uint8_t start_rtkpos;
    uint8_t pnt_pos_error;
    uint8_t no_base_obs_data;

    uint8_t diff_age_error;
    uint8_t moveb_time_sync_error;
    uint8_t waiting_for_rover_packet;
    uint8_t waiting_for_base_packet;

	uint8_t lsq_error;
    uint8_t lack_of_valid_sats;
    uint8_t divergent_pnt_pos_iteration;
    uint8_t chi_square_error;

    uint32_t cycle_slips;

    float ubx_error;

	uint8_t solStatus;
	uint8_t rescode_err_marker;
	uint8_t error_count;
	uint8_t error_code;

    float dist2base;

	uint8_t reserved1;
	uint8_t gdop_error;
	uint8_t warning_count;
	uint8_t warning_code;

    double double_debug[4];

	uint8_t debug[2];
	uint8_t obs_count_bas;
	uint8_t obs_count_rov;

	uint8_t obs_pairs_filtered;
	uint8_t obs_pairs_used;
	uint8_t raw_ptr_queue_overrun;
    uint8_t raw_dat_queue_overrun;
} rtk_debug_t;

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

    /** SBAS AR mode (0:off,1:on) */
    int32_t sbsmodear;

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

    /** reset sat biases after this long trying to get fix if not acquired */
    int fix_reset_base_msgs;

    /** reject threshold of NIS */
    double maxinnocode;
    double maxinnophase;
    double maxnis;

	/** reject threshold of gdop */
	double maxgdop;

    /** baseline length constraint {const,sigma before fix, sigma after fix} (m) */
    double baseline[3];
    double max_baseline_error;
    double reset_baseline_error;

    /** maximum error wrt ubx position (triggers reset if more than this far) (m) */
    float max_ubx_error;

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
	/** Receiver local time approximately aligned to the GPS time system (GPST) */
	gtime_t time;

	/** Satellite number in RTKlib notation.  GPS: 1-32, GLONASS: 33-59, Galilleo: 60-89, SBAS: 90-95 */
	uint8_t sat;

	/** receiver number */
	uint8_t rcv;

	/** Cno, carrier-to-noise density ratio (signal strength) (0.25 dB-Hz) */
	uint8_t SNR[1];

	/** Loss of Lock Indicator. Set to non-zero values only when carrier-phase is valid (L > 0).  bit1 = loss-of-lock, bit2 = half-cycle-invalid */
	uint8_t LLI[1];

	/** Code indicator: CODE_L1C (1) = L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS), CODE_L1X (12) = E1B+C,L1C(D+P) (GAL,QZS), CODE_L1I (47) = B1I (BeiDou) */
	uint8_t code[1];

	/** Estimated carrier phase measurement standard deviation (0.004 cycles), zero means invalid */
	uint8_t qualL[1];

	/** Estimated pseudorange measurement standard deviation (0.01 m), zero means invalid */
	uint8_t qualP[1];

	/** reserved, for alignment */
	uint8_t reserved;

	/** Observation data carrier-phase (cycle). The carrier phase initial ambiguity is initialized using an approximate value to make the magnitude of the phase close to the pseudorange measurement. Clock resets are applied to both phase and code measurements in accordance with the RINEX specification. */
	double L[1];

	/** Observation data pseudorange (m). GLONASS inter frequency channel delays are compensated with an internal calibration table */
	double P[1]; 

	/** Observation data Doppler measurement (positive sign for approaching satellites) (Hz) */
	float D[1];
} obsd_t;

#define GPS_RAW_MESSAGE_BUF_SIZE    1000
#define MAX_OBSERVATION_COUNT_IN_RTK_MESSAGE (GPS_RAW_MESSAGE_BUF_SIZE / sizeof(obsd_t))

/** observation data */
typedef struct
{
	/** number of observation slots used */
	uint32_t n;

	/** number of observation slots allocated */
	uint32_t nmax;

	/** observation data buffer */
	obsd_t* data;
} obs_t;

/** non-Glonass ephemeris data */
typedef struct
{
    /** Satellite number in RTKlib notation.  GPS: 1-32, GLONASS: 33-59, Galilleo: 60-89, SBAS: 90-95 */
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

	/** GPS/QZS: code on L2. (00 = Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid).  GAL/CMP: data sources */
	int32_t code;

	/** GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel). CMP: nav type */
	int32_t flag;

	/** Time Of Ephemeris, ephemeris reference epoch in seconds within the week (s) */
	gtime_t toe;
	
	/** clock data reference time (s) (20.3.4.5) */
	gtime_t toc;
	
	/** T_trans (s) */
	gtime_t ttr;

	/** Orbit semi-major axis (m) */
	double A;

	/** Orbit eccentricity (non-dimensional)  */
	double e;

	/** Orbit inclination angle at reference time (rad) */
	double i0;

	/** Longitude of ascending node of orbit plane at weekly epoch (rad) */
	double OMG0;

	/** Argument of perigee (rad) */
	double omg;

	/** Mean anomaly at reference time (rad) */
	double M0;

	/** Mean Motion Difference From Computed Value (rad) */
	double deln;

	/** Rate of Right Ascension (rad/s) */
	double OMGd;

	/** Rate of Inclination Angle (rad/s) */
	double idot;

	/** Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius (m) */
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

	/** Time Of Ephemeris, ephemeris reference epoch in seconds within the week (s), same as <toe> above but represented as double type. Note that toe is computed as eph->toe = gst2time(week, eph->toes) */
	double toes;

	/** Fit interval (h) (0: 4 hours, 1: greater than 4 hours) */
	double fit;

	/** SV clock offset, af0 (s) */
	double f0;
	
	/** SV clock drift, af1 (s/s, non-dimensional) */
	double f1;
	
	/** SV clock drift rate, af2 (1/s) */
	double f2;

	/** Group delay parameters GPS/QZS: tgd[0] = TGD (IRN-IS-200H p.103). Galilleo: tgd[0] = BGD E5a/E1, tgd[1] = BGD E5b/E1. Beidou: tgd[0] = BGD1, tgd[1] = BGD2 */
	double tgd[4];

	/** Adot for CNAV, not used */
	double Adot;
	
	/** First derivative of mean motion n (second derivative of mean anomaly M), ndot for CNAV (rad/s/s). Not used. */
	double ndot;
} eph_t;

/** Glonass ephemeris data */
typedef struct
{        
    /** Satellite number in RTKlib notation.  GPS: 1-32, GLONASS: 33-59, Galilleo: 60-89, SBAS: 90-95 */
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

    /** Ephemeris reference epoch in seconds within the week in GPS time gpst (s) */
	gtime_t toe;

	/** message frame time in gpst (s) */
	gtime_t tof;

	/** satellite position (ecef) (m) */
	double pos[3];

	/** satellite velocity (ecef) (m/s) */
	double vel[3];

	/** satellite acceleration (ecef) (m/s^2) */
	double acc[3];

	/** SV clock bias (s) */
	double taun;

	/** relative frequency bias */
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
} gps_raw_t;

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

	/** Manufacturing unlocked */
    EVB_STATUS_MANF_UNLOCKED                = 0x02000000,

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
#define EVB_CFG_BITS_SET_IDX_WIFI(bits,idx)     {(bits)&=EVB_CFG_BITS_WIFI_SELECT_MASK; (bits)|=(((idx)<<EVB_CFG_BITS_WIFI_SELECT_OFFSET)&EVB_CFG_BITS_WIFI_SELECT_MASK);}
#define EVB_CFG_BITS_SET_IDX_SERVER(bits,idx)   {(bits)&=EVB_CFG_BITS_SERVER_SELECT_MASK; (bits)|=(((idx)<<EVB_CFG_BITS_SERVER_SELECT_OFFSET)&EVB_CFG_BITS_SERVER_SELECT_MASK);}
#define EVB_CFG_BITS_IDX_WIFI(bits)             (((bits)&EVB_CFG_BITS_WIFI_SELECT_MASK)>>EVB_CFG_BITS_WIFI_SELECT_OFFSET)
#define EVB_CFG_BITS_IDX_SERVER(bits)           (((bits)&EVB_CFG_BITS_SERVER_SELECT_MASK)>>EVB_CFG_BITS_SERVER_SELECT_OFFSET)

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
	IMX_RTOS_NUM_TASKS                 // Keep last
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

	/** Last runtime microseconds */
	uint32_t                runtimeUs;

	/** Average runtime */
	float					avgRuntimeUs;

	/** Average of runtimes less than avgRuntimeUs */
	float					lowerRuntimeUs;

	/** Average of runtimes greater than avgRuntimeUs */
	float					upperRuntimeUs;

	/** Max runtime microseconds */
	uint32_t                maxRuntimeUs;

	/** Local time when task loop started (following delay) */
	uint32_t                startTimeUs;

	/** Counter of times task took too long to run */
	uint16_t				gapCount;

	/** Counter of times task took too long to run twice in a row */
	uint8_t					doubleGapCount;

	/** Reserved */
	uint8_t					reserved;

	/** Processor usage percent */
    float					cpuUsage;

	/** Handle */
	uint32_t                handle;
	
} rtos_task_t;

/** Internal RTOS task profiling info (processor ticks instead of usec) */
typedef struct PACKED
{
	/** Time in microseconds */
	uint32_t                timeTicks;

	/** Runtime in microseconds */
	uint32_t                runtimeTicks;

	/** LPF average runtime */
	float					avgRuntimeTicks;

	/** Average of runtimes less than avgRuntimeTicks */
	float					lowerRuntimeTicks;

	/** Average of runtimes greater than avgRuntimeTicks */
	float					upperRuntimeTicks;

	/** Maximum runtime microseconds */
	uint32_t                maxRuntimeTicks;

	/** Local time when task loop started (following delay) */
	uint32_t                startTimeTicks;

	/** Counter of times task took too long to run */
	uint16_t				gapCount;

	/** Counter of times task took too long to run back-to-back */
	uint8_t					doubleGapCount;

	/** Indicates whether gap occurd on last update */
	uint8_t					gapOnLast;

	/** Task period ms */
	uint32_t 				periodTicks;

} rtos_profile_t;

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
	rtos_task_t             task[IMX_RTOS_NUM_TASKS];

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
    CID_GPS2_POS,
    CID_GPS1_RTK_POS_REL,
    CID_GPS2_RTK_CMP_REL,
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
    /** Broadcast period multiple - CAN time message. 0 to disable. */
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
	pos_measurement_t		posMeasurement;
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

/** DID to RMC bit look-up table */
extern const uint64_t g_didToRmcBit[DID_COUNT_UINS];
uint64_t didToRmcBit(uint32_t dataId, uint64_t defaultRmcBits, uint64_t devInfoRmcBits);

/** DID to NMEA RMC bit look-up table */
extern const uint64_t g_didToNmeaRmcBit[DID_COUNT_UINS];

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

#ifndef __RTKLIB_EMBEDDED_DEFINES_H_

#undef ENAGLO
#define ENAGLO

#undef ENAGAL
#define ENAGAL

#undef ENAQZS
//#define ENAQZS

#undef ENASBS
#define ENASBS

#undef MAXSUBFRMLEN
#define MAXSUBFRMLEN 152

#undef MAXRAWLEN
#define MAXRAWLEN 2048

#undef NFREQ
#define NFREQ 1

#undef NFREQGLO
#ifdef ENAGLO
#define NFREQGLO 1
#else
#define NFREQGLO 0
#endif

#undef NFREQGAL
#ifdef ENAGAL
#define NFREQGAL 1
#else
#define NFREQGAL 0
#endif

#undef NEXOBS
#define NEXOBS 0

#undef MAXOBS
#define MAXOBS 56               // Also defined inside rtklib_defines.h
#define HALF_MAXOBS (MAXOBS/2)

#undef NUMSATSOL
#define NUMSATSOL 22

#undef MAXERRMSG
#define MAXERRMSG 0

#ifdef ENASBS

// sbas waas only satellites
#undef MINPRNSBS
#define MINPRNSBS 133                 /* min satellite PRN number of SBAS */

#undef MAXPRNSBS
#define MAXPRNSBS 138                 /* max satellite PRN number of SBAS */

#undef NSATSBS
#define NSATSBS (MAXPRNSBS - MINPRNSBS + 1) /* number of SBAS satellites */

#define SBAS_EPHEMERIS_ARRAY_SIZE NSATSBS

#else

#define SBAS_EPHEMERIS_ARRAY_SIZE 0

#endif


#endif

#ifndef RTKLIB_H

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#ifdef ENAGLO
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   27                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif
#ifdef ENAGAL
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   30                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL     1
#else
#define MINPRNGAL   0
#define MAXPRNGAL   0
#define NSATGAL     0
#define NSYSGAL     0
#endif
#ifdef ENAQZS
#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS   199                 /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 189                 /* max satellite PRN number of QZSS SAIF */
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define NSYSQZS     1
#else
#define MINPRNQZS   0
#define MAXPRNQZS   0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS     0
#define NSYSQZS     0
#endif
#ifdef ENACMP
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   35                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1
#else
#define MINPRNCMP   0
#define MAXPRNCMP   0
#define NSATCMP     0
#define NSYSCMP     0
#endif
#ifdef ENAIRN
#define MINPRNIRN   1                   /* min satellite sat number of IRNSS */
#define MAXPRNIRN   7                   /* max satellite sat number of IRNSS */
#define NSATIRN     (MAXPRNIRN-MINPRNIRN+1) /* number of IRNSS satellites */
#define NSYSIRN     1
#else
#define MINPRNIRN   0
#define MAXPRNIRN   0
#define NSATIRN     0
#define NSYSIRN     0
#endif
#ifdef ENALEO
#define MINPRNLEO   1                   /* min satellite sat number of LEO */
#define MAXPRNLEO   10                  /* max satellite sat number of LEO */
#define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
#define NSYSLEO     1
#else
#define MINPRNLEO   0
#define MAXPRNLEO   0
#define NSATLEO     0
#define NSYSLEO     0
#endif
#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP+NSYSIRN+NSYSLEO) /* number of systems */
#ifndef NSATSBS
#ifdef ENASBS
#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */
#else
#define MINPRNSBS   0
#define MAXPRNSBS   0
#define NSATSBS     0
#endif
#endif

#endif
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


#ifdef __cplusplus
}
#endif

#endif // DATA_SETS_H
