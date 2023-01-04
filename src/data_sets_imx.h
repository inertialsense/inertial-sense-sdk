/*
MIT LICENSE

Copyright (c) 2014-2022 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef IMX_DATA_SETS_H
#define IMX_DATA_SETS_H

#include <stdint.h>
#include "data_sets.h"

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

#define IMX_DID_NULL                        (eDataIDs)0  /** NULL (INVALID) */
#define IMX_DID_DEV_INFO                    (eDataIDs)1  /** (dev_info_t) Device information */
#define IMX_DID_SYS_FAULT                   (eDataIDs)2  /** (system_fault_t) System fault information */
#define IMX_DID_PIMU                        (eDataIDs)3  /** (pimu_t) Preintegrated IMU (a.k.a. Coning and Sculling integral) in body/IMU frame.  Updated at IMU rate. Also know as delta theta delta velocity, or preintegrated IMU (PIMU). For clarification, the name "Preintegrated IMU" or "PIMU" throughout our User Manual. This data is integrated from the IMU data at the IMU update rate (startupImuDtMs, default 1ms).  The integration period (dt) and output data rate are the same as the NAV rate (startupNavDtMs) and cannot be output at any other rate. If a faster output data rate is desired, DID_IMU_RAW can be used instead. PIMU data acts as a form of compression, adding the benefit of higher integration rates for slower output data rates, preserving the IMU data without adding filter delay and addresses antialiasing. It is most effective for systems that have higher dynamics and lower communications data rates.  The minimum data period is DID_FLASH_CONFIG_IMX.startupImuDtMs or 4, whichever is larger (250Hz max). The PIMU value can be converted to IMU by dividing PIMU by dt (i.e. IMU = PIMU / dt)  */
#define IMX_DID_INS_1                       (eDataIDs)4  /** (ins_1_t) INS output: euler rotation w/ respect to NED, NED position from reference LLA. */
#define IMX_DID_INS_2                       (eDataIDs)5  /** (ins_2_t) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
#define IMX_DID_GNSS1_UBX_POS               (eDataIDs)6  /** (gnss_pos_t) GNSS 1 position data from ublox receiver. */
#define IMX_DID_SYS_CMD                     (eDataIDs)7  /** (system_command_t) System commands. Both the command and invCommand fields must be set at the same time for a command to take effect. */
#define IMX_DID_ASCII_BCAST_PERIOD          (eDataIDs)8  /** (ascii_msgs_t) Broadcast period for ASCII messages */
#define IMX_DID_RMC                         (eDataIDs)9  /** (rmc_t) Realtime Message Controller (RMC). The data sets available through RMC are driven by the availability of the data. The RMC provides updates from various data sources (i.e. sensors) as soon as possible with minimal latency. Several of the data sources (sensors) output data at different data rates that do not all correspond. The RMC is provided so that broadcast of sensor data is done as soon as it becomes available. All RMC messages can be enabled using the standard Get Data packet format. */
#define IMX_DID_SYS_PARAMS                  (eDataIDs)10 /** (sys_params_t) System parameters / info */
#define IMX_DID_SYS_SENSORS                 (eDataIDs)11 /** (sys_sensors_t) System sensor information */
#define IMX_DID_FLASH_CONFIG_IMX                (eDataIDs)12 /** (nvm_cfg_imx_t) Flash memory configuration */
#define IMX_DID_GNSS1_POS                   (eDataIDs)13 /** (gnss_pos_t) GNSS 1 position data.  This comes from IMX_DID_GNSS1_UBX_POS or IMX_DID_GNSS1_RTK_POS, depending on whichever is more accurate. */
#define IMX_DID_GNSS2_POS                   (eDataIDs)14 /** (gnss_pos_t) GNSS 2 position data */
#define IMX_DID_GNSS1_SAT                   (eDataIDs)15 /** (gnss_sat_t) GNSS 1 GNSS and sat identifiers, carrier to noise ratio (signal strength), elevation and azimuth angles, pseudo range residual. */
#define IMX_DID_GNSS2_SAT                   (eDataIDs)16 /** (gnss_sat_t) GNSS 2 GNSS and sat identifiers, carrier to noise ratio (signal strength), elevation and azimuth angles, pseudo range residual. */
#define IMX_DID_GNSS1_VERSION               (eDataIDs)17 /** (gnss_version_t) GNSS 1 version info */
#define IMX_DID_GNSS2_VERSION               (eDataIDs)18 /** (gnss_version_t) GNSS 2 version info */
#define IMX_DID_MAG_CAL                     (eDataIDs)19 /** (mag_cal_t) Magnetometer calibration */
#define IMX_DID_INTERNAL_DIAGNOSTIC         (eDataIDs)20 /** INTERNAL USE ONLY (internal_diagnostic_t) Internal diagnostic info */
#define IMX_DID_GNSS1_RTK_POS_REL           (eDataIDs)21 /** (gnss_rtk_rel_t) RTK precision position base to rover relative info. */
#define IMX_DID_GNSS1_RTK_POS_MISC          (eDataIDs)22 /** (gnss_rtk_misc_t) RTK precision position related data. */
#define IMX_DID_FEATURE_BITS                (eDataIDs)23 /** INTERNAL USE ONLY (feature_bits_t) */
#define IMX_DID_SENSORS_UCAL                (eDataIDs)24 /** INTERNAL USE ONLY (sensors_w_temp_t) Uncalibrated IMU output. */
#define IMX_DID_SENSORS_TCAL                (eDataIDs)25 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated IMU output. */
#define IMX_DID_SENSORS_TC_BIAS             (eDataIDs)26 /** INTERNAL USE ONLY (sensors_t) */
#define IMX_DID_IO                          (eDataIDs)27 /** (io_t) I/O */
#define IMX_DID_SENSORS_ADC                 (eDataIDs)28 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define IMX_DID_SCOMP                       (eDataIDs)29 /** INTERNAL USE ONLY (sensor_compensation_t) */
#define IMX_DID_GNSS1_VEL                   (eDataIDs)30 /** (gnss_vel_t) GNSS 1 velocity data */
#define IMX_DID_GNSS2_VEL                   (eDataIDs)31 /** (gnss_vel_t) GNSS 2 velocity data */
#define IMX_DID_HDW_PARAMS                  (eDataIDs)32 /** INTERNAL USE ONLY (hdw_params_t) */
#define IMX_DID_NVR_MANAGE_USERPAGE         (eDataIDs)33 /** INTERNAL USE ONLY (nvr_manage_t) */
#define IMX_DID_NVR_USERPAGE_SN             (eDataIDs)34 /** INTERNAL USE ONLY (nvm_group_sn_t) */
#define IMX_DID_NVR_USERPAGE_G0             (eDataIDs)35 /** INTERNAL USE ONLY (nvm_group_0_t) */
#define IMX_DID_NVR_USERPAGE_G1             (eDataIDs)36 /** INTERNAL USE ONLY (nvm_group_1_t) */
#define IMX_DID_DEBUG_STRING                (eDataIDs)37 /** INTERNAL USE ONLY (debug_string_t) */
#define IMX_DID_RTOS_INFO                   (eDataIDs)38 /** (rtos_info_t) RTOS information. */
#define IMX_DID_DEBUG_ARRAY                 (eDataIDs)39 /** INTERNAL USE ONLY (debug_array_t) */
#define IMX_DID_SENSORS_MCAL                (eDataIDs)40 /** INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated and motion calibrated IMU output. */
#define IMX_DID_GNSS1_TIMEPULSE             (eDataIDs)41 /** INTERNAL USE ONLY (gnss_timepulse_t) */
#define IMX_DID_CAL_SC                      (eDataIDs)42 /** INTERNAL USE ONLY (sensor_cal_t) */
#define IMX_DID_CAL_TEMP_COMP               (eDataIDs)43 /** INTERNAL USE ONLY (sensor_tcal_group_t) */
#define IMX_DID_CAL_MOTION                  (eDataIDs)44 /** INTERNAL USE ONLY (sensor_mcal_group_t) */
// #define IMX_DID_UNUSED_45           		(eDataIDs)45 /** used to be internal DID_SYS_SENSORS_SIGMA */
#define IMX_DID_SENSORS_ADC_SIGMA           (eDataIDs)46 /** INTERNAL USE ONLY (sys_sensors_adc_t) */
#define IMX_DID_REFERENCE_MAGNETOMETER      (eDataIDs)47 /** (magnetometer_t) Reference or truth magnetometer used for manufacturing calibration and testing */
#define IMX_DID_INL2_STATES                 (eDataIDs)48 /** (inl2_states_t) */
#define IMX_DID_INL2_COVARIANCE_LD          (eDataIDs)49 /** (INL2_COVARIANCE_LD_ARRAY_SIZE) */
#define IMX_DID_INL2_STATUS                 (eDataIDs)50 /** (inl2_status_t) */
#define IMX_DID_INL2_MISC                   (eDataIDs)51 /** (inl2_misc_t) */
#define IMX_DID_MAGNETOMETER                (eDataIDs)52 /** (magnetometer_t) Magnetometer sensor output */
#define IMX_DID_BAROMETER                   (eDataIDs)53 /** (barometer_t) Barometric pressure sensor data */
#define IMX_DID_GNSS1_RTK_POS               (eDataIDs)54 /** (gnss_pos_t) GNSS RTK position data */
#define IMX_DID_ROS_COVARIANCE_POSE_TWIST   (eDataIDs)55 /** (ros_covariance_pose_twist_t) INL2 EKF covariances matrix lower diagonals */
#define IMX_DID_COMMUNICATIONS_LOOPBACK     (eDataIDs)56 /** INTERNAL USE ONLY - Unit test for communications manager  */
#define IMX_DID_IMU3_UNCAL                  (eDataIDs)57 /** INTERNAL USE ONLY (imu3_t) Uncalibrated triple IMU data.  We recommend use of DID_IMU or DID_PIMU as they are calibrated and oversampled and contain less noise.  Minimum data period is DID_FLASH_CONFIG_IMX.startupImuDtMs or 4, whichever is larger (250Hz max). */
#define IMX_DID_IMU                         (eDataIDs)58 /** (imu_t) Inertial measurement unit data down-sampled from IMU rate (DID_FLASH_CONFIG_IMX.startupImuDtMs (1KHz)) to navigation update rate (DID_FLASH_CONFIG_IMX.startupNavDtMs) as an anti-aliasing filter to reduce noise and preserve accuracy.  Minimum data period is DID_FLASH_CONFIG_IMX.startupNavDtMs (1KHz max).  */
#define IMX_DID_INL2_MAG_OBS_INFO           (eDataIDs)59 /** (inl2_mag_obs_info_t) INL2 magnetometer calibration information. */
#define IMX_DID_GNSS_BASE_RAW               (eDataIDs)60 /** (gnss_raw_t) GNSS raw data for base station (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define IMX_DID_GNSS_RTK_OPT                (eDataIDs)61 /** (gnss_rtk_opt_t) RTK options - requires little endian CPU. */
#define IMX_DID_REFERENCE_PIMU              (eDataIDs)62 /** (pimu_t) Reference or truth IMU used for manufacturing calibration and testing */
#define IMX_DID_MANUFACTURING_INFO          (eDataIDs)63 /** INTERNAL USE ONLY (manufacturing_info_t) Manufacturing info */
#define IMX_DID_BIT                         (eDataIDs)64 /** (bit_t) System built-in self-test */
#define IMX_DID_INS_3                       (eDataIDs)65 /** (ins_3_t) Inertial navigation data with quaternion NED to body rotation and ECEF position. */
#define IMX_DID_INS_4                       (eDataIDs)66 /** (ins_4_t) INS output: quaternion rotation w/ respect to ECEF, ECEF position. */
#define IMX_DID_INL2_NED_SIGMA              (eDataIDs)67 /** (inl2_ned_sigma_t) Standard deviation of INL2 EKF estimates in the NED frame. */
#define IMX_DID_STROBE_IN_TIME              (eDataIDs)68 /** (strobe_in_time_t) Timestamp for input strobe. */
#define IMX_DID_GNSS1_RAW                   (eDataIDs)69 /** (gnss_raw_t) GNSS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define IMX_DID_GNSS2_RAW                   (eDataIDs)70 /** (gnss_raw_t) GNSS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define IMX_DID_WHEEL_ENCODER               (eDataIDs)71 /** (wheel_encoder_t) Wheel encoder data to be fused with GNSS-INS measurements, set DID_GROUND_VEHICLE for configuration before sending this message */
#define IMX_DID_DIAGNOSTIC_MESSAGE          (eDataIDs)72 /** (diag_msg_t) Diagnostic message */
#define IMX_DID_SURVEY_IN                   (eDataIDs)73 /** (survey_in_t) Survey in, used to determine position for RTK base station. Base correction output cannot run during a survey and will be automatically disabled if a survey is started. */
#define IMX_DID_CAL_SC_INFO                 (eDataIDs)74 /** INTERNAL USE ONLY (sensor_cal_info_t) */
#define IMX_DID_PORT_MONITOR                (eDataIDs)75 /** (port_monitor_t) Data rate and status monitoring for each communications port. */
#define IMX_DID_RTK_STATE                   (eDataIDs)76 /** INTERNAL USE ONLY (rtk_state_t) */
#define IMX_DID_RTK_PHASE_RESIDUAL          (eDataIDs)77 /** INTERNAL USE ONLY (rtk_residual_t) */
#define IMX_DID_RTK_CODE_RESIDUAL           (eDataIDs)78 /** INTERNAL USE ONLY (rtk_residual_t) */
#define IMX_DID_RTK_DEBUG                   (eDataIDs)79 /** INTERNAL USE ONLY (rtk_debug_t) */
#define IMX_DID_EVB_STATUS                  (eDataIDs)80 /** (evb_status_t) EVB monitor and log control interface. */
#define IMX_DID_EVB_FLASH_CFG               (eDataIDs)81 /** (evb_flash_cfg_t) EVB configuration. */
#define IMX_DID_EVB_DEBUG_ARRAY             (eDataIDs)82 /** INTERNAL USE ONLY (debug_array_t) */
#define IMX_DID_EVB_RTOS_INFO               (eDataIDs)83 /** (evb_rtos_info_t) EVB-2 RTOS information. */
// #define DID_UNUSED_84                	(eDataIDs)84 /** Unused */
#define IMX_DID_IMU_MAG                     (eDataIDs)85 /** (imu_mag_t) DID_IMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define IMX_DID_PIMU_MAG                    (eDataIDs)86 /** (pimu_mag_t) DID_PIMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define IMX_DID_GROUND_VEHICLE				(eDataIDs)87 /** (ground_vehicle_t) Static configuration for wheel transform measurements. */
// #define DID_UNUSED_88					(eDataIDs)88 /** Unused */
#define IMX_DID_RTK_DEBUG_2                 (eDataIDs)89 /** INTERNAL USE ONLY (rtk_debug_2_t) */
#define IMX_DID_CAN_CONFIG					(eDataIDs)90 /** (can_config_t) Addresses for CAN messages*/
#define IMX_DID_GNSS2_RTK_CMP_REL           (eDataIDs)91 /** (gnss_rtk_rel_t) Dual GNSS RTK compassing / moving base to rover (GNSS 1 to GNSS 2) relative info. */
#define IMX_DID_GNSS2_RTK_CMP_MISC          (eDataIDs)92 /** (gnss_rtk_misc_t) RTK Dual GNSS RTK compassing related data. */
#define IMX_DID_EVB_DEV_INFO                (eDataIDs)93 /** (dev_info_t) EVB device information */
#define IMX_DID_INFIELD_CAL                 (eDataIDs)94 /** (infield_cal_t) Measure and correct IMU calibration error.  Estimate INS rotation to align INS with vehicle. */
#define IMX_DID_REFERENCE_IMU               (eDataIDs)95 /** (imu_t) Raw reference or truth IMU used for manufacturing calibration and testing. Input from testbed. */
#define IMX_DID_IMU3_RAW                    (eDataIDs)96 /** (imu3_t) Triple IMU data calibrated from DID_IMU3_UNCAL.  We recommend use of DID_IMU or DID_PIMU as they are oversampled and contain less noise. */
#define IMX_DID_IMU_RAW                     (eDataIDs)97 /** (imu_t) IMU data averaged from DID_IMU3_RAW.  Use this IMU data for output data rates faster than DID_FLASH_CONFIG_IMX.startupNavDtMs.  Otherwise we recommend use of DID_IMU or DID_PIMU as they are oversampled and contain less noise. */

// Adding a new data id?
// 1] Add it above and increment the previous number, include the matching data structure type in the comments
// 2] Add flip doubles and flip strings entries in data_sets.c
// 3] Add data id to ISDataMappings.cpp
// 4] Increment DID_COUNT
// 5) Update the DIDs in IS-src/python/src/ci_hdw/data_sets.py
// 6] Test!

/** Count of data ids (including null data id 0) - MUST BE MULTPLE OF 4 and larger than last DID number! */
#define IMX_DID_COUNT		(eDataIDs)120	// Used in SDK
#define IMX_DID_COUNT_UINS	(eDataIDs)100	// Used in uINS

// END DATA IDENTIFIERS --------------------------------------------------------------------------

/** Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */
// #define PROTOCOL_VERSION_CHAR0 1        // Major (in ISComm.h)
// #define PROTOCOL_VERSION_CHAR1 0
#define PROTOCOL_VERSION_CHAR2 (0x000000FF&IMX_DID_COUNT_UINS)
#define PROTOCOL_VERSION_CHAR3 9         // Minor (in data_sets.h)

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
#define RMC_BITS_IMU                    0x0000000000000010      // DID_FLASH_CONFIG_IMX.startupNavDtMs (4ms default)
#define RMC_BITS_PIMU                   0x0000000000000020      // "
#define RMC_BITS_BAROMETER              0x0000000000000040      // ~8ms
#define RMC_BITS_MAGNETOMETER           0x0000000000000080      // ~10ms
// #define RMC_BITS_UNUSED              0x0000000000000100
// #define RMC_BITS_UNUSED              0x0000000000000200 
#define RMC_BITS_GPS1_POS               0x0000000000000400      // DID_FLASH_CONFIG_IMX.startupGpsDtMs (200ms default)
#define RMC_BITS_GPS2_POS               0x0000000000000800      // "
#define RMC_BITS_GPS1_RAW               0x0000000000001000      // "
#define RMC_BITS_GPS2_RAW               0x0000000000002000      // "
#define RMC_BITS_GPS1_SAT               0x0000000000004000      // 1s
#define RMC_BITS_GPS2_SAT               0x0000000000008000      // "
#define RMC_BITS_GPS_BASE_RAW           0x0000000000010000      // 
#define RMC_BITS_STROBE_IN_TIME         0x0000000000020000      // On strobe input event
#define RMC_BITS_DIAGNOSTIC_MESSAGE     0x0000000000040000
#define RMC_BITS_IMU3_UNCAL             0x0000000000080000      // DID_FLASH_CONFIG_IMX.startupImuDtMs (1ms default)
#define RMC_BITS_GPS1_VEL               0x0000000000100000      // DID_FLASH_CONFIG_IMX.startupGpsDtMs (200ms default)
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
#define RMC_BITS_GPS1_RTK_HDG_REL       0x0000001000000000      // DID_FLASH_CONFIG_IMX.startupGpsDtMs (200ms default)
#define RMC_BITS_GPS1_RTK_HDG_MISC      0x0000002000000000      // "
#define RMC_BITS_REFERENCE_IMU          0x0000004000000000		// DID_FLASH_CONFIG_IMX.startupNavDtMs
#define RMC_BITS_REFERENCE_PIMU         0x0000008000000000		// "
#define RMC_BITS_IMU3_RAW               0x0000010000000000
#define RMC_BITS_IMU_RAW                0x0000020000000000

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

/** (DID_FLASH_CONFIG_IMX) Configuration data
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

    /** Navigation filter (system output data) update period in milliseconds set on startup. 1ms minimum (1KHz max). */
    uint32_t				startupNavDtMs;

    /** Serial port 0 baud rate in bits per second */
    uint32_t				ser0BaudRate;

    /** Serial port 1 baud rate in bits per second */
    uint32_t				ser1BaudRate;

    /** Rotation in radians about the X, Y, Z axes from Sensor Frame to Intermediate Output Frame.  Order applied: Z, Y, X. */
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
	
	/** GPS measurement (system input data) update period in milliseconds set on startup. 200ms minimum (5Hz max). */
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

} nvm_cfg_imx_t;

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

POP_PACK

/** GNSS satellite system signal constellation (used with nvm_cfg_imx_t.gnssSatSigConst) */
enum eGnssSatSigConst
{
	/*! GPS  */
	GNSS_SAT_SIG_CONST_GPS                              = (uint16_t)0x0003,
	/*! QZSS  */
	GNSS_SAT_SIG_CONST_QZSS                             = (uint16_t)0x000C,
	/*! Galileo  */
	GNSS_SAT_SIG_CONST_GAL                              = (uint16_t)0x0030,
	/*! BeiDou  */
	GNSS_SAT_SIG_CONST_BDS                              = (uint16_t)0x00C0,
	/*! GLONASS  */
	GNSS_SAT_SIG_CONST_GLO                              = (uint16_t)0x0300,
	/*! SBAS  */
	GNSS_SAT_SIG_CONST_SBAS                             = (uint16_t)0x1000,
	
	/*! GNSS default */
	GNSS_SAT_SIG_CONST_DEFAULT = \
		GNSS_SAT_SIG_CONST_GPS | \
		GNSS_SAT_SIG_CONST_SBAS | \
		GNSS_SAT_SIG_CONST_QZSS | \
		GNSS_SAT_SIG_CONST_GAL | \
		GNSS_SAT_SIG_CONST_GLO | \
		GNSS_SAT_SIG_CONST_BDS
};

/** System Configuration (used with DID_FLASH_CONFIG_IMX.sysCfgBits) */
enum eSysConfigBits
{
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

};

/** RTK Configuration (used with nvm_cfg_imx_t.RTKCfgBits) */
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

/** Sensor Configuration (used with nvm_cfg_imx_t.sensorConfig) */
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
	SENSOR_CFG_ACC_FS_4G				= (int)0x00000001,
	SENSOR_CFG_ACC_FS_8G				= (int)0x00000002,
	SENSOR_CFG_ACC_FS_16G				= (int)0x00000003,
	SENSOR_CFG_ACC_FS_MASK				= (int)0x0000000C,
	SENSOR_CFG_ACC_FS_OFFSET			= (int)2,
	
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

/** IO configuration (used with nvm_cfg_imx_t.ioConfig) */
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

	/** Unused bits */

	/** External GPS TIMEPULSE source */
	IO_CFG_GPS_TIMEPUSE_SOURCE_BITMASK			= (int)0x0000E000,	
	/** 0 = internal, 1 = disabled, 2 = G2_PIN6, 3 = G5_PIN9, 4 = G8_PIN12, 5 = G9_PIN13 */
	IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET			= (int)13,
	IO_CFG_GPS_TIMEPUSE_SOURCE_MASK				= (int)0x00000007,
	IO_CFG_GPS_TIMEPUSE_SOURCE_DISABLED			= (int)0,
	IO_CFG_GPS_TIMEPUSE_SOURCE_GPS1_PPS_PIN20	= (int)1,
	IO_CFG_GPS_TIMEPUSE_SOURCE_GPS2_PPS			= (int)2,
	IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G2_PIN6	= (int)3,
	IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G5_PIN9	= (int)4,
	IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G8_PIN12	= (int)5,
	IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G9_PIN13	= (int)6,
#define SET_STATUS_OFFSET_MASK(result,val,offset,mask)	{ (result) &= ~((mask)<<(offset)); (result) |= ((val)<<(offset)); }
	
#define IO_CFG_GPS_TIMEPUSE_SOURCE(ioConfig) ((ioConfig>>IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET)&IO_CFG_GPS_TIMEPUSE_SOURCE_MASK)
	
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

#define IO_CONFIG_GPS1_SOURCE(ioConfig) ((ioConfig>>IO_CONFIG_GPS1_SOURCE_OFFSET)&IO_CONFIG_GPS_SOURCE_MASK)
#define IO_CONFIG_GPS2_SOURCE(ioConfig) ((ioConfig>>IO_CONFIG_GPS2_SOURCE_OFFSET)&IO_CONFIG_GPS_SOURCE_MASK)
#define IO_CONFIG_GPS1_TYPE(ioConfig)	((ioConfig>>IO_CONFIG_GPS1_TYPE_OFFSET)&IO_CONFIG_GPS_TYPE_MASK)
#define IO_CONFIG_GPS2_TYPE(ioConfig)	((ioConfig>>IO_CONFIG_GPS2_TYPE_OFFSET)&IO_CONFIG_GPS_TYPE_MASK)

	/** IMU 1 disable */	
	IO_CONFIG_IMU_1_DISABLE						= (int)0x10000000,
	/** IMU 2 disable */
	IO_CONFIG_IMU_2_DISABLE						= (int)0x20000000,
	/** IMU 3 disable */
	IO_CONFIG_IMU_3_DISABLE						= (int)0x40000000,

	/** Unused bits */
};

#define IO_CONFIG_DEFAULT 	(IO_CONFIG_G1G2_DEFAULT | IO_CONFIG_G5G8_DEFAULT | IO_CONFIG_G6G7_DEFAULT | IO_CONFIG_G9_DEFAULT | (IO_CONFIG_GPS_SOURCE_ONBOARD_1<<IO_CONFIG_GPS1_SOURCE_OFFSET) | (IO_CONFIG_GPS_SOURCE_ONBOARD_2<<IO_CONFIG_GPS2_SOURCE_OFFSET))

enum ePlatformConfig
{
	// IMX Carrier Board
	PLATFORM_CFG_TYPE_MASK                      = (int)0x0000001F,
	PLATFORM_CFG_TYPE_NONE                      = (int)0,		// IMX-5 default
	PLATFORM_CFG_TYPE_NONE_ONBOARD_G2           = (int)1,		// uINS-3 default
	PLATFORM_CFG_TYPE_RUG1                      = (int)2,
	PLATFORM_CFG_TYPE_RUG2_0_G1                 = (int)3,
	PLATFORM_CFG_TYPE_RUG2_0_G2                 = (int)4,
	PLATFORM_CFG_TYPE_RUG2_1_G0                 = (int)5,	    // PCB RUG-2.1, Case RUG-3.  GPS1 timepulse on G9
	PLATFORM_CFG_TYPE_RUG2_1_G1                 = (int)6,       // "
	PLATFORM_CFG_TYPE_RUG2_1_G2                 = (int)7,       // "
	PLATFORM_CFG_TYPE_RUG3_G0                   = (int)8,       // PCB RUG-3.x.  GPS1 timepulse on GPS1_PPS TIMESYNC (pin 20)
	PLATFORM_CFG_TYPE_RUG3_G1                   = (int)9,       // "
	PLATFORM_CFG_TYPE_RUG3_G2                   = (int)10,      // "
	PLATFORM_CFG_TYPE_EVB2_G2                   = (int)11,
	PLATFORM_CFG_TYPE_EVB3                      = (int)12,
	PLATFORM_CFG_TYPE_IG1_0_G2                  = (int)13,      // PCB IG-1.0.  GPS1 timepulse on G8
	PLATFORM_CFG_TYPE_IG1_G1                    = (int)14,      // PCB IG-1.1 and later.  GPS1 timepulse on GPS1_PPS TIMESYNC (pin 20)
	PLATFORM_CFG_TYPE_IG1_G2                    = (int)15,
	PLATFORM_CFG_TYPE_LAMBDA_G1                 = (int)16,		// Enable UBX output on Lambda for testbed
	PLATFORM_CFG_TYPE_LAMBDA_G2                 = (int)17,		// "
	PLATFORM_CFG_TYPE_TESTBED_G1_W_LAMBDA       = (int)18,		// Enable UBX input from Lambda
	PLATFORM_CFG_TYPE_TESTBED_G2_W_LAMBDA       = (int)19,		// "
	PLATFORM_CFG_TYPE_COUNT                     = (int)20,

	// Presets
	PLATFORM_CFG_PRESET_MASK                    = (int)0x0000FF00,
	PLATFORM_CFG_PRESET_OFFSET                  = (int)8,

	// RUG-3 - Presets
	PLATFORM_CFG_RUG3_PRESET__0__PRESETS_DISABLED								= 0,	// Don't use presets.  IOEXP_BITS can be set directly.
	PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GPS1			= 1,	// RUG-3-G0 default
	PLATFORM_CFG_RUG3_PRESET__2__S0_TTL_7_9_____CAN_11_12______S1_GPS1			= 2,
	PLATFORM_CFG_RUG3_PRESET__3__S0_TTL_7_9_____S2_TTL_8_10____S1_GPS1			= 3,
	PLATFORM_CFG_RUG3_PRESET__4__S0_RS232_7_9___S1_RS232_8_10__S2_GPS1			= 4,
	PLATFORM_CFG_RUG3_PRESET__5__S2_RS485_7_8_9_10_____________S2_GPS1__S0_GPS2	= 5,
	PLATFORM_CFG_RUG3_PRESET__6__SPI_7_8_9_10__________________S2_GPS1__S0_GPS2	= 6,
	PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GPS1__S0_GPS2	= 7,	// RUG-3-G2 default
	PLATFORM_CFG_RUG3_PRESET__8_________________CAN_11_12______S1_GPS1__S0_GPS2	= 8,
	PLATFORM_CFG_RUG3_PRESET__9__S2_TTL_8_10___________________S1_GPS1__S0_GPS2	= 9,
	PLATFORM_CFG_RUG3_PRESET__COUNT												= 10,

	PLATFORM_CFG_RUG3_PRESET__G0_DEFAULT		= PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GPS1,
	PLATFORM_CFG_RUG3_PRESET__G2_DEFAULT		= PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GPS1__S0_GPS2,

	// RUG-3 - I/O Expander disabled if platform type is != PLATFORM_CFG_TYPE_RUG3_x.
	PLATFORM_CFG_RUG3_IOEXP_BIT_MASK            = (int)0x00FF0000,
	PLATFORM_CFG_RUG3_IOEXP_BIT_OFFSET          = (int)16,

	RUG3_IOEXP_BIT_OFFSET_n232_485    			= (int)0,
	RUG3_IOEXP_BIT_OFFSET_n232_TTL    			= (int)1,
	RUG3_IOEXP_BIT_OFFSET_nRS_CAN     			= (int)2,
	RUG3_IOEXP_BIT_OFFSET_nGPS2_RS    			= (int)3,
	RUG3_IOEXP_BIT_OFFSET_nSPIEN      			= (int)4,
	RUG3_IOEXP_BIT_OFFSET_nSPI_SER    			= (int)5,
	RUG3_IOEXP_BIT_OFFSET_nGPSRST     			= (int)6,
};

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

#ifdef __cplusplus
}
#endif

#endif  // IMX_DATA_SETS_H

