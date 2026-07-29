/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file data_sets.h
 * @brief Canonical on-wire and in-memory data structure definitions for the IMX/GPX firmware interface.
 *
 * This header is the single source of truth for every Data Identification Number (DID) payload
 * exchanged between host software and Inertial Sense IMX/GPX devices, plus the in-memory structs,
 * unions, and enums those payloads are unpacked into. It covers, among others:
 *  - INS/AHRS navigation solutions (@ref ins_1_t, @ref ins_2_t, @ref ins_3_t, @ref ins_4_t)
 *  - Raw and preintegrated IMU data (@ref imu_t, @ref pimu_t, @ref imus_t)
 *  - GNSS position, velocity, satellite, signal, and RTK data (@ref gnss_pos_t, @ref gnss_vel_t,
 *    @ref gnss_sat_t, @ref gnss_sig_t, @ref gnss_rtk_rel_t, @ref gnss_rtk_misc_t)
 *  - Device configuration and flash-backed settings (@ref nvm_flash_cfg_t)
 *  - System status, built-in test, and diagnostic structures
 *  - EVB (evaluation board) and GPX-specific status/configuration structures
 *
 * DID values are defined above each structure as DID_* macros (see @ref eDataIDs) and are never
 * reordered or reused, since they are part of the wire protocol. Each DID macro's trailing comment
 * names the payload type it carries, e.g. DID_PIMU carries a @ref pimu_t.
 *
 * Unless otherwise noted: angles are in radians, angular rates are in radians/second, linear
 * accelerations are in meters/second^2, linear velocities are in meters/second, positions given as
 * LLA are in (degrees, degrees, meters), and ECEF/NED positions are in meters. INS outputs use the
 * NED (North-East-Down) or ECEF frame as named in the type; IMU/PIMU data is in the body/IMU frame.
 * All multi-byte fields are little-endian on the wire.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef DATA_SETS_H
#define DATA_SETS_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "core/types.h"
#include "core/base_port.h"
#include "ISConstants.h"
#include "rtk_defines.h"

#ifdef __cplusplus
extern "C" {
#endif


// *****************************************************************************
// ****** InertialSense binary message Data Identification Numbers (DIDs) ******
// ******                                                                 ******
// ****** NEVER REORDER THESE VALUES!                                     ******
// *****************************************************************************
/** Data identifiers - these are unsigned int and are declared as preprocessor macros rather than an enum, because enum is signed according to the C standard */
typedef uint32_t eDataIDs;

#define DID_NULL                        (eDataIDs)0     /**< NULL (INVALID) */
#define DID_DEV_INFO                    (eDataIDs)1     /**< (dev_info_t) Device information */
#define DID_IMX_DEV_INFO                (DID_DEV_INFO)  /**< Alias of DID_DEV_INFO used on IMX firmware */
#define DID_SYS_FAULT                   (eDataIDs)2     /**< (system_fault_t) System fault information. This is broadcast automatically every 10s if a critical fault is detected. */
#define DID_PIMU                        (eDataIDs)3     /**< (pimu_t) Preintegrated IMU (a.k.a. Coning and Sculling integral) in body/IMU frame.  Updated at IMU rate. Also know as delta theta delta velocity, or preintegrated IMU (PIMU). For clarification, the name "Preintegrated IMU" or "PIMU" throughout our User Manual. This data is integrated from the IMU data at the IMU update rate (startupImuDtMs, default 1ms).  The PIMU integration period (dt) and INS NAV update data period are the same.  DID_FLASH_CONFIG.startupNavDtMs sets the NAV output period at startup.  The minimum NAV update and output periods are found here:  https://docs.inertialsense.com/user-manual/imx/application-config/imu_ins_gnss_configuration/#navigation-update-and-output-periods.  If a faster output data rate for IMU is desired, DID_IMU_RAW can be used instead. PIMU data acts as a form of compression, adding the benefit of higher integration rates for slower output data rates, preserving the IMU data without adding filter delay and addresses antialiasing. It is most effective for systems that have higher dynamics and lower communications data rates.  The minimum data period is DID_FLASH_CONFIG.startupImuDtMs or 4, whichever is larger (250Hz max). The PIMU value can be converted to IMU by dividing PIMU by dt (i.e. IMU = PIMU / dt)  */
#define DID_INS_1                       (eDataIDs)4     /**< (ins_1_t) INS output: euler rotation w/ respect to NED, NED position from reference LLA. */
#define DID_INS_2                       (eDataIDs)5     /**< (ins_2_t) INS output: quaternion rotation w/ respect to NED, ellipsoid altitude */
#define DID_GNSS1_RCVR_POS              (eDataIDs)6     /**< (gnss_pos_t) GNSS 1 position data from GNSS receiver. */
#define DID_SYS_CMD                     (eDataIDs)7     /**< (system_command_t) System commands. Both the command and invCommand fields must be set at the same time for a command to take effect. */
#define DID_NMEA_BCAST_PERIOD           (eDataIDs)8     /**< (nmea_msgs_t) Set broadcast periods for NMEA messages */
#define DID_RMC                         (eDataIDs)9     /**< (rmc_t) Realtime Message Controller (RMC). The data sets available through RMC are driven by the availability of the data. The RMC provides updates from various data sources (i.e. sensors) as soon as possible with minimal latency. Several of the data sources (sensors) output data at different data rates that do not all correspond. The RMC is provided so that broadcast of sensor data is done as soon as it becomes available. All RMC messages can be enabled using the standard Get Data packet format. */
#define DID_SYS_PARAMS                  (eDataIDs)10    /**< (sys_params_t) System parameters / info */
#define DID_SYS_SENSORS                 (eDataIDs)11    /**< (sys_sensors_t) System sensor information */
#define DID_FLASH_CONFIG                (eDataIDs)12    /**< (nvm_flash_cfg_t) Flash memory configuration */
#define DID_GNSS1_POS                   (eDataIDs)13    /**< (gnss_pos_t) GNSS 1 position data.  This comes from DID_GNSS1_RCVR_POS or DID_GNSS1_RTK_POS, depending on whichever is more accurate. */
#define DID_GNSS2_POS                   (eDataIDs)14    /**< (gnss_pos_t) GNSS 2 position data */
#define DID_GNSS1_SAT                   (eDataIDs)15    /**< (gnss_sat_t) GNSS 1 GNSS satellite information: sat identifiers, carrier to noise ratio, elevation and azimuth angles, pseudo range residual. */
#define DID_GNSS2_SAT                   (eDataIDs)16    /**< (gnss_sat_t) GNSS 2 GNSS satellite information: sat identifiers, carrier to noise ratio, elevation and azimuth angles, pseudo range residual. */
#define DID_GNSS1_VERSION               (eDataIDs)17    /**< (gnss_version_t) GNSS 1 version info */
#define DID_GNSS2_VERSION               (eDataIDs)18    /**< (gnss_version_t) GNSS 2 version info */
#define DID_MAG_CAL                     (eDataIDs)19    /**< (mag_cal_t) Magnetometer calibration */
#define DID_IMUS                        (eDataIDs)20    /**< (imus_t) Multiple inertial measurement units data down-sampled from IMU rate (DID_FLASH_CONFIG.startupImuDtMs (1KHz)) to navigation update rate (DID_FLASH_CONFIG.startupNavDtMs) as an anti-aliasing filter to reduce noise and preserve accuracy.  Minimum data period is DID_FLASH_CONFIG.startupNavDtMs (1KHz max).  Enabled by using RMC preset "Allan Variance IMUs".  Enabling this message adds processing overhead to sensor RTOS task. */
#define DID_GNSS1_RTK_POS_REL           (eDataIDs)21    /**< (gnss_rtk_rel_t) RTK precision position base to rover relative info. */
#define DID_GNSS1_RTK_POS_MISC          (eDataIDs)22    /**< (gnss_rtk_misc_t) RTK precision position related data. */
#define DID_FEATURE_BITS                (eDataIDs)23    /**< INTERNAL USE ONLY (feature_bits_t) */
#define DID_SENSORS_UCAL                (eDataIDs)24    /**< INTERNAL USE ONLY (sensors_w_temp_t) Uncalibrated IMU output. */
#define DID_SENSORS_TCAL                (eDataIDs)25    /**< INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated IMU output. */
#define DID_SENSORS_TC_BIAS             (eDataIDs)26    /**< INTERNAL USE ONLY (sensors_t) */
#define DID_GNSS2_TIMEPULSE             (eDataIDs)27    /**< (gnss_timepulse_t) GNSS2 PPS time synchronization. */
#define DID_SENSORS_ADC                 (eDataIDs)28    /**< INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_SCOMP                       (eDataIDs)29    /**< INTERNAL USE ONLY (sensor_compensation_t) */
#define DID_GNSS1_VEL                   (eDataIDs)30    /**< (gnss_vel_t) GNSS 1 velocity data */
#define DID_GNSS2_VEL                   (eDataIDs)31    /**< (gnss_vel_t) GNSS 2 velocity data */
#define DID_HDW_PARAMS                  (eDataIDs)32    /**< INTERNAL USE ONLY (hdw_params_t) */
#define DID_NVR_MANAGE_USERPAGE         (eDataIDs)33    /**< INTERNAL USE ONLY (nvr_manage_t) */
#define DID_NVR_USERPAGE_SN             (eDataIDs)34    /**< INTERNAL USE ONLY (nvm_group_sn_t) */
#define DID_NVR_USERPAGE_G0             (eDataIDs)35    /**< INTERNAL USE ONLY (nvm_group_0_t) */
#define DID_NVR_USERPAGE_G1             (eDataIDs)36    /**< INTERNAL USE ONLY (nvm_group_1_t) */
#define DID_DEBUG_STRING                (eDataIDs)37    /**< INTERNAL USE ONLY (debug_string_t) */
#define DID_RTOS_INFO                   (eDataIDs)38    /**< (rtos_info_t) RTOS information. */
#define DID_DEBUG_ARRAY                 (eDataIDs)39    /**< INTERNAL USE ONLY (debug_array_t) */
#define DID_SENSORS_MCAL                (eDataIDs)40    /**< INTERNAL USE ONLY (sensors_w_temp_t) Temperature compensated and motion calibrated IMU output. */
#define DID_GNSS1_TIMEPULSE             (eDataIDs)41    /**< (gnss_timepulse_t) GNSS1 PPS time synchronization. */
#define DID_CAL_SC                      (eDataIDs)42    /**< INTERNAL USE ONLY (sensor_cal_t) */
#define DID_UNUSED_43                   (eDataIDs)43    /**< unused */
#define DID_CANFD_CONFIG                (eDataIDs)44    /**< (can_config_t) CAN FD configuration: FD message broadcast rates, transmit addresses, and baud rate. Shares the same data structure as DID_CAN_CONFIG. */
#define DID_GNSS1_SIG                   (eDataIDs)45    /**< (gnss_sig_t) GNSS 1 GNSS signal information. */
#define DID_SENSORS_ADC_SIGMA           (eDataIDs)46    /**< INTERNAL USE ONLY (sys_sensors_adc_t) */
#define DID_REFERENCE_MAGNETOMETER      (eDataIDs)47    /**< (magnetometer_t) Reference or truth magnetometer used for manufacturing calibration and testing */
#define DID_INL2_STATES                 (eDataIDs)48    /**< (inl2_states_t) INS Extended Kalman Filter (EKF) states */
#define DID_INL2_COVARIANCE_LD          (eDataIDs)49    /**< (INL2_COVARIANCE_LD_ARRAY_SIZE) */
#define DID_INL2_STATUS                 (eDataIDs)50    /**< (inl2_status_t) */
#define DID_INL2_MISC                   (eDataIDs)51    /**< (inl2_misc_t) */
#define DID_MAGNETOMETER                (eDataIDs)52    /**< (magnetometer_t) Magnetometer sensor output */
#define DID_BAROMETER                   (eDataIDs)53    /**< (barometer_t) Barometric pressure sensor data */
#define DID_GNSS1_RTK_POS               (eDataIDs)54    /**< (gnss_pos_t) GNSS RTK position data */
#define DID_ROS_COVARIANCE_POSE_TWIST   (eDataIDs)55    /**< (ros_covariance_pose_twist_t) INL2 EKF 6x6 covariance matrices packed in arrays containing their elements on main diagonal and below */
#define DID_COMMUNICATIONS_LOOPBACK     (eDataIDs)56    /**< INTERNAL USE ONLY - Unit test for communications manager  */
#define DID_IMUS_UNCAL                  (eDataIDs)57    /**< INTERNAL USE ONLY (imus_t) Uncalibrated multiple IMU data.  We recommend use of DID_IMU or DID_PIMU as they are calibrated and oversampled and contain less noise.  Minimum data period is DID_FLASH_CONFIG.startupImuDtMs or 4, whichever is larger (250Hz max). */
#define DID_IMU                         (eDataIDs)58    /**< (imu_t) Inertial measurement unit data down-sampled from IMU rate (DID_FLASH_CONFIG.startupImuDtMs (1KHz)) to navigation update rate (DID_FLASH_CONFIG.startupNavDtMs) as an anti-aliasing filter to reduce noise and preserve accuracy.  Minimum data period is DID_FLASH_CONFIG.startupNavDtMs (1KHz max).  */
#define DID_INL2_MAG_OBS_INFO           (eDataIDs)59    /**< (inl2_mag_obs_info_t) INL2 magnetometer calibration information. */
#define DID_GNSS_BASE_RAW               (eDataIDs)60    /**< (gnss_raw_t) GNSS raw data for base station (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_GNSS_RTK_OPT                (eDataIDs)61    /**< (gnss_rtk_opt_t) RTK options - requires little endian CPU. */
#define DID_REFERENCE_PIMU              (eDataIDs)62    /**< (pimu_t) Reference or truth IMU used for manufacturing calibration and testing */
#define DID_MANUFACTURING_INFO          (eDataIDs)63    /**< INTERNAL USE ONLY (manufacturing_info_t) Manufacturing info */
#define DID_BIT                         (eDataIDs)64    /**< (bit_t) System built-in self-test */
#define DID_INS_3                       (eDataIDs)65    /**< (ins_3_t) Inertial navigation data with quaternion NED to body rotation and ECEF position. */
#define DID_INS_4                       (eDataIDs)66    /**< (ins_4_t) INS output: quaternion rotation w/ respect to ECEF, ECEF position. */
#define DID_INL2_NED_SIGMA              (eDataIDs)67    /**< (inl2_ned_sigma_t) Standard deviation of INL2 EKF estimates in the NED frame. */
#define DID_STROBE_IN_TIME              (eDataIDs)68    /**< (strobe_in_time_t) Timestamp for input strobe. */
#define DID_GNSS1_RAW                   (eDataIDs)69    /**< (gnss_raw_t) GNSS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_GNSS2_RAW                   (eDataIDs)70    /**< (gnss_raw_t) GNSS raw data for rover (observation, ephemeris, etc.) - requires little endian CPU. The contents of data can vary for this message and are determined by dataType field. RTK positioning or RTK compassing must be enabled to stream this message. */
#define DID_WHEEL_ENCODER               (eDataIDs)71    /**< (wheel_encoder_t) Wheel encoder data to be fused with GNSS-INS measurements, set DID_GROUND_VEHICLE for configuration before sending this message */
#define DID_DIAGNOSTIC_MESSAGE          (eDataIDs)72    /**< (diag_msg_t) Diagnostic message */
#define DID_SURVEY_IN                   (eDataIDs)73    /**< (survey_in_t) Survey in, used to determine position for RTK base station. Base correction output cannot run during a survey and will be automatically disabled if a survey is started. */
#define DID_UNUSED_74                   (eDataIDs)74    /**< unused */
#define DID_PORT_MONITOR                (eDataIDs)75    /**< (port_monitor_t) Data rate and status monitoring for each communications port. */
#define DID_RTK_STATE                   (eDataIDs)76    /**< INTERNAL USE ONLY (rtk_state_t) */
#define DID_RTK_PHASE_RESIDUAL          (eDataIDs)77    /**< INTERNAL USE ONLY (rtk_residual_t) */
#define DID_RTK_CODE_RESIDUAL           (eDataIDs)78    /**< INTERNAL USE ONLY (rtk_residual_t) */
#define DID_RTK_DEBUG                   (eDataIDs)79    /**< INTERNAL USE ONLY (rtk_debug_t) */
#define DID_EVB_STATUS                  (eDataIDs)80    /**< (evb_status_t) EVB monitor and log control interface. */
#define DID_EVB_FLASH_CFG               (eDataIDs)81    /**< (evb_flash_cfg_t) EVB configuration. */
#define DID_EVB_DEBUG_ARRAY             (eDataIDs)82    /**< INTERNAL USE ONLY (debug_array_t) */
#define DID_EVB_RTOS_INFO               (eDataIDs)83    /**< (evb_rtos_info_t) EVB-2 RTOS information. */
#define DID_GNSS2_SIG                   (eDataIDs)84    /**< (gnss_sig_t) GNSS 2 signal information. */
#define DID_IMU_MAG                     (eDataIDs)85    /**< (imu_mag_t) DID_IMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define DID_PIMU_MAG                    (eDataIDs)86    /**< (pimu_mag_t) DID_PIMU + DID_MAGNETOMETER. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously. */
#define DID_GROUND_VEHICLE              (eDataIDs)87    /**< (ground_vehicle_t) Static configuration for wheel transform measurements. */
#define DID_POSITION_MEASUREMENT        (eDataIDs)88    /**< (pos_measurement_t) External position estimate */
#define DID_RTK_DEBUG_2                 (eDataIDs)89    /**< INTERNAL USE ONLY (rtk_debug_2_t) */
#define DID_CAN_CONFIG                  (eDataIDs)90    /**< (can_config_t) Addresses for CAN messages*/
#define DID_GNSS2_RTK_CMP_REL           (eDataIDs)91    /**< (gnss_rtk_rel_t) Dual GNSS RTK compassing / moving base to rover (GNSS 1 to GNSS 2) relative info. */
#define DID_GNSS2_RTK_CMP_MISC          (eDataIDs)92    /**< (gnss_rtk_misc_t) RTK Dual GNSS RTK compassing related data. */
#define DID_EVB_DEV_INFO                (eDataIDs)93    /**< (dev_info_t) EVB device information */
#define DID_INFIELD_CAL                 (eDataIDs)94    /**< (infield_cal_t) Measure and correct IMU calibration error.  Estimate INS rotation to align INS with vehicle. */
#define DID_REFERENCE_IMU               (eDataIDs)95    /**< (imu_t) Raw reference or truth IMU used for manufacturing calibration and testing. Input from testbed. */
#define DID_IMUS_RAW                    (eDataIDs)96    /**< (imus_t) Multiple IMU data calibrated from DID_IMUS_UNCAL.  We recommend use of DID_IMU or DID_PIMU as they are oversampled and contain less noise. */
#define DID_IMU_RAW                     (eDataIDs)97    /**< (imu_t) IMU data averaged from DID_IMUS_RAW.  Use this IMU data for output data rates faster than DID_FLASH_CONFIG.startupNavDtMs.  Otherwise we recommend use of DID_IMU or DID_PIMU as they are oversampled and contain less noise. */
#define DID_FIRMWARE_UPDATE             (eDataIDs)98    /**< (firmware_payload_t) firmware update payload */
#define DID_RUNTIME_PROFILER            (eDataIDs)99    /**< INTERNAL USE ONLY (runtime_profiler_t) System runtime profiler */
#define DID_CAL_TEMP_COMP_GYR           (eDataIDs)100   /**< INTERNAL USE ONLY (sensor_tcal_group_t) */
#define DID_CAL_TEMP_COMP_ACC           (eDataIDs)101   /**< INTERNAL USE ONLY (sensor_tcal_group_t) */
#define DID_CAL_TEMP_COMP_MAG           (eDataIDs)102   /**< INTERNAL USE ONLY (sensor_tcal_group_t) */
#define DID_CAL_MOTION_GYR              (eDataIDs)103   /**< INTERNAL USE ONLY (sensor_mcal_group_t) */
#define DID_CAL_MOTION_ACC              (eDataIDs)104   /**< INTERNAL USE ONLY (sensor_mcal_group_t) */
#define DID_CAL_MOTION_MAG              (eDataIDs)105   /**< INTERNAL USE ONLY (sensor_mcal_group_t) */
#define DID_EXT_POS                     (eDataIDs)106   /**< (ext_pos_t) External position observation */
#define DID_EXT_VEL                     (eDataIDs)107   /**< (ext_vel_t) External velocity observation */
#define DID_EVENT                       (eDataIDs)119   /**< INTERNAL USE ONLY (did_event_t)*/

#define DID_GPX_FIRST                   (eDataIDs)120   /**< First of GPX DIDs */
#define DID_GPX_DEV_INFO                (eDataIDs)120   /**< (dev_info_t) GPX device information */
#define DID_GPX_FLASH_CFG               (eDataIDs)121   /**< (gpx_flash_cfg_t) GPX flash configuration */
#define DID_GPX_RTOS_INFO               (eDataIDs)122   /**< (gpx_rtos_info_t) GPX RTOs info */
#define DID_GPX_STATUS                  (eDataIDs)123   /**< (gpx_status_t) GPX status */
#define DID_GPX_DEBUG_ARRAY             (eDataIDs)124   /**< (debug_array_t) GPX debug */
#define DID_GPX_BIT                     (eDataIDs)125   /**< (gpx_bit_t) GPX BIT test */
#define DID_GPX_RMC                     (eDataIDs)126   /**< (rmc_t) GPX rmc  */
#define DID_GPX_PORT_MONITOR            (eDataIDs)127   /**< (port_monitor_t) Data rate and status monitoring for each communications port. */
#define DID_GPX_SYS_FAULT               (eDataIDs)128   /**< (system_fault_t) System fault information. This is broadcast automatically every 10s if a critical fault is detected. */
#define DID_GPX_LAST                    128             /**< Last of GPX DIDs */

// Adding a new data id?
// 1] Add it above and increment the previous number, include the matching data structure type in the comments
// 2] Add flip doubles and flip strings entries in data_sets.c
// 3] Add data id to ISDataMappings.cpp
// 4] Increment DID_COUNT
// 5) Update the DIDs in IS-src/python/src/ci_hdw/data_sets.py
// 6] Test!

/** Count of data ids (including null data id 0) - MUST BE MULTPLE OF 4 and larger than last DID number! */
#define DID_COUNT       (eDataIDs)132   //!< Used in SDK

/** Maximum number of data ids */
#define DID_MAX_COUNT   256  //!< Maximum number of data ids

// END DATA IDENTIFIERS --------------------------------------------------------------------------

/** Maximum number of satellite channels */
#define MAX_NUM_SATELLITES  50

/** Maximum number of satellite signals */
#define MAX_NUM_SAT_SIGNALS 100

/** Maximum length of device info manufacturer string (must be a multiple of 4) */
#define DEVINFO_MANUFACTURER_STRLEN 24
#define DEVINFO_ADDINFO_STRLEN      24

/** Communications Protocol Version. See release notes. */

// Increment w/ breaking changes (in ISComm.cpp) that prevent backwards compatibility with older protocols.
// #define PROTOCOL_VERSION_CHAR0   .   // Breaking changes (Packet)        (defined in ISComm.h)
#define PROTOCOL_VERSION_CHAR1      2   // Breaking changes (Payload)

// Increment w/ non-breaking changes (in data_sets.h) that would still backward compatibility with older protocols
// #define PROTOCOL_VERSION_CHAR2   .   // Non-breaking changes (Packet):   (defined in ISComm.h)
#define PROTOCOL_VERSION_CHAR3      0   // Non-breaking changes (Payload):

/** Rtk rover receiver index */
#define RECEIVER_INDEX_GNSS1            1   // DO NOT CHANGE
#define RECEIVER_INDEX_EXTERNAL_BASE    2   // DO NOT CHANGE
#define RECEIVER_INDEX_GNSS2            3   // DO NOT CHANGE

// Version 1.3 of sensor calibration format supports up to 3 IMUs and 2 mags, with separate orthonormalization and bias calibration for each device
#define NUM_IMU_DEVICES_V1P3    3
#define NUM_MAG_DEVICES_V1P3    2
// Version 1.4 of sensor calibration format supports up to 5 IMUs and 1 mag, with separate orthonormalization and bias calibration for each device
#define NUM_IMU_DEVICES_V1P4    5
#define NUM_MAG_DEVICES_V1P4    1
// Per-build-target native counts. SN-7966: IMX-5 hardware is permanently Cal v1.3,
// IMX-6 (and host SDK) is permanently Cal v1.4. Host code (no IMX_5/IMX_6 define) uses v1.4.
#define MAX_IMU_DEVICES         NUM_IMU_DEVICES_V1P4
#define MAX_MAG_DEVICES         NUM_MAG_DEVICES_V1P4

/**
 * @brief INS/AHRS solution status bitflags, carried in DID_SYS_PARAMS.insStatus and copied into
 * every INS output message (DID_INS_1, DID_INS_2, DID_INS_3, DID_INS_4).
 *
 * Bits [0:2] and [4:6] report alignment quality (COARSE = usable but outside spec, FINE = within
 * spec) for heading, velocity, and position independently. Bits [16:19] (INS_STATUS_SOLUTION_MASK)
 * report the overall solution mode via INS_STATUS_SOLUTION(). Bits [24:25] report the GNSS
 * navigation fix type via INS_STATUS_NAV_FIX_STATUS() (see @ref eGnssNavFixStatus). The
 * remaining bits report aiding sources in use and fault/error conditions; @ref INS_STATUS_ERROR_MASK
 * aggregates the bits that indicate a solution problem requiring attention.
 */
enum eInsStatusFlags
{
    INS_STATUS_HDG_ALIGN_COARSE                         = (int)0x00000001,  //!< Heading estimate is usable but outside spec (COARSE)
    INS_STATUS_VEL_ALIGN_COARSE                         = (int)0x00000002,  //!< Velocity estimate is usable but outside spec (COARSE)
    INS_STATUS_POS_ALIGN_COARSE                         = (int)0x00000004,  //!< Position estimate is usable but outside spec (COARSE)
    INS_STATUS_ALIGN_COARSE_MASK                        = (int)0x00000007,  //!< Mask of all COARSE (usable but outside spec) alignment bits

    INS_STATUS_WHEEL_AIDING_VEL                         = (int)0x00000008,  //!< Velocity aided by wheel sensor

    /** Heading estimate is within spec (FINE). `INS_STATUS_HDG_ALIGN_COARSE` and `INS_STATUS_HDG_ALIGN_FINE` flags indicate whether INS heading is aided by any heading sensor (including GNSS or magnetometer). More accurate heading sensors (i.e. GNSS) are prioritized over less accurate sensors (i.e. magnetometers) and will fall back to the less accurate sensors when the more accurate sensors are not available. A momentary blip in these alignment flags may occur during heading transition from higher to lower accuracy aiding sensors (i.e. GNSS to magnetometer). `INS_STATUS_HDG_ALIGN_FINE` and `INS_STATUS_HDG_ALIGN_COARSE` flags will not be set when no heading aiding is available. */
    INS_STATUS_HDG_ALIGN_FINE                           = (int)0x00000010,  //!< Heading estimate is within spec (FINE); see detailed note above
    INS_STATUS_VEL_ALIGN_FINE                           = (int)0x00000020,  //!< Velocity estimate is within spec (FINE)
    INS_STATUS_POS_ALIGN_FINE                           = (int)0x00000040,  //!< Position estimate is within spec (FINE)
    INS_STATUS_ALIGN_FINE_MASK                          = (int)0x00000070,  //!< Mask of all FINE (within spec) alignment bits

    INS_STATUS_GNSS_AIDING_HEADING                      = (int)0x00000080,  //!< Heading aided by GNSS

    INS_STATUS_GNSS_AIDING_POS                          = (int)0x00000100,  //!< Position aided by GNSS position
    INS_STATUS_GNSS_UPDATE_IN_SOLUTION                  = (int)0x00000200,  //!< GNSS update event occurred in solution, potentially causing discontinuity in position path
    INS_STATUS_EKF_USING_REFERENCE_IMU                  = (int)0x00000400,  //!< Reference IMU used in EKF
    INS_STATUS_MAG_AIDING_HEADING                       = (int)0x00000800,  //!< Heading aided by magnetic heading

    INS_STATUS_NAV_MODE                                 = (int)0x00001000,  //!< Nav mode (set) = estimating velocity and position. AHRS mode (cleared) = NOT estimating velocity and position

    /** In dead reckoning mode.  The GNSS is not aiding the solution while the position is being estimated.  */
#define INS_STATUS_DEAD_RECKONING(insStatus)    (((insStatus)&(INS_STATUS_POS_ALIGN_FINE|INS_STATUS_POS_ALIGN_COARSE)) && (((insStatus)&INS_STATUS_GNSS_AIDING_POS)==0))

    INS_STATUS_STATIONARY_MODE                          = (int)0x00002000,  //!< INS in stationary mode. If initiated by zero velocity command, user should not move (keep system motionless) to assist on-board processing.
    INS_STATUS_GNSS_AIDING_VEL                          = (int)0x00004000,  //!< Velocity aided by GNSS velocity
    INS_STATUS_KINEMATIC_CAL_GOOD                       = (int)0x00008000,  //!< Vehicle kinematic calibration is good

    INS_STATUS_SOLUTION_MASK                            = (int)0x000F0000,  //!< INS/AHRS solution status mask; extract with INS_STATUS_SOLUTION()
    INS_STATUS_SOLUTION_OFFSET                          = 16,               //!< Bit offset of the solution status field within insStatus
#define INS_STATUS_SOLUTION(insStatus)          (((insStatus)&INS_STATUS_SOLUTION_MASK)>>INS_STATUS_SOLUTION_OFFSET)

    INS_STATUS_SOLUTION_OFF                             = 0,  //!< System is off
    INS_STATUS_SOLUTION_ALIGNING                        = 1,  //!< System is in alignment mode
    INS_STATUS_SOLUTION_NAV                             = 3,  //!< System is in navigation mode and solution is good.
    INS_STATUS_SOLUTION_NAV_HIGH_VARIANCE               = 4,  //!< System is in navigation mode but the attitude uncertainty has exceeded the threshold.
    INS_STATUS_SOLUTION_AHRS                            = 5,  //!< System is in AHRS mode and solution is good.
    INS_STATUS_SOLUTION_AHRS_HIGH_VARIANCE              = 6,  //!< System is in AHRS mode but the attitude uncertainty has exceeded the threshold.
    INS_STATUS_SOLUTION_VRS                             = 7,  //!< System is in VRS mode (no earth relative heading) and roll and pitch are good.
    INS_STATUS_SOLUTION_VRS_HIGH_VARIANCE               = 8,  //!< System is in VRS mode (no earth relative heading) but roll and pitch uncertainty has exceeded the threshold.

    INS_STATUS_RTK_COMPASSING_BASELINE_UNSET            = (int)0x00100000,  //!< GNSS compassing antenna offsets are not set in flashCfg.
    INS_STATUS_RTK_COMPASSING_BASELINE_BAD              = (int)0x00200000,  //!< GNSS antenna baseline specified in flashCfg and measured by GNSS do not match.
    INS_STATUS_RTK_COMPASSING_MASK                      = (INS_STATUS_RTK_COMPASSING_BASELINE_UNSET|INS_STATUS_RTK_COMPASSING_BASELINE_BAD),  //!< Mask of RTK compassing baseline error bits

    INS_STATUS_MAG_RECALIBRATING                        = (int)0x00400000,  //!< Magnetometer is being recalibrated. Device requires rotation to complete the calibration process. HDW_STATUS_MAG_RECAL_COMPLETE is set when complete.
    INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL_OR_NO_CAL    = (int)0x00800000,  //!< Magnetometer is experiencing interference or calibration is bad. Attention may be required to remove interference (move the device) or recalibrate the magnetometer.

    INS_STATUS_GNSS_NAV_FIX_MASK                        = (int)0x03000000,  //!< GNSS navigation fix type mask (see eGnssNavFixStatus); extract with INS_STATUS_NAV_FIX_STATUS()
    INS_STATUS_GNSS_NAV_FIX_OFFSET                      = 24,               //!< Bit offset of the GNSS navigation fix type field within insStatus
#define INS_STATUS_NAV_FIX_STATUS(insStatus)    (((insStatus)&INS_STATUS_GNSS_NAV_FIX_MASK)>>INS_STATUS_GNSS_NAV_FIX_OFFSET)

    INS_STATUS_RTK_COMPASSING_VALID                     = (int)0x04000000,  //!< RTK compassing heading is accurate and aiding INS heading. (RTK fix and hold status)

    /* NOTE: If you add or modify these INS_STATUS_RTK_ values, please update eInsStatusRtkBase in IS-src/python/src/ci_hdw/data_sets.py */
    INS_STATUS_RTK_RAW_GNSS_DATA_ERROR                  = (int)0x08000000,  //!< RTK error: Observations invalid or not received (i.e. RTK differential corrections)
    INS_STATUS_RTK_ERR_BASE_DATA_MISSING                = (int)0x10000000,  //!< RTK error: Either base observations or antenna position have not been received
    INS_STATUS_RTK_ERR_BASE_POSITION_MOVING             = (int)0x20000000,  //!< RTK error: base position moved when it should be stationary
    INS_STATUS_RTK_ERR_BASE_POSITION_INVALID            = (int)0x30000000,  //!< RTK error: base position invalid or not surveyed
    INS_STATUS_RTK_ERR_BASE_MASK                        = (int)0x30000000,  //!< RTK error: NO base position received (mask of base position error codes)
    INS_STATUS_RTK_ERROR_MASK                           = (INS_STATUS_RTK_RAW_GNSS_DATA_ERROR|INS_STATUS_RTK_ERR_BASE_MASK),  //!< GNSS base mask: combined RTK observation/base error bits

    INS_STATUS_RTOS_TASK_PERIOD_OVERRUN                 = (int)0x40000000,  //!< RTOS task ran longer than allotted period
    INS_STATUS_GENERAL_FAULT                            = (int)0x80000000,  //!< General fault (see sys_params_t.genFaultCode)

    INS_STATUS_ERROR_MASK                               = INS_STATUS_GENERAL_FAULT |  //!< Bitmask of all insStatus bits indicating an error condition
                                                    INS_STATUS_RTK_COMPASSING_MASK |
                                                    INS_STATUS_MAG_INTERFERENCE_OR_BAD_CAL_OR_NO_CAL |
                                                    INS_STATUS_RTK_ERROR_MASK |
                                                    INS_STATUS_RTOS_TASK_PERIOD_OVERRUN,
};

/**
 * @brief GNSS navigation fix type, packed into @ref eInsStatusFlags::INS_STATUS_GNSS_NAV_FIX_MASK
 * and extracted via INS_STATUS_NAV_FIX_STATUS(). Higher values indicate a more accurate fix.
 */
/* NOTE: If you modify this enum, please also modify the eGnssNavFixStatus enum
 *       in IS-src/python/src/ci_hdw/data_sets.py */
enum eGnssNavFixStatus
{
    GNSS_NAV_FIX_NONE                   = (int)0x00000000,  //!< No GNSS fix
    GNSS_NAV_FIX_POSITIONING_3D         = (int)0x00000001,  //!< Standard 3D GNSS position fix (no RTK corrections)
    GNSS_NAV_FIX_POSITIONING_RTK_FLOAT  = (int)0x00000002,  //!< RTK fix with floating (unresolved) integer ambiguities
    GNSS_NAV_FIX_POSITIONING_RTK_FIX    = (int)0x00000003,  //!< RTK fix with resolved integer ambiguities. Includes fix & hold
};

/**
 * @brief Hardware status bitflags, carried in DID_SYS_PARAMS.hdwStatus and copied into every INS
 * output message (DID_INS_1, DID_INS_2, DID_INS_3, DID_INS_4). Reports motion state, sensor
 * saturation/fault conditions, GNSS timing/PPS health, communications errors, built-in self-test
 * (BIT) state, and the cause of the last system reset. @ref HDW_STATUS_ERROR_MASK aggregates the
 * bits that indicate a hardware fault requiring attention.
 */
enum eHdwStatusFlags
{
    HDW_STATUS_MOTION_GYR                   = (int)0x00000001,  //!< Gyro motion detected
    HDW_STATUS_MOTION_ACC                   = (int)0x00000002,  //!< Accelerometer motion detected
    HDW_STATUS_MOTION_MASK                  = (int)0x00000003,  //!< Unit is moving and NOT stationary
    HDW_STATUS_IMU_FAULT_REJECT_GYR         = (int)0x00000004,  //!< IMU gyro fault rejection. One of the redundant gyro sensors is divergent and being excluded.
    HDW_STATUS_IMU_FAULT_REJECT_ACC         = (int)0x00000008,  //!< IMU accelerometer fault rejection. One of the redundant accelerometer sensors is divergent and being excluded.
    HDW_STATUS_IMU_FAULT_REJECT_MASK        = (int)0x0000000C,  //!< IMU fault rejection mask. One of the redundant IMU sensors is divergent and being excluded.

    HDW_STATUS_GNSS_SATELLITE_RX_VALID      = (int)0x00000010,  //!< GNSS satellite signals are being received (antenna and cable are good). Unset indicates weak signal or no output from GNSS receiver.
    HDW_STATUS_STROBE_IN_EVENT              = (int)0x00000020,  //!< Event occurred on strobe input pin
    HDW_STATUS_GNSS_TIME_OF_WEEK_VALID      = (int)0x00000040,  //!< GPS time of week is valid and reported. Otherwise the timeOfWeek is local system time.
    HDW_STATUS_REFERENCE_IMU_RX             = (int)0x00000080,  //!< Reference IMU data being received

    HDW_STATUS_SATURATION_GYR               = (int)0x00000100,  //!< Sensor saturation on gyro
    HDW_STATUS_SATURATION_ACC               = (int)0x00000200,  //!< Sensor saturation on accelerometer
    HDW_STATUS_SATURATION_MAG               = (int)0x00000400,  //!< Sensor saturation on magnetometer
    HDW_STATUS_SATURATION_BARO              = (int)0x00000800,  //!< Sensor saturation on barometric pressure

    HDW_STATUS_IMU_SATURATION_MASK          = (int)(HDW_STATUS_SATURATION_GYR | HDW_STATUS_SATURATION_ACC),  //!< Sensor saturation mask (IMU only: gyro + accelerometer)
    HDW_STATUS_SATURATION_MASK              = (int)0x00000F00,  //!< Sensor saturation mask (all sensors: gyro, accelerometer, magnetometer, barometer)
    HDW_STATUS_SATURATION_OFFSET            = 8,                //!< Bit offset of the sensor saturation field within hdwStatus

    HDW_STATUS_SYSTEM_RESET_REQUIRED        = (int)0x00001000,  //!< System Reset is required for proper function
    HDW_STATUS_ERR_GNSS_PPS_NOISE           = (int)0x00002000,  //!< GNSS PPS timepulse signal has noise and occurred too frequently
    HDW_STATUS_MAG_RECAL_COMPLETE           = (int)0x00004000,  //!< Magnetometer recalibration has finished (when INS_STATUS_MAG_RECALIBRATING is unset).
    HDW_STATUS_FLASH_WRITE_PENDING          = (int)0x00008000,  //!< System flash write staging or occurring now. Processor will pause and not respond during a flash write, typically 150-250 ms.

    HDW_STATUS_ERR_COM_TX_LIMITED           = (int)0x00010000,  //!< Communications Tx buffer limited
    HDW_STATUS_ERR_COM_RX_OVERRUN           = (int)0x00020000,  //!< Communications Rx buffer overrun

    HDW_STATUS_ERR_NO_GNSS_PPS              = (int)0x00040000,  //!< GNSS PPS timepulse signal has not been received or is in error
    HDW_STATUS_GNSS_PPS_TIMESYNC            = (int)0x00080000,  //!< Time synchronized by GNSS PPS

    HDW_STATUS_COM_PARSE_ERR_COUNT_MASK     = (int)0x00F00000,  //!< Communications parse error count mask; extract with HDW_STATUS_COM_PARSE_ERROR_COUNT()
    HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET   = 20,               //!< Bit offset of the communications parse error count field within hdwStatus
#define HDW_STATUS_COM_PARSE_ERROR_COUNT(hdwStatus) ((hdwStatus & HDW_STATUS_COM_PARSE_ERR_COUNT_MASK) >> HDW_STATUS_COM_PARSE_ERR_COUNT_OFFSET)

    HDW_STATUS_BIT_RUNNING                  = (int)0x01000000,  //!< (BIT) Built-in self-test running
    HDW_STATUS_BIT_PASSED                   = (int)0x02000000,  //!< (BIT) Built-in self-test passed
    HDW_STATUS_BIT_FAILED                   = (int)0x03000000,  //!< (BIT) Built-in self-test failure
    HDW_STATUS_BIT_MASK                     = (int)0x03000000,  //!< (BIT) Built-in self-test mask

    HDW_STATUS_ERR_TEMPERATURE              = (int)0x04000000,  //!< Temperature outside spec'd operating range

    HDW_STATUS_SPI_INTERFACE_ENABLED        = (int)0x08000000,  //!< IMX pins G5-G8 are configure for SPI use

    HDW_STATUS_RESET_CAUSE_MASK             = (int)0x70000000,  //!< Cause of system reset mask
    HDW_STATUS_RESET_CAUSE_BACKUP_MODE      = (int)0x10000000,  //!< Reset from backup mode (low-power state w/ CPU off)
    HDW_STATUS_RESET_CAUSE_WATCHDOG_FAULT   = (int)0x20000000,  //!< Reset from watchdog fault
    HDW_STATUS_RESET_CAUSE_SOFT             = (int)0x30000000,  //!< Reset from software
    HDW_STATUS_RESET_CAUSE_HDW              = (int)0x40000000,  //!< Reset from hardware (NRST pin low)

    HDW_STATUS_FAULT_SYS_CRITICAL           = (int)0x80000000,  //!< Critical System Fault, CPU error. (see DID_SYS_FAULT.status, eSysFaultStatus)

    HDW_STATUS_ERROR_MASK                   = HDW_STATUS_FAULT_SYS_CRITICAL |  //!< Bitmask of all hdwStatus bits indicating a hardware error
                                                    HDW_STATUS_IMU_FAULT_REJECT_MASK |
                                                    HDW_STATUS_SATURATION_MASK |
                                                    HDW_STATUS_ERR_GNSS_PPS_NOISE |
                                                    HDW_STATUS_ERR_COM_TX_LIMITED |
                                                    HDW_STATUS_ERR_COM_RX_OVERRUN |
                                                    HDW_STATUS_ERR_NO_GNSS_PPS |
                                                    HDW_STATUS_BIT_FAILED |
                                                    HDW_STATUS_ERR_TEMPERATURE,
};

/**
 * @brief System status bitflags, carried in sys_params_t.sysStatus (DID_SYS_PARAMS).
 * Reports miscellaneous board-level configuration state that doesn't fit eHdwStatusFlags.
 */
enum eSysStatusFlags
{
    SYS_STATUS_TBED3_LEDS_ENABLED                   = (int)0x00000001,  //!< Allow IMX to drive Testbed-3 status LEDs

    SYS_STATUS_PRIMARY_GNSS_SOURCE_IS_GNSS2         = (int)0x00000004,  //!< 0 = GNSS1 is the primary NMEA GNSS source, 1 = GNSS2 is the primary NMEA GNSS source
    SYS_STATUS_PRIMARY_GNSS_SOURCE_IS_GNSS2_offest  = 2,                //!< Bit offset of SYS_STATUS_PRIMARY_GNSS_SOURCE_IS_GNSS2 within sysStatus
};

// Used to validate GNSS position (and velocity)
#define GNSS_THRESH_SATS_USED   5
#define GNSS_THRESH_P_DOP       3.0f
#define GNSS_THRESH_H_ACC       10.0f
#define GNSS_THRESH_V_ACC       20.0f
#define GNSS_THRESH_S_ACC       2.0f

/**
 * @brief GNSS receiver status bitflags, carried in gnss_pos_t.status (DID_GNSS1_POS, DID_GNSS2_POS,
 * DID_GNSS1_RCVR_POS, DID_GNSS1_RTK_POS, etc). Packs [0x000000xx] a legacy satellite-used count,
 * [0x0000xx00] the fix type, and [0x xxxx0000] status/quality flags into a single 32-bit word.
 */
enum eGnssStatus
{
    // TODO: THIS FIELD WILL END OF LIFE IN PROTOCOL 3
    // PLEASE USE gnss_pos_t.satsUsed for all new development
    GNSS_STATUS_NUM_SATS_USED_MASK                      = (int)0x000000FF,  //!< (deprecated) Number of satellites used in solution; use gnss_pos_t.satsUsed instead

    GNSS_STATUS_FIX_NONE                                = (int)0x00000000,  //!< No fix
    GNSS_STATUS_FIX_DEAD_RECKONING_ONLY                 = (int)0x00000100,  //!< Dead reckoning only, no GNSS fix
    GNSS_STATUS_FIX_2D                                  = (int)0x00000200,  //!< 2D fix (no reliable altitude)
    GNSS_STATUS_FIX_3D                                  = (int)0x00000300,  //!< 3D fix
    GNSS_STATUS_FIX_GNSS_PLUS_DEAD_RECK                 = (int)0x00000400,  //!< GNSS fix combined with dead reckoning
    GNSS_STATUS_FIX_TIME_ONLY                           = (int)0x00000500,  //!< Time-only fix (no position)
    GNSS_STATUS_FIX_REF_LLA                             = (int)0x00000600,  //!< Fixed at a reference LLA position (e.g. surveyed base station)
    GNSS_STATUS_FIX_UNUSED2                             = (int)0x00000700,  //!< Unused
    GNSS_STATUS_FIX_DGPS                                = (int)0x00000800,  //!< Differential GPS (DGPS) fix
    GNSS_STATUS_FIX_SBAS                                = (int)0x00000900,  //!< SBAS (satellite-based augmentation) fix
    GNSS_STATUS_FIX_RTK_SINGLE                          = (int)0x00000A00,  //!< RTK single (no differential corrections applied)
    GNSS_STATUS_FIX_RTK_FLOAT                           = (int)0x00000B00,  //!< RTK fix with floating (unresolved) integer ambiguities
    GNSS_STATUS_FIX_RTK_FIX                             = (int)0x00000C00,  //!< RTK fix with resolved integer ambiguities
    GNSS_STATUS_FIX_MASK                                = (int)0x00001F00,  //!< Mask isolating the fix-type field
    GNSS_STATUS_FIX_BIT_OFFSET                          = (int)8,           //!< Bit offset of the fix-type field within status

    GNSS_STATUS_FLAGS_FIX_OK                            = (int)0x00010000,  //!< Fix is within limits (e.g. DOP & accuracy)
    GNSS_STATUS_FLAGS_DGPS_USED                         = (int)0x00020000,  //!< Differential GPS (DGPS) used.
    GNSS_STATUS_FLAGS_RTK_FIX_AND_HOLD                  = (int)0x00040000,  //!< RTK feedback on the integer solutions to drive the float biases towards the resolved integers
    GNSS_STATUS_FLAGS_UNUSED_1                          = (int)0x00080000,  //!< Unused
    GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_ENABLED        = (int)0x00100000,  //!< GNSS1 RTK precision positioning mode enabled
    GNSS_STATUS_FLAGS_STATIC_MODE                       = (int)0x00200000,  //!< Static mode
    GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_ENABLED         = (int)0x00400000,  //!< GNSS2 RTK moving base mode enabled
    GNSS_STATUS_FLAGS_GNSS1_RTK_RAW_GNSS_DATA_ERROR     = (int)0x00800000,  //!< GNSS1 RTK error: observations or ephemeris are invalid or not received (i.e. RTK differential corrections)
    GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_DATA_MISSING       = (int)0x01000000,  //!< GNSS1 RTK error: Either base observations or antenna position have not been received.
    GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_MOVING    = (int)0x02000000,  //!< GNSS1 RTK error: base position moved when it should be stationary
    GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_INVALID   = (int)0x03000000,  //!< GNSS1 RTK error: base position is invalid or not surveyed well
    GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_MASK      = (int)0x03000000,  //!< GNSS1 RTK error: base position error bitmask
    GNSS_STATUS_FLAGS_ERROR_MASK                        = (GNSS_STATUS_FLAGS_GNSS1_RTK_RAW_GNSS_DATA_ERROR |
                                                       GNSS_STATUS_FLAGS_GNSS1_RTK_BASE_POSITION_MASK),  //!< Mask of GNSS1 RTK error bits
    GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_VALID          = (int)0x04000000,  //!< GNSS1 RTK precision position and carrier phase range solution with fixed ambiguities (i.e. < 6cm horizontal accuracy). The carrier phase range solution with floating ambiguities occurs if GNSS_STATUS_FIX_RTK_FIX is set and GNSS_STATUS_FLAGS_GNSS1_RTK_POSITION_VALID is not set (i.e. > 6cm horizontal accuracy).
    GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_VALID           = (int)0x08000000,  //!< GNSS2 RTK moving base heading. Indicates RTK compassing heading valid and available in DID_GNSS2_RTK_CMP_REL.
    GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_BAD    = (int)0x00002000,  //!< GNSS2 RTK compassing antenna baseline specified in flashCfg and measured by GNSS do not match
    GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_UNSET  = (int)0x00004000,  //!< GNSS2 RTK compassing antenna baseline offsets are not set in flashCfg
    GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_MASK            = (GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_ENABLED|
                                                       GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_VALID|
                                                       GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_BAD|
                                                       GNSS_STATUS_FLAGS_GNSS2_RTK_COMPASS_BASELINE_UNSET),  //!< Mask of all GNSS2 RTK compassing bits
    GNSS_STATUS_FLAGS_GNSS_NMEA_DATA                    = (int)0x00008000,  //!< 1 = Data from NMEA message. GNSS velocity is NED (not ECEF).
    GNSS_STATUS_FLAGS_GNSS_PPS_TIMESYNC                 = (int)0x10000000,  //!< Time is synchronized by GNSS PPS.

    GNSS_STATUS_FLAGS_MASK                              = (int)0x1FFFE000,  //!< Mask isolating all status/quality flag bits
    GNSS_STATUS_FLAGS_BIT_OFFSET                        = (int)16,          //!< Bit offset of the status/quality flags field within status

    GNSS_STATUS_FLAGS_RTK_COV_ECEF_PACKED_VALID         = (int)0x20000000,  //!< RTK ECEF covariance matrix is valid and packed in rel->covEcefPacked
    GNSS_STATUS_FLAGS_UNUSED_3                          = (int)0x40000000,  //!< Unused
    GNSS_STATUS_FLAGS_UNUSED_4                          = (int)0x80000000,  //!< Unused
};

/**
 * @brief Secondary GNSS status byte, carried in gnss_pos_t.status2 (DID_GNSS1_POS, DID_GNSS2_POS,
 * etc). Reports RF interference (jamming) and spoofing detection state reported by the GNSS receiver.
 */
enum eGnssStatus2
{
    GNSS_STATUS2_FLAGS_GNSS_POSSIBLE_JAM_DETECT     = (uint8_t) 0x01,  //!< Possible RF jamming detected on the GNSS antenna
    GNSS_STATUS2_FLAGS_GNSS_JAM_DETECTED            = (uint8_t) 0x02,  //!< RF jamming confirmed on the GNSS antenna
    GNSS_STATUS2_FLAGS_GNSS_POSSIBLE_SPOOF_DETECT   = (uint8_t) 0x04,  //!< Possible GNSS spoofing detected
    GNSS_STATUS2_FLAGS_GNSS_SPOOF_DETECTED          = (uint8_t) 0x08,  //!< GNSS spoofing confirmed

    GNSS_STATUS2_FLAGS_JAM_SPOOF_POSSIBLE_MASK      = (uint8_t) 0x05,  //!< Mask of "possible" jam/spoof detection bits
    GNSS_STATUS2_FLAGS_JAM_SPOOF_DETECTED_MASK      = (uint8_t) 0x0A,  //!< Mask of confirmed jam/spoof detection bits
    GNSS_STATUS2_FLAGS_JAM_SPOOF_MASK               = (uint8_t) 0x0F,  //!< Mask of all jam/spoof detection bits (possible + confirmed)

    GNSS_STATUS2_FLAGS_UNUSED                       = 0xF0,  //!< Unused (reserved) bits
};

PUSH_PACK_1

/**
 * @brief (DID_POSITION_MEASUREMENT) External position measurement, sent to the device to aid the
 * INS/EKF solution (e.g. from an external vision system, motion capture rig, or other independent
 * positioning source). accuracyCovUD holds the upper-triangular part of the 3x3 symmetric position
 * accuracy covariance matrix associated with ecef, stored in row-major upper-diagonal order.
 */
typedef struct PACKED
{
    double      timeOfWeek;         //!< GPS time of week (since Sunday morning), s
    double      ecef[3];            //!< Position in ECEF (earth-centered earth-fixed) frame, m
    float       psi;                //!< Heading with respect to NED frame, rad
    float       accuracyCovUD[6];   //!< Upper Diagonal of the 3x3 position accuracy covariance matrix (indices: [0 1 2 / _ 3 4 / _ _ 5])
} pos_measurement_t;

/***
 * Product Hardware ID Mask  [6:4:6]
 * Product hardware ID is masked into 16 bits:
 *  [ 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 ]
 *    |- TYPE  -| |MAJOR| |- MINOR -|
 *
 *  Upper 6 bits are the hardware type (IMX, GPX, uINS, etc; 64 possible values)
 *  Middle 4 bits are the major hardware version (GPX-1, uINS-3, IMX-5, etc; 16 possible values)
 *  Lower 6 bits are the minor hardware version (IMX-6, uINS-3.2, GPX-1.0; 64 possible values)
 *
 *  If the TYPE and MAJOR are 0, then fall back to eIsHardwareType to determine the type from the legacy map:
 *      0 = Unknown
 *      1 = UINS32
 *      2 = EVB2
 *      3 = IMX5
 *      4 = GPX1
 */

#define HDW_TYPE__MASK      0xFC00                                          // 6 bits, bits 10-15
#define HDW_TYPE__SHIFT     10
#define DECODE_HDW_TYPE(x)  (((x) & HDW_TYPE__MASK) >> HDW_TYPE__SHIFT)
// Use eIsHardwareType for hardware type
#define HDW_MAJOR__MASK     0x03C0                                          // 4 bits, bits 6-9 (max value of 16)
#define HDW_MAJOR__SHIFT    6
#define DECODE_HDW_MAJOR(x) (((x) & HDW_MAJOR__MASK) >> HDW_MAJOR__SHIFT)

#define HDW_MINOR__MASK     0x003F                                          // 6 bits, bits 0-5 (max value of 63)
#define HDW_MINOR__SHIFT    0
#define DECODE_HDW_MINOR(x) (((x) & HDW_MINOR__MASK) >> HDW_MINOR__SHIFT)

#define ENCODE_HDW_ID(type,                     major, minor)      ((((uint8_t)(type) << HDW_TYPE__SHIFT) & HDW_TYPE__MASK) | (((uint8_t)(major) << HDW_MAJOR__SHIFT) & HDW_MAJOR__MASK) | (((uint8_t)(minor) << HDW_MINOR__SHIFT) & HDW_MINOR__MASK))
#define ENCODE_UNIQUE_ID(hdwId,                 serialNo)      (((uint64_t)hdwId << 48) | (uint64_t)serialNo)
#define ENCODE_DEV_INFO_TO_HDW_ID(devinfo)      (((devinfo.hardwareType << HDW_TYPE__SHIFT) & HDW_TYPE__MASK) | ((devinfo.hardwareVer[0] << HDW_MAJOR__SHIFT) & HDW_MAJOR__MASK) | ((devinfo.hardwareVer[1] << HDW_MINOR__SHIFT) & HDW_MINOR__MASK))
#define ENCODE_DEV_INFO_TO_UNIQUE_ID(devinfo)   (((uint64_t)(ENCODE_DEV_INFO_TO_HDW_ID(devinfo)) << 48) | (uint64_t)devinfo.serialNumber)
#define DECODE_UNIQUE_ID_TO_HDW_ID(devId)       ((uint16_t)((devId >> 48) & 0xFFFF))
#define DECODE_UNIQUE_ID_TO_SERIALNO(devId)     ((uint32_t)devId)
#define DEV_INFO_MATCHES_HDW_ID(di,             hdwId)     ( (ENCODE_DEV_INFO_TO_HDW_ID(di) & hdwId) == ENCODE_DEV_INFO_TO_HDW_ID(di) )

#define IS_HDW_TYPE_PERIPHERAL  0x20    // non-peripherals are 0-31, peripherals are 32-63
/**
 * @brief Hardware type identifier, occupying the TYPE field ([15:10]) of the packed hardware ID
 * (see the ID mask layout above) and stored in dev_info_t.hardwareType. Values below
 * IS_HDW_TYPE_PERIPHERAL (0x20) identify a top-level Inertial Sense product (IMX, GPX, uINS, EVB);
 * values at or above it identify a peripheral device (e.g. an onboard GNSS receiver chipset)
 * attached to that product.
 */
enum eIsHardwareType
{
    IS_HARDWARE_TYPE_MIXED      = -1,  //!< Mixed/wildcard hardware type. Used for ci-hdw testing
    IS_HARDWARE_TYPE_UNKNOWN    = 0,   //!< Unknown hardware type
    IS_HARDWARE_TYPE_UINS       = 1,   //!< uINS
    IS_HARDWARE_TYPE_EVB        = 2,   //!< EVB (evaluation board)
    IS_HARDWARE_TYPE_IMX        = 3,   //!< IMX
    IS_HARDWARE_TYPE_GPX        = 4,   //!< GPX
    IS_HDW_GNSS_UBLOX           = IS_HDW_TYPE_PERIPHERAL + 1,  //!< Ublox F9P GNSS receiver
    IS_HDW_GNSS_SONY            = IS_HDW_TYPE_PERIPHERAL + 2,  //!< Sony CXD5610 GNSS receiver
    IS_HDW_GNSS_SEPTENTRIO      = IS_HDW_TYPE_PERIPHERAL + 3,  //!< Septentrio GNSS receiver
    IS_HDW_GNSS_STM_TESSIO      = IS_HDW_TYPE_PERIPHERAL + 4,  //!< STM Tessio GNSS receiver

    IS_HARDWARE_TYPE_COUNT      = 5,  //!< Count of non-peripheral hardware types. Keep last non-peripheral
    IS_HDW_GNSS_TYPE_COUNT      = 4   //!< Number of entries in g_isGnssHardwareNames (IS_HDW_GNSS_UBLOX..STM_TESSIO)
};

typedef uint16_t is_hardware_t;
static const is_hardware_t IS_HARDWARE_NONE     = ENCODE_HDW_ID(IS_HARDWARE_TYPE_UNKNOWN, 0, 0);
static const is_hardware_t IS_HARDWARE_ANY      = ENCODE_HDW_ID(IS_HARDWARE_TYPE_MIXED, -1, -1);
static const is_hardware_t IS_HARDWARE_EVB_2_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_EVB, 2, 0);
static const is_hardware_t IS_HARDWARE_UINS_3_2 = ENCODE_HDW_ID(IS_HARDWARE_TYPE_UINS, 3, 2);
static const is_hardware_t IS_HARDWARE_IMX      = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, -1, -1);
static const is_hardware_t IS_HARDWARE_IMX_5_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 5, 0);
static const is_hardware_t IS_HARDWARE_IMX_6_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_IMX, 6, 0);
static const is_hardware_t IS_HARDWARE_GPX      = ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, -1, -1);
static const is_hardware_t IS_HARDWARE_GPX_1_0  = ENCODE_HDW_ID(IS_HARDWARE_TYPE_GPX, 1, 0);

static const is_hardware_t IS_HDW_UBLOX_F9P      = ENCODE_HDW_ID(IS_HDW_GNSS_UBLOX, 'F' - 'A', 9);
static const is_hardware_t IS_HDW_SONY_CXD5610   = ENCODE_HDW_ID(IS_HDW_GNSS_SONY, 5, 6);
static const is_hardware_t IS_HDW_TESSIO_6       = ENCODE_HDW_ID(IS_HDW_GNSS_STM_TESSIO, 6, 0);
static const is_hardware_t IS_HDW_SEPTENTRIO_G5  = ENCODE_HDW_ID(IS_HDW_GNSS_SEPTENTRIO, 'G' - 'A', 5);
static const is_hardware_t IS_HDW_SEPTENTRIO_P3  = ENCODE_HDW_ID(IS_HDW_GNSS_SEPTENTRIO, 'P' - 'A', 3);
static const is_hardware_t IS_HDW_SEPTENTRIO_M3  = ENCODE_HDW_ID(IS_HDW_GNSS_SEPTENTRIO, 'M' - 'A', 3);

extern const char* g_isHardwareTypeNames[IS_HARDWARE_TYPE_COUNT];
/// Names for the peripheral GNSS hardware types, indexed by (type - IS_HDW_TYPE_PERIPHERAL - 1).
extern const char* g_isGnssHardwareNames[IS_HDW_GNSS_TYPE_COUNT];

/** @brief Device run state, stored in dev_info_t.hdwRunState. Indicates whether the device is executing the bootloader or the main application firmware. */
enum eHdwRunStates {
    HDW_STATE_UNKNOWN,      //!< Run state not known/reported
    HDW_STATE_BOOTLOADER,   //!< Device is executing the bootloader
    HDW_STATE_APP,          //!< Device is executing the main application firmware
};

/** @brief Firmware build flags, stored in dev_info_t.buildFlags. */
enum eBuildFlags {
    BUILD_FLAGS_DEBUG   = 0x1,  //!< Firmware was built in debug mode (asserts/extra checks enabled, not optimized for production)
    BUILD_FLAGS_DIRTY   = 0x2,  //!< Firmware was built from a working tree with uncommitted local changes
};

/**
 * @brief (DID_DEV_INFO) Device information: identifies the connected device's hardware, firmware
 * build, and manufacturer. Reported once at connection and after firmware updates; use
 * @ref devInfoPopulateMissingHardware() to backfill hardwareType/hardwareVer on older firmware
 * that doesn't report them.
 */
typedef struct PACKED
{
    uint8_t         reserved;                                  //!< Reserved bits

    uint8_t         buildFlags;                                //!< Build flags: 0x1=debug mode, 0x2=dirty (see eBuildFlags)

    uint8_t         hardwareType;                              //!< Hardware Type: 1=uINS, 2=EVB, 3=IMX, 4=GPX (see eIsHardwareType)

    uint8_t         hdwRunState;                                //!< Device Run State: Bootloader, App, etc (see eHdwRunStates)

    uint32_t        serialNumber;                              //!< Serial number

    uint8_t         hardwareVer[4];                            //!< Hardware version

    uint8_t         firmwareVer[4];                            //!< Firmware (software) version

    uint32_t        buildNumber;                               //!< Build number

    uint8_t         protocolVer[4];                            //!< Communications protocol version

    uint32_t        repoRevision;                              //!< Repository revision number

    char            manufacturer[DEVINFO_MANUFACTURER_STRLEN]; //!< Manufacturer name

    uint8_t         buildType;                                 //!< Build type (0=production, 'c'=release candidate, 'b'=beta, 'a'=alpha, 'd'=developer, 's'=snapshot, '^'=dirty)

    uint8_t         buildYear;                                 //!< Build date year - 2000
    uint8_t         buildMonth;                                //!< Build date month
    uint8_t         buildDay;                                  //!< Build date day

    uint8_t         buildHour;                                 //!< Build time hour
    uint8_t         buildMinute;                                //!< Build time minute
    uint8_t         buildSecond;                                //!< Build time second
    uint8_t         buildMillisecond;                           //!< Build time millisecond

    char            addInfo[DEVINFO_ADDINFO_STRLEN];           //!< Additional info

} dev_info_t;

/** Add missing hardware descriptor to dev_info_t. */
void devInfoPopulateMissingHardware(dev_info_t *devInfo);

/**
 * @brief Convert NMEA talker string to ID (eNmeaMsgId).
 *
 * @param msg NMEA talker string
 * @param msgSize Length of the talker string
 * @return int NMEA ID (eNmeaMsgId) on success or negative for failure. -1 for NMEA head not found, -2 for invalid length, -3 other error
 */
int getNmeaMsgId(const void* msg, int msgSize);

/**
 * @brief Convert NMEA ID (eNmeaMsgId) to talker string
 *
 * @param msgId NMEA ID (eNmeaMsgId)
 * @param buf Talker id string output
 * @param bufSize Max size of buffer talker string will be written to.  Must be 5 or larger.
 * @return int 0 on success, -1 on failure.
 */
int nmeaMsgIdToTalker(int msgId, void *buf, int bufSize);

/**
 * @brief Get RTCM ID from
 *
 * @param buff
 * @param pos
 * @param len
 * @return unsigned int
 */
unsigned int messageStatsGetbitu(const unsigned char *buff, int pos, int len);

#define RTCM3_MSG_ID(msg)       messageStatsGetbitu((const unsigned char*)msg, 24, 12)
#define RTCM3_MSG_LENGTH(msg)   messageStatsGetbitu((const unsigned char*)msg, 14, 10)

/**
 * @brief (DID_MANUFACTURING_INFO) Manufacturing info, INTERNAL USE ONLY. One-time-programmable
 * (OTP) identity and provenance data written during manufacturing/testing; not intended for
 * general application use.
 */
typedef struct PACKED
{
    uint32_t    serialNumber;  //!< Inertial Sense serial number

    uint16_t    hardwareId;    //!< Hardware ID: packed identifier, encoding the Hardware Type, hardwareVer Major, and hardwareVer Minor (see ENCODE_HDW_ID/DECODE_HDW_TYPE macros)

    uint16_t    lotNumber;     //!< Inertial Sense lot number

    char        date[16];      //!< Inertial Sense manufacturing date (YYYYMMDDHHMMSS)

    uint32_t    key;           //!< Key - write: unlock manufacturing info, read: number of times OTP has been set, 15 max

    int32_t     platformType;  //!< Platform / carrier board (ePlatformConfig::PLATFORM_CFG_TYPE_MASK). Only valid if greater than zero.

    int32_t     reserved;      //!< Reserved

    uint32_t    uid[4];        //!< Microcontroller unique identifier, 128 bits for SAM / 96 for STM32
} manufacturing_info_t;

/**
 * @brief (DID_INS_1) INS navigation solution: Euler attitude w/ respect to NED, and NED position
 * relative to a reference LLA. This is the smallest/lightest of the four INS output variants;
 * position is reported both as absolute WGS84 LLA and as a NED offset from a reference point,
 * which avoids the precision loss of differencing two large LLA values downstream. Body angular
 * attitude is Euler (roll/pitch/yaw), which is compact but subject to gimbal-lock singularities
 * near +/-90 deg pitch; use @ref ins_2_t / @ref ins_4_t (quaternion) if that matters.
 */
typedef struct PACKED
{
    uint32_t    week;           //!< GPS number of weeks since January 6th, 1980

    double      timeOfWeek;     //!< GPS time of week (since Sunday morning) in seconds

    uint32_t    insStatus;      //!< INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus

    uint32_t    hdwStatus;      //!< Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus

    float       theta[3];       //!< Euler angles: roll, pitch, yaw in radians with respect to NED

    float       uvw[3];         //!< Velocity U, V, W in meters/second, in body frame. Convert to NED velocity using "vectorBodyToReference(uvw, theta, vel_ned)".

    double      lla[3];         //!< WGS84 latitude, longitude, height above ellipsoid (degrees, degrees, meters)

    float       ned[3];         //!< North, east, down (meters) offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude
} ins_1_t;


/**
 * @brief (DID_INS_2) INS navigation solution: quaternion attitude w/ respect to NED, and absolute
 * WGS84 position with ellipsoid (not MSL) altitude. Quaternion attitude avoids the Euler
 * gimbal-lock singularity of @ref ins_1_t. Unlike @ref ins_1_t, position is reported only as
 * absolute LLA (no NED offset from a reference point).
 */
typedef struct PACKED
{
    uint32_t    week;           //!< GPS number of weeks since January 6th, 1980

    double      timeOfWeek;     //!< GPS time of week (since Sunday morning) in seconds

    uint32_t    insStatus;      //!< INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus

    uint32_t    hdwStatus;      //!< Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus

    float       qn2b[4];        //!< Quaternion body rotation with respect to NED: W, X, Y, Z

    float       uvw[3];         //!< Velocity U, V, W in meters/second, in body frame. Convert to NED velocity using "quatRot(vel_ned, qn2b, uvw)".

    double      lla[3];         //!< WGS84 latitude, longitude, height above ellipsoid in meters (not MSL)
} ins_2_t;


/**
 * @brief (DID_INS_3) INS navigation solution: quaternion attitude w/ respect to NED, absolute
 * WGS84 position (ellipsoid altitude), plus a separately reported mean-sea-level (MSL) altitude.
 * Identical layout to @ref ins_2_t with one additional field (msl) — useful when an MSL-referenced
 * altitude (e.g. for display or barometric cross-check) is needed without recomputing the
 * geoid separation from ellipsoid height.
 */
typedef struct PACKED
{
    uint32_t    week;           //!< GPS number of weeks since January 6th, 1980

    double      timeOfWeek;     //!< GPS time of week (since Sunday morning) in seconds

    uint32_t    insStatus;      //!< INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus

    uint32_t    hdwStatus;      //!< Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus

    float       qn2b[4];        //!< Quaternion body rotation with respect to NED: W, X, Y, Z

    float       uvw[3];         //!< Velocity U, V, W in meters/second, in body frame. Convert to NED velocity using "quatRot(vel_ned, qn2b, uvw)".

    double      lla[3];         //!< WGS84 latitude, longitude, height above ellipsoid in meters (not MSL)

    float       msl;            //!< Height above mean sea level (MSL) in meters
} ins_3_t;


/**
 * @brief (DID_INS_4) INS navigation solution: quaternion attitude w/ respect to ECEF, and absolute
 * position/velocity in the ECEF (earth-centered, earth-fixed) frame. This is the only INS output
 * variant referenced entirely to ECEF rather than NED/LLA — attitude, velocity, and position are
 * all expressed in the rotating earth-fixed Cartesian frame, which is useful for consumers that
 * need a globally-consistent Cartesian frame (e.g. across the antimeridian or poles) without
 * NED's local-tangent-plane singularities.
 */
typedef struct PACKED
{
    uint32_t    week;           //!< GPS number of weeks since January 6th, 1980

    double      timeOfWeek;     //!< GPS time of week (since Sunday morning) in seconds

    uint32_t    insStatus;      //!< INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus

    uint32_t    hdwStatus;      //!< Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus

    float       qe2b[4];        //!< Quaternion body rotation with respect to ECEF: W, X, Y, Z

    float       ve[3];          //!< Velocity in ECEF (earth-centered earth-fixed) frame in meters per second

    double      ecef[3];        //!< Position in ECEF (earth-centered earth-fixed) frame in meters
} ins_4_t;


/**
 * @brief Single Inertial Measurement Unit (IMU) sample: angular rate and acceleration, in the
 * sensor/body frame. Embedded as a member of @ref imu_t (single IMU) and @ref imus_t (array of
 * up to MAX_IMU_DEVICES IMUs) — not a top-level DID payload itself.
 */
typedef struct PACKED
{
    float       pqr[3];         //!< Gyroscope P, Q, R (angular rate about body X, Y, Z) in radians/second

    float       acc[3];         //!< Acceleration X, Y, Z in meters/second^2, in body frame
} imui_t;


/**
 * @brief (DID_IMU, DID_IMU_RAW, DID_REFERENCE_IMU) Single combined Inertial Measurement Unit (IMU)
 * sample, in body/sensor frame. DID_IMU is down-sampled from the raw IMU rate
 * (DID_FLASH_CONFIG.startupImuDtMs) to the navigation update rate (DID_FLASH_CONFIG.startupNavDtMs)
 * as an anti-aliasing filter; DID_IMU_RAW is averaged from DID_IMUS_RAW for output rates faster
 * than the nav update rate; DID_REFERENCE_IMU carries a reference/truth IMU input from a testbed
 * used for manufacturing calibration and testing.
 */
typedef struct PACKED
{
    double      time;           //!< Time since boot up in seconds. Convert to GPS time of week by adding gps.towOffset

    uint32_t    status;         //!< IMU status flags (eImuStatus)

    imui_t      I;               //!< Combined Inertial Measurement Unit (IMU) sample: angular rate and acceleration
} imu_t;


/**
 * @brief (DID_IMUS_UNCAL, DID_IMUS_RAW, DID_IMUS) Samples from up to MAX_IMU_DEVICES individual
 * IMUs, in body/sensor frame. DID_IMUS_UNCAL is uncalibrated raw multi-IMU data (internal use
 * only); DID_IMUS_RAW is the calibrated version of DID_IMUS_UNCAL; DID_IMUS is DID_IMUS_RAW
 * down-sampled from the IMU rate (DID_FLASH_CONFIG.startupImuDtMs) to the navigation update rate
 * (DID_FLASH_CONFIG.startupNavDtMs) as an anti-aliasing filter. DID_IMU/DID_PIMU (the combined,
 * oversampled single-IMU outputs) are recommended over these for general use since they contain
 * less noise; this per-device data is primarily used for Allan variance / calibration analysis.
 */
typedef struct PACKED
{
    double                  time;         //!< Time since boot up in seconds. Convert to GPS time of week by adding gps.towOffset

    uint32_t                status;       //!< IMUs status flags (eImusStatus)

    imui_t                  I[MAX_IMU_DEVICES];  //!< Per-device Inertial Measurement Unit (IMU) samples: angular rate and acceleration
} imus_t;

#define SIZEOF_IMUS_T(numDevices)       (sizeof(double) + sizeof(uint32_t) + (sizeof(imui_t) * (numDevices)))
#define IMUS_T_NUM_DEVICES(byteSize)    ((byteSize - sizeof(double) - sizeof(uint32_t)) / sizeof(imui_t))

/**
 * @brief (DID_MAGNETOMETER, DID_REFERENCE_MAGNETOMETER) Magnetometer sensor data, in body/sensor
 * frame. DID_MAGNETOMETER is the onboard magnetometer output used for heading aiding;
 * DID_REFERENCE_MAGNETOMETER carries a reference/truth magnetometer input used for manufacturing
 * calibration and testing.
 */
typedef struct PACKED
{
    double                  time;   //!< Time since boot up in seconds. Convert to GPS time of week by adding gps.towOffset

    float                   mag[3]; //!< Magnetometer X, Y, Z in microtesla (uT), in body frame
} magnetometer_t;


/** @brief (DID_BAROMETER) Barometric pressure sensor data, used as an aiding source for altitude/vertical velocity. */
typedef struct PACKED
{
    double                  time;      //!< Time since boot up in seconds. Convert to GPS time of week by adding gps.towOffset

    float                   bar;       //!< Barometric pressure in kilopascals (kPa)

    float                   mslBar;    //!< MSL altitude derived from barometric pressure sensor, in meters

    float                   barTemp;   //!< Temperature of barometric pressure sensor in Celsius

    float                   humidity;  //!< Relative humidity as a percent (%rH). Range is 0% - 100%
} barometer_t;


/**
 * @brief (DID_PIMU, DID_REFERENCE_PIMU) Preintegrated IMU (PIMU), a.k.a. delta theta / delta
 * velocity or the Coning and Sculling integral, in body/IMU frame. Rather than reporting
 * instantaneous angular rate and acceleration, this data is integrated from the raw IMU samples
 * at the IMU update rate (DID_FLASH_CONFIG.startupImuDtMs, default 1ms); the integration period
 * (dt) matches the INS navigation update period (DID_FLASH_CONFIG.startupNavDtMs). This acts as a
 * form of lossless compression: it preserves the high-rate IMU information (avoiding the
 * antialiasing/filter-delay cost of simply decimating) while allowing a lower output data rate,
 * which is most beneficial for systems with higher dynamics and lower-bandwidth communication
 * links. To recover instantaneous IMU-equivalent values, divide by dt (IMU = PIMU / dt). Minimum
 * data period is DID_FLASH_CONFIG.startupImuDtMs or 4ms, whichever is larger (250Hz max).
 * DID_REFERENCE_PIMU carries a reference/truth IMU input used for manufacturing calibration and
 * testing.
 */
typedef struct PACKED
{
    double                  time;      //!< Time since boot up in seconds. Convert to GPS time of week by adding gps.towOffset

    float                   dt;        //!< Integration period in seconds for delta theta and delta velocity. Configured using DID_FLASH_CONFIG.startupNavDtMs

    uint32_t                status;    //!< IMU status flags (eImuStatus)

    float                   theta[3];  //!< IMU delta theta: gyroscope {p,q,r} integral over dt, in radians, in sensor/body frame

    float                   vel[3];    //!< IMU delta velocity: accelerometer {x,y,z} integral over dt, in meters/second, in sensor/body frame
} pimu_t;


/**
 * @brief (DID_IMU_MAG) Combined IMU + magnetometer sample: @ref imu_t (DID_IMU) paired with a
 * @ref magnetometer_t (DID_MAGNETOMETER) sample, both in body/sensor frame. Only one of
 * DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously.
 */
typedef struct PACKED
{
    imu_t           imu;  //!< Combined IMU sample (angular rate + acceleration), in body/sensor frame

    magnetometer_t  mag;  //!< Magnetometer sample, in body/sensor frame
} imu_mag_t;


/**
 * @brief (DID_PIMU_MAG) Combined preintegrated-IMU + magnetometer sample: @ref pimu_t
 * (DID_PIMU) paired with a @ref magnetometer_t (DID_MAGNETOMETER) sample, both in body/sensor
 * frame. Only one of DID_IMU_MAG or DID_PIMU_MAG should be streamed simultaneously.
 */
typedef struct PACKED
{
    pimu_t          pimu; //!< Preintegrated IMU sample (delta theta + delta velocity), in body/sensor frame

    magnetometer_t  mag;  //!< Magnetometer sample, in body/sensor frame
} pimu_mag_t;

/** @brief Per-device status bitflags for @ref imus_t.status (DID_IMUS_UNCAL, DID_IMUS_RAW, DID_IMUS). */
enum eImusStatus
{
    IMUS_STATUS_GYR_X_OK        = (int)0x00000001,  //!< Gyro X sensor valid status
    IMUS_STATUS_GYR_Y_OK        = (int)0x00000002,  //!< Gyro Y sensor valid status
    IMUS_STATUS_GYR_Z_OK        = (int)0x00000004,  //!< Gyro Z sensor valid status
    IMUS_STATUS_ACC_X_OK        = (int)0x00000008,  //!< Accelerometer X sensor valid status
    IMUS_STATUS_ACC_Y_OK        = (int)0x00000010,  //!< Accelerometer Y sensor valid status
    IMUS_STATUS_ACC_Z_OK        = (int)0x00000020,  //!< Accelerometer Z sensor valid status
    IMUS_STATUS_IMU_OK_BITSIZE  = 6,                //!< Number of IMU OK bits
    IMUS_STATUS_IMU_OK_MASK     = IMUS_STATUS_GYR_X_OK | IMUS_STATUS_GYR_Y_OK | IMUS_STATUS_GYR_Z_OK | IMUS_STATUS_ACC_X_OK | IMUS_STATUS_ACC_Y_OK | IMUS_STATUS_ACC_Z_OK,  //!< Mask of all IMU-valid bits
    IMUS_STATUS_GYR_OK_MASK     = IMUS_STATUS_GYR_X_OK | IMUS_STATUS_GYR_Y_OK | IMUS_STATUS_GYR_Z_OK,  //!< Mask of all gyro-valid bits
    IMUS_STATUS_ACC_OK_MASK     = IMUS_STATUS_ACC_X_OK | IMUS_STATUS_ACC_Y_OK | IMUS_STATUS_ACC_Z_OK,  //!< Mask of all accelerometer-valid bits

    IMUS_STATUS_SATURATION_GYR  = (int)0x40000000,  //!< Gyro sensor saturation detected
    IMUS_STATUS_SATURATION_ACC  = (int)0x80000000,  //!< Accelerometer sensor saturation detected
    IMUS_STATUS_SATURATION_MASK = (int)0xC0000000,  //!< Mask of all sensor-saturation bits
};

/** @brief Combined-IMU status bitflags for @ref imu_t.status and @ref pimu_t.status (DID_IMU, DID_IMU_RAW, DID_PIMU, etc). */
enum eImuStatus
{
    /** IMU X gyro is valid */
    IMU_STATUS_GYR_X_OK                 = (int)0x00000001,
    /** IMU Y gyro is valid */
    IMU_STATUS_GYR_Y_OK                 = (int)0x00000002,
    /** IMU Z gyro is valid */
    IMU_STATUS_GYR_Z_OK                 = (int)0x00000004,
    /** IMU X accelerometer is valid */
    IMU_STATUS_ACC_X_OK                 = (int)0x00000008,
    /** IMU Y accelerometer is valid */
    IMU_STATUS_ACC_Y_OK                 = (int)0x00000010,
    /** IMU Z accelerometer is valid */
    IMU_STATUS_ACC_Z_OK                 = (int)0x00000020,
    /** Number of IMU OK bits */
    IMU_STATUS_IMU_OK_BITSIZE           = 6,
    /** IMU valid mask */
    IMU_STATUS_IMU_OK_MASK              = IMU_STATUS_GYR_X_OK | IMU_STATUS_GYR_Y_OK | IMU_STATUS_GYR_Z_OK | IMU_STATUS_ACC_X_OK | IMU_STATUS_ACC_Y_OK | IMU_STATUS_ACC_Z_OK,
    /** Gyro valid mask */
    IMU_STATUS_GYR_OK_MASK              = IMU_STATUS_GYR_X_OK | IMU_STATUS_GYR_Y_OK | IMU_STATUS_GYR_Z_OK,
    /** Accelerometer valid mask */
    IMU_STATUS_ACC_OK_MASK              = IMU_STATUS_ACC_X_OK | IMU_STATUS_ACC_Y_OK | IMU_STATUS_ACC_Z_OK,

    /** Sensor shock detected */
    IMU_STATUS_SHOCK_PRESENT            = (int)0x00000040,

    /** Magnetometer sample occurred */
    IMU_STATUS_MAG_UPDATE               = (int)0x00000100,
    /** Data was received at least once from Reference IMU */
    IMU_STATUS_REFERENCE_IMU_PRESENT    = (int)0x00000200,
    /** Reserved */
    // IMU_STATUS_RESERVED2                        = (int)0x00000400,

//     /** Sensor saturation happened within past 10 seconds */
//     IMU_STATUS_SATURATION_HISTORY               = (int)0x00000100,
//     /** Sample rate fault happened within past 10 seconds */
//     IMU_STATUS_SAMPLE_RATE_FAULT_HISTORY        = (int)0x00000200,

    /** IMU fault rejection is excluding one of the gyros from the combined IMU output */
    IMU_STATUS_GYR_FAULT_REJECT         = (int)0x01000000,
    /** IMU fault rejection is excluding one of the accelerometers from the combined IMU output */
    IMU_STATUS_ACC_FAULT_REJECT         = (int)0x02000000,

    /** Sensor saturation */
    IMU_STATUS_SATURATION_GYR           = (int)0x40000000,
    IMU_STATUS_SATURATION_ACC           = (int)0x80000000,
    IMU_STATUS_SATURATION_MASK          = (int)0xC0000000,
};

/**
 * @brief (DID_GNSS1_POS, DID_GNSS2_POS, DID_GNSS1_RTK_POS, DID_GNSS1_RCVR_POS) GNSS position
 * solution. DID_GNSS1_POS is the "best" GNSS1 position, sourced from either DID_GNSS1_RCVR_POS
 * (raw receiver position) or DID_GNSS1_RTK_POS (RTK-corrected position), whichever is more
 * accurate; DID_GNSS2_POS is the equivalent for the second GNSS receiver. Position is reported
 * redundantly in both ECEF and LLA (WGS84) representations.
 */
typedef struct PACKED
{
    uint32_t                week;           //!< GPS number of weeks since January 6th, 1980

    uint32_t                timeOfWeekMs;   //!< GPS time of week (since Sunday morning) in milliseconds

    uint32_t                status;         //!< GNSS status (see eGnssStatus): [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag

    double                  ecef[3];        //!< Position in ECEF {x,y,z}, in meters

    double                  lla[3];         //!< Position in WGS84 latitude, longitude, height above ellipsoid (not MSL), in degrees, degrees, meters

    float                   hMSL;           //!< Height above mean sea level (MSL), in meters

    float                   hAcc;           //!< Horizontal accuracy, in meters

    float                   vAcc;           //!< Vertical accuracy, in meters

    float                   pDop;           //!< Position dilution of precision (unitless)

    float                   cnoMean;        //!< Average of all non-zero satellite carrier to noise ratios (signal strengths), in dBHz

    double                  towOffset;      //!< Time sync offset between local time since boot up and GPS time of week, in seconds. Add this to IMU and sensor time to get GPS time of week in seconds.

    uint8_t                 leapS;          //!< GPS leap second (GPS-UTC) offset, in seconds. Receiver's best knowledge of the leap seconds offset from UTC to GPS time. Subtract from GPS time of week to get UTC time of week. (18 seconds as of December 31, 2016)

    uint8_t                 satsUsed;       //!< Number of satellites used in the position solution

    uint8_t                 cnoMeanSigma;   //!< Standard deviation of cnoMean over the past 5 seconds, in dBHz x10

    uint8_t                 status2;        //!< Secondary GNSS status byte (see eGnssStatus2): [0x0X] spoofing/jamming status, [0xX0] unused

} gnss_pos_t;


/**
 * @brief (DID_GNSS1_VEL, DID_GNSS2_VEL) GNSS velocity solution. Velocity is reported in ECEF
 * unless the data originated from an NMEA message (status bit GNSS_STATUS_FLAGS_GNSS_NMEA_DATA
 * set), in which case it is reported in the local tangent-plane (NED) frame with vertical velocity
 * omitted.
 */
typedef struct PACKED
{
    uint32_t                    timeOfWeekMs;   //!< GPS time of week (since Sunday morning) in milliseconds

    float                       vel[3];         //!< GNSS velocity, in meters/second. In ECEF {vx,vy,vz} if status bit GNSS_STATUS_FLAGS_GNSS_NMEA_DATA (0x00008000) is NOT set; in local tangent plane with no vertical velocity {vNorth, vEast, 0} if that bit IS set.

    float                       sAcc;           //!< Speed accuracy, in meters/second

    uint32_t                    status;         //!< GNSS status (see eGnssStatus): [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag
} gnss_vel_t;


/** @brief Per-satellite tracking information, one entry per space vehicle in @ref gnss_sat_t.sat (DID_GNSS1_SAT, DID_GNSS2_SAT). */
typedef struct PACKED
{
    uint8_t                 gnssId;     //!< GNSS constellation identifier (see eSatSvGnssId)

    uint8_t                 svId;       //!< Satellite identifier (PRN/slot number, meaning depends on gnssId)

    int8_t                  elev;       //!< Elevation, in degrees (range: +/-90)

    int16_t                 azim;       //!< Azimuth, in degrees (range: +/-180)

    uint8_t                 cno;        //!< Carrier to noise ratio (signal strength), in dBHz

    uint16_t                status;     //!< Satellite status bitflags (see eSatSvStatus)
} gnss_sat_sv_t;

/** @brief GNSS constellation identifier, carried in @ref gnss_sat_sv_t.gnssId and @ref gnss_sig_sv_t.gnssId. */
enum eSatSvGnssId
{
    SAT_SV_GNSS_ID_UNKNOWN  = 0,  //!< Unknown constellation
    SAT_SV_GNSS_ID_GNSS     = 0,  //!< Multi-constellation (combined/undifferentiated)
    SAT_SV_GNSS_ID_GPS      = 1,  //!< GPS (USA)
    SAT_SV_GNSS_ID_SBS      = 2,  //!< SBAS (multiple regional systems, see flash config for selection)
    SAT_SV_GNSS_ID_GAL      = 3,  //!< Galileo (European Union)
    SAT_SV_GNSS_ID_BEI      = 4,  //!< BeiDou (China)
    SAT_SV_GNSS_ID_QZS      = 5,  //!< QZSS (Japan)
    SAT_SV_GNSS_ID_GLO      = 6,  //!< GLONASS (Russia)
    SAT_SV_GNSS_ID_IRN      = 7,  //!< IRNSS / NavIC (India)
    SAT_SV_GNSS_ID_IME      = 8,  //!< IMES (Japan's Indoor Messaging System)
    SAT_SV_GNSS_ID_COUNT    = 9,  //!< Number of constellations
};

/** @brief Per-satellite status bitflags, carried in @ref gnss_sat_sv_t.status. */
enum eSatSvStatus
{
    SAT_SV_STATUS_SIGNAL_QUALITY_MASK       = 0x0007,  //!< Mask for best signal quality among this satellite's signals (see eSatSigQuality)
    SAT_SV_STATUS_USED_IN_SOLUTION          = 0x0008,  //!< Satellite used in the position/velocity solution
    SAT_SV_STATUS_USED_IN_SOLUTION_OFFSET   = 3,        //!< Bit offset of SAT_SV_STATUS_USED_IN_SOLUTION within status
    SAT_SV_STATUS_HEALTH_UNKNOWN            = 0x0000,  //!< Health: 0 = unknown
    SAT_SV_STATUS_HEALTH_GOOD               = 0x0010,  //!< Health: 1 = healthy
    SAT_SV_STATUS_HEALTH_BAD                = 0x0020,  //!< Health: 2 = unhealthy
    SAT_SV_STATUS_HEALTH_MASK               = 0x0030,  //!< Mask isolating the health field
    SAT_SV_STATUS_HEALTH_OFFSET             = 4,        //!< Bit offset of the health field within status

    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_MASK   = 0x0300,  //!< Mask isolating the RTK solution fix-status field (1=float, 2=fix)
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_OFFSET = 8,        //!< Bit offset of the RTK solution fix-status field within status
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_FLOAT  = 1,        //!< RTK solution: floating (unresolved) integer ambiguities
    SAT_SV_STATUS_RTK_SOL_FIX_STATUS_FIX    = 2,        //!< RTK solution: fixed (resolved) integer ambiguities

    SAT_SV_STATUS_RTK_EPH_RTCM_PULSE        = 0x1000,  //!< Set when ephemeris is received for this SV; cleared by the next message send
    SAT_SV_STATUS_RTK_EPH_RTK_LIB_PULSE     = 0x2000,  //!< Set when ephemeris is added to RTKLIB for this SV
    SAT_SV_STATUS_RTK_EPH_PULSE_MASK        = SAT_SV_STATUS_RTK_EPH_RTK_LIB_PULSE | SAT_SV_STATUS_RTK_EPH_RTCM_PULSE,  //!< Mask of both ephemeris-received pulse bits; cleared by the next message send

    SAT_SV_STATUS_RTK_EPH_SEND_PULSE        = 0x4000,  //!< Set when ephemeris is transmitted from the device

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

/** @brief (DID_GNSS1_SAT, DID_GNSS2_SAT) Per-satellite tracking information: satellite identifiers, carrier to noise ratio, elevation/azimuth angles, and status, for every satellite currently visible/tracked by the receiver. */
typedef struct PACKED
{
    uint32_t                    timeOfWeekMs;   //!< GPS time of week (since Sunday morning) in milliseconds
    uint32_t                    numSats;        //!< Number of satellites in the sky (valid entries in the sat[] list below)
    gnss_sat_sv_t               sat[MAX_NUM_SATELLITES];   //!< Per-satellite tracking information list
} gnss_sat_t;

/** @brief Signal identifier (frequency/code description) carried in @ref gnss_sig_sv_t.sigId. Values are per-constellation; the same numeric value has different meaning depending on the associated gnssId (see eSatSvGnssId). */
enum eSatSvSigId
{
    SAT_SV_SIG_ID_GPS_L1CA      = 0,  //!< GPS L1 C/A
    SAT_SV_SIG_ID_GPS_L2CL      = 3,  //!< GPS L2C(L)
    SAT_SV_SIG_ID_GPS_L2CM      = 4,  //!< GPS L2C(M)
    SAT_SV_SIG_ID_GPS_L5I       = 6,  //!< GPS L5 I
    SAT_SV_SIG_ID_GPS_L5Q       = 7,  //!< GPS L5 Q
    SAT_SV_SIG_ID_GPS_L5        = SAT_SV_SIG_ID_GPS_L5Q,  //!< GPS L5 (alias of L5Q)

    SAT_SV_SIG_ID_SBAS_L1CA     = 0,  //!< SBAS L1 C/A
    SAT_SV_SIG_ID_SBAS_L2       = 1,  //!< SBAS L2
    SAT_SV_SIG_ID_SBAS_L5       = 2,  //!< SBAS L5

    SAT_SV_SIG_ID_Galileo_E1C2  = 0,  //!< Galileo E1 C2
    SAT_SV_SIG_ID_Galileo_E1B2  = 1,  //!< Galileo E1 B2
    SAT_SV_SIG_ID_Galileo_E1BC  = SAT_SV_SIG_ID_Galileo_E1B2,  //!< Galileo E1 BC (alias of E1B2)
    SAT_SV_SIG_ID_Galileo_E5aI  = 3,  //!< Galileo E5a I
    SAT_SV_SIG_ID_Galileo_E5aQ  = 4,  //!< Galileo E5a Q
    SAT_SV_SIG_ID_Galileo_E5a   = SAT_SV_SIG_ID_Galileo_E5aQ,  //!< Galileo E5a (alias of E5aQ)
    SAT_SV_SIG_ID_Galileo_E5bI  = 5,  //!< Galileo E5b I
    SAT_SV_SIG_ID_Galileo_E5bQ  = 6,  //!< Galileo E5b Q
    SAT_SV_SIG_ID_Galileo_E5    = SAT_SV_SIG_ID_Galileo_E5bQ,  //!< Galileo E5 (alias of E5bQ)

    SAT_SV_SIG_ID_BeiDou_B1D1   = 0,  //!< BeiDou B1 D1
    SAT_SV_SIG_ID_BeiDou_B1D2   = 1,  //!< BeiDou B1 D2
    SAT_SV_SIG_ID_BeiDou_B2D1   = 2,  //!< BeiDou B2 D1
    SAT_SV_SIG_ID_BeiDou_B2D2   = 3,  //!< BeiDou B2 D2
    SAT_SV_SIG_ID_BeiDou_B2     = SAT_SV_SIG_ID_BeiDou_B2D1,  //!< BeiDou B2 (alias of B2D1)
    SAT_SV_SIG_ID_BeiDou_B1C    = 5,  //!< BeiDou B1C
    SAT_SV_SIG_ID_BeiDou_B2a    = 7,  //!< BeiDou B2a

    SAT_SV_SIG_ID_QZSS_L1CA     = 0,  //!< QZSS L1 C/A
    SAT_SV_SIG_ID_QZSS_L1S      = 1,  //!< QZSS L1S
    SAT_SV_SIG_ID_QZSS_L2CM     = 4,  //!< QZSS L2C(M)
    SAT_SV_SIG_ID_QZSS_L2CL     = 5,  //!< QZSS L2C(L)
    SAT_SV_SIG_ID_QZSS_L2       = SAT_SV_SIG_ID_QZSS_L2CL,  //!< QZSS L2 (alias of L2CL)
    SAT_SV_SIG_ID_QZSS_L5I      = 8,  //!< QZSS L5 I
    SAT_SV_SIG_ID_QZSS_L5Q      = 9,  //!< QZSS L5 Q
    SAT_SV_SIG_ID_QZSS_L5       = SAT_SV_SIG_ID_QZSS_L5Q,  //!< QZSS L5 (alias of L5Q)

    SAT_SV_SIG_ID_GLONASS_L1OF  = 0,  //!< GLONASS L1OF
    SAT_SV_SIG_ID_GLONASS_L2OF  = 2,  //!< GLONASS L2OF

    SAT_SV_SIG_ID_NAVIC_L5A     = 0,  //!< NavIC (IRNSS) L5A
};

/** @brief Per-signal tracking quality/lock-progression state, carried in @ref gnss_sig_sv_t.quality. Values increase as the receiver progresses from no signal through search/acquisition to full code+carrier lock. */
enum eSatSigQuality
{
    SAT_SIG_QUALITY_NO_SIGNAL                   = 0,  //!< No signal
    SAT_SIG_QUALITY_SEARCHING                   = 1,  //!< Searching for signal
    SAT_SIG_QUALITY_ACQUIRED                    = 2,  //!< Signal acquired
    SAT_SIG_QUALITY_DETECTED                    = 3,  //!< Signal detected but unusable
    SAT_SIG_QUALITY_CODE_LOCK_TIME_SYNC         = 4,  //!< Code locked and time synchronized
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_1    = 5,  //!< Code and carrier locked and time synchronized
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_2    = 6,  //!< Code and carrier locked and time synchronized
    SAT_SIG_QUALITY_CODE_CARRIER_TIME_SYNC_3    = 7,  //!< Code and carrier locked and time synchronized
};

/** @brief Per-signal status bitflags, carried in @ref gnss_sig_sv_t.status. */
enum eSatSigStatus
{
    SAT_SIG_STATUS_HEALTH_UNKNOWN           = 0x0000,  //!< Health: 0 = unknown
    SAT_SIG_STATUS_HEALTH_GOOD              = 0x0001,  //!< Health: 1 = healthy
    SAT_SIG_STATUS_HEALTH_BAD               = 0x0002,  //!< Health: 2 = unhealthy
    SAT_SIG_STATUS_HEALTH_MASK              = 0x0003,  //!< Mask isolating the health field
    SAT_SIG_STATUS_USED_IN_SOLUTION         = 0x0004,  //!< Signal is used in the position/velocity solution
    SAT_SIG_STATUS_USED_IN_SOLUTION_OFFSET  = 2,        //!< Bit offset of SAT_SIG_STATUS_USED_IN_SOLUTION within status
};


/** @brief Per-signal tracking information, one entry per tracked signal in @ref gnss_sig_t.sig (DID_GNSS1_SIG, DID_GNSS2_SIG). A single satellite (gnssId+svId) may contribute multiple entries, one per tracked frequency/code. */
typedef struct PACKED
{
    uint8_t                     gnssId;     //!< GNSS constellation identifier (see eSatSvGnssId)

    uint8_t                     svId;       //!< Satellite identifier (PRN/slot number, meaning depends on gnssId)

    uint8_t                     sigId;      //!< Signal identifier, frequency/code description (see eSatSvSigId)

    uint8_t                     cno;        //!< Carrier to noise ratio (signal strength), in dBHz

    uint8_t                     quality;    //!< Signal tracking quality/lock state (see eSatSigQuality)

    uint16_t                    status;     //!< Signal status bitflags (see eSatSigStatus)

} gnss_sig_sv_t;

/** @brief (DID_GNSS1_SIG, DID_GNSS2_SIG) Per-signal GNSS tracking information for every satellite signal currently tracked by the receiver, one entry per frequency/code per satellite. */
typedef struct PACKED
{
    uint32_t                    timeOfWeekMs;   //!< GPS time of week (since Sunday morning) in milliseconds
    uint32_t                    numSigs;        //!< Number of satellite signals in the following satellite signal list (valid entries in sig[] below)
    gnss_sig_sv_t               sig[MAX_NUM_SAT_SIGNALS];   //!< Per-signal tracking information list
} gnss_sig_t;

typedef uint8_t         gnss_extension_ver_t[30];   //!< 30-byte GNSS receiver version-info extension string
#define GNSS_VER_NUM_EXTENSIONS 6
/** @brief (DID_GNSS1_VERSION, DID_GNSS2_VERSION) GNSS receiver firmware/hardware version identification strings, as reported by the GNSS receiver module itself. */
typedef struct PACKED
{
    uint8_t                     swVersion[30];      //!< GNSS receiver software version string
    uint8_t                     hwVersion[10];       //!< GNSS receiver hardware version string
    gnss_extension_ver_t        extension[GNSS_VER_NUM_EXTENSIONS];    //!< Additional 30-byte extension version-info strings reported by the receiver
} gnss_version_t;

/** @brief (DID_INL2_STATES) INL2 - INS Extended Kalman Filter (EKF) navigation states: attitude, velocity, and position in the ECEF frame, plus estimated sensor biases and magnetic field model parameters. */
typedef struct PACKED
{
    double                  timeOfWeek;     //!< GPS time of week (since Sunday morning), in seconds

    float                   qe2b[4];        //!< Quaternion body rotation with respect to ECEF: W, X, Y, Z

    float                   ve[3];          //!< Velocity in ECEF frame, in meters/second

    double                  ecef[3];        //!< Position in ECEF frame, in meters

    float                   biasPqr[3];     //!< Gyro bias estimate, in radians/second

    float                   biasAcc[3];     //!< Accelerometer bias estimate, in meters/second^2

    float                   biasBaro;       //!< Barometer bias estimate (altitude), in meters

    float                   magDec;         //!< Magnetic declination estimate, in radians

    float                   magInc;         //!< Magnetic inclination estimate, in radians
} inl2_states_t;

/** @brief (DID_ROS_COVARIANCE_POSE_TWIST) INL2 EKF pose and twist error covariance matrices, packed for direct use in a ROS PoseWithCovariance/TwistWithCovariance message. */
typedef struct PACKED
{
    double                      timeOfWeek;     //!< GPS time of week (since Sunday morning), in seconds

    /** Packed 6x6 lower-diagonal covariance matrix (21 values, row-major) for EKF pose errors: Attitude (roll,pitch,yaw) error (body frame, rad^2), Position (x,y,z) error (ECEF frame, m^2).
     * Index layout:
     *    0 __ __ __ __ __
     *    1  2 __ __ __ __
     *    3  4  5 __ __ __
     *    6  7  8  9 __ __
     *   10 11 12 13 14 __
     *   15 16 17 18 19 20  */
    float                       covPoseLD[21];  //!< Lower-diagonal pose covariance (attitude rad^2, position m^2); see index layout above

    /** Packed 6x6 lower-diagonal covariance matrix (21 values, row-major) for EKF twist errors: Velocity (x,y,z) error (ECEF frame, (m/s)^2), Angular rate (p,q,r) error (body frame, (rad/s)^2).
     * Index layout:
     *   0 __ __ __ __ __
     *   1  2 __ __ __ __
     *   3  4  5 __ __ __
     *   6  7  8  9 __ __
     *  10 11 12 13 14 __
     *  15 16 17 18 19 20  */
    float                       covTwistLD[21]; //!< Lower-diagonal twist covariance (velocity (m/s)^2, angular rate (rad/s)^2); see index layout above
} ros_covariance_pose_twist_t;

/** @brief (DID_INL2_STATUS) INL2 EKF internal status/diagnostic flags: motion detection state, heading alignment progress, and mag calibration state used during INS initialization. */
typedef struct PACKED
{
    int             ahrs;           //!< Non-zero while the filter is running in AHRS-only mode (prior to full INS/GNSS navigation)
    int             zero_accel;     //!< Non-zero when zero-acceleration condition is detected
    int             zero_angrate;   //!< Non-zero when zero-angular-rate condition is detected
    int             accel_motion;   //!< Non-zero when accelerometer-sensed motion is detected
    int             rot_motion;     //!< Non-zero when rotational motion is detected
    int             zero_vel;       //!< Non-zero when zero-velocity condition is detected
    int             ahrs_gnss_cnt;  //!< Counter of sequential valid GNSS data (for switching from AHRS to navigation)
    float           hdg_err;        //!< Estimated heading error, in radians
    int             hdg_coarse;     //!< Flag whether a coarse (uncertain) initial heading has been established
    int             hdg_aligned;    //!< Flag whether initial attitude error has converged (heading alignment complete)
    int             hdg_aligning;   //!< Flag whether heading alignment is currently in progress
    int             ekf_init_done;  //!< Hot EKF initialization completed
    int             mag_cal_good;   //!< Flag whether the magnetometer calibration is good
    int             mag_cal_done;   //!< Flag whether the magnetometer calibration process has completed
    int             stat_magfield;  //!< Flag whether the magnetic field is stationary/consistent (suitable for mag calibration)
} inl2_status_t;

/** @brief External position sensor sample. */

typedef struct PACKED
{
    uint32_t   timeOfWeekMs; //!< GPS time of week (since Sunday morning) in milliseconds
    double     pos[3];       //!< position {x,y,z} (m)
    float      offset[3];    //!< point of measurement relative to IMU origin in IMU/body frame {x,y,z} (m)
    float      var[3];       //!< observation variance 
    //uint32_t   frame;        //!< frame of measurement: 0=ECEF, 1=LLA
} ext_pos_t;

/** @brief External velocity sensor sample. */

typedef struct PACKED
{
    uint32_t   timeOfWeekMs; //!< GPS time of week (since Sunday morning) in milliseconds
    float      vel[3];       //!< velocity {vx,vy,vz} (m/s)
    float      offset[3];    //!< point of measurement relative to IMU origin in IMU/body frame {x,y,z} (m)
    float      var[3];       //!< observation variance 
    uint32_t   frame;        //!< frame of measurement: 0=ECEF, 1=NED, 2=Body
} ext_vel_t;


/** @brief Generic single-axis scalar sensor sample: a timestamped single value, reused across multiple DIDs for simple scalar sensor outputs. */
typedef struct PACKED
{
    double                  time;   //!< Time in seconds (meaning is source-dependent; typically time since boot up or GPS time of week)
    float                   val;    //!< Sensor value (units are source-dependent)
} gen_1axis_sensor_t;

/** @brief Generic 3-axis single-precision sensor sample: a timestamped 3-element vector, reused across multiple DIDs for 3-axis sensor outputs (e.g. gyro, accelerometer, magnetometer). */
typedef struct PACKED
{
    double                  time;   //!< Time in seconds (meaning is source-dependent; typically time since boot up or GPS time of week)
    float                   val[3]; //!< 3-axis sensor value {x,y,z} (units are source-dependent)
    float                   pos[3]; //!< point of measurement relative to IMU origin in IMU/body frame {x,y,z} (m)
} gen_3axis_sensor_t;

/** @brief Generic dual 3-axis single-precision sensor sample: a timestamped pair of 3-element vectors, reused across multiple DIDs that report two related 3-axis sensors together (e.g. uncalibrated + calibrated, or two redundant sensors). */
typedef struct PACKED
{
    double                  time;   //!< Time in seconds (meaning is source-dependent; typically time since boot up or GPS time of week)
    float                   val1[3]; //!< First 3-axis sensor value {x,y,z} (units are source-dependent)
    float                   val2[3]; //!< Second 3-axis sensor value {x,y,z} (units are source-dependent)
} gen_dual_3axis_sensor_t;

/** @brief Generic 3-axis double-precision sensor sample: a timestamped 3-element vector, reused across multiple DIDs where extra precision beyond gen_3axis_sensor_t is needed. */
typedef struct PACKED
{
    double                  time;   //!< Time in seconds (meaning is source-dependent; typically time since boot up or GPS time of week)
    double                  val[3]; //!< 3-axis sensor value {x,y,z} (units are source-dependent)
    float                   pos[3]; //!< point of measurement relative to IMU origin in IMU/body frame {x,y,z} (m)
} gen_3axis_sensord_t;

/** @brief (DID_SYS_SENSORS) Raw/calibrated output from the system's onboard sensors: IMU (gyro/accelerometer), magnetometer, barometer, humidity, and analog/voltage monitor inputs. */
typedef struct PACKED
{
    double                  time;       //!< Time since boot up, in seconds. Convert to GPS time of week by adding gps.towOffset

    float                   temp;       //!< IMU temperature, in Celsius

    float                   pqr[3];     //!< Gyros {p,q,r}, in radians/second

    float                   acc[3];     //!< Accelerometers {x,y,z}, in meters/second^2

    float                   mag[3];     //!< Magnetometers {x,y,z} (uncalibrated units)

    float                   bar;        //!< Barometric pressure, in kilopascals

    float                   barTemp;    //!< Temperature of barometric pressure sensor, in Celsius

    float                   mslBar;     //!< MSL altitude derived from barometric pressure sensor, in meters

    float                   humidity;   //!< Relative humidity as a percent (%rH). Range is 0% - 100%

    float                   vin;        //!< EVB system input voltage, in volts. uINS pin 5 (G2/AN2). Use 10K/1K resistor divider between Vin and GND.

    float                   ana1;       //!< ADC analog input, in volts. uINS pin 4 (G1/AN1)

    float                   ana3;       //!< ADC analog input, in volts. uINS pin 19 (G3/AN3)

    float                   ana4;       //!< ADC analog input, in volts. uINS pin 20 (G4/AN4)
} sys_sensors_t;

/** @brief Minimal INS navigation output: position (LLA), body-frame velocity, and attitude quaternion with respect to NED. */
typedef struct PACKED
{
    uint32_t                    timeOfWeekMs;   //!< GPS time of week (since Sunday morning) in milliseconds

    double                      lla[3];         //!< Latitude, longitude, and height above ellipsoid, in radians, radians, meters

    float                       uvw[3];         //!< Velocity in body frame {X,Y,Z}, in meters/second

    float                       qn2b[4];        //!< Quaternion body rotation with respect to NED: W, X, Y, Z
} ins_output_t;

/** @brief (DID_SYS_PARAMS) System-level parameters and status: aggregated status flags (see @ref eInsStatusFlags, @ref eHdwStatusFlags, @ref eSysStatusFlags), temperatures, timing configuration, and general fault code. */
typedef struct PACKED
{
    uint32_t                timeOfWeekMs;       //!< GPS time of week (since Sunday morning) in milliseconds

    uint32_t                insStatus;          //!< INS status flags (see eInsStatusFlags)

    uint32_t                hdwStatus;          //!< Hardware status flags (see eHdwStatusFlags)

    float                   imuTemp;            //!< IMU temperature, in Celsius

    float                   baroTemp;           //!< Barometer temperature, in Celsius

    float                   mcuTemp;            //!< MCU temperature, in Celsius (not available yet)

    uint32_t                sysStatus;          //!< System status flags (see eSysStatusFlags)

    uint32_t                imuSamplePeriodMs;  //!< IMU sample period, in milliseconds. Zero disables sampling.

    uint32_t                navOutputPeriodMs;  //!< Preintegrated IMU (PIMU) integration period and navigation/AHRS filter output period, in milliseconds

    double                  sensorTruePeriod;   //!< Actual sample period relative to GNSS PPS, in seconds

    uint32_t                flashCfgChecksum;   //!< Flash config checksum used with host SDK synchronization

    uint32_t                navUpdatePeriodMs;  //!< Navigation/AHRS filter update period, in milliseconds

    uint32_t                genFaultCode;       //!< General fault code descriptor (see eGenFaultCodes). Set to zero to reset fault code.

    double                  upTime;             //!< System up time, in seconds (double precision)

} sys_params_t;

/** @brief General fault code bitflags, carried in @ref sys_params_t.genFaultCode (DID_SYS_PARAMS). Multiple bits may be set simultaneously; a value of zero indicates no fault is latched. */
enum eGenFaultCodes
{
    GFC_INS_STATE_ORUN_UVW      = 0x00000001,  //!< INS state limit overrun - UVW (body frame velocity)
    GFC_INS_STATE_ORUN_LAT      = 0x00000002,  //!< INS state limit overrun - Latitude
    GFC_INS_STATE_ORUN_ALT      = 0x00000004,  //!< INS state limit overrun - Altitude
    GFC_UNHANDLED_INTERRUPT     = 0x00000010,  //!< Unhandled interrupt
    GFC_GNSS_CRITICAL_FAULT     = 0x00000020,  //!< GNSS receiver critical fault. See the corresponding GNSS status fault flags (i.e. GPX_STATUS_FATAL_MASK)
    GFC_GNSS_TX_LIMITED         = 0x00000040,  //!< GNSS Tx limited
    GFC_GNSS_RX_OVERRUN         = 0x00000080,  //!< GNSS Rx overrun
    GFC_INIT_SENSORS            = 0x00000100,  //!< Fault: sensor initialization
    GFC_INIT_SPI                = 0x00000200,  //!< Fault: SPI bus initialization
    GFC_CONFIG_SPI              = 0x00000400,  //!< Fault: SPI configuration
    GFC_GNSS1_INIT              = 0x00000800,  //!< Fault: GNSS1 init
    GFC_GNSS2_INIT              = 0x00001000,  //!< Fault: GNSS2 init
    GFC_FLASH_INVALID_VALUES    = 0x00002000,  //!< Flash failed to load valid values
    GFC_FLASH_CHECKSUM_FAILURE  = 0x00004000,  //!< Flash checksum failure
    GFC_FLASH_WRITE_FAILURE     = 0x00008000,  //!< Flash write failure
    GFC_SYS_FAULT_GENERAL       = 0x00010000,  //!< System Fault: general
    GFC_SYS_FAULT_CRITICAL      = 0x00020000,  //!< System Fault: CRITICAL system fault (see DID_SYS_FAULT)
    GFC_SENSOR_SATURATION       = 0x00040000,  //!< Sensor(s) saturated
    GFC_EKF_STATES_INVALID      = 0x00080000,  //!< INS extended kalman filter states invalid and the EKF was reset
    GFC_INIT_IMU                = 0x00100000,  //!< Fault: IMU initialization
    GFC_INIT_BAROMETER          = 0x00200000,  //!< Fault: Barometer initialization
    GFC_INIT_MAGNETOMETER       = 0x00400000,  //!< Fault: Magnetometer initialization
    GFC_INIT_I2C                = 0x00800000,  //!< Fault: I2C initialization
    GFC_CHIP_ERASE_INVALID      = 0x01000000,  //!< Fault: Chip erase line toggled but did not meet required hold time.  This is caused by noise/transient on chip erase pin.
    GFC_EKF_GNSS_TIME_FAULT     = 0x02000000,  //!< Fault: EKF GNSS time fault
    GFC_GNSS_RECEIVER_TIME      = 0x04000000,  //!< Fault: GNSS receiver time fault
    GFC_GNSS_GENERAL_FAULT      = 0x08000000,  //!< Fault: GNSS reciever ceneral fault. See the corresponding GNSS status fault flags (i.e. GPX_STATUS_GENERAL_FAULT_MASK)
    GFC_EKF_INPUT_INVALID_IMU   = 0x10000000,  //!< Fault: Invalid IMU input rejected by EKF
    GFC_GNSS_RTOS_ERROR         = 0x20000000,  //!< Fault: GNSS RTOS error

    GFC_GPX_STATUS_COMMON_MASK  = GFC_GNSS1_INIT | GFC_GNSS2_INIT | GFC_GNSS_TX_LIMITED | GFC_GNSS_RX_OVERRUN | GFC_GNSS_CRITICAL_FAULT | GFC_GNSS_RECEIVER_TIME | GFC_GNSS_GENERAL_FAULT,  //!< IMX GFC flags that relate to GPX status flags
};


/** @brief (DID_SYS_CMD) System command request. Both fields must be set together in the same write for the command to take effect. */
typedef struct PACKED
{
    uint32_t                command;    //!< System command (see eSystemCommand) 1=save current persistent messages, 5=zero motion, 97=save flash, 99=software reset.  "invCommand" (following variable) must be set to bitwise inverse of this value for this command to be processed.

    uint32_t                invCommand; //!< Error checking field that must be set to bitwise inverse of command field for the command to take effect.

} system_command_t;

/** @brief Command codes for @ref system_command_t.command (DID_SYS_CMD). The paired invCommand value (shown in parens as "inv") is the bitwise inverse of command and must be sent in the same write for the command to be processed. */
enum eSystemCommand
{
    SYS_CMD_NONE                                                    = 0,  //!< No command (uint32 inv: 4294967295)
    SYS_CMD_SAVE_PERSISTENT_MESSAGES                                = 1,  //!< Save current persistent messages (uint32 inv: 4294967294)
    SYS_CMD_ENABLE_BOOTLOADER_AND_RESET                             = 2,  //!< Enable bootloader and reset (uint32 inv: 4294967293)
    SYS_CMD_ENABLE_SENSOR_STATS                                     = 3,  //!< Enable sensor stats collection (uint32 inv: 4294967292)
    SYS_CMD_ENABLE_RTOS_STATS                                       = 4,  //!< Enable RTOS task stats collection (uint32 inv: 4294967291)
    SYS_CMD_ZERO_MOTION                                             = 5,  //!< Zero motion (stationary) command, used to aid the EKF (uint32 inv: 4294967290)
    SYS_CMD_REF_POINT_STATIONARY                                    = 6,  //!< Mark reference/truth input as stationary, for calibration (uint32 inv: 4294967289)
    SYS_CMD_REF_POINT_MOVING                                        = 7,  //!< Mark reference/truth input as moving, for calibration (uint32 inv: 4294967288)
    SYS_CMD_RESET_RTOS_STATS                                        = 8,  //!< Reset RTOS task stats (uint32 inv: 4294967287)

    SYS_CMD_ENABLE_GNSS_LOW_LEVEL_CONFIG                            = 10,  //!< Enable GNSS low level (chipset-specific) configuration (uint32 inv: 4294967285)
    SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE                              = 11,  //!< Disable serial port bridge (uint32 inv: 4294967284)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GNSS1                  = 12,  //!< Enable serial port bridge: USB to GNSS1 (uint32 inv: 4294967283)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GNSS2                  = 13,  //!< Enable serial port bridge: USB to GNSS2 (uint32 inv: 4294967282)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER0                   = 14,  //!< Enable serial port bridge: USB to SER0 (uint32 inv: 4294967281)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER1                   = 15,  //!< Enable serial port bridge: USB to SER1 (uint32 inv: 4294967280)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER2                   = 16,  //!< Enable serial port bridge: USB to SER2 (uint32 inv: 4294967279)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_TO_GNSS1                 = 17,  //!< Enable serial port bridge: SER0 to GNSS1 (uint32 inv: 4294967278)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GNSS1             = 18,  //!< Enable serial port bridge: current port to GNSS1 (uint32 inv: 4294967277)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GNSS2             = 19,  //!< Enable serial port bridge: current port to GNSS2 (uint32 inv: 4294967276)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_USB               = 20,  //!< Enable serial port bridge: current port to USB (uint32 inv: 4294967275)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER0              = 21,  //!< Enable serial port bridge: current port to SER0 (uint32 inv: 4294967274)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER1              = 22,  //!< Enable serial port bridge: current port to SER1 (uint32 inv: 4294967273)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER2              = 23,  //!< Enable serial port bridge: current port to SER2 (uint32 inv: 4294967272)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_LOOPBACK                  = 24,  //!< Enable serial port bridge: USB loopback (uint32 inv: 4294967271)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_LOOPBACK                 = 25,  //!< Enable serial port bridge: SER0 loopback (uint32 inv: 4294967270)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER1_LOOPBACK                 = 26,  //!< Enable serial port bridge: SER1 loopback (uint32 inv: 4294967269)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER2_LOOPBACK                 = 27,  //!< Enable serial port bridge: SER2 loopback (uint32 inv: 4294967268)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK             = 28,  //!< Enable serial port bridge: current port loopback (uint32 inv: 4294967267)
    SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK_TESTMODE    = 29,  //!< Enable serial port bridge: current port loopback, driver test mode (uint32 inv: 4294967266)

    SYS_CMD_GPX_ENABLE_BOOTLOADER_MODE                              = 30,  //!< GPX: enable bootloader mode (uint32 inv: 4294967265)
    SYS_CMD_GPX_ENABLE_GNSS1_CHIPSET_BOOTLOADER                     = 31,  //!< GPX: enable GNSS1 chipset bootloader (uint32 inv: 4294967264)
    SYS_CMD_GPX_ENABLE_GNSS2_CHIPSET_BOOTLOADER                     = 32,  //!< GPX: enable GNSS2 chipset bootloader (uint32 inv: 4294967263)
    SYS_CMD_GPX_ENABLE_GNSS1_PASS_THROUGH                           = 33,  //!< GPX: enable GNSS1 pass-through (uint32 inv: 4294967262)
    SYS_CMD_GPX_ENABLE_GNSS2_PASS_THROUGH                           = 34,  //!< GPX: enable GNSS2 pass-through (uint32 inv: 4294967261)
    SYS_CMD_GPX_HARD_RESET_GNSS1                                    = 36,  //!< GPX: hard reset GNSS1 receiver (uint32 inv: 4294967259)
    SYS_CMD_GPX_HARD_RESET_GNSS2                                    = 37,  //!< GPX: hard reset GNSS2 receiver (uint32 inv: 4294967258)
    SYS_CMD_GPX_SOFT_RESET_GPX                                      = 38,  //!< GPX: soft reset GPX (uint32 inv: 4294967257)
    SYS_CMD_GPX_ENABLE_SERIAL_BRIDGE_CUR_PORT_LOOPBACK              = 39,  //!< GPX: enable serial bridge on IMX to GPX and loopback on GPX (uint32 inv: 4294967256)
    SYS_CMD_GPX_ENABLE_SERIAL_BRIDGE_CUR_PORT_LOOPBACK_TESTMODE     = 40,  //!< GPX: enable serial bridge on IMX to GPX and loopback on GPX, driver test mode (uint32 inv: 4294967255)
    SYS_CMD_GPX_ENABLE_RTOS_STATS                                   = 41,  //!< GPX: enable RTOS task stats collection (uint32 inv: 4294967254)

    SYS_CMD_GNSS_RCVR_QUIET_MODE                                    = 60,  //!< Put GNSS receiver into quiet mode (uint32 inv: 4294967235)
    SYS_CMD_GNSS_RCVR_SOFT_RESET                                    = 61,  //!< Soft reset GNSS receiver (uint32 inv: 4294967234)
    SYS_CMD_GNSS_RCVR_HARD_RESET                                    = 62,  //!< Hard reset GNSS receiver (uint32 inv: 4294967233)

    SYS_CMD_RESET_EKF_STATES                                        = 70,  //!< Resets the Extended Kalman Filter (EKF) states in the INS solution. Use to reinitialize navigation filter without a full system reset. (uint32 inv: 4294967226)
    SYS_CMD_CLEAR_ERROR_STATUS                                      = 71,  //!< Resets debug information such as CPU reset cause and fault states. (uint32 inv: 4294967225)

    SYS_CMD_SAVE_FLASH                                              = 97,  //!< Save current flash config to persistent flash memory (uint32 inv: 4294967198)
    SYS_CMD_SAVE_GNSS_ASSIST_TO_FLASH_RESET                         = 98,  //!< Save GNSS assist data (ephemeris/almanac) to flash then reset (uint32 inv: 4294967197)
    SYS_CMD_SOFTWARE_RESET                                          = 99,  //!< Software reset (uint32 inv: 4294967196)
    SYS_CMD_MANF_UNLOCK                                             = 1122334455,  //!< Manufacturing unlock, required before any SYS_CMD_MANF_* command below is accepted (uint32 inv: 3172632840)
    SYS_CMD_MANF_ERASE_CALIBRATION_MOTION                           = 1357924678,  //!< Manufacturing: erase motion calibration. SYS_CMD_MANF_UNLOCK must be sent prior to this command. (uint32 inv: 2937042617)
    SYS_CMD_MANF_ERASE_CALIBRATION                                  = 1357924679,  //!< Manufacturing: erase calibration. SYS_CMD_MANF_UNLOCK must be sent prior to this command. (uint32 inv: 2937042616)
    SYS_CMD_MANF_FACTORY_RESET                                      = 1357924680,  //!< Manufacturing: factory reset. SYS_CMD_MANF_UNLOCK must be sent prior to this command. (uint32 inv: 2937042615)
    SYS_CMD_MANF_CHIP_ERASE                                         = 1357924681,  //!< Manufacturing: chip erase. SYS_CMD_MANF_UNLOCK must be sent prior to this command.  A device power cycle may be necessary to complete this command. (uint32 inv: 2937042614)
    SYS_CMD_MANF_DOWNGRADE_CALIBRATION                              = 1357924682,  //!< Manufacturing: downgrade calibration. SYS_CMD_MANF_UNLOCK must be sent prior to this command. (uint32 inv: 2937042613)
    SYS_CMD_MANF_ENABLE_ROM_BOOTLOADER                              = 1357924683,  //!< Manufacturing: enable ROM bootloader. SYS_CMD_MANF_UNLOCK must be sent prior to this command.  A device power cycle may be necessary to complete this command. (uint32 inv: 2937042612)
    SYS_CMD_MANF_LED_ON                                             = 1357924684,  //!< Enable testbed LED on IMX (sysStatus) and GPX (hdwStatus). (uint32 inv: 2937042611)
    SYS_CMD_MANF_LED_OFF                                            = 1357924685,  //!< Disable testbed LED on IMX (sysStatus) and GPX (hdwStatus). (uint32 inv: 2937042610)

    SYS_CMD_FAULT_TEST_TRIG_MALLOC                                  = 57005,  //!< Fault injection test: trigger a malloc-related fault
    SYS_CMD_FAULT_TEST_TRIG_HARD_FAULT                              = 57006,  //!< Fault injection test: trigger a hard fault
    SYS_CMD_FAULT_TEST_TRIG_WATCHDOG                                = 57007,  //!< Fault injection test: trigger a watchdog timeout
    SYS_CMD_FAULT_TEST_TRIG_FLASH_TORN_WRITE                        = 57008,  //!< Fault injection test (SN-7873): erase test page, start dword program, NVIC_SystemReset mid-flight to leave a torn dword with bad ECC; auto-fires NMI on next boot to exercise the full classify+recover cycle
};

/** @brief Serial port bridge routing mode, set via the SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_* / SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE commands in @ref eSystemCommand. Selects which two UARTs/USB are cross-connected so raw bytes on one port are echoed to the other (e.g. for pass-through configuration of a GNSS receiver, or port loopback testing). */
enum eSerialPortBridge
{
    SERIAL_PORT_BRIDGE_DISABLED         = 0,   //!< Serial port bridge disabled; ports operate normally

    SERIAL_PORT_BRIDGE_GNSS1_TO_USB     = 1,   //!< Bridge GNSS1 receiver port to USB
    SERIAL_PORT_BRIDGE_GNSS1_TO_SER0    = 2,   //!< Bridge GNSS1 receiver port to SER0
    SERIAL_PORT_BRIDGE_GNSS1_TO_SER1    = 3,   //!< Bridge GNSS1 receiver port to SER1
    SERIAL_PORT_BRIDGE_GNSS1_TO_SER2    = 4,   //!< Bridge GNSS1 receiver port to SER2

    SERIAL_PORT_BRIDGE_GNSS2_TO_USB     = 5,   //!< Bridge GNSS2 receiver port to USB
    SERIAL_PORT_BRIDGE_GNSS2_TO_SER0    = 6,   //!< Bridge GNSS2 receiver port to SER0
    SERIAL_PORT_BRIDGE_GNSS2_TO_SER1    = 7,   //!< Bridge GNSS2 receiver port to SER1
    SERIAL_PORT_BRIDGE_GNSS2_TO_SER2    = 8,   //!< Bridge GNSS2 receiver port to SER2

    SERIAL_PORT_BRIDGE_USB_TO_SER0      = 9,   //!< Bridge USB to SER0
    SERIAL_PORT_BRIDGE_USB_TO_SER1      = 10,  //!< Bridge USB to SER1
    SERIAL_PORT_BRIDGE_USB_TO_SER2      = 11,  //!< Bridge USB to SER2
    SERIAL_PORT_BRIDGE_SER0_TO_SER1     = 12,  //!< Bridge SER0 to SER1
    SERIAL_PORT_BRIDGE_SER0_TO_SER2     = 13,  //!< Bridge SER0 to SER2
    SERIAL_PORT_BRIDGE_SER1_TO_SER2     = 14,  //!< Bridge SER1 to SER2

    SERIAL_PORT_BRIDGE_USB_TO_USB       = 15,  //!< USB loopback (test mode)
    SERIAL_PORT_BRIDGE_SER0_TO_SER0     = 16,  //!< SER0 loopback (test mode)
    SERIAL_PORT_BRIDGE_SER1_TO_SER1     = 17,  //!< SER1 loopback (test mode)
    SERIAL_PORT_BRIDGE_SER2_TO_SER2     = 18,  //!< SER2 loopback (test mode)
};

#define NMEA_BUFFER_SIZE    256

/** @brief One NMEA message ID / broadcast-period pair, used as an element of @ref nmea_msgs_t.nmeaBroadcastMsgs (DID_NMEA_BCAST_PERIOD). */
typedef struct nmeaBroadcastMsgPair
{
    uint8_t msgID;      //!< Message ID. (see eNmeaMsgId)

    uint8_t msgPeriod;  //!< Message period multiple, relative to DID_FLASH_CONFIG.startupGnssDtMs. E.g. msgPeriod=1 broadcasts every navigation update, msgPeriod=5 broadcasts every 5th update.
} nmeaBroadcastMsgPair_t;

#define MAX_nmeaBroadcastMsgPairs   20

/** @brief (DID_NMEA_BCAST_PERIOD) Sets NMEA message broadcast periods. This data structure is zeroed out on stop_all_broadcasts. */
typedef struct PACKED
{
    uint32_t                options;    //!< Options: Port selection[0x0=current, 0x1=ser0, 0x2=ser1, 0x4=ser2, 0x8=USB, 0x100=preserve, 0x200=Persistent] (see RMC_OPTIONS_...)

    nmeaBroadcastMsgPair_t  nmeaBroadcastMsgs[MAX_nmeaBroadcastMsgPairs];  //!< NMEA message to be set.  Up to 20 message ID/period pairs.  Message ID of zero indicates the remaining pairs are not used. (see eNmeaMsgId)

    /*  Example usage:
     *  If you are setting message GGA (6) at 1Hz and GGL (7) at 5Hz with the default DID_FLASH_CONFIG.startupGnssDtMs = 200 (5Hz)
     *  nmeaBroadcastMsgs[0].msgID = 6, nmeaBroadcastMsgs[0].msgPeriod = 5
     *  nmeaBroadcastMsgs[1].msgID = 7, nmeaBroadcastMsgs[1].msgPeriod = 1 */

} nmea_msgs_t;

/** @brief Single-IMU sensor sample with temperature, embedded as an element of @ref sys_sensors_adc_t.imu (DID_SENSORS_ADC, DID_SENSORS_ADC_SIGMA). Values are raw ADC-scale/LSB counts for the uncalibrated ADC path; the units below apply once the corresponding calibration/scale factors have been applied. */
typedef struct PACKED
{
    float                   pqr[3];     //!< (rad/s) Gyros.  Units only apply for calibrated data.

    float                   acc[3];     //!< (m/s^2) Accelerometers.  Units only apply for calibrated data.

    float                   temp;       //!< (°C) Temperature of IMU.  Units only apply for calibrated data.
} sensors_imu_w_temp_t;

/** @brief Single-magnetometer sensor sample, embedded as an element of @ref sys_sensors_adc_t.mag (DID_SENSORS_ADC, DID_SENSORS_ADC_SIGMA). Units only apply for calibrated data. */
typedef struct PACKED
{
    float                   mag[3];     //!< (uT) Magnetometers
} sensors_mag_t;

/** @brief Combined gyro/accelerometer/magnetometer sample for a single IMU device, embedded as an element of @ref sensors_t.mpu (DID_SENSORS_TC_BIAS). Units only apply for calibrated data. */
typedef struct PACKED
{
    float                   pqr[3];     //!< (rad/s) Gyros.  Units only apply for calibrated data.

    float                   acc[3];     //!< (m/s^2) Accelerometers.  Units only apply for calibrated data.

    float                   mag[3];     //!< (uT) Magnetometers.  Units only apply for calibrated data.
} sensors_mpu_t;

/** @brief INTERNAL USE ONLY (DID_SENSORS_TC_BIAS) Per-device combined IMU + magnetometer sample set, used internally to track temperature-compensation bias state. */
typedef struct PACKED
{
    double                  time;                   //!< Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset. Units only apply for calibrated data.

    sensors_mpu_t           mpu[MAX_IMU_DEVICES];    //!< Per-device combined IMU + magnetometer sample
} sensors_t;

/** @brief Magnetometer XYZ sample, embedded as an element of @ref sensors_w_temp_t.mag. */
typedef struct PACKED
{
    float                   xyz[3];     //!< (uT) Magnetometer X, Y, Z.  Units only apply for calibrated data.
} mag_xyz_t;

/** @brief INTERNAL USE ONLY. Per-device IMU + temperature + magnetometer sample set shared by three calibration-stage DIDs: DID_SENSORS_UCAL carries uncalibrated (raw) IMU output, DID_SENSORS_TCAL carries temperature-compensated IMU output, and DID_SENSORS_MCAL carries temperature-compensated and motion-calibrated IMU output. */
typedef struct PACKED
{
    imus_t                  imus;                   //!< Per-device gyro/accelerometer samples (see imus_t)

    float                   temp[MAX_IMU_DEVICES];  //!< (°C) Temperature of IMU.  Units only apply for calibrated data.

    mag_xyz_t               mag[MAX_MAG_DEVICES];   //!< (uT) Magnetometers.  Units only apply for calibrated data.
} sensors_w_temp_t;

/** @brief Per-axis-group temperature-compensation state for one sensor unit (gyro, accelerometer, or magnetometer), embedded as an element of @ref sensor_compensation_v1p3_t / @ref sensor_compensation_v1p4_t (DID_SCOMP). Tracks the running temperature-compensation curve fit used to correct sensor bias/scale drift with temperature. */
typedef struct PACKED
{
    float                   lpfLsb[3];      //!< Low-pass filtered of g_sensors.lsb
    float                   lpfTemp;        //!< (°C) Low-pass filtered sensor temperature
    float                   k[3];           //!< Slope (moved from flash to here)
    float                   temp;           //!< (°C) Temperature of sensor
    float                   tempRampRate;   //!< (°C/s) Temperature ramp rate
    uint32_t                tci;            //!< Index of current temperature compensation point
    uint32_t                numTcPts;       //!< Total number of tc points
    float                   dtTemp;         //!< (°C) Temperature from last calibration point
} sensor_comp_unit_t;

/**
 * @brief INTERNAL USE ONLY (DID_SCOMP) IMX-5 wire-format variant of the sensor temperature/motion
 * compensation state (3 IMU + 2 mag). Explicit wire-format variants of the DID_SCOMP payload
 * exist because IMX-5 firmware sends v1.3 (3 IMU + 2 mag) while IMX-6 firmware sends v1.4 (5 IMU +
 * 1 mag). Host SDK keeps sensor_compensation_t at the v1.4 shape (MAX_IMU_DEVICES /
 * MAX_MAG_DEVICES) and converts v1.3 payloads on receive via convert_scomp_v1p3_to_v1p4() in
 * IS_calibration_convert.h.
 */
typedef struct PACKED
{
    uint32_t                timeMs;                                 //!< Time since boot up, in milliseconds
    sensor_comp_unit_t      pqr[NUM_IMU_DEVICES_V1P3];                //!< Per-IMU gyro temperature-compensation state
    sensor_comp_unit_t      acc[NUM_IMU_DEVICES_V1P3];                //!< Per-IMU accelerometer temperature-compensation state
    sensor_comp_unit_t      mag[NUM_MAG_DEVICES_V1P3];                //!< Per-magnetometer temperature-compensation state
    imui_t                  referenceImu;                            //!< Reference/truth IMU sample used during calibration
    float                   referenceMag[3];                        //!< (uT) Reference/truth magnetometer sample used during calibration
    uint32_t                sampleCount;                             //!< Number of samples collected for the current calibration state
    uint32_t                calState;                                //!< Current calibration state/step
    uint32_t                status;                                  //!< Calibration status flags
    float                   alignAccel[3];                           //!< (m/s^2) Accelerometer reading used for alignment/leveling during calibration
} sensor_compensation_v1p3_t;

/**
 * @brief INTERNAL USE ONLY (DID_SCOMP) IMX-6 wire-format variant of the sensor temperature/motion
 * compensation state (5 IMU + 1 mag). See @ref sensor_compensation_v1p3_t for the IMX-5 variant
 * and the rationale for the two wire formats; field meanings are identical between the two, only
 * the per-device array sizes differ.
 */
typedef struct PACKED
{
    uint32_t                timeMs;                                 //!< Time since boot up, in milliseconds
    sensor_comp_unit_t      pqr[NUM_IMU_DEVICES_V1P4];                //!< Per-IMU gyro temperature-compensation state
    sensor_comp_unit_t      acc[NUM_IMU_DEVICES_V1P4];                //!< Per-IMU accelerometer temperature-compensation state
    sensor_comp_unit_t      mag[NUM_MAG_DEVICES_V1P4];                //!< Per-magnetometer temperature-compensation state
    imui_t                  referenceImu;                            //!< Reference/truth IMU sample used during calibration
    float                   referenceMag[3];                        //!< (uT) Reference/truth magnetometer sample used during calibration
    uint32_t                sampleCount;                             //!< Number of samples collected for the current calibration state
    uint32_t                calState;                                //!< Current calibration state/step
    uint32_t                status;                                  //!< Calibration status flags
    float                   alignAccel[3];                           //!< (m/s^2) Accelerometer reading used for alignment/leveling during calibration
} sensor_compensation_v1p4_t;

/** (DID_SCOMP) INTERNAL USE ONLY - aliased to sensor_compensation_v1p4_t since
 * MAX_IMU_DEVICES / MAX_MAG_DEVICES are unconditionally v1.4 on every build target. */
typedef sensor_compensation_v1p4_t  sensor_compensation_t;

#define NUM_ANA_CHANNELS    4

/** @brief INTERNAL USE ONLY (DID_SENSORS_ADC, DID_SENSORS_ADC_SIGMA) Raw ADC/LSB-scale sensor output for all onboard sensors. DID_SENSORS_ADC carries the raw sample; DID_SENSORS_ADC_SIGMA carries the corresponding sample-to-sample standard deviation (same layout, used for noise/sigma analysis). LSB units apply for all fields except temperature, which is in Celsius. */
typedef struct PACKED
{
    double                  time;                   //!< Time since boot up in seconds
    sensors_imu_w_temp_t    imu[MAX_IMU_DEVICES];    //!< Per-device raw IMU (gyro/accel) + temperature samples
    sensors_mag_t           mag[MAX_MAG_DEVICES];   //!< Magnetometers
    float                   bar;                    //!< Barometric pressure
    float                   barTemp;                //!< (°C) Temperature of barometric pressure sensor
    float                   humidity;               //!< Relative humidity as a percent (%rH).  Range is 0% - 100%
    float                   ana[NUM_ANA_CHANNELS];  //!< ADC analog input
} sys_sensors_adc_t;

#if defined(IMX_5)
    #define NUM_COM_PORTS           4
#elif defined(IMX_6)
    #define NUM_COM_PORTS           4
#elif defined(GPX_1)
    #define NUM_COM_PORTS           6
    #define NUM_USR_PORTS           4
    #define NUM_GNSS_PORTS           2
#else   // NPP and Unit Tests
    #define NUM_COM_PORTS           6
#endif

#define NUM_SERIAL_PORTS            6

#ifndef NUM_USR_PORTS
#define NUM_USR_PORTS           NUM_COM_PORTS
#endif

/** Realtime Message Controller (used in rmc_t).
    The data sets available through RMC are broadcast at the availability of the data.  A goal of RMC is
    to provide updates from each onboard sensor as fast as possible with minimal latency.  The RMC is
    provided so that broadcast of sensor data is done as soon as it becomes available.   The exception to
    this rule is the INS output data, which has a configurable output data rate according to DID_RMC.insPeriodMs.
*/

#define RMC_OPTIONS_PORT_MASK                   0x000000FF
#define RMC_OPTIONS_PORT_ALL                    (RMC_OPTIONS_PORT_MASK)
#define RMC_OPTIONS_PORT_CURRENT                0x00000000
#define RMC_OPTIONS_PORT_SER0                   0x00000001
#define RMC_OPTIONS_PORT_SER1                   0x00000002                  // also SPI
#define RMC_OPTIONS_PORT_SER2                   0x00000004
#define RMC_OPTIONS_PORT_USB                    0x00000008
#define RMC_OPTIONS_PRESERVE_CTRL               0x00000100                  // Prevent any messages from getting turned off by bitwise OR'ing new message bits with current message bits.
#define RMC_OPTIONS_PERSISTENT                  0x00000200                  // Save current port RMC to flash memory for use following reboot, eliminating need to re-enable RMC to start data streaming.
#define RMC_OPTIONS_NMEA_SPEED_FILTER_BITMASK   0x00000C00                  // Enable speed filtering (NMEA message only, i.e. GLL, RMC, VTG).  This filters out small velocity caused by system noise.
#define RMC_OPTIONS_NMEA_SPEED_FILTER_OFFSET    10                          // Bit offset for NMEA speed filter options
#define RMC_OPTIONS_NMEA_SPEED_FILTER_ENABLE    1                           // NMEA speed filtering: Enable
#define RMC_OPTIONS_NMEA_SPEED_FILTER_DISABLE   2                           // NMEA speed filtering: Disable

                                                                // RMC message data rates:
#define RMC_BITS_INS1                   0x0000000000000001  // rmc.insPeriodMs (4ms default)
#define RMC_BITS_INS2                   0x0000000000000002  // "
#define RMC_BITS_INS3                   0x0000000000000004  // "
#define RMC_BITS_INS4                   0x0000000000000008  // "
#define RMC_BITS_IMU                    0x0000000000000010  // DID_FLASH_CONFIG.startupNavDtMs (4ms default)
#define RMC_BITS_PIMU                   0x0000000000000020  // "
#define RMC_BITS_BAROMETER              0x0000000000000040  // ~8ms
#define RMC_BITS_MAGNETOMETER           0x0000000000000080  // ~10ms
#define RMC_BITS_IMUS                   0x0000000000000100  // DID_FLASH_CONFIG.startupNavDtMs (4ms default)
// #define RMC_BITS_UNUSED                 0x0000000000000200
#define RMC_BITS_GNSS1_POS          0x0000000000000400  // DID_FLASH_CONFIG.startupGnssDtMs (200ms default)
#define RMC_BITS_GNSS2_POS          0x0000000000000800  // "
#define RMC_BITS_GNSS1_RAW          0x0000000000001000  // "
#define RMC_BITS_GNSS2_RAW          0x0000000000002000  // "
#define RMC_BITS_GNSS1_SAT          0x0000000000004000  // 1s
#define RMC_BITS_GNSS2_SAT          0x0000000000008000  // "
#define RMC_BITS_GNSS_BASE_RAW      0x0000000000010000  //
#define RMC_BITS_STROBE_IN_TIME     0x0000000000020000  // On strobe input event
#define RMC_BITS_DIAGNOSTIC_MESSAGE 0x0000000000040000
#define RMC_BITS_IMUS_UNCAL         0x0000000000080000  // DID_FLASH_CONFIG.startupImuDtMs (1ms default)
#define RMC_BITS_GNSS1_VEL          0x0000000000100000  // DID_FLASH_CONFIG.startupGnssDtMs (200ms default)
#define RMC_BITS_GNSS2_VEL          0x0000000000200000  // "
#define RMC_BITS_GNSS1_UBX_POS      0x0000000000400000  // "
#define RMC_BITS_GNSS1_RTK_POS      0x0000000000800000  // "
#define RMC_BITS_GNSS1_RTK_POS_REL  0x0000000001000000  // "
#define RMC_BITS_GNSS1_RTK_POS_MISC 0x0000000004000000  // "
#define RMC_BITS_INL2_NED_SIGMA     0x0000000008000000
#define RMC_BITS_RTK_STATE          0x0000000010000000
#define RMC_BITS_RTK_CODE_RESIDUAL  0x0000000020000000
#define RMC_BITS_RTK_PHASE_RESIDUAL 0x0000000040000000
#define RMC_BITS_WHEEL_ENCODER      0x0000000080000000
#define RMC_BITS_GROUND_VEHICLE     0x0000000100000000
// #define RMC_BITS_UNUSED                 0x0000000200000000
#define RMC_BITS_IMU_MAG                0x0000000400000000
#define RMC_BITS_PIMU_MAG               0x0000000800000000
#define RMC_BITS_GNSS1_RTK_HDG_REL      0x0000001000000000  // DID_FLASH_CONFIG.startupGnssDtMs (200ms default)
#define RMC_BITS_GNSS1_RTK_HDG_MISC     0x0000002000000000  // "
#define RMC_BITS_REFERENCE_IMU          0x0000004000000000  // DID_FLASH_CONFIG.startupNavDtMs
#define RMC_BITS_REFERENCE_PIMU         0x0000008000000000  // "
#define RMC_BITS_IMUS_RAW               0x0000010000000000
#define RMC_BITS_IMU_RAW                0x0000020000000000
#define RMC_BITS_GNSS1_SIG              0x0000040000000000  // 1s
#define RMC_BITS_GNSS2_SIG              0x0000080000000000

// GPX messages could go into a local grmc if imx memory we expanded. (TM)
#define RMC_BITS_GPX_RTOS_INFO          0x0000100000000000
#define RMC_BITS_GPX_DEBUG_ARRAY        0x0000200000000000
#define RMC_BITS_GPX_STATUS             0x0000400000000000
#define RMC_BITS_GPX_DEV_INFO           0x0000800000000000
#define RMC_BITS_GPX_RMC                0x0001000000000000
#define RMC_BITS_GPX_SYS_FAULT          0x0002000000000000
#define RMC_BITS_GPX_BIT                0x0004000000000000
#define RMC_BITS_GPX_PORT_MON           0x0008000000000000
#define RMC_BITS_GPX_RTK_DBG            0x0010000000000000

#define RMC_BITS_EVENT                  0x0800000000000000

#define RMC_BITS_MASK                   0x0FFFFFFFFFFFFFFF
#define RMC_BITS_INTERNAL_PPD           0x4000000000000000  //
#define RMC_BITS_PRESET                 0x8000000000000000  // Indicate BITS is a preset.  This sets the rmc period multiple and enables broadcasting.

#define RMC_PRESET_PPD_NAV_PERIOD_MULT_MS   100     // uint8
#define RMC_PRESET_PPD_IMUS_PERIOD_MULT     255     // uint8

// Preset: Post Processing Data
#define RMC_PRESET_IMX_PPD_NO_IMU           (RMC_BITS_PRESET \
                                            | RMC_BITS_INS2 \
                                            | RMC_BITS_BAROMETER \
                                            | RMC_BITS_MAGNETOMETER \
                                            | RMC_BITS_GNSS1_POS \
                                            | RMC_BITS_GNSS2_POS \
                                            | RMC_BITS_GNSS1_VEL \
                                            | RMC_BITS_GNSS2_VEL \
                                            | RMC_BITS_GNSS1_RAW \
                                            | RMC_BITS_GNSS2_RAW \
                                            | RMC_BITS_GNSS_BASE_RAW \
                                            | RMC_BITS_GNSS1_RTK_POS_REL \
                                            | RMC_BITS_GNSS1_RTK_HDG_REL \
                                            | RMC_BITS_GPX_STATUS \
                                            | RMC_BITS_GPX_DEBUG_ARRAY \
                                            | RMC_BITS_INTERNAL_PPD \
                                            | RMC_BITS_DIAGNOSTIC_MESSAGE\
                                            | RMC_BITS_GPX_SYS_FAULT)
#define RMC_PRESET_IMX_PPD                  (RMC_PRESET_IMX_PPD_NO_IMU \
                                            | RMC_BITS_PIMU \
                                            | RMC_BITS_REFERENCE_PIMU)
#define RMC_PRESET_IMX_PPD_IMUS_RAW         (RMC_PRESET_IMX_PPD_NO_IMU \
                                            | RMC_BITS_IMUS_RAW \
                                            | RMC_BITS_PIMU)
#define RMC_PRESET_IMX_PPD_IMUS_UNCAL       (RMC_PRESET_IMX_PPD_NO_IMU \
                                            | RMC_BITS_IMUS_UNCAL \
                                            | RMC_BITS_PIMU)
#define RMC_PRESET_INS                      (RMC_BITS_INS2 \
                                            | RMC_BITS_GNSS1_POS \
                                            | RMC_BITS_PRESET)
#define RMC_PRESET_IMX_PPD_RTK_DBG          (RMC_PRESET_IMX_PPD \
                                            | RMC_BITS_RTK_STATE \
                                            | RMC_BITS_RTK_CODE_RESIDUAL \
                                            | RMC_BITS_RTK_PHASE_RESIDUAL \
                                            | RMC_BITS_GPX_DEBUG_ARRAY \
                                            | RMC_BITS_GNSS1_SAT \
                                            | RMC_BITS_GNSS2_SAT \
                                            | RMC_BITS_EVENT \
                                            | RMC_BITS_GPX_RTK_DBG)
#define RMC_PRESET_IMX_PPD_GROUND_VEHICLE   (RMC_PRESET_IMX_PPD \
                                            | RMC_BITS_WHEEL_ENCODER \
                                            | RMC_BITS_GROUND_VEHICLE)
#define RMC_PRESET_ALLAN_VARIANCE           (RMC_BITS_PRESET \
                                            | RMC_BITS_IMU)
#define RMC_PRESET_ALLAN_VARIANCE_IMUS      (RMC_BITS_PRESET \
                                            | RMC_BITS_IMU \
                                            | RMC_BITS_IMUS)
#define RMC_PRESET_GNSS (RMC_BITS_PRESET \
                                            | RMC_BITS_GNSS1_POS \
                                            | RMC_BITS_GNSS2_POS \
                                            | RMC_BITS_GNSS1_VEL \
                                            | RMC_BITS_GNSS2_VEL \
                                            | RMC_BITS_GNSS1_RTK_POS_REL \
                                            | RMC_BITS_GNSS1_RTK_HDG_REL \
                                            | RMC_BITS_GPX_DEBUG_ARRAY \
                                            | RMC_BITS_GPX_PORT_MON \
                                            | RMC_BITS_EVENT \
                                            | RMC_BITS_GPX_STATUS\
                                            | RMC_BITS_GPX_SYS_FAULT)
#define RMC_PRESET_GPX_PPD                  (RMC_BITS_PRESET \
                                            | RMC_PRESET_GNSS \
                                            | RMC_BITS_GNSS1_RAW \
                                            | RMC_BITS_GNSS2_RAW \
                                            | RMC_BITS_GNSS_BASE_RAW)

/** @brief (DID_RMC, DID_GPX_RMC) Realtime Message Controller (RMC) enable state. Selects which data sets are broadcast and on which port(s); see the RMC description above and the RMC_BITS_... / RMC_OPTIONS_... defines for the full bit layout. Also embedded as the live broadcast-enable state inside @ref rmci_t and @ref grmci_t. IMU and Integrated IMU (PIMU) data transmit period is set separately using DID_SYS_PARAMS.navPeriodMs, not by this struct. */
typedef struct PACKED
{
    uint64_t                bits;       //!< Data stream enable bits for the specified ports (see RMC_BITS_...)

    uint32_t                options;    //!< Options to select alternate ports to output data, persist across reboot, NMEA speed filtering, etc. (see RMC_OPTIONS_...)
} rmc_t;

#define NMEA_GNGSV_FREQ_BAND1_BIT   (0x01)
#define NMEA_GNGSV_FREQ_BAND2_BIT   (0x01 << 1)
#define NMEA_GNGSV_FREQ_BAND3_BIT   (0x01 << 2)
#define NMEA_GNGSV_FREQ_5_BIT       (0x01 << 3)

#define NMEA_GNGSV_GPS_OFFSET       (SAT_SV_GNSS_ID_GPS << 4)
#define NMEA_GNGSV_GAL_OFFSET       (SAT_SV_GNSS_ID_GAL << 4)
#define NMEA_GNGSV_BEI_OFFSET       (SAT_SV_GNSS_ID_BEI << 4)
#define NMEA_GNGSV_QZS_OFFSET       (SAT_SV_GNSS_ID_QZS << 4)
#define NMEA_GNGSV_GLO_OFFSET       (SAT_SV_GNSS_ID_GLO << 4)

/** @brief NMEA/proprietary-sentence message ID used to identify talkers for RMC/ASCE broadcast enable, nmea_talker_to_id()/nmea_id_to_talker(), and nmeaBroadcastMsgPair_t.msgID. IDs below NMEA_MSG_ID_COUNT are "base" message types; IDs at/above NMEA_MSG_ID_SPECIAL_CASE_START are special-case encodings layered on a base message (currently only $..GSV variants, see below). */
enum eNmeaMsgId
{
    NMEA_MSG_ID_INVALID             = 0,   //!< Invalid/unrecognized NMEA message ID
    NMEA_MSG_ID_PIMU                = 1,   //!< $PIMU proprietary preintegrated IMU (delta theta / delta velocity) sentence
    NMEA_MSG_ID_PPIMU               = 2,   //!< $PPIMU proprietary preintegrated IMU sentence
    NMEA_MSG_ID_PRIMU               = 3,   //!< $PRIMU proprietary raw/reference IMU (angular rate + acceleration) sentence
    NMEA_MSG_ID_PINS1               = 4,   //!< $PINS1 proprietary INS sentence (Euler angles, LLA, NED velocity)
    NMEA_MSG_ID_PINS2               = 5,   //!< $PINS2 proprietary INS sentence (quaternion, LLA)
    NMEA_MSG_ID_PGPSP               = 6,   //!< $PGPSP proprietary GNSS position and velocity sentence
    NMEA_MSG_ID_GNGGA               = 7,   //!< $GNGGA standard NMEA GNSS fix data sentence
    NMEA_MSG_ID_GNGLL               = 8,   //!< $GNGLL standard NMEA geographic position (lat/lon) sentence
    NMEA_MSG_ID_GNGSA               = 9,   //!< $GNGSA standard NMEA GNSS DOP and active satellites sentence
    NMEA_MSG_ID_GNRMC               = 10,  //!< $GNRMC standard NMEA recommended minimum navigation information sentence
    NMEA_MSG_ID_GNZDA               = 11,  //!< $GNZDA standard NMEA time and date sentence
    NMEA_MSG_ID_PASHR               = 12,  //!< $PASHR proprietary attitude (roll/pitch/heading) sentence
    NMEA_MSG_ID_PSTRB               = 13,  //!< $PSTRB proprietary strobe-input timestamp sentence
    NMEA_MSG_ID_INFO                = 14,  //!< $INFO proprietary device info sentence
    NMEA_MSG_ID_GNGSV               = 15,  //!< $GNGSV standard NMEA satellites-in-view sentence (all constellations); see the special-case GSV variants below for per-constellation/frequency filtering
    NMEA_MSG_ID_GNVTG               = 16,  //!< $GNVTG standard NMEA course and speed over ground sentence
    NMEA_MSG_ID_INTEL               = 17,  //!< $INTEL proprietary GNSS sentence (IS-internal extended device info + position + velocity format)
    NMEA_MSG_ID_POWGPS              = 18,  //!< $POWGPS proprietary GNSS position sentence
    NMEA_MSG_ID_POWTLV              = 19,  //!< $POWTLV proprietary GNSS position and velocity sentence
    NMEA_MSG_ID_COUNT,                     //!< Number of base (non special-case) NMEA message IDs

    // IMX/GPX Input Commands
    NMEA_MSG_ID_ASCE,         //!< "ASCE" - NMEA messages broadcast enable
    NMEA_MSG_ID_BLEN,         //!< "BLEN" - Enable bootloader on IMX (app firmware update)
    NMEA_MSG_ID_EBLE,         //!< "EBLE" - Enable bootloader on EVB
    NMEA_MSG_ID_NELB,         //!< "NELB" - Enable SAM-BA mode
    NMEA_MSG_ID_PERS,         //!< "PERS" - Save perstent messages
    NMEA_MSG_ID_SRST,         //!< "SRTS" - Software reset
    NMEA_MSG_ID_STPB,         //!< "STPB" - Stop broadcasts on all ports
    NMEA_MSG_ID_STPC,         //!< "STPC" - Stop broadcasts on current port

    // Special case messages for each supported base message those with ID less than NMEA_MSG_ID_COUNT.
    // Each base message get a 256 range of ID's for their special cases. Example for NMEA_MSG_ID_GNGSV:
    // NMEA_MSG_ID_GNGSV_START = NMEA_MSG_ID_GNGSV * NMEA_MSG_ID_SPECIAL_CASE_START giving a message ID 0x0f00 (3,840)
    // NOTE: Any ID greater than 256 is a special case, use the follow to extract the root case:
    //   if (msgId >= NMEA_MSG_ID_SPECIAL_CASE_START) msgId >>= 8;
    NMEA_MSG_ID_SPECIAL_CASE_START  = 256, //!< First value in the special-case ID range; base_id * 256 is the start of that base message's special-case block

    // Filtered GNGSV NMEA Message IDs:

    // GNGSV - All constellations
    NMEA_MSG_ID_GNGSV_START         = NMEA_MSG_ID_GNGSV * NMEA_MSG_ID_SPECIAL_CASE_START,  //!< (3840) Used for reference only
    NMEA_MSG_ID_GNGSV_0             = NMEA_MSG_ID_GNGSV_START,  //!< GNGSV_0 (3840) Clear all constellations and frequencies
    NMEA_MSG_ID_GNGSV_1             = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND1_BIT),  //!< GNGSV_1 (3841) Enable all constellations band1
    NMEA_MSG_ID_GNGSV_2             = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND2_BIT),  //!< GNGSV_2 (3842) Enable all constellations band2
    NMEA_MSG_ID_GNGSV_2_1           = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND2_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),  //!< GNGSV_2_1 (3843) Enable all constellations band1, band2
    NMEA_MSG_ID_GNGSV_3             = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND3_BIT),  //!< GNGSV_3 (3844) Enable all constellations band3
    NMEA_MSG_ID_GNGSV_3_1           = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),  //!< GNGSV_3_1 (3845) Enable all constellations band1, band3
    NMEA_MSG_ID_GNGSV_3_2           = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND2_BIT),  //!< GNGSV_3_2 (3846) Enable all constellations band2, band3
    NMEA_MSG_ID_GNGSV_3_2_1         = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND2_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),  //!< GNGSV_3_2_1 (3847) Enable all constellations band1, band2, band3
    NMEA_MSG_ID_GNGSV_5             = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT),  //!< GNGSV_5 (3848) Enable all constellations band5
    NMEA_MSG_ID_GNGSV_5_1           = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),  //!< GNGSV_5_1 (3849) Enable all constellations band1, band5
    NMEA_MSG_ID_GNGSV_5_2           = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND2_BIT),  //!< GNGSV_5_2 (3850) Enable all constellations band2, band5
    NMEA_MSG_ID_GNGSV_5_2_1         = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND2_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),  //!< GNGSV_5_2_1 (3851) Enable all constellations band1, band2, band5
    NMEA_MSG_ID_GNGSV_5_3           = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND3_BIT),  //!< GNGSV_5_3 (3852) Enable all constellations band3, band5
    NMEA_MSG_ID_GNGSV_5_3_1         = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),  //!< GNGSV_5_3_1 (3853) Enable all constellations band1, band3, band5
    NMEA_MSG_ID_GNGSV_5_3_2         = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND2_BIT),  //!< GNGSV_5_3_2 (3854) Enable all constellations band2, band3, band5
    NMEA_MSG_ID_GNGSV_5_3_2_1       = (NMEA_MSG_ID_GNGSV_START | NMEA_GNGSV_FREQ_5_BIT | NMEA_GNGSV_FREQ_BAND3_BIT | NMEA_GNGSV_FREQ_BAND2_BIT | NMEA_GNGSV_FREQ_BAND1_BIT),  //!< GNGSV_5_3_2_1 / plain GNGSV (3855) Enable all constellations and all frequencies (band1, band2, band3, band5)

    // GPGSV - GPS
    NMEA_MSG_ID_GPGSV_0             = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV_0 (3856) Disable all GPS frequencies
    NMEA_MSG_ID_GPGSV_1             = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV_1 (3857) Enable GPS L1
    NMEA_MSG_ID_GPGSV_2             = (NMEA_MSG_ID_GNGSV_2 + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV_2 (3858) Enable GPS L2
    NMEA_MSG_ID_GPGSV_2_1           = (NMEA_MSG_ID_GNGSV_2_1 + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV_2_1 (3859) Enable GPS L1, L2
    NMEA_MSG_ID_GPGSV_5             = (NMEA_MSG_ID_GNGSV_5 + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV_5 (3864) Enable GPS L5
    NMEA_MSG_ID_GPGSV_5_1           = (NMEA_MSG_ID_GNGSV_5_1 + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV_5_1 (3865) Enable GPS L1, L5
    NMEA_MSG_ID_GPGSV_5_2           = (NMEA_MSG_ID_GNGSV_5_2 + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV_5_2 (3866) Enable GPS L2, L5
    NMEA_MSG_ID_GPGSV_5_2_1         = (NMEA_MSG_ID_GNGSV_5_2_1 + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV_5_2_1 (3867) Enable GPS L1, L2, L5
    NMEA_MSG_ID_GPGSV               = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_GPS_OFFSET),  //!< GPGSV (3871) Enable all GPS frequencies

    // GAGSV - Galileo
    NMEA_MSG_ID_GAGSV_0             = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_GAL_OFFSET),  //!< GAGSV_0 (3888) Disable all Galileo frequencies
    NMEA_MSG_ID_GAGSV_1             = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_GAL_OFFSET),  //!< GAGSV_1 (3889) Enable Galileo E1
    NMEA_MSG_ID_GAGSV_5             = (NMEA_MSG_ID_GNGSV_5 + NMEA_GNGSV_GAL_OFFSET),  //!< GAGSV_5 (3896) Enable Galileo E5
    NMEA_MSG_ID_GAGSV_5_1           = (NMEA_MSG_ID_GNGSV_5_1 + NMEA_GNGSV_GAL_OFFSET),  //!< GAGSV_5_1 (3897) Enable Galileo E1, E5
    NMEA_MSG_ID_GAGSV               = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_GAL_OFFSET),  //!< GAGSV (3903) Enable all Galileo frequencies

    // GBGSV - Beido
    NMEA_MSG_ID_GBGSV_0             = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV_0 (3904) Disable all Beidou frequencies
    NMEA_MSG_ID_GBGSV_1             = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV_1 (3905) Enable Beidou B1
    NMEA_MSG_ID_GBGSV_2             = (NMEA_MSG_ID_GNGSV_2 + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV_2 (3906) Enable Beidou B2
    NMEA_MSG_ID_GBGSV_2_1           = (NMEA_MSG_ID_GNGSV_2_1 + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV_2_1 (3907) Enable Beidou B1, B2
    NMEA_MSG_ID_GBGSV_3             = (NMEA_MSG_ID_GNGSV_3 + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV_3 (3908) Enable Beidou B3
    NMEA_MSG_ID_GBGSV_3_1           = (NMEA_MSG_ID_GNGSV_3_1 + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV_3_1 (3909) Enable Beidou B1, B3
    NMEA_MSG_ID_GBGSV_3_2           = (NMEA_MSG_ID_GNGSV_3_2 + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV_3_2 (3910) Enable Beidou B2, B3
    NMEA_MSG_ID_GBGSV_3_2_1         = (NMEA_MSG_ID_GNGSV_3_2_1 + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV_3_2_1 (3911) Enable Beidou B1, B2, B3
    NMEA_MSG_ID_GBGSV               = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_BEI_OFFSET),  //!< GBGSV (3919) Enable all Beidou frequencies

    // GQGSV - QZSS
    NMEA_MSG_ID_GQGSV_0             = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV_0 (3920) Disable all QZSS frequencies
    NMEA_MSG_ID_GQGSV_1             = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV_1 (3921) Enable QZSS L1
    NMEA_MSG_ID_GQGSV_2             = (NMEA_MSG_ID_GNGSV_2 + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV_2 (3922) Enable QZSS L2
    NMEA_MSG_ID_GQGSV_2_1           = (NMEA_MSG_ID_GNGSV_2_1 + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV_2_1 (3923) Enable QZSS L1, L2
    NMEA_MSG_ID_GQGSV_5             = (NMEA_MSG_ID_GNGSV_5 + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV_5 (3928) Enable QZSS L5
    NMEA_MSG_ID_GQGSV_5_1           = (NMEA_MSG_ID_GNGSV_5_1 + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV_5_1 (3929) Enable QZSS L1, L5
    NMEA_MSG_ID_GQGSV_5_2           = (NMEA_MSG_ID_GNGSV_5_2 + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV_5_2 (3930) Enable QZSS L2, L5
    NMEA_MSG_ID_GQGSV_5_2_1         = (NMEA_MSG_ID_GNGSV_5_2_1 + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV_5_2_1 (3931) Enable QZSS L1, L2, L5
    NMEA_MSG_ID_GQGSV               = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_QZS_OFFSET),  //!< GQGSV (3935) Enable all QZSS frequencies

    // GLGSV - Glonass
    NMEA_MSG_ID_GLGSV_0             = (NMEA_MSG_ID_GNGSV_START + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV_0 (3936) Disable all Glonass frequencies
    NMEA_MSG_ID_GLGSV_1             = (NMEA_MSG_ID_GNGSV_1 + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV_1 (3937) Enable Glonass L1
    NMEA_MSG_ID_GLGSV_2             = (NMEA_MSG_ID_GNGSV_2 + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV_2 (3938) Enable Glonass L2
    NMEA_MSG_ID_GLGSV_2_1           = (NMEA_MSG_ID_GNGSV_2_1 + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV_2_1 (3939) Enable Glonass L1, L2
    NMEA_MSG_ID_GLGSV_3             = (NMEA_MSG_ID_GNGSV_3 + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV_3 (3940) Enable Glonass L3
    NMEA_MSG_ID_GLGSV_3_1           = (NMEA_MSG_ID_GNGSV_3_1 + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV_3_1 (3941) Enable Glonass L1, L3
    NMEA_MSG_ID_GLGSV_3_2           = (NMEA_MSG_ID_GNGSV_3_2 + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV_3_2 (3942) Enable Glonass L2, L3
    NMEA_MSG_ID_GLGSV_3_2_1         = (NMEA_MSG_ID_GNGSV_3_2_1 + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV_3_2_1 (3943) Enable Glonass L1, L2, L3
    NMEA_MSG_ID_GLGSV               = (NMEA_MSG_ID_GNGSV_5_3_2_1 + NMEA_GNGSV_GLO_OFFSET),  //!< GLGSV (3951) Enable all Glonass frequencies

    NMEA_MSG_ID_GNGSV_END           = NMEA_MSG_ID_GLGSV,  //!< (3951) Used for reference only
};

/** @brief Per-constellation NMEA $..GSV output filter mask, indexed by eSatSvGnssId. Used internally by the NMEA GSV special-case ID handling (NMEA_MSG_ID_GNGSV_START..NMEA_MSG_ID_GNGSV_END) to track which frequency bands are enabled for each GNSS constellation. */
typedef struct {
    uint8_t constMask[SAT_SV_GNSS_ID_COUNT];    //!< Per-constellation frequency-band enable bitmask (NMEA_GNGSV_FREQ_BAND1_BIT/BAND2_BIT/FREQ_5_BIT), indexed by eSatSvGnssId
} gsvMask_t;

#define NMEA_RMC_BITS_PIMU          (1<<NMEA_MSG_ID_PIMU)
#define NMEA_RMC_BITS_PPIMU         (1<<NMEA_MSG_ID_PPIMU)
#define NMEA_RMC_BITS_PRIMU         (1<<NMEA_MSG_ID_PRIMU)
#define NMEA_RMC_BITS_PINS1         (1<<NMEA_MSG_ID_PINS1)
#define NMEA_RMC_BITS_PINS2         (1<<NMEA_MSG_ID_PINS2)
#define NMEA_RMC_BITS_PGPSP         (1<<NMEA_MSG_ID_PGPSP)
#define NMEA_RMC_BITS_GNGGA         (1<<NMEA_MSG_ID_GNGGA)
#define NMEA_RMC_BITS_GNGLL         (1<<NMEA_MSG_ID_GNGLL)
#define NMEA_RMC_BITS_GNGSA         (1<<NMEA_MSG_ID_GNGSA)
#define NMEA_RMC_BITS_GNRMC         (1<<NMEA_MSG_ID_GNRMC)
#define NMEA_RMC_BITS_GNZDA         (1<<NMEA_MSG_ID_GNZDA)
#define NMEA_RMC_BITS_PASHR         (1<<NMEA_MSG_ID_PASHR)
#define NMEA_RMC_BITS_PSTRB         (1<<NMEA_MSG_ID_PSTRB)
#define NMEA_RMC_BITS_INFO          (1<<NMEA_MSG_ID_INFO)
#define NMEA_RMC_BITS_GNGSV         (1<<NMEA_MSG_ID_GNGSV)
#define NMEA_RMC_BITS_GNVTG         (1<<NMEA_MSG_ID_GNVTG)
#define NMEA_RMC_BITS_INTEL         (1<<NMEA_MSG_ID_INTEL)
#define NMEA_RMC_BITS_POWGPS        (1<<NMEA_MSG_ID_POWGPS)
#define NMEA_RMC_BITS_POWTLV        (1<<NMEA_MSG_ID_POWTLV)

/** @brief NMEA sentence broadcast enable/period state, embedded inside @ref rmci_t and @ref grmci_t alongside the binary-DID rmc_t/periodMultiple fields. Tracks, per base NMEA message ID (see eNmeaMsgId), whether the sentence is enabled and at what period multiple. */
typedef struct PACKED
{
    uint32_t                nmeaBits;                       //!< NMEA sentence enable bits, one per base message ID (see NMEA_RMC_BITS_...)

    uint8_t                 nmeaPeriod[NMEA_MSG_ID_COUNT];  //!< NMEA output period multiple of the underlying data's base period, indexed by eNmeaMsgId (base message IDs only)

}rmcNmea_t;


/** @brief Classic CAN 2.0 message ID, one per single-value/small field carved out of an ISB DID for legacy CAN bus output. Each ID maps to exactly one CAN_ISB_*_to_CAN_* encoder in protocol_CAN.cpp (see CAN_ISB_dispatch()) and has an independent period multiplier/address in can_config_t. */
enum can_cid_t
{
    CID_INS_TIME = 0,      //!< INS time (from ins_1_t)
    CID_INS_STATUS,        //!< INS status flags (from ins_1_t, see eInsStatusFlags)
    CID_INS_EULER,         //!< INS Euler angles (from ins_1_t)
    CID_INS_QUATN2B,       //!< INS NED-to-body quaternion (from ins_2_t)
    CID_INS_QUATE2B,       //!< INS ECEF-to-body quaternion (from ins_4_t)
    CID_INS_UVW,           //!< INS body-frame velocity U,V,W (from ins_1_t)
    CID_INS_VE,            //!< INS ECEF velocity (from ins_4_t)
    CID_INS_LAT,           //!< INS latitude (from ins_1_t)
    CID_INS_LON,           //!< INS longitude (from ins_1_t)
    CID_INS_ALT,           //!< INS altitude (from ins_1_t)
    CID_INS_NORTH_EAST,    //!< INS NED north/east position (from ins_1_t)
    CID_INS_DOWN,          //!< INS NED down position (from ins_1_t)
    CID_INS_ECEF_X,        //!< INS ECEF X position (from ins_4_t)
    CID_INS_ECEF_Y,        //!< INS ECEF Y position (from ins_4_t)
    CID_INS_ECEF_Z,        //!< INS ECEF Z position (from ins_4_t)
    CID_INS_MSL,           //!< INS MSL altitude (from ins_3_t)
    CID_PREINT_PX,         //!< Preintegrated IMU (PIMU) P-axis / X delta theta+velocity (from pimu_t)
    CID_PREINT_QY,         //!< Preintegrated IMU (PIMU) Q-axis / Y delta theta+velocity (from pimu_t)
    CID_PREINT_RZ,         //!< Preintegrated IMU (PIMU) R-axis / Z delta theta+velocity (from pimu_t)
    CID_DUAL_PX,           //!< Dual/raw IMU P-axis / X gyro+accel (from imu_t)
    CID_DUAL_QY,           //!< Dual/raw IMU Q-axis / Y gyro+accel (from imu_t)
    CID_DUAL_RZ,           //!< Dual/raw IMU R-axis / Z gyro+accel (from imu_t)
    CID_GNSS1_POS,         //!< GNSS 1 position (from gnss_pos_t)
    CID_GNSS2_POS,         //!< GNSS 2 position (from gnss_pos_t)
    CID_GNSS1_RTK_POS_REL, //!< GNSS 1 RTK relative position (from gnss_rtk_rel_t)
    CID_GNSS2_RTK_CMP_REL, //!< GNSS 2 RTK compassing relative position (from gnss_rtk_rel_t)
    CID_ROLL_ROLLRATE,     //!< Roll angle and roll rate (from imu_t)
    NUM_CIDS               //!< Number of classic CAN message IDs
};

/** @brief CAN FD message IDs — one frame per source DID, using native float/double precision (unlike can_cid_t, which splits a DID across several fixed-point CAN 2.0 frames). Each ID maps to one CANFD_ISB_*_to_CAN_* encoder (see CANFD_ISB_dispatch() in protocol_CAN.cpp). */
enum canfd_cid_t
{
    FDCID_INS_1 = 0,           //!< DID_INS_1: full INS-1 data in 64 bytes
    FDCID_INS_2,                //!< DID_INS_2: native-precision qn2b quaternion (16 bytes)
    FDCID_INS_3,                //!< DID_INS_3: MSL altitude (4 bytes)
    FDCID_INS_4,                //!< DID_INS_4: qe2b + ve + ecef (52 bytes in 64-byte frame)
    FDCID_PIMU,                 //!< DID_PIMU: theta + vel + dt + status (32 bytes)
    FDCID_IMU,                  //!< DID_IMU: pqr + acc + status (28 bytes in 32-byte frame)
    FDCID_GNSS1_POS,            //!< DID_GNSS1_POS: status + cnoMean (8 bytes)
    FDCID_GNSS2_POS,            //!< DID_GNSS2_POS: status + cnoMean (8 bytes)
    FDCID_GNSS1_RTK_POS_REL,   //!< DID_GNSS1_RTK_POS_REL: native float (16 bytes)
    FDCID_GNSS2_RTK_CMP_REL,   //!< DID_GNSS2_RTK_CMP_REL: native float (16 bytes)
    NUM_FDCIDS                  //!< Number of CAN FD message IDs
};

/** @brief Realtime Message Controller Internal (RMCI) - IMX-side runtime state that drives the RMC broadcast engine. Combines the live enable bits/options (rmc), a per-DID period multiplier table used to throttle binary DID broadcast rate, and the parallel NMEA sentence enable/period state (rmcNmea). Not part of the wire protocol itself; DID_RMC only exchanges the rmc_t bits/options. */
typedef struct PACKED
{
    rmc_t                   rmc;                    //!< Data stream enable bits and options for the specified ports (see RMC_BITS_...)

    uint8_t                 periodMultiple[DID_COUNT]; //!< Per-DID output period multiplier, indexed by eDataIDs; used for both DID binary and NMEA messages

    rmcNmea_t               rmcNmea;                //!< NMEA sentence broadcast enable bits and period multipliers

} rmci_t;

// GPX Realtime Message Controller (GRMC) - message broadcast mechanism.
#define GRMC_OPTIONS_PORT_MASK      0x000000FF
#define GRMC_OPTIONS_PORT_ALL       (RMC_OPTIONS_PORT_MASK)
#define GRMC_OPTIONS_PORT_CURRENT   0x00000000
#define GRMC_OPTIONS_PORT_SER0      0x00000001
#define GRMC_OPTIONS_PORT_SER1      0x00000002                  // also SPI
#define GRMC_OPTIONS_PORT_SER2      0x00000004
#define GRMC_OPTIONS_PORT_USB       0x00000008
#define GRMC_OPTIONS_PRESERVE_CTRL  0x00000100                  // Prevent any messages from getting turned off by bitwise OR'ing new message bits with current message bits.
#define GRMC_OPTIONS_PERSISTENT     0x00000200                  // Save current port RMC to flash memory for use following reboot, eliminating need to re-enable RMC to start data streaming.


/** @brief Bit position (not mask) for each data set in the GPX Realtime Message Controller (GRMC) broadcast-enable word. Use GRMC_BITS_* (below) for the corresponding (1<<pos) mask; GRMC_BIT_POS_COUNT is also used to size grmci_t.periodMultiple. */
enum GRMC_BIT_POS{
    GRMC_BIT_POS_DEV_INFO           = 0,   //!< DID_DEV_INFO - device information
    GRMC_BIT_POS_FLASH_CFG          = 1,   //!< DID_FLASH_CONFIG - flash configuration
    GRMC_BIT_POS_STATUS             = 2,   //!< DID_GPX_STATUS - GPX status
    GRMC_BIT_POS_RTOS_INFO          = 3,   //!< DID_GPX_RTOS_INFO - RTOS task/heap info
    GRMC_BIT_POS_DEBUG_ARRAY        = 4,   //!< DID_GPX_DEBUG_ARRAY - internal debug values
    GRMC_BIT_POS_GNSS1_POS          = 5,   //!< DID_GNSS1_POS - GNSS 1 position
    GRMC_BIT_POS_GNSS1_VEL          = 6,   //!< DID_GNSS1_VEL - GNSS 1 velocity
    GRMC_BIT_POS_GNSS1_SAT          = 7,   //!< DID_GNSS1_SAT - GNSS 1 satellite info
    GRMC_BIT_POS_GNSS1_SIG          = 8,   //!< DID_GNSS1_SIG - GNSS 1 signal info
    GRMC_BIT_POS_GNSS1_RAW          = 9,   //!< DID_GNSS1_RAW - GNSS 1 raw observation/ephemeris data
    GRMC_BIT_POS_GNSS1_VERSION      = 10,  //!< DID_GNSS1_VERSION - GNSS 1 receiver version
    GRMC_BIT_POS_GNSS2_POS          = 11,  //!< DID_GNSS2_POS - GNSS 2 position
    GRMC_BIT_POS_GNSS2_VEL          = 12,  //!< DID_GNSS2_VEL - GNSS 2 velocity
    GRMC_BIT_POS_GNSS2_SAT          = 13,  //!< DID_GNSS2_SAT - GNSS 2 satellite info
    GRMC_BIT_POS_GNSS2_SIG          = 14,  //!< DID_GNSS2_SIG - GNSS 2 signal info
    GRMC_BIT_POS_GNSS2_RAW          = 15,  //!< DID_GNSS2_RAW - GNSS 2 raw observation/ephemeris data
    GRMC_BIT_POS_GNSS2_VERSION      = 16,  //!< DID_GNSS2_VERSION - GNSS 2 receiver version
    GRMC_BIT_POS_GNSS1_RTK_POS      = 17,  //!< DID_GNSS1_RTK_POS - GNSS 1 RTK position
    GRMC_BIT_POS_GNSS1_RTK_POS_MISC = 18,  //!< DID_GNSS1_RTK_POS_MISC - GNSS 1 RTK position misc/diagnostic info
    GRMC_BIT_POS_GNSS1_RTK_POS_REL  = 19,  //!< DID_GNSS1_RTK_POS_REL - GNSS 1 RTK relative position (baseline to base)
    GRMC_BIT_POS_GNSS2_RTK_CMP_MISC = 20,  //!< DID_GNSS2_RTK_CMP_MISC - GNSS 2 RTK compassing misc/diagnostic info
    GRMC_BIT_POS_GNSS2_RTK_CMP_REL  = 21,  //!< DID_GNSS2_RTK_CMP_REL - GNSS 2 RTK compassing relative position/heading
    GRMC_BIT_POS_DID_RTK_DEBUG      = 22,  //!< DID_RTK_DEBUG - RTK solver debug info
    GRMC_BIT_POS_DID_PORT_MON       = 23,  //!< DID_PORT_MONITOR - serial port monitor/statistics
    GRMC_BIT_POS_DID_GPX_PORT_MON   = 24,  //!< DID_GPX_PORT_MONITOR - GPX serial port monitor/statistics
    GRMC_BIT_POS_DID_GNSS_BASE_RAW  = 25,  //!< DID_GNSS_BASE_RAW - raw observation data forwarded from the RTK base station
    GRMC_BIT_POS_DID_GPX_SYS_FAULT  = 26,  //!< DID_GPX_SYS_FAULT - GPX system fault/exception info
    GRMC_BIT_POS_GNSS1_RCVR_POS     = 27,  //!< DID_GNSS1_RCVR_POS - GNSS 1 receiver-reported position
    GRMC_BIT_POS_COUNT,                    //!< Number of GRMC bit positions; sizes grmci_t.periodMultiple
};

#define GRMC_BITS_DEV_INFO              (0x0000000000000001 << GRMC_BIT_POS_DEV_INFO)
#define GRMC_BITS_FLASH_CFG             (0x0000000000000001 << GRMC_BIT_POS_FLASH_CFG)
#define GRMC_BITS_STATUS                (0x0000000000000001 << GRMC_BIT_POS_STATUS)
#define GRMC_BITS_RTOS_INFO             (0x0000000000000001 << GRMC_BIT_POS_RTOS_INFO)
#define GRMC_BITS_DEBUG_ARRAY           (0x0000000000000001 << GRMC_BIT_POS_DEBUG_ARRAY)
#define GRMC_BITS_GNSS1_POS             (0x0000000000000001 << GRMC_BIT_POS_GNSS1_POS)
#define GRMC_BITS_GNSS1_VEL             (0x0000000000000001 << GRMC_BIT_POS_GNSS1_VEL)
#define GRMC_BITS_GNSS1_SAT             (0x0000000000000001 << GRMC_BIT_POS_GNSS1_SAT)
#define GRMC_BITS_GNSS1_SIG             (0x0000000000000001 << GRMC_BIT_POS_GNSS1_SIG)
#define GRMC_BITS_GNSS1_RAW             (0x0000000000000001 << GRMC_BIT_POS_GNSS1_RAW)
#define GRMC_BITS_GNSS1_VERSION         (0x0000000000000001 << GRMC_BIT_POS_GNSS1_VERSION)
#define GRMC_BITS_GNSS2_POS             (0x0000000000000001 << GRMC_BIT_POS_GNSS2_POS)
#define GRMC_BITS_GNSS2_VEL             (0x0000000000000001 << GRMC_BIT_POS_GNSS2_VEL)
#define GRMC_BITS_GNSS2_SAT             (0x0000000000000001 << GRMC_BIT_POS_GNSS2_SAT)
#define GRMC_BITS_GNSS2_SIG             (0x0000000000000001 << GRMC_BIT_POS_GNSS2_SIG)
#define GRMC_BITS_GNSS2_RAW             (0x0000000000000001 << GRMC_BIT_POS_GNSS2_RAW)
#define GRMC_BITS_GNSS2_VERSION         (0x0000000000000001 << GRMC_BIT_POS_GNSS2_VERSION)
#define GRMC_BITS_GNSS1_RTK_POS         (0x0000000000000001 << GRMC_BIT_POS_GNSS1_RTK_POS)
#define GRMC_BITS_GNSS1_RTK_POS_MISC    (0x0000000000000001 << GRMC_BIT_POS_GNSS1_RTK_POS_MISC)
#define GRMC_BITS_GNSS1_RTK_POS_REL     (0x0000000000000001 << GRMC_BIT_POS_GNSS1_RTK_POS_REL)
#define GRMC_BITS_GNSS2_RTK_CMP_MISC    (0x0000000000000001 << GRMC_BIT_POS_GNSS2_RTK_CMP_MISC)
#define GRMC_BITS_GNSS2_RTK_CMP_REL     (0x0000000000000001 << GRMC_BIT_POS_GNSS2_RTK_CMP_REL)
#define GRMC_BITS_DID_RTK_DEBUG         (0x0000000000000001 << GRMC_BIT_POS_DID_RTK_DEBUG)
#define GRMC_BITS_PORT_MON              (0x0000000000000001 << GRMC_BIT_POS_DID_PORT_MON)
#define GRMC_BITS_GPX_PORT_MON          (0x0000000000000001 << GRMC_BIT_POS_DID_GPX_PORT_MON)
#define GRMC_BITS_GNSS_BASE_RAW         (0x0000000000000001 << GRMC_BIT_POS_DID_GNSS_BASE_RAW)
#define GRMC_BITS_GPX_SYS_FAULT         (0x0000000000000001 << GRMC_BIT_POS_DID_GPX_SYS_FAULT)
#define GRMC_BITS_GNSS1_RCVR_POS        (0x0000000000000001 << GRMC_BIT_POS_GNSS1_RCVR_POS)
#define GRMC_BITS_PRESET                (0x8000000000000000)                                        // Indicate BITS is a preset.  This sets the rmc period multiple and enables broadcasting.

#define GRMC_PRESET_DID_RTK_DEBUG_PERIOD_MS     1000
#define GRMC_PRESET_GPX_DEV_INFO_PERIOD_MS      1000
#define GRMC_PRESET_GPX_GNSS1_VERSION_PERIOD_MS 1000
#define GRMC_PRESET_GPX_GNSS2_VERSION_PERIOD_MS 1000
#define GRMC_PRESET_GPX_RTOS_INFO_PERIOD_MS     500
#define GRMC_PRESET_GPX_STATUS_PERIOD_MS        500
#define GRMC_PRESET_GPX_DEBUG_ARRAY_PERIOD_MS   500
#define GRMC_PRESET_GPX_PORT_MON_PERIOD_MS      500
#define GRMC_PRESET_GPX_SYS_FAULT_PERIOD_MS     30000

#define GRMC_PRESET_GPX_BASE            (GRMC_BITS_PRESET \
                                        /*| GRMC_BITS_DEV_INFO*/ \
                                        /*| GRMC_BITS_RTOS_INFO*/ \
                                        | GRMC_BITS_STATUS \
                                        /*| GRMC_BITS_DEBUG_ARRAY*/ \
                                        | GRMC_BITS_GPX_SYS_FAULT)

#define GRMC_PRESET_GPX_GNSS1   (GRMC_BITS_GNSS1_POS \
                                        | GRMC_BITS_GNSS1_VEL \
                                        | GRMC_BITS_GNSS1_SAT \
                                        | GRMC_BITS_GNSS1_SIG \
                                        | GRMC_BITS_GNSS1_VERSION \
                                        /*| GRMC_BITS_GNSS1_RTK_POS*/ \
                                        | GRMC_BITS_GNSS1_RAW)

#define GRMC_PRESET_GPX_GNSS2   (GRMC_BITS_GNSS2_POS \
                                        | GRMC_BITS_GNSS2_VEL \
                                        | GRMC_BITS_GNSS2_SAT \
                                        | GRMC_BITS_GNSS2_SIG \
                                        | GRMC_BITS_GNSS2_VERSION \
                                        | GRMC_BITS_GNSS2_RAW)

#define GRMC_PRESET_GPX_IMX             ( GRMC_PRESET_GPX_BASE\
                                        | GRMC_PRESET_GPX_GNSS1 \
                                        | GRMC_PRESET_GPX_GNSS2 \
                                        | GRMC_BITS_GNSS2_RTK_CMP_REL \
                                        | GRMC_BITS_GNSS2_RTK_CMP_MISC)

#define GRMC_PRESET_GPX_IMX_RTK_DBG     (GRMC_PRESET_GPX_IMX | GRMC_BITS_DID_RTK_DEBUG)


/** @brief GPX Realtime Message Controller Internal (GRMCI) - GPX-side counterpart to rmci_t. Combines the live enable bits/options (rmc), a per-data-set period table indexed by GRMC_BIT_POS, and the NMEA sentence enable/period state (rmcNmea). Not part of the wire protocol itself. */
typedef struct PACKED
{
    rmc_t       rmc;                           //!< Data stream enable bits and options for the specified ports (see RMC_BITS_.../GRMC_BITS_...)

    uint16_t    periodMultiple[GRMC_BIT_POS_COUNT]; //!< (ms) Period of the message to be sent, indexed by GRMC_BIT_POS

    rmcNmea_t   rmcNmea;                       //!< NMEA data stream enable bits and period multipliers for the specified ports (see NMEA_RMC_BITS_...)
} grmci_t;

/** @brief Magnetometer calibration command/status codes, used for mag_cal_t.state (DID_MAG_CAL). Values below 100 are commands written by the host to start/stop a calibration; values 100+ are abort/status codes reported back by the device. */
enum eMagCalState
{
    MAG_CAL_STATE_DO_NOTHING                = (int)0,    //!< No command / idle, no calibration in progress

    MAG_CAL_STATE_MULTI_AXIS                = (int)1,    //!< COMMAND: Recalibrate magnetometers using multiple axis

    MAG_CAL_STATE_SINGLE_AXIS               = (int)2,    //!< COMMAND: Recalibrate magnetometers using only one axis

    MAG_CAL_STATE_ABORT                     = (int)101,  //!< COMMAND: Stop mag recalibration and do not save results

    MAG_CAL_STATE_RECAL_RUNNING             = (int)200,  //!< STATUS: Mag recalibration is in progress

    MAG_CAL_STATE_RECAL_COMPLETE            = (int)201,  //!< STATUS: Mag recalibration has completed

    MAG_CAL_STATE_RECAL_MODE_NOT_SUPPORTED  = (int)202,  //!< STATUS: Mag recalibration mode not supported
};

/** @brief (DID_MAG_CAL) Magnetometer Calibration command/status. Write state with a MAG_CAL_STATE_* command to start/stop a recalibration; read back state/progress/declination to monitor it. */
typedef struct PACKED
{
    uint32_t                    state;          //!< Mag recalibration command/status.  COMMANDS: 1=multi-axis, 2=single-axis, 101=abort, STATUS: 200=running, 201=done (see eMagCalState)

    float                       progress;       //!< (%) Mag recalibration progress indicator: 0-100

    float                       declination;    //!< (rad) Magnetic declination estimate
} mag_cal_t;

/** @brief (DID_INL2_MAG_OBS_INFO) INL2 magnetometer observer diagnostic/calibration info. Reports the internal state of the online magnetometer bias/cross-axis calibration and heading-innovation gating used by the INL2 EKF to accept or reject magnetometer updates. */
typedef struct PACKED
{
    uint32_t                    timeOfWeekMs;   //!< (ms) GPS time of week

    uint32_t                    Ncal_samples;   //!< Number of calibration samples collected

    uint32_t                    ready;          //!< Data ready to be processed

    uint32_t                    calibrated;     //!< Calibration data present.  Set to -1 to force mag recalibration.

    uint32_t                    auto_recal;     //!< Allow mag to auto-recalibrate

    uint32_t                    outlier;        //!< Bad sample data detected/rejected

    float                       magHdg;         //!< (rad) Heading from magnetometer

    float                       insHdg;         //!< (rad) Heading from INS

    float                       magInsHdgDelta; //!< (rad) Difference between mag heading and (INS heading plus mag declination)

    float                       nis;            //!< Normalized innovation squared (likelihood metric) of the current mag heading update

    float                       nis_threshold;  //!< Threshold for maximum NIS, above which the mag update is rejected as an outlier

    float                       Wcal[9];        //!< Magnetometer calibration matrix (row-major 3x3). Must be initialized with a unit matrix, not zeros!

    uint32_t                    activeCalSet;   //!< Active calibration set (0 or 1)

    float                       magHdgOffset;   //!< (rad) Offset between magnetometer heading and estimated heading

    float                       Tcal;           //!< Scaled computed variance between calibrated magnetometer samples

    float                       bias_cal[3];    //!< (uT) Magnetometer calibration bias. Calibrated magnetometer output can be produced using: Bcal = Wcal * (Braw - bias_cal)
} inl2_mag_obs_info_t;

/** @brief Built-in Test (BIT): input command codes written to bit_t.command (DID_BIT) to select which self-test the IMX runs. BIT_CMD_BASIC_MOVING runs automatically after bootup; the others must be requested by the host. */
enum eBitCommand
{
    BIT_CMD_NONE                            = (int)0,  //!< No command
    BIT_CMD_OFF                             = (int)1,  //!< Stop built-in test
    BIT_CMD_FULL_STATIONARY                 = (int)2,  //!< (FULL) Comprehensive test.  Requires system be completely stationary without vibrations.
    BIT_CMD_BASIC_MOVING                    = (int)3,  //!< (BASIC) Ignores sensor output.  Can be run while moving.  This mode is automatically run after bootup.
    BIT_CMD_FULL_STATIONARY_HIGH_ACCURACY   = (int)4,  //!< Same as BIT_CMD_FULL_STATIONARY but with higher requirements for accuracy.  In order to pass, this test may require the Infield Calibration (DID_INFIELD_CAL) to be run.
    BIT_CMD_RESERVED_2                      = (int)5,  //!< Reserved, unused
    BIT_CMD_IMU_REJECT                      = (int)6,  //!< IMU fault rejection test
    BIT_CMD_IMU_REJECT_CONTINUOUS           = (int)7,  //!< Continuous IMU fault rejection test without ending
    BIT_CMD_IMU_INVALID_DATA                = (int)8,  //!< IMU invalid data test
    BIT_CMD_IMU_SATURATED_SENSOR            = (int)9,  //!< IMU saturated sensor test
};

/** @brief Built-in Test (BIT): run-state of bit_t.state (DID_BIT). Transitions OFF -> RUNNING when a command is accepted, RUNNING -> FINISHING while results are computed, then FINISHING -> DONE once results (hdwBitStatus/calBitStatus) are valid. */
enum eBitState
{
    BIT_STATE_OFF       = (int)0,  //!< No test running
    BIT_STATE_DONE      = (int)1,  //!< Test is finished
    BIT_STATE_RUNNING   = (int)6,  //!< Test is actively sampling/running
    BIT_STATE_FINISHING = (int)7,  //!< Computing results
};

/** @brief Built-in Test (BIT): self-test/simulation mode reported in bit_t.testMode. Values below 100 indicate a BIT_CMD_IMU_* test outcome; values 100+ select or report a fault-injection/communications simulation used to verify fault detection and recovery paths. */
enum eBitTestMode
{
    BIT_TEST_MODE_FAILED                    = (int)98,  //!< Test mode ran and failed
    BIT_TEST_MODE_DONE                      = (int)99,  //!< Test mode ran and completed
    BIT_TEST_MODE_SIM_GNSS_NOISE            = (int)100,  //!< Simulate CNO noise
    BIT_TEST_MODE_COMMUNICATIONS_REPEAT     = (int)101,  //!< Send duplicate message
    BIT_TEST_MODE_SERIAL_DRIVER_RX_OVERFLOW = (int)102,  //!< Cause Rx buffer overflow on current serial port by blocking date read until the overflow occurs.
    BIT_TEST_MODE_SERIAL_DRIVER_TX_OVERFLOW = (int)103,  //!< Cause Tx buffer overflow on current serial port by sending too much data.
    BIT_TEST_MODE_IMU_FAULT_REJECTION       = (int)104,  //!< Simulate a fault on each IMU sensor and ensure it is detected and rejected.
    BIT_TEST_MODE_IMU_INVALID_DATA          = (int)105,  //!< Simulate invalid IMU data (NaN) and ensure it is detected and rejected.
    BIT_TEST_MODE_IMU_SATURATION_DATA       = (int)106,  //!< Simulate saturated IMU data (out-of-range) and ensure it is detected and rejected.
};

/** @brief Hardware built-in test (BIT) result flags, reported in bit_t.hdwBitStatus (DID_BIT). Low nibble (HDW_BIT_PASSED_MASK) reports pass state; bits 4-7 (HDW_BIT_MODE_MASK, decoded with HDW_BIT_MODE()) echo which eBitTestMode/eBitCommand test produced the result; bits 8+ (HDW_BIT_FAILED_MASK) are individual sensor/interface fault flags, any of which indicate the overall BIT failed. */
enum eHdwBitStatusFlags
{
    HDW_BIT_PASSED_MASK                     = (int)0x0000000F,  //!< Mask for the pass/fail result bits
    HDW_BIT_PASSED_ALL                      = (int)0x00000001,  //!< All hardware BIT checks passed
    HDW_BIT_PASSED_NO_GNSS                  = (int)0x00000002,  //!< Passed w/o valid GNSS signal
    HDW_BIT_MODE_MASK                       = (int)0x000000F0,  //!< Mask for the BIT mode run, see HDW_BIT_MODE()
    HDW_BIT_MODE_OFFSET                     = (int)4,           //!< Bit offset of HDW_BIT_MODE_MASK
#define HDW_BIT_MODE(hdwBitStatus)          (((hdwBitStatus)&HDW_BIT_MODE_MASK)>>HDW_BIT_MODE_OFFSET)
    HDW_BIT_FAILED_MASK                     = (int)0xFFFFFF00,  //!< Mask for any hardware BIT failure flag
    HDW_BIT_FAILED_AHRS_MASK                = (int)0xFFFF0F00,  //!< Mask for AHRS-relevant failure flags (excludes GNSS-only bits)
    HDW_BIT_FAULT_NOISE_PQR                 = (int)0x00000100,  //!< Excessive noise detected on gyros
    HDW_BIT_FAULT_NOISE_ACC                 = (int)0x00000200,  //!< Excessive noise detected on accelerometers
    HDW_BIT_FAULT_MAGNETOMETER              = (int)0x00000400,  //!< Magnetometer fault detected
    HDW_BIT_FAULT_BAROMETER                 = (int)0x00000800,  //!< Barometer fault detected
    HDW_BIT_FAULT_GNSS_NO_COM               = (int)0x00001000,  //!< No GNSS serial communications
    HDW_BIT_FAULT_GNSS_POOR_CNO             = (int)0x00002000,  //!< Poor GNSS signal strength.  Check antenna
    HDW_BIT_FAULT_GNSS_POOR_ACCURACY        = (int)0x00004000,  //!< Low number of satellites, or bad accuracy
    HDW_BIT_FAULT_GNSS_NOISE                = (int)0x00008000,  //!< (Not implemented)
    HDW_BIT_FAULT_IMU_FAULT_REJECTION       = (int)0x00010000,  //!< IMU fault rejection failure
    HDW_BIT_FAULT_INCORRECT_HARDWARE_TYPE   = (int)0x01000000,  //!< Hardware type does not match firmware
    HDW_BIT_FAULT_EKF_NOT_INITIALIZED       = (int)0x02000000,  //!< EKF not initialized
};

/** @brief Calibration built-in test (BIT) result flags, reported in bit_t.calBitStatus (DID_BIT). Low nibble (CAL_BIT_PASSED_MASK) reports pass state; bits 4-7 (CAL_BIT_MODE_MASK, decoded with CAL_BIT_MODE()) echo the test mode run; CAL_BIT_FAILED_MASK bits report temperature/motion calibration faults (test fails); bits 0x01000000+ are advisory notices (bias offset detected) that do not by themselves fail the test. */
enum eCalBitStatusFlags
{
    CAL_BIT_PASSED_MASK             = (int)0x0000000F,  //!< Mask for the pass/fail result bits
    CAL_BIT_PASSED_ALL              = (int)0x00000001,  //!< All calibration BIT checks passed
    CAL_BIT_MODE_MASK               = (int)0x000000F0,  //!< Mask for the BIT mode run, see CAL_BIT_MODE()
    CAL_BIT_MODE_OFFSET             = (int)4,           //!< Bit offset of CAL_BIT_MODE_MASK
#define CAL_BIT_MODE(calBitStatus)          (((calBitStatus)&CAL_BIT_MODE_MASK)>>CAL_BIT_MODE_OFFSET)
    CAL_BIT_FAILED_MASK             = (int)0x00FFFF00,  //!< Mask for any calibration BIT failure flag
    CAL_BIT_FAULT_TCAL_EMPTY        = (int)0x00000100,  //!< Temperature calibration not present
    CAL_BIT_FAULT_TCAL_TSPAN        = (int)0x00000200,  //!< Temperature calibration temperature range is inadequate
    CAL_BIT_FAULT_TCAL_INCONSISTENT = (int)0x00000400,  //!< Temperature calibration number of points or slopes are not consistent
    CAL_BIT_FAULT_TCAL_CORRUPT      = (int)0x00000800,  //!< Temperature calibration memory corruption
    CAL_BIT_FAULT_TCAL_PQR_BIAS     = (int)0x00001000,  //!< Temperature calibration gyro bias
    CAL_BIT_FAULT_TCAL_PQR_SLOPE    = (int)0x00002000,  //!< Temperature calibration gyro slope
    CAL_BIT_FAULT_TCAL_PQR_LIN      = (int)0x00004000,  //!< Temperature calibration gyro linearity
    CAL_BIT_FAULT_TCAL_ACC_BIAS     = (int)0x00008000,  //!< Temperature calibration accelerometer bias
    CAL_BIT_FAULT_TCAL_ACC_SLOPE    = (int)0x00010000,  //!< Temperature calibration accelerometer slope
    CAL_BIT_FAULT_TCAL_ACC_LIN      = (int)0x00020000,  //!< Temperature calibration accelerometer linearity
    CAL_BIT_FAULT_CAL_SERIAL_NUM    = (int)0x00040000,  //!< Calibration info: wrong device serial number
    CAL_BIT_FAULT_MCAL_MAG_INVALID  = (int)0x00080000,  //!< Motion calibration MAG Cross-axis alignment is poorly formed
    CAL_BIT_FAULT_MCAL_EMPTY        = (int)0x00100000,  //!< Motion calibration Cross-axis alignment is not calibrated
    CAL_BIT_FAULT_MCAL_IMU_INVALID  = (int)0x00200000,  //!< Motion calibration IMU Cross-axis alignment is poorly formed
    CAL_BIT_FAULT_MOTION_PQR        = (int)0x00400000,  //!< Motion on gyros
    CAL_BIT_FAULT_MOTION_ACC        = (int)0x00800000,  //!< Motion on accelerometers
    CAL_BIT_NOTICE_IMU1_PQR_BIAS    = (int)0x01000000,  //!< IMU 1 gyro bias offset detected.  If stationary, zero gyros command may be used.
    CAL_BIT_NOTICE_IMU2_PQR_BIAS    = (int)0x02000000,  //!< IMU 2 gyro bias offset detected.  If stationary, zero gyros command may be used.
    CAL_BIT_NOTICE_IMU1_ACC_BIAS    = (int)0x10000000,  //!< IMU 1 accelerometer bias offset detected.  If stationary, zero accelerometer command may be used only on the vertical access.
    CAL_BIT_NOTICE_IMU2_ACC_BIAS    = (int)0x20000000,  //!< IMU 2 accelerometer bias offset detected.  If stationary, zero accelerometer command may be used only on the vertical access.
};


/** @brief (DID_BIT) Built-in self-test (BIT) command/status for the IMX. Write "command" with an eBitCommand value to start/stop a test; "state" (eBitState) reports progress OFF -> RUNNING -> FINISHING -> DONE; once DONE, hdwBitStatus/calBitStatus hold the pass/fail/fault results. */
typedef struct PACKED
{
    uint8_t                 command;            //!< BIT input command (see eBitCommand).  Ignored when zero.

    uint8_t                 lastCommand;        //!< BIT last input command (see eBitCommand)

    uint8_t                 state;              //!< BIT current state (see eBitState)

    uint8_t                 reserved;           //!< Unused

    uint32_t                hdwBitStatus;       //!< Hardware BIT status (see eHdwBitStatusFlags)

    uint32_t                calBitStatus;       //!< Calibration BIT status (see eCalBitStatusFlags)

    float                   tcPqrBias;          //!< (rad/s) Gyro bias residual from temperature calibration
    float                   tcAccBias;          //!< (m/s^2) Accelerometer bias residual from temperature calibration

    float                   tcPqrSlope;         //!< (rad/s per deg C) Gyro temperature-compensation slope error
    float                   tcAccSlope;         //!< (m/s^2 per deg C) Accelerometer temperature-compensation slope error

    float                   tcPqrLinearity;     //!< (rad/s) Gyro temperature-compensation curve-fit linearity error
    float                   tcAccLinearity;     //!< (m/s^2) Accelerometer temperature-compensation curve-fit linearity error

    float                   pqr;                //!< (rad/s) Gyro angular rate error measured during BIT

    float                   acc;                //!< (m/s^2) Accelerometer error measured during BIT

    float                   pqrSigma;           //!< (rad/s) Standard deviation of the gyro angular rate error, over the BIT sample window

    float                   accSigma;           //!< (m/s^2) Standard deviation of the accelerometer error, over the BIT sample window

    uint8_t                 testMode;           //!< Self-test/fault-simulation mode (see eBitTestMode)

    uint8_t                 testVar;            //!< Self-test mode bi-directional variable used with testMode

    uint16_t                detectedHardwareId; //!< Detected hardware type (see "Product Hardware ID"), used to ensure correct firmware is used

} bit_t;

// GPX Built-in Test (GPX-BIT)
/** @brief Bit position (not mask) for each GPX-BIT sub-test/status in the eGPXBit_results word (gpx_bit_t.results, DID_GPX_BIT). Use eGPXBit_results (below) for the corresponding (1<<pos) mask. */
enum eGPXBit_resultsPos{
    GPXBit_resultsPos_PPS1  = 0,   //!< Bit position for the PPS1 (pulse-per-second, GNSS 1) test result
    GPXBit_resultsPos_PPS2,        //!< Bit position for the PPS2 (pulse-per-second, GNSS 2) test result
    GPXBit_resultsPos_UART,        //!< Bit position for the UART communications test result
    GPXBit_resultsPos_IO,          //!< Bit position for the I/O test result
    GPXBit_resultsPos_GNSS,        //!< Bit position for the GNSS test result

    GPXBit_resultsPos_FINISHED,    //!< Bit position indicating the GPX-BIT test has finished

    GPXBit_resultsPos_CANCELED,    //!< Bit position indicating the GPX-BIT test was canceled
    GPXBit_resultsPos_ERROR,       //!< Bit position indicating the GPX-BIT test ended in error
};

/** @brief GPX-BIT result flags, reported in gpx_bit_t.results (DID_GPX_BIT). Each PPS1/PPS2/UART/IO/GNSS bit is set when that sub-test passes; FINISHED/CANCELED/ERROR report the overall test's terminal status (see GPXBit_resultMasks_PASSED for the "test passed" mask). */
enum eGPXBit_results{
    GPXBit_resultsBit_PPS1      = (0x01 << GPXBit_resultsPos_PPS1),      //!< PPS1 (GNSS 1 pulse-per-second) test passed
    GPXBit_resultsBit_PPS2      = (0x01 << GPXBit_resultsPos_PPS2),      //!< PPS2 (GNSS 2 pulse-per-second) test passed
    GPXBit_resultsBit_UART      = (0x01 << GPXBit_resultsPos_UART),      //!< UART communications test passed
    GPXBit_resultsBit_IO        = (0x01 << GPXBit_resultsPos_IO),        //!< I/O test passed
    GPXBit_resultsBit_GNSS      = (0x01 << GPXBit_resultsPos_GNSS),      //!< GNSS test passed
    GPXBit_resultsBit_FINISHED  = (0x01 << GPXBit_resultsPos_FINISHED),  //!< Test has finished running
    GPXBit_resultsBit_CANCELED  = (0x01 << GPXBit_resultsPos_CANCELED),  //!< Test was canceled before completion
    GPXBit_resultsBit_ERROR     = (0x01 << GPXBit_resultsPos_ERROR),     //!< Test ended in error
};

#define GPXBit_RESULT_GNSS_QT_EXIT_Mask GPXBit_resultsBit_PPS1 | GPXBit_resultsBit_PPS2

/** @brief GPX-BIT input command codes, written to gpx_bit_t.command (DID_GPX_BIT) to start/stop the GPX manufacturing self-test or a specific fault-injection/communications simulation. */
enum eGPXBit_CMD{
    GPXBit_CMD_NONE                                 = 0,   //!< No command
    GPXBit_CMD_START_MANUF_TEST                     = 1,   //!< Start the manufacturing test (PPS1/PPS2/UART/IO/GNSS sub-tests)
    GPXBit_CMD_ALERT_UART_TEST_STR                  = 2,   //!< Alert that the UART test string was received (used by test fixture to signal the UART sub-test)
    GPXBit_CMD_ALERT_PPS1_RX                        = 3,   //!< Alert that a PPS1 pulse was received (used by test fixture to signal the PPS1 sub-test)
    GPXBit_CMD_ALERT_PPS2_RX                        = 4,   //!< Alert that a PPS2 pulse was received (used by test fixture to signal the PPS2 sub-test)
    GPXBit_CMD_REPORT                               = 5,   //!< Request a report of the current test results
    GPXBit_CMD_STOP                                 = 6,   //!< Stop the test

    GPXBit_CMD_START_SIM_GNSS_NOISE                 = 7,   //!< Simulate CNO noise
    GPXBit_CMD_START_COMMUNICATIONS_REPEAT          = 8,   //!< Send duplicate message
    GPXBit_CMD_START_SERIAL_DRIVER_TX_OVERFLOW      = 9,   //!< Cause Tx buffer overflow on current serial port by sending too much data.
    GPXBit_CMD_START_SERIAL_DRIVER_RX_OVERFLOW      = 10,  //!< Cause Rx buffer overflow on current serial port by blocking date read until the overflow occurs.
    GPXBit_CMD_FORCE_SYS_FAULT_WATCHDOG_COMM_TASK   = 11,  //!< Cause watchdog reset by stalling COMM task
    GPXBit_CMD_FORCE_SYS_FAULT_WATCHDOG_RTK_TASK    = 12,  //!< Cause watchdog reset by stalling RTK task
    GPXBit_CMD_FORCE_SYS_FAULT_HARD_FAULT           = 13,  //!< Cause hard fault
    GPXBit_CMD_FORCE_SYS_FAULT_MALLOC               = 14,  //!< Cause malloc failure
};

/** @brief GPX-BIT test/simulation mode, reported in gpx_bit_t.testMode (DID_GPX_BIT). Values below 10 report the manufacturing test outcome; 10 selects the standard manufacturing test; values 100+ select or report a fault-injection/communications simulation used to verify fault detection and recovery paths, mirroring eGPXBit_CMD's simulation commands. */
enum eGPXBit_test_mode{
    GPXBit_test_mode_NONE                           = (int)0,    //!< No test mode active
    GPXBit_test_mode_FAILURE                        = (int)8,    //!< Test mode ran and failed
    GPXBit_test_mode_DONE                           = (int)9,    //!< Test mode ran and completed
    GPXBit_test_mode_MANUFACTURING                  = (int)10,   //!< Standard manufacturing test

    GPXBit_test_mode_SIM_GNSS_NOISE                 = (int)100,  //!< Simulate CNO noise
    GPXBit_test_mode_COMMUNICATIONS_REPEAT          = (int)101,  //!< Send duplicate message
    GPXBit_test_mode_SERIAL_DRIVER_TX_OVERFLOW      = (int)102,  //!< Cause Tx buffer overflow on current serial port by sending too much data.
    GPXBit_test_mode_SERIAL_DRIVER_RX_OVERFLOW      = (int)103,  //!< Cause Rx buffer overflow on current serial port by blocking date read until the overflow occurs.
    GPXBit_test_mode_SYS_FAULT_WATCHDOG_COMM_TASK   = (int)104,  //!< Cause watchdog reset by stalling COMM task
    GPXBit_test_mode_SYS_FAULT_WATCHDOG_RTK_TASK    = (int)105,  //!< Cause watchdog reset by stalling RTK task
};

#define GPXBit_resultMasks_PASSED   (GPXBit_resultsBit_PPS1 | GPXBit_resultsBit_PPS2 | GPXBit_resultsBit_UART | GPXBit_resultsBit_IO | GPXBit_resultsBit_GNSS | GPXBit_resultsBit_FINISHED)

/** @brief (DID_GPX_BIT) GPX built-in self-test (BIT) command/status. Write "command" with an eGPXBit_CMD value to start/stop a test; "results" (eGPXBit_results) reports per-sub-test pass bits plus FINISHED/CANCELED/ERROR terminal status. */
typedef struct PACKED
{
    uint32_t                results;             //!< GPX built-in test status (see eGPXBit_results)

    uint8_t                 command;             //!< Command (see eGPXBit_CMD)

    uint8_t                 port;                //!< Port used with the test

    uint8_t                 testMode;            //!< Self-test mode (see eGPXBit_test_mode)

    uint8_t                 state;               //!< Built-in self-test state (see eGPXBit_state)

    uint16_t                detectedHardwareId;  //!< The hardware ID detected (see "Product Hardware ID").  This is used to ensure correct firmware is used.

    uint8_t                 reserved[2];         //!< Unused

} gpx_bit_t;

/** @brief Infield Calibration (IFC) command/status codes, used for infield_cal_t.state (DID_INFIELD_CAL). The host writes an INIT_* command to select which bias(es) to zero and clear prior samples, then START_SAMPLE for each of the up-to-6 orientations (3 axes x up/down, each requiring a 180-degree delta-yaw pair), then SAVE_AND_FINISH to compute and flash the result. Values 0-11 are host commands; 50-53 are read-only progress status; 100+ are read-only error status reported when a command fails. */
enum eInfieldCalState
{
    /** User Commands: */
    INFIELD_CAL_STATE_CMD_OFF                                   = 0,   //!< Turn off / idle infield calibration

    /** Initialization Commands.  Select one of the following to clear prior samples and set the mode.  Zero accels requires vertical alignment.  No motion is required for all unless disabled.  */
    INFIELD_CAL_STATE_CMD_INIT_ZERO_IMU                         = 1,   //!< Zero accel and gyro biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_GYRO                        = 2,   //!< Zero only gyro  biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ACCEL                       = 3,   //!< Zero only accel biases.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE                    = 4,   //!< Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_IMU                = 5,   //!< Zero gyro and accel biases.  Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_GYRO               = 6,   //!< Zero only gyro  biases.  Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATE_CMD_INIT_ZERO_ATTITUDE_ACCEL              = 7,   //!< Zero only accel biases.  Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATE_CMD_INIT_OPTION_DISABLE_MOTION_DETECT     = 0x00010000,  //!< Bitwise AND this with the above init commands to disable motion detection during sampling (allow for more tolerant sampling).
    INFIELD_CAL_STATE_CMD_INIT_OPTION_DISABLE_REQUIRE_VERTIAL   = 0x00020000,  //!< Bitwise AND this with the above init commands to disable vertical alignment requirement for accelerometer bias calibration (allow for more tolerant sampling).

    /** Sample and End Commands: */
    INFIELD_CAL_STATE_CMD_START_SAMPLE                          = 8,   //!< Initiate 5 second sensor sampling and averaging.  Run for each orientation and 180 degree yaw rotation.
    INFIELD_CAL_STATE_CMD_SAVE_AND_FINISH                       = 9,   //!< Run this command to compute and save results.  Must be run following INFIELD_CAL_STATE_CMD_START_SAMPLE.

    /** Status: (read only) */
    INFIELD_CAL_STATE_READY_FOR_SAMPLING                        = 50,  //!< System has been initialized and is waiting for user to intiate sampling.  User must send a command to exit this state.
    INFIELD_CAL_STATE_SAMPLING                                  = 51,  //!< System is averaging the IMU data.  Minimize all motion and vibration.
    INFIELD_CAL_STATE_RUN_BIT_AND_FINISH                        = 52,  //!< Follow up calibration zero with BIT and copy out IMU biases.
    INFIELD_CAL_STATE_SAVED_AND_FINISHED                        = 53,  //!< Calculations are complete and DID_INFIELD_CAL.imu holds the update IMU biases.  Updates are saved to flash.

    /** Error Status: (read only) */
    INFIELD_CAL_STATE_ERROR_NOT_INITIALIZED                     = 100,  //!< Init command (INFIELD_CAL_STATE_CMD_INIT_...) not set.
    INFIELD_CAL_STATE_ERROR_SAMPLE_ABORT_MOTION_DETECTED        = 101,  //!< Error: Motion detected. Sampling aborted.
    INFIELD_CAL_STATE_ERROR_SAMPLE_ABORT_NOT_VERTICAL           = 102,  //!< Error: System not vertical. Sampling aborted.
    INFIELD_CAL_STATE_ERROR_NO_SAMPLES_COLLECTED                = 103,  //!< Error: No samples have been collected
    INFIELD_CAL_STATE_ERROR_POOR_CAL_FIT                        = 104,  //!< Error: Calibration zero is not well-conditioned / fit did not converge

    /** Internal Use Only */
    INFIELD_CAL_STATE_CMD_MASK                                  = 0x0000FFFF,  //!< Mask isolating the command/status value from the DISABLE_* option bits
    INFIELD_CAL_STATE_CMD_START_SAMPLE_BIT                      = 11,  //!< Initiate 5 second sensor sample and averaging.  Does not save sample into cal data.
};

/** @brief Infield Calibration (IFC) status flags, used for infield_cal_t.status (DID_INFIELD_CAL). Per-axis AXIS_DN_ and AXIS_UP_ bits (packed per-axis at INFIELD_CAL_STATUS_SAMPLE_X/Y/Z_OFFSET, see INFIELD_CAL_STATUS_AXIS_MASK) track which orientation of which axis has been sampled; ENABLED_ bits mirror which biases the active INFIELD_CAL_STATE_CMD_INIT_... command is solving for; the two trailing bits report why a sample was rejected. */
enum eInfieldCalStatus
{
    INFIELD_CAL_STATUS_AXIS_DN_GRAVITY              = 0x00000001,  //!< Axis points in direction of gravity more than any other axis.
    INFIELD_CAL_STATUS_AXIS_DN_SAMPLED              = 0x00000002,  //!< Sampled
    INFIELD_CAL_STATUS_AXIS_DN_SAMPLED_180          = 0x00000004,  //!< Sampled based on average of two orientations with 180 degree delta yaw.
    INFIELD_CAL_STATUS_AXIS_UP_GRAVITY              = 0x00000008,  //!< Axis points in direction of gravity more than any other axis.
    INFIELD_CAL_STATUS_AXIS_UP_SAMPLED              = 0x00000010,  //!< Sampled
    INFIELD_CAL_STATUS_AXIS_UP_SAMPLED_180          = 0x00000020,  //!< Sampled based on average of two orientations with 180 degree delta yaw.

    INFIELD_CAL_STATUS_SAMPLE_X_OFFSET              = 0,           //!< Bit offset of the X-axis AXIS_DN_ and AXIS_UP_ status bits
    INFIELD_CAL_STATUS_SAMPLE_Y_OFFSET              = 6,           //!< Bit offset of the Y-axis AXIS_DN_ and AXIS_UP_ status bits
    INFIELD_CAL_STATUS_SAMPLE_Z_OFFSET              = 12,          //!< Bit offset of the Z-axis AXIS_DN_ and AXIS_UP_ status bits

    INFIELD_CAL_STATUS_AXIS_MASK                    = 0x0000003F,  //!< Mask for one axis' 6-bit AXIS_DN_ and AXIS_UP_ status group
    INFIELD_CAL_STATUS_AXES_GRAVITY_MASK            = (\
        ((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_X_OFFSET) | \
        ((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_Y_OFFSET) | \
        ((INFIELD_CAL_STATUS_AXIS_DN_GRAVITY|INFIELD_CAL_STATUS_AXIS_UP_GRAVITY)<<INFIELD_CAL_STATUS_SAMPLE_Z_OFFSET)),  //!< Mask combining the AXIS_DN_GRAVITY and AXIS_UP_GRAVITY bits across all three axes

    INFIELD_CAL_STATUS_ENABLED_ZERO_ACCEL           = 0x00100000,  //!< Zero accel bias.  Require vertical alignment for sampling.
    INFIELD_CAL_STATUS_ENABLED_ZERO_GYRO            = 0x00200000,  //!< Zero gyro bias.
    INFIELD_CAL_STATUS_ENABLED_ZERO_ATTITUDE        = 0x00400000,  //!< Zero (level) INS attitude by adjusting INS rotation.
    INFIELD_CAL_STATUS_ENABLED_MOTION_DETECT        = 0x00800000,  //!< Require no motion during sampling.
    INFIELD_CAL_STATUS_ENABLED_NORMAL_MASK          = 0x00F00000,  //!< Mask for the normal (non-BIT) ENABLED_ZERO_ and ENABLED_MOTION_DETECT bits
    INFIELD_CAL_STATUS_ENABLED_BIT                  = 0x01000000,  //!< Used for BIT
    INFIELD_CAL_STATUS_DISABLED_REQUIRE_VERTICAL    = 0x02000000,  //!< Do not require vertical alignment for accelerometer calibration.

    INFIELD_CAL_STATUS_AXIS_NOT_VERTICAL            = 0x10000000,  //!< Axis is not aligned vertically and cannot be used for zero accel sampling.
    INFIELD_CAL_STATUS_MOTION_DETECTED              = 0x20000000,  //!< System is not stationary and cannot be used for infield calibration.
};

/** @brief Single-IMU-device accelerometer sample used during infield calibration averaging, embedded as an element of @ref infield_cal_direction_t.dev (DID_INFIELD_CAL). */
typedef struct PACKED
{
    float                   acc[3];  //!< (m/s^2) Vertical axis acceleration, per IMU device, averaged over the sample window
} imu_acc_t;

/** @brief Infield calibration accelerometer sample set for one orientation (system pointed "down" or "up" along a given vertical axis), embedded as the down/up members of @ref infield_cal_vaxis_t (DID_INFIELD_CAL). Populated by an INFIELD_CAL_STATE_CMD_START_SAMPLE command. */
typedef struct PACKED
{
    imu_acc_t               dev[MAX_IMU_DEVICES];  //!< Averaged accelerometer sample, per IMU device, for this orientation

    float                   yaw;                    //!< (rad) Heading of IMU sample.  Used to determine how to average additional samples.  0 = invalid, 999 = averaged
} infield_cal_direction_t;

/** @brief Down/up accelerometer sample pair for one vertical axis of the infield calibration procedure, embedded as an element of @ref infield_cal_t.calData (DID_INFIELD_CAL). Averaging the down and up samples (each optionally itself an average of a 180-degree-yaw pair) isolates that axis' accelerometer bias from gravity. */
typedef struct PACKED
{
    infield_cal_direction_t down;   //!< Pointed toward earth
    infield_cal_direction_t up;     //!< Pointed toward sky
} infield_cal_vaxis_t;

/** @brief (DID_INFIELD_CAL) Measure and correct IMU calibration error; estimate INS rotation to align INS with vehicle. Write "state" with an INFIELD_CAL_STATE_CMD_INIT_... command to select which bias(es) to solve for, run INFIELD_CAL_STATE_CMD_START_SAMPLE for each required orientation (see infield_cal_vaxis_t), then INFIELD_CAL_STATE_CMD_SAVE_AND_FINISH to compute and flash the resulting IMU bias/INS rotation correction. "status" (eInfieldCalStatus) tracks which axes/orientations have been sampled and which corrections are enabled. */
typedef struct PACKED
{
    uint32_t                state;   //!< Used to set and monitor the state of the infield calibration system. (see eInfieldCalState)

    uint32_t                status;  //!< Infield calibration status. (see eInfieldCalStatus)

    uint32_t                sampleTimeMs;  //!< (ms) Number of samples used in IMU average. sampleTimeMs = 0 means "imu" member contains the IMU bias from flash.

    imui_t                  imu[MAX_IMU_DEVICES];  //!< Dual purpose variable.  1.) This is the averaged IMU sample when sampleTimeMs != 0.  2.) This is a mirror of the motion calibration IMU bias from flash when sampleTimeMs = 0.

    infield_cal_vaxis_t     calData[3];  //!< Collected data used to solve for the bias error and INS rotation.  Vertical axis: 0 = X, 1 = Y, 2 = Z

} infield_cal_t;


/** @brief System configuration bitflags, used with nvm_flash_cfg_t.sysCfgBits (DID_FLASH_CONFIG). OR these bits together to disable/enable EKF sensor fusion sources, mag calibration behavior, automatic zero-velocity/zero-rate updates, BIT-on-startup, and brownout reset threshold. Written to flash and applied on the next reset. */
enum eSysConfigBits
{
    UNUSED1                                             = (int)0x00000001,  //!< Unused/reserved bit
    SYS_CFG_BITS_ENABLE_MAG_CONTINUOUS_CAL              = (int)0x00000002,  //!< Enable mag continuous calibration. Allow slow background magnetometer calibration in the EKF.
    SYS_CFG_BITS_AUTO_MAG_RECAL                         = (int)0x00000004,  //!< Enable automatic mag recalibration
    SYS_CFG_BITS_DISABLE_MAG_DECL_ESTIMATION            = (int)0x00000008,  //!< Disable mag declination estimation

    SYS_CFG_BITS_DISABLE_LEDS                           = (int)0x00000010,  //!< Disable LEDs

    SYS_CFG_BITS_MAG_RECAL_MODE_MASK                    = (int)0x00000700,  //!< Magnetometer recalibration mode mask. (see eMagCalState) 1 = multi-axis, 2 = single-axis
    SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET                  = 8,                //!< Bit offset of SYS_CFG_BITS_MAG_RECAL_MODE_MASK within sysCfgBits
#define SYS_CFG_BITS_MAG_RECAL_MODE(sysCfgBits) ((sysCfgBits&SYS_CFG_BITS_MAG_RECAL_MODE_MASK)>>SYS_CFG_BITS_MAG_RECAL_MODE_OFFSET)

    SYS_CFG_BITS_MAG_ENABLE_WMM_DECLINATION             = (int)0x00000800,  //!< When set, World Magnetic Model (WMM) will be used to set mag declination

    SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION            = (int)0x00001000,  //!< Disable magnetometer fusion
    SYS_CFG_BITS_DISABLE_BAROMETER_FUSION               = (int)0x00002000,  //!< Disable barometer fusion
    SYS_CFG_BITS_DISABLE_GNSS1_FUSION                   = (int)0x00004000,  //!< Disable GNSS 1 fusion
    SYS_CFG_BITS_DISABLE_GNSS2_FUSION                   = (int)0x00008000,  //!< Disable GNSS 2 fusion

    SYS_CFG_BITS_DISABLE_AUTO_ZERO_VELOCITY_UPDATES     = (int)0x00010000,  //!< Disable automatic Zero Velocity Updates (ZUPT). Useful for degraded GNSS environments or applications with very slow velocities.
    SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES = (int)0x00020000,  //!< Disable automatic Zero Angular Rate Updates (ZARU). Useful for applications with small/slow angular rates.
    SYS_CFG_BITS_DISABLE_INS_EKF                        = (int)0x00040000,  //!< Disable INS EKF updates
    SYS_CFG_BITS_DISABLE_AUTO_BIT_ON_STARTUP            = (int)0x00080000,  //!< Prevent built-in test (BIT) from running automatically on startup

    SYS_CFG_BITS_DISABLE_WHEEL_ENCODER_FUSION           = (int)0x00100000,  //!< Disable wheel encoder fusion

    SYS_CFG_BITS_ENABLE_GNSS_ANTENNA_OFFSET_ESTIMATION  = (int)0x00200000,  //!< Enable rover GNSS antenna offset estimation in RTK compassing mode

    SYS_CFG_BITS_BOR_LEVEL_0                            = 0x0,  //!< Brownout reset threshold: 1.65 - 1.75 V (default)
    SYS_CFG_BITS_BOR_LEVEL_1                            = 0x1,  //!< Brownout reset threshold: 2.0 - 2.1 V
    SYS_CFG_BITS_BOR_LEVEL_2                            = 0x2,  //!< Brownout reset threshold: 2.25 - 2.35 V
    SYS_CFG_BITS_BOR_LEVEL_3                            = 0x3,  //!< Brownout reset threshold: 2.5 - 2.6 V
    SYS_CFG_BITS_BOR_THRESHOLD_MASK                     = (int)0x00C00000,  //!< Mask for brownout reset threshold voltage level field
    SYS_CFG_BITS_BOR_THRESHOLD_OFFSET                   = 22,               //!< Bit offset of SYS_CFG_BITS_BOR_THRESHOLD_MASK within sysCfgBits

    SYS_CFG_USE_REFERENCE_IMU_IN_EKF                    = (int)0x01000000,  //!< Use reference IMU in EKF instead of onboard IMU
    SYS_CFG_EKF_REF_POINT_STATIONARY_ON_STROBE_INPUT    = (int)0x02000000,  //!< Reference point stationary on strobe input
};

/** @brief GNSS satellite system/signal constellation enable bits for the GPX GNSS receiver, used with nvm_flash_cfg_t.gnssSatSigConst (or GPX-equivalent config). OR together the desired per-constellation, per-signal-band bits to select which satellite systems and frequency bands the receiver tracks; the combined per-constellation bit (e.g. GPX_GNSS_SAT_SIG_CONST_GPS) is a convenience mask covering all of that constellation's bands. */
enum eGpxGnssSatSigConst
{
    GPX_GNSS_SAT_SIG_CONST_GPS_L1   = (uint16_t)0x0001,  //!< GPS L1 signal
    GPX_GNSS_SAT_SIG_CONST_GPS_L5   = (uint16_t)0x0002,  //!< GPS L5 signal
    GPX_GNSS_SAT_SIG_CONST_GPS      = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_GPS_L1 | GPX_GNSS_SAT_SIG_CONST_GPS_L5),  //!< GPS, all supported signals (L1 | L5)
    GPX_GNSS_SAT_SIG_CONST_QZS_L1   = (uint16_t)0x0004,  //!< QZSS L1 signal
    GPX_GNSS_SAT_SIG_CONST_QZS_L5   = (uint16_t)0x0008,  //!< QZSS L5 signal
    GPX_GNSS_SAT_SIG_CONST_QZS      = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_QZS_L1 | GPX_GNSS_SAT_SIG_CONST_QZS_L5),  //!< QZSS, all supported signals (L1 | L5)
    GPX_GNSS_SAT_SIG_CONST_GAL_E1   = (uint16_t)0x0010,  //!< Galileo E1 signal
    GPX_GNSS_SAT_SIG_CONST_GAL_E5   = (uint16_t)0x0020,  //!< Galileo E5 signal
    GPX_GNSS_SAT_SIG_CONST_GAL      = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_GAL_E1 | GPX_GNSS_SAT_SIG_CONST_GAL_E5),  //!< Galileo, all supported signals (E1 | E5)
    GPX_GNSS_SAT_SIG_CONST_BDS_B1   = (uint16_t)0x0040,  //!< BeiDou B1 signal
    GPX_GNSS_SAT_SIG_CONST_BDS_B2   = (uint16_t)0x0080,  //!< BeiDou B2 signal
    GPX_GNSS_SAT_SIG_CONST_BDS      = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_BDS_B1 | GPX_GNSS_SAT_SIG_CONST_BDS_B2),  //!< BeiDou, all supported signals (B1 | B2)
    GPX_GNSS_SAT_SIG_CONST_GLO_L1   = (uint16_t)0x0300,  //!< GLONASS L1 signal
    GPX_GNSS_SAT_SIG_CONST_GLO      = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_GLO_L1),  //!< GLONASS, all supported signals (L1)
    GPX_GNSS_SAT_SIG_CONST_SBS_L1   = (uint16_t)0x1000,  //!< SBAS L1 signal
    GPX_GNSS_SAT_SIG_CONST_SBS      = (uint16_t)(GPX_GNSS_SAT_SIG_CONST_SBS_L1),  //!< SBAS, all supported signals (L1)
    GPX_GNSS_SAT_SIG_CONST_IRN      = (uint16_t)0x2000,  //!< IRNSS / NavIC
    GPX_GNSS_SAT_SIG_CONST_IME      = (uint16_t)0x4000,  //!< IMES
};

/** @brief GNSS satellite system/signal constellation enable bits, used with nvm_flash_cfg_t.gnssSatSigConst (DID_FLASH_CONFIG). OR together the per-constellation bits (each already covers that constellation's tracked signal bands) to select which satellite systems the receiver tracks; GNSS_SAT_SIG_CONST_DEFAULT is the factory selection applied unless overridden. */
enum eGnssSatSigConst
{
    GNSS_SAT_SIG_CONST_GPS              = (uint16_t)0x0003,  //!< GPS, all supported signals
    GNSS_SAT_SIG_CONST_QZS              = (uint16_t)0x000C,  //!< QZSS, all supported signals
    GNSS_SAT_SIG_CONST_GAL              = (uint16_t)0x0030,  //!< Galileo, all supported signals
    GNSS_SAT_SIG_CONST_BDS              = (uint16_t)0x00C0,  //!< BeiDou, all supported signals
    GNSS_SAT_SIG_CONST_GLO              = (uint16_t)0x0300,  //!< GLONASS, all supported signals
    GNSS_SAT_SIG_CONST_SBS              = (uint16_t)0x1000,  //!< SBAS, all supported signals
    GNSS_SAT_SIG_CONST_IRN              = (uint16_t)0x2000,  //!< IRNSS / NavIC
    GNSS_SAT_SIG_CONST_IME              = (uint16_t)0x4000,  //!< IMES

    GNSS_SAT_SIG_CONST_ALL              = \
        GNSS_SAT_SIG_CONST_GPS | \
        GNSS_SAT_SIG_CONST_QZS | \
        GNSS_SAT_SIG_CONST_GAL | \
        GNSS_SAT_SIG_CONST_BDS | \
        GNSS_SAT_SIG_CONST_GLO | \
        GNSS_SAT_SIG_CONST_SBS | \
        GNSS_SAT_SIG_CONST_IRN | \
        GNSS_SAT_SIG_CONST_IME,  //!< Enable every supported constellation

    GNSS_SAT_SIG_CONST_DEFAULT          = \
        GNSS_SAT_SIG_CONST_GPS | \
        GNSS_SAT_SIG_CONST_SBS | \
        GNSS_SAT_SIG_CONST_QZS | \
        GNSS_SAT_SIG_CONST_GAL | \
        GNSS_SAT_SIG_CONST_GLO | \
        GNSS_SAT_SIG_CONST_BDS,  //!< Factory-default constellation selection (excludes IRNSS and IMES)

    GNSS_SAT_SIG_CONST_DEFAULT_INTEL    = \
        GNSS_SAT_SIG_CONST_GPS | \
        GNSS_SAT_SIG_CONST_GAL,  //!< Default constellation selection for Intel-based receiver variant (GPS and Galileo only)
};

/** @brief RTK positioning, compassing, and base-station configuration bitflags, used with nvm_flash_cfg_t.RTKCfgBits (DID_FLASH_CONFIG). Selects rover mode (precision positioning vs. dual-GNSS compassing), and independently configures the device to act as an RTK base station broadcasting ublox/RTCM3 correction data from GNSS1 and/or GNSS2 out any combination of serial ports, USB, or the "current" port. OR the desired bits together; RTK_CFG_BITS_ROVER_MODE_MASK and RTK_CFG_BITS_BASE_MODE isolate the rover-mode and base-mode sub-fields respectively. */
enum eRTKConfigBits
{
    RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_DEPRECATED  = (int)0x00000001,  //!< Enable onboard RTK GNSS precision positioning (GNSS1) DEPRECATED

    RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING             = (int)0x00000002,  //!< Enable external RTK GNSS positioning (GNSS1)

    RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING              = (int)0x00000004,  //!< Enable dual GNSS RTK compassing (GNSS2 to GNSS1)

    RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_DEPRECATED   = (int)0x00000008,  //!< Enable dual GNSS RTK compassing (GNSS2 to GNSS1) DEPRECATED

    RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_MASK        = (RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING | RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_DEPRECATED),  //!< Mask of RTK GNSS positioning types

    RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_MASK         = (RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING | RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_DEPRECATED),  //!< Mask of dual GNSS RTK compassing types

    RTK_CFG_BITS_ROVER_MODE_MASK                        = (int)0x0000000F,  //!< Mask of RTK position, heading, and base modes

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0           = (int)0x00000010,  //!< Enable RTK base and output ublox data from GNSS 1 on serial port 0

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1           = (int)0x00000020,  //!< Enable RTK base and output ublox data from GNSS 1 on serial port 1

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2           = (int)0x00000040,  //!< Enable RTK base and output ublox data from GNSS 1 on serial port 2

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB            = (int)0x00000080,  //!< Enable RTK base and output ublox data from GNSS 1 on USB port

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0           = (int)0x00000100,  //!< Enable RTK base and output RTCM3 data from GNSS 1 on serial port 0

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1           = (int)0x00000200,  //!< Enable RTK base and output RTCM3 data from GNSS 1 on serial port 1

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2           = (int)0x00000400,  //!< Enable RTK base and output RTCM3 data from GNSS 1 on serial port 2

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB            = (int)0x00000800,  //!< Enable RTK base and output RTCM3 data from GNSS 1 on USB port

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0           = (int)0x00001000,  //!< Enable RTK base and output ublox data from GNSS 2 on serial port 0

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1           = (int)0x00002000,  //!< Enable RTK base and output ublox data from GNSS 2 on serial port 1

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2           = (int)0x00004000,  //!< Enable RTK base and output ublox data from GNSS 2 on serial port 2

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB            = (int)0x00008000,  //!< Enable RTK base and output ublox data from GNSS 2 on USB port

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0           = (int)0x00010000,  //!< Enable RTK base and output RTCM3 data from GNSS 2 on serial port 0

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1           = (int)0x00020000,  //!< Enable RTK base and output RTCM3 data from GNSS 2 on serial port 1

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2           = (int)0x00040000,  //!< Enable RTK base and output RTCM3 data from GNSS 2 on serial port 2

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB            = (int)0x00080000,  //!< Enable RTK base and output RTCM3 data from GNSS 2 on USB port

    RTK_CFG_BITS_BASE_POS_MOVING                        = (int)0x00100000,  //!< Enable base mode moving position. (For future use. Not implemented. This bit should always be 0 for now.) TODO: Implement moving base.

    RTK_CFG_BITS_RESERVED1                              = (int)0x00200000,  //!< Reserved for future use

    RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER         = (int)0x00400000,  //!< When using RTK, specifies whether the base station is identical hardware to this rover. If so, optimizations are enabled to get fix faster.

    RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH                 = (int)0x00800000,  //!< Forward all messages between the selected GNSS and serial port. Disable for RTK base use (to forward only GNSS raw messages and use the surveyed location refLLA instead of current GNSS position).

    RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_CUR_PORT       = (int)0x01000000,  //!< Enable RTK base and output RTCM3 data from GNSS 1 on the current serial port

    RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_CUR_PORT       = (int)0x02000000,  //!< Enable RTK base and output RTCM3 data from GNSS 2 on the current serial port

    RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CLEAR_CUR_PORT       = (int)0x04000000,  //!< If this bit is set in conjunction with setting the current port, this clears the current port

    RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CUR_PORT_MASK        = (RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_CUR_PORT | RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_CUR_PORT),  //!< Mask of RTK base and output RTCM3 data on the current serial ports

    RTK_CFG_BITS_BASE_GNSS1_UBLOX_MASK                  = (
            RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB  ),  //!< Mask of all GNSS1 ublox base-output port bits

    RTK_CFG_BITS_BASE_GNSS2_UBLOX_MASK                  = (
            RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB  ),  //!< Mask of all GNSS2 ublox base-output port bits

    RTK_CFG_BITS_BASE_GNSS1_RTCM3_MASK                  = (
            RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB  ),  //!< Mask of all GNSS1 RTCM3 base-output port bits

    RTK_CFG_BITS_BASE_GNSS2_RTCM3_MASK                  = (
            RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2 |
            RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB  ),  //!< Mask of all GNSS2 RTCM3 base-output port bits

    RTK_CFG_BITS_BASE_UBLOX_MASK                        = ( RTK_CFG_BITS_BASE_GNSS1_UBLOX_MASK | RTK_CFG_BITS_BASE_GNSS2_UBLOX_MASK),  //!< Mask of all ublox base-output bits, both GNSS receivers
    RTK_CFG_BITS_BASE_RTCM3_MASK                        = ( RTK_CFG_BITS_BASE_GNSS1_RTCM3_MASK | RTK_CFG_BITS_BASE_GNSS2_RTCM3_MASK),  //!< Mask of all RTCM3 base-output bits, both GNSS receivers

    RTK_CFG_BITS_BASE_MODE                              = ( RTK_CFG_BITS_BASE_UBLOX_MASK | RTK_CFG_BITS_BASE_RTCM3_MASK),  //!< All base station bits

    RTK_CFG_BITS_RTK_BASE_SER0                          = (
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0 | RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0),  //!< Base station bits enabled on Ser0

    RTK_CFG_BITS_RTK_BASE_SER1                          = (
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1 | RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1),  //!< Base station bits enabled on Ser1

    RTK_CFG_BITS_RTK_BASE_SER2                          = (
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2 | RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2 | RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2),  //!< Base station bits enabled on Ser2

    RTK_CFG_BITS_RTK_BASE_OUTPUT_GNSS1_UBLOX            = (
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB),  //!< Base station bits for GNSS1 ublox output, any port

    RTK_CFG_BITS_RTK_BASE_OUTPUT_GNSS2_UBLOX            = (
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB),  //!< Base station bits for GNSS2 ublox output, any port

    RTK_CFG_BITS_RTK_BASE_OUTPUT_GNSS1_RTCM             = (
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB),  //!< Base station bits for GNSS1 RTCM3 output, any port

    RTK_CFG_BITS_RTK_BASE_OUTPUT_GNSS2_RTCM             = (
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2 |
        RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB),  //!< Base station bits for GNSS2 RTCM3 output, any port

    RTK_CFG_BITS_ROVER_MODE_ONBOARD_MASK                = (RTK_CFG_BITS_ROVER_MODE_RTK_POSITIONING_DEPRECATED | RTK_CFG_BITS_ROVER_MODE_RTK_COMPASSING_DEPRECATED),  //!< Mask selecting the deprecated onboard RTK engine rover modes

    RTK_CFG_BITS_ALL_MODES_MASK                         = (RTK_CFG_BITS_ROVER_MODE_MASK | RTK_CFG_BITS_BASE_MODE),  //!< Mask of Rover, Compassing, and Base modes
};

#define DEFAULT_DYNAMIC_MODEL                   DYNAMIC_MODEL_AIRBORNE_4G
#define DEFAULT_GNSS_MIN_ELEVATION_ANGLE        (10.0f * C_DEG2RAD_F)       // (rad)
#define DEFAULT_GNSS_RTK_CN0_MINIMUM            30                          // (dBHz)
#define DEFAULT_GNSS_RTK_CN0_DYN_MIN_OFFSET     20                          // (dBHz)

/** @brief IMU/magnetometer sensor hardware configuration, used with nvm_flash_cfg_t.sensorConfig (DID_FLASH_CONFIG). Packs several small sub-fields (each with its own _MASK/_OFFSET pair) selecting gyro/accel full-scale range, gyro/accel DLPF bandwidth, IMU-to-sensor-frame mounting rotation, and disable/fault-detection flags for the magnetometer, barometer, and multi-IMU voting. Most fields require a system reset to take effect. */
enum eSensorConfig
{
    SENSOR_CFG_GYR_FS_250                       = (int)0x00000000,  //!< Gyro full-scale range: +-250 deg/s
    SENSOR_CFG_GYR_FS_500                       = (int)0x00000001,  //!< Gyro full-scale range: +-500 deg/s
    SENSOR_CFG_GYR_FS_1000                      = (int)0x00000002,  //!< Gyro full-scale range: +-1000 deg/s
    SENSOR_CFG_GYR_FS_2000                      = (int)0x00000003,  //!< Gyro full-scale range: +-2000 deg/s
    SENSOR_CFG_GYR_FS_4000                      = (int)0x00000004,  //!< Gyro full-scale range: +-4000 deg/s
    SENSOR_CFG_GYR_FS_MAX                       = (int)0x00000007,  //!< Gyro full-scale range: use individual sensor's max range
    SENSOR_CFG_GYR_FS_MASK                      = (int)0x00000007,  //!< Mask for gyro full-scale range field
    SENSOR_CFG_GYR_FS_OFFSET                    = (int)0,           //!< Bit offset of SENSOR_CFG_GYR_FS_MASK within sensorConfig

    SENSOR_CFG_ACC_FS_2G                        = (int)0x00000000,  //!< Accelerometer full-scale range: +-2 g (m/s^2)
    SENSOR_CFG_ACC_FS_4G                        = (int)0x00000001,  //!< Accelerometer full-scale range: +-4 g (m/s^2)
    SENSOR_CFG_ACC_FS_8G                        = (int)0x00000002,  //!< Accelerometer full-scale range: +-8 g (m/s^2)
    SENSOR_CFG_ACC_FS_16G                       = (int)0x00000003,  //!< Accelerometer full-scale range: +-16 g (m/s^2)
    SENSOR_CFG_ACC_FS_32G                       = (int)0x00000004,  //!< Accelerometer full-scale range: +-32 g (m/s^2)
    // SENSOR_CFG_ACC_FS_80G              = (int)0x00000005, Unsupported at this time (available in future; contact sales for more information)
    SENSOR_CFG_ACC_FS_MAX                       = (int)0x00000007,  //!< Accelerometer full-scale range: use individual sensor's max range
    SENSOR_CFG_ACC_FS_MASK                      = (int)0x00000070,  //!< Mask for accelerometer full-scale range field
    SENSOR_CFG_ACC_FS_OFFSET                    = (int)4,           //!< Bit offset of SENSOR_CFG_ACC_FS_MASK within sensorConfig

    /** Gyro digital low-pass filter (DLPF) is set automatically based on the IMU sample rate.  The following
    bit values can be used to override the bandwidth (frequency) to: 250, 184, 92, 41, 20, 10, 5 Hz */
    SENSOR_CFG_GYR_DLPF_250HZ                   = (int)0x00000000,  //!< Gyro DLPF bandwidth override: 250 Hz
    SENSOR_CFG_GYR_DLPF_184HZ                   = (int)0x00000001,  //!< Gyro DLPF bandwidth override: 184 Hz
    SENSOR_CFG_GYR_DLPF_92HZ                    = (int)0x00000002,  //!< Gyro DLPF bandwidth override: 92 Hz
    SENSOR_CFG_GYR_DLPF_41HZ                    = (int)0x00000003,  //!< Gyro DLPF bandwidth override: 41 Hz
    SENSOR_CFG_GYR_DLPF_20HZ                    = (int)0x00000004,  //!< Gyro DLPF bandwidth override: 20 Hz
    SENSOR_CFG_GYR_DLPF_10HZ                    = (int)0x00000005,  //!< Gyro DLPF bandwidth override: 10 Hz
    SENSOR_CFG_GYR_DLPF_5HZ                     = (int)0x00000006,  //!< Gyro DLPF bandwidth override: 5 Hz
    SENSOR_CFG_GYR_DLPF_MASK                    = (int)0x00000F00,  //!< Mask for gyro DLPF bandwidth field
    SENSOR_CFG_GYR_DLPF_OFFSET                  = (int)8,           //!< Bit offset of SENSOR_CFG_GYR_DLPF_MASK within sensorConfig

    /** Accelerometer digital low-pass filter (DLPF) is set automatically based on the IMU sample rate.  The
    following bit values can be used to override the bandwidth (frequency) to: 218, 218, 99, 45, 21, 10, 5 Hz */
    SENSOR_CFG_ACC_DLPF_218HZ                   = (int)0x00000000,  //!< Accelerometer DLPF bandwidth override: 218 Hz
    SENSOR_CFG_ACC_DLPF_218HZb                  = (int)0x00000001,  //!< Accelerometer DLPF bandwidth override: 218 Hz (alternate register value)
    SENSOR_CFG_ACC_DLPF_99HZ                    = (int)0x00000002,  //!< Accelerometer DLPF bandwidth override: 99 Hz
    SENSOR_CFG_ACC_DLPF_45HZ                    = (int)0x00000003,  //!< Accelerometer DLPF bandwidth override: 45 Hz
    SENSOR_CFG_ACC_DLPF_21HZ                    = (int)0x00000004,  //!< Accelerometer DLPF bandwidth override: 21 Hz
    SENSOR_CFG_ACC_DLPF_10HZ                    = (int)0x00000005,  //!< Accelerometer DLPF bandwidth override: 10 Hz
    SENSOR_CFG_ACC_DLPF_5HZ                     = (int)0x00000006,  //!< Accelerometer DLPF bandwidth override: 5 Hz
    SENSOR_CFG_ACC_DLPF_MASK                    = (int)0x0000F000,  //!< Mask for accelerometer DLPF bandwidth field
    SENSOR_CFG_ACC_DLPF_OFFSET                  = (int)12,          //!< Bit offset of SENSOR_CFG_ACC_DLPF_MASK within sensorConfig

    /** Euler rotation of IMU and magnetometer from Hardware Frame to Sensor Frame.  Rotation applied in the order of yaw, pitch, roll from the sensor frame (labeled on uINS). */
    SENSOR_CFG_SENSOR_ROTATION_MASK             = (int)0x001F0000,  //!< Mask for IMU/magnetometer mounting rotation field
    SENSOR_CFG_SENSOR_ROTATION_OFFSET           = (int)16,          //!< Bit offset of SENSOR_CFG_SENSOR_ROTATION_MASK within sensorConfig
    SENSOR_CFG_SENSOR_ROTATION_0_0_0            = (int)0,   //!< Mounting rotation (roll, pitch, yaw deg): 0, 0, 0
    SENSOR_CFG_SENSOR_ROTATION_0_0_90           = (int)1,   //!< Mounting rotation (roll, pitch, yaw deg): 0, 0, 90
    SENSOR_CFG_SENSOR_ROTATION_0_0_180          = (int)2,   //!< Mounting rotation (roll, pitch, yaw deg): 0, 0, 180
    SENSOR_CFG_SENSOR_ROTATION_0_0_N90          = (int)3,   //!< Mounting rotation (roll, pitch, yaw deg): 0, 0, -90
    SENSOR_CFG_SENSOR_ROTATION_90_0_0           = (int)4,   //!< Mounting rotation (roll, pitch, yaw deg): 90, 0, 0
    SENSOR_CFG_SENSOR_ROTATION_90_0_90          = (int)5,   //!< Mounting rotation (roll, pitch, yaw deg): 90, 0, 90
    SENSOR_CFG_SENSOR_ROTATION_90_0_180         = (int)6,   //!< Mounting rotation (roll, pitch, yaw deg): 90, 0, 180
    SENSOR_CFG_SENSOR_ROTATION_90_0_N90         = (int)7,   //!< Mounting rotation (roll, pitch, yaw deg): 90, 0, -90
    SENSOR_CFG_SENSOR_ROTATION_180_0_0          = (int)8,   //!< Mounting rotation (roll, pitch, yaw deg): 180, 0, 0
    SENSOR_CFG_SENSOR_ROTATION_180_0_90         = (int)9,   //!< Mounting rotation (roll, pitch, yaw deg): 180, 0, 90
    SENSOR_CFG_SENSOR_ROTATION_180_0_180        = (int)10,  //!< Mounting rotation (roll, pitch, yaw deg): 180, 0, 180
    SENSOR_CFG_SENSOR_ROTATION_180_0_N90        = (int)11,  //!< Mounting rotation (roll, pitch, yaw deg): 180, 0, -90
    SENSOR_CFG_SENSOR_ROTATION_N90_0_0          = (int)12,  //!< Mounting rotation (roll, pitch, yaw deg): -90, 0, 0
    SENSOR_CFG_SENSOR_ROTATION_N90_0_90         = (int)13,  //!< Mounting rotation (roll, pitch, yaw deg): -90, 0, 90
    SENSOR_CFG_SENSOR_ROTATION_N90_0_180        = (int)14,  //!< Mounting rotation (roll, pitch, yaw deg): -90, 0, 180
    SENSOR_CFG_SENSOR_ROTATION_N90_0_N90        = (int)15,  //!< Mounting rotation (roll, pitch, yaw deg): -90, 0, -90
    SENSOR_CFG_SENSOR_ROTATION_0_90_0           = (int)16,  //!< Mounting rotation (roll, pitch, yaw deg): 0, 90, 0
    SENSOR_CFG_SENSOR_ROTATION_0_90_90          = (int)17,  //!< Mounting rotation (roll, pitch, yaw deg): 0, 90, 90
    SENSOR_CFG_SENSOR_ROTATION_0_90_180         = (int)18,  //!< Mounting rotation (roll, pitch, yaw deg): 0, 90, 180
    SENSOR_CFG_SENSOR_ROTATION_0_90_N90         = (int)19,  //!< Mounting rotation (roll, pitch, yaw deg): 0, 90, -90
    SENSOR_CFG_SENSOR_ROTATION_0_N90_0          = (int)20,  //!< Mounting rotation (roll, pitch, yaw deg): 0, -90, 0
    SENSOR_CFG_SENSOR_ROTATION_0_N90_90         = (int)21,  //!< Mounting rotation (roll, pitch, yaw deg): 0, -90, 90
    SENSOR_CFG_SENSOR_ROTATION_0_N90_180        = (int)22,  //!< Mounting rotation (roll, pitch, yaw deg): 0, -90, 180
    SENSOR_CFG_SENSOR_ROTATION_0_N90_N90        = (int)23,  //!< Mounting rotation (roll, pitch, yaw deg): 0, -90, -90

    /** Magnetometer output data rate (ODR).  Set to enable 100Hz output data rate.  System reset required to enable. */
    // SENSOR_CFG_MAG_ODR_100_HZ                   = (int)0x00200000,       // This is commented out to save instruction space memory.  Uncomment after the system has been optimized.

    SENSOR_CFG_DISABLE_MAGNETOMETER             = (int)0x00400000,  //!< Disable magnetometer sensor (sensorConfig[22])
    SENSOR_CFG_DISABLE_BAROMETER                = (int)0x00800000,  //!< Disable barometer sensor (sensorConfig[23])

    SENSOR_CFG_IMU_FAULT_DETECT_MASK            = (int)0xFF000000,  //!< Mask for multiple-IMU fault detection level field. Higher levels add new features to previous levels.
    SENSOR_CFG_IMU_FAULT_DETECT_GYR             = (int)0x01000000,  //!< Enable multiple IMU gyro fault detection. Must be enabled for other gyro detection modes (offline, large bias, and noise).
    SENSOR_CFG_IMU_FAULT_DETECT_ACC             = (int)0x02000000,  //!< Enable multiple IMU accelerometer fault detection. Must be enabled for other accel detection modes (offline, large bias, and noise).

    // Set to ZERO to exclude from build
    SENSOR_CFG_IMU_FAULT_DETECT_OFFLINE         = 0,  //!< (Disabled/reserved, would be (int)0x04000000) One or more IMUs is offline or stuck
    SENSOR_CFG_IMU_FAULT_DETECT_LARGE_BIAS      = 0,  //!< (Disabled/reserved, would be (int)0x08000000) IMU large-bias fault detection
    SENSOR_CFG_IMU_FAULT_DETECT_SENSOR_NOISE    = 0,  //!< (Disabled/reserved, would be (int)0x10000000) IMU excessive-noise fault detection
};

/** @brief General-purpose I/O pin function configuration, used with nvm_flash_cfg_t.ioConfig (DID_FLASH_CONFIG). Each shared GPIO pin (or pin group, e.g. G1/G2) has a small multi-bit field selecting which of its mutually-exclusive alternate functions is active (strobe I/O, serial port, CAN, I2C, SPI, quadrature encoder, GNSS timepulse); the fields are packed together into a single 32-bit ioConfig value along with GNSS receiver source/type selection and per-IMU disable flags. Use the associated _MASK/_OFFSET pairs and helper macros to read/write individual fields without disturbing the others. */
enum eIoConfig
{
    IO_CONFIG_STROBE_TRIGGER_HIGH       = (int)0x00000001,  //!< Strobe (input and output) trigger on rising edge (0 = falling edge) (ioConfig[0])

    // G1,G2 - STROBE, CAN, Ser2, I2C (future) (ioConfig[3-1])
    IO_CONFIG_G1G2_STROBE_INPUT_G2      = (int)0x00000002,  //!< G1,G2 - STROBE input on G2
    IO_CONFIG_G1G2_CAN_BUS              = (int)0x00000004,  //!< G1,G2 - CAN Bus
    IO_CONFIG_G1G2_COM2                 = (int)0x00000006,  //!< G1,G2 - General Communications on Ser2. Excludes GNSS communications.
    IO_CONFIG_G1G2_I2C                  = (int)0x00000008,  //!< G1,G2 - I2C
    IO_CONFIG_G1G2_MASK                 = (int)0x0000000E,  //!< G1,G2 - MASK.  Note: This G1,G2 setting is overridden when GNSS1 or GNSS2 is configured to use Ser2.
    IO_CONFIG_G1G2_DEFAULT              = IO_CONFIG_G1G2_COM2,  //!< G1,G2 - Default

    // G9 - STROBE, QDEC0 (future) (ioConfig[5-4])
    IO_CONFIG_G9_STROBE_INPUT           = (int)0x00000010,  //!< G9 - Strobe input
    IO_CONFIG_G9_STROBE_OUTPUT_NAV      = (int)0x00000020,  //!< G9 - Enable Nav update strobe output pulse on G9 (uINS pin 10) indicating preintegrated IMU and navigation updates
    IO_CONFIG_G9_SPI_DRDY               = (int)0x00000030,  //!< G9 - SPI DRDY
    IO_CONFIG_G9_MASK                   = (int)0x00000030,  //!< G9 - Bit mask
    IO_CONFIG_G9_DEFAULT                = (int)0,           //!< G9 - Default

    // G6,G7 - Ser1, QDEC0 (future) (ioConfig[7-6])
    IO_CONFIG_G6G7_COM1                 = (int)0x00000040,  //!< G6,G7 - General Communications on Ser1. Excludes GNSS communications.  Overriden when SPI is enabled (G9 held low on bootup/config).
    /** G6,G7 - Quadrature wheel encoder input (G6 QDEC0-A).  Overriden when SPI is enabled (G9 held low on bootup/config). */
//  IO_CONFIG_G6G7_QDEC0_INPUT_G6               = (int)0x00000080,
    IO_CONFIG_G6G7_MASK                 = (int)0x000000C0,  //!< G6,G7 - Bit mask
    IO_CONFIG_G6G7_DEFAULT              = IO_CONFIG_G6G7_COM1,  //!< G6,G7 - Default

    // G5,G8 - STROBE, QDEC1 (future), SPI (enabled when G9 is held low on bootup/config) (ioConfig[10-8])
    IO_CONFIG_G5G8_STROBE_INPUT_G5      = (int)0x00000100,  //!< G5,G8 - Strobe input on G5
    IO_CONFIG_G5G8_STROBE_INPUT_G8      = (int)0x00000200,  //!< G5,G8 - Strobe input on G8
    IO_CONFIG_G5G8_STROBE_INPUT_G5_G8   = (int)0x00000300,  //!< G5,G8 - Strobe input on both G5 and G8
    IO_CONFIG_G5G8_G6G7_SPI_ENABLE      = (int)0x00000400,  //!< G5,G8 - Enable SPI on G6,G7 (SPI mode select)
    IO_CONFIG_G5G8_QDEC_INPUT           = (int)0x00000500,  //!< G5,G8 - Quadrature wheel encoder input (G5 QDEC1-B, G8 QDEC1-A)
    IO_CONFIG_G5G8_MASK                 = (int)0x00000700,  //!< G5,G8 - Bit mask
    IO_CONFIG_G5G8_DEFAULT              = (int)0,           //!< G5,G8 - Default

    IO_CONFIG_G15_STROBE_INPUT          = (int)0x00000800,  //!< G15 (GNSS PPS) - STROBE (ioConfig[11])

    /** GNSS TIMEPULSE source (ioConfig[15-13]) */
    IO_CFG_GNSS1_PPS_SOURCE_OFFSET      = (int)13,          //!< Bit offset of GNSS1 PPS/timepulse source field within ioConfig
    IO_CFG_GNSS1_PPS_SOURCE_MASK        = (int)0x00000007,  //!< Mask (pre-shift) for GNSS1 PPS/timepulse source field
    IO_CFG_GNSS1_PPS_SOURCE_BITMASK     = (int)(IO_CFG_GNSS1_PPS_SOURCE_MASK<<IO_CFG_GNSS1_PPS_SOURCE_OFFSET),  //!< Mask (in-place) for GNSS1 PPS/timepulse source field within ioConfig
    IO_CFG_GNSS1_PPS_SOURCE_DISABLED    = (int)0,  //!< GNSS1 PPS/timepulse source: disabled
    IO_CFG_GNSS1_PPS_SOURCE_G15         = (int)1,  //!< GNSS1 PPS/timepulse source: pin G15
    IO_CFG_GNSS1_PPS_SOURCE_G2          = (int)3,  //!< GNSS1 PPS/timepulse source: pin G2
    IO_CFG_GNSS1_PPS_SOURCE_G5          = (int)4,  //!< GNSS1 PPS/timepulse source: pin G5
    IO_CFG_GNSS1_PPS_SOURCE_G12         = (int)5,  //!< GNSS1 PPS/timepulse source: pin G12
    IO_CFG_GNSS1_PPS_SOURCE_G9          = (int)6,  //!< GNSS1 PPS/timepulse source: pin G9

 #define SET_STATUS_OFFSET_MASK(result,val,offset,mask)    { (result) &= ~((mask)<<(offset)); (result) |= ((val)<<(offset)); }
 #define IO_CFG_GNSS1_PPS_SOURCE(ioConfig) (((ioConfig)>>IO_CFG_GNSS1_PPS_SOURCE_OFFSET)&IO_CFG_GNSS1_PPS_SOURCE_MASK)

     IO_CONFIG_GNSS1_SOURCE_OFFSET                = (int)16,  //!< GNSS 1 source field bit OFFSET (ioConfig[18-16])
     IO_CONFIG_GNSS2_SOURCE_OFFSET                = (int)19,  //!< GNSS 2 source field bit OFFSET (ioConfig[21-19])
     IO_CONFIG_GNSS1_TYPE_OFFSET                  = (int)22,  //!< GNSS 1 type field bit OFFSET (ioConfig[24-22])
     IO_CONFIG_GNSS2_TYPE_OFFSET                  = (int)25,  //!< GNSS 2 type field bit OFFSET (ioConfig[27-25])

    IO_CONFIG_GNSS1_NO_INIT             = (int)0x00001000,  //!< GNSS 1 skip initialization (ioConfig[12])
    IO_CONFIG_GNSS2_NO_INIT             = (int)0x10000000,  //!< GNSS 2 skip initialization (ioConfig[28])

    IO_CONFIG_GNSS_SOURCE_MASK          = (int)0x00000007,  //!< GNSS source field MASK (pre-shift), shared by GNSS1/GNSS2 source fields
    IO_CONFIG_GNSS_SOURCE_DISABLE       = (int)0,  //!< GNSS source - Disable
    IO_CONFIG_GNSS_SOURCE_SER0          = (int)3,  //!< GNSS source - Serial 0
    IO_CONFIG_GNSS_SOURCE_SER1          = (int)4,  //!< GNSS source - Serial 1
    IO_CONFIG_GNSS_SOURCE_SER2          = (int)5,  //!< GNSS source - Serial 2
    IO_CONFIG_GNSS_SOURCE_LAST          = IO_CONFIG_GNSS_SOURCE_SER2,  //!< GNSS source - last valid value (set to last source)

    IO_CONFIG_GNSS_TYPE_MASK            = (int)0x00000007,  //!< GNSS type field MASK (pre-shift), shared by GNSS1/GNSS2 type fields
    IO_CONFIG_GNSS_TYPE_NONE            = (int)0,  //!< GNSS type - Unused.  USE this when adding a new GNSS Receiver
    IO_CONFIG_GNSS_TYPE_UBLOX           = (int)1,  //!< GNSS type - ublox (ZED-F9P or X20) w/ RTK
    IO_CONFIG_GNSS_TYPE_NMEA            = (int)2,  //!< GNSS type - NMEA
    IO_CONFIG_GNSS_TYPE_GPX             = (int)3,  //!< GNSS type - InertialSense GPX
    IO_CONFIG_GNSS_TYPE_SEPTENTRIO      = (int)4,  //!< GNSS type - Septentrio
    IO_CONFIG_GNSS_TYPE_ISB             = (int)5,  //!< GNSS type - Host (pass-through from connected IMX host)
    IO_CONFIG_GNSS_TYPE_LAST            = IO_CONFIG_GNSS_TYPE_ISB,  //!< GNSS type - last valid value (set to last type)

#define IO_CONFIG_GNSS1_SOURCE(ioConfig)    (((ioConfig)>>IO_CONFIG_GNSS1_SOURCE_OFFSET)&IO_CONFIG_GNSS_SOURCE_MASK)
#define IO_CONFIG_GNSS2_SOURCE(ioConfig)    (((ioConfig)>>IO_CONFIG_GNSS2_SOURCE_OFFSET)&IO_CONFIG_GNSS_SOURCE_MASK)
#define IO_CONFIG_GNSS1_TYPE(ioConfig)      (((ioConfig)>>IO_CONFIG_GNSS1_TYPE_OFFSET)&IO_CONFIG_GNSS_TYPE_MASK)
#define IO_CONFIG_GNSS2_TYPE(ioConfig)      (((ioConfig)>>IO_CONFIG_GNSS2_TYPE_OFFSET)&IO_CONFIG_GNSS_TYPE_MASK)

#define SET_IO_CFG_GNSS1_SOURCE(result,val) SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GNSS1_SOURCE_OFFSET, IO_CONFIG_GNSS_SOURCE_MASK)
#define SET_IO_CFG_GNSS2_SOURCE(result,val) SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GNSS2_SOURCE_OFFSET, IO_CONFIG_GNSS_SOURCE_MASK)
#define SET_IO_CFG_GNSS1_TYPE(result,val)   SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GNSS1_TYPE_OFFSET, IO_CONFIG_GNSS_TYPE_MASK)
#define SET_IO_CFG_GNSS2_TYPE(result,val)   SET_STATUS_OFFSET_MASK(result, val, IO_CONFIG_GNSS2_TYPE_OFFSET, IO_CONFIG_GNSS_TYPE_MASK)

    IO_CONFIG_IMU_1_DISABLE             = (int)0x20000000,  //!< IMU 1 disable (ioConfig[29])
    IO_CONFIG_IMU_2_DISABLE             = (int)0x40000000,  //!< IMU 2 disable (ioConfig[30])
    IO_CONFIG_IMU_3_DISABLE             = (int)0x80000000,  //!< IMU 3 disable (ioConfig[31])
};

#define IO_CONFIG_DEFAULT   (IO_CONFIG_G1G2_DEFAULT | IO_CONFIG_G5G8_DEFAULT | IO_CONFIG_G6G7_DEFAULT | IO_CONFIG_G9_DEFAULT)

/** @brief Secondary general-purpose I/O pin function configuration, used with nvm_flash_cfg_t.ioConfig2. Extends eIoConfig to cover pins G11-G13 (each shared between its debug function - SWDIO/SWO/DRDY - and strobe input or XSCL/XSDA), the GNSS2-as-NMEA-source select, and GNSS2 timepulse source. Each field provides both its shifted enumerant value (e.g. IO_CFG2_G11_STROBE_INPUT) and, where distinguished with an "_val" suffix, the already-offset value ready to OR directly into ioConfig2. Note: IO_CFG2_G11_STROBE_INPUT and IO_CFG2_G12_STROBE_INPUT cannot both be set; attempting to do so forces IO_CFG2_G12_STROBE_INPUT back to IO_CFG2_G12_SWO. */
enum eIoConfig2
{
    // NOTE IO_CFG2_G11_STROBE_INPUT and IO_CFG2_G12_STROBE_INPUT
    // cannot be set at the same time. If this is attemped
    // IO_CFG2_G12_STROBE_INPUT will be set to IO_CFG2_G12_SWO
    IO_CFG2_G11_OFFSET                      = (int)0,     //!< G11 (SWDIO) - bit offset of the G11 function field within ioConfig2 (eIoConfig2[0])
    IO_CFG2_G11_MASK                        = (int)0x01,  //!< G11 (SWDIO) - MASK (pre-shift) for the G11 function field
    IO_CFG2_G11_BITMASK                     = (int)(IO_CFG2_G11_MASK<<IO_CFG2_G11_OFFSET),  //!< G11 (SWDIO) - MASK (in-place) for the G11 function field within ioConfig2
    IO_CFG2_G11_SWDIO                       = (int)0,     //!< G11 - function select: SWDIO (debug)
    IO_CFG2_G11_STROBE_INPUT                = (int)1,     //!< G11 - function select: strobe input
    IO_CFG2_G11_SWDIO_val                   = (int)0x00,  //!< G11 - SWDIO, pre-shifted into ioConfig2 bit position
    IO_CFG2_G11_STROBE_INPUT_val            = (int)0x01,  //!< G11 - strobe input, pre-shifted into ioConfig2 bit position
    IO_CFG2_G11_DEFAULT                     = IO_CFG2_G11_SWDIO,  //!< G11 - Default function (SWDIO)


    IO_CFG2_G12_OFFSET                      = (int)1,     //!< G12 (SWO) - bit offset of the G12 function field within ioConfig2 (eIoConfig2[2-1])
    IO_CFG2_G12_MASK                        = (int)0x03,  //!< G12 (SWO) - MASK (pre-shift) for the G12 function field
    IO_CFG2_G12_BITMASK                     = (int)(IO_CFG2_G12_MASK<<IO_CFG2_G12_OFFSET),  //!< G12 (SWO) - MASK (in-place) for the G12 function field within ioConfig2
    IO_CFG2_G12_SWO                         = (int)0,     //!< G12 - function select: SWO (debug)
    IO_CFG2_G12_XSCL                        = (int)1,     //!< G12 - function select: XSCL (secondary I2C clock)
    IO_CFG2_G12_STROBE_INPUT                = (int)2,     //!< G12 - function select: strobe input
    IO_CFG2_G12_SWO_val                     = (int)0x00,  //!< G12 - SWO, pre-shifted into ioConfig2 bit position
    IO_CFG2_G12_XSCL_val                    = (int)0x02,  //!< G12 - XSCL, pre-shifted into ioConfig2 bit position
    IO_CFG2_G12_STROBE_INPUT_val            = (int)0x04,  //!< G12 - strobe input, pre-shifted into ioConfig2 bit position
    IO_CFG2_G12_DEFAULT                     = IO_CFG2_G12_SWO,  //!< G12 - Default function (SWO)

    IO_CFG2_G13_OFFSET                      = (int)3,     //!< G13 (DRDY) - bit offset of the G13 function field within ioConfig2 (eIoConfig2[4-3])
    IO_CFG2_G13_MASK                        = (int)0x03,  //!< G13 (DRDY) - MASK (pre-shift) for the G13 function field
    IO_CFG2_G13_BITMASK                     = (int)(IO_CFG2_G13_MASK<<IO_CFG2_G13_OFFSET),  //!< G13 (DRDY) - MASK (in-place) for the G13 function field within ioConfig2
    IO_CFG2_G13_DRDY                        = (int)0,     //!< G13 - function select: DRDY (data-ready output)
    IO_CFG2_G13_XSDA                        = (int)1,     //!< G13 - function select: XSDA (secondary I2C data)
    IO_CFG2_G13_STROBE_INPUT                = (int)2,     //!< G13 - function select: strobe input
    IO_CFG2_G13_DRDY_val                    = (int)0x00,  //!< G13 - DRDY, pre-shifted into ioConfig2 bit position
    IO_CFG2_G13_XSDA_val                    = (int)0x08,  //!< G13 - XSDA, pre-shifted into ioConfig2 bit position
    IO_CFG2_G13_STROBE_INPUT_val            = (int)0x10,  //!< G13 - strobe input, pre-shifted into ioConfig2 bit position
    IO_CFG2_G13_DEFAULT                     = (int)IO_CFG2_G13_DRDY,  //!< G13 - Default function (DRDY)

    IO_CFG2_USE_GNSS2_AS_SOURCE_OFFSET      = (int)5,     //!< Bit offset of the "use GNSS2 as NMEA source" flag within ioConfig2 (eIoConfig2[5])
    IO_CFG2_USE_GNSS2_AS_SOURCE             = (int)0x20,  //!< When set, use GNSS 2 (instead of GNSS 1) as the NMEA data source

    IO_CFG2_GNSS2_PPS_SOURCE_OFFSET         = (int)6,     //!< Bit offset of GNSS2 PPS/timepulse source field within ioConfig2 (eIoConfig2[7-6])
    IO_CFG2_GNSS2_PPS_SOURCE_MASK           = (int)0x03,  //!< MASK (pre-shift) for GNSS2 PPS/timepulse source field
    IO_CFG2_GNSS2_PPS_SOURCE_BITMASK        = (int)(IO_CFG2_GNSS2_PPS_SOURCE_MASK<<IO_CFG2_GNSS2_PPS_SOURCE_OFFSET),  //!< MASK (in-place) for GNSS2 PPS/timepulse source field within ioConfig2
    IO_CFG2_GNSS2_PPS_SOURCE_DISABLED       = (int)0,  //!< GNSS2 PPS/timepulse source: disabled
    IO_CFG2_GNSS2_PPS_SOURCE_G8             = (int)1,  //!< GNSS2 PPS/timepulse source: pin G8
    IO_CFG2_GNSS2_PPS_SOURCE_G11            = (int)2,  //!< GNSS2 PPS/timepulse source: pin G11
    IO_CFG2_GNSS2_PPS_SOURCE_G13            = (int)3,  //!< GNSS2 PPS/timepulse source: pin G13
    IO_CFG2_GNSS2_PPS_SOURCE_DISABLED_val   = (int)0x00,  //!< GNSS2 PPS/timepulse source disabled, pre-shifted into ioConfig2 bit position
    IO_CFG2_GNSS2_PPS_SOURCE_G8_val         = (int)0x40,  //!< GNSS2 PPS/timepulse source G8, pre-shifted into ioConfig2 bit position
    IO_CFG2_GNSS2_PPS_SOURCE_G11_val        = (int)0x80,  //!< GNSS2 PPS/timepulse source G11, pre-shifted into ioConfig2 bit position
    IO_CFG2_GNSS2_PPS_SOURCE_G13_val        = (int)0xC0,  //!< GNSS2 PPS/timepulse source G13, pre-shifted into ioConfig2 bit position
};

#define IO_CFG2_GNSS2_PPS_SOURCE(ioConfig)  (((ioConfig)>>IO_CFG2_GNSS2_PPS_SOURCE_OFFSET)&IO_CFG2_GNSS2_PPS_SOURCE_MASK)

/** @brief Carrier-board/platform identification and configuration, used with nvm_flash_cfg_t.platformConfig (DID_FLASH_CONFIG). Packs the carrier-board/platform type (which fixes things like PPS pin routing), an optional carrier-specific preset selector (currently defined for the RUG-3 board's serial-port/CAN/GNSS pin-mux presets), the RUG-3 I/O-expander bit positions when presets are disabled, and a one-shot "update ioConfig" trigger. The platform type is normally set once at manufacturing and may be write-protected from OTP memory. */
enum ePlatformConfig
{
    // IMX Carrier Board
    PLATFORM_CFG_TYPE_MASK                                                          = (int)0x0000003F,  //!< Mask for platform/carrier-board type field
    PLATFORM_CFG_TYPE_FROM_MANF_OTP                                                 = (int)0x00000080,  //!< Type is overwritten from manufacturing OTP memory.  Write protection, prevents direct change of platformType in flashConfig.
    PLATFORM_CFG_TYPE_NONE                                                          = (int)0,   //!< No/unknown carrier board (IMX-5 default)
    PLATFORM_CFG_TYPE_RUG3_G0                                                       = (int)8,   //!< PCB RUG-3.x.         PPS disabled
    PLATFORM_CFG_TYPE_RUG3_G1                                                       = (int)9,   //!< PCB RUG-3.x.         PPS1 on G15 (pin 20)
    PLATFORM_CFG_TYPE_RUG3_G2                                                       = (int)10,  //!< PCB RUG-3.x.         PPS1 on G15 (pin 20)
    PLATFORM_CFG_TYPE_EVB2_G2                                                       = (int)11,  //!< EVB-2 carrier board
    PLATFORM_CFG_TYPE_TBED3                                                         = (int)12,  //!< Testbed-3:           PPS1 on  G5 (pin  9), PPS2 on G8 (pin 8)
    PLATFORM_CFG_TYPE_IG1_0_G2                                                      = (int)13,  //!< IG-1.0:              PPS1 on  G8 (pin  8)
    PLATFORM_CFG_TYPE_IG1_G1                                                        = (int)14,  //!< IG-1.1 and later:    PPS1 on G15 (pin 20)
    PLATFORM_CFG_TYPE_IG1_G2                                                        = (int)15,  //!< IG-1.1 and later:    PPS1 on G15 (pin 20)
    PLATFORM_CFG_TYPE_IG2                                                           = (int)16,  //!< IG-2:                PPS1 on G15 (pin 20)
    PLATFORM_CFG_TYPE_LAMBDA_G1                                                     = (int)17,  //!< Enable UBX output on Lambda for testbed
    PLATFORM_CFG_TYPE_LAMBDA_G2                                                     = (int)18,  //!< Enable UBX output on Lambda for testbed
    PLATFORM_CFG_TYPE_TBED2_G1_W_LAMBDA                                             = (int)19,  //!< Enable UBX input from Lambda
    PLATFORM_CFG_TYPE_TBED2_G2_W_LAMBDA                                             = (int)20,  //!< Enable UBX input from Lambda
    PLATFORM_CFG_TYPE_IMX_BRK_1                                                     = (int)21,  //!< IS-IMX-GPX-DEV-1:    PPS1 on G15 (pin 20), PPS2 on G13 (pin 14)
    PLATFORM_CFG_TYPE_RUG4_G2                                                       = (int)22,  //!< PCB RUG-4.x:         PPS1 on G15 (pin 20), PPS2 on G11 (pin 16)
    PLATFORM_CFG_TYPE_IG2_1                                                         = (int)23,  //!< IG-2.1 and later:    PPS1 on G15 (pin 20), PPS2 on G13 (pin 14)
    PLATFORM_CFG_TYPE_COUNT                                                         = (int)24,  //!< Number of defined platform/carrier-board types

    // Presets
    PLATFORM_CFG_PRESET_MASK                                                        = (int)0x0000FF00,  //!< Mask for carrier-specific preset selector field
    PLATFORM_CFG_PRESET_OFFSET                                                      = (int)8,            //!< Bit offset of PLATFORM_CFG_PRESET_MASK within platformConfig

    // RUG-3 - Presets
    PLATFORM_CFG_RUG3_PRESET__0__PRESETS_DISABLED                                   = 0,  //!< RUG-3 preset: don't use presets. IOEXP_BITS can be set directly.
    PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GNSS1             = 1,  //!< RUG-3 preset: Ser0 RS232 (pins 7,9), CAN (pins 11,12), Ser1 GNSS1 (RUG-3-G0 default)
    PLATFORM_CFG_RUG3_PRESET__2__S0_TTL_7_9_____CAN_11_12______S1_GNSS1             = 2,  //!< RUG-3 preset: Ser0 TTL (pins 7,9), CAN (pins 11,12), Ser1 GNSS1
    PLATFORM_CFG_RUG3_PRESET__3__S0_TTL_7_9_____S2_TTL_8_10____S1_GNSS1             = 3,  //!< RUG-3 preset: Ser0 TTL (pins 7,9), Ser2 TTL (pins 8,10), Ser1 GNSS1
    PLATFORM_CFG_RUG3_PRESET__4__S0_RS232_7_9___S1_RS232_8_10__S2_GNSS1             = 4,  //!< RUG-3 preset: Ser0 RS232 (pins 7,9), Ser1 RS232 (pins 8,10), Ser2 GNSS1
    PLATFORM_CFG_RUG3_PRESET__5__S1_RS485_7_8_9_10_____________S2_GNSS1__S0_GNSS2   = 5,  //!< RUG-3 preset: Ser1 RS485 (pins 7,8,9,10), Ser2 GNSS1, Ser0 GNSS2
    PLATFORM_CFG_RUG3_PRESET__6__SPI_7_8_9_10__________________S2_GNSS1__S0_GNSS2   = 6,  //!< RUG-3 preset: SPI (pins 7,8,9,10), Ser2 GNSS1, Ser0 GNSS2
    PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GNSS1__S0_GNSS2   = 7,  //!< RUG-3 preset: Ser1 RS232 (pins 8,10), Ser2 GNSS1, Ser0 GNSS2 (RUG-3-G2 default)
    PLATFORM_CFG_RUG3_PRESET__8_________________CAN_11_12______S1_GNSS1__S0_GNSS2   = 8,  //!< RUG-3 preset: CAN (pins 11,12), Ser1 GNSS1, Ser0 GNSS2
    PLATFORM_CFG_RUG3_PRESET__9__S2_TTL_8_10___________________S1_GNSS1__S0_GNSS2   = 9,  //!< RUG-3 preset: Ser2 TTL (pins 8,10), Ser1 GNSS1, Ser0 GNSS2
    PLATFORM_CFG_RUG3_PRESET__COUNT                                                 = 10,  //!< Number of defined RUG-3 presets

    PLATFORM_CFG_RUG3_PRESET__G0_DEFAULT                                            = PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GNSS1,  //!< Default preset for RUG-3-G0
    PLATFORM_CFG_RUG3_PRESET__G2_DEFAULT                                            = PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GNSS1__S0_GNSS2,  //!< Default preset for RUG-3-G2

    // RUG-3 - I/O Expander disabled if platform type is != PLATFORM_CFG_TYPE_RUG3_x.
    PLATFORM_CFG_RUG3_IOEXP_BIT_MASK                                                = (int)0x00FF0000,  //!< Mask for RUG-3 I/O-expander raw bits field (used only when presets are disabled)
    PLATFORM_CFG_RUG3_IOEXP_BIT_OFFSET                                              = (int)16,           //!< Bit offset of PLATFORM_CFG_RUG3_IOEXP_BIT_MASK within platformConfig

    RUG3_IOEXP_BIT_OFFSET_n232_485                                                  = (int)0,  //!< RUG-3 I/O expander bit: n232/485 select
    RUG3_IOEXP_BIT_OFFSET_n232_TTL                                                  = (int)1,  //!< RUG-3 I/O expander bit: n232/TTL select
    RUG3_IOEXP_BIT_OFFSET_nRS_CAN                                                   = (int)2,  //!< RUG-3 I/O expander bit: nRS/CAN select
    RUG3_IOEXP_BIT_OFFSET_nGNSS2_RS                                                 = (int)3,  //!< RUG-3 I/O expander bit: nGNSS2/RS select
    RUG3_IOEXP_BIT_OFFSET_nSPIEN                                                    = (int)4,  //!< RUG-3 I/O expander bit: nSPI enable
    RUG3_IOEXP_BIT_OFFSET_nSPI_SER                                                  = (int)5,  //!< RUG-3 I/O expander bit: nSPI/Serial select
    RUG3_IOEXP_BIT_OFFSET_nGPSRST                                                   = (int)6,  //!< RUG-3 I/O expander bit: nGPS reset

    PLATFORM_CFG_UPDATE_IO_CONFIG                                                   = (int)0x01000000,  //!< Generate ioConfig based on platform config
};

/** @brief (DID_WHEEL_ENCODER) Wheel encoder measurements to be fused with GNSS-INS. Configure DID_GROUND_VEHICLE (wheel_config_t: transform, track width, radius) before sending/enabling this message; the theta_l/theta_r and wrap_count fields are internal development plumbing and should not be relied on by host applications. */
typedef struct PACKED
{
    double      timeOfWeek;    //!< (s) (Do not use, internal development only) Time of measurement, seconds into current GNSS week

    uint32_t    status;        //!< Wheel encoder status bits

    float       theta_l;       //!< (rad) (Do not use, internal development only) Left wheel angle

    float       theta_r;       //!< (rad) (Do not use, internal development only) Right wheel angle

    float       omega_l;       //!< (rad/s) Left wheel angular rate. Positive when wheel is turning toward the forward direction of the vehicle. Use WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT in DID_FLASH_CONFIG::wheelConfig to reverse this.

    float       omega_r;       //!< (rad/s) Right wheel angular rate. Positive when wheel is turning toward the forward direction of the vehicle. Use WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT in DID_FLASH_CONFIG::wheelConfig to reverse this.

    uint32_t    wrap_count_l;  //!< (Do not use, internal development only) Left wheel revolution (wrap-around) count

    uint32_t    wrap_count_r;  //!< (Do not use, internal development only) Right wheel revolution (wrap-around) count

    float       var_wheel_omega;   //!< (rad^2/s^2) Wheel encoder velocity noise variance

    float       var_wheel_theta;   //!< (rad^2) Wheel encoder angle noise variance

} wheel_encoder_t;

/** @brief Wheel encoder/kinematic-constraint configuration bitflags, used with wheel_config_t.bits (DID_GROUND_VEHICLE) and DID_FLASH_CONFIG::wheelConfig. OR the desired bits together to enable encoder fusion and/or wheel-based velocity control, reverse the sign convention of either wheel's reported angular rate to match physical wiring, and select whether the encoder hardware is read by the uINS or the EVB. */
enum eWheelCfgBits
{
    WHEEL_CFG_BITS_ENABLE_ENCODER           = (int)0x00000002,    //!< Enable wheel encoder fusion with GNSS-INS
    WHEEL_CFG_BITS_ENABLE_CONTROL           = (int)0x00000004,    //!< Enable wheel-based velocity control
    WHEEL_CFG_BITS_ENABLE_MASK              = (int)0x0000000F,    //!< Mask isolating the enable sub-field
    WHEEL_CFG_BITS_DIRECTION_REVERSE_LEFT   = (int)0x00000100,    //!< Reverse the reported direction of DID_WHEEL_ENCODER::omega_l
    WHEEL_CFG_BITS_DIRECTION_REVERSE_RIGHT  = (int)0x00000200,    //!< Reverse the reported direction of DID_WHEEL_ENCODER::omega_r
    WHEEL_CFG_BITS_ENCODER_SOURCE           = (int)0x00000400,    //!< Source of wheel encoder hardware: 0 = uINS, 1 = EVB
};

/** @brief Ground vehicle kinematic-learning mode/command codes, used with ground_vehicle_t.mode (DID_GROUND_VEHICLE). GV_MODE_STANDBY and GV_MODE_LEARNING are read-back state values; the GV_CMD_* values are commands written by the host to start, resume, restart, save, or cancel the kinematic (IMU-to-wheel transform) learning process. */
enum eGroundVehicleMode
{
    GV_MODE_STANDBY                 = 0,   //!< STATUS: Kinematic learning is idle
    GV_MODE_LEARNING                = 1,   //!< STATUS: Kinematic learning is in progress
    GV_CMD_LEARNING_START           = 2,   //!< COMMAND: Start learning, using the provided transform and sigma values as the initial estimate
    GV_CMD_LEARNING_RESUME          = 3,   //!< COMMAND: Resume learning, resetting sigma values while keeping the current transform estimate
    GV_CMD_LEARNING_CLEAR_AND_START = 4,   //!< COMMAND: Zero the transform and reset sigma values, then start learning from scratch
    GV_CMD_LEARNING_STOP_AND_SAVE   = 5,   //!< COMMAND: Stop learning and save the resulting transform to flash
    GV_CMD_LEARNING_CANCEL          = 6,   //!< COMMAND: Abort learning without saving
};

/** @brief Rigid-body transform (rotation + translation) from the IMU/body frame to the ground-vehicle wheel frame (defined at the center of the non-steering axle), embedded in wheel_config_t (DID_GROUND_VEHICLE). Populated by the kinematic learning process (see eGroundVehicleMode); the *_sigma fields report the current estimation uncertainty of the corresponding transform component and shrink toward zero as learning converges. */
typedef struct PACKED
{
    float                   e_b2w[3];        //!< (rad) Euler angles describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)

    float                   e_b2w_sigma[3];  //!< (rad) Euler angle standard deviation of measurements describing the rotation from imu (body) to the wheel frame (center of the non-steering axle)

    float                   t_b2w[3];        //!< (m) Translation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame

    float                   t_b2w_sigma[3];  //!< (m) Translation standard deviation from the imu (body) to the wheel frame origin (center of the non-steering axle), expressed in the imu (body) frame

} wheel_transform_t;

/** @brief Wheel encoder kinematic configuration: the IMU-to-wheel transform plus the fixed vehicle geometry (track width, wheel radius) needed to convert wheel_encoder_t angular rates into a vehicle velocity constraint for GNSS-INS fusion. Embedded in ground_vehicle_t (DID_GROUND_VEHICLE) and mirrored at DID_FLASH_CONFIG::wheelConfig. */
typedef struct PACKED
{
    uint32_t                bits;         //!< Config bits (see eWheelCfgBits)

    wheel_transform_t       transform;    //!< Euler angles and offset describing the rotation and translation from imu (body) to the wheel frame (center of the non-steering axle)

    float                   track_width;  //!< (m) Distance between the left and right wheels

    float                   radius;       //!< (m) Estimate of wheel radius

} wheel_config_t;

/** @brief Ground vehicle kinematic-constraint status bitflags, used with ground_vehicle_t.status (DID_GROUND_VEHICLE). Report whether transform learning is currently enabled, whether navigation is dead-reckoning without GNSS, whether the learned kinematic parameters agree with GNSS, and whether learning has converged or is still needed. */
enum eGroundVehicleStatus
{
    GV_STATUS_LEARNING_ENABLED      = 0x00000001,  //!< Kinematic learning is solving for the translation from IMU to wheel (wheel_config)

    GV_STATUS_DEAD_RECKONING        = 0x01000000,  //!< Navigation is running without GNSS input

    GV_STATUS_KINEMATIC_CAL_GOOD    = 0x02000000,  //!< Vehicle kinematic parameters agree with GNSS

    GV_STATUS_LEARNING_CONVERGED    = 0x04000000,  //!< Vehicle kinematic learning has converged and is complete

    GV_STATUS_LEARNING_NEEDED       = 0x08000000,  //!< Vehicle kinematic learning data (wheel_config_t) is missing

};

/** @brief (DID_GROUND_VEHICLE) Static configuration for wheel transform measurements. Reports and commands the ground-vehicle kinematic learning process: mode is written by the host to start/resume/save/cancel learning (see eGroundVehicleMode) and status reports learning progress/health (see eGroundVehicleStatus); wheelConfig holds the resulting IMU-to-wheel transform and vehicle geometry consumed when fusing DID_WHEEL_ENCODER measurements. */
typedef struct PACKED
{
    uint32_t                timeOfWeekMs;  //!< (ms) GPS time of week (since Sunday morning)

    uint32_t                status;        //!< Ground vehicle status flags (see eGroundVehicleStatus)

    uint32_t                mode;          //!< Current mode of the ground vehicle.  Use this field to apply commands. (see eGroundVehicleMode)

    wheel_config_t          wheelConfig;   //!< Wheel transform, track width, and wheel radius

} ground_vehicle_t;

/** @brief INS dynamic platform model selection, used with nvm_flash_cfg_t.dynamicModel (DID_FLASH_CONFIG). Selects a motion-profile model (expected acceleration/jerk limits) that the EKF and the GNSS receiver's own navigation filter use to balance measurement noise rejection against tracking responsiveness; the model chosen must be at least as dynamic as the actual platform motion or navigation accuracy will suffer. Also passed through to the GNSS receiver's dynamic model setting where supported. */
enum eDynamicModel
{
    DYNAMIC_MODEL_PORTABLE          = 0,   //!< General portable use, default balance of noise rejection and responsiveness
    DYNAMIC_MODEL_FIXED_POSITION    = 1,   //!< Stationary, no motion expected at all (e.g. surveyed base station)
    DYNAMIC_MODEL_STATIONARY        = 2,   //!< Stationary, minimal motion expected
    DYNAMIC_MODEL_PEDESTRIAN        = 3,   //!< Walking-speed motion with frequent direction/speed changes
    DYNAMIC_MODEL_GROUND_VEHICLE    = 4,   //!< Wheeled ground vehicle dynamics
    DYNAMIC_MODEL_MARINE            = 5,   //!< Surface marine vessel dynamics
    DYNAMIC_MODEL_AIRBORNE_1G       = 6,   //!< Airborne, limited to approximately 1g of acceleration
    DYNAMIC_MODEL_AIRBORNE_2G       = 7,   //!< Airborne, limited to approximately 2g of acceleration
    DYNAMIC_MODEL_AIRBORNE_4G       = 8,   //!< Airborne, limited to approximately 4g of acceleration
    DYNAMIC_MODEL_WRIST             = 9,   //!< Wrist-worn device dynamics
    DYNAMIC_MODEL_INDOOR            = 10,  //!< Indoor pedestrian/robot use with minimal GNSS visibility
    DYNAMIC_MODEL_COUNT    //!< Must be last; count of defined dynamic models, not a valid model selection
};

/** @brief IMU shock-rejection configuration bitflags, used with nvm_flash_cfg_t.imuShockOptions (DID_FLASH_CONFIG). Controls whether the EKF detects and rejects IMU shock/impact events during attitude estimation. */
enum eImuShockOptions
{
    IMU_SHOCK_OPTIONS_ENABLE        = 0x01,  //!< Enable IMU shock detection and rejection
    IMU_SHOCK_OPTIONS_FAST_RECOVERY = 0x02   //!< Useful when shocks occur during rotation, as it minimizes post-shock error and reduces the time required for attitude convergence. However, this option increases the amount of attitude error induced by shock events and is therefore discouraged when the platform is stationary and not rotating during shocks.
};

/** @brief (DID_FLASH_CONFIG) Persistent device configuration, written to and read back from flash memory (NVM). This is the device's primary user-configurable settings block, covering IMU/GNSS/navigation update rates, serial port baud rates, sensor-to-body-frame rotations and lever-arm offsets, dynamic model, platform config, RTK config, hardware I/O config, sensor config, magnetometer calibration thresholds, ground-vehicle/wheel-encoder config, magnetic declination, and IMU shock-rejection tuning. Applied by the device firmware at boot and re-applied/persisted whenever the host writes this DID. Most fields participate in the flash checksum (the checksum field itself covers the whole struct except size and checksum); PLATFORM_CFG_UPDATE_IO_CONFIG within platformConfig is a documented exception, excluded from the checksum and from upload change-detection so it can be used as a one-shot trigger.
 * IMPORTANT: These fields should not be deleted, they can be deprecated and marked as reserved,
 * or new fields added to the end.
 * NOTE: The key value must be incremented to ensure the defaults are restored anytime the fields
 * change or the default values change.  Default changes should be noted in the changelog.
*/
typedef struct PACKED
{
    uint32_t                size;                               //!< Size of group or union, which is nvm_group_x_t + padding

    uint32_t                checksum;                           //!< Checksum, excluding size and checksum.  0xFFFFFFFF is invalid.

    uint32_t                key;                                //!< Manufacturer method for restoring flash defaults

    uint32_t                startupImuDtMs;                     //!< (ms) IMU sample (system input) period set on startup. Cannot be larger than startupNavDtMs. Zero disables sensor/IMU sampling.

    uint32_t                startupNavDtMs;                     //!< (ms) Navigation filter (system output) output period set on startup.  Used to initialize sysParams.navOutputPeriodMs.

    uint32_t                ser0BaudRate;                       //!< (bps) Serial port 0 baud rate

    uint32_t                ser1BaudRate;                       //!< (bps) Serial port 1 baud rate

    float                   insRotation[3];                     //!< (rad) Rotation about the X,Y,Z axes from Sensor Frame to Intermediate Output Frame.  Order applied: Z,Y,X.

    float                   insOffset[3];                       //!< (m) X,Y,Z offset from Intermediate Output Frame to INS Output Frame.

    float                   gnss1AntOffset[3];                  //!< (m) X,Y,Z offset in Sensor Frame to GNSS 1 antenna.

    uint8_t                 dynamicModel;                       //!< INS dynamic platform model (see eDynamicModel).  Options are: 0=PORTABLE, 2=STATIONARY, 3=PEDESTRIAN, 4=GROUND VEHICLE, 5=SEA, 6=AIRBORNE_1G, 7=AIRBORNE_2G, 8=AIRBORNE_4G, 9=WRIST.  Used to balance noise and performance characteristics of the system.  The dynamics selected here must be at least as fast as your system or you experience accuracy error.  This is tied to the GNSS position estimation model and intend in the future to be incorporated into the INS position model.

    uint8_t                 debug;                              //!< Debug

    uint16_t                gnssSatSigConst;                    //!< Satellite system constellation used in GNSS solution (see eGnssSatSigConst). 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS

    uint32_t                sysCfgBits;                         //!< System configuration bits (see eSysConfigBits).

    double                  refLla[3];                          //!< (deg, deg, m) Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations

    double                  lastLla[3];                         //!< (deg, deg, m) Last latitude, longitude, HAE (height above ellipsoid) used to aid GNSS startup.  Updated when the distance between current LLA and lastLla exceeds lastLlaUpdateDistance.

    uint32_t                lastLlaTimeOfWeekMs;                //!< (ms) Last LLA GPS time since week start (Sunday morning)

    uint32_t                lastLlaWeek;                        //!< Last LLA GPS number of weeks since January 6th, 1980

    float                   lastLlaUpdateDistance;              //!< (m) Distance between current and last LLA that triggers an update of lastLla

    uint32_t                ioConfig;                           //!< Hardware interface configuration bits (see eIoConfig).

    uint32_t                platformConfig;                     //!< Hardware platform specifying the IMX carrier board type (i.e. RUG, EVB, IG) and configuration bits (see ePlatformConfig).  The platform type is used to simplify the GNSS and I/O configuration process.  Bit PLATFORM_CFG_UPDATE_IO_CONFIG is excluded from the flashConfig checksum and from determining whether to upload.

    float                   gnss2AntOffset[3];                  //!< (m) X,Y,Z offset in Sensor Frame origin to GNSS 2 antenna.

    float                   zeroVelRotation[3];                 //!< (rad) Euler (roll, pitch, yaw) rotation from INS Sensor Frame to Intermediate ZeroVelocity Frame.  Order applied: heading, pitch, roll.

    float                   zeroVelOffset[3];                   //!< (m) X,Y,Z offset from Intermediate ZeroVelocity Frame to Zero Velocity Frame.

    float                   gnssTimeUserDelay;                  //!< (sec) User defined delay for GPS time.  This parameter can be used to account for GNSS antenna cable delay.

    float                   magDeclination;                     //!< (rad) Earth magnetic field (magnetic north) declination (heading offset from true north)

    uint32_t                gnssTimeSyncPeriodMs;               //!< (ms) Time between GPS time synchronization pulses.  Requires reboot to take effect.

    uint32_t                startupGnssDtMs;                    //!< (ms) GNSS measurement (system input) update period set on startup. 200ms minimum (5Hz max).

    uint32_t                RTKCfgBits;                         //!< RTK configuration bits (see eRTKConfigBits).

    uint32_t                sensorConfig;                       //!< Sensor config to specify the full-scale sensing ranges and output rotation for the IMU and magnetometer (see eSensorConfig)

    float                   gnssMinimumElevation;               //!< (rad) Minimum elevation of a satellite above the horizon to be used in the solution. Low elevation satellites may provide degraded accuracy, due to the long signal path through the atmosphere.

    uint32_t                ser2BaudRate;                       //!< (bps) Serial port 2 baud rate

    wheel_config_t          wheelConfig;                        //!< Wheel encoder: euler angles describing the rotation from imu to left wheel, plus track width/radius and config bits (see eWheelCfgBits)

    float                   magInterferenceThreshold;           //!< Magnetometer interference sensitivity threshold. Typical range is 2-10 (3 default) and 1000 to disable mag interference detection.

    float                   magCalibrationQualityThreshold;     //!< Magnetometer calibration quality sensitivity threshold. Typical range is 10-20 (10 default) and 1000 to disable mag calibration quality check, forcing it to be always good.

    uint8_t                 gnssCn0Minimum;                     //!< (dBHz) GNSS CN0 absolute minimum threshold for signals.  Used to filter signals in RTK solution.

    uint8_t                 gnssCn0DynMinOffset;                //!< (dBHz) GNSS CN0 dynamic minimum threshold offset below max CN0 across all satellites. Used to filter signals used in RTK solution. To disable, set gnssCn0DynMinOffset to zero and increase gnssCn0Minimum.

    uint8_t                 imuRejectThreshGyroLow;             //!< IMU gyro fault rejection threshold low

    uint8_t                 imuRejectThreshGyroHigh;            //!< IMU gyro fault rejection threshold high

    uint8_t                 imuShockDetectLatencyMs;            //!< (ms) IMU shock detection latency.  Time used for EKF rewind to prevent shock from influencing EKF estimates.

    uint8_t                 imuShockRejectLatchMs;              //!< (ms) IMU shock rejection latch time.  Time required following detected shock end to disable shock rejection.

    uint8_t                 imuShockOptions;                    //!< IMU shock rejection options (see eImuShockOptions)

    uint8_t                 imuShockDeltaAccHighThreshold;      //!< (m/s^2) IMU shock detection. Min acceleration difference between the 3 IMUs to detect the start of a shock.

    uint8_t                 imuShockDeltaAccLowThreshold;       //!< (m/s^2) IMU shock detection. Max acceleration difference between the 3 IMUs within the latch time to detect the end of a shock.

    uint8_t                 imuShockDeltaGyroHighThreshold;     //!< (deg/s) IMU shock detection. Min angular rate difference between the 3 IMUs to detect the start of a shock.

    uint8_t                 imuShockDeltaGyroLowThreshold;      //!< (deg/s) IMU shock detection. Max angular rate difference between the 3 IMUs within the latch time to detect the end of a shock.

    uint8_t                 ioConfig2;                          //!< Hardware interface configuration bits for GNSS2 PPS (see eIoConfig2).

} nvm_flash_cfg_t;

/** @brief (DID_INL2_NED_SIGMA) 1-sigma standard deviations of the INL2 (INS/GNSS loosely-coupled) EKF state estimates, expressed in the NED (north/east/down) frame. Reports estimation uncertainty, not the estimates themselves; see the corresponding INL2 state/status DIDs for the actual state values. */
typedef struct PACKED
{
    unsigned int            timeOfWeekMs;       //!< (ms) GPS time of week (since Sunday morning)
    float                   StdPosNed[3];       //!< (m) NED position error sigma
    float                   StdVelNed[3];       //!< (m/s) NED velocity error sigma
    float                   StdAttNed[3];       //!< (rad) NED attitude error sigma
    float                   StdAccBias[3];      //!< (m/s^2) Acceleration bias error sigma
    float                   StdGyrBias[3];      //!< (rad/s) Angular rate bias error sigma
    float                   StdBarBias;         //!< (m) Barometric altitude bias error sigma
    float                   StdMagDeclination;  //!< (rad) Mag declination error sigma
} inl2_ned_sigma_t;

/** @brief (DID_STROBE_IN_TIME) Timestamp of a rising/falling edge detected on a configured strobe input pin (see eIoConfig / eIoConfig2 for pin-to-function assignment). Emitted once per detected edge. */
typedef struct PACKED
{
    uint32_t                week;               //!< GPS number of weeks since January 6th, 1980

    uint32_t                timeOfWeekMs;       //!< (ms) GPS time of week (since Sunday morning)

    uint16_t                pin;                //!< Strobe input pin (i.e. G1, G2, G5, G9, G11, G12, G13, G15)

    uint16_t                count;              //!< Strobe serial index number, incremented once per detected strobe edge
} strobe_in_time_t;

#define DEBUG_I_ARRAY_SIZE      9
#define DEBUG_F_ARRAY_SIZE      9
#define DEBUG_LF_ARRAY_SIZE     3

/** @brief (DID_DEBUG_ARRAY / DID_EVB_DEBUG_ARRAY / DID_GPX_DEBUG_ARRAY) INTERNAL USE ONLY. Generic scratch container of integer, float, and double values used by firmware developers to expose ad-hoc debug values over the DID stream without needing a dedicated message; field meanings are not fixed and vary by firmware build/debug session. */
typedef struct PACKED
{
    int32_t                 i[DEBUG_I_ARRAY_SIZE];   //!< Debug integer values, meaning defined by current firmware debug build
    float                   f[DEBUG_F_ARRAY_SIZE];   //!< Debug float values, meaning defined by current firmware debug build
    double                  lf[DEBUG_LF_ARRAY_SIZE];  //!< Debug double (long float) values, meaning defined by current firmware debug build
} debug_array_t;

#define DEBUG_STRING_SIZE   80

/** @brief (DID_DEBUG_STRING) INTERNAL USE ONLY. Fixed-size ASCII debug string used by firmware developers for ad-hoc textual debug output. */
typedef struct PACKED
{
    uint8_t     s[DEBUG_STRING_SIZE];  //!< Debug string content, null-terminated if shorter than DEBUG_STRING_SIZE
} debug_string_t;

POP_PACK

PUSH_PACK_8

/** @brief (DID_RTK_STATE) INTERNAL USE ONLY. RTK solver's internal Kalman filter state: rover/base position, velocity, and acceleration in ECEF, plus the per-satellite carrier-phase bias states and their covariances used to resolve integer ambiguities. Not intended for consumption by host applications. */
typedef struct PACKED
{
    gtime_t time;               //!< GPS time of solution
    double  rp_ecef[3];         //!< (m) Rover position, ECEF
    double  rv_ecef[3];         //!< (m/s) Rover velocity, ECEF
    double  ra_ecef[3];         //!< (m/s^2) Rover acceleration, ECEF
    double  bp_ecef[3];         //!< (m) Base position, ECEF
    double  bv_ecef[3];         //!< (m/s) Base velocity, ECEF
    double  qr[6];               //!< Rover position and velocity covariance, main diagonal
    double  b[24];               //!< (cycles) Satellite carrier-phase bias state
    double  qb[24];              //!< Main diagonal of satellite bias covariances
    uint8_t sat_id[24];          //!< Satellite id corresponding to each entry of b[]
} rtk_state_t;

/** @brief (DID_RTK_PHASE_RESIDUAL / DID_RTK_CODE_RESIDUAL) INTERNAL USE ONLY. Per-satellite-pair measurement residuals from the RTK double-difference solver, used to evaluate solution quality and debug ambiguity resolution. DID_RTK_PHASE_RESIDUAL carries carrier-phase residuals, DID_RTK_CODE_RESIDUAL carries code (pseudorange) residuals; both share this same layout. */
typedef struct PACKED
{
    gtime_t time;                //!< GPS time of the measurement epoch
    int32_t nv;                  //!< Number of measurements (valid entries in the arrays below)
    uint8_t sat_id_i[24];        //!< Satellite id of measurement (reference satellite of the double-difference pair)
    uint8_t sat_id_j[24];        //!< Satellite id of measurement (other satellite of the double-difference pair)
    uint8_t type[24];            //!< Residual type (0 = dd-range, 1 = dd-phase, 2 = baseline)
    double  v[24];               //!< Residual value
} rtk_residual_t;

/** @brief (DID_RTK_DEBUG) RTK solver debug/diagnostic counters and error flags, reported periodically to help diagnose RTK float/fix solution quality issues (base link age, cycle slips, observation filtering, etc.). Many rtkd_unused* fields are reserved padding preserved for binary layout compatibility. */
typedef struct PACKED
{
    gtime_t     time;                   //!< GPS time of debug snapshot

    uint8_t     rtkd_unused8_1;          //!< Reserved/unused (padding)
    uint8_t     code_outlier;           //!< Code residual in float solution too large
    uint8_t     phase_outlier;          //!< Phase residual in float solution too large
    uint8_t     rtkd_unused8_2;          //!< Reserved/unused (padding)

    uint8_t     rtkd_unused8_3;          //!< Reserved/unused (padding)
    uint8_t     rtkd_unused8_4;          //!< Reserved/unused (padding)
    uint8_t     bad_baseline_holdamb;   //!< Bad baseline during hold ambiguity (may not be needed, consider removing)
    uint8_t     rtkd_unused8_5;          //!< Reserved/unused (padding)

    uint8_t     outc_ovfl;              //!< Observation/reject outage counter
    uint8_t     rtkd_unused8_6;          //!< Reserved/unused (padding)
    uint8_t     rtkd_unused8_7;          //!< Reserved/unused (padding)
    uint8_t     large_v2b;              //!< Vector to base distance too large

    uint8_t     base_position_update;   //!< Received position of base correction counter
    uint8_t     rover_position_error;   //!< Rover position error counter
    uint8_t     reset_bias;             //!< Satellite bias reset counter
    uint8_t     rtkd_unused8_8;          //!< Reserved/unused (padding)

    float       pos_variance;           //!< position variance

    uint8_t     diff_age_error;         //!< Difference age too large
    uint8_t     rtkd_unused8_9;          //!< Reserved/unused (padding)
    uint8_t     rover_packet_age_ms;    //!< Age of last received rover packet  (TODO) convert to int16_t
    uint8_t     base_packet_age_ms;     //!< Age of last received base packet  (TODO) convert to int16_t

    uint32_t    rtkd_unused32_1;         //!< Reserved/unused (padding)

    uint32_t    cycle_slips;            //!< Accumulation of total cycle slips

    float       rtk_to_rcvr_pos_error;  //!< RTK position Error with respect to GNSS receiver

    uint8_t     rtkd_unused8_10;         //!< Reserved/unused (padding)
    uint8_t     rtkd_unused8_11;         //!< Reserved/unused (padding)
    uint8_t     error_count;            //!< Pre-filtered observations error count
    uint8_t     error_code;             //!< Pre-filtered observations error code

    uint32_t    rtkd_unused32_2;         //!< Reserved/unused (padding)

    uint8_t     rtkd_unused8_12;         //!< Reserved/unused (padding)
    uint8_t     rtkd_unused8_13;         //!< Reserved/unused (padding)
    uint8_t     warning_count;          //!< Pre-filtered observations warning count
    uint8_t     warning_code;           //!< Pre-filtered observations warning code

    double      double_debug[4];        //!< Generic double-precision debug values, meaning defined by current firmware debug build

    uint8_t     debug[2];               //!< Generic byte debug values, meaning defined by current firmware debug build
    uint8_t     obs_base_unfiltered;    //!< Number of base observations from the receiver (before filtering)
    uint8_t     obs_rover_unfiltered;   //!< Number of rovr observations from the receiver (before filtering)

    uint8_t     rtkd_unused8_14;         //!< Reserved/unused (padding)
    uint8_t     rtkd_unused8_15;         //!< Reserved/unused (padding)
    uint8_t     rtkd_unused8_16;         //!< Reserved/unused (padding)
    uint8_t     obs_unhealthy;          //!< number of sats marked as "unhealthy" by GNSS receiver (nonzero terms in svh)

    uint8_t     obs_rover_relpos;       //!< nu - number of observations input to relpos() before selsat(), rover
    uint8_t     obs_base_relpos;        //!< nr - number of observations input to relpos() before selsat(), base
    uint8_t     obs_pairs_used_float;   //!< number of sat pairs used to compute the float solution
    uint8_t     obs_pairs_used_fixed;   //!< number of sat pairs used to compute the fixed solution

    uint8_t     obs_eph_relpos;         //!< number of sats with ephemeris available (min is 0, max is nu)
    uint8_t     obs_low_snr_rover;      //!< number of sats with low snr at rover and exclude from solution
    uint8_t     obs_low_snr_base;       //!< number of sats with low snr at base and exclude from solution
    uint8_t     rtkd_unused8_17;         //!< Reserved/unused (padding)

    uint8_t     obs_zero_L1_rover;      //!< number of sats with zero L1 pseudorange or phase at rover
    uint8_t     obs_zero_L1_base;       //!< number of sats with zero L1 pseudorange or phase at base
    uint8_t     obs_low_elev;           //!< number of sats with low elevation
    uint8_t     rtkd_unused8_18;         //!< Reserved/unused (padding)

    uint8_t     rtkd_unused8_19;         //!< Reserved/unused (padding)
    uint8_t     rtkd_unused8_20;         //!< Reserved/unused (padding)
    uint8_t     reserved[2];            //!< Reserved/unused (padding)
} rtk_debug_t;

POP_PACK

PUSH_PACK_1

/** @brief (DID_GNSS_RTK_OPT) RTK positioning processing options: positioning mode, ambiguity-resolution (AR) settings, process-noise/measurement-error tuning, and baseline/reset thresholds used to configure the RTKLIB-derived relative-positioning (relpos) engine. */
typedef struct
{
    int32_t mode;                //!< Positioning mode (PMODE_???: e.g. single, DGPS, kinematic, static, moving-baseline)

    int32_t soltype;             //!< Solution type (0 = forward, 1 = backward, 2 = combined forward/backward)

    int32_t nf;                  //!< Number of frequencies used (1 = L1, 2 = L1+L2, 3 = L1+L2+L5)

    int32_t navsys;              //!< Navigation systems bitmask (GPS/GLONASS/Galileo/BeiDou/QZSS/SBAS, SYS_??? bits)

    float   elmin;               //!< Elevation mask angle, satellites below this are excluded (rad)

    int32_t snrmin;               //!< Minimum SNR/Cno for a satellite to be considered for RTK (0.25 dB-Hz units, see obsd_t.SNR)

    int32_t snrrange;            //!< SNR range from the highest-SNR satellite to consider (overrides snrmin if non-zero)

    int32_t modear;              //!< Integer ambiguity resolution (AR) mode (0 = off, 1 = continuous, 2 = instantaneous, 3 = fix-and-hold, 4 = PPP-AR)

    int32_t glomodear;           //!< GLONASS ambiguity resolution mode (0 = off, 1 = on, 2 = auto-calibrate inter-frequency bias, 3 = external calibration)

    int32_t sbsmodear;           //!< SBAS ambiguity resolution mode (0 = off, 1 = on)

    int32_t bdsmodear;           //!< BeiDou ambiguity resolution mode (0 = off, 1 = on)

    int32_t arfilter;            //!< Ambiguity-resolution filtering to reject bad satellites (0 = off, 1 = on)

    int32_t maxout;              //!< Consecutive observation outage count before resetting a satellite's carrier-phase bias

    int32_t maxrej;              //!< Consecutive rejection count before resetting a satellite's carrier-phase bias

    int32_t minlock;             //!< Minimum lock (continuous-tracking) count required before fixing an ambiguity

    int32_t minfixsats;          //!< Minimum number of satellites required to fix integer ambiguities

    int32_t minholdsats;         //!< Minimum number of satellites required to hold fixed integer ambiguities

    int32_t mindropsats;         //!< Minimum number of satellites below which satellites are dropped during ambiguity resolution

    int32_t rcvstds;             //!< Use receiver-reported stdev estimates to scale measurement variances (0 = off, 1 = on)

    int32_t minfix;              //!< Minimum consecutive fix count required before holding an ambiguity

    int32_t armaxiter;           //!< Maximum number of iterations used to resolve integer ambiguities

    int32_t dynamics;            //!< Dynamics model used by the filter (0 = none, 1 = velocity, 2 = acceleration)

    int32_t intpref;             //!< Interpolate reference (base) observations, used for post-mission processing (0 = off, 1 = on)

    int32_t rovpos;              //!< Rover position mode for fixed-position solutions

    int32_t refpos;              //!< Base station position mode for relative-positioning solutions

    float   err[12];             //!< Measurement error factor coefficients (indexed by error model term)

    float   std[3];              //!< Initial-state standard deviations: [0] carrier-phase bias, [1] ionosphere, [2] troposphere

    float   prn[6];              //!< Process-noise standard deviations: [0] bias, [1] iono, [2] trop, [3] horizontal accel, [4] vertical accel, [5] position

    double  sclkstab;            //!< Satellite clock stability (sec/sec, i.e. fractional frequency error)

    float   thresar[8];          //!< Ambiguity-resolution validation thresholds (e.g. ratio test and related AR acceptance criteria)

    float   elmaskar;            //!< Elevation mask for ambiguity resolution of a newly rising satellite (rad)

    float   elmaskhold;          //!< Elevation mask below which a held ambiguity is dropped (rad)

    float   thresslip;           //!< Cycle-slip detection threshold on the geometry-free phase combination (m)

    float   thresdop;            //!< Cycle-slip detection threshold based on Doppler-predicted phase (m)

    float   varholdamb;          //!< Variance assigned to fix-and-hold pseudo-measurements of ambiguity (cycle^2)

    float   gainholdamb;         //!< Gain applied to GLONASS and SBAS satellites when adjusting held ambiguities

    float   maxtdiff;            //!< Maximum allowed time difference between rover and base observations (sec)

    int     fix_reset_base_msgs; //!< Number of base messages without a fix after which satellite biases are reset

    float   maxinno[2];          //!< Innovation (measurement residual) rejection thresholds: [0] carrier-phase, [1] code/pseudorange (m)
    float   maxnis_lo[2];        //!< Lower normalized innovation squared (NIS) rejection thresholds: [0] phase, [1] code
    float   maxnis_hi[2];        //!< Upper normalized innovation squared (NIS) rejection thresholds: [0] phase, [1] code

    double  maxgdop;             //!< Rejection threshold on geometric dilution of precision (GDOP)

    float   baseline[3];         //!< Baseline length constraint: {constrained length, sigma before fix, sigma after fix} (m)
    float   max_baseline_error;  //!< Maximum allowed baseline length error before the solution is considered invalid (m)
    float   reset_baseline_error; //!< Baseline length error above which the filter is reset (m)

    float   max_ubx_error;       //!< Maximum error with respect to the receiver's (u-blox) reported position, triggers a reset if exceeded (m)

    double  ru[3];               //!< Rover position for fixed-position mode {x,y,z} (ECEF, m)

    double  rb[3];                //!< Base station position for relative-positioning mode {x,y,z} (ECEF, m)

    int32_t maxaveep;             //!< Maximum number of epochs used when averaging a position

    int32_t outsingle;            //!< Output a single-point solution on DGPS/float/fixed/PPP outage (0 = off, 1 = on)

    float   velcon[2];            //!< Velocity constraint variance in compassing/moving-baseline mode: {before fix, after fix} (m^2/s^2)

    float   mp_bias_lpf_alpha;    //!< Low-pass-filter alpha for multipath bias estimation; smaller values apply heavier filtering

    float   mp_var_lpf_alpha;     //!< Low-pass-filter alpha for multipath variance estimation; smaller values apply heavier filtering
} prcopt_t;
typedef prcopt_t gnss_rtk_opt_t;

/** @brief Single-satellite, single-epoch raw GNSS observation record (pseudorange/carrier-phase/Doppler/SNR per frequency/signal), following RTKLIB's obsd_t layout. Per-frequency/signal quantities are stored in parallel arrays of length NFREQ+NEXOBS, indexed identically across SNR/LLI/code/qualL/qualP/L/P/D. */
typedef struct PACKED
{
    gtime_t time;                //!< Receiver local time of the observation, approximately aligned to GPS time system (GPST)

    uint8_t sat;                 //!< Satellite number in RTKLIB notation. GPS: 1-32, GLONASS: 33-59, Galileo: 60-89, SBAS: 90-95

    uint8_t rcv;                 //!< Receiver number (identifies rover vs. base in a differential pair)

    uint8_t SNR[NFREQ+NEXOBS];   //!< Cno, carrier-to-noise density ratio / signal strength (0.25 dB-Hz units), indexed by frequency/signal

    uint8_t LLI[NFREQ+NEXOBS];   //!< Loss-of-Lock Indicator, non-zero only when carrier-phase is valid (L > 0): bit1 = loss-of-lock, bit2 = half-cycle-ambiguity-invalid; indexed by frequency/signal

    uint8_t code[NFREQ+NEXOBS];  //!< RINEX/RTKLIB signal code indicator, e.g. CODE_L1C (1) = L1 C/A, G1 C/A, E1C (GPS/GLO/GAL/QZS/SBS); CODE_L1X (12) = E1 B+C, L1C(D+P) (GAL/QZS); CODE_L1I (47) = B1I (BeiDou); indexed by frequency/signal

    uint8_t qualL[NFREQ+NEXOBS]; //!< Estimated carrier-phase measurement standard deviation (0.004-cycle units), zero means invalid; indexed by frequency/signal

    uint8_t qualP[NFREQ+NEXOBS]; //!< Estimated pseudorange measurement standard deviation (0.01 m units), zero means invalid; indexed by frequency/signal

    /** reserved, for alignment */
//if NFREQ == 1
    uint8_t reserved;            //!< Reserved/unused (padding for alignment)
//#elif NFREQ == 3
//    uint8_t reserved[3];
//#endif

    double  L[NFREQ+NEXOBS];     //!< Carrier-phase observation (cycles), indexed by frequency/signal, see code[]. Initial ambiguity is set to an approximate value so the phase magnitude tracks the pseudorange; clock resets are applied to both phase and code per the RINEX specification

    double  P[NFREQ+NEXOBS];     //!< Pseudorange observation (m), indexed by frequency/signal, see code[]. GLONASS inter-frequency channel delays are compensated using an internal calibration table

    float   D[NFREQ+NEXOBS];     //!< Doppler observation, positive sign for an approaching satellite (Hz), indexed by frequency/signal, see code[]
} obsd_t;

#define GNSS_RAW_MESSAGE_BUF_SIZE               1000
#define MAX_OBSERVATION_COUNT_IN_RTK_MESSAGE    (GNSS_RAW_MESSAGE_BUF_SIZE / sizeof(obsd_t))

/** @brief Observation data buffer: a growable array of per-satellite obsd_t observation records for one epoch, following RTKLIB's obs_t container convention. */
typedef struct
{
    uint32_t    n;               //!< Number of observation slots currently used (valid entries in data[])

    uint32_t    nmax;            //!< Number of observation slots allocated in data[]

    obsd_t*     data;            //!< Pointer to the observation data buffer, an array of obsd_t records
} obs_t;

/** @brief Broadcast ephemeris (Keplerian orbital elements + clock terms) for GPS, Galileo, BeiDou, QZSS, IRNSS, and SBAS satellites (non-GLONASS), following RTKLIB's eph_t layout. Field names/units match the RINEX navigation-message and ICD conventions for these orbital elements. */
typedef struct
{
    int32_t sat;                 //!< Satellite number in RTKLIB notation. GPS: 1-32, GLONASS: 33-59, Galileo: 60-89, SBAS: 90-95

    int32_t iode;                //!< IODE, Issue Of Data Ephemeris (ephemeris data-set version number)

    int32_t iodc;                //!< IODC, Issue Of Data Clock (clock data-set version number)

    int32_t sva;                 //!< SV accuracy (URA index), see IS-GPS-200/IRN-IS-200H p.97

    int32_t svh;                 //!< SV health for GPS/QZS (0 = ok)

    int32_t week;                //!< Ephemeris reference week number: GPS week for GPS/QZS, Galileo week (GST week) for GAL

    int32_t code;                //!< GPS/QZS: code on L2 (00 = invalid, 01 = P-code on, 11 = C/A-code on, 11 = invalid). GAL/CMP (BeiDou): data source indicator

    int32_t flag;                //!< GPS/QZS: L2 P-code data flag (nonzero indicates the NAV data stream is commanded OFF on the P-code of the L2 in-phase component). CMP (BeiDou): nav message type

    gtime_t toe;                 //!< Time Of Ephemeris: ephemeris reference epoch (GPST)

    gtime_t toc;                 //!< Clock data reference time (GPST), see IS-GPS-200 20.3.4.5

    gtime_t ttr;                 //!< Transmission time of the message, T_trans (GPST)

    double  A;                   //!< Orbit semi-major axis (m)

    double  e;                   //!< Orbit eccentricity (dimensionless)

    double  i0;                  //!< Orbit inclination angle at reference time (rad)

    double  OMG0;                //!< Longitude of ascending node of orbit plane at the weekly epoch (rad)

    double  omg;                 //!< Argument of perigee (rad)

    double  M0;                  //!< Mean anomaly at reference time (rad)

    double  deln;                //!< Mean motion difference from computed value, delta-n (rad/s)

    double  OMGd;                //!< Rate of right ascension / longitude of ascending node, OMEGA-dot (rad/s)

    double  idot;                //!< Rate of inclination angle, i-dot (rad/s)

    double  crc;                 //!< Amplitude of the cosine harmonic correction term to the orbit radius (m)

    double  crs;                 //!< Amplitude of the sine harmonic correction term to the orbit radius (m)

    double  cuc;                 //!< Amplitude of the cosine harmonic correction term to the argument of latitude (rad)

    double  cus;                 //!< Amplitude of the sine harmonic correction term to the argument of latitude (rad)

    double  cic;                 //!< Amplitude of the cosine harmonic correction term to the angle of inclination (rad)

    double  cis;                 //!< Amplitude of the sine harmonic correction term to the angle of inclination (rad)

    double  toes;                //!< Time Of Ephemeris in seconds within the week (s), the double-precision counterpart of toe above. toe is computed as eph->toe = gst2time(week, eph->toes); this value is the ephemeris expiration reference and is generally ~2 hours ahead of the current time

    double  fit;                 //!< Curve-fit interval (h) (0 = 4 hours, 1 = greater than 4 hours)

    double  f0;                  //!< SV clock bias, af0 (s)

    double  f1;                  //!< SV clock drift, af1 (s/s, dimensionless)

    double  f2;                  //!< SV clock drift rate, af2 (s/s^2)

    double  tgd[4];               //!< Group delay parameters. GPS/QZS: tgd[0] = TGD (IRN-IS-200H p.103). Galileo: tgd[0] = BGD E5a/E1, tgd[1] = BGD E5b/E1. BeiDou: tgd[0] = BGD1, tgd[1] = BGD2

    double  Adot;                 //!< Semi-major axis rate, A-dot, for CNAV messages; not used

    double  ndot;                 //!< First derivative of mean motion n (equivalently second derivative of mean anomaly M), n-dot, for CNAV messages (rad/s^2); not used
} eph_t;

/** @brief GLONASS broadcast ephemeris, following RTKLIB's geph_t layout. Unlike GPS-style Keplerian eph_t, GLONASS broadcasts a direct ECEF position/velocity/acceleration state vector (numerically integrated by the receiver/user) rather than orbital elements. */
typedef struct
{
    int32_t sat;                 //!< Satellite number in RTKLIB notation. GPS: 1-32, GLONASS: 33-59, Galileo: 60-89, SBAS: 90-95

    int32_t iode;                //!< IODE, derived from bits 0-6 of the tb (time interval index) field

    int32_t frq;                 //!< Satellite frequency channel number (GLONASS FDMA slot, -7..+13)

    int32_t svh;                 //!< Satellite health flag

    int32_t sva;                 //!< Satellite accuracy (URA-like indicator)

    int32_t age;                 //!< Satellite age of operation (days)

    gtime_t toe;                 //!< Ephemeris reference epoch within the week, GPS time system (GPST)

    gtime_t tof;                 //!< Message frame transmission time, GPS time system (GPST)

    double  pos[3];              //!< Satellite position at toe (ECEF) (m)

    double  vel[3];              //!< Satellite velocity at toe (ECEF) (m/s)

    double  acc[3];              //!< Satellite (luni-solar) acceleration at toe (ECEF) (m/s^2)

    double  taun;                //!< SV clock bias, -tau_n (s)

    double  gamn;                //!< Relative frequency bias, gamma_n (dimensionless, s/s)

    double  dtaun;               //!< Time delay between the L1 and L2 signal transmissions (s)
} geph_t;

/** @brief Raw SBAS (e.g. WAAS/EGNOS/MSAS) broadcast message, following RTKLIB's sbsmsg_t layout. Carries one 250-bit-symbol/226-bit-payload SBAS message frame along with its reception time. */
typedef struct
{
    int32_t week;                //!< GPS week number of message reception

    int32_t tow;                 //!< Time of week of message reception (s)

    int32_t prn;                 //!< SBAS satellite PRN number

    uint8_t msg[29];             //!< Raw SBAS message payload (226 bits), zero-padded to 29 bytes

    uint8_t reserved[3];         //!< Reserved/unused (padding for alignment)
} sbsmsg_t;

/** @brief Station/receiver metadata (antenna offset and reference position), following RTKLIB's sta_t layout. Used to describe a base or rover station for RTCM/RINEX-style differential positioning. */
typedef struct
{
    int32_t deltype;             //!< Antenna delta (offset) type: 0 = e/n/u (east/north/up), 1 = x/y/z (ECEF)

    double  pos[3];              //!< Station reference position (ECEF) (m)

    double  del[3];              //!< Antenna position delta from the station reference position, interpreted per deltype: e/n/u or x/y/z (m)

    double  hgt;                 //!< Antenna height above the marker/reference point (m)

    int32_t stationId;           //!< Station identifier (e.g. RTCM/RINEX station ID)
} sta_t;

/** @brief GNSS almanac data: reduced-precision, long-validity orbital and clock parameters for a satellite, following RTKLIB's alm_t layout. Used for coarse orbit prediction (e.g. acquisition assistance) rather than precise positioning. */
typedef struct
{
    int32_t sat;                 //!< Satellite number in RTKLIB notation. GPS: 1-32, GLONASS: 33-59, Galileo: 60-89, SBAS: 90-95

    int32_t svh;                 //!< SV health (0 = ok)

    int32_t svconf;              //!< Anti-spoofing (AS) and SV configuration flags

    int32_t week;                //!< Almanac reference week number: GPS week for GPS/QZS, Galileo week (GST week) for GAL

    gtime_t toa;                 //!< Time Of Almanac, almanac reference epoch (gtime_t)

    double  A;                   //!< SV orbit semi-major axis (m)

    double  e;                   //!< SV orbit eccentricity (dimensionless)

    double  i0;                  //!< SV orbit inclination angle at reference time (rad)

    double  OMG0;                //!< SV orbit longitude of ascending node at the weekly epoch (rad)

    double  omg;                 //!< SV orbit argument of perigee (rad)

    double  M0;                  //!< SV orbit mean anomaly at reference time (rad)

    double  OMGd;                //!< SV orbit rate of right ascension, OMEGA-dot (rad/s)

    double  toas;                //!< Time Of Almanac in seconds within the week (s), double-precision counterpart of toa above

    double  f0;                  //!< SV clock bias, af0 (s)

    double  f1;                  //!< SV clock drift, af1 (s/s, dimensionless)
} alm_t;

/** @brief Combined ionospheric correction model, UTC time-conversion parameters, and almanac data for the supported GNSS constellations, following RTKLIB's convention for broadcast navigation-message auxiliary data (e.g. as decoded from GPS/BeiDou subframe 4/5 or Galileo/QZSS equivalents). */
typedef struct
{
    double  ion_gps[8];     //!< GPS (Klobuchar) ionospheric model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
    double  ion_gal[4];     //!< Galileo (NeQuick-G) ionospheric model parameters {ai0,ai1,ai2,0}
    double  ion_qzs[8];     //!< QZSS ionospheric model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
    double  ion_cmp[8];     //!< BeiDou ionospheric model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
    double  ion_irn[8];     //!< IRNSS ionospheric model parameters {a0,a1,a2,a3,b0,b1,b2,b3}

    double  utc_gps[4];     //!< GPS delta-UTC parameters {A0,A1,T,W} (bias, drift, reference time of week, reference week)
    double  utc_glo[4];     //!< GLONASS UTC/GPS time-offset parameters
    double  utc_gal[4];     //!< Galileo UTC/GPS time-offset parameters
    double  utc_qzs[4];     //!< QZSS UTC/GPS time-offset parameters
    double  utc_cmp[4];     //!< BeiDou UTC time-offset parameters
    double  utc_irn[4];     //!< IRNSS UTC time-offset parameters
    double  utc_sbs[4];     //!< SBAS UTC time-offset parameters

    int32_t leaps;          //!< Leap second count / current GPS-UTC offset (s)

    alm_t   alm;            //!< Almanac data for the associated satellite
} ion_model_utc_alm_t;

/**
 * @brief RTK solution status, reported as the value decoded from the fix-type field of an
 * eGnssStatus status word (e.g. gnss_pos_t.status, gnss_rtk_rel_t.status). Ordered roughly from
 * lowest to highest accuracy: single is a standalone (non-differential) solution, dgps/sbas are
 * code-based differential corrections, float is a carrier-phase RTK solution with unresolved
 * (floating) integer ambiguities, and fix is a carrier-phase RTK solution with resolved integer
 * ambiguities (highest accuracy).
 */
enum eRtkSolStatus
{
    rtk_solution_status_none    = 0,   //!< No RTK solution status available
    rtk_solution_status_fix     = 1,   //!< RTK fix: carrier-phase solution with resolved (fixed) integer ambiguities (highest accuracy, typically < 6cm horizontal)
    rtk_solution_status_float   = 2,   //!< RTK float: carrier-phase solution with unresolved (floating) integer ambiguities
    rtk_solution_status_sbas    = 3,   //!< SBAS (satellite-based augmentation system) corrected solution
    rtk_solution_status_dgps    = 4,   //!< DGPS (code-based differential GPS) corrected solution
    rtk_solution_status_single  = 5    //!< Single point (standalone, non-differential) solution
};

/**
 * @brief (DID_GNSS1_RTK_POS_REL, DID_GNSS2_RTK_CMP_REL) RTK / dual-GNSS compassing relative
 * position and heading. For DID_GNSS1_RTK_POS_REL this is the vector from the RTK base station
 * to the GNSS1 rover antenna; for DID_GNSS2_RTK_CMP_REL (dual-GNSS moving-base compassing) this
 * is instead the vector between the two onboard antennas, used to derive a magnetometer-free
 * heading.
 */
typedef struct PACKED
{
    uint32_t                timeOfWeekMs;           //!< (ms) GPS time of week (since Sunday morning)

    float                   differentialAge;        //!< (s) Age of differential corrections

    float                   arRatio;                //!< Ambiguity resolution ratio factor for validation (unitless; higher indicates greater confidence the fixed integer ambiguity is correct)

    float                   baseToRoverVector[3];   //!< (m) Vector from base to rover GNSS antennas {x,y,z} in ECEF.  Precision positioning mode: RTK station (base) to GNSS1 (rover).  Compassing mode: GNSS1 (base) to GNSS2 (rover)

    float                   baseToRoverDistance;    //!< (m) Distance from base to rover GNSS antennas (baseline length)

    float                   baseToRoverHeading;     //!< (rad) Heading of baseToRoverVector in the local tangent (NED) plane

    float                   baseToRoverHeadingAcc;  //!< (rad) Accuracy (standard deviation) of baseToRoverHeading

    uint32_t                status;                 //!< GNSS status (see eGnssStatus): [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags, NMEA input flag

    // float                   covEcefPacked[6];       //!< RTK solution covariance in ECEF packed as [Pxx, Pyy, Pzz, Pxy, Pyz, Pzx], in meters^2

} gnss_rtk_rel_t;

/**
 * @brief (DID_GNSS1_RTK_POS_MISC, DID_GNSS2_RTK_CMP_MISC) RTK engine diagnostics and per-constellation
 * observation/ephemeris element counters accompanying gnss_rtk_rel_t. Requires a little-endian
 * host CPU.
 */
typedef struct PACKED
{
    uint32_t                timeOfWeekMs;               //!< GPS time of week (since Sunday morning), in milliseconds

    float                   accuracyPos[3];             //!< Accuracy: estimated standard deviations of the position solution assuming the a priori error model and error parameters used by the positioning options. []: {ECEF x,y,z} or {north,east,down}, in meters

    float                   accuracyCov[3];             //!< Accuracy: estimated covariance of the position solution. []: absolute value of the square root of the estimated covariance {NE, EU, UN}, in meters

    float                   arThreshold;                //!< Ambiguity resolution threshold for validation (unitless)

    float                   gDop;                       //!< Geometric dilution of precision (unitless)

    float                   hDop;                       //!< Horizontal dilution of precision (unitless)

    float                   vDop;                       //!< Vertical dilution of precision (unitless)

    double                  baseLla[3];                 //!< Base station position: latitude, longitude, height (deg, deg, m)

    uint32_t                cycleSlipCount;             //!< Cycle slip counter

    uint32_t                roverGpsObservationCount;      //!< Rover GPS observation element counter

    uint32_t                baseGpsObservationCount;       //!< Base station GPS observation element counter

    uint32_t                roverGlonassObservationCount;  //!< Rover GLONASS observation element counter

    uint32_t                baseGlonassObservationCount;   //!< Base station GLONASS observation element counter

    uint32_t                roverGalileoObservationCount;  //!< Rover Galileo observation element counter

    uint32_t                baseGalileoObservationCount;   //!< Base station Galileo observation element counter

    uint32_t                roverBeidouObservationCount;   //!< Rover BeiDou observation element counter

    uint32_t                baseBeidouObservationCount;    //!< Base station BeiDou observation element counter

    uint32_t                roverQzsObservationCount;      //!< Rover QZSS observation element counter

    uint32_t                baseQzsObservationCount;       //!< Base station QZSS observation element counter

    uint32_t                roverGpsEphemerisCount;        //!< Rover GPS ephemeris element counter

    uint32_t                baseGpsEphemerisCount;         //!< Base station GPS ephemeris element counter

    uint32_t                roverGlonassEphemerisCount;    //!< Rover GLONASS ephemeris element counter

    uint32_t                baseGlonassEphemerisCount;     //!< Base station GLONASS ephemeris element counter

    uint32_t                roverGalileoEphemerisCount;    //!< Rover Galileo ephemeris element counter

    uint32_t                baseGalileoEphemerisCount;     //!< Base station Galileo ephemeris element counter

    uint32_t                roverBeidouEphemerisCount;     //!< Rover BeiDou ephemeris element counter

    uint32_t                baseBeidouEphemerisCount;      //!< Base station BeiDou ephemeris element counter

    uint32_t                roverQzsEphemerisCount;        //!< Rover QZSS ephemeris element counter

    uint32_t                baseQzsEphemerisCount;         //!< Base station QZSS ephemeris element counter

    uint32_t                roverSbasCount;             //!< Rover SBAS element counter

    uint32_t                baseSbasCount;              //!< Base station SBAS element counter

    uint32_t                baseAntennaCount;           //!< Base station antenna position element counter

    uint32_t                ionUtcAlmCount;             //!< Ionosphere model, UTC, and almanac element counter

    uint32_t                correctionChecksumFailures; //!< Number of checksum failures from received corrections

    uint32_t                timeToFirstFixMs;           //!< Time to first RTK fix, in milliseconds

} gnss_rtk_misc_t;

/**
 * @brief Identifies which member of the uGnssRawData union in gnss_raw_t is populated, mapping
 * onto the RTKLIB-derived structures documented above (obsd_t, eph_t, geph_t, sbsmsg_t, sta_t,
 * ion_model_utc_alm_t). Used for DID_GNSS_BASE_RAW, DID_GNSS1_RAW, and DID_GNSS2_RAW.
 */
enum eRawDataType
{
    raw_data_type_observation                   = 1,    //!< Satellite pseudorange/carrier-phase observation data (obsd_t[]; see uGnssRawData.obs)
    raw_data_type_ephemeris                     = 2,    //!< Broadcast ephemeris for non-GLONASS constellations - GPS, Galileo, BeiDou, QZSS (eph_t; see uGnssRawData.eph)
    raw_data_type_glonass_ephemeris             = 3,    //!< Broadcast GLONASS ephemeris (geph_t; see uGnssRawData.gloEph)
    raw_data_type_sbas                          = 4,    //!< SBAS (satellite-based augmentation system) message (sbsmsg_t; see uGnssRawData.sbas)
    raw_data_type_base_station_antenna_position = 5,    //!< Base station position / antenna information (sta_t; see uGnssRawData.sta)
    raw_data_type_ionosphere_model_utc_alm      = 6,    //!< Ionosphere model, UTC, and almanac data (ion_model_utc_alm_t; see uGnssRawData.ion)
    raw_data_type_rtk_solution                  = 123   //!< RTK solution / related statistics (gnss_rtk_misc_t)
};

/**
 * @brief Raw GNSS data payload carried in gnss_raw_t.data. Exactly one member is valid at a time,
 * selected by the enclosing gnss_raw_t.dataType field (see eRawDataType); obs, eph, gloEph, sbas,
 * sta, and ion mirror the RTKLIB-derived structures documented above (obsd_t, eph_t, geph_t,
 * sbsmsg_t, sta_t, and ion_model_utc_alm_t respectively).
 */
typedef union PACKED
{
    obsd_t              obs[MAX_OBSERVATION_COUNT_IN_RTK_MESSAGE];  //!< Satellite observation data, valid when dataType == raw_data_type_observation

    eph_t               eph;                                       //!< Satellite non-GLONASS ephemeris data (GPS, Galileo, Beidou, QZSS), valid when dataType == raw_data_type_ephemeris

    geph_t              gloEph;                                    //!< Satellite GLONASS ephemeris data, valid when dataType == raw_data_type_glonass_ephemeris

    sbsmsg_t            sbas;                                      //!< Satellite-Based Augmentation Systems (SBAS) data, valid when dataType == raw_data_type_sbas

    sta_t               sta;                                       //!< Base station information (base position, antenna position, antenna height, etc.), valid when dataType == raw_data_type_base_station_antenna_position

    ion_model_utc_alm_t ion;                                       //!< Ionosphere model and UTC parameters, valid when dataType == raw_data_type_ionosphere_model_utc_alm

    uint8_t             buf[GNSS_RAW_MESSAGE_BUF_SIZE];             //!< Byte buffer providing untyped access to the same union storage

} uGnssRawData;

/**
 * @brief (DID_GNSS_BASE_RAW, DID_GNSS1_RAW, DID_GNSS2_RAW) Wrapper for raw GNSS receiver data
 * (observations, ephemeris, SBAS messages, base station info, ionosphere/UTC/almanac data) used
 * to feed the onboard or an external RTK engine. Requires a little-endian host CPU. The contents
 * of the data union vary per message and are determined by the dataType field; RTK positioning
 * or RTK compassing must be enabled to stream this message.
 */
typedef struct PACKED
{
    uint8_t         receiverIndex;  //!< Source receiver: 1=RECEIVER_INDEX_GNSS1, 2=RECEIVER_INDEX_EXTERNAL_BASE, 3=RECEIVER_INDEX_GNSS2

    uint8_t         dataType;       //!< Type of data in the data union (see eRawDataType): 1=observations, 2=ephemeris, 3=glonassEphemeris, 4=SBAS, 5=baseAntenna, 6=IonosphereModel

    uint8_t         obsCount;       //!< Number of observations (obsd_t elements) present in data.obs when dataType==1 (raw_data_type_observation)

    uint8_t         reserved;       //!< Reserved for alignment / future use

    uGnssRawData    data;           //!< Raw data payload; interpret based on dataType (see eRawDataType)
} gnss_raw_t;

/**
 * @brief (DID_GNSS1_TIMEPULSE, DID_GNSS2_TIMEPULSE) GNSS PPS (pulse-per-second) time
 * synchronization status, correlating the local MCU clock to GPS time of week via the receiver's
 * timepulse (PPS) output.
 */
typedef struct
{
    double      towOffset;                //!< Week seconds offset from MCU to GPS time, in seconds

    double      towGps;                   //!< Week seconds for next timepulse (from start of GPS week), in seconds

    double      timeMcu;                  //!< Local MCU week seconds, in seconds

    uint32_t    msgTimeMs;                //!< Local timestamp of TIM-TP message used to validate timepulse, in milliseconds

    uint32_t    plsTimeMs;                //!< Local timestamp of time sync pulse external interrupt used to validate timepulse, in milliseconds

    uint8_t     syncCount;                //!< Counter for successful timesync events

    uint8_t     badPulseAgeCount;         //!< Counter for failed timesync events

    uint8_t     ppsInterruptReinitCount;  //!< Counter for GNSS PPS interrupt re-initialization

    uint8_t     plsCount;                 //!< Counter of GNSS PPS via GPIO, not interrupt

    uint32_t    lastSyncTimeMs;           //!< Local timestamp of last valid PPS sync, in milliseconds

    uint32_t    sinceLastSyncTimeMs;      //!< Time since last valid PPS sync, in milliseconds

} gnss_timepulse_t;

/**
 * @brief (DID_DIAGNOSTIC_MESSAGE) Null-terminated diagnostic/debug text message emitted by the
 * device firmware.
 */
typedef struct
{
    uint32_t    timeOfWeekMs;    //!< GPS time of week (since Sunday morning), in milliseconds

    uint32_t    messageLength;   //!< Message length, including null terminator, in bytes

    char        message[256];   //!< Message data (null-terminated string); max size of message is 256 bytes
} diag_msg_t;

/**
 * @brief State machine for survey_in_t.state (DID_SURVEY_IN), used to determine a stationary RTK
 * base station's precise position by averaging its GNSS solution over time. The
 * SURVEY_IN_STATE_START_* values are commands written by the host to begin a survey requiring a
 * given minimum fix quality; SURVEY_IN_STATE_RUNNING_*, _SAVE_POS, and _DONE are status values
 * reported back by the device.
 */
enum eSurveyInStatus
{
    // default state
    SURVEY_IN_STATE_OFF             = 0,    //!< No survey in progress

    // commands
    SURVEY_IN_STATE_CANCEL          = 1,    //!< Command: cancel the current survey
    SURVEY_IN_STATE_START_3D        = 2,    //!< Command: start a survey requiring only a 3D fix
    SURVEY_IN_STATE_START_FLOAT     = 3,    //!< Command: start a survey requiring at least an RTK float fix
    SURVEY_IN_STATE_START_FIX       = 4,    //!< Command: start a survey requiring an RTK fixed-ambiguity fix

    // status
    SURVEY_IN_STATE_RUNNING_3D      = 8,    //!< Status: survey running, current fix is 3D
    SURVEY_IN_STATE_RUNNING_FLOAT   = 9,    //!< Status: survey running, current fix is RTK float
    SURVEY_IN_STATE_RUNNING_FIX     = 10,   //!< Status: survey running, current fix is RTK fixed-ambiguity
    SURVEY_IN_STATE_SAVE_POS        = 19,   //!< Status: survey complete, saving surveyed position
    SURVEY_IN_STATE_DONE            = 20    //!< Status: survey complete, surveyed position saved and available in lla
};

/**
 * @brief (DID_SURVEY_IN) Status/control for an RTK base station position survey, used to
 * determine a stationary base's precise position (see eSurveyInStatus) before it starts
 * broadcasting differential corrections. Base correction output cannot run during a survey and
 * is automatically disabled if a survey is started.
 */
typedef struct
{
    uint32_t    state;           //!< State of current survey (see eSurveyInStatus)

    uint32_t    maxDurationSec;  //!< Maximum duration the survey will run, in seconds, if minAccuracy is not first achieved (ignored if 0)

    float       minAccuracy;     //!< Required horizontal accuracy for the survey to complete before maxDurationSec elapses, in meters (ignored if 0)

    uint32_t    elapsedTimeSec;  //!< Elapsed time of the survey, in seconds

    float       hAccuracy;       //!< Approximate horizontal accuracy of the survey's current position estimate, in meters

    double      lla[3];          //!< Current surveyed position: latitude, longitude, altitude (deg, deg, m)
} survey_in_t;


//////////////////////////////////////////////////////////////////////////
//  GPX
//////////////////////////////////////////////////////////////////////////

/** @brief GPX system configuration bits, used with DID_GPX_FLASH_CFG::sysCfgBits (gpx_flash_cfg_t.sysCfgBits). Controls VCC_RF power switching and the brownout reset threshold. GPX_SYS_CFG_BITS_BOR_LEVEL_x are raw 2-bit codes (0-3), not pre-shifted bit flags -- shift the desired code left by GPX_SYS_CFG_BITS_BOR_THRESHOLD_OFFSET and mask with GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK before ORing it into sysCfgBits. */
enum eGpxSysConfigBits
{
    GPX_SYS_CFG_BITS_DISABLE_VCC_RF         = 0x00000001,  //!< Disable (tri-state) VCC_RF (GPX pin 16) output supplied via VAUX (GPX pin 40)

    GPX_SYS_CFG_BITS_BOR_LEVEL_0            = 0x0,  //!< Brownout reset threshold voltage level: 1.65 - 1.75 V (default)
    GPX_SYS_CFG_BITS_BOR_LEVEL_1            = 0x1,  //!< Brownout reset threshold voltage level: 2.0 - 2.1 V
    GPX_SYS_CFG_BITS_BOR_LEVEL_2            = 0x2,  //!< Brownout reset threshold voltage level: 2.25 - 2.35 V
    GPX_SYS_CFG_BITS_BOR_LEVEL_3            = 0x3,  //!< Brownout reset threshold voltage level: 2.5 - 2.6 V
    GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK     = (int)0x00C00000,  //!< Mask for the brownout reset threshold field, once shifted into position within sysCfgBits
    GPX_SYS_CFG_BITS_BOR_THRESHOLD_OFFSET   = 22,               //!< Bit offset of GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK within sysCfgBits
};

/**
 * @brief (DID_GPX_FLASH_CFG) GPX flash-backed configuration -- the GPX's persistent config, analogous to nvm_flash_cfg_t (DID_FLASH_CONFIG) on the IMX. Stored in the GPX's onboard flash and restored on boot; covers serial port baud rates, GNSS receiver timing/filtering, RTK configuration, and system configuration bits. Fields shared in purpose with nvm_flash_cfg_t (gnssSatSigConst, dynamicModel, gnssTimeUserDelay, gnssMinimumElevation, RTKCfgBits, gnssCn0Minimum, gnssCn0DynMinOffset, refLla) reuse the same enum/bit definitions and semantics as their IMX counterparts.
 * IMPORTANT: These fields should not be deleted, they can be deprecated and marked as reserved,
 * or new fields added to the end.
 * NOTE: The key value must be incremented to ensure the defaults are restored anytime the fields
 * change or the default values change.  Default changes should be noted in the changelog.
*/
typedef struct
{
    uint32_t                size;                   //!< Size of this struct

    uint32_t                checksum;               //!< Checksum, excluding size and checksum. 0xFFFFFFFF is invalid.

    uint32_t                key;                    //!< Manufacturer method for restoring flash defaults

    uint32_t                ser0BaudRate;            //!< (bps) Serial port 0 baud rate

    uint32_t                ser1BaudRate;            //!< (bps) Serial port 1 baud rate

    uint32_t                ser2BaudRate;            //!< (bps) Serial port 2 baud rate

    uint32_t                startupGnssDtMs;         //!< (ms) GNSS measurement (system input data) update period set on startup. 200ms minimum (5Hz max).

    float                   gnss1AntOffset[3];       //!< (m) X,Y,Z offset in Sensor Frame to GNSS 1 antenna

    float                   gnss2AntOffset[3];       //!< (m) X,Y,Z offset in Sensor Frame to GNSS 2 antenna

    uint16_t                gnssSatSigConst;         //!< Satellite system constellation used in GNSS solution (see eGnssSatSigConst). 0x0003=GPS, 0x000C=QZSS, 0x0030=Galileo, 0x00C0=Beidou, 0x0300=GLONASS, 0x1000=SBAS

    uint8_t                 dynamicModel;            //!< Dynamic platform model (see eDynamicModel). Options are: 0=PORTABLE, 1=FIXED POSITION, 2=STATIONARY, 3=PEDESTRIAN, 4=GROUND VEHICLE, 5=SEA, 6=AIRBORNE_1G, 7=AIRBORNE_2G, 8=AIRBORNE_4G, 9=WRIST. Used to balance noise and performance characteristics of the system. The dynamics selected here must be at least as fast as your system or you experience accuracy error. This is tied to the GNSS position estimation model and intended in the future to be incorporated into the INS position model.

    uint8_t                 debug;                   //!< Debug

    uint32_t                gnssTimeSyncPeriodMs;    //!< (ms) Time between GPS time synchronization pulses. Requires reboot to take effect.

    float                   gnssTimeUserDelay;       //!< (sec) User defined delay for GPS time. This parameter can be used to account for GNSS antenna cable delay.

    float                   gnssMinimumElevation;    //!< (rad) Minimum elevation of a satellite above the horizon to be used in the solution. Low elevation satellites may provide degraded accuracy, due to the long signal path through the atmosphere.

    uint32_t                RTKCfgBits;              //!< RTK configuration bits (see eRTKConfigBits).

    uint8_t                 gnssCn0Minimum;          //!< (dBHz) GNSS CN0 absolute minimum threshold for signals. Used to filter signals in RTK solution.

    uint8_t                 gnssCn0DynMinOffset;     //!< (dBHz) GNSS CN0 dynamic minimum threshold offset below max CN0 across all satellites. Used to filter signals used in RTK solution. To disable, set gnssCn0DynMinOffset to zero and increase gnssCn0Minimum.

    uint8_t                 reserved1[2];            //!< Reserved

    uint32_t                sysCfgBits;              //!< System configuration bits (see eGpxSysConfigBits).

    uint32_t                reserved2;               //!< Reserved

    double                  refLla[3];               //!< (deg, deg, m) Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations

} gpx_flash_cfg_t;

/** @brief (DID_GPX_STATUS) GPX status flags, reported in gpx_status_t.status. Packs a communications parse-error counter, per-port Rx-traffic-not-detected flags, an update-confirmed flag, and a general fault region (bits 16-31) covering RTK/GNSS/RTOS/DMA fault flags plus a fatal-fault sub-field (GPX_STATUS_FATAL_MASK) that reports the specific cause the last time a critical CPU reset occurred. GPX_STATUS_FATAL_RESET_LOW_POW..GPX_STATUS_FATAL_UNKNOWN are raw cause codes, not individual bit flags -- shift the code left by GPX_STATUS_FATAL_OFFSET and mask with GPX_STATUS_FATAL_MASK to read/write the sub-field. */
enum eGpxStatus
{
    GPX_STATUS_COM_PARSE_ERR_COUNT_MASK         = (int)0x0000000F,  //!< Mask for the communications parse error count field
    GPX_STATUS_COM_PARSE_ERR_COUNT_OFFSET       = 0,                //!< Bit offset of GPX_STATUS_COM_PARSE_ERR_COUNT_MASK within status
#define GPX_STATUS_COM_PARSE_ERROR_COUNT(gpxStatus) ((gpxStatus&GPX_STATUS_COM_PARSE_ERR_COUNT_MASK)>>GPX_STATUS_COM_PARSE_ERR_COUNT_OFFSET)  //!< Extract the communications parse error count from a status value

    GPX_STATUS_COM0_RX_TRAFFIC_NOT_DETECTED     = (int)0x00000010,  //!< Rx communications not detected on serial port 0 in the last 30 seconds
    GPX_STATUS_COM1_RX_TRAFFIC_NOT_DETECTED     = (int)0x00000020,  //!< Rx communications not detected on serial port 1 in the last 30 seconds
    GPX_STATUS_COM2_RX_TRAFFIC_NOT_DETECTED     = (int)0x00000040,  //!< Rx communications not detected on serial port 2 in the last 30 seconds
    GPX_STATUS_USB_RX_TRAFFIC_NOT_DETECTED      = (int)0x00000080,  //!< Rx communications not detected on USB in the last 30 seconds

    GPX_STATUS_UPDATE_CONFIRMED                 = (int)0x00000100,  //!< Update confirmed

    GPX_STATUS_GENERAL_FAULT_MASK               = (int)0xFFFF0000,  //!< Mask covering all general fault bits (RTK/GNSS/RTOS/DMA faults and the fatal sub-field)

    GPX_STATUS_FAULT_RTK_QUEUE_LIMITED          = (int)0x00010000,  //!< RTK buffer filled causing data loss

    GPX_STATUS_FAULT_GNSS_RCVR_TIME             = (int)0x00100000,  //!< GNSS receiver time fault
    GPX_STATUS_FAULT_RTOS_TASK_PERIOD_OVERRUN   = (int)0x00200000,  //!< RTOS task period overrun
    GPX_STATUS_FAULT_DMA                        = (int)0x00800000,  //!< DMA fault detected

    GPX_STATUS_FATAL_MASK                       = (int)0x1F000000,  //!< Mask for the fatal-fault cause sub-field (critical failure resulting in CPU reset), once shifted into position within status
    GPX_STATUS_FATAL_OFFSET                     = 24,                //!< Bit offset of GPX_STATUS_FATAL_MASK within status
    GPX_STATUS_FATAL_RESET_LOW_POW              = (int)1,   //!< Fatal cause: reset from low power
    GPX_STATUS_FATAL_RESET_BROWN                = (int)2,   //!< Fatal cause: reset from brown out
    GPX_STATUS_FATAL_RESET_WATCHDOG             = (int)3,   //!< Fatal cause: reset from watchdog
    GPX_STATUS_FATAL_CPU_EXCEPTION              = (int)4,   //!< Fatal cause: CPU exception
    GPX_STATUS_FATAL_UNHANDLED_INTERRUPT        = (int)5,   //!< Fatal cause: unhandled interrupt
    GPX_STATUS_FATAL_STACK_OVERFLOW             = (int)6,   //!< Fatal cause: stack overflow
    GPX_STATUS_FATAL_KERNEL_OOPS                = (int)7,   //!< Fatal cause: kernel oops
    GPX_STATUS_FATAL_KERNEL_PANIC                = (int)8,   //!< Fatal cause: kernel panic
    GPX_STATUS_FATAL_UNALIGNED_ACCESS           = (int)9,   //!< Fatal cause: unaligned memory access
    GPX_STATUS_FATAL_MEMORY_ERROR               = (int)10,  //!< Fatal cause: memory error
    GPX_STATUS_FATAL_BUS_ERROR                  = (int)11,  //!< Fatal cause: bus error (bad pointer or malloc)
    GPX_STATUS_FATAL_USAGE_ERROR                = (int)12,  //!< Fatal cause: usage error
    GPX_STATUS_FATAL_DIV_ZERO                   = (int)13,  //!< Fatal cause: divide by zero
    GPX_STATUS_FATAL_SER0_REINIT                = (int)14,  //!< Fatal cause: serial port 0 re-initialization
    GPX_STATUS_FATAL_UNKNOWN                    = (int)0x1F,  //!< Fatal cause: unknown. TODO: Temporarily set to (5 bits). Reset to 0xFF when gpx_flash_cfg.debug is no longer used with fault reporting. (WHJ)

    GPX_STATUS_FAULT_RP                         = (int)0x20000000,  //!< Internal use
    GPX_STATUS_FAULT_UNUSED                     = (int)0xC0000000,  //!< Reserved for future fault status
};

/** @brief (DID_GPX_STATUS) GPX hardware status flags, reported in gpx_status_t.hdwStatus. Reports per-GNSS-receiver signal/time/reset/init health, communications and PPS timing errors, CNO (carrier-to-noise) signal quality errors, built-in self-test (BIT) progress, temperature, and the cause of the last system reset. GPX_HDW_STATUS_ERROR_MASK ORs together the sub-fields considered indicative of an error condition for quick fault checks. */
enum eGPXHdwStatusFlags
{
    GPX_HDW_STATUS_GNSS1_SATELLITE_RX       = (int)0x00000001,  //!< GNSS1 satellite signals are being received (antenna and cable are good)
    GPX_HDW_STATUS_GNSS2_SATELLITE_RX       = (int)0x00000002,  //!< GNSS2 satellite signals are being received (antenna and cable are good)
    GPX_HDW_STATUS_GNSS1_TIME_OF_WEEK_VALID = (int)0x00000004,  //!< GPS time of week is valid and reported for GNSS1. Otherwise the timeOfWeek is local system time.
    GPX_HDW_STATUS_GNSS2_TIME_OF_WEEK_VALID = (int)0x00000008,  //!< GPS time of week is valid and reported for GNSS2. Otherwise the timeOfWeek is local system time.

    GPX_HDW_STATUS_GNSS1_RESET_COUNT_MASK   = (int)0x00000070,  //!< Mask for the GNSS1 reset-required count field
    GPX_HDW_STATUS_GNSS1_RESET_COUNT_OFFSET = 4,                //!< Bit offset of GPX_HDW_STATUS_GNSS1_RESET_COUNT_MASK within hdwStatus
#define GPX_HDW_STATUS_GNSS1_RESET_COUNT(hdwStatus)     ((hdwStatus&GPX_HDW_STATUS_GNSS1_RESET_COUNT_MASK)>>GPX_HDW_STATUS_GNSS1_RESET_COUNT_OFFSET)  //!< Extract the GNSS1 reset-required count from a hdwStatus value

    GPX_HDW_STATUS_FAULT_GNSS1_INIT         = (int)0x00000080,  //!< Failed to communicate or setup GNSS receiver 1
    GPX_HDW_STATUS_GNSS1_FAULT_FLAG_OFFSET  = 7,                //!< Bit offset of GPX_HDW_STATUS_FAULT_GNSS1_INIT within hdwStatus

    GPX_HDW_STATUS_GNSS2_RESET_COUNT_MASK   = (int)0x00000700,  //!< Mask for the GNSS2 reset-required count field
    GPX_HDW_STATUS_GNSS2_RESET_COUNT_OFFSET = 8,                //!< Bit offset of GPX_HDW_STATUS_GNSS2_RESET_COUNT_MASK within hdwStatus
#define GPX_HDW_STATUS_GNSS2_RESET_COUNT(hdwStatus)     ((hdwStatus&GPX_HDW_STATUS_GNSS2_RESET_COUNT_MASK)>>GPX_HDW_STATUS_GNSS2_RESET_COUNT_OFFSET)  //!< Extract the GNSS2 reset-required count from a hdwStatus value

    GPX_HDW_STATUS_FAULT_GNSS2_INIT         = (int)0x00000800,  //!< Failed to communicate or setup GNSS receiver 2
    GPX_HDW_STATUS_GNSS2_FAULT_FLAG_OFFSET  = 11,               //!< Bit offset of GPX_HDW_STATUS_FAULT_GNSS2_INIT within hdwStatus

    GPX_HDW_STATUS_GNSS_FW_UPDATE_REQUIRED  = (int)0x00001000,  //!< GNSS is faulting; firmware update REQUIRED
    GPX_HDW_STATUS_LED_ENABLED              = (int)0x00002000,  //!< Enables LED in Manufacturing TBed
    GPX_HDW_STATUS_SYSTEM_RESET_REQUIRED    = (int)0x00004000,  //!< System reset is required for proper function
    GPX_HDW_STATUS_FLASH_WRITE_PENDING      = (int)0x00008000,  //!< System flash write staging or occurring now. Processor will pause and not respond during a flash write, typically 150-250 ms.

    GPX_HDW_STATUS_ERR_COM_TX_LIMITED       = (int)0x00010000,  //!< Communications Tx buffer limited
    GPX_HDW_STATUS_ERR_COM_RX_OVERRUN       = (int)0x00020000,  //!< Communications Rx buffer overrun
    GPX_HDW_STATUS_ERR_COM_MASK             = (int)0x00030000,  //!< Communications error mask

    GPX_HDW_STATUS_ERR_NO_GNSS1_PPS         = (int)0x00040000,  //!< GNSS1 PPS timepulse signal has not been received or is in error
    GPX_HDW_STATUS_ERR_NO_GNSS2_PPS         = (int)0x00080000,  //!< GNSS2 PPS timepulse signal has not been received or is in error
    GPX_HDW_STATUS_ERR_PPS_MASK             = (int)0x000C0000,  //!< GNSS PPS error mask

    GPX_HDW_STATUS_ERR_LOW_CNO_GNSS1        = (int)0x00100000,  //!< GNSS1 signal strength low (<20)
    GPX_HDW_STATUS_ERR_LOW_CNO_GNSS2        = (int)0x00200000,  //!< GNSS2 signal strength low (<20)
    GPX_HDW_STATUS_ERR_CNO_GNSS1_IR         = (int)0x00400000,  //!< GNSS1 signal irregular. High CNO standard deviation over 5 second period detected. 10x CNO mean sigma (i.e. >1.0 dBHz)
    GPX_HDW_STATUS_ERR_CNO_GNSS2_IR         = (int)0x00800000,  //!< GNSS2 signal irregular. High CNO standard deviation over 5 second period detected. 10x CNO mean sigma (i.e. >1.0 dBHz)
    GPX_HDW_STATUS_ERR_CNO_MASK             = (int)0x00F00000,  //!< GNSS signal error mask

    GPX_HDW_STATUS_BIT_RUNNING              = (int)0x01000000,  //!< (BIT) Built-in self-test running
    GPX_HDW_STATUS_BIT_PASSED               = (int)0x02000000,  //!< (BIT) Built-in self-test passed
    GPX_HDW_STATUS_BIT_FAULT                = (int)0x03000000,  //!< (BIT) Built-in self-test failure
    GPX_HDW_STATUS_BIT_MASK                 = (int)0x03000000,  //!< (BIT) Built-in self-test mask
    GPX_HDW_STATUS_BIT_OFFSET               = 24,               //!< (BIT) Built-in self-test offset

    GPX_HDW_STATUS_ERR_TEMPERATURE          = (int)0x04000000,  //!< Temperature outside spec'd operating range
    GPX_HDW_STATUS_GNSS_PPS_TIMESYNC        = (int)0x08000000,  //!< Time synchronized by GNSS PPS

    GPX_HDW_STATUS_RESET_CAUSE_MASK         = (int)0x70000000,  //!< Mask for the cause-of-system-reset field
    GPX_HDW_STATUS_RESET_CAUSE_BACKUP_MODE  = (int)0x10000000,  //!< Reset from Backup mode (low-power state w/ CPU off)
    GPX_HDW_STATUS_RESET_CAUSE_SOFT         = (int)0x20000000,  //!< Reset from Software
    GPX_HDW_STATUS_RESET_CAUSE_HDW          = (int)0x40000000,  //!< Reset from Hardware (NRST pin low)

    GPX_HDW_STATUS_FAULT_SYS_CRITICAL       = (int)0x80000000,  //!< Critical System Fault, CPU error. (see DID_GPX_STATUS.status, eGpxStatus::GPX_STATUS_FATAL_MASK)

    GPX_HDW_STATUS_ERROR_MASK               = (int)(GPX_HDW_STATUS_GNSS1_RESET_COUNT_MASK | GPX_HDW_STATUS_FAULT_GNSS1_INIT |
                                                                GPX_HDW_STATUS_GNSS2_RESET_COUNT_MASK | GPX_HDW_STATUS_FAULT_GNSS2_INIT |
                                                                GPX_HDW_STATUS_ERR_COM_MASK | GPX_HDW_STATUS_ERR_PPS_MASK | GPX_HDW_STATUS_ERR_CNO_MASK |
                                                                GPX_HDW_STATUS_ERR_TEMPERATURE | GPX_HDW_STATUS_RESET_CAUSE_MASK |
                                                                GPX_HDW_STATUS_FAULT_SYS_CRITICAL),  //!< Combined mask of all sub-fields considered an error/fault condition
};

/** @brief Reset cause reported by the GNSS receiver's internal driver ("cxd" = Sony CXD-series GNSS chipset used on the GPX), carried in gpx_gnss_status_t.lastRstCause. Reports why the GNSS receiver's driver state machine last reset/restarted the receiver; cxdRst_Max is a sentinel bounding the valid range, not a reported cause. */
enum eGNSSDriverRstCause {
    cxdRst_PowerOn          = 0,   //!< Reset from initial power-on
    cxdRst_Watchdog         = 1,   //!< Reset from a watchdog timeout
    cxdRst_ErrOpCode        = 2,   //!< Reset following an error opcode/response from the GNSS receiver
    cxdRst_ErrOpCode_FwUp   = 3,   //!< Reset following an error opcode/response from the GNSS receiver while in firmware-update mode
    cxdRst_ErrOpCode_init   = 4,   //!< Reset following an error opcode/response from the GNSS receiver during initialization
    cxdRst_UserRequested    = 5,   //!< Reset requested by the user/host
    cxdRst_FWUpdate         = 6,   //!< Reset to enter or complete a firmware update
    cxdRst_SysCmd           = 7,   //!< Reset issued via a system command
    cxdRst_InitTimeout      = 8,   //!< Reset after the GNSS receiver failed to complete initialization within the timeout period
    cxdRst_Status5          = 9,   //!< Reset following receipt of GNSS receiver status code 5
    cxdRst_StatusNot0       = 10,  //!< Reset following a non-zero (error) GNSS receiver status code
    cxdRst_flashUpdate      = 11,  //!< Reset to perform a flash update
    cxdRst_RTKEphMissing    = 12,  //!< Reset because required RTK ephemeris data was missing
    cxdRst_Max                     //!< Sentinel: one past the last valid reset-cause value; not a reported cause
};

/** @brief GNSS receiver driver run state, carried in gpx_gnss_status_t.runState. Tracks the GPX's internal state machine for managing the GNSS receiver, including normal init/run operation, pass-through (direct host-to-receiver communication) mode, and the firmware-update sequence (kFwInit stages the receiver into update mode in preparation for code injection, kFwUpdate is the ready-to-inject state). */
enum eGPXGnssRunState {
    kReset  = 0,     //!< GNSS receiver is being held in reset
    kInit,           //!< GNSS receiver is being initialized/configured
    kRun,            //!< GNSS receiver is initialized and running normally
    kPassthrough,    //!< Host communications are being passed through directly to the GNSS receiver
    kFwInit,         //!< Initializing into firmware-update mode (prep for code injections)
    kFwUpdate,       //!< Ready and able to accept code injections
    kError,          //!< GNSS receiver driver encountered an error
    kShutdown,       //!< GNSS receiver is being shut down
    kReinit,         //!< GNSS receiver is being re-initialized
    kHardReset,      //!< GNSS receiver is being hard reset
};

#define GNSS_RECEIVER_COUNT 2   //!< Number of GNSS receivers on the GPX (and number of elements in gpx_status_t.gnssStatus)

/** @brief Per-GNSS-receiver driver status, one entry per receiver in gpx_status_t.gnssStatus (DID_GPX_STATUS). */
typedef struct
{
    uint8_t lastRstCause;   //!< Last reset cause (see eGNSSDriverRstCause)
    uint8_t fwUpdateState;  //!< GNSS receiver firmware update status (see FirmwareUpdateState, defined in GPX firmware)
    uint8_t initState;      //!< GNSS receiver init status (see InitSteps, defined in GPX firmware)
    uint8_t runState;       //!< GNSS receiver run status (see eGPXGnssRunState)
} gpx_gnss_status_t;

#define GPX_INVALID_MCU_TEMP    -274.0f     //!< (C) Sentinel value for gpx_status_t.mcuTemp when the MCU temperature is not available; 1 degree less than absolute zero

/**
* @brief (DID_GPX_STATUS) GPX status: current time, fault/health status, per-receiver GNSS status, and streaming (GRMC/NMEA) message enable bits for each communications port.
*/
typedef struct
{
    uint32_t                timeOfWeekMs;       //!< (ms) GPS time of week (since Sunday morning)

    uint32_t                status;             //!< Status (see eGpxStatus)

    uint64_t                grmcBitsSer0;        //!< GRMC message enable bits for serial port 0 (see GRMC_BITS_...)
    uint64_t                grmcBitsSer1;        //!< GRMC message enable bits for serial port 1 (see GRMC_BITS_...)
    uint64_t                grmcBitsSer2;        //!< GRMC message enable bits for serial port 2 (see GRMC_BITS_...)
    uint64_t                grmcBitsUSB;         //!< GRMC message enable bits for USB (see GRMC_BITS_...)
    uint64_t                grmcNMEABitsSer0;    //!< NMEA message enable bits for serial port 0 (see NMEA_MSG_ID...)
    uint64_t                grmcNMEABitsSer1;    //!< NMEA message enable bits for serial port 1 (see NMEA_MSG_ID...)
    uint64_t                grmcNMEABitsSer2;    //!< NMEA message enable bits for serial port 2 (see NMEA_MSG_ID...)
    uint64_t                grmcNMEABitsUSB;     //!< NMEA message enable bits for USB (see NMEA_MSG_ID...)

    uint32_t                hdwStatus;           //!< Hardware status flags (see eGPXHdwStatusFlags)

    float                   mcuTemp;             //!< (C) MCU temperature (GPX_INVALID_MCU_TEMP if not available)

    uint32_t                navOutputPeriodMs;   //!< (ms) Navigation output period

    uint32_t                flashCfgChecksum;    //!< Flash config checksum used with host SDK synchronization

    uint32_t                rtkMode;             //!< RTK mode bits (see eRTKConfigBits)

    gpx_gnss_status_t       gnssStatus[GNSS_RECEIVER_COUNT];    //!< Per-GNSS-receiver driver status (GNSS1, GNSS2)

    uint8_t                 gpxSourcePort;       //!< Port this status message was sourced from

    double                  upTime;              //!< (s) Time since system was started
} gpx_status_t;


//////////////////////////////////////////////////////////////////////////
//  EVB
//////////////////////////////////////////////////////////////////////////

/** @brief (DID_EVB_STATUS) EVB status flags, reported in evb_status_t.evbStatus. Reports SD card logger readiness/state, SD card fault conditions, WiFi and XBee radio link state, in-progress flash writes, and manufacturing unlock state. */
enum eEvbStatus
{
    EVB_STATUS_SD_CARD_READY            = 0x00000001,  //!< SD card logger: card ready

    EVB_STATUS_SD_LOG_ENABLED           = 0x00000002,  //!< SD card Logger: running

    EVB_STATUS_SD_ERR_CARD_FAULT        = 0x00000010,  //!< SD card error: card file system
    EVB_STATUS_SD_ERR_CARD_FULL         = 0x00000020,  //!< SD card error: card full
    EVB_STATUS_SD_ERR_CARD_MASK         = 0x000000F0,  //!< SD card error: mask

    EVB_STATUS_WIFI_ENABLED             = 0x00010000,  //!< WiFi: enabled
    EVB_STATUS_WIFI_CONNECTED           = 0x00020000,  //!< WiFi: connected to access point (hot spot) or another device

    EVB_STATUS_XBEE_ENABLED             = 0x00100000,  //!< XBee: enabled
    EVB_STATUS_XBEE_CONNECTED           = 0x00200000,  //!< XBee: connected
    EVB_STATUS_XBEE_CONFIGURED          = 0x00400000,  //!< XBee: configured
    EVB_STATUS_XBEE_CONFIG_FAILURE      = 0x00800000,  //!< XBee: failed to configure

    EVB_STATUS_FLASH_WRITE_IN_PROGRESS  = 0x01000000,  //!< System flash write staging or occuring now.  Processor will pause and not respond during a flash write, typicaly 150-250 ms.

    EVB_STATUS_MANF_UNLOCKED            = 0x02000000,  //!< Manufacturing unlocked
};

/** @brief EVB-2 communications ports, indexed into evb_flash_cfg_t.cbf[] (communications bridge forwarding table) and used to select uinsComPort/uinsAuxPort. */
enum eEvb2CommPorts
{
    EVB2_PORT_UINS0     = 0,  //!< uINS serial port 0
    EVB2_PORT_UINS1     = 1,  //!< uINS serial port 1
    EVB2_PORT_XBEE      = 2,  //!< XBee radio module
    EVB2_PORT_XRADIO    = 3,  //!< External radio, TTL UART on H4-8 (orange) Tx, H4-7 (brown) Rx
    EVB2_PORT_BLE       = 4,  //!< Bluetooth Low Energy module
    EVB2_PORT_SP330     = 5,  //!< RS232/RS422/RS485 transceiver (SP330), H3-2 (brown) Tx, H3-5 (green) Rx
    EVB2_PORT_GPIO_H8   = 6,  //!< TTL UART on GPIO header H8-5 (brown) Tx, H8-6 (orange) Rx
    EVB2_PORT_USB       = 7,  //!< USB virtual COM port
    EVB2_PORT_WIFI      = 8,  //!< WiFi TCP/UDP link
    EVB2_PORT_CAN       = 9,  //!< CAN bus, H2-3 CANL (brown), H2-4 CANH (orange)
    EVB2_PORT_COUNT           //!< Number of communications ports
};

/** @brief EVB-2 communications bridge options, used in evb_flash_cfg_t.cbOptions to enable/configure individual ports of the bridge. */
enum eEvb2ComBridgeOptions
{
    EVB2_CB_OPTIONS_TRISTATE_UINS_IO    = 0x00000001,  //!< Tri-state (release) the uINS communication I/O lines
    EVB2_CB_OPTIONS_SP330_RS422         = 0x00000002,  //!< Configure SP330 transceiver for RS422 (differential) instead of RS232
    EVB2_CB_OPTIONS_XBEE_ENABLE         = 0x00000010,  //!< Enable the XBee radio port
    EVB2_CB_OPTIONS_WIFI_ENABLE         = 0x00000020,  //!< Enable the WiFi port
    EVB2_CB_OPTIONS_BLE_ENABLE          = 0x00000040,  //!< Enable the Bluetooth Low Energy port
    EVB2_CB_OPTIONS_SPI_ENABLE          = 0x00000080,  //!< Enable the SPI port
    EVB2_CB_OPTIONS_CAN_ENABLE          = 0x00000100,  //!< Enable the CAN port
    EVB2_CB_OPTIONS_I2C_ENABLE          = 0x00000200,  //!< Enable the I2C port. Tied to uINS G1, G2
};

/** @brief EVB-2 port behavior options, used in evb_flash_cfg_t.portOptions. */
enum eEvb2PortOptions
{
    EVB2_PORT_OPTIONS_RADIO_RTK_FILTER  = 0x00000001,  //!< Filter data forwarded to the radio port: allow RTCM3, NMEA, and RTCM3; reject Inertial Sense binary protocol
    EVB2_PORT_OPTIONS_DEFAULT           = EVB2_PORT_OPTIONS_RADIO_RTK_FILTER,  //!< Default port options
};

/** @brief (DID_EVB_STATUS) EVB monitor and log control interface. Reports EVB firmware version, aggregate evbStatus flags (see eEvbStatus), SD-card data logger control state, WiFi connection address, and a host-issued system command echo, along with the GPS time-of-week timestamp of the report. */
typedef struct
{
    uint32_t                week;                //!< GPS number of weeks since January 6th, 1980

    uint32_t                timeOfWeekMs;        //!< (ms) GPS time of week (since Sunday morning)

    uint8_t                 firmwareVer[4];      //!< Firmware (software) version

    uint32_t                evbStatus;           //!< Status (see eEvbStatus)

    uint32_t                loggerMode;          //!< Data logger control state (see eEvb2LoggerMode)

    uint32_t                loggerElapsedTimeMs; //!< (ms) Elapsed time of the current data log

    uint32_t                wifiIpAddr;          //!< WiFi IP address

    uint32_t                sysCommand;          //!< System command (see eSystemCommand). 99 = software reset

    double                  towOffset;           //!< (s) Time sync offset between local time since boot up to GPS time of week. Add this to IMU and sensor time to get GPS time of week in seconds.

} evb_status_t;

#define WIFI_SSID_PSK_SIZE      40

/** @brief WiFi access point credentials, stored per-preset in evb_flash_cfg_t.wifi[]. */
typedef struct
{
    char                    ssid[WIFI_SSID_PSK_SIZE];  //!< WiFi SSID, null-terminated string, max WIFI_SSID_PSK_SIZE-1 characters

    char                    psk[WIFI_SSID_PSK_SIZE];   //!< WiFi PSK (pre-shared key / password), null-terminated string, max WIFI_SSID_PSK_SIZE-1 characters

} evb_wifi_t;

/** @brief WiFi server (data forwarding) endpoint address, stored per-preset in evb_flash_cfg_t.server[]. */
typedef struct
{
    /** Server IP address */
    union {
        uint32_t        u32;    //!< IP address as a packed 32-bit value
        uint8_t         u8[4];  //!< IP address as four individual octets
    } ipAddr;                   //!< Server IP address

    uint32_t            port;   //!< Server port number

} evb_server_t;

/** @brief EVB configuration bits, used in evb_flash_cfg_t.bits. */
enum eEvbFlashCfgBits
{
    EVB_CFG_BITS_WIFI_SELECT_MASK               = 0x00000003,  //!< Mask for the selected WiFi preset index field
    EVB_CFG_BITS_WIFI_SELECT_OFFSET             = 0,           //!< Bit offset of EVB_CFG_BITS_WIFI_SELECT_MASK within bits
    EVB_CFG_BITS_SERVER_SELECT_MASK             = 0x0000000C,  //!< Mask for the selected server preset index field
    EVB_CFG_BITS_SERVER_SELECT_OFFSET           = 2,           //!< Bit offset of EVB_CFG_BITS_SERVER_SELECT_MASK within bits
    EVB_CFG_BITS_NO_STREAM_PPD_ON_LOG_BUTTON    = 0x00000010,  //!< Don't enable PPD stream when log button is pressed
    EVB_CFG_BITS_ENABLE_ADC4                    = 0x00000200,  //!< Enable 4-channel ADC sampling
    EVB_CFG_BITS_ENABLE_ADC10                   = 0x00000400,  //!< Enable 10-channel ADC sampling
};

#define NUM_WIFI_PRESETS                        3
#define EVB_CFG_BITS_SET_IDX_WIFI(bits,idx)     {(bits)&=EVB_CFG_BITS_WIFI_SELECT_MASK; (bits)|=(((idx)<<EVB_CFG_BITS_WIFI_SELECT_OFFSET)&EVB_CFG_BITS_WIFI_SELECT_MASK);}
#define EVB_CFG_BITS_SET_IDX_SERVER(bits,idx)   {(bits)&=EVB_CFG_BITS_SERVER_SELECT_MASK; (bits)|=(((idx)<<EVB_CFG_BITS_SERVER_SELECT_OFFSET)&EVB_CFG_BITS_SERVER_SELECT_MASK);}
#define EVB_CFG_BITS_IDX_WIFI(bits)             (((bits)&EVB_CFG_BITS_WIFI_SELECT_MASK)>>EVB_CFG_BITS_WIFI_SELECT_OFFSET)
#define EVB_CFG_BITS_IDX_SERVER(bits)           (((bits)&EVB_CFG_BITS_SERVER_SELECT_MASK)>>EVB_CFG_BITS_SERVER_SELECT_OFFSET)

/**
* (DID_EVB_FLASH_CFG) EVB-2 flash config for monitor, config, and logger control interface
*/
typedef struct
{
    uint32_t                size;                //!< Size of this struct

    uint32_t                checksum;            //!< Checksum, excluding size and checksum

    uint32_t                key;                 //!< Manufacturer method for restoring flash defaults

    uint8_t                 cbPreset;            //!< Communications bridge preset (see eEvb2ComBridgePreset)

    uint8_t                 reserved1[3];        //!< Reserved for 32-bit alignment

    uint32_t                cbf[EVB2_PORT_COUNT];  //!< Communications bridge forwarding, indexed by eEvb2CommPorts

    uint32_t                cbOptions;           //!< Communications bridge options (see eEvb2ComBridgeOptions)

    uint32_t                bits;                //!< Config bits (see eEvbFlashCfgBits)

    uint32_t                radioPID;            //!< Radio preamble ID (PID), 0x0 to 0x9. Only radios with matching PIDs can communicate together. Different PIDs minimize interference between multiple sets of networks. Checked before the network ID.

    uint32_t                radioNID;            //!< Radio network ID (NID), 0x0 to 0x7FFF. Only radios with matching NID can communicate together. Checked after the preamble ID.

    uint32_t                radioPowerLevel;     //!< Radio transmitter output power level (XBee PRO SX 0=20dBm, 1=27dBm, 2=30dBm)

    evb_wifi_t              wifi[NUM_WIFI_PRESETS];    //!< WiFi SSID and PSK presets

    evb_server_t            server[NUM_WIFI_PRESETS];  //!< Server IP and port presets

    float                   encoderTickToWheelRad;  //!< (rad) Encoder tick to wheel rotation conversion factor. Encoder tick count per revolution on 1 channel x gear ratio x 2pi.

    uint32_t                CANbaud_kbps;        //!< (kbps) CAN baudrate

    uint32_t                can_receive_address; //!< CAN receive address

    uint8_t                 uinsComPort;         //!< EVB port for uINS communications and SD card logging. 0=uINS-Ser0 (default), 1=uINS-Ser1, SP330=5, 6=GPIO_H8 (use eEvb2CommPorts)

    uint8_t                 uinsAuxPort;         //!< EVB port for uINS aux com and RTK corrections. 0=uINS-Ser0, 1=uINS-Ser1 (default), 5=SP330, 6=GPIO_H8 (use eEvb2CommPorts)

    uint8_t                 reserved2[2];        //!< Reserved to ensure 32-bit alignment

    uint32_t                portOptions;         //!< Enable radio RTK filtering, etc. (see eEvb2PortOptions)

    uint32_t                h3sp330BaudRate;     //!< (bps) Baud rate for EVB serial port H3 (SP330 RS233 and RS485/422)

    uint32_t                h4xRadioBaudRate;    //!< (bps) Baud rate for EVB serial port H4 (TTL to external radio)

    uint32_t                h8gpioBaudRate;      //!< (bps) Baud rate for EVB serial port H8 (TTL)

    uint32_t                wheelCfgBits;        //!< Wheel encoder configuration (see eWheelCfgBits)

    uint32_t                velocityControlPeriodMs;  //!< (ms) Wheel update period. Sets the wheel encoder and control update period.

} evb_flash_cfg_t;


/** @brief EVB-2 communications bridge configuration presets, selecting a fixed forwarding topology between uINS-COM/uINS-AUX and the physical ports (USB, RS232/RS422, XBee, WiFi, XRadio, H8). Set in evb_flash_cfg_t.cbPreset. */
enum eEvb2ComBridgePreset
{
    EVB2_CB_PRESET_NA               = 0,  //!< No change. Sending this value causes no effect.

    EVB2_CB_PRESET_ALL_OFF          = 1,  //!< No connections. Off: XBee, WiFi

    EVB2_CB_PRESET_RS232            = 2,  //!< [uINS Hub] LED-GRN (uINS-COM): USB, RS232, H8. (uINS-AUX): XRadio. Off: XBee, WiFi

    EVB2_CB_PRESET_RS232_XBEE       = 3,  //!< [uINS Hub] LED-BLU (uINS-COM): USB, RS232, H8. (uINS-AUX): XBee, XRadio. Off: WiFi

    EVB2_CB_PRESET_RS422_WIFI       = 4,  //!< [uINS Hub] LED-PUR (uINS-COM): USB, RS422, H8. (uINS-AUX): WiFi, XRadio. Off: XBee

    EVB2_CB_PRESET_SPI_RS232        = 5,  //!< [uINS Hub] LED-CYA (uINS-SER1 SPI): USB, RS423, H8. Off: WiFi, XBee. A reset is required following selection of this CBPreset to enable SPI on the uINS, in order to assert uINS pin 10 (G9/nSPI_EN) during bootup.

    EVB2_CB_PRESET_USB_HUB_RS232    = 6,  //!< [USB Hub] LED-YEL (USB): RS232, H8, XBee, XRadio.

    EVB2_CB_PRESET_USB_HUB_RS422    = 7,  //!< [USB Hub] LED-WHT (USB): RS485/RS422, H8, XRadio.

    EVB2_CB_PRESET_COUNT            = 8,  //!< Number of bridge configuration presets

};

#define EVB2_CB_PRESET_DEFAULT      EVB2_CB_PRESET_RS232

/** @brief Data logger control commands for evb_status_t.loggerMode / DID_EVB_STATUS logger control. Values labeled CMD are one-shot commands. */
enum eEvb2LoggerMode
{
    EVB2_LOG_NA         = 0,  //!< Do not change. Sending this value causes no effect.

    EVB2_LOG_CMD_START  = 2,  //!< Start new log

    EVB2_LOG_CMD_STOP   = 4,  //!< Stop logging

    EVB2_LOG_CMD_PURGE  = 1002,  //!< Purge all data logs from drive

};

/** @brief Communications port hardware type, encoded in the upper nibble of a port monitor port identifier. */
enum ePortMonPortType
{
    PORT_MON_PORT_TYPE_UART = (uint8_t)(1 << 4),  //!< UART/serial port
    PORT_MON_PORT_TYPE_USB  = (uint8_t)(2 << 4),  //!< USB virtual COM port
    PORT_MON_PORT_TYPE_SPI  = (uint8_t)(3 << 4),  //!< SPI port
    PORT_MON_PORT_TYPE_I2C  = (uint8_t)(4 << 4),  //!< I2C port
    PORT_MON_PORT_TYPE_CAN  = (uint8_t)(5 << 4),  //!< CAN port
    PORT_MON_PORT_TYPE_MAX  = (uint8_t)(6 << 4)   //!< Upper bound / invalid port type marker
};

/** Alias of port_stats_t used as the per-port monitor record type in port_monitor_t.port[]. */
typedef port_stats_t port_monitor_set_t;

/**
* @brief (DID_PORT_MONITOR / DID_GPX_PORT_MONITOR) Data rate and status monitoring for each communications port.
*/
typedef struct
{
    port_stats_t    port[NUM_SERIAL_PORTS];  //!< Per-port data rate and status statistics, one entry per communications port (see port_stats_t)

    uint8_t         activePorts;             //!< Number of ports in the port[] array. FIXME: This should be moved to BEFORE the port definition, so on the receiving end, we know how many ports to expect.
} port_monitor_t;

/** @brief Stores data for the event mask, used in did_event_filter_t.eventMask to filter DID_EVENT messages by priority and message type. */
typedef struct
{
    int8_t      priorityLevel;  //!< Priority mask (see eEventPriority)

    uint32_t    msgTypeIdMask;  //!< ID mask field (see eEventProtocol, i.e. 0x01 << eEventProtocol)
} did_event_mask_t;

/** @brief Sent in the data field of DID_EVENT for eEventProtocol: EVENT_MSG_TYPE_ID_ENA_GNSS1_FILTER, EVENT_MSG_TYPE_ID_ENA_GNSS2_FILTER, EVENT_MSG_TYPE_ID_ENA_FILTER. Selects which port(s) the enclosed event mask applies to. */
typedef struct
{
    uint8_t             portMask;   //!< Target port mask. 0x80 selects the current (receiving) port; otherwise a specific port is selected via (0x01 << TARGET_PORT).

    did_event_mask_t    eventMask;  //!< Priority and message-type filter mask applied to the targeted port(s)

} did_event_filter_t;

#define EVENT_MEM_REQ_SIZE  16

/** @brief Response payload for a DID_EVENT memory-read request, returning EVENT_MEM_REQ_SIZE bytes read from device memory starting at reqAddr. */
typedef struct
{
    uint32_t    reqAddr;                    //!< Starting memory address the returned data was read from
    uint8_t     data[EVENT_MEM_REQ_SIZE];   //!< Memory contents read starting at reqAddr, EVENT_MEM_REQ_SIZE bytes
} did_event_memResp_t;

/** @brief Request payload for a DID_EVENT memory-read request, requesting EVENT_MEM_REQ_SIZE bytes of device memory starting at reqAddr. */
typedef struct
{
    uint32_t    reqAddr;  //!< Starting memory address to read from
} did_event_memReq_t;

/** @brief Identifies the type/format of the payload carried in did_event_t.data[] (did_event_t.msgTypeID). Values 0-31 identify normal event payloads (raw receiver data, ASCII debug text, or internal register/memory-read dumps); negative values (cast to uint16_t) are reserved for filter configuration and filter-response control messages sent to/from the device rather than broadcast telemetry. */
enum eEventMsgTypeID
{
    EVENT_MSG_TYPE_ID_RAW               = 1,           //!< Raw (unparsed) receiver byte stream
    EVENT_MSG_TYPE_ID_ASCII             = 2,           //!< ASCII text (debug/log) payload
    EVENT_MSG_TYPE_ID_RTMC3_RCVR1       = 11,          //!< RTCM3 data from GNSS receiver 1
    EVENT_MSG_TYPE_ID_RTMC3_RCVR2       = 12,          //!< RTCM3 data from GNSS receiver 2
    EVENT_MSG_TYPE_ID_RTMC3_EXT         = 13,          //!< RTCM3 data from an external source
    EVENT_MSG_TYPE_ID_SONY_BIN_RCVR1    = 14,          //!< Sony binary protocol data from GNSS receiver 1
    EVENT_MSG_TYPE_ID_SONY_BIN_RCVR2    = 15,          //!< Sony binary protocol data from GNSS receiver 2
    // EVENT_MSG_TYPE_ID_DBG_READ          = 16,

    EVENT_MSG_TYPE_ID_IMX_MEM_READ      = 20,          //!< IMX device memory-read response (see did_event_memResp_t)
    EVENT_MSG_TYPE_ID_GPX_MEM_READ      = 21,          //!< GPX device memory-read response (see did_event_memResp_t)
    EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_INST = 22,          //!< IMX DMA TX channel 0 instance state dump (see eventImxDmaTxInst_t)
    EVENT_MSG_TYPE_ID_IMX_SER0_REG      = 23,          //!< IMX serial port 0 peripheral register dump
    EVENT_MSG_TYPE_ID_IMX_SER0_CFG      = 24,          //!< IMX serial port 0 configuration dump
    EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_CHAN = 25,          //!< IMX DMA TX channel 0 channel-register dump
    EVENT_MSG_TYPE_ID_IMX_GPIO_TX_0_REG = 26,          //!< IMX GPIO TX port 0 register dump

    EVENT_MSG_TYPE_ID_GPX_DMA_RX_0_INST = 27,          //!< GPX DMA RX channel 0 instance state dump
    EVENT_MSG_TYPE_ID_GPX_SER0_REG      = 28,          //!< GPX serial port 0 peripheral register dump
    EVENT_MSG_TYPE_ID_GPX_SER0_CFG      = 29,          //!< GPX serial port 0 configuration dump
    EVENT_MSG_TYPE_ID_GPX_DMA_RX_0_CHAN = 30,          //!< GPX DMA RX channel 0 channel-register dump
    EVENT_MSG_TYPE_ID_GPX_GPIO_RX_0_REG = 31,          //!< GPX GPIO RX port 0 register dump

    EVENT_MSG_TYPE_ID_FILTER_RESPONSE   = (uint16_t)-4,    //!< Reply confirming the event filter currently in effect
    EVENT_MSG_TYPE_ID_ENA_GNSS1_FILTER  = (uint16_t)-3,    //!< Sets/enables the event filter applied to GNSS receiver 1 data (see did_event_filter_t)
    EVENT_MSG_TYPE_ID_ENA_GNSS2_FILTER  = (uint16_t)-2,    //!< Sets/enables the event filter applied to GNSS receiver 2 data (see did_event_filter_t)
    EVENT_MSG_TYPE_ID_ENA_FILTER        = (uint16_t)-1,    //!< Sets/enables the general (non-GNSS) event filter (see did_event_filter_t)
};

/** @brief Snapshot of an IMX DMA TX channel instance, combining live DMA controller registers with the driver's channel configuration/state. Sent as the payload for EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_INST for low-level debugging of serial transmit DMA stalls/overflows. */
typedef struct{
    uint32_t                inst_CCR;                   //!< DMA channel x configuration register (live hardware register)
    uint32_t                inst_CNDTR;                 //!< DMA channel x number of data register, remaining transfer count (live hardware register)
    uint32_t                inst_CPAR;                  //!< DMA channel x peripheral address register (live hardware register)
    uint32_t                inst_CMAR;                  //!< DMA channel x memory address register, current transfer address (live hardware register)

    uint8_t                 *ptr_start;                 //!< Pointer to the start of the DMA transfer buffer
    uint8_t                 *ptr_end;                   //!< Pointer to the end of the DMA transfer buffer
    uint16_t                active_tx_len;              //!< Length in bytes of the transfer currently in progress
    uint8_t                 done;                       //!< Transfer-complete flag; currently only used in TX

    uint8_t                 cfg_dir;                    //!< DMA transfer direction: DMA_RX or DMA_TX
    uint8_t                 cfg_circular;               //!< Circular buffer mode: DMA_CIRC_ON or DMA_CIRC_OFF
    uint8_t                 cfg_priority;               //!< DMA channel priority: DMA_PRIO_LOW, DMA_PRIO_MEDIUM, DMA_PRIO_HIGH, DMA_PRIO_VERY_HIGH
    uint8_t                 cfg_interrupt;              //!< Non-zero if the DMA transfer-complete interrupt is enabled
    uint8_t                 cfg_interrupt_priority;     //!< Interrupt priority, 0 to 15 (15 is lowest priority)
    uint8_t                 cfg_dma_channel_select;     //!< DMA channel select, 0 to 7. See RM0394 11.6.7
    uint8_t                 cfg_parent_type;            //!< Type of peripheral that owns this DMA channel: DMA_PARENT_USART, ...
    void                    *cfg_parent;                //!< Pointer to the parent peripheral's init/base structure
    uint32_t                *cfg_periph_reg;            //!< Pointer to the peripheral data register targeted by this DMA channel
    uint8_t                 *cfg_buf;                   //!< Pointer to the configured DMA buffer
    uint16_t                cfg_buf_len;                //!< Size of cfg_buf in bytes; does not correspond to the length register, just however big the buffer is
    uint8_t                 cfg_linear_buf;             //!< If true, the buffer is user-specified and treated like a non-circular buffer
    void                    *cfg_tcie_handler;          //!< Transfer-complete interrupt callback handler, if configured

    int                     lastDmaUsed;                //!< Number of bytes in the buffer minus bytes last read; used to detect buffer overflow
    uint8_t                 overflow;                   //!< Non-zero if a buffer overflow has been detected

} eventImxDmaTxInst_t;

/** @brief Severity/verbosity level of a DID_EVENT message, used both to tag did_event_t.priority and as the priorityLevel filter mask in did_event_mask_t (one bit per level, i.e. 0x01 << eEventPriority). Lower numeric values are higher severity. */
enum eEventPriority
{
    EVENT_PRIORITY_FAULT        = 0,   //!< Critical fault condition
    EVENT_PRIORITY_ERR          = 1,   //!< Error condition
    EVENT_PRIORITY_WARNING      = 2,   //!< Warning condition
    EVENT_PRIORITY_INFO         = 3,   //!< Informational message
    EVENT_PRIORITY_INFO_VERBOSE = 4,   //!< Verbose informational message
    EVENT_PRIORITY_DBG          = 5,   //!< Debug message
    EVENT_PRIORITY_DBG_VERBOSE  = 6,   //!< Verbose debug message
    EVENT_PRIORITY_TRIVIAL      = 7,   //!< Trivial/lowest-priority message

    EVENT_PRIORITY_NONE         = -1,  //!< No priority; used on messages that should only be filtered/broadcast based on message type ID, not priority
};

/** @brief (DID_EVENT) INTERNAL USE ONLY. Generic, variable-length event/diagnostic message envelope used to carry debug data (raw protocol bytes, register/memory dumps, ASCII logs, filter control messages, etc.) out of the device without a dedicated DID. The actual payload size is length bytes starting at data[0]; the struct's true wire size is DID_EVENT_HEADER_SIZE + length. */
typedef struct
{
    double          time;          //!< Time (uptime in seconds)

    uint32_t        senderSN;      //!< Serial number of the device that generated this event

    uint16_t        senderHdwId;   //!< Hardware type of the sender: 0=Host, 1=uINS, 2=EVB, 3=IMX, 4=GPX (see "Product Hardware ID")

    int8_t          priority;      //!< Event priority/severity (see eEventPriority)
    uint8_t         res8;          //!< Reserved for byte alignment

    uint16_t        msgTypeID;     //!< Type/format of the payload in data[] (see eEventMsgTypeID)
    uint16_t        length;        //!< Number of valid payload bytes in data[]

    uint8_t         data[1];       //!< Variable-length payload, length bytes; interpretation depends on msgTypeID
} did_event_t;

#define DID_EVENT_HEADER_SIZE           (sizeof(did_event_t) - sizeof(uint8_t))  //!< Size of did_event_t excluding its trailing variable-length data payload byte

/** @brief Bitmask/value set for system_fault_t.status (DID_SYS_FAULT / DID_GPX_SYS_FAULT). Bits 0-23 are general, non-fatal status/error flags that may accumulate independently; bits 24-31 (SYS_FAULT_STATUS_CRITICAL_ERROR_MASK) instead encode a single small integer value (1-7) at SYS_FAULT_STATUS_CRITICAL_ERROR_pos identifying the specific critical fault that caused a reset, since only one critical fault can be active/reported at a time. */
enum eSysFaultStatus
{
    SYS_FAULT_STATUS_HARDWARE_RESET             = 0x00000000,  //!< Reset was caused by hardware (power-on/pin reset), not a logged fault
    SYS_FAULT_STATUS_USER_RESET                 = 0x00000001,  //!< Reset was explicitly requested by the user/host
    SYS_FAULT_STATUS_ENABLE_BOOTLOADER          = 0x00000002,  //!< Reset into the bootloader was requested

    // General:
    SYS_FAULT_STATUS_SOFT_RESET                 = 0x00000010,  //!< Reset was a software-initiated reset
    SYS_FAULT_STATUS_FLASH_MIGRATION_EVENT      = 0x00000020,  //!< A flash configuration/data migration was performed
    SYS_FAULT_STATUS_FLASH_MIGRATION_COMPLETED  = 0x00000040,  //!< Flash configuration/data migration completed successfully
    SYS_FAULT_STATUS_RTK_MISC_ERROR             = 0x00000080,  //!< Miscellaneous RTK error occurred
    SYS_FAULT_STATUS_MCUBOOT_SWAP_FAILURE       = 0x00000100,  //!< MCUboot image swap operation failed
    SYS_FAULT_STATUS_RTK_BUFFER_LIMIT           = 0x00000200,  //!< RTK buffer limit was reached
    SYS_FAULT_STATUS_SENSOR_CALIBRATION         = 0x00000400,  //!< Sensor calibration fault/event occurred
    SYS_FAULT_STATUS_HARDWARE_DETECTION         = 0x00000800,  //!< Hardware auto-detection fault/event occurred
    SYS_FAULT_STATUS_FLASH_ECCD_NVM             = 0x00001000,  //!< Uncorrectable flash ECC error in NVM region; page erased on boot recovery
    SYS_FAULT_STATUS_FLASH_ECCD_APP_CODE        = 0x00002000,  //!< Uncorrectable flash ECC error in app code region; informational (PR2 will stay-in-ISbl)
    SYS_FAULT_STATUS_FLASH_ECCD_BL_CODE         = 0x00004000,  //!< Uncorrectable flash ECC error in IS-bootloader code region (IMX-5 only); informational (PR2 will jump to ROM DFU)
    SYS_FAULT_STATUS_FLASH_ECCC                 = 0x00008000,  //!< Correctable flash ECC error (single-bit, hardware-corrected); informational
    SYS_FAULT_STATUS_GENERAL_ERROR_MASK         = 0x00FFFFF0,  //!< Mask covering all general (non-critical) status/error bits


    // Critical: (usually associated with system reset)
    SYS_FAULT_STATUS_CRITICAL_ERROR_pos         = 24,                                          //!< Bit position of the critical-fault-cause value within status
    SYS_FAULT_STATUS_HARD_FAULT                 = 1 << SYS_FAULT_STATUS_CRITICAL_ERROR_pos,     //!< Critical fault cause: CPU hard fault
    SYS_FAULT_STATUS_USAGE_FAULT                = 2 << SYS_FAULT_STATUS_CRITICAL_ERROR_pos,     //!< Critical fault cause: CPU usage fault
    SYS_FAULT_STATUS_MEM_MANGE                  = 3 << SYS_FAULT_STATUS_CRITICAL_ERROR_pos,     //!< Critical fault cause: CPU memory management fault
    SYS_FAULT_STATUS_BUS_FAULT                  = 4 << SYS_FAULT_STATUS_CRITICAL_ERROR_pos,     //!< Critical fault cause: CPU bus fault
    SYS_FAULT_STATUS_MALLOC_FAILED              = 5 << SYS_FAULT_STATUS_CRITICAL_ERROR_pos,     //!< Critical fault cause: dynamic memory allocation failed
    SYS_FAULT_STATUS_STACK_OVERFLOW             = 6 << SYS_FAULT_STATUS_CRITICAL_ERROR_pos,     //!< Critical fault cause: RTOS task stack overflow detected
    SYS_FAULT_STATUS_WATCHDOG_RESET             = 7 << SYS_FAULT_STATUS_CRITICAL_ERROR_pos,     //!< Critical fault cause: watchdog timer reset (a task failed to feed the WDT)
    SYS_FAULT_STATUS_CRITICAL_ERROR_MASK        = 0xFF000000,                                   //!< Mask isolating the critical-fault-cause value from status
};

/** @brief Word index into the MCU's battery/VDD-backed backup register (or retained-RAM) array used to preserve fault context across a reset, so it can survive a hard fault/watchdog reset and be read back into system_fault_t on the next boot. Each enumerator names the backup-register word holding the corresponding system_fault_t field. */
enum eBackupOffset
{
    IS_BACKUP_UPTIME            = 0,   //!< Backup word holding system_fault_t.upTime
    IS_BACKUP_STATUS            = 1,   //!< Backup word holding system_fault_t.status (see eSysFaultStatus)
    IS_BACKUP_FILE_NUM          = 2,   //!< Backup word holding system_fault_t.fileNum
    IS_BACKUP_LINE_NUM          = 3,   //!< Backup word holding system_fault_t.lineNum
    IS_BACKUP_HALT_REASON       = 4,   //!< Backup word holding system_fault_t.haltReason
    IS_BACKUP_LR                = 5,   //!< Backup word holding system_fault_t.lr
    IS_BACKUP_PC                = 6,   //!< Backup word holding system_fault_t.pc
    IS_BACKUP_PSR               = 7,   //!< Backup word holding system_fault_t.psr
    IS_BACKUP_TASK_A_LAST_FEED  = 8,   //!< Backup word holding system_fault_t.taskALastFeed
    IS_BACKUP_TASK_B_LAST_FEED  = 9,   //!< Backup word holding system_fault_t.taskBLastFeed
    IS_BACKUP_WDT_LAST_FEED     = 10,  //!< Backup word holding system_fault_t.wdtLastFeed
    IS_BACKUP_VAR0              = 11,  //!< Backup word holding system_fault_t.var0
    IS_BACKUP_VAR1              = 12,  //!< Backup word holding system_fault_t.var1
    IS_BACKUP_VAR2              = 13,  //!< Backup word holding system_fault_t.var2
    IS_BACKUP_VAR3              = 14,  //!< Backup word holding system_fault_t.var3
    IS_BACKUP_COUNT                    //!< Number of words in the backup register array
};

/** @brief (DID_SYS_FAULT / DID_GPX_SYS_FAULT) System fault information. This is broadcast automatically every 10s if a critical fault is detected. Fields are populated from the MCU backup register array (see eBackupOffset) so the information survives the reset caused by the fault. */
typedef struct
{
    uint32_t    upTime;         //!< Time of fault, uptime in milliseconds

    uint32_t    status;         //!< System fault status (see eSysFaultStatus)

    uint32_t    fileNum;        //!< File number (source file identifier) where the fault occurred

    uint32_t    lineNum;        //!< Line number within the file where the fault occurred

    uint32_t    haltReason;     //!< Zephyr halt reason code

    uint32_t    lr;             //!< Link register value at time of fault

    uint32_t    pc;             //!< Program Counter value at time of fault

    uint32_t    psr;            //!< Program Status Register value at time of fault

    uint32_t    taskALastFeed;  //!< Milliseconds since task A last ran

    uint32_t    taskBLastFeed;  //!< Milliseconds since task B last ran

    uint32_t    wdtLastFeed;    //!< Milliseconds since the watchdog timer was last fed

    uint32_t    var0;           //!< Multi-purpose register 0, fault-specific diagnostic value

    uint32_t    var1;           //!< Multi-purpose register 1, fault-specific diagnostic value

    uint32_t    var2;           //!< Multi-purpose register 2, fault-specific diagnostic value

    uint32_t    var3;           //!< Multi-purpose register 3, fault-specific diagnostic value

} system_fault_t;

/** @brief RTOS task IDs for the IMX target, indexing rtos_info_t.task[] (DID_RTOS_INFO). Order must match the task creation order in IMX firmware; IMX_RTOS_NUM_TASKS (kept last) sizes the task[] array and is not itself a task ID. */
enum eImxRtosTask
{
    IMX_TASK_SAMPLE = 0,        //!< Task 0: Sample - sensor sampling task
    IMX_TASK_NAV,                //!< Task 1: Nav - navigation filter task
    IMX_TASK_COMMUNICATIONS,     //!< Task 2: Communications - host/port communications task
    IMX_TASK_MAINTENANCE,        //!< Task 3: Maintenance - housekeeping/maintenance task
    IMX_TASK_IDLE,                //!< Task 4: Idle - RTOS idle task
    IMX_TASK_TIMER,               //!< Task 5: Timer - RTOS timer service task

    IMX_RTOS_NUM_TASKS                 //!< Number of RTOS tasks on IMX; keep last, sizes rtos_info_t.task[]
};

/** @brief RTOS task IDs for the GPX target, indexing gpx_rtos_info_t.task[] (DID_GPX_RTOS_INFO). Order must match the task creation order in GPX firmware; GPX_RTOS_NUM_TASKS (kept last) sizes the task[] array and is not itself a task ID. */
enum eGpxRtosTask
{
    GPX_TASK_COMM   = 0,   //!< Task 0: Communication - host/port communications task
    GPX_TASK_RTK,           //!< Task 1: RTK - RTK correction processing task
    GPX_TASK_IDLE,          //!< Task 2: Idle - RTOS idle task
    GPX_TASK_TIMER,         //!< Task 3: Timer - RTOS timer service task

    GPX_RTOS_NUM_TASKS,                 //!< Number of RTOS tasks on GPX; keep last, sizes gpx_rtos_info_t.task[]
};

/** @brief RTOS task IDs for the EVB-2 target, indexing evb_rtos_info_t.task[] (DID_EVB_RTOS_INFO). Order must match the task creation order in EVB firmware; EVB_RTOS_NUM_TASKS (kept last) sizes the task[] array and is not itself a task ID. */
enum eEvbRtosTask
{
    EVB_TASK_COMMUNICATIONS,   //!< Task 0: Communications - host/port communications task
    EVB_TASK_LOGGER,           //!< Task 1: Logger - data logging task
    EVB_TASK_WIFI,             //!< Task 2: WiFi - WiFi communications task
    EVB_TASK_MAINTENANCE,      //!< Task 3: Maintenance - housekeeping/maintenance task
    EVB_TASK_IDLE,             //!< Task 4: Idle - RTOS idle task
    EVB_TASK_TIMER,            //!< Task 5: Timer - RTOS timer service task
    EVB_TASK_SPI_UINS_COM,     //!< Task 6: SPI to uINS - SPI communications task talking to the attached uINS/IMX

    EVB_RTOS_NUM_TASKS                  //!< Number of RTOS tasks on EVB; keep last, sizes evb_rtos_info_t.task[]
};

/** @brief Generic/shared RTOS task IDs resolved at compile time to the target-specific enum (eGpxRtosTask when building for GPX_1, eImxRtosTask otherwise). Only the handful of task IDs common to all targets (idle, timer, and the task count) are re-exposed here; code that needs to name a target-specific task (e.g. IMX_TASK_NAV or EVB_TASK_WIFI) must use that target's enum directly. Not used by EVB firmware, which has no conditional branch here and works with eEvbRtosTask directly. */
enum eRtosTask
{
#if defined(GPX_1)
    TASK_IDLE       = GPX_TASK_IDLE,        //!< Idle task ID, aliased from eGpxRtosTask
    TASK_TIMER      = GPX_TASK_TIMER,       //!< Timer task ID, aliased from eGpxRtosTask
    RTOS_NUM_TASKS  = GPX_RTOS_NUM_TASKS    //!< Number of RTOS tasks, aliased from eGpxRtosTask
#else   // IMX_5
    TASK_IDLE       = IMX_TASK_IDLE,        //!< Idle task ID, aliased from eImxRtosTask
    TASK_TIMER      = IMX_TASK_TIMER,       //!< Timer task ID, aliased from eImxRtosTask
    RTOS_NUM_TASKS  = IMX_RTOS_NUM_TASKS    //!< Number of RTOS tasks, aliased from eImxRtosTask
#endif
};

/** Max task name length - do not change */
#define MAX_TASK_NAME_LEN   12

/** @brief Per-task RTOS status/profiling record, one per task in rtos_info_t.task[] / gpx_rtos_info_t.task[] / evb_rtos_info_t.task[] (indexed by eImxRtosTask, eGpxRtosTask, or eEvbRtosTask respectively). Runtime statistics are updated on every task iteration and are in real-world microsecond/millisecond units (contrast with rtos_profile_t, which is an internal tick-based variant of the same statistics). */
typedef struct PACKED
{
    char                    name[MAX_TASK_NAME_LEN];   //!< Task name, null-terminated (up to MAX_TASK_NAME_LEN characters)

    uint32_t                priority;        //!< Task priority (0 - 8)

    uint32_t                stackUnused;     //!< Stack high water mark, in unused bytes remaining (higher is healthier)

    uint32_t                periodMs;        //!< Task period, in milliseconds

    uint32_t                runtimeUs;       //!< Last measured runtime, in microseconds

    float                   avgRuntimeUs;    //!< Low-pass-filtered average runtime, in microseconds

    float                   lowerRuntimeUs;  //!< Average of runtimes less than avgRuntimeUs, in microseconds

    float                   upperRuntimeUs;  //!< Average of runtimes greater than avgRuntimeUs, in microseconds

    uint32_t                maxRuntimeUs;    //!< Maximum runtime observed, in microseconds

    uint32_t                startTimeUs;     //!< Local time when the task loop last started (following its delay), in microseconds

    uint8_t                 gapCount;        //!< Counter of times the task took too long to run

    uint8_t                 quadGapCount;    //!< Counter of times the task took too long to run 4x in a row

    uint8_t                 doubleGapCount;  //!< Counter of times the task took too long to run 2x in a row

    uint8_t                 reserved;        //!< Reserved for byte alignment

    float                   cpuUsage;        //!< Processor usage, in percent

    uint32_t                handle;          //!< RTOS task handle (opaque, implementation-defined identifier)

} rtos_task_t;

/** @brief Internal RTOS task profiling info, functionally the same set of statistics as rtos_task_t but expressed in raw processor ticks rather than converted to microseconds/milliseconds; used internally by the firmware profiler and not broadcast directly in a DID. */
typedef struct PACKED
{
    uint32_t                timeTicks;             //!< Time, in processor ticks

    uint32_t                runtimeTicks;          //!< Last measured runtime, in processor ticks

    float                   avgRuntimeTicks;        //!< Low-pass-filtered average runtime, in processor ticks

    float                   lowerRuntimeTicks;      //!< Average of runtimes less than avgRuntimeTicks, in processor ticks

    float                   upperRuntimeTicks;      //!< Average of runtimes greater than avgRuntimeTicks, in processor ticks

    uint32_t                maxRuntimeTicks;        //!< Maximum runtime observed, in processor ticks

    uint32_t                startTimeTicks;         //!< Local time when the task loop last started (following its delay), in processor ticks

    uint8_t                 gapCount;               //!< Counter of times the task took too long to run

    uint8_t                 quadGapCount;           //!< Counter of times the task took too long to run 4x in a row

    uint8_t                 doubleGapCount;         //!< Counter of times the task took too long to run 2x in a row

    uint8_t                 successiveGapCount;     //!< Count of back-to-back (consecutive) gaps

    uint32_t                periodTicks;            //!< Task period, in processor ticks

} rtos_profile_t;

/** @brief (DID_RTOS_INFO) IMX RTOS heap and per-task status/profiling information, broadcast for host-side monitoring of firmware task health (stack margins, CPU usage, timing overruns). task[] is indexed by eImxRtosTask. */
typedef struct PACKED
{
    uint32_t                freeHeapSize;   //!< Heap high water mark, in free bytes remaining (lowest historical value)

    uint32_t                mallocSize;     //!< Total memory allocated using RTOS pvPortMalloc(), in bytes

    uint32_t                freeSize;       //!< Total memory freed using RTOS vPortFree(), in bytes

    rtos_task_t             task[IMX_RTOS_NUM_TASKS];   //!< Per-task status/profiling info, indexed by eImxRtosTask
} rtos_info_t;

/** @brief (DID_GPX_RTOS_INFO) GPX RTOS heap and per-task status/profiling information, broadcast for host-side monitoring of firmware task health (stack margins, CPU usage, timing overruns). task[] is indexed by eGpxRtosTask. */
typedef struct PACKED
{
    uint32_t                freeHeapSize;   //!< Heap high water mark, in free bytes remaining (lowest historical value)

    uint32_t                mallocSize;     //!< Total memory allocated using RTOS pvPortMalloc(), in bytes

    uint32_t                freeSize;       //!< Total memory freed using RTOS vPortFree(), in bytes

    rtos_task_t             task[GPX_RTOS_NUM_TASKS];   //!< Per-task status/profiling info, indexed by eGpxRtosTask

} gpx_rtos_info_t;

/** @brief (DID_EVB_RTOS_INFO) EVB-2 RTOS heap and per-task status/profiling information, broadcast for host-side monitoring of firmware task health (stack margins, CPU usage, timing overruns). task[] is indexed by eEvbRtosTask. */
typedef struct PACKED
{
    uint32_t                freeHeapSize;   //!< Heap high water mark, in free bytes remaining (lowest historical value)

    uint32_t                mallocSize;     //!< Total memory allocated using RTOS pvPortMalloc(), in bytes

    uint32_t                freeSize;       //!< Total memory freed using RTOS vPortFree(), in bytes

    rtos_task_t             task[EVB_RTOS_NUM_TASKS];   //!< Per-task status/profiling info, indexed by eEvbRtosTask

} evb_rtos_info_t;


/** @brief Timing statistics for a single profiled code section, updated by calls to profiler_start()/profiler_stop() around that section and periodically decayed by profiler_maintenance_1s(). One entry lives in runtime_profiler_t.p[]; which array index corresponds to which code section is defined by the caller. */
typedef struct
{
    uint32_t    runTimeUs;      //!< Duration of the most recently completed run, in microseconds (set by profiler_stop())

    uint32_t    maxRuntimeUs;   //!< Maximum runTimeUs observed since the last periodic reset in profiler_maintenance_1s(), in microseconds

    uint32_t    StartTimeUs;    //!< Timestamp of the most recent profiler_start() call, in microseconds

    uint32_t    startPeriodUs;  //!< Elapsed time between the two most recent profiler_start() calls, in microseconds (i.e. the section's call period)
} runtime_profile_t;

/** Number of hand-instrumented timing slots in runtime_profiler_t.p[]. */
#define RUNTIME_PROFILE_COUNT   4

/** @brief (DID_RUNTIME_PROFILER) INTERNAL USE ONLY. System runtime profiler: a fixed set of RUNTIME_PROFILE_COUNT timing slots used by firmware to track the runtime of hand-instrumented code sections via profiler_start()/profiler_stop(). Not intended for general application use. */
typedef struct
{
    runtime_profile_t   p[RUNTIME_PROFILE_COUNT];   //!< Timing statistics for each profiled code section, RUNTIME_PROFILE_COUNT entries
} runtime_profiler_t;



/** Valid baud rates for Inertial Sense hardware */
/** Valid CAN bus baud rates (in kbps) for can_config_t.can_baudrate_kbps (masked by
 *  CAN_BAUDRATE_KBPS_MASK). The numeric value of each enumerator is the baud rate itself. */
enum can_baudrate_t
{
    CAN_BAUDRATE_20_KBPS    = 20,   //!< 20 kbps
    CAN_BAUDRATE_33_KBPS    = 33,   //!< 33 kbps
    CAN_BAUDRATE_50_KBPS    = 50,   //!< 50 kbps
    CAN_BAUDRATE_83_KBPS    = 83,   //!< 83 kbps
    CAN_BAUDRATE_100_KBPS   = 100,  //!< 100 kbps
    CAN_BAUDRATE_125_KBPS   = 125,  //!< 125 kbps
    CAN_BAUDRATE_200_KBPS   = 200,  //!< 200 kbps
    CAN_BAUDRATE_250_KBPS   = 250,  //!< 250 kbps
    CAN_BAUDRATE_500_KBPS   = 500,  //!< 500 kbps
    CAN_BAUDRATE_1000_KBPS  = 1000, //!< 1000 kbps

    CAN_BAUDRATE_COUNT      = 10    //!< Number of valid baud rate options
};

/** Bit packed into can_config_t.can_setting (a uint16_t; valid baud rate values
 *  only use bits 0-9). When set, enables CAN-FD on FDCAN-capable hardware (IMX-6,
 *  GPX-1): frames with a payload > 8 bytes are sent using CAN FD framing with bit-rate
 *  switching (BRS). When clear (default), CAN runs classic-only (payload <= 8 bytes),
 *  matching legacy IMX-5 (bxCAN) behavior. Ignored on IMX-5, which has no FD hardware. */
#define CAN_BAUDRATE_KBPS_FD_ENABLE     (uint16_t)0x8000
/** Mask isolating the baud rate (in kbps) from can_setting, excluding the
 *  CAN_BAUDRATE_KBPS_FD_ENABLE flag bit. */
#define CAN_BAUDRATE_KBPS_MASK          (uint16_t)0x7FFF

/** (DID_CAN_CONFIG / DID_CANFD_CONFIG) CAN bus broadcast configuration: per-message period
 *  multipliers, transmit addresses, baud rate, and receive address. */
typedef struct PACKED
{
    uint16_t                can_period_mult[NUM_CIDS];       //!< Broadcast period multiple for each CAN message. 0 disables the message. Indices 0..NUM_CIDS-1 correspond to classic can_cid_t values. In CAN-FD mode the same array is reused: indices 0..NUM_FDCIDS-1 correspond to canfd_cid_t values (FDCID_INS_1=0, FDCID_INS_2=1, …). NUM_FDCIDS < NUM_CIDS so there is no overlap.

    uint32_t                can_transmit_address[NUM_CIDS];  //!< Transmit address for each CAN message. Indices 0..NUM_CIDS-1 correspond to classic can_cid_t values. In CAN-FD mode indices 0..NUM_FDCIDS-1 correspond to canfd_cid_t values and are validated / defaulted by CAN_init() when FD is enabled.

    uint16_t                can_setting;                     //!< Baud rate (kbps) (See can_baudrate_t for valid baud rates). Bit 15 (CAN_BAUDRATE_KBPS_FD_ENABLE) enables CAN-FD on capable hardware.

    uint32_t                can_receive_address;             //!< Receive address

} can_config_t;

#if defined(INCLUDE_LUNA_DATA_SETS)
#include "luna_data_sets.h"
#endif

/** Union of every DID payload type in this file, used to size/alias a generic packet
 *  data buffer without a separate case per DID. Exactly one member is valid at a time,
 *  selected by the packet's DID (see eDataIDs); see each member's own type for details. */
typedef union PACKED
{
    dev_info_t                      devInfo;        //!< DID_DEV_INFO / DID_EVB_DEV_INFO / DID_GPX_DEV_INFO
    ins_1_t                         ins1;           //!< DID_INS_1
    ins_2_t                         ins2;           //!< DID_INS_2
    ins_3_t                         ins3;           //!< DID_INS_3
    ins_4_t                         ins4;           //!< DID_INS_4
    imu_t                           imu;            //!< DID_IMU / DID_IMU_RAW / DID_REFERENCE_IMU
    imus_t                          imus;           //!< DID_IMUS / DID_IMUS_RAW / DID_IMUS_UNCAL
    magnetometer_t                  mag;            //!< DID_MAGNETOMETER / DID_REFERENCE_MAGNETOMETER
    mag_cal_t                       magCal;         //!< DID_MAG_CAL
    barometer_t                     baro;           //!< DID_BAROMETER
    wheel_encoder_t                 wheelEncoder;   //!< DID_WHEEL_ENCODER
    ground_vehicle_t                groundVehicle;  //!< DID_GROUND_VEHICLE
    pos_measurement_t               posMeasurement; //!< DID_POSITION_MEASUREMENT
    pimu_t                          pImu;           //!< DID_PIMU / DID_REFERENCE_PIMU
    gnss_pos_t                      gnssPos;        //!< DID_GNSS1_POS / DID_GNSS2_POS / DID_GNSS1_RTK_POS / DID_GNSS1_RCVR_POS
    gnss_vel_t                      gnssVel;        //!< DID_GNSS1_VEL / DID_GNSS2_VEL
    gnss_sat_t                      gnssSat;        //!< DID_GNSS1_SAT / DID_GNSS2_SAT
    gnss_sig_t                      gnssSig;        //!< DID_GNSS1_SIG / DID_GNSS2_SIG
    gnss_version_t                  gnssVer;        //!< DID_GNSS1_VERSION / DID_GNSS2_VERSION
    gnss_rtk_rel_t                  gnssRtkRel;     //!< DID_GNSS1_RTK_POS_REL / DID_GNSS2_RTK_CMP_REL
    gnss_rtk_misc_t                 gnssRtkMisc;    //!< DID_GNSS1_RTK_POS_MISC / DID_GNSS2_RTK_CMP_MISC
    inl2_states_t                   inl2States;     //!< DID_INL2_STATES
    inl2_ned_sigma_t                inl2NedSigma;   //!< DID_INL2_NED_SIGMA
    nvm_flash_cfg_t                 flashCfg;       //!< DID_FLASH_CONFIG
    survey_in_t                     surveyIn;       //!< DID_SURVEY_IN
    sys_params_t                    sysParams;      //!< DID_SYS_PARAMS
    sys_sensors_t                   sysSensors;     //!< DID_SYS_SENSORS
    rtos_info_t                     rtosInfo;       //!< DID_RTOS_INFO
    gpx_rtos_info_t                 gRtosInfo;      //!< DID_GPX_RTOS_INFO
    gnss_raw_t                      gnssRaw;        //!< DID_GNSS_BASE_RAW / DID_GNSS1_RAW / DID_GNSS2_RAW
    sys_sensors_adc_t               sensorsAdc;     //!< DID_SENSORS_ADC / DID_SENSORS_ADC_SIGMA
    rmc_t                           rmc;            //!< DID_RMC / DID_GPX_RMC
    evb_status_t                    evbStatus;      //!< DID_EVB_STATUS
    infield_cal_t                   infieldCal;     //!< DID_INFIELD_CAL
    gpx_status_t                    gpxStatus;      //!< DID_GPX_STATUS
    debug_array_t                   imxDebugArray;  //!< DID_DEBUG_ARRAY / DID_EVB_DEBUG_ARRAY
    debug_array_t                   gpxDebugArray;  //!< DID_GPX_DEBUG_ARRAY
    port_monitor_t                  portMonitor;    //!< DID_PORT_MONITOR / DID_GPX_PORT_MONITOR
    did_event_t                     event;          //!< DID_EVENT
    manufacturing_info_t            manfInfo;       //!< DID_MANUFACTURING_INFO
    bit_t                           bit;            //!< DID_BIT

#if defined(INCLUDE_LUNA_DATA_SETS)
    evb_luna_velocity_control_t     wheelController; //!< Luna-specific wheel velocity controller data set
#endif
} uDatasets;

/** Union of just the four INS navigation-solution output variants (DID_INS_1..DID_INS_4),
 *  used where a caller only needs to alias/store whichever single INS output type is enabled. */
typedef union PACKED
{
    ins_1_t                     ins1;   //!< DID_INS_1: euler attitude, NED position
    ins_2_t                     ins2;   //!< DID_INS_2: quaternion attitude, LLA + ellipsoid altitude
    ins_3_t                     ins3;   //!< DID_INS_3: quaternion attitude, LLA + MSL altitude
    ins_4_t                     ins4;   //!< DID_INS_4: quaternion attitude, ECEF position
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
#define BE_SWAP64F(_i)  flipDoubleCopy(_i)
#define BE_SWAP32F(_i)  flipFloatCopy(_i)
#define BE_SWAP32(_i)   (SWAP32(_i))
#define BE_SWAP16(_i)   (SWAP16(_i))
#define LE_SWAP64F(_i)  (_i)
#define LE_SWAP32F(_i)  (_i)
#define LE_SWAP32(_i)   (_i)
#define LE_SWAP16(_i)   (_i)
#else // little endian
#define BE_SWAP64F(_i)  (_i)
#define BE_SWAP32F(_i)  (_i)
#define BE_SWAP32(_i)   (_i)
#define BE_SWAP16(_i)   (_i)
#define LE_SWAP64F(_i)  flipDoubleCopy(_i)
#define LE_SWAP32F(_i)  flipFloatCopy(_i)
#define LE_SWAP32(_i)   (SWAP32(_i))
#define LE_SWAP16(_i)   (SWAP16(_i))
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
extern const uint64_t g_didToRmcBit[DID_COUNT];
uint64_t didToRmcBit(uint32_t dataId, uint64_t defaultRmcBits, uint64_t devInfoRmcBits);

/** DID to NMEA RMC bit look-up table */
extern const uint64_t g_didToNmeaRmcBit[DID_COUNT];

/** DID to GPX RMC bit look-up table */
extern const uint64_t g_gpxDidToGrmcBit[DID_COUNT];
extern const uint16_t g_gpxGRMCPresetLookup[GRMC_BIT_POS_COUNT];

#ifndef GPX_1

/*
Convert gnssID to ubx gnss indicator (ref [2] 25)

@param gnssID gnssID of satellite
@return ubx gnss indicator
*/
int ubxSys(int gnssID);

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

void profiler_start(runtime_profile_t *p, uint32_t timeUs);
void profiler_stop(runtime_profile_t *p, uint32_t timeUs);
void profiler_maintenance_1s(runtime_profiler_t *p);

int manufacturing_info_checkRequirementsToWrite(manufacturing_info_t *newInfo);


#ifdef __cplusplus
}
#endif

#endif // DATA_SETS_H



/****************************************
 * PROPOSED CHANGES FOR PROTOCOL IS V3.0
 *
 * - Remove GNSS_STATUS_NUM_SATS_USED_MASK bits in eGnssStatus this is reported in satsUsed in gnss_pos_t.
 * - Move spoofing/jamming status into gnss_pos_t.status and reclaim gnss_pos_t.status2 as resevered.
 * - Change $INFO to conform to NMEA 0183 standard. $INFO is a proprietary message and should start with $P and have max of 79 characters. see SN-6231
 *
 * - GNSS rename references to GPSn to GNSSn
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 ****************************************/
