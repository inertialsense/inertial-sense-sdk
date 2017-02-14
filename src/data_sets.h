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

#ifdef __cplusplus
extern "C" {
#endif

// with this you can tell the compiler not to insert padding - be VERY careful as this can cause hard-faults if your struct or class is not 4 byte (or 8 byte for 64 bit) aligned
#if defined(_MSC_VER)
#define PUSH_PACK_NONE __pragma(pack(push, 1))
#define POP_PACK_NONE __pragma(pack(pop))
#define PACK_ONE
#elif defined(AVR)
// there is a bug in the AVR compiler in debug mode that can generate bad instructions, crashing the process - until this is figured out, debug on AVR cannot pack globally
#if DEBUG
#define PUSH_PACK_NONE
#define POP_PACK_NONE
#define PACK_ONE __attribute__ ((packed))
#else
#define PUSH_PACK_NONE _Pragma("pack(1)")
#define POP_PACK_NONE _Pragma("pack(0)")
#define PACK_ONE
#endif
#else
#define PUSH_PACK_NONE _Pragma("pack(push, 1)")
#define POP_PACK_NONE _Pragma("pack(pop)")
#define PACK_ONE
#endif

#ifndef INLINE
#if defined(_MSC_VER)
#define INLINE __inline 
#else
#define INLINE inline
#endif
#endif

/*! Maximum number of messages that may be broadcast simultaneously */
#define MAX_NUM_BCAST_MSGS 10

/*! Maximum number of satellite channels */
#define MAX_NUM_SAT_CHANNELS 50

/*! Maximum length of device info manufacturer string (must be a multiple of 4) */
#define DEVINFO_MANUFACTURER_STRLEN 24
#define DEVINFO_ADDINFO_STRLEN 24

/*! Start of extended external ids - NEVER change this value */
#define DID_EXTERNAL_EXTENDED_START 8192

#ifndef UNMASK
#define UNMASK(_word, _prefix) (((_word) & (_prefix##_MASK)) >> (_prefix##_SHIFT))
#endif

#ifndef MASK
#define MASK(_prefix, _val) ((_prefix##_MASK) & ((_val) << (_prefix##_SHIFT)))
#endif

#ifndef SWAP16
#define SWAP16(v) ((uint16_t)(((uint16_t)(v) >> 8) | ((uint16_t)(v) << 8)))
#endif

#ifndef SWAP32
#if defined(__GNUC__)
#define SWAP32 __builtin_bswap32
#elif defined(__ICCAVR32__)
#define SWAP32 __swap_bytes
#elif defined(_WIN32)
#include "intrin.h"
#define SWAP32 _byteswap_ulong
#else
#define SWAP32(v) ((uint32_t)(((uint32_t)SWAP16((uint32_t)(v) >> 16)) | ((uint32_t)SWAP16((uint32_t)(v)) << 16)))
#endif
#endif

#ifndef _MAX 
#define _MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef _MIN
#define _MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef _LIMIT2
#define _LIMIT2(x, xmin, xmax) { if ((x) < (xmin)) { (x) = (xmin); } else { if ((x) > (xmax)) { (x) = (xmax); } } }
#endif

#ifndef _ARRAY_BYTE_COUNT
#define _ARRAY_BYTE_COUNT(a) sizeof(a)
#endif

#ifndef _ARRAY_ELEMENT_COUNT
#define _ARRAY_ELEMENT_COUNT(a) (sizeof(a) / sizeof(a[0]))
#endif

#ifndef OFFSETOF
#define OFFSETOF offsetof//(TYPE, MEMBER) ((uint8_t)&(((TYPE*)0)->MEMBER))
#endif

#ifdef RTK_EMBEDDED

#include "rtklib_embedded.h"

#endif

// *****************************************
// ****** NEVER REORDER THESE VALUES! ******
// *****************************************
/*! Data identifiers - these are unsigned int and #define because enum are signed according to C standard */
typedef uint32_t eDataIDs;

/*! 0 : NULL (INVALID) */
#define DID_NULL (eDataIDs)0

/*! 1 : (dev_info_t) Device information */
#define DID_DEV_INFO (eDataIDs)1
	
/*! 2: (imu_t) Inertial measurement unit data: calibrated gyro, accelerometer, magnetometer, and barometric pressure. */
#define DID_IMU_1 (eDataIDs)2

/*! 3: (delta_theta_vel_t) Conning and sculling integral in body/IMU frame. Updated at IMU rate. */
#define DID_DELTA_THETA_VEL (eDataIDs)3

/*! 4 : (ins_1_t) Inertial navigation data with euler attitude and NED from reference LLA */
#define DID_INS_1 (eDataIDs)4

/*! 5 : (ins_2_t) Inertial navigation data with quaternion NED to body attitude */
#define DID_INS_2 (eDataIDs)5

/*! 6 : (gps_t) GPS data */
#define DID_GPS (eDataIDs)6

/*! 7 : (config_t) Configuration data */
#define DID_CONFIG (eDataIDs)7

/*! 8 : (ascii_msgs_t) Broadcast period for ASCII messages */
#define DID_ASCII_BCAST_PERIOD (eDataIDs)8

/*! 9 : (ins_misc_t) Other INS data */
#define DID_INS_MISC (eDataIDs)9

/*! 10 : (sys_params_t) System parameters */
#define DID_SYS_PARAMS (eDataIDs)10

/*! 11 : (sys_sensors_t) System sensor information (10) */
#define DID_SYS_SENSORS (eDataIDs)11

/*! 12 : (nvm_flash_cfg_t) Flash memory configuration */
#define DID_FLASH_CONFIG (eDataIDs)12

/*! 13 : (gps_rssi_t) GPS received signal strength indicator */
#define DID_GPS_RSSI (eDataIDs)13

/*! 14 : (gps_nav_poslla_t) GPS velocity data */
#define DID_GPS_POS (eDataIDs)14

/*! 15 : (gps_nav_velned_t) GPS velocity data */
#define DID_GPS_VEL (eDataIDs)15

/*! 16 : (io_t) I/O: Servos */
#define DID_IO (eDataIDs)16

/*! 17 : (io_servos_t) I/O: Servos Pulse Width Modulation (PWM) */
#define DID_IO_SERVOS_PWM (eDataIDs)17

/*! 18 : (io_servos_t) I/O: Servos Pulse Position Modulation (PPM), single wire servo pulse train */
#define DID_IO_SERVOS_PPM (eDataIDs)18

/*! 19 : (mag_cal_t) Magnetometer calibration */
#define DID_MAGNETOMETER_CAL (eDataIDs)19

/*! 20 : (ins_res_t) Resources */
#define DID_INS_RESOURCES (eDataIDs)20

/*! 21 : Differential GPS Correction */
#define DID_DGPS_CORRECTION (eDataIDs)21

/*! 22 : (Rtk data - packet parsing is done in RTKLib specific code) */
#define DID_RTK (eDataIDs)22

/*! INTERNAL USE ONLY (23 feature_bits_t) */
#define DID_FEATURE_BITS (eDataIDs)23

/*! INTERNAL USE ONLY (24 sensors_w_temp_t) */
#define DID_SENSORS_IS1 (eDataIDs)24

/*! INTERNAL USE ONLY (25 sensors_w_temp_t) */
#define DID_SENSORS_IS2 (eDataIDs)25

/*! INTERNAL USE ONLY (26 sensors_t) */
#define DID_SENSORS_TC_BIAS (eDataIDs)26

/*! INTERNAL USE ONLY (27 sensor_bias_t) */
#define DID_SENSORS_CF_BIAS (eDataIDs)27

/*! INTERNAL USE ONLY (28 sys_sensors_adc_t) */
#define DID_SENSORS_ADC (eDataIDs)28

/*! INTERNAL USE ONLY (29 sensor_compensation_t) */
#define DID_SCOMP (eDataIDs)29

/*! INTERNAL USE ONLY (30 ins_params_t) */
#define DID_INS_PARAMS (eDataIDs)30

/*! INTERNAL USE ONLY (31 obs_params_t) */
#define DID_OBS_PARAMS (eDataIDs)31

/*! INTERNAL USE ONLY (32 hdw_params_t) */
#define DID_HDW_PARAMS (eDataIDs)32

/*! INTERNAL USE ONLY (33 nvr_manage_t) */
#define DID_NVR_MANAGE_USERPAGE (eDataIDs)33

/*! INTERNAL USE ONLY (34 nvm_group_sn_t) */
#define DID_NVR_USERPAGE_SN (eDataIDs)34

/*! INTERNAL USE ONLY (35 nvm_group_0_t) */
#define DID_NVR_USERPAGE_G0 (eDataIDs)35

/*! INTERNAL USE ONLY (36 nvm_group_1_t) */
#define DID_NVR_USERPAGE_G1 (eDataIDs)36

/*! INTERNAL USE ONLY (37 debug_string_t) */
#define DID_DEBUG_STRING (eDataIDs)37

/*! INTERNAL USE ONLY (38 rtos_info_t) */
#define DID_RTOS_INFO (eDataIDs)38

/*! INTERNAL USE ONLY (39 debug_array_t) */
#define DID_DEBUG_ARRAY (eDataIDs)39

/*! INTERNAL USE ONLY (40 sensors_mpu_w_temp_t) */
#define DID_SENSORS_CAL1 (eDataIDs)40

/*! INTERNAL USE ONLY (41 sensors_mpu_w_temp_t) */
#define DID_SENSORS_CAL2 (eDataIDs)41

/*! INTERNAL USE ONLY (42 sensor_cal_t) */
#define DID_CAL_SC (eDataIDs)42

/*! INTERNAL USE ONLY (43 sensor_cal_mpu_t) */
#define DID_CAL_SC1 (eDataIDs)43

/*! INTERNAL USE ONLY (44 sensor_cal_mpu_t) */
#define DID_CAL_SC2 (eDataIDs)44

/*! INTERNAL USE ONLY (45 sys_sensors_t) */
#define DID_SYS_SENSORS_SIGMA (eDataIDs)45

/*! INTERNAL USE ONLY (46 sys_sensors_adc_t) */
#define DID_SENSORS_ADC_SIGMA (eDataIDs)46

/*! INTERNAL USE ONLY (47 ins_dev_1_t) */
#define DID_INS_DEV_1 (eDataIDs)47

/*! (48 ekf_states_t) */
#define DID_EKF_STATES (eDataIDs)48

/*! INTERNAL USE ONLY (49 float[24]) */
#define DID_EKF_COVARIANCE (eDataIDs)49

/*! INTERNAL USE ONLY (50 ekf_innov_t) */
#define DID_EKF_INNOV (eDataIDs)50

/*! INTERNAL USE ONLY (51 ekf_innov_var_t) */
#define DID_EKF_INNOV_VAR (eDataIDs)51

/*! 52 : (magnetometer_t) Magnetometer sensor data */
#define DID_MAGNETOMETER_1 (eDataIDs)52

/*! 53 : (barometer_t) Barometric pressure sensor data */
#define DID_BAROMETER (eDataIDs)53

/*! 54: (imu_t) 2nd inertial measurement unit data: calibrated gyroscope and accelerometer. */
#define DID_IMU_2 (eDataIDs)54

/*! 55 : (magnetometer_t) 2nd magnetometer sensor data */
#define DID_MAGNETOMETER_2 (eDataIDs)55

/*! 56 : (gps_version_t) GPS version info */
#define DID_GPS_VERSION (eDataIDs)56

/*! 57 : () Unit test for communications manager  */
#define DID_COMMUNICATIONS_LOOPBACK (eDataIDs)57

/*! 58: (dual_imu_t) dual inertial measurement units data: calibrated gyroscope and accelerometer. */
#define DID_DUAL_IMU (eDataIDs)58

// Adding a new data id?
// 1] Add it above and increment the previous number, include the matching data structure type in the comments
// 2] Add flip doubles and flip strings entries in data_sets.c
// 3] Increment DID_COUNT
// 4] Test!

/*! Count of data ids - make sure to increment if you add a new data id! */
#define DID_COUNT (eDataIDs)59

/*! Maximum number of data ids */
#define DID_MAX_COUNT 256

// END DATA IDENTIFIERS --------------------------------------------------------------------------

/*! Defines the 4 parts to the communications version. Major changes involve changes to the com manager. Minor changes involve additions to data structures */
// #define PROTOCOL_VERSION_CHAR0 1        // Major (in com_manager.h)
// #define PROTOCOL_VERSION_CHAR1 0
#define PROTOCOL_VERSION_CHAR2 (0x000000FF&DID_COUNT)
#define PROTOCOL_VERSION_CHAR3 6         // Minor (in data_sets.h)

/*! INS status flags */
enum eInsStatus
{
	/*! INS attitude is coarse aligned */
	INS_STATUS_ATT_ALIGNED_COARSE			= (int)0x00000001,
	/*! INS velocity is coarse aligned */
	INS_STATUS_VEL_ALIGNED_COARSE			= (int)0x00000002,
	/*! INS position is coarse aligned */
	INS_STATUS_POS_ALIGNED_COARSE			= (int)0x00000004,
	/*! INS coarse aligned mask */
	INS_STATUS_ALIGNED_COARSE_MASK			= (int)0x00000007,

	/*! INS attitude is aligned */
	INS_STATUS_ATT_ALIGNED					= (int)0x00000010,
	/*! INS velocity is aligned */
	INS_STATUS_VEL_ALIGNED					= (int)0x00000020,
	/*! INS position is aligned */
	INS_STATUS_POS_ALIGNED					= (int)0x00000040,
	/*! INS aligned mask */
	INS_STATUS_ALIGNED_MASK					= (int)0x00000070,
	/*! INS attitude is fine aligned */
	INS_STATUS_ATT_ALIGNED_FINE				= (int)0x00000080,
	/*! INS all fine aligned mask */
	INS_STATUS_FINE_ALIGNED_MASK			= (int)0x000000FF,

	/*! INS attitude is aligning from GPS */
	INS_STATUS_ATT_ALIGNING_GPS				= (int)0x00000100,
	/*! INS velocity is aligning */
	INS_STATUS_VEL_ALIGNING					= (int)0x00000200,
	/*! INS position is aligning */
	INS_STATUS_POS_ALIGNING					= (int)0x00000400,
	/*! INS attitude is aligning from Mag */
	INS_STATUS_ATT_ALIGNING_MAG				= (int)0x00000800,
	/*! INS aligning mask */
	INS_STATUS_ALIGNING_MASK				= (int)0x00000F00,

	/*! Reference (GPS) position is valid */
	INS_STATUS_REF_POS_VALID				= (int)0x00001000,
	/*! Reference (GPS) Velocity is valid */
	INS_STATUS_REF_VEL_VALID				= (int)0x00002000,
	/*! Reference (GPS) Acceleration is valid */
	INS_STATUS_REF_ACC_VALID				= (int)0x00004000,

	/*! INS accelerating in horizontal plane. */
	INS_STATUS_INS_ACC_2D					= (int)0x00010000,
	/*! Reference (GPS) accelerating in horizontal plane. */
	INS_STATUS_REF_ACC_2D					= (int)0x00020000,
	/*! Reference (GPS) moving in horizontal plane. */
	INS_STATUS_REF_VEL_2D					= (int)0x00040000,
	
	/*! Startup static aligning */
	INS_STATUS_STARTUP_STATIC_ALIGNING		= (int)0x00100000,
	/*! Startup dynamic alignment complete */
	INS_STATUS_STARTUP_DYNAMIC_ALIGNED		= (int)0x00200000,
	/*! Magnetometer calibration running */
	INS_STATUS_CALIBRATING_MAG				= (int)0x00400000,

	/*! INS PQR bias estimation running */
	INS_STATUS_BIAS_EST_PQR					= (int)0x01000000,
	/*! INS acceleration bias estimation running */
	INS_STATUS_BIAS_EST_ACC					= (int)0x02000000,
	/*! INS barometric altimeter bias estimation running */
	INS_STATUS_BIAS_EST_BARO				= (int)0x04000000,
	/*! INS bias estimation mask */
	INS_STATUS_BIAS_EST_MASK				= (int)0x07000000,
	/*! INS PQR bias estimation stable */
	INS_STATUS_BIAS_EST_PQR_STABLE			= (int)0x08000000,

	/*! INS has rotated while not translating/moving */
	INS_STATUS_STATIONARY_ROTATION			= (int)0x10000000,
	/*! General fault */
	INS_STATUS_GENERAL_FAULT				= (int)0x80000000,
};

/*! Hardware status flags */
enum eHardwareStatus
{
	/*! Gyro motion detected sigma */
	HDW_STATUS_MOTION_GYR_SIG				= (int)0x00000001,
	/*! Gyro motion detected sigma inverse */
	HDW_STATUS_MOTION_GYR_SIG_INV			= (int)0xFFFFFFFE,
	/*! Accelerometer motion detected sigma */
	HDW_STATUS_MOTION_ACC_SIG				= (int)0x00000002,
	/*! Accelerometer motion detected sigma inverse */
	HDW_STATUS_MOTION_ACC_SIG_INV			= (int)0xFFFFFFFD,
	/*! Gyro motion detected deviation */
	HDW_STATUS_MOTION_GYR_DEV				= (int)0x00000004,
	/*! Gyro motion detected deviation inverse */
	HDW_STATUS_MOTION_GYR_DEV_INV			= (int)0xFFFFFFFB,
	/*! Accelerometer motion detected deviation */
	HDW_STATUS_MOTION_ACC_DEV				= (int)0x00000008,
	/*! Accelerometer motion detected deviation inverse */
	HDW_STATUS_MOTION_ACC_DEV_INV			= (int)0xFFFFFFF7,
	/*! Unit is moving and NOT stationary */
	HDW_STATUS_MOTION_SIG_MASK				= (int)0x00000003,
	/*! Motion mask */
	HDW_STATUS_MOTION_MASK					= (int)0x0000000F,

	/*! Sensor saturation on gyro 1 */
	HDW_STATUS_SATURATION_GYR1				= (int)0x00000100,
	/*! Sensor saturation on accelerometer 1 */
	HDW_STATUS_SATURATION_ACC1				= (int)0x00000200,
	/*! Sensor saturation on magnetometer 1 */
	HDW_STATUS_SATURATION_MAG1				= (int)0x00000400,
	/*! Sensor saturation on barometric pressure */
	HDW_STATUS_SATURATION_BARO				= (int)0x00000800,

	/*! Sensor saturation on gyro 2 */
	HDW_STATUS_SATURATION_GYR2				= (int)0x00001000,
	/*! Sensor saturation on accelerometer 2 */
	HDW_STATUS_SATURATION_ACC2				= (int)0x00002000,
	/*! Sensor saturation on magnetometer 2 */
	HDW_STATUS_SATURATION_MAG2				= (int)0x00004000,
	/*! Sensor saturation happened in past for MPU1 and MPU2 */
	HDW_STATUS_SATURATION_HISTORY			= (int)0x00008000,
	/*! Sensor saturation mask */
	HDW_STATUS_SATURATION_MASK				= (int)0x0000FF00,
	/*! Bitwise inverse of sensor saturation mask */
	HDW_STATUS_SATURATION_MASK_INV			= (int)0xFFFF00FF,
	/*! Sensor saturation offset */
	HDW_STATUS_SATURATION_OFFSET			= 8,

	/*! Communications Tx buffer limited */
	HDW_STATUS_ERR_COM_TX_LIMITED			= (int)0x00010000,
	/*! Communications Rx buffer overrun */
	HDW_STATUS_ERR_COM_RX_OVERRUN			= (int)0x00020000,
	/*! GPS Tx buffer limited */
	HDW_STATUS_ERR_GPS_TX_LIMITED			= (int)0x00040000,
	/*! GPS Tx buffer overrun */
	HDW_STATUS_ERR_GPS_RX_OVERRUN			= (int)0x00080000,

	/*! Automatic baudrate detection fault */
	HDW_STATUS_ERR_AUTOBAUD_FAULT			= (int)0x00100000,
	/*! Communications read fault */
	HDW_STATUS_ERR_COM_READ_FAULT			= (int)0x00200000,

	/*! Auto-baud negotiated */
	HDW_STATUS_AUTOBAUD_DETECTED			= (int)0x01000000,

	/*! Watchdog reset fault */
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

enum
{
	SLOG_DISABLED			= 0,
	SLOG_W_INS1				= 1,	// Log INS1, INS_PARAMS, SYS_PARAMS, INS_INPUT, BIASES, OBS_PARAMS, GPS_POS
	SLOG_W_INS2				= 2,	// Log INS2, "
	SLOG_REDUCED_INS1		= 20,	// Log INS1, INS_PARAMS, SYS_PARAMS
	SLOG_REDUCED_INS2		= 21,	// Log INS2, "
};

enum eSysConfigBits
{
	/*! Disable automatic baudrate detection on startup */
	SYS_CFG_BITS_DISABLE_AUTOBAUD					= (int)0x00000001,

	/*! Disable automatic mag calibration */
	SYS_CFG_BITS_DISABLE_MAG_AUTO_CAL				= (int)0x00000002,

	/*! Disable LEDs */
	SYS_CFG_BITS_DISABLE_LEDS						= (int)0x00000004,

	/*! Send communications (com manager) data as little endian */
	SYS_CFG_BITS_COM_MANAGER_SEND_AS_LITTLE_ENDIAN	= (int)0x00000008,

	/*! Enable com manager pass through of GPS data */
	SYS_CFG_BITS_ENABLE_COM_MANAGER_PASS_THROUGH	= (int)0x00000010
};

PUSH_PACK_NONE

/*! (DID_DEV_INFO) Device information */
typedef struct
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


/*! (DID_INS_1) INS data with euler attitude and NED from reference LLA */
typedef struct
{
	/*! Weeks since January 1st, 1980 */
	uint32_t				week;
	
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/*! INS status flags (eInsStatus). Copy of DID_SYS_PARAMS.iStatus */
	uint32_t				iStatus;

	/*! Hardware status flags (eHardwareStatus). Copy of DID_SYS_PARAMS.hStatus */
	uint32_t				hStatus;

	/*! Euler angles: roll, pitch, yaw in radians */
	float					theta[3];

	/*! Velocity U, V, W in meters per second */
	float					uvw[3];

	/*! WGS84 Latitude, longitude, height above ellipsoid in meters (not MSL) */
	double					lla[3];

	/*! North, east and down offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude */
	float					ned[3];
} ins_1_t;


/*! (DID_INS_2) INS data with quaternion attitude */
typedef struct
{
	/*! Weeks since January 1st, 1980 */
	uint32_t				week;
	
	/*! Time of week (since Sunday morning) in seconds, GMT */
	double					timeOfWeek;

	/*! INS status flags (eInsStatus). Copy of DID_SYS_PARAMS.iStatus */
	uint32_t				iStatus;

	/*! Hardware status flags (eHardwareStatus). Copy of DID_SYS_PARAMS.hStatus */
	uint32_t				hStatus;

	/*! Quaternion body rotation with respect to NED: W, X, Y, Z */
	float					qn2b[4];

	/*! Velocity U, V, W in meters per second */
	float					uvw[3];

	/*! WGS84 Latitude, longitude, height above ellipsoid in meters (not MSL) */
	double					lla[3];
} ins_2_t;


typedef struct
{
	/*! Gyroscope P, Q, R in radians / second */
	float                   pqr[3];

	/*! Acceleration X, Y, Z in meters / second squared */
	float                   acc[3];
} imus_t;


/*! (DID_IMU_1, DID_IMU_2) Inertial Measurement Unit (IMU) data */
typedef struct
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/*! Inertial Measurement Unit (IMU) */
	imus_t					I;
} imu_t;


/*! (DID_DUAL_IMU) Dual Inertial Measurement Units (IMUs) data */
typedef struct
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;

	/*! Inertial Measurement Units (IMUs) */
	imus_t                  I[2];
} dual_imu_t;


/*! (DID_MAGNETOMETER_1, DID_MAGNETOMETER_2) Magnetometer sensor data */
typedef struct
{
	/*! Time since boot up in seconds.  Convert to GPS time of week by adding gps.towOffset */
	double                  time;
	
	/*! Magnetometers in Gauss */
	float                   mag[3];
} magnetometer_t;


/*! (DID_BAROMETER) Barometric pressure sensor data */
typedef struct
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
typedef struct
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
typedef struct
{
	/*! Number of week since January 1st, 1980 */
	uint32_t                week;
	
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! GPS status: [7:0] number of satellites used in solution, [15:8] status flags, [23:16] fix type */
	uint32_t                status;

	/*! Carrier to noise ratio (receiver signal strength) of strongest satellite (dBHz) */
	uint32_t                cno;

	/*! WGS84 Latitude, longitude, height above ellipsoid in meters (not geoid / MSL) */
	double					lla[3];

	/*! Mean sea level (MSL) height above geoid altitude in meters */
	float					hMSL;

	/*! Horizontal accuracy in meters */
	float					hAcc;

	/*! Vertical accuracy in meters */
	float					vAcc;

	/*! Position dilution of precision in meters */
	float                   pDop;
} gps_nav_poslla_t;

	
/*! (DID_GPS_VEL) GPS velocity */
typedef struct
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
typedef struct
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
typedef struct
{
	/*! Satellite identifier */
	uint32_t				svId;

	/*! Carrier to noise ratio (receiver signal strength, dBHz) */
	uint32_t				cno;
} gps_sat_info_t;


/*! (DID_GPS_RSSI) GPS received signal strength indicator */
typedef struct
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! Number of satellites in the sky */
	uint32_t				numSats;

	/*! Satellite information list */
	gps_sat_info_t			info[MAX_NUM_SAT_CHANNELS];
} gps_rssi_t;


/*! (DID_GPS_VERSION) GPS version strings */
typedef struct
{
	uint8_t                 swVersion[30];
	uint8_t                 hwVersion[10];
	uint8_t                 extension[30];
	uint8_t					reserved[2];		// ensure 32 bit aligned in memory
} gps_version_t;


/*! (DID_ASCII_BCAST_PERIOD) ASCII broadcast periods. This data structure (when it is included in the sCommData struct) is zeroed out on stop_all_broadcasts */ 
typedef struct
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
} ascii_msgs_t;

/*! Generic 1 axis sensor */
typedef struct
{
	/*! Time in seconds */
	double                  time;

	/*! Three axis sensor */
	float                   val;
} gen_1axis_sensor_t;

/*! Generic 3 axis sensor */
typedef struct
{
	/*! Time in seconds */
	double                  time;

	/*! Three axis sensor */
	float                   val[3];
} gen_3axis_sensor_t;

/*! Generic 3 axis sensor */
typedef struct
{
	/*! Time in seconds */
	double                  time;

	/*! Three axis sensor */
	double                  val[3];
} gen_3axis_sensord_t;


/*! (DID_SYS_SENSORS) Output from system sensors */
typedef struct
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


// (DID_EKF_STATES) INS Extended Kalman Filter (EKF) states
typedef struct
{	
	double                  time;					// (s)     Time since boot up in seconds
	float					qe2b[4];                //         Quaternion body rotation with respect to ECEF
	float					ve[3];					// (m/s)   Velocity in ECEF frame
	double					ecef[3];					// (m)     Position in ECEF frame
	float					biasPqr[3];	            // (rad/s) Gyro bias
	float					biasAcc[3];	            // (m/s^2) Accelerometer bias
	float					biasBaro;               // (m)     Barometer bias
	float					magDec;                 // (rad)   Magnetic declination
	float					magInc;                 // (rad)   Magnetic inclination
} ekf_states_t;


/*! Sensor state variables */
typedef struct
{
	/*! Latitude, longitude and height above ellipsoid in radians, radians and meters */
	double                  lla[3];

	/*! Velocities in body frames of X, Y and Z in meters per second */
	float                   uvw[3];

	/*! Quaternion body rotation with respect to NED: W, X, Y, Z */
	float					qn2b[4];
} state_vars_t;

/*! (DID_INS_MISC) INS Misc data */
typedef struct
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
typedef struct
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
typedef struct
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! System status 1 flags (eInsStatus) */
	uint32_t                iStatus;

	/*! System status 2 flags (eHardwareStatus) */
	uint32_t                hStatus;

	/*! Attitude alignment detection */
	float				    alignAttDetect;

	/*! Attitude alignment error (approximation) in radians */
	float				    alignAttError;

	/*! Velocity alignment error in meters per second */
	float				    alignVelError;

	/*! Position alignment error in meters per second */
	float				    alignPosError;

	/*! Sample period in milliseconds */
	uint32_t				sampleDtMs;

	/*! Solution update period in milliseconds */
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


/*! (DID_CONFIG) Configuration functions */
typedef struct
{
	/*! Set to 1 to reset processor into bootloader mode */
	uint32_t                enBootloader;

	/*! Set to 1 to log solution input */
	uint32_t                sLogCtrl;

	/*! Set to 1 to enable sensor stats */
	uint32_t                enSensorStats;

	/*! Set to 1 to enable RTOS stats */
	uint32_t                enRTOSStats;

	/*! Set to 1 to enable GPS low-level configuration */
	uint32_t                gpsStatus;
} config_t;

#define NUM_SERVOS			8
#define SERVO_PULSE_US_MIN	700
#define SERVO_PULSE_US_MAX	2300

/*! (DID_IO) Input/Output */
typedef struct
{
	/*! Time of week (since Sunday morning) in milliseconds, GMT */
	uint32_t                timeOfWeekMs;

	/*! General purpose I/O status */
	uint32_t				gpioStatus;
} io_t;


/*! (DID_IO_SERVOS_PWM & DID_IO_SERVOS_PPM) I/O: PWM and PPM Servos */
typedef struct
{
	/*! Servo pulse time (us) */
	uint32_t				ch[NUM_SERVOS];
} io_servos_t;


// typedef struct
// {											// Magnetic Distortions:
// 	float				bFrame[3];				// static to body frame
// 	float				iFrame[3];				// static to inertial frame
// 	float				accuracy;				// Goodness of fit cal accuracy indicator (smaller is better)
// } magDistortion_t;

typedef struct
{
	// 		uint32_t			timeMs;				// (ms)		Sample timestamp used to identify age of sample
	float				theta[3];			// (rad)	Euler attitude
	float				mag[3];				// (Gauss)	Measured magnetometer output (body frame)
} magCalPoint_t;

typedef struct
{
	magCalPoint_t		pt[5];
	float				delta;				// (Gauss)	Difference between pt[1].mag and pt[3].mag.
} magCalSet_t;

/*! (DID_MAGNETOMETER_CAL) Magnetometer Calibration */
typedef struct
{
	uint32_t				state;			// Calibration state
	magCalSet_t				data[3];		// Data array.  Each element contains the min and max value found for roll, pitch, and yaw.
// 	magDistortion_t			mDist;			// Temporary holder for newly calculated magnetic distortions
	float					accuracy;		// Goodness of fit cal accuracy indicator (smaller is better)
} mag_cal_t;


/*! (DID_FLASH_CONFIG) Configuration data */
typedef struct
{
	/*! Size of group or union, which is nvm_group_x_t + padding */
	uint32_t				size;

	/*! Checksum, excluding size and checksum */
	uint32_t                checksum;

	/*! Manufacturer method for restoring flash defaults */
	uint32_t                key;

	/*! Startup sample period in milliseconds */
	uint32_t				startupSampleDtMs;

	/*! Startup solution update period in milliseconds */
	uint32_t				startupNavDtMs;

	/*! Serial port 0 baud rate in bits per second */
	uint32_t				ser0BaudRate;
	
	/*! Serial port 1 baud rate in bits per second */
	uint32_t				ser1BaudRate;

	/*! Euler rotation from INS computational frame to INS output frame.  Order applied: heading, pitch, roll. RADIANS. */
	float					insRotation[3];

	/*! Offset to INS output (in INS output frame) in meters */
	float					insOffset[3];

	/*! GPS antenna offset from INS comp frame origin (in INS comp frame) in meters */
	float					gpsAntOffset[3];

	/* INS dynamic platform model.  Determines performance characteristics of system.  */
	uint32_t				insDynModel;
	
	/*! System configuration bits */
	uint32_t				sysCfgBits;

	/*! Reference latitude, longitude and height above ellipsoid for north east down (NED) calculations in degrees, degrees, and meters */
	double                  refLla[3];

	/*! Last latitude, longitude, HAE (height above ellipsoid) used to aid GPS startup in degrees, degrees, and meters */
	double					lastLla[3];

	/*! Last LLA time since week start (Sunday morning) in milliseconds */
	uint32_t				lastLlaTimeOfWeekMs;

	/*! Last LLA number of week since January 1st, 1980 */
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
typedef struct
{	
	uint32_t                timeOfWeekMs;		//			Time of week (since Sunday morning) in milliseconds, GMT
	state_vars_t            x_dot;				//			State variables derivative
	float					magYawOffset;		// (rad)	Temporary offset in mag heading used to remove discontinuities when transitioning from moving to stationary (from GPS to mag heading)
} ins_res_t;

#ifdef RTK_EMBEDDED

typedef enum
{
	rtk_data_type_single_observation = 0,
	rtk_data_type_ephemeris = 1,
	rtk_data_type_glonass_ephemeris = 2,
	rtk_data_type_base_station_antenna = 3,
	rtk_data_type_observation_group_start = 100,
	rtk_data_type_observation_group_end = 101
} rtk_data_type_t;

/*! (DID_RTK) - for this data id, the data must be pre-flipped to the endianess of the receiver */
typedef struct PACK_ONE
{
	uint8_t	dataType; // rtk_data_type_t
	union
	{
		obsd_t obsd; // rtk_data_type_single_observation
		eph_t eph; // rtk_data_type_rover_ephemeris
		geph_t geph; // rtk_data_type_rover_glonass_ephemeris
		antenna_t antenna; // rtk_data_type_base_station_antenna
		uint8_t index; // rtk_data_type_observation_group_start or rtk_data_type_observation_group_end
	} data;
} rtk_data_t;

// flip 32 bit integers and 64 bit doubles in rtk data
void flipRTK(rtk_data_t* r);

#endif

POP_PACK_NONE

/*! Union of datasets */
union uDatasets
{
	dev_info_t			devInfo;
	ins_1_t				ins1;
	ins_2_t				ins2;
	imu_t				imu;
	dual_imu_t          dualImu;
	magnetometer_t		mag;
	barometer_t			baro;
	delta_theta_vel_t	dThetaVel;
	gps_t				gps;
	gps_nav_poslla_t	gpsPos;
	gps_nav_velned_t	gpsVel;
	gps_rssi_t			gpsRssi;
	nvm_flash_cfg_t		flashCfg;
	ins_misc_t			insMisc;
	sys_params_t		sysParams;
	sys_sensors_t		sysSensors;
	io_t				io;
	ins_res_t			insRes;
	ekf_states_t		ekfStates;
#ifdef RTK_EMBEDDED

	rtk_data_t			rtkData;

#endif

};

/*!
Contains 0 if big endian, 1 if little endian, -1 if unknown CPU architecture.
initDataSets must be called before this value is populated properly.
*/
extern int IS_LITTLE_ENDIAN; // 0 if big endian, 1 if little endian, -1 if unknown architecture

// calculates whether the current CPU is little endian or big endian architecture.
// 0 if big endian, 1 if little endian, -1 if unknown
int isCpuLittleEndian(void);

/*!
Initializes data structures - this must be called ONCE and only ONCE at program start
Upon completion, IS_LITTLE_ENDIAN is populated

@return 0 if CPU architecture could not be determined, otherwise non-zero
*/
int initDataSets(void);

/*!
Creates a 32 bit checksum from data

@param data the data to create a checksum for
@param count the number of bytes in data

@return the 32 bit checksum for data
*/
uint32_t serialNumChecksum32(void* data, int size);
uint32_t flashChecksum32(void* data, int size);

/*!
Flip the endianess of 32 bit values in data

@param data the data to flip 32 bit values in
@param dataLength the number of bytes in data
*/
void flipEndianess32(uint8_t* data, int dataLength);

/*!
Flip the bytes of a float (4 bytes) - ptr is assumed to be at least 4 bytes

@param ptr the float to flip
*/
void flipFloat(uint8_t* ptr);

/*!
Flip the bytes of a double (8 bytes) - ptr is assumed to be at least 8 bytes

@param ptr the double to flip
*/
void flipDouble(uint8_t* ptr);

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

// BE_SWAP: if big endian then swap
// LE_SWAP: if little endian then swap
#ifdef AVR
#define BE_SWAP32(_i) (SWAP32(_i))
#define BE_SWAP16(_i) (SWAP16(_i))
#define LE_SWAP32(_i) (_i)
#define LE_SWAP16(_i) (_i)
#elif ARM
#define BE_SWAP32(_i) (_i)
#define BE_SWAP16(_i) (_i)
#define LE_SWAP32(_i) (SWAP32(_i))
#define LE_SWAP16(_i) (SWAP16(_i))
#else
#define BE_SWAP32(_i) ((IS_LITTLE_ENDIAN) ? (_i) : (SWAP32(_i)))
#define BE_SWAP16(_i) ((IS_LITTLE_ENDIAN) ? (_i) : (SWAP16(_i)))
#define LE_SWAP32(_i) ((IS_LITTLE_ENDIAN) ? (SWAP32(_i)) : (_i))
#define LE_SWAP16(_i) ((IS_LITTLE_ENDIAN) ? (SWAP16(_i)) : (_i))
#endif

static INLINE float BE_SWAP_FLOAT32(uint8_t* f)
{
	if (IS_LITTLE_ENDIAN) { return *(float*)f; } else { flipFloat(f); return *(float*)f; }
}

static INLINE double BE_SWAP_FLOAT64(uint8_t* d)
{
	if (IS_LITTLE_ENDIAN) { return *(double*)d; } else { flipDouble(d); return *(double*)d; }
}

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
gen_3axis_sensord_t gen3AxisSensorDataD(double time, const double val[3]);

#ifdef __cplusplus
}
#endif

#endif // DATA_SETS_H
