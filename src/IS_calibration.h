/**
 * @file IS_calibration.h
 * @brief On-device sensor calibration data structures: temperature compensation (TCAL) and motion
 * calibration (MCAL), in the two currently-supported layout versions (v1.3, v1.4).
 *
 * v1.3 (3x IMU + 2x Mag) is used natively on IMX-5 hardware; v1.4 (5x IMU + 1x Mag) is used natively
 * on IMX-6 and the host SDK. sensor_tcal_group_t/sensor_mcal_group_t/sensor_cal_data_t/sensor_cal_t
 * resolve to whichever version is native for the current build target (see the typedefs below); use
 * IS_calibration_convert.h's convert_*_v1p3_to_v1p4()/convert_*_v1p4_to_v1p3() to convert between
 * versions, e.g. when a v1.3 calibration record is read from an IMX-5 device on a v1.4-native host.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef IS_CALIBRATION_H
#define IS_CALIBRATION_H

#include <stdint.h>

#include "data_sets.h"

#ifdef __cplusplus
extern "C" {
#endif

//_____ D E F I N I T I O N S ______________________________________________

#define TCAL_MAX_NUM_POINTS     20      //!< Maximum number of calibration points allowable
#define TCAL_MAX_TEMPERATURE    85      //!< Maximum temperature for temperature calibration

/** Sensor calibration layout version: 1.2.0 = 2x IMU, 1.3.0 = 3x IMU + 2x Mag, 1.4.0 = 5x IMU + 1x Mag. Selected by build target. */
#if defined(IMX_5)
#define SENSOR_CAL_VER0     1      //!< Major version
#define SENSOR_CAL_VER1     3      //!< Minor version (IMX-5: v1.3, 3x IMU + 2x Mag)
#define SENSOR_CAL_VER2     0      //!< Patch version
#else // IMX-6 and host SDK
#define SENSOR_CAL_VER0     1      //!< Major version
#define SENSOR_CAL_VER1     4      //!< Minor version (IMX-6/host: v1.4, 5x IMU + 1x Mag)
#define SENSOR_CAL_VER2     0      //!< Patch version
#endif

/** Sensor compensation/calibration state machine states, stepping through temperature calibration (TCAL) and motion calibration (MCAL) sampling. */
enum eScompCalState
{
    SC_RUNTIME                      = 0,    //!< Calibration off
    SC_TCAL_MONITOR_TEMP            = 1,    //!< Monitoring temperature for a calibration-worthy change
    SC_TCAL_INIT                    = 2,    //!< Initializing temperature calibration
    SC_TCAL_STARTUP_MEAN_LSB        = 3,    //!< Averaging raw ADC LSB at startup
    SC_TCAL_STARTUP_DELAY           = 4,    //!< Waiting for sensor to stabilize before sampling
    SC_TCAL_READY_TO_RUN            = 5,    //!< Ready to begin temperature calibration sampling
    SC_TCAL_RUNNING                 = 6,    //!< Temperature calibration sampling in progress
    SC_TCAL_STOP                    = 7,    //!< Stopping temperature calibration sampling
    SC_TCAL_DONE                    = 8,    //!< Temperature calibration complete
    SC_ACCEL_ALIGN_CHECK            = 9,    //!< Checking accelerometer alignment
    SC_MCAL_SAMPLE_INIT             = 10,   //!< Initializing motion calibration sampling
    SC_MCAL_SAMPLE_MEAN_UCAL        = 11,   //!< Sampling uncalibrated sensor mean
    SC_MCAL_SAMPLE_MEAN_TCAL        = 12,   //!< Sampling temperature-compensated sensor mean
    SC_MCAL_SAMPLE_MEAN_MCAL        = 13,   //!< Sampling motion-calibrated + compensated sensor mean
    SC_MCAL_SAMPLE_STOP             = 14,   //!< Stopping motion calibration sampling
    SC_LPF_SAMPLE                   = 15,   //!< Low-pass-filtered sampling
    SC_LPF_SAMPLE_FAST              = 16,   //!< Low-pass-filtered sampling, fast corner frequency
    SC_DONE                         = 17,   //!< Calibration sequence complete
    SC_LINEARITY_MEAN_TCAL          = 18,   //!< Like SC_MCAL_SAMPLE_MEAN_TCAL, but returns to SC_RUNTIME after some samples
};

/** Status bits for the sensor compensation/calibration subsystem. */
enum eScompStatus
{
    SC_STATUS_ALIGNMENT_MASK        = 0x0000000F,   //!< Mask isolating the alignment-status field
    SC_STATUS_ALIGNMENT_OFF         = 0x00000000,    //!< Alignment check not performed
    SC_STATUS_ALIGNMENT_GOOD        = 0x00000001,    //!< Alignment check passed
    SC_STATUS_ALIGNMENT_BAD         = 0x00000002,    //!< Alignment check failed
    SC_STATUS_SAMPLE_VALID_MASK     = 0x00000F00,    //!< Mask of all per-sensor sample-valid bits
    SC_STATUS_SAMPLE_VALID_GYR      = 0x00000100,    //!< Gyro sample is valid
    SC_STATUS_SAMPLE_VALID_ACC      = 0x00000200,    //!< Accelerometer sample is valid
    SC_STATUS_SAMPLE_VALID_MAG      = 0x00000400,    //!< Magnetometer sample is valid
    SC_STATUS_REFERENCE_IMU_VALID   = 0x00010000,    //!< Reference (truth) IMU is present and valid
    SC_STATUS_REFERENCE_MAG_VALID   = 0x00020000,    //!< Reference (truth) magnetometer is present and valid
    SC_STATUS_USING_INFERRED_IMU    = 0x00040000,    //!< IMU reference is inferred (no physical reference IMU available)
    SC_STATUS_USING_INFERRED_MAG    = 0x00080000,    //!< Magnetometer reference is inferred (no physical reference magnetometer available)
    SC_STATUS_TEMPERATURE_CAL_VALID = 0x00100000,    //!< Temperature calibration data is valid
    SC_STATUS_MOTION_CAL_VALID_MASK = 0x00600000,    //!< Mask isolating the motion-cal-valid field (see SC_STATUS_MOTION_CAL_VALID_OFFSET)
    SC_STATUS_MOTION_CAL_VALID_OFFSET = 21,          //!< Bit offset of the motion-cal-valid field
    SC_STATUS_MOTION_CAL_VALID_IMU  = 0x00200000,    //!< IMU motion calibration is valid
    SC_STATUS_MOTION_CAL_VALID_MAG  = 0x00400000,    //!< Magnetometer motion calibration is valid
};

/** Calibration record header: version/date/serial-number identification, common to all calibration layout versions. */
typedef struct PACKED
{
    uint32_t                size;                               //!< Size of this struct
    uint32_t                checksum;                           //!< XOR of all bytes in this struct excluding size and checksum
    uint8_t                 version[4];                         //!< Sensor calibration version: [0] = major, [1] = mid, [2] = minor, [3] = unused
    uint8_t                 calDate[4];                         //!< Sensor calibration date, little endian order: [0] = year-2000, [1] = month, [2] = day, [3] = unused
    uint8_t                 calTime[4];                         //!< Sensor calibration time, little endian order: [0] = hour, [1] = minute, [2] = second, [3] = unused
    uint32_t                devSerialNum;                       //!< Device serial number
} sensor_cal_info_t;

/** Size/checksum header for the calibration data payload (see sensor_cal_v1p3_data_t / sensor_cal_v1p4_data_t). */
typedef struct PACKED
{
    uint32_t                size;                               //!< Size of this struct
    uint32_t                checksum;                           //!< XOR of all bytes in this struct excluding size and checksum
} sensor_data_info_t;

////////////////////////////////////////////////
// TCAL v1.2
/** Single temperature-calibration point, v1.2 layout: per-axis steady-state value and slope to the next point, for gyro/accel/mag. */
typedef struct PACKED
{
    float                   temp;           //!< Temperature of calibration point
    float                   gyrS[3];        //!< Gyro: tc point steady-state ADC LSB value
    float                   gyrK[3];        //!< Gyro: tc point slope between current and next ADC point
    float                   accS[3];        //!< Accel: tc point steady-state ADC LSB value
    float                   accK[3];        //!< Accel: tc point slope between current and next ADC point
    float                   magS[3];        //!< Mag: tc point steady-state ADC LSB value
    float                   magK[3];        //!< Mag: tc point slope between current and next ADC point
} sensor_tcal_pt_t;

/** Temperature calibration curve, v1.2 layout: a set of calibration points for one sensor. */
typedef struct PACKED
{
    uint32_t                numPts;                             //!< Number of temperature calibration points
    sensor_tcal_pt_t        pt[TCAL_MAX_NUM_POINTS];            //!< Sensor temperature calibration points
} nvm_sensor_tcal_t;

////////////////////////////////////////////////
// TCAL v1.3
/** Single temperature-calibration point, v1.3+ layout: per-axis steady-state value only (slope is derived from adjacent points). */
typedef struct PACKED
{
    float                   temp;           //!< Temperature of calibration point
    float                   ss[3];          //!< Per-axis (X/Y/Z) tc point steady-state ADC LSB value
} sensor_tcal_3axis_pt_t;

/** Temperature calibration curve, v1.3+ layout: a set of calibration points for one 3-axis sensor. */
typedef struct PACKED
{
    uint32_t                numPts;                             //!< Number of temperature calibration points
    sensor_tcal_3axis_pt_t  pt[TCAL_MAX_NUM_POINTS];            //!< Sensor temperature calibration points
} nvm_sensor_tcal_3axis_t;

/** Temperature calibration for every sensor, v1.3 layout (IMX-5 native: 3x IMU + 2x Mag). */
typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t gyr[NUM_IMU_DEVICES_V1P3];          //!< Gyro temperature calibration, one per IMU
    nvm_sensor_tcal_3axis_t acc[NUM_IMU_DEVICES_V1P3];          //!< Accel temperature calibration, one per IMU
    nvm_sensor_tcal_3axis_t mag[NUM_MAG_DEVICES_V1P3];          //!< Mag temperature calibration, one per magnetometer
} sensor_tcal_group_v1p3_t;

/** Temperature calibration for every sensor, v1.4 layout (IMX-6/host native: 5x IMU + 1x Mag). */
typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t gyr[NUM_IMU_DEVICES_V1P4];          //!< Gyro temperature calibration, one per IMU
    nvm_sensor_tcal_3axis_t acc[NUM_IMU_DEVICES_V1P4];          //!< Accel temperature calibration, one per IMU
    nvm_sensor_tcal_3axis_t mag[NUM_MAG_DEVICES_V1P4];          //!< Mag temperature calibration, one per magnetometer
} sensor_tcal_group_v1p4_t;

/** Gyro temperature calibration for all IMUs; 1/3 of sensor_tcal_group_t, used for uploading calibration one sensor type at a time. */
typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t sensor[MAX_IMU_DEVICES];            //!< Per-IMU gyro temperature calibration
} sensor_tcal_gyr_group_t;

/** Accel temperature calibration for all IMUs; 1/3 of sensor_tcal_group_t, used for uploading calibration one sensor type at a time. */
typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t sensor[MAX_IMU_DEVICES];            //!< Per-IMU accel temperature calibration
} sensor_tcal_acc_group_t;

/** Mag temperature calibration for all magnetometers; 1/3 of sensor_tcal_group_t, used for uploading calibration one sensor type at a time. */
typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t sensor[MAX_MAG_DEVICES];            //!< Per-magnetometer temperature calibration
} sensor_tcal_mag_group_t;

////////////////////////////////////////////////
// MCAL v1.3
/** Single sensor's motion calibration: cross-axis/scale-factor ortho-normalization matrix plus bias. */
typedef struct PACKED
{
    float                   orth[9];        //!< Ortho-normalization matrix (cross-axis and scale factor), row-major 3x3
    float                   bias[3];        //!< Per-axis (X/Y/Z) bias
} sensor_motion_cal_t;

/** Gyro motion calibration for all IMUs; 1/3 of sensor_mcal_group_t, used for uploading calibration one sensor type at a time. */
typedef struct PACKED
{
    sensor_motion_cal_t sensor[MAX_IMU_DEVICES];                //!< Per-IMU gyro motion calibration
} sensor_mcal_gyr_group_t;

/** Accel motion calibration for all IMUs; 1/3 of sensor_mcal_group_t, used for uploading calibration one sensor type at a time. */
typedef struct PACKED
{
    sensor_motion_cal_t sensor[MAX_IMU_DEVICES];                //!< Per-IMU accel motion calibration
} sensor_mcal_acc_group_t;

/** Mag motion calibration for all magnetometers; 1/3 of sensor_mcal_group_t, used for uploading calibration one sensor type at a time. */
typedef struct PACKED
{
    sensor_motion_cal_t sensor[MAX_MAG_DEVICES];                //!< Per-magnetometer motion calibration
} sensor_mcal_mag_group_t;

/** Motion calibration for every sensor, v1.3 layout (IMX-5 native: 3x IMU + 2x Mag). */
typedef struct PACKED
{
    sensor_motion_cal_t     pqr[NUM_IMU_DEVICES_V1P3];          //!< Gyros (x3 IMUs)
    sensor_motion_cal_t     acc[NUM_IMU_DEVICES_V1P3];          //!< Accelerometers (x3 IMUs)
    sensor_motion_cal_t     mag[NUM_MAG_DEVICES_V1P3];          //!< Magnetometers
} sensor_mcal_group_v1p3_t;

/** Motion calibration for every sensor, v1.4 layout (IMX-6/host native: 5x IMU + 1x Mag). */
typedef struct PACKED
{
    sensor_motion_cal_t     pqr[NUM_IMU_DEVICES_V1P4];          //!< Gyros (x5 IMUs)
    sensor_motion_cal_t     acc[NUM_IMU_DEVICES_V1P4];          //!< Accelerometers (x5 IMUs)
    sensor_motion_cal_t     mag[NUM_MAG_DEVICES_V1P4];          //!< Magnetometers
} sensor_mcal_group_v1p4_t;

////////////////////////////////////////////////
// v1.3
/** Calibration data payload (temperature + motion calibration), v1.3 layout. */
typedef struct PACKED
{
    sensor_data_info_t          dinfo;                      //!< Size and checksum
    sensor_tcal_group_v1p3_t    tcal;                       //!< Temperature compensation
    sensor_mcal_group_v1p3_t    mcal;                       //!< Motion calibration
} sensor_cal_v1p3_data_t;

/** Full calibration record (header + data payload), v1.3 layout. */
typedef struct PACKED
{
    sensor_cal_info_t           info;                       //!< Calibration record header. Hardware IMX-5 and later have info before data to support various versions of calibration without hardware detection.
    sensor_cal_v1p3_data_t      data;                       //!< Calibration data payload
} sensor_cal_v1p3_t;

////////////////////////////////////////////////
// v1.4
/** Calibration data payload (temperature + motion calibration), v1.4 layout. */
typedef struct PACKED
{
    sensor_data_info_t          dinfo;                      //!< Size and checksum
    sensor_tcal_group_v1p4_t    tcal;                       //!< Temperature compensation
    sensor_mcal_group_v1p4_t    mcal;                       //!< Motion calibration
} sensor_cal_v1p4_data_t;

/** Full calibration record (header + data payload), v1.4 layout. */
typedef struct PACKED
{
    sensor_cal_info_t           info;                       //!< Calibration record header
    sensor_cal_v1p4_data_t      data;                       //!< Calibration data payload
} sensor_cal_v1p4_t;

/**
 * Per-build-target native calibration types. SN-7966: IMX-5 hardware is permanently Cal v1.3,
 * IMX-6 (and host SDK) is permanently Cal v1.4. Host code (no IMX_5/IMX_6 define) sees v1.4 so
 * ISDeviceCal etc. continue to operate on the v1.4 in-memory representation.
 */
#if defined(IMX_5)
typedef sensor_tcal_group_v1p3_t    sensor_tcal_group_t;   //!< Native temperature calibration group type for this build target (v1.3 on IMX-5)
typedef sensor_mcal_group_v1p3_t    sensor_mcal_group_t;   //!< Native motion calibration group type for this build target (v1.3 on IMX-5)
typedef sensor_cal_v1p3_data_t      sensor_cal_data_t;     //!< Native calibration data payload type for this build target (v1.3 on IMX-5)
typedef sensor_cal_v1p3_t           sensor_cal_t;          //!< Native full calibration record type for this build target (v1.3 on IMX-5)
#else
typedef sensor_tcal_group_v1p4_t    sensor_tcal_group_t;   //!< Native temperature calibration group type for this build target (v1.4 on IMX-6/host)
typedef sensor_mcal_group_v1p4_t    sensor_mcal_group_t;   //!< Native motion calibration group type for this build target (v1.4 on IMX-6/host)
typedef sensor_cal_v1p4_data_t      sensor_cal_data_t;     //!< Native calibration data payload type for this build target (v1.4 on IMX-6/host)
typedef sensor_cal_v1p4_t           sensor_cal_t;          //!< Native full calibration record type for this build target (v1.4 on IMX-6/host)
#endif

/** Byte sizes of each v1.3 calibration sub-group, for allocation/copy sizing. */
#define SIZE_OF_SENSOR_TCAL_GYR_V1P3    (NUM_IMU_DEVICES_V1P3*sizeof(nvm_sensor_tcal_3axis_t))   //!< Size of the v1.3 gyro temperature calibration group
#define SIZE_OF_SENSOR_TCAL_ACC_V1P3    (NUM_IMU_DEVICES_V1P3*sizeof(nvm_sensor_tcal_3axis_t))   //!< Size of the v1.3 accel temperature calibration group
#define SIZE_OF_SENSOR_TCAL_MAG_V1P3    (NUM_MAG_DEVICES_V1P3*sizeof(nvm_sensor_tcal_3axis_t))   //!< Size of the v1.3 mag temperature calibration group
#define SIZE_OF_SENSOR_MCAL_GYR_V1P3    (NUM_IMU_DEVICES_V1P3*sizeof(sensor_motion_cal_t))       //!< Size of the v1.3 gyro motion calibration group
#define SIZE_OF_SENSOR_MCAL_ACC_V1P3    (NUM_IMU_DEVICES_V1P3*sizeof(sensor_motion_cal_t))       //!< Size of the v1.3 accel motion calibration group
#define SIZE_OF_SENSOR_MCAL_MAG_V1P3    (NUM_MAG_DEVICES_V1P3*sizeof(sensor_motion_cal_t))       //!< Size of the v1.3 mag motion calibration group

/** Byte sizes of each v1.4 calibration sub-group, for allocation/copy sizing. */
#define SIZE_OF_SENSOR_TCAL_GYR_V1P4    (NUM_IMU_DEVICES_V1P4*sizeof(nvm_sensor_tcal_3axis_t))   //!< Size of the v1.4 gyro temperature calibration group
#define SIZE_OF_SENSOR_TCAL_ACC_V1P4    (NUM_IMU_DEVICES_V1P4*sizeof(nvm_sensor_tcal_3axis_t))   //!< Size of the v1.4 accel temperature calibration group
#define SIZE_OF_SENSOR_TCAL_MAG_V1P4    (NUM_MAG_DEVICES_V1P4*sizeof(nvm_sensor_tcal_3axis_t))   //!< Size of the v1.4 mag temperature calibration group
#define SIZE_OF_SENSOR_MCAL_GYR_V1P4    (NUM_IMU_DEVICES_V1P4*sizeof(sensor_motion_cal_t))       //!< Size of the v1.4 gyro motion calibration group
#define SIZE_OF_SENSOR_MCAL_ACC_V1P4    (NUM_IMU_DEVICES_V1P4*sizeof(sensor_motion_cal_t))       //!< Size of the v1.4 accel motion calibration group
#define SIZE_OF_SENSOR_MCAL_MAG_V1P4    (NUM_MAG_DEVICES_V1P4*sizeof(sensor_motion_cal_t))       //!< Size of the v1.4 mag motion calibration group

/** Byte sizes of each calibration sub-group for the current build target's native version (v1.3 on IMX-5, v1.4 otherwise). */
#if defined(IMX_5)
#define SIZE_OF_SENSOR_TCAL_GYR         SIZE_OF_SENSOR_TCAL_GYR_V1P3   //!< Native gyro temperature calibration group size
#define SIZE_OF_SENSOR_TCAL_ACC         SIZE_OF_SENSOR_TCAL_ACC_V1P3   //!< Native accel temperature calibration group size
#define SIZE_OF_SENSOR_TCAL_MAG         SIZE_OF_SENSOR_TCAL_MAG_V1P3   //!< Native mag temperature calibration group size
#define SIZE_OF_SENSOR_MCAL_GYR         SIZE_OF_SENSOR_MCAL_GYR_V1P3   //!< Native gyro motion calibration group size
#define SIZE_OF_SENSOR_MCAL_ACC         SIZE_OF_SENSOR_MCAL_ACC_V1P3   //!< Native accel motion calibration group size
#define SIZE_OF_SENSOR_MCAL_MAG         SIZE_OF_SENSOR_MCAL_MAG_V1P3   //!< Native mag motion calibration group size
#else
#define SIZE_OF_SENSOR_TCAL_GYR         SIZE_OF_SENSOR_TCAL_GYR_V1P4   //!< Native gyro temperature calibration group size
#define SIZE_OF_SENSOR_TCAL_ACC         SIZE_OF_SENSOR_TCAL_ACC_V1P4   //!< Native accel temperature calibration group size
#define SIZE_OF_SENSOR_TCAL_MAG         SIZE_OF_SENSOR_TCAL_MAG_V1P4   //!< Native mag temperature calibration group size
#define SIZE_OF_SENSOR_MCAL_GYR         SIZE_OF_SENSOR_MCAL_GYR_V1P4   //!< Native gyro motion calibration group size
#define SIZE_OF_SENSOR_MCAL_ACC         SIZE_OF_SENSOR_MCAL_ACC_V1P4   //!< Native accel motion calibration group size
#define SIZE_OF_SENSOR_MCAL_MAG         SIZE_OF_SENSOR_MCAL_MAG_V1P4   //!< Native mag motion calibration group size
#endif

#ifdef __cplusplus
}
#endif

#endif // IS_CALIBRATION_H

