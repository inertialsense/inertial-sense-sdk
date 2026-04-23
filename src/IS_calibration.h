#ifndef IS_CALIBRATION_H
#define IS_CALIBRATION_H

#include <stdint.h>

#include "data_sets.h"

#ifdef __cplusplus
extern "C" {
#endif

//_____ D E F I N I T I O N S ______________________________________________

#define TCAL_MAX_NUM_POINTS     20      // Maximum number of calibration points allowable
#define TCAL_MAX_TEMPERATURE    85      // Maximum temperature for temperature calibration

// 1.2.0 = 2x IMU, 1.3.0 = 3x IMU + 2x Mag, 1.4.0 = 5x IMU + 1x Mag
#define SENSOR_CAL_VER0     1
#define SENSOR_CAL_VER1     4
#define SENSOR_CAL_VER2     0

enum eScompCalState
{
    SC_RUNTIME                      = 0,   // Calibration off
    SC_TCAL_MONITOR_TEMP            = 1,
    SC_TCAL_INIT                    = 2,
    SC_TCAL_STARTUP_MEAN_LSB        = 3,
    SC_TCAL_STARTUP_DELAY           = 4,
    SC_TCAL_READY_TO_RUN            = 5,
    SC_TCAL_RUNNING                 = 6,
    SC_TCAL_STOP                    = 7,
    SC_TCAL_DONE                    = 8,
    SC_ACCEL_ALIGN_CHECK            = 9,
    SC_MCAL_SAMPLE_INIT             = 10,
    SC_MCAL_SAMPLE_MEAN_UCAL        = 11,    // Uncalibrated sensor
    SC_MCAL_SAMPLE_MEAN_TCAL        = 12,    // Temperature compensated sensor 
    SC_MCAL_SAMPLE_MEAN_MCAL        = 13,    // Motion calibrated + compensated sensor 
    SC_MCAL_SAMPLE_STOP             = 14,
    SC_LPF_SAMPLE                   = 15,
    SC_LPF_SAMPLE_FAST              = 16,
    SC_DONE                         = 17,
    SC_LINEARITY_MEAN_TCAL          = 18,    // Like SC_MCAL_SAMPLE_MEAN_TCAL, but goes back to SC_RUNTIME after some samples
};

enum eScompStatus
{
    SC_STATUS_ALIGNMENT_MASK        = 0x0000000F,
    SC_STATUS_ALIGNMENT_OFF         = 0x00000000,
    SC_STATUS_ALIGNMENT_GOOD        = 0x00000001,
    SC_STATUS_ALIGNMENT_BAD         = 0x00000002,
    SC_STATUS_SAMPLE_VALID_MASK     = 0x00000F00,
    SC_STATUS_SAMPLE_VALID_GYR      = 0x00000100,
    SC_STATUS_SAMPLE_VALID_ACC      = 0x00000200,
    SC_STATUS_SAMPLE_VALID_MAG      = 0x00000400,
    SC_STATUS_REFERENCE_IMU_VALID   = 0x00010000,
    SC_STATUS_REFERENCE_MAG_VALID   = 0x00020000,
    SC_STATUS_USING_INFERRED_IMU    = 0x00040000,
    SC_STATUS_USING_INFERRED_MAG    = 0x00080000,
    SC_STATUS_TEMPERATURE_CAL_VALID = 0x00100000,
    SC_STATUS_MOTION_CAL_VALID_MASK = 0x00600000,
    SC_STATUS_MOTION_CAL_VALID_OFFSET = 21,
    SC_STATUS_MOTION_CAL_VALID_IMU  = 0x00200000,
    SC_STATUS_MOTION_CAL_VALID_MAG  = 0x00400000,
};
typedef struct PACKED
{
    uint32_t                size;                               // Size of this struct
    uint32_t                checksum;                           // XOR of all bytes in this struct excluding size and checksum
    uint8_t                 version[4];                         // Sensor calibration version: [0] = major, [1] = mid, [2] = minor, [3] = unused
    uint8_t                 calDate[4];                         // Sensor calibration date, little endian order: [0] = year-2000, [1] = month, [2] = day, [3] = unused  */
    uint8_t                 calTime[4];                         // Sensor calibration date, little endian order: [0] = hour, [1] = minute, [2] = second, [3] = unused */
    uint32_t                devSerialNum;                       // Device serial number
} sensor_cal_info_t;

typedef struct PACKED
{
    uint32_t                size;                               // Size of this struct
    uint32_t                checksum;                           // XOR of all bytes in this struct excluding size and checksum
} sensor_data_info_t;

////////////////////////////////////////////////
// TCAL v1.2
typedef struct PACKED
{
    float                   temp;           // temperature of calibration point
    float                   gyrS[3];        // gyro - tc point steady state ADC LSB point
    float                   gyrK[3];        // gyro - tc point slope between current and next ADC point
    float                   accS[3];        // acc  - tc point steady state ADC LSB point
    float                   accK[3];        // acc  - tc point slope between current and next ADC point
    float                   magS[3];        // mag  - tc point steady state ADC LSB point
    float                   magK[3];        // mag  - tc point slope between current and next ADC point
} sensor_tcal_pt_t;

typedef struct PACKED
{
    uint32_t                numPts;                             // Number of temperature calibration points
    sensor_tcal_pt_t        pt[TCAL_MAX_NUM_POINTS];            // Sensor temperature calibration
} nvm_sensor_tcal_t;

////////////////////////////////////////////////
// TCAL v1.3
typedef struct PACKED
{
    float                   temp;           // temperature of calibration point
    float                   ss[3];          // tc point steady state ADC LSB point
} sensor_tcal_3axis_pt_t;

typedef struct PACKED
{
    uint32_t                numPts;                             // Number of temperature calibration points
    sensor_tcal_3axis_pt_t  pt[TCAL_MAX_NUM_POINTS];            // Sensor temperature calibration
} nvm_sensor_tcal_3axis_t;

typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t gyr[MAX_IMU_DEVICES_V1P3];          // Gyro temperature calibration
    nvm_sensor_tcal_3axis_t acc[MAX_IMU_DEVICES_V1P3];          // Accel temperature calibration
    nvm_sensor_tcal_3axis_t mag[MAX_MAG_DEVICES_V1P3];          // Mag temperature calibration
} sensor_tcal_group_v1p3_t;

typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t gyr[MAX_IMU_DEVICES_V1P4];          // Gyro temperature calibration
    nvm_sensor_tcal_3axis_t acc[MAX_IMU_DEVICES_V1P4];          // Accel temperature calibration
    nvm_sensor_tcal_3axis_t mag[MAX_MAG_DEVICES_V1P4];          // Mag temperature calibration
} sensor_tcal_group_v1p4_t;

// 1/3 of sensor_tcal_group_t used for uploading calibration
typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t sensor[MAX_IMU_DEVICES];            // temperature calibration
} sensor_tcal_group_subset_t;

////////////////////////////////////////////////
// MCAL v1.3
typedef struct PACKED
{
    float                   orth[9];        // Ortho-normalization matrices (cross-axis and scale factor)
    float                   bias[3];        // Bias
} sensor_motion_cal_t;

typedef struct PACKED
{
    sensor_motion_cal_t     pqr[MAX_IMU_DEVICES_V1P3];          // Gyros (x3 IMUs)
    sensor_motion_cal_t     acc[MAX_IMU_DEVICES_V1P3];          // Accelerometers (x3 IMUs)
    sensor_motion_cal_t     mag[MAX_MAG_DEVICES_V1P3];          // Magnetometers
} sensor_mcal_group_v1p3_t;

typedef struct PACKED
{
    sensor_motion_cal_t     pqr[MAX_IMU_DEVICES_V1P4];          // Gyros (x5 IMUs)
    sensor_motion_cal_t     acc[MAX_IMU_DEVICES_V1P4];          // Accelerometers (x5 IMUs)
    sensor_motion_cal_t     mag[MAX_MAG_DEVICES_V1P4];          // Magnetometers
} sensor_mcal_group_v1p4_t;

////////////////////////////////////////////////
// v1.3
typedef struct PACKED
{
    sensor_data_info_t          dinfo;                      // Size and checksum
    sensor_tcal_group_v1p3_t    tcal;                       // Temperature compensation
    sensor_mcal_group_v1p3_t    mcal;                       // Motion calibration
} sensor_cal_v1p3_data_t;

typedef struct PACKED
{
    sensor_cal_info_t           info;                       // Hardware IMX-5 and later have info before data to support various versions of calibration without hardware detection
    sensor_cal_v1p3_data_t      data;
} sensor_cal_v1p3_t;

////////////////////////////////////////////////
// v1.4
typedef struct PACKED
{
    sensor_data_info_t          dinfo;                      // Size and checksum
    sensor_tcal_group_v1p4_t    tcal;                       // Temperature compensation
    sensor_mcal_group_v1p4_t    mcal;                       // Motion calibration
} sensor_cal_v1p4_data_t;

typedef struct PACKED
{
    sensor_cal_info_t           info;
    sensor_cal_v1p4_data_t      data;
} sensor_cal_v1p4_t;

// Current version of sensor calibration in use
typedef sensor_tcal_group_v1p4_t    sensor_tcal_group_t;
typedef sensor_mcal_group_v1p4_t    sensor_mcal_group_t;
typedef sensor_cal_v1p4_data_t      sensor_cal_data_t;
typedef sensor_cal_v1p4_t           sensor_cal_t;               

#ifdef __cplusplus
}
#endif

#endif // IS_CALIBRATION_H

