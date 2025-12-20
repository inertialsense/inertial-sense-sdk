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

// 1.2.0 = uINS-3, 1.3.0 = IMX-5
#define SENSOR_CAL_VER0     1
#define SENSOR_CAL_VER1     3
#define SENSOR_CAL_VER2     0

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
    nvm_sensor_tcal_3axis_t gyr[MAX_IMU_DEVICES];               // Gyro temperature calibration
    nvm_sensor_tcal_3axis_t acc[MAX_IMU_DEVICES];               // Accel temperature calibration
    nvm_sensor_tcal_3axis_t mag[MAX_MAG_DEVICES];               // Mag temperature calibration
} sensor_tcal_group_t;

// 1/3 of sensor_tcal_group_t
typedef struct PACKED
{
    nvm_sensor_tcal_3axis_t sensor[MAX_IMU_DEVICES];            // temperature calibration
} sensor_tcal_group_subset_t;

////////////////////////////////////////////////
// MCAL v1.2
typedef struct PACKED
{                                       // Cross-axis scale factor matrices
    float                   pqr[9];         // gyros
    float                   acc[9];         // accelerometers
    float                   mag[9];         // magnetometers
} sensor_scalars_t;

////////////////////////////////////////////////
// MCAL v1.3
typedef struct PACKED
{
    float                   orth[9];        // Ortho-normalization matrices (cross-axis and scale factor)
    float                   bias[3];        // Bias
} sensor_motion_cal_t;

typedef struct PACKED
{
    sensor_motion_cal_t     pqr[MAX_IMU_DEVICES];               // Gyros (x3 IMUs)
    sensor_motion_cal_t     acc[MAX_IMU_DEVICES];               // Accelerometers (x3 IMUs)
    sensor_motion_cal_t     mag[MAX_MAG_DEVICES];               // Magnetometers
} sensor_mcal_group_t;

////////////////////////////////////////////////
// v1.2
typedef struct PACKED
{
    sensor_scalars_t        orth;                       // Sensor ortho-normalization matrices (cross-axis and scale factor)
    sensors_mpu_t           bias;
    nvm_sensor_tcal_t       tcal;
} sensor_cal_mpu_t;

typedef struct PACKED
{
    sensor_data_info_t      dinfo;                      // Size and checksum
    sensor_cal_mpu_t        mpu[2];
} sensor_cal_v1p2_data_t;

typedef struct PACKED
{
    sensor_cal_v1p2_data_t  data;
    sensor_cal_info_t       info;                       // uINS-3 legacy has info after data 
} sensor_cal_v1p2_t;

////////////////////////////////////////////////
// v1.3
typedef struct PACKED
{
    sensor_data_info_t      dinfo;                      // Size and checksum
    sensor_tcal_group_t     tcal;                       // Temperature compensation
    sensor_mcal_group_t     mcal;                       // Motion calibration
} sensor_cal_v1p3_data_t;

typedef struct PACKED
{
    sensor_cal_info_t       info;                       // Hardawre IMX-5 and later have info before data to support various versions of calibration without hardware detection
    sensor_cal_v1p3_data_t  data;
} sensor_cal_v1p3_t;

typedef sensor_cal_v1p3_t   sensor_cal_t;               // Current version

#ifdef __cplusplus
}
#endif

#endif // IS_CALIBRATION_H

