#ifndef IS_CALIBRATION_CONVERT_H
#define IS_CALIBRATION_CONVERT_H

#include "IS_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

void set_sensor_cal_data_defaults_v1p3(sensor_cal_v1p3_data_t *data);
void set_sensor_cal_data_defaults_v1p4(sensor_cal_v1p4_data_t *data);
void set_sensor_cal_data_defaults_v1p5(sensor_cal_v1p5_data_t *data);
void set_sensor_mcal_data_defaults_v1p3(sensor_cal_v1p3_data_t *data);
void set_sensor_mcal_data_defaults_v1p4(sensor_cal_v1p4_data_t *data);
void set_sensor_mcal_data_defaults_v1p5(sensor_cal_v1p5_data_t *data);

// Native (build-target) defaults init. Resolves to _v1p3 under IMX_5, _v1p4 elsewhere.
static inline void set_sensor_cal_data_defaults(sensor_cal_data_t *data)
{
#if defined(IMX_5)
    set_sensor_cal_data_defaults_v1p3(data);
#else
    set_sensor_cal_data_defaults_v1p5(data);
#endif
}

// Native (build-target) defaults init. Resolves to _v1p3 under IMX_5, _v1p4 elsewhere.
static inline void set_sensor_motion_cal_data_defaults(sensor_cal_data_t *data)
{
#if defined(IMX_5)
    set_sensor_mcal_data_defaults_v1p3(data);
#else
    set_sensor_mcal_data_defaults_v1p5(data);
#endif
}

void convert_tcal_v1p3_to_v1p4(const sensor_tcal_group_v1p3_t *v1p3, sensor_tcal_group_v1p4_t *v1p4);
void convert_mcal_v1p3_to_v1p4(const sensor_mcal_group_v1p3_t *v1p3, sensor_mcal_group_v1p4_t *v1p4);
void convert_sensor_cal_v1p3_to_v1p4(const sensor_cal_v1p3_t *v1p3, sensor_cal_v1p4_t *v1p4);
void convert_sensor_cal_v1p4_to_v1p5(const sensor_cal_v1p4_t *v1p4, sensor_cal_v1p5_t *v1p5);

void convert_tcal_v1p4_to_v1p3(const sensor_tcal_group_v1p4_t *v1p4, sensor_tcal_group_v1p3_t *v1p3);
void convert_mcal_v1p4_to_v1p3(const sensor_mcal_group_v1p4_t *v1p4, sensor_mcal_group_v1p3_t *v1p3);
void convert_sensor_cal_v1p4_to_v1p3(const sensor_cal_v1p4_t *v1p4, sensor_cal_v1p3_t *v1p3);

#ifdef __cplusplus
}
#endif

#endif // IS_CALIBRATION_CONVERT_H
