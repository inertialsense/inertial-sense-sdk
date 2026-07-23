/**
 * @file IS_calibration_convert.h
 * @brief Default-initialization and v1.3 <-> v1.4 conversion helpers for the sensor calibration
 * structures defined in IS_calibration.h.
 *
 * The set_sensor_*_defaults() functions (without a _v1pN suffix) resolve to the build target's
 * native version (v1.3 on IMX-5, v1.4 elsewhere) via @ref sensor_cal_data_t; use the explicit _v1p3
 * / _v1p4 conversion functions when translating a calibration record between versions, e.g. when a
 * v1.3 record read from an IMX-5 device needs to be consumed by v1.4-native host code.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef IS_CALIBRATION_CONVERT_H
#define IS_CALIBRATION_CONVERT_H

#include "IS_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize a v1.3 calibration data payload's temperature-calibration fields to defaults.
 * @param data Calibration data payload to initialize.
 */
void set_sensor_cal_data_defaults_v1p3(sensor_cal_v1p3_data_t *data);

/**
 * @brief Initialize a v1.4 calibration data payload's temperature-calibration fields to defaults.
 * @param data Calibration data payload to initialize.
 */
void set_sensor_cal_data_defaults_v1p4(sensor_cal_v1p4_data_t *data);

/**
 * @brief Initialize a v1.3 calibration data payload's motion-calibration fields to defaults.
 * @param data Calibration data payload to initialize.
 */
void set_sensor_mcal_data_defaults_v1p3(sensor_cal_v1p3_data_t *data);

/**
 * @brief Initialize a v1.4 calibration data payload's motion-calibration fields to defaults.
 * @param data Calibration data payload to initialize.
 */
void set_sensor_mcal_data_defaults_v1p4(sensor_cal_v1p4_data_t *data);

/**
 * @brief Initialize a calibration data payload's temperature-calibration fields to defaults, using
 * the build target's native version (v1.3 on IMX_5, v1.4 elsewhere).
 * @param data Calibration data payload to initialize.
 */
static inline void set_sensor_cal_data_defaults(sensor_cal_data_t *data)
{
#if defined(IMX_5)
    set_sensor_cal_data_defaults_v1p3(data);
#else
    set_sensor_cal_data_defaults_v1p4(data);
#endif
}

/**
 * @brief Initialize a calibration data payload's motion-calibration fields to defaults, using
 * the build target's native version (v1.3 on IMX_5, v1.4 elsewhere).
 * @param data Calibration data payload to initialize.
 */
static inline void set_sensor_motion_cal_data_defaults(sensor_cal_data_t *data)
{
#if defined(IMX_5)
    set_sensor_mcal_data_defaults_v1p3(data);
#else
    set_sensor_mcal_data_defaults_v1p4(data);
#endif
}

/**
 * @brief Convert a v1.3 temperature calibration group (3x IMU + 2x Mag) to v1.4 layout (5x IMU + 1x Mag).
 * @param v1p3 Source v1.3 temperature calibration group.
 * @param v1p4 Output: v1.4 temperature calibration group.
 */
void convert_tcal_v1p3_to_v1p4(const sensor_tcal_group_v1p3_t *v1p3, sensor_tcal_group_v1p4_t *v1p4);

/**
 * @brief Convert a v1.3 motion calibration group (3x IMU + 2x Mag) to v1.4 layout (5x IMU + 1x Mag).
 * @param v1p3 Source v1.3 motion calibration group.
 * @param v1p4 Output: v1.4 motion calibration group.
 */
void convert_mcal_v1p3_to_v1p4(const sensor_mcal_group_v1p3_t *v1p3, sensor_mcal_group_v1p4_t *v1p4);

/**
 * @brief Convert a full v1.3 calibration record (header + temperature + motion calibration) to v1.4 layout.
 * @param v1p3 Source v1.3 calibration record.
 * @param v1p4 Output: v1.4 calibration record.
 */
void convert_sensor_cal_v1p3_to_v1p4(const sensor_cal_v1p3_t *v1p3, sensor_cal_v1p4_t *v1p4);

/**
 * @brief Convert v1.3 sensor compensation data to v1.4 layout.
 * @param v1p3 Source v1.3 sensor compensation data.
 * @param v1p4 Output: v1.4 sensor compensation data.
 */
void convert_scomp_v1p3_to_v1p4(const sensor_compensation_v1p3_t *v1p3, sensor_compensation_v1p4_t *v1p4);

/**
 * @brief Convert a v1.4 temperature calibration group (5x IMU + 1x Mag) to v1.3 layout (3x IMU + 2x Mag).
 * @param v1p4 Source v1.4 temperature calibration group.
 * @param v1p3 Output: v1.3 temperature calibration group.
 */
void convert_tcal_v1p4_to_v1p3(const sensor_tcal_group_v1p4_t *v1p4, sensor_tcal_group_v1p3_t *v1p3);

/**
 * @brief Convert a v1.4 motion calibration group (5x IMU + 1x Mag) to v1.3 layout (3x IMU + 2x Mag).
 * @param v1p4 Source v1.4 motion calibration group.
 * @param v1p3 Output: v1.3 motion calibration group.
 */
void convert_mcal_v1p4_to_v1p3(const sensor_mcal_group_v1p4_t *v1p4, sensor_mcal_group_v1p3_t *v1p3);

/**
 * @brief Convert a full v1.4 calibration record (header + temperature + motion calibration) to v1.3 layout.
 * @param v1p4 Source v1.4 calibration record.
 * @param v1p3 Output: v1.3 calibration record.
 */
void convert_sensor_cal_v1p4_to_v1p3(const sensor_cal_v1p4_t *v1p4, sensor_cal_v1p3_t *v1p3);

/**
 * @brief Convert v1.4 sensor compensation data to v1.3 layout.
 * @param v1p4 Source v1.4 sensor compensation data.
 * @param v1p3 Output: v1.3 sensor compensation data.
 */
void convert_scomp_v1p4_to_v1p3(const sensor_compensation_v1p4_t *v1p4, sensor_compensation_v1p3_t *v1p3);

#ifdef __cplusplus
}
#endif

#endif // IS_CALIBRATION_CONVERT_H
