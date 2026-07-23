/*
 * statistics.h
 *
 *  Created on: Jul 30, 2013
 *      Author: Walt Johnson
 */

/**
 * @file statistics.h
 * @brief Mean, variance, and standard-deviation helpers over strided arrays and 3-vectors, plus a
 * low-cost realtime (LPF-based) standard-deviation estimator.
 *
 * The scalar functions (mean/variance/standard_deviation and their _int32/_int64/_d suffixed
 * variants) walk an array using an explicit byteIncrement rather than assuming a packed C array,
 * so they can compute statistics over one field of a larger struct array (e.g. every Nth float
 * inside an array of structs) without copying. The _Vec3 functions apply the same scalar functions
 * independently to each of a 3-vector's components (units follow whatever the caller's data
 * represents; these functions are unit-agnostic).
 *
 * @author Walt Johnson
 * @copyright Copyright (c) 2013-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "ISMatrix.h"


//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

/** Realtime (LPF-based) per-axis mean/variance/standard-deviation estimator state (see @ref init_realtime_std_dev_Vec3 / @ref realtime_std_dev_Vec3). */
typedef struct
{
    ixVector3                 ave;      //!< Mean, low-pass filtered
    ixVector3                 var;      //!< Variance, low-pass filtered
    ixVector3                 std;      //!< Standard deviation (sqrt of var)
    float                   aveAlph;    //!< Mean filter alpha gain
    float                   aveBeta;    //!< Mean filter beta gain
    float                   varAlph;    //!< Variance filter alpha gain
    float                   varBeta;    //!< Variance filter beta gain
} sRTSDVec3;

//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________


/**
 * @brief Find the average value of a strided array of 32-bit floats.
 * @param input         Pointer to the first element.
 * @param size           Number of elements to average.
 * @param byteIncrement Byte offset between successive elements (sizeof(f_t) for a packed array).
 * @return Average value.
 */
f_t mean(f_t *input, int size, int byteIncrement);

/**
 * @brief Find the average value of a strided array of int32_t.
 * @param input         Pointer to the first element.
 * @param size           Number of elements to average.
 * @param byteIncrement Byte offset between successive elements (sizeof(int32_t) for a packed array).
 * @return Average value.
 */
f_t mean_int32(int32_t *input, int size, int byteIncrement);

/**
 * @brief Find the average value of a strided array of int64_t.
 * @param input         Pointer to the first element.
 * @param size           Number of elements to average.
 * @param byteIncrement Byte offset between successive elements (sizeof(int64_t) for a packed array).
 * @return Average value.
 */
double mean_int64(int64_t *input, int size, int byteIncrement);

/**
 * @brief Find the average value of a strided array of doubles.
 * @param input         Pointer to the first element.
 * @param size           Number of elements to average.
 * @param byteIncrement Byte offset between successive elements (sizeof(double) for a packed array).
 * @return Average value.
 */
double mean_d(double *input, int size, int byteIncrement);

/**
 * @brief Find the variance of a strided array of 32-bit floats.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(f_t) for a packed array).
 * @return Variance.
 */
f_t variance(f_t *input, int size, int byteIncrement);

/**
 * @brief Find the variance of a strided array of int32_t.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(int32_t) for a packed array).
 * @return Variance.
 */
f_t variance_int32(int32_t *input, int size, int byteIncrement);

/**
 * @brief Find the variance of a strided array of int64_t.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(int64_t) for a packed array).
 * @return Variance.
 */
double variance_int64(int64_t *input, int size, int byteIncrement);

/**
 * @brief Find the variance of a strided array of doubles.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(double) for a packed array).
 * @return Variance.
 */
double variance_d(double *input, int size, int byteIncrement);

/**
 * @brief Find the variance of a strided array of 32-bit floats, also returning the mean used.
 * @param input         Pointer to the first element.
 * @param ave           Output: the mean computed and used to derive the variance.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(f_t) for a packed array).
 * @return Variance.
 */
f_t variance_mean(f_t *input, f_t *ave, int size, int byteIncrement);

/**
 * @brief Sum of the squares of (input - ave) over a strided array of 32-bit floats.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(f_t) for a packed array).
 * @param ave           Reference value to subtract from each element before squaring.
 * @return Sum of squared deviations from ave.
 */
f_t delta_mean(f_t *input, int size, int byteIncrement, float ave);

/**
 * @brief Find the standard deviation of a strided array of 32-bit floats.
 *
 * @param input         Float pointer to start of data.
 * @param size          Number of values used in output.
 * @param byteIncrement Number of bytes size of data structure used to increment pointer by (f_t = 4).  Used to iterate across an structure arrays.
 * @return Standard deviation.
 */
f_t standard_deviation(f_t *input, int size, int byteIncrement);

/**
 * @brief Find the standard deviation of a strided array of doubles.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(double) for a packed array).
 * @return Standard deviation.
 */
double standard_deviation_d(double *input, int size, int byteIncrement);

/**
 * @brief Find the standard deviation of a strided array of int32_t.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(int32_t) for a packed array).
 * @return Standard deviation.
 */
f_t standard_deviation_int32(int32_t *input, int size, int byteIncrement);

/**
 * @brief Find the standard deviation of a strided array of int64_t.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(int64_t) for a packed array).
 * @return Standard deviation.
 */
double standard_deviation_int64(int64_t *input, int size, int byteIncrement);

/**
 * @brief Find the standard deviation of a strided array of 32-bit floats, given a precomputed mean.
 * @param input         Pointer to the first element.
 * @param mean          Precomputed mean of the array.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(f_t) for a packed array).
 * @return Standard deviation.
 */
f_t standard_deviation_mean(f_t *input, f_t *mean, int size, int byteIncrement);

/**
 * @brief Find the standard deviation of each component of a strided array of 3-vectors.
 * @param result        Output: standard deviation per axis (X, Y, Z).
 * @param input         Pointer to the first 3-vector's X component; Y and Z are input[1]/input[2].
 * @param size           Number of 3-vectors.
 * @param byteIncrement Byte offset between successive 3-vectors (sizeof(ixVector3) for a packed array).
 */
void standard_deviation_Vec3(ixVector3 result, ixVector3 input, int size, int byteIncrement);

/**
 * @brief Find the standard deviation of each component of a strided array of 3-vectors, given a precomputed per-axis mean.
 * @param result        Output: standard deviation per axis (X, Y, Z).
 * @param input         Pointer to the first 3-vector's X component; Y and Z are input[1]/input[2].
 * @param mean          Precomputed mean per axis (X, Y, Z).
 * @param size           Number of 3-vectors.
 * @param byteIncrement Byte offset between successive 3-vectors (sizeof(ixVector3) for a packed array).
 */
void stardard_deviation_mean_Vec3(ixVector3 result, ixVector3 input, ixVector3 mean, int size, int byteIncrement);

/**
 * @brief Find the average of each component of a strided array of 3-vectors.
 * @param ave           Output: average per axis (X, Y, Z).
 * @param input         Pointer to the first 3-vector's X component; Y and Z are input[1]/input[2].
 * @param size           Number of 3-vectors.
 * @param byteIncrement Byte offset between successive 3-vectors (sizeof(ixVector3) for a packed array).
 */
void mean_Vec3(ixVector3 ave, ixVector3 input, int size, int byteIncrement);

/**
 * @brief Root-mean-squared of (input - ave) over a strided array of 32-bit floats. Pass ave = 0 to get true RMS.
 * @param input         Pointer to the first element.
 * @param size           Number of elements.
 * @param byteIncrement Byte offset between successive elements (sizeof(f_t) for a packed array).
 * @param ave           Reference value to subtract from each element before squaring; 0 for true RMS.
 * @return Root-mean-squared value.
 */
f_t root_mean_squared(f_t *input, int size, int byteIncrement, float ave);


/**
 * @brief Initialize a realtime (LPF-based) per-axis standard-deviation estimator.
 * @param s               Estimator state to initialize.
 * @param dt              Expected update period (seconds), used to derive the LPF alpha/beta gains.
 * @param aveCornerFreqHz Low-pass corner frequency (Hz) for the running mean filter.
 * @param varCornerFreqHz Low-pass corner frequency (Hz) for the running variance filter.
 * @param initVal         Initial mean value per axis (X, Y, Z).
 */
void init_realtime_std_dev_Vec3(sRTSDVec3 *s, float dt, float aveCornerFreqHz, float varCornerFreqHz, ixVector3 initVal);

/**
 * @brief Update a realtime (LPF-based) per-axis standard-deviation estimator with a new 3-vector sample.
 * This is less accurate than @ref standard_deviation_Vec3 and must run every iteration (more processing
 * expensive overall), but requires far less memory since it doesn't retain a sample history.
 * @param input Latest 3-vector sample (X, Y, Z).
 * @param v     Estimator state; updated in place (mean, variance, and standard deviation).
 */
void realtime_std_dev_Vec3(f_t *input, sRTSDVec3 *v);

#endif /* STATISTICS_H_ */