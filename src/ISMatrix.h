/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file ISMatrix.h
 * @brief Vector/matrix math for 2/3/4-element vectors and 2x2/3x3/4x4/NxM matrices: dot/cross
 * products, magnitude/normalization, transpose/inverse, element-wise arithmetic, comparisons, and
 * low-pass filtering.
 *
 * Naming convention: an operation on vectors/matrices of dimension N is suffixed _VecN / _MatN
 * (e.g. add_Vec3_Vec3, mul_Mat3x3_Vec3x1); a trailing "d" (e.g. Vec3d) means double precision,
 * otherwise types are f_t (float, unless the SDK is built for double precision). Macro variants of
 * the simplest operations (DOT_VEC3, MAG_VEC3, etc.) are provided as faster inline alternatives to
 * their function equivalents (e.g. dot_Vec3()), at the cost of evaluating their argument multiple
 * times -- prefer the function form when the argument expression has side effects.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include "ISConstants.h"

#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

//_____ M A C R O S ________________________________________________________

/** Magnitude squared (dot product of a vector with itself). Inline macros (faster); call the equivalent dot_VecN() function instead for slower but better memory usage, or if v has side effects. */
#define DOT_VEC2(v)     ((v)[0]*(v)[0] + (v)[1]*(v)[1])
#define DOT_VEC3(v)     ((v)[0]*(v)[0] + (v)[1]*(v)[1] + (v)[2]*(v)[2])
#define DOT_VEC4(v)     ((v)[0]*(v)[0] + (v)[1]*(v)[1] + (v)[2]*(v)[2] + (v)[3]*(v)[3])

/** Magnitude (norm) of a vector, single precision (F) or double precision (D). */
#define MAG_VEC2(v)     (_SQRT(DOT_VEC2(v)))
#define MAG_VEC3(v)     (_SQRT(DOT_VEC3(v)))
#define MAG_VEC4(v)     (_SQRT(DOT_VEC4(v)))
#define MAG_VEC2D(v)    (sqrt(DOT_VEC2(v)))
#define MAG_VEC3D(v)    (sqrt(DOT_VEC3(v)))
#define MAG_VEC4D(v)    (sqrt(DOT_VEC4(v)))

#define EPSF32 (1.0e-16f)  //!< Smallest number for safe division, single precision
#define EPSF64 (1.0e-16l)  //!< Smallest number for safe division, double precision

/** Reciprocal of a vector's magnitude, clamped away from zero by EPSF32/EPSF64 for safe division (e.g. when normalizing). */
#define RECIPNORM_VEC2(v)   (1.0f/_MAX(MAG_VEC2(v),  EPSF32))
#define RECIPNORM_VEC3(v)   (1.0f/_MAX(MAG_VEC3(v),  EPSF32))
#define RECIPNORM_VEC4(v)   (1.0f/_MAX(MAG_VEC4(v),  EPSF32))
#define RECIPNORM_VEC3D(v)  (1.0l/_MAX(MAG_VEC3D(v), EPSF64))
#define RECIPNORM_VEC4D(v)  (1.0l/_MAX(MAG_VEC4D(v), EPSF64))

/** Unwrap each element of a 3-vector of radian angles into the canonical +-pi range (see UNWRAP_RAD_F32). */
#define UNWRAP_VEC3(v)          {UNWRAP_RAD_F32(v[0]); UNWRAP_RAD_F32(v[1]); UNWRAP_RAD_F32(v[2]) }

/** Type-correct zero literal for x's type (float/double/long double/int), used by the VEC3_*_ZERO macros below. */
#ifdef __cplusplus
#define ZERO_OF(x)                          static_cast<std::remove_reference_t<decltype(x)>>(0)
#else
#define ZERO_OF(x)                          _Generic((x), float: 0.0f, double: 0.0, long double: 0.0l, default: 0)
#endif
/** Per-element threshold comparisons over a 3-vector: true if any/all element(s) are less/greater than x (plain or absolute value). */
#define VEC3_ANY_LESS_THAN_X(v,x)           ( (((v)[0])<(x)) || (((v)[1])<(x)) || (((v)[2])<(x)) )
#define VEC3_ANY_GRTR_THAN_X(v,x)           ( (((v)[0])>(x)) || (((v)[1])>(x)) || (((v)[2])>(x)) )
#define VEC3_ALL_LESS_THAN_X(v,x)           ( (((v)[0])<(x)) && (((v)[1])<(x)) && (((v)[2])<(x)) )
#define VEC3_ALL_GRTR_THAN_X(v,x)           ( (((v)[0])>(x)) && (((v)[1])>(x)) && (((v)[2])>(x)) )
#define VEC3_ABS_ALL_LESS_THAN_X(v,x)       ( (fabs((v)[0])<(x)) && (fabs((v)[1])<(x)) && (fabs((v)[2])<(x)) )
#define VEC3_ABS_ALL_GRTR_THAN_X(v,x)       ( (fabs((v)[0])>(x)) && (fabs((v)[1])>(x)) && (fabs((v)[2])>(x)) )
#define VEC3_ABSF_ALL_LESS_THAN_X(v,x)      ( (fabsf((v)[0])<(x)) && (fabsf((v)[1])<(x)) && (fabsf((v)[2])<(x)) )
#define VEC3_ABSF_ALL_GRTR_THAN_X(v,x)      ( (fabsf((v)[0])>(x)) && (fabsf((v)[1])>(x)) && (fabsf((v)[2])>(x)) )
/** Zero/non-zero tests over a 3-vector: true if all/any/none of its elements are (not) zero, or if any element is NaN. */
#define VEC3_ALL_ZERO(v)                    ( (((v)[0]) == ZERO_OF((v)[0])) && (((v)[1]) == ZERO_OF((v)[1])) && (((v)[2]) == ZERO_OF((v)[2])) )
#define VEC3_ANY_ZERO(v)                    ( (((v)[0]) == ZERO_OF((v)[0])) || (((v)[1]) == ZERO_OF((v)[1])) || (((v)[2]) == ZERO_OF((v)[2])) )
#define VEC3_ANY_NOT_ZERO(v)                ( (((v)[0]) != ZERO_OF((v)[0])) || (((v)[1]) != ZERO_OF((v)[1])) || (((v)[2]) != ZERO_OF((v)[2])) )
#define VEC3_ANY_NAN(v)                     ( (is_nan_f((v)[0])) || (is_nan_f((v)[1])) || (is_nan_f((v)[2])) )
#define VEC3_NO_NAN(v)                      ( !VEC3_ANY_NAN(v) )

/** True if any element of an integer 3-vector is non-zero. */
#define INT3_ANY_NOT_ZERO(v)                ( ((v[0])!=(0)) || ((v[1])!=(0)) || ((v[2])!=(0)) )

/** Set every element of a 3- or 4-vector to the scalar x. */
#define SET_VEC3_X(v,x)                     { (v[0])=(x); (v[1])=(x); (v[2])=(x); }
#define SET_VEC4_X(v,x)                     { (v[0])=(x); (v[1])=(x); (v[2])=(x); (v[3])=(x); }

/** Zero-order (single-pole) low-pass filter state for a 3-vector; see @ref LPFO0_init_Vec3 / @ref LPFO0_Vec3. */
typedef struct
{
    ixVector3               v;      //!< Filter output / running value
    f_t                   alpha;    //!< Alpha gain (input weight)
    f_t                   beta;     //!< Beta gain (memory weight)
} sLpfO0;

/** First-order low-pass filter state for a 3-vector, with an explicit model-coefficient state (see O1_LPF_Vec3()). */
typedef struct
{
    ixVector3               v;      //!< Filter output / running value
    ixVector3               c1;     //!< Model-coefficient state
    f_t                   alpha;    //!< Alpha gain (input weight)
    f_t                   beta;     //!< Beta gain (memory weight)
} sLpfO1;

//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________

/**
 * @brief Check whether a float is exactly zero via its raw bit pattern, avoiding a floating-point comparison.
 * @param f Pointer to the float to check.
 * @return 1 if *f is exactly zero, 0 otherwise.
 */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
static __inline char is_zero(const f_t * f)
{
    const uint32_t *x = (const uint32_t*) f;
    return (*x == 0) ? 1 : 0;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif


/**
 * @brief General matrix multiplication: result[m,p] = A[m,n] * B[n,p] (or +=/-= per add).
 * @param result      Output: result[m,p].
 * @param A_ptr       Input matrix A[m,n], row-major.
 * @param B_ptr       Input matrix B[n,p], row-major (or [p,n] if transpose_B is set).
 * @param m           Row count of A (and result).
 * @param n           Column count of A / row count of B.
 * @param p           Column count of B (and result).
 * @param transpose_A Non-zero to treat A_ptr as an [n,m] matrix, transposed during the computation.
 * @param transpose_B Non-zero to treat B_ptr as a [p,n] matrix (traversed column-major instead of
 *                    row-major), transposing B during the computation.
 * @param add         0: result = A*B. >0: result += A*B. <0: result -= A*B.
 */
void mul_MatMxN(f_t *result, const f_t *A_ptr, const f_t *B_ptr, i_t m, i_t n, i_t p, char transpose_A, char transpose_B, char add);

/**
 * @brief Initialize an nxn matrix as the identity matrix (0s with 1s on the diagonal).
 * @param A Output: nxn identity matrix.
 * @param n Matrix dimension.
 */
void eye_MatN(f_t *A, i_t n);

/**
 * @brief Invert an nxn matrix: result = M^-1.
 * @param result Output: nxn inverse matrix.
 * @param M      Input nxn matrix.
 * @param n      Matrix dimension.
 * @return 0 on success, non-zero on numerical error (e.g. singular matrix).
 */
char inv_MatN(f_t *result, const f_t *M, i_t n);


/**
 * @brief Matrix transpose: M[m x n] -> result[n x m].
 * @param result Output: transposed matrix.
 * @param M      Input matrix.
 * @param m      Row count of M.
 * @param n      Column count of M.
 */
void trans_MatMxN(f_t *result, const f_t *M, int m, int n);


/** @brief Matrix multiply: result(3x3) = m1(3x3) * m2(3x3), single/double precision. */
void mul_Mat3x3_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2);
void mul_Mat3x3_Mat3x3_d(ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2);

/** @brief Matrix multiply with transpose: result(3x3) = m1.T(3x3) * m2(3x3), single/double precision. */
void mul_Mat3x3_Trans_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2);
void mul_Mat3x3_Trans_Mat3x3_d(ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2);

/** @brief Matrix multiply with transpose: result(3x3) = m1(3x3) * m2.T(3x3), single/double precision. */
void mul_Mat3x3_Mat3x3_Trans(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2);
void mul_Mat3x3_Mat3x3_Trans_d(ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2);

/** @brief Matrix addition: result(3x3) = m1(3x3) + m2(3x3). */
void add_Mat3x3_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2);

/** @brief Matrix subtraction: result(3x3) = m1(3x3) - m2(3x3). */
void sub_Mat3x3_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2);

/** @brief Matrix-vector multiply: result(2x1) = m(2x2) * v(2x1). */
void mul_Mat2x2_Vec2x1(ixVector2 result, const ixMatrix2 m, const ixVector2 v);

/** @brief Matrix-vector multiply with transpose: result(2x1) = m(2x2).T * v(2x1). */
void mul_Mat2x2_Trans_Vec2x1(ixVector2 result, const ixMatrix2 m, const ixVector2 v);

/** @brief Matrix-vector multiply: result(3x1) = m(3x3) * v(3x1). (9 multiplies, 6 adds) */
void mul_Mat3x3_Vec3x1(ixVector3 result, const ixMatrix3 m, const ixVector3 v);

/** @brief Matrix-vector multiply with transpose: result(3x1) = m(3x3).T * v(3x1). */
void mul_Mat3x3_Trans_Vec3x1(ixVector3 result, const ixMatrix3 m, const ixVector3 v);

/** @brief Matrix-vector multiply: result(4x1) = m(4x4) * v(4x1). */
void mul_Mat4x4_Vec4x1(ixVector4 result, const ixMatrix4 m, const ixVector4 v);

/** @brief Matrix-vector multiply with transpose: result(4x1) = m(4x4).T * v(4x1). */
void mul_Mat4x4_Trans_Vec4x1(ixVector4 result, const ixMatrix4 m, const ixVector4 v);

/** @brief Negate a 3x3 matrix: result = -m. */
void neg_Mat3x3(ixMatrix3 result, const ixMatrix3 m);

/** @brief Scalar multiply: result(3x3) = m(3x3) .* x. */
void mul_Mat3x3_X(ixMatrix3 result, const ixMatrix3 m, const f_t x);

/** @brief Scalar divide: result(3x3) = m(3x3) ./ x. */
void div_Mat3x3_X(ixMatrix3 result, const ixMatrix3 m, const f_t x);

/** @brief Outer product: result(3x3) = v1(3x1) * v2(1x3). */
void mul_Vec3x1_Vec1x3(ixMatrix3 result, const ixVector3 v1, const ixVector3 v2);

/** @brief Element-wise multiply: result(2) = v1(2) .* v2(2). */
void mul_Vec2_Vec2(ixVector2 result, const ixVector2 v1, const ixVector2 v2);

/** @brief Element-wise multiply: result(3) = v1(3) .* v2(3). */
void mul_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2);

/** @brief Element-wise multiply: result(4) = v1(4) .* v2(4). */
void mul_Vec4_Vec4(ixVector4 result, const ixVector4 v1, const ixVector4 v2);

/** @brief Element-wise square root: result(3) = sqrt(v(3)). */
void sqrt_Vec3(ixVector3 result, const ixVector3 v);

/** @brief Element-wise square root: result(4) = sqrt(v(4)). */
void sqrt_Vec4(ixVector4 result, const ixVector4 v);

/** @brief Element-wise absolute value: result(n) = abs(v(n)), single precision. */
void abs_Vec2(ixVector2 result, const ixVector2 v);
void abs_Vec3(ixVector3 result, const ixVector3 v);
void abs_Vec4(ixVector4 result, const ixVector4 v);

/** @brief Element-wise absolute value: result(n) = abs(v(n)), double precision. */
void abs_Vec2d(ixVector2d result, const ixVector2d v);
void abs_Vec3d(ixVector3d result, const ixVector3d v);
void abs_Vec4d(ixVector4d result, const ixVector4d v);

/** @brief Dot product of a vector with itself: result = v(n) dot v(n), single/double precision. */
f_t dot_Vec2(const ixVector2 v);
f_t dot_Vec3(const ixVector3 v);
f_t dot_Vec4(const ixVector4 v);
double dot_Vec2d(const ixVector2d v);
double dot_Vec3d(const ixVector3d v);
double dot_Vec4d(const ixVector4d v);

/** @brief Dot product: result = v1(n) dot v2(n), single/double precision. */
f_t dot_Vec2_Vec2(const ixVector2 v1, const ixVector2 v2);
f_t dot_Vec3_Vec3(const ixVector3 v1, const ixVector3 v2);
f_t dot_Vec4_Vec4(const ixVector4 v1, const ixVector4 v2);
double dot_Vec2d_Vec2d(const ixVector2d v1, const ixVector2d v2);
double dot_Vec3d_Vec3d(const ixVector3d v1, const ixVector3d v2);
double dot_Vec4d_Vec4d(const ixVector4d v1, const ixVector4d v2);

/** @brief Vector magnitude: result = sqrt(v(n) dot v(n)). */
f_t mag_Vec2(const ixVector2 v);
f_t mag_Vec3(const ixVector3 v);
f_t mag_Vec4(const ixVector4 v);

/** @brief Cross product: result(3) = v1(3) x v2(3), single-precision inputs, single- or double-precision output. */
void cross_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2);
void crossd_Vec3(ixVector3d result, const ixVector3 v1, const ixVector3 v2);

// /* Vector length
//  * result(3) = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
//  */
// f_t length_Vec3(ixVector3 v);

/** @brief Scalar multiply: result(2x1) = v(2) .* x, single/double precision. */
void mul_Vec2_X(ixVector2 result, const ixVector2 v, const f_t x);
void mul_Vec2d_X(ixVector2d result, const ixVector2d v, const double x);

/** @brief Scalar multiply: result(3x1) = v(3) .* x, single/double precision. */
void mul_Vec3_X(ixVector3 result, const ixVector3 v, const  f_t x);
void mul_Vec3d_X(ixVector3d result, const ixVector3d v, const double x);

/** @brief Scalar multiply: result(4x1) = v(4) .* x, single/double precision. */
void mul_Vec4_X(ixVector4 result, const ixVector4 v, const f_t x);
void mul_Vec4d_X(ixVector4d result, const ixVector4d v, const double x);

/** @brief Scalar divide: result(3x1) = v(3) ./ x, single/double precision. */
void div_Vec3_X(ixVector3 result, const ixVector3 v, const f_t x);
void div_Vec3d_X(ixVector3d result, const ixVector3d v, const double x);

/** @brief Scalar divide: result(4x1) = v(4) ./ x, single/double precision. */
void div_Vec4_X(ixVector4 result, const ixVector4 v, const f_t x);
void div_Vec4d_X(ixVector4d result, const ixVector4d v, const double x);


/** @brief Vector add: result(2) = v1(2) + v2(2). */
void add_Vec2_Vec2(ixVector2 result, const ixVector2 v1, const ixVector2 v2);

/** @brief Vector add: result(3) = v1(3) + v2(3), single/double precision. */
void add_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2);
void add_Vec3d_Vec3d(ixVector3d result, const ixVector3d v1, const ixVector3d v2);

/** @brief Weighted vector add: result(3) = k1*v1(3) + k2*v2(3). */
void add_K1Vec3_K2Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2, float k1, float k2);

/** @brief Vector add: result(4) = v1(4) + v2(4), single/double precision. */
void add_Vec4_Vec4(ixVector4 result, const ixVector4 v1, const ixVector4 v2);
void add_Vec4d_Vec4d(ixVector4d result, const ixVector4d v1, const ixVector4d v2);

/** @brief Vector subtract: result(3) = v1(3) - v2(3), single/double precision. */
void sub_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2);
void sub_Vec3d_Vec3d(ixVector3d result, const ixVector3d v1, const ixVector3d v2);

/** @brief Vector subtract: result(2) = v1(2) - v2(2). */
void sub_Vec2_Vec2(ixVector2 result, const ixVector2 v1, const ixVector2 v2);

/** @brief Vector subtract: result(4) = v1(4) - v2(4). */
void sub_Vec4_Vec4(ixVector4 result, const ixVector4 v1, const ixVector4 v2);

/** @brief Element-wise divide: result(3) = v1(3) ./ v2(3). */
void div_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2);

/** @brief Element-wise divide: result(4) = v1(4) ./ v2(4). */
void div_Vec4_Vec4(ixVector4 result, const ixVector4 v1, const ixVector4 v2);

/** @brief Negate: result(3) = -v(3). */
void neg_Vec3(ixVector3 result, const ixVector3 v);

/** @brief Average: result(3) = (v1(3) + v2(3)) * 0.5, single/double precision. */
void mean_Vec3_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2);
void mean_Vec3d_Vec3d(ixVector3d result, const ixVector3d v1, const ixVector3d v2);


/**
 * @brief Minimum of a 3-vector's elements: min(v[0], v[1], v[2]).
 * @param v Input vector.
 * @return Minimum element value.
 */
static __inline f_t min_Vec3_X(const ixVector3 v)
{
    f_t val = v[0];
    
    if (val > v[1])
        val = v[1];

    if (val > v[2])
        val = v[2];
        
    return val;
}

/**
 * @brief Maximum of a 3-vector's elements: max(v[0], v[1], v[2]).
 * @param v Input vector.
 * @return Maximum element value.
 */
static __inline f_t max_Vec3_X(const ixVector3 v)
{
    f_t val = v[0];
    
    if (val < v[1])
        val = v[1];

    if (val < v[2])
        val = v[2];
        
    return val;
}

/**
 * @brief Maximum of a 3-vector's absolute-value elements: max(fabsf(v[0]), fabsf(v[1]), fabsf(v[2])).
 * @param v Input vector.
 * @return Maximum absolute element value.
 */
static __inline f_t abs_Vec3_X(const ixVector3 v)
{
    f_t result = fabsf(v[0]);
    f_t val1   = fabsf(v[1]);
    f_t val2   = fabsf(v[2]);

    if ( result < val1 )
        result = val1;

    if ( result < val2 )
        result = val2;
        
    return result;
}

/**
 * @brief Element-wise minimum of two 3-vectors: result[i] = min(v1[i], v2[i]).
 * @param result Output: element-wise minimum.
 * @param v1     First input vector.
 * @param v2     Second input vector.
 */
static __inline void min_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2)
{
    result[0] = _MIN(v1[0], v2[0]);
    result[1] = _MIN(v1[1], v2[1]);
    result[2] = _MIN(v1[2], v2[2]);
}

/**
 * @brief Element-wise maximum of two 3-vectors: result[i] = max(v1[i], v2[i]).
 * @param result Output: element-wise maximum.
 * @param v1     First input vector.
 * @param v2     Second input vector.
 */
static __inline void max_Vec3(ixVector3 result, const ixVector3 v1, const ixVector3 v2)
{
    result[0] = _MAX(v1[0], v2[0]);
    result[1] = _MAX(v1[1], v2[1]);
    result[2] = _MAX(v1[2], v2[2]);
}

/** @brief Zero a 2-vector: v(2) = {0, 0}, single/double precision. */
static __inline void zero_Vec2(ixVector2 v)
{
    v[0] = 0.0f;
    v[1] = 0.0f;
}
static __inline void zero_Vec2d(ixVector2d v)
{
    v[0] = 0.0;
    v[1] = 0.0;
}

/** @brief Zero a 3-vector: v(3) = {0, 0, 0}, single/double precision. */
static __inline void zero_Vec3(ixVector3 v)
{
    v[0] = 0.0f;
    v[1] = 0.0f;
    v[2] = 0.0f;
}
static __inline void zero_Vec3d(ixVector3d v)
{
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
}

/** @brief Zero a 4-vector: v(4) = {0, 0, 0, 0}, single/double precision. */
static __inline void zero_Vec4(ixVector4 v)
{
    v[0] = 0.0f;
    v[1] = 0.0f;
    v[2] = 0.0f;
    v[3] = 0.0f;
}
static __inline void zero_Vec4d(ixVector4d v)
{
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
    v[3] = 0.0;
}

/**
 * @brief Zero an n-vector: v(n) = {0, ..., 0}.
 * @param v Vector to zero.
 * @param n Number of elements.
 */
static __inline void zero_VecN(f_t *v, i_t n)
{
    for (int i=0; i<n; i++)
    {
        v[i] = 0.0f;
    }
}

/**
 * @brief Zero an mxn matrix.
 * @param M Matrix to zero.
 * @param m Row count.
 * @param n Column count.
 */
static __inline void zero_MatMxN(f_t *M, i_t m, i_t n)
{
    for (int i=0; i<(m*n); i++)
    {
        M[i] = 0.0f;
    }
}

/**
 * @brief Check whether a 3x3 matrix is the identity matrix.
 * @param m Input 3x3 matrix.
 * @return 1 if m is an identity matrix, 0 otherwise.
 */
int mat3x3_IsIdentity(const f_t m[]);

/** @brief Copy a 3-vector: result(3) = v(3). Also converts between single/double precision. */
static __inline void cpy_Vec3_Vec3(ixVector3 result, const ixVector3 v)
{
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
}
static __inline void cpy_Vec3d_Vec3d(ixVector3d result, const ixVector3d v)
{
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
}
static __inline void cpy_Vec3d_Vec3(ixVector3d result, const  ixVector3 v)
{
    result[0] = (double)v[0];
    result[1] = (double)v[1];
    result[2] = (double)v[2];
}
static __inline void cpy_Vec3_Vec3d(ixVector3 result, const ixVector3d v)
{
    result[0] = (f_t)v[0];
    result[1] = (f_t)v[1];
    result[2] = (f_t)v[2];
}

/** @brief Copy a 4-vector: result(4) = v(4). Also converts between single/double precision. */
static __inline void cpy_Vec4_Vec4(ixVector4 result, const ixVector4 v)
{
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
    result[3] = v[3];
}
static __inline void cpy_Vec4d_Vec4d(ixVector4d result, const ixVector4d v)
{
    result[0] = v[0];
    result[1] = v[1];
    result[2] = v[2];
    result[3] = v[3];
}
static __inline void cpy_Vec4d_Vec4(ixVector4d result, const ixVector4 v)
{
    result[0] = (double)v[0];
    result[1] = (double)v[1];
    result[2] = (double)v[2];
    result[3] = (double)v[3];
}
static __inline void cpy_Vec4_Vec4d(ixVector4 result, const ixVector4d v)
{
    result[0] = (f_t)v[0];
    result[1] = (f_t)v[1];
    result[2] = (f_t)v[2];
    result[3] = (f_t)v[3];
}

/**
 * @brief Copy an n-vector: result(n) = v(n).
 * @param result Output vector.
 * @param v      Input vector.
 * @param n      Number of elements.
 */
static __inline void cpy_VecN_VecN(f_t *result, const f_t *v, i_t n)
{
    for (int i=0; i<n; i++)
    {
        result[i] = v[i];
    }
}

/**
 * @brief Copy an mxn matrix: result(mxn) = M(mxn).
 * @param result Output matrix.
 * @param M      Input matrix.
 * @param m      Row count.
 * @param n      Column count.
 */
static __inline void cpy_MatMxN(f_t *result, const f_t *M, i_t m, i_t n)
{
    for (int i=0; i<(m*n); i++)
    {
        result[i] = M[i];
    }
}

/**
 * @brief Copy matrix A(mxn) into a sub-block of matrix result(rxc), starting at (r_offset, c_offset).
 * A must fit inside result's dimensions at that offset.
 * @param result   Output matrix, r rows by c columns.
 * @param r        Row count of result.
 * @param c        Column count of result.
 * @param r_offset Row offset within result at which to place A.
 * @param c_offset Column offset within result at which to place A.
 * @param A        Input matrix, m rows by n columns.
 * @param m        Row count of A.
 * @param n        Column count of A.
 */
void cpy_MatRxC_MatMxN(f_t *result, i_t r, i_t c, i_t r_offset, i_t c_offset, f_t *A, i_t m, i_t n);


/** @brief Matrix transpose: result(2x2) = m(2x2)'. */
void transpose_Mat2(ixMatrix2 result, const ixMatrix2 m);

/** @brief Matrix transpose: result(3x3) = m(3x3)'. */
void transpose_Mat3(ixMatrix3 result, const ixMatrix3 m);

/** @brief Matrix transpose: result(4x4) = m(4x4)'. */
void transpose_Mat4(ixMatrix4 result, const ixMatrix4 m);

/**
 * @brief Invert a 2x2 matrix: result = m^-1.
 * @param result Output: 2x2 inverse matrix.
 * @param m      Input 2x2 matrix.
 * @return 0 on success, non-zero on numerical error (e.g. singular matrix).
 */
char inv_Mat2(ixMatrix2 result, ixMatrix2 m);

/**
 * @brief Invert a 3x3 matrix: result = m^-1.
 * @param result Output: 3x3 inverse matrix.
 * @param m      Input 3x3 matrix.
 * @return 0 on success, -1 on numerical error.
 */
char inv_Mat3(ixMatrix3 result, const ixMatrix3 m);

/**
 * @brief Invert a 4x4 matrix: result = m^-1.
 * @param result Output: 4x4 inverse matrix.
 * @param m      Input 4x4 matrix.
 * @return 0 on success, -1 on numerical error.
 */
char inv_Mat4(ixMatrix4 result, const ixMatrix4 m);

/**
 * @brief Normalize a 2-dimensional vector in place.
 * @param v Vector to normalize; updated in place.
 */
static __inline void normalize_Vec2(ixVector2 v)
{
    // Normalize vector
    mul_Vec2_X(v, v, RECIPNORM_VEC2(v));
}

/**
 * @brief Normalize a 3-dimensional vector.
 * @param result Output: normalized vector.
 * @param v      Input vector.
 */
static __inline void normalize_Vec3(ixVector3 result, const ixVector3 v)
{
    // Normalize vector
    mul_Vec3_X(result, v, RECIPNORM_VEC3(v));
}

/**
 * @brief Normalize a 4-dimensional vector, single precision.
 * @param result Output: normalized vector.
 * @param v      Input vector.
 */
static __inline void normalize_Vec4(ixVector4 result, const ixVector4 v)
{
    // Normalize vector
    mul_Vec4_X(result, v, RECIPNORM_VEC4(v));
}
/**
 * @brief Normalize a 4-dimensional vector, double precision.
 * @param result Output: normalized vector.
 * @param v      Input vector.
 */
static __inline void normalize_Vec4d(ixVector4d result, const ixVector4d v)
{
    // Normalize vector
    mul_Vec4d_X(result, v, RECIPNORM_VEC4D(v));
}

/**
 * @brief Check whether two 2-dimensional vectors are equal.
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Non-zero if v1 == v2 element-wise, 0 otherwise.
 */
static __inline int is_equal_Vec2(const ixVector2 v1, const ixVector2 v2)
{
    return
        (v1[0] == v2[0]) &&
        (v1[1] == v2[1]);
}

/**
 * @brief Check whether two 3-dimensional vectors are equal.
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Non-zero if v1 == v2 element-wise, 0 otherwise.
 */
static __inline int is_equal_Vec3(const ixVector3 v1, const ixVector3 v2)
{
    return
        (v1[0] == v2[0]) &&
        (v1[1] == v2[1]) &&
        (v1[2] == v2[2]);
}

/**
 * @brief Check whether two 4-dimensional vectors are equal.
 * @param v1 First vector.
 * @param v2 Second vector.
 * @return Non-zero if v1 == v2 element-wise, 0 otherwise.
 */
static __inline int is_equal_Vec4(const ixVector4 v1, const ixVector4 v2)
{
    return
        (v1[0] == v2[0]) &&
        (v1[1] == v2[1]) &&
        (v1[2] == v2[2]) &&
        (v1[3] == v2[3]);
}

/**
 * @brief Limit a 3-dimensional vector's elements to +-limit, in place.
 * @param v     Vector to limit; updated in place.
 * @param limit Symmetric limit magnitude.
 */
static __inline void limit_Vec3(ixVector3 v, f_t limit)
{
    _LIMIT(v[0], limit);
    _LIMIT(v[1], limit);
    _LIMIT(v[2], limit);
}

/**
 * @brief Limit a 3-dimensional vector's elements to [min, max], in place.
 * @param v   Vector to limit; updated in place.
 * @param min Lower limit.
 * @param max Upper limit.
 */
static __inline void limit2_Vec3(ixVector3 v, f_t min, f_t max)
{
    _LIMIT2(v[0], min, max);
    _LIMIT2(v[1], min, max);
    _LIMIT2(v[2], min, max);
}

/**
 * @brief Check whether any element of a single-precision 3-vector is NaN.
 * @param v Input vector.
 * @return Non-zero if any element is NaN, 0 otherwise.
 */
static inline int is_nan_vec3_f(float v[3])
{
    return  is_nan_f(v[0]) ||
            is_nan_f(v[1]) ||
            is_nan_f(v[2]);
}

/**
 * @brief Check whether any element of a double-precision 3-vector is NaN.
 * @param v Input vector.
 * @return Non-zero if any element is NaN, 0 otherwise.
 */
static inline int is_nan_vec3(double v[3])
{
    return  is_nan(v[0]) ||
            is_nan(v[1]) ||
            is_nan(v[2]);
}

/**
 * @brief Check whether every element of a single-precision 3-vector is finite (not NaN or infinite).
 * @param v Input vector.
 * @return Non-zero if all elements are finite, 0 otherwise.
 */
static inline int is_valid_vec3_f(float v[3])
{
    return  is_finite_f(v[0]) &&
            is_finite_f(v[1]) &&
            is_finite_f(v[2]);
}

/**
 * @brief Check whether every element of a double-precision 3-vector is finite (not NaN or infinite).
 * @param v Input vector.
 * @return Non-zero if all elements are finite, 0 otherwise.
 */
static inline int is_valid_vec3(double v[3])
{
    return  is_finite(v[0]) &&
            is_finite(v[1]) &&
            is_finite(v[2]);
}

/**
 * @brief Check whether an array contains NaN, single precision.
 * @param a    Input array.
 * @param size Number of elements in a.
 * @return 1 if any element is NaN, 0 otherwise.
 */
static __inline int isNan_array(f_t *a, int size)
{
    int i;

    for (i=0; i<size; i++)
    {
        if (is_nan_f(a[i]))
            return 1;
    }

    return 0;
}

/**
 * @brief Check whether an array contains NaN, double precision.
 * @param a    Input array.
 * @param size Number of elements in a.
 * @return 1 if any element is NaN, 0 otherwise.
 */
static __inline int isNan_array_d(double *a, int size)
{
    int i;

    for (i=0; i<size; i++)
    {
        if (is_nan(a[i]))
            return 1;
    }

    return 0;
}

#if defined(PLATFORM_IS_WINDOWS)
#pragma warning(push)
#pragma warning(disable : 4723)
#endif

/**
 * @brief Check whether an array contains infinity, single precision.
 * @param a    Input array.
 * @param size Number of elements in a.
 * @return 1 if any element is infinite, 0 otherwise.
 */
static __inline int isInf_array(f_t *a, int size)
{
    int i;

    f_t tmp = 1.0f;
    f_t inf = 1.0f / (tmp - 1.0f);

    for (i=0; i<size; i++)
    {
        if (a[i] == inf)
            return 1;
    }

    return 0;
}


/**
 * @brief Check whether an array contains infinity, double precision.
 * @param a    Input array.
 * @param size Number of elements in a.
 * @return 1 if any element is infinite, 0 otherwise.
 */
static __inline int isInf_array_d(double *a, int size)
{
    int i;

    double tmp = 1.0l;
    double inf = 1.0l / (tmp - 1.0l);

    for (i = 0; i<size; i++)
    {
        if (a[i] == inf)
            return 1;
    }

    return 0;
}

#if defined(PLATFORM_IS_WINDOWS)
#pragma warning(pop) 
#endif

/**
 * @brief Check whether every element of an array is finite (no NaN or infinity), single precision.
 * @param a    Input array.
 * @param size Number of elements in a.
 * @return 1 if no element is NaN or infinite, 0 otherwise.
 */
static __inline int isFinite_array(f_t *a, int size)
{
    if (isNan_array(a, size))
        return 0;

    if (isInf_array(a, size))
        return 0;

    return 1;
}


/**
 * @brief Check whether every element of an array is finite (no NaN or infinity), double precision.
 * @param a    Input array.
 * @param size Number of elements in a.
 * @return 1 if no element is NaN or infinite, 0 otherwise.
 */
static __inline int isFinite_array_d(double *a, int size)
{
    if (isNan_array_d(a, size))
        return 0;

    if (isInf_array_d(a, size))
        return 0;

    return 1;
}

/**
 * @brief Check whether every element of an array is strictly less than x.
 * @param a    Input array.
 * @param x    Threshold.
 * @param size Number of elements in a.
 * @return 1 if all elements are < x, 0 otherwise.
 */
int isAllLessThanX_array(f_t *a, f_t x, int size);

/**
 * @brief Check whether every element of an array is strictly greater than x.
 * @param a    Input array.
 * @param x    Threshold.
 * @param size Number of elements in a.
 * @return 1 if all elements are > x, 0 otherwise.
 */
int isAllMoreThanX_array(f_t *a, f_t x, int size);

/**
 * @brief Check whether every element's absolute value is strictly less than x.
 * @param a    Input array.
 * @param x    Threshold.
 * @param size Number of elements in a.
 * @return 1 if all elements' absolute values are < x, 0 otherwise.
 */
int isAllAbsLessThanX_array(f_t *a, f_t x, int size);

/**
 * @brief Initialize a zero-order low-pass filter's alpha/beta gains and initial value.
 * @param lpf          Filter state to initialize.
 * @param dt           Expected update period (seconds).
 * @param cornerFreqHz Low-pass filter corner frequency (Hz).
 * @param initVal      Initial value for the filter's running output.
 */
void LPFO0_init_Vec3(sLpfO0 *lpf, f_t dt, f_t cornerFreqHz, const ixVector3 initVal);

/**
 * @brief Update a zero-order low-pass filter with a new sample: v[n+1] = beta*v[n] + alpha*input.
 * @param lpf   Filter state; updated in place.
 * @param input New input sample.
 */
void LPFO0_Vec3(sLpfO0 *lpf, const ixVector3 input);

/**
 * @brief Zero-order (single-pole) low-pass filter update, standalone (no sLpfO0 state struct needed): result = beta*result + alpha*input.
 * @param result Filter output and running state; updated in place.
 * @param input  New input sample.
 * @param alph   Filter alpha parameter (input gain).
 * @param beta   Filter beta parameter (memory gain).
 */
static __inline void O0_LPF_Vec3(ixVector3 result, const ixVector3 input, f_t alph, f_t beta)
{
    ixVector3 tmp3;

    // val[n+1] = beta*val[n] + alpha*input
    mul_Vec3_X(tmp3,    input,    alph);
    mul_Vec3_X(result,    result,    beta);
    add_Vec3_Vec3(result,    result,    tmp3);
}


/**
 * @brief First-order low-pass filter update with an explicit model-coefficient state (for larger dt).
 * @param result Filter output and running state; updated in place.
 * @param input  New input sample.
 * @param c1     Model-coefficient state; updated in place.
 * @param alph   Filter alpha parameter (input gain).
 * @param beta   Filter beta parameter (memory gain).
 * @param dt     Time since the last update (seconds).
 */
static __inline void O1_LPF_Vec3(ixVector3 result, const ixVector3 input, ixVector3 c1, f_t alph, f_t beta, f_t dt)
{
    ixVector3 tmp3;

    // Estimate next models coefficients:            d1 = (input - result) / dt
    sub_Vec3_Vec3(tmp3, input, result);
    div_Vec3_X(tmp3, tmp3, dt);

    // LPF these coefficients:                        c1 = beta*c1 + alph*d1
    O0_LPF_Vec3(c1, tmp3, alph, beta);

    // Current state estimates:                        est = (last result) + c1*dt
    mul_Vec3_X(tmp3, c1, dt);
    add_Vec3_Vec3(result, result, tmp3);
    
    // LPF input into state estimates:                result = beta*est + alph*input
    O0_LPF_Vec3(result, input, alph, beta);    
}



#ifdef __cplusplus
}
#include <type_traits>
#endif

#endif /* MATRIX_H_ */
