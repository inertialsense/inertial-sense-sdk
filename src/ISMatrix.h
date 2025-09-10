/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MATRIX_H_
#define MATRIX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ISConstants.h"

#include <math.h>
#include <string.h>

//_____ M A C R O S ________________________________________________________

// Magnitude Squared or Dot Product of vector w/ itself
// Inline macros (faster).  Call functions (i.e. dot_Vec3()) for slower but better memory usage.
#define DOT_VEC2(v)     ((v)[0]*(v)[0] + (v)[1]*(v)[1])
#define DOT_VEC3(v)     ((v)[0]*(v)[0] + (v)[1]*(v)[1] + (v)[2]*(v)[2])
#define DOT_VEC4(v)     ((v)[0]*(v)[0] + (v)[1]*(v)[1] + (v)[2]*(v)[2] + (v)[3]*(v)[3])

// Magnitude or Norm 
#define MAG_VEC2(v)     (_SQRT(DOT_VEC2(v)))
#define MAG_VEC3(v)     (_SQRT(DOT_VEC3(v)))
#define MAG_VEC4(v)     (_SQRT(DOT_VEC4(v)))
#define MAG_VEC2D(v)    (sqrt(DOT_VEC2(v)))
#define MAG_VEC3D(v)    (sqrt(DOT_VEC3(v)))
#define MAG_VEC4D(v)    (sqrt(DOT_VEC4(v)))

#define EPSF32 (1.0e-16f)  // Smallest number for safe division
#define EPSF64 (1.0e-16l)  // Smallest number for safe division

#define RECIPNORM_VEC2(v)	(1.0f/_MAX(MAG_VEC2(v),  EPSF32))
#define RECIPNORM_VEC3(v)	(1.0f/_MAX(MAG_VEC3(v),  EPSF32))
#define RECIPNORM_VEC4(v)	(1.0f/_MAX(MAG_VEC4(v),  EPSF32))
#define RECIPNORM_VEC3D(v)	(1.0l/_MAX(MAG_VEC3D(v), EPSF64))
#define RECIPNORM_VEC4D(v)	(1.0l/_MAX(MAG_VEC4D(v), EPSF64))

#define UNWRAP_VEC3(v)	{UNWRAP_RAD_F32(v[0]); UNWRAP_RAD_F32(v[1]); UNWRAP_RAD_F32(v[2]) }

#define VEC3_ANY_LESS_THAN_X(v,x)           ( ((v[0])<(x)) || ((v[1])<(x)) || ((v[2])<(x)) )
#define VEC3_ANY_GRTR_THAN_X(v,x)           ( ((v[0])>(x)) || ((v[1])>(x)) || ((v[2])>(x)) )
#define VEC3_ALL_LESS_THAN_X(v,x)           ( ((v[0])<(x)) && ((v[1])<(x)) && ((v[2])<(x)) )
#define VEC3_ALL_GRTR_THAN_X(v,x)           ( ((v[0])>(x)) && ((v[1])>(x)) && ((v[2])>(x)) )
#define VEC3_ABS_ALL_LESS_THAN_X(v,x)       ( (fabs(v[0])<(x)) && (fabs(v[1])<(x)) && (fabs(v[2])<(x)) )
#define VEC3_ABS_ALL_GRTR_THAN_X(v,x)       ( (fabs(v[0])>(x)) && (fabs(v[1])>(x)) && (fabs(v[2])>(x)) )
#define VEC3_ABSF_ALL_LESS_THAN_X(v,x)      ( (fabsf(v[0])<(x)) && (fabsf(v[1])<(x)) && (fabsf(v[2])<(x)) )
#define VEC3_ABSF_ALL_GRTR_THAN_X(v,x)      ( (fabsf(v[0])>(x)) && (fabsf(v[1])>(x)) && (fabsf(v[2])>(x)) )
#define VEC3_ALL_ZERO(v)                    ( ((v[0])==(0.0f)) && ((v[1])==(0.0f)) && ((v[2])==(0.0f)) )
#define VEC3_ANY_ZERO(v)                    ( ((v[0])==(0.0f)) || ((v[1])==(0.0f)) || ((v[2])==(0.0f)) )
#define VEC3_ANY_NOT_ZERO(v)                ( ((v[0])!=(0.0f)) || ((v[1])!=(0.0f)) || ((v[2])!=(0.0f)) )

#define INT3_ANY_NOT_ZERO(v)                ( ((v[0])!=(0)) || ((v[1])!=(0)) || ((v[2])!=(0)) )

#define SET_VEC3_X(v,x)                     { (v[0])=(x); (v[1])=(x); (v[2])=(x); }
#define SET_VEC4_X(v,x)                     { (v[0])=(x); (v[1])=(x); (v[2])=(x); (v[3])=(x); }

#define IS_NAN(v)                           ((v) != (v))
#define IS_INF(v)                           (isinf(v))

// Zero order low-pass filter 
typedef struct
{
    ixVector3               v;
    f_t                   alpha;  // alpha gain
    f_t                   beta;   // beta  gain
} sLpfO0;

// First order low-pass filter
typedef struct
{
	ixVector3               v;
	ixVector3               c1;
	f_t                   alpha;  // alpha gain
	f_t                   beta;   // beta  gain
} sLpfO1;

//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________

/*
 * We can avoid expensive floating point operations if one of the
 * values in question is zero.  This provides an easy way to check
 * to see if it is actually a zero without using any floating point
 * code.
 */
#if !defined(PLATFORM_IS_WINDOWS)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
static __inline char is_zero( const f_t * f )
{
	const uint32_t *x = (const uint32_t*) f;
	return (*x == 0) ? 1 : 0;
}
#if !defined(PLATFORM_IS_WINDOWS)
#pragma GCC diagnostic pop
#endif


/*
 * Perform the matrix multiplication A[m,n] * B[n,p], storing the
 * result in result[m,p].
 *
 * If transpose_B is set, B is assumed to be a [p,n] matrix that is
 * transversed in column major order instead of row major.  This
 * has the effect of transposing B during the computation.
 *
 * If add == 0, OUT  = A * B.
 * If add >  0, OUT += A * B.
 * If add <  0, OUT -= A * B.
 */
void mul_MatMxN( f_t *result, const f_t *A_ptr, const f_t *B_ptr, i_t m, i_t n, i_t p, char transpose_A, char transpose_B, char add );

/* Initialize square matrix as identity (0's with 1's in the diagonal)
* result(nxn) = eye(nxn)
*/
void eye_MatN( f_t *A, i_t n );

/* Matrix Inverse
* result(nxn) = M(nxn)^-1
*/
char inv_MatN( f_t *result, const f_t *M, i_t n );


/* Matrix Transpose:  M[m x n] -> result[n x m]
*/
void trans_MatMxN( f_t *result, const f_t *M, int m, int n );


/* Matrix Multiply
 * result(3x3) = m1(3x3) * m2(3x3)
 */
void mul_Mat3x3_Mat3x3( ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2 );
void mul_Mat3x3_Mat3x3_d( ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2);

/* Matrix Multiply w/ Transpose
 * result(3x3) = m1.T(3x3) * m2(3x3)
 */
void mul_Mat3x3_Trans_Mat3x3( ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2 );
void mul_Mat3x3_Trans_Mat3x3_d( ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2);

/* Matrix Multiply w/ Transpose
 * result(3x3) = m1(3x3) * m2.T(3x3)
 */
void mul_Mat3x3_Mat3x3_Trans( ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2 );
void mul_Mat3x3_Mat3x3_Trans_d( ixMatrix3d result, const ixMatrix3d m1, const ixMatrix3d m2);

/* Matrix addition
 * result(3x3) = m1(3x3) + m2(3x3)
 */
void add_Mat3x3_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2);

/* Matrix subtraction
 * result(3x3) = m1(3x3) - m2(3x3)
 */
void sub_Mat3x3_Mat3x3(ixMatrix3 result, const ixMatrix3 m1, const ixMatrix3 m2);

/* Matrix Multiply
 * result(1x2) = m(2x2) * v(2x1)
 */
void mul_Mat2x2_Vec2x1( ixVector2 result, const ixMatrix2 m, const ixVector2 v );

/* Matrix Multiply w/ Transpose
 * result(1x2) = m(2x2).T * v(2x1)
 */
void mul_Mat2x2_Trans_Vec2x1( ixVector2 result, const ixMatrix2 m, const ixVector2 v );

/* Matrix Multiply
 * result(1x3) = m(3x3) * v(3x1)
 * (9 multiplies, 6 adds)
 */
void mul_Mat3x3_Vec3x1( ixVector3 result, const ixMatrix3 m, const ixVector3 v );

/* Matrix Multiply w/ Transpose
 * result(1x3) = m(3x3).T * v(3x1)
 */
void mul_Mat3x3_Trans_Vec3x1( ixVector3 result, const ixMatrix3 m, const ixVector3 v );

/* Matrix Multiply
 * result(1x4) = m(4x4) * v(4x1)
 */
void mul_Mat4x4_Vec4x1( ixVector4 result, const ixMatrix4 m, const ixVector4 v );

/* Matrix Multiply w/ Transpose
 * result(1x4) = m(4x4).T * v(4x1)
 */
void mul_Mat4x4_Trans_Vec4x1( ixVector4 result, const ixMatrix4 m, const ixVector4 v );

/* Negate 
*/
void neg_Mat3x3( ixMatrix3 result, const ixMatrix3 m);

/* Multiply
 * result(3x3) = m(3x3) .* x
 */
void mul_Mat3x3_X( ixMatrix3 result, const ixMatrix3 m, const f_t x );

/* Divide
 * result(3x3) = m(3x3) ./ x
 */
void div_Mat3x3_X( ixMatrix3 result, const ixMatrix3 m, const f_t x );

/* Multiply
 * result(3x3) = v1(3x1) * v2(1x3)
 */
void mul_Vec3x1_Vec1x3( ixMatrix3 result, const ixVector3 v1, const ixVector3 v2 );

/* Multiply
 * result(2) = v1(2) * v2(2)
 */
void mul_Vec2_Vec2(ixVector2 result, const ixVector2 v1, const ixVector2 v2);

/* Multiply
 * result(3) = v1(3) * v2(3)
 */
void mul_Vec3_Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2 );

/* Multiply
 * result(4) = v1(4) .* v2(4)
 */
void mul_Vec4_Vec4( ixVector4 result, const ixVector4 v1, const ixVector4 v2 );

/* Square Root
 * result(3) = .sqrt(v(3))
 */
void sqrt_Vec3( ixVector3 result, const ixVector3 v );

/* Square Root
 * result(4) = .sqrt(v(4))
 */
void sqrt_Vec4( ixVector4 result, const ixVector4 v );

/* Absolute Value
 * result(n) = .abs(v(n))
 */
void abs_Vec2( ixVector2 result, const ixVector2 v );
void abs_Vec3( ixVector3 result, const ixVector3 v );
void abs_Vec4( ixVector4 result, const ixVector4 v );

void abs_Vec2d( ixVector2d result, const ixVector2d v );
void abs_Vec3d( ixVector3d result, const ixVector3d v );
void abs_Vec4d( ixVector4d result, const ixVector4d v );

/* Dot product
 * result = v(n) dot v(n)
 */
f_t dot_Vec2(const ixVector2 v);
f_t dot_Vec3(const ixVector3 v);
f_t dot_Vec4(const ixVector4 v);
double dot_Vec2d(const ixVector2d v);
double dot_Vec3d(const ixVector3d v);
double dot_Vec4d(const ixVector4d v);

/* Dot product
 * result = v1(n) dot v2(n)
 */
f_t dot_Vec2_Vec2(const ixVector2 v1, const ixVector2 v2 );
f_t dot_Vec3_Vec3(const ixVector3 v1, const ixVector3 v2 );
f_t dot_Vec4_Vec4(const ixVector4 v1, const ixVector4 v2 );
double dot_Vec2d_Vec2d(const ixVector2d v1, const ixVector2d v2 );
double dot_Vec3d_Vec3d(const ixVector3d v1, const ixVector3d v2 );
double dot_Vec4d_Vec4d(const ixVector4d v1, const ixVector4d v2 );

/* Vector magnitude
 * result = sqrt( v(n) dot v(n) )
 */
f_t mag_Vec2( const ixVector2 v);
f_t mag_Vec3( const ixVector3 v);
f_t mag_Vec4( const ixVector4 v);

/* Cross product
 * result(3) = v1(3) x v2(3)
 */
void cross_Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2 );
void crossd_Vec3( ixVector3d result, const ixVector3 v1, const ixVector3 v2 );

// /* Vector length
//  * result(3) = sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] )
//  */
// f_t length_Vec3( ixVector3 v );

/* Multiply
 * result(2x1) = v(2) .* x
 */
void mul_Vec2_X( ixVector2 result, const ixVector2 v, const f_t x );
void mul_Vec2d_X( ixVector2d result, const ixVector2d v, const double x );

/* Multiply
 * result(3x1) = v(3) .* x
 */
void mul_Vec3_X( ixVector3 result, const ixVector3 v, const  f_t x );
void mul_Vec3d_X( ixVector3d result, const ixVector3d v, const double x );

/* Multiply
 * result(4x1) = v(4) .* x
 */
void mul_Vec4_X( ixVector4 result, const ixVector4 v, const f_t x );
void mul_Vec4d_X( ixVector4d result, const ixVector4d v, const double x );

/* Divide
 * result(3x1) = v(3) ./ x
 */
void div_Vec3_X( ixVector3 result, const ixVector3 v, const f_t x );
void div_Vec3d_X( ixVector3d result, const ixVector3d v, const double x );

/* Divide
 * result(4x1) = v(4) ./ x
 */
void div_Vec4_X( ixVector4 result, const ixVector4 v, const f_t x );
void div_Vec4d_X( ixVector4d result, const ixVector4d v, const double x );


/* Add
 * result(2) = v1(2) + v2(2)
 */
void add_Vec2_Vec2(ixVector2 result, const ixVector2 v1, const ixVector2 v2);

/* Add
 * result(3) = v1(3) + v2(3)
 */
void add_Vec3_Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2 );
void add_Vec3d_Vec3d( ixVector3d result, const ixVector3d v1, const ixVector3d v2 );

/* Add
 * result(3) = k1*v1(3) + k2*v2(3)
 */
void add_K1Vec3_K2Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2, float k1, float k2);

/* Add
 * result(4) = v1(4) + v2(4)
 */
void add_Vec4_Vec4( ixVector4 result, const ixVector4 v1, const ixVector4 v2 );
void add_Vec4d_Vec4d( ixVector4d result, const ixVector4d v1, const ixVector4d v2 );

/* Subtract
 * result(3) = v1(3) - v2(3)
 */
void sub_Vec3_Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2 );
void sub_Vec3d_Vec3d( ixVector3d result, const ixVector3d v1, const ixVector3d v2 );

/* Subtract
 * result(2) = v1(2) - v2(2)
 */
void sub_Vec2_Vec2( ixVector2 result, const ixVector2 v1, const ixVector2 v2 );

/* Subtract
 * result(4) = v1(4) +- v2(4)
 */
void sub_Vec4_Vec4( ixVector4 result, const ixVector4 v1, const ixVector4 v2 );

/* Divide
 * result(3) = v1(3) ./ v2(3)
 */
void div_Vec3_Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2 );

/* Divide
 * result(4) = v1(4) ./ v2(4)
 */
void div_Vec4_Vec4( ixVector4 result, const ixVector4 v1, const ixVector4 v2 );

/* Negate*/
void neg_Vec3( ixVector3 result, const ixVector3 v );

/* Average
 * result(3) = (v1(3) + v2(3)) * 0.5
 */
void mean_Vec3_Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2 );
void mean_Vec3d_Vec3d( ixVector3d result, const ixVector3d v1, const ixVector3d v2 );


/* Min of vector elements
 * = min( v[0], v[1], v[2] )
 */
static __inline f_t min_Vec3_X(const ixVector3 v )
{
	f_t val = v[0];
	
    if( val > v[1] )
		val = v[1];

    if( val > v[2] )
		val = v[2];
		
	return val;
}

/* Max of vector elements
 * = max( v[0], v[1], v[2] )
 */
static __inline f_t max_Vec3_X(const ixVector3 v )
{
	f_t val = v[0];
	
    if( val < v[1] )
		val = v[1];

    if( val < v[2] )
		val = v[2];
		
	return val;
}

/* Max of vector elements
 * = max( fabsf(v[0]), fabsf(v[1]), fabsf(v[2]) )
 */
static __inline f_t abs_Vec3_X(const ixVector3 v )
{
    f_t result = fabsf(v[0]);
    f_t val1   = fabsf(v[1]);
    f_t val2   = fabsf(v[2]);

    if( result < val1 )
        result = val1;

    if( result < val2 )
        result = val2;
		
    return result;
}

/* Max of vector elements
 * = max( v1, v[1], v[2] }
 */
static __inline void min_Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2 )
{
	result[0] = _MIN(v1[0], v2[0]);
	result[1] = _MIN(v1[1], v2[1]);
	result[2] = _MIN(v1[2], v2[2]);
}

/* Max of vector elements
 * = max( v1, v[1], v[2] }
 */
static __inline void max_Vec3( ixVector3 result, const ixVector3 v1, const ixVector3 v2 )
{
	result[0] = _MAX(v1[0], v2[0]);
	result[1] = _MAX(v1[1], v2[1]);
	result[2] = _MAX(v1[2], v2[2]);
}

/* Zero vector
 * v(2) = { 0, 0 }
 */
static __inline void zero_Vec2( ixVector2 v )
{
	v[0] = 0.0f;
	v[1] = 0.0f;
}
static __inline void zero_Vec2d( ixVector2d v )
{
	v[0] = 0.0;
	v[1] = 0.0;
}

/* Zero vector
 * v(3) = { 0, 0, 0 }
 */
static __inline void zero_Vec3( ixVector3 v )
{
	v[0] = 0.0f;
	v[1] = 0.0f;
	v[2] = 0.0f;
}
static __inline void zero_Vec3d( ixVector3d v )
{
	v[0] = 0.0;
	v[1] = 0.0;
	v[2] = 0.0;
}

/* Zero vector
 * v(4) = { 0, 0, 0 }
 */
static __inline void zero_Vec4( ixVector4 v )
{
	v[0] = 0.0f;
	v[1] = 0.0f;
	v[2] = 0.0f;
	v[3] = 0.0f;
}
static __inline void zero_Vec4d( ixVector4d v )
{
	v[0] = 0.0;
	v[1] = 0.0;
	v[2] = 0.0;
	v[3] = 0.0;
}

/* Zero vector
* v(n) = { 0, ..., 0 }
*/
static __inline void zero_VecN( f_t *v, i_t n )
{
	for (int i=0; i<n; i++)
	{
		v[i] = 0.0f;
	}
}

/* Zero matrix
* m(m,n) = { 0, ..., 0 }
*/
static __inline void zero_MatMxN( f_t *M, i_t m, i_t n )
{
	for (int i=0; i<(m*n); i++)
	{
		M[i] = 0.0f;
	}
}

/**
 * Return 1 if 3x3 matrix is an identity, 0 if not.
 */
int mat3x3_IsIdentity(const f_t m[]);

/* Copy vector
 * result(3) = v(3)
 */
static __inline void cpy_Vec3_Vec3( ixVector3 result, const ixVector3 v )
{
	result[0] = v[0];
	result[1] = v[1];
	result[2] = v[2];
}
static __inline void cpy_Vec3d_Vec3d( ixVector3d result, const ixVector3d v )
{
	result[0] = v[0];
	result[1] = v[1];
	result[2] = v[2];
}
static __inline void cpy_Vec3d_Vec3( ixVector3d result, const  ixVector3 v )
{
	result[0] = (double)v[0];
	result[1] = (double)v[1];
	result[2] = (double)v[2];
}
static __inline void cpy_Vec3_Vec3d( ixVector3 result, const ixVector3d v )
{
	result[0] = (f_t)v[0];
	result[1] = (f_t)v[1];
	result[2] = (f_t)v[2];
}

/* Copy vector
 * result(4) = v(4)
 */
static __inline void cpy_Vec4_Vec4( ixVector4 result, const ixVector4 v )
{
	result[0] = v[0];
	result[1] = v[1];
	result[2] = v[2];
	result[3] = v[3];
}
static __inline void cpy_Vec4d_Vec4d( ixVector4d result, const ixVector4d v )
{
	result[0] = v[0];
	result[1] = v[1];
	result[2] = v[2];
	result[3] = v[3];
}
static __inline void cpy_Vec4d_Vec4( ixVector4d result, const ixVector4 v )
{
	result[0] = (double)v[0];
	result[1] = (double)v[1];
	result[2] = (double)v[2];
	result[3] = (double)v[3];
}
static __inline void cpy_Vec4_Vec4d( ixVector4 result, const ixVector4d v )
{
	result[0] = (f_t)v[0];
	result[1] = (f_t)v[1];
	result[2] = (f_t)v[2];
	result[3] = (f_t)v[3];
}

/* Copy vector
* result(n) = v(n)
*/
static __inline void cpy_VecN_VecN( f_t *result, const f_t *v, i_t n )
{
	for (int i=0; i<n; i++)
	{
		result[i] = v[i];
	}
}

/* Copy matrix
* result(mxn) = M(mxn)
*/
static __inline void cpy_MatMxN( f_t *result, const f_t *M, i_t m, i_t n )
{
	for (int i=0; i<(m*n); i++)
	{
		result[i] = M[i];
	}
}

/* Copy matrix A(mxn) into result(rxc) matrix, starting at offset_r, offset_c.  Matrix A must fit inside result matrix dimensions.
* result(rxc) <= A(mxn)
*/
void cpy_MatRxC_MatMxN( f_t *result, i_t r, i_t c, i_t r_offset, i_t c_offset, f_t *A, i_t m, i_t n );


/* Matrix Transpose
 * result(2x2) = m(2x2)'
 */
void transpose_Mat2( ixMatrix2 result, const ixMatrix2 m );

/* Matrix Transpose
 * result(3x3) = m(3x3)'
 */
void transpose_Mat3( ixMatrix3 result, const ixMatrix3 m );

/* Matrix Transpose
 * result(4x4) = m(4x4)'
 */
void transpose_Mat4( ixMatrix4 result, const ixMatrix4 m );

/*
 * Invert a 2x2 matrix.
 */
char inv_Mat2( ixMatrix2 result, ixMatrix2 m );

/* Matrix Inverse
 * result(3x3) = m(3x3)^-1
 * return 0 on success, -1 on numerical error
 */
char inv_Mat3( ixMatrix3 result, const ixMatrix3 m );

/* Matrix Inverse
 * result(4x4) = m(4x4)^-1
 * return 0 on success, -1 on numerical error
 */
char inv_Mat4( ixMatrix4 result, const ixMatrix4 m );

/*
 * Normalize 2 dimensional vector
 */
static __inline void normalize_Vec2( ixVector2 v )
{
    // Normalize vector
    mul_Vec2_X( v, v, RECIPNORM_VEC2(v) );
}

/*
 * Normalize 3 dimensional vector
 */
static __inline void normalize_Vec3( ixVector3 result, const ixVector3 v )
{
    // Normalize vector
	mul_Vec3_X( result, v, RECIPNORM_VEC3(v) );
}

/*
 * Normalize 4 dimensional vector
 */
static __inline void normalize_Vec4( ixVector4 result, const ixVector4 v )
{
    // Normalize vector
    mul_Vec4_X( result, v, RECIPNORM_VEC4(v) );
}
static __inline void normalize_Vec4d( ixVector4d result, const ixVector4d v )
{
	// Normalize vector
	mul_Vec4d_X( result, v, RECIPNORM_VEC4D(v) );
}

/*
* Check if 2 dimensional vectors are equal
*/
static __inline int is_equal_Vec2(const ixVector2 v1, const ixVector2 v2)
{
	return
		(v1[0] == v2[0]) &&
		(v1[1] == v2[1]);
}

/*
* Check if 3 dimensional vectors are equal
*/
static __inline int is_equal_Vec3(const ixVector3 v1, const ixVector3 v2)
{
	return
		(v1[0] == v2[0]) &&
		(v1[1] == v2[1]) &&
		(v1[2] == v2[2]);
}

/*
* Check if 4 dimensional vectors are equal
*/
static __inline int is_equal_Vec4(const ixVector4 v1, const ixVector4 v2)
{
	return
		(v1[0] == v2[0]) &&
		(v1[1] == v2[1]) &&
		(v1[2] == v2[2]) &&
		(v1[3] == v2[3]);
}

/*
* Limit 3 dimensional vector to +- specified limit
*/
static __inline void limit_Vec3( ixVector3 v, f_t limit )
{ 	
	_LIMIT( v[0], limit );
	_LIMIT( v[1], limit );
	_LIMIT( v[2], limit );
}

/*
* Limit 3 dimensional vector to min and max values
*/
static __inline void limit2_Vec3( ixVector3 v, f_t min, f_t max )
{
	_LIMIT2( v[0], min, max );
	_LIMIT2( v[1], min, max );
	_LIMIT2( v[2], min, max );
}


/* Array contains NAN
 * return 1 if NAN found in array, 0 if not
 */
static __inline int isNan_array( f_t *a, int size )
{
    int i;

    for( i=0; i<size; i++ )
    {
        if( IS_NAN(a[i]) )
            return 1;
    }

    return 0;
}


/* Array contains NAN
 * return 1 if NAN found in double array, 0 if not
 */
static __inline int isNan_array_d( double *a, int size )
{
    int i;

    for( i=0; i<size; i++ )
    {
        if( IS_NAN(a[i]) )
            return 1;
    }

    return 0;
}

#if defined(PLATFORM_IS_WINDOWS)
#pragma warning( push )
#pragma warning( disable : 4723)
#endif

/* Array contains INF
 * return 1 if INF found in array, 0 if not
 */
static __inline int isInf_array( f_t *a, int size )
{
    int i;

    f_t tmp = 1.0f;
    f_t inf = 1.0f / (tmp - 1.0f);

    for( i=0; i<size; i++ )
    {
        if( a[i] == inf )
            return 1;
    }

    return 0;
}


/* Array contains INF
* return 1 if INF found in double array, 0 if not
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
#pragma warning( pop ) 
#endif

/* Array does not contain NAN or INF
 * return 0 if NAN or INF found in array, 1 if not
 */
static __inline int isFinite_array( f_t *a, int size )
{
    if( isNan_array(a, size) )
        return 0;

    if( isInf_array(a, size) )
        return 0;

    return 1;
}


/* Array does not contain NAN or INF
* return 0 if NAN or INF found in double array, 1 if not
*/
static __inline int isFinite_array_d(double *a, int size)
{
	if (isNan_array_d(a, size))
		return 0;

	if (isInf_array_d(a, size))
		return 0;

	return 1;
}


// Low-Pass Alpha Filter
void LPFO0_init_Vec3( sLpfO0 *lpf, f_t dt, f_t cornerFreqHz, const ixVector3 initVal );
// output[n+1] = beta*output[n] + alpha*input
void LPFO0_Vec3( sLpfO0 *lpf, const ixVector3 input );

// Zero order Low-Pass Filter
static __inline void O0_LPF_Vec3( ixVector3 result, const ixVector3 input, f_t alph, f_t beta )
{
	ixVector3 tmp3;

	// val[n+1] = beta*val[n] + alpha*input
	mul_Vec3_X(		tmp3,	input,	alph );
	mul_Vec3_X(		result,	result,	beta );
	add_Vec3_Vec3(  result,	result,	tmp3 );
}


// First order Low-Pass Filter
static __inline void O1_LPF_Vec3( ixVector3 result, const ixVector3 input, ixVector3 c1, f_t alph, f_t beta, f_t dt )
{
	ixVector3 tmp3;

	// Estimate next models coefficients:			d1 = (input - result) / dt
	sub_Vec3_Vec3( tmp3, input, result );
	div_Vec3_X( tmp3, tmp3, dt );

	// LPF these coefficients:						c1 = beta*c1 + alph*d1
	O0_LPF_Vec3( c1, tmp3, alph, beta );

	// Current state estimates:						est = (last result) + c1*dt
	mul_Vec3_X( tmp3, c1, dt );
	add_Vec3_Vec3( result, result, tmp3 );
	
	// LPF input into state estimates:				result = beta*est + alph*input
	O0_LPF_Vec3( result, input, alph, beta );	
}



#ifdef __cplusplus
}
#endif

#endif /* MATRIX_H_ */
