/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef MATH_TYPES_H_
#define MATH_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#define _MATH_DEFINES_DEFINED
#include <math.h>

#ifndef UNWRAP_F64
#define UNWRAP_F64(x)			{while( (x) > (C_PI) )   (x) -= (C_TWOPI);   while( (x) < (-C_PI) )   (x) += (C_TWOPI);}	// unwrap to +- PI
#endif

#ifndef UNWRAP_F
#define UNWRAP_F(x)				{while( (x) > (C_PI_F) ) (x) -= (C_TWOPI_F); while( (x) < (-C_PI_F) ) (x) += (C_TWOPI_F);}	// unwrap to +- PI
#endif

//_____ M A C R O S ________________________________________________________

#if 1
	// Use single precision floating point
	typedef float       f_t;
	#define _SIN        sinf
	#define _COS        cosf
	#define _TAN        tanf
	#define _ASIN       asinf
	#define _ACOS       acosf
	#define _ATAN2      atan2f
	#define _SQRT       sqrtf
	#define _FABS       fabsf
	#define _LOG		logf

	#define _DEG2RAD    C_DEG2RAD_F
	#define _RAD2DEG    C_RAD2DEG_F
	#define _ZERO		0.0f

	#define _UNWRAP     UNWRAP_F
#else
	// Use double precision floating point
	typedef double      f_t;
	#define _SIN        sin
	#define _COS        cos
	#define _TAN        tan
	#define _ASIN       asin
	#define _ACOS       acos
	#define _ATAN2      atan2
	#define _SQRT       sqrt
	#define _FABS       fabs
	#define _LOG		log

	#define _DEG2RAD    C_DEG2RAD
	#define _RAD2DEG    C_RAD2DEG
	#define _ZERO		0.0

	#define _UNWRAP     UNWRAP_F64
#endif


//_____ D E F I N I T I O N S ______________________________________________


typedef int			i_t;

// typedef f_t         v_t[4];
// typedef v_t         m_t[4];
// typedef uint8_t     iter_t;

typedef double      Vector2d[3];    // V = | 0 1 |
typedef f_t         Vector2[3];     // V = | 0 1 |
typedef double      Vector3d[3];    // V = | 0 1 2 |
typedef f_t         Vector3[3];     // V = | 0 1 2 |
typedef double      Vector4d[4];    // V = | 0 1 2 3 |
typedef f_t         Vector4[4];     // V = | 0 1 2 3 |
typedef f_t         Vector5[5];     // V = | 0 1 2 3 4 |
typedef f_t         Vector6[6];     // V = | 0 1 2 3 4 5 |
typedef Vector4     Quat;		    // w,x,y,z
typedef Vector3     Euler;          // roll,pitch,yaw
typedef f_t         Matrix2[4];
typedef f_t         Matrix3[9];
typedef f_t         Matrix4[16];
typedef f_t         Matrix5[25];
typedef f_t         Matrix6[36];
//typedef f_t       Matrix[4][4];
// Matrix Layout
//        |  0  1  2  3 |
//        |             |              | 0 1 2 |
//        |  4  5  6  7 |              |       |            | 0 1 |
//    M = |             |          M = | 3 4 5 |        M = |     |
//        |  8  9 10 11 |              |       |            | 2 3 |
//        |             |              | 6 7 8 |
//        | 12 13 14 15 |


//_____ G L O B A L S ______________________________________________________

//_____ P R O T O T Y P E S ________________________________________________



#ifdef __cplusplus
}
#endif

#endif /* MATH_TYPES_H_ */
