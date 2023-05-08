/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#define _MATH_DEFINES_DEFINED
#include <math.h>

// #include "misc/debug.h"
#include "ISConstants.h"
#include "ISPose.h"
#include "ISEarth.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ G L O B A L S ______________________________________________________

//_____ L O C A L   P R O T O T Y P E S ____________________________________

//_____ F U N C T I O N S __________________________________________________

/*
 * Initialize Quaternion q = [w, x, y, z]
 */
void quat_init(ixQuat q)
{
#if 1
	q[0] = 1.0;
	q[1] = q[2] = q[3] = 0.0f;
#else
	ixEuler theta = { 0.0f, 0.0f, 0.0f };
	euler2quat(theta, q);
#endif
}


/* Quaternion Conjugate: q* = [ w, -x, -y, -z ] of quaterion q = [ w, x, y, z ] 
 * Rotation in opposite direction.
 */
void quatConj(ixQuat result, const ixQuat q)
{
    result[0] =  q[0];
    result[1] = -q[1];
    result[2] = -q[2];
    result[3] = -q[3];
}


/* 
 * Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!  
 * Combines two quaternion rotations into one rotation.
 * result = q1 * q2
 * Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
 */
void mul_Quat_Quat(ixQuat result, const ixQuat q1, const ixQuat q2)
{
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2];
    result[2] = q1[0]*q2[2] + q1[1]*q2[3] + q1[2]*q2[0] - q1[3]*q2[1];
    result[3] = q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] + q1[3]*q2[0];
}

/*
* Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!
* Combines two quaternion rotations into one rotation.
* result = quatConj(q1) * q2
* Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
*/
void mul_ConjQuat_Quat(ixQuat result, const ixQuat qc, const ixQuat q2)
{
    result[0] = qc[0]*q2[0] + qc[1]*q2[1] + qc[2]*q2[2] + qc[3]*q2[3];
    result[1] = qc[0]*q2[1] - qc[1]*q2[0] + qc[2]*q2[3] - qc[3]*q2[2];
    result[2] = qc[0]*q2[2] - qc[1]*q2[3] - qc[2]*q2[0] + qc[3]*q2[1];
    result[3] = qc[0]*q2[3] + qc[1]*q2[2] - qc[2]*q2[1] - qc[3]*q2[0];
}

/*
* Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!
* Combines two quaternion rotations into one rotation.
* result = q1 * quatConj(q2)
* Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
*/
void mul_Quat_ConjQuat(ixQuat result, const ixQuat q1, const ixQuat qc)
{
    result[0] =  q1[0]*qc[0] + q1[1]*qc[1] + q1[2]*qc[2] + q1[3]*qc[3];
    result[1] = -q1[0]*qc[1] + q1[1]*qc[0] + q1[2]*qc[3] - q1[3]*qc[2];
    result[2] = -q1[0]*qc[2] - q1[1]*qc[3] + q1[2]*qc[0] + q1[3]*qc[1];
    result[3] = -q1[0]*qc[3] + q1[1]*qc[2] - q1[2]*qc[1] + q1[3]*qc[0];
}

/*
 * Division of two Quaternions.  Order matters!!!
 * result = q1 / q2. 
 * Reference: http://www.mathworks.com/help/aeroblks/quaterniondivision.html
 */
void div_Quat_Quat(ixQuat result, const ixQuat q1, const ixQuat q2)
{
	f_t d = (f_t)1.0 / (q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2] + q1[3] * q1[3]);

    result[0] = (q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]) * d;
    result[1] = (q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2]) * d;
    result[2] = (q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1]) * d;
    result[3] = (q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]) * d;
}


/* Quaternion rotation from vector v1 to vector v2.
 */
void quat_Vec3_Vec3(ixQuat result, const ixVector3 v1, const ixVector3 v2)
{
    ixVector3 w1, w2;
    
    // Normalize input vectors
    mul_Vec3_X( w1, v1, recipNorm_Vec3(v1) );
    mul_Vec3_X( w2, v2, recipNorm_Vec3(v2) );
 
    // q[1:3]
	cross_Vec3( &result[1], w1, w2 );

    // q[0]
    result[0] = (f_t)(_SQRT( dot_Vec3(w1) * dot_Vec3(w1) ) + dot_Vec3_Vec3(w1, w2));

	// Normalize quaternion
	div_Vec4_X( result, result, mag_Vec4(result) );
}


/* Computationally simple means to apply quaternion rotation to a vector.
 * Requires quaternion be normalized first.
 * If quaternion describes current attitude, then rotation is body -> inertial frame.
 * Equivalent to a DCM.T * vector multiply.
 */
void quatRot(ixVector3 result, const ixQuat q, const ixVector3 v)
{
    ixVector3 t;
    cross_Vec3( t, &q[1], v );
    mul_Vec3_X( t, t, 2.0f );

    cross_Vec3( result, &q[1], t );
    mul_Vec3_X( t, t, q[0] );
    add_Vec3_Vec3( result, result, t );
    add_Vec3_Vec3( result, result, v );
}


/* Computationally simple means to apply quaternion conjugate (opposite) rotation to a vector
 * (18 multiplies, 6 subtracts, 6 adds).  Using a DCM uses (27 multiplies, 12 adds, 6 subtracts).
 * Requires quaternion be normalized first.
 * If quaternion describes current attitude, then rotation is inertial -> body frame.
 * Equivalent to a DCM * vector multiply.
 */
void quatConjRot(ixVector3 result, const ixQuat q, const ixVector3 v)
{
    ixQuat qC;
    ixVector3 t;

    // Rotation in opposite direction
    quatConj( qC, q );

    cross_Vec3( t, &qC[1], v );
    mul_Vec3_X( t, t, 2.0f );

    cross_Vec3( result, &qC[1], t );
    mul_Vec3_X( t, t, qC[0] );
    add_Vec3_Vec3( result, result, t );
    add_Vec3_Vec3( result, result, v );
}


/*
 * This will convert from quaternions to euler angles
 * q(W,X,Y,Z) -> euler(phi,theta,psi) (rad)
 *
 * Reference: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
void quat2euler(const ixQuat q, ixEuler theta)
{
    float sinang = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sinang > 1.0f) { sinang = 1.0f; }
    if (sinang < -1.0f) { sinang = -1.0f; }

	theta[0] = _ATAN2(2.0f * (q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]));
	theta[1] = _ASIN (sinang);
	theta[2] = _ATAN2(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));
}
void quat2phiTheta(const ixQuat q, f_t *phi, f_t *theta)
{
    float sinang = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sinang > 1.0f) { sinang = 1.0f; }
    if (sinang < -1.0f) { sinang = -1.0f; }

	*phi	= _ATAN2( 2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]) );
	*theta  = _ASIN (sinang);
}
void quat2psi(const ixQuat q, f_t *psi)
{
	float sinang = 2.0f * (q[0] * q[2] - q[3] * q[1]);
	if (sinang > 1.0f) { sinang = 1.0f; }
	if (sinang < -1.0f) { sinang = -1.0f; }

	*psi = _ATAN2(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]));
}


/*
 * This will convert from euler angles to quaternion vector
 * euler(phi,theta,psi) (rad) -> q(W,X,Y,Z)
 */
void euler2quat(const ixEuler euler, ixQuat q)
{
	f_t hphi = euler[0] * 0.5f;
	f_t hthe = euler[1] * 0.5f;
	f_t hpsi = euler[2] * 0.5f;

	f_t shphi = _SIN(hphi);
	f_t chphi = _COS(hphi);

	f_t shthe = _SIN(hthe);
	f_t chthe = _COS(hthe);

	f_t shpsi = _SIN(hpsi);
	f_t chpsi = _COS(hpsi);

	q[0] = chphi * chthe * chpsi + shphi * shthe * shpsi;
	q[1] = shphi * chthe * chpsi - chphi * shthe * shpsi;
	q[2] = chphi * shthe * chpsi + shphi * chthe * shpsi;
	q[3] = chphi * chthe * shpsi - shphi * shthe * chpsi;
}


/*
* Convert ECEF quaternion to NED euler at specified LLA (rad)
*/
void qe2b2EulerNedLLA(ixVector3 eul, const ixVector4 qe2b, const ixVector3d lla)
{
	ixVector3 eulned;
	ixVector4 qe2n;
	ixVector4 qn2b;

	eulned[0] = 0.0f;
	eulned[1] = ((float)-lla[0]) - 0.5f * C_PI_F;
	eulned[2] = (float)lla[1];
	euler2quat(eulned, qe2n);
	mul_Quat_ConjQuat(qn2b, qe2b, qe2n);
	quat2euler(qn2b, eul);
}


/*
 * This will construct a direction cosine matrix from
 * the psi angle - rotates from NE to body frame
 *
 * body = tBL(2,2)*NE
 *
 */
void psiDCM(const f_t psi, ixMatrix2 m)
{
	f_t cpsi = _COS(psi);  // cos(psi)
	f_t spsi = _SIN(psi);  // sin(psi)

	// Row 1
	m[0] =  cpsi;
	m[1] =  spsi;
	// Row 2
    m[2] = -spsi;
	m[3] =  cpsi;
}


/*
* This will extract the psi euler angle from a direction cosine matrix in the
* standard rotation sequence, for either a 2x2 or 3x3 DCM matrix.
* [phi][theta][psi] from NED to body frame
*
* body = tBL(2,2)*NE
* body = tBL(3,3)*NED
*
* reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
*/
f_t DCMpsi(const f_t *m )
{
	return _ATAN2( m[1], m[0] );
}


/*
 * This will construct a direction cosine matrix from
 * euler angles in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 *
 * reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void eulerDCM(const ixEuler euler, ixMatrix3 m)
{
	f_t cphi = _COS(euler[0]);  // cos(phi)
	f_t cthe = _COS(euler[1]);  // cos(theta)
	f_t cpsi = _COS(euler[2]);  // cos(psi)

	f_t sphi = _SIN(euler[0]);	// sin(phi)
	f_t sthe = _SIN(euler[1]);  // sin(theta)
	f_t spsi = _SIN(euler[2]);  // sin(psi)

	// Row 1
	m[0] =  cpsi*cthe;
	m[1] =  spsi*cthe;
	m[2] = -sthe;
	// Row 2
	m[3] = -spsi*cphi + cpsi*sthe*sphi;
	m[4] =  cpsi*cphi + spsi*sthe*sphi;
	m[5] =  cthe*sphi;
	// Row 3
	m[6] =  spsi*sphi + cpsi*sthe*cphi;
	m[7] = -cpsi*sphi + spsi*sthe*cphi;
	m[8] =  cthe*cphi;
}


void phiThetaDCM(const ixEuler euler, ixMatrix3 m )
{
	f_t cphi = _COS( euler[0] );	// cos(phi)
	f_t cthe = _COS( euler[1] );	// cos(theta)

	f_t sphi = _SIN( euler[0] );	// sin(phi)
	f_t sthe = _SIN( euler[1] );	// sin(theta)

	// Row 1
	m[0] =  cthe;
	m[1] =  0.0f;
	m[2] = -sthe;
	// Row 2
	m[3] =  sthe*sphi;
	m[4] =  cphi;
	m[5] =  cthe*sphi;
	// Row 3
	m[6] =  sthe*cphi;
	m[7] = -sphi;
	m[8] =  cthe*cphi;
}


/*
* This will construct the transpose matrix of
* the direction cosine matrix from
* euler angles in the standard rotation sequence
* [phi][theta][psi] from NED to body frame
*
* body = tBL(3,3)*NED
*
* reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
*/
void eulerDCM_Trans(const ixEuler euler, ixMatrix3 m )
{
	f_t cphi = _COS( euler[0] );	// cos(phi)
	f_t cthe = _COS( euler[1] );	// cos(theta)
	f_t cpsi = _COS( euler[2] );	// cos(psi)

	f_t sphi = _SIN( euler[0] );	// sin(phi)
	f_t sthe = _SIN( euler[1] );	// sin(theta)
	f_t spsi = _SIN( euler[2] );	// sin(psi)

	// Col 1
	m[0] =  cpsi*cthe;
	m[3] =  spsi*cthe;
	m[6] = -sthe;
	// Col 2
	m[1] = -spsi*cphi + cpsi*sthe*sphi;
	m[4] =  cpsi*cphi + spsi*sthe*sphi;
	m[7] =  cthe*sphi;
	// Col 3
	m[2] =  spsi*sphi + cpsi*sthe*cphi;
	m[5] = -cpsi*sphi + spsi*sthe*cphi;
	m[8] =  cthe*cphi;
}


/*
 * This will extract euler angles from a direction cosine matrix in the
 * standard rotation sequence.
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 *
 * reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void DCMeuler(const ixMatrix3 m, ixEuler euler)
{
    euler[0] =  _ATAN2( m[5], m[8] );		// phi
    euler[1] =  _ASIN( -m[2] );				// theta
    euler[2] =  _ATAN2( m[1], m[0] );		// psi
}


/*
 * This will construct a direction cosine matrix from
 * quaternions in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame
 * (18 multiplies, 6 adds, 6 subtracts)
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void quatDCM(const ixQuat q, ixMatrix3 mat)
{
    f_t q0q1 = q[0]*q[1];
    f_t q0q2 = q[0]*q[2];
    f_t q0q3 = q[0]*q[3];
    f_t q1q1 = q[1]*q[1];
    f_t q1q2 = q[1]*q[2];
    f_t q1q3 = q[1]*q[3];
    f_t q2q2 = q[2]*q[2];
    f_t q2q3 = q[2]*q[3];
    f_t q3q3 = q[3]*q[3];

	// Row 1
	mat[0] = 1.0f - 2.0f * (q2q2 + q3q3);
	mat[1] =        2.0f * (q1q2 + q0q3);
	mat[2] =        2.0f * (q1q3 - q0q2);
	// Row 2
	mat[3] =        2.0f * (q1q2 - q0q3);
	mat[4] = 1.0f - 2.0f * (q1q1 + q3q3);
	mat[5] =        2.0f * (q2q3 + q0q1);
	// Row 3
	mat[6] =        2.0f * (q1q3 + q0q2);
	mat[7] =        2.0f * (q2q3 - q0q1);
	mat[8] = 1.0f - 2.0f * (q1q1 + q2q2);
}
void quatdDCM(const ixVector4d q, ixMatrix3 mat )
{
	f_t q0q1 = (f_t)(q[0] * q[1]);
	f_t q0q2 = (f_t)(q[0] * q[2]);
	f_t q0q3 = (f_t)(q[0] * q[3]);
	f_t q1q1 = (f_t)(q[1] * q[1]);
	f_t q1q2 = (f_t)(q[1] * q[2]);
	f_t q1q3 = (f_t)(q[1] * q[3]);
	f_t q2q2 = (f_t)(q[2] * q[2]);
	f_t q2q3 = (f_t)(q[2] * q[3]);
	f_t q3q3 = (f_t)(q[3] * q[3]);

	// Row 1
	mat[0] = 1.0f - 2.0f * (q2q2 + q3q3);
	mat[1] =        2.0f * (q1q2 + q0q3);
	mat[2] =        2.0f * (q1q3 - q0q2);
	// Row 2
	mat[3] =        2.0f * (q1q2 - q0q3);
	mat[4] = 1.0f - 2.0f * (q1q1 + q3q3);
	mat[5] =        2.0f * (q2q3 + q0q1);
	// Row 3
	mat[6] =        2.0f * (q1q3 + q0q2);
	mat[7] =        2.0f * (q2q3 - q0q1);
	mat[8] = 1.0f - 2.0f * (q1q1 + q2q2);
}

/*
 * This will construct quaternions from a direction cosine 
 * matrix in the standard rotation sequence.
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void DCMquat(const ixMatrix3 mat, ixQuat q)
{
    f_t d;

    q[0] = 0.5f * _SQRT(1.0f + mat[0] + mat[4] + mat[8]);
    
    d = 1.0f / (4.0f * q[0]);
    
    q[1] = d * (mat[5] - mat[7]);
    q[2] = d * (mat[6] - mat[2]);
    q[3] = d * (mat[1] - mat[3]);
}

/*
 * This will construct the euler omega-cross matrix
 * wx(3,3)
 * p, q, r (rad/sec)
 */
void eulerWx(const ixEuler euler, ixMatrix3 mat)
{
	f_t p = euler[0];
	f_t q = euler[1];
	f_t r = euler[2];

	// Row 1
	mat[0] =  0.0f;
	mat[1] = -r;
	mat[2] =  q;
	// Row 2
	mat[3] =  r;
	mat[4] =  0.0f;
	mat[5] = -p;
	// Row 3
	mat[6] = -q;
	mat[7] =  p;
	mat[8] =  0.0f;
}

/*
 * This will construct the quaternion omega matrix
 * W(4,4)
 * p, q, r (rad/sec)
 */
void quatW(const ixEuler euler, ixMatrix4 mat)
{
	f_t p = euler[0] * 0.5f;
	f_t q = euler[1] * 0.5f;
	f_t r = euler[2] * 0.5f;

	// Row 1
	mat[0]  =  0.0f;
	mat[1]  = -p;
	mat[2]  = -q;
	mat[3]  = -r;
	// Row 2
	mat[4]  =  p;
	mat[5]  =  0.0f;
	mat[6]  =  r;
	mat[7]  = -q;
	// Row 3
	mat[8]  =  q;
	mat[9]  = -r;
	mat[10] =  0.0f;
	mat[11] =  p;
	// Row 4
	mat[12] =  r;
	mat[13] =  q;
	mat[14] = -p;
	mat[15] =  0.0f;
}


/*
*   Convert quaternion to rotation axis (and angle).  Quaternion must be normalized.
*/
void quatRotAxis(const ixQuat q, ixVector3 pqr)
{
    // Normalize quaternion
//     mul_Vec4_X( q, q, 1/mag_Vec4(q) );

//     f_t theta = _ACOS( q[0] ) * (f_t)2.0;
    f_t sin_a, d;
    
    sin_a = _SQRT( 1.0f - q[0] * q[0] );

    if ( _FABS( sin_a ) < 0.0005f ) 
        sin_a = 1.0f;

    d = 1.0f / sin_a;

    pqr[0] = q[1] * d;
    pqr[1] = q[2] * d;
    pqr[2] = q[3] * d;
}


/*
 *  Compute the derivative of the ixEuler angle psi with respect
 * to the quaternion Q.  The result is a row vector
 *
 * d(psi)/d(q0)
 * d(psi)/d(q1)
 * d(psi)/d(q2)
 * d(psi)/d(q3)
 */
void dpsi_dq(const ixQuat q, ixQuat dq)
{
	f_t t1 = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[2]);
	f_t t2 = 2.0f * (q[1]*q[2] + q[0]*q[3]);
	f_t err = 2.0f / ( t1*t1 + t2*t2 );

	dq[0] = err * (q[3]*t1);
	dq[1] = err * (q[2]*t1);
	dq[2] = err * (q[1]*t1 + 2.0f * q[2]*t2);
	dq[3] = err * (q[0]*t1 + 2.0f * q[3]*t2);
}


/*
 * NED to ixEuler
 */
void nedEuler(const ixVector3 ned, ixEuler e)
{
	e[0] = 0.0f;
	e[1] = _ATAN2( -ned[2], _SQRT( ned[0] * ned[0] + ned[1] * ned[1] ) );
	e[2] = _ATAN2( ned[1], ned[0] );
}


/*
 * ixEuler to NED
 */
void eulerNed(const ixEuler e, ixVector3 ned)
{
    ixVector3 v = { 1.0f, 0.0f, 0.0f };
    
    vectorBodyToReference( v, e, ned );
}


/*
 * Rotate eulers from body to inertial frame by ins eulers, in order: phi, theta, psi
 */
void eulerBodyToReference(const ixEuler e, const ixEuler rot, ixEuler result)
{
	ixMatrix3 Ai, At, AiAt;
	// Create DCMs (rotation matrices)
	eulerDCM(rot, Ai);
	eulerDCM(e, At);
	
	// Apply INS Rotation to Desired Target vector
	mul_Mat3x3_Mat3x3(AiAt, At, Ai);	// Apply rotation
	DCMeuler(AiAt, result);				// Pull out new eulers
}


/*
 * Rotate eulers from inertial to body frame by ins eulers, in order: psi, theta, phi
 */
void eulerReferenceToBody(const ixEuler e, const ixEuler rot, ixEuler result)
{
	ixMatrix3 Ai, At, AiAt;
	// Create DCMs (rotation matrices)
	eulerDCM(rot, Ai);
	eulerDCM(e, At);
	
	// Apply INS Rotation to Desired Target vector
	mul_Mat3x3_Mat3x3_Trans(AiAt, At, Ai);	// Apply rotation
	DCMeuler(AiAt, result);					// Pull out new eulers
}


/*
 * Rotate vector from body to inertial frame by euler angles, in order: phi, theta, psi
 */
void vectorBodyToReference(const ixVector3 v, const ixEuler rot, ixVector3 result)
{
	ixMatrix3 DCM;
    
	// Create DCM (rotation matrix)
	eulerDCM(rot, DCM);

	// Apply rotation to vector
    mul_Mat3x3_Trans_Vec3x1( result, DCM, v );
}


/*
 * Rotate vector from inertial to body frame by euler angles, in order: psi, theta, phi
 */
void vectorReferenceToBody(const ixVector3 v, const ixEuler rot, ixVector3 result)
{
	ixMatrix3 DCM;
    
	// Create DCM (rotation matrix)
	eulerDCM(rot, DCM);

	// Apply rotation to vector
    mul_Mat3x3_Vec3x1( result, DCM, v );
}


/*
 * Vector to euler roll angle
 */
float vectorToRoll(const ixVector3 v)
{
	return -atan2f(-v[2], v[1]);
}


/*
 * Vector to euler pitch angle
 */
float vectorToPitch(const ixVector3 v)
{
	float mag = mag_Vec3(v);
	if(mag == 0.0f)
	{	
		return 0.0f;
	}

	return asinf(v[0]/mag);
}

/*
 * Returns the pitch angle of the vector selected axis.
 */
float vectorSelectedAxisToPitch(const ixVector3 v, int pitchAxis)
{
	float mag = mag_Vec3(v);
	if (mag == 0.0f)
	{
		return 0.0f;
	}

	return asinf(v[pitchAxis] / mag);
}
