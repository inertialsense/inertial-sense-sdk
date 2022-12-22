/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef POSE_H_
#define POSE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ISMatrix.h"
#include "ISConstants.h"

#if (!defined (__cplusplus) && (!defined (inline)))
#       define inline __inline          // allow "inline" keyword to work in windows w/ c program
#endif


/*
 * Initialize Quaternion q = [w, x, y, z]
 */
void quat_init( ixQuat q );

/* Quaternion Conjugate: q* = [ w, -x, -y, -z ] of quaterion q = [ w, x, y, z ] 
 * Rotation in opposite direction.
 */
void quatConj( ixQuat result, const ixQuat q );

/* 
* Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!  
* Combines two quaternion rotations into one rotation.
* result = q1 * q2. 
* Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
*/
void mul_Quat_Quat( ixQuat result, const ixQuat q1, const ixQuat q2 );

/*
* Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!
* Combines two quaternion rotations into one rotation.
* result = quatConj(q1) * q2.
* Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
*/
void mul_ConjQuat_Quat( ixQuat result, const ixQuat qc, const ixQuat q2 );

/*
* Product of two Quaternions.  Order of q1 and q2 matters (same as applying two successive DCMs)!!!
* Combines two quaternion rotations into one rotation.
* result = q1 * quatConj(q2)
* Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
*/
void mul_Quat_ConjQuat(ixQuat result, const ixQuat q1, const ixQuat qc);

/*
 * Division of two Quaternions.  Order matters!!!
 * result = q1 / q2. 
 * Reference: http://www.mathworks.com/help/aeroblks/quaterniondivision.html
 */
void div_Quat_Quat( ixQuat result, const ixQuat q1, const ixQuat q2 );

/* 
 * Quaternion rotation from vector v1 to vector v2.
 */
void quat_Vec3_Vec3( ixQuat result, const ixVector3 v1, const ixVector3 v2 );

/* Computationally simple means to apply quaternion rotation to a vector.
 * Requires quaternion be normalized first.  
 * If quaternion describes current attitude, then rotation is body frame -> reference frame.
 */
void quatRot( ixVector3 result, const ixQuat q, const ixVector3 v );

/* Computationally simple means to apply quaternion conjugate (opposite) rotation to a vector
 * Requires quaternion be normalized first
 * If quaternion describes current attitude, then rotation is reference frame -> body frame.
 */
void quatConjRot( ixVector3 result, const ixQuat q, const ixVector3 v );

/*
 * This will convert from quaternions to euler angles
 * q(W,X,Y,Z) -> euler(phi,theta,psi) (rad)
 *
 * Reference: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
void quat2euler(const ixQuat q, ixEuler theta);
void quat2phiTheta(const ixQuat q, f_t *phi, f_t *theta);
void quat2psi(const ixQuat q, f_t *psi);

/*
 * This will convert from euler angles to quaternion vector
 * euler(phi,theta,psi) (rad) -> q(W,X,Y,Z)
 */
void euler2quat(const ixEuler euler, ixQuat q );



/*
 * This will construct a direction cosine matrix from
 * the psi angle - rotates from NE to body frame
 *
 * body = tBL(2,2)*NE
 *
 */
void psiDCM(const f_t psi, ixMatrix2 m);

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
f_t DCMpsi(const f_t *m );

/*
 * This will construct a direction cosine matrix from
 * euler angles in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
//const Matrix<3,3> eulerDCM( const Vector<3> & euler )
void eulerDCM(const ixEuler euler, ixMatrix3 m );
// Only use phi and theta (exclude psi) in rotation
void phiThetaDCM(const ixEuler euler, ixMatrix3 m );

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
void eulerDCM_Trans(const ixEuler euler, ixMatrix3 m );

/*
 * This will extract euler angles from a direction cosine matrix in the
 * standard rotation sequence.
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void DCMeuler(const ixMatrix3 m, ixEuler euler);


/*
 * This will construct a direction cosine matrix from
 * quaternions in the standard rotation  sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void quatDCM(const ixQuat q, ixMatrix3 mat);
void quatdDCM(const ixVector4d q, ixMatrix3 mat);

/*
 * This will construct a quaternion from direction 
 * cosine matrix in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame
 *
 * body = tBL(3,3)*NED
 * q(4,1)
 *
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 */
void DCMquat(const ixMatrix3 mat, ixQuat q);

/*
 * This will construct the euler omega-cross matrix
 * wx(3,3)
 * p, q, r (rad/sec)
 */
void eulerWx(const ixEuler euler, ixMatrix3 mat);

/*
 * This will construct the quaternion omega matrix
 * W(4,4)
 * p, q, r (rad/sec)
 */
void quatW(const ixEuler euler, ixMatrix4 mat);

/*
*   Convert quaternion to rotation axis (and angle).  Quaternion must be normalized.
*/
void quatRotAxis(const ixQuat q, ixVector3 pqr);

/*
 *  Compute the derivative of the ixEuler angle psi with respect
 * to the quaternion Q.  The result is a row vector
 *
 * d(psi)/d(q0)
 * d(psi)/d(q1)
 * d(psi)/d(q2)
 * d(psi)/d(q3)
 */
void dpsi_dq(const ixQuat q, ixQuat dq);

/*
 * NED to ixEuler
 */
void nedEuler(const ixVector3 ned, ixEuler e);

/*
 * ixEuler to NED
 */
void eulerNed(const ixEuler e, ixVector3 ned);

/*
 * Rotate theta eulers from body frame to reference frame by eulers, in order: phi, theta, psi
 */
void eulerBodyToReference(const  ixEuler e, const  ixEuler rot, ixEuler result);

/*
 * Rotate theta eulers from reference frame to body frame by eulers, in order: psi, theta, phi
 */
void eulerReferenceToBody(const  ixEuler e, const ixEuler rot, ixEuler result);

/*
 * Rotate vector from body frame to reference frame by euler angles, in order: phi, theta, psi
 */
void vectorBodyToReference(const  ixVector3 v, const ixEuler rot, ixVector3 result);

/*
 * Rotate vector from reference frame to body frame by euler angles, in order: psi, theta, phi
 */
void vectorReferenceToBody(const  ixVector3 v, const ixEuler rot, ixVector3 result);

/*
 * Vector to euler roll angle
 */
float vectorToRoll(const ixVector3 v);

/*
 * Vector to euler pitch angle
 */
float vectorToPitch(const ixVector3 v);

/*
 * Returns the pitch angle of the vector selected axis.
 */
float vectorSelectedAxisToPitch(const ixVector3 v, int pitchAxis);


#ifdef __cplusplus
}
#endif

#endif /* POSE_H_ */
