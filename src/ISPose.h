/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file ISPose.h
 * @brief Attitude/pose math: quaternion algebra, quaternion<->Euler<->DCM conversions, and
 * body<->reference frame vector/Euler rotations.
 *
 * Unless otherwise noted: quaternions are (W, X, Y, Z); Euler angles are (phi, theta, psi) =
 * (roll, pitch, yaw) in radians; DCMs (direction cosine matrices) rotate NED to body frame
 * (body = DCM * NED) in the standard [phi][theta][psi] rotation sequence.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef POSE_H_
#define POSE_H_

#include "ISMatrix.h"
#include "ISConstants.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (!defined (__cplusplus) && (!defined (inline)))
#       define inline __inline          //!< Allow the "inline" keyword to work in Windows w/ a C (non-C++) program
#endif


/**
 * @brief Initialize a quaternion to identity, q = [1, 0, 0, 0].
 * @param q Output: quaternion (W, X, Y, Z) to initialize.
 */
void quat_init(ixQuat q);

/**
 * @brief Quaternion conjugate: q* = [w, -x, -y, -z] of quaternion q = [w, x, y, z]. Rotation in the opposite direction.
 * @param result Output: conjugate of q.
 * @param q      Input quaternion (W, X, Y, Z).
 */
void quatConj(ixQuat result, const ixQuat q);

/**
 * @brief Product of two quaternions: result = q1 * q2. Order of q1 and q2 matters (same as applying
 * two successive DCMs)! Combines two quaternion rotations into one rotation.
 * Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
 * @param result Output: q1 * q2.
 * @param q1     First quaternion (W, X, Y, Z).
 * @param q2     Second quaternion (W, X, Y, Z).
 */
void mul_Quat_Quat(ixQuat result, const ixQuat q1, const ixQuat q2);

/**
 * @brief Product of two quaternions: result = quatConj(qc) * q2. Order matters (same as applying
 * two successive DCMs)! Combines two quaternion rotations into one rotation.
 * Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
 * @param result Output: quatConj(qc) * q2.
 * @param qc     Quaternion (W, X, Y, Z) to conjugate before multiplying.
 * @param q2     Second quaternion (W, X, Y, Z).
 */
void mul_ConjQuat_Quat(ixQuat result, const ixQuat qc, const ixQuat q2);

/**
 * @brief Product of two quaternions: result = q1 * quatConj(qc). Order matters (same as applying
 * two successive DCMs)! Combines two quaternion rotations into one rotation.
 * Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
 * @param result Output: q1 * quatConj(qc).
 * @param q1     First quaternion (W, X, Y, Z).
 * @param qc     Quaternion (W, X, Y, Z) to conjugate before multiplying.
 */
void mul_Quat_ConjQuat(ixQuat result, const ixQuat q1, const ixQuat qc);

/**
 * @brief Division of two quaternions: result = q1 / q2. Order matters!
 * Reference: http://www.mathworks.com/help/aeroblks/quaterniondivision.html
 * @param result Output: q1 / q2.
 * @param q1     Numerator quaternion (W, X, Y, Z).
 * @param q2     Denominator quaternion (W, X, Y, Z).
 */
void div_Quat_Quat(ixQuat result, const ixQuat q1, const ixQuat q2);

/**
 * @brief Compute the quaternion rotation from vector v1 to vector v2.
 * @param result Output: quaternion (W, X, Y, Z) rotating v1 onto v2.
 * @param v1     Source vector.
 * @param v2     Target vector.
 */
void quat_Vec3_Vec3(ixQuat result, const ixVector3 v1, const ixVector3 v2);

/**
 * @brief Computationally simple means to apply a quaternion rotation to a vector. Requires the
 * quaternion be normalized first. If the quaternion describes the current attitude, this rotates
 * body frame -> reference frame.
 * @param result Output: rotated vector.
 * @param q      Normalized quaternion (W, X, Y, Z) to rotate by.
 * @param v      Vector to rotate.
 */
void quatRot(ixVector3 result, const ixQuat q, const ixVector3 v);

/**
 * @brief Computationally simple means to apply a quaternion's conjugate (opposite) rotation to a
 * vector. Requires the quaternion be normalized first. If the quaternion describes the current
 * attitude, this rotates reference frame -> body frame.
 * @param result Output: rotated vector.
 * @param q      Normalized quaternion (W, X, Y, Z) whose conjugate rotation is applied.
 * @param v      Vector to rotate.
 */
void quatConjRot(ixVector3 result, const ixQuat q, const ixVector3 v);

/**
 * @brief Convert from quaternion to Euler angles: q(W,X,Y,Z) -> euler(phi,theta,psi), radians.
 * Reference: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * @param q     Input quaternion (W, X, Y, Z).
 * @param theta Output: Euler angles (phi, theta, psi), radians.
 */
void quat2euler(const ixQuat q, ixEuler theta);

/**
 * @brief Extract just the phi (roll) and theta (pitch) Euler angles from a quaternion.
 * @param q     Input quaternion (W, X, Y, Z).
 * @param phi   Output: phi (roll) angle, radians.
 * @param theta Output: theta (pitch) angle, radians.
 */
void quat2phiTheta(const ixQuat q, f_t *phi, f_t *theta);

/**
 * @brief Extract just the psi (yaw) Euler angle from a quaternion.
 * @param q   Input quaternion (W, X, Y, Z).
 * @param psi Output: psi (yaw) angle, radians.
 */
void quat2psi(const ixQuat q, f_t *psi);

/**
 * @brief Convert from Euler angles to a quaternion: euler(phi,theta,psi) (rad) -> q(W,X,Y,Z).
 * @param euler Input Euler angles (phi, theta, psi), radians.
 * @param q     Output: quaternion (W, X, Y, Z).
 */
void euler2quat(const ixEuler euler, ixQuat q);



/**
 * @brief Construct a 2x2 direction cosine matrix from the psi (yaw) angle: rotates from NE to body frame (body = m*NE).
 * @param psi Yaw angle, radians.
 * @param m   Output: 2x2 DCM.
 */
void psiDCM(const f_t psi, ixMatrix2 m);

/**
 * @brief Extract the psi Euler angle from a direction cosine matrix in the standard rotation
 * sequence, for either a 2x2 (body = m*NE) or 3x3 (body = m*NED) DCM.
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 * @param m Input DCM (2x2 or 3x3, row-major).
 * @return Psi (yaw) angle, radians.
 */
f_t DCMpsi(const f_t *m);

/**
 * @brief Construct a 3x3 direction cosine matrix from Euler angles in the standard rotation
 * sequence [phi][theta][psi] from NED to body frame (body = m*NED).
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 * @param euler Input Euler angles (phi, theta, psi), radians.
 * @param m     Output: 3x3 DCM.
 */
//const Matrix<3,3> eulerDCM(const Vector<3> & euler)
void eulerDCM(const ixEuler euler, ixMatrix3 m);

/**
 * @brief Same as @ref eulerDCM, but using only phi and theta (psi is excluded from the rotation).
 * @param euler Input Euler angles (phi, theta used; psi ignored), radians.
 * @param m     Output: 3x3 DCM.
 */
void phiThetaDCM(const ixEuler euler, ixMatrix3 m);

/**
 * @brief Construct the transpose of the direction cosine matrix from Euler angles in the standard
 * rotation sequence [phi][theta][psi] from NED to body frame (body = m*NED, before transposing).
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 * @param euler Input Euler angles (phi, theta, psi), radians.
 * @param m     Output: transpose of the 3x3 DCM.
 */
void eulerDCM_Trans(const ixEuler euler, ixMatrix3 m);

/**
 * @brief Extract Euler angles from a direction cosine matrix in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame (body = m*NED).
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 * @param m     Input 3x3 DCM.
 * @param euler Output: Euler angles (phi, theta, psi), radians.
 */
void DCMeuler(const ixMatrix3 m, ixEuler euler);


/**
 * @brief Construct a 3x3 direction cosine matrix from a quaternion, in the standard rotation
 * sequence [phi][theta][psi] from NED to body frame (body = mat*NED), single precision.
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 * @param q   Input quaternion (W, X, Y, Z).
 * @param mat Output: 3x3 DCM.
 */
void quatDCM(const ixQuat q, ixMatrix3 mat);

/**
 * @brief Same as @ref quatDCM, double-precision quaternion input.
 * @param q   Input quaternion (W, X, Y, Z), double precision.
 * @param mat Output: 3x3 DCM.
 */
void quatdDCM(const ixVector4d q, ixMatrix3 mat);

/**
 * @brief Construct a quaternion from a direction cosine matrix in the standard rotation sequence
 * [phi][theta][psi] from NED to body frame (body = mat*NED).
 * Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
 * @param mat Input 3x3 DCM.
 * @param q   Output: quaternion (W, X, Y, Z).
 */
void DCMquat(const ixMatrix3 mat, ixQuat q);

/**
 * @brief Construct the Euler omega-cross matrix wx(3,3) from body angular rates.
 * @param euler Body angular rates p, q, r (rad/sec), packed into an ixEuler-shaped vector.
 * @param mat   Output: 3x3 omega-cross matrix.
 */
void eulerWx(const ixEuler euler, ixMatrix3 mat);

/**
 * @brief Construct the quaternion omega matrix W(4,4) from body angular rates.
 * @param euler Body angular rates p, q, r (rad/sec), packed into an ixEuler-shaped vector.
 * @param mat   Output: 4x4 quaternion omega matrix.
 */
void quatW(const ixEuler euler, ixMatrix4 mat);

/**
 * @brief Convert a quaternion to a rotation axis (and implicitly, angle). Quaternion must be normalized.
 * @param q   Input normalized quaternion (W, X, Y, Z).
 * @param pqr Output: rotation axis vector, scaled by the rotation angle.
 */
void quatRotAxis(const ixQuat q, ixVector3 pqr);

/**
 * @brief Compute the derivative of the Euler angle psi with respect to the quaternion q. The
 * result is a row vector: d(psi)/d(q0), d(psi)/d(q1), d(psi)/d(q2), d(psi)/d(q3).
 * @param q  Input quaternion (W, X, Y, Z).
 * @param dq Output: derivative row vector, packed into an ixQuat-shaped array.
 */
void dpsi_dq(const ixQuat q, ixQuat dq);

/**
 * @brief Convert an NED vector to its equivalent ixEuler-shaped representation (no rotation; a repacking helper).
 * @param ned Input NED vector.
 * @param e   Output: same values, packed into an ixEuler-shaped array.
 */
void nedEuler(const ixVector3 ned, ixEuler e);

/**
 * @brief Convert an ixEuler-shaped vector to its equivalent NED representation (no rotation; a repacking helper).
 * @param e   Input, packed into an ixEuler-shaped array.
 * @param ned Output: same values as an NED vector.
 */
void eulerNed(const ixEuler e, ixVector3 ned);

/**
 * @brief Rotate Euler angles from body frame to reference frame by a rotation given as Euler angles, in order: phi, theta, psi.
 * @param e      Input Euler angles (body frame), radians.
 * @param rot    Rotation to apply, as Euler angles (phi, theta, psi), radians.
 * @param result Output: rotated Euler angles (reference frame), radians.
 */
void eulerBodyToReference(const  ixEuler e, const  ixEuler rot, ixEuler result);

/**
 * @brief Rotate Euler angles from reference frame to body frame by a rotation given as Euler angles, in order: psi, theta, phi.
 * @param e      Input Euler angles (reference frame), radians.
 * @param rot    Rotation to apply, as Euler angles (phi, theta, psi), radians.
 * @param result Output: rotated Euler angles (body frame), radians.
 */
void eulerReferenceToBody(const  ixEuler e, const ixEuler rot, ixEuler result);

/**
 * @brief Rotate a vector from body frame to reference frame by Euler angles, in order: phi, theta, psi.
 * @param v      Input vector (body frame).
 * @param rot    Rotation to apply, as Euler angles (phi, theta, psi), radians.
 * @param result Output: rotated vector (reference frame).
 */
void vectorBodyToReference(const  ixVector3 v, const ixEuler rot, ixVector3 result);

/**
 * @brief Rotate a vector from reference frame to body frame by Euler angles, in order: psi, theta, phi.
 * @param v      Input vector (reference frame).
 * @param rot    Rotation to apply, as Euler angles (phi, theta, psi), radians.
 * @param result Output: rotated vector (body frame).
 */
void vectorReferenceToBody(const  ixVector3 v, const ixEuler rot, ixVector3 result);

/**
 * @brief Compute the Euler roll angle implied by a vector's direction.
 * @param v Input vector.
 * @return Roll angle, radians.
 */
float vectorToRoll(const ixVector3 v);

/**
 * @brief Compute the Euler pitch angle implied by a vector's direction.
 * @param v Input vector.
 * @return Pitch angle, radians.
 */
float vectorToPitch(const ixVector3 v);

/**
 * @brief Compute the pitch angle of a vector about a selected axis.
 * @param v         Input vector.
 * @param pitchAxis Axis index (0=X, 1=Y, 2=Z) to compute the pitch angle about.
 * @return Pitch angle, radians.
 */
float vectorSelectedAxisToPitch(const ixVector3 v, int pitchAxis);

/**
 * Convert Azimuth and Elevation to a 3D vector
 * @param az Azimuth in radians
 * @param el Elevation in radians
 * @param vec Output vector
 */
void azelToVec3(double az, double el, ixVector3 vec);


#ifdef __cplusplus
}
#endif

#endif /* POSE_H_ */
