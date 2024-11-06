'''
Created 9/4/2013

Author: Walt Johnson
'''
from __future__ import print_function

import numpy as np
from numpy import sin, cos, tan, arccos, arcsin, arctan2, arctan, r_, c_, dot, pi


def quatInit():
    """ 
    Create a unit quaternion
    """
    q = np.zeros(4)
    q[0] = 1.0
    return q


def quatConj(q):
    """
    Quaternion Conjugate: q* = [ w, -x, -y, -z ] of quaterion q = [ w, x, y, z ]
    Describes rotation in opposite direction.
    """
    qc = np.empty_like(q)
    if len(np.shape(q)) == 1:
        qc[0] = q[0]
        qc[1:4] = -q[1:4]
    else:
        assert np.shape(q)[1] == 4, "Wrong shape of array of quaternions"
        qc[:,0] = q[:,0]
        qc[:,1:4] = -q[:,1:4]
    return qc


def mul_Quat_Quat(q1, q2):
    """
    Product of two Quaternions. 
    Order of q1 and q2 matters (same as applying two successive DCMs)!!!
    Concatenates two quaternion rotations into one.
    result = q1 * q2
    Order of rotation in rotation matrix notation: R(result) = R(q1) * R(q2)
    i.e. rotation by q2 followed by rotation by q1
    References:
    http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    http://physicsforgames.blogspot.com/2010/02/quaternions.html
    """
    if len(np.shape(q1)) == 1:
        q1 = np.expand_dims(q1, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(q2)) == 1:
        q2 = np.expand_dims(q2, axis=0)
    else:
        array = 1
    n1 = np.shape(q1)[0]
    n2 = np.shape(q2)[0]
    assert n1 == n2 or n1 == 1 or n2 == 1, "Number of quaternions in arrays do not match"

    result = np.empty((max(n1, n2), 4))
    result[:,0] = q1[:,0]*q2[:,0] - q1[:,1]*q2[:,1] - q1[:,2]*q2[:,2] - q1[:,3]*q2[:,3]
    result[:,1] = q1[:,0]*q2[:,1] + q1[:,1]*q2[:,0] - q1[:,2]*q2[:,3] + q1[:,3]*q2[:,2]
    result[:,2] = q1[:,0]*q2[:,2] + q1[:,1]*q2[:,3] + q1[:,2]*q2[:,0] - q1[:,3]*q2[:,1]
    result[:,3] = q1[:,0]*q2[:,3] - q1[:,1]*q2[:,2] + q1[:,2]*q2[:,1] + q1[:,3]*q2[:,0]

    if array == 0:
        result = np.squeeze(result)
    return result


def mul_ConjQuat_Quat(q1, q2):
    """
    Product of two Quaternions. 
    Order of q1 and q2 matters (same as applying two successive DCMs)!!!
    Combines two quaternion rotations into one rotation.
    result = quatConj(q1) * q2
    Order of rotation in rotation matrix notation: R(result) = R(q1).transpose() * R(q2)
    i.e. rotation by q2 followed by rotation by inverse of q1
    Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    """
    if len(np.shape(q1)) == 1:
        q1 = np.expand_dims(q1, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(q2)) == 1:
        q2 = np.expand_dims(q2, axis=0)
    else:
        array = 1
    n1 = np.shape(q1)[0]
    n2 = np.shape(q2)[0]
    assert n1 == n2 or n1 == 1 or n2 == 1, "Number of quaternions in arrays do not match"

    result = np.empty((max(n1, n2), 4))
    result[:,0] = q1[:,0]*q2[:,0] + q1[:,1]*q2[:,1] + q1[:,2]*q2[:,2] + q1[:,3]*q2[:,3]
    result[:,1] = q1[:,0]*q2[:,1] - q1[:,1]*q2[:,0] + q1[:,2]*q2[:,3] - q1[:,3]*q2[:,2]
    result[:,2] = q1[:,0]*q2[:,2] - q1[:,1]*q2[:,3] - q1[:,2]*q2[:,0] + q1[:,3]*q2[:,1]
    result[:,3] = q1[:,0]*q2[:,3] + q1[:,1]*q2[:,2] - q1[:,2]*q2[:,1] - q1[:,3]*q2[:,0]

    if array == 0:
        result = np.squeeze(result)
    return result


def mul_Quat_ConjQuat(q1, q2):
    """
    Product of two Quaternions. 
    Order of q1 and q2 matters (same as applying two successive DCMs)!!!
    Combines two quaternion rotations into one rotation.
    result = q1 * quatConj(q2)
    Order of rotation in rotation matrix notation: R(result) = R(q1) * R(q2).transpose()
    i.e. rotation by inverse of q2 followed by rotation by q1
    Reference: http://www.mathworks.com/help/aeroblks/quaternionmultiplication.html
    """
    if len(np.shape(q1)) == 1:
        q1 = np.expand_dims(q1, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(q2)) == 1:
        q2 = np.expand_dims(q2, axis=0)
    else:
        array = 1
    n1 = np.shape(q1)[0]
    n2 = np.shape(q2)[0]
    assert n1 == n2 or n1 == 1 or n2 == 1, "Number of quaternions in arrays do not match"

    result = np.empty((max(n1, n2), 4))
    result[:,0] =  q1[:,0]*q2[:,0] + q1[:,1]*q2[:,1] + q1[:,2]*q2[:,2] + q1[:,3]*q2[:,3]
    result[:,1] = -q1[:,0]*q2[:,1] + q1[:,1]*q2[:,0] + q1[:,2]*q2[:,3] - q1[:,3]*q2[:,2]
    result[:,2] = -q1[:,0]*q2[:,2] - q1[:,1]*q2[:,3] + q1[:,2]*q2[:,0] + q1[:,3]*q2[:,1]
    result[:,3] = -q1[:,0]*q2[:,3] + q1[:,1]*q2[:,2] - q1[:,2]*q2[:,1] + q1[:,3]*q2[:,0]

    if array == 0:
        result = np.squeeze(result)
    return result


def div_Quat_Quat(q1, q2):
    """
    Division of two Quaternions.  Order matters!!!
    result = q1 / q2.
    Reference: http://www.mathworks.com/help/aeroblks/quaterniondivision.html
    """
    if len(np.shape(q1)) == 1:
        q1 = np.expand_dims(q1, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(q2)) == 1:
        q2 = np.expand_dims(q2, axis=0)
    else:
        array = 1
    n1 = np.shape(q1)[0]
    n2 = np.shape(q2)[0]
    assert n1 == n2 or n1 == 1 or n2 == 1, "Number of quaternions in arrays do not match"

    result = np.empty((max(n1, n2), 4))
    d = 1.0 / np.einsum('ij,ij->i', q1, q1)
    result[:,0] = q1[:,0]*q2[:,0] + q1[:,1]*q2[:,1] + q1[:,2]*q2[:,2] + q1[:,3]*q2[:,3]
    result[:,1] = q1[:,0]*q2[:,1] - q1[:,1]*q2[:,0] - q1[:,2]*q2[:,3] + q1[:,3]*q2[:,2]
    result[:,2] = q1[:,0]*q2[:,2] + q1[:,1]*q2[:,3] - q1[:,2]*q2[:,0] - q1[:,3]*q2[:,1]
    result[:,3] = q1[:,0]*q2[:,3] - q1[:,1]*q2[:,2] + q1[:,2]*q2[:,1] - q1[:,3]*q2[:,0]
    result = result * d

    if array == 0:
        result = np.squeeze(result)
    return result


def quat_Vec3_Vec3(v1, v2):
    """
    Quaternion describing rotation from vector v1 to vector v2.
    Reference:
    """
    # Normalize input vectors
    w1 = normalize(v1)
    w2 = normalize(v2)

    qResult = np.zeros(4)
    qResult[1:4] = np.cross(w1, w2)
    qResult[0] = np.sqrt(np.square(np.dot(w1, w1))) + np.dot(w1, w2)

    # Normalize quaternion
    qResult = qResult / np.linalg.norm(qResult)
    return qResult


def norm(v, axis=None):
    """
    Compute norm of a single vector or each vector in an array
    """
    return np.sqrt(np.sum(v*v, axis=axis))


def normalize(v, axis=None):
    """
    Normalize a vector or each vector in an array
    """
    result = np.empty_like(v)
    if len(np.shape(v)) == 1:
        result = v / np.linalg.norm(v)
    else:
        vnorm = np.linalg.norm(v, axis=axis)
        vnorm = np.expand_dims(vnorm, axis=axis)
        result = v / vnorm
    return result


def quatRot(q, v):
    """
    Computationally simple means to apply quaternion rotation to a vector.
    Requires quaternion be normalized first.
    If quaternion describes current attitude, then rotation is body -> inertial frame.
    Equivalent to using rotation matrix: DCM(q).transpose * v
    """
    if len(np.shape(q)) == 1:
        q = np.expand_dims(q, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(v)) == 1:
        v = np.expand_dims(v, axis=0)
    else:
        array = 1
    n1 = np.shape(q)[0]
    n2 = np.shape(v)[0]
    assert n1 == n2 or n1 == 1 or n2 == 1, "Number of quaternions and vectors in arrays do not match"

    result = np.empty((max(n1, n2), 3))
    t = 2.0 * np.cross(q[:,1:4], v)
    result = v + (q[:,0] * t.T).T + np.cross(q[:,1:4], t)

    if array == 0:
        result = np.squeeze(result)
    return result


def quatConjRot(q, v):
    """
    Computationally simple means to apply quaternion conjugate (opposite) rotation to a vector
    (18 multiplies, 6 subtracts, 6 adds).  Using a DCM uses (27 multiplies, 12 adds, 6 subtracts).
    Requires quaternion be normalized first.
    If quaternion describes current attitude, then rotation is inertial -> body frame.
    Equivalent to using rotation matrix: DCM(q) * v
    """
    qc = quatConj(q)
    return quatRot(qc, v)


def quatNLerp(q1, q2, blend):
    """
    Find quaternion interpolation between two quaterions.  Blend must be 0 to 1.
    Reference:  http://physicsforgames.blogspot.com/2010/02/quaternions.html
    """
    if len(np.shape(q1)) == 1:
        q1 = np.expand_dims(q1, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(q2)) == 1:
        q2 = np.expand_dims(q2, axis=0)
    else:
        array = 1
    n1 = np.shape(q1)[0]
    n2 = np.shape(q2)[0]
    assert n1 == n2 or n1 == 1 or n2 == 1, "Number of quaternions in arrays do not match"

    result = np.empty((max(n1, n2), 4))
    dot_q1q2 = np.einsum('ij,ij->i', q1, q2)
    blendI = 1.0 - blend

    ind = dot_q1q2 >= 0.0
    result[~ind,:] = blendI * q1[~ind,:] - blend * q2[~ind,:]
    result[ind,:] = blendI * q1[ind,:] + blend * q2[ind,:]
    result = normalize(result, axis=1)

    if array == 0:
        result = np.squeeze(result)
    return result


def quat2euler(q):
    """
    Convert quaternion to Euler angles.
    Quaternions must have unit norm.
    q(4,1) -> euler[phi;theta;psi] (rad)
    Reference: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    """
    if len(np.shape(q)) == 1:
        q = np.expand_dims(q, axis=0)
        array = 0
    else:
        array = 1
    theta = np.empty(shape=(np.shape(q)[0], 3))
    theta[:, 0] = np.arctan2(2.0 * (q[:,0]*q[:,1] + q[:,2]*q[:,3]),
                             1.0 - 2.0 * (q[:,1]**2 + q[:,2]**2))
    theta[:, 1] = np.arcsin(2.0 * (q[:,0]*q[:,2] - q[:,3]*q[:,1]))
    theta[:, 2] = np.arctan2(2.0 * (q[:,0]*q[:,3] + q[:,1]*q[:,2]),
                             1.0 - 2.0 * (q[:,2]**2 + q[:,3]**2))
    if array == 0:
        theta = np.squeeze(theta)
    return theta


def euler2quat(euler):
    """
    Convert Euler angles to quaternion
    [phi, theta, psi] -> q(4,1)
    Euler angles in radians
    """
    if len(np.shape(euler)) == 1:
        euler = np.expand_dims(euler, axis=0)
        array = 0
    else:
        array = 1
    q = np.zeros((np.shape(euler)[0], 4))

    hphi = euler[:,0] * 0.5
    hthe = euler[:,1] * 0.5
    hpsi = euler[:,2] * 0.5

    shphi = np.sin(hphi)
    chphi = np.cos(hphi)
    shthe = np.sin(hthe)
    chthe = np.cos(hthe)
    shpsi = np.sin(hpsi)
    chpsi = np.cos(hpsi)

    q[:,0] = chphi * chthe * chpsi + shphi * shthe * shpsi
    q[:,1] = shphi * chthe * chpsi - chphi * shthe * shpsi
    q[:,2] = chphi * shthe * chpsi + shphi * chthe * shpsi
    q[:,3] = chphi * chthe * shpsi - shphi * shthe * chpsi

    if array == 0:
        q = np.squeeze(q)
    return q


def psiDCM(psi):
    """
    NE to heading/body frame.
    Construct a DCM from the psi angle (rotates from NE to body frame)
    body = tBL(2,2)*NE
    """
    cpsi = cos(psi)
    spsi = sin(psi)

    DCM = r_[
        c_[cpsi, spsi],
        c_[-spsi, cpsi],
    ]
    return DCM


def DCMpsi(A):
    """
    Extract the yaw Euler angle from a DCM in the
    standard rotation sequence, for either a 2x2 or 3x3 DCM matrix.
    [phi][theta][psi] from reference to body frame
    body = tBL(2,2)*NE
    body = tBL(3,3)*NED
    Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
    """
    psi = arctan2(A[0, 1], A[0, 0])
    return psi


def eulerDCM(euler):
    """
    Reference to body frame DCM (in the 1-2-3 roll-pitch-yaw order).
    Construct a DCM from Euler angles in the standard rotation sequence
    [phi][theta][psi] from reference to body frame
    body = tBL(3,3)*NED
    Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
    """
    if len(np.shape(euler)) == 1:
        euler = np.expand_dims(euler, axis=0)
        array = 0
    else:
        array = 1

    DCM = np.empty((np.shape(euler)[0], 3, 3))

    cphi = cos(euler[:,0])  # cos(phi)
    cthe = cos(euler[:,1])  # cos(theta)
    cpsi = cos(euler[:,2])  # cos(psi)

    sphi = sin(euler[:,0])  # sin(phi)
    sthe = sin(euler[:,1])  # sin(theta)
    spsi = sin(euler[:,2])  # sin(psi)

    DCM[:,0,0] =  cthe * cpsi
    DCM[:,0,1] =  cthe * spsi
    DCM[:,0,2] = -sthe
    DCM[:,1,0] = -cphi * spsi + sphi * sthe * cpsi
    DCM[:,1,1] =  cphi * cpsi + sphi * sthe * spsi
    DCM[:,1,2] =  sphi * cthe
    DCM[:,2,0] =  sphi * spsi + cphi * sthe * cpsi
    DCM[:,2,1] = -sphi * cpsi + cphi * sthe * spsi
    DCM[:,2,2] =  cphi * cthe

    if array == 0:
        DCM = np.squeeze(DCM)
    return DCM


def DCMeuler(mat):
    """
    Extract Euler angles from a DCM in the standard rotation sequence.
    [phi][theta][psi] from reference to body frame
    body = tBL(3,3)*NED
    Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
    """
    if len(np.shape(mat)) == 2:
        mat = np.expand_dims(mat, axis=0)
        array = 0
    else:
        array = 1

    eul = np.empty(shape=(np.shape(mat)[0], 3))

    eul[:,0] = arctan2(mat[:,1,2], mat[:,2,2])
    eul[:,1] = arcsin(-mat[:,0,2])
    eul[:,2] = arctan2(mat[:,0,1], mat[:,0,0])

    if array == 0:
        eul = np.squeeze(eul)
    return eul


def DCMeulerToPsi(A, phi, the):
    psi = arctan2(A[0, 1]/cos(the), A[0, 0]/cos(the))
    return r_[phi, the, psi]


def quatDCM(q):
    """
    Construct a DCM from quaternions in the standard rotation sequence
    [phi][theta][psi] from reference to body frame.
    body = tBL(3,3)*NED
    Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
    """
    if len(np.shape(q)) == 1:
        q = np.expand_dims(q, axis=0)
        array = 0
    else:
        array = 1

    DCM = np.empty(shape=(np.shape(q)[0], 3, 3))

    DCM[:,0,0] = 1.0 - 2.0 * (q[:,2]**2 + q[:,3]**2)
    DCM[:,0,1] = 2.0 * (q[:,1]*q[:,2] + q[:,0]*q[:,3])
    DCM[:,0,2] = 2.0 * (q[:,1]*q[:,3] - q[:,0]*q[:,2])
    DCM[:,1,0] = 2.0 * (q[:,1]*q[:,2] - q[:,0]*q[:,3])
    DCM[:,1,1] = 1.0 - 2.0 * (q[:,1]**2 + q[:,3]**2)
    DCM[:,1,2] = 2.0 * (q[:,2]*q[:,3] + q[:,0]*q[:,1])
    DCM[:,2,0] = 2.0 * (q[:,1]*q[:,3] + q[:,0]*q[:,2])
    DCM[:,2,1] = 2.0 * (q[:,2]*q[:,3] - q[:,0]*q[:,1])
    DCM[:,2,2] = 1.0 - 2.0 * (q[:,1]**2 + q[:,2]**2)

    if array == 0:
        DCM = np.squeeze(DCM)
    return DCM


def DCMquat(mat):
    """
    Construct quaternion from a DCM in the standard rotation sequence
    [phi][theta][psi] from reference to body frame.
    body = tBL(3,3)*NED
    Reference: http://en.wikipedia.org/wiki/Rotation_representation_%28mathematics%29
    """
    if len(np.shape(mat)) == 2:
        mat = np.expand_dims(mat, axis=0)
        array = 0
    else:
        array = 1

    q = np.empty(shape=(np.shape(mat)[0], 4))
    q_sq4 = np.empty_like(q)

    q_sq4[:,0] = 1.0 + mat[:,0,0] + mat[:,1,1] + mat[:,2,2]
    q_sq4[:,1] = 1.0 + mat[:,0,0] - mat[:,1,1] - mat[:,2,2]
    q_sq4[:,2] = 1.0 - mat[:,0,0] + mat[:,1,1] - mat[:,2,2]
    q_sq4[:,3] = 1.0 - mat[:,0,0] - mat[:,1,1] + mat[:,2,2]

    for i in range(0, np.shape(mat)[0]):
        ind = np.argmax(q_sq4[i, :])
        q[i,ind] = 0.5 * np.sqrt(q_sq4[i,ind])
        d = 0.25 / q[i,ind]
        if ind == 0:
            q[i,1] = d * (mat[i,1,2] - mat[i,2,1])
            q[i,2] = d * (mat[i,2,0] - mat[i,0,2])
            q[i,3] = d * (mat[i,0,1] - mat[i,1,0])
        elif ind == 1:
            q[i,0] = d * (mat[i,1,2] - mat[i,2,1])
            q[i,2] = d * (mat[i,1,0] + mat[i,0,1])
            q[i,3] = d * (mat[i,0,2] + mat[i,2,0])
        elif ind == 2:
            q[i,0] = d * (mat[i,2,0] - mat[i,0,2])
            q[i,1] = d * (mat[i,1,0] + mat[i,0,1])
            q[i,3] = d * (mat[i,2,1] + mat[i,1,2])
        else:
            q[i,0] = d * (mat[i,0,1] - mat[i,1,0])
            q[i,1] = d * (mat[i,0,2] + mat[i,2,0])
            q[i,2] = d * (mat[i,2,1] + mat[i,1,2])

    ind = q[:,0] < 0
    q[ind, :] = -q[ind, :]

    if array == 0:
        q = np.squeeze(q)
    return q


def eulerWx(x):
    """
    Construct the cross-product matrix Wx(3,3)
    such that cross(x,y) = Wx * y
    """
    if len(np.shape(x)) == 1:
        x = np.expand_dims(x, axis=0)
        array = 0
    else:
        array = 1

    mat = np.empty(shape=(np.shape(x)[0], 3, 3))

    # Row 1
    mat[:,0,0] =  0.0
    mat[:,0,1] = -x[:,2]
    mat[:,0,2] =  x[:,1]
    # Row 2
    mat[:,1,0] =  x[:,2]
    mat[:,1,1] =  0.0
    mat[:,1,2] = -x[:,0]
    # Row 3
    mat[:,2,0] = -x[:,1]
    mat[:,2,1] =  x[:,0]
    mat[:,2,2] =  0.0

    if array == 0:
        mat = np.squeeze(mat)
    return mat


def quatW(omega):
    """
    Construct the quaternion 4x4 skew-symmetric matrix W(4,4)
    that describes the quaternion evolution due 
    to the angular rate vector omega, i.e.
    dq/dt = 0.5*W*q
    """
    if len(np.shape(omega)) == 1:
        omega = np.expand_dims(omega, axis=0)
        array = 0
    else:
        array = 1

    mat = np.empty(shape=(np.shape(omega)[0], 4, 4))
    p = omega[:,0] * 0.5
    q = omega[:,1] * 0.5
    r = omega[:,2] * 0.5

    # Row 1
    mat[:,0,0] =  0.0
    mat[:,0,1] = -p
    mat[:,0,2] = -q
    mat[:,0,3] = -r
    # Row 2
    mat[:,1,0] =  p
    mat[:,1,1] =  0.0
    mat[:,1,2] =  r
    mat[:,1,3] = -q
    # Row 3
    mat[:,2,0] =  q
    mat[:,2,1] = -r
    mat[:,2,2] =  0.0
    mat[:,2,3] =  p
    # Row 4
    mat[:,3,0] =  r
    mat[:,3,1] =  q
    mat[:,3,2] = -p
    mat[:,3,3] =  0.0

    if array == 0:
        mat = np.squeeze(mat)
    return mat


def quatRotAxis(q):
    """
    Extract rotation axis from a quaternion
    """
    if len(np.shape(q)) == 1:
        q = np.expand_dims(q, axis=0)
        array = 0
    else:
        array = 1

    axis = normalize(q[:,1:4], axis=1)

    if array == 0:
        axis = np.squeeze(axis)
    return axis


def quatRotVec(q):
    """
    Convert quaternion to a rotation vector
    """
    if len(np.shape(q)) == 1:
        q = np.expand_dims(q, axis=0)
        array = 0
    else:
        array = 1

    rv = np.zeros(shape=(np.shape(q)[0], 3))
    ind = q[:,0] < 0
    q[ind,:] = -q[ind,:]

    q0 = np.clip(q[:,0], -1.0, 1.0)
    theta = np.arccos(q0) * 2.0
    sin_half_theta = np.sqrt(1.0 - q0**2)
    ind = theta > 1e-6
    rv[ind,:] = q[ind,1:4] * (theta[ind] / sin_half_theta[ind])[:, None]
    rv[~ind,:] = q[~ind,1:4] * 2.0

    if array == 0:
        rv = np.squeeze(rv)
    return rv


def rotVecQuat(v):
    """
    Convert rotation vector to a quaternion
    """
    if len(np.shape(v)) == 1:
        v = np.expand_dims(v, axis=0)
        array = 0
    else:
        array = 1

    q = np.zeros(shape=(np.shape(v)[0], 4))

    theta = norm(v, axis=1)
    half_theta = 0.5 * theta
    ind = half_theta > 1e-6

    q[~ind,0] = 1.0
    q[~ind,1:] = v[~ind,:]

    q[ind] = np.cos(half_theta[ind,None])
    q[ind,1:] = v[ind,:] * (np.sin(half_theta[ind]) / theta[ind])[:,None]

    # Enforce unit norm
    q = normalize(q, axis=1)

    if array == 0:
        q = np.squeeze(q)
    return q


def dpsi_dq(q):
    """
    Compute the derivative of the yaw Euler angle (psi) w.r.t.
    to the quaternion q. The result is a row vector
    d(psi)/d(q0)
    d(psi)/d(q1)
    d(psi)/d(q2)
    d(psi)/d(q3)
    """
    if len(np.shape(q)) == 1:
        q = np.expand_dims(q, axis=0)
        array = 0
    else:
        array = 1
    dq = np.zeros(shape=(np.shape(q)[0], 4))

    t1 = 1.0 - 2.0 * (q[:,2]**2 + q[:,3]*q[:,2])
    t2 = 2.0 * (q[:,1]*q[:,2] + q[:,0]*q[:,3])
    err = 2.0 / (t1**2 + t2**2)

    dq[:,0] = err * (q[:,3]*t1)
    dq[:,1] = err * (q[:,2]*t1)
    dq[:,2] = err * (q[:,1]*t1 + 2.0 * q[:,2]*t2)
    dq[:,3] = err * (q[:,0]*t1 + 2.0 * q[:,3]*t2)

    if array == 0:
        dq = np.squeeze(dq)
    return dq


def lla2ecef(lla_deg):
    """
    Find Earth Centered Earth Fixed coordinate from LLA
    lla[0] = latitude (decimal degree)
    lla[1] = longitude (decimal degree)
    lla[2] = msl altitude (m)
    """
    # Earth first eccentricity: e = sqrt((R^2-b^2)/R^2);
    e = 0.08181919084262
     # Earth equatorial and polar radii (from flattening, f = 1/298.257223563;
    R = 6378137.0         # m
    # Earth polar radius b = R * (1-f)
    b = 6356752.31424518

    if len(np.shape(lla_deg)) == 1:
        lla_deg = np.expand_dims(lla_deg, axis=0)
        array = 0
    else:
        array = 1
    LLA = np.copy(lla_deg)
    LLA[:,0:2] = np.radians(LLA[:,0:2])

    Smu = sin(LLA[:,0])
    Cmu = cos(LLA[:,0])
    Sl  = sin(LLA[:,1])
    Cl  = cos(LLA[:,1])

    # Radius of curvature at a surface point:
    Rn = R / np.sqrt(1 - e**2 * Smu**2)

    Pe = np.empty(np.shape(LLA))
    Pe[:,0] = (Rn + LLA[:,2]) * Cmu * Cl
    Pe[:,1] = (Rn + LLA[:,2]) * Cmu * Sl
    Pe[:,2] = (Rn * (b/R)**2 + LLA[:,2]) * Smu

    if array == 0:
        Pe = np.squeeze(Pe)
    return Pe


def ecef2lla(Pe, Niter=5):
    """
    Coordinate transformation from ECEF coordinates to 
    latitude/longitude/altitude (deg,deg,m)
    """
    # Earth equatorial radius
    R = 6378137.0
    # Earth polar radius b = R * (1-f)
    b = 6356752.31424518
    # Earth first eccentricity
    e = 0.08181919084262    # e = sqrt((R^2-b^2)/R^2);
    e2 = e**2
    # Earth flattening
    f = 0.0033528106647474805

    if len(np.shape(Pe)) == 1:
        Pe = np.expand_dims(Pe, axis=0)
        array = 0
    else:
        array = 1

    LLA = np.empty(np.shape(Pe))
    LLA[:,1] = arctan2(Pe[:,1], Pe[:,0]) # Longitude

    # Latitude computation using Bowring's method
    s = np.sqrt(Pe[:,0]**2 + Pe[:,1]**2)
    beta = arctan2(Pe[:,2], (1.0-f)*s)  # reduced latitude, initial guess

    B = e2 * R
    A = e2 * R / (1.0 - f)
    for i in range(0, Niter):
        # iterative latitude computation
        LLA[:,0] = arctan2(Pe[:, 2]+A*sin(beta)**3, s-B*cos(beta)**3)
        beta = arctan((1-f) * tan(LLA[:,0]))

    # Radius of curvature in the vertical prime
    sin_lat = sin(LLA[:,0])
    Rn = R / np.sqrt(1.0 - e2 * sin_lat**2)

    # Altitude above planetary ellipsoid
    LLA[:,2] = s * cos(LLA[:,0]) + (Pe[:,2] + e2 * Rn * sin_lat) * sin_lat - Rn

    # Convert to degrees
    LLA[:,0:2] = np.degrees(LLA[:,0:2])

    if array == 0:
        LLA = np.squeeze(LLA)
    return LLA


def lla2ned(lla_ref_deg, lla_deg):
    """
    Find NED (north, east, down) offset from lla_ref_deg to lla_deg
    lla_ref_deg[0] = reference latitude (decimal degree)
    lla_ref_deg[1] = reference longitude (decimal degree)
    lla_ref_deg[2] = reference msl altitude (m)
    lla_deg[0] = latitude (decimal degree)
    lla_deg[1] = longitude (decimal degree)
    lla_deg[2] = msl altitude (m)
    """
    # Earth equatorial radius
    R = 6378137.0
    # Earth first eccentricity
    e = 0.08181919084262    # e = sqrt((R^2-b^2)/R^2);
    e2 = e**2

    if len(np.shape(lla_deg)) == 1:
        lla_deg = np.expand_dims(lla_deg, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(lla_ref_deg)) == 1:
        lla_ref_deg = np.expand_dims(lla_ref_deg, axis=0)

    lla = np.copy(lla_deg)
    lla_ref = np.copy(lla_ref_deg)
    # Convert deg to rad
    lla[:,0:2] = np.radians(lla[:,0:2])
    lla_ref[:,0:2] = np.radians(lla_ref_deg[:,0:2])

    deltaLLA = lla - lla_ref
    # radius of curvature in the prime vertical:
    sin_lat_ref = sin(lla_ref[:, 0])
    Rn = R / np.sqrt(1.0 - e2 * sin_lat_ref**2)
    # radius of curvature in the meridian
    Rm = Rn * (1.0 - e2) / (1.0 - e2 * sin_lat_ref**2)

    Pn = np.empty(np.shape(lla))
    Pn[:,0] =  deltaLLA[:,0] * Rm
    Pn[:,1] =  deltaLLA[:,1] * Rn * cos(lla_ref[:, 0])
    Pn[:,2] = -deltaLLA[:,2]

    if array == 0:
        Pn = np.squeeze(Pn)
    return Pn


def ned2lla(lla_ref_deg, Pn):
    """
    Find position in terms of LLA (degrees, m) given 
    NED (north, east, down) coordinates and NED origin at lla_ref_deg.
    lla_ref_deg[0] = reference latitude (decimal degree)
    lla_ref_deg[1] = reference longitude (decimal degree)
    lla_ref_deg[2] = reference msl altitude (m)
    Pn[0] = position North (m)
    Pn[1] = position East (m)
    Pn[2] = position Down (m)
    """
    # Earth equatorial radius
    R = 6378137.0
    # Earth first eccentricity
    e = 0.08181919084262    # e = sqrt((R^2-b^2)/R^2);
    e2 = e**2

    if len(np.shape(Pn)) == 1:
        Pn = np.expand_dims(Pn, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(lla_ref_deg)) == 1:
        lla_ref_deg = np.expand_dims(lla_ref_deg, axis=0)

    # Convert deg to rad
    lla_ref = np.copy(lla_ref_deg)
    lla_ref[:,0:2] = np.radians(lla_ref_deg[:,0:2])

    # radius of curvature in the prime vertical:
    sin_lat_ref = sin(lla_ref[:, 0])
    Rn = R / np.sqrt(1.0 - e2 * sin_lat_ref**2)
    # radius of curvature in the meridian
    Rm = Rn * (1.0 - e2) / (1.0 - e2 * sin_lat_ref**2)

    lla = np.empty(np.shape(Pn))
    lla[:,0] = lla_ref[:,0] + Pn[:,0] / Rm
    lla[:,1] = lla_ref[:,1] + Pn[:,1] / (Rn * cos(lla_ref[:,0]))
    lla[:,2] = lla_ref[:,2] - Pn[:,2]

    # Convert lat/lon to degrees
    lla[:,0:2] = np.degrees(lla[:,0:2])
    if array == 0:
        lla = np.squeeze(lla)
    return lla


def ned2DeltaLla(lla_ref_deg, Pn):
    """
    Find LLA offset from the reference LLA given 
    NED (north, east, down) coordinates with NED origin at the reference LLA
    lla_ref_deg[0] = reference latitude (decimal degree)
    lla_ref_deg[1] = reference longitude (decimal degree)
    lla_ref_deg[2] = reference msl altitude (m)
    Pn[0] = position North (m)
    Pn[1] = position East (m)
    Pn[2] = position Down (m)
    """
    # Earth equatorial radius
    R = 6378137.0
    # Earth first eccentricity
    e = 0.08181919084262    # e = sqrt((R^2-b^2)/R^2);
    e2 = e**2

    if len(np.shape(Pn)) == 1:
        Pn = np.expand_dims(Pn, axis=0)
        array = 0
    else:
        array = 1
    if len(np.shape(lla_ref_deg)) == 1:
        lla_ref_deg = np.expand_dims(lla_ref_deg, axis=0)

    # Convert deg to rad
    lla_ref = np.copy(lla_ref_deg)
    lla_ref[:, 0:2] = np.radians(lla_ref_deg[:, 0:2])

    # radius of curvature in the prime vertical:
    sin_lat_ref = sin(lla_ref[:, 0])
    Rn = R / np.sqrt(1.0 - e2 * sin_lat_ref**2)
    # radius of curvature in the meridian
    Rm = Rn * (1.0 - e2) / (1.0 - e2 * sin_lat_ref**2)

    deltaLLA = np.empty(np.shape(Pn))
    deltaLLA[:,0] =  Pn[:,0] / Rm
    deltaLLA[:,1] =  Pn[:,1] / (Rn * cos(lla_ref[:, 0]))
    deltaLLA[:,2] = -Pn[:,2]

    # Convert delta lat/lon to degrees
    deltaLLA[:, 0:2] = np.degrees(deltaLLA[:, 0:2])
    if array == 0:
        deltaLLA = np.squeeze(deltaLLA)
    return deltaLLA


def quat_ecef2ned(latlon):
    """
    Compute quaternion that described rotation from
    ECEF to NED at given latitude/longitude
    """
    if len(np.shape(latlon)) == 1:
        latlon = np.expand_dims(latlon, axis=0)
        array = 0
    else:
        array = 1

    eul = np.empty(shape=(np.shape(latlon)[0], 3))
    eul[:,0] =  0.0;
    eul[:,1] = -latlon[:,0] - 0.5 * pi
    eul[:,2] =  latlon[:,1]
    qe2n = euler2quat(eul)

    if array == 0:
        qe2n = np.squeeze(qe2n)
    return qe2n


def rotmat_ned2ecef(latlon):
    """
    Compute rotation matrix (DCM) from NED to ECEF at given latitude/longitude
    """
    if len(np.shape(latlon)) == 1:
        latlon = np.expand_dims(latlon, axis=0)
        array = 0
    else:
        array = 1

    Smu = sin(latlon[:,0])
    Cmu = cos(latlon[:,0])
    Sl  = sin(latlon[:,1])
    Cl  = cos(latlon[:,1])

    R = np.empty(shape=(np.shape(latlon)[0], 3, 3))
    R[:,0,0] = -Smu * Cl
    R[:,0,1] = -Sl
    R[:,0,2] = -Cmu * Cl
    R[:,1,0] = -Smu * Sl
    R[:,1,1] =  Cl
    R[:,1,2] = -Cmu * Sl
    R[:,2,0] =  Cmu
    R[:,2,1] =  0.0
    R[:,2,2] = -Smu

    if array == 0:
        R = np.squeeze(R)
    return R


def rotmat_ecef2ned(latlon):
    """
    Compute rotation matrix (DCM) from ECEF to NED at given latitude/longitude
    """
    if len(np.shape(latlon)) == 1:
        latlon = np.expand_dims(latlon, axis=0)
        array = 0
    else:
        array = 1

    Smu = sin(latlon[:,0])
    Cmu = cos(latlon[:,0])
    Sl  = sin(latlon[:,1])
    Cl  = cos(latlon[:,1])

    R = np.empty(shape=(np.shape(latlon)[0], 3, 3))
    R[:,0,0] = -Smu * Cl
    R[:,0,1] = -Smu * Sl
    R[:,0,2] =  Cmu
    R[:,1,0] = -Sl
    R[:,1,1] =  Cl
    R[:,1,2] =  0.0
    R[:,2,0] = -Cmu * Cl
    R[:,2,1] = -Cmu * Sl
    R[:,2,2] = -Smu

    if array == 0:
        R = np.squeeze(R)
    return R


def enu2nedEuler(eul_e2b):
    """
    Convert body attitude in Euler angles relative to ENU frame
    to body attitude in Euler angles relative to NED frame
    """
    q_n2e = euler2quat(np.array([pi, 0.0, 0.5*pi]))
    q_e2b = euler2quat(eul_e2b)
    q_n2b = mul_Quat_Quat(q_e2b, q_n2e)
    eul_n2b = quat2euler(q_n2b)
    return eul_n2b


def nedEuler(ned):
    """
    Compute body atitude as Euler angles (pitch and heading with zero roll)
    given body X unit vector NED cordinates
    """
    if len(np.shape(ned)) == 1:
        ned = np.expand_dims(ned, axis=0)
        array = 0
    else:
        array = 1

    euler = np.empty(shape=(np.shape(ned)[0], 3))
    euler[:,2] = arctan2(ned[:,1], ned[:,0])
    euler[:,1] = arctan2(-ned[:,2], np.sqrt(ned[:,0]**2 + ned[:,1]**2))
    euler[:,0] = 0.0

    if array == 0:
        euler = np.squeeze(euler)
    return euler


def eulerNed(euler):
    """
    Compute coordinates of a body X unit vector in NED
    given body attitude in NED as Euler angles
    """
    if len(np.shape(euler)) == 1:
        euler = np.expand_dims(euler, axis=0)
        array = 0
    else:
        array = 1
    e0 = np.array([1.0, 0.0, 0.0])
    q  = euler2quat(euler)
    e0n = quatRot(q, e0)
    if array == 0:
        e0n = np.squeeze(e0n)
    return e0n


def nedQuat(v):
    """
    Compute body atitude quaternion assuming zero roll and
    given body X unit vector NED cordinates
    """
    return euler2quat(nedEuler(v))


def eulerRotateBodyToInertial(e, rot):
    """
    Compute Euler angles describing attitude after two rotations:
    first represented by Euler angles (e) followed by the second
    represented by Euler angles (rot).
    Equivalent to: R(eul) = R(rot) * R(e)
    """
    qe = euler2quat(e)
    qr = euler2quat(rot)
    q = mul_Quat_Quat(qr, qe)
    eul = quat2euler(q)
    return eul


def eulerRotateInertialToBody(e, rot):
    """
    Compute Euler angles describing attitude after two rotations:
    first represented by Euler angles (e) followed by the inverse of
    the second represented by Euler angles (rot).
    Equivalent to: R(eul) = R(rot).transpose() * R(e)
    """
    qe = euler2quat(e)
    qr = euler2quat(rot)
    q = mul_ConjQuat_Quat(qr, qe)
    eul = quat2euler(q)
    return eul


def vectorRotateInertialToBody(euler, v_in):
    """
    Rotate vector from inertial to body frame by euler angles, in order: psi, theta, phi
    Equivalent to: DCM * vector
    Similar to quatConjRot() that does the same but uses quaternions.
    """
    q  = euler2quat(euler)
    v_out = quatConjRot(q, v_in)
    return v_out


def vectorRotateBodyToInertial(euler, v_in):
    """
    Rotate vector from body to inertial frame by euler angles, in order: phi, theta, psi
    Equivalent to: DCM.T() * vector
    Similar to quatConjRot() that does the same but uses quaternions.
    """
    q  = euler2quat(euler)
    v_out = quatRot(q, v_in)
    return v_out


def unwrapAngle(angle):
    """
    Constrain an angle to be within [-pi, pi]
    """
    if len(np.shape(angle)) < 1:
        angle = np.expand_dims(angle, axis=0)
        array = 0
    else:
        array = 1

    twoPi = pi*2
    for i in range(0, np.shape(angle)[0]):
        while angle[i] < -pi:
            angle[i] += twoPi
        while angle[i] > pi:
            angle[i] -= twoPi

    if array == 0:
        angle = np.squeeze(angle)
    return angle


def accellToEuler(acc):
    """
    Find body attitude as Euler angles (roll and pitch, yaw is kept zero)
    given coordinates of the body Z-axis vector in reference rame
    Application: non-accelerated gravity used to determine attitude
    """
    if len(np.shape(acc)) == 1:
        acc = np.expand_dims(acc, axis=0)
        array = 0
    else:
        array = 1

    euler = np.empty(shape=(np.shape(acc)[0], 3))
    euler[:,0] = np.arctan2(-acc[:,1], -acc[:,2])
    euler[:,1] = np.arctan2(acc[:,0], np.sqrt(acc[:,1]**2 + acc[:,2]**2))

    if array == 0:
        euler = np.squeeze(euler)
    return euler


def acc2AttAndBias(acc):
    """
    Find body attitude as Euler angles (roll and pitch, yaw is kept zero)
    and accelerometer bias given non-accelerated gravity
    """
    if len(np.shape(acc)) == 1:
        acc = np.expand_dims(acc, axis=0)
        array = 0
    else:
        array = 1

    bias = np.zeros(np.shape(acc))
    for i in range(0, 4):
        att = accellToEuler(acc-bias)
        gIF = np.r_[0, 0, -9.80665]
        DCM = eulerDCM(att)
        g = np.einsum('ijk,k->ij', DCM, gIF)
        bias = acc - g

    if array == 0:
        att = np.squeeze(att)
        bias = np.squeeze(bias)

    return [att, bias]


def qlog(q):
    """
    Compute logarithm of a unit quaternion (unit norm is important here).
    Let q = [a, qv], where a is the scalar part and qv is the vector part.
    qv = sin(phi/2)*nv, where nv is a unit vector. Then
    ln(q) = ln(||q||) + qv / ||qv|| * arccos(a / ||q||)
    Therefore for a unit quaternion, the scalar part of ln(q) is zero
    and the vector part of ln(q) is 1/2 * phi * nv,
    i.e. half of rotation vector rv = phi * nv because 
    a = cos(phi/2) in attitude quaternion (see quatRotVec())
    Reference: https://en.wikipedia.org/wiki/Quaternion
    NOTE 1: due to existing implementation in C++, this function
    returns just the vector part of ln(q)
    NOTE 2: According to Wiki description, ln(q)_v should be a 
    half of rotation vector. However the previous 
    implementation computed the full rotation vector. 
    So, using the rotation vector for now until cleared up.
    """
    rv = quatRotVec(q)
    return rv


def qexp(v):
    """
    Compute exponent of a unit quaternion
    Let q = [a, qv], where a is the scalar part and qv is the vector part.
    qv = sin(phi/2)*nv, where nv is a unit vector. Then
    exp(q) = exp(a) * (cos(||q_v||) + qv / ||qv|| * sin(||q_v||))
    Reference: https://en.wikipedia.org/wiki/Quaternion
    NOTE: due to existing implementation in C++, 
    this function takes just the vector part as its argument to act as
    inverse of qlog()
    """
    q = rotVecQuat(v)
    return q


def qboxplus(q, v):
    """
    Attitude quaternion resulting from q1 followed by rotation due to rotation vector v
    """
    return mul_Quat_Quat(qexp(v), q)


def qboxminus(q1, q2):
    """
    Rotation from attitude q1 to q2 in terms of rotation vector
    """
    return qlog(mul_Quat_Quat(q1, quatConj(q2)))


def meanOfQuat(q):
    """
    Implementation of "Mean of Sigma Points" from Integrating Generic Sensor Fusion Algorithms with
    Sound State Representations through Encapsulation of Manifolds by Hertzberg et. al.
    Reference: https://arxiv.org/pdf/1107.1119.pdf p.13
    """
    n = float(q.shape[0])
    mu = q[None, 0, :]
    prev_mu = None
    iter = 0
    while prev_mu is None or norm(qboxminus(mu, prev_mu)) > 1e-3:
        iter += 1
        prev_mu = mu
        mu = qboxplus(mu, np.sum(qboxminus(q, mu), axis=0)[None, :]/n)
    assert np.abs(1.0 - norm(mu)) <= 1e-3
    return mu


def meanOfQuatArray(q):
    assert q.shape[2] == 4
    mu = np.empty((q.shape[0], 4))
    for i in range(q.shape[0]):
        mu[None, i, :] = meanOfQuat(q[i, :, :])
    return mu


# def salemUtLla():
#     return np.r_[ 40.0557114, -111.6585476, 1426.77 ]    # // (deg,deg,m) Lat,Lon,Height above sea level (not HAE, height above ellipsoid)
#
# def salemUtMagDecInc():
#     return np.r_[ 0.20303997, 1.141736219 ]    # (rad) Declination: 11.6333333 deg or 11 deg 38', Inclination: -65.4166667 deg or 65 deg 25'

if __name__ == '__main__':
    q = np.random.random((5000, 4))
    q = normalize(q, axis=1)
    y = np.random.random((5000, 3))

    q1 = q[0:2, :]
    q2 = q[2:4, :]
    q3 = mul_Quat_Quat(q2, q1)
    q0 = q[0, :]

    qq = quatNLerp(q1, q2, 0.5)

    # Test quat-to-DCM conversions
    R = quatDCM(q1)
    quat = DCMquat(R)
    assert np.sqrt(np.sum(np.square(q1 - quat))) < 1e-8

    # Test cross-product matrix,
    # see https://ajcr.net/Basic-guide-to-einsum/ for einsum() usage
    u = np.cross(y[0:2, :], y[1:3, :])
    # v = np.einsum('...ij,...j', eulerWx(y[0:2,:]), y[1:3,:])
    v = np.einsum('ijk,ik->ij', eulerWx(y[0:2, :]), y[1:3, :])
    assert np.sqrt(np.sum(np.square(u - v))) < 1e-8

    # Test quaternion to rotation vector conversion
    a = quatRotAxis(q)
    rv = quatRotVec(q)
    rv1 = qlog(q)
    assert np.sqrt(np.sum(np.square(rv - rv1))) < 1e-8
    quat = rotVecQuat(rv)
    assert np.sqrt(np.sum(np.square(q - quat))) < 1e-8

    # Test quat-to-Euler conversions
    eul = quat2euler(q)
    quat = euler2quat(eul)
    assert np.sqrt(np.sum(np.square(q - quat))) < 1e-8

    # Euler angles from sequential rotations
    eul1 = eulerRotateBodyToInertial(eul, eul[0,:])
    eul2 = eulerRotateInertialToBody(eul, eul[0,:])

    # Test quaternion multiplication with inverse
    quat0 = mul_Quat_Quat(quatConj(q0), q0)
    quat0 = mul_Quat_Quat(quatConj(q1), q2)
    quat1 = mul_ConjQuat_Quat(q1, q2)
    assert np.sqrt(np.sum(np.square(quat0 - quat1))) < 1e-8
    quat0 = mul_Quat_Quat(q1, quatConj(q2))
    quat1 = mul_Quat_ConjQuat(q1, q2)
    assert np.sqrt(np.sum(np.square(quat0 - quat1))) < 1e-8
    print("q2 =", q2)
    print("q3 = q2*q1; q3*inv(q1) = ", mul_Quat_Quat(q3, quatConj(q1)))
    assert np.sqrt(np.sum(np.square(mul_Quat_Quat(q3, quatConj(q1)) - q2))) < 1e-8

    dq1 = dpsi_dq(q1)

    # Test ECEF-to-LLA transformations
    xe0 = y[0:5, :]*100000
    ind1 = xe0 >= 0
    ind0 = xe0 < 0
    xe0[ind1] = np.clip(xe0[ind1], 10000, 1e10)
    xe0[ind0] = np.clip(xe0[ind0], -1e10, -10000)
    lla_deg = ecef2lla(xe0, 100)
    xe1 = lla2ecef(lla_deg)
    assert np.sqrt(np.sum(np.square(xe0 - xe1))) < 1e-4

    # Test NED-to-LLA transformations
    lla_ref = np.array([40.0, 117.0, 10.0])
    lla = np.array([[40.1, 117.1, 20.0], [40.2, 117.2, 0.0]])
    ned1 = lla2ned(lla_ref, lla)
    lla1 = ned2lla(lla_ref, ned1)
    assert np.sqrt(np.sum(np.square(lla - lla1))) < 1e-4

    # Test attitude in ENU to attitude in NED
    eul_e2b = np.radians(np.array([[90.0, 0.0, 180.0], [0.0, 45.0, -90.0]]))
    eul_n2b = enu2nedEuler(eul_e2b)
    un = eulerNed(eul_n2b)

    # Test rotation matrix from Euler angles
    R = eulerDCM(eul_n2b)
    R1 = eulerDCM(eul_n2b[0, :])
    R2 = eulerDCM(eul_n2b[1, :])
    assert np.sqrt(np.sum(np.square(R[0, :, :] - R1))) < 1e-8
    assert np.sqrt(np.sum(np.square(R[1, :, :] - R2))) < 1e-8

    # Test vector rotation using attitude as Euler angles
    u0 = y[0:2,:]
    v = vectorRotateInertialToBody(eul_n2b, u0)
    u = vectorRotateBodyToInertial(eul_n2b, v)
    assert np.sqrt(np.sum(np.square(u - u0))) < 1e-8

    u = y[0:2,:]
    eul = accellToEuler(u)
    a = unwrapAngle(np.radians(205.0))
    eul, bias = acc2AttAndBias(u)

    # Find the mean Quaternion
    mu = meanOfQuat(q)
    qarr = np.random.random((5000, 25, 4))
    qarr = normalize(qarr, axis=2)
    mu = meanOfQuatArray(qarr)
    print(mu)

    pass

# THIS FUNCTION SHALL BE DELETED
# qmat_matrix = np.array([[[1.0, 0, 0, 0],
#                          [0, -1.0, 0, 0],
#                          [0, 0, -1.0, 0],
#                          [0, 0, 0, -1.0]],
#                         [[0, 1.0, 0, 0],
#                          [1.0, 0, 0, 0],
#                          [0, 0, 0, 1.0],
#                          [0, 0, -1.0, 0]],
#                         [[0, 0, 1.0, 0],
#                          [0, 0, 0, -1.0],
#                          [1.0, 0, 0, 0],
#                          [0, 1.0, 0, 0]],
#                         [[0, 0, 0, 1.0],
#                          [0, 0, 1.0, 0],
#                          [0, -1.0, 0, 0],
#                          [1.0, 0, 0, 0]]])
# def qmult(q1, q2):
#     if q1.shape[0] == 1 and q2.shape[0] == 1:
#         dots = np.empty_like(q2)
#         for i in range(q2.shape[0]):
#             dots[i, :] = qmat_matrix.dot(q2.T).squeeze().dot(q1.T).T
#     elif q1.shape[0] == 1 and q2.shape[0] != 1:
#         dots = np.empty_like(q2)
#         for i in range(q2.shape[0]):
#             dots[i, :] = qmat_matrix.dot(q2[i, :].T).squeeze().dot(q1.T).T
#     elif q1.shape[0] != 1 and q2.shape[0] == 1:
#         dots = np.empty_like(q2)
#         for i in range(q2.shape[0]):
#             dots[i, :] = qmat_matrix.dot(q2.T).squeeze().dot(q1[i,:].T).T
#     elif q1.shape[0] == q2.shape[0]:
#         dots = np.empty_like(q2)
#         for i in range(q2.shape[0]):
#             dots[i, :] = qmat_matrix.dot(q2[i,:].T).squeeze().dot(q1[i, :].T).T
#     else:
#         raise Exception("Incompatible quaternion arrays -- cannot multiply")

#     # print qmat_matrix.dot(q2.T).squeeze()
#     # print np.tensordot(qmat_matrix.T, q2, axes=[0,1]).T.squeeze()
#     # dots = np.empty((2,4,4))
#     # for i in range(q1.shape[0]):
#     #      dots[i,:] = qmat_matrix.dot(q2[i,:].T)
#     # tensordots = np.tensordot(qmat_matrix.T, q2, axes=[0,1]).T
#     # dots = np.empty_like(q1)
#     # for i in range(q1.shape[0]):
#     #      dots[i,:] = qmat_matrix.dot(q2[i,:].T).squeeze().dot(q1[i,:].T).T
#     # tensordots = np.tensordot(q1, np.tensordot(qmat_matrix.T, q2, axes=[0,1]), axes=1).T
#     # print "dots = ", dots, "\ntensordots = ", tensordots
#     # print "diff = ", dots - tensordots

#     # return np.tensordot(q1, np.tensordot(q2.T, qmat_matrix.T, axes=[0,1]).T, axes=1)[0].T
#     return dots
