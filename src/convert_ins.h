/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file convert_ins.h
 * @brief Conversions between the four INS output data sets (ins_1_t, ins_2_t, ins_3_t, ins_4_t).
 *
 * The INS DIDs all carry the same navigation solution (time, status, velocity, position) but differ
 * in attitude representation (Euler angles vs. quaternion) and reference frame (NED-relative vs.
 * ECEF). These helpers convert between them so callers only need to work with one representation.
 * All rotations are in radians; velocities in meters/second; ins_1_t/ins_2_t/ins_3_t positions are
 * WGS84 latitude/longitude (degrees) + ellipsoid altitude (meters) with an optional NED offset from
 * a caller-supplied reference LLA, while ins_4_t uses ECEF position (meters) directly.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef IS_SDK_CONVERT_INS_H_
#define IS_SDK_CONVERT_INS_H_

#include "data_sets.h"

/**
 * @brief Convert ins_1_t (Euler/NED) to ins_2_t (quaternion/NED).
 * @param ins1   Source INS-1 solution (Euler angles w.r.t. NED, LLA position).
 * @param result Output INS-2 solution: quaternion body rotation w.r.t. NED (qn2b), same LLA position.
 */
void convertIns1ToIns2(ins_1_t* ins1, ins_2_t* result);

/**
 * @brief Convert ins_2_t (quaternion/NED) to ins_1_t (Euler/NED).
 * @param ins2   Source INS-2 solution (quaternion body rotation w.r.t. NED, LLA position).
 * @param result Output INS-1 solution: Euler angles w.r.t. NED, same LLA position, and NED offset
 *               from refLla (if provided).
 * @param refLla Optional reference WGS84 latitude/longitude/altitude (degrees, degrees, meters)
 *               used to compute result->ned. If NULL, result->ned is zeroed.
 */
void convertIns2ToIns1(ins_2_t *ins2, ins_1_t *result, double *refLla=NULL);

/**
 * @brief Convert ins_3_t (quaternion/NED, LLA position) to ins_1_t (Euler/NED, LLA position).
 * @param ins3   Source INS-3 solution (quaternion body rotation w.r.t. NED, LLA position).
 * @param result Output INS-1 solution: Euler angles w.r.t. NED, same LLA position, and NED offset
 *               from refLla (if provided).
 * @param refLla Optional reference WGS84 latitude/longitude/altitude (degrees, degrees, meters)
 *               used to compute result->ned. If NULL, result->ned is zeroed.
 */
void convertIns3ToIns1(ins_3_t *ins3, ins_1_t *result, double *refLla=NULL);

/**
 * @brief Convert ins_4_t (quaternion/ECEF) to ins_1_t (Euler/NED, LLA position).
 * @param ins4   Source INS-4 solution (quaternion body rotation w.r.t. ECEF, ECEF position/velocity).
 * @param result Output INS-1 solution: body-frame velocity (derived by rotating ECEF velocity into
 *               the body frame), Euler angles w.r.t. NED, WGS84 LLA position (converted from ECEF),
 *               and NED offset from refLla (if provided).
 * @param refLla Optional reference WGS84 latitude/longitude/altitude (degrees, degrees, meters)
 *               used to compute result->ned. If NULL, result->ned is zeroed.
 */
void convertIns4ToIns1(ins_4_t *ins4, ins_1_t *result, double *refLla=NULL);

#endif //IS_SDK_CONVERT_INS_H_
