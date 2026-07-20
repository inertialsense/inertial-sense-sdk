/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @file data_sets_canbus.h
 * @brief CAN-bus payload structures for classic CAN (8-byte) and CAN FD (up to 64-byte) frames.
 *
 * Each struct here is the payload of exactly one CAN (or CAN FD) frame, addressed by a CAN ID
 * from @ref can_cid_t (classic CAN) or the corresponding FDCID_* macro (CAN FD); see data_sets.h
 * and can_config_t for the broadcast configuration that maps DIDs to CAN addresses. Classic-CAN payloads (is_can_*)
 * pack multiple scaled-integer fields into 8 bytes to fit the CAN 2.0 frame limit; CAN FD payloads
 * (is_canfd_*) carry the same information at native float/double precision since CAN FD supports
 * larger frames (valid CAN FD payload sizes are 0-8, 12, 16, 20, 24, 32, 48, or 64 bytes -- structs
 * that don't land on a valid boundary are padded by the driver).
 *
 * Unless otherwise noted: angles are in radians, velocities are in meters/second, and positions
 * given as ECEF are in meters. INS Euler/quaternion fields are body rotation with respect to NED
 * or ECEF as named in the type. Classic-CAN scaled-integer fields state their scale factor and
 * decimal-place precision in their inline comment.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef CAN_COMM_H_
#define CAN_COMM_H_
#ifdef __cplusplus
extern "C" {
#endif


PUSH_PACK_1

/** GMT week and time-of-week, classic-CAN native precision. */
typedef struct PACKED
{
    uint32_t        week;                                //!< Weeks since January 6th, 1980
    float           timeOfWeekMs;                        //!< Time of week (since Sunday morning) in milliseconds, GMT
} is_can_time;

/** INS/hardware status flags, copied from DID_SYS_PARAMS. */
typedef struct PACKED
{
    uint32_t        insStatus;                            //!< INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus
    uint32_t        hdwStatus;                            //!< Hardware status flags (eHdwStatusFlags). Copy of DID_SYS_PARAMS.hdwStatus
} is_can_ins_status;

/** Euler angles: roll, pitch, yaw with respect to NED, scaled by 10000 (4 decimal places precision) for 2-byte transport. */
typedef struct PACKED
{
    int16_t                 theta1;                                //!< Roll (radians, scaled by 10000)
    int16_t                 theta2;                                //!< Pitch (radians, scaled by 10000)
    int16_t                 theta3;                                //!< Yaw (radians, scaled by 10000)
    //int16_t                    reserved;
} is_can_ins_euler;

/** Quaternion body rotation with respect to NED, scaled by 10000 (4 decimal places precision) for 2-byte transport. */
typedef struct PACKED
{
    int16_t                 qn2b1;                                //!< Quaternion W component (scaled by 10000)
    int16_t                 qn2b2;                                //!< Quaternion X component (scaled by 10000)
    int16_t                 qn2b3;                                //!< Quaternion Y component (scaled by 10000)
    int16_t                 qn2b4;                                //!< Quaternion Z component (scaled by 10000)
} is_can_ins_quatn2b;

/** Quaternion body rotation with respect to ECEF, scaled by 10000 (4 decimal places precision) for 2-byte transport. */
typedef struct PACKED
{
    int16_t                 qe2b1;                                //!< Quaternion W component (scaled by 10000)
    int16_t                 qe2b2;                                //!< Quaternion X component (scaled by 10000)
    int16_t                 qe2b3;                                //!< Quaternion Y component (scaled by 10000)
    int16_t                 qe2b4;                                //!< Quaternion Z component (scaled by 10000)
} is_can_ins_quate2b;

/** Velocity in the body frame, scaled by 100 (2 decimal places precision, max absolute value 327.67 m/s) for 2-byte transport. Convert to NED velocity using vectorBodyToReference(uvw, theta, vel_ned). */
typedef struct PACKED
{
    int16_t                 uvw1;                                //!< Body-frame U (forward) velocity, m/s (scaled by 100)
    int16_t                 uvw2;                                //!< Body-frame V (right) velocity, m/s (scaled by 100)
    int16_t                 uvw3;                                //!< Body-frame W (down) velocity, m/s (scaled by 100)
} is_can_uvw;

/** Velocity in ECEF (earth-centered earth-fixed) frame, scaled by 100 (2 decimal places precision, max absolute value 327.67 m/s) for 2-byte transport. */
typedef struct PACKED
{
    int16_t                 ve1;                                //!< ECEF X velocity, m/s (scaled by 100)
    int16_t                 ve2;                                //!< ECEF Y velocity, m/s (scaled by 100)
    int16_t                 ve3;                                //!< ECEF Z velocity, m/s (scaled by 100)
} is_can_ve;

/** WGS84 latitude, native double precision. */
typedef struct PACKED
{
    double                  lat;                                //!< WGS84 latitude (degrees)
} is_can_ins_lat;

/** WGS84 longitude, native double precision. */
typedef struct PACKED
{
    double                  lon;                                //!< WGS84 longitude (degrees)
} is_can_ins_lon;

/** WGS84 height above ellipsoid plus GNSS fix status. */
typedef struct PACKED
{
    float                   alt;                                //!< WGS84 height above ellipsoid (meters)
    uint32_t                status;                             //!< (see eGnssStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags
} is_can_ins_alt;

/** North and East offset from a reference LLA. */
typedef struct PACKED
{
    float                   ned1;                                //!< North offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude (meters)
    float                   ned2;                                //!< East offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude (meters)
} is_can_north_east;

/** Down offset from a reference LLA, plus INS status. */
typedef struct PACKED
{
    float                   ned3;                                //!< Down offset from reference latitude, longitude, and altitude to current latitude, longitude, and altitude (meters)
    uint32_t                insStatus;                            //!< INS status flags (eInsStatusFlags). Copy of DID_SYS_PARAMS.insStatus
} is_can_down;

/** X position in ECEF (earth-centered earth-fixed) frame, native double precision. */
typedef struct PACKED
{
    double                  ecef1;                                //!< X position in ECEF frame (meters)
} is_can_ecef_x;

/** Y position in ECEF (earth-centered earth-fixed) frame, native double precision. */
typedef struct PACKED
{
    double                  ecef2;                                //!< Y position in ECEF frame (meters)
} is_can_ecef_y;

/** Z position in ECEF (earth-centered earth-fixed) frame, native double precision. */
typedef struct PACKED
{
    double                  ecef3;                                //!< Z position in ECEF frame (meters)
} is_can_ecef_z;

/** Height above Mean Sea Level, native float precision. */
typedef struct PACKED
{
    float                   msl;                                //!< Height above mean sea level (MSL) in meters
} is_can_msl;

/** Preintegrated IMU delta theta/velocity (X axis) and integral period, body/IMU frame, accelerometer 0. */
typedef struct PACKED
{
    int16_t                 theta0;                                    //!< Delta theta, X axis, radians (scaled by 1000, 3 decimal places precision)
    int16_t                 vel0;                                    //!< Delta velocity, X axis, m/s (scaled by 100, 2 decimal places precision)
    uint16_t                dt;                                        //!< Integral period in seconds for delta theta and delta velocity (scaled by 1000)
} is_can_preint_imu_px;

/** Preintegrated IMU delta theta/velocity (Y axis) and integral period, body/IMU frame, accelerometer 0. */
typedef struct PACKED
{
    int16_t                 theta1;                                    //!< Delta theta, Y axis, radians (scaled by 1000, 3 decimal places precision)
    int16_t                 vel1;                                    //!< Delta velocity, Y axis, m/s (scaled by 100, 2 decimal places precision)
    uint16_t                dt;                                        //!< Integral period in seconds for delta theta and delta velocity (scaled by 1000)
} is_can_preint_imu_qy;

/** Preintegrated IMU delta theta/velocity (Z axis) and integral period, body/IMU frame, accelerometer 0. */
typedef struct PACKED
{
    int16_t                 theta2;                                    //!< Delta theta, Z axis, radians (scaled by 1000, 3 decimal places precision)
    int16_t                 vel2;                                    //!< Delta velocity, Z axis, m/s (scaled by 100, 2 decimal places precision)
    uint16_t                dt;                                        //!< Integral period in seconds for delta theta and delta velocity (scaled by 1000)
} is_can_preint_imu_rz;

/** Dual-rate IMU delta theta/velocity (X axis) and status. */
typedef struct PACKED
{
    int16_t                 theta0;                                    //!< Delta theta, X axis, radians (scaled by 1000, 3 decimal places precision)
    int16_t                 vel0;                                    //!< Delta velocity, X axis, m/s (scaled by 100, 2 decimal places precision)
    uint32_t                status;                                    //!< IMU Status (eImuStatus)
} is_can_dual_imu_px;

/** Dual-rate IMU delta theta/velocity (Y axis) and status. */
typedef struct PACKED
{
    int16_t                 theta1;                                    //!< Delta theta, Y axis, radians (scaled by 1000, 3 decimal places precision)
    int16_t                 vel1;                                    //!< Delta velocity, Y axis, m/s (scaled by 100, 2 decimal places precision)
    uint32_t                status;                                    //!< IMU Status (eImuStatus)
} is_can_dual_imu_qy;

/** Dual-rate IMU delta theta/velocity (Z axis) and status. */
typedef struct PACKED
{
    int16_t                 theta2;                                    //!< Delta theta, Z axis, radians (scaled by 1000, 3 decimal places precision)
    int16_t                 vel2;                                    //!< Delta velocity, Z axis, m/s (scaled by 100, 2 decimal places precision)
    uint32_t                status;                                    //!< IMU Status (eImuStatus)
} is_can_dual_imu_rz;

/** GNSS position fix status and mean signal quality. */
typedef struct PACKED
{
    uint32_t                status;                                    //!< (see eGnssStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags
    uint32_t                cnoMean;                                //!< Average of all satellite carrier to noise ratios (signal strengths) that are non-zero, in dBHz
} is_can_gnss_pos_status;

/** RTK relative-positioning quality metrics. */
typedef struct PACKED
{
    uint8_t                 arRatio;                                //!< Ambiguity resolution ratio factor for validation
    uint8_t                 differentialAge;                        //!< Age of differential correction (seconds)
    float                   distanceToBase;                            //!< Distance to base station (meters)
    int16_t                 headingToBase;                            //!< Angle from north to vectorToBase in the local tangent plane, radians (scaled by 1000, 3 decimal places precision)
} is_can_gnss_rtk_rel;

/** INS roll and per-IMU roll rates. */
typedef struct PACKED
{
        int16_t             insRoll;                                //!< INS Euler roll, radians (scaled by 10000, 4 decimal places precision)
        int16_t             pImu1;                                    //!< IMU 1 roll rate, radians/second (scaled by 1000, 3 decimal places precision), from DID_IMU
        int16_t             pImu2;                                    //!< IMU 2 roll rate, radians/second (scaled by 1000, 3 decimal places precision), from DID_IMU
} is_can_roll_rollRate;

/** Union of all classic-CAN (8-byte) payload types; the active member is selected by the frame's CAN ID. */
typedef union PACKED
{
    is_can_time time;                          //!< GMT week/time-of-week
    is_can_ins_status insstatus;               //!< INS/hardware status flags
    is_can_ins_euler euler;                    //!< Euler angles (NED)
    is_can_ins_quatn2b quatn2b;                //!< Quaternion body rotation (NED)
    is_can_ins_quate2b quate2b;                //!< Quaternion body rotation (ECEF)
    is_can_uvw uvw;                            //!< Body-frame velocity
    is_can_ve ve;                              //!< ECEF velocity
    is_can_ins_lat lat;                        //!< WGS84 latitude
    is_can_ins_lon lon;                        //!< WGS84 longitude
    is_can_ins_alt alt;                        //!< WGS84 altitude + GNSS status
    is_can_north_east ne;                      //!< North/East offset from reference LLA
    is_can_down down;                          //!< Down offset from reference LLA + INS status
    is_can_ecef_x ecefx;                       //!< ECEF X position
    is_can_ecef_y ecefy;                       //!< ECEF Y position
    is_can_ecef_z ecefz;                       //!< ECEF Z position
    is_can_msl msl;                            //!< Height above mean sea level
    is_can_preint_imu_px pimupx;               //!< Preintegrated IMU, X axis
    is_can_preint_imu_qy pimuqy;                //!< Preintegrated IMU, Y axis
    is_can_preint_imu_rz pimurz;                //!< Preintegrated IMU, Z axis
    is_can_dual_imu_px dimupx;                  //!< Dual-rate IMU, X axis
    is_can_dual_imu_qy dimuqy;                  //!< Dual-rate IMU, Y axis
    is_can_dual_imu_rz dimurz;                  //!< Dual-rate IMU, Z axis
    is_can_gnss_pos_status gnsspos;             //!< GNSS position fix status
    is_can_gnss_rtk_rel rtkrel;                 //!< RTK relative-positioning quality metrics
    is_can_roll_rollRate rollrollrate;          //!< INS roll + per-IMU roll rates
} is_can_payload;


POP_PACK


// ============================================================================
// CAN FD payloads — native float/double precision, one frame per DID
// Valid CAN FD payload sizes: 0-8, 12, 16, 20, 24, 32, 48, 64 bytes.
// Structs that don't land on a valid boundary are padded by the driver.
// ============================================================================
PUSH_PACK_1

/** FDCID_INS_1: key INS-1 fields in native float/double precision, 64 bytes (exact FD DLC). */
typedef struct PACKED
{
    uint32_t    week;           //!< Weeks since January 6th, 1980
    double      timeOfWeek;     //!< Time of week (seconds)
    uint32_t    insStatus;      //!< INS status flags (eInsStatusFlags)
    uint32_t    hdwStatus;      //!< Hardware status flags (eHdwStatusFlags)
    float       theta[3];       //!< Euler angles roll/pitch/yaw w.r.t. NED (radians)
    float       uvw[3];         //!< Body-frame velocity U/V/W (m/s)
    double      lat;            //!< WGS84 latitude (degrees)
    double      lon;            //!< WGS84 longitude (degrees)
    float       alt;            //!< WGS84 height above ellipsoid (meters)
} is_canfd_ins1;                //!< 64 bytes total

/** FDCID_INS_2: native-precision NED quaternion. */
typedef struct PACKED
{
    float       qn2b[4];        //!< Quaternion body rotation w.r.t. NED: W, X, Y, Z
} is_canfd_ins2;                //!< 16 bytes total

/** FDCID_INS_3: MSL altitude. */
typedef struct PACKED
{
    float       msl;            //!< Height above mean sea level (meters)
} is_canfd_ins3;                //!< 4 bytes total

/** FDCID_INS_4: ECEF quaternion, ECEF velocity, ECEF position (52 bytes -> padded to a 64-byte FD frame). */
typedef struct PACKED
{
    float       qe2b[4];        //!< Quaternion body rotation w.r.t. ECEF: W, X, Y, Z
    float       ve[3];          //!< ECEF velocity X/Y/Z (m/s)
    double      ecef[3];        //!< ECEF position X/Y/Z (meters)
} is_canfd_ins4;                //!< 52 bytes total

/** FDCID_PIMU: preintegrated IMU in native float (32 bytes, exact FD DLC). */
typedef struct PACKED
{
    float       theta[3];       //!< Delta theta X/Y/Z, body/IMU frame (radians)
    float       vel[3];         //!< Delta velocity X/Y/Z, body/IMU frame (m/s)
    float       dt;             //!< Integral period for delta theta and delta velocity (seconds)
    uint32_t    status;         //!< IMU Status (eImuStatus)
} is_canfd_pimu;                //!< 32 bytes total

/** FDCID_IMU: IMU angular rate, acceleration, status (28 bytes -> padded to a 32-byte FD frame). */
typedef struct PACKED
{
    float       pqr[3];         //!< Angular rate P/Q/R, body/IMU frame (radians/second)
    float       acc[3];         //!< Acceleration X/Y/Z, body/IMU frame (m/s^2)
    uint32_t    status;         //!< IMU Status (eImuStatus)
} is_canfd_imu;                 //!< 28 bytes total

/** FDCID_GNSS1_POS / FDCID_GNSS2_POS: GNSS fix status and mean CNO. */
typedef struct PACKED
{
    uint32_t    status;         //!< (see eGnssStatus) GPS status: [0x000000xx] number of satellites used, [0x0000xx00] fix type, [0x00xx0000] status flags
    float       cnoMean;        //!< Average of all non-zero satellite carrier-to-noise ratios (dBHz); native float (gnss_pos_t.cnoMean is float)
} is_canfd_gnss_pos;            //!< 8 bytes total

/** FDCID_GNSS1_RTK_POS_REL / FDCID_GNSS2_RTK_CMP_REL: RTK relative-positioning quality metrics, native float (16 bytes). */
typedef struct PACKED
{
    float       arRatio;            //!< Ambiguity resolution ratio factor for validation
    float       differentialAge;    //!< Age of differential correction (seconds)
    float       distanceToBase;     //!< Distance to base station (meters)
    float       headingToBase;      //!< Angle from north to vectorToBase in the local tangent plane (radians)
} is_canfd_gnss_rtk_rel;        //!< 16 bytes total

/** Union of all CAN FD payload types; the active member is selected by the frame's CAN ID. */
typedef union PACKED
{
    is_canfd_ins1           ins1;       //!< INS-1 fields
    is_canfd_ins2           ins2;       //!< NED quaternion
    is_canfd_ins3           ins3;       //!< MSL altitude
    is_canfd_ins4           ins4;       //!< ECEF quaternion/velocity/position
    is_canfd_pimu           pimu;       //!< Preintegrated IMU
    is_canfd_imu            imu;        //!< IMU angular rate/acceleration/status
    is_canfd_gnss_pos       gnsspos;    //!< GNSS position fix status
    is_canfd_gnss_rtk_rel   rtkrel;     //!< RTK relative-positioning quality metrics
} is_canfd_payload;

POP_PACK

#ifdef __cplusplus
}
#endif

#endif // CAN_COMM_H_
